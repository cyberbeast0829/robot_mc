#!/usr/bin/env python3
"""
motorcontrol endpoint 上位机（odrivetool 风格）。

设备会先返回一份 JSON 描述符（名字、ID、类型、读写权限），本工具据此把名字
映射到 endpoint ID，因此不需要和固件版本手工对齐。

三种传输，命令和参数名完全相同（**出货板只有 CAN**，日常用 `--can`）：

  --can          走 CAN 的 CYBERBEAST 协议（PARAM_READ/PARAM_WRITE + 描述符流，
                 设备侧见 Core/Src/can_cyberbeast.c）。要求 config.can_prot = 1
                 （CYBERBEAST，出厂默认）。速率：Classic 下 --bitrate 等于
                 config.can_baud（出厂 1 Mbit/s）；FD 时固件把仲裁域固定为
                 1 Mbit/s，所以 --bitrate 保持 1000000、加 --fd，并把
                 config.can_baud（2M/4M/5M）填到 --data-bitrate。
                 --can-id 是设备 config.can_id（出厂 1）。后端用 --interface 选：
                   slcan      CANable/CandleLight 串口（只多要 pyserial，含 FD）
                   pcan       PCAN-USB（需要 PCAN 驱动）
                   socketcan  Linux SocketCAN（接口要先用 ip link 配好）
                   其它       gs_usb / kvaser / vector / ixxat …（python-can 支持即可）
                 ⚠ 写 ACK 不带成败位，所以 set/call 之后都会读回/校验；
                 也就无法区分“被限幅”和“被拒绝”（用 get 确认）。
  --port COM7    USB CDC（虚拟串口，开发/台架板子），外层是 0xAA + CRC 字节流，
                 和 ASCII 菜单共用数据流；只需要 pyserial。
  --usb          USB vendor interface（interface 2）的 bulk 端点（开发/台架板子），
                 一个 USB 包就是一帧，没有 0xAA/CRC 外层；
                 需要 pyusb，Windows 上还要用 Zadig 给 interface 2 装 WinUSB
                 （CDC 那部分保持 usbser，不需要动）。

依赖:
    pip install pyserial              # --can --interface slcan / --port
    pip install python-can            # --can --interface pcan|socketcan|…
    pip install pyusb libusb-package  # --usb（libusb-package 在 Windows 上自带 DLL）

示例:
    python tools/mc_ep.py --can tree                                      # CANable
    python tools/mc_ep.py --can --interface pcan --channel PCAN_USBBUS1 tree
    python tools/mc_ep.py --can --interface socketcan --channel can0 tree  # 先 ip link 配好
    python tools/mc_ep.py --can --interface slcan --channel COM3 --fd --data-bitrate 5000000 tree
    python tools/mc_ep.py --can --interface slcan --channel COM3 probe     # 只监听总线
    python tools/mc_ep.py --can get config.i_max
    python tools/mc_ep.py --can set config.i_max 3.5
    python tools/mc_ep.py --can call action.save
    python tools/mc_ep.py --can monitor telemetry.i_q telemetry.position --rate 100

    # 开发/台架板子的 USB 入口（命令和参数名完全一样）
    python tools/mc_ep.py --port COM7 tree
    python tools/mc_ep.py --usb tree
    python tools/mc_ep.py --usb call action.save --read   # 用“读”触发（ODrive 语义）

协议（见 Core/Inc/endpoint.h）:
    CAN   : 29-bit 扩展帧，ID = Priority(3)|MsgType(8)|Dest(8)|Source(8)|Seq(2)；
            PARAM_READ(0x20) / PARAM_WRITE(0x21) / JSON_DESC_READ(0x24) /
            JSON_DESC_DATA(0x25)，没有 status 字节
    CDC   : AA | LEN | CRC8 | PAYLOAD | CRC16_HI CRC16_LO
    USB   : 一个 bulk 包 = 一个 PAYLOAD（<= 63 字节，短包）
    请求 PAYLOAD: seq(2) | id|flags(2) | rxlen(2) | tx_data | trailer(2)
    响应 PAYLOAD: seq|0x8000(2) | status(1) | rx_data
    flags: bit15 = 需要应答，bit14 = 写/执行（不置位是读）
    动作端点（action.*）例外：读和写都会执行——ODrive 的 endpoint 协议没有
    读写位，function 端点被“读”也会执行，这里跟随它以兼容 JointSDK
    （CAN 侧就是 PARAM_READ，`--read` 对它没有区别）。
"""

import argparse
import json
import struct
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

try:
    import can
except ImportError:  # pragma: no cover
    can = None

try:
    import usb.core
    import usb.util
    try:
        import usb.backend.libusb1
    except ImportError:  # pragma: no cover
        pass
except ImportError:  # pragma: no cover
    usb = None


def _usb_backend():
    """优先用 libusb-package 自带的 DLL（Windows 上不用手工配 PATH）。"""
    try:
        import libusb_package
        import usb.backend.libusb1
        return usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
    except Exception:
        return None

DEFAULT_VID = 0x1209
DEFAULT_PID = 0x4342
DEFAULT_USB_INTERFACE = 2
DEFAULT_EP_OUT = 0x01   # EP1 OUT, fibre RX
DEFAULT_EP_IN = 0x83    # EP3 IN,  fibre TX
USB_READ_SIZE = 64

FRAME_PREFIX = 0xAA
PROTOCOL_VERSION = 2
REPLY_FLAG = 0x8000
WRITE_FLAG = 0x4000

CRC8_POLY = 0x37
CRC8_INIT = 0x42
CRC16_POLY = 0x3D65
CRC16_INIT = 0x1337

STATUS_NAMES = {
    0: "OK",
    1: "未知 endpoint ID",
    2: "长度错误",
    3: "数值非法（NaN）",
    4: "只读",
    5: "只写",
    6: "当前状态不允许",
    7: "接口版本不匹配（描述符已更新，请重新连接）",
}

# JSON 类型 -> (struct 格式, 字节数)
TYPE_CODES = {
    "bool": ("<B", 1),
    "int8": ("<b", 1),
    "uint8": ("<B", 1),
    "int16": ("<h", 2),
    "uint16": ("<H", 2),
    "int32": ("<i", 4),
    "uint32": ("<I", 4),
    "uint64": ("<Q", 8),
    "float": ("<f", 4),
}

# ---- CYBERBEAST（CAN）----------------------------------------------------

CB_PRI_CONFIG = 4
CB_MSG_PARAM_READ = 0x20
CB_MSG_PARAM_WRITE = 0x21
CB_MSG_JSON_DESC_READ = 0x24
CB_MSG_JSON_DESC_DATA = 0x25
CB_READ_MORE = 0x80        # PARAM_READ 响应的 bit7：这个值还有后续字节
CB_WRITE_MORE = 0x80       # PARAM_WRITE 请求的 bit7：分段写还没完

CAN_MSG_NAMES = {
    0x00: "MIT", 0x01: "CSP", 0x02: "CSV", 0x03: "TORQUE", 0x04: "CURRENT",
    0x20: "PARAM_READ", 0x21: "PARAM_WRITE", 0x22: "CONFIG_SAVE",
    0x23: "CONFIG_RESET", 0x24: "DESC_READ", 0x25: "DESC_DATA",
    0x40: "QUERY_STATUS", 0x41: "QUERY_POS_VEL", 0x42: "QUERY_CURRENT",
    0x43: "QUERY_TEMP", 0x44: "QUERY_BUS", 0x45: "QUERY_ERROR",
    0x46: "QUERY_DEV", 0x47: "QUERY_POWER", 0x48: "HEARTBEAT",
    0x49: "STATUS_FEEDBACK", 0x60: "SET_NODE_ID", 0x61: "SET_ZERO",
    0x62: "START", 0x63: "STOP", 0x64: "RESET", 0x65: "CLEAR_ERRORS",
    0x80: "MIT_BCAST", 0xC0: "ESTOP", 0xC1: "FAULT_ALERT",
}

# CANable/CandleLight 的 slcan 命令码（固件只认这几档）
SLCAN_BITRATES = {
    10000: "S0", 20000: "S1", 50000: "S2", 100000: "S3", 125000: "S4",
    250000: "S5", 500000: "S6", 750000: "S7", 1000000: "S8",
}
SLCAN_DATA_BITRATES = {2000000: "Y2", 5000000: "Y5"}
SLCAN_CODE2LEN = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
                  9: 12, 0xA: 16, 0xB: 20, 0xC: 24, 0xD: 32, 0xE: 48, 0xF: 64}
SLCAN_LEN2CODE = {v: k for k, v in SLCAN_CODE2LEN.items()}

DEFAULT_CAN_BITRATE = 1000000        # = config.can_baud 出厂值（Classic）
CAN_FD_NOMINAL_BAUD = 1000000        # = can.h 的 CAN_FD_NOMINAL_BAUD：FD 仲裁域写死 1 Mbit/s
DEFAULT_CAN_DATA_BITRATE = 5000000   # = config.can_baud > 1M 时的常用值
DEFAULT_CAN_NODE_ID = 1              # = config.can_id 出厂值
DEFAULT_CAN_MASTER_ID = 0xFE


class EpError(Exception):
    pass


class _NoDescriptorReply(EpError):
    """设备对描述符请求一帧都没回（重试没有意义，直接报配置检查清单）。"""


def crc8(data, remainder=CRC8_INIT):
    for value in data:
        remainder ^= value
        for _ in range(8):
            if remainder & 0x80:
                remainder = ((remainder << 1) ^ CRC8_POLY) & 0xFF
            else:
                remainder = (remainder << 1) & 0xFF
    return remainder


def crc16(data, remainder=CRC16_INIT):
    for value in data:
        remainder ^= value << 8
        for _ in range(8):
            if remainder & 0x8000:
                remainder = ((remainder << 1) ^ CRC16_POLY) & 0xFFFF
            else:
                remainder = (remainder << 1) & 0xFFFF
    return remainder


def build_frame(payload):
    if len(payload) > 64:
        raise EpError("请求太长（%d 字节）" % len(payload))
    header = bytes([FRAME_PREFIX, len(payload)])
    crc = crc16(payload)
    return header + bytes([crc8(header)]) + payload + bytes([crc >> 8, crc & 0xFF])


class SerialTransport:
    """CDC 上的 0xAA/CRC 帧（和 ASCII 菜单共用数据流）。"""

    def __init__(self, port, baud=115200, timeout=1.0):
        if serial is None:
            raise EpError("需要 pyserial：pip install pyserial")
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)

    def close(self):
        self.ser.close()

    def reset_input(self):
        self.ser.reset_input_buffer()

    def write(self, payload):
        self.ser.write(build_frame(payload))

    def read_packet(self):
        while True:
            b = self.ser.read(1)
            if not b:
                raise EpError("等待响应超时")
            if b[0] != FRAME_PREFIX:
                continue
            length = self.ser.read(1)
            if length == b"":
                raise EpError("等待响应超时")
            length = length[0]
            if length == 0 or length > 64:
                continue
            header = bytes([FRAME_PREFIX, length])
            crc = self.ser.read(1)
            if crc == b"":
                raise EpError("等待响应超时")
            if crc[0] != crc8(header):
                continue
            payload = self.ser.read(length)
            if len(payload) != length:
                raise EpError("响应 payload 不完整")
            tail = self.ser.read(2)
            if len(tail) != 2:
                raise EpError("响应 CRC 不完整")
            if crc16(payload) != ((tail[0] << 8) | tail[1]):
                continue  # 坏帧，重新同步
            return payload


class UsbTransport:
    """vendor interface（interface 2）的 bulk 端点，一个包就是一帧。

    响应长度上限 63 字节（固件保证短包），所以一次 64 字节 bulk 读就能收全。
    Windows 需要先用 Zadig 把 interface 2 绑到 WinUSB；CDC 那部分仍是 usbser。
    """

    def __init__(self, vid=DEFAULT_VID, pid=DEFAULT_PID, interface=DEFAULT_USB_INTERFACE,
                 ep_out=DEFAULT_EP_OUT, ep_in=DEFAULT_EP_IN, timeout=1.0):
        if usb is None:
            raise EpError("需要 pyusb：pip install pyusb libusb-package")
        try:
            dev = usb.core.find(idVendor=vid, idProduct=pid, backend=_usb_backend())
        except (usb.core.USBError, NotImplementedError) as e:
            raise EpError("pyusb 没有可用的 libusb 后端（Windows: pip install libusb-package）：%s" % e)
        if dev is None:
            raise EpError("找不到 USB 设备 %04x:%04x（检查 USB 连接、VID/PID 和 WinUSB 驱动）"
                          % (vid, pid))
        self.dev = dev
        self.interface = interface
        self.ep_out = ep_out
        self.ep_in = ep_in
        self.timeout = timeout
        try:
            dev.get_active_configuration()
        except (usb.core.USBError, NotImplementedError):
            # 设备还没被配置（Windows 的 usbccgp 通常已经配好，不要重设，
            # 免得打断 CDC 那一路）
            try:
                dev.set_configuration()
            except (usb.core.USBError, NotImplementedError) as e:
                raise EpError(
                    "无法打开 USB 设备：%s\n"
                    "  - Windows: 用 Zadig 只给 interface %d 装 WinUSB（CDC 保持 usbser）；\n"
                    "  - 旧固件只有 CDC，没有 vendor interface。" % (e, interface))
        # Linux/macOS 上内核驱动需要先 detach；Windows 上由 WinUSB 绑定，忽略异常
        try:
            if dev.is_kernel_driver_active(interface):
                dev.detach_kernel_driver(interface)
        except (NotImplementedError, usb.core.USBError):
            pass
        try:
            usb.util.claim_interface(dev, interface)
        except (usb.core.USBError, NotImplementedError) as e:
            raise EpError(
                "无法占用 interface %d：%s\n"
                "  - 确认固件支持复合设备（旧固件只有 CDC，没有 interface 2）；\n"
                "  - Windows 上用 Zadig 只给 interface 2 装 WinUSB。" % (interface, e))

    def close(self):
        try:
            usb.util.release_interface(self.dev, self.interface)
        except Exception:
            pass
        try:
            usb.util.dispose_resources(self.dev)
        except Exception:
            pass

    def reset_input(self):
        # 只保留请求/应答语义：不主动读，避免把上一条迟到的响应当成这次的
        pass

    def write(self, payload):
        try:
            self.dev.write(self.ep_out, payload, int(self.timeout * 1000))
        except usb.core.USBError as e:
            raise EpError("USB 写失败：%s" % e)

    def read_packet(self):
        try:
            return bytes(self.dev.read(self.ep_in, USB_READ_SIZE, int(self.timeout * 1000)))
        except usb.core.USBError as e:
            if "timeout" in str(e).lower():
                raise EpError("等待响应超时")
            raise EpError("USB 读失败：%s" % e)


class FibreLink:
    """fibre 风格 payload 层（CDC 的 0xAA 帧 / vendor bulk 包）。

    请求 PAYLOAD: seq(2) | id|flags(2) | rxlen(2) | tx_data | 描述符 CRC16(2)
    响应 PAYLOAD: seq|0x8000(2) | status(1) | rx_data
    """

    def __init__(self, io):
        self.io = io
        self.seq = 0
        self.crc = 0        # 描述符 CRC16，同时是接口版本（请求里的 trailer）

    def close(self):
        self.io.close()

    def _transact(self, endpoint_id, tx_data, rx_len, trailer, write=False):
        self.seq = (self.seq + 1) & 0x7FFF
        flags = REPLY_FLAG | (WRITE_FLAG if write else 0)
        payload = struct.pack("<HHH", self.seq, endpoint_id | flags, rx_len)
        payload += tx_data
        payload += struct.pack("<H", trailer)

        self.io.reset_input()
        self.io.write(payload)

        # 上一次超时可能留下一个迟到的响应：跳过 seq 不符的包（最多 3 个）
        resp = b""
        for _ in range(3):
            resp = self.io.read_packet()
            if len(resp) < 3:
                raise EpError("响应太短")
            rseq = struct.unpack("<H", resp[:2])[0]
            if (rseq & 0x7FFF) == self.seq:
                break
        else:
            raise EpError("响应 seq 不匹配（期望 %d）" % self.seq)
        status = resp[2]
        if status != 0:
            raise EpError("status %d: %s" % (status, STATUS_NAMES.get(status, "未知错误")))
        return resp[3:]

    # -- 三种传输共用的四个动作 -------------------------------------------

    def fetch_descriptor(self):
        info = self._transact(0, struct.pack("<I", 0xFFFFFFFF), 4, PROTOCOL_VERSION)
        if len(info) != 4:
            raise EpError("设备没有返回描述符版本信息")
        json_crc, total_len = struct.unpack("<HH", info)

        raw = bytearray()
        while len(raw) < total_len:
            want = min(60, total_len - len(raw))
            chunk = self._transact(0, struct.pack("<I", len(raw)), want, PROTOCOL_VERSION)
            if not chunk:
                break
            raw += chunk

        if len(raw) != total_len:
            raise EpError("描述符下载不完整（%d/%d 字节）" % (len(raw), total_len))
        self.crc = json_crc
        return bytes(raw)

    def read_value(self, ep_id, size):
        return self._transact(ep_id, b"", size, self.crc)

    def write_value(self, ep_id, data, read_back=False):
        # 固件的写应答直接回显限幅后的值，不需要（也不该）再读一次
        return self._transact(ep_id, data, len(data), self.crc, write=True)

    def call_action(self, ep_id, via_read=False):
        # ODrive 的 endpoint 协议没有读写位，function 端点被“读”也会执行；
        # 固件两种意图都接受，via_read 用来验证这条兼容路径。
        self._transact(ep_id, b"", 0, self.crc, write=not via_read)


# ---- CAN 包层 ------------------------------------------------------------


def cb_id(pri, msgtype, dest, src, seq=0):
    return ((pri & 7) << 26) | ((msgtype & 0xFF) << 18) | ((dest & 0xFF) << 10) \
        | ((src & 0xFF) << 2) | (seq & 3)


def cb_id_fields(value):
    """-> (priority, msgtype, dest, source, seq)"""
    return ((value >> 26) & 7, (value >> 18) & 0xFF, (value >> 10) & 0xFF,
            (value >> 2) & 0xFF, value & 3)


class SlcanBus:
    """CANable/CandleLight 的 slcan 串口协议（Classic + FD）。

    python-can 4.6 的 slcan 后端没有暴露数据域速率（FD 只能传寄存器级的
    ``timing=BitTimingFd``），所以这里直接发 S/Y/M/A/O 命令，和
    tools/cb_bench.py、tools/slcan_cb.py 的做法一致。
    """

    def __init__(self, channel, bitrate=DEFAULT_CAN_BITRATE, fd=False,
                 data_bitrate=DEFAULT_CAN_DATA_BITRATE, tty_baudrate=115200):
        if serial is None:
            raise EpError("slcan 需要 pyserial：pip install pyserial")
        if bitrate not in SLCAN_BITRATES:
            raise EpError(
                "slcan 固件只支持这几档仲裁域速率：%s（要的是 %d）。\n"
                "  - 改设备的 config.can_baud，或\n"
                "  - 换 --interface pcan / socketcan（它们支持任意速率）。"
                % ("/".join(str(k) for k in sorted(SLCAN_BITRATES)), bitrate))
        if fd and data_bitrate not in SLCAN_DATA_BITRATES:
            raise EpError("slcan(CANable) 的 FD 数据域只支持 2M/5M（要的是 %d）"
                          % data_bitrate)
        self.channel = channel
        self.fd = fd
        self.buf = b""
        try:
            self.ser = serial.Serial(channel, tty_baudrate, timeout=0.05)
        except Exception as e:
            raise EpError("打不开串口 %s：%s" % (channel, e))
        self.ser.reset_input_buffer()
        try:
            self._cmd("C")
            self._cmd(SLCAN_BITRATES[bitrate])
            if fd:
                self._cmd(SLCAN_DATA_BITRATES[data_bitrate])
            self._cmd("M0")
            self._cmd("A1")
            self._cmd("O", wait=0.15)
        except EpError:
            self.ser.close()
            raise
        self.ser.reset_input_buffer()

    def describe(self):
        return "slcan %s" % self.channel

    def _cmd(self, text, wait=0.05):
        self.ser.write((text + "\r").encode())
        self.ser.flush()
        time.sleep(wait)
        reply = self.ser.read(self.ser.in_waiting or 0)
        if b"\a" in reply:      # BEL = 适配器拒绝了这条命令
            raise EpError("slcan 适配器拒绝命令 %r（检查固件是否支持该模式）" % text)
        return reply

    def send(self, can_id, data):
        can_id &= 0x1FFFFFFF
        if self.fd:
            code = SLCAN_LEN2CODE.get(len(data))
            if code is None:
                raise EpError("FD 帧长 %d 不能编码" % len(data))
            # >8 字节开 BRS（和固件 can_send_frame 一致）
            line = "%s%08X%X%s" % ("B" if len(data) > 8 else "D", can_id, code,
                                   data.hex().upper())
        else:
            if len(data) > 8:
                raise EpError("Classic 帧不能超过 8 字节")
            line = "T%08X%X%s" % (can_id, len(data), data.hex().upper())
        self.ser.write((line + "\r").encode())
        self.ser.flush()

    @staticmethod
    def _parse(line):
        line = line.strip().strip("\x07")
        if not line:
            return None
        kind = line[0]
        try:
            if kind == "T":
                can_id = int(line[1:9], 16)
                dlc = int(line[9], 16)
                return can_id, bytes.fromhex(line[10:])[:dlc]
            if kind == "t":
                return None         # 旧 11-bit 协议：CYBERBEAST 不用
            if kind in "DdBb":
                can_id = int(line[1:9], 16)
                size = SLCAN_CODE2LEN.get(int(line[9], 16))
                if size is None:
                    return None
                return can_id, bytes.fromhex(line[10:])[:size]
        except (ValueError, IndexError):
            return None
        return None             # 状态/错误行（适配器上报的 bus-off 等）忽略

    def _pop_line(self):
        idx = self.buf.find(b"\r")
        if idx < 0:
            return None
        line = self.buf[:idx]
        self.buf = self.buf[idx + 1:]
        return line.decode(errors="replace")

    def recv(self, timeout=0.0):
        """收一帧 (can_id, data)；timeout=0 时只冲掉已经到手的字节。"""
        end = time.time() + max(timeout, 0.0)
        while True:
            line = self._pop_line()
            if line is not None:
                frame = self._parse(line)
                if frame is not None:
                    return frame
                continue
            pending = self.ser.in_waiting
            if pending:
                self.buf += self.ser.read(pending)
                continue
            if time.time() >= end:
                return None
            time.sleep(0.001)

    def close(self):
        try:
            self._cmd("C")
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


class PyCanBus:
    """python-can 后端（pcan / socketcan / gs_usb / kvaser / vector …）。"""

    def __init__(self, interface, channel, bitrate=DEFAULT_CAN_BITRATE, fd=False,
                 data_bitrate=DEFAULT_CAN_DATA_BITRATE):
        if can is None:
            raise EpError("这个 CAN 后端需要 python-can：pip install python-can")
        self.interface = interface
        self.channel = channel
        self.fd = fd
        kwargs = {"interface": interface, "channel": channel}
        if interface == "socketcan":
            # SocketCAN 的速率由 `ip link set can0 up type can bitrate …` 配好，
            # 这里只决定是否允许 FD 帧
            if fd:
                kwargs["fd"] = True
        elif interface == "pcan":
            # PCAN 的 FD 只认寄存器级的 timings（没有 data_bitrate 参数），
            # 这里按 80 MHz 时钟、80%/75% 采样点算出来交给它。
            if fd:
                try:
                    kwargs["timing"] = can.BitTimingFd.from_sample_point(
                        f_clock=80_000_000, nom_bitrate=bitrate, nom_sample_point=80.0,
                        data_bitrate=data_bitrate, data_sample_point=75.0)
                except Exception as e:
                    raise EpError("PCAN FD 时基算不出来（仲裁域 %d / 数据域 %d）：%s"
                                  % (bitrate, data_bitrate, e))
            else:
                kwargs["bitrate"] = bitrate
        else:
            kwargs["bitrate"] = bitrate
            if fd:
                kwargs.update(fd=True, data_bitrate=data_bitrate)
        try:
            self.bus = can.Bus(**kwargs)
        except Exception as e:
            raise EpError("打不开 CAN 后端 %s/%s：%s" % (interface, channel, e))

    def describe(self):
        return "%s %s %s" % (self.interface, self.channel,
                             "FD" if self.fd else "Classic")

    def send(self, can_id, data):
        msg = can.Message(arbitration_id=can_id, is_extended_id=True,
                          is_fd=self.fd,
                          bitrate_switch=bool(self.fd and len(data) > 8),
                          data=data)
        try:
            self.bus.send(msg, timeout=0.5)
        except Exception as e:
            raise EpError("CAN 发送失败：%s" % e)

    def recv(self, timeout=0.0):
        try:
            msg = self.bus.recv(timeout if timeout > 0 else 0)
        except Exception as e:
            raise EpError("CAN 接收失败：%s" % e)
        if msg is None:
            return None
        return msg.arbitration_id, bytes(msg.data)

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass


def _find_canable_port():
    """按 VID:PID（16D0:117E = CANable/CandleLight）找 slcan 串口。"""
    try:
        import serial.tools.list_ports as list_ports
    except ImportError:
        return None
    for port in list_ports.comports():
        text = " ".join(filter(None, [port.description, port.product,
                                      port.manufacturer, port.hwid])).lower()
        if "16d0:117e" in text or "canable" in text or "candle" in text:
            return port.device
    return None


def guess_can_channel(interface):
    if interface == "pcan":
        return "PCAN_USBBUS1"
    if interface == "socketcan":
        return "can0"
    if interface == "slcan":
        port = _find_canable_port()
        if port:
            return port
        raise EpError("没找到 CANable 串口，请用 --channel COMx 指定")
    raise EpError("--interface %s 需要 --channel" % interface)


class CanLink:
    """CYBERBEAST（29-bit 扩展帧）上的 endpoint 访问。

    设备侧见 Core/Src/can_cyberbeast.c：
      PARAM_READ(0x20)   读；响应 flags 的 bit7 = 还有后续字节
                         （Classic 每次最多 4 字节，>4 字节的值要按 offset 分段读）
      PARAM_WRITE(0x21)  写；Classic 下 >4 字节走两帧分段写（bit7 = MORE）
      JSON_DESC_READ(0x24)/JSON_DESC_DATA(0x25)  描述符分片流
    ⚠ ACK **不含成败位**（只读/状态不符/被限幅都照样 ACK），所以写完一律读回。
    ⚠ 固件的“主站 ID”是从收到的帧里学的，同一条总线上只能有一个主站在发命令，
      否则应答会被发到另一个主站去。
    """

    def __init__(self, interface="slcan", channel=None, bitrate=DEFAULT_CAN_BITRATE,
                 fd=False, data_bitrate=DEFAULT_CAN_DATA_BITRATE,
                 node_id=DEFAULT_CAN_NODE_ID, master_id=DEFAULT_CAN_MASTER_ID,
                 timeout=1.0, tty_baudrate=115200):
        if not channel:
            channel = guess_can_channel(interface)
        self.interface = interface
        self.channel = channel
        self.node_id = node_id & 0xFF
        self.master_id = master_id & 0xFF
        self.timeout = timeout
        self.fd = fd
        self.bitrate = bitrate
        self.data_bitrate = data_bitrate
        self.crc = 0
        self.seq = 0
        if self.node_id == 0:
            raise EpError("--can-id 不能是 0（0 表示“未配置”，设备不应答）")
        if fd and bitrate != CAN_FD_NOMINAL_BAUD:
            # 固件在 FD 下把仲裁域写死成 1 Mbit/s（config.can_baud 变成数据域速率），
            # 这里用别的仲裁域速率不可能通，直接点出来省得白折腾。
            print("警告: CAN FD 的仲裁域由固件固定为 %d bit/s；--bitrate 应保持 %d，"
                  "config.can_baud（2M/4M/5M）填到 --data-bitrate。"
                  % (CAN_FD_NOMINAL_BAUD, CAN_FD_NOMINAL_BAUD), file=sys.stderr)
        if interface == "slcan":
            self.bus = SlcanBus(channel, bitrate, fd, data_bitrate, tty_baudrate)
        else:
            self.bus = PyCanBus(interface, channel, bitrate, fd, data_bitrate)

    def close(self):
        self.bus.close()

    def describe(self):
        if self.fd:
            return "%s，仲裁域 %d / 数据域 %d bit/s（FD）" % (
                self.bus.describe(), self.bitrate, self.data_bitrate)
        return "%s，%d bit/s（Classic）" % (self.bus.describe(), self.bitrate)

    # -- 帧收发 -----------------------------------------------------------

    def _send(self, msgtype, data, seq=0):
        self.bus.send(cb_id(CB_PRI_CONFIG, msgtype, self.node_id, self.master_id, seq),
                      data)

    def _wait(self, msgtype, seq=None, timeout=None):
        """等一帧来自本节点、指定 msgtype 的应答，返回 data。"""
        end = time.time() + (self.timeout if timeout is None else timeout)
        while True:
            left = end - time.time()
            frame = self.bus.recv(left if left > 0 else 0)
            if frame is None:
                if time.time() >= end:
                    return None
                continue
            _pri, msgtype_rx, dest, src, seq_rx = cb_id_fields(frame[0])
            if msgtype_rx != msgtype or src != self.node_id:
                continue
            if seq is not None and seq_rx != seq:
                continue        # 上一条请求迟到的应答
            if dest != self.master_id:
                continue
            return frame[1]

    def _drain(self):
        while self.bus.recv(0) is not None:
            pass

    def sniff(self, seconds):
        """数一数总线上都有谁在发帧：{(source, msgtype): 帧数}。"""
        counts = {}
        end = time.time() + seconds
        while True:
            left = end - time.time()
            frame = self.bus.recv(left if left > 0 else 0)
            if frame is None:
                if left <= 0:
                    break
                continue
            _pri, msgtype, _dest, src, _seq = cb_id_fields(frame[0])
            key = (src, msgtype)
            counts[key] = counts.get(key, 0) + 1
        return counts

    # -- 三个动作 ---------------------------------------------------------

    def fetch_descriptor(self):
        last = None
        for _ in range(3):
            try:
                return self._fetch_descriptor_once()
            except _NoDescriptorReply:
                raise           # 一帧都没回来：重试只会白等
            except EpError as e:
                last = e
        raise last

    def _fetch_descriptor_once(self):
        self._drain()
        self._send(CB_MSG_JSON_DESC_READ, struct.pack("<I", 0))

        stall = max(3.0, self.timeout * 3)
        total = None
        json_crc = 0
        data = bytearray()
        while True:
            chunk = self._wait(CB_MSG_JSON_DESC_DATA, timeout=stall)
            if chunk is None:
                if total is None:
                    raise _NoDescriptorReply(
                        "设备没有回应描述符请求：检查 config.can_prot=1（CYBERBEAST）、"
                        "速率（Classic：--bitrate = config.can_baud，默认 1 Mbit/s；"
                        "FD：仲裁域固定 1 Mbit/s + --fd，--data-bitrate = config.can_baud）、"
                        "--can-id 是否等于 config.can_id、接线/终端电阻。")
                raise EpError("描述符下载中断（%d/%d 字节）" % (len(data), total))
            if total is None:
                # 第一帧固定是元数据：[00 00][长度 u32 LE][CRC16 LE]
                if len(chunk) < 8:
                    raise EpError("描述符元数据帧太短（%d 字节）" % len(chunk))
                total = struct.unpack("<I", chunk[2:6])[0]
                json_crc = struct.unpack("<H", chunk[6:8])[0]
                if not (16 < total <= 0x10000):
                    raise EpError("描述符长度异常（%d 字节）" % total)
                continue
            if len(chunk) < 2:
                continue
            offset = struct.unpack("<H", chunk[:2])[0]
            if offset < len(data):
                continue        # 重复帧
            if offset != len(data):
                raise EpError("描述符分片丢失（收到偏移 %d，期望 %d）" % (offset, len(data)))
            data += chunk[2:]
            if len(data) >= total:
                break

        del data[total:]
        # 上一次请求可能还有几帧在路上（_drain 之后才到），被当成元数据就会
        # 拿到垃圾长度/CRC；用 CRC 兜底，失败交给 fetch_descriptor 的重试。
        if crc16(bytes(data)) != json_crc:
            raise EpError("描述符 CRC 校验失败（可能有上一次请求的残留帧）")
        self.crc = json_crc
        return bytes(data)

    def read_value(self, ep_id, size):
        if size <= 0:
            return b""
        out = bytearray()
        chunk_max = 8 if self.fd else 4
        while len(out) < size:
            want = min(chunk_max, size - len(out))
            self.seq = (self.seq + 1) & 3
            req = bytes([0]) + struct.pack(">H", ep_id) + bytes([want]) \
                + struct.pack(">I", len(out))
            self._send(CB_MSG_PARAM_READ, req, self.seq)
            rsp = self._wait(CB_MSG_PARAM_READ, seq=self.seq)
            if rsp is None:
                raise EpError("endpoint %d 读超时（设备没应答）" % ep_id)
            if len(rsp) < 4:
                raise EpError("endpoint %d 应答太短（%d 字节）" % (ep_id, len(rsp)))
            rsp_ep = (rsp[1] << 8) | rsp[2]
            n = rsp[3]
            if rsp_ep != ep_id or len(rsp) < 4 + n:
                raise EpError("endpoint %d 应答格式不对" % ep_id)
            if n == 0:
                raise EpError("endpoint %d 没有返回数据（只写端点，或当前状态不允许读）"
                              % ep_id)
            out += rsp[4:4 + n]
            if len(out) < size and not (rsp[0] & CB_READ_MORE):
                raise EpError("endpoint %d 只回了 %d/%d 字节" % (ep_id, len(out), size))
        return bytes(out[:size])

    def write_value(self, ep_id, data, read_back=False):
        self._drain()
        if self.fd or len(data) <= 4:
            req = bytes([0]) + struct.pack(">H", ep_id) + bytes([len(data)]) + data
            self._send(CB_MSG_PARAM_WRITE, req)
        else:
            if len(data) > 8:
                raise EpError("Classic CAN 只能写 ≤8 字节（FD 可以到 64）")
            self._send(CB_MSG_PARAM_WRITE,
                       bytes([CB_WRITE_MORE]) + struct.pack(">H", ep_id)
                       + bytes([len(data)]) + data[:4])
            self._send(CB_MSG_PARAM_WRITE,
                       bytes([0]) + struct.pack(">H", ep_id)
                       + bytes([len(data)]) + data[4:])

        ack = self._wait(CB_MSG_PARAM_WRITE)
        if ack is None or len(ack) < 3 or ((ack[1] << 8) | ack[2]) != ep_id:
            raise EpError("endpoint %d 写没有应答（数据长度/分段不匹配，或总线丢帧）"
                          % ep_id)
        # ACK 不带成败位 ⇒ 读回才能知道是被限幅、被拒绝还是真写进去了
        if read_back:
            return self.read_value(ep_id, len(data))
        return b""

    def call_action(self, ep_id, via_read=False):
        # 动作端点用 PARAM_READ 触发（ODrive/JointSDK 的语义，固件两种都放行）；
        # CAN 只有这一种方式，via_read 对它是无效参数。
        self.seq = (self.seq + 1) & 3
        req = bytes([0]) + struct.pack(">H", ep_id) + bytes([0])
        self._send(CB_MSG_PARAM_READ, req, self.seq)
        rsp = self._wait(CB_MSG_PARAM_READ, seq=self.seq)
        if rsp is None:
            raise EpError("动作 %d 没有应答（超时）" % ep_id)
        if len(rsp) < 4 or ((rsp[1] << 8) | rsp[2]) != ep_id:
            raise EpError("动作 %d 应答格式不对" % ep_id)


class Device:
    def __init__(self, port=None, baud=115200, timeout=1.0, use_usb=False,
                 vid=DEFAULT_VID, pid=DEFAULT_PID, interface=DEFAULT_USB_INTERFACE,
                 ep_out=DEFAULT_EP_OUT, ep_in=DEFAULT_EP_IN, io=None,
                 can_interface=None, can_channel=None,
                 can_bitrate=DEFAULT_CAN_BITRATE, can_fd=False,
                 can_data_bitrate=DEFAULT_CAN_DATA_BITRATE,
                 can_id=DEFAULT_CAN_NODE_ID, can_master_id=DEFAULT_CAN_MASTER_ID,
                 can_tty_baudrate=115200):
        if io is not None:
            self.link = FibreLink(io)
        elif can_interface:
            self.link = CanLink(interface=can_interface, channel=can_channel,
                                bitrate=can_bitrate, fd=can_fd,
                                data_bitrate=can_data_bitrate, node_id=can_id,
                                master_id=can_master_id, timeout=timeout,
                                tty_baudrate=can_tty_baudrate)
        elif use_usb:
            self.link = FibreLink(UsbTransport(vid, pid, interface, ep_out, ep_in, timeout))
        else:
            if not port:
                raise EpError("需要 --port（CDC 串口）、--usb（vendor interface）"
                              "或 --can（CYBERBEAST）")
            self.link = FibreLink(SerialTransport(port, baud, timeout))
        self.descriptor = []
        self.by_name = {}
        self.by_id = {}
        self.json_crc = 0

    def close(self):
        self.link.close()

    # -- 描述符与类型 -----------------------------------------------------

    def fetch_descriptor(self):
        raw = self.link.fetch_descriptor()
        if crc16(raw) != self.link.crc:
            raise EpError("描述符 CRC 校验失败")
        self.json_crc = self.link.crc
        self.descriptor = json.loads(raw.decode("ascii"))
        self.by_name = {ep["name"]: ep for ep in self.descriptor}
        self.by_id = {ep["id"]: ep for ep in self.descriptor}
        return self.descriptor

    def _entry(self, name):
        if not self.by_name:
            self.fetch_descriptor()
        if name not in self.by_name:
            raise EpError("没有这个 endpoint: %s" % name)
        return self.by_name[name]

    def _decode(self, ep, data):
        fmt, size = TYPE_CODES[ep["type"]]
        if len(data) != size:
            raise EpError("%s: 返回 %d 字节，期望 %d" % (ep["name"], len(data), size))
        value = struct.unpack(fmt, data)[0]
        return bool(value) if ep["type"] == "bool" else value

    def _encode(self, ep, value):
        fmt, size = TYPE_CODES[ep["type"]]
        if ep["type"] == "bool":
            if isinstance(value, str):
                text = value.strip().lower()
                if text in ("1", "true", "on", "yes"):
                    value = 1
                elif text in ("0", "false", "off", "no"):
                    value = 0
                else:
                    raise EpError("bool 参数只接受 0/1、true/false")
            else:
                value = 1 if value else 0
        elif ep["type"] == "float":
            value = float(value)
        elif ep["type"].startswith("int") or ep["type"].startswith("uint"):
            value = int(value)
        return struct.pack(fmt, value)

    # -- 公共 API ---------------------------------------------------------

    def read(self, name):
        ep = self._entry(name)
        if ep["type"] in ("void", "function"):
            raise EpError("%s 是动作端点，请用 call" % name)
        if ep["access"] not in ("r", "rw"):
            raise EpError("%s 是只写端点" % name)
        _, size = TYPE_CODES[ep["type"]]
        return self._decode(ep, self.link.read_value(ep["id"], size))

    def write(self, name, value):
        ep = self._entry(name)
        if ep["type"] in ("void", "function"):
            raise EpError("%s 是动作端点，请用 call" % name)
        if ep["access"] not in ("w", "rw"):
            raise EpError("%s 是只读端点" % name)
        data = self.link.write_value(ep["id"], self._encode(ep, value),
                                     read_back=ep["access"] in ("r", "rw"))
        return self._decode(ep, data) if data else None

    def call(self, name, via_read=False):
        ep = self._entry(name)
        # "void" 是我们 params.json 的内部写法；线上描述符是 ODrive 的 "function"
        if ep["type"] not in ("void", "function"):
            raise EpError("%s 不是动作端点" % name)
        self.link.call_action(ep["id"], via_read=via_read)


def format_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return "%.6g" % value
    return str(value)


def cmd_tree(dev):
    dev.fetch_descriptor()
    last_group = None
    for ep in dev.descriptor:
        group = ep["name"].split(".")[0]
        if group != last_group:
            print("\n[%s]" % group)
            last_group = group
        extra = []
        if "unit" in ep:
            extra.append(ep["unit"])
        if "min" in ep or "max" in ep:
            extra.append("范围 %s..%s" % (ep.get("min", "-inf"), ep.get("max", "+inf")))
        print("  %-34s %-8s %-3s  %s" % (
            ep["name"], ep["type"], ep["access"], " ".join(extra)))


def cmd_dump(dev):
    dev.fetch_descriptor()
    for ep in dev.descriptor:
        if ep["type"] in ("void", "function") or ep["access"] not in ("r", "rw"):
            continue
        try:
            print("%-34s %s" % (ep["name"], format_value(dev.read(ep["name"]))))
        except EpError as e:
            print("%-34s <错误: %s>" % (ep["name"], e))


def cmd_monitor(dev, names, rate):
    dev.fetch_descriptor()
    period = 1.0 / rate if rate > 0 else 0.0
    try:
        while True:
            row = ["%-24s %s" % (name, format_value(dev.read(name))) for name in names]
            print("  ".join(row), flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        pass


def cmd_probe(dev, seconds=2.0):
    """--can 下的链路诊断：先只听（看总线上谁在发帧），再试读描述符。"""
    link = dev.link
    if not isinstance(link, CanLink):
        raise EpError("probe 只在 --can 下可用")
    print("监听 %.1f s（%s；节点 --can-id=%d，本机 --master-id=0x%02X）"
          % (seconds, link.describe(), link.node_id, link.master_id))
    counts = link.sniff(seconds)
    if not counts:
        print("  一帧都没有。依次检查：")
        print("    1) 接线（CANH/CANL）与两端 120 Ω 终端电阻；")
        print("    2) 速率：Classic 下 --bitrate 必须等于 config.can_baud；"
              "FD 下仲裁域固定 1 Mbit/s（--bitrate 1000000 + --fd），"
              "--data-bitrate 填 config.can_baud；")
        print("    3) 协议：config.can_prot 必须为 1（CYBERBEAST）；"
              "旧 11-bit 协议没有 endpoint 访问。")
        print("  ⚠ 如果刚才用错过波特率：设备的帧没人应答会让 TEC 涨到 ~248，"
              "固件不会自动恢复，")
        print("    CAN 会一直\"死\"到你用 USB 重启它。用另一个终端确认/恢复：")
        print("      python tools/mc_ep.py --port COMx get telemetry.can_tec"
              "   # >128 就是中招了")
        print("      python tools/mc_ep.py --port COMx call action.reboot")
        return False
    for (src, msgtype), n in sorted(counts.items()):
        mark = " ←--can-id" if src == link.node_id else ""
        print("  src=%-3d %-14s 0x%02X  %3d 帧%s"
              % (src, CAN_MSG_NAMES.get(msgtype, "?"), msgtype, n, mark))
    if not any(src == link.node_id for (src, _mt) in counts):
        print("  ⚠ 没有来自节点 %d 的帧：如果有别的 src 在发心跳，"
              "用 --can-id 指定它；" % link.node_id)
        print("    也可能是另一个主站在抢应答（同一总线只允许一个主站）。")
    try:
        desc = dev.fetch_descriptor()
    except EpError as e:
        print("  描述符请求失败：%s" % e)
        return False
    print("  描述符 OK：%d 个 endpoint，CRC16=0x%04X" % (len(desc), dev.json_crc))
    return True


def main():
    ap = argparse.ArgumentParser(description="motorcontrol endpoint 上位机")
    conn = ap.add_mutually_exclusive_group(required=True)
    conn.add_argument("--port", help="CDC 串口，例如 COM7 或 /dev/ttyACM0")
    conn.add_argument("--usb", action="store_true",
                      help="走 vendor interface（fibre，需要 pyusb + WinUSB）")
    conn.add_argument("--can", action="store_true",
                      help="走 CAN 的 CYBERBEAST SDO（后端见 --interface）")
    ap.add_argument("--vid", type=lambda s: int(s, 0), default=DEFAULT_VID)
    ap.add_argument("--pid", type=lambda s: int(s, 0), default=DEFAULT_PID)
    ap.add_argument("--usb-interface", type=int, default=DEFAULT_USB_INTERFACE,
                    help="vendor interface 号（默认 2）")
    ap.add_argument("--ep-out", type=lambda s: int(s, 0), default=DEFAULT_EP_OUT)
    ap.add_argument("--ep-in", type=lambda s: int(s, 0), default=DEFAULT_EP_IN)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=1.0)

    cang = ap.add_argument_group("CAN 选项（--can）")
    cang.add_argument("--interface", default="slcan",
                      help="CAN 后端：slcan（CANable，默认）/pcan/socketcan/"
                           "gs_usb/kvaser/vector/…（python-can 的名字）")
    cang.add_argument("--channel", default=None,
                      help="CAN 通道：slcan=COMx / pcan=PCAN_USBBUS1 / "
                           "socketcan=can0；不给时按后端猜")
    cang.add_argument("--bitrate", type=int, default=DEFAULT_CAN_BITRATE,
                      help="仲裁域速率：Classic 下等于 config.can_baud（默认 1 Mbit/s）；"
                           "FD 下固件固定 1 Mbit/s，别改它")
    cang.add_argument("--fd", action="store_true",
                      help="CAN FD（config.can_baud > 1M）；仲裁域固定 1 Mbit/s，"
                           "数据域速率用 --data-bitrate")
    cang.add_argument("--data-bitrate", type=int, default=DEFAULT_CAN_DATA_BITRATE,
                      help="FD 数据域速率，等于 config.can_baud"
                           "（固件只支持 2M/4M/5M，默认 5 Mbit/s）")
    cang.add_argument("--can-id", type=int, default=DEFAULT_CAN_NODE_ID,
                      help="设备节点 ID，等于 config.can_id（默认 1）")
    cang.add_argument("--master-id", type=lambda s: int(s, 0),
                      default=DEFAULT_CAN_MASTER_ID,
                      help="本机主站 ID（默认 0xFE），应答按这个 ID 送回")
    cang.add_argument("--tty-baudrate", type=int, default=115200,
                      help="slcan 串口速率（默认 115200，适配器固件固定值）")

    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("tree", help="打印设备自带的参数树")
    sub.add_parser("dump", help="读取所有可读参数")
    sub.add_parser("descriptor", help="打印原始 JSON 描述符")

    p = sub.add_parser("probe", help="只看链路（--can）：监听总线并试读描述符")
    p.add_argument("--seconds", type=float, default=2.0, help="监听时长（默认 2 s）")

    p = sub.add_parser("get", help="读一个参数")
    p.add_argument("name")

    p = sub.add_parser("set", help="写一个参数（打印限幅后的实际值）")
    p.add_argument("name")
    p.add_argument("value")

    p = sub.add_parser("call", help="执行一个 action.* 端点")
    p.add_argument("name")
    p.add_argument("--read", action="store_true",
                   help="用“读”请求触发（ODrive/JointSDK 的做法；固件两种都接受）")

    p = sub.add_parser("monitor", help="周期读取若干参数")
    p.add_argument("names", nargs="+")
    p.add_argument("--rate", type=float, default=10.0, help="刷新率 Hz（默认 10）")

    args = ap.parse_args()

    try:
        if args.can:
            dev = Device(timeout=args.timeout, can_interface=args.interface,
                         can_channel=args.channel, can_bitrate=args.bitrate,
                         can_fd=args.fd, can_data_bitrate=args.data_bitrate,
                         can_id=args.can_id, can_master_id=args.master_id,
                         can_tty_baudrate=args.tty_baudrate)
        else:
            dev = Device(port=args.port, baud=args.baud, timeout=args.timeout,
                         use_usb=args.usb, vid=args.vid, pid=args.pid,
                         interface=args.usb_interface,
                         ep_out=args.ep_out, ep_in=args.ep_in)
    except Exception as e:
        print("无法打开设备: %s" % e, file=sys.stderr)
        return 1

    rc = 0
    try:
        if args.cmd == "tree":
            cmd_tree(dev)
        elif args.cmd == "dump":
            cmd_dump(dev)
        elif args.cmd == "descriptor":
            dev.fetch_descriptor()
            print(json.dumps(dev.descriptor, ensure_ascii=False, indent=2))
        elif args.cmd == "probe":
            rc = 0 if cmd_probe(dev, args.seconds) else 3
        elif args.cmd == "get":
            print(format_value(dev.read(args.name)))
        elif args.cmd == "set":
            print(format_value(dev.write(args.name, args.value)))
        elif args.cmd == "call":
            dev.call(args.name, via_read=args.read)
            print("OK")
        elif args.cmd == "monitor":
            cmd_monitor(dev, args.names, args.rate)
    except EpError as e:
        print("错误: %s" % e, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        pass
    finally:
        dev.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
