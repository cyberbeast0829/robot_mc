"""CYBERBEAST（29-bit 扩展帧）上的 endpoint 访问。

设备侧见 Core/Src/can_cyberbeast.c：
  PARAM_READ(0x20)   读；响应 flags 的 bit7 = 还有后续字节
                     （Classic 每次最多 4 字节，>4 字节的值要按 offset 分段读）
  PARAM_WRITE(0x21)  写；Classic 下 >4 字节走两帧分段写（bit7 = MORE）
  JSON_DESC_READ(0x24)/JSON_DESC_DATA(0x25)  描述符分片流

⚠ ACK **不含成败位**（只读/状态不符/被限幅都照样 ACK），所以写完一律读回。
⚠ 固件的“主站 ID”是从收到的帧里学的，同一条总线上只能有一个主站在发命令。
"""

import struct
import sys
import time

from ..errors import EpError, NoDescriptorReply
from ..protocol import (
    CAN_FD_NOMINAL_BAUD, CAN_MSG_NAMES, CB_MSG_JSON_DESC_DATA, CB_MSG_JSON_DESC_READ,
    CB_MSG_PARAM_READ, CB_MSG_PARAM_WRITE, CB_PRI_CONFIG, CB_READ_MORE, CB_WRITE_MORE,
    DEFAULT_CAN_BITRATE, DEFAULT_CAN_DATA_BITRATE, DEFAULT_CAN_INTERFACE,
    DEFAULT_CAN_MASTER_ID, DEFAULT_CAN_NODE_ID, SLCAN_BITRATES, SLCAN_CODE2LEN,
    SLCAN_DATA_BITRATES, SLCAN_LEN2CODE, cb_id, cb_id_fields, crc16,
)

try:
    import serial
except ImportError:      # pragma: no cover
    serial = None

try:
    import can
except ImportError:      # pragma: no cover
    can = None


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
        elif interface == "udp_multicast":
            # 本机跨进程的模拟总线（tools/rmc/tests/sim_device.py）：
            # --channel 239.74.163.10:28700（IP[:port]）。不支持 FD。
            host, sep, port = str(channel).partition(":")
            kwargs["channel"] = host
            if sep:
                kwargs["port"] = int(port)
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
        return "%s %s" % (self.interface, self.channel)

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


class CanLink:
    """CYBERBEAST 上的 endpoint 访问（见模块头注释）。"""

    is_can = True

    def __init__(self, interface=DEFAULT_CAN_INTERFACE, channel=None,
                 bitrate=DEFAULT_CAN_BITRATE, fd=False,
                 data_bitrate=DEFAULT_CAN_DATA_BITRATE,
                 node_id=DEFAULT_CAN_NODE_ID, master_id=DEFAULT_CAN_MASTER_ID,
                 timeout=1.0, tty_baudrate=115200, bus=None):
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
        self._owns_bus = bus is None
        if self.node_id == 0:
            raise EpError("--can-id 不能是 0（0 表示“未配置”，设备不应答）")
        if fd and bitrate != CAN_FD_NOMINAL_BAUD:
            # 固件在 FD 下把仲裁域写死成 1 Mbit/s（config.can_baud 变成数据域速率），
            # 这里用别的仲裁域速率不可能通，直接点出来省得白折腾。
            print("警告: CAN FD 的仲裁域由固件固定为 %d bit/s；--bitrate 应保持 %d，"
                  "config.can_baud（2M/4M/5M）填到 --data-bitrate。"
                  % (CAN_FD_NOMINAL_BAUD, CAN_FD_NOMINAL_BAUD), file=sys.stderr)
        if bus is not None:
            self.bus = bus
        elif interface == "slcan":
            self.bus = SlcanBus(channel, bitrate, fd, data_bitrate, tty_baudrate)
        else:
            self.bus = PyCanBus(interface, channel, bitrate, fd, data_bitrate)

    def close(self):
        if self._owns_bus:
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

    def listen(self, seconds):
        """监听并把帧交给回调的版本（scan 用），返回 [(id, data), ...]。"""
        frames = []
        end = time.time() + seconds
        while True:
            left = end - time.time()
            frame = self.bus.recv(left if left > 0 else 0)
            if frame is None:
                if left <= 0:
                    break
                continue
            frames.append(frame)
        return frames

    # -- 三个动作 ---------------------------------------------------------

    def fetch_descriptor(self):
        last = None
        for _ in range(3):
            try:
                return self._fetch_descriptor_once()
            except NoDescriptorReply:
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
                    raise NoDescriptorReply(
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


class CanBusManager:
    """一个 shell 会话复用同一物理 CAN 通道，按 node_id 创建逻辑 link。"""

    def __init__(self):
        self._buses = {}

    @staticmethod
    def _key(interface, channel, bitrate, fd, data_bitrate, tty_baudrate):
        return (interface, channel, bitrate, bool(fd), data_bitrate, tty_baudrate)

    def link(self, interface=DEFAULT_CAN_INTERFACE, channel=None,
             bitrate=DEFAULT_CAN_BITRATE, fd=False,
             data_bitrate=DEFAULT_CAN_DATA_BITRATE,
             node_id=DEFAULT_CAN_NODE_ID, master_id=DEFAULT_CAN_MASTER_ID,
             timeout=1.0, tty_baudrate=115200):
        if not channel:
            channel = guess_can_channel(interface)
        key = self._key(interface, channel, bitrate, fd, data_bitrate, tty_baudrate)
        bus = self._buses.get(key)
        if bus is None:
            if interface == "slcan":
                bus = SlcanBus(channel, bitrate, fd, data_bitrate, tty_baudrate)
            else:
                bus = PyCanBus(interface, channel, bitrate, fd, data_bitrate)
            self._buses[key] = bus
        return CanLink(interface=interface, channel=channel, bitrate=bitrate,
                       fd=fd, data_bitrate=data_bitrate, node_id=node_id,
                       master_id=master_id, timeout=timeout,
                       tty_baudrate=tty_baudrate, bus=bus)

    def close(self):
        buses, self._buses = self._buses, {}
        for bus in buses.values():
            try:
                bus.close()
            except Exception:
                pass


def describe_can_bus(link):
    """给 scan/probe 用的通道描述。"""
    if not isinstance(link, CanLink):
        raise EpError("这个命令只在 --can 下可用")
    return link.describe()


__all__ = ["CanLink", "CanBusManager", "PyCanBus", "SlcanBus",
           "guess_can_channel", "describe_can_bus", "CAN_MSG_NAMES"]
