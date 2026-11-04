#!/usr/bin/env python3
"""
motorcontrol endpoint 上位机（odrivetool 风格）。

设备会先通过 endpoint 0 返回一份 JSON 描述符（名字、ID、类型、读写权限），
本工具据此把名字映射到 endpoint ID，因此不需要和固件版本手工对齐。

两种传输，用同一套请求/应答 payload：

  --port COM7    USB CDC（虚拟串口），外层是 0xAA + CRC 字节流，
                 和 ASCII 菜单共用数据流；只需要 pyserial。
  --usb          USB vendor interface（interface 2）的 bulk 端点，
                 一个 USB 包就是一帧，没有 0xAA/CRC 外层；
                 需要 pyusb，Windows 上还要用 Zadig 给 interface 2 装 WinUSB
                 （CDC 那部分保持 usbser，不需要动）。

依赖:
    pip install pyserial            # --port
    pip install pyusb libusb-package  # --usb（libusb-package 在 Windows 上自带 DLL）

示例:
    python tools/mc_ep.py --port COM7 tree
    python tools/mc_ep.py --usb tree
    python tools/mc_ep.py --usb get config.i_max
    python tools/mc_ep.py --usb set config.i_max 3.5
    python tools/mc_ep.py --usb call action.save
    python tools/mc_ep.py --usb monitor telemetry.i_q telemetry.position --rate 100

协议（见 Core/Inc/endpoint.h）:
    CDC   : AA | LEN | CRC8 | PAYLOAD | CRC16_HI CRC16_LO
    USB   : 一个 bulk 包 = 一个 PAYLOAD（<= 63 字节，短包）
    请求 PAYLOAD: seq(2) | id|flags(2) | rxlen(2) | tx_data | trailer(2)
    响应 PAYLOAD: seq|0x8000(2) | status(1) | rx_data
    flags: bit15 = 需要应答，bit14 = 写/执行（不置位是读）
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
    "float": ("<f", 4),
}


class EpError(Exception):
    pass


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


class Device:
    def __init__(self, port=None, baud=115200, timeout=1.0, use_usb=False,
                 vid=DEFAULT_VID, pid=DEFAULT_PID, interface=DEFAULT_USB_INTERFACE,
                 ep_out=DEFAULT_EP_OUT, ep_in=DEFAULT_EP_IN, io=None):
        if io is not None:
            self.io = io
        elif use_usb:
            self.io = UsbTransport(vid, pid, interface, ep_out, ep_in, timeout)
        else:
            if not port:
                raise EpError("需要 --port（CDC 串口）或 --usb（vendor interface）")
            self.io = SerialTransport(port, baud, timeout)
        self.seq = 0
        self.descriptor = []
        self.by_name = {}
        self.by_id = {}
        self.json_crc = 0

    def close(self):
        self.io.close()

    # -- 底层收发 ---------------------------------------------------------

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

    # -- 描述符与类型 -----------------------------------------------------

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
        if crc16(bytes(raw)) != json_crc:
            raise EpError("描述符 CRC 校验失败")

        self.json_crc = json_crc
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
        data = self._transact(ep["id"], b"", size, self.json_crc)
        return self._decode(ep, data)

    def write(self, name, value):
        ep = self._entry(name)
        if ep["type"] in ("void", "function"):
            raise EpError("%s 是动作端点，请用 call" % name)
        if ep["access"] not in ("w", "rw"):
            raise EpError("%s 是只读端点" % name)
        _, size = TYPE_CODES[ep["type"]]
        data = self._transact(ep["id"], self._encode(ep, value), size, self.json_crc, write=True)
        return self._decode(ep, data) if data else None

    def call(self, name):
        ep = self._entry(name)
        # "void" 是我们 params.json 的内部写法；线上描述符是 ODrive 的 "function"
        if ep["type"] not in ("void", "function"):
            raise EpError("%s 不是动作端点" % name)
        self._transact(ep["id"], b"", 0, self.json_crc, write=True)


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


def main():
    ap = argparse.ArgumentParser(description="motorcontrol endpoint 上位机")
    link = ap.add_mutually_exclusive_group(required=True)
    link.add_argument("--port", help="CDC 串口，例如 COM7 或 /dev/ttyACM0")
    link.add_argument("--usb", action="store_true",
                      help="走 vendor interface（fibre，需要 pyusb + WinUSB）")
    ap.add_argument("--vid", type=lambda s: int(s, 0), default=DEFAULT_VID)
    ap.add_argument("--pid", type=lambda s: int(s, 0), default=DEFAULT_PID)
    ap.add_argument("--usb-interface", type=int, default=DEFAULT_USB_INTERFACE,
                    help="vendor interface 号（默认 2）")
    ap.add_argument("--ep-out", type=lambda s: int(s, 0), default=DEFAULT_EP_OUT)
    ap.add_argument("--ep-in", type=lambda s: int(s, 0), default=DEFAULT_EP_IN)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=1.0)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("tree", help="打印设备自带的参数树")
    sub.add_parser("dump", help="读取所有可读参数")
    sub.add_parser("descriptor", help="打印原始 JSON 描述符")

    p = sub.add_parser("get", help="读一个参数")
    p.add_argument("name")

    p = sub.add_parser("set", help="写一个参数（打印限幅后的实际值）")
    p.add_argument("name")
    p.add_argument("value")

    p = sub.add_parser("call", help="执行一个 action.* 端点")
    p.add_argument("name")

    p = sub.add_parser("monitor", help="周期读取若干参数")
    p.add_argument("names", nargs="+")
    p.add_argument("--rate", type=float, default=10.0, help="刷新率 Hz（默认 10）")

    args = ap.parse_args()

    try:
        dev = Device(port=args.port, baud=args.baud, timeout=args.timeout,
                     use_usb=args.usb, vid=args.vid, pid=args.pid,
                     interface=args.usb_interface,
                     ep_out=args.ep_out, ep_in=args.ep_in)
    except Exception as e:
        print("无法打开设备: %s" % e, file=sys.stderr)
        return 1

    try:
        if args.cmd == "tree":
            cmd_tree(dev)
        elif args.cmd == "dump":
            cmd_dump(dev)
        elif args.cmd == "descriptor":
            dev.fetch_descriptor()
            print(json.dumps(dev.descriptor, ensure_ascii=False, indent=2))
        elif args.cmd == "get":
            print(format_value(dev.read(args.name)))
        elif args.cmd == "set":
            print(format_value(dev.write(args.name, args.value)))
        elif args.cmd == "call":
            dev.call(args.name)
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
