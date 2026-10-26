#!/usr/bin/env python3
"""
motorcontrol endpoint 上位机。

设备会先通过 endpoint 0 返回一份 JSON 描述符（名字、ID、类型、读写权限），
本工具据此把名字映射到 endpoint ID，因此不需要和固件版本手工对齐。

依赖:
    pip install pyserial

示例:
    python tools/mc_ep.py --port COM7 tree
    python tools/mc_ep.py --port COM7 get config.i_max
    python tools/mc_ep.py --port COM7 set config.i_max 3.5
    python tools/mc_ep.py --port COM7 call action.save
    python tools/mc_ep.py --port COM7 dump
    python tools/mc_ep.py --port COM7 monitor telemetry.i_q telemetry.position

协议（见 Core/Inc/endpoint.h）:
    AA | LEN | CRC8 | PAYLOAD | CRC16_HI CRC16_LO
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


class Device:
    def __init__(self, port, baud=115200, timeout=1.0):
        if serial is None:
            raise EpError("需要 pyserial：pip install pyserial")
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.seq = 0
        self.descriptor = []
        self.by_name = {}
        self.by_id = {}
        self.json_crc = 0

    def close(self):
        self.ser.close()

    # -- 底层收发 ---------------------------------------------------------

    def _read_byte(self):
        b = self.ser.read(1)
        if not b:
            raise EpError("等待响应超时")
        return b[0]

    def _read_frame(self):
        while True:
            if self._read_byte() != FRAME_PREFIX:
                continue
            length = self._read_byte()
            if length == 0 or length > 64:
                continue
            header = bytes([FRAME_PREFIX, length])
            if self._read_byte() != crc8(header):
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

    def _transact(self, endpoint_id, tx_data, rx_len, trailer, write=False):
        self.seq = (self.seq + 1) & 0x7FFF
        flags = REPLY_FLAG | (WRITE_FLAG if write else 0)
        payload = struct.pack("<HHH", self.seq, endpoint_id | flags, rx_len)
        payload += tx_data
        payload += struct.pack("<H", trailer)

        self.ser.reset_input_buffer()
        self.ser.write(build_frame(payload))

        resp = self._read_frame()
        if len(resp) < 3:
            raise EpError("响应太短")
        rseq, status = struct.unpack("<HB", resp[:3])
        if (rseq & 0x7FFF) != self.seq:
            raise EpError("响应 seq 不匹配（期望 %d，收到 %d）" % (self.seq, rseq & 0x7FFF))
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
    ap.add_argument("--port", required=True, help="串口，例如 COM7 或 /dev/ttyACM0")
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
        dev = Device(args.port, args.baud, args.timeout)
    except Exception as e:
        print("无法打开串口: %s" % e, file=sys.stderr)
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
