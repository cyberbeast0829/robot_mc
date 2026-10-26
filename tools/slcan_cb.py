#!/usr/bin/env python3
"""CANable 2.0 (slcan + CANFD) 小工具：给 motorcontrol 的 CyberBeast 协议做台架测试。

用法示例：
  python tools/slcan_cb.py --port COM7 --probe-legacy
  python tools/slcan_cb.py --port COM7 --cyberbeast-classic
"""

import argparse
import struct
import sys
import time

import serial

CODE2LEN = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
            9: 12, 0xA: 16, 0xB: 20, 0xC: 24, 0xD: 32, 0xE: 48, 0xF: 64}
LEN2CODE = {v: k for k, v in CODE2LEN.items()}


class Slcan:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser.reset_input_buffer()

    def cmd(self, text, wait=0.05):
        self.ser.write((text + "\r").encode())
        self.ser.flush()
        time.sleep(wait)
        return self.ser.read(4096).decode(errors="replace")

    def open(self, nominal_code, data_code=None):
        self.cmd("C")
        self.cmd("S%d" % nominal_code)
        if data_code is not None:
            self.cmd("Y%d" % data_code)
        self.cmd("M0")
        self.cmd("A1")
        self.cmd("O", 0.2)
        self.ser.reset_input_buffer()

    def close(self):
        try:
            self.cmd("C")
        finally:
            self.ser.close()

    def send(self, can_id, data, fd=False, brs=False, ext=True):
        if fd:
            code = LEN2CODE.get(len(data))
            if code is None:
                raise ValueError("FD length %d not encodable" % len(data))
            letter = ("B" if brs else "D") if ext else ("b" if brs else "d")
            line = "%s%08X%X%s" % (letter, can_id & 0x1FFFFFFF, code, data.hex().upper())
        else:
            if len(data) > 8:
                raise ValueError("classic frame > 8 bytes")
            letter = "T" if ext else "t"
            idfmt = "%08X" % (can_id & 0x1FFFFFFF) if ext else "%03X" % (can_id & 0x7FF)
            line = "%s%s%X%s" % (letter, idfmt, len(data), data.hex().upper())
        self.ser.write((line + "\r").encode())
        self.ser.flush()

    @staticmethod
    def parse(line):
        line = line.strip()
        if not line:
            return None
        c = line[0]
        try:
            if c in "tT":
                idlen = 3 if c == "t" else 8
                can_id = int(line[1:1 + idlen], 16)
                dlc = int(line[1 + idlen], 16)
                raw = line[2 + idlen:]
                data = bytes.fromhex(raw)[:dlc]
                return {"ext": c == "T", "fd": False, "brs": False,
                        "id": can_id, "data": data}
            if c in "dDbB":
                can_id = int(line[1:9], 16)
                code = int(line[9], 16)
                n = CODE2LEN[code]
                data = bytes.fromhex(line[10:])[:n]
                return {"ext": c in "DB", "fd": True, "brs": c in "bB",
                        "id": can_id, "data": data}
        except (ValueError, IndexError):
            return None
        return None

    def drain(self):
        self.ser.reset_input_buffer()

    def read_frames(self, duration=0.3):
        end = time.time() + duration
        buf = b""
        while time.time() < end:
            chunk = self.ser.read(4096)
            if chunk:
                buf += chunk
        frames = []
        for line in buf.decode(errors="replace").replace("\n", "\r").split("\r"):
            f = Slcan.parse(line)
            if f:
                frames.append(f)
        return frames


# ---- CyberBeast helpers -------------------------------------------------

PRI = {"critical": 0, "emergency": 1, "high_ctrl": 2, "ctrl": 3,
       "config": 4, "query": 5, "status": 6}

MT = {"mit": 0x00, "pos": 0x01, "vel": 0x02, "torque": 0x03, "current": 0x04,
      "param_read": 0x20, "param_write": 0x21, "config_save": 0x22,
      "config_reset": 0x23, "desc_read": 0x24, "desc_data": 0x25,
      "query_status": 0x40, "query_pos_vel": 0x41, "query_current": 0x42,
      "query_temp": 0x43, "query_bus": 0x44, "query_error": 0x45,
      "query_dev": 0x46, "query_power": 0x47, "heartbeat": 0x48,
      "status_feedback": 0x49, "set_node_id": 0x60, "set_zero": 0x61,
      "start": 0x62, "stop": 0x63, "reset": 0x64, "clear": 0x65,
      "mit_bcast": 0x80, "estop": 0xC0}


def cb_id(pri, mt, dest, src, seq=0):
    return ((pri & 7) << 26) | ((mt & 0xFF) << 18) | ((dest & 0xFF) << 10) \
        | ((src & 0xFF) << 2) | (seq & 3)


def id_fields(can_id):
    return {"pri": (can_id >> 26) & 7, "mt": (can_id >> 18) & 0xFF,
            "dest": (can_id >> 10) & 0xFF, "src": (can_id >> 2) & 0xFF,
            "seq": can_id & 3}


def be_f32(v):
    return struct.pack(">f", v)


def le_u32(v):
    return struct.pack("<I", v)


def probe_legacy(bus):
    """旧协议在 500k 下的一次读变量：验证接线/适配器。"""
    bus.open(6)  # S6 = 500k
    bus.drain()
    # [CAN_LEGACY_READ_VAR=1, var 0=position], standard ID = node 1
    bus.send(0x001, bytes([0x01, 0x00]), fd=False, ext=False)
    frames = bus.read_frames(0.5)
    print("legacy probe frames:")
    for f in frames:
        print("  %s id=0x%03X data=%s" % ("ext" if f["ext"] else "std",
                                          f["id"], f["data"].hex()))
    ok = any((not f["ext"]) and f["id"] == 0 and len(f["data"]) >= 6
             and f["data"][1] == 0 for f in frames)
    print("legacy probe:", "OK" if ok else "NO REPLY")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM7")
    ap.add_argument("--probe-legacy", action="store_true")
    ap.add_argument("--cyberbeast-classic", action="store_true")
    args = ap.parse_args()

    bus = Slcan(args.port)
    try:
        if args.probe_legacy:
            probe_legacy(bus)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
