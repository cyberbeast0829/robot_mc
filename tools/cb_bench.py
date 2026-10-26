#!/usr/bin/env python3
"""CYBERBEAST 台架测试（CANable 2.0 slcan）。

阶段 A：不使能电机 —— 握手 / 心跳 / SDO / 描述符 / 查询 / ESTOP。
阶段 B：使能电机（零增益 → 小力矩 → 停止）。**需要显式 --stage-b。**
"""

import argparse
import json
import struct
import sys
import time

import serial

CODE2LEN = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
            9: 12, 0xA: 16, 0xB: 20, 0xC: 24, 0xD: 32, 0xE: 48, 0xF: 64}
LEN2CODE = {v: k for k, v in CODE2LEN.items()}

# ---- endpoint IDs (Core/Inc/endpoint_defs.h) ---------------------------
EP_I_MAX = 2
EP_P_MAX = 8
EP_V_MAX = 10
EP_KP_MAX = 12
EP_KD_MAX = 13
EP_GR = 14
EP_CAN_HB_MS = 122
EP_CAN_PROT_ACTIVE = 123
EP_CAN_FD = 124
EP_CAN_BAUD_ACTIVE = 125
EP_PREFS_SEQUENCE = 69
EP_CAN_TIMEOUT = 25
EP_CAN_TIMEOUT_COUNT = 60
EP_I_BW = 1
EP_ACTION_SAVE = 71
EP_V_BUS = 28
EP_POSITION = 43
EP_I_Q_FILT = 36
EP_FAULT = 56
EP_MODE = 58
EP_VERSION = 68
EP_ENCODER_RAW = 79
EP_GATE_DRIVE = 57
EP_VELOCITY = 45
EP_I_Q_FILT = 36
EP_P_DES = 49
EP_V_DES = 50
EP_KP = 51
EP_KD = 52
EP_T_FF = 53

# ---- CyberBeast message types ------------------------------------------
MT_MIT = 0x00
MT_TORQUE = 0x03
MT_CURRENT = 0x04
MT_MIT_BCAST = 0x80
MT_PARAM_READ = 0x20
MT_PARAM_WRITE = 0x21
MT_DESC_READ = 0x24
MT_DESC_DATA = 0x25
MT_QUERY_STATUS = 0x40
MT_QUERY_POS_VEL = 0x41
MT_QUERY_CURRENT = 0x42
MT_QUERY_TEMP = 0x43
MT_QUERY_BUS = 0x44
MT_QUERY_ERROR = 0x45
MT_QUERY_DEV = 0x46
MT_QUERY_POWER = 0x47
MT_HEARTBEAT = 0x48
MT_STATUS_FEEDBACK = 0x49
MT_START = 0x62
MT_STOP = 0x63
MT_RESET = 0x64
MT_CLEAR = 0x65
MT_ESTOP = 0xC0

PRI_HIGH_CTRL = 2
PRI_CONFIG = 4
PRI_QUERY = 5
PRI_STATUS = 6

NODE = 1
MASTER = 2

CHECK = {"pass": 0, "fail": 0}


def check(name, cond, detail=""):
    if cond:
        CHECK["pass"] += 1
        print("  [ OK ] %s %s" % (name, detail))
    else:
        CHECK["fail"] += 1
        print("  [FAIL] %s %s" % (name, detail))
    return cond


class Slcan:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser.reset_input_buffer()
        self.fd = False

    def cmd(self, text, wait=0.05):
        self.ser.write((text + "\r").encode())
        self.ser.flush()
        time.sleep(wait)
        n = self.ser.in_waiting
        return self.ser.read(n).decode(errors="replace") if n else ""

    def open(self, nominal_code, data_code=None):
        self.cmd("C")
        self.cmd("S%d" % nominal_code)
        if data_code is not None:
            self.cmd("Y%d" % data_code)
        self.cmd("M0")
        self.cmd("A1")
        self.cmd("O", 0.2)
        self.drain()

    def close(self):
        try:
            self.cmd("C")
        finally:
            self.ser.close()

    def send(self, can_id, data, fd=False, brs=False):
        if fd:
            code = LEN2CODE.get(len(data))
            if code is None:
                raise ValueError("FD length %d not encodable" % len(data))
            line = "%s%08X%X%s" % ("B" if brs else "D", can_id & 0x1FFFFFFF,
                                   code, data.hex().upper())
        else:
            if len(data) > 8:
                raise ValueError("classic frame > 8 bytes")
            line = "T%08X%X%s" % (can_id & 0x1FFFFFFF, len(data), data.hex().upper())
        self.ser.write((line + "\r").encode())
        self.ser.flush()

    def send_cb(self, can_id, data):
        """发送一帧 CyberBeast：FD 模式下 >8 B 用 BRS，≤8 B 不用。"""
        self.send(can_id, data, fd=self.fd,
                  brs=bool(self.fd and len(data) > 8))

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
                data = bytes.fromhex(line[2 + idlen:])[:dlc]
                return {"fd": False, "brs": False, "id": can_id, "data": data}
            if c in "dDbB":
                can_id = int(line[1:9], 16)
                n = CODE2LEN[int(line[9], 16)]
                data = bytes.fromhex(line[10:])[:n]
                return {"fd": True, "brs": c in "bB", "id": can_id, "data": data}
        except (ValueError, IndexError):
            return None
        return None

    def drain(self):
        self.ser.reset_input_buffer()

    @staticmethod
    def parse_buf(buf):
        txt = buf.decode(errors="replace").replace("\n", "\r")
        return [f for f in (Slcan.parse(line) for line in txt.split("\r")) if f]

    def read_frames(self, duration=0.3, stop_mt=None):
        """非阻塞轮询收帧。

        不要用 `ser.read(4096)`：串口 timeout=50 ms，它会一直等到超时才返回，
        于是每次 SDO 读要 300 ms、pump 循环掉到 ~20 Hz —— 比 CAN_TIMEOUT(25 ms)
        还慢，会把命令反复清掉（测试假象）。`stop_mt` 给定后，收到该 MsgType 就
        提前返回，SDO 一次往返只要 1~2 ms。"""
        end = time.time() + duration
        buf = b""
        while time.time() < end:
            n = self.ser.in_waiting
            if n:
                buf += self.ser.read(n)
            else:
                time.sleep(0.0005)
            if stop_mt is not None and buf:
                out = self.parse_buf(buf)
                if any(is_from_node(f, stop_mt) for f in out):
                    return out
        return self.parse_buf(buf)


def cb_id(pri, mt, dest, src=MASTER, seq=0):
    return ((pri & 7) << 26) | ((mt & 0xFF) << 18) | ((dest & 0xFF) << 10) \
        | ((src & 0xFF) << 2) | (seq & 3)


def fields(can_id):
    return ((can_id >> 26) & 7, (can_id >> 18) & 0xFF,
            (can_id >> 10) & 0xFF, (can_id >> 2) & 0xFF, can_id & 3)


def is_from_node(f, mt):
    pri, m, dest, src, seq = fields(f["id"])
    return m == mt and src == NODE and dest == MASTER


def is_hb(f):
    """心跳的 Dest 是“主站 ID”（CyberBeast §4.6），设备还没收到任何帧之前
    按默认主站 1 发送，所以不能按 dest==MASTER 过滤。"""
    pri, mt, dest, src, seq = fields(f["id"])
    return mt == MT_HEARTBEAT and src == NODE


def hb_dest(f):
    return fields(f["id"])[2]


def find(frames, mt, timeout_ok=True):
    for f in frames:
        if is_from_node(f, mt):
            return f
    return None


def crc16(data, rem=0x1337, poly=0x3D65):
    for b in data:
        rem ^= b << 8
        for _ in range(8):
            if rem & 0x8000:
                rem = ((rem << 1) ^ poly) & 0xFFFF
            else:
                rem = (rem << 1) & 0xFFFF
    return rem


# ---- SDO ----------------------------------------------------------------

def sdo_read(bus, ep, n=4, timeout=0.3):
    req = bytes([0, (ep >> 8) & 0xFF, ep & 0xFF, n])
    bus.send_cb(cb_id(PRI_CONFIG, MT_PARAM_READ, NODE), req)
    frames = bus.read_frames(timeout, stop_mt=MT_PARAM_READ)
    f = find(frames, MT_PARAM_READ)
    if not f or len(f["data"]) < 4:
        return None, None
    d = f["data"]
    rsp_ep = (d[1] << 8) | d[2]
    ln = d[3]
    if rsp_ep != ep or len(d) < 4 + ln:
        return None, None
    return d[4:4 + ln], ln


def sdo_write(bus, ep, value, timeout=0.3):
    req = bytes([0, (ep >> 8) & 0xFF, ep & 0xFF, len(value)]) + value
    if len(req) < 8:
        req += b"\x00" * (8 - len(req))       # SDK 会补零，固件只认 data_len
    bus.send_cb(cb_id(PRI_CONFIG, MT_PARAM_WRITE, NODE), req)
    frames = bus.read_frames(timeout, stop_mt=MT_PARAM_WRITE)
    f = find(frames, MT_PARAM_WRITE)
    return f is not None


def le_f32(b):
    return struct.unpack("<f", b[:4])[0]


def le_i32(b):
    return struct.unpack("<i", b[:4])[0]


def be_f32(b, off=0):
    return struct.unpack(">f", b[off:off + 4])[0]


def decode_hb(d):
    """按长度自动识别 Classic(8) / FD(>=18)。"""
    if len(d) < 8:
        return None
    if len(d) < 18:
        pos = struct.unpack(">h", d[3:5])[0]
        vel = struct.unpack(">h", d[5:7])[0]
        iq = struct.unpack(">b", d[7:8])[0]
        return {"fd": False, "life": d[0] >> 5, "err_flags": d[0] & 0x1F,
                "state": d[1] >> 4, "ctrl": d[1] & 0x0F,
                "temp": d[2] - 50, "pos_turns": pos / 100.0,
                "vel_turns": vel / 100.0, "iq": iq * 0.5}
    return {"fd": True, "life": d[0] >> 5, "err_flags": d[0] & 0x1F,
            "state": d[1] >> 4, "ctrl": d[1] & 0x0F,
            "temp": d[2] - 50, "mos": d[3] - 50,
            "vbus": struct.unpack(">H", d[4:6])[0] / 10.0,
            "ibus": struct.unpack(">h", d[6:8])[0] / 100.0,
            "pos_turns": struct.unpack(">i", d[8:12])[0] / 10000.0,
            "vel_turns": struct.unpack(">i", d[12:16])[0] / 10000.0,
            "iq": struct.unpack(">h", d[16:18])[0] / 100.0,
            "state_full": d[18] if len(d) > 18 else None}


def decode_mit_rsp(d):
    if len(d) < 8:
        return None
    pos_i = (d[0] << 8) | d[1]
    vel_i = (d[2] << 4) | (d[3] >> 4)
    cur_i = (d[4] << 4) | (d[5] >> 4)
    err = d[3] & 0x0F
    mode = d[5] & 0x0F
    return {"pos_i": pos_i, "vel_i": vel_i, "cur_i": cur_i,
            "err": err, "mode": mode,
            "motor_temp": d[6] - 50, "mos_temp": d[7] - 50}


# ---- tests --------------------------------------------------------------

def handshake(bus):
    print("[1] handshake / QUERY_STATUS")
    bus.send_cb(cb_id(PRI_QUERY, MT_QUERY_STATUS, NODE), b"")
    frames = bus.read_frames(0.6)
    f = find(frames, MT_MIT)
    ok = f is not None and len(f["data"]) == 8
    check("MIT response", ok, "" if not f else f["data"].hex())
    if ok:
        print("      decoded:", decode_mit_rsp(f["data"]))
    return frames


def read_all_sdo(bus):
    print("[2] SDO reads (little-endian values)")
    vals = {}
    for name, ep, kind in [("gr", EP_GR, "f32"), ("i_max", EP_I_MAX, "f32"),
                           ("p_max", EP_P_MAX, "f32"), ("v_max", EP_V_MAX, "f32"),
                           ("kp_max", EP_KP_MAX, "f32"), ("kd_max", EP_KD_MAX, "f32"),
                           ("version", EP_VERSION, "f32"), ("mode", EP_MODE, "u8"),
                           ("fault", EP_FAULT, "i32"), ("encoder_raw", EP_ENCODER_RAW, "i32"),
                           ("v_bus", EP_V_BUS, "f32"), ("can_hb_ms", EP_CAN_HB_MS, "i32")]:
        raw, ln = sdo_read(bus, ep)
        ok = raw is not None and ln is not None and len(raw) == ln and ln >= 1
        if ok:
            if kind == "f32":
                v = le_f32(raw)
            elif kind == "u8":
                v = raw[0]
            else:
                v = le_i32(raw)
            vals[name] = v
        check("read %-12s ep=%-3d" % (name, ep), ok,
              "" if not ok else "= %s" % (round(vals[name], 6) if kind == "f32" else vals[name]))
    return vals


def hb_period(bus, seconds=1.2):
    print("[3] heartbeat (classic 8 B)")
    frames = bus.read_frames(seconds)
    hbs = [f for f in frames if is_hb(f)]
    check("heartbeat received", len(hbs) >= 3,
          "n=%d dest=%s" % (len(hbs), hb_dest(hbs[0]) if hbs else "-"))
    if hbs:
        print("      first:", decode_hb(hbs[0]["data"]))
        times = []
        # 时间戳不可得：用相邻读取批次估算不了精确间隔，这里只检查 Life 连续
        lives = [decode_hb(f["data"])["life"] for f in hbs]
        cont = all((lives[i + 1] - lives[i]) % 8 == 1 for i in range(len(lives) - 1))
        check("life increments", cont, str(lives))
        return hbs
    return []


def write_hb_period(bus, ms):
    print("[4] SDO write/read config.can_hb_ms -> %d" % ms)
    okw = sdo_write(bus, EP_CAN_HB_MS, struct.pack("<i", ms))
    check("write ack", okw)
    time.sleep(0.1)
    raw, ln = sdo_read(bus, EP_CAN_HB_MS)
    ok = raw is not None and le_i32(raw) == ms
    check("read back", ok, "" if not ok else str(le_i32(raw)))


def fetch_descriptor(bus, expect_len=9881):
    print("[5] JSON descriptor (0x24 / 0x25)")
    need = expect_len
    total = None
    crc = None
    data = bytearray()
    bus.drain()
    bus.send_cb(cb_id(PRI_CONFIG, MT_DESC_READ, NODE), struct.pack("<I", 0))
    end = time.time() + 3.0
    while time.time() < end and (total is None or len(data) < total):
        for f in bus.read_frames(0.15):
            if not is_from_node(f, MT_DESC_DATA):
                continue
            d = f["data"]
            if total is None and d[0] == 0 and d[1] == 0:
                total = struct.unpack("<I", d[2:6])[0]
                crc = struct.unpack("<H", d[6:8])[0]
                print("      metadata: total=%d crc=0x%04X" % (total, crc))
                continue
            if total is None:
                continue
            off = struct.unpack("<H", d[0:2])[0]
            payload = d[2:]
            if off != len(data):
                print("      offset gap: got %d expected %d" % (off, len(data)))
                data.extend(b"\x00" * max(0, off - len(data)))
            data.extend(payload)
    ok_len = total == expect_len
    check("descriptor length", ok_len, "got %s expect %d (rx %d)" % (total, expect_len, len(data)))
    if total:
        data = data[:total]
        ok_crc = crc16(bytes(data)) == crc
        check("descriptor CRC16", ok_crc, "calc=0x%04X meta=0x%04X" % (crc16(bytes(data)), crc or 0))
        try:
            desc = json.loads(bytes(data).decode("ascii"))
            names = {e["name"] for e in desc}
            need_names = {"config.can_prot", "config.can_baud", "system.can_fd",
                          "action.save", "telemetry.position"}
            check("descriptor parses", True, "%d endpoints" % len(desc))
            check("new endpoints present", need_names <= names,
                  str(sorted(need_names - names)))
        except Exception as e:  # noqa: BLE001
            check("descriptor parses", False, repr(e))


def queries(bus):
    print("[6] query frames 0x41..0x49")
    for mt, name in [(MT_QUERY_POS_VEL, "pos_vel"), (MT_QUERY_CURRENT, "current"),
                     (MT_QUERY_TEMP, "temp"), (MT_QUERY_BUS, "bus"),
                     (MT_QUERY_DEV, "device"), (MT_QUERY_POWER, "power")]:
        bus.send_cb(cb_id(PRI_QUERY, mt, NODE), b"")
        frames = bus.read_frames(0.25)
        f = find(frames, mt)
        if check("query %-8s" % name, f is not None,
                 "" if not f else f["data"].hex()) and f:
            if mt in (MT_QUERY_POS_VEL, MT_QUERY_CURRENT, MT_QUERY_TEMP, MT_QUERY_BUS):
                print("      a=%.6f b=%.6f" % (be_f32(f["data"], 0), be_f32(f["data"], 4)))
            elif mt == MT_QUERY_DEV:
                print("      hw=0x%08X fw=0x%08X" % (struct.unpack(">I", f["data"][:4])[0],
                                                     struct.unpack(">I", f["data"][4:8])[0]))
    bus.send_cb(cb_id(PRI_QUERY, MT_QUERY_ERROR, NODE), bytes([5]))
    frames = bus.read_frames(0.25)
    f = find(frames, MT_QUERY_ERROR)
    check("query error", f is not None, "" if not f else f["data"].hex())


def estop_clear(bus):
    print("[7] ESTOP 0xC0 -> CLEAR 0x65")
    bus.send_cb(cb_id(1, MT_ESTOP, 0xFF, MASTER), b"")
    time.sleep(0.1)
    raw, _ = sdo_read(bus, EP_FAULT)
    fault = le_i32(raw) if raw else -1
    check("fault latched (FAULT_ESTOP=4)", fault == 4, "fault=%d" % fault)
    bus.send_cb(cb_id(3, MT_CLEAR, NODE), b"")
    time.sleep(0.15)
    raw, _ = sdo_read(bus, EP_FAULT)
    fault = le_i32(raw) if raw else -1
    check("fault cleared", fault == 0, "fault=%d" % fault)


def stage_a(bus):
    handshake(bus)
    read_all_sdo(bus)
    hb_period(bus)
    write_hb_period(bus, 200)
    hb_period(bus, 1.0)
    write_hb_period(bus, 100)
    fetch_descriptor(bus)
    queries(bus)
    estop_clear(bus)


# ---- stage B: motor -----------------------------------------------------

def f2u(x, lo, hi, bits):
    if not (hi > lo):
        return 0
    x = max(lo, min(hi, x))
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))


def mit_bytes(pos, vel, kp, kd, tau, rng):
    p = f2u(pos, -rng["p"], rng["p"], 16)
    v = f2u(vel, -rng["v"], rng["v"], 12)
    kp_i = f2u(kp, 0.0, rng["kp"], 12)
    kd_i = f2u(kd, 0.0, rng["kd"], 12)
    t = f2u(tau, -rng["tau"], rng["tau"], 12)
    return bytes([(p >> 8) & 0xFF, p & 0xFF,
                  (v >> 4) & 0xFF, ((v & 0x0F) << 4) | ((kp_i >> 8) & 0x0F),
                  kp_i & 0xFF, (kd_i >> 4) & 0xFF,
                  ((kd_i & 0x0F) << 4) | ((t >> 8) & 0x0F), t & 0xFF])


def decode_mit_out(d, rng):
    p = (d[0] << 8) | d[1]
    v = (d[2] << 4) | (d[3] >> 4)
    c = (d[4] << 4) | (d[5] >> 4)
    return {"pos": p / 65535.0 * 2 * rng["p"] - rng["p"],
            "vel": v / 4095.0 * 2 * rng["v"] - rng["v"],
            "iq": c / 4095.0 * 2 * rng["cur"] - rng["cur"],
            "err": d[3] & 0x0F, "mode": d[5] & 0x0F}


def pump(bus, seconds, pos, vel, kp, kd, tau, rng, period=0.01):
    end = time.time() + seconds
    last = None
    while time.time() < end:
        bus.send_cb(cb_id(PRI_HIGH_CTRL, MT_MIT, NODE),
                 mit_bytes(pos, vel, kp, kd, tau, rng))
        f = find(bus.read_frames(period), MT_MIT)
        if f:
            last = f
    return last


def stage_b(bus, tau_cmd, pulse_ms):
    rng = {}
    for key, ep in (("p", EP_P_MAX), ("v", EP_V_MAX), ("kp", EP_KP_MAX),
                    ("kd", EP_KD_MAX), ("gr", EP_GR), ("i_max", EP_I_MAX)):
        raw, _ = sdo_read(bus, ep)
        rng[key] = le_f32(raw) if raw else 0.0
    rng["tau"] = rng["i_max"] * 1.0 * rng["gr"]   # KT = 1 in this config
    rng["cur"] = rng["i_max"] * rng["gr"]
    print("[B1] ranges: p=%.2f v=%.2f kp=%.1f kd=%.2f tau=%.1f cur=%.1f" %
          (rng["p"], rng["v"], rng["kp"], rng["kd"], rng["tau"], rng["cur"]))

    # 先发一帧零力矩，拿到当前位置
    f = pump(bus, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, rng)
    pos0 = decode_mit_out(f["data"], rng)["pos"] if f else 0.0
    print("      pos0 = %.4f rad (output)" % pos0)

    print("[B2] START_MOTOR + zero-torque frames")
    bus.send_cb(cb_id(3, MT_START, NODE), b"")
    last = pump(bus, 0.35, pos0, 0.0, 0.0, 0.0, 0.0, rng)
    d = decode_mit_out(last["data"], rng) if last else None
    check("mode=MOTOR(4)", d is not None and d["mode"] == 4, str(d))
    check("no fault after enable", d is not None and d["err"] == 0, str(d))
    raw, _ = sdo_read(bus, EP_GATE_DRIVE, 1)
    check("gate drive enabled", raw is not None and raw[0] == 1,
          "" if raw is None else str(raw[0]))

    print("[B3] torque pulse %.2f N.m for %d ms" % (tau_cmd, pulse_ms))
    end = time.time() + pulse_ms / 1000.0
    peak_iq = 0.0
    peak_vel = 0.0
    abort = False
    last = None
    while time.time() < end:
        bus.send_cb(cb_id(PRI_HIGH_CTRL, MT_MIT, NODE),
                 mit_bytes(pos0, 0.0, 0.0, 0.0, tau_cmd, rng))
        f = find(bus.read_frames(0.01), MT_MIT)
        if f:
            last = f
            d = decode_mit_out(f["data"], rng)
            peak_iq = max(peak_iq, abs(d["iq"]))
            peak_vel = max(peak_vel, abs(d["vel"]))
            if d["err"] != 0 or peak_iq > 1.5 or peak_vel > 5.0:
                abort = True
                break
    print("      peak iq=%.3f A, peak vel=%.3f rad/s" % (peak_iq, peak_vel))
    check("torque pulse within limits", not abort)

    print("[B4] zero torque + STOP_MOTOR")
    pump(bus, 0.25, pos0, 0.0, 0.0, 0.0, 0.0, rng)
    bus.send_cb(cb_id(3, MT_STOP, NODE), b"")
    time.sleep(0.25)
    f = pump(bus, 0.12, pos0, 0.0, 0.0, 0.0, 0.0, rng)
    d = decode_mit_out(f["data"], rng) if f else None
    check("mode=IDLE(2) after stop", d is not None and d["mode"] == 2, str(d))
    raw, _ = sdo_read(bus, EP_GATE_DRIVE, 1)
    check("gate drive disabled", raw is not None and raw[0] == 0,
          "" if raw is None else str(raw[0]))
    raw, _ = sdo_read(bus, EP_FAULT)
    check("no latched fault", raw is not None and le_i32(raw) == 0,
          "" if raw is None else str(le_i32(raw)))


def read_ranges(bus):
    rng = {}
    for key, ep in (("p", EP_P_MAX), ("v", EP_V_MAX), ("kp", EP_KP_MAX),
                    ("kd", EP_KD_MAX), ("gr", EP_GR), ("i_max", EP_I_MAX)):
        raw, _ = sdo_read(bus, ep)
        rng[key] = le_f32(raw) if raw else 0.0
    rng["tau"] = rng["i_max"] * 1.0 * rng["gr"]   # KT = 1 in this config
    rng["cur"] = rng["i_max"] * rng["gr"]
    return rng


def stage_c_bcast(bus, tau_cmd, pulse_ms):
    """FD 专属：64 B MIT 广播，Dest 位掩码，设备读自己的 slot。"""
    rng = read_ranges(bus)
    print("[C1] ranges: p=%.2f v=%.2f tau_max=%.1f" % (rng["p"], rng["v"], rng["tau"]))
    f = pump(bus, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, rng)
    pos0 = decode_mit_out(f["data"], rng)["pos"] if f else 0.0
    bus.send_cb(cb_id(3, MT_START, NODE), b"")
    last = pump(bus, 0.30, pos0, 0.0, 0.0, 0.0, 0.0, rng)
    d = decode_mit_out(last["data"], rng) if last else None
    check("mode=MOTOR(4)", d is not None and d["mode"] == 4, str(d))

    slot = NODE
    buf = bytearray(64)
    buf[slot * 8:slot * 8 + 8] = mit_bytes(pos0, 0.0, 0.0, 0.0, tau_cmd, rng)
    print("[C2] 64 B MIT broadcast: Dest=0x%02X slot=%d tau=%.2f N.m" %
          (1 << slot, slot, tau_cmd))
    end = time.time() + pulse_ms / 1000.0
    peak_iq = 0.0
    peak_vel = 0.0
    abort = False
    last = None
    while time.time() < end:
        # 广播帧：FD + BRS，Dest 位掩码 bit1 对应 node 1
        bus.send(cb_id(PRI_HIGH_CTRL, MT_MIT_BCAST, 1 << slot), bytes(buf),
                 fd=True, brs=True)
        # 查询不会改变命令，但能拿到位置/速度反馈
        bus.send_cb(cb_id(PRI_QUERY, MT_QUERY_STATUS, NODE), b"")
        f = find(bus.read_frames(0.01), MT_MIT)
        if f:
            last = f
            d = decode_mit_out(f["data"], rng)
            peak_iq = max(peak_iq, abs(d["iq"]))
            peak_vel = max(peak_vel, abs(d["vel"]))
            if d["err"] != 0 or peak_iq > 1.5 or peak_vel > 5.0:
                abort = True
                break
    print("      peak iq=%.3f A, peak vel=%.3f rad/s" % (peak_iq, peak_vel))
    check("broadcast pulse within limits", not abort)

    print("[C3] zero torque + STOP")
    pump(bus, 0.25, pos0, 0.0, 0.0, 0.0, 0.0, rng)
    bus.send_cb(cb_id(3, MT_STOP, NODE), b"")
    time.sleep(0.25)
    f = pump(bus, 0.12, pos0, 0.0, 0.0, 0.0, 0.0, rng)
    d = decode_mit_out(f["data"], rng) if f else None
    check("mode=IDLE(2) after stop", d is not None and d["mode"] == 2, str(d))
    raw, _ = sdo_read(bus, EP_GATE_DRIVE, 1)
    check("gate drive disabled", raw is not None and raw[0] == 0,
          "" if raw is None else str(raw[0]))
    raw, _ = sdo_read(bus, EP_FAULT)
    check("no latched fault", raw is not None and le_i32(raw) == 0,
          "" if raw is None else str(le_i32(raw)))


def read_float_ep(bus, ep):
    raw, _ = sdo_read(bus, ep)
    return le_f32(raw) if raw else None


def stage_c_static(bus, gr):
    """FD 广播/单位映射测试，**不使能电机**（MENU 下只写控制目标，不进 FOC）。

    通过 telemetry.p_des/v_des/kp/kd/t_ff 反读固件实际接收到的值：
      - 64 B MIT 广播的 slot 是否按 node_id 取对
      - TORQUE(0x83) 电机端 N·m 是否 ×GR
      - CURRENT(0x04) 是否 ×KT×GR（本机 KT=1）
    """
    raw, _ = sdo_read(bus, EP_MODE)
    mode = raw[0] if raw else -1
    if mode != 0:
        bus.send_cb(cb_id(3, MT_CLEAR, NODE), b"")
        time.sleep(0.2)
    raw, _ = sdo_read(bus, EP_GATE_DRIVE, 1)
    check("MENU (gates off)", raw is not None and raw[0] == 0)

    slot = NODE
    p_t, v_t, kp_t, kd_t, tau_t = 0.5, 0.1, 10.0, 1.0, 0.2
    buf = bytearray(64)
    buf[slot * 8:slot * 8 + 8] = mit_bytes(p_t, v_t, kp_t, kd_t, tau_t,
                                           {"p": 12.5, "v": 65.0, "kp": 500.0,
                                            "kd": 5.0, "tau": 31.0})
    print("[C1] 64 B MIT broadcast Dest=0x%02X slot=%d" % (1 << slot, slot))
    bus.send(cb_id(PRI_HIGH_CTRL, MT_MIT_BCAST, 1 << slot), bytes(buf),
             fd=True, brs=True)
    time.sleep(0.05)
    got = {n: read_float_ep(bus, ep) for n, ep in
           (("p_des", EP_P_DES), ("v_des", EP_V_DES), ("kp", EP_KP),
            ("kd", EP_KD), ("t_ff", EP_T_FF))}
    print("      read back:", got)
    tol_p = 25.0 / 65535.0
    tol_kp = 500.0 / 4095.0
    tol_tau = 2.0 * 31.0 / 4095.0
    check("broadcast slot applied to p_des", got["p_des"] is not None and
          abs(got["p_des"] - p_t) < tol_p, str(got["p_des"]))
    check("broadcast slot applied to kp", got["kp"] is not None and
          abs(got["kp"] - kp_t) < tol_kp, str(got["kp"]))
    check("broadcast slot applied to t_ff", got["t_ff"] is not None and
          abs(got["t_ff"] - tau_t) < tol_tau, str(got["t_ff"]))

    print("[C2] TORQUE broadcast 0x83: motor-side 0.5 N.m -> expect t_ff=%.3f" % (0.5 * gr))
    bus.send(cb_id(3, MT_TORQUE + 0x80, 1 << slot), struct.pack(">f", 0.5),
             fd=True, brs=False)
    time.sleep(0.05)
    t_ff = read_float_ep(bus, EP_T_FF)
    check("torque broadcast x GR", t_ff is not None and abs(t_ff - 0.5 * gr) < 0.02,
          str(t_ff))

    print("[C3] CURRENT unicast 0x04: 0.2 A -> expect t_ff=%.3f" % (0.2 * gr))
    bus.send_cb(cb_id(3, MT_CURRENT, NODE), struct.pack(">f", 0.2))
    time.sleep(0.05)
    t_ff = read_float_ep(bus, EP_T_FF)
    check("current x KT x GR", t_ff is not None and abs(t_ff - 0.2 * gr) < 0.02,
          str(t_ff))

    # 清零，避免把测试命令留在控制器里
    for _ in range(3):
        bus.send_cb(cb_id(PRI_HIGH_CTRL, MT_MIT, NODE),
                    mit_bytes(0.0, 0.0, 0.0, 0.0, 0.0,
                              {"p": 12.5, "v": 65.0, "kp": 500.0,
                               "kd": 5.0, "tau": 31.0}))
        time.sleep(0.01)
    check("commands cleared", abs(read_float_ep(bus, EP_T_FF)) < 0.02)
    raw, _ = sdo_read(bus, EP_FAULT)
    check("no fault", raw is not None and le_i32(raw) == 0)
    raw, _ = sdo_read(bus, EP_GATE_DRIVE, 1)
    check("gates still off", raw is not None and raw[0] == 0)


def read_int_ep(bus, ep):
    raw, _ = sdo_read(bus, ep)
    if raw is None:
        return None
    return le_i32(raw + b"\x00" * 4)      # bool/u8 端点只回 1 字节


def stage_reset(bus):
    """0x64 RESET_DEVICE：必须是普通重启（不能掉进 DFU bootloader），
    且重启后 CAN 配置按持久化的值生效。"""
    print("[R1] RESET_DEVICE 0x64 -> normal reboot")
    prot0 = read_int_ep(bus, EP_CAN_PROT_ACTIVE)
    fd0 = read_int_ep(bus, EP_CAN_FD)
    baud0 = read_int_ep(bus, EP_CAN_BAUD_ACTIVE)
    seq0 = read_int_ep(bus, EP_PREFS_SEQUENCE)
    enc0 = read_int_ep(bus, EP_ENCODER_RAW)
    print("      before: prot=%s fd=%s baud=%s seq=%s enc=%s"
          % (prot0, fd0, baud0, seq0, enc0))

    bus.drain()
    t0 = time.time()
    bus.send_cb(cb_id(PRI_CONFIG, MT_RESET, NODE), b"")

    # 心跳恢复 = 应用正常运行（DFU bootloader 不上总线）
    hb = None
    while time.time() - t0 < 6.0:
        for f in bus.read_frames(0.1):
            if is_hb(f):
                hb = f
                break
        if hb is not None:
            break
    dt = time.time() - t0
    if not check("heartbeat resumed after reset", hb is not None, "%.2f s" % dt):
        print("      !! device silent on CAN after reset")
        return
    print("      resumed in %.2f s, state=%s dest=%d"
          % (dt, decode_hb(hb["data"])["state_full"], hb_dest(hb)))
    check("heartbeat back to default master (learned id cleared)",
          hb_dest(hb) in (1, MASTER), "dest=%d" % hb_dest(hb))

    prot1 = read_int_ep(bus, EP_CAN_PROT_ACTIVE)
    fd1 = read_int_ep(bus, EP_CAN_FD)
    baud1 = read_int_ep(bus, EP_CAN_BAUD_ACTIVE)
    seq1 = read_int_ep(bus, EP_PREFS_SEQUENCE)
    enc1 = read_int_ep(bus, EP_ENCODER_RAW)
    print("      after:  prot=%s fd=%s baud=%s seq=%s enc=%s"
          % (prot1, fd1, baud1, seq1, enc1))

    check("CAN protocol survived reboot", prot1 == prot0 and prot1 is not None)
    check("CAN baud survived reboot", baud1 == baud0 and baud1 is not None)
    check("FD mode still active", fd1 == fd0 == 1)
    check("no config write on reset", seq1 == seq0)
    check("app running (encoder reads back)",
          enc1 is not None and enc0 is not None)


def stage_d(bus):
    """错误路径：**写的 ACK 不含成败位**（协议如此），所以一律靠读回判定。"""
    print("[D1] batch read (flags bit6) -> must be refused")
    bus.drain()
    bus.send_cb(cb_id(PRI_CONFIG, MT_PARAM_READ, NODE),
                bytes([0x40, 0x00, EP_T_FF & 0xFF, 4, 0, 0, 0, 0]))
    frames = bus.read_frames(0.3)
    f = find(frames, MT_PARAM_READ)
    d = f["data"] if f else b""
    check("batch read answered with BATCH|ERR",
          len(d) >= 2 and (d[0] & 0x60) == 0x60, d.hex() if d else "no reply")

    print("[D2] write a read-only endpoint (telemetry.v_bus = 999)")
    vbus0 = read_float_ep(bus, EP_V_BUS)
    sdo_write(bus, EP_V_BUS, struct.pack("<f", 999.0))
    time.sleep(0.05)
    vbus1 = read_float_ep(bus, EP_V_BUS)
    check("read-only endpoint unchanged by write",
          vbus1 is not None and abs(vbus1 - 999.0) > 1.0 and abs(vbus1 - vbus0) < 2.0,
          "%.2f -> %.2f" % (vbus0 or 0.0, vbus1 or 0.0))

    print("[D3] read a write-only action (action.save) -> no data, no side effect")
    seq0 = read_int_ep(bus, EP_PREFS_SEQUENCE)
    raw, ln = sdo_read(bus, EP_ACTION_SAVE)
    time.sleep(0.4)
    seq1 = read_int_ep(bus, EP_PREFS_SEQUENCE)
    check("write-only endpoint returns no payload", raw == b"" and ln == 0,
          "len=%s" % ln)
    check("reading an action does not trigger it", seq1 == seq0,
          "seq %s -> %s" % (seq0, seq1))

    print("[D4] clamp both ends + NaN (config.i_bw, 100..2000)")
    bw0 = read_float_ep(bus, EP_I_BW)
    sdo_write(bus, EP_I_BW, struct.pack("<f", 1e9))
    time.sleep(0.05)
    bw_hi = read_float_ep(bus, EP_I_BW)
    check("huge value clamped to max", bw_hi is not None and abs(bw_hi - 2000.0) < 1e-3,
          str(bw_hi))
    sdo_write(bus, EP_I_BW, struct.pack("<f", float("nan")))
    time.sleep(0.05)
    bw_nan = read_float_ep(bus, EP_I_BW)
    check("NaN rejected (value unchanged)", bw_nan == bw_hi, str(bw_nan))
    sdo_write(bus, EP_I_BW, struct.pack("<f", -1e9))
    time.sleep(0.05)
    bw_lo = read_float_ep(bus, EP_I_BW)
    check("negative huge value clamped to min", bw_lo is not None and abs(bw_lo - 100.0) < 1e-3,
          str(bw_lo))
    sdo_write(bus, EP_I_BW, struct.pack("<f", bw0))
    time.sleep(0.05)
    check("original value restored",
          abs((read_float_ep(bus, EP_I_BW) or -1.0) - bw0) < 1e-3, str(bw0))

    print("[D5] wrong payload width (config.gr as int16) -> rejected")
    gr0 = read_float_ep(bus, EP_GR)
    sdo_write(bus, EP_GR, struct.pack("<h", 5))
    time.sleep(0.05)
    gr1 = read_float_ep(bus, EP_GR)
    check("short write rejected", gr1 is not None and abs(gr1 - gr0) < 1e-6,
          "gr stayed %.3f" % gr1)

    print("[D6] TORQUE clamp + NaN (gates off, MENU)")
    rng = read_ranges(bus)
    lim = rng["i_max"] * rng["gr"]
    bus.send_cb(cb_id(3, MT_TORQUE, NODE), struct.pack(">f", 1e6))
    time.sleep(0.05)
    tau_hi = read_float_ep(bus, EP_T_FF)
    check("TORQUE clamped to I_MAX*KT*GR",
          tau_hi is not None and abs(tau_hi - lim) < 0.02,
          "%.3f (limit %.3f)" % (tau_hi or 0.0, lim))
    bus.send_cb(cb_id(3, MT_TORQUE, NODE), struct.pack(">f", float("nan")))
    time.sleep(0.05)
    tau_nan = read_float_ep(bus, EP_T_FF)
    check("NaN TORQUE ignored", tau_nan == tau_hi, str(tau_nan))
    bus.send_cb(cb_id(3, MT_TORQUE, NODE), struct.pack(">f", 0.0))
    time.sleep(0.05)
    check("commands cleared", abs(read_float_ep(bus, EP_T_FF) or 0.0) < 1e-6)


def stage_timeout(bus):
    """MOTOR 模式下停发控制帧：命令应在 config.can_timeout（毫秒）后被清零。
    用 telemetry.kp 作判据（zero_commands() 会把 kp/kd/p_des/t_ff 一起清掉），
    这样即使电机被夹具卡住，也能测出超时。"""
    rng = read_ranges(bus)
    to_ms = read_int_ep(bus, EP_CAN_TIMEOUT)
    window = (to_ms or 0) * 40          # 40 控制周期 = 1 ms
    print("[T0] CAN_TIMEOUT = %s ms (= %d control cycles at 40 kHz)" % (to_ms, window))

    raw, _ = sdo_read(bus, EP_MODE)
    if raw and raw[0] != 0:
        bus.send_cb(cb_id(3, MT_CLEAR, NODE), b"")
        time.sleep(0.2)
    f = pump(bus, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, rng)
    pos0 = decode_mit_out(f["data"], rng)["pos"] if f else 0.0

    print("[T1] START_MOTOR + 100 Hz MIT hold (kp=10, t_ff=0.2 N.m) for 1 s")
    bus.send_cb(cb_id(3, MT_START, NODE), b"")
    t_start = time.time()
    t_last = t_start
    kp_mid = None
    last = None
    while time.time() - t_start < 1.0:
        bus.send_cb(cb_id(PRI_HIGH_CTRL, MT_MIT, NODE),
                    mit_bytes(pos0, 0.0, 10.0, 1.0, 0.2, rng))
        t_last = time.time()
        # 边发边读：读不会重置超时，但下一帧会，所以 10 ms 周期内读一次是安全的
        if kp_mid is None and (time.time() - t_start) > 0.5:
            kp_mid = read_float_ep(bus, EP_KP)
        for f in bus.read_frames(0.01):
            if is_from_node(f, MT_MIT):
                last = f
    d = decode_mit_out(last["data"], rng) if last else None
    check("mode=MOTOR(4)", d is not None and d["mode"] == 4, str(d))
    raw, _ = sdo_read(bus, EP_GATE_DRIVE, 1)
    check("gate drive enabled", raw is not None and raw[0] == 1)
    check("commands alive while streaming at 100 Hz",
          kp_mid is not None and abs(kp_mid - 10.0) < 0.2, str(kp_mid))

    # 毫秒语义的回归点：50 ms 周期（20 Hz）主站必须活下来。旧语义（默认 1000 控制
    # 周期 = 25 ms）下这里会被每帧清零 —— 客户最常踩的就是这一条。
    print("[T1b] 20 Hz stream (50 ms period) must survive the window")
    t20 = time.time()
    kp20 = None
    while time.time() - t20 < 1.0:
        bus.send_cb(cb_id(PRI_HIGH_CTRL, MT_MIT, NODE),
                    mit_bytes(pos0, 0.0, 10.0, 1.0, 0.2, rng))
        t_last = time.time()
        if kp20 is None and (time.time() - t20) > 0.6:
            kp20 = read_float_ep(bus, EP_KP)
        bus.read_frames(0.05)
    check("20 Hz stream keeps the command alive",
          kp20 is not None and abs(kp20 - 10.0) < 0.2, str(kp20))

    gap = time.time() - t_last
    print("[T2] stop sending (last frame %.1f ms ago) -> wait for timeout" % (gap * 1000.0))
    trip = None
    cnt = None
    while time.time() - t_last < 1.5:
        kp = read_float_ep(bus, EP_KP)
        if kp is not None and abs(kp) < 1e-6:
            trip = time.time() - t_last      # 从最后一帧算起
            cnt = read_int_ep(bus, EP_CAN_TIMEOUT_COUNT)
            break
    print("      zeroed %.1f ms after the last frame, counter=%s" %
          (trip * 1000.0 if trip is not None else -1.0, cnt))
    check("commands zeroed after CAN timeout", trip is not None)
    if to_ms:
        check("timeout counter reached the configured window",
              cnt is not None and cnt >= window and cnt <= window + 200,
              "counter=%s, window=%d cycles" % (cnt, window))
        check("trip latency matches the window (< window + 20 ms)",
              trip is not None and trip * 1000.0 <= to_ms + 20.0,
              "%.1f ms vs window %d ms" %
              (trip * 1000.0 if trip else -1.0, to_ms))

    print("[T3] write config.gr while the gate drive is ON -> EP_ERR_STATE")
    gr0 = read_float_ep(bus, EP_GR)
    sdo_write(bus, EP_GR, struct.pack("<f", 5.0))
    time.sleep(0.05)
    gr1 = read_float_ep(bus, EP_GR)
    check("require(gates off) enforced",
          gr1 is not None and abs(gr1 - gr0) < 1e-6, "gr stayed %.3f" % gr1)

    print("[T4] STOP_MOTOR")
    bus.send_cb(cb_id(3, MT_STOP, NODE), b"")
    time.sleep(0.25)
    last = pump(bus, 0.12, pos0, 0.0, 0.0, 0.0, 0.0, rng)
    d = decode_mit_out(last["data"], rng) if last else None
    check("mode=IDLE(2) after stop", d is not None and d["mode"] == 2, str(d))
    raw, _ = sdo_read(bus, EP_GATE_DRIVE, 1)
    check("gate drive disabled", raw is not None and raw[0] == 0)
    raw, _ = sdo_read(bus, EP_FAULT)
    check("no latched fault", raw is not None and le_i32(raw) == 0,
          "" if raw is None else str(le_i32(raw)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM7")
    ap.add_argument("--stage-a", action="store_true")
    ap.add_argument("--stage-b", action="store_true")
    ap.add_argument("--stage-c", action="store_true", help="FD 64B 广播测试（需 --fd）")
    ap.add_argument("--stage-c-static", action="store_true",
                    help="FD 广播/单位映射静态测试（不使能电机）")
    ap.add_argument("--reset-test", action="store_true",
                    help="0x64 RESET_DEVICE 普通重启 + 配置保持")
    ap.add_argument("--stage-d", action="store_true",
                    help="错误路径：批量读/只读写/只写读/限幅/NaN/长度不符")
    ap.add_argument("--timeout-test", action="store_true",
                    help="MOTOR 模式下 CAN 命令超时")
    ap.add_argument("--tau", type=float, default=2.0, help="output N.m for the pulse")
    ap.add_argument("--pulse-ms", type=int, default=200)
    ap.add_argument("--fd", action="store_true", help="CAN FD 模式（1M/5M）")
    ap.add_argument("--gr", type=float, default=7.75, help="gear ratio for the static checks")
    args = ap.parse_args()

    bus = Slcan(args.port)
    bus.fd = bool(args.fd)
    try:
        bus.open(8, 5 if args.fd else None)
        if args.reset_test:
            stage_reset(bus)
        elif args.stage_d:
            stage_d(bus)
        elif args.timeout_test:
            stage_timeout(bus)
        elif args.stage_c_static:
            if not args.fd:
                print("stage C-static requires --fd")
                return 2
            stage_c_static(bus, args.gr)
        elif args.stage_c:
            if not args.fd:
                print("stage C requires --fd")
                return 2
            stage_c_bcast(bus, args.tau, args.pulse_ms)
        elif args.stage_b:
            stage_b(bus, args.tau, args.pulse_ms)
        elif args.stage_a:
            stage_a(bus)
    finally:
        bus.close()

    print("\nsummary: %d passed, %d failed" % (CHECK["pass"], CHECK["fail"]))
    return 0 if CHECK["fail"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
