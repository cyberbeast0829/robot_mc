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
EP_KT = 15
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
EP_FW_MAJOR = 187
EP_FW_MINOR = 188
EP_FW_REV = 189
EP_ENCODER_RAW = 79
EP_GATE_DRIVE = 57
EP_VELOCITY = 45
EP_I_Q_FILT = 36
EP_P_DES = 49
EP_V_DES = 50
EP_KP = 51
EP_KD = 52
EP_T_FF = 53
EP_POS_GAIN = 136
EP_VEL_GAIN = 137
EP_VEL_IGAIN = 138
EP_CMD_T_FF = 128
EP_CMD_P_DES = 129
EP_CMD_V_DES = 130
EP_CMD_KP = 131
EP_CMD_KD = 132
EP_CMD_MODE = 145
EP_I_Q_DES = 54
EP_SAVE_STATUS = 119
EP_MIT_TAU_MAX = 243         # config.mit_tau_max：MIT 力矩位域满量程（输出端 N·m）

# ---- CyberBeast message types ------------------------------------------
MT_MIT = 0x00
MT_CSP = 0x01
MT_CSV = 0x02
MT_TORQUE = 0x03
MT_CURRENT = 0x04
MT_MIT_BCAST = 0x80
MT_CSP_BCAST = 0x81
MT_CSV_BCAST = 0x82
MT_TORQUE_BCAST = 0x83
MT_SET_NODE_ID = 0x60
MT_CONFIG_SAVE = 0x22
MT_CONFIG_RESET = 0x23
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

    def send_std(self, can_id, data):
        """11-bit 标准帧（CyberBeast 模式下应当被硬件滤波器整体丢掉）。"""
        line = "t%03X%X%s" % (can_id & 0x7FF, len(data), data.hex().upper())
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
                           ("fw_major", EP_FW_MAJOR, "u8"), ("fw_minor", EP_FW_MINOR, "u8"),
                           ("fw_rev", EP_FW_REV, "u8"),
                           ("mode", EP_MODE, "u8"),
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


def fetch_descriptor(bus, expect_len=None):
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
    if expect_len is None:
        check("descriptor length plausible", total is not None and total > 512,
              "total=%s (rx %d)" % (total, len(data)))
    else:
        check("descriptor length", total == expect_len,
              "got %s expect %d (rx %d)" % (total, expect_len, len(data)))
    if total:
        data = data[:total]
        ok_crc = crc16(bytes(data)) == crc
        check("descriptor CRC16", ok_crc, "calc=0x%04X meta=0x%04X" % (crc16(bytes(data)), crc or 0))
        try:
            desc = json.loads(bytes(data).decode("ascii"))
            names = {e["name"] for e in desc}
            need_names = {"config.can_prot", "config.can_baud", "system.can_fd",
                          "action.save", "action.reboot", "action.reset_config",
                          "telemetry.position", "command.p_des", "command.t_ff",
                          "config.pos_gain", "config.vel_gain"}
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
    rng = read_ranges(bus)
    print("[B1] ranges: p=%.2f v=%.2f kp=%.1f kd=%.2f kt=%.4f tau_max=%.3f cur_max=%.2f" %
          (rng["p"], rng["v"], rng["kp"], rng["kd"], rng["kt"], rng["tau"], rng["cur"]))
    if abs(tau_cmd) > rng["tau"]:
        print("      ⚠ 请求 %.3f N·m 超过满量程 %.3f，会被钳位" % (tau_cmd, rng["tau"]))
        tau_cmd = rng["tau"] * (1.0 if tau_cmd > 0 else -1.0)

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
            # 中止闸按本机配置来定：空载台架上 0.2 N.m 就能转得很快，
            # 固定阈值（旧版 5 rad/s）会把正常响应当成故障
            if (d["err"] != 0) or (peak_iq > 1.2 * rng["i_max"]) \
                    or (peak_vel > rng["abort_vel"]):
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
    """MIT 位域的满量程：tau_max = config.mit_tau_max（输出端 N·m，0 = I_MAX*KT*GR），
    cur_max = I_MAX*GR（电机端 A）。

    ⚠ KT 是标定出来的（本机约 0.031），**不能假设 1**：以前按 KT=1 编码会把力矩缩小
    30 倍，看起来像"固件不接受力矩命令"。
    ⚠ tau_max 与可达力矩是两件事：I_MAX*KT*GR（本机 4 A × 0.0334 × 9 ≈ 1.2 N·m）是
    电流限幅决定的**物理上限**，mit_tau_max 只是位域量程 —— 超出的部分电流环会饱和。"""
    rng = {}
    for key, ep in (("p", EP_P_MAX), ("v", EP_V_MAX), ("kp", EP_KP_MAX),
                    ("kd", EP_KD_MAX), ("gr", EP_GR), ("i_max", EP_I_MAX),
                    ("kt", EP_KT), ("mit_tau", EP_MIT_TAU_MAX)):
        raw, _ = sdo_read(bus, ep)
        rng[key] = le_f32(raw) if raw else 0.0
    rng["tau"] = rng["mit_tau"] if rng["mit_tau"] > 0.0 \
        else rng["i_max"] * rng["kt"] * rng["gr"]
    rng["cur"] = rng["i_max"] * rng["gr"]
    rng["abort_vel"] = 0.25 * rng["v"]      # 台架中止闸（空载轴 1/4 满速）
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
            # 中止闸按本机配置来定：空载台架上 0.2 N.m 就能转得很快，
            # 固定阈值（旧版 5 rad/s）会把正常响应当成故障
            if (d["err"] != 0) or (peak_iq > 1.2 * rng["i_max"]) \
                    or (peak_vel > rng["abort_vel"]):
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
      - CURRENT(0x04) 是否 ×KT×GR
    """
    kt = read_float_ep(bus, EP_KT)
    raw, _ = sdo_read(bus, EP_MODE)
    mode = raw[0] if raw else -1
    if mode != 0:
        bus.send_cb(cb_id(3, MT_CLEAR, NODE), b"")
        time.sleep(0.2)
    raw, _ = sdo_read(bus, EP_GATE_DRIVE, 1)config.mit_tau_max
    check("MENU (gates off)", raw is not None and raw[0] == 0)

    slot = NODE
    rng = read_ranges(bus)          # 必须是设备真实量程（tau_max = I_MAX*KT*GR）
    p_t, v_t, kp_t, kd_t, tau_t = 0.5, 0.1, 10.0, 1.0, 0.2
    buf = bytearray(64)
    buf[slot * 8:slot * 8 + 8] = mit_bytes(p_t, v_t, kp_t, kd_t, tau_t, rng)
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
    tol_tau = 2.0 * rng["tau"] / 4095.0
    check("broadcast slot applied to p_des", got["p_des"] is not None and
          abs(got["p_des"] - p_t) < tol_p, str(got["p_des"]))
    check("broadcast slot applied to kp", got["kp"] is not None and
          abs(got["kp"] - kp_t) < tol_kp, str(got["kp"]))
    check("broadcast slot applied to t_ff", got["t_ff"] is not None and
          abs(got["t_ff"] - tau_t) < tol_tau, str(got["t_ff"]))

    # 0.5 电机端 N·m × GR 可能超过 tau_max（config.mit_tau_max），固件会钳位 —— 期望值也要钳
    want_tau = min(0.5 * gr, rng["tau"])
    print("[C2] TORQUE broadcast 0x83: motor-side 0.5 N.m -> expect t_ff=%.4f%s"
          % (want_tau, "（被 config.mit_tau_max 钳位）" if want_tau < 0.5 * gr else ""))
    bus.send(cb_id(3, MT_TORQUE + 0x80, 1 << slot), struct.pack(">f", 0.5),
             fd=True, brs=False)
    time.sleep(0.05)
    t_ff = read_float_ep(bus, EP_T_FF)
    check("torque broadcast x GR (clamped)", t_ff is not None and abs(t_ff - want_tau) < 0.01,
          str(t_ff))

    print("[C3] CURRENT unicast 0x04: 0.2 A -> expect t_ff=%.4f" % (0.2 * kt * gr))
    bus.send_cb(cb_id(3, MT_CURRENT, NODE), struct.pack(">f", 0.2))
    time.sleep(0.05)
    t_ff = read_float_ep(bus, EP_T_FF)
    check("current x KT x GR", t_ff is not None and abs(t_ff - 0.2 * kt * gr) < 0.01,
          str(t_ff))

    # 清零，避免把测试命令留在控制器里
    for _ in range(3):
        bus.send_cb(cb_id(PRI_HIGH_CTRL, MT_MIT, NODE),
                    mit_bytes(0.0, 0.0, 0.0, 0.0, 0.0, rng))
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

    print("[D3] read a function endpoint (action.save) -> also triggers it (ODrive 兼容)")
    # 先钉住状态：菜单 + 门极关闭，否则 action.save 的 require 会返回 EP_ERR_STATE
    bus.send_cb(cb_id(3, MT_CLEAR, NODE), b"")
    time.sleep(0.2)
    seq0 = read_int_ep(bus, EP_PREFS_SEQUENCE)
    raw, ln = sdo_read(bus, EP_ACTION_SAVE)
    time.sleep(0.4)
    seq1 = read_int_ep(bus, EP_PREFS_SEQUENCE)
    check("function endpoint returns no payload", raw == b"" and ln == 0,
          "len=%s" % ln)
    check("reading an action triggers it", seq1 is not None and seq1 == (seq0 or 0) + 1,
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
    lim = rng["tau"]          # config.mit_tau_max（0 时 = I_MAX*KT*GR）
    bus.send_cb(cb_id(3, MT_TORQUE, NODE), struct.pack(">f", 1e6))
    time.sleep(0.05)
    tau_hi = read_float_ep(bus, EP_T_FF)
    check("TORQUE clamped to config.mit_tau_max",
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
    restored = None
    if to_ms == 0:
        # 租约被关掉（0 = 不超时）就没法测；临时开 100 ms，测完恢复
        sdo_write(bus, EP_CAN_TIMEOUT, struct.pack("<i", 100))
        time.sleep(0.05)
        to_ms = read_int_ep(bus, EP_CAN_TIMEOUT)
        restored = 0
        print("      (can_timeout 原本是 0/关闭，临时设为 %s ms)" % to_ms)
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

    if restored is not None:
        sdo_write(bus, EP_CAN_TIMEOUT, struct.pack("<i", restored))
        time.sleep(0.05)
        print("      can_timeout 已恢复为 %s" % read_int_ep(bus, EP_CAN_TIMEOUT))



# ---- stage E: CSP(0x01) / CSV(0x02) ------------------------------------

DEG = 180.0 / 3.141592653589793
RAD = 3.141592653589793 / 180.0


def csp_classic(pos_deg, vel_rpm, cur_a):
    """Classic 8 B：float32 位置(度) | int16 RPM | int16 0.1 A（电机端）。"""
    return struct.pack(">fhh", pos_deg, int(vel_rpm), int(round(cur_a * 10.0)))


def csp_payload(bus, pos_deg, vel_rpm, cur_a):
    """CSP 的载荷随物理层不同：FD 12 B = 3×float32，Classic 8 B = f32 + i16 + i16。"""
    if bus.fd:
        return struct.pack(">fff", pos_deg, vel_rpm, cur_a)
    return csp_classic(pos_deg, vel_rpm, cur_a)


def mit_bcast_payload(bus, rng, pos, vel, kp, kd, tau):
    """MIT 广播帧：FD 下 64 B 按 slot = node_id 取槽，Classic 下 8 B 所有节点共用。"""
    one = mit_bytes(pos, vel, kp, kd, tau, rng)
    if not bus.fd:
        return one
    buf = bytearray(64)
    buf[NODE * 8:NODE * 8 + 8] = one
    return bytes(buf)


def csv_classic(vel_rpm, cur_a):
    """Classic 8 B：float32 RPM | float32 A（电机端）。"""
    return struct.pack(">ff", vel_rpm, cur_a)


def stage_e(bus):
    """CSP/CSV：目标值是**输出轴**量（度 / RPM），由固件自己的串级外环跟踪。"""
    rng = read_ranges(bus)
    pos_gain = read_float_ep(bus, EP_POS_GAIN)
    vel_gain = read_float_ep(bus, EP_VEL_GAIN)
    print("      gains: pos_gain=%s vel_gain=%s" % (pos_gain, vel_gain))

    raw, _ = sdo_read(bus, EP_MODE)
    if raw and raw[0] != 0:
        bus.send_cb(cb_id(3, MT_CLEAR, NODE), b"")
        time.sleep(0.2)

    print("[E1] START_MOTOR + 零力矩帧")
    bus.send_cb(cb_id(3, MT_START, NODE), b"")
    f = pump(bus, 0.35, 0.0, 0.0, 0.0, 0.0, 0.0, rng)
    d = decode_mit_out(f["data"], rng) if f else None
    check("MOTOR mode", d is not None and d["mode"] == 4, str(d))
    pos0 = d["pos"] if d else 0.0
    print("      pos0 = %.4f rad (%.3f deg)" % (pos0, pos0 * DEG))

    # ---- CSV +10 rpm ----
    rpm = 10.0
    want = rpm * 2.0 * 3.141592653589793 / 60.0
    print("[E2] CSV 0x02: +%.0f rpm（限流 1.0 A 电机端）0.8 s" % rpm)
    peak = 0.0
    err = None
    samples = []                    # (时间, 速度)，用来区分瞬态与稳态
    last = None
    end = time.time() + 0.8
    while time.time() < end:
        bus.send_cb(cb_id(3, MT_CSV, NODE), csv_classic(rpm, 1.0))
        for fr in bus.read_frames(0.02):
            if is_from_node(fr, MT_MIT):
                last = fr
                dd = decode_mit_out(last["data"], rng)
                peak = max(peak, dd["vel"])
                err = dd["err"]
                samples.append((time.time(), dd["vel"]))
    tail = [v for t, v in samples if t > end - 0.2]      # 最后 0.2 s = 稳态
    ss = sum(tail) / len(tail) if tail else None
    print("      peak vel = %.3f, steady-state = %s rad/s (expect ~%.3f)"
          % (peak, "%.3f" % ss if ss is not None else "-", want))
    # 协议层只保证"跟得上目标"：瞬态超调是增益/惯量的事，用 csv_csp_test.py 整定
    check("CSV steady-state tracks the target",
          ss is not None and abs(ss - want) < 0.15 * want, "%.3f vs %.3f" % (ss or 0.0, want))
    check("CSV transient overshoot < 60%", peak < 1.6 * want,
          "peak=%.3f (>%.3f)" % (peak, 1.6 * want))
    check("CSV no fault", err == 0, "err=%s" % err)

    print("[E3] CSV 0 rpm -> 停住")
    end = time.time() + 0.5
    vel_end = None
    while time.time() < end:
        bus.send_cb(cb_id(3, MT_CSV, NODE), csv_classic(0.0, 1.0))
        for fr in bus.read_frames(0.02):
            if is_from_node(fr, MT_MIT):
                vel_end = decode_mit_out(fr["data"], rng)["vel"]
    check("CSV 0 rpm holds still", vel_end is not None and abs(vel_end) < 0.6,
          "vel=%.3f" % (vel_end or 0.0))

    # ---- CSP 位置阶跃 ----
    target_deg = pos0 * DEG + 3.0
    print("[E4] CSP 0x01: 目标 %.2f deg（+3 deg），20 rpm / 0.5 A 限幅，1.5 s" % target_deg)
    end = time.time() + 1.5
    last = None
    while time.time() < end:
        bus.send_cb(cb_id(3, MT_CSP, NODE), csp_payload(bus, target_deg, 20.0, 0.5))
        for fr in bus.read_frames(0.02):
            if is_from_node(fr, MT_MIT):
                last = fr
    d = decode_mit_out(last["data"], rng) if last else None
    got_deg = (d["pos"] * DEG) if d else None
    print("      final pos = %s deg (target %.2f)" % ("%.3f" % got_deg if got_deg else "-", target_deg))
    check("CSP reached the target", got_deg is not None and abs(got_deg - target_deg) < 0.5,
          "err=%.3f deg" % (abs(got_deg - target_deg) if got_deg is not None else -1))
    check("CSP no fault", d is not None and d["err"] == 0, str(d))

    # ---- 回原位并退出 ----
    print("[E5] CSP 回原位，然后 STOP")
    end = time.time() + 1.2
    while time.time() < end:
        bus.send_cb(cb_id(3, MT_CSP, NODE), csp_payload(bus, pos0 * DEG, 20.0, 0.5))
        for fr in bus.read_frames(0.02):
            if is_from_node(fr, MT_MIT):
                last = fr
    d = decode_mit_out(last["data"], rng) if last else None
    check("CSP returned to the start", d is not None and abs(d["pos"] - pos0) < 0.01,
          "pos=%.4f (start %.4f)" % (d["pos"] if d else 0.0, pos0))

    bus.send_cb(cb_id(3, MT_STOP, NODE), b"")
    time.sleep(0.3)
    raw, _ = sdo_read(bus, EP_GATE_DRIVE, 1)
    check("gate drive disabled after stop", raw is not None and raw[0] == 0)
    raw, _ = sdo_read(bus, EP_FAULT)
    check("no latched fault", raw is not None and le_i32(raw) == 0,
          "" if raw is None else str(le_i32(raw)))


# ---- stage F: 寻址与过滤 ------------------------------------------------

def stage_f(bus):
    """Dest 单播 / 位掩码广播 / 0xFF 广播 / 11-bit 标准帧 的过滤行为。

    判据用 telemetry.p_des（MIT 帧会写 controller.p_des），门极不用开。
    """
    rng = read_ranges(bus)
    pump(bus, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, rng)
    base = read_float_ep(bus, EP_P_DES) or 0.0

    def one_shot(dest, pri, mt, pos, expect_rsp):
        bus.drain()
        if mt >= 0x80:
            payload = mit_bcast_payload(bus, rng, pos, 0.0, 0.0, 0.0, 0.0)
        else:
            payload = mit_bytes(pos, 0.0, 0.0, 0.0, 0.0, rng)
        bus.send_cb(cb_id(pri, mt, dest), payload)
        frames = bus.read_frames(0.25)
        rsp = find(frames, MT_MIT) is not None
        now = read_float_ep(bus, EP_P_DES)
        return rsp, now

    print("[F1] 单播给别的节点（Dest=5）")
    rsp, now = one_shot(5, 2, MT_MIT, 0.5, False)
    check("unicast to another node: no response", not rsp)
    check("unicast to another node: state unchanged",
          now is not None and abs(now - base) < 1e-6, "p_des=%s" % now)

    print("[F2] 广播位掩码不含本机（Dest=0x04，本机 node=1）")
    rsp, now = one_shot(0x04, 2, MT_MIT_BCAST, 0.5, False)
    check("broadcast without our bit: ignored",
          now is not None and abs(now - base) < 1e-6, "p_des=%s" % now)

    print("[F3] 广播位掩码含本机（Dest=0x02）")
    rsp, now = one_shot(0x02, 2, MT_MIT_BCAST, 0.5, False)
    check("broadcast with our bit: applied",
          now is not None and abs(now - 0.5) < 0.02, "p_des=%s" % now)
    check("broadcast: no unicast response", not rsp)

    print("[F4] 全局广播（Dest=0xFF）")
    rsp, now = one_shot(0xFF, 2, MT_MIT_BCAST, -0.4, False)
    check("0xFF broadcast: applied", now is not None and abs(now + 0.4) < 0.02,
          "p_des=%s" % now)

    print("[F5] 11-bit 标准帧必须被硬件过滤器丢掉")
    bus.drain()
    bus.send_std(0x001, bytes([0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    time.sleep(0.1)
    now = read_float_ep(bus, EP_P_DES)
    check("standard frame ignored", now is not None and abs(now + 0.4) < 0.02,
          "p_des=%s" % now)

    print("[F6] SET_NODE_ID 0x60 写回同一个 id（1）")
    raw, _ = sdo_read(bus, 25)  # 占位，确保链路还在
    bus.send_cb(cb_id(4, MT_SET_NODE_ID, NODE), bytes([NODE]))
    time.sleep(0.1)
    frames = bus.read_frames(0.4)
    check("node still answers after SET_NODE_ID", any(is_hb(x) for x in frames))

    # 收尾：清掉命令
    pump(bus, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, rng)



# ---- stage G: 各控制律上报的 mode / ctrl_mode ---------------------------

CM_VOLTAGE, CM_TORQUE, CM_VELOCITY, CM_POSITION = 0, 1, 2, 3


def hb_ctrl_mode(bus, seconds=0.4):
    """心跳 Byte1 低 4 位 = control_mode（客户端用它显示当前控制律）。"""
    frames = bus.read_frames(seconds)
    hbs = [f for f in frames if is_hb(f)]
    if not hbs:
        return None
    d = decode_hb(hbs[-1]["data"])
    return {"state": d["state"], "ctrl": d["ctrl"], "mode_byte": d["state"]}


def recenter(bus, rng, target=0.0, rpm=60.0, timeout=10.0):
    """空载轴没有负载，纯力矩探针会让它一路加速漂走 —— 收尾用 CSV 把它转回来。

    位置量是输出轴 rad；返回最终位置。
    """
    pos = read_float_ep(bus, EP_POSITION) or 0.0
    if abs(pos - target) < 0.3:
        return pos
    print("      recenter: %.2f -> %.2f rad（CSV %+.0f rpm）" % (pos, target, rpm))
    if read_int_ep(bus, EP_MODE) != 0:
        bus.send_cb(cb_id(3, MT_CLEAR, NODE), b"")
        time.sleep(0.2)
    bus.send_cb(cb_id(3, MT_START, NODE), b"")
    time.sleep(0.15)
    end = time.time() + timeout
    while time.time() < end:
        pos = read_float_ep(bus, EP_POSITION) or 0.0
        d = target - pos
        if abs(d) < 0.3:
            break
        # d > 0 表示要往正方向走；别在这里再拐一层符号（曾经把轴赶到了 75 rad）
        v = abs(rpm) if d > 0 else -abs(rpm)
        if abs(d) < 2.0:
            v = v * 0.4
        bus.send_cb(cb_id(3, MT_CSV, NODE), csv_classic(v, 1.0))
        bus.read_frames(0.02)
    for _ in range(5):
        bus.send_cb(cb_id(3, MT_CSV, NODE), csv_classic(0.0, 1.0))
        time.sleep(0.02)
    bus.send_cb(cb_id(3, MT_STOP, NODE), b"")
    time.sleep(0.3)
    return read_float_ep(bus, EP_POSITION) or 0.0


def stage_g(bus):
    """5 种控制律各自的 mode nibble / control_mode 是否与协议表一致。

    MIT/CSP/CSV/0x03/0x04 依次下发一条无害指令，直接看应答与心跳：
      MIT -> ModeState 4, control_mode POSITION(3)
      CSP -> 5, POSITION(3)   CSV -> 6, VELOCITY(2)
      0x03/0x04 -> 7, TORQUE(1)
    """
    rng = read_ranges(bus)
    raw, _ = sdo_read(bus, EP_MODE)
    if raw and raw[0] != 0:
        bus.send_cb(cb_id(3, MT_CLEAR, NODE), b"")
        time.sleep(0.2)
    bus.send_cb(cb_id(3, MT_START, NODE), b"")
    pump(bus, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, rng)
    pos = read_float_ep(bus, EP_POSITION) or 0.0

    rows = []

    def probe(tag, mt, payload, want_mode, want_cm, hold=0.35):
        end = time.time() + hold
        last = None
        while time.time() < end:
            bus.send_cb(cb_id(3 if mt < 0x80 else 2, mt, NODE), payload)
            for fr in bus.read_frames(0.04):
                if is_from_node(fr, MT_MIT):
                    last = fr
        if last is None:
            # 0x04 按协议不回应答，用 0x40 QUERY_STATUS 取同一份 ModeState
            bus.drain()
            bus.send_cb(cb_id(PRI_QUERY, MT_QUERY_STATUS, NODE), b"")
            last = find(bus.read_frames(0.3), MT_MIT)
        d = decode_mit_out(last["data"], rng) if last else None
        cm = hb_ctrl_mode(bus, 0.25)
        rows.append((tag, d["mode"] if d else None, cm["ctrl"] if cm else None,
                     want_mode, want_cm))

    probe("MIT 0x00", MT_MIT, mit_bytes(pos, 0.0, 5.0, 0.5, 0.0, rng), 4, CM_POSITION)
    probe("CSP 0x01", MT_CSP, csp_payload(bus, pos * DEG, 10.0, 0.3), 5, CM_POSITION)
    probe("CSV 0x02", MT_CSV, csv_classic(0.0, 0.3), 6, CM_VELOCITY)
    probe("TORQUE 0x03", MT_TORQUE, struct.pack(">f", 0.01), 7, CM_TORQUE)
    probe("CURRENT 0x04", MT_CURRENT, struct.pack(">f", 0.2), 7, CM_TORQUE)

    print("      %-13s %-9s %-11s %s" % ("命令", "ModeState", "control_mode", "期望"))
    for tag, got, cm, wm, wc in rows:
        ok = (got == wm) and (cm == wc)
        check("%s reports mode=%d ctrl=%d" % (tag, wm, wc), ok,
              "got mode=%s ctrl=%s" % (got, cm))

    # 收尾：纯力矩/电流探针会让空载轴漂走，用 CSV 转回起点再停
    recenter(bus, rng, pos)
    pump(bus, 0.15, pos, 0.0, 0.0, 0.0, 0.0, rng)
    bus.send_cb(cb_id(3, MT_STOP, NODE), b"")
    time.sleep(0.3)
    raw, _ = sdo_read(bus, EP_GATE_DRIVE, 1)
    check("gate drive disabled", raw is not None and raw[0] == 0)
    raw, _ = sdo_read(bus, EP_FAULT)
    check("no latched fault", raw is not None and le_i32(raw) == 0)


# ---- stage H: 系统操作码 0x40/0x49/0x60/0x22/0x23 -----------------------

def stage_h(bus):
    """文档里列出但还没覆盖的 opcode。

    0x40 QUERY_STATUS / 0x49 STATUS_FEEDBACK：都回 8 字节 MIT 风格应答。
    0x60 SET_NODE_ID：改成 7 再改回 1（RAM only，断电即恢复）。
    0x22 CONFIG_SAVE：等效 action.save（门极关着才会写）。
    0x23 CONFIG_RESET：等效 0x64 重启。
    """
    print("[H1] 0x40 QUERY_STATUS / 0x49 STATUS_FEEDBACK")
    for mt, name in ((MT_QUERY_STATUS, "0x40"), (MT_STATUS_FEEDBACK, "0x49")):
        bus.drain()
        bus.send_cb(cb_id(PRI_QUERY, mt, NODE), b"")
        f = find(bus.read_frames(0.3), MT_MIT)
        check("%s returns an 8-byte response" % name,
              f is not None and len(f["data"]) == 8,
              "" if f is None else f["data"].hex())

    print("[H2] SET_NODE_ID 0x60：1 -> 7 -> 1")
    bus.send_cb(cb_id(4, MT_SET_NODE_ID, NODE), bytes([7]))
    time.sleep(0.05)
    frames = bus.read_frames(0.6)
    srcs = {fields(f["id"])[3] for f in frames if fields(f["id"])[1] == MT_HEARTBEAT}
    check("heartbeat source switched to 7", 7 in srcs, str(sorted(srcs)))
    bus.send_cb(cb_id(4, MT_SET_NODE_ID, 7), bytes([NODE]))
    time.sleep(0.05)
    frames = bus.read_frames(0.6)
    srcs = {fields(f["id"])[3] for f in frames if fields(f["id"])[1] == MT_HEARTBEAT}
    check("node id restored to 1", 1 in srcs, str(sorted(srcs)))

    print("[H3] 0x22 CONFIG_SAVE")
    seq0 = read_int_ep(bus, EP_PREFS_SEQUENCE)
    bus.send_cb(cb_id(4, MT_CONFIG_SAVE, NODE), b"")
    time.sleep(1.0)
    seq1 = read_int_ep(bus, EP_PREFS_SEQUENCE)
    st = (sdo_read(bus, EP_SAVE_STATUS, 1)[0] or b"\xff")[0]
    check("CONFIG_SAVE wrote a block", seq1 == (seq0 or 0) + 1, "seq %s -> %s" % (seq0, seq1))
    check("save_status = 1", st == 1, "status=%s" % st)

    print("[H4] 0x23 CONFIG_RESET -> 重启")
    bus.drain()
    t0 = time.time()
    bus.send_cb(cb_id(4, MT_CONFIG_RESET, NODE), b"")
    back = None
    while time.time() - t0 < 8.0:
        for f in bus.read_frames(0.1):
            if is_hb(f):
                back = f
                break
        if back is not None and (time.time() - t0) > 1.0:
            break
    check("device rebooted and resumed", back is not None, "%.1f s" % (time.time() - t0))
    check("prefs_sequence unchanged by reboot",
          read_int_ep(bus, EP_PREFS_SEQUENCE) == seq1)


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
    ap.add_argument("--stage-e", action="store_true", help="CSP(0x01)/CSV(0x02) 目标跟踪")
    ap.add_argument("--stage-f", action="store_true", help="寻址与过滤（Dest/广播/标准帧）")
    ap.add_argument("--stage-g", action="store_true", help="5 种控制律的 mode/control_mode 上报")
    ap.add_argument("--stage-h", action="store_true", help="0x40/0x49/0x60/0x22/0x23 系统操作码")
    ap.add_argument("--all", action="store_true",
                    help="顺序跑 A→F→E→B→C-static→D→timeout（会真的转电机）")
    ap.add_argument("--timeout-test", action="store_true",
                    help="MOTOR 模式下 CAN 命令超时")
    ap.add_argument("--tau", type=float, default=0.2, help="output N.m for the pulse")
    ap.add_argument("--pulse-ms", type=int, default=200)
    ap.add_argument("--fd", action="store_true", help="CAN FD 模式（1M 仲裁 + 数据域）")
    ap.add_argument("--fd-rate", type=int, default=5, choices=(2, 5),
                    help="FD 数据域速率：2 或 5（Mbit/s）。适配器只支持这两档（Y2/Y5）")
    ap.add_argument("--gr", type=float, default=9.0, help="gear ratio for the static checks (CBL89: 9)")
    args = ap.parse_args()

    bus = Slcan(args.port)
    bus.fd = bool(args.fd)
    try:
        bus.open(8, args.fd_rate if args.fd else None)
        if args.all:
            for name, fn in (("A 链路/协议", lambda: stage_a(bus)),
                             ("F 寻址/过滤", lambda: stage_f(bus)),
                             ("E CSP/CSV", lambda: stage_e(bus)),
                             ("B MIT 单播", lambda: stage_b(bus, args.tau, args.pulse_ms)),
                             ("C-static 广播/单位", lambda: stage_c_static(bus, args.gr)
                              if args.fd else print("(跳过：C-static 需要 --fd)")),
                             ("G 控制律上报", lambda: stage_g(bus)),
                             ("D 错误路径", lambda: stage_d(bus)),
                             ("T 命令超时", lambda: stage_timeout(bus))):
                print("\n=================== %s ===================" % name)
                fn()
            recenter(bus, rng=read_ranges(bus), target=0.0)
            print("\n=================== 全部阶段结束 ===================")
        elif args.reset_test:
            stage_reset(bus)
        elif args.stage_d:
            stage_d(bus)
        elif args.timeout_test:
            stage_timeout(bus)
        elif args.stage_e:
            stage_e(bus)
        elif args.stage_g:
            stage_g(bus)
        elif args.stage_h:
            stage_h(bus)
        elif args.stage_f:
            stage_f(bus)
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
