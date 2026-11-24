#!/usr/bin/env python3
"""vel_ripple.py — 速度纹波/频谱测量（CSV 恒速 + scope PASSIVE 捕获）

固件侧用 `scope.wave=3`（PASSIVE，只观测不注入）把 raw/count/速度/i_q 按
`scope.decim` 抽取存进 `scope.buf`，再由 `action.scope_dump` 从 USB CDC 以十六进制
打出。CAN 轮询一次 endpoint 只有约 190 Hz，Nyquist 连 100 Hz 都不到，测不了纹波。

用法（控制走 CAN，波形走 USB CDC）：
  python tools/vel_ripple.py --rpm 300 --seconds 3 --decim 5
  python tools/vel_ripple.py --rpm 1 --seconds 4 --decim 40 --fit 10
  python tools/vel_ripple.py --hold          # 不命令转速，只录当前状态

参数：
  --rpm      输出轴转速（RPM），负值反向
  --seconds  先按该转速稳一段时间（期间持续刷新命令租约）
  --decim    scope.decim（1 = 40 kHz，5 = 8 kHz，40 = 1 kHz）；时间基准 = 40000/decim
  --fit      离线最小二乘窗口（样本数，0 = 按 0.5% 噪声自动选）

输出：机械速度（用 raw count 离线重算，不信被测算器）、速度估计、i_q 的
mean/std/pp，以及机械速度/i_q 的主频峰。

⚠ 会真的转电机。`--min-bus` 默认 15 V，低于它拒绝启动。
"""

import argparse
import math
import os
import re
import sys
import time

import numpy as np
import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mc_ep import Device  # noqa: E402

RPM = 2.0 * math.pi / 60.0
GR = 9.0
CPR = 65536.0


def parse_dump(rows):
    """把 'SCOPE' 表头下的十六进制行解析成 (n, ch, fs, data)"""
    hexs = "".join(rows)
    n = len(hexs) // 16
    data = np.zeros((n, 4), dtype=np.int64)
    for i in range(n):
        for c in range(4):
            u = int(hexs[(i * 4 + c) * 4:(i * 4 + c + 1) * 4], 16)
            data[i, c] = u - 65536 if u & 0x8000 else u
    return data


def unwrap16(x):
    inc = (np.diff(x) + 32768) % 65536 - 32768
    return np.concatenate(([0], np.cumsum(inc))).astype(np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rpm", type=float, default=0.0)
    ap.add_argument("--hold", action="store_true", help="不命令转速，只录当前状态")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--decim", type=int, default=5)
    ap.add_argument("--fit", type=int, default=0)
    ap.add_argument("--port", default="COM10", help="USB CDC 端口（波形从这里出）")
    ap.add_argument("--interface", default="slcan")
    ap.add_argument("--channel", default="COM7")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--can-id", type=int, default=1)
    ap.add_argument("--min-bus", type=float, default=15.0)
    ap.add_argument("--gr", type=float, default=GR)
    ap.add_argument("--csv", default="", help="把解析后的 4 通道原始整数存成 CSV 备查")
    a = ap.parse_args()

    dev = Device(can_interface=a.interface, can_channel=a.channel,
                 can_bitrate=a.bitrate, can_id=a.can_id, timeout=1.0)
    dev.fetch_descriptor()
    ser = serial.Serial(a.port, 115200, timeout=0.2)

    def idle():
        try:
            dev.call("action.idle_mode")
        except Exception:
            pass

    def refresh(v):
        # command.* 有 100 ms 租约（config.can_timeout），每个周期都要重写
        dev.write("command.mode", 1)
        dev.write("command.v_des", v)

    v_cmd = a.rpm * RPM
    try:
        idle()
        time.sleep(0.2)
        vbus = dev.read("telemetry.v_bus")
        if dev.read("telemetry.fault"):
            raise SystemExit("REFUSE: 已锁存故障，先 action.idle_mode 清故障")
        if vbus < a.min_bus:
            raise SystemExit("REFUSE: v_bus=%.1f V < %.1f V" % (vbus, a.min_bus))

        if a.rpm or a.hold:
            dev.call("action.motor_mode")
            time.sleep(0.2)
            t0 = time.time()
            while time.time() - t0 < a.seconds:
                refresh(v_cmd)
                time.sleep(0.02)

        dev.write("scope.decim", a.decim)
        dev.write("scope.wave", 3)          # SCOPE_PASSIVE
        refresh(v_cmd)
        dev.call("action.scope_start")
        t0 = time.time()
        while time.time() - t0 < 15.0:
            refresh(v_cmd)                   # 捕获期间租约也要活着
            if dev.read("telemetry.scope_state") == 3:
                break
            time.sleep(0.005)
        st = dev.read("telemetry.scope_state")
        if st != 3:
            raise SystemExit("scope 中止/超时：state=%d" % st)

        ser.reset_input_buffer()
        dev.call("action.scope_dump")
        rows, fs = [], None
        t0 = time.time()
        while time.time() - t0 < 10.0:
            line = ser.readline().decode("ascii", "ignore").strip()
            if not line:
                continue
            i = line.find("SCOPE ")
            if i >= 0:
                fs = int(line[i:].split()[4])      # 表头第 4 个字段 = 有效采样率
                continue
            if line.endswith("END"):
                break
            if fs and re.fullmatch(r"[0-9A-F]+", line):
                rows.append(line)
    finally:
        idle()
        dev.close()
        ser.close()

    if not rows or not fs:
        raise SystemExit("没收到波形（USB CDC 是否 CONNECTED？）")

    data = parse_dump(rows)
    n = len(data)
    dt = 1.0 / fs
    if a.csv:
        np.savetxt(a.csv, data, fmt="%d", delimiter=",")
    count = unwrap16(data[:, 1])
    pll = data[:, 2] / 500.0          # 输出轴 rad/s
    iq = data[:, 3] / 1000.0

    L = a.fit
    if L <= 1:
        v_typ = abs(count[-1] - count[0]) / max(1e-6, (n - 1) * dt) if n > 2 else 1.0
        if v_typ < 1.0:
            v_typ = 1.0
        L = int(round((4.3 * math.sqrt(12) / (0.005 * v_typ) / dt) ** (2.0 / 3.0)))
        L = max(8, min(n // 4, L))
    step = 1
    idx = np.arange(L - 1, n, step)
    tt = np.arange(L) * dt
    tt -= tt.mean()
    win = np.lib.stride_tricks.sliding_window_view(count, L)
    vel = (win[idx - L + 1] @ tt) / float(np.sum(tt * tt))
    t_vel = idx * dt
    vel_rad = vel * 2 * math.pi / CPR

    print("N=%d fs=%d Hz duration=%.3f s  LS window L=%d (%.1f ms, -3dB ~%.0f Hz)"
          % (n, fs, n / fs, L, L * dt * 1e3, 0.44 / (L * dt)))

    def stats(name, x, scale=1.0):
        m, s = x.mean(), x.std()
        print("%-14s mean=%+9.5f (%.3f rpm) std=%8.5f (%.4f rpm) pp=%8.5f std%%=%6.2f"
              % (name, m / scale, m / scale / RPM, s / scale, s / scale / RPM,
                 (x.max() - x.min()) / scale,
                 100 * s / abs(m) if abs(m) > 1e-12 else float("nan")))

    stats("velocity_mech", vel_rad / a.gr)
    stats("velocity_est", pll)
    stats("iq", iq)

    def peaks(x, name, dts, k=6):
        x = np.asarray(x, dtype=float)
        t = np.arange(len(x))
        x = x - np.polyval(np.polyfit(t, x, 1), t)
        w = np.hanning(len(x))
        X = np.abs(np.fft.rfft(x * w)) / max(1e-9, np.sum(w) / 2)
        f = np.fft.rfftfreq(len(x), dts)
        out = []
        for i in np.argsort(X)[::-1]:
            if f[i] < 0.3:
                continue
            if any(abs(f[i] - p) / max(0.4, p) < 0.08 for p, _ in out):
                continue
            out.append((f[i], X[i]))
            if len(out) >= k:
                break
        print("  %s peaks: %s" % (name, ", ".join("%.2fHz:%.5f" % p for p in sorted(out))))

    d_vel = t_vel[1] - t_vel[0] if len(t_vel) > 1 else dt
    peaks(vel_rad / a.gr, "mech", d_vel)
    peaks(iq, "iq", dt)
    if a.rpm:
        print("command %.3f rpm, measured %.3f rpm" % (a.rpm, vel_rad.mean() / a.gr / RPM))

    # 角度域残差：count 已经是 LUT 线性化之后的值，把它对"匀速直线"拟合后剩下的
    # 就是**角度误差**（编码器/DMA 定时/机械偏心），与速度无关。速度纹波若与转速
    # 成正比（振幅/频率 = 常数），源头就在这里
    for name, ch in (("raw", 0), ("count(LUT)", 1)):
        ang = unwrap16(data[:, ch])
        tt = np.arange(n) * dt
        p = np.polyfit(tt, ang, 1)
        res = ang - np.polyval(p, tt)
        deg = 360.0 / CPR
        print("angle resid %-11s std=%6.3f deg pp=%6.3f deg (%.1f counts)"
              % (name, res.std() * deg, (res.max() - res.min()) * deg,
                 res.max() - res.min()))
    revs = abs(count[-1] - count[0]) / CPR
    if revs > 0.2:
        tt = np.arange(n) * dt
        f_rev = revs / (n * dt)
        for name, ch in (("raw", 0), ("count(LUT)", 1)):
            ang = unwrap16(data[:, ch])
            res = ang - np.polyval(np.polyfit(tt, ang, 1), tt)
            w = np.hanning(len(res))
            X = np.abs(np.fft.rfft(res * w)) / max(1e-9, np.sum(w) / 2)
            f = np.fft.rfftfreq(len(res), dt)
            out = [(f[i] / f_rev, X[i] * 360.0 / CPR) for i in np.argsort(X)[::-1][:12]
                   if f[i] > 0.4 * f_rev]
            out.sort()
            print("angle harmonics %-11s (order:deg): %s"
                  % (name, ", ".join("%.2f:%.3f" % o for o in out[:10])))


if __name__ == "__main__":
    main()
