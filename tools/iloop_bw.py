"""电流环带宽极限：台架扫频 / 阶跃测量（配合固件里的 scope.* endpoint）。

固件侧（Core/Src/scope.c）在 40 kHz 中断里注入激励、自己采样、自己做锁相解调，
所以这里只需要按频点发命令、读回几个浮点数 —— 不受上位机采样率限制。
（endpoint 轮询只有 100~300 Hz，直接读 telemetry.i_q 测不到 762 Hz 的穿越点。）

三种用法：

  # 单个 i_bw 下的频率响应（Bode）：d 轴 1 A 偏置 ± 0.5 A
  python tools/iloop_bw.py --port COM7 bode --axis d --bias 1 --amp 0.5

  # 换几档 i_bw 各扫一遍，直接给出"穿越频率/相位裕度 vs i_bw"的极限表
  python tools/iloop_bw.py --port COM7 sweep --axis d

  # 阶跃响应（可把 1024 点原始波形导成 csv，只有 --port 能导）
  python tools/iloop_bw.py --port COM7 step --bw 1000 --dump step_1000.csv

CAN（出货板没有 USB 座）把 --port 换成 --can --interface slcan --channel COM12。
测量原理与判据见 docs/DEV.md 的"电流环带宽的实测"一节。

⚠ 台架注意
  * 转子要锁住（或用 d 轴注入：表贴机 i_d 不出转矩，机械扰动最小）。q 轴注入
    时看 telemetry.scope_motion —— 超过几度电角度就说明转子/齿隙在动，这一次
    的数据不能用（脚本会告警）。
  * bias 不要取 0（小电流区死区畸变最大），建议 1~2 A；bias+amp 必须 ≤ i_max。
  * 每档 i_bw 只改增益、不需要重启（写 config.i_bw 会立刻重算 PI）。
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mc_ep import Device  # noqa: E402

FS = 40000.0            # 控制/采样频率（hw_config.h 的 DT = 25 us）
NBIN = 2048             # scope.c 的锁相累积窗
FBIN = FS / NBIN        # 19.53125 Hz：固件把频率量化到这个栅格上
# dq 电压指令 → 物理相电压（hw_config.h 的 V_PER_CMD；scope 的 v 通道存的是指令单位）
V_PER_CMD = 0.5 / (0.7 * 0.57735026919)      # ≈ 1.2368 V / 指令单位

ST_IDLE, ST_PREROLL, ST_RUN, ST_DONE, ST_ERR = 0, 1, 2, 3, 4
STATE_NAMES = {ST_IDLE: "空闲", ST_PREROLL: "预滚", ST_RUN: "采集中",
               ST_DONE: "完成", ST_ERR: "中止"}


# --------------------------------------------------------------------------
# 设备与频点
# --------------------------------------------------------------------------
def open_device(args):
    kwargs = {}
    if args.can:
        kwargs.update(can_interface=args.interface, can_channel=args.channel,
                      can_bitrate=args.bitrate, can_fd=args.fd, can_id=args.can_id)
    elif args.usb:
        kwargs.update(use_usb=True)
    else:
        if not args.port:
            raise SystemExit("需要 --port COMx（CDC）、--usb（vendor 接口）或 --can")
        kwargs.update(port=args.port, baud=args.baud)
    dev = Device(timeout=args.timeout, **kwargs)
    dev.fetch_descriptor()
    if not dev.read("config.pre_calibrated"):
        raise SystemExit("config.pre_calibrated 是 false：先跑 action.calibrate")
    return dev


def quantize(f):
    """把频率吸到固件的栅格上（f = k·fs/2048 ⇒ 窗口里整数个周期，锁相没有泄漏）。"""
    k = max(1, int(round(f / FBIN)))
    return k, k * FBIN


def freq_list(f0, f1, points):
    if points <= 1 or f1 <= f0:
        return [quantize(f0)]
    lo, hi = math.log(f0), math.log(f1)
    out, last = [], None
    for i in range(points):
        k, fr = quantize(math.exp(lo + (hi - lo) * i / (points - 1)))
        if k != last:
            out.append((k, fr))
            last = k
    return out


# --------------------------------------------------------------------------
# 数学（与 scope_law.h 的 sl_closed_to_open / sl_margins 同一套算法）
# --------------------------------------------------------------------------
def loop_from_closed(mag, phase_deg):
    """闭环 T → 开环 L = T/(1-T)。返回 (|L|, ∠L[deg])。"""
    ar = math.radians(phase_deg)
    tr, ti = mag * math.cos(ar), mag * math.sin(ar)
    dr, di = 1.0 - tr, -ti
    den = dr * dr + di * di
    if den < 1e-12:
        return 1e6, phase_deg
    lr = (tr * dr + ti * di) / den
    li = (ti * dr - tr * di) / den
    return math.hypot(lr, li), math.degrees(math.atan2(li, lr))


def margins(freqs, mags, phases):
    """log|L|-log f 上插值找 |L|=1 与 ∠L=-180°，返回 (fc, PM, GM)。

    相位先解卷绕：atan2 给的是 (−180, +180]，不处理的话 −180° 穿越永远找不到、
    跨卷绕点插值还会把 PM 算成 180~350°（真值其实是负的、已经失稳）。
    """
    ph = list(phases)
    off = 0.0
    for i in range(1, len(ph)):
        d = phases[i] - phases[i - 1]        # 原始（未加偏置）的相邻差
        if d > 180.0:
            off -= 360.0
        elif d < -180.0:
            off += 360.0
        ph[i] = phases[i] + off
    fc = pm = gm = None
    for i in range(len(freqs) - 1):
        m0, m1 = mags[i], mags[i + 1]
        if m0 <= 0 or m1 <= 0 or (m0 - 1.0) * (m1 - 1.0) > 0.0:
            continue
        l0, l1 = math.log(m0), math.log(m1)
        t = (l0 / (l0 - l1)) if l0 != l1 else 0.0
        fc = math.exp(math.log(freqs[i]) + t * (math.log(freqs[i + 1]) - math.log(freqs[i])))
        pm = 180.0 + ph[i] + t * (ph[i + 1] - ph[i])
        break
    for i in range(len(freqs) - 1):
        if ph[i] > -180.0 >= ph[i + 1]:
            t = (ph[i] + 180.0) / (ph[i] - ph[i + 1])
            db0 = 20 * math.log10(mags[i]) if mags[i] > 1e-6 else -120.0
            db1 = 20 * math.log10(mags[i + 1]) if mags[i + 1] > 1e-6 else -120.0
            gm = -(db0 + t * (db1 - db0))
            break
    return fc, pm, gm


# --------------------------------------------------------------------------
# 采集
# --------------------------------------------------------------------------
def run_point(dev, k, timeout=3.0):
    """注入一个频点并等结果。返回 dict（频率、幅值、相移、噪声、转角）。"""
    dev.write("scope.freq", k * FBIN)
    dev.call("action.scope_start")
    t0 = time.time()
    st = ST_IDLE
    while time.time() - t0 < timeout:
        st = dev.read("telemetry.scope_state")
        if st == ST_DONE:
            break
        if st == ST_ERR:
            raise SystemExit("scope 中止：多半是门极已关，或 |bias|+amp 超过当前 i_max")
        time.sleep(0.004)
    else:
        raise SystemExit("scope 超时：状态 %s" % STATE_NAMES.get(st, st))

    return dict(f=dev.read("telemetry.scope_freq"),
                cmd=dev.read("telemetry.scope_cmd_amp"),
                fb=dev.read("telemetry.scope_fb_amp"),
                ph=dev.read("telemetry.scope_phase"),
                noise=dev.read("telemetry.scope_noise"),
                motion=dev.read("telemetry.scope_motion"),
                clip=int(dev.read("telemetry.scope_clip")))


def sweep_freqs(dev, pts, axis, amp, verbose=True):
    """扫一组频点，返回 (rows, fc, pm, gm)；rows = [(f, |T|, ∠T, |L|, ∠L, 噪声, 转角)]。"""
    if verbose:
        print("   f[Hz]   |T|[dB]  ∠T[°]   |L|[dB]  ∠L[°]   噪声[A]  转角[°]")
    rows = []
    for k, _fr in pts:
        r = run_point(dev, k)
        if r["cmd"] <= 1e-4:
            if verbose:
                print("  %7.1f   参考幅值为 0（被 i_max 限幅？）" % r["f"])
            continue
        mag = r["fb"] / r["cmd"]
        l_mag, l_ph = loop_from_closed(mag, r["ph"])
        rows.append((r["f"], mag, r["ph"], l_mag, l_ph, r["noise"], r["motion"]))
        if verbose:
            warn = "  <-- 参考被 i_max 削掉，此点作废" if r["clip"] else ""
            if axis == "q" and r["motion"] > 5.0:
                warn += "  <-- 转子在动，此点作废"
            print("  %7.1f  %+7.2f %+7.1f  %+7.2f %+7.1f  %8.4f %7.2f%s"
                  % (r["f"], 20 * math.log10(mag) if mag > 0 else -99.0, r["ph"],
                     20 * math.log10(l_mag) if l_mag > 0 else -99.0, l_ph,
                     r["noise"], r["motion"], warn))

    fc, pm, gm = margins([r[0] for r in rows], [r[3] for r in rows], [r[4] for r in rows])
    note = unwrap_note([r[4] for r in rows])
    if verbose and note:
        print("  " + note)
    return rows, fc, pm, gm


def apply_scope_cfg(dev, args):
    dev.write("scope.axis", 1 if args.axis == "d" else 0)
    dev.write("scope.bias", args.bias)
    dev.write("scope.amp", args.amp)
    i_max = dev.read("config.i_max")
    if abs(args.bias) + args.amp > i_max:
        raise SystemExit("bias+amp = %.2f A 超过 config.i_max = %.2f A"
                         % (abs(args.bias) + args.amp, i_max))


def fmt(v, spec="%.1f", unit="", none="—"):
    return (spec % v + unit) if v is not None else none


def unwrap_note(phases):
    """相邻频点相位跳变 > 180° 时解卷绕不可靠（分不清"真跳了一大截"还是跨卷绕）。"""
    worst = 0.0
    for i in range(1, len(phases)):
        d = abs(phases[i] - phases[i - 1])
        if d > worst:
            worst = d
    if worst > 180.0:
        return ("注意：相邻频点相位跳变最大 %.0f° > 180°，解卷绕可能不可靠"
                "（加大 --points 或缩小扫频范围）" % worst)
    return None


def fc_note(rows, fc):
    """没找到穿越时，区分"增益还没到 0 dB（扫频上限太低）"和"真的没有穿越"。"""
    if fc is not None or not rows:
        return ""
    return "（扫频上限太低，|L| 还是正的：加大 --f1）" if rows[-1][3] > 1.0 else ""


# --------------------------------------------------------------------------
# 三个子命令
# --------------------------------------------------------------------------
def cmd_bode(dev, args):
    apply_scope_cfg(dev, args)
    dev.write("scope.wave", 2)
    if args.bw is not None:
        dev.write("config.i_bw", args.bw)
        time.sleep(0.05)
    print("i_bw=%.0f Hz  axis=%s  bias=%.2f A  amp=%.2f A"
          % (dev.read("config.i_bw"), args.axis, args.bias, args.amp))
    pts = freq_list(args.f0, args.f1, args.points)
    rows, fc, pm, gm = sweep_freqs(dev, pts, args.axis, args.amp)
    print("\n穿越 %s   相位裕度 %s   增益裕度 %s"
          % (fmt(fc, "%.0f", " Hz"), fmt(pm, "%.1f", "°"), fmt(gm, "%.1f", " dB")))
    note = fc_note(rows, fc)
    if note:
        print(note)
    if args.csv:
        with open(args.csv, "w", encoding="utf-8") as fh:
            fh.write("f_hz,mag_t,phase_t_deg,mag_l,phase_l_deg,noise_a,motion_deg\n")
            for r in rows:
                fh.write("%.3f,%.6g,%.4g,%.6g,%.4g,%.4g,%.4g\n" % r)
        print("已写入 %s" % args.csv)


def cmd_sweep(dev, args):
    apply_scope_cfg(dev, args)
    dev.write("scope.wave", 2)
    pts = freq_list(args.f0, args.f1, args.points)
    print("axis=%s  bias=%.2f A  amp=%.2f A  %d 个频点（%.0f..%.0f Hz）"
          % (args.axis, args.bias, args.amp, len(pts), args.f0, args.f1))
    print(" %-6s %-10s %-9s %-9s %s" % ("i_bw", "实际穿越", "相位裕度", "增益裕度", "备注"))
    out = []
    for bw in args.bw:
        dev.write("config.i_bw", bw)
        time.sleep(0.05)
        rows, fc, pm, gm = sweep_freqs(dev, pts, args.axis, args.amp, verbose=args.verbose)
        note = "q 轴（转子务必锁住）" if args.axis == "q" else ""
        if pm is not None and pm < 45.0:
            note += " PM<45°，已到极限"
        note += fc_note(rows, fc)
        print(" %-6.0f %-10s %-9s %-9s %s"
              % (dev.read("config.i_bw"), fmt(fc, "%.0f", " Hz"), fmt(pm, "%.1f", "°"),
                 fmt(gm, "%.1f", " dB"), note))
        out.append((bw, fc, pm, gm))
    print("\n判据：实测穿越约为 0.79×i_bw（2 拍 ADC→占空比延迟 + 离散化的固有损失），"
          "PM 掉到 45° 以下就别再用这一档。")
    return out


def cmd_step(dev, args):
    apply_scope_cfg(dev, args)
    dev.write("scope.wave", 1)
    if args.bw is not None:
        dev.write("config.i_bw", args.bw)
        time.sleep(0.05)
    dev.call("action.scope_start")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        if dev.read("telemetry.scope_state") == ST_DONE:
            break
        time.sleep(0.004)
    else:
        raise SystemExit("scope 超时（状态 %d）" % dev.read("telemetry.scope_state"))

    t90 = dev.read("telemetry.scope_t90")
    st = dev.read("telemetry.scope_settle")
    print("i_bw=%.0f  axis=%s  %.2f → %.2f A"
          % (dev.read("config.i_bw"), args.axis, args.bias, args.bias + args.amp))
    print("  峰值 %.3f A   超调 %.1f %%   上升(10-90%%) %s   稳定(±2%%) %s   转角 %.2f°"
          % (dev.read("telemetry.scope_peak"), dev.read("telemetry.scope_overshoot"),
             ("%.3f ms" % (t90 * 1e3)) if t90 >= 0 else "未找到",
             ("%.2f ms" % (st * 1e3)) if st >= 0 else "未稳定",
             dev.read("telemetry.scope_motion")))

    if args.dump:
        if not args.port:
            print("!! 只有 --port（CDC）能导波形：原文由 myprintf 打在 CDC 上，"
                  "vendor 接口和 CAN 都没有这条通道")
            return
        dump_cdc(dev, args.dump)


def dump_cdc(dev, path):
    """action.scope_dump 之后从 CDC 抓十六进制波形（与 endpoint 帧共用一条数据流）。

    固件在同一个主循环里先发 endpoint 应答（endpoint_serial_poll），再由
    run_fsm_slow → scope_poll 打印，所以 dev.call() 返回之后读到的就是干净的行。
    """
    ser = dev.link.io.ser
    ser.reset_input_buffer()
    dev.call("action.scope_dump")
    t0 = time.time()
    header, rows = None, []
    while time.time() - t0 < 10.0:
        line = ser.readline().decode("ascii", "replace").strip()
        if not line:
            continue
        i = line.find("SCOPE ")
        if i >= 0:
            header = [int(x) for x in line[i:].split()[1:]]
            continue
        if line.endswith("END"):
            break
        if header and all(c in "0123456789ABCDEF" for c in line):
            for j in range(0, len(line) - 15, 16):
                raw = [int(line[j + q:j + q + 4], 16) for q in range(0, 16, 4)]
                rows.append([(v - 65536 if v >= 32768 else v) for v in raw])
    if not rows:
        print("!! 没收到波形（USB CDC 是否 CONFIGURED？）")
        return

    with open(path, "w", encoding="utf-8") as fh:
        fh.write("t_us,cmd_a,fb_a,v_phase_v,theta_elec_rad\n")
        for i, r in enumerate(rows):
            fh.write("%.1f,%.3f,%.3f,%.3f,%.4f\n"
                     % (i * 1e6 / FS, r[0] / 1000.0, r[1] / 1000.0,
                        r[2] / 256.0 * V_PER_CMD, r[3] / 4096.0))
    print("波形 %d 点（阶跃在第 %s 点）已写入 %s"
          % (len(rows), header[2] if header and len(header) > 2 else "?", path))


# --------------------------------------------------------------------------
def main():
    # Windows 控制台默认 GBK：把不能编码的字符替换掉，别让一次 print 打断整轮扫频
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="电流环带宽极限：扫频/阶跃测量")
    ap.add_argument("--port", help="CDC 串口（开发板）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--usb", action="store_true", help="vendor interface（需要 WinUSB）")
    ap.add_argument("--can", action="store_true", help="CYBERBEAST SDO")
    ap.add_argument("--interface", default="slcan", help="slcan|pcan|socketcan|…")
    ap.add_argument("--channel", help="CAN 适配器的串口/通道名")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--fd", action="store_true", help="CAN FD")
    ap.add_argument("--can-id", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=1.0)
    ap.add_argument("--motor-wait", type=float, default=0.3,
                    help="进电机模式后等多久（秒，默认 0.3）")

    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p, with_bw=True):
        p.add_argument("--axis", choices=["q", "d"], default="d",
                       help="被测轴（默认 d：表贴机 i_d 不出转矩，机械扰动最小）")
        p.add_argument("--bias", type=float, default=1.0, help="直流偏置 A（默认 1.0）")
        p.add_argument("--amp", type=float, default=0.5, help="激励峰值 A（默认 0.5）")
        p.add_argument("--f0", type=float, default=20.0)
        p.add_argument("--f1", type=float, default=2500.0)
        p.add_argument("--points", type=int, default=18)
        if with_bw:
            p.add_argument("--bw", type=float, help="先写 config.i_bw（Hz）")

    p = sub.add_parser("bode", help="单个 i_bw 的频率响应")
    common(p)
    p.add_argument("--csv", help="把 Bode 结果写成 csv")

    p = sub.add_parser("sweep", help="多个 i_bw 的极限对比")
    common(p, with_bw=False)
    p.add_argument("--bw", type=str, default="500,1000,1500,2000,2500",
                   help="逗号分隔的 i_bw 列表（默认 500,1000,1500,2000,2500）")
    p.add_argument("--verbose", action="store_true", help="每档都打印完整 Bode 表")

    p = sub.add_parser("step", help="阶跃响应")
    common(p)
    p.add_argument("--dump", help="把 1024 点原始波形写成 csv（需要 --port）")

    args = ap.parse_args()
    if args.cmd == "sweep":
        args.bw = [float(x) for x in args.bw.split(",")]

    dev = open_device(args)
    try:
        dev.call("action.clear_fault")
        time.sleep(0.05)
        dev.call("action.motor_mode")
        time.sleep(args.motor_wait)
        for name in ("command.kp", "command.kd", "command.p_des", "command.v_des",
                     "command.t_ff", "command.mode"):
            dev.write(name, 0.0)
        if dev.read("config.cogging_en"):
            print("!! config.cogging_en 是打开的：scope 会覆盖它的前馈，不影响测量")
        if args.cmd == "bode":
            cmd_bode(dev, args)
        elif args.cmd == "sweep":
            cmd_sweep(dev, args)
        else:
            cmd_step(dev, args)
    finally:
        try:
            dev.call("action.scope_stop")
        except Exception:
            pass
        try:
            dev.write("command.t_ff", 0.0)
            dev.call("action.idle_mode")
            print("fault=%d" % dev.read("telemetry.fault"))
        finally:
            dev.close()


if __name__ == "__main__":
    main()
