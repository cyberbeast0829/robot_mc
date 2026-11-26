#!/usr/bin/env python3
"""MTPA 台架检查（CAN / CYBERBEAST，slcan 适配器）。

用法：
    python tools/mtpa_test.py --channel COM7 status
    python tools/mtpa_test.py --channel COM7 verify  [--torque 0.3]
    python tools/mtpa_test.py --channel COM7 budget  [--torque 0.2] [--rounds 2]

子命令：
    status  读固件版本、标定参数、凸极指数与 `telemetry.mtpa_gain`（判断值不值得开）
    verify  正负转矩各做一次 MTPA 关/开对比：Id 符号与幅值、kt_eff、**参考转矩守恒**、
            |I| 比（会临时关 dither/齿槽并在结束时复原，不写 flash）
    budget  MTPA 关/开交替采样 `timing.seg_fsm_max` / `total_max`（ISR 预算 A/B）
    peak    峰值转矩口径：临时把 i_max 降到 2 A（饱和力矩 ≈0.55 N·m，关节是自由的
            就不要调大），在**同一 |I|** 下比 MTPA 开/关的参考转矩（= mtpa_gain）

依赖：python-can 不需要；只用 `tools/rmc` 的 Device。

⚠ 这些测试都在**阻抗模式 kp=kd=0** 下做（纯力矩），自由关节每个脉冲都会加速：
先看 `telemetry.position` 有没有余量，脉冲尽量短（`--ms`），跑完确认关节停住。
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "rmc"))
from rmc.device import connect  # noqa: E402

TEL = ("i_d_des", "i_q_des", "mtpa_id", "current_kt_eff", "torque", "i_d", "i_q")
TIM = ("seg_fsm_max", "total_max", "total_avg", "overrun")


def read_many(dev, names):
    return {n: dev.read(n) for n in names}


def hold(dev, torque, secs, timing=False, period=0.08):
    """按住 t_ff（每 20 ms 刷新租约），期间按 period 采样遥测。"""
    t0 = time.time()
    acc = {k: [] for k in TEL}
    last = 0.0
    while time.time() - t0 < secs:
        dev.write("command.t_ff", torque)
        if time.time() - last > period:
            last = time.time()
            for k in TEL:
                acc[k].append(dev.read("telemetry." + k))
        time.sleep(0.02)
    if timing:
        for k in TIM:
            acc[k] = [dev.read("timing." + k)]
    return {k: (sum(v) / len(v) if v else float("nan")) for k, v in acc.items()}


def cmd_status(dev, args):
    kt = dev.read("config.kt")
    pp = dev.read("config.ppairs")
    ld = dev.read("config.l_d")
    lq = dev.read("config.l_q")
    imax = dev.read("config.i_max")
    lam = kt / (1.5 * pp) if pp > 0 else float("nan")
    xi = imax * abs(ld - lq) / lam if lam > 0 else float("nan")
    print("fw           %d.%d.%d (unreleased=%d)"
          % (dev.read("system.fw_version_major"),
             dev.read("system.fw_version_minor"),
             dev.read("system.fw_version_revision"),
             dev.read("system.fw_version_unreleased")))
    print("kt           %.6f N·m/A   ppairs %.0f   λ %.6f mWb"
          % (kt, pp, lam * 1e3))
    print("Ld/Lq        %.2f / %.2f µH   ΔL %.2f µH"
          % (ld * 1e6, lq * 1e6, (ld - lq) * 1e6))
    print("i_max        %.2f A" % imax)
    print("凸极指数 ξ   %.4f" % xi)
    print("mtpa_gain    %.6f  (+%.2f%% 每安培转矩)"
          % (dev.read("telemetry.mtpa_gain"),
             100.0 * (dev.read("telemetry.mtpa_gain") - 1.0)))
    print("enable_mtpa  %s   mtpa_id_frac %.2f"
          % (dev.read("config.enable_mtpa"), dev.read("config.mtpa_id_frac")))
    if dev.read("telemetry.mtpa_gain") < 1.002:
        print("=> 凸极太弱，开 MTPA 意义不大（收益 < 0.2%）")


def cmd_verify(dev, args):
    kt = dev.read("config.kt")
    saved = {k: dev.read(k) for k in
             ("config.can_timeout", "config.dither_amp", "config.cogging_en",
              "config.enable_mtpa")}
    try:
        dev.write("config.can_timeout", 1000)     # 别让 100 ms 租约把命令清掉
        dev.write("config.dither_amp", 0.0)
        dev.write("config.cogging_en", 0)
        dev.write("config.enable_mtpa", 0)
        if dev.read("config.pre_calibrated") is not True:
            print("设备未标定，先 action.calibrate")
            return 2
        dev.call("action.motor_mode")
        time.sleep(0.3)
        ok = True
        for tau in (args.torque, -args.torque):
            hold(dev, tau, 0.3)
            dev.write("config.enable_mtpa", 0)
            off = hold(dev, tau, args.secs)
            dev.write("config.enable_mtpa", 1)
            time.sleep(0.05)
            on = hold(dev, tau, args.secs)
            ref_off = kt * off["i_q_des"]
            ref_on = on["current_kt_eff"] * on["i_q_des"]
            mag_off = abs(off["i_q_des"])
            mag_on = math.hypot(on["i_d_des"], on["i_q_des"])
            d_ref = 100.0 * (ref_on / ref_off - 1.0) if ref_off else float("nan")
            ratio = mag_on / mag_off if mag_off else float("nan")
            print("\n== t_ff=%+0.3f N·m ==" % tau)
            print("  off i_d=%+8.5f i_q=%+8.5f kt_eff=%.6f |I|=%.5f ref=%.6f"
                  % (off["i_d_des"], off["i_q_des"], off["current_kt_eff"],
                     mag_off, ref_off))
            print("  on  i_d=%+8.5f i_q=%+8.5f kt_eff=%.6f |I|=%.5f ref=%.6f mtpa_id=%+.5f"
                  % (on["i_d_des"], on["i_q_des"], on["current_kt_eff"],
                     mag_on, ref_on, on["mtpa_id"]))
            print("  -> ref tau %+0.3f%%  |I| ratio %.6f  Id %s"
                  % (d_ref, ratio,
                     "OK(<0)" if on["i_d_des"] < 0 else "**WRONG SIGN**"))
            ok = ok and (abs(d_ref) < 0.5) and (on["i_d_des"] < 0)
        return 0 if ok else 3
    finally:
        try:
            dev.write("command.t_ff", 0.0)
            dev.call("action.idle_mode")
            for k, v in saved.items():
                dev.write(k, v)
        except Exception as e:  # noqa: BLE001
            print("cleanup failed: %s" % e)


def cmd_budget(dev, args):
    saved = {k: dev.read(k) for k in ("config.can_timeout", "config.enable_mtpa")}

    def case(label):
        res = [None] * args.rounds
        for i in range(args.rounds):
            dev.call("action.reset_timing")
            time.sleep(0.25)
            res[i] = hold(dev, args.torque, args.secs, timing=True)
        out = {k: sum(r[k] for r in res) / len(res) for k in TEL + TIM}
        print("%-8s seg_fsm=%4.0f tmax=%4.0f tavg=%4.0f ovr=%5.1f  "
              "(i_d_des=%+.5f kt_eff=%.6f)"
              % (label, out["seg_fsm_max"], out["total_max"], out["total_avg"],
                 out["overrun"], out["i_d_des"], out["current_kt_eff"]))
        return out

    try:
        dev.write("config.can_timeout", 1000)
        dev.write("config.enable_mtpa", 0)
        dev.call("action.motor_mode")
        time.sleep(0.3)
        hold(dev, args.torque, 0.4)
        off = case("MTPA=0")
        dev.write("config.enable_mtpa", 1)
        time.sleep(0.1)
        on = case("MTPA=1")
        dev.write("config.enable_mtpa", 0)
        time.sleep(0.1)
        off2 = case("MTPA=0b")
        base = 0.5 * (off["seg_fsm_max"] + off2["seg_fsm_max"])
        print("=> seg_fsm +%.0f ticks (%+.1f%%), total_max %0.0f (limit 6000)"
              % (on["seg_fsm_max"] - base,
                 100.0 * (on["seg_fsm_max"] / base - 1.0), on["total_max"]))
        return 0 if on["total_max"] < 6000 else 4
    finally:
        try:
            dev.write("command.t_ff", 0.0)
            dev.call("action.idle_mode")
            for k, v in saved.items():
                dev.write(k, v)
        except Exception as e:  # noqa: BLE001
            print("cleanup failed: %s" % e)


def cmd_peak(dev, args):
    """峰值转矩口径：临时把 i_max 降到安全值，让"电流饱和"落在很小的力矩上，
    然后看 MTPA 开/关在**同一 |I|** 下的参考转矩差（= mtpa_gain）。"""
    kt = dev.read("config.kt")
    gr = dev.read("config.gr")
    itest = args.imax
    saved = {k: dev.read(k) for k in
             ("config.can_timeout", "config.i_max", "config.dither_amp",
              "config.cogging_en", "config.enable_mtpa")}
    tau_sat = kt*itest*gr
    tau_cmd = 1.3*tau_sat
    print("i_test=%.2f A -> 饱和力矩 %.3f N·m，指令 %.3f N·m，脉冲 %.0f ms"
          % (itest, tau_sat, tau_cmd, args.ms))
    try:
        dev.write("config.can_timeout", 1500)
        dev.write("config.dither_amp", 0.0)
        dev.write("config.cogging_en", 0)
        dev.write("config.i_max", itest)
        dev.call("action.motor_mode")
        time.sleep(0.2)
        res = {}
        for mtpa in (0, 1):
            dev.write("config.enable_mtpa", mtpa)
            time.sleep(0.05)
            t0 = time.time()
            rows = []
            while (time.time() - t0)*1000.0 < args.ms:
                dev.write("command.t_ff", tau_cmd)
                rows.append((dev.read("telemetry.i_d_des"),
                             dev.read("telemetry.i_q_des"),
                             dev.read("telemetry.current_kt_eff")))
            dev.write("command.t_ff", 0.0)
            if not rows:
                print("MTPA=%d 没采到样本" % mtpa)
                return 5
            idm = sum(r[0] for r in rows)/len(rows)
            iqm = sum(r[1] for r in rows)/len(rows)
            ktm = sum(r[2] for r in rows)/len(rows)
            mag = math.hypot(idm, iqm)
            res[mtpa] = (idm, iqm, ktm, mag, ktm*iqm)
            print("  MTPA=%d  i_d_des=%+8.5f  i_q_des=%+8.5f  kt_eff=%.6f  "
                  "|I|=%.5f  ref_tau=%.6f" % (mtpa, idm, iqm, ktm, mag, ktm*iqm))
            time.sleep(0.3)          # 让它停一下，别把速度攒起来
        g = res[1][4]/res[0][4]
        print("  -> 同 |I| 下参考转矩比 %.6f（mtpa_gain = %.6f）"
              % (g, dev.read("telemetry.mtpa_gain")))
        ok = (abs(res[1][3] - res[0][3]) < 1e-3*itest      # |I| 相同
              and g > res[0][4] and g <= 1.01)              # 转矩只增、幅度合理
        return 0 if ok else 3
    finally:
        try:
            dev.write("command.t_ff", 0.0)
            dev.call("action.idle_mode")
            for k, v in saved.items():
                dev.write(k, v)
        except Exception as e:  # noqa: BLE001
            print("cleanup failed: %s" % e)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("status", "verify", "budget", "peak"))
    ap.add_argument("--channel", default=None, help="slcan 串口，如 COM7（默认自动找）")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--torque", type=float, default=0.3, help="输出端 N·m（默认 0.3）")
    ap.add_argument("--imax", type=float, default=2.0,
                    help="peak 用的临时 i_max（A，默认 2：饱和力矩约 0.55 N·m）")
    ap.add_argument("--ms", type=float, default=200.0, help="peak 脉冲长度（ms）")
    ap.add_argument("--secs", type=float, default=0.8)
    ap.add_argument("--rounds", type=int, default=2)
    args = ap.parse_args()

    dev = connect(interface="slcan", channel=args.channel, bitrate=args.bitrate)
    try:
        return {"status": cmd_status, "verify": cmd_verify,
                "budget": cmd_budget, "peak": cmd_peak}[args.cmd](dev, args) or 0
    finally:
        dev.close()


if __name__ == "__main__":
    sys.exit(main())
