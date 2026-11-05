#!/usr/bin/env python3
"""
齿轮式双编（Vernier 双磁编）台架验证工具。

用 endpoint 协议（tools/mc_ep.py 的 Device）把电机开到若干位置，把固件解出的绝对
圈数（telemetry.gear_abs_turns）和电机侧多圈计数（telemetry.position_motor/2pi）
比一比，检查"断电记忆输出轴位置"是否成立。参考算法见 Core/Inc/dual_gear.h。

用法:
    python tools/gear_test.py --port COM10 check           # 配置 / 副编在线 / 量程
    python tools/gear_test.py --port COM10 verify          # 多位置一致性（默认 ±3 rad 输出轴）
    python tools/gear_test.py --port COM10 powercycle      # 每个位置断电重启后比对
    python tools/gear_test.py --port COM10 capture --at 0  # 重取参考（门极必须关闭）
    python tools/gear_test.py --port COM10 zero            # 清零后位置是否为 0、参考是否跟着挪
    python tools/gear_test.py --port COM10 ratio           # 反推实际传动比（齿数/方向不确定时）

依赖: pip install pyserial（--usb 方式见 tools/mc_ep.py）

台架上踩过的坑（改脚本前先看）:
  * 定位只用 telemetry.position（输出轴 rad）。telemetry.theta_mech 在退出电机模式时
    被 reset_foc() 清零，别拿它算位置。
  * 进电机模式后必须立刻把 command.p_des 钉在当前位置，否则 zero_commands() 留在
    命令里的 0 目标会把转子拉走。
  * command.* 的租约是 config.can_timeout（默认 100 ms），所以 hold() 每 30 ms 重发。
  * action.gear_read / gear_capture / zero 都要求门极关闭 → 先回 MENU。
  * 任何退出路径都要回 MENU 并恢复被临时改过的 i_max / accel_limit / pre_calibrated。

安全: 默认把 i_max 临时压到 2 A、accel_limit 压到 20 rad/s^2（只改 RAM，结束恢复），
--i-max/--accel 可调，--no-limit 跳过。移动中一旦读到 fault 或速度过高立刻中止。
"""

import argparse
import math
import sys
import time
from fractions import Fraction

import mc_ep
from mc_ep import Device

TWO_PI = 6.283185307179586
CPR = 65536.0             # 主编码器 KTH7823 单圈计数（ENC_CPR）
SEC_CPR = 65536.0         # 副编码器同款（SEC_ENC_CPR）
MENU_MODE = 0
MOTOR_MODE = 2

_CURRENT = {"dev": None}  # reboot() 换了句柄，收尾清理要用最新的那个


# ---------------------------------------------------------------- 基础设施

def connect(args, verbose=False):
    """连设备并拉一次描述符。刚复位后 CDC 可能半开，所以重试。"""
    last = None
    for i in range(60):
        dev = None
        try:
            dev = Device(port=args.port, baud=args.baud, timeout=args.timeout,
                         use_usb=args.usb, vid=args.vid, pid=args.pid,
                         interface=args.usb_interface, ep_out=args.ep_out, ep_in=args.ep_in)
            dev.fetch_descriptor()
            dev.read("system.fw_version_major")     # 端口真的活着才算连上
            if verbose and i:
                print("  (connected after %d tries)" % i)
            _CURRENT["dev"] = dev
            return dev
        except Exception as exc:                    # noqa: BLE001
            last = exc
            try:
                if dev is not None:
                    dev.close()
            except Exception:                       # noqa: BLE001
                pass
            time.sleep(0.3)
    raise SystemExit("cannot talk to the device: %s" % last)


def rd(dev, name, tries=6):
    last = None
    for _ in range(tries):
        try:
            return dev.read(name)
        except Exception as exc:                    # noqa: BLE001
            last = exc
            time.sleep(0.02)
    raise SystemExit("read %s failed: %s" % (name, last))


def wr(dev, name, value, tries=6):
    last = None
    for _ in range(tries):
        try:
            dev.write(name, value)
            return
        except Exception as exc:                    # noqa: BLE001
            last = exc
            time.sleep(0.02)
    raise SystemExit("write %s failed: %s" % (name, last))


def call(dev, name, tries=6):
    last = None
    for _ in range(tries):
        try:
            return dev.call(name)
        except Exception as exc:                    # noqa: BLE001
            last = exc
            time.sleep(0.02)
    raise SystemExit("call %s failed: %s" % (name, last))


def mode(dev):
    return rd(dev, "telemetry.mode")


def pos_out(dev):
    """输出轴位置（rad）。所有模式下都有效，定位只用它。"""
    return rd(dev, "telemetry.position")


def ensure_menu(dev, timeout=2.0):
    if mode(dev) != MENU_MODE:
        call(dev, "action.idle_mode")
        t0 = time.time()
        while mode(dev) != MENU_MODE:
            if time.time() - t0 > timeout:
                raise SystemExit("cannot get back to MENU (mode=%s)" % mode(dev))
            time.sleep(0.05)


def hold(dev, p_des, seconds, dt=0.03):
    """以 30 ms 周期刷新命令租约（config.can_timeout 默认 100 ms），保持 p_des。"""
    t0 = time.time()
    while time.time() - t0 < seconds:
        wr(dev, "command.mode", 2)
        wr(dev, "command.p_des", p_des)
        time.sleep(dt)


def enter_motor(dev):
    """进电机模式并把 p_des 钉在当前位置（避免 zero_commands() 的 0 目标）。"""
    if mode(dev) != MOTOR_MODE:
        p = pos_out(dev)
        ensure_menu(dev)
        wr(dev, "config.pre_calibrated", True)
        call(dev, "action.motor_mode")
        t0 = time.time()
        while time.time() - t0 < 0.35:
            wr(dev, "command.mode", 2)
            wr(dev, "command.p_des", p)
            time.sleep(0.02)
        if mode(dev) != MOTOR_MODE:
            raise SystemExit("motor mode refused (mode=%s)" % mode(dev))
    else:
        hold(dev, pos_out(dev), 0.15)


def move_to(dev, target, step=0.15, settle=0.15, abort_vel=8.0):
    """分小段挪到 target（输出轴 rad），每段检查 fault 和速度。"""
    cur = pos_out(dev)
    n = max(1, int(abs(target - cur) / step + 0.999))
    for i in range(1, n + 1):
        hold(dev, cur + (target - cur) * i / n, settle)
        fault = rd(dev, "telemetry.fault")
        if fault:
            raise SystemExit("ABORT: fault=%d while moving to %+.2f" % (fault, target))
        if abs(rd(dev, "telemetry.velocity_pll")) > abort_vel:
            raise SystemExit("ABORT: velocity too high while moving to %+.2f" % target)
    hold(dev, target, 0.4)
    return pos_out(dev)


def snapshot(dev):
    """一次要齐所有相关量（都来自同一时刻的接口快照，差别在几十 ms 内）。"""
    pm = rd(dev, "telemetry.position_motor")
    return {
        "pos_out": rd(dev, "telemetry.position"),
        "pos_motor": pm,
        "fw_turns": pm / TWO_PI,
        "enc_raw": rd(dev, "telemetry.encoder_raw"),
        "sec_raw": rd(dev, "telemetry.sec_enc_raw"),
        "sec_phase": rd(dev, "telemetry.sec_enc_phase"),
        "present": rd(dev, "telemetry.sec_enc_present"),
        "lost": rd(dev, "telemetry.sec_enc_lost_frames"),
        "gear_turns": rd(dev, "telemetry.gear_abs_turns"),
        "gear_status": rd(dev, "system.gear_status"),
        "mode": rd(dev, "telemetry.mode"),
        "fault": rd(dev, "telemetry.fault"),
        "gate": rd(dev, "telemetry.gate_drive"),
        "v_bus": rd(dev, "telemetry.v_bus"),
    }


def show_row(tag, s):
    print("%-9s pos_out=%+7.4f  fw=%+9.5f t  gear=%+9.5f t  diff=%+9.5f t (%+5.0f cnt)"
          " | sec=%5d st=%s mode=%d fault=%d"
          % (tag, s["pos_out"], s["fw_turns"], s["gear_turns"], s["gear_turns"] - s["fw_turns"],
             (s["gear_turns"] - s["fw_turns"]) * CPR, s["sec_raw"], s["gear_status"],
             s["mode"], s["fault"]))


def move_then_menu(dev, target, args):
    """开到 target 再回 MENU。enter_motor() 会临时置 pre_calibrated，这里恢复它 ——
    action.zero 会立刻写 flash，不能把工具临时改的状态带进去。"""
    pre = rd(dev, "config.pre_calibrated")
    enter_motor(dev)
    move_to(dev, target, step=args.move_step, settle=args.settle)
    ensure_menu(dev)
    wr(dev, "config.pre_calibrated", pre)


def gear_pair(dev):
    """回 MENU → action.gear_read → 同一时刻的一对（固件多圈, 双编解码）。"""
    ensure_menu(dev)
    time.sleep(0.12)
    call(dev, "action.gear_read")
    return snapshot(dev)


def reboot(dev, args, settle=4.0):
    print("reboot ...")
    try:
        call(dev, "action.reboot", tries=2)
    except SystemExit:
        pass
    try:
        dev.close()
    except Exception:                               # noqa: BLE001
        pass
    time.sleep(settle)
    dev = connect(args, verbose=True)
    print("  reconnected: mode=%s fault=%s" % (rd(dev, "telemetry.mode"), rd(dev, "telemetry.fault")))
    return dev


def safe_limits(dev, args):
    """临时压小电流/加速度（只改 RAM），返回原值。--no-limit 时返回 None。"""
    if args.no_limit:
        return None
    saved = {"i_max": rd(dev, "config.i_max"), "accel": rd(dev, "config.accel_limit"),
             "pre": rd(dev, "config.pre_calibrated")}
    ensure_menu(dev)
    wr(dev, "config.i_max", args.i_max)
    wr(dev, "config.accel_limit", args.accel)
    print("limits: i_max %s -> %s, accel_limit %s -> %s"
          % (saved["i_max"], args.i_max, saved["accel"], args.accel))
    return saved


def restore_limits(dev, saved):
    if not saved:
        return
    ensure_menu(dev)
    wr(dev, "config.i_max", saved["i_max"])
    wr(dev, "config.accel_limit", saved["accel"])
    wr(dev, "config.pre_calibrated", saved["pre"])


# ---------------------------------------------------------------- 数学小工具

def frac(x):
    return x - math.floor(x)


def local_phase(enc_raw, sec_raw, reversed_):
    """按 dual_gear.h 的 dg_phase() 在 PC 上重算韦尼尔相位，用来交叉验证固件。"""
    s_cnt = (SEC_CPR - sec_raw) if reversed_ else sec_raw
    return frac(s_cnt / SEC_CPR - enc_raw / CPR)


def fit_slope(thetas, deltas, guess):
    """把 Δ 解缠后线性拟合 Δθ 对 θ（转子圈）的斜率 = k/T2。

    Δ 是 [0,1) 的相位，跨度超过一圈就得先按 guess 解缠；guess 错得离谱时解缠会错，
    所以结果里同时给出残差 rms，让操作者自己判断。
    """
    th0, d0 = thetas[0], deltas[0]
    ys = []
    for th, d in zip(thetas, deltas):
        pred = d0 + guess * (th - th0)
        ys.append(d + round(pred - d))
    n = len(ys)
    mx = sum(thetas) / n
    my = sum(ys) / n
    sxx = sum((t - mx) ** 2 for t in thetas)
    if sxx <= 0.0:
        raise SystemExit("ratio: all samples at the same position")
    sxy = sum((t - mx) * (y - my) for t, y in zip(thetas, ys))
    a = sxy / sxx
    b = my - a * mx
    rms = math.sqrt(sum((y - (a * t + b)) ** 2 for t, y in zip(thetas, ys)) / n)
    return a, b, rms


def positions_from_args(args):
    if args.positions:
        return [float(x) for x in args.positions.replace(",", " ").split()]
    out = []
    p = args.start
    while p <= args.stop + 1e-9:
        out.append(round(p, 6))
        p += args.step
    return out


# ---------------------------------------------------------------- 子命令

def cmd_check(dev, args):
    t_first = rd(dev, "config.gear_teeth_first")
    t_sec = rd(dev, "config.gear_teeth_sec")
    rev = bool(rd(dev, "config.gear_sec_reversed"))
    gr = rd(dev, "config.gr")
    # gear_abs_turns 只有 action.gear_read / 上电恢复时才刷新，check 也要现读一次，
    # 否则看到的是上次调用留下的旧值（门极开着读不了，那就明说）
    if mode(dev) == MENU_MODE:
        call(dev, "action.gear_read")
        time.sleep(0.05)
    else:
        print("(mode=%d，门极可能使能，gear_abs_turns 是上次的缓存值；回 MENU 再跑更准)" % mode(dev))
    s = snapshot(dev)

    print("fw            : %u.%u.%u   descriptor_crc=0x%04X"
          % (rd(dev, "system.fw_version_major"), rd(dev, "system.fw_version_minor"),
             rd(dev, "system.fw_version_revision"), dev.json_crc))
    print("config        : gr=%.4f  teeth %d/%d  reversed=%s  ref_valid=%s"
          % (gr, t_first, t_sec, rev, rd(dev, "config.gear_ref_valid")))
    print("reference     : phase=%.9f  count=%d  m_zero=%d"
          % (rd(dev, "config.gear_zero_phase"), rd(dev, "config.gear_zero_count"),
             rd(dev, "config.m_zero")))
    print("secondary enc : present=%s lost_frames=%s raw=%d"
          % (s["present"], s["lost"], s["sec_raw"]))
    print("phase         : firmware=%.9f  pc=%.9f  (delta=%+.1e，两次读数不同步 + 编码器抖动)"
          % (s["sec_phase"], local_phase(s["enc_raw"], s["sec_raw"], rev),
             s["sec_phase"] - local_phase(s["enc_raw"], s["sec_raw"], rev)))
    print("now           : fw=%+.5f t  gear=%+.5f t  diff=%+.5f t (%+.0f cnt)  status=%s"
          % (s["fw_turns"], s["gear_turns"], s["gear_turns"] - s["fw_turns"],
             (s["gear_turns"] - s["fw_turns"]) * CPR, s["gear_status"]))

    k = t_first - t_sec
    if t_first <= 0 or t_sec <= 0 or k == 0:
        print("!! 齿数非法（t_first=%d t_sec=%d）：双编不会参与" % (t_first, t_sec))
        return 1
    win = t_sec / (2.0 * abs(k))
    center = rd(dev, "config.gear_zero_count") / CPR      # 参考位姿在当前坐标里的位置
    print("range         : ±%.3f 转子圈 = ±%.4f 输出圈 (±%.3f rad)  周期=%.3f 转子圈"
          % (win, win / gr, win / gr * TWO_PI, t_sec / abs(k)))
    print("window        : 中心 %+.3f 圈（= gear_zero_count/CPR，即参考位姿），"
          "有效 [%+.3f, %+.3f] 转子圈 = [%+.3f, %+.3f] 输出圈"
          % (center, center - win, center + win,
             (center - win) / gr, (center + win) / gr))
    print("                ⚠ 窗口以 capture 参考为中心；action.zero 会把参考一起挪到新零点，"
          "改 config.m_zero 不会")
    print("ratio         : T1/T2 = %.6f  (~%s)" % (t_first / float(t_sec), Fraction(t_first, t_sec)))
    if not s["present"]:
        print("!! 副编码器没应答：检查 SEC_ENC_SPI_NSS / 供电 / SPI 线序")
        return 1
    if not rd(dev, "config.gear_ref_valid"):
        print("!! gear_ref_valid=0：断电记忆关闭（跑 capture 或 action.calibrate）")
        return 1
    if abs(s["gear_turns"] - s["fw_turns"]) > 0.5:
        print("!! 解码与固件多圈差 >0.5 圈：参考可能已失效（齿数/方向改过？）")
        return 1
    return 0


def cmd_verify(dev, args):
    positions = positions_from_args(args)
    rows = []
    print("--- 多位置一致性（%d 点，输出轴 rad）---" % len(positions))
    for p in positions:
        enter_motor(dev)
        move_to(dev, p, step=args.move_step, settle=args.settle)
        s = gear_pair(dev)
        s["cmd"] = p
        rows.append(s)
        show_row("p=%+5.2f" % p, s)

    base = rows[0]
    worst = 0.0
    print("--- 相对第一点（消掉参考的常数偏移）---")
    for s in rows:
        d_fw = s["fw_turns"] - base["fw_turns"]
        d_gear = s["gear_turns"] - base["gear_turns"]
        miss = d_gear - d_fw
        worst = max(worst, abs(miss))
        print("p=%+5.2f  d(fw)=%+8.4f  d(gear)=%+8.4f  miss=%+8.5f t (%+5.0f cnt)"
              % (s["cmd"], d_fw, d_gear, miss, miss * CPR))
    print("worst |miss| = %.5f turns (%.0f cnt), 判据 %.3f turns"
          % (worst, worst * CPR, args.max_diff))
    return 0 if worst <= args.max_diff else 1


def cmd_powercycle(dev, args):
    positions = positions_from_args(args)
    print("--- 断电恢复（每个位置 record → reboot → compare）---")
    worst = 0.0
    for target in positions:
        wr(dev, "config.pre_calibrated", True)
        enter_motor(dev)
        move_to(dev, target, step=args.move_step, settle=args.settle)
        time.sleep(0.3)
        before = gear_pair(dev)
        dev = reboot(dev, args)
        time.sleep(args.boot_settle)
        after = gear_pair(dev)
        d_fw = after["fw_turns"] - before["fw_turns"]
        d_miss = (after["gear_turns"] - after["fw_turns"]) - (before["gear_turns"] - before["fw_turns"])
        worst = max(worst, abs(d_fw), abs(d_miss))
        ok = abs(d_fw) <= args.max_diff and abs(d_miss) <= args.max_diff
        print("p=%+5.2f: fw %+9.4f -> %+9.4f t  pos_out %+7.4f -> %+7.4f rad  "
              "delta=%+.5f t (%+.0f cnt)  miss=%+.5f t  %s"
              % (target, before["fw_turns"], after["fw_turns"], before["pos_out"], after["pos_out"],
                 d_fw, d_fw * CPR, d_miss, "OK" if ok else "!!! MISMATCH"))
        if abs(d_fw) > 1.0:
            center = rd(dev, "config.gear_zero_count") / CPR
            win = rd(dev, "config.gear_teeth_sec") / (2.0 * abs(
                rd(dev, "config.gear_teeth_first") - rd(dev, "config.gear_teeth_sec")))
            print("    (整圈级偏差：断电点 %.3f 圈落在参考窗口 [%+.3f, %+.3f] 之外，"
                  "差的就是一个相位周期 T2/|T1-T2|)" % (after["fw_turns"], center - win, center + win))
    print("worst |delta| = %.5f turns, 判据 %.3f turns" % (worst, args.max_diff))
    return 0 if worst <= args.max_diff else 1


def cmd_capture(dev, args):
    if args.at is not None:
        move_then_menu(dev, args.at, args)
    ensure_menu(dev)
    old_phase = rd(dev, "config.gear_zero_phase")
    old_count = rd(dev, "config.gear_zero_count")
    call(dev, "action.gear_capture")
    status = rd(dev, "system.gear_status")
    s = snapshot(dev)
    print("capture: status=%s  ref_valid=%s" % (status, rd(dev, "config.gear_ref_valid")))
    print("  old reference: phase=%.9f count=%d" % (old_phase, old_count))
    print("  new reference: phase=%.9f count=%d"
          % (rd(dev, "config.gear_zero_phase"), rd(dev, "config.gear_zero_count")))
    print("  %s" % ("OK" if status == 1 else "FAILED - 旧参考已保留（齿数非法/副编无应答？）"))
    if status == 1:
        print("  diff now = %+.5f t（新参考下解码应当≈固件多圈）"
              % (s["gear_turns"] - s["fw_turns"]))
        print("  提醒: 参考只在 RAM，需要 action.save 或随标定一起存盘")
    return 0 if status == 1 else 1


def cmd_zero(dev, args):
    """action.zero：当前位置的读数必须变成 0，且双编参考要跟着挪（窗口重新以零点为中心）。"""
    if args.at is not None:
        move_then_menu(dev, args.at, args)
    before = gear_pair(dev)
    show_row("before", before)
    ensure_menu(dev)
    print("注意: action.zero 会立即写 flash（m_zero 与双编参考一起存）")
    call(dev, "action.zero")
    time.sleep(0.2)
    after = gear_pair(dev)
    show_row("after", after)
    d_fw = after["fw_turns"] - before["fw_turns"]
    d_miss = (after["gear_turns"] - after["fw_turns"]) - (before["gear_turns"] - before["fw_turns"])
    print("参考位移: 解码 miss = %+.5f t（判据 %.3f t）" % (d_miss, args.max_diff))
    print("位置: 固件 %+.5f t / %+.4f rad，解码 %+.5f t —— 清零后两者都应当是 0"
          % (after["fw_turns"], after["pos_out"], after["gear_turns"]))
    ok = (abs(d_miss) <= args.max_diff
          and abs(after["fw_turns"]) <= args.max_diff
          and abs(after["gear_turns"]) <= args.max_diff)
    print("  %s" % ("OK - 位置已归零且参考同步平移" if ok
                    else "!! 位置没归零或参考没跟上（旧固件只有单圈零点会这样）"))
    return 0 if ok else 1


def cmd_ratio(dev, args):
    t_first = rd(dev, "config.gear_teeth_first")
    t_sec = rd(dev, "config.gear_teeth_sec")
    rev = bool(rd(dev, "config.gear_sec_reversed"))
    guess = args.slope if args.slope is not None else (t_first - t_sec) / float(t_sec)
    positions = positions_from_args(args)
    print("--- 实测传动比（%d 点，解缠斜率猜测 k/T2=%.6f）---" % (len(positions), guess))
    thetas, deltas = [], []
    for p in positions:
        enter_motor(dev)
        move_to(dev, p, step=args.move_step, settle=args.settle)
        s = gear_pair(dev)
        d = local_phase(s["enc_raw"], s["sec_raw"], rev)
        thetas.append(s["fw_turns"])
        deltas.append(d)
        print("p=%+5.2f  theta=%+8.4f t  delta=%.9f  (firmware sec_phase=%.9f)"
              % (p, s["fw_turns"], d, s["sec_phase"]))
    a, b, rms = fit_slope(thetas, deltas, guess)
    span = max(thetas) - min(thetas)
    print("span=%.3f 转子圈   斜率 dΔ/dθ=%+.6f (配置 %+.6f, 偏差 %+.2f%%)   rms=%.6f"
          % (span, a, guess, (a - guess) / guess * 100.0 if guess else 0.0, rms))
    print("  reversed=%s 时量得  T1/T2 = 1 + slope = %.6f" % (rev, 1.0 + a))
    print("  按 config.gear_teeth_sec=%d 反推: gear_teeth_first ≈ %.2f → %d"
          % (t_sec, (1.0 + a) * t_sec, int(round((1.0 + a) * t_sec))))
    print("  副路方向反了（外啮合）就打开 gear_sec_reversed：镜像后 slope 变号，")
    print("  正号对应 T1/T2 = 1 + slope，负号对应 1 - slope（上面只算了 reversed 当前值）。")
    print("  短跨度会被齿隙/偏心带偏，求平均值请用大跨度（--start -3 --stop 3，约 8.6 圈）。")
    print("  改完齿数要 action.gear_capture 重新取参考（齿数变了旧参考就失效了）。")
    return 0 if rms < 0.01 else 1


# ---------------------------------------------------------------- main

def add_common(p, suppress):
    """safety / sweep 两组选项。子命令里那份用 SUPPRESS，免得默认值覆盖掉

    写在子命令前面的同名选项（argparse 的 parents 会无条件写回默认值）。"""
    d = (lambda v: argparse.SUPPRESS) if suppress else (lambda v: v)
    safety = p.add_argument_group("safety")
    safety.add_argument("--i-max", type=float, default=d(2.0), help="临时限流 A（默认 2）")
    safety.add_argument("--accel", type=float, default=d(20.0), help="临时加速度 rad/s^2（默认 20）")
    safety.add_argument("--no-limit", action="store_true", default=d(False), help="不改 i_max/accel_limit")
    safety.add_argument("--move-step", type=float, default=d(0.15), help="移动分段长度 rad（默认 0.15）")
    safety.add_argument("--settle", type=float, default=d(0.15), help="每段停留 s（默认 0.15）")
    safety.add_argument("--boot-settle", type=float, default=d(0.5), help="重启后等多久再读 s（默认 0.5）")
    safety.add_argument("--max-diff", type=float, default=d(0.01), help="判定阈值（转子圈，默认 0.01）")

    sweep = p.add_argument_group("sweep")
    sweep.add_argument("--start", type=float, default=d(-3.0), help="起始输出轴位置 rad（默认 -3）")
    sweep.add_argument("--stop", type=float, default=d(3.0), help="结束位置 rad（默认 +3）")
    sweep.add_argument("--step", type=float, default=d(0.5), help="位置间隔 rad（默认 0.5）")
    sweep.add_argument("--positions", default=d(None),
                       help="直接给位置列表（逗号或空格分隔，优先级高于 start/stop）")


def main():
    ap = argparse.ArgumentParser(description="齿轮式双编（Vernier）台架验证",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    link = ap.add_argument_group("link")
    link.add_argument("--port", help="CDC 串口，例如 COM10")
    link.add_argument("--usb", action="store_true", help="用 vendor interface 的 bulk 端点（需 pyusb）")
    link.add_argument("--baud", type=int, default=115200)
    link.add_argument("--timeout", type=float, default=1.0)
    link.add_argument("--vid", type=lambda s: int(s, 0), default=mc_ep.DEFAULT_VID)
    link.add_argument("--pid", type=lambda s: int(s, 0), default=mc_ep.DEFAULT_PID)
    link.add_argument("--usb-interface", type=int, default=mc_ep.DEFAULT_USB_INTERFACE)
    link.add_argument("--ep-out", type=lambda s: int(s, 0), default=mc_ep.DEFAULT_EP_OUT)
    link.add_argument("--ep-in", type=lambda s: int(s, 0), default=mc_ep.DEFAULT_EP_IN)
    add_common(ap, suppress=False)

    sub_parent = argparse.ArgumentParser(add_help=False)
    add_common(sub_parent, suppress=True)

    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="配置/在线/量程（不动电机）")
    sub.add_parser("verify", parents=[sub_parent], help="多位置一致性")
    sub.add_parser("powercycle", parents=[sub_parent], help="各位置断电重启比对")
    p_cap = sub.add_parser("capture", parents=[sub_parent], help="重取参考")
    p_cap.add_argument("--at", type=float, help="先开到该输出轴位置 rad")
    p_zero = sub.add_parser("zero", parents=[sub_parent], help="清零后位置是否为 0、参考是否跟着挪")
    p_zero.add_argument("--at", type=float, help="先开到该输出轴位置 rad")
    p_ratio = sub.add_parser("ratio", parents=[sub_parent], help="反推实际传动比")
    p_ratio.add_argument("--slope", type=float, help="解缠用的 k/T2 猜测（默认取配置齿数）")

    args = ap.parse_args()
    if not args.usb and not args.port:
        ap.error("需要 --port COMx 或 --usb")

    handler = {"check": cmd_check, "verify": cmd_verify, "powercycle": cmd_powercycle,
               "capture": cmd_capture, "zero": cmd_zero, "ratio": cmd_ratio}[args.cmd]

    dev = connect(args)
    saved = None
    rc = 1
    try:
        # zero/check 不动电机，不需要临时限流 —— 而且 action.zero 会立刻把整份配置
        # 写进 flash，带着临时限流存盘会改掉用户的 i_max
        if args.cmd in ("verify", "powercycle", "capture", "ratio"):
            saved = safe_limits(dev, args)
        rc = handler(dev, args)
    finally:
        dev = _CURRENT["dev"]                        # powercycle 期间可能换过句柄
        try:
            restore_limits(dev, saved)
            s = snapshot(dev)
            print("final: mode=%d gate=%d fault=%d pre_cal=%s i_max=%s gr=%s teeth=%s/%s rev=%s"
                  " present=%s lost=%s ref_valid=%s"
                  % (s["mode"], s["gate"], s["fault"], rd(dev, "config.pre_calibrated"),
                     rd(dev, "config.i_max"), rd(dev, "config.gr"),
                     rd(dev, "config.gear_teeth_first"), rd(dev, "config.gear_teeth_sec"),
                     rd(dev, "config.gear_sec_reversed"), s["present"], s["lost"],
                     rd(dev, "config.gear_ref_valid")))
        except Exception as exc:                    # noqa: BLE001
            print("cleanup:", exc)
        try:
            dev.close()
        except Exception:                           # noqa: BLE001
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
