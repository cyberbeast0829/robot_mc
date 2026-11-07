#!/usr/bin/env python3
"""
齿槽转矩补偿（anti-cogging）的台架工具。

用法：
    python tools/cogging_test.py --port COM10 status        # 配置/标定结果/表幅值
    python tools/cogging_test.py --port COM10 calibrate     # 一条指令跑完整标定（约 2~4 分钟）
    python tools/cogging_test.py --port COM10 dump          # 把表读出来做频谱/存 CSV
    python tools/cogging_test.py --port COM10 on|off        # 打开/关闭补偿（RAM，需 action.save 落盘）
    python tools/cogging_test.py --port COM10 reset         # 丢弃表（RAM）

依赖：pip install pyserial（-usb 方式见 tools/mc_ep.py）

台架注意事项：
  * 标定要求已经 action.tune_gains（pos_gain / vel_gain / vel_integrator_gain > 0）——
    标定用的就是这套外环增益，增益没调好标出来的表也不可信。
  * 标定时**不要有人/外力碰关节**：它是靠"把位置保持住需要多少电流"来测力矩的。
  * 结果只写 RAM。要断电保留：先回菜单（门极关闭）再 action.save。
  * 补偿打开后先用低速 CSV 走一段（tools/csv_csp_test.py）比较速度波动，再决定是否保存。
  * 换编码器、重新 action.zero、拆装电机之后必须重标（表的相位跟着编码器走，
    但 config.m_zero 不影响它）。
"""

import argparse
import math
import sys
import time

import mc_ep
from mc_ep import Device

COG_N = 1024
COG_SCALE = 4096.0
MENU_MODE = 0

STATUS_TEXT = {
    0: "not run",
    1: "running",
    2: "success",
    -1: "aborted (left motor mode / CAN timeout)",
    -2: "too many bad points",
    -3: "the two passes disagree (backlash / friction / gains?)",
    -4: "timeout",
    -5: "outer-loop gains not configured",
}


def connect(args):
    return Device(port=args.port, baud=args.baud, timeout=args.timeout,
                  use_usb=args.usb, vid=args.vid, pid=args.pid,
                  interface=args.usb_interface, ep_out=args.ep_out, ep_in=args.ep_in)


def print_status(dev):
    st = dev.read("system.cogging_status")
    print("  status          : %d (%s)" % (st, STATUS_TEXT.get(st, "?")))
    print("  valid / enabled : %s / %s"
          % (dev.read("config.cogging_valid"), dev.read("config.cogging_en")))
    print("  gain / max      : %.3f x / %.3f A"
          % (dev.read("config.cogging_gain"), dev.read("config.cogging_max")))
    print("  method          : %s (%d), %d ms/rev"
          % ("sweep" if dev.read("config.cogging_method") == 1 else "hold",
             dev.read("config.cogging_method"), dev.read("config.cogging_sweep_ms")))
    print("  amplitude       : %.4f A RMS" % dev.read("system.cogging_amp"))
    print("  mismatch        : %.4f A RMS" % dev.read("system.cogging_mismatch"))
    print("  bad points      : %d / %d" % (dev.read("system.cogging_bad"), COG_N))
    print("  last feedforward: %.4f A" % dev.read("telemetry.cogging_ff"))
    print("  mode            : %d (0 = menu)" % dev.read("telemetry.mode"))


def cmd_status(dev, args):
    print("Anti-cogging:")
    print_status(dev)
    return 0


def cmd_calibrate(dev, args):
    mode = dev.read("telemetry.mode")
    if mode != MENU_MODE:
        raise SystemExit("device must be in the menu (mode 0, gate drive off), currently %d" % mode)
    if not dev.read("config.pre_calibrated"):
        raise SystemExit("run action.calibrate first (config.pre_calibrated is false)")

    print("Starting anti-cogging calibration...")
    method = dev.read("config.cogging_method")
    if method == 1:
        print("  method: sweep (%d ms per rotor revolution, 2 revolutions) - the shaft turns"
              " one rotor turn forward and back" % dev.read("config.cogging_sweep_ms"))
    else:
        print("  method: hold (2 x %d points) - the joint is held at each rotor position" % COG_N)
    dev.call("action.cogging_calibrate")

    t0 = time.time()
    last = -1
    while True:
        time.sleep(args.poll)
        st = dev.read("system.cogging_status")
        done = dev.read("system.cogging_progress")
        total = dev.read("system.cogging_progress_total")
        if done != last:
            last = done
            if total > 0:
                print("  %4d / %4d points  (%.1f s)" % (done, total, time.time() - t0))
        if st != 1:                       # 1 = running
            break
        if time.time() - t0 > args.timeout:
            print("host-side timeout: the device still reports 'running'")
            break

    print("\nResult:")
    print_status(dev)
    st = dev.read("system.cogging_status")
    if st != 2:
        return 1
    print("\nRAM only. Verify at low speed, then (menu, gate off) action.save.")
    return 0


def cmd_dump(dev, args):
    if not dev.read("config.cogging_valid"):
        print("warning: config.cogging_valid is false - this table is not being used")

    data = []
    for i in range(COG_N):
        dev.write("config.cogging_index", i)
        data.append(dev.read("telemetry.cogging_value"))

    rms = math.sqrt(sum(v * v for v in data) / len(data))
    peak = max(abs(v) for v in data)
    mean = sum(data) / len(data)
    print("  points %d   RMS %.5f A   peak %.5f A   mean %.6f A" % (len(data), rms, peak, mean))

    # 找主要谐波（转子每转的周期数）。12 槽/14 极对的基波是 LCM(12,28)=84，
    # 和 168、252… 以及极对数 14 的倍数。纯 Python DFT 足够看趋势。
    top = []
    for order in range(1, args.max_order + 1):
        c = s = 0.0
        for i, v in enumerate(data):
            ph = 2.0 * math.pi * order * i / COG_N
            c += v * math.cos(ph)
            s += v * math.sin(ph)
        top.append((math.hypot(c, s) * 2.0 / COG_N, order))
    top.sort(reverse=True)
    print("  dominant harmonics (cycles per rotor turn):")
    for amp, order in top[:args.top]:
        print("    %5d : %8.5f A" % (order, amp))

    if args.csv:
        with open(args.csv, "w", encoding="ascii") as f:
            f.write("index,turns,amps\n")
            for i, v in enumerate(data):
                f.write("%d,%.9f,%.9f\n" % (i, i / float(COG_N), v))
        print("  written to %s" % args.csv)
    return 0


def cmd_on(dev, args):
    if not dev.read("config.cogging_valid"):
        raise SystemExit("config.cogging_valid is false - run calibrate first")
    dev.write("config.cogging_en", True)
    print("config.cogging_en = true (RAM only; action.save to keep it)")
    return 0


def cmd_off(dev, args):
    dev.write("config.cogging_en", False)
    print("config.cogging_en = false (RAM only; action.save to keep it)")
    return 0


def cmd_reset(dev, args):
    dev.call("action.cogging_reset")
    print("table cleared, config.cogging_valid = false (RAM only; action.save to keep it)")
    return 0


def main():
    ap = argparse.ArgumentParser(description="anti-cogging bench tool")
    link = ap.add_argument_group("link")
    link.add_argument("--port", help="CDC serial port, e.g. COM10")
    link.add_argument("--usb", action="store_true", help="use the vendor bulk interface (needs pyusb)")
    link.add_argument("--baud", type=int, default=115200)
    link.add_argument("--timeout", type=float, default=1.0)
    link.add_argument("--vid", type=lambda s: int(s, 0), default=mc_ep.DEFAULT_VID)
    link.add_argument("--pid", type=lambda s: int(s, 0), default=mc_ep.DEFAULT_PID)
    link.add_argument("--usb-interface", type=int, default=mc_ep.DEFAULT_USB_INTERFACE)
    link.add_argument("--ep-out", type=lambda s: int(s, 0), default=mc_ep.DEFAULT_EP_OUT)
    link.add_argument("--ep-in", type=lambda s: int(s, 0), default=mc_ep.DEFAULT_EP_IN)

    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("status", help="print calibration state / table amplitude")

    p = sub.add_parser("calibrate", help="run action.cogging_calibrate and watch progress")
    p.add_argument("--poll", type=float, default=0.5, help="polling period s (default 0.5)")
    p.add_argument("--timeout", type=float, default=900.0, help="host-side timeout s (default 900)")

    p = sub.add_parser("dump", help="read the whole table, report harmonics, optional CSV")
    p.add_argument("--csv", help="write the table to this CSV file")
    p.add_argument("--top", type=int, default=8, help="how many harmonics to print")
    p.add_argument("--max-order", type=int, default=200, help="highest harmonic order to scan")

    sub.add_parser("on", help="set config.cogging_en = true (RAM)")
    sub.add_parser("off", help="set config.cogging_en = false (RAM)")
    sub.add_parser("reset", help="discard the table (RAM)")

    args = ap.parse_args()
    if not args.cmd:
        ap.print_help()
        return 2

    dev = connect(args)
    try:
        return {
            "status": cmd_status,
            "calibrate": cmd_calibrate,
            "dump": cmd_dump,
            "on": cmd_on,
            "off": cmd_off,
            "reset": cmd_reset,
        }[args.cmd](dev, args)
    finally:
        dev.close()


if __name__ == "__main__":
    sys.exit(main())
