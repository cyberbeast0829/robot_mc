#!/usr/bin/env python3
"""
csv_csp_test.py

CSV（速度环）/ CSP（位置环）的 USB 台架测试与增益整定工具。走 endpoint 协议
（`mc_ep.py` 的 Device），不需要 CAN 主站。

为什么需要单独一个脚本：
  * `command.*` 是**命令租约**——`config.can_timeout`（默认 100 ms）内不刷新就
    会 `zero_commands()` 卸力。用 `mc_ep.py set` 一条条敲，两条之间早就超时了，
    测出来的"环路不工作"其实是保护动作。这里每 25 ms 重写一次目标值。
  * 台架测试必须有中止闸：故障锁存、超速、跟随误差失控都立即关门极（`finally`）。

子命令：
  plain    进入 MOTOR_MODE，用 MIT 律（kp/kd）把当前位置保住，做回归用
  csv      速度阶跃：0 → +v → 0 → -v → 0
  csp      位置阶跃：保持 → +step → 保持 → 回原位 → 保持
  timeout  刷新 0.6 s 后停手，验证租约到期后是否卸力（command.mode 回 0）
  sweep    扫描 config.vel_gain（纯 P），打印稳态误差/超调表，结束后恢复原值

用法示例（COM6，输出轴量）：
  python tools/csv_csp_test.py --port COM6 plain --hold 2 --kp 20 --kd 1
  python tools/csv_csp_test.py --port COM6 csv --vel 1.0 --hold 0.8 --move 2.5
  python tools/csv_csp_test.py --port COM6 csp --step 0.2 --hold 1.2 --move 2.0
  python tools/csv_csp_test.py --port COM6 timeout --vel 0.5
  python tools/csv_csp_test.py --port COM6 sweep --vel 1.0 --gains 1,2,3,4,6
  python tools/csv_csp_test.py --port COM6 csv --vel 1.0 --log run.csv

⚠ 会真的转电机。母线/电流限幅/机械行程自己先确认好；`--min-bus` 默认 15 V，
  低于它会拒绝启动（`--allow-low-bus` 可在无功率级时只验证逻辑）。
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mc_ep import Device, EpError  # noqa: E402

READ = ["telemetry.position", "telemetry.velocity", "telemetry.velocity_pll",
        "telemetry.pos_err", "telemetry.vel_err", "telemetry.v_setpoint",
        "telemetry.vel_integral", "telemetry.i_q_des", "telemetry.i_q_filt",
        "telemetry.v_bus", "telemetry.fault", "telemetry.gate_drive",
        "command.mode"]

LOG_FIELDS = [n.split(".")[-1] for n in READ]


class Runner:
    def __init__(self, args):
        self.a = args
        self.dev = Device(args.port)
        self.dev.fetch_descriptor()
        self.rows = []

    # ---- 生命周期 ------------------------------------------------------

    def close(self):
        try:
            self.dev.call("action.idle_mode")
        except Exception as exc:  # noqa: BLE001 - 收尾必须尽力而为
            print("WARN idle_mode failed: %s" % exc)
        self.dev.close()

    def check_ready(self):
        # 先回菜单：action.motor_mode 只允许从 MENU 进入（否则 EP_ERR_STATE），
        # 而且上一次异常退出/手工调试可能把设备留在电机模式里
        try:
            self.dev.call("action.idle_mode")
        except EpError:
            pass
        time.sleep(0.15)
        fault = self.dev.read("telemetry.fault")
        vbus = self.dev.read("telemetry.v_bus")
        print("gate_drive=%s fault=%s v_bus=%.2f V mode=%s" % (
            self.dev.read("telemetry.gate_drive"), fault, vbus,
            self.dev.read("command.mode")))
        if fault:
            raise SystemExit("REFUSE: 已锁存故障 %s，先 action.idle_mode 清故障" % fault)
        if vbus < self.a.min_bus:
            raise SystemExit("REFUSE: v_bus=%.2f V < %.1f V，主电源没上电"
                             "（--allow-low-bus 可只验证逻辑）" % (vbus, self.a.min_bus))
        return vbus

    def enter_motor(self):
        self.dev.call("action.motor_mode")
        time.sleep(0.1)
        print("entered motor mode: gate_drive=%s command.mode=%s" % (
            self.dev.read("telemetry.gate_drive"), self.dev.read("command.mode")))

    # ---- 采样与刷新 ----------------------------------------------------

    def sample(self, phase):
        row = {"t": time.time(), "phase": phase}
        for name in READ:
            row[name.split(".")[-1]] = self.dev.read(name)
        self.rows.append(row)
        # 中止闸：故障/超速/跟随误差失控都立即退出（finally 会关门极）
        if row["fault"]:
            raise SystemExit("ABORT: fault=%s (phase %s)" % (row["fault"], phase))
        if abs(row["velocity_pll"]) > self.a.abort_vel:
            raise SystemExit("ABORT: |velocity_pll|=%.2f > %.2f rad/s (phase %s)"
                             % (row["velocity_pll"], self.a.abort_vel, phase))
        if abs(row["pos_err"]) > self.a.abort_pos_err:
            raise SystemExit("ABORT: |pos_err|=%.2f > %.2f rad (phase %s)"
                             % (row["pos_err"], self.a.abort_pos_err, phase))
        return row

    def refresh(self, mode, p_des=None, v_des=None):
        """命令租约刷新。每个周期都要重写，否则 100 ms 后固件会卸力。"""
        self.dev.write("command.mode", mode)
        if p_des is not None:
            self.dev.write("command.p_des", p_des)
        if v_des is not None:
            self.dev.write("command.v_des", v_des)

    def run_phase(self, phase, seconds, target_fn, quiet=False):
        t0 = time.time()
        next_cmd = 0.0
        next_y = 0.0
        last = None
        while True:
            now = time.time() - t0
            if now >= seconds:
                break
            if now >= next_y:
                # 采样前先刷新一次：一次采样要发十几次读请求（十几毫秒），
                # 不先刷新的话租约余量会被吃掉
                mode, p_des, v_des = target_fn(now)
                self.refresh(mode, p_des, v_des)
                last = self.sample(phase)
                now = time.time() - t0
                next_cmd = now + self.a.refresh_ms / 1000.0
                next_y = now + self.a.sample_ms / 1000.0
            elif now >= next_cmd:
                mode, p_des, v_des = target_fn(now)
                self.refresh(mode, p_des, v_des)
                next_cmd = now + self.a.refresh_ms / 1000.0
            else:
                time.sleep(0.002)
        if last is not None and not quiet:
            print("  %-6s end: pos=%+7.4f vel=%+7.4f pos_err=%+7.4f vel_err=%+7.4f "
                  "v_sp=%+7.4f integ=%+6.3f i_q_des=%+6.3f" % (
                      phase, last["position"], last["velocity"], last["pos_err"],
                      last["vel_err"], last["v_setpoint"], last["vel_integral"],
                      last["i_q_des"]))
        return last

    # ---- 统计 ----------------------------------------------------------

    @staticmethod
    def phase_stats(rows, phase, cmd):
        sel = [r for r in rows if r["phase"] == phase]
        if len(sel) < 4:
            return None
        half = sel[len(sel) // 2:]
        v_mean = sum(r["velocity_pll"] for r in half) / len(half)
        v_peak = max(r["velocity_pll"] for r in half)
        i_peak = max(abs(r["i_q_filt"]) for r in half)
        return {
            "vel_mean": v_mean,
            "err_pct": 100.0 * (v_mean - cmd) / cmd if cmd else 0.0,
            "overshoot_pct": 100.0 * (v_peak - cmd) / cmd if cmd else 0.0,
            "iq_peak": i_peak,
        }

    def summary(self):
        if not self.rows:
            return
        t0 = self.rows[0]["t"]
        print("\n  t[s]  phase   pos      vel    vel_pll  pos_err  vel_err  v_sp     integ   i_q_des  i_q_f  v_bus")
        for r in self.rows:
            print("  %5.2f %-6s %+7.4f %+7.4f %+7.4f %+8.4f %+8.4f %+7.4f %+6.3f %+7.3f %+6.3f %5.1f" % (
                r["t"] - t0, r["phase"], r["position"], r["velocity"], r["velocity_pll"],
                r["pos_err"], r["vel_err"], r["v_setpoint"], r["vel_integral"],
                r["i_q_des"], r["i_q_filt"], r["v_bus"]))

        if self.a.cmd in ("csv", "sweep"):
            for phase, cmd in (("fwd", self.a.vel), ("rev", -self.a.vel)):
                st = self.phase_stats(self.rows, phase, cmd)
                if st:
                    print("  %-5s: vel_pll(2nd half)=%+.4f  err=%+.1f%%  overshoot=%+.1f%%  "
                          "|i_q|peak=%.2f A" % (phase, st["vel_mean"], st["err_pct"],
                                                st["overshoot_pct"], st["iq_peak"]))
        if self.a.cmd == "csp":
            for phase in ("hold0", "step+", "hold1", "back", "hold2"):
                sel = [r for r in self.rows if r["phase"] == phase]
                if sel:
                    pmax = max(abs(r["pos_err"]) for r in sel)
                    print("  %-6s: |pos_err|max=%.4f rad  last=%+.4f rad  pos=%+.4f"
                          % (phase, pmax, sel[-1]["pos_err"], sel[-1]["position"]))

    def write_log(self):
        if not self.a.log or not self.rows:
            return
        with open(self.a.log, "w", encoding="utf-8", newline="") as f:
            f.write("t,phase," + ",".join(LOG_FIELDS) + "\n")
            t0 = self.rows[0]["t"]
            for r in self.rows:
                f.write("%.4f,%s," % (r["t"] - t0, r["phase"]))
                f.write(",".join("%.6g" % r[k] for k in LOG_FIELDS) + "\n")
        print("log -> %s (%d rows)" % (self.a.log, len(self.rows)))

    # ---- 场景 ----------------------------------------------------------

    def cmd_plain(self):
        """MIT 律回归：kp/kd 顶住当前位置（不经过外环）。"""
        self.enter_motor()
        p0 = self.dev.read("telemetry.position")
        print("MIT hold at %+.4f rad, kp=%s kd=%s" % (p0, self.a.kp, self.a.kd))
        if self.a.kp is not None:
            self.dev.write("command.kp", self.a.kp)
        if self.a.kd is not None:
            self.dev.write("command.kd", self.a.kd)
        self.run_phase("hold", self.a.hold, lambda t: (0, p0, 0.0))

    def _csv_phases(self, quiet=False):
        v = self.a.vel
        self.run_phase("hold0", self.a.hold, lambda t: (1, 0.0, 0.0), quiet)
        self.run_phase("fwd", self.a.move, lambda t: (1, 0.0, +v), quiet)
        self.run_phase("hold1", self.a.hold * 0.5, lambda t: (1, 0.0, 0.0), quiet)
        self.run_phase("rev", self.a.move, lambda t: (1, 0.0, -v), quiet)
        self.run_phase("hold2", self.a.hold, lambda t: (1, 0.0, 0.0), quiet)

    def cmd_csv(self):
        self.enter_motor()
        self._csv_phases()

    def cmd_csp(self):
        self.enter_motor()
        p0 = self.dev.read("telemetry.position")
        step = self.a.step
        print("start position = %+.4f rad, step = %+.3f rad" % (p0, step))
        self.run_phase("hold0", self.a.hold, lambda t: (2, p0, 0.0))
        self.run_phase("step+", self.a.move, lambda t: (2, p0 + step, 0.0))
        self.run_phase("hold1", self.a.hold, lambda t: (2, p0 + step, 0.0))
        self.run_phase("back", self.a.move, lambda t: (2, p0, 0.0))
        self.run_phase("hold2", self.a.hold, lambda t: (2, p0, 0.0))

    def cmd_timeout(self):
        """停手后应卸力：command.mode 回 0、i_q_des → 0、v_setpoint → 0。"""
        self.enter_motor()
        self.run_phase("csv", 0.6, lambda t: (1, 0.0, self.a.vel))
        print("stop refreshing, waiting 400 ms ...")
        time.sleep(0.4)
        self.sample("after_timeout")
        r = self.rows[-1]
        print("  after timeout: command.mode=%s i_q_des=%+.3f vel=%+.4f v_setpoint=%+.4f"
              % (r["mode"], r["i_q_des"], r["velocity"], r["v_setpoint"]))

    def cmd_sweep(self):
        """纯 P 扫描 vel_gain：稳态误差给出摩擦/增益够不够，超调给出稳定性余量。"""
        dev = self.dev
        kv0 = dev.read("config.vel_gain")
        ki0 = dev.read("config.vel_integrator_gain")
        print("sweep vel_gain: %s (vel=%.2f rad/s, ki 置 0)" % (self.a.gains, self.a.vel))
        print("  %-8s %-10s %-9s %-11s %s" % ("vel_gain", "vel(2nd)", "err%", "overshoot%", "peak|i_q|"))
        try:
            dev.write("config.vel_integrator_gain", 0.0)
            self.enter_motor()
            for kv in self.a.gains:
                dev.write("config.vel_gain", kv)
                self.rows = []
                try:
                    self._csv_phases(quiet=True)
                    st = self.phase_stats(self.rows, "fwd", self.a.vel)
                    if st is None:
                        print("  %-8.2f (no data)" % kv)
                    else:
                        print("  %-8.2f %+9.4f %+8.1f %+10.1f %9.2f" % (
                            kv, st["vel_mean"], st["err_pct"],
                            st["overshoot_pct"], st["iq_peak"]))
                except SystemExit as exc:
                    print("  %-8.2f UNSTABLE / ABORT: %s" % (kv, exc))
        finally:
            dev.write("config.vel_gain", kv0)
            dev.write("config.vel_integrator_gain", ki0)
            print("restored vel_gain=%s vel_integrator_gain=%s" % (kv0, ki0))


DEFAULTS = dict(port="COM7", min_bus=15.0, allow_low_bus=False, hold=1.0, move=2.0,
                vel=1.0, step=0.10, gains="1,2,3,4", refresh_ms=25.0, sample_ms=60.0,
                abort_vel=3.0, abort_pos_err=0.5, kp=None, kd=None, log=None)


def add_common_args(p):
    """选项同时挂在主 parser 和每个子命令上，所以 `--vel 1.0 csv` 与
    `csv --vel 1.0` 都能用。default 用 SUPPRESS，避免子 parser 的默认值把
    写在子命令前面的值覆盖掉。"""
    sup = argparse.SUPPRESS
    p.add_argument("--port", default=sup, help="设备串口")
    p.add_argument("--min-bus", type=float, default=sup, help="低于此母线电压拒绝做运动测试")
    p.add_argument("--allow-low-bus", action="store_true", default=sup,
                   help="无功率级时只验证逻辑")
    p.add_argument("--hold", type=float, default=sup, help="保持段时长 (s)")
    p.add_argument("--move", type=float, default=sup, help="运动段时长 (s)")
    p.add_argument("--vel", type=float, default=sup, help="CSV 速度 (输出轴 rad/s)")
    p.add_argument("--step", type=float, default=sup, help="CSP 位置阶跃 (输出轴 rad)")
    p.add_argument("--gains", default=sup, help="sweep 的 vel_gain 列表，逗号分隔")
    p.add_argument("--refresh-ms", type=float, default=sup,
                   help="命令刷新周期，必须 < can_timeout")
    p.add_argument("--sample-ms", type=float, default=sup, help="遥测采样周期")
    p.add_argument("--abort-vel", type=float, default=sup,
                   help="超此速度立即中止 (输出轴 rad/s)")
    p.add_argument("--abort-pos-err", type=float, default=sup,
                   help="超此跟随误差立即中止 (rad)")
    p.add_argument("--kp", type=float, default=sup, help="plain 用的 MIT 位置刚度")
    p.add_argument("--kd", type=float, default=sup, help="plain 用的 MIT 速度阻尼")
    p.add_argument("--log", default=sup, help="把采样表写成 CSV（UTF-8）")


def main():
    common = argparse.ArgumentParser(add_help=False)
    add_common_args(common)

    ap = argparse.ArgumentParser(parents=[common], description="CSV/CSP 台架测试与速度环增益整定")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("plain", "csv", "csp", "timeout", "sweep"):
        sub.add_parser(name, parents=[common])
    args = ap.parse_args()

    for key, value in DEFAULTS.items():
        if not hasattr(args, key):
            setattr(args, key, value)
    args.gains = [float(g) for g in str(args.gains).split(",") if g.strip()]
    if args.allow_low_bus:
        args.min_bus = 0.0

    run = Runner(args)
    try:
        run.check_ready()
        {"plain": run.cmd_plain,
         "csv": run.cmd_csv,
         "csp": run.cmd_csp,
         "timeout": run.cmd_timeout,
         "sweep": run.cmd_sweep}[args.cmd]()
    except KeyboardInterrupt:
        print("interrupted")
    except EpError as exc:
        print("EP ERROR: %s" % exc)
    except SystemExit as exc:
        print(exc)
    finally:
        run.summary()
        run.write_log()
        run.close()


if __name__ == "__main__":
    main()
