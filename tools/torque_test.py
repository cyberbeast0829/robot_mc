#!/usr/bin/env python3
"""
纯力矩加载测试：进入 MOTOR_MODE 后只改 command.t_ff（kp/kd 保持 0）。

用法:
    python tools/torque_test.py --port COM7
    python tools/torque_test.py --port COM7 --torques 0,2,4,6 --dwell 5

前提:
    * 已标定；**必须接负载/测功机**——纯力矩没有速度环，空载会一直加速；
    * 门极随时可以用 `mc_ep.py call action.idle_mode` 或断电切断。

命令租约（重要）:
    `command.*` 的每次写入都会把 controller.timeout 清零，和 CAN 帧一样；
    `config.can_timeout`（毫秒）内没有新的写入，控制 ISR 就会把命令全部清零。
    所以本脚本**不改** can_timeout，而是按它的值自动把刷新率提到至少 3 次/超时
    窗口：主机掉线 → 固件自己卸力。can_timeout = 0 时没有这层保护，脚本会警告。
"""

import argparse
import time

from mc_ep import Device, EpError

MOTOR_MODE = 2


def main():
    ap = argparse.ArgumentParser(description="纯力矩加载测试")
    ap.add_argument("--port", required=True, help="串口，例如 COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=1.0)
    ap.add_argument("--torques", default="0,1,2,3,4",
                    help="依次施加的输出轴力矩 N·m，逗号分隔（默认 0,1,2,3,4）")
    ap.add_argument("--dwell", type=float, default=3.0, help="每一档保持秒数")
    ap.add_argument("--rate", type=float, default=20.0,
                    help="刷新/采样频率 Hz（默认 20；can_timeout 较小时会自动提高）")
    args = ap.parse_args()

    torques = [float(x) for x in args.torques.split(",")]
    dev = Device(args.port, args.baud, args.timeout)
    dev.fetch_descriptor()

    i_max = dev.read("config.i_max")
    kt = dev.read("config.kt")
    gr = dev.read("config.gr")
    fs = dev.read("config.mit_tau_max") or 0.0
    t_phys = i_max * kt * gr                       # 电流限幅决定的物理可达力矩
    t_max = fs if fs > 0 else t_phys               # 命令量程（固件钳位用）
    print("I_MAX=%.2f A  KT=%.4f  GR=%.3f  →  可达力矩约 %.2f N·m，命令量程 ±%.2f N·m"
          % (i_max, kt, gr, t_phys, t_max))
    over = [t for t in torques if abs(t) > t_max + 1e-3]
    if over:
        print("警告: %s 超过命令量程，会被固件限幅到 ±%.2f N·m"
              % (", ".join("%g" % t for t in over), t_max))
    beyond = [t for t in torques if abs(t) > t_phys + 1e-3]
    if beyond:
        print("注意: %s 超过可达力矩 %.2f N·m，电流环会饱和（i_q 钳在 i_max），"
              "实测力矩不会更大" % (", ".join("%g" % t for t in beyond), t_phys))

    can_timeout = int(dev.read("config.can_timeout"))
    if can_timeout > 0:
        # 一个超时窗口内至少刷 3 次，留出丢帧/调度抖动的余量
        rate = max(args.rate, 3.0 * 1000.0 / can_timeout)
        print("命令租约 %d ms → 刷新率 %.1f Hz（主机掉线后固件会自动卸力）"
              % (can_timeout, rate))
    else:
        rate = args.rate
        print("警告: config.can_timeout = 0，没有命令租约；主机掉线后力矩会一直保持")
        print("      建议先 `set config.can_timeout 500` 再跑本脚本")

    entered = False
    try:
        dev.call("action.idle_mode")          # 关门极、清锁存故障
        time.sleep(0.2)
        if dev.read("telemetry.gate_drive"):
            raise SystemExit("门极仍然使能，已中止")

        dev.call("action.motor_mode")
        time.sleep(0.3)                        # 等 ISR 完成状态切换、门极使能
        entered = True
        mode = int(dev.read("telemetry.mode"))
        if mode != MOTOR_MODE:
            raise SystemExit("没有进入 MOTOR_MODE（当前 mode=%d），检查故障/标定" % mode)
        if not dev.read("telemetry.gate_drive"):
            raise SystemExit("已在 MOTOR_MODE 但门极未使能，检查故障")

        # 进入电机模式时固件已经把命令清零；这里显式再写一遍，语义更清楚
        for name in ("command.t_ff", "command.p_des", "command.v_des",
                     "command.kp", "command.kd"):
            dev.write(name, 0.0)
        print("已进入 MOTOR_MODE，开始加载测试\n")

        for t in torques:
            actual = dev.write("command.t_ff", t)
            print("t_ff 设定 %+.2f N·m（实际 %+.2f）" % (t, actual))
            t0 = time.time()
            while time.time() - t0 < args.dwell:
                # 每轮重写同一个值：既刷新命令租约，也拿到限幅后的真实值
                dev.write("command.t_ff", t)
                fault = int(dev.read("telemetry.fault"))
                if fault:
                    raise SystemExit("\n故障位 0x%X，已停止" % fault)
                print("  i_q=%6.2f A  i_q_des=%6.2f A  torque=%6.2f N·m  v_bus=%5.1f V"
                      % (dev.read("telemetry.i_q"), dev.read("telemetry.i_q_des"),
                         dev.read("telemetry.torque"), dev.read("telemetry.v_bus")),
                      end="\r", flush=True)
                time.sleep(1.0 / rate)
            print()
    except KeyboardInterrupt:
        print("\n中断")
    except EpError as e:
        print("\n通信错误: %s" % e)
    except Exception as e:                     # 拔线抛的是 serial.SerialException
        print("\n异常: %s" % e)
    finally:
        ok = True
        try:
            dev.write("command.t_ff", 0.0)
            dev.call("action.idle_mode")
        except Exception as e:
            ok = False
            print("\n!!! 卸力失败（%s）" % e)
            print("!!! 链路已断：力矩可能仍在输出，请立即切断动力电或按下急停")
        try:
            dev.close()
        except Exception:
            pass
        if ok:
            print("已卸力并回菜单" if entered else "已回菜单")


if __name__ == "__main__":
    main()
