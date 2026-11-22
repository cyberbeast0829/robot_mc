#!/usr/bin/env python3
"""本机模拟设备（不需要硬件、不需要 CAN 适配器）。

把 `tests/fake_device.py` 挂在 python-can 的 **udp_multicast** 总线上，
这样另一个进程（`rmc` 命令 / shell）能通过正常的 CAN 传输访问它。

终端 1：
    python tools/rmc/tests/sim_device.py
终端 2：
    python -m rmc --interface udp_multicast --channel 239.74.163.10:28700 tree
    python -m rmc --interface udp_multicast --channel 239.74.163.10:28700 get config.f
    python -m rmc --interface udp_multicast --channel 239.74.163.10:28700
    python tools/mc_ep.py --can --interface udp_multicast \\
           --channel 239.74.163.10:28700 tree        # 验证 shim

需要：`pip install python-can msgpack`（udp_multicast 后端依赖 msgpack）。
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import can  # noqa: E402

from fake_device import FakeDevice  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="rmc 模拟设备（udp_multicast）")
    parser.add_argument("--channel", default="239.74.163.10",
                        help="组播地址（默认 239.74.163.10）")
    parser.add_argument("--port", type=int, default=28700, help="组播端口（默认 28700）")
    parser.add_argument("--fd", action="store_true", help="模拟 CAN FD 帧（可选）")
    args = parser.parse_args()

    try:
        bus = can.Bus(interface="udp_multicast", channel=args.channel, port=args.port,
                      bitrate=1000000)
    except can.CanInterfaceNotImplementedError:
        print("udp_multicast 需要 msgpack：pip install msgpack", file=sys.stderr)
        return 2
    fake = FakeDevice(bus, args.fd).start()
    endpoint = "%s:%d" % (args.channel, args.port)
    print("模拟设备已启动：udp_multicast %s，node=1，%s"
          % (endpoint, "FD" if args.fd else "Classic"), flush=True)
    print("另一个终端：", flush=True)
    print("  python -m rmc --interface udp_multicast --channel %s tree" % endpoint,
          flush=True)
    print("  python -m rmc --interface udp_multicast --channel %s" % endpoint,
          flush=True)
    print("Ctrl-C 退出。", flush=True)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        fake.stop()
        bus.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
