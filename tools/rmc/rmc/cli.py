"""rmc 命令行入口。

用法（连接选项可以写在子命令前面或后面）：
  rmc                                              # 交互 shell
  rmc tree | dump | descriptor
  rmc get config.i_max
  rmc set config.i_max 3.5
  rmc call action.save
  rmc monitor telemetry.i_q telemetry.position --rate 100
  rmc scan | probe
  rmc backup cfg.json | restore cfg.json [--yes] [--save]
  rmc liveplotter telemetry.position --window 10

连接选项：
  CAN（默认，出货板）：--interface slcan|pcan|socketcan|gs_usb|… --channel COM3
  开发板：--port COM7（CDC）或 --usb（vendor interface）
"""

import argparse
import json
import sys

from . import ops
from .device import connect
from .discovery import discover_devices
from .errors import EpError
from .format import format_value, print_restore, print_tree
from .protocol import (
    DEFAULT_CAN_BITRATE, DEFAULT_CAN_DATA_BITRATE, DEFAULT_CAN_INTERFACE,
    DEFAULT_CAN_MASTER_ID, DEFAULT_CAN_NODE_ID, DEFAULT_EP_IN, DEFAULT_EP_OUT,
    DEFAULT_PID, DEFAULT_USB_INTERFACE, DEFAULT_VID, PROTOCOL_VERSION,
)
from .version import __version__
from .transport.can import CanBusManager


def _add_connection_args(parser, suppress=False):
    """suppress=True 给 subparser 用：没写的选项不要覆盖主 parser 的值。"""
    default = argparse.SUPPRESS if suppress else None
    parser.add_argument("--can", action="store_true", default=argparse.SUPPRESS
                        if suppress else False,
                        help="走 CAN 的 CYBERBEAST SDO（默认；仅为兼容 mc_ep.py）")
    parser.add_argument("--port", default=default,
                        help="CDC 串口（开发/台架板），例如 COM7 或 /dev/ttyACM0")
    parser.add_argument("--usb", action="store_true",
                        default=argparse.SUPPRESS if suppress else False,
                        help="vendor interface 的 bulk 端点（开发/台架板）")
    parser.add_argument("--vid", type=lambda s: int(s, 0),
                        default=DEFAULT_VID if not suppress else argparse.SUPPRESS,
                        help="USB VID（默认 0x%04X）" % DEFAULT_VID)
    parser.add_argument("--pid", type=lambda s: int(s, 0),
                        default=DEFAULT_PID if not suppress else argparse.SUPPRESS,
                        help="USB PID（默认 0x%04X）" % DEFAULT_PID)
    parser.add_argument("--usb-interface", type=int,
                        default=DEFAULT_USB_INTERFACE if not suppress else argparse.SUPPRESS,
                        help="vendor interface 号（默认 2）")
    parser.add_argument("--ep-out", type=lambda s: int(s, 0),
                        default=DEFAULT_EP_OUT if not suppress else argparse.SUPPRESS)
    parser.add_argument("--ep-in", type=lambda s: int(s, 0),
                        default=DEFAULT_EP_IN if not suppress else argparse.SUPPRESS)
    parser.add_argument("--baud", type=int,
                        default=115200 if not suppress else argparse.SUPPRESS,
                        help="CDC 串口速率（默认 115200）")
    parser.add_argument("--timeout", type=float,
                        default=1.0 if not suppress else argparse.SUPPRESS,
                        help="单次请求超时秒数（默认 1.0）")
    parser.add_argument("--interface", default=default,
                        help="CAN 后端：slcan（CANable，默认）/pcan/socketcan/"
                             "gs_usb/kvaser/vector/…（python-can 的名字）")
    parser.add_argument("--channel", default=default,
                        help="CAN 通道：slcan=COMx / pcan=PCAN_USBBUS1 / "
                             "socketcan=can0；不给时按后端猜")
    parser.add_argument("--bitrate", type=int,
                        default=DEFAULT_CAN_BITRATE if not suppress else argparse.SUPPRESS,
                        help="仲裁域速率：Classic 下等于 config.can_baud（默认 1 Mbit/s）")
    parser.add_argument("--fd", action="store_true",
                        default=argparse.SUPPRESS if suppress else False,
                        help="CAN FD（config.can_baud > 1M）")
    parser.add_argument("--data-bitrate", type=int,
                        default=DEFAULT_CAN_DATA_BITRATE if not suppress else argparse.SUPPRESS,
                        help="FD 数据域速率，等于 config.can_baud（默认 5 Mbit/s）")
    parser.add_argument("--can-id", type=int,
                        default=DEFAULT_CAN_NODE_ID if not suppress else argparse.SUPPRESS,
                        help="设备节点 ID，等于 config.can_id（默认 1）")
    parser.add_argument("--master-id", type=lambda s: int(s, 0),
                        default=DEFAULT_CAN_MASTER_ID if not suppress else argparse.SUPPRESS,
                        help="本机主站 ID（默认 0xFE）")
    parser.add_argument("--tty-baudrate", type=int,
                        default=115200 if not suppress else argparse.SUPPRESS,
                        help="slcan 串口速率（默认 115200，适配器固件固定值）")
    parser.add_argument("-v", "--verbose", action="count",
                        default=argparse.SUPPRESS if suppress else 0)


def build_parser():
    common = argparse.ArgumentParser(add_help=False)
    _add_connection_args(common, suppress=True)

    parser = argparse.ArgumentParser(
        prog="rmc",
        description="robot_mc endpoint 上位机（CAN 优先；兼容 mc_ep.py 的开发板 USB 入口）\n"
                    "不带子命令时进入交互 shell。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
        parents=[common])
    parser.add_argument("--version", action="version",
                        version="rmc %s (protocol %d)" % (__version__, PROTOCOL_VERSION))

    sub = parser.add_subparsers(dest="cmd")

    def add(name, help_text):
        return sub.add_parser(name, help=help_text, parents=[common])

    add("shell", "交互 shell（默认）")
    add("tree", "打印设备自带的参数树")

    p = add("dump", "读取所有可读参数")
    p.add_argument("--persist", action="store_true", help="只导出 persist 参数")
    p.add_argument("--json", default=None, help="另存为 JSON 文件")

    p = add("descriptor", "打印原始 JSON 描述符")
    p.add_argument("--raw", action="store_true", help="输出紧凑 JSON")

    p = add("scan", "只听 CAN 总线，列出节点与消息（不发帧）")
    p.add_argument("--seconds", type=float, default=2.0)

    p = add("probe", "链路诊断：监听总线并试读描述符")
    p.add_argument("--seconds", type=float, default=2.0)

    p = add("get", "读一个或多个参数")
    p.add_argument("names", nargs="+")

    p = add("set", "写一个参数（打印读回的实际值）")
    p.add_argument("name")
    p.add_argument("value")
    p.add_argument("--no-verify", action="store_true",
                   help="只发写请求，不读回（CAN ACK 无状态，不推荐）")

    p = add("call", "执行一个 action.* 端点")
    p.add_argument("names", nargs="+")
    p.add_argument("--read", action="store_true",
                   help="用“读”请求触发（ODrive/JointSDK 的做法；固件两种都接受）")

    p = add("monitor", "周期读取若干参数")
    p.add_argument("names", nargs="+")
    p.add_argument("--rate", type=float, default=10.0, help="刷新率 Hz（默认 10）")
    p.add_argument("--count", type=int, default=None, help="采多少行后退出（默认一直跑）")

    p = add("backup", "把 persist 参数导出为 JSON")
    p.add_argument("file")

    p = add("restore", "把 JSON 里的 persist 参数写回设备（默认 dry-run）")
    p.add_argument("file")
    p.add_argument("--yes", action="store_true", help="真的写入（默认只显示 diff）")
    p.add_argument("--save", action="store_true", help="写完后调用 action.save")

    def add_liveplotter(name, help_text):
        p = sub.add_parser(name, parents=[common], help=help_text)
        p.add_argument("names", nargs="+")
        p.add_argument("--rate", type=float, default=20.0)
        p.add_argument("--window", type=float, default=10.0, help="显示窗口秒数")
        p.add_argument("--duration", type=float, default=None, help="运行多少秒后退出")
        return p

    add_liveplotter("liveplotter", "实时曲线（需要 matplotlib）")
    add_liveplotter("plot", argparse.SUPPRESS).set_defaults(cmd="liveplotter")

    add("version", "打印版本")
    return parser


def _get(args, name, default):
    value = getattr(args, name, None)
    return default if value is None else value


def connection_kwargs(args):
    """CLI 选项 -> rmc.connect() 的关键字参数。"""
    port = _get(args, "port", None)
    use_usb = bool(_get(args, "usb", False))
    timeout = _get(args, "timeout", 1.0)
    if port and use_usb:
        raise EpError("--port 和 --usb 只能选一个")
    if bool(_get(args, "can", False)) and (port or use_usb):
        raise EpError("--can 与 --port/--usb 互斥")
    if port:
        return {"port": port, "timeout": timeout, "baud": _get(args, "baud", 115200)}
    if use_usb:
        return {
            "usb": True,
            "timeout": timeout,
            "vid": _get(args, "vid", DEFAULT_VID),
            "pid": _get(args, "pid", DEFAULT_PID),
            "usb_interface": _get(args, "usb_interface", DEFAULT_USB_INTERFACE),
            "ep_out": _get(args, "ep_out", DEFAULT_EP_OUT),
            "ep_in": _get(args, "ep_in", DEFAULT_EP_IN),
        }
    return {
        "interface": _get(args, "interface", DEFAULT_CAN_INTERFACE),
        "channel": _get(args, "channel", None),
        "bitrate": _get(args, "bitrate", DEFAULT_CAN_BITRATE),
        "fd": bool(_get(args, "fd", False)),
        "data_bitrate": _get(args, "data_bitrate", DEFAULT_CAN_DATA_BITRATE),
        "node_id": _get(args, "can_id", DEFAULT_CAN_NODE_ID),
        "master_id": _get(args, "master_id", DEFAULT_CAN_MASTER_ID),
        "timeout": timeout,
        "tty_baudrate": _get(args, "tty_baudrate", 115200),
    }


def _open_device(args, fetch=True):
    return connect(fetch=fetch, **connection_kwargs(args))


# --------------------------------------------------------------------------
# 子命令
# --------------------------------------------------------------------------


def cmd_tree(dev, args):
    print_tree(dev)
    return 0


def cmd_dump(dev, args):
    rows = {}
    for ep in dev.endpoints:
        if ep["type"] in ("void", "function") or ep["access"] not in ("r", "rw"):
            continue
        if getattr(args, "persist", False) and not ep.get("persist"):
            continue
        try:
            value = dev.read(ep["name"])
            rows[ep["name"]] = value
            print("%-36s %s" % (ep["name"], format_value(value)))
        except EpError as e:
            print("%-36s <错误: %s>" % (ep["name"], e))
    if getattr(args, "json", None):
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(rows, handle, indent=2, ensure_ascii=False)
        print("已写入 %s（%d 项）" % (args.json, len(rows)), file=sys.stderr)
    return 0


def cmd_get(dev, args):
    values = []
    for name in args.names:
        value = dev.read(name)
        values.append((name, value))
    if len(values) == 1:
        print(format_value(values[0][1]))
    else:
        for name, value in values:
            print("%-36s %s" % (name, format_value(value)))
    return 0


def cmd_set(dev, args):
    ep = dev._entry(args.name)
    from .protocol import parse_value
    try:
        requested = parse_value(args.value, ep["type"])
    except (TypeError, ValueError) as e:
        raise EpError(str(e))
    actual = dev.write(args.name, requested, verify=not args.no_verify)
    print(format_value(actual) if actual is not None else "OK")
    if actual is not None and actual != requested:
        print("注意: 请求 %s，设备读回 %s（被限幅或被拒绝）"
              % (format_value(requested), format_value(actual)), file=sys.stderr)
    return 0


def cmd_call(dev, args):
    for name in args.names:
        dev.call(name, via_read=args.read)
    print("OK")
    return 0


def cmd_monitor(dev, args):
    ops.monitor(dev, args.names, rate=args.rate, count=args.count)
    return 0


def cmd_descriptor(dev, args):
    if args.raw:
        print(json.dumps(dev.endpoints, separators=(",", ":")))
    else:
        print(json.dumps(dev.endpoints, indent=2, ensure_ascii=False))
    print("// %d endpoints, CRC16=0x%04X" % (len(dev.endpoints), dev.json_crc),
          file=sys.stderr)
    return 0


def cmd_scan(args):
    kwargs = connection_kwargs(args)
    if kwargs.get("port") or kwargs.get("usb"):
        raise EpError("scan 只在 --can 下可用")
    kwargs.pop("timeout", None)
    by_source = ops.scan(args.seconds, **kwargs)
    ops.print_scan(by_source, kwargs.get("node_id"))
    return 0


def cmd_probe(dev, args):
    return 0 if ops.probe(dev, seconds=args.seconds) else 3


def cmd_backup(dev, args):
    data = dev.backup()
    with open(args.file, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False)
    print("已写入 %s（%d 项，描述符 CRC %s）"
          % (args.file, len(data["values"]), data["descriptor_crc"]))
    if data.get("errors"):
        print("有 %d 项读取失败（已记录在 errors 里）" % len(data["errors"]),
              file=sys.stderr)
    return 0


def cmd_restore(dev, args):
    with open(args.file, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    results = dev.restore(data, dry_run=not args.yes)
    changed = print_restore(results)
    if not args.yes:
        print("dry-run：加 --yes 才会写入；再加 --save 写 flash。")
        return 0
    if args.save and changed:
        if dev.save():
            print("已调用 action.save。")
        else:
            print("固件没有 action.save。", file=sys.stderr)
    return 0


def cmd_liveplotter(dev, args):
    from . import liveplot
    liveplot.run(dev, args.names, rate=args.rate, window=args.window,
                 duration=args.duration)
    return 0


def cmd_shell(args):
    from .shell import ShellSession, launch_shell

    kwargs = connection_kwargs(args)
    can_manager = CanBusManager()
    explicit_port = bool(_get(args, "port", None))
    explicit_usb = bool(_get(args, "usb", False))
    devices, errors = discover_devices(
        kwargs, include_can=not (explicit_port or explicit_usb),
        include_usb=True, port=_get(args, "port", None),
        cdc=not explicit_usb, vendor=not explicit_port,
        seconds=1.0, can_manager=can_manager)
    session = ShellSession(connect_kwargs=kwargs, can_manager=can_manager)
    session.discovery_errors = errors
    session.discovery_options = {
        "include_can": not (explicit_port or explicit_usb),
        "include_usb": True,
        "cdc": not explicit_usb,
        "vendor": not explicit_port,
    }
    for device in devices:
        session.register(device)
    launch_shell(session)
    return 0


def cmd_version():
    print("rmc %s (protocol %d)" % (__version__, PROTOCOL_VERSION))
    return 0


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    cmd = getattr(args, "cmd", None) or "shell"

    if cmd == "version":
        return cmd_version()
    if cmd == "shell":
        return cmd_shell(args)
    if cmd == "scan":
        try:
            return cmd_scan(args)
        except EpError as e:
            print("错误: %s" % e, file=sys.stderr)
            return 2

    dev = None
    try:
        # probe 必须先能监听/诊断，不能被“描述符读取失败”挡在门外
        dev = _open_device(args, fetch=(cmd != "probe"))
        handlers = {
            "tree": cmd_tree,
            "dump": cmd_dump,
            "descriptor": cmd_descriptor,
            "get": cmd_get,
            "set": cmd_set,
            "call": cmd_call,
            "monitor": cmd_monitor,
            "probe": cmd_probe,
            "backup": cmd_backup,
            "restore": cmd_restore,
            "liveplotter": cmd_liveplotter,
        }
        handler = handlers.get(cmd)
        if handler is None:
            print("未知子命令: %s" % cmd, file=sys.stderr)
            return 2
        return handler(dev, args)
    except EpError as e:
        print("错误: %s" % e, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 0
    finally:
        if dev is not None:
            try:
                dev.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
