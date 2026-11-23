"""rmc 交互式 shell；每台设备按首次发现顺序命名为 m0、m1、……。"""

import json
import re
import threading

from . import ops
from .device import Device, connect as open_device
from .discovery import discover_devices
from .errors import EpError
from .format import format_value as _fmt, print_restore, print_tree
from .transport.can import CanBusManager
from .version import __version__


_RESET = "\x1b[0m"
_GREEN = "\x1b[32m"
_CYAN = "\x1b[36m"
_YELLOW = "\x1b[33m"

HELP_TEXT = """\
可用对象/函数：
  m0, m1, ...           每台设备的参数树（m0.<TAB> 补全）
  connect(...)          再连接一台设备；按 serial_number 去重并恢复原名称
  disconnect("m0")      断开设备；重新连接后原名称不变
  devices()             列出已连接设备、序列号、硬件与固件版本
  help()                显示本帮助
  info(device=None)     传输/固件/描述符信息；默认最近连接的设备
  tree(device=None)     打印参数树
  dump(device=None)     读取所有可读参数
  watch(...)            周期读取参数；例如 watch("m0.telemetry.i_q", rate=20)
  scan(seconds=2)       只听 CAN 总线，列出节点与消息
  probe(seconds=2)      链路诊断（监听 + 读描述符）
  backup(file=None)     persist 参数导出 JSON
  restore(file, ...)    回写 JSON（默认 dry-run）
  save(device=None)     调用 action.save（写 flash）
  plot("m0.telemetry.position", window=10)  实时曲线（需 matplotlib）

示例：
  m0.config.i_max
  m1.config.i_max = 3.5
  m0.action.save()
  watch("m0.telemetry.position", rate=20)
  connect(node_id=2, channel="COM4")
"""


class ShellSession:
    """管理多条设备连接，并按固件 serial_number 保持会话内名称稳定。"""

    def __init__(self, device=None, connect_kwargs=None, can_manager=None):
        self.connect_kwargs = dict(connect_kwargs or {})
        self.can_manager = can_manager or CanBusManager()
        self._lock = threading.RLock()
        self.devices = {}
        self._identity_names = {}
        self._next_index = 0
        self._selected_name = None
        self.discovery_errors = []
        self.discovery_options = {
            "include_can": True,
            "include_usb": True,
            "cdc": True,
            "vendor": True,
        }
        if device is not None:
            self.register(device)

    @property
    def device(self):
        """兼容单设备调用方；默认使用最近选择的在线设备。"""
        with self._lock:
            if self._selected_name in self.devices:
                return self.devices[self._selected_name]
            return next(iter(self.devices.values()), None)

    def register(self, device):
        with self._lock:
            key = device.identity_key()
            name = self._identity_names.get(key)
            if name is not None and name in self.devices:
                existing = self.devices[name]
                if existing is not device:
                    try:
                        device.close()
                    except Exception:
                        pass
                    self._selected_name = name
                    return name, existing, False
            if name is None:
                name = "m%d" % self._next_index
                self._next_index += 1
                self._identity_names[key] = name
            self.devices[name] = device
            self._selected_name = name
            return name, device, True

    def snapshot(self):
        with self._lock:
            return list(self.devices.items())

    def is_current(self, name, device):
        with self._lock:
            return self.devices.get(name) is device

    def connect(self, **kwargs):
        with self._lock:
            return self._connect_locked(**kwargs)

    def _connect_locked(self, **kwargs):
        params = dict(self.connect_kwargs)
        if any(key in kwargs for key in ("port", "usb", "interface")):
            for key in ("port", "usb", "interface", "channel"):
                params.pop(key, None)
        params.update(kwargs)
        if not params.get("port") and not params.get("usb"):
            can_args = {
                "interface": params.get("interface"),
                "channel": params.get("channel"),
                "bitrate": params.get("bitrate"),
                "fd": params.get("fd"),
                "data_bitrate": params.get("data_bitrate"),
                "node_id": params.get("node_id"),
                "master_id": params.get("master_id"),
                "timeout": params.get("timeout"),
                "tty_baudrate": params.get("tty_baudrate"),
            }
            can_args = {key: value for key, value in can_args.items()
                        if value is not None}
            device = Device(link=self.can_manager.link(**can_args))
            try:
                device.fetch_descriptor()
            except Exception:
                device.close()
                raise
        else:
            device = open_device(**params)
        return self.register(device)

    def disconnect(self, name):
        with self._lock:
            device = self.devices.pop(name, None)
            if device is None:
                raise EpError("设备 %s 当前未连接" % name)
            try:
                device.close()
            finally:
                if self._selected_name == name:
                    self._selected_name = next(iter(self.devices), None)
            return device

    def close(self):
        devices, self.devices = self.devices, {}
        for device in devices.values():
            try:
                device.close()
            except Exception:
                pass
        self.can_manager.close()

    def select(self, target=None):
        if isinstance(target, Device):
            return target
        if hasattr(target, "_device"):
            return target._device
        with self._lock:
            name = target or self._selected_name
            if name is None:
                raise EpError("还没有连接设备：先运行 connect(...)")
            if name not in self.devices:
                raise EpError("设备 %s 当前未连接：先运行 connect(...) 或 devices()" % name)
            self._selected_name = name
            return self.devices[name]


def _paint(text, color):
    return "%s%s%s" % (color, text, _RESET)


def _firmware_string(info):
    parts = [info.get("fw_version_" + key)
             for key in ("major", "minor", "revision")]
    if any(part is None for part in parts):
        return "unknown"
    version = ".".join(str(int(part)) for part in parts)
    unreleased = info.get("fw_version_unreleased")
    if unreleased is not None:
        version += "." + str(int(unreleased))
    return version


def _hardware_string(info):
    parts = [info.get("hw_version_" + key)
             for key in ("major", "minor", "variant")]
    if any(part is None for part in parts):
        return "unknown"
    return ".".join(str(int(part)) for part in parts)


def print_device_status(name, device, duplicate=False):
    info = device.info()
    serial = info.get("serial_number", "unknown")
    serial_text = (_paint(serial, _CYAN) if serial != "unknown"
                   else _paint("unknown", _YELLOW))
    prefix = "已发现" if duplicate else "已连接"
    print("%s %s | serial=%s | fw=%s | hw=%s | %s | %d endpoints | CRC 0x%04X"
          % (_paint("✓ " + prefix, _GREEN), name, serial_text,
             _firmware_string(info), _hardware_string(info), device.describe(),
             len(device.descriptor), device.json_crc))


def print_banner(session):
    print("rmc %s — robot_mc endpoint tool (protocol 2)" % __version__)
    devices = session.snapshot()
    if devices:
        for name, device in devices:
            print_device_status(name, device)
        print("m0.<TAB> 补全；输入 help() 查看用法，exit 退出。")
    else:
        print("未连接设备。可用 connect(...) 手动连接，或先检查 CAN/USB 设备。")
        print("输入 help() 查看用法，exit 退出。")
    for error in session.discovery_errors:
        print(_paint("发现设备时遇到问题: " + error, _YELLOW))


class _NoDevice:
    def __init__(self, name):
        self._name = name

    def __getattr__(self, attr):
        if attr.startswith("_"):
            raise AttributeError(attr)
        raise EpError("设备 %s 当前未连接：先运行 connect(...) 或 devices()" % self._name)

    def __repr__(self):
        return "<%s 未连接>" % self._name

    def __dir__(self):
        return []


def _watch_names(session, names, default_device=None):
    if not names:
        raise EpError("watch 需要至少一个参数名，例如 watch(\"m0.telemetry.i_q\", rate=20)")
    device = None
    paths = []
    for name in names:
        if hasattr(name, "_device"):
            current = name._device
            path = name.path
        else:
            text = str(name)
            match = re.match(r"^(m\d+)\.(.+)$", text)
            if match:
                current = session.select(match.group(1))
                path = match.group(2)
            else:
                current = session.select(default_device)
                path = text[4:] if text.startswith("dev.") else text
        if device is not None and current is not device:
            raise EpError("一次 watch/plot 只能读取同一台设备")
        device = current
        paths.append(path)
    return device, paths


def build_namespace(session, live_ns):
    """构造 shell 全局名；连接新设备时把 mN 写入实时 IPython namespace。"""

    def connect(**kwargs):
        name, device, added = session.connect(**kwargs)
        live_ns()[name] = device.root
        print_device_status(name, device, duplicate=not added)
        return device.root

    def disconnect(name):
        if hasattr(name, "_device"):
            name = next((candidate for candidate, device in session.devices.items()
                         if device is name._device), None)
        if not isinstance(name, str) or name is None:
            name = getattr(name, "__name__", str(name))
        session.disconnect(name)
        live_ns()[name] = _NoDevice(name)
        print("%s 已断开；名称保留，重连后仍使用该名称。" % name)

    def list_devices():
        devices = session.snapshot()
        if not devices:
            print("当前没有在线设备。")
        for name, device in devices:
            print_device_status(name, device)
        return dict(devices)

    def tree(target=None):
        print_tree(session.select(target))

    def dump(target=None):
        device = session.select(target)
        for ep in device.endpoints:
            if ep["type"] in ("void", "function") or ep["access"] not in ("r", "rw"):
                continue
            try:
                print("%-36s %s" % (ep["name"], _fmt(device.read(ep["name"]))))
            except EpError as error:
                print("%-36s <错误: %s>" % (ep["name"], error))

    def info(target=None):
        for key, value in session.select(target).info().items():
            print("  %-24s %s" % (key, value))

    def watch(*names, rate=10.0, count=None):
        device, paths = _watch_names(session, names)
        ops.monitor(device, paths, rate=rate, count=count)

    def scan(seconds=2.0, **kwargs):
        params = dict(session.connect_kwargs)
        params.update(kwargs)
        params.pop("port", None)
        params.pop("usb", None)
        can_args = {
            "interface": params.get("interface"),
            "channel": params.get("channel"),
            "bitrate": params.get("bitrate"),
            "fd": params.get("fd"),
            "data_bitrate": params.get("data_bitrate"),
            "node_id": params.get("node_id"),
            "master_id": params.get("master_id"),
            "timeout": params.get("timeout"),
            "tty_baudrate": params.get("tty_baudrate"),
        }
        can_args = {key: value for key, value in can_args.items()
                    if value is not None}
        counts = session.can_manager.link(**can_args).sniff(seconds)
        by_source = {}
        for (source, msgtype), count in counts.items():
            by_source.setdefault(source, {})[msgtype] = count
        ops.print_scan(by_source, params.get("node_id"))

    def probe(seconds=2.0, device=None):
        return ops.probe(session.select(device), seconds=seconds)

    def backup(file=None, device=None):
        data = session.select(device).backup()
        if file:
            with open(file, "w", encoding="utf-8") as handle:
                json.dump(data, handle, indent=2, ensure_ascii=False)
            print("已写入 %s（%d 项）" % (file, len(data["values"])))
            return None
        print("persist 参数 %d 项（未保存；backup(\"file.json\") 可写文件）"
              % len(data["values"]))
        return data

    def restore(file, yes=False, save=False, device=None):
        with open(file, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        selected = session.select(device)
        results = selected.restore(data, dry_run=not yes)
        changed = print_restore(results)
        if yes and save and changed:
            if selected.save():
                print("已调用 action.save。")
            else:
                print("固件没有 action.save。")
        elif not yes:
            print("dry-run：加 yes=True 才会写入；再加 save=True 写 flash。")

    def save(device=None):
        if not session.select(device).save():
            print("固件没有 action.save。")

    def plot(*names, device=None, **kwargs):
        from . import liveplot
        if names:
            selected, paths = _watch_names(session, names, default_device=device)
        else:
            selected, paths = session.select(device), []
        if not paths:
            raise EpError("plot 需要参数名，例如 plot(\"m0.telemetry.position\")")
        liveplot.run(selected, paths, **kwargs)

    def help_():
        print(HELP_TEXT)

    namespace = {
        "connect": connect,
        "disconnect": disconnect,
        "devices": list_devices,
        "help": help_,
        "info": info,
        "tree": tree,
        "dump": dump,
        "watch": watch,
        "monitor": watch,
        "scan": scan,
        "probe": probe,
        "backup": backup,
        "restore": restore,
        "save": save,
        "plot": plot,
        "rmc": __import__("rmc"),
    }
    for name, device in session.snapshot():
        try:
            namespace[name] = device.root
        except EpError:
            namespace[name] = _NoDevice(name)
    return namespace


class ConnectionMonitor:
    """后台探测已连接设备，并发现新出现的 CAN/USB 设备。"""

    def __init__(self, session, namespace, interval=1.0):
        self.session = session
        self.namespace = namespace
        self.interval = interval
        self._stop = threading.Event()
        self._thread = None
        self._failures = {}
        self._reported_errors = set()

    @staticmethod
    def _probe(device):
        names = device.by_name
        if "system.fw_version_major" in names:
            device.read("system.fw_version_major")
            return
        if "system.serial_number" in names:
            device.read("system.serial_number")
            return
        for endpoint in device.endpoints:
            if endpoint.get("type") not in ("void", "function") \
                    and endpoint.get("access") in ("r", "rw"):
                device.read(endpoint["name"])
                return
        device.link.fetch_descriptor()

    def poll_once(self):
        snapshot = self.session.snapshot()
        active_names = {name for name, _device in snapshot}
        for name in list(self._failures):
            if name not in active_names:
                self._failures.pop(name, None)
        for name, device in snapshot:
            try:
                self._probe(device)
            except (EpError, OSError) as error:
                failures = self._failures.get(name, 0) + 1
                self._failures[name] = failures
                if failures >= 2 and self.session.is_current(name, device):
                    self.session.disconnect(name)
                    self.namespace()[name] = _NoDevice(name)
                    self._failures.pop(name, None)
                    print("%s 传输已断开：%s" % (name, error), flush=True)
                    if getattr(device.link, "is_can", False) and not any(
                            getattr(other.link, "is_can", False)
                            for _other_name, other in self.session.snapshot()):
                        self.session.can_manager.close()
            else:
                self._failures.pop(name, None)

        devices = self.session.snapshot()
        exclude_transports = {device.describe() for _name, device in devices}
        exclude_nodes = {
            device.link.node_id for _name, device in devices
            if getattr(device.link, "is_can", False)
        }
        options = dict(self.session.discovery_options)
        options.update({
            "port": None,
            "seconds": 0.25,
            "can_manager": self.session.can_manager,
            "exclude_transports": exclude_transports,
            "exclude_nodes": exclude_nodes,
            "include_configured_node": False,
        })
        found, errors = discover_devices(self.session.connect_kwargs, **options)
        for error in errors:
            if error not in self._reported_errors:
                self._reported_errors.add(error)
                print(_paint("设备扫描提示: " + error, _YELLOW), flush=True)
            if error.startswith("CAN:") and not any(
                    getattr(device.link, "is_can", False)
                    for _name, device in self.session.snapshot()):
                self.session.can_manager.close()
        for device in found:
            try:
                name, registered, added = self.session.register(device)
            except (EpError, OSError) as error:
                try:
                    device.close()
                except (EpError, OSError):
                    pass
                message = "新设备连接失败: %s" % error
                if message not in self._reported_errors:
                    self._reported_errors.add(message)
                    print(_paint(message, _YELLOW), flush=True)
                continue
            if added:
                try:
                    self.namespace()[name] = registered.root
                    print_device_status(name, registered)
                except (EpError, OSError) as error:
                    self.session.disconnect(name)
                    self.namespace()[name] = _NoDevice(name)
                    message = "新设备连接失败: %s" % error
                    if message not in self._reported_errors:
                        self._reported_errors.add(message)
                        print(_paint(message, _YELLOW), flush=True)

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="rmc-connection-monitor",
                                        daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def _run(self):
        while not self._stop.is_set():
            self.poll_once()
            self._stop.wait(self.interval)


def launch_shell(session, close_on_exit=True):
    """进入 IPython 交互 shell（IPython 是 rmc 的必需依赖）。"""
    try:
        import colorama
        colorama.just_fix_windows_console()
        from IPython.terminal.embed import InteractiveShellEmbed
    except ImportError as error:
        raise EpError("交互 shell 需要 IPython 和 colorama：pip install ipython colorama (%s)"
                      % error)

    console = InteractiveShellEmbed.instance(banner1="")
    namespace = build_namespace(session, lambda: console.user_ns)
    print_banner(session)
    monitor = ConnectionMonitor(session, lambda: console.user_ns)
    monitor.start()
    try:
        console(local_ns=namespace, global_ns=namespace)
    except SystemExit:
        pass
    finally:
        monitor.stop()
        if close_on_exit:
            session.close()
