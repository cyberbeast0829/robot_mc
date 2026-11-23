"""shell 命名空间（不进入 IPython 主循环）。"""

import pytest

from fake_device import MemoryLink
from rmc.device import Device
from rmc.shell import HELP_TEXT, ShellSession, build_namespace
from rmc.shell import ConnectionMonitor, print_device_status


def make_session():
    dev = Device(link=MemoryLink())
    dev.fetch_descriptor()
    return ShellSession(dev, {})


def test_namespace_contains_helpers():
    session = make_session()
    namespace = build_namespace(session, lambda: {})
    assert namespace["m0"] is session.device.root
    assert "dev" not in namespace
    for key in ("help", "info", "tree", "dump", "watch", "monitor", "scan",
                "probe", "backup", "restore", "save", "plot", "connect",
                "disconnect", "devices"):
        assert key in namespace, key
    assert "m0.<TAB>" in HELP_TEXT


def test_no_device_placeholder():
    namespace = build_namespace(ShellSession(None, {}), lambda: {})
    assert "m0" not in namespace
    assert "dev" not in namespace
    assert callable(namespace["connect"])


def test_broken_descriptor_gets_placeholder():
    from rmc.errors import EpError
    from rmc.shell import _NoDevice

    class BrokenLink(MemoryLink):
        def fetch_descriptor(self):
            raise EpError("no descriptor")

    dev = Device(link=BrokenLink())
    namespace = build_namespace(ShellSession(dev, {}), lambda: {})
    assert isinstance(namespace["m0"], _NoDevice)


def test_tree_and_dump(capsys):
    session = make_session()
    namespace = build_namespace(session, lambda: {})
    namespace["tree"]()
    out = capsys.readouterr().out
    assert "[config]" in out
    assert "config.f" in out

    namespace["dump"]()
    out = capsys.readouterr().out
    assert "config.f" in out
    assert "3.5" in out
    assert "action.save" not in out        # function 端点不能“读”


def test_backup_and_save(capsys):
    session = make_session()
    namespace = build_namespace(session, lambda: {})
    data = namespace["backup"]()
    assert set(data["values"]) == {"config.f", "config.b"}

    namespace["save"]()
    assert session.device.link.calls == [("action.save", False)]


def test_watch_accepts_proxy_nodes(capsys):
    session = make_session()
    namespace = build_namespace(session, lambda: {})
    namespace["watch"](namespace["m0"].config.f, rate=0, count=2)
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 2
    assert "config.f" in lines[0]
    assert "3.5" in lines[0]


def test_info(capsys):
    session = make_session()
    namespace = build_namespace(session, lambda: {})
    namespace["info"]()
    out = capsys.readouterr().out
    assert "transport" in out
    assert "descriptor_crc" in out
    assert "serial_number" in out
    assert "fw_version_unreleased" in out
    assert "hw_version_variant" in out


def _device_with_serial(serial_number):
    params = dict(MemoryLink().params)
    params[30] = ("system.serial_number", "uint64", "r", serial_number)
    dev = Device(link=MemoryLink(params))
    dev.fetch_descriptor()
    return dev


def test_serial_number_assigns_stable_names_after_reconnect():
    session = ShellSession(_device_with_serial(0x1234))
    assert list(session.devices) == ["m0"]

    session.disconnect("m0")
    name, reconnected, added = session.register(_device_with_serial(0x1234))
    assert (name, added) == ("m0", True)
    assert reconnected.serial_number_hex == "0x0000000000001234"

    name, second, added = session.register(_device_with_serial(0x5678))
    assert (name, added) == ("m1", True)
    assert list(session.devices) == ["m0", "m1"]


def test_duplicate_serial_is_merged_and_connection_is_colored(capsys):
    session = ShellSession(_device_with_serial(0xABCDEF))
    duplicate = _device_with_serial(0xABCDEF)
    name, existing, added = session.register(duplicate)
    assert (name, added) == ("m0", False)
    assert existing is session.devices["m0"]
    assert list(session.devices) == ["m0"]

    print_device_status(name, existing)
    output = capsys.readouterr().out
    assert "\x1b[" in output
    assert "0x0000000000ABCDEF" in output
    assert "fw=1.2.3.2" in output
    assert "hw=4.2.89" in output


def test_missing_serial_uses_transport_identity_and_displays_unknown(capsys):
    params = {ep_id: entry for ep_id, entry in MemoryLink().params.items()
              if ep_id != 30}
    device = Device(link=MemoryLink(params))
    device.fetch_descriptor()
    session = ShellSession(device)
    assert device.serial_number_hex is None

    print_device_status("m0", device)
    output = capsys.readouterr().out
    assert "serial=\x1b[33munknown" in output

    session.disconnect("m0")
    reconnected = Device(link=MemoryLink(params))
    reconnected.fetch_descriptor()
    assert session.register(reconnected)[0] == "m0"


def test_watch_routes_mN_prefixed_names(capsys):
    session = ShellSession(_device_with_serial(0x1234))
    namespace = build_namespace(session, lambda: {})
    namespace["watch"]("m0.config.f", rate=0, count=1)
    output = capsys.readouterr().out
    assert "config.f" in output
    assert "3.5" in output


def test_shell_connect_and_disconnect_update_namespaces(capsys):
    session = ShellSession(_device_with_serial(0x1234))
    user_ns = {}
    namespace = build_namespace(session, lambda: user_ns)
    user_ns.update(namespace)

    second = _device_with_serial(0x5678)
    session.connect = lambda **_kwargs: session.register(second)
    namespace["connect"](node_id=2)
    assert user_ns["m1"] is second.root

    namespace["disconnect"](user_ns["m0"])
    assert "<m0 未连接>" == repr(user_ns["m0"])
    first_reconnected = _device_with_serial(0x1234)
    session.connect = lambda **_kwargs: session.register(first_reconnected)
    namespace["connect"](interface="virtual")
    assert user_ns["m0"] is first_reconnected.root
    capsys.readouterr()


def test_connection_monitor_reports_disconnect_and_reconnect(capsys, monkeypatch):
    class FlakyLink(MemoryLink):
        online = True
        is_can = True
        node_id = 1

        def read_value(self, ep_id, size):
            if not self.online:
                from rmc.errors import EpError
                raise EpError("link down")
            return super().read_value(ep_id, size)

    class Manager:
        closed = 0

        def close(self):
            self.closed += 1

    link = FlakyLink()
    first = Device(link=link)
    first.fetch_descriptor()
    manager = Manager()
    session = ShellSession(first, can_manager=manager)
    user_ns = {"m0": first.root}
    monitor = ConnectionMonitor(session, lambda: user_ns)
    monkeypatch.setattr("rmc.shell.discover_devices", lambda *_a, **_k: ([], []))

    link.online = False
    monitor.poll_once()
    assert list(session.devices) == ["m0"]
    monitor.poll_once()
    assert not session.devices
    assert repr(user_ns["m0"]) == "<m0 未连接>"
    assert manager.closed == 1
    assert "m0 传输已断开" in capsys.readouterr().out

    reconnected = _device_with_serial(0x0123456789ABCDEF)
    monkeypatch.setattr(
        "rmc.shell.discover_devices", lambda *_a, **_k: ([reconnected], []))
    monitor.poll_once()
    assert list(session.devices) == ["m0"]
    assert user_ns["m0"] is reconnected.root
    output = capsys.readouterr().out
    assert "已连接" in output
    assert "fw=1.2.3.2" in output
    assert "hw=4.2.89" in output
