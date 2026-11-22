"""CLI 参数解析与连接选项映射。"""

import pytest

from rmc.cli import build_parser, connection_kwargs, main
from rmc.errors import EpError
from rmc.protocol import (
    DEFAULT_CAN_BITRATE, DEFAULT_CAN_DATA_BITRATE, DEFAULT_CAN_INTERFACE,
    DEFAULT_CAN_MASTER_ID, DEFAULT_CAN_NODE_ID,
)

CONNECTION_NAMES = ("port", "usb", "interface", "channel", "bitrate", "fd",
                    "data_bitrate", "node_id", "master_id", "timeout",
                    "tty_baudrate", "vid", "pid", "usb_interface", "ep_out", "ep_in")


def test_defaults_are_can():
    args = build_parser().parse_args(["tree"])
    assert args.cmd == "tree"
    for name in CONNECTION_NAMES:
        assert not hasattr(args, name), name      # 未指定 -> SUPPRESS
    kwargs = connection_kwargs(args)
    assert kwargs["interface"] == DEFAULT_CAN_INTERFACE
    assert kwargs["bitrate"] == DEFAULT_CAN_BITRATE
    assert kwargs["data_bitrate"] == DEFAULT_CAN_DATA_BITRATE
    assert kwargs["node_id"] == DEFAULT_CAN_NODE_ID
    assert kwargs["master_id"] == DEFAULT_CAN_MASTER_ID
    assert "port" not in kwargs and "usb" not in kwargs


def test_options_before_and_after_subcommand():
    for argv in (["--interface", "pcan", "--channel", "PCAN_USBBUS1", "tree"],
                 ["tree", "--interface", "pcan", "--channel", "PCAN_USBBUS1"]):
        args = build_parser().parse_args(argv)
        kwargs = connection_kwargs(args)
        assert kwargs["interface"] == "pcan"
        assert kwargs["channel"] == "PCAN_USBBUS1"


def test_can_fd_options():
    args = build_parser().parse_args(
        ["--fd", "--data-bitrate", "5000000", "--can-id", "3", "get", "config.i_max"])
    kwargs = connection_kwargs(args)
    assert kwargs["fd"] is True
    assert kwargs["data_bitrate"] == 5000000
    assert kwargs["node_id"] == 3


def test_port_and_usb_are_exclusive():
    args = build_parser().parse_args(["--port", "COM7", "--usb", "tree"])
    with pytest.raises(EpError):
        connection_kwargs(args)


def test_can_and_usb_are_exclusive():
    args = build_parser().parse_args(["--can", "--port", "COM7", "tree"])
    with pytest.raises(EpError):
        connection_kwargs(args)


def test_probe_opens_without_fetching_descriptor(monkeypatch):
    """probe 的用途就是诊断“描述符读不到”，所以它不能被这一步挡住。"""
    from rmc.errors import EpError

    called = {}

    def fake_connect(**kwargs):
        called.update(kwargs)
        raise EpError("stop here")

    monkeypatch.setattr("rmc.cli.connect", fake_connect)
    assert main(["probe", "--seconds", "0"]) == 2
    assert called.get("fetch") is False


def test_port_transport():
    args = build_parser().parse_args(["--port", "COM7", "--baud", "921600", "tree"])
    kwargs = connection_kwargs(args)
    assert kwargs == {"port": "COM7", "timeout": 1.0, "baud": 921600}


def test_plot_alias():
    args = build_parser().parse_args(["plot", "config.i_max"])
    assert args.cmd == "liveplotter"


def test_usb_transport():
    args = build_parser().parse_args(["--usb", "tree"])
    kwargs = connection_kwargs(args)
    assert kwargs["usb"] is True
    assert "port" not in kwargs


def test_mc_ep_style_invocation_still_works():
    # 旧文档里的写法：python tools/mc_ep.py --can get config.i_max
    args = build_parser().parse_args(["--can", "get", "config.i_max"])
    assert args.cmd == "get"
    assert args.names == ["config.i_max"]
    assert connection_kwargs(args)["interface"] == DEFAULT_CAN_INTERFACE


def test_version(capsys):
    assert main(["version"]) == 0
    out = capsys.readouterr().out
    assert out.startswith("rmc ")
    assert "protocol 2" in out


def test_shell_discovers_multiple_devices_and_passes_session(monkeypatch):
    from fake_device import MemoryLink
    from rmc.device import Device

    discovered = [
        Device(link=MemoryLink()),
        Device(link=MemoryLink()),
    ]
    for device in discovered:
        device.fetch_descriptor()
    captured = {}

    def fake_discover(kwargs, **options):
        captured["kwargs"] = kwargs
        captured["options"] = options
        return discovered, []

    def fake_launch(session):
        captured["session"] = session

    monkeypatch.setattr("rmc.cli.discover_devices", fake_discover)
    monkeypatch.setattr("rmc.shell.launch_shell", fake_launch)
    assert main([]) == 0
    session = captured["session"]
    assert list(session.devices) == ["m0"]  # identical serial numbers are merged
    assert captured["options"]["include_can"] is True
    assert captured["options"]["include_usb"] is True


def test_shell_explicit_cdc_uses_only_requested_port(monkeypatch):
    captured = {}

    def fake_discover(kwargs, **options):
        captured["kwargs"] = kwargs
        captured["options"] = options
        return [], []

    monkeypatch.setattr("rmc.cli.discover_devices", fake_discover)
    monkeypatch.setattr("rmc.shell.launch_shell", lambda session: None)
    assert main(["--port", "COM7"]) == 0
    assert captured["options"]["include_can"] is False
    assert captured["options"]["port"] == "COM7"
    assert captured["options"]["vendor"] is False
