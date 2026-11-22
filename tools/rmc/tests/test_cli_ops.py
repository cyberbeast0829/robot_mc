"""CLI 子命令接线（用 MemoryLink 假设备，不碰真实总线）。"""

import json

import pytest

from fake_device import MemoryLink
from rmc import cli
from rmc.device import Device


@pytest.fixture
def memory_device(monkeypatch):
    dev = Device(link=MemoryLink())
    dev.fetch_descriptor()
    monkeypatch.setattr(cli, "connect", lambda **kwargs: dev)
    return dev


def test_get(memory_device, capsys):
    assert cli.main(["get", "config.f"]) == 0
    assert capsys.readouterr().out.strip() == "3.5"


def test_get_many(memory_device, capsys):
    assert cli.main(["get", "config.f", "config.i8"]) == 0
    out = capsys.readouterr().out
    assert "3.5" in out
    assert "-7" in out


def test_set_and_readback(memory_device, capsys):
    assert cli.main(["set", "config.f", "2.0"]) == 0
    assert capsys.readouterr().out.strip() == "2"
    assert memory_device.read("config.f") == 2.0


def test_set_clamped_warns(memory_device, capsys):
    assert cli.main(["set", "config.f", "99"]) == 0
    captured = capsys.readouterr()
    assert captured.out.strip() == "5"
    assert "限幅" in captured.err


def test_call(memory_device, capsys):
    assert cli.main(["call", "action.save"]) == 0
    assert memory_device.link.calls == [("action.save", False)]
    assert capsys.readouterr().out.strip() == "OK"


def test_tree(memory_device, capsys):
    assert cli.main(["tree"]) == 0
    out = capsys.readouterr().out
    assert "[config]" in out
    assert "config.f" in out


def test_dump_json(memory_device, tmp_path, capsys):
    target = tmp_path / "dump.json"
    assert cli.main(["dump", "--json", str(target)]) == 0
    data = json.loads(target.read_text(encoding="utf-8"))
    assert data["config.f"] == 3.5
    assert "action.save" not in data
    capsys.readouterr()


def test_descriptor_raw(memory_device, capsys):
    assert cli.main(["descriptor", "--raw"]) == 0
    captured = capsys.readouterr()
    entries = json.loads(captured.out)
    assert any(entry["name"] == "config.f" for entry in entries)
    assert "CRC16" in captured.err


def test_monitor_count(memory_device, capsys):
    assert cli.main(["monitor", "config.f", "--count", "3", "--rate", "0"]) == 0
    assert len(capsys.readouterr().out.splitlines()) == 3


def test_backup_restore(memory_device, tmp_path, capsys):
    path = tmp_path / "cfg.json"
    assert cli.main(["backup", str(path)]) == 0
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["values"]["config.f"] == 3.5

    memory_device.write("config.f", 1.0)
    capsys.readouterr()
    # 默认 dry-run，不写
    assert cli.main(["restore", str(path)]) == 0
    assert memory_device.read("config.f") == 1.0

    assert cli.main(["restore", str(path), "--yes"]) == 0
    assert memory_device.read("config.f") == 3.5
