"""属性树代理 / tab 补全 / backup-restore。"""

import json

import pytest

from fake_device import MemoryLink
from rmc.device import Device
from rmc.errors import EpError


def make_device():
    dev = Device(link=MemoryLink())
    dev.fetch_descriptor()
    return dev


def test_attribute_tree_and_read():
    dev = make_device()
    assert "config" in dir(dev.root)
    assert "telemetry" in dir(dev.root)
    assert set(dir(dev.root.config)) == {"f", "u64", "i8", "b", "wo"}
    assert float(dev.config.f) == 3.5
    assert repr(dev.config.f) == "3.5"
    assert dev.config.i8 == -7
    assert dev.config.b == True          # noqa: E712  (代理的 __eq__ 会读) 


def test_assignment_writes():
    dev = make_device()
    dev.config.f = 4.0
    assert dev.read("config.f") == 4.0
    dev.config.b = "false"
    assert dev.read("config.b") is False
    # 超出范围由设备限幅（读回 5.0），客户端不报错
    dev.config.f = 99.0
    assert dev.read("config.f") == 5.0


def test_unknown_attribute_suggests():
    dev = make_device()
    with pytest.raises(AttributeError) as excinfo:
        _ = dev.config.ff
    assert "没有 config.ff" in str(excinfo.value)
    assert "f" in str(excinfo.value)


def test_action_call_and_repr():
    dev = make_device()
    # 动作端点的 repr 不能触发执行（读=执行）
    _ = repr(dev.action.save)
    assert dev.link.calls == []
    dev.action.save()
    assert dev.link.calls == [("action.save", False)]
    with pytest.raises(AttributeError):
        dev.action.save = 1
    with pytest.raises(TypeError):
        dev.action.save(1)


def test_write_only():
    dev = make_device()
    assert dev.write("config.wo", 1.5) is None
    with pytest.raises(EpError):
        dev.read("config.wo")


def test_backup_restore():
    dev = make_device()
    backup = dev.backup()
    assert backup["format"] == "rmc-backup"
    assert set(backup["values"]) == {"config.f", "config.b"}
    assert backup["values"]["config.f"] == 3.5
    # dry-run 不写
    data = {"values": {"config.f": 1.0, "config.b": False}}
    results = dev.restore(data, dry_run=True)
    assert {r["name"]: r["status"] for r in results} == {
        "config.f": "将写入", "config.b": "将写入"}
    assert dev.read("config.f") == 3.5

    results = dev.restore(json.loads(json.dumps(data)), dry_run=False)
    assert {r["name"]: r["status"] for r in results} == {
        "config.f": "已写入", "config.b": "已写入"}
    assert dev.read("config.f") == 1.0
    assert dev.read("config.b") is False

    # 再写一遍应当全是“相同”
    results = dev.restore(data, dry_run=False)
    assert all(r["status"] == "相同" for r in results)


def test_backup_fetches_descriptor_lazily():
    dev = Device(link=MemoryLink())
    data = dev.backup()
    assert data["values"]["config.f"] == 3.5


def test_restore_skips_unknown_and_readonly():
    dev = make_device()
    results = dev.restore({"values": {"config.nope": 1, "telemetry.t": 2.0}},
                          dry_run=True)
    status = {r["name"]: r["status"] for r in results}
    assert status == {"config.nope": "跳过", "telemetry.t": "跳过"}


def test_tab_completion_via_dir():
    """tab 补全依赖 `__dir__`（readline / rlcompleter 的机制）。"""
    import rlcompleter

    dev = make_device()
    completer = rlcompleter.Completer({"dev": dev.root})
    assert completer.complete("dev.co", 0) == "dev.config()"
    assert completer.complete("dev.config.i", 0) == "dev.config.i8"
    assert completer.complete("dev.action.", 0) == "dev.action.save("


def test_ipython_tab_completion():
    """IPython（rmc shell 用的）能看见代理树。"""
    pytest.importorskip("IPython")
    import warnings

    from IPython.core.interactiveshell import InteractiveShell

    dev = make_device()
    shell = InteractiveShell.instance()
    shell.user_ns["dev"] = dev.root

    def complete(text):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            return [item.text for item in shell.Completer.completions(text, len(text))]

    assert "config" in complete("dev.co")
    assert "i8" in complete("dev.config.i")
    assert "save" in complete("dev.action.")

