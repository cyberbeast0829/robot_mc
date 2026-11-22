"""CYBERBEAST CAN 传输的回归测试（python-can virtual 总线，不需要硬件）。"""

import can
import pytest

from fake_device import PARAMS, FakeDevice
from rmc.device import Device
from rmc.discovery import discover_devices
from rmc.errors import EpError
from rmc.protocol import CB_PRI_CONFIG, TYPE_CODES, cb_id
from rmc.shell import ShellSession
from rmc.transport.can import CanBusManager


def _run(fd):
    channel = "rmc_selftest_fd" if fd else "rmc_selftest"
    dev_bus = can.Bus(interface="virtual", channel=channel)
    fake = FakeDevice(dev_bus, fd).start()
    dev = None
    try:
        dev = Device(can_interface="virtual", can_channel=channel, can_fd=fd,
                     timeout=0.5)
        desc = dev.fetch_descriptor()
        assert len(desc) == len(PARAMS)
        assert dev.json_crc == fake.crc

        assert dev.read("config.f") == 3.5
        assert dev.read("config.i8") == -7
        assert dev.read("config.b") is True
        assert dev.read("config.u64") == 0x0102030405060708

        # Classic 下 8 字节要按 offset 分两帧读，FD 一次读完
        before = fake.sdo_read_requests
        dev.read("config.u64")
        rounds = fake.sdo_read_requests - before
        assert rounds == (1 if fd else 2)

        # 写后读回（固件把 float 限到 ±5）
        assert dev.write("config.f", 4.0) == 4.0
        assert dev.write("config.f", 99.0) == 5.0
        assert fake.values[10] == 5.0

        assert dev.write("config.u64", 0x1122334455667788) == 0x1122334455667788
        assert fake.values[11] == 0x1122334455667788
        assert dev.write("config.b", "false") is False

        # 只写端点：写成功但没有读回
        assert dev.write("config.wo", 1.5) is None
        assert fake.values[14] == 1.5

        dev.call("action.save")
        assert fake.actions == ["save"]

        with pytest.raises(EpError):
            dev.link.read_value(14, TYPE_CODES["float"][1])
        with pytest.raises(EpError):
            dev.write("telemetry.t", 5.0)
        assert fake.values[12] == -7
        assert fake.values[15] == 1.25
    finally:
        if dev is not None:
            dev.close()
        fake.stop()
        dev_bus.shutdown()


def test_classic_1mbit():
    _run(fd=False)


def test_can_fd():
    _run(fd=True)


def test_proxy_reads_over_can():
    """属性树跑的是同一条链路。"""
    channel = "rmc_proxy_test"
    bus = can.Bus(interface="virtual", channel=channel)
    fake = FakeDevice(bus, False).start()
    dev = None
    try:
        dev = Device(can_interface="virtual", can_channel=channel, timeout=0.5)
        dev.fetch_descriptor()
        assert float(dev.config.f) == 3.5
        dev.config.f = 2.0
        assert dev.read("config.f") == 2.0
        dev.action.save()
        assert fake.actions == ["save"]
    finally:
        if dev is not None:
            dev.close()
        fake.stop()
        bus.shutdown()


def test_multiple_nodes_share_one_can_bus_and_use_serial_identity():
    channel = "rmc_multi_node_test"
    first_bus = can.Bus(interface="virtual", channel=channel)
    second_bus = can.Bus(interface="virtual", channel=channel)
    first_fake = FakeDevice(first_bus, False, node_id=1,
                            serial_number=0x1111222233334444).start()
    second_fake = FakeDevice(second_bus, False, node_id=2,
                             serial_number=0xAAAABBBBCCCCDDDD).start()
    manager = CanBusManager()
    try:
        manager.link(interface="virtual", channel=channel, node_id=1, timeout=0.5)
        for node_id, bus in ((1, first_bus), (2, second_bus)):
            heartbeat = cb_id(CB_PRI_CONFIG, 0x48, 0xFE, node_id)
            bus.send(can.Message(arbitration_id=heartbeat, is_extended_id=True,
                                 data=b"\x00" * 8))
        devices, errors = discover_devices(
            {"interface": "virtual", "channel": channel, "node_id": 1,
             "timeout": 0.5},
            include_usb=False, seconds=0.1, can_manager=manager)
        assert errors == []
        assert len(devices) == 2
        first, second = devices
        session = ShellSession(can_manager=manager)
        assert session.register(first)[0] == "m0"
        assert session.register(second)[0] == "m1"
        assert first.link.bus is second.link.bus
        assert first.serial_number_hex == "0x1111222233334444"
        assert second.serial_number_hex == "0xAAAABBBBCCCCDDDD"

        first.write("config.f", 1.0)
        assert first_fake.values[10] == 1.0
        assert second_fake.values[10] == 3.5
    finally:
        manager.close()
        first_fake.stop()
        second_fake.stop()
        first_bus.shutdown()
        second_bus.shutdown()
