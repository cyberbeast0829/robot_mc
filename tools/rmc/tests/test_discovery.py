"""Device enumeration across shell startup transports."""

from rmc import discovery


class _Transport:
    def __init__(self, description):
        self.description = description

    def describe(self):
        return self.description


def test_usb_discovery_enumerates_all_cdc_and_vendor_devices(monkeypatch):
    seen_ports = []
    monkeypatch.setattr(
        discovery, "_cdc_ports",
        lambda port=None, vid=None, pid=None: ["COM4", "COM7"])
    monkeypatch.setattr(
        discovery, "SerialTransport",
        lambda port, baud, timeout: _Transport("CDC " + port))
    monkeypatch.setattr(discovery, "_usb_devices", lambda vid, pid: ["usb-a", "usb-b"])
    monkeypatch.setattr(
        discovery, "UsbTransport",
        lambda **kwargs: _Transport("USB " + kwargs["device"]))
    monkeypatch.setattr(discovery, "FibreLink", lambda io: io)
    monkeypatch.setattr(
        discovery, "_open_device",
        lambda link: seen_ports.append(link.describe()) or link.describe())

    devices, errors = discovery.discover_devices(
        {"timeout": 1.0}, include_can=False, include_usb=True, vendor=True)

    assert devices == ["CDC COM4", "CDC COM7", "USB usb-a", "USB usb-b"]
    assert seen_ports == devices
    assert errors == []


def test_explicit_cdc_port_skips_vendor_discovery(monkeypatch):
    opened = []
    monkeypatch.setattr(
        discovery, "_cdc_ports",
        lambda port=None, vid=None, pid=None: [port])
    monkeypatch.setattr(
        discovery, "SerialTransport",
        lambda port, baud, timeout: _Transport("CDC " + port))
    monkeypatch.setattr(
        discovery, "UsbTransport",
        lambda **kwargs: opened.append("vendor"))
    monkeypatch.setattr(discovery, "FibreLink", lambda io: io)
    monkeypatch.setattr(discovery, "_open_device", lambda link: link.describe())

    devices, errors = discovery.discover_devices(
        {"timeout": 1.0}, include_can=False, include_usb=True, port="COM8",
        cdc=True, vendor=False)

    assert devices == ["CDC COM8"]
    assert opened == []
    assert errors == []


def test_usb_discovery_skips_already_connected_cdc_port(monkeypatch):
    seen_ports = []
    monkeypatch.setattr(
        discovery, "_cdc_ports",
        lambda port=None, vid=None, pid=None: ["COM4", "COM7"])
    monkeypatch.setattr(
        discovery, "SerialTransport",
        lambda port, baud, timeout: _Transport("CDC " + port))
    monkeypatch.setattr(discovery, "_usb_devices", lambda vid, pid: [])
    monkeypatch.setattr(discovery, "FibreLink", lambda io: io)
    monkeypatch.setattr(
        discovery, "_open_device",
        lambda link: seen_ports.append(link.describe()) or link.describe())
    devices, errors = discovery.discover_devices(
        {"timeout": 1.0}, include_can=False, include_usb=True,
        exclude_transports={"CDC COM4"})

    assert devices == ["CDC COM7"]
    assert seen_ports == devices
    assert errors == []


def test_can_discovery_skips_already_connected_nodes(monkeypatch):
    class Link:
        def __init__(self, node_id):
            self.node_id = node_id

        def sniff(self, _seconds):
            return {(1, 0x48): 1, (2, 0x48): 1}

    class Manager:
        def __init__(self):
            self.nodes = []

        def link(self, node_id, **_kwargs):
            self.nodes.append(node_id)
            return Link(node_id)

    manager = Manager()
    opened = []
    monkeypatch.setattr(
        discovery, "_open_device",
        lambda link: opened.append(link.node_id) or link.node_id)

    devices, errors = discovery.discover_devices(
        {"node_id": 1}, include_usb=False, seconds=0.1,
        can_manager=manager, exclude_nodes={1}, include_configured_node=False)

    assert devices == [2]
    assert opened == [2]
    assert errors == []


def test_usb_discovery_skips_already_connected_vendor_interface(monkeypatch):
    class UsbDevice:
        bus = 1
        address = 2

    opened = []
    monkeypatch.setattr(discovery, "_cdc_ports", lambda *args: [])
    monkeypatch.setattr(discovery, "_usb_devices", lambda vid, pid: [UsbDevice()])
    monkeypatch.setattr(
        discovery, "UsbTransport",
        lambda **_kwargs: opened.append("opened"))

    devices, errors = discovery.discover_devices(
        {"usb_interface": 2}, include_can=False, vendor=True,
        exclude_transports={"USB 1209:4342 interface 2 bus 1 address 2"})

    assert devices == []
    assert opened == []
    assert errors == []
