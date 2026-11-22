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
