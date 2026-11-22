"""Shell startup discovery for CAN nodes, CDC ports, and USB vendor interfaces."""

from .device import Device
from .protocol import (
    DEFAULT_CAN_BITRATE, DEFAULT_CAN_DATA_BITRATE, DEFAULT_CAN_INTERFACE,
    DEFAULT_CAN_MASTER_ID, DEFAULT_CAN_NODE_ID, DEFAULT_EP_IN, DEFAULT_EP_OUT,
    DEFAULT_PID, DEFAULT_USB_INTERFACE, DEFAULT_VID,
)
from .transport.can import CanBusManager
from .transport.fibre import FibreLink, SerialTransport, UsbTransport


def _open_device(link):
    device = Device(link=link)
    try:
        device.fetch_descriptor()
    except Exception:
        device.close()
        raise
    return device


def _can_options(kwargs):
    return {
        "interface": kwargs.get("interface") or DEFAULT_CAN_INTERFACE,
        "channel": kwargs.get("channel"),
        "bitrate": kwargs.get("bitrate", DEFAULT_CAN_BITRATE),
        "fd": kwargs.get("fd", False),
        "data_bitrate": kwargs.get("data_bitrate", DEFAULT_CAN_DATA_BITRATE),
        "master_id": kwargs.get("master_id", DEFAULT_CAN_MASTER_ID),
        "timeout": kwargs.get("timeout", 1.0),
        "tty_baudrate": kwargs.get("tty_baudrate", 115200),
    }


def _discover_can(kwargs, manager, seconds, devices, errors):
    options = _can_options(kwargs)
    try:
        listener = manager.link(node_id=kwargs.get("node_id", DEFAULT_CAN_NODE_ID),
                                **options)
        counts = listener.sniff(seconds)
    except Exception as error:
        errors.append("CAN: %s" % error)
        return

    node_messages = {0x25, *range(0x40, 0x4A), 0xC1}
    nodes = {source for (source, msgtype) in counts
             if 1 <= source <= 127 and msgtype in node_messages}
    nodes.add(kwargs.get("node_id", DEFAULT_CAN_NODE_ID))
    for node_id in sorted(nodes):
        try:
            link = manager.link(node_id=node_id, **options)
            devices.append(_open_device(link))
        except Exception as error:
            errors.append("CAN node %d: %s" % (node_id, error))


def _cdc_ports(port=None, vid=DEFAULT_VID, pid=DEFAULT_PID):
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    if port:
        return [port]
    return sorted(
        entry.device for entry in list_ports.comports()
        if entry.vid == vid and entry.pid == pid
    )


def _usb_devices(vid=DEFAULT_VID, pid=DEFAULT_PID):
    from .transport import fibre

    if fibre.usb is None:
        return []
    return fibre.usb.core.find(
        find_all=True, idVendor=vid, idProduct=pid,
        backend=fibre._usb_backend())


def _discover_usb(kwargs, devices, errors, port=None, cdc=True, vendor=False):
    if cdc:
        for device_port in _cdc_ports(
                port, kwargs.get("vid", DEFAULT_VID), kwargs.get("pid", DEFAULT_PID)):
            try:
                io = SerialTransport(device_port, kwargs.get("baud", 115200),
                                     kwargs.get("timeout", 1.0))
                devices.append(_open_device(FibreLink(io)))
            except Exception as error:
                errors.append("CDC %s: %s" % (device_port, error))

    if not vendor:
        return
    try:
        usb_devices = _usb_devices(kwargs.get("vid", DEFAULT_VID),
                                   kwargs.get("pid", DEFAULT_PID))
        if usb_devices is None:
            return
        for usb_device in usb_devices:
            try:
                io = UsbTransport(
                    vid=kwargs.get("vid", DEFAULT_VID),
                    pid=kwargs.get("pid", DEFAULT_PID),
                    interface=kwargs.get("usb_interface", DEFAULT_USB_INTERFACE),
                    ep_out=kwargs.get("ep_out", DEFAULT_EP_OUT),
                    ep_in=kwargs.get("ep_in", DEFAULT_EP_IN),
                    timeout=kwargs.get("timeout", 1.0), device=usb_device)
                devices.append(_open_device(FibreLink(io)))
            except Exception as error:
                errors.append("USB vendor: %s" % error)
    except Exception as error:
        errors.append("USB vendor discovery: %s" % error)


def discover_devices(kwargs=None, include_can=True, include_usb=True, port=None,
                     cdc=True, vendor=True, seconds=1.0, can_manager=None):
    """Return ``(devices, errors)``; CAN discovery is passive plus the configured node."""
    kwargs = dict(kwargs or {})
    devices = []
    errors = []
    if include_can:
        _discover_can(kwargs, can_manager or CanBusManager(), seconds, devices, errors)
    if include_usb:
        _discover_usb(kwargs, devices, errors, port=port, cdc=cdc, vendor=vendor)
    return devices, errors


__all__ = ["discover_devices"]
