"""开发/台架板的 USB 传输：CDC 的 0xAA 帧与 vendor interface 的 bulk 包。

出货板没有 USB 座，客户只走 CAN；这里保留是为了让 rmc 完全替代
tools/mc_ep.py（`--port` / `--usb`）。
"""

import struct
import threading

from ..errors import EpError
from ..protocol import (
    DEFAULT_EP_IN, DEFAULT_EP_OUT, DEFAULT_PID, DEFAULT_USB_INTERFACE, DEFAULT_VID,
    FRAME_PREFIX, PROTOCOL_VERSION, REPLY_FLAG, STATUS_NAMES, STATUS_VERSION_MISMATCH,
    USB_READ_SIZE, WRITE_FLAG, build_frame, crc8, crc16,
)

try:
    import serial
except ImportError:      # pragma: no cover
    serial = None

try:
    import usb.core
    import usb.util
    try:
        import usb.backend.libusb1
    except ImportError:  # pragma: no cover
        pass
except ImportError:      # pragma: no cover
    usb = None


def _usb_backend():
    """优先用 libusb-package 自带的 DLL（Windows 上不用手工配 PATH）。"""
    try:
        import libusb_package
        import usb.backend.libusb1
        return usb.backend.libusb1.get_backend(find_library=libusb_package.find_library)
    except Exception:
        return None


def _usb_description(vid, pid, interface, dev):
    location = ""
    if getattr(dev, "bus", None) is not None or getattr(dev, "address", None) is not None:
        location = " bus %s address %s" % (getattr(dev, "bus", "?"),
                                           getattr(dev, "address", "?"))
    return "USB %04x:%04x interface %d%s" % (vid, pid, interface, location)


class SerialTransport:
    """CDC 上的 0xAA/CRC 帧（和 ASCII 菜单共用数据流）。"""

    def __init__(self, port, baud=115200, timeout=1.0):
        if serial is None:
            raise EpError("需要 pyserial：pip install pyserial")
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.description = "CDC %s" % port

    def close(self):
        self.ser.close()

    def reset_input(self):
        self.ser.reset_input_buffer()

    def write(self, payload):
        self.ser.write(build_frame(payload))

    def read_packet(self):
        while True:
            b = self.ser.read(1)
            if not b:
                raise EpError("等待响应超时")
            if b[0] != FRAME_PREFIX:
                continue
            length = self.ser.read(1)
            if length == b"":
                raise EpError("等待响应超时")
            length = length[0]
            if length == 0 or length > 64:
                continue
            header = bytes([FRAME_PREFIX, length])
            crc = self.ser.read(1)
            if crc == b"":
                raise EpError("等待响应超时")
            if crc[0] != crc8(header):
                continue
            payload = self.ser.read(length)
            if len(payload) != length:
                raise EpError("响应 payload 不完整")
            tail = self.ser.read(2)
            if len(tail) != 2:
                raise EpError("响应 CRC 不完整")
            if crc16(payload) != ((tail[0] << 8) | tail[1]):
                continue  # 坏帧，重新同步
            return payload


class UsbTransport:
    """vendor interface（interface 2）的 bulk 端点，一个包就是一帧。

    响应长度上限 63 字节（固件保证短包），所以一次 64 字节 bulk 读就能收全。
    Windows 需要先用 Zadig 把 interface 2 绑到 WinUSB；CDC 那部分仍是 usbser。
    """

    def __init__(self, vid=DEFAULT_VID, pid=DEFAULT_PID, interface=DEFAULT_USB_INTERFACE,
                 ep_out=DEFAULT_EP_OUT, ep_in=DEFAULT_EP_IN, timeout=1.0, device=None):
        if usb is None:
            raise EpError("需要 pyusb：pip install pyusb libusb-package")
        if device is None:
            try:
                dev = usb.core.find(idVendor=vid, idProduct=pid, backend=_usb_backend())
            except (usb.core.USBError, NotImplementedError) as e:
                raise EpError("pyusb 没有可用的 libusb 后端（Windows: pip install libusb-package）：%s" % e)
        else:
            dev = device
        if dev is None:
            raise EpError("找不到 USB 设备 %04x:%04x（检查 USB 连接、VID/PID 和 WinUSB 驱动）"
                          % (vid, pid))
        self.dev = dev
        self.interface = interface
        self.ep_out = ep_out
        self.ep_in = ep_in
        self.timeout = timeout
        self.description = _usb_description(vid, pid, interface, dev)
        try:
            dev.get_active_configuration()
        except (usb.core.USBError, NotImplementedError):
            # 设备还没被配置（Windows 的 usbccgp 通常已经配好，不要重设，
            # 免得打断 CDC 那一路）
            try:
                dev.set_configuration()
            except (usb.core.USBError, NotImplementedError) as e:
                raise EpError(
                    "无法打开 USB 设备：%s\n"
                    "  - Windows: 用 Zadig 只给 interface %d 装 WinUSB（CDC 保持 usbser）；\n"
                    "  - 旧固件只有 CDC，没有 vendor interface。" % (e, interface))
        # Linux/macOS 上内核驱动需要先 detach；Windows 上由 WinUSB 绑定，忽略异常
        try:
            if dev.is_kernel_driver_active(interface):
                dev.detach_kernel_driver(interface)
        except (NotImplementedError, usb.core.USBError):
            pass
        try:
            usb.util.claim_interface(dev, interface)
        except (usb.core.USBError, NotImplementedError) as e:
            raise EpError(
                "无法占用 interface %d：%s\n"
                "  - 确认固件支持复合设备（旧固件只有 CDC，没有 interface 2）；\n"
                "  - Windows 上用 Zadig 只给 interface 2 装 WinUSB。" % (interface, e))

    def close(self):
        try:
            usb.util.release_interface(self.dev, self.interface)
        except Exception:
            pass
        try:
            usb.util.dispose_resources(self.dev)
        except Exception:
            pass

    def reset_input(self):
        # 只保留请求/应答语义：不主动读，避免把上一条迟到的响应当成这次的
        pass

    def write(self, payload):
        try:
            self.dev.write(self.ep_out, payload, int(self.timeout * 1000))
        except usb.core.USBError as e:
            raise EpError("USB 写失败：%s" % e)

    def read_packet(self):
        try:
            return bytes(self.dev.read(self.ep_in, USB_READ_SIZE, int(self.timeout * 1000)))
        except usb.core.USBError as e:
            if "timeout" in str(e).lower():
                raise EpError("等待响应超时")
            raise EpError("USB 读失败：%s" % e)


class FibreLink:
    """fibre 风格 payload 层（CDC 的 0xAA 帧 / vendor bulk 包）。

    请求 PAYLOAD: seq(2) | id|flags(2) | rxlen(2) | tx_data | 描述符 CRC16(2)
    响应 PAYLOAD: seq|0x8000(2) | status(1) | rx_data
    """

    is_can = False

    def __init__(self, io):
        self.io = io
        self.seq = 0
        self.crc = 0        # 描述符 CRC16，同时是接口版本（请求里的 trailer）
        self._lock = threading.Lock()

    def close(self):
        with self._lock:
            self.io.close()

    def describe(self):
        return getattr(self.io, "description", "CDC/USB")

    def _transact(self, endpoint_id, tx_data, rx_len, trailer, write=False):
        with self._lock:
            return self._transact_locked(endpoint_id, tx_data, rx_len, trailer, write)

    def _transact_locked(self, endpoint_id, tx_data, rx_len, trailer, write):
        self.seq = (self.seq + 1) & 0x7FFF
        flags = REPLY_FLAG | (WRITE_FLAG if write else 0)
        payload = struct.pack("<HHH", self.seq, endpoint_id | flags, rx_len)
        payload += tx_data
        payload += struct.pack("<H", trailer)

        self.io.reset_input()
        self.io.write(payload)

        # 上一次超时可能留下一个迟到的响应：跳过 seq 不符的包（最多 3 个）
        resp = b""
        for _ in range(3):
            resp = self.io.read_packet()
            if len(resp) < 3:
                raise EpError("响应太短")
            rseq = struct.unpack("<H", resp[:2])[0]
            if (rseq & 0x7FFF) == self.seq:
                break
        else:
            raise EpError("响应 seq 不匹配（期望 %d）" % self.seq)
        status = resp[2]
        if status != 0:
            raise EpError("status %d: %s" % (status, STATUS_NAMES.get(status, "未知错误")),
                          status=status)
        return resp[3:]

    # -- 三种传输共用的四个动作 -------------------------------------------

    def fetch_descriptor(self):
        info = self._transact(0, struct.pack("<I", 0xFFFFFFFF), 4, PROTOCOL_VERSION)
        if len(info) != 4:
            raise EpError("设备没有返回描述符版本信息")
        json_crc, total_len = struct.unpack("<HH", info)

        raw = bytearray()
        while len(raw) < total_len:
            want = min(60, total_len - len(raw))
            chunk = self._transact(0, struct.pack("<I", len(raw)), want, PROTOCOL_VERSION)
            if not chunk:
                break
            raw += chunk

        if len(raw) != total_len:
            raise EpError("描述符下载不完整（%d/%d 字节）" % (len(raw), total_len))
        self.crc = json_crc
        return bytes(raw)

    def read_value(self, ep_id, size):
        return self._transact(ep_id, b"", size, self.crc)

    def write_value(self, ep_id, data, read_back=False):
        # 固件的写应答直接回显限幅后的值，不需要（也不该）再读一次
        if read_back:
            return self._transact(ep_id, data, len(data), self.crc, write=True)
        self._transact(ep_id, data, 0, self.crc, write=True)
        return b""

    def call_action(self, ep_id, via_read=False):
        # ODrive 的 endpoint 协议没有读写位，function 端点被“读”也会执行；
        # 固件两种意图都接受，via_read 用来验证这条兼容路径。
        self._transact(ep_id, b"", 0, self.crc, write=not via_read)


__all__ = ["FibreLink", "SerialTransport", "UsbTransport", "STATUS_VERSION_MISMATCH"]
