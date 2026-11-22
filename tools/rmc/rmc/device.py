"""设备对象：描述符下载、参数读写、动作调用、备份/恢复。

构造参数与 tools/mc_ep.py 的 Device 完全兼容（其它台架脚本还依赖它），
更友好的入口是 rmc.connect() / `rmc` 命令行。
"""

import json
import time

from . import protocol as _p
from .errors import EpError
from .protocol import (
    DEFAULT_EP_IN, DEFAULT_EP_OUT, DEFAULT_PID, DEFAULT_USB_INTERFACE, DEFAULT_VID,
    STATUS_VERSION_MISMATCH, decode_value, encode_value, is_action, is_readable,
    is_writable, type_size,
)
from .transport.can import CanLink
from .transport.fibre import FibreLink, SerialTransport, UsbTransport


def _same(a, b):
    """备份/恢复时的“值是否相同”（float 用相对容差，避免 3.5000001 反复写）。"""
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    if isinstance(a, float) or isinstance(b, float):
        try:
            a = float(a)
            b = float(b)
        except (TypeError, ValueError):
            return False
        return abs(a - b) <= max(1e-9, 1e-6 * abs(a))
    return a == b


class Device:
    """一台设备（一个 CAN 节点 / 一条 USB 连接）。"""

    def __init__(self, port=None, baud=115200, timeout=1.0, use_usb=False,
                 vid=DEFAULT_VID, pid=DEFAULT_PID, interface=DEFAULT_USB_INTERFACE,
                 ep_out=DEFAULT_EP_OUT, ep_in=DEFAULT_EP_IN, io=None, link=None,
                 can_interface=None, can_channel=None,
                 can_bitrate=_p.DEFAULT_CAN_BITRATE, can_fd=False,
                 can_data_bitrate=_p.DEFAULT_CAN_DATA_BITRATE,
                 can_id=_p.DEFAULT_CAN_NODE_ID,
                 can_master_id=_p.DEFAULT_CAN_MASTER_ID,
                 can_tty_baudrate=115200):
        if link is not None:
            self.link = link
        elif io is not None:
            self.link = FibreLink(io)
        elif can_interface:
            self.link = CanLink(interface=can_interface, channel=can_channel,
                                bitrate=can_bitrate, fd=can_fd,
                                data_bitrate=can_data_bitrate, node_id=can_id,
                                master_id=can_master_id, timeout=timeout,
                                tty_baudrate=can_tty_baudrate)
        elif use_usb:
            self.link = FibreLink(UsbTransport(vid, pid, interface, ep_out, ep_in, timeout))
        else:
            if not port:
                raise EpError("需要 --port（CDC 串口）、--usb（vendor interface）"
                              "或 --can（CYBERBEAST）")
            self.link = FibreLink(SerialTransport(port, baud, timeout))
        self.descriptor = []
        self.by_name = {}
        self.by_id = {}
        self.json_crc = 0
        self._root = None
        self._serial_number_loaded = False
        self.serial_number = None

    # -- 生命周期 ---------------------------------------------------------

    def close(self):
        self.link.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def describe(self):
        return self.link.describe()

    # -- 描述符与类型 -----------------------------------------------------

    def fetch_descriptor(self):
        raw = self.link.fetch_descriptor()
        if _p.crc16(raw) != self.link.crc:
            raise EpError("描述符 CRC 校验失败")
        self.json_crc = self.link.crc
        self.descriptor = json.loads(raw.decode("ascii"))
        self.by_name = {ep["name"]: ep for ep in self.descriptor}
        self.by_id = {ep["id"]: ep for ep in self.descriptor}
        self._root = None
        return self.descriptor

    @property
    def endpoints(self):
        return self.descriptor

    @property
    def descriptor_crc(self):
        return self.json_crc

    def _entry(self, name):
        if not self.by_name:
            self.fetch_descriptor()
        if not isinstance(name, str):
            name = getattr(name, "path", name)
        if name not in self.by_name:
            raise EpError("没有这个 endpoint: %s" % name)
        return self.by_name[name]

    @property
    def root(self):
        """属性树代理（`dev.config.i_max` / tab 补全用）。"""
        if self._root is None:
            from .proxy import build_tree
            if not self.descriptor:
                self.fetch_descriptor()
            self._root = build_tree(self)
        return self._root

    def __getattr__(self, name):
        # 只处理非下划线名字，避免干扰 pickle/copy 等
        if name.startswith("_"):
            raise AttributeError(name)
        return getattr(self.root, name)

    def __getitem__(self, name):
        return getattr(self.root, name)

    def __dir__(self):
        names = set(super().__dir__())
        if self.descriptor:
            names.update(ep["name"].split(".")[0] for ep in self.descriptor)
        return sorted(names)

    def read_serial_number(self):
        """读取并缓存 MCU UID 低 64 位；旧固件或读失败时返回 None。"""
        if self._serial_number_loaded:
            return self.serial_number
        self._serial_number_loaded = True
        if "system.serial_number" not in self.by_name:
            return None
        try:
            value = self.read("system.serial_number")
            value = int(value)
            if 0 <= value <= 0xFFFFFFFFFFFFFFFF:
                self.serial_number = value
        except (EpError, TypeError, ValueError, OverflowError):
            pass
        return self.serial_number

    @property
    def serial_number_hex(self):
        value = self.read_serial_number()
        return "0x%016X" % value if value is not None else None

    def identity_key(self):
        """以固件 UID 去重；读不到时退回到可重连的传输定位符。"""
        serial_number = self.read_serial_number()
        if serial_number is not None:
            return ("serial", serial_number)
        if getattr(self.link, "is_can", False):
            location = "%s node=%d" % (self.describe(), self.link.node_id)
        else:
            location = self.describe()
        return ("path", location)

    # -- 公共 API ---------------------------------------------------------

    def _with_version_retry(self, fn):
        """CDC/USB 刷了“描述符有变化”的镜像后会返回 EP_ERR_VERSION：
        自动重下描述符再试一次。"""
        try:
            return fn()
        except EpError as e:
            if getattr(e, "status", None) != STATUS_VERSION_MISMATCH:
                raise
            self.fetch_descriptor()
            return fn()

    def read(self, name):
        ep = self._entry(name)
        if is_action(ep):
            raise EpError("%s 是动作端点，请用 call" % ep["name"])
        if not is_readable(ep):
            raise EpError("%s 是只写端点" % ep["name"])

        def do():
            return decode_value(ep, self.link.read_value(ep["id"], type_size(ep["type"])))

        return self._with_version_retry(do)

    def write(self, name, value, verify=None):
        """写参数；可读端点默认写后读回（CAN 的 ACK 不带成败位）。

        verify=False 时只发写请求；返回读回值，或 None。
        """
        ep = self._entry(name)
        if is_action(ep):
            raise EpError("%s 是动作端点，请用 call" % ep["name"])
        if not is_writable(ep):
            raise EpError("%s 是只读端点" % ep["name"])
        payload = encode_value(ep, value)
        read_back = is_readable(ep) if verify is None else bool(verify)

        def do():
            return self.link.write_value(ep["id"], payload, read_back=read_back)

        data = self._with_version_retry(do)
        return decode_value(ep, data) if data else None

    def call(self, name, via_read=False):
        ep = self._entry(name)
        # "void" 是我们 params.json 的内部写法；线上描述符是 ODrive 的 "function"
        if not is_action(ep):
            raise EpError("%s 不是动作端点" % ep["name"])

        def do():
            self.link.call_action(ep["id"], via_read=via_read)

        self._with_version_retry(do)

    # -- 信息 / 备份 / 恢复 ------------------------------------------------

    def info(self):
        info = {
            "transport": self.link.describe(),
            "descriptor_crc": "0x%04X" % self.json_crc,
            "endpoints": len(self.descriptor),
        }
        serial_number = self.read_serial_number()
        info["serial_number"] = (
            "0x%016X" % serial_number if serial_number is not None else "unknown")
        for key in ("major", "minor", "revision", "unreleased"):
            name = "system.fw_version_" + key
            if name in self.by_name:
                try:
                    info["fw_version_" + key] = self.read(name)
                except EpError:
                    pass
        for name in ("system.config_schema", "system.prefs_loaded", "system.prefs_sequence"):
            if name in self.by_name:
                try:
                    info[name.split(".", 1)[1]] = self.read(name)
                except EpError:
                    pass
        return info

    def persist_endpoints(self):
        if not self.descriptor:
            self.fetch_descriptor()
        return [ep for ep in self.descriptor
                if ep.get("persist") and is_readable(ep)]

    def backup(self):
        values = {}
        errors = {}
        for ep in self.persist_endpoints():
            try:
                values[ep["name"]] = self.read(ep["name"])
            except EpError as e:
                errors[ep["name"]] = str(e)
        data = {
            "format": "rmc-backup",
            "version": 1,
            "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "transport": self.link.describe(),
            "descriptor_crc": "0x%04X" % self.json_crc,
            "values": values,
        }
        if errors:
            data["errors"] = errors
        return data

    def restore(self, data, dry_run=False):
        """把备份里的值写回设备，返回逐条结果。

        data 可以是 backup() 的结果，也可以是 {name: value} 字典。
        """
        values = data.get("values", data) if isinstance(data, dict) else {}
        if not self.descriptor:
            self.fetch_descriptor()
        results = []
        for name in sorted(values):
            value = values[name]
            record = {"name": name, "before": None, "after": None,
                      "status": "", "error": ""}
            ep = self.by_name.get(name)
            if ep is None:
                record["status"] = "跳过"
                record["error"] = "本固件没有这个 endpoint"
                results.append(record)
                continue
            if not is_writable(ep):
                record["status"] = "跳过"
                record["error"] = "只读端点"
                results.append(record)
                continue
            try:
                record["before"] = self.read(name) if is_readable(ep) else None
            except EpError:
                record["before"] = None
            if record["before"] is not None and _same(record["before"], value):
                record["status"] = "相同"
                record["after"] = record["before"]
                results.append(record)
                continue
            if dry_run:
                record["status"] = "将写入"
                record["after"] = value
                results.append(record)
                continue
            try:
                record["after"] = self.write(name, value)
                record["status"] = "已写入" if record["after"] is None or \
                    _same(record["after"], value) else "被限幅/拒绝"
            except EpError as e:
                record["status"] = "失败"
                record["error"] = str(e)
            results.append(record)
        return results

    def save(self):
        """`action.save`（把 RAM 参数写进 flash）。返回是否真的调用了。"""
        if "action.save" not in self.by_name:
            return False
        self.call("action.save")
        return True


def connect(port=None, usb=False, interface=None, channel=None,
            bitrate=_p.DEFAULT_CAN_BITRATE, fd=False,
            data_bitrate=_p.DEFAULT_CAN_DATA_BITRATE,
            node_id=_p.DEFAULT_CAN_NODE_ID, master_id=_p.DEFAULT_CAN_MASTER_ID,
            timeout=1.0, tty_baudrate=115200, baud=115200,
            vid=DEFAULT_VID, pid=DEFAULT_PID, usb_interface=DEFAULT_USB_INTERFACE,
            ep_out=DEFAULT_EP_OUT, ep_in=DEFAULT_EP_IN,
            fetch=True):
    """连接设备（库入口）。

    默认走 CAN（CYBERBEAST）：connect(channel="COM3") / connect(interface="pcan") /
    connect(interface="socketcan", channel="can0")。
    开发板可 connect(port="COM7") 或 connect(usb=True)。
    """
    if port:
        dev = Device(port=port, baud=baud, timeout=timeout)
    elif usb:
        dev = Device(use_usb=True, vid=vid, pid=pid, interface=usb_interface,
                     ep_out=ep_out, ep_in=ep_in, timeout=timeout)
    else:
        dev = Device(can_interface=interface or _p.DEFAULT_CAN_INTERFACE,
                     can_channel=channel, can_bitrate=bitrate, can_fd=fd,
                     can_data_bitrate=data_bitrate, can_id=node_id,
                     can_master_id=master_id, timeout=timeout,
                     can_tty_baudrate=tty_baudrate)
    if fetch:
        try:
            dev.fetch_descriptor()
        except Exception:
            dev.close()
            raise
    return dev
