"""测试用假设备（照 Core/Src/can_cyberbeast.c 的行为）。

两个层次：
  FakeDevice  在 python-can 的 virtual 总线上跑，测真正的 CanLink 帧格式
  MemoryLink  不进总线，直接给 Device 提供描述符/读写，测代理树与上层逻辑
"""

import json
import struct
import threading

import can

from rmc.errors import EpError
from rmc.protocol import (
    CB_MSG_JSON_DESC_DATA, CB_MSG_JSON_DESC_READ, CB_MSG_PARAM_READ,
    CB_MSG_PARAM_WRITE, CB_PRI_CONFIG, CB_READ_MORE, CB_WRITE_MORE, TYPE_CODES,
    cb_id, cb_id_fields, crc16, decode_value, encode_value,
)

NODE = 1
MASTER = 0xFE

# 模拟设备里标成 persist 的参数（用来测 backup/restore）
PERSIST_IDS = {10, 13}

# endpoint id -> (名字, 类型, 权限, 初值)
PARAMS = {
    10: ("config.f", "float", "rw", 3.5),
    11: ("config.u64", "uint64", "rw", 0x0102030405060708),
    12: ("config.i8", "int8", "rw", -7),
    13: ("config.b", "bool", "rw", True),
    14: ("config.wo", "float", "w", 0.0),
    15: ("telemetry.t", "float", "r", 1.25),
    20: ("action.save", "function", "rw", None),
    30: ("system.serial_number", "uint64", "r", 0x0123456789ABCDEF),
    31: ("system.fw_version_major", "uint8", "r", 1),
    32: ("system.fw_version_minor", "uint8", "r", 2),
    33: ("system.fw_version_revision", "uint8", "r", 3),
    34: ("system.fw_version_unreleased", "uint8", "r", 2),
    35: ("system.hw_version_major", "uint8", "r", 4),
    36: ("system.hw_version_minor", "uint8", "r", 2),
    37: ("system.hw_version_variant", "uint8", "r", 89),
}
ACTION_SAVE = 20


def descriptor_entries(params=PARAMS):
    out = []
    for ep_id, (name, type_name, access, _value) in sorted(params.items()):
        entry = {"name": name, "id": ep_id, "type": type_name, "access": access}
        if type_name == "function":
            entry["inputs"] = []
            entry["outputs"] = []
        if ep_id in PERSIST_IDS:
            entry["persist"] = True
        out.append(entry)
    return out


def build_descriptor(params=PARAMS):
    return json.dumps(descriptor_entries(params), separators=(",", ":")).encode("ascii")


def _clamp(ep, value):
    """模拟固件的 JSON min/max 限幅（测试里只对 float 做）。"""
    if ep["type"] != "float":
        return value
    lo = ep.get("min", -5.0)
    hi = ep.get("max", 5.0)
    return min(max(value, lo), hi)


class FakeDevice:
    """照 can_cyberbeast.c 的行为回答 SDO / 描述符请求。"""

    def __init__(self, bus, fd, params=PARAMS, node_id=NODE, serial_number=None):
        self.bus = bus
        self.fd = fd
        self.node_id = node_id
        self.params = params
        self.values = {ep: value for ep, (_n, _t, _a, value) in params.items()}
        if serial_number is not None and 30 in self.values:
            self.values[30] = serial_number
        self.descriptor = build_descriptor(params)
        self.crc = crc16(self.descriptor)
        self.actions = []
        self.sdo_read_requests = 0
        self.acked_writes = 0
        self._desc_master = None
        self._desc_meta_sent = False
        self._desc_offset = 0
        self._wasm = None
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop = True
        self._thread.join(timeout=2.0)

    def _run(self):
        while not self._stop:
            frame = self.bus.recv(0.002)
            if frame is not None:
                self._handle(frame)
            if self._desc_master is not None:
                for _ in range(2):      # 固件是 2 帧/ms
                    if not self._desc_step():
                        break

    # -- 帧收发 -----------------------------------------------------------

    def _send(self, msgtype, dest, seq, data):
        can_id = cb_id(CB_PRI_CONFIG, msgtype, dest, self.node_id, seq)
        self.bus.send(can.Message(arbitration_id=can_id, is_extended_id=True,
                                  is_fd=self.fd, data=data))

    def _handle(self, frame):
        _pri, msgtype, dest, src, seq = cb_id_fields(frame.arbitration_id)
        if dest != self.node_id or src == 0:
            return
        data = bytes(frame.data)
        if msgtype == CB_MSG_PARAM_READ:
            self._param_read(src, seq, data)
        elif msgtype == CB_MSG_PARAM_WRITE:
            self._param_write(src, seq, data)
        elif msgtype == CB_MSG_JSON_DESC_READ:
            offset = int.from_bytes(data[:4], "little")
            self._desc_master = src
            self._desc_meta_sent = False
            self._desc_offset = offset

    # -- SDO --------------------------------------------------------------

    def _encode(self, ep):
        value = self.values.get(ep)
        type_name = self.params[ep][1]
        if type_name == "function":
            return b""                     # 动作端点：执行，没有返回值
        if self.params[ep][2] == "w":
            return b""                     # 固件：只写端点在 EP_OP_READ 下返回错误
        fmt = TYPE_CODES[type_name][0]
        return struct.pack(fmt, value)

    def _param_read(self, src, seq, data):
        self.sdo_read_requests += 1
        flags = data[0]
        ep = (data[1] << 8) | data[2]
        req_len = data[3]
        offset = int.from_bytes(data[4:8], "big") if len(data) >= 8 else 0
        req_len = 4 if req_len == 0 else min(req_len, 8)
        if not self.fd and req_len > 4:
            req_len = 4                    # Classic：ReqLen 钳到 4

        if ep == ACTION_SAVE:
            self.actions.append("save")

        value = self._encode(ep)
        rsp = bytearray(data[:3])          # flags / ep_hi / ep_lo
        rsp[0] = flags & 0x7F
        rsp.append(0)
        if offset < len(value):
            n = min(req_len, len(value) - offset)
            rsp += value[offset:offset + n]
            rsp[3] = n
            if offset + n < len(value):
                rsp[0] |= CB_READ_MORE
        self._send(CB_MSG_PARAM_READ, src, seq, bytes(rsp))

    def _commit(self, ep, payload):
        type_name = self.params[ep][1]
        if type_name == "function":
            return
        fmt = TYPE_CODES[type_name][0]
        value = struct.unpack(fmt, payload)[0]
        if type_name == "float":
            value = min(max(value, -5.0), 5.0)     # 验证读回看到的是限幅值
        self.values[ep] = value
        self.acked_writes += 1

    def _param_write(self, src, seq, data):
        flags = data[0]
        ep = (data[1] << 8) | data[2]
        total = data[3]

        if not self.fd and total > 4:
            if not 5 <= total <= 8:
                return
            if self._wasm is None or self._wasm[0] != ep:
                self._wasm = [ep, bytearray(total), 0]
            buf, offset = self._wasm[1], self._wasm[2]
            chunk = data[4:4 + min(4, total - offset)]
            buf[offset:offset + len(chunk)] = chunk
            self._wasm[2] = offset + len(chunk)
            if flags & CB_WRITE_MORE:
                return
            complete = self._wasm[2] == total
            payload = bytes(buf)
            self._wasm = None
            if not complete:
                return
            self._commit(ep, payload)
        else:
            if len(data) < 4 + total:
                return
            self._wasm = None
            self._commit(ep, data[4:4 + total])

        # ACK：固件的 seq 恒为 0，也不带成败位
        self._send(CB_MSG_PARAM_WRITE, src, 0,
                   bytes([flags & 0x7F, data[1], data[2], 0, 0, 0, 0, 0]))

    # -- 描述符 -----------------------------------------------------------

    def _desc_step(self):
        frame_len = 64 if self.fd else 8
        data = bytearray(frame_len)
        if not self._desc_meta_sent:
            data[2:6] = len(self.descriptor).to_bytes(4, "little")
            data[6:8] = self.crc.to_bytes(2, "little")
            self._send(CB_MSG_JSON_DESC_DATA, self._desc_master, 0, bytes(data))
            self._desc_meta_sent = True
            return True
        if self._desc_offset >= len(self.descriptor):
            self._desc_master = None
            return False
        data[0:2] = (self._desc_offset & 0xFFFF).to_bytes(2, "little")
        room = frame_len - 2
        chunk = self.descriptor[self._desc_offset:self._desc_offset + room]
        data[2:2 + len(chunk)] = chunk
        self._send(CB_MSG_JSON_DESC_DATA, self._desc_master, 0, bytes(data))
        self._desc_offset += len(chunk)
        return True


class MemoryLink:
    """不经过总线的 link（与 CanLink/FibreLink 同接口），测代理树与上层逻辑。"""

    is_can = False
    PERSIST = {10, 13}      # config.f / config.b（测 backup/restore）

    def __init__(self, params=PARAMS):
        self.params = params
        self.by_id = {}
        for ep, (name, type_name, access, value) in params.items():
            entry = {"name": name, "id": ep, "type": type_name, "access": access}
            if type_name == "function":
                entry["inputs"] = []
                entry["outputs"] = []
            if type_name == "float":
                entry.update(min=-5.0, max=5.0)
            if ep in self.PERSIST:
                entry["persist"] = True
            self.by_id[ep] = entry
        self.raw = json.dumps(list(self.by_id.values()),
                              separators=(",", ":")).encode("ascii")
        self.crc = crc16(self.raw)
        self.values = {ep: value for ep, (_n, _t, _a, value) in params.items()}
        self.calls = []
        self.writes = []

    def describe(self):
        return "memory"

    def fetch_descriptor(self):
        return self.raw

    def read_value(self, ep_id, size):
        ep = self.by_id[ep_id]
        if ep["type"] in ("void", "function") or ep["access"] == "w":
            raise EpError("endpoint %d 没有返回数据" % ep_id)
        return encode_value(ep, self.values[ep_id])

    def write_value(self, ep_id, data, read_back=False):
        ep = self.by_id[ep_id]
        value = decode_value(ep, data)
        if ep["type"] in ("float", "double"):
            value = _clamp(ep, value)
        self.values[ep_id] = value
        self.writes.append((ep_id, value))
        return encode_value(ep, value) if read_back else b""

    def call_action(self, ep_id, via_read=False):
        self.calls.append((self.by_id[ep_id]["name"], via_read))

    def close(self):
        pass
