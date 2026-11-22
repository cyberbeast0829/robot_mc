#!/usr/bin/env python3
"""rmc 的 CAN 传输回归测试（不需要硬件、不需要 CAN 适配器）。

`mc_ep.py` 现在是转发到 rmc 的 shim，所以这里测的就是 `rmc` 的实现
（`make hosttest-rmc` 另跑 tools/rmc/tests 里的包级测试）。

用 python-can 的 virtual 总线在同一进程里放一台"假设备"，行为照
Core/Src/can_cyberbeast.c 写（PARAM_READ / PARAM_WRITE / 描述符流），
再让 mc_ep.CanLink 去访问它。覆盖：

  - 描述符流：元数据帧 + 分片重装 + CRC16
  - 1/2/4/8 字节读：Classic 每帧最多 4 字节，uint64 要按 offset 分两段读
  - 单帧写；Classic 下 >4 字节走两帧分段写；ACK 不带状态 ⇒ 必须读回
  - 动作端点用 PARAM_READ 触发（ODrive/JointSDK 语义）
  - 读一个只写端点（应答 n=0）时必须明确报错，而不是当成长度 0 的值
  - CAN FD：64 字节描述符分片、8 字节一次读完（Classic 要两次）

跑法:
    python tools/test_mc_ep_can.py
"""

import json
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import can  # noqa: E402

import mc_ep  # noqa: E402

NODE = 1
MASTER = 0xFE

# endpoint id -> (名字, 类型, 权限, 初值)
PARAMS = {
    10: ("config.f", "float", "rw", 3.5),
    11: ("config.u64", "uint64", "rw", 0x0102030405060708),
    12: ("config.i8", "int8", "rw", -7),
    13: ("config.b", "bool", "rw", 1),
    14: ("config.wo", "float", "w", 0.0),
    15: ("telemetry.t", "float", "r", 1.25),
    20: ("action.save", "function", "rw", None),
}
ACTION_SAVE = 20


def build_descriptor():
    out = []
    for ep_id, (name, type_name, access, _value) in sorted(PARAMS.items()):
        entry = {"name": name, "id": ep_id, "type": type_name, "access": access}
        if type_name == "function":
            entry["inputs"] = []
            entry["outputs"] = []
        out.append(entry)
    return json.dumps(out, separators=(",", ":")).encode("ascii")


class FakeDevice:
    """照 can_cyberbeast.c 的行为回答 SDO / 描述符请求。"""

    def __init__(self, bus, fd):
        self.bus = bus
        self.fd = fd
        self.values = {ep: value for ep, (_n, _t, _a, value) in PARAMS.items()}
        self.descriptor = build_descriptor()
        self.crc = mc_ep.crc16(self.descriptor)
        self.actions = []
        self.sdo_read_requests = 0
        self.acked_writes = 0
        self._desc_master = None
        self._desc_meta_sent = False
        self._desc_offset = 0
        self._wasm = None
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    # -- 生命周期 ---------------------------------------------------------

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
        can_id = mc_ep.cb_id(mc_ep.CB_PRI_CONFIG, msgtype, dest, NODE, seq)
        self.bus.send(can.Message(arbitration_id=can_id, is_extended_id=True,
                                  is_fd=self.fd, data=data))

    def _handle(self, frame):
        _pri, msgtype, dest, src, seq = mc_ep.cb_id_fields(frame.arbitration_id)
        if dest != NODE or src == 0:
            return
        data = bytes(frame.data)
        if msgtype == mc_ep.CB_MSG_PARAM_READ:
            self._param_read(src, seq, data)
        elif msgtype == mc_ep.CB_MSG_PARAM_WRITE:
            self._param_write(src, seq, data)
        elif msgtype == mc_ep.CB_MSG_JSON_DESC_READ:
            offset = int.from_bytes(data[:4], "little")
            self._desc_master = src
            self._desc_meta_sent = False
            self._desc_offset = offset

    # -- SDO --------------------------------------------------------------

    def _encode(self, ep):
        value = self.values.get(ep)
        type_name = PARAMS[ep][1]
        if type_name == "function":
            return b""                     # 动作端点：执行，没有返回值
        if PARAMS[ep][2] == "w":
            return b""                     # 固件：只写端点在 EP_OP_READ 下返回错误
        fmt = mc_ep.TYPE_CODES[type_name][0]
        return mc_ep.struct.pack(fmt, value)

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
                rsp[0] |= mc_ep.CB_READ_MORE
        self._send(mc_ep.CB_MSG_PARAM_READ, src, seq, bytes(rsp))

    def _commit(self, ep, payload):
        type_name = PARAMS[ep][1]
        if type_name == "function":
            return
        fmt = mc_ep.TYPE_CODES[type_name][0]
        value = mc_ep.struct.unpack(fmt, payload)[0]
        if type_name == "float":
            value = min(max(value, -5.0), 5.0)     # 限幅：验证读回看到的是限幅值
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
            if flags & mc_ep.CB_WRITE_MORE:
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
        self._send(mc_ep.CB_MSG_PARAM_WRITE, src, 0,
                   bytes([flags & 0x7F, data[1], data[2], 0, 0, 0, 0, 0]))

    # -- 描述符 -----------------------------------------------------------

    def _desc_step(self):
        frame_len = 64 if self.fd else 8
        data = bytearray(frame_len)
        if not self._desc_meta_sent:
            data[2:6] = len(self.descriptor).to_bytes(4, "little")
            data[6:8] = self.crc.to_bytes(2, "little")
            self._send(mc_ep.CB_MSG_JSON_DESC_DATA, self._desc_master, 0, bytes(data))
            self._desc_meta_sent = True
            return True
        if self._desc_offset >= len(self.descriptor):
            self._desc_master = None
            return False
        data[0:2] = (self._desc_offset & 0xFFFF).to_bytes(2, "little")
        room = frame_len - 2
        chunk = self.descriptor[self._desc_offset:self._desc_offset + room]
        data[2:2 + len(chunk)] = chunk
        self._send(mc_ep.CB_MSG_JSON_DESC_DATA, self._desc_master, 0, bytes(data))
        self._desc_offset += len(chunk)
        return True


CHECKS = {"pass": 0, "fail": 0}


def check(name, cond, detail=""):
    if cond:
        CHECKS["pass"] += 1
        print("  [ OK ] %s %s" % (name, detail))
    else:
        CHECKS["fail"] += 1
        print("  [FAIL] %s %s" % (name, detail))
    return cond


def expect_error(name, fn):
    try:
        fn()
    except mc_ep.EpError as e:
        return check(name, True, "-> %s" % e)
    return check(name, False, "没有报错")


def run(fd):
    label = "CAN FD" if fd else "Classic 1 Mbit/s"
    print("[%s]" % label)
    channel = "mc_ep_selftest_fd" if fd else "mc_ep_selftest"
    dev_bus = can.Bus(interface="virtual", channel=channel)
    fake = FakeDevice(dev_bus, fd).start()
    dev = None
    try:
        dev = mc_ep.Device(can_interface="virtual", can_channel=channel,
                           can_fd=fd, timeout=0.5)
        desc = dev.fetch_descriptor()
        check("描述符条目数", len(desc) == len(PARAMS), "%d 个" % len(desc))
        check("描述符 CRC16", dev.json_crc == fake.crc,
              "0x%04X" % dev.json_crc)

        check("float 读", dev.read("config.f") == 3.5)
        check("int8 读", dev.read("config.i8") == -7)
        check("bool 读", dev.read("config.b") is True)
        check("uint64 读", dev.read("config.u64") == 0x0102030405060708,
              "0x%016X" % dev.read("config.u64"))
        reads_before = fake.sdo_read_requests
        dev.read("config.u64")
        rounds = fake.sdo_read_requests - reads_before
        check("uint64 分段读请求数", rounds == (1 if fd else 2),
              "%d 次 %s" % (rounds, "(FD 一帧读完)" if fd else "(Classic 两次)"))

        check("float 写（读回原值）", dev.write("config.f", 4.0) == 4.0)
        check("float 写（读回限幅值）", dev.write("config.f", 99.0) == 5.0)
        check("写回存进设备", fake.values[10] == 5.0, str(fake.values[10]))
        check("uint64 写", dev.write("config.u64", 0x1122334455667788)
              == 0x1122334455667788, "0x%016X" % fake.values[11])
        check("uint64 写进设备", fake.values[11] == 0x1122334455667788)
        check("bool 写", dev.write("config.b", "false") is False)
        check("只写端点写（不读回）", dev.write("config.wo", 1.5) is None
              and fake.values[14] == 1.5)

        dev.call("action.save")
        check("动作端点（PARAM_READ 触发）", fake.actions == ["save"],
              str(fake.actions))

        expect_error("只写端点读取报错",
                     lambda: dev.link.read_value(14, mc_ep.TYPE_CODES["float"][1]))
        expect_error("只读端点写入报错", lambda: dev.write("telemetry.t", 5.0))
        check("只读/只写端点未发到线上", fake.values[12] == -7 and fake.values[15] == 1.25)
    finally:
        if dev is not None:
            dev.close()
        fake.stop()
        dev_bus.shutdown()


def check_wire_format():
    """线上格式用**写死的字面量**钉住（来自 can_cyberbeast.c / can.h）。

    下面的 fake device 用 mc_ep.cb_id / mc_ep.TYPE_CODES / mc_ep.crc16 自己算帧，
    所以只靠它跑通是“自证”；ID 移位、字节序、CRC 参数一起改坏时那些检查不会失败。
    """
    ident = mc_ep.cb_id(mc_ep.CB_PRI_CONFIG, mc_ep.CB_MSG_PARAM_READ, 1, 0xFE, 2)
    check("29-bit ID = pri|msgtype|dest|src|seq", ident == 0x108007FA,
          "0x%08X" % ident)
    check("ID 反解", mc_ep.cb_id_fields(0x108007FA) == (4, 0x20, 1, 0xFE, 2))
    check("float 小端", mc_ep.struct.pack(mc_ep.TYPE_CODES["float"][0], 3.5)
          == b"\x00\x00\x60\x40")
    check("uint64 小端", mc_ep.struct.pack(mc_ep.TYPE_CODES["uint64"][0],
                                           0x0102030405060708)
          == b"\x08\x07\x06\x05\x04\x03\x02\x01")
    check("描述符 CRC16（0x3D65 / init 0x1337）", mc_ep.crc16(b"{}") == 0x717E,
          "0x%04X" % mc_ep.crc16(b"{}"))


def main():
    try:
        check_wire_format()
        run(fd=False)
        run(fd=True)
    except mc_ep.EpError as e:
        print("[FAIL] 异常：%s" % e)
        CHECKS["fail"] += 1
    print("\n%d 通过 / %d 失败" % (CHECKS["pass"], CHECKS["fail"]))
    return 0 if CHECKS["fail"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
