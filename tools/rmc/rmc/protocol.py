"""线上协议常量与编解码。

必须与固件保持一致：
  Core/Inc/endpoint.h          请求/响应 payload、状态码
  Core/Src/can_cyberbeast.c    CYBERBEAST 的 29-bit ID 与 SDO/描述符消息
  Core/Inc/can.h               FD 仲裁域固定速率

USB（CDC / vendor interface）的 payload：
    请求: seq(2) | id|flags(2) | rxlen(2) | tx_data... | trailer(2)
    响应: seq|0x8000(2) | status(1) | data...
CAN（CYBERBEAST，无 status 字节）：
    PARAM_READ  请求 [0][ep u16 BE][want][offset u32 BE]
                响应 [flags][ep u16 BE][n][data]   flags bit7 = 还有后续字节
    PARAM_WRITE 请求 [flags][ep u16 BE][len][data] flags bit7 = 分段写未结束
                响应 [flags][ep u16 BE][0][0..]
"""

import struct

# ---- USB（CDC 0xAA 帧）---------------------------------------------------

FRAME_PREFIX = 0xAA
PROTOCOL_VERSION = 2
REPLY_FLAG = 0x8000
WRITE_FLAG = 0x4000

CRC8_POLY = 0x37
CRC8_INIT = 0x42
CRC16_POLY = 0x3D65
CRC16_INIT = 0x1337

STATUS_OK = 0
STATUS_VERSION_MISMATCH = 7

STATUS_NAMES = {
    0: "OK",
    1: "未知 endpoint ID",
    2: "长度错误",
    3: "数值非法（NaN）",
    4: "只读",
    5: "只写",
    6: "当前状态不允许",
    7: "接口版本不匹配（描述符已更新，请重新连接）",
}

# JSON 类型 -> (struct 格式, 字节数)
TYPE_CODES = {
    "bool": ("<B", 1),
    "int8": ("<b", 1),
    "uint8": ("<B", 1),
    "int16": ("<h", 2),
    "uint16": ("<H", 2),
    "int32": ("<i", 4),
    "uint32": ("<I", 4),
    "int64": ("<q", 8),
    "uint64": ("<Q", 8),
    "float": ("<f", 4),
    "double": ("<d", 8),
}

ACTION_TYPES = ("void", "function")

# ---- USB 设备身份（开发/台架板；出货板只有 CAN）-------------------------

DEFAULT_VID = 0x1209
DEFAULT_PID = 0x4342
DEFAULT_USB_INTERFACE = 2
DEFAULT_EP_OUT = 0x01   # EP1 OUT, fibre RX
DEFAULT_EP_IN = 0x83    # EP3 IN,  fibre TX
USB_READ_SIZE = 64

# ---- CYBERBEAST（CAN）---------------------------------------------------

CB_PRI_CONFIG = 4
CB_MSG_PARAM_READ = 0x20
CB_MSG_PARAM_WRITE = 0x21
CB_MSG_JSON_DESC_READ = 0x24
CB_MSG_JSON_DESC_DATA = 0x25
CB_READ_MORE = 0x80        # PARAM_READ 响应的 bit7：还有后续字节
CB_WRITE_MORE = 0x80       # PARAM_WRITE 请求的 bit7：分段写还没完

CAN_MSG_NAMES = {
    0x00: "MIT", 0x01: "CSP", 0x02: "CSV", 0x03: "TORQUE", 0x04: "CURRENT",
    0x20: "PARAM_READ", 0x21: "PARAM_WRITE", 0x22: "CONFIG_SAVE",
    0x23: "CONFIG_RESET", 0x24: "DESC_READ", 0x25: "DESC_DATA",
    0x40: "QUERY_STATUS", 0x41: "QUERY_POS_VEL", 0x42: "QUERY_CURRENT",
    0x43: "QUERY_TEMP", 0x44: "QUERY_BUS", 0x45: "QUERY_ERROR",
    0x46: "QUERY_DEV", 0x47: "QUERY_POWER", 0x48: "HEARTBEAT",
    0x49: "STATUS_FEEDBACK", 0x60: "SET_NODE_ID", 0x61: "SET_ZERO",
    0x62: "START", 0x63: "STOP", 0x64: "RESET", 0x65: "CLEAR_ERRORS",
    0x80: "MIT_BCAST", 0xC0: "ESTOP", 0xC1: "FAULT_ALERT",
}

# CANable/CandleLight 的 slcan 命令码（固件只认这几档）
SLCAN_BITRATES = {
    10000: "S0", 20000: "S1", 50000: "S2", 100000: "S3", 125000: "S4",
    250000: "S5", 500000: "S6", 750000: "S7", 1000000: "S8",
}
SLCAN_DATA_BITRATES = {2000000: "Y2", 5000000: "Y5"}
SLCAN_CODE2LEN = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
                  9: 12, 0xA: 16, 0xB: 20, 0xC: 24, 0xD: 32, 0xE: 48, 0xF: 64}
SLCAN_LEN2CODE = {v: k for k, v in SLCAN_CODE2LEN.items()}

DEFAULT_CAN_INTERFACE = "slcan"      # CANable / CandleLight 串口
DEFAULT_CAN_BITRATE = 1000000        # = config.can_baud 出厂值（Classic）
CAN_FD_NOMINAL_BAUD = 1000000        # = can.h 的 CAN_FD_NOMINAL_BAUD
DEFAULT_CAN_DATA_BITRATE = 5000000   # = config.can_baud > 1M 时的常用值
DEFAULT_CAN_NODE_ID = 1              # = config.can_id 出厂值
DEFAULT_CAN_MASTER_ID = 0xFE


# ---- 校验与帧 ------------------------------------------------------------


def crc8(data, remainder=CRC8_INIT):
    for value in data:
        remainder ^= value
        for _ in range(8):
            if remainder & 0x80:
                remainder = ((remainder << 1) ^ CRC8_POLY) & 0xFF
            else:
                remainder = (remainder << 1) & 0xFF
    return remainder


def crc16(data, remainder=CRC16_INIT):
    for value in data:
        remainder ^= value << 8
        for _ in range(8):
            if remainder & 0x8000:
                remainder = ((remainder << 1) ^ CRC16_POLY) & 0xFFFF
            else:
                remainder = (remainder << 1) & 0xFFFF
    return remainder


def build_frame(payload):
    """CDC 外层：AA | LEN | CRC8 | payload | CRC16(HI,LO)"""
    if len(payload) > 64:
        raise ValueError("请求太长（%d 字节）" % len(payload))
    header = bytes([FRAME_PREFIX, len(payload)])
    crc = crc16(payload)
    return header + bytes([crc8(header)]) + payload + bytes([crc >> 8, crc & 0xFF])


def cb_id(pri, msgtype, dest, src, seq=0):
    """CYBERBEAST 29-bit ID = Priority(3)|MsgType(8)|Dest(8)|Source(8)|Seq(2)"""
    return ((pri & 7) << 26) | ((msgtype & 0xFF) << 18) | ((dest & 0xFF) << 10) \
        | ((src & 0xFF) << 2) | (seq & 3)


def cb_id_fields(value):
    """-> (priority, msgtype, dest, source, seq)"""
    return ((value >> 26) & 7, (value >> 18) & 0xFF, (value >> 10) & 0xFF,
            (value >> 2) & 0xFF, value & 3)


# ---- 值编解码 ------------------------------------------------------------


def is_action(ep):
    return ep["type"] in ACTION_TYPES


def is_readable(ep):
    return (not is_action(ep)) and ep["access"] in ("r", "rw")


def is_writable(ep):
    return (not is_action(ep)) and ep["access"] in ("w", "rw")


def type_size(type_name):
    try:
        return TYPE_CODES[type_name][1]
    except KeyError:
        raise ValueError("不支持的参数类型: %s" % type_name)


def decode_value(ep, data):
    type_name = ep["type"]
    if type_name not in TYPE_CODES:
        raise ValueError("%s: 不支持的参数类型 %s" % (ep["name"], type_name))
    fmt, size = TYPE_CODES[type_name]
    if len(data) != size:
        raise ValueError("%s: 返回 %d 字节，期望 %d" % (ep["name"], len(data), size))
    value = struct.unpack(fmt, data)[0]
    return bool(value) if type_name == "bool" else value


def parse_value(text, type_name):
    """命令行字符串 -> Python 值（供 `rmc set` 用）。"""
    if type_name == "bool":
        lowered = str(text).strip().lower()
        if lowered in ("1", "true", "on", "yes"):
            return True
        if lowered in ("0", "false", "off", "no"):
            return False
        raise ValueError("bool 参数只接受 0/1、true/false、on/off、yes/no")
    if type_name in ("float", "double"):
        return float(text)
    if type_name.startswith(("int", "uint")):
        return int(text, 0)     # 允许 0x10 / 0b1010
    return str(text)


def encode_value(ep, value):
    """Python 值 -> 线上字节。bool 也接受字符串（"false" / "off" …）。"""
    type_name = ep["type"]
    if type_name not in TYPE_CODES:
        raise ValueError("%s: 不支持的参数类型 %s" % (ep["name"], type_name))
    if type_name == "bool" and not isinstance(value, bool):
        value = parse_value(value, "bool")
    elif type_name in ("float", "double"):
        value = float(value)
        if value != value or value in (float("inf"), float("-inf")):
            raise ValueError("%s: 不接受 NaN/Inf" % ep["name"])
    elif type_name.startswith(("int", "uint")):
        value = int(value)
    fmt, _size = TYPE_CODES[type_name]
    try:
        return struct.pack(fmt, value)
    except struct.error as e:
        raise ValueError("%s: %s 超出 %s 的范围（%s）"
                         % (ep["name"], value, type_name, e))
