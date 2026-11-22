"""协议常量 / 编解码回归（与 can_cyberbeast.c、endpoint.h 对齐）。"""

import struct

import pytest

from rmc.protocol import (
    CB_MSG_PARAM_READ, CB_PRI_CONFIG, TYPE_CODES, build_frame, cb_id, cb_id_fields,
    crc8, crc16, decode_value, encode_value, parse_value,
)


def test_29bit_id():
    ident = cb_id(CB_PRI_CONFIG, CB_MSG_PARAM_READ, 1, 0xFE, 2)
    assert ident == 0x108007FA
    assert cb_id_fields(0x108007FA) == (4, 0x20, 1, 0xFE, 2)
    assert cb_id_fields(ident)[1] == CB_MSG_PARAM_READ


def test_crc():
    # 描述符 CRC16 同时是接口版本，参数表一变就变
    assert crc16(b"{}") == 0x717E
    frame = build_frame(b"\x01\x02")
    assert frame[0] == 0xAA
    assert frame[1] == 2
    assert frame[2] == crc8(bytes([0xAA, 2]))
    assert frame[-2:] == struct.pack(">H", crc16(b"\x01\x02"))


def test_type_codes_little_endian():
    assert struct.pack(TYPE_CODES["float"][0], 3.5) == b"\x00\x00\x60\x40"
    assert struct.pack(TYPE_CODES["uint64"][0], 0x0102030405060708) \
        == b"\x08\x07\x06\x05\x04\x03\x02\x01"


def test_parse_value():
    assert parse_value("3.5", "float") == 3.5
    assert parse_value("0x10", "uint8") == 16
    assert parse_value("off", "bool") is False
    with pytest.raises(ValueError):
        parse_value("maybe", "bool")


def test_encode_decode_bool_and_reject_nan():
    ep = {"name": "config.b", "type": "bool", "access": "rw"}
    assert encode_value(ep, "false") == b"\x00"
    assert decode_value(ep, b"\x01") is True

    ep = {"name": "config.f", "type": "float", "access": "rw"}
    with pytest.raises(ValueError):
        encode_value(ep, float("nan"))


def test_encode_out_of_range():
    ep = {"name": "config.u8", "type": "uint8", "access": "rw"}
    with pytest.raises(ValueError):
        encode_value(ep, 300)
