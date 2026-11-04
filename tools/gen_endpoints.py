#!/usr/bin/env python3
"""
从 params.json 生成 endpoint_defs.h / endpoint_defs.c。

设计要点（与 ODrive 的 fibre 一致）：
  * params.json 是唯一真源；
  * endpoint id 由 endpoint_ids.json 锁定，只增不改，固件升级不破坏上位机；
  * 生成的 endpoint_defs.c 里嵌一份紧凑 JSON 描述符，设备因此可以自描述；
  * 描述符的 CRC16 同时作为“接口版本号”，旧上位机的请求会被拒绝。

用法:
    python tools/gen_endpoints.py                      # 生成并更新 ID 注册表
    python tools/gen_endpoints.py --check              # 只检查生成结果是否最新
    python tools/gen_endpoints.py --out . --ids tools/endpoint_ids.json

生成的 C 代码约定:
    * `v` 是解码并校验/限幅后的写入值；
    * `ref`  是可直接读写的 C 左值（宏也可以，例如 user_config.h 里的 I_MAX）；
    * `get`  是只读表达式；
    * `set`  是写入语句（可以引用 v）；
    * `call` 是动作端点的语句；
    * `require` 为假的写入返回 EP_ERR_STATE。

线上描述符（设备回给上位机的那份）有两条硬约束，改 descriptor_json() 时别破坏：
    * 纯 ASCII：中文 brief 只留在 params.json，不进描述符（JointSDK/ODrive 的
      轻量解析器不支持 \\uXXXX 转义）；
    * 动作端点写 "type":"function"（空 inputs/outputs），不能写 "void"
      （JointSDK 的解析器不认识 void，会整体解析失败）。
"""

import argparse
import json
import os
import re
import sys

# ---------------------------------------------------------------------------
# 类型表
# ---------------------------------------------------------------------------

TYPES = {
    "bool":   {"c": "uint8_t",  "size": 1, "get": "ep_get_u8",  "put": "ep_put_u8",  "kind": "uint"},
    "int8":   {"c": "int8_t",   "size": 1, "get": "ep_get_i8",  "put": "ep_put_i8",  "kind": "int"},
    "uint8":  {"c": "uint8_t",  "size": 1, "get": "ep_get_u8",  "put": "ep_put_u8",  "kind": "uint"},
    "int16":  {"c": "int16_t",  "size": 2, "get": "ep_get_i16", "put": "ep_put_i16", "kind": "int"},
    "uint16": {"c": "uint16_t", "size": 2, "get": "ep_get_u16", "put": "ep_put_u16", "kind": "uint"},
    "int32":  {"c": "int32_t",  "size": 4, "get": "ep_get_i32", "put": "ep_put_i32", "kind": "int"},
    "uint32": {"c": "uint32_t", "size": 4, "get": "ep_get_u32", "put": "ep_put_u32", "kind": "uint"},
    "uint64": {"c": "uint64_t", "size": 8, "get": "ep_get_u64", "put": "ep_put_u64", "kind": "uint"},
    "float":  {"c": "float",    "size": 4, "get": "ep_get_f32", "put": "ep_put_f32", "kind": "float"},
    "void":   {"c": None,       "size": 0},
    "blob":   {"c": None,       "size": 0},   # 只用于 hidden 持久化条目，不进 wire 描述符
}

ACCESS = ("r", "w", "rw")

PREFS_TYPE = {
    "bool":   "PREFS_T_BOOL",
    "uint8":  "PREFS_T_U8",
    "int8":   "PREFS_T_I8",
    "uint16": "PREFS_T_U16",
    "int16":  "PREFS_T_I16",
    "uint32": "PREFS_T_U32",
    "int32":  "PREFS_T_I32",
    "float":  "PREFS_T_F32",
    "blob":   "PREFS_T_BLOB",
}

NAME_RE = re.compile(r"^[a-z][a-z0-9_]*(\.[a-z][a-z0-9_]*)+$")


# ---------------------------------------------------------------------------
# CRC（与 Core/Src/endpoint.c 逐位一致）
# ---------------------------------------------------------------------------

def crc16(data, remainder=0x1337, poly=0x3D65):
    for value in data:
        remainder ^= value << 8
        for _ in range(8):
            if remainder & 0x8000:
                remainder = ((remainder << 1) ^ poly) & 0xFFFF
            else:
                remainder = (remainder << 1) & 0xFFFF
    return remainder


# ---------------------------------------------------------------------------
# 校验与 ID 分配
# ---------------------------------------------------------------------------

def fail(message):
    sys.stderr.write("gen_endpoints: %s\n" % message)
    sys.exit(1)


def load_params(path):
    with open(path, "r", encoding="utf-8") as f:
        doc = json.load(f)

    if doc.get("protocol_version", 1) != 1:
        fail("protocol_version 必须与 Core/Inc/endpoint.h 的 EP_PROTOCOL_VERSION 一致（当前为 1）")

    endpoints = doc.get("endpoints")
    if not isinstance(endpoints, list) or not endpoints:
        fail("params.json 里没有 endpoints 数组")

    names = set()
    enum_names = {}

    for i, ep in enumerate(endpoints):
        where = "endpoints[%d]" % i
        name = ep.get("name")
        if not isinstance(name, str) or not NAME_RE.match(name):
            fail("%s: name 必须是扁平点分小写名字，例如 config.i_max" % where)
        if name in names:
            fail("%s: 名字重复: %s" % (where, name))
        names.add(name)

        enum = "EP_" + re.sub(r"[^A-Za-z0-9]", "_", name).upper()
        if enum in enum_names:
            fail("%s: 枚举名冲突: %s (%s 和 %s)" % (where, enum, enum_names[enum], name))
        enum_names[enum] = name
        ep["_enum"] = enum

        t = ep.get("type")
        if t not in TYPES:
            fail("%s: 未知类型 %r（可选：%s）" % (where, t, ", ".join(sorted(TYPES))))

        hidden = bool(ep.get("hidden"))
        persist = bool(ep.get("persist"))

        if hidden:
            # hidden 条目只进持久化表，不生成描述符/handler。
            if not persist:
                fail("%s: hidden 端点必须 persist:true" % where)
            if "ref" not in ep:
                fail("%s: hidden 端点需要 ref" % where)
            if t in ("void",):
                fail("%s: hidden 不能是 void" % where)
            if t == "blob":
                if not isinstance(ep.get("size"), int) or ep["size"] <= 0:
                    fail("%s: blob 需要正整数 size" % where)
            elif t not in TYPES or t == "blob":
                fail("%s: hidden 类型必须可持久化" % where)
        else:
            access = ep.get("access")
            if access not in ACCESS:
                fail("%s: access 必须是 r / w / rw" % where)
            if t == "blob":
                fail("%s: blob 只能用于 hidden 持久化条目" % where)

            if t == "void":
                if access != "w":
                    fail("%s: void 端点只能写（access: w）" % where)
                if "call" not in ep:
                    fail("%s: void 端点需要 call" % where)
            else:
                if access in ("r", "rw") and not ("get" in ep or "ref" in ep):
                    fail("%s: 可读端点需要 get 或 ref" % where)
                if access in ("w", "rw") and not ("set" in ep or "ref" in ep):
                    fail("%s: 可写端点需要 set 或 ref" % where)

        if persist and t != "blob" and "ref" not in ep:
            fail("%s: 持久化标量需要 ref（用于访问 RAM 存储）" % where)
        # 擦除过的芯片/新芯片上没有记录，只能靠 default。缺了它，参数就是 .bss
        # 的 0 —— I_MAX=0 会在第一个控制周期锁存过流故障。
        if persist and t != "blob" and "default" not in ep:
            fail("%s: 持久化条目需要 default（芯片擦除时用）" % where)

        if "default" in ep:
            d = ep["default"]
            if not isinstance(d, (str, int, float)) or isinstance(d, bool):
                fail("%s: default 必须是数字或 C 表达式字符串" % where)
            if t == "blob":
                fail("%s: blob 默认值为全 0，不要写 default" % where)
            if not isinstance(d, str):
                if t == "float":
                    ep["default"] = float(d)
                else:
                    if not isinstance(d, int):
                        fail("%s: 整数类型的 default 必须是整数" % where)

        for key in ("min", "max"):
            if key in ep and not isinstance(ep[key], (int, float)):
                fail("%s: %s 必须是数字" % (where, key))
        if "min" in ep and "max" in ep and ep["min"] > ep["max"]:
            fail("%s: min > max" % where)

        # range_dynamic 只影响线上描述符：C 侧仍然用 min/max(+expr) 限幅，
        # 所以两者必须都在，否则既没有描述符范围也没有限幅。
        if ep.get("range_dynamic"):
            if "min" not in ep or "max" not in ep:
                fail("%s: range_dynamic 仍需要 min/max（C 侧限幅用）" % where)
            if not (("min_expr" in ep) or ("max_expr" in ep)):
                fail("%s: range_dynamic 需要 min_expr/max_expr 说明运行期范围" % where)

    return doc, endpoints


def load_registry(path):
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            reg = json.load(f)
    else:
        reg = {}
    reg.setdefault("next_id", 1)
    reg.setdefault("map", {})
    return reg


def assign_ids(endpoints, reg, check):
    used = {}
    for name, eid in reg["map"].items():
        if eid in used:
            fail("ID 注册表里 %d 被 %s 和 %s 同时占用" % (eid, used[eid], name))
        used[eid] = name

    changed = False
    for ep in endpoints:
        name = ep["name"]
        if name in reg["map"]:
            ep["id"] = reg["map"][name]
            continue
        if check:
            fail("endpoint %s 还没有 ID；先运行 `make endpoints` 生成" % name)
        eid = reg["next_id"]
        while eid == 0 or eid in used:
            eid += 1
        reg["next_id"] = eid + 1
        reg["map"][name] = eid
        used[eid] = name
        ep["id"] = eid
        changed = True

    for ep in endpoints:
        if ep["id"] == 0:
            fail("endpoint %s 不能使用保留 ID 0" % ep["name"])

    return changed


# ---------------------------------------------------------------------------
# 紧凑描述符
# ---------------------------------------------------------------------------

def descriptor_json(endpoints):
    out = []
    for ep in endpoints:
        if ep.get("hidden"):
            continue        # 只进持久化表，不暴露给上位机
        # 动作端点在 params.json 里写成 void（生成 C 时没有值），但线上描述符
        # 必须用 ODrive/Fibre 的 function 表示，否则 JointSDK 的解析器会以
        # "unknown endpoint type" 整体失败。
        wire_type = "function" if ep["type"] == "void" else ep["type"]
        item = {
            "name": ep["name"],
            "id": ep["id"],
            "type": wire_type,
            "access": ep["access"],
        }
        if wire_type == "function":
            item["inputs"] = []
            item["outputs"] = []
        # 注意：brief 只留在 params.json 里做人读文档，不进设备描述符。
        # 中文 brief 会变成 \uXXXX 转义，而 JointSDK/ODrive 的轻量 JSON 解析器
        # 只支持基础转义（不支持 \u），会直接解析失败。
        for key in ("unit", "min", "max"):
            if key not in ep:
                continue
            # range_dynamic：真实范围由固件宏/用户配置在运行期决定，JSON 里的数字
            # 只是给 C 限幅用的名义值，不能发给上位机（否则 UI 会按错的范围缩放）。
            if ep.get("range_dynamic") and key in ("min", "max"):
                continue
            item[key] = ep[key]
        if ep.get("persist"):
            item["persist"] = True      # 上位机 backup/restore 用；SDK 会忽略未知字段
        out.append(item)
    return json.dumps(out, ensure_ascii=True, separators=(",", ":"))


def c_string_literal(text, chunk=800):
    parts = []
    for i in range(0, len(text), chunk):
        part = text[i:i + chunk].replace("\\", "\\\\").replace('"', '\\"')
        parts.append('"%s"' % part)
    return "\n".join(parts)


# ---------------------------------------------------------------------------
# C 代码生成
# ---------------------------------------------------------------------------

def num(v):
    return repr(float(v)) if isinstance(v, float) else str(v)


def require_check(expr):
    """生成“require 为假则拒绝”的 C 语句；开头的 ! 直接省掉一层括号。"""
    if expr.startswith("!"):
        return "if(%s){return EP_ERR_STATE;}" % expr[1:].strip()
    return "if(!(%s)){return EP_ERR_STATE;}" % expr


def bound(ep, key):
    """上下限的 C 表达式。*_expr 允许直接引用固件宏（例如 I_MAX_LIMIT），
    避免 JSON 里的数字和 hw_config.h 各写一份、慢慢漂移。"""
    expr = ep.get(key + "_expr")
    return expr if expr else num(ep[key])


def limit_c(ep, key):
    """config_apply_limits() 用：浮点字面量补 f 后缀，宏表达式原样使用。"""
    expr = ep.get(key + "_expr")
    if expr:
        return expr
    return num(ep[key]) + ("f" if TYPES[ep["type"]]["kind"] == "float" else "")


def gen_write_body(ep, indent):
    t = TYPES[ep["type"]]
    pad = "\t" * indent
    lines = []

    if "require" in ep:
        lines.append(pad + require_check(ep["require"]))

    if t["kind"] == "float":
        lines.append("%sfloat v;" % pad)
        lines.append("%sif(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}" % pad)
        lines.append("%sif(!isfinite(v)){return EP_ERR_BAD_VALUE;}" % pad)
        if "min" in ep and "max" in ep:
            lines.append("%sv = ep_clampf(v, %s, %s);" % (pad, bound(ep, "min"), bound(ep, "max")))
        elif "min" in ep:
            lines.append("%sif(v < %s){v = %s;}" % (pad, bound(ep, "min"), bound(ep, "min")))
        elif "max" in ep:
            lines.append("%sif(v > %s){v = %s;}" % (pad, bound(ep, "max"), bound(ep, "max")))
    elif t["kind"] in ("int", "uint"):
        ctype = t["c"]
        lines.append("%s%s v;" % (pad, ctype))
        lines.append("%sif(!%s(in, &v)){return EP_ERR_BAD_LEN;}" % (pad, t["get"]))
        if ep["type"] == "bool":
            lines.append("%sif(v > 1){v = 1;}" % pad)
        else:
            if "min" in ep and "max" in ep:
                if t["kind"] == "int":
                    lines.append("%sv = (%s)ep_clampi(v, %s, %s);" % (pad, ctype, bound(ep, "min"), bound(ep, "max")))
                else:
                    lines.append("%sv = (%s)ep_clampu((uint32_t)v, %s, %s);" % (pad, ctype, bound(ep, "min"), bound(ep, "max")))
            elif "min" in ep:
                lines.append("%sif(v < %s){v = %s;}" % (pad, bound(ep, "min"), bound(ep, "min")))
            elif "max" in ep:
                lines.append("%sif(v > %s){v = %s;}" % (pad, bound(ep, "max"), bound(ep, "max")))

    if "set" in ep:
        lines.append("%s%s" % (pad, ep["set"]))
    elif "ref" in ep:
        lines.append("%s%s = v;" % (pad, ep["ref"]))
    else:
        fail("%s: 可写端点缺少 set/ref" % ep["name"])

    return lines


def gen_read_body(ep, indent):
    t = TYPES[ep["type"]]
    pad = "\t" * indent
    expr = ep.get("get", ep.get("ref"))
    ctype = t["c"]
    return [
        "%sif(!%s(out, (%s)(%s))){return EP_ERR_BAD_LEN;}" % (pad, t["put"], ctype, expr),
        "%sreturn EP_OK;" % pad,
    ]


def gen_case(ep):
    access = ep["access"]
    t = ep["type"]
    lines = ["\t\tcase %s: {" % ep["_enum"]]

    if t == "void":
        # 动作端点没有数据：必须由传输层明确声明“这是写/执行”。只靠
        # “载荷为空”判断会让一次读把电机停掉。
        lines.append("\t\t\tif(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}")
        if "require" in ep:
            lines.append("\t\t\t" + require_check(ep["require"]))
        lines.append("\t\t\t%s" % ep["call"])
        lines.append("\t\t\treturn EP_OK;")
    elif access == "r":
        lines.append("\t\t\tif(op != EP_OP_READ){return EP_ERR_READ_ONLY;}")
        lines.extend(gen_read_body(ep, 3))
    elif access == "w":
        lines.append("\t\t\tif(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}")
        lines.extend(gen_write_body(ep, 3))
        lines.append("\t\t\treturn EP_OK;")
    else:  # rw
        # 写完再回读新值（上位机 set 会打印它）；读路径在末尾统一返回
        lines.append("\t\t\tif(op == EP_OP_WRITE){")
        lines.extend(gen_write_body(ep, 4))
        lines.append("\t\t\t}")
        # 写分支已把长度消耗掉；这里还有剩余字节说明不是合法请求
        lines.append("\t\t\tif(in->begin != in->end){return EP_ERR_BAD_LEN;}")
        lines.extend(gen_read_body(ep, 3))

    lines.append("\t\t}")
    return lines


def generate_header(endpoints):
    lines = [
        "/*",
        " * endpoint_defs.h",
        " *",
        " * 由 tools/gen_endpoints.py 从 params.json 生成，请勿手工修改。",
        " */",
        "#ifndef ENDPOINT_DEFS_H",
        "#define ENDPOINT_DEFS_H",
        "",
        "#include <stdint.h>",
        '#include "endpoint.h"',
        "",
        "typedef enum{",
        "\tEP_ID_DESC = 0,",
    ]
    for ep in endpoints:
        if ep.get("hidden"):
            continue
        lines.append("\t%s = %d," % (ep["_enum"], ep["id"]))
    lines += [
        "}endpoint_id_t;",
        "",
        "extern const char endpoint_json[];",
        "extern const uint16_t endpoint_json_len;",
        "extern const uint16_t endpoint_json_crc;",
        "",
        "ep_status_t endpoint_handler(uint16_t id, ep_op_t op, ep_cbuf_t *in, ep_buf_t *out);",
        "",
        "#endif /* ENDPOINT_DEFS_H */",
        "",
    ]
    return "\n".join(lines)


def generate_source(endpoints, descriptor):
    crc = crc16(descriptor.encode("ascii"))
    if len(descriptor) > 0xFFFF:
        fail("描述符太长（%d 字节），超过 16 位长度字段" % len(descriptor))

    lines = [
        "/*",
        " * endpoint_defs.c",
        " *",
        " * 由 tools/gen_endpoints.py 从 params.json 生成，请勿手工修改。",
        " */",
        "",
        "#include <math.h>",
        "#include <string.h>",
        '#include "endpoint_defs.h"',
        '#include "endpoint.h"',
        '#include "user_config.h"',
        '#include "hw_config.h"',
        '#include "structs.h"',
        '#include "timing.h"',
        '#include "main.h"',
        "",
        "const char endpoint_json[] =",
        c_string_literal(descriptor) + ";",
        "const uint16_t endpoint_json_len = sizeof(endpoint_json) - 1;",
        "const uint16_t endpoint_json_crc = 0x%04X;" % crc,
        "",
        "ep_status_t endpoint_handler(uint16_t id, ep_op_t op, ep_cbuf_t *in, ep_buf_t *out){",
        "\tswitch(id){",
    ]

    for ep in endpoints:
        if ep.get("hidden"):
            continue
        lines.extend(gen_case(ep))

    lines += [
        "\t\tdefault:",
        "\t\t\treturn EP_ERR_UNKNOWN_ID;",
        "\t}",
        "}",
        "",
    ]
    source = "\n".join(lines)

    # 自检：每个 case 都必须以 return 结束。漏掉 return 会静默 fall through
    # 到下一个端点（比如写 p_min 顺手改了 v_max），编译器不会报错。
    body = source.split("switch(id){", 1)[1]
    for chunk in body.split("\t\tcase ")[1:]:
        name = chunk.split(":", 1)[0].strip()
        if "return" not in chunk.split("\t\tdefault:")[0]:
            fail("%s: 生成的 case 没有 return（会 fall through）" % name)
    return source


# ---------------------------------------------------------------------------
# 持久化表 / 默认值 / 限幅
# ---------------------------------------------------------------------------

def default_literal(ep):
    v = ep["default"]
    if isinstance(v, str):
        return v                      # C 表达式，例如 I_MAX_LIMIT
    if ep["type"] == "float":
        return ("%r" % float(v)) + "f"
    if ep["type"] == "bool":
        return "1" if v else "0"
    return str(int(v))


def prefs_size(ep):
    return ep["size"] if ep["type"] == "blob" else TYPES[ep["type"]]["size"]


def generate_prefs_header():
    return "\n".join([
        "/*",
        " * prefs_defs.h",
        " *",
        " * 由 tools/gen_endpoints.py 从 params.json 生成，请勿手工修改。",
        " */",
        "#ifndef PREFS_DEFS_H",
        "#define PREFS_DEFS_H",
        "",
        "#include <stdint.h>",
        '#include "prefs_codec.h"',
        "",
        "extern const prefs_entry_t prefs_entries[];",
        "extern const unsigned prefs_entry_count;",
        "",
        "/* 把缺失记录所需的默认值写进 RAM（上电时先调用它，再加载记录） */",
        "void config_apply_defaults(void);",
        "",
        "/* 对加载后的配置做 NaN/Inf 检查和 min/max 限幅 */",
        "void config_apply_limits(void);",
        "",
        "#endif /* PREFS_DEFS_H */",
        "",
    ])


def load_prefs_limits(out):
    """从 prefs_codec.h 读容量上限，避免生成器和编解码层各写一份数字。"""
    path = os.path.join(out, "Core", "Inc", "prefs_codec.h")
    lim = {"MAX_PAYLOAD": 4096, "MAX_RECORDS": 256, "MAX_VALUE": 512, "REC_HDR_LEN": 8}
    try:
        text = open(path, "r", encoding="utf-8").read()
    except OSError:
        return lim
    for key, name in (("MAX_PAYLOAD", "PREFS_MAX_PAYLOAD"),
                      ("MAX_RECORDS", "PREFS_MAX_RECORDS"),
                      ("MAX_VALUE", "PREFS_MAX_VALUE"),
                      ("REC_HDR_LEN", "PREFS_REC_HDR_LEN")):
        m = re.search(r"#define\s+%s\s+(\d+)u?" % name, text)
        if not m:
            fail("%s 里找不到 %s" % (path, name))
        lim[key] = int(m.group(1))
    return lim


def generate_prefs_source(endpoints, lim):
    persist = sorted([e for e in endpoints if e.get("persist")], key=lambda e: e["id"])

    # 容量必须在生成期就检查：prefs_block_build() 对超限条目是静默跳过的，
    # 等到运行期才发现，表现就是“标定/配置某次保存后悄悄没了”。
    if len(persist) > lim["MAX_RECORDS"]:
        fail("持久化条目 %d 条，超过 PREFS_MAX_RECORDS=%d" % (len(persist), lim["MAX_RECORDS"]))
    payload = 0
    for ep in persist:
        size = prefs_size(ep)
        if size <= 0 or size > lim["MAX_VALUE"]:
            fail("%s: 值 %d 字节，超出 PREFS_MAX_VALUE=%d" % (ep["name"], size, lim["MAX_VALUE"]))
        payload += lim["REC_HDR_LEN"] + ((size + 3) & ~3)
    if payload > lim["MAX_PAYLOAD"]:
        fail("持久化载荷 %d 字节，超过 PREFS_MAX_PAYLOAD=%d" % (payload, lim["MAX_PAYLOAD"]))

    lines = [
        "/*",
        " * prefs_defs.c",
        " *",
        " * 由 tools/gen_endpoints.py 从 params.json 生成，请勿手工修改。",
        " */",
        "",
        "#include <math.h>",
        "#include <string.h>",
        '#include "prefs_defs.h"',
        '#include "prefs_codec.h"',
        '#include "user_config.h"',
        '#include "hw_config.h"',
        '#include "position_sensor.h"',
        "",
        "/* 顺序按 endpoint ID 升序，便于 diff 和线性查找 */",
        "const prefs_entry_t prefs_entries[] = {",
    ]

    for ep in persist:
        # ENCODER_LUT 是 __int_reg[6] 这样的左值，必须取地址；它本身不是数组
        ptr = "(void *)&(%s)" % ep["ref"]
        lines.append("\t{%d, %s, %d, %s}," % (
            ep["id"], PREFS_TYPE[ep["type"]], prefs_size(ep), ptr))

    lines += [
        "};",
        "const unsigned prefs_entry_count = sizeof(prefs_entries) / sizeof(prefs_entries[0]);",
        "",
        "_Static_assert(sizeof(prefs_entries) / sizeof(prefs_entries[0]) <= %du,"
        % lim["MAX_RECORDS"],
        "            \"params.json: 持久化条目超过 PREFS_MAX_RECORDS\");",
    ]
    for ep in persist:
        if "assert" in ep:
            lines.append("_Static_assert(%s, \"%s: 尺寸与固件不一致\");" % (ep["assert"], ep["name"]))
    lines += [
        "",
        "void config_apply_defaults(void){",
    ]

    for ep in persist:
        if ep["type"] == "blob":
            lines.append("\tmemset((void *)&(%s), 0, %d);" % (ep["ref"], ep["size"]))
        elif "default" in ep:
            lines.append("\t%s = %s;" % (ep["ref"], default_literal(ep)))

    lines += [
        "}",
        "",
        "void config_apply_limits(void){",
    ]

    for ep in persist:
        if ep["type"] == "blob":
            continue
        ref = ep["ref"]
        t = ep["type"]
        if t == "float":
            if "default" in ep:
                lines.append("\tif(!isfinite(%s)){%s = %s;}" % (ref, ref, default_literal(ep)))
            else:
                lines.append("\tif(!isfinite(%s)){%s = 0.0f;}" % (ref, ref))
            if "min" in ep:
                lo = limit_c(ep, "min")
                lines.append("\tif(%s < %s){%s = %s;}" % (ref, lo, ref, lo))
            if "max" in ep:
                hi = limit_c(ep, "max")
                lines.append("\tif(%s > %s){%s = %s;}" % (ref, hi, ref, hi))
        elif t == "bool":
            lines.append("\tif(%s > 1){%s = 1;}" % (ref, ref))
        else:
            if "min" in ep:
                lines.append("\tif(%s < %s){%s = %s;}" % (ref, num(ep["min"]), ref, num(ep["min"])))
            if "max" in ep:
                lines.append("\tif(%s > %s){%s = %s;}" % (ref, num(ep["max"]), ref, num(ep["max"])))

    lines += [
        "}",
        "",
    ]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 文档生成（参数手册 / 端点索引 / 机器可读描述符）
# ---------------------------------------------------------------------------

PREFIX_TITLES = [
    ("config", "config：可写参数"),
    ("command", "command：运行命令（电机模式下有效）"),
    ("telemetry", "telemetry：实时测量（只读）"),
    ("timing", "timing：ISR 耗时统计（只读）"),
    ("system", "system：状态与版本（只读）"),
    ("action", "action：动作（只写）"),
]


def fmt_value(ep, key):
    if key + "_expr" in ep:
        return "`%s`" % ep[key + "_expr"]
    v = ep.get(key)
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v)


def fmt_range(ep):
    if ep.get("range_dynamic"):
        lo = ("`%s`" % ep["min_expr"]) if "min_expr" in ep else \
             (fmt_value(ep, "min") if "min" in ep else "")
        hi = ("`%s`" % ep["max_expr"]) if "max_expr" in ep else \
             (fmt_value(ep, "max") if "max" in ep else "")
        if lo and hi:
            return "运行期：%s .. %s" % (lo, hi)
        if hi:
            return "运行期：≤ %s" % hi
        if lo:
            return "运行期：≥ %s" % lo
        return "运行期"
    lo = fmt_value(ep, "min") if "min" in ep else ""
    hi = fmt_value(ep, "max") if "max" in ep else ""
    if lo and hi:
        return "%s .. %s" % (lo, hi)
    if hi:
        return "≤ %s" % hi
    if lo:
        return "≥ %s" % lo
    return "—"


def generate_docs(endpoints):
    """只生成一份按分组排列的参数手册（docs/parameters.md）。

    端点索引/描述符副本不再单独出文件：params.json 就是索引，设备本身也会回送
    描述符（`mc_ep.py descriptor`）。文档数量越少越不容易过期。
    """
    visible = [e for e in endpoints if not e.get("hidden")]
    lines = [
        "<!-- 由 tools/gen_endpoints.py 从 params.json 生成，请勿手工修改。 -->",
        "# 参数手册",
        "",
        "ID 由 `tools/endpoint_ids.json` 锁定，固件升级不会改变；`access` 为 `r` 的只能读，",
        "`w` 的只能写/执行，`rw` 可读写。写入值在固件里做 NaN 检查和上下限限幅。",
        "`function` 就是 `action.*` 动作端点。`默认` 列偶尔是固件宏（如 `I_MAX_LIMIT`），",
        "以固件为准。",
        "",
    ]
    for prefix, title in PREFIX_TITLES:
        group = [e for e in visible if e["name"].split(".")[0] == prefix]
        if not group:
            continue
        lines += ["## " + title, "",
                  "| 名称 | ID | 类型 | 访问 | 单位 | 范围 | 默认 | 持久化 | 说明 |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for ep in group:
            lines.append("| `%s` | %d | %s | %s | %s | %s | %s | %s | %s |" % (
                ep["name"], ep["id"], "function" if ep["type"] == "void" else ep["type"],
                ep["access"],
                ep.get("unit", ""), fmt_range(ep),
                fmt_value(ep, "default") if "default" in ep else "—",
                "是" if ep.get("persist") else "否",
                ep.get("brief", "").replace("|", "\\|")))
        lines.append("")
    return {"docs/parameters.md": "\n".join(lines)}


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------

def write_if_changed(path, content, check):
    old = None
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8", newline="") as f:
            old = f.read()
    if old == content:
        return False
    if check:
        sys.stderr.write("gen_endpoints: 需要重新生成 %s\n" % path)
        return True
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(content)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", default="params.json")
    ap.add_argument("--ids", default="tools/endpoint_ids.json")
    ap.add_argument("--out", default=".")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    doc, endpoints = load_params(args.params)
    reg = load_registry(args.ids)
    ids_changed = assign_ids(endpoints, reg, args.check)

    descriptor = descriptor_json(endpoints)
    header = generate_header(endpoints)
    source = generate_source(endpoints, descriptor)
    prefs_header = generate_prefs_header()
    prefs_source = generate_prefs_source(endpoints, load_prefs_limits(args.out))
    docs = generate_docs(endpoints)

    changed = False
    changed |= write_if_changed(os.path.join(args.out, "Core", "Inc", "endpoint_defs.h"), header, args.check)
    changed |= write_if_changed(os.path.join(args.out, "Core", "Src", "endpoint_defs.c"), source, args.check)
    changed |= write_if_changed(os.path.join(args.out, "Core", "Inc", "prefs_defs.h"), prefs_header, args.check)
    changed |= write_if_changed(os.path.join(args.out, "Core", "Src", "prefs_defs.c"), prefs_source, args.check)
    for rel, content in docs.items():
        changed |= write_if_changed(os.path.join(args.out, rel), content, args.check)

    if args.check:
        if changed:
            fail("生成文件不是最新的；运行 `make endpoints`")
        print("endpoint 生成结果已是最新（%d 个端点，描述符 %d 字节）" % (len(endpoints), len(descriptor)))
        return

    if ids_changed:
        reg["map"] = dict(sorted(reg["map"].items(), key=lambda kv: kv[1]))
        with open(args.ids, "w", encoding="utf-8", newline="") as f:
            json.dump(reg, f, indent=2, ensure_ascii=False)
            f.write("\n")

    print("生成 %d 个端点，描述符 %d 字节，json_crc=0x%04X" %
          (len(endpoints), len(descriptor), crc16(descriptor.encode("ascii"))))


if __name__ == "__main__":
    main()
