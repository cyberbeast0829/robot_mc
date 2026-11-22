"""CLI 与 shell 共用的操作：scan / probe / monitor。"""

import sys
import time

from .errors import EpError
from .format import format_value
from .protocol import CAN_MSG_NAMES
from .transport.can import CanLink


def scan(seconds=2.0, **can_kwargs):
    """只听不发：统计总线上的 (source, msgtype) 帧数。

    返回 {source: {msgtype: count}}。不发帧，所以不会把设备打进 bus-off。
    """
    kwargs = dict(can_kwargs)
    link = CanLink(**kwargs)
    try:
        counts = link.sniff(seconds)
    finally:
        link.close()
    by_source = {}
    for (src, msgtype), n in counts.items():
        by_source.setdefault(src, {})[msgtype] = n
    return by_source


def print_scan(by_source, node_id=None):
    if not by_source:
        print("  一帧都没有。依次检查：")
        print("    1) 接线（CANH/CANL）与两端 120 Ω 终端电阻；")
        print("    2) 速率：Classic 下 --bitrate 必须等于 config.can_baud；"
              "FD 下仲裁域固定 1 Mbit/s（--bitrate 1000000 + --fd），"
              "--data-bitrate 填 config.can_baud；")
        print("    3) 协议：config.can_prot 必须为 1（CYBERBEAST）；"
              "旧 11-bit 协议没有 endpoint 访问。")
        return False
    for src in sorted(by_source):
        total = sum(by_source[src].values())
        mark = "  ←--can-id" if node_id is not None and src == node_id else ""
        detail = "  ".join("%s(0x%02X)×%d" % (CAN_MSG_NAMES.get(mt, "?"), mt, n)
                           for mt, n in sorted(by_source[src].items()))
        print("  src=%-3d 共 %5d 帧%s\n      %s" % (src, total, mark, detail))
    if node_id is not None and node_id not in by_source:
        print("  ⚠ 没有来自节点 %d 的帧：如果有别的 src 在发心跳，"
              "用 --can-id 指定它；" % node_id)
        print("    也可能是另一个主站在抢应答（同一总线只允许一个主站）。")
    return True


def probe(device, seconds=2.0):
    """先只听（看总线上谁在发帧），再试读描述符。返回是否成功。"""
    link = device.link
    if not getattr(link, "is_can", False):
        raise EpError("probe 只在 --can 下可用")
    print("监听 %.1f s（%s；节点 --can-id=%d，本机 --master-id=0x%02X）"
          % (seconds, link.describe(), link.node_id, link.master_id))
    counts = link.sniff(seconds)
    by_source = {}
    for (src, msgtype), n in counts.items():
        by_source.setdefault(src, {})[msgtype] = n
    if not print_scan(by_source, link.node_id):
        print("  ⚠ 如果刚才用错过波特率：设备的帧没人应答会让 TEC 涨到 ~248，"
              "固件不会自动恢复，")
        print("    CAN 会一直\"死\"到你用 USB 重启它。用另一个终端确认/恢复：")
        print("      rmc --port COMx get telemetry.can_tec   # >128 就是中招了")
        print("      rmc --port COMx call action.reboot")
        return False
    try:
        desc = device.fetch_descriptor()
    except EpError as e:
        print("  描述符请求失败：%s" % e)
        return False
    print("  描述符 OK：%d 个 endpoint，CRC16=0x%04X" % (len(desc), device.json_crc))
    return True


def monitor(device, names, rate=10.0, count=None, out=None):
    """周期读取若干参数；Ctrl-C 退出。names 可以是字符串或代理节点。"""
    from .proxy import resolve

    out = out or sys.stdout
    names = [resolve(device, n) for n in names]
    if not device.descriptor:
        device.fetch_descriptor()
    period = 1.0 / rate if rate > 0 else 0.0
    rows = 0
    try:
        while count is None or rows < count:
            start = time.time()
            row = ["%-24s %s" % (name, format_value(device.read(name))) for name in names]
            print("  ".join(row), file=out, flush=True)
            rows += 1
            if count is not None and rows >= count:
                break
            remaining = period - (time.time() - start)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    return rows


watch = monitor
