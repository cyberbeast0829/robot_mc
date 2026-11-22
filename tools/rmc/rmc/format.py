"""输出格式化。"""


def format_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if value != value:
            return "nan"
        if value == float("inf"):
            return "inf"
        if value == float("-inf"):
            return "-inf"
        return "%.6g" % value
    return str(value)


def format_diff(requested, actual):
    """`rmc set` 的回显：写进去的和读回的不一样时标注出来。"""
    if actual is None:
        return ""
    if requested == actual or isinstance(actual, float) and isinstance(requested, (int, float)) \
            and abs(float(actual) - float(requested)) <= max(1e-9, 1e-6 * abs(float(actual))):
        return ""
    return "  (请求 %s；设备限幅或拒绝)" % format_value(requested)


def format_range(ep):
    lo = ep.get("min")
    hi = ep.get("max")
    if lo is None and hi is None:
        return ""
    if lo is None:
        return "范围 <= %s" % hi
    if hi is None:
        return "范围 >= %s" % lo
    return "范围 %s..%s" % (lo, hi)


def format_endpoint(ep):
    extra = []
    if ep.get("unit"):
        extra.append(ep["unit"])
    rng = format_range(ep)
    if rng:
        extra.append(rng)
    if ep.get("persist") and "persist" not in extra:
        extra.append("persist")
    return "%-36s %-8s %-3s  %s" % (ep["name"], ep["type"], ep["access"],
                                    " ".join(extra))


def print_restore(results):
    """打印 restore() 的逐条结果，返回真正需要变更的记录。"""
    changed = [r for r in results if r["status"] not in ("相同", "跳过")]
    for r in results:
        before = "-" if r["before"] is None else format_value(r["before"])
        after = "-" if r["after"] is None else format_value(r["after"])
        line = "  %-36s %-10s %s -> %s" % (r["name"], r["status"], before, after)
        if r["error"]:
            line += "  (%s)" % r["error"]
        print(line)
    print("共 %d 项，需变更 %d 项。" % (len(results), len(changed)))
    return changed


def print_tree(device):
    """按组打印参数树（和 mc_ep.py tree 的输出一致）。"""
    last_group = None
    for ep in device.endpoints:
        group = ep["name"].split(".")[0]
        if group != last_group:
            print("\n[%s]" % group)
            last_group = group
        print("  " + format_endpoint(ep))
