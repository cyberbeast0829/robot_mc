"""参数树代理：`dev.config.i_max` 读、`dev.config.i_max = 3.5` 写、
`dev.action.save()` 执行，tab 补全靠 `__dir__`。

只构造代理不会访问总线（组和叶子都是懒的）；只有显示/取值时才真正读。
动作端点的 `repr` 绝不读——对动作端点而言“读”就是“执行”。
"""

import difflib

from .protocol import is_action


def _suggest(name, candidates):
    close = difflib.get_close_matches(name, sorted(candidates), n=3, cutoff=0.6)
    if not close:
        return ""
    return "，你是不是要: %s" % ", ".join(close)


class Node:
    def __init__(self, device, path, name):
        object.__setattr__(self, "_device", device)
        object.__setattr__(self, "path", path)
        object.__setattr__(self, "name", name)

    def __dir__(self):
        return []


class Group(Node):
    """一层参数组（config / telemetry / action …）。"""

    def __init__(self, device, path):
        Node.__init__(self, device, path, path.split(".")[-1] if path else "")
        object.__setattr__(self, "_children", {})

    def _add(self, node):
        self._children[node.name] = node

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        children = object.__getattribute__(self, "_children")
        try:
            return children[name]
        except KeyError:
            raise AttributeError("没有 %s%s" % (self.path + "." + name if self.path else name,
                                              _suggest(name, children)))

    def __getitem__(self, name):
        return getattr(self, name)

    def __setattr__(self, name, value):
        if name.startswith("_"):
            object.__setattr__(self, name, value)
            return
        children = object.__getattribute__(self, "_children")
        node = children.get(name)
        if node is None:
            raise AttributeError("没有 %s%s" % (self.path + "." + name if self.path else name,
                                              _suggest(name, children)))
        if isinstance(node, Group):
            raise AttributeError("%s 是参数组，不能整体赋值" % node.path)
        if isinstance(node, Action):
            raise AttributeError("%s 是动作端点，请用 %s()" % (node.path, node.path))
        node.write(value)

    def __dir__(self):
        return list(self._children)

    def __repr__(self):
        return "<group %s (%d 项)>" % (self.path or "<root>", len(self._children))

    def __call__(self):
        raise TypeError("%s 是参数组；动作请用 %s()" % (self.path, self.path))


class Scalar(Node):
    """一个标量参数。repr/float/int/bool 都会读一次。"""

    def __init__(self, device, ep):
        Node.__init__(self, device, ep["name"], ep["name"].split(".")[-1])
        object.__setattr__(self, "_ep", ep)

    @property
    def endpoint(self):
        return self._ep

    def read(self):
        return self._device.read(self.path)

    def write(self, value):
        return self._device.write(self.path, value)

    def __repr__(self):
        from .format import format_value
        try:
            return format_value(self.read())
        except Exception as e:      # 补全/交互里不要把异常变成 traceback
            return "<读取失败 %s: %s>" % (self.path, e)

    __str__ = __repr__

    def __float__(self):
        return float(self.read())

    def __int__(self):
        return int(self.read())

    def __bool__(self):
        return bool(self.read())

    # 交互式比较：dev.config.i_max > 3 / dev.telemetry.i_q == 0
    def __eq__(self, other):
        return self.read() == (other.read() if isinstance(other, Scalar) else other)

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        return self.read() < (other.read() if isinstance(other, Scalar) else other)

    def __le__(self, other):
        return self.read() <= (other.read() if isinstance(other, Scalar) else other)

    def __gt__(self, other):
        return self.read() > (other.read() if isinstance(other, Scalar) else other)

    def __ge__(self, other):
        return self.read() >= (other.read() if isinstance(other, Scalar) else other)


class Action(Node):
    """动作端点（线上 `"type":"function"`）：`dev.action.save()`。"""

    def __init__(self, device, ep):
        Node.__init__(self, device, ep["name"], ep["name"].split(".")[-1])
        object.__setattr__(self, "_ep", ep)

    @property
    def endpoint(self):
        return self._ep

    def __call__(self, *args, **kwargs):
        if args or kwargs:
            raise TypeError("%s 不接受参数" % self.path)
        self._device.call(self.path)

    def __repr__(self):
        return "<action %s (ID %d)>" % (self.path, self._ep["id"])

    __str__ = __repr__


def build_tree(device):
    """把扁平的描述符（"config.i_max"）搭成属性树。"""
    root = Group(device, "")
    for ep in device.endpoints:
        parts = ep["name"].split(".")
        group = root
        for index in range(len(parts) - 1):
            path = ".".join(parts[:index + 1])
            child = group._children.get(parts[index])
            if child is None:
                child = Group(device, path)
                group._add(child)
            group = child
        leaf = Action(device, ep) if is_action(ep) else Scalar(device, ep)
        group._add(leaf)
    return root


def resolve(device, name):
    """把 "dev.config.i_max" / "config.i_max" / 代理节点 统一成 endpoint 名字。"""
    if isinstance(name, Node):
        return name.path
    text = str(name)
    if text.startswith("dev."):
        text = text[4:]
    return text
