#!/usr/bin/env python3
"""已弃用：请改用 `rmc`（pip install robotmc）。

这个文件只是转发 shim，保留是为了让老脚本、文档里的
`python tools/mc_ep.py ...` 和 `make hosttest-py` 继续工作：

    # 旧写法
    python tools/mc_ep.py --can get config.i_max
    # 新写法（接口/参数完全一样）
    rmc --can get config.i_max

同时重新导出 rmc 的常量/类（Device、EpError、TYPE_CODES、cb_id、
DEFAULT_* …），因为 tools/*.py 里有脚本 `from mc_ep import Device`。
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_RMC_ROOT = os.path.join(_HERE, "rmc")
# 注意顺序：tools/ 通常也在 sys.path 里，而 tools/rmc 本身是个目录名，
# 会先被当成 namespace package 抢走 `import rmc`。先把真正的包目录插到最前面。
if os.path.isdir(os.path.join(_RMC_ROOT, "rmc")) and _RMC_ROOT not in sys.path:
    sys.path.insert(0, _RMC_ROOT)

from rmc import cli
from rmc.device import Device
from rmc.errors import EpError, NoDescriptorReply, TransportError
from rmc.protocol import *          # noqa: F401,F403
from rmc.protocol import struct     # noqa: F401  (历史上这里能拿到 struct)
from rmc.transport.can import CanLink, PyCanBus, SlcanBus  # noqa: F401
from rmc.transport.fibre import FibreLink, SerialTransport, UsbTransport  # noqa: F401


def main(argv=None):
    print("warning: tools/mc_ep.py 已弃用，请改用 `rmc`（pip install robotmc）。",
          file=sys.stderr)
    return cli.main(argv)


if __name__ == "__main__":
    sys.exit(main())
