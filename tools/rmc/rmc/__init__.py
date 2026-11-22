"""rmc — robot_mc endpoint 工具与 Python 库（CYBERBEAST CAN 优先）。

库用法：
    from rmc import connect
    with connect(channel="COM3") as dev:
        print(dev.config.i_max)
        dev.config.i_max = 3.5
        dev.action.save()

命令行：`rmc --help`
"""

from .device import Device, connect
from .errors import EpError, NoDescriptorReply, TransportError
from .version import __version__

__all__ = ["connect", "Device", "EpError", "NoDescriptorReply", "TransportError",
           "__version__"]
