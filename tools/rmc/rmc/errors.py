"""rmc 的异常类型。"""


class EpError(Exception):
    """endpoint / 传输层错误。

    status 是 USB 那两条传输的响应状态码（CAN 没有状态字节，恒为 None）。
    """

    def __init__(self, message, status=None):
        super().__init__(message)
        self.status = status


class NoDescriptorReply(EpError):
    """设备对描述符请求一帧都没回（重试没有意义）。"""


class TransportError(EpError):
    """打不开/读写传输层。"""
