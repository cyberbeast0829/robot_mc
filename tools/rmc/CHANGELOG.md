# 变更记录

## 0.1.0

首个版本，替代 `tools/mc_ep.py`：

- CAN（CYBERBEAST）优先：slcan/CANable（含 FD）、PCAN、SocketCAN、gs_usb 等
  python-can 后端；保留 `--port`（CDC）与 `--usb`（vendor interface）给开发/台架板。
- `rmc` 交互 shell（IPython）：`dev.config.i_max` 读、赋值写、`dev.action.save()` 执行，
  `dev.<TAB>` 补全。
- 子命令：`tree/dump/descriptor/get/set/call/monitor/probe/scan/backup/restore/
  liveplotter`。
- 库入口：`from rmc import connect`，属性树、`backup()/restore()`、`info()`。
- 与 `tools/mc_ep.py` 的 Device/常量/`EpError` 接口兼容（`mc_ep.py` 已成为转发 shim）。
- 不接硬件的跨进程端到端测试：`tests/sim_device.py`（python-can `udp_multicast`，
  需要 `msgpack`），可用 `rmc --interface udp_multicast --channel 239.74.163.10:28700`
  完整验证 CLI / shell / backup / shim。
