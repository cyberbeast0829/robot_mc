# robot_mc

三相 FOC 电机控制器固件，移植自 Ben Katz 的
[mini-cheetah motorcontrol](https://github.com/bgkatz/motorcontrol)，运行在
**Nations N32H473CE**（Cortex-M4F）上，裸机、无 RTOS。

- 控制环 40 kHz，`ATIM1` 触发三路注入 ADC，编码器走 SPI1 + DMA2。
- 参数访问：**fibre 风格 endpoint**，USB 复合设备上有两条传输——CDC 串口
  （ASCII 菜单 + 带 CRC 的帧）和 vendor interface（一个 USB 包一帧）；设备自带
  JSON 描述符，上位机按名字读写，固件升级不用改脚本。
- 配置持久化：ID-keyed TLV，双页 ping-pong + CRC32；**OTA 升级只擦 app 区时，
  配置和标定都会保留**。

## 构建

```bash
make              # build/mit_h473.elf/.hex/.bin
make DEBUG=1      # -Og -g3
make size         # 段大小
make check        # 生成的参数表/文档是否最新（提交前跑）
make hosttest     # 主机单测（prefs_codec，不需要板子）
make flash        # J-Link；可覆盖 JLINK_DEVICE/JLINK_IF/JLINK_SPEED
make clean
```

需要 `arm-none-eabi-gcc` 在 PATH（`GCC_PATH=` 可覆盖）。Windows 上建议用
MSYS2 bash：Chocolatey 的 make 用 cmd，`mkdir -p` 会失败。

也支持 **Keil MDK（AC6）**：`Keil/mit_h473.uvprojx`，scatter `Keil/ODrive_N32.sct`。

> ⚠ `adc.c`/`tim.c` 有 `USER CODE` 块之外的手工修改，CubeMX 重新生成会静默丢掉它们。
> 上游 STM32F446 时代的 `motorcontrol.ioc`、`.cproject`/`.project`/`.mxproject`、
> `STM32F446RETX_*.ld` 与 `stm32f4xx_hal_conf.h` 已从本仓库删除，不要从上游合并回来。

## 文档

- [docs/README.md](docs/README.md)：入口、版本矩阵、五分钟上手
- [docs/USER.md](docs/USER.md)：用户手册（参数读写、标定、备份、OTA、故障排查）
- [docs/DEV.md](docs/DEV.md)：开发者手册（架构、协议、codegen、持久化、Flash/OTA、CAN）
- [docs/parameters.md](docs/parameters.md)：参数手册（自动生成）
- [AGENTS.md](AGENTS.md)：给 AI 的短契约

## 上位机

驱动板默认**只有 CAN 接口**（没有 USB 座），所以上位机走 CAN。
上位机是 PyPI 包 **`robotmc`**（命令 `rmc`，源码 `tools/rmc/`；
`tools/mc_ep.py` 是转发到它的弃用 shim）：

```bash
pip install robotmc                     # 已含 CAN、USB/CDC、IPython shell 与彩色终端依赖
rmc tree                                # CANable/slcan，自动找适配器
rmc get config.i_max
rmc set config.i_max 3.5
rmc call action.save
rmc get system.save_status               # 1 = 保存成功
rmc probe                                # 链路诊断：只监听总线
rmc                                      # 交互 shell（自动发现设备，m0.<TAB> 补全）

# 其它 CAN 后端（python-can 的名字）
rmc --interface pcan get config.i_max              # PCAN-USB
rmc --interface socketcan --channel can0 tree      # Linux
rmc --fd --data-bitrate 5000000 tree               # CAN FD 5M
```

驱动板还支持 USB 复合设备，需要手工连接 USB 焊点，参数名和输出完全一样：

```bash
rmc --port COM7 tree   # CDC 虚拟串口
rmc --usb tree         # vendor interface（还需 pip install pyusb libusb-package）
```

## 许可证

本项目采用 **MIT License**（见 [LICENSE](LICENSE)）。原始版权归 Ben Katz（mini-cheetah
`motorcontrol`，2020）；本仓库的 N32H473CE 移植与后续修改版权归 CyberBeast（北京守护兽科技有限公司）。

MIT 条款只要求一件事：分发本软件或其派生作品时，**必须保留原始版权声明和许可全文**。
因此 `LICENSE` 中同时保留原作者与本公司的版权行；在此前提下可以自由修改、再许可
（sub-license）或闭源分发。

以下第三方组件沿用其各自的原始许可，**不要**改动这些文件的版权头：

| 组件 | 位置 | 许可 |
| --- | --- | --- |
| Nations N32H47x 标准外设库 / USBFSD | `Drivers/`、`Core/Src/usb*.c` | Nations Technologies Inc. 版权声明（见各文件头） |
| STM32CubeMX 生成的外设初始化 | `Core/Src/{adc,tim,usart,gpio,spi}.c` 等 | BSD 3-Clause（STMicroelectronics） |
| CMSIS | `Drivers/CMSIS/` | Apache-2.0（ARM） |
| 移植自 ODrive 的算法（`motion_law`、`dual_gear`、`cogging_law`、标定、梯形规划等） | `Core/Inc`、`Core/Src` | MIT（ODrive Robotics） |
