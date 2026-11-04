# motorcontrol（N32H473CE 移植）

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

> ⚠ `motorcontrol.ioc` 是上游 STM32F446 的 CubeMX 文件，**不要重新生成**；
> `adc.c`/`tim.c` 有 `USER CODE` 块之外的手工修改。

## 文档

- [docs/README.md](docs/README.md)：入口、版本矩阵、五分钟上手
- [docs/USER.md](docs/USER.md)：用户手册（参数读写、标定、备份、OTA、故障排查）
- [docs/DEV.md](docs/DEV.md)：开发者手册（架构、协议、codegen、持久化、Flash/OTA、CAN）
- [docs/parameters.md](docs/parameters.md)：参数手册（自动生成）
- [AGENTS.md](AGENTS.md)：给 AI 的短契约

## 上位机

```bash
pip install pyserial
python tools/mc_ep.py --port COM7 tree
python tools/mc_ep.py --port COM7 get config.i_max
python tools/mc_ep.py --port COM7 set config.i_max 3.5
python tools/mc_ep.py --port COM7 call action.save
```
