# 文档

N32H473CE 三相 FOC 固件。只有三份要读的文档，按"你是谁"选一份即可（本页是索引）：

| 文档 | 给谁 | 内容 |
|---|---|---|
| [USER.md](USER.md) | 使用者 / 上位机开发者 | 连接、读写参数、保存、标定、备份、OTA、故障排查、FAQ |
| [DEV.md](DEV.md) | 改固件的人 | 架构、endpoint 协议、`params.json` 与代码生成、配置持久化、Flash/OTA 契约、CAN 移植、测试与发布 |
| [parameters.md](parameters.md) | 所有人 | 参数手册（由 `params.json` 自动生成，勿手改） |

构建命令、仓库结构见根目录 [README.md](../README.md)；给 AI 的短契约见
[AGENTS.md](../AGENTS.md)。刷机工具的使用说明在
[`tools/blcan_使用说明.md`](../tools/blcan_使用说明.md)（放在 `tools/` 里，不占用
`docs/` 的三份文档）。

## 版本

| 项目 | 当前值 | 设备上怎么读 |
|---|---|---|
| 固件 | `1.0.7`（`UNRELEASED=1`，MTPA 首个测试版，非正式发布） | `system.fw_version_major/minor/revision` + `system.fw_version_unreleased` |
| 硬件 | `1.0.0`（CBL89） | `system.hw_version_major/minor/variant` |
| 序列号 | 每个芯片不同 | `system.serial_number`（64 位，MCU UID） |
| endpoint 协议 | `2` | endpoint 0 的版本探测（偏移 `0xFFFFFFFF`） |
| 配置记录 schema | `2` | `system.config_schema` |
| 配置块序号 | 每次保存 +1 | `system.prefs_sequence` |

描述符的 CRC 就是接口版本，每次增删参数都会变（具体值用 endpoint 0 探测，或
`rmc descriptor` 看）。CRC 变了以后旧上位机的请求会被 `EP_ERR_VERSION`
拒绝，不会出现"名字对上了、类型对不上"的静默错误。

## 五分钟上手

出货版本默认只有 CAN 接口（没有 USB 座）。装上位机包 **`robotmc`**（命令 `rmc`；
`pip install rmc` 是另一个项目，别装错），用 CANable 之类的适配器走 CAN：

```bash
pip install robotmc
rmc tree                            # 参数树（CANable/slcan，自动找适配器）
rmc get config.i_max
rmc set config.i_max 3.5
rmc call action.save
rmc get system.save_status           # 1 = 保存成功
rmc probe                            # 一帧都看不到时先跑这个
rmc                                  # 交互 shell（自动发现设备，m0.<TAB> 补全）

# 其它后端：pcan / socketcan / gs_usb …（python-can 的名字）
rmc --interface pcan get config.i_max
rmc --interface socketcan --channel can0 tree
```

开发/台架板子上还有 USB 复合设备，命令和参数名完全一样：
`--port COM7`（CDC 虚拟串口）或 `--usb`（vendor interface，需要 pyusb；
Windows 用 Zadig 给 interface 2 装 WinUSB）。

改完参数要 `action.save` 才写 flash；门极使能时保存会被拒绝（先
`call action.idle_mode`）。细节见 [USER.md](USER.md) 第 1~2 节。

## 升级固件（CAN OTA）

板子上跑的是"OpenBootloader（`0x08000000`，32 KB）+ App（`0x08008000`）"。升级用本仓库
自带的 CAN 工具 [`tools/blcan.py`](../tools/blcan.py)（`pip install python-can pyserial`，
CANable/PCAN 都行；bootloader 固定 500 kbit/s Classic，和 App 的 CAN 配置无关）：

```bash
rmc call action.enter_bootloader_mode    # 带 USB 的板子也可按串口菜单 b
python tools/blcan.py --channel COM3 sync
python tools/blcan.py --channel COM3 upgrade --firmware build/mit_h473.bin \
        --erase auto --addr 0x08008000 --chunk-size 256 --verify --go-addr 0x08008000
```

只要不用 `--erase mass`，配置、零点、标定 LUT 全部保留。工具用法/诊断命令/变砖恢复见
[`tools/blcan_使用说明.md`](../tools/blcan_使用说明.md)，Flash 分区与升级契约见
[DEV.md](DEV.md)，面向使用者的步骤见 [USER.md](USER.md) 第 7 节。

## 一页纸数据流

```
上位机 ──CAN (CYBERBEAST)─> can_cyberbeast.c ─> endpoint_handler            [出货默认]
上位机 ──USB CDC──────> endpoint_serial.c ┐
上位机 ──USB vendor i/f─> endpoint_usb.c  ┴─> endpoint_process() → endpoint_handler [生成]
                                              └─ 直接读写 params.json 里的变量
主循环 ──> run_fsm_slow() ──> save_preferences() ──> TLV + CRC32 ──> 双页 ping-pong
控制 ISR (40 kHz) ──> 采样 → 故障检查 → 状态机 → CAN
```

参数表、描述符、持久化表、参数手册全部由根目录 `params.json` 生成：

```bash
make endpoints   # 重新生成
make check       # 校验生成物是否最新（提交前跑）
```
