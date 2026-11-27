# rmc — robot_mc

CyberBeast 电机控制器（N32 motorcontrol 固件）的**上位机工具 + Python 库**，
功能对标 `odrivetool`，但**以 CAN 为主**（CANable/slcan、PCAN、SocketCAN 等），
并完整替代仓库里的 `tools/mc_ep.py`。

- PyPI 分发名：**`robotmc`**（`rmc` 这个名字在 PyPI 已被别的项目占用）
- import 名：`rmc`
- 命令：**`rmc`**

> ⚠ `pip install rmc` 会装到别人的包，请始终使用 `pip install robotmc`。

## 安装

```bash
pip install robotmc                 # 基础功能（CAN + shell + 补全）
pip install "robotmc[plot]"         # 另外装 matplotlib（liveplotter）
```

Windows / Linux 都支持。CAN 适配器按后端另需：

| 后端 | 需要 |
|---|---|
| `slcan`（CANable / CandleLight，默认） | 仅 `pyserial`（已在依赖里） |
| `pcan`（PCAN-USB） | 安装 PEAK 的 PCAN-Basic 驱动 |
| `socketcan`（Linux） | 先 `ip link set can0 up type can bitrate 1000000` |
| `cansub`（CSS Electronics CANsub，USB / 以太网） | `pip install python-can-cansub`，无需驱动；`--channel aabbccdd-usb.local@1` |
| `gs_usb` / `kvaser` / `vector` / `ixxat` … | 对应 python-can 后端与厂商驱动 |

## 五分钟上手

```bash
rmc --interface slcan --channel COM3 tree          # Windows：CANable 在 COM3
rmc --interface pcan --channel PCAN_USBBUS1 tree   # PCAN（通道名可省略，默认它）
rmc --interface socketcan --channel can0 tree      # Linux SocketCAN（通道默认 can0）

rmc get config.i_max
rmc set config.i_max 3.5
rmc call action.save
rmc monitor telemetry.i_q telemetry.position --rate 100
```

CAN 出厂参数：**CYBERBEAST 协议、Classic 1 Mbit/s、`config.can_id = 1`**，所以默认值
`--interface slcan --bitrate 1000000 --can-id 1` 对出货板就能用。

CAN FD（`config.can_baud > 1M`）：

```bash
rmc --fd --data-bitrate 5000000 --interface slcan --channel COM3 tree
```

FD 下仲裁域由固件固定在 1 Mbit/s（`--bitrate` 保持 1000000），`--data-bitrate`
填 `config.can_baud`（2M/4M/5M）。5M 需要固件 `config.can_tdc` 自动补偿（默认开）。

## 交互 shell（odrivetool 风格，多设备）

```bash
$ rmc
rmc 0.1.1 — robot_mc endpoint tool (protocol 2)
✓ 已连接 m0 | serial=0x0123456789ABCDEF | fw=1.0.6.2 | hw=4.2.89 | slcan COM3，1000000 bit/s（Classic） | 261 endpoints | CRC 0x5A31
m0.<TAB> 补全；输入 help() 查看用法，exit 退出。

>>> m0.config.i_max
12
>>> m0.config.i_max = 3.5
>>> m0.action.save()
>>> m0.telemetry.i_q
-0.1832
>>> connect(node_id=2)             # 同一 CAN 通道上的另一台设备
✓ 已连接 m1 | serial=0xFEDCBA9876543210 | fw=1.0.6.2 | hw=4.2.89 | ...
>>> m1.config.i_max
12
>>> watch("m0.telemetry.position", rate=20)
>>> scan()                      # 只听总线，列出节点和消息
>>> devices()                    # 查看所有已连接设备
>>> disconnect("m0")            # 断开后重连仍恢复 m0 名称
>>> plot("m1.telemetry.position", window=10)
```

- shell 启动时被动发现配置 CAN 通道上的活动节点，并扫描匹配 VID/PID 的 CDC、
  vendor USB；运行中持续监测断连并发现新连接。连接信息包含 `mN`、固件
  `system.serial_number`（固定 16 位十六进制）、固件版本
  `fw_version_major.minor.revision.unreleased` 和硬件版本
  `hw_version_major.minor.variant`。也可在 shell 中反复 `connect(...)` 加入设备。
- 连接状态约每秒检查一次，连续两次探测失败才报告断开，以避免瞬时丢帧误判。
  CAN 自动发现依赖节点发送活动帧；静默节点可用 `connect(node_id=...)` 手动连接。
- `mN` 按本次 shell 会话中首次连接顺序分配；同一序列号断开后重连会复用原名称。
  若固件没有序列号端点或读取失败，会以连接路径作为临时身份并显示 `unknown`。
- 一台电机同时通过 CAN 和 USB 连接时，按序列号合并为同一设备；连接重复项不会
  创建第二个 `mN`。CAN 多节点共享同一个 CAN 通道/适配器，不会为每个节点重复打开
  slcan 串口。shell 是串行访问，CAN 总线上仍必须只有一个主站发命令。
- 读：表达式（`m0.config.i_max`）；写：赋值（`m0.config.i_max = 3.5`，写后自动读回）；
  动作：`m0.action.xxx()`。
- **tab 补全**由 `__dir__` 提供，IPython 直接识别 `m0.config.`、`m1.action.` 等路径。
- 列出所有入口：`help()`。没有设备也能进 shell：先 `scan()` 诊断，再 `connect(...)`。

## 子命令

| 命令 | 说明 |
|---|---|
| `rmc` / `rmc shell` | 交互 shell（默认） |
| `rmc tree` | 设备自带的参数树（含类型/权限/单位/范围） |
| `rmc dump [--persist] [--json F]` | 读所有可读参数（`function` 端点会跳过——读=执行） |
| `rmc descriptor [--raw]` | 打印原始 JSON 描述符 |
| `rmc get NAME...` | 读参数 |
| `rmc set NAME VALUE [--no-verify]` | 写参数，打印读回值；被限幅时在 stderr 提示 |
| `rmc call NAME... [--read]` | 执行 `action.*`（`--read` 用 PARAM_READ 触发，ODrive 语义） |
| `rmc monitor NAME... [--rate HZ] [--count N]` | 周期读取 |
| `rmc scan [--seconds S]` | **只监听不发帧**，列出总线上的节点与消息（不会把设备打进 bus-off） |
| `rmc probe [--seconds S]` | 监听 + 读描述符的链路诊断 |
| `rmc backup FILE` | 把描述符里 `persist: true` 的参数导出 JSON |
| `rmc restore FILE [--yes] [--save]` | 回写（默认 dry-run 显示 diff；`--yes` 写；`--save` 再写 flash） |
| `rmc liveplotter NAME... [--window S] [--rate HZ]` | 实时曲线（需 `robotmc[plot]`） |
| `rmc version` / `--version` | 版本 |

连接选项（可写在子命令前或后）：

```
--interface slcan|pcan|socketcan|gs_usb|kvaser|vector|...
--channel COM3 / PCAN_USBBUS1 / can0            （slcan/pcan/socketcan 不给时自动猜）
--bitrate 1000000   --fd --data-bitrate 5000000
--can-id 1          --master-id 0xFE            --timeout 1.0
--port COM7（CDC）  --usb（vendor interface）    --vid/--pid/--usb-interface
--can（兼容 mc_ep.py 的写法，可省略）
```

## Python 库

```python
from rmc import connect

with connect(interface="slcan", channel="COM3") as dev:
    print(dev.info())                    # 传输/固件/描述符 CRC
    print(dev.config.i_max)              # 属性读（每次访问都走总线）
    dev.config.i_max = 3.5               # 属性写（写后读回）
    dev.action.save()                    # 动作
    print(dev["telemetry.position"])     # 等价字典访问
    print(dev.read("telemetry.i_q"))
    dev.write("config.i_max", 4.0)

    backup = dev.backup()                # persist 参数 dict
    dev.restore(backup, dry_run=False)   # 回写，返回逐条结果
```

- 代理树只构造不读总线；只有取值/显示时才读。**动作端点的 `repr` 不会执行它**。
- 写操作：客户端先拒绝 NaN/Inf；真实范围由固件在 `endpoint_handler()` 里限幅，
  可读端点写后自动读回，因此 `set` 能看出“被限幅/被拒绝”。
- `command.*`（`t_ff/p_des/...`）写入即刷新固件命令租约（与 CAN 帧语义一致）。

## 排障

- **一帧都没有**：先 `rmc probe`（只监听）。检查 CANH/CANL、120 Ω 终端、
  速率是否等于 `config.can_baud`、`config.can_prot` 是否为 1（CYBERBEAST）。
- **用错波特率之后再也连不上**：设备 TX 队列卡在一帧等 ACK，`can_tec` 会到 ~248
  且不会自愈。用 USB 读 `telemetry.can_tec`（>128 即中招）后
  `rmc --port COMx call action.reboot`。
- **刷了固件后报 “接口版本不匹配”**：CDC/USB 会返回 `EP_ERR_VERSION`，`rmc`
  会自动重下描述符；自定义长连接必须重连。
- **5M FD 不通**：确认 `config.can_tdc = -1`（自动）并重启（TDC 只能 init 时写）。
- **同一总线只能有一个主站**：固件从收到的帧里学习 master id，两个上位机同时发
  命令会把应答抢走。

## 与 `tools/mc_ep.py` 的关系

`rmc` 覆盖了 `mc_ep.py` 的全部功能（同样的参数名、同样的 CAN/USB 传输），
并且命令行写法兼容：`python tools/mc_ep.py --can tree` → `rmc --can tree`。
仓库里的 `tools/mc_ep.py` 现在是**转发到 rmc 的弃用 shim**（打印 warning），
老脚本不用改。

## 本地测试（不接硬件）

三层，按需要选：

### 1) 单元测试（python-can virtual 总线，同进程假设备）

```bash
cd tools/rmc
pip install pytest python-can
python -m pytest -q            # 或：仓库根 make hosttest-rmc
```

覆盖：描述符流/CRC、Classic 分段读写、FD 64 字节分片、写后读回、动作端点、
属性树、tab 补全（rlcompleter + IPython）、backup/restore、CLI 接线。

### 2) 跨进程端到端（推荐：测 CLI / shell / mc_ep shim）

`virtual` 总线只在进程内可见，所以这里用 python-can 的 **udp_multicast**
（需要 `pip install msgpack`）：

```bash
pip install python-can msgpack

# 终端 1：模拟设备（node=1，参数表见 tests/fake_device.py；Ctrl-C 退出）
python tools/rmc/tests/sim_device.py

# 终端 2（注意：未安装时要在 tools/rmc 目录下执行 python -m rmc）
cd tools/rmc
C="--interface udp_multicast --channel 239.74.163.10:28700"   # bash
python -m rmc $C tree
python -m rmc $C get config.f
python -m rmc $C set config.f 2.5
python -m rmc $C set config.f 99        # 设备限幅 -> 读回 5，stderr 提示
python -m rmc $C call action.save
python -m rmc $C monitor config.f config.i8 --count 5 --rate 20
python -m rmc $C backup /tmp/cfg.json
python -m rmc $C restore /tmp/cfg.json              # dry-run
python -m rmc $C restore /tmp/cfg.json --yes
python -m rmc $C                                    # 交互 shell，m0.<TAB>
```

Windows PowerShell 里把 `$C` 换成数组：`$C = @("--interface","udp_multicast",
"--channel","239.74.163.10:28700")`，调用写 `python -m rmc @C tree`。

shell 里可以验证补全与代理：

```python
m0.config.f            # 读
m0.config.f = 1.5      # 写
m0.action.save()
watch("m0.config.f", count=3, rate=2)
backup()                # 返回 dict
help()
```

验证旧入口（仓库根执行）：

```bash
python tools/mc_ep.py --can --interface udp_multicast \
       --channel 239.74.163.10:28700 tree
```

### 3) 装成 wheel 再测（最接近用户）

```bash
python -m build
python -m venv .venv-rmc
.venv-rmc\Scripts\activate            # Linux: source .venv-rmc/bin/activate
pip install dist/robotmc-0.1.1-py3-none-any.whl
rmc --version
```

（改代码时用 `pip install -e .` 更方便，`rmc` 命令会立刻反映改动。）

### 4) 连真机

先 `rmc scan`（**只听不发帧**，不会把设备打进 bus-off），再 `rmc tree` /
`get` / `set` / `call` / `monitor`；shell 里试 `m0.<TAB>`。
CANable 不给 `--channel` 时会自动按 VID:PID 找串口。

## 发布

```bash
cd tools/rmc
python -m build               # 生成 dist/
python -m twine check dist/*
python -m twine upload --repository testpypi dist/*
python -m twine upload dist/*
```

协议细节见 `Core/Inc/endpoint.h`、`Core/Src/can_cyberbeast.c` 与仓库
`docs/DEV.md` 的 “endpoint 协议 / CAN：协议选择与 CYBERBEAST” 两节。
