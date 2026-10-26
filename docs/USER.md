# 用户手册

面向使用板子/写上位机的人。参数清单见 [parameters.md](parameters.md)，
协议内部细节见 [DEV.md](DEV.md)。

## 1. 连接

- 板子 USB 接电脑（USB CDC 虚拟串口）。Windows 记下 COM 号（如 `COM7`），
  Linux 是 `/dev/ttyACM0`。
- `pip install pyserial`
- 工具：`tools/mc_ep.py`

```bash
python tools/mc_ep.py --port COM7 tree
```

设备会先回送一份 JSON 描述符（名字/ID/类型/权限），工具据此把名字映射到 ID，
所以**固件升级后脚本不用改**。`--baud` 对 CDC 无实际作用。

> 同一个串口只能有一个使用者：`mc_ep.py` 运行时不要再开串口终端。原来的
> ASCII 菜单仍然可用（115200 8N1：`m` 电机、`c` 标定、`s` 设置、`e` 编码器、
> `z` 零点、`d` DFU）。

## 2. 读写参数

```bash
python tools/mc_ep.py --port COM7 tree                 # 参数树
python tools/mc_ep.py --port COM7 descriptor           # 原始 JSON 描述符
python tools/mc_ep.py --port COM7 get  <name>          # 读
python tools/mc_ep.py --port COM7 set  <name> <value>  # 写（打印实际值）
python tools/mc_ep.py --port COM7 call <action>        # 执行 action.*
python tools/mc_ep.py --port COM7 dump                 # 读所有可读参数
python tools/mc_ep.py --port COM7 monitor <name>... --rate 20
```

| 前缀 | 内容 | 权限 |
|---|---|---|
| `config.*` | 电流/位置/速度/电机/编码器/CAN/零点 | `rw` |
| `telemetry.*` | 实时测量 | `r` |
| `timing.*` | ISR 耗时统计 | `r` |
| `system.*` | 版本、配置序号、schema、保存状态 | `r` |
| `action.*` | 保存、置零、切模式、标定 | `w` |

规则：

- **写会被限幅**：超出 `min`/`max` 夹到边界，不报错。`set` 打印实际写入值。
- `NaN`/`Inf` → `EP_ERR_BAD_VALUE`（3）。
- 写只读端点 → `EP_ERR_READ_ONLY`（4）。
- **读动作端点 → `EP_ERR_WRITE_ONLY`（5）**。这是安全设计：一次读不能让电机动作。
- 状态前置条件不满足（如门极使能时改 `config.ppairs`）→ `EP_ERR_STATE`（6）。
- `config.p_min`/`v_min` 可单独写（通常由 `p_max`/`v_max` 联动），用于备份恢复。

状态码：0 OK、1 未知 ID、2 长度错、3 数值非法、4 只读、5 只写、6 状态不允许、
7 描述符版本不匹配（工具会自动重新下载描述符）。

### 在 Python 脚本里用

```python
import sys
sys.path.insert(0, "tools")
from mc_ep import Device

dev = Device("COM7", 115200, 1.0)
dev.fetch_descriptor()
print(dev.read("config.gr"))
dev.write("config.gr", 7.75)
dev.call("action.save")
print(dev.read("system.save_status"))
dev.close()
```

## 3. 保存

改参数只改 RAM，**断电会丢**：

```bash
python tools/mc_ep.py --port COM7 call action.save
python tools/mc_ep.py --port COM7 get system.save_status
# 0 还没执行 / 1 成功 / 2 因门极使能跳过 / 3 写失败
python tools/mc_ep.py --port COM7 get system.prefs_sequence   # 成功时 +1
```

`call action.save` 返回 OK 只表示**已受理**，真正的擦写在主循环里做；用
`save_status` 或 `prefs_sequence` 确认。

保存过程中掉电：上电后仍是上一次的完整配置（双页 + 序号）。

## 4. 动作与模式

```bash
python tools/mc_ep.py --port COM7 call action.enter_menu    # 回菜单，清锁存故障
python tools/mc_ep.py --port COM7 call action.motor_mode    # 进入电机模式
python tools/mc_ep.py --port COM7 call action.calibrate     # 编码器标定
python tools/mc_ep.py --port COM7 call action.zero          # 当前位置设为零点并保存
python tools/mc_ep.py --port COM7 call action.setup_mode    # 参数设置界面
python tools/mc_ep.py --port COM7 call action.encoder_mode  # 编码器显示
python tools/mc_ep.py --port COM7 call action.clear_fault   # 等价于回菜单
```

`action.enter_menu` 会停机并清故障，是正常的"急停"用法。

## 5. 标定

第一次使用、换电机/编码器、整片擦除后要做一次。**OTA 升级不需要重新标定。**

前置：电机轴能自由转动；动力电已上；`config.i_cal` 合适（默认 5 A，小电机
可降到 1–2 A）；门极关闭（菜单）。

```bash
python tools/mc_ep.py --port COM7 call action.enter_menu
python tools/mc_ep.py --port COM7 set config.i_cal 2.0
python tools/mc_ep.py --port COM7 call action.calibrate
```

电机会先对齐相序，再正反转各一圈采样。完成后自动回菜单、应用结果并保存
（LUT 写入隐藏记录 `internal.encoder_lut`）。核对：

```bash
python tools/mc_ep.py --port COM7 get system.save_status      # 1
python tools/mc_ep.py --port COM7 get system.prefs_sequence   # 比标定前 +1
```

失败（电机卡住、动力电不足、`i_cal` 不合适）会自动回菜单；随时
`call action.enter_menu` 中止。

标定结果里 `E_ZERO`/`PPAIRS`/零点在 `config.*`，128 点 LUT 在隐藏记录里，
**都持久化、都随 OTA 保留**；LUT 不可通过 endpoint 备份。

## 6. 备份与恢复

只备份"持久化 = 是"的参数（见 [parameters.md](parameters.md)）。LUT 是隐藏记录，
不在备份里。

```python
# backup
dev = Device("COM7", 115200, 1.0); dev.fetch_descriptor()
data = {e["name"]: dev.read(e["name"]) for e in dev.descriptor
        if e.get("persist") and e.get("access") in ("r", "rw")}
json.dump(data, open("backup.json", "w"), indent=2)

# restore（门极关闭；按 ID 升序写，p_max 先于 p_min）
by = {e["name"]: e for e in dev.descriptor}
for name in sorted(data, key=lambda n: by[n]["id"]):
    if by[name].get("access") in ("w", "rw") and by[name].get("persist"):
        dev.write(name, data[name])
dev.call("action.save")
```

恢复脚本会跳过当前固件里不存在的参数。换主板/整片擦除后必须重新标定。

## 7. OTA 升级

**只要 OTA 只擦 app 区（`0x08000000..0x0807BFFF`），升级后配置、零点、标定 LUT
全部保留。** 整片擦除会全丢——这是固件救不了的。

升级前（建议）：

```bash
python tools/mc_ep.py --port COM7 get system.prefs_sequence
python tools/mc_ep.py --port COM7 get config.gr
python tools/mc_ep.py --port COM7 get config.m_zero
```

升级后核对：

```bash
python tools/mc_ep.py --port COM7 get system.version        # 新版本
python tools/mc_ep.py --port COM7 get system.prefs_loaded   # true
python tools/mc_ep.py --port COM7 get system.config_schema  # 与发布说明一致
python tools/mc_ep.py --port COM7 get config.gr             # 与基线一致
```

LUT 的验证：把电机停到升级前同一机械角度，比较 `telemetry.encoder_raw`。

| 现象 | 原因 | 处理 |
|---|---|---|
| `prefs_loaded = false`，参数全默认 | 配置页被擦了 | 查 OTA 擦除范围；从备份恢复；重新标定 |
| 个别参数变默认 | 该 ID 的类型/长度变了（不应发生） | 查发布说明；备份恢复 |
| `system.version` 没变 | 没烧进去 | 重试 |
| 工具报状态 7 | 描述符 CRC 变了 | 用新版 `mc_ep.py`，自动重下描述符 |

**第一次刷入 TLV 配置格式的固件**（本仓库之前的版本用的是裸数组）会忽略旧块，
参数回默认值，需要重新设置 + 标定一次；之后每次升级都会保留。

回退到旧固件：旧固件不认识新格式，参数回默认值；在旧固件里保存会覆盖配置页，
再升回来也回不到新格式的值。回退前先备份。

## 8. 故障排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `无法打开串口` | 端口错；被别的程序占用 | 关掉串口终端/另一个工具；确认 COM 号 |
| 一直超时 | 板子没运行；USB 没枚举成 CDC | 重新上电；看设备管理器 |
| `没有返回描述符版本信息` | 端口错或收到的是菜单输出 | 用 `descriptor` 看原始返回；只留一个使用者 |
| 状态 1 未知 ID | 名字错；上位机描述符过期 | `tree`；升级工具 |
| 状态 4/5/6 | 写只读、读动作、状态不允许 | 看参数手册；`call` 代替 `get`；先回菜单 |
| 状态 7 | 描述符 CRC 变了 | 新版工具会自动重下 |
| `save_status = 2` | 门极使能 | 回菜单再存 |
| `save_status = 3` | flash 写失败 | 看串口打印；`make check` 查容量；重试 |
| `set` 的值和给的不一样 | 被限幅 | 正常，`set` 打印的是实际值 |
| 串口终端乱码 | 两个程序抢一个串口 | 关掉多余连接，重新上电 |

## 9. FAQ

**升级后要重新标定吗？** 不用，除非换了硬件或整片擦除。

**参数存在哪？** `0x0807C000`/`0x0807E000` 两页，双份 + CRC32 + 序号；
写了要 `action.save` 才落盘。

**为什么 `get action.save` 报错？** 动作端点只能执行不能读——否则一次误读就能
让电机停机。

**为什么 `p_min` 可以单独写？** 正常由 `p_max` 联动；单独写是为了备份恢复，
限幅仍保证它 ≤ 0。

**LUT 能导出吗？** 不能，隐藏记录。它随配置页保留；换板/整片擦除要重新标定。

**能加多少参数？** 配置记录 256 条、总载荷 4 KB、描述符 64 KB。当前用了 27 条
配置 + 1 条 LUT（512 B）。超限会在 `make endpoints` 时直接报错。

**保存会磨损 flash 吗？** 每次擦一页（8 KB），只在显式 `action.save` 时发生。

**支持 CAN 吗？** 现有 11-bit 协议不变；CyberBeast 29-bit 的 endpoint 访问正在
移植，方案见 [DEV.md](DEV.md#can--cyberbeast-移植)。
