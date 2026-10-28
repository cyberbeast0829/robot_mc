# 开发者手册

面向修改固件的人。用户侧用法见 [USER.md](USER.md)。

目录：[架构与不变式](#架构与不变式) · [endpoint 协议](#endpoint-协议) ·
[params.json 与代码生成](#paramsjson-与代码生成) · [配置持久化](#配置持久化) ·
[Flash 布局与 OTA 契约](#flash-布局与-ota-契约) · [CAN：协议选择与 CYBERBEAST](#can协议选择与-cyberbeast) ·
[测试](#测试) · [发布检查表](#发布检查表) · [接口变更规则](#接口变更规则)

## 架构与不变式

N32H473CE，Cortex-M4F，裸机。控制环 **40 kHz**（`DT = 25 µs`），由
`ATIM1_UP_IRQHandler`（优先级 0）驱动；ATIM1 的 update 同时硬件触发三路注入
ADC，采样点在 PWM 峰值、低侧采样窗口内。

```
上位机 ──USB CDC──> endpoint_serial.c   中断只累积帧，主循环解析/应答
                        └─ endpoint_handler(id, op, in, out)   [endpoint_defs.c 生成]
                              └─ 直接读写 params.json 里的 C 变量
主循环 ──> run_fsm_slow() ──> save_preferences() ──> prefs_codec (TLV+CRC32)
                                                     └─ 0x0807C000 / 0x0807E000 ping-pong
控制 ISR ──> analog_sample → ps_sample → check_faults → run_fsm → can_tx_rx
```

| 文件 | 职责 | 可在 ISR 里跑 |
|---|---|---|
| `n32h47x_48x_it.c` | 控制 ISR 调度、`fault_shutdown()` | 是 |
| `foc.c` | 电流环/外环/`check_faults` | 是 |
| `position_sensor.c` | SPI1+DMA2 读编码器 | 是（唯一允许阻塞等 DMA） |
| `fsm.c` | 状态机：`run_fsm`（ISR）/`run_fsm_slow`（主循环） | 分开 |
| `endpoint.c` | CRC、LE 编解码、endpoint 0 描述符 | 是（纯计算） |
| `endpoint_serial.c` | USB 帧收发 | `feed()` 是，`poll()` 否 |
| `endpoint_defs.c` | 生成的 handler | 是（只读写变量/置标志） |
| `prefs_codec.c` | TLV 编解码（纯逻辑，可主机测试） | 否 |
| `preference_writer.c` | 擦/写/校验 flash | 否 |
| `main.c` | 初始化、主循环、`myprintf`/`console_write_raw` | 否 |

必须保持：

- **ISR 不 printf、不写 flash、不阻塞**（唯一例外：`ps_sample` 的编码器 SPI+DMA）。
- **handler 只读写变量/置标志**：它将来会在 CAN 中断上下文里被调用。
- **动作端点必须由 `EP_OP_WRITE` 触发**，不能从"payload 为空"推断读/写。
- **`save_preferences()` 只在主循环、门极关闭时调用**；它内部关中断覆盖整个
  擦写序列（N32 的 64-bit 编程在两次写之间被中断会死锁）。
- **IWDG 由控制 ISR kick**；不要制造 > 4 s 的停顿。
- **配置页只由 `preference_writer.c` 访问**。
- **参数表只改 `params.json`**，不手改 `endpoint_defs.*` / `prefs_defs.*`。

加一个参数：`params.json` 加条目（要持久化再加 `persist: true` + `default`）→
`make endpoints` → `make check` → `make` → `make hosttest`。

## endpoint 协议

与 ODrive fibre v0.1 同构，跑在 USB CDC 上，和 ASCII 菜单共用数据流。
实现见 `Core/Inc/endpoint.h`、`endpoint.c`、`endpoint_serial.c`。

### 帧

```
AA | LEN | CRC8 | PAYLOAD[LEN] | CRC16_HI CRC16_LO
```

- `LEN`：payload 长度，1..64。
- CRC8 对 `AA, LEN`；CRC16 对 payload，**高字节在前**。
- CRC8 多项式 `0x37` / 初值 `0x42`；CRC16 `0x3D65` / `0x1337`（MSB-first、
  无反射、无最终异或），与 ODrive `crc.hpp` 逐位一致。

### 请求 payload

```
seq(2) | id|flags(2) | rxlen(2) | tx_data... | trailer(2)
```

- `id|flags`：bit15 `EP_REQ_REPLY`（必须置位，不置位设备丢弃请求）；
  bit14 `EP_REQ_WRITE`（写/执行；不置位是读）；低 14 位是 ID。
- `rxlen`：主机希望的返回字节数（超出被截断）。
- `trailer`：endpoint 0 是 `EP_PROTOCOL_VERSION`（2），其它是描述符 CRC16
  （每次改参数表都会变）。
- 一次只有一个未完成请求；`req_pending` 未清时新帧被丢弃。

### 响应

```
seq|0x8000(2) | status(1) | data...
```

| 状态 | 含义 |
|---|---|
| 0 `EP_OK` | 成功 |
| 1 `EP_ERR_UNKNOWN_ID` | 没有这个 endpoint |
| 2 `EP_ERR_BAD_LEN` | 长度不够/缓冲区太小 |
| 3 `EP_ERR_BAD_VALUE` | NaN / 无法表示 |
| 4 `EP_ERR_READ_ONLY` | 写了 `r` 端点 |
| 5 `EP_ERR_WRITE_ONLY` | 读了 `w` 端点（含所有 `action.*`） |
| 6 `EP_ERR_STATE` | 状态不允许（如门极使能时保存） |
| 7 `EP_ERR_VERSION` | 描述符 CRC 不匹配，主机应重新下载 |

### 读 / 写

**由传输层显式给出**：USB 用 bit14，CyberBeast CAN 用 `0x20`/`0x21` 操作码。
生成的 handler 按 `access` 统一校验：`r` 拒绝写、`w` 拒绝读、`rw` 写后回读新值。

### endpoint 0（描述符）

`tx_data` 是 `u32` 小端偏移：`0xFFFFFFFF` 返回 `[crc u16][len u16]`；其它偏移
返回从该处开始的一段（长度由 `rxlen` 决定，`mc_ep.py` 用 60）。

描述符是纯 ASCII JSON 数组。约束（改 `descriptor_json()` 时别破坏）：

- 不能出现 `\uXXXX`（JointSDK 的轻量解析器不支持）；
- 类型名用 `bool/uint8/int8/uint16/int16/uint32/int32/uint64/int64/float/double/
  object/endpoint_ref/json/function`；
- 动作端点写 `"type":"function"` + 空 `inputs`/`outputs`，**不能写 `void`**；
- 总长 ≤ 65535（当前约 10.2 KB）。

例子（读 `config.i_max`，ID 2）：

```
payload = 01 00 | 02 80 | 04 00 | 56 34
frame   = AA 08 <crc8> 01 00 02 80 04 00 56 34 <crc16_hi> <crc16_lo>
```

写 `action.save`（ID 71）：

```
payload = 03 00 | 47 40 | 00 00 | 56 34      ; 0x4047 = 71 | EP_REQ_WRITE
```

动作端点里与运维相关的两个（都只置标志，真正的工作在 `run_fsm_slow()`）：

| 端点 | ID | 行为 |
|---|---|---|
| `action.reboot` | 127 | `state.reboot_pending = 1` → 先 `request_fsm_state(MENU_MODE)`（关栅），0.5 s 后（`REBOOT_DELAY_CYCLES`）`NVIC_SystemReset()`。**不动 `_reboot_cookie`**：正常启动后它本来就是 42，而 `main.c` 的 `!= 42 → 再复位一次` 是"启动状态不确定"的保险，不该由这里关掉。没保存的改动会丢 |
| `action.reset_config` | 126 | `state.reset_config_pending = 1` → `config_apply_defaults()` + `config_apply_limits()` + `save_preferences()`，**保存成功后自动 `reboot_pending = 1`**（`CAN_ID/CAN_PROT/CAN_BAUD` 是运行期读取的，改了不重启会让主站失联，而且 `config.*` 与 `comm_encoder.*` 会处于混合状态）；`require` 要求门极关闭（否则端点返回 `EP_ERR_STATE=6`），主循环里再加一道 `save_status = 2` 兜底。写 flash 失败时**不**重启 |

**`action.encoder_mode`(77) / `action.setup_mode`(78) 已退役**：固件不再提供它们
（串口菜单里的 `e`/`s` 入口仍然保留）。按“ID 只增不改”的规则，这两个 ID 仍在
`tools/endpoint_ids.json` 的 map 里占位，**永远不要重新分配给别的参数**；新动作取
126/127。

**一次性例外**：`action.enter_menu` 曾原地改名为 `action.idle_mode` 但**保留 ID 73**
（工程尚未交付客户，不欠外部兼容）。今后再改名必须走"新名字 = 新 ID，旧 ID 留空号"。

`action.*` 会写 `system.save_status` 的路径共有四条（`action.save`、`action.zero`、
`action.reset_config`、标定完成、setup 退出），它的含义是"最近一次写 flash 的结果"。

### command.*（运行命令）

五个 `command.*` 端点直接写 `controller.t_ff/p_des/v_des/kp/kd`，**不持久化**，
描述符里也不给 `min`/`max`（`range_dynamic: true`：真实范围由 `config.i_max/kt/gr/
p_max/v_max/kp_max/kd_max` 在运行期决定，写死一个数字会让上位机按错范围缩放）。

两条必须保持的语义：

- **写入会刷新命令租约**（`"set": "... = v; controller.timeout = 0;"`）。CAN 帧一直是
  这么做的；endpoint 必须一致，否则默认 `config.can_timeout = 100 ms` 一到，
  `run_fsm` 的 MOTOR_MODE 分支就会 `zero_commands()`，USB 写进去的力矩活不过 100 ms。
  刷新的好处是"主机掉线 ⇒ 自动卸力"。**不要靠把 `can_timeout` 置 0 来绕过**，
  那等于永久关掉失效保护（而且会被 `action.save` 存进 flash）。
- **进入 MOTOR_MODE 先 `zero_commands()`**（`fsm_enter_state`）。`command.*` 在任何
  状态都能写，若不在使能前清零，空闲态设的 `t_ff` 会在门极打开的第一拍生效。

## params.json 与代码生成

`params.json`（仓库根）是唯一真源：endpoint、描述符、持久化表、默认值、限幅、
参数手册都由它生成。生成物提交进仓库（Keil 和没装 Python 的机器要能编译）。

| 文件 | 作用 |
|---|---|
| `params.json` | 参数/遥测/动作定义 |
| `tools/endpoint_ids.json` | 名字 → 16 位 ID 的永久注册表 + `next_id` |
| `tools/gen_endpoints.py` | 生成器（`--check` 只校验） |
| `Core/{Inc,Src}/endpoint_defs.*` | ID 枚举、描述符、handler |
| `Core/{Inc,Src}/prefs_defs.*` | 持久化表、默认值、限幅 |
| `docs/parameters.md` | 参数手册 |

字段：

| 字段 | 说明 |
|---|---|
| `name` | 点分小写；第一段决定分组（`config/telemetry/timing/system/action`） |
| `type` | `bool/int8/uint8/int16/uint16/int32/uint32/float/void/blob` |
| `access` | `r` / `w` / `rw` |
| `ref` | 可赋值的 C 左值（写路径） |
| `get` | 读取表达式（`rw` 可同时给 `get` 和 `ref`） |
| `set` | 写入语句，`v` 是已校验/限幅的值；缺省 `ref = v;` |
| `call` | 动作语句，只能置标志/写变量 |
| `require` | 为假时 `EP_ERR_STATE`；支持以 `!` 开头 |
| `min`/`max` | 描述符范围 + 限幅依据 |
| `min_expr`/`max_expr` | C 侧限幅用宏（如 `I_MAX_LIMIT`、`P_MIN`），避免两处数字漂移 |
| `range_dynamic` | 真实范围由固件宏/用户配置在运行期决定 → **不发** `min`/`max` 给上位机（仍用它们做 C 限幅）；生成器要求同时有 `min/max` 和 `*_expr` |
| `unit`/`brief` | 只进文档/描述符。**`brief` 是中文字符串，不进线上描述符** |
| `persist`/`default` | 进持久化表；`default` 必填（擦除芯片时用） |
| `hidden` | 不进描述符/handler，只进持久化表（编码器 LUT） |
| `size`/`assert` | blob 字节数；`assert` 生成 `_Static_assert` 绑死尺寸 |

ID 规则：**只增不改、永不复用**。改名 = 新增（旧 ID 留空号）；改单位/含义 =
新 ID + 迁移。新增条目时 `make endpoints` 自动分配 `next_id` 并回写注册表。

生成器还会校验：

- 类型/access 合法、`void` 有 `call`、可读/可写端点有 `get`/`ref`；
- `persist` 必须有 `ref` 和 `default`；
- **容量**（从 `prefs_codec.h` 读上限）：单条 ≤512 B、总载荷 ≤4096 B、条数 ≤256；
- 每个生成的 `case` 必须 `return`（防止 fall through 到下个端点）。

命令：

```bash
make endpoints   # 重新生成
make check       # 校验生成物（提交前/CI）
make hosttest    # 主机单测（prefs_codec）
make             # 编译
```

Windows 默认 `PYTHON=py -3`，其它平台 `python3`；可 `make check PYTHON=...` 覆盖。

`call`/`set` 里**不能** printf、写 flash、等 DMA。需要延后做的事只置标志：
`state.save_pending = 1;`、`state.zero_pending = 1;`、
`request_fsm_state(&state, MOTOR_MODE);`（带临界区；不要从 handler 直接调
`update_fsm()`，那是串口输入路径）。

## 配置持久化

`prefs_codec.c`（纯逻辑，可主机测试）+ `preference_writer.c`（flash）。

### 为什么不用裸数组

旧格式是 `int[256] + float[64]` 整块 + `CONFIG_VERSION` 相等判断：数组下标就是
语义，加/删/重排一个参数所有值错位，版本不等就整体丢弃。现在记录以 **endpoint
ID** 为 key，与 RAM 布局解耦。

### 块格式

两页各 8 KB，每页最多一个块。

头（20 B，小端）：`magic 'PCFG'`(u32) | `schema`(u16) | `flags`(u16) |
`sequence`(u32) | `payload_len`(u16) | `record_count`(u16) | `crc32`(u32)。

CRC 是 **CRC-32/MPEG-2**（`0x04C11DB7`，初值 `0xFFFFFFFF`，不反射、无最终异或），
覆盖 header[0..15] + payload。

记录：

```
id u16 | type u8 | flags u8 | len u16 | reserved u16 | value[len] | padding
```

`id` 与 `endpoint_ids.json` 同一命名空间；`len` ≤ 512；值小端、4 字节对齐。

### 保存

1. 开中断状态下把 RAM 序列化成 TLV，并把旧块里"当前固件不认识"的记录原样追加
   （未知记录透传）。
2. `__disable_irq()`：擦非活动页 → 按 8 字节 double-word 编程（每 8 字节 kick
   IWDG）→ 校验 magic/schema/CRC/序号。
3. 通过才切换 `active_sector`；失败保留旧页并返回 0。

掉电最多丢"这一次保存"；关中断是必须的（见"架构与不变式"）。

### 加载

`main.c`：`config_apply_defaults()` → `preference_writer_load()` →
`config_apply_limits()`。两页都有效取序号大的；记录匹配要求 **ID + type + len
全等**，不会把 `int32` memcpy 到 `float`；缺记录保持默认值。

### 变更策略

| 变更 | 做法 | 升级后 |
|---|---|---|
| 新增参数 | 加 `persist` + `default` | 用默认值，其余不动 |
| 删除参数 | 直接删 | 旧记录透传保留；恢复同 ID 能拿回旧值 |
| 改名 | 新名字新 ID | 旧值留在旧 ID 下 |
| 改单位/含义 | 新 ID + `PREFS_SCHEMA` +1 + 迁移 | 迁移函数换算 |
| 改 C 类型/长度（同 ID） | **不要** | 该参数静默回默认值 |
| 改块布局/编码 | `PREFS_SCHEMA` +1 | 见下 |

迁移：`PREFS_SCHEMA` +1 → `prefs_block_valid()` 本来会拒旧块。做法是让它**继续接受**
上一个 schema（`PREFS_SCHEMA_V1`），再在 `preference_writer_load()` 里用
`prefs_block_schema()` 辨认版本、就地做一次性换算。这样旧配置（含编码器 LUT/标定）
不会丢；换算只发生在 RAM，下次保存写出的块已经是新 schema（**不要**在加载后再
“顺手再存一次”——那会在开机时擦写 flash）。

已有迁移：**schema 1 → 2**——`config.can_timeout` 的单位从「控制周期（25 µs）」
改成**毫秒**（`prefs_v1_cycles_to_ms()`，40 控制周期 = 1 ms，四舍五入，0 仍是关闭）。
新默认值 100 ms（旧默认 1000 周期 = 25 ms）。

容量：单条 512 B、总载荷 4096 B、记录 256 条。`internal.encoder_lut` 是 512 B，
`prefs_defs.c` 里的 `_Static_assert(N_LUT * sizeof(int) == 512)` 会在改 `N_LUT`
时编译失败——同时改 `params.json` 的 `size`。

主机测试：`make hosttest`（往返、缺记录默认值、未知记录透传、512 B blob、
CRC/schema 拒绝、空表拒绝、`old_block == out` 别名拒绝、容量失败、
schema 1→2 单位迁移与降级块仍可读）。

## Flash 布局与 OTA 契约

**这是"OTA 后配置/标定还在"的唯一硬前提。**

| 区域 | 地址 | 大小 |
|---|---|---|
| App | `0x08000000` – `0x0807BFFF` | 496 KB (`0x7C000`) |
| 配置页 A | `0x0807C000` – `0x0807DFFF` | 8 KB |
| 配置页 B | `0x0807E000` – `0x0807FFFF` | 8 KB |

链接器强制：`N32H473CE_FLASH.ld` 的 `LENGTH = 0x7C000`；
`Keil/ODrive_N32.sct` 的 `LR_IROM1 0x08000000 0x0007C000`。镜像超了直接报错。

bootloader 契约（本仓库不含 bootloader，app 链接在 `0x08000000`）：

1. **只擦/写 `0x08000000..0x0807BFFF`**，禁止 mass erase / 512 KB 整段擦。
2. 不把 bootloader 自己的参数放在这两个页。
3. "恢复出厂"只擦 app 区，配置由固件自己的 action 清理。

**OTA 闭环验证（发布前必做）**

1. 记录 `config.gr`、`config.m_zero`、`system.prefs_sequence`，以及某固定机械
   角度下的 `telemetry.encoder_raw`。
2. 用客户同款流程执行一次真实 OTA。
3. 读回：`system.version` 为新版；`system.prefs_loaded = true`；
   `config.gr`/`m_zero` 一致；`prefs_sequence` 未变；同一角度 `encoder_raw` 一致。
4. `call action.save` → `prefs_sequence` +1、`save_status = 1`；断电重启再抽查。

任何一项不符，先查 bootloader 擦除范围，不要先怀疑 TLV 代码。

失败恢复：保存掉电用另一页；擦写失败 `save_status = 3` 且旧页保留；两页都坏
→ 默认值 + `prefs_loaded = 0`，需要重新设置 + 标定。

主动“恢复出厂”：`call action.reset_config`（需门极关闭）——它 = `config_apply_defaults()`
、`config_apply_limits()`、保存一页，**会连标定一起清掉**（`ppairs`/零点/LUT 回默认），
`CAN_ID/CAN_PROT/CAN_BAUD` 也回默认（`can_prot=0` ⇒ 旧 11-bit），所以保存成功后
固件会自动重启（0.5 s），之后要重新配 CAN 并重新标定。运行中的 `comm_encoder` 副本
故意不跟着改：重启前继续用旧标定，比拿全零 LUT 去换相安全。

## CAN：协议选择与 CYBERBEAST

**三个持久化参数决定协议与物理层，开机时读取一次，写入后下次上电生效：**

| 参数 | 含义 |
|---|---|
| `config.can_prot` | `0` = 旧 11-bit 协议（默认），`1` = CYBERBEAST |
| `config.can_baud` | bit/s。`≤ 1M`：Classic CAN，该值就是位速率；`> 1M`：CAN FD，仲裁域固定 1 M，数据域 = 该值（支持 2M/4M/5M） |
| `config.can_hb_ms` | CYBERBEAST 心跳周期（默认 100 ms，0 关闭） |

`system.can_prot_active` / `system.can_fd` / `system.can_baud_active` 报告**真正在跑**
的协议、物理层与速率。`can_prot=0` 时 `can_baud > 1M` 会被钳到 1M（旧协议只有
8 字节帧）；FD 数据速率不在 2M/4M/5M 表里时退回 Classic 1M，并让
`system.can_fd=0` 如实反映，不做“看起来是 FD、其实没跑起来”的静默降级。
Classic 速率必须能由 40 MHz / (40 个时间量子) 整除：50k/100k/125k/200k/250k/500k/1M。

为什么必须开机生效：NBTP/DBTP、消息 RAM 元素大小（8 B vs 64 B）和滤波器类型
都由这两个值决定，运行期重建会打断 40 kHz 控制循环。因此 `MX_CAN1_Init()` 从
CubeMX 的 SysInit 段移到了配置加载之后
（`config_apply_defaults` → `preference_writer_load` → `config_apply_limits` 之后）。

### 代码结构

| 文件 | 职责 |
|---|---|
| `can.c` | 时钟/引脚、NBTP/DBTP、消息 RAM、滤波器、`can_send_frame` / `can_rx_fetch` |
| `can_proto.c` | 协议表（`can_protocol_t`）与分发；25 µs → 1 ms 时基 |
| `can_mit.c` | 旧 11-bit 协议（从 `n32h47x_48x_it.c` 原样搬出） |
| `can_cyberbeast.c/.h` | CYBERBEAST |
| `can_cyberbeast_codec.h` | 纯编解码（固件与 host 测试共用） |

`n32h47x_48x_it.c` 的 `can_tx_rx()` 只做：`can_proto_tick()` + 每周期最多取 2 帧
→ `can_proto_rx()`。控制帧、查询、心跳、SDO 参数访问都在 ISR；**描述符流式发送
在主循环**（`can_proto_poll()`，2 帧/ms）。新增协议 = 一个 `can_protocol_t` + 一个
`can_prot` 枚举值，不动硬件层。

### CYBERBEAST 实现范围

| MsgType | 行为 |
|---|---|
| `0x00/0x80` MIT | 8 字节位域 → `p_des/v_des/kp/kd/t_ff`（**输出端 1:1**）；单播回 8 字节响应；FD 广播按 `slot = node_id` 取槽（node_id 必须 1..7） |
| `0x03/0x83` TORQUE | 线上是电机端 N·m → `t_ff = τ × GR`，`kp = kd = 0` |
| `0x04` CURRENT | `t_ff = I × KT × GR`，不回应答；**刷新超时**（比 ODrive 的 F19 更安全） |
| `0x01/0x02` CSP/CSV | **暂不支持**（本固件没有独立位置/速度环），静默忽略 |
| `0x20/0x21` SDO | 直接 `endpoint_handler()`；值小端；Classic 下 ReqLen 钳到 4；`>4` 字节走分段写；批量读回 ERR |
| `0x22` SAVE | 置 `state.save_pending`（门极关闭时由主循环写 flash） |
| `0x24/0x25` 描述符 | 元数据 + 定长分片（8/64 B，末尾补 0），2 帧/ms |
| `0x40..0x49` 查询 | 状态/位置速度/电流/温度/母线/错误/版本；功率与母线电流返回 0 |
| `0x48` 心跳 | Classic 8 B / FD 19 B；pos/vel 按**原始计数域**钳位（F32 教训）；字节 18 是映射后的 ODrive AxisState |
| `0x60..0x65` | SET_NODE_ID（RAM，需 save 持久化）/ SET_ZERO / START（仅 MENU）/ STOP / RESET / CLEAR |
| `0xC0` ESTOP | `controller.fault |= FAULT_ESTOP` → FAULT_MODE 关栅，需显式回菜单清除 |

心跳的**目的地址是主站 ID**（协议 §4.6 `Dest=MasterID`，不是广播）：`cb_master_id` 从收到的帧里
学 Source，**复位/上电清零**，清零期间按默认主站 `1` 发送（`config.can_master` 只在没学到值之前
起作用，学到了就听学到的）。⇒ 主站 ID 不是 1 时，必须**先发一帧**（任意类型）才会把心跳改发到
自己的 ID；台架脚本不要按 `dest == 本机` 过滤心跳，否则"刚复位没心跳"是假故障。

单位表（最容易错的地方）：

| 量 | 线上 | 本工程 |
|---|---|---|
| MIT pos/vel/kp/kd/τ | 输出端 rad / rad/s / N·m/rad / N·m | 同口径，1:1 |
| TORQUE / CURRENT | 电机端 N·m / A | `×GR` / `×KT×GR` 换成输出端 `t_ff` |
| 心跳 pos/vel | 电机端 turns（×100 / ×10000） | `angle_multiturn/(2π)` |
| QUERY_POS_VEL | 电机端 turns | 同上 |

量程沿用 `P_MAX/V_MAX/KP_MAX/KD_MAX/I_MAX`：MIT 力矩满量程 `I_MAX·KT·GR`，
响应电流满量程 `I_MAX·GR`（= `mit_max_torque / KT`），与 ODrive 的动态算法一致。

### 与 JointSDK 的现状

`jsdk_context_configure()` 仍会失败：它要求 `axis0.motor.config.gear_ratio` 等
7 个 ODrive 路径。**本工程刻意不加这些别名**（客户将改 JointSDK 适配），
所以现阶段用 `jsdk_context_desc_fetch()` + `jsdk_joint_param_get/set()` 访问参数，
控制用 `0x00/0x03/0x04`。描述符仍是纯 ASCII、动作端点为 `function` 类型，
可直接被 SDK 的 `cb_jsondesc_parse` 解析（129 个可见端点，约 10.2 KB）。

### 台架注意

- 5 M 数据域没有额外配置 TDC（与 ODrive 现状一致），需要实测误码率/TEC。
- 仲裁域统一 40 个时间量子；旧协议保持原采样点 **87.5%**（34/5，总线行为不变），
  CYBERBEAST 用 80%（31/8，与 ODrive 一致；FD 数据域 5 M = 8 tq/75%）。
- 扩展帧过滤器接受**全部**扩展帧，Dest 由软件过滤；1M 满总线约 2k 帧/s，
  40 kHz 轮询可消化 40k 帧/s。
- 改 `can_prot` / `can_baud` / `can_id` 后必须断电重启，`system.can_prot_active`
  可核对；`can_id` 的滤波器和 CyberBeast 的 Dest 都按新值生效。
- `action.save` 写入的是新参数值，但运行中的物理层不变（下次上电才切换）。

### 台架实测（2026-10-26，Keil 构建 + CANable 2.0）

| 场景 | 结果 |
|---|---|
| Classic 1M：A 握手/SDO/心跳/描述符/查询/ESTOP | 34/34 |
| Classic 1M：B 闭环力矩脉冲 | 7/7 |
| FD 1M/5M：A | 34/34 |
| FD：B MIT 单播脉冲 / C 64 B 广播脉冲 | 7/7 · 5/5 |
| FD：C-static 广播槽位 + 单位映射 | 9/9 |
| FD：`0x64` 重启（回 1.99 s）+ 配置保持 | 7/7 |
| FD：SDO/控制错误路径（D） | 12/12 |
| FD：MOTOR 下命令超时（T，含 20 Hz 主站存活） | 11/11 |

schema 1 → 2 迁移已实机验证（同一块配置，不动 flash）：旧块算出
`config.can_timeout = 25`（原 1000 控制周期）、`system.config_schema = 2`、
`prefs_sequence` 不变、`e_zero = 934` / `ppairs = 14`（同块的标定记录）全部保留；
改成 100 后 `action.save` → sequence +1，重启后直读 schema 2 块，值仍是 100
（**没有二次换算**）。

几条必须记住的行为（都是客户会踩的）：

- **`config.can_timeout` 的单位是毫秒**（默认 **100 ms**，0 = 关闭）。固件内部用
  `CAN_TIMEOUT_CYCLES`（×40）与 `controller.timeout`（控制周期）比较，实测最后一帧后
  **100 ms 量级**被 `zero_commands()` 清零。⇒ 主站必须快于 10 Hz（旧版本是 25 ms 超时、
  要求 >40 Hz，已改）。`telemetry.can_timeout_count` 报的是**控制周期数**（饱和在
  `can_timeout × 40`），用来判断“离超时还有多远”。
- **写的 ACK 不含成败位**（协议如此）：只读端点、类型不符、`require` 不满足都**照样回 ACK**，
  必须读回判定。实测：写 `telemetry.v_bus` 无效果；门极使能时写 `config.gr` 被拒（值不变）。
- 错误路径实测：批量读（flags bit6）回 `BATCH|ERR`（`0x60`）；读 `action.*` 回 0 字节
  且**不会触发动作**；越界写钳到 min/max（`config.i_bw` 1e9→2000、-1e9→100）；
  NaN 被丢弃（值不变）；宽度不符（`config.gr` 用 int16）被丢弃；`0x03` TORQUE 超大值
  钳到 `I_MAX·KT·GR`，NaN 帧被忽略。
- 台架脚本：`python tools/cb_bench.py --port COMx --stage-a|--stage-b|--stage-c|--stage-c-static|--stage-d|--reset-test|--timeout-test [--fd]`。
  ⚠ 脚本自身的收发周期很关键：早期版本用 `ser.read(4096)`（串口 timeout 50 ms）导致
  实际只有 ~20 Hz、比 25 ms 的超时还慢，量出来的力矩/位置变化偏小 —— **别用"命令没反应"
  去怀疑固件，先数自己发了多少帧**。

## 测试

自动检查：`make check`（生成物/文档最新）、
`make hosttest`（`prefs_codec` + `test_can_cyberbeast`）、`make`（链接）。
Windows 用 MSYS2 bash 构建（Chocolatey 的 make 用 cmd，
`mkdir -p` 会失败）：

```bash
export PATH="/d/Debug/arm-gnu-toolchain-15.2.rel1-mingw-w64-x86_64-arm-none-eabi/bin:$PATH"
make -j8
```

改了协议/描述符时，用 JointSDK 的 `cb_jsondesc_parse.c` 解析
`mc_ep.py descriptor` 导出的 JSON，确认 `rc=0`、类型/权限正确。

台架清单（改 endpoint/持久化后）：

| # | 操作 | 期望 |
|---|---|---|
| 1 | 上电，`tree` | 列出参数树 |
| 2 | `set config.i_max 3.5` | 返回 3.5；再 `get` 一致 |
| 3 | `set config.i_max 99` | 返回 4.0（`I_MAX_LIMIT`） |
| 4 | `set telemetry.i_q 0` | 状态 4 |
| 5 | `get action.save` | **状态 5，电机不能动** |
| 6 | `call action.save` | `save_status = 1`，`prefs_sequence` +1 |
| 7 | 门极使能时 `call action.save` | 状态 6 |
| 8 | 断电重启 | 第 2 步的值保留；`prefs_loaded = true` |
| 9 | 标定后重启 | 同一角度 `encoder_raw` 不变 |
| 10 | 连续保存 3 次 | 序号连续，不丢配置 |
| 11 | 保存瞬间断电（重复几次） | 上电后是上一次的完整配置 |
| 12 | CRC 错的帧 | 设备不应答，后续通信正常 |
| 13 | 旧描述符 CRC | 状态 7 |
| 14 | `call action.reboot` | 应答先返回，~0.5 s 后重启；门极在复位前已关闭 |
| 15 | `call action.reset_config`（先记下 `prefs_sequence`） | `save_status = 1`、序号 +1；`config.e_zero/ppairs` 回默认；**随后自动重启**，重启后 `prefs_loaded = true`、`config.can_prot = 0`（记得切回需要的协议）并重新标定 |
| 16 | 门极使能时 `call action.reset_config` | `status 6`（`EP_ERR_STATE`），`save_status` 不变，配置不动 |
| 17 | 空闲态 `set command.t_ff 5` 后 `call action.motor_mode` | 门极使能后 `telemetry.t_ff = 0`（进入模式时清零），电机不窜动 |
| 18 | 电机模式下写 `command.t_ff`，然后停发 3×`can_timeout` | 命令自动清零（租约到期），`telemetry.i_q_des → 0` |

## 发布检查表

- [ ] `VERSION_NUM`、`EP_PROTOCOL_VERSION`、`PREFS_SCHEMA` 是否变化已确认；
- [ ] ID 注册表只追加、没有复用/重排；
- [ ] `make check` 通过，生成物与文档一起提交；
- [ ] GNU Make 和 Keil AC6 都能干净构建；`make size` 镜像 < `0x0807C000`；
- [ ] `make hosttest` + 台架清单通过；
- [ ] bootloader 擦除范围已确认，OTA 闭环验证通过（参数/零点/LUT 保留）；
- [ ] [README](README.md) 的版本矩阵已更新；客户可见的接口变化写进发布说明。

## 接口变更规则

对外契约只有三样：endpoint 协议、描述符、配置记录格式。

- **endpoint 协议**语义变化 → `EP_PROTOCOL_VERSION` +1（当前 2；v2 增加了
  bit14 读写标志，v1 的"空 payload 即执行"已移除）。
- **描述符**内容变化 → CRC 自动变，旧上位机会收到 `EP_ERR_VERSION`。
  改名不改 ID；改单位/含义用新 ID。
- **配置格式**变化 → `PREFS_SCHEMA` +1 + 迁移函数。
- **Flash 分区**变化 → 必须重新验证 bootloader 契约。

每次变化同步更新 [README](README.md) 的版本矩阵和本文档对应章节。
