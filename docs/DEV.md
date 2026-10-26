# 开发者手册

面向修改固件的人。用户侧用法见 [USER.md](USER.md)。

目录：[架构与不变式](#架构与不变式) · [endpoint 协议](#endpoint-协议) ·
[params.json 与代码生成](#paramsjson-与代码生成) · [配置持久化](#配置持久化) ·
[Flash 布局与 OTA 契约](#flash-布局与-ota-契约) · [CAN / CyberBeast 移植](#can--cyberbeast-移植) ·
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
- 总长 ≤ 65535（当前约 9 KB）。

例子（读 `config.i_max`，ID 2）：

```
payload = 01 00 | 02 80 | 04 00 | 56 34
frame   = AA 08 <crc8> 01 00 02 80 04 00 56 34 <crc16_hi> <crc16_lo>
```

写 `action.save`（ID 71）：

```
payload = 03 00 | 47 40 | 00 00 | 56 34      ; 0x4047 = 71 | EP_REQ_WRITE
```

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
| `min_expr`/`max_expr` | C 侧限幅用宏（如 `I_MAX_LIMIT`），避免两处数字漂移 |
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

迁移：`PREFS_SCHEMA` +1 → `prefs_block_valid()` 会拒旧块 → 在
`preference_writer_load()` 加"旧 schema 分支"读出旧值、立即按新格式保存一次。
当前没有迁移函数（schema 1 是第一版）。

容量：单条 512 B、总载荷 4096 B、记录 256 条。`internal.encoder_lut` 是 512 B，
`prefs_defs.c` 里的 `_Static_assert(N_LUT * sizeof(int) == 512)` 会在改 `N_LUT`
时编译失败——同时改 `params.json` 的 `size`。

主机测试：`make hosttest`（往返、缺记录默认值、未知记录透传、512 B blob、
CRC/schema 拒绝、空表拒绝、`old_block == out` 别名拒绝、容量失败）。

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
→ 默认值 + `prefs_loaded = 0`，需要重新设置 + 标定。没有自动"恢复出厂"。

## CAN / CyberBeast 移植

目标：用 JointSDK 的 CyberBeast 协议访问同一套 endpoint。endpoint 层已经就绪
（16 位 ID、小端、`r/w/rw`、纯 ASCII 描述符、`function` 类型的动作端点），
缺的只是 CAN 传输层。

CyberBeast 用 **29-bit 扩展帧**：`Priority(3) | MsgType(8) | Dest(8) | Source(8) | Seq(2)`；
现有协议用 11-bit 标准帧 `(opcode << 7) | node_id`。给 FDCAN2 增加扩展帧过滤器，
两套共存。

| 消息 | 帧格式 | 映射 |
|---|---|---|
| `0x20` 读 | `[flags][id u16 BE][req_len][offset u32 BE]` | `endpoint_handler(id, EP_OP_READ, …)`，按 `offset/req_len` 切片（Classic 每帧 ≤4 B） |
| `0x21` 写 | `[flags][id u16 BE][data_len][value]`（整帧 ≥8 B，SDK 补零） | `endpoint_handler(id, EP_OP_WRITE, …)`，成功回 8 字节 ACK |
| `0x24/0x25` 描述符 | 元数据 `[00 00][total u32 LE][crc u16 LE]`，数据 `[chunkOffset u16 LE][≤6 B]` | 流式发送 `endpoint_json`，≤50 帧/ms（约 9 KB ≈ 1550 帧） |

已知不兼容：`jsdk_context_configure()` 会先做 `0x00` MIT 握手并解析 7 个 ODrive
专有路径（`axis0.motor.config.gear_ratio` 等）。先用底层
`jsdk_context_desc_fetch()` + `jsdk_joint_param_get/set`；SDK 侧的适配由
SDK 项目解决，不要为它改固件参数命名。

移植清单：扩展帧过滤器 → `0x20/0x21`（带 op）→ `0x24/0x25` 流式状态机 →
回归（SDK 解析描述符；读写所有 access 组合；`action.*` 的读必须返回
`EP_ERR_WRITE_ONLY`）→ 更新本文档。`can_tx_rx` 在 ISR 里，动作只置标志。

## 测试

自动检查：`make check`（生成物/文档最新）、`make hosttest`（`prefs_codec`）、
`make`（链接）。Windows 用 MSYS2 bash 构建（Chocolatey 的 make 用 cmd，
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
