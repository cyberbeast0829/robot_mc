# AGENTS.md

Guidance for AI coding agents working in this repository.

## What this is

Bare-metal C firmware (no RTOS) for a 3-phase FOC motor controller on the
**Nations N32H473CE** (Cortex-M4). It is a fork/port of Ben Katz's mini-cheetah
`motorcontrol` firmware (git remote `originG` = `github.com/bgkatz/motorcontrol`)
onto the N32H47x standard-peripheral SDK. The team's fork lives at the `origin` remote,
active branch `mit_h473`.

**This hardware has no DRV8323 gate driver, and the upstream DRV8323 support has been
removed.** The DRV8323 SPI setup/control is commented out in `main.c`, and the power-stage
gate drive is now enabled/disabled directly through the **ATIM1 break/MOE bits**
(`ATIM1->BKDT |= TIM_BKDT_AOEN | TIM_BKDT_MOEN` to enable, `&= ~` to disable — see
`fsm_enter_state`/`fsm_exit_state` in `fsm.c`). Do **not** reintroduce DRV8323 code.

`CLAUDE.md` (repo root) is inherited from the STM32F446 upstream and is the best
description of the **control architecture** (FSM, FOC, flash config, watchdog, CAN).
**Its hardware/build specifics are stale for this fork** — read the corrections below
first (that also means its DRV8323 and "SPI1 shared" statements no longer apply). Prefer
keeping fork-specific corrections in this file (upstream merges will not touch it) rather
than editing `CLAUDE.md`, which is overwritten by upstream merges.

## Build

Two equivalent builds produce the same firmware:

- **GNU Make (command line)**: `make` (needs `arm-none-eabi-gcc` on PATH; override the
  directory with `GCC_PATH=`). It uses the Nations-supplied GNU startup
  `Drivers/CMSIS/device/startup/startup_n32h47x_48x_gcc.S` (capital `.S` — the Makefile has
  a `.S` rule) and `N32H473CE_FLASH.ld` (mirrors `Keil/ODrive_N32.sct`, including the
  16-byte NO_INIT region that holds `_reboot_cookie`). Defines:
  `N32H473, ARM_MATH_CM4, HSE_VALUE=24000000, __FPU_PRESENT=1`. Also:
  `make DEBUG=1` (-Og/-g3), `make size`, `make clean`, and `make flash` (J-Link; override
  with `JLINK_DEVICE`/`JLINK_IF`/`JLINK_SPEED`; `flash-pyocd` / `flash-openocd` also exist).
  `make check` 校验生成表/文档是否最新；`make hosttest` 在主机上跑 `prefs_codec` 单测。
  ⚠ The startup vector table must match the device IRQ numbering — a mismatch silently
  misroutes every ISR (keep it in sync if you ever edit it).
- **Keil MDK (uVision)**: project `Keil/mit_h473.uvprojx`, target `mit_h473`, device
  `N32H473CE`, **AC6** compiler, scatter file `Keil/ODrive_N32.sct`. Include paths:
  `Core/Inc`, `Drivers/CMSIS/{core,device}`,
  `Drivers/n32h47x_48x_std_periph_driver/inc`, `Drivers/n32h47x_48x_usbfsd_driver/inc`.

- ⚠ `Core/Inc/stm32f4xx_hal_conf.h` is a stale upstream leftover (STM32F446) used by
  neither build. `motorcontrol.ioc` is the **STM32F446RETx** CubeMX file — **do not
  regenerate** from it (it would revert the port); `adc.c`/`tim.c` also contain hand edits
  *outside* the `USER CODE` blocks that a regeneration would silently drop.
- Testing: `make check` (generated tables/docs are current) and `make hosttest`
  (`tools/test_prefs_codec.c`, no target hardware) are the only automated checks.
  Everything else is verified by a clean build + bench testing by the maintainer.

## 文档

`docs/` 只有三份文档，本文件（`AGENTS.md`）是给 agent 的短契约，不要互相复制
长文，改接口时同时更新：

- `docs/README.md`：入口、版本矩阵、五分钟上手；
- `docs/USER.md`：用户手册（参数读写、标定、备份、OTA、故障排查、FAQ）；
- `docs/DEV.md`：开发者手册（架构、endpoint 协议、`params.json`/codegen、
  TLV 持久化、Flash/OTA 契约、CyberBeast/CAN、测试、发布检查表、变更规则）；
- `docs/parameters.md`：由 `make endpoints` 生成，**不要手改**。

不要再拆成 `docs/dev/*`、`docs/user/*` 的多文件结构——文档数量本身就是维护成本。

## SDK/API differences from upstream (important)

Use the Nations **standard peripheral driver** (`Drivers/n32h47x_48x_std_periph_driver`),
**not** ST HAL. In-tree examples: `GPIO_WriteBits`, `TIM_ClrIntPendingBit`,
`USART_GetIntStatus`/`USART_GetFlagStatus`, `ADC_CalibrationOperation`,
`FDCAN_GetRxMsg`/`FDCAN_Start`, `IWDG_ReloadKey`/`IWDG_WriteConfig`.
The PWM/advanced timer is `ATIM1` (aliased `TIM_PWM` in `hw_config.h`); FOC writes
`TIM_PWM->CCDAT1/2/3` directly against period `TIM_PWM->AR` (see `foc.c`).

The three phase-current shunts are **5 mOhm in the low-side legs**, confirmed by
the maintainer. `I_SCALE` is nominally `0.005035400390625 A/count` for the current
ADC/PGA configuration; the previous `0.050354...` coefficient assumed 0.5 mOhm
and over-reported current by 10x. The board current limit is conservatively
capped at 4 A (6 A software OC trip). Do not restore the old 40 A limit or alter
the PGA gain without verifying the analogue range and actual phase current.

⚠ **Never assume an `HAL_*` / `stm32f4xx_*` / `TIM1->BDTR` / `.Instance` reference resolves
here** — verify it against the N32 SDK. The incomplete-port leftovers have been cleaned up
(the `HAL_GPIO_WritePin` / `drv_disable_gd` / `TIM1->BDTR` calls; `foc.c`'s `__HAL_ADC_*` →
`ADC_GetFlagStatus`/`ADC_ClearFlag` on `ADC1` with `ADC_FLAG_JENDC`; the CAN TX
`HAL_CAN_AddTxMessage` → `FDCAN_AddMsgToTxFifoQ(FDCAN2, …)`; `can.c`'s `CAN_H.Instance->ESR`
→ `FDCAN2->ECR`/`FDCAN2->PSR`; and `fsm.c`'s `HAL_Delay`). `preference_writer.c` is now on the
N32 flash API — 8 KB page erase + 64-bit `FLASH_ProgramdoubleWord` and a software CRC-32 — in
the top two pages `0x0807C000`/`0x0807E000`. The payload is an **ID-keyed TLV record table**
(`prefs_codec.c`), not the old raw array; see "配置持久化" below. Only commented-out ST
calls remain (`main.c`, `foc.c`, `it.c`).

## Real-time architecture (full description in `CLAUDE.md`)

Same shape as upstream, with these N32 names:

- **`ATIM1_UP_IRQHandler`** (`Core/Src/n32h47x_48x_it.c`, priority 0), every PWM cycle:
  clear update flag → kick IWDG (`IWDG->KR = 0xAAAA`) → `analog_sample` → `ps_sample`
  → `check_faults` → `run_fsm` → `can_tx_rx`.
- **Main loop** calls `run_fsm_slow(&state)` — all printing, setup/zero command handling,
  and flash saves. ISR→main handoff is via the `volatile *_pending` flags in `FSMStruct`
  (`fsm.h`).
- `USART3_IRQHandler` → `update_fsm`; `USB_FS_LP_IRQHandler` → `USB_Istr`; USB CDC RX also
  feeds `update_fsm` through `usb_rx_process_packet`.
- Control loop is 40 kHz (`DT = 25 µs` in `hw_config.h`).

ATIM1 update/TRGO and the control ISR are fixed at the PWM **peak**, inside the
low-side sampling window. Preserve `tim.c`'s startup sequence: load the internal
repetition counter with 0 using UG, then write `REPCNT=1` without another UG.
Another software update can change the sampling phase. All three injected ADCs
use `ATIM1_TRGO`; there is no separate OC4/TRGO2 sampling path.

Invariants carried from upstream — keep them:

- **Never** `printf`, write flash, or do blocking SPI in the ISR path (`run_fsm` and below).
- Flash saves go through `save_preferences()` in `fsm.c` (kicks the watchdog first, and
  only runs with the gate drive off); the page erase stalls the CPU.
- The IWDG (~4 s) is kicked by the control ISR — don't shorten it or add longer stalls.
- The commutation encoder is the only SPI user: it is read over **SPI1 + DMA2** from inside
  the ISR (`ps_sample` in `position_sensor.c`, which spin-waits on the DMA flags). Don't add
  SPI work to the main loop without accounting for that.
- `fault_shutdown()` (in `n32h47x_48x_it.c`) must stay register-writes-only: gate drive off
  via the ATIM1 BKDT bits (never SPI, never assume the bus is usable).

## Custom printf

`myprintf()` (declared in `main.h`, defined in `main.c`) replaces `printf`: it sends over
USB CDC when `bDeviceState == CONFIGURED`, otherwise it polls USART3. It **can block**
(it busy-waits on packet completion with timeouts) → main loop only, never in the ISR.

## CAN

`can.h`'s comment table is the spec of record; the controller is `FDCAN2`. ID =
`(opcode << 7) | node_id`. Floats on the wire are float32 little-endian. All externally
supplied floats (CAN, serial) must be clamped and NaN-checked before reaching the control loop.

## Endpoint 参数访问层（fibre 风格）

用 16 位 endpoint ID 访问任意参数/遥测/动作，灵感来自 ODrive 的 fibre：

- **唯一真源是仓库根的 `params.json`**。每个条目描述名字、类型、读写权限、
  C 引用（`ref`/`get`/`set`/`call`）、上下限和可选的 `require` 前置条件。
- 改完 JSON 后运行 **`make endpoints`**（或 `python tools/gen_endpoints.py`），
  生成 `Core/Inc/endpoint_defs.h`/`Core/Src/endpoint_defs.c`、
  `Core/Inc/prefs_defs.h`/`Core/Src/prefs_defs.c`，以及 `docs/parameters.md`。
  生成文件**提交进仓库**（Keil 构建不依赖 Python），不要手工编辑它们；
  **`make check`**（= `make endpoints-check`）检查它们是否过期——改了 JSON
  却忘了重新生成，描述符和固件就会静默错配，提交前必须跑。
- **endpoint ID 由 `tools/endpoint_ids.json` 锁定，只增不改**。改名等于新增
  （旧 ID 保留但不响应）；不要手工重排 ID。
- 设备把一份紧凑 JSON 描述符烧进 flash（当前约 9.3 KB），通过 **endpoint 0**
  分块下载；描述符的 CRC16 同时是“接口版本”（每次增删参数都会变），除
  endpoint 0 外的每个请求都必须在 payload 末尾带这个 CRC，否则返回
  `EP_ERR_VERSION`，避免新旧固件静默错配。
- **读和写必须显式区分**：请求 `id` 字段的 bit14 是 `EP_REQ_WRITE`，bit15 是
  `EP_REQ_REPLY`，低 14 位是 ID。handler 按描述符的 `access` 校验（`r` 拒绝写、
  `w` 拒绝读）。**不要**退回“payload 为空就当读”的写法：那样一次读
  `action.enter_menu` 就会停机。USB 侧由 `mc_ep.py` 置位；CAN 侧由
  `PARAM_READ(0x20)`/`PARAM_WRITE(0x21)` 操作码映射。协议版本已升到 2。
- 传输目前只有 **USB CDC**：二进制帧和 ASCII 菜单共用同一条数据流，
  `0xAA` 帧头之外的字节仍然交给 `update_fsm()`。帧格式和状态码见
  `Core/Inc/endpoint.h`。
- **实时性**：USB 中断里的 `endpoint_serial_feed()` 只做帧累积（含一次
  最多 64 字节的 CRC）；解析、调用 handler、应答都在主循环的
  `endpoint_serial_poll()` 里。不要在 handler 里 printf、写 flash 或阻塞。
- **写入安全**：生成的代码统一做 NaN/Inf 检查和 JSON `min`/`max` 限幅；
  `require` 为假时返回 `EP_ERR_STATE`（例如 `config.ppairs` 要求门极关闭）。
  动作端点（`action.*`）只置 `state.save_pending` / `state.zero_pending` 等
  标志，flash 擦写仍由 `run_fsm_slow()` 在门极关闭时完成。`action.save` 返回
  `EP_OK` 只表示“已受理”，结果读 `system.save_status`（1 成功 / 2 因门极使能
  跳过 / 3 写失败）或看 `system.prefs_sequence` 是否增加。
  状态切换用 `request_fsm_state()`（带临界区），不要在 handler 里直接调
  `update_fsm()`——后者是串口/CAN 输入路径。
- 上位机：`python tools/mc_ep.py --port COMx tree|get|set|call|dump|monitor`
  （需要 `pip install pyserial`）。工具会先下载描述符，再按名字收发。
- **线上描述符必须是纯 ASCII，且类型名必须是 ODrive/Fibre 认得的**：
  `params.json` 里的中文 `brief` 只用于文档，**不进**设备描述符（OM/MS 端的
  JointSDK 轻量解析器不支持 `\uXXXX`，会整体解析失败）；动作端点在
  `params.json` 里写作 `void`，生成到描述符时必须变成
  `"type":"function"`（带空 `inputs`/`outputs`），否则同一解析器会报
  “unknown endpoint type”。这两点由 `gen_endpoints.py` 的 `descriptor_json()`
  保证，改它时不要退回去。

## 与 JointSDK（CyberBeast）的兼容性

用 JointSDK 自己的 `cb_jsondesc_parse.c` 验证过描述符（新增端点后需在 SDK 环境
重跑一次）：118 个可见端点、ID/类型/权限全部正确。endpoint 层的约定与
CyberBeast 完全一致：u16 ID、参数值小端、`bool/uint8/int32/uint32/float`、
`r/w/rw` 权限、数组根节点、描述符总长 < 64 KB。
把 CyberBeast 移植过来时，`0x20/0x21/0x24/0x25` 四条消息直接复用
`endpoint_handler()` / `endpoint_json` / `endpoint_json_crc` 即可，不需要改参数表；
`0x20` 传 `EP_OP_READ`、`0x21` 传 `EP_OP_WRITE`。

仍属**传输/协议层**的工作（与 endpoint 无关）：

- CyberBeast 用 **29-bit 扩展帧**，本工程现有协议用 11-bit 标准帧
  `(opcode<<7)|node_id`；需要增加扩展帧过滤器，两套协议可以共存。
- `0x20` 读：`[flags][id u16 BE][req_len][offset u32 BE]`，Classic 下每帧最多
  4 字节；`0x21` 写：`[flags][id u16 BE][data_len][value]`，**整帧必须 ≥ 8 字节**
  （SDK 会补零），成功后回 8 字节确认。值是小端。
- `0x24/0x25` 描述符：一次请求后设备自主流式发送，先发元数据帧
  `[00 00][total u32 LE][crc u16 LE]`，再发 `[chunkOffset u16 LE][6 字节数据]`
  （Classic），每毫秒 ≤50 帧。当前描述符约 9.3 KB（约 1550 帧）。
- **高层 `jsdk_context_configure()` 仍不兼容**：它会先做握手（等 `0x00` MIT
  响应）并解析 `axis0.motor.config.gear_ratio` 等 7 个 ODrive 专有路径。
  只用 endpoint 访问时，改用 `jsdk_context_desc_fetch()` + `jsdk_joint_param_get/set`；
  要让 `configure()` 通过，需要额外加 ODrive 路径别名和状态枚举映射。

## 配置持久化（ID-keyed TLV）

`preference_writer.c` 的 payload 是 **ID-keyed TLV**（编解码在 `prefs_codec.c`），
不再是 `int[256] + float[64]` 的裸数组：

- 记录：`{id u16, type u8, flags u8, len u16, reserved u16, value}`，值小端、
  按 4 字节对齐。块头：`magic 'PCFG'`、`schema`、`sequence`、`payload_len`、
  `record_count`、`CRC-32/MPEG-2`。
- **记录的 key 是 endpoint ID，不是 RAM 数组下标**。重排 `__float_reg`/
  `__int_reg` 不会影响已存配置；新增参数也不需要在固件里写迁移代码。
- **新增参数**：在 `params.json` 里加 `persist: true` 和 `default`，运行
  `make endpoints`。生成器会更新 `prefs_entries[]`、
  `config_apply_defaults()`、`config_apply_limits()`。旧块里没有这条记录 →
  保持默认值，其余记录不动。
- **容量是生成期检查的**：单条 `≤ PREFS_MAX_VALUE(512)`、总载荷
  `≤ PREFS_MAX_PAYLOAD(4096)`、条数 `≤ PREFS_MAX_RECORDS(256)`。超限时
  `prefs_block_build()` 会静默跳过条目（例如 LUT 变大 → 标定悄悄不保存），
  所以生成器直接报错；`prefs_defs.c` 里另有 `_Static_assert` 把
  `internal.encoder_lut` 的 `size` 和 `N_LUT` 绑死。
- **上电顺序**（`main.c`）：`config_apply_defaults()` →
  `preference_writer_load()` → `config_apply_limits()`。默认值只在
  `params.json` 维护，不要再往 `main.c` 里加 `if(isnan(...))`。
- **未知记录透传**：保存时把旧块里当前固件不认识的记录原样带到新块。
  删除后又加回的同名参数（注册表恢复同一 ID）能拿回旧值。
- `ENCODER_LUT` 是一条 `hidden` blob 记录（512 B），只进持久化表，**不进设备
  描述符**（否则 SDK 解析器会遇到未知类型）。
- **ID 永久**：绝不重用、绝不改类型/单位/含义。改名用注册表别名/同一 ID；
  语义变化用新 ID + 一次性迁移（`PREFS_SCHEMA` +1，迁移函数在
  `prefs_codec.c`/`preference_writer.c` 里加）。
- **存储区**：`0x0807C000` / `0x0807E000` 两个 8 KB 页 ping-pong。链接脚本
  （`N32H473CE_FLASH.ld`）和 Keil scatter（`ODrive_N32.sct`）的 app 区已收紧到
  `0x7C000`，镜像不可能长进配置页。**bootloader 必须只擦 app 区**；整片擦除
  会清掉配置，任何软件格式都救不了。这是 OTA 保留配置的唯一硬前提，发布前
  必须做一次“设参数 + 标定 → OTA → 读回”的闭环验证（见 `docs/DEV.md` 的
  “Flash 布局与 OTA 契约”“发布检查表”）。
- **保存**：仍只由 `save_preferences()` 在门极关闭时执行；序列化在关中断
  之前完成，关中断只覆盖擦除 + 编程。掉电时另一页仍是完整旧块。
- 产测/OTA 前后可用 `system.config_schema`、`system.prefs_sequence`、
  `system.prefs_loaded` 核对；描述符里带 `persist: true` 的端点可做
  主机侧 backup/restore。
- 本工程尚未交付客户，因此**不兼容**旧的 `'CONF'` 裸数组块：首次刷入 TLV
  固件会回到默认值，需要重新设置/标定一次；之后的所有升级都会保留。

## Conventions

- Indentation is **tabs**; match the existing compact style, commenting only where hardware
  behavior isn't obvious.
- **Chinese comments and user-facing strings appear throughout the codebase** — keep them,
  and feel free to write new comments in the same language as the surrounding code.
- Global singletons (`controller`, `observer`, `state`, `comm_encoder`, `comm_encoder_cal`,
  `can_tx/rx`, `prefs`) are defined in `main.c` and `extern`'d via `structs.h`. (The upstream
  `drv` singleton was removed together with the DRV8323.)
- Config params in `user_config.h` alias `__float_reg[]`/`__int_reg[]`; `ENCODER_LUT` is
  `__int_reg[6..133]`; `VERSION_NUM` lives in `main.h` (so CAN can report it).
- Keil/AC6 `NO_INIT` trick: `__attribute__((section(".bss.NO_INIT")))` (used for
  `_reboot_cookie`) must stay in sync with the scatter file `Keil/ODrive_N32.sct`.
