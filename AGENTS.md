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

**The image runs behind a 32 KB OpenBootloader** (separate repo, not built here): the
bootloader owns `0x08000000..0x08007FFF`, waits 200 ms for a CAN `SYNC` (`0x128`,
500 kbit/s Classic) and otherwise jumps to the application at **`0x08008000`**
(`VECT_TAB_OFFSET = 0x8000`). To stay in the bootloader, the app writes
`BOOTLOADER_MAGIC_VALUE` (`0xB00710AD`) into `_reboot_cookie` (NO_INIT at `0x20023FF0`,
the address the bootloader polls) and resets — `action.enter_bootloader_mode` / menu `b`.
Bootloader and app must both keep the IWDG fed (see "配置持久化" below).
OTA flashing is done with the in-repo `tools/blcan.py` (usage:
`tools/blcan_使用说明.md`; protocol/contract: `docs/DEV.md`).

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
  a `.S` rule) and `N32H473CE_FLASH.ld` (mirrors `Keil/ODrive_N32.sct`: app at
  `0x08008000`, length `0x74000`, plus the 16-byte NO_INIT region at `0x20023FF0`
  that holds `_reboot_cookie`). Defines:
  `N32H473, ARM_MATH_CM4, HSE_VALUE=24000000, __FPU_PRESENT=1`. Also:
  `make DEBUG=1` (-Og/-g3), `make size`, `make clean`, and `make flash` (J-Link; override
  with `JLINK_DEVICE`/`JLINK_IF`/`JLINK_SPEED`; `flash-pyocd` / `flash-openocd` also exist).
  `make check` 校验生成表/文档是否最新；`make hosttest` 在主机上跑 `prefs_codec`、
  CAN codec、`motion_law`、`dual_gear`、`cogging_law`、`mtpa_law` 和
  `endpoint_process()` 单测；
  `make hosttest-py` 跑 `tools/test_mc_ep_can.py`（上位机 CAN 传输，python-can 的
  virtual 总线，不需要硬件；需要 `pip install python-can`）；
  `make hosttest-rmc` 跑 `tools/rmc/tests`（rmc 包的属性树/补全/backup 测试，
  需要 `pip install pytest python-can`）。
  ⚠ The startup vector table must match the device IRQ numbering — a mismatch silently
  misroutes every ISR (keep it in sync if you ever edit it).
- **Keil MDK (uVision)**: project `Keil/mit_h473.uvprojx`, target `mit_h473`, device
  `N32H473CE`, **AC6** compiler, scatter file `Keil/ODrive_N32.sct`. Include paths:
  `Core/Inc`, `Drivers/CMSIS/{core,device}`,
  `Drivers/n32h47x_48x_std_periph_driver/inc`, `Drivers/n32h47x_48x_usbfsd_driver/inc`.

- ⚠ `adc.c`/`tim.c` contain hand edits *outside* the `USER CODE` blocks that a CubeMX
  regeneration would silently drop. The STM32F446-era CubeMX/CubeIDE files
  (`motorcontrol.ioc`, `.cproject`, `.project`, `.mxproject`), the upstream
  `STM32F446RETX_{FLASH,RAM}.ld` linker scripts, `Core/Inc/stm32f4xx_hal_conf.h` and the
  IAR startup `startup_n32h473_EWARM.s` were **deleted** in this fork — neither build uses
  them; don't restore them from upstream merges.
- ⚠⚠ **The two builds are NOT equivalent on hardware** (2026-10-26 bench): the `make`
  (GNU/`arm-none-eabi-gcc`) image boots and talks CAN, but the commutation encoder reads a
  constant 0 (`telemetry.encoder_raw == 0`, `elec_angle` frozen, `lost_frames == 0`) with the
  same sources that work when flashed from Keil. Root cause **unknown / not investigated**.
  Bench-verify with the **Keil** image (`Keil/Objects/mit_h473.axf`) until this is fixed, and
  don't conclude "the firmware is broken" from a `make` image symptom.
- Testing: `make check` (generated tables/docs are current) and `make hosttest`
  (`tools/test_{prefs_codec,can_cyberbeast,motion_law,dual_gear,cogging,scope_law,endpoint_proto}.c`,
  no target hardware) are the only automated checks; `make hosttest-py`
  (`tools/test_mc_ep_can.py`, needs `python-can`, fake device on a virtual bus)
  covers the host tool's CAN transport; `make hosttest-rmc`
  (`tools/rmc/tests`, needs `pytest`) covers the `rmc` package (proxy tree,
  tab completion, backup/restore, CLI parsing).
  Everything else is verified by a clean build + bench testing by the maintainer
  (`tools/cb_bench.py` drives the CYBERBEAST platform over slcan: `--stage-a/-b/-c/-c-static/-d`,
  `--reset-test`, `--timeout-test`; `tools/csv_csp_test.py` drives the velocity/position
  loops over USB: `plain|csv|csp|timeout|sweep`; `tools/gear_test.py` verifies the dual-gear
  absolute position over USB CDC: `check|verify|powercycle|capture|zero|ratio`;
  `tools/blcan.py` drives the OpenBootloader over CAN for OTA: `sync|version|id|read|write|
  erase|go|upgrade` — usage in `tools/blcan_使用说明.md`).
- ⚠ **Flashing this board is unreliable at high SWD clocks** (bench, 2026-10-30): reads and
  programming silently corrupt at 5 MHz (Keil default) and even 1 MHz, J-Link's own
  `Verify` fails at random addresses, and the device then keeps running the **old image** —
  which looks like "the new endpoint isn't there / telemetry contradicts itself". Always
  flash with `python tools/flash_jlink.py` (250 kHz, 8 KB chunks, per-chunk read-back
  compare + retry, full-image verify, then `r`+`g`; writes the app to `0x08008000`,
  `--base 0x08000000` flashes the bootloader; `--erase` clears the **whole** chip —
  bootloader and config pages included).
  After flashing, sanity-check `system.prefs_loaded` / `config.kt` to confirm the new image
  really is running; after a mass erase re-run `action.calibrate` and `action.save`.

## 文档

`docs/` 只有三份文档，本文件（`AGENTS.md`）是给 agent 的短契约，不要互相复制
长文，改接口时同时更新：

- `docs/README.md`：入口、版本矩阵、五分钟上手；
- `docs/USER.md`：用户手册（参数读写、标定、备份、OTA、故障排查、FAQ）；
- `docs/DEV.md`：开发者手册（架构、endpoint 协议、`params.json`/codegen、
  TLV 持久化、Flash/OTA 契约、CyberBeast/CAN、测试、发布检查表、变更规则）；
- `docs/parameters.md`：由 `make endpoints` 生成，**不要手改**。
- `tools/rmc/README.md`：上位机包 `rmc`（PyPI **`robotmc`**）的安装、命令、shell、
  backup/restore 与发布流程（不占 `docs/` 的三份名额）。`tools/mc_ep.py` 只是
  转发到它的弃用 shim。

不要再拆成 `docs/dev/*`、`docs/user/*` 的多文件结构——文档数量本身就是维护成本。

## SDK/API differences from upstream (important)

Use the Nations **standard peripheral driver** (`Drivers/n32h47x_48x_std_periph_driver`),
**not** ST HAL. In-tree examples: `GPIO_WriteBits`, `TIM_ClrIntPendingBit`,
`USART_GetIntStatus`/`USART_GetFlagStatus`, `ADC_CalibrationOperation`,
`FDCAN_GetRxMsg`/`FDCAN_Start`, `IWDG_ReloadKey`/`IWDG_WriteConfig`.
The PWM/advanced timer is `ATIM1` (aliased `TIM_PWM` in `hw_config.h`); FOC writes
`TIM_PWM->CCDAT1/2/3` directly against period `TIM_PWM->AR` (see `foc.c`).

The three phase-current shunts are **5 mOhm in the low-side legs**, confirmed by
the maintainer. `I_SCALE` is a **formula** in `hw_config.h`, not a literal:

```
I_SCALE = (ADC_DIFF_SPAN / 2^12) / (R_SHUNT * PGA_GAIN)   // ADC_DIFF_SPAN = 3.3 V (PGA out +-VREF/2, 2048 = 0 A)
```

`PGA_GAIN` is the **actual** PGA differential gain, i.e. half of the `<n>` in
`pga.c`'s `PGA_CTRL_CH1GAIN_<n>_DIFF_<2n>`: `32_DIFF_64` → 16, `24_DIFF_48` → 12.
It is derived from `PGA_GAIN_CODE` in `hw_config.h` (the gain field: 7 = 32_DIFF_64,
6 = 24_DIFF_48), because the code ↔ gain table is not a formula.
This board (5 mOhm + gain 12) gives `0.013427734375 A/count`, i.e. +-2048 counts =
**±27.5 A**; the old gain-16 board was `0.01007080078125` (`±20.6 A`). The earlier
`0.005035400390625` value halved the span (the ADC then saturated at half the PGA
range), so gains tuned against it were tuned on a 2x-low current reading; the
pre-fork `0.050354...` assumed 0.5 mOhm and over-reported current by 10x.
Changing the gain means editing `pga.c` **and** `PGA_GAIN_CODE` together — `pga.c`
has a `_Static_assert` on it so they cannot drift apart. The board current limit
is conservatively capped (`I_MAX_LIMIT` = 12 A → 18 A software OC trip,
~65 % of the 2048-count span); keep
`I_OC_FAULT_FACTOR * I_MAX_LIMIT` below the ±27.5 A saturation point or a
saturated reading silently masks the overcurrent fault. Do not restore the old
40 A limit or alter the PGA gain without verifying the analogue range and actual
phase current.

**Low-side sampling window** (`Core/Inc/phase_current_law.h`, called from
`analog_sample()`): the injected ADCs sample at the PWM peak, so a phase whose
duty approaches the modulation limit has the shortest low-side conduction
window and the least trustworthy shunt reading — that one phase is rebuilt from
the other two (`i_x = -i_y - i_z`, same as ODrive's `fetch_and_reset_adcs()`).
PWM1 makes the low side the complement of CHx (it conducts only while CNT > CCR),
so the window at the peak is `AR - CCR` — the **polarity parameter is not needed**
(`INVERT_DTC` only changes how `set_dtc()` derives CCR from duty) — and with
`INVERT_DTC = 1` the window is ∝ `dtc`: the phase with the **smallest duty** (the
one min-max injection pins to the positive rail) is the bad one. ⚠ ODrive's board
uses **PWM mode 2**, so their "smallest CCR is bad" rule inverts here.
It engages when the window falls below `I_SAMPLE_MIN_LOW_FRAC` (15 % of the period);
at `DTC_MAX = 0.9` full modulation leaves the bottom phase ~50 ticks, so the guard
is active near the voltage limit. Keep it cheap in the ISR: the check runs
*before* the `PHASE_ORDER` swap on the physical pairing (i_a ↔ CCDAT1, i_b ↔
CCDAT2, i_c ↔ CCDAT3) and the threshold `i_min_low_ticks` is converted once in
`init_controller_params()`, so the common path is 3 loads + 3 subtracts + 3 compares.
`DTC_MAX` is also inside `V_PER_CMD`: raising it scales `v_max` (+28.6 % going
0.7 → 0.9) and shortens the bottom phase's window, so recheck the calibration path
(`CAL_V_PER_CMD` follows automatically) and the `K_SCALE` fallback together.
`controller.i_recon` / `i_recon_count` report it to a debugger; `make hosttest`
covers the logic.

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

⚠ **Always call `DMA_StructInit(&x)` before filling a `DMA_InitType`** — `DMA_Init()` also
programs the three `BURST_*` fields (into `CHCFG`'s BLEN/BURST_DYNAMIC bits and into the
whole-controller `DMAy->BURSTEN`). `spi.c` used to skip it, so those came from stale stack
(SRAM contents from the previous run) and could silently open burst mode: a 1-halfword SPI
frame then never satisfies a burst, `TXNUM` stays 1, the transfer-complete flag never sets,
and `ps_warmup` spun forever at `position_sensor.c` (`DMA_FLAG_TC2`) before the FSM started.
That also means "the encoder reads 0 / DMA never finishes" symptoms can differ between the
GNU and Keil images and between power cycles **of the same source**; check the burst fields
and `DMA2->BURSTEN` (0x400204A8) before blaming the encoder. `adc.c` has always done it right.

⚠ **The general-purpose ADC channels (`adc_measurements_[]`) must actually be trigger-driven.**
`main.c` sets every regular *and* injected external trigger to SOFTWARE while the ADCs are
calibrated, then restores them: injected on all three ADCs, regular on **ADC2 (MOS_TEMP) and
ADC3 (VBUS + MOT_TEMP)**. Missing the ADC3 regular restore left VBUS on a software trigger
nobody started, so `adc_measurements_[1]` stayed 0 → `controller.v_bus = 0` → `svm()` had no
scale and clamped all three duties to the midpoint, i.e. **the power stage output nothing at
all** and `action.calibrate` failed with "measured 0 pole pairs". Diagnose with
`telemetry.v_bus` (the boot banner prints `VBUS: x.x V` too): if it reads 0 the motor cannot
turn in any mode, whatever else the symptoms suggest. ADC channel map in use:
ADC2_CH2/PA7 = MOS_TEMP → `adc_measurements_[0]`, ADC3_CH4/PE7 = VBUS → `[1]`,
ADC3_CH6/PE8 = MOT_TEMP → `[2]`.

## Real-time architecture (full description in `CLAUDE.md`)

Same shape as upstream, with these N32 names:

- **`ATIM1_UP_IRQHandler`** (`Core/Src/n32h47x_48x_it.c`, priority 0), every PWM cycle:
  clear update flag → kick IWDG (`IWDG->KR = 0xAAAA`) → `analog_sample` → `ps_sample`
  → `check_faults`   → `run_fsm` → `can_tx_rx` (25 µs 时基 + 最多 2 帧 → 协议分发)。
- **Main loop** calls `endpoint_serial_poll()` → `endpoint_usb_poll()` → `can_proto_poll()`
  → `run_fsm_slow(&state)` — all printing, setup/zero command handling, and flash saves.
  ISR→main handoff is via the `volatile *_pending` flags in `FSMStruct` (`fsm.h`).
- `USART3_IRQHandler` → `update_fsm`; `USB_FS_LP_IRQHandler` → `USB_Istr`; USB CDC RX also
  feeds `update_fsm` through `usb_rx_process_packet`.
- Control loop is 40 kHz (`DT = 25 µs` in `hw_config.h`).
- **MOTOR_MODE 的控制律**由 `motion_control()`（`motion.c`）分发：`CTRL_IMPEDANCE`
  原样调 `torque_control()` 且**每拍都算**（MIT / 旧协议 / 0x03 / 0x04）；
  `CTRL_VELOCITY`(CSV) 与 `CTRL_POSITION`(CSP) 是串级外环，按
  `config.vel_loop_prescaler` / `pos_loop_prescaler` 分频（默认 8 kHz / 4 kHz）。
  外环只决定 `i_q_des` 的参考，**电流环 `commutate()` 不变**；纯数学在
  `motion_law.h`（host 可测）。测速用**自适应窗口最小二乘**（`position_sensor.c`
  的 `ps_update_velocity`，见下一条），不再用 PLL 或 19 拍整数差分。
  ⚠ `field_weaken()` 就地累加 `i_q_des`，所以分频模式每拍都要
  `i_q_des = i_q_ref` 重新赋值，否则弱磁会一拍接一拍叠上去。
  增益来源：`action.measure_inertia` 测 `config.inertia`（一条指令、自动进出电机
  模式），`action.tune_gains` 再按 `config.omega_c` / `config.tune_margin` 算出
  `vel_gain` / `vel_integrator_gain` / `pos_gain` 写进 RAM（`motion_tune_gains()`，
  公式与台架依据见 `docs/DEV.md`）。两个 action 都**不自动保存**。
- **测速（`ps_update_velocity`）不能退回 PLL**：16 位编码器静止时原始计数 σ≈4.3
  count，二阶 PLL 的速度噪声 ∝ bw^1.5（bw=3000 时静止 σ=0.05 rad/s 输出轴 ≈ 0.5 RPM，
  峰峰 3 RPM）—— 客户投诉的"静止 1 RPM 波动"就是它。现在用**两个**自适应窗口的
  位置最小二乘：`velocity`（`config.vel_est_noise`=0.4%，遥测/标定/测量）与
  `velocity_fast`（`config.vel_ctrl_noise`=0.02，只给速度环/位置环）。
  **MIT 阻尼与 `enable_torque_vel_lim` 仍用 `velocity_diff`（19 拍 / 475 us）**：
  最小二乘窗口在静止时是 256 ms，从静止起步的那几十毫秒里阻尼反馈会被读成 0；
  `inertia_step()` 同理仍用 `pll_velocity`（它的阈值和 8 ms 台阶是按 PLL 噪声标的）。
  窗口长度按速度
  （以及**指令值**，否则静止时窗口最长→增益最低→推不动→测速恒 0 的死循环）自适应，
  静止时计数不动 ⇒ 斜率**精确为 0**。PLL 保留但只用于齿槽标定内环。
  ⚠ 窗口越长滞后越大，`motion.c` 用 `ml_lag_gain_scale(OMEGA_C, vel_fast_lag)` 把
  速度环的 kp/ki 同比例压低（`config.vel_sched_en`），低速时不会自激。
  ⚠ 累加量是 int64（`ml_ls_t` 的 c 到 1e12 量级，int32 会溢出成"转起来测速恒为 0"）。
- **低速摩擦**：`config.fric_coulomb` / `fric_viscous` / `fric_eps` 把摩擦前馈进
  i_q（`ml_friction_ff`，参与限幅判断）；`config.dither_amp` / `dither_freq` /
  `dither_v_max` 在低速（0<|v_setpoint|<v_max）叠一个高频小电流正弦，压
  stick-slip（台架 1 RPM：无 dither 25~33% → 0.1 A/300 Hz 时 7~16%）。
  这两个都不改控制环结构，参数为 0 时行为与没有它们时逐位一致。
- **片上捕获只有一个入口（`scope.c`）**，三种模式用 `scope.wave` 选：1 = 阶跃、
  2 = 正弦扫频（都会在电流参考上主动注入，结果走 `telemetry.scope_*`）、
  **3 = PASSIVE**（只观测不注入：按 `scope.decim` 抽样存 raw/count/速度/i_q）。
  三种模式共用 `scope.buf`（1024×4 int16）与 `action.scope_dump`（十六进制、
  表头第 4 个字段 = 有效采样率 = 40000/decim）。测速度纹波/FFT 用 PASSIVE：
  CAN 读一次 endpoint 只有约 190 Hz，Nyquist 连 100 Hz 都不到。台架工具
  `tools/vel_ripple.py`（还会算 raw/count 对匀速直线的**角度域残差**与阶次谱，
  那是判断"纹波是转矩还是测角"的唯一手段）。**不要再加第二个捕获模块**。
  ⚠ `scope.wave` 的 `max` 要写成 3；`scope_poll()` 的"卡住"判据必须按
  `scope.decim` 放大，否则 decim=100 时 PASSIVE 一启动就 ERR(state=4)。
- **中高速纹波已定位到测角，不在控制环**：300~600 RPM 的 1× / 2× 转频分量幅值
  ∝ 转速（0.0028 rad/s per Hz ≈ 0.16° 角度误差），关齿槽补偿、把 `omega_c`
  从 450 抬到 700（kp ×1.6）都改变不了（后者更差）。所以别再靠调 PID / 加补偿
  去压这一段；要动就动编码器线性化表（台架实测 LUT 之后的角度残差比 raw 还大，
  见 `docs/DEV.md` 的"中高速纹波的来源"）。`config.vel_ctrl_noise` 默认 **0.02**
  （0.05→0.02 让 300 RPM 1.95%→1.76%；0.01 反而更差）。
- **ISR 耗时（`timing.*`）**：`timing.total_max`（周期数，1 拍 = 6000 周期）是预算
  判据，`timing.seg_*_max` 是分段；`action.reset_timing` 在工况切换前清零。
  测速的窗口调整、dither、PASSIVE 抽样都会进 `total_max`，改动 ISR 后要复测。
- **电流环内部（`commutate()`）的顺序不能动**：`svm()` → 死区补偿（`dtc_comp.h`，
  参数 `config.dt_comp` / `dt_comp_tol`）→ `set_dtc()`。补偿必须在 `set_dtc()` 之前，
  因为那里按占空比算 `pcl_check_needed`（低边采样窗口判据），它要的是补偿后的值。
  补偿在 `dtc` 空间做，所以**不需要知道 GHS/GLS 与 CHx/CHxN 的对应关系**：
  "dtc 变大 ⇒ i 变大"由电流环自身的正增益保证。换功率板/FET 只重标 `dt_comp`
  （方法见 `docs/DEV.md` 的"死区补偿"），标定路径（`commutate_open_loop()`）刻意不补。
- **输入整形**（`config.input_mode`，编号同 ODrive 的 `InputMode`）只整形喂给控制环的
  设定值，**不选控制环**（那是 `command.mode`）：0 忽略/1 直通/**2 VEL_RAMP（默认）**/
  3 POS_FILTER/5 TRAP_TRAJ/6 TORQUE_RAMP/9 MIT；4/7/8 不支持，按直通。
  - 3/5 是**轨迹发生器**（位置环频率上产出 `(pos, vel)`），轨迹速度**取代**帧间
    前馈 `pos_ff`（叠加会过冲）；它们自己就平滑，**不再叠加** `accel_limit` 的斜率。
  - `ml_trap_*` 是 ODrive `trapTraj.cpp` 的逐行移植；`ml_posf_*` 是 ODrive
    `INPUT_MODE_POS_FILTER` 的二阶跟踪（ki=2·bw、kp=ki²/4，kp·dt≤0.5 自动压增益）。
  - **斜坡/轨迹有专用限值**（对齐 ODrive 的 `vel_ramp_rate` 与 `trap_traj.config.*`）：
    `config.vel_ramp_rate`（>0 时 VEL_RAMP 的加速与减速都用它）、
    `config.trap_vel_limit` / `trap_accel_limit` / `trap_decel_limit`。
    默认全 0 = 沿用 `accel_limit`/`decel_limit`/`v_max`（`ml_limit_or()` /
    `ml_trap_vel()` 负责回退，行为与没有这组参数时逐位一致）。
    ⚠ `trap_vel_limit` 只能收紧（与运行期限速取 min），不能放宽 —— 否则轨迹要求的
    速度位置环给不出，必然触发 `FAULT_POS`。
  - `motion_seed()` / 模式变化时清 `pos_trap.planned` 与 `pos_filter.seeded`，
    下一拍从**实测位置**起步；`zero_commands()` 置 `input_mode_prev = 0xFF` 强制重播种。
  - `config.enable_vel_limit`（默认 true）关掉后 CSV/CSP 与 CYBERBEAST 帧解析都不再
    压到 `v_max`，`command.v_des` 端点也改成固定 ±200 rad/s（否则写不进去）；
    `config.enable_torque_vel_lim`（默认 false）在阻抗/力矩模式按
    `i_q = clamp(i_q, (v_max−v)·vel_gain, (−v_max−v)·vel_gain)` 限速（安培域，量纲天然成立）。
- **齿槽转矩补偿（anti-cogging）**：纯数学在 `Core/Inc/cogging_law.h`（host 测
  `tools/test_cogging.c`），状态机/前馈在 `Core/Src/cogging.c`。
  - 表 `COGGING_LUT[COG_N=1024]`（int16，4096 counts/A）覆盖**一个转子机械周期**，
    索引用 `encoder->count`（线性化后、**未减 `M_ZERO`**）——所以 `action.zero` 不会
    让表失效，换编码器才会。持久化是 `internal.cogging_lut`（hidden blob 2048 B，
    `PREFS_MAX_VALUE` 因此是 4096）。
  - `action.cogging_calibrate` = 自动进电机模式 + **匀速扫两圈**（`config.cogging_method`
    默认 1；`cogging_sweep_ms` 一圈的毫秒数，默认 8000，全程约 20 s）：位置目标匀速
    斜坡，按转子位置把速度环输出（P+I）分箱累积；分箱前过两级 90 Hz 低通压掉速度环
    振铃；换向各丢 0.25 s。主循环合并：两遍各自去均值 + 三次多项式去趋势 → 取平均 →
    坏点插值 → 再整体去均值/去趋势 → 量化，并质检（坏点 > 1/4 或两遍差异 > 0.5×信号
    + 5 mA → 失败并停用补偿，旧表不动）；成功只置 `config.cogging_valid`，
    **不自动保存、不自动打开** `config.cogging_en`。
    - ⚠ `config.cogging_method = 0` 是旧的"逐点停住读积分器"，保留但**不推荐**：
      台架实测 564 s、两遍差异 0.021 A（信号 0.052 A 的 40%，静摩擦所致）。它的
      幅值是对的，但要慢得多。**两种方法存表都必须按"物理格"索引**
      （`cog_bin_of_count()` / `start_bin`）—— hold 法曾经用"从起点数起的点号"
      当索引，整张表被旋转了一个随机的 `k0` 格，补偿相位全错；`tools/test_cogging.c`
      有回归测试。
    - ⚠ 多项式去趋势（`config.cogging_detrend`）是必须的：重力/外载在正反两遍里方向
      相同、抵消不了，而它在一个转子圈里只是慢变趋势（本机 40° 输出角）；齿槽是
      20~120 次/转，不会被误删。
  - 标定用**用户自己的** `pos_gain`/`vel_gain`/`vel_integrator_gain`（所以要先
    `action.tune_gains`）。状态机在 `motion_control()` 里接管（同惯量标定），
    ISR 里 `cogging_apply()` 紧跟 `motion_control()`、**在 `field_weaken()` 之前**
    把前馈加进 `i_q_des`；`zero_commands()` 会 `cogging_abort()`。
  - ⚠ `run_fsm()` 的 CAN 超时 `zero_commands()` 条件里必须保留
    `cogging_cal.state == COG_CAL_IDLE && !controller.cogging_req`，否则标定每拍都被
    清零（`vel_integ` 是测量量）。
  - 台架：`python tools/cogging_test.py --port COMx status|calibrate|dump|on|off|reset`。

- **MTPA（每安培最大转矩，`config.enable_mtpa` 默认关）**：纯数学在
  `Core/Inc/mtpa_law.h`（host 测 `tools/test_mtpa.c`），控制路径是 `foc.c` 的
  `mtpa_apply()`（中断查表）/ `mtpa_service()`（主循环建表）/ `mtpa_on()`。
  - 台架：`python tools/mtpa_test.py --channel COMx status|verify|budget`。
  - 插入点（`fsm.c` MOTOR_MODE）：`cogging_apply()` → **`mtpa_apply()`** →
    `field_weaken()`。必须放在所有转矩来源汇总之后、弱磁之前（弱磁要在 MTPA 的
    Id 上继续加深，不能覆盖它）；`scope_apply()` 更在后面，扫频不受影响。
  - 语义：`i_q_des` 是"SPM 等价"的 q 电流（τ = KT·i_q_des），`mtpa_apply()` 把它
    拆成 `(i_d_des, i_q_des)`，**转矩指令不变**；`mtpa_iq_cmd` 存拆解前的值，
    `field_weaken()` 在 Id 变化后用它按 `Iq = iq_cmd·λ/(λ+ΔL·Id)` 保转矩重算。
  - ⚠ 弱磁与 MTPA 的两个 Id **取更深的那个、不叠加**；`kt_eff = KT+1.5p·ΔL·Id`
    贯通转矩限幅（用上一拍的 kt_eff）/ 机械功率 / `telemetry.torque` / 老协议
    转矩读数。MTPA 关且 Id=0 时 `kt_eff == KT`，下游逐位不变。
  - ⚠ **不要退回 ODrive 4680 的"轨迹 + 一次代入"固定点迭代**：强凸极（ξ>1）会震荡
    （实测 Id 从 −134 跳到 −17.5 A、转矩偏大 25%）。现用幅值参数化的 Newton，
    与凸极强度无关。也**不要去掉有理化**（`q²/(2ΔL(s+λ))`）：小电流时
    `(−λ+√(λ²+q²))` 抵消会把 Id 归零。
  - `mtpa_on()` 在惯量/齿槽标定期间返回 0（那两套标定按 KT 的线性转矩关系整定），
    凸极指数 `i_max·|ΔL|/λ < 0.02` 时主循环里就把 `mtpa_ready` 清 0（SPM 电机不付开销）。
  - 台架实测（MZW55，12 A，ξ=0.136）：`mtpa_gain`=1.0090；`t_ff`±0.3 N·m 时
    `i_d_des`=−0.01357（理论 −0.01344）、`kt_eff/kt`=1.000069（理论 1.000068）、
    **参考转矩 `kt_eff·i_q_des` 相对差 0.000%**、`|I|` 比 0.999924（理论同值）。
    判据用参考侧，不要用 `telemetry.torque`（自由关节上随运动漂 ±5%）。
  - ⚠ **求解在主循环、中断里只查表**：`mtpa_service()`（`run_fsm_slow` 每轮调，参数
    变化才干活）按 `KT/PPAIRS/L_D/L_Q/I_MAX/mtpa_id_frac` 重建一张 **49 点表**
    （双缓冲 + `mtpa_tab_gen` 发布），`mtpa_apply()` 只做一次乘法 + 两次插值。
    第一版在中断里直接解方程，台架实测 `seg_fsm_max` 2551 → 3366（**+815 拍**）、
    `total_max` 6102 **超了 6000**；改查表后是 **+154 拍（+6.2%）**、`total_max` 5374。
    本仓库的取指成本约 2.7 拍/指令，不要在 40 kHz 中断里加几百条指令的数学。
    改完必须 `action.reset_timing` 复测 `timing.total_max`；
    `python tools/mtpa_test.py --channel COMx status|verify|budget|peak` 就是这套检查。
  - **峰值转矩口径**：`i_q_des` 是"SPM 等价"电流，MTPA 在电流圆上还能多给
    `g = mtpa_gain` 倍的转矩。`commutate()` 每拍把**实际**电流上限（含温度降流与
    `torque_lim`）换成 `controller.i_q_peak = mtpa_iq_peak(i_max)` 并更新
    `mtpa_scale_f`；`torque_control()` 钳在 `i_q_peak`、`motion.c` 的 `i_cmd_max`
    乘 `mtpa_iq_scale()` 再用 `i_q_peak` 兜住（未激活时 = `i_max` / 1，逐位不变）。
    物理 `|I|` 仍由 `commutate()` 的 `limit_norm` 按 `i_max` 收口。**外环增益不用重调**
    （`i_q_des → 转矩` 仍是严格的 `KT·GR`）。
  - ⚠ **两个收口细节别写回"想当然"的版本**（review 抓到的，仿真 + host 测试守着）：
    ① `config.torque_lim` 换算成电流要除 **cosφ = |i_q|/|I|**（转矩由 i_q 产生、
    限幅作用在 |I| 上），只除 `kt_eff` 会让交付转矩只剩配置值的 59~92%
    （ξ 越大越差）；② 饱和点要按**实际** `i_max` 收口：表锚在 `I_MAX`，降流到 15%
    时若仍按大圆取 Id，强凸极电机的转矩只有 SPM 的 72%（比不开还差）。
    见 `docs/DEV.md` 的"降流/限幅时的收口"。
  - 台架实测（MZW55，24.4 V，12 A，ξ=0.136，`mtpa_gain`=1.0090）：`t_ff`±0.3 N·m 时
    `i_d_des` = −0.01357（理论 −0.01344）、`kt_eff/kt` = 1.000069（理论 1.000068）、
    **参考转矩 `kt_eff·i_q_des` 相对差 0.000%**、`|I|` 比 0.999924（理论同值）。
    判据用参考侧，不要用 `telemetry.torque`（自由关节上会随运动漂 ±5%）。

- **电流环扫频/带宽实测（scope）**：纯数学在 `Core/Inc/scope_law.h`（host 测
  `tools/test_scope_law.c`），激励/采集/锁相在 `Core/Src/scope.c`。
  - 为什么在固件里：endpoint 读一次是一个往返（100~300 Hz），测不到 762 Hz 的
    穿越点。所以 40 kHz 中断里自己注入、自己采样、自己锁相。
  - `scope_apply()` 在 `field_weaken()` 之后、`commutate()` 之前**改写两个轴的
    i_d_des/i_q_des**（被测轴 = bias+激励，另一轴 = 0），`scope_capture()` 在
    `commutate()` 之后取同一拍；`scope_poll()` 在主循环算结果/导波形/看门狗。
    ⚠ 不要把它挪到 `commutate()` 之后注入 —— 那样量到的是上一拍的参考。
  - 频点强制落在 `40 kHz/2048 = 19.53125 Hz` 的整数倍上（累积窗 2048 拍 = 整数
    周期，锁相无泄漏）；一个频点 64 ms（预滚 512 + 累积 2048）。
  - 判据：闭环 T → 开环 `L = T/(1−T)` → 在 log|L|–log f 上插值读穿越/PM/GM
    （`sl_margins()`）。**不要**在线性幅值上插值：1/f 在 mag–log f 里不是直线，
    粗网格会差 5%。⚠ **相位必须先解卷绕**（`atan2` 给 ±180）：不解卷绕时
    −180° 穿越永远找不到、跨卷绕点插值会把 PM 算成 180~350°（真值可能是负的，
    也就是已经失稳）。解卷绕要求相邻频点相位变化 < 180°（4 kHz / 20 点约 35°）。
  - `telemetry.scope_state == 3` 只在 `scope_finish()` 跑完之后由**主循环**置位
    （中断只置内部 `acc_done`）；别改回"中断直接置 DONE"，主机有概率读到还没算的 0。
  - **台架实测（2026-11-19，本机 d 轴，36.8 V）：穿越 = 0.76~0.78×`i_bw`，
    PM 每 1000 Hz 掉约 10°**：500→390 Hz/84°、1000→776 Hz/79°、
    2000→1544 Hz/68°、3000→2296 Hz/58°、4000→3032 Hz/48°。等效延迟 ~38 µs
    （1.5 拍），所以 PM=45° 在 `i_bw ≈ 4300`。**生产建议 ≤2000**，
    `config.i_bw` 上限 4000 只给台架用。
  - 中断开销（实测）：scope 空闲约 +10 周期；正弦测量期间 `total_max`
    5739 → ~6100（**偶发丢拍 ~0.04%**，基线中断本来就占 96%，不是 scope 的锅）。
  - 空载不锁轴也能测：d 轴注入不出转矩，`telemetry.scope_motion` 实测只有
    1.5~2.3°；q 轴会把转子甩起来，没有夹具不要做。
  - 安全：只在 `gate_drive_enabled` 时注入，离开电机模式/故障由 `scope_poll()`
    自动中止，另有 2 s 硬超时（主机掉线不会一直灌电流）。
  - 台架：`python tools/iloop_bw.py --port COMx bode|sweep|step [--axis d|q]`。
- **速度纹波/FFT（PASSIVE）**：`python tools/vel_ripple.py --rpm 300 --decim 5 --fit 4`
  （控制走 CAN `--interface slcan --channel COMx`，波形走 USB CDC `--port COMy`）。
  `--decim 100`（400 Hz / 2.56 s）看机械阶次，`--decim 5`（8 kHz / 128 ms）看
  宽带到 900 Hz，`--decim 1`（40 kHz）看单拍增量。**报数必须写窗口**（见 `docs/DEV.md`）。

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

## 保护与限幅（P0，2026-11）

阈值全部是参数，**不要往 `hw_config.h` 加回编译期常量**
（`V_BUS_FAULT` 已删除，改成 `config.vbus_ov_trip`）：

- `check_faults()`（ISR）：`FAULT_OV` 用 `config.vbus_ov_trip`（任何状态都查）；
  `FAULT_UV` 用 `config.vbus_uv_trip`，**只在门极使能时**查 + ~10 ms 去抖
  （USB 单独供电时 `v_bus=0`，否则一上电就锁故障把人挡在菜单外）；
  `FAULT_FET_OT`/`FAULT_MOT_OT` 在 `hi + 5 °C` 时锁存（与 ODrive 的 `do_checks()` 一致）。
- `commutate()` 每拍算一次电流上限：`I_MAX` × 温度降流（`[lo,hi]` 线性）× 
  `config.torque_lim/(KT·GR)`，再按估算母线电流 `config.i_dc_max` 回压指令。
  温度保护关闭时**完全不影响** `I_MAX`。
- **NTC 温度在主循环算**（`update_thermistors()`，从 `run_fsm_slow()` 第一件事调用），
  不在 40 kHz 中断里算；`dt` 由 `controller.loop_count` 的差得出（主循环没有固定周期）。
  换算照搬 ODrive `ThermistorCurrentLimiter` 的四阶多项式 + 两级 0.1 s 低通，
  MOS=`adc_measurements_[0]`、电机=`adc_measurements_[2]`。
  ⚠ 系数是 ODrive 板的，CBL89 上台架先核对室温读数，**校正前不要打开
  `fet_temp_en` / `mot_temp_en`**。
- **母线电流/功率是估算值**（没有直流分流器）：`P_elec = 1.5·V_PER_CMD·(v_d·i_d+v_q·i_q)`，
  `i_bus = P_elec/v_bus`，`mech_power = i_q_filt·KT·GR·dtheta_mech`。只能做监控和软限幅，
  **不能当保护**。`V_PER_CMD` 现在在 `hw_config.h`，`calibration.c` 的 `CAL_V_PER_CMD`
  也指向它 —— 改 `svm()`/`DTC_*` 时一处改完即可。
- `config.pre_calibrated` 是**进电机模式的前置条件**：三层门禁
  （endpoint `require` / `cb_start_motor()` / `run_fsm()` 兜底）。标定完成时置 1，
  进标定或 `action.reset_config` 清 0。别为了"方便调试"把它默认设 1。

## 标定

`CALIBRATION_MODE` 的 ISR 链路是 `order_phases()` → `calibrate_encoder()` →
`measure_lr()`（`calibration.c`）。`measure_lr()` 是 ODrive
`measure_phase_resistance` / `measure_phase_inductance` / `measure_ld_lq` /
`measure_flux_linkage` 的移植，用两段直流外推 R、±2 V 翻转测 L、锁住转子测
Ld/Lq、开环升速测磁链并令 `KT = 1.5·PPAIRS·λ`。测试电流 = `min(I_CAL, I_MAX)`。

- `run_fsm_slow()` 在 `done_ordering && done_cal && done_rl` 全为真后先置
  `PRE_CALIBRATED = 1` 再存盘；这一刻才算"标定完成"（见上一节的进模式门禁）。

- 极对数（`order_phases()`）：斜坡转 **`CAL_ORDER_CYCLES`=3 个电周期**，行程用两个
  **匀速段**采样点（`CAL_ORDER_FRAC_LO/_HI` = 20%/90%）的差来算，再除 0.7 外推出全程。
  两个坑都踩过：①在斜坡结束瞬间取角度会读到"动态负载角"偏短（14 极对 0.449 rad 行程，
  0.95° 机械滞后就算成 15）；②改成"保持末相位 0.75 s 后比较两个静止点"仍不够——
  静摩擦让转子停在离平衡点 `T_fric/k_spring` 的地方（本机 ±0.04 rad，占单周期 0.571 rad
  的 7%），台架实测 11 极对会随机算成 12，随后 `e_zero`/LUT 全错、**标定完电机不出力矩**。
  匀速段两点负载角相同、在差里抵消，与静摩擦无关。改这里必须连跑几轮标定确认极对数稳定。

- 结果写 `R_PHASE`/`L_D`/`L_Q`/`KT`（`__float_reg[13]/[11]/[12]/[14]`，都持久化；
  L_D/L_Q 已从 `hw_config.h` 常量改成配置寄存器）。新增 endpoint 是
  `config.l_d`/`config.l_q`（133/134）和只读的 `telemetry.ld_lq_diff`（135）。
- **两类电压上限别混用**：`CAL_V_MAX`(2 V) 是电感翻转幅值，`CAL_LOCK_V_MAX`(12 V)
  是 R / 锁转子的直流钳位；R 用两步**实测电流**算 `(V2-V1)/(I2-I1)`，所以第二步
  被钳位也不影响结果（电流差太小才失败）。`CAL_KI_R`/`CAL_KI_LOCK` 用 4.0 而不是
  ODrive 的 1.0（1 Ω 级电机 1.5 s 内要能稳定）。
- 电感公式用 `|delta_i|`（`cal_ripple_inductance()`）：PWM 比较值预装载，ISR 里
  ADC 采到的电流滞后写入的占空比两个周期，有符号写法会得到负 delta 而误判失败；
  分子里也不要加 R·I 项。
- 控制环里的 dq 电压不是物理伏特：`svm()` 的换算给出
  `CAL_V_PER_CMD = 0.5/((DTC_MAX-DTC_MIN)·√(1/3))`（`DTC_MAX = 0.9` 时约 0.962）。
  测量结果在存盘前乘上它；**改 `svm()`/`DTC_MAX`/`DTC_MIN` 时必须同步改
  `CAL_V_PER_CMD`**（本仓库里它就是 `V_PER_CMD` 的别名，自动跟随）。
- 任一段失败就置 `comm_encoder_cal.failed` → 回菜单且不写 flash；`run_fsm_slow()`
  只在 `done_ordering && done_cal && done_rl` 后保存。细节见 `docs/DEV.md`。
- **电流环 PI 增益由标定的 R/L 推导**（`foc.c` 的 `current_loop_retune()`，极点对消）：
  `k = 2π·i_bw·L/V_PER_CMD`、`ki = 1-exp(-R_PHASE·DT/L)`，`k_d/ki_d` 用 `L_D`、
  `k_q/ki_q` 用 `L_Q`。所以 `config.i_bw` 就是**实际穿越频率**、与电机无关。
  **不要退回 `K_SCALE`/`KI_D`/`KI_Q` 那套固定常数**——它们只是"L≈24 µH、R/L≈1800 rad/s"
  那一台电机的值，作为未标定（`r_phase<=0`）时的兜底保留。重算的四个时机：
  `init_controller_params()`（上电）、`reset_foc()`（每次进电机模式）、标定收尾
  （`run_fsm_slow()`，并打印推导结果）、endpoint 写 `config.i_bw`（串口菜单 `b`
  也调一次）。**实测穿越 = 0.76~0.78×`i_bw`**（离散化 + 2 拍延迟，见下面 scope
  一节）：默认 **2000**（穿越 1544 Hz、PM 68°），生产建议 ≤2000；
  上限 4000（PM 48°）只给台架测极限用。

## 齿轮式双编（断电记忆输出轴位置）

第二颗 KTH7823 经小齿轮啮合在转子齿轮旁（同一 SPI1，`SEC_ENC_SPI_NSS` = PD1），
两路单圈绝对值合起来解出转子"第几圈"，上电即恢复输出轴绝对位置。ODrive 分支
f5b1f8ba6 / 59e2ad0dc / 6bcbd9017 的移植（但零点用"参考快照"而不是找重合点，
理由见 `dual_gear.h` 头注释）。

- 纯数学在 `Core/Inc/dual_gear.h`（不含固件头文件），host 回归
  `tools/test_dual_gear.c`（已进 `make hosttest`）。硬件侧在 `position_sensor.c`：
  `ps_sec_read()` / `ps_sec_probe()` / `ps_dual_gear_restore|capture|read()` /
  `ps_set_m_zero()`。`ps_sec_probe()` 连读两帧判在线（`telemetry.sec_enc_present`），
  上电、进入标定、gear_capture/gear_read 都会更新它。
- **副编码器不进 40 kHz 中断**：它和主编码器抢 SPI1 + DMA2，所以只在主循环读
  （上电 / 标定入口探测 / 标定收尾 capture / 诊断），读时 `NVIC_DisableIRQ(PWM_ISR)`，
  有界等待（`PS_WAIT_ITERS`），读不到只累加 `sec_encoder.lost_frames`。
- **标定流程自带双编检测**：`run_fsm_slow()` 的 `CALIBRATION_MODE` 分支先探一次
  （打印 "GEAR: ..."），收尾块再 capture 参考并随标定一起存盘；失败时保留旧参考。
- `ps_dual_gear_restore()` 必须在 `main()` 里、`TIM_ConfigInt(ATIM1,…,ENABLE)` 之前
  调用；恢复值经 `EncoderStruct.abs_turns_valid` 由**下一帧** `ps_finish_sample()`
  生效（直接改 `turns` 会被 first_sample 逻辑清零并留下速度尖峰）。
- **改 `M_ZERO` 只能走 `ps_set_m_zero()`**：它同时平移 `GEAR_ZERO_COUNT`，否则
  断电再上电多圈位置会整体偏掉。`action.zero` 与 `config.m_zero` 的写入口都走它。
- 约束 `k = T1 - T2 ≠ 0`；量程 = ±T2/(2|T1-T2|) 转子圈。本机（MZW55，**30/27 反向**，
  GR = 9）量程 ±4.5 转子圈 = ±0.5 输出圈，相位周期 9 转子圈（超窗后差整整一圈）。
  `gcd(|k|,T2)` 不必为 1（最近候选解码不需要）；齿数成比例（10/9 = 30/27）等价。
  `config.gear_ref_valid = 0` 时整套功能不参与；出厂默认齿数/方向就是 30/27 + reversed。
- 参数/动作/遥测：`config.gear_teeth_first|sec|sec_reversed|zero_phase|zero_count|ref_valid`、
  `action.gear_capture|gear_read`、`system.gear_status`、`telemetry.sec_enc_*` /
  `gear_abs_turns`；台架验收步骤见 `docs/DEV.md`。

## Custom printf

`myprintf()` (declared in `main.h`, defined in `main.c`) replaces `printf`: it sends over
USB CDC when `bDeviceState == CONFIGURED`, otherwise it polls USART3. It **can block**
(it busy-waits on packet completion with timeouts) → main loop only, never in the ISR.

## CAN

`can.h`'s comment table is the spec of record; the controller is `FDCAN2`. ID =
`(opcode << 7) | node_id`. Floats on the wire are float32 little-endian. All externally
supplied floats (CAN, serial) must be clamped and NaN-checked before reaching the control loop.

CYBERBEAST (`can_cyberbeast.c`) gotchas verified on the bench:

- **An SDO write ACK carries no status.** Read-only endpoint, wrong width, and a failing
  `require` all still ACK — always read back. (`docs/DEV.md` 台架实测 has the table.)
- **`CAN_TIMEOUT` (`config.can_timeout`) is in milliseconds** (default `100`), but
  `controller.timeout` counts control cycles (25 µs), so every comparison goes through
  `CAN_TIMEOUT_CYCLES` (= `CAN_TIMEOUT * 40` in `user_config.h`) — never compare the raw
  value. On timeout `zero_commands()` clears `p_des/v_des/kp/kd/t_ff`.
- **Heartbeat `Dest` is the master id** (learned from received frames, cleared at reset →
  falls back to 1), not a broadcast; don't filter heartbeats by `dest == my_host_id`.
- The bench tool's own frame rate matters: a slow serial read once pushed the harness to
  ~20 Hz, below the 25 ms timeout, which looked like "the firmware ignores commands".
- **The MIT torque full scale is the parameter `config.mit_tau_max`** (output side N·m,
  default 5.0, `0` = the old `I_MAX*KT*GR`). It is used for the MIT 12-bit torque field
  *and* as the clamp for `command.t_ff`, `TORQUE`/`CURRENT` and legacy `SET_TFF` (single
  `TORQUE_FS` expression in `user_config.h`). It is a **wire** constant — a master must
  read the endpoint and encode/decode with the same value. It is not the achievable
  torque: `I_MAX*KT*GR` still caps the current loop (12 A × 0.0334 × 9 ≈ 3.6 N·m, 4 A
  ≈ 1.2 N·m), so a command above that just saturates `i_q` at `i_max`.

## USB 身份

`usb_desc.c` 的 VID/PID 是 **`0x1209/0x4342`**（pid.codes 共享 VID，"CB" = CyberBeast）。
**不要再出现 `0x1209/0x0D32`**——那是 ODrive Robotics 在 pid.codes 登记的 PID，上游
`usb_desc.c` 或合并冲突里遇到它必须改回 `0x4342`。`0x1209` 下的 PID 需在 pid.codes
提 PR 注册（要求公开仓库），未注册前只是暂用；换 ID 会让 Windows 重新枚举出新的 COM 号。

**复合设备**：`bDeviceClass=0xEF` + 两个 IAD，interface 0/1 = CDC（**端点分配和
原来完全一样**：EP1 IN / EP3 OUT / EP2 通知），interface 2 = vendor specific
（EP1 OUT / EP3 IN，走 fibre 风格的 endpoint 协议）。PMA 布局见 `usb_conf.h`。
`endpoint_usb.c` 的约束不能破坏：USB 中断只把 ≤64 B 从 PMA 拷进邮箱并让端点
保持 NAK，解析/应答全在主循环 `endpoint_usb_poll()`；这是“不损失 FOC 高带宽”
的关键。响应 ≤63 B（短包），不要发 64 B 整包。Windows 上 interface 2 需要
Zadig 装 WinUSB（CDC 不受影响）。

（历史：早先的复合描述符用的是 ODrive 的 PID `0x1209/0x0D32`，Windows 已为它
绑过驱动，于是只出 COM 口、数据管道不启动；换成我们自己的 PID 后复合布局可用。）

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
- 设备把一份紧凑 JSON 描述符烧进 flash（当前约 20.1 KB），通过 **endpoint 0**
  分块下载；描述符的 CRC16 同时是“接口版本”（每次增删参数都会变），除
  endpoint 0 外的每个请求都必须在 payload 末尾带这个 CRC，否则返回
  `EP_ERR_VERSION`，避免新旧固件静默错配。
- **读和写必须显式区分**：请求 `id` 字段的 bit14 是 `EP_REQ_WRITE`，bit15 是
  `EP_REQ_REPLY`，低 14 位是 ID。handler 按描述符的 `access` 校验标量端点
  （`r` 拒绝写、`w` 拒绝读）；**动作端点（`type: void` / 线上 `function`）例外：
  读和写都会执行**，因为 ODrive 的 endpoint 协议没有读写位、function 端点被“读”
  也会执行（JointSDK 就用 `PARAM_READ(0x20)` 调动作）。
  ⚠ `require` 只是**状态前置条件**，不是读写判别：`action.idle_mode` /
  `action.clear_fault` / `action.reboot` / `action.reset_timing` 连 `require` 都没有，
  一次读就会停机/复位。所以**上位机遍历参数时必须跳过 `function` 类型**
  （`rmc dump` 就是这么做的）。**不要**把标量端点退回“payload 为空就是
  读”的写法。USB 侧由 `rmc` 置位；CAN 侧由 `PARAM_READ(0x20)`/`PARAM_WRITE(0x21)`
  操作码映射。协议版本已升到 2。
- 两条传输共用 `endpoint_process()`（`endpoint.c`），只是外层不同：
  **USB CDC** 是 `0xAA+LEN+CRC8/CRC16` 字节流（和 ASCII 菜单共用数据流，
  `0xAA` 帧头之外的字节交给 `update_fsm()`）；**vendor interface 2** 上一个
  USB bulk 包就是一帧（`endpoint_usb.c`，响应 ≤63 B 短包）。帧格式和状态码见
  `Core/Inc/endpoint.h`。
  第三条是 **CAN**（CYBERBEAST，`can_cyberbeast.c` 直接调 `endpoint_handler()`，
  不经过 `endpoint_process()`，因此没有 status 字节、写 ACK 不带成败位）；
  上位机 `rmc --can`（`--interface slcan|pcan|socketcan|…`）用的就是它。
- **实时性**：USB 中断里的 `endpoint_serial_feed()` 只做帧累积（含一次
  最多 64 字节的 CRC）；`endpoint_usb_rx_isr()` 只做 64 B PMA 拷贝并保持端点
  NAK（主循环处理完再 arm，用 USB 流控代替队列）。解析、调用 handler、应答都在主循环的
  `endpoint_serial_poll()` 里。不要在 handler 里 printf、写 flash 或阻塞。
- **写入安全**：生成的代码统一做 NaN/Inf 检查和 JSON `min`/`max` 限幅；
  `require` 为假时返回 `EP_ERR_STATE`（例如 `config.ppairs` 要求门极关闭）。
  动作端点（`action.*`）只置 `state.save_pending` / `state.zero_pending` 等
  标志，flash 擦写仍由 `run_fsm_slow()` 在门极关闭时完成。`action.save` 返回
  `EP_OK` 只表示“已受理”，结果读 `system.save_status`（1 成功 / 2 因门极使能
  跳过 / 3 写失败）或看 `system.prefs_sequence` 是否增加。
  状态切换用 `request_fsm_state()`（带临界区），不要在 handler 里直接调
  `update_fsm()`——后者是串口/CAN 输入路径。`action.reset_config` 保存成功后
  会自动置 `reboot_pending`（`CAN_ID/CAN_PROT/CAN_BAUD` 运行期生效，改了不重启
  会让主站失联）；`action.reboot` 不要动 `_reboot_cookie`（`main.c` 的
  `!= 42 → 再复位一次` 是启动状态不确定时的保险）。
  **例外是进引导程序的两个动作**：`action.enter_dfu_mode` 置 `state.dfu_pending`，
  `run_fsm_slow()` 在应答发完后把 cookie 写成 `0xDEADBEEF` 再复位，等价于串口菜单的
  `d`（`early_start_checks()` 延时 ~2 s 后 `Jump_To_BOOT()`，走 ROM ISP）；
  `action.enter_bootloader_mode` 置 `state.bootloader_pending`，cookie 写成
  `BOOTLOADER_MAGIC_VALUE`（`0xB00710AD`，`main.h`），`0x08000000` 的 OpenBootloader
  看到后永久停在升级模式（CAN 500 kbit/s、Classic、SYNC ID `0x128`），等价于菜单 `b`。
  三条复位路径共用 `reboot_armed/reboot_deadline`，靠 `reboot_mode`（0 普通 /
  1 ROM DFU / 2 OpenBootloader）区分；**不要在 handler 里直接复位**，否则 USB/CAN
  的应答发不出去。
  ⚠ 装了 OpenBootloader 之后 **`action.enter_dfu_mode` 进不去 ROM DFU**（台架两次
  复现）：ROM ISP 确实被启动了（它用自己的栈改写了 `0x20023FF0` 附近），但 ~0.4 s
  后芯片以上电复位（`POR|PIN`，无 `SFTRSTF`）从 `0x08000000` 回来，多出一个 boot
  cycle。所以**刷机只走 CAN OTA**（`action.enter_bootloader_mode` + `blcan.py`），
  详见 `docs/DEV.md`。
- **`command.*`（t_ff/p_des/v_des/kp/kd）有两条硬语义**：写入必须刷新命令租约
  （`"set": "... = v; controller.timeout = 0;"`，与 CAN 帧一致，否则默认 100 ms
  的 `config.can_timeout` 一到就会被 `zero_commands()` 清掉；**不要**用把
  `can_timeout` 置 0 来绕过）；**进入 MOTOR_MODE 必须先 `zero_commands()`**
  （`fsm_enter_state`），否则空闲态写入的 `t_ff` 会在门极打开的第一拍生效。
  ⚠ `zero_commands()` 自己**必须把 `controller.timeout` 清零**：计数器只在
  MOTOR_MODE 自增、停在 `CAN_TIMEOUT_CYCLES+1`，而判超时用的是 `>`；不清零的话
  一旦超时过就**每一拍**都再进一次，主站刚写的命令当拍被清掉，表现为
  "CSV/CSP/MIT 完全不跟指令"。
  它们的范围是运行期决定的（`range_dynamic: true`），描述符里不发 `min`/`max`。
- 上位机：`rmc`（PyPI 包 **`robotmc`**，源码 `tools/rmc/`；`tools/mc_ep.py` 是
  转发到它的弃用 shim）：`tree|get|set|call|dump|monitor|probe|scan|backup|restore|
  liveplotter`，不带子命令进 IPython shell，启动时发现 CAN/USB 设备，按
  `system.serial_number` 分配稳定的 `m0/m1/...` 参数树名称；`m0.config.i_max` 读、
  赋值写，`m0.action.save()`，`m0.<TAB>` 补全。CAN 多节点共享同一 adapter。
  （`--interface slcan|pcan|socketcan|…`；**出货板没有 USB 座，客户只走这条**）。
  同一套命令也支持开发板上的 `--port COMx`（CDC）和 `--usb`（vendor interface）。
  工具会先下载描述符，再按名字收发。CAN 侧走 CYBERBEAST 的
  `PARAM_READ/WRITE` + 描述符 `0x24/0x25`，**没有 status 字节、写 ACK 不带成败**，
  所以 `set` 一律读回。CAN 传输的硬件无关回归测试是
  `python tools/test_mc_ep_can.py`（`make hosttest-py`）和 `tools/rmc/tests`
  （`make hosttest-rmc`），见 `docs/DEV.md` "测试"。
- **线上描述符必须是纯 ASCII，且类型名必须是 ODrive/Fibre 认得的**：
  `params.json` 里的中文 `brief` 只用于文档，**不进**设备描述符（OM/MS 端的
  JointSDK 轻量解析器不支持 `\uXXXX`，会整体解析失败）；动作端点在
  `params.json` 里写作 `void`，生成到描述符时必须变成
  `"type":"function"`（带空 `inputs`/`outputs`），否则同一解析器会报
  “unknown endpoint type”。这两点由 `gen_endpoints.py` 的 `descriptor_json()`
  保证，改它时不要退回去。

## 与 JointSDK（CyberBeast）的兼容性

⚠ **CAN FD 5 Mbit/s 必须开 TDC**（`config.can_tdc`，默认 `-1` = 自动 = 数据域 tseg1）。
2026-11-03 台架：不开 TDC 时 5M 一帧都不通（`can_tec` 涨到 bus-off、适配器收不到任何东西），
2M 正常；TDC 只能在 init 模式写，位置在 `MX_CAN1_Init()` 的 `FDCAN_Init()` 之后、
`FDCAN_Start()` 之前。改 `can_tdc` 后要重启才生效。

CAN 传输层已实现（`can_proto.c` 分发 + `can_cyberbeast.c`）：`config.can_prot`
选协议（0=旧 11-bit，1=CYBERBEAST），`config.can_baud` 选物理层（≤1M Classic，
>1M CAN FD：仲裁域固定 1M、数据域=该值），**出厂默认 = CYBERBEAST @ 1 Mbit/s
（Classic）**，**都只在下次上电生效**；
`system.can_prot_active` / `system.can_fd` 报告实际运行值。消息 RAM、滤波器、
NBTP/DBTP 都在 `can.c` 的 `MX_CAN1_Init()` 里按这两个值构建，所以该函数必须在
`config_apply_defaults()`/`preference_writer_load()` 之后调用（不能放回 CubeMX 的
SysInit 段）。

- CYBERBEAST 用 **29-bit 扩展帧**；旧协议用 11-bit 标准帧 `(opcode<<7)|node_id`。
  两套**不共存**（各装自己的滤波器），由 `can_prot` 二选一。
- `0x20/0x21/0x24/0x25` 直接复用 `endpoint_handler()` / `endpoint_json`；
  参数值小端、帧头大端。描述符流式发送在**主循环**（`can_proto_poll()`，2 帧/ms），
  不要在 ISR 里发；`can_cyberbeast_codec.h` 是 pure codec，`make hosttest` 覆盖。
- 已实现：MIT(0x00/0x80)、CSP(0x01/0x81)、CSV(0x02/0x82)、TORQUE(0x03/0x83)、
  CURRENT(0x04)、SDO、查询 0x40~0x49、心跳 0x48、系统 0x60~0x65、ESTOP 0xC0。
  CSP/CSV 走固件自己的串级外环（`motion.c`），增益在 `config.pos_gain` /
  `vel_gain` / `vel_integrator_gain`，帧里只有目标值与限幅。
- **未实现**：批量读（0x20 bit6 → 回 ERR，主站退回单读）、ODrive 路径别名。
- **高层 `jsdk_context_configure()` 仍不兼容**：它会先做握手（等 `0x00` MIT
  响应）并解析 `axis0.motor.config.gear_ratio` 等 7 个 ODrive 专有路径。
  只用 endpoint 访问时，改用 `jsdk_context_desc_fetch()` + `jsdk_joint_param_get/set`；
  **不要为它改固件参数命名**（客户将改 JointSDK）。

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
  （`N32H473CE_FLASH.ld`）和 Keil scatter（`ODrive_N32.sct`）的 app 区是
  `0x08008000..0x0807BFFF`（`0x74000`），镜像不可能长进配置页，也不会压到
  `0x08000000..0x08007FFF` 的 OpenBootloader。**bootloader 必须只擦 app 区**；
  整片擦除会连 bootloader 和配置一起清掉，任何软件格式都救不了。这是 OTA 保留
  配置的唯一硬前提，发布前必须做一次“设参数 + 标定 → OTA → 读回”的闭环验证
  （见 `docs/DEV.md` 的“Flash 布局与 OTA 契约”“发布检查表”）。
- **bootloader 必须喂 IWDG**：App 一上电就启动 IWDG（~4 s），它只有上电复位才停，
  所以从 App 软复位进 bootloader 后仍在计数。OpenBootloader 的
  `OPENBL_IWDG_Refresh()` 已实现为 `IWDG_ReloadKey()` 并在主循环 / CAN 收包等待里
  调用——**不要把它改回空函数**，否则 OTA 每 4 s 被复位一次，永远写不完。
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
  `__int_reg[6..133]`；**固件版本只有 `main.h` 的 `FW_VERSION_MAJOR/MINOR/REVISION/
  UNRELEASED` 一处**（endpoint `system.fw_version_*`、CYBERBEAST 设备信息帧、启动
  横幅都由它派生；老协议 `CAN_VAR_VERSION` 按协议仍是 float，编码为
  `major + 0.01*minor`（minor 占百分位、0..99，靠 `_Static_assert` 守住；
  用 0.1 的话 minor ≥ 10 会进位到整数部分）。曾经的手写 float `VERSION_NUM` /
  `system.version` 已删除，别再加回来。
  **布尔标志**（`PRE_CALIBRATED` / `FET_TEMP_EN` / `MOT_TEMP_EN`）用 `main.c` 里的
  `uint8_t` 全局（`*_flag`）而不是 `__int_reg`：TLV 里 bool 记录只有 1 字节，
  塞进 `int` 只会写低字节。新增 bool 参数照这个来。
- Keil/AC6 `NO_INIT` trick: `__attribute__((section(".bss.NO_INIT")))` (used for
  `_reboot_cookie`) must stay in sync with the scatter file `Keil/ODrive_N32.sct`.
