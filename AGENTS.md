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
- There are no automated tests. Verification = a clean build + bench testing by the
  maintainer; the `Makefile` link is the only host-runnable check.

## SDK/API differences from upstream (important)

Use the Nations **standard peripheral driver** (`Drivers/n32h47x_48x_std_periph_driver`),
**not** ST HAL. In-tree examples: `GPIO_WriteBits`, `TIM_ClrIntPendingBit`,
`USART_GetIntStatus`/`USART_GetFlagStatus`, `ADC_CalibrationOperation`,
`FDCAN_GetRxMsg`/`FDCAN_Start`, `IWDG_ReloadKey`/`IWDG_WriteConfig`.
The PWM/advanced timer is `ATIM1` (aliased `TIM_PWM` in `hw_config.h`); FOC writes
`TIM_PWM->CCDAT1/2/3` directly against period `TIM_PWM->AR` (see `foc.c`).

⚠ **Never assume an `HAL_*` / `stm32f4xx_*` / `TIM1->BDTR` / `.Instance` reference resolves
here** — verify it against the N32 SDK. The incomplete-port leftovers have been cleaned up
(the `HAL_GPIO_WritePin` / `drv_disable_gd` / `TIM1->BDTR` calls; `foc.c`'s `__HAL_ADC_*` →
`ADC_GetFlagStatus`/`ADC_ClearFlag` on `ADC1` with `ADC_FLAG_JENDC`; the CAN TX
`HAL_CAN_AddTxMessage` → `FDCAN_AddMsgToTxFifoQ(FDCAN2, …)`; `can.c`'s `CAN_H.Instance->ESR`
→ `FDCAN2->ECR`/`FDCAN2->PSR`; and `fsm.c`'s `HAL_Delay`). `preference_writer.c` is now on the
N32 flash API — 8 KB page erase + 64-bit `FLASH_ProgramdoubleWord` and a software CRC-32 — in
the top two pages `0x0807C000`/`0x0807E000` (matching `flash_writer.h`). Only commented-out ST
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
