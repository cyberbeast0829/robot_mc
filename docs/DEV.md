# 开发者手册

面向修改固件的人。用户侧用法见 [USER.md](USER.md)。

目录：[架构与不变式](#架构与不变式) · [标定](#标定) ·
[endpoint 协议](#endpoint-协议) ·
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
控制 ISR ──> analog_sample → ps_sample(+PLL) → check_faults → run_fsm(motion_control) → can_tx_rx
```

| 文件 | 职责 | 可在 ISR 里跑 |
|---|---|---|
| `n32h47x_48x_it.c` | 控制 ISR 调度、`fault_shutdown()` | 是 |
| `foc.c` | 电流环/`torque_control`/`check_faults` | 是 |
| `motion.c` | 控制律分发 + 分频的速度环/位置环（CSP/CSV） | 是 |
| `motion_law.h` | 外环纯数学（PI/斜率/故障/PLL，可主机测试） | 是（纯计算） |
| `position_sensor.c` | SPI1+DMA2 读编码器、速度 PLL | 是（唯一允许阻塞等 DMA） |
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

## 标定

`CALIBRATION_MODE` 的 ISR 链路是 `order_phases()` → `calibrate_encoder()` →
`measure_lr()`（`Core/Src/calibration.c`），全程在 40 kHz 控制中断里跑，所以
**不能 printf**；阶段进度用 `cal_msg_push()` 交给主循环的 `cal_print_flush()`。
`run_fsm_slow()` 只在 `done_ordering && done_cal && done_rl` 都置位后退出到菜单
并保存。

`order_phases()` 用"电压矢量转一个电周期 ÷ 机械行程"求极对数。斜坡结束后必须
**保持末相位（`CAL_ORDER_HOLD_S`，与起始同相位）等转子停稳再取角度**：转子加速
时滞后，14 极对的理想行程只有 0.449 rad，滞后 0.95°（机械）就会舍入成 15；保持
后静态负载角在 `theta_end − theta_start` 中相消。两端各用 `CAL_ORDER_AVG_S`
（0.1 s）平均抑制抖动。

`measure_lr()` 是 ODrive `Motor::measure_phase_resistance` /
`measure_phase_inductance` / `measure_ld_lq` / `measure_flux_linkage` 及其
控制律的 C 移植：

| 阶段 | 方法 | 时长 |
|---|---|---|
| R | alpha 轴积分电流源，两步 `I_CAL/2` → `I_CAL`；用**实测的**两步电流和电压做 `R = (V2-V1)/(I2-I1)`，死区项相消 | 2 × 1.5 s |
| L | alpha 轴每拍翻转 ±2 V，累加带符号的 ΔIα 并取幅值 | 1.25 s |
| Ld / Lq | 先在 alpha/beta 上锁住转子，再翻转电压测该轴电感 | 2 × 1.25 s |
| 磁链 | 开环升速到 60 rad/s（电角度），稳态采样 `Vq/Id/Iq`：`λ = (Vq - R·Iq - ω·Ld·Id)/ω`，再 `Kt = 1.5·pole_pairs·λ` | 4.5 s |

- **两类电压上限要分开**：`CAL_V_MAX`（2 V）是电感测量的翻转幅值，决定电流纹波；
  `CAL_LOCK_V_MAX`（12 V）只是 R 积分器和 Ld/Lq 锁转子的直流钳位。用小钳位会让
  高阻电机（1 Ω 级：4 A × 1 Ω ≈ 4 V 物理电压）永远到不了测试电流 —— 台架上
  4.00 A 时 R 阶段在 2 V 饱和就是这么发生的。
- R 用**实测电流**而不是目标电流，所以高阻电机第二步即使被 12 V 钳位、或积分器
  还没完全稳定，只要两步电流差足够大（≥ 0.2·I_low）结果仍然成立；两步电流差
  太小（开路、母线过低）才失败。`CAL_KI_R`/`CAL_KI_LOCK` 用 4.0 而不是 ODrive 的
  1.0：ODrive 的 1.0 对 50 mΩ 电机没问题，但对 1 Ω 电机 1.5 s 只能稳定到约
  84 %（R 会低 17 %）。
- **电感公式用幅值**：PWM 比较值是预装载的，ISR 里 ADC 采到的电流对应两个控制
  周期前写入的占空比，比本周期 `test_voltage` 的翻转晚一拍，所以台架上用有符号
  累加会得到负 delta。`cal_ripple_inductance()` 用 `|delta_i|`（`L = V·dt/|ΔI|`），
  不要"修正"回有符号形式，也不要在分子里加 R·I 项（采样点在纹波拐点，Ld/Lq 锁
  转子的直流压降已被 lock 电压抵消，剩余阻尼是二阶小量）。
- 串口会在两步之间打印 `R step 1: <V> @ <A>`，失败信息带阶段名、失败值和辅助值
  （R 阶段是两步电流差），便于判断是"电流没到"还是"接线/母线问题"。
- 测试电流 = `min(config.i_cal, config.i_max)`（`i_cal` 和编码器标定共用；
  这几段要持续通直流，不能超过板级限流）。过流硬件/软件保护照常在
  `check_faults()` 里生效。
- 结果直接写 `R_PHASE` / `L_D` / `L_Q` / `KT`（都持久化，ID 17/133/134/15）。
  `L_D`/`L_Q` 从 `hw_config.h` 的固定宏改成了 `user_config.h` 的 `__float_reg[11]/[12]`。
  `controller.flux_linkage` 在 `init_controller_params()` 里按 `KT/(1.5·PPAIRS)`
  重算，所以重启后 `telemetry.flux_linkage` 仍能反映标定值；
  `telemetry.ld_lq_diff = L_D - L_Q` 是只读计算值。
- **电压单位换算**：控制环里的 dq 电压并不是物理伏特。`svm()` 把
  `(u - v_offset)` 变成 `0.5/v_scale` 的占空比偏移，而
  `v_scale = v_bus·(DTC_MAX-DTC_MIN)·√(1/3)`，所以实际相电压 = `CAL_V_PER_CMD ×
  指令电压`，`CAL_V_PER_CMD = 0.5/((DTC_MAX-DTC_MIN)·√(1/3))`（当前 DTC 0…0.7
  时约 1.237）。`measure_lr()` 在计算 R/L/λ 前把电压项乘上这个因子，落盘的都是
  SI 值；**改 `svm()`、`DTC_MAX/DTC_MIN` 或 `OVERMODULATION` 时必须一起改它**，
  否则 R/L/Kt 会按比例失准。台架上可用万用表量相电阻做一次核对。
- 任何一段失败（电流差太小、母线/过调制限制、`Iβ` 不平衡、λ 超范围）都会置
  `comm_encoder_cal.failed`，回到菜单并且**不写 flash**；已有参数保持不变。
- 标定顺序有依赖：Ld/Lq 用同一测试电流把转子锁在 alpha/beta 上，磁链公式要用
  刚测出的 R 和 L_D（以及极对数算 Kt）。因此 `measure_lr()` 必须排在
  `order_phases()` 之后；失败会中止整轮标定。

## 速度环与位置环（CSV / CSP）

三条环都跑在控制中断里、相位锁在 PWM 更新（update/TRGO）上：

| 环 | 频率 | 实现 | 周期 |
|---|---|---|---|
| 电流环 | **固定 40 kHz** | `commutate()`（`foc.c`） | 25 µs |
| 速度环 | 40 kHz / `config.vel_loop_prescaler`（默认 5 → **8 kHz**） | `ml_vel_pi()` | 125 µs |
| 位置环 | 速度环 / `config.pos_loop_prescaler`（默认 2 → **4 kHz**） | `ml_pos_p()` | 250 µs |

`motion_control()`（`motion.c`）在 `MOTOR_MODE` 里取代了原来对 `torque_control()` 的
直接调用，按 `controller.ctrl_mode` 分发：

| `ctrl_mode` | 来源 | 行为 |
|---|---|---|
| `CTRL_IMPEDANCE`（默认） | MIT `0x00/0x80`、旧 11-bit 协议、`0x03`、`0x04`、`command.mode=0` | 原样调用 `torque_control()`，**每拍都算**，与改动前逐字节一致 |
| `CTRL_VELOCITY` | CSV `0x02/0x82`、`command.mode=1` | 速度 PI → `i_q` |
| `CTRL_POSITION` | CSP `0x01/0x81`、`command.mode=2` | 位置 P → 速度指令 → 速度 PI → `i_q` |

纯数学部分在 `Core/Inc/motion_law.h`（零固件依赖），host 测试
`tools/test_motion_law.c` 直接包含它，见“测试”一节。

### 为什么这样不损失电流环带宽

- 外环**只产生 `i_q_des` 这个参考**。`commutate()` 的增益、采样相位、`svm()`
  预装载顺序、`v_max` 计算、`d/q` 积分器一个字节都没改。
- 外环在更新拍插在 `commutate()` 之前，多花约 0.3–0.8 µs（25 µs 的 1–3%），
  对电流环相位裕度的影响 < 1°；分频后绝大多数拍完全不执行外环。
- 外环的可用带宽由**速度估计**决定，不由控制频率决定：PLL 带宽默认 500 rad/s
  （约 80 Hz）+ 8 kHz 采样的 62.5 µs 零阶保持，比电流环低一个数量级。
  所以“降外环频率”换不到电流环的任何好处，反过来也一样。
- ⚠ **`i_q_des` 必须每拍从 `controller.i_q_ref` 重新赋值**：`field_weaken()` 是
  *就地累加* `i_q_des`（`i_q_des += fw_int`）。分频等待期间若直接沿用 `i_q_des`，
  弱磁会一拍接一拍地叠加下去。

### 单位

全部是**输出轴**量（与 `torque_control()` 同口径），输出是**电机端安培**：

| 参数 | 单位 | 说明 |
|---|---|---|
| `config.pos_gain` | (rad/s)/rad | 位置误差 → 速度指令 |
| `config.vel_gain` | A/(rad/s) | 速度误差 → 电流指令（电机端） |
| `config.vel_integrator_gain` | A/(rad/s)/s | 速度积分 |
| `config.accel_limit` | rad/s² | 速度设定值斜率限制，0 = 关闭 |
| `config.pos_error_fault` | rad | 跟随误差故障阈值，0 = 关闭 |
| `config.pll_bandwidth` | rad/s | 编码器速度 PLL 带宽 |

出厂值：`pos_gain 15` / `vel_gain 2.469` / `vel_integrator_gain 44.44` /
`accel_limit 50` / `inertia 0.0006` / `tune_margin 2.2` / `pos_error_fault 0` /
`pll_bandwidth 2000`。这些都是 2026-11-02 台架整定的结果（见"惯量标定与增益推荐值"
里的实测表）：`tune_gains(ω_c 450)` 用 `tune_margin 2.2` 正好给出这一组
`vel_gain` / `vel_integrator_gain` / `pos_gain`；`accel_limit 50` 是 CSP 不过冲的前提
（10 时 1 rad 阶跃过冲 49%）。⚠ 它们仍是**起点**：换电机/换关节必须重新
`action.measure_inertia` + `tune_gains`，尤其是 `config.inertia` 的默认值只是本台架的
实测值，**不代表任何其他机构**。

为什么在安培域而不在 N·m 域：CSP/CSV 帧的限幅本来就是电流（A），
`limit_norm()`、`field_weaken()`、过流阈值也都在安培域。省掉每拍 2 次乘法 +
1 次除法，增益也不依赖标定出来的 `KT`。MIT 路径仍然是 N·m 域，不受影响。

帧到环的换算（`can_cyberbeast.c`）：

| 帧字段 | 线上单位 | 环内 |
|---|---|---|
| CSP 目标位置 | deg（输出端） | `× π/180` → `p_des`，再钳到 `[P_MIN,P_MAX]` |
| CSP `vel_limit` | RPM（输出端） | `× 2π/60` → `controller.v_limit`；0/NaN 退回 `V_MAX` |
| CSP/CSV `cur_limit` | A（**电机端**） | → `controller.i_cmd_max`，钳到 `[0, I_MAX]` |
| CSV 目标速度 | RPM（输出端） | `× 2π/60` → `v_des`，再钳到 `[V_MIN,V_MAX]` |

`vel_limit` 收到 0 一律按“未指定”处理并退回 `config.v_max`：位置模式配一个 0 限速
会让关节在重力下直接垮掉。要“别动”请用速度模式给 0。

### 抗饱和与限幅

```c
i_unlim = vel_gain*v_err + vel_integ;
i_q     = clamp(i_unlim, ±i_cmd_max);
if (i_q != i_unlim) { vel_integ *= 0.99f; }       /* 饱和 → 衰减，不累积 */
else                { vel_integ += vel_integrator_gain*v_err*dt; }
vel_integ = clamp(vel_integ, ±i_cmd_max);
```

与 ODrive 的 `vel_integrator_torque_ *= 0.99f` 同款。位置环没有积分器，所以不存在
位置积分饱和；位置误差通过 `v_limit` 钳位转成“按速度上限追赶”。

### 编码器速度 PLL

`ps_update_pll()`（`position_sensor.c`）用 `ml_pll_step()` 在**单圈计数**上做临界阻尼
PLL（`kp = 2·bw`、`ki = 0.25·kp²`，与 ODrive `Encoder::update()` 一致），输出
`comm_encoder.pll_velocity`（电机端 rad/s）。

- 用单圈计数而不是多圈浮点位置：`|count|` 超过 2^24 后 float32 已经丢掉整数分辨率，
  位置一大相位检测就变成台阶。
- 丢帧（`ps_finish_sample()` 返回 0）时不预测，否则估计会一路漂。
- 第一个有效样本直接对齐（`pll_seeded`），不追赶 —— 半圈相位差经 `ki` 会变成
  几十 rad/s 的速度尖峰，再进电流环就是一下冲车。
- **MIT 与旧协议继续用 `comm_encoder.velocity`**（475 µs 整数差分），
  这条路径没有改动；`telemetry.velocity` 与新增的 `telemetry.velocity_pll`
  可以并排看噪声差别。

**带宽怎么选**（`config.pll_bandwidth`，默认 **2000 rad/s ≈ 318 Hz**）：
估计器的滞后是叠加在速度环“被控对象积分器固有的 -90°”之上的，所以要按穿越频率挑：

| `pll_bandwidth` | 100 Hz 处相位 | 200 Hz 处相位 | 噪声（1 rev/s 时，电机端） |
|---|---|---|---|
| 500 (80 Hz) | -8° | -58° | ≈ 0.001% |
| 1000 (160 Hz) | -2° | -35° | ≈ 0.002% |
| **2000 (318 Hz)** | **-2°** | **-13°** | **≈ 0.05%** |
| 3000 (477 Hz) | -1° | -5.5° | ≈ 0.13% |

对照：19 点整数差分（`comm_encoder.velocity`）在同条件下噪声 ≈ **1.6%**，
但 200 Hz 只有 -17° 相位。也就是说 PLL 在**噪声更好的同时相位也更差不到哪去**，
可以把带宽开到 2000；真要在 200 Hz 以上做穿越，就调到 3000。

`17 位编码器 + 40 kHz` 的量化噪声本来就很低，所以“噪声太大”不是实际限制，
低速抖动更多来自机械与齿隙。

### 惯量标定与增益推荐值

外环的增益其实只有三个未知量：`J_out`、`KT`（已标定）、`GR`（配置）。给定目标穿越
频率 `ω_c`，其余都能算出来；**`action.tune_gains`** 就按下面这套算并直接写进 RAM：

```
J_out   = KT·GR·Δi / Δa                          （惯量标定直接给）
vel_gain            = tune_margin·J_out·ω_c/(KT·GR)      [A/(rad/s)]
vel_integrator_gain = vel_gain·clamp(ω_c/25, 5, 30)      [A/(rad/s)/s]   （ω_i ≈ 18 rad/s @ ω_c=450）
pos_gain            = ω_c/30                             [(rad/s)/rad]
```

输入是 `config.inertia`（必须先标定）、`config.omega_c`（默认 450）、
`config.tune_margin`（默认 1.0）。要求菜单态 + `KT>0` + `GR>0`，**只写 RAM**，
不落盘；写完用 `csv`/`csp` 子命令验证，满意再 `action.save`。

⚠ 这里的 25 和 30 不是教科书取值（线性串级设计会说 `ω_c/5`），是台架上试出来的：

| 试过的值 | 结果 |
|---|---|
| `vel_integrator_gain = vel_gain·ω_c/5`（= 246 @ ω_c 450） | stick-slip 极限环，速度误差从 −0.8% 涨到 **+13.6%**，过冲 58% |
| `vel_integrator_gain = vel_gain·18`（= 49 / 44） | 误差 +0.3% / +3.9%，干净 —— ω_i ≈ 18 rad/s 是上限 |
| `pos_gain = ω_c/5 = 90` | 0.1 rad 阶跃过冲 **65%** |
| `pos_gain = 60` / `45` | 过冲 47% / 33%（齿隙 + 摩擦引起的极限环，不是线性失稳） |
| `pos_gain = 22.5`（原来的 `ω_c/20`） | 0.1 rad 阶跃还行，0.2 rad 阶跃过冲 **29%**（accel_limit 10） |
| `pos_gain = 15`（`ω_c/30`） | 0.2 rad / 1 rad 阶跃过冲 0.2% / 4.2%，`\|pos_err\|` ≤ 0.1 mrad |

**位置环的过冲主要不是相位裕度问题，而是 `config.accel_limit` 太小**：位置环是纯 P，
它给出的速度设定值要经过斜率限制，限得太慢就出现"位置一直追、速度却上不去，
等追上了速度还没降下来"的过冲。把 `accel_limit` 放到 200 后，`pos_gain` 45 也只有
0.1% 过冲、0.10 s 稳定。所以：

| `accel_limit` | `pos_gain=15` 的 1 rad 阶跃过冲 |
|---|---|
| 10 | 48.7% |
| 30 | 20.2% |
| 50 | **4.1%** |
| 200 | 0.1% |

⇒ **CSP 想要快，先放开 `accel_limit`（30~100 rad/s² 对小惯量关节很温和），
不要靠加 `pos_gain`。**

**`tune_margin` 补的是线性公式的低估**：公式假设纯惯性负载，实测库仑摩擦把有效
对象增益压低，同一台关节要把速度环放大到 2.2~2.6 倍才跟得上指令（`vel_gain` 1.12 → 2.47）；
它只放大速度环，位置环不跟着放大。默认 1.0 = 纯公式（保守），摩擦主导的关节从
2~3 起试。最终值仍以 `csv --sweep` 为准（见"台架整定步骤"）。

**`action.measure_inertia`** —— 一条指令完成，不需要先手动进电机模式。
`run_fsm_slow()` 负责编排（`fsm.c` 的 `inertia_phase`）：请求 → `MOTOR_MODE`
→ 门极一开就触发 ISR 里的状态机 → 等它结束 → **自动回菜单**。
自动回菜单是必须的：测完若停在 MOTOR_MODE，门极还开着而转矩是 0，有重力负载的
关节会直接垮下来；回菜单后紧接着就能 `action.save`（save 要求门极关闭）。

状态机本身（`Core/Src/motion.c` 的 `inertia_step()`）在 MOTOR_MODE 里跑，约 1.3 s，
全程在控制 ISR 内、不 printf、不写 flash：

| 阶段 | 时长 | 做什么 |
|---|---|---|
| `INERTIA_SETTLE` | 150 ms | 零电流，确认关节是静止的（窗内最大 \|ω\| 超过 0.15 rad/s 就直接失败） |
| `INERTIA_BREAKAWAY` | ≤600 ms | 每 8 ms 加 0.05 A，直到 \|ω\| > 0.15 rad/s → 得到"摩擦+重力"等效电流 `i_break` |
| `INERTIA_BRAKE` | 20–150 ms | **主动刹车**（`i = -ω`，限幅到 `0.5·I_MAX`）而不是零电流滑行 —— 滑行会让关节多走几倍行程，有限位的关节直接撞限位 |
| `INERTIA_TEST_LO` | ≤60 ms | 电流台阶 `1.5·i_break`，跳过前 8 ms（电流环建立时间）后流式最小二乘拟合 `dω/dt` |
| `INERTIA_BRAKE` | 20–150 ms | 同上 |
| `INERTIA_TEST_HI` | ≤60 ms | 电流台阶 `2.5·i_break`（都钳在 `0.5·I_MAX` 以内） |
| `INERTIA_BRAKE` → `INERTIA_DONE` | 20–150 ms | `J = KT·GR·(ī_hi − ī_lo)/(a_hi − a_lo)`，写 `config.inertia` |

单个台阶还会在**位移超过 0.10 rad** 或速度超过 2 rad/s 时提前结束（轻关节几毫秒就到），
实测整轮约 1.3 s、关节净行程 ~0.10 rad。

`system.inertia_status` 的失败码是分段的，方便定位：3 = 关节本来就在动，4 = 加电流到
上限还不动（卡住/摩擦过大），5 = 台阶太短采不到足够的点，6 = 激励不足/斜率不可信。

诊断遥测（只读）：`telemetry.inertia_state`（状态机停在哪）、
`telemetry.inertia_break_current`（本轮认定的突破电流）、
`telemetry.inertia_accel_lo` / `inertia_accel_hi`（两个台阶各自拟合出的加速度）。
`J` 由 `(ī_hi − ī_lo)` 和 `(a_hi − a_lo)` 算出来，所以这两个加速度**必须明显不同**；
它们相等而 `status` 仍是 2，就说明读到的不是这一轮的数（旧镜像 / 烧录没成功，见
"烧录）。台架典型值（48 V、裸关节、GR 7.75、KT 0.125）：`break = 0.10 A`、
`a_lo ≈ 15.6`、`a_hi ≈ 37.3`、`J ≈ 0.0023 kg·m²`。

**为什么是两个同向台阶，而不是 ODrive 的 ±对称法**：模型是 `J·a = KT·GR·i − c`，
`c`（重力 + 库仑摩擦）是常值。两个同向台阶拟合出的斜率天然把 `c` 消掉；而 ±对称法
里库仑摩擦的方向随运动反向，相加抵不掉，会把 `J` 高估 —— 台架上摩擦占到力矩的
30% 时就是 30% 的增益偏差。

- 拟合只累计 5 个和（`ml_fit_t`），ISR 里不需要缓存样本。
- 结果**只写 RAM**，不会自动 `action.save`；`system.inertia_status` 报
  0 未跑 / 1 运行中 / 2 成功 / 3..7 失败原因。
- 打断路径：`zero_commands()`（超时、回菜单、故障）会把状态机复位成 IDLE 并置
  失败 —— 否则下次进电机模式会从半个台阶续跑，测出来的 J 是错的。
- 标定期间 `run_fsm` **跳过命令超时检查**（它自己就是命令来源），并且
  **跳过 `inertia_req` 待处理的周期**（否则 `zero_commands()` 会在
  `motion_control()` 采纳请求之前把它清掉 —— 这是实现时踩过的坑）。
- 前提：`KT > 0`、`GR > 0`（endpoint 的 `require` 就挡在这里）、关节能自由转
  一小段（净行程约 0.10 rad）、齿隙已经消除。卡住/欠激励会返回失败码，不会写垃圾值。

**`ω_c` 的上限由延迟决定，不由惯量决定**：电流环 ≈40 µs + PLL（2000 rad/s）+
速度环零阶保持（125 µs）+ 采样 ≈ 0.3–1 ms ⇒ `ω_c` 超过约 1000 rad/s（160 Hz）
实际就是在跟延迟赛跑。

台架实测（2026-10-30，48 V，裸关节，`prescaler 5/2`，`pll_bandwidth 2000`）：

| 项目 | 实测 |
|---|---|
| 标定出的 `config.inertia` | **0.00206 / 0.00237 / 0.00289**（连续 3 次，均值 0.0024，离散 ±17%；重标电机后 0.00227，`break 0.10 A`、`a_lo 15.6`、`a_hi 37.3`） |
| 标定耗时 / 关节行程 | ~1.3 s / ~0.10 rad（没加刹车时是 0.35 rad） |
| `vel_gain = 3` | 稳定（`ω_c = 3·KT·GR/J ≈ 1100 rad/s`），超调 26% |
| `vel_gain = 4` | 失稳（`ω_c ≈ 1500 rad/s`） |

⇒ 反推出的延迟约 1 ms（含摩擦/齿隙造成的等效滞后），与 `vel_gain 3` 的 26% 超调吻合。
**注意这个台架是"小惯量 + 大摩擦"**：按公式 `vel_gain = J·ω_c/(KT·GR)` 取 `ω_c=450`
只有 1.15，实测却要到 3 才跟得上指令 —— 因为库仑摩擦把有效对象增益压低了，线性公式
给的是**惯性主导**时的值。所以用法是：**公式给起点（并用来卡 `ω_c` 上限），
`config.tune_margin` 补摩擦（≈2.6），`sweep` 给最终值**。

台架实测（2026-11-02，新板 + 新电机，48 V，裸关节，GR 7.75、11 极对、
`KT 0.0311`、`R 0.414 Ω`、`Ld/Lq 22.8/45.3 µH`、`i_max 3 A`、`prescaler 5/2`）：

| 项目 | 实测 |
|---|---|
| 惯量标定 | **0.000536 / 0.000563 / 0.000601 / 0.000612**（`break 0.15~0.20 A`）⇒ 取 0.0006。⚠ 5 次里有 3 次 `status=3`（上一次的残余摆动被"静止判定"挡下），重测前要等 1 s 以上 |
| MIT `kp=1.5` / `4` / `10` / `20`（`kd = 2√(kp·J)`） | 上升(90%) 126 / 56 / 43 / 30 ms；稳态误差 **17 / 3.0 / 0.1 / 0.1 mrad** ⇒ 摩擦死区 ≈ `T_fric/kp`，`kp` 小时误差明显 |
| MIT `kp=10, kd=0.155` | 43 ms 上升、55 ms 稳定、0.1 mrad、峰值 0.96 A ⇒ **推荐工作点** |
| CSV `vel_gain` 1 / 2 / 3 / 4 | P-only 误差 −12.5 / −7.2 / −4.7 / −3.9%；**4 开始过冲 128%**（≈极限环） |
| CSV `vel_gain 2.5 + vel_integrator_gain 30` | 1 / 10 / 40 rad/s 误差 −0.1 / −0.1 / −0.6%，纹波 ≤2.6%；40 rad/s 峰值 2.6 A（`i_max 3 A`） |
| CSV 60~65 rad/s | 也能跟（−0.3%/−0.7%），峰值 2.65 A ≈ `i_max` |
| CSP `pos_gain` 10 / 15 / 22.5 / 30 / 45（accel 10） | 0.2 rad 阶跃过冲 0.5 / 4.3 / 29 / 44 / 62% |
| CSP `pos_gain 15`（accel 50） | 0.2 rad：0.2%、0.26 s；1 rad：4.2%、0.34 s ⇒ **推荐工作点** |

⇒ 新板/新电机的结论与旧板一致（`tune_margin ≈ 2.2`、`ω_i ≈ 18 rad/s`），
但 `pos_gain` 的甜点从 `ω_c/20` 下移到 **`ω_c/30`**（两块板上 22.5 都会过冲）。
⚠ 两个坑：① `config.p_max`（默认 ±12.5 rad）会把 `p_des` **钳位而不是拒绝**，
反复跑 CSV 后位置会漂到边界，之后 CSP 看起来"完全不动"；② `accel_limit` 200
做 0→65 rad/s 的猛冲会偶发 `FAULT_OC` 锁存（`accel_limit 100` 正常），
而 `i_max` 已按新电机降到 3 A（OC 门限 = 1.5×`i_max` = 4.5 A）。

### 位置跟随误差故障

`config.pos_error_fault > 0` 时，位置环在**没有被 `v_limit` 饱和**的情况下，
`|pos_err|` 连续超过阈值 100 ms 就锁存 `FAULT_POS`（`foc.h`）→ FAULT_MODE 关栅，
需要显式回菜单才能清除。

只用“未饱和”判定的原因是：目标离得很远（上电对齐、长距离移动）时位置环本来就
会顶在速度上限上追，这时误差大是正常的。**因此阈值必须小于 `v_max / pos_gain`**，
否则该条件永远不成立（误差一到 `v_max/pos_gain` 就饱和了，计时被清零）。

**推荐值**：固件直接算好一个上限放在 `telemetry.pos_error_fault_suggest`：

```
e_recover = ½ · a_max · T²            a_max = KT·GR·I_MAX/J_out，T = 100 ms
e_limit   = ½ · v_max / pos_gain
suggest   = min(e_recover, e_limit)
```

`e_recover` 是"驱动器在 100 ms 内能把静止的关节拉回来多少"，`e_limit` 是"再大就
必然饱和"。惯量未标定时（`config.inertia = 0`）只用 `e_limit`。它给的是**上限**：
机械行程小的关节应取更小的值（例如行程的 1/4）。

上报：MIT 响应帧 `CB_ERR_CONTROLLER(0x3)`；`QUERY_ERROR` 的 `type=3/5`。


### 分频怎么选

- 默认 5 / 2（8 kHz / 4 kHz）。ODrive 在 12 kHz FOC 上用的是 2 / 4（6 kHz / 1.5 kHz），
  所以这个量级完全够用。
- 全部设 1（两个环都 40 kHz）可以作为“带宽上限”对照，用来判断限制来自
  增益还是来自采样。
- 速度环不要低于 ~1 kHz：外环的相位裕度会开始被零阶保持吃掉。
  `MOTION_PRESCALER_MAX = 100` 就是这条线的硬上限（40 kHz/100 = 400 Hz 是允许的
  边界）。
- 两个环都必须留在控制 ISR 里。放主循环（`run_fsm_slow`，1 ms + 抖动）会让
  速度环带宽掉到 10 Hz 量级。

### 台架测试脚本

`tools/csv_csp_test.py`（USB endpoint，不需要 CAN 主站）。**它自己负责命令租约刷新**——
`command.*` 的租约只有 `config.can_timeout`（默认 100 ms），用 `mc_ep.py set` 一条条
敲根本无法维持，测出来的“环路不工作”其实是保护动作。

| 子命令 | 作用 |
|---|---|
| `plain` | 进 MOTOR_MODE，用 MIT 律（`--kp/--kd`）顶住当前位置 → 重构回归 |
| `csv` | 速度阶跃 `0 → +v → 0 → −v → 0` |
| `csp` | 位置阶跃 `保持 → +step → 保持 → 回原位 → 保持` |
| `timeout` | 刷新 0.6 s 后停手，验证租约到期后确实卸力 |
| `sweep` | 扫 `config.vel_gain`（纯 P），打印稳态误差/超调，结束后恢复原值 |

```bash
python tools/csv_csp_test.py --port COM6 csv --vel 1.0 --log run.csv
```

- 三重中止闸：`telemetry.fault` 非 0、`|velocity_pll| > --abort-vel`（默认 3 rad/s）、
  `|pos_err| > --abort-pos-err`（默认 0.5 rad）。任一触发都立刻退出并在 `finally`
  里 `action.idle_mode` 关门极。
- `--min-bus`（默认 15 V）挡住“主电源没上电还以为环路坏了”的情况；
  `--allow-low-bus` 可以在无功率级时只验证逻辑/遥测（此时 `i_q_des` 会顶到限幅而
  `i_q_filt` 恒为 0，这是正常的）。
- `--log` 输出 UTF-8 CSV，可以直接画曲线。
- ⚠ 它会**真的转电机**；纯速度/位置环没有行程保护（除非开 `config.pos_error_fault`），
  调试时手放急停上。

### 台架整定步骤

按下面顺序走，每一步都用上一节的脚本（它自带租约刷新与中止闸）：

```bash
# 0) 先看基线：MIT 律能不能顶住（kp/kd 走的是 CTRL_IMPEDANCE，不经过外环）
python tools/csv_csp_test.py --port COM6 plain --hold 2 --kp 20 --kd 1

# 1) 速度环 P：扫描 vel_gain（脚本内部把 vel_integrator_gain 临时置 0，结束后恢复）
python tools/csv_csp_test.py --port COM6 sweep --vel 1.0 --gains 1,2,3,4,6

# 2) 选定 vel_gain 后加积分，再跑一次完整阶跃看正反转
python tools/mc_ep.py --port COM6 set config.vel_integrator_gain 10
python tools/csv_csp_test.py --port COM6 csv --vel 1.0 --hold 0.8 --move 2.5 --log csv.csv

# 3) 位置环：从 ω_c/20 起试（台架上 ω_c/5 会因齿隙/摩擦过冲，见"惯量标定"一节）
python tools/mc_ep.py --port COM6 set config.pos_gain 22
python tools/csv_csp_test.py --port COM6 csp --step 0.2 --hold 1.2 --move 2.0 --log csp.csv

# 4) 惯量标定 —— 一条指令，自动进出电机模式（结束自动回菜单）
python tools/mc_ep.py --port COM6 call action.measure_inertia
python tools/mc_ep.py --port COM6 get system.inertia_status     # 1 运行中 / 2 成功 / 3..7 失败原因
python tools/mc_ep.py --port COM6 get config.inertia            # kg·m^2（只写 RAM，未保存）
python tools/mc_ep.py --port COM6 get telemetry.pos_error_fault_suggest

# 5) 由惯量算三个增益（只写 RAM；不满意就改 omega_c / tune_margin 重算）
python tools/mc_ep.py --port COM6 set config.omega_c 450
python tools/mc_ep.py --port COM6 set config.tune_margin 2.6     # 摩擦主导的关节从 2~3 起试
python tools/mc_ep.py --port COM6 call action.tune_gains
python tools/mc_ep.py --port COM6 get config.vel_gain            # 确认写进去了
python tools/csv_csp_test.py --port COM6 csv --vel 1.0           # 验证
python tools/csv_csp_test.py --port COM6 csp --step 0.2
python tools/mc_ep.py --port COM6 call action.save               # 满意后再保存
```

`action.tune_gains` 的输入只有 `config.inertia`、`config.omega_c`、
`config.tune_margin`（公式见"惯量标定与增益推荐值"），`ω_c` 取 300–600 rad/s
起步（上限由延迟预算决定）。**摩擦主导的台架实测需要 `tune_margin ≈ 2.6`** ——
公式给起点、margin 补摩擦、`sweep` 给最终值。标定/整定结果**都不会自动保存**。
```

判读方法：

| 现象 | 含义 | 动作 |
|---|---|---|
| `sweep` 的 `err%` 很大（>20%） | 该增益下力矩不足以克服摩擦 | 加大 `vel_gain`，或靠积分补 |
| `sweep` 的 `overshoot%` > 30% 或出现 `UNSTABLE` | 相位裕度不足（延迟/惯量） | 用该增益的 1/2.5 作为工作点 |
| `vel_integral` 持续爬升不收敛 | 积分太强或机械卡住 | 降 `vel_integrator_gain`，查机构 |
| CSP `|pos_err|max` 收敛但末端有极限环 | 低速干摩擦 + 量化（机械层面） | 开 `pos_error_fault`，或加大 `accel_limit` 平滑 |
| `i_q_des` 顶在限幅而速度不动 | 电压/电流不足或堵转 | 查母线、`config.i_max`、机械 |

**实机参考数据**（2026-10-30，48 V，裸关节：低惯量 + 干摩擦，GR 7.75、
KT 0.123 N·m/A）：

| `vel_gain`（纯 P，1.0 rad/s 阶跃） | 结果 |
|---|---|
| 1 | 稳定，稳态 ≈0.73 rad/s（−27%） |
| 2 | 稳定，稳态 ≈0.86–0.90，有超调 |
| 3 | 稳定 |
| 4 / 6 | **失稳**：速度冲到 3.6+ rad/s、电流限幅 |

该台架上 `vel_gain 2` + `vel_integrator_gain 10` 把稳态误差压到 −5…−9%，
`pos_gain 2` 的 0.2 rad 阶跃超调 12%、静止误差 2–3 mrad。
（这张表是旧电机 KT 0.123、J 0.0023 的数据；新电机见"惯量标定与增益推荐值"
里的 2026-11-02 表，出厂值按新电机定。）

**编码器 PLL 的真机校验**（用精确整数位置差分当参考，同区间均值）：
PLL 误差 ≤4%，19 点差分 9–16%。这也解释了为什么外环必须用 PLL：低速时差分
只有 ±1 LSB 的台阶（输出轴 0.026 rad/s/bit）。

### ISR 预算判据

改动前后各读一次：

| 端点 | 判据 |
|---|---|
| `timing.overrun` | 必须保持 0 |
| `timing.period_max` | < 0.6 × `timing.isr_period`（6000 cycles） |
| `timing.seg_fsm_max` | 外环更新那拍会变高；对比 1/1 与 5/2 两种配置 |

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
- 总长 ≤ 65535（当前约 10.5 KB）。

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
`CAN_ID/CAN_PROT/CAN_BAUD` 也回默认（出厂默认 **CYBERBEAST + Classic 1 Mbit/s**），所以保存成功后
固件会自动重启（0.5 s），之后要重新配 CAN 并重新标定。运行中的 `comm_encoder` 副本
故意不跟着改：重启前继续用旧标定，比拿全零 LUT 去换相安全。

## CAN：协议选择与 CYBERBEAST

**三个持久化参数决定协议与物理层，开机时读取一次，写入后下次上电生效：**

| 参数 | 含义 |
|---|---|
| `config.can_prot` | `0` = 旧 11-bit 协议，`1` = **CYBERBEAST（默认）** |
| `config.can_baud` | bit/s，默认 `1000000`（Classic 1 Mbit/s）。`≤ 1M`：Classic CAN，该值就是位速率；`> 1M`：CAN FD，仲裁域固定 1 M，数据域 = 该值（支持 2M/4M/5M） |
| `config.can_hb_ms` | CYBERBEAST 心跳周期（默认 100 ms，0 关闭） |
| `config.can_tdc` | CAN FD 发送延迟补偿偏移（单位：数据域 tq）。`-1` = 自动（= 数据域 tseg1，默认，与 ODrive / canable2-fw 一致）、`0` = 关闭、`>0` = 手动；只对 FD 生效，下次上电应用 |

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
| `0x01/0x81` CSP | 目标位置 deg(输出端) + `vel_limit` RPM(输出端) + `cur_limit` A(电机端)；Classic 8 B 用 float32 位置 + int16 RPM + int16 0.1A，FD 12 B 全 float32；进 `CTRL_POSITION`，单播回 MIT 响应 |
| `0x02/0x82` CSV | 目标速度 RPM(输出端) + `cur_limit` A(电机端)；进 `CTRL_VELOCITY`，单播回 MIT 响应 |
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
可直接被 SDK 的 `cb_jsondesc_parse` 解析（当前 132 个可见端点、约 10.5 KB；
改 `params.json` 后需在 SDK 环境重跑一次确认）。

### 台架注意

- **5 M 数据域必须开 TDC**（`config.can_tdc` 默认 `-1` = 自动）。2026-11-03 台架实测：
  不开 TDC 时 5M **完全不通** —— 设备自报 `can_fd=true`/`can_baud_active=5000000`，但
  `can_tec=248`、`can_rec=0`、`lec=7`，适配器一帧都收不到（发送无 ACK → 反复重发 → bus-off）；
  同一固件把数据域改成 2M（20 tq，裕量 2.5 倍）则 97/97 全过。根因：8 tq/25 ns 下收发器
  环回延迟（100~250 ns）吃掉采样裕量，发送端需要靠 TDC 把自己的求值点往后推。
  TDC 只能在 init 模式写，所以 `MX_CAN1_Init()` 里必须放在 `FDCAN_Init()` 之后、
  `FDCAN_Start()` 之前；offset 与 filter 都用数据域 tseg1。
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
`make hosttest`（`prefs_codec` + `test_can_cyberbeast` + `test_motion_law`）、
`make`（链接）。
Windows 用 MSYS2 bash 构建（Chocolatey 的 make 用 cmd，
`mkdir -p` 会失败）：

```bash
export PATH="/d/Debug/arm-gnu-toolchain-15.2.rel1-mingw-w64-x86_64-arm-none-eabi/bin:$PATH"
make -j8
```

改了协议/描述符时，用 JointSDK 的 `cb_jsondesc_parse.c` 解析
`mc_ep.py descriptor` 导出的 JSON，确认 `rc=0`、类型/权限正确。

### 烧录（J-Link，可靠流程）

**别用 `JLink.exe loadfile` 或 UV4 默认设置直接烧这块板。** 台架实测
（2026-10-30，J-Link V11 + JLink DLL V8.26，VTarget 2.917 V）：SWD **读回不稳**——
同一段 flash 连续读三次会得到三个不同内容（1 MHz），Keil 默认的 5 MHz 下 J-Link
自带 verify 随机报 `Verification failed @ 0x08000000`，而**编程其实只写了一部分**。
最坑的是它有时"看起来成功了"：设备继续跑旧镜像，于是新加的 endpoint 用的是旧实现、
遥测值自相矛盾（这一次就是 `inertia_accel_lo` 与 `inertia_accel_hi` 读回同一个值，
排查了半天才发现是镜像没换）。

可靠做法（`tools/flash_jlink.py` 就是把这一套写成脚本）：

```
python tools/flash_jlink.py            # 250 kHz + 8 KB 分块写 + 逐块回读比对 + 重试
python tools/flash_jlink.py --erase    # 需要时先整片擦除（会清掉配置页！）
python tools/flash_jlink.py --bin build/motorcontrol.bin   # GNU 构建产物也可以
```

- 250 kHz 是实测稳定的上限；1 MHz 读回就会偶发错字节（全镜像一次读完仍可能错 2 字节，
  所以**校验也要分块 + 失败重读**，不能只信 J-Link 的 `Verify`）。
- 烧完必须回读全镜像并逐字节比对，一致才算成功（脚本退出码非 0 就是没烧进去）。
- 烧完核心处于 halt，`COM` 口不会出现；脚本最后会 `r`+`g`。手动流程是
  `si SWD / speed 250 / device N32H473CE / connect / r / g / exit`。
- 整片擦除会清掉配置页（`prefs_loaded = false`、`prefs_sequence = 0`）：
  之后必须重新 `action.calibrate`（R/L/KT/编码器零点）再 `action.save`，
  否则电流环/惯量标定拿到的都是默认值（实测未标定时 `breakaway = 0.4 A`、
  加速度符号都反了，J 差 10 倍）。
- 烧录成功后核对一次现场：`system.prefs_loaded` / `system.prefs_sequence` /
  `config.kt` 是否符合预期，能立刻发现"其实没烧进去"。

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
| 15 | `call action.reset_config`（先记下 `prefs_sequence`） | `save_status = 1`、序号 +1；`config.e_zero/ppairs` 回默认；**随后自动重启**，重启后 `prefs_loaded = true`、`config.can_prot = 1` / `can_baud = 1000000`（出厂默认）并重新标定 |
| 16 | 门极使能时 `call action.reset_config` | `status 6`（`EP_ERR_STATE`），`save_status` 不变，配置不动 |
| 17 | 空闲态 `set command.t_ff 5` 后 `call action.motor_mode` | 门极使能后 `telemetry.t_ff = 0`（进入模式时清零），电机不窜动 |
| 18 | 电机模式下写 `command.t_ff`，然后停发 3×`can_timeout` | 命令自动清零（租约到期），`telemetry.i_q_des → 0` |
| 19 | `set command.mode 1` + `set command.v_des 1` | 电机按速度环转到 ~1 rad/s；`telemetry.v_setpoint`/`vel_err` 有值，`command.mode` 回读为 1 |
| 20 | `set command.mode 2` + `set command.p_des <当前+0.5>` | 位置环把关节转过去并停住；`telemetry.pos_err → 0`，无过冲振荡 |
| 21 | CSP 时连发 3×`can_timeout` 不刷新 | 卸力（零转矩），不是“保持位置” |
| 22 | `set config.pos_error_fault 0.2`，堵住关节让 CSP 追不上 | 约 100 ms 后 `telemetry.fault` 出现 `FAULT_POS(8)`，进 FAULT_MODE 关栅 |
| 23 | `set config.vel_loop_prescaler 1` / `pos_loop_prescaler 1` | `timing.overrun` 仍为 0，`timing.period_max` < 0.6×`isr_period` |

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
