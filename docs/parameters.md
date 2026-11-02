<!-- 由 tools/gen_endpoints.py 从 params.json 生成，请勿手工修改。 -->
# 参数手册

ID 由 `tools/endpoint_ids.json` 锁定，固件升级不会改变；`access` 为 `r` 的只能读，
`w` 的只能写/执行，`rw` 可读写。写入值在固件里做 NaN 检查和上下限限幅。
`function` 就是 `action.*` 动作端点。`默认` 列偶尔是固件宏（如 `I_MAX_LIMIT`），
以固件为准。

## config：可写参数

| 名称 | ID | 类型 | 访问 | 单位 | 范围 | 默认 | 持久化 | 说明 |
|---|---|---|---|---|---|---|---|---|
| `config.i_bw` | 1 | float | rw | Hz | 100.0 .. 2000.0 | 1000.0 | 是 | 电流环带宽 |
| `config.i_max` | 2 | float | rw | A | 0.0 .. `I_MAX_LIMIT` | I_MAX_LIMIT | 是 | 相电流限幅（硬上限 4A） |
| `config.i_fw_max` | 3 | float | rw | A | 0.0 .. 33.0 | 0.0 | 是 | 弱磁电流上限 |
| `config.i_max_cont` | 4 | float | rw | A | 0.0 .. 40.0 | 14.0 | 是 | 连续电流上限 |
| `config.i_cal` | 5 | float | rw | A | 0.0 .. 20.0 | 5.0 | 是 | 标定电流（编码器/R/L/Ld-Lq/磁链共用；测量时限制在 i_max 以内） |
| `config.theta_min` | 6 | float | rw | rad | — | 0.0 | 是 | 机械角下限 |
| `config.theta_max` | 7 | float | rw | rad | — | 0.0 | 是 | 机械角上限 |
| `config.p_max` | 8 | float | rw | rad | ≥ 0.0 | 12.5 | 是 | 位置设定值上限；写入同时更新 p_min = -p_max |
| `config.p_min` | 9 | float | rw | rad | ≤ 0.0 | -12.5 | 是 | 位置设定值下限（通常由 p_max 联动，也可单独写入以便备份恢复） |
| `config.v_max` | 10 | float | rw | rad/s | ≥ 0.0 | 65.0 | 是 | 速度设定值上限；写入同时更新 v_min = -v_max |
| `config.v_min` | 11 | float | rw | rad/s | ≤ 0.0 | -65.0 | 是 | 速度设定值下限（通常由 v_max 联动，也可单独写入以便备份恢复） |
| `config.kp_max` | 12 | float | rw |  | ≥ 0.0 | 500.0 | 是 | 位置增益上限 |
| `config.kd_max` | 13 | float | rw |  | ≥ 0.0 | 5.0 | 是 | 速度增益上限 |
| `config.pos_gain` | 136 | float | rw | (rad/s)/rad | 0.0 .. 1000.0 | 15.0 | 是 | CSP 位置环 P 增益（输出轴）。位置误差 × 该值 = 速度指令；0 表示位置环只剩帧间前馈（要完全关闭请把 pos_ff_gain 也设 0） |
| `config.vel_gain` | 137 | float | rw | A/(rad/s) | 0.0 .. 2000.0 | 2.469 | 是 | CSV/CSP 速度环 P 增益（电机端安培 / 输出轴角速度）；0 时速度环只剩积分器（要完全关闭请把 vel_integrator_gain 也设 0） |
| `config.vel_integrator_gain` | 138 | float | rw | A/(rad/s)/s | 0.0 .. 200000.0 | 44.44 | 是 | 速度环 I 增益；消掉重力/摩擦下的稳态速度误差（抗饱和：输出限幅时积分按 0.99 衰减） |
| `config.pos_ff_gain` | 139 | float | rw |  | 0.0 .. 2.0 | 1.0 | 是 | CSP 帧间速度前馈系数（指令差分的比例）；1.0 = 全前馈，0 = 关闭 |
| `config.accel_limit` | 140 | float | rw | rad/s^2 | 0.0 .. 100000.0 | 50.0 | 是 | 速度设定值的斜率限制（输出轴）；0 = 关闭。非零默认值能在指令阶跃时压住电流冲击，对稳态无影响 |
| `config.pos_error_fault` | 141 | float | rw | rad | 0.0 .. 1000.0 | 0.0 | 是 | 位置跟随误差故障阈值（输出轴）；0 = 关闭。只在位置环未被速度限幅饱和时判定，需持续 100 ms；必须小于 v_max/pos_gain 才会触发 |
| `config.pll_bandwidth` | 142 | float | rw | rad/s | 20.0 .. 5000.0 | 2000.0 | 是 | 编码器速度 PLL 带宽（速度环/位置环的反馈来源）；越大相位滞后越小、噪声越大。默认 2000（约 318 Hz），200 Hz 穿越点只吃 -13° 相位 |
| `config.inertia` | 151 | float | rw | kg·m^2 | 0.0 .. 100.0 | 0.0006 | 是 | 输出端惯量（关节折算值）。可由 action.measure_inertia 测得（只写 RAM，不自动保存）；用于推荐增益与 pos_error_fault。0 = 未知 |
| `config.omega_c` | 159 | float | rw | rad/s | 50.0 .. 1200.0 | 450.0 | 是 | action.tune_gains 的目标穿越频率：决定算出来的 vel_gain / pos_gain 有多激进。受延迟限制，超过约 1000 rad/s 实际是在跟延迟赛跑，建议 300~600 起步 |
| `config.tune_margin` | 160 | float | rw | x | 0.2 .. 5.0 | 2.2 | 是 | action.tune_gains 对速度环增益的放大系数。线性公式给的是惯性主导时的值，摩擦/齿隙大的关节要放大（台架实测约 2.6）；只放大速度环，pos_gain 不受影响。1 = 纯公式 |
| `config.vel_loop_prescaler` | 143 | int32 | rw |  | 1 .. 100 | 5 | 是 | 速度环分频：控制环 40 kHz / 该值 = 速度环频率（默认 5 → 8 kHz）；1 = 全速 40 kHz |
| `config.pos_loop_prescaler` | 144 | int32 | rw |  | 1 .. 100 | 2 | 是 | 位置环分频：速度环 / 该值 = 位置环频率（默认 2 → 4 kHz）；位置环只在速度环执行的拍里执行 |
| `config.gr` | 14 | float | rw |  | ≥ 0.001 | 7.75 | 是 | 减速比（门极关闭时才允许写） |
| `config.kt` | 15 | float | rw | N-m/A | ≥ 0.0001 | 1.0 | 是 | 转矩常数（标定得出；门极关闭时才允许写） |
| `config.ppairs` | 16 | float | rw |  | 1.0 .. 64.0 | 14.0 | 是 | 极对数（标定得出；门极关闭时才允许写） |
| `config.r_phase` | 17 | float | rw | Ohm | — | 0.0 | 是 | 单相电阻（标定得出） |
| `config.l_d` | 133 | float | rw | H | 1e-06 .. 0.01 | 9.5e-05 | 是 | D 轴电感（标定得出；门极关闭时才允许写） |
| `config.l_q` | 134 | float | rw | H | 1e-06 .. 0.01 | 0.000124 | 是 | Q 轴电感（标定得出；门极关闭时才允许写） |
| `config.r_nominal` | 18 | float | rw | Ohm | — | 0.0 | 是 | 标称电机电阻 |
| `config.r_th` | 19 | float | rw | C/W | — | 0.0 | 是 | 热阻 |
| `config.c_th` | 20 | float | rw | J/C | — | 0.0 | 是 | 热容 |
| `config.temp_max` | 21 | float | rw | C | 0.0 .. 150.0 | 125.0 | 是 | 过温保护阈值（0 = 关闭） |
| `config.phase_order` | 22 | int32 | rw |  | 0 .. 1 | 0 | 是 | 相序（标定得出；门极关闭时才允许写） |
| `config.can_id` | 23 | int32 | rw |  | 0 .. 127 | 1 | 是 | CAN 节点 ID |
| `config.can_master` | 24 | int32 | rw |  | 0 .. 127 | 0 | 是 | CAN 主站 ID |
| `config.can_timeout` | 25 | int32 | rw | ms | 0 .. 100000 | 100 | 是 | CAN 命令超时（毫秒，0 = 关闭）；超时后清零 p_des/v_des/kp/kd/t_ff |
| `config.can_prot` | 120 | int32 | rw |  | 0 .. 1 | 1 | 是 | CAN 协议（0=旧 11-bit 协议，1=CYBERBEAST；下次上电生效） |
| `config.can_baud` | 121 | int32 | rw | bit/s | 50000 .. 5000000 | 1000000 | 是 | CAN 波特率：≤1M 为 Classic CAN（该速率），>1M 为 CAN FD（仲裁域固定 1M，数据域=本值）；下次上电生效 |
| `config.can_hb_ms` | 122 | int32 | rw | ms | 0 .. 10000 | 100 | 是 | CYBERBEAST 心跳周期（0 = 关闭） |
| `config.m_zero` | 26 | int32 | rw |  | — | 0 | 是 | 机械零位编码器计数（门极关闭时才允许写） |
| `config.e_zero` | 27 | int32 | rw |  | — | 0 | 是 | 电角度零点编码器计数（门极关闭时才允许写） |

## command：运行命令（电机模式下有效）

| 名称 | ID | 类型 | 访问 | 单位 | 范围 | 默认 | 持久化 | 说明 |
|---|---|---|---|---|---|---|---|---|
| `command.t_ff` | 128 | float | rw | N-m | 运行期：`-I_MAX*KT*GR` .. `I_MAX*KT*GR` | — | 否 | 前馈转矩（输出轴 N·m）；写入会刷新命令租约，进入 MOTOR_MODE 时清零 |
| `command.p_des` | 129 | float | rw | rad | 运行期：`P_MIN` .. `P_MAX` | — | 否 | 位置设定值（输出轴 rad）；范围由 config.p_max 决定，进入 MOTOR_MODE 时清零 |
| `command.v_des` | 130 | float | rw | rad/s | 运行期：`V_MIN` .. `V_MAX` | — | 否 | 速度设定值（输出轴 rad/s）；范围由 config.v_max 决定，进入 MOTOR_MODE 时清零 |
| `command.kp` | 131 | float | rw |  | 运行期：0.0 .. `KP_MAX` | — | 否 | 位置增益；上限由 config.kp_max 决定，进入 MOTOR_MODE 时清零 |
| `command.kd` | 132 | float | rw |  | 运行期：0.0 .. `KD_MAX` | — | 否 | 速度增益；上限由 config.kd_max 决定，进入 MOTOR_MODE 时清零 |
| `command.mode` | 145 | int32 | rw |  | 0 .. 2 | — | 否 | 控制律：0 = 阻抗/MIT（默认，kp·p_des·kd·v_des·t_ff），1 = 速度环（CSV，用 command.v_des），2 = 位置环（CSP，用 command.p_des）。写完刷新命令租约；进入 MOTOR_MODE 时回 0 |

## telemetry：实时测量（只读）

| 名称 | ID | 类型 | 访问 | 单位 | 范围 | 默认 | 持久化 | 说明 |
|---|---|---|---|---|---|---|---|---|
| `telemetry.v_bus` | 28 | float | r | V | — | — | 否 | 母线电压（滤波后） |
| `telemetry.v_bus_raw` | 29 | float | r | V | — | — | 否 | 母线电压（原始） |
| `telemetry.i_a` | 30 | float | r | A | — | — | 否 |  |
| `telemetry.i_b` | 31 | float | r | A | — | — | 否 |  |
| `telemetry.i_c` | 32 | float | r | A | — | — | 否 |  |
| `telemetry.i_d` | 33 | float | r | A | — | — | 否 |  |
| `telemetry.i_q` | 34 | float | r | A | — | — | 否 |  |
| `telemetry.i_d_filt` | 35 | float | r | A | — | — | 否 |  |
| `telemetry.i_q_filt` | 36 | float | r | A | — | — | 否 |  |
| `telemetry.v_d` | 37 | float | r | V | — | — | 否 |  |
| `telemetry.v_q` | 38 | float | r | V | — | — | 否 |  |
| `telemetry.theta_mech` | 39 | float | r | rad | — | — | 否 | 机械角（电机侧） |
| `telemetry.theta_elec` | 40 | float | r | rad | — | — | 否 | 电角度 |
| `telemetry.dtheta_mech` | 41 | float | r | rad/s | — | — | 否 | 机械角速度（电机侧） |
| `telemetry.dtheta_elec` | 42 | float | r | rad/s | — | — | 否 | 电角速度 |
| `telemetry.position` | 43 | float | r | rad | — | — | 否 | 输出轴多圈位置 |
| `telemetry.position_motor` | 44 | float | r | rad | — | — | 否 | 电机侧多圈位置 |
| `telemetry.velocity` | 45 | float | r | rad/s | — | — | 否 | 输出轴速度 |
| `telemetry.velocity_motor` | 46 | float | r | rad/s | — | — | 否 | 电机侧速度 |
| `telemetry.count` | 47 | int32 | r |  | — | — | 否 | 编码器多圈计数 |
| `telemetry.torque` | 48 | float | r | N-m | — | — | 否 | 输出轴转矩估计 |
| `telemetry.p_des` | 49 | float | r | rad | — | — | 否 | 当前位置设定值（输出轴） |
| `telemetry.v_des` | 50 | float | r | rad/s | — | — | 否 | 当前速度设定值（输出轴） |
| `telemetry.kp` | 51 | float | r |  | — | — | 否 | 当前位置增益 |
| `telemetry.kd` | 52 | float | r |  | — | — | 否 | 当前速度增益 |
| `telemetry.t_ff` | 53 | float | r | N-m | — | — | 否 | 前馈转矩 |
| `telemetry.i_q_des` | 54 | float | r | A | — | — | 否 | q 轴电流给定 |
| `telemetry.fw_int` | 55 | float | r | A | — | — | 否 | 弱磁积分量 |
| `telemetry.fault` | 56 | uint32 | r |  | — | — | 否 | 锁存的故障位（FAULT_x） |
| `telemetry.gate_drive` | 57 | bool | r |  | — | — | 否 | 门极是否使能 |
| `telemetry.mode` | 58 | uint8 | r |  | — | — | 否 | 当前 FSM 状态（MENU_MODE 等） |
| `telemetry.loop_count` | 59 | uint32 | r |  | — | — | 否 | 控制周期计数 |
| `telemetry.can_timeout_count` | 60 | int32 | r |  | — | — | 否 | 距上一帧控制命令的控制周期数（40 拍 = 1 ms，饱和在 can_timeout 对应值） |
| `telemetry.temperature` | 61 | float | r | C | — | — | 否 | 温度观测器估计值 |
| `telemetry.resistance` | 62 | float | r | Ohm | — | — | 否 | 温度观测器估计电阻 |
| `telemetry.can_tec` | 63 | uint8 | r |  | — | — | 否 | CAN 发送错误计数 |
| `telemetry.can_rec` | 64 | uint8 | r |  | — | — | 否 | CAN 接收错误计数 |
| `telemetry.can_lec` | 65 | uint8 | r |  | — | — | 否 | CAN 最后错误码 |
| `telemetry.encoder_ppairs` | 66 | float | r |  | — | — | 否 | 编码器当前使用的极对数 |
| `telemetry.encoder_lost_frames` | 67 | int32 | r |  | — | — | 否 | 编码器丢帧计数 |
| `telemetry.encoder_raw` | 79 | int32 | r |  | — | — | 否 | 编码器原始读数 |
| `telemetry.encoder_turns` | 80 | int32 | r |  | — | — | 否 | 编码器多圈数 |
| `telemetry.encoder_elec_angle` | 81 | float | r | rad | — | — | 否 | 编码器电角度 |
| `telemetry.encoder_elec_velocity` | 82 | float | r | rad/s | — | — | 否 | 编码器电角速度 |
| `telemetry.dtc_u` | 83 | float | r |  | — | — | 否 | U 相占空比 |
| `telemetry.dtc_v` | 84 | float | r |  | — | — | 否 | V 相占空比 |
| `telemetry.dtc_w` | 85 | float | r |  | — | — | 否 | W 相占空比 |
| `telemetry.i_mag_max` | 86 | float | r | A | — | — | 否 | 电流矢量幅值上限 |
| `telemetry.i_scale` | 87 | float | r | A/count | — | — | 否 | 相电流标定系数 |
| `telemetry.flux_linkage` | 88 | float | r | Wb | — | — | 否 | 磁链 λ（标定得出；由 KT 与极对数换算） |
| `telemetry.ld_lq_diff` | 135 | float | r | H | — | — | 否 | 感差 Ld-Lq（标定得出；凸极电机为负） |
| `telemetry.v_setpoint` | 146 | float | r | rad/s | — | — | 否 | 速度环实际使用的设定值（已过速度限幅与 accel_limit 斜率限制，输出轴） |
| `telemetry.vel_integral` | 147 | float | r | A | — | — | 否 | 速度环积分器状态（电机端安培）；用它判断是积分在顶负载还是增益在振荡 |
| `telemetry.pos_err` | 148 | float | r | rad | — | — | 否 | 位置环误差（输出轴，最近一次位置环更新的值） |
| `telemetry.vel_err` | 149 | float | r | rad/s | — | — | 否 | 速度环误差（输出轴，最近一次速度环更新的值） |
| `telemetry.velocity_pll` | 150 | float | r | rad/s | — | — | 否 | 速度环/位置环使用的编码器 PLL 速度（输出轴）；与 telemetry.velocity 对比可看差分估计的噪声 |
| `telemetry.pos_error_fault_suggest` | 152 | float | r | rad | — | — | 否 | config.pos_error_fault 的推荐值 = min(½·a_max·0.1², ½·v_max/pos_gain)，其中 a_max=KT·GR·i_max/inertia；inertia 未知时只用后半项 |
| `telemetry.adc_a_offset` | 89 | int32 | r |  | — | — | 否 | A 相电流零偏 |
| `telemetry.adc_b_offset` | 90 | int32 | r |  | — | — | 否 | B 相电流零偏 |
| `telemetry.adc_c_offset` | 91 | int32 | r |  | — | — | 否 | C 相电流零偏 |
| `telemetry.adc_a_raw` | 92 | int32 | r |  | — | — | 否 | A 相电流 ADC 原始值 |
| `telemetry.adc_b_raw` | 93 | int32 | r |  | — | — | 否 | B 相电流 ADC 原始值 |
| `telemetry.adc_c_raw` | 94 | int32 | r |  | — | — | 否 | C 相电流 ADC 原始值 |
| `telemetry.observer_qd_in` | 95 | float | r |  | — | — | 否 | 温度观测器热输入 |
| `telemetry.observer_qd_out` | 96 | float | r |  | — | — | 否 | 温度观测器热输出 |
| `telemetry.observer_k` | 97 | float | r |  | — | — | 否 | 温度观测器增益 |
| `telemetry.observer_trust` | 98 | float | r |  | — | — | 否 | 温度观测器置信度 |
| `telemetry.observer_delta_t` | 99 | float | r | C | — | — | 否 | 温度观测器温升 |
| `telemetry.inertia_state` | 155 | uint8 | r |  | — | — | 否 | 惯量标定状态机的阶段（0 空闲 / 2 试探 / 3 刹车 / 4 台阶 LO / 5 台阶 HI / 6 计算 / 7 失败） |
| `telemetry.inertia_break_current` | 156 | float | r | A | — | — | 否 | 标定试探得到的"摩擦+重力"等效电流；台阶电流取它的 1.5×/2.5× |
| `telemetry.inertia_accel_lo` | 157 | float | r | rad/s^2 | — | — | 否 | 台阶 LO 的拟合加速度（输出轴） |
| `telemetry.inertia_accel_hi` | 158 | float | r | rad/s^2 | — | — | 否 | 台阶 HI 的拟合加速度（输出轴）；与 LO 的差决定 J |

## timing：ISR 耗时统计（只读）

| 名称 | ID | 类型 | 访问 | 单位 | 范围 | 默认 | 持久化 | 说明 |
|---|---|---|---|---|---|---|---|---|
| `timing.isr_period` | 100 | uint32 | r |  | — | — | 否 | 控制中断周期（CPU cycles） |
| `timing.count` | 101 | uint32 | r |  | — | — | 否 | 已统计的中断周期数 |
| `timing.overrun` | 102 | uint32 | r |  | — | — | 否 | 错过的控制周期数 |
| `timing.period_last` | 103 | uint32 | r |  | — | — | 否 | 上一次中断周期 |
| `timing.period_min` | 104 | uint32 | r |  | — | — | 否 | 最小中断周期 |
| `timing.period_max` | 105 | uint32 | r |  | — | — | 否 | 最大中断周期 |
| `timing.total_last` | 106 | uint32 | r |  | — | — | 否 | 上次中断总耗时 |
| `timing.total_min` | 107 | uint32 | r |  | — | — | 否 | 最小中断总耗时 |
| `timing.total_max` | 108 | uint32 | r |  | — | — | 否 | 最大中断总耗时 |
| `timing.seg_adc_max` | 109 | uint32 | r |  | — | — | 否 | ADC 采样段最大耗时 |
| `timing.seg_enc_max` | 110 | uint32 | r |  | — | — | 否 | 编码器读取段最大耗时 |
| `timing.seg_faults_max` | 111 | uint32 | r |  | — | — | 否 | 故障检查段最大耗时 |
| `timing.seg_fsm_max` | 112 | uint32 | r |  | — | — | 否 | FSM/FOC 段最大耗时 |
| `timing.seg_can_max` | 113 | uint32 | r |  | — | — | 否 | CAN 收发段最大耗时 |
| `timing.ext_enc_wait_max` | 114 | uint32 | r |  | — | — | 否 | 编码器 SPI/DMA 等待最大耗时 |
| `timing.ext_adc_wait_max` | 115 | uint32 | r |  | — | — | 否 | 注入 ADC 等待最大耗时 |

## system：状态与版本（只读）

| 名称 | ID | 类型 | 访问 | 单位 | 范围 | 默认 | 持久化 | 说明 |
|---|---|---|---|---|---|---|---|---|
| `system.version` | 68 | float | r |  | — | — | 否 | 固件版本号 |
| `system.prefs_sequence` | 69 | uint32 | r |  | — | — | 否 | 当前配置块的序号 |
| `system.config_schema` | 117 | uint16 | r |  | — | — | 否 | 配置记录的格式版本（PREFS_SCHEMA），用于升级迁移 |
| `system.prefs_loaded` | 70 | bool | r |  | — | — | 否 | 是否从 flash 成功加载过配置 |
| `system.inertia_status` | 153 | uint8 | r |  | — | — | 否 | 惯量标定状态：0 未跑 / 1 运行中（约 1.5 s，含进出电机模式）/ 2 成功（结果在 config.inertia）/ 3 关节本来就在动 / 4 突破电流超时（卡住或摩擦过大）/ 5 台阶太短（样本不足）或超时 / 6 激励不足（斜率不可信）/ 7 进不了电机模式 |
| `system.save_status` | 119 | uint8 | r |  | — | — | 否 | 最近一次写 flash 的结果（save/zero/reset_config/标定共用）：0 未执行、1 成功、2 因门极使能跳过、3 写失败 |
| `system.can_prot_active` | 123 | int32 | r |  | — | — | 否 | 当前生效的 CAN 协议（0=旧 11-bit，1=CYBERBEAST） |
| `system.can_fd` | 124 | bool | r |  | — | — | 否 | 当前是否以 CAN FD 运行 |
| `system.can_baud_active` | 125 | int32 | r | bit/s | — | — | 否 | 当前生效的 CAN 速率（Classic 为位速率；FD 为数据域速率） |

## action：动作（只写）

| 名称 | ID | 类型 | 访问 | 单位 | 范围 | 默认 | 持久化 | 说明 |
|---|---|---|---|---|---|---|---|---|
| `action.save` | 71 | function | w |  | — | — | 否 | 把当前配置写入 flash（要求门极关闭） |
| `action.zero` | 72 | function | w |  | — | — | 否 | 把当前位置设为零点并保存（仅菜单状态） |
| `action.idle_mode` | 73 | function | w |  | — | — | 否 | 空闲/停机：关门极、清锁存故障、回到菜单（退出电机模式、急停用） |
| `action.clear_fault` | 74 | function | w |  | — | — | 否 | 清除锁存故障（等价于 action.idle_mode） |
| `action.motor_mode` | 75 | function | w |  | — | — | 否 | 进入电机模式（仅菜单状态） |
| `action.calibrate` | 76 | function | w |  | — | — | 否 | 进入标定（相序/极对数、编码器零位与 LUT、R/L/Ld-Lq/磁链；仅菜单状态） |
| `action.reset_config` | 126 | function | w |  | — | — | 否 | 恢复出厂默认值并写入 flash（要求门极关闭；会清掉标定，需重新标定并重启） |
| `action.reboot` | 127 | function | w |  | — | — | 否 | 重启控制器（应答发出后约 0.5 s 复位，配置按已保存的值重新加载） |
| `action.reset_timing` | 116 | function | w |  | — | — | 否 | 清零 ISR 耗时统计 |
| `action.measure_inertia` | 154 | function | w |  | — | — | 否 | 一条指令完成惯量标定：自动进电机模式 → 打两个电流台阶测输出端惯量 → 回菜单。约 1.5 s、关节行程 ~0.1 rad。结果写 config.inertia（**只写 RAM，不自动保存**），进度/结果读 system.inertia_status；测完自动回菜单，可直接 action.save |
| `action.tune_gains` | 161 | function | w |  | — | — | 否 | 用 config.inertia 算外环增益并写进 RAM（不自动保存）：vel_gain = tune_margin·J·omega_c/(KT·GR)；vel_integrator_gain = vel_gain·clamp(omega_c/25, 5, 30)；pos_gain = omega_c/20。写完用 csv/csp 子命令验证，满意再 action.save |
