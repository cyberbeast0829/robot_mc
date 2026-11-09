<!-- 由 tools/gen_endpoints.py 从 params.json 生成，请勿手工修改。 -->
# 参数手册

ID 由 `tools/endpoint_ids.json` 锁定，固件升级不会改变；`access` 为 `r` 的只能读，
`w` 的只能写/执行，`rw` 可读写。写入值在固件里做 NaN 检查和上下限限幅。
`function` 就是 `action.*` 动作端点：读和写都会执行（ODrive 的 endpoint 协议
不区分读写，兼容按 `PARAM_READ` 调用动作的上位机）。`默认` 列偶尔是固件宏（如 `I_MAX_LIMIT`），
以固件为准。

## config：可写参数

| 名称 | ID | 类型 | 访问 | 单位 | 范围 | 默认 | 持久化 | 说明 |
|---|---|---|---|---|---|---|---|---|
| `config.i_bw` | 1 | float | rw | Hz | 100.0 .. 2000.0 | 1000.0 | 是 | 电流环目标穿越频率（Hz）：P/I 增益由标定的 config.r_phase / config.l_d / config.l_q 与 V_PER_CMD 推导（极点对消），与电机无关，写入后立即重算、无需重启（未标定 r_phase=0 时退回编译期常量）。实际穿越约 0.76×（2 拍 ADC→占空比延迟）：1000→762 Hz/PM 78°，2000→1650 Hz/PM 59° |
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
| `config.vel_gain` | 137 | float | rw | A/(rad/s) | 0.0 .. 2000.0 | 2.158127 | 是 | CSV/CSP 速度环 P 增益（电机端安培 / 输出轴角速度）；0 时速度环只剩积分器（要完全关闭请把 vel_integrator_gain 也设 0） |
| `config.vel_integrator_gain` | 138 | float | rw | A/(rad/s)/s | 0.0 .. 200000.0 | 38.846287 | 是 | 速度环 I 增益；消掉重力/摩擦下的稳态速度误差（抗饱和：输出限幅时积分按 0.99 衰减） |
| `config.pos_ff_gain` | 139 | float | rw |  | 0.0 .. 2.0 | 1.0 | 是 | CSP 帧间速度前馈系数（指令差分的比例）；1.0 = 全前馈，0 = 关闭 |
| `config.accel_limit` | 140 | float | rw | rad/s^2 | 0.0 .. 100000.0 | 50.0 | 是 | 速度设定值的**加速**斜率限制（输出轴），input_mode=2 时生效；TRAP_TRAJ（5）优先用自己的 config.trap_accel_limit。0 = 关闭。非零默认值能在指令阶跃时压住电流冲击，对稳态无影响 |
| `config.decel_limit` | 181 | float | rw | rad/s^2 | 0.0 .. 100000.0 | 50.0 | 是 | 速度设定值的**减速**斜率限制（输出轴）；0 = 与 accel_limit 相同（不单独限制）。input_mode=2 时生效 |
| `config.vel_ramp_rate` | 210 | float | rw | rad/s^2 | 0.0 .. 100000.0 | 0.0 | 是 | input_mode=2（速度斜坡）的对称斜率（对应 ODrive 的 controller.config.vel_ramp_rate）：>0 时加速与减速都用它；0 = 沿用 accel_limit / decel_limit |
| `config.trap_vel_limit` | 211 | float | rw | rad/s | 0.0 .. 200.0 | 0.0 | 是 | input_mode=5（梯形规划）的巡航速度上限（输出轴），与运行期限速取较小值；0 = 用 config.v_max / 帧内 vel_limit |
| `config.trap_accel_limit` | 212 | float | rw | rad/s^2 | 0.0 .. 100000.0 | 0.0 | 是 | input_mode=5 的加速度；0 = 用 config.accel_limit |
| `config.trap_decel_limit` | 213 | float | rw | rad/s^2 | 0.0 .. 100000.0 | 0.0 | 是 | input_mode=5 的减速度；0 = 用 config.decel_limit（再为 0 时用 accel_limit） |
| `config.input_mode` | 182 | int32 | rw |  | 0 .. 9 | 2 | 是 | 输入整形模式（与 ODrive InputMode 编号一致）：0=忽略输入保持、1=直通、2=速度斜率（默认）、3=位置跟踪滤波、5=梯形位置规划、6=转矩斜坡、9=MIT(同直通)；4/7/8 不支持，按直通处理。它只整形设定值，控制环仍由 command.mode 选择 |
| `config.input_bw` | 183 | float | rw | 1/s | 1.0 .. 200.0 | 10.0 | 是 | input_mode=3（位置跟踪滤波）的跟踪带宽；离散稳定限制在 4 kHz 位置环上约 45 1/s，超了固件会自动压低增益 |
| `config.torque_ramp_rate` | 184 | float | rw | N-m/s | 0.0 .. 10000.0 | 1.0 | 是 | input_mode=6（转矩斜坡）下前馈转矩的变化率；0 = 不限（直通）。只在阻抗/MIT 模式下生效 |
| `config.enable_vel_limit` | 185 | bool | rw |  | — | 1 | 是 | 速度设定值限幅总开关：false 时 CSV/CSP/帧内 vel_limit 都不再受 config.v_max 约束（此时 command.v_des 的物理范围是 ±200 rad/s）。默认 true |
| `config.enable_torque_vel_lim` | 186 | bool | rw |  | — | 0 | 是 | 阻抗/力矩/电流模式下的速度限制开关：打开后按 i_q <= (v_max - v)·vel_gain 限制推力，防止超过 v_max（默认关，与改动前行为一致） |
| `config.pos_error_fault` | 141 | float | rw | rad | 0.0 .. 1000.0 | 0.0 | 是 | 位置跟随误差故障阈值（输出轴）；0 = 关闭。只在位置环未被速度限幅饱和时判定，需持续 100 ms；必须小于 v_max/pos_gain 才会触发 |
| `config.pll_bandwidth` | 142 | float | rw | rad/s | 20.0 .. 5000.0 | 2000.0 | 是 | 编码器速度 PLL 带宽（速度环/位置环的反馈来源）；越大相位滞后越小、噪声越大。默认 2000（约 318 Hz），200 Hz 穿越点只吃 -13° 相位 |
| `config.inertia` | 151 | float | rw | kg·m^2 | 0.0 .. 100.0 | 0.00096 | 是 | 输出端惯量（关节折算值）。可由 action.measure_inertia 测得（只写 RAM，不自动保存）；用于推荐增益与 pos_error_fault。0 = 未知。默认 0.00096 来自 2026-11-06 台架（GR 9、KT 0.0334）：5 次 action 各自的中位数落在 0.000865~0.000967（极差 11%、σ 3.7%），取 0.00096 |
| `config.omega_c` | 159 | float | rw | rad/s | 50.0 .. 1200.0 | 450.0 | 是 | action.tune_gains 的目标穿越频率：决定算出来的 vel_gain / pos_gain 有多激进。受延迟限制，超过约 1000 rad/s 实际是在跟延迟赛跑，建议 300~600 起步 |
| `config.tune_margin` | 160 | float | rw | x | 0.2 .. 5.0 | 1.5 | 是 | action.tune_gains 对速度环增益的放大系数。线性公式给的是惯性主导时的值，摩擦/齿隙大的关节要放大（旧电机/旧电流环约 2.6）；只放大速度环，pos_gain 不受影响。1 = 纯公式。2026-11-06 台架（GR 9、电流环按标定 R/L 推导后）：1.5 给出 vel_gain 2.16，0.2 rad 位置阶跃与 1 rad/s 速度阶跃均干净（速度超调 3~7%、位置残差 <0.5 mrad），1.7 起速度超调 >25%，2.2 失稳 |
| `config.vel_loop_prescaler` | 143 | int32 | rw |  | 1 .. 100 | 5 | 是 | 速度环分频：控制环 40 kHz / 该值 = 速度环频率（默认 5 → 8 kHz）；1 = 全速 40 kHz |
| `config.pos_loop_prescaler` | 144 | int32 | rw |  | 1 .. 100 | 2 | 是 | 位置环分频：速度环 / 该值 = 位置环频率（默认 2 → 4 kHz）；位置环只在速度环执行的拍里执行 |
| `config.gr` | 14 | float | rw |  | ≥ 0.001 | 9.0 | 是 | 减速比：转子圈 / 输出圈（门极关闭时才允许写）。CBL89 + MZW55 实测 9.0 |
| `config.kt` | 15 | float | rw | N-m/A | ≥ 0.0001 | 1.0 | 是 | 转矩常数（标定得出；门极关闭时才允许写） |
| `config.ppairs` | 16 | float | rw |  | 1.0 .. 64.0 | 14.0 | 是 | 极对数（标定得出；门极关闭时才允许写） |
| `config.r_phase` | 17 | float | rw | Ohm | — | 0.0 | 是 | 单相电阻（标定得出） |
| `config.l_d` | 133 | float | rw | H | 1e-06 .. 0.01 | 9.5e-05 | 是 | D 轴电感（标定得出；门极关闭时才允许写） |
| `config.l_q` | 134 | float | rw | H | 1e-06 .. 0.01 | 0.000124 | 是 | Q 轴电感（标定得出；门极关闭时才允许写） |
| `config.r_nominal` | 18 | float | rw | Ohm | — | 0.0 | 是 | 标称电机电阻 |
| `config.r_th` | 19 | float | rw | C/W | — | 0.0 | 是 | 热阻 |
| `config.c_th` | 20 | float | rw | J/C | — | 0.0 | 是 | 热容 |
| `config.temp_max` | 21 | float | rw | C | 0.0 .. 150.0 | 125.0 | 是 | 历史参数（温度观测器的短路阈值），当前未参与任何控制；温度保护请用 config.fet_temp_* / config.mot_temp_* |
| `config.phase_order` | 22 | int32 | rw |  | 0 .. 1 | 0 | 是 | 相序（标定得出；门极关闭时才允许写） |
| `config.can_id` | 23 | int32 | rw |  | 0 .. 127 | 1 | 是 | CAN 节点 ID |
| `config.can_master` | 24 | int32 | rw |  | 0 .. 127 | 0 | 是 | CAN 主站 ID |
| `config.can_timeout` | 25 | int32 | rw | ms | 0 .. 100000 | 100 | 是 | CAN 命令超时（毫秒，0 = 关闭）；超时后清零 p_des/v_des/kp/kd/t_ff |
| `config.can_prot` | 120 | int32 | rw |  | 0 .. 1 | 1 | 是 | CAN 协议（0=旧 11-bit 协议，1=CYBERBEAST；下次上电生效） |
| `config.can_baud` | 121 | int32 | rw | bit/s | 50000 .. 5000000 | 1000000 | 是 | CAN 波特率：≤1M 为 Classic CAN（该速率），>1M 为 CAN FD（仲裁域固定 1M，数据域=本值）；下次上电生效 |
| `config.can_hb_ms` | 122 | int32 | rw | ms | 0 .. 10000 | 100 | 是 | CYBERBEAST 心跳周期（0 = 关闭） |
| `config.can_tdc` | 162 | int32 | rw | tq | -1 .. 31 | -1 | 是 | CAN FD 发送延迟补偿偏移（数据域 tq）：-1 = 自动（= 数据域 tseg1，默认），0 = 关闭，>0 = 手动；5M 必须开，下次上电生效 |
| `config.m_zero` | 26 | int32 | rw |  | — | 0 | 是 | 机械零位编码器计数（门极关闭时才允许写）。会同时平移双编参考（ps_set_m_zero），所以断电再上电的位置仍然连续 |
| `config.e_zero` | 27 | int32 | rw |  | — | 0 | 是 | 电角度零点编码器计数（门极关闭时才允许写） |
| `config.vbus_ov_trip` | 163 | float | rw | V | 10.0 .. 100.0 | 67.0 | 是 | 母线过压硬故障阈值；超过即锁存 FAULT_OV 并关闭门极（原编译期常量 V_BUS_FAULT） |
| `config.vbus_uv_trip` | 164 | float | rw | V | 0.0 .. 100.0 | 0.0 | 是 | 母线欠压故障阈值（0 = 关闭）；只在门极使能时判定，并带 ~10 ms 去抖 |
| `config.i_dc_max` | 165 | float | rw | A | 0.0 .. 100.0 | 0.0 | 是 | 母线电流软限幅（0 = 关闭）。硬件没有直流分流器，按 P=1.5·V_PER_CMD·(v_d·i_d+v_q·i_q) 估算，只做软限流、不锁故障 |
| `config.torque_lim` | 166 | float | rw | N-m | 0.0 .. 100.0 | 0.0 | 是 | 输出端转矩上限（0 = 关闭）；与 command.t_ff / telemetry.torque 同一量纲，内部换算 i_q_lim = torque_lim/(KT·GR) |
| `config.fet_temp_en` | 167 | bool | rw |  | — | 0 | 是 | MOS/FET 温度保护开关（默认关，关闭时温度只上报不参与控制） |
| `config.fet_temp_lo` | 168 | float | rw | C | 0.0 .. 200.0 | 75.0 | 是 | MOS 温度降流起点：达到该温度开始线性压低电流上限 |
| `config.fet_temp_hi` | 169 | float | rw | C | 0.0 .. 200.0 | 95.0 | 是 | MOS 温度上限：达到该温度电流上限降为 0，再超过 5 °C 锁存 FAULT_FET_OT |
| `config.mot_temp_en` | 170 | bool | rw |  | — | 0 | 是 | 电机温度保护开关（默认关，关闭时温度只上报不参与控制） |
| `config.mot_temp_lo` | 171 | float | rw | C | 0.0 .. 200.0 | 75.0 | 是 | 电机温度降流起点 |
| `config.mot_temp_hi` | 172 | float | rw | C | 0.0 .. 200.0 | 95.0 | 是 | 电机温度上限：达到该温度电流上限降为 0，再超过 5 °C 锁存 FAULT_MOT_OT |
| `config.pre_calibrated` | 173 | bool | rw |  | — | 0 | 是 | 电机/编码器是否已标定：false 时拒绝进入电机模式（action.calibrate 成功后自动置 1，进入标定或恢复默认时清 0；门极使能时不可写） |
| `config.gear_teeth_first` | 195 | int32 | rw |  | 1 .. 400 | 30 | 是 | 齿轮式双编：转子（主编码器）侧齿轮齿数 T1。量程 = ±T2/(2\|T1-T2\|) 转子圈（30/27 给 ±4.5），所以要接近但不等；成比例齿数（10/9 = 20/18 = 30/27）行为相同 |
| `config.gear_teeth_sec` | 196 | int32 | rw |  | 1 .. 400 | 27 | 是 | 齿轮式双编：副编码器（输出轴侧）齿轮齿数 T2。断电期间转子移动超过 ±T2/(2\|T1-T2\|) 圈就会差整整一个相位周期 T2/\|T1-T2\| |
| `config.gear_sec_reversed` | 208 | bool | rw |  | — | 1 | 是 | 副编码器读数方向：false = 读数随转子正转增大；true = 随正转变小（副轴外啮合反向 / 磁铁反装），固件解算前先做 SEC_ENC_CPR-raw 镜像。CBL89 + MZW55 实测副轴反向，默认 true；改这个值后要重新 action.gear_capture |
| `config.gear_zero_phase` | 198 | float | rw |  | 0.0 .. 1.0 | 0.0 | 是 | 齿轮式双编的相位参考（action.gear_capture 写入，不要手改）：标定时刻两路编码器的韦尼尔相位，上电时据此解出转子第几圈 |
| `config.gear_zero_count` | 199 | int32 | rw |  | — | 0 | 是 | 齿轮式双编的计数参考（action.gear_capture 写入，不要手改）：标定时刻的多圈计数 turns*CPR + count_wrapped，与 gear_zero_phase 配对使用 |
| `config.gear_ref_valid` | 200 | bool | rw |  | — | 0 | 是 | 齿轮式双编参考是否有效：true 才在上电时用副编码器恢复输出轴位置（action.gear_capture 置 1，恢复默认值清 0）。没有第二编码器的板子保持 false 即可，行为与改动前一致 |
| `config.cogging_en` | 218 | bool | rw |  | — | 0 | 是 | 齿槽转矩补偿开关。只有 config.cogging_valid 为 true 时才真正生效（表没标定过就打开会被忽略）。建议先用 action.cogging_calibrate 标定、验证，再打开 |
| `config.cogging_valid` | 219 | bool | rw |  | — | 0 | 是 | 齿槽补偿表是否有效（标定成功后置 1，action.cogging_reset 或恢复出厂值清 0）。手动清 0 等于停用补偿但保留表，重新 action.save 也不会再自动置 1 |
| `config.cogging_gain` | 220 | float | rw | x | 0.0 .. 2.0 | 1.0 | 是 | 齿槽补偿缩放：1.0 = 用标定值原样前馈。偏小（0.7~0.9）更保守；大于 1 只在确认标定低估时用 |
| `config.cogging_max` | 221 | float | rw | A | 0.0 .. 10.0 | 1.0 | 是 | 前馈电流钳位（A，0 = 不使用补偿）。用来兜住一张错误的表，正常齿槽补偿只有零点几安，取 1~2 A 足够 |
| `config.cogging_pos_thr` | 222 | float | rw | rad | 1e-05 .. 0.01 | 0.0003 | 是 | 标定收敛阈值（输出轴 rad）：位置误差连续低于它 cogging_settle_ms 毫秒后才开始采样。建议 ≤ 半个表距 2π/(1024·GR)，本机（GR=9）约 0.0007 rad |
| `config.cogging_settle_ms` | 223 | int32 | rw | ms | 0 .. 500 | 40 | 是 | 标定：每个点位置进入阈值后还要保持多久才开始采样（默认 40 ms）。调大更稳、标定更慢 |
| `config.cogging_avg` | 224 | int32 | rw | cycle | 1 .. 400 | 80 | 是 | 标定：每个点采样平均的控制周期数（40 kHz，默认 80 = 2 ms）。调大抑制噪声、标定更慢 |
| `config.cogging_detrend` | 237 | int32 | rw |  | 0 .. 6 | 3 | 是 | 标定后去掉一个 N 阶多项式趋势（0 = 不去）。重力/外载正反两遍抵消不了，而它在一个转子圈里是慢变趋势（本机 GR=9，只走 1/9 输出圈 ≈ 40°），三次多项式就能拟合掉；齿槽是 84/168… 次/转，不会被误删。有重力负载的关节保持默认 3；直接驱动（GR=1）用 5~6；确认无外载可设 0 |
| `config.cogging_method` | 238 | int32 | rw |  | 0 .. 1 | 1 | 是 | 标定方法：1 = 匀速扫描（默认，约 10 s）—— 转子匀速转一圈再反向转回来，按转子位置分箱累积速度环输出；动摩擦是常值且两向相反，平均后抵消，没有静摩擦误差。0 = 逐点停住读积分器 —— 测的是纯静态力矩，但静摩擦（本机粘滞带 ±0.15 A）会把 0.02 A 量级的齿槽信号淹没，台架实测幅值偏大 3 倍、相位差 60°，不推荐 |
| `config.cogging_sweep_ms` | 239 | int32 | rw | ms | 500 .. 20000 | 8000 | 是 | 扫描法：一个转子机械圈走多少毫秒（默认 8000 = 0.125 圈/s，齿槽基波约 3~15 Hz，远低于速度环带宽）。台架实测：2000/4000/8000/16000 对应两遍差异 0.243/0.111/0.043/0.032 A（信号约 0.055 A）；已经加了 90 Hz 二阶低通，默认值够用。调慢更干净但更久（两圈合计约 2×该值） |
| `config.cogging_index` | 225 | int32 | rw |  | 0 .. 1023 | 0 | 否 | 诊断游标：配合 telemetry.cogging_value 逐个读表（tools/cogging_test.py dump 用），不持久化 |

## command：运行命令（电机模式下有效）

| 名称 | ID | 类型 | 访问 | 单位 | 范围 | 默认 | 持久化 | 说明 |
|---|---|---|---|---|---|---|---|---|
| `command.t_ff` | 128 | float | rw | N-m | 运行期：`-I_MAX*KT*GR` .. `I_MAX*KT*GR` | — | 否 | 前馈转矩（输出轴 N·m）；写入会刷新命令租约，进入 MOTOR_MODE 时清零 |
| `command.p_des` | 129 | float | rw | rad | 运行期：`P_MIN` .. `P_MAX` | — | 否 | 位置设定值（输出轴 rad）；范围由 config.p_max 决定，进入 MOTOR_MODE 时清零 |
| `command.v_des` | 130 | float | rw | rad/s | -200.0 .. 200.0 | — | 否 | 速度设定值（输出轴 rad/s），进入 MOTOR_MODE 时清零。运行期限幅：config.enable_vel_limit=true 时压到 ±config.v_max，false 时只受 ±200 rad/s 的物理范围约束 |
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
| `telemetry.temperature` | 61 | float | r | C | — | — | 否 | MOS/FET 温度（NTC 换算；NTC 未接或系数不符时读数不可信） |
| `telemetry.mos_temp` | 174 | float | r | C | — | — | 否 | MOS/FET 温度（MOS_TEMP / ADC2_CH2） |
| `telemetry.mot_temp` | 175 | float | r | C | — | — | 否 | 电机温度（MOT_TEMP / ADC3_CH6） |
| `telemetry.i_bus` | 176 | float | r | A | — | — | 否 | 估算母线电流（无直流分流器，由 dq 功率除以母线电压得到，仅用于监控与软限幅） |
| `telemetry.i_d_des` | 177 | float | r | A | — | — | 否 | d 轴电流给定（对应 ODrive 的 Id_setpoint；Iq_setpoint 见 telemetry.i_q_des） |
| `telemetry.mechanical_power` | 178 | float | r | W | — | — | 否 | 机械功率 = 输出端转矩 × 输出轴角速度（>0 输出机械功，<0 回馈） |
| `telemetry.sensor_mode` | 179 | uint8 | r |  | — | — | 否 | 位置传感器模式：本固件只有 SPI 编码器 + PLL，恒为 0（无无感/混合估计器） |
| `telemetry.resistance` | 62 | float | r | Ohm | — | — | 否 | 温度观测器估计电阻 |
| `telemetry.can_tec` | 63 | uint8 | r |  | — | — | 否 | CAN 发送错误计数 |
| `telemetry.can_rec` | 64 | uint8 | r |  | — | — | 否 | CAN 接收错误计数 |
| `telemetry.can_lec` | 65 | uint8 | r |  | — | — | 否 | CAN 最后错误码 |
| `telemetry.encoder_ppairs` | 66 | float | r |  | — | — | 否 | 编码器当前使用的极对数 |
| `telemetry.encoder_lost_frames` | 67 | int32 | r |  | — | — | 否 | 编码器丢帧计数 |
| `telemetry.sec_enc_raw` | 201 | int32 | r |  | — | — | 否 | 齿轮式双编副编码器（输出轴侧）的原始读数，最近一次探测 / action.gear_capture / action.gear_read 时读到 |
| `telemetry.sec_enc_present` | 209 | bool | r |  | — | — | 否 | 副编码器是否在线（上电、进入标定、gear_capture/gear_read 时探测；判据是连续两帧读到有效值，MISO 恒定的板子会判成 false） |
| `telemetry.sec_enc_phase` | 202 | float | r |  | — | — | 否 | 齿轮式双编的韦尼尔相位 Δ ∈ [0,1)（最近一次读数）。转子每转一圈变化约 (T1-T2)/T2 |
| `telemetry.sec_enc_lost_frames` | 203 | int32 | r |  | — | — | 否 | 副编码器读取失败次数（没插、接线断，或被控制中断的 SPI 访问打断） |
| `telemetry.gear_abs_turns` | 204 | float | r | turn | — | — | 否 | 齿轮式双编解算出的转子绝对圈数（含小数，与 telemetry.position_motor 同一坐标系）。上电恢复时写入，action.gear_read 时刷新 |
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
| `telemetry.current_k_d` | 214 | float | r | V/A | — | — | 否 | 电流环 d 轴 P 增益（指令伏/A，由 config.i_bw 与标定的 l_d 推导，见 current_loop_retune()） |
| `telemetry.current_k_q` | 215 | float | r | V/A | — | — | 否 | 电流环 q 轴 P 增益（指令伏/A）；凸极电机 q 轴电感更大，所以 k_q > k_d |
| `telemetry.current_ki_d` | 216 | float | r |  | — | — | 否 | 电流环 d 轴积分零点（rad/sample）= 1-exp(-r_phase*DT/l_d)，与带宽无关 |
| `telemetry.current_ki_q` | 217 | float | r |  | — | — | 否 | 电流环 q 轴积分零点（rad/sample）= 1-exp(-r_phase*DT/l_q) |
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
| `telemetry.cogging_ff` | 226 | float | r | A | — | — | 否 | 本拍实际叠加到 i_q_des 上的齿槽补偿电流（0 = 未生效：没标定、config.cogging_en=false，或正在标定） |
| `telemetry.cogging_value` | 227 | float | r | A | — | — | 否 | 齿槽补偿表在 config.cogging_index 处的值（A，已去掉直流分量）。标定点 i 对应转子绝对计数 i·ENC_CPR/1024 |
| `telemetry.cogging_pass1` | 240 | float | r | A | — | — | 否 | 诊断：最近一次标定正转那一遍在 config.cogging_index 处的原始测量值（未去均值/去趋势）。与 cogging_pass2 对比可以看出重复性 |
| `telemetry.cogging_pass2` | 241 | float | r | A | — | — | 否 | 诊断：最近一次标定反转那一遍在 config.cogging_index 处的原始测量值 |

## timing：ISR 耗时统计（只读）

| 名称 | ID | 类型 | 访问 | 单位 | 范围 | 默认 | 持久化 | 说明 |
|---|---|---|---|---|---|---|---|---|
| `timing.isr_period` | 100 | uint32 | r |  | — | — | 否 | 控制中断周期（CPU cycles） |
| `timing.count` | 101 | uint32 | r |  | — | — | 否 | 已统计的中断周期数 |
| `timing.overrun` | 102 | uint32 | r |  | — | — | 否 | 错过的控制周期数 |
| `timing.period_last` | 103 | uint32 | r |  | — | — | 否 | 上一次中断周期（CPU cycles，正常应约等于 timing.isr_period） |
| `timing.period_min` | 104 | uint32 | r |  | — | — | 否 | 最小中断周期（CPU cycles）；0xFFFFFFFF = 尚未采样 |
| `timing.period_max` | 105 | uint32 | r |  | — | — | 否 | 最大中断周期（CPU cycles）；约为 isr_period 的整数倍表示漏拍 |
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
| `system.fw_version_major` | 187 | uint8 | r |  | — | — | 否 | 固件主版本号（对应 ODrive 的 fw_version_major） |
| `system.fw_version_minor` | 188 | uint8 | r |  | — | — | 否 | 固件次版本号（对应 ODrive 的 fw_version_minor） |
| `system.fw_version_revision` | 189 | uint8 | r |  | — | — | 否 | 固件修订号（对应 ODrive 的 fw_version_revision） |
| `system.fw_version_unreleased` | 194 | uint8 | r |  | — | — | 否 | 1 = 非正式发布构建（ODrive 的 fw_version_unreleased）；0 = 正式版 |
| `system.hw_version_major` | 190 | uint8 | r |  | — | — | 否 | 硬件主版本号（对应 ODrive 的 hw_version_major；CBL89 板，定义在 hw_config.h） |
| `system.hw_version_minor` | 191 | uint8 | r |  | — | — | 否 | 硬件次版本号 |
| `system.hw_version_variant` | 192 | uint8 | r |  | — | — | 否 | 硬件装配变体号 |
| `system.serial_number` | 193 | uint64 | r |  | — | — | 否 | 芯片 96 位唯一 ID 的低 64 位（对应 ODrive 的 serial_number，但这里直接取自 MCU UID，不依赖板载 EEPROM） |
| `system.prefs_sequence` | 69 | uint32 | r |  | — | — | 否 | 当前配置块的序号 |
| `system.config_schema` | 117 | uint16 | r |  | — | — | 否 | 配置记录的格式版本（PREFS_SCHEMA），用于升级迁移 |
| `system.prefs_loaded` | 70 | bool | r |  | — | — | 否 | 是否从 flash 成功加载过配置 |
| `system.inertia_status` | 153 | uint8 | r |  | — | — | 否 | 惯量标定状态：0 未跑 / 1 运行中（约 3 s：连测 3 次取中位 + 进出电机模式）/ 2 成功（结果在 config.inertia）/ 3 关节本来就在动 / 4 突破电流超时（卡住或摩擦过大）/ 5 台阶太短（样本不足）或超时 / 6 激励不足（斜率不可信）/ 7 进不了电机模式 |
| `system.save_status` | 119 | uint8 | r |  | — | — | 否 | 最近一次写 flash 的结果（save/zero/reset_config/标定共用）：0 未执行、1 成功、2 因门极使能跳过、3 写失败 |
| `system.gear_status` | 205 | int8 | r |  | — | — | 否 | 最近一次齿轮式双编动作的结果：0 未执行、1 成功、-1 齿数/方向配置非法、-2 还没有参考（先 action.gear_capture）、-3 副编码器没响应、-4 门极使能时被拒绝 |
| `system.can_prot_active` | 123 | int32 | r |  | — | — | 否 | 当前生效的 CAN 协议（0=旧 11-bit，1=CYBERBEAST） |
| `system.can_fd` | 124 | bool | r |  | — | — | 否 | 当前是否以 CAN FD 运行 |
| `system.can_baud_active` | 125 | int32 | r | bit/s | — | — | 否 | 当前生效的 CAN 速率（Classic 为位速率；FD 为数据域速率） |
| `system.cogging_status` | 228 | int8 | r |  | — | — | 否 | 齿槽补偿标定状态：0 未运行 / 1 运行中 / 2 成功（表已写 RAM，用 action.save 落盘）/ -1 中止（离开电机模式或 CAN 超时）/ -2 太多点没收敛 / -3 正反两遍差异过大 / -4 超时 / -5 外环增益未配置 |
| `system.cogging_progress` | 229 | int32 | r |  | — | — | 否 | 本次标定已完成的表点数（正转 + 反转，共 2×1024 点） |
| `system.cogging_progress_total` | 230 | int32 | r |  | — | — | 否 | 一次完整标定的总点数（2 × 1024） |
| `system.cogging_amp` | 231 | float | r | A | — | — | 否 | 最近一次标定得到的齿槽力矩 RMS 幅值（折算成 i_q 电流）。太小（< 0.01 A）说明电机本身齿槽很弱 |
| `system.cogging_mismatch` | 232 | float | r | A | — | — | 否 | 正转/反转两遍的 RMS 差异（A）。与 cogging_amp 同量级说明重复性差（齿隙/摩擦大、增益不对或有外力） |
| `system.cogging_bad` | 233 | uint16 | r |  | — | — | 否 | 两遍都没收敛的点数（这些点用相邻有效点插值补上；超过 1/4 判定失败） |

## action：动作（读/写都执行）

| 名称 | ID | 类型 | 访问 | 单位 | 范围 | 默认 | 持久化 | 说明 |
|---|---|---|---|---|---|---|---|---|
| `action.save` | 71 | function | w |  | — | — | 否 | 把当前配置写入 flash（要求门极关闭） |
| `action.zero` | 72 | function | w |  | — | — | 否 | 把当前位置设为零点并保存（仅菜单状态）。会同时平移齿轮式双编的参考，断电再上电位置仍然连续 |
| `action.gear_capture` | 206 | function | w |  | — | — | 否 | 齿轮式双编：把当前位置定为输出轴绝对位置的参考（不旋转电机，只读一次副编码器）。之后要 action.save 才会保留；结果看 system.gear_status |
| `action.gear_read` | 207 | function | w |  | — | — | 否 | 齿轮式双编：读一次副编码器并用已保存的参考解算位置，刷新 telemetry.sec_enc_* / telemetry.gear_abs_turns（诊断用，不写 flash）。结果看 system.gear_status |
| `action.idle_mode` | 73 | function | w |  | — | — | 否 | 空闲/停机：关门极、清锁存故障、回到菜单（退出电机模式、急停用） |
| `action.clear_fault` | 74 | function | w |  | — | — | 否 | 清除锁存故障（等价于 action.idle_mode） |
| `action.motor_mode` | 75 | function | w |  | — | — | 否 | 进入电机模式（仅菜单状态，且 config.pre_calibrated 必须为 true） |
| `action.calibrate` | 76 | function | w |  | — | — | 否 | 进入标定（相序/极对数、编码器零位与 LUT、R/L/Ld-Lq/磁链；仅菜单状态）。成功后自动置 config.pre_calibrated 并保存；失败不写 flash 且保持未标定 |
| `action.reset_config` | 126 | function | w |  | — | — | 否 | 恢复出厂默认值并写入 flash（要求门极关闭；会清掉标定，需重新标定并重启） |
| `action.reboot` | 127 | function | w |  | — | — | 否 | 重启控制器（应答发出后约 0.5 s 复位，配置按已保存的值重新加载） |
| `action.enter_dfu_mode` | 180 | function | w |  | — | — | 否 | 先应答、约 0.5 s 后复位并跳进 ROM DFU 引导程序（之后约 2 s 跳转，USB 会重新枚举）。与串口菜单的 'd' 同义，用于救砖/整片烧写；配置与标定是否保留取决于下载的地址范围（只写 app 区就保留） |
| `action.reset_timing` | 116 | function | w |  | — | — | 否 | 清零 ISR 耗时统计 |
| `action.measure_inertia` | 154 | function | w |  | — | — | 否 | 一条指令完成惯量标定：自动进电机模式 → 打两个电流台阶测输出端惯量 → 回菜单。约 3 s（内部连测 3 次取中位数，单次离散太大）、关节净行程 ~0.5 rad。结果写 config.inertia（**只写 RAM，不自动保存**），进度/结果读 system.inertia_status；测完自动回菜单，可直接 action.save |
| `action.tune_gains` | 161 | function | w |  | — | — | 否 | 用 config.inertia 算外环增益并写进 RAM（不自动保存）：vel_gain = tune_margin·J·omega_c/(KT·GR)；vel_integrator_gain = vel_gain·clamp(omega_c/25, 5, 30)；pos_gain = omega_c/20。写完用 csv/csp 子命令验证，满意再 action.save |
| `action.cogging_calibrate` | 234 | function | w |  | — | — | 否 | 齿槽转矩补偿标定：自动进电机模式 → 转子逐点保持（正转 1024 点 + 反转 1024 点）→ 回菜单。每点几十毫秒，全程约 2~4 分钟。结果写 RAM（表 + config.cogging_valid），**不自动保存**，也不自动打开 config.cogging_en。进度读 system.cogging_progress，结果读 system.cogging_status / cogging_amp。要求外环增益已配置（先 action.tune_gains） |
| `action.cogging_reset` | 235 | function | w |  | — | — | 否 | 丢弃齿槽补偿表：清零并置 config.cogging_valid=false（只改 RAM，action.save 后才会写进 flash）。换编码器/拆装电机后必须重标 |
