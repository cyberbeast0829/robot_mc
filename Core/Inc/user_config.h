/// Values stored in flash, which are modified by user actions ///

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define I_BW                    __float_reg[2]                                  // Current loop bandwidth
#define I_MAX                   __float_reg[3]                                  // Current limit
#define THETA_MIN               __float_reg[4]                                  // Minimum position setpoint
#define THETA_MAX               __float_reg[5]                                  // Maximum position setpoint
#define I_FW_MAX                __float_reg[6]                                  // Maximum field weakening current
#define R_NOMINAL               __float_reg[7]                                  // Nominal motor resistance, set during calibration
#define TEMP_MAX                __float_reg[8]                                  // Temperature safety lmit
#define I_MAX_CONT              __float_reg[9]                                  // Continuous max current
#define PPAIRS					__float_reg[10]									// Number of motor pole-pairs
#define L_D						__float_reg[11]									// D-axis inductance (measured during calibration)
#define L_Q						__float_reg[12]									// Q-axis inductance (measured during calibration)
#define R_PHASE					__float_reg[13]									// Single phase resistance
#define KT						__float_reg[14]									// Torque Constant (N-m/A)
#define R_TH					__float_reg[15]									// Thermal resistance (C/W)
#define C_TH					__float_reg[16]									// Thermal mass (C/J)
#define GR						__float_reg[17]									// Gear ratio
#define I_CAL					__float_reg[18]									// Calibration Current
#define P_MIN					__float_reg[19]									// Position setpoint lower limit (rad)
#define P_MAX					__float_reg[20]									// Position setupoint upper bound (rad)
/* P_MAX/P_MIN 的软限位规则见 foc.c 的 pos_cmd_lo/hi/limit()（通信层写 p_des）
 * 与 motion_law.h 的 ml_pos_cmd_range()（控制层与 host 测试） */
#define V_MIN					__float_reg[21]									// Velocity setpoint lower bound (rad/s)
#define V_MAX					__float_reg[22]									// Velocity setpoint upper bound (rad/s)
#define KP_MAX					__float_reg[23]									// Max position gain (N-m/rad)
#define KD_MAX					__float_reg[24]									// Max velocity gain (N-m/rad/s)
/* 速度环 / 位置环（CSP、CSV）。单位都是**输出轴**量，输出是电机端安培：
 * 帧里的电流限幅本身就是 A，电流环/弱磁/过流阈值也都在安培域，所以外环直接
 * 算电流，省掉 KT/GR 的来回换算。 */
#define POS_GAIN				__float_reg[25]									// 位置环 P：(rad/s)/rad
#define VEL_GAIN				__float_reg[26]									// 速度环 P：A/(rad/s)
#define VEL_INTEGRATOR_GAIN		__float_reg[27]									// 速度环 I：A/(rad/s)/s
#define POS_FF_GAIN				__float_reg[28]									// CSP 帧间速度前馈系数（0 = 关闭）
#define ACCEL_LIMIT				__float_reg[29]									// 速度设定值斜率限制 rad/s^2（0 = 关闭）
#define POS_ERROR_FAULT			__float_reg[30]									// 位置跟随误差故障阈值 rad（0 = 关闭）
#define PLL_BW					__float_reg[31]									// 编码器速度 PLL 带宽 rad/s
#define INERTIA					__float_reg[32]									// 输出端惯量 kg·m^2（action.measure_inertia 测得，不自动保存）
#define OMEGA_C					__float_reg[33]									// action.tune_gains 的目标穿越频率 rad/s
#define TUNE_MARGIN				__float_reg[34]									// action.tune_gains 的速度环放大系数（补摩擦/齿隙造成的低估）
/* ---- 电流环：死区补偿（算法见 Core/Inc/dtc_comp.h）----
 * DT_COMP 是每桥臂注入的**占空比当量** Ve/v_bus，不是伏特（见 dtc_comp.h 的量纲段）。
 * 硬件死区 = tim.c 的 DeadTime = 20 tick、一个周期 2*AR = 6000 tick ⇒ 0.00333；出厂
 * 默认 0.008 是本板实测的"死区 + 器件压降"合计当量（0.00333 只补死区，还剩 59% 的
 * 截距），换功率板要按 docs/DEV.md 的 d 轴静态 V/I 拟合重标。0 = 关闭。 */
#define DT_COMP					__float_reg[55]									// 死区补偿强度（占空比当量，0 = 关闭）
#define DT_COMP_TOL				__float_reg[56]									// 死区补偿的符号平滑宽度 A（必须大于采样时刻的电流纹波幅值）
/* ---- 保护与限幅（P0）----
 * 温度上下限是 °C；enabled = 0 时温度只上报、不参与控制（默认就是关，
 * 因为 NTC 型号/分压需要台架核对后才可信）。 */
#define VBUS_OV_TRIP			__float_reg[35]									// 母线过压硬故障阈值 V
#define VBUS_UV_TRIP			__float_reg[36]									// 母线欠压故障阈值 V（0 = 关闭）
#define I_DC_MAX				__float_reg[37]									// 母线电流软限幅 A（0 = 关闭）
#define TORQUE_LIM				__float_reg[38]									// 输出端转矩上限 N·m（0 = 关闭）
#define FET_TEMP_LO				__float_reg[39]									// MOS 温度降流起点 °C
#define FET_TEMP_HI				__float_reg[40]									// MOS 温度上限 °C
#define MOT_TEMP_LO				__float_reg[41]									// 电机温度降流起点 °C
#define MOT_TEMP_HI				__float_reg[42]									// 电机温度上限 °C
/* ---- 输入整形（P1）---- */
#define INPUT_BW				__float_reg[43]									// POS_FILTER 的跟踪带宽 rad/s
#define TORQUE_RAMP_RATE		__float_reg[44]									// TORQUE_RAMP 的转矩斜率 N·m/s
#define DECEL_LIMIT				__float_reg[45]									// 速度设定值减速度 rad/s²（与 ACCEL_LIMIT 配对）
/* 输出端力矩满量程（N·m）：MIT(0x00/0x80) 12 位力矩位域的定标，也是所有输出端
 * 力矩命令（command.t_ff / legacy SET_TFF / TORQUE / CURRENT）的钳位上限。
 * 0 = 沿用旧行为 I_MAX*KT*GR（随 i_max 变化的物理可达值）。
 * ⚠ 这是**线上量程**：上位机必须用同一个值编解码（读 config.mit_tau_max），
 * 改它等于改协议。它只决定分辨率/可表达范围，实际能输出的力矩仍受
 * I_MAX*KT*GR（以及温度降流 / config.torque_lim）限制，超出部分只会让电流环饱和。
 * 旧值 I_MAX*KT*GR 在 i_max=4 A / KT=0.0334 / GR=9 时只有 1.2 N·m，12 位分辨率
 * 只有 0.6 mN·m 却表达不了 3.5 N·m 级的峰值力矩，所以独立出来。 */
/* ⚠ 索引必须唯一：曾经误用 __float_reg[46]（与 GEAR_ZERO_PHASE 撞车），
 * config_apply_limits() 的 gear_zero_phase ≤ 1.0 钳位会把它压成 1.0 N·m。 */
#define MIT_TAU_MAX				__float_reg[54]									// 力矩满量程 N·m（输出端，0 = I_MAX*KT*GR）
#define TORQUE_FS				((MIT_TAU_MAX > 0.0f) ? MIT_TAU_MAX : (I_MAX * KT * GR))
/* ---- 斜坡/轨迹的专用限值（0 = 沿用上面那组通用值，保持老配置行为不变）----
 * 为什么要分开：轨迹模式想慢慢爬、同时保留手动高速点动时，共用一个 v_max /
 * accel_limit 就做不到。默认全 0，等价于"没有这两组参数"。 */
#define VEL_RAMP_RATE			__float_reg[47]									// VEL_RAMP 的对称斜率 rad/s²（>0 时加速与减速都用它）
#define TRAP_VEL_LIMIT			__float_reg[48]									// TRAP_TRAJ 的巡航上限 rad/s（与运行期限速取较小值）
#define TRAP_ACCEL_LIMIT		__float_reg[49]									// TRAP_TRAJ 的加速度 rad/s²
#define TRAP_DECEL_LIMIT		__float_reg[50]									// TRAP_TRAJ 的减速度 rad/s²
/* ---- 齿槽转矩补偿（anti-cogging，见 cogging.h）----
 * 表本身是 internal.cogging_lut（hidden blob，1024 × int16，一个转子机械周期）。
 * 标定用外环自己的增益（pos_gain / vel_gain / vel_integrator_gain），所以
 * **先 action.tune_gains 再标定**。 */
#define COG_GAIN				__float_reg[51]									// 补偿缩放（0..2，1 = 标定值原样）
#define COG_MAX					__float_reg[52]									// 前馈电流钳位 A（0 = 不使用补偿）
#define COG_POS_THR				__float_reg[53]									// 标定收敛阈值（输出轴 rad），建议 ≤ 半个表距
/* ---- 摩擦前馈（输出轴速度 → 电机端电流）----
 * 速度环的积分器本来就能吃掉库仑摩擦，但它要靠速度误差慢慢积累：低速时这表现
 * 为爬行/stick-slip，而且积分器一旦去补摩擦就没法同时压住扰动。这里按
 * "运动方向 + 速度大小"把摩擦直接前馈掉，积分器只负责剩下的小量。
 * coulomb 取空载突破电流（action.measure_inertia 的 telemetry.inertia_break_current），
 * viscous 取 (i_q(高速) − i_q(近零速)) / 速度差。0 = 关闭（默认，不改变旧行为）。 */
#define FRIC_COULOMB			__float_reg[57]									// 库仑摩擦前馈 A
#define FRIC_VISCOUS			__float_reg[58]									// 粘滞摩擦前馈 A/(rad/s)
#define FRIC_EPS				__float_reg[59]									// 库仑项在 ±eps 内线性过渡 rad/s
/* ---- 测速 ---- */
#define VEL_EST_NOISE			__float_reg[60]									// 最小二乘测速的相对噪声目标（0.004 = 0.4%，遥测/测量）
#define VEL_CTRL_NOISE			__float_reg[61]									// 速度环反馈窗口的相对噪声目标（0.05 = 5%，换更小滞后）
/* ---- 低速抖动（dither）----
 * 0.1~1 RPM 的"蠕动"是摩擦的 Stribeck 负阻尼 + 静摩擦造成的 stick-slip 极限环，
 * 靠提高速度环增益治不了（编码器噪声不允许）。在 q 轴电流上叠一个高频小幅
 * 正弦，让接触面始终处在"动摩擦"状态，负阻尼消失。幅值取库仑摩擦的 10~30%，
 * 频率要远高于速度环带宽（机械上被惯量滤掉：Δv ≈ ΔT/(J·2πf)）。
 * amp = 0 关闭（默认）。只在 0 < |v_setpoint| < v_max 时注入，静止时不抖。 */
#define DITHER_AMP				__float_reg[62]									// dither 幅值 A（0 = 关闭）
#define DITHER_FREQ				__float_reg[63]									// dither 频率 Hz
#define DITHER_V_MAX			__float_reg[64]									// 只在 |v_setpoint| 低于它时注入（输出轴 rad/s）

#define PHASE_ORDER             __int_reg[0]                                    // Phase swapping during calibration
#define CAN_ID                  __int_reg[1]                                    // CAN bus ID
#define CAN_MASTER              __int_reg[2]                                    // CAN bus "master" ID
#define CAN_TIMEOUT             __int_reg[3]                                    // CAN command timeout in **milliseconds** (0 = disabled)
/* The control loop runs at 40 kHz (DT = 25 us in hw_config.h), so 1 ms = 40 ISR
 * ticks. controller.timeout counts ticks; compare against this, never against
 * CAN_TIMEOUT itself. Change both here and in prefs_codec.c's v1 migration if
 * DT ever changes. */
#define CAN_TIMEOUT_CYCLES      (CAN_TIMEOUT * 40)
#define M_ZERO					__int_reg[4]
#define E_ZERO					__int_reg[5]
#define ENCODER_LUT             __int_reg[6]                                    // Encoder offset LUT - 128 elements long (6..133)
#define CAN_PROT                __int_reg[134]                                  // CAN protocol: 0 = legacy 11-bit, 1 = CYBERBEAST. Applied at power-up
#define CAN_BAUD                __int_reg[135]                                  // bit/s. <= 1M: Classic CAN at this rate. > 1M: CAN FD (1M arbitration, data = this)
#define CAN_HB_MS               __int_reg[136]                                  // CYBERBEAST heartbeat period (ms), 0 = disabled
/* 外环分频：速度环每 vel_loop_prescaler 个控制周期算一次，位置环再每
 * pos_loop_prescaler 个速度周期算一次（嵌套，见 motion.c）。默认 40 kHz / 5 = 8 kHz，
 * 再 / 2 = 4 kHz。设成 1/1 就是两个环都跑 40 kHz。 */
#define VEL_LOOP_PRESCALER      __int_reg[137]                                  // 控制环 / 该值 = 速度环频率
#define POS_LOOP_PRESCALER      __int_reg[138]                                  // 速度环 / 该值 = 位置环频率
/* CAN FD 发送延迟补偿（TDC）偏移，单位是**数据域 tq**：
 *   -1 = 自动（= 数据域 tseg1，与 ODrive / canable2-fw 一致，默认）
 *    0 = 关闭（仅用于实验对照）
 *   >0 = 手写偏移（TDCR 的 offset 与 filter 都用该值；上限 31）
 * 5 Mbit/s 下 1 tq 只有 25 ns，收发器环回延迟超过剩余采样裕量，不开 TDC 时
 * 设备会把自己发出的帧判成错误帧 → TEC 涨到 bus-off、总线上什么都看不到。
 * 只在 FD 模式下生效，且**下次上电**才应用（DBTP/TDCR 只能在 init 模式写）。 */
#define CAN_TDC                 __int_reg[139]
#define INPUT_MODE              __int_reg[140]									// 输入整形模式（INPUT_MODE_x，见 foc.h）
/* ---- 齿轮式双编（断电记忆输出轴位置）----
 * 主编码器在转子轴上，副编码器在一个与转子齿轮啮合、齿数不同的齿轮上（SPI 共
 * 用 SPI1，NSS 独立）。两路单圈绝对读数合起来解出"转子第几圈"，上电即恢复输出轴
 * 的绝对位置；原理与量程见 dual_gear.h。全部在门极关闭时改。
 *
 * 量程 = ±T2/(2·|T1 - T2|) 转子圈（34/33 给 ±16.5，10/9 只给 ±4.5），所以齿数要
 * 接近但不等；成比例放大（10/9 = 20/18 = 30/27）行为完全相同（只看 k/T2）。
 * 副编码器读数如果随转子正转变小（磁铁反装 / 外啮合让副轴反转），置
 * `GEAR_SEC_REVERSED`：固件先镜像读数再解算，量程不变 —— 不要试图用"求和"去解释
 * 方向，那样每转一圈相位跑两圈多，圈数就没法解了（原因见 dual_gear.h）。 */
#define GEAR_TEETH_FIRST        __int_reg[141]                                  // T1：转子侧齿轮齿数
#define GEAR_TEETH_SEC          __int_reg[142]                                  // T2：副编码器侧齿轮齿数
#define GEAR_ZERO_COUNT         __int_reg[144]                                  // 标定时刻的多圈计数 turns*CPR + count_wrapped
#define GEAR_ZERO_PHASE         __float_reg[46]                                 // 标定时刻的韦尼尔相位 Δ ∈ [0,1)
/* 齿槽补偿的标定参数（一个转子机械周期的 1024 个点，见 cogging.h）。
 * settle_ms 是"位置误差进阈值后还要连续保持多久才开始采样"，
 * avg 是采样平均的控制周期数（40 kHz，默认 80 = 2 ms）。 */
#define COG_SETTLE_MS           __int_reg[145]                                  // 标定：每点收敛后的保持时间 ms
#define COG_AVG                 __int_reg[146]                                  // 标定：每点平均的控制周期数
#define COG_INDEX               __int_reg[147]                                  // 诊断：读表用的索引（不持久化）
#define COG_DETREND             __int_reg[148]                                  // 标定后去掉一个 N 阶多项式趋势（重力/外载，0 = 不去）
#define COG_METHOD              __int_reg[149]                                  // 标定方法：1 = 匀速扫描（默认），0 = 逐点停住
#define COG_SWEEP_MS            __int_reg[150]                                  // 扫描法：一个转子机械圈走多少毫秒
#define GEAR_SEC_REVERSED       gear_sec_reversed_flag	// 副编码器读数随正转变小（磁铁反装/外啮合）→ 解算前镜像
#define GEAR_REF_VALID          gear_ref_valid_flag		// 双编参考是否已建立（=1 才在上电时恢复位置）
/* 布尔/标志类配置单独用 uint8_t 存：TLV 里 bool 记录只有 1 字节，塞进 __int_reg
 * （int）会只写低字节，虽然在小端上能用，但不如独立字节干净。 */
#define PRE_CALIBRATED          pre_calibrated_flag		// 标定完成标志（0 = 未标定，禁止进电机模式）
#define FET_TEMP_EN             fet_temp_en_flag			// MOS 温度保护开关
#define MOT_TEMP_EN             mot_temp_en_flag			// 电机温度保护开关
#define VEL_LIMIT_EN            vel_limit_en_flag			// 速度设定值限幅开关（config.enable_vel_limit）
#define TORQUE_VEL_LIM_EN       torque_vel_lim_en_flag		// 阻抗/力矩模式下的速度限制开关
#define COGGING_EN              cogging_en_flag				// 齿槽补偿开关（config.cogging_en）
#define COGGING_VALID           cogging_valid_flag			// 齿槽补偿表有效（标定成功才置 1，config.cogging_valid）
#define VEL_SCHED_EN            vel_sched_en_flag			// 速度环增益按测速滞后自动降带宽（config.vel_sched_en）
extern uint8_t pre_calibrated_flag, fet_temp_en_flag, mot_temp_en_flag;
extern uint8_t vel_limit_en_flag, torque_vel_lim_en_flag;
extern uint8_t gear_ref_valid_flag;
extern uint8_t gear_sec_reversed_flag;
extern uint8_t cogging_en_flag, cogging_valid_flag;
extern uint8_t vel_sched_en_flag;


extern float __float_reg[];
extern int __int_reg[];

#ifdef __cplusplus
}
#endif

#endif
