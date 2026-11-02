/// Values stored in flash, which are modified by user actions ///

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

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




extern float __float_reg[];
extern int __int_reg[];

#ifdef __cplusplus
}
#endif

#endif
