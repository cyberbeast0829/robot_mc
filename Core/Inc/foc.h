/*
 * foc.h
 *
 *  Created on: Aug 2, 2020
 *      Author: ben
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#include <stdint.h>
#include "position_sensor.h"
#include "motion_law.h"

/* Latched fault bits.  Any nonzero controller.fault forces the FSM into
 * FAULT_MODE (gates off) until explicitly cleared by exiting to the menu */
#define FAULT_OV		(1<<0)		// bus overvoltage
#define FAULT_OC		(1<<1)		// phase overcurrent
#define FAULT_ESTOP		(1<<2)		// E-stop requested over CAN (CYBERBEAST 0xC0)
#define FAULT_POS		(1<<3)		// position following error (config.pos_error_fault)
#define FAULT_UV		(1<<4)		// bus undervoltage (config.vbus_uv_trip, gate drive only)
#define FAULT_FET_OT	(1<<5)		// MOS/FET over-temperature (config.fet_temp_*)
#define FAULT_MOT_OT	(1<<6)		// motor over-temperature (config.mot_temp_*)

/* Control law selection for MOTOR_MODE (motion.c).  The default is the
 * impedance law that MIT / legacy / torque / current frames have always used;
 * CSP and CSV switch to the cascade below.  The value is RAM-only and is reset
 * to CTRL_IMPEDANCE by zero_commands(), so a CAN timeout or leaving motor mode
 * always ends in zero torque. */
#define CTRL_IMPEDANCE	0			// kp/kd/t_ff as-is (MIT, legacy packed, 0x03, 0x04)
#define CTRL_VELOCITY	1			// CSV: velocity PI -> i_q
#define CTRL_POSITION	2			// CSP: position P -> velocity PI -> i_q

/* 输入整形模式（config.input_mode，与 ODrive 的 InputMode 编号一致）。
 * 它只负责**整形喂给控制环的设定值**，不选择控制环——控制环是 command.mode。
 * 4/7/8 本固件不支持（按 PASSTHROUGH 处理）。 */
#define INPUT_MODE_INACTIVE		0	// 忽略输入，保持上一拍的设定值
#define INPUT_MODE_PASSTHROUGH	1	// 设定值直通（不做斜率限制）
#define INPUT_MODE_VEL_RAMP		2	// 速度设定值按 accel/decel 限斜率（**默认**，与旧行为一致）
#define INPUT_MODE_POS_FILTER	3	// 二阶位置跟踪滤波（config.input_bw），速度作前馈
#define INPUT_MODE_MIX_CHANNELS	4	// 不支持
#define INPUT_MODE_TRAP_TRAJ	5	// 梯形位置规划（accel/decel 限值），速度作前馈
#define INPUT_MODE_TORQUE_RAMP	6	// 阻抗模式的 t_ff 按 config.torque_ramp_rate 斜坡
#define INPUT_MODE_MIRROR		7	// 不支持
#define INPUT_MODE_TUNING		8	// 不支持
#define INPUT_MODE_MIT			9	// 等同 PASSTHROUGH（我们的阻抗律本来就是 MIT 语义）

/* 惯量标定（action.measure_inertia）的阶段。非 0 时 motion_control() 独占控制权，
 * 顺序执行：静置 → 试探突破电流 → 静置 → 电流台阶 LO → 静置 → 台阶 HI → 计算。
 * 全程在控制 ISR 里，不 printf、不写 flash。 */
#define INERTIA_IDLE		0
#define INERTIA_SETTLE		1			// 零电流等关节停稳（开头的检查）
#define INERTIA_BREAKAWAY	2			// 缓慢加电流找突破摩擦的阈值
#define INERTIA_BRAKE		3			// 主动刹车，把关节拉停（不是零电流滑行）
#define INERTIA_TEST_LO		4			// 电流台阶 LO，边跑边拟合
#define INERTIA_TEST_HI		5			// 电流台阶 HI，边跑边拟合
#define INERTIA_DONE		6			// 计算并写回 config.inertia
#define INERTIA_FAIL		7			// 失败（欠激励/关节卡住/超速）


typedef struct{
	int adc_a_raw, adc_b_raw, adc_c_raw, adc_vbus_raw;	// Physical ADC1/2/3 samples, then VBUS; independent of PHASE_ORDER
    float i_a, i_b, i_c;                                    // Phase currents
    uint8_t i_recon;                                        // 本拍被 KCL 重建的相（0/1/2，物理桥臂序），0xFF = 三相都是实测（调试）
    uint32_t i_recon_count;                                 // 上电以来重建过的控制周期数（饱和，不回绕）
    uint32_t i_min_low_ticks;                               // 低边采样窗口阈值（tick）= I_SAMPLE_MIN_LOW_FRAC*AR，初始化时换算一次
    uint8_t pcl_check_needed;                               // 上一拍写下的占空比里有低于阈值的相 → 本拍 analog_sample 要扫低边窗口
    float v_bus, v_bus_filt;                                // DC link voltage
    float theta_mech, theta_elec;                           // Rotor mechanical and electrical angle
    float dtheta_mech, dtheta_elec;                         // Rotor mechanical and electrical angular velocity
    float i_d, i_q, i_q_filt, i_d_filt;                     // D/Q currents
    float v_d, v_q;                                         // D/Q voltages
    float dtc_u, dtc_v, dtc_w;                              // Terminal duty cycles
    float v_u, v_v, v_w;                                    // Terminal voltages
    float i_scale, k_d, k_q, ki_d, ki_q, ki_fw;             // Current loop gains
    float flux_linkage;
    float d_int, q_int;                                     // Current error integrals
	int adc_a_offset, adc_b_offset, adc_c_offset;		// Physical ADC1/2/3 zero offsets; independent of PHASE_ORDER
    float i_d_des, i_q_des, t_ff_filt;                      // Current references
    int loop_count;                                         // Degubbing counter
    int timeout;                                            // Watchdog counter
    union{
    	float commands[5];									// Making this easier to pass around without including foc.h everywhere
    	struct{
    		float p_des, v_des, kp, kd, t_ff;                   // Desired position, velocity, gains, torque
    	};
    };
    float v_max, v_ref, fw_int, v_margin;                             // output voltage magnitude, field-weakening integral
    int otw_flag;                                           // Over-temp warning
    float i_max, i_mag_max;											// Maximum current
    uint32_t fault;                                         // Latched fault bits (FAULT_x)
    float i_oc_limit;                                       // Overcurrent fault threshold
    uint8_t gate_drive_enabled;                             // Set alongside the ATIM1 gate-drive enable/disable.
                                                            // Overcurrent is only meaningful while driving
    float fault_vbus, fault_iabs;                           // Values captured when a fault first latched

    /* ---- 保护：温度 / 母线电流估算（见 foc.c 的 update_thermistors / commutate）----
     * mos_temp / mot_temp 由主循环按 NTC 换算后写入（不在 40 kHz 中断里算），
     * check_faults() 与 commutate() 只读。i_bus 是估算值，不是测量值。 */
    float mos_temp, mot_temp;                               // NTC 温度（°C）
    float i_bus, i_bus_filt;                                // 估算母线电流（A，滤波前后）
    float p_elec;                                           // 估算电功率（W，见 commutate）
    float mech_power;                                       // 机械功率（W）：输出端转矩 × 输出轴角速度
    float temp_i_lim;                                        // 温度降流 + 转矩上限后的电流上限（A，debug）
    uint16_t uv_cycles;                                     // 欠压去抖计数（控制周期）

    /* ---- 速度环 / 位置环（motion.c；CTRL_IMPEDANCE 下不用这些字段）----
     * 全部是输出轴量（除 i_q_ref / i_cmd_max / vel_integ 是电机端安培）。
     * 分频运行：速度环每 vel_prescale 个控制周期算一次，位置环再每
     * pos_prescale 个速度周期算一次，期间 i_q_ref 保持不变。 */
    uint8_t ctrl_mode;                                      // CTRL_x, set by the control frames
    uint8_t ctrl_mode_prev;                                 // previous cycle's mode, for state seeding
    uint8_t vel_update, pos_update;                         // this cycle actually ran the loop (debug)
    uint16_t vel_prescale_cnt, pos_prescale_cnt;
    float i_q_ref;                                          // outer-loop output (A), held between updates
    float i_cmd_max;                                        // per-frame current limit (A, motor side)
    float v_limit;                                          // per-frame velocity limit (output rad/s)
    float v_target;                                         // outer-loop velocity command (output rad/s)
    float v_setpoint;                                       // after the accel slew limiter (output rad/s)
    float vel_integ;                                        // velocity loop integrator (A)
    float pos_err, vel_err;                                 // last loop errors (telemetry)
    float pos_ff;                                           // CSP frame-to-frame velocity feedforward
    float pos_ff_last;                                      // previous CSP position command (output rad)
    int32_t pos_ff_cycle;                                   // loop_count of the previous CSP frame
    float pos_fault_t;                                      // following-error fault debounce timer (s)
    uint32_t dither_phase;                                  // 低速 dither 的相位累加器（config.dither_*）

    /* ---- 位置软限位的“界外冻结”（见 motion.c 的 motion_seed / CTRL_POSITION 分支）----
     * 切到 CSP 时如果实测位置已经在 [P_MIN,P_MAX] 外，不去追被钳到边界的 p_des
     * （那等于强制把关节拉回限位），而是零速保持原地；主站给出新目标
     * （p_des 变化）或关节自己回到界内后恢复跟踪。 */
    uint8_t pos_hold;                                       // 1 = 界外冻结中
    float pos_hold_des;                                     // 冻结时的 p_des 快照（判“新目标”）

    /* ---- 输入整形（config.input_mode；见 motion_law.h / motion.c）----
     * POS_FILTER / TRAP_TRAJ 产生 (位置, 速度) 设定给位置环跟踪；
     * TORQUE_RAMP 在阻抗模式下给 t_ff 限斜率。切模式时由 motion_seed() 对齐。 */
    uint8_t input_mode_prev;                                // 上一拍的 INPUT_MODE_x（检测模式切换）
    ml_trap_t pos_trap;                                     // TRAP_TRAJ 规划器
    ml_posf_t pos_filter;                                   // POS_FILTER 滤波状态
    float t_ff_slew;                                        // TORQUE_RAMP 斜坡后的 t_ff（N·m）

    /* ---- 惯量标定（action.measure_inertia；inertia_state == INERTIA_IDLE 时全 0）----
     * 测出来的是**输出端**惯量 J（kg·m^2），写 config.inertia，但**不写 flash**。 */
    uint8_t inertia_state;                                  // INERTIA_x
    uint8_t inertia_next;                                   // BRAKE 结束后去的状态
    uint8_t inertia_status;                                 // 对外：0 未跑 / 1 运行中 / 2 成功 / 3..7 失败原因（只由 run_fsm_slow 写）
    int8_t inertia_run_code;                                // 单轮结果码（ISR 写）；多轮取中位时中间轮不对外公布
    volatile uint8_t inertia_req;                           // action 置位，ISR 采纳后清零
    uint16_t inertia_cycles;                                // 当前阶段计时（控制周期）
    float inertia_i;                                        // 当前施加的电流（A，电机端）
    float inertia_i_break;                                  // 突破摩擦的电流估计
    float inertia_i_lo, inertia_i_hi;                       // 两个测试电流
    float inertia_acc_lo, inertia_acc_hi;                   // 两个台阶的拟合加速度
    float inertia_i_sum_lo, inertia_i_sum_hi;               // 台阶内实测电流之和（求均值）
    uint16_t inertia_n_lo, inertia_n_hi;
    float inertia_meas;                                     // 本次测得的 J
    float inertia_pos_start;                                // 台阶开始时的位置（输出轴 rad）
    float inertia_v_peak;                                   // 静置窗内的最大速度（判断"是否静止"）
    ml_fit_t inertia_fit_lo, inertia_fit_hi;                // 流式最小二乘
    float pos_fault_suggest;                                // 位置跟随误差故障的推荐阈值（telemetry）

    /* ---- 齿槽转矩补偿（anti-cogging，见 cogging.h）----
     * 标定请求由 action.cogging_calibrate 置位，控制中断采纳后清零；
     * 标定期间 motion_control() 被它接管（同惯量标定）。cog_ff 是本拍实际
     * 叠加到 i_q_des 上的前馈电流（遥测用）。 */
    volatile uint8_t cogging_req;                           // 标定请求（ISR 采纳后清零）
    float cog_ff;                                           // 本拍齿槽补偿电流（A，0 = 未生效）
    } ControllerStruct;

typedef struct{
    double temperature;                                     // Estimated temperature
    float temp_measured;									// "Measured" temperature computed from resistance
    float qd_in, qd_out;									// Thermal power in and out
    float resistance;										// Motor resistance
    float k;												// Temperature observer gain
    float trust;											// Temperature observer "trust' (kind of like 1/covariance)
    float delta_t;											// Temperature rise
    }   ObserverStruct;

void set_dtc(ControllerStruct *controller);
void analog_sample(ControllerStruct *controller);
void abc(float theta, float d, float q, float *a, float *b, float *c);
void dq0(float theta, float a, float b, float c, float *d, float *q);
void svm(float v_max, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w);
void zero_current(ControllerStruct *controller);
void reset_foc(ControllerStruct *controller);
void reset_observer(ObserverStruct *observer);
void init_controller_params(ControllerStruct *controller);
void current_loop_retune(ControllerStruct *controller);
void commutate(ControllerStruct *controller, EncoderStruct *encoder);
void commutate_open_loop(ControllerStruct *controller, EncoderStruct *encoder);
void torque_control(ControllerStruct *controller);
/* Control-law dispatcher for MOTOR_MODE (motion.c): picks impedance / velocity /
 * position from controller.ctrl_mode and runs the prescaled outer loops. */
void motion_control(ControllerStruct *controller, EncoderStruct *encoder);
/* 低速 dither（摩擦 stick-slip）：motion_control() 之后、cogging_apply() 之前调用 */
void motion_dither(ControllerStruct *controller);
/* action.tune_gains: 由 config.inertia 算 vel_gain / vel_integrator_gain / pos_gain
 * 并写进 RAM（不落盘）。受 config.omega_c 与 config.tune_margin 控制。 */
void motion_tune_gains(ControllerStruct *controller);
void check_faults(ControllerStruct *controller);
/* NTC → °C 换算（照搬 ODrive Board/v3 的 Horner 多项式）+ 一阶滤波。
 * 只在主循环调用：dt 由 controller.loop_count 的差算出，与主循环的实际节奏无关。 */
void update_thermistors(ControllerStruct *controller);
void update_observer(ControllerStruct *controller, ObserverStruct *observer);
void field_weaken(ControllerStruct *controller);
void zero_commands(ControllerStruct * controller);

/* 位置指令写入口的允许区间（输出轴 rad），实现见 foc.c：
 *   界内 -> [P_MIN, P_MAX]；界外 -> 放宽到当前实测位置（可以往界内给任意目标，
 *   哪怕目标仍在界外，但不能比现在更外）。P_MAX <= P_MIN 时返回 ±FLT_MAX。
 * 通信层写 controller.p_des 时用它们；控制层每拍用 motion_law.h 的
 * ml_pos_cmd_clamp() 重算。 */
float pos_cmd_lo(void);
float pos_cmd_hi(void);
float pos_cmd_limit(float p);


#endif /* INC_FOC_H_ */
