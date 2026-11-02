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

/* Control law selection for MOTOR_MODE (motion.c).  The default is the
 * impedance law that MIT / legacy / torque / current frames have always used;
 * CSP and CSV switch to the cascade below.  The value is RAM-only and is reset
 * to CTRL_IMPEDANCE by zero_commands(), so a CAN timeout or leaving motor mode
 * always ends in zero torque. */
#define CTRL_IMPEDANCE	0			// kp/kd/t_ff as-is (MIT, legacy packed, 0x03, 0x04)
#define CTRL_VELOCITY	1			// CSV: velocity PI -> i_q
#define CTRL_POSITION	2			// CSP: position P -> velocity PI -> i_q

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

    /* ---- 惯量标定（action.measure_inertia；inertia_state == INERTIA_IDLE 时全 0）----
     * 测出来的是**输出端**惯量 J（kg·m^2），写 config.inertia，但**不写 flash**。 */
    uint8_t inertia_state;                                  // INERTIA_x
    uint8_t inertia_next;                                   // BRAKE 结束后去的状态
    uint8_t inertia_status;                                 // 0 未跑 / 1 运行中 / 2 成功 / 3..6 失败原因
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
void commutate(ControllerStruct *controller, EncoderStruct *encoder);
void commutate_open_loop(ControllerStruct *controller, EncoderStruct *encoder);
void torque_control(ControllerStruct *controller);
/* Control-law dispatcher for MOTOR_MODE (motion.c): picks impedance / velocity /
 * position from controller.ctrl_mode and runs the prescaled outer loops. */
void motion_control(ControllerStruct *controller, EncoderStruct *encoder);
/* action.tune_gains: 由 config.inertia 算 vel_gain / vel_integrator_gain / pos_gain
 * 并写进 RAM（不落盘）。受 config.omega_c 与 config.tune_margin 控制。 */
void motion_tune_gains(ControllerStruct *controller);
void check_faults(ControllerStruct *controller);
void update_observer(ControllerStruct *controller, ObserverStruct *observer);
void field_weaken(ControllerStruct *controller);
void zero_commands(ControllerStruct * controller);


#endif /* INC_FOC_H_ */
