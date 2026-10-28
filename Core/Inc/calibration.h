/*
 * calibration.h
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "position_sensor.h"
#include "foc.h"

#define V_CAL 	1.0f							// Calibration voltage
//#define I_CAL	5.0f
#define W_CAL 	10.0f							// Calibration speed in rad/s
#define T1		1.0f							// Cal settling period
#define PPAIRS_MAX 64
#define SAMPLES_PER_PPAIR N_LUT
#define N_CAL SAMPLES_PER_PPAIR*PPAIRS_MAX		// Calibration lookup table maximum length, so I don't have to deal with dynamic allocation based on number of pole pairs

/* Pole-pair measurement (order_phases) */
#define CAL_ORDER_HOLD_S	0.75f		// [s] hold the final electrical phase until the rotor stops
#define CAL_ORDER_AVG_S		0.1f		// [s] encoder averaging window at both ends

/* ---- R / L / Ld-Lq / flux-linkage measurement ----------------------------
 *
 * Ported from ODrive's Motor::measure_phase_resistance /
 * measure_phase_inductance / measure_ld_lq / measure_flux_linkage (and their
 * control laws).  Runs after the encoder calibration, also inside the 40 kHz
 * control interrupt: everything here is register/I-O work plus a few floats,
 * no printing (milestones go through cal_msg_push()).
 *
 * Test current: I_CAL clamped to I_MAX (the board current limit).
 */
#define CAL_I_MIN			0.2f		// [A] minimum usable test current
#define CAL_V_MAX			2.0f		// [V] inductance probe amplitude (voltage toggled every cycle)
#define CAL_LOCK_V_MAX		12.0f		// [V] DC clamp for the R integrator and the Ld/Lq rotor lock
#define CAL_FLUX_V_MAX		10.0f		// [V] voltage clamp while spinning for the flux measurement
/* CAL_V_MAX is the toggle amplitude, tuned for the motors on this board
 * (~100 uH): lowering it reduces the current ripple (and the risk of tripping
 * the overcurrent latch on a very low-inductance motor) at the cost of
 * measurement resolution.  CAL_LOCK_V_MAX is only a clamp for the DC loops:
 * it must be large enough for i_cal x R (e.g. 4 A x 1 ohm = 4 V, i.e. ~3.2 in
 * the control loop's voltage units) or the R / Ld / Lq stages never settle. */
#define CAL_R_SETTLE_S		1.5f		// [s] settling per R current step (ODrive: 1500 ms)
#define CAL_R_AVG_S			0.1f		// [s] voltage averaging window at the end of each R step
#define CAL_L_MEAS_S		1.25f		// [s] inductance voltage-toggle window (ODrive: 1250 ms)
#define CAL_LD_LOCK_S		0.75f		// [s] rotor lock at alpha / beta (needs a few R/L time constants)
#define CAL_LD_MEAS_S		0.75f		// [s] inductance measurement window per axis
#define CAL_FLUX_LOCK_S		0.5f		// [s] open-loop rotor lock
#define CAL_FLUX_OMEGA		60.0f		// [rad/s electrical] target speed
#define CAL_FLUX_ACCEL		40.0f		// [rad/s^2 electrical] ramp
#define CAL_FLUX_SETTLE_S	0.5f		// [s] settle at speed before sampling
#define CAL_FLUX_MEAS_S		2.0f		// [s] sampling window
#define CAL_R_MIN			1e-4f		// [Ohm] plausible measurement range
#define CAL_R_MAX			100.0f
#define CAL_L_MIN			2e-6f		// [H]
#define CAL_L_MAX			4e-3f
#define CAL_LAMBDA_MIN		1e-4f		// [Wb] plausible PM flux linkage range
#define CAL_LAMBDA_MAX		0.4f
/* Integrator gains.  ODrive uses 1.0 / 2.0, but those take R/kI seconds to
 * settle: fine for a 50 mohm motor, far too slow for the 1 ohm class motors
 * this 4 A board drives (R would read ~17 % low after 1.5 s).  4.0 still has
 * plenty of damping for low-R/high-L motors. */
#define CAL_KI_R			4.0f		// [(V/s)/A] resistance measurement
#define CAL_KI_LOCK			4.0f		// [(V/s)/A] Ld/Lq rotor lock
#define CAL_KI_FLUX			2.0f		// [(V/s)/A] flux measurement current PI
#define CAL_IBETA_FILT		80.0f		// [1/s] Ibeta low-pass for the phase-balance check

/* Measurement sub-stages (CalRlStruct.stage) */
#define RL_IDLE				0
#define RL_R_LOW			1			// R, first step at I_CAL/2
#define RL_R_HIGH			2			// R, second step at I_CAL
#define RL_L				3			// L (voltage toggle)
#define RL_LD_LOCK			4			// lock rotor d-axis on alpha
#define RL_LD_MEAS			5			// Ld measurement
#define RL_LQ_LOCK			6			// lock rotor d-axis on beta
#define RL_LQ_MEAS			7			// Lq measurement
#define RL_FLUX_LOCK		8			// open-loop flux: lock rotor
#define RL_FLUX_RAMP		9			// open-loop flux: ramp up speed
#define RL_FLUX_MEAS		10			// open-loop flux: sample Vq/Id/Iq

/* Calibration milestone messages.
 *
 * Calibration runs inside the 40 kHz commutation interrupt, where printing is
 * forbidden (myprintf() blocks on USB/serial and would stall the control loop
 * for milliseconds).  The ISR queues the message here and the main loop prints
 * it with cal_print_flush() - see run_fsm_slow(). */
#define CAL_MSG_QUEUE_LEN	8
#define CAL_MSG_CHECK_PHASE	0	// "Checking phase sign, pole pairs"
#define CAL_MSG_PHASE_ORDER	1	// i0: 0 = order correct, 1 = swapped
#define CAL_MSG_PPAIRS		2	// i0: measured pole pairs
#define CAL_MSG_START_END	3	// f0/f1: start/end angle
#define CAL_MSG_FAILED		4	// i0: measured ppairs, i1: max
#define CAL_MSG_START_LIN	5	// "Starting offset cal and linearization"
#define CAL_MSG_RL_START	6	// f0: test current [A]
#define CAL_MSG_R_DONE		7	// f0: phase resistance [Ohm]
#define CAL_MSG_L_DONE		8	// f0: phase inductance [H]
#define CAL_MSG_LD_LQ		9	// f0: Ld [H], f1: Lq [H]
#define CAL_MSG_FLUX		10	// f0: flux linkage [Wb], f1: Kt [N-m/A]
#define CAL_MSG_RL_FAIL		11	// i0: stage, f0: measured value, f1: current/aux value
#define CAL_MSG_R_STEP		12	// f0: V1 (command volts), f1: I1 [A]

typedef struct{
	uint8_t id;
	int i0, i1;
	float f0, f1;
} CalMsg;

/* State of the R/L/Ld-Lq/flux measurement chain (see measure_lr()) */
typedef struct{
	uint8_t stage;						// RL_* stage
	int start_count;					// loop_count when the current stage started
	uint32_t tick;						// cycles in the current stage
	float test_current;					// [A] test current (I_CAL clamped to I_MAX)
	float i_low, i_high;				// [A] the two R-measurement current targets
	float test_voltage;					// command volts: integrator state / toggle probe
	float i_beta_filt;					// [A] filtered Ibeta for the phase-balance check
	float v1, v2;						// R measurement steady-state voltages (command volts)
	float i1_meas, i2_meas;				// R measurement steady-state currents [A]
	float v_sum, i_sum;					// averaging accumulators
	int v_count;						// averaging samples
	float last_ialpha;					// previous Ialpha sample
	float delta_i;						// accumulated current ripple [A]
	float lock_v_alpha, lock_v_beta;	// Ld/Lq rotor-lock voltages (command volts)
	float ld, lq;						// measured inductances [H]
	float theta, omega;					// open-loop flux measurement angle/speed
	float vd, vq;						// flux measurement dq voltage (command volts)
	float vq_sum, id_sum, iq_sum;		// flux measurement accumulators
	uint32_t meas_count;				// flux measurement samples
} CalRlStruct;

typedef struct{
	uint8_t ppairs;									// number of pole pairs measured
	int offset;								// electrical zero position in counts
	float theta_ref;								// reference angle used for calibration
	int start_count;								// loop count at cal start
	uint8_t started;								// has cal started or not?
	float time;										// cal time
	float theta_start;								// cal start angle (averaged)
	float theta_end;								// settled angle after the rotation
	float angle_sum;								// encoder averaging accumulator
	int angle_count;								// encoder averaging samples
	uint8_t order_stage;							// order_phases() sub-stage: settle / ramp / hold
	int ezero;
	uint8_t phase_order;							// phase order correct (0) or swapped (1)
	uint8_t done_ordering, done_cal, done_rl;		// flags for different cals finished
	uint8_t failed;									// set when calibration fails; aborts back to the menu
	uint16_t sample_count;							// keep track of how many samples taken
	float next_sample_time;							// time to take next sample
	int error_arr[N_CAL];
	int lut_arr[N_LUT];
	EncoderStruct cal_position;						// Position reference used for calibration
	CalRlStruct rl;									// R/L/Ld-Lq/flux measurement state
	CalMsg msg_q[CAL_MSG_QUEUE_LEN];				// milestone messages queued by the ISR for the main loop
	volatile uint8_t msg_head;						// written in the ISR
	volatile uint8_t msg_tail;						// written by the main loop

} CalStruct;

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal, int loop_count);
void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal,
		int loop_count);
void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal,  int loop_count);
void cal_print_flush(CalStruct *cal);

#endif /* INC_CALIBRATION_H_ */
