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

typedef struct{
	uint8_t id;
	int i0, i1;
	float f0, f1;
} CalMsg;

typedef struct{
	uint8_t ppairs;									// number of pole pairs measured
	int offset;								// electrical zero position in counts
	float theta_ref;								// reference angle used for calibration
	int start_count;								// loop count at cal start
	uint8_t started;								// has cal started or not?
	float time;										// cal time
	float theta_start;								// cal start angle
	int ezero;
	uint8_t phase_order;							// phase order correct (0) or swapped (1)
	uint8_t done_ordering, done_cal, done_rl;		// flags for different cals finished
	uint8_t failed;									// set when calibration fails; aborts back to the menu
	uint16_t sample_count;							// keep track of how many samples taken
	float next_sample_time;							// time to take next sample
	int error_arr[N_CAL];
	int lut_arr[N_LUT];
	EncoderStruct cal_position;						// Position reference used for calibration
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
