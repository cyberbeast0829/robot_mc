/*
 * calibration.c
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */


#include "calibration.h"
#include "hw_config.h"
#include "user_config.h"
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "math_ops.h"

/* Queue a calibration milestone message for the main loop.
 * Called from the commutation interrupt - must not block, so it only stores a
 * fixed-size record; run_fsm_slow() prints it with cal_print_flush() */
static void cal_msg_push(CalStruct *cal, uint8_t id, int i0, int i1, float f0, float f1){
	uint8_t next = (uint8_t)((cal->msg_head + 1) % CAL_MSG_QUEUE_LEN);
	if(next == cal->msg_tail){return;}		// queue full: drop (main loop flushes every iteration, so this cannot happen in practice)
	CalMsg *m = &cal->msg_q[cal->msg_head];
	m->id = id;
	m->i0 = i0;
	m->i1 = i1;
	m->f0 = f0;
	m->f1 = f1;
	/* Publish the entry only after it is fully written (compiler barrier - the
	 * entry fields are plain memory, msg_head is the volatile flag) */
	__asm volatile ("" ::: "memory");
	cal->msg_head = next;
}

/* Print queued calibration messages.  Main loop only (myprintf can block) */
void cal_print_flush(CalStruct *cal){
	while(cal->msg_tail != cal->msg_head){
		CalMsg *m = &cal->msg_q[cal->msg_tail];
		switch(m->id){
			case CAL_MSG_CHECK_PHASE:	myprintf("Checking phase sign, pole pairs\r\n"); break;
			case CAL_MSG_PHASE_ORDER:	myprintf(m->i0 ? "Swapping phase sign\r\n" : "Phase order correct\r\n"); break;
			case CAL_MSG_PPAIRS:		myprintf("Pole Pairs: %d\r\n", m->i0); break;
			case CAL_MSG_START_END:		myprintf("Start: %.3f   End: %.3f\r\n", m->f0, m->f1); break;
			case CAL_MSG_FAILED:		myprintf("Calibration failed: measured %d pole pairs (max %d).  Check the encoder and I_CAL\r\n", m->i0, m->i1); break;
			case CAL_MSG_START_LIN:		myprintf("Starting offset cal and linearization\r\n"); break;
			default:					break;
		}
		cal->msg_tail = (uint8_t)((cal->msg_tail + 1) % CAL_MSG_QUEUE_LEN);
	}
}

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	/* Checks phase order, to ensure that positive Q current produces
	   torque in the positive direction wrt the position sensor */
	PHASE_ORDER = 0;

	if(!cal->started){
		cal_msg_push(cal, CAL_MSG_CHECK_PHASE, 0, 0, 0.0f, 0.0f);
		cal->started = 1;
		cal->start_count = loop_count;
	}
	cal->time = (float)(loop_count - cal->start_count)*DT;

    if(cal->time < T1){
			// Set voltage angle to zero, wait for rotor position to settle
			cal->theta_ref = 0;//W_CAL*cal->time;
			cal->cal_position.elec_angle = cal->theta_ref;
			cal->cal_position.elec_velocity = 0;
			controller->i_d_des = I_CAL;
			controller->i_q_des = 0.0f;
			commutate_open_loop(controller, &cal->cal_position);
    	cal->theta_start = encoder->angle_multiturn;
    	return;
    }

    else if(cal->time < T1+2.0f*PI_F/W_CAL){
    	// rotate voltage vector through one electrical cycle
    	cal->theta_ref = W_CAL*(cal->time-T1);
    	cal->cal_position.elec_angle = cal->theta_ref;
			commutate_open_loop(controller, &cal->cal_position);
    	return;
    }

	reset_foc(controller);

	float theta_end = encoder->angle_multiturn;
	float theta_range = fabsf(theta_end - cal->theta_start);
	int ppairs = theta_range > 0.001f ? (int)roundf(2.0f*PI_F/theta_range) : 0;
	if((ppairs < 1) || (ppairs > PPAIRS_MAX)){
		/* Rotor barely moved, or moved a non-physical amount - stalled rotor,
		 * a bad encoder, or I_CAL too low.  Bail out before the pole-pair count
		 * can run calibration sampling off the end of its arrays */
		cal_msg_push(cal, CAL_MSG_FAILED, ppairs, PPAIRS_MAX, 0.0f, 0.0f);
		cal->failed = 1;
		return;
	}
	cal->ppairs = ppairs;

	if(cal->theta_start < theta_end){
		cal->phase_order = 0;
	}
	else{
		cal->phase_order = 1;
	}
    cal_msg_push(cal, CAL_MSG_PHASE_ORDER, cal->phase_order, 0, 0.0f, 0.0f);
    cal_msg_push(cal, CAL_MSG_PPAIRS, cal->ppairs, 0, 0.0f, 0.0f);
    cal_msg_push(cal, CAL_MSG_START_END, 0, 0, cal->theta_start, theta_end);
    PHASE_ORDER = cal->phase_order;
    PPAIRS = (float)cal->ppairs;
    cal->started = 0;
    cal->done_ordering = 1;	// Finished checking phase order
}

void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	/* Calibrates e-zero and encoder nonliearity */

	if(!cal->started){
			cal_msg_push(cal, CAL_MSG_START_LIN, 0, 0, 0.0f, 0.0f);
			cal->started = 1;
			cal->start_count = loop_count;
			cal->next_sample_time = T1;
			cal->sample_count = 0;
		}

	cal->time = (float)(loop_count - cal->start_count)*DT;

    if(cal->time < T1){
        // Set voltage angle to zero, wait for rotor position to settle
        cal->theta_ref = 0;//W_CAL*cal->time;
        cal->cal_position.elec_angle = cal->theta_ref;
        controller->i_d_des = I_CAL;
        controller->i_q_des = 0.0f;
        commutate_open_loop(controller, &cal->cal_position);

    	cal->theta_start = encoder->angle_multiturn;
    	cal->next_sample_time = cal->time;
    	return;

	}
		// 正转一个机械角度
    else if (cal->time < T1+2.0f*PI_F*PPAIRS/W_CAL){
    	// rotate voltage vector through one mechanical rotation in the positive direction
		cal->theta_ref += W_CAL*DT;//(cal->time-T1);
		cal->cal_position.elec_angle = cal->theta_ref;
		commutate_open_loop(controller, &cal->cal_position);

		// sample SAMPLES_PER_PPAIR times per pole-pair
		if(cal->time > cal->next_sample_time){
			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
			int error = encoder->raw - count_ref;//- encoder->raw;
			cal->error_arr[cal->sample_count] = error + ENC_CPR*(error<0);	// samples are streamed out by the main loop, not printed here
			cal->next_sample_time += 2.0f*PI_F/(W_CAL*SAMPLES_PER_PPAIR);
			if((cal->sample_count >= PPAIRS*SAMPLES_PER_PPAIR-1) || (cal->sample_count >= N_CAL-1)){	// second check guards the array bound in case PPAIRS is bad
				return;
			}
			cal->sample_count++;

		}
		return;
    }
	// 反转一个机械角度
	else if (cal->time < T1+4.0f*PI_F*PPAIRS/W_CAL){
		// rotate voltage vector through one mechanical rotation in the negative direction
		cal->theta_ref -= W_CAL*DT;//(cal->time-T1);
		controller->i_d_des = I_CAL;
		controller->i_q_des = 0.0f;
		cal->cal_position.elec_angle = cal->theta_ref;
		commutate_open_loop(controller, &cal->cal_position);

		// sample SAMPLES_PER_PPAIR times per pole-pair
		if((cal->time > cal->next_sample_time)&&(cal->sample_count>0)){
			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
			int error = encoder->raw - count_ref;// - encoder->raw;
			error = error + ENC_CPR*(error<0);

			cal->error_arr[cal->sample_count] = (cal->error_arr[cal->sample_count] + error)/2;
			cal->sample_count--;
			cal->next_sample_time += 2.0f*PI_F/(W_CAL*SAMPLES_PER_PPAIR);
		}
		return;
    }

    reset_foc(controller);

    // Calculate average offset
    int ezero_mean = 0;
	for(int i = 0; i<((int)PPAIRS*SAMPLES_PER_PPAIR); i++){
		ezero_mean += cal->error_arr[i];
	}
	cal->ezero = ezero_mean/(SAMPLES_PER_PPAIR*PPAIRS);

	// Moving average to filter out cogging ripple

	int window = SAMPLES_PER_PPAIR;
	int lut_offset = (ENC_CPR-cal->error_arr[0])*N_LUT/ENC_CPR;
	for(int i = 0; i<N_LUT; i++){
			int moving_avg = 0;
			for(int j = (-window)/2; j<(window)/2; j++){
				int index = i*PPAIRS*SAMPLES_PER_PPAIR/N_LUT + j;
				if(index<0){index += (SAMPLES_PER_PPAIR*PPAIRS);}
				else if(index>(SAMPLES_PER_PPAIR*PPAIRS-1)){index -= (SAMPLES_PER_PPAIR*PPAIRS);}
				moving_avg += cal->error_arr[index];
			}
			moving_avg = moving_avg/window;
			int lut_index = lut_offset + i;
			if(lut_index>(N_LUT-1)){lut_index -= N_LUT;}
			cal->lut_arr[lut_index] = moving_avg - cal->ezero;	// the finished table is printed by the main loop
		}

	cal->started = 0;
	cal->done_cal = 1;
}

void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){

}
