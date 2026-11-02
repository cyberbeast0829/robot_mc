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

/* Human-readable name of a measurement sub-stage, for failure reports */
static const char *cal_rl_stage_name(uint8_t stage){
	switch(stage){
		case RL_R_LOW:		return "R (low step)";
		case RL_R_HIGH:		return "R (high step)";
		case RL_L:			return "L";
		case RL_LD_LOCK:	return "Ld lock";
		case RL_LD_MEAS:	return "Ld";
		case RL_LQ_LOCK:	return "Lq lock";
		case RL_LQ_MEAS:	return "Lq";
		case RL_FLUX_LOCK:	return "flux lock";
		case RL_FLUX_RAMP:	return "flux ramp";
		case RL_FLUX_MEAS:	return "flux";
		default:			return "start-up";
	}
}

/* Print queued calibration messages.  Main loop only (myprintf can block) */
void cal_print_flush(CalStruct *cal){
	while(cal->msg_tail != cal->msg_head){
		CalMsg *m = &cal->msg_q[cal->msg_tail];
		switch(m->id){
			case CAL_MSG_CHECK_PHASE:	myprintf("Checking phase sign, pole pairs\r\n"); break;
			case CAL_MSG_PHASE_ORDER:	myprintf(m->i0 ? "Swapping phase sign\r\n" : "Phase order correct\r\n"); break;
			case CAL_MSG_PPAIRS:		myprintf("Pole Pairs: %d   (travel %.4f rad)\r\n", m->i0, m->f0); break;
			case CAL_MSG_START_END:		myprintf("Start: %.3f   End: %.3f\r\n", m->f0, m->f1); break;
			case CAL_MSG_FAILED:		myprintf("Calibration failed: measured %d pole pairs (max %d).  Encoder travel %.4f rad, bus %.1f V.  Check the encoder, I_CAL and the bus supply\r\n", m->i0, m->i1, m->f1, m->f0); break;
			case CAL_MSG_START_LIN:		myprintf("Starting offset cal and linearization\r\n"); break;
			case CAL_MSG_RL_START:		myprintf("Measuring R, L, Ld/Lq and flux linkage (%.2f A)\r\n", m->f0); break;
			case CAL_MSG_R_STEP:		myprintf("R step 1: %.4f V @ %.2f A\r\n", m->f0, m->f1); break;
			case CAL_MSG_R_DONE:		myprintf("Phase resistance: %.5f Ohm\r\n", m->f0); break;
			case CAL_MSG_L_DONE:		myprintf("Phase inductance (rough): %.2f uH\r\n", m->f0*1e6f); break;
			case CAL_MSG_LD_LQ:			myprintf("Ld: %.2f uH   Lq: %.2f uH   (Ld-Lq: %.2f uH)\r\n", m->f0*1e6f, m->f1*1e6f, (m->f0-m->f1)*1e6f); break;
			case CAL_MSG_FLUX:			myprintf("Flux linkage: %.6f Wb   Kt: %.4f N-m/A\r\n", m->f0, m->f1); break;
			case CAL_MSG_RL_FAIL:		myprintf("Measurement failed (%s): %.6f (aux %.4f).  Check I_CAL, bus voltage and motor wiring\r\n", cal_rl_stage_name((uint8_t)m->i0), m->f0, m->f1); break;
			default:					break;
		}
		cal->msg_tail = (uint8_t)((cal->msg_tail + 1) % CAL_MSG_QUEUE_LEN);
	}
}

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	/* Checks phase order, to ensure that positive Q current produces
	   torque in the positive direction wrt the position sensor, and measures
	   the pole pairs: rotate the voltage vector through exactly one electrical
	   cycle and divide by the mechanical angle travelled.

	   The rotor lags the rotating field while it accelerates, and sampling the
	   end angle at the instant the ramp ends made the travel too short: with 14
	   pole pairs the ideal travel is only 0.449 rad, so a lag of just 0.95 deg
	   mechanical already rounds to 15.  Hold the final electrical phase for
	   CAL_ORDER_HOLD_S and average the encoder at both ends - the rotor then
	   stops at the same electrical phase it started from, so the static load
	   angle cancels in theta_end - theta_start. */
	PHASE_ORDER = 0;

	if(!cal->started){
		cal_msg_push(cal, CAL_MSG_CHECK_PHASE, 0, 0, 0.0f, 0.0f);
		cal->started = 1;
		cal->start_count = loop_count;
		cal->order_stage = 0;
		cal->angle_sum = 0.0f;
		cal->angle_count = 0;
	}
	cal->time = (float)(loop_count - cal->start_count)*DT;

	{
		float t_ramp_end = T1 + 2.0f*PI_F/W_CAL;
		float t_hold_end = t_ramp_end + CAL_ORDER_HOLD_S;

		switch(cal->order_stage){
			case 0:
				/* Set voltage angle to zero, wait for rotor position to settle */
				cal->theta_ref = 0.0f;
				cal->cal_position.elec_angle = 0.0f;
				cal->cal_position.elec_velocity = 0.0f;
				controller->i_d_des = I_CAL;
				controller->i_q_des = 0.0f;
				commutate_open_loop(controller, &cal->cal_position);
				if(cal->time >= T1 - CAL_ORDER_AVG_S){
					cal->angle_sum += encoder->angle_multiturn;
					cal->angle_count++;
					cal->theta_start = cal->angle_sum/(float)cal->angle_count;
				}
				if(cal->time >= T1){
					cal->angle_sum = 0.0f;
					cal->angle_count = 0;
					cal->order_stage = 1;
				}
				return;

			case 1:
				/* Rotate the voltage vector through one electrical cycle */
				cal->theta_ref = W_CAL*(cal->time - T1);
				cal->cal_position.elec_angle = cal->theta_ref;
				cal->cal_position.elec_velocity = 0.0f;
				controller->i_d_des = I_CAL;
				controller->i_q_des = 0.0f;
				commutate_open_loop(controller, &cal->cal_position);
				if(cal->time >= t_ramp_end){
					cal->angle_sum = 0.0f;
					cal->angle_count = 0;
					cal->order_stage = 2;
				}
				return;

			case 2:
				/* Hold the final electrical phase (same phase as the start,
				 * one mechanical pole pitch further on) until the rotor stops */
				cal->theta_ref = TWO_PI_F;
				cal->cal_position.elec_angle = TWO_PI_F;
				cal->cal_position.elec_velocity = 0.0f;
				controller->i_d_des = I_CAL;
				controller->i_q_des = 0.0f;
				commutate_open_loop(controller, &cal->cal_position);
				if(cal->time >= t_hold_end - CAL_ORDER_AVG_S){
					cal->angle_sum += encoder->angle_multiturn;
					cal->angle_count++;
					cal->theta_end = cal->angle_sum/(float)cal->angle_count;
				}
				if(cal->time < t_hold_end){return;}
				break;

			default:
				return;
		}
	}

	reset_foc(controller);

	if(cal->angle_count == 0){cal->theta_end = encoder->angle_multiturn;}
	float theta_range = fabsf(cal->theta_end - cal->theta_start);
	int ppairs = theta_range > 0.001f ? (int)roundf(2.0f*PI_F/theta_range) : 0;
	if((ppairs < 1) || (ppairs > PPAIRS_MAX)){
		/* Rotor barely moved, or moved a non-physical amount - stalled rotor,
		 * a bad encoder, or I_CAL too low.  Bail out before the pole-pair count
		 * can run calibration sampling off the end of its arrays.
		 * The bus voltage is part of the report because v_bus == 0 silences the
		 * whole power stage (svm() has no scale to work with), which looks
		 * exactly like a stalled motor here */
		cal_msg_push(cal, CAL_MSG_FAILED, ppairs, PPAIRS_MAX, controller->v_bus, theta_range);
		cal->failed = 1;
		return;
	}
	cal->ppairs = ppairs;

	if(cal->theta_start < cal->theta_end){
		cal->phase_order = 0;
	}
	else{
		cal->phase_order = 1;
	}
    cal_msg_push(cal, CAL_MSG_PHASE_ORDER, cal->phase_order, 0, 0.0f, 0.0f);
    cal_msg_push(cal, CAL_MSG_PPAIRS, cal->ppairs, 0, theta_range, 0.0f);
    cal_msg_push(cal, CAL_MSG_START_END, 0, 0, cal->theta_start, cal->theta_end);
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

/* ---- R / L / Ld-Lq / flux-linkage measurement ----------------------------
 *
 * Port of ODrive's Motor::measure_phase_resistance / measure_phase_inductance /
 * measure_ld_lq / measure_flux_linkage and their control laws.  Everything runs
 * in the control interrupt, so no printing here (milestones are queued).
 */

/* Physical volts per commanded voltage unit.  svm() turns (u - v_offset) into a
 * duty deviation of 0.5/v_scale with v_scale = v_bus*(DTC_MAX-DTC_MIN)*SQRT1_3,
 * and the phase-neutral voltage the inverter applies is v_bus times the duty
 * deviation.  The results below are scaled by this so the persisted R/L/lambda
 * values are in SI units.  Keep in sync with svm()/commutate(). */
#define CAL_V_PER_CMD	(0.5f/((DTC_MAX-DTC_MIN)*SQRT1_3))

/* Run one calibration control-law cycle: apply a dq voltage in the stationary
 * (theta = 0) or an arbitrary open-loop frame.  Returns 1 if the vector had to
 * be scaled to fit the bus voltage (the measurement voltage would then be
 * wrong, so callers treat that as a failure). */
static int cal_set_dq(ControllerStruct *controller, float theta, float v_d, float v_q){
	int overmodulated;

	controller->v_max = OVERMODULATION*controller->v_bus_filt*(DTC_MAX-DTC_MIN)*SQRT1_3;
	overmodulated = !(sqrtf(v_d*v_d + v_q*v_q) <= controller->v_max);
	controller->v_d = v_d;
	controller->v_q = v_q;
	limit_norm(&controller->v_d, &controller->v_q, controller->v_max);
	abc(theta, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w);
	svm(controller->v_max/OVERMODULATION, controller->v_u, controller->v_v, controller->v_w,
			&controller->dtc_u, &controller->dtc_v, &controller->dtc_w);
	set_dtc(controller);
	return overmodulated;
}

/* Stationary-frame currents, same convention as commutate()/PHASE_ORDER */
static void cal_alphabeta(const ControllerStruct *controller, float *i_alpha, float *i_beta){
	dq0(0.0f, controller->i_a, controller->i_b, controller->i_c, i_alpha, i_beta);
}

static void cal_rl_enter(CalStruct *cal, int loop_count, uint8_t stage){
	cal->rl.stage = stage;
	cal->rl.start_count = loop_count;
	cal->rl.tick = 0;
	cal->rl.v_sum = 0.0f;
	cal->rl.i_sum = 0.0f;
	cal->rl.v_count = 0;
	cal->rl.delta_i = 0.0f;
}

static void cal_rl_fail(CalStruct *cal, ControllerStruct *controller, uint8_t stage, float value, float aux){
	cal_msg_push(cal, CAL_MSG_RL_FAIL, stage, 0, value, aux);
	cal->failed = 1;
	reset_foc(controller);		// stop driving now; the FSM exits to the menu next cycle
}

/* Inductance from the accumulated current ripple: L = V*dt/|delta I|.
 *
 * delta_i is sum(sign(V)*(I-I_prev)), accumulated with the sign of test_voltage
 * *before* the toggle.  The PWM compare is preloaded and the ADC trigger is the
 * update event, so the current sampled by this ISR belongs to the interval
 * written two control cycles ago - one toggle behind that value.  The
 * accumulated magnitude is unaffected, so only |delta_i| is used here; do not
 * turn this into a signed formula (it reads negative on hardware).
 *
 * No R*I term belongs in the numerator: the samples sit at the ripple turning
 * points, where the average current over an interval is the DC component, and
 * for the Ld/Lq stages the lock voltage already cancels R*I_dc.  The remaining
 * resistive damping of the ripple is second order ((R*dt/L)^2/12). */
static float cal_ripple_inductance(const CalRlStruct *rl, float dt){
	float di = fabsf(rl->delta_i);
	if(!(di > 0.0f)){return 0.0f;}
	return CAL_V_PER_CMD*CAL_V_MAX*dt/di;
}

/* Elapsed time covered by the accumulated current-ripple samples: N samples
 * give N-1 differences */
static float cal_ripple_dt(uint32_t tick){
	return (tick > 1u) ? (float)(tick - 1u)*DT : 0.0f;
}

/* One cycle of the open-loop flux measurement: regulates Id = test_current and
 * Iq = 0 in the applied frame, advances the angle and drives the inverter.
 * Returns 1 when the voltage limit was hit (the current is not tracking). */
static int cal_flux_cycle(ControllerStruct *controller, CalRlStruct *rl, int accumulate){
	float i_d, i_q;
	int saturated;

	dq0(rl->theta, controller->i_a, controller->i_b, controller->i_c, &i_d, &i_q);
	rl->vd += (CAL_KI_FLUX*DT)*(rl->test_current - i_d);
	rl->vq += (CAL_KI_FLUX*DT)*(0.0f - i_q);

	saturated = !(fabsf(rl->vd) <= CAL_FLUX_V_MAX) || !(fabsf(rl->vq) <= CAL_FLUX_V_MAX);
	rl->vd = fast_fmaxf(fast_fminf(rl->vd, CAL_FLUX_V_MAX), -CAL_FLUX_V_MAX);
	rl->vq = fast_fmaxf(fast_fminf(rl->vq, CAL_FLUX_V_MAX), -CAL_FLUX_V_MAX);

	if(accumulate){
		rl->vq_sum += rl->vq;
		rl->id_sum += i_d;
		rl->iq_sum += i_q;
		rl->meas_count++;
	}

	/* Advance the applied angle before output (ODrive applies the new angle) */
	rl->theta += rl->omega*DT;
	if(rl->theta > PI_F){rl->theta -= TWO_PI_F;}

	return saturated || cal_set_dq(controller, rl->theta, rl->vd, rl->vq);
}

void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal, int loop_count){
	CalRlStruct *rl = &cal->rl;
	float i_alpha = 0.0f, i_beta = 0.0f, t;

	(void)encoder;

	if(rl->stage == RL_IDLE){
		/* I_CAL doubles as the test current for these measurements; clamp it to
		 * the board limit, because the R/L/Ld-Lq stages hold DC current */
		float i_test = fminf(I_CAL, I_MAX);
		if(!(i_test >= CAL_I_MIN)){
			cal_rl_fail(cal, controller, RL_IDLE, i_test, 0.0f);
			return;
		}
		rl->test_current = i_test;
		rl->i_low = 0.5f*i_test;
		rl->i_high = i_test;
		rl->test_voltage = 0.0f;
		rl->i_beta_filt = 0.0f;
		cal_msg_push(cal, CAL_MSG_RL_START, 0, 0, i_test, 0.0f);
		cal_rl_enter(cal, loop_count, RL_R_LOW);
		return;
	}

	/* Only the standstill stages need the stationary-frame currents; the flux
	 * stage does its own Park transform (double LUT work would eat into the
	 * 25 us ISR budget) */
	if(rl->stage < RL_FLUX_LOCK){
		cal_alphabeta(controller, &i_alpha, &i_beta);
	}
	t = (float)(loop_count - rl->start_count)*DT;

	switch(rl->stage){
		case RL_R_LOW:
		case RL_R_HIGH:{
			/* Integrator current source on the alpha axis.  The second step runs
			 * at a higher current; the dead-time term is current independent, so
			 * R = (V2 - V1)/(I2 - I1) needs no dead-time model.  The *measured*
			 * currents are used (not the targets), so a step that is still
			 * settling or hits the voltage clamp is harmless */
			float target = (rl->stage == RL_R_LOW) ? rl->i_low : rl->i_high;
			rl->test_voltage += (CAL_KI_R*DT)*(target - i_alpha);
			rl->test_voltage = fast_fmaxf(fast_fminf(rl->test_voltage, CAL_LOCK_V_MAX), -CAL_LOCK_V_MAX);
			rl->i_beta_filt += (CAL_IBETA_FILT*DT)*(i_beta - rl->i_beta_filt);
			if(cal_set_dq(controller, 0.0f, rl->test_voltage, 0.0f)){
				cal_rl_fail(cal, controller, rl->stage, rl->test_voltage, controller->v_bus_filt);
				return;
			}

			if(t >= (CAL_R_SETTLE_S - CAL_R_AVG_S)){
				rl->v_sum += rl->test_voltage;
				rl->i_sum += i_alpha;
				rl->v_count++;
			}
			if(t < CAL_R_SETTLE_S){break;}

			{
				float v = rl->v_count > 0 ? rl->v_sum/(float)rl->v_count : rl->test_voltage;
				float i_end = rl->v_count > 0 ? rl->i_sum/(float)rl->v_count : i_alpha;
				if(rl->stage == RL_R_LOW){
					rl->v1 = v;
					rl->i1_meas = i_end;
					cal_msg_push(cal, CAL_MSG_R_STEP, 0, 0, rl->v1, i_end);
					cal_rl_enter(cal, loop_count, RL_R_HIGH);
					break;
				}
				rl->v2 = v;
				rl->i2_meas = i_end;
				{
					float di = rl->i2_meas - rl->i1_meas;
					float r;
					if(!(di >= 0.1f) || !(di >= 0.2f*rl->i_low)){
						cal_rl_fail(cal, controller, RL_R_HIGH, rl->v1, di);
						return;
					}
					r = CAL_V_PER_CMD*(rl->v2 - rl->v1)/di;
					if(!(r >= CAL_R_MIN) || !(r <= CAL_R_MAX) ||
							(fabsf(rl->i_beta_filt) > 0.5f*rl->i2_meas)){
						cal_rl_fail(cal, controller, RL_R_HIGH, r, di);
						return;
					}
					R_PHASE = r;
				}
				cal_msg_push(cal, CAL_MSG_R_DONE, 0, 0, R_PHASE, 0.0f);
			}
			rl->test_voltage = CAL_V_MAX;
			cal_rl_enter(cal, loop_count, RL_L);
			break;
		}

		case RL_L:{
			/* Toggle the alpha voltage every cycle; the current change over the
			 * previous interval was driven by the voltage *before* the flip, so
			 * delta-I is accumulated with the pre-flip sign (same convention as
			 * ODrive's on_measurement/on_shlk scheme) */
			if(rl->tick == 0){
				rl->last_ialpha = i_alpha;
			}
			else{
				float sign = (rl->test_voltage >= 0.0f) ? 1.0f : -1.0f;
				rl->delta_i += sign*(i_alpha - rl->last_ialpha);
				rl->last_ialpha = i_alpha;
			}
			rl->tick++;
			rl->test_voltage = -rl->test_voltage;
			if(cal_set_dq(controller, 0.0f, rl->test_voltage, 0.0f)){
				cal_rl_fail(cal, controller, RL_L, rl->test_voltage, controller->v_bus_filt);
				return;
			}
			if(t < CAL_L_MEAS_S){break;}

			{
				float l = cal_ripple_inductance(rl, cal_ripple_dt(rl->tick));
				if(!(l >= CAL_L_MIN) || !(l <= CAL_L_MAX)){
					cal_rl_fail(cal, controller, RL_L, l, fabsf(rl->delta_i));
					return;
				}
				L_Q = l;			// provisional; the Ld/Lq stage refines it
				cal_msg_push(cal, CAL_MSG_L_DONE, 0, 0, l, 0.0f);
			}
			rl->lock_v_alpha = 0.0f;
			rl->lock_v_beta = 0.0f;
			cal_rl_enter(cal, loop_count, RL_LD_LOCK);
			break;
		}

		case RL_LD_LOCK:{
			/* DC current on alpha aligns the rotor d-axis with alpha */
			rl->lock_v_alpha += (CAL_KI_LOCK*DT)*(rl->test_current - i_alpha);
			rl->lock_v_alpha = fast_fmaxf(fast_fminf(rl->lock_v_alpha, CAL_LOCK_V_MAX), -CAL_LOCK_V_MAX);
			if(cal_set_dq(controller, 0.0f, rl->lock_v_alpha, 0.0f)){
				cal_rl_fail(cal, controller, RL_LD_LOCK, rl->lock_v_alpha, controller->v_bus_filt);
				return;
			}
			if(t < CAL_LD_LOCK_S){break;}
			if(fabsf(i_alpha - rl->test_current) > 0.3f*rl->test_current){
				cal_rl_fail(cal, controller, RL_LD_LOCK, i_alpha, rl->lock_v_alpha);
				return;
			}
			rl->test_voltage = CAL_V_MAX;
			cal_rl_enter(cal, loop_count, RL_LD_MEAS);
			break;
		}

		case RL_LD_MEAS:{
			if(rl->tick == 0){
				rl->last_ialpha = i_alpha;
			}
			else{
				float sign = (rl->test_voltage >= 0.0f) ? 1.0f : -1.0f;
				rl->delta_i += sign*(i_alpha - rl->last_ialpha);
				rl->last_ialpha = i_alpha;
			}
			rl->tick++;
			rl->test_voltage = -rl->test_voltage;
			if(cal_set_dq(controller, 0.0f, rl->lock_v_alpha + rl->test_voltage, 0.0f)){
				cal_rl_fail(cal, controller, RL_LD_MEAS, rl->lock_v_alpha + rl->test_voltage, controller->v_bus_filt);
				return;
			}
			if(t < CAL_LD_MEAS_S){break;}

			{
				float ld = cal_ripple_inductance(rl, cal_ripple_dt(rl->tick));
				if(!(ld >= CAL_L_MIN) || !(ld <= CAL_L_MAX)){
					cal_rl_fail(cal, controller, RL_LD_MEAS, ld, fabsf(rl->delta_i));
					return;
				}
				rl->ld = ld;
			}
			rl->lock_v_beta = 0.0f;
			cal_rl_enter(cal, loop_count, RL_LQ_LOCK);
			break;
		}

		case RL_LQ_LOCK:{
			/* DC current on beta puts the q-axis on alpha */
			rl->lock_v_beta += (CAL_KI_LOCK*DT)*(rl->test_current - i_beta);
			rl->lock_v_beta = fast_fmaxf(fast_fminf(rl->lock_v_beta, CAL_LOCK_V_MAX), -CAL_LOCK_V_MAX);
			if(cal_set_dq(controller, 0.0f, 0.0f, rl->lock_v_beta)){
				cal_rl_fail(cal, controller, RL_LQ_LOCK, rl->lock_v_beta, controller->v_bus_filt);
				return;
			}
			if(t < CAL_LD_LOCK_S){break;}
			if(fabsf(i_beta - rl->test_current) > 0.3f*rl->test_current){
				cal_rl_fail(cal, controller, RL_LQ_LOCK, i_beta, rl->lock_v_beta);
				return;
			}
			rl->test_voltage = CAL_V_MAX;
			cal_rl_enter(cal, loop_count, RL_LQ_MEAS);
			break;
		}

		case RL_LQ_MEAS:{
			if(rl->tick == 0){
				rl->last_ialpha = i_alpha;
			}
			else{
				float sign = (rl->test_voltage >= 0.0f) ? 1.0f : -1.0f;
				rl->delta_i += sign*(i_alpha - rl->last_ialpha);
				rl->last_ialpha = i_alpha;
			}
			rl->tick++;
			rl->test_voltage = -rl->test_voltage;
			if(cal_set_dq(controller, 0.0f, rl->test_voltage, rl->lock_v_beta)){
				cal_rl_fail(cal, controller, RL_LQ_MEAS, rl->test_voltage, controller->v_bus_filt);
				return;
			}
			if(t < CAL_LD_MEAS_S){break;}

			{
				float lq = cal_ripple_inductance(rl, cal_ripple_dt(rl->tick));
				if(!(lq >= CAL_L_MIN) || !(lq <= CAL_L_MAX)){
					cal_rl_fail(cal, controller, RL_LQ_MEAS, lq, fabsf(rl->delta_i));
					return;
				}
				rl->lq = lq;
			}
			L_D = rl->ld;
			L_Q = rl->lq;
			cal_msg_push(cal, CAL_MSG_LD_LQ, 0, 0, rl->ld, rl->lq);

			/* Set up the open-loop flux measurement */
			rl->theta = 0.0f;
			rl->omega = 0.0f;
			rl->vd = 0.0f;
			rl->vq = 0.0f;
			rl->vq_sum = 0.0f;
			rl->id_sum = 0.0f;
			rl->iq_sum = 0.0f;
			rl->meas_count = 0;
			cal_rl_enter(cal, loop_count, RL_FLUX_LOCK);
			break;
		}

		case RL_FLUX_LOCK:
			if(cal_flux_cycle(controller, rl, 0)){
				cal_rl_fail(cal, controller, RL_FLUX_LOCK, rl->vd, rl->vq);
				return;
			}
			if(t < CAL_FLUX_LOCK_S){break;}
			cal_rl_enter(cal, loop_count, RL_FLUX_RAMP);
			break;

		case RL_FLUX_RAMP:
			rl->omega += CAL_FLUX_ACCEL*DT;
			if(rl->omega >= CAL_FLUX_OMEGA){
				rl->omega = CAL_FLUX_OMEGA;
				cal_rl_enter(cal, loop_count, RL_FLUX_MEAS);
			}
			if(cal_flux_cycle(controller, rl, 0)){
				cal_rl_fail(cal, controller, RL_FLUX_RAMP, rl->vd, rl->vq);
				return;
			}
			break;

		case RL_FLUX_MEAS:{
			if(cal_flux_cycle(controller, rl, t >= CAL_FLUX_SETTLE_S)){
				cal_rl_fail(cal, controller, RL_FLUX_MEAS, rl->vd, rl->vq);
				return;
			}
			if(t < (CAL_FLUX_SETTLE_S + CAL_FLUX_MEAS_S)){break;}

			{
				float vq_avg, id_avg, iq_avg, lambda;
				if(rl->meas_count == 0){
					cal_rl_fail(cal, controller, RL_FLUX_MEAS, 0.0f, 0.0f);
					return;
				}
				vq_avg = rl->vq_sum/(float)rl->meas_count;
				id_avg = rl->id_sum/(float)rl->meas_count;
				iq_avg = rl->iq_sum/(float)rl->meas_count;
				/* Steady-state q-axis equation: Vq = R*Iq + omega*Ld*Id + omega*lambda */
				lambda = (CAL_V_PER_CMD*vq_avg - R_PHASE*iq_avg - CAL_FLUX_OMEGA*L_D*id_avg)/CAL_FLUX_OMEGA;
				if((fabsf(id_avg - rl->test_current) > 0.3f*rl->test_current) ||
						!(lambda >= CAL_LAMBDA_MIN) || !(lambda <= CAL_LAMBDA_MAX)){
					cal_rl_fail(cal, controller, RL_FLUX_MEAS, lambda, id_avg);
					return;
				}
				KT = 1.5f*PPAIRS*lambda;		// Kt = (3/2) * pole pairs * lambda
				controller->flux_linkage = lambda;
				cal_msg_push(cal, CAL_MSG_FLUX, 0, 0, lambda, KT);
			}
			reset_foc(controller);		// stop driving before the main loop saves
			cal->done_rl = 1;
			break;
		}

		default:
			cal_rl_fail(cal, controller, rl->stage, 0.0f, 0.0f);
			break;
	}
}

