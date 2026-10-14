/*
 * foc.c
 *
 *  Created on: Aug 2, 2020
 *      Author: ben
 */

#include "foc.h"
#include "adc.h"
#include "tim.h"
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"
#include "timing.h"

extern uint16_t adc_measurements_[3];

void set_dtc(ControllerStruct *controller){

	/* Invert duty cycle if that's how hardware is configured */

	float dtc_u = controller->dtc_u;
	float dtc_v = controller->dtc_v;
	float dtc_w = controller->dtc_w;

	if(INVERT_DTC){
		dtc_u = 1.0f - controller->dtc_u;
		dtc_v = 1.0f - controller->dtc_v;
		dtc_w = 1.0f - controller->dtc_w;
	}
	/* Handle phase order swapping so that voltage/current/torque match encoder direction */
	if(!PHASE_ORDER){
        TIM_PWM->CCDAT1 = ((TIM_PWM->AR))*dtc_u;
        TIM_PWM->CCDAT2 = ((TIM_PWM->AR))*dtc_v;
        TIM_PWM->CCDAT3 = ((TIM_PWM->AR))*dtc_w;
		// __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, ((TIM_PWM->AR))*dtc_u);
		// __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, ((TIM_PWM->AR))*dtc_v);
		// __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, ((TIM_PWM->AR))*dtc_w);
	}
	else{
        TIM_PWM->CCDAT2 = ((TIM_PWM->AR))*dtc_u;
        TIM_PWM->CCDAT1 = ((TIM_PWM->AR))*dtc_v;
        TIM_PWM->CCDAT3 = ((TIM_PWM->AR))*dtc_w;
		// __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, ((TIM_PWM->AR))*dtc_u);
		// __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, ((TIM_PWM->AR))*dtc_v);
		// __HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, ((TIM_PWM->AR))*dtc_w);
	}
}

void analog_sample (ControllerStruct *controller){
	/* Phase current conversions are hardware-triggered by the PWM timer update
	 * event - ADC1 and ADC2 each run an injected channel off ATIM1_TRGO - so they
	 * are sampled at the same point in the PWM cycle every loop, in the middle of
	 * the low-side conduction window, the same event that runs this interrupt.
	 * ADC1 converts phase A, ADC2 phase B (VBUS is on ADC2's regular channel).
	 * Wait for BOTH injected sequence-complete flags: they are separate
	 * converters, so their completion is not guaranteed to be simultaneous.
	 * The conversion finishes within a microsecond of the interrupt starting, so
	 * just wait for it.  If it never finishes (ADC or trigger misconfiguration),
	 * the watchdog resets the chip. */
	ISR_TIMING_SPAN_BEGIN(t_adc_wait);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_JENDC) == RESET){;}
	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_JENDC) == RESET){;}
	while(ADC_GetFlagStatus(ADC3, ADC_FLAG_JENDC) == RESET){;}
	ISR_TIMING_SPAN_END(ISR_EXT_ADC_WAIT, t_adc_wait);

	ADC_ClearFlag(ADC1, ADC_FLAG_JENDC);
	ADC_ClearFlag(ADC2, ADC_FLAG_JENDC);
	ADC_ClearFlag(ADC3, ADC_FLAG_JENDC);

	/* Handle phase order swapping so that voltage/current/torque match encoder direction */
	if(!PHASE_ORDER){
		controller->adc_a_raw = ADC1->JDAT1;
		controller->adc_b_raw = ADC2->JDAT1;
		controller->adc_c_raw = ADC3->JDAT1;
	}
	else{
		controller->adc_a_raw = ADC2->JDAT1;
		controller->adc_b_raw = ADC1->JDAT1;
		controller->adc_c_raw = ADC3->JDAT1;
	}

	controller->adc_vbus_raw = adc_measurements_[0];	// ADC2 regular channel 12 is VBUS
	controller->v_bus = (float)controller->adc_vbus_raw*V_SCALE;
	/* Filtered here rather than in commutate() so it tracks in every state.
	 * It used to be updated only while commutating, which left it stuck at its
	 * power-on value of zero whenever the gate drive was off - so CAN telemetry
	 * and the legacy packed reply both reported 0 V outside motor mode, and
	 * v_max started from zero on entry to motor mode until the filter caught
	 * up.  Same filter, same update rate while commutating: no change to
	 * control behaviour, but the value is now always valid */
	controller->v_bus_filt = (1.0f-VBUS_FILT_ALPHA)*controller->v_bus_filt + VBUS_FILT_ALPHA*controller->v_bus;

    controller->i_a = controller->i_scale*(float)(controller->adc_a_raw - controller->adc_a_offset);    // Calculate phase currents from ADC readings
    controller->i_b = controller->i_scale*(float)(controller->adc_b_raw - controller->adc_b_offset);
    controller->i_c = controller->i_scale*(float)(controller->adc_c_raw - controller->adc_c_offset);
    // controller->i_c = -controller->i_a - controller->i_b;

    // ADC_EnableSoftwareStartConv(ADC2, ENABLE);
}

void abc( float theta, float d, float q, float *a, float *b, float *c){
    /* Inverse DQ0 Transform
    Phase current amplitude = lengh of dq vector
    i.e. iq = 1, id = 0, peak phase current of 1 */

    float sf, cf;
    sincos_lut(theta, &sf, &cf);

    *a = cf*d - sf*q;
    *b = (SQRT3_2*sf-.5f*cf)*d - (-SQRT3_2*cf-.5f*sf)*q;
    *c = (-SQRT3_2*sf-.5f*cf)*d - (SQRT3_2*cf-.5f*sf)*q;
    }


void dq0(float theta, float a, float b, float c, float *d, float *q){
    /* DQ0 Transform
    Phase current amplitude = lengh of dq vector
    i.e. iq = 1, id = 0, peak phase current of 1*/

    float sf, cf;
    sincos_lut(theta, &sf, &cf);

    *d = 0.6666667f*(cf*a + (SQRT3_2*sf-.5f*cf)*b + (-SQRT3_2*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-SQRT3_2*cf-.5f*sf)*b - (SQRT3_2*cf-.5f*sf)*c);

    }

void svm(float v_scale, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w){
    /* Space Vector Modulation
     u,v,w amplitude = v_bus for full modulation depth
     v_scale normalises the voltage vector to the bus voltage.  It must NOT
     include OVERMODULATION: the over-modulation factor is already applied where
     the voltage limit controller->v_max is formed, and while it was applied here
     as well the two cancelled out, so the macro had no effect at all.  The
     [DTC_MIN, DTC_MAX] clamp below is what limits the modulation depth */

    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))*0.5f;
    float v_midpoint = .5f*(DTC_MAX+DTC_MIN);
    float scale = v_scale > 0.0f ? .5f/v_scale : 0.0f;	// no drive voltage until the bus voltage measurement is valid

    *dtc_u = fast_fminf(fast_fmaxf(((u - v_offset)*scale + v_midpoint ), DTC_MIN), DTC_MAX);
    *dtc_v = fast_fminf(fast_fmaxf(((v - v_offset)*scale + v_midpoint ), DTC_MIN), DTC_MAX);
    *dtc_w = fast_fminf(fast_fmaxf(((w - v_offset)*scale + v_midpoint ), DTC_MIN), DTC_MAX);

    }

void zero_current(ControllerStruct *controller){
	/* Measure zero-current ADC offset */

    int adc_a_offset = 0;
    int adc_b_offset = 0;
    int adc_c_offset = 0;
    int n = 60000;
    controller->dtc_u = 0.f;
    controller->dtc_v = 0.f;
    controller->dtc_w = 0.f;
    set_dtc(controller);

    for (int i = 0; i<n; i++){               // Average n samples
    	analog_sample(controller);
    	adc_a_offset +=  controller->adc_a_raw;
    	adc_b_offset += controller->adc_b_raw;
    	adc_c_offset += controller->adc_c_raw;
     }
    controller->adc_a_offset = adc_a_offset/n;
    controller->adc_b_offset = adc_b_offset/n;
    controller->adc_c_offset = adc_c_offset/n;

    }

void init_controller_params(ControllerStruct *controller){

	controller->ki_d = KI_D;
    controller->ki_q = KI_Q;
    controller->k_d = K_SCALE*I_BW;
    controller->k_q = K_SCALE*I_BW;
    controller->ki_fw = .1f*controller->ki_d;
    controller->flux_linkage = KT/(1.5f*PPAIRS);
    /* The current sense gain is fixed in hardware (on-chip PGA, differential
     * gain 32 - see pga.c).  The old firmware doubled i_scale above 40 A to
     * follow a DRV8323 CSA gain change; that driver is not on this board, so
     * the same I_SCALE is correct for every I_MAX */
    controller->i_scale = I_SCALE;
    controller->fault = 0;
    /* Seed the overcurrent latch threshold; check_faults() keeps it in sync
     * with I_MAX every cycle so a setup change takes effect immediately */
    controller->i_oc_limit = I_OC_FAULT_FACTOR*I_MAX;

    }

void reset_foc(ControllerStruct *controller){

	TIM_PWM->CCDAT3 = ((TIM_PWM->AR))*(0.5f);
	TIM_PWM->CCDAT1 = ((TIM_PWM->AR))*(0.5f);
	TIM_PWM->CCDAT2 = ((TIM_PWM->AR))*(0.5f);
    controller->i_d_des = 0;
    controller->i_q_des = 0;
    controller->i_d = 0;
    controller->i_q = 0;
    controller->i_q_filt = 0;
    controller->q_int = 0;
    controller->d_int = 0;
    controller->v_q = 0;
    controller->v_d = 0;
    controller->fw_int = 0;
    controller->otw_flag = 0;

    }

void reset_observer(ObserverStruct *observer){
/*
    observer->temperature = 25.0f;
    observer->temp_measured = 25.0f;
    //observer->resistance = .1f;
*/
}

void update_observer(ControllerStruct *controller, ObserverStruct *observer)
{
	/*
    /// Update observer estimates ///
    // Resistance observer //
    // Temperature Observer //
    observer->delta_t = (float)observer->temperature - T_AMBIENT;
    float i_sq = controller->i_d*controller->i_d + controller->i_q*controller->i_q;
    observer->q_in = (R_NOMINAL*1.5f)*(1.0f + .00393f*observer->delta_t)*i_sq;
    observer->q_out = observer->delta_t*R_TH;
    observer->temperature += (INV_M_TH*DT)*(observer->q_in-observer->q_out);

    //float r_d = (controller->v_d*(DTC_MAX-DTC_MIN) + SQRT3*controller->dtheta_elec*(L_Q*controller->i_q))/(controller->i_d*SQRT3);
    float r_q = (controller->v_q*(DTC_MAX-DTC_MIN) - SQRT3*controller->dtheta_elec*(L_D*controller->i_d + WB))/(controller->i_q*SQRT3);
    observer->resistance = r_q;//(r_d*controller->i_d + r_q*controller->i_q)/(controller->i_d + controller->i_q); // voltages more accurate at higher duty cycles

    //observer->resistance = controller->v_q/controller->i_q;
    if(isnan(observer->resistance) || isinf(observer->resistance)){observer->resistance = R_NOMINAL;}
    float t_raw = ((T_AMBIENT + ((observer->resistance/R_NOMINAL) - 1.0f)*254.5f));
    if(t_raw > 200.0f){t_raw = 200.0f;}
    else if(t_raw < 0.0f){t_raw = 0.0f;}
    observer->temp_measured = .999f*observer->temp_measured + .001f*t_raw;
    float e = (float)observer->temperature - observer->temp_measured;
    observer->trust = (1.0f - .004f*fminf(abs(controller->dtheta_elec), 250.0f)) * (.01f*(fminf(i_sq, 100.0f)));
    observer->temperature -= observer->trust*.0001f*e;
    //myprintf("%.3f\n\r", e);

    if(observer->temperature > TEMP_MAX){controller->otw_flag = 1;}
    else{controller->otw_flag = 0;}
    */
}

void field_weaken(ControllerStruct *controller)
{
       /// Field Weakening ///

       controller->fw_int += controller->ki_fw*(controller->v_max - 1.0f - controller->v_ref);
       controller->fw_int = fast_fmaxf(fast_fminf(controller->fw_int, 0.0f), -fast_fminf(I_FW_MAX, controller->i_max));
       /* Reduce the q current reference magnitude as field weakening increases, saturating at zero */
       if(controller->i_q_des > 0.0f){controller->i_q_des = fast_fmaxf(controller->i_q_des + controller->fw_int, 0.0f);}
       else{controller->i_q_des = fast_fminf(controller->i_q_des - controller->fw_int, 0.0f);}
       controller->i_d_des = controller->fw_int;
       float q_max_squared = controller->i_max*controller->i_max - controller->i_d_des*controller->i_d_des;
       float q_max = q_max_squared > 0.0f ? sqrtf(q_max_squared) : 0.0f;
       controller->i_q_des = fast_fmaxf(fast_fminf(controller->i_q_des, q_max), -q_max);


}
void commutate(ControllerStruct *controller, EncoderStruct *encoder)
{
	/* Do Field Oriented Control */

		controller->theta_elec = encoder->elec_angle;
		controller->dtheta_elec = encoder->elec_velocity;
		controller->dtheta_mech = encoder->velocity/GR;
		controller->theta_mech = encoder->angle_multiturn/GR;

       /// Commutation  ///
       dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents - 3.8 us

       controller->i_q_filt = (1.0f-CURRENT_FILT_ALPHA)*controller->i_q_filt + CURRENT_FILT_ALPHA*controller->i_q;	// these aren't used for control but are sometimes nice for debugging
       controller->i_d_filt = (1.0f-CURRENT_FILT_ALPHA)*controller->i_d_filt + CURRENT_FILT_ALPHA*controller->i_d;
       /* v_bus_filt is maintained by analog_sample(), which runs every cycle in
        * every state - see the note there */

       controller->v_max = OVERMODULATION*controller->v_bus_filt*(DTC_MAX-DTC_MIN)*SQRT1_3;
       controller->i_max = I_MAX; //I_MAX*(!controller->otw_flag) + I_MAX_CONT*controller->otw_flag;

       limit_norm(&controller->i_d_des, &controller->i_q_des, controller->i_max);	// 2.3 us

       /// PI Controller ///
       float i_d_error = controller->i_d_des - controller->i_d;
       float i_q_error = controller->i_q_des - controller->i_q;

       // Calculate decoupling feed-forward voltages //

       float v_d_ff = 0.0f;//-SQRT3*controller->dtheta_elec*L_Q*controller->i_q;
       float v_q_ff = 0.0f;//SQRT3*controller->dtheta_elec*(0.0f*L_D*controller->i_d + controller->flux_linkage);

       controller->v_d = controller->k_d*i_d_error + controller->d_int + v_d_ff;

       controller->v_d = fast_fmaxf(fast_fminf(controller->v_d, controller->v_max), -controller->v_max);

       controller->d_int += controller->k_d*controller->ki_d*i_d_error;
       controller->d_int = fast_fmaxf(fast_fminf(controller->d_int, controller->v_max), -controller->v_max);
       float vq_max = controller->v_max;//sqrtf(controller->v_max*controller->v_max - controller->v_d*controller->v_d);

       controller->v_q = controller->k_q*i_q_error + controller->q_int + v_q_ff;
       controller->q_int += controller->k_q*controller->ki_q*i_q_error;
       controller->q_int = fast_fmaxf(fast_fminf(controller->q_int, controller->v_max), -controller->v_max);
       controller->v_ref = sqrtf(controller->v_d*controller->v_d + controller->v_q*controller->v_q);
       controller->v_q = fast_fmaxf(fast_fminf(controller->v_q, vq_max), -vq_max);

       limit_norm(&controller->v_d, &controller->v_q, controller->v_max);

       abc(controller->theta_elec + 1.5f*DT*controller->dtheta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
       /* The SVM is normalised to the bus voltage, so pass v_max WITHOUT the
        * over-modulation factor - see svm() */
       svm(controller->v_max/OVERMODULATION, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation

       set_dtc(controller);

    }
float angle_est = 0; 

void commutate_open_loop(ControllerStruct *controller, EncoderStruct *encoder)
{
    	/* Do Field Oriented Control */

		controller->theta_elec = encoder->elec_angle;
        angle_est = fmodf(controller->theta_elec, TWO_PI_F);
		controller->dtheta_elec = encoder->elec_velocity;
		controller->dtheta_mech = encoder->velocity/GR;
		controller->theta_mech = encoder->angle_multiturn/GR;

       /// Commutation  ///
       dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents - 3.8 us

       controller->i_q_filt = (1.0f-CURRENT_FILT_ALPHA)*controller->i_q_filt + CURRENT_FILT_ALPHA*controller->i_q;	// these aren't used for control but are sometimes nice for debugging
       controller->i_d_filt = (1.0f-CURRENT_FILT_ALPHA)*controller->i_d_filt + CURRENT_FILT_ALPHA*controller->i_d;
       /* v_bus_filt is maintained by analog_sample(), which runs every cycle in
        * every state - updating it again here would apply the filter twice per
        * cycle and make it behave differently from motor mode */

       controller->v_max = OVERMODULATION*controller->v_bus_filt*(DTC_MAX-DTC_MIN)*SQRT1_3;
       controller->v_margin = controller->v_max - controller->v_ref;
       controller->i_max = I_MAX; //I_MAX*(!controller->otw_flag) + I_MAX_CONT*controller->otw_flag;


       if(controller->i_q > controller->i_mag_max){
    	   controller->i_mag_max = controller->i_q;
       }

        controller->v_d =  controller->i_d_des/6.0f;    //开环转       
        controller->v_q = controller->i_q_des/6.0f;

       limit_norm(&controller->v_d, &controller->v_q, controller->v_max);

       abc(controller->theta_elec + 1.5f*DT*controller->dtheta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
       /* bus-derived scale, i.e. v_max without the over-modulation factor (see svm()) */
       svm(controller->v_max/OVERMODULATION, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation

       set_dtc(controller);

    }

void torque_control(ControllerStruct *controller){
	controller->t_ff_filt = 0.9f*controller->t_ff_filt + 0.1f*controller->t_ff;
    float torque_des = controller->kp*(controller->p_des - controller->theta_mech) + controller->t_ff_filt + controller->kd*(controller->v_des - controller->dtheta_mech);
    controller->i_q_des = fast_fmaxf(fast_fminf(torque_des/(KT*GR), controller->i_max), -controller->i_max);
    if(controller->v_bus > V_BUS_MAX){controller->i_q_des = 0;}
    controller->i_d_des = 0.0f;

    }



void check_faults(ControllerStruct *controller){
	/* Latched software faults, checked every commutation cycle in all states.
	 * A nonzero fault word forces the FSM into FAULT_MODE (gates off) until
	 * the user explicitly exits to the menu.  The value that tripped each
	 * fault is captured for the fault report. */
	/* Track I_MAX live, so raising/lowering it in setup takes effect at once
	 * (the threshold used to be computed once at boot) */
	controller->i_oc_limit = I_OC_FAULT_FACTOR*I_MAX;

	if(controller->v_bus > V_BUS_FAULT){
		if(!(controller->fault & FAULT_OV)){controller->fault_vbus = controller->v_bus;}
		controller->fault |= FAULT_OV;
	}
	/* Overcurrent is only checked while the gate drive is on.  With the FETs
	 * off the phase current is noise around zero, so a low threshold latches on
	 * nothing - and because a latched fault forces FAULT_MODE from any state,
	 * that locks the user out of the very menu needed to raise the limit.  A
	 * zero I_MAX (an erased chip, or 0.0 entered in setup, which is allowed)
	 * made that happen on the first control cycle after boot - so a zero
	 * threshold disables the check instead.
	 * Overvoltage above is deliberately still checked in every state: the bus
	 * can be driven over the limit with the gates off. */
	if(controller->gate_drive_enabled && (controller->i_oc_limit > 0.0f)){
		float i_abs = fmaxf3(fabsf(controller->i_a), fabsf(controller->i_b), fabsf(controller->i_c));
		if(i_abs > controller->i_oc_limit){
			if(!(controller->fault & FAULT_OC)){controller->fault_iabs = i_abs;}
			controller->fault |= FAULT_OC;
		}
	}
}

void zero_commands(ControllerStruct * controller){
	controller->t_ff = 0;
	controller->kp = 0;
	controller->kd = 0;
	controller->p_des = 0;
	controller->v_des = 0;
	controller->i_q_des = 0;
}
