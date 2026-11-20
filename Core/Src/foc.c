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
#include "cogging.h"
#include "timing.h"
#include <math.h>

extern uint16_t adc_measurements_[3];
/* 位置指令的允许区间要读实测位置（见文件末尾的 pos_cmd_*） */
extern EncoderStruct comm_encoder;

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
	/* The PWM peak update triggers all three injected ADCs and this ISR
	 * once per 40 kHz cycle, in the low-side conduction window.  ADC2 also
	 * runs the regular VBUS sequence, so wait for each independent injected
	 * sequence to finish.  If a trigger or converter fails, the watchdog
	 * resets the chip. */
	ISR_TIMING_SPAN_BEGIN(t_adc_wait);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_JENDC) == RESET){;}
	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_JENDC) == RESET){;}
	while(ADC_GetFlagStatus(ADC3, ADC_FLAG_JENDC) == RESET){;}
	ISR_TIMING_SPAN_END(ISR_EXT_ADC_WAIT, t_adc_wait);

	ADC_ClearFlag(ADC1, ADC_FLAG_JENDC);
	ADC_ClearFlag(ADC2, ADC_FLAG_JENDC);
	ADC_ClearFlag(ADC3, ADC_FLAG_JENDC);

	/* Raw samples and zero offsets stay bound to physical ADC1/2/3, even
	 * when calibration changes PHASE_ORDER.  Swap only corrected currents. */
	controller->adc_a_raw = ADC3->JDAT1;
	controller->adc_b_raw = ADC2->JDAT1;
	controller->adc_c_raw = ADC1->JDAT1;

	controller->adc_vbus_raw = adc_measurements_[1];	// ADC3 regular channel 4 is VBUS
	controller->v_bus = (float)controller->adc_vbus_raw*V_SCALE;
	/* Filtered here rather than in commutate() so it tracks in every state.
	 * It used to be updated only while commutating, which left it stuck at its
	 * power-on value of zero whenever the gate drive was off - so CAN telemetry
	 * and the legacy packed reply both reported 0 V outside motor mode, and
	 * v_max started from zero on entry to motor mode until the filter caught
	 * up.  Same filter, same update rate while commutating: no change to
	 * control behaviour, but the value is now always valid */
	controller->v_bus_filt = (1.0f-VBUS_FILT_ALPHA)*controller->v_bus_filt + VBUS_FILT_ALPHA*controller->v_bus;

	controller->i_a = controller->i_scale*(float)(controller->adc_a_raw - controller->adc_a_offset);
	controller->i_b = controller->i_scale*(float)(controller->adc_b_raw - controller->adc_b_offset);
	controller->i_c = controller->i_scale*(float)(controller->adc_c_raw - controller->adc_c_offset);
	if(PHASE_ORDER){
		float i_a = controller->i_a;
		controller->i_a = controller->i_b;
		controller->i_b = i_a;
	}
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
	/* Measure physical ADC1/2/3 zero offsets, independent of PHASE_ORDER. */

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

/* 1 - exp(-x) 的 Padé 近似（x = R*DT/L），x 钳到 [0,3]：
 * x=0.23 误差 <0.02%，x=1 误差 0.08%，x=2 误差 0.9%。不用 expf 是因为
 * current_loop_retune() 会在状态切换时被控制 ISR 调用 */
static float one_minus_exp_neg(float x)
{
	if (!(x > 0.0f)) { return 0.0f; }
	if (x > 3.0f) { x = 3.0f; }
	return 12.0f*x/(12.0f + 6.0f*x + x*x);
}

/* ---- 电流环 PI 增益：由标定的 R_PHASE / L_D / L_Q 推导 --------------------
 *
 * 对象是 dq 域的 R-L 环节，PI 做极点对消（零点对准 -R/L），于是 config.i_bw
 * 就是**真正的穿越频率**：
 *
 *     k  = 2*pi*I_BW*L / V_PER_CMD     [指令 V/A]  -> |k*V_PER_CMD/(j*w*L)| = 1 @ w = 2*pi*I_BW
 *     ki = 1 - exp(-R*DT/L)                        -> PI 零点 = R/L（每拍）
 *
 * 为什么不能用固定常数：k 正比于 L，ki 只取决于 R/L，两者都是电机参数。写死一套
 * 只对"L≈24 µH、R/L≈1800 rad/s"的电机成立；同样的常数换一台电机，穿越频率会从
 * 9 Hz（5 Ω / 300 µH 的云台电机）一路变到 2.3 kHz（10 µH 的高转速电机）——要么慢
 * 得做不了外环，要么相位裕度掉到 50° 边缘。详见 docs/DEV.md 的"电流环增益"。
 *
 * 未标定（R_PHASE<=0 或 L<=0，出厂默认 R=0）时退回 hw_config.h 的
 * K_SCALE/KI_D/KI_Q 常量，行为与改动前逐位一致。
 */
void current_loop_retune(ControllerStruct *controller){
	float bw = I_BW;
	const float r = R_PHASE;
	const float ld = L_D;
	const float lq = L_Q;

	/* config_apply_limits() 已经把 I_BW 钳到 [100,4000]；这里只挡住"增益变成
	 * 0 或 NaN"这一种致命情况（100 就是 params.json 里的下限） */
	if (!(bw > 0.0f) || !(bw < 1.0e6f)) { bw = 100.0f; }

	if ((r > 0.0f) && (ld > 0.0f) && (lq > 0.0f)) {
		const float wc = 6.28318531f*bw;
		float k_d = wc*ld/V_PER_CMD;
		float k_q = wc*lq/V_PER_CMD;

		/* 防手误：l_d/l_q 写成量级错误的值时增益会大 100 倍直接失稳。
		 * 本机（45 µH @ 1 kHz）只需要 0.23，10 mH 级电机才用得到 10 */
		if (k_d > I_LOOP_K_MAX) { k_d = I_LOOP_K_MAX; }
		if (k_q > I_LOOP_K_MAX) { k_q = I_LOOP_K_MAX; }
		controller->k_d = k_d;
		controller->k_q = k_q;
		controller->ki_d = one_minus_exp_neg(r*DT/ld);
		controller->ki_q = one_minus_exp_neg(r*DT/lq);
	}
	else{
		controller->k_d = K_SCALE*bw;
		controller->k_q = K_SCALE*bw;
		controller->ki_d = KI_D;
		controller->ki_q = KI_Q;
	}
	controller->ki_fw = .1f*controller->ki_d;	// 弱磁积分沿用 d 轴零点的 1/10
}

void init_controller_params(ControllerStruct *controller){

	/* 电流环 PI 增益由标定的 R/L 推导（未标定时退回常量），见 current_loop_retune() */
	current_loop_retune(controller);
    controller->flux_linkage = KT/(1.5f*PPAIRS);
    /* Fixed on-chip PGA configuration and 5 mOhm shunts; I_MAX does not
     * change the hardware gain or the current conversion coefficient. */
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
    controller->i_bus = 0;
    controller->i_bus_filt = 0;
    controller->p_elec = 0;
    controller->mech_power = 0;
    controller->uv_cycles = 0;
    /* 电流上限由 commutate() 每拍重算（温度降流 + 转矩上限）。这里先给一个有效值，
     * 否则进电机模式的第一拍 torque_control() 会用上一次留下的（可能是 0）上限 */
    controller->i_max = I_MAX;
    controller->temp_i_lim = I_MAX;
    /* 每次进电机模式都按当前标定的 R/L 与 config.i_bw 重算电流环增益，这样
     * 标定完成或改了 i_bw 都不必等重启。纯浮点运算，只在状态切换时执行一次 */
    current_loop_retune(controller);
    }

/* ---- NTC 温度 --------------------------------------------------------------
 *
 * 照搬 ODrive Board/v3 的 ThermistorCurrentLimiter：把 ADC 原始值归一化到
 * [0,1]（x = count/4096，即相对参考电压的比例），用四阶 Horner 多项式换算成 °C，
 * 再过两级一阶低通（每级 tau = 0.1 s，等效二阶、约 0.26 s 的时间常数）。
 *
 * 系数就是 ODrive 板子的那两组（MOS 板载 NTC / 外接电机 NTC）。CBL89 的分压网络
 * 如果与 ODrive 不同，读数会偏——所以在台架上用室温核对一次；偏差大就改这两组
 * 系数（换算结构不变）。
 *
 * 上下文：本硬件 MOS_TEMP = ADC2_CH2 → adc_measurements_[0]，
 * MOT_TEMP = ADC3_CH6 → adc_measurements_[2]（见 main.c 的 start_general_purpose_adc）。
 * 这两个 DMA 目标每个 PWM 周期刷新，主循环读到的永远是最新值。
 * 只用于监控和（默认关闭的）温度保护：不参与电流环，也不在这里 printf。 */
static const float fet_thermistor_poly[] = { 296.7f, -385.7f, 257.1f, -26.81f };
static const float mot_thermistor_poly[] = {-296.7f, 504.3f, -375.8f, 141.3f };

#define THERM_LPF_TAU	0.1f

/* 系数按 ODrive 的顺序给出：c[0] 是最高次幂（与 horner_poly_eval / numpy.polyfit
 * 一致），所以必须从 c[0] 开始乘。把顺序反过来（拿 c[0] 当常数项）不会报错，
 * 但读数会整体错乱：室温下 MOS 会读到 200 °C 以上、电机读到 -100 °C 以下 */
static float thermistor_celsius(const float *c, unsigned n, float x){
	float v = 0.0f;
	for(unsigned i = 0; i < n; i++){v = v*x + c[i];}
	return v;
}

static float thermistor_step(float raw, float *stage1, float *stage2, float k){
	*stage1 += k*(raw - *stage1);
	*stage2 += k*(*stage1 - *stage2);
	return *stage2;
}

void update_thermistors(ControllerStruct *controller){
	/* 两级 LPF 的状态跨调用保留；第一次调用时用当时的原始值播种，避免读数从上电
	 * 的 0 °C 慢慢爬上来（否则头几百毫秒会报一个假的低温） */
	static float fet_s1, fet_s2, mot_s1, mot_s2;
	static uint32_t last_count;
	static uint8_t seeded;

	/* dt 从控制周期计数算出：主循环没有固定周期（打印/擦 flash 都会拉长一拍），
	 * 用固定 dt 会让滤波时间常数随固件行为漂移。夹在 [0.05 ms, 500 ms] 之间，
	 * 防止计数回绕或长时间停在这里时把滤波做废 */
	uint32_t now = (uint32_t)controller->loop_count;
	float dt = (float)(now - last_count) * DT;
	last_count = now;
	if(dt < 0.00005f){dt = 0.00005f;}
	if(dt > 0.5f){dt = 0.5f;}

	float k = dt/THERM_LPF_TAU;
	if(k > 1.0f){k = 1.0f;}

	float fet_raw = thermistor_celsius(fet_thermistor_poly,
	                                   sizeof(fet_thermistor_poly)/sizeof(fet_thermistor_poly[0]),
	                                   (float)adc_measurements_[0]/ADC_FULL_SCALE);
	float mot_raw = thermistor_celsius(mot_thermistor_poly,
	                                   sizeof(mot_thermistor_poly)/sizeof(mot_thermistor_poly[0]),
	                                   (float)adc_measurements_[2]/ADC_FULL_SCALE);

	if(!seeded){
		fet_s1 = fet_s2 = fet_raw;
		mot_s1 = mot_s2 = mot_raw;
		seeded = 1;
	}

	float fet = thermistor_step(fet_raw, &fet_s1, &fet_s2, k);
	float mot = thermistor_step(mot_raw, &mot_s1, &mot_s2, k);
	/* 换算出来的 NaN（系数/通道异常）不能进比较，也不能进遥测。滤波状态也要
	 * 一起清掉：NaN 一旦进了状态就再也出不来，读数会永远停在 0（ODrive 的
	 * ThermistorCurrentLimiter::update() 也是这么处理的） */
	if(!isfinite(fet)){fet = 0.0f; fet_s1 = fet_s2 = 0.0f;}
	if(!isfinite(mot)){mot = 0.0f; mot_s1 = mot_s2 = 0.0f;}

	controller->mos_temp = fet;
	controller->mot_temp = mot;
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

       /* ---- 母线电流 / 机械功率（估算，见下）----
        * 硬件没有直流分流器：用上一拍的 dq 电压和的功率反推。
        *   P_elec = 1.5 * V_PER_CMD * (v_d*i_d + v_q*i_q)   [W]
        *   i_bus  = P_elec / v_bus                           [A]
        * 1.5 是幅值不变 dq 变换的功率系数。v_d/v_q 是上一拍的输出（指令在上一拍
        * 已经生效），所以误差在一个控制周期内，对监控和软限幅足够。
        * 这不是精密测量，不能拿它当保护阈值 —— 保护仍然靠相电流与母线电压。 */
       {
           float vbus = controller->v_bus_filt;
           float p_elec = 1.5f*V_PER_CMD*(controller->v_d*controller->i_d
                                         + controller->v_q*controller->i_q);
           float i_bus = (vbus > 1.0f) ? (p_elec/vbus) : 0.0f;
           if(!isfinite(i_bus)){i_bus = 0.0f;}
           if(!isfinite(p_elec)){p_elec = 0.0f;}
           controller->p_elec = p_elec;
           controller->i_bus = i_bus;
           controller->i_bus_filt = (1.0f-VBUS_FILT_ALPHA)*controller->i_bus_filt
                                  + VBUS_FILT_ALPHA*i_bus;
           /* 机械功率：输出端转矩 × 输出轴角速度（>0 输出机械功，<0 回馈） */
           controller->mech_power = controller->i_q_filt*KT*GR*controller->dtheta_mech;
       }

       /* ---- 电流上限：I_MAX 再叠加温度降流与转矩上限 ----
        * 温度保护关闭时完全不影响 I_MAX（默认就是关）。 */
       {
           float i_lim = I_MAX;

           if(FET_TEMP_EN && (FET_TEMP_HI > FET_TEMP_LO)){
               float frac = (FET_TEMP_HI - controller->mos_temp)/(FET_TEMP_HI - FET_TEMP_LO);
               if(frac < 0.0f){frac = 0.0f;}
               if(frac > 1.0f){frac = 1.0f;}
               i_lim *= frac;
           }
           if(MOT_TEMP_EN && (MOT_TEMP_HI > MOT_TEMP_LO)){
               float frac = (MOT_TEMP_HI - controller->mot_temp)/(MOT_TEMP_HI - MOT_TEMP_LO);
               if(frac < 0.0f){frac = 0.0f;}
               if(frac > 1.0f){frac = 1.0f;}
               i_lim *= frac;
           }
           /* config.torque_lim 是**输出端** N·m，与 command.t_ff / telemetry.torque
            * 同量纲；换算到电机端安培是 /(KT*GR) */
           if((TORQUE_LIM > 0.0f) && (KT*GR > 0.0f)){
               float t_lim = TORQUE_LIM/(KT*GR);
               if(t_lim < i_lim){i_lim = t_lim;}
           }
           controller->i_max = i_lim;
           controller->temp_i_lim = i_lim;
       }

       /* 母线电流软限幅：只按比例回压，不锁故障（估算值不足以判断过流）。
        * 只压不抬，所以不会因为估算噪声把指令放大 */
       if((I_DC_MAX > 0.0f) && (controller->i_bus_filt > I_DC_MAX)){
           float scale = I_DC_MAX/controller->i_bus_filt;
           if(scale < 0.0f){scale = 0.0f;}
           if(scale > 1.0f){scale = 1.0f;}
           controller->i_q_des *= scale;
           controller->i_d_des *= scale;
       }

       limit_norm(&controller->i_d_des, &controller->i_q_des, controller->i_max);	// 2.3 us

       /// PI Controller ///
       float i_d_error = controller->i_d_des - controller->i_d;
       float i_q_error = controller->i_q_des - controller->i_q;

       /* ---- dq 解耦 + 反电动势前馈（默认关闭，保留公式供以后启用）----
        * 上游 mini-cheetah 原本是启用的，commit f982604 把系数改成 0，此后一直注释着。
        * 关闭是**有意的**：电流环 PI 由标定的 R/L 极点对消整定（见 current_loop_retune()），
        * 带宽 300~800 Hz，交叉耦合项和反电动势都被积分项吸收，跟踪滞后可以接受；
        * 而前馈依赖 L_D/L_Q/flux_linkage 的精度，估偏了等于往指令里塞一份常值误差。
        *
        * 单位：本文件用的是**幅值不变**的 Park 变换（i_q = 相电流峰值，见 dq0()），
        * svm() 又把 dq 电压按 v_bus*(DTC_MAX-DTC_MIN)*sqrt(1/3) 归一化，所以 v_d/v_q
        * 的 1 个单位 = V_PER_CMD 伏：物理伏特必须 /V_PER_CMD 才是指令值。
        * 这里**没有 SQRT3**——上游注释里那个 SQRT3 是早期 svm()/dq0 归一化不同时留下的，
        * 照抄会偏 √3 倍。用 i_*_des 而不是测量的 i_d/i_q：要减的是参考路径上的耦合，
        * 测量值还会把编码器差分速度和 ADC 噪声灌进电压指令。
        *
        * 启用前（高速/弱磁区，或把 config.i_bw 提到 1~2 kHz 以上才真正值得）：
        *  - 必须在下面的限幅（v_max 钳位、limit_norm）之前加，且不能进 d_int/q_int 累加器；
        *  - 未标定时 R_PHASE=0 -> L_D/L_Q/flux_linkage 都是 0，前馈自动退化为 0，天然安全；
        *  - 打开后台架复测 csv_csp_test.py sweep 与电流环阶跃：PI 与前馈的分工变了，
        *    d_int/q_int 的 v_max 抗饱和钳位可能要重新看。 */
       float v_d_ff = 0.0f;	// -controller->dtheta_elec*L_Q*controller->i_q_des/V_PER_CMD;
       float v_q_ff = 0.0f;	//  controller->dtheta_elec*(L_D*controller->i_d_des + controller->flux_linkage)/V_PER_CMD;

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
	/* TORQUE_RAMP：把 t_ff 按 config.torque_ramp_rate（N·m/s）限斜率，再走原来的
	 * 一阶滤波。两者目的不同：斜坡限制的是“变化有多快”，滤波抑制的是尖峰。
	 * 只在 input_mode == 6 时生效；其他模式下 t_ff_slew 直接跟随，行为与改动前一致。 */
	float t_ff = controller->t_ff;
	if((INPUT_MODE == INPUT_MODE_TORQUE_RAMP) && (TORQUE_RAMP_RATE > 0.0f)){
		controller->t_ff_slew = ml_slew(controller->t_ff_slew, t_ff, TORQUE_RAMP_RATE, DT);
		t_ff = controller->t_ff_slew;
	}
	else{
		controller->t_ff_slew = t_ff;
	}

	controller->t_ff_filt = 0.9f*controller->t_ff_filt + 0.1f*t_ff;
    float torque_des = controller->kp*(controller->p_des - controller->theta_mech) + controller->t_ff_filt + controller->kd*(controller->v_des - controller->dtheta_mech);
    controller->i_q_des = fast_fmaxf(fast_fminf(torque_des/(KT*GR), controller->i_max), -controller->i_max);

    /* config.enable_torque_vel_lim（默认关）：阻抗/力矩/电流模式下也限制速度。
     * 与 ODrive 的 limitVel() 同构，只是直接在安培域做 —— VEL_GAIN 就是
     * A/(rad/s)，所以 (v_lim - v)·vel_gain 的量纲正好是 A。
     * 高速时把转矩往“刹车”方向压：超过上限就没有往超速方向推的余量。 */
    if(TORQUE_VEL_LIM_EN && (V_MAX > 0.0f) && (VEL_GAIN > 0.0f)){
        float v = controller->dtheta_mech;
        float i_hi = (V_MAX - v)*VEL_GAIN;
        float i_lo = (-V_MAX - v)*VEL_GAIN;
        if(controller->i_q_des > i_hi){controller->i_q_des = i_hi;}
        if(controller->i_q_des < i_lo){controller->i_q_des = i_lo;}
    }

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

	/* 母线过压：阈值来自 config.vbus_ov_trip（默认 67 V，原编译期常量）。
	 * 任何状态都检查 —— 门极关闭时母线也可能被外部推高 */
	if(controller->v_bus > VBUS_OV_TRIP){
		if(!(controller->fault & FAULT_OV)){controller->fault_vbus = controller->v_bus;}
		controller->fault |= FAULT_OV;
	}
	/* 母线欠压：只在门极使能时判定。USB 单独供电（没有母线电压）时 v_bus 就是
	 * 0，若在任何状态都检查，一上电就会锁存故障、把用户挡在菜单外。
	 * 加速/制动时母线会瞬间跌落，所以用 ~10 ms 去抖（40 kHz ⇒ 400 拍） */
	if((VBUS_UV_TRIP > 0.0f) && controller->gate_drive_enabled){
		if(controller->v_bus < VBUS_UV_TRIP){
			if(controller->uv_cycles < 0xFFFFu){controller->uv_cycles++;}
			if(controller->uv_cycles >= 400u){
				if(!(controller->fault & FAULT_UV)){controller->fault_vbus = controller->v_bus;}
				controller->fault |= FAULT_UV;
			}
		}
		else{controller->uv_cycles = 0;}
	}
	else{controller->uv_cycles = 0;}
	/* 过温：mos_temp/mot_temp 由主循环（update_thermistors）更新。与 ODrive 一致：
	 * 达到 hi 时电流上限已经降到 0，再高出 5 °C 才锁故障，避免在阈值上抖动 */
	if(FET_TEMP_EN && (controller->mos_temp >= FET_TEMP_HI + 5.0f)){
		controller->fault |= FAULT_FET_OT;
	}
	if(MOT_TEMP_EN && (controller->mot_temp >= MOT_TEMP_HI + 5.0f)){
		controller->fault |= FAULT_MOT_OT;
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

	/* 外环状态一并复位。CAN 超时、进 MOTOR_MODE、故障都必须回到零转矩，
	 * 不能把上一个模式的 i_q_ref/积分当成“保持”延续下去；ctrl_mode 回阻抗律，
	 * 下一拍 motion_control() 会检测到模式变化并按新状态重新播种。 */
	controller->ctrl_mode = CTRL_IMPEDANCE;
	controller->i_q_ref = 0;
	controller->i_cmd_max = I_MAX;
	controller->v_limit = V_MAX;
	controller->v_target = 0;
	controller->v_setpoint = 0;
	controller->vel_integ = 0;
	controller->pos_err = 0;
	controller->vel_err = 0;
	controller->pos_ff = 0;
	controller->pos_ff_last = 0;
	controller->pos_ff_cycle = 0;
	controller->pos_fault_t = 0;
	/* 输入整形状态一并丢弃：下一次进外环时从当前状态重新播种（见 motion_seed）。
	 * 0xFF 是哨兵值，保证下一次 motion_control() 一定重新播种一次 */
	controller->input_mode_prev = 0xFF;
	controller->pos_trap.planned = 0;
	controller->pos_trap.t = 0;
	controller->pos_filter.seeded = 0;
	controller->t_ff_slew = 0;
	/* 位置软限位的“界外冻结”状态同样丢弃：下次切到 CSP 时按当时的实测位置
	 * 重新判断（见 motion_seed） */
	controller->pos_hold = 0;
	controller->pos_hold_des = 0;

	/* 正在跑的惯量标定必须中止：离开 MOTOR_MODE 后 motion_control() 不再被调用，
	 * 状态会停在半路，下次进电机模式时从半个台阶"续跑"，测出来的 J 是错的 */
	if (controller->inertia_state != INERTIA_IDLE) {
		controller->inertia_state = INERTIA_IDLE;
		controller->inertia_status = 3;
	}
	controller->inertia_req = 0;
	controller->inertia_cycles = 0;
	controller->inertia_i = 0;
	controller->inertia_v_peak = 0;

	/* 齿槽补偿标定同理：中途退出电机模式后不能接着上次的表点继续扫
	 * （齿槽标定会清空自己的结果，见 cogging_abort） */
	cogging_abort(controller);
	controller->cog_ff = 0;
}

/* ---- 位置指令的允许区间（通信层写 p_des 用）----------------------------
 * 判据在纯函数 ml_pos_cmd_range()（motion_law.h，host 可测），这里只把当前
 * 实测位置换算成输出轴角度。
 *
 * 界内：钳 [P_MIN, P_MAX]；界外：放宽到当前实测位置这一侧 —— 限位 0..10、
 * 当前位置 15 时，目标 12 是合法的"往界内"指令，不能被钳成 10；而比 15 更外
 * 的目标要挡掉。控制层每拍还会用 ml_pos_cmd_clamp() 再算一次，因为位置在动，
 * 写入口那一刻的界可能已经变了。 */
static void pos_cmd_range(float *lo, float *hi)
{
	float gr = GR;
	float p = comm_encoder.angle_multiturn / ((gr > 0.0f) ? gr : 1.0f);

	ml_pos_cmd_range(p, P_MIN, P_MAX, lo, hi);
}

float pos_cmd_lo(void)
{
	float lo, hi;
	pos_cmd_range(&lo, &hi);
	return lo;
}

float pos_cmd_hi(void)
{
	float lo, hi;
	pos_cmd_range(&lo, &hi);
	return hi;
}

float pos_cmd_limit(float p)
{
	float lo, hi;
	pos_cmd_range(&lo, &hi);
	if (p < lo) { return lo; }
	if (p > hi) { return hi; }
	return p;
}
