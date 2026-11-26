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
#include "mtpa_law.h"
#include "phase_current_law.h"
#include "dtc_comp.h"
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

	/* 下一拍 analog_sample() 要不要扫低边窗口：窗口 = AR*dtc（见
	 * phase_current_law.h），判据 dtc < I_SAMPLE_MIN_LOW_FRAC 与
	 * pcl_worst_phase() 的 tick 比较等价，所以在这里按刚写下的三个占空比一次
	 * 算清，中断里就省掉 4 次外设读 + 4 次比较。
	 * 放在 set_dtc() 里而不是 commutate() 里：开环（标定）和 zero_current() 也
	 * 直接调这个函数，写在这些路径上的占空比同样可能把某一相压到底部。 */
	controller->pcl_check_needed =
			(fminf3(controller->dtc_u, controller->dtc_v, controller->dtc_w)
			 < I_SAMPLE_MIN_LOW_FRAC);
}

void analog_sample (ControllerStruct *controller){
	/* The PWM peak update triggers all three injected ADCs and this ISR
	 * once per 40 kHz cycle, in the low-side conduction window.  ADC2 also
	 * runs the regular VBUS sequence, so wait for each independent injected
	 * sequence to finish.  If a trigger or converter fails, the watchdog
	 * resets the chip. */
	ISR_TIMING_SPAN_BEGIN(t_adc_wait);
	while(!adc_jendc_set(ADC1)){;}
	while(!adc_jendc_set(ADC2)){;}
	while(!adc_jendc_set(ADC3)){;}
	ISR_TIMING_SPAN_END(ISR_EXT_ADC_WAIT, t_adc_wait);

	adc_jendc_clear(ADC1);
	adc_jendc_clear(ADC2);
	adc_jendc_clear(ADC3);

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

	/* ---- 低边采样窗口太短的那一相：用另外两相按 KCL 重建（见 phase_current_law.h）----
	 * 采样点固定在 PWM 峰值，低边导通窗口 = AR - CCR（PWM1 低边是 CHx 的互补，
	 * 只在 CNT > CCR 时导通）；本机 INVERT_DTC = 1 时 CCR = AR*(1-dtc)，所以窗口
	 * ∝ dtc ——**duty 最小的那一相**（min-max 注入把它钉在母线正端，低边分流几乎
	 * 不导通）读数最不可信，而三相瞬时电流之和恒为零，所以它能由另外两相补回来。
	 * 三个阶段：
	 *   1) 读本拍生效的比较值（上一拍 set_dtc() 写入、本次 UEV 刚装入，本拍的
	 *      set_dtc() 还没执行）；
	 *   2) 三相窗口都够就什么都不做（唯一的常见路径：3 次比较 + 一次分支）；
	 *   3) 否则重建窗口最小的那一相（= CCR 最大的那一相）。
	 * 这里在 PHASE_ORDER 交换**之前**做，用的是物理对应（i_a ↔ 桥臂 1/CCDAT1、
	 * i_b ↔ 桥臂 2、i_c ↔ 桥臂 3），所以不用判断 PHASE_ORDER；阈值 i_min_low_ticks
	 * 在 init_controller_params() 里按 AR 换算过一次。 */
	int i_recon = PCL_NONE;
	if(controller->gate_drive_enabled && controller->pcl_check_needed){
		const uint32_t ar = TIM_PWM->AR;
		i_recon = pcl_worst_phase(pcl_low_side_ticks(ar, TIM_PWM->CCDAT1),
		                          pcl_low_side_ticks(ar, TIM_PWM->CCDAT2),
		                          pcl_low_side_ticks(ar, TIM_PWM->CCDAT3),
		                          controller->i_min_low_ticks);
		if(i_recon != PCL_NONE){
			pcl_reconstruct(&controller->i_a, &controller->i_b, &controller->i_c, i_recon);
			if(controller->i_recon_count != 0xFFFFFFFFu){ controller->i_recon_count++; }
		}
	}
	controller->i_recon = (uint8_t)i_recon;	/* PCL_NONE(-1) 存成 0xFF */

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
    /* MTPA 的等效转矩常数先按 Id=0 起步；commutate() 每拍刷新。
     * 整定为 KT 保证 MTPA 关闭时下游（转矩限幅/遥测）逐位不变 */
    controller->kt_eff = KT;
    controller->mtpa_id = 0.0f;
    controller->mtpa_iq_cmd = 0.0f;
    controller->i_q_peak = I_MAX;
    /* 低边采样窗口阈值（analog_sample / phase_current_law.h）：AR 运行期不变，
     * 所以在这里换算一次，40 kHz 中断里只剩一次比较。必须在 MX_TIM1_Init() 之后 */
    controller->i_min_low_ticks = (uint32_t)(I_SAMPLE_MIN_LOW_FRAC*(float)TIM_PWM->AR + 0.5f);
    /* 保守起见先扫一次；commutate() 每拍按实际占空比刷新（见那里的注释） */
    controller->pcl_check_needed = 1;
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
    controller->pcl_check_needed = 1;
    /* MTPA 状态同样清零：进电机模式的第一拍 torque_control() 只用 KT 换算，
     * 别让上一次留下的 Id/kt_eff 参与转矩限幅 */
    controller->kt_eff = KT;
    controller->mtpa_id = 0.0f;
    controller->mtpa_iq_cmd = 0.0f;
    controller->i_q_peak = I_MAX;
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

/* ---- MTPA（每安培最大转矩）：见 Core/Inc/mtpa_law.h 的推导与不变式 ----
 *
 * 要不要真的去算：默认关；SPM 电机（凸极太弱）、未标定、以及**惯量/齿槽标定期间**
 * 都不参与 —— 那两套标定都用 KT 把电流换算成转矩（ml_inertia_from_slopes /
 * ml_tune_gains），MTPA 会改变这个隐含增益，测出来的 J 和齿槽表会带系统偏差。
 * 这些门都在 `mtpa_service()`（主循环）里算成 `mtpa_ready`，中断只看它 + 两个
 * 标定状态；关闭时 mtpa_apply()/field_weaken() 都走原来逐位不变的路径。
 *
 * 为什么是"主循环建表、中断查表"：直接在 40 kHz 中断里解 MTPA 方程（Newton +
 * 有理化）在台架上实测让 seg_fsm_max 从 2551 涨到 3366（**+815 拍 / +32%**）——
 * 本仓库的中断已经跑到 ~96% 的周期预算，而 flash 等待 + ICACHE 未命中会把每条
 * 指令放大到约 2.7 拍，300 条指令就变成 800 拍。所以求解搬到主循环：
 * 参数（KT/PPAIRS/LD/LQ/I_MAX/mtpa_id_frac）变化时重建一张表（约 12 µs），
 * 中断里只剩一次乘法、一次取整和两次线性插值（约 30 条指令）。
 * 表范围 [0, I_MAX·g]（g = 电流圆上的转矩增益，见 mtpa_law.h）：上界就是
 * torque_control() 的钳位上限，顶端几格落在饱和分支（圆上最优点，恒定）。
 * 点数由 host 测试 test_table_resolution 守着：49 点时强凸极（ξ=3）的插值误差
 * 也有 0.06% 满量程转矩 / 0.15% Imax 的 Id，本机（ξ=0.14）是 0.0004% 量级。 */

#define MTPA_TAB_N	49

typedef struct{
	float inv_da;					// (MTPA_TAB_N-1)/I_MAX，把 |iq_cmd| 换算成表下标
	float id[MTPA_TAB_N];			// 有符号（凸极电机为负）
	float iq[MTPA_TAB_N];			// |Iq|（正）
} MtpaTable;

/* 常量缓存：λ / ΔL 只在标定或改参数时变，中断里直接读快照 */
typedef struct{
	float kt, ppairs, ld, lq;		// 缓存键：变了就重算
	float lambda, dL;
	int ok;							// 0 = 参数不可用（未标定 / 非法）
} MtpaConsts;
static MtpaConsts mtpa_c;

static MtpaTable mtpa_tab[2];			// 双缓冲：中断只读 mtpa_tab_gen 指定的那张
static volatile uint8_t mtpa_tab_gen;
static volatile uint8_t mtpa_ready;		// 主循环维护：开关/参数/凸极都允许
/* **实际**电流上限处的转矩增益 g = i_q_peak/i_max（≥1）。由 commutate() 每拍更新，
 * 因为 i_max 会被温度降流/转矩限幅改动，而表的 Id(Iq) 轨迹与电流上限无关 ——
 * 见 mtpa_iq_peak() 的注释。MTPA 未就绪时恒为 1 */
static float mtpa_scale_f = 1.0f;
static float mtpa_lambda_f, mtpa_dL_f;	// mtpa_ready 为 1 时中断直接用
/* mtpa_iq_peak() 的缓存：实际上限变化不大就复用上次的峰值（见该函数注释） */
static float mtpa_peak_in = -1.0f, mtpa_peak_out;

static const MtpaConsts *mtpa_consts(void)
{
	if ((mtpa_c.kt != KT) || (mtpa_c.ppairs != PPAIRS) || (mtpa_c.ld != L_D) || (mtpa_c.lq != L_Q)) {
		mtpa_c.kt = KT;
		mtpa_c.ppairs = PPAIRS;
		mtpa_c.ld = L_D;
		mtpa_c.lq = L_Q;
		mtpa_c.ok = (KT > 0.0f) && (PPAIRS > 0.0f) && (L_D > 0.0f) && (L_Q > 0.0f) && (L_D != L_Q);
		if (mtpa_c.ok) {
			mtpa_c.lambda = KT/(1.5f*PPAIRS);
			mtpa_c.dL = L_D - L_Q;
		} else {
			mtpa_c.lambda = 0.0f;
			mtpa_c.dL = 0.0f;
		}
	}
	return &mtpa_c;
}

/* 中断侧的门：主循环算好的 mtpa_ready + 两个标定状态（由中断自己改，必须现场看） */
static inline int mtpa_on(const ControllerStruct *controller)
{
	return mtpa_ready
	    && (controller->inertia_state == INERTIA_IDLE)
	    && (cogging_cal.state == COG_CAL_IDLE)
	    && !controller->cogging_req;
}

/* 主循环：重建表 + 刷新 mtpa_ready。每 ~1 ms 调一次，只有参数变化时才真的干活 */
void mtpa_service(void)
{
	static float key_kt, key_pp, key_ld, key_lq, key_imax, key_frac;
	static int key_init;
	const MtpaConsts *c = mtpa_consts();
	int ok;

	if (!c->ok || !(I_MAX > 0.0f) || !(MTPA_ID_FRAC > 0.0f)) {
		mtpa_ready = 0;
		return;
	}
	ok = (MTPA_EN != 0)
	     && (ml_mtpa_saliency(I_MAX, c->lambda, c->dL) >= ML_MTPA_SALIENCY_MIN);

	if (!(key_init
	      && (key_kt == KT) && (key_pp == PPAIRS) && (key_ld == L_D) && (key_lq == L_Q)
	      && (key_imax == I_MAX) && (key_frac == MTPA_ID_FRAC))) {
		uint8_t g = (uint8_t)(mtpa_tab_gen ^ 1u);
		MtpaTable *t = &mtpa_tab[g];
		/* 表的上界 = I_MAX 这个电流圆上还能交付的最大 q 指令（SPM 等价）= I_MAX·g。
		 * 顶端几格落入 ml_mtpa_split() 的饱和分支（圆上最优点，恒定）。
		 * 表只锚在**配置**的 I_MAX 上；实际电流上限（温度降流/转矩限幅）更小时，
		 * 由 mtpa_iq_peak() 按小圆重新收口，不需要重建表。
		 * g 钳在 [1, 8)：增益 8 对应 ξ≈16，已经不是"凸极"而是参数错误了 */
		float gs = ml_mtpa_gain_at_limit(c->lambda, c->dL, I_MAX, MTPA_ID_FRAC);
		float a_max;
		int k;

		if (!(gs >= 1.0f) || !(gs < 8.0f)) { gs = 1.0f; }
		a_max = I_MAX*gs;

		key_kt = KT; key_pp = PPAIRS; key_ld = L_D; key_lq = L_Q;
		key_imax = I_MAX; key_frac = MTPA_ID_FRAC;
		key_init = 1;

		t->inv_da = (float)(MTPA_TAB_N - 1)/a_max;
		for (k = 0; k < MTPA_TAB_N; k++) {
			float a = (float)k*a_max/(float)(MTPA_TAB_N - 1);
			ml_mtpa_t r = ml_mtpa_split(a, c->lambda, c->dL, I_MAX, MTPA_ID_FRAC);
			t->id[k] = r.id;
			t->iq[k] = fabsf(r.iq);
		}
		mtpa_tab_gen = g;		/* 发布：中断下次就读到新表 */
	}

	/* 快照与开关**最后**发布，保证中断看到 mtpa_ready=1 时表和常量都已就绪 */
	mtpa_lambda_f = c->lambda;
	mtpa_dL_f = c->dL;
	mtpa_ready = (uint8_t)ok;
	mtpa_peak_in = -1.0f;		/* 参数/开关变了：让 i_q_peak 的缓存失效 */
}

/* 把**实际**电流上限换算成 SPM 等价 q 电流峰值（≥ i_max）。
 *
 * 为什么必须按实际（而不是配置的 I_MAX）算：表的 Id(Iq) 轨迹只由 λ/ΔL 决定、与电流
 * 上限无关，但"饱和点"在圆上。温度降流把 i_max 压到 I_MAX 的 30% 时，若仍按大圆的
 * 比例取 Id，小圆上就会留下一个过深的 Id、Iq 被 √(i_max²−Id²) 卡住 —— 台架仿真：
 * ξ=3 的电机降流到 15% 时 MTPA 只有 SPM 转矩的 72%（比不开还差）。按实际圆收口后
 * 同样工况是 108%。 */
static float mtpa_iq_peak(float i_max)
{
	if (!mtpa_ready) {
		mtpa_peak_in = -1.0f;
		mtpa_scale_f = 1.0f;
		return i_max;
	}
	/* 只在**实际**上限变化超过 2% 时重算（两次开方 + 三次除法，稳态下这块完全不执行；
	 * 温度降流是 0.1 s 低通后的慢变量）。缓存值偏大只会让指令被 limit_norm 稍微收一下，
	 * 方向是安全的。g 也在这里算好，省掉中断里每拍一次除法 */
	if ((mtpa_peak_in <= 0.0f) || (fabsf(i_max - mtpa_peak_in) > 0.02f*mtpa_peak_in)) {
		float p;
		if (!(i_max > 0.0f)) { mtpa_peak_in = -1.0f; mtpa_scale_f = 1.0f; return i_max; }
		p = ml_mtpa_iq_cmd_at_limit(mtpa_lambda_f, mtpa_dL_f, i_max, MTPA_ID_FRAC);
		mtpa_peak_out = (p > i_max) ? p : i_max;
		mtpa_peak_in = i_max;
		mtpa_scale_f = mtpa_peak_out/i_max;
	}
	return mtpa_peak_out;
}

/* 电流上限的换算系数 g（≥1）：SPM 等价 i_q 域的上限 = 物理电流上限 × g。
 * 按**实际**电流上限算（commutate() 每拍更新），MTPA 未激活时恒为 1（逐位不变）。 */
float mtpa_iq_scale(void)
{
	return mtpa_scale_f;
}

void mtpa_apply(ControllerStruct *controller)
{
	controller->mtpa_id = 0.0f;
	controller->mtpa_iq_cmd = 0.0f;
	if (!mtpa_on(controller)) { return; }

	{
		const MtpaTable *t = &mtpa_tab[mtpa_tab_gen];
		float cmd = controller->i_q_des;
		float lim = controller->i_q_peak;		/* 上一拍：实际电流上限的 SPM 等价峰值 */
		float x, f, id, iq;
		int i, i1;

		if (!isfinite(cmd)) {
			/* 坏指令不进控制环：连 Id 一起清零，别留上一拍的 i_d_des */
			controller->i_d_des = 0.0f;
			controller->i_q_des = 0.0f;
			return;
		}
		/* 按**实际**电流圆收口（温度降流/转矩限幅后 i_max 可能远小于 I_MAX）。
		 * 表锚在 I_MAX，但轨迹与电流上限无关，钳到小圆的 SPM 等价峰值后查表
		 * 得到的仍正是小圆上的 MTPA 最优点（圆上最优点恒在轨迹上） */
		if (cmd > lim) { cmd = lim; }
		if (cmd < -lim) { cmd = -lim; }

		x = fabsf(cmd)*t->inv_da;
		if (!(x <= (float)(MTPA_TAB_N - 1))) { x = (float)(MTPA_TAB_N - 1); }
		if (x < 0.0f) { x = 0.0f; }
		i = (int)x;
		f = x - (float)i;
		i1 = (i < (MTPA_TAB_N - 1)) ? (i + 1) : (MTPA_TAB_N - 1);
		id = t->id[i] + f*(t->id[i1] - t->id[i]);
		iq = t->iq[i] + f*(t->iq[i1] - t->iq[i]);

		controller->mtpa_iq_cmd = cmd;		/* 转矩当量，field_weaken() 要用 */
		controller->mtpa_id = id;			/* Id 与转矩方向无关（磁阻 ∝ Id·Iq） */
		controller->i_d_des = id;
		controller->i_q_des = (cmd < 0.0f) ? -iq : iq;
	}
}

/* telemetry.mtpa_gain：电流极限处 MTPA 相对 SPM 的转矩增益（主循环调用，1 = 没用） */
float mtpa_gain(void)
{
	const MtpaConsts *c = mtpa_consts();
	if (!c->ok) { return 1.0f; }
	return ml_mtpa_gain_at_limit(c->lambda, c->dL, I_MAX, MTPA_ID_FRAC);
}

void field_weaken(ControllerStruct *controller)
{
       /// Field Weakening ///

       controller->fw_int += controller->ki_fw*(controller->v_max - 1.0f - controller->v_ref);
       controller->fw_int = fast_fmaxf(fast_fminf(controller->fw_int, 0.0f), -fast_fminf(I_FW_MAX, controller->i_max));

       if (mtpa_on(controller)) {
           /* ---- MTPA + 弱磁：两个都是 ≤0 的 Id，取更深的那个，**不叠加** ----
            * Id 变了之后按"转矩不变"重算 Iq（Iq = iq_cmd·λ/(λ+ΔL·Id)），再按电流圆收口。
            * 见 mtpa_law.h 的文件头，以及 ODrive 的 vesc_field_weakening.cpp 末尾。
            * λ/ΔL 用主循环发布的快照，中断里只做一次除法（λ/kt）与一次开方（qmax，
            * 且只在实际需要收口时才开方） */
           const float lambda = mtpa_lambda_f;
           const float id = fast_fminf(controller->i_d_des, controller->fw_int);

           controller->i_d_des = id;
           if (id == 0.0f) {
               /* 没弱磁、MTPA 也没让 Id 离 0：与旧快路径等价，省掉除法/开方 */
               controller->i_q_des = fast_fmaxf(fast_fminf(controller->mtpa_iq_cmd, controller->i_max), -controller->i_max);
               return;
           }
           {
               float kt = lambda + mtpa_dL_f*id;
               float iq, q2, qmax;
               if (!(kt > 0.5f*lambda)) {
                   /* 参数异常（Ld/Lq 填错、或 Id 太深）：整对退回 SPM，别留下
                    * 一个 kt 与 (Id,Iq) 不自洽的工作点 */
                   controller->i_d_des = 0.0f;
                   controller->i_q_des = fast_fmaxf(fast_fminf(controller->mtpa_iq_cmd, controller->i_max), -controller->i_max);
                   return;
               }
               iq = controller->mtpa_iq_cmd*(lambda/kt);
               q2 = controller->i_max*controller->i_max - id*id;
               /* 先比平方再开方：正常工况（|Iq| 远小于圆上余量）直接跳过 sqrtf */
               if ((q2 <= 0.0f) || (iq*iq > q2)) {
                   qmax = (q2 > 0.0f) ? sqrtf(q2) : 0.0f;
                   controller->i_q_des = fast_fmaxf(fast_fminf(iq, qmax), -qmax);
               } else {
                   controller->i_q_des = iq;
               }
           }
           return;
       }

       /* Reduce the q current reference magnitude as field weakening increases, saturating at zero */
       if(controller->i_q_des > 0.0f){controller->i_q_des = fast_fmaxf(controller->i_q_des + controller->fw_int, 0.0f);}
       else{controller->i_q_des = fast_fminf(controller->i_q_des - controller->fw_int, 0.0f);}
       controller->i_d_des = controller->fw_int;
       /* 弱磁没出力时 q_max 就是 i_max，不用开方（vsqrt 14 拍，且每拍都跑）。
        * 走这条分支时 i_d_des == 0，i_q_des 仍按 ±i_max 钳 —— 与开方版本等价，
        * 因为 sqrt(i_max²-0) 恰好是 i_max。 */
       if(controller->fw_int == 0.0f){
           controller->i_q_des = fast_fmaxf(fast_fminf(controller->i_q_des, controller->i_max), -controller->i_max);
           return;
       }
       float q_max_squared = controller->i_max*controller->i_max - controller->i_d_des*controller->i_d_des;
       float q_max = q_max_squared > 0.0f ? sqrtf(q_max_squared) : 0.0f;
       controller->i_q_des = fast_fmaxf(fast_fminf(controller->i_q_des, q_max), -q_max);


}
void commutate(ControllerStruct *controller, EncoderStruct *encoder)
{
	/* Do Field Oriented Control */

		controller->theta_elec = encoder->elec_angle;
		controller->dtheta_elec = encoder->elec_velocity;
		/* MIT 阻尼与 TORQUE_VEL_LIM 要**最短滞后**的信号：最小二乘窗口在静止时是 256 ms
		 * （速度提示为 0，见 ml_ls_window），从静止起步的几十毫秒里会把阻尼反馈读成 0，
		 * 冲击/阶跃的时机会错。这里保留 19 拍整数差分（475 us，行为与改动前一致）*/
		controller->dtheta_mech = encoder->velocity_diff/GR;
		controller->theta_mech = encoder->angle_multiturn/GR;

       /// Commutation  ///
       dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents - 3.8 us

       controller->i_q_filt = (1.0f-CURRENT_FILT_ALPHA)*controller->i_q_filt + CURRENT_FILT_ALPHA*controller->i_q;	// these aren't used for control but are sometimes nice for debugging
       controller->i_d_filt = (1.0f-CURRENT_FILT_ALPHA)*controller->i_d_filt + CURRENT_FILT_ALPHA*controller->i_d;
       /* v_bus_filt is maintained by analog_sample(), which runs every cycle in
        * every state - see the note there */

       controller->v_max = OVERMODULATION*controller->v_bus_filt*(DTC_MAX-DTC_MIN)*SQRT1_3;

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
            * 同量纲；换算到电机端安培是 /(KT*GR)。
            * MTPA 下"每安培的转矩"是 kt_eff 而不是 KT，但转矩由 **i_q** 产生、限幅却
            * 作用在 |I| 上，所以要再除本拍电流矢量的 q 轴占比 cosφ = |iq|/|I|：
            *   |I|_torque = TORQUE_LIM / (GR·kt_eff·cosφ)
            * 只除 kt_eff（假设 cosφ=1）会让实际可交付转矩比配置值小（ξ=3 的电机在
            * 满电流时只剩 59%）。MTPA 关时 Id=0 ⇒ cosφ=1，与改动前逐位一致。
            * 电流方向是**本拍**的（field_weaken 已经定完），kt_eff 是上一拍的
            * （本拍的 Id 还没定）——与 ODrive 的 max_available_torque() 同一个理由。 */
           {
               float kt = controller->kt_eff;
               if (!(kt > 0.0f) || !isfinite(kt)) { kt = KT; }
               if ((TORQUE_LIM > 0.0f) && (kt*GR > 0.0f)) {
                   float t_lim = TORQUE_LIM/(kt*GR);
                   /* 只有"可能生效"时才去算 cosφ（一次开方 + 一次除法），
                    * 而 cosφ ≤ 1 只会把上限**抬高**，所以 t_lim ≥ i_lim 时必然不生效 */
                   if (t_lim < i_lim) {
                       /* 只有 d 轴真有电流（MTPA/弱磁）时 cosφ 才不等于 1；Id=0 的
                        * 常见情形连开方都不做，与改动前逐位一致且零开销 */
                       if (controller->i_d_des != 0.0f) {
                           float m2 = controller->i_d_des*controller->i_d_des
                                    + controller->i_q_des*controller->i_q_des;
                           if (m2 > 0.0f) {
                               float cosphi = fabsf(controller->i_q_des)/sqrtf(m2);
                               /* cosφ 很小说明电流几乎全在 d 轴：限幅不适用 */
                               if (cosphi > 0.05f) { t_lim /= cosphi; }
                           }
                       }
                       if(t_lim < i_lim){i_lim = t_lim;}
                   }
               }
           }
           controller->i_max = i_lim;
           controller->temp_i_lim = i_lim;

           /* MTPA：实际电流上限对应的 SPM 等价 q 电流峰值与增益 g。表的轨迹与电流
            * 上限无关，所以降流不需要重建表，只需要按小圆重新收口（见 mtpa_iq_peak，
            * g 在里面一起算好并缓存）。torque_control() / motion.c 用它当钳位上限，
            * mtpa_apply() 用它兜底 */
           controller->i_q_peak = mtpa_iq_peak(i_lim);
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

       /* 等效转矩常数（含磁阻分量）：本拍的 Id 到这里才定下来（MTPA 与弱磁都写完了）。
        * MTPA 关、Id=0 时 kt_eff == KT，转矩限幅/机械功率/遥测全部逐位不变。
        * 下限 5% KT 挡住 Ld/Lq 填错时 kt_eff 变成 0/负（那会让 i_q_des = τ/kt 爆掉）。 */
       {
           float kt = KT + 1.5f*PPAIRS*(L_D - L_Q)*controller->i_d_des;
           controller->kt_eff = (kt > 0.05f*KT) ? kt : KT;
       }

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

       /* ---- 死区补偿（config.dt_comp > 0 才生效，算法与符号约定见 dtc_comp.h）----
        * 逐桥臂、用**实测**相电流的符号（不是 i_des：电流环饱和时两者会分叉），
        * 直接加在占空比上 —— 死区误差本来就是逐桥臂的，而星形接法没有中线，
        * 补偿量里的共模分量对电机没有影响（min-max 注入也会吸收一部分），所以
        * 逐桥臂注入不需要额外处理零序。
        * 位置在 svm() 之后、set_dtc() 之前：set_dtc() 的 pcl_check_needed 判据要看
        * **补偿后**的占空比（那才是真正写进 CCR、决定低边窗口的值）。
        * 只在闭环路径（本函数）里补；标定走 commutate_open_loop()，要干净的电压
        * 阶跃，不补（与 ODrive 68701584c 一致）。 */
       if(DT_COMP > 0.0f){
           /* 1/tol 只在参数变化时算一次：中断里不做除法。tol_prev 初值 0 保证第一拍
            * 就会算（config.dt_comp_tol 的下限是 0.01，不会等于 0）。参数由主循环的
            * endpoint 写、这里只读，32 位对齐 float 的读写在 M4 上是原子的。 */
           static float tol_prev = 0.0f;
           static float inv_tol = 0.0f;
           float tol = DT_COMP_TOL;
           if(tol != tol_prev){
               tol_prev = tol;
               inv_tol = (tol > 1e-3f) ? (1.0f/tol) : 1000.0f;
           }
           controller->dtc_u = dtc_comp_duty(controller->dtc_u, controller->i_a, DT_COMP, inv_tol, DTC_MIN, DTC_MAX);
           controller->dtc_v = dtc_comp_duty(controller->dtc_v, controller->i_b, DT_COMP, inv_tol, DTC_MIN, DTC_MAX);
           controller->dtc_w = dtc_comp_duty(controller->dtc_w, controller->i_c, DT_COMP, inv_tol, DTC_MIN, DTC_MAX);
       }

       /* 占空比先出去，再做遥测。双更新（docs/DEV.md 的 P2）之后这个写操作有
        * 12.5 µs 的硬截止（谷值 UEV 要装载它），所以"算完立刻写、其余往后排"
        * 是必须的；现在这样排在 40 kHz 上也只多给一点余量，控制结果不变。 */
       set_dtc(controller);

       /* ---- 母线电流 / 机械功率（估算，见下）----
        * 硬件没有直流分流器：用 dq 电压和的功率反推。
        *   P_elec = 1.5 * V_PER_CMD * (v_d*i_d + v_q*i_q)   [W]
        *   i_bus  = P_elec / v_bus                           [A]
        * 1.5 是幅值不变 dq 变换的功率系数。放在 set_dtc() 之后，v_d/v_q 用的是
        * 本拍刚算出的输出（比原来用上一拍的更准，差一个控制周期，遥测和软限幅
        * 都用不着更严）；i_d/i_q 仍是本拍的测量值。
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
           controller->mech_power = controller->i_q_filt*controller->kt_eff*GR*controller->dtheta_mech;
       }

    }
float angle_est = 0; 

void commutate_open_loop(ControllerStruct *controller, EncoderStruct *encoder)
{
    	/* Do Field Oriented Control */

		controller->theta_elec = encoder->elec_angle;
        angle_est = fmodf(controller->theta_elec, TWO_PI_F);
		controller->dtheta_elec = encoder->elec_velocity;
		/* 与 commutate() 同理：阻尼要最短滞后的速度（见那里的注释）*/
		controller->dtheta_mech = encoder->velocity_diff/GR;
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
    {
        /* MTPA：电流上限在"SPM 等价 i_q"域里是 i_q_peak —— 实际电流圆上还能多出
         * 这份转矩（= telemetry.mtpa_gain），饱和时就是峰值转矩提升。
         * MTPA 未激活时 i_q_peak == i_max，钳位与改动前逐位一致。
         * `i_max` 仍是**物理**电流上限，commutate() 的 limit_norm 仍按它收口 */
        float i_lim = controller->i_q_peak;
        controller->i_q_des = fast_fmaxf(fast_fminf(torque_des/(KT*GR), i_lim), -i_lim);
    }

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
	/* 必须把租约计数器一并清零：它只在 MOTOR_MODE 里自增并停在
	 * CAN_TIMEOUT_CYCLES+1，而 run_fsm() 用 `> CAN_TIMEOUT_CYCLES` 判超时。
	 * 不清的话超时之后每一拍都会再进来一次，把主站刚写进来的 command.* 当拍
	 * 清零 —— 表现为 CSV/CSP/MIT 只生效一个控制周期，电机完全不跟指令。 */
	controller->timeout = 0;
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
