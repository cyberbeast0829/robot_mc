/*
 * scope.c
 *
 * 电流环"片上示波器 + 扫频仪"，接口与用法见 scope.h。
 *
 * 分工（和 cogging.c 同构）：
 *   40 kHz 中断  scope_apply()   产生激励（阶跃或正弦）并改写 i_d_des/i_q_des
 *                scope_capture() 取同一拍的实测电流/电压做定点存储或锁相累加
 *   主循环       scope_poll()    算锁相结果/阶跃指标、看门狗、导波形（myprintf）
 *
 * 中断里只做乘法与加法：正弦用 math_ops.c 的 sincos_lut（查表 + 线性插值），
 * 没有 libm 调用；atan2/log 这些全在主循环。
 */

#include <math.h>

#include "scope.h"
#include "scope_law.h"
#include "math_ops.h"
#include "structs.h"
#include "user_config.h"
#include "main.h"

ScopeStruct scope;

/* 32 位相位累加器转弧度 */
#define SCOPE_PHASE_TO_RAD (TWO_PI_F/4294967296.0f)
/* 度数换算（主循环用） */
#define SCOPE_RAD_TO_DEG   (180.0f/PI_F)

void scope_start(ControllerStruct *controller)
{
	const uint8_t wave = scope.wave;

	if(wave == SCOPE_PASSIVE){
		/* 只观测不注入：不需要 amp/bias，采样窗固定 SCOPE_N 点，抽取比由
		 * scope.decim 给（时间基准 = SCOPE_FS/decim）。录 raw/count/速度/i_q。 */
		const uint32_t primask = __get_PRIMASK();
		__disable_irq();
		scope.err = SCOPE_ERR_NONE;
		scope.acc_done = 0;
		scope.dump_req = 0;
		scope.wave_run = SCOPE_PASSIVE;
		scope.idx = 0;
		scope.ticks = 0;
		scope.sub = 0;
		scope.n_run = SCOPE_N;
		scope.raw0 = comm_encoder.raw;
		scope.count0 = comm_encoder.count;
		scope.state = SCOPE_ST_RUN;
		__set_PRIMASK(primask);
		return;
	}

	if((wave != SCOPE_STEP) && (wave != SCOPE_SINE)){ scope_stop(); return; }	/* 0 = 关闭 */

	float amp = scope.amp;
	float bias = scope.bias;
	if(!(amp > 0.0f)){
		scope_stop();
		scope.state = SCOPE_ST_ERR;
		scope.err = SCOPE_ERR_PARAM;			/* amp 必须 > 0 */
		return;
	}

	const float total = fabsf(bias) + amp;	float i_lim = controller->i_max;
	if(!(i_lim > 0.0f)){ i_lim = 0.0f; }
	if(total > i_lim){
		scope.state = SCOPE_ST_ERR;
		scope.err = SCOPE_ERR_PARAM;		/* 幅值超过本轮 i_max：先降 amp 或抬 i_max */
		return;
	}

	/* 与 40 kHz 中断互斥：清计数器/累加器的过程中不能让中断看到半套状态 */
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();

	scope.err = SCOPE_ERR_NONE;
	scope.acc_done = 0;
	scope.dump_req = 0;
	scope.wave_run = wave;			/* 锁存模式：运行中主站改 scope.wave 不影响本次 */
	scope.u_scale = total;			/* 限幅检测的绝对基准（|bias|+amp）*/
	scope.idx = 0;
	scope.ticks = 0;
	scope.phase = 0;
	scope.phase_step = 0;
	scope.u = 0.0f;
	scope.sin_ref = 0.0f;
	scope.cos_ref = 1.0f;
	scope.clipped = 0;
	scope.fb_sc = scope.fb_ss = scope.fb_s0 = scope.fb_s2 = 0.0f;
	scope.th_acc = 0.0f;
	scope.th_min = 0.0f;
	scope.th_max = 0.0f;
	scope.th_prev = controller->theta_elec;
	scope.freq_act = 0.0f;
	scope.fb_amp = 0.0f;
	scope.cmd_amp = amp;			/* 参考基波峰值：解析可知（u = bias + amp*cos），不用累加 */
	scope.t_mag = 1.0f;
	scope.t_phase_deg = 0.0f;
	scope.dc = scope.resid = 0.0f;
	scope.motion = 0.0f;
	scope.peak = scope.overshoot = scope.t90 = 0.0f;
	scope.settle = -1.0f;
	/* 不清 scope.buf：8 KB 的 memset 关中断要 2 ms，等于打断控制环 ~80 拍
	 * （台架实测 period_max 从 6021 跳到 46515 周期）。阶跃捕获会把 1024×4 个
	 * 点全部写一遍，正弦模式根本不读它，所以没必要清零 */

	if(wave == SCOPE_SINE){
		/* 频点必须落在 SCOPE_FS/SCOPE_ACC_N 的整数倍上：窗口里正好 k 个周期，
		 * 锁相求和没有泄漏。k 的上限取到 1/4 采样率（10 kHz）；参数表把 freq
		 * 限在 5000 Hz（k=256），留了一倍余量 */
		int k = (int)lroundf(scope.freq*((float)SCOPE_ACC_N/(float)SCOPE_FS));
		if(k < 0){ k = 0; }
		if(k > (SCOPE_ACC_N/4)){ k = SCOPE_ACC_N/4; }
		if(k < 1){ k = 1; }								/* freq 太小也至少 1 个周期 */
		scope.k = (uint32_t)k;
		/* N 是 2 的幂：2^32/N 是精确移位，累加器不会漂 */
		scope.phase_step = (uint32_t)k << 21;			/* = k * 2^32 / 2048 */
		scope.freq_act = (float)k*(SCOPE_FS/(float)SCOPE_ACC_N);
		scope.n_run = SCOPE_ACC_N;
	}
	else{
		scope.n_run = SCOPE_N;
	}

	scope.state = SCOPE_ST_PREROLL;

	__set_PRIMASK(primask);
}

void scope_stop(void)
{
	scope.state = SCOPE_ST_IDLE;
	scope.err = SCOPE_ERR_NONE;
	scope.u = 0.0f;
}

void scope_request_dump(void)
{
	scope.dump_req = 1;
}

/* ---------------------------------------------------------------------------
 * 中断侧：注入。必须在 commutate() 之前调用。
 * 两个轴都被改写 —— 被测轴 = bias + stimulus，另一轴 = 0。这样齿槽前馈（加到
 * i_q_des）、弱磁（累加到 i_q_des）、外环残留都不会混进测量。
 * ------------------------------------------------------------------------- */
void scope_apply(ControllerStruct *controller)
{
	/* PASSIVE 只观测：一个轴都不改 */
	if(scope.wave_run == SCOPE_PASSIVE){ return; }
	if((scope.state != SCOPE_ST_PREROLL) && (scope.state != SCOPE_ST_RUN)){ return; }
	if(scope.acc_done){ return; } /* 采样完成：交还控制权，i_*_des 当作 0 */

	const uint8_t wave = scope.wave_run;		/* 用 start 时锁存的模式 */
	float u = scope.bias;

	if(wave == SCOPE_SINE){
		float s, c;
		sincos_lut((float)scope.phase*SCOPE_PHASE_TO_RAD, &s, &c);
		scope.sin_ref = s;
		scope.cos_ref = c;
		u += scope.amp*c;
		scope.phase += scope.phase_step;
	}
	else if(wave == SCOPE_STEP){
		/* 阶跃只在 RUN 之后才叠加：预滚期间保持 bias，这样捕获窗开头的 64 拍
		 * 基线是已经稳定的，而不是上一次阶跃的衰减尾巴 */
		if((scope.state == SCOPE_ST_RUN) && (scope.idx >= SCOPE_STEP_AT)){ u += scope.amp; }
		scope.sin_ref = 0.0f;
		scope.cos_ref = 1.0f;
	}
	else{
		scope.sin_ref = 0.0f;
		scope.cos_ref = 1.0f;
	}

	scope.u = u;
	if(scope.axis == 0){
		controller->i_q_des = u;
		controller->i_d_des = 0.0f;
	}
	else{
		controller->i_d_des = u;
		controller->i_q_des = 0.0f;
	}
}

/* ---------------------------------------------------------------------------
 * 中断侧：采样/累加。必须在 commutate() 之后调用（i_d/i_q/v_d/v_q 是这一拍的）。
 * ------------------------------------------------------------------------- */
void scope_capture(ControllerStruct *controller)
{
	if(scope.state == SCOPE_ST_PREROLL){
		scope.idx++;
		if(scope.idx >= SCOPE_PREROLL){
			/* 预滚结束：清累加器，从"实测位置"重新开始记角度行程。
			 * 相位累加器**不**清零：正弦保持连续，窗口里没有额外的相位台阶。
			 * 累积窗正好是整数个周期（2048 = k 个周期），所以窗口起点的相位
			 * 是多少都不影响解调结果 */
			scope.idx = 0;
			scope.fb_sc = scope.fb_ss = scope.fb_s0 = scope.fb_s2 = 0.0f;
			scope.th_acc = 0.0f;
			scope.th_min = 0.0f;
			scope.th_max = 0.0f;
			scope.th_prev = controller->theta_elec;
			scope.state = SCOPE_ST_RUN;
		}
		return;
	}

	if(scope.state != SCOPE_ST_RUN){ return; }
	if(scope.acc_done){ return; }

	if(scope.wave_run == SCOPE_PASSIVE){
		/* 抽取采样：时间基准 = decim × 25 us。硬超时按"最长窗口"给
		 * （decim 上限 400 × 1024 点 ≈ 10.3 s），不能沿用 2 s 的注入超时 */
		if(++scope.ticks > (SCOPE_TIMEOUT + SCOPE_TIMEOUT_PASSIVE)){
			scope.state = SCOPE_ST_ERR;
			scope.err = SCOPE_ERR_TIMEOUT;
			return;
		}
		const uint16_t dec = (scope.decim > 0u) ? scope.decim : 1u;
		if(++scope.sub < dec){ return; }
		scope.sub = 0;

		const uint32_t ip = scope.idx;
		if(ip >= SCOPE_N){ scope.acc_done = 1; return; }
		int16_t *p = scope.buf[ip];
		p[0] = (int16_t)(comm_encoder.raw - scope.raw0);
		p[1] = (int16_t)(uint16_t)(comm_encoder.count - scope.count0);
		/* 速度通道按**输出轴** rad/s 存（×500 = 0.002 rad/s ≈ 0.02 rpm 分辨率，
		 * 量程 ±64 rad/s ≈ ±610 rpm）。用电机侧 rad/s 时 1000 倍率在 300 rpm 就饱和，
		 * 饱和后纹波被削平（通道退化成常数），低速又嫌分辨率不够 */
		float vv = comm_encoder.velocity * (500.0f / (GR > 0.0f ? GR : 1.0f));
		if(vv > 32000.0f){ vv = 32000.0f; }
		else if(vv < -32000.0f){ vv = -32000.0f; }
		p[2] = (int16_t)vv;
		float iq = controller->i_q_des * 1000.0f;
		if(iq > 32000.0f){ iq = 32000.0f; }
		else if(iq < -32000.0f){ iq = -32000.0f; }
		p[3] = (int16_t)iq;
		scope.idx = ip + 1u;
		if(scope.idx >= scope.n_run){ scope.acc_done = 1; }
		return;
	}

	if(++scope.ticks > SCOPE_TIMEOUT){
		scope.state = SCOPE_ST_ERR;
		scope.err = SCOPE_ERR_TIMEOUT;
		return;
	}

	const uint8_t q_axis = (scope.axis == 0) ? 1 : 0;
	const float fb = q_axis ? controller->i_q : controller->i_d;
	const float v = q_axis ? controller->v_q : controller->v_d;

	/* 转子角度行程（展开后累加）：判"转子到底动没动"。齿隙/背隙的台架上
	 * 这是判断这次测量还有没有意义的关键指标 */
	float d = controller->theta_elec - scope.th_prev;
	if(d > PI_F){ d -= TWO_PI_F; }
	else if(d < -PI_F){ d += TWO_PI_F; }
	scope.th_prev = controller->theta_elec;
	scope.th_acc += d;
	if(scope.th_acc < scope.th_min){ scope.th_min = scope.th_acc; }
	if(scope.th_acc > scope.th_max){ scope.th_max = scope.th_acc; }

	const uint32_t i = scope.idx;
	if(scope.wave_run == SCOPE_SINE){
		/* 只累加实测电流：参考是固件自己产生的 cos，相量解析可知（基波峰值 =
		 * amp、相位 = 0），不必花中断时间去累加它。4 个累加器而不是 8 个 */
		const float s = scope.sin_ref;
		const float c = scope.cos_ref;
		scope.fb_sc += fb*c;
		scope.fb_ss += fb*s;
		scope.fb_s0 += fb;
		scope.fb_s2 += fb*fb;
		/* 参考有没有被 commutate() 里的 limit_norm() 削过：没有削的话，
		 * commutate 之后的 i_*_des 就是本拍要的 u。门限用 u_scale（|bias|+amp）
		 * 而不是 |u|：bias=0 时 u 每个周期都过零，相对门限会在零点附近误判 */
		const float des = q_axis ? controller->i_q_des : controller->i_d_des;
		if(fabsf(des - scope.u) > 0.02f*scope.u_scale){ scope.clipped = 1; }
	}
	else if(i < SCOPE_N){
		scope.buf[i][0] = sl_quant(scope.u, SL_SCALE_A);
		scope.buf[i][1] = sl_quant(fb, SL_SCALE_A);
		scope.buf[i][2] = sl_quant(v, SL_SCALE_V);
		scope.buf[i][3] = sl_quant(controller->theta_elec, SL_SCALE_THETA);
	}

	if(++scope.idx >= scope.n_run){
		scope.acc_done = 1;
	}
}

/* ---------------------------------------------------------------------------
 * 主循环：算结果、看门狗、导波形
 * ------------------------------------------------------------------------- */
static void scope_finish(void)
{
	scope.motion = (scope.th_max - scope.th_min)*SCOPE_RAD_TO_DEG;

	if(scope.wave_run == SCOPE_SINE){
		const sl_phasor_t fb = sl_lockin((int)scope.n_run, scope.fb_sc, scope.fb_ss,
		                                 scope.fb_s0, scope.fb_s2);
		scope.fb_amp = fb.amp;
		scope.dc = fb.dc;
		scope.resid = fb.resid;
		/* 参考 = amp*cos(theta) ⇒ 相量就是 amp（纯实部），所以 T = fb/amp、
		 * 相位就是 arg(fb) */
		if(scope.cmd_amp > 1.0e-6f){
			scope.t_mag = fb.amp/scope.cmd_amp;
			scope.t_phase_deg = sl_cplx_arg_deg(sl_cplx_make(fb.re, fb.im));
		}
	}
	else if((scope.wave_run == SCOPE_STEP) && (scope.n_run == SCOPE_N)){
		/* 只解二阶的 i_q/i_d 那一路，采样间隔就是一个控制周期 */
		static float trace[SCOPE_N];
		for(uint32_t i = 0; i < SCOPE_N; i++){
			trace[i] = sl_dequant(scope.buf[i][1], SL_SCALE_A);
		}
		float span = 0.0f, peak = 0.0f, ovs = 0.0f, t90 = -1.0f, settle = -1.0f;
		span = sl_step_metrics(trace, (int)SCOPE_N, 1.0f/SCOPE_FS, SCOPE_STEP_AT,
		                       &peak, &ovs, &t90, &settle);
		(void)span;
		scope.peak = peak;
		scope.overshoot = ovs;
		scope.t90 = t90;
		scope.settle = settle;
		scope.dc = trace[0];			/* 阶跃前的基线 */
	}
}

/* 一次导 8 个点：4 通道 × 4 位十六进制（int16 的补码），一行 128 个字符。
 * 表头第 4 个字段是**有效采样率**（PASSIVE 时 = SCOPE_FS/decim），解析器据此算
 * 时间轴；STEP/SINE 仍然是 40000。 */
static unsigned scope_fs(void)
{
	if((scope.wave_run == SCOPE_PASSIVE) && (scope.decim > 0u)){
		return (unsigned)(SCOPE_FS/(float)scope.decim);
	}
	return (unsigned)SCOPE_FS;
}

static void scope_dump(void)
{
	static char line[8*SCOPE_CH*4 + 1];
	static const char hexd[] = "0123456789ABCDEF";

	myprintf("\r\nSCOPE %u %u %u %u\r\n",
	         (unsigned)SCOPE_N, (unsigned)SCOPE_CH,
	         (unsigned)SCOPE_STEP_AT, (unsigned)scope_fs());

	for(uint32_t i = 0; i < SCOPE_N; i += 8){
		char *p = line;
		for(uint32_t j = i; j < i + 8; j++){
			for(uint32_t c = 0; c < SCOPE_CH; c++){
				const uint16_t v = (uint16_t)scope.buf[j][c];
				*p++ = hexd[(v >> 12) & 0x0F];
				*p++ = hexd[(v >> 8) & 0x0F];
				*p++ = hexd[(v >> 4) & 0x0F];
				*p++ = hexd[v & 0x0F];
			}
		}
		*p = '\0';
		myprintf("%s\r\n", line);
	}
	myprintf("END\r\n");
}

void scope_poll(void)
{
	static uint32_t last_idx;
	static uint32_t stale;

	/* 采样完成：先算结果，再把 state 置成 DONE（主机看到 3 就一定读得到真结果）。
	 * ⚠ 必须放在下面 RUN 分支的早退**之前**：acc_done 之后中断不再推进 idx，
	 * 若先进 RUN 分支，它会在 200 拍后判成"卡住"直接 ERR，DONE 永远到不了 */
	if((scope.state == SCOPE_ST_RUN) && scope.acc_done){
		scope_finish();
		scope.state = SCOPE_ST_DONE;
	}

	if((scope.state == SCOPE_ST_PREROLL) || (scope.state == SCOPE_ST_RUN)){
		/* 离开电机模式 / 门极关掉：立刻停掉注入。正常路径是 action.idle_mode，
		 * 但故障、CAN 掉线也会走到这里 */
		if(!controller.gate_drive_enabled){
			scope.state = SCOPE_ST_ERR;
			scope.err = SCOPE_ERR_GATE;
			return;
		}
		/* 状态机没在前进（例如在 CALIBRATION 里被启动，run_fsm 的 MOTOR_MODE
		 * 分支根本不会调 scope_capture）：主循环看到 idx 不变就中止，
		 * 免得主机一直等一个永远不会完成的捕获。
		 * ⚠ 阈值必须按 decim 放大：PASSIVE 的 idx 每 decim 个控制周期才动一次，
		 * 主循环一圈只要几微秒，200 圈在 decim=100（2.5 ms/点）时会先到顶，
		 * 于是"刚开始就报错"（台架踩过）*/
		const uint32_t stale_max = 200u*((scope.decim > 0u) ? scope.decim : 1u);
		if(scope.idx != last_idx){
			last_idx = scope.idx;
			stale = 0;
		}
		else if(++stale > stale_max){
			scope.state = SCOPE_ST_ERR;
			scope.err = SCOPE_ERR_TIMEOUT;
		}
		return;
	}
	stale = 0;
	last_idx = scope.idx;

	if(scope.state != SCOPE_ST_DONE){ return; }

	if(scope.dump_req){
		scope.dump_req = 0;
		if((scope.wave_run == SCOPE_STEP) || (scope.wave_run == SCOPE_PASSIVE)){
			scope_dump();		/* 会阻塞主循环几百毫秒（myprintf 忙等）*/
		}
	}
}

/* ---- 只读访问器（endpoint 的 get 表达式）---- */
uint8_t scope_state(void)          { return scope.state; }
float scope_freq_actual(void)      { return scope.freq_act; }
float scope_cmd_amplitude(void)    { return scope.cmd_amp; }
float scope_fb_amplitude(void)     { return scope.fb_amp; }
float scope_phase(void)            { return scope.t_phase_deg; }
float scope_dc(void)               { return scope.dc; }
float scope_noise(void)            { return scope.resid; }
float scope_motion(void)           { return scope.motion; }
float scope_peak(void)             { return scope.peak; }
float scope_overshoot(void)        { return scope.overshoot; }
float scope_t90(void)              { return scope.t90; }
float scope_settle(void)           { return scope.settle; }
uint8_t scope_clip(void)           { return scope.clipped; }

float scope_gain_db(void)
{
	if(!(scope.t_mag > 1.0e-6f)){ return -120.0f; }
	return 20.0f*log10f(scope.t_mag);
}
