/*
 * scope.h
 *
 * 电流环的"片上示波器 + 扫频仪"（台架测带宽极限用）。
 *
 * 为什么必须做在固件里：`telemetry.i_q` 通过 endpoint 读一次要一整次请求/应答
 * 往返（100~300 Hz，还有抖动），Nyquist 连 150 Hz 都不到，而电流环的实际穿越
 * 在 0.78×`config.i_bw`（默认 2000 → 1546 Hz，台架实测）。要在 kHz 量级测到
 * PM=45° 的极限，只能在 40 kHz 中断里自己采样、自己解调。
 *
 * 两种模式（`scope.wave`）：
 *   1 STEP  在 i_q 参考上叠加一个阶跃（bias → bias+amp），把 cmd/fb/v/θ 四通道
 *           按 40 kHz 存进 `scope.buf`，主循环用 myprintf 打出来给上位机。
 *   2 SINE  按整数倍频点（f = k·40 kHz/2048）连续注入正弦，在中断里用
 *           sincos_lut 做锁相累加（Σx·cos / Σx·sin），一个频点只回几个 float
 *           —— 不需要大数据传输，信噪比按 √N 提升，CAN 也能跑。
 *
 * 注入点是 `run_fsm()` 的 MOTOR_MODE 分支、`field_weaken()` 之后、`commutate()`
 * 之前：**两个轴都被改写**（被测轴 = bias + stimulus，另一轴 = 0），所以齿槽
 * 前馈/弱磁的额外 i_q 不会污染测量；实测值在 `commutate()` 之后取（同一拍的
 * dq0 结果）。
 *
 * 安全：只有在门极使能时才会注入（`scope_apply` 只从 MOTOR_MODE 调用，并且
 * 主循环的 `scope_poll()` 在门极关掉时把状态机打成 ERR），另有 2 s 硬超时，
 * 主机掉线也不会一直灌电流。捕获完整次约 64 ms（PREROLL 512 + ACC 2048 拍）。
 */

#ifndef INC_SCOPE_H_
#define INC_SCOPE_H_

#include <stdint.h>
#include "foc.h"

#define SCOPE_N			1024		/* 阶跃/波形捕获点数（1024 × 4 通道 × 2 B = 8 KB）*/
#define SCOPE_ACC_N		2048		/* 锁相累积窗，必须是 2 的幂（整数周期）*/
#define SCOPE_PREROLL	512			/* 累积前的稳定拍数（12.8 ms）*/
#define SCOPE_STEP_AT	64			/* 阶跃发生在第几拍（前面留基线）*/
#define SCOPE_CH		4			/* buf 的通道数：cmd / fb / v / theta */
#define SCOPE_TIMEOUT	(40000u*2u)	/* 硬超时（控制周期，2 s）*/
#define SCOPE_FS		40000.0f	/* 电流环/采样频率（Hz）*/

/* scope.wave */
#define SCOPE_OFF		0
#define SCOPE_STEP		1
#define SCOPE_SINE		2
#define SCOPE_PASSIVE	3		/* 只观测不注入：raw/count/速度/i_q（原 diag.c 的功能）*/

/* scope.state */
#define SCOPE_ST_IDLE	0
#define SCOPE_ST_PREROLL 1
#define SCOPE_ST_RUN	2
#define SCOPE_ST_DONE	3			/* 结果已算好，可以读遥测（**只有主循环会置**，
									 * 中断只置 acc_done，避免主机读到还没算的 0）*/
#define SCOPE_ST_ERR	4			/* 被中止（离开电机模式/超时/参数非法）*/
/* PASSIVE 的额外超时预算（控制周期）：decim 上限 400 × 1024 点 ≈ 10.3 s */
#define SCOPE_TIMEOUT_PASSIVE	(40000u*11u)

/* scope.err */
#define SCOPE_ERR_NONE		0
#define SCOPE_ERR_PARAM		1		/* 幅值/频率越界 */
#define SCOPE_ERR_GATE		2		/* 门极已关（离开电机模式）*/
#define SCOPE_ERR_TIMEOUT	3		/* 硬超时 */

typedef struct{
	/* 参数（endpoint 可写；不持久化，上电全 0 = 关闭）*/
	volatile uint8_t wave;			/* SCOPE_OFF / SCOPE_STEP / SCOPE_SINE */
	volatile uint8_t axis;			/* 0 = q 轴，1 = d 轴 */
	volatile float amp;				/* 正弦峰值 / 阶跃幅度（A）*/
	volatile float bias;			/* 直流偏置（A）*/
	volatile float freq;			/* 正弦频率（Hz，按 19.53 Hz 栅格量化）*/
	volatile uint16_t decim;		/* PASSIVE 抽取：每 decim 个控制周期存一点（≥1）*/

	/* 运行时 */
	volatile uint8_t state;
	volatile uint8_t err;
	volatile uint8_t acc_done;		/* 中断采样够了（RUN → 由主循环置 DONE）*/
	volatile uint8_t dump_req;		/* 主循环请求导波形（action.scope_dump）*/
	volatile uint32_t idx;			/* 已跑的控制拍数 */
	volatile uint32_t ticks;		/* 硬超时计时 */
	uint32_t n_run;					/* 本次要跑的拍数（N 或 ACC_N）*/
	uint8_t  wave_run;				/* scope_start 时锁存的模式：运行中被主站改
									 * 了 scope.wave 也不会让累加和注入岔开 */
	float    u_scale;				/* |bias| + amp：限幅检测的绝对基准 */
	uint32_t k;						/* 频点：f = k·SCOPE_FS/SCOPE_ACC_N */
	uint32_t phase;					/* 32 位相位累加器 */
	uint32_t phase_step;			/* = k·2^32/SCOPE_ACC_N（N=2048 时是精确移位）*/
	uint16_t sub;					/* PASSIVE 的抽取计数器 */
	int32_t raw0, count0;			/* PASSIVE 的差分基准 */
	float u;						/* 本拍注入值 */
	float sin_ref, cos_ref;			/* 本拍激励的参考相量（apply 算好，capture 直接用）*/

	/* 锁相累加：只累加**实测**电流。参考是固件自己产生的 cos，相量解析可知
	 * （基波峰值 = amp、相位 = 0），不必花中断时间去累加它 —— 这一步省下
	 * 约一半的每拍开销（40 kHz 中断本来就用掉了 ~95% 的预算）。
	 * 是否被限幅由 clipped 标志单独报告 */
	float fb_sc, fb_ss, fb_s0, fb_s2;
	volatile uint8_t clipped;		/* 参考被 commutate() 的 i_max 限幅过 */
	float th_prev, th_acc, th_min, th_max;

	/* 结果 */
	float freq_act;					/* 实际频率（量化后）*/
	float cmd_amp, fb_amp;			/* 参考基波峰值（= scope.amp）/ 实测基波峰值（A）*/
	float t_mag, t_phase_deg;		/* 闭环 T = fb/cmd */
	float dc, resid;				/* fb 的直流与残差 RMS（A）*/
	float motion;					/* 捕获窗内转子电角度行程（°，判"转子动了没有"）*/
	float peak, overshoot, t90, settle;	/* 阶跃指标（A / % / s / s）*/

	int16_t buf[SCOPE_N][SCOPE_CH];
} ScopeStruct;

extern ScopeStruct scope;

/* ---- 主循环 / endpoint 入口 ---- */
void scope_start(ControllerStruct *controller);	/* action.scope_start */
void scope_stop(void);							/* action.scope_stop */
void scope_request_dump(void);					/* action.scope_dump */
void scope_poll(void);							/* run_fsm_slow()：算结果、导波形、看门狗 */

/* ---- 40 kHz 中断入口（run_fsm 的 MOTOR_MODE 分支）---- */
void scope_apply(ControllerStruct *controller);		/* commutate() 之前：注入 */
void scope_capture(ControllerStruct *controller);	/* commutate() 之后：采样/累加 */

/* ---- 只读访问器（endpoint 的 get 表达式直接调用）---- */
uint8_t scope_state(void);
float scope_freq_actual(void);
float scope_cmd_amplitude(void);
float scope_fb_amplitude(void);
float scope_gain_db(void);
float scope_phase(void);
float scope_dc(void);
float scope_noise(void);
float scope_motion(void);
float scope_peak(void);
float scope_overshoot(void);
float scope_t90(void);
float scope_settle(void);
uint8_t scope_clip(void);

#endif /* INC_SCOPE_H_ */
