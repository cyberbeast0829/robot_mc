/*
 * cogging.h
 *
 * 齿槽转矩补偿（anti-cogging）。齿槽力矩是转子磁钢与定子齿相互作用产生的、
 * 随转子机械位置周期变化的力矩脉动：低速时表现为速度波动/爬行，位置保持时
 * 表现为周期性误差，也是行星减速关节"转速不稳"的主要来源之一。
 *
 * 原理与 ODrive 的做法一致（前馈，不是反馈）：
 *   1) 标定时把转子逐个位置**保持住**，读速度环积分器 —— 稳态下它就等于
 *      "维持这个位置所需的 i_q"，也就是齿槽力矩（+ 重力/摩擦的直流分量）；
 *   2) 正反各扫一遍取平均，抵消摩擦/齿隙的迟滞项，再减掉均值去掉重力与
 *      平均摩擦；
 *   3) 运行时按转子位置查表，把补偿电流**加到** i_q 指令上（见 cogging_apply）。
 *
 * 表长 COG_N 个点、覆盖**一个转子机械周期**（一个编码器整圈），int16 定点。
 * 索引直接用编码器原始计数（不受 config.m_zero 影响），零点/机械零点改动不会
 * 让表失效。
 *
 * 实时性：cogging_apply() 只在控制中断里做一次查表插值（约 30 个周期）；
 * 标定状态机也在控制中断里（与惯量标定同构），合并/质检/打印在主循环。
 */

#ifndef INC_COGGING_H_
#define INC_COGGING_H_

#include <stdint.h>
#include "foc.h"
#include "position_sensor.h"
#include "cogging_law.h"

/* 标定状态机（cogging_cal.state） */
#define COG_CAL_IDLE		0		// 不在标定
#define COG_CAL_SETTLE		1		// 零电流等关节停稳
#define COG_CAL_MOVE		2		// （hold 法）位置保持到下一个表点，等收敛
#define COG_CAL_SAMPLE		3		// （hold 法）在该点累积速度环积分器
#define COG_CAL_SWEEP		4		// （sweep 法）匀速扫一圈，按转子位置分箱累积

/* 标定方法（config.cogging_method） */
#define COG_METHOD_HOLD		0		// 逐点停住读积分器：静态力矩，但受静摩擦（粘滞）污染
#define COG_METHOD_SWEEP	1		// 匀速旋转时读速度环输出：动摩擦是常值、两向抵消，精度高一个量级

/* 结果码（cogging_cal.status / system.cogging_status） */
#define COG_STATUS_IDLE		0		// 从未运行
#define COG_STATUS_RUNNING	1		// 运行中
#define COG_STATUS_OK		2		// 成功（表已写入 RAM，用 action.save 落盘）
#define COG_ERR_ABORT		(-1)	// 中途退出电机模式 / 被 zero_commands 中止
#define COG_ERR_POINTS		(-2)	// 太多点没收敛
#define COG_ERR_MATCH		(-3)	// 正反两遍差异过大（齿隙/摩擦/参数不对）
#define COG_ERR_TIMEOUT		(-4)	// 整体超时
#define COG_ERR_CONFIG		(-5)	// 外环增益未配置（pos_gain / vel_gain / vel_integrator_gain）

/* 一个表点的 state 相关变量都放在这里（不在 ControllerStruct 里，免得控制结构
 * 被一张 2 KB 表和两个 3 KB 的临时数组撑大）。pass1/pass2 只在标定期间使用。 */
typedef struct{
	volatile uint8_t  state;		// COG_CAL_x
	volatile uint8_t  pass;			// 0 = 正转，1 = 反转
	volatile uint16_t index;		// 当前表点
	volatile uint16_t done;			// 已完成的点数（0..2*COG_N，进度用）
	volatile uint8_t  data_ready;	// ISR 采集完成，等主循环合并
	volatile int8_t   status;		// COG_STATUS_x / COG_ERR_x
	uint8_t  method;				// COG_METHOD_x（开始标定时锁定）
	uint32_t t_point;				// 当前点的计时 / sweep 的圈内计时（控制周期）
	uint32_t t_rev;					// sweep 一圈的周期数
	uint16_t settle_cnt;			// 位置误差连续在阈值内的周期数 / 反向后的跳过计数
	int32_t  start_abs;				// 第 0 个点的绝对计数（config.m_zero 坐标系）
	uint16_t start_bin;				// 第 0 个点对应的表索引（hold 法存表要用，见 cog_record）
	float    gr;					// 标定开始时的齿轮比（中途改动不会打乱目标）
	float    p_tgt;					// 目标位置（输出轴 rad；hold = 点，sweep = 连续斜坡）
	float    rate;					// sweep 的位置斜坡速率（输出轴 rad/s，带方向）
	float    sum;					// 积分器累积
	uint16_t n;						// 已累积的点数
	uint16_t bad;					// 两遍都没收敛的点数
	float    amp;					// 表的 RMS 幅值（A）
	float    mismatch;				// 正反两遍的 RMS 差异（A）
	float    pass1[COG_N];			// hold：每点测量值；sweep：每格累积和
	float    pass2[COG_N];
	uint8_t  ok1[COG_N];
	uint8_t  ok2[COG_N];
	uint16_t cnt1[COG_N];			// sweep：每格的样本数
	uint16_t cnt2[COG_N];
} CoggingCalStruct;

extern CoggingCalStruct cogging_cal;
extern int16_t COGGING_LUT[COG_N];

/* 运行时前馈：把查表得到的补偿电流加到 controller->i_q_des 上。
 * 标定期间、表无效、config.cogging_en 为假时什么都不做。只在控制中断里调用。 */
void cogging_apply(ControllerStruct *controller, const EncoderStruct *encoder);

/* 标定状态机。返回 1 表示已接管本拍控制（motion_control 直接 return），
 * 0 表示没在标定（正常控制律继续跑）。只在控制中断里调用。 */
int cogging_cal_step(ControllerStruct *controller, const EncoderStruct *encoder);

/* 中止标定并把请求清零（zero_commands 调用：离开电机模式、故障、CAN 超时） */
void cogging_abort(ControllerStruct *controller);

/* 清空表并让补偿失效（action.cogging_reset，只改 RAM） */
void cogging_reset(ControllerStruct *controller);

/* 主循环：合并两遍扫描、质检、量化写表。data_ready 由 ISR 置位。 */
void cogging_publish_result(void);

/* 遥测/诊断用的只读访问器（endpoint 的 get 表达式直接调用） */
int8_t cogging_status(void);
int32_t cogging_progress(void);
int32_t cogging_progress_total(void);
float cogging_amplitude(void);
float cogging_mismatch(void);
uint16_t cogging_bad_points(void);
float cogging_index_value(int32_t idx);
/* 诊断：标定两遍各自的原始图（cogging_publish_result 之前的副本） */
float cogging_pass1_value(int32_t idx);
float cogging_pass2_value(int32_t idx);

#endif /* INC_COGGING_H_ */
