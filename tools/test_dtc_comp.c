/*
 * test_dtc_comp.c
 *
 * 死区补偿（dtc_comp.h）的 host 侧回归测试，不需要目标板。
 *
 * 覆盖：
 *   - 平滑符号：0 处为 0、饱和到 ±1、线性过渡、NaN → 0（不补）
 *   - 单桥臂：frac = 0 关闭；正/负电流对称；脉宽短于死区的桥臂不动；
 *     钳位到 [lo, hi]；NaN 输入不往下游传
 *   - 线电压精确抵消：逐桥臂注入后 Vbus*(duty_x − duty_y) − Ve*(sign_x − sign_y)
 *     与补偿前完全一致 —— 这是"补偿量对、符号对"的代数形式
 *   - 平滑符号的残余只在 |i| < tol 的过零段出现，且不超过不补偿时的误差
 *
 * 运行：make hosttest
 */

#include <stdio.h>
#include <math.h>

#include "dtc_comp.h"

static int failures;

#define CHECK(cond) do{ \
	if(!(cond)){ \
		printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		failures++; \
	} \
}while(0)

#define CHECK_NEAR(a, b, tol) do{ \
	double _a = (double)(a), _b = (double)(b); \
	if(!(fabs(_a - _b) <= (tol))){ \
		printf("FAIL %s:%d: %s == %.9f, 期望 %.9f (tol %g)\n", \
		       __FILE__, __LINE__, #a, _a, _b, (double)(tol)); \
		failures++; \
	} \
}while(0)

/* 本机量纲：DTC_MIN/DTC_MAX、0.2 A 的平滑宽度（inv_tol = 5） */
#define LO		0.0f
#define HI		0.9f
#define FRAC	0.00333f
#define INV_TOL	5.0f

static void test_sign(void)
{
	CHECK_NEAR(dtc_comp_sign(0.0f, INV_TOL), 0.0f, 1e-9);
	CHECK_NEAR(dtc_comp_sign(0.1f, INV_TOL), 0.5f, 1e-6);	/* 过渡段线性 */
	CHECK_NEAR(dtc_comp_sign(-0.1f, INV_TOL), -0.5f, 1e-6);
	CHECK_NEAR(dtc_comp_sign(1.5f, INV_TOL), 1.0f, 1e-9);	/* 饱和 */
	CHECK_NEAR(dtc_comp_sign(-1.5f, INV_TOL), -1.0f, 1e-9);
	CHECK_NEAR(dtc_comp_sign(NAN, INV_TOL), 0.0f, 1e-9);	/* NaN 不补 */
}

static void test_leg(void)
{
	/* frac = 0 / 负值 = 关闭 */
	CHECK_NEAR(dtc_comp_duty(0.45f, 2.0f, 0.0f, INV_TOL, LO, HI), 0.45f, 1e-9);
	CHECK_NEAR(dtc_comp_duty(0.45f, 2.0f, -1.0f, INV_TOL, LO, HI), 0.45f, 1e-9);

	/* 正电流加、负电流减，幅度对称（"误差总与电流反号"的直接体现） */
	CHECK_NEAR(dtc_comp_duty(0.45f, 2.0f, FRAC, INV_TOL, LO, HI), 0.45f + FRAC, 1e-6);
	CHECK_NEAR(dtc_comp_duty(0.45f, -2.0f, FRAC, INV_TOL, LO, HI), 0.45f - FRAC, 1e-6);
	CHECK_NEAR(dtc_comp_duty(0.45f, 0.0f, FRAC, INV_TOL, LO, HI), 0.45f, 1e-9);

	/* 没在开关的桥臂：脉宽短于死区，不能补 */
	CHECK_NEAR(dtc_comp_duty(0.001f, 2.0f, FRAC, INV_TOL, LO, HI), 0.001f, 1e-9);
	CHECK_NEAR(dtc_comp_duty(0.999f, -2.0f, FRAC, INV_TOL, LO, HI), 0.999f, 1e-9);

	/* 钳位：补偿不改变 [DTC_MIN, DTC_MAX] 这个不变量。注意 lo 只可能被 NaN 命中：
	 * 门控要求 dtc > frac，所以负向补偿永远减不到 0 以下 */
	CHECK_NEAR(dtc_comp_duty(HI, 2.0f, FRAC, INV_TOL, LO, HI), HI, 1e-9);
	CHECK_NEAR(dtc_comp_duty(LO, -2.0f, FRAC, INV_TOL, LO, HI), LO, 1e-9);
	CHECK_NEAR(dtc_comp_duty(0.004f, -2.0f, FRAC, INV_TOL, LO, HI), 0.004f - FRAC, 1e-6);

	/* 非法输入不能把 NaN 传下去（下游要转成 CCR/uint16_t） */
	CHECK_NEAR(dtc_comp_duty(NAN, 2.0f, FRAC, INV_TOL, LO, HI), LO, 1e-9);
	CHECK_NEAR(dtc_comp_duty(0.45f, NAN, FRAC, INV_TOL, LO, HI), 0.45f, 1e-9);
}

/* ---------------------------------------------------------------------------
 * 线电压残余：电机只看得见线电压（星形无中线），所以判据是
 *
 *   V_actual = Vbus*(duty_x - duty_y) - Ve*(sign_x - sign_y)   对比补偿前的
 *   V_ideal  = Vbus*(duty0_x - duty0_y)
 *
 * 逐桥臂注入 duty_x += frac*sign_x，取 Ve = frac*Vbus 时残余恒为 0
 * （这正是"占空比当量"这个量纲的由来：与母线电压无关）。平滑符号在 |i| < tol 的
 * 过零段给不出满幅 ±1，残余因此不为 0，但它必须小于不补偿时的误差。
 * ------------------------------------------------------------------------- */
#define VBUS	36.8
#define I_AMP	2.0
#define I_TOL	(1.0f/INV_TOL)
#define VE		((double)FRAC*VBUS)

static double hard_sign(double v)
{
	return (v > 0.0) ? 1.0 : ((v < 0.0) ? -1.0 : 0.0);
}

static void test_line_voltage(void)
{
	static const float duty0[3] = { 0.30f, 0.45f, 0.62f };
	double worst_full = 0.0, worst_any = 0.0;
	int n_full = 0;

	for(int deg = 0; deg < 360; deg++){
		double th = (double)deg*M_PI/180.0;
		float i[3] = { (float)(I_AMP*cos(th)),
		               (float)(I_AMP*cos(th - 2.0*M_PI/3.0)),
		               (float)(I_AMP*cos(th + 2.0*M_PI/3.0)) };
		float d[3];
		int all_big = 1;
		for(int x = 0; x < 3; x++){
			d[x] = dtc_comp_duty(duty0[x], i[x], FRAC, INV_TOL, LO, HI);
			if(fabsf(i[x]) < I_TOL){ all_big = 0; }
		}
		if(all_big){ n_full++; }
		for(int p = 0; p < 3; p++){
			int x = p, y = (p + 1) % 3;
			double ideal = VBUS*((double)duty0[x] - (double)duty0[y]);
			double act   = VBUS*((double)d[x] - (double)d[y])
			             - VE*(hard_sign(i[x]) - hard_sign(i[y]));
			double e = fabs(act - ideal);
			if(e > worst_any){ worst_any = e; }
			if(all_big && (e > worst_full)){ worst_full = e; }
		}
	}

	/* 三相都远离过零：残余只剩 float 舍入（< 1e-5 V，比 Ve 小 4 个量级以上） */
	CHECK(n_full > 270);
	CHECK(worst_full < 1e-5);
	/* 过零段（平滑符号）也不允许比不补偿更差：残余 ≈ Ve 封顶 */
	CHECK(worst_any <= VE*1.01 + 1e-6);
	printf("  线电压残余：正常段 %.3e V，全部角度 %.4f V（Ve = %.4f V）\n",
	       worst_full, worst_any, VE);
}

int main(void)
{
	test_sign();
	test_leg();
	test_line_voltage();

	if(failures){
		printf("test_dtc_comp: %d 项失败\n", failures);
		return 1;
	}
	printf("test_dtc_comp: 全部通过\n");
	return 0;
}
