#ifndef MATH_OPS_H
#define MATH_OPS_H

#define SQRT3 		1.73205080757f
#define SQRT3_2 	0.86602540378f
#define SQRT1_3		0.57735026919f
#define PI_F 		3.14159274101f
#define TWO_PI_F 	6.28318548203f
#define PI_OVER_2_F	1.57079632679f
#define INV_TWO_PI_F	0.15915494310f

#include "math.h"

/* 这几个小函数做成 static inline：它们是纯三元比较，内联没有风险，而 40 kHz
 * 中断里每拍会被调用 20 次以上（commutate/svm/field_weaken/torque_control/
 * check_faults）。台架实测：把包括这几个在内的全部跨 TU 调用展开，整个中断
 * 只快 ~2%（见 docs/DEV.md "中断预算"）—— 真正贵的是取指等待，不是调用本身；
 * 这一项属于"顺手且零代价"，不要指望它解决预算问题。
 * 注意：不要再把它们搬回 math_ops.c 做外部函数——调用点又会退化成调用。 */
static inline float fast_fmaxf(float x, float y){ return (((x)>(y))?(x):(y)); }
static inline float fast_fminf(float x, float y){ return (((x)<(y))?(x):(y)); }
static inline float fmaxf3(float x, float y, float z){ return (x > y ? (x > z ? x : z) : (y > z ? y : z)); }
static inline float fminf3(float x, float y, float z){ return (x < y ? (x < z ? x : z) : (y < z ? y : z)); }
static inline void limit(float *x, float min, float max){ *x = fast_fmaxf(fast_fminf(*x, max), min); }

/* |(x,y)| > limit 时按比例缩回。只做**一次**除法：原来写成
 * `*x = *x*limit/norm` 会被编成两次 vdiv（每次 14 个周期），本函数每拍调用两次。
 * 判据仍是 norm > limit；比例只在限幅生效时用，两种写法的浮点结果差别 ≤1 ULP
 * （数学上相同，扫描电流环 Bode 看不出区别）。 */
static inline void limit_norm(float *x, float *y, float limit){
	float norm = sqrtf(*x * *x + *y * *y);
	if(norm > limit){
		float s = limit/norm;
		*x = *x * s;
		*y = *y * s;
	}
}
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void sincos_lut(float theta, float *s, float *c);
float sin_lut(float theta);
float cos_lut(float theta);



#endif
