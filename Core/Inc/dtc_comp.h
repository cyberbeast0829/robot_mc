/*
 * dtc_comp.h
 *
 * 逆变器死区补偿（纯数学，不引用任何固件头文件；host 回归 tools/test_dtc_comp.c）。
 *
 * 半桥的死区（加上器件正向压降）使实际输出电压系统性偏离指令，误差方向总与
 * 相电流相反：
 *
 *     dV_x = -Ve * sign(i_x)        Ve ≈ (t_dt/Ts)*v_bus + 器件压降
 *
 * 电流环能抑制它的基波分量，抑制不了的是下面这两件事，而它们正是"低力矩时
 * 电流环迟钝"的根因（ODrive 提交 68701584c 实测：一个与 Kp 无关的 ~0.65 V 恒定
 * 压降，表现为约 1 ms 的纯延迟，把 i_bw 调多高都没用）：
 *  - 小电流/零电流附近 Kp*err 抵不过这段电压损失，必须等积分器累积；
 *  - 相电流过零时误差翻向，表现为 6 次谐波电流纹波。
 *
 * 【为什么补在每桥臂的占空比上，而不是折算成 alpha-beta 电压矢量】
 *  - 死区误差本来就是逐桥臂的（每条腿有自己的电流符号），逐桥臂注入是最直接的
 *    形式：duty_x += frac*sign(i_x) 之后线电压误差被精确抵消（取 Ve = frac*v_bus），
 *    见 tools/test_dtc_comp.c 的"线电压残余"回归。
 *  - 折算成 alpha-beta 电压矢量要先做 Clarke 投影，把三路的 ±Ve 压成一个平面矢量，
 *    丢掉的是它们的共模部分（sa+sb+sc）。星形负载看不见共模电压，但三路占空比的
 *    整体偏移会变，接近调制限幅时先被钳位 —— ODrive 提交 68701584c 的旧实现
 *    （2cfde815f）就是直接投影到 alpha-beta 上，实测补偿量偏大 52.8%、方向偏 11°。
 *  - 逐桥臂还有个实际好处：每条腿可以独立门控 —— 没在开关的腿没有死区误差。
 *
 * 【符号怎么定】
 * 死区误差总是**减小**电流幅值（死区期间电流在二极管里续流，电压被拉向"零电流"
 * 方向），所以补偿方向就是"让 i 变大"的方向。本文件只在占空比空间工作，而
 * "dtc 变大 ⇒ i 变大"由电流环自身的正增益稳定保证，因此这里不需要知道
 * GHS/GLS 与 CHx/CHxN 的对应关系、也不需要知道低边分流放大的极性 ——
 * 那两个是原理图/台架的事，写死在这里只会让下一个人不敢改。
 *
 * 【量纲】
 * frac 是**占空比当量**（Ve/v_bus），不是伏特：死区是一段被吃掉的时间
 * （t_dt/Ts），折算到占空比上乘 v_bus 就抵消了，所以补偿天然对母线电压波动免疫，
 * 中断里也不用读 v_bus、不做除法。器件压降那部分只影响 frac 的取值（参数标定）。
 */

#ifndef INC_DTC_COMP_H_
#define INC_DTC_COMP_H_

/* 平滑符号 sign'(i) = clamp(i/tol, -1, +1)，inv_tol = 1/tol（调用方预先算好，
 * 中断里不做除法）。硬 sign() 在相电流过零时会来回跳，注入的畸变比消掉的还多；
 * 线性过渡的宽度 tol **必须大于采样时刻的电流纹波幅值**（本机 2~3 A 峰峰纹波 →
 * 0.5 A），否则过零段的符号每拍翻转、补偿自己成了扰动源：台架实测 tol = 0.2 A 时
 * 近零电流的残差涨 10 倍、低偏置穿越从 1097 掉到 857 Hz（见 docs/DEV.md）。
 * NaN 电流 → 0（本拍不补）。 */
static inline float dtc_comp_sign(float i, float inv_tol)
{
	float s = i*inv_tol;
	if(s >= 1.0f){ return 1.0f; }
	if(s <= -1.0f){ return -1.0f; }
	if(!(s == s)){ return 0.0f; }	/* NaN：不用 isnan 也不用 math.h */
	return s;
}

/* 一条桥臂补偿后的占空比，返回值已经落在 [lo, hi] 内。
 *   dtc     svm() 给的该桥臂占空比（本机 INVERT_DTC = 1，具体是高边还是低边当量
 *           对本文件不重要 —— 理由见上面【符号怎么定】）
 *   i       同一相的实测电流 A
 *   frac    死区的占空比当量 Ve/v_bus（参数 config.dt_comp；<= 0 = 关闭）
 *   inv_tol 1/config.dt_comp_tol
 *   lo, hi  调制限幅（DTC_MIN/DTC_MAX）：补偿不改变这个不变量
 *
 * 脉宽本来就短于死区的桥臂（占空比落在两端 frac 以内）根本没在开关，没有死区
 * 误差，不能补 —— 那也正是低边采样窗口判据要处理的那一相，别在这里再动它。
 * 钳位写成取反比较：输入是 NaN 时落到 lo，等价于"这一拍不补"，而不是把 NaN
 * 传下去（下游要转成 CCR/uint16_t）。 */
static inline float dtc_comp_duty(float dtc, float i, float frac, float inv_tol,
                                  float lo, float hi)
{
	if(!(frac > 0.0f)){ return dtc; }
	if((dtc <= frac) || (dtc >= 1.0f - frac)){ return dtc; }
	float d = dtc + dtc_comp_sign(i, inv_tol)*frac;
	if(!(d > lo)){ return lo; }
	if(d > hi){ return hi; }
	return d;
}

#endif /* INC_DTC_COMP_H_ */
