/*
 * can_cyberbeast_codec.h
 *
 * CYBERBEAST 协议的纯编解码函数：大端读写、MIT 8 字节位域、心跳定标。
 *
 * 这里刻意不包含任何固件头文件，host 侧测试（tools/test_can_cyberbeast.c）
 * 可以直接包含它。固件与测试共用同一份实现，避免“测试的是另一个模型”。
 *
 * ⚠ 两条从 ODrive 移植时踩过的硬规矩（见 cyberbeast-protocol.md F16/F32）：
 *   1. 定点编码必须**先钳位再转换**。float32 只有 24 位有效位，
 *      `2147483647.0f` 会被舍入成 2^31，直接转 int32 是 UB（实测回绕成负数）。
 *      所以定标函数在浮点域钳一次、转换后再在整型域钳一次。
 *   2. 钳位界必须与**被钳的量同量纲**（原始计数域），不要写“乘完倍率之后”
 *      还应该用计数域的值这种混合写法 —— 那正是 F32 把量程缩小 10000 倍的原因。
 */

#ifndef CAN_CYBERBEAST_CODEC_H
#define CAN_CYBERBEAST_CODEC_H

#include <stdint.h>
#include <string.h>

/* ---- 大端（Motorola）读写：ID 与各帧固定字段用大端 ---- */

static inline uint16_t cb_be_get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static inline uint32_t cb_be_get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}

static inline void cb_be_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static inline void cb_be_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

/* Cortex-M4 小端：先取位模式，再按大端写出 */
static inline float cb_be_get_f32(const uint8_t *p)
{
    uint32_t raw = cb_be_get_u32(p);
    float f;
    memcpy(&f, &raw, sizeof(f));
    return f;
}

static inline void cb_be_put_f32(uint8_t *p, float v)
{
    uint32_t raw;
    memcpy(&raw, &v, sizeof(raw));
    cb_be_put_u32(p, raw);
}

/* ---- 小端读写：SDO 参数值与 JSON 描述符字段用小端 ---- */

static inline uint16_t cb_le_get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[1] << 8) | p[0]);
}

static inline uint32_t cb_le_get_u32(const uint8_t *p)
{
    return ((uint32_t)p[3] << 24) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[1] << 8)  |  (uint32_t)p[0];
}

static inline void cb_le_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static inline void cb_le_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/* ---- 定点映射（与 math_ops.c 的 float_to_uint/uint_to_float 同式）---- */

/* 有符号/无符号浮点 → 定点。先钳位；NaN 返回 0（与位域掩码后得到一个
 * 看似合法的错误值相比，0 至少是“中性值”，且不会 UB）。 */
static inline int32_t cb_float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (!(x_max > x_min)) { return 0; }
    if (x != x) { return 0; }
    if (x < x_min) { x = x_min; }
    else if (x > x_max) { x = x_max; }
    float span = x_max - x_min;
    return (int32_t)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

static inline float cb_uint_to_float(int32_t x_int, float x_min, float x_max, int bits)
{
    if (!(x_max > x_min)) { return x_min; }
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

/* ---- MIT 编解码 -------------------------------------------------------- */

typedef struct{
    float pos_max;      /* 输出端 rad */
    float vel_max;      /* 输出端 rad/s */
    float kp_max;       /* N·m/rad（输出端） */
    float kd_max;       /* N·m·s/rad（输出端） */
    float tau_max;      /* N·m（输出端） */
    float cur_max;      /* A（电机端，响应帧电流量程） */
} cb_mit_range_t;

/* 8 字节命令（每设备一个槽位）：
 *   [0-1] pos int16 BE   [2-3] vel 12b | kp 高 4b
 *   [4]   kp 低 8b       [5-6] kd 12b | tau 高 4b   [7] tau 低 8b
 * 所有量都是输出端单位。 */
static inline void cb_mit_unpack_cmd(const uint8_t *d, const cb_mit_range_t *r,
                                     float *pos, float *vel, float *kp,
                                     float *kd, float *tau)
{
    int32_t p_int  = ((int32_t)d[0] << 8) | d[1];
    int32_t v_int  = ((int32_t)d[2] << 4) | (d[3] >> 4);
    int32_t kp_int = ((int32_t)(d[3] & 0x0F) << 8) | d[4];
    int32_t kd_int = ((int32_t)d[5] << 4) | (d[6] >> 4);
    int32_t t_int  = ((int32_t)(d[6] & 0x0F) << 8) | d[7];

    *pos = cb_uint_to_float(p_int,  -r->pos_max, r->pos_max, 16);
    *vel = cb_uint_to_float(v_int,  -r->vel_max, r->vel_max, 12);
    *kp  = cb_uint_to_float(kp_int,  0.0f,       r->kp_max,  12);
    *kd  = cb_uint_to_float(kd_int,  0.0f,       r->kd_max,  12);
    *tau = cb_uint_to_float(t_int,  -r->tau_max, r->tau_max, 12);
}

/* 8 字节响应：
 *   [0-1] pos int16 BE  [2-3] vel 12b | err 4b
 *   [4-5] cur 12b | mode 4b  [6] motor temp  [7] mos temp
 * 温度编码 = 实际 °C + 50（0 → −50 °C，255 → +205 °C）。 */
static inline void cb_mit_pack_rsp(uint8_t *d, const cb_mit_range_t *r,
                                   float pos, float vel, float cur,
                                   uint8_t err, uint8_t mode,
                                   float motor_temp, float mos_temp)
{
    int32_t p_int = cb_float_to_uint(pos, -r->pos_max, r->pos_max, 16);
    int32_t v_int = cb_float_to_uint(vel, -r->vel_max, r->vel_max, 12);
    int32_t c_int = cb_float_to_uint(cur, -r->cur_max, r->cur_max, 12);

    float mt = motor_temp + 50.0f;
    float ft = mos_temp + 50.0f;
    if (!(mt == mt)) { mt = 0.0f; }
    if (!(ft == ft)) { ft = 0.0f; }
    if (mt < 0.0f) { mt = 0.0f; } else if (mt > 255.0f) { mt = 255.0f; }
    if (ft < 0.0f) { ft = 0.0f; } else if (ft > 255.0f) { ft = 255.0f; }

    d[0] = (uint8_t)(p_int >> 8);
    d[1] = (uint8_t)p_int;
    d[2] = (uint8_t)(v_int >> 4);
    d[3] = (uint8_t)(((v_int & 0x0F) << 4) | (err & 0x0F));
    d[4] = (uint8_t)(c_int >> 4);
    d[5] = (uint8_t)(((c_int & 0x0F) << 4) | (mode & 0x0F));
    d[6] = (uint8_t)mt;
    d[7] = (uint8_t)ft;
}

/* ---- 心跳定标（int16 / int32 原始计数域钳位）------------------------- */

/* int32 原始计数的最大幅值取 2^31（不是 2147483647.0f —— float32 会把它
 * 舍入成 2^31，转换回 int32 是 UB）。 */
#define CB_INT32_RAW_LIMIT  2147483648.0f
#define CB_INT16_RAW_LIMIT  32767.0f

/* 四舍五入（远离 0），不依赖 roundf：Keil/AC6 的 microlib 不保证提供它。
 * 先转成 int64 再转回，避免 2^31 附近的浮点→int32 越界（UB）。 */
static inline int64_t cb_round_i64(float x)
{
    return (x >= 0.0f) ? (int64_t)(x + 0.5f) : (int64_t)(x - 0.5f);
}

static inline int32_t cb_scale_i32(float value, float scale)
{
    if (value != value) { return 0; }
    float lo = -CB_INT32_RAW_LIMIT / scale;
    float hi =  CB_INT32_RAW_LIMIT / scale;
    if (value < lo) { value = lo; }
    if (value > hi) { value = hi; }
    int64_t q = cb_round_i64(value * scale);
    if (q > INT32_MAX) { q = INT32_MAX; }
    if (q < INT32_MIN) { q = INT32_MIN; }
    return (int32_t)q;
}

static inline int16_t cb_scale_i16(float value, float scale)
{
    if (value != value) { return 0; }
    float lo = -CB_INT16_RAW_LIMIT / scale;
    float hi =  CB_INT16_RAW_LIMIT / scale;
    if (value < lo) { value = lo; }
    if (value > hi) { value = hi; }
    int32_t q = (int32_t)cb_round_i64(value * scale);
    if (q > 32767) { q = 32767; }
    if (q < -32768) { q = -32768; }
    return (int16_t)q;
}

static inline void cb_be_put_i32(uint8_t *p, int32_t v) { cb_be_put_u32(p, (uint32_t)v); }
static inline void cb_be_put_i16(uint8_t *p, int16_t v) { cb_be_put_u16(p, (uint16_t)v); }

#endif /* CAN_CYBERBEAST_CODEC_H */
