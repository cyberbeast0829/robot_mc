/*
 * test_can_cyberbeast.c
 *
 * CYBERBEAST 编解码的 host 侧回归测试（不需要目标板）。
 *
 * 覆盖：
 *   - 大端读写（含 −12.5f 的已知字节序列）
 *   - MIT 命令/响应的位域与量程
 *   - 心跳定标：大位置不饱和、超量程不反号、NaN → 0
 *   - 定点编码的钳位（对照 ODrive F16/F32 两个已修缺陷）
 *
 * 运行：make hosttest
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "can_cyberbeast_codec.h"

static int failures;

#define CHECK(cond) do{ \
    if(!(cond)){ \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        failures++; \
    } \
}while(0)

#define CHECK_NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (tol))

static void test_big_endian(void)
{
    uint8_t b[4];

    cb_be_put_f32(b, -12.5f);
    CHECK(b[0] == 0xC1 && b[1] == 0x48 && b[2] == 0x00 && b[3] == 0x00);
    CHECK_NEAR(cb_be_get_f32(b), -12.5f, 0.0f);

    cb_be_put_u16(b, 0x1234);
    CHECK(b[0] == 0x12 && b[1] == 0x34);
    CHECK(cb_be_get_u16(b) == 0x1234u);
}

static void test_little_endian(void)
{
    uint8_t b[4];

    cb_le_put_u32(b, 0x01234567u);
    CHECK(b[0] == 0x67 && b[1] == 0x45 && b[2] == 0x23 && b[3] == 0x01);
    CHECK(cb_le_get_u32(b) == 0x01234567u);

    cb_le_put_u16(b, 0xABCDu);
    CHECK(b[0] == 0xCD && b[1] == 0xAB);
    CHECK(cb_le_get_u16(b) == 0xABCDu);
}

static void test_mit_roundtrip(void)
{
    cb_mit_range_t r = {
        .pos_max = 12.5f, .vel_max = 30.0f,
        .kp_max = 500.0f, .kd_max = 100.0f,
        .tau_max = 18.0f, .cur_max = 40.0f
    };
    uint8_t d[8];
    float p, v, kp, kd, tau;

    /* 用编码器生成一帧，再解回来：误差应在一个 LSB 内 */
    int p_i = cb_float_to_uint(-3.25f, -r.pos_max, r.pos_max, 16);
    int v_i = cb_float_to_uint(7.5f, -r.vel_max, r.vel_max, 12);
    int kp_i = cb_float_to_uint(120.0f, 0.0f, r.kp_max, 12);
    int kd_i = cb_float_to_uint(2.5f, 0.0f, r.kd_max, 12);
    int t_i = cb_float_to_uint(-1.75f, -r.tau_max, r.tau_max, 12);

    d[0] = (uint8_t)(p_i >> 8);
    d[1] = (uint8_t)p_i;
    d[2] = (uint8_t)(v_i >> 4);
    d[3] = (uint8_t)(((v_i & 0x0F) << 4) | ((kp_i >> 8) & 0x0F));
    d[4] = (uint8_t)kp_i;
    d[5] = (uint8_t)(kd_i >> 4);
    d[6] = (uint8_t)(((kd_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F));
    d[7] = (uint8_t)t_i;

    cb_mit_unpack_cmd(d, &r, &p, &v, &kp, &kd, &tau);

    CHECK_NEAR(p, -3.25f, 2.0f * r.pos_max / 65535.0f);
    CHECK_NEAR(v, 7.5f, 2.0f * r.vel_max / 4095.0f);
    CHECK_NEAR(kp, 120.0f, r.kp_max / 4095.0f);
    CHECK_NEAR(kd, 2.5f, r.kd_max / 4095.0f);
    CHECK_NEAR(tau, -1.75f, 2.0f * r.tau_max / 4095.0f);

    /* 中点：位置 0 用截断编码应为 0x7FFF（与 ODrive F16 的现状一致） */
    CHECK(cb_float_to_uint(0.0f, -r.pos_max, r.pos_max, 16) == 32767);
}

static void test_mit_response(void)
{
    cb_mit_range_t r = {
        .pos_max = 12.5f, .vel_max = 30.0f,
        .kp_max = 500.0f, .kd_max = 100.0f,
        .tau_max = 18.0f, .cur_max = 4.0f
    };
    uint8_t d[8];

    cb_mit_pack_rsp(d, &r, 1.0f, -2.0f, 1.0f, 0x6, 0x4, 25.0f, 30.0f);

    /* 速度低 4 bit 与错误码拼在 d[3]，电流低 4 bit 与模式拼在 d[5] */
    CHECK((d[3] & 0x0F) == 0x6);
    CHECK((d[5] & 0x0F) == 0x4);
    /* 温度 +50 */
    CHECK(d[6] == 75);
    CHECK(d[7] == 80);

    /* NaN 温度不能变成 0xFF/0x00 的随机值 */
    cb_mit_pack_rsp(d, &r, 0.0f, 0.0f, 0.0f, 0, 0, NAN, NAN);
    CHECK(d[6] == 0);
    CHECK(d[7] == 0);
}

static void test_clamp_encoding(void)
{
    /* 超量程必须饱和，不能回绕（ODrive F16 的反向满力矩缺陷） */
    int hi = cb_float_to_uint(1.0e9f, -12.5f, 12.5f, 16);
    int lo = cb_float_to_uint(-1.0e9f, -12.5f, 12.5f, 16);
    CHECK(hi == 65535);
    CHECK(lo == 0);

    /* NaN 返回中性值 0 */
    CHECK(cb_float_to_uint(NAN, -12.5f, 12.5f, 16) == 0);

    /* 空量程不能除零 */
    CHECK(cb_float_to_uint(0.0f, 0.0f, 0.0f, 12) == 0);
}

static void test_heartbeat_scaling(void)
{
    /* F32 回归：88 电机端 turns 必须无损编码成 880000，而不是饱和到 214748 */
    CHECK(cb_scale_i32(88.0f, 10000.0f) == 880000);
    CHECK(cb_scale_i32(-88.0f, 10000.0f) == -880000);

    /* 满量程附近饱和到 INT32_MAX/MIN，绝不反号 */
    CHECK(cb_scale_i32(1.0e9f, 10000.0f) == INT32_MAX);
    CHECK(cb_scale_i32(-1.0e9f, 10000.0f) == INT32_MIN);

    /* NaN 是“未初始化”，给 0，不给 INT32_MIN */
    CHECK(cb_scale_i32(NAN, 10000.0f) == 0);

    /* Classic 的 int16 心跳（对称量程 ±32767，与 ODrive 的
     * scale_symmetric() 一致；不要期待 −32768） */
    CHECK(cb_scale_i16(1.0f, 100.0f) == 100);
    CHECK(cb_scale_i16(10000.0f, 100.0f) == 32767);
    CHECK(cb_scale_i16(-10000.0f, 100.0f) == -32767);
    CHECK(cb_scale_i16(NAN, 100.0f) == 0);
}

int main(void)
{
    test_big_endian();
    test_little_endian();
    test_mit_roundtrip();
    test_mit_response();
    test_clamp_encoding();
    test_heartbeat_scaling();

    if (failures) {
        printf("test_can_cyberbeast: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_can_cyberbeast: all tests passed\n");
    return 0;
}
