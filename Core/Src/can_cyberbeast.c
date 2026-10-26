/*
 * can_cyberbeast.c
 *
 * CYBERBEAST 协议设备侧实现（裁剪版）。
 *
 * 帧格式、字节序与消息语义以 ODrive 仓库 docs/cyberbeast-protocol.md 为准：
 *   - 29-bit 扩展帧 ID: Priority(3) | MsgType(8) | Dest(8) | Source(8) | Seq(2)
 *   - ID 与各帧固定字段：大端；SDO 参数值 / JSON 描述符字段：小端
 *   - 控制：MIT(0x00/0x80)、TORQUE(0x03/0x83)、CURRENT(0x04)
 *     （CSP 0x01 / CSV 0x02 暂不支持：本固件没有独立的轨迹/速度环）
 *   - 查询 0x40~0x49、心跳 0x48、SDO 0x20/0x21、描述符 0x24/0x25
 *   - 系统 0x22/0x60~0x65、急停 0xC0
 *
 * 单位约定（与客户端一致，最容易出错的地方）：
 *   MIT 命令/响应：位置 rad、速度 rad/s、kp N·m/rad、kd N·m·s/rad、力矩 N·m
 *                  —— 全部是**输出端**，与本工程 torque_control() 同口径，1:1
 *   TORQUE/CURRENT：线上是**电机端** N·m / A（客户端已除以 gear_ratio），
 *                  本工程 t_ff 是输出端，所以必须 × GR
 *   心跳 pos/vel：**电机端 turns**（× 100 / × 10000 定点）
 *   QUERY_POS_VEL：电机端 turns、turns/s
 *
 * 实时性：rx/tick 在 40 kHz 控制中断里运行；描述符流式发送在主循环
 * （can_proto_poll）。这里不允许 printf / 写 flash / 阻塞。
 */

#include "can.h"
#include "can_cyberbeast.h"
#include "can_cyberbeast_codec.h"
#include "endpoint_defs.h"
#include "structs.h"
#include "user_config.h"
#include "math_ops.h"

#include <string.h>

/* ---- 协议常量 ---------------------------------------------------------- */

#define CB_PRI_CRITICAL     0
#define CB_PRI_EMERGENCY    1
#define CB_PRI_HIGH_CTRL    2
#define CB_PRI_CTRL         3
#define CB_PRI_CONFIG       4
#define CB_PRI_QUERY        5
#define CB_PRI_STATUS       6

#define CB_MSG_MIT_CONTROL          0x00
#define CB_MSG_POS_CONTROL          0x01
#define CB_MSG_VEL_CONTROL          0x02
#define CB_MSG_TORQUE_CONTROL       0x03
#define CB_MSG_CURRENT_CONTROL      0x04
#define CB_MSG_PARAM_READ           0x20
#define CB_MSG_PARAM_WRITE          0x21
#define CB_MSG_CONFIG_SAVE          0x22
#define CB_MSG_CONFIG_RESET         0x23
#define CB_MSG_JSON_DESC_READ       0x24
#define CB_MSG_JSON_DESC_DATA       0x25
#define CB_MSG_QUERY_STATUS         0x40
#define CB_MSG_QUERY_POS_VEL        0x41
#define CB_MSG_QUERY_CURRENT        0x42
#define CB_MSG_QUERY_TEMPERATURE    0x43
#define CB_MSG_QUERY_BUS            0x44
#define CB_MSG_QUERY_ERROR          0x45
#define CB_MSG_QUERY_DEVICE_INFO    0x46
#define CB_MSG_QUERY_POWER          0x47
#define CB_MSG_HEARTBEAT            0x48
#define CB_MSG_STATUS_FEEDBACK      0x49
#define CB_MSG_SET_NODE_ID          0x60
#define CB_MSG_SET_ZERO             0x61
#define CB_MSG_START_MOTOR          0x62
#define CB_MSG_STOP_MOTOR           0x63
#define CB_MSG_RESET_DEVICE         0x64
#define CB_MSG_CLEAR_ERRORS         0x65
#define CB_MSG_MIT_CONTROL_BCAST    0x80
#define CB_MSG_POS_CONTROL_BCAST    0x81
#define CB_MSG_VEL_CONTROL_BCAST    0x82
#define CB_MSG_TORQUE_CONTROL_BCAST 0x83
#define CB_MSG_ESTOP                0xC0
#define CB_MSG_FAULT_ALERT          0xC1

#define CB_ADDR_BROADCAST           0xFF
#define CB_MAX_BCAST_DEVICES        8

/* 模式状态（MIT 响应/心跳的 4-bit nibble） */
#define CB_MODE_CALIBRATING         1
#define CB_MODE_IDLE                2
#define CB_MODE_CLOSED_LOOP         3
#define CB_MODE_MIT                 4
#define CB_MODE_TORQUE              7

/* 心跳的控制环 nibble（ODrive 的 control_mode 编码） */
#define CB_CM_VOLTAGE               0
#define CB_CM_TORQUE                1
#define CB_CM_POSITION              3

/* 错误码 */
#define CB_ERR_NONE                 0x0
#define CB_ERR_VOLTAGE              0x4
#define CB_ERR_OVER_CURRENT         0x6
#define CB_ERR_CAN_TIMEOUT          0x9
#define CB_ERR_MULTIPLE             0xF

/* PARAM_READ 批量标志（本实现不支持批量，回 ERR 让主站退回单读） */
#define CB_PARAM_READ_FLAG_BATCH    0x40
#define CB_PARAM_READ_FLAG_ERR      0x20

/* PARAM_WRITE 分段写标志（Classic CAN 写 >4 字节参数） */
#define CB_PARAM_WRITE_FLAG_MORE    0x80

/* ---- ID 编解码 -------------------------------------------------------- */

static uint32_t cb_make_id(uint8_t pri, uint8_t msgtype, uint8_t dest, uint8_t src, uint8_t seq)
{
    return ((uint32_t)(pri & 0x7) << 26)
         | ((uint32_t)msgtype << 18)
         | ((uint32_t)dest << 10)
         | ((uint32_t)src << 2)
         | (uint32_t)(seq & 0x3);
}

static uint8_t cb_get_msgtype(uint32_t id) { return (uint8_t)((id >> 18) & 0xFF); }
static uint8_t cb_get_dest(uint32_t id)    { return (uint8_t)((id >> 10) & 0xFF); }
static uint8_t cb_get_source(uint32_t id)  { return (uint8_t)((id >> 2) & 0xFF); }
static uint8_t cb_get_seq(uint32_t id)     { return (uint8_t)(id & 0x3); }

/* ---- 模块状态 --------------------------------------------------------- */

static uint8_t  cb_master_id;       /* 从收到的帧里学到的 Source（心跳/应答目的地） */
static uint8_t  cb_life;            /* 心跳 Life 计数（3 bit） */
static uint8_t  cb_mode;            /* MOTOR_MODE 里最近一次控制帧对应的 ModeState */
static uint8_t  cb_ctrl_mode;       /* 心跳 Byte1 低 4 bit */
static uint32_t cb_now_ms;          /* 由 40 kHz 控制中断累加出来的 1 ms 时基 */
static uint32_t cb_last_hb_ms;

/* JSON 描述符传输。ISR 只写下面三个“请求”变量（32 位对齐写是原子的），
 * 主循环用序号判断有新请求，自己维护发送进度 —— 主循环里不需要关中断，
 * 也就不会为了发一帧而推迟 40 kHz 控制 ISR。 */
static volatile uint32_t cb_desc_req_seq;      /* ISR: 新请求序号 */
static volatile uint32_t cb_desc_req_offset;   /* ISR: 请求的起始偏移 */
static volatile uint8_t  cb_desc_req_master;   /* ISR: 请求者 */

static uint8_t  cb_desc_active;         /* 主循环私有状态 */
static uint8_t  cb_desc_metadata_sent;
static uint32_t cb_desc_offset;
static uint8_t  cb_desc_master;
static uint32_t cb_desc_gen;            /* 已采纳的请求序号 */

/* Classic CAN 写 >4 字节参数时的分段装配（只在 ISR 里访问） */
static struct{
    uint8_t  active;
    uint8_t  master;
    uint16_t ep;
    uint8_t  total;
    uint8_t  offset;
    uint8_t  buf[8];
} cb_wasm;

/* ---- 小工具 ----------------------------------------------------------- */

static bool cb_is_broadcast(uint32_t id) { return cb_get_msgtype(id) >= 0x80; }

static uint8_t cb_temp_u8(float c)
{
    float v = c + 50.0f;
    if (v != v) { return 0; }
    if (v < 0.0f) { return 0; }
    if (v > 255.0f) { return 255; }
    return (uint8_t)v;
}

static void cb_mit_range(cb_mit_range_t *r)
{
    float tau = I_MAX * KT * GR;        /* 输出端 N·m，与 legacy unpack_cmd 同量程 */
    float cur = I_MAX * GR;             /* 电机端 A：mit_max_torque / KT */
    r->pos_max = (P_MAX > 0.0f) ? P_MAX : 1.0f;
    r->vel_max = (V_MAX > 0.0f) ? V_MAX : 1.0f;
    r->kp_max  = (KP_MAX > 0.0f) ? KP_MAX : 1.0f;
    r->kd_max  = (KD_MAX > 0.0f) ? KD_MAX : 1.0f;
    r->tau_max = (tau > 0.0f) ? tau : 1.0f;
    r->cur_max = (cur > 0.0f) ? cur : 1.0f;
}

static bool cb_send(uint8_t pri, uint8_t msgtype, uint8_t dest, uint8_t seq,
                    const uint8_t *data, uint8_t len)
{
    if (CAN_ID <= 0) { return false; }
    return can_send_frame(cb_make_id(pri, msgtype, dest, (uint8_t)CAN_ID, seq),
                          FDCAN_EXTENDED_ID, data, len);
}

/* 该帧是否发给本节点：Dest 单播 / 位掩码广播 / 0xFF 全局广播 */
static bool cb_is_for_me(uint32_t id)
{
    uint8_t my = (uint8_t)CAN_ID;
    uint8_t dest = cb_get_dest(id);
    if (my == 0) { return false; }

    if (cb_is_broadcast(id)) {
        if (dest == CB_ADDR_BROADCAST) { return true; }
        if (my >= CB_MAX_BCAST_DEVICES) { return false; }
        return (dest & (1u << my)) != 0;
    }
    return dest == my;
}

static uint8_t cb_error_code(void)
{
    if (controller.fault & FAULT_ESTOP) { return CB_ERR_CAN_TIMEOUT; }
    if (controller.fault & FAULT_OV)    { return CB_ERR_VOLTAGE; }
    if (controller.fault & FAULT_OC)    { return CB_ERR_OVER_CURRENT; }
    if (controller.fault != 0)          { return CB_ERR_MULTIPLE; }
    /* 只有真正在闭环里才可能“命令超时”；出 MOTOR 后 timeout 计数保留旧值，
     * 不能拿它去骗上位机。CAN_TIMEOUT 是毫秒，计数器是控制周期（25 us）。 */
    if ((state.state == MOTOR_MODE) && (CAN_TIMEOUT > 0)
        && (controller.timeout > CAN_TIMEOUT_CYCLES)) {
        return CB_ERR_CAN_TIMEOUT;
    }
    return CB_ERR_NONE;
}

/* 心跳 Byte 18 是完整的 ODrive AxisState（不是上面的 4-bit ModeState）。
 * 客户端的"使能/失能完成"判定靠 ModeState，精确定位靠这个字节。 */
static uint8_t cb_axis_state_byte(void)
{
    switch (state.state) {
    case MOTOR_MODE:        return 8;      /* AXIS_STATE_CLOSED_LOOP_CONTROL */
    case CALIBRATION_MODE:  return 4;      /* AXIS_STATE_ENCODER_OFFSET_CALIBRATION */
    default:                return 1;      /* AXIS_STATE_IDLE */
    }
}

static uint8_t cb_mode_state_now(void)
{
    switch (state.state) {
    case MOTOR_MODE:        return cb_mode;         /* 4 = MIT, 7 = torque/current */
    case CALIBRATION_MODE:
    case ENCODER_MODE:      return CB_MODE_CALIBRATING;
    case FAULT_MODE:        return CB_MODE_IDLE;
    default:                return CB_MODE_IDLE;
    }
}

static uint8_t cb_ctrl_mode_now(void)
{
    return (state.state == MOTOR_MODE) ? cb_ctrl_mode : CB_CM_VOLTAGE;
}

/* ---- 应答 ------------------------------------------------------------- */

static void cb_send_mit_response(uint8_t dest, uint8_t seq)
{
    cb_mit_range_t r;
    uint8_t d[8];

    cb_mit_range(&r);
    cb_mit_pack_rsp(d, &r,
                    comm_encoder.angle_multiturn / GR,
                    comm_encoder.velocity / GR,
                    controller.i_q_filt,
                    cb_error_code(),
                    cb_mode_state_now(),
                    observer.temp_measured,
                    observer.temp_measured);    /* 本硬件没有 MOS 温度传感器 */

    (void)cb_send(CB_PRI_HIGH_CTRL, CB_MSG_MIT_CONTROL, dest, seq, d, 8);
}

static void cb_send_query(uint8_t msgtype, uint8_t dest, uint8_t seq,
                          const uint8_t *data, uint8_t len)
{
    (void)cb_send(CB_PRI_QUERY, msgtype, dest, seq, data, len);
}

/* ---- 实时控制 --------------------------------------------------------- */

static void cb_mit_control(const can_frame_t *f)
{
    uint8_t my = (uint8_t)CAN_ID;
    uint8_t src = cb_get_source(f->id);
    bool bcast = cb_is_broadcast(f->id);
    uint8_t slot;
    cb_mit_range_t r;
    float p, v, kp, kd, tau;

    if (bcast) {
        if (can_is_fd()) {
            /* 64 B 广播帧按槽位逐设备编码；Dest 位掩码里置位的设备读自己的槽 */
            if (my >= CB_MAX_BCAST_DEVICES) { return; }
            slot = my;
            if (f->len < (uint8_t)((slot + 1u) * 8u)) { return; }
        } else {
            /* Classic 8 B 广播：所有匹配设备应用同一命令 */
            if (f->len < 8u) { return; }
            slot = 0;
        }
    } else {
        if (f->len < 8u) { return; }
        slot = 0;
    }

    cb_mit_range(&r);
    cb_mit_unpack_cmd(&f->data[slot * 8u], &r, &p, &v, &kp, &kd, &tau);

    controller.p_des = p;
    controller.v_des = v;
    controller.kp = kp;
    controller.kd = kd;
    controller.t_ff = tau;
    controller.timeout = 0;

    cb_mode = CB_MODE_MIT;
    cb_ctrl_mode = CB_CM_POSITION;

    if (!bcast && (src != 0)) { cb_send_mit_response(src, cb_get_seq(f->id)); }
}

/* TORQUE(0x03) / CURRENT(0x04)：线上是电机端量，本工程 t_ff 是输出端。
 * 不加 kp/kd，否则上一帧 MIT 的刚度会继续作用 —— 那就不是力矩模式了。
 * respond：0x03 单播要回 MIT 响应，0x04 按协议不回。 */
static void cb_torque_like(const can_frame_t *f, float tau_out, bool respond)
{
    float lim = I_MAX * KT * GR;
    if (lim <= 0.0f) { lim = 1.0f; }
    if (tau_out > lim) { tau_out = lim; }
    if (tau_out < -lim) { tau_out = -lim; }

    controller.p_des = controller.theta_mech;
    controller.v_des = 0.0f;
    controller.kp = 0.0f;
    controller.kd = 0.0f;
    controller.t_ff = tau_out;
    controller.timeout = 0;

    cb_mode = CB_MODE_TORQUE;
    cb_ctrl_mode = CB_CM_TORQUE;

    if (respond && !cb_is_broadcast(f->id) && (cb_get_source(f->id) != 0)) {
        cb_send_mit_response(cb_get_source(f->id), cb_get_seq(f->id));
    }
}

static void cb_torque_control(const can_frame_t *f)
{
    if (f->len < 4u) { return; }
    float t_motor = cb_be_get_f32(&f->data[0]);
    if (t_motor != t_motor) { return; }
    cb_torque_like(f, t_motor * GR, true);          /* 电机端 N·m → 输出端 */
}

static void cb_current_control(const can_frame_t *f)
{
    if (f->len < 4u) { return; }
    float i_motor = cb_be_get_f32(&f->data[0]);
    if (i_motor != i_motor) { return; }
    cb_torque_like(f, i_motor * KT * GR, false);    /* 电机端 A → 输出端 N·m */
}

/* ---- 系统管理 --------------------------------------------------------- */

static void cb_start_motor(void)
{
    /* 只允许从菜单进入：request_fsm_state() 不做状态校验，直接写 next_state。
     * 客户端的使能序列是 CLEAR_ERRORS → START_MOTOR，两步之间状态机已经在
     * 25 us 内回到 MENU。 */
    if (state.state != MENU_MODE) { return; }
    zero_commands(&controller);
    controller.timeout = 0;
    request_fsm_state(&state, MOTOR_MODE);
}

static void cb_stop_motor(void)
{
    if (state.state != MOTOR_MODE) { return; }
    request_fsm_state(&state, MENU_MODE);
}

static void cb_reset_device(void)
{
    extern uint32_t _reboot_cookie;
    __asm volatile ("CPSID I\n\t":::"memory");
    /* 必须是普通复位：0xDEADBEEF 在 main.c 的 early_start_checks() 里是
     * “进入 DFU bootloader”的 cookie（那是旧协议 0xFF 特殊命令的语义）。
     * 42 表示正常启动，避免多一次复位。 */
    _reboot_cookie = 42;
    NVIC_SystemReset();
}

/* ---- 参数访问（SDO）--------------------------------------------------- */

static void cb_param_read(const can_frame_t *f)
{
    if (f->len < 4u) { return; }

    uint8_t flags = f->data[0];

    if (flags & CB_PARAM_READ_FLAG_BATCH) {
        /* 批量读没实现：明确回 ERR，主站会退回逐参数单读 */
        uint8_t rsp[2];
        rsp[0] = CB_PARAM_READ_FLAG_BATCH | CB_PARAM_READ_FLAG_ERR;
        rsp[1] = 0;
        (void)cb_send(CB_PRI_CONFIG, CB_MSG_PARAM_READ, cb_get_source(f->id),
                      cb_get_seq(f->id), rsp, 2);
        return;
    }

    uint16_t ep = (uint16_t)cb_be_get_u16(&f->data[1]);
    uint8_t req_len = f->data[3];
    uint32_t offset = 0;
    if (f->len >= 8u) { offset = cb_be_get_u32(&f->data[4]); }

    if (req_len == 0u) { req_len = 4; }             /* 旧客户端兼容 */
    if (req_len > 8u) { req_len = 8; }
    if (!can_is_fd() && (req_len > 4u)) { req_len = 4; }

    /* 输入游标必须是非空指针（长度 0），否则生成的解码器会从地址 0 读 */
    static const uint8_t empty = 0;
    uint8_t value[8] = {0};
    ep_cbuf_t in = { &empty, &empty };
    ep_buf_t out = { value, value + sizeof(value) };
    ep_status_t st = endpoint_handler(ep, EP_OP_READ, &in, &out);
    uint8_t full_len = (st == EP_OK) ? (uint8_t)(out.begin - value) : 0u;

    uint8_t rsp[12];
    uint8_t n = 0;
    rsp[0] = flags & 0x7Fu;
    rsp[1] = f->data[1];
    rsp[2] = f->data[2];

    if (offset < full_len) {
        uint8_t avail = (uint8_t)(full_len - offset);
        n = (req_len < avail) ? req_len : avail;
        memcpy(&rsp[4], &value[offset], n);
        if ((offset + n) < full_len) { rsp[0] |= 0x80u; }   /* More */
    }
    rsp[3] = n;

    (void)cb_send(CB_PRI_CONFIG, CB_MSG_PARAM_READ, cb_get_source(f->id),
                  cb_get_seq(f->id), rsp, (uint8_t)(4u + n));
}

static void cb_send_write_ack(uint8_t flags, uint16_t ep, uint8_t dest)
{
    uint8_t ack[8];
    ack[0] = flags;
    ack[1] = (uint8_t)(ep >> 8);
    ack[2] = (uint8_t)ep;
    ack[3] = 0;
    ack[4] = 0; ack[5] = 0; ack[6] = 0; ack[7] = 0;
    (void)cb_send(CB_PRI_CONFIG, CB_MSG_PARAM_WRITE, dest, 0, ack, 8);
}

/* 末块到齐后一次性写入。写失败（端点不存在/只读/状态不符/类型不符）只体现为
 * 值没变：协议本身的 ACK 不含成败位，主站需要读回校验。 */
static void cb_param_write_commit(uint16_t ep, const uint8_t *buf, uint8_t len)
{
    ep_cbuf_t in = { buf, buf + len };
    uint8_t dummy[8] = {0};
    ep_buf_t out = { dummy, dummy + sizeof(dummy) };
    (void)endpoint_handler(ep, EP_OP_WRITE, &in, &out);
}

static void cb_param_write_segmented(const can_frame_t *f)
{
    uint8_t flags = f->data[0];
    uint16_t ep = (uint16_t)cb_be_get_u16(&f->data[1]);
    uint8_t total = f->data[3];
    uint8_t src = cb_get_source(f->id);

    if ((total < 5u) || (total > 8u)) { cb_wasm.active = 0; return; }

    if (!cb_wasm.active || (cb_wasm.ep != ep) || (cb_wasm.master != src)) {
        cb_wasm.active = 1;
        cb_wasm.master = src;
        cb_wasm.ep = ep;
        cb_wasm.total = total;
        cb_wasm.offset = 0;
        memset(cb_wasm.buf, 0, sizeof(cb_wasm.buf));
    }

    if (cb_wasm.offset >= total) { cb_wasm.active = 0; return; }

    uint8_t remaining = (uint8_t)(total - cb_wasm.offset);
    if (remaining > 4u) { remaining = 4u; }
    memcpy(&cb_wasm.buf[cb_wasm.offset], &f->data[4], remaining);
    cb_wasm.offset = (uint8_t)(cb_wasm.offset + remaining);

    if (flags & CB_PARAM_WRITE_FLAG_MORE) {
        if (cb_wasm.offset >= total) { cb_wasm.active = 0; }     /* 填满却还声明 More */
        return;
    }

    /* 末块必须恰好填满，否则会拿清零缓冲的剩余字节去写（静默写入错误值） */
    uint8_t complete = (cb_wasm.offset == total);
    cb_wasm.active = 0;
    if (!complete) { return; }

    cb_param_write_commit(ep, cb_wasm.buf, total);
    cb_send_write_ack(flags, ep, src);
}

static void cb_param_write(const can_frame_t *f)
{
    if (f->len < 4u) { return; }

    uint16_t ep = (uint16_t)cb_be_get_u16(&f->data[1]);
    uint8_t data_len = f->data[3];

    if (!can_is_fd() && (data_len > 4u)) {
        cb_param_write_segmented(f);
        return;
    }
    if ((data_len > 8u) || (f->len < (uint8_t)(4u + data_len))) { return; }

    cb_wasm.active = 0;     /* 单帧写覆盖掉任何未装配完的分段写 */
    cb_param_write_commit(ep, &f->data[4], data_len);
    cb_send_write_ack(f->data[0], ep, cb_get_source(f->id));
}

/* ---- JSON 描述符 ------------------------------------------------------ */

static void cb_json_desc_read(const can_frame_t *f)
{
    if (f->len < 4u) { return; }

    /* 只登记请求；真正的分帧发送在主循环（cb_poll）。
     * 先写参数、最后写序号，保证主循环采纳时参数已经就位。 */
    cb_desc_req_offset = cb_le_get_u32(&f->data[0]);
    cb_desc_req_master = cb_get_source(f->id);
    cb_desc_req_seq++;
}

static bool cb_desc_send_one(void)
{
    uint8_t frame_len = can_is_fd() ? 64u : 8u;
    uint8_t d[64];
    uint8_t master = cb_desc_master;

    if (!cb_desc_metadata_sent) {
        memset(d, 0, frame_len);
        /* [00 00][total u32 LE][crc u16 LE] */
        cb_le_put_u32(&d[2], (uint32_t)endpoint_json_len);
        cb_le_put_u16(&d[6], endpoint_json_crc);
        if (!cb_send(CB_PRI_CONFIG, CB_MSG_JSON_DESC_DATA, master, 0, d, frame_len)) {
            return false;
        }
        cb_desc_metadata_sent = 1;
        return true;
    }

    if (cb_desc_offset >= endpoint_json_len) {
        cb_desc_active = 0;
        cb_desc_metadata_sent = 0;
        cb_desc_offset = 0;
        return false;
    }

    memset(d, 0, frame_len);
    uint16_t chunk = (uint16_t)cb_desc_offset;
    cb_le_put_u16(&d[0], chunk);

    uint8_t payload = (uint8_t)(frame_len - 2u);
    uint32_t left = (uint32_t)endpoint_json_len - cb_desc_offset;
    uint8_t n = (left < payload) ? (uint8_t)left : payload;
    memcpy(&d[2], &endpoint_json[cb_desc_offset], n);

    if (!cb_send(CB_PRI_CONFIG, CB_MSG_JSON_DESC_DATA, master, 0, d, frame_len)) {
        return false;
    }
    cb_desc_offset += n;
    return true;
}

/* ---- 心跳 ------------------------------------------------------------- */

static void cb_send_heartbeat(void)
{
    uint8_t d[20];
    uint8_t frame_len = can_is_fd() ? 19u : 8u;
    uint8_t my = (uint8_t)CAN_ID;
    uint8_t master = cb_master_id ? cb_master_id : ((CAN_MASTER > 0) ? (uint8_t)CAN_MASTER : 1u);

    uint8_t err_flags = 0;
    if (controller.fault != 0) { err_flags |= 0x01; }                       /* AXIS */
    if (controller.fault & FAULT_OC) { err_flags |= 0x02; }                 /* MOTOR */
    if (controller.fault & FAULT_OV) { err_flags |= 0x10; }                 /* BOARD */

    if (my == 0) { return; }

    memset(d, 0, sizeof(d));
    d[0] = (uint8_t)(((cb_life++ & 0x07u) << 5) | (err_flags & 0x1Fu));
    /* Byte 1 = AxisState(高 4 bit) | control_mode(低 4 bit)。AxisState 与 ODrive
     * 对齐（1=IDLE, 4=编码器标定, 8=CLOSED_LOOP），不是 ModeState nibble。 */
    d[1] = (uint8_t)(((cb_axis_state_byte() & 0x0Fu) << 4) | (cb_ctrl_mode_now() & 0x0Fu));
    d[2] = cb_temp_u8(observer.temp_measured);

    float pos_turns = comm_encoder.angle_multiturn * INV_TWO_PI_F;   /* 电机端 turns */
    float vel_turns = comm_encoder.velocity * INV_TWO_PI_F;          /* 电机端 turns/s */

    if (can_is_fd()) {
        d[3] = cb_temp_u8(observer.temp_measured);                  /* MOS：无传感器，用同一估计 */
        /* 母线电压 uint16、0.1 V/LSB（与 ODrive 心跳一致） */
        float vbus_v = controller.v_bus_filt * 10.0f;
        if (!(vbus_v > 0.0f)) { vbus_v = 0.0f; }
        if (vbus_v > 65535.0f) { vbus_v = 65535.0f; }
        cb_be_put_u16(&d[4], (uint16_t)vbus_v);
        cb_be_put_i16(&d[6], 0);                                    /* 母线电流未测量 */
        cb_be_put_i32(&d[8], cb_scale_i32(pos_turns, 10000.0f));
        cb_be_put_i32(&d[12], cb_scale_i32(vel_turns, 10000.0f));
        cb_be_put_i16(&d[16], cb_scale_i16(controller.i_q_filt, 100.0f));
        d[18] = cb_axis_state_byte();                              /* 完整状态字节 */
    } else {
        cb_be_put_i16(&d[3], cb_scale_i16(pos_turns, 100.0f));
        cb_be_put_i16(&d[5], cb_scale_i16(vel_turns, 100.0f));
        float iq = controller.i_q_filt * 2.0f;                      /* 0.5 A/bit */
        if (iq > 127.0f) { iq = 127.0f; }
        if (iq < -128.0f) { iq = -128.0f; }
        d[7] = (uint8_t)(int8_t)iq;
    }

    (void)cb_send(CB_PRI_STATUS, CB_MSG_HEARTBEAT, master, 0, d, frame_len);
}

/* ---- 查询 ------------------------------------------------------------- */

static void cb_query_error(const can_frame_t *f)
{
    uint8_t dest = cb_get_source(f->id);
    if ((dest == 0) || (f->len < 1u)) { return; }

    uint8_t type = f->data[0];
    uint32_t value = 0;
    switch (type) {
    case 0:  value = controller.fault & FAULT_OC; break;                    /* motor */
    case 3:  value = 0; break;                                              /* controller */
    case 4:  value = controller.fault & FAULT_OV; break;                    /* system */
    case 5:  value = controller.fault; break;                               /* axis */
    default: value = 0; break;
    }

    uint8_t d[8];
    memset(d, 0, sizeof(d));
    d[0] = type;
    cb_be_put_u32(&d[4], value);
    cb_send_query(CB_MSG_QUERY_ERROR, dest, cb_get_seq(f->id), d, 8);
}

static void cb_query_device_info(const can_frame_t *f)
{
    uint8_t dest = cb_get_source(f->id);
    if (dest == 0) { return; }

    uint8_t d[16];
    memset(d, 0, sizeof(d));
    uint32_t hw = 1u;
    /* ODrive 的编码是 (major<<16)|(minor<<8)|revision；VERSION_NUM 是 2.0f */
    uint32_t fw = ((uint32_t)VERSION_NUM << 16);
    cb_be_put_u32(&d[0], hw);
    cb_be_put_u32(&d[4], fw);
    /* FD 版尾部是 64 位序列号；本硬件没有，留 0 */
    cb_send_query(CB_MSG_QUERY_DEVICE_INFO, dest, cb_get_seq(f->id), d,
                  can_is_fd() ? 16u : 8u);
}

/* ---- 帧分发（ISR 上下文）--------------------------------------------- */

static void cb_rx(const can_frame_t *f)
{
    if (f->id_type != FDCAN_EXTENDED_ID) { return; }
    if (!cb_is_for_me(f->id)) { return; }

    uint8_t mt = cb_get_msgtype(f->id);
    uint8_t src = cb_get_source(f->id);
    uint8_t seq = cb_get_seq(f->id);

    if (src != 0) { cb_master_id = src; }

    switch (mt) {
    case CB_MSG_MIT_CONTROL:
    case CB_MSG_MIT_CONTROL_BCAST:
        cb_mit_control(f);
        break;

    case CB_MSG_TORQUE_CONTROL:
    case CB_MSG_TORQUE_CONTROL_BCAST:
        cb_torque_control(f);
        break;

    case CB_MSG_CURRENT_CONTROL:
        cb_current_control(f);
        break;

    /* CSP/CSV 暂不支持：静默忽略（客户端的 reject_or_clamp 会保证不发）。 */

    case CB_MSG_PARAM_READ:         cb_param_read(f); break;
    case CB_MSG_PARAM_WRITE:        cb_param_write(f); break;
    case CB_MSG_JSON_DESC_READ:     cb_json_desc_read(f); break;

    case CB_MSG_CONFIG_SAVE:
        if (!controller.gate_drive_enabled) {
            state.save_status = 0;
            state.save_pending = 1;
        }
        break;
    case CB_MSG_CONFIG_RESET:
        /* 没有“恢复出厂”的持久化格式，按复位处理会更安全 */
        cb_reset_device();
        break;

    case CB_MSG_QUERY_STATUS:
    case CB_MSG_STATUS_FEEDBACK:
        if (src != 0) { cb_send_mit_response(src, seq); }
        break;

    case CB_MSG_QUERY_POS_VEL:
        if (src != 0) {
            uint8_t d[8];
            cb_be_put_f32(&d[0], comm_encoder.angle_multiturn * INV_TWO_PI_F);
            cb_be_put_f32(&d[4], comm_encoder.velocity * INV_TWO_PI_F);
            cb_send_query(CB_MSG_QUERY_POS_VEL, src, seq, d, 8);
        }
        break;

    case CB_MSG_QUERY_CURRENT:
        if (src != 0) {
            uint8_t d[8];
            cb_be_put_f32(&d[0], controller.i_q_filt);
            cb_be_put_f32(&d[4], controller.i_d_filt);
            cb_send_query(CB_MSG_QUERY_CURRENT, src, seq, d, 8);
        }
        break;

    case CB_MSG_QUERY_TEMPERATURE:
        if (src != 0) {
            uint8_t d[8];
            cb_be_put_f32(&d[0], observer.temp_measured);
            cb_be_put_f32(&d[4], observer.temp_measured);
            cb_send_query(CB_MSG_QUERY_TEMPERATURE, src, seq, d, 8);
        }
        break;

    case CB_MSG_QUERY_BUS:
        if (src != 0) {
            uint8_t d[8];
            cb_be_put_f32(&d[0], controller.v_bus_filt);
            cb_be_put_f32(&d[4], 0.0f);                 /* 母线电流未测量 */
            cb_send_query(CB_MSG_QUERY_BUS, src, seq, d, 8);
        }
        break;

    case CB_MSG_QUERY_ERROR:
        cb_query_error(f);
        break;

    case CB_MSG_QUERY_DEVICE_INFO:
        cb_query_device_info(f);
        break;

    case CB_MSG_QUERY_POWER:
        if (src != 0) {
            uint8_t d[8];
            memset(d, 0, sizeof(d));
            cb_send_query(CB_MSG_QUERY_POWER, src, seq, d, 8);
        }
        break;

    case CB_MSG_SET_NODE_ID:
        /* 只改 RAM 里的 CAN_ID；滤波器/时序在下次上电重建（与 config.can_id 一致） */
        if ((f->len >= 1u) && (f->data[0] >= 1u) && (f->data[0] <= 127u)) {
            CAN_ID = f->data[0];
        }
        break;

    case CB_MSG_SET_ZERO:
        if (state.state == MENU_MODE) { state.zero_pending = 1; }
        break;

    case CB_MSG_START_MOTOR:        cb_start_motor(); break;
    case CB_MSG_STOP_MOTOR:         cb_stop_motor(); break;
    case CB_MSG_RESET_DEVICE:       cb_reset_device(); break;
    case CB_MSG_CLEAR_ERRORS:       request_fsm_state(&state, MENU_MODE); break;

    case CB_MSG_ESTOP:
        /* 锁存故障 → run_fsm() 下一拍强制进 FAULT_MODE（关栅），
         * 需要显式回菜单才清除 */
        controller.fault |= FAULT_ESTOP;
        break;

    case CB_MSG_FAULT_ALERT:
        /* 设备→主站的广播，本机不处理 */
        break;

    default:
        break;
    }
}

/* ---- 1 ms 时基 / 主循环 ---------------------------------------------- */

static void cb_tick_1ms(void)
{
    cb_now_ms++;

    if (CAN_HB_MS > 0) {
        if ((uint32_t)(cb_now_ms - cb_last_hb_ms) >= (uint32_t)CAN_HB_MS) {
            cb_last_hb_ms = cb_now_ms;
            cb_send_heartbeat();
        }
    }
}

/* 主循环：描述符流式发送。限速 2 帧/ms —— 50 帧/ms 的 64 B FD 帧会占满总线
 * （ODrive 实测），而 2 帧/ms 在 Classic 下约 50% 负载、9.3 KB 描述符约 0.8 s。 */
static uint32_t cb_poll_last_loop;

static void cb_poll(void)
{
    uint32_t now = (uint32_t)controller.loop_count;
    if ((uint32_t)(now - cb_poll_last_loop) < 40u) { return; }   /* 40 × 25 us = 1 ms */
    cb_poll_last_loop = now;

    /* 采纳 ISR 的新请求。先读序号、再读参数、再读一次序号：两次一致且
     * 与已采纳的不同才切换，这样即使 ISR 在中间又发来新请求，也只会
     * 多重启一次，不会把两代参数混在一起。 */
    uint32_t s1 = cb_desc_req_seq;
    uint32_t offset = cb_desc_req_offset;
    uint8_t  master = cb_desc_req_master;
    uint32_t s2 = cb_desc_req_seq;
    if ((s1 == s2) && (s1 != cb_desc_gen)) {
        cb_desc_gen = s1;
        cb_desc_offset = offset;
        cb_desc_master = master;
        cb_desc_metadata_sent = 0;
        cb_desc_active = 1;
    }

    if (!cb_desc_active) { return; }

    for (uint8_t i = 0; i < 2u; i++) {
        if (!cb_desc_send_one()) { break; }
    }
}

static void cb_init(void)
{
    cb_master_id = 0;
    cb_life = 0;
    cb_mode = CB_MODE_MIT;
    cb_ctrl_mode = CB_CM_POSITION;
    cb_now_ms = 0;
    cb_last_hb_ms = 0;

    cb_desc_active = 0;
    cb_desc_metadata_sent = 0;
    cb_desc_offset = 0;
    cb_desc_master = 0;
    cb_desc_gen = cb_desc_req_seq;      /* 不采纳初始化之前的请求 */

    memset(&cb_wasm, 0, sizeof(cb_wasm));
    cb_poll_last_loop = (uint32_t)controller.loop_count;
}

const can_protocol_t can_proto_cyberbeast = {
    .id = CAN_PROT_CYBERBEAST,
    .init = cb_init,
    .rx = cb_rx,
    .tick_1ms = cb_tick_1ms,
    .poll = cb_poll,
};
