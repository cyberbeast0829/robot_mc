/*
 * can_mit.c
 *
 * 原有 11-bit 标准帧协议（can_prot = 0，默认）。这是从
 * n32h47x_48x_it.c 的 can_tx_rx() 原样搬出来的处理体：帧格式、超时语义、
 * 特殊命令（0xFC/0xFD/0xFE/0xFF）都保持不变，只是接收缓冲换成了协议无关的
 * can_frame_t，发送走 can_send_legacy()。
 *
 * 协议规格见 can.h 的注释表。
 */

#include "can.h"
#include "structs.h"
#include "user_config.h"
#include "math_ops.h"

#include <string.h>

extern uint32_t _reboot_cookie;

static void can_mit_rx(const can_frame_t *f)
{
    /* 旧协议只走 11-bit 标准帧；CYBERBEAST 的扩展帧即使被同一过滤器收进来，
     * 也由协议层各自丢弃 */
    if (f->id_type != FDCAN_STANDARD_ID) { return; }

    /* The 11-bit CAN ID is (opcode << 7) | node_id - see the table in can.h.
     * The hardware filter only accepts this node's ID, so just dispatch on
     * the opcode.  Malformed frames (wrong length, NaN floats) are ignored */
    uint32_t opcode = CAN_OPCODE(f->id);
    uint32_t dlc = f->len;
    int send_telemetry_reply = 0;
    float f_a, f_b;

    switch (opcode) {

    case CAN_OP_LEGACY:
        if (dlc == 8) {
            /* Legacy protocol: all 8-byte frames.  Reply with the packed telemetry */
            pack_reply(&can_tx, CAN_ID, comm_encoder.angle_multiturn / GR, comm_encoder.velocity / GR,
                       controller.i_q_filt * KT * GR, controller.v_bus_filt);
            can_send_legacy(&can_tx);

            /* Check for special Commands: bytes 0-6 all 0xFF, command in byte 7 */
            int special = (f->data[0] & f->data[1] & f->data[2] & f->data[3]
                         & f->data[4] & f->data[5] & f->data[6]) == 0xFF;
            if (special && (f->data[7] == 0xFC)) {
                update_fsm(&state, MOTOR_CMD);
            } else if (special && (f->data[7] == 0xFD)) {
                update_fsm(&state, MENU_CMD);
            } else if (special && (f->data[7] == 0xFE)) {
                update_fsm(&state, ZERO_CMD);
            } else if (special && (f->data[7] == 0xFF)) {
                __asm volatile ("CPSID I\n\t":::"memory");   // disable interrupts
                _reboot_cookie = 0xDEADBEEF;
                NVIC_SystemReset();
            } else {
                unpack_cmd(f, controller.commands);           // Unpack commands
                controller.timeout = 0;                       // Reset timeout counter
            }
        } else if ((dlc == 2) && (f->data[0] == CAN_LEGACY_READ_VAR)) {
            /* Legacy single float read */
            pack_float_reply(&can_tx, f->data[1], can_read_variable(f->data[1]));
            can_send_legacy(&can_tx);
        } else if ((dlc == 3) && (f->data[0] == CAN_LEGACY_READ_VAR)) {
            /* Legacy float pair read */
            pack_float_pair_reply(&can_tx, can_read_variable(f->data[1]), can_read_variable(f->data[2]));
            can_send_legacy(&can_tx);
        }
        break;

    case CAN_OP_SET_PV:
        if ((dlc == 8) && can_unpack_two_floats(f, &f_a, &f_b)) {
            /* 软限位规则见 foc.h/foc.c 的 pos_cmd_limit()：界内钳到限位，
             * 界外放宽到当前位置（只能往界内给目标） */
            controller.p_des = pos_cmd_limit(f_a);
            controller.v_des = fast_fmaxf(fast_fminf(f_b, V_MAX), V_MIN);
            controller.timeout = 0;
            send_telemetry_reply = 1;
        }
        break;

    case CAN_OP_SET_GAINS:
        if ((dlc == 8) && can_unpack_two_floats(f, &f_a, &f_b)) {
            controller.kp = fast_fmaxf(fast_fminf(f_a, KP_MAX), KP_MIN);
            controller.kd = fast_fmaxf(fast_fminf(f_b, KD_MAX), KD_MIN);
            controller.timeout = 0;
            send_telemetry_reply = 1;
        }
        break;

    case CAN_OP_SET_TFF:
        if (dlc == 4) {
            memcpy(&f_a, &f->data[0], sizeof(float));
            if (!isnan(f_a)) {
                controller.t_ff = fast_fmaxf(fast_fminf(f_a, TORQUE_FS), -TORQUE_FS);
                controller.timeout = 0;
                send_telemetry_reply = 1;
            }
        }
        break;

    case CAN_OP_SET_REPLY_FORMAT:
        /* Select which two variables come back in telemetry replies.
         * Stored in RAM - re-send after a power cycle.  Doesn't reset the
         * command timeout.  The reply acts as an ack, in the new format */
        if (dlc == 2) {
            can_tx.reply_var_a = f->data[0];
            can_tx.reply_var_b = f->data[1];
            send_telemetry_reply = 1;
        }
        break;

    case CAN_OP_READ_VARS:
        /* One-shot read of one or two variables.  Doesn't change the
         * configured reply format, and doesn't reset the command timeout -
         * telemetry polling shouldn't keep a stale command alive */
        if (dlc == 2) {
            pack_telemetry_reply(&can_tx, f->data[0], f->data[1]);
            can_send_legacy(&can_tx);
        } else if (dlc == 1) {
            pack_telemetry_reply(&can_tx, f->data[0], 0xFF);      // second slot reads back NaN
            can_send_legacy(&can_tx);
        }
        break;

    case CAN_OP_CONTROL:
        if (dlc == 1) {
            if (f->data[0] == CAN_CTRL_MOTOR_MODE) { update_fsm(&state, MOTOR_CMD); }
            else if (f->data[0] == CAN_CTRL_MENU_MODE) { update_fsm(&state, MENU_CMD); }
            else if (f->data[0] == CAN_CTRL_ZERO) { update_fsm(&state, ZERO_CMD); }
        }
        break;

    /* Unknown opcodes are ignored */
    }

    if (send_telemetry_reply) {
        pack_telemetry_reply(&can_tx, can_tx.reply_var_a, can_tx.reply_var_b);
        can_send_legacy(&can_tx);
    }
}

const can_protocol_t can_proto_legacy = {
    .id = CAN_PROT_LEGACY,
    .init = NULL,
    .rx = can_mit_rx,
    .tick_1ms = NULL,
    .poll = NULL,
};
