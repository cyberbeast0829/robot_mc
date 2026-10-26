/*
 * can_proto.c
 *
 * CAN 协议分发。can_prot（config.can_prot）在开机时选定一张协议表；
 * 新协议只需增加一个 can_protocol_t 并在这里注册，不动硬件层。
 *
 * 调用位置：
 *   can_proto_init()  上电、硬件就绪之后（main.c）
 *   can_proto_rx()    40 kHz 控制中断，每个收到的帧一次
 *   can_proto_tick()  40 kHz 控制中断，内部产生 1 ms 时基（心跳/超时）
 *   can_proto_poll()  主循环（描述符流式发送这类慢活）
 */

#include "can.h"
#include "structs.h"

static const can_protocol_t *can_active_table = &can_proto_legacy;
static uint32_t can_tick_25us;

void can_proto_init(void)
{
    if (can_active_prot() == CAN_PROT_CYBERBEAST) {
        can_active_table = &can_proto_cyberbeast;
    } else {
        can_active_table = &can_proto_legacy;
    }

    can_tick_25us = 0;
    if (can_active_table->init) { can_active_table->init(); }
}

void can_proto_rx(const can_frame_t *f)
{
    if (can_active_table->rx) { can_active_table->rx(f); }
}

void can_proto_tick(void)
{
    /* 控制中断是精确的 25 us，40 次就是 1 ms */
    if (++can_tick_25us >= 40u) {
        can_tick_25us = 0;
        if (can_active_table->tick_1ms) { can_active_table->tick_1ms(); }
    }
}

void can_proto_poll(void)
{
    if (can_active_table->poll) { can_active_table->poll(); }
}
