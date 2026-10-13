/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include "hw_config.h"
#include "user_config.h"
#include "math_ops.h"
#include "structs.h"
/* USER CODE END 0 */

uint32_t FDCAN_ram[512]; // 消息缓冲 RAM（512 words = 2 KB，标准帧滤波器/Rx FIFO/Tx FIFO 都从这里分配）
FDCAN_MsgRamType Node_msg;

FDCAN_InitType hcan1 = {
  .FrameFormat = FDCAN_FRAME_CLASSIC,
  .Mode = FDCAN_MODE_NORMAL,
  .Prescaler = 2,
  .SyncJumpWidth = 2,
  .TimeSeg1 = 34,
  .TimeSeg2 = 5, // 默认500K波特率
  .MsgRamStrAddr = (uint32_t)FDCAN_ram,
  .MsgRamOffset = 0,
  .pMsgInfo = &Node_msg,
  .RxFifo0Size = 8,
  .RxFifo0DataSize = FDCAN_DATA_BYTES_8,
  .RxFifo1Size = 0,
  .RxFifo1DataSize = FDCAN_DATA_BYTES_8,
  .RxBufferSize = 0,
  .RxBufferDataSize = FDCAN_DATA_BYTES_8,
  .TxBufferSize = 0,
  .TxBufferDataSize = FDCAN_DATA_BYTES_8,
  .TxFifoQueueSize = 8,
  .TxFifoQueueMode = FDCAN_TX_FIFO_MODE,
  .TxEventSize = 0,
  .AutoRetransmission = DISABLE,
  .TransmitPause = DISABLE,
  .ProtocolException = DISABLE,
};

/* CAN1 init function */
void MX_CAN1_Init(void)
{
  GPIO_InitType GPIO_InitStructure;
  NVIC_InitType NVIC_InitStructure;

  RCC_ConfigFDCANPllClk(RCC_FDCAN_PLLSRC_DIV6); // 6分频，240M / 6 = 40M
  RCC_ConfigFDCANClksrc(RCC_FDCAN_CLKSRC_PLL);

  RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_FDCAN2, ENABLE);
  RCC_EnableAPB1PeriphReset(RCC_APB1_PERIPH_FDCAN2);
  RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
  RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOB, ENABLE);

  GPIO_InitStructure.Pin       = CAN_RX_Pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_INPUT;
  GPIO_InitStructure.GPIO_Alternate = GPIO_AF_FDCAN2_RX;
  GPIO_InitStructure.GPIO_Pull  = GPIO_PULL_UP;
  GPIO_InitStructure.GPIO_Current = GPIO_DC_12mA;
  GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
  GPIO_InitPeripheral(CAN_RX_GPIO_Port, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin        = CAN_TX_Pin;
  GPIO_InitStructure.GPIO_Alternate = GPIO_AF_FDCAN2_TX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
  GPIO_InitPeripheral(CAN_TX_GPIO_Port, &GPIO_InitStructure);
  
  FDCAN_DeInit(FDCAN2);

  FDCAN_Init(FDCAN2, &hcan1);

  FDCAN_ConfigTSPrescaler(FDCAN2, FDCAN_TIMESTAMP_PRESC_16);
  FDCAN_Config_TS(FDCAN2, FDCAN_TIMESTAMP_DISABLE);

  NVIC_InitStructure.NVIC_IRQChannel                   = FDCAN2_INT0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* CAN RX is polled from the commutation interrupt (can_tx_rx), not interrupt-driven,
     * so the CAN1_RX0 interrupt is left disabled */
  // FDCAN_ConfigIntLine(FDCAN2, FDCAN_INT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
  // FDCAN_ActivateInt(FDCAN2, FDCAN_INT_RX_FIFO0_NEW_MESSAGE, FDCAN_TX_BUFFER0);

}

/* USER CODE BEGIN 1 */

void can_rx_init(CANRxMessage *msg){
  msg->filter.FilterIndex = 0;
	msg->filter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0; 	// set fifo assignment
	/* Classic (mask) filter with M_CAN semantics: the hardware matches
	 * (RxID & FilterID2) == (FilterID1 & FilterID2) on the standard 11 ID bits,
	 * where a 1 in FilterID2 means "this bit must match".  The 11-bit ID is
	 * (opcode << 7) | node_id, so masking only the low 7 bits accepts this
	 * node's frames for every opcode while ignoring everything addressed to
	 * another node.
	 * NOTE: the filter ID is the raw 11-bit standard ID here - the previous
	 * "CAN_ID << 5" was carried over from the STM32 bxCAN 16-bit filter
	 * register (FxR1[15:5]) and never matched this node's frames. */
	msg->filter.FilterID1=CAN_ID & 0x7F;			// this node's ID, in the 11-bit standard ID field
	msg->filter.FilterID2=0x7F;						// match the 7 node ID bits; the 4 opcode bits are "don't care"
	msg->filter.FilterType = FDCAN_FILTER_MASK;
	msg->filter.IdType = FDCAN_STANDARD_ID;
	FDCAN_ConfigFilter(FDCAN2, &msg->filter);
	/* Reject frames that do not match the filter above (and all remote frames).
	 * Without this the M_CAN default (ANFS = 00) accepts non-matching standard
	 * frames into FIFO 0, which would make the filter above a no-op: every node
	 * on the bus could put this controller in motor mode. */
	FDCAN_ConfigGlobalFilter(FDCAN2, FDCAN_REJECT_STD, FDCAN_REJECT_EXT,
	  FDCAN_REJECT_STD_REMOTE, FDCAN_REJECT_EXT_REMOTE);
}

void can_tx_init(CANTxMessage *msg){
  msg->tx_header.IdType = FDCAN_STANDARD_ID;
  msg->tx_header.ErrorState = FDCAN_ESI_PASSIVE;
  msg->tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  msg->tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  msg->tx_header.TxEventFifo = FDCAN_NO_TX_EVENTS;
  msg->tx_header.MsgMarker = 0x55;
  msg->tx_header.TxFrameType = FDCAN_DATA_FRAME;
	msg->tx_header.DataLength = 7 << FDCAN_ELEMENT_DLC_OFFSET; 			// message size of 7 byte
	msg->tx_header.ID = CAN_MASTER;  // recipient CAN ID
  msg->reply_var_a = CAN_VAR_POSITION;	// default telemetry reply until CAN_OP_SET_REPLY_FORMAT changes it
	msg->reply_var_b = CAN_VAR_VELOCITY;
}

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 7 8-bit words (DLC = 7)
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t, float vb){
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t, -(I_MAX+SENSE_BUFFER)*KT*GR, (I_MAX+SENSE_BUFFER)*KT*GR, 12);
    int vb_int = float_to_uint(vb, VB_MIN, VB_MAX, 8);
    msg->tx_header.ID = CAN_MASTER;
    msg->tx_header.DataLength = 7 << FDCAN_ELEMENT_DLC_OFFSET;
    msg->data[0] = id;
    msg->data[1] = p_int>>8;
    msg->data[2] = p_int&0xFF;
    msg->data[3] = v_int>>4;
    msg->data[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg->data[5] = t_int&0xFF;
    msg->data[6] = vb_int;
    }

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void unpack_cmd(CANRxMessage msg, float *commands){// ControllerStruct * controller){
        int p_int = (msg.data[0]<<8)|msg.data[1];
        int v_int = (msg.data[2]<<4)|(msg.data[3]>>4);
        int kp_int = ((msg.data[3]&0xF)<<8)|msg.data[4];
        int kd_int = (msg.data[5]<<4)|(msg.data[6]>>4);
        int t_int = ((msg.data[6]&0xF)<<8)|msg.data[7];

        commands[0] = uint_to_float(p_int, P_MIN, P_MAX, 16);
        commands[1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
        commands[2] = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
        commands[3] = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
        commands[4] = uint_to_float(t_int, -I_MAX*KT*GR, I_MAX*KT*GR, 12);
    //myprintf("Received   ");
    //myprintf("%.3f  %.3f  %.3f  %.3f  %.3f   %.3f", controller->p_des, controller->v_des, controller->kp, controller->kd, controller->t_ff, controller->i_q_ref);
    //myprintf("\n\r");
    }

/// Float Variable Read Reply ///
/// Response to a CAN_OP_READ_VAR request
/// 0:   [CAN ID]
/// 1:   [variable ID]
/// 2-5: [float32, little-endian]
void pack_float_reply(CANTxMessage *msg, uint8_t var_id, float value){
    msg->tx_header.ID = CAN_MASTER;
    msg->tx_header.DataLength = 6 << FDCAN_ELEMENT_DLC_OFFSET;
    msg->data[0] = CAN_ID;
    msg->data[1] = var_id;
    memcpy(&msg->data[2], &value, sizeof(float));	// Cortex-M4 is little-endian
    }

/// Float Variable Pair Read Reply ///
/// Response to a 3-byte CAN_OP_READ_VAR request: two variables in one frame.
/// 0-3: [float32_a, little-endian]
/// 4-7: [float32_b, little-endian]
/// No room for the CAN_ID/var_id echo - the master matches replies to requests by order
void pack_float_pair_reply(CANTxMessage *msg, float value_a, float value_b){
    msg->tx_header.ID = CAN_MASTER;
    msg->tx_header.DataLength = 8 << FDCAN_ELEMENT_DLC_OFFSET;
    memcpy(&msg->data[0], &value_a, sizeof(float));
    memcpy(&msg->data[4], &value_b, sizeof(float));
    }

/// Telemetry Reply ///
/// Reply to the ID-opcode protocol (opcodes 1-5): two variables as float32.
/// Sent with ID = (CAN_OP_TELEMETRY<<7) | CAN_ID, so the master identifies
/// the sender from the ID bits and the full payload carries data.
/// 0-3: [float32 var_a, little-endian]
/// 4-7: [float32 var_b, little-endian]
void pack_telemetry_reply(CANTxMessage *msg, uint8_t var_a, uint8_t var_b){
    float value_a = can_read_variable(var_a);
    float value_b = can_read_variable(var_b);
    msg->tx_header.ID = (CAN_OP_TELEMETRY<<7) | CAN_ID;
    msg->tx_header.DataLength = 8 << FDCAN_ELEMENT_DLC_OFFSET;
    memcpy(&msg->data[0], &value_a, sizeof(float));
    memcpy(&msg->data[4], &value_b, sizeof(float));
    }

/// Unpack two float32s from a command frame ///
/// Returns 0 (and leaves the outputs unusable) if either is NaN,
/// so garbage frames can't inject NaN into the control loop
int can_unpack_two_floats(CANRxMessage *msg, float *a, float *b){
    memcpy(a, &msg->data[0], sizeof(float));
    memcpy(b, &msg->data[4], sizeof(float));
    return !(isnan(*a) || isnan(*b));
    }

/// Look up a variable by ID for the float read protocol ///
/// Called from the commutation interrupt, so everything here must be fast
float can_read_variable(uint8_t var_id){
    switch(var_id){
        case CAN_VAR_POSITION:		return comm_encoder.angle_multiturn/GR;
        case CAN_VAR_VELOCITY:		return comm_encoder.velocity/GR;
        case CAN_VAR_I_Q:			return controller.i_q_filt;
        case CAN_VAR_I_D:			return controller.i_d_filt;
        case CAN_VAR_TORQUE:		return controller.i_q_filt*KT*GR;
        case CAN_VAR_V_BUS:			return controller.v_bus_filt;
        case CAN_VAR_V_Q:			return controller.v_q;
        case CAN_VAR_V_D:			return controller.v_d;
        case CAN_VAR_I_A:			return controller.i_a;
        case CAN_VAR_I_B:			return controller.i_b;
        case CAN_VAR_I_C:			return controller.i_c;
        case CAN_VAR_THETA_ELEC:	return comm_encoder.elec_angle;
        case CAN_VAR_FW_INT:		return controller.fw_int;
        case CAN_VAR_FAULT:			return (float)controller.fault;

        /* Read-only config and status.  All plain register/struct loads, so this
         * costs the interrupt nothing beyond the wider jump table */
        case CAN_VAR_P_MAX:			return P_MAX;
        case CAN_VAR_V_MAX:			return V_MAX;
        case CAN_VAR_KP_MAX:		return KP_MAX;
        case CAN_VAR_KD_MAX:		return KD_MAX;
        case CAN_VAR_I_MAX:			return I_MAX;
        case CAN_VAR_KT:			return KT;
        case CAN_VAR_GR:			return GR;
        case CAN_VAR_CAN_TIMEOUT:	return (float)CAN_TIMEOUT;
        case CAN_VAR_VERSION:		return VERSION_NUM;
        case CAN_VAR_MODE:			return (float)state.state;

        /* Bus health, read straight from the FDCAN error counter / protocol
         * status registers (ECR: TEC[7:0], REC[14:8]; PSR: LEC[2:0]) */
        case CAN_VAR_CAN_TEC:		return (float)(FDCAN2->ECR & FDCAN_ECR_TEC);
        case CAN_VAR_CAN_REC:		return (float)((FDCAN2->ECR & FDCAN_ECR_REC) >> 8);
        case CAN_VAR_CAN_LEC:		return (float)(FDCAN2->PSR & FDCAN_PSR_LEC);

        default:					return NAN;		// unknown ID reads back NaN
        }
    }

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
