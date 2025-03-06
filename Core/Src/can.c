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
#include "hw_config.h"
#include "user_config.h"
#include "math_ops.h"
/* USER CODE END 0 */

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

  FDCAN_ConfigIntLine(FDCAN2, FDCAN_INT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
  FDCAN_ActivateInt(FDCAN2, FDCAN_INT_RX_FIFO0_NEW_MESSAGE, FDCAN_TX_BUFFER0);

  FDCAN_Start(FDCAN2);

}

/* USER CODE BEGIN 1 */

void can_rx_init(CANRxMessage *msg){
	msg->filter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 	// set fifo assignment
	msg->filter.FilterIdHigh=CAN_ID<<5; 				// CAN ID
	msg->filter.FilterIdLow=0x0;
	msg->filter.FilterMaskIdHigh=0xFFF;
	msg->filter.FilterMaskIdLow=0;
	msg->filter.FilterMode = CAN_FILTERMODE_IDMASK;
	msg->filter.FilterScale=CAN_FILTERSCALE_32BIT;
	msg->filter.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&CAN_H, &msg->filter);
}

void can_tx_init(CANTxMessage *msg){
	msg->tx_header.DLC = 7; 			// message size of 7 byte
	msg->tx_header.IDE=CAN_ID_STD; 		// set identifier to standard
	msg->tx_header.RTR=CAN_RTR_DATA; 	// set data type to remote transmission request?
	msg->tx_header.StdId = CAN_MASTER;  // recipient CAN ID
}

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
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
    //printf("Received   ");
    //printf("%.3f  %.3f  %.3f  %.3f  %.3f   %.3f", controller->p_des, controller->v_des, controller->kp, controller->kd, controller->t_ff, controller->i_q_ref);
    //printf("\n\r");
    }

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
