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

uint32_t FDCAN_ram[512]; // 消息缓冲 RAM（512 words = 2 KB）。FD 布局：8×64B Rx + 8×64B Tx
                         // + 1 个标准或扩展滤波器 ≈ 291 words，仍有余量
FDCAN_MsgRamType Node_msg;

FDCAN_InitType hcan1;    // 由 MX_CAN1_Init() 按 CAN_PROT / CAN_BAUD 填好

/* 本板 FDCAN 内核时钟：RCC_FDCAN_PLLSRC_DIV6 于 240 MHz PLL → 40 MHz */
#define FDCAN_CLK_HZ            40000000u

/* 当前真正生效的物理层/协议。开机时由 CAN_PROT/CAN_BAUD 决定一次：消息 RAM
 * 布局、滤波器、NBTP/DBTP 都依赖它，所以写入参数只在下次上电生效。 */
static uint8_t  can_prot_running = CAN_PROT_LEGACY;
static uint32_t can_baud_running = 500000u;
static bool     can_fd_running   = false;

static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/* 仲裁域时序：40 个时间量子/位，prescaler = 40 MHz / (40 × baud)。
 * 1M→1、500k→2、250k→4、125k→8 …；采样点由 MX_CAN1_Init 按协议选
 * （旧协议 34/5 = 87.5%，CYBERBEAST 31/8 = 80%）。 */
static bool can_nominal_timing(uint32_t baud, uint32_t *prescaler){
        uint32_t denom = baud*40u;
        if((denom == 0u) || ((FDCAN_CLK_HZ % denom) != 0u)){return false;}
        uint32_t p = FDCAN_CLK_HZ/denom;
        if((p < 1u) || (p > 512u)){return false;}
        *prescaler = p;
        return true;
}

/* 数据域时序。40 MHz 只能精确产生少数几档，超出表就退回 Classic 1M 并让
 * system.can_fd 如实报告 0（不做“看起来是 FD、其实没跑起来”的静默降级）。 */
static bool can_data_timing(uint32_t baud, uint32_t *p, uint32_t *t1, uint32_t *t2, uint32_t *sjw){
        switch(baud){
        case 2000000u: *p = 1; *t1 = 15; *t2 = 4; *sjw = 1; return true;        // 20 tq, 80 %
        case 4000000u: *p = 1; *t1 = 7;  *t2 = 2; *sjw = 1; return true;        // 10 tq, 80 %
        case 5000000u: *p = 1; *t1 = 5;  *t2 = 2; *sjw = 1; return true;        //  8 tq, 75 %
        default: return false;
        }
}

/* CAN1 init function */
void MX_CAN1_Init(void)
{
  GPIO_InitType GPIO_InitStructure;
  uint32_t prescaler, data_prescaler = 1, data_tseg1 = 15, data_tseg2 = 4, data_sjw = 1;

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

  /* ---- 由持久化参数决定物理层 ----
   * 该函数在 config_apply_defaults()/preference_writer_load() 之后调用，
   * 否则读到的是默认值而不是客户配置。 */
  can_prot_running = (CAN_PROT == CAN_PROT_CYBERBEAST) ? CAN_PROT_CYBERBEAST : CAN_PROT_LEGACY;
  can_baud_running = (CAN_BAUD >= 50000) ? (uint32_t)CAN_BAUD : 500000u;
  if((can_prot_running == CAN_PROT_LEGACY) && (can_baud_running > CAN_FD_BAUD_THRESHOLD)){
          can_baud_running = CAN_FD_NOMINAL_BAUD;       // 旧协议只有 8 字节帧，不用 FD
  }

  /* can_baud 在 FD 下是**数据域**速率：先决定物理层，再算仲裁域时序。
   * （不能把 5M 当仲裁速率去做 40 MHz/(40×baud) 的整除检查——那会永远失败。） */
  can_fd_running = false;
  if(can_baud_running > CAN_FD_BAUD_THRESHOLD){
          if(can_data_timing(can_baud_running, &data_prescaler, &data_tseg1, &data_tseg2, &data_sjw)){
                  can_fd_running = true;
          }
          else{
                  /* 数据域速率不在 2M/4M/5M 表里 → 退回 Classic 1M，
                   * 由 system.can_fd = 0 如实反映 */
                  can_baud_running = CAN_FD_NOMINAL_BAUD;
          }
  }

  uint32_t nominal_baud = can_fd_running ? CAN_FD_NOMINAL_BAUD : can_baud_running;
  if(!can_nominal_timing(nominal_baud, &prescaler)){
          can_baud_running = 500000u;
          can_fd_running = false;
          (void)can_nominal_timing(can_baud_running, &prescaler);
  }

  hcan1.FrameFormat       = can_fd_running ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_CLASSIC;
  hcan1.Mode              = FDCAN_MODE_NORMAL;
  hcan1.Prescaler         = prescaler;
  hcan1.SyncJumpWidth     = 2;
  /* 旧协议保持原有 87.5% 采样点（34/5），不改变已在跑的客户总线；
   * CYBERBEAST 用 Vector/CiA 推荐的 80%（31/8，与 ODrive 参考实现一致）。 */
  hcan1.TimeSeg1          = (can_prot_running == CAN_PROT_LEGACY) ? 34u : 31u;
  hcan1.TimeSeg2          = (can_prot_running == CAN_PROT_LEGACY) ? 5u : 8u;
  hcan1.DataPrescaler     = data_prescaler;
  hcan1.DataSyncJumpWidth = data_sjw;
  hcan1.DataTimeSeg1      = data_tseg1;
  hcan1.DataTimeSeg2      = data_tseg2;
  hcan1.MsgRamStrAddr     = (uint32_t)FDCAN_ram;
  hcan1.MsgRamOffset      = 0;
  hcan1.pMsgInfo          = &Node_msg;
  /* 滤波器元素个数必须与 can_rx_init() 实际写入的元素匹配：FDCAN_InitMsgRam()
   * 用它们计算消息 RAM 布局，并且 LSS/LSE = 0 时写进去的滤波器元素不会生效。
   * 旧协议用 1 个标准滤波器；CYBERBEAST 用 1 个扩展滤波器。 */
  hcan1.StdFilterSize     = (can_prot_running == CAN_PROT_LEGACY) ? 1 : 0;
  hcan1.ExtFilterSize     = (can_prot_running == CAN_PROT_CYBERBEAST) ? 1 : 0;
  hcan1.RxFifo0Size       = 8;
  hcan1.RxFifo0DataSize   = can_fd_running ? FDCAN_DATA_BYTES_64 : FDCAN_DATA_BYTES_8;
  hcan1.RxFifo1Size       = 0;
  hcan1.RxFifo1DataSize   = FDCAN_DATA_BYTES_8;
  hcan1.RxBufferSize      = 0;
  hcan1.RxBufferDataSize  = FDCAN_DATA_BYTES_8;
  hcan1.TxBufferSize      = 0;
  hcan1.TxBufferDataSize  = can_fd_running ? FDCAN_DATA_BYTES_64 : FDCAN_DATA_BYTES_8;
  hcan1.TxFifoQueueSize   = 8;
  hcan1.TxFifoQueueMode   = FDCAN_TX_FIFO_MODE;
  hcan1.TxEventSize       = 0;
  /* 自动重发必须打开（CCCR.DAR = 0）：DAR = 1 时一帧只要没被应答就不会重发，
   * 上位机侧速率不匹配/暂时没上线时，TEC 很快涨到 255 → bus-off，之后连收都不收。 */
  hcan1.AutoRetransmission = ENABLE;
  hcan1.TransmitPause      = DISABLE;
  hcan1.ProtocolException  = ENABLE;

  FDCAN_DeInit(FDCAN2);

  FDCAN_Init(FDCAN2, &hcan1);

  /* CAN FD 数据域很快（5M 时 1 tq = 25 ns），收发器环回延迟（典型 100~250 ns）
   * 已经吃掉一个位时间里剩下的采样裕量。M_CAN 的解决方式是 TDC：发送端把
   * 自己评估 ACK / 位错误的采样点往后推 TDCO 个数据域 tq。
   * 不开它时（2026-11-03 台架）5M 完全不通：can_tec = 248、can_rec = 0，
   * 适配器一帧都收不到，而同一套代码 2M（20 tq，裕量 2.5 倍）一切正常。
   * 默认值与 ODrive 参考实现 / canable2-fw 一致：offset = filter = 数据域 tseg1。
   * DBTP/TDCR 只能在 init 模式写，所以必须放在 FDCAN_Init() 之后、FDCAN_Start() 之前。 */
  if(can_fd_running){
          int tdc = CAN_TDC;
          if(tdc < 0){tdc = (int)hcan1.DataTimeSeg1;}      /* -1 = 自动 */
          if(tdc > 31){tdc = 31;}
          if(tdc > 0){
                  FDCAN_ConfigTxDelayCompensation(FDCAN2, (uint32_t)tdc, (uint32_t)tdc);
                  FDCAN_EnableTxDelayCompensation(FDCAN2);
          }
  }

  FDCAN_ConfigTSPrescaler(FDCAN2, FDCAN_TIMESTAMP_PRESC_16);
  FDCAN_Config_TS(FDCAN2, FDCAN_TIMESTAMP_DISABLE);

  /* RX 由 40 kHz 控制中断轮询（can_rx_fetch），FDCAN 中断源保持关闭，
   * 所以这里也不去动 NVIC（main.c 的优先级表里 CAN_ISR 只是占位）。 */

  can_rx_init();          // 滤波器
  FDCAN_Start(FDCAN2);    // 收帧从 40 kHz 控制中断轮询
}

/* USER CODE BEGIN 1 */

/* 安装当前协议的验收滤波器。
 * 旧协议：标准 (mask) 滤波器，M_CAN 语义 (RxID & FilterID2) == (FilterID1 & FilterID2)。
 * 11 位 ID 是 (opcode<<7)|node_id，只掩低 7 位就等于“接受本节点所有 opcode”。
 * CYBERBEAST：29 位扩展帧，Dest 可能是本节点、位掩码广播或 0xFF 全局广播。
 * 掩码过滤器无法同时表达这三种情况，所以硬件接受全部扩展帧（掩码 0），
 * Dest 由软件过滤。1M FD 下满总线约 2k 帧/s，而 40 kHz 轮询可消化 40k 帧/s。 */
void can_rx_init(void)
{
	FDCAN_FilterType filter;
	memset(&filter, 0, sizeof(filter));
	filter.FilterIndex  = 0;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterType   = FDCAN_FILTER_MASK;

	if(can_prot_running == CAN_PROT_LEGACY){
		filter.IdType    = FDCAN_STANDARD_ID;
		filter.FilterID1 = CAN_ID & 0x7F;       // this node's ID, in the 11-bit standard ID field
		filter.FilterID2 = 0x7F;                // match the 7 node ID bits; opcode bits are "don't care"
	}
	else{
		filter.IdType    = FDCAN_EXTENDED_ID;
		filter.FilterID1 = 0;                   // accept every extended frame ...
		filter.FilterID2 = 0;                   // ... mask 0 = all bits "don't care"
	}
	FDCAN_ConfigFilter(FDCAN2, &filter);

	/* Reject frames that do not match (and all remote frames).  Without this the
	 * M_CAN default (ANFS = 00) accepts non-matching standard frames into FIFO 0,
	 * which would make the filter above a no-op. */
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

/* ---- 帧收发（协议无关） ------------------------------------------------ */

uint8_t can_active_prot(void){return can_prot_running;}
bool can_is_fd(void){return can_fd_running;}
uint32_t can_active_baud(void){return can_baud_running;}

/* 字节数 → DLC 编码（只有 0..8, 12, 16, 20, 24, 32, 48, 64 是合法长度） */
static uint32_t dlc_from_len(uint8_t len){
        if(len <= 8u){return (uint32_t)len << FDCAN_ELEMENT_DLC_OFFSET;}
        if(len <= 12u){return FDCAN_DLC_BYTES_12;}
        if(len <= 16u){return FDCAN_DLC_BYTES_16;}
        if(len <= 20u){return FDCAN_DLC_BYTES_20;}
        if(len <= 24u){return FDCAN_DLC_BYTES_24;}
        if(len <= 32u){return FDCAN_DLC_BYTES_32;}
        if(len <= 48u){return FDCAN_DLC_BYTES_48;}
        return FDCAN_DLC_BYTES_64;
}

/* 发一帧。FD 模式下所有帧都用 FD 格式（与 ODrive 一致），但 ≤8 字节的帧
 * 不启用 BRS：控制/查询帧没有必要付数据域提速的开销。
 * ⚠ Classic 模式下必须拒绝 >8 字节：Tx FIFO 元素只有 8 字节，硬塞 64 字节
 * 会踩坏消息 RAM。 */
bool can_send_frame(uint32_t id, uint32_t id_type, const uint8_t *data, uint8_t len){
        if(len > CAN_MAX_FRAME_BYTES){return false;}
        if(!can_fd_running && (len > 8u)){return false;}

        FDCAN_TxHeaderType h;
        memset(&h, 0, sizeof(h));
        h.IdType        = id_type;
        h.ID            = id;
        h.TxFrameType   = FDCAN_DATA_FRAME;
        h.ErrorState    = FDCAN_ESI_PASSIVE;
        h.TxEventFifo   = FDCAN_NO_TX_EVENTS;
        h.MsgMarker     = 0x55;
        h.DataLength    = dlc_from_len(len);
        h.FDFormat      = can_fd_running ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
        h.BitRateSwitch = (can_fd_running && (len > 8u)) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
        return FDCAN_AddMsgToTxFifoQ(FDCAN2, &h, (uint8_t *)data) == SUCCESS;
}

/* 旧协议用：把 CANTxMessage 里配好的 ID/DLC 发出去 */
bool can_send_legacy(CANTxMessage *msg){
        uint8_t len = DLCtoBytes[msg->tx_header.DataLength >> FDCAN_ELEMENT_DLC_OFFSET];
        return can_send_frame(msg->tx_header.ID, FDCAN_STANDARD_ID, msg->data, len);
}

/* 从 Rx FIFO0 取一帧。返回 false = 没有待处理帧。 */
bool can_rx_fetch(can_frame_t *f){
        FDCAN_RxHeaderType h;
        if(FDCAN_GetRxFifoFillLevel(FDCAN2, FDCAN_RX_FIFO0) <= 0){return false;}
        if(FDCAN_GetRxMsg(FDCAN2, FDCAN_RX_FIFO0, &h, f->data) != SUCCESS){return false;}
        if(h.RxFrameType != FDCAN_DATA_FRAME){return false;}
        f->id      = h.ID;
        f->id_type = h.IdType;
        f->len     = DLCtoBytes[h.DataLength >> FDCAN_ELEMENT_DLC_OFFSET];
        if(f->len > CAN_MAX_FRAME_BYTES){f->len = CAN_MAX_FRAME_BYTES;}
        return true;
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
void unpack_cmd(const can_frame_t *f, float *commands){// ControllerStruct * controller){
        int p_int = (f->data[0]<<8)|f->data[1];
        int v_int = (f->data[2]<<4)|(f->data[3]>>4);
        int kp_int = ((f->data[3]&0xF)<<8)|f->data[4];
        int kd_int = (f->data[5]<<4)|(f->data[6]>>4);
        int t_int = ((f->data[6]&0xF)<<8)|f->data[7];

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
int can_unpack_two_floats(const can_frame_t *f, float *a, float *b){
    memcpy(a, &f->data[0], sizeof(float));
    memcpy(b, &f->data[4], sizeof(float));
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
