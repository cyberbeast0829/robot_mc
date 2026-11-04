/*
 * endpoint_serial.c
 *
 * USB CDC 上的 endpoint 帧收发：中断里只累积字节，主循环里解析和应答。
 * 帧格式见 endpoint.h。
 */

#include <string.h>
#include "endpoint_serial.h"
#include "endpoint.h"
#include "endpoint_defs.h"
#include "main.h"
#include "usb_pwr.h"

typedef enum{
	EP_RX_IDLE,
	EP_RX_LEN,
	EP_RX_HDR_CRC,
	EP_RX_PAYLOAD,
	EP_RX_CRC_HI,
	EP_RX_CRC_LO
}ep_rx_state_t;

static ep_rx_state_t rx_state = EP_RX_IDLE;
static uint8_t rx_header[2];
static uint8_t rx_len;
static uint8_t rx_pos;
static uint8_t rx_work[EP_MAX_PAYLOAD];
static uint16_t rx_crc;

/* 中断写完、主循环读走，用 volatile 防止编译器把轮询优化掉。
 * req_pending 为 1 期间中断不会碰 req_buf，主循环可以安全地做快照。 */
static uint8_t req_buf[EP_MAX_PAYLOAD];
static volatile uint16_t req_len;
static volatile uint8_t req_pending;

/* 主循环专用，避免在栈上放两帧缓冲 */
static uint8_t req_snapshot[EP_MAX_PAYLOAD];
static uint8_t tx_frame[EP_MAX_PAYLOAD + 5];

bool endpoint_serial_feed(uint8_t byte){
	switch(rx_state){
		case EP_RX_IDLE:
			if(byte != EP_FRAME_PREFIX){return false;}	// 不是帧头，交给 ASCII 菜单
			rx_header[0] = byte;
			rx_state = EP_RX_LEN;
			return true;

		case EP_RX_LEN:
			rx_header[1] = byte;
			rx_len = byte;
			if((rx_len == 0) || (rx_len > EP_MAX_PAYLOAD)){
				rx_state = EP_RX_IDLE;
			}
			else{
				rx_state = EP_RX_HDR_CRC;
			}
			return true;

		case EP_RX_HDR_CRC:
			if(byte != ep_crc8(EP_CRC8_INIT, rx_header, 2)){
				rx_state = EP_RX_IDLE;		// 帧头坏了，丢弃已收的 3 字节后重新同步
			}
			else{
				rx_pos = 0;
				rx_state = EP_RX_PAYLOAD;
			}
			return true;

		case EP_RX_PAYLOAD:
			rx_work[rx_pos++] = byte;
			if(rx_pos >= rx_len){rx_state = EP_RX_CRC_HI;}
			return true;

		case EP_RX_CRC_HI:
			rx_crc = (uint16_t)byte << 8;	// fibre 的 CRC16 是先高字节
			rx_state = EP_RX_CRC_LO;
			return true;

		case EP_RX_CRC_LO:
			rx_crc |= byte;
			/* 上一帧还没被主循环取走时，新帧直接丢弃：主机是单请求串行的 */
			if((ep_crc16(EP_CRC16_INIT, rx_work, rx_len) == rx_crc) && !req_pending){
				memcpy(req_buf, rx_work, rx_len);
				req_len = rx_len;
				__asm volatile ("" ::: "memory");	// req_buf/req_len 写完后再置标志
				req_pending = 1;
			}
			rx_state = EP_RX_IDLE;
			return true;
	}

	rx_state = EP_RX_IDLE;
	return false;
}

static void ep_send_frame(const uint8_t *payload, uint16_t len){
	/* USB 可能在处理和应答之间掉线；console_write_raw() 此时会退回 USART3，
	 * 二进制帧会污染 ASCII 菜单 */
	if(bDeviceState != CONFIGURED){return;}

	uint8_t header[2];
	header[0] = EP_FRAME_PREFIX;
	header[1] = (uint8_t)len;

	tx_frame[0] = header[0];
	tx_frame[1] = header[1];
	tx_frame[2] = ep_crc8(EP_CRC8_INIT, header, 2);
	memcpy(&tx_frame[3], payload, len);

	uint16_t crc = ep_crc16(EP_CRC16_INIT, payload, len);
	tx_frame[3 + len] = (uint8_t)(crc >> 8);
	tx_frame[4 + len] = (uint8_t)(crc & 0xFF);

	console_write_raw(tx_frame, (uint32_t)(5 + len));
}

void endpoint_serial_poll(void){
	if(!req_pending){return;}
	if(bDeviceState != CONFIGURED){
		/* USB 在请求和应答之间断开了：丢掉请求，别把二进制响应
		 * 退回到 USART3 控制台上 */
		req_pending = 0;
		return;
	}

	/* 先做快照再清标志：清标志之后中断才可能写 req_buf，这里读的
	 * 始终是稳定的一份。代价只是一次 64 字节以内的 memcpy */
	__asm volatile ("" ::: "memory");
	uint16_t n = req_len;
	memcpy(req_snapshot, req_buf, n);
	__asm volatile ("" ::: "memory");
	req_pending = 0;

	uint8_t resp[EP_MAX_PAYLOAD];
	uint16_t resp_len;
	if(endpoint_process(req_snapshot, n, resp, sizeof(resp), &resp_len)){
		ep_send_frame(resp, resp_len);
	}
}
