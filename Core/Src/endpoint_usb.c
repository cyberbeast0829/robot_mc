/*
 * endpoint_usb.c
 *
 * fibre 风格 endpoint 协议跑在 USB vendor interface（EP1 OUT / EP3 IN）上。
 * 帧格式见 endpoint.h；这里只做“一个 USB 包 = 一帧”的搬运。
 *
 * 为什么不在中断里应答：
 *   - 40 kHz 的 ATIM1 控制中断不能被 USB 解析/CRC/handler 拖慢；
 *   - 主循环本来就要跑 endpoint_serial_poll()/run_fsm_slow()，加一次
 *     64 字节以内的 memcpy + dispatch 对 FOC 没有影响。
 * 请求不会丢：收到后端点保持 NAK，主机自动重试，直到主循环重新 arm。
 */

#include <string.h>
#include "usbfsd_lib.h"
#include "endpoint_usb.h"
#include "endpoint.h"
#include "usb.h"
#include "usb_conf.h"
#include "usb_pwr.h"

#define EP_USB_RX_ENDPOINT		ENDP1
#define EP_USB_TX_ENDPOINT		ENDP3

/* 响应永远小于一个满包：主机用一次 bulk 读（64 字节）就能收全，
 * 不需要零长度包来结束传输。 */
#define EP_USB_TX_MTU			63

/* 主机在总线复位/掉线时可能永远不来取 IN：主循环等这么多轮后自行放弃，
 * 免得 tx_busy 卡死（正常情况下一两个主循环周期内就会完成） */
#define EP_USB_TX_TIMEOUT		100000ul

static uint8_t rx_buf[EP_MAX_PAYLOAD];
static volatile uint16_t rx_len;
static volatile uint8_t rx_pending;

static uint8_t req_snapshot[EP_MAX_PAYLOAD];
static uint8_t tx_buf[EP_MAX_PAYLOAD];
static uint16_t tx_len;
static volatile uint8_t tx_busy;
static uint32_t tx_wait;

/* 重新挂上 OUT 端点：显式恢复 64 字节缓冲长度（收到包后计数寄存器里是
 * 实际字节数），再置 VALID。 */
static void ep_usb_rx_arm(void)
{
	USB_SetEpRxCnt(EP_USB_RX_ENDPOINT, VIRTUAL_COM_PORT_DATA_SIZE);
	USB_SetEpRxValid(EP_USB_RX_ENDPOINT);
}

void endpoint_usb_rx_isr(uint16_t pma_addr, uint16_t len)
{
	/* 端点只在主循环取走上一包后才重新 arm，正常不会到这里 */
	if (rx_pending) {
		return;
	}
	if (len > EP_MAX_PAYLOAD) {
		len = EP_MAX_PAYLOAD;
	}
	USB_CopyPMAToUserBuf(rx_buf, pma_addr, len);
	rx_len = len;
	__asm volatile ("" ::: "memory");
	rx_pending = 1;
}

void endpoint_usb_tx_isr(void)
{
	tx_wait = 0;
	tx_busy = 0;
}

void endpoint_usb_reset(void)
{
	rx_pending = 0;
	tx_busy = 0;
	tx_wait = 0;
}

void endpoint_usb_poll(void)
{
	if (tx_busy) {
		if (++tx_wait < EP_USB_TX_TIMEOUT) {
			return;
		}
		/* 主机没来取：放弃这包，免得复位后 tx_busy 永远为 1 */
		tx_wait = 0;
		tx_busy = 0;
		SetEPTxStatus(EP_USB_TX_ENDPOINT, EP_TX_NAK);
	}
	if (rx_pending == 0) {
		return;
	}
	if (bDeviceState != CONFIGURED) {
		/* USB 在请求和应答之间掉线：丢掉请求并重新 arm */
		rx_pending = 0;
		ep_usb_rx_arm();
		return;
	}

	__asm volatile ("" ::: "memory");
	uint16_t n = rx_len;
	memcpy(req_snapshot, rx_buf, n);
	__asm volatile ("" ::: "memory");
	rx_pending = 0;
	/* 先重新 arm：下一帧可以在我们算应答时到达，ISR 只会写 rx_buf */
	ep_usb_rx_arm();

	uint16_t resp_len;
	if (!endpoint_process(req_snapshot, n, tx_buf, EP_USB_TX_MTU, &resp_len)) {
		return;
	}
	if (bDeviceState != CONFIGURED) {
		return;		/* 算应答期间掉线：丢弃，主机重连后重发 */
	}
	tx_len = resp_len;
	/* 先置忙再 arm：IN 完成中断可能在 USB_Send_DATA() 返回前就发生 */
	tx_wait = 0;
	tx_busy = 1;
	USB_Send_DATA(tx_buf, (uint8_t)tx_len, EP_USB_TX_ENDPOINT);
}
