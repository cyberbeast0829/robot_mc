/*
 * endpoint_usb.h
 *
 * fibre 风格 endpoint 协议在 USB vendor interface（interface 2）上的传输层。
 *
 * 和 CDC（endpoint_serial.c）共用同一份请求/应答 payload 编解码
 * （endpoint_process()），区别只在外层封装：
 *
 *   CDC   : AA | LEN | CRC8 | PAYLOAD | CRC16_HI CRC16_LO   （字节流）
 *   vendor: 一个 USB bulk 包就是一帧 PAYLOAD                  （USB 自带分帧）
 *
 * 实时性约定（不能破坏 40 kHz FOC 控制中断）：
 *   - USB 中断只把 64 字节从 PMA 拷到邮箱、置标志，然后 NAK 住端点；
 *     解析、调用 handler、应答全部在主循环的 endpoint_usb_poll() 里完成。
 *   - 响应长度上限 63 字节（短包），主机一次 bulk 读就能收全。
 */

#ifndef INC_ENDPOINT_USB_H_
#define INC_ENDPOINT_USB_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USB 中断（EP1 OUT）调用：把收到的包搬进邮箱。不重挂端点，
 * 由主循环取走后再调用 USB_Receive_DATA()，天然实现流控。 */
void endpoint_usb_rx_isr(uint16_t pma_addr, uint16_t len);

/* USB 中断（EP3 IN）调用：上一包已经发给主机。 */
void endpoint_usb_tx_isr(void);

/* USB 总线复位时清掉状态（进行中的 IN 传输已被硬件丢弃）。 */
void endpoint_usb_reset(void);

/* 主循环调用：解析请求并应答。 */
void endpoint_usb_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ENDPOINT_USB_H_ */
