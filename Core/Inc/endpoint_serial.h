/*
 * endpoint_serial.h
 *
 * USB CDC 上的 endpoint 帧收发。
 *
 * endpoint_serial_feed() 在 USB 中断里逐字节调用，只做帧累积；收齐一帧后
 * 由主循环的 endpoint_serial_poll() 解析并应答。ASCII 菜单和二进制帧共用
 * 同一条 CDC 数据流：feed() 返回 true 表示这个字节属于二进制帧。
 */

#ifndef INC_ENDPOINT_SERIAL_H_
#define INC_ENDPOINT_SERIAL_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool endpoint_serial_feed(uint8_t byte);
void endpoint_serial_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ENDPOINT_SERIAL_H_ */
