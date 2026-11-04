/*
 * endpoint.h
 *
 * 轻量级 fibre 风格 endpoint 访问层。
 *
 * 参数表（名字/类型/访问权限/上下限/C 引用）由仓库根目录的 params.json
 * 维护，tools/gen_endpoints.py 生成 endpoint_defs.c/h；本文件是手写的
 * 协议核心：CRC、小端序列化、endpoint 0（JSON 描述符）。
 *
 * 两种传输共用下面的 PAYLOAD 格式（细节见 AGENTS.md）：
 *   CDC   : AA | LEN | CRC8 | PAYLOAD[LEN] | CRC16_HI CRC16_LO
 *   vendor: 一个 USB bulk 包 = 一个 PAYLOAD（无外层、无 CRC）
 *
 * 请求 PAYLOAD: seq(2) | id|flags(2) | rxlen(2) | tx_data... | trailer(2)
 * 响应 PAYLOAD: seq|0x8000(2) | status(1) | rx_data...
 *
 * id 字段的 flag 位：
 *   bit15 EP_REQ_REPLY  需要应答（主机总是置位）
 *   bit14 EP_REQ_WRITE  这是一次写/执行；不置位就是读
 * 读和写必须由传输层显式给出，不能靠“有没有数据”来猜：动作端点如果被
 * 当成读来执行，一次 PARAM_READ 就能让电机停机。
 *
 * id==0 是 JSON 描述符，trailer 为 EP_PROTOCOL_VERSION；其它 endpoint 的
 * trailer 是 JSON 描述符的 CRC16。这样固件升级后旧上位机发来的请求会被
 * 直接拒绝，不会出现“名字对上了、类型对不上”的静默错误。
 */

#ifndef INC_ENDPOINT_H_
#define INC_ENDPOINT_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EP_PROTOCOL_VERSION		2

/* 一帧 payload 上限。USB CDC 一次最多 64 字节，描述符分块也按这个上限走 */
#define EP_MAX_PAYLOAD			64

/* 请求 id 字段的 flag（低 14 位是 endpoint ID） */
#define EP_REQ_REPLY			0x8000
#define EP_REQ_WRITE			0x4000
#define EP_ID_MASK				0x3FFF

/* 帧头前缀。ASCII 菜单里不会出现这个字节 */
#define EP_FRAME_PREFIX			0xAA

/* 与 fibre v0.1 相同的 CRC 参数（MSB-first，无反射，无最终异或） */
#define EP_CRC8_POLY			0x37
#define EP_CRC8_INIT			0x42
#define EP_CRC16_POLY			0x3D65
#define EP_CRC16_INIT			0x1337

/* 响应里的 status 字节。0 表示成功 */
typedef enum{
	EP_OK = 0,
	EP_ERR_UNKNOWN_ID = 1,		// 端点号不存在
	EP_ERR_BAD_LEN = 2,			// 数据长度不够/缓冲区太小
	EP_ERR_BAD_VALUE = 3,		// NaN 或无法表示的数值
	EP_ERR_READ_ONLY = 4,
	EP_ERR_WRITE_ONLY = 5,
	EP_ERR_STATE = 6,			// 当前状态不允许（例如门极已使能）
	EP_ERR_VERSION = 7			// JSON CRC 不匹配，需要重新下载描述符
}ep_status_t;

/* 和 fibre 的 cbufptr_t / bufptr_t 同构的游标，避免到处传 (ptr, len) */
typedef struct{
	const uint8_t *begin;
	const uint8_t *end;
}ep_cbuf_t;

typedef struct{
	uint8_t *begin;
	uint8_t *end;
}ep_buf_t;

/* 请求意图。传输层必须显式给出：USB 用 EP_REQ_WRITE 位，CAN 用
 * PARAM_READ/PARAM_WRITE 操作码。生成的 handler 按描述符的 access 校验。 */
typedef enum{
	EP_OP_READ = 0,
	EP_OP_WRITE = 1
}ep_op_t;

uint8_t ep_crc8(uint8_t remainder, const uint8_t *data, size_t len);
uint16_t ep_crc16(uint16_t remainder, const uint8_t *data, size_t len);

/* 小端解码/编码。空间不足时返回 false，游标不前进 */
bool ep_get_u8(ep_cbuf_t *b, uint8_t *v);
bool ep_get_i8(ep_cbuf_t *b, int8_t *v);
bool ep_get_u16(ep_cbuf_t *b, uint16_t *v);
bool ep_get_i16(ep_cbuf_t *b, int16_t *v);
bool ep_get_u32(ep_cbuf_t *b, uint32_t *v);
bool ep_get_i32(ep_cbuf_t *b, int32_t *v);
bool ep_get_f32(ep_cbuf_t *b, float *v);

bool ep_put_u8(ep_buf_t *b, uint8_t v);
bool ep_put_i8(ep_buf_t *b, int8_t v);
bool ep_put_u16(ep_buf_t *b, uint16_t v);
bool ep_put_i16(ep_buf_t *b, int16_t v);
bool ep_put_u32(ep_buf_t *b, uint32_t v);
bool ep_put_i32(ep_buf_t *b, int32_t v);
bool ep_put_f32(ep_buf_t *b, float v);

float ep_clampf(float v, float lo, float hi);
int32_t ep_clampi(int32_t v, int32_t lo, int32_t hi);
uint32_t ep_clampu(uint32_t v, uint32_t lo, uint32_t hi);

/* endpoint 0：按偏移返回 JSON 描述符片段。
 * offset == 0xFFFFFFFF 时返回 [json_crc u16][json_len u16]，供上位机
 * 一次性拿到版本和总长度。 */
ep_status_t endpoint_descriptor(ep_cbuf_t *in, ep_buf_t *out);

/* 把一帧请求 PAYLOAD 处理成一帧应答 PAYLOAD（seq|0x8000 | status | data）。
 * 传输层只负责外层封装：CDC 用 0xAA+LEN+CRC8/CRC16 流，fibre 的 vendor
 * 接口一个 USB 包就是一帧。返回 false 表示这帧不需要应答（或长度不够）。
 * resp_cap 必须 >= 3；返回长度不会超过 resp_cap。 */
bool endpoint_process(const uint8_t *req, uint16_t req_len,
                      uint8_t *resp, uint16_t resp_cap, uint16_t *resp_len);

#ifdef __cplusplus
}
#endif

#endif /* INC_ENDPOINT_H_ */
