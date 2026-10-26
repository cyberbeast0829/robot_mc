/*
 * endpoint.c
 *
 * 协议无关的 endpoint 核心：CRC、小端编解码、endpoint 0 描述符。
 * 生成的参数分发在 endpoint_defs.c（见 params.json / gen_endpoints.py）。
 */

#include <string.h>
#include "endpoint.h"
#include "endpoint_defs.h"

uint8_t ep_crc8(uint8_t remainder, const uint8_t *data, size_t len){
	/* 与 fibre 的 calc_crc<uint8_t, 0x37> 逐位一致 */
	while(len--){
		remainder ^= *data++;
		for(int bit = 0; bit < 8; bit++){
			remainder = (remainder & 0x80u) ? (uint8_t)((remainder << 1) ^ EP_CRC8_POLY) : (uint8_t)(remainder << 1);
		}
	}
	return remainder;
}

uint16_t ep_crc16(uint16_t remainder, const uint8_t *data, size_t len){
	/* 与 fibre 的 calc_crc<uint16_t, 0x3D65> 逐位一致 */
	while(len--){
		remainder ^= (uint16_t)(*data++) << 8;
		for(int bit = 0; bit < 8; bit++){
			remainder = (remainder & 0x8000u) ? (uint16_t)((remainder << 1) ^ EP_CRC16_POLY) : (uint16_t)(remainder << 1);
		}
	}
	return remainder;
}

static bool ep_take(ep_cbuf_t *b, const uint8_t **p, size_t n){
	if((size_t)(b->end - b->begin) < n){return false;}
	*p = b->begin;
	b->begin += n;
	return true;
}

static bool ep_room(ep_buf_t *b, size_t n){
	return (size_t)(b->end - b->begin) >= n;
}

bool ep_get_u8(ep_cbuf_t *b, uint8_t *v){
	const uint8_t *p;
	if(!ep_take(b, &p, 1)){return false;}
	*v = p[0];
	return true;
}

bool ep_get_i8(ep_cbuf_t *b, int8_t *v){
	uint8_t u;
	if(!ep_get_u8(b, &u)){return false;}
	*v = (int8_t)u;
	return true;
}

bool ep_get_u16(ep_cbuf_t *b, uint16_t *v){
	const uint8_t *p;
	if(!ep_take(b, &p, 2)){return false;}
	*v = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
	return true;
}

bool ep_get_i16(ep_cbuf_t *b, int16_t *v){
	uint16_t u;
	if(!ep_get_u16(b, &u)){return false;}
	*v = (int16_t)u;
	return true;
}

bool ep_get_u32(ep_cbuf_t *b, uint32_t *v){
	const uint8_t *p;
	if(!ep_take(b, &p, 4)){return false;}
	*v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
	return true;
}

bool ep_get_i32(ep_cbuf_t *b, int32_t *v){
	uint32_t u;
	if(!ep_get_u32(b, &u)){return false;}
	*v = (int32_t)u;
	return true;
}

bool ep_get_f32(ep_cbuf_t *b, float *v){
	uint32_t u;
	if(!ep_get_u32(b, &u)){return false;}
	memcpy(v, &u, sizeof(u));
	return true;
}

bool ep_put_u8(ep_buf_t *b, uint8_t v){
	if(!ep_room(b, 1)){return false;}
	*b->begin++ = v;
	return true;
}

bool ep_put_i8(ep_buf_t *b, int8_t v){
	return ep_put_u8(b, (uint8_t)v);
}

bool ep_put_u16(ep_buf_t *b, uint16_t v){
	if(!ep_room(b, 2)){return false;}
	*b->begin++ = (uint8_t)(v & 0xFF);
	*b->begin++ = (uint8_t)(v >> 8);
	return true;
}

bool ep_put_i16(ep_buf_t *b, int16_t v){
	return ep_put_u16(b, (uint16_t)v);
}

bool ep_put_u32(ep_buf_t *b, uint32_t v){
	if(!ep_room(b, 4)){return false;}
	*b->begin++ = (uint8_t)(v & 0xFF);
	*b->begin++ = (uint8_t)((v >> 8) & 0xFF);
	*b->begin++ = (uint8_t)((v >> 16) & 0xFF);
	*b->begin++ = (uint8_t)((v >> 24) & 0xFF);
	return true;
}

bool ep_put_i32(ep_buf_t *b, int32_t v){
	return ep_put_u32(b, (uint32_t)v);
}

bool ep_put_f32(ep_buf_t *b, float v){
	uint32_t u;
	memcpy(&u, &v, sizeof(u));
	return ep_put_u32(b, u);
}

float ep_clampf(float v, float lo, float hi){
	if(v < lo){return lo;}
	if(v > hi){return hi;}
	return v;
}

int32_t ep_clampi(int32_t v, int32_t lo, int32_t hi){
	if(v < lo){return lo;}
	if(v > hi){return hi;}
	return v;
}

uint32_t ep_clampu(uint32_t v, uint32_t lo, uint32_t hi){
	if(v < lo){return lo;}
	if(v > hi){return hi;}
	return v;
}

ep_status_t endpoint_descriptor(ep_cbuf_t *in, ep_buf_t *out){
	uint32_t offset;
	if(!ep_get_u32(in, &offset)){return EP_ERR_BAD_LEN;}

	if(offset == 0xFFFFFFFFu){
		/* 版本探测：[json_crc][json_len]，上位机据此一次性拿到接口版本和长度 */
		if(!ep_put_u16(out, endpoint_json_crc) || !ep_put_u16(out, endpoint_json_len)){return EP_ERR_BAD_LEN;}
		return EP_OK;
	}

	if(offset >= endpoint_json_len){return EP_OK;}	// 越界返回空片段，客户端据此结束下载

	size_t n = (size_t)(endpoint_json_len - offset);
	size_t room = (size_t)(out->end - out->begin);
	if(n > room){n = room;}
	memcpy(out->begin, endpoint_json + offset, n);
	out->begin += n;
	return EP_OK;
}
