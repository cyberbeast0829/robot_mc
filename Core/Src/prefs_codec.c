/*
 * prefs_codec.c
 *
 * 配置 TLV 编解码。纯逻辑：不包含任何 flash 访问，主机测试可以直接链接。
 */

#include <string.h>
#include "prefs_codec.h"

static uint16_t rd16(const uint8_t *p){
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd32(const uint8_t *p){
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
	     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr16(uint8_t *p, uint16_t v){
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)(v >> 8);
}

static void wr32(uint8_t *p, uint32_t v){
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
	p[2] = (uint8_t)((v >> 16) & 0xFF);
	p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static size_t pad4(size_t n){
	return (n + 3u) & ~(size_t)3u;
}

uint32_t prefs_crc32(uint32_t state, const void *data, size_t len){
	const uint8_t *p = (const uint8_t *)data;
	while(len--){
		state ^= (uint32_t)(*p++) << 24;
		for(int b = 0; b < 8; b++){
			state = (state & 0x80000000u) ? ((state << 1) ^ 0x04C11DB7u) : (state << 1);
		}
	}
	return state;
}

static const prefs_entry_t *find_entry(const prefs_entry_t *entries, unsigned count,
                                       uint16_t id){
	for(unsigned i = 0; i < count; i++){
		if(entries[i].id == id){return &entries[i];}
	}
	return NULL;
}

bool prefs_block_valid(const uint8_t *block, size_t len,
                       uint32_t *sequence, uint16_t *payload_len){
	if(!block || (len < PREFS_HEADER_LEN)){return false;}
	if(rd32(block) != PREFS_MAGIC){return false;}
	if(rd16(block + 4) != PREFS_SCHEMA){return false;}

	uint16_t plen = rd16(block + 12);
	uint16_t nrec = rd16(block + 14);
	if((plen == 0) || (plen > PREFS_MAX_PAYLOAD)){return false;}
	if(((size_t)PREFS_HEADER_LEN + plen) > len){return false;}
	if(nrec > PREFS_MAX_RECORDS){return false;}

	uint32_t crc = prefs_crc32(0xFFFFFFFFu, block, 16);
	crc = prefs_crc32(crc, block + PREFS_HEADER_LEN, plen);
	if(crc != rd32(block + 16)){return false;}

	if(sequence){*sequence = rd32(block + 8);}
	if(payload_len){*payload_len = plen;}
	return true;
}

unsigned prefs_block_load(const uint8_t *block, size_t len,
                          const prefs_entry_t *entries, unsigned count){
	uint16_t plen;
	if(!prefs_block_valid(block, len, NULL, &plen)){return 0;}

	const uint8_t *p = block + PREFS_HEADER_LEN;
	const uint8_t *end = p + plen;
	unsigned applied = 0;

	while((size_t)(end - p) >= PREFS_REC_HDR_LEN){
		uint16_t id   = rd16(p);
		uint8_t  type = p[2];
		uint16_t vlen = rd16(p + 4);
		size_t   need = PREFS_REC_HDR_LEN + pad4(vlen);

		if((vlen == 0) || (vlen > PREFS_MAX_VALUE)){break;}
		if((size_t)(end - p) < need){break;}

		const prefs_entry_t *e = find_entry(entries, count, id);
		if(e && e->ptr && (e->type == type) && (e->size == vlen)){
			memcpy(e->ptr, p + PREFS_REC_HDR_LEN, vlen);
			applied++;
		}
		p += need;
	}
	return applied;
}

prefs_status_t prefs_block_build(uint8_t *out, size_t cap, uint32_t sequence,
                                 const prefs_entry_t *entries, unsigned count,
                                 const uint8_t *old_block, size_t old_len,
                                 size_t *out_len){
	if(!out || (cap < PREFS_HEADER_LEN)){return PREFS_ERR_TOO_BIG;}
	/* 旧块是输入，绝不能和目标缓冲重叠：头部 memset 会把源数据毁掉 */
	if(old_block == out){return PREFS_ERR_VALUE;}

	memset(out, 0, PREFS_HEADER_LEN);
	size_t off = PREFS_HEADER_LEN;
	unsigned nrec = 0;

	for(unsigned i = 0; i < count; i++){
		const prefs_entry_t *e = &entries[i];
		if(!e->ptr || (e->size == 0) || (e->size > PREFS_MAX_VALUE)){continue;}

		size_t padded = pad4(e->size);
		if((off + PREFS_REC_HDR_LEN + padded) > cap){return PREFS_ERR_TOO_BIG;}
		if(nrec >= PREFS_MAX_RECORDS){return PREFS_ERR_TOO_BIG;}

		wr16(out + off, e->id);
		out[off + 2] = e->type;
		out[off + 3] = 0;
		wr16(out + off + 4, (uint16_t)e->size);
		wr16(out + off + 6, 0);
		memcpy(out + off + PREFS_REC_HDR_LEN, e->ptr, e->size);
		if(padded > e->size){
			memset(out + off + PREFS_REC_HDR_LEN + e->size, 0, padded - e->size);
		}
		off += PREFS_REC_HDR_LEN + padded;
		nrec++;
	}

	/* 未知记录透传：旧块里 ID 不在当前表里的记录原样保留 */
	uint16_t old_plen;
	if(old_block && prefs_block_valid(old_block, old_len, NULL, &old_plen)){
		const uint8_t *p = old_block + PREFS_HEADER_LEN;
		const uint8_t *end = p + old_plen;

		while((size_t)(end - p) >= PREFS_REC_HDR_LEN){
			uint16_t id   = rd16(p);
			uint16_t vlen = rd16(p + 4);
			size_t   need = PREFS_REC_HDR_LEN + pad4(vlen);

			if((vlen == 0) || (vlen > PREFS_MAX_VALUE)){break;}
			if((size_t)(end - p) < need){break;}

			if(!find_entry(entries, count, id)){
				if((off + need) > cap){return PREFS_ERR_TOO_BIG;}
				if(nrec >= PREFS_MAX_RECORDS){return PREFS_ERR_TOO_BIG;}
				memcpy(out + off, p, need);
				off += need;
				nrec++;
			}
			p += need;
		}
	}

	/* 一条记录都没有的块 prefs_block_valid() 会拒收；提前报错，别白擦一页 */
	if(nrec == 0){return PREFS_ERR_VALUE;}

	size_t plen = off - PREFS_HEADER_LEN;
	if(plen > PREFS_MAX_PAYLOAD){return PREFS_ERR_TOO_BIG;}

	wr32(out, PREFS_MAGIC);
	wr16(out + 4, PREFS_SCHEMA);
	wr16(out + 6, 0);
	wr32(out + 8, sequence);
	wr16(out + 12, (uint16_t)plen);
	wr16(out + 14, (uint16_t)nrec);

	uint32_t crc = prefs_crc32(0xFFFFFFFFu, out, 16);
	crc = prefs_crc32(crc, out + PREFS_HEADER_LEN, plen);
	wr32(out + 16, crc);

	if(out_len){*out_len = off;}
	return PREFS_OK;
}
