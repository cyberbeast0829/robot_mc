/*
 * test_prefs_codec.c
 *
 * prefs_codec.c 的主机单测（不碰硬件，gcc 直接编译运行）。
 *
 *   make hosttest
 *   make hosttest HOSTCC=/path/to/gcc
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "prefs_codec.h"

static int failures;

#define CHECK(cond) do{ \
	if(!(cond)){ \
		printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		failures++; \
	} \
}while(0)

static const prefs_entry_t entries[] = {
	{1, PREFS_T_F32, 4, NULL},
	{2, PREFS_T_I32, 4, NULL},
	{3, PREFS_T_BLOB, 512, NULL},
};
#define N_ENTRIES (sizeof(entries) / sizeof(entries[0]))

static uint8_t block[PREFS_HEADER_LEN + PREFS_MAX_PAYLOAD];
static uint8_t work[PREFS_HEADER_LEN + PREFS_MAX_PAYLOAD];

/* 每轮测试用独立的 RAM 副本，避免指针全局化 */
static float f32_a, f32_b;
static int32_t i32_a;
static uint8_t blob_a[512];

static void bind(prefs_entry_t *e){
	e[0].ptr = &f32_a;
	e[1].ptr = &i32_a;
	e[2].ptr = &blob_a[0];
}

/* 改写 schema 字段并重算 CRC（用来伪造上一个版本的块） */
static void patch_schema(uint8_t *b, uint16_t schema){
	uint16_t plen = (uint16_t)(b[12] | (b[13] << 8));
	b[4] = (uint8_t)(schema & 0xFF);
	b[5] = (uint8_t)(schema >> 8);
	uint32_t crc = prefs_crc32(0xFFFFFFFFu, b, 16);
	crc = prefs_crc32(crc, b + PREFS_HEADER_LEN, plen);
	b[16] = (uint8_t)(crc & 0xFF);
	b[17] = (uint8_t)((crc >> 8) & 0xFF);
	b[18] = (uint8_t)((crc >> 16) & 0xFF);
	b[19] = (uint8_t)((crc >> 24) & 0xFF);
}

static void test_schema_migration(void){
	prefs_entry_t e[N_ENTRIES];
	size_t len = 0;

	memcpy(e, entries, sizeof(entries));
	bind(e);
	f32_a = 2.0f; i32_a = 1000;
	CHECK(prefs_block_build(block, sizeof(block), 5, e, N_ENTRIES, NULL, 0, &len) == PREFS_OK);
	CHECK(prefs_block_schema(block) == PREFS_SCHEMA);

	/* 降级成 schema 1（老固件写出的块）：仍必须能读，且 schema 可辨认 */
	patch_schema(block, PREFS_SCHEMA_V1);
	CHECK(prefs_block_valid(block, sizeof(block), NULL, NULL));
	CHECK(prefs_block_schema(block) == PREFS_SCHEMA_V1);

	/* 从 schema 1 旧块保存 -> 新块必须是当前 schema */
	CHECK(prefs_block_build(work, sizeof(work), 6, e, N_ENTRIES, block, sizeof(block), &len)
              == PREFS_OK);
	CHECK(prefs_block_schema(work) == PREFS_SCHEMA);

	/* v1 -> v2 单位换算：控制周期(25 us) -> 毫秒，四舍五入，0 仍是关闭 */
	CHECK(prefs_v1_cycles_to_ms(0) == 0);
	CHECK(prefs_v1_cycles_to_ms(20) == 1);
	CHECK(prefs_v1_cycles_to_ms(1000) == 25);
	CHECK(prefs_v1_cycles_to_ms(40000) == 1000);
	CHECK(prefs_v1_cycles_to_ms(-40) == 0);
}

static void test_roundtrip(void){
	prefs_entry_t e[N_ENTRIES];
	size_t len = 0;

	memcpy(e, entries, sizeof(entries));
	f32_a = 123.5f; i32_a = -77; memset(blob_a, 0, sizeof(blob_a));
	e[0].ptr = &f32_a;
	e[1].ptr = &i32_a;
	e[2].ptr = &blob_a[0];

	CHECK(prefs_block_build(block, sizeof(block), 7, e, N_ENTRIES, NULL, 0, &len) == PREFS_OK);
	CHECK(len == PREFS_HEADER_LEN + (8 + 4) * 2 + (8 + 512));
	CHECK(prefs_block_valid(block, sizeof(block), NULL, NULL));

	/* 目标改成别的值，再加载回来 */
	f32_a = 0.0f; i32_a = 0; memset(blob_a, 0, sizeof(blob_a));
	CHECK(prefs_block_load(block, len, e, N_ENTRIES) == N_ENTRIES);
	CHECK(fabsf(f32_a - 123.5f) < 1e-6f);
	CHECK(i32_a == -77);
}

static void test_defaults_and_carry(void){
	prefs_entry_t e[N_ENTRIES];
	size_t len = 0;
	uint8_t *p;
	uint16_t id = 99;		// 未来固件新增的参数，当前表里没有

	memcpy(e, entries, sizeof(entries));
	bind(e);
	f32_a = 1.25f; i32_a = 5; memset(blob_a, 0x5A, sizeof(blob_a));
	CHECK(prefs_block_build(block, sizeof(block), 1, e, N_ENTRIES, NULL, 0, &len) == PREFS_OK);

	/* 把 f32 记录的 ID 改成 99，模拟“旧固件不认识的新记录” */
	p = block + PREFS_HEADER_LEN;
	p[0] = (uint8_t)(id & 0xFF);
	p[1] = (uint8_t)(id >> 8);
	/* 改 ID 后 CRC 失效，重算一遍（测试直接调用公开的 crc） */
	{
		uint32_t crc = prefs_crc32(0xFFFFFFFFu, block, 16);
		uint16_t plen = (uint16_t)(block[12] | (block[13] << 8));
		crc = prefs_crc32(crc, block + PREFS_HEADER_LEN, plen);
		block[16] = (uint8_t)(crc & 0xFF);
		block[17] = (uint8_t)((crc >> 8) & 0xFF);
		block[18] = (uint8_t)((crc >> 16) & 0xFF);
		block[19] = (uint8_t)((crc >> 24) & 0xFF);
	}

	/* 新表里没有 ID 99，加载时跳过；保存时原样带到新块 */
	f32_a = 0.0f; i32_a = 0;
	CHECK(prefs_block_load(block, len, e, N_ENTRIES) == N_ENTRIES - 1);
	CHECK(f32_a == 0.0f);
	CHECK(prefs_block_build(work, sizeof(work), 2, e, N_ENTRIES, block, sizeof(block), &len) == PREFS_OK);
	CHECK(prefs_block_valid(work, sizeof(work), NULL, NULL));
	{
		uint16_t plen = (uint16_t)(work[12] | (work[13] << 8));
		const uint8_t *q;
		int found = 0;
		q = work + PREFS_HEADER_LEN;
		while((size_t)(work + PREFS_HEADER_LEN + plen - q) >= PREFS_REC_HDR_LEN){
			uint16_t rid = (uint16_t)(q[0] | (q[1] << 8));
			uint16_t vlen = (uint16_t)(q[4] | (q[5] << 8));
			if(rid == 99){found = 1; break;}
			q += PREFS_REC_HDR_LEN + ((vlen + 3u) & ~3u);
		}
		CHECK(found);
	}
}

static void test_reject(void){
	prefs_entry_t e[N_ENTRIES];
	size_t len = 0;

	memcpy(e, entries, sizeof(entries));
	bind(e);
	f32_a = 1.0f; i32_a = 1;
	CHECK(prefs_block_build(block, sizeof(block), 1, e, N_ENTRIES, NULL, 0, &len) == PREFS_OK);

	/* CRC */
	block[16] ^= 0xFF;
	CHECK(!prefs_block_valid(block, sizeof(block), NULL, NULL));
	block[16] ^= 0xFF;

	/* schema：不认识的版本拒收（CRC 一并重算，确保是 schema 判据在起作用） */
	patch_schema(block, 9);
	CHECK(!prefs_block_valid(block, sizeof(block), NULL, NULL));
	patch_schema(block, PREFS_SCHEMA);
	CHECK(prefs_block_valid(block, sizeof(block), NULL, NULL));

	/* 别名：旧块不能和目标缓冲是同一块内存 */
	CHECK(prefs_block_build(block, sizeof(block), 2, e, N_ENTRIES, block, sizeof(block), &len) == PREFS_ERR_VALUE);

	/* 空表：一条记录都没有的块是无效的，必须提前报错 */
	CHECK(prefs_block_build(work, sizeof(work), 2, e, 0, NULL, 0, &len) == PREFS_ERR_VALUE);

	/* 容量：一张装不下的表 */
	{
		prefs_entry_t big[PREFS_MAX_RECORDS + 1];
		memset(big, 0, sizeof(big));
		for(unsigned k = 0; k < PREFS_MAX_RECORDS + 1; k++){
			big[k].id = (uint16_t)(1000 + k);
			big[k].type = PREFS_T_F32;
			big[k].size = 4;
			big[k].ptr = &f32_b;
		}
		CHECK(prefs_block_build(work, sizeof(work), 2, big, PREFS_MAX_RECORDS + 1, NULL, 0, &len)
		      == PREFS_ERR_TOO_BIG);
	}
}

int main(void){
	test_roundtrip();
	test_defaults_and_carry();
	test_reject();
	test_schema_migration();

	if(failures){
		printf("%d failure(s)\n", failures);
		return 1;
	}
	printf("prefs_codec: all tests passed\n");
	return 0;
}
