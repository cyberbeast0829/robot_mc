/*
 * prefs_codec.h
 *
 * 配置持久化的纯编解码层（不碰硬件，可主机单测）。
 *
 * 块格式（schema 2）：
 *
 *   Header (20 B):
 *     magic        u32  'PCFG'
 *     schema       u16  PREFS_SCHEMA
 *     flags        u16
 *     sequence     u32
 *     payload_len  u16
 *     record_cnt   u16
 *     crc32        u32  覆盖 header[0..15] + payload
 *
 *   Record (8 B 头):
 *     id           u16  稳定 endpoint ID
 *     type         u8   PREFS_T_*
 *     flags        u8
 *     len          u16  值字节数（≤ PREFS_MAX_VALUE）
 *     reserved     u16
 *     value[len]       小端，按 4 字节对齐补齐
 *
 * 记录以 endpoint ID 为 key，与 __float_reg/__int_reg 的下标无关。新增参数
 * 缺记录时用默认值；旧块里不认识的记录在保存时原样透传。
 */

#ifndef INC_PREFS_CODEC_H_
#define INC_PREFS_CODEC_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PREFS_MAGIC				0x47464350u	// 'PCFG' (little-endian bytes)
#define PREFS_SCHEMA			2u
/* schema 1 -> 2：config.can_timeout 的单位从“控制周期(25 us)”换成毫秒。
 * 旧块仍然可以被读入（prefs_block_valid() 接受它），随后由
 * preference_writer_load() 调 prefs_v1_cycles_to_ms() 做一次性换算。
 * 注意：换算只发生在 RAM 里；下次保存时写出的块已经是 schema 2。 */
#define PREFS_SCHEMA_V1			1u
#define PREFS_CONTROL_CYCLES_PER_MS	40	// 控制环 40 kHz（DT = 25 us），与 user_config.h 的 CAN_TIMEOUT_CYCLES 一致
#define PREFS_HEADER_LEN		20u
#define PREFS_REC_HDR_LEN		8u
#define PREFS_MAX_PAYLOAD		4096u
#define PREFS_MAX_RECORDS		256u
#define PREFS_MAX_VALUE			512u

typedef enum{
	PREFS_OK = 0,
	PREFS_ERR_FORMAT,
	PREFS_ERR_CRC,
	PREFS_ERR_TOO_BIG,
	PREFS_ERR_VALUE
}prefs_status_t;

typedef enum{
	PREFS_T_BOOL = 1,
	PREFS_T_U8,
	PREFS_T_I8,
	PREFS_T_U16,
	PREFS_T_I16,
	PREFS_T_U32,
	PREFS_T_I32,
	PREFS_T_F32,
	PREFS_T_BLOB
}prefs_type_t;

/* 生成器产出的持久化表条目：ptr 指向 RAM 里的实际变量/数组 */
typedef struct{
	uint16_t id;
	uint8_t  type;
	uint16_t size;
	void    *ptr;
}prefs_entry_t;

/** CRC-32/MPEG-2：poly 0x04C11DB7，init 0xFFFFFFFF，无反射、无最终异或。 */
uint32_t prefs_crc32(uint32_t state, const void *data, size_t len);

/** 校验块头 + CRC。成功时可选返回 sequence / payload 长度。
 *  schema 字段为 PREFS_SCHEMA 或 PREFS_SCHEMA_V1 都算有效（后者需迁移）。 */
bool prefs_block_valid(const uint8_t *block, size_t len,
                       uint32_t *sequence, uint16_t *payload_len);

/** 块的 schema 字段（调用前先 prefs_block_valid()）。 */
uint16_t prefs_block_schema(const uint8_t *block);

/** schema 1 迁移：把“控制周期”计的值换算成毫秒（四舍五入）。 */
int32_t prefs_v1_cycles_to_ms(int32_t cycles);

/** 把块里的已知记录写回 entries[].ptr。返回成功应用的记录数。 */
unsigned prefs_block_load(const uint8_t *block, size_t len,
                          const prefs_entry_t *entries, unsigned count);

/**
 * 构造新块：先写 entries 的当前值，再把 old_block 里 ID 不在 entries 中的
 * 记录原样附加（未知记录透传）。
 * @param out_len 成功时写入实际使用的字节数（≤ PREFS_HEADER_LEN + PREFS_MAX_PAYLOAD）
 */
prefs_status_t prefs_block_build(uint8_t *out, size_t cap, uint32_t sequence,
                                 const prefs_entry_t *entries, unsigned count,
                                 const uint8_t *old_block, size_t old_len,
                                 size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* INC_PREFS_CODEC_H_ */
