/*
 * test_endpoint_proto.c
 *
 * 主机测试：endpoint_process() 的请求/应答语义。
 * 不依赖目标板，也不依赖生成的 endpoint_defs.c（这里给出最小的
 * endpoint_json / endpoint_handler 桩）。
 *
 *   make hosttest
 */

#include <stdio.h>
#include <string.h>
#include "endpoint.h"

/* ---- 生成的端点表的最小桩 ------------------------------------------------ */

const char endpoint_json[] = "[]";
const uint16_t endpoint_json_len = sizeof(endpoint_json) - 1;
const uint16_t endpoint_json_crc = 0x1234;

static uint32_t fake_value = 0x11223344;
static int action_calls = 0;

ep_status_t endpoint_handler(uint16_t id, ep_op_t op, ep_cbuf_t *in, ep_buf_t *out){
	switch(id){
		case 1:		/* rw：读写同一个变量 */
			if(op == EP_OP_WRITE){
				uint32_t v;
				if(!ep_get_u32(in, &v)){return EP_ERR_BAD_LEN;}
				fake_value = v;
			}
			return ep_put_u32(out, fake_value) ? EP_OK : EP_ERR_BAD_LEN;

		case 2:		/* w：动作，只能写 */
			if(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}
			action_calls++;
			return EP_OK;

		case 3:		/* w：动作，有参数 */
			if(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}
			uint8_t mode;
			if(!ep_get_u8(in, &mode)){return EP_ERR_BAD_LEN;}
			action_calls += 1 + mode;
			return EP_OK;

		default:
			return EP_ERR_UNKNOWN_ID;
	}
}

/* ---- 辅助 ---------------------------------------------------------------- */

static int failures = 0;

#define CHECK(cond) do { \
	if(!(cond)){ \
		printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		failures++; \
	} \
} while(0)

static uint16_t seq_no = 0;

static uint16_t build_request(uint8_t *buf, uint16_t id, bool write,
                              const uint8_t *data, uint16_t data_len,
                              uint16_t want, uint16_t trailer){
	uint16_t seq = ++seq_no;
	uint16_t n = 0;
	buf[n++] = (uint8_t)(seq & 0xFF);
	buf[n++] = (uint8_t)(seq >> 8);
	uint16_t idf = id | EP_REQ_REPLY | (write ? EP_REQ_WRITE : 0);
	buf[n++] = (uint8_t)(idf & 0xFF);
	buf[n++] = (uint8_t)(idf >> 8);
	buf[n++] = (uint8_t)(want & 0xFF);
	buf[n++] = (uint8_t)(want >> 8);
	if(data_len){memcpy(&buf[n], data, data_len); n += data_len;}
	buf[n++] = (uint8_t)(trailer & 0xFF);
	buf[n++] = (uint8_t)(trailer >> 8);
	return n;
}

static uint16_t get_u16(const uint8_t *p){return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));}

/* ---- 用例 ---------------------------------------------------------------- */

static void test_descriptor_probe(void){
	uint8_t req[32], resp[64];
	uint32_t off = 0xFFFFFFFFu;
	uint16_t n = build_request(req, 0, false, (const uint8_t *)&off, 4, 4, EP_PROTOCOL_VERSION);
	uint16_t out = 0;
	CHECK(endpoint_process(req, n, resp, sizeof(resp), &out));
	CHECK(out == 3 + 4);
	CHECK((get_u16(resp) & 0x7FFF) == seq_no);
	CHECK((get_u16(resp) & 0x8000) != 0);
	CHECK(resp[2] == EP_OK);
	CHECK(get_u16(resp + 3) == endpoint_json_crc);
	CHECK(get_u16(resp + 5) == endpoint_json_len);
}

static void test_read(void){
	uint8_t req[32], resp[64];
	uint16_t n = build_request(req, 1, false, NULL, 0, 4, endpoint_json_crc);
	uint16_t out = 0;
	CHECK(endpoint_process(req, n, resp, sizeof(resp), &out));
	CHECK(out == 3 + 4);
	CHECK(resp[2] == EP_OK);
	CHECK(memcmp(resp + 3, "\x44\x33\x22\x11", 4) == 0);
}

static void test_write_readback(void){
	uint8_t req[32], resp[64];
	uint32_t v = 0xA1B2C3D4u;
	uint16_t n = build_request(req, 1, true, (const uint8_t *)&v, 4, 4, endpoint_json_crc);
	uint16_t out = 0;
	CHECK(endpoint_process(req, n, resp, sizeof(resp), &out));
	CHECK(resp[2] == EP_OK);
	CHECK(memcmp(resp + 3, "\xD4\xC3\xB2\xA1", 4) == 0);
	CHECK(fake_value == v);
}

static void test_action_read_does_not_execute(void){
	uint8_t req[32], resp[64];
	int before = action_calls;
	uint16_t n = build_request(req, 2, false, NULL, 0, 0, endpoint_json_crc);
	uint16_t out = 0;
	CHECK(endpoint_process(req, n, resp, sizeof(resp), &out));
	CHECK(resp[2] == EP_ERR_WRITE_ONLY);
	CHECK(action_calls == before);
}

static void test_action_write(void){
	uint8_t req[32], resp[64];
	int before = action_calls;
	uint16_t n = build_request(req, 2, true, NULL, 0, 0, endpoint_json_crc);
	uint16_t out = 0;
	CHECK(endpoint_process(req, n, resp, sizeof(resp), &out));
	CHECK(resp[2] == EP_OK);
	CHECK(action_calls == before + 1);
}

static void test_action_with_arg(void){
	uint8_t req[32], resp[64];
	uint8_t mode = 3;
	int before = action_calls;
	uint16_t n = build_request(req, 3, true, &mode, 1, 0, endpoint_json_crc);
	uint16_t out = 0;
	CHECK(endpoint_process(req, n, resp, sizeof(resp), &out));
	CHECK(resp[2] == EP_OK);
	CHECK(action_calls == before + 4);
}

static void test_version_mismatch(void){
	uint8_t req[32], resp[64];
	uint16_t n = build_request(req, 1, false, NULL, 0, 4, 0x9999);
	uint16_t out = 0;
	CHECK(endpoint_process(req, n, resp, sizeof(resp), &out));
	CHECK(resp[2] == EP_ERR_VERSION);
	CHECK(out == 3);
}

static void test_truncate_to_want(void){
	uint8_t req[32], resp[64];
	uint16_t n = build_request(req, 1, false, NULL, 0, 2, endpoint_json_crc);
	uint16_t out = 0;
	CHECK(endpoint_process(req, n, resp, sizeof(resp), &out));
	CHECK(resp[2] == EP_OK);
	CHECK(out == 3 + 2);
}

static void test_no_reply_and_short(void){
	uint8_t resp[64];
	uint16_t out = 0;
	/* REPLY 位没置：不应答 */
	uint8_t raw[] = {0x01, 0x00, 0x01, 0x00, 0x04, 0x00, 0x34, 0x12};
	CHECK(!endpoint_process(raw, sizeof(raw), resp, sizeof(resp), &out));
	/* 连 seq/id/want 都不够 */
	CHECK(!endpoint_process(raw, 4, resp, sizeof(resp), &out));
}

static void test_u64_codec(void){
	/* 8 字节小端：序列号/UID 这类不参与控制的整数 */
	uint8_t buf[16];
	ep_buf_t out = { buf, buf + sizeof(buf) };
	uint64_t v = 0x0123456789ABCDEFull;
	uint64_t back = 0;
	ep_cbuf_t in;

	CHECK(ep_put_u64(&out, v));
	CHECK(out.begin == buf + 8);
	CHECK(buf[0] == 0xEF && buf[1] == 0xCD && buf[7] == 0x01);

	in.begin = buf; in.end = buf + 8;
	CHECK(ep_get_u64(&in, &back));
	CHECK(back == v);
	CHECK(in.begin == in.end);

	/* 空间不足时必须失败且不动游标 */
	in.begin = buf; in.end = buf + 7;
	back = 0;
	CHECK(!ep_get_u64(&in, &back));
	CHECK(in.begin == buf);
}

int main(void){
	test_descriptor_probe();
	test_read();
	test_write_readback();
	test_action_read_does_not_execute();
	test_action_write();
	test_action_with_arg();
	test_version_mismatch();
	test_truncate_to_want();
	test_no_reply_and_short();
	test_u64_codec();

	if(failures){
		printf("test_endpoint_proto: %d failure(s)\n", failures);
		return 1;
	}
	printf("test_endpoint_proto: all tests passed\n");
	return 0;
}
