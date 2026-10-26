/*
 * preference_writer.c
 *
 *  Created on: Apr 13, 2020
 *      Author: ben
 */

#include <string.h>
#include "preference_writer.h"
#include "prefs_codec.h"
#include "prefs_defs.h"
#include "n32h47x_48x_flash.h"
#include "n32h47x_48x_iwdg.h"	// the save kicks the watchdog itself, the ISR is masked

/* 两个 8 KB 页做 ping-pong。v2 块是 ID-keyed TLV，当前约 1 KB，一页足够。
 * 注意：链接脚本把 app 限制在 0x7C000，配置页 0x0807C000/0x0807E000 必须由
 * bootloader 保留（只擦 app 区）。 */
#define N_SECTORS			2
#define PREFS_PAGE_SIZE		8192u
static const uint32_t sector_addr[N_SECTORS] = {0x0807C000, 0x0807E000};

static int block_valid(int sector, uint32_t *sequence, uint16_t *payload_len){
	return prefs_block_valid((const uint8_t *)sector_addr[sector], PREFS_PAGE_SIZE,
	                         sequence, payload_len);
}

void preference_writer_load(PreferenceWriter *pr){
	uint32_t seq[N_SECTORS];
	uint16_t plen[N_SECTORS];
	int valid[N_SECTORS];

	for(int s = 0; s < N_SECTORS; s++){valid[s] = block_valid(s, &seq[s], &plen[s]);}

	int src = -1;
	if(valid[0] && valid[1]){src = (seq[1] > seq[0]) ? 1 : 0;}	// newest block wins
	else if(valid[0]){src = 0;}
	else if(valid[1]){src = 1;}

	if(src >= 0){
		/* 记录按 endpoint ID 写回 RAM；缺记录由 main.c 先调用
		 * config_apply_defaults() 补上的默认值保持不变 */
		(void)prefs_block_load((const uint8_t *)sector_addr[src], PREFS_PAGE_SIZE,
		                       prefs_entries, prefs_entry_count);
		pr->active_sector = src;
		pr->sequence = seq[src];
		pr->schema = PREFS_SCHEMA;
		pr->loaded = 1;
		return;
	}

	/* 擦除状态：默认值已经由 main.c 填好，这里只记录“没有块”。 */
	pr->active_sector = 0;		// so the first save targets sector 1
	pr->sequence = 0;
	pr->schema = PREFS_SCHEMA;
	pr->loaded = 0;
}

/* Wait until the flash controller reports that no operation is in progress.
 * The UM requires this before writing any FLASH register: both
 * FLASH_EraseOnePage() and FLASH_ProgramdoubleWord() start with
 * FLASH_ClearFlag(), i.e. a write to FLASH->STS, and writing any Flash register
 * while FLASH_STS.BUSY is set is not allowed ("当 FLASH_STS.BUSY 为'1'时，不能对
 * 任何 Flash 寄存器执行写操作").  Such a write can leave the controller wedged,
 * and from then on every flash access - including every instruction fetch -
 * suspends the CPU, which is the hang seen on the bench.  Reading FLASH->STS
 * while BUSY is safe: that read is itself the access which gets stalled until
 * the operation finishes. */
static int flash_wait_idle(void){
	uint32_t timeout = EraseTimeout;
	while(timeout-- != 0){
		if(FLASH_GetSTS() != FLASH_BUSY){return 1;}
	}
	return 0;
}

/* One 64-bit program, with the idle check the SDK omits at entry */
static int flash_program64(uint32_t address, uint32_t data0, uint32_t data1){
	return flash_wait_idle() && (FLASH_ProgramdoubleWord(address, data0, data1) == FLASH_EOP);
}

int preference_writer_save(PreferenceWriter *pr){
	int target = !pr->active_sector;
	uint32_t base = sector_addr[target];
	uint32_t sequence = pr->sequence + 1;
	const uint8_t *old_block = pr->loaded
	                         ? (const uint8_t *)sector_addr[pr->active_sector] : NULL;
	static uint8_t work[PREFS_HEADER_LEN + PREFS_MAX_PAYLOAD];
	size_t out_len = 0;
	int ok = 0;

	/* 序列化放在关中断之前：读 RAM + 读旧块（旧块在另一页，不会被擦）。
	 * 未知 ID 的记录会从旧块原样带到新块。 */
	if(prefs_block_build(work, sizeof(work), sequence, prefs_entries, prefs_entry_count,
	                     old_block, PREFS_PAGE_SIZE, &out_len) != PREFS_OK){
		return 0;
	}

	/* The erase + program sequence has to be atomic with respect to every other
	 * flash access, so interrupts are masked for all of it:
	 *  - one 64-bit program means "write two words to one flash address" (UM) -
	 *    the two stores must be issued back to back with nothing in between;
	 *  - while BUSY is set, any flash access (an instruction fetch, or an
	 *    exception's vector fetch) suspends the CPU until the operation ends.
	 * An interrupt taken between those two stores therefore stalls the CPU inside
	 * the vector fetch while the controller waits for a second word that can never
	 * be written - a permanent deadlock.  The watchdog cannot break it either: it
	 * is kicked from the control ISR, which can no longer run, so the chip is
	 * reset a few seconds later and the save silently never lands.
	 *
	 * The whole save is a few hundred milliseconds, inside the ~4 s watchdog
	 * window, and the caller (save_preferences in fsm.c) only runs this from the
	 * main loop, with the gate drive off.  The watchdog is kicked from here
	 * because the masked ISR cannot do it. */
	IWDG_ReloadKey();
	__disable_irq();

	if(flash_wait_idle()){
		FLASH_Unlock();
		ok = (FLASH_EraseOnePage(base) == FLASH_EOP);
		for(size_t off = 0; ok && (off < out_len); off += 8){
			uint32_t w0 = (uint32_t)work[off]
			            | ((uint32_t)work[off + 1] << 8)
			            | ((uint32_t)work[off + 2] << 16)
			            | ((uint32_t)work[off + 3] << 24);
			uint32_t w1 = 0xFFFFFFFFu;
			if((off + 4) < out_len){
				w1 = (uint32_t)work[off + 4]
				   | ((uint32_t)work[off + 5] << 8)
				   | ((uint32_t)work[off + 6] << 16)
				   | ((uint32_t)work[off + 7] << 24);
			}
			ok = flash_program64(base + (uint32_t)off, w0, w1);
			IWDG_ReloadKey();	// the control ISR is masked and cannot kick it
		}
		flash_wait_idle();	// let the last program finish before touching CTRL again
		FLASH_Lock();
	}

	__enable_irq();
	IWDG_ReloadKey();

	if(!ok){return 0;}		// caller keeps the previous block as the active one

	/* Verify what actually landed in flash before pointing at it */
	uint32_t check_seq;
	uint16_t check_len;
	if(!block_valid(target, &check_seq, &check_len) || (check_seq != sequence)){return 0;}
	pr->active_sector = target;
	pr->sequence = sequence;
	pr->schema = PREFS_SCHEMA;
	pr->loaded = 1;
	return 1;
}
