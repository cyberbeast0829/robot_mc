/*
 * preference_writer.c
 *
 *  Created on: Apr 13, 2020
 *      Author: ben
 */

#include <string.h>
#include "preference_writer.h"
#include "user_config.h"
#include "n32h47x_48x_flash.h"
#include "n32h47x_48x_iwdg.h"	// the save kicks the watchdog itself, the ISR is masked

/* The two 8 KB flash pages at the top of the N32H473CE's 512 KB flash (same page
 * layout as flash_writer.h).  The stored block is ~1.3 KB, so one page each is
 * plenty and the ping-pong swap only costs a single page erase. */
#define N_SECTORS			2
static const uint32_t sector_addr[N_SECTORS] = {0x0807C000, 0x0807E000};	// top two 8 KB pages

#define CONFIG_MAGIC		0x434F4E46	// 'CONF'
#define CONFIG_VERSION		1
#define N_INT_REG			256
#define N_FLOAT_REG			64

/* Word offsets within a stored block */
#define OFFS_MAGIC			0
#define OFFS_VERSION		1
#define OFFS_SEQUENCE		2
#define OFFS_CRC			3
#define OFFS_PAYLOAD		4
#define N_PAYLOAD_WORDS		(N_INT_REG + N_FLOAT_REG)

/* Software CRC-32, the same algorithm the hardware CRC unit implements
 * (CRC-32/MPEG-2: poly 0x04C11DB7, init 0xFFFFFFFF, no bit reflection, no final
 * XOR).  Kept in software so it needs no peripheral clock. */
static uint32_t crc_state;

static void crc_begin(void){
	crc_state = 0xFFFFFFFFu;
}

static void crc_feed(uint32_t word){
	crc_state ^= word;
	for(int b = 0; b < 32; b++){
		crc_state = (crc_state & 0x80000000u) ? ((crc_state << 1) ^ 0x04C11DB7u) : (crc_state << 1);
	}
}

static uint32_t float_bits(float f){
	uint32_t u;
	memcpy(&u, &f, sizeof(u));
	return u;
}

static uint32_t crc_of_ram_config(uint32_t sequence){
	crc_begin();
	crc_feed(sequence);
	for(int i = 0; i < N_INT_REG; i++){crc_feed((uint32_t)__int_reg[i]);}
	for(int i = 0; i < N_FLOAT_REG; i++){crc_feed(float_bits(__float_reg[i]));}
	return crc_state;
}

static int block_valid(int sector, uint32_t *sequence){
	const uint32_t *b = (const uint32_t *)sector_addr[sector];
	if(b[OFFS_MAGIC] != CONFIG_MAGIC){return 0;}
	if(b[OFFS_VERSION] != CONFIG_VERSION){return 0;}
	crc_begin();
	crc_feed(b[OFFS_SEQUENCE]);
	for(int i = 0; i < N_PAYLOAD_WORDS; i++){crc_feed(b[OFFS_PAYLOAD + i]);}
	if(crc_state != b[OFFS_CRC]){return 0;}
	*sequence = b[OFFS_SEQUENCE];
	return 1;
}

void preference_writer_load(PreferenceWriter *pr){
	uint32_t seq[N_SECTORS];
	int valid[N_SECTORS];
	for(int s = 0; s < N_SECTORS; s++){valid[s] = block_valid(s, &seq[s]);}

	int src = -1;
	if(valid[0] && valid[1]){src = (seq[1] > seq[0]) ? 1 : 0;}	// newest block wins
	else if(valid[0]){src = 0;}
	else if(valid[1]){src = 1;}

	if(src >= 0){
		const uint32_t *b = (const uint32_t *)sector_addr[src];
		for(int i = 0; i < N_INT_REG; i++){__int_reg[i] = (int)b[OFFS_PAYLOAD + i];}
		memcpy(__float_reg, (const void *)&b[OFFS_PAYLOAD + N_INT_REG], N_FLOAT_REG*sizeof(float));
		pr->active_sector = src;
		pr->sequence = seq[src];
		pr->loaded = 1;
		return;
	}

	/* No valid block.  If there's a legacy (pre-CRC) config in the first
	 * sector - raw registers starting at the sector base, written by firmware
	 * from before this format - import it so calibration survives the
	 * firmware upgrade.  It isn't re-saved here; the first save writes the
	 * new format to the OTHER sector, so the legacy data stays intact as a
	 * fallback until the save after that. */
	/* Only treat sector 0 as legacy data if it actually looks like it.  A block
	 * that carries the current magic but failed its CRC is a corrupt block of the
	 * CURRENT format, not legacy data - importing it would load its header words
	 * (magic / version / sequence / CRC) as config registers, including a garbage
	 * PHASE_ORDER and ENCODER_LUT.  A couple of range checks on the well-known
	 * legacy registers catch random flash as well. */
	const uint32_t *legacy = (const uint32_t *)sector_addr[0];
	int legacy_ok = 1;
	if(legacy[OFFS_MAGIC] == CONFIG_MAGIC){legacy_ok = 0;}			// corrupt block of the current format
	else if(legacy[0] == 0xFFFFFFFFu){legacy_ok = 0;}				// erased
	else if((legacy[0] & ~1u) != 0u){legacy_ok = 0;}				// PHASE_ORDER must be 0 or 1
	else if((legacy[1] > 127u) || (legacy[2] > 127u)){legacy_ok = 0;}	// CAN_ID / CAN_MASTER must fit 7 bits
	pr->loaded = legacy_ok;
	if(pr->loaded){
		for(int i = 0; i < N_INT_REG; i++){__int_reg[i] = (int)legacy[i];}
		memcpy(__float_reg, (const void *)&legacy[N_INT_REG], N_FLOAT_REG*sizeof(float));
	}
	/* Otherwise the chip is erased: the boot-time sanitization in main()
	 * supplies defaults */
	pr->active_sector = 0;		// so the first save targets sector 1
	pr->sequence = 0;
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
	/* Write the RAM config to the page NOT holding the newest valid block.
	 *
	 * The erase + program sequence has to be atomic with respect to every other
	 * flash access, so interrupts are masked for all of it:
	 *  - one 64-bit program means "write two words to one flash address" (UM) -
	 *    the two stores must be issued back to back with nothing in between;
	 *  - while BUSY is set, any flash access (an instruction fetch, or an
	 *    exception's vector fetch) suspends the CPU until the operation ends.
	 * An interrupt taken between those two stores therefore stalls the CPU inside
	 * the vector fetch while the controller waits for a second word that can never
	 * be written - a permanent deadlock.  The watchdog cannot break it either: it
	 * is kicked from the control ISR, which can no longer run, so the chip is
	 * reset a few seconds later and the save silently never lands.  The UM states
	 * the same rule for the debugger: "调试模式下，Row/双字编程过程中不要打断点，
	 * 否则会导致编程错误".
	 *
	 * The whole save is a few hundred milliseconds (the controller suspends the
	 * CPU for ~2.6 ms per operation - measured by stepping it with a debugger),
	 * inside the ~4 s watchdog window even with the LSI at the fast end of its
	 * tolerance, and the caller (save_preferences in fsm.c) only runs this from
	 * the main loop, with the gate drive off, in SETUP/MENU.  The watchdog is
	 * kicked from here because the masked ISR cannot do it. */
	int target = !pr->active_sector;
	uint32_t base = sector_addr[target];
	uint32_t sequence = pr->sequence + 1;
	int ok = 0;

	IWDG_ReloadKey();
	__disable_irq();

	if(flash_wait_idle()){
		FLASH_Unlock();
		ok = (FLASH_EraseOnePage(base) == FLASH_EOP);
		/* Header: magic, version, sequence, CRC - two 64-bit programs */
		ok = ok && flash_program64(base + 4*OFFS_MAGIC, CONFIG_MAGIC, CONFIG_VERSION);
		ok = ok && flash_program64(base + 4*OFFS_SEQUENCE, sequence, crc_of_ram_config(sequence));
		/* Payload: 256 ints then 64 floats, two 32-bit words per program */
		for(int i = 0; ok && (i < N_INT_REG); i += 2){
			ok = flash_program64(base + 4*OFFS_PAYLOAD + 4*i, (uint32_t)__int_reg[i], (uint32_t)__int_reg[i+1]);
			IWDG_ReloadKey();	// the control ISR is masked and cannot kick it
		}
		for(int i = 0; ok && (i < N_FLOAT_REG); i += 2){
			ok = flash_program64(base + 4*(OFFS_PAYLOAD + N_INT_REG) + 4*i, float_bits(__float_reg[i]), float_bits(__float_reg[i+1]));
			IWDG_ReloadKey();
		}
		flash_wait_idle();	// let the last program finish before touching CTRL again
		FLASH_Lock();
	}

	__enable_irq();
	IWDG_ReloadKey();

	if(!ok){return 0;}		// caller keeps the previous block as the active one

	/* Verify what actually landed in flash before pointing at it */
	uint32_t check_seq;
	if(!block_valid(target, &check_seq) || (check_seq != sequence)){return 0;}
	pr->active_sector = target;
	pr->sequence = sequence;
	return 1;
}
