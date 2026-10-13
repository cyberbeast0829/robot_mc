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

int preference_writer_save(PreferenceWriter *pr){
	/* Write the RAM config to the page NOT holding the newest valid block.
	 * Erasing the page stalls the CPU (the caller - save_preferences in fsm.c -
	 * kicks the watchdog first and only runs from the main loop, with the gate
	 * drive off).  The N32 flash programs in 64-bit double-words. */
	int target = !pr->active_sector;
	uint32_t base = sector_addr[target];
	uint32_t sequence = pr->sequence + 1;

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_STS_CLRFLAG);
	if(FLASH_EraseOnePage(base) != FLASH_EOP){
		FLASH_Lock();
		return 0;
	}
	/* Header: magic, version, sequence, CRC - two 64-bit programs */
	FLASH_ProgramdoubleWord(base + 4*OFFS_MAGIC,    CONFIG_MAGIC, CONFIG_VERSION);
	FLASH_ProgramdoubleWord(base + 4*OFFS_SEQUENCE, sequence, crc_of_ram_config(sequence));
	/* Payload: 256 ints then 64 floats, two 32-bit words per program */
	for(int i = 0; i < N_INT_REG; i += 2){
		FLASH_ProgramdoubleWord(base + 4*OFFS_PAYLOAD + 4*i, (uint32_t)__int_reg[i], (uint32_t)__int_reg[i+1]);
	}
	for(int i = 0; i < N_FLOAT_REG; i += 2){
		FLASH_ProgramdoubleWord(base + 4*(OFFS_PAYLOAD + N_INT_REG) + 4*i, float_bits(__float_reg[i]), float_bits(__float_reg[i+1]));
	}
	FLASH_Lock();

	/* Verify what actually landed in flash before pointing at it */
	uint32_t check_seq;
	if(!block_valid(target, &check_seq) || (check_seq != sequence)){return 0;}
	pr->active_sector = target;
	pr->sequence = sequence;
	return 1;
}
