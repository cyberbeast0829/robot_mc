/*
 * flash_writer.c
 *
 * ⚠ NOT USED BY THIS FORK - see the banner in flash_writer.h.  Configuration is
 * stored by preference_writer.c instead; this file (and its upstream sector
 * table) is kept only so the Keil project file does not have to change.
 */
#include "n32h47x_48x_flash.h"
#include "flash_writer.h"

/* Upstream sector table (was a "static" array in the header, which gave every
 * translation unit its own copy and an -Wunused-variable warning) */
static const uint32_t __SECTOR_ADDRS[] = {ADDR_FLASH_SECTOR_0, ADDR_FLASH_SECTOR_1, ADDR_FLASH_SECTOR_2, ADDR_FLASH_SECTOR_3,
                             ADDR_FLASH_SECTOR_4, ADDR_FLASH_SECTOR_5, ADDR_FLASH_SECTOR_6, ADDR_FLASH_SECTOR_7};

void flash_writer_init(FlashWriter *fw, uint32_t sector) {
	if(sector>7) sector = 7;
	fw->sector = sector;
	fw->base = __SECTOR_ADDRS[sector];
	fw->ready = false;
}
bool flash_writer_ready(FlashWriter fw) {
    return fw.ready;
}

void flash_writer_open(FlashWriter * fw) {
    FLASH_Unlock();
    // FLASH_ClearFlag( FLASH_STS_CLRFLAG );
    FLASH_EraseOnePage(fw->base);
    fw->ready = true;
}

/* One 32-bit value per 8-byte double-word slot (the N32 flash programs 64 bits
 * at a time); the read side only uses the low word.
 * NOTE: the old code aliased an anonymous inner union twice by mistake
 * ("union {uint32_t c; uint32_t d} b;" - c and d overlap), which only worked by
 * accident.  Keep writing the value into both halves. */
void flash_writer_write_int(FlashWriter fw, uint32_t index, int x) {
    union {int i; uint32_t u;} v;
    v.i = x;
    FLASH_ProgramdoubleWord(fw.base + 8 * index, v.u, v.u);
}

void flash_writer_write_uint(FlashWriter fw, uint32_t index, unsigned int x) {
    uint32_t v = (uint32_t)x;
    FLASH_ProgramdoubleWord(fw.base + 8 * index, v, v);
}

void flash_writer_write_float(FlashWriter fw, uint32_t index, float x) {
    union {float f; uint32_t u;} v;
    v.f = x;
    FLASH_ProgramdoubleWord(fw.base + 8 * index, v.u, v.u);
}

void flash_writer_close(FlashWriter * fw) {
    FLASH_Lock();
    fw->ready = false;
}

int flash_read_int(FlashWriter fw, uint32_t index) {
    return *(int*) (__SECTOR_ADDRS[fw.sector] + 8 * index);
}

uint32_t flash_read_uint(FlashWriter fw, uint32_t index) {
    return *(uint32_t*) (__SECTOR_ADDRS[fw.sector] + 8 * index);
}

float flash_read_float(FlashWriter fw, uint32_t index) {
    return *(float*) (__SECTOR_ADDRS[fw.sector] + 8 * index);
}


