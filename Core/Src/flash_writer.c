#include "n32h47x_48x_flash.h"
#include "flash_writer.h"




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
    FLASH_EraseOnePage(w->base);
    fw->ready = true;
}

void flash_writer_write_int(FlashWriter fw, uint32_t index, int x) {
    union UN {int a; union {uint32_t c; uint32_t d} b;};
    union UN un;
    un.a = x;
    FLASH_ProgramdoubleWord(fw.base + 8 * index, un.b.c, un.b.d);
}

void flash_writer_write_uint(FlashWriter fw, uint32_t index, unsigned int x) {
    union UN {unsigned int a; union {uint32_t c; uint32_t d} b;};
    union UN un;
    un.a = x;
    FLASH_ProgramdoubleWord(fw.base + 8 * index, un.b.c, un.b.d);
}

void flash_writer_write_float(FlashWriter fw, uint32_t index, float x) {
    union UN {float a; union {uint32_t c; uint32_t d} b;};
    union UN un;
    un.a = x;
    FLASH_ProgramdoubleWord(fw.base + 8 * index, un.b.c, un.b.d);
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


