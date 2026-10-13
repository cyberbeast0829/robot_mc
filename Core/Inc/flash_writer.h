/*
 * flash_writer.h
 *
 *  Created on: Apr 12, 2020
 *      Author: Ben
 *
 * ⚠ NOT USED BY THIS FORK.  This is the upstream sector-based flash writer
 * (STM32F446 / ODrive).  Configuration storage was replaced by
 * preference_writer.c (magic + version + sequence + CRC-32, ping-ponged between
 * the top two 8 KB pages).  Nothing calls the flash_writer_ or flash_read_
 * functions any more; the module is kept only so the Keil project file does not
 * have to change.  The sector addresses and sizes below are the upstream
 * layout and do NOT match the N32H473CE's uniform 8 KB pages (0x2000).
 */

#ifndef INC_FLASH_WRITER_H_
#define INC_FLASH_WRITER_H_
#ifdef __cplusplus
extern "C" {
#endif


#include "n32h47x_48x_flash.h"
#include "stdbool.h"

/* Base address of the flash sectors (upstream layout - unused, see the note) */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08070000)
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08072000)
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08074000)
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08076000)
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08078000)
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x0807A000)
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x0807C000)
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x0807E000)

typedef struct {
	bool ready;
	uint32_t base;
	uint32_t sector;
}FlashWriter;

void flash_writer_init(FlashWriter *fw, uint32_t sector);
bool flash_writer_ready(FlashWriter fw);
void flash_writer_open(FlashWriter *fw);
void flash_writer_write_int(FlashWriter fw, uint32_t index, int x);
void flash_writer_write_uint(FlashWriter fw, uint32_t index, unsigned int x);
void flash_writer_write_float(FlashWriter fw, uint32_t index, float x);
void flash_writer_close(FlashWriter *fw);
int flash_read_int(FlashWriter fw, uint32_t index);
uint32_t flash_read_uint(FlashWriter fw, uint32_t index);
float flash_read_float(FlashWriter fw, uint32_t index);

#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH_WRITER_H_ */
