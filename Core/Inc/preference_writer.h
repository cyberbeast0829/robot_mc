/*
 * preference_writer.h
 *
 *  Created on: Apr 13, 2020
 *      Author: ben
 */

#ifndef INC_PREFERENCE_WRITER_H_
#define INC_PREFERENCE_WRITER_H_

#include <stdint.h>

/* Flash-backed config storage (ID-keyed TLV, see prefs_codec.h).
 *
 * Records are stored in the top two 8 KB flash pages, ping-ponged with a
 * sequence counter and CRC-32.  Each save erases and rewrites the page NOT
 * holding the newest valid block, so a power loss during a save never loses
 * the previous config.
 *
 * The record key is the endpoint ID, NOT the __float_reg/__int_reg index:
 * adding/reordering RAM parameters cannot invalidate stored values.  Missing
 * records keep the defaults that main() writes with config_apply_defaults();
 * records whose IDs are no longer known are carried through on save.
 *
 * The application linker region ends at 0x0807C000, so the image can never
 * grow into these pages.  The bootloader must only erase the app region. */

typedef struct{
	uint32_t sequence;			// sequence number of the loaded / last-saved block
	uint32_t active_sector;		// 0 or 1: which sector holds the newest valid block
	uint16_t schema;			// PREFS_SCHEMA of the loaded block
	int loaded;					// 1 if a config was found, 0 if erased
}PreferenceWriter;

/* Call config_apply_defaults() BEFORE this: the loader only overwrites the
 * records that exist in flash. */
void preference_writer_load(PreferenceWriter *pr);
int preference_writer_save(PreferenceWriter *pr);	// 1 on success, verified by CRC read-back

extern PreferenceWriter prefs;

#endif /* INC_PREFERENCE_WRITER_H_ */
