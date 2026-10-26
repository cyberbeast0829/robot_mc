/*
 * prefs_defs.h
 *
 * 由 tools/gen_endpoints.py 从 params.json 生成，请勿手工修改。
 */
#ifndef PREFS_DEFS_H
#define PREFS_DEFS_H

#include <stdint.h>
#include "prefs_codec.h"

extern const prefs_entry_t prefs_entries[];
extern const unsigned prefs_entry_count;

/* 把缺失记录所需的默认值写进 RAM（上电时先调用它，再加载记录） */
void config_apply_defaults(void);

/* 对加载后的配置做 NaN/Inf 检查和 min/max 限幅 */
void config_apply_limits(void);

#endif /* PREFS_DEFS_H */
