/*
 * prefs_defs.c
 *
 * 由 tools/gen_endpoints.py 从 params.json 生成，请勿手工修改。
 */

#include <math.h>
#include <string.h>
#include "prefs_defs.h"
#include "prefs_codec.h"
#include "user_config.h"
#include "hw_config.h"
#include "position_sensor.h"

/* 顺序按 endpoint ID 升序，便于 diff 和线性查找 */
const prefs_entry_t prefs_entries[] = {
	{1, PREFS_T_F32, 4, (void *)&(I_BW)},
	{2, PREFS_T_F32, 4, (void *)&(I_MAX)},
	{3, PREFS_T_F32, 4, (void *)&(I_FW_MAX)},
	{4, PREFS_T_F32, 4, (void *)&(I_MAX_CONT)},
	{5, PREFS_T_F32, 4, (void *)&(I_CAL)},
	{6, PREFS_T_F32, 4, (void *)&(THETA_MIN)},
	{7, PREFS_T_F32, 4, (void *)&(THETA_MAX)},
	{8, PREFS_T_F32, 4, (void *)&(P_MAX)},
	{9, PREFS_T_F32, 4, (void *)&(P_MIN)},
	{10, PREFS_T_F32, 4, (void *)&(V_MAX)},
	{11, PREFS_T_F32, 4, (void *)&(V_MIN)},
	{12, PREFS_T_F32, 4, (void *)&(KP_MAX)},
	{13, PREFS_T_F32, 4, (void *)&(KD_MAX)},
	{14, PREFS_T_F32, 4, (void *)&(GR)},
	{15, PREFS_T_F32, 4, (void *)&(KT)},
	{16, PREFS_T_F32, 4, (void *)&(PPAIRS)},
	{17, PREFS_T_F32, 4, (void *)&(R_PHASE)},
	{18, PREFS_T_F32, 4, (void *)&(R_NOMINAL)},
	{19, PREFS_T_F32, 4, (void *)&(R_TH)},
	{20, PREFS_T_F32, 4, (void *)&(C_TH)},
	{21, PREFS_T_F32, 4, (void *)&(TEMP_MAX)},
	{22, PREFS_T_I32, 4, (void *)&(PHASE_ORDER)},
	{23, PREFS_T_I32, 4, (void *)&(CAN_ID)},
	{24, PREFS_T_I32, 4, (void *)&(CAN_MASTER)},
	{25, PREFS_T_I32, 4, (void *)&(CAN_TIMEOUT)},
	{26, PREFS_T_I32, 4, (void *)&(M_ZERO)},
	{27, PREFS_T_I32, 4, (void *)&(E_ZERO)},
	{118, PREFS_T_BLOB, 512, (void *)&(ENCODER_LUT)},
};
const unsigned prefs_entry_count = sizeof(prefs_entries) / sizeof(prefs_entries[0]);

_Static_assert(sizeof(prefs_entries) / sizeof(prefs_entries[0]) <= 256u,
            "params.json: 持久化条目超过 PREFS_MAX_RECORDS");
_Static_assert(N_LUT * sizeof(int) == 512, "internal.encoder_lut: 尺寸与固件不一致");

void config_apply_defaults(void){
	I_BW = 1000.0f;
	I_MAX = I_MAX_LIMIT;
	I_FW_MAX = 0.0f;
	I_MAX_CONT = 14.0f;
	I_CAL = 5.0f;
	THETA_MIN = 0.0f;
	THETA_MAX = 0.0f;
	P_MAX = 12.5f;
	P_MIN = -12.5f;
	V_MAX = 65.0f;
	V_MIN = -65.0f;
	KP_MAX = 500.0f;
	KD_MAX = 5.0f;
	GR = 7.75f;
	KT = 1.0f;
	PPAIRS = 14.0f;
	R_PHASE = 0.0f;
	R_NOMINAL = 0.0f;
	R_TH = 0.0f;
	C_TH = 0.0f;
	TEMP_MAX = 125.0f;
	PHASE_ORDER = 0;
	CAN_ID = 1;
	CAN_MASTER = 0;
	CAN_TIMEOUT = 100;
	M_ZERO = 0;
	E_ZERO = 0;
	memset((void *)&(ENCODER_LUT), 0, 512);
}

void config_apply_limits(void){
	if(!isfinite(I_BW)){I_BW = 1000.0f;}
	if(I_BW < 100.0f){I_BW = 100.0f;}
	if(I_BW > 2000.0f){I_BW = 2000.0f;}
	if(!isfinite(I_MAX)){I_MAX = I_MAX_LIMIT;}
	if(I_MAX < 0.0f){I_MAX = 0.0f;}
	if(I_MAX > I_MAX_LIMIT){I_MAX = I_MAX_LIMIT;}
	if(!isfinite(I_FW_MAX)){I_FW_MAX = 0.0f;}
	if(I_FW_MAX < 0.0f){I_FW_MAX = 0.0f;}
	if(I_FW_MAX > 33.0f){I_FW_MAX = 33.0f;}
	if(!isfinite(I_MAX_CONT)){I_MAX_CONT = 14.0f;}
	if(I_MAX_CONT < 0.0f){I_MAX_CONT = 0.0f;}
	if(I_MAX_CONT > 40.0f){I_MAX_CONT = 40.0f;}
	if(!isfinite(I_CAL)){I_CAL = 5.0f;}
	if(I_CAL < 0.0f){I_CAL = 0.0f;}
	if(I_CAL > 20.0f){I_CAL = 20.0f;}
	if(!isfinite(THETA_MIN)){THETA_MIN = 0.0f;}
	if(!isfinite(THETA_MAX)){THETA_MAX = 0.0f;}
	if(!isfinite(P_MAX)){P_MAX = 12.5f;}
	if(P_MAX < 0.0f){P_MAX = 0.0f;}
	if(!isfinite(P_MIN)){P_MIN = -12.5f;}
	if(P_MIN > 0.0f){P_MIN = 0.0f;}
	if(!isfinite(V_MAX)){V_MAX = 65.0f;}
	if(V_MAX < 0.0f){V_MAX = 0.0f;}
	if(!isfinite(V_MIN)){V_MIN = -65.0f;}
	if(V_MIN > 0.0f){V_MIN = 0.0f;}
	if(!isfinite(KP_MAX)){KP_MAX = 500.0f;}
	if(KP_MAX < 0.0f){KP_MAX = 0.0f;}
	if(!isfinite(KD_MAX)){KD_MAX = 5.0f;}
	if(KD_MAX < 0.0f){KD_MAX = 0.0f;}
	if(!isfinite(GR)){GR = 7.75f;}
	if(GR < 0.001f){GR = 0.001f;}
	if(!isfinite(KT)){KT = 1.0f;}
	if(KT < 0.0001f){KT = 0.0001f;}
	if(!isfinite(PPAIRS)){PPAIRS = 14.0f;}
	if(PPAIRS < 1.0f){PPAIRS = 1.0f;}
	if(PPAIRS > 64.0f){PPAIRS = 64.0f;}
	if(!isfinite(R_PHASE)){R_PHASE = 0.0f;}
	if(!isfinite(R_NOMINAL)){R_NOMINAL = 0.0f;}
	if(!isfinite(R_TH)){R_TH = 0.0f;}
	if(!isfinite(C_TH)){C_TH = 0.0f;}
	if(!isfinite(TEMP_MAX)){TEMP_MAX = 125.0f;}
	if(TEMP_MAX < 0.0f){TEMP_MAX = 0.0f;}
	if(TEMP_MAX > 150.0f){TEMP_MAX = 150.0f;}
	if(PHASE_ORDER < 0){PHASE_ORDER = 0;}
	if(PHASE_ORDER > 1){PHASE_ORDER = 1;}
	if(CAN_ID < 0){CAN_ID = 0;}
	if(CAN_ID > 127){CAN_ID = 127;}
	if(CAN_MASTER < 0){CAN_MASTER = 0;}
	if(CAN_MASTER > 127){CAN_MASTER = 127;}
	if(CAN_TIMEOUT < 0){CAN_TIMEOUT = 0;}
	if(CAN_TIMEOUT > 100000){CAN_TIMEOUT = 100000;}
}
