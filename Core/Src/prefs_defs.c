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
	{120, PREFS_T_I32, 4, (void *)&(CAN_PROT)},
	{121, PREFS_T_I32, 4, (void *)&(CAN_BAUD)},
	{122, PREFS_T_I32, 4, (void *)&(CAN_HB_MS)},
	{133, PREFS_T_F32, 4, (void *)&(L_D)},
	{134, PREFS_T_F32, 4, (void *)&(L_Q)},
	{136, PREFS_T_F32, 4, (void *)&(POS_GAIN)},
	{137, PREFS_T_F32, 4, (void *)&(VEL_GAIN)},
	{138, PREFS_T_F32, 4, (void *)&(VEL_INTEGRATOR_GAIN)},
	{139, PREFS_T_F32, 4, (void *)&(POS_FF_GAIN)},
	{140, PREFS_T_F32, 4, (void *)&(ACCEL_LIMIT)},
	{141, PREFS_T_F32, 4, (void *)&(POS_ERROR_FAULT)},
	{142, PREFS_T_F32, 4, (void *)&(PLL_BW)},
	{143, PREFS_T_I32, 4, (void *)&(VEL_LOOP_PRESCALER)},
	{144, PREFS_T_I32, 4, (void *)&(POS_LOOP_PRESCALER)},
	{151, PREFS_T_F32, 4, (void *)&(INERTIA)},
	{159, PREFS_T_F32, 4, (void *)&(OMEGA_C)},
	{160, PREFS_T_F32, 4, (void *)&(TUNE_MARGIN)},
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
	CAN_PROT = 0;
	CAN_BAUD = 500000;
	CAN_HB_MS = 100;
	L_D = 9.5e-05f;
	L_Q = 0.000124f;
	POS_GAIN = 15.0f;
	VEL_GAIN = 2.469f;
	VEL_INTEGRATOR_GAIN = 44.44f;
	POS_FF_GAIN = 1.0f;
	ACCEL_LIMIT = 50.0f;
	POS_ERROR_FAULT = 0.0f;
	PLL_BW = 2000.0f;
	VEL_LOOP_PRESCALER = 5;
	POS_LOOP_PRESCALER = 2;
	INERTIA = 0.0006f;
	OMEGA_C = 450.0f;
	TUNE_MARGIN = 2.2f;
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
	if(CAN_PROT < 0){CAN_PROT = 0;}
	if(CAN_PROT > 1){CAN_PROT = 1;}
	if(CAN_BAUD < 50000){CAN_BAUD = 50000;}
	if(CAN_BAUD > 5000000){CAN_BAUD = 5000000;}
	if(CAN_HB_MS < 0){CAN_HB_MS = 0;}
	if(CAN_HB_MS > 10000){CAN_HB_MS = 10000;}
	if(!isfinite(L_D)){L_D = 9.5e-05f;}
	if(L_D < 1e-06f){L_D = 1e-06f;}
	if(L_D > 0.01f){L_D = 0.01f;}
	if(!isfinite(L_Q)){L_Q = 0.000124f;}
	if(L_Q < 1e-06f){L_Q = 1e-06f;}
	if(L_Q > 0.01f){L_Q = 0.01f;}
	if(!isfinite(POS_GAIN)){POS_GAIN = 15.0f;}
	if(POS_GAIN < 0.0f){POS_GAIN = 0.0f;}
	if(POS_GAIN > 1000.0f){POS_GAIN = 1000.0f;}
	if(!isfinite(VEL_GAIN)){VEL_GAIN = 2.469f;}
	if(VEL_GAIN < 0.0f){VEL_GAIN = 0.0f;}
	if(VEL_GAIN > 2000.0f){VEL_GAIN = 2000.0f;}
	if(!isfinite(VEL_INTEGRATOR_GAIN)){VEL_INTEGRATOR_GAIN = 44.44f;}
	if(VEL_INTEGRATOR_GAIN < 0.0f){VEL_INTEGRATOR_GAIN = 0.0f;}
	if(VEL_INTEGRATOR_GAIN > 200000.0f){VEL_INTEGRATOR_GAIN = 200000.0f;}
	if(!isfinite(POS_FF_GAIN)){POS_FF_GAIN = 1.0f;}
	if(POS_FF_GAIN < 0.0f){POS_FF_GAIN = 0.0f;}
	if(POS_FF_GAIN > 2.0f){POS_FF_GAIN = 2.0f;}
	if(!isfinite(ACCEL_LIMIT)){ACCEL_LIMIT = 50.0f;}
	if(ACCEL_LIMIT < 0.0f){ACCEL_LIMIT = 0.0f;}
	if(ACCEL_LIMIT > 100000.0f){ACCEL_LIMIT = 100000.0f;}
	if(!isfinite(POS_ERROR_FAULT)){POS_ERROR_FAULT = 0.0f;}
	if(POS_ERROR_FAULT < 0.0f){POS_ERROR_FAULT = 0.0f;}
	if(POS_ERROR_FAULT > 1000.0f){POS_ERROR_FAULT = 1000.0f;}
	if(!isfinite(PLL_BW)){PLL_BW = 2000.0f;}
	if(PLL_BW < 20.0f){PLL_BW = 20.0f;}
	if(PLL_BW > 5000.0f){PLL_BW = 5000.0f;}
	if(VEL_LOOP_PRESCALER < 1){VEL_LOOP_PRESCALER = 1;}
	if(VEL_LOOP_PRESCALER > 100){VEL_LOOP_PRESCALER = 100;}
	if(POS_LOOP_PRESCALER < 1){POS_LOOP_PRESCALER = 1;}
	if(POS_LOOP_PRESCALER > 100){POS_LOOP_PRESCALER = 100;}
	if(!isfinite(INERTIA)){INERTIA = 0.0006f;}
	if(INERTIA < 0.0f){INERTIA = 0.0f;}
	if(INERTIA > 100.0f){INERTIA = 100.0f;}
	if(!isfinite(OMEGA_C)){OMEGA_C = 450.0f;}
	if(OMEGA_C < 50.0f){OMEGA_C = 50.0f;}
	if(OMEGA_C > 1200.0f){OMEGA_C = 1200.0f;}
	if(!isfinite(TUNE_MARGIN)){TUNE_MARGIN = 2.2f;}
	if(TUNE_MARGIN < 0.2f){TUNE_MARGIN = 0.2f;}
	if(TUNE_MARGIN > 5.0f){TUNE_MARGIN = 5.0f;}
}
