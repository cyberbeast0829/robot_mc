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
#include "cogging.h"

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
	{162, PREFS_T_I32, 4, (void *)&(CAN_TDC)},
	{163, PREFS_T_F32, 4, (void *)&(VBUS_OV_TRIP)},
	{164, PREFS_T_F32, 4, (void *)&(VBUS_UV_TRIP)},
	{165, PREFS_T_F32, 4, (void *)&(I_DC_MAX)},
	{166, PREFS_T_F32, 4, (void *)&(TORQUE_LIM)},
	{167, PREFS_T_BOOL, 1, (void *)&(FET_TEMP_EN)},
	{168, PREFS_T_F32, 4, (void *)&(FET_TEMP_LO)},
	{169, PREFS_T_F32, 4, (void *)&(FET_TEMP_HI)},
	{170, PREFS_T_BOOL, 1, (void *)&(MOT_TEMP_EN)},
	{171, PREFS_T_F32, 4, (void *)&(MOT_TEMP_LO)},
	{172, PREFS_T_F32, 4, (void *)&(MOT_TEMP_HI)},
	{173, PREFS_T_BOOL, 1, (void *)&(PRE_CALIBRATED)},
	{181, PREFS_T_F32, 4, (void *)&(DECEL_LIMIT)},
	{182, PREFS_T_I32, 4, (void *)&(INPUT_MODE)},
	{183, PREFS_T_F32, 4, (void *)&(INPUT_BW)},
	{184, PREFS_T_F32, 4, (void *)&(TORQUE_RAMP_RATE)},
	{185, PREFS_T_BOOL, 1, (void *)&(VEL_LIMIT_EN)},
	{186, PREFS_T_BOOL, 1, (void *)&(TORQUE_VEL_LIM_EN)},
	{195, PREFS_T_I32, 4, (void *)&(GEAR_TEETH_FIRST)},
	{196, PREFS_T_I32, 4, (void *)&(GEAR_TEETH_SEC)},
	{198, PREFS_T_F32, 4, (void *)&(GEAR_ZERO_PHASE)},
	{199, PREFS_T_I32, 4, (void *)&(GEAR_ZERO_COUNT)},
	{200, PREFS_T_BOOL, 1, (void *)&(GEAR_REF_VALID)},
	{208, PREFS_T_BOOL, 1, (void *)&(GEAR_SEC_REVERSED)},
	{210, PREFS_T_F32, 4, (void *)&(VEL_RAMP_RATE)},
	{211, PREFS_T_F32, 4, (void *)&(TRAP_VEL_LIMIT)},
	{212, PREFS_T_F32, 4, (void *)&(TRAP_ACCEL_LIMIT)},
	{213, PREFS_T_F32, 4, (void *)&(TRAP_DECEL_LIMIT)},
	{218, PREFS_T_BOOL, 1, (void *)&(COGGING_EN)},
	{219, PREFS_T_BOOL, 1, (void *)&(COGGING_VALID)},
	{220, PREFS_T_F32, 4, (void *)&(COG_GAIN)},
	{221, PREFS_T_F32, 4, (void *)&(COG_MAX)},
	{222, PREFS_T_F32, 4, (void *)&(COG_POS_THR)},
	{223, PREFS_T_I32, 4, (void *)&(COG_SETTLE_MS)},
	{224, PREFS_T_I32, 4, (void *)&(COG_AVG)},
	{236, PREFS_T_BLOB, 2048, (void *)&(COGGING_LUT)},
	{237, PREFS_T_I32, 4, (void *)&(COG_DETREND)},
	{238, PREFS_T_I32, 4, (void *)&(COG_METHOD)},
	{239, PREFS_T_I32, 4, (void *)&(COG_SWEEP_MS)},
	{243, PREFS_T_F32, 4, (void *)&(MIT_TAU_MAX)},
	{266, PREFS_T_F32, 4, (void *)&(DT_COMP)},
	{267, PREFS_T_F32, 4, (void *)&(DT_COMP_TOL)},
};
const unsigned prefs_entry_count = sizeof(prefs_entries) / sizeof(prefs_entries[0]);

_Static_assert(sizeof(prefs_entries) / sizeof(prefs_entries[0]) <= 256u,
            "params.json: 持久化条目超过 PREFS_MAX_RECORDS");
_Static_assert(N_LUT * sizeof(int) == 512, "internal.encoder_lut: 尺寸与固件不一致");
_Static_assert(COG_N * sizeof(int16_t) == 2048, "internal.cogging_lut: 尺寸与固件不一致");

void config_apply_defaults(void){
	I_BW = 2000.0f;
	I_MAX = I_MAX_LIMIT;
	I_FW_MAX = 0.0f;
	I_MAX_CONT = 14.0f;
	I_CAL = 5.0f;
	THETA_MIN = 0.0f;
	THETA_MAX = 0.0f;
	P_MAX = 0.0f;
	P_MIN = 0.0f;
	V_MAX = 65.0f;
	V_MIN = -65.0f;
	KP_MAX = 500.0f;
	KD_MAX = 5.0f;
	GR = 9.0f;
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
	CAN_PROT = 1;
	CAN_BAUD = 1000000;
	CAN_HB_MS = 100;
	L_D = 9.5e-05f;
	L_Q = 0.000124f;
	POS_GAIN = 20.0f;
	VEL_GAIN = 1.397f;
	VEL_INTEGRATOR_GAIN = 25.152f;
	POS_FF_GAIN = 1.0f;
	ACCEL_LIMIT = 200.0f;
	POS_ERROR_FAULT = 0.0f;
	PLL_BW = 3000.0f;
	VEL_LOOP_PRESCALER = 5;
	POS_LOOP_PRESCALER = 2;
	INERTIA = 0.000996f;
	OMEGA_C = 450.0f;
	TUNE_MARGIN = 0.94f;
	CAN_TDC = -1;
	VBUS_OV_TRIP = 67.0f;
	VBUS_UV_TRIP = 0.0f;
	I_DC_MAX = 0.0f;
	TORQUE_LIM = 3.0f;
	FET_TEMP_EN = 1;
	FET_TEMP_LO = 75.0f;
	FET_TEMP_HI = 95.0f;
	MOT_TEMP_EN = 1;
	MOT_TEMP_LO = 75.0f;
	MOT_TEMP_HI = 95.0f;
	PRE_CALIBRATED = 0;
	DECEL_LIMIT = 200.0f;
	INPUT_MODE = 2;
	INPUT_BW = 10.0f;
	TORQUE_RAMP_RATE = 1.0f;
	VEL_LIMIT_EN = 1;
	TORQUE_VEL_LIM_EN = 0;
	GEAR_TEETH_FIRST = 30;
	GEAR_TEETH_SEC = 27;
	GEAR_ZERO_PHASE = 0.0f;
	GEAR_ZERO_COUNT = 0;
	GEAR_REF_VALID = 0;
	GEAR_SEC_REVERSED = 1;
	VEL_RAMP_RATE = 0.0f;
	TRAP_VEL_LIMIT = 0.0f;
	TRAP_ACCEL_LIMIT = 0.0f;
	TRAP_DECEL_LIMIT = 0.0f;
	COGGING_EN = 0;
	COGGING_VALID = 0;
	COG_GAIN = 1.0f;
	COG_MAX = 1.0f;
	COG_POS_THR = 0.0003f;
	COG_SETTLE_MS = 40;
	COG_AVG = 80;
	memset((void *)&(COGGING_LUT), 0, 2048);
	COG_DETREND = 3;
	COG_METHOD = 1;
	COG_SWEEP_MS = 8000;
	MIT_TAU_MAX = 5.0f;
	DT_COMP = 0.008f;
	DT_COMP_TOL = 0.5f;
}

void config_apply_limits(void){
	if(!isfinite(I_BW)){I_BW = 2000.0f;}
	if(I_BW < 100.0f){I_BW = 100.0f;}
	if(I_BW > 4000.0f){I_BW = 4000.0f;}
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
	if(!isfinite(P_MAX)){P_MAX = 0.0f;}
	if(P_MAX < 0.0f){P_MAX = 0.0f;}
	if(!isfinite(P_MIN)){P_MIN = 0.0f;}
	if(P_MIN > 0.0f){P_MIN = 0.0f;}
	if(!isfinite(V_MAX)){V_MAX = 65.0f;}
	if(V_MAX < 0.0f){V_MAX = 0.0f;}
	if(!isfinite(V_MIN)){V_MIN = -65.0f;}
	if(V_MIN > 0.0f){V_MIN = 0.0f;}
	if(!isfinite(KP_MAX)){KP_MAX = 500.0f;}
	if(KP_MAX < 0.0f){KP_MAX = 0.0f;}
	if(!isfinite(KD_MAX)){KD_MAX = 5.0f;}
	if(KD_MAX < 0.0f){KD_MAX = 0.0f;}
	if(!isfinite(GR)){GR = 9.0f;}
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
	if(!isfinite(POS_GAIN)){POS_GAIN = 20.0f;}
	if(POS_GAIN < 0.0f){POS_GAIN = 0.0f;}
	if(POS_GAIN > 1000.0f){POS_GAIN = 1000.0f;}
	if(!isfinite(VEL_GAIN)){VEL_GAIN = 1.397f;}
	if(VEL_GAIN < 0.0f){VEL_GAIN = 0.0f;}
	if(VEL_GAIN > 2000.0f){VEL_GAIN = 2000.0f;}
	if(!isfinite(VEL_INTEGRATOR_GAIN)){VEL_INTEGRATOR_GAIN = 25.152f;}
	if(VEL_INTEGRATOR_GAIN < 0.0f){VEL_INTEGRATOR_GAIN = 0.0f;}
	if(VEL_INTEGRATOR_GAIN > 200000.0f){VEL_INTEGRATOR_GAIN = 200000.0f;}
	if(!isfinite(POS_FF_GAIN)){POS_FF_GAIN = 1.0f;}
	if(POS_FF_GAIN < 0.0f){POS_FF_GAIN = 0.0f;}
	if(POS_FF_GAIN > 2.0f){POS_FF_GAIN = 2.0f;}
	if(!isfinite(ACCEL_LIMIT)){ACCEL_LIMIT = 200.0f;}
	if(ACCEL_LIMIT < 0.0f){ACCEL_LIMIT = 0.0f;}
	if(ACCEL_LIMIT > 100000.0f){ACCEL_LIMIT = 100000.0f;}
	if(!isfinite(POS_ERROR_FAULT)){POS_ERROR_FAULT = 0.0f;}
	if(POS_ERROR_FAULT < 0.0f){POS_ERROR_FAULT = 0.0f;}
	if(POS_ERROR_FAULT > 1000.0f){POS_ERROR_FAULT = 1000.0f;}
	if(!isfinite(PLL_BW)){PLL_BW = 3000.0f;}
	if(PLL_BW < 20.0f){PLL_BW = 20.0f;}
	if(PLL_BW > 5000.0f){PLL_BW = 5000.0f;}
	if(VEL_LOOP_PRESCALER < 1){VEL_LOOP_PRESCALER = 1;}
	if(VEL_LOOP_PRESCALER > 100){VEL_LOOP_PRESCALER = 100;}
	if(POS_LOOP_PRESCALER < 1){POS_LOOP_PRESCALER = 1;}
	if(POS_LOOP_PRESCALER > 100){POS_LOOP_PRESCALER = 100;}
	if(!isfinite(INERTIA)){INERTIA = 0.000996f;}
	if(INERTIA < 0.0f){INERTIA = 0.0f;}
	if(INERTIA > 100.0f){INERTIA = 100.0f;}
	if(!isfinite(OMEGA_C)){OMEGA_C = 450.0f;}
	if(OMEGA_C < 50.0f){OMEGA_C = 50.0f;}
	if(OMEGA_C > 1200.0f){OMEGA_C = 1200.0f;}
	if(!isfinite(TUNE_MARGIN)){TUNE_MARGIN = 0.94f;}
	if(TUNE_MARGIN < 0.2f){TUNE_MARGIN = 0.2f;}
	if(TUNE_MARGIN > 5.0f){TUNE_MARGIN = 5.0f;}
	if(CAN_TDC < -1){CAN_TDC = -1;}
	if(CAN_TDC > 31){CAN_TDC = 31;}
	if(!isfinite(VBUS_OV_TRIP)){VBUS_OV_TRIP = 67.0f;}
	if(VBUS_OV_TRIP < 10.0f){VBUS_OV_TRIP = 10.0f;}
	if(VBUS_OV_TRIP > 100.0f){VBUS_OV_TRIP = 100.0f;}
	if(!isfinite(VBUS_UV_TRIP)){VBUS_UV_TRIP = 0.0f;}
	if(VBUS_UV_TRIP < 0.0f){VBUS_UV_TRIP = 0.0f;}
	if(VBUS_UV_TRIP > 100.0f){VBUS_UV_TRIP = 100.0f;}
	if(!isfinite(I_DC_MAX)){I_DC_MAX = 0.0f;}
	if(I_DC_MAX < 0.0f){I_DC_MAX = 0.0f;}
	if(I_DC_MAX > 100.0f){I_DC_MAX = 100.0f;}
	if(!isfinite(TORQUE_LIM)){TORQUE_LIM = 3.0f;}
	if(TORQUE_LIM < 0.0f){TORQUE_LIM = 0.0f;}
	if(TORQUE_LIM > 100.0f){TORQUE_LIM = 100.0f;}
	if(FET_TEMP_EN > 1){FET_TEMP_EN = 1;}
	if(!isfinite(FET_TEMP_LO)){FET_TEMP_LO = 75.0f;}
	if(FET_TEMP_LO < 0.0f){FET_TEMP_LO = 0.0f;}
	if(FET_TEMP_LO > 200.0f){FET_TEMP_LO = 200.0f;}
	if(!isfinite(FET_TEMP_HI)){FET_TEMP_HI = 95.0f;}
	if(FET_TEMP_HI < 0.0f){FET_TEMP_HI = 0.0f;}
	if(FET_TEMP_HI > 200.0f){FET_TEMP_HI = 200.0f;}
	if(MOT_TEMP_EN > 1){MOT_TEMP_EN = 1;}
	if(!isfinite(MOT_TEMP_LO)){MOT_TEMP_LO = 75.0f;}
	if(MOT_TEMP_LO < 0.0f){MOT_TEMP_LO = 0.0f;}
	if(MOT_TEMP_LO > 200.0f){MOT_TEMP_LO = 200.0f;}
	if(!isfinite(MOT_TEMP_HI)){MOT_TEMP_HI = 95.0f;}
	if(MOT_TEMP_HI < 0.0f){MOT_TEMP_HI = 0.0f;}
	if(MOT_TEMP_HI > 200.0f){MOT_TEMP_HI = 200.0f;}
	if(PRE_CALIBRATED > 1){PRE_CALIBRATED = 1;}
	if(!isfinite(DECEL_LIMIT)){DECEL_LIMIT = 200.0f;}
	if(DECEL_LIMIT < 0.0f){DECEL_LIMIT = 0.0f;}
	if(DECEL_LIMIT > 100000.0f){DECEL_LIMIT = 100000.0f;}
	if(INPUT_MODE < 0){INPUT_MODE = 0;}
	if(INPUT_MODE > 9){INPUT_MODE = 9;}
	if(!isfinite(INPUT_BW)){INPUT_BW = 10.0f;}
	if(INPUT_BW < 1.0f){INPUT_BW = 1.0f;}
	if(INPUT_BW > 200.0f){INPUT_BW = 200.0f;}
	if(!isfinite(TORQUE_RAMP_RATE)){TORQUE_RAMP_RATE = 1.0f;}
	if(TORQUE_RAMP_RATE < 0.0f){TORQUE_RAMP_RATE = 0.0f;}
	if(TORQUE_RAMP_RATE > 10000.0f){TORQUE_RAMP_RATE = 10000.0f;}
	if(VEL_LIMIT_EN > 1){VEL_LIMIT_EN = 1;}
	if(TORQUE_VEL_LIM_EN > 1){TORQUE_VEL_LIM_EN = 1;}
	if(GEAR_TEETH_FIRST < 1){GEAR_TEETH_FIRST = 1;}
	if(GEAR_TEETH_FIRST > 400){GEAR_TEETH_FIRST = 400;}
	if(GEAR_TEETH_SEC < 1){GEAR_TEETH_SEC = 1;}
	if(GEAR_TEETH_SEC > 400){GEAR_TEETH_SEC = 400;}
	if(!isfinite(GEAR_ZERO_PHASE)){GEAR_ZERO_PHASE = 0.0f;}
	if(GEAR_ZERO_PHASE < 0.0f){GEAR_ZERO_PHASE = 0.0f;}
	if(GEAR_ZERO_PHASE > 1.0f){GEAR_ZERO_PHASE = 1.0f;}
	if(GEAR_REF_VALID > 1){GEAR_REF_VALID = 1;}
	if(GEAR_SEC_REVERSED > 1){GEAR_SEC_REVERSED = 1;}
	if(!isfinite(VEL_RAMP_RATE)){VEL_RAMP_RATE = 0.0f;}
	if(VEL_RAMP_RATE < 0.0f){VEL_RAMP_RATE = 0.0f;}
	if(VEL_RAMP_RATE > 100000.0f){VEL_RAMP_RATE = 100000.0f;}
	if(!isfinite(TRAP_VEL_LIMIT)){TRAP_VEL_LIMIT = 0.0f;}
	if(TRAP_VEL_LIMIT < 0.0f){TRAP_VEL_LIMIT = 0.0f;}
	if(TRAP_VEL_LIMIT > 200.0f){TRAP_VEL_LIMIT = 200.0f;}
	if(!isfinite(TRAP_ACCEL_LIMIT)){TRAP_ACCEL_LIMIT = 0.0f;}
	if(TRAP_ACCEL_LIMIT < 0.0f){TRAP_ACCEL_LIMIT = 0.0f;}
	if(TRAP_ACCEL_LIMIT > 100000.0f){TRAP_ACCEL_LIMIT = 100000.0f;}
	if(!isfinite(TRAP_DECEL_LIMIT)){TRAP_DECEL_LIMIT = 0.0f;}
	if(TRAP_DECEL_LIMIT < 0.0f){TRAP_DECEL_LIMIT = 0.0f;}
	if(TRAP_DECEL_LIMIT > 100000.0f){TRAP_DECEL_LIMIT = 100000.0f;}
	if(COGGING_EN > 1){COGGING_EN = 1;}
	if(COGGING_VALID > 1){COGGING_VALID = 1;}
	if(!isfinite(COG_GAIN)){COG_GAIN = 1.0f;}
	if(COG_GAIN < 0.0f){COG_GAIN = 0.0f;}
	if(COG_GAIN > 2.0f){COG_GAIN = 2.0f;}
	if(!isfinite(COG_MAX)){COG_MAX = 1.0f;}
	if(COG_MAX < 0.0f){COG_MAX = 0.0f;}
	if(COG_MAX > 10.0f){COG_MAX = 10.0f;}
	if(!isfinite(COG_POS_THR)){COG_POS_THR = 0.0003f;}
	if(COG_POS_THR < 1e-05f){COG_POS_THR = 1e-05f;}
	if(COG_POS_THR > 0.01f){COG_POS_THR = 0.01f;}
	if(COG_SETTLE_MS < 0){COG_SETTLE_MS = 0;}
	if(COG_SETTLE_MS > 500){COG_SETTLE_MS = 500;}
	if(COG_AVG < 1){COG_AVG = 1;}
	if(COG_AVG > 400){COG_AVG = 400;}
	if(COG_DETREND < 0){COG_DETREND = 0;}
	if(COG_DETREND > 6){COG_DETREND = 6;}
	if(COG_METHOD < 0){COG_METHOD = 0;}
	if(COG_METHOD > 1){COG_METHOD = 1;}
	if(COG_SWEEP_MS < 500){COG_SWEEP_MS = 500;}
	if(COG_SWEEP_MS > 20000){COG_SWEEP_MS = 20000;}
	if(!isfinite(MIT_TAU_MAX)){MIT_TAU_MAX = 5.0f;}
	if(MIT_TAU_MAX < 0.0f){MIT_TAU_MAX = 0.0f;}
	if(MIT_TAU_MAX > 40.0f){MIT_TAU_MAX = 40.0f;}
	if(!isfinite(DT_COMP)){DT_COMP = 0.008f;}
	if(DT_COMP < 0.0f){DT_COMP = 0.0f;}
	if(DT_COMP > 0.012f){DT_COMP = 0.012f;}
	if(!isfinite(DT_COMP_TOL)){DT_COMP_TOL = 0.5f;}
	if(DT_COMP_TOL < 0.01f){DT_COMP_TOL = 0.01f;}
	if(DT_COMP_TOL > 5.0f){DT_COMP_TOL = 5.0f;}
}
