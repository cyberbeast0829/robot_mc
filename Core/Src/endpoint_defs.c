/*
 * endpoint_defs.c
 *
 * 由 tools/gen_endpoints.py 从 params.json 生成，请勿手工修改。
 */

#include <math.h>
#include <string.h>
#include "endpoint_defs.h"
#include "endpoint.h"
#include "user_config.h"
#include "hw_config.h"
#include "structs.h"
#include "timing.h"
#include "main.h"

const char endpoint_json[] =
"[{\"name\":\"config.i_bw\",\"id\":1,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"Hz\",\"min\":100.0,\"max\":2000.0,\"persist\":true},{\"name\":\"config.i_max\",\"id\":2,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"A\",\"min\":0.0,\"max\":4.0,\"persist\":true},{\"name\":\"config.i_fw_max\",\"id\":3,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"A\",\"min\":0.0,\"max\":33.0,\"persist\":true},{\"name\":\"config.i_max_cont\",\"id\":4,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"A\",\"min\":0.0,\"max\":40.0,\"persist\":true},{\"name\":\"config.i_cal\",\"id\":5,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"A\",\"min\":0.0,\"max\":20.0,\"persist\":true},{\"name\":\"config.theta_min\",\"id\":6,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"rad\",\"persist\":true},{\"name\":\"config.theta_max\",\"id\":7,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"rad\",\"persist\":true},{\"name\":\"config.p_max\",\"id\":8,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"ra"
"d\",\"min\":0.0,\"persist\":true},{\"name\":\"config.p_min\",\"id\":9,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"rad\",\"max\":0.0,\"persist\":true},{\"name\":\"config.v_max\",\"id\":10,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"rad/s\",\"min\":0.0,\"persist\":true},{\"name\":\"config.v_min\",\"id\":11,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"rad/s\",\"max\":0.0,\"persist\":true},{\"name\":\"config.kp_max\",\"id\":12,\"type\":\"float\",\"access\":\"rw\",\"min\":0.0,\"persist\":true},{\"name\":\"config.kd_max\",\"id\":13,\"type\":\"float\",\"access\":\"rw\",\"min\":0.0,\"persist\":true},{\"name\":\"config.gr\",\"id\":14,\"type\":\"float\",\"access\":\"rw\",\"min\":0.001,\"persist\":true},{\"name\":\"config.kt\",\"id\":15,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"N-m/A\",\"min\":0.0001,\"persist\":true},{\"name\":\"config.ppairs\",\"id\":16,\"type\":\"float\",\"access\":\"rw\",\"min\":1.0,\"max\":64.0,\"persist\":true},{\"name\":\"conf"
"ig.r_phase\",\"id\":17,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"Ohm\",\"persist\":true},{\"name\":\"config.r_nominal\",\"id\":18,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"Ohm\",\"persist\":true},{\"name\":\"config.r_th\",\"id\":19,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"C/W\",\"persist\":true},{\"name\":\"config.c_th\",\"id\":20,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"J/C\",\"persist\":true},{\"name\":\"config.temp_max\",\"id\":21,\"type\":\"float\",\"access\":\"rw\",\"unit\":\"C\",\"min\":0.0,\"max\":150.0,\"persist\":true},{\"name\":\"config.phase_order\",\"id\":22,\"type\":\"int32\",\"access\":\"rw\",\"min\":0,\"max\":1,\"persist\":true},{\"name\":\"config.can_id\",\"id\":23,\"type\":\"int32\",\"access\":\"rw\",\"min\":0,\"max\":127,\"persist\":true},{\"name\":\"config.can_master\",\"id\":24,\"type\":\"int32\",\"access\":\"rw\",\"min\":0,\"max\":127,\"persist\":true},{\"name\":\"config.can_timeout\",\"id\":25,\"type\":\"int3"
"2\",\"access\":\"rw\",\"unit\":\"ms\",\"min\":0,\"max\":100000,\"persist\":true},{\"name\":\"config.can_prot\",\"id\":120,\"type\":\"int32\",\"access\":\"rw\",\"min\":0,\"max\":1,\"persist\":true},{\"name\":\"config.can_baud\",\"id\":121,\"type\":\"int32\",\"access\":\"rw\",\"unit\":\"bit/s\",\"min\":50000,\"max\":5000000,\"persist\":true},{\"name\":\"config.can_hb_ms\",\"id\":122,\"type\":\"int32\",\"access\":\"rw\",\"unit\":\"ms\",\"min\":0,\"max\":10000,\"persist\":true},{\"name\":\"system.can_prot_active\",\"id\":123,\"type\":\"int32\",\"access\":\"r\"},{\"name\":\"system.can_fd\",\"id\":124,\"type\":\"bool\",\"access\":\"r\"},{\"name\":\"system.can_baud_active\",\"id\":125,\"type\":\"int32\",\"access\":\"r\",\"unit\":\"bit/s\"},{\"name\":\"config.m_zero\",\"id\":26,\"type\":\"int32\",\"access\":\"rw\",\"persist\":true},{\"name\":\"config.e_zero\",\"id\":27,\"type\":\"int32\",\"access\":\"rw\",\"persist\":true},{\"name\":\"telemetry.v_bus\",\"id\":2"
"8,\"type\":\"float\",\"access\":\"r\",\"unit\":\"V\"},{\"name\":\"telemetry.v_bus_raw\",\"id\":29,\"type\":\"float\",\"access\":\"r\",\"unit\":\"V\"},{\"name\":\"telemetry.i_a\",\"id\":30,\"type\":\"float\",\"access\":\"r\",\"unit\":\"A\"},{\"name\":\"telemetry.i_b\",\"id\":31,\"type\":\"float\",\"access\":\"r\",\"unit\":\"A\"},{\"name\":\"telemetry.i_c\",\"id\":32,\"type\":\"float\",\"access\":\"r\",\"unit\":\"A\"},{\"name\":\"telemetry.i_d\",\"id\":33,\"type\":\"float\",\"access\":\"r\",\"unit\":\"A\"},{\"name\":\"telemetry.i_q\",\"id\":34,\"type\":\"float\",\"access\":\"r\",\"unit\":\"A\"},{\"name\":\"telemetry.i_d_filt\",\"id\":35,\"type\":\"float\",\"access\":\"r\",\"unit\":\"A\"},{\"name\":\"telemetry.i_q_filt\",\"id\":36,\"type\":\"float\",\"access\":\"r\",\"unit\":\"A\"},{\"name\":\"telemetry.v_d\",\"id\":37,\"type\":\"float\",\"access\":\"r\",\"unit\":\"V\"},{\"name\":\"telemetry.v_q\",\"id\":38,\"type\":\"float\",\"access\":\"r\",\"unit\":\"V\"},{\"name\":\"telemetry.the"
"ta_mech\",\"id\":39,\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad\"},{\"name\":\"telemetry.theta_elec\",\"id\":40,\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad\"},{\"name\":\"telemetry.dtheta_mech\",\"id\":41,\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad/s\"},{\"name\":\"telemetry.dtheta_elec\",\"id\":42,\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad/s\"},{\"name\":\"telemetry.position\",\"id\":43,\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad\"},{\"name\":\"telemetry.position_motor\",\"id\":44,\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad\"},{\"name\":\"telemetry.velocity\",\"id\":45,\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad/s\"},{\"name\":\"telemetry.velocity_motor\",\"id\":46,\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad/s\"},{\"name\":\"telemetry.count\",\"id\":47,\"type\":\"int32\",\"access\":\"r\"},{\"name\":\"telemetry.torque\",\"id\":48,\"type\":\"float\",\"access\":\"r\",\"unit\":\"N-m\"},{\"name\":\"telemetry.p"
"_des\",\"id\":49,\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad\"},{\"name\":\"telemetry.v_des\",\"id\":50,\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad/s\"},{\"name\":\"telemetry.kp\",\"id\":51,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"telemetry.kd\",\"id\":52,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"telemetry.t_ff\",\"id\":53,\"type\":\"float\",\"access\":\"r\",\"unit\":\"N-m\"},{\"name\":\"telemetry.i_q_des\",\"id\":54,\"type\":\"float\",\"access\":\"r\",\"unit\":\"A\"},{\"name\":\"telemetry.fw_int\",\"id\":55,\"type\":\"float\",\"access\":\"r\",\"unit\":\"A\"},{\"name\":\"telemetry.fault\",\"id\":56,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"telemetry.gate_drive\",\"id\":57,\"type\":\"bool\",\"access\":\"r\"},{\"name\":\"telemetry.mode\",\"id\":58,\"type\":\"uint8\",\"access\":\"r\"},{\"name\":\"telemetry.loop_count\",\"id\":59,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"telemetry.can_timeout_count\",\"id\":60,\"type\":\"int3"
"2\",\"access\":\"r\"},{\"name\":\"telemetry.temperature\",\"id\":61,\"type\":\"float\",\"access\":\"r\",\"unit\":\"C\"},{\"name\":\"telemetry.resistance\",\"id\":62,\"type\":\"float\",\"access\":\"r\",\"unit\":\"Ohm\"},{\"name\":\"telemetry.can_tec\",\"id\":63,\"type\":\"uint8\",\"access\":\"r\"},{\"name\":\"telemetry.can_rec\",\"id\":64,\"type\":\"uint8\",\"access\":\"r\"},{\"name\":\"telemetry.can_lec\",\"id\":65,\"type\":\"uint8\",\"access\":\"r\"},{\"name\":\"telemetry.encoder_ppairs\",\"id\":66,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"telemetry.encoder_lost_frames\",\"id\":67,\"type\":\"int32\",\"access\":\"r\"},{\"name\":\"telemetry.encoder_raw\",\"id\":79,\"type\":\"int32\",\"access\":\"r\"},{\"name\":\"telemetry.encoder_turns\",\"id\":80,\"type\":\"int32\",\"access\":\"r\"},{\"name\":\"telemetry.encoder_elec_angle\",\"id\":81,\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad\"},{\"name\":\"telemetry.encoder_elec_velocity\",\"id\":82"
",\"type\":\"float\",\"access\":\"r\",\"unit\":\"rad/s\"},{\"name\":\"telemetry.dtc_u\",\"id\":83,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"telemetry.dtc_v\",\"id\":84,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"telemetry.dtc_w\",\"id\":85,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"telemetry.i_mag_max\",\"id\":86,\"type\":\"float\",\"access\":\"r\",\"unit\":\"A\"},{\"name\":\"telemetry.i_scale\",\"id\":87,\"type\":\"float\",\"access\":\"r\",\"unit\":\"A/count\"},{\"name\":\"telemetry.flux_linkage\",\"id\":88,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"telemetry.adc_a_offset\",\"id\":89,\"type\":\"int32\",\"access\":\"r\"},{\"name\":\"telemetry.adc_b_offset\",\"id\":90,\"type\":\"int32\",\"access\":\"r\"},{\"name\":\"telemetry.adc_c_offset\",\"id\":91,\"type\":\"int32\",\"access\":\"r\"},{\"name\":\"telemetry.adc_a_raw\",\"id\":92,\"type\":\"int32\",\"access\":\"r\"},{\"name\":\"telemetry.adc_b_raw\",\"id\":93,\"type\":\"int32\",\"acces"
"s\":\"r\"},{\"name\":\"telemetry.adc_c_raw\",\"id\":94,\"type\":\"int32\",\"access\":\"r\"},{\"name\":\"telemetry.observer_qd_in\",\"id\":95,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"telemetry.observer_qd_out\",\"id\":96,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"telemetry.observer_k\",\"id\":97,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"telemetry.observer_trust\",\"id\":98,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"telemetry.observer_delta_t\",\"id\":99,\"type\":\"float\",\"access\":\"r\",\"unit\":\"C\"},{\"name\":\"timing.isr_period\",\"id\":100,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.count\",\"id\":101,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.overrun\",\"id\":102,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.period_last\",\"id\":103,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.period_min\",\"id\":104,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.period_max\""
",\"id\":105,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.total_last\",\"id\":106,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.total_min\",\"id\":107,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.total_max\",\"id\":108,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.seg_adc_max\",\"id\":109,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.seg_enc_max\",\"id\":110,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.seg_faults_max\",\"id\":111,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.seg_fsm_max\",\"id\":112,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.seg_can_max\",\"id\":113,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.ext_enc_wait_max\",\"id\":114,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"timing.ext_adc_wait_max\",\"id\":115,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"system.version\",\"id\":68,\"type\":\"float\",\"access\":\"r\"},{\"name\":\"s"
"ystem.prefs_sequence\",\"id\":69,\"type\":\"uint32\",\"access\":\"r\"},{\"name\":\"system.config_schema\",\"id\":117,\"type\":\"uint16\",\"access\":\"r\"},{\"name\":\"system.prefs_loaded\",\"id\":70,\"type\":\"bool\",\"access\":\"r\"},{\"name\":\"system.save_status\",\"id\":119,\"type\":\"uint8\",\"access\":\"r\"},{\"name\":\"action.save\",\"id\":71,\"type\":\"function\",\"access\":\"w\",\"inputs\":[],\"outputs\":[]},{\"name\":\"action.zero\",\"id\":72,\"type\":\"function\",\"access\":\"w\",\"inputs\":[],\"outputs\":[]},{\"name\":\"action.enter_menu\",\"id\":73,\"type\":\"function\",\"access\":\"w\",\"inputs\":[],\"outputs\":[]},{\"name\":\"action.clear_fault\",\"id\":74,\"type\":\"function\",\"access\":\"w\",\"inputs\":[],\"outputs\":[]},{\"name\":\"action.motor_mode\",\"id\":75,\"type\":\"function\",\"access\":\"w\",\"inputs\":[],\"outputs\":[]},{\"name\":\"action.calibrate\",\"id\":76,\"type\":\"function\",\"access\":\"w\",\"inputs\":[],\"outp"
"uts\":[]},{\"name\":\"action.encoder_mode\",\"id\":77,\"type\":\"function\",\"access\":\"w\",\"inputs\":[],\"outputs\":[]},{\"name\":\"action.setup_mode\",\"id\":78,\"type\":\"function\",\"access\":\"w\",\"inputs\":[],\"outputs\":[]},{\"name\":\"action.reset_timing\",\"id\":116,\"type\":\"function\",\"access\":\"w\",\"inputs\":[],\"outputs\":[]}]";
const uint16_t endpoint_json_len = sizeof(endpoint_json) - 1;
const uint16_t endpoint_json_crc = 0x53B5;

ep_status_t endpoint_handler(uint16_t id, ep_op_t op, ep_cbuf_t *in, ep_buf_t *out){
	switch(id){
		case EP_CONFIG_I_BW: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				v = ep_clampf(v, 100.0, 2000.0);
				I_BW = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(I_BW))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_I_MAX: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				v = ep_clampf(v, 0.0, I_MAX_LIMIT);
				I_MAX = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(I_MAX))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_I_FW_MAX: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				v = ep_clampf(v, 0.0, 33.0);
				I_FW_MAX = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(I_FW_MAX))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_I_MAX_CONT: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				v = ep_clampf(v, 0.0, 40.0);
				I_MAX_CONT = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(I_MAX_CONT))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_I_CAL: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				v = ep_clampf(v, 0.0, 20.0);
				I_CAL = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(I_CAL))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_THETA_MIN: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				THETA_MIN = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(THETA_MIN))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_THETA_MAX: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				THETA_MAX = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(THETA_MAX))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_P_MAX: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				if(v < 0.0){v = 0.0;}
				P_MAX = v; P_MIN = -v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(P_MAX))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_P_MIN: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				if(v > 0.0){v = 0.0;}
				P_MIN = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(P_MIN))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_V_MAX: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				if(v < 0.0){v = 0.0;}
				V_MAX = v; V_MIN = -v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(V_MAX))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_V_MIN: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				if(v > 0.0){v = 0.0;}
				V_MIN = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(V_MIN))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_KP_MAX: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				if(v < 0.0){v = 0.0;}
				KP_MAX = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(KP_MAX))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_KD_MAX: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				if(v < 0.0){v = 0.0;}
				KD_MAX = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(KD_MAX))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_GR: {
			if(op == EP_OP_WRITE){
				if(controller.gate_drive_enabled){return EP_ERR_STATE;}
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				if(v < 0.001){v = 0.001;}
				GR = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(GR))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_KT: {
			if(op == EP_OP_WRITE){
				if(controller.gate_drive_enabled){return EP_ERR_STATE;}
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				if(v < 0.0001){v = 0.0001;}
				KT = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(KT))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_PPAIRS: {
			if(op == EP_OP_WRITE){
				if(controller.gate_drive_enabled){return EP_ERR_STATE;}
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				v = ep_clampf(v, 1.0, 64.0);
				PPAIRS = v; comm_encoder.ppairs = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(PPAIRS))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_R_PHASE: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				R_PHASE = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(R_PHASE))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_R_NOMINAL: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				R_NOMINAL = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(R_NOMINAL))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_R_TH: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				R_TH = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(R_TH))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_C_TH: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				C_TH = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(C_TH))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_TEMP_MAX: {
			if(op == EP_OP_WRITE){
				float v;
				if(!ep_get_f32(in, &v)){return EP_ERR_BAD_LEN;}
				if(!isfinite(v)){return EP_ERR_BAD_VALUE;}
				v = ep_clampf(v, 0.0, 150.0);
				TEMP_MAX = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_f32(out, (float)(TEMP_MAX))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_PHASE_ORDER: {
			if(op == EP_OP_WRITE){
				if(controller.gate_drive_enabled){return EP_ERR_STATE;}
				int32_t v;
				if(!ep_get_i32(in, &v)){return EP_ERR_BAD_LEN;}
				v = (int32_t)ep_clampi(v, 0, 1);
				PHASE_ORDER = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_i32(out, (int32_t)(PHASE_ORDER))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_CAN_ID: {
			if(op == EP_OP_WRITE){
				int32_t v;
				if(!ep_get_i32(in, &v)){return EP_ERR_BAD_LEN;}
				v = (int32_t)ep_clampi(v, 0, 127);
				CAN_ID = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_i32(out, (int32_t)(CAN_ID))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_CAN_MASTER: {
			if(op == EP_OP_WRITE){
				int32_t v;
				if(!ep_get_i32(in, &v)){return EP_ERR_BAD_LEN;}
				v = (int32_t)ep_clampi(v, 0, 127);
				CAN_MASTER = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_i32(out, (int32_t)(CAN_MASTER))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_CAN_TIMEOUT: {
			if(op == EP_OP_WRITE){
				int32_t v;
				if(!ep_get_i32(in, &v)){return EP_ERR_BAD_LEN;}
				v = (int32_t)ep_clampi(v, 0, 100000);
				CAN_TIMEOUT = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_i32(out, (int32_t)(CAN_TIMEOUT))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_CAN_PROT: {
			if(op == EP_OP_WRITE){
				int32_t v;
				if(!ep_get_i32(in, &v)){return EP_ERR_BAD_LEN;}
				v = (int32_t)ep_clampi(v, 0, 1);
				CAN_PROT = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_i32(out, (int32_t)(CAN_PROT))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_CAN_BAUD: {
			if(op == EP_OP_WRITE){
				int32_t v;
				if(!ep_get_i32(in, &v)){return EP_ERR_BAD_LEN;}
				v = (int32_t)ep_clampi(v, 50000, 5000000);
				CAN_BAUD = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_i32(out, (int32_t)(CAN_BAUD))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_CAN_HB_MS: {
			if(op == EP_OP_WRITE){
				int32_t v;
				if(!ep_get_i32(in, &v)){return EP_ERR_BAD_LEN;}
				v = (int32_t)ep_clampi(v, 0, 10000);
				CAN_HB_MS = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_i32(out, (int32_t)(CAN_HB_MS))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_SYSTEM_CAN_PROT_ACTIVE: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(can_active_prot()))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_SYSTEM_CAN_FD: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u8(out, (uint8_t)(can_is_fd()))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_SYSTEM_CAN_BAUD_ACTIVE: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(can_active_baud()))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_M_ZERO: {
			if(op == EP_OP_WRITE){
				if(controller.gate_drive_enabled){return EP_ERR_STATE;}
				int32_t v;
				if(!ep_get_i32(in, &v)){return EP_ERR_BAD_LEN;}
				M_ZERO = v; comm_encoder.m_zero = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_i32(out, (int32_t)(M_ZERO))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_CONFIG_E_ZERO: {
			if(op == EP_OP_WRITE){
				if(controller.gate_drive_enabled){return EP_ERR_STATE;}
				int32_t v;
				if(!ep_get_i32(in, &v)){return EP_ERR_BAD_LEN;}
				E_ZERO = v; comm_encoder.e_zero = v;
			}
			if(in->begin != in->end){return EP_ERR_BAD_LEN;}
			if(!ep_put_i32(out, (int32_t)(E_ZERO))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_V_BUS: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.v_bus_filt))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_V_BUS_RAW: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.v_bus))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_I_A: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.i_a))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_I_B: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.i_b))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_I_C: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.i_c))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_I_D: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.i_d))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_I_Q: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.i_q))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_I_D_FILT: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.i_d_filt))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_I_Q_FILT: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.i_q_filt))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_V_D: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.v_d))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_V_Q: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.v_q))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_THETA_MECH: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.theta_mech))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_THETA_ELEC: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.theta_elec))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_DTHETA_MECH: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.dtheta_mech))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_DTHETA_ELEC: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.dtheta_elec))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_POSITION: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(comm_encoder.angle_multiturn / GR))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_POSITION_MOTOR: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(comm_encoder.angle_multiturn))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_VELOCITY: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(comm_encoder.velocity / GR))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_VELOCITY_MOTOR: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(comm_encoder.velocity))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_COUNT: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(comm_encoder.count))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_TORQUE: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.i_q_filt * KT * GR))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_P_DES: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.p_des))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_V_DES: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.v_des))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_KP: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.kp))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_KD: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.kd))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_T_FF: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.t_ff))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_I_Q_DES: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.i_q_des))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_FW_INT: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.fw_int))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_FAULT: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(controller.fault))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_GATE_DRIVE: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u8(out, (uint8_t)(controller.gate_drive_enabled))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_MODE: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u8(out, (uint8_t)(state.state))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_LOOP_COUNT: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(controller.loop_count))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_CAN_TIMEOUT_COUNT: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(controller.timeout))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_TEMPERATURE: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(observer.temp_measured))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_RESISTANCE: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(observer.resistance))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_CAN_TEC: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u8(out, (uint8_t)((FDCAN2->ECR & FDCAN_ECR_TEC)))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_CAN_REC: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u8(out, (uint8_t)(((FDCAN2->ECR & FDCAN_ECR_REC) >> 8)))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_CAN_LEC: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u8(out, (uint8_t)((FDCAN2->PSR & FDCAN_PSR_LEC)))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ENCODER_PPAIRS: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(comm_encoder.ppairs))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ENCODER_LOST_FRAMES: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(comm_encoder.lost_frames))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ENCODER_RAW: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(comm_encoder.raw))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ENCODER_TURNS: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(comm_encoder.turns))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ENCODER_ELEC_ANGLE: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(comm_encoder.elec_angle))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ENCODER_ELEC_VELOCITY: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(comm_encoder.elec_velocity))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_DTC_U: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.dtc_u))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_DTC_V: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.dtc_v))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_DTC_W: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.dtc_w))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_I_MAG_MAX: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.i_mag_max))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_I_SCALE: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.i_scale))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_FLUX_LINKAGE: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(controller.flux_linkage))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ADC_A_OFFSET: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(controller.adc_a_offset))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ADC_B_OFFSET: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(controller.adc_b_offset))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ADC_C_OFFSET: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(controller.adc_c_offset))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ADC_A_RAW: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(controller.adc_a_raw))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ADC_B_RAW: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(controller.adc_b_raw))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_ADC_C_RAW: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_i32(out, (int32_t)(controller.adc_c_raw))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_OBSERVER_QD_IN: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(observer.qd_in))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_OBSERVER_QD_OUT: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(observer.qd_out))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_OBSERVER_K: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(observer.k))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_OBSERVER_TRUST: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(observer.trust))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TELEMETRY_OBSERVER_DELTA_T: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(observer.delta_t))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_ISR_PERIOD: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.isr_period))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_COUNT: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.n))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_OVERRUN: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.overrun))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_PERIOD_LAST: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.period_last))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_PERIOD_MIN: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.period_min))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_PERIOD_MAX: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.period_max))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_TOTAL_LAST: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.total_last))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_TOTAL_MIN: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.total_min))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_TOTAL_MAX: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.total_max))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_SEG_ADC_MAX: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.seg_max[ISR_SEG_ADC]))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_SEG_ENC_MAX: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.seg_max[ISR_SEG_ENC]))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_SEG_FAULTS_MAX: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.seg_max[ISR_SEG_FAULTS]))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_SEG_FSM_MAX: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.seg_max[ISR_SEG_FSM]))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_SEG_CAN_MAX: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.seg_max[ISR_SEG_CAN]))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_EXT_ENC_WAIT_MAX: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.ext_max[ISR_EXT_ENC_WAIT]))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_TIMING_EXT_ADC_WAIT_MAX: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(isr_timing.ext_max[ISR_EXT_ADC_WAIT]))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_SYSTEM_VERSION: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_f32(out, (float)(VERSION_NUM))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_SYSTEM_PREFS_SEQUENCE: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u32(out, (uint32_t)(prefs.sequence))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_SYSTEM_CONFIG_SCHEMA: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u16(out, (uint16_t)(prefs.schema))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_SYSTEM_PREFS_LOADED: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u8(out, (uint8_t)(prefs.loaded))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_SYSTEM_SAVE_STATUS: {
			if(op != EP_OP_READ){return EP_ERR_READ_ONLY;}
			if(!ep_put_u8(out, (uint8_t)(state.save_status))){return EP_ERR_BAD_LEN;}
			return EP_OK;
		}
		case EP_ACTION_SAVE: {
			if(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}
			if(controller.gate_drive_enabled){return EP_ERR_STATE;}
			state.save_status = 0; state.save_pending = 1;
			return EP_OK;
		}
		case EP_ACTION_ZERO: {
			if(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}
			if(!(state.state == MENU_MODE)){return EP_ERR_STATE;}
			state.zero_pending = 1;
			return EP_OK;
		}
		case EP_ACTION_ENTER_MENU: {
			if(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}
			request_fsm_state(&state, MENU_MODE);
			return EP_OK;
		}
		case EP_ACTION_CLEAR_FAULT: {
			if(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}
			request_fsm_state(&state, MENU_MODE);
			return EP_OK;
		}
		case EP_ACTION_MOTOR_MODE: {
			if(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}
			if(!(state.state == MENU_MODE)){return EP_ERR_STATE;}
			request_fsm_state(&state, MOTOR_MODE);
			return EP_OK;
		}
		case EP_ACTION_CALIBRATE: {
			if(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}
			if(!(state.state == MENU_MODE)){return EP_ERR_STATE;}
			request_fsm_state(&state, CALIBRATION_MODE);
			return EP_OK;
		}
		case EP_ACTION_ENCODER_MODE: {
			if(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}
			if(!(state.state == MENU_MODE)){return EP_ERR_STATE;}
			request_fsm_state(&state, ENCODER_MODE);
			return EP_OK;
		}
		case EP_ACTION_SETUP_MODE: {
			if(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}
			if(!(state.state == MENU_MODE)){return EP_ERR_STATE;}
			request_fsm_state(&state, SETUP_MODE);
			return EP_OK;
		}
		case EP_ACTION_RESET_TIMING: {
			if(op != EP_OP_WRITE){return EP_ERR_WRITE_ONLY;}
			isr_timing.reset_req = 1;
			return EP_OK;
		}
		default:
			return EP_ERR_UNKNOWN_ID;
	}
}
