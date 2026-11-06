/*
 * fsm.cpp
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */

#include "fsm.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "user_config.h"
#include "hw_config.h"
#include "structs.h"
#include "foc.h"
#include "math_ops.h"
#include "position_sensor.h"
#include "prefs_defs.h"		// config_apply_defaults() / config_apply_limits()

extern uint32_t _reboot_cookie;

/* 重启前给上位机留的窗口（等 USB/CAN 应答发出去）。控制环 40 kHz ⇒ 20000 拍 = 0.5 s */
#define REBOOT_DELAY_CYCLES     20000u

/* 惯量标定每个 action 连跑几轮取中位。单次读数受齿隙/摩擦与突破电流档位影响，
 * 台架实测离散 ±50%，中位数才稳（见 docs/DEV.md"惯量标定"一节） */
#define INERTIA_RUNS            3u

 static int save_preferences(void);

 /* n 个样本的中位数（n = 1/2/3；2 个时取平均） */
 static float inertia_median(const float *a, uint8_t n){
	 if(n == 1u){return a[0];}
	 if(n == 2u){return 0.5f*(a[0] + a[1]);}
	 float x = a[0], y = a[1], z = a[2];
	 if(x > y){float t = x; x = y; y = t;}
	 if(y > z){float t = y; y = z; z = t;}
	 if(x > y){float t = x; x = y; y = t;}
	 return y;
 }


 void run_fsm(FSMStruct * fsmstate){
	 /* run_fsm is run every commutation interrupt cycle */

	 /* A latched fault forces the fault state, from any state.  Clearing it
	  * requires explicitly exiting to the menu (esc or the CAN menu command) */
	 if((controller.fault != 0) && (fsmstate->state != FAULT_MODE) && (fsmstate->next_state != FAULT_MODE)){
		 fsmstate->next_state = FAULT_MODE;
		 fsmstate->ready = 0;
	 }

	 /* 未标定（config.pre_calibrated == false）就不许打开门极：电角度零点、极对数和
	  * 相序都还没测过，闭环只会乱转甚至倒灌。这里是**所有**入口的兜底
	  * （ASCII 菜单、CYBERBEAST 的 START_MOTOR、endpoint action.*）；
	  * endpoint 侧另有 require 会直接返回 EP_ERR_STATE，用户能立刻看到原因。
	  * 标定成功后 PRE_CALIBRATED 置 1，进标定/恢复默认时清 0。 */
	 if((fsmstate->next_state == MOTOR_MODE) && (fsmstate->state != MOTOR_MODE) && !PRE_CALIBRATED){
		 fsmstate->next_state = fsmstate->state;	// 留在原地（通常是菜单）
		 fsmstate->ready = 0;
		 fsmstate->motor_refused = 1;			// 主循环打印一次
	 }

	 /* state transition management */
	 if(fsmstate->next_state != fsmstate->state){
		 fsm_exit_state(fsmstate);		// safely exit the old state
		 if(fsmstate->ready){			// if the previous state is ready, enter the new state
			 fsmstate->state = fsmstate->next_state;
			 fsm_enter_state(fsmstate);
		 }
	 }

	 switch(fsmstate->state){
		 case MENU_MODE:
			 break;

		 case CALIBRATION_MODE:
			 if(comm_encoder_cal.failed){
				 update_fsm(fsmstate, MENU_CMD);	// abort back to the menu, which disables the gate drive on exit
			 }
			 else if(!comm_encoder_cal.done_ordering){
				 order_phases(&comm_encoder, &controller, &comm_encoder_cal, controller.loop_count);
			 }
			 else if(!comm_encoder_cal.done_cal){
				 calibrate_encoder(&comm_encoder, &controller, &comm_encoder_cal, controller.loop_count);
			 }
			 else if(!comm_encoder_cal.done_rl){
				 /* R / L / Ld-Lq / flux linkage + Kt, using the phase order and
				  * pole pairs measured above (see measure_lr()) */
				 measure_lr(&comm_encoder, &controller, &comm_encoder_cal, controller.loop_count);
			 }
			//  else{
			// 	 /* Exit calibration mode when done */
			// 	 E_ZERO = comm_encoder_cal.ezero;
			// 	 myprintf("E_ZERO: %d  %f\r\n", E_ZERO, TWO_PI_F*fmodf((comm_encoder.ppairs*(float)(-E_ZERO))/((float)ENC_CPR), 1.0f));
			// 	 memcpy(&comm_encoder.offset_lut, comm_encoder_cal.lut_arr, sizeof(comm_encoder.offset_lut));
			// 	 memcpy(&ENCODER_LUT, comm_encoder_cal.lut_arr, sizeof(comm_encoder_cal.lut_arr));
			// 	 //for(int i = 0; i<128; i++){myprintf("%d\r\n", ENCODER_LUT[i]);}
			// 	 if (!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
			// 	 preference_writer_flush(&prefs);
			// 	 preference_writer_close(&prefs);
			// 	 preference_writer_load(prefs);
			// 	 update_fsm(fsmstate, 27);
			//  }
			 /* Once done_cal is set, idle here.  The main loop (run_fsm_slow) saves
			  * the result to flash and exits to the menu - flash writes stall the
			  * CPU for far too long to do them in this interrupt */

			 break;

		 case MOTOR_MODE:
			 /* If CAN has timed out, reset all commands.  CAN_TIMEOUT is in ms;
			  * controller.timeout counts control cycles (25 us each).
			  * 惯量标定期间不适用：它自己就是命令来源，被 zero_commands() 打断
			  * 会让状态机停在半个台阶上。pending 的请求同样要放过 —— 否则
			  * zero_commands() 会在 motion_control() 采纳它之前把标志清掉。 */
			 if((CAN_TIMEOUT > 0 ) && (controller.timeout > CAN_TIMEOUT_CYCLES)
			    && (controller.inertia_state == INERTIA_IDLE)
			    && !controller.inertia_req){
				 zero_commands(&controller);
			 }
			 /* 控制律分发：CTRL_IMPEDANCE 走原来的 torque_control()（MIT/legacy），
			  * CTRL_POSITION/CTRL_VELOCITY 走 motion.c 的串级外环。两者都只产生
			  * i_q_des，电流环（commutate）照旧在 40 kHz 上跑 */
			 motion_control(&controller, &comm_encoder);
			 field_weaken(&controller);
			 commutate(&controller, &comm_encoder);

			 /* Stop counting once past the threshold so the counter can't
			  * overflow during long sessions with the CAN watchdog disabled */
			 if(controller.timeout <= CAN_TIMEOUT_CYCLES){controller.timeout++;}
			 break;

		 case SETUP_MODE:
			 break;

		 case ENCODER_MODE:
			 break;		// encoder data is printed by the main loop

		 case FAULT_MODE:
			 break;		// gates off, wait for the user to clear the fault

		 case INIT_TEMP_MODE:
			 break;
	 }

 }

 void fsm_enter_state(FSMStruct * fsmstate){
	 /* Called when entering a new state
	  * Do necessary setup   */

		switch(fsmstate->state){
				case MENU_MODE:
				fsmstate->print_pending = 1;	// main loop prints the menu
				break;
			case SETUP_MODE:
				/* Start with a clean command buffer, so fragments typed before an
				 * escape-to-menu can't be appended to the first command of the
				 * next setup session */
				fsmstate->bytecount = 0;
				fsmstate->cmd_id = 0;
				fsmstate->cmd_pending = 0;
				memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
				fsmstate->print_pending = 1;	// main loop prints the setup screen
				break;
			case ENCODER_MODE:
				//myprintf("Entering Encoder Mode\r\n");
				break;
			case MOTOR_MODE:

				//myprintf("Entering Motor Mode\r\n");
				GPIO_WriteBits(LED, Bit_SET );
				reset_foc(&controller);
				/* 外环的速度反馈来自编码器 PLL，进电机模式时播种到当前计数，
				 * 否则它会从 0 一路追上来（表现为使能瞬间的一下冲车） */
				ps_reset_pll(&comm_encoder);
				/* 门极打开 ⇒ 命令必须从零开始：endpoint 的 command.* 可以在任何状态
				 * 写入，若不清，空闲时设的 t_ff/kp/kd/p_des/v_des 会在使能的第一拍就
				 * 生效，表现为"一进电机模式就窜车"。CAN 是持续推流、下一帧就会覆盖，
				 * endpoint 是"设一次"，所以这里必须兜住。 */
				zero_commands(&controller);
				// drv_enable_gd(drv);
				/* Enable the six outputs.  AutomaticOutput is disabled in tim.c, so
				 * MOE is software-only and AOE must stay 0 - with AOE set, a break
				 * event would re-enable the gate drive on its own */
				ATIM1->BKDT |= TIM_BKDT_MOEN;
				controller.gate_drive_enabled = 1;
				break;
			case FAULT_MODE:
				/* Make sure the gate drive is off no matter what state we
				 * faulted out of, and report through the main loop */
				ATIM1->BKDT &= ~(TIM_BKDT_AOEN | TIM_BKDT_MOEN);
				controller.gate_drive_enabled = 0;
				reset_foc(&controller);
				zero_commands(&controller);
				GPIO_WriteBits(LED, Bit_RESET );
				fsmstate->print_pending = 1;
				break;
			case CALIBRATION_MODE:
				//myprintf("Entering Calibration Mode\r\n");
				/* 重新标定期间“已标定”不再成立：中途失败（comm_encoder_cal.failed）
				 * 会回菜单，此时必须保持 false，否则失败后还能进电机模式 */
				PRE_CALIBRATED = 0;
				/* zero out all calibrations before starting */

				comm_encoder_cal.done_cal = 0;
				comm_encoder_cal.done_ordering = 0;
				comm_encoder_cal.done_rl = 0;
				comm_encoder_cal.started = 0;
				comm_encoder_cal.failed = 0;
				comm_encoder_cal.sample_count = 0;	// so the main loop doesn't stream samples left over from a previous calibration
				comm_encoder_cal.msg_head = 0;		// drop milestone messages left unprinted by a previous run
				comm_encoder_cal.msg_tail = 0;
				memset(&comm_encoder_cal.rl, 0, sizeof(comm_encoder_cal.rl));	// restart the R/L/Ld-Lq/flux chain
				comm_encoder.e_zero = 0;
				memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));
				// drv_enable_gd(drv);
				ATIM1->BKDT |= TIM_BKDT_MOEN;		// see the note in enter MOTOR_MODE
				controller.gate_drive_enabled = 1;
				break;

		}
 }

 void fsm_exit_state(FSMStruct * fsmstate){
	 /* Called when exiting the current state
	  * Do necessary cleanup  */

		switch(fsmstate->state){
			case MENU_MODE:
				//myprintf("Leaving Main Menu\r\n");
				fsmstate->ready = 1;
				break;
			case SETUP_MODE:
				//myprintf("Leaving Setup Menu\r\n");
				fsmstate->ready = 1;
				break;
			case ENCODER_MODE:
				//myprintf("Leaving Encoder Mode\r\n");
				fsmstate->ready = 1;
				break;
			case MOTOR_MODE:
				/* Don't stop commutating if there are high currents or FW happening */
				//if( (fabs(controller.i_q_filt)<1.0f) && (fabs(controller.i_d_filt)<1.0f) ){
					fsmstate->ready = 1;
					ATIM1->BKDT &= ~TIM_BKDT_MOEN;
					controller.gate_drive_enabled = 0;
					reset_foc(&controller);
					//myprintf("Leaving Motor Mode\r\n");
					GPIO_WriteBits(LED, Bit_RESET );
				//}
				zero_commands(&controller);		// Set commands to zero
				break;
			case CALIBRATION_MODE:
				//myprintf("Exiting Calibration Mode\r\n");
				ATIM1->BKDT &= ~TIM_BKDT_MOEN;
				// drv_disable_gd(drv);
				//free(error_array);
				//free(lut_array);
				controller.gate_drive_enabled = 0;
				fsmstate->ready = 1;
				break;
			case FAULT_MODE:
				controller.fault = 0;	// leaving the fault state is what clears the fault
				fsmstate->ready = 1;
				break;
		}

 }

 void update_fsm(FSMStruct * fsmstate, char fsm_input){
	 /*update_fsm is only run when new state-change information is received
	  * on serial terminal input or CAN input
	  */
	if(fsm_input == MENU_CMD){	// escape to exit to rest mode
		fsmstate->next_state = MENU_MODE;
		fsmstate->ready = 0;
		return;
	}
	switch(fsmstate->state){
		case MENU_MODE:
			switch (fsm_input){
				case DFU_CMD:
					/* Enter the ROM bootloader.
					 * Deliberately only reachable from the menu: 'd' is also the
					 * setup prefix for KD_MAX, so a global 'd' handler made that
					 * setting impossible to enter and turned any stray 'd' on the
					 * serial line into a reboot */
					__asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
					_reboot_cookie = 0xDEADBEEF;
					NVIC_SystemReset();
					break;
				case CAL_CMD:
					fsmstate->next_state = CALIBRATION_MODE;
					fsmstate->ready = 0;
					break;
				case MOTOR_CMD:
					fsmstate->next_state = MOTOR_MODE;
					fsmstate->ready = 0;
					break;
				case ENCODER_CMD:
					fsmstate->next_state = ENCODER_MODE;
					fsmstate->ready = 0;
					break;
				case SETUP_CMD:
					fsmstate->next_state = SETUP_MODE;
					fsmstate->ready = 0;
					break;
				case ZERO_CMD:
					// comm_encoder.m_zero = 0;
					// ps_sample(&comm_encoder, DT);
					// int zero_count = comm_encoder.count;
					// M_ZERO = zero_count;
					// if (!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
					// preference_writer_flush(&prefs);
					// preference_writer_close(&prefs);
					// preference_writer_load(prefs);
					// myprintf("\n\r  Saved new zero position:  %d\n\r\n\r", M_ZERO);
					fsmstate->zero_pending = 1;		// main loop samples the position and saves it to flash
					break;
				}
			break;
		case SETUP_MODE:
			if(fsmstate->cmd_pending){break;}	// drop input until the main loop has processed the pending command
			if(fsm_input == ENTER_CMD){
				fsmstate->cmd_pending = 1;		// main loop runs process_user_input
				break;
			}
			else if (fsm_input == 0x0A) { // 上述把0x0D定义成了ENTER_CMD，所以这里0x0A就不需要了，直接忽略
				break;
			}
			if(fsmstate->bytecount == 0){
				fsmstate->cmd_id = fsm_input;
				fsmstate->bytecount++;
			}
			else if(fsmstate->bytecount < sizeof(fsmstate->cmd_buff)){	// drop excess characters, and keep the last byte null so atoi/atof always see a terminated string
				fsmstate->cmd_buff[fsmstate->bytecount-1] = fsm_input;
				fsmstate->bytecount++;
			}
			/* If enter is typed, process user input */

			break;

		case ENCODER_MODE:
			break;
		case MOTOR_MODE:
			break;
		case FAULT_MODE:
			break;		// only the escape-to-menu above is accepted while faulted
	}
 }


 void enter_menu_state(void){
	    //drv.disable_gd();
	    //reset_foc(&controller);
	    //gpio.enable->write(0);
	    myprintf("\n\r\n\r");
	    myprintf(" Commands:\n\r");
	    myprintf(" m - Motor Mode\n\r");
	    myprintf(" c - Calibrate Encoder\n\r");
	    myprintf(" s - Setup\n\r");
	    myprintf(" e - Display Encoder\n\r");
	    myprintf(" z - Set Zero Position\n\r");
	    myprintf(" d - Enter DFU/Bootloader\n\r");
	    myprintf(" esc - Exit to Menu\n\r");

	    //gpio.led->write(0);
 }

 void enter_setup_state(void){
	    myprintf("\r\n Configuration Options \n\r");
	    myprintf(" %-4s %-31s %-5s %-6s %-2s\r\n", "prefix", "parameter", "min", "max", "current value");
	    myprintf("\r\n Motor:\r\n");
	    myprintf(" %-4s %-31s %-5s %-6s %.3f\n\r", "g", "Gear Ratio", "0", "-", GR);
	    myprintf(" %-4s %-31s %-5s %-6s %.5f\n\r", "k", "Torque Constant (N-m/A)", "0", "-", KT);
	    myprintf("\r\n Control:\r\n");
	    myprintf(" %-4s %-31s %-5s %-6s %.3f\n\r", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
	    myprintf(" %-4s %-31s %-5s %-6.1f %.3f\n\r", "l", "Current Limit (A)", "0.0", I_MAX_LIMIT, I_MAX);
	    myprintf(" %-4s %-31s %-5s %-6s %.3f\n\r", "p", "Max Position Setpoint (rad)", "-", "-", P_MAX);
	    myprintf(" %-4s %-31s %-5s %-6s %.3f\n\r", "v", "Max Velocity Setpoint (rad)/s", "-", "-", V_MAX);
	    myprintf(" %-4s %-31s %-5s %-6s %.3f\n\r", "x", "Max Position Gain (N-m/rad)", "0.0", "1000.0", KP_MAX);
	    myprintf(" %-4s %-31s %-5s %-6s %.3f\n\r", "d", "Max Velocity Gain (N-m/rad/s)", "0.0", "5.0", KD_MAX);
	    myprintf(" %-4s %-31s %-5s %-6s %.3f\n\r", "f", "FW Current Limit (A)", "0.0", "33.0", I_FW_MAX);
	    //myprintf(" %-4s %-31s %-5s %-6s %.1f\n\r", "h", "Temp Cutoff (C) (0 = none)", "0", "150", TEMP_MAX);
	    myprintf(" %-4s %-31s %-5s %-6s %.3f\n\r", "c", "Continuous Current (A)", "0.0", "40.0", I_MAX_CONT);
	    myprintf(" %-4s %-31s %-5s %-6s %.3f\n\r", "a", "Calibration Current (A)", "0.0", "20.0", I_CAL);
	    myprintf("\r\n CAN:\r\n");
	    myprintf(" %-4s %-31s %-5s %-6s %-5i\n\r", "i", "CAN ID", "0", "127", CAN_ID);
	    myprintf(" %-4s %-31s %-5s %-6s %-5i\n\r", "m", "CAN TX ID", "0", "127", CAN_MASTER);
	    myprintf(" %-4s %-31s %-5s %-6s %d\n\r", "t", "CAN Timeout (ms)(0 = none)", "0", "100000", CAN_TIMEOUT);
	    myprintf(" \n\r To change a value, type 'prefix''value''ENTER'\n\r e.g. 'b1000''ENTER'\r\n ");
	    myprintf("VALUES NOT ACTIVE UNTIL POWER CYCLE! \n\r\n\r");
 }

 void request_fsm_state(FSMStruct * fsmstate, uint8_t next){
	 /* Main-loop callers (endpoint action.*) must not race run_fsm()'s read of
	  * next_state/ready: apply both under one critical section. */
	 uint32_t primask = __get_PRIMASK();
	 __disable_irq();
	 fsmstate->next_state = next;
	 fsmstate->ready = 0;
	 if(!primask){__enable_irq();}
 }

 void process_user_input(FSMStruct * fsmstate){
	 /* Collects user input from serial (maybe eventually CAN) and updates settings */

	 switch (fsmstate->cmd_id){
		 case 'b':
			 I_BW = fmaxf(fminf(atof(fsmstate->cmd_buff), 2000.0f), 100.0f);
			 myprintf("I_BW set to %f\r\n", I_BW);
			 break;
		 case 'i':
			 CAN_ID = atoi(fsmstate->cmd_buff);
			 if(CAN_ID < 0){CAN_ID = 0;}
			 if(CAN_ID > 127){CAN_ID = 127;}	// only 7 bits of node ID - the upper 4 ID bits are the opcode
			 myprintf("CAN_ID set to %d\r\n", CAN_ID);
			 break;
		 case 'm':
			 CAN_MASTER = atoi(fsmstate->cmd_buff);
			 if(CAN_MASTER < 0){CAN_MASTER = 0;}
			 if(CAN_MASTER > 127){CAN_MASTER = 127;}
			 myprintf("CAN_TX_ID set to %d\r\n", CAN_MASTER);
			 break;
		 case 'l':
			 /* Keep the OC trip inside the corrected 5 mOhm sense range. */
			 I_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), I_MAX_LIMIT), 0.0f);
			 myprintf("I_MAX set to %f\r\n", I_MAX);
			 break;
		 case 'f':
			 I_FW_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 33.0f), 0.0f);
			 myprintf("I_FW_MAX set to %f\r\n", I_FW_MAX);
			 break;
		 case 't':
			 CAN_TIMEOUT = atoi(fsmstate->cmd_buff);
			 myprintf("CAN_TIMEOUT set to %d\r\n", CAN_TIMEOUT);
			 break;
		 case 'h':
			 TEMP_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 150.0f), 0.0f);
			 myprintf("TEMP_MAX set to %f\r\n", TEMP_MAX);
			 break;
		 case 'c':
			 I_MAX_CONT = fmaxf(fminf(atof(fsmstate->cmd_buff), 40.0f), 0.0f);
			 myprintf("I_MAX_CONT set to %f\r\n", I_MAX_CONT);
			 break;
		 case 'a':
			 I_CAL = fmaxf(fminf(atof(fsmstate->cmd_buff), 20.0f), 0.0f);
			 myprintf("I_CAL set to %f\r\n", I_CAL);
			 break;
		 case 'g':
			 GR = fmaxf(atof(fsmstate->cmd_buff), .001f);	// Limit prevents divide by zero if user tries to enter zero
			 myprintf("GR set to %f\r\n", GR);
			 break;
		 case 'k':
			 KT = fmaxf(atof(fsmstate->cmd_buff), 0.0001f);	// Limit prevents divide by zero.  Seems like a reasonable LB?
			 myprintf("KT set to %f\r\n", KT);
			 break;
		 case 'x':
			 KP_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 myprintf("KP_MAX set to %f\r\n", KP_MAX);
			 break;
		 case 'd':
			 KD_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 myprintf("KD_MAX set to %f\r\n", KD_MAX);
			 break;
		 case 'p':
			 P_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 P_MIN = -P_MAX;
			 myprintf("P_MAX set to %f\r\n", P_MAX);
			 break;
		 case 'v':
			 V_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 V_MIN = -V_MAX;
			 myprintf("V_MAX set to %f\r\n", V_MAX);
			 break;
		 default:
			 myprintf("\n\r '%c' Not a valid command prefix\n\r\n\r", fsmstate->cmd_id);
			 break;

		 }

	 /* Write new settings to flash */
	 fsmstate->save_status = save_preferences() ? 1 : 3;

	 /* cmd_id / cmd_buff belong to the caller's snapshot, so the ISR-side buffer
	  * is cleared by run_fsm_slow() (before this runs) rather than here */
	 enter_setup_state();
 }

 static int save_preferences(void){
	 /* Write settings to flash.  preference_writer_save() masks interrupts for
	  * the whole erase + program sequence (the N32 flash dead-locks the CPU if an
	  * interrupt lands between the two stores of a 64-bit program) and kicks the
	  * watchdog itself while that lasts, so nothing here may print or block before
	  * it returns.  Only call this from the main loop, in states where the gate
	  * drive is off.  Returns 1 on success so the caller can report it through
	  * system.save_status. */
	 int ok;
	 IWDG_ReloadKey();
	 ok = preference_writer_save(&prefs);
	 if(!ok){
		 myprintf("\n\r!!! Preference save failed - config unchanged in flash !!!\r\n");
	 }
	 IWDG_ReloadKey();
	 return ok;
 }

 void run_fsm_slow(FSMStruct * fsmstate){
	 /* All the slow, non-realtime state machine work: printing and flash writes.
	  * Called from the main loop every millisecond or so - never from an interrupt.
	  * The fast ISR (run_fsm) hands work over here through the *_pending flags. */

	 static int cal_samples_printed = 0;
	 static uint8_t gear_probed = 0;		// 本次进入标定后是否已经探过副编码器
	 static int encoder_print_count = 0;
	 static uint8_t reboot_armed = 0;		// 1 = a reset is queued
	 static uint32_t reboot_deadline = 0;	// loop_count threshold (valid when reboot_armed)
	 static uint8_t reboot_dfu = 0;		// 1 = reset into the ROM DFU bootloader (cookie), 0 = plain reset
	 static uint8_t inertia_phase = 0;		// 0 idle / 1 等门极 / 2 等结果 / 3 等回菜单 / 4 两轮之间停稳
	 static uint8_t inertia_started = 0;	// ISR 是否已经把请求转成状态机
	 static uint8_t inertia_tries = 0;		// 已完成的轮数
	 static uint8_t inertia_ok = 0;			// 成功的轮数
	 static int8_t inertia_fail = 0;		// 最近一次的失败码
	 static float inertia_samples[INERTIA_RUNS];
	 static uint32_t inertia_deadline = 0;

	 /* Print any calibration milestone messages queued by the ISR (calibration
	  * runs in the control interrupt and must not print there) */
	 cal_print_flush(&comm_encoder_cal);

	 /* NTC 温度换算 + 滤波。放在主循环而不是 40 kHz 中断里：多项式要几十条
	  * 浮点指令，而温度的时间常数是 0.1 s 量级，1 ms 更新一次绰绰有余。
	  * dt 由 loop_count 的差算出，与主循环节奏无关（见 update_thermistors） */
	 update_thermistors(&controller);

	 /* 未标定被拒绝进入电机模式：打印一次，让串口/菜单用户知道原因 */
	 if(fsmstate->motor_refused){
		 fsmstate->motor_refused = 0;
		 myprintf("\r\n!!! Motor mode refused: config.pre_calibrated is false - run calibration (c / action.calibrate) first !!!\r\n");
	 }

	 /* 惯量标定的编排：用户只发一条 action.measure_inertia，这里负责
	  * 进电机模式 → 触发 ISR 里的状态机 → 等它跑完 → 回菜单。
	  * 为什么必须回菜单：测完如果停在 MOTOR_MODE，门极还是开的、转矩是 0，
	  * 有重力负载的关节会直接垮下来。回菜单还能让紧接着的 action.save 生效
	  * （save 要求门极关闭）。
	  *
	  * 连跑 INERTIA_RUNS 轮取中位数：单次读数受齿隙/摩擦和突破电流档位影响，
	  * 台架实测离散 ±50%（0.00053~0.00167），只有中位数稳定。轮与轮之间停
	  * 0.375 s —— ISR 的 SETTLE 只有 150 ms，刚刹完车还在轻微漂移时会被判成
	  * "本来就在动"（status 3）。净行程因此约为单轮的 3 倍（~0.5 rad） */
	 if(fsmstate->inertia_pending){
		 fsmstate->inertia_pending = 0;
		 controller.inertia_status = 1;
		 inertia_phase = 1;
		 inertia_started = 0;
		 inertia_tries = 0;
		 inertia_ok = 0;
		 inertia_fail = 0;
		 controller.inertia_run_code = 0;
		 inertia_deadline = controller.loop_count + REBOOT_DELAY_CYCLES / 4;	/* 0.125 s 等到门极 */
		 request_fsm_state(fsmstate, MOTOR_MODE);
	 }
	 switch(inertia_phase){
		 case 1:
			 if((fsmstate->state == MOTOR_MODE) && controller.gate_drive_enabled){
				 controller.inertia_req = 1;
				 inertia_phase = 2;
				 inertia_deadline = controller.loop_count + REBOOT_DELAY_CYCLES / 2;	/* 0.25 s 内要启动 */
			 }
			 else if(controller.loop_count > inertia_deadline){
				 controller.inertia_status = 7;		/* 进不了电机模式 */
				 inertia_phase = 0;
			 }
			 break;
		 case 2: {
			 /* 必须先看到 ISR 把请求转成 INERTIA_SETTLE，才能把"回到 IDLE"当成结束。
			  * 否则本循环比 ISR 快一拍，会在请求还没被采纳时就宣布测完并回菜单，
			  * 请求随后被 zero_commands() 清掉 —— 状态永远停在"运行中" */
			 uint8_t done = 0;
			 if(controller.inertia_state != INERTIA_IDLE){
				 if(!inertia_started){
					 inertia_started = 1;
					 inertia_deadline = controller.loop_count + 4u * REBOOT_DELAY_CYCLES;
				 }
				 else if((controller.loop_count > inertia_deadline)
				         || (fsmstate->state != MOTOR_MODE)){
					 controller.inertia_run_code = 5;	/* 状态机没跑完（卡住 / 中途退出电机模式） */
					 done = 1;
				 }
			 }
			 else if(inertia_started){
				 done = 1;
			 }
			 else if(controller.loop_count > inertia_deadline){
				 controller.inertia_run_code = 5;		/* 请求没被采纳（没进状态机） */
				 done = 1;
			 }
			 if(!done){break;}

			 /* 一轮结束：收结果。失败的一轮不写样本，只记失败码。
			  * 注意看的是 inertia_run_code —— inertia_status 只由这里发布，
			  * 中间轮不能让它变成 2，否则上位机轮询会以为整个动作测完了 */
			 inertia_started = 0;
			 if(controller.inertia_run_code == 2){
				 if(inertia_ok < INERTIA_RUNS){
					 inertia_samples[inertia_ok] = controller.inertia_meas;
				 }
				 inertia_ok++;
			 }
			 else{
				 inertia_fail = controller.inertia_run_code;
			 }
			 inertia_tries++;

			 if((inertia_tries < INERTIA_RUNS) && (fsmstate->state == MOTOR_MODE)){
				 /* 下一轮：先停稳，再由 case 4 重新发请求 */
				 inertia_phase = 4;
				 inertia_deadline = controller.loop_count + (REBOOT_DELAY_CYCLES * 3u) / 4u;	/* 0.375 s */
				 break;
			 }

			 /* 收尾：至少 2 轮成功才认为可信（单轮离散太大） */
			 if(inertia_ok >= 2u){
				 INERTIA = inertia_median(inertia_samples, inertia_ok);	/* RAM only，存盘仍由 action.save 决定 */
				 controller.inertia_status = 2;
				 myprintf("\r\n  Inertia: J = %.6f kg.m^2 (median of %u/%u runs)\r\n",
						  (double)INERTIA, inertia_ok, INERTIA_RUNS);
			 }
			 else{
				 controller.inertia_status = (inertia_fail != 0) ? inertia_fail : 5;
				 myprintf("\r\n  Inertia: failed (status %d, %u/%u runs ok) - keeping J = %.6f\r\n",
						  (int)controller.inertia_status, inertia_ok, INERTIA_RUNS, (double)INERTIA);
			 }
			 request_fsm_state(fsmstate, MENU_MODE);
			 inertia_phase = 3;
			 inertia_deadline = controller.loop_count + REBOOT_DELAY_CYCLES / 4;
			 break;
		 }
		 case 4:
			 /* 两轮之间：等关节停稳再发下一次请求 */
			 if(controller.loop_count > inertia_deadline){
				 controller.inertia_state = INERTIA_IDLE;	/* 上一轮若卡在非 IDLE，这里兜底复位 */
				 controller.inertia_status = 1;				/* 对外仍然报"运行中" */
				 controller.inertia_req = 1;
				 inertia_started = 0;
				 inertia_phase = 2;
				 inertia_deadline = controller.loop_count + REBOOT_DELAY_CYCLES / 2;
			 }
			 break;
		 case 3:
			 if(fsmstate->state == MENU_MODE || controller.loop_count > inertia_deadline){
				 inertia_phase = 0;
				 inertia_started = 0;
			 }
			 break;
		 default:
			 inertia_phase = 0;
			 break;
	 }

	 /* Print the menu/setup screen after a state change */
	 if(fsmstate->print_pending){
		 fsmstate->print_pending = 0;
		 if(fsmstate->state == MENU_MODE){enter_menu_state();}
		 else if(fsmstate->state == SETUP_MODE){enter_setup_state();}
		 else if(fsmstate->state == FAULT_MODE){
			 myprintf("\n\r!!! FAULT - gate drive disabled !!!\r\n");
			 if(controller.fault & FAULT_OV){myprintf("  Overvoltage: bus hit %.2f V (limit %.1f V)\r\n", controller.fault_vbus, VBUS_OV_TRIP);}
			 if(controller.fault & FAULT_UV){myprintf("  Undervoltage: bus hit %.2f V (limit %.1f V)\r\n", controller.fault_vbus, VBUS_UV_TRIP);}
			 if(controller.fault & FAULT_OC){myprintf("  Overcurrent: phase current hit %.1f A (limit %.1f A)\r\n", controller.fault_iabs, controller.i_oc_limit);}
			 if(controller.fault & FAULT_FET_OT){myprintf("  FET over-temperature: %.1f C (limit %.1f C)\r\n", controller.mos_temp, FET_TEMP_HI);}
			 if(controller.fault & FAULT_MOT_OT){myprintf("  Motor over-temperature: %.1f C (limit %.1f C)\r\n", controller.mot_temp, MOT_TEMP_HI);}
			 if(controller.fault & FAULT_POS){myprintf("  Position following error: %.3f rad (limit %.3f rad)\r\n", controller.pos_err, POS_ERROR_FAULT);}
			 if(controller.fault & FAULT_ESTOP){myprintf("  E-stop requested over CAN\r\n");}
			 myprintf("  esc (or the CAN menu command) to clear\r\n");
		 }
	 }

	 /* Process a completed setup command.
	  * Take the command out of the ISR-shared buffer - under a very short critical
	  * section - and hand process_user_input() a private copy.  That releases the
	  * interrupt to receive immediately, instead of dropping everything typed
	  * while process_user_input() blocks for tens of ms in save_preferences() */
	 if(fsmstate->cmd_pending){
		 FSMStruct cmd_snapshot;			// only cmd_id / cmd_buff are used
		 __disable_irq();
		 cmd_snapshot.cmd_id = fsmstate->cmd_id;
		 memcpy(cmd_snapshot.cmd_buff, fsmstate->cmd_buff, sizeof(cmd_snapshot.cmd_buff));
		 fsmstate->bytecount = 0;
		 fsmstate->cmd_id = 0;
		 memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
		 fsmstate->cmd_pending = 0;
		 __enable_irq();
		 if(fsmstate->state == SETUP_MODE){process_user_input(&cmd_snapshot);}
	 }

	 /* endpoint action.save：只在门极关闭时写 flash。ISR/中断里只置标志，
	  * 真正的擦写在主循环里做 */
	 if(fsmstate->save_pending){
		 fsmstate->save_pending = 0;
		 if(!controller.gate_drive_enabled){
			 fsmstate->save_status = save_preferences() ? 1 : 3;
			 if(fsmstate->save_status == 1){myprintf("\r\n  Preferences saved\r\n");}
		 }
		 else{
			 fsmstate->save_status = 2;
			 myprintf("\r\n  Save skipped: gate drive is enabled\r\n");
		 }
	 }

	 /* Zero the mechanical position and save it */
	 if(fsmstate->zero_pending){
		 fsmstate->zero_pending = 0;
		 if(fsmstate->state == MENU_MODE){	// only with the gate drive off
			 /* 走 ps_set_m_zero：机械零点同时是双编参考的坐标系，改零点要把参考
			  * 平移同样的量，否则断电再上电多圈位置会整体偏掉 */
			 ps_set_m_zero(comm_encoder.count);
			 fsmstate->save_status = save_preferences() ? 1 : 3;
			 myprintf("\n\r  Saved new zero position:  %d\n\r\n\r", M_ZERO);
		 }
	 }

	 /* endpoint action.gear_capture / action.gear_read：齿轮式双编的副编码器与主
	  * 编码器共用 SPI1，只能在主循环里读，而且两路都要门极关闭、转子静止才有意义。
	  * capture 只更新 RAM 里的参考（写 flash 仍由 action.save 负责） */
	 if(fsmstate->gear_capture_pending || fsmstate->gear_read_pending){
		 uint8_t do_capture = fsmstate->gear_capture_pending;
		 fsmstate->gear_capture_pending = 0;
		 fsmstate->gear_read_pending = 0;
		 if(controller.gate_drive_enabled){
			 fsmstate->gear_status = -4;
			 myprintf("\r\n  Gear encoder: refused - gate drive is enabled\r\n");
		 }
		 else{
			 int rc = do_capture ? ps_dual_gear_capture(&comm_encoder)
			                     : ps_dual_gear_read(&comm_encoder);
			 fsmstate->gear_status = (rc == 0) ? 1 : (int8_t)rc;
			 if(rc == 0){
				 myprintf("\r\n  Gear encoder: sec_raw %d  phase %.6f  abs_turns %.3f%s\r\n",
						  sec_encoder.raw, (double)sec_encoder.phase, (double)sec_encoder.abs_turns,
						  do_capture ? "  (reference captured - run action.save to keep it)" : "");
			 }
			 else if(rc == -2){
				 myprintf("\r\n  Gear encoder: no reference yet - run action.gear_capture first\r\n");
			 }
			 else if(rc == -1){
				 myprintf("\r\n  Gear encoder: gear teeth / direction config is invalid\r\n");
			 }
			 else{
				 myprintf("\r\n  Gear encoder: no reply from the second encoder (%d)\r\n", rc);
			 }
		 }
	 }

	 /* endpoint action.reset_config：恢复出厂默认值并写 flash。
	  * 会连标定（PPAIRS / E_ZERO / M_ZERO / ENCODER_LUT）一起清掉，所以同样只在主循环、
	  * 门极关闭时做。运行中的 comm_encoder 副本故意不动：重启前继续用旧标定，
	  * 比拿全零 LUT 去换相安全。
	  * 保存成功后自动重启：CAN_ID / CAN_PROT / CAN_BAUD 是运行期读取的，改了却不重启
	  * 会让主站立刻失联；而且 config.* 与 comm_encoder.* 会处于混合状态。 */
	 if(fsmstate->reset_config_pending){
		 fsmstate->reset_config_pending = 0;
		 if(!controller.gate_drive_enabled){
			 config_apply_defaults();
			 config_apply_limits();
			 fsmstate->save_status = save_preferences() ? 1 : 3;
			 myprintf("\r\n  Config reset to defaults (calibration cleared)\r\n");
			 if(fsmstate->save_status == 1){
				 myprintf("  Rebooting; recalibrate afterwards\r\n");
				 fsmstate->reboot_pending = 1;	// 下面的 reboot 块处理（关栅 + 0.5 s 后复位）
			 }
			 else{
				 myprintf("  ...but writing flash failed, defaults are RAM-only\r\n");
			 }
		 }
		 else{
			 fsmstate->save_status = 2;
			 myprintf("\r\n  Reset skipped: gate drive is enabled\r\n");
		 }
	 }

	 /* endpoint action.reboot / action.enter_dfu_mode：应答已经发完（调用者已返回）
	  * 才复位。用控制 ISR 的 loop_count 计时，不在主循环里阻塞。
	  * 先回菜单：由控制 ISR 走 fsm_enter_state() 把门极关掉，0.5 s 后才真正复位，
	  * 避免带着 PWM 直接复位（引脚复位成输入，功率级状态不确定）。
	  *
	  * reboot_dfu = 0（action.reboot）：普通复位，**不动** _reboot_cookie ——
	  *   正常启动之后它本来就是 42，而 main.c 的 "启动状态不确定就再复位一次"
	  *   （!= 42）是故意留的保险，不该由这里关掉。
	  * reboot_dfu = 1（action.enter_dfu_mode）：与菜单里的 'd'（DFU_CMD）同义，
	  *   cookie 置 0xDEADBEEF，early_start_checks() 会先延时 ~2 s（等功率级放电）
	  *   再 Jump_To_BOOT()。 */
	 if(fsmstate->reboot_pending){
		 fsmstate->reboot_pending = 0;
		 request_fsm_state(fsmstate, MENU_MODE);
		 reboot_dfu = 0;
		 reboot_deadline = (uint32_t)controller.loop_count + REBOOT_DELAY_CYCLES;
		 reboot_armed = 1;
	 }
	 if(fsmstate->dfu_pending){
		 fsmstate->dfu_pending = 0;
		 request_fsm_state(fsmstate, MENU_MODE);
		 reboot_dfu = 1;
		 reboot_deadline = (uint32_t)controller.loop_count + REBOOT_DELAY_CYCLES;
		 reboot_armed = 1;
		 myprintf("\r\n  Entering ROM DFU bootloader in 0.5 s...\r\n");
	 }
	 if(reboot_armed && ((int32_t)((uint32_t)controller.loop_count - reboot_deadline) >= 0)){
		 reboot_armed = 0;
		 if(reboot_dfu){_reboot_cookie = 0xDEADBEEF;}
		 __disable_irq();
		 NVIC_SystemReset();
	 }

	 if(fsmstate->state == CALIBRATION_MODE){
		 /* 齿轮式双编：进标定时先探一次副编码器，让用户在校准跑起来之前就知道
		  * "断电记忆输出轴位置"这次会不会生效，而不是等 30 s 后才看到失败。
		  * 副编码器与主编码器共用 SPI1，探测会挡住控制中断（在线约 4 us，没插到
		  * 时两帧各等到上限约 2×16 us），发生在标定最开头的开环慢速段，最多推迟
		  * 一个控制周期，对开环电压控制没有影响。
		  * 真在的话标定收尾会顺手 capture 一次参考（见下面 done_* 块） */
		 if(!gear_probed){
			 gear_probed = 1;
			 int gear_present = ps_sec_probe();
			 if(!gear_present){
				 myprintf("\r\nGEAR: no second encoder - output position memory stays off\r\n");
			 }
			 else if(!ps_dual_gear_config_ok()){
				 myprintf("\r\nGEAR: second encoder found, but gear config is invalid (T1=%d T2=%d) - memory off\r\n",
						  (int)GEAR_TEETH_FIRST, (int)GEAR_TEETH_SEC);
			 }
			 else{
				 myprintf("\r\nGEAR: second encoder found (raw %d, teeth %d/%d) - reference will be captured at the end\r\n",
						  sec_encoder.raw, (int)GEAR_TEETH_FIRST, (int)GEAR_TEETH_SEC);
			 }
		 }

		 /* Stream out new calibration samples as they're taken (forward pass only -
		  * sample_count counts back down during the reverse pass) */
		 while(cal_samples_printed < comm_encoder_cal.sample_count){
			 myprintf("%d %d\r\n", cal_samples_printed, comm_encoder_cal.error_arr[cal_samples_printed]);
			 cal_samples_printed++;
		 }

		 /* When calibration finishes, exit to the menu first (the state transition
		  * in the ISR disables the gate drive), then save the results to flash */
		 if(comm_encoder_cal.done_ordering && comm_encoder_cal.done_cal && comm_encoder_cal.done_rl){
			 update_fsm(fsmstate, MENU_CMD);
			 /* Wait for the commutation ISR to run the transition out of
			  * calibration mode (a couple of 25 us cycles) before saving the
			  * results; bounded so a stuck transition can't hang the main loop */
			 volatile uint8_t *pstate = &fsmstate->state;
			 for(int i = 0; (i < 100000) && (*pstate == CALIBRATION_MODE); i++){__NOP();}
			 E_ZERO = comm_encoder_cal.ezero;
			 /* Apply the result to the running encoder now.  ps_sample() reads its
			  * own ppairs/e_zero copy for the electrical angle, so without this the
			  * freshly measured pole pairs only took effect after a reboot */
			 comm_encoder.ppairs = PPAIRS;
			 comm_encoder.e_zero = E_ZERO;
			 comm_encoder.m_zero = M_ZERO;
			 memcpy(&comm_encoder.offset_lut, comm_encoder_cal.lut_arr, sizeof(comm_encoder.offset_lut));
			 memcpy(&ENCODER_LUT, comm_encoder_cal.lut_arr, sizeof(comm_encoder_cal.lut_arr));
			 /* 刚标定完 R/L：立刻按新值重算电流环增益（否则要等下一次进电机模式
			  * 才用得上）。顺手打印一行，台架上可以直接核对推导结果 */
			 current_loop_retune(&controller);
			 myprintf("Current loop: i_bw %.0f Hz, R %.4f ohm, Ld %.1f uH, Lq %.1f uH -> k_d %.4f, k_q %.4f, ki_d %.4f, ki_q %.4f\r\n",
					  (double)I_BW, (double)R_PHASE, (double)(L_D*1.0e6f), (double)(L_Q*1.0e6f),
					  (double)controller.k_d, (double)controller.k_q,
					  (double)controller.ki_d, (double)controller.ki_q);
			 /* 齿轮式双编：此刻转子刚标定完、门极已关、静止不动，正好把"当前位置
			  * = 输出轴绝对位置参考"记下来（phase + 多圈计数），一起存盘。
			  * 不转电机、不需要额外操作。失败时**保留旧参考**：参考是 (相位, 计数)
			  * 成对的，只要主编码器多圈计数没断，旧参考继续有效，只是没跟着更新 */
			 {
				 int gear_rc = ps_dual_gear_capture(&comm_encoder);
				 if(gear_rc == 0){
					 myprintf("GEAR: reference captured (phase %.6f, count %d, teeth %d/%d)\r\n",
							  (double)GEAR_ZERO_PHASE, (int)GEAR_ZERO_COUNT,
							  (int)GEAR_TEETH_FIRST, (int)GEAR_TEETH_SEC);
				 }
				 else{
					 myprintf("GEAR: capture failed (%d) - %s\r\n", gear_rc,
							  GEAR_REF_VALID ? "keeping the previous reference" : "output position memory off");
				 }
			 }
			 /* 全部三段（相序 / 编码器零点 / R-L-Ld-Lq-磁链）都跑完才会到这里，
			  * 所以这就是“已标定”。先置标志再存盘，一条记录里一起落盘 */
			 PRE_CALIBRATED = 1;
			 fsmstate->save_status = save_preferences() ? 1 : 3;
			 myprintf("E_ZERO: %d  %f\r\n", E_ZERO, TWO_PI_F*fmodf((comm_encoder.ppairs*(float)(-E_ZERO))/((float)ENC_CPR), 1.0f));
			 for(int i = 0; i<N_LUT; i++){myprintf("%d %d\r\n", i, comm_encoder_cal.lut_arr[i]);}
			 comm_encoder_cal.done_cal = 0;		// consume, so this only runs once
			 comm_encoder_cal.done_ordering = 0;
			 comm_encoder_cal.done_rl = 0;
		 }
	 }
	 else{
		 cal_samples_printed = 0;
		 gear_probed = 0;			// 下次进标定重新探测副编码器
	 }

	 /* Print encoder data a few times a second in encoder mode */
	 if(fsmstate->state == ENCODER_MODE){
		 if(++encoder_print_count >= 200){
			 encoder_print_count = 0;
			 ps_print(&comm_encoder, 0);
		 }
	 }

 }

 void enter_motor_mode(void){

 }

