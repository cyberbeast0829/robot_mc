/*
 * fsm.h
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */


#ifndef INC_FSM_H_
#define INC_FSM_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>



#define MENU_MODE           0
#define CALIBRATION_MODE    1
#define MOTOR_MODE          2
#define SETUP_MODE          4
#define ENCODER_MODE        5
#define INIT_TEMP_MODE      6
#define FAULT_MODE          7

#define MENU_CMD			27
#define MOTOR_CMD			'm'
#define CAL_CMD				'c'
#define ENCODER_CMD		'e'
#define SETUP_CMD			's'
#define ZERO_CMD			'z'
#define DFU_CMD       'd'
#define ENTER_CMD			13



typedef struct{
	uint8_t state;
	uint8_t next_state;
	uint8_t ready;
	char cmd_buff[8];
	uint8_t bytecount;
	char cmd_id;
	/* Requests handed from interrupt context to the main loop, which does all
	 * the slow work (printing, flash writes) that must not block commutation */
	volatile uint8_t print_pending;			// print the menu/setup screen for the current state
	volatile uint8_t cmd_pending;			// a setup command is waiting in cmd_buff
	volatile uint8_t zero_pending;			// zero the mechanical position and save it
	volatile uint8_t save_pending;			// write the current config to flash (endpoint action.save)
	volatile uint8_t reset_config_pending;		// restore factory defaults + save (endpoint action.reset_config)
	volatile uint8_t reboot_pending;		// reboot after the reply went out (endpoint action.reboot)
	volatile uint8_t inertia_pending;		// run the inertia calibration (endpoint action.measure_inertia)
	volatile uint8_t save_status;			// result of the last save: 1 ok, 2 skipped, 3 failed
}FSMStruct;

void run_fsm(FSMStruct* fsmstate);
void run_fsm_slow(FSMStruct* fsmstate);
void update_fsm(FSMStruct * fsmstate, char fsm_input);
/* Request a state transition from non-ISR context (endpoint actions).  Writes
 * next_state/ready as one critical section, because run_fsm() reads them from
 * the control interrupt.  update_fsm() remains the serial/CAN input path. */
void request_fsm_state(FSMStruct * fsmstate, uint8_t next);
void fsm_enter_state(FSMStruct * fsmstate);
void fsm_exit_state(FSMStruct * fsmstate);
void enter_menu_state(void);
void enter_setup_state(void);
void enter_motor_mode(void);
void process_user_input(FSMStruct * fsmstate);

#ifdef __cplusplus
}
#endif

#endif /* INC_FSM_H_ */
