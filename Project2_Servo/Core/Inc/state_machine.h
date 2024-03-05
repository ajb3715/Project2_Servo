/*
 * state_machine.h
 *
 *  Created on: Oct 2, 2023
 *      Author: tcber
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

enum events process_event( enum events one_event, int servo,UART_HandleTypeDef* huart2 );
void process_servo1(void );
void process_servo2(void );
void state_machine(void);
void process_servo_command(char cmd, int servo);
void move_servo(int pos, int servo);
int check_errors(char cmd,int servo);
void init_servo1(void);
void init_servo2(void);


enum status
{
	status_running,
	status_paused,
	status_stopped,
	status_command_error,
	status_nested_error
} ;

enum servo_states
{
	state_at_position,		// use a separate integer to record the current position (0 through 5) for each servo.
	state_unknown,
	state_moving,
	state_waiting,
	state_recipe_ended
} ;

enum events
{
	recipe_begin,
	user_entered_left,
	user_entered_Right,
	user_entered_Pause,
	user_entered_Continue,
	user_entered_NOOP,
	user_entered_begin,
	user_entered_recipe1,
	user_entered_recipe2,
	user_entered_recipe3,
	user_entered_recipe4,
	user_entered_recipe5,
	recipe_ended,
	user_entered_invalid,
	user_entered_led1,
	user_entered_led2
} ;




#endif /* INC_STATE_MACHINE_H_ */
