/*
 * state_machine.c
 *
 *  Created on: Mar 1, 2024
 *      Author: ajb3715
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <main.h>
#include <state_machine.h>

#define MOV 		(0x20)
#define WAIT 		(0x40)
#define LOOP 		(0x80)
#define END_LOOP	(0xA0)
#define TEST_ERROR	(0xFF)
#define RECIPE_END (0)

//Recipe end followed by commands
unsigned char recipe1[] = { MOV + 3, LOOP + 10, MOV + 4, MOV + 5, MOV + 0, MOV
		+ 1, END_LOOP, RECIPE_END, LOOP + 5, MOV + 4, MOV + 5, MOV + 0, MOV + 1,
		END_LOOP, };

unsigned char recipe2[] = { MOV + 0, MOV + 5, MOV + 0, MOV + 3, LOOP + 0, MOV
		+ 1, MOV + 4, END_LOOP, MOV + 0, MOV + 2, WAIT + 0, MOV + 3, WAIT + 0,
		MOV + 2, MOV + 3, WAIT + 31, WAIT + 31, MOV + 4, RECIPE_END };

//Error Test
unsigned char recipe3[] = { MOV + 3, MOV + 2, TEST_ERROR, MOV + 4, RECIPE_END };
unsigned char recipe4[] = { MOV + 1, WAIT + 9, MOV + 2, WAIT + 9, MOV + 3, WAIT
		+ 9, MOV + 4, WAIT + 9, MOV + 5, WAIT + 9, MOV + 6, RECIPE_END };

//Verifies all positions
unsigned char recipe5[] = { LOOP + 15, WAIT + 9, MOV + 1, WAIT + 9, MOV + 2,
		WAIT + 9, MOV + 3, WAIT + 9, MOV + 4, WAIT + 9, MOV + 5, MOV + 0,
		END_LOOP, RECIPE_END };

unsigned char *recipes[] = { recipe1, recipe2, recipe3, recipe4, recipe5, NULL };

static unsigned int servo1_index = 0;

static unsigned int servo2_index = 0;

unsigned int servo1_recipe = 0;

unsigned int servo2_recipe = 0;

enum servo_states servo1_state = state_at_position;
enum servo_states servo2_state = state_at_position;
enum status current_status_servo1 = status_paused;
enum status current_status_servo2 = status_paused;
int servo1_pos = 0;
int servo2_pos = 0;
int servo1_in_loop = 0;
int servo2_in_loop = 0;
int servo1_move_cycles = 0;
int servo2_move_cycles = 0;
int servo1_wait_cycles = 0;
int servo2_wait_cycles = 0;
int servo1_loop_index_start = 0;
int servo2_loop_index_start = 0;
int servo1_loop_index_end = 0;
int servo2_loop_index_end = 0;
int servo1_loop_times = 0;
int servo2_loop_times = 0;
int user_led1 = 0;
int user_led2 = 0;
UART_HandleTypeDef huart;

enum events process_event(enum events one_event, int servo,
		UART_HandleTypeDef *huart2) {
	huart = *huart2;
	char *Buffer = malloc(256);
	if (Buffer == NULL) {
		exit(98);
	}
	switch (one_event) {
	//Change the servos to start their recipes
	case (recipe_begin):
	case (user_entered_begin):
		if (servo == 1) {
			init_servo1();
			current_status_servo1 = status_running;
			servo1_state = state_at_position;
			servo1_index = 0;
		} else {
			init_servo2();
			current_status_servo2 = status_running;
			servo2_state = state_at_position;
			servo2_index = 0;
		}
		break;
		//Start the servo back up
	case (user_entered_Continue):
		if (servo == 1) {
			current_status_servo1 = status_running;
		} else {
			current_status_servo2 = status_running;
		}
		break;
		//Pause the servo motor
	case (user_entered_Pause):
		if (servo == 1) {
			current_status_servo1 = status_paused;
		} else {
			current_status_servo2 = status_paused;
		}
		break;

	case (user_entered_Right):
		if (servo == 1) {
			//check if the recipe is paused and not at the rightmost position
			if (current_status_servo1 == status_paused) {
				if (servo1_pos != 0) {
					Move_Servo(servo1_pos - 1, 1);
					servo1_pos--;
				}
			}
		} else {
			//Do the same for the second servo if that command is entered
			if (current_status_servo2 == status_paused) {
				if (servo2_pos != 0) {
					Move_Servo(servo2_pos - 1, 2);
					servo2_pos--;
				}
			}
		}
		break;

	case (user_entered_left):
		if (servo == 1) {
			if (current_status_servo1 == status_paused) {
				if (servo1_pos != 5) {
					Move_Servo(servo1_pos + 1, 1);
					servo1_pos++;
				}
			}
		} else {
			if (current_status_servo2 == status_paused) {
				if (servo2_pos != 5) {
					Move_Servo(servo2_pos + 1, 2);
					servo2_pos++;
				}
			}
		}

		break;
		//Selects each servo's own recipe
	case (user_entered_recipe1):
		if (servo == 1) {
			servo1_recipe = 1;
			servo1_index = 0;
			servo1_state = state_at_position;
			sprintf(Buffer, "User selected recipe %d for servo %d \r\n", 1,
					servo);
			HAL_UART_Transmit(huart2, (uint8_t*) Buffer, strlen(Buffer), 200);
		} else {
			servo2_recipe = 1;
			servo2_index = 0;
			servo2_state = state_at_position;
			sprintf(Buffer, "User selected recipe %d for servo %d \r\n", 1,
					servo);
			HAL_UART_Transmit(huart2, (uint8_t*) Buffer, strlen(Buffer), 200);
		}

		break;
	case (user_entered_recipe2):
		if (servo == 1) {
			servo1_recipe = 2;
			servo1_index = 0;
			servo1_state = state_at_position;
			sprintf(Buffer, "User selected recipe %d for servo %d \r\n", 2,
					servo);
			HAL_UART_Transmit(huart2, (uint8_t*) Buffer, strlen(Buffer), 200);
		} else {
			servo2_recipe = 2;
			servo2_index = 0;
			servo2_state = state_at_position;
			sprintf(Buffer, "User selected recipe %d for servo %d \r\n", 2,
					servo);
			HAL_UART_Transmit(huart2, (uint8_t*) Buffer, strlen(Buffer), 200);
		}

		break;
	case (user_entered_recipe3):
		if (servo == 1) {
			servo1_recipe = 3;
			servo1_index = 0;
			servo1_state = state_at_position;
			sprintf(Buffer, "User selected recipe %d for servo %d \r\n", 3,
					servo);
			HAL_UART_Transmit(huart2, (uint8_t*) Buffer, strlen(Buffer), 200);
		} else {
			servo2_recipe = 3;
			servo2_index = 0;
			servo2_state = state_at_position;
			sprintf(Buffer, "User selected recipe %d for servo %d \r\n", 3,
					servo);
			HAL_UART_Transmit(huart2, (uint8_t*) Buffer, strlen(Buffer), 200);
		}
		break;
	case (user_entered_recipe4):
		if (servo == 1) {
			servo1_recipe = 4;
			servo1_index = 0;
			servo1_state = state_at_position;
			sprintf(Buffer, "User selected recipe %d for servo %d \r\n", 4,
					servo);
			HAL_UART_Transmit(huart2, (uint8_t*) Buffer, strlen(Buffer), 200);
		} else {
			servo2_recipe = 4;
			servo2_index = 0;
			servo2_state = state_at_position;
			sprintf(Buffer, "User selected recipe %d for servo %d \r\n", 4,
					servo);
			HAL_UART_Transmit(huart2, (uint8_t*) Buffer, strlen(Buffer), 200);
		}
		break;
	case (user_entered_recipe5):
		if (servo == 1) {
			servo1_recipe = 5;
			servo1_index = 0;
			servo1_state = state_at_position;
			sprintf(Buffer, "User selected recipe %d for servo %d \r\n", 5,
					servo);
			HAL_UART_Transmit(huart2, (uint8_t*) Buffer, strlen(Buffer), 200);
		} else {
			servo2_recipe = 5;
			servo2_index = 0;
			servo2_state = state_at_position;
			sprintf(Buffer, "User selected recipe %d for servo %d \r\n", 5,
					servo);
			HAL_UART_Transmit(huart2, (uint8_t*) Buffer, strlen(Buffer), 200);
		}

		break;
	default:
		//handle errors
		break;

	}
	return one_event;
}

void process_servo1(void) {
	char cmd;
	switch (servo1_state) {
	case (state_at_position):
		//Do another command
		cmd = recipes[servo1_recipe - 1][servo1_index];
		int check = check_errors(cmd, 1);
		if (check == 0) {
			process_servo_command(cmd, 1);
		}
		break;
	case (state_moving):
		if (servo1_move_cycles != 0) {
			servo1_move_cycles--;

		} else {
			//Change state to at position and ready for a new command
			servo1_state = state_at_position;
			servo1_index++;
		}
		break;

	case (state_recipe_ended):
		init_servo1();
		servo1_state = state_at_position;
		break;

	case (state_unknown):
		//Error State
		break;

	case (state_waiting):
		if (servo1_wait_cycles != 0) {
			servo1_wait_cycles--;
		} else {
			servo1_state = state_at_position;
			servo1_index++;
		}
		break;
	}

}

void process_servo2(void) {
	char cmd;
	switch (servo2_state) {
	case (state_at_position):
		//Do another command
		cmd = recipes[servo2_recipe - 1][servo2_index];
		int check = check_errors(cmd, 2);
		if (check == 0) {
			process_servo_command(cmd, 2);
		}
		break;
	case (state_moving):
		//decrement the move cycles until it is 0, meaning we are at the position
		if (servo2_move_cycles != 0) {
			servo2_move_cycles--;
		} else {
			//Change state to at position and ready for a new command
			servo2_state = state_at_position;
			servo2_index++;
		}
		break;

	case (state_waiting):
		if (servo2_wait_cycles != 0) {
			servo2_wait_cycles--;
		} else {
			servo2_state = state_at_position;
			servo2_index++;
		}
		break;

	case (state_recipe_ended):
		init_servo2();
		servo2_state = state_at_position;
		break;

	case (state_unknown):
		//Error State
		break;
	}

}

void state_machine() {
	switch (current_status_servo1) {
	case status_running:
		//Step the servo1 thorough one recipe
		process_servo1();
		break;
	case (status_paused):
		break;
	case status_command_error:
		break;
	case status_nested_error:
		break;
	case status_stopped:
		;
		break;

	}

	switch (current_status_servo2) {
	case status_running:
		//Step the servos thorough one recipe
		process_servo2();
		break;
	case status_paused:
		;
		break;
	case status_command_error:
		;
		break;
	case status_nested_error:
		;
		break;
	case status_stopped:
		;
		break;

	}

}

void process_servo_command(char cmd, int servo) {
	int opcode = cmd & 0xE0;
	int parameter = cmd & 0x1F;
	switch (opcode) {
	case MOV:
		//DO MOV command
		if (parameter <= 5) {
			move_servo(parameter, servo);
			if (servo == 1) {
				servo1_move_cycles = abs(servo1_pos - parameter);
				servo1_state = state_moving;
			} else {
				servo2_move_cycles = abs(servo1_pos - parameter);
				servo2_state = state_moving;
			}
		} else {
			//TO DO: Implement error for out of range
			if (servo == 1) {
				current_status_servo1 = status_command_error;
			} else {
				current_status_servo2 = status_command_error;
			}
		}
		break;
	case WAIT:
		//DO Wait command
		if (servo == 1) {
			//Wait the amount of cycles now
			servo1_state = state_waiting;
			servo1_wait_cycles = parameter + 1;
		} else {
			servo2_state = state_waiting;
			servo2_wait_cycles = parameter + 1;
		}
		break;
	case LOOP:
		//DO LOOP command
		if (servo == 1) {
			servo1_loop_index_start = servo1_index;
			servo1_loop_times = parameter + 1;
			servo1_index++;
			servo1_in_loop = 1;
		} else {
			servo2_loop_index_start = servo2_index;
			servo2_loop_times = parameter + 1;
			servo2_index++;
			servo2_in_loop = 1;
		}
		break;
	case END_LOOP:
		//DO ENDLOOP command
		if (servo == 1) {
			//finished iterations, move past loop
			if (servo1_loop_times == 0) {
				servo1_index++;
				servo1_in_loop = 0;
			} else {
				servo1_loop_index_end = servo1_index;
				servo1_index = servo1_loop_index_start + 1;
				servo1_loop_times--;
			}
		} else {
			if (servo2_loop_times == 0) {
				servo2_index++;
				servo2_in_loop = 0;
			} else {
				servo2_loop_index_end = servo2_index;
				servo2_index = servo2_loop_index_start + 1;
				servo2_loop_times--;
			}
		}
		break;
	case RECIPE_END:
		//Handle recipe end
		if (servo == 1) {
			servo1_state = state_recipe_ended;
		} else {
			servo2_state = state_recipe_ended;
		}
		break;
		//Handle Extra command
	default:
		break;

	}

}
void move_servo(int pos, int servo) {
	switch (pos) {
	case 0:
		if (servo == 1) {
			MOVE_MOTOR1_POS1()
		} else {
			MOVE_MOTOR2_POS1()
		}
		break;
	case 1:
		if (servo == 1) {
			MOVE_MOTOR1_POS2()
		} else {
			MOVE_MOTOR2_POS2()
		}
		break;
	case 2:
		if (servo == 1) {
			MOVE_MOTOR1_POS3()
		} else {
			MOVE_MOTOR2_POS3()
		}
		break;
	case 3:
		if (servo == 1) {
			MOVE_MOTOR1_POS4()
		} else {
			MOVE_MOTOR2_POS4()
		}
		break;
	case 4:
		if (servo == 1) {
			MOVE_MOTOR1_POS5()
		} else {
			MOVE_MOTOR2_POS5()
		}
		break;
	default:
		if (servo == 1) {
			MOVE_MOTOR1_POS6()
		} else {
			MOVE_MOTOR2_POS6()
		}
		break;
	}
}
// returns 0 if no problems with command, otherwise will return -1 for nested error
int check_errors(char cmd, int servo) {
	int opcode = cmd & 0xE0;
	if ((opcode == LOOP) && servo1_in_loop && servo == 1) {
		current_status_servo1 = status_nested_error;
		return -1;
	}
	if ((opcode == LOOP) && servo2_in_loop && servo == 2) {
		current_status_servo2 = status_nested_error;
		return -1;
	}
	return 0;

}
void init_servo1() {
	MOVE_MOTOR1_POS1()
	servo1_index = 0;
	servo1_loop_index_end = 0;
	servo1_loop_index_start = 0;
	servo1_loop_times = 0;
	servo1_pos = 0;
	current_status_servo1 = status_paused;
}

void init_servo2() {
	MOVE_MOTOR2_POS1()
	servo2_index = 0;
	servo2_loop_index_end = 0;
	servo2_loop_index_start = 0;
	servo2_loop_times = 0;
	servo2_pos = 0;
	current_status_servo2 = status_paused;
}
