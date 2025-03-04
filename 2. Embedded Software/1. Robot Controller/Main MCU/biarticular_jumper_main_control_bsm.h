/*
 * biarticular_jumper_main_control_bsm.h
 *
 *  Created on: 2024. 3. 21.
 *      Author: BSM
 */

#ifndef INC_BIARTICULAR_JUMPER_MAIN_CONTROL_BSM_H_
#define INC_BIARTICULAR_JUMPER_MAIN_CONTROL_BSM_H_

#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

#include "nrf_uart_hc12_bsm.h"
#include "file_handle_bsm.h"
#include "math_utils_bsm.h"
#include "bldc_motor_communication_main.h"


#define TIM_TAKEOFF		0.85f
#define TIM_LANDING		2.0f//1.9f
#define CNTR_FREQ		500



typedef struct {
	uint8_t current_ctrl_cnt;
	uint8_t velocity_ctrl_cnt;
	uint8_t position_ctrl_cnt;
} _ROBOT_CONTROL_CNTS;

extern volatile uint8_t robot_state;
extern volatile uint32_t robot_state_cnt;




void BLDC_Control_Manual_Pot();
//void Jumping_Control_Auto_Single();
//void Jumping_Control_Auto_Repeat();

void Robot_Main_Controller();

int Tact_1_State();
int Tact_2_State();

void Check_Robot_State(uint32_t tim_cnt);


#endif /* INC_BIARTICULAR_JUMPER_MAIN_CONTROL_BSM_H_ */
