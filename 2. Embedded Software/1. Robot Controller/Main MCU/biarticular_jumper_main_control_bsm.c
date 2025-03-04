/*
 * biarticular_jumper_main_control_bsm.c
 *
 *  Created on: 2024. 3. 21.
 *      Author: BSM
 */
#include "biarticular_jumper_main_control_bsm.h"



volatile uint8_t robot_state = 0;
volatile uint32_t robot_state_cnt = 0;





void BLDC_Control_Manual_Pot()
{
	uint8_t pot_data = _robot_uart.pot;
	float target_position = (float) pot_data*_2PI/256;
	//BLDC_Motor_Cascade_Position_Control(target_position*BLDC_GEAR_RATIO);



	float target_velocity = (float) (pot_data-125)*5;
	//BLDC_Motor_Cascade_Velocity_Control(target_velocity);


	float target_current = (float) (pot_data)/80;
	//BLDC_Motor_Cascade_Current_Control(target_current);


	//BLDC_Motor_Openloop_Velocity_Control(target_velocity);
	//BLDC_Motor_Openloop_Position_Control(target_position);
}


void Robot_Main_Controller()
{
	uint8_t dip_bit_1 = 0;
	uint8_t dip_bit_2 = 0;
	uint8_t dip_bit_3 = 0;
	uint8_t dip_bit_4 = 0;

	dip_bit_1 = (_robot_uart.dip_state >> 3) & 1;
	dip_bit_2 = (_robot_uart.dip_state >> 2) & 1;
	dip_bit_3 = (_robot_uart.dip_state >> 1) & 1;
	dip_bit_4 = (_robot_uart.dip_state) & 1;

	if(dip_bit_2 & (!dip_bit_3))
	{
		// Auto, Repeat Jumping
	}

	if(dip_bit_3 & (!dip_bit_2))
	{
		// Auto, Single Jumping
	}

	if((!dip_bit_2) & (!dip_bit_3))
	{
		// Manual Controller Pot BLDC run
		BLDC_Control_Manual_Pot();

	}

	if(dip_bit_1 & dip_bit_4)
	{
		// uSD close
		uint8_t file_closed = 0;
		char sprintf_buff[255];
		//sprintf(sprintf_buff, "EOF \r\n");
		//Write_openedFile(sprintf_buff);
		//BLDC_Motor_Set_Phase_Voltage_Trap_150(0, 0, 0);

		while(!file_closed)
		{
			if(Close_File() == FR_OK) file_closed = 1;
		}

		if(file_closed)
		{
			while(1)
			{
				Off_BLDC_CMD();
				HAL_GPIO_TogglePin(LED_GPIO_Output_GPIO_Port, LED_GPIO_Output_Pin);
				HAL_Delay(500);
			}

		}

	}

}
int Tact_1_State()
{
	return _bldc_cmd.ref_cmd[3] >> 4;
}

int Tact_2_State()
{
	return ((_bldc_cmd.ref_cmd[3] >> 3) & 1);
}


void Check_Robot_State(uint32_t tim_cnt)
{
	uint32_t tim_delta;

	if((robot_state == 0) && (Tact_1_State() == 1))	// if robot in rest & tact 1 on => robot state energy storage and push off
	{
		robot_state = 1;
		robot_state_cnt = tim_cnt;
	}

	tim_delta = tim_cnt - robot_state_cnt;		// cnt time after robot starts energy storage

	if((robot_state == 1) && (tim_delta > TIM_TAKEOFF*CNTR_FREQ))	// if robot in energy storage & push off duration is passed => robot state mid-air
	{
		robot_state = 2;
	}

	if((robot_state == 2) && (tim_delta > TIM_LANDING*CNTR_FREQ))	// if robot in mid-air state & flight duration is passed => robot state landing
	{
		robot_state = 0;
	}

	if((robot_state == 0) && (Tact_2_State() == 1))
	{
		robot_state = 21;
		robot_state_cnt = tim_cnt;
	}

	if((robot_state == 21) && (tim_delta > 0.01*CNTR_FREQ))
	{
		robot_state = 22;
	}

	if((robot_state == 22) && (tim_delta > TIM_LANDING*CNTR_FREQ))
	{
		robot_state = 0;
	}

}
