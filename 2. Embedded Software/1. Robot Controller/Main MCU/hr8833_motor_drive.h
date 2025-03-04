/*
 * hr8833_motor_drive.h
 *
 *  Created on: Sep 12, 2024
 *      Author: BSM
 */

#ifndef INC_HR8833_MOTOR_DRIVE_H_
#define INC_HR8833_MOTOR_DRIVE_H_

#include "stm32h7xx_hal.h"
#include "ism330dhcx_spi_BSM.h"
#include "math_utils_bsm.h"

#define P_TAIL		1500.1f
#define I_TAIL		000.0f
#define D_TAIL		80.0f
#define V_GAIN_TAIL	0.010f
#define PID_TAIL_LIMIT		500.0f

#define P_TAIL_GYRO		-50.1f
#define I_TAIL_GYRO		0.0f
#define D_TAIL_GYRO		0.0f



typedef struct{
	TIM_HandleTypeDef *htim;
	float pitch_target;
	float gyro_target;
} _TAIL_MOTOR;


typedef struct{
	unsigned long t_prev;
	float error_prev;
	float integral_prev;
	float out_prev;
} _TAIL_MOTOR_PID;



extern _TAIL_MOTOR _tail_motor;
extern _TAIL_MOTOR_PID _tail_motor_pid;
extern _TAIL_MOTOR_PID _tail_gyro_pid;

void Tail_Pitch_Control_Init();
void Tail_Pitch_Control(float target_pitch);
void Tail_Pitch_PID(float target_pitch);
void Tail_Pitch_PID_Velocity_Feedback(float target_pitch);
void Tail_Gyro_Regulator_Init();
void Tail_Gyro_Regulator(float target_gyro);
void Tail_Gyro_PID(float target_gyro);

void Tail_Motor_Run(int16_t pwm_input); // if pwm_input > 0 : Forward run, else if pwm_input < 0 : Backward run
void Tail_Motor_Stop();

void Init_Tail_Motor(TIM_HandleTypeDef *htim);

#endif /* INC_HR8833_MOTOR_DRIVE_H_ */
