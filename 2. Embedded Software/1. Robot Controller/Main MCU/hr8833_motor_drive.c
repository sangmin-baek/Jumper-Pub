/*
 * hr8833_motor_drive.c
 *
 *  Created on: Sep 12, 2024
 *      Author: BSM
 */

/*
 *  [HR8833]
 *  	[	On / Off mode	]
 *  	|	in1	|	in2	|	out1	|	out2	|		Function		|
 *  	|	0	|	0	|		Z	|		Z	|	Coast / Fast decay	|
 *  	|	0	|	1	|		L	|		H	|		Reverse			|
 *   	|	1	|	0	|		H	|		L	|		Forward			|
 *  	|	1	|	1	|		L	|		L	|	Brake / Slow decay	|
 *  	[	PWM mode	]
 *  	|	in1	|	in2	|			Function			|		Speed	|
 *  	|	PWM	|	0	|	Forward PWM / Fast decay	|	High duty	|
 *  	|	1	|	PWM	|	Forward PWM / Slow decay	|  	Low duty	|
 *   	|	0	|	PWM	|	Reverse PWM / Fast decay	|	High duty	|
 *  	|	PWM	|	1	|	Reverse PWM / Slow decay	|  	Low duty	|
 *
 */

#include "hr8833_motor_drive.h"


_TAIL_MOTOR _tail_motor;
_TAIL_MOTOR_PID _tail_motor_pid;
_TAIL_MOTOR_PID _tail_gyro_pid;

void Tail_Gyro_Regulator_Init()
{
	_tail_gyro_pid.error_prev = 0;
	_tail_gyro_pid.integral_prev = 0;
	_tail_gyro_pid.out_prev = 0;
	_tail_gyro_pid.t_prev = 0;
}


void Tail_Gyro_Regulator(float target_gyro)
{
	Tail_Gyro_PID(target_gyro);
	Tail_Motor_Run((int16_t)_tail_gyro_pid.out_prev);
}

void Tail_Gyro_PID(float target_gyro)
{
	unsigned long current_t = _micros();
	float dt = (float)(current_t - _tail_gyro_pid.t_prev)*1e-6;
	if(dt <= 0 || dt > 0.5f) dt = 2e-3f;

	float error = target_gyro - _imu_data.gyro.y;

	float p_out = P_TAIL_GYRO*error;
	float i_out = _tail_gyro_pid.integral_prev + 0.5f*I_TAIL_GYRO*dt*(_tail_gyro_pid.error_prev + error);
	i_out = _constrain(i_out, -PID_TAIL_LIMIT, PID_TAIL_LIMIT);
	float d_out = D_TAIL_GYRO*(error - _tail_gyro_pid.error_prev)/dt;

	float pid_out = p_out + i_out + d_out;
	pid_out = _constrain(pid_out, -PID_TAIL_LIMIT, PID_TAIL_LIMIT);

	_tail_gyro_pid.t_prev = current_t;
	_tail_gyro_pid.error_prev = error;
	_tail_gyro_pid.integral_prev = i_out;
	_tail_gyro_pid.out_prev = pid_out;
}


void Tail_Pitch_Control_Init()
{
	_tail_motor_pid.error_prev = 0;
	_tail_motor_pid.integral_prev = 0;
	_tail_motor_pid.out_prev = 0;
	_tail_motor_pid.t_prev = 0;
}

void Tail_Pitch_Control(float target_pitch)
{
	Tail_Pitch_PID(target_pitch);
	//Tail_Pitch_PID_Velocity_Feedback(target_pitch);
	Tail_Motor_Run((int16_t)_tail_motor_pid.out_prev);
}

void Tail_Pitch_PID(float target_pitch)
{
	unsigned long current_t = _micros();
	float dt = (float)(current_t - _tail_motor_pid.t_prev)*1e-6;
	if(dt <= 0 || dt > 0.5f) dt = 2e-3f;

	float error = target_pitch - _imu_filter.pitch_gyro_int;

	float p_out = P_TAIL*error;
	float i_out = _tail_motor_pid.integral_prev + 0.5f*I_TAIL*dt*(_tail_motor_pid.error_prev + error);
	i_out = _constrain(i_out, -PID_TAIL_LIMIT, PID_TAIL_LIMIT);
	float d_out = D_TAIL*(error - _tail_motor_pid.error_prev)/dt;

	float pid_out = p_out + i_out + d_out;
	pid_out = _constrain(pid_out, -PID_TAIL_LIMIT, PID_TAIL_LIMIT);

	_tail_motor_pid.t_prev = current_t;
	_tail_motor_pid.error_prev = error;
	_tail_motor_pid.integral_prev = i_out;
	_tail_motor_pid.out_prev = pid_out;
}


void Tail_Pitch_PID_Velocity_Feedback(float target_pitch)
{
	unsigned long current_t = _micros();
	float dt = (float)(current_t - _tail_motor_pid.t_prev)*1e-6;
	if(dt <= 0 || dt > 0.5f) dt = 2e-3f;

	float error = target_pitch - _imu_filter.pitch_gyro_int - V_GAIN_TAIL*_imu_data.gyro.y;

	float p_out = P_TAIL*error;
	float i_out = _tail_motor_pid.integral_prev + 0.5f*I_TAIL*dt*(_tail_motor_pid.error_prev + error);
	i_out = _constrain(i_out, -PID_TAIL_LIMIT, PID_TAIL_LIMIT);
	float d_out = D_TAIL*(error - _tail_motor_pid.error_prev)/dt;

	float pid_out = p_out + i_out + d_out;
	pid_out = _constrain(pid_out, -PID_TAIL_LIMIT, PID_TAIL_LIMIT);

	_tail_motor_pid.t_prev = current_t;
	_tail_motor_pid.error_prev = error;
	_tail_motor_pid.integral_prev = i_out;
	_tail_motor_pid.out_prev = pid_out;
}


void Tail_Motor_Run(int16_t pwm_input) // if pwm_input > 0 : Forward run, else if pwm_input < 0 : Backward run
{
	uint16_t abs_pwm_input = 0;

	if(pwm_input >> 15)	// negative
	{
		abs_pwm_input = -pwm_input;
		_tail_motor.htim->Instance->CCR1 = abs_pwm_input;
		_tail_motor.htim->Instance->CCR2 = 1;
	}
	else	// positive
	{
		abs_pwm_input = pwm_input;
		_tail_motor.htim->Instance->CCR1 = 1;
		_tail_motor.htim->Instance->CCR2 = abs_pwm_input;
	}
}


void Tail_Motor_Stop()
{
	_tail_motor.htim->Instance->CCR1 = _tail_motor.htim->Instance->ARR;
	_tail_motor.htim->Instance->CCR2 = _tail_motor.htim->Instance->ARR;
}

void Init_Tail_Motor(TIM_HandleTypeDef *htim)
{
	_tail_motor.htim = htim;

	_tail_motor.pitch_target = 0;
	_tail_motor.gyro_target = 0;

	_tail_motor_pid.t_prev = 0;
	_tail_motor_pid.integral_prev = 0;
	_tail_motor_pid.error_prev = 0;
	_tail_motor_pid.out_prev = 0;
}
