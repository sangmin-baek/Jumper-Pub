/*
 * bldc_motor_control_bsm.c
 *
 *  Created on: 2024. 3. 24.
 *      Author: BSM
 */


#include "bldc_motor_control_bsm.h"

_BLDC_MOTOR _bldc_motor;

_BLDC_PID _bldc_pid_current_dc;
_BLDC_PID _bldc_pid_current_d;
_BLDC_PID _bldc_pid_current_q;
_BLDC_PID _bldc_pid_velocity;
_BLDC_PID _bldc_pid_position;

volatile uint32_t motor_tim_cnt = 0;
uint8_t motor_run_flag;
uint8_t motor_position_e_storage_flag;
uint8_t motor_position_init_flag;
float motor_current_geared_angle;

int trap_150_map[12][3] = {
                           { 0,  1, -1},
                           {-1,  1, -1},
                           {-1,  1,  0},
                           {-1,  1,  1},
                           {-1,  0,  1},
                           {-1, -1,  1},
                           { 0, -1,  1},
                           { 1, -1,  1},
                           { 1, -1,  0},
                           { 1, -1, -1},
                           { 1,  0, -1},
                           { 1,  1, -1}
};


void BLDC_Motor_Init(TIM_HandleTypeDef *htim)
{
	_bldc_motor.htim = htim;
	_bldc_motor.angle_geared = 0;
	_bldc_motor.velocity_geared = 0;

	_bldc_motor.angle_shaft = 0;
	_bldc_motor.velocity_shaft = 0;

	_bldc_motor.angle_electrical = 0;
	_bldc_motor.zero_elec_angle = 0;

	_bldc_motor.target_angle = 0;
	_bldc_motor.target_velocity = 0;
	_bldc_motor.target_current_dc = 0;
	_bldc_motor.target_current_d = 0;
	_bldc_motor.target_current_q = 0;

	_bldc_motor.phase_volt_a = 0;
	_bldc_motor.phase_volt_b = 0;
	_bldc_motor.phase_volt_c = 0;

	_bldc_motor.ctrl_flag_current = 0;
	_bldc_motor.ctrl_flag_velocity = 0;
	_bldc_motor.ctrl_flag_position = 0;

	_bldc_motor.openloop_t_prev = 0;

	HAL_TIM_PWM_Start(_bldc_motor.htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(_bldc_motor.htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(_bldc_motor.htim, TIM_CHANNEL_3);
}


void BLDC_Motor_Get_States()
{

	BLDC_Motor_Get_Shaft_Angle();
	BLDC_Motor_Get_Shaft_Velocity();
	BLDC_Motor_Get_Geared_Angle();
	BLDC_Motor_Get_Geared_Velocity();
	BLDC_Motor_Get_Elec_Angle();
}


void BLDC_Motor_Get_Geared_Angle()
{
	_bldc_motor.angle_geared = _bldc_motor.angle_shaft/BLDC_GEAR_RATIO;
}
void BLDC_Motor_Get_Geared_Velocity()
{
	_bldc_motor.velocity_geared = _bldc_motor.velocity_shaft/BLDC_GEAR_RATIO;
}


void BLDC_Motor_Get_Shaft_Angle()
{
	//LPF_ANGLE();
	_bldc_motor.angle_shaft = _lpf_angle.filter_out_prev;
}


void BLDC_Motor_Get_Shaft_Velocity()
{
	//LPF_VELOCITY();
	_bldc_motor.velocity_shaft = _lpf_velocity.filter_out_prev;
}


void BLDC_Motor_Get_Elec_Angle()
{
	_bldc_motor.angle_electrical = _normalize_angle( (float)(BLDC_SENSOR_DIRECTION * BLDC_POLE_PAIRS)*_bldc_motor.angle_shaft - _bldc_motor.zero_elec_angle );
}



void BLDC_Motor_Control_Init()
{


/* FOC code */

	/*******************************
	 * Abs Zero Search *************
	 *******************************/

	float zero_search_target = 1.5f*_2PI;
	_bldc_motor.angle_shaft = 0;
	while(_bldc_motor.angle_shaft < _2PI)
	{
		unsigned long current_t = _micros();
		float dt = (float)(current_t - _bldc_motor.openloop_t_prev)*1e-6f;
		if(dt <= 0 || dt > 0.5f) dt = 1e-4f;

		if(fabs(zero_search_target - _bldc_motor.angle_shaft) > fabs(BLDC_VELOCITY_ALIGN*dt))
		{
			_bldc_motor.angle_shaft += _sign(zero_search_target - _bldc_motor.angle_shaft)*fabs(BLDC_VELOCITY_ALIGN)*dt;
			_bldc_motor.velocity_shaft = BLDC_VELOCITY_ALIGN;
		}
		else
		{
			_bldc_motor.angle_shaft = zero_search_target;
			_bldc_motor.velocity_shaft = 0;
		}
		_bldc_motor.target_current_q = _constrain(BLDC_VOLTAGE_ALIGN, -BLDC_VOLTAGE_LIMIT, BLDC_VOLTAGE_LIMIT);
		_bldc_motor.angle_electrical = _normalize_angle(_bldc_motor.angle_shaft)*BLDC_POLE_PAIRS;
		BLDC_Motor_Set_Phase_Voltage_FOC(_bldc_motor.target_current_q, 0, _bldc_motor.angle_electrical);
		_bldc_motor.openloop_t_prev = current_t;

		AS5047P_Get_Raw_Angle_Velocity();
	}
	BLDC_Motor_Set_Phase_Voltage_FOC(0, 0, 0);



	/*******************************
	 * Set Zero Elec Angle *********
	 *******************************/

	BLDC_Motor_Set_Phase_Voltage_FOC(BLDC_VOLTAGE_ALIGN, 0, _3PI_2);
	HAL_Delay(2000);
	AS5047P_Get_Raw_Angle_Velocity();
	_bldc_motor.zero_elec_angle = 0;
	BLDC_Motor_Get_States();
	_bldc_motor.zero_elec_angle = _bldc_motor.angle_electrical;
	BLDC_Motor_Set_Phase_Voltage_FOC(0, 0, 0);
	HAL_Delay(200);



/* Trapezoidal code */

	/*******************************
	 * Abs Zero Search *************
	 *******************************/
/*
	float zero_search_target = 1.5f*_2PI;
	_bldc_motor.angle_shaft = 0;
	while(_bldc_motor.angle_shaft < _2PI)
	{
		unsigned long current_t = _micros();
		float dt = (float)(current_t - _bldc_motor.openloop_t_prev)*1e-6f;
		if(dt <= 0 || dt > 0.5f) dt = 1e-4f;

		if(fabs(zero_search_target - _bldc_motor.angle_shaft) > fabs(BLDC_VELOCITY_ALIGN*dt))
		{
			_bldc_motor.angle_shaft += _sign(zero_search_target - _bldc_motor.angle_shaft)*fabs(BLDC_VELOCITY_ALIGN)*dt;
			_bldc_motor.velocity_shaft = BLDC_VELOCITY_ALIGN;
		}
		else
		{
			_bldc_motor.angle_shaft = zero_search_target;
			_bldc_motor.velocity_shaft = 0;
		}
		_bldc_motor.target_current_q = _constrain(BLDC_VOLTAGE_ALIGN, -BLDC_VOLTAGE_LIMIT, BLDC_VOLTAGE_LIMIT);
		_bldc_motor.angle_electrical = _normalize_angle(_bldc_motor.angle_shaft)*BLDC_POLE_PAIRS;
		BLDC_Motor_Set_Phase_Voltage_Trap_150(_bldc_motor.target_current_q, 0, _bldc_motor.angle_electrical);
		_bldc_motor.openloop_t_prev = current_t;

		AS5047P_Get_Raw_Angle_Velocity();
	}
	BLDC_Motor_Set_Phase_Voltage_Trap_150(0, 0, 0);
*/


	/*******************************
	 * Set Zero Elec Angle *********
	 *******************************/
/*
	BLDC_Motor_Set_Phase_Voltage_Trap_150(BLDC_VOLTAGE_ALIGN, 0, _3PI_2);
	HAL_Delay(2000);
	AS5047P_Get_Raw_Angle_Velocity();
	_bldc_motor.zero_elec_angle = 0;
	BLDC_Motor_Get_States();
	_bldc_motor.zero_elec_angle = _bldc_motor.angle_electrical;
	BLDC_Motor_Set_Phase_Voltage_Trap_150(0, 0, 0);
	HAL_Delay(200);
*/

}


void BLDC_Motor_Control_Init_Trapz()
{


/* Trapezoidal code */

	/*******************************
	 * Abs Zero Search *************
	 *******************************/

	float zero_search_target = 1.5f*_2PI;
	_bldc_motor.angle_shaft = 0;
	while(_bldc_motor.angle_shaft < _2PI)
	{
		unsigned long current_t = _micros();
		float dt = (float)(current_t - _bldc_motor.openloop_t_prev)*1e-6f;
		if(dt <= 0 || dt > 0.5f) dt = 1e-4f;

		if(fabs(zero_search_target - _bldc_motor.angle_shaft) > fabs(BLDC_VELOCITY_ALIGN*dt))
		{
			_bldc_motor.angle_shaft += _sign(zero_search_target - _bldc_motor.angle_shaft)*fabs(BLDC_VELOCITY_ALIGN)*dt;
			_bldc_motor.velocity_shaft = BLDC_VELOCITY_ALIGN;
		}
		else
		{
			_bldc_motor.angle_shaft = zero_search_target;
			_bldc_motor.velocity_shaft = 0;
		}
		_bldc_motor.target_current_q = _constrain(BLDC_VOLTAGE_ALIGN, -BLDC_VOLTAGE_LIMIT, BLDC_VOLTAGE_LIMIT);
		_bldc_motor.angle_electrical = _normalize_angle(_bldc_motor.angle_shaft)*BLDC_POLE_PAIRS;
		BLDC_Motor_Set_Phase_Voltage_Trap_150(_bldc_motor.target_current_q, 0, _bldc_motor.angle_electrical);
		_bldc_motor.openloop_t_prev = current_t;

		AS5047P_Get_Raw_Angle_Velocity();
	}
	BLDC_Motor_Set_Phase_Voltage_Trap_150(0, 0, 0);



	/*******************************
	 * Set Zero Elec Angle *********
	 *******************************/

	BLDC_Motor_Set_Phase_Voltage_Trap_150(BLDC_VOLTAGE_ALIGN, 0, _3PI_2);
	HAL_Delay(2000);
	AS5047P_Get_Raw_Angle_Velocity();
	_bldc_motor.zero_elec_angle = 0;
	BLDC_Motor_Get_States();
	_bldc_motor.zero_elec_angle = _bldc_motor.angle_electrical;
	BLDC_Motor_Set_Phase_Voltage_Trap_150(0, 0, 0);
	HAL_Delay(200);


}

void BLDC_Motor_Controller(uint32_t tim_cnt)
{
	uint8_t index_found = 0;
	uint8_t start_index = 0;
	uint32_t current_tim_cnt = 0;

	while(!index_found)
	{
		if(_bldc_cmd.ref_cmd[start_index] == 0xAA)
		{
			if(_bldc_cmd.ref_cmd[(start_index + 1) % 5] == 0xBB)
			{
				index_found = 1;
			}
		}

		start_index = (start_index + 1) % 5;
	}

	uint8_t dip_state = _bldc_cmd.ref_cmd[(start_index + 1) % 5];
	uint8_t tact_state = _bldc_cmd.ref_cmd[(start_index + 2) % 5];
	uint8_t pot_data = _bldc_cmd.ref_cmd[(start_index + 3) % 5];

	uint8_t dip_bit_1 = (dip_state >> 3) & 1;
	uint8_t dip_bit_2 = (dip_state >> 2) & 1;
	uint8_t dip_bit_3 = (dip_state >> 1) & 1;
	uint8_t dip_bit_4 = (dip_state) & 1;

	uint8_t tact_1 = (tact_state >> 4) & 1;
	uint8_t tact_2 = (tact_state >> 3) & 1;
	uint8_t tact_3 = (tact_state >> 2) & 1;
	uint8_t tact_4 = (tact_state >> 1) & 1;
	uint8_t tact_5 = (tact_state) & 1;

	current_tim_cnt = tim_cnt;

	if(dip_bit_1 & dip_bit_4)
	{
		// uSD write, Motor stop
		BLDC_Motor_Cascade_Current_Control(0);
	}
	else
	{
		if(dip_bit_2 & (!dip_bit_3))
		{
			// Auto, Repeated Jumping
		}

		if(dip_bit_3 & (!dip_bit_2))
		{
			if(tact_1)
			{

				motor_position_e_storage_flag = 1;

				motor_position_init_flag = 0;
				motor_current_geared_angle = (floor((_lpf_angle.filter_out_prev + 0.1)/BLDC_GEAR_RATIO/_2PI))*_2PI*BLDC_GEAR_RATIO;

				motor_tim_cnt = tim_cnt;
			}


			if(motor_position_e_storage_flag == 1)
			{

				if((current_tim_cnt - motor_tim_cnt) > BLDC_TIM_TAKEOFF*BLDC_CNTR_FREQ)
				{
					motor_position_init_flag = 1;
					motor_position_e_storage_flag = 0;
				}
				if(tact_2)
				{
					motor_position_init_flag = 1;
					motor_position_e_storage_flag = 0;
				}

				if(tact_3)
				{
					motor_position_init_flag = 0;
					motor_position_e_storage_flag = 0;
				}

				float target_position = (float) pot_data*_2PI/256*BLDC_GEAR_RATIO;
				BLDC_Motor_Cascade_Position_Control(motor_current_geared_angle + 4.35*BLDC_GEAR_RATIO);

				float target_velocity = (float) (pot_data-128)*3;
				//BLDC_Motor_Cascade_Velocity_Control(target_velocity);

				float target_current = (float) (pot_data)/80;
				//BLDC_Motor_Cascade_Current_Control(target_current);

				//BLDC_Motor_Openloop_Velocity_Control(target_velocity);
				//BLDC_Motor_Openloop_Position_Control(target_position);
			}

			if(motor_position_init_flag == 1)
			{
				if(tact_1)
				{
					motor_position_init_flag = 0;
					motor_position_e_storage_flag = 1;
				}

				if(tact_3)
				{
					motor_position_init_flag = 0;
					motor_position_e_storage_flag = 0;
				}
				// Auto, Single Jumping
				float target_position = (float) pot_data*_2PI/256*BLDC_GEAR_RATIO;
				BLDC_Motor_Cascade_Position_Control(motor_current_geared_angle + _2PI*BLDC_GEAR_RATIO);



				float target_velocity = (float) (pot_data-128)*3;
				//BLDC_Motor_Cascade_Velocity_Control(target_velocity);


				float target_current = (float) (pot_data)/80;
				//BLDC_Motor_Cascade_Current_Control(target_current);


				//BLDC_Motor_Openloop_Velocity_Control(target_velocity);
				//BLDC_Motor_Openloop_Position_Control(target_position);
			}

			if((!motor_position_e_storage_flag) & (!motor_position_init_flag))
			{
				BLDC_Motor_Cascade_Current_Control(0);
			}

		}

		if((!dip_bit_3) & (!dip_bit_2))
		{
			if(tact_1)
			{
				motor_run_flag = 1;
			}

			if(motor_run_flag == 1)
			{
				if(tact_2)
				{
					motor_run_flag = 0;
				}

				// Manual, pot BLDC control
				float target_position = (float) pot_data*_2PI/256;
				//BLDC_Motor_Cascade_Position_Control(target_position*BLDC_GEAR_RATIO);



				float target_velocity = (float) (pot_data-128)*3;
				BLDC_Motor_Cascade_Velocity_Control(target_velocity);


				float target_current = (float) (pot_data)/80;
				//BLDC_Motor_Cascade_Current_Control(target_current);


				//BLDC_Motor_Openloop_Velocity_Control(target_velocity);
				//BLDC_Motor_Openloop_Position_Control(target_position);
			}

			if(motor_run_flag == 0)
			{
				BLDC_Motor_Cascade_Velocity_Control(0);
			}

		}
	}

}


void BLDC_Motor_Cascade_Current_Control(float target_current)
{
	_bldc_motor.ctrl_flag_current++;
	_bldc_motor.ctrl_flag_velocity++;
	_bldc_motor.ctrl_flag_position++;

	BLDC_Motor_Get_States();

	if(_bldc_motor.ctrl_flag_current == BLDC_CURRENT_LOOP_PERIOD)
	{
		BLDC_Motor_Current_Control(target_current);
		_bldc_motor.ctrl_flag_current = 0;
	}

	if(_bldc_motor.ctrl_flag_velocity == BLDC_VELOCITY_LOOP_PERIOD)
	{

		_bldc_motor.ctrl_flag_velocity = 0;
	}

	if(_bldc_motor.ctrl_flag_position == BLDC_POSITION_LOOP_PERIOD)
	{
		_bldc_motor.ctrl_flag_position = 0;
	}
}

void BLDC_Motor_Cascade_Velocity_Control(float target_velocity)
{
	_bldc_motor.ctrl_flag_current++;
	_bldc_motor.ctrl_flag_velocity++;
	_bldc_motor.ctrl_flag_position++;

	BLDC_Motor_Get_States();

	if(_bldc_motor.ctrl_flag_current == BLDC_CURRENT_LOOP_PERIOD)
	{
		BLDC_Motor_Current_Control(_bldc_pid_velocity.out_prev);
		_bldc_motor.ctrl_flag_current = 0;
	}

	if(_bldc_motor.ctrl_flag_velocity == BLDC_VELOCITY_LOOP_PERIOD)
	{
		BLDC_Motor_Velocity_Control(target_velocity);
		_bldc_motor.ctrl_flag_velocity = 0;
	}

	if(_bldc_motor.ctrl_flag_position == BLDC_POSITION_LOOP_PERIOD)
	{
		_bldc_motor.ctrl_flag_position = 0;
	}
}

void BLDC_Motor_Cascade_Position_Control(float target_position)
{
	_bldc_motor.ctrl_flag_current++;
	_bldc_motor.ctrl_flag_velocity++;
	_bldc_motor.ctrl_flag_position++;

	BLDC_Motor_Get_States();

	if(_bldc_motor.ctrl_flag_current == BLDC_CURRENT_LOOP_PERIOD)
	{
		BLDC_Motor_Current_Control(_bldc_pid_velocity.out_prev);
		_bldc_motor.ctrl_flag_current = 0;
	}

	if(_bldc_motor.ctrl_flag_velocity == BLDC_VELOCITY_LOOP_PERIOD)
	{
		BLDC_Motor_Velocity_Control(_bldc_pid_position.out_prev);
		_bldc_motor.ctrl_flag_velocity = 0;
	}

	if(_bldc_motor.ctrl_flag_position == BLDC_POSITION_LOOP_PERIOD)
	{
		BLDC_Motor_Position_Control(target_position);
		_bldc_motor.ctrl_flag_position = 0;
	}
}


void BLDC_Motor_Current_Control(float target_current)
{
	/* FOC */

	_bldc_motor.target_current_q = target_current;
	_bldc_motor.target_current_d = 0;
	BLDC_PID_Current_D();
	BLDC_PID_Current_Q();

	BLDC_Motor_Set_Phase_Voltage_FOC_v2(_bldc_pid_current_q.out_prev, _bldc_pid_current_d.out_prev, _bldc_motor.angle_electrical);


	/* Trapzoidal */
/*
	_bldc_motor.target_current_dc = target_current;
	BLDC_PID_Current_DC();

	BLDC_Motor_Set_Phase_Voltage_Trap_150(_bldc_pid_current_dc.out_prev, 0, _bldc_motor.angle_electrical);
*/
}

void BLDC_Motor_Velocity_Control(float target_velocity)
{
	_bldc_motor.target_velocity = target_velocity;
	BLDC_PID_Velocity();
}

void BLDC_Motor_Position_Control(float target_position)
{
	_bldc_motor.target_angle = target_position;
	BLDC_PID_Position();
}


void BLDC_Motor_Openloop_Velocity_Control(float target_velocity)
{
	unsigned long current_t = _micros();
	float dt = (float)(current_t - _bldc_motor.openloop_t_prev)*1e-6f;
	if(dt <= 0 || dt > 0.5f) dt = 1e-4f;

	_bldc_motor.angle_shaft = _normalize_angle(_bldc_motor.angle_shaft + target_velocity*dt);
	_bldc_motor.velocity_shaft = target_velocity;

	_bldc_motor.target_current_q = _constrain(2, -BLDC_VOLTAGE_LIMIT, BLDC_VOLTAGE_LIMIT);
	_bldc_motor.angle_electrical = _bldc_motor.angle_shaft*BLDC_POLE_PAIRS;

	BLDC_Motor_Set_Phase_Voltage_FOC(_bldc_motor.target_current_q, 0, _bldc_motor.angle_electrical);

	_bldc_motor.openloop_t_prev = current_t;
}


void BLDC_Motor_Openloop_Position_Control(float target_position)
{
	unsigned long current_t = _micros();
	float dt = (float)(current_t - _bldc_motor.openloop_t_prev)*1e-6f;
	if(dt <= 0 || dt > 0.5f) dt = 1e-4f;

	if(fabs(target_position - _bldc_motor.angle_shaft) > fabs(BLDC_OPENLOOP_VELOCITY_LIMIT*dt))
	{
		_bldc_motor.angle_shaft += _sign(target_position - _bldc_motor.angle_shaft)*fabs(BLDC_OPENLOOP_VELOCITY_LIMIT)*dt;
		_bldc_motor.velocity_shaft = BLDC_OPENLOOP_VELOCITY_LIMIT;
	}
	else
	{
		_bldc_motor.angle_shaft = target_position;
		_bldc_motor.velocity_shaft = 0;
	}

	_bldc_motor.target_current_q = _constrain(1, -BLDC_VOLTAGE_LIMIT, BLDC_VOLTAGE_LIMIT);
	_bldc_motor.angle_electrical = _normalize_angle(_bldc_motor.angle_shaft)*BLDC_POLE_PAIRS;
	BLDC_Motor_Set_Phase_Voltage_FOC(_bldc_motor.target_current_q, 0, _bldc_motor.angle_electrical);
	_bldc_motor.openloop_t_prev = current_t;
}

void BLDC_Motor_Set_Phase_Voltage_FOC_v2(float uq, float ud, float elec_angle)
{
	float center;
	float sin_el;
	float cos_el;
	float u_alpha;
	float u_beta;

	sin_el = sin(elec_angle);
	cos_el = cos(elec_angle);

	u_alpha = cos_el*ud - sin_el*uq;
	u_beta = sin_el*ud + cos_el*uq;

	_bldc_motor.phase_volt_a = u_alpha;
	_bldc_motor.phase_volt_b = -0.5f * u_alpha + _SQRT3_2 * u_beta;
	_bldc_motor.phase_volt_c = -0.5f * u_alpha - _SQRT3_2 * u_beta;

	center = BLDC_VOLTAGE_LIMIT/2;

	float u_min = _min(_bldc_motor.phase_volt_a, _min(_bldc_motor.phase_volt_b, _bldc_motor.phase_volt_c));
	float u_max = _max(_bldc_motor.phase_volt_a, _max(_bldc_motor.phase_volt_b, _bldc_motor.phase_volt_c));
	center -= (u_min + u_max)/2;

	_bldc_motor.phase_volt_a += center;
	_bldc_motor.phase_volt_b += center;
	_bldc_motor.phase_volt_c += center;

	BLDC_Motor_Set_PWM(_bldc_motor.phase_volt_a, _bldc_motor.phase_volt_b, _bldc_motor.phase_volt_c);
}

void BLDC_Motor_Set_Phase_Voltage_FOC(float uq, float ud, float elec_angle)
{

	uint8_t sector;

	float u_out;

	u_out = _sqrtApprox(ud*ud + uq*uq) / BLDC_VOLTAGE_LIMIT;
	elec_angle = _normalize_angle(elec_angle + atan2(uq, ud));
	sector = floor(elec_angle/_PI_3) + 1;

	float T1 = _SQRT3*sin((float)sector*_PI_3 - elec_angle)*u_out;
	float T2 = _SQRT3*sin(elec_angle - ((float)sector - 1.0f)*_PI_3)*u_out;

	float T0 = 0;
	T0 = 1 - T1 - T2; 	// if modulation centered

	float Ta, Tb, Tc;

	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;

		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;

		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;

		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;

		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;

		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;

		default:
		// possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;

	}

	_bldc_motor.phase_volt_a = Ta*BLDC_VOLTAGE_LIMIT;
	_bldc_motor.phase_volt_b = Tb*BLDC_VOLTAGE_LIMIT;
	_bldc_motor.phase_volt_c = Tc*BLDC_VOLTAGE_LIMIT;

	BLDC_Motor_Set_PWM(_bldc_motor.phase_volt_a, _bldc_motor.phase_volt_b, _bldc_motor.phase_volt_c);
}


void BLDC_Motor_Set_Phase_Voltage_Trap_150(float uq, float ud, float elec_angle)
{
	float center;
	int sector;

	float Ua, Ub, Uc;
	sector = 12 * (_normalize_angle(elec_angle + _PI_6) / _2PI);
	// modulation centered 	=> voltage limit /2
	// center = modulation_centered ? (voltage limit)/2 : uq;

	center = BLDC_VOLTAGE_LIMIT/2;

	if(trap_150_map[sector][0] == 0)
	{
		Ua = center;
		Ub = trap_150_map[sector][1] * uq + center;
		Uc = trap_150_map[sector][2] * uq + center;
	}
	else if(trap_150_map[sector][1] == 0)
	{
		Ua = trap_150_map[sector][0] * uq + center;
		Ub = center;
		Uc = trap_150_map[sector][2] * uq + center;
	}
	else if(trap_150_map[sector][3] == 0)
	{
		Ua = trap_150_map[sector][0] * uq + center;
		Ub = trap_150_map[sector][1] * uq + center;
		Uc = center;
	}
	else
	{
		Ua = trap_150_map[sector][0] * uq + center;
		Ub = trap_150_map[sector][1] * uq + center;
		Uc = trap_150_map[sector][2] * uq + center;
	}


	_bldc_motor.phase_volt_a = Ua;
	_bldc_motor.phase_volt_b = Ub;
	_bldc_motor.phase_volt_c = Uc;

	BLDC_Motor_Set_PWM(_bldc_motor.phase_volt_a, _bldc_motor.phase_volt_b, _bldc_motor.phase_volt_c);
}



void BLDC_Motor_Set_PWM(float ua, float ub, float uc)
{
	ua = _constrain(ua, 0.0f, BLDC_VOLTAGE_LIMIT);
	ub = _constrain(ub, 0.0f, BLDC_VOLTAGE_LIMIT);
	uc = _constrain(uc, 0.0f, BLDC_VOLTAGE_LIMIT);

	ua = _constrain(ua/BLDC_VOLTAGE_SUPPLY, 0.0f, 1.0f);
	ub = _constrain(ub/BLDC_VOLTAGE_SUPPLY, 0.0f, 1.0f);
	uc = _constrain(uc/BLDC_VOLTAGE_SUPPLY, 0.0f, 1.0f);


	_bldc_motor.htim->Instance->CCR1 = (uint16_t)(ua*BLDC_PWM_PERIOD);
	_bldc_motor.htim->Instance->CCR2 = (uint16_t)(ub*BLDC_PWM_PERIOD);
	_bldc_motor.htim->Instance->CCR3 = (uint16_t)(uc*BLDC_PWM_PERIOD);
}



void BLDC_PID_Init()
{
	_bldc_pid_current_dc.t_prev = 0;
	_bldc_pid_current_dc.error_prev = 0;
	_bldc_pid_current_dc.integral_prev = 0;
	_bldc_pid_current_dc.out_prev = 0;

	_bldc_pid_current_d.t_prev = 0;
	_bldc_pid_current_d.error_prev = 0;
	_bldc_pid_current_d.integral_prev = 0;
	_bldc_pid_current_d.out_prev = 0;

	_bldc_pid_current_q.t_prev = 0;
	_bldc_pid_current_q.error_prev = 0;
	_bldc_pid_current_q.integral_prev = 0;
	_bldc_pid_current_q.out_prev = 0;

	_bldc_pid_velocity.t_prev = 0;
	_bldc_pid_velocity.error_prev = 0;
	_bldc_pid_velocity.integral_prev = 0;
	_bldc_pid_velocity.out_prev = 0;

	_bldc_pid_position.t_prev = 0;
	_bldc_pid_position.error_prev = 0;
	_bldc_pid_position.integral_prev = 0;
	_bldc_pid_position.out_prev = 0;

}

void BLDC_PID_Current_DC()
{
	unsigned long current_t = _micros();
	float dt = (float)(current_t - _bldc_pid_current_dc.t_prev)*1e-6f;
	if(dt <= 0 || dt > 0.5f) dt = 1e-4f;

	float error = _bldc_motor.target_current_dc - _lpf_current_dc.filter_out_prev;

	float p_out = P_CURRENT_DC*error;
	float i_out = _bldc_pid_current_dc.integral_prev + I_CURRENT_DC*dt*0.5f*(_bldc_pid_current_dc.error_prev + error);
	i_out = _constrain(i_out, -PID_CURRENT_DC_LIMIT, PID_CURRENT_DC_LIMIT);
	float d_out = D_CURRENT_DC*(error - _bldc_pid_current_dc.error_prev)/dt;

	float pid_out = p_out + i_out + d_out;
	pid_out = _constrain(pid_out, -PID_CURRENT_DC_LIMIT, PID_CURRENT_DC_LIMIT);

	float pid_ramp = (pid_out - _bldc_pid_current_dc.out_prev)/dt;
	if(pid_ramp > PID_CURRENT_DC_RAMP)
	{
		pid_out = _bldc_pid_current_dc.out_prev + PID_CURRENT_DC_RAMP*dt;
	}
	else if(pid_ramp < -PID_CURRENT_DC_RAMP)
	{
		pid_out = _bldc_pid_current_dc.out_prev - PID_CURRENT_DC_RAMP*dt;
	}

	_bldc_pid_current_dc.t_prev = current_t;
	_bldc_pid_current_dc.error_prev = error;
	_bldc_pid_current_dc.integral_prev = i_out;
	_bldc_pid_current_dc.out_prev = pid_out;
}

void BLDC_PID_Current_D()
{
	unsigned long current_t = _micros();
	float dt = (float)(current_t - _bldc_pid_current_d.t_prev)*1e-6f;
	if(dt <= 0 || dt > 0.5f) dt = 4e-4f;

	float error = _bldc_motor.target_current_d - _lpf_current_d.filter_out_prev;

	float p_out = P_CURRENT_D*error;
	float i_out = _bldc_pid_current_d.integral_prev + I_CURRENT_D*dt*0.5f*(_bldc_pid_current_d.error_prev + error);
	i_out = _constrain(i_out, -PID_CURRENT_D_LIMIT, PID_CURRENT_D_LIMIT);
	float d_out = D_CURRENT_D*(error - _bldc_pid_current_d.error_prev)/dt;

	float pid_out = p_out + i_out + d_out;
	pid_out = _constrain(pid_out, -PID_CURRENT_D_LIMIT, PID_CURRENT_D_LIMIT);

	float pid_ramp = (pid_out - _bldc_pid_current_d.out_prev)/dt;
	if(pid_ramp > PID_CURRENT_D_RAMP)
	{
		pid_out = _bldc_pid_current_d.out_prev + PID_CURRENT_D_RAMP*dt;
	}
	else if(pid_ramp < -PID_CURRENT_D_RAMP)
	{
		pid_out = _bldc_pid_current_d.out_prev - PID_CURRENT_D_RAMP*dt;
	}

	_bldc_pid_current_d.t_prev = current_t;
	_bldc_pid_current_d.error_prev = error;
	_bldc_pid_current_d.integral_prev = i_out;
	_bldc_pid_current_d.out_prev = pid_out;
}

void BLDC_PID_Current_Q()
{
	unsigned long current_t = _micros();
	float dt = (float)(current_t - _bldc_pid_current_q.t_prev)*1e-6f;
	if(dt <= 0 || dt > 0.5f) dt = 4e-4f;

	float error = _bldc_motor.target_current_q - _lpf_current_q.filter_out_prev;

	float p_out = P_CURRENT_Q*error;
	float i_out = _bldc_pid_current_q.integral_prev + I_CURRENT_Q*dt*0.5f*(_bldc_pid_current_q.error_prev + error);
	i_out = _constrain(i_out, -PID_CURRENT_Q_LIMIT, PID_CURRENT_Q_LIMIT);
	float d_out = D_CURRENT_Q*(error - _bldc_pid_current_q.error_prev)/dt;

	float pid_out = p_out + i_out + d_out;
	pid_out = _constrain(pid_out, -PID_CURRENT_Q_LIMIT, PID_CURRENT_Q_LIMIT);

	float pid_ramp = (pid_out - _bldc_pid_current_q.out_prev)/dt;
	if(pid_ramp > PID_CURRENT_Q_RAMP)
	{
		pid_out = _bldc_pid_current_q.out_prev + PID_CURRENT_Q_RAMP*dt;
	}
	else if(pid_ramp < -PID_CURRENT_Q_RAMP)
	{
		pid_out = _bldc_pid_current_q.out_prev - PID_CURRENT_Q_RAMP*dt;
	}

	_bldc_pid_current_q.t_prev = current_t;
	_bldc_pid_current_q.error_prev = error;
	_bldc_pid_current_q.integral_prev = i_out;
	_bldc_pid_current_q.out_prev = pid_out;
}

void BLDC_PID_Velocity()
{
	unsigned long current_t = _micros();
	float dt = (float)(current_t - _bldc_pid_velocity.t_prev)*1e-6f;
	if(dt <= 0 || dt > 0.5f) dt = 2e-3f;

	float error = _bldc_motor.target_velocity - _lpf_velocity.filter_out_prev;

	float p_out = P_BLDC_VELOCITY*error;
	float i_out = _bldc_pid_velocity.integral_prev + I_BLDC_VELOCITY*dt*0.5f*(_bldc_pid_velocity.error_prev + error);
	i_out = _constrain(i_out, -PID_BLDC_VELOCITY_LIMIT, PID_BLDC_VELOCITY_LIMIT);
	float d_out = D_BLDC_VELOCITY*(error - _bldc_pid_velocity.error_prev)/dt;

	float pid_out = p_out + i_out + d_out;
	pid_out = _constrain(pid_out, -PID_BLDC_VELOCITY_LIMIT, PID_BLDC_VELOCITY_LIMIT);

	float pid_ramp = (pid_out - _bldc_pid_velocity.out_prev)/dt;
	if(pid_ramp > PID_BLDC_VELOCITY_RAMP)
	{
		pid_out = _bldc_pid_velocity.out_prev + PID_BLDC_VELOCITY_RAMP*dt;
	}
	else if(pid_ramp < -PID_BLDC_VELOCITY_RAMP)
	{
		pid_out = _bldc_pid_velocity.out_prev - PID_BLDC_VELOCITY_RAMP*dt;
	}

	_bldc_pid_velocity.t_prev = current_t;
	_bldc_pid_velocity.error_prev = error;
	_bldc_pid_velocity.integral_prev = i_out;
	_bldc_pid_velocity.out_prev = pid_out;
}

void BLDC_PID_Position()
{
	unsigned long current_t = _micros();
	float dt = (float)(current_t - _bldc_pid_position.t_prev)*1e-6f;
	if(dt <= 0 || dt > 0.5f) dt = 2e-3f;

	float error = _bldc_motor.target_angle - _lpf_angle.filter_out_prev;

	float p_out = P_BLDC_POSITION*error;
	float i_out = _bldc_pid_position.integral_prev + I_BLDC_POSITION*dt*0.5f*(_bldc_pid_position.error_prev + error);
	i_out = _constrain(i_out, -PID_BLDC_POSITION_LIMIT, PID_BLDC_POSITION_LIMIT);
	float d_out = D_BLDC_POSITION*(error - _bldc_pid_position.error_prev)/dt;

	float pid_out = p_out + i_out + d_out;
	pid_out = _constrain(pid_out, -PID_BLDC_POSITION_LIMIT, PID_BLDC_POSITION_LIMIT);

	float pid_ramp = (pid_out - _bldc_pid_position.out_prev)/dt;
	if(pid_ramp > PID_BLDC_POSITION_RAMP)
	{
		pid_out = _bldc_pid_position.out_prev + PID_BLDC_POSITION_RAMP*dt;
	}
	else if(pid_ramp < -PID_BLDC_POSITION_RAMP)
	{
		pid_out = _bldc_pid_position.out_prev - PID_BLDC_POSITION_RAMP*dt;
	}

	_bldc_pid_position.t_prev = current_t;
	_bldc_pid_position.error_prev = error;
	_bldc_pid_position.integral_prev = i_out;
	_bldc_pid_position.out_prev = pid_out;
}
