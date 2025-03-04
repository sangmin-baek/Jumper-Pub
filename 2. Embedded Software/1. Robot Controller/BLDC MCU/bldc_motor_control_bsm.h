/*
 * bldc_motor_control_bsm.h
 *
 *  Created on: 2024. 3. 24.
 *      Author: BSM
 */

#ifndef INC_BLDC_MOTOR_CONTROL_BSM_H_
#define INC_BLDC_MOTOR_CONTROL_BSM_H_

#include <bldc_motor_communication_driver.h>
#include "math_utils_bsm.h"
#include "drv8311_current_sensor_bsm.h"
#include "as5047p_mag_position_spi_bsm.h"
#include "low_pass_fliter_bsm.h"


/*-----------------------------------------
 * Peripheral settings---------------------
 * ---------------------------------------*/
#define BLDC_PWM_FREQ			55000
#define BLDC_PWM_PERIOD			499

#define BLDC_CNTR_FREQ			20000
#define BLDC_TIM_TAKEOFF		1.0

/*-----------------------------------------
 * BLDC Motor Parameters-------------------
 * ---------------------------------------*/
#define BLDC_PHASE_RESISTANCE	0.19f
#define BLDC_PHASE_INDUCTANCE	0*1e-6f
#define BLDC_POLE_PAIRS			7
#define BLDC_GEAR_RATIO			125

#define BLDC_MOTOR_DIRECTION	1	// CW
#define BLDC_SENSOR_DIRECTION	1
#define BLDC_GEAR_DIRECTION		1


/*-----------------------------------------
 * POWER Input and Limits------------------
 * ---------------------------------------*/
#define BLDC_VOLTAGE_SUPPLY		12.0f
#define BLDC_VOLTAGE_LIMIT		12.0f
#define BLDC_CURRENT_LIMIT		1.0f
#define BLDC_VELOCITY_LIMIT		1e6f
#define BLDC_OPENLOOP_VELOCITY_LIMIT	20.0f
#define BLDC_VOLTAGE_ALIGN		1.50f
#define BLDC_VELOCITY_ALIGN		0.50f


/*-----------------------------------------
 * Control Parameters----------------------
 * ---------------------------------------*/
#define BLDC_CURRENT_LOOP_FREQ		2500
#define BLDC_VELOCITY_LOOP_FREQ		500
#define BLDC_POSITION_LOOP_FREQ		100
#define BLDC_CURRENT_LOOP_PERIOD	1
#define BLDC_VELOCITY_LOOP_PERIOD	1
#define BLDC_POSITION_LOOP_PERIOD	1


#define P_CURRENT_DC		3.032f
#define I_CURRENT_DC		0.0f
#define D_CURRENT_DC		0.0f

#define P_CURRENT_D			0.042f	//0.103f	//0.0035f	//0.0232f	//0.0204f	//0.042f	//0.0335f
#define I_CURRENT_D			406.0f	//1028.0f	//44.0f	//324.0f	//184.0f	//406.0f	//356.0f
#define D_CURRENT_D			0.0f

#define P_CURRENT_Q			0.042f	//0.103f	//0.0035f	//0.0232f	//0.0204f	//0.042f	//0.0335f
#define I_CURRENT_Q			406.0f	//1028.0f	//44.0f	//324.0f	//184.0f	//406.0f	//356.0f
#define D_CURRENT_Q			0.0f

#define P_BLDC_VELOCITY		0.01f
#define I_BLDC_VELOCITY		0.1f
#define D_BLDC_VELOCITY		0.0f

#define P_BLDC_POSITION		12.0f
#define I_BLDC_POSITION		0.0f
#define D_BLDC_POSITION		1.02f
#define BLDC_FEED_FORWARD_VELOVITY	0.0f

#define PID_CURRENT_DC_LIMIT			BLDC_VOLTAGE_LIMIT
#define PID_CURRENT_D_LIMIT			BLDC_VOLTAGE_LIMIT
#define PID_CURRENT_Q_LIMIT			BLDC_VOLTAGE_LIMIT
#define PID_BLDC_VELOCITY_LIMIT		BLDC_VOLTAGE_LIMIT
#define PID_BLDC_POSITION_LIMIT		BLDC_VELOCITY_LIMIT

#define PID_CURRENT_DC_RAMP			1e10f
#define PID_CURRENT_D_RAMP			1e6f
#define PID_CURRENT_Q_RAMP			1e6f
#define PID_BLDC_VELOCITY_RAMP		1e6f
#define PID_BLDC_POSITION_RAMP		1e6f



/*-----------------------------------------
 * BLDC_CMD related Macros-----------------
 * ---------------------------------------*/
#define DIP_AUTO_SINGLE_JUMPING		0b0010
#define DIP_MANUAL_POT				0b0000
#define DIP_uSD_CLOSE				0b1001



typedef struct{
	TIM_HandleTypeDef *htim;
	float angle_geared;
	float velocity_geared;

	float angle_shaft;
	float velocity_shaft;

	float angle_electrical;
	float zero_elec_angle;

	float target_angle;
	float target_velocity;
	float target_current_dc;
	float target_current_d;
	float target_current_q;

	float phase_volt_a;
	float phase_volt_b;
	float phase_volt_c;

	uint16_t ctrl_flag_current;
	uint16_t ctrl_flag_velocity;
	uint16_t ctrl_flag_position;

	unsigned long openloop_t_prev;
}_BLDC_MOTOR;


typedef struct{
	unsigned long t_prev;
	float error_prev;
	float integral_prev;
	float out_prev;
}_BLDC_PID;



extern _BLDC_MOTOR _bldc_motor;

extern _BLDC_PID _bldc_pid_current_dc;
extern _BLDC_PID _bldc_pid_current_d;
extern _BLDC_PID _bldc_pid_current_q;
extern _BLDC_PID _bldc_pid_velocity;
extern _BLDC_PID _bldc_pid_position;


extern volatile uint32_t motor_tim_cnt;
extern uint8_t motor_run_flag;
extern uint8_t motor_position_e_storage_flag;
extern uint8_t motor_position_init_flag;
extern float motor_current_geared_angle;

void BLDC_Motor_Init(TIM_HandleTypeDef *htim);


void BLDC_Motor_Get_States();
void BLDC_Motor_Get_Geared_Angle();
void BLDC_Motor_Get_Geared_Velocity();
void BLDC_Motor_Get_Shaft_Angle();
void BLDC_Motor_Get_Shaft_Velocity();
void BLDC_Motor_Get_Elec_Angle();



void BLDC_Motor_Control_Init();
void BLDC_Motor_Control_Init_Trapz();

void BLDC_Motor_Controller(uint32_t tim_cnt);
void BLDC_Motor_Cascade_Current_Control(float target_current);
void BLDC_Motor_Cascade_Velocity_Control(float target_velocity);
void BLDC_Motor_Cascade_Position_Control(float target_position);

void BLDC_Motor_Current_Control(float target_current);
void BLDC_Motor_Velocity_Control(float target_velocity);
void BLDC_Motor_Position_Control(float target_position);
void BLDC_Motor_Openloop_Velocity_Control(float target_velocity);
void BLDC_Motor_Openloop_Position_Control(float target_position);

void BLDC_Motor_Set_Phase_Voltage_FOC_v2(float uq, float ud, float elec_angle);
void BLDC_Motor_Set_Phase_Voltage_FOC(float uq, float ud, float elec_angle);
void BLDC_Motor_Set_Phase_Voltage_Trap_150(float uq, float ud, float elec_angle);
void BLDC_Motor_Set_PWM(float ua, float ub, float uc);


void BLDC_PID_Init();
void BLDC_PID_Current_DC();
void BLDC_PID_Current_D();
void BLDC_PID_Current_Q();
void BLDC_PID_Velocity();
void BLDC_PID_Position();

#endif /* INC_BLDC_MOTOR_CONTROL_BSM_H_ */
