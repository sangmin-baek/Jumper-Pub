/*
 * low_pass_fliter_bsm.h
 *
 *  Created on: 2024. 3. 23.
 *      Author: BSM
 */

#ifndef INC_LOW_PASS_FLITER_BSM_H_
#define INC_LOW_PASS_FLITER_BSM_H_

#include "math_utils_bsm.h"
#include "drv8311_current_sensor_bsm.h"
#include "as5047p_mag_position_spi_bsm.h"


/*-----------------------------------------
 * CUSTOM SETTINGS-------------------------
 * ---------------------------------------*/
#define LPF_SAMPLING_Fs		10000.0f
#define LPF_CURRENT_BW_D_Fc	2300.0f
#define LPF_CURRENT_BW_Q_Fc	2300.0f
#define LPF_CURRENT_DC_Fc	100.0f
#define LPF_CURRENT_D_Fc	50.0f
#define LPF_CURRENT_Q_Fc	50.0f
#define LPF_ANGLE_Fc		10000.0f
#define LPF_VELOCITY_Fc		100.0f

typedef struct{

	double d1_1;
	double d2_1;

	double d1_2;
	double d2_2;

	double n0_1;
	double n1_1;
	double n2_1;

	double n0_2;
	double n1_2;
	double n2_2;

	double y1[3];
	double x1[3];

	double y2[3];
	double x2[3];

	double filter_out;
}_LPF_BW;

extern _LPF_BW _lpf_bw_current_d;
extern _LPF_BW _lpf_bw_current_q;

typedef struct{
	unsigned long filter_t_prev;
	float filter_out_prev;
}_LPF;

extern _LPF _lpf_current_dc;
extern _LPF _lpf_current_d;
extern _LPF _lpf_current_q;
extern _LPF _lpf_angle;
extern _LPF _lpf_velocity;

void LPF_BW_Current_Q_4th();
void LPF_BW_Current_D_4th();
void LPF_Current_DC();
void LPF_Current_D();
void LPF_Current_Q();
void LPF_ANGLE();
void LPF_VELOCITY();

void LPF_Init();
void LPF_BW_Init();

#endif /* INC_LOW_PASS_FLITER_BSM_H_ */
