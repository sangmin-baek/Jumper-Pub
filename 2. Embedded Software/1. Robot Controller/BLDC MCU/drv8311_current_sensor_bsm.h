/*
 * drv8311_current_sensor_bsm.h
 *
 *  Created on: 2024. 3. 23.
 *      Author: BSM
 */

#ifndef INC_DRV8311_CURRENT_SENSOR_BSM_H_
#define INC_DRV8311_CURRENT_SENSOR_BSM_H_

#include "stm32h7xx_hal.h"

#include "math_utils_bsm.h"


/*-----------------------------------------
 * CUSTOM SETTINGS-------------------------
 * ---------------------------------------*/
#define CURRENT_ADC_BIT_TO_VOLT		0.0128906	// 8 Bit => 3.3 Volts
#define CURRENT_GAIN				2.0f			// 2 A/V for 47k // 4 A/V for 0k


typedef struct{
	ADC_HandleTypeDef *hadc;
	uint16_t current_adc_raw[2];

	float i_phase_a;
	float i_phase_b;

	float i_phase_a_offset;
	float i_phase_b_offset;

	float i_d;
	float i_q;

	float i_dc;
}_DRV8311_CURRENT;

extern _DRV8311_CURRENT _drv8311_current;

void DRV8311_Current_Sensor_Get_DC_Current(float angle_electrical);
void DRV8311_Current_Sensor_Get_DQ_Current(float angle_electrical);
void DRV8311_Current_Sensor_Get_Phase_Current();
void DRV8311_Current_Sensor_Init(ADC_HandleTypeDef *hadc);
void DRV8311_Current_Sensor_Calib_Offset();

#endif /* INC_DRV8311_CURRENT_SENSOR_BSM_H_ */
