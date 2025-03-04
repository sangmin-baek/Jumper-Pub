/*
 * drv8311_current_sensor_bsm.c
 *
 *  Created on: 2024. 3. 23.
 *      Author: BSM
 */

#include <drv8311_current_sensor_bsm.h>

_DRV8311_CURRENT _drv8311_current;

void DRV8311_Current_Sensor_Get_DC_Current(float angle_electrical)
{
	DRV8311_Current_Sensor_Get_Phase_Current();

	float sign = 1;
	float i_alpha, i_beta;

	i_alpha = _drv8311_current.i_phase_a;
	i_beta = _1_SQRT3*_drv8311_current.i_phase_a + _2_SQRT3*_drv8311_current.i_phase_b;

	float ct, st;
	ct = cos(angle_electrical);
	st = sin(angle_electrical);

	sign = (i_beta*ct - i_alpha*st) > 0 ? 1 : -1;

	_drv8311_current.i_dc = sign*_sqrtApprox(i_alpha*i_alpha + i_beta*i_beta);
}

void DRV8311_Current_Sensor_Get_DQ_Current(float angle_electrical)
{
	DRV8311_Current_Sensor_Get_Phase_Current();

	float i_alpha, i_beta;

	i_alpha = _drv8311_current.i_phase_a;
	i_beta = _1_SQRT3*_drv8311_current.i_phase_a + _2_SQRT3*_drv8311_current.i_phase_b;

	float ct, st;
	//ct = _cos(angle_electrical);
	//st = _sin(angle_electrical);

	ct = cos(angle_electrical);
	st = sin(angle_electrical);

	_drv8311_current.i_d = i_alpha*ct + i_beta*st;
	_drv8311_current.i_q = i_beta*ct - i_alpha*st;
}

void DRV8311_Current_Sensor_Get_Phase_Current()
{
	float i_a_sensed, i_b_sensed;
	i_a_sensed = ((float)_drv8311_current.current_adc_raw[0] - _drv8311_current.i_phase_a_offset) * CURRENT_ADC_BIT_TO_VOLT * CURRENT_GAIN;
	i_b_sensed = ((float)_drv8311_current.current_adc_raw[1] - _drv8311_current.i_phase_b_offset) * CURRENT_ADC_BIT_TO_VOLT * CURRENT_GAIN;
	_drv8311_current.i_phase_a = 1.004346*i_a_sensed - 0.000199*i_b_sensed;
	_drv8311_current.i_phase_b = 0.022060*i_a_sensed + 1.020405*i_b_sensed;
	/*
	_drv8311_current.i_phase_a = ((float)_drv8311_current.current_adc_raw[0] - _drv8311_current.i_phase_a_offset) * CURRENT_ADC_BIT_TO_VOLT * CURRENT_GAIN;
	_drv8311_current.i_phase_b = ((float)_drv8311_current.current_adc_raw[1] - _drv8311_current.i_phase_b_offset) * CURRENT_ADC_BIT_TO_VOLT * CURRENT_GAIN;
	*/
}


void DRV8311_Current_Sensor_Calib_Offset()
{
	uint16_t cnt = 1000;
	uint32_t a_offset = 0;
	uint32_t b_offset = 0;

	for(uint16_t i = 0; i < cnt; i++)
	{
		a_offset += _drv8311_current.current_adc_raw[0];
		b_offset += _drv8311_current.current_adc_raw[1];
		HAL_Delay(1);
	}

	_drv8311_current.i_phase_a_offset = ((float)a_offset)/cnt;
	_drv8311_current.i_phase_b_offset = ((float)b_offset)/cnt;
}


void DRV8311_Current_Sensor_Init(ADC_HandleTypeDef *hadc)
{
	_drv8311_current.hadc = hadc;

	_drv8311_current.i_phase_a = 0;
	_drv8311_current.i_phase_b = 0;

	_drv8311_current.i_phase_a_offset = 0;
	_drv8311_current.i_phase_b_offset = 0;

	_drv8311_current.i_d = 0;
	_drv8311_current.i_q = 0;

	_drv8311_current.i_dc = 0;

	HAL_ADC_Start_DMA(hadc, (uint32_t*)_drv8311_current.current_adc_raw, 2);
	DRV8311_Current_Sensor_Calib_Offset();
}
