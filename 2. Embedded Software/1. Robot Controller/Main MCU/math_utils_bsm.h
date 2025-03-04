/*
 * math_utils_bsm.h
 *
 *  Created on: 2023. 12. 22.
 *      Author: BSM
 */

#ifndef INC_MATH_UTILS_BSM_H_
#define INC_MATH_UTILS_BSM_H_


#include "main.h"
#include "math.h"
#include "stm32h7xx_hal.h"



#define _2_SQRT3 		1.15470053838f
#define _SQRT3 			1.73205080757f
#define _1_SQRT3 		0.57735026919f
#define _SQRT3_2 		0.86602540378f
#define _SQRT2 			1.41421356237f
#define _120_D2R 		2.09439510239f
#define _PI 			3.14159265359f
#define _PI_2 			1.57079632679f
#define _PI_3 			1.0471975512f
#define _2PI 			6.28318530718f
#define _3PI_2 			4.71238898038f
#define _PI_6 			0.52359877559f
#define _RPM_TO_RADS 	0.10471975512f
#define DEG_TO_RAD		(0.01745329252f)
#define RAD_TO_DEG		(57.29577951f)

#define _constrain(amt,low,high) 	((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sign(a) 					( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )

extern TIM_HandleTypeDef htim5;

float _sin(float a);
float _cos(float a);
float _inv_sqrtApprox_low_precision(float number);
float _sqrtApprox_low_precision(float number);
float _inv_sqrtApprox(float number);
float _sqrtApprox(float number);
float _normalize_angle(float angle);

unsigned long _micros();
void _micros_delay(uint16_t micros);

#endif /* INC_MATH_UTILS_BSM_H_ */
