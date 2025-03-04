/*
 * math_utils_bsm.c
 *
 *  Created on: 2023. 12. 22.
 *      Author: BSM
 */

#include "math_utils_bsm.h"

float _inv_sqrtApprox_low_precision(float number)
{
	const float f = 1.5F;
	float x = number * 0.5F;
	float y = number;
	long i = * ( long * ) &y;
	i = 0x5f375a86 - ( i >> 1 );
	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) ); // better precision
	//y = y * ( f - ( x * y * y ) );
	return y;
}

float _sqrtApprox_low_precision(float number)
{
	float x;
	const float f = 1.5F; // better precision

	x = number * 0.5F;
	float y = number;
	long i = * ( long * ) &y;
	i = 0x5f375a86 - ( i >> 1 );
	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) ); // better precision
	//y = y * ( f - ( x * y * y ) );
	return number * y;
}

float _inv_sqrtApprox(float number)
{
	const float f = 1.5F;
	float x = number * 0.5F;
	float y = number;
	long i = * ( long * ) &y;
	i = 0x5f375a86 - ( i >> 1 );
	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) ); // better precision
	y = y * ( f - ( x * y * y ) );
	return y;
}

float _sqrtApprox(float number) // https://kipl.tistory.com/236
{//low in fat
   float x;
   const float f = 1.5F; // better precision

   x = number * 0.5F;
  float y = number;
  long i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) ); // better precision
  y = y * ( f - ( x * y * y ) );
  return number * y;
}


float _sin(float a){
  // 16bit integer array for sine lookup. interpolation is used for better precision
  // 16 bit precision on sine value, 8 bit fractional value for interpolation, 6bit LUT size
  // resulting precision compared to stdlib sine is 0.00006480 (RMS difference in range -PI,PI for 3217 steps)
  static uint16_t sine_array[65] = {0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,
  11793,12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,23170,
  23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,30274,30572,30853,
  31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,32768};
  unsigned int i = (unsigned int)(a * (64*4*256.0 /_2PI));
  int t1, t2, frac = i & 0xff;
  i = (i >> 8) & 0xff;
  if (i < 64) {
    t1 = sine_array[i]; t2 = sine_array[i+1];
  }
  else if(i < 128) {
    t1 = sine_array[128 - i]; t2 = sine_array[127 - i];
  }
  else if(i < 192) {
    t1 = -sine_array[-128 + i]; t2 = -sine_array[-127 + i];
  }
  else {
    t1 = -sine_array[256 - i]; t2 = -sine_array[255 - i];
  }
  return (1.0f/32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
}

float _cos(float a)
{
  float a_sin = a + _PI_2;
  a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
  return _sin(a_sin);
}

float _normalize_angle(float angle)
{
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}


unsigned long _micros()
{
	return htim5.Instance->CNT;
}

void _micros_delay(uint16_t micros)
{
	unsigned long current_micros = _micros();

	while(_micros() - current_micros < micros)
	{

	}
}
