/*
 * low_pass_fliter_bsm.c
 *
 *  Created on: 2024. 3. 23.
 *      Author: BSM
 */


#include "low_pass_fliter_bsm.h"

_LPF_BW _lpf_bw_current_d;
_LPF_BW _lpf_bw_current_q;

_LPF _lpf_current_dc;
_LPF _lpf_current_d;
_LPF _lpf_current_q;
_LPF _lpf_angle;
_LPF _lpf_velocity;

void LPF_Init()
{
	_lpf_current_dc.filter_out_prev = 0;
	_lpf_current_dc.filter_t_prev = 0;


	_lpf_current_d.filter_out_prev = 0;
	_lpf_current_d.filter_t_prev = 0;

	_lpf_current_q.filter_out_prev = 0;
	_lpf_current_q.filter_t_prev = 0;

	_lpf_angle.filter_out_prev = 0;
	_lpf_angle.filter_t_prev = 0;

	_lpf_velocity.filter_out_prev = 0;
	_lpf_velocity.filter_t_prev = 0;


}

void LPF_BW_Init()
{
	double freq_norm = LPF_CURRENT_BW_D_Fc/LPF_SAMPLING_Fs;
	double gamma = 1/tan(_PI*freq_norm);
	double alpha_1 = 2*cos(_PI_2*5/4);
	double alpha_2 = 2*cos(_PI_2*7/4);
	double d0_1 = gamma*gamma - alpha_1*gamma + 1;
	double d0_2 = gamma*gamma - alpha_2*gamma + 1;


	_lpf_bw_current_d.n0_1 = 1/d0_1;
	_lpf_bw_current_d.n1_1 = 2/d0_1;
	_lpf_bw_current_d.n2_1 = 1/d0_1;

	_lpf_bw_current_d.d1_1 = 2*(gamma*gamma - 1)/d0_1;
	_lpf_bw_current_d.d2_1 = (gamma*gamma + alpha_1*gamma + 1)/d0_1;

	_lpf_bw_current_d.n0_2 = 1/d0_2;
	_lpf_bw_current_d.n1_2 = 2/d0_2;
	_lpf_bw_current_d.n2_2 = 1/d0_2;

	_lpf_bw_current_d.d1_2 = 2*(gamma*gamma - 1)/d0_2;
	_lpf_bw_current_d.d2_2 = (gamma*gamma + alpha_2*gamma + 1)/d0_2;

	_lpf_bw_current_d.x1[0] = 0;
	_lpf_bw_current_d.x1[1] = 0;
	_lpf_bw_current_d.x1[2] = 0;

	_lpf_bw_current_d.y1[0] = 0;
	_lpf_bw_current_d.y1[1] = 0;
	_lpf_bw_current_d.y1[2] = 0;

	_lpf_bw_current_d.x2[0] = 0;
	_lpf_bw_current_d.x2[1] = 0;
	_lpf_bw_current_d.x2[2] = 0;

	_lpf_bw_current_d.y2[0] = 0;
	_lpf_bw_current_d.y2[1] = 0;
	_lpf_bw_current_d.y2[2] = 0;

	_lpf_bw_current_d.filter_out = 0;



	 freq_norm = LPF_CURRENT_BW_Q_Fc/LPF_SAMPLING_Fs;
	 gamma = 1/tan(_PI*freq_norm);
	 alpha_1 = 2*cos(_PI_2*5/4);
	 alpha_2 = 2*cos(_PI_2*7/4);
	 d0_1 = gamma*gamma - alpha_1*gamma + 1;
	 d0_2 = gamma*gamma - alpha_2*gamma + 1;


	_lpf_bw_current_q.n0_1 = 1/d0_1;
	_lpf_bw_current_q.n1_1 = 2/d0_1;
	_lpf_bw_current_q.n2_1 = 1/d0_1;

	_lpf_bw_current_q.d1_1 = 2*(gamma*gamma - 1)/d0_1;
	_lpf_bw_current_q.d2_1 = (gamma*gamma + alpha_1*gamma + 1)/d0_1;

	_lpf_bw_current_q.n0_2 = 1/d0_2;
	_lpf_bw_current_q.n1_2 = 2/d0_2;
	_lpf_bw_current_q.n2_2 = 1/d0_2;

	_lpf_bw_current_q.d1_2 = 2*(gamma*gamma - 1)/d0_2;
	_lpf_bw_current_q.d2_2 = (gamma*gamma + alpha_2*gamma + 1)/d0_2;

	_lpf_bw_current_q.x1[0] = 0;
	_lpf_bw_current_q.x1[1] = 0;
	_lpf_bw_current_q.x1[2] = 0;

	_lpf_bw_current_q.y1[0] = 0;
	_lpf_bw_current_q.y1[1] = 0;
	_lpf_bw_current_q.y1[2] = 0;

	_lpf_bw_current_q.x2[0] = 0;
	_lpf_bw_current_q.x2[1] = 0;
	_lpf_bw_current_q.x2[2] = 0;

	_lpf_bw_current_q.y2[0] = 0;
	_lpf_bw_current_q.y2[1] = 0;
	_lpf_bw_current_q.y2[2] = 0;

	_lpf_bw_current_q.filter_out = 0;
}


void LPF_BW_Current_Q_4th()
{
	// first loop of 2nd order butterworth filter
	_lpf_bw_current_q.x1[0] = _drv8311_current.i_q;
	_lpf_bw_current_q.y1[0] = (_lpf_bw_current_q.n0_1*_lpf_bw_current_q.x1[0]
							+ _lpf_bw_current_q.n1_1*_lpf_bw_current_q.x1[1]
							+ _lpf_bw_current_q.n2_1*_lpf_bw_current_q.x1[2]
			                - _lpf_bw_current_q.d1_1*_lpf_bw_current_q.y1[1]
						    - _lpf_bw_current_q.d2_1*_lpf_bw_current_q.y1[2]);

	_lpf_bw_current_q.x1[2] = _lpf_bw_current_q.x1[1];
	_lpf_bw_current_q.x1[1] = _lpf_bw_current_q.x1[0];


	_lpf_bw_current_q.y1[2] = _lpf_bw_current_q.y1[1];
	_lpf_bw_current_q.y1[1] = _lpf_bw_current_q.y1[0];


	// second loop of 2nd order butterworth filter
	_lpf_bw_current_q.x2[0] = _lpf_bw_current_q.y1[0];
	_lpf_bw_current_q.y2[0] = (_lpf_bw_current_q.n0_2*_lpf_bw_current_q.x2[0]
							+ _lpf_bw_current_q.n1_2*_lpf_bw_current_q.x2[1]
							+ _lpf_bw_current_q.n2_2*_lpf_bw_current_q.x2[2]
			                - _lpf_bw_current_q.d1_2*_lpf_bw_current_q.y2[1]
						    - _lpf_bw_current_q.d2_2*_lpf_bw_current_q.y2[2]);

	_lpf_bw_current_q.x2[2] = _lpf_bw_current_q.x2[1];
	_lpf_bw_current_q.x2[1] = _lpf_bw_current_q.x2[0];


	_lpf_bw_current_q.y2[2] = _lpf_bw_current_q.y2[1];
	_lpf_bw_current_q.y2[1] = _lpf_bw_current_q.y2[0];

	_lpf_bw_current_q.filter_out = 	_lpf_bw_current_q.y2[0];
}

void LPF_BW_Current_D_4th()
{
	// first loop of 2nd order butterworth filter
	_lpf_bw_current_d.x1[0] = _drv8311_current.i_d;
	_lpf_bw_current_d.y1[0] = (_lpf_bw_current_d.n0_1*_lpf_bw_current_d.x1[0]
							+ _lpf_bw_current_d.n1_1*_lpf_bw_current_d.x1[1]
							+ _lpf_bw_current_d.n2_1*_lpf_bw_current_d.x1[2]
			                - _lpf_bw_current_d.d1_1*_lpf_bw_current_d.y1[1]
						    - _lpf_bw_current_d.d2_1*_lpf_bw_current_d.y1[2]);

	_lpf_bw_current_d.x1[2] = _lpf_bw_current_d.x1[1];
	_lpf_bw_current_d.x1[1] = _lpf_bw_current_d.x1[0];


	_lpf_bw_current_d.y1[2] = _lpf_bw_current_d.y1[1];
	_lpf_bw_current_d.y1[1] = _lpf_bw_current_d.y1[0];


	// second loop of 2nd order butterworth filter
	_lpf_bw_current_d.x2[0] = _lpf_bw_current_d.y1[0];
	_lpf_bw_current_d.y2[0] = (_lpf_bw_current_d.n0_2*_lpf_bw_current_d.x2[0]
							+ _lpf_bw_current_d.n1_2*_lpf_bw_current_d.x2[1]
							+ _lpf_bw_current_d.n2_2*_lpf_bw_current_d.x2[2]
			                - _lpf_bw_current_d.d1_2*_lpf_bw_current_d.y2[1]
						    - _lpf_bw_current_d.d2_2*_lpf_bw_current_d.y2[2]);

	_lpf_bw_current_d.x2[2] = _lpf_bw_current_d.x2[1];
	_lpf_bw_current_d.x2[1] = _lpf_bw_current_d.x2[0];


	_lpf_bw_current_d.y2[2] = _lpf_bw_current_d.y2[1];
	_lpf_bw_current_d.y2[1] = _lpf_bw_current_d.y2[0];

	_lpf_bw_current_d.filter_out = 	_lpf_bw_current_d.y2[0];
}


void LPF_Current_DC()
{
	unsigned long current_t = _micros();
	float dt = ((float)(current_t - _lpf_current_dc.filter_t_prev))*1e-6f;
	if(dt < 0.00000f) dt = 1e-4f;
	else if(dt > 1.0f/LPF_CURRENT_DC_Fc) dt = 1.0f/LPF_CURRENT_DC_Fc;
	{
		//_lpf_current_d.filter_out_prev = _drv8311_current.i_d;
		//_lpf_current_d.filter_t_prev = current_t;
	}
	//dt = 1e-4f;
	//float alpha = _2PI*LPF_CURRENT_D_Fc*dt/(1 + _2PI*LPF_CURRENT_D_Fc*dt);
	float alpha = 1/(1 + _2PI*LPF_CURRENT_DC_Fc*dt);
	//alpha = 0.94;
	_lpf_current_dc.filter_out_prev = (alpha)*_lpf_current_dc.filter_out_prev + (1.0f - alpha)*_drv8311_current.i_dc;
	_lpf_current_dc.filter_t_prev = current_t;
}


/*
void LPF_Current_D()
{
	unsigned long current_t = _micros();
	float dt = ((float)(current_t - _lpf_current_d.filter_t_prev))*1e-6f;
	if(dt < 0.00000f) dt = 1e-5f;
	else if(dt > 1.0f/LPF_CURRENT_D_Fc) dt = 1.0f/LPF_CURRENT_D_Fc;
	{
		//_lpf_current_d.filter_out_prev = _drv8311_current.i_d;
		//_lpf_current_d.filter_t_prev = current_t;
	}
	//dt = 1e-4f;
	//float alpha = _2PI*LPF_CURRENT_D_Fc*dt/(1 + _2PI*LPF_CURRENT_D_Fc*dt);
	float alpha = 1/(1 + _2PI*LPF_CURRENT_D_Fc*dt);
	//alpha = 0.94;
	_lpf_current_d.filter_out_prev = (alpha)*_lpf_current_d.filter_out_prev + (1.0f - alpha)*_drv8311_current.i_d;
	_lpf_current_d.filter_t_prev = current_t;
}


void LPF_Current_Q()
{
	unsigned long current_t = _micros();
	float dt = ((float)(current_t - _lpf_current_q.filter_t_prev))*1e-6f;
	if(dt < 0.00000f) dt = 1e-5f;
	else if(dt > 1.0f/LPF_CURRENT_Q_Fc) dt = 1.0f/LPF_CURRENT_Q_Fc;
	{
		//_lpf_current_q.filter_out_prev = _drv8311_current.i_q;
		//_lpf_current_q.filter_t_prev = current_t;
	}
	//dt = 1e-4f;
	//float alpha = _2PI*LPF_CURRENT_Q_Fc*dt/(1 + _2PI*LPF_CURRENT_Q_Fc*dt);//
	float alpha = 1/(1 + _2PI*LPF_CURRENT_Q_Fc*dt);
	//alpha = 0.5;
	_lpf_current_q.filter_out_prev = (alpha)*_lpf_current_q.filter_out_prev + (1.0f - alpha)*_drv8311_current.i_q;
	_lpf_current_q.filter_t_prev = current_t;
}
*/

void LPF_Current_D()
{
	unsigned long current_t = _micros();
	float dt = ((float)(current_t - _lpf_current_d.filter_t_prev))*1e-6f;
	if(dt < 0.0f) dt = 1e-5f;
	else if(dt > 0.3f)
	{
		_lpf_current_d.filter_out_prev = _drv8311_current.i_d;
		_lpf_current_d.filter_t_prev = current_t;
	}
	//dt = 1e-4f;
	//float alpha = _2PI*LPF_CURRENT_D_Fc*dt/(1 + _2PI*LPF_CURRENT_D_Fc*dt);
	float alpha = 1/(1 + _2PI*LPF_CURRENT_D_Fc*dt);
	//alpha = 0.94;
	_lpf_current_d.filter_out_prev = (alpha)*_lpf_current_d.filter_out_prev + (1.0f - alpha)*_drv8311_current.i_d;
	_lpf_current_d.filter_t_prev = current_t;
}


void LPF_Current_Q()
{
	unsigned long current_t = _micros();
	float dt = ((float)(current_t - _lpf_current_q.filter_t_prev))*1e-6f;
	if(dt < 0.0f) dt = 1e-5f;
	else if(dt > 0.3f)
	{
		_lpf_current_q.filter_out_prev = _drv8311_current.i_q;
		_lpf_current_q.filter_t_prev = current_t;
	}
	//dt = 1e-4f;
	//float alpha = _2PI*LPF_CURRENT_Q_Fc*dt/(1 + _2PI*LPF_CURRENT_Q_Fc*dt);//
	float alpha = 1/(1 + _2PI*LPF_CURRENT_Q_Fc*dt);
	//alpha = 0.5;
	_lpf_current_q.filter_out_prev = (alpha)*_lpf_current_q.filter_out_prev + (1.0f - alpha)*_drv8311_current.i_q;
	_lpf_current_q.filter_t_prev = current_t;
}


void LPF_ANGLE()
{
	unsigned long current_t = _micros();
	float dt = ((float)(current_t - _lpf_angle.filter_t_prev))*1e-6f;
	if(dt < 0.0f) dt = 1e-5f;
	else if(dt > 0.3f)
	{
		_lpf_angle.filter_out_prev = _as5047p.angle_raw_full;
		_lpf_angle.filter_t_prev = current_t;
	}

	float alpha = 1/(1 + _2PI*LPF_ANGLE_Fc*dt);
	_lpf_angle.filter_out_prev = alpha*_lpf_angle.filter_out_prev + (1.0f - alpha)*_as5047p.angle_raw_full;
	_lpf_angle.filter_t_prev = current_t;
}


void LPF_VELOCITY()
{
	unsigned long current_t = _micros();
	float dt = ((float)(current_t - _lpf_velocity.filter_t_prev))*1e-6f;
	if(dt < 0.0f) dt = 1e-5f;
	else if(dt > 0.3f)
	{
		_lpf_velocity.filter_out_prev = _as5047p.velocity_raw;
		_lpf_velocity.filter_t_prev = current_t;
	}

	float alpha = 1/(1 + _2PI*LPF_VELOCITY_Fc*dt);
	_lpf_velocity.filter_out_prev = alpha*_lpf_velocity.filter_out_prev + (1.0f - alpha)*_as5047p.velocity_raw;
	_lpf_velocity.filter_t_prev = current_t;
}
