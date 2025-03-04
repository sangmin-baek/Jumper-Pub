/*
 * ism330dhcx_spi_BSM.c
 *
 *  Created on: 2024. 9. 5.
 *      Author: BSM
 */


#include "ism330dhcx_spi_BSM.h"

_ISM330DHCX _ism330dhcx;
_IMU_DATA _imu_data;
_IMU_FILTER _imu_filter;
float _ism330dhcx_bit_to_xl;		// m/s^2
float _ism330dhcx_bit_to_gy;		// rad/s

void Get_IMU_Data_ISM330DHCX()
{
	Get_Gyro_ISM330DHCX();
	Get_Accel_ISM330DHCX();
}

void Get_Gyro_ISM330DHCX()
{
	uint8_t read_data[6];
	int16_t data_raw[3];

	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_OUTX_L_G, 6, read_data);
	data_raw[0] = (int16_t)read_data[1];
	data_raw[0] = data_raw[0]*256 + (int16_t)read_data[0];
	data_raw[1] = (int16_t)read_data[3];
	data_raw[1] = data_raw[1]*256 + (int16_t)read_data[2];
	data_raw[2] = (int16_t)read_data[5];
	data_raw[2] = data_raw[2]*256 + (int16_t)read_data[4];



	/*	IMU frame => Robot frame
	 * nose : x, right wing : y
	 *
	 * a_robot_x = a_imu_x
	 * a_robot_y = - a_imu_z
	 * a_robot_z = a_imu_y
	 *
	 * g_robot_x = g_imu_x
	 * g_robot_y = - g_imu_z
	 * g_robot_z = g_imu_y
	 *
	 */
	_imu_data.gyro.x = _ism330dhcx_bit_to_gy*(float)data_raw[0];
	_imu_data.gyro.y =  - _ism330dhcx_bit_to_gy*(float)data_raw[2];
	_imu_data.gyro.z = _ism330dhcx_bit_to_gy*(float)data_raw[1];
}

void Get_Accel_ISM330DHCX()
{
	uint8_t read_data[6];
	int16_t data_raw[3];

	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_OUTX_L_A, 6, read_data);
	data_raw[0] = (int16_t)read_data[1];
	data_raw[0] = data_raw[0]*256 + (int16_t)read_data[0];
	data_raw[1] = (int16_t)read_data[3];
	data_raw[1] = data_raw[1]*256 + (int16_t)read_data[2];
	data_raw[2] = (int16_t)read_data[5];
	data_raw[2] = data_raw[2]*256 + (int16_t)read_data[4];


	/*	IMU frame => Robot frame
	 * nose : x, right wing : y
	 *
	 * a_robot_x = a_imu_x
	 * a_robot_y = - a_imu_z
	 * a_robot_z = a_imu_y
	 *
	 * g_robot_x = g_imu_x
	 * g_robot_y = - g_imu_z
	 * g_robot_z = g_imu_y
	 *
	 */

	_imu_data.accel.x = _ism330dhcx_bit_to_xl*(float)data_raw[0];
	_imu_data.accel.y = - _ism330dhcx_bit_to_xl*(float)data_raw[2];
	_imu_data.accel.z = _ism330dhcx_bit_to_xl*(float)data_raw[1];
}

void Set_ISM330DHCX()
{
	/*
	 * 	set_device_conf
	 * 	set_block_data_update
	 * 	set_data_rate_accel
	 * 	set_data_rate_gyro
	 * 	set_full_scale_accel
	 * 	set_full_scale_gyro
	 * 	set_accel_slope_filter | set_accel_hp_path_on_out
	 * 	set_accel_filter_LP2	|	set_accel_filter_lp2
	 * 	set_gyro_filter_LP1
	 * 	set_gyro_LP1_bandwidth
	 */
	Set_Device_Conf_ISM330DHCX();
	Set_ODR_Accel_ISM330DHCX(ISM330DHCX_XL_ODR_1666Hz);
	Set_ODR_Gyro_ISM330DHCX(ISM330DHCX_GY_ODR_833Hz);
	//Set_ODR_Gyro_ISM330DHCX(ISM330DHCX_GY_ODR_1666Hz);
	Set_Full_Scale_Accel_ISM330DHCX(ISM330DHCX_XL_FS_16g);
	Set_Full_Scale_Gyro_ISM330DHCX(ISM330DHCX_GY_FS_4000dps);
	Set_Filter_Path_Accel_ISM330DHCX(ISM330DHCX_XL_FILT_NO);
	Set_Filter_Path_Gyro_ISM330DHCX(ISM330DHCX_GY_FILT_NO);
}

void Set_Filter_Path_Gyro_ISM330DHCX(FILTER_PATH_GYRO_ISM330DHCX filter_path)
{
	uint8_t read_data = 0;
	uint8_t lpf1_sel_g = 0;
	uint8_t ftype = 0;
	uint8_t data_ctrl4_c = 0;
	uint8_t data_ctrl6_c = 0;

	lpf1_sel_g = (filter_path & 0b1000) >> 3;
	ftype = (filter_path & 0b0111);

	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_CTRL4_C, 1, &read_data);
	data_ctrl4_c = read_data & (0b11111101);
	data_ctrl4_c = data_ctrl4_c | (lpf1_sel_g << 1);
	Write_Byte_SPI_ISM330DHCX(ISM330DHCX_CTRL4_C, data_ctrl4_c);

	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_CTRL6_C, 1, &read_data);
	data_ctrl6_c = read_data & (0b11111000);
	data_ctrl6_c = data_ctrl6_c | ftype;
	Write_Byte_SPI_ISM330DHCX(ISM330DHCX_CTRL6_C, data_ctrl6_c);
}

void Set_Filter_Path_Accel_ISM330DHCX(FILTER_PATH_ACCEL_ISM330DHCX filter_path)
{
	uint8_t read_data = 0;
	uint8_t data_ctrl8_xl = 0;
	uint8_t data_ctrl1_xl = 0;
	uint8_t hpcf_xl = 0;
	uint8_t hp_slope_xl_en = 0;
	uint8_t lpf2_xl_en = 0;

	hpcf_xl = (filter_path & 0b111);
	hp_slope_xl_en = ((filter_path & 0b10000) >> 4);
	lpf2_xl_en = ((filter_path & 0b1000) >> 3);

	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_CTRL8_XL, 1, &read_data);
	data_ctrl8_xl = read_data & (0b00011001);
	data_ctrl8_xl = data_ctrl8_xl | (hpcf_xl << 5) | (hp_slope_xl_en << 2);
	Write_Byte_SPI_ISM330DHCX(ISM330DHCX_CTRL8_XL, data_ctrl8_xl);

	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_CTRL1_XL, 1, &read_data);
	data_ctrl1_xl = read_data & (0b11111101);
	data_ctrl1_xl = data_ctrl1_xl | (lpf2_xl_en << 1);
	Write_Byte_SPI_ISM330DHCX(ISM330DHCX_CTRL1_XL, data_ctrl1_xl);
}

void Set_Full_Scale_Gyro_ISM330DHCX(FULL_SCALE_GYRO_ISM330DHCX full_scale)
{
	uint8_t read_data = 0;
	uint8_t data_ctrl2_g = 0;

	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_CTRL2_G, 1, &read_data);
	data_ctrl2_g = read_data & 0b11110000;
	data_ctrl2_g = data_ctrl2_g | (full_scale);
	Write_Byte_SPI_ISM330DHCX(ISM330DHCX_CTRL2_G, data_ctrl2_g);

	switch (full_scale) {

		case ISM330DHCX_GY_FS_125dps:
			_ism330dhcx_bit_to_gy = 4.375/1000 * DEG_TO_RAD;
			break;

		case ISM330DHCX_GY_FS_250dps:
			_ism330dhcx_bit_to_gy = 8.75/1000 * DEG_TO_RAD;
			break;

		case ISM330DHCX_GY_FS_500dps:
			_ism330dhcx_bit_to_gy = 17.5/1000 * DEG_TO_RAD;
			break;

		case ISM330DHCX_GY_FS_1000dps:
			_ism330dhcx_bit_to_gy = 35.0/1000 * DEG_TO_RAD;
			break;

		case ISM330DHCX_GY_FS_2000dps:
			_ism330dhcx_bit_to_gy = 70.0/1000 * DEG_TO_RAD;
			break;

		case ISM330DHCX_GY_FS_4000dps:
			_ism330dhcx_bit_to_gy = 140.0/1000 * DEG_TO_RAD;
			break;

		default:
			break;
	}
}

void Set_Full_Scale_Accel_ISM330DHCX(FULL_SCALE_ACCEL_ISM330DHCX full_scale)
{
	uint8_t read_data = 0;
	uint8_t data_ctrl1_xl = 0;

	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_CTRL1_XL, 1, &read_data);
	data_ctrl1_xl = read_data & 0b11110011;
	data_ctrl1_xl = data_ctrl1_xl | (full_scale << 2);
	Write_Byte_SPI_ISM330DHCX(ISM330DHCX_CTRL1_XL, data_ctrl1_xl);


	switch (full_scale) {
		case ISM330DHCX_XL_FS_2g:
			_ism330dhcx_bit_to_xl = 0.061/1000 * 9.81;
			break;

		case ISM330DHCX_XL_FS_4g:
			_ism330dhcx_bit_to_xl = 0.122/1000 * 9.81;
			break;

		case ISM330DHCX_XL_FS_8g:
			_ism330dhcx_bit_to_xl = 0.244/1000 * 9.81;
			break;

		case ISM330DHCX_XL_FS_16g:
			_ism330dhcx_bit_to_xl = 0.488/1000 * 9.81;
			break;

		default:
			break;
	}
}

void Set_ODR_Gyro_ISM330DHCX(ODR_GYRO_ISM330DHCX odr)
{
	uint8_t read_data = 0;
	uint8_t data_ctrl2_g = 0;

	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_CTRL2_G, 1, &read_data);
	data_ctrl2_g = read_data & 0b00001111;
	data_ctrl2_g = data_ctrl2_g | (odr << 4);
	Write_Byte_SPI_ISM330DHCX(ISM330DHCX_CTRL2_G, data_ctrl2_g);
}

void Set_ODR_Accel_ISM330DHCX(ODR_ACCEL_ISM330DHCX odr)
{
	uint8_t read_data = 0;
	uint8_t data_ctrl1_xl = 0;

	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_CTRL1_XL, 1, &read_data);
	data_ctrl1_xl = read_data & 0b00001111;
	data_ctrl1_xl = data_ctrl1_xl | (odr << 4);
	Write_Byte_SPI_ISM330DHCX(ISM330DHCX_CTRL1_XL, data_ctrl1_xl);
}


void Set_Device_Conf_ISM330DHCX()
{
	uint8_t read_data = 0;
	uint8_t data_ctrl9_xl = 0;

	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_CTRL9_XL, 1, &read_data);
	data_ctrl9_xl = read_data & 0b11111101;
	data_ctrl9_xl = data_ctrl9_xl | 0x02;
	Write_Byte_SPI_ISM330DHCX(ISM330DHCX_CTRL9_XL, data_ctrl9_xl);
}


void Reset_ISM330DHCX()
{
	uint8_t read_data = 0;
	Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_CTRL3_C, 1, &read_data);
	read_data = read_data | 0x01;
	Write_Byte_SPI_ISM330DHCX(ISM330DHCX_CTRL3_C, read_data);

	while(read_data & 0x01)
	{
		Read_Bytes_SPI_ISM330DHCX(ISM330DHCX_CTRL3_C, 1, &read_data);
		read_data = read_data & 0x01;
	}
}


void Write_Byte_SPI_ISM330DHCX(uint8_t address, uint8_t write_data)
{
	uint8_t RW_add = 0;
	RW_add = (address | 0x00);

	HAL_GPIO_WritePin(_ism330dhcx.csPort, _ism330dhcx.csPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_ism330dhcx.hspi, &RW_add, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(_ism330dhcx.hspi, &write_data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(_ism330dhcx.csPort, _ism330dhcx.csPin, GPIO_PIN_SET);
}


void Read_Bytes_SPI_ISM330DHCX(uint8_t address, uint8_t len, uint8_t* read_data)
{
	uint8_t RW_add = 0;
	RW_add = (address | 0x80);

	HAL_GPIO_WritePin(_ism330dhcx.csPort, _ism330dhcx.csPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_ism330dhcx.hspi, &RW_add, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(_ism330dhcx.hspi, read_data, len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(_ism330dhcx.csPort, _ism330dhcx.csPin, GPIO_PIN_SET);
}


void Init_SPI_ISM330DHCX(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
	_ism330dhcx.hspi = hspi;
	_ism330dhcx.csPort = csPort;
	_ism330dhcx.csPin = csPin;

	_imu_data.accel.x = 0;
	_imu_data.accel.y = 0;
	_imu_data.accel.z = 0;

	_imu_data.gyro.x = 0;
	_imu_data.gyro.y = 0;
	_imu_data.gyro.z = 0;

	_imu_filter.quaternion[0] = 1;
	_imu_filter.quaternion[1] = 0;
	_imu_filter.quaternion[2] = 0;
	_imu_filter.quaternion[3] = 0;

	_imu_filter.roll = 0;
	_imu_filter.pitch = 0;
	_imu_filter.yaw = 0;

	_imu_filter.quaternion_from_gyro[0] = 1;
	_imu_filter.quaternion_from_gyro[1] = 0;
	_imu_filter.quaternion_from_gyro[2] = 0;
	_imu_filter.quaternion_from_gyro[3] = 0;

	_imu_filter.roll_from_gyro = 0;
	_imu_filter.pitch_from_gyro = 0;
	_imu_filter.yaw_from_gyro = 0;

	_imu_filter.pitch_gyro_int = 0;


	_imu_filter.w_b.x = 0;
	_imu_filter.w_b.y = 0;
	_imu_filter.w_b.z = 0;
}

void Quaternion_To_YPR_zyx()
{
	float r_11, r_12, r_21, r_13, r_31, r_23, r_32, r_33;
	/* Robot frame = R * World frame
	 * R	 = [ cos(p)cos(y)							cos(p)sin(y)					-sin(p);
	 * 			sin(r)sin(p)cos(y)-cos(r)sin(y)		sin(r)sin(p)sin(y)+cos(r)cos(y)		cos(p)sin(r);
	 * 			cos(r)sin(p)cos(y)+sin(r)sin(y)		cos(r)sin(p)sin(y)-sin(r)cos(y)		cos(p)cos(r);]
	 *
	 * R(q)	 = [q0^2 + q1^2 - q2^2 - q3^2		2(q1q2 + q0q3)				2(q1q3 - q0q2);
	 * 			2(q1q2 - q0q3)				q0^2 - q1^2 + q2^2 - q3^2		2(q2q3 + q0q1);
	 * 			2(q0q2 + q1q3)					2(q2q3 - q0q1)			q0^2 - q1^2 - q2^2 + q3^2]
	 */
	r_11 = 2.0f * (_imu_filter.quaternion[0]*_imu_filter.quaternion[0] + _imu_filter.quaternion[1]*_imu_filter.quaternion[1]) - 1;
	r_12 = 2.0f * (_imu_filter.quaternion[1]*_imu_filter.quaternion[2] + _imu_filter.quaternion[0]*_imu_filter.quaternion[3]);
	r_21 = 2.0f * (_imu_filter.quaternion[1]*_imu_filter.quaternion[2] - _imu_filter.quaternion[0]*_imu_filter.quaternion[3]);

	r_13 = 2.0f * (_imu_filter.quaternion[1]*_imu_filter.quaternion[3] - _imu_filter.quaternion[0]*_imu_filter.quaternion[2]);
	r_31 = 2.0f * (_imu_filter.quaternion[1]*_imu_filter.quaternion[3] + _imu_filter.quaternion[0]*_imu_filter.quaternion[2]);

	r_23 = 2.0f * (_imu_filter.quaternion[2]*_imu_filter.quaternion[3] + _imu_filter.quaternion[0]*_imu_filter.quaternion[1]);
	r_32 = 2.0f * (_imu_filter.quaternion[2]*_imu_filter.quaternion[3] - _imu_filter.quaternion[0]*_imu_filter.quaternion[1]);
	r_33 = 2.0f * (_imu_filter.quaternion[0]*_imu_filter.quaternion[0] + _imu_filter.quaternion[3]*_imu_filter.quaternion[3]) - 1;

	_imu_filter.roll = atan2f(r_23,r_33);
	_imu_filter.pitch = asinf(r_13);
	/*
	if(fabs(-r_13) >= 1.0)
	{
		_imu_filter.pitch = copysignf(_PI/2.0, -r_13);
	}
	else
	{
		_imu_filter.pitch = asinf(-r_13);
	}
	*/
	_imu_filter.yaw = -atan2f(r_12,r_11);


	_imu_filter.roll = wrapAngle(_imu_filter.roll);
	_imu_filter.pitch = wrapAngle(_imu_filter.pitch);
	_imu_filter.yaw = wrapAngle(_imu_filter.yaw);
/*
	_imu_filter.roll = atan2f(r_32,r_33);
	_imu_filter.pitch = - asinf(r_31);
	_imu_filter.yaw = atan2f(r_21,r_11);
*/


	//_imu_filter.yaw = ((_imu_filter.yaw > _PI) ? (_imu_filter.yaw - _2PI) : ((_imu_filter.yaw < - _PI) ? (_imu_filter.yaw + _2PI) : _imu_filter.yaw));
}


void Quaternion_To_YPR_zyx_from_gyro()
{
	float r_11, r_12, r_21, r_13, r_31, r_23, r_32, r_33;
	/*	Robot frame = R * World frame
	 * R	 = [ cos(p)cos(y)							cos(p)sin(y)					-sin(p);
	 * 			sin(r)sin(p)cos(y)-cos(r)sin(y)		sin(r)sin(p)sin(y)+cos(r)cos(y)		cos(p)sin(r);
	 * 			cos(r)sin(p)cos(y)+sin(r)sin(y)		cos(r)sin(p)sin(y)-sin(r)cos(y)		cos(p)cos(r);]
	 *
	 * R(q)	 = [q0^2 + q1^2 - q2^2 - q3^2		2(q1q2 + q0q3)				2(q1q3 - q0q2);
	 * 			2(q1q2 - q0q3)				q0^2 - q1^2 + q2^2 - q3^2		2(q2q3 + q0q1);
	 * 			2(q0q2 + q1q3)					2(q2q3 - q0q1)			q0^2 - q1^2 - q2^2 + q3^2]
	 */
	r_11 = 2.0f * (_imu_filter.quaternion_from_gyro[0]*_imu_filter.quaternion_from_gyro[0] + _imu_filter.quaternion_from_gyro[1]*_imu_filter.quaternion_from_gyro[1]) - 1;
	r_12 = 2.0f * (_imu_filter.quaternion_from_gyro[1]*_imu_filter.quaternion_from_gyro[2] + _imu_filter.quaternion_from_gyro[0]*_imu_filter.quaternion_from_gyro[3]);
	r_21 = 2.0f * (_imu_filter.quaternion_from_gyro[1]*_imu_filter.quaternion_from_gyro[2] - _imu_filter.quaternion_from_gyro[0]*_imu_filter.quaternion_from_gyro[3]);

	r_13 = 2.0f * (_imu_filter.quaternion_from_gyro[1]*_imu_filter.quaternion_from_gyro[3] - _imu_filter.quaternion_from_gyro[0]*_imu_filter.quaternion_from_gyro[2]);
	r_31 = 2.0f * (_imu_filter.quaternion_from_gyro[1]*_imu_filter.quaternion_from_gyro[3] + _imu_filter.quaternion_from_gyro[0]*_imu_filter.quaternion_from_gyro[2]);

	r_23 = 2.0f * (_imu_filter.quaternion_from_gyro[2]*_imu_filter.quaternion_from_gyro[3] + _imu_filter.quaternion_from_gyro[0]*_imu_filter.quaternion_from_gyro[1]);
	r_32 = 2.0f * (_imu_filter.quaternion_from_gyro[2]*_imu_filter.quaternion_from_gyro[3] - _imu_filter.quaternion_from_gyro[0]*_imu_filter.quaternion_from_gyro[1]);
	r_33 = 2.0f * (_imu_filter.quaternion_from_gyro[0]*_imu_filter.quaternion_from_gyro[0] + _imu_filter.quaternion_from_gyro[3]*_imu_filter.quaternion_from_gyro[3]) - 1;

	_imu_filter.roll_from_gyro = atan2f(r_23,r_33);
	_imu_filter.pitch_from_gyro = asinf(r_13);
	/*
	if(fabs(-r_13) >= 1.0)
	{
		_imu_filter.pitch_from_gyro = copysignf(_PI/2.0, -r_13);
	}
	else
	{
		_imu_filter.pitch_from_gyro = asinf(-r_13);
	}
	*/
	_imu_filter.yaw_from_gyro = -atan2f(r_12,r_11);

	_imu_filter.roll_from_gyro = wrapAngle(_imu_filter.roll_from_gyro);
	_imu_filter.pitch_from_gyro = wrapAngle(_imu_filter.pitch_from_gyro);
	_imu_filter.yaw_from_gyro = wrapAngle(_imu_filter.yaw_from_gyro);
/*
	_imu_filter.roll_from_gyro = atan2f(r_32,r_33);
	_imu_filter.pitch_from_gyro = - asinf(r_31);
	_imu_filter.yaw_from_gyro = atan2f(r_21,r_11);
*/


	//_imu_filter.yaw_from_gyro = ((_imu_filter.yaw_from_gyro > _PI) ? (_imu_filter.yaw_from_gyro - _2PI) : ((_imu_filter.yaw_from_gyro < - _PI) ? (_imu_filter.yaw_from_gyro + _2PI) : _imu_filter.yaw_from_gyro));
}


void IMU_Filter_Madgwick()
{
	float q0 = _imu_filter.quaternion[0];
	float q1 = _imu_filter.quaternion[1];
	float q2 = _imu_filter.quaternion[2];
	float q3 = _imu_filter.quaternion[3];

	float a_x = _imu_data.accel.x;
	float a_y = _imu_data.accel.y;
	float a_z = _imu_data.accel.z;

	float g_x = _imu_data.gyro.x;
	float g_y = _imu_data.gyro.y;
	float g_z = _imu_data.gyro.z;


	float q0_dot_w, q1_dot_w, q2_dot_w, q3_dot_w;			// Quaternion rate from Gyro
	float q0_hat_dot, q1_hat_dot, q2_hat_dot, q3_hat_dot;	// Quaternion rate from gradient descent
	float wx_e, wy_e, wz_e;									// Gyro error
	float w_x, w_y, w_z;									// Bias removed Gyro



	float fg1, fg2, fg3;									// Error (quaternion prediction - accel sensor read)
	float fg1_d0, fg1_d1, fg1_d2, fg1_d3;					// Gradients
	float fg2_d0, fg2_d1, fg2_d2, fg2_d3;
	float fg3_d0, fg3_d1, fg3_d2, fg3_d3;



	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;


	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;


	float q2q2 = q2*q2;
	float q2q3 = q2*q3;

	float q3q3 = q3*q3;



	float norm;
	float norm_inv;
	float dt = 1.0f / MADGWICK_FILTER_FREQ;


	// Normalize accelerometer & Magnetometer data
	norm = a_x*a_x + a_y*a_y + a_z*a_z;
	if(norm == 0.0f) return;
	norm_inv = _inv_sqrtApprox(norm);
	a_x *= norm_inv;
	a_y *= norm_inv;
	a_z *= norm_inv;




	// Convert reference axis to the sensor axis

	fg1 = 2.0f * (q1q3 - q0q2) - a_x;
	fg2 = 2.0f * (q0q1 + q2q3) - a_y;
	fg3 = 2.0f * (0.5f - q1q1 - q2q2) - a_z;



	// Gradients of above fcns.
	fg1_d0 = - 2.0f * q2;
	fg1_d1 = 2.0f * q3;
	fg1_d2 = - 2.0f * q0;
	fg1_d3 = 2.0f * q1;

	fg2_d0 = fg1_d3;
	fg2_d1 = - fg1_d2;
	fg2_d2 = fg1_d1;
	fg2_d3 = - fg1_d0;

	fg3_d0 = 0;
	fg3_d1 = - 4.0f * q1;
	fg3_d2 = - 4.0f * q2;
	fg3_d3 = 0;



	// gradient descent
	q0_hat_dot = fg1_d0 * fg1 + fg2_d0 * fg2 + fg3_d0 * fg3;		// use magnetometer data or not
	q1_hat_dot = fg1_d1 * fg1 + fg2_d1 * fg2 + fg3_d1 * fg3;
	q2_hat_dot = fg1_d2 * fg1 + fg2_d2 * fg2 + fg3_d2 * fg3;
	q3_hat_dot = fg1_d3 * fg1 + fg2_d3 * fg2 + fg3_d3 * fg3;

	norm = q0_hat_dot*q0_hat_dot + q1_hat_dot*q1_hat_dot + q2_hat_dot*q2_hat_dot + q3_hat_dot*q3_hat_dot;
	norm_inv = _inv_sqrtApprox(norm);
	q0_hat_dot *= norm_inv;
	q1_hat_dot *= norm_inv;
	q2_hat_dot *= norm_inv;
	q3_hat_dot *= norm_inv;


	// Compute gyro error
	wx_e = 2.0f * (q0 * q1_hat_dot - q1 * q0_hat_dot - q2 * q3_hat_dot + q3 * q2_hat_dot);
	wy_e = 2.0f * (q0 * q2_hat_dot + q1 * q3_hat_dot - q2 * q0_hat_dot - q3 * q1_hat_dot);
	wz_e = 2.0f * (q0 * q3_hat_dot - q1 * q2_hat_dot + q2 * q1_hat_dot - q3 * q0_hat_dot);


	// gyro bias compensation
	_imu_filter.w_b.x += wx_e * dt *ZETA_QUATERNION;
	_imu_filter.w_b.y += wy_e * dt *ZETA_QUATERNION;
	_imu_filter.w_b.z += wz_e * dt *ZETA_QUATERNION;

	w_x = g_x - _imu_filter.w_b.x;
	w_y = g_y - _imu_filter.w_b.y;
	w_z = g_z - _imu_filter.w_b.z;


	// rate of change of quaternion using gyro read & Numetical integration
	q0_dot_w = 0.5f * (-q1*w_x - q2*w_y - q3*w_z ) - BETA_QUATERNION * q0_hat_dot;
	q1_dot_w = 0.5f * (q0*w_x + q2*w_z - q3*w_y ) - BETA_QUATERNION * q1_hat_dot;
	q2_dot_w = 0.5f * (q0*w_y - q1*w_z + q3*w_x ) - BETA_QUATERNION * q2_hat_dot;
	q3_dot_w = 0.5f * (q0*w_z + q1*w_y - q2*w_x ) - BETA_QUATERNION * q3_hat_dot;



	q0 += q0_dot_w * dt;
	q1 += q1_dot_w * dt;
	q2 += q2_dot_w * dt;
	q3 += q3_dot_w * dt;


	norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
	norm_inv = _inv_sqrtApprox(norm);

	q0 *= norm_inv;
	q1 *= norm_inv;
	q2 *= norm_inv;
	q3 *= norm_inv;



	_imu_filter.quaternion[0] = q0;
	_imu_filter.quaternion[1] = q1;
	_imu_filter.quaternion[2] = q2;
	_imu_filter.quaternion[3] = q3;
}

void IMU_Attitude_from_Gyro()
{
	float q0 = _imu_filter.quaternion_from_gyro[0];
	float q1 = _imu_filter.quaternion_from_gyro[1];
	float q2 = _imu_filter.quaternion_from_gyro[2];
	float q3 = _imu_filter.quaternion_from_gyro[3];

	float g_x = _imu_data.gyro.x;
	float g_y = _imu_data.gyro.y;
	float g_z = _imu_data.gyro.z;

	float q0_dot = -0.5*(q1*g_x + q2*g_y + q3*g_z);
	float q1_dot = 0.5*(q0*g_x + q2*g_z - q3*g_y);
	float q2_dot = 0.5*(q0*g_y - q1*g_z + q3*g_x);
	float q3_dot = 0.5*(q0*g_z + q1*g_y - q2*g_x);

	float norm;
	float norm_inv;
	float dt = 1.0f / MADGWICK_FILTER_FREQ;

	q0 += q0_dot*dt;
	q1 += q1_dot*dt;
	q2 += q2_dot*dt;
	q3 += q3_dot*dt;

	norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
	norm_inv = _inv_sqrtApprox(norm);

	q0 *= norm_inv;
	q1 *= norm_inv;
	q2 *= norm_inv;
	q3 *= norm_inv;

	_imu_filter.quaternion_from_gyro[0] = q0;
	_imu_filter.quaternion_from_gyro[1] = q1;
	_imu_filter.quaternion_from_gyro[2] = q2;
	_imu_filter.quaternion_from_gyro[3] = q3;
}

void IMU_Attitude_from_Gyro_Int_Pitch()
{
	_imu_filter.pitch_gyro_int = _imu_filter.pitch_gyro_int + _imu_data.gyro.y * 1/MADGWICK_FILTER_FREQ;
}

void Switch_Attitude_estimator()
{
	//float a_mag = _imu_data.accel.x*_imu_data.accel.x + _imu_data.accel.y*_imu_data.accel.y + _imu_data.accel.z*_imu_data.accel.z;

	//if(a_mag > IMU_SW_CRITERIA)
	{
		_imu_filter.quaternion_from_gyro[0] = _imu_filter.quaternion[0];
		_imu_filter.quaternion_from_gyro[1] = _imu_filter.quaternion[1];
		_imu_filter.quaternion_from_gyro[2] = _imu_filter.quaternion[2];
		_imu_filter.quaternion_from_gyro[3] = _imu_filter.quaternion[3];

		_imu_filter.pitch_gyro_int = _imu_filter.pitch;
	}
}

void Copy_Attitude_Init_Val()
{
	//float a_mag = _imu_data.accel.x*_imu_data.accel.x + _imu_data.accel.y*_imu_data.accel.y + _imu_data.accel.z*_imu_data.accel.z;

	//if(a_mag > IMU_SW_CRITERIA)
	{
		_imu_filter.quaternion_from_gyro[0] = _imu_filter.quaternion[0];
		_imu_filter.quaternion_from_gyro[1] = _imu_filter.quaternion[1];
		_imu_filter.quaternion_from_gyro[2] = _imu_filter.quaternion[2];
		_imu_filter.quaternion_from_gyro[3] = _imu_filter.quaternion[3];

		_imu_filter.pitch_gyro_int = _imu_filter.pitch;
	}
}


float wrapAngle(float angle)
{
    while (angle > _PI) {
        angle -= 2.0 * _PI;
    }
    while (angle < -_PI) {
        angle += 2.0 * _PI;
    }
    return angle;
}
