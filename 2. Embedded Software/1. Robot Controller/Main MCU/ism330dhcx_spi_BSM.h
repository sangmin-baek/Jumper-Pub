/*
 * ism330dhcx_spi_BSM.h
 *
 *  Created on: Sep 5, 2024
 *      Author: BSM
 */

/*	ioc related settings
 * CPOL : 1, High
 * CPHA : 1, 2nd edge
 * clk freq max : 10 MHz
 * MSB first
 * 8 bit
 **
 *
 */

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


#ifndef INC_ISM330DHCX_SPI_BSM_H_
#define INC_ISM330DHCX_SPI_BSM_H_

#include "stm32h7xx_hal.h"
#include "math_utils_bsm.h"

/**********************************
 * 	[ISM330DHCX Register map]
 *********************************/
#define ISM330DHCX_WHO_AM_I			0x0FU
#define ISM330DHCX_CTRL1_XL			0x10U
#define ISM330DHCX_CTRL2_G			0x11U
#define ISM330DHCX_CTRL3_C			0x12U
#define ISM330DHCX_CTRL4_C			0x13U
#define ISM330DHCX_CTRL5_C			0x14U
#define ISM330DHCX_CTRL6_C			0x15U
#define ISM330DHCX_CTRL7_C			0x16U
#define ISM330DHCX_CTRL8_XL			0x17U
#define ISM330DHCX_CTRL9_XL			0x18U
#define ISM330DHCX_CTRL10_C			0x19U

#define ISM330DHCX_OUT_TEMP_L		0x20U
#define ISM330DHCX_OUT_TEMP_H		0x21U

#define ISM330DHCX_OUTX_L_G			0x22U
#define ISM330DHCX_OUTX_H_G			0x23U
#define ISM330DHCX_OUTY_L_G			0x24U
#define ISM330DHCX_OUTY_H_G			0x25U
#define ISM330DHCX_OUTZ_L_G			0x26U
#define ISM330DHCX_OUTZ_H_G			0x27U

#define ISM330DHCX_OUTX_L_A			0x28U
#define ISM330DHCX_OUTX_H_A			0x29U
#define ISM330DHCX_OUTY_L_A			0x2AU
#define ISM330DHCX_OUTY_H_A			0x2BU
#define ISM330DHCX_OUTZ_L_A			0x2CU
#define ISM330DHCX_OUTZ_H_A			0x2DU



/**********************************
 * 	[Madgwick filter related Macros]
 *********************************/

#define MADGWICK_FILTER_FREQ		500.0f
#define GYRO_MEAS_ERROR				_PI * 5.0f / 180.0f
#define GYRO_MEAS_DRIFT				_PI * 0.2f / 180.0f
#define BETA_QUATERNION				_SQRT3_2*GYRO_MEAS_ERROR
#define ZETA_QUATERNION				_SQRT3_2*GYRO_MEAS_DRIFT




/**********************************
 * 	[Estimator Switch related Macros]
 *********************************/

#define IMU_SW_CRITERIA		(1.0f)*(1.0f)

typedef enum{
	ISM330DHCX_XL_ODR_12Hz5  = 1,
	ISM330DHCX_XL_ODR_26Hz   = 2,
	ISM330DHCX_XL_ODR_52Hz   = 3,
	ISM330DHCX_XL_ODR_104Hz  = 4,
	ISM330DHCX_XL_ODR_208Hz  = 5,
	ISM330DHCX_XL_ODR_416Hz  = 6,
	ISM330DHCX_XL_ODR_833Hz  = 7,
	ISM330DHCX_XL_ODR_1666Hz = 8,
	ISM330DHCX_XL_ODR_3332Hz = 9,
	ISM330DHCX_XL_ODR_6667Hz = 10,
} ODR_ACCEL_ISM330DHCX;

typedef enum{
	ISM330DHCX_GY_ODR_12Hz5  = 1,
	ISM330DHCX_GY_ODR_26Hz   = 2,
	ISM330DHCX_GY_ODR_52Hz   = 3,
	ISM330DHCX_GY_ODR_104Hz  = 4,
	ISM330DHCX_GY_ODR_208Hz  = 5,
	ISM330DHCX_GY_ODR_416Hz  = 6,
	ISM330DHCX_GY_ODR_833Hz  = 7,
	ISM330DHCX_GY_ODR_1666Hz = 8,
	ISM330DHCX_GY_ODR_3332Hz = 9,
	ISM330DHCX_GY_ODR_6667Hz = 10,
} ODR_GYRO_ISM330DHCX;

typedef enum{
	ISM330DHCX_XL_FS_2g   = 0,
	ISM330DHCX_XL_FS_16g  = 1,
	ISM330DHCX_XL_FS_4g   = 2,
	ISM330DHCX_XL_FS_8g   = 3,
} FULL_SCALE_ACCEL_ISM330DHCX;

typedef enum{
	ISM330DHCX_GY_FS_125dps = 2,
	ISM330DHCX_GY_FS_250dps = 0,
	ISM330DHCX_GY_FS_500dps = 4,
	ISM330DHCX_GY_FS_1000dps = 8,
	ISM330DHCX_GY_FS_2000dps = 12,
	ISM330DHCX_GY_FS_4000dps = 1,
} FULL_SCALE_GYRO_ISM330DHCX;

typedef enum{
	ISM330DHCX_XL_FILT_NO				= 0,		// HP_SLOPE_XL_EN, LPF2_XL_EN, HPCF_XL_[2:0]
	ISM330DHCX_XL_FILT_LP_ODR_DIV_4		= 0b01000,
	ISM330DHCX_XL_FILT_LP_ODR_DIV_10	= 0b01001,
	ISM330DHCX_XL_FILT_LP_ODR_DIV_20	= 0b01010,
	ISM330DHCX_XL_FILT_LP_ODR_DIV_45	= 0b01011,
	ISM330DHCX_XL_FILT_LP_ODR_DIV_100	= 0b01100,
	ISM330DHCX_XL_FILT_LP_ODR_DIV_200	= 0b01101,
	ISM330DHCX_XL_FILT_LP_ODR_DIV_400	= 0b01110,
	ISM330DHCX_XL_FILT_LP_ODR_DIV_800	= 0b01111,

	ISM330DHCX_XL_FILT_HP_ODR_DIV_4		= 0b10000,
	ISM330DHCX_XL_FILT_HP_ODR_DIV_10	= 0b10001,
	ISM330DHCX_XL_FILT_HP_ODR_DIV_20	= 0b10010,
	ISM330DHCX_XL_FILT_HP_ODR_DIV_45	= 0b10011,
	ISM330DHCX_XL_FILT_HP_ODR_DIV_100	= 0b10100,
	ISM330DHCX_XL_FILT_HP_ODR_DIV_200	= 0b10101,
	ISM330DHCX_XL_FILT_HP_ODR_DIV_400	= 0b10110,
	ISM330DHCX_XL_FILT_HP_ODR_DIV_800	= 0b10111,

} FILTER_PATH_ACCEL_ISM330DHCX;

typedef enum{
	ISM330DHCX_GY_FILT_NO			= 0b0000,		// LPF1_SEL_G, FTYPE[2:0]
	ISM330DHCX_GY_FILT_ULTRA_LIGHT	= 0b1000,
	ISM330DHCX_GY_FILT_VERY_LIGHT	= 0b1001,
	ISM330DHCX_GY_FILT_LIGHT		= 0b1010,
	ISM330DHCX_GY_FILT_MEDIUM		= 0b1011,
	ISM330DHCX_GY_FILT_STRONG		= 0b1100,
	ISM330DHCX_GY_FILT_VERY_STRONG	= 0b1101,
	ISM330DHCX_GY_FILT_AGGRESSIVE	= 0b1110,
	ISM330DHCX_GY_FILT_XTREME		= 0b1111,

} FILTER_PATH_GYRO_ISM330DHCX;


typedef struct{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *csPort;
	uint16_t csPin;
} _ISM330DHCX;

typedef struct{
	volatile float x, y, z;
} _VECT_3;

typedef struct{
	_VECT_3 gyro;
	_VECT_3 accel;
} _IMU_DATA;

typedef struct{

	float quaternion[4];
	float roll;
	float pitch;
	float yaw;
	float quaternion_from_gyro[4];
	float roll_from_gyro;
	float pitch_from_gyro;
	float yaw_from_gyro;
	float pitch_gyro_int;



	_VECT_3 w_b;			// estimate gyro bias error

} _IMU_FILTER;

extern _ISM330DHCX _ism330dhcx;
extern _IMU_DATA _imu_data;
extern _IMU_FILTER _imu_filter;
extern float _ism330dhcx_bit_to_xl;		// m/s^2
extern float _ism330dhcx_bit_to_gy;		// rad/s


void Get_IMU_Data_ISM330DHCX();
void Get_Gyro_ISM330DHCX();
void Get_Accel_ISM330DHCX();

void Set_ISM330DHCX();

void Set_Filter_Path_Gyro_ISM330DHCX(FILTER_PATH_GYRO_ISM330DHCX filter_path);
void Set_Filter_Path_Accel_ISM330DHCX(FILTER_PATH_ACCEL_ISM330DHCX filter_path);
void Set_Full_Scale_Gyro_ISM330DHCX(FULL_SCALE_GYRO_ISM330DHCX full_scale);
void Set_Full_Scale_Accel_ISM330DHCX(FULL_SCALE_ACCEL_ISM330DHCX full_scale);
void Set_ODR_Gyro_ISM330DHCX(ODR_GYRO_ISM330DHCX odr);
void Set_ODR_Accel_ISM330DHCX(ODR_ACCEL_ISM330DHCX odr);
void Set_Device_Conf_ISM330DHCX();

void Reset_ISM330DHCX();

void Write_Byte_SPI_ISM330DHCX(uint8_t address, uint8_t write_data);
void Read_Bytes_SPI_ISM330DHCX(uint8_t address, uint8_t len, uint8_t* read_data);

void Init_SPI_ISM330DHCX(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);


void Quaternion_To_YPR_zyx();
void Quaternion_To_YPR_zyx_from_gyro();
void IMU_Filter_Madgwick();
void IMU_Attitude_from_Gyro();
void IMU_Attitude_from_Gyro_Int_Pitch();


void Switch_Attitude_estimator();
void Copy_Attitude_Init_Val();
float wrapAngle(float angle);
#endif /* INC_ISM330DHCX_SPI_BSM_H_ */
