/*
 * as5047p_mag_position_spi_bsm.h
 *
 *  Created on: 2024. 3. 21.
 *      Author: BSM
 */

#ifndef INC_AS5047P_MAG_POSITION_SPI_BSM_H_
#define INC_AS5047P_MAG_POSITION_SPI_BSM_H_


/*-----------------------------------------
 * Include---------------------------------
 * ---------------------------------------*/


#include "math_utils_bsm.h"



/*-----------------------------------------
 * CONSTANTS---------------------------------
 * ---------------------------------------*/
#define AS5047P_OPT_ENABLED		1
#define AS5047P_OPT_DISABLED	0

#define AS5047P_ACCESS_WRITE 	0
#define AS5047P_ACCESS_READ 	1

#define AS5047P_FRAME_PARD		0x8000
#define AS5047P_FRAME_EF 		0x4000
#define AS5047P_FRAME_DATA		0x3FFF

// --- Volatile registers
#define AS5047P_NOP          	0x0000
#define AS5047P_ERRFL        	0x0001
#define AS5047P_PROG        	0x0003
#define AS5047P_DIAAGC       	0x3FFC
#define AS5047P_MAG          	0x3FFD
#define AS5047P_ANGLEUNC     	0x3FFE
#define AS5047P_ANGLECOM     	0x3FFF

// --- Non-volatile registers
#define AS5047P_ZPOSM        	0x0016
#define AS5047P_ZPOSL        	0x0017
#define AS5047P_SETTINGS1    	0x0018
#define AS5047P_SETTINGS2    	0x0019

// --- Fields in registers
#define AS5047P_ERRFL_PARERR			4
#define AS5047P_ERRFL_INVCOMM			2
#define AS5047P_ERRFL_FRERR				1
#define AS5047P_PROG_PROGVER			0x40
#define AS5047P_PROG_PROGOTP			8
#define AS5047P_PROG_OTPREF				4
#define AS5047P_PROG_PROGEN				1
#define AS5047P_DIAAGC_MAGL				0x800
#define AS5047P_DIAAGC_MAGH				0x400
#define AS5047P_DIAAGC_COF				0x200
#define AS5047P_DIAAGC_LF				0x100
#define AS5047P_DIAAGC_AGC				0x00FF
#define AS5047P_MAG_CMAG				0x3FFF
#define AS5047P_ANGLEUNC_CORDICANG		0x3FFF
#define AS5047P_ANGLECOM_DAECANG		0x3FFF
#define AS5047P_ZPOSM_ZPOSM				0x00FF
#define AS5047P_ZPOSL_COMP_H_ERR_EN		0x80
#define AS5047P_ZPOSL_COMP_I_ERR_EN		0x40
#define AS5047P_ZPOSL_ZPOSL				0x003F
#define AS5047P_SETTINGS1_BIT0			1
#define AS5047P_SETTINGS1_NOISESET		2
#define AS5047P_SETTINGS1_DIR			4
#define AS5047P_SETTINGS1_UVW_ABI		8
#define AS5047P_SETTINGS1_DAECDIS		0x10
#define AS5047P_SETTINGS1_ABIBIN		0x20
#define AS5047P_SETTINGS1_DATASEL		0x40
#define AS5047P_SETTINGS1_PWMON			0x80
#define AS5047P_SETTINGS2_UVWPP			0x0007
#define AS5047P_SETTINGS2_HYS			0x0018
#define AS5047P_SETTINGS2_ABIRES		0x00E0



/*-----------------------------------------
 * CUSTOM SETTINGS-------------------------
 * ---------------------------------------*/
#define AS5047P_SET1_ROTATION_DIR		0	//0
#define AS5047P_SET1_UVW				1	// 0=ABI, 1=UWV
#define AS5047P_SET1_ABI				0
#define AS5047P_SET1_DAECDIS			0	// 0=DAE compensation ON, 1=OFF
#define AS5047P_SET1_ABIBIN				0
#define AS5047P_SET1_DATASELECT			0	// 0=DAECANG, 1=CORDICANG
#define AS5047P_SET1_PWMON				0

#define AS5047P_SET2_UVWPP				0b111	// number of pole pairs
#define AS5047P_SET2_HYS				0
#define AS5047P_SET2_ABIRES				0

#define _14BIT_TO_RAD					0.000383495197f
#define _1000CNT_TO_RAD					0.006283185307f
#define _4000CNT_TO_RAD					0.001570796327f

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *csPort;
	uint16_t csPin;

	TIM_HandleTypeDef *htim;

	float angle_raw;
	float angle_raw_full;
	float velocity_raw;

	int32_t rotation_cnt;
	unsigned long t_velocity_prev;
}_AS5047P;

typedef int16_t AS5047P_Result;
/* >= 0 valid
 * < 0 error
 * */


typedef int16_t AS5047P_ErrCode;
/* != 0 error code
 * == 0 no error
 * */


extern _AS5047P _as5047p;


void AS5047P_Sensing_Init();
void AS5047P_Get_Raw_Angle_Velocity_ABZ();
void AS5047P_Get_Raw_Angle_Velocity();


AS5047P_ErrCode AS5047P_Setting();
AS5047P_ErrCode AS5047P_Setting_ABZ();
AS5047P_ErrCode AS5047P_Set_Factory_Settings();
AS5047P_Result AS5047P_Read_Position();
AS5047P_Result AS5047P_Read_Position_ABZ();


AS5047P_ErrCode AS5047P_Set_Zero_Position();
AS5047P_ErrCode Init_SPI_AS5047P(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);
AS5047P_ErrCode Init_ABZ_AS5047P(TIM_HandleTypeDef *htim);

AS5047P_ErrCode AS5047P_Write_Register(uint16_t address, uint16_t data_write);
AS5047P_Result AS5047P_Read_Register(uint16_t address);
AS5047P_Result AS5047P_Read_Write_Raw(uint16_t data_tx, uint8_t rw_bit);

AS5047P_ErrCode AS5047P_Burn_OTP();

#endif /* INC_AS5047P_MAG_POSITION_SPI_BSM_H_ */
