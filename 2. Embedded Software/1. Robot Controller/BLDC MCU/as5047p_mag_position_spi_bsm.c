/*
 * as5047p_mag_position_spi_bsm.c
 *
 *  Created on: 2024. 3. 21.
 *      Author: BSM
 */


#include "as5047p_mag_position_spi_bsm.h"

_AS5047P _as5047p;


static uint8_t AS5047P_Calc_Parity(uint32_t v)
{
	v ^= v >> 1;
	v ^= v >> 2;
	v = (v & 0x11111111U) * 0x11111111U;
	return (v >> 28) & 1;
}





void AS5047P_Sensing_Init()
{
	AS5047P_Get_Raw_Angle_Velocity();
	_micros_delay(100);
	AS5047P_Get_Raw_Angle_Velocity();
	_micros_delay(100);
	AS5047P_Get_Raw_Angle_Velocity();
	_micros_delay(100);
	AS5047P_Get_Raw_Angle_Velocity();
}



void AS5047P_Get_Raw_Angle_Velocity_ABZ()
{
	float current_angle = AS5047P_Read_Position_ABZ()*_4000CNT_TO_RAD;
	int32_t d_rotation_cnt = 0;
	unsigned long current_time = _micros();
	float dt = ((float)(current_time - _as5047p.t_velocity_prev))*1e-6f;
	if(dt <= 0 || dt > 0.5f) dt = 1e-4f;

	float d_angle = current_angle - _as5047p.angle_raw;
	if(fabs(d_angle) > (0.8f*_2PI))
	{
		d_rotation_cnt = (d_angle > 0) ? -1 : 1;
	}

	_as5047p.velocity_raw = ((float)(d_rotation_cnt*_2PI + d_angle))/dt;

	_as5047p.angle_raw = current_angle;
	_as5047p.rotation_cnt += d_rotation_cnt;
	_as5047p.t_velocity_prev = current_time;
	_as5047p.angle_raw_full = _as5047p.rotation_cnt*_2PI + _as5047p.angle_raw;
}


void AS5047P_Get_Raw_Angle_Velocity()
{
	float current_angle = AS5047P_Read_Position()*_14BIT_TO_RAD;
	int32_t d_rotation_cnt = 0;
	unsigned long current_time = _micros();
	float dt = ((float)(current_time - _as5047p.t_velocity_prev))*1e-6f;
	if(dt <= 0 || dt > 0.5f) dt = 1e-4f;

	float d_angle = current_angle - _as5047p.angle_raw;
	if(fabs(d_angle) > (0.8f*_2PI))
	{
		d_rotation_cnt = (d_angle > 0) ? -1 : 1;
	}

	_as5047p.velocity_raw = ((float)(d_rotation_cnt*_2PI + d_angle))/dt;

	_as5047p.angle_raw = current_angle;
	_as5047p.rotation_cnt += d_rotation_cnt;
	_as5047p.t_velocity_prev = current_time;
	_as5047p.angle_raw_full = _as5047p.rotation_cnt*_2PI + _as5047p.angle_raw;
}

AS5047P_ErrCode AS5047P_Setting_ABZ()
{
	AS5047P_ErrCode error_code = 0;
	uint16_t setting_1_data = 0;
	uint16_t setting_2_data = 0;

	setting_1_data |= AS5047P_SET1_ROTATION_DIR << 2;
	setting_1_data |= AS5047P_SET1_ABI << 3;
	setting_1_data |= AS5047P_SET1_DAECDIS << 4;
	setting_1_data |= AS5047P_SET1_ABIBIN << 5;
	setting_1_data |= AS5047P_SET1_DATASELECT << 6;
	setting_1_data |= AS5047P_SET1_PWMON << 7;

	error_code = AS5047P_Write_Register(AS5047P_SETTINGS1, setting_1_data);
	if(error_code != 0) return -1;

	setting_2_data |= AS5047P_SET2_UVWPP;
	setting_2_data |= AS5047P_SET2_HYS << 3;
	setting_2_data |= AS5047P_SET2_ABIRES << 5;

	error_code = AS5047P_Write_Register(AS5047P_SETTINGS2, setting_2_data);
	if(error_code != 0) return -1;

	return 0;
}


AS5047P_ErrCode AS5047P_Setting()
{
	AS5047P_ErrCode error_code = 0;
	uint16_t setting_1_data = 0;
	uint16_t setting_2_data = 0;

	setting_1_data |= AS5047P_SET1_ROTATION_DIR << 2;
	setting_1_data |= AS5047P_SET1_UVW << 3;
	setting_1_data |= AS5047P_SET1_DAECDIS << 4;
	setting_1_data |= AS5047P_SET1_ABIBIN << 5;
	setting_1_data |= AS5047P_SET1_DATASELECT << 6;
	setting_1_data |= AS5047P_SET1_PWMON << 7;

	error_code = AS5047P_Write_Register(AS5047P_SETTINGS1, setting_1_data);
	if(error_code != 0) return -1;

	setting_2_data |= AS5047P_SET2_UVWPP;
	setting_2_data |= AS5047P_SET2_HYS << 3;
	setting_2_data |= AS5047P_SET2_ABIRES << 5;

	error_code = AS5047P_Write_Register(AS5047P_SETTINGS2, setting_2_data);
	if(error_code != 0) return -1;

	return 0;
}


AS5047P_ErrCode AS5047P_Set_Factory_Settings()
{
	AS5047P_ErrCode error_code = 0;

	error_code = AS5047P_Write_Register(AS5047P_SETTINGS1, 0x0001);
	error_code |= AS5047P_Write_Register(AS5047P_SETTINGS2, 0x0000);
	error_code |= AS5047P_Write_Register(AS5047P_ZPOSL, 0x000);
	error_code |= AS5047P_Write_Register(AS5047P_ZPOSM, 0x000);
	if(error_code != 0) return -1;
	return 0;
}


AS5047P_Result AS5047P_Read_Position_ABZ()
{
	AS5047P_Result current_position = 0;
	current_position = _as5047p.htim->Instance->CNT;

	return current_position;

}





AS5047P_Result AS5047P_Read_Position()
{
	AS5047P_Result current_position = 0;

	AS5047P_Read_Write_Raw(AS5047P_ANGLECOM, AS5047P_ACCESS_READ);
	current_position = AS5047P_Read_Write_Raw(AS5047P_DIAAGC, AS5047P_ACCESS_READ);
	if(current_position < 0) return -1;
	return current_position;
}



AS5047P_ErrCode AS5047P_Set_Zero_Position()
{
	AS5047P_Result current_position = 0;
	AS5047P_Result current_ZPOSL_content = 0;
	AS5047P_ErrCode error_code = 0;

	uint16_t new_ZPOSL_content = 0;
	uint16_t new_ZPOSM_content = 0;

	uint16_t temp_data = 0;

	current_ZPOSL_content = AS5047P_Read_Register(AS5047P_ZPOSL);
	if(current_ZPOSL_content < 0) return -1;

	temp_data = current_ZPOSL_content & (AS5047P_ZPOSL_COMP_H_ERR_EN | AS5047P_ZPOSL_COMP_I_ERR_EN);
	error_code = AS5047P_Write_Register(AS5047P_ZPOSL, temp_data);
	error_code |= AS5047P_Write_Register(AS5047P_ZPOSM, 0x0000);
	if(error_code != 0) return -1;

	HAL_Delay(1);

	current_position = AS5047P_Read_Register(AS5047P_ANGLECOM);
	if(current_position < 0) return -1;

	new_ZPOSL_content = temp_data | (current_position & AS5047P_ZPOSL_ZPOSL);
	new_ZPOSM_content = (current_position >> 6) & AS5047P_ZPOSM_ZPOSM;

	error_code = AS5047P_Write_Register(AS5047P_ZPOSL, new_ZPOSL_content);
	error_code |= AS5047P_Write_Register(AS5047P_ZPOSM, new_ZPOSM_content);
	if(error_code != 0) return -1;
	return 0;

}

AS5047P_ErrCode Init_ABZ_AS5047P(TIM_HandleTypeDef *htim)
{
	_as5047p.htim = htim;


	_as5047p.angle_raw = 0;
	_as5047p.angle_raw_full = 0;
	_as5047p.velocity_raw = 0;

	_as5047p.rotation_cnt = 0;
	_as5047p.t_velocity_prev = 0;


	HAL_Delay(10);		// AS5047P needs 10ms after power up to recover

	if(HAL_TIM_Encoder_Start(_as5047p.htim, TIM_CHANNEL_ALL) == HAL_OK)
	{

		return 0;
	}
}

AS5047P_ErrCode Init_SPI_AS5047P(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
	_as5047p.hspi = hspi;
	_as5047p.csPort = csPort;
	_as5047p.csPin = csPin;


	_as5047p.angle_raw = 0;
	_as5047p.angle_raw_full = 0;
	_as5047p.velocity_raw = 0;

	_as5047p.rotation_cnt = 0;
	_as5047p.t_velocity_prev = 0;

	AS5047P_Result rx_buff = 0;

	HAL_Delay(10);		// AS5047P needs 10ms after power up to recover

	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

	AS5047P_Read_Register(AS5047P_ERRFL);
	rx_buff = AS5047P_Read_Register(AS5047P_ERRFL);

	if(rx_buff != 0) return -1;
	return 0;
}

AS5047P_ErrCode AS5047P_Write_Register(uint16_t address, uint16_t data_write)
{
	AS5047P_Result rx_buff = 0;

	/************************************************************************************
	* STEP 1: Transmit COMMAND and receive DUMMY RESPONSE.
	* 	     Ignore response and EF flag.
	************************************************************************************/

	AS5047P_Read_Write_Raw(address, AS5047P_ACCESS_WRITE);

	/************************************************************************************
	* STEP 2: Transmit NEW CONTENT and receive PREVIOUS REGISTER CONTENT.
	* 	     Then check if no framing error occurred.
	************************************************************************************/

	rx_buff = AS5047P_Read_Write_Raw(data_write, AS5047P_ACCESS_WRITE);

	if(rx_buff < 0) return -1;
	return 0;
}


AS5047P_Result AS5047P_Read_Register(uint16_t address)
{
	AS5047P_Result rx_buff = 0;


	/************************************************************************************
   * STEP 1: Transmit COMMAND AND receive DUMMY RESPONSE.
   * 	     Ignore response and framing errors.
   ************************************************************************************/
	AS5047P_Read_Write_Raw(address, AS5047P_ACCESS_READ);


	/************************************************************************************
   * STEP 2: Transmit DIAAGC command AND receive COMMAND RESPONSE.
   * 	     DIAAGC register chosen because it's always non-zero value.
   * 	     Then check if no framing error occurred.
   * 	     Then check parity.
   ************************************************************************************/
	rx_buff = AS5047P_Read_Write_Raw(AS5047P_DIAAGC, AS5047P_ACCESS_READ);


	if(rx_buff < 0) return -1;
	return rx_buff;
}


AS5047P_Result AS5047P_Read_Write_Raw(uint16_t data_tx, uint8_t rw_bit)
{
	AS5047P_Result tx_buff = 0;
	AS5047P_Result rx_buff = 0;

	tx_buff = data_tx & AS5047P_FRAME_DATA;		// 14bit mask, 0b0011 1111 1111 1111
	tx_buff |= rw_bit << 14;
	tx_buff |= AS5047P_Calc_Parity(tx_buff) << 15;

	HAL_GPIO_WritePin(_as5047p.csPort, _as5047p.csPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(_as5047p.hspi, (uint8_t *)&tx_buff, (uint8_t *)&rx_buff, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(_as5047p.csPort, _as5047p.csPin, GPIO_PIN_SET);

	return rx_buff & AS5047P_FRAME_DATA;
}






/************************************************************************************
 * 	Function: 	AS5047P_BurnOTP(instance,yesImSure)
 * 	Returns:
 *		-1 : Execution with errors.
 *		 0 : Success.
 *
 * 	Description:
 *	Function writes permanently current content of four registers: SETTINGS1,
 *	SETTINGS2, ZPOSL, ZPOSM. When OTP burn process is finished, registers are
 *	filled with zeros and then refreshed with data written to non-volatile OTP
 *	memory. After that comparison takes place to check if data (old and new) match.
 ************************************************************************************/
/************************************************************************************
*				IMPORTANT!
*
* 	 Remember to cycle the power and then check content of non-volatile registers.
* 	 Only then you can be sure that OTP burn succeeded.
*
************************************************************************************/
AS5047P_ErrCode AS5047P_Burn_OTP()
{
	AS5047P_ErrCode error_code = 0;
	AS5047P_Result rx_data = 0;
	AS5047P_Result old_ZPOSM, old_ZPOSL, old_Setting1, old_Setting2;
	AS5047P_Result new_ZPOSM, new_ZPOSL, new_Setting1, new_Setting2;



	/************************************************************************************
	* STEP 1: Read current contents of ZPOS, ZPOSL, SETTINGS1 and SETTINGS2 registers
	* 	for verification later.
	************************************************************************************/
	old_ZPOSM = AS5047P_Read_Register(AS5047P_ZPOSM);
	old_ZPOSL = AS5047P_Read_Register(AS5047P_ZPOSL);
	old_Setting1 = AS5047P_Read_Register(AS5047P_SETTINGS1);
	old_Setting2 = AS5047P_Read_Register(AS5047P_SETTINGS2);

	if((old_ZPOSL < 0) || (old_ZPOSM < 0) || (old_Setting1 < 0) || (old_Setting2 < 0)) return -1;



	/************************************************************************************
	* STEP 2: Set bit0 of SETTINGS1 to 0.
	* 	Datasheet:
	*  	For programming it's mandatory to clear bit0 (although it's read-only)
	************************************************************************************/
	error_code = AS5047P_Write_Register(AS5047P_SETTINGS1, (~AS5047P_SETTINGS1_BIT0) & old_Setting1);
	if(error_code != 0) return -1;


	/************************************************************************************
	* STEP 3: Unlock OTParea for burning
	* 	(PROGEN=1)
	************************************************************************************/
	error_code = AS5047P_Write_Register(AS5047P_PROG, AS5047P_PROG_PROGEN);
	if(error_code != 0) return -1;


	/************************************************************************************
	* STEP 4: Start OTP burning procedure
	* 	(PROGOTP=1)
	************************************************************************************/
	error_code = AS5047P_Write_Register(AS5047P_PROG, AS5047P_PROG_PROGOTP);
	if(error_code != 0) return -1;


	/************************************************************************************
	* STEP 5: Wait for OTP burning procedure complete.
	* 	Reg(0x0003) == 0x01
	************************************************************************************/
	do
	{
		rx_data = AS5047P_Read_Register(AS5047P_PROG);
		if(rx_data < 0) return -1;

		if(rx_data == 0x0001) break;

		HAL_Delay(1);
	}while(1);



	/************************************************************************************
	* STEP 6: Fill ZPOS, ZPOSL, SETTINGS1 and SETTINGS2 registers with zeros.
	************************************************************************************/
	error_code = AS5047P_Write_Register(AS5047P_ZPOSM, 0x0000);
	error_code |= AS5047P_Write_Register(AS5047P_ZPOSL, 0x0000);
	error_code |= AS5047P_Write_Register(AS5047P_SETTINGS1, 0x0000);
	error_code |= AS5047P_Write_Register(AS5047P_SETTINGS2, 0x0000);
	if(error_code != 0) return -1;


	/************************************************************************************
	* STEP 7: Set Guardband
	************************************************************************************/
	error_code = AS5047P_Write_Register(AS5047P_PROG, AS5047P_PROG_PROGVER);
	if(error_code != 0) return -1;


	/************************************************************************************
	* STEP 8: Refresh non-volatile memory with OTP content
	************************************************************************************/
	error_code = AS5047P_Write_Register(AS5047P_PROG, AS5047P_PROG_OTPREF);
	if(error_code != 0) return -1;

	HAL_Delay(1);


	/************************************************************************************
	* STEP 9: Read current contents of ZPOS, ZPOSL, SETTINGS1 and SETTINGS2 registers
	************************************************************************************/
	new_ZPOSM = AS5047P_Read_Register(AS5047P_ZPOSM);
	new_ZPOSL = AS5047P_Read_Register(AS5047P_ZPOSL);
	new_Setting1 = AS5047P_Read_Register(AS5047P_SETTINGS1);
	new_Setting2 = AS5047P_Read_Register(AS5047P_SETTINGS2);
	if((old_ZPOSL < 0) || (old_ZPOSM < 0) || (old_Setting1 < 0) || (old_Setting2 < 0)) return -1;


	/************************************************************************************
	* STEP 10: Final comparison: old to new.
	* 	 Mask out bit0 in register SETTINGS1.
	************************************************************************************/
	if((old_ZPOSM != new_ZPOSM) || (old_ZPOSL != new_ZPOSL) ||
			( ((~AS5047P_SETTINGS1_BIT0) & old_Setting1) != ((~AS5047P_SETTINGS1_BIT0) & new_Setting1) ) ||
			(old_Setting2 != new_Setting2)) return -1;

	return 0;
}

