/*
 * bldc_motor_communication_main.c
 *
 *  Created on: Jun 10, 2024
 *      Author: BSM
 */

#include <bldc_motor_communication_driver.h>


_BLDC_CMD _bldc_cmd;
_BLDC_DATA _bldc_data;

void Init_BLDC_Data_SPI_Driver(SPI_HandleTypeDef *hspi)//, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
	_bldc_data.hspi = hspi;
//	_bldc_data.cs_port = cs_port;
//	_bldc_data.cs_pin = cs_pin;



	for(uint8_t i = 0; i < BLDC_SPI_DATA_SIZE; i++)
	{
		_bldc_data.bldc_data[i] = 0;
	}

	/**
	  ***************************************
	  * test float constants
	  ***************************************
	  *	[0  -  1]  : start bytes
	  * [2  -  5]  : position raw
	  * [6  -  9]  : position filtered
	  * [10 - 13]  : velocity raw
	  * [14 - 17]  : velocity filtered
	  * [18 - 21]  : current dc raw
	  * [22 - 25]  : current dc filtered
	  * [26 - 29]  : current a raw
	  * [30 - 33]  : current a filtered
	  * [34 - 37]  : current b raw
	  * [38 - 41]  : current b filtered
	  */

}

void Init_BLDC_Data_SPI_Main(SPI_HandleTypeDef *hspi)//, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
	_bldc_data.hspi = hspi;
//	_bldc_data.cs_port = cs_port;
//	_bldc_data.cs_pin = cs_pin;

	for (uint8_t i = 0; i < BLDC_SPI_DATA_SIZE; i++)
	{
		_bldc_data.bldc_data[i] = 0;
		_bldc_data.bldc_data_process[i] = 0;
	}

	//memset(_bldc_data.bldc_data, 0, sizeof(_bldc_data.bldc_data));
	//memset(_bldc_data.bldc_data_process, 0, sizeof(_bldc_data.bldc_data_process));

}

void Float_to_Byte_Array(volatile float float_value, volatile uint8_t *byte_array)
{
	uint8_t *float_pointer = (uint8_t*)&float_value;
	uint8_t i = 0;

	for (i = 0; i < 4; i++)
	{
		byte_array[i] = float_pointer[i];
	}
}



void Form_BLDC_DATA_TX_Buff()
{
	_bldc_data.bldc_data[0] = START_BYTE_1;
	_bldc_data.bldc_data[1] = START_BYTE_2;


	/**
	  ***************************************
	  * test float constants
	  ***************************************
	  *	[1]  : 3.1415926
	  * [2]  : 2.7182818
	  * [3]	 : 1.4142135
	  * [4]  : 0
	  * [5]  : 1
	  * [6]  : -1
	  * [7]  : -31415.926
	  * [8]  : -2718281.8
	  * [9]  : -14.142135
	  * [10] : 100.001
	  */
/*
	Float_to_Byte_Array(3.1415926, _bldc_data.bldc_data + 2);
	Float_to_Byte_Array(2.7182818, _bldc_data.bldc_data + 6);
	Float_to_Byte_Array(1.4142135, _bldc_data.bldc_data + 10);
	Float_to_Byte_Array(0, _bldc_data.bldc_data + 14);
	Float_to_Byte_Array(1, _bldc_data.bldc_data + 18);
	Float_to_Byte_Array(-1, _bldc_data.bldc_data + 22);
	Float_to_Byte_Array(-31415.926, _bldc_data.bldc_data + 26);
	Float_to_Byte_Array(-2718281.8, _bldc_data.bldc_data + 30);
	Float_to_Byte_Array(-14.142135, _bldc_data.bldc_data + 34);
	Float_to_Byte_Array(100.001, _bldc_data.bldc_data + 38);
*/
	Float_to_Byte_Array(_drv8311_current.i_d, _bldc_data.bldc_data + 2);
	Float_to_Byte_Array(_lpf_current_d.filter_out_prev, _bldc_data.bldc_data + 6);
	Float_to_Byte_Array(_drv8311_current.i_q, _bldc_data.bldc_data + 10);
	Float_to_Byte_Array(_lpf_current_q.filter_out_prev, _bldc_data.bldc_data + 14);
	Float_to_Byte_Array(_as5047p.angle_raw_full, _bldc_data.bldc_data + 18);
	Float_to_Byte_Array(_lpf_angle.filter_out_prev, _bldc_data.bldc_data + 22);
	Float_to_Byte_Array(_as5047p.velocity_raw, _bldc_data.bldc_data + 26);
	Float_to_Byte_Array(_lpf_velocity.filter_out_prev, _bldc_data.bldc_data + 30);
	Float_to_Byte_Array(_bldc_motor.angle_electrical, _bldc_data.bldc_data + 34);
	Float_to_Byte_Array(_bldc_motor.target_current_d, _bldc_data.bldc_data + 38);
	Float_to_Byte_Array(_bldc_motor.target_current_q, _bldc_data.bldc_data + 42);
	Float_to_Byte_Array(_bldc_motor.target_angle, _bldc_data.bldc_data + 46);
	Float_to_Byte_Array(_bldc_motor.target_velocity, _bldc_data.bldc_data + 50);
	Float_to_Byte_Array(_bldc_pid_current_d.out_prev, _bldc_data.bldc_data + 54);
	Float_to_Byte_Array(_bldc_pid_current_q.out_prev, _bldc_data.bldc_data + 58);
	Float_to_Byte_Array(_bldc_pid_position.out_prev, _bldc_data.bldc_data + 62);
	Float_to_Byte_Array(_bldc_pid_velocity.out_prev, _bldc_data.bldc_data + 66);

}

void Shift_data_by_one_bit()
{
	uint8_t carry = 0;
	uint8_t temp[BLDC_SPI_DATA_SIZE] = {0};

	for(uint8_t i = 0; i < BLDC_SPI_DATA_SIZE; i++)
	{
		temp[i] = _bldc_data.bldc_data_process[i];
	}

	for(uint8_t i = 0; i < BLDC_SPI_DATA_SIZE; i++)
	{
		carry = (temp[(i + 1) % BLDC_SPI_DATA_SIZE] & 0x80) >> 7;
		uint8_t current = temp[i];
		_bldc_data.bldc_data_process[i] = (current << 1) | carry;

	}
}

void Prepare_BLDC_data_process()
{
	uint8_t i = 0;

	while (i < BLDC_SPI_DATA_SIZE)
	{
		_bldc_data.bldc_data_process[i] = _bldc_data.bldc_data[i];
		i++;
	}
}

uint8_t Check_Start_Bytes()
{
	uint8_t start_index = 0;
	uint8_t index_found = 0;
	uint8_t bit_shift_cnt = 0;





	//memcpy(_bldc_data.bldc_data_process, _bldc_data.bldc_data, sizeof(uint8_t)*42);
	while(!index_found)
	{

		if(bit_shift_cnt == 0)
		{
			Prepare_BLDC_data_process();
		}

		if(_bldc_data.bldc_data_process[start_index] == 0xAA)
		{
			if(_bldc_data.bldc_data_process[(start_index + 1) % BLDC_SPI_DATA_SIZE] == 0xBB)
			{
				index_found = 1;


			}
		}

		if ((start_index == BLDC_SPI_DATA_SIZE - 1) && !index_found)
		{

			Shift_data_by_one_bit();
			bit_shift_cnt = (bit_shift_cnt + 1) % 8;

		}

		start_index = (start_index + 1) % BLDC_SPI_DATA_SIZE;
	}




	return (start_index + 1) % BLDC_SPI_DATA_SIZE;
}

float Byte_Array_to_Float(uint8_t *byte_array)
{
	float float_value;
	uint8_t *float_pointer = (uint8_t *) &float_value;

	for(uint8_t i = 0; i < 4; i++)
	{
		float_pointer[i] = byte_array[i];
	}

	return float_value;
}

void Process_BLDC_Data_RX(){
	uint8_t start_index = Check_Start_Bytes();

	for(uint8_t i = 0; i < BLDC_SPI_DATA_NUM; i++)
	{
		_bldc_data.bldc_data_float[i] = Byte_Array_to_Float(_bldc_data.bldc_data_process + (start_index + 4*i) % BLDC_SPI_DATA_SIZE);
	}
}

void Start_BLDC_TX_SPI_DMA()
{
//	HAL_GPIO_WritePin(_bldc_data.cs_port, _bldc_data.cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(_bldc_data.hspi, (uint8_t *)_bldc_data.bldc_data, BLDC_SPI_DATA_SIZE);
}

void Start_BLDC_RX_SPI_DMA()
{
	HAL_SPI_Receive_DMA(_bldc_data.hspi, (uint8_t *)_bldc_data.bldc_data, BLDC_SPI_DATA_SIZE);
}

void Init_BLDC_CMD_UART(UART_HandleTypeDef *huart)
{
	_bldc_cmd.huart = huart;
	_bldc_cmd.ref_cmd[0] = 0xAA;
	_bldc_cmd.ref_cmd[1] = 0xBB;
	_bldc_cmd.ref_cmd[2] = 0;
	_bldc_cmd.ref_cmd[3] = 0;
	_bldc_cmd.ref_cmd[4] = 0;

}


void Start_BLDC_CMD_UART_TX_DMA()
{
	HAL_UART_Transmit_DMA(_bldc_cmd.huart, _bldc_cmd.ref_cmd, 5);
}

void Start_BLDC_CMD_UART_RX_DMA()
{
	HAL_UART_Receive_DMA(_bldc_cmd.huart, _bldc_cmd.ref_cmd, 5);
}

void Off_BLDC_CMD()
{
	_bldc_cmd.ref_cmd[0] = 0xAA;
	_bldc_cmd.ref_cmd[1] = 0xBB;
	_bldc_cmd.ref_cmd[2] = 0;
	_bldc_cmd.ref_cmd[3] = 0;
	_bldc_cmd.ref_cmd[4] = 0;
}
