/*
 * nrf_uart_hc12_bsm.c
 *
 *  Created on: Mar 14, 2024
 *      Author: BSM
 */
#include "nrf_uart_hc12_bsm.h"

_ROBOT_UART _robot_uart;

void Convert_RX_to_User_Input(uint8_t* rx_data)
{
	_robot_uart.dip_state = rx_data[0];
	_robot_uart.tact_state = rx_data[1];
	_robot_uart.joy_x = rx_data[2];
	_robot_uart.joy_y = rx_data[3];
	_robot_uart.pot = rx_data[4];
}

void Check_RF_Connection()
{
	uint8_t tx_data = 0;
	uint8_t rx_data = 0;

	while(_robot_uart.cntr_status == RF_CONNECTING)
	{
		HAL_UART_Receive(_robot_uart.huart, &rx_data, 1, 100);

		if(rx_data == 0x0F)
		{
			_robot_uart.cntr_status = RF_SET;
		}
		else
		{
			tx_data = rx_data;
			HAL_UART_Transmit(_robot_uart.huart, &tx_data, 1, 100);
		}

		HAL_Delay(10);
	}

	while(_robot_uart.cntr_status == RF_SET)
	{
		HAL_UART_Receive(_robot_uart.huart, &rx_data, 1, 100);

		if(rx_data == 0x0F)
		{
			tx_data = 0x0F;
			HAL_UART_Transmit(_robot_uart.huart, &tx_data, 1, 100);
			_robot_uart.cntr_status = SENSOR_CONNECTING;
		}

		HAL_Delay(10);
	}
}

void Init_Robot_UART(UART_HandleTypeDef* huart)
{
	_robot_uart.huart = huart;
	_robot_uart.cntr_status = 0;

	_robot_uart.dip_state = 0;
	_robot_uart.tact_state = 0;
	_robot_uart.joy_x = 0;
	_robot_uart.joy_y = 0;
	_robot_uart.pot = 0;
}
