/*
 * nrf_uart_hc12_bsm.h
 *
 *  Created on: Mar 14, 2024
 *      Author: BSM
 */

#ifndef INC_NRF_UART_HC12_BSM_H_
#define INC_NRF_UART_HC12_BSM_H_


#include "main.h"
#include "stm32h7xx_hal.h"





//**	Robot Status Macros		**//
#define RF_CONNECTING		0x00
#define RF_SET				0x01
#define SENSOR_CONNECTING	0x02



typedef struct{
	UART_HandleTypeDef *huart;
	volatile uint8_t cntr_status;

	volatile uint8_t dip_state;
	volatile uint8_t tact_state;
	volatile uint8_t joy_x;
	volatile uint8_t joy_y;
	volatile uint8_t pot;

} _ROBOT_UART;

extern _ROBOT_UART _robot_uart;

void Convert_RX_to_User_Input(uint8_t* rx_data);
void Check_RF_Connection();
void Init_Robot_UART(UART_HandleTypeDef* huart);
#endif /* INC_NRF_UART_HC12_BSM_H_ */
