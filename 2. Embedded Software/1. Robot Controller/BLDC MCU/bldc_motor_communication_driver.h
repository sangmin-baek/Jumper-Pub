/*
 * bldc_motor_communication_main.h
 *
 *  Created on: Jun 10, 2024
 *      Author: BSM
 */

#ifndef INC_BLDC_MOTOR_COMMUNICATION_DRIVER_H_
#define INC_BLDC_MOTOR_COMMUNICATION_DRIVER_H_

#include "stm32h7xx_hal.h"
#include "string.h"


#include "bldc_motor_control_bsm.h"


#define BLDC_SPI_DATA_NUM	17
#define BLDC_SPI_DATA_SIZE	(2 + BLDC_SPI_DATA_NUM * 4)

#define START_BYTE_1 0xAA
#define START_BYTE_2 0xBB




typedef struct{
	UART_HandleTypeDef *huart;
	volatile uint8_t ref_cmd[5];
} _BLDC_CMD;

typedef struct{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
	volatile uint8_t bldc_data[BLDC_SPI_DATA_SIZE];
	volatile uint8_t bldc_data_process[BLDC_SPI_DATA_SIZE];
	float bldc_data_float[BLDC_SPI_DATA_NUM];
} _BLDC_DATA;

extern _BLDC_CMD _bldc_cmd;
extern _BLDC_DATA _bldc_data;

void Init_BLDC_Data_SPI_Driver(SPI_HandleTypeDef *hspi);
void Init_BLDC_Data_SPI_Main(SPI_HandleTypeDef *hspi);
void Float_to_Byte_Array(volatile float float_value, volatile uint8_t *byte_array);
void Form_BLDC_DATA_TX_Buff();

uint8_t Check_Start_Bytes();
float Byte_Array_to_Float(uint8_t *byte_array);
void Process_BLDC_Data_RX();
void Shift_data_by_one_bit();
void Prepare_BLDC_data_process();

void Start_BLDC_TX_SPI_DMA();
void Start_BLDC_RX_SPI_DMA();
void Init_BLDC_CMD_UART(UART_HandleTypeDef *huart);
void Read_BLDC_SPI();
void Start_BLDC_CMD_UART_TX_DMA();
void Start_BLDC_CMD_UART_RX_DMA();
void Off_BLDC_CMD();


#endif /* INC_BLDC_MOTOR_COMMUNICATION_DRIVER_H_ */
