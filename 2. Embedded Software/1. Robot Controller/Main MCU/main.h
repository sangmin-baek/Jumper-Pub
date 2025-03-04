/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DC_Motor_PWM_A_TIM15_CH1_Pin GPIO_PIN_5
#define DC_Motor_PWM_A_TIM15_CH1_GPIO_Port GPIOE
#define DC_Motor_PWM_B_TIM15_CH2_Pin GPIO_PIN_6
#define DC_Motor_PWM_B_TIM15_CH2_GPIO_Port GPIOE
#define BLDC_CMD_UART4_TX_Pin GPIO_PIN_0
#define BLDC_CMD_UART4_TX_GPIO_Port GPIOA
#define BLDC_CMD_UART4_RX_Pin GPIO_PIN_1
#define BLDC_CMD_UART4_RX_GPIO_Port GPIOA
#define BLDC_Data_SPI1_SCK_Pin GPIO_PIN_5
#define BLDC_Data_SPI1_SCK_GPIO_Port GPIOA
#define DC_Angle_ADC1_INP5_Pin GPIO_PIN_1
#define DC_Angle_ADC1_INP5_GPIO_Port GPIOB
#define RF_UART7_RX_Pin GPIO_PIN_7
#define RF_UART7_RX_GPIO_Port GPIOE
#define RF_UART7_TX_Pin GPIO_PIN_8
#define RF_UART7_TX_GPIO_Port GPIOE
#define IMU_SPI2_nCS_GPIO_Output_Pin GPIO_PIN_12
#define IMU_SPI2_nCS_GPIO_Output_GPIO_Port GPIOB
#define IMU_SPI2_SCK_Pin GPIO_PIN_13
#define IMU_SPI2_SCK_GPIO_Port GPIOB
#define IMU_SPI2_MISO_Pin GPIO_PIN_14
#define IMU_SPI2_MISO_GPIO_Port GPIOB
#define IMU_SPI2_MOSI_Pin GPIO_PIN_15
#define IMU_SPI2_MOSI_GPIO_Port GPIOB
#define LED_GPIO_Output_Pin GPIO_PIN_11
#define LED_GPIO_Output_GPIO_Port GPIOD
#define SDMMC1_CD_GPIO_Input_Pin GPIO_PIN_7
#define SDMMC1_CD_GPIO_Input_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
