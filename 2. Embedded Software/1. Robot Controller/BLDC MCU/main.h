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
#define BLDC_Motor_nSLEEP_GPIO_Output_Pin GPIO_PIN_14
#define BLDC_Motor_nSLEEP_GPIO_Output_GPIO_Port GPIOC
#define BLDC_Motor_nFAULT_GPIO_Input_Pin GPIO_PIN_15
#define BLDC_Motor_nFAULT_GPIO_Input_GPIO_Port GPIOC
#define Current_A_ADC3_IN0_Pin GPIO_PIN_2
#define Current_A_ADC3_IN0_GPIO_Port GPIOC
#define Current_B_ADC3_IN1_Pin GPIO_PIN_3
#define Current_B_ADC3_IN1_GPIO_Port GPIOC
#define BLDC_Motor_PWM_A_TIM2_CH1_Pin GPIO_PIN_0
#define BLDC_Motor_PWM_A_TIM2_CH1_GPIO_Port GPIOA
#define BLDC_Motor_PWM_B_TIM2_CH2_Pin GPIO_PIN_1
#define BLDC_Motor_PWM_B_TIM2_CH2_GPIO_Port GPIOA
#define BLDC_Motor_PWM_C_TIM2_CH3_Pin GPIO_PIN_2
#define BLDC_Motor_PWM_C_TIM2_CH3_GPIO_Port GPIOA
#define Mag_Encoder_SPI1_nCS_GPIO_Output_Pin GPIO_PIN_4
#define Mag_Encoder_SPI1_nCS_GPIO_Output_GPIO_Port GPIOA
#define Mag_Encoder_SPI1_SCK_Pin GPIO_PIN_5
#define Mag_Encoder_SPI1_SCK_GPIO_Port GPIOA
#define Mag_Encoder_SPI1_MISO_Pin GPIO_PIN_6
#define Mag_Encoder_SPI1_MISO_GPIO_Port GPIOA
#define Mag_Encoder_SPI1_MOSI_Pin GPIO_PIN_7
#define Mag_Encoder_SPI1_MOSI_GPIO_Port GPIOA
#define Mag_Encoder_A_TIM1_CH1_Pin GPIO_PIN_9
#define Mag_Encoder_A_TIM1_CH1_GPIO_Port GPIOE
#define Mag_Encoder_A_TIM1_CH2_Pin GPIO_PIN_11
#define Mag_Encoder_A_TIM1_CH2_GPIO_Port GPIOE
#define BLDC_Data_SPI2_SCK_Pin GPIO_PIN_13
#define BLDC_Data_SPI2_SCK_GPIO_Port GPIOB
#define BLDC_Data_SPI2_GPIO_Pin GPIO_PIN_14
#define BLDC_Data_SPI2_GPIO_GPIO_Port GPIOB
#define BLDC_Data_SPI2_MOSI_Pin GPIO_PIN_15
#define BLDC_Data_SPI2_MOSI_GPIO_Port GPIOB
#define BLDC_CMD_UART9_RX_Pin GPIO_PIN_14
#define BLDC_CMD_UART9_RX_GPIO_Port GPIOD
#define BLDC_CMD_UART9_TX_Pin GPIO_PIN_15
#define BLDC_CMD_UART9_TX_GPIO_Port GPIOD
#define LED_GPIO_Output_Pin GPIO_PIN_12
#define LED_GPIO_Output_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
