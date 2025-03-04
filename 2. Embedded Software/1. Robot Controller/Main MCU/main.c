/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/**
  ******************************************************************************
  * Overview
  ******************************************************************************
  * RF communication		: UART7 - DMA	: 100 Hz	: Baud rate 9600 Bit/s
  *
  * DC motor angle read 	: ADC1 - DMA	: 500 Hz	:
  * DC motor control		: PWM - TIM15	: 500 Hz	:
  *
  * BLDC position command	: UART4 - DMA	: 500 Hz	:
  * BLDC data read			: SPI1 - DMA	: 500 Hz	:
  *
  * IMU read 				: SPI2			: 500 Hz	:
  * uSD write 				: SDMMC			: 500 Hz	: 4bit, clk divider 5
  ******************************************************************************
  */


/**
  ******************************************************************************
  * Timer info
  ******************************************************************************
  * Tim5	: micro second counter
  * Tim7	: Main control loop
  * Tim15	: DC motor PWM
  *
  ******************************************************************************
  */


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/**
  ***************************************
  * C Standard Libs
  ***************************************
  */
#include <stdio.h>
#include <string.h>

/**
  ***************************************
  * BSM Custom Libs
  ***************************************
  */
#include "math_utils_bsm.h"
#include "ism330dhcx_spi_BSM.h"
//#include "mpu9250_spi_bsm_v2.h"
#include "nrf_uart_hc12_bsm.h"
#include "bldc_motor_communication_main.h"
#include "file_handle_bsm.h"
#include "biarticular_jumper_main_control_bsm.h"
#include "hr8833_motor_drive.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/**
  ***************************************
  * Control Loop Related Macros
  ***************************************
  */
#define USER_INPUT_READ_PERIOD		5

#define DC_ANGLE_READ_PERIOD		1
#define DC_MOTOR_CONTROL_PERIOD		1

#define BLDC_COMMAND_PERIOD			1
#define BLDC_DATA_READ_PERIOD		1

#define IMU_READ_PERIOD				1
#define uSD_WRITE_PERIOD			1

#define ROBOT_STATE_CHECK_PERIOD	1




/**
  ***************************************
  * IMU Related Macros
  ***************************************
  */
#define IMU_CALIBRATE 0			// IMU Calibration Process Flag [1 if want to calibrate]


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart7_tx;

/* USER CODE BEGIN PV */

/**
  ***************************************
  * Control Loop Related Variables
  ***************************************
  */
volatile uint8_t tim7_flag = 0;
volatile uint32_t tim7_cnt = 0;

volatile uint16_t user_input_read_cnt = 0;

volatile uint8_t dc_angle_read_cnt = 0;
volatile uint8_t dc_motor_ctrl_cnt = 0;

volatile uint8_t bldc_cmd_cnt = 0;
volatile uint8_t bldc_data_read_cnt = 0;

volatile uint8_t imu_read_cnt = 0;
volatile uint8_t usd_write_cnt = 0;

volatile uint8_t robot_state_check_cnt = 0;




/**
  ***************************************
  * UART HC12 Related Variables
  ***************************************
  */
volatile uint8_t uart_rx_dma[10] = {0};

/**
  ***************************************
  * Robot Init Related Variables
  ***************************************
  */
volatile uint8_t imu_setup = 0;
volatile uint8_t usd_setup = 0;
volatile uint8_t usd_not_open = 1;



/**
  ***************************************
  * UART4 BLDC CMD Related Variables
  ***************************************
  */


/**
  ***************************************
  * SPI1 BLDC date Related Variables
  ***************************************
  */


/**
  ***************************************
  * IMU Related Variables
  ***************************************
  */
volatile uint8_t imu_temp_data[21] = {0};

/**
  ***************************************
  * uSD Related Variables
  ***************************************
  */

char sprintf_robot_data[500];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART7_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM15_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_UART7_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_TIM15_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /**
    ***************************************
    * 0. SDMMC BSP init
    ***************************************
    */
  BSP_SD_Init();


  /**
    ***************************************
    * 1. User Input UART Init & RF connection Check
    ***************************************
    */
  Init_Robot_UART(&huart7);
  Check_RF_Connection();
  HAL_UART_Receive_DMA(&huart7, uart_rx_dma, 5);

  /**
    ***************************************
    * 2. IMU Init & Calibration & Check
    ***************************************
    */

  Init_SPI_ISM330DHCX(&hspi2, IMU_SPI2_nCS_GPIO_Output_GPIO_Port, IMU_SPI2_nCS_GPIO_Output_Pin);
  Read_Bytes_SPI_ISM330DHCX(0x01, 2, imu_temp_data);
  Read_Bytes_SPI_ISM330DHCX(0x07, 19, imu_temp_data + 2);
  Reset_ISM330DHCX();
  Set_ISM330DHCX();



/*
  Init_SPI_MPU9250(&hspi2, IMU_SPI2_nCS_GPIO_Out_GPIO_Port, IMU_SPI2_nCS_GPIO_Out_Pin);
  while (imu_setup != 1)
  {
	  imu_setup = Setup_IMU();
	  HAL_Delay(10);
  }
  if (IMU_CALIBRATE) Calibrate_MPU9250(&huart4);
  else Apply_Calibrated_Val_MPU9250();
*/


  /**
    ***************************************
    * 3. uSD Init & Check
    ***************************************
    */

  while(usd_setup != 1)
  {
	  usd_setup = CHECK_uSD_Connection();
  }
  while(usd_not_open)
  {
	  if(Open_File() == FR_OK) usd_not_open = 0;
  }
  Init_BLDC_Data_uSD();

  /**
    ***************************************
    * 4. DC Angle Init & Calibration
    ***************************************
    */

  /**
    ***************************************
    * 5. DC Motor Init & Check
    ***************************************
    */
  Init_Tail_Motor(&htim15);

  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);


  /**
    ***************************************
    * 6. BLDC Command Init & Check
    ***************************************
    */
  Init_BLDC_CMD_UART(&huart4);
  Start_BLDC_CMD_UART_TX_DMA();


  /**
    ***************************************
    * 7. BLDC Read Init & Check
    ***************************************
    */
  Init_BLDC_Data_SPI_Main(&hspi1);//, BLDC_Read_SPI1_nCS_GPIO_Out_GPIO_Port, BLDC_Read_SPI1_nCS_GPIO_Out_Pin);
  Start_BLDC_RX_SPI_DMA();




  /**
    ***************************************
    * F. Main loop IT on
    ***************************************
    */
  HAL_TIM_Base_Start_IT(&htim7);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(tim7_flag == 1)
	  {
		  tim7_flag = 0;


		  HAL_GPIO_TogglePin(LED_GPIO_Output_GPIO_Port, LED_GPIO_Output_Pin);


		  /**
		    ***************************************
		    * 1. Cnt
		    ***************************************
		    */

		  tim7_cnt++;

		  user_input_read_cnt++;

		  dc_angle_read_cnt++;
		  dc_motor_ctrl_cnt++;

		  bldc_cmd_cnt++;
		  bldc_data_read_cnt++;

		  imu_read_cnt++;
		  usd_write_cnt++;

		  robot_state_check_cnt++;


		  /**
		    ***************************************
		    * 2. User input read
		    ***************************************
		    */

		  if(user_input_read_cnt == USER_INPUT_READ_PERIOD)
		  {
			  user_input_read_cnt = 0;
			  Convert_RX_to_User_Input(uart_rx_dma);
		  }



		  /**
		    ***************************************
		    * 3. BLDC CMD UART
		    ***************************************
		    */
		  if (bldc_cmd_cnt == BLDC_COMMAND_PERIOD)
		  {
			  bldc_cmd_cnt = 0;
			  _bldc_cmd.ref_cmd[0] = 0xAA;
			  _bldc_cmd.ref_cmd[1] = 0xBB;
			  _bldc_cmd.ref_cmd[2] = _robot_uart.dip_state;
			  _bldc_cmd.ref_cmd[3] = _robot_uart.tact_state;
			  _bldc_cmd.ref_cmd[4] = _robot_uart.pot;
		  }


		  /**
		    ***************************************
		    * 4. BLDC data SPI
		    ***************************************
		    */
		  if (bldc_data_read_cnt == BLDC_DATA_READ_PERIOD)
		  {
			  bldc_data_read_cnt = 0;
			  Process_BLDC_Data_RX();

		  }



		  /**
		    ***************************************
		    * 5. Check Robot State
		    ***************************************
		    */
		  if(robot_state_check_cnt == ROBOT_STATE_CHECK_PERIOD)
		  {
			  robot_state_check_cnt = 0;
			  Check_Robot_State(tim7_cnt);
		  }



		  /**
		    ***************************************
		    * DC motor angle read
		    ***************************************
		    */


		  /**
		    ***************************************
		    * IMU Read
		    ***************************************
		    */

		  if(imu_read_cnt == IMU_READ_PERIOD)
		  {
			  imu_read_cnt = 0;
			  Get_IMU_Data_ISM330DHCX();
			  //Switch_Attitude_estimator();

			  if(robot_state == 1)
			  {
				  Copy_Attitude_Init_Val();
			  }

			  if(robot_state == 2)
			  {
				  IMU_Attitude_from_Gyro_Int_Pitch();
			  }

			  if(robot_state == 21)
			  {
				  Copy_Attitude_Init_Val();
			  }

			  if(robot_state == 22)
			  {
				  IMU_Attitude_from_Gyro_Int_Pitch();
			  }

			  IMU_Filter_Madgwick();
			  Quaternion_To_YPR_zyx();
		  }
		  /*
		  if (imu_read_cnt == IMU_READ_PERIOD)
		  {
			  Get_Filtered_RPY();
			  imu_read_cnt = 0;
		  }
		  */

		  /**
		    ***************************************
		    * DC Motor Control
		    ***************************************
		    */
		  if(dc_motor_ctrl_cnt == DC_MOTOR_CONTROL_PERIOD)
		  {
			  dc_motor_ctrl_cnt = 0;
			  //Tail_Motor_Run(((int16_t)_robot_uart.pot - 128)*3);

			  if(robot_state == 1)
			  {
				  Tail_Pitch_Control_Init();
				  _tail_motor.pitch_target = _imu_filter.pitch_gyro_int;
				  Tail_Gyro_Regulator(0.0);
			  }
			  if(robot_state == 2)
			  {
				  Tail_Gyro_Regulator_Init();
				  //Tail_Pitch_Control(1.3);
				  Tail_Pitch_Control(_tail_motor.pitch_target - 0.17);
				  //Tail_Motor_Run(500);
			  }
			  if(robot_state == 0)
			  {
				  Tail_Gyro_Regulator_Init();
				  Tail_Pitch_Control_Init();
				  Tail_Motor_Stop();
			  }

			  if(robot_state == 21)
			  {
				  Tail_Pitch_Control_Init();
				  _tail_motor.pitch_target = _imu_filter.pitch_gyro_int;
				  //Tail_Gyro_Regulator(0.0);
			  }
			  if(robot_state == 22)
			  {
				  Tail_Gyro_Regulator_Init();
				  //Tail_Pitch_Control(1.3);
				  Tail_Pitch_Control(_tail_motor.pitch_target - 0.17);
				  //Tail_Motor_Run(500);
			  }

		  }


		  /**
		    ***************************************
		    * uSD write
		    ***************************************
		    */
		  if(usd_write_cnt == uSD_WRITE_PERIOD)
		  {
			  usd_write_cnt = 0;

			  /*
			  sprintf(sprintf_robot_data, "%lu	raw current d	%.4f	filt current d	%.4f	raw current q	%.4f	filt current w	%.4f	"
					  "raw angle	%.4f	filt angle	%.4f	raw velocity	%.4f	filt velocity	%.4f	elec angle	%.4f	"
					  "target current d	%.4f	target current q	%.4f	target angle	%.4f	target velocity	%.4f	"
					  "pid current d	%.4f	pid current q	%.4f	pid angle	%.4f	pid velocity	%.4f	"
					  "a_x	%.4f	a_y	%.4f	a_z	%.4f	g_x	%.4f	g_y	%.4f	g_z	%.4f	"
					  "roll	%.4f	pitch	%.4f	yaw	%.4f	roll_g	%.4f	pitch_g	%.4f	yaw_g	%.4f\r\n",
			          tim7_cnt, _bldc_data_uSD.uSD_data[0],  _bldc_data_uSD.uSD_data[1], _bldc_data_uSD.uSD_data[2], _bldc_data_uSD.uSD_data[3],
			          _bldc_data_uSD.uSD_data[4], _bldc_data_uSD.uSD_data[5], _bldc_data_uSD.uSD_data[6], _bldc_data_uSD.uSD_data[7], _bldc_data_uSD.uSD_data[8],
			          _bldc_data_uSD.uSD_data[9], _bldc_data_uSD.uSD_data[10], _bldc_data_uSD.uSD_data[11], _bldc_data_uSD.uSD_data[12],
			          _bldc_data_uSD.uSD_data[13], _bldc_data_uSD.uSD_data[14], _bldc_data_uSD.uSD_data[15], _bldc_data_uSD.uSD_data[16],
			          _imu_data.accel.x, _imu_data.accel.y, _imu_data.accel.z, _imu_data.gyro.x, _imu_data.gyro.y, _imu_data.gyro.z,
			          _imu_filter.roll, _imu_filter.pitch, _imu_filter.yaw, _imu_filter.roll_from_gyro, _imu_filter.pitch_from_gyro, _imu_filter.yaw_from_gyro);
*/
			  sprintf(sprintf_robot_data, "%lu	raw current d	%.4f	filt current d	%.4f	raw current q	%.4f	filt current w	%.4f	"
					  "raw angle	%.4f	filt angle	%.4f	raw velocity	%.4f	filt velocity	%.4f	elec angle	%.4f	"
			          "q_0	%.4f	q_1	%.4f	q_2	%.4f	q_3	%.4f	"
			          "a_x	%.4f	a_y	%.4f	a_z	%.4f	g_x	%.4f	g_y	%.4f	g_z	%.4f	"
					  "roll	%.4f	pitch	%.4f	yaw	%.4f	"
			          "pitch_gyro	%.4f	"
					  "robot state	%d	"
					  "pid_tail	%.4f	pid_gyro	%.4f\r\n",
			          tim7_cnt, _bldc_data_uSD.uSD_data[0],  _bldc_data_uSD.uSD_data[1], _bldc_data_uSD.uSD_data[2], _bldc_data_uSD.uSD_data[3],
			          _bldc_data_uSD.uSD_data[4], _bldc_data_uSD.uSD_data[5], _bldc_data_uSD.uSD_data[6], _bldc_data_uSD.uSD_data[7], _bldc_data_uSD.uSD_data[8],
			          _imu_filter.quaternion[0], _imu_filter.quaternion[1], _imu_filter.quaternion[2], _imu_filter.quaternion[3],
			          _imu_data.accel.x, _imu_data.accel.y, _imu_data.accel.z, _imu_data.gyro.x, _imu_data.gyro.y, _imu_data.gyro.z,
			          _imu_filter.roll, _imu_filter.pitch, _imu_filter.yaw,
			          _imu_filter.pitch_gyro_int,
			          robot_state,
			          _tail_motor_pid.out_prev, _tail_gyro_pid.out_prev);


			  Write_openedFile(sprintf_robot_data);

		  }

		  /**
			***************************************
			* Robot Main Control CMD
			***************************************
			*/


		  Robot_Main_Controller();


	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 5;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 274;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999999999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 54;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 10;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 499;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 9600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_SPI2_nCS_GPIO_Output_GPIO_Port, IMU_SPI2_nCS_GPIO_Output_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Output_GPIO_Port, LED_GPIO_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IMU_SPI2_nCS_GPIO_Output_Pin */
  GPIO_InitStruct.Pin = IMU_SPI2_nCS_GPIO_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_SPI2_nCS_GPIO_Output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GPIO_Output_Pin */
  GPIO_InitStruct.Pin = LED_GPIO_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC1_CD_GPIO_Input_Pin */
  GPIO_InitStruct.Pin = SDMMC1_CD_GPIO_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDMMC1_CD_GPIO_Input_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim7)
	{
		tim7_flag = 1;
	}
}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
