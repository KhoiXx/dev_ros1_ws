/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>                     /* This ert_main.c example uses printf/fflush */
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_it.h"
#include "SERVO.h"
#include "pca9685.h"
#include "pid_controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_Motor1   0
#define SERVO_COUNT	6
#define BS 30
#define radius 0.039 //m
#define len 0.21 
//#define vmax 30.00
//#define vmin -30.00
#define pulmax 3500
#define pulmin 100
#define pi 3.1416

#define Header_val 0xffaa
#define EndOfFrame_val 0x0e0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

/* USER CODE BEGIN PV */
/*variables define*/
uint8_t ActiveServo;
uint8_t dma_buffer[BS], data_rx[BS];
static uint8_t bset[4];
uint16_t rxByteCount = 0;
int j=2000;

float vvl = 0.00, vvr = 0.00, vpl = 0.00, vpr = 0.00, v_spin = 0.5; //van toc dat, vi tri dat, van toc spin
int16_t spin_angle = 0, vvl_t = 0, vvr_t =0;
float pulfl = 0, pulfr = 0, pulbr = 0, pulbl = 0;
bool __flag_stop = false, __flag_run_speed = false, __flag_run_position = false;
uint8_t __base_flag = 0; // 0: stop; 1:set speed; 2: set position; 3: spin
float Kpid[3] ;
int count = 0;

/**/


/*struct define*/
PIDControl pidfl, pidfr, pidbr, pidbl;

typedef struct
{
  float ppr;
  float vmax;
  float vmin;
  int16_t enco;
  int16_t enco_sum;
  float pos;
  float v;
  float kp,ki,kd;
}Wheelselect;
Wheelselect whfl = {.ppr = 1491, .vmax = 1.23, .vmin = -1.23, .enco = 0, .enco_sum = 0, .pos = 0.00, .v = 0.00, .kp = 0, .ki = 0, .kd = 0},
            whfr = {.ppr = 1491, .vmax = 1.23, .vmin = -1.23, .enco = 0, .enco_sum = 0, .pos = 0.00, .v = 0.00, .kp = 0, .ki = 0, .kd = 0},
            whbr = {.ppr = 1500, .vmax = 1.23, .vmin = -1.23, .enco = 0, .enco_sum = 0, .pos = 0.00, .v = 0.00, .kp = 0, .ki = 0, .kd = 0},
            whbl = {.ppr = 1500, .vmax = 1.23, .vmin = -1.23, .enco = 0, .enco_sum = 0, .pos = 0.00, .v = 0.00, .kp = 0, .ki = 0, .kd = 0};

enum flag{
  STOP = 0,
  SET_SPEED = 1,
  SET_POSITION = 2,
  SET_SPIN = 3
};

enum function_code {
  COMMAND_SEND_SPEED = 0xA0,
  RESP_ACK_SEND_SPEED = 0xB0,
  COMMAND_SEND_ENCODER  = 0xA1,
  RESP_ACK_SEND_ENCODER  = 0xB1,
  COMMAND_SET_SPEED = 0xA2,
  RESP_ACK_SET_SPEED = 0xB2,
  COMMAND_SET_POSITION = 0xA3,
  RESP_ACK_SET_POSITION = 0xB3,
  COMMAND_SPIN = 0xA4,
  RESP_ACK_SPIN = 0xB4,
  COMMAND_STOP = 0xA5,
  RESP_ACK_STOP = 0xB5, 
  SET_PID = 0xB6
};

typedef struct{
  uint16_t Header; //0:2
  uint8_t FunctionCode; //  2:3 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5
  uint8_t ACK; //3:4 'Y' for ACK, 'N' for NACK
  uint16_t CheckSum;//4:6
  uint16_t EndOfFrame;//6:8
}__attribute__((packed)) SendAckStruct;
SendAckStruct SendAckFrame;

typedef struct{
  uint16_t Header; //0:2
  uint8_t FunctionCode; //2:3 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5
  uint8_t Data[8]; //3:11 1 float == 4byte but convert float -> uint multiple by 1000 
  uint16_t CheckSum; //11:13
  uint16_t EndOfFrame; //13:15
}__attribute__((packed)) SendDataStruct;
SendDataStruct SendDataFrame;
/**/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
						 
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UART_ReceiveData(UART_HandleTypeDef *huart, uint8_t *pdma_buffer, uint16_t Size);
void UART_SendAck(UART_HandleTypeDef *huart, uint8_t ack, uint8_t functionCode);
void UART_SendData(UART_HandleTypeDef *huart, uint8_t functionCode);
bool Pre_Ack_Send(UART_HandleTypeDef *huart, uint16_t _rxByteCount, uint16_t _startIndex, uint16_t _frameByteCount,uint8_t _functionCode );

void calc_pid_position(float _vpl, float _vpr);
void calc_pid_speed(float _vvl, float _vvr);
void set_pwm(float _pulfl, float _pulfr, float _pulbr, float _pulbl);
void check_flag(void);
void inversespeed(void);
void split(char in[],uint8_t out[]);
float checkpul(float pul2check);
float calspeed(int16_t value,int ppr);
void stop(void);
void set_spin(int16_t _angle, float _spin_speed);

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	PCA9685_Init(&hi2c1);
	
	//khoi tao  Servo
	PCA9685_SetPwmFrequency(48);
	HAL_Delay(500);

	//khoi tao dong co
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);	
	
	//  HAL_Delay(2000);
	HAL_UART_Receive_DMA(&huart5,(uint8_t*)dma_buffer, BS);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	
	//khoitaoPID
	PIDInit(&pidfl,whfl.kp,whfl.ki,whfl.kd,0.005,whfl.vmin,whfl.vmax,AUTOMATIC,DIRECT);
	PIDInit(&pidfr,whfr.kp,whfr.ki,whfr.kd,0.005,whfr.vmin,whfr.vmax,AUTOMATIC,DIRECT);
	PIDInit(&pidbr,whbr.kp,whbr.ki,whbr.kd,0.005,whbr.vmin,whbr.vmax,AUTOMATIC,DIRECT);
	PIDInit(&pidbl,whbl.kp,whbl.ki,whbl.kd,0.005,whbl.vmin,whbl.vmax,AUTOMATIC,DIRECT);
	
  memset(Kpid, 0, sizeof(Kpid));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		/*bat dau ham ngat 5ms*/
		if(tick_flag ==1){
			tick_flag =0;
			//doc encoder
			whfl.enco = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
			whfr.enco = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
			whbr.enco = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
			whbl.enco = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
      
      whfl.enco_sum += whfl.enco;
      whfr.enco_sum += whfl.enco;
      whbr.enco_sum += whfl.enco;
      whbl.enco_sum += whfl.enco;
			
			__HAL_TIM_SET_COUNTER(&htim1,0);
			__HAL_TIM_SET_COUNTER(&htim5,0);
			__HAL_TIM_SET_COUNTER(&htim3,0);
			__HAL_TIM_SET_COUNTER(&htim4,0);
			
			//xu li chuoi nhan
      UART_ReceiveData(&huart5, (uint8_t *) dma_buffer, BS);
      check_flag();
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3599;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC7 PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void split(char in[], uint8_t out[])
{
	char* token = strtok(in," ");
	char* ptr;
	uint8_t k=0;
	while( token != NULL ) {
			out[k] = strtod(token,&ptr);
      token = strtok(NULL, " ");
			k++;
   }
}

float calspeed(int16_t value,  int ppr)
{
	float 	p = 0.00, v = 0.00;
	
	p = (float)value*(1.00/ppr)*(1000.00/5.00)*60; //rpm
	v = radius*2.00*3.1416*p*(1.00/60.00); // m/s
	return v;
}

void inversespeed(void)
{
	float vc_set = (float)bset[0]/100000.00000;
	float omegac_set = (float)bset[1]/100.00;
	vvl = (2*vc_set-omegac_set*len)/2;
	vvr = (2*vc_set+omegac_set*len)/2;
	if(vvl>1.23){
		vvl = 1.23;
	}
	if(vvr>1.23){
		vvr = 1.23;
	}
}

float checkpul(float pul2check){
	if((pul2check)>=pulmax){
		pul2check = pulmax-1;
	}else if(pul2check <-pulmax){ 
		pul2check = -pulmax+1;
	}
	return pul2check;
}

void stop(void){
  // __flag_stop = true;
  __base_flag = STOP;
  // set_pwm(0, 0, 0, 0);
  calc_pid_speed(0, 0);
  set_pwm(pulfl, pulfr, pulbr, pulbl);
  whfl.enco_sum = 0;
  whfr.enco_sum = 0;
  whbr.enco_sum = 0;
  whbl.enco_sum = 0;  
}

void check_flag()
{
  switch (__base_flag)
  {
  case STOP:
    stop();
    break;
  case SET_SPEED:
    vvl = (float)vvl_t / 10000;
    vvr = (float)vvr_t / 10000;
    calc_pid_speed(vvl, vvr);
    set_pwm(pulfl, pulfr, pulbr, pulbl);
		break;
  case SET_POSITION:
    calc_pid_position(vpl, vpr);
    set_pwm(pulfl, pulfr, pulbr, pulbl);
		break;
  case SET_SPIN:
    set_spin(spin_angle, v_spin);
		break;
  default:
    break;
  }
}

void set_pwm(float _pulfl, float _pulfr, float _pulbr, float _pulbl)
{
  if (_pulfl >= 0)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)_pulfl);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  }
  else
  {
    int fltmp = 3599 + _pulfl;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, fltmp);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  }

  if (_pulfr >= 0)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)_pulfr);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
  }
  else
  {
    int frtmp = 3599 + _pulfr;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, frtmp);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  }

  if (_pulbr >= 0)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint16_t)_pulbr);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  }
  else
  {
    int brtmp = 3599 + _pulbr;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, brtmp);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  }

  if (_pulbl >= 0)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t)_pulbl);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
  }
  else
  {
    int bltmp = 3599 + _pulbl;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, bltmp);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
  }
}

void set_spin(int16_t _angle, float _spin_speed) //angle deg, speed m/s
{
  int8_t spin_direction;
  if (_angle > 0){
    //spin anti_clockwise
    spin_direction = 1;
  } else if(_angle <0){
    spin_direction = -1;
  } else {
    stop();
    return;
  }
  float w = _spin_speed / radius; //omega = v / R rad/s
  float time_to_spin = ((float)_angle * pi) / (w * 180);
  float s_to_spin = _spin_speed * time_to_spin;
  calc_pid_position(s_to_spin, - s_to_spin);
  set_pwm(pulfl, pulfr, pulbr, pulbl);
}

/*PID vitri*/
void calc_pid_position(float _vpl, float _vpr)
{
  whfl.pos = whfl.enco_sum * 360 / whfl.ppr;
  whfr.pos = whfr.enco_sum * 360 / whfr.ppr;
  whbr.pos = whbr.enco_sum * 360 / whbr.ppr;
  whbl.pos = whbl.enco_sum * 360 / whbl.ppr;
  //Dat thong so PID
  float empty_string[3];
  if (memcmp(Kpid, empty_string,3) == 0)
  {
    PIDTuningsSet(&pidfl, 2.6, 4, 0.005);
    PIDTuningsSet(&pidbl, 3, 4, 0.004);
    PIDTuningsSet(&pidfr, 2.6, 4, 0.005);
    PIDTuningsSet(&pidbr, 3, 4, 0.004);
  }
  else
  {
    PIDTuningsSet(&pidfl, Kpid[0], Kpid[1], Kpid[2]);
    PIDTuningsSet(&pidbl, Kpid[0], Kpid[1], Kpid[2]);
    PIDTuningsSet(&pidfr, Kpid[0], Kpid[1], Kpid[2]);
    PIDTuningsSet(&pidbr, Kpid[0], Kpid[1], Kpid[2]);
  }

  //PID for Front Left Wheel
  PIDInputSet(&pidfl, whfl.pos);
  PIDSetpointSet(&pidfl, _vpl);
  PIDCompute(&pidfl);
  pulfl = PIDOutputGet(&pidfl) / (whfl.vmax) * (pulmax - pulmin) + pulmin;
  pulfl = checkpul(pulfl);

  //PID for Front Right Wheel
  PIDInputSet(&pidfr, whfr.pos);
  PIDSetpointSet(&pidfr, _vpr);
  PIDCompute(&pidfr);
  pulfr = PIDOutputGet(&pidfr) / (whfr.vmax) * (pulmax - pulmin) + pulmin;
  pulfr = checkpul(pulfr);

  //PID for Back Right Wheel
  PIDInputSet(&pidbr, whbr.pos);
  PIDSetpointSet(&pidbr, _vpr);
  PIDCompute(&pidbr);
  pulbr = PIDOutputGet(&pidbr) / (whbr.vmax) * (pulmax - pulmin) + pulmin;
  pulbr = checkpul(pulbr);

  //PID for Back Left Wheel
  PIDInputSet(&pidbl, whbl.pos);
  PIDSetpointSet(&pidbl, _vpl);
  PIDCompute(&pidbl);
  pulbl = PIDOutputGet(&pidbl) / (whbl.vmax) * (pulmax - pulmin) + pulmin;
  pulbl = checkpul(pulbl);
}

/*PID van toc*/
void calc_pid_speed(float _vvl, float _vvr)
{

  //Dat thong so PID
  float empty_string[3];
  // if (memcmp(Kpid, empty_string, 3) == 0)
  // {
  PIDTuningsSet(&pidfl, 2.6, 4, 0.005);
  PIDTuningsSet(&pidbl, 3, 4, 0.004);
  PIDTuningsSet(&pidfr, 2.6, 4, 0.005);
  PIDTuningsSet(&pidbr, 3, 4, 0.004);
  // }
  // else
  // {
  //   PIDTuningsSet(&pidfl, Kpid[0], Kpid[1], Kpid[2]);
  //   PIDTuningsSet(&pidbl, Kpid[0], Kpid[1], Kpid[2]);
  //   PIDTuningsSet(&pidfr, Kpid[0], Kpid[1], Kpid[2]);
  //   PIDTuningsSet(&pidbr, Kpid[0], Kpid[1], Kpid[2]);
  // }
  whfl.v = calspeed(whfl.enco,whfl.ppr);
  whfr.v = calspeed(whfr.enco,whfr.ppr);
  whbr.v = calspeed(whbr.enco,whbr.ppr);
  whbl.v = calspeed(whbl.enco,whbl.ppr);

  //PID setup
  PIDInputSet(&pidfl, whfl.v);
  PIDInputSet(&pidfr, whfr.v);
  PIDInputSet(&pidbr, whbr.v);
  PIDInputSet(&pidbl, whbl.v);

  PIDSetpointSet(&pidfl, _vvl);
  PIDSetpointSet(&pidfr, _vvr);
  PIDSetpointSet(&pidbr, _vvr);
  PIDSetpointSet(&pidbl, _vvl);

  PIDCompute(&pidfl);
  PIDCompute(&pidfr);
  PIDCompute(&pidbr);
  PIDCompute(&pidbl);

  //calib output
  if (_vvl >= 0)
  {
    pidfl.output += 0.9 * (_vvl - 0.08) + 0.01;
    pidbl.output += 0.85 * (_vvl - 0.08) + 0.01;
  }
  else
  {
    pidfl.output += 0.9 * (_vvl + 0.08) - 0.01;
    pidbl.output += 0.85 * (_vvl + 0.08) - 0.01;
  }

  if (_vvr >= 0)
  {
    pidfr.output += 0.9 * (_vvr - 0.08) + 0.01;
    pidbr.output += 0.85 * (_vvr - 0.08) + 0.01;
  }
  else
  {
    pidfr.output += 0.9 * (_vvr + 0.08) - 0.01;
    pidbr.output += 0.85 * (_vvr + 0.08) - 0.01;
  }

  pulfl = PIDOutputGet(&pidfl) / (whfl.vmax) * (pulmax - pulmin) + pulmin;
  pulfl = checkpul(pulfl);
  pulfr = PIDOutputGet(&pidfr) / (whfr.vmax) * (pulmax - pulmin) + pulmin;
  pulfr = checkpul(pulfr);
  pulbr = PIDOutputGet(&pidbr) / (whbr.vmax) * (pulmax - pulmin) + pulmin;
  pulbr = checkpul(pulbr);
  pulbl = PIDOutputGet(&pidbl) / (whbl.vmax) * (pulmax - pulmin) + pulmin;
  pulbl = checkpul(pulbl);
}

/*truyen nhan*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);
  __HAL_DMA_DISABLE(huart5.hdmarx);
  HAL_UART_Receive_DMA(&huart5, (uint8_t *)dma_buffer, BS);
  rxByteCount = 0;
}

void UART_ReceiveData(UART_HandleTypeDef *huart, uint8_t *pdma_buffer, uint16_t Size)
{
  //nhan du 4 byte de check header, function code, id
  rxByteCount = Size - __HAL_DMA_GET_COUNTER(huart->hdmarx);

  if (rxByteCount < 4)
    return; //khong nhan du byte
  for (int i = 0; i < rxByteCount - 4; i++)
  {
    if (dma_buffer[i] != 0xAA || dma_buffer[i + 1] != 0xFF)
      continue;
    uint16_t startIndex = i; // start index of header in dma_buffer
    uint8_t function_code = dma_buffer[i + 2];

    if (function_code == COMMAND_SEND_SPEED)
    {
      uint16_t frameByteCount = 7; //2b header 1b function 2b crc 2b end
//      bool check_status = Pre_Ack_Send(huart, rxByteCount, startIndex, frameByteCount, function_code);
//      if (check_status == true)
//        HAL_Delay(10);
       UART_SendData(&huart5, function_code);
    }
    else if (function_code == COMMAND_SEND_ENCODER)
    {
      uint16_t frameByteCount = 7; //2b header 1b function 2b crc 2b end
      bool check_status = Pre_Ack_Send(huart, rxByteCount, startIndex, frameByteCount, function_code);
      if (check_status == true)
        HAL_Delay(10);
        UART_SendData(&huart5, function_code);
    }
    else if (function_code == COMMAND_SET_SPEED)
    {
      uint16_t frameByteCount = 13; //2b header 1b function 6b data(3b whr 3b whl) 2b crc 2b end
      
      bool check_status = Pre_Ack_Send(huart, rxByteCount, startIndex, frameByteCount, function_code);
      if (check_status == true)
      {
        memcpy(&vvl_t, &dma_buffer[startIndex + 3], 2); //vv_t, vvr_t : int16_t (/10000) chuyen 2 byte thanh so unsigned
        memcpy(&vvr_t, &dma_buffer[startIndex + 6], 2);
        if (dma_buffer[startIndex + 5] == 0xff)
        {
          vvl_t *= -1;
        }
        if (dma_buffer[startIndex + 8] == 0xff)
        {
          vvr_t *= -1;
        }
        // __flag_stop = false;
        // __flag_run_speed = true;
        __base_flag = SET_SPEED;
      }
    }
    else if (function_code == COMMAND_SET_POSITION)
    {
      uint16_t frameByteCount = 15; //2b header 1b function 8b data(4b whl(%f) 4b whr(%f)) 2b crc 2b end
      bool check_status = Pre_Ack_Send(huart, rxByteCount, startIndex, frameByteCount, function_code);
      if (check_status == true)
      {
        memcpy(&vpl, &dma_buffer[startIndex + 3], 4);
        memcpy(&vpr, &dma_buffer[startIndex + 7], 4);
        // __flag_stop = false;
        // __flag_run_position = true;
        __base_flag = SET_POSITION;
      }
    }
    else if (function_code == COMMAND_SPIN)
    {
      uint16_t frameByteCount = 13; //2b header 1b function 8b data(2b spin_angle(int16) 4b spin_speed(float)) 2b crc 2b end
      bool check_status = Pre_Ack_Send(huart, rxByteCount, startIndex, frameByteCount, function_code);
      if (check_status == true)
      {
        memcpy(&spin_angle, &dma_buffer[startIndex + 3], 2);
        memcpy(&v_spin, &dma_buffer[startIndex + 5], 4);
        // __flag_stop = false;
        __base_flag = SET_SPIN;
      }
    }
    else if (function_code == COMMAND_STOP)
    {
      uint16_t frameByteCount = 7; //2b header 1b function 2b crc 2b end
      Pre_Ack_Send(huart, rxByteCount, startIndex, frameByteCount, function_code);
      stop();
    }
    else if (function_code == SET_PID)
    {
      uint16_t frameByteCount = 13; //2b header 1b function 6b data(2b kP(int16_t) 2b kI(int16_t) 2b kD(int16_t)) 2b crc 2b end
      bool check_status = Pre_Ack_Send(huart, rxByteCount, startIndex, frameByteCount, function_code);
      if (check_status == true)
      {
        int16_t tmp;
        memcpy(&tmp, &dma_buffer[startIndex + 3], 2);
        Kpid[0] = tmp / 1000; //Kp (float) /1000
        memcpy(&tmp, &dma_buffer[startIndex + 5], 2);
        Kpid[1] = tmp / 1000;
        memcpy(&tmp, &dma_buffer[startIndex + 7], 2);
        Kpid[2] = tmp / 1000;
        // __flag_stop = false;
        // __flag_run_position = true;
        __base_flag = SET_POSITION;
      }
    }

    __HAL_DMA_DISABLE(huart5.hdmarx);
		memset(pdma_buffer, 0, BS);
    HAL_UART_Receive_DMA(huart, pdma_buffer, Size);
    rxByteCount = 0;
    break;
  }
}

bool Pre_Ack_Send(UART_HandleTypeDef *huart, uint16_t _rxByteCount, uint16_t _startIndex, uint16_t _frameByteCount, uint8_t _functionCode)
{
  if (_rxByteCount - _startIndex < _frameByteCount)
    return false;
  // get frame data to an array
  uint8_t arrFrame[_frameByteCount];
  for (int i = 0; i < _frameByteCount; i++)
    arrFrame[i] = dma_buffer[i + _startIndex];

  // check sum
  uint16_t crc = 0;
  for (int i = 0; i < _frameByteCount - 4; i++)
    crc += arrFrame[i];
  
  if ((crc & 0xff) != arrFrame[_frameByteCount - 3] || ((crc >> 8) & 0xff) != arrFrame[_frameByteCount - 4])
  {
    // send NACK
    UART_SendAck(huart, 'N', _functionCode);
    return false;
  }
  // send ACK
  UART_SendAck(huart, 'Y', _functionCode);
  return true;
}

void UART_SendAck(UART_HandleTypeDef *huart, uint8_t ack, uint8_t functionCode)
{
	SendAckFrame.Header = Header_val;
	SendAckFrame.FunctionCode = functionCode;
	SendAckFrame.ACK = ack;
	SendAckFrame.EndOfFrame = EndOfFrame_val;
	
	// calculate check sum
	uint16_t crc = 0;
	crc +=  0xAA + 0xFF;
	crc += SendAckFrame.FunctionCode;
	crc += SendAckFrame.ACK;
	SendAckFrame.CheckSum = crc;
	// send data
	HAL_UART_Transmit_DMA(huart, (uint8_t *) &SendAckFrame, sizeof(SendAckFrame));
	count = sizeof(SendAckFrame);
}

void UART_SendData(UART_HandleTypeDef *huart, uint8_t functionCode){
  SendDataFrame.Header = Header_val;
  SendDataFrame.FunctionCode = functionCode;
	SendDataFrame.EndOfFrame = EndOfFrame_val;
	
  if (functionCode == COMMAND_SEND_SPEED){
    int16_t Speed_whfl = (int16_t)(whfl.v*1000);
    int16_t Speed_whfr = (int16_t)(whfr.v*1000);
    int16_t Speed_whbr = (int16_t)(whbr.v*1000);
    int16_t Speed_whbl = (int16_t)(whbl.v*1000);
    // 01 23 45 67= whfl whfr whbr whbl 
    for (int i = 1; i >= 0; i--){
      SendDataFrame.Data[i] = (Speed_whfl >> 8 * i) & 0xff;
      SendDataFrame.Data[2 + i] = (Speed_whfr >> 8 * i) & 0xff;
      SendDataFrame.Data[4 + i] = (Speed_whbr >> 8 * i) & 0xff;
      SendDataFrame.Data[6 + i] = (Speed_whbl >> 8 * i & 0xff);
    }
  } else if (functionCode == COMMAND_SEND_ENCODER){
    // 01 23 45 67= whfl whfr whbr whbl 
    for (int i = 1; i >= 0; i--){
      SendDataFrame.Data[i] = (whfl.enco_sum >> 8 * i) & 0xff;
      SendDataFrame.Data[2 + i] = (whfr.enco_sum >> 8 * i) & 0xff;
      SendDataFrame.Data[4 + i] = (whbr.enco_sum >> 8 * i) & 0xff;
      SendDataFrame.Data[6 + i] = (whbl.enco_sum >> 8 * i & 0xff);
    }
  } 

  // calculate check sum
	uint16_t crc = 0;
	crc += 0xFF + 0xAA;
  crc += SendDataFrame.FunctionCode;
  for (int i = 0; i<8; i++){
    crc += SendDataFrame.Data[i];
  }
 SendDataFrame.CheckSum = crc;
  HAL_UART_Transmit_DMA(huart, (uint8_t *) &SendDataFrame, sizeof(SendDataFrame));
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
