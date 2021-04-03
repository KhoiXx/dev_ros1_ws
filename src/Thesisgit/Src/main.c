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
#define BS 29
#define radius 0.039
#define len 0.21 
//#define vmax 30.00
//#define vmin -30.00
#define pulmax 3500
#define pulmin 100
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
  uint8_t ActiveServo;
	char dma_buffer[BS], data_rx[BS];
	/**/
	static uint8_t set[7],bset[4];
	/**/
	char* i;
	int j=2000;
	float vvl = 0.00, vvr = 0.00;
	float pulfl = 0, pulfr = 0, pulbr = 0, pulbl = 0;

	PIDControl pidfl, pidfr, pidbr, pidbl;
	
	typedef struct
	{
		float ppr;
		float vmax;
		float vmin;
		float enco;
		float enco_pre;
		float vset;
		float v;
		float kp,ki,kd;
	}Wheelselect;
	Wheelselect whfl = {.ppr = 1491, .vmax = 1.23, .vmin = -1.23, .enco = 0, .enco_pre = 0, .vset=0.00, .v=0.00, .kp=0, .ki=0, .kd=0},
							whfr = {.ppr = 1491, .vmax = 1.23, .vmin = -1.23, .enco = 0, .enco_pre = 0, .vset=0.00, .v=0.00, .kp=0, .ki=0, .kd=0},
							whbr = {.ppr = 1700, .vmax = 1.23, .vmin = -1.23, .enco = 0, .enco_pre = 0, .vset=0.00, .v=0.00, .kp=0, .ki=0, .kd=0},
							whbl = {.ppr = 1700, .vmax = 1.23, .vmin = -1.23, .enco = 0, .enco_pre = 0, .vset=0.00, .v=0.00, .kp=0, .ki=0, .kd=0};
	typedef struct
		{
			int joint0;
			int joint1;
			int joint2;
			int joint3;
			int joint4;
			int joint5;
		}Jointangle;
	Jointangle curangle = {.joint0 = 0, .joint1 = 180, .joint2 = 90, .joint3 = 60, .joint4 = 50, .joint5 = 30},
						 posangle = {.joint0 = 0, .joint1 = 180, .joint2 = 90, .joint3 = 60, .joint4 = 50, .joint5 = 30};
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
void inversespeed(void);
float checkpul(float pul2check);
void split(char in[],uint8_t out[]);
float calspeed(int32_t value,int ppr);						 
void Rotate(int curangle, int posangle, uint8_t Channel);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*ham ngat nhan du du lieu*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_DMAStop(&huart5);
//	HAL_UART_Transmit(&huart5,"ok",3,0x200);
/*	char *tmp = dma_buffer;
	for(int tmp = 0; tmp <27;tmp++){
		data_rx[tmp] = dma_buffer[tmp];
	}
	memset(dma_buffer,0,sizeof(dma_buffer));
	if(strchr(data_rx,'S') != NULL){
		if(strchr(data_rx,'E') != NULL){
			HAL_UART_Transmit(&huart5,"ok",3,0x200);
		}
	}*/
	char *tmp = dma_buffer;
	if(strchr(dma_buffer,'S') != NULL){
		if(strchr(dma_buffer,'E') != NULL){
			HAL_UART_Transmit(&huart5,"ok",3,0x200);
			for(int l=0; l<BS; l++){
				data_rx[l] = *tmp++;
			}
		}
	}
	memset(dma_buffer,0,sizeof(dma_buffer));
	HAL_UART_DMAResume(&huart5);
	HAL_UART_Receive_DMA(&huart5,(uint8_t*)dma_buffer, BS);
}
/*het ham*/

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
	
	bset[0]=0;
	bset[1]=0;
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
	
	static float temp1;
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
			
			__HAL_TIM_SET_COUNTER(&htim1,0);
			__HAL_TIM_SET_COUNTER(&htim5,0);
			__HAL_TIM_SET_COUNTER(&htim3,0);
			__HAL_TIM_SET_COUNTER(&htim4,0);
			
			//xu li chuoi nhan
			if(strchr(data_rx,'S') != NULL){
				if(strchr(data_rx,'E') != NULL){
					if(strchr(data_rx,'A') != NULL){
						char *tmp1 = strchr(data_rx,' ');
						tmp1 = strchr(tmp1,' ');
						//split(data_rx,set);
						split(tmp1,set);
					}
					if(strchr(data_rx,'B') != NULL){
						char *tmp1 = strchr(data_rx,' ');
						tmp1 = strchr(tmp1,' ');
						//send speed
						split(tmp1,bset);
					}
				}
			}
			/*het ham ngat 5ms*/
			
			//tinh van toc dai

			
			//tinh van toc dat
			inversespeed();

			vvl = -0.14;
			vvr = -0.14;
			
			//Dat thong so PID
			PIDTuningsSet(&pidfl,2.6,4,0.005);
			PIDTuningsSet(&pidbl,3,4,0.004);
			PIDTuningsSet(&pidfr,2.6,4,0.005);
			PIDTuningsSet(&pidbr,3,4,0.004);
//			}
			//PID for Front Left Wheel
			PIDInputSet(&pidfl,whfl.v);
			PIDSetpointSet(&pidfl,vvl);
			PIDCompute(&pidfl);
			if(vvl>=0)
				pidfl.output += 0.9*(vvl-0.08)+0.01;
			else 
				pidfl.output += 0.9*(vvl+0.08)-0.01;
			pulfl=PIDOutputGet(&pidfl)/(whfl.vmax)*(pulmax-pulmin)+pulmin;
			pulfl = checkpul(pulfl);

			//PID for Front Right Wheel
			PIDInputSet(&pidfr,whfr.v);
			PIDSetpointSet(&pidfr,vvr);
			PIDCompute(&pidfr);
			if(vvr>=0)
				pidfr.output += 0.9*(vvr-0.08)+0.01;
			else 
				pidfr.output += 0.9*(vvr+0.08)-0.01;
			pulfr=PIDOutputGet(&pidfr)/(whfr.vmax)*(pulmax-pulmin)+pulmin;
			pulfr = checkpul(pulfr);

			//PID for Back Right Wheel
			PIDInputSet(&pidbr,whbr.v);
			PIDSetpointSet(&pidbr,vvr);
			PIDCompute(&pidbr);
			if (vvr >=0)
				pidbr.output += 0.85*(vvr-0.08)+0.01;
			else
				pidbr.output += 0.85*(vvr+0.08)-0.01;
			pulbr=PIDOutputGet(&pidbr)/(whbr.vmax)*(pulmax-pulmin)+pulmin;
			pulbr = checkpul(pulbr);
			

			//PID for Back Left Wheel
			PIDInputSet(&pidbl,whbl.v);
			PIDSetpointSet(&pidbl,vvl);
			PIDCompute(&pidbl);
			if (vvl >=0)
				pidbl.output += 0.85*(vvl-0.08)+0.01;
			else
				pidbl.output += 0.85*(vvl+0.08)-0.01;
			pulbl=PIDOutputGet(&pidbl)/(whbl.vmax)*(pulmax-pulmin)+pulmin;
			pulbl = checkpul(pulbl);
		}

		whfl.v = calspeed(whfl.enco,whfl.ppr);
		whfr.v = calspeed(whfr.enco,whfr.ppr);
		whbr.v = calspeed(whbr.enco,whbr.ppr);
		whbl.v = calspeed(whbl.enco,whbl.ppr);
		
		/*xuat pwm cho 4 dong co*/


		if(pulfl >= 0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint16_t)pulfl);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		}else{
			int fltmp = 3599+pulfl;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, fltmp);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
		}

		if(pulfr >= 0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)pulfr);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	  }else{
			int frtmp = 3599+pulfr;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, frtmp);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		}

		if(pulbr >= 0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint16_t)pulbr);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
		}else{
			int brtmp = 3599+pulbr;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, brtmp);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
		}

		if(pulbl >= 0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t)pulbl);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
    }else{
			int bltmp = 3599+pulbl;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, bltmp);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
		}

////		if(j <3500) j+=5;
//		j=200;
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, j);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
//		
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, j);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
//		
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, j);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
//		
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, j);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
//		HAL_Delay(10);
		/**/
	
		
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

float calspeed(int32_t value,  int ppr)
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

void Rotate(int curangle, int posangle, uint8_t Channel){
	int anglemax = 180, anglemin = 0;
	if(Channel == 0) anglemax = 10;
	if(Channel == 4) anglemin = 30;
	
	if(curangle >anglemax) curangle = anglemax;
	else if(curangle <anglemin) curangle = anglemin;
	
	if(Channel == 5) curangle =  curangle*2/3; 
	
	if(curangle >= posangle){
		int tmp1;
		for(tmp1 = posangle; tmp1 <=  curangle; tmp1++){
			PCA9685_SetServoAngle(Channel, tmp1);
			HAL_Delay(7);			
		}
	} else {
		int tmp2;
		for(tmp2 = posangle; tmp2 >= curangle; tmp2--){
			PCA9685_SetServoAngle(Channel, tmp2);
			HAL_Delay(7);
		}
	}
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
