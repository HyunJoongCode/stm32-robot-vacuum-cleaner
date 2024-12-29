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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t UART_flag = 0;

uint8_t Timer_flag = 0;
uint32_t Time = 0;
uint8_t oper_flag = 1;
uint8_t play_flag = 0;

uint8_t mode_ch[4];
uint8_t rotate_flag = 0;
uint16_t initial_speed = 1000;

uint8_t STOP_L_Flag = 0;
uint8_t STOP_R_Flag = 0;
uint16_t Time_1Sec = 0;

uint8_t crash_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __cplusplus
	extern "C" int _write(int32_t file, uint8_t *ptr, int32_t len) {
#else
	int _write(int32_t file, uint8_t *ptr, int32_t len) {
#endif
	if( HAL_UART_Transmit(&huart1, ptr, len, len) == HAL_OK ) return len;
	else return 0;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_IncTick(void) {
	uwTick++;

	if ((uwTick % 1000) == 0)
	{
		Time_1Sec++;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	Time++;

	if(Time == 2 && rotate_flag == 1) {
		htim2.Instance->CCR2 = initial_speed;
		htim3.Instance->CCR3 = initial_speed;
		rotate_flag = 0;
		Time = 0;
		HAL_TIM_Base_Stop_IT(&htim4);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART1) {
		UART_flag = 1;
		HAL_UART_Receive_IT(&huart1, mode_ch, sizeof(mode_ch));
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == user_key_Pin) {
		play_flag = 1;
	}

    if(GPIO_Pin == start_Pin) {
    	play_flag = 0;
    	crash_flag = 0;
    	STOP_R_Flag = 0;
    	STOP_L_Flag = 0;
    	oper_flag = 1;
		htim2.Instance->CCR2 = 0;
		htim3.Instance->CCR3 = 0;
    }

    if(GPIO_Pin == Crash_L_Pin && crash_flag == 0) {
    	crash_flag = 1;

        STOP_L_Flag = 1;
        Time_1Sec = 0;
    }

    if(GPIO_Pin == Crash_R_Pin && crash_flag == 0) {
    	crash_flag = 1;

        STOP_R_Flag = 1;
        Time_1Sec = 0;
    }
}

void adjust_motor_speed(void) {
	if (mode_ch[0] == 'S' && mode_ch[1] == 'R'&&mode_ch[2] == 'I' && mode_ch[3] == 'E') {
		htim2.Instance->CCR2 = 850;
		htim3.Instance->CCR3 = initial_speed;

		__HAL_TIM_SET_COUNTER(&htim4, 0);
		rotate_flag = 1;
		HAL_TIM_Base_Start_IT(&htim4);
	}

	else if (mode_ch[0] == 'S' && mode_ch[1] == 'L'&&mode_ch[2] == 'E' && mode_ch[3] == 'E') {
		htim2.Instance->CCR2 = initial_speed;
		htim3.Instance->CCR3 = 800;

		__HAL_TIM_SET_COUNTER(&htim4, 0);
		rotate_flag = 1;
		HAL_TIM_Base_Start_IT(&htim4);
	}

	else if (mode_ch[0] == 'S' && mode_ch[1] == 'F'&&mode_ch[2] == 'R' && mode_ch[3] == 'E') {
		htim2.Instance->CCR2 = initial_speed;
		htim3.Instance->CCR3 = initial_speed;
		rotate_flag = 0;
	}

	else if (mode_ch[0] == 'S' && mode_ch[1] == 'S'&&mode_ch[2] == 'T' && mode_ch[3] == 'E') {
		htim2.Instance->CCR2 = 0;
		htim3.Instance->CCR3 = 0;

		HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, 0);
		HAL_GPIO_WritePin(cleaningmotor_GPIO_Port, cleaningmotor_Pin, 0);
		HAL_GPIO_WritePin(vacuum_GPIO_Port, vacuum_Pin, 0);
		HAL_GPIO_WritePin(windmill_GPIO_Port, windmill_Pin, 0);

		play_flag = 0;
		rotate_flag = 0;
	}
}

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  HAL_UART_Receive_IT(&huart1, mode_ch, sizeof(mode_ch));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(play_flag == 1) {

	      if(STOP_L_Flag == 1 && STOP_R_Flag == 0) {
	          	  if(Time_1Sec == 0 ) {
	          		  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, 0);
	          		  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, 1);
	          		  HAL_GPIO_WritePin(in3_GPIO_Port, in3_Pin, 0);
	          		  HAL_GPIO_WritePin(in4_GPIO_Port, in4_Pin, 1);

	          		  htim2.Instance->CCR2 = initial_speed;
	          		  htim3.Instance->CCR3 = initial_speed;
	          	  }
	              if(Time_1Sec == 2 ) {
	            	  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, 1);
		              HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, 0);
		              HAL_GPIO_WritePin(in3_GPIO_Port, in3_Pin, 1);
		              HAL_GPIO_WritePin(in4_GPIO_Port, in4_Pin, 0);

	          		  htim2.Instance->CCR2 = initial_speed;
	          		  htim3.Instance->CCR3 = 800;
	              }
	              if(Time_1Sec == 3 ) {
	          		  htim2.Instance->CCR2 = 0;
	          		  htim3.Instance->CCR3 = 0;
	                  Time_1Sec = 0;
	                  STOP_L_Flag = 0;
	                  crash_flag = 0;
	              }
	      }

	      if(STOP_L_Flag == 0 && STOP_R_Flag == 1) {
				  if(Time_1Sec == 0 ) {
					  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, 0);
					  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, 1);
					  HAL_GPIO_WritePin(in3_GPIO_Port, in3_Pin, 0);
					  HAL_GPIO_WritePin(in4_GPIO_Port, in4_Pin, 1);

	          		  htim2.Instance->CCR2 = initial_speed;
	          		  htim3.Instance->CCR3 = initial_speed;
				  }
				  if(Time_1Sec == 2 ) {
					  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, 1);
					  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, 0);
					  HAL_GPIO_WritePin(in3_GPIO_Port, in3_Pin, 1);
					  HAL_GPIO_WritePin(in4_GPIO_Port, in4_Pin, 0);

	          		  htim2.Instance->CCR2 = 800;
	          		  htim3.Instance->CCR3 = initial_speed;
				  }
				  if(Time_1Sec == 3 ) {
	          		  htim2.Instance->CCR2 = 0;
	          		  htim3.Instance->CCR3 = 0;
					  Time_1Sec = 0;
					  STOP_R_Flag = 0;
					  crash_flag = 0;
				  }
	      }
	      if(STOP_L_Flag == 1 && STOP_R_Flag == 1) {
	            if(Time_1Sec == 0 ) {
	                HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, 0);
	                HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, 1);
	                HAL_GPIO_WritePin(in3_GPIO_Port, in3_Pin, 0);
	                HAL_GPIO_WritePin(in4_GPIO_Port, in4_Pin, 1);

	                htim2.Instance->CCR2 = initial_speed;
				    htim3.Instance->CCR3 = initial_speed;
	            }
	            if(Time_1Sec == 1 ) {
	            	  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, 1);
					  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, 0);
					  HAL_GPIO_WritePin(in3_GPIO_Port, in3_Pin, 1);
					  HAL_GPIO_WritePin(in4_GPIO_Port, in4_Pin, 0);

				    htim2.Instance->CCR2 = 800;
					htim3.Instance->CCR3 = initial_speed;
				}
	            if(Time_1Sec == 3 ) {
	            	htim2.Instance->CCR2 = 0;
				    htim3.Instance->CCR3 = 0;
					Time_1Sec = 0;
					STOP_L_Flag = 0;
					STOP_R_Flag = 0;
					crash_flag = 0;
				}
	        }

	        if(oper_flag == 1) {
			  HAL_UART_Receive_IT(&huart1, mode_ch, sizeof(mode_ch));

/*			  HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, 1);
			  HAL_GPIO_WritePin(cleaningmotor_GPIO_Port, cleaningmotor_Pin, 1);
			  HAL_GPIO_WritePin(vacuum_GPIO_Port, vacuum_Pin, 1);
			  HAL_GPIO_WritePin(windmill_GPIO_Port, windmill_Pin, 1);*/

			  htim3.Instance->CCR3 = initial_speed;
			  HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, 1);
			  HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, 0);

			  htim2.Instance->CCR2 = initial_speed;
			  HAL_GPIO_WritePin(in3_GPIO_Port, in3_Pin, 1);
			  HAL_GPIO_WritePin(in4_GPIO_Port, in4_Pin, 0);

			  oper_flag = 0;
		   }
		   else if(UART_flag == 1){
			  adjust_motor_speed();
			  UART_flag = 0;
		   }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 18-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 18-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 36-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, laser_Pin|cleaningmotor_Pin|vacuum_Pin|windmill_Pin
                          |in1_Pin|in2_Pin|in3_Pin|in4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : start_Pin */
  GPIO_InitStruct.Pin = start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(start_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : laser_Pin cleaningmotor_Pin vacuum_Pin windmill_Pin
                           in1_Pin in2_Pin in3_Pin in4_Pin */
  GPIO_InitStruct.Pin = laser_Pin|cleaningmotor_Pin|vacuum_Pin|windmill_Pin
                          |in1_Pin|in2_Pin|in3_Pin|in4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : user_key_Pin */
  GPIO_InitStruct.Pin = user_key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(user_key_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Crash_L_Pin Crash_R_Pin */
  GPIO_InitStruct.Pin = Crash_L_Pin|Crash_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
