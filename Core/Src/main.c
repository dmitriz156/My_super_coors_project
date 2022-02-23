/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"



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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

uint16_t curentS = 10;
uint16_t pusword[4] = {1, 2, 3, 4};
uint16_t change_pusword[4] = {0};
uint16_t curent_pusword[4] = {0};
char simbol_pus [4];
uint16_t count = 0;
uint16_t flag = 0;
uint16_t flag_enter = 0;
uint16_t flag_change_pas = 0;
uint16_t FC = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ComparePusword(){

	for(int i = 0; i <= 3; ++i)
	{
		if (pusword[i] != curent_pusword[i])
		{
			TIM2->CCR1 = 1200;
			flag = 0;
			for(int i = 0; i <= 3; ++i)
				curent_pusword[i] = 10;
			return;
		}
	}
	TIM2->CCR1 = 7200;
	flag = 1;
	for(int i = 0; i <= 3; ++i)
		curent_pusword[i] = 10;
}

void EnterPusword()
{
	if (count > 4)
		count = 0;
	else{
		if(flag == 0){
			curent_pusword[count-1] = curentS;
		}
		else{
			pusword[count-1] = curentS;
		}
	}
}
/*
void ChangePusword()
{
	if (count > 4){
		count = 0;
	}
	else
		pusword[count-1] = curentS;
}*/
/*
void EnterPusword()
{
	if(flag == 0){
		if (count > 4)
			count = 0;
		else{
			curent_pusword[count-1] = curentS;
		}
	}
	else
	{
		if (count > 4){
			count = 0;
		}
		else
			pusword[count-1] = curentS;
	}
}
*/



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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  SSD1306_Init (); // initialize the display
  SSD1306_Clear();
  SSD1306_GotoXY (10,10); // goto 10, 10
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  TIM2->CCR1 = 1200;

  SSD1306_DrawFilledRectangle(1, 1, 128, 64, 0);
  SSD1306_GotoXY (10, 10); // goto 10, 10
  SSD1306_Puts ("CLOSE", &Font_11x18, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	  EnterPusword();
	  if(flag_enter == 1)
	  {
		  ComparePusword();
		  flag_enter = 0;
	  }
	  if(flag == 1)
	  {
		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == 0){
			  while (flag_change_pas == 1){
				  SSD1306_DrawFilledRectangle(1, 1, 128, 64, 0);
				  SSD1306_GotoXY (10, 10); // goto 10, 10
				  SSD1306_Puts ("CHANGING", &Font_11x18, 1);
				  SSD1306_GotoXY (10, 30);
				  switch (count)
				  {
				  case 0:
					  SSD1306_Puts (" ", &Font_11x18, 1);
					  break;
				  case 1:
					  SSD1306_Puts ("#", &Font_11x18, 1);
					  break;
				  case 2:
					  SSD1306_Puts ("##", &Font_11x18, 1);
					  break;
				  case 3:
					  SSD1306_Puts ("###", &Font_11x18, 1);
					  break;
				  case 4:
					  SSD1306_Puts ("####", &Font_11x18, 1);
					  break;
				  }
				  SSD1306_UpdateScreen();
				  SSD1306_DrawFilledRectangle(1, 1, 128, 64, 0);

				  //ChangePusword();
				  EnterPusword();
			  }
		  }
		  else{
			  SSD1306_DrawFilledRectangle(1, 1, 128, 64, 0);
			  SSD1306_GotoXY (10, 10); // goto 10, 10
			  SSD1306_Puts ("OPEN", &Font_11x18, 1);
			  SSD1306_GotoXY (10, 30);
			  SSD1306_Puts (simbol_pus, &Font_11x18, 1);
			  SSD1306_UpdateScreen();
			  SSD1306_DrawFilledRectangle(1, 1, 128, 64, 0);
			  //SSD1306_Clear();
			  /*if(flag_enter == 1)
			  {
				  flag_enter = 0;
			  }*/
		  }

	  	}
	  	else
	  	{
	  		if(flag_change_pas == 1){
	  			flag_change_pas = 0;
	  		}
	  		SSD1306_DrawFilledRectangle(1, 1, 128, 64, 0);
	  		SSD1306_GotoXY (10, 10); // goto 10, 10
	  		SSD1306_Puts ("CLOSE", &Font_11x18, 1);
	  		SSD1306_GotoXY (10, 30);
	  		switch (count)
	  		{
	  		case 0:
	  			SSD1306_Puts (" ", &Font_11x18, 1);
	  			break;
	  		case 1:
	  			SSD1306_Puts ("#", &Font_11x18, 1);
	  			break;
	  		case 2:;
	  			SSD1306_Puts ("##", &Font_11x18, 1);
	  			break;
	  		case 3:
	  			SSD1306_Puts ("###", &Font_11x18, 1);
	  			break;
	  		case 4:
	  			SSD1306_Puts ("####", &Font_11x18, 1);
	  			break;
	  		}
	  		SSD1306_UpdateScreen();
	  		//SSD1306_Clear();
	  		SSD1306_DrawFilledRectangle(1, 1, 128, 64, 0);
	  	}




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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim2.Init.Prescaler = 22;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65450;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 22;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65450;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA2 PA8 PA9 PA10
                           PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB3 PB4 PB5
                           PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/*штука яка повиннв обробляти переривання*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_6 || GPIO_Pin == GPIO_PIN_5 || GPIO_Pin == GPIO_PIN_4 || GPIO_Pin == GPIO_PIN_3 || GPIO_Pin == GPIO_PIN_2 || GPIO_Pin == GPIO_PIN_12 || GPIO_Pin == GPIO_PIN_11 || GPIO_Pin == GPIO_PIN_10 || GPIO_Pin == GPIO_PIN_9 || GPIO_Pin == GPIO_PIN_8 || GPIO_Pin == GPIO_PIN_7 || GPIO_Pin == GPIO_PIN_14)
	{
		//flag = 1;
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); // сразу же отключаем прерывания на этом пине
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		HAL_NVIC_DisableIRQ(EXTI4_IRQn);
		HAL_NVIC_DisableIRQ(EXTI3_IRQn);
		HAL_NVIC_DisableIRQ(EXTI2_IRQn);
		// либо выполняем какое-то действие прямо тут, либо поднимаем флажок
		HAL_TIM_Base_Start_IT(&htim3); // запускаем таймер

	} else {
		//flag = 0;
		//__NOP();

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		HAL_TIM_Base_Stop_IT(&htim3); // останавливаем таймер
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);  // очищаем бит EXTI_PR (бит прерывания)
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);

		NVIC_ClearPendingIRQ(EXTI15_10_IRQn); // очищаем бит NVIC_ICPRx (бит очереди)
		NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
		NVIC_ClearPendingIRQ(EXTI4_IRQn);
		NVIC_ClearPendingIRQ(EXTI3_IRQn);
		NVIC_ClearPendingIRQ(EXTI2_IRQn);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);   // включаем внешнее прерывание
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
		HAL_NVIC_EnableIRQ(EXTI2_IRQn);
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0){
			curentS = 1;
			count ++;
		}
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0){
			curentS = 2;
			count ++;
		}
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0){
			curentS = 3;
			count ++;
		}
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0){
			curentS = 4;
			count ++;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0){
			curentS = 5;
			count ++;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 0){
			curentS = 6;
			count ++;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0){
			curentS = 7;
			count ++;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0){
			curentS = 8;
			count ++;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0){
			curentS = 9;
			count ++;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 0){
			curentS = 0;
			count ++;
		}
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0){
			flag_enter = 1;
			count = 0;
		}//enter
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == 0){
			if(flag_change_pas == 1){
				flag_change_pas = 0;
			}
			else
				flag_change_pas = 1;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
