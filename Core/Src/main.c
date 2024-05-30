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
#include "math.h"
#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R_divider1 10e3
#define R_divider2 6e6
#define R_divider3 100e3
#define R_divider4 1e6
#define Vref 1.224
#define Vmait 3.3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
COMP_HandleTypeDef hcomp2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern int counter_duration;
extern int counter_PIM;
uint8_t Duration_int;
extern int counter_atvaizdavimas;
float talpa[20];
uint16_t TIM6Counter1;
uint16_t TIM6Counter2;
uint8_t i=0, n=0;
float galutinis=0, suma=0, duration;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_COMP2_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
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
  MX_USART2_UART_Init();
  MX_COMP2_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
/*Inicializacija ------START------*/
		/* Start COMP1 */
		if(HAL_COMP_Start(&hcomp2) != HAL_OK)
		{
			/* Initialization Error */
			Error_Handler(); 
		}
		/*Isvesti logini 1 ir pradeti skaiciavima*/
		HAL_TIM_Base_Start(&htim6);
		HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_SET);
		counter_duration=0;
		counter_PIM=0;
		TIM6Counter1=__HAL_TIM_GET_COUNTER(&htim6);
		/*Isvesti duomenu perdavimo i pc*/
		uint8_t TxBuffer[30];
		sprintf((char *)TxBuffer,"Initialising.....  \n\r");
		if (HAL_UART_Transmit(&huart2,TxBuffer,strlen(TxBuffer),1000)!=HAL_OK)
		{
		 Error_Handler();
		};
/*Inicializacija ------END------*/
		/*isvesti pradini ekrana*/
		HAL_GPIO_WritePin(OLED_VCC_GPIO_Port, OLED_VCC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GND_GPIO_Port, GND_Pin, GPIO_PIN_RESET);
		SSD1306_Init (); // initialize the display 

		SSD1306_GotoXY (10,10); // goto 10, 10 
		SSD1306_Puts ("START", &Font_11x18, 1); // print 
		SSD1306_UpdateScreen(); // update screen
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*Pradedamas impulsas ir taimeris*/
		if (counter_PIM==20*Duration_int)
		{
			HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_SET);
			counter_duration=0;
			__HAL_TIM_SET_COUNTER(&htim6,0);
			counter_PIM=0;
		}
		else if (counter_PIM>20*Duration_int)
		{
			HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_SET);
			counter_duration=0;
			__HAL_TIM_SET_COUNTER(&htim6,0);
			counter_PIM=0;
		}
		
		/*Atvaizdavimas*/
		/*UART*/
		if (counter_atvaizdavimas==1000)
		{
			for(suma=0,n=0,i=0;i<20;i++)
			{
				if(talpa[i]>0)
				{
					suma+=talpa[i];
					n++;
				}
			}
			//apskaiciuota uF
			galutinis=suma/n;
			sprintf((char *)TxBuffer,"C=%2.2f uF \n\r", galutinis);
			if (HAL_UART_Transmit(&huart2,TxBuffer,strlen(TxBuffer),1000)!=HAL_OK)
			{
			 Error_Handler();
			};
			counter_atvaizdavimas=0;
			
			i=0;
			/*Ekranas*/
			SSD1306_GotoXY (10,10); // goto 10, 10 
			SSD1306_Puts (TxBuffer, &Font_11x18, 1); // print 
			SSD1306_UpdateScreen(); // update screen
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INPUT_MINUS_1_2VREFINT;
  hcomp2.Init.NonInvertingInput = COMP_INPUT_PLUS_IO2;
  hcomp2.Init.LPTIMConnection = COMP_LPTIMCONNECTION_DISABLED;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Mode = COMP_POWERMODE_MEDIUMSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

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
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 50;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|S1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OLED_VCC_Pin|GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_VCC_Pin GND_Pin */
  GPIO_InitStruct.Pin = OLED_VCC_Pin|GND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : S1_Pin */
  GPIO_InitStruct.Pin = S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(S1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
  /* Prevent unused argument(s) compilation warning */
	Duration_int=counter_duration;
	TIM6Counter2=__HAL_TIM_GET_COUNTER(&htim6)-TIM6Counter1;
	duration=(1e6/640000)*TIM6Counter2;
	talpa[i]=1000*(-1*duration/(R_divider3*(log(1-((Vref*0.5)/Vmait)))));
	/*Isvesti logini 0*/
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_RESET);
	counter_PIM=0;
	i++;
  UNUSED(hcomp);
  
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_COMP_TriggerCallback should be implemented in the user file
   */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */

  UNUSED(GPIO_Pin);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
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
