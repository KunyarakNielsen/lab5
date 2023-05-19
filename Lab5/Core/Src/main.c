/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include <locale.h>
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint8_t RXbuffer[20];
uint8_t TXbuffer[1000];


uint8_t status = 0;


uint32_t ledhz = 1;

int8_t ledonoff = 1;
uint8_t state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  sprintf((char*)TXbuffer,
		  "Choose which of this topics\r\n 0: LED Control\r\n 1: Button Status\r\n\r\n\r\n\r\n\r\n");

  HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));


  UARTDMAConfig();
  while (1)
  {

	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
	  menu();
	  ledhztask();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UARTDMAConfig(){
	HAL_UART_Receive_DMA(&huart2, RXbuffer, 1);
}

void ledhztask()
{
	uint32_t sec;
	sec = 1000 / (2 * ledhz);
	static uint32_t timestamp = 0;
	if(ledonoff == 1)
	{
		if(HAL_GetTick() >= timestamp)
			{
				timestamp = HAL_GetTick() + sec;
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			}
	}
	else if(ledonoff == 0)
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}

}


void menu()
{
	switch (status)
	{

		case 0:

			if(RXbuffer[0] == '0')
			{
				status = 1;
				sprintf((char*)TXbuffer,
						"LED_CONTROL MODE \r\n a: Gain Hz \r\n s: Reduce Hz \r\n d: ON/OFF button \r\n x: Back to menu \r\n");
				HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
				RXbuffer[0] = '\000';
			}

			else if(RXbuffer[0] == '1')
			{
				status = 2;
				sprintf((char*)TXbuffer,
						"BUTTON_STATUS MODE \r\n PRESS USER BUTTON: PRESS \r\n LEFT USER BUTTON: UNPRESS \r\n x: Back to menu \r\n");
				HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
				RXbuffer[0] = '\000';
			}

			else if(RXbuffer[0] != '0' && RXbuffer[0] != '1' && RXbuffer[0] != '\000')
			{
				RXbuffer[0] = '\000';
				sprintf((char*)TXbuffer,"Please press the right button \r\n");
				HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
			}

			break;


		case 1:
			if(RXbuffer[0] == 'a')
			{
				ledhz += 1;
				sprintf((char*)TXbuffer,"---> HZ (+) %d\r\n", ledhz);
				HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
				RXbuffer[0] = '\000';
			}

			else if(RXbuffer[0] == 's')
			{
				ledhz -= 1;
				sprintf((char*)TXbuffer,"<--- HZ (-) %d\r\n", ledhz);
				HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
				RXbuffer[0] = '\000';
				if(ledhz <= 1)
				{
					ledhz = 1;
				}
			}

			else if(RXbuffer[0] == 'd')
			{
				if(ledonoff == 0)
				{
					ledonoff = 1;
					sprintf((char*)TXbuffer,"ON LIGHT\r\n");
					HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
				}
				else if(ledonoff == 1)
				{
					ledonoff = 0;
					sprintf((char*)TXbuffer,"OFF LIGHT\r\n");
					HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
				}
				RXbuffer[0] = '\000';
			}

			else if(RXbuffer[0] == 'x')
			{
				sprintf((char*)TXbuffer,
						"\r\n\r\n\r\n\r\n\r\nMain Menu\r\n 0: LED Control\r\n 1: Button Status\r\n\r\n\r\n\r\n\r\n");
				HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
				status = 0;
				RXbuffer[0] = '\000';
			}

			else if(RXbuffer[0] != 'a' && RXbuffer[0] != 's' && RXbuffer[0] != 'd' && RXbuffer[0] != 'x' && RXbuffer[0] != '\000' && RXbuffer[0] != '\0'){
				RXbuffer[0] = '\000';
				sprintf((char*)TXbuffer,"Please press the right button \r\n");
				HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
			}

			break;


		case 2:
			state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
			static uint32_t timestamp2 = 0;
			if(HAL_GetTick()>=timestamp2)
			{
				timestamp2 = HAL_GetTick() + 1000;
				if(!state)
				{
					sprintf((char*)TXbuffer,"Press \r\n");
					HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
				}

				else if(state)
				{
					sprintf((char*)TXbuffer,"Unpress \r\n");
					HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
				}
			}

			if(RXbuffer[0] == 'x')
			{
				sprintf((char*)TXbuffer,"\r\n\r\n\r\n\r\n\r\nMain Menu\r\n 0: LED Control\r\n 1: Button Status\r\n\r\n\r\n\r\n\r\n");
				HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
				status = 0;
				RXbuffer[0] = '\000';
			}

			else if(RXbuffer[0] != 'x' && RXbuffer[0] != '\000' && RXbuffer[0] != '\0')
			{
				sprintf((char*)TXbuffer,"Please press the right button \r\n");
				HAL_UART_Transmit_DMA(&huart2, TXbuffer, strlen((char*)TXbuffer));
				RXbuffer[0] = '\000';
			}

			break;

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
