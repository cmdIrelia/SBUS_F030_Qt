/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define START_CHAR (0xDF)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t timer3_timeout = 0;

//0:0xdf  1:CH1_H 2:CH1_L 3:CH2_H 4:CH2_L 5:CH3_H 6:CH3_L 
//7:CH4_H 8:CH4_L 9:CH5_H 10:CH5_L 11:CH6_H 12:CH6_L
uint8_t uart2_recv_buff[UART2_RECV_BUFF_LEN];

uint8_t dataH[16],dataL[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SBUS_SendPkg(uint8_t *dataH, uint8_t* dataL);
void UART1_Snd_Uint8(uint8_t data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void UART1_Snd_Uint8(uint8_t data)
{
	HAL_UART_Transmit(&huart1,&data,1,1000);
	while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);
}

void SBUS_SendPkg(uint8_t *dataH, uint8_t* dataL)
{
	int i,j=0;
	uint8_t snd;
	uint16_t cur_channel;
	int channel_counter = 0;

	UART1_Snd_Uint8(0x0F);
	
	cur_channel = (dataH[channel_counter]<<8) | dataL[channel_counter];
	do {
		snd = 0;
		for (i = 0; i < 8; i++) {
			snd >>= 1;
			snd |= (cur_channel & 0x01) << 7;
			cur_channel >>= 1;
			j++;
			if (j == 11) {
				j = 0;
				channel_counter++;
				cur_channel = (dataH[channel_counter]<<8) | dataL[channel_counter];
			}
		}
		UART1_Snd_Uint8(snd);
		
	} while (channel_counter<16);
	
	UART1_Snd_Uint8(0x00);
	UART1_Snd_Uint8(0x00);
}

//Put the start Byte at the start of the pbuf
int FormatFrame(uint8_t *pbuf)
{
	int16_t i,start_at;
	uint8_t swapBuf[UART2_RECV_BUFF_LEN];
	start_at = -1;
	for(i=0;i<UART2_RECV_BUFF_LEN;i++)
	{
		if(pbuf[i]==START_CHAR)
		{
			start_at = i;
			break;
		}
	}
	if(start_at == -1) 
	{
		return -1;	//Not Found
	}
	for(i=0;i<start_at;i++)
	{
		swapBuf[UART2_RECV_BUFF_LEN-start_at+i]=pbuf[i];
	}
	for(i=start_at;i<UART2_RECV_BUFF_LEN;i++)
	{
		pbuf[i-start_at]=pbuf[i];
	}
	for(i=UART2_RECV_BUFF_LEN-start_at;i<UART2_RECV_BUFF_LEN;i++)
	{
		pbuf[i]=swapBuf[i];
	}
	return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t uart2_snd_buff[]={"Uart2 OK.\n"};
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_IT(&huart2,(uint8_t*)uart2_recv_buff,UART2_RECV_BUFF_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
  {
		if(timer3_timeout == 1)
		{
			//HAL_UART_Transmit(&huart2,uart2_snd_buff,sizeof(uart2_snd_buff),1000);
			//while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);
			int rtn = FormatFrame(uart2_recv_buff);
			if(uart2_recv_buff[0]==0xdf && rtn!=-1)
			{
				//0:0xdf  1:CH1_H 2:CH1_L 3:CH2_H 4:CH2_L 5:CH3_H 6:CH3_L 
				//7:CH4_H 8:CH4_L 9:CH5_H 10:CH5_L 11:CH6_H 12:CH6_L
				dataH[0] = uart2_recv_buff[1];
				dataL[0] = uart2_recv_buff[2];
				
				dataH[1] = uart2_recv_buff[3];
				dataL[1] = uart2_recv_buff[4];
				
				dataH[2] = uart2_recv_buff[5];
				dataL[2] = uart2_recv_buff[6];
				
				dataH[3] = uart2_recv_buff[7];
				dataL[3] = uart2_recv_buff[8];
				
				dataH[4] = uart2_recv_buff[9];
				dataL[4] = uart2_recv_buff[10];
				
				dataH[5] = uart2_recv_buff[11];
				dataL[5] = uart2_recv_buff[12];
				
				dataH[6] = uart2_recv_buff[13];
				dataL[6] = uart2_recv_buff[14];
			}
			SBUS_SendPkg(dataH,dataL);
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			timer3_timeout = 0;
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
