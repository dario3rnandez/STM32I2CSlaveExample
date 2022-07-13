/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "retarget.h"
                TXBUFFERSIZE

// emulated I2C RAM
static uint8_t ram[256];
static uint8_t offset; 	// index of current RAM cell
static uint8_t first=1;	// first byte --> new offset

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO uint32_t     Transfer_Direction = 0;
__IO uint32_t     Xfer_Complete = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart2);
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }
//  HAL_I2C_Slave_Receive_IT(&hi2c1,aRxBuffer,sizeof(aRxBuffer));  //Enable slave interrupt reception
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/* USER CODE BEGIN 4 */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("LCB\n");
	first = 1;
	HAL_I2C_EnableListen_IT(hi2c); // slave is ready again
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	printf("ACB %s\n", TransferDirection==I2C_DIRECTION_RECEIVE ? "rx" : "tx" );
//	HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET );
	if( TransferDirection==I2C_DIRECTION_TRANSMIT ) {
		if( first ) {
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, &offset, 1, I2C_NEXT_FRAME);
		} else {
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, &ram[offset], 1, I2C_NEXT_FRAME);
		}
	} else {
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &ram[offset], 1, I2C_NEXT_FRAME);
	}
//	HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET );
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
//	HAL_GPIO_WritePin( PA6_GPIO_Port, PA6_Pin, GPIO_PIN_SET );
	if(first) {
		printf("RXCB: offset <== %3d\n", offset );
		first = 0;
	} else {
		printf("RXCB: ram[%3d] <== %3d\n", offset,  ram[offset] );
		offset++;
	}
	HAL_I2C_Slave_Seq_Receive_IT(hi2c, &ram[offset], 1, I2C_NEXT_FRAME);
//	HAL_GPIO_WritePin(PA6_GPIO_Port, PA6_Pin, GPIO_PIN_RESET );
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
//	HAL_GPIO_WritePin( PA7_GPIO_Port, PA7_Pin, GPIO_PIN_SET );
	printf("TXCB: ram[%3d] ==> %3d\n", offset, ram[offset] );
	offset++;
	HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &ram[offset], 1, I2C_NEXT_FRAME);
//	HAL_GPIO_WritePin( PA7_GPIO_Port, PA7_Pin, GPIO_PIN_RESET );
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
//	HAL_GPIO_WritePin( PA9_GPIO_Port, PA9_Pin, GPIO_PIN_SET );
	if( HAL_I2C_GetError(hi2c)==HAL_I2C_ERROR_AF ) {
		// transaction terminated by master
		printf("ECB end\n" );
		offset--;
	} else {
		printf("ECB err=0x%02X\n", HAL_I2C_GetError(hi2c) );
	}
//	HAL_GPIO_WritePin( PA9_GPIO_Port, PA9_Pin, GPIO_PIN_RESET );
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("aborted\n" );  // never seen...
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
