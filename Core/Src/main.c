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
//#include "spi.h"
//#include "tim.h"
#include "bsp_SysTick.h"
#include "gpio.h"
//#include "fsmc.h"
#include "bsp_exti.h"
#include "bsp_led.h"
#include "bsp_ili9341_lcd.h"
//#include "bsp_spi_flash.h"
#include "bsp_basic_tim.h"

/* Private includes ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

volatile uint32_t time = 0; // ms 
volatile uint8_t cnt = 0;
extern TIM_HandleTypeDef htimx;

int main(void)
{
  HAL_Init();
  SystemClock_Config();

	//SysTick_Init();
	BASIC_TIMx_Init();
  MX_GPIO_Init();
  //MX_TIM4_Init();
  //MX_FSMC_Init();
	//LED_GPIO_Config();	
	EXTI_Key_Config();	
	ILI9341_Init();
	
	// xyf: set the display mode
  ILI9341_GramScan ( 4 );
	LCD_SetFont(&Font24x32);
  /* USER CODE END 2 */
	ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	
	
	LCD_SetColors(BLACK,WHITE);
	ILI9341_SetPointPixel(1,1);
	ILI9341_SetPointPixel(1,2);
	ILI9341_SetPointPixel(1,3);
	ILI9341_SetPointPixel(1,4);
	ILI9341_SetPointPixel(1,5);
	//EXTI_Key_Config();
	//HAL_TIM_PeriodElapsedCallback(&htimx);
//HAL_TIM_Base_Start_IT(&htimx);
  while (1)
  {
		if(time % 100000 == 0){
			//ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	
			Chinese_Show();
		
		}
		time++;
		//HAL_TIM_Base_Start_IT(&htimx);
		//HAL_TIM_PeriodElapsedCallback(&htimx);

  }
}
extern uint16_t lcdid;

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
