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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

	GPIOC->ODR ^= (1 << 8);
	GPIOC->ODR ^= (1 << 9);
	TIM2->SR &= ~TIM_SR_UIF;
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
	// Enable the timer 2 peripheral (TIM2) in the RCC
  // RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST; // wrong!
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable Timer 2 clock
	// Enable the timer 3 peripheral (TIM2) in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	// blue LED (PC7), channel 2, red LED PC6, Channel 1, green LED PC9, yellow LED PC8
	// channel 1: PC6
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to PC
	// RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
	// Set the enable/start bit to start the timer

	// Configure the timer to trigger an update event (UEV) at 4 Hz.
	TIM2->PSC = 7999;
	TIM2->ARR = 250;
	
	// 
		// Configure the timer to trigger an update event (UEV) at 800 Hz.
		// 
	TIM3->PSC = 99;
	TIM3->ARR = 100;
	// Bits 1:0 CC1S: Capture/Compare 1 selection, 00: CC1 channel is configured as output
	TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
  TIM3->CCMR1 &= ~TIM_CCMR1_CC2S;
	// set output channel 1 to PWM Mode 2 (111).
	TIM3->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0);
	// set output channel 2 to PWM mode 1 (110)
	TIM3->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
	TIM3->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
	// Enable the capture/compare output for channels 1 and 2
	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
	// Set CCR1 for channel 1 and 2
	TIM3->CCR1 = 10;
  TIM3->CCR2 = 10;
	// Use the DMA/Interrupt Enable Register (DIER) to enable the update interrupt.
	TIM2->DIER |= TIM_DIER_UIE;
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	
	// Enable the timer 2 interrupt in the NVIC
  NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 1); // Set interrupt priority
	
	// set the MODER, 01: General purpose output mode
	GPIOC->MODER |= (1 << 16);  // MODER8, 8 is the pin of PC8
	GPIOC->MODER |= (1 << 18);  // MODER9, 9 is the pin of PC9

	// set pc6 to AF mode, 0x10
	GPIOC->MODER |= (1 << 13);
	GPIOC->MODER &= ~(1 << 12);
	// set pc7 to AF mode, 0x10
	GPIOC->MODER |= (1 << 15);
	GPIOC->MODER &= ~(1 << 14);
	// set PC6 AFRL to 0000: AF0
	GPIOC->AFR[0] &= ~(0x0 << GPIO_AFRL_AFRL6_Pos);
	// set PC7 AFRL to 0000: AF0
	GPIOC->AFR[1] &= ~(0x0 << GPIO_AFRL_AFRL7_Pos);

	// Set the pins to low speed in the OSPEEDR register
	GPIOC->OSPEEDR &= ~((1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));

	// Set to no pull-up/down resistors in the PUPDR register
	//00: No pull-up, pull-down
	GPIOC->PUPDR &= ~((1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
	// GPIOC->PUPDR &= ~((1 << 12) | (1 << 13));
	// set PC8 to 0
	GPIOC->ODR &= ~(1 << 8);
	// set PC8 to 1
	GPIOC->ODR |= (1 << 9);

	//  Note that you should not enable a timer until you’ve finished setting all the basic
  // parameters and options.
	// Set the enable/start bit to start the timer
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
