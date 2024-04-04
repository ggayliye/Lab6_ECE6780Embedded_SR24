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

                uint16_t ADC_data = 0;

                uint8_t DAC_data = 0;

  /* USER CODE BEGIN SysInit */

                // Enable GPIOC clock

    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

                //Enable GPIOA clock

                                RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

                // Enable the clock for ADC1

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

                //Enable the clock for DAC1

                                RCC->APB1ENR |= RCC_APB1ENR_DACEN;

                // Initializing GPIO LEDs

                                GPIOC->MODER |= ((1<<12) | (1<<14) | (1<<16) | (1<<18));

                                GPIOC->OTYPER = 0;

                                GPIOC->OSPEEDR = 0;

                                GPIOC->PUPDR = 0;

                                GPIOC->ODR &= ~(1<<6);

                                GPIOC->ODR &= ~(1<<7);

                                GPIOC->ODR &= ~(1<<8);

                                GPIOC->ODR &= ~(1<<9);

  /* USER CODE END SysInit */

                                //Setting PC0

                                GPIOC->MODER |= (3<<0); //Setting Analog Mode

                                //Setting PA4 to Addtional Function Mode

                                GPIOA->MODER |= (3<<8);

                                GPIOA->PUPDR &= ~(3<<8);

                                //Setting Up ADC

                                ADC1->CFGR1 |= (1<<13); //Setting the ADC to continuous conversion mode

                                ADC1->CFGR1 |= (2<<3); //8 bit Data Resolution

                                ADC1->CFGR1 &= ~(3<<10); //No External Triggers

                                ADC1->CHSELR |= (1<<10);////Enabling the ADC to the PC0 input

                                if (((ADC1->CR & (1<<4)) == 0) & ((ADC1->CFGR1 & (1<<0)) == 0)) //Cheking if DMAEN and ADEN is 0

                                {

                                                ADC1->CR |= (1<<31); //Starting ADCAL

                                }

                                while((ADC1->CR & (1<<31))) //Waiting for calibration to complete

                                {

                                }                              

                                ADC1->CR |= (1<<0); // ADC Enable

                                while((ADC1->ISR & (1<<0)) == 0) //Checking ADSTART till ADC conversion is complete

                                {

                                }

                                ADC1->CR |= (1<<2); //ADC Start

                                //Setting Up DAC

                                DAC1->CR |= (1<<2); //Enable Triggering

                                DAC1->CR |= (7<<3); //Software Trigger

                                DAC1->CR |= (1<<0); // DAC1 enabled

                                // Triangle Wave: 8-bit, 32 samples/cycle

                                const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,

                                190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */

 

  /* USER CODE END 2 */

 

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  while (1)

  {

                                uint32_t i = 0; //Increment variable

    /* USER CODE END WHILE */
		
//// PART 1 STARTS HERE. Connect PC0, 3V, and ground to the potantiometer. the middle pin is PC0 (or the first pin and the middle can be 3V)
//                                if (ADC1->ISR & (1<<2))

//                                {

//                                                ADC_data = (ADC1->DR);

//                                }

//                                if (ADC_data <= 63)

//                                {

//                                                GPIOC->ODR |= (1<<6);

//                                                GPIOC->ODR &= ~(1<<7);

//                                                GPIOC->ODR &= ~(1<<8);

//                                                GPIOC->ODR &= ~(1<<9);

//                                }

//                                else if (ADC_data > 63 & ADC_data <= 126)

//                                {

//                                                GPIOC->ODR |= (1<<7);

//                                                GPIOC->ODR |= (1<<6);

//                                                GPIOC->ODR &= ~(1<<8);

//                                                GPIOC->ODR &= ~(1<<9);

//                                }

//                                else if (ADC_data > 126 & ADC_data <= 190)

//                                {

//                                                GPIOC->ODR |= (1<<8);

//                                                GPIOC->ODR |= (1<<6);

//                                                GPIOC->ODR |= (1<<7);

//                                                GPIOC->ODR &= ~(1<<9);

//                                }

//                                else if (ADC_data > 190 & ADC_data <=256)

//                                {

//                                                GPIOC->ODR |= (1<<9);

//                                                GPIOC->ODR |= (1<<6);

//                                                GPIOC->ODR |= (1<<7);

//                                                GPIOC->ODR |= (1<<8);

//                                }

//                                else

//                                {

//                                                GPIOC->ODR &= ~(1<<6);

//                                                GPIOC->ODR &= ~(1<<7);

//                                                GPIOC->ODR &= ~(1<<8);

//                                                GPIOC->ODR &= ~(1<<9);

//                                }
//																
	
	
	
	
//	PART 2. Link PA4 to Scope/logic analizer to observe triangle wave. Black goes to ground
                                while (i <= 31) //running through  

                                {

                                                DAC_data = triangle_table[i]; //getting individual array elements

                                                DAC->DHR8R1 = DAC_data; //??

                                                DAC1->SWTRIGR |= (1<<0);

                                                HAL_Delay(1); // 1 millisecond delay   

                                                i+=1;

                                }
																
	//END: PART 2 

                               

                                                               

                                               

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