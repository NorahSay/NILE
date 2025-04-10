/*******************************************************************************
 * NILE Test Stand Data Acquisition and Control Unit
 *******************************************************************************
 * @file           : main.c
 * @brief          : Acquire pressure and weight data from the test stand and
 * 					 display them onto a monitor. Also let user control the
 * 					 feed system remotely.
 * project         :
 * authors         : Norah Say (ns) - nsay@calpoly.edu
 * version         : 0.1
 * date            : 250304
 * compiler        : STM32CubeIDE v.1.15.0
 * target          : NUCLEO-L4A6ZG
 * clocks          : 4MHz
 ******************************************************************************
 * REVISION HISTORY
 * 0.1 250304 ns	Create Project
 *****************************************************************************
 * TODO:Debug issue with A1 and A3 not reading correctly when A1 and A2 are
 * 		enabled
 *****************************************************************************/



/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ADC.h"
#include "delay.h"

/* Private variables ---------------------------------------------------------*/
uint16_t A0_data;
uint16_t A1_data;
uint16_t A2_data;
uint16_t A3_data;


void SystemClock_Config(void);
void setup_GPIO_PORTB(void);

int main(void)
{
	  /* Configure and initialize all peripherals */
	  HAL_Init();
	  SysTick_Init();
	  SystemClock_Config();
	  setup_GPIO_PORTB();
	  ADC_init();


	  A0_data = get_pressure(A0);
	  A1_data = get_pressure(A1);
	  delay_us(5000);
	  A2_data = get_pressure(A2);
	  A3_data = get_pressure(A3);

  while (1)
  {

  }
  /* USER CODE END 3 */
}

void setup_GPIO_PORTB(void) {
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
	GPIOB->MODER &= ~( GPIO_MODER_MODE14 );
	GPIOB->MODER |= ( GPIO_MODER_MODE14_0 );
	GPIOB->OTYPER &= ~( GPIO_OTYPER_OT14 );
	GPIOB->PUPDR &= ~( GPIO_PUPDR_PUPD14 );
	GPIOB->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED14_Pos);
	GPIOB->BRR = (GPIO_PIN_14 );
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
