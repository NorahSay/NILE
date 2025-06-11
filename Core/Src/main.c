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
 *****************************************************************************/



/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ADC.h"
#include "delay.h"
#include "HX711.h"
#include "stm32l4xx_hal.h"
#include "kalman.h"
#include "solenoid_driver.h"
#include "LPUART.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
uint16_t A0_data;
uint16_t A1_data;
uint16_t A2_data;
uint16_t A3_data;
hx711_t loadcell[4] ={0};
kalman_filter_t kf;
int32_t w_load[4];
int32_t no_load[4];
float weight[4];
int32_t load_array[200];
uint16_t i = 0;
char str[8];



void SystemClock_Config(void);
void setup_GPIO_PORTB(void);
void setup_LC_port(void);

int main(void)
{
	  /* Configure and initialize all peripherals */
	  HAL_Init();
	  SysTick_Init();
	  SystemClock_Config();
	  LPUART_init();
	  Terminal_Init();

	  setup_LC_port();
//	  kalman_init(&kf,Q,I,initial_val);
	  //ADC_init();
	  //MX_USART2_UART_Init();

//	  A0_data = get_pressure(A0);
//	  A1_data = get_pressure(A1);
//	  A2_data = get_pressure(A2);
//	  A3_data = get_pressure(A3);


	  hx711_init(&loadcell[0], GPIOA, GPIO_PIN_2, GPIOA, GPIO_PIN_1);
	  hx711_init(&loadcell[1],GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_4);
	  hx711_init(&loadcell[2], GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_3);
	  hx711_init(&loadcell[3],GPIOA, GPIO_PIN_0, GPIOB, GPIO_PIN_0);


	  for(i = 0; i < 4; i++) {
		  hx711_tare(&loadcell[i], 20);
		  w_load[i] = hx711_value_ave(&loadcell[i],20);
	  }
	  for(i = 0; i < 4; i++) {
		  no_load[i] = hx711_value_ave(&loadcell[i],20);
	  }
	  for(i = 0; i < 4; i++) {
		  hx711_calibration(&loadcell[i],no_load[i],w_load[i],6);
	  }

	  while (1)
	  {
//	    if (i < 200) {
//	    	load_array[i] = hx711_value(&loadcell);
//	    	sprintf(str,"%d",load_array[i]);
//	    	LPUART_Print(str);
//	    	LPUART_ESC_Print("1B");
//	    	LPUART_ESC_Print("8D");
//	    	i++;
//	    }

		weight[0] = hx711_weight(&loadcell[2], 10);
	    weight[1] = hx711_weight(&loadcell[1], 10);
	    weight[2] = hx711_weight(&loadcell[2], 10);
	    weight[3] = hx711_weight(&loadcell[2], 10);
	  }
}
void setup_LC_port(void) {
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
}

float hx711_value_ave_filtered(hx711_t *hx711, kalman_filter_t *kf, uint16_t sample)
{
    float raw_value_ave = hx711_value_ave(hx711, sample);
    return kalman_update(kf, raw_value_ave);
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
