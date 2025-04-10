/* USER CODE BEGIN Header */
/*****************************************************************************
 * @file           : ADC.h
 * @brief          : Header for ADC.c file.
 *                   This file contains all the defines of the application.
 *****************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 ****************************************************************************/

#ifndef ADC_H_
#define ADC_H_

/* Exported functions prototypes ---------------------------------------------*/
void ADC_init(void);
void ADC_config(uint8_t addr);
uint16_t ADC_read();
uint8_t get_pressure(uint8_t addr);
/* Private Defines -----------------------------------------------------------*/
#define ADC_ADDRESS		(0x48)




#endif /* ADC_H_ */
