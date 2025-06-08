/*****************************************************************************
 * @file           : ADC.h
 * @brief          : Header for ADC.c file.
 *                   This file contains all the defines of the application.
 *****************************************************************************
 * author		   : Karla Lira (kliragon@calpoly.edu)
 * project		   : NILE Test Stand
 * version         : 0.1
 * date            : 250424
 * compiler        : STM32CubeIDE v.1.18.0
 * target          : NUCLEO-L4A6ZG
 * clocks          : 4MHz
 *
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *******************************************************************************
 * REVISION HISTORY
 * 0.1 250424 kl	Create Project
 *
 ****************************************************************************/
#include "stm32l4xx.h" // Register Definintions

#ifndef SOLENOID_DRIVER_H_
#define SOLENOID_DRIVER_H_

//--------Defines--------//
#define SOLENOID_PORT (GPIOE)

#define SOLENOID_PIN_0 (1U << 2)
#define SOLENOID_PIN_1 (1U << 3)
#define SOLENOID_PIN_2 (1U << 4)
#define SOLENOID_PIN_3 (1U << 5)
#define SOLENOID_PIN_4 (1U << 6)
#define SOLENOID_PIN_5 (1U << 7)
#define SOLENOID_PIN_6 (1U << 8)
#define SOLENOID_PIN_7 (1U << 9)

#define SOLENOID_ALL_PINS (SOLENOID_PIN_0 | SOLENOID_PIN_1 | \
						   SOLENOID_PIN_2 | SOLENOID_PIN_3 | \
						   SOLENOID_PIN_4 | SOLENOID_PIN_5 | \
						   SOLENOID_PIN_6 | SOLENOID_PIN_7);


//---Function Prototypes---//
void solenoid_driver_Init(void);
void solenoid_on(uint8_t index);
void solenoid_off(uint8_t index);
void solenoid_toggle(uint8_t index);
void solenoid_all_off(void);

#endif /* SOLENOID_DRIVER_H_ */
