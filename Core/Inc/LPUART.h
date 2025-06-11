/* USER CODE BEGIN Header */
/*****************************************************************************
 * @file           : LPUART.h
 * @brief          : Header for LPUART.c file.
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
 *
 ****************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _LPUART_H_
#define _LPUART_H_

/* Exported functions prototypes ---------------------------------------------*/
void LPUART_init(void);
void LPUART_Print( const char* );
void LPUART_ESC_Print( const char*);
void Terminal_Init(void);

#endif /* _LPUART_H_ */
