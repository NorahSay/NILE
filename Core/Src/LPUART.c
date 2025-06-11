/* ****************************************************************************
 * @file		: LPUART.c
 * @brief		:
 * project		: EE 329 Assignment 7
 * authors		: Norah Say (ns) - nsay@calpoly.edu
 * version		: 0.1
 * date			: 240507
 * compiler		: STM32CubeIDE v.1.15.0 Build:
 * target		: NUCLEO-L4A6ZG
 * @attention	: (c) 2024 STMicroelectronics. All rights reserved.
 ******************************************************************************
 * REVISION HISTORY
 * 0.1 240507 ns
 *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "LPUART.h"
#include "main.h"

/* -----------------------------------------------------------------------------
* function : LPUART_init()
* INs      : none
* OUTs     : none
* action   : Initialize LPUART1 and config Tx (PG7) and Rx (PG8) pin
* authors  : Norah Say (ns) - nsay@calpoly.edu
* version  : 0.1
* date     : 240503
* -----------------------------------------------------------------------------
* REVISION HISTORY
* 0.1 240503 ns		Imported code from S'24 lab manual, Config Tx and Rx pin
* -------------------------------------------------------------------------- */
void LPUART_init(void) {
	PWR->CR2 |= (PWR_CR2_IOSV);            // power avail on PG[15:2] (LPUART1)
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOGEN);   // enable GPIOG clock
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; // enable LPUART clock bridge

	/*configure PG7(TX) and PG8(Rx) - AF/push-pull/no pull-up, no pull-down*/
	GPIOG->MODER &= ~( GPIO_MODER_MODE7 | GPIO_MODER_MODE8);
	GPIOG->MODER |= ( GPIO_MODER_MODE7_1 |  GPIO_MODER_MODE8_1 );
	GPIOG->OTYPER &= ~( GPIO_OTYPER_OT7 |  GPIO_OTYPER_OT8 );
	GPIOG->PUPDR &= ~( GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8 );
	GPIOG->OSPEEDR |= ((3 << GPIO_OSPEEDR_OSPEED7_Pos)|
					   (3 << GPIO_OSPEEDR_OSPEED8_Pos));
	//Configure PG7 as Tx ( 4'b1000 )
	GPIOG->AFR[0] |= GPIO_AFRL_AFSEL7_3;
	GPIOG->AFR[0] &= ~GPIO_AFRL_AFSEL7_2;
	GPIOG->AFR[0] &= ~GPIO_AFRL_AFSEL7_1;
	GPIOG->AFR[0] &= ~GPIO_AFRL_AFSEL7_0;

	//Configure PG8 as Rx ( 4'b1000 )
	GPIOG->AFR[1] |=  GPIO_AFRH_AFSEL8_3;
	GPIOG->AFR[1] &= ~GPIO_AFRH_AFSEL8_2;
	GPIOG->AFR[1] &= ~GPIO_AFRH_AFSEL8_1;
	GPIOG->AFR[1] &= ~GPIO_AFRH_AFSEL8_0;

	LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // 8-bit data
	LPUART1->CR1 |= USART_CR1_UE;                   // enable LPUART1
	LPUART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // enable xmit & recv
	LPUART1->CR1 |= USART_CR1_RXNEIE;        // enable LPUART1 recv interrupt
	LPUART1->ISR &= ~(USART_ISR_RXNE);       // clear Recv-Not-Empty flag
	LPUART1->BRR = (0x22B9);				// Set baud rate to 115.2kbps
	NVIC->ISER[2] = (1 << (LPUART1_IRQn & 0x1F));   // enable LPUART1 ISR
	__enable_irq();                          // enable global interrupts
}

/* -----------------------------------------------------------------------------
* function : LPUART_Print()
* INs      : message - a string to be transmitted
* OUTs     : none
* action   : Serial transmission of a string to Data Register
* authors  : Norah Say (ns) - nsay@calpoly.edu
* version  : 0.1
* date     : 240503
* -----------------------------------------------------------------------------
* REVISION HISTORY
* 0.1 240503 ns		Imported code from S'24 lab manual
* -------------------------------------------------------------------------- */
void LPUART_Print( const char* message) {
   uint16_t iStrIdx = 0;
   while ( message[iStrIdx] != 0 ) {		//End if NULL char detected
      while(!(LPUART1->ISR & USART_ISR_TXE)); // wait for empty xmit buffer
      LPUART1->TDR = message[iStrIdx];       // send this character
      iStrIdx++;                             // advance index to next char
   }
}

/* -----------------------------------------------------------------------------
* function : LPUART1_ESC_Print()
* INs      : accepts VT100 Escape code
* OUTs     : none
* action   : Transmit VT100 Escape codes to Data Register
* authors  : Norah Say (ns) - nsay@calpoly.edu
* version  : 0.1
* date     : 240507
* -----------------------------------------------------------------------------
* REVISION HISTORY
* 0.1 240507 ns		Concatenate 'ESC' code to the message then pass to
* 					LPUART_Print()
* -------------------------------------------------------------------------- */
void LPUART_ESC_Print(const char* message) {
	char str[strlen(message)+7];

	sprintf(str,"\x1B[%s",message);
	LPUART_Print(str);
}

/* -----------------------------------------------------------------------------
* function : LPUART1_ESC_Print()
* INs      : accepts VT100 Escape code
* OUTs     : none
* action   : Transmit VT100 Escape codes to Data Register
* authors  : Norah Say (ns) - nsay@calpoly.edu
* version  : 0.1
* date     : 240507
* -----------------------------------------------------------------------------
* REVISION HISTORY
* 0.1 240507 ns		Concatenate 'ESC' code to the message then pass to
* 					LPUART_Print()
* -------------------------------------------------------------------------- */

void Terminal_Init(void) {
LPUART_ESC_Print("H"); //Moves cursor to origin
LPUART_ESC_Print("2J"); //clear screen
}





