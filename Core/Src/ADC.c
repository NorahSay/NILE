/* ****************************************************************************
 * @file		: ADC.c
 * @brief		: Establish I2C communication with ADS1115
 * project		: NILE
 * authors		: Norah Say (ns) - nsay@calpoly.edu
 * version		: 0.3
 * date			: 250304
 * compiler		: STM32CubeIDE v.1.15.0 Build:
 * target		: NUCLEO-L4A6ZG
 * @attention	: (c) 2024 STMicroelectronics. All rights reserved.
 ******************************************************************************
 * REVISION HISTORY
 * 0.1 250304 ns Create file
 * 0.2 250404 ns Added ADC_read, ADC_config - tested operational
 * 0.3 250409 ns Added get_pressure - not tested
 *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ADC.h"
#include "delay.h"
#include "math.h"

/* -----------------------------------------------------------------------------
* function : ADC_init()
* INs      : none
* OUTs     : none
* action   : Initialize I2C to communicate with ADC
* authors  : Norah Say (ns) - nsay@calpoly.edu
* version  : 0.2
* date     : 250304
* -----------------------------------------------------------------------------
* REVISION HISTORY
* 0.1 250304 Configure SPI pins and configure I2C
* 0.2 250404 Initialize ADC to A0
* 0.3 250410 Remove initialize to A0
* -----------------------------------------------------------------------------
* SPI pins configuration:
	 * SCL (PB8) - floating, open-drain, high speed
	 * SDA (PB9) - floating, open-drain, high speed
* -----------------------------------------------------------------------------*/
void ADC_init(void) {
	// Configure SPI pins
	SysTick_Init();

	RCC->AHB2ENR |= ( RCC_AHB2ENR_GPIOBEN); // Enable clock access to GPIOB
	GPIOB->MODER &= ~( GPIO_MODER_MODE8 | GPIO_MODER_MODE9);	// Clear mode register
	GPIOB->MODER |= ( GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1); // Set PB8 and PB9 to AF
	GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9); 		// Set to open-drain
	GPIOB->PUPDR &= ~( GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9); 	// Using external Pull-Up
	GPIOB->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED8_Pos |
					   0x2 << GPIO_OSPEEDR_OSPEED9_Pos);

	GPIOB->AFR[1] &= ~( GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9);
	GPIOB->AFR[1] |= (0x4 << GPIO_AFRH_AFSEL8_Pos |
					  0x4 << GPIO_AFRH_AFSEL9_Pos );		// Set PB8 and PB9 for I2C pin
	// Configure I2C
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;  	// enable I2C bus clock
	I2C1->CR1 &= ~( I2C_CR1_PE);        	// put I2C into reset (release SDA, SCL)
	I2C1->CR1 &= ~( I2C_CR1_ANFOFF);    	// Analog noise filter enabled
	I2C1->CR1 &= ~( I2C_CR1_DNF);       	// Digital noise filter disabled
	I2C1->TIMINGR = 0x60303D5B;         	// 4kHz SYSCLK timing from CubeMX
	I2C1->CR2 &= ~( I2C_CR2_AUTOEND);   	// Manual send STOP
	I2C1->CR2 &= ~( I2C_CR2_ADD10);     	// 7-bit address mode
	I2C1->CR1 |= ( I2C_CR1_PE); 			// enable I2C

	ADC_config(A0);
	ADC_read();
}

/* -----------------------------------------------------------------------------
* function : ADC_config()
* INs      : Register pointer address
* OUTs     : None
* action   : Configure the ADC to read from a specific analog pin.
* 			 Takes 10.2ms with 4kHz clock.
* authors  : Norah Say (ns) - nsay@calpoly.edu
* version  : 0.1
* date     : 250403
* -----------------------------------------------------------------------------
* REVISION HISTORY
* 0.1 250403 ns Created function - tested operational
* -------------------------------------------------------------------------- */
void ADC_config(uint8_t addr){
	// Establish ADC transaction
	while (I2C1->ISR & I2C_ISR_BUSY);
	I2C1->CR2 = 0;
	I2C1->CR2 &= ~( I2C_CR2_RD_WRN); 		// set WRITE mode
	I2C1->CR2 &= ~( I2C_CR2_SADD); 			// clear device address
	I2C1->CR2 |= ( ADC_ADDRESS << 1) | 		// Set target address
				 (3 << I2C_CR2_NBYTES_Pos)|
				 I2C_CR2_START; 			// Start read
	I2C1->TXDR = (CONFIG_R); 				// xmit config address
	while(!(I2C1->ISR & I2C_ISR_TXE)) ; 	// Wait for TXDR to empty
	I2C1->TXDR = (addr);					// xmit analog pin address
	while(!(I2C1->ISR & I2C_ISR_TXE)) ;		// Wait for TXDR to empty
	I2C1->TXDR = (0x83); //xmit LSB
	while(!(I2C1->ISR & I2C_ISR_TXE)) ;		// Wait for TXDR to empty

	// Initiate STOP condition
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)) ;
	I2C1->ICR |= I2C_ICR_STOPCF; 			// Clear stop flag
}

/* -----------------------------------------------------------------------------
* function : ADC_read()
* INs      : NONE
* OUTs     : return 16-bit raw data
* action   : Read analog value from ADS1115. Takes 13.24ms with 4kHz clock.
* authors  : Norah Say (ns) - nsay@calpoly.edu
* version  : 0.1
* date     : 240521
* -----------------------------------------------------------------------------
* REVISION HISTORY
* 0.1 250404 ns Created function - tested operational
* -------------------------------------------------------------------------- */
uint16_t ADC_read(){

	uint16_t data;

	// Write conversion register pointer
	while(I2C1->ISR & I2C_ISR_BUSY);		// Wait if bus is busy
	I2C1->CR2 = 0;
	I2C1->CR2 |= ( ADC_ADDRESS << 1) | 		// Set target address
				 (1 << I2C_CR2_NBYTES_Pos)|
				 I2C_CR2_START; 			// Start WRITE
	I2C1->TXDR = (CONV_R); 					// xmit conversion register
	while(!(I2C1->ISR & I2C_ISR_TXE)) ; 	// Wait for TXDR to empty
	I2C1->CR2 |= I2C_CR2_STOP;				// STOP condition
	while(!(I2C1->ISR & I2C_ISR_STOPF)) ;	// Clear STOP detection Flag

	// Read 2 bytes from conversion register
	while(I2C1->ISR & I2C_ISR_BUSY);			// Wait if bus is busy
	I2C1->CR2 = 0;
	I2C1->CR2 |= ((ADC_ADDRESS << 1) | 0X1) |	// Read from target address
				 (2 << I2C_CR2_NBYTES_Pos) |	// Set to read 2bytes
				 I2C_CR2_RD_WRN |				// set READ mode
				 I2C_CR2_START;					// Start read
	while(!(I2C1->ISR & I2C_ISR_RXNE));		// Wait for RX data to be available
	data = (I2C1->RXDR) << 8;				// Read and store MSB of data
	data |= I2C1->RXDR;						// Read and store LSB of data
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)) ;	// Clear STOP detection flag

	delay_us(200);
	return data;
}


/* -----------------------------------------------------------------------------
* function : get_pressure()
* INs      : Register pointer address
* OUTs     : Return pressure data in psi
* action   : Convert 16-bit data to mV
* authors  : Norah Say (ns) - nsay@calpoly.edu
* version  : 0.3
* date     : 250403
* -----------------------------------------------------------------------------
* REVISION HISTORY
* 0.1 250409 ns Created function
* 0.2 250410 ns Translate raw data to psi
* 0.3 250416 ns Modify psi conversion
* -------------------------------------------------------------------------- */
uint16_t get_pressure(uint8_t addr){
	uint16_t pressure;
	uint16_t voltage;

	ADC_config(addr);
	ADC_read();					//discard this
	voltage = ADC_read() * 1885 /10000;		//valid data - in mV
	pressure = voltage * 1600 / 4000 - 193;	// Convert to pressure

	return pressure;
}

