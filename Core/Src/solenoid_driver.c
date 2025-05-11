/*******************************************************************************
 * NILE Test Stand Solenoid Driver
 *******************************************************************************
 * @file           : solenoid_driver.c
 * @brief          : Will take user input and drive/pull output to ground
 * 					 when prompted.
 * project         :
 * authors         : Karla Lira (kl) - kliragon@calpoly.edu
 * version         : 0.1
 * date            : 250417
 * compiler        : STM32CubeIDE v.1.18.0
 * target          : NUCLEO-L4A6ZG
 * clocks          : 4MHz
 * @attention      : (c) 2023 STMicroelectronics.  All rights reserved.
 ******************************************************************************
 * REVISION HISTORY
 * 0.1 250417 kl	Create Project
 ****************************************************************************/

/******Includes*******/
#include "main.h"
#include "solenoid_driver.h"
#include "delay.h"


/* -----------------------------------------------------------------------------
* function : SolDriver_init()
* INs      : none
* OUTs     : none
* action   : Initialize solenoid driver
* authors  : Karla Lira (kl) - kliragon@calpoly.edu
* version  : 0.1
* date     : 250417
* -----------------------------------------------------------------------------
* REVISION HISTORY
* 0.1 250417 Configure driver input pins
* 0.2
* -----------------------------------------------------------------------------
* Driver input pins configuration:
	 * IN1 (PE2) - Output, push-pull, no pull-up/pull-down, high speed
	 * IN2 (PE3) - Output, push-pull, no pull-up/pull-down, high speed
	 * IN3 (PE4) - Output, push-pull, no pull-up/pull-down, high speed
	 * IN4 (PE5) - Output, push-pull, no pull-up/pull-down, high speed
	 * IN5 (PE6) - Output, push-pull, no pull-up/pull-down, high speed
	 * IN6 (PE7) - Output, push-pull, no pull-up/pull-down, high speed
	 * IN7 (PE8) - Output, push-pull, no pull-up/pull-down, high speed
	 * IN8 (PE9) - Output, push-pull, no pull-up/pull-down, high speed
* -----------------------------------------------------------------------------*/
void solenoid_driver_Init(void){

	// Enable GPIOE clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOEEN);


	// configure GPIO pins PE2-PE9 for:
	// output mode, push-pull, no pull up or pull down, high speed
	for (uint8_t i = 2; i <= 9; i++) {
	        SOLENOID_PORT->MODER &= ~(0x3 << (i * 2));   // Clear mode
	        SOLENOID_PORT->MODER |=  (0x1 << (i * 2));   // Set to output mode
	        SOLENOID_PORT->OTYPER &= ~(1U << i);         // Push-pull
	        SOLENOID_PORT->PUPDR  &= ~(0x3 << (i * 2));  // No pull
	        SOLENOID_PORT->OSPEEDR |= (0x3 << (i * 2));  // High speed
	    }

	GPIOE->BSRR = SOLENOID_ALL_PINS;  // Drives PE2–PE9 HIGH (inactive state)

}

/* -----------------------------------------------------------------------------
* function : solenoid_on()
* INs      : index - solenoid number (0–7)
* OUTs     : none
* action   : Turns ON solenoid (active-low logic)
* authors  : Karla Lira (kl)
* version  : 0.1
* date     : 250417
* -----------------------------------------------------------------------------
*/
void solenoid_on(uint8_t index) {
    if (index < 8) {
        SOLENOID_PORT->BRR = (1U << (index + 2));
        delay_us(25000);  // debounce
    }
}

/* -----------------------------------------------------------------------------
* function : solenoid_off()
* INs      : index - solenoid number (0–7)
* OUTs     : none
* action   : Turns OFF solenoid (drives output HIGH)
* authors  : Karla Lira (kl)
* version  : 0.1
* date     : 250417
* -----------------------------------------------------------------------------
*/
void solenoid_off(uint8_t index) {
    if (index < 8) {
        SOLENOID_PORT->BSRR = (1U << (index + 2));
    }
}

/* -----------------------------------------------------------------------------
* function : solenoid_toggle()
* INs      : index - solenoid number (0–7)
* OUTs     : none
* action   : Toggles solenoid state
* authors  : Karla Lira (kl)
* version  : 0.1
* date     : 250417
* -----------------------------------------------------------------------------
*/
void solenoid_toggle(uint8_t index) {
    if (index < 8) {
        SOLENOID_PORT->ODR ^= (1U << (index + 2));
    }
}

/* -----------------------------------------------------------------------------
* function : solenoid_all_off()
* INs      : none
* OUTs     : none
* action   : Turns OFF all solenoids (outputs HIGH)
* authors  : Karla Lira (kl)
* version  : 0.1
* date     : 250417
* -----------------------------------------------------------------------------
*/
void solenoid_all_off(void) {
    SOLENOID_PORT->BSRR = SOLENOID_ALL_PINS;
}
