/*
 * led.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Juita Sari
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stm32f4xx_hal.h"

// Init
void LED_Init(void);

// Control dasar
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

// Non-blocking blink handler
void LED_Blink(uint32_t interval_ms);

#endif /* INC_LED_H_ */
