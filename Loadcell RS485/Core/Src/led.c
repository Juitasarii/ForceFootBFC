/*
 * led.c
 *
 *  Created on: Apr 9, 2026
 *      Author: Juita Sari
 */

#include "led.h"

// Konfigurasi LED
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13

// Variable internal (private)
static uint32_t last_time = 0;
static uint8_t led_state = 0;

void LED_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // Default OFF (active LOW)
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void LED_On(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    led_state = 1;
}

void LED_Off(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    led_state = 0;
}

void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    led_state = !led_state;
}

// Fungsi NON-BLOCKING
void LED_Blink(uint32_t interval_ms)
{
    uint32_t now = HAL_GetTick();

    if (now - last_time >= interval_ms)
    {
        last_time = now;
        LED_Toggle();
    }
}
