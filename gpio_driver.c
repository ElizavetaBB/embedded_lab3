/*
 * gpio_driver.c
 *
 *  Created on: 14 дек. 2021 г.
 *      Author: boris
 */

#include "gpio_driver.h"

#include "main.h"


extern TIM_HandleTypeDef htim4;

const char * const led_names[] = {
    [LED_GREEN] = "GREEN",
    [LED_YELLOW] = "YELLOW",
    [LED_RED] = "RED",
};


typedef void (* led_set_function)(uint16_t);


static void led_green_set_function(uint16_t power) {
    htim4.Instance->CCR2 = power;
}

static void led_yellow_set_function(uint16_t power) {
    htim4.Instance->CCR3 = power;
}

static void led_red_set_function(uint16_t power) {
    htim4.Instance->CCR4 = power;
}

static const led_set_function led_set_functions[] = {
    [LED_GREEN] = led_green_set_function,
    [LED_YELLOW] = led_yellow_set_function,
    [LED_RED] = led_red_set_function,
};

void led_set_power(enum led led, uint8_t power) {
    if (power > 100) {
        power = 100;
    }

    led_set_functions[led]((uint16_t) power * 10);
}

bool get_button_state() {
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET) return 0;
    else return 1;
}

uint32_t get_current_time() {
    return HAL_GetTick();
}

uint32_t get_time_difference(uint32_t start) {
    return HAL_GetTick() - start;
}
