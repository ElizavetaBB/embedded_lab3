/*
 * gpio_driver.c
 *
 *  Created on: 14 дек. 2021 г.
 *      Author: boris
 */

#include "gpio_driver.h"

#include "main.h"

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
