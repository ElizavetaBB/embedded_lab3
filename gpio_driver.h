/*
 * gpio_driver.h
 *
 *  Created on: 14 дек. 2021 г.
 *      Author: boris
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

bool get_button_state();

uint32_t get_current_time();

uint32_t get_time_difference(uint32_t start);

#endif /* INC_GPIO_DRIVER_H_ */
