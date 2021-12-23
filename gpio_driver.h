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

enum led {
    LED_GREEN = 0,
    LED_YELLOW = 1,
    LED_RED = 2,
};

struct led_mode {
    enum led led;
    uint8_t power;
};

extern const char * const led_names[];

void led_set_power(enum led led, uint8_t power);

static inline void led_mode_enable(struct led_mode mode) {
    led_set_power(mode.led, mode.power);
}

static inline void led_mode_disable(struct led_mode mode) {
    led_set_power(mode.led, 0);
}

bool get_button_state();

uint32_t get_current_time();

uint32_t get_time_difference(uint32_t start);

#endif /* INC_GPIO_DRIVER_H_ */
