#ifndef INC_KEYBOARD_H_
#define INC_KEYBOARD_H_

#include <stdint.h>
#include <stdbool.h>

bool is_buffer_empty();
uint8_t get_buffer();

void kb_init(I2C_HandleTypeDef * i2c);
void kb_scan_step(I2C_HandleTypeDef * i2c);

#endif /* INC_KEYBOARD_H_ */
