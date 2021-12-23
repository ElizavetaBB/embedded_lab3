#include "main.h"
#include <stdbool.h>
#include "pca9538.h"
#include "kb.h"
#include "sdk_uart.h"
#include "usart.h"
#include "utils.h"
#include <string.h>
#include "gpio_driver.h"

#define KBRD_RD_ADDR 0xE3
#define KBRD_WR_ADDR 0xE2
#define FIFO_BUFFER_SIZE 32

#define KB_I2C_READ_ADDRESS (0xE3)
#define KB_I2C_WRITE_ADDRESS (0xE2)
#define KB_INPUT_REG (0x0)
#define KB_OUTPUT_REG (0x1)
#define KB_CONFIG_REG (0x3)

static uint8_t buffer[FIFO_BUFFER_SIZE] = { 0 };
static size_t read_ptr = 0;
static size_t write_ptr = 0;

uint32_t const counted_duration = 50;

void append_buffer(uint8_t event) {
	const uint32_t priMask = __get_PRIMASK();
	__disable_irq();
    buffer[write_ptr] = event;
    write_ptr = (write_ptr + 1) % FIFO_BUFFER_SIZE;
    __set_PRIMASK(priMask);
}

bool is_buffer_empty() {
    const uint32_t priMask = __get_PRIMASK();
    __disable_irq();

    bool ret = read_ptr == write_ptr;

    __set_PRIMASK(priMask);
    return ret;
}

uint8_t get_buffer() {
    const uint32_t priMask = __get_PRIMASK();
    __disable_irq();

    const uint8_t evt = buffer[read_ptr];
    read_ptr = (read_ptr + 1) % FIFO_BUFFER_SIZE;

    __set_PRIMASK(priMask);
    return evt;
}

void kb_init(I2C_HandleTypeDef * i2c) {
    static uint8_t output = 0x0;

    HAL_I2C_Mem_Write(i2c, KB_I2C_WRITE_ADDRESS, KB_OUTPUT_REG, 1, &output, 1, 100);
}

void kb_write_config(I2C_HandleTypeDef * i2c, uint8_t data) {
    static uint8_t buf;
    buf = data;
    HAL_I2C_Mem_Write_IT(i2c, KB_I2C_WRITE_ADDRESS, KB_CONFIG_REG, 1, &buf, 1);
}

void kb_select_row(I2C_HandleTypeDef * i2c, uint8_t row) {
    kb_write_config(i2c, ~((uint8_t) (1 << row)));
}

void kb_read_input(I2C_HandleTypeDef * i2c, uint8_t * data) {
    HAL_I2C_Mem_Read_IT(i2c, KB_I2C_READ_ADDRESS, KB_INPUT_REG, 1, data, 1);
}

void kb_scan_step(I2C_HandleTypeDef * i2c) {
    static uint8_t reg_buffer = ~0;

    static uint8_t row = 0;
    static bool read = false;
    static bool input_keys[12] = { 0 };
    static bool output_keys[12] = { 0 };
    static uint32_t key_time[12] = { 0 };

    if (HAL_I2C_GetState(i2c) != HAL_I2C_STATE_READY) {
        return;
    }

    if (!read) {
        // считываем данные
        for (uint8_t i = 0, mask = 0x10; i < 3; ++i, mask <<= 1) {
            if ((reg_buffer & mask) == 0) {
                input_keys[row * 3 + i] = true;
            }
        }

        row = (row + 1) % 4;

        // если все строки прочитаны, считаем результат
        if (row == 0) {
            uint8_t count = 0;

            // считаем нажатые кнопки
            for (int i = 0; i < 12; ++i) {
                if (input_keys[i]) {
                    ++count;
                    if (count > 1) break;
                }
            }

            // если нажато более 1 кнопки, игнорируем
            if (count > 1) {
                for (int i = 0; i < 12; ++i) {
                    input_keys[i] = false;
                }
            }

            for (int i = 0; i < 12; ++i) {

                if (output_keys[i] == input_keys[i]) {
                    key_time[i] = 0;
                } else if (key_time[i] == 0) {
                    key_time[i] = get_current_time();
                } else if (get_time_difference(key_time[i]) >= counted_duration && output_keys[i] == 0) {
                    output_keys[i] = 1;
                    append_buffer(i+1);
                }
            }

            memset(input_keys, 0, sizeof(input_keys));
        }

        // записываем
        kb_select_row(i2c, row);
    } else {
        // читаем
        kb_read_input(i2c, &reg_buffer);
    }

    read = !read;
}
