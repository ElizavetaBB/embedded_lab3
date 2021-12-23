/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "sdk_uart.h"
#include "kb.h"
#include "gpio_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_LEN 4
#define MODE_COUNT 5
#define MAX_BRIGHTNESS 100
#define MAX_PHASE 10
#define USER_MODE 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//size_t write_ptr = 0;
//size_t read_ptr = 0;
char timer_print_buf[256];
char handle_print_buf[256];

uint32_t current_timestamp = 0;
uint32_t click_duration = 0;
uint32_t const counted_click_duration = 100;
bool clicked = false;

bool in_menu = false;

bool needed_change_mode = false;
uint8_t change_mode = 0;

bool needed_change_speed = false;

bool direction = true;
bool test_mode = true;
bool settings_mode = false;

bool user_mode_defined = false;
uint16_t user_chosen_pin = -1;
uint16_t pins_sequence[4] = {GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, 0};
uint16_t user_brightness = -1;
uint16_t user_speed = -1;
uint8_t user_pin_id = 0;
uint8_t user_pin_count = 0;
uint16_t user_mode[MAX_LEN] = {0};

uint8_t brightness[MODE_COUNT] = {MAX_BRIGHTNESS, 0.7*MAX_BRIGHTNESS,
		0.5*MAX_BRIGHTNESS, 0.2*MAX_BRIGHTNESS, MAX_BRIGHTNESS};

uint16_t const standart_speed = 1000;

uint16_t mode[MODE_COUNT][MAX_LEN] = {
		{GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, 0},
		{GPIO_PIN_14, GPIO_PIN_13, 0, 0},
		{GPIO_PIN_15, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_13},
		{GPIO_PIN_15, GPIO_PIN_14, 0, 0},
		{0, 0, 0, 0}
};

uint16_t speed[MODE_COUNT] = {standart_speed, 0.8*standart_speed,
		0.6*standart_speed, 1.4*standart_speed, standart_speed
};

uint8_t current_mode = 0;
uint8_t current_pin = 0;
uint16_t current_speed = standart_speed;
uint8_t phase = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void light_off() {
	htim4.Instance->CCR2 = 0;
	htim4.Instance->CCR3 = 0;
	htim4.Instance->CCR4 = 0;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}

typedef enum {
    red,
    green,
    yellow
} COLOR;

void light_LED(COLOR color, uint16_t brightness) {
	switch (color) {
		case red:
			htim4.Instance->CCR4 = brightness;
	        break;
	    case green:
	    	htim4.Instance->CCR2 = brightness;
	        break;
	    case yellow:
	      	htim4.Instance->CCR3 = brightness;
	      	break;
	}
}

void init_user_mode(){
	user_brightness = brightness[USER_MODE];
	user_speed = speed[USER_MODE];
}

void reset_current_mode(){
	current_pin = 0;
	direction = true;
	phase = 0;
	light_off();
}

void print_diodes(uint8_t mode_id){
	char current_buf[256];
	for (int i = 0; i < MAX_LEN; i++){
		if (mode[mode_id][i] == GPIO_PIN_13){
			snprintf(current_buf, sizeof(current_buf), "Led %d = green\r\n", i);
			UART_Transmit((uint8_t*) current_buf);
		} else if (mode[mode_id][i] == GPIO_PIN_14){
			snprintf(current_buf, sizeof(current_buf), "Led %d = yellow\r\n", i);
			UART_Transmit((uint8_t*) current_buf);
		} else if (mode[mode_id][i] == GPIO_PIN_15){
			snprintf(current_buf, sizeof(current_buf), "Led %d = red\r\n", i);
			UART_Transmit((uint8_t*) current_buf);
		}
	}

	if (mode[mode_id][0] == 0){
		snprintf(current_buf, sizeof(current_buf), "Mode isn't defined. Number of diodes = 0\n\r");
		UART_Transmit((uint8_t*) current_buf);
	}
}

void print_mode(uint8_t mode_id){
	char current_buf[256];
	UART_Transmit((uint8_t*) "--------------------------------------------------------\n\r");
	if (mode_id == USER_MODE){
		UART_Transmit((uint8_t*) "User mode:\n\r");
	} else {
		UART_Transmit((uint8_t*) "Current mode:\n\r");
	}
	print_diodes(mode_id);
	snprintf(current_buf, sizeof(current_buf), "Brightness = %d\r\n", brightness[mode_id]);
	UART_Transmit((uint8_t*) current_buf);
	snprintf(current_buf, sizeof(current_buf), "Speed = %d\r\n",
			(mode_id == USER_MODE) ? speed[mode_id] : current_speed);
	UART_Transmit((uint8_t*) current_buf);
	UART_Transmit((uint8_t*) "--------------------------------------------------------\n\r");
}

void print_settings(){
	UART_Transmit((uint8_t*) "--------------------------------------------------------\n\r");
	UART_Transmit((uint8_t*) "User settings:\n\r");
	print_diodes(USER_MODE);
	char current_buf[256];
	snprintf(current_buf, sizeof(current_buf), "Brightness = %d\r\n", user_brightness);
	UART_Transmit((uint8_t*) current_buf);
	snprintf(current_buf, sizeof(current_buf), "Speed = %d\r\n", user_speed);
	UART_Transmit((uint8_t*) current_buf);
	UART_Transmit((uint8_t*) "--------------------------------------------------------\n\r");
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6){
		kb_scan_step(&hi2c1);
	}
	else if (htim->Instance == TIM7){
		if (test_mode || settings_mode){
			reset_current_mode();
		} else {
			if (needed_change_mode){
				reset_current_mode();
				current_mode = change_mode;
				needed_change_mode = false;
				change_mode = 0;
				htim7.Instance->PSC = speed[current_mode];
				current_speed = speed[current_mode];
				print_mode(current_mode);
			}

			if (direction) {
				phase++;
			} else {
				phase--;
			}

			if (phase == MAX_PHASE || phase == 0){
				direction = !direction;
			}

			if (mode[current_mode][current_pin] == GPIO_PIN_13){
				htim4.Instance->CCR2 = phase * brightness[current_mode];
			} else if (mode[current_mode][current_pin] == GPIO_PIN_14){
				htim4.Instance->CCR3 = phase * brightness[current_mode];
			} else if (mode[current_mode][current_pin] == GPIO_PIN_15){
				htim4.Instance->CCR4 = phase * brightness[current_mode];
			}

			if (phase == 0){
				light_off();
				current_pin = (current_pin + 1) % MAX_LEN;
				while (mode[current_mode][current_pin] == 0){
					current_pin = (current_pin + 1) % MAX_LEN;
				}
				if (needed_change_speed){
					htim7.Instance->PSC = current_speed;
					needed_change_speed = false;
					//print_mode(current_mode, (uint8_t *) "Current mode:\n\r");
				}
			}
		}
	}
}

bool is_button_clicked(){
	bool button_state = get_button_state();

	if (button_state == false){
		clicked = false;
		current_timestamp = 0;
	} else if (current_timestamp == 0){
		current_timestamp = get_current_time();
	} else if (get_time_difference(current_timestamp) >= counted_click_duration && !clicked){
		clicked = true;
		return true;
	}

	return false;
}

void handle_menu(uint8_t command){
	switch (command) {
		case 8:
			if (user_mode_defined == false){
				user_chosen_pin = 0;
				user_mode_defined = true;
				UART_Transmit((uint8_t*) "Green led was chosen - 1\r\n");
			} else {
				user_chosen_pin = (user_chosen_pin + 1) % 3;
				if (user_chosen_pin == 0){
					UART_Transmit((uint8_t*) "Green led was chosen\r\n");
				} else if (user_chosen_pin == 1){
					UART_Transmit((uint8_t*) "Yellow led was chosen\r\n");
				} else if (user_chosen_pin == 2){
					UART_Transmit((uint8_t*) "Red led was chosen\r\n");
				} else {
					UART_Transmit((uint8_t*) "Diodes was unset\r\n");
				}
			}
			print_settings();
			break;
		case 9:
			user_brightness -= 10;
			if (user_brightness < 0) user_brightness = MAX_BRIGHTNESS;
			snprintf(handle_print_buf, sizeof(handle_print_buf), "Chosen brightness = %d\n\r", user_brightness);
			UART_Transmit((uint8_t*) handle_print_buf);
			print_settings();
			break;
		case 10:
			user_speed -= 100;
			if (user_speed < 200) user_speed = standart_speed * 2;
			snprintf(handle_print_buf, sizeof(handle_print_buf), "Chosen speed = %d\n\r", user_speed);
			UART_Transmit((uint8_t*) handle_print_buf);
			print_settings();
			break;
		case 11:
			user_pin_id = (user_pin_id + 1) % MAX_LEN;
			snprintf(handle_print_buf, sizeof(handle_print_buf), "Chosen id = %d\n\r", user_pin_id);
			UART_Transmit((uint8_t*) handle_print_buf);
			print_settings();
			break;
		case 12:
			if (user_chosen_pin != -1){
				uint8_t previous_pin_id = (user_pin_id - 1 + MAX_LEN) % MAX_LEN;
				uint8_t next_pin_id = (user_pin_id + 1) % MAX_LEN;
				if (mode[USER_MODE][previous_pin_id] == pins_sequence[user_chosen_pin] &&
						pins_sequence[user_chosen_pin] != 0) {
					UART_Transmit((uint8_t *) "You can't chose same color twice in row\n\r");
					UART_Transmit((uint8_t *) "New diode wasn't set\n\r");
					print_settings();
				} else if ((mode[USER_MODE][next_pin_id] != 0 && pins_sequence[user_chosen_pin] == 0
						&& next_pin_id != 0) || (mode[USER_MODE][previous_pin_id] == 0 &&
								pins_sequence[user_chosen_pin] != 0 && user_pin_id != 0)){
					UART_Transmit((uint8_t *) "You can't set here diode, because there are diodes after this\n\r");
					UART_Transmit((uint8_t *) "New diode wasn't set\n\r");
					print_settings();
				} else {
					mode[USER_MODE][user_pin_id] = pins_sequence[user_chosen_pin];
					user_pin_id = (user_pin_id + 1) % MAX_LEN;
					user_chosen_pin = 0;
					user_pin_count++;
					UART_Transmit((uint8_t *) "New diode was set\n\r");
					print_settings();
				}
			}
			break;
		case 6:
			if (user_pin_count < 2){
				UART_Transmit((uint8_t *) "You have to set minimum 2 diodes before saving\n\r");
				print_settings();
			} else {
			brightness[USER_MODE] = user_brightness;
			speed[USER_MODE] = user_speed;
			in_menu = false;
			if (current_mode == USER_MODE){
				needed_change_speed = true;
				current_speed = speed[USER_MODE];
				//htim6.Instance->PSC = speed[USER_MODE];
				//current_speed = speed[USER_MODE];
			}
			settings_mode = false;
			UART_Transmit((uint8_t*) "Leave menu\n\r");
			}
			break;
		default:
			UART_Transmit((uint8_t*) "Wrong key\n\r");
		    break;
	}
}

void handle_command(uint8_t command) {
	if (in_menu) handle_menu(command);
	else {
	switch (command) {
		case 1:
			change_mode = (current_mode + 1) % MODE_COUNT;
			if (change_mode == 4 && !user_mode_defined) change_mode = 0;
			needed_change_mode = true;
			break;
		case 2:
			change_mode = (current_mode - 1 + MODE_COUNT) % MODE_COUNT;
			if (change_mode == 4 && !user_mode_defined) change_mode = 3;
			needed_change_mode = true;
			break;
		case 3:
			if (current_speed > 200){
				current_speed = 0.9 * current_speed;
				needed_change_speed = true;
			}
			print_mode(current_mode);
			break;
		case 4:
			if (current_speed < 2000){
				current_speed += 0.1 * current_speed;
				needed_change_speed = true;
			}
			print_mode(current_mode);
			break;
		case 5:
			settings_mode = true;
			in_menu = true;
			UART_Transmit((uint8_t*) "Enter menu\n\r");
			print_settings();
			break;
		case 7:
			print_mode(USER_MODE);
			break;
		default:
			UART_Transmit((uint8_t*) "Wrong key\n\r");
			break;
	}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  light_off();
  init_user_mode();
  kb_init(&hi2c1);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (is_button_clicked()) {
		  if (test_mode) UART_Transmit((uint8_t*) "Return to work mode\n\r");
		  else UART_Transmit((uint8_t*) "Test mode\n\r");
		  test_mode = !test_mode;
	  }

	  if (test_mode) {
		  if (!is_buffer_empty()){
			  uint8_t key = get_buffer();
			  if (key >= 0) {
				  snprintf(handle_print_buf, sizeof(handle_print_buf), "Pressed key = %d\r\n", key);
				  UART_Transmit((uint8_t*) handle_print_buf);
			  }
		  }
	  } else {
		  if (!is_buffer_empty()){
		  	  uint8_t key = get_buffer();
			  if(key >= 0){
			  	  handle_command(key);
		  	  }
		  }
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

