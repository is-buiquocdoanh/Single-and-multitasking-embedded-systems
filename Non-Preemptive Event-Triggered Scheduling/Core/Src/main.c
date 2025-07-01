/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "i2c_lcd.h"
#include "aht10.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
AHT10_HandleTypeDef aht10;
I2C_LCD_HandleTypeDef lcd;

char command_buffer[64];
volatile uint8_t command_ready = 0;
uint8_t rx_byte;  // byte nhận qua UART

// Motor control
//Biến toàn cục lưu trạng thái điều khiển
typedef enum { OFF = 0, ON = 1 } MotorPower;
typedef enum { CW = 0, CCW = 1 } MotorDirection;
typedef enum { DISPLAY_ALL, DISPLAY_TEMP, DISPLAY_HUMID } DisplayMode;

volatile MotorPower motor_power = OFF;
volatile MotorDirection motor_dir = CW;
volatile uint8_t motor_speed = 0;
volatile DisplayMode display_mode = DISPLAY_ALL;

float temperature = 0.0f;
float humidity = 0.0f;

// biến cho event triggered scheduling
volatile uint8_t uart_event = 0;
volatile uint8_t sensor_event = 0;
volatile uint8_t lcd_event = 0;
volatile uint8_t uart_tx_event = 0;
volatile uint8_t motor_event = 0;
volatile uint32_t ms_ticks = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
//Xử lí lệnh UART
void parse_command(char *cmd) {
	if (strncmp(cmd, "!motor ", 7) == 0) {
		if (strstr(cmd, "on"))
			motor_power = ON;
		else if (strstr(cmd, "off"))
			motor_power = OFF;
	} else if (strncmp(cmd, "!rotate ", 8) == 0) {
		if (strstr(cmd, "ccw"))
			motor_dir = CCW;
		else if (strstr(cmd, "cw"))
			motor_dir = CW;
	} else if (strncmp(cmd, "!speed ", 7) == 0) {
		int val = atoi(&cmd[7]);
		if (val >= 0 && val <= 100)
			motor_speed = val;
	} else if (strncmp(command_buffer, "!display ", 9) == 0) {
		if (strstr(command_buffer, "all"))
			display_mode = DISPLAY_ALL;
		else if (strstr(command_buffer, "temp"))
			display_mode = DISPLAY_TEMP;
		else if (strstr(command_buffer, "humid"))
			display_mode = DISPLAY_HUMID;
	}

	// Phản hồi trạng thái
	char msg[64];
	sprintf(msg, "Motor: %s, Dir: %s, Speed: %d, Display: %s\r\n",
			(motor_power == ON) ? "ON" : "OFF",
			(motor_dir == CW) ? "CW" : "CCW", motor_speed,
			(display_mode == DISPLAY_ALL) ? "ALL" :
			(display_mode == DISPLAY_TEMP) ? "TEMP" : "HUMID");

	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), 100);

	memset(command_buffer, 0, sizeof(command_buffer));
}


//Task 1: đọc cảm biến AHT10
void task_sensor(void) {
    uint32_t hum_raw = 0, temp_raw = 0;
    if (AHT10_ReadRaw(&aht10, &hum_raw, &temp_raw)) {
        temperature = AHT10_ReadTemperature(temp_raw);
        humidity = AHT10_ReadHumidity(hum_raw);
    }
}

//Task 2: hiển thị LCD
void task_lcd(void) {
	char lcd_buf[32];
	int ti = (int)temperature;
	int tf = (int)((temperature - ti) * 10);
	int hi = (int)humidity;
	int hf = (int)((humidity - hi) * 10);

	switch (display_mode) {
	case DISPLAY_ALL:
		sprintf(lcd_buf, "Temp: %d.%d C", ti, tf);
		lcd_gotoxy(&lcd, 0, 0);
		lcd_puts(&lcd, lcd_buf);

		sprintf(lcd_buf, "Hum:  %d.%d %%", hi, hf);
		lcd_gotoxy(&lcd, 0, 1);
		lcd_puts(&lcd, lcd_buf);
		break;

	case DISPLAY_TEMP:
		sprintf(lcd_buf, "Temp: %d.%d C", ti, tf);
		lcd_gotoxy(&lcd, 0, 0);
		lcd_puts(&lcd, lcd_buf);

		lcd_gotoxy(&lcd, 0, 1);
		lcd_puts(&lcd, "                "); // Clear dòng 2
		break;

	case DISPLAY_HUMID:
		sprintf(lcd_buf, "Hum:  %d.%d %%", hi, hf);
		lcd_gotoxy(&lcd, 0, 0);
		lcd_puts(&lcd, lcd_buf);

		lcd_gotoxy(&lcd, 0, 1);
		lcd_puts(&lcd, "                "); // Clear dòng 2
		break;
	}
}

//Task 3: gửi UART
void task_uart_tx(void) {
    char buf[64];
    sprintf(buf, "temperature: %.1f C, humidity: %.1f %%\r\n", temperature, humidity);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
}

//Task 4: điều khiển động cơ
void task_motor(void) {
    if (motor_power == ON) {
        // Điều chỉnh chiều quay
        if (motor_dir == CW) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // IN1
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // IN2
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        }

        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
        // Tốc độ: từ 0 đến 100
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, motor_speed);
    } else {
        // Dừng động cơ
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    }
}

// Hàm systick
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        ms_ticks ++;

        if (ms_ticks % 2000 == 0) sensor_event = 1;
        if (ms_ticks % 500 == 0)  lcd_event = 1;
        if (ms_ticks % 5000 == 0) uart_tx_event = 1;
        if (ms_ticks % 100 == 0)  motor_event = 1;

        if (ms_ticks >= 10000) ms_ticks = 0;  // Reset sau mỗi 10s để tránh tràn số
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //Khởi tạo uart nhận 1 byte
  	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  	//Khởi tạo timmer2 CH3
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  	HAL_TIM_Base_Start_IT(&htim1);  // Bắt đầu timer với ngắt

  	//Init LCD
  	lcd.hi2c = &hi2c1;
  	lcd.address = 0x4E;
  	lcd_init(&lcd);
  	lcd_clear(&lcd);

  	//Init AHT10
  	aht10.hi2c = &hi2c2;
  	aht10.address = 0x38 << 1; //AHT10 default address << 1 = 0x38 << 1 = 0x70
  	AHT10_Init(&aht10);
  	HAL_Delay(50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // IN1
//	              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//	              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 100);

		if (uart_event) {
			parse_command(command_buffer);
			uart_event = 0;
		}
		if (sensor_event) {
			task_sensor();
			sensor_event = 0;
		}
		if (lcd_event) {
			task_lcd();
			lcd_event = 0;
		}
		if (uart_tx_event) {
			task_uart_tx();
			uart_tx_event = 0;
		}
		if (motor_event) {
			task_motor();
			motor_event = 0;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static uint8_t idx = 0;
	if (huart->Instance == USART1) {
		if (rx_byte != '\n' && idx < sizeof(command_buffer) - 1) {
			command_buffer[idx++] = rx_byte;
		} else {
			command_buffer[idx] = '\0';
			idx = 0;
			uart_event = 1;
		}
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	}
}

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
