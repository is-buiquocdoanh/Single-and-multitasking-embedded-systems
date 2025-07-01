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
#include "cmsis_os.h"

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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for task_sensor */
osThreadId_t task_sensorHandle;
const osThreadAttr_t task_sensor_attributes = {
  .name = "task_sensor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task_lcd */
osThreadId_t task_lcdHandle;
const osThreadAttr_t task_lcd_attributes = {
  .name = "task_lcd",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task_uart */
osThreadId_t task_uartHandle;
const osThreadAttr_t task_uart_attributes = {
  .name = "task_uart",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RXTask */
osThreadId_t RXTaskHandle;
const osThreadAttr_t RXTask_attributes = {
  .name = "RXTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};
/* USER CODE BEGIN PV */
AHT10_HandleTypeDef aht10;
I2C_LCD_HandleTypeDef lcd;

char debug[32];
char buffer[32];
float temperature, humidity;
uint32_t temp_raw = 0;
uint32_t hum_raw = 0;

//Biến dùng chung cho nhận UART
uint8_t rx_byte;
char command_buffer[64];
volatile uint8_t command_ready = 0;

//Biến toàn cục lưu trạng thái điều khiển
typedef enum { OFF = 0, ON = 1 } MotorPower;
typedef enum { CW = 0, CCW = 1 } MotorDirection;
typedef enum { DISPLAY_ALL, DISPLAY_TEMP, DISPLAY_HUMID } DisplayMode;

volatile MotorPower motor_power = OFF;
volatile MotorDirection motor_dir = CW;
volatile uint8_t motor_speed = 0;
volatile DisplayMode display_mode = DISPLAY_ALL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartSensorTask(void *argument);
void StartLCDTask(void *argument);
void StartUARTTxTask(void *argument);
void StartUARTRxTask(void *argument);
void StartMotorTask(void *argument);

/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

  	 //Khởi tạo uart nhận 1 byte
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

	//Khởi tạo timmer2 CH3
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

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

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task_sensor */
  task_sensorHandle = osThreadNew(StartSensorTask, NULL, &task_sensor_attributes);

  /* creation of task_lcd */
  task_lcdHandle = osThreadNew(StartLCDTask, NULL, &task_lcd_attributes);

  /* creation of task_uart */
  task_uartHandle = osThreadNew(StartUARTTxTask, NULL, &task_uart_attributes);

  /* creation of RXTask */
  RXTaskHandle = osThreadNew(StartUARTRxTask, NULL, &RXTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
			command_buffer[idx] = '\0'; // kết thúc chuỗi
			idx = 0;
			command_ready = 1;
		}

		// Bắt đầu nhận byte tiếp theo
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the task_sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	/* Infinite loop */
	for (;;) {
		if (AHT10_ReadRaw(&aht10, &hum_raw, &temp_raw)) {
			float temp = AHT10_ReadTemperature(temp_raw);
			float hum = AHT10_ReadHumidity(hum_raw);

			temperature = temp;
			humidity = hum;
		}
		osDelay(2000);
		taskYIELD();
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLCDTask */
/**
  * @brief  Function implementing the task_lcd thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLCDTask */
void StartLCDTask(void *argument)
{
  /* USER CODE BEGIN StartLCDTask */
	char lcd_buf[32];
	/* Infinite loop */
	for (;;) {
		float temp, hum;

		temp = temperature;
		hum = humidity;

		int temp_int = (int) temp;
		int temp_frac = (int) ((temp - temp_int) * 10);
		int hum_int = (int) hum;
		int hum_frac = (int) ((hum - hum_int) * 10);

		sprintf(lcd_buf, "Temp: %d.%d C", temp_int, temp_frac);
		lcd_gotoxy(&lcd, 0, 0);
		lcd_puts(&lcd, lcd_buf);

		sprintf(lcd_buf, "Hum:  %d.%d %%", hum_int, hum_frac);
		lcd_gotoxy(&lcd, 0, 1);
		lcd_puts(&lcd, lcd_buf);

		osDelay(2000);
		taskYIELD();
	}
  /* USER CODE END StartLCDTask */
}

/* USER CODE BEGIN Header_StartUARTTxTask */
/**
* @brief Function implementing the task_uart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTTxTask */
void StartUARTTxTask(void *argument)
{
  /* USER CODE BEGIN StartUARTTxTask */
	char uart_buf[64];
	/* Infinite loop */
	for (;;) {
		float temp = temperature;
		float hum = humidity;

		int temp_int = (int) temp;
		int temp_frac = (int) ((temp - temp_int) * 10);
		int hum_int = (int) hum;
		int hum_frac = (int) ((hum - hum_int) * 10);

		sprintf(uart_buf, "temperature: %d.%d C, humidity: %d.%d %%\r\n",
				temp_int, temp_frac, hum_int, hum_frac);

		HAL_UART_Transmit(&huart1, (uint8_t*) uart_buf, strlen(uart_buf), 100);

		osDelay(5000);  // Gửi mỗi 1 giây
		taskYIELD();
	}
  /* USER CODE END StartUARTTxTask */
}

/* USER CODE BEGIN Header_StartUARTRxTask */
/**
* @brief Function implementing the RXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTRxTask */
void StartUARTRxTask(void *argument)
{
  /* USER CODE BEGIN StartUARTRxTask */
	/* Infinite loop */
	for (;;) {
		if (command_ready) {
			command_ready = 0;

			osMutexAcquire(uartMutexHandle, osWaitForever);

			// Hiển thị lệnh nhận được
			HAL_UART_Transmit(&huart1, (uint8_t*) "Cmd: ", 5, HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t*) command_buffer,
					strlen(command_buffer), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);

			// Xử lý lệnh
			if (strncmp(command_buffer, "!motor ", 7) == 0) {
				if (strstr(command_buffer, "on"))
					motor_power = ON;
				else if (strstr(command_buffer, "off"))
					motor_power = OFF;

			} else if (strncmp(command_buffer, "!rotate ", 8) == 0) {
				if (strstr(command_buffer, "ccw"))
					motor_dir = CCW;
				else if (strstr(command_buffer, "cw"))
					motor_dir = CW;

			} else if (strncmp(command_buffer, "!speed ", 7) == 0) {
				int val = atoi(&command_buffer[7]);
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

			// Phản hồi trạng thái hiện tại
			char debug_msg[64];
			sprintf(debug_msg, "Motor: %s, Dir: %s, Speed: %d, Display: %s\r\n",
					(motor_power == ON) ? "ON" : "OFF",
					(motor_dir == CW) ? "CW" : "CCW", motor_speed,
					(display_mode == DISPLAY_ALL) ? "ALL" :
					(display_mode == DISPLAY_TEMP) ? "TEMP" : "HUMID");

			HAL_UART_Transmit(&huart1, (uint8_t*) debug_msg, strlen(debug_msg),
			HAL_MAX_DELAY);

			// Xóa buffer cũ
			memset(command_buffer, 0, sizeof(command_buffer));

			osMutexRelease(uartMutexHandle);
		}
//		osDelay(50);
		taskYIELD();
	}
  /* USER CODE END StartUARTRxTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
	/* Infinite loop */
	for (;;) {
		if (motor_power == ON) {
			// Điều chỉnh chiều quay
			if (motor_dir == CW) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // IN1
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // IN2
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			}

			// Điều chỉnh tốc độ
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, motor_speed);
		} else {
			// Tắt động cơ
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		}

//		osDelay(100);  // Cập nhật mỗi 100ms
		taskYIELD();
	}
  /* USER CODE END StartMotorTask */
}

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
