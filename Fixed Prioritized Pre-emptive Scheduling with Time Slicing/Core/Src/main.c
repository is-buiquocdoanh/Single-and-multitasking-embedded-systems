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
#include "i2c_lcd.h"
#include "aht10.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
	#define Putchar_Prototype int __io_putchar(int ch)
#else
	#define Putchar_Prototype int fputc(int ch, FILE *f)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
  .name = "Task3",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task4 */
osThreadId_t Task4Handle;
const osThreadAttr_t Task4_attributes = {
  .name = "Task4",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task5 */
osThreadId_t Task5Handle;
const osThreadAttr_t Task5_attributes = {
  .name = "Task5",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskISR */
osThreadId_t TaskISRHandle;
const osThreadAttr_t TaskISR_attributes = {
  .name = "TaskISR",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for CommandQueue */
osMessageQueueId_t CommandQueueHandle;
const osMessageQueueAttr_t CommandQueue_attributes = {
  .name = "CommandQueue"
};
/* Definitions for SensorDataMutex */
osMutexId_t SensorDataMutexHandle;
const osMutexAttr_t SensorDataMutex_attributes = {
  .name = "SensorDataMutex"
};
/* Definitions for DisplayModeMutex */
osMutexId_t DisplayModeMutexHandle;
const osMutexAttr_t DisplayModeMutex_attributes = {
  .name = "DisplayModeMutex"
};
/* Definitions for UART_AccessMutex */
osMutexId_t UART_AccessMutexHandle;
const osMutexAttr_t UART_AccessMutex_attributes = {
  .name = "UART_AccessMutex"
};
/* Definitions for SpeedValueMutex */
osMutexId_t SpeedValueMutexHandle;
const osMutexAttr_t SpeedValueMutex_attributes = {
  .name = "SpeedValueMutex"
};
/* Definitions for DirectionMutex */
osMutexId_t DirectionMutexHandle;
const osMutexAttr_t DirectionMutex_attributes = {
  .name = "DirectionMutex"
};
/* Definitions for MotorModeMutex */
osMutexId_t MotorModeMutexHandle;
const osMutexAttr_t MotorModeMutex_attributes = {
  .name = "MotorModeMutex"
};
/* Definitions for ISR_RxUART */
osEventFlagsId_t ISR_RxUARTHandle;
const osEventFlagsAttr_t ISR_RxUART_attributes = {
  .name = "ISR_RxUART"
};
/* USER CODE BEGIN PV */
AHT10_HandleTypeDef aht10;
I2C_LCD_HandleTypeDef lcd;

char RxByte;
uint8_t RxIndex = 0;
char RxBuffer[RX_BUFFER_SIZE];

uint32_t periodTask2 = 100;
uint32_t periodTask3 = 2000;
uint32_t periodTask4 = 500;
uint32_t periodTask5 = 2000;

CommandParameter_t Mode = OFF;
uint8_t SpeedValue = 0;
CommandParameter_t Direction = CW;
SensorData_t SensorData;
CommandParameter_t currentModeLCD = ALL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
void UART_CommandHandlerTask(void *argument);
void MotorControlTask(void *argument);
void SensorReadTask(void *argument);
void LCD_DisplayTask(void *argument);
void UART_ReportSendTask(void *argument);
void UART_ParseCommandTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Putchar_Prototype
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
	return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	if(huart->Instance == USART1)
	{
		if(RxByte != 13) {
			if(RxIndex < RX_BUFFER_SIZE - 1) {
				RxBuffer[RxIndex++] = RxByte;
			} else {
				memset(&RxBuffer, 0, RX_BUFFER_SIZE);
				RxIndex = 0;
			}

		} else {
			RxBuffer[RxIndex] = '\0';
			RxIndex = 0;

			osEventFlagsSet(ISR_RxUARTHandle, FlagISR);
		}
		// Gọi lại HAL_UART_Receive_IT để tiếp tục nhận byte tiếp theo
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxByte, 1);
	}
}

void MOTOR_SetSpeed(uint8_t speed)
{
	if(speed > 100) {
		speed = 100;
	}

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed);
}

void MOTOR_SetDirection(CommandParameter_t rotate)
{
	if(rotate == CW) {
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	}
	else if(rotate == CCW) {
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
	}
}

void MOTOR_Stop(void)
{
	MOTOR_SetSpeed(0);
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Khởi tạo AHT10
  aht10.hi2c = &hi2c2;
  aht10.address = 0x38<<1;
  AHT10_Init(&aht10);
  HAL_Delay(50);

  // Khởi tạo LCD
  lcd.hi2c = &hi2c1;
  lcd.address = 0x4E;
  lcd_init(&lcd);
  lcd_clear(&lcd);
  // Khởi tạo TIM PWN
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  // Bắt đầu nhận dữ liệu ngắt UART
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxByte, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of SensorDataMutex */
  SensorDataMutexHandle = osMutexNew(&SensorDataMutex_attributes);

  /* creation of DisplayModeMutex */
  DisplayModeMutexHandle = osMutexNew(&DisplayModeMutex_attributes);

  /* creation of UART_AccessMutex */
  UART_AccessMutexHandle = osMutexNew(&UART_AccessMutex_attributes);

  /* creation of SpeedValueMutex */
  SpeedValueMutexHandle = osMutexNew(&SpeedValueMutex_attributes);

  /* creation of DirectionMutex */
  DirectionMutexHandle = osMutexNew(&DirectionMutex_attributes);

  /* creation of MotorModeMutex */
  MotorModeMutexHandle = osMutexNew(&MotorModeMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CommandQueue */
  CommandQueueHandle = osMessageQueueNew (5, sizeof(Command_t), &CommandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1 */
  Task1Handle = osThreadNew(UART_CommandHandlerTask, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(MotorControlTask, NULL, &Task2_attributes);

  /* creation of Task3 */
  Task3Handle = osThreadNew(SensorReadTask, NULL, &Task3_attributes);

  /* creation of Task4 */
  Task4Handle = osThreadNew(LCD_DisplayTask, NULL, &Task4_attributes);

  /* creation of Task5 */
  Task5Handle = osThreadNew(UART_ReportSendTask, NULL, &Task5_attributes);

  /* creation of TaskISR */
  TaskISRHandle = osThreadNew(UART_ParseCommandTask, NULL, &TaskISR_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of ISR_RxUART */
  ISR_RxUARTHandle = osEventFlagsNew(&ISR_RxUART_attributes);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_GPIO_WritePin(GPIOA, IN1_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_UART_CommandHandlerTask */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_UART_CommandHandlerTask */
void UART_CommandHandlerTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  Command_t command;
  osStatus_t status;
  /* Infinite loop */
  for(;;)
  {
	  // Chờ nhận một lệnh từ Queue
	  // osMessageQueueGet là hàm CMSIS_V2 tương đương với xQueueReceive
	  // osWaitForever: Task sẽ block vô thời hạn cho đến khi có lệnh trong Queue
	  status = osMessageQueueGet(CommandQueueHandle, &command, NULL, osWaitForever);
	  if(status == osOK)
	  {
		  switch(command.cmd)
		  {
		  case MOTOR:
			  switch(command.param) {
			  case ON:
				  status = osMutexAcquire(MotorModeMutexHandle, osWaitForever);
				  if(status == osOK) {
					  Mode = ON;
					  osMutexRelease(MotorModeMutexHandle);
				  }
				  break;
			  case OFF:
				  status = osMutexAcquire(MotorModeMutexHandle, osWaitForever);
				  if(status == osOK) {
					  Mode = OFF;
					  osMutexRelease(MotorModeMutexHandle);
				  }
				  break;
			  default:
				  break;
			  }
			  break;
		  case ROTATE:
			  switch(command.param) {
			  case CW:
				  status = osMutexAcquire(DirectionMutexHandle, osWaitForever);
				  if(status == osOK) {
					  Direction = CW;
					  osMutexRelease(DirectionMutexHandle);
				  }
				  break;
			  case CCW:
				  status = osMutexAcquire(DirectionMutexHandle, osWaitForever);
				  if(status == osOK) {
					  Direction = CCW;
					  osMutexRelease(DirectionMutexHandle);
				  }
				  break;
			  default:
				  break;
			  }
			  break;
		  case SPEED:
			  status = osMutexAcquire(SpeedValueMutexHandle, osWaitForever);
			  if(status == osOK) {
				  SpeedValue = command.param;
				  osMutexRelease(SpeedValueMutexHandle);
			  }
			  break;
		  case DISPLAY:
			  switch(command.param) {
			  case ALL:
				  status = osMutexAcquire(DisplayModeMutexHandle, osWaitForever);
				  if(status == osOK) {
					  currentModeLCD = ALL;
					  osMutexRelease(DisplayModeMutexHandle);
				  }
				  break;
			  case TEMP:
				  status = osMutexAcquire(DisplayModeMutexHandle, osWaitForever);
				  if(status == osOK) {
					  currentModeLCD = TEMP;
					  osMutexRelease(DisplayModeMutexHandle);
				  }
				  break;
			  case HUMID:
				  status = osMutexAcquire(DisplayModeMutexHandle, osWaitForever);
				  if(status == osOK) {
					  currentModeLCD = HUMID;
					  osMutexRelease(DisplayModeMutexHandle);
				  }
				  break;
			  default:
				  break;
			  }
			  break;
		  case PTASK2:
			  periodTask2 = command.param;
			  break;
		  case PTASK3:
			  periodTask3 = command.param;
			  break;
		  case PTASK4:
			  periodTask4 = command.param;
			  break;
		  case PTASK5:
			  periodTask5 = command.param;
			  break;
		  case CMD_INVALID:
			  break;
		  default:
			  break;
		  }
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MotorControlTask */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorControlTask */
void MotorControlTask(void *argument)
{
  /* USER CODE BEGIN MotorControlTask */
  osStatus_t status;
  CommandParameter_t localMode;
  uint8_t localSpeed;
  CommandParameter_t localDirection;

  TickType_t xLastWakeTime = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  status = osMutexAcquire(MotorModeMutexHandle, osWaitForever);
	  if(status == osOK) {
		  localMode = Mode;
		  osMutexRelease(MotorModeMutexHandle);
	  }
	  status = osMutexAcquire(DirectionMutexHandle, osWaitForever);
	  if(status == osOK) {
		  localDirection = Direction;
		  osMutexRelease(DirectionMutexHandle);
	  }
	  status = osMutexAcquire(SpeedValueMutexHandle, osWaitForever);
	  if(status == osOK) {
		  localSpeed = SpeedValue;
		  osMutexRelease(SpeedValueMutexHandle);
	  }

	  if(localMode == ON) {
		  MOTOR_SetDirection(localDirection);
		  MOTOR_SetSpeed(localSpeed);
	  }
	  else if(localMode == OFF) {
		  MOTOR_Stop();
	  }

	  osDelayUntil(xLastWakeTime + pdMS_TO_TICKS(periodTask2));
	  xLastWakeTime = osKernelGetTickCount();
  }
  /* USER CODE END MotorControlTask */
}

/* USER CODE BEGIN Header_SensorReadTask */
/**
* @brief Function implementing the Task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorReadTask */
void SensorReadTask(void *argument)
{
  /* USER CODE BEGIN SensorReadTask */
  uint32_t temperature_raw, humidity_raw;
  float temperature, humidity;
  osStatus_t status;

  TickType_t xLastWakeTime = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  if(AHT10_ReadRaw(&aht10, &humidity_raw, &temperature_raw))
	  {
          // Chuyển đổi dữ liệu thô sang giá trị thực
		  temperature = AHT10_ReadTemperature(temperature_raw);
		  humidity = AHT10_ReadHumidity(humidity_raw);

		  status = osMutexAcquire(SensorDataMutexHandle, osWaitForever);
		  if(status == osOK) {
			  SensorData.temp = temperature;
			  SensorData.humid = humidity;
			  osMutexRelease(SensorDataMutexHandle);
		  } else {
			  // Debug
		  }
	  } else {
		  // Debug
	  }

	  osDelayUntil(xLastWakeTime + pdMS_TO_TICKS(periodTask3));
	  xLastWakeTime = osKernelGetTickCount();
  }
  /* USER CODE END SensorReadTask */
}

/* USER CODE BEGIN Header_LCD_DisplayTask */
/**
* @brief Function implementing the Task4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LCD_DisplayTask */
void LCD_DisplayTask(void *argument)
{
  /* USER CODE BEGIN LCD_DisplayTask */
  SensorData_t localData;
  CommandParameter_t localMode;
  char line1[17];
  char line2[17];
  osStatus_t status;

  TickType_t xLastWakeTime = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  status = osMutexAcquire(DisplayModeMutexHandle, osWaitForever);
	  if(status == osOK) {
		  localMode = currentModeLCD;
		  osMutexRelease(DisplayModeMutexHandle);
	  } else {
		  // Debug
	  }

	  status = osMutexAcquire(SensorDataMutexHandle, osWaitForever);
	  if(status == osOK) {
		  localData = SensorData;
		  osMutexRelease(SensorDataMutexHandle);
	  } else {
		  // Debug
	  }

	  switch(localMode)
	  {
	  case ALL:
		  lcd_gotoxy(&lcd, 0, 0);
		  snprintf(line1, sizeof(line1), "Temp: %.2f C   ", localData.temp);
		  lcd_puts(&lcd, line1);

		  lcd_gotoxy(&lcd, 0, 1);
		  snprintf(line2, sizeof(line2), "Humid: %.2f %%  ", localData.humid);
		  lcd_puts(&lcd, line2);

		  break;
	  case TEMP:
		  lcd_gotoxy(&lcd, 0, 0);
		  snprintf(line1, sizeof(line1), "Temp: %.2f C   ", localData.temp);
		  lcd_puts(&lcd, line1);

		  lcd_gotoxy(&lcd, 0, 1);
		  lcd_puts(&lcd, "                ");

		  break;
	  case HUMID:
		  lcd_gotoxy(&lcd, 0, 0);
		  snprintf(line1, sizeof(line1), "Humid: %.2f %%  ", localData.humid);
		  lcd_puts(&lcd, line1);

		  lcd_gotoxy(&lcd, 0, 1);
		  lcd_puts(&lcd, "                ");

		  break;
	  default:
		  break;
	  }

	  osDelayUntil(xLastWakeTime + pdMS_TO_TICKS(periodTask4));
	  xLastWakeTime = osKernelGetTickCount();
  }
  /* USER CODE END LCD_DisplayTask */
}

/* USER CODE BEGIN Header_UART_ReportSendTask */
/**
* @brief Function implementing the Task5 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_ReportSendTask */
void UART_ReportSendTask(void *argument)
{
  /* USER CODE BEGIN UART_ReportSendTask */
  SensorData_t localData;
  osStatus_t status;

  TickType_t xLastWakeTime = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  status = osMutexAcquire(SensorDataMutexHandle, osWaitForever);
	  if(status == osOK) {
		  localData = SensorData;
		  osMutexRelease(SensorDataMutexHandle);
	  } else {
		  // Debug
	  }

	  status = osMutexAcquire(UART_AccessMutexHandle, osWaitForever);
	  if(status == osOK) {
		  printf("Sensor Data: Temp = %.2f C, Humid = %.2f %%\r\n", localData.temp, localData.humid);
		  osMutexRelease(UART_AccessMutexHandle);
	  }

	  osDelayUntil(xLastWakeTime + pdMS_TO_TICKS(periodTask5));
	  xLastWakeTime = osKernelGetTickCount();
  }
  /* USER CODE END UART_ReportSendTask */
}

/* USER CODE BEGIN Header_UART_ParseCommandTask */
/**
* @brief Function implementing the TaskISR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_ParseCommandTask */
void UART_ParseCommandTask(void *argument)
{
  /* USER CODE BEGIN UART_ParseCommandTask */
  osStatus_t status;
  static char OriginCommand[RX_BUFFER_SIZE];
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(ISR_RxUARTHandle, FlagISR, osFlagsWaitAny | osFlagsNoClear, osWaitForever);
	  osEventFlagsClear(ISR_RxUARTHandle, FlagISR);

	  strncpy(OriginCommand, RxBuffer, RX_BUFFER_SIZE - 1);
	  OriginCommand[RX_BUFFER_SIZE - 1] = '\0';

	  Command_t command;
	  command.cmd = CMD_INVALID;
	  command.param = PARAM_INVALID;

	  char *token;
	  char *save_ptr;

	  char *start_ptr = RxBuffer; // Con trỏ tới ký tự đầu đầu của chuỗi lệnh
	  while(*start_ptr == ' ' || *start_ptr == '\t') {
		  start_ptr++; // Bỏ qua ký tự này và kiểm tra tiếp
	  }
      char *end_ptr = start_ptr + strlen(start_ptr) - 1; // Con trỏ đến ký tự cuối cùng của chuỗi lệnh
      while (end_ptr > start_ptr && (*end_ptr == '\n' || *end_ptr == '\r' || *end_ptr == ' ' || *end_ptr == '\t')) {
          *end_ptr = '\0'; // Thay thế thành ký tự kết thúc chuỗi
          end_ptr--;       // Lùi con trỏ để kiểm tra tiếp
      }

      // Kiểm tra tiền tố '!"
      if (start_ptr[0] == '!') {
    	  start_ptr++; // Di chuyển con trỏ qua ký tự '!'

    	  // Tách từ dầu tiên
    	  token = strtok_r(start_ptr, " ", &save_ptr);
    	  if(token != NULL) {
    		  if(strcmp(token, "motor") == 0) {
    			  command.cmd = MOTOR;
    			  token = strtok_r(NULL, " ", &save_ptr);
    			  if (token != NULL) {
    				  if (strcmp(token, "on") == 0) command.param = ON;
    				  else if (strcmp(token, "off") == 0) command.param = OFF;
    			  }
    		  }
    		  else if(strcmp(token, "rotate") == 0) {
    			  command.cmd = ROTATE;
    			  token = strtok_r(NULL, " ", &save_ptr);
    			  if (token != NULL) {
    				  if (strcmp(token, "cw") == 0) command.param = CW;
    				  else if (strcmp(token, "ccw") == 0) command.param = CCW;
    			  }
    		  }
    		  else if (strcmp(token, "speed") == 0) {
    			  command.cmd = SPEED;
    			  token = strtok_r(NULL, " ", &save_ptr);
    			  if (token != NULL) {
    				  int speed_val;
    				  if (sscanf(token, "%d", &speed_val) == 1) {
    					  command.param = (uint32_t)speed_val;
    				  }
    			  }
    		  }
    		  else if (strcmp(token, "display") == 0) {
    			  command.cmd = DISPLAY;
    			  token = strtok_r(NULL, " ", &save_ptr);
    			  if (token != NULL) {
    				  if (strcmp(token, "all") == 0) command.param = ALL;
    				  else if (strcmp(token, "temp") == 0) command.param = TEMP;
    				  else if (strcmp(token, "humid") == 0) command.param = HUMID;
    			  }
    		  }
    		  else if (strcmp(token, "ptask2") == 0) {
    			  command.cmd = PTASK2;
    			  token = strtok_r(NULL, " ", &save_ptr);
    			  if (token != NULL) {
    				  int period;
    				  if (sscanf(token, "%d", &period) == 1) {
    					  command.param = (uint32_t)period;
    				  }
    			  }
    		  }
    		  else if (strcmp(token, "ptask3") == 0) {
    			  command.cmd = PTASK3;
    			  token = strtok_r(NULL, " ", &save_ptr);
    			  if (token != NULL) {
    				  int period;
    				  if (sscanf(token, "%d", &period) == 1) {
    					  command.param = (uint32_t)period;
    				  }
    			  }
    		  }
    		  else if (strcmp(token, "ptask4") == 0) {
    			  command.cmd = PTASK4;
    			  token = strtok_r(NULL, " ", &save_ptr);
    			  if (token != NULL) {
    				  int period;
    				  if (sscanf(token, "%d", &period) == 1) {
    					  command.param = (uint32_t)period;
    				  }
    			  }
    		  }
    		  else if (strcmp(token, "ptask5") == 0) {
    			  command.cmd = PTASK5;
    			  token = strtok_r(NULL, " ", &save_ptr);
    			  if (token != NULL) {
    				  int period;
    				  if (sscanf(token, "%d", &period) == 1) {
    					  command.param = (uint32_t)period;
    				  }
    			  }
    		  }
    	  }
      }

      status = osMessageQueuePut(CommandQueueHandle, &command, 0, osWaitForever);
      if(status == osOK) {
    	  status =  osMutexAcquire(UART_AccessMutexHandle, osWaitForever);
    	  if(status == osOK) {
    		  printf("Command sent successfully: %s\r\n", OriginCommand);
    		  osMutexRelease(UART_AccessMutexHandle);
    	  }
      }
  }
  /* USER CODE END UART_ParseCommandTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
