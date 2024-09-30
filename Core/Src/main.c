/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "usbd_cdc_if.h"
#include "Led.h"
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for Blinker */
osThreadId_t BlinkerHandle;
const osThreadAttr_t Blinker_attributes = { .name = "Blinker", .stack_size = 128
		* 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for usbTerminal */
osThreadId_t usbTerminalHandle;
const osThreadAttr_t usbTerminal_attributes = { .name = "usbTerminal",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for taskToRpi */
osThreadId_t taskToRpiHandle;
const osThreadAttr_t taskToRpi_attributes = { .name = "taskToRpi", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for taskToSensor */
osThreadId_t taskToSensorHandle;
const osThreadAttr_t taskToSensor_attributes = { .name = "taskToSensor",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for quToSenzor */
osMessageQueueId_t quToSenzorHandle;
const osMessageQueueAttr_t quToSenzor_attributes = { .name = "quToSenzor" };
/* Definitions for quToRpi */
osMessageQueueId_t quToRpiHandle;
const osMessageQueueAttr_t quToRpi_attributes = { .name = "quToRpi" };
/* Definitions for quTest1 */
osMessageQueueId_t quTest1Handle;
const osMessageQueueAttr_t quTest1_attributes = { .name = "quTest1" };
/* Definitions for quTest2 */
osMessageQueueId_t quTest2Handle;
const osMessageQueueAttr_t quTest2_attributes = { .name = "quTest2" };
/* Definitions for timer01 */
osTimerId_t timer01Handle;
const osTimerAttr_t timer01_attributes = { .name = "timer01" };
/* Definitions for timer02 */
osTimerId_t timer02Handle;
const osTimerAttr_t timer02_attributes = { .name = "timer02" };
/* Definitions for mutex01 */
osMutexId_t mutex01Handle;
const osMutexAttr_t mutex01_attributes = { .name = "mutex01" };
/* Definitions for mutex02 */
osMutexId_t mutex02Handle;
const osMutexAttr_t mutex02_attributes = { .name = "mutex02" };
/* Definitions for myRecursiveMutex01 */
osMutexId_t myRecursiveMutex01Handle;
const osMutexAttr_t myRecursiveMutex01_attributes = { .name =
		"myRecursiveMutex01", .attr_bits = osMutexRecursive, };
/* Definitions for sem01 */
osSemaphoreId_t sem01Handle;
const osSemaphoreAttr_t sem01_attributes = { .name = "sem01" };
/* Definitions for sem02 */
osSemaphoreId_t sem02Handle;
const osSemaphoreAttr_t sem02_attributes = { .name = "sem02" };
/* Definitions for sem03 */
osSemaphoreId_t sem03Handle;
const osSemaphoreAttr_t sem03_attributes = { .name = "sem03" };
/* Definitions for myCountingSem01 */
osSemaphoreId_t myCountingSem01Handle;
const osSemaphoreAttr_t myCountingSem01_attributes =
		{ .name = "myCountingSem01" };
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void BlinkerStart(void *argument);
void usbTerminalStart(void *argument);
void taskToRpiStart(void *argument);
void taskToSensorStart(void *argument);
void timer01Callback(void *argument);
void timer02Callback(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of mutex01 */
	mutex01Handle = osMutexNew(&mutex01_attributes);

	/* creation of mutex02 */
	mutex02Handle = osMutexNew(&mutex02_attributes);

	/* Create the recursive mutex(es) */
	/* creation of myRecursiveMutex01 */
	myRecursiveMutex01Handle = osMutexNew(&myRecursiveMutex01_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of sem01 */
	sem01Handle = osSemaphoreNew(1, 0, &sem01_attributes);

	/* creation of sem02 */
	sem02Handle = osSemaphoreNew(1, 1, &sem02_attributes);

	/* creation of sem03 */
	sem03Handle = osSemaphoreNew(1, 1, &sem03_attributes);

	/* creation of myCountingSem01 */
	myCountingSem01Handle = osSemaphoreNew(2, 2, &myCountingSem01_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* Create the timer(s) */
	/* creation of timer01 */
	timer01Handle = osTimerNew(timer01Callback, osTimerPeriodic, NULL,
			&timer01_attributes);

	/* creation of timer02 */
	timer02Handle = osTimerNew(timer02Callback, osTimerOnce, NULL,
			&timer02_attributes);

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of quToSenzor */
	quToSenzorHandle = osMessageQueueNew(128, sizeof(uint8_t),
			&quToSenzor_attributes);

	/* creation of quToRpi */
	quToRpiHandle = osMessageQueueNew(128, sizeof(uint8_t),
			&quToRpi_attributes);

	/* creation of quTest1 */
	quTest1Handle = osMessageQueueNew(16, sizeof(uint8_t), &quTest1_attributes);

	/* creation of quTest2 */
	quTest2Handle = osMessageQueueNew(16, sizeof(uint8_t), &quTest2_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of Blinker */
	BlinkerHandle = osThreadNew(BlinkerStart, NULL, &Blinker_attributes);

	/* creation of usbTerminal */
	usbTerminalHandle = osThreadNew(usbTerminalStart, NULL,
			&usbTerminal_attributes);

	/* creation of taskToRpi */
	taskToRpiHandle = osThreadNew(taskToRpiStart, NULL, &taskToRpi_attributes);

	/* creation of taskToSensor */
	taskToSensorHandle = osThreadNew(taskToSensorStart, NULL,
			&taskToSensor_attributes);

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
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}

	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* USART1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	/* USART2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 10000 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 60000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : BOARD_LED_Pin */
	GPIO_InitStruct.Pin = BOARD_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SW_delay_Pin SW_dropByte_Pin SW_insertByte_Pin SW_dataError_Pin
	 SW_headerError_Pin */
	GPIO_InitStruct.Pin = SW_delay_Pin | SW_dropByte_Pin | SW_insertByte_Pin
			| SW_dataError_Pin | SW_headerError_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

bool delayFlag;
bool dropFlag;
bool insertFlag;
bool dataErrorFlag;
bool headerErrorFlag;
uint8_t rxu1;
uint8_t rxu2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//		rxu1 = (uint8_t) (huart->Instance->DR & 0xFF);
		xQueueSendFromISR(quToSenzorHandle, &rxu1, &xHigherPriorityTaskWoken);

		// Restart UART reception
		HAL_UART_Receive_IT(&huart1, &rxu1, 1);
		// Yield if a higher priority task was woken
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	if (huart->Instance == USART2) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//		rxu2 = (uint8_t) (huart->Instance->DR & 0xFF);
		xQueueSendFromISR(quToRpiHandle, &rxu2, &xHigherPriorityTaskWoken);

		// Restart UART reception
		HAL_UART_Receive_IT(&huart2, &rxu2, 1);
		// Yield if a higher priority task was woken
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */
	HAL_UART_Receive_IT(&huart1, &rxu1, 1);
	HAL_UART_Receive_IT(&huart2, &rxu2, 1);
	/* Infinite loop */
	for (;;) {
		delayFlag =			(HAL_GPIO_ReadPin(SW_delay_GPIO_Port, 			SW_delay_Pin) == GPIO_PIN_RESET) ? true : false;
		dropFlag =			(HAL_GPIO_ReadPin(SW_dropByte_GPIO_Port, 		SW_dropByte_Pin) == GPIO_PIN_RESET) ? true : false;
		insertFlag =		(HAL_GPIO_ReadPin(SW_insertByte_GPIO_Port, 		SW_insertByte_Pin) == GPIO_PIN_RESET) ? true : false;
		dataErrorFlag =		(HAL_GPIO_ReadPin(SW_dataError_GPIO_Port, 		SW_dataError_Pin) == GPIO_PIN_RESET) ? true : false;
		headerErrorFlag = 	(HAL_GPIO_ReadPin(SW_headerError_GPIO_Port,		SW_headerError_Pin) == GPIO_PIN_RESET) ? true : false;
		osDelay(2000);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_BlinkerStart */
/**
 * @brief Function implementing the Blinker thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_BlinkerStart */
void BlinkerStart(void *argument) {
	/* USER CODE BEGIN BlinkerStart */
	/* Infinite loop */
	for (;;) {
		int tajm = ulTaskNotifyTake(pdTRUE, 5000);
		if (0 == tajm) {
			ledBlinkCount(10, 1, 19);// 10 puta (1:19 duty cycle) = 200mS smanjenim intenzitetom
		} else {
			ledBlink(10, 10);
		}

	}
	/* USER CODE END BlinkerStart */
}

/* USER CODE BEGIN Header_usbTerminalStart */
/**
 * @brief Function implementing the usbTerminal thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_usbTerminalStart */
void usbTerminalStart(void *argument) {
	/* USER CODE BEGIN usbTerminalStart */
	/* Infinite loop */
	for (;;) {
		HAL_UART_Transmit(&huart1, (uint8_t*) "1", 1, 10);
		HAL_UART_Transmit(&huart2, (uint8_t*) "2", 1, 10);
		osDelay(1000);
	}
	/* USER CODE END usbTerminalStart */
}

/* USER CODE BEGIN Header_taskToRpiStart */
/**
 * @brief Function implementing the taskToRpi thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskToRpiStart */
void taskToRpiStart(void *argument) {
	/* USER CODE BEGIN taskToRpiStart */
	uint8_t rxByte;
	uint8_t cnt;
	/* Infinite loop */
	for (;;) {
		cnt = 0;
		xQueueReceive(quToRpiHandle, &rxByte, portMAX_DELAY);
		osDelay(100);			// posle prvog bajta, cekam dok sigurno pristignu svi ostali i tek onda prosledjujem dalje
		if (delayFlag) {
			osDelay(3000);
		}
		xTaskNotifyGive(BlinkerHandle);
		CDC_Transmit_FS(&rxByte, 1);
		//
		// 0123456789
		//
		while (xQueueReceive(quToRpiHandle, &rxByte, 0) == pdPASS) {
			cnt++;	// ovo je prvi bajt jer je izvan petlje vec primljen nulti
			// unosim razne greske na raznim pozicijama
			// pozicija je uvek drugacija jer cemu sluzi pogresan bajt ako cu posle da ga dropujem
			// ili bi se ponistili drop i insert
			if ((dropFlag) && (cnt == 3)) {
				continue; 									// dropuje ovaj bajt
			};
			if ((insertFlag) && (cnt == 4)) {
				uint8_t bzv = 43;	// ascii '+'
				while (USBD_OK != CDC_Transmit_FS(&bzv, 1)) { // insertuje neki bezvezni bajt
				}
			};
			if ((dataErrorFlag) && (cnt == 2)) {
//				rxByte++;									// unosi gresku u podatke
				rxByte = 101;								// ascii 'e'
			};
			if ((headerErrorFlag) && (cnt == 1)) {
//				rxByte++;									// unosi gresku u header (nulti i prvi bajt)
				rxByte = 72;								// ascii 'h'
			};
			CDC_Transmit_FS(&rxByte, 1);
			osDelay(1);
		}
	}
	/* USER CODE END taskToRpiStart */
}

/* USER CODE BEGIN Header_taskToSensorStart */
/**
 * @brief Function implementing the taskToSensor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskToSensorStart */
void taskToSensorStart(void *argument) {
	/* USER CODE BEGIN taskToSensorStart */
	uint8_t rxByte;
	/* Infinite loop */
	for (;;) {
		xQueueReceive(quToSenzorHandle, &rxByte, portMAX_DELAY);
		CDC_Transmit_FS(&rxByte, 1);
		while (xQueueReceive(quToSenzorHandle, &rxByte, 0) == pdPASS) {
			CDC_Transmit_FS(&rxByte, 1);
			osDelay(2);
		}
	}
	/* USER CODE END taskToSensorStart */
}

/* timer01Callback function */
void timer01Callback(void *argument) {
	/* USER CODE BEGIN timer01Callback */

	/* USER CODE END timer01Callback */
}

/* timer02Callback function */
void timer02Callback(void *argument) {
	/* USER CODE BEGIN timer02Callback */

	/* USER CODE END timer02Callback */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM11 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM11) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
