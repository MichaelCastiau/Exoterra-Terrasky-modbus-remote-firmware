/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

#define EN_NEC_READ
#define EN_MODBUS
#define NEC_SEND

#include "nec_decode.h"
#include "types.h"
#include "nec_encode.h"
#include "modbus.h"
#include "ir_commands.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IR_DEBOUNCE_S 60	//Send command every x seconds
#define MODBUS_SLAVE_ID 30
#define DEFAULT_IR_ADDRESS 4

#define DEFAULT_DAY_COMMAND IR_WHITE_COMMAND
#define DEFAULT_NIGHT_COMMAND IR_MOON2_COMMAND
#define DEFAULT_TWILIGHT_COMMAND IR_RED_COMMAND

#define R_TIMEOFDAY 40001
#define R_DAYCODE 40002
#define R_NIGHTCODE 40003
#define R_TWILIGHTCODE 40004

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim3_ch1_trig;
DMA_HandleTypeDef hdma_tim16_ch1_up;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

volatile AppState state = { .mode = CYCLING, .timeOfDay = DAY, .dayFrame = {
		.address = DEFAULT_IR_ADDRESS, .command = DEFAULT_DAY_COMMAND },
		.nightFrame = { .address = 4, .command = DEFAULT_NIGHT_COMMAND },
		.twilightFrame = { .address = 4, .command = DEFAULT_TWILIGHT_COMMAND } };
NEC nec;
uint32_t period_SW_SHUFFLE = 0;
uint32_t period_SW_LEARN = 0;
char OC_count = 0;
volatile ModbusConfig modbus = { .slaveId = MODBUS_SLAVE_ID };

volatile uint32_t irDebouncer = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_IRTIM_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

void processState(AppState state);
void enterLearningState();
void enterCyclingState();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Callback executed when a NEC frame is successfully decoded
 * from the IR receiver. Call NEC_Read beforehand.
 */
void NecDecodedCallback(uint16_t address, uint8_t cmd) {
	if (state.mode != LEARNING)
		return;

	switch (state.timeOfDay) {
	case DAY:
		state.dayFrame.address = address;
		state.dayFrame.command = cmd;
		break;
	case NIGHT:
		state.nightFrame.address = address;
		state.nightFrame.command = cmd;
		break;
	case TWILIGHT:
		state.twilightFrame.address = address;
		state.twilightFrame.command = cmd;
		break;
	}

	return enterCyclingState();

}
/**
 * Callback when a decoded NEC frame is invalid
 */
void NecErrorCallback() {
	// Retry reading
#ifdef EN_NEC_READ
	NEC_Read(&nec);
#endif
}

void NecRepeatCallback() {
#ifdef EN_NEC_READ
	NEC_Read(&nec);
#endif
}

/**
 * Callback executed when the input capture is finished (we captured x bytes)
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
#ifdef EN_NEC_READ
	if (htim == &htim3) {
		// Decode captured data
		NEC_TIM_IC_CaptureCallback(&nec);
	}
#endif
}

void enterLearningState() {
	state.mode = LEARNING;
#ifdef EN_NEC_READ
	HAL_GPIO_WritePin(LED_LEARN_GPIO_Port, LED_LEARN_Pin, GPIO_PIN_SET);

	NEC_Read(&nec);
#endif
}
void enterCyclingState() {
	if (state.mode != CYCLING) {
		state.mode = CYCLING;
		HAL_GPIO_WritePin(LED_LEARN_GPIO_Port, LED_LEARN_Pin, GPIO_PIN_RESET);
#ifdef EN_NEC_READ
		NEC_Stop(&nec);
#endif
		return processState(state);
	}
}

void processState(AppState newState) {
	if (state.mode == CYCLING) {
		state.timeOfDay = newState.timeOfDay;
	}

	HAL_GPIO_WritePin(LED_DAY_GPIO_Port, LED_DAY_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_NIGHT_GPIO_Port, LED_NIGHT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_TWILIGHT_GPIO_Port, LED_TWILIGHT_Pin, GPIO_PIN_RESET);

	//Start the PWM timer for the IR LED
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

	switch (state.timeOfDay) {
	case DAY:
		HAL_GPIO_WritePin(LED_DAY_GPIO_Port, LED_DAY_Pin, GPIO_PIN_SET);
		NEC_Send(&htim16, state.dayFrame.address, state.dayFrame.command);
		break;
	case NIGHT:
		HAL_GPIO_WritePin(LED_NIGHT_GPIO_Port, LED_NIGHT_Pin, GPIO_PIN_SET);
		NEC_Send(&htim16, state.nightFrame.address, state.nightFrame.command);
		break;
	case TWILIGHT:
		HAL_GPIO_WritePin(LED_TWILIGHT_GPIO_Port, LED_TWILIGHT_Pin,
				GPIO_PIN_SET);
		NEC_Send(&htim16, state.twilightFrame.address,
				state.twilightFrame.command);
		break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	switch (GPIO_Pin)
	case SW_SHUFFLE_Pin: {
		if (period_SW_SHUFFLE == 0) {
			period_SW_SHUFFLE = HAL_GetTick();
		}
#ifndef DEBUG
		if (HAL_GetTick() - period_SW_SHUFFLE < 250) {
			return;
		} else {
			period_SW_SHUFFLE = HAL_GetTick();
		}
#endif

		IRMode previousMode = state.mode;

		enterCyclingState();

		if (previousMode == LEARNING)
			return;

		switch (state.timeOfDay) {
		case DAY:
			state.timeOfDay = NIGHT;
			break;
		case NIGHT:
			state.timeOfDay = TWILIGHT;
			break;
		case TWILIGHT:
			state.timeOfDay = DAY;
			break;
		}

		processState(state);
		break;
		case SW_LEARN_Pin:

#ifndef DEBUG
		if (period_SW_LEARN == 0) {
			period_SW_LEARN = HAL_GetTick();
		}

		if (HAL_GetTick() - period_SW_LEARN < 250) {
			return;
		} else {
			period_SW_LEARN = HAL_GetTick();
		}
#endif

		enterLearningState();
		break;
	}
}

/**
 * Callback gets called when a timer overflows
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim14) {
		//500ms passed
		++irDebouncer;

		if (state.mode == LEARNING) {
			HAL_GPIO_TogglePin(LED_LEARN_GPIO_Port, LED_LEARN_Pin);
		}
	}

}

/**
 * Callback gets called when the output compare is executed.
 * This callback is executed for every byte processed by the output compare
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	++OC_count;
	if (htim == &htim16 && OC_count > NEC_FRAME_LENGTH) {
		OC_count = 0;
		HAL_TIM_OC_Stop_DMA(&htim16, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
	}
}

uint16_t modbus_lib_read_handler(uint16_t la) { // la: logical_address
	return 0; //We do not enable reading registers on this device, only writing
}

int modbus_lib_transport_write(uint8_t *buffer, uint16_t start, uint16_t length) {
#ifdef EN_MODBUS
	HAL_UART_Transmit_DMA(&huart1, buffer, length);
	return 0;
#endif
}

/*
 * This method gets excecuted when a 'write register' command
 * is received from the Modbus
 */
uint16_t modbus_lib_write_handler(uint16_t registerAddress, uint16_t value) {
#ifdef EN_MODBUS

	if (irDebouncer < (IR_DEBOUNCE_S * 2)) { //Ignore commands every x seconds
		return 0;
	}

	irDebouncer = 0;
	AppState newState = state;
	switch (registerAddress) {
	case R_TIMEOFDAY: {
		switch (value) {
		case NIGHT:
			newState.timeOfDay = NIGHT;
			break;
		case TWILIGHT:
			newState.timeOfDay = TWILIGHT;
			break;
		case DAY:
			newState.timeOfDay = DAY;
			break;
		default:
			return -1;
		}

		processState(newState);
		break;
	}
	case R_DAYCODE:
		state.dayFrame.command = value;
		break;
	case R_NIGHTCODE:
		state.nightFrame.command = value;
		break;
	case R_TWILIGHTCODE:
		state.twilightFrame.command = value;
		break;
	}
	return 0; // success

#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

#ifdef EN_MODBUS

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) { // Check if it is an "Idle Interrupt"
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);				// clear the interrupt
		modbus.RxCounter++;							// increment the Rx Counter

		const uint16_t start = modbus.RxBfrPos;	// Rx bytes start position (=last buffer position)
		modbus.RxBfrPos = MODBUS_LIB_MAX_BUFFER
				- (uint16_t) huart->hdmarx->Instance->CNDTR;// determine actual buffer position
		uint16_t len = MODBUS_LIB_MAX_BUFFER;		// init len with max. size

		if (modbus.RxRollover < 2) {
			if (modbus.RxRollover) {						// rolled over once
				if (modbus.RxBfrPos <= start)
					len = modbus.RxBfrPos + MODBUS_LIB_MAX_BUFFER - start;// no bytes overwritten
				else
					len = MODBUS_LIB_MAX_BUFFER + 1;// bytes overwritten error
			} else {
				len = modbus.RxBfrPos - start;			// no bytes overwritten
			}
		} else {
			len = MODBUS_LIB_MAX_BUFFER + 2;			// dual rollover error
		}

		if (len && (len <= MODBUS_LIB_MAX_BUFFER)) {
			modbus.start = start;
			modbus.length = len;
			modbus_lib_end_of_telegram(&modbus);
		} else {
			// buffer overflow error:

		}

		modbus.RxRollover = 0;					// reset the Rollover variable
	} else {
		// no idle flag? --> DMA rollover occurred
		modbus.RxRollover++;		// increment Rollover Counter
	}

#endif
}

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
	MX_DMA_Init();
	MX_IRTIM_Init();
	MX_TIM3_Init();
	MX_TIM14_Init();
	MX_TIM16_Init();
	MX_TIM17_Init();
	MX_USART1_UART_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */

#ifdef EN_NEC_READ
	nec.timerHandle = &htim3;

	nec.timerChannel = TIM_CHANNEL_1;
	nec.timerChannelActive = HAL_TIM_ACTIVE_CHANNEL_1;

	nec.timingBitBoundary = 134;
	nec.timingAgcBoundary = 400;
	nec.type = NEC_NOT_EXTENDED;

	nec.NEC_DecodedCallback = NecDecodedCallback;
	nec.NEC_ErrorCallback = NecErrorCallback;
	nec.NEC_RepeatCallback = NecRepeatCallback;
#endif

	processState(state);

	/* USER CODE END 2 */

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_RTC;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief IRTIM Initialization Function
 * @param None
 * @retval None
 */
static void MX_IRTIM_Init(void) {

	/* USER CODE BEGIN IRTIM_Init 0 */

	/* USER CODE END IRTIM_Init 0 */

	/* USER CODE BEGIN IRTIM_Init 1 */

	/* USER CODE END IRTIM_Init 1 */
	/* USER CODE BEGIN IRTIM_Init 2 */

	/* USER CODE END IRTIM_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

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
	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 699;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0xffff;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 4;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 48000;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 500;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	HAL_TIM_Base_Start_IT(&htim14);

	/* USER CODE END TIM14_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 95;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 0xffff;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	__HAL_DBGMCU_FREEZE_TIM16();

	/* USER CODE END TIM16_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 9;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 125;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 125 / 2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */

	/* USER CODE END TIM17_Init 2 */

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
	huart1.Init.BaudRate = 4800;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT;
	huart1.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
	if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  // enable idle line detection
	HAL_UART_Receive_DMA(&huart1, &modbus.RxBuffer[0], MODBUS_LIB_MAX_BUFFER);

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	/* DMA1_Channel4_5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	LED_LEARN_Pin | LED_DAY_Pin | LED_NIGHT_Pin | LED_TWILIGHT_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : SW_SHUFFLE_Pin */
	GPIO_InitStruct.Pin = SW_SHUFFLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(SW_SHUFFLE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SW_LEARN_Pin */
	GPIO_InitStruct.Pin = SW_LEARN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(SW_LEARN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_LEARN_Pin LED_DAY_Pin LED_NIGHT_Pin LED_TWILIGHT_Pin */
	GPIO_InitStruct.Pin = LED_LEARN_Pin | LED_DAY_Pin | LED_NIGHT_Pin
			| LED_TWILIGHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_IR;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
