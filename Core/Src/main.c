/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "retarget.h"
#include <stdio.h>
#include <stdbool.h>
#include <dwt_delay.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	MODE_MOTOR_STOP,
	MODE_MOTOR_CALIBRATION,
	MODE_MOTOR_READY,
	MODE_MOTOR_START,
	MODE_MOTOR_RUN
} MODE_MOTOR_t;

typedef enum {
	BEMF_FALLING = 0, BEMF_RISING
} BEMF_DIR_t;

typedef enum {
	PHASE_A = 0, PHASE_B, PHASE_C
} PHASE_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMP1_CALLBACK 1
#define COMP2_CALLBACK 2
#define COMP3_CALLBACK 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp3;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for comparatorTask */
osThreadId_t comparatorTaskHandle;
const osThreadAttr_t comparatorTask_attributes = {
  .name = "comparatorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for analogInTask */
osThreadId_t analogInTaskHandle;
const osThreadAttr_t analogInTask_attributes = {
  .name = "analogInTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for controllerTask */
osThreadId_t controllerTaskHandle;
const osThreadAttr_t controllerTask_attributes = {
  .name = "controllerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for positioningTask */
osThreadId_t positioningTaskHandle;
const osThreadAttr_t positioningTask_attributes = {
  .name = "positioningTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* USER CODE BEGIN PV */
uint8_t compTrig = 0;

uint8_t buffer[200];
volatile uint16_t strSize;
volatile uint8_t commutationStepCounter;
volatile uint8_t waitForCommutation = 1;

uint32_t tim2cnt;
uint8_t RXBuffer[200];
bool USART1DataFlag = false;
float adcIntegral = 0;
bool adcDMAStarted = false;

volatile int16_t newPWM = 100;
volatile int16_t setPWM = 100;
volatile int16_t compWindowOffset = 250 * 2; //this opens a comparator window to block overshooting signals from triggering (adjust to Vbat)
volatile uint16_t oc5Value = 200; //preset offset-value (for fine tuning) of the DAC voltage output as the reference voltage for the compartors (about 0V or Vbat/2)
volatile uint8_t pwmState = 0; //indicates whether a zero crossing (pwm low) or a Vbat/2 crossing (pwm high) will be detected; 0 = PWM low detection, 1 = PWM high detection

ADC_ChannelConfTypeDef sConfig;
TIM_OC_InitTypeDef sConfigOC5B;
TIM_OC_InitTypeDef sConfigOC5A;
bool firstStart = false;

volatile bool readRotation = false;

float measuredRPM = 0;
float inputRPM = 0;
uint32_t rpm_cnt = 0;

uint16_t input_pwm_min = 1000;
uint16_t input_pwm_max = 2000;

uint32_t inputDutyCycle = 0;
uint32_t pastInputDutyCycle = 0;
float inputFrequency = 0;
uint16_t oc5ValueOld = 0;

uint16_t adcVbus = 3900;

PHASE_t phase_bemf = PHASE_B;

BEMF_DIR_t bemf_dir = BEMF_FALLING;

MODE_MOTOR_t mode_motor = MODE_MOTOR_STOP;

uint16_t adc2DMA[10] = { 0 };

float errorPWM = 0.0;
float integralErrorPWM = 0.0;

uint32_t IntegralOffset = 3800;
uint32_t errorVal = 5;

bool a = false;
bool x = false;
bool y = false;
bool z = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP3_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM3_Init(void);
void StartComparatorTask(void *argument);
void StartAnalogInTask(void *argument);
void StartControllerTask(void *argument);
void StartPositioningTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void commutateNow_0(void) {
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_SET); //PHASE A HIGH
	HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_RESET); //PHASE B FLOATING
	HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_SET); //PHASE C LOW
	HAL_GPIO_WritePin(IN_C_GPIO_Port, IN_C_Pin, GPIO_PIN_RESET);
}

void commutateNow_1(void) {
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_RESET); //PHASE A FLOATING
	HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_SET); //PHASE B HIGH
	HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_SET); //PHASE C LOW
	HAL_GPIO_WritePin(IN_C_GPIO_Port, IN_C_Pin, GPIO_PIN_RESET);
}

void commutateNow_2(void) {
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_SET); //PHASE A LOW
	HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_SET); //PHASE B HIGH
	HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_RESET); //PHASE C FLOATING
	HAL_GPIO_WritePin(IN_C_GPIO_Port, IN_C_Pin, GPIO_PIN_RESET);
}

void commutateNow_3(void) {
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_SET); //PHASE A LOW
	HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_RESET); //PHASE B FLOATING
	HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_SET); //PHASE C HIGH
	HAL_GPIO_WritePin(IN_C_GPIO_Port, IN_C_Pin, GPIO_PIN_SET);

}

void commutateNow_4(void) {
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_RESET); //PHASE A FLOATING
	HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_SET); //PHASE B LOW
	HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_SET); //PHASE C HIGH
	HAL_GPIO_WritePin(IN_C_GPIO_Port, IN_C_Pin, GPIO_PIN_SET);

}
void commutateNow_5(void) {
	HAL_GPIO_WritePin(INH_A_GPIO_Port, INH_A_Pin, GPIO_PIN_SET); //PHASE A HIGH
	HAL_GPIO_WritePin(IN_A_GPIO_Port, IN_A_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(INH_B_GPIO_Port, INH_B_Pin, GPIO_PIN_SET); //PHASE B LOW
	HAL_GPIO_WritePin(IN_B_GPIO_Port, IN_B_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(INH_C_GPIO_Port, INH_C_Pin, GPIO_PIN_RESET); //PHASE C FLOATING
	HAL_GPIO_WritePin(IN_C_GPIO_Port, IN_C_Pin, GPIO_PIN_RESET);
}

void commutationPattern(uint8_t step) {
	if (step == NEXT && waitForCommutation == 1) {
		bemf_dir = !bemf_dir;
		if (commutationStepCounter < STEP_5)
			commutationStepCounter++;
		else {
			commutationStepCounter = STEP_0;
		}
		switch (commutationStepCounter) {
		case STEP_0:
			commutateNow_0();
			COMPDELAY
			;
			waitForCommutation = 0;
			phase_bemf = PHASE_B;
			bemf_dir = BEMF_RISING;
			HAL_COMP_Start_IT(&hcomp2);
			break;
		case STEP_1:
			commutateNow_1();
			COMPDELAY
			;
			waitForCommutation = 0;
			phase_bemf = PHASE_A;
			bemf_dir = BEMF_FALLING;
			HAL_COMP_Start_IT(&hcomp1);
			break;
		case STEP_2:
			commutateNow_2();
			COMPDELAY
			;
			waitForCommutation = 0;
			phase_bemf = PHASE_C;
			bemf_dir = BEMF_RISING;
			HAL_COMP_Start_IT(&hcomp3);
			break;
		case STEP_3:
			commutateNow_3();
			COMPDELAY
			;
			waitForCommutation = 0;
			phase_bemf = PHASE_B;
			bemf_dir = BEMF_FALLING;
			HAL_COMP_Start_IT(&hcomp2);
			break;
		case STEP_4:
			commutateNow_4();
			COMPDELAY
			;
			waitForCommutation = 0;
			phase_bemf = PHASE_A;
			bemf_dir = BEMF_RISING;
			HAL_COMP_Start_IT(&hcomp1);
			break;
		case STEP_5:
			commutateNow_5();
			COMPDELAY
			;
			waitForCommutation = 0;
			phase_bemf = PHASE_C;
			bemf_dir = BEMF_FALLING;
			HAL_COMP_Start_IT(&hcomp3);
			break;
		}
	} else {
		waitForCommutation = 0;
		switch (step) {
		case STEP_0:
			commutateNow_0();
			break;
		case STEP_1:
			commutateNow_1();
			break;
		case STEP_2:
			commutateNow_2();
			break;
		case STEP_3:
			commutateNow_3();
			break;
		case STEP_4:
			commutateNow_4();
			break;
		case STEP_5:
			commutateNow_5();
			break;
		}
	}
}

void startMotor() {
	setPWM = newPWM = 150;

	TIM1->CCR1 = setPWM;
	TIM1->CCR5 = setPWM + compWindowOffset;

	commutateNow_0();
	HAL_Delay(20);
	commutateNow_1();
	HAL_Delay(20);
	commutateNow_2();
	HAL_Delay(20);
	commutateNow_3();
	HAL_Delay(20);
	commutateNow_4();
	HAL_Delay(20);
	commutateNow_5();
	HAL_Delay(20);

	uint8_t step = 0;
	uint16_t i = 4300;


	while (i > 1300) {
		DWT_Delay(i);
		commutationPattern(step);
		setPWM += 1;
		TIM1->CCR1 = newPWM = setPWM;
		TIM1->CCR1 = setPWM + compWindowOffset;
		step += 1;
		step %= 6;
		i -= 30;
	}

	waitForCommutation = 1;

	commutationStepCounter = step;
	commutationPattern(NEXT);

	mode_motor = MODE_MOTOR_RUN;
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
  MX_DMA_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_COMP3_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	RetargetInit(&huart1);
	DWT_Init();
	printf("Test\r\n");
	printf("asdf\r\n");

//	HAL_ADC_Start(&hadc4);
//	while (__HAL_ADC_GET_FLAG(&hadc4, ADC_FLAG_EOC) != SET)
//		;

//	adcVbus = HAL_ADC_GetValue(&hadc4);

	sConfigOC5B.OCMode = TIM_OCMODE_PWM1;
	sConfigOC5B.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC5B.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC5B.OCPolarity = TIM_OCPOLARITY_HIGH; //HIGH for PWM low detection

	sConfigOC5A.OCMode = TIM_OCMODE_PWM1;
	sConfigOC5A.OCFastMode = TIM_OCFAST_DISABLE; //DISABLE
	sConfigOC5A.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC5A.OCPolarity = TIM_OCPOLARITY_LOW; //LOW for PWM high detection

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);
	//HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);

	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adc2DMA, 3);

	__HAL_DMA_DISABLE_IT(&hdma_adc2, DMA_IT_TC);
	__HAL_DMA_DISABLE_IT(&hdma_adc2, DMA_IT_HT);
	__HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_EOS);
	__HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_EOC);

	HAL_TIM_Base_Start(&htim2);

	setPWM = TIM1->CCR1 = TIM1->CCR5 = 0;

	printf("Mulai!\r\n");

//	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of comparatorTask */
  comparatorTaskHandle = osThreadNew(StartComparatorTask, NULL, &comparatorTask_attributes);

  /* creation of analogInTask */
  analogInTaskHandle = osThreadNew(StartAnalogInTask, NULL, &analogInTask_attributes);

  /* creation of controllerTask */
  controllerTaskHandle = osThreadNew(StartControllerTask, NULL, &controllerTask_attributes);

  /* creation of positioningTask */
  positioningTaskHandle = osThreadNew(StartPositioningTask, NULL, &positioningTask_attributes);

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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV2;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */
  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO2;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp1.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp1.Init.Output = COMP_OUTPUT_NONE;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRCE_TIM1OC5;
  hcomp1.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO2;
  hcomp2.Init.Output = COMP_OUTPUT_NONE;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRCE_TIM1OC5;
  hcomp2.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief COMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP3_Init(void)
{

  /* USER CODE BEGIN COMP3_Init 0 */

  /* USER CODE END COMP3_Init 0 */

  /* USER CODE BEGIN COMP3_Init 1 */

  /* USER CODE END COMP3_Init 1 */
  hcomp3.Instance = COMP3;
  hcomp3.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp3.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp3.Init.Output = COMP_OUTPUT_NONE;
  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRCE_TIM1OC5;
  hcomp3.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp3.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP3_Init 2 */

  /* USER CODE END COMP3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 1800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC5REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 72 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 72 - 1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 72 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_A_Pin|OUT_B_Pin|OUT_C_Pin|LED2_Pin
                          |LED1_Pin|IN_A_Pin|INH_A_Pin|IN_B_Pin
                          |INH_B_Pin|IN_C_Pin|INH_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADC_CS_Pin */
  GPIO_InitStruct.Pin = ADC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_A_Pin OUT_B_Pin OUT_C_Pin IN_A_Pin
                           IN_B_Pin IN_C_Pin */
  GPIO_InitStruct.Pin = OUT_A_Pin|OUT_B_Pin|OUT_C_Pin|IN_A_Pin
                          |IN_B_Pin|IN_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_VBUS_Pin */
  GPIO_InitStruct.Pin = ADC_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin INH_A_Pin INH_B_Pin
                           INH_C_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin|INH_A_Pin|INH_B_Pin
                          |INH_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
	if (hcomp->Instance == COMP1 && waitForCommutation == 0) {
		HAL_COMP_Stop_IT(&hcomp1);
		compTrig = COMP1_CALLBACK;
	} else if (hcomp->Instance == COMP2 && waitForCommutation == 0) {
		HAL_COMP_Stop_IT(&hcomp2);
		compTrig = COMP2_CALLBACK;
	} else if (hcomp->Instance == COMP3 && waitForCommutation == 0) {
		HAL_COMP_Stop_IT(&hcomp3);
		compTrig = COMP3_CALLBACK;
	}
	osThreadFlagsSet(comparatorTaskHandle, 0x1);

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	static uint32_t total;

	//Motor Kecil 930KV
	//float IntegralOffset = 0.0750;

	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		switch (phase_bemf) {
		case PHASE_A:
			total += adc2DMA[0];
			break;
		case PHASE_B:
			total += adc2DMA[1];
			break;
		case PHASE_C:
			total += adc2DMA[2];
			break;
		}
		if (total >= IntegralOffset) {
			if(y){
				if(total - IntegralOffset > errorVal){
					total--;
				}else{
					total = total - IntegralOffset;
				}
			}else{
				total = 0;
			}
			commutationPattern(NEXT);
		}
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	static uint32_t Count_RisingEdge;
	static uint32_t Count_FallingEdge;
	static uint32_t Count_Freq1;
	static uint32_t Count_Freq2;
	static bool Freq_State;
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		Count_RisingEdge = TIM15->CCR1;
		if(Freq_State == 0){
			Freq_State = 1;
			Count_Freq1 = Count_RisingEdge;
		}
		else if(Freq_State == 1){
			Freq_State = 0;
			Count_Freq2 = Count_RisingEdge;
			if(Count_Freq2 > Count_Freq1) inputFrequency = 1/((float)Count_Freq2 - (float)Count_Freq1) * 1000000; //in kHz;
		}
	}else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		Count_FallingEdge = TIM15->CCR2;

		if(Count_RisingEdge < Count_FallingEdge){
			inputDutyCycle = Count_FallingEdge - Count_RisingEdge;
			Count_RisingEdge = 0;
			Count_FallingEdge = 0;
			osThreadFlagsSet(analogInTaskHandle, 0x1);
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartComparatorTask */
/**
 * @brief  Function implementing the comparatorTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartComparatorTask */
void StartComparatorTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	bool d = false;
	/* Infinite loop */
	for (;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);

		if (compTrig == COMP1_CALLBACK && waitForCommutation == 0) {
				__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

			waitForCommutation = 1;
		} else if (compTrig == COMP2_CALLBACK && waitForCommutation == 0) {
				__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

			waitForCommutation = 1;
		} else if (compTrig == COMP3_CALLBACK && waitForCommutation == 0) {
				__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

			waitForCommutation = 1;
		}

		if (setPWM >= 200 && !pwmState) {
			HAL_GPIO_WritePin(GPIOB, OUT_A_Pin | OUT_B_Pin | OUT_C_Pin,
					GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_5);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2021);
			compWindowOffset = -80;

			HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC5A, TIM_CHANNEL_5);
			TIM1->CCR5 = setPWM + compWindowOffset;
			pwmState = 1;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);
		}

		if (setPWM < 200 && pwmState) {
			HAL_GPIO_WritePin(GPIOB, OUT_A_Pin | OUT_B_Pin | OUT_C_Pin,
					GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_5);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
			compWindowOffset = 250;

			HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC5B, TIM_CHANNEL_5);
			TIM1->CCR5 = setPWM + compWindowOffset;
			pwmState = 0;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_5);
		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartAnalogInTask */
/**
 * @brief Function implementing the analogInTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAnalogInTask */
void StartAnalogInTask(void *argument)
{
  /* USER CODE BEGIN StartAnalogInTask */
//	uint16_t analogIn = 0;
//	uint16_t analogVbus = 0;
//	HAL_ADC_Start(&hadc3);
	/* Infinite loop */
	for (;;) {
//		analogIn = (uint16_t) HAL_ADC_GetValue(&hadc3);
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		switch (mode_motor) {
		case MODE_MOTOR_STOP:
//			if ((float) analogIn >= 4095.0 * 0.2) {
			if ((float) inputDutyCycle >=  1150) {
				mode_motor = MODE_MOTOR_START;
			}
			break;
		case MODE_MOTOR_CALIBRATION:
			break;
		case MODE_MOTOR_START:
			startMotor();
			break;
		case MODE_MOTOR_RUN:

			if(inputDutyCycle > 1400){
				y = true;
			}else{
				y = false;
			}

			if(pastInputDutyCycle > inputDutyCycle){
				errorVal = 7;
				IntegralOffset = 3800;
			}else if(pastInputDutyCycle < inputDutyCycle){
				errorVal = 5;
				IntegralOffset = 2400;
			}else{
				errorVal = 5;
				IntegralOffset = 3800;
			}
			pastInputDutyCycle = inputDutyCycle;

			if(!y){
				newPWM = map(inputDutyCycle, input_pwm_min, input_pwm_max, PWM_MIN, PWM_MAX);
			}else{
				setPWM = newPWM = map(inputDutyCycle, input_pwm_min, input_pwm_max, PWM_MIN, PWM_MAX);
				TIM1->CCR1 = setPWM;
				TIM1->CCR5 = TIM1->CCR1 + compWindowOffset;
			}

//			if ((float) analogIn < 4095.0 * 0.1) {
			if ((float) inputDutyCycle < 1150) {
				HAL_NVIC_SystemReset();
			}
			break;
		default:
			break;
		}
		osDelay(10);

	}
  /* USER CODE END StartAnalogInTask */
}

/* USER CODE BEGIN Header_StartControllerTask */
/**
 * @brief Function implementing the controllerTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControllerTask */
void StartControllerTask(void *argument)
{
  /* USER CODE BEGIN StartControllerTask */
	float kp = 0.07;
	float ki = 0.005;
	float outputPWM = 0.0;
	float outputIntegralPWM = 0.0;
	/* Infinite loop */
	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
	for(;;)
	  {
		if(y){
			osDelay(5);
			continue;
		}
		if (mode_motor == MODE_MOTOR_RUN){
			errorPWM = (float) (setPWM - newPWM);
			integralErrorPWM += errorPWM;

			outputIntegralPWM = (integralErrorPWM * ki);
			outputIntegralPWM = constrain(outputIntegralPWM, -500.0, 500.0);

			outputPWM = -(errorPWM * kp) + (-outputIntegralPWM);
			outputPWM = constrain(outputPWM, -500.0, 500.0);
			setPWM += (int16_t) outputPWM;
			setPWM = constrain(setPWM, 0, PWM_MAX);
			TIM1->CCR1 = setPWM;
			TIM1->CCR5 = TIM1->CCR1 + compWindowOffset;
		}
		osDelay(5);
	  }
  /* USER CODE END StartControllerTask */
}

/* USER CODE BEGIN Header_StartPositioningTask */
/**
* @brief Function implementing the positioningTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPositioningTask */
void StartPositioningTask(void *argument)
{
  /* USER CODE BEGIN StartPositioningTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(osWaitForever);
  }
  /* USER CODE END StartPositioningTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAithL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
