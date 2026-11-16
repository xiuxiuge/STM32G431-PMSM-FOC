/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include <math.h>
#include "vofa.h"

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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

int cnt = 0;
// 开环控制
float theta_e = 0.0f;

const float PI = 3.14159265358979323846f;
const float Ts = 50e-6f;
const float fe = 5.0f;      // 电角频率 5 Hz
float Uq = 0.0f;      // 很小的电压矢量幅值
float Ud = 2.0f;
const float Vbus = 13.0f;

#define ENCODER_CPR 10000
#define MOTOR_POLE_PAIRS 5	// 极对数

// 编码器读取
volatile int32_t encoder_position = 0;	// 编码位置
volatile float theta_mech_deg = 0.0f;	// 机械角度
volatile float theta_mech_rad = 0.0f;	// 机械角度

volatile float theta_elec_rad = 0.0f;	// 电角度
volatile float theta_elec_deg = 0.0f;	// 电角度

// 电流采样零点校准
volatile uint32_t adc_raw = 0;			// ADC原始值
volatile float adc_voltage = 0.0f;		// ADC电压
volatile HAL_StatusTypeDef adc_status;	// ADC状态

volatile uint32_t adc_min = 4095;
volatile uint32_t adc_max = 0;
volatile uint32_t adc_avg = 0;

#define ADC_SAMPLE_COUNT 1000

volatile uint32_t adc_avg_1 = 0;
volatile uint32_t adc_avg_2 = 0;
volatile uint32_t adc_avg_3 = 0;

volatile uint32_t adc_error_count = 0;

#define ADC_VREF       3.3f
#define ADC_FULL_SCALE 4095.0f
#define SHUNT_R        0.010f
#define CURRENT_GAIN   5.18f

volatile uint32_t adc_offset[3] = {0};

volatile float current_1 = 0.0f;
volatile float current_2 = 0.0f;
volatile float current_3 = 0.0f;

// TIM1硬件触发ADC注入组、ADC 注入组三通道同步采样
volatile uint32_t adc_injected_count = 0;		// 注入组累计完成次数
volatile uint32_t adc_injected_freq = 0;		// 每秒完成次数
volatile uint32_t adc_injected_raw[3] = {0};	// ADC 采样值

// 当前PWM周期是否具备有效采样窗口
volatile uint8_t adc_window_valid = 0;

// 方便调试
volatile uint32_t adc_window_ticks = 0;
volatile uint32_t adc_ccr4_value = 0;

volatile float current_phase[3] = {0.0f};
volatile float current_sum = 0.0f;

#define ADC_OFFSET_SAMPLES 1000U

// 注入组偏置
static uint32_t adc_offset_sum[3] = {0};
static uint32_t adc_offset_count = 0;

volatile uint8_t adc_offset_ready = 0;

// 克拉克变换
volatile float Ialpha = 0.0f;
volatile float Ibeta  = 0.0f;

// 当前真实矢量的模长
volatile float Ialpha_beta_magnitude = 0.0f;

// 帕克变换
volatile float Id = 0.0f;
volatile float Iq = 0.0f;
volatile float Idq_magnitude = 0.0f;
volatile float magnitude_error = 0.0f;

// 校准电角度零点
volatile int32_t encoder_align = 0;
volatile float theta_elec_calibrated = 0.0f;
volatile uint8_t encoder_calibrated = 0;

volatile float angle_diff = 0.0f;

//TIM控制模式状态机
typedef enum
{
    FOC_MODE_ALIGN,			// 固定磁场对齐
    FOC_MODE_ENCODER_TEST,	// 编码器角度控制测试
	FOC_MODE_CURRENT_LOOP,	// 电流环模式
	FOC_MODE_SPEED_LOOP,	// 速度环模式
	FOC_MODE_POSITION_LOOP	// 位置环模式
} FOC_Mode_t;

volatile FOC_Mode_t foc_mode = FOC_MODE_ALIGN;

volatile float Uq_test = 0.0f;

typedef struct
{
    float kp;
    float ki;
    float integral;

} PI_Controller;

static PI_Controller pi_d = {0};
static PI_Controller pi_q = {0};
static PI_Controller pi_speed = {0};
static PI_Controller pi_position = {0};

static float Id_ref = 0.0f;
static float Iq_ref = 0.5f;

#define CURRENT_LOOP_TS  0.00005f

static volatile float current_ud_cmd = 0.0f;	// 传递电压指令
static volatile float current_uq_cmd = 0.0f;
static float current_voltage_limit = 3.5f; 		// 实验电压限制

volatile float motor_rpm_raw = 0.0f;
volatile float motor_rpm_filtered = 0.0f;

#define ADC_WAIT_TICKS      340U
#define ADC_SEQUENCE_TICKS  680U
#define ADC_MARGIN_TICKS    100U

float iq_error_avg = 0;

// 速度环
static float speed_ref_rpm = 200.0f;

#define SPEED_LOOP_TS  0.001f
#define SPEED_IQ_LIMIT 0.5f

// 位置环
static float position_ref_deg = 90.0f;

#define POSITION_SPEED_LIMIT 150.0f

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
static void SVPWM(float u_alpha, float u_beta, float vbus);
static void Encoder_UpdateAngle(void);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);
static void CurrentPI_Update(float id_ref, float iq_ref, float id, float iq, float u_limit, float *ud_out, float *uq_out);
static float SpeedPI_Update(float rpm_ref, float rpm, float iq_limit);
static float PositionP_Update(float target_deg, float current_deg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float ADC_ToCurrent(uint32_t raw, uint32_t offset)
{
    // ADC计数差转成电压差
    float delta_v = ((float)raw - (float)offset) * ADC_VREF / ADC_FULL_SCALE;

    // 电压差转成电流
    return delta_v / (CURRENT_GAIN * SHUNT_R) * -1; // 实验测得方向是反的
}


static void Encoder_CalibrateZero(void)
{
	uint32_t primask = __get_PRIMASK();	// 保存原来的中断状态
	__disable_irq();	// 暂时禁止中断，避免ADC回调同时更新编码器

	Encoder_UpdateAngle();				// 读取编码器
    encoder_align = encoder_position;	// 当前转子已对齐到定子参考磁场方向
    encoder_calibrated = 1;				// 校准完成

    // 恢复原来的中断状态
	__set_PRIMASK(primask);
}

static void CurrentPI_Reset(void)
{
    pi_d.integral = 0.0f;
    pi_q.integral = 0.0f;
    pi_speed.integral = 0.0f;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // VOFA+ 监控串口中断优先级
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);
  HAL_NVIC_SetPriority(LPUART1_IRQn, 6, 0);

  foc_mode = FOC_MODE_ALIGN;	 // 确保启动时处于固定磁场对齐模式

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);	// ADC内部校准

  HAL_ADCEx_InjectedStart_IT(&hadc1);			// 启用ADC注入组中断，等待TIM1硬件触发

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);		// 启动TIM1 CH4内部比较

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	// TIM2编码器方波计数使能

  __HAL_TIM_SET_COUNTER(&htim2, 0);

  HAL_Delay(1000);	// 等待ADC注入组采集稳定的偏置电流

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);		// TIM1中断使能

  // 固定磁场对齐，等待转子稳定
  HAL_Delay(1000);

  // 记录当前对齐位置
  Encoder_CalibrateZero();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // 初始q轴电压设为0
//  Uq_test = 0.6f;
//  // 切换控制模式
//  foc_mode = FOC_MODE_ENCODER_TEST;

//  HAL_Delay(2000);

  pi_d.kp = 0.1508f;
  pi_d.ki = 314.2f;

  pi_q.kp = 0.1508f;
  pi_q.ki = 314.2f;

  pi_speed.kp = 0.008f;
  pi_speed.ki = 0.08f;

  pi_position.kp = 2.0f;

  CurrentPI_Reset();

  current_ud_cmd = 0.0f;
  current_uq_cmd = 0.0f;

  Id_ref = 0.0f;
  Iq_ref = 0.0f;

  // 最后才允许进入电流闭环
//  foc_mode = FOC_MODE_CURRENT_LOOP;
//  foc_mode = FOC_MODE_SPEED_LOOP;
  foc_mode = FOC_MODE_POSITION_LOOP;

  // 开启 DWT 周期计数器
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  // 记录上次发送时间
  uint32_t last_vofa_cycle = DWT->CYCCNT;
  // 目标发送频率：4000Hz
  uint32_t vofa_period_cycles = SystemCoreClock / 4000U;

  uint32_t last_iq_cycle = DWT->CYCCNT;
  uint32_t iq_period_cycles = SystemCoreClock * 2U;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	uint32_t now_cycle = DWT->CYCCNT;

	if ((uint32_t)(now_cycle - last_vofa_cycle) >= vofa_period_cycles)
	{
	  last_vofa_cycle = now_cycle;

	  if (adc_offset_ready && encoder_calibrated)
	  {
		  float monitor_data[8];

		  monitor_data[0] = position_ref_deg;

		  monitor_data[1] = theta_mech_deg;

		  monitor_data[2] = motor_rpm_filtered;

		  monitor_data[3] = speed_ref_rpm;

		  monitor_data[4] = Iq_ref;

		  // Park变换
		  monitor_data[5] = adc_window_valid;

		  monitor_data[6] = Iq;

		  monitor_data[7] = current_uq_cmd;

		  VOFA_Send8(monitor_data);
	  }
	}


	if ((uint32_t)(now_cycle - last_iq_cycle) >= iq_period_cycles)
	{
	    last_iq_cycle += iq_period_cycles;

	    position_ref_deg += 70.0;
	    if (position_ref_deg > 360.0f)
	    {
	    	position_ref_deg -= 360.0f;
	    }

//	    float target = 10.0f;
//
//	    if (position_ref_deg == target)
//	    {
//	    	position_ref_deg = 350.0f;
//	    }
//	    else
//	    {
//	    	position_ref_deg = target;
//	    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 2000000;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2125;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 149;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static float PositionP_Update(float target_deg, float current_deg)
{
    float error = target_deg - current_deg;

    // 单圈最短路径
    if (error > 180.0f)
        error -= 360.0f;
    else if (error < -180.0f)
        error += 360.0f;

    float rpm = pi_position.kp * error;

    // 目标转速限幅
    if (rpm > POSITION_SPEED_LIMIT)
        rpm = POSITION_SPEED_LIMIT;
    else if (rpm < -POSITION_SPEED_LIMIT)
        rpm = -POSITION_SPEED_LIMIT;

    return rpm;
}

static float SpeedPI_Update(float rpm_ref, float rpm, float iq_limit)
{
    const float ts = 0.001f;

    float error = rpm_ref - rpm;

    float base = pi_speed.kp * error + pi_speed.ki * pi_speed.integral;

    float di = pi_speed.ki * error * ts;

    float candidate = base + di;

    // 条件积分抗饱和
    if (fabsf(candidate) <= iq_limit || candidate * di <= 0.0f)
    {
        pi_speed.integral += error * ts;
        base = candidate;
    }

    // 输出限幅
    if (base > iq_limit) base = iq_limit;
    if (base < -iq_limit) base = -iq_limit;

    return base;
}

static void CurrentPI_Update(float id_ref, float iq_ref, float id, float iq, float u_limit, float *ud_out, float *uq_out)
{
    const float ts = CURRENT_LOOP_TS;

    float ed = id_ref - id;
    float eq = iq_ref - iq;

    // 当前比例输出和积分贡献
    float ud_base = pi_d.kp * ed + pi_d.ki * pi_d.integral;
    float uq_base = pi_q.kp * eq + pi_q.ki * pi_q.integral;

    // 本次积分对输出电压的增量
    float dud_i = pi_d.ki * ed * ts;
    float duq_i = pi_q.ki * eq * ts;

    // 候选电压
    float ud_candidate = ud_base + dud_i;
    float uq_candidate = uq_base + duq_i;

    float limit2 = u_limit * u_limit;

    float candidate2 = ud_candidate * ud_candidate + uq_candidate * uq_candidate;

    // 积分增量与候选电压的点积
    float outward = ud_candidate * dud_i + uq_candidate * duq_i;

    // 饱和时，拒绝继续向外推动电压矢量的积分
    if (candidate2 <= limit2 || outward <= 0.0f)
    {
        pi_d.integral += ed * ts;
        pi_q.integral += eq * ts;

        ud_base = ud_candidate;
        uq_base = uq_candidate;
    }

    // 对最终电压矢量进行同比例限幅
    float mag2 = ud_base * ud_base + uq_base * uq_base;

    if (mag2 > limit2)
    {
        float scale = u_limit / sqrtf(mag2);
        ud_base *= scale;
        uq_base *= scale;
    }

    *ud_out = ud_base;
    *uq_out = uq_base;
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
    	adc_injected_raw[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);	// U
    	adc_injected_raw[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);	// W

		adc_injected_count++;

		// 2. 零电流偏置校准
		if (!adc_offset_ready)
		{
			for (uint32_t i = 0; i < 2; i++)
			{
				adc_offset_sum[i] += adc_injected_raw[i];
			}

			adc_offset_count++;

			if (adc_offset_count >= ADC_OFFSET_SAMPLES)
			{
				for (uint32_t i = 0; i < 2; i++)
				{
					adc_offset[i] = adc_offset_sum[i] / ADC_OFFSET_SAMPLES;
				}

				adc_offset_ready = 1;
			}

			return;
		}

		current_phase[0] = ADC_ToCurrent(adc_injected_raw[0], adc_offset[0]);
		current_phase[2] = ADC_ToCurrent(adc_injected_raw[1], adc_offset[1]);
		current_phase[1] = - current_phase[0] - current_phase[2];

		// Clarke变换：三相静止坐标系 → 两相静止坐标系
		Ialpha = current_phase[0];
		Ibeta = (current_phase[0] + 2.0f * current_phase[1]) * 0.577350269f;

		// 没有完成编码器校准之前，不计算转子坐标系电流
		if (!encoder_calibrated)
		    return;

		// 更新当前编码器角度
		Encoder_UpdateAngle();

		// Park变换：αβ静止坐标系 → dq旋转坐标系
		float angle = theta_elec_calibrated;  // 暂时使用开环电角度 -> 使用校准后的电角度

		float sin_t = sinf(angle);
		float cos_t = cosf(angle);

		float id = Ialpha * cos_t + Ibeta * sin_t;
		float iq = -Ialpha * sin_t + Ibeta * cos_t;

		Id = id;
		Iq = iq;

		float iq_error = Iq_ref - Iq;
		iq_error_avg += 0.001f * (iq_error - iq_error_avg); // 一阶低通，滤除瞬时电流采样噪声

		if (foc_mode == FOC_MODE_CURRENT_LOOP || foc_mode == FOC_MODE_SPEED_LOOP || foc_mode == FOC_MODE_POSITION_LOOP)
		{
			float ud_new, uq_new;

			CurrentPI_Update(Id_ref, Iq_ref, Id, Iq, current_voltage_limit, &ud_new, &uq_new);

			current_ud_cmd = ud_new;
			current_uq_cmd = uq_new;
		}
    }
}

static void Encoder_UpdateAngle(void)
{
    static uint32_t last_cnt = 0;

    uint32_t now_cnt = TIM2->CNT;

    // 计算计数增量，处理32位计数器回绕
    int32_t delta = (int32_t)(now_cnt - last_cnt);

    last_cnt = now_cnt;

    // 软件位置限制在单圈范围 [0, 9999]
    int32_t pos = encoder_position + delta;
    pos %= ENCODER_CPR;

    if (pos < 0)
        pos += ENCODER_CPR;

    encoder_position = pos;

    // 转换成机械角度
    theta_mech_deg = pos * (360.0f / ENCODER_CPR);
//    theta_mech_rad = pos * (2.0f * PI / ENCODER_CPR);

    // 零点校准
    if (encoder_calibrated)
    {
        int32_t delta_pos = encoder_position - encoder_align;

        float angle = delta_pos * (2.0f * PI * MOTOR_POLE_PAIRS / ENCODER_CPR);

        theta_elec_calibrated = fmodf(angle, 2.0f * PI);

        if (theta_elec_calibrated < 0.0f)
        {
            theta_elec_calibrated += 2.0f * PI;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
    	float angle;
		float ud;
		float uq;

		if (foc_mode == FOC_MODE_ALIGN)
		{
			// 阶段1：固定磁场对齐
			angle = 0.0f;
			ud = 2.0f;   // 使用此前验证过的较低对齐电压
			uq = 0.0f;
		}
		else if (foc_mode == FOC_MODE_ENCODER_TEST)
		{
			// 阶段2：编码器反馈角度
			if (!encoder_calibrated)
				return;

			angle = theta_elec_calibrated;

			ud = 0.0f;
			uq = Uq_test;
		}
		else if (foc_mode == FOC_MODE_CURRENT_LOOP || foc_mode == FOC_MODE_SPEED_LOOP || foc_mode == FOC_MODE_POSITION_LOOP)
		{
		    angle = theta_elec_calibrated;

		    ud = current_ud_cmd;
		    uq = current_uq_cmd;
		}

		float sin_t = sinf(angle);
		float cos_t = cosf(angle);

		// 逆Park变换
		float Ualpha = ud * cos_t - uq * sin_t;
		float Ubeta  = ud * sin_t + uq * cos_t;

		SVPWM(Ualpha, Ubeta, Vbus);

		static uint8_t speed_divider = 0;
		static uint32_t speed_last_cnt = 0;

		// 速度环：1KHz
		if (++speed_divider >= 20)
		{
			speed_divider = 0;

			uint32_t speed_now_cnt = TIM2->CNT;
			int32_t delta = (int32_t)(speed_now_cnt - speed_last_cnt);

			speed_last_cnt = speed_now_cnt;

			// 1ms测速窗口
			motor_rpm_raw = delta * 6.0f;

			// 一阶低通滤波
			motor_rpm_filtered += 0.2f * (motor_rpm_raw - motor_rpm_filtered);

			static uint8_t position_divider = 0;

			// 位置环：200Hz
			if (foc_mode == FOC_MODE_POSITION_LOOP)
			{
			    if (++position_divider >= 5)
			    {
			        position_divider = 0;

			        speed_ref_rpm = PositionP_Update(position_ref_deg, theta_mech_deg);
			    }
			}

			if (foc_mode == FOC_MODE_SPEED_LOOP || foc_mode == FOC_MODE_POSITION_LOOP)
			{
				Iq_ref = SpeedPI_Update(speed_ref_rpm, motor_rpm_filtered, 4.0f);
			}
		}
    }
}

static void SVPWM(float u_alpha, float u_beta, float vbus)
{
    /* 1. αβ → 三相参考电压 */
    float ua = u_alpha;
    float ub = -0.5f * u_alpha + 0.8660254f * u_beta;
    float uc = -0.5f * u_alpha - 0.8660254f * u_beta;

    /* 2. 找三相最大值和最小值 */
    float u_max = ua;
    float u_min = ua;

    if (ub > u_max) u_max = ub;
    if (uc > u_max) u_max = uc;

    if (ub < u_min) u_min = ub;
    if (uc < u_min) u_min = uc;

    /* 3. 注入公共偏置 */
    float u_offset = -0.5f * (u_max + u_min);

    ua += u_offset;
    ub += u_offset;
    uc += u_offset;

    /* 4. 电压 → 占空比 */
    float duty_a = 0.5f + ua / vbus;
    float duty_b = 0.5f + ub / vbus;
    float duty_c = 0.5f + uc / vbus;

    /* 5. 防止异常参数导致占空比越界 */
    if (duty_a > 1.0f) duty_a = 1.0f;
    if (duty_a < 0.0f) duty_a = 0.0f;

    if (duty_b > 1.0f) duty_b = 1.0f;
    if (duty_b < 0.0f) duty_b = 0.0f;

    if (duty_c > 1.0f) duty_c = 1.0f;
    if (duty_c < 0.0f) duty_c = 0.0f;

    /* 6. 占空比 → TIM1 CCR */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

    uint32_t ccr1 = (uint32_t)(duty_a * (arr + 1.0f));
    uint32_t ccr2 = (uint32_t)(duty_b * (arr + 1.0f));
    uint32_t ccr3 = (uint32_t)(duty_c * (arr + 1.0f));

    // 避免比较值超过ARR
    if (ccr1 > arr) ccr1 = arr;
    if (ccr2 > arr) ccr2 = arr;
    if (ccr3 > arr) ccr3 = arr;

    /* 8. 双电阻采样：只考虑U、W两相 */

    uint32_t ccr_max = (ccr1 > ccr3) ? ccr1 : ccr3;

    adc_window_ticks = arr - ccr_max;
    /* 9. 判断窗口 */
    if (adc_window_ticks >= ADC_WAIT_TICKS + ADC_SEQUENCE_TICKS)
    {
    	adc_window_valid = 1;
    }
    else
    {
    	adc_window_valid = 0;
    	TIM1->EGR = TIM_EGR_BG;
    	return;
    }

    /* 10. 计算采样位置 */
    uint32_t ccr4 = ccr_max + ADC_WAIT_TICKS;

    /* 11. 更新CCR */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr4);
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
#ifdef USE_FULL_ASSERT
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
