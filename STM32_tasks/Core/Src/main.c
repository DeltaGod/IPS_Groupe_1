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

//Some change
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ctype.h>
#include <stdarg.h>
#include <stdlib.h>  // strtoul
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_BUF_LEN 64

#define ADC_VREF_V              3.3f
#define ADC_MAX_COUNTS          4095.0f

#define ISENSE_ADC_CHANNEL      ADC_CHANNEL_0   /* A0 = PA0 on Nucleo F411RE */
#define TEMP_ADC_CHANNEL        ADC_CHANNEL_1   /* A1 = PA1 on Nucleo F411RE */

#define ISENSE_RSHUNT_OHMS      0.05f           // 50 mΩ
#define ISENSE_AD623_RG_OHMS    4300.0f         // 4.3k
#define ISENSE_AD623_GAIN       (1.0f + 49400.0f / ISENSE_AD623_RG_OHMS)
#define ISENSE_AD623_VREF_V     0.0f            // REF = GND


#define NTC_RREF_OHMS           47000.0f        // 47k  ohm
#define TEMP_AD623_RG_OHMS      100000.0f       // 100K ohm
#define TEMP_AD623_GAIN         (1.0f + 49400.0f / TEMP_AD623_RG_OHMS)

#define TEMP_REF_RTOP_OHMS      27000.0f        // R4
#define TEMP_REF_RBOT_OHMS      10000.0f        // R3
#define TEMP_AD623_VREF_V       (ADC_VREF_V * (TEMP_REF_RBOT_OHMS / (TEMP_REF_RTOP_OHMS + TEMP_REF_RBOT_OHMS)))

/* Thermistor parameters (ADJUST to your NTC part) */
#define NTC_R0_OHMS             47000.0f       // e.g., 47k at 25°C
#define NTC_BETA_K              3950.0f         // e.g., B=3950
#define NTC_T0_K                298.15f         // 25°C in Kelvin

#define ADC_SAMPLES_AVG         8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t  uart2_rx_byte;           // byte recived in Interrupt
volatile uint8_t  adc_mode_analog = 0;   // 0=consola, 1=analógico
volatile int16_t  last_sent_pct   = -1;  // last %

volatile uint8_t  cmd_idx = 0;             // index
volatile char     cmd_buf[CMD_BUF_LEN];    // buffer


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void TIM2_SetPeriod_ms(uint32_t ms);
uint32_t TIM2_GetCounterTickHz(void);
void Command_Interpreter(const char *buf);
void uart2_printf(const char *fmt, ...);

static uint16_t ADC1_Read12_Channel(uint32_t channel);
static float    ADC1_Read_Average_V(uint32_t channel);
float           Mesure_current_A(void);
float           Mesure_Temperature_C(void);

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart2_rx_byte, 1);
  uart2_printf("UART OK. Cmds: :P<num_ms>, :A0/:A1 (console/analog), ENTER sale de analog.\r\n");
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (adc_mode_analog) {
          /* Measure current (mA) and temperature (°C) */
          float i_mA = Mesure_current_A();
          float t_C  = Mesure_Temperature_C();

          /* Send one line */
          uart2_printf("%.1f\t%.2f\r\n", i_mA, t_C);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 49999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Toggle LED L2 (PA5)
    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint8_t b = uart2_rx_byte;

        if (b == ':') {
            cmd_idx = 0; // nuevo comando (no guardamos ':')
        } else if (b == '\r' || b == '\n') {
            // Si estoy en modo analógico, ENTER sale a modo consola
            if (adc_mode_analog) {
                adc_mode_analog = 0;
                last_sent_pct = -1;
                cmd_idx = 0;
                uart2_printf("Analog mode OFF -> console duty\r\n");
            } else {
                if (cmd_idx > 0) {
                    // Cerrar string y llamar al intérprete
                    uint8_t i = cmd_idx;
                    if (i >= CMD_BUF_LEN) i = CMD_BUF_LEN - 1;
                    cmd_buf[i] = '\0';
                    Command_Interpreter((const char*)cmd_buf);
                    cmd_idx = 0;
                }
            }
        } else {
            // Acumular carácter si hay espacio
            if (cmd_idx < CMD_BUF_LEN - 1) {
                cmd_buf[cmd_idx++] = (char)b;
            }
        }

        // Rearmar la recepción de 1 byte
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart2_rx_byte, 1);
    }
}

static uint16_t ADC1_Read12_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel      = channel;

/* F4 HAL uses numeric rank (1..16). Newer families define ADC_REGULAR_RANK_1. */
#if defined(ADC_REGULAR_RANK_1)
    sConfig.Rank         = ADC_REGULAR_RANK_1;
#else
    sConfig.Rank         = 1;  // STM32F4
#endif

    /* Use a reasonable sampling time to accommodate source impedance */
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

#if defined(ADC_SINGLE_ENDED)
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;   // not present on F4
#endif

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }

    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc1);   // 0..4095
    HAL_ADC_Stop(&hadc1);
    return val;
}

/* Read N samples on a channel and return the average in volts. */
static float ADC1_Read_Average_V(uint32_t channel)
{
    uint32_t acc = 0;
    for (int i = 0; i < ADC_SAMPLES_AVG; ++i) {
        acc += ADC1_Read12_Channel(channel);
    }
    /* Avoid integer division loss of precision */
    float counts = (float)acc / (float)ADC_SAMPLES_AVG;
    return (counts / ADC_MAX_COUNTS) * ADC_VREF_V;
}


/* Measure load current and return it in milliamps (mA).
 * Topology: AD623 with gain = ISENSE_AD623_GAIN, REF = ISENSE_AD623_VREF_V,
 *           shunt = ISENSE_RSHUNT_OHMS, ADC channel = ISENSE_ADC_CHANNEL.
 *
 * Flow:
 *   1) Read AD623 output voltage at ADC pin.
 *   2) Remove AD623 REF offset.
 *   3) Divide by AD623 gain to recover shunt differential voltage.
 *   4) I = Vsense / Rshunt  -> convert to mA.
 */
float Mesure_current_A(void)
{
    /* 1) Read averaged ADC voltage at A0 (AD623 output) */
    float v_adc = ADC1_Read_Average_V(ISENSE_ADC_CHANNEL);

    /* 2) Remove AD623 REF offset (0 V in your current path) */
    float v_out_rel = v_adc - ISENSE_AD623_VREF_V;

    /* 3) Recover shunt voltage */
    float v_sense = v_out_rel / ISENSE_AD623_GAIN;

    /* 4) Convert to current and return in mA */
    float i_a = v_sense / ISENSE_RSHUNT_OHMS;   /* Amps */
    return i_a * 1000.0f;                       /* mA */
}


/* Measure temperature using Steinhart–Hart (3rd-order) fitted to EPCOS 47k, R/T #4003.
 * The AD623 REF and gain are removed first to recover the divider node voltage.
 * Datasheet anchor points (R/R25 for R/T 4003):
 *   T1 = 0°C,  R1 = R25 * 3.5243
 *   T2 = 25°C, R2 = R25
 *   T3 = 85°C, R3 = R25 * 0.10053
 * This yields SH: 1/T = A + B*ln(R) + C*ln(R)^3   (T in Kelvin).
 */
float Mesure_Temperature_C(void)
{
    /* ---------- 1) Read AD623 output (A1) and recover divider node voltage ---------- */
    float v_adc  = ADC1_Read_Average_V(TEMP_ADC_CHANNEL);               // AD623 output @ ADC
    float v_node = (v_adc - TEMP_AD623_VREF_V) / TEMP_AD623_GAIN;       // raw divider node

    /* Clamp to (0, Vref) to avoid division-by-zero and log() issues */
    if (v_node >= (ADC_VREF_V - 1e-6f)) v_node = ADC_VREF_V - 1e-6f;
    if (v_node <= 1e-6f)                 v_node = 1e-6f;

    /* ---------- 2) Compute thermistor resistance from the divider ---------- */
    /* Vnode = Vref * (Rntc / (Rref + Rntc))  =>  Rntc = Rref * Vnode / (Vref - Vnode) */
    float r_ntc = NTC_RREF_OHMS * (v_node / (ADC_VREF_V - v_node));

    /* ---------- 3) Steinhart–Hart using 3 points from the datasheet (computed once) ---------- */
    /* Anchors (datasheet R/T 4003) */
    const float R25      = 47000.0f;        // 47k at 25°C
    const float ratio0C  = 3.5243f;         // R(0°C)/R25
    const float ratio85C = 0.10053f;        // R(85°C)/R25

    const float R1 = R25 * ratio0C;         // at 0°C
    const float R2 = R25;                   // at 25°C
    const float R3 = R25 * ratio85C;        // at 85°C

    const float T1 = 273.15f;               // 0°C  in Kelvin
    const float T2 = 298.15f;               // 25°C in Kelvin
    const float T3 = 358.15f;               // 85°C in Kelvin

    /* Precompute SH coefficients once */
    static int   sh_init = 0;
    static float sh_A, sh_B, sh_C;

    if (!sh_init) {
        /* Solve for A, B, C in: 1/T = A + B*L + C*L^3, with L = ln(R) */
        float L1 = logf(R1), L2 = logf(R2), L3 = logf(R3);
        float Y1 = 1.0f / T1, Y2 = 1.0f / T2, Y3 = 1.0f / T3;

        /* From AN206-style derivation */
        float gamma2 = (Y2 - Y1) / (L2 - L1);
        float gamma3 = (Y3 - Y1) / (L3 - L1);
        sh_C = (gamma3 - gamma2) / (L3 - L2) / (L1 + L2 + L3);
        sh_B = gamma2 - sh_C * (L2*L2 + L1*L2 + L1*L1);
        sh_A = Y1 - sh_B*L1 - sh_C*(L1*L1*L1);

        sh_init = 1;
    }

    /* ---------- 4) Compute temperature from measured R using SH ---------- */
    float L  = logf(r_ntc);
    float Y  = sh_A + sh_B*L + sh_C*(L*L*L);    // 1/T in 1/K
    if (Y <= 0.0f) Y = 1e-6f;                   // safety
    float T_K = 1.0f / Y;
    float T_C = T_K - 273.15f;

    return T_C;  /* °C */
}


void uart2_printf(const char *fmt, ...)
{
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n > sizeof(buf)) n = sizeof(buf);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, 100);
}


uint32_t TIM2_GetCounterTickHz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    /* Si el prescaler de APB1 != 1, los timers corren al doble */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        pclk1 *= 2U;
    }
    return pclk1 / (htim2.Init.Prescaler + 1U);
}

void TIM2_SetPeriod_ms(uint32_t ms)
{
    if (ms == 0) ms = 1;
    uint32_t tick_hz = TIM2_GetCounterTickHz();     // p.ej. 2 kHz con tu PSC=49999
    uint32_t arr = (tick_hz * ms) / 1000U;
    if (arr == 0) arr = 1;
    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr - 1U);     // ARR = N-1
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_ENABLE(&htim2);
}

// Intérprete de comandos (switch-case): buf = "L123" (sin ':')

void Command_Interpreter(const char *buf)
{
    if (!buf || !buf[0]) return;

    char cmd = (char)toupper((unsigned char)buf[0]);

    const char *p = buf + 1;
    while (*p == ' ') p++;
    uint32_t val = strtoul(p, NULL, 10);

    switch (cmd) {
    	case 'P':  // :P<num_ms>
    		TIM2_SetPeriod_ms(val);
        	uart2_printf("TIM2 period set to %lu ms\r\n", (unsigned long)val);
        	break;


        case 'A': // Modo: :A1 = analog ON, :A0 = analog OFF
            if (val == 1) {
                adc_mode_analog = 1;
                last_sent_pct = -1;
                uart2_printf("Analog mode ON (ENTER para salir)\r\n");
            } else {
                adc_mode_analog = 0;
                last_sent_pct = -1;
                uart2_printf("Analog mode OFF -> console duty\r\n");
            }
            break;


        default:
            uart2_printf("ERR cmd '%c'\r\n", cmd);
            break;
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
