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
  *65535
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

/* Add near your other #defines (only once) */
#define TEMP_BIAS_RTOP_OHMS   47000.0f   // R1
#define TEMP_BIAS_RBOT_OHMS   47000.0f   // R2
#define TEMP_BIAS_VMID        (ADC_VREF_V * (TEMP_BIAS_RBOT_OHMS / (TEMP_BIAS_RTOP_OHMS + TEMP_BIAS_RBOT_OHMS)))
/* For 47k/47k this is ~1.65 V */


#define TEMP_REF_RTOP_OHMS      27000.0f        // R4
#define TEMP_REF_RBOT_OHMS      10000.0f        // R3
#define TEMP_AD623_VREF_V       (ADC_VREF_V * (TEMP_REF_RBOT_OHMS / (TEMP_REF_RTOP_OHMS + TEMP_REF_RBOT_OHMS)))

/* Thermistor parameters (ADJUST to your NTC part) */
#define NTC_R0_OHMS             47000.0f       // e.g., 47k at 25°C
#define NTC_BETA_K              3950.0f         // e.g., B=3950
#define NTC_T0_K                298.15f         // 25°C in Kelvin

#define ADC_SAMPLES_AVG         8

#define SUPPLY_VOLTAGE_V   24.0f

#define CONTROL_DT_S           0.10f                // 100 ms bucle PI (TIM2)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t  uart2_rx_byte;           // byte recived in Interrupt
volatile int16_t  last_sent_pct   = -1;  // last %

volatile uint8_t  cmd_idx = 0;             // index
volatile char     cmd_buf[CMD_BUF_LEN];    // buffer

volatile uint8_t pwm_test_mode   = 0;

/* === New runtime state === */
volatile uint8_t stream_on   = 0;   // 0=OFF, 1=ON
volatile uint8_t duty_pct    = 0;   // Current duty in % (0..100)

/* === PI control runtime === */
volatile uint8_t  pi_mode      = 0;                 // 0=OFF, 1=ON
volatile float    kp           = 12.528630f;        // default
volatile float    ki           = 0.2527029f;        // default
volatile float    temp_sp_C    = 25.0f;             // setpoint °C
static   float    pi_int       = 0.0f;              // integrador (en %)



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void TIM1_CH1_SetDutyPercent(float duty_pct);
void TIM2_SetPeriod_ms(uint32_t ms);
uint32_t TIM2_GetCounterTickHz(void);
void Command_Interpreter(const char *buf);
void uart2_printf(const char *fmt, ...);

static uint16_t ADC1_Read12_Channel(uint32_t channel);
static float    ADC1_Read_Average_V(uint32_t channel);
void TIM1_CH1_SetDutyPercent(float duty_pct);
float           Mesure_current_A(void);
float           Mesure_Temperature_C(void);

static int parse_sp_xdddd(const char *p, float *out_spC);
static double parse_float_from(const char *p);

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* Start 20 kHz PWM on TIM1 CH1 -> PA8 (D7) */

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
  }
  __HAL_TIM_MOE_ENABLE(&htim1);
  TIM1_CH1_SetDutyPercent(0.0f);

  HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart2_rx_byte, 1);
  HAL_TIM_Base_Start_IT(&htim2);

  TIM2_SetPeriod_ms(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      /* ===== Streaming :S =====
       * Emits: Corriente;Temperatura;DuttyCycle;Potencia\n
       * Potencia = 24V * (duty%/100) * Corriente
       */
      if (stream_on) {
          float i_A = Mesure_current_A();
          float t_C = Mesure_Temperature_C();
          float p_W = SUPPLY_VOLTAGE_V * ((float)duty_pct / 100.0f) * i_A;

          /* Format EXACTLY: Corriente;Temperatura;DuttyCycle;Potencia\n */
          uart2_printf("%.4f;%.2f;%u;%.2f\r\n",
                       i_A, t_C, (unsigned)duty_pct, p_W);

          HAL_Delay(100);   // ~10 Hz stream
          continue;
      }

      /* ===== Test mode :K (prints only when streaming is OFF) ===== */
      if (pwm_test_mode) {
          static uint8_t header_printed_k = 0;
          if (!header_printed_k) {
              uart2_printf("\r\nVraw_I [V]\tI [A]\tVraw_T [V]\tT [°C]\tDuty [%]\r\n");
              header_printed_k = 1;
          }

          float vraw_a0 = ADC1_Read_Average_V(ISENSE_ADC_CHANNEL);
          float vraw_a1 = ADC1_Read_Average_V(TEMP_ADC_CHANNEL);
          float i_A     = Mesure_current_A();
          float t_C     = Mesure_Temperature_C();

          uart2_printf("%.4f [V]\t%.4f [A]\t%.4f [V]\t%.2f [°C]\t%u [%%]\r\n",
                       vraw_a0, i_A, vraw_a1, t_C, (unsigned)duty_pct);

          HAL_Delay(100);
          continue;
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
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Blink diagnostico

        /* === Lazo PI de temperatura === */
        if (pi_mode && !pwm_test_mode) {
            float t_C   = Mesure_Temperature_C();
            float err   = temp_sp_C - t_C;

            /* Acción proporcional (en % directamente) */
            float u_p   = kp * err;

            /* Integración con anti-windup condicional */
            float du_i  = ki * err * CONTROL_DT_S;   // incremento del integrador
            float u_raw = u_p + pi_int;              // antes de saturar

            /* Saturación dura 0..100 % aplicada a la salida */
            float duty  = u_raw;
            if (duty > 100.0f) duty = 100.0f;
            if (duty < 0.0f)   duty = 0.0f;

            /* Anti-windup: solo integramos si no estamos empujando hacia la saturación */
            int sat_hi = (u_raw >= 100.0f) && (err > 0.0f);
            int sat_lo = (u_raw <=   0.0f) && (err < 0.0f);
            if (!(sat_hi || sat_lo)) {
                pi_int += du_i;
                /* Acotar el integrador a rango razonable de % */
                if (pi_int > 100.0f) pi_int = 100.0f;
                if (pi_int < -100.0f) pi_int = -100.0f;
            }

            TIM1_CH1_SetDutyPercent(duty);
            duty_pct = (uint8_t)(duty + 0.5f);
        }
    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uint8_t b = uart2_rx_byte;

        if (b == ':') {
            /* Start of a new command: drop ':' and reset the buffer */
            cmd_idx = 0;

        } else if (b == '\r' || b == '\n') {
            /* ENTER pressed: close special modes or dispatch buffered command */

            if (pwm_test_mode) {
                /* Exit K-mode on ENTER (return to 0% for safety) */
                pwm_test_mode = 0;
                duty_pct = 0;
                TIM1_CH1_SetDutyPercent(0.0f);
                htim1.Instance->EGR = TIM_EGR_UG;  /* force immediate update */
                cmd_idx = 0;
                uart2_printf("PWM test END -> duty=0%%\r\n");

            } else {
                /* No special mode: if we have a buffered command, run it */
                if (cmd_idx > 0) {
                    if (cmd_idx >= CMD_BUF_LEN) cmd_idx = CMD_BUF_LEN - 1;
                    cmd_buf[cmd_idx] = '\0';
                    Command_Interpreter((const char*)cmd_buf);
                    cmd_idx = 0;
                }
            }

        } else if ((b == '+' || b == '-') && pwm_test_mode) {
            /* Jog duty in K-mode by ±10% steps, clamped to 0..100 */
            if (b == '+') {
                duty_pct = (duty_pct >= 90) ? 100 : (uint8_t)(duty_pct + 10);
            } else {
                duty_pct = (duty_pct <= 10) ?   0 : (uint8_t)(duty_pct - 10);
            }
            TIM1_CH1_SetDutyPercent((float)duty_pct);
            htim1.Instance->EGR = TIM_EGR_UG;  /* make the new CCR1 effective now */
        } else {
            /* Accumulate printable characters for a command payload */
            if (cmd_idx < CMD_BUF_LEN - 1) {
                if (b >= 32 && b <= 126) {
                    cmd_buf[cmd_idx++] = (char)b;
                }
            }
        }

        /* Re-arm reception of the next byte */
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



/* Measure load current and return it in amps (A).
 * Topology: AD623 with gain = ISENSE_AD623_GAIN, REF = ISENSE_AD623_VREF_V,
 *           shunt = ISENSE_RSHUNT_OHMS, ADC channel = ISENSE_ADC_CHANNEL.
 *
 * Flow:
 *   1) Read AD623 output voltage at ADC pin.
 *   2) Remove AD623 REF offset.
 *   3) Divide by AD623 gain to recover shunt differential voltage.
 *   4) I = Vsense / Rshunt  -> convert to A.
 */
float Mesure_current_A(void)
{
    /* 1) Read averaged ADC voltage at A0 (AD623 output) */
    float v_adc = ADC1_Read_Average_V(ISENSE_ADC_CHANNEL);

    /* 2) Remove AD623 REF offset (0 V in your current path) */
    float v_out_rel = v_adc - ISENSE_AD623_VREF_V;

    /* 3) Recover shunt voltage */
    float v_sense = v_out_rel / ISENSE_AD623_GAIN;

    /* 4) Convert to current and return in A */
    float i_a = v_sense / ISENSE_RSHUNT_OHMS;   /* Amps */
    return i_a;
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
    /* 1) Read AD623 output (what the ADC sees) */
    float v_adc  = ADC1_Read_Average_V(TEMP_ADC_CHANNEL);

    /* 2) INA de-embedding: differential at inputs */
    float v_diff = (v_adc - TEMP_AD623_VREF_V) / TEMP_AD623_GAIN;

    /* 3) Rebuild the actual divider node: Vnode = VMID + v_diff
          (AD623 measures node - VMID; if inputs were swapped, the sign flips) */
    float v_node = TEMP_BIAS_VMID - v_diff;

    /* If the result looks out-of-range, try opposite polarity (inverted wiring). */
    if (v_node <= 0.0f || v_node >= ADC_VREF_V) {
        v_node = TEMP_BIAS_VMID - v_diff;
    }

    /* Keep within (0, Vref) to avoid singularities */
    const float EPS = 1e-6f;
    if (v_node >= (ADC_VREF_V - EPS)) v_node = ADC_VREF_V - EPS;
    if (v_node <= EPS)                 v_node = EPS;

    /* 4) Thermistor resistance from divider:
          Vnode = Vref * (Rntc / (Rref + Rntc))  =>  Rntc = Rref * Vnode / (Vref - Vnode) */
    float r_ntc = NTC_RREF_OHMS * (v_node / (ADC_VREF_V - v_node));

    /* 5) Steinhart–Hart with EPCOS 47k, R/T #4003 anchors (computed once) */
    const float R25      = 47000.0f;     // 47k @ 25°C
    const float ratio0C  = 3.5243f;      // R(0°C)/R25
    const float ratio85C = 0.10053f;     // R(85°C)/R25

    const float R1 = R25 * ratio0C;      // at 0°C
    const float R2 = R25;                // at 25°C
    const float R3 = R25 * ratio85C;     // at 85°C

    const float T1 = 273.15f;            // 0°C  in K
    const float T2 = 298.15f;            // 25°C in K
    const float T3 = 358.15f;            // 85°C in K

    static int   sh_init = 0;
    static float sh_A, sh_B, sh_C;

    if (!sh_init) {
        float L1 = logf(R1), L2 = logf(R2), L3 = logf(R3);
        float Y1 = 1.0f / T1, Y2 = 1.0f / T2, Y3 = 1.0f / T3;

        float gamma2 = (Y2 - Y1) / (L2 - L1);
        float gamma3 = (Y3 - Y1) / (L3 - L1);
        sh_C = (gamma3 - gamma2) / (L3 - L2) / (L1 + L2 + L3);
        sh_B = gamma2 - sh_C * (L2*L2 + L1*L2 + L1*L1);
        sh_A = Y1 - sh_B*L1 - sh_C*(L1*L1*L1);

        sh_init = 1;
    }

    float L  = logf(r_ntc);
    float Y  = sh_A + sh_B*L + sh_C*(L*L*L);   /* 1/T [1/K] */
    if (Y <= 0.0f) Y = EPS;
    float T_K = 1.0f / Y;
    float T_C = T_K - 273.15f;

    return T_C;
}

/* Set TIM1 CH1 duty in percent (0..100)  (HEATING RESISTENCE DUTTY-CYCLE)*/
// Apply duty in % to TIM1 CH1 and force the update immediately
void TIM1_CH1_SetDutyPercent(float duty_pct)
{
    if (duty_pct < 0.0f)   duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);              // e.g. 4999
    uint32_t ccr = (uint32_t)((duty_pct * (float)(arr + 1U)) * 0.01f);
    if (ccr > arr) ccr = arr;                                     // 100% -> arr

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);            // write CCR1

    // Force an update event so the new CCR is taken immediately (no shadow delay)
    htim1.Instance->EGR = TIM_EGR_UG;                             // generate UG
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

/* ---------- Helpers for command parsing (keep near the interpreter) ---------- */

/* Parse a floating-point number after optional spaces. Example: "  12.5" -> 12.5 */
static double parse_float_from(const char *p) {
    while (*p == ' ') p++;
    return strtod(p, NULL);
}

/* Parse temperature setpoint in the :Xdddd format
   - Exactly 4 digits: first 3 are integer part, last 1 is tenths (e.g., X1234 -> 123.4°C)
   - Returns 0 on success and writes to *out_spC; -1 on malformed input. */
static int parse_sp_xdddd(const char *p, float *out_spC) {
    while (*p == ' ') p++;
    const char *q = p;

    /* Require exactly 4 digits */
    for (int i = 0; i < 4; ++i) {
        if (q[i] < '0' || q[i] > '9') return -1;
    }
    if (q[4] != '\0' && q[4] != ' ' && q[4] != '\r' && q[4] != '\n') {
        /* Extra junk after 4 digits: reject */
        return -1;
    }

    unsigned v = (unsigned)strtoul(p, NULL, 10);  /* e.g., 1234 */
    *out_spC   = (float)v / 10.0f;                /* 123.4 */
    return 0;
}

/* ------------------------------ Command Interpreter ------------------------------ */
/* buf contains the command WITHOUT the leading ':' and without CR/LF.
   Examples:
     ":C" or ":C1" -> enable PI mode
     ":C0"         -> disable PI mode
     ":X1234"      -> setpoint = 123.4 °C
     ":KP12.5"     -> Kp = 12.5
     ":KI0.25"     -> Ki = 0.25
     ":K"          -> manual PWM jog mode (K-mode), disables PI
     ":D50"        -> duty = 50%, disables PI
     ":S" / ":S0"  -> start/stop streaming
     ":A"          -> safe stop (everything off)
*/
void Command_Interpreter(const char *buf)
{
    if (!buf || !buf[0]) return;

    /* Normalize main command letter to uppercase */
    char cmd = (char)toupper((unsigned char)buf[0]);

    /* Point to the first character after the main command letter */
    const char *p = buf + 1;
    while (*p == ' ') p++;

    /* Keep val for commands that still want an integer after spaces (e.g., :Dnn) */
    uint32_t val = strtoul(p, NULL, 10);

    switch (cmd) {

    /* ======== PI mode control ========
       :C or :C1 -> enable PI (resets integrator, sets duty=0 for safety)
       :C0       -> disable PI (sets duty=0)
    */
    case 'C':
    {
        int enable = 1;
        if (*p == '0') enable = 0;

        if (enable) {
            pwm_test_mode = 0;           /* Leave manual jog if active */
            pi_mode       = 1;
            pi_int        = 0.0f;        /* Reset integral term */
            TIM1_CH1_SetDutyPercent(0.0f);
            duty_pct = 0;
            uart2_printf("PI ON (Kp=%.6f, Ki=%.6f, SP=%.1f C)\r\n", kp, ki, temp_sp_C);
        } else {
            pi_mode = 0;
            TIM1_CH1_SetDutyPercent(0.0f);
            duty_pct = 0;
            uart2_printf("PI OFF\r\n");
        }
    }
    break;

    /* ======== Temperature setpoint ========
       :Xdddd -> 4 digits, last is tenths (e.g., X0850 -> 85.0 C)
       Rejects formats that are not exactly 4 digits.
    */
    case 'X':
    {
        float spC;
        if (parse_sp_xdddd(p, &spC) == 0) {
            temp_sp_C = spC;
            uart2_printf("SP=%.1f C\r\n", temp_sp_C);
        } else {
            uart2_printf("ERR setpoint format (use :Xdddd, e.g., :X1234 -> 123.4 C)\r\n");
        }
    }
    break;

    /* ======== Gains and K-mode ========
       :KP<float> -> set Kp
       :KI<float> -> set Ki
       :K         -> enter manual PWM jog mode (disables PI)
    */
    case 'K':
    {
        /* If there's a second letter, it should be P or I for gain edits */
        char c1 = (char)toupper((unsigned char)p[0]);

        if (c1 == 'P') {
            double v = parse_float_from(p + 1);
            kp = (float)v;
            uart2_printf("Kp=%.6f\r\n", kp);
        } else if (c1 == 'I') {
            double v = parse_float_from(p + 1);
            ki = (float)v;
            uart2_printf("Ki=%.6f\r\n", ki);
        } else if (p[0] == '\0') {
            /* Bare :K -> enter manual jog mode */
            pwm_test_mode = 1;
            pi_mode       = 0;            /* PI off while in manual */
            duty_pct      = 0;
            TIM1_CH1_SetDutyPercent(0.0f);
            uart2_printf("PWM test START. Use '+' / '-' (±10%%). ENTER to stop.\r\n");
        } else {
            uart2_printf("ERR cmd 'K%.*s'\r\n", 4, p);
        }
    }
    break;

    /* ======== Direct duty command ========
       :Dnn -> duty in % (0..100). Disables PI.
    */
    case 'D':
    {
        if (val > 100U) val = 100U;
        pi_mode  = 0;                     /* leave PI if forcing duty */
        duty_pct = (uint8_t)val;
        TIM1_CH1_SetDutyPercent((float)duty_pct);
        uart2_printf("Duty=%lu %% (PI OFF)\r\n", (unsigned long)val);
    }
    break;

    /* ======== Streaming ========
       :S  -> start streaming
       :S0 -> stop streaming
    */
    case 'S':
    {
        if (*p == '0') {
            stream_on = 0;
            uart2_printf("Stream OFF\r\n");
        } else {
            stream_on = 1;
            uart2_printf("Stream ON\r\n");
        }
    }
    break;

    /* ======== Safe stop ========
       :A -> immediate safe stop (stream OFF, K-mode OFF, PI OFF, duty=0)
    */
    case 'A':
    {
        stream_on     = 0;
        pwm_test_mode = 0;
        pi_mode       = 0;
        duty_pct      = 0;
        pi_int        = 0.0f;
        TIM1_CH1_SetDutyPercent(0.0f);
        uart2_printf("SAFE STOP\r\n");
    }
    break;

    /* ======== Unknown command ======== */
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
