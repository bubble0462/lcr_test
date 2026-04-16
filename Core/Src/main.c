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
#include "adc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>

#include "9834.h"
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCR_EXCITATION_FREQ_MILLIHZ   976600UL
#define LCR_TIM2_PRESCALER            0UL
#define LCR_TIM2_AUTORELOAD           21502UL
#define LCR_ADC_SAMPLE_COUNT          64U
#define LCR_MEASUREMENT_SETTLE_MS     180U
#define LCR_ADC_VREF_VOLT             3.3f
#define LCR_ADC_FULL_SCALE_COUNT      4095.0f
#define LCR_ADC_BIAS_VOLT             1.65f
#define LCR_REFERENCE_RESISTOR_OHM    1000.0f
#define LCR_OLED_REFRESH_MS           250U
#define LCR_OLED_SMOOTH_ALPHA         0.25f
#define LCR_KEY_DEBOUNCE_MS           30U
/* These switch defaults are a first pass for the 3.2 kOhm bring-up path. */
#define LCR_IR_RANGE_S0_STATE         GPIO_PIN_SET
#define LCR_IR_RANGE_S1_STATE         GPIO_PIN_RESET
#define LCR_FILTER_S0_STATE           GPIO_PIN_RESET
#define LCR_FILTER_S1_STATE           GPIO_PIN_SET
#define LCR_GAIN_S0_STATE             GPIO_PIN_SET
#define LCR_GAIN_S1_STATE             GPIO_PIN_RESET
/* 3.2 kOhm target: U5 selects 2Y1, which is address 01 and maps to the 10 kOhm branch. */
#define LCR_VR_S0_STATE               GPIO_PIN_RESET
#define LCR_VR_S1_STATE               GPIO_PIN_SET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef enum
{
  LCR_STEP_VOLTAGE_IN_PHASE = 0,
  LCR_STEP_VOLTAGE_QUADRATURE,
  LCR_STEP_CURRENT_IN_PHASE,
  LCR_STEP_CURRENT_QUADRATURE,
  LCR_STEP_COUNT
} LCR_MeasurementStep;

typedef struct
{
  volatile uint16_t raw_adc[LCR_STEP_COUNT];
  volatile float projection_volt[LCR_STEP_COUNT];
  volatile float voltage_real;
  volatile float voltage_imag;
  volatile float current_real;
  volatile float current_imag;
  volatile float impedance_real_ohm;
  volatile float impedance_imag_ohm;
  volatile float impedance_abs_ohm;
  volatile float phase_deg;
  volatile uint32_t update_count;
  volatile uint32_t sync_start_count;
  volatile uint32_t last_sync_tick_ms;
  volatile uint32_t last_sample_tick_ms;
  volatile uint16_t last_adc_average;
  volatile uint8_t last_sync_step;
  volatile uint8_t active_step;
  volatile uint8_t active_path_is_voltage;
  volatile uint8_t active_reference_is_quadrature;
} LCR_DebugData;

typedef enum
{
  LCR_DISPLAY_PAGE_MEASUREMENT = 0,
  LCR_DISPLAY_PAGE_DEBUG_VALUES
} LCR_DisplayPage;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint8_t quadrature_step = 0U;
volatile LCR_DebugData lcr_debug = {0};
static LCR_MeasurementStep lcr_measurement_step = LCR_STEP_VOLTAGE_IN_PHASE;
static uint32_t lcr_next_sample_tick_ms = 0U;
static uint32_t lcr_next_oled_refresh_tick_ms = 0U;
static LCR_DisplayPage lcr_display_page = LCR_DISPLAY_PAGE_MEASUREMENT;
volatile uint8_t lcr_force_step_enable = 0U;
volatile uint8_t lcr_force_step_index = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void LCR_Power_Enable(void);
static void LCR_Configure_ExcitationPath(void);
static void LCR_Configure_MeasurementPath(void);
static void LCR_Start_Excitation(void);
static void LCR_QuadratureOutput_SetStep(uint8_t step);
static void LCR_QuadratureOutput_Start(void);
static void LCR_Set_InputSelection(uint8_t select_voltage);
static void LCR_Set_ReferenceSelection(uint8_t select_quadrature);
static uint16_t LCR_ADC_ReadAverage(uint32_t sample_count);
static float LCR_ADC_CountToInputVolt(uint16_t adc_count);
static float LCR_ADC_CountToProjectionVolt(uint16_t adc_count);
static void LCR_UpdateImpedanceEstimate(void);
static void LCR_StartMeasurementStep(LCR_MeasurementStep step);
static void LCR_Measurement_Task(void);
static void LCR_Key_Task(void);
static void LCR_OLED_InitScreen(void);
static void LCR_OLED_Task(void);
static const char *LCR_GetStepName(LCR_MeasurementStep step);
void LCR_QuadratureOutput_Tick(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void LCR_Power_Enable(void)
{
 
  HAL_GPIO_WritePin(EN__3_3VB4_GPIO_Port, EN__3_3VB4_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VOUT_EN_GPIO_Port, VOUT_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(EN__5V_GPIO_Port, EN__5V_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN__3_3V_GPIO_Port, EN__3_3V_Pin, GPIO_PIN_SET);
}

static void LCR_Configure_ExcitationPath(void)
{
  /* First bench bring-up: keep the DDS excitation switch matrix in the baseline path. */
  HAL_GPIO_WritePin(DDS_SW_S0_GPIO_Port, DDS_SW_S0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DDS_SW_S1_GPIO_Port, DDS_SW_S1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Q7_RELAY_GPIO_Port, Q7_RELAY_Pin, GPIO_PIN_RESET);
}

static void LCR_Configure_MeasurementPath(void)
{
  /* Fixed 3.2 kOhm bring-up path. 53_S1_IO is intentionally unused on this board. */
  HAL_GPIO_WritePin(IR_S0_GPIO_Port, IR_S0_Pin, LCR_IR_RANGE_S0_STATE);
  HAL_GPIO_WritePin(IR_S1_GPIO_Port, IR_S1_Pin, LCR_IR_RANGE_S1_STATE);
  HAL_GPIO_WritePin(FIL_SW_S0_GPIO_Port, FIL_SW_S0_Pin, LCR_FILTER_S0_STATE);
  HAL_GPIO_WritePin(FIL_SW_S1_GPIO_Port, FIL_SW_S1_Pin, LCR_FILTER_S1_STATE);
  HAL_GPIO_WritePin(GAIN_SW_S0_GPIO_Port, GAIN_SW_S0_Pin, LCR_GAIN_S0_STATE);
  HAL_GPIO_WritePin(GAIN_SW_S1_GPIO_Port, GAIN_SW_S1_Pin, LCR_GAIN_S1_STATE);
  HAL_GPIO_WritePin(VR_S0_GPIO_Port, VR_S0_Pin, LCR_VR_S0_STATE);
  HAL_GPIO_WritePin(VR_S1_GPIO_Port, VR_S1_Pin, LCR_VR_S1_STATE);
  HAL_GPIO_WritePin(IO_SW_S1_GPIO_Port, IO_SW_S1_Pin, GPIO_PIN_RESET);
}

static void LCR_Start_Excitation(void)
{
  AD9834_Set_MclkHz(75000000UL);
  AD9834_Init();
  AD9834_Set_FreqMilliHz(FREQ_0, LCR_EXCITATION_FREQ_MILLIHZ);
  AD9834_Set_Phase(PHASE_0, 0.0f);
  AD9834_Select_Wave(SINE_WAVE);
  AD9834_Set_OutputEnabled(1U);
  AD9834_Set_Reset(0U);
}

static void LCR_QuadratureOutput_SetStep(uint8_t step)
{
  uint32_t bsrr_value = 0U;

  switch (step & 0x03U)
  {
    case 0U:
      bsrr_value = ANGLE_0_SW_Pin | ((uint32_t)ANGLE_90_SW_Pin << 16U);
      break;

    case 1U:
      bsrr_value = ANGLE_0_SW_Pin | ANGLE_90_SW_Pin;
      break;

    case 2U:
      bsrr_value = ANGLE_90_SW_Pin | ((uint32_t)ANGLE_0_SW_Pin << 16U);
      break;

    default:
      bsrr_value = ((uint32_t)(ANGLE_0_SW_Pin | ANGLE_90_SW_Pin) << 16U);
      break;
  }

  GPIOB->BSRR = bsrr_value;
}

static void LCR_QuadratureOutput_Start(void)
{
  quadrature_step = 0U;
  LCR_QuadratureOutput_SetStep(quadrature_step);

  __HAL_RCC_TIM2_CLK_ENABLE();

  TIM2->CR1 = 0U;
  TIM2->PSC = (uint32_t)LCR_TIM2_PRESCALER;
  TIM2->ARR = (uint32_t)LCR_TIM2_AUTORELOAD;
  TIM2->CNT = 0U;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0U;
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

  HAL_NVIC_SetPriority(TIM2_IRQn, 4U, 0U);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void LCR_QuadratureOutput_Tick(void)
{
  quadrature_step = (uint8_t)((quadrature_step + 1U) & 0x03U);
  LCR_QuadratureOutput_SetStep(quadrature_step);
}

static void LCR_Set_InputSelection(uint8_t select_voltage)
{
  /* User-confirmed mapping: 53_S3_IO=1 selects V_SIGE1, 53_S3_IO=0 selects I_SIGE1. */
  HAL_GPIO_WritePin(IO_SW_S3_GPIO_Port, IO_SW_S3_Pin, select_voltage ? GPIO_PIN_SET : GPIO_PIN_RESET);
  lcr_debug.active_path_is_voltage = select_voltage ? 1U : 0U;
}

static void LCR_Set_ReferenceSelection(uint8_t select_quadrature)
{
  /* User-confirmed mapping: PB10=1 selects 90 deg, PB10=0 selects 0 deg. */
  HAL_GPIO_WritePin(Q7_RELAY_GPIO_Port, Q7_RELAY_Pin, select_quadrature ? GPIO_PIN_SET : GPIO_PIN_RESET);
  lcr_debug.active_reference_is_quadrature = select_quadrature ? 1U : 0U;
}

static uint16_t LCR_ADC_ReadAverage(uint32_t sample_count)
{
  uint32_t i;
  uint32_t sum = 0U;

  if (sample_count == 0U)
  {
    sample_count = 1U;
  }

  for (i = 0U; i < sample_count; i++)
  {
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      return 0U;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc1);
      return 0U;
    }

    sum += HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
  }

  return (uint16_t)(sum / sample_count);
}

static float LCR_ADC_CountToInputVolt(uint16_t adc_count)
{
  return ((float)adc_count * LCR_ADC_VREF_VOLT) / LCR_ADC_FULL_SCALE_COUNT;
}

static float LCR_ADC_CountToProjectionVolt(uint16_t adc_count)
{
  return LCR_ADC_CountToInputVolt(adc_count) - LCR_ADC_BIAS_VOLT;
}

static void LCR_UpdateImpedanceEstimate(void)
{
  float denominator;
  float real_part;
  float imag_part;
  float magnitude_sq;

  lcr_debug.voltage_real = lcr_debug.projection_volt[LCR_STEP_VOLTAGE_IN_PHASE];
  lcr_debug.voltage_imag = lcr_debug.projection_volt[LCR_STEP_VOLTAGE_QUADRATURE];
  lcr_debug.current_real = lcr_debug.projection_volt[LCR_STEP_CURRENT_IN_PHASE];
  lcr_debug.current_imag = lcr_debug.projection_volt[LCR_STEP_CURRENT_QUADRATURE];

  denominator = (lcr_debug.current_real * lcr_debug.current_real) +
                (lcr_debug.current_imag * lcr_debug.current_imag);
  if (denominator < 1.0e-12f)
  {
    lcr_debug.impedance_real_ohm = 0.0f;
    lcr_debug.impedance_imag_ohm = 0.0f;
    lcr_debug.impedance_abs_ohm = 0.0f;
    lcr_debug.phase_deg = 0.0f;
    return;
  }

  real_part = ((lcr_debug.voltage_real * lcr_debug.current_real) +
               (lcr_debug.voltage_imag * lcr_debug.current_imag)) / denominator;
  imag_part = ((lcr_debug.voltage_imag * lcr_debug.current_real) -
               (lcr_debug.voltage_real * lcr_debug.current_imag)) / denominator;

  lcr_debug.impedance_real_ohm = real_part * LCR_REFERENCE_RESISTOR_OHM;
  lcr_debug.impedance_imag_ohm = imag_part * LCR_REFERENCE_RESISTOR_OHM;

  magnitude_sq = (lcr_debug.impedance_real_ohm * lcr_debug.impedance_real_ohm) +
                 (lcr_debug.impedance_imag_ohm * lcr_debug.impedance_imag_ohm);
  lcr_debug.impedance_abs_ohm = sqrtf(magnitude_sq);
  lcr_debug.phase_deg = atan2f(lcr_debug.impedance_imag_ohm, lcr_debug.impedance_real_ohm) * 57.2957795f;
}

static void LCR_StartMeasurementStep(LCR_MeasurementStep step)
{
  lcr_measurement_step = step;
  lcr_debug.active_step = (uint8_t)step;

  switch (step)
  {
    case LCR_STEP_VOLTAGE_IN_PHASE:
      LCR_Set_InputSelection(1U);
      LCR_Set_ReferenceSelection(0U);
      break;

    case LCR_STEP_VOLTAGE_QUADRATURE:
      LCR_Set_InputSelection(1U);
      LCR_Set_ReferenceSelection(1U);
      break;

    case LCR_STEP_CURRENT_IN_PHASE:
      LCR_Set_InputSelection(0U);
      LCR_Set_ReferenceSelection(0U);
      break;

    default:
      LCR_Set_InputSelection(0U);
      LCR_Set_ReferenceSelection(1U);
      break;
  }

  lcr_debug.last_sync_step = (uint8_t)step;
  lcr_debug.last_sync_tick_ms = HAL_GetTick();
  lcr_debug.sync_start_count++;
  lcr_next_sample_tick_ms = HAL_GetTick() + LCR_MEASUREMENT_SETTLE_MS;
}

static void LCR_Measurement_Task(void)
{
  uint16_t adc_average;
  uint32_t now_tick_ms;
  LCR_MeasurementStep next_step;
  static uint8_t last_force_step_enable = 0U;
  static uint8_t last_force_step_index = 0xFFU;

  now_tick_ms = HAL_GetTick();

  if ((lcr_force_step_enable != last_force_step_enable) ||
      (lcr_force_step_index != last_force_step_index))
  {
    last_force_step_enable = lcr_force_step_enable;
    last_force_step_index = lcr_force_step_index;

    if (lcr_force_step_enable != 0U)
    {
      LCR_StartMeasurementStep((LCR_MeasurementStep)(lcr_force_step_index & 0x03U));
    }
  }

  if ((int32_t)(now_tick_ms - lcr_next_sample_tick_ms) < 0)
  {
    return;
  }

  adc_average = LCR_ADC_ReadAverage(LCR_ADC_SAMPLE_COUNT);
  lcr_debug.last_adc_average = adc_average;
  lcr_debug.last_sample_tick_ms = now_tick_ms;
  lcr_debug.raw_adc[lcr_measurement_step] = adc_average;
  lcr_debug.projection_volt[lcr_measurement_step] = LCR_ADC_CountToProjectionVolt(adc_average);

  if (lcr_force_step_enable != 0U)
  {
    next_step = (LCR_MeasurementStep)(lcr_force_step_index & 0x03U);
  }
  else
  {
    next_step = (LCR_MeasurementStep)(((uint32_t)lcr_measurement_step + 1U) % (uint32_t)LCR_STEP_COUNT);
  }

  if (next_step == LCR_STEP_VOLTAGE_IN_PHASE)
  {
    LCR_UpdateImpedanceEstimate();
    lcr_debug.update_count++;
  }

  LCR_StartMeasurementStep(next_step);
}

static void LCR_OLED_InitScreen(void)
{
  OLED_Init();
  OLED_Clear();
  OLED_Update();
}

static void LCR_Key_Task(void)
{
  GPIO_PinState key4_state;
  GPIO_PinState key1_state;
  uint32_t now_tick_ms;
  static GPIO_PinState last_key4_sampled_state = GPIO_PIN_SET;
  static GPIO_PinState key4_stable_state = GPIO_PIN_SET;
  static uint32_t last_key4_transition_tick_ms = 0U;
  static GPIO_PinState last_key1_sampled_state = GPIO_PIN_SET;
  static GPIO_PinState key1_stable_state = GPIO_PIN_SET;
  static uint32_t last_key1_transition_tick_ms = 0U;

  now_tick_ms = HAL_GetTick();
  key4_state = HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin);
  key1_state = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);

  if (key4_state != last_key4_sampled_state)
  {
    last_key4_sampled_state = key4_state;
    last_key4_transition_tick_ms = now_tick_ms;
  }

  if (((now_tick_ms - last_key4_transition_tick_ms) >= LCR_KEY_DEBOUNCE_MS) &&
      (key4_stable_state != last_key4_sampled_state))
  {
    key4_stable_state = last_key4_sampled_state;

    if (key4_stable_state == GPIO_PIN_RESET)
    {
      if (lcr_display_page == LCR_DISPLAY_PAGE_MEASUREMENT)
      {
        lcr_display_page = LCR_DISPLAY_PAGE_DEBUG_VALUES;
      }
      else
      {
        lcr_display_page = LCR_DISPLAY_PAGE_MEASUREMENT;
      }
    }
  }

  if (key1_state != last_key1_sampled_state)
  {
    last_key1_sampled_state = key1_state;
    last_key1_transition_tick_ms = now_tick_ms;
  }

  if (((now_tick_ms - last_key1_transition_tick_ms) >= LCR_KEY_DEBOUNCE_MS) &&
      (key1_stable_state != last_key1_sampled_state))
  {
    key1_stable_state = last_key1_sampled_state;

    if (key1_stable_state == GPIO_PIN_RESET)
    {
      lcr_force_step_enable = 1U;
      lcr_force_step_index = (uint8_t)((lcr_force_step_index + 1U) & 0x03U);
      lcr_display_page = LCR_DISPLAY_PAGE_DEBUG_VALUES;
    }
  }
}

static const char *LCR_GetStepName(LCR_MeasurementStep step)
{
  static const char *const step_names[LCR_STEP_COUNT] =
  {
    "V0",
    "V90",
    "I0",
    "I90"
  };

  return step_names[(uint32_t)step];
}

static void LCR_OLED_Task(void)
{
  char line_buffer[18];
  uint32_t now_tick_ms;
  float resistance_ohm;
  float phase_deg;
  static float filtered_resistance_ohm = 0.0f;
  static float filtered_phase_deg = 0.0f;
  static uint8_t filter_initialized = 0U;

  now_tick_ms = HAL_GetTick();
  if ((int32_t)(now_tick_ms - lcr_next_oled_refresh_tick_ms) < 0)
  {
    return;
  }

  lcr_next_oled_refresh_tick_ms = now_tick_ms + LCR_OLED_REFRESH_MS;

  resistance_ohm = lcr_debug.impedance_abs_ohm;
  if (resistance_ohm < 0.0f)
  {
    resistance_ohm = -resistance_ohm;
  }
  phase_deg = lcr_debug.phase_deg;

  if (filter_initialized == 0U)
  {
    filtered_resistance_ohm = resistance_ohm;
    filtered_phase_deg = phase_deg;
    filter_initialized = 1U;
  }
  else
  {
    filtered_resistance_ohm += (resistance_ohm - filtered_resistance_ohm) * LCR_OLED_SMOOTH_ALPHA;
    filtered_phase_deg += (phase_deg - filtered_phase_deg) * LCR_OLED_SMOOTH_ALPHA;
  }

  OLED_Clear();

  if (lcr_display_page == LCR_DISPLAY_PAGE_MEASUREMENT)
  {
  OLED_ShowString(0U, 0U, "LCR 976.6Hz", OLED_8X16);
    OLED_ShowString(0U, 16U, "R:", OLED_8X16);
    if (filtered_resistance_ohm >= 1000.0f)
    {
      (void)snprintf(line_buffer, sizeof(line_buffer), "%6.2fk", filtered_resistance_ohm / 1000.0f);
    }
    else
    {
      (void)snprintf(line_buffer, sizeof(line_buffer), "%7.1f", filtered_resistance_ohm);
    }
    OLED_ShowString(16U, 16U, line_buffer, OLED_8X16);

    OLED_ShowString(0U, 32U, "Phase:", OLED_8X16);
    (void)snprintf(line_buffer, sizeof(line_buffer), "%+6.2f", filtered_phase_deg);
    OLED_ShowString(48U, 32U, line_buffer, OLED_8X16);

    OLED_ShowString(0U, 48U, "Mode:", OLED_8X16);
    (void)snprintf(line_buffer, sizeof(line_buffer),
                   "%c %c %5lu",
                   lcr_debug.active_path_is_voltage ? 'V' : 'I',
                   lcr_debug.active_reference_is_quadrature ? 'Q' : 'P',
                   (unsigned long)lcr_debug.update_count);
    OLED_ShowString(40U, 48U, line_buffer, OLED_8X16);
  }
  else
  {
    (void)snprintf(line_buffer, sizeof(line_buffer), "DBG F:%s", LCR_GetStepName((LCR_MeasurementStep)(lcr_force_step_index & 0x03U)));
    OLED_ShowString(0U, 0U, line_buffer, OLED_6X8);
    (void)snprintf(line_buffer, sizeof(line_buffer), "ADC:%+5.3f", LCR_ADC_CountToInputVolt(lcr_debug.raw_adc[lcr_force_step_index & 0x03U]));
    OLED_ShowString(0U, 9U, line_buffer, OLED_6X8);
    (void)snprintf(line_buffer, sizeof(line_buffer), "V0 :%+6.3f", lcr_debug.projection_volt[LCR_STEP_VOLTAGE_IN_PHASE]);
    OLED_ShowString(0U, 18U, line_buffer, OLED_6X8);
    (void)snprintf(line_buffer, sizeof(line_buffer), "V90:%+6.3f", lcr_debug.projection_volt[LCR_STEP_VOLTAGE_QUADRATURE]);
    OLED_ShowString(0U, 27U, line_buffer, OLED_6X8);
    (void)snprintf(line_buffer, sizeof(line_buffer), "I0 :%+6.3f", lcr_debug.projection_volt[LCR_STEP_CURRENT_IN_PHASE]);
    OLED_ShowString(0U, 36U, line_buffer, OLED_6X8);
    (void)snprintf(line_buffer, sizeof(line_buffer), "I90:%+6.3f", lcr_debug.projection_volt[LCR_STEP_CURRENT_QUADRATURE]);
    OLED_ShowString(0U, 45U, line_buffer, OLED_6X8);
  }

  OLED_Update();
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  LCR_Power_Enable();
  LCR_Configure_ExcitationPath();
  LCR_Configure_MeasurementPath();
  LCR_Start_Excitation();
  LCR_QuadratureOutput_Start();
  HAL_Delay(20);
  LCR_OLED_InitScreen();
  lcr_next_oled_refresh_tick_ms = HAL_GetTick();
  LCR_StartMeasurementStep(LCR_STEP_VOLTAGE_IN_PHASE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LCR_Measurement_Task();
    LCR_Key_Task();
    LCR_OLED_Task();
    __WFI();
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
