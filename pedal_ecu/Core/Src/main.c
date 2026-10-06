/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Pedal ECU
  ******************************************************************************
  * CAN ID : 0x100
  * DLC    : 8
  * Cycle  : 10 ms
  *
  * Byte 0   : Accelerator Pedal (%)
  * Byte 1   : Brake Pedal (%)
  * Byte 2~3 : Accelerator Rate (int16_t, 0.1 %/s, Little Endian)
  * Byte 4~5 : Brake Rate       (int16_t, 0.1 %/s, Little Endian)
  * Byte 6   : Sensor Status
  * Byte 7   : Alive Counter
  *
  * Sensor Status
  * 0x00 : Normal
  * 0x01 : Accelerator ADC Range Error
  * 0x02 : Brake ADC Range Error
  * 0x04 : Accelerator Sensor Disconnect
  * 0x08 : Brake Sensor Disconnect
  * 0x10 : Accelerator Calibration Error
  * 0x20 : Brake Calibration Error
  * 0x40 : Reserved
  * 0x80 : Reserved
  *
 * Sensor disconnect policy:
 *   PB0/PB1 Detect lines are NOT used.
 *   PA0/PA1 internal pull-up/pull-down are NOT used.
 *   Percentage-based disconnect detection is disabled
 *   because the floating value overlaps the normal pedal range.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdint.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* Private define ------------------------------------------------------------*/

/* ADC mapping */
#define BRAKE_ADC_CHANNEL               ADC_CHANNEL_0   /* PA0 / A0 */
#define ACCEL_ADC_CHANNEL               ADC_CHANNEL_1   /* PA1 / A1 */

/* Calibration values */
#define ACCEL_RELEASED_ADC              3842U
#define ACCEL_FULL_ADC                   451U
#define BRAKE_RELEASED_ADC              3705U
#define BRAKE_FULL_ADC                   404U
#define CALIBRATION_MIN_SPAN            2500U

/* ============================================================
 * ADC FILTER
 *
 * 각 채널을 16회 읽어서 평균값을 사용한다.
 * 평균값이 기존 필터값에서 8 count 이하로만 변하면
 * 기존 값을 유지해서 작은 노이즈를 제거한다.
 * ============================================================ */
#define ADC_AVERAGE_SAMPLES               16U
#define ADC_DEADBAND                      15U

/* ============================================================
 * PEDAL PERCENT STABILIZATION
 *
 * 0~2%는 0%로 고정
 * 98~100%는 100%로 고정
 * 중간 영역에서는 1% 변화는 무시하고
 * 2% 이상 차이가 날 때만 새 퍼센트를 반영
 * ============================================================ */
#define PEDAL_ZERO_DEADZONE_PERCENT        2U
#define PEDAL_FULL_DEADZONE_PERCENT       98U
#define PEDAL_PERCENT_HYSTERESIS            2U


/*
 * SENSOR DISCONNECT DIAGNOSTIC
 *
 * 현재 하드웨어는 3선식 페달(+ / GND / Signal) 구조이며,
 * 센서 단선 시 floating ADC 값이 정상 페달 입력 범위와
 * 겹치는 현상이 확인되었다.
 *
 * 따라서 percentage 기반 Disconnect 진단은
 * false positive 방지를 위해 현재 버전에서는 비활성화한다.
 *
 * 센서 이상 판단은 ADC Range Error,
 * Calibration Error 및 Safety ECU의
 * PEDAL_STATUS communication timeout을 사용한다.
 */
#define ACC_DISCONNECT_MIN_PERCENT          28U
#define ACC_DISCONNECT_MAX_PERCENT          32U

#define BRAKE_DISCONNECT_MIN_PERCENT        32U
#define BRAKE_DISCONNECT_MAX_PERCENT        36U

#define BOTH_ACC_DISCONNECT_MIN_PERCENT     74U
#define BOTH_ACC_DISCONNECT_MAX_PERCENT     79U

#define BOTH_BRAKE_DISCONNECT_MIN_PERCENT   69U
#define BOTH_BRAKE_DISCONNECT_MAX_PERCENT   72U

/* 10 ms x 5 = 50 ms 동안 패턴이 지속되면 Disconnect 확정 */
#define DISCONNECT_CONFIRM_COUNT             5U

/* 정상 패턴이 10 ms x 10 = 100 ms 지속되면 Disconnect 해제 */
#define DISCONNECT_RECOVERY_COUNT           10U


/* ============================================================
 * ADC Range Error
 *
 * 내부 Pull-up/Pull-down은 사용하지 않는다.
 * 정상 Calibration 범위보다 약간 넓게 잡는다.
 * ============================================================ */
#define ACCEL_ADC_MIN_VALID              300U
#define ACCEL_ADC_MAX_VALID             4000U

#define BRAKE_ADC_MIN_VALID              250U
#define BRAKE_ADC_MAX_VALID             3900U

/* 10 ms x 10 = 100 ms */
#define ADC_RANGE_ERROR_COUNT_LIMIT       10U

/* 정상값 20회(200 ms) 연속 확인 후 오류 해제 */
#define ADC_RANGE_RECOVERY_COUNT_LIMIT    20U

/* CAN */
#define PEDAL_CAN_ID                    0x100U
#define PEDAL_CAN_DLC                      8U
#define PEDAL_CAN_PERIOD_MS               10U

/* Live Expressions debug update */
#define DEBUG_UPDATE_PERIOD_MS            200U

/* Global debug/state variables ---------------------------------------------*/
/* 최종 필터링된 ADC 값 */
volatile uint16_t accel_adc = 0U;
volatile uint16_t brake_adc = 0U;

/* 16회 평균 직후 값 (Deadband 적용 전) */
volatile uint16_t accel_adc_average = 0U;
volatile uint16_t brake_adc_average = 0U;

static uint8_t adc_filter_initialized = 0U;

volatile uint16_t accel_tenth_percent = 0U;
volatile uint16_t brake_tenth_percent = 0U;

volatile uint8_t accel_percent = 0U;
volatile uint8_t brake_percent = 0U;

static uint8_t accel_percent_stable = 0U;
static uint8_t brake_percent_stable = 0U;
static uint8_t percent_filter_initialized = 0U;


/* Sensor Disconnect 진단 상태 */
volatile uint8_t acc_disconnect_active = 0U;
volatile uint8_t brake_disconnect_active = 0U;

static uint8_t acc_disconnect_count = 0U;
static uint8_t brake_disconnect_count = 0U;

static uint8_t acc_disconnect_recovery_count = 0U;
static uint8_t brake_disconnect_recovery_count = 0U;

static uint8_t acc_disconnect_pattern = 0U;
static uint8_t brake_disconnect_pattern = 0U;
static uint8_t both_disconnect_pattern = 0U;


volatile int16_t accel_rate_raw = 0;
volatile int16_t brake_rate_raw = 0;

volatile uint8_t sensor_status = 0x00U;
volatile uint8_t alive_counter = 0U;

/* ADC Range Error debounce / recovery */
volatile uint8_t acc_adc_range_error_count = 0U;
volatile uint8_t brake_adc_range_error_count = 0U;

volatile uint8_t acc_adc_range_recovery_count = 0U;
volatile uint8_t brake_adc_range_recovery_count = 0U;

volatile uint8_t acc_adc_range_error_active = 0U;
volatile uint8_t brake_adc_range_error_active = 0U;

volatile uint32_t can_tx_ok_count = 0U;
volatile uint32_t can_tx_error_count = 0U;

/* Slow variables for Live Expressions */
volatile uint16_t debug_accel_adc = 0U;
volatile uint16_t debug_brake_adc = 0U;
volatile uint16_t debug_accel_adc_average = 0U;
volatile uint16_t debug_brake_adc_average = 0U;
volatile uint8_t debug_accel_percent = 0U;
volatile uint8_t debug_brake_percent = 0U;
volatile int16_t debug_accel_rate_raw = 0;
volatile int16_t debug_brake_rate_raw = 0;
volatile uint8_t debug_sensor_status = 0x00U;
volatile uint8_t debug_alive_counter = 0U;
volatile uint8_t debug_acc_adc_range_error = 0U;
volatile uint8_t debug_brake_adc_range_error = 0U;

volatile uint8_t debug_acc_disconnect = 0U;
volatile uint8_t debug_brake_disconnect = 0U;
volatile uint8_t debug_acc_disconnect_pattern = 0U;
volatile uint8_t debug_brake_disconnect_pattern = 0U;
volatile uint8_t debug_both_disconnect_pattern = 0U;

static uint32_t last_pedal_tick = 0U;
static uint32_t last_debug_tick = 0U;

static uint16_t prev_accel_tenth_percent = 0U;
static uint16_t prev_brake_tenth_percent = 0U;
static uint8_t first_pedal_cycle = 1U;

static CAN_TxHeaderTypeDef pedal_tx_header;
static uint8_t pedal_tx_data[8];
static uint32_t pedal_tx_mailbox;

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);

static void CAN_Start(void);
static void Read_Pedal_ADC(void);
static uint16_t Apply_ADC_Deadband(uint16_t previous_value,
                                   uint16_t new_value,
                                   uint16_t deadband);
static uint8_t Apply_Percent_Stabilization(uint8_t previous_percent,
                                           uint8_t new_percent);
static uint16_t ADC_To_TenthPercent(uint16_t adc_value,
                                    uint16_t released_adc,
                                    uint16_t full_adc);
static int16_t Saturate_Int16(int32_t value);
static void Calculate_Pedal_Data(void);
static void Update_ADC_Range_Diagnostic(void);
static void Update_Disconnect_Diagnostic(void);
static void Update_Sensor_Status(void);
static void Send_Pedal_CAN(void);
static void Pedal_ECU_10ms_Task(void);
static void Update_Debug_Variables(void);

/* User functions ------------------------------------------------------------*/

static void CAN_Start(void)
{
    pedal_tx_header.StdId = PEDAL_CAN_ID;
    pedal_tx_header.ExtId = 0U;
    pedal_tx_header.IDE = CAN_ID_STD;
    pedal_tx_header.RTR = CAN_RTR_DATA;
    pedal_tx_header.DLC = PEDAL_CAN_DLC;
    pedal_tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void Read_Pedal_ADC(void)
{
    uint32_t brake_sum = 0U;
    uint32_t accel_sum = 0U;
    uint16_t sample;
    uint16_t valid_sample_count = 0U;

    /*
     * 10 ms Task가 호출될 때마다
     * Brake / Accelerator를 각각 16회 측정해서 평균값 사용
     */
    for (sample = 0U; sample < ADC_AVERAGE_SAMPLES; sample++)
    {
        if (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            continue;
        }

        /* Rank 1 : Brake / PA0 / ADC1_IN0 */
        if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            continue;
        }

        brake_sum += HAL_ADC_GetValue(&hadc1);

        /* Rank 2 : Accelerator / PA1 / ADC1_IN1 */
        if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            continue;
        }

        accel_sum += HAL_ADC_GetValue(&hadc1);

        HAL_ADC_Stop(&hadc1);

        valid_sample_count++;
    }

    /*
     * 정상적으로 읽은 샘플이 하나도 없으면
     * 이전 ADC 값을 그대로 유지
     */
    if (valid_sample_count == 0U)
    {
        return;
    }

    brake_adc_average =
        (uint16_t)(brake_sum / valid_sample_count);

    accel_adc_average =
        (uint16_t)(accel_sum / valid_sample_count);

    /*
     * 첫 ADC 측정은 평균값 그대로 사용
     */
    if (adc_filter_initialized == 0U)
    {
        brake_adc = brake_adc_average;
        accel_adc = accel_adc_average;

        adc_filter_initialized = 1U;
    }
    else
    {
        /*
         * 평균값이 기존값과 8 count 이하 차이면
         * 기존값 유지 → 작은 노이즈 제거
         */
        brake_adc =
            Apply_ADC_Deadband(
                brake_adc,
                brake_adc_average,
                ADC_DEADBAND
            );

        accel_adc =
            Apply_ADC_Deadband(
                accel_adc,
                accel_adc_average,
                ADC_DEADBAND
            );
    }
}


static uint16_t Apply_ADC_Deadband(uint16_t previous_value,
                                   uint16_t new_value,
                                   uint16_t deadband)
{
    uint16_t diff;

    if (new_value >= previous_value)
    {
        diff = new_value - previous_value;
    }
    else
    {
        diff = previous_value - new_value;
    }

    /*
     * Deadband 범위 안의 작은 변화는 무시
     */
    if (diff <= deadband)
    {
        return previous_value;
    }

    /*
     * 실제 페달 변화로 판단되면 새 값 반영
     */
    return new_value;
}


static uint8_t Apply_Percent_Stabilization(uint8_t previous_percent,
                                           uint8_t new_percent)
{
    uint8_t diff;

    /* Release 근처는 확실하게 0% */
    if (new_percent <= PEDAL_ZERO_DEADZONE_PERCENT)
    {
        return 0U;
    }

    /* Full 근처는 확실하게 100% */
    if (new_percent >= PEDAL_FULL_DEADZONE_PERCENT)
    {
        return 100U;
    }

    if (new_percent >= previous_percent)
    {
        diff = new_percent - previous_percent;
    }
    else
    {
        diff = previous_percent - new_percent;
    }

    /* 1% 정도의 작은 변화는 노이즈로 보고 무시 */
    if (diff < PEDAL_PERCENT_HYSTERESIS)
    {
        return previous_percent;
    }

    return new_percent;
}


static uint16_t ADC_To_TenthPercent(uint16_t adc_value,
                                    uint16_t released_adc,
                                    uint16_t full_adc)
{
    uint32_t result;

    if (released_adc <= full_adc)
    {
        return 0U;
    }

    if (adc_value >= released_adc)
    {
        return 0U;
    }

    if (adc_value <= full_adc)
    {
        return 1000U;
    }

    result = ((uint32_t)(released_adc - adc_value) * 1000U) /
             (uint32_t)(released_adc - full_adc);

    if (result > 1000U)
    {
        result = 1000U;
    }

    return (uint16_t)result;
}

static int16_t Saturate_Int16(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }

    if (value < -32768)
    {
        return -32768;
    }

    return (int16_t)value;
}

static void Calculate_Pedal_Data(void)
{
    int32_t acc_rate_temp;
    int32_t brake_rate_temp;

    accel_tenth_percent = ADC_To_TenthPercent(accel_adc,
                                               ACCEL_RELEASED_ADC,
                                               ACCEL_FULL_ADC);

    brake_tenth_percent = ADC_To_TenthPercent(brake_adc,
                                               BRAKE_RELEASED_ADC,
                                               BRAKE_FULL_ADC);

    /* --------------------------------------------------------
     * CAN Byte 0, 1 : 0~100%
     * -------------------------------------------------------- */
    {
        uint8_t accel_percent_new;
        uint8_t brake_percent_new;

        accel_percent_new =
            (uint8_t)((accel_tenth_percent + 5U) / 10U);

        brake_percent_new =
            (uint8_t)((brake_tenth_percent + 5U) / 10U);

        if (accel_percent_new > 100U)
        {
            accel_percent_new = 100U;
        }

        if (brake_percent_new > 100U)
        {
            brake_percent_new = 100U;
        }

        if (percent_filter_initialized == 0U)
        {
            accel_percent_stable =
                Apply_Percent_Stabilization(
                    accel_percent_new,
                    accel_percent_new
                );

            brake_percent_stable =
                Apply_Percent_Stabilization(
                    brake_percent_new,
                    brake_percent_new
                );

            percent_filter_initialized = 1U;
        }
        else
        {
            accel_percent_stable =
                Apply_Percent_Stabilization(
                    accel_percent_stable,
                    accel_percent_new
                );

            brake_percent_stable =
                Apply_Percent_Stabilization(
                    brake_percent_stable,
                    brake_percent_new
                );
        }

        accel_percent = accel_percent_stable;
        brake_percent = brake_percent_stable;
    }

    /* First sample: rate = 0 */
    if (first_pedal_cycle != 0U)
    {
        accel_rate_raw = 0;
        brake_rate_raw = 0;
        prev_accel_tenth_percent = accel_tenth_percent;
        prev_brake_tenth_percent = brake_tenth_percent;
        first_pedal_cycle = 0U;
        return;
    }

    /*
     * Sample time = 10ms = 0.01s
     * Internal position unit = 0.1%
     * delta * 100 -> 0.1 %/s
     */
    acc_rate_temp = ((int32_t)accel_tenth_percent -
                     (int32_t)prev_accel_tenth_percent) * 100;

    brake_rate_temp = ((int32_t)brake_tenth_percent -
                       (int32_t)prev_brake_tenth_percent) * 100;

    accel_rate_raw = Saturate_Int16(acc_rate_temp);
    brake_rate_raw = Saturate_Int16(brake_rate_temp);

    prev_accel_tenth_percent = accel_tenth_percent;
    prev_brake_tenth_percent = brake_tenth_percent;
}

/**
  * @brief ADC 정상 범위를 10ms마다 검사
  *
  * 100ms 연속 비정상 -> Error ON
  * 200ms 연속 정상   -> Error OFF
  */
static void Update_ADC_Range_Diagnostic(void)
{
    uint8_t acc_invalid;
    uint8_t brake_invalid;

    acc_invalid =
        ((accel_adc < ACCEL_ADC_MIN_VALID) ||
         (accel_adc > ACCEL_ADC_MAX_VALID))
        ? 1U : 0U;

    brake_invalid =
        ((brake_adc < BRAKE_ADC_MIN_VALID) ||
         (brake_adc > BRAKE_ADC_MAX_VALID))
        ? 1U : 0U;

    /* --------------------------------------------------------
     * Accelerator
     * -------------------------------------------------------- */
    if (acc_invalid != 0U)
    {
        acc_adc_range_recovery_count = 0U;

        if (acc_adc_range_error_count < ADC_RANGE_ERROR_COUNT_LIMIT)
        {
            acc_adc_range_error_count++;
        }

        if (acc_adc_range_error_count >= ADC_RANGE_ERROR_COUNT_LIMIT)
        {
            acc_adc_range_error_active = 1U;
        }
    }
    else
    {
        acc_adc_range_error_count = 0U;

        if (acc_adc_range_error_active != 0U)
        {
            if (acc_adc_range_recovery_count <
                ADC_RANGE_RECOVERY_COUNT_LIMIT)
            {
                acc_adc_range_recovery_count++;
            }

            if (acc_adc_range_recovery_count >=
                ADC_RANGE_RECOVERY_COUNT_LIMIT)
            {
                acc_adc_range_error_active = 0U;
                acc_adc_range_recovery_count = 0U;
            }
        }
        else
        {
            acc_adc_range_recovery_count = 0U;
        }
    }

    /* --------------------------------------------------------
     * Brake
     * -------------------------------------------------------- */
    if (brake_invalid != 0U)
    {
        brake_adc_range_recovery_count = 0U;

        if (brake_adc_range_error_count < ADC_RANGE_ERROR_COUNT_LIMIT)
        {
            brake_adc_range_error_count++;
        }

        if (brake_adc_range_error_count >= ADC_RANGE_ERROR_COUNT_LIMIT)
        {
            brake_adc_range_error_active = 1U;
        }
    }
    else
    {
        brake_adc_range_error_count = 0U;

        if (brake_adc_range_error_active != 0U)
        {
            if (brake_adc_range_recovery_count <
                ADC_RANGE_RECOVERY_COUNT_LIMIT)
            {
                brake_adc_range_recovery_count++;
            }

            if (brake_adc_range_recovery_count >=
                ADC_RANGE_RECOVERY_COUNT_LIMIT)
            {
                brake_adc_range_error_active = 0U;
                brake_adc_range_recovery_count = 0U;
            }
        }
        else
        {
            brake_adc_range_recovery_count = 0U;
        }
    }
}

static void Update_Disconnect_Diagnostic(void)
{
    /*
     * 현재 하드웨어는 3선식 페달(+ / GND / Signal) 구조이며,
     * 센서 단선 시 floating 값이 정상 페달 영역과 겹칠 수 있다.
     *
     * 기존 방식:
     *   Accelerator 28~32% -> Disconnect
     *   Brake       32~36% -> Disconnect
     *
     * 문제:
     *   정상 운전 중 동일한 퍼센트 영역을 유지할 경우
     *   실제 단선이 아닌데도 Disconnect가 발생할 수 있다.
     *
     * 따라서 현재 버전에서는 percentage 기반 Disconnect 진단을
     * 비활성화한다.
     *
     * 센서 이상은 다음 진단을 사용한다.
     *   - ADC Range Error
     *   - Calibration Error
     *   - Safety ECU의 PEDAL_STATUS communication timeout
     *
     * 향후 별도 Detect 회로 / 센서 이중화 등을 적용하면
     * 이 함수에서 Disconnect 진단을 다시 활성화한다.
     */

    acc_disconnect_pattern = 0U;
    brake_disconnect_pattern = 0U;
    both_disconnect_pattern = 0U;

    acc_disconnect_count = 0U;
    brake_disconnect_count = 0U;

    acc_disconnect_recovery_count = 0U;
    brake_disconnect_recovery_count = 0U;

    acc_disconnect_active = 0U;
    brake_disconnect_active = 0U;
}

static void Update_Sensor_Status(void)
{
    uint8_t status = 0x00U;

    /* ADC Range Error */
    if (acc_adc_range_error_active != 0U)
    {
        status |= 0x01U;
    }

    if (brake_adc_range_error_active != 0U)
    {
        status |= 0x02U;
    }

    /* --------------------------------------------------------
    * Sensor Disconnect
    * 현재 버전에서는 percentage 기반 진단 비활성화
    * -------------------------------------------------------- */
    if (acc_disconnect_active != 0U)
    {
        status |= 0x04U;
    }

    if (brake_disconnect_active != 0U)
    {
        status |= 0x08U;
    }

    /* Calibration Error */
    if ((ACCEL_RELEASED_ADC <= ACCEL_FULL_ADC) ||
        ((ACCEL_RELEASED_ADC - ACCEL_FULL_ADC) < CALIBRATION_MIN_SPAN) ||
        (ACCEL_RELEASED_ADC > 4095U) ||
        (ACCEL_FULL_ADC > 4095U))
    {
        status |= 0x10U;
    }

    if ((BRAKE_RELEASED_ADC <= BRAKE_FULL_ADC) ||
        ((BRAKE_RELEASED_ADC - BRAKE_FULL_ADC) < CALIBRATION_MIN_SPAN) ||
        (BRAKE_RELEASED_ADC > 4095U) ||
        (BRAKE_FULL_ADC > 4095U))
    {
        status |= 0x20U;
    }

    /*
     * No error = 0x00
     * Normal 상태를 위한 별도 bit는 사용하지 않는다.
     */
    sensor_status = status;
}

static void Send_Pedal_CAN(void)
{
    uint16_t accel_rate_u16 = (uint16_t)accel_rate_raw;
    uint16_t brake_rate_u16 = (uint16_t)brake_rate_raw;

    pedal_tx_data[0] = accel_percent;
    pedal_tx_data[1] = brake_percent;

    /* Little Endian */
    pedal_tx_data[2] = (uint8_t)(accel_rate_u16 & 0xFFU);
    pedal_tx_data[3] = (uint8_t)((accel_rate_u16 >> 8) & 0xFFU);

    pedal_tx_data[4] = (uint8_t)(brake_rate_u16 & 0xFFU);
    pedal_tx_data[5] = (uint8_t)((brake_rate_u16 >> 8) & 0xFFU);

    pedal_tx_data[6] = sensor_status;
    pedal_tx_data[7] = alive_counter;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
    {
        if (HAL_CAN_AddTxMessage(&hcan1,
                                 &pedal_tx_header,
                                 pedal_tx_data,
                                 &pedal_tx_mailbox) == HAL_OK)
        {
            can_tx_ok_count++;
            alive_counter++;
        }
        else
        {
            can_tx_error_count++;
        }
    }
    else
    {
        can_tx_error_count++;
    }
}

static void Pedal_ECU_10ms_Task(void)
{
    Read_Pedal_ADC();
    Calculate_Pedal_Data();
    Update_ADC_Range_Diagnostic();
    Update_Disconnect_Diagnostic();
    Update_Sensor_Status();
    Send_Pedal_CAN();
}

static void Update_Debug_Variables(void)
{
    debug_accel_adc = accel_adc;
    debug_brake_adc = brake_adc;

    debug_accel_adc_average = accel_adc_average;
    debug_brake_adc_average = brake_adc_average;

    debug_accel_percent = accel_percent;
    debug_brake_percent = brake_percent;

    debug_accel_rate_raw = accel_rate_raw;
    debug_brake_rate_raw = brake_rate_raw;

    debug_sensor_status = sensor_status;
    debug_alive_counter = alive_counter;

    debug_acc_adc_range_error = acc_adc_range_error_active;
    debug_brake_adc_range_error = brake_adc_range_error_active;

    debug_acc_disconnect = acc_disconnect_active;
    debug_brake_disconnect = brake_disconnect_active;

    debug_acc_disconnect_pattern = acc_disconnect_pattern;
    debug_brake_disconnect_pattern = brake_disconnect_pattern;
    debug_both_disconnect_pattern = both_disconnect_pattern;
}

/* Main ----------------------------------------------------------------------*/
int main(void)
{
    uint32_t now;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();

    /* PA0/A1은 일반 Analog Input으로 사용.
     * 내부 Pull-up/Pull-down은 사용하지 않는다.
     */
    MX_CAN1_Init();
    MX_TIM3_Init();

    CAN_Start();

    last_pedal_tick = HAL_GetTick();
    last_debug_tick = HAL_GetTick();

    while (1)
    {
        now = HAL_GetTick();

        if ((uint32_t)(now - last_pedal_tick) >= PEDAL_CAN_PERIOD_MS)
        {
            last_pedal_tick += PEDAL_CAN_PERIOD_MS;
            Pedal_ECU_10ms_Task();
        }

        if ((uint32_t)(now - last_debug_tick) >= DEBUG_UPDATE_PERIOD_MS)
        {
            last_debug_tick = now;
            Update_Debug_Variables();
        }
    }
}

/* System Clock --------------------------------------------------------------*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ADC1 ----------------------------------------------------------------------*/
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Rank 1 : Brake / PA0 / ADC1_IN0 */
    sConfig.Channel = BRAKE_ADC_CHANNEL;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Rank 2 : Accelerator / PA1 / ADC1_IN1 */
    sConfig.Channel = ACCEL_ADC_CHANNEL;
    sConfig.Rank = 2;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* CAN1 : 500 kbps -----------------------------------------------------------*/
static void MX_CAN1_Init(void)
{
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 6;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* TIM3 ----------------------------------------------------------------------*/
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim3);
}

/* USART2 --------------------------------------------------------------------*/
static void MX_USART2_UART_Init(void)
{
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
}

/* GPIO ----------------------------------------------------------------------*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA,
                      LD2_Pin | GPIO_PIN_8 | GPIO_PIN_9,
                      GPIO_PIN_RESET);

    /* User button */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /* LD2 / PA8 / PA9 */
    GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA0 / PA1 : ADC Analog Input
     * 내부 Pull-up/Pull-down은 사용하지 않는다.
     */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB0 / PB1 Detect 기능은 사용하지 않음 */
}

/* Error Handler -------------------------------------------------------------*/
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
