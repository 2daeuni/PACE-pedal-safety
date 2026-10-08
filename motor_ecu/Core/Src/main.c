/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FINAL Motor ECU - NUCLEO-F446RE
  *
  * ============================= HARDWARE ====================================
  * Board           : NUCLEO-F446RE
  *
  * ESC PWM
  *   PA8 / TIM1_CH1
  *   50 Hz
  *   Neutral        : 1500 us
  *   Drive range    : 1600 ~ 1800 us
  *
  * CAN1
  *   PA11           : CAN1_RX
  *   PA12           : CAN1_TX
  *   Bitrate        : 500 kbps
  *
  * ============================== CAN ========================================
  * 0x100 PEDAL_STATUS : Pedal ECU -> Motor ECU
  *   B0 Accelerator [%]             0~100
  *   B1 Brake [%]                   0~100
  *   B2~3 Accel rate                int16 LE
  *   B4~5 Brake rate                int16 LE
  *   B6 Sensor Status               0x00 = Normal
  *   B7 Alive Counter
  *
  * 0x120 MOTOR_COMMAND : Raspberry Pi Safety ECU -> Motor ECU
  *   B0 Control Mode
  *      0x00 Normal
  *      0x01 Output Limit
  *      0x02 Motor Stop
  *   B1 Output Limit [%]            0~100
  *   B2 Risk Level                  0~3
  *   B3 Command Flags
  *      bit0 Command Valid
  *      bit1 Emergency Stop
  *      bit2~7 Reserved
  *   B4~6 Reserved                  0x00
  *   B7 Alive Counter
  *
  * 0x210 MOTOR_STATUS : Motor ECU -> Raspberry Pi Safety ECU
  *   B0 Actual PWM [%]              0~100
  *   B1 Applied Output Limit [%]    0~100
  *   B2~3 Vehicle Speed             uint16 LE, 0.01 m/s/bit
  *                                  0xFFFF = Invalid / unavailable
  *   B4 Motor State
  *      0x00 Stop
  *      0x01 Running
  *      0x02 Output Limited
  *      0x03 Fault
  *   B5 Fault Flags
  *      bit0 Motor Driver Fault
  *      bit1 PEDAL_STATUS Timeout
  *      bit2 MOTOR_COMMAND Timeout
  *      bit3 Emergency Stop Active
  *      bit4~7 Reserved
  *   B6 Applied Command Counter
  *   B7 Alive Counter
  *
  * ============================= CONTROL =====================================
  * Normal:
  *   Final Output = Accelerator
  *
  * Output Limit:
  *   Final Output = min(Accelerator, Output Limit)
  *
  * Motor Stop:
  *   Final Output = 0
  *
  * Emergency Stop:
  *   Final Output = 0 while MOTOR_COMMAND Byte3 bit1 = 1
  *
  * Brake >= 5%:
  *   Final Output = 0
  *
  * PEDAL_STATUS / MOTOR_COMMAND timeout:
  *   Final Output = 0
  *
  * IMPORTANT:
  *   Startup 5-second 1500 us neutral hold has been removed.
  *   CAN and PWM start immediately, then normal control logic takes over.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32f4xx_hal_can.h"

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
void Error_Handler(void);

/* USER CODE BEGIN PV */

/* ==========================================================================
 * CAN IDs
 * ========================================================================== */
#define CAN_ID_PEDAL_STATUS              0x100U
#define CAN_ID_MOTOR_COMMAND             0x120U
#define CAN_ID_MOTOR_STATUS              0x210U

/* ==========================================================================
 * Timing
 * ========================================================================== */
#define PEDAL_TIMEOUT_MS                 200U
#define MOTOR_COMMAND_TIMEOUT_MS         200U
#define ALIVE_STUCK_TIMEOUT_MS           200U

#define MOTOR_CONTROL_PERIOD_MS          10U
#define MOTOR_STATUS_PERIOD_MS           10U

/* ==========================================================================
 * PEDAL_STATUS
 * ========================================================================== */
#define PEDAL_STATUS_NORMAL              0x00U

#define ACCEL_DEADBAND_PERCENT           3U
#define BRAKE_THRESHOLD_PERCENT          5U

/* ==========================================================================
 * MOTOR_COMMAND
 * ========================================================================== */
#define MOTOR_MODE_NORMAL                0x00U
#define MOTOR_MODE_OUTPUT_LIMIT          0x01U
#define MOTOR_MODE_STOP                  0x02U

#define MOTOR_CMD_FLAG_VALID             0x01U
#define MOTOR_CMD_FLAG_ESTOP             0x02U
#define MOTOR_CMD_FLAG_ALLOWED_MASK      0x03U

/* ==========================================================================
 * MOTOR_STATUS - Motor State
 * ========================================================================== */
#define MOTOR_STATE_STOP                 0x00U
#define MOTOR_STATE_RUNNING              0x01U
#define MOTOR_STATE_OUTPUT_LIMITED       0x02U
#define MOTOR_STATE_FAULT                0x03U

/* ==========================================================================
 * MOTOR_STATUS - Fault Flags
 * ========================================================================== */
#define MOTOR_FAULT_DRIVER               0x01U
#define MOTOR_FAULT_PEDAL_TIMEOUT        0x02U
#define MOTOR_FAULT_COMMAND_TIMEOUT      0x04U
#define MOTOR_FAULT_ESTOP_ACTIVE         0x08U

/* ==========================================================================
 * ESC
 * ========================================================================== */
#define ESC_NEUTRAL_US                   1500U
#define ESC_DRIVE_MIN_US                 1600U
#define ESC_DRIVE_MAX_US                 1800U
#define ESC_ABSOLUTE_MIN_US              1000U
#define ESC_ABSOLUTE_MAX_US              2000U

/* 10 us per 10 ms control cycle */
#define ESC_RAMP_STEP_US                 10U

/* ==========================================================================
 * PEDAL RX
 * ========================================================================== */
volatile uint8_t  pedal_accel = 0U;
volatile uint8_t  pedal_brake = 0U;
volatile int16_t  pedal_accel_rate = 0;
volatile int16_t  pedal_brake_rate = 0;
volatile uint8_t  pedal_sensor_status = 0xFFU;
volatile uint8_t  pedal_alive = 0U;

volatile uint8_t  pedal_received = 0U;
volatile uint8_t  pedal_alive_initialized = 0U;
volatile uint8_t  pedal_previous_alive = 0U;

volatile uint32_t pedal_last_rx_tick = 0U;
volatile uint32_t pedal_last_alive_change_tick = 0U;

/* ==========================================================================
 * MOTOR_COMMAND RX
 * ========================================================================== */
volatile uint8_t command_mode = MOTOR_MODE_STOP;
volatile uint8_t command_output_limit = 0U;
volatile uint8_t command_risk_level = 0U;
volatile uint8_t command_flags = 0U;
volatile uint8_t command_alive = 0U;

volatile uint8_t command_received = 0U;
volatile uint8_t command_alive_initialized = 0U;
volatile uint8_t command_previous_alive = 0U;

volatile uint32_t command_last_rx_tick = 0U;
volatile uint32_t command_last_alive_change_tick = 0U;

/* ==========================================================================
 * MOTOR STATUS / CONTROL
 * ========================================================================== */
volatile uint8_t motor_state = MOTOR_STATE_STOP;
volatile uint8_t motor_fault_flags = 0U;

volatile uint8_t requested_output_percent = 0U;
volatile uint8_t actual_pwm_percent = 0U;
volatile uint8_t applied_output_limit = 0U;

volatile uint8_t applied_command_counter = 0U;
volatile uint8_t motor_status_alive_counter = 0U;

/* No speed sensor yet */
volatile uint16_t vehicle_speed_raw = 0xFFFFU;

/* No motor-driver diagnostic input yet */
volatile uint8_t motor_driver_fault_active = 0U;

/* ESC pulse state */
volatile uint16_t esc_target_us = ESC_NEUTRAL_US;
volatile uint16_t esc_current_us = ESC_NEUTRAL_US;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */

static void ESC_SetPulse(uint16_t pulse_us);
static uint16_t Motor_PercentToEscPulse(uint8_t percent);
static uint8_t ESC_PulseToPercent(uint16_t pulse_us);

static void CAN_Filter_Config(void);
static void CAN_ProcessRx(void);
static void CAN_ParsePedal(const uint8_t data[8]);
static void CAN_ParseMotorCommand(const uint8_t data[8]);

static uint8_t Motor_UpdateFaultFlags(void);
static uint8_t Motor_CalculateOutputPercent(void);
static void Motor_Update(void);
static void Motor_SendStatus(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ==========================================================================
 * ESC
 * ========================================================================== */
static void ESC_SetPulse(uint16_t pulse_us)
{
    if (pulse_us < ESC_ABSOLUTE_MIN_US)
    {
        pulse_us = ESC_ABSOLUTE_MIN_US;
    }

    if (pulse_us > ESC_ABSOLUTE_MAX_US)
    {
        pulse_us = ESC_ABSOLUTE_MAX_US;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
    esc_current_us = pulse_us;
}

/* 0% -> 1500 us, 1~100% -> 1600~1800 us */
static uint16_t Motor_PercentToEscPulse(uint8_t percent)
{
    uint32_t range;
    uint32_t pulse;

    if (percent == 0U)
    {
        return ESC_NEUTRAL_US;
    }

    if (percent > 100U)
    {
        percent = 100U;
    }

    range = (uint32_t)(ESC_DRIVE_MAX_US - ESC_DRIVE_MIN_US);

    pulse =
        (uint32_t)ESC_DRIVE_MIN_US +
        (((uint32_t)(percent - 1U) * range) / 99U);

    if (pulse > ESC_DRIVE_MAX_US)
    {
        pulse = ESC_DRIVE_MAX_US;
    }

    return (uint16_t)pulse;
}

/* Convert actual ESC pulse back to 0~100% for MOTOR_STATUS Byte0 */
static uint8_t ESC_PulseToPercent(uint16_t pulse_us)
{
    uint32_t range;
    uint32_t offset;
    uint32_t percent;

    if (pulse_us < ESC_DRIVE_MIN_US)
    {
        return 0U;
    }

    if (pulse_us >= ESC_DRIVE_MAX_US)
    {
        return 100U;
    }

    range = (uint32_t)(ESC_DRIVE_MAX_US - ESC_DRIVE_MIN_US);
    offset = (uint32_t)(pulse_us - ESC_DRIVE_MIN_US);

    percent = 1U + ((offset * 99U) / range);

    if (percent > 100U)
    {
        percent = 100U;
    }

    return (uint8_t)percent;
}

/* ==========================================================================
 * CAN FILTER
 * Receive all standard CAN frames into FIFO0.
 * Actual IDs are selected in software.
 * ========================================================================== */
static void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    filter.FilterIdHigh = 0x0000U;
    filter.FilterIdLow = 0x0000U;
    filter.FilterMaskIdHigh = 0x0000U;
    filter.FilterMaskIdLow = 0x0000U;

    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ==========================================================================
 * PEDAL_STATUS 0x100
 * ========================================================================== */
static void CAN_ParsePedal(const uint8_t data[8])
{
    uint32_t now = HAL_GetTick();
    uint8_t new_alive = data[7];

    /* Reject invalid application values safely */
    if ((data[0] > 100U) || (data[1] > 100U))
    {
        pedal_accel = 0U;
        pedal_brake = 0U;
        pedal_sensor_status = 0xFFU;
    }
    else
    {
        pedal_accel = data[0];
        pedal_brake = data[1];
        pedal_sensor_status = data[6];
    }

    pedal_accel_rate =
        (int16_t)(((uint16_t)data[3] << 8) | (uint16_t)data[2]);

    pedal_brake_rate =
        (int16_t)(((uint16_t)data[5] << 8) | (uint16_t)data[4]);

    pedal_alive = new_alive;

    pedal_received = 1U;
    pedal_last_rx_tick = now;

    if (pedal_alive_initialized == 0U)
    {
        pedal_alive_initialized = 1U;
        pedal_previous_alive = new_alive;
        pedal_last_alive_change_tick = now;
    }
    else if (new_alive != pedal_previous_alive)
    {
        pedal_previous_alive = new_alive;
        pedal_last_alive_change_tick = now;
    }
}

/* ==========================================================================
 * MOTOR_COMMAND 0x120
 *
 * Accepted only when:
 * - Valid flag = 1
 * - reserved flag bits = 0
 * - mode = 0..2
 * - output limit = 0..100
 * - risk level = 0..3
 * - reserved bytes B4~B6 = 0
 * ========================================================================== */
static void CAN_ParseMotorCommand(const uint8_t data[8])
{
    uint32_t now = HAL_GetTick();

    uint8_t mode = data[0];
    uint8_t limit = data[1];
    uint8_t risk = data[2];
    uint8_t flags = data[3];
    uint8_t new_alive = data[7];

    if ((flags & MOTOR_CMD_FLAG_VALID) == 0U)
    {
        return;
    }

    if ((flags & (uint8_t)(~MOTOR_CMD_FLAG_ALLOWED_MASK)) != 0U)
    {
        return;
    }

    if (mode > MOTOR_MODE_STOP)
    {
        return;
    }

    if (limit > 100U)
    {
        return;
    }

    if (risk > 3U)
    {
        return;
    }

    if ((data[4] != 0x00U) ||
        (data[5] != 0x00U) ||
        (data[6] != 0x00U))
    {
        return;
    }

    command_mode = mode;
    command_output_limit = limit;
    command_risk_level = risk;
    command_flags = flags;
    command_alive = new_alive;

    command_received = 1U;
    command_last_rx_tick = now;

    if (command_alive_initialized == 0U)
    {
        command_alive_initialized = 1U;
        command_previous_alive = new_alive;
        command_last_alive_change_tick = now;
    }
    else if (new_alive != command_previous_alive)
    {
        command_previous_alive = new_alive;
        command_last_alive_change_tick = now;
    }

    /*
     * This is the last accepted MOTOR_COMMAND Counter.
     * MOTOR_STATUS Byte6 echoes this counter after application.
     */
    applied_command_counter = new_alive;

    /*
     * Immediate fail-safe actions:
     * STOP / E-STOP do not wait for the next 10 ms motor-control tick.
     */
    if ((flags & MOTOR_CMD_FLAG_ESTOP) != 0U)
    {
        ESC_SetPulse(ESC_NEUTRAL_US);
        actual_pwm_percent = 0U;
    }
    else if (mode == MOTOR_MODE_STOP)
    {
        ESC_SetPulse(ESC_NEUTRAL_US);
        actual_pwm_percent = 0U;
    }
}

/* ==========================================================================
 * CAN RX
 * ========================================================================== */
static void CAN_ProcessRx(void)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(
                &hcan1,
                CAN_RX_FIFO0,
                &rxHeader,
                rxData) != HAL_OK)
        {
            return;
        }

        if (rxHeader.IDE != CAN_ID_STD)
        {
            continue;
        }

        if (rxHeader.RTR != CAN_RTR_DATA)
        {
            continue;
        }

        if (rxHeader.DLC != 8U)
        {
            continue;
        }

        if (rxHeader.StdId == CAN_ID_PEDAL_STATUS)
        {
            CAN_ParsePedal(rxData);
        }
        else if (rxHeader.StdId == CAN_ID_MOTOR_COMMAND)
        {
            CAN_ParseMotorCommand(rxData);
        }
    }
}

/* ==========================================================================
 * MOTOR_STATUS Byte5 fault flags
 * ========================================================================== */
static uint8_t Motor_UpdateFaultFlags(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t faults = 0U;

    if (motor_driver_fault_active != 0U)
    {
        faults |= MOTOR_FAULT_DRIVER;
    }

    /*
     * PEDAL_STATUS communication fault.
     * Missing frames OR stuck Alive Counter are both represented as
     * PEDAL_STATUS Timeout because the published MOTOR_STATUS has one bit.
     */
    if ((pedal_received == 0U) ||
        ((now - pedal_last_rx_tick) > PEDAL_TIMEOUT_MS))
    {
        faults |= MOTOR_FAULT_PEDAL_TIMEOUT;
    }
    else if ((pedal_alive_initialized != 0U) &&
             ((now - pedal_last_alive_change_tick) > ALIVE_STUCK_TIMEOUT_MS))
    {
        faults |= MOTOR_FAULT_PEDAL_TIMEOUT;
    }

    /*
     * MOTOR_COMMAND communication fault.
     */
    if ((command_received == 0U) ||
        ((now - command_last_rx_tick) > MOTOR_COMMAND_TIMEOUT_MS))
    {
        faults |= MOTOR_FAULT_COMMAND_TIMEOUT;
    }
    else if ((command_alive_initialized != 0U) &&
             ((now - command_last_alive_change_tick) > ALIVE_STUCK_TIMEOUT_MS))
    {
        faults |= MOTOR_FAULT_COMMAND_TIMEOUT;
    }

    /*
     * E-STOP is active while MOTOR_COMMAND Byte3 bit1 = 1.
     * No separate latch/clear protocol is invented here.
     */
    if ((command_received != 0U) &&
        ((command_flags & MOTOR_CMD_FLAG_ESTOP) != 0U))
    {
        faults |= MOTOR_FAULT_ESTOP_ACTIVE;
    }

    motor_fault_flags = faults;
    return faults;
}

/* ==========================================================================
 * Calculate final output from PEDAL_STATUS + MOTOR_COMMAND
 * ========================================================================== */
static uint8_t Motor_CalculateOutputPercent(void)
{
    uint8_t output;

    applied_output_limit = 0U;

    /*
     * Communication / motor driver / E-STOP fault.
     */
    if (Motor_UpdateFaultFlags() != 0U)
    {
        motor_state = MOTOR_STATE_FAULT;
        return 0U;
    }

    /*
     * Pedal sensor status:
     * 0x00 = Normal.
     *
     * The published MOTOR_STATUS fault byte has no dedicated pedal-sensor
     * fault bit, so do not invent a new bit. We still fail-safe to zero
     * and report Motor State = Fault.
     */
    if (pedal_sensor_status != PEDAL_STATUS_NORMAL)
    {
        motor_state = MOTOR_STATE_FAULT;
        return 0U;
    }

    /*
     * Brake overrides accelerator.
     */
    if (pedal_brake >= BRAKE_THRESHOLD_PERCENT)
    {
        motor_state = MOTOR_STATE_STOP;
        return 0U;
    }

    /*
     * Accelerator released.
     */
    if (pedal_accel <= ACCEL_DEADBAND_PERCENT)
    {
        if (command_mode == MOTOR_MODE_NORMAL)
        {
            applied_output_limit = 100U;
        }
        else if (command_mode == MOTOR_MODE_OUTPUT_LIMIT)
        {
            applied_output_limit = command_output_limit;
        }
        else
        {
            applied_output_limit = 0U;
        }

        motor_state = MOTOR_STATE_STOP;
        return 0U;
    }

    switch (command_mode)
    {
        case MOTOR_MODE_NORMAL:

            applied_output_limit = 100U;
            motor_state = MOTOR_STATE_RUNNING;

            return pedal_accel;

        case MOTOR_MODE_OUTPUT_LIMIT:

            applied_output_limit = command_output_limit;

            output = pedal_accel;

            if (output > command_output_limit)
            {
                output = command_output_limit;
            }

            if (output == 0U)
            {
                motor_state = MOTOR_STATE_STOP;
                return 0U;
            }

            motor_state = MOTOR_STATE_OUTPUT_LIMITED;

            return output;

        case MOTOR_MODE_STOP:

            applied_output_limit = 0U;
            motor_state = MOTOR_STATE_STOP;

            return 0U;

        default:

            motor_state = MOTOR_STATE_FAULT;
            return 0U;
    }
}

/* ==========================================================================
 * Motor control update - every 10 ms
 * ========================================================================== */
static void Motor_Update(void)
{
    uint8_t output_percent;
    uint16_t target;
    uint16_t next;

    output_percent = Motor_CalculateOutputPercent();
    requested_output_percent = output_percent;

    target = Motor_PercentToEscPulse(output_percent);
    esc_target_us = target;

    /*
     * Fault / STOP / brake / accelerator released -> neutral immediately.
     */
    if (target == ESC_NEUTRAL_US)
    {
        ESC_SetPulse(ESC_NEUTRAL_US);
        actual_pwm_percent = 0U;
        return;
    }

    /*
     * Soft acceleration only.
     */
    if (esc_current_us < target)
    {
        next = (uint16_t)(esc_current_us + ESC_RAMP_STEP_US);

        if (next > target)
        {
            next = target;
        }

        ESC_SetPulse(next);
    }
    else if (esc_current_us > target)
    {
        /*
         * Output reduction is immediate.
         */
        ESC_SetPulse(target);
    }
    else
    {
        ESC_SetPulse(target);
    }

    /*
     * Actual PWM is based on the pulse actually being applied now.
     */
    actual_pwm_percent = ESC_PulseToPercent(esc_current_us);
}

/* ==========================================================================
 * MOTOR_STATUS 0x210 TX
 * ========================================================================== */
static void Motor_SendStatus(void)
{
    CAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[8];
    uint32_t txMailbox;

    txHeader.StdId = CAN_ID_MOTOR_STATUS;
    txHeader.ExtId = 0U;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8U;
    txHeader.TransmitGlobalTime = DISABLE;

    /* B0 Actual PWM */
    txData[0] = actual_pwm_percent;

    /* B1 Applied Output Limit */
    txData[1] = applied_output_limit;

    /* B2~3 Vehicle Speed, Little Endian */
    txData[2] = (uint8_t)(vehicle_speed_raw & 0xFFU);
    txData[3] = (uint8_t)((vehicle_speed_raw >> 8) & 0xFFU);

    /* B4 Motor State */
    txData[4] = motor_state;

    /* B5 Fault Flags */
    txData[5] = motor_fault_flags;

    /* B6 Last Applied MOTOR_COMMAND Counter */
    txData[6] = applied_command_counter;

    /* B7 MOTOR_STATUS Alive Counter */
    txData[7] = motor_status_alive_counter;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
    {
        if (HAL_CAN_AddTxMessage(
                &hcan1,
                &txHeader,
                txData,
                &txMailbox) == HAL_OK)
        {
            motor_status_alive_counter++;
        }
    }
}

/* USER CODE END 0 */

/* ==========================================================================
 * MAIN
 * ========================================================================== */
int main(void)
{
    uint32_t motor_tick;
    uint32_t status_tick;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();

    /*
     * No 5-second startup 1500 us hold.
     * Initialize CAN and PWM, then enter normal control immediately.
     */
    MX_TIM1_Init();
    MX_CAN1_Init();
    CAN_Filter_Config();

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    motor_tick = HAL_GetTick();
    status_tick = HAL_GetTick();

    while (1)
    {
        CAN_ProcessRx();

        if ((HAL_GetTick() - motor_tick) >= MOTOR_CONTROL_PERIOD_MS)
        {
            motor_tick = HAL_GetTick();
            Motor_Update();
        }

        if ((HAL_GetTick() - status_tick) >= MOTOR_STATUS_PERIOD_MS)
        {
            status_tick = HAL_GetTick();
            Motor_SendStatus();
        }
    }
}

/* ==========================================================================
 * CAN1 INIT : 500 kbps
 *
 * System clock:
 * APB1 = 42 MHz
 *
 * 42 MHz / [6 * (1 + 11 + 2)] = 500 kbps
 * ========================================================================== */
static void MX_CAN1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();

    /*
     * PA11 CAN1_RX
     * PA12 CAN1_TX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/* ==========================================================================
 * TIM1 PWM INIT : PA8 / 50 Hz
 *
 * TIM1 clock = 84 MHz
 * PSC = 83     -> 1 MHz
 * ARR = 19999  -> 20 ms -> 50 Hz
 * ========================================================================== */
static void MX_TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    htim1.Instance = TIM1;

    htim1.Init.Prescaler = 83;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 19999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = ESC_NEUTRAL_US;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(
            &htim1,
            &sConfigOC,
            TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * PA8 = TIM1_CH1
     *
     * Configure directly to preserve the PWM behavior already confirmed.
     */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ==========================================================================
 * GPIO INIT
 * ========================================================================== */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* ==========================================================================
 * SYSTEM CLOCK : 84 MHz
 *
 * HSI = 16 MHz
 * PLLM = 16
 * PLLN = 336
 * PLLP = 4
 *
 * SYSCLK = 84 MHz
 * APB1   = 42 MHz
 * APB2   = 84 MHz
 * ========================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE2
    );

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ==========================================================================
 * ERROR HANDLER
 * ========================================================================== */
void Error_Handler(void)
{
    /*
     * If TIM1 is already active, keep ESC at neutral.
     */
    if (htim1.Instance == TIM1)
    {
        __HAL_TIM_SET_COMPARE(
            &htim1,
            TIM_CHANNEL_1,
            ESC_NEUTRAL_US
        );
    }

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
