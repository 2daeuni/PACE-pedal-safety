/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FINAL Motor ECU
  *
  * Board           : NUCLEO-F446RE
  *
  * CAN bus         : CAN1, 500 kbps
  * CAN1_RX         : PA11
  * CAN1_TX         : PA12
  *
  * ESC PWM         : PA8 / TIM1_CH1
  * PWM frequency   : 50 Hz
  * Neutral         : 1500 us
  *
  * CAN IDs
  * 0x100 : Pedal ECU  -> CAN bus
  * 0x120 : Safety ECU -> Motor ECU
  * 0x210 : Motor ECU  -> Safety/UI status
  *
  * IMPORTANT
  * - Motor ECU requires BOTH Pedal ECU and Safety ECU messages.
  * - Any timeout/fault/brake/E-STOP forces ESC Neutral immediately.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

/* USER CODE BEGIN PV */

/* ========================================================================== */
/* CAN ID / PERIOD                                                            */
/* ========================================================================== */

#define CAN_ID_PEDAL_STATUS              0x100U
#define CAN_ID_SAFETY_MOTOR_CMD          0x120U
#define CAN_ID_MOTOR_STATUS              0x210U

#define PEDAL_EXPECTED_PERIOD_MS         10U
#define SAFETY_EXPECTED_PERIOD_MS        20U
#define MOTOR_STATUS_PERIOD_MS           20U

#define PEDAL_TIMEOUT_MS                 200U
#define SAFETY_TIMEOUT_MS                200U
#define ALIVE_STUCK_TIMEOUT_MS           200U

/* ========================================================================== */
/* PEDAL ECU : 0x100 / DLC 8                                                  */
/* ========================================================================== */
/*
 * Byte 0 : Accelerator [%]            uint8_t, 0~100
 * Byte 1 : Brake [%]                  uint8_t, 0~100
 * Byte 2 : Accel rate LSB             int16 little-endian, 0.1 %/s
 * Byte 3 : Accel rate MSB
 * Byte 4 : Brake rate LSB             int16 little-endian, 0.1 %/s
 * Byte 5 : Brake rate MSB
 * Byte 6 : Sensor Status
 * Byte 7 : Alive Counter
 *
 * Sensor Status
 * 0x01 : Normal
 * 0x02 : Accelerator ADC range error
 * 0x04 : Brake ADC range error
 * 0x08 : Accelerator disconnected
 * 0x10 : Brake disconnected
 * 0x20 : Accelerator calibration error
 * 0x40 : Brake calibration error
 * 0x80 : Reserved
 */

#define PEDAL_STATUS_NORMAL              0x01U

/* ========================================================================== */
/* SAFETY ECU -> MOTOR ECU : 0x120 / DLC 8                                    */
/* ========================================================================== */
/*
 * Byte 0 : Command
 *          0x00 STOP
 *          0x01 DRIVE_ALLOW
 *          0x02 BRAKE
 *          0x03 EMERGENCY_STOP
 *          0x04 CLEAR_ESTOP
 *
 * Byte 1 : Output limit [%]            0~100
 *          Final motor demand = MIN(Pedal accelerator, Output limit)
 *
 * Byte 2 : Safety Status
 *          0x01 = Normal
 *          Other = Fault -> Motor STOP
 *
 * Byte 3 : Safety Alive Counter
 *
 * Byte 4 : Safety Level
 *          0 = Normal
 *          1 = Warning
 *          2 = Intervention
 *          3 = Emergency
 *
 * Byte 5 : Reason Code                 Safety ECU-defined reason
 * Byte 6 : Reserved                    0
 * Byte 7 : Reserved                    0
 */

#define SAFETY_CMD_STOP                  0x00U
#define SAFETY_CMD_DRIVE_ALLOW           0x01U
#define SAFETY_CMD_BRAKE                 0x02U
#define SAFETY_CMD_EMERGENCY_STOP        0x03U
#define SAFETY_CMD_CLEAR_ESTOP           0x04U

#define SAFETY_STATUS_NORMAL             0x01U

/* ========================================================================== */
/* MOTOR ECU -> SAFETY/UI : 0x210 / DLC 8                                     */
/* ========================================================================== */
/*
 * Byte 0 : Motor State
 *          0 STOP
 *          1 DRIVE
 *          2 BRAKE
 *          3 ESTOP
 *          4 FAULT
 *
 * Byte 1 : Pedal accelerator [%]
 * Byte 2 : Applied motor output [%]
 * Byte 3 : Pedal sensor status
 * Byte 4 : Safety command
 * Byte 5 : Safety status
 * Byte 6 : Motor fault flags
 * Byte 7 : Motor ECU alive counter
 *
 * Motor Fault Flags
 * bit0 0x01 : Pedal CAN timeout
 * bit1 0x02 : Safety CAN timeout
 * bit2 0x04 : Pedal alive stuck
 * bit3 0x08 : Safety alive stuck
 * bit4 0x10 : Pedal sensor fault
 * bit5 0x20 : Safety status fault
 * bit6 0x40 : Emergency stop latched
 * bit7 0x80 : Reserved
 */

#define MOTOR_STATE_STOP                 0U
#define MOTOR_STATE_DRIVE                1U
#define MOTOR_STATE_BRAKE                2U
#define MOTOR_STATE_ESTOP                3U
#define MOTOR_STATE_FAULT                4U

#define MOTOR_FAULT_PEDAL_TIMEOUT        0x01U
#define MOTOR_FAULT_SAFETY_TIMEOUT       0x02U
#define MOTOR_FAULT_PEDAL_ALIVE          0x04U
#define MOTOR_FAULT_SAFETY_ALIVE         0x08U
#define MOTOR_FAULT_PEDAL_STATUS         0x10U
#define MOTOR_FAULT_SAFETY_STATUS        0x20U
#define MOTOR_FAULT_ESTOP_LATCHED        0x40U

/* ========================================================================== */
/* MOTOR / ESC                                                                */
/* ========================================================================== */

#define ACCEL_DEADBAND_PERCENT           3U
#define BRAKE_THRESHOLD_PERCENT          5U

#define ESC_NEUTRAL_US                   1500U

/*
 * 실제 차량 ESC dead-band를 고려한 구동 시작값.
 * 차량이 1600 us에서 돌지 않고 1700 us부터 돌면
 * ESC_DRIVE_MIN_US만 1700U로 변경.
 */
#define ESC_DRIVE_MIN_US                 1600U
#define ESC_DRIVE_MAX_US                 1800U

#define ESC_ABSOLUTE_MIN_US              1000U
#define ESC_ABSOLUTE_MAX_US              2000U

#define MOTOR_CONTROL_PERIOD_MS          10U
#define ESC_RAMP_STEP_US                 5U

/* ========================================================================== */
/* RECEIVED PEDAL DATA                                                        */
/* ========================================================================== */

volatile uint8_t  pedal_accel = 0U;
volatile uint8_t  pedal_brake = 0U;
volatile int16_t  pedal_accel_rate = 0;
volatile int16_t  pedal_brake_rate = 0;
volatile uint8_t  pedal_status = 0x00U;
volatile uint8_t  pedal_alive = 0U;

volatile uint8_t  pedal_received = 0U;
volatile uint8_t  pedal_alive_initialized = 0U;
volatile uint8_t  pedal_previous_alive = 0U;

volatile uint32_t pedal_last_rx_tick = 0U;
volatile uint32_t pedal_last_alive_change_tick = 0U;

/* ========================================================================== */
/* RECEIVED SAFETY DATA                                                       */
/* ========================================================================== */

volatile uint8_t safety_command = SAFETY_CMD_STOP;
volatile uint8_t safety_output_limit = 0U;
volatile uint8_t safety_status = 0x00U;
volatile uint8_t safety_alive = 0U;
volatile uint8_t safety_level = 0U;
volatile uint8_t safety_reason = 0U;

volatile uint8_t safety_received = 0U;
volatile uint8_t safety_alive_initialized = 0U;
volatile uint8_t safety_previous_alive = 0U;

volatile uint32_t safety_last_rx_tick = 0U;
volatile uint32_t safety_last_alive_change_tick = 0U;

/* ========================================================================== */
/* MOTOR STATE                                                                */
/* ========================================================================== */

volatile uint8_t  motor_state = MOTOR_STATE_STOP;
volatile uint8_t  motor_fault_flags = 0U;
volatile uint8_t  motor_output_percent = 0U;
volatile uint8_t  motor_alive_counter = 0U;

volatile uint8_t  estop_latched = 0U;

volatile uint16_t esc_target_us = ESC_NEUTRAL_US;
volatile uint16_t esc_current_us = ESC_NEUTRAL_US;

/* USER CODE END PV */


/* USER CODE BEGIN PFP */

static void CAN_Filter_Config(void);
static void CAN_ProcessRx(void);
static void CAN_ParsePedal(const uint8_t data[8]);
static void CAN_ParseSafety(const uint8_t data[8]);
static void Motor_SendStatus(void);

static void ESC_SetPulse(uint16_t pulse_us);
static uint8_t Motor_UpdateFaultFlags(void);
static uint8_t Motor_GetAllowedOutputPercent(void);
static uint16_t Motor_PercentToEscPulse(uint8_t percent);
static void Motor_Update(void);

/* USER CODE END PFP */


/* USER CODE BEGIN 0 */

/* ========================================================================== */
/* ESC                                                                        */
/* ========================================================================== */

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


/* ========================================================================== */
/* CAN FILTER                                                                 */
/* ========================================================================== */

static void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef filter = {0};

    /*
     * 0x100, 0x120을 둘 다 받기 위해 CAN1 Standard frame을
     * FIFO0로 받은 뒤 Software에서 필요한 ID만 처리한다.
     */
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


/* ========================================================================== */
/* PEDAL PARSE                                                                */
/* ========================================================================== */

static void CAN_ParsePedal(const uint8_t data[8])
{
    uint32_t now = HAL_GetTick();
    uint8_t new_alive = data[7];

    pedal_accel = data[0];
    pedal_brake = data[1];

    if (pedal_accel > 100U)
    {
        pedal_accel = 100U;
    }

    if (pedal_brake > 100U)
    {
        pedal_brake = 100U;
    }

    pedal_accel_rate =
        (int16_t)(((uint16_t)data[3] << 8) | (uint16_t)data[2]);

    pedal_brake_rate =
        (int16_t)(((uint16_t)data[5] << 8) | (uint16_t)data[4]);

    pedal_status = data[6];
    pedal_alive = new_alive;

    pedal_last_rx_tick = now;
    pedal_received = 1U;

    if (pedal_alive_initialized == 0U)
    {
        pedal_previous_alive = new_alive;
        pedal_last_alive_change_tick = now;
        pedal_alive_initialized = 1U;
    }
    else if (new_alive != pedal_previous_alive)
    {
        pedal_previous_alive = new_alive;
        pedal_last_alive_change_tick = now;
    }
}


/* ========================================================================== */
/* SAFETY PARSE                                                               */
/* ========================================================================== */

static void CAN_ParseSafety(const uint8_t data[8])
{
    uint32_t now = HAL_GetTick();
    uint8_t new_alive = data[3];

    safety_command = data[0];

    safety_output_limit = data[1];
    if (safety_output_limit > 100U)
    {
        safety_output_limit = 100U;
    }

    safety_status = data[2];
    safety_alive = new_alive;
    safety_level = data[4];
    safety_reason = data[5];

    safety_last_rx_tick = now;
    safety_received = 1U;

    if (safety_alive_initialized == 0U)
    {
        safety_previous_alive = new_alive;
        safety_last_alive_change_tick = now;
        safety_alive_initialized = 1U;
    }
    else if (new_alive != safety_previous_alive)
    {
        safety_previous_alive = new_alive;
        safety_last_alive_change_tick = now;
    }

    /*
     * Emergency stop은 Motor ECU 내부에서 latch.
     */
    if (safety_command == SAFETY_CMD_EMERGENCY_STOP)
    {
        estop_latched = 1U;
    }

    /*
     * E-STOP 해제는:
     * - Safety ECU가 CLEAR_ESTOP 명령
     * - Safety status 정상
     * - 가속페달이 dead-band 이하
     * 조건에서만 허용.
     */
    if ((safety_command == SAFETY_CMD_CLEAR_ESTOP) &&
        (safety_status == SAFETY_STATUS_NORMAL) &&
        (pedal_accel <= ACCEL_DEADBAND_PERCENT))
    {
        estop_latched = 0U;
        safety_command = SAFETY_CMD_STOP;
    }
}


/* ========================================================================== */
/* CAN RX POLLING                                                             */
/* ========================================================================== */

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
        else if (rxHeader.StdId == CAN_ID_SAFETY_MOTOR_CMD)
        {
            CAN_ParseSafety(rxData);
        }
        else
        {
            /* Ignore unrelated CAN IDs */
        }
    }
}


/* ========================================================================== */
/* MOTOR FAULT EVALUATION                                                     */
/* ========================================================================== */

static uint8_t Motor_UpdateFaultFlags(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t faults = 0U;

    if ((pedal_received == 0U) ||
        ((now - pedal_last_rx_tick) > PEDAL_TIMEOUT_MS))
    {
        faults |= MOTOR_FAULT_PEDAL_TIMEOUT;
    }

    if ((safety_received == 0U) ||
        ((now - safety_last_rx_tick) > SAFETY_TIMEOUT_MS))
    {
        faults |= MOTOR_FAULT_SAFETY_TIMEOUT;
    }

    if ((pedal_alive_initialized != 0U) &&
        ((now - pedal_last_alive_change_tick) > ALIVE_STUCK_TIMEOUT_MS))
    {
        faults |= MOTOR_FAULT_PEDAL_ALIVE;
    }

    if ((safety_alive_initialized != 0U) &&
        ((now - safety_last_alive_change_tick) > ALIVE_STUCK_TIMEOUT_MS))
    {
        faults |= MOTOR_FAULT_SAFETY_ALIVE;
    }

    if ((pedal_received != 0U) &&
        (pedal_status != PEDAL_STATUS_NORMAL))
    {
        faults |= MOTOR_FAULT_PEDAL_STATUS;
    }

    if ((safety_received != 0U) &&
        (safety_status != SAFETY_STATUS_NORMAL))
    {
        faults |= MOTOR_FAULT_SAFETY_STATUS;
    }

    if (estop_latched != 0U)
    {
        faults |= MOTOR_FAULT_ESTOP_LATCHED;
    }

    motor_fault_flags = faults;
    return faults;
}


/* ========================================================================== */
/* PEDAL + SAFETY -> FINAL MOTOR OUTPUT                                       */
/* ========================================================================== */

static uint8_t Motor_GetAllowedOutputPercent(void)
{
    uint8_t requested;
    uint8_t limited;

    /*
     * 모든 필수 ECU 통신/상태 검사.
     */
    if (Motor_UpdateFaultFlags() != 0U)
    {
        if (estop_latched != 0U)
        {
            motor_state = MOTOR_STATE_ESTOP;
        }
        else
        {
            motor_state = MOTOR_STATE_FAULT;
        }

        return 0U;
    }

    /*
     * 운전자 브레이크가 최우선.
     */
    if (pedal_brake >= BRAKE_THRESHOLD_PERCENT)
    {
        motor_state = MOTOR_STATE_BRAKE;
        return 0U;
    }

    /*
     * Safety ECU 명령 우선.
     */
    if (safety_command == SAFETY_CMD_EMERGENCY_STOP)
    {
        estop_latched = 1U;
        motor_fault_flags |= MOTOR_FAULT_ESTOP_LATCHED;
        motor_state = MOTOR_STATE_ESTOP;
        return 0U;
    }

    if (safety_command == SAFETY_CMD_BRAKE)
    {
        motor_state = MOTOR_STATE_BRAKE;
        return 0U;
    }

    if (safety_command == SAFETY_CMD_STOP)
    {
        motor_state = MOTOR_STATE_STOP;
        return 0U;
    }

    if (safety_command == SAFETY_CMD_CLEAR_ESTOP)
    {
        motor_state = MOTOR_STATE_STOP;
        return 0U;
    }

    if (safety_command != SAFETY_CMD_DRIVE_ALLOW)
    {
        motor_state = MOTOR_STATE_FAULT;
        return 0U;
    }

    /*
     * Accelerator dead-band.
     */
    if (pedal_accel <= ACCEL_DEADBAND_PERCENT)
    {
        motor_state = MOTOR_STATE_STOP;
        return 0U;
    }

    /*
     * Driver request.
     */
    requested = pedal_accel;

    /*
     * Safety output limit.
     *
     * 예:
     * Pedal 80%, Safety limit 40% -> Motor 40%
     * Pedal 20%, Safety limit 40% -> Motor 20%
     */
    limited = requested;

    if (limited > safety_output_limit)
    {
        limited = safety_output_limit;
    }

    if (limited == 0U)
    {
        motor_state = MOTOR_STATE_STOP;
        return 0U;
    }

    motor_state = MOTOR_STATE_DRIVE;
    return limited;
}


/* ========================================================================== */
/* OUTPUT % -> RC ESC PWM                                                     */
/* ========================================================================== */

static uint16_t Motor_PercentToEscPulse(uint8_t percent)
{
    uint32_t pulse;
    uint32_t range;

    if (percent == 0U)
    {
        return ESC_NEUTRAL_US;
    }

    if (percent > 100U)
    {
        percent = 100U;
    }

    /*
     * 1~100% -> ESC_DRIVE_MIN_US ~ ESC_DRIVE_MAX_US
     */
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


/* ========================================================================== */
/* MOTOR UPDATE                                                               */
/* ========================================================================== */

static void Motor_Update(void)
{
    uint8_t allowed_percent;
    uint16_t target;
    uint16_t next;

    allowed_percent = Motor_GetAllowedOutputPercent();
    motor_output_percent = allowed_percent;

    target = Motor_PercentToEscPulse(allowed_percent);
    esc_target_us = target;

    /*
     * STOP / BRAKE / FAULT / ESTOP은 즉시 Neutral.
     */
    if (target == ESC_NEUTRAL_US)
    {
        ESC_SetPulse(ESC_NEUTRAL_US);
        return;
    }

    /*
     * 정상 DRIVE에서만 ramp 적용.
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
         * 가속페달을 놓거나 Safety limit가 낮아진 경우
         * 부드럽게 출력 감소.
         */
        if (esc_current_us > (ESC_NEUTRAL_US + ESC_RAMP_STEP_US))
        {
            next = (uint16_t)(esc_current_us - ESC_RAMP_STEP_US);
        }
        else
        {
            next = target;
        }

        if (next < target)
        {
            next = target;
        }

        ESC_SetPulse(next);
    }
    else
    {
        ESC_SetPulse(target);
    }
}


/* ========================================================================== */
/* MOTOR STATUS TX : 0x210                                                    */
/* ========================================================================== */

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

    txData[0] = motor_state;
    txData[1] = pedal_accel;
    txData[2] = motor_output_percent;
    txData[3] = pedal_status;
    txData[4] = safety_command;
    txData[5] = safety_status;
    txData[6] = motor_fault_flags;
    txData[7] = motor_alive_counter++;

    /*
     * Mailbox가 있을 때만 전송.
     * Status TX 실패가 실제 모터 제어를 막지는 않는다.
     */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
    {
        (void)HAL_CAN_AddTxMessage(
            &hcan1,
            &txHeader,
            txData,
            &txMailbox
        );
    }
}

/* USER CODE END 0 */


/**
  * @brief  Main
  */
int main(void)
{
    uint32_t motor_tick;
    uint32_t status_tick;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_TIM1_Init();

    /* USER CODE BEGIN 2 */

    /*
     * ESC PWM 시작 전 Neutral 세팅.
     */
    ESC_SetPulse(ESC_NEUTRAL_US);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * ESC가 Neutral을 인식할 시간.
     * STM32를 먼저 켜고 차량 ESC 전원을 켜는 것을 권장.
     */
    HAL_Delay(3000);

    CAN_Filter_Config();

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    motor_tick = HAL_GetTick();
    status_tick = HAL_GetTick();

    /* USER CODE END 2 */

    while (1)
    {
        /*
         * CAN RX는 polling.
         * RX interrupt 설정이 없어도 동작.
         */
        CAN_ProcessRx();

        /*
         * Motor control : 10 ms
         */
        if ((HAL_GetTick() - motor_tick) >= MOTOR_CONTROL_PERIOD_MS)
        {
            motor_tick = HAL_GetTick();
            Motor_Update();
        }

        /*
         * Motor Status : 20 ms
         */
        if ((HAL_GetTick() - status_tick) >= MOTOR_STATUS_PERIOD_MS)
        {
            status_tick = HAL_GetTick();
            Motor_SendStatus();
        }
    }
}


/* ========================================================================== */
/* CAN1 INIT : 500 kbps                                                       */
/* ========================================================================== */

static void MX_CAN1_Init(void)
{
    hcan1.Instance = CAN1;

    /*
     * APB1 = 45 MHz
     *
     * 45 MHz / [5 * (1 + 15 + 2)]
     * = 500 kbps
     *
     * Sample Point = (1 + 15) / 18 = 88.9%
     */
    hcan1.Init.Prescaler = 5;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
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


/* ========================================================================== */
/* TIM1 INIT : PA8 / CH1 / 50 Hz                                              */
/* ========================================================================== */

static void MX_TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /*
     * APB2 timer clock = 180 MHz
     *
     * 180 MHz / (179 + 1) = 1 MHz
     * 1 count = 1 us
     *
     * ARR = 19999
     * Period = 20000 us = 20 ms = 50 Hz
     */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 179;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 19999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim1,
            &sMasterConfig) != HAL_OK)
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

    HAL_TIM_MspPostInit(&htim1);
}


/* ========================================================================== */
/* GPIO INIT                                                                  */
/* ========================================================================== */

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}


/* ========================================================================== */
/* SYSTEM CLOCK : 180 MHz                                                     */
/* ========================================================================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );

    /*
     * HSI = 16 MHz
     * PLLM = 8
     * PLLN = 180
     * PLLP = 2
     *
     * SYSCLK = 180 MHz
     */
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

    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
        RCC_HCLK_DIV4;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* ERROR HANDLER                                                              */
/* ========================================================================== */

void Error_Handler(void)
{
    /*
     * TIM1이 초기화된 뒤 오류가 발생했다면 ESC Neutral.
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
