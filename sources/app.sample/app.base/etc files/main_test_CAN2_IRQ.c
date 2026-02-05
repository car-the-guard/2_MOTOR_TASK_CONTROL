// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
* FileName : main.c (Fixed Build Errors)
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <gpio.h>
#include <pdm.h>
#include <string.h>

// [CAN & ICTC Drivers]
#include "can_config.h"
#include "can_reg.h"  // [수정] 레지스터 타입 정의 포함
#include "can.h"
#include "can_drv.h"
#include "ictc.h" 

/*
***************************************************************************************************
* DEFINES
***************************************************************************************************
*/
// CAN Config
#define CAN_RX_QUEUE_SIZE       (64)
#define CAN_CH_CMD              (0)
#define CAN_ID_DRIVE_CMD        (0x106)  
#define CAN_ID_SPEED_FEEDBACK   (0x13)  

// ICTC & Encoder Config
#define ICTC_CH_ENC_L           (0)     
#define ICTC_CH_ENC_R           (1)     
#define ENCODER_PPR             (100.0f)
#define GEAR_RATIO              (1.0f)  
#define WHEEL_RADIUS_CM         (3.0f)  
#define PI                      (3.141592f)

// Task Configuration
#define TASK_CAN_PRIO           (SAL_PRIO_APP_CFG + 5)
#define TASK_MOTOR_PRIO         (SAL_PRIO_APP_CFG + 2)
#define TASK_STK_SIZE           (1024)

// Hardware Pinmap
#define MOTOR_L_PWM_CH          (0)
#define MOTOR_L_IN1             GPIO_GPB(23)
#define MOTOR_L_IN2             GPIO_GPB(21)
#define MOTOR_R_PWM_CH          (1)
#define MOTOR_R_IN1             GPIO_GPB(24)
#define MOTOR_R_IN2             GPIO_GPB(22)
#define PWM_PERIOD_NS           (1000000UL)
#define MAX_DUTY_SCALE          (1000UL)
#define TURN_SENSITIVITY        (1.0f)
#define USE_PMM_MONITOR         (0UL)

// 중복 정의 방지
#ifndef GPIO_PERICH_CH0
#define GPIO_PERICH_CH0         (0UL)
#endif

// Rate Limiter
#define STEP_VAL                (2.0f)

/*
***************************************************************************************************
* DATA STRUCTURES
***************************************************************************************************
*/
typedef struct
{
    uint32 id;      // uint32_t -> uint32
    uint32 dlc;     
    uint8  data[8]; // uint8_t -> uint8
} CAN_RxPacket_t;

/*
***************************************************************************************************
* GLOBAL VARIABLES
***************************************************************************************************
*/
// [수정 1] console.c에서 참조하는 변수 복구
uint32 gALiveMsgOnOff = 1; 

static uint32 g_canRxQueueHandle = 0;

// Shared Control Variables
volatile int32 g_TargetF = 0;
volatile int32 g_TargetR = 0;

// Measured Speed (Feedback)
volatile short g_CurrentSpeedL = 0;
volatile short g_CurrentSpeedR = 0;

// Applied PWM Direction
volatile int32 g_AppliedPWM_L = 0;
volatile int32 g_AppliedPWM_R = 0;

// Rate Limit State
static float curF = 0.0f;
static float curR = 0.0f;

// Task Stacks
static uint32 TaskCanID;
static uint32 TaskCanStk[TASK_STK_SIZE];
static uint32 TaskMotorID;
static uint32 TaskMotorStk[TASK_STK_SIZE];

static uint32 g_LastDutyL = 0xFFFF;
static uint32 g_LastDutyR = 0xFFFF;

/*
***************************************************************************************************
* FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void Main_StartTask(void * pArg);
static void AppTaskCreate(void);
static void Task_CAN_Rx(void *pArg);
static void Task_MotorControl(void *pArg);

static void Init_CAN_HW(void);
static void Motor_HW_Init(void);
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale);
static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed);
static void Car_Update_Logic(void);

static void CAN_Send_Feedback(short speedL, short speedR);
static short Calculate_Speed_From_ICTC(uint32 channel, int32 current_pwm_val);

static float Clamp_Val(float x, float lo, float hi);
static float rate_limit(float cur, float target, float step);

// Callbacks
static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError);
static void CAN_AppCallbackTxEvent(uint8 ucCh, CANTxInterruptType_t uiIntType);
static void CAN_AppCallbackErrorEvent(uint8 ucCh, CANErrorType_t uiError);

// [수정 2] can_app.c와의 링크 에러 방지를 위한 더미 함수
void CAN_consume_rx_message(void *pMsg, void *pPayload); 

/*
***************************************************************************************************
* MAIN ENTRY
***************************************************************************************************
*/
void cmain (void)
{
    static uint32 AppTaskStartID = 0;
    static uint32 AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    SALRetCode_t err;

    (void)SAL_Init();
    BSP_PreInit();
    BSP_Init();

    mcu_printf("\n[MAIN] RC Car RTOS (Fixed Version)\n");

    err = (SALRetCode_t)SAL_TaskCreate(&AppTaskStartID, (const uint8 *)"StartTask", (SALTaskFunc)&Main_StartTask,
                                     &AppTaskStartStk[0], ACFG_TASK_MEDIUM_STK_SIZE, SAL_PRIO_APP_CFG, NULL);
    if (err == SAL_RET_SUCCESS) (void)SAL_OsStart();
}

/*
***************************************************************************************************
* TASKS
***************************************************************************************************
*/
static void Main_StartTask(void * pArg)
{
    (void)pArg;
    (void)SAL_OsInitFuncs();
    AppTaskCreate();
    while (1) { (void)SAL_TaskSleep(5000); }
}

static void Task_CAN_Rx(void *pArg)
{
    (void)pArg;
    CAN_RxPacket_t rxPacket;
    SALRetCode_t salRet;
    uint32 uiSizeCopied;

    mcu_printf("[Task] CAN Rx Listening...\n");

    while(1)
    {
        salRet = SAL_QueueGet(g_canRxQueueHandle, &rxPacket, &uiSizeCopied, 0, SAL_OPT_BLOCKING);

        if (salRet == SAL_RET_SUCCESS)
        {
            if (rxPacket.id == CAN_ID_DRIVE_CMD) // 0x12
            {
                g_TargetF = (char)rxPacket.data[0];
                g_TargetR = (char)rxPacket.data[3] - (char)rxPacket.data[2];
                // mcu_printf("[CMD] F:%d R:%d\n", g_TargetF, g_TargetR);
            }
        }
        else
        {
            SAL_TaskSleep(10);
        }
    }
}

// [핵심] 모터 제어 및 피드백 태스크
static void Task_MotorControl(void *pArg)
{
    (void)pArg;
    mcu_printf("[Task] Motor Control & Feedback Ready\n");

    while(1)
    {
        // 1. 모터 제어
        Car_Update_Logic();

        // 2. ICTC 엔코더 값 읽기
        g_CurrentSpeedL = Calculate_Speed_From_ICTC(ICTC_CH_ENC_L, g_AppliedPWM_L);
        g_CurrentSpeedR = Calculate_Speed_From_ICTC(ICTC_CH_ENC_R, g_AppliedPWM_R);

        // 3. CAN으로 속도 전송
        CAN_Send_Feedback(g_CurrentSpeedL, g_CurrentSpeedR);

        // 4. 주기 (20ms = 50Hz)
        SAL_TaskSleep(20); 
    }
}

static void AppTaskCreate(void)
{
    SALRetCode_t err;

    Motor_HW_Init();
    
    err = SAL_QueueCreate(&g_canRxQueueHandle, (const uint8 *)"CAN_Q", CAN_RX_QUEUE_SIZE, sizeof(CAN_RxPacket_t));
    if(err != SAL_RET_SUCCESS) mcu_printf("[ERR] Queue Fail\n");

    Init_CAN_HW();

    SAL_TaskCreate(&TaskCanID, (const uint8 *)"CAN Task", (SALTaskFunc)&Task_CAN_Rx, 
                   &TaskCanStk[0], TASK_STK_SIZE, TASK_CAN_PRIO, NULL);

    SAL_TaskCreate(&TaskMotorID, (const uint8 *)"Motor Task", (SALTaskFunc)&Task_MotorControl, 
                   &TaskMotorStk[0], TASK_STK_SIZE, TASK_MOTOR_PRIO, NULL);
}

/*
***************************************************************************************************
* ICTC & CAN Logic Functions
***************************************************************************************************
*/

static short Calculate_Speed_From_ICTC(uint32 channel, int32 current_pwm_val)
{
    uint32 period_cnt = 0;
    uint32 frequency_hz = 0;
    float speed_cm_s = 0.0f;

    if (current_pwm_val == 0) return 0;

    // 함수명 수정 (GetPrev -> GetPre)
    period_cnt = ICTC_GetPrePeriodCnt(channel);

    if (period_cnt > 0)
    {
        frequency_hz = ICTC_PERI_CLOCK / period_cnt;
    }
    else
    {
        return 0; 
    }

    float rps = (float)frequency_hz / (ENCODER_PPR * GEAR_RATIO);
    speed_cm_s = rps * (2.0f * PI * WHEEL_RADIUS_CM);

    if (current_pwm_val < 0)
    {
        speed_cm_s = -speed_cm_s;
    }

    return (short)speed_cm_s;
}

static void CAN_Send_Feedback(short speedL, short speedR)
{
    CANMessage_t txMsg;
    uint8 ucTxBufferIndex;

    memset(&txMsg, 0, sizeof(CANMessage_t));
    txMsg.mId = CAN_ID_SPEED_FEEDBACK; 
    txMsg.mDataLength = 8;
    txMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO; 
    
    txMsg.mData[0] = (uint8)(speedL & 0xFF);
    txMsg.mData[1] = (uint8)((speedL >> 8) & 0xFF);
    txMsg.mData[2] = (uint8)(speedR & 0xFF);
    txMsg.mData[3] = (uint8)((speedR >> 8) & 0xFF);

    (void)CAN_SendMessage(CAN_CH_CMD, &txMsg, &ucTxBufferIndex);
}

static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError)
{
    CANMessage_t sRxMsg;
    CAN_RxPacket_t qPacket;

    (void)uiRxIndex; (void)uiRxBufferType;

    if (uiError == CAN_ERROR_NONE)
    {
        if (CAN_GetNewRxMessage(ucCh, &sRxMsg) == CAN_ERROR_NONE)
        {
            qPacket.id = sRxMsg.mId;
            qPacket.dlc = sRxMsg.mDataLength;
            for(int i=0; i<8; i++) qPacket.data[i] = sRxMsg.mData[i];
            
            SAL_QueuePut(g_canRxQueueHandle, &qPacket, sizeof(CAN_RxPacket_t), 0, SAL_OPT_NON_BLOCKING);
        }
    }
    else mcu_printf("[TX FAIL] Err: %d\n", uiError);
}

static void CAN_AppCallbackTxEvent(uint8 ucCh, CANTxInterruptType_t uiIntType) { (void)ucCh; (void)uiIntType; }
static void CAN_AppCallbackErrorEvent(uint8 ucCh, CANErrorType_t uiError) { (void)ucCh; (void)uiError; }

static void Init_CAN_HW(void)
{
    CAN_RegisterCallbackFunctionTx(&CAN_AppCallbackTxEvent);
    CAN_RegisterCallbackFunctionRx(&CAN_AppCallbackRxEvent);
    CAN_RegisterCallbackFunctionError(&CAN_AppCallbackErrorEvent);
    CAN_Init();
}

// [수정 2 구현] Linker Error 방지용 더미 함수
// (can_app.c가 빌드 목록에 있어서 필요함)
void CAN_consume_rx_message(void *pMsg, void *pPayload)
{
    (void)pMsg;
    (void)pPayload;
}

/*
***************************************************************************************************
* MOTOR LOGIC
***************************************************************************************************
*/
void Motor_HW_Init(void) {
    GPIO_Config(MOTOR_L_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_L_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    PDM_Init();
    
    ICTC_Init();
    ICTC_EnableCapture(ICTC_CH_ENC_L);
    ICTC_EnableCapture(ICTC_CH_ENC_R);

    Update_PWM_Duty(MOTOR_L_PWM_CH, 0);
    Update_PWM_Duty(MOTOR_R_PWM_CH, 0);
}

static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale) {
    PDMModeConfig_t pdmConfig;
    uint32 *lastDutyPtr = (channel == MOTOR_L_PWM_CH) ? &g_LastDutyL : &g_LastDutyR;
    uint32 wait_cnt = 0;
    
    if (*lastDutyPtr == duty_1000_scale) return;
    *lastDutyPtr = duty_1000_scale;
    
    memset(&pdmConfig, 0, sizeof(PDMModeConfig_t));
    pdmConfig.mcPortNumber = GPIO_PERICH_CH0;
    pdmConfig.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1;
    pdmConfig.mcOutputCtrl = 1;
    pdmConfig.mcClockDivide = 0;
    pdmConfig.mcPeriodNanoSec1 = PWM_PERIOD_NS;
    if(duty_1000_scale > MAX_DUTY_SCALE) duty_1000_scale = MAX_DUTY_SCALE;
    pdmConfig.mcDutyNanoSec1 = duty_1000_scale * (PWM_PERIOD_NS / MAX_DUTY_SCALE);
    
    PDM_Disable(channel, USE_PMM_MONITOR);
    while(PDM_GetChannelStatus(channel) == PDM_STATE_BUSY) { wait_cnt++; if(wait_cnt > 10000) break; __asm("nop"); }
    PDM_SetConfig(channel, &pdmConfig);
    if (duty_1000_scale > 0) PDM_Enable(channel, USE_PMM_MONITOR);
}

static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed) {
    uint32 duty = 0;
    float abs_speed = 0.0f;
    
    if (speed > 0) { GPIO_Set(p1, 1); GPIO_Set(p2, 0); abs_speed = speed; }
    else if (speed < 0) { GPIO_Set(p1, 0); GPIO_Set(p2, 1); abs_speed = -speed; }
    else { GPIO_Set(p1, 0); GPIO_Set(p2, 0); abs_speed = 0; }
    
    if (channel == MOTOR_L_PWM_CH) g_AppliedPWM_L = (int32)speed;
    else g_AppliedPWM_R = (int32)speed;

    duty = (uint32)(abs_speed * 10.0f);
    Update_PWM_Duty(channel, duty);
}

static void Car_Update_Logic(void) {
    float targetF = (float)g_TargetF;
    float targetR = (float)g_TargetR;

    curF = rate_limit(curF, targetF, STEP_VAL);
    curR = rate_limit(curR, targetR, STEP_VAL);

    float left_val  = curF + (curR * TURN_SENSITIVITY);
    float right_val = curF - (curR * TURN_SENSITIVITY);

    left_val  = Clamp_Val(left_val,  -100.0f, 100.0f);
    right_val = Clamp_Val(right_val, -100.0f, 100.0f);

    Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, left_val);
    Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, right_val);
}

static float Clamp_Val(float x, float lo, float hi) {
    if (x < lo) return lo; if (x > hi) return hi; return x;
}

static float rate_limit(float cur, float target, float step) {
    if (target > cur + step) return cur + step;
    if (target < cur - step) return cur - step;
    return target;
}
#endif // ( MCU_BSP_SUPPORT_APP_BASE == 1 )