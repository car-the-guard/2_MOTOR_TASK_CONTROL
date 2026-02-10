// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
* FileName : main.c (RTOS CAN Control Version)
*
* Copyright (c) Telechips Inc.
*
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

// [CAN Include Files]
#include "can_config.h"
#include "can.h"
#include "can_drv.h"

/*
***************************************************************************************************
* DEFINES & MACROS
***************************************************************************************************
*/
// --- Task Configuration ---
#define TASK_CAN_PRIO           (SAL_PRIO_APP_CFG + 1) // CAN 처리 우선순위
#define TASK_MOTOR_PRIO         (SAL_PRIO_APP_CFG + 2) // 모터 제어 우선순위 (높음)
#define TASK_STK_SIZE           (512)

// --- CAN Configuration ---
#define CAN_CH_CMD              (0)             // 사용할 CAN 채널 (0번 채널 가정)
#define CAN_ID_DRIVE_CMD        (0x10)          // 로그상 주행 명령으로 추정되는 ID
#define CAN_ID_STATUS_MSG       (0x48)          // 로그상 자주 보이는 ID (상태/센서값 추정)

// --- Motor Pinmap & Parameters ---
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
#define GPIO_PERICH_CH0         (0UL)

/*
***************************************************************************************************
* GLOBAL VARIABLES
***************************************************************************************************
*/
uint32                          gALiveMsgOnOff;
static uint32                   gALiveCount;

// Shared Variables (CAN Task <-> Motor Task)
volatile int32 g_TargetF = 0;   // Forward/Backward Speed
volatile int32 g_TargetR = 0;   // Rotation/Steering

// Motor Internal State
static uint32 g_LastDutyL = 0xFFFF;
static uint32 g_LastDutyR = 0xFFFF;

// Task Stacks & IDs
static uint32 TaskCanID;
static uint32 TaskCanStk[TASK_STK_SIZE];

static uint32 TaskMotorID;
static uint32 TaskMotorStk[TASK_STK_SIZE];

/*
***************************************************************************************************
* FUNCTION PROTOTYPES
***************************************************************************************************
*/
// OS Core Functions
static void Main_StartTask(void * pArg);
static void AppTaskCreate(void);
static void DisplayAliveLog(void);

// App Tasks
static void Task_CAN_Rx(void *pArg);
static void Task_MotorControl(void *pArg);

// Logic Helpers
static void Motor_HW_Init(void);
static void Init_CAN_HW(void); // CAN 초기화
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale);
static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed);
static void Car_Update_Logic(void);
static float Clamp_Val(float x, float lo, float hi);

// CAN Callbacks (Dummy for initialization requirements)
static void CAN_AppCallbackTxEvent(uint8 ucCh, CANTxInterruptType_t uiIntType);
static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError);
static void CAN_AppCallbackErrorEvent(uint8 ucCh, CANErrorType_t uiError);

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

    mcu_printf("\n===============================\n");
    mcu_printf("    RC Car CAN Control Ver     \n");
    mcu_printf("===============================\n");

    err = (SALRetCode_t)SAL_TaskCreate(&AppTaskStartID,
                                     (const uint8 *)"App Task Start",
                                     (SALTaskFunc) &Main_StartTask,
                                     &AppTaskStartStk[0],
                                     ACFG_TASK_MEDIUM_STK_SIZE,
                                     SAL_PRIO_APP_CFG,
                                     NULL);

    if (err == SAL_RET_SUCCESS)
    {
        (void)SAL_OsStart();
    }
}

/*
***************************************************************************************************
* TASKS
***************************************************************************************************
*/

// [Task 1] Main Start Task
static void Main_StartTask(void * pArg)
{
    (void)pArg;
    (void)SAL_OsInitFuncs();

    AppTaskCreate();

    while (1)
    {
        DisplayAliveLog();
        (void)SAL_TaskSleep(5000);
    }
}

// [Task 2] CAN RX Task: CAN 메시지 수신 및 파싱
static void Task_CAN_Rx(void *pArg)
{
    (void)pArg;
    uint32 uiRxMsgNum;
    CANMessage_t sRxMsg;
    
    // 파싱용 임시 변수
    uint8_t val_front, val_back, val_left, val_right;

    mcu_printf("[Task] CAN RX Ready (Ch: %d)\n", CAN_CH_CMD);

    while(1)
    {
        // 1. 수신된 메시지가 있는지 확인 (Polling 방식)
        uiRxMsgNum = CAN_CheckNewRxMessage(CAN_CH_CMD);

        if( uiRxMsgNum > 0 )
        {
            // 2. 메시지 가져오기
            ( void ) CAN_GetNewRxMessage(CAN_CH_CMD, &sRxMsg);

            // 3. ID 필터링 및 파싱
            if (sRxMsg.mId == CAN_ID_DRIVE_CMD) // ID: 0x10
            {
                // Protocol: [0]Front, [1]Back, [2]Left, [3]Right (각 1byte)
                if (sRxMsg.mDataLength >= 4)
                {
                    val_front = sRxMsg.mData[0];
                    val_back  = sRxMsg.mData[1];
                    val_left  = sRxMsg.mData[2];
                    val_right = sRxMsg.mData[3];

                    // 전역 제어 변수 업데이트
                    // 전진이 입력되면 양수, 후진이 입력되면 음수
                    if (val_front > 0)      g_TargetF = (int32)val_front;
                    else if (val_back > 0)  g_TargetF = -((int32)val_back);
                    else                    g_TargetF = 0;

                    // 우회전이 입력되면 양수, 좌회전이 입력되면 음수
                    if (val_right > 0)      g_TargetR = (int32)val_right;
                    else if (val_left > 0)  g_TargetR = -((int32)val_left);
                    else                    g_TargetR = 0;

                    mcu_printf("[CAN CMD] F:%d R:%d (Raw: %02X %02X %02X %02X)\n", 
                               g_TargetF, g_TargetR, val_front, val_back, val_left, val_right);
                }
            }
            else if (sRxMsg.mId == CAN_ID_STATUS_MSG) // ID: 0x48
            {
                // 로그용: 0x48 ID는 수신은 하되 제어에는 반영 안 함
                // mcu_printf("[CAN LOG] Status ID 0x48 Recv\n");
            }
        }
        else
        {
            // 데이터가 없으면 슬립 (CPU 양보)
            SAL_TaskSleep(5); 
        }
    }
}

// [Task 3] Motor Task (기존과 동일)
static void Task_MotorControl(void *pArg)
{
    (void)pArg;
    
    mcu_printf("[Task] Motor Control Ready\n");

    while(1)
    {
        Car_Update_Logic();
        SAL_TaskSleep(20); // 50Hz Control Loop
    }
}

static void AppTaskCreate(void)
{
    SALRetCode_t err;

    // 1. 하드웨어 초기화
    Motor_HW_Init();
    Init_CAN_HW(); // CAN 초기화 추가

    // 2. CAN RX Task 생성
    err = SAL_TaskCreate(&TaskCanID,
                         (const uint8 *)"CAN Rx Task",
                         (SALTaskFunc)&Task_CAN_Rx,
                         &TaskCanStk[0],
                         TASK_STK_SIZE,
                         TASK_CAN_PRIO,
                         NULL);
    if(err != SAL_RET_SUCCESS) mcu_printf("CAN Task Create Fail!\n");
    else mcu_printf("CAN Task is Created\n");

    // 3. Motor Task 생성
    err = SAL_TaskCreate(&TaskMotorID,
                         (const uint8 *)"Motor Task",
                         (SALTaskFunc)&Task_MotorControl,
                         &TaskMotorStk[0],
                         TASK_STK_SIZE,
                         TASK_MOTOR_PRIO,
                         NULL);
    if(err != SAL_RET_SUCCESS) mcu_printf("Motor Task Create Fail!\n");
    else mcu_printf("Motor Task is Created\n");
}

/*
***************************************************************************************************
* HELPER FUNCTIONS
***************************************************************************************************
*/

static void Init_CAN_HW(void)
{
    // CAN 콜백 등록 (필수)
    ( void ) CAN_RegisterCallbackFunctionTx( &CAN_AppCallbackTxEvent );
    ( void ) CAN_RegisterCallbackFunctionRx( &CAN_AppCallbackRxEvent );
    ( void ) CAN_RegisterCallbackFunctionError( &CAN_AppCallbackErrorEvent );

    // CAN 하드웨어 초기화
    if (CAN_Init() == CAN_ERROR_NONE)
    {
        mcu_printf("[INIT] CAN HW Init Success\n");
    }
    else
    {
        mcu_printf("[INIT] CAN HW Init Fail\n");
    }
}

// CAN Callbacks - RTOS Task 방식에서는 콜백에서 최소한의 일만 하거나,
// 현재 구조처럼 Task에서 Polling 할 경우 비워두거나 플래그만 세웁니다.
static void CAN_AppCallbackTxEvent(uint8 ucCh, CANTxInterruptType_t uiIntType) { (void)ucCh; (void)uiIntType; }
static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError) 
{ 
    // RX 인터럽트 발생 시 호출됨. 
    // Task_CAN_Rx가 Polling 중이므로 여기서 특별한 처리를 안 해도 되지만,
    // 필요하다면 Semaphore를 Give하여 Task를 깨울 수 있음.
    (void)ucCh; (void)uiRxIndex; (void)uiRxBufferType; (void)uiError; 
}
static void CAN_AppCallbackErrorEvent(uint8 ucCh, CANErrorType_t uiError) { (void)ucCh; (void)uiError; }


void Motor_HW_Init(void)
{
    // 기존과 동일
    GPIO_Config(MOTOR_L_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_L_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);

    PDM_Init();
    Update_PWM_Duty(MOTOR_L_PWM_CH, 0);
    Update_PWM_Duty(MOTOR_R_PWM_CH, 0);
}

// ... (Update_PWM_Duty, Motor_Set_VCP 함수는 기존과 동일하여 생략 가능하지만, 컴파일 위해 유지 필요) ...
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale)
{
    PDMModeConfig_t pdmConfig;
    uint32 *lastDutyPtr = (channel == MOTOR_L_PWM_CH) ? &g_LastDutyL : &g_LastDutyR;
    uint32 wait_cnt = 0;

    if (*lastDutyPtr == duty_1000_scale) return;
    *lastDutyPtr = duty_1000_scale;

    memset(&pdmConfig, 0, sizeof(PDMModeConfig_t));
    pdmConfig.mcPortNumber      = GPIO_PERICH_CH0;
    pdmConfig.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1;
    pdmConfig.mcOutputCtrl      = 1;
    pdmConfig.mcClockDivide     = 0;
    pdmConfig.mcPeriodNanoSec1  = PWM_PERIOD_NS;
    
    if(duty_1000_scale > MAX_DUTY_SCALE) duty_1000_scale = MAX_DUTY_SCALE;
    pdmConfig.mcDutyNanoSec1    = duty_1000_scale * (PWM_PERIOD_NS / MAX_DUTY_SCALE);

    PDM_Disable(channel, USE_PMM_MONITOR);
    while(PDM_GetChannelStatus(channel) == PDM_STATE_BUSY) {
        wait_cnt++; if(wait_cnt > 10000) break; __asm("nop");
    }
    PDM_SetConfig(channel, &pdmConfig);
    if (duty_1000_scale > 0) PDM_Enable(channel, USE_PMM_MONITOR);
}

static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed)
{
    uint32 duty = 0;
    float abs_speed = 0.0f;

    if (speed > 0) {
        GPIO_Set(p1, 1); GPIO_Set(p2, 0); abs_speed = speed;
    } else if (speed < 0) {
        GPIO_Set(p1, 0); GPIO_Set(p2, 1); abs_speed = -speed;
    } else {
        GPIO_Set(p1, 0); GPIO_Set(p2, 0); abs_speed = 0;
    }

    duty = (uint32)(abs_speed * 10.0f);
    Update_PWM_Duty(channel, duty);
}

static void Car_Update_Logic(void)
{
    float F = (float)g_TargetF;
    float R = (float)g_TargetR;

    float left_val  = F + (R * TURN_SENSITIVITY);
    float right_val = F - (R * TURN_SENSITIVITY);

    left_val  = Clamp_Val(left_val,  -100.0f, 100.0f);
    right_val = Clamp_Val(right_val, -100.0f, 100.0f);

    Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, left_val);
    Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, right_val);
}

static float Clamp_Val(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void DisplayAliveLog(void)
{
    if (gALiveMsgOnOff != 0U) {
        mcu_printf("\n %d", gALiveCount++);
        if(gALiveCount >= MAIN_UINT_MAX_NUM) gALiveCount = 0;
    } else {
        gALiveCount = 0;
    }
}

#define LDT1_AREA_ADDR  0xA1011800U
#define PMU_REG_ADDR    0xA0F28000U
static void DisplayOTPInfo(void)
{
    volatile uint32 *ldt1Addr;
    volatile uint32 *chipNameAddr;
    volatile uint32 *remapAddr;
    volatile uint32 *hsmStatusAddr;
    uint32          chipName = 0;
    uint32          dualBankVal = 0;
    uint32          dual_bank = 0;
    uint32          expandFlashVal = 0;
    uint32          expand_flash = 0;
    uint32          remap_mode = 0;
    uint32          hsm_ready = 0;

    //----------------------------------------------------------------
    // OTP LDT1 Read
    // [11:0]Dual_Bank_Selection, [59:48]EXPAND_FLASH
    // Dual_Bank_Sel: [0xC0][11: 0] & [0xD0][11: 0] & [0xE0][11: 0] & [0xF0][11: 0]
    // EXPAND_FLASH : [0xC4][27:16] & [0xD4][27:16] & [0xE4][27:16] & [0xF4][27:16]
    // HwMC_PRG_FLS_LDT1: 0xA1011800

    ldt1Addr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x00C0);
    chipNameAddr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x0300);
    remapAddr = (volatile uint32 *)(PMU_REG_ADDR);
    hsmStatusAddr = (volatile uint32 *)(PMU_REG_ADDR + 0x0020);

    chipName = *chipNameAddr;
    chipName &= 0x000FFFFF;

    dualBankVal = ldt1Addr[ 0];
    expandFlashVal = ldt1Addr[ 1];

    dualBankVal &= ldt1Addr[ 4];
    expandFlashVal &= ldt1Addr[ 5];

    dualBankVal &= ldt1Addr[ 8];
    expandFlashVal &= ldt1Addr[ 9];

    dualBankVal &= ldt1Addr[12];
    expandFlashVal &= ldt1Addr[13];

    dualBankVal = (dualBankVal >> 0) & 0x0FFF;
    expandFlashVal  = (expandFlashVal >> 16) & 0x0FFF;

    dual_bank = (dualBankVal == 0x0FFF) ? 0 : 1;            // (single_bank : dual_bank)
    expand_flash  = (expandFlashVal  == 0x0000) ? 0 : 1;    // (only_eFlash : use_extSNOR)

    remap_mode = remapAddr[ 0];

    mcu_printf("    CHIP   NAME  : %x\n",    chipName);
    mcu_printf("    DUAL   BANK  : %d\n",    dual_bank);
    mcu_printf("    EXPAND FLASH : %d\n",    expand_flash);
    mcu_printf("    REMAP  MODE  : %d\n",    (remap_mode >> 16));

    hsm_ready = hsmStatusAddr[ 0];
    hsm_ready = (hsm_ready >> 2) & 0x0001;
#if 0
    if(hsm_ready)
    {
        mcu_printf("    HSM    READY : %d\n",    hsm_ready);
    }
    else
    {
        while(hsm_ready != 1)
        {
            mcu_printf("    HSM    READY : %d\n",    hsm_ready);
            mcu_printf("    wait...\n");
            hsm_ready = (hsm_ready >> 2) & 0x0001;
        }
    }
#else
    mcu_printf("    HSM    READY : %d\n",    hsm_ready);
#endif
}

#endif  // ( MCU_BSP_SUPPORT_APP_BASE == 1 )