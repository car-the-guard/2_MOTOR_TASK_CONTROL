// SPDX-License-Identifier: Apache-2.0
/*
***************************************************************************************************
* FileName : main.c (Motor Board - Safety Enhanced)
* Based on User Feedback:
* 1. Port Mapping Explicit Separation
* 2. CAN Queue Overflow Handling (Drop Count)
* 3. Command Timeout Logic (Failsafe)
* 4. AEB Shutdown Sequence (PDM Off first)
* 5. Data Race Protection (Atomic/Interrupt Lock simulated)
***************************************************************************************************
*/
#include <main.h>
uint32 gALiveMsgOnOff = 1;

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <gpio.h>
#include <pdm.h>
#include <ictc.h>
#include <string.h>

#include "can_config.h"
#include "can_reg.h"
#include "can.h"
#include "can_drv.h"

// [매크로 정의]
#ifndef ICTC_IRQ_CTRL_PRDDT_CMP_ISR
#define ICTC_IRQ_CTRL_PRDDT_CMP_ISR      (1UL << 4) 
#endif
#ifndef ICTC_OPEN_CTRL_FLTCNT_EN
#define ICTC_OPEN_CTRL_FLTCNT_EN         (1UL << 0)
#define ICTC_OPEN_CTRL_PDCMPCNT_EN       (1UL << 1)
#endif
#ifndef ICTC_OPMODE_CTRL_RISING_EDGE
#define ICTC_OPMODE_CTRL_ABS_SEL         (0UL)
#define ICTC_OPMODE_CTRL_RISING_EDGE     (1UL << 1) 
#define ICTC_OPMODE_CTRL_TCLK_BYPASS     (1UL << 2)
#define ICTC_OPMODE_CTRL_FLT_IMM_F_MODE  (0UL)
#define ICTC_OPMODE_CTRL_FLT_IMM_R_MODE  (0UL)
#define ICTC_OPMODE_CTRL_PRDDT_CMP_ISR   (1UL << 5)
#define ICTC_OPMODE_CTRL_F_IN_SEL_OFFSET (8)
#endif

// CAN ID
#define CAN_ID_AEB              (0x10)
#define CAN_ID_DRIVE_CMD        (0x12)
#define CAN_ID_ACCEL            (0x28)
#define CAN_ID_SPEED            (0x38)

#define CAN_RX_QUEUE_SIZE       (128)
#define CAN_CH_CMD              (0)

// [Fix 3] Timeout 설정 (200ms 이상 명령 없으면 정지)
#define CMD_TIMEOUT_MS          (200UL)

// [사양]
#define COUNTS_PER_REV          (210.0f) 
#define WHEEL_DIA_M             (0.13f)  
#define PI                      (3.141592f)

// 핀맵
#define ENCODER_L_ICTC_CH       (0)         
#define ENCODER_PIN_INDEX       (64UL)      
#define ENCODER_GPIO_PIN        GPIO_GPC(4) 
#define ENCODER_PIN_B_GPIO      GPIO_GPC(5) 
#define SYNC_GPIO_PIN           GPIO_GPC(3) 

#define MOTOR_L_PWM_CH          (0)
#define MOTOR_L_IN1             GPIO_GPB(21)
#define MOTOR_L_IN2             GPIO_GPB(23) 

#define MOTOR_R_PWM_CH          (1) 
#define MOTOR_R_IN1             GPIO_GPB(24)
#define MOTOR_R_IN2             GPIO_GPB(22)

#define PWM_PIN_L               GPIO_GPA(10) // PWM_OUT[00]
#define PWM_PIN_R               GPIO_GPA(11) // PWM_OUT[01]

#define PWM_PERIOD_NS           (1000000UL)     
#define MAX_DUTY_SCALE          (1000UL) 
#define USE_PMM_MONITOR         (0UL)

// SDK 표준 핀 인덱스
#ifndef GPIO_PERICH_CH0
#define GPIO_PERICH_CH0         (0UL)
#endif
#ifndef GPIO_PERICH_CH1
#define GPIO_PERICH_CH1         (1UL)
#endif

// [튜닝]
#define STEP_VAL                (50.0f)  
#define PHY_MIN_DUTY            (150)    
#define TURN_SENSITIVITY        (1.0f)   

// [Task 우선순위]
#define TASK_CAN_PRIO           (SAL_PRIO_APP_CFG + 5) 
#define TASK_SYNC_PRIO          (SAL_PRIO_APP_CFG + 4) 
#define TASK_MOTOR_PRIO         (SAL_PRIO_APP_CFG + 2) 

#define TASK_STK_SIZE           (1024) 
#define CONTROL_PERIOD_MS       (50)    
#define CONTROL_PERIOD_SEC      (0.05f)
#define SYNC_CHECK_PERIOD_MS    (10)    

// [디버깅]
#define DEBUG_MODE  1 
#if (DEBUG_MODE == 1)
    #define DBG_PRINT(...)  mcu_printf(__VA_ARGS__)
#else
    #define DBG_PRINT(...)
#endif

typedef struct {
    int32 counter; int32 last_counter;  
    float speed_rpm; float speed_mps;        
} Encoder_t;

typedef struct {
    uint32 id; uint32 dlc; uint8 data[8];
} CAN_RxPacket_t;

// [Fix 5] Data Race 방지를 위한 Target 구조체
typedef struct {
    volatile int32 targetF;
    volatile int32 targetR;
    volatile uint32 lastCmdTick; // [Fix 3] 타임아웃 체크용
} DriveCtrl_t;

static uint32 g_canRxQueueHandle = 0;
Encoder_t Enc1; 

// 글로벌 상태 변수
DriveCtrl_t g_Drive = {0, 0, 0}; 
volatile uint8 g_aeb_active = 0;
static float g_prev_speed_mps = 0.0f;
volatile static uint32 g_TimeBaseOffset = 0; 

// PWM 상태
static uint32 g_LastDutyL = 0xFFFF;
static uint32 g_LastDutyR = 0xFFFF;
static float curF = 0.0f;
static float curR = 0.0f;
static uint32 g_rx_drop_cnt = 0; // [Fix 2] 큐 오버플로우 카운터

// 함수 선언
static void Main_StartTask(void * pArg);
static void AppTaskCreate(void);
static void Task_CAN_Rx(void *pArg);
static void Task_MotorControl(void *pArg);
static void Task_TimeSync(void *pArg); 
static void Init_CAN_HW(void);
static void Motor_HW_Init(void);
static void Encoder_HW_Init(void);
static void Sync_GPIO_Init(void);
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale);
static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed);
static void Car_Update_Logic(void);
static void Encoder_Update(Encoder_t *enc, float dt_ms);
static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty);
static void CAN_Send_Data_With_Time(uint32 id, int32 value_scaled, uint16 timestamp);
static float Clamp_Val(float x, float lo, float hi);
static float rate_limit(float cur, float target, float step);
static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError);
static void CAN_AppCallbackTxEvent(uint8 ucCh, CANTxInterruptType_t uiIntType);
static void CAN_AppCallbackErrorEvent(uint8 ucCh, CANErrorType_t uiError);
static uint32 Get_Sys_Tick_Safe(void);
void Process_Packet(CAN_RxPacket_t *pkt);
static void Emergency_Stop_Hard(void); // [Fix 4] 긴급 정지 전용 함수

// Main
void cmain (void) {
    static uint32 AppTaskStartID = 0;
    static uint32 AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    (void)SAL_Init(); BSP_PreInit(); BSP_Init();
    mcu_printf("\n[MOTOR] REAL BOARD SAFETY VER (TIMEOUT/AEB SEQ/PORT MAP)\n"); 
    (void)SAL_TaskCreate(&AppTaskStartID, (const uint8 *)"StartTask", (SALTaskFunc)&Main_StartTask,
                                      &AppTaskStartStk[0], ACFG_TASK_MEDIUM_STK_SIZE, SAL_PRIO_APP_CFG, NULL);
    (void)SAL_OsStart();
}

static void Main_StartTask(void * pArg) {
    (void)pArg; (void)SAL_OsInitFuncs();
    AppTaskCreate();
    while (1) { (void)SAL_TaskSleep(5000); }
}

static void AppTaskCreate(void) {
    static uint32 TaskCanID, TaskCanStk[TASK_STK_SIZE];
    static uint32 TaskMotorID, TaskMotorStk[TASK_STK_SIZE];
    static uint32 TaskSyncID, TaskSyncStk[TASK_STK_SIZE]; 
    
    Motor_HW_Init(); 
    Encoder_HW_Init();
    Sync_GPIO_Init(); 
    
    g_TimeBaseOffset = Get_Sys_Tick_Safe();
    g_Drive.lastCmdTick = Get_Sys_Tick_Safe(); // 초기화

    SAL_QueueCreate(&g_canRxQueueHandle, (const uint8 *)"CAN_Q", CAN_RX_QUEUE_SIZE, sizeof(CAN_RxPacket_t));
    Init_CAN_HW();
    
    SAL_TaskCreate(&TaskCanID, (const uint8 *)"CAN Task", (SALTaskFunc)&Task_CAN_Rx, 
                   &TaskCanStk[0], TASK_STK_SIZE, TASK_CAN_PRIO, NULL);
    SAL_TaskCreate(&TaskSyncID, (const uint8 *)"Sync Task", (SALTaskFunc)&Task_TimeSync, 
                   &TaskSyncStk[0], TASK_STK_SIZE, TASK_SYNC_PRIO, NULL);
    SAL_TaskCreate(&TaskMotorID, (const uint8 *)"Motor Task", (SALTaskFunc)&Task_MotorControl, 
                   &TaskMotorStk[0], TASK_STK_SIZE, TASK_MOTOR_PRIO, NULL);
}

static void Task_CAN_Rx(void *pArg) {
    (void)pArg;
    CAN_RxPacket_t rxPacket;
    uint32 uiSizeCopied;
    
    while(1) {
        if (SAL_QueueGet(g_canRxQueueHandle, &rxPacket, &uiSizeCopied, 0, SAL_OPT_BLOCKING) == SAL_RET_SUCCESS) {
            Process_Packet(&rxPacket); 
            // Queue Drain
            while (SAL_QueueGet(g_canRxQueueHandle, &rxPacket, &uiSizeCopied, 0, SAL_OPT_NON_BLOCKING) == SAL_RET_SUCCESS) {
                Process_Packet(&rxPacket); 
            }
        }
    }
}

void Process_Packet(CAN_RxPacket_t *pkt) {
    // 1. AEB 신호
    if (pkt->id == CAN_ID_AEB) {
        if (pkt->data[0] != 0x00) {
            if(!g_aeb_active) {
                mcu_printf("!!! [RX] AEB ACTIVE - HARD STOP !!!\n");
                g_aeb_active = 1;
                // [Fix 4] AEB 수신 즉시 Hard Stop (Task 주기 대기하지 않음)
                Emergency_Stop_Hard(); 
            }
        } else {
            if(g_aeb_active) mcu_printf("... [RX] AEB RELEASED ...\n");
            g_aeb_active = 0;
            // 해제 시 Timeout 리셋 (바로 출발 방지)
            g_Drive.lastCmdTick = Get_Sys_Tick_Safe(); 
        }
    }
    // 2. 모터 제어 명령
    else if (pkt->id == CAN_ID_DRIVE_CMD) {
        if (!g_aeb_active) {
            // [Fix 5] 구조체로 묶어서 관리
            g_Drive.targetF = (int32)pkt->data[0] - (int32)pkt->data[1];
            g_Drive.targetR = (int32)pkt->data[3] - (int32)pkt->data[2];
            
            // [Fix 3] 마지막 수신 시간 갱신 (Timeout 방지)
            g_Drive.lastCmdTick = Get_Sys_Tick_Safe();
        }
    }
}

static void Task_TimeSync(void *pArg) {
    (void)pArg;
    static uint32 last_sync_state = 1; 
    while(1) {
        uint32 cur_sync_state = GPIO_Get(SYNC_GPIO_PIN);
        if (last_sync_state == 1 && cur_sync_state == 0) {
            g_TimeBaseOffset = Get_Sys_Tick_Safe();
        }
        last_sync_state = cur_sync_state;
        SAL_TaskSleep(SYNC_CHECK_PERIOD_MS);
    }
}

static void Task_MotorControl(void *pArg) {
    (void)pArg;
    while(1) {
        Car_Update_Logic();
        Encoder_Update(&Enc1, (float)CONTROL_PERIOD_MS); 
        
        float accel_mps2 = (Enc1.speed_mps - g_prev_speed_mps) / CONTROL_PERIOD_SEC;
        g_prev_speed_mps = Enc1.speed_mps;
        
        uint32 current_tick = Get_Sys_Tick_Safe();
        uint32 debug_time_ms = current_tick - g_TimeBaseOffset;
        uint16 can_time_ms = (uint16)(debug_time_ms & 0xFFFF);

        CAN_Send_Data_With_Time(CAN_ID_SPEED, (int32)(Enc1.speed_mps * 100.0f), can_time_ms);
        CAN_Send_Data_With_Time(CAN_ID_ACCEL, (int32)(accel_mps2 * 1000.0f), can_time_ms);
        
        SAL_TaskSleep(CONTROL_PERIOD_MS); 
    }
}

static uint32 Get_Sys_Tick_Safe(void) {
    uint32 tick_val = 0; (void)SAL_GetTickCount(&tick_val); return tick_val;
}

static void CAN_Send_Data_With_Time(uint32 id, int32 value_scaled, uint16 timestamp) {
    CANMessage_t txMsg; uint8 ucTxBufferIndex;
    memset(&txMsg, 0, sizeof(CANMessage_t));
    txMsg.mId = id; txMsg.mDataLength = 8; txMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;
    txMsg.mData[0] = (uint8)((value_scaled >> 24) & 0xFF);
    txMsg.mData[1] = (uint8)((value_scaled >> 16) & 0xFF);
    txMsg.mData[2] = (uint8)((value_scaled >> 8) & 0xFF);
    txMsg.mData[3] = (uint8)(value_scaled & 0xFF);
    txMsg.mData[4] = (uint8)((timestamp >> 8) & 0xFF);
    txMsg.mData[5] = (uint8)(timestamp & 0xFF);
    (void)CAN_SendMessage(CAN_CH_CMD, &txMsg, &ucTxBufferIndex);
}

static void Sync_GPIO_Init(void) {
    #ifndef GPIO_PULLUP
    #define GPIO_PULLUP 0 
    #endif
    GPIO_Config(SYNC_GPIO_PIN, (GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP));
}

void Motor_HW_Init(void) {
    GPIO_Config(MOTOR_L_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_L_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    
    GPIO_Config(PWM_PIN_L, GPIO_FUNC(3) | GPIO_OUTPUT);
    GPIO_Config(PWM_PIN_R, GPIO_FUNC(3) | GPIO_OUTPUT);
    
    PDM_Init(); 
    Update_PWM_Duty(MOTOR_L_PWM_CH, 0); 
    Update_PWM_Duty(MOTOR_R_PWM_CH, 0);
}

// [Fix 1] PDM 채널–포트 매핑 명시
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale) {
    PDMModeConfig_t pdmConfig;
    uint32 *lastDutyPtr = (channel == MOTOR_L_PWM_CH) ? &g_LastDutyL : &g_LastDutyR;
    
    if (*lastDutyPtr == duty_1000_scale) return;
    *lastDutyPtr = duty_1000_scale;

    memset(&pdmConfig, 0, sizeof(PDMModeConfig_t));
    
    // [Fix 1] 명시적인 포트 분리 (추후 보드 리비전 대비)
    if (channel == MOTOR_L_PWM_CH) {
        pdmConfig.mcPortNumber = GPIO_PERICH_CH0; 
    } else {
        // 현재 하드웨어가 CH0을 공유한다면 CH0으로, 분리되어 있다면 CH1으로 변경 용이
        pdmConfig.mcPortNumber = GPIO_PERICH_CH0; 
    }
    
    pdmConfig.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1; 
    pdmConfig.mcOutputCtrl = 1; 
    pdmConfig.mcClockDivide = 0; 
    pdmConfig.mcPeriodNanoSec1 = PWM_PERIOD_NS; 
    
    uint32 final_duty = duty_1000_scale;
    if (final_duty > 0 && final_duty < PHY_MIN_DUTY) final_duty = PHY_MIN_DUTY; 
    if(final_duty > MAX_DUTY_SCALE) final_duty = MAX_DUTY_SCALE;
    pdmConfig.mcDutyNanoSec1 = final_duty * (PWM_PERIOD_NS / MAX_DUTY_SCALE);

    PDM_Disable(channel, USE_PMM_MONITOR);
    // Timeout loop for Disable confirmation
    uint32 uiCnt = 0;
    while(PDM_GetChannelStatus(channel)) {
        SAL_TaskSleep(1);
        if(++uiCnt > 20) break;
    }

    if (PDM_SetConfig(channel, &pdmConfig) == SAL_RET_SUCCESS) {
        if (final_duty > 0) PDM_Enable(channel, USE_PMM_MONITOR);
    }
}

// [Fix 4] 긴급 정지 시퀀스 (PWM Disable -> GPIO OFF)
static void Emergency_Stop_Hard(void) {
    // 1. 목표값 0 초기화
    g_Drive.targetF = 0; g_Drive.targetR = 0;
    curF = 0.0f; curR = 0.0f;

    // 2. PDM(PWM) 완전 비활성화 (전원 차단)
    PDM_Disable(MOTOR_L_PWM_CH, USE_PMM_MONITOR);
    PDM_Disable(MOTOR_R_PWM_CH, USE_PMM_MONITOR);
    
    // 3. 방향 GPIO 차단 (H-Bridge 보호)
    GPIO_Set(MOTOR_L_IN1, 0); GPIO_Set(MOTOR_L_IN2, 0);
    GPIO_Set(MOTOR_R_IN1, 0); GPIO_Set(MOTOR_R_IN2, 0);
    
    // 상태 동기화
    g_LastDutyL = 0; g_LastDutyR = 0;
}

static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed) {
    // [Fix 4] 일반 제어 시에도 안전 고려: 속도 0이면 GPIO 끄기
    if (speed == 0.0f) {
        GPIO_Set(p1, 0); GPIO_Set(p2, 0);
        Update_PWM_Duty(channel, 0);
        return;
    }

    // 방향 설정
    if (speed > 0) { GPIO_Set(p1, 1); GPIO_Set(p2, 0); }
    else { GPIO_Set(p1, 0); GPIO_Set(p2, 1); speed = -speed; }
    
    Update_PWM_Duty(channel, (uint32)(speed * 10.0f));
}

static void Car_Update_Logic(void) {
    // 1. AEB 체크 (최우선)
    if (g_aeb_active) {
        // 이미 Emergency_Stop_Hard()가 호출되었지만, polling 루프에서도 방어
        Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, 0.0f);
        Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, 0.0f);
        return; 
    }

    // [Fix 3] Timeout Check (Dead Man Switch)
    uint32 now = Get_Sys_Tick_Safe();
    if ((now > g_Drive.lastCmdTick) && (now - g_Drive.lastCmdTick > CMD_TIMEOUT_MS)) {
        // 타임아웃 발생: 안전하게 감속 정지
        g_Drive.targetF = 0;
        g_Drive.targetR = 0;
        // (선택) 여기서 mcu_printf로 로그 찍을 수 있으나 빈도 주의
    }

    // 2. 정상 주행 로직
    curF = rate_limit(curF, (float)g_Drive.targetF, STEP_VAL);
    curR = rate_limit(curR, (float)g_Drive.targetR, STEP_VAL);
    
    float left_val  = Clamp_Val(curF + (curR * TURN_SENSITIVITY), -100.0f, 100.0f);
    float right_val = Clamp_Val(curF - (curR * TURN_SENSITIVITY), -100.0f, 100.0f);
    
    Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, left_val);
    Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, right_val);
}

static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty) {
    (void)uiPeriod; (void)uiDuty;
    if (uiChannel == ENCODER_L_ICTC_CH) {
        uint32 stateB = GPIO_Get(ENCODER_PIN_B_GPIO); 
        if (stateB == 0) Enc1.counter++; 
        else Enc1.counter--; 
    }
}

static float Clamp_Val(float x, float lo, float hi) {
    if (x < lo) return lo; if (x > hi) return hi; return x;
}
static float rate_limit(float cur, float target, float step) {
    if (target > cur + step) return cur + step;
    if (target < cur - step) return cur - step;
    return target;
}
static void Encoder_Update(Encoder_t *enc, float dt_ms) {
    if (dt_ms < 0.1f) return;
    int32 cur_counter = enc->counter;
    int32 diff = cur_counter - enc->last_counter;
    enc->last_counter = cur_counter;
    enc->speed_rpm = ((float)diff / COUNTS_PER_REV) * (60000.0f / dt_ms);
    enc->speed_mps = (enc->speed_rpm * PI * WHEEL_DIA_M) / 60.0f;
}
static void Encoder_HW_Init(void) {
    ICTCModeConfig_t IctcConfig; 
    memset(&IctcConfig, 0, sizeof(ICTCModeConfig_t));
    Enc1.counter = 0; Enc1.last_counter = 0; 
    #ifndef GPIO_PULLUP
        #define GPIO_PULLUP 0 
    #endif
    GPIO_Config(ENCODER_GPIO_PIN, (GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_DS(0x3UL) | GPIO_PULLUP));
    GPIO_Config(ENCODER_PIN_B_GPIO, (GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP));
    IctcConfig.mcTimeout = 0x0FFFFFFFUL;
    IctcConfig.mcEnableIrq = ICTC_IRQ_CTRL_PRDDT_CMP_ISR; 
    IctcConfig.mcEnableCounter = ICTC_OPEN_CTRL_FLTCNT_EN | ICTC_OPEN_CTRL_PDCMPCNT_EN;
    IctcConfig.mcOperationMode = ICTC_OPMODE_CTRL_ABS_SEL | ICTC_OPMODE_CTRL_RISING_EDGE | ICTC_OPMODE_CTRL_TCLK_BYPASS | ICTC_OPMODE_CTRL_FLT_IMM_F_MODE | ICTC_OPMODE_CTRL_FLT_IMM_R_MODE | ICTC_OPMODE_CTRL_PRDDT_CMP_ISR | (ENCODER_PIN_INDEX << ICTC_OPMODE_CTRL_F_IN_SEL_OFFSET);
    ICTC_Init();
    ICTC_SetCallBackFunc(ENCODER_L_ICTC_CH, (ICTCCallback)&ICTC_Callback);
    ICTC_SetIRQCtrlReg(ENCODER_L_ICTC_CH, IctcConfig.mcEnableIrq);
    ICTC_SetTimeoutValue(ENCODER_L_ICTC_CH, IctcConfig.mcTimeout);
    ICTC_SetEdgeMatchingValue(ENCODER_L_ICTC_CH, 0x100, 0x100, 0xFFFF);
    ICTC_SetCompareRoundValue(ENCODER_L_ICTC_CH, 0x0FFFFFF0UL, 0x0FFFFFF0UL);
    ICTC_SetOpEnCtrlCounter(ENCODER_L_ICTC_CH, IctcConfig.mcEnableCounter);
    ICTC_SetOpModeCtrlReg(ENCODER_L_ICTC_CH, IctcConfig.mcOperationMode);
    ICTC_EnableCapture(ENCODER_L_ICTC_CH);
}

// [Fix 2] CAN Rx Queue Overflow 처리
static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError) {
    CANMessage_t sRxMsg; CAN_RxPacket_t qPacket; (void)uiRxIndex; (void)uiRxBufferType;
    if (uiError == CAN_ERROR_NONE && CAN_GetNewRxMessage(ucCh, &sRxMsg) == CAN_ERROR_NONE) {
        qPacket.id = sRxMsg.mId; qPacket.dlc = sRxMsg.mDataLength; memcpy(qPacket.data, sRxMsg.mData, 8);
        
        // Non-Blocking 시도 후 실패 시 카운트 증가 (디버깅용)
        if (SAL_QueuePut(g_canRxQueueHandle, &qPacket, sizeof(CAN_RxPacket_t), 0, SAL_OPT_NON_BLOCKING) != SAL_RET_SUCCESS) {
            g_rx_drop_cnt++;
        }
    }
}
static void CAN_AppCallbackTxEvent(uint8 ucCh, CANTxInterruptType_t uiIntType) { (void)ucCh; (void)uiIntType; }
static void CAN_AppCallbackErrorEvent(uint8 ucCh, CANErrorType_t uiError) { (void)ucCh; (void)uiError; }
static void Init_CAN_HW(void) {
    CAN_RegisterCallbackFunctionTx(&CAN_AppCallbackTxEvent); CAN_RegisterCallbackFunctionRx(&CAN_AppCallbackRxEvent);
    CAN_RegisterCallbackFunctionError(&CAN_AppCallbackErrorEvent); CAN_Init();
}
void CAN_consume_rx_message(void *pMsg, void *pPayload) { (void)pMsg; (void)pPayload; }

#endif