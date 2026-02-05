// main.c
#include <main.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <gpio.h>

// [모듈 헤더 포함]
#include "app_types.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "can_manager.h"

uint32 gALiveMsgOnOff = 1;

// [글로벌 변수 실체 정의]
DriveCtrl_t g_Drive = {0, 0, 0};
Encoder_t Enc1;
volatile uint8 g_aeb_active = 0;
static volatile uint32 g_TimeBaseOffset = 0;
//static float g_prev_speed_mps = 0.0f; // 가속도 계산용

// 태스크 설정
#define TASK_STK_SIZE       (1024) 
#define TASK_CAN_PRIO       (SAL_PRIO_APP_CFG + 5) 
#define TASK_SYNC_PRIO      (SAL_PRIO_APP_CFG + 4) 
#define TASK_MOTOR_PRIO     (SAL_PRIO_APP_CFG + 2) 
#define CONTROL_PERIOD_MS   (50)
#define CMD_TIMEOUT_MS      (200UL)
#define SYNC_GPIO_PIN       GPIO_GPC(3)

// 함수 선언
static void Main_StartTask(void * pArg);
static void AppTaskCreate(void);
static void Task_MotorControl(void *pArg);
static void Task_TimeSync(void *pArg);
uint32 Get_Sys_Tick_Safe(void) { uint32 tick; SAL_GetTickCount(&tick); return tick; }

// Main
void cmain (void) {
    static uint32 AppTaskStartID = 0;
    static uint32 AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    (void)SAL_Init(); BSP_PreInit(); BSP_Init();
    mcu_printf("\n[MOTOR] MODULAR SAFETY VER (SECURITY APPLIED)\n"); 
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
    
    // 모듈 초기화
    Motor_HW_Init(); 
    Encoder_HW_Init();
    CAN_Manager_Init();
    
    // Sync Pin Init
    #ifndef GPIO_PULLUP
    #define GPIO_PULLUP 0 
    #endif
    GPIO_Config(SYNC_GPIO_PIN, (GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP));
    
    g_TimeBaseOffset = Get_Sys_Tick_Safe();
    g_Drive.lastCmdTick = Get_Sys_Tick_Safe();

    // 태스크 생성
    SAL_TaskCreate(&TaskCanID, (const uint8 *)"CAN Task", (SALTaskFunc)&Task_CAN_Rx, 
                   &TaskCanStk[0], TASK_STK_SIZE, TASK_CAN_PRIO, NULL);
    SAL_TaskCreate(&TaskSyncID, (const uint8 *)"Sync Task", (SALTaskFunc)&Task_TimeSync, 
                   &TaskSyncStk[0], TASK_STK_SIZE, TASK_SYNC_PRIO, NULL);
    SAL_TaskCreate(&TaskMotorID, (const uint8 *)"Motor Task", (SALTaskFunc)&Task_MotorControl, 
                   &TaskMotorStk[0], TASK_STK_SIZE, TASK_MOTOR_PRIO, NULL);
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
        SAL_TaskSleep(10);
    }
}

static void Task_MotorControl(void *pArg) {
    (void)pArg;
    while(1) {
        // 1. AEB 체크 (Hard Stop은 CAN Manager에서 호출하지만, 여기서도 지속 방어)
        if (g_aeb_active) {
            Motor_Emergency_Stop();
            SAL_TaskSleep(CONTROL_PERIOD_MS);
            continue;
        }

        // 2. Timeout 체크
        uint32 now = Get_Sys_Tick_Safe();
        if ((now > g_Drive.lastCmdTick) && (now - g_Drive.lastCmdTick > CMD_TIMEOUT_MS)) {
            g_Drive.targetF = 0; g_Drive.targetR = 0;
        }

        // 3. 모터 제어 수행
        Motor_Process_Drive();

        // 4. 엔코더 계산 및 전송
        Encoder_Update_Calc((float)CONTROL_PERIOD_MS);
        
        uint32 debug_time = now - g_TimeBaseOffset;
        CAN_Send_Telemetry(debug_time);
        
        SAL_TaskSleep(CONTROL_PERIOD_MS); 
    }
}