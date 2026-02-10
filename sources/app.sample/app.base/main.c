// sources/app.sample/app.base/main.c
// SPDX-License-Identifier: Apache-2.0
/*
***************************************************************************************************
* FileName : main.c (Integrated CAN Motor Control)
* Description:
* - CAN RX/TX logic integrated into main.c (No external manager)
* - Security (MAC) removed for simplicity
* - Failsafe: Stops motor if no command received for 200ms
* - Interrupt-based CAN reception using Queue
***************************************************************************************************
*/

#include <main.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <gpio.h>
#include <string.h>

// [모듈 헤더 포함]
#include "app_types.h"
#include "motor_driver.h"
#include "encoder_driver.h"

// [CAN 드라이버 헤더]
#include "can_config.h"
#include "can_reg.h"
#include "can.h"
#include "can_drv.h"

// [설정]
#define TASK_STK_SIZE       (1024) 
#define TASK_CAN_PRIO       (SAL_PRIO_APP_CFG + 5) 
#define TASK_SYNC_PRIO      (SAL_PRIO_APP_CFG + 4) 
#define TASK_MOTOR_PRIO     (SAL_PRIO_APP_CFG + 2) 
#define CONTROL_PERIOD_MS   (50)
#define CMD_TIMEOUT_MS      (200UL) // 200ms 동안 명령 없으면 정지
#define SYNC_GPIO_PIN       GPIO_GPC(3)

// [CAN 설정]
#define CAN_RX_QUEUE_SIZE   (128)
#define CAN_CH_CMD          (0)
#define CAN_ID_AEB          (0x10)
#define CAN_ID_DRIVE_CMD    (0x12)
#define CAN_ID_ACCEL        (0x28)
#define CAN_ID_SPEED        (0x38)

// [전역 변수]
uint32 gALiveMsgOnOff = 1;
DriveCtrl_t g_Drive = {0, 0, 0};
Encoder_t Enc1;
volatile uint8 g_aeb_active = 0;
static volatile uint32 g_TimeBaseOffset = 0;

// CAN 내부 변수
static uint32 g_canRxQueueHandle = 0;
static uint32 g_rx_drop_cnt = 0;
static uint8 g_tx_counter = 0;

// [Test] 통계 및 디버깅 변수 추가
static int32 g_last_cmd_counter = -1;
static uint32 g_total_packets = 0;
static uint32 g_lost_packets = 0;

// [함수 선언]
static void Main_StartTask(void * pArg);
static void AppTaskCreate(void);
static void Task_MotorControl(void *pArg);
static void Task_TimeSync(void *pArg);
static void Task_CAN_Rx(void *pArg);
static void Init_CAN_HW(void);
static void Process_Packet(CAN_RxPacket_t *pkt);
static void CAN_Send_Telemetry(uint32 time_ms);

// [CAN 콜백]
static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError);
static void CAN_AppCallbackTxEvent(uint8 ucCh, CANTxInterruptType_t uiIntType) { (void)ucCh; (void)uiIntType; }
static void CAN_AppCallbackErrorEvent(uint8 ucCh, CANErrorType_t uiError) { (void)ucCh; (void)uiError; }

// [유틸리티]
uint32 Get_Sys_Tick_Safe(void) { uint32 tick; SAL_GetTickCount(&tick); return tick; }

// ============================================================================
// Main Entry
// ============================================================================
void cmain (void) {
    static uint32 AppTaskStartID = 0;
    static uint32 AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    (void)SAL_Init(); BSP_PreInit(); BSP_Init();
    
    mcu_printf("\n[MOTOR] INTEGRATED CAN VER (NO SECURITY / FAILSAFE ON)\n"); 
    
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
    
    // 1. 하드웨어 초기화
    Motor_HW_Init(); 
    Encoder_HW_Init();
    Init_CAN_HW(); // CAN 초기화 (main 내부 함수)
    
    // Sync Pin 초기화
    #ifndef GPIO_PULLUP
    #define GPIO_PULLUP 0 
    #endif
    GPIO_Config(SYNC_GPIO_PIN, (GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP));
    
    g_TimeBaseOffset = Get_Sys_Tick_Safe();
    g_Drive.lastCmdTick = Get_Sys_Tick_Safe();

    // 2. 태스크 생성
    SAL_TaskCreate(&TaskCanID, (const uint8 *)"CAN Task", (SALTaskFunc)&Task_CAN_Rx, 
                   &TaskCanStk[0], TASK_STK_SIZE, TASK_CAN_PRIO, NULL);
                   
    SAL_TaskCreate(&TaskSyncID, (const uint8 *)"Sync Task", (SALTaskFunc)&Task_TimeSync, 
                   &TaskSyncStk[0], TASK_STK_SIZE, TASK_SYNC_PRIO, NULL);
                   
    SAL_TaskCreate(&TaskMotorID, (const uint8 *)"Motor Task", (SALTaskFunc)&Task_MotorControl, 
                   &TaskMotorStk[0], TASK_STK_SIZE, TASK_MOTOR_PRIO, NULL);
}

// ============================================================================
// CAN Logic (Integrated)
// ============================================================================
static void Init_CAN_HW(void) {
    // 큐 생성
    SAL_QueueCreate(&g_canRxQueueHandle, (const uint8 *)"CAN_Q", CAN_RX_QUEUE_SIZE, sizeof(CAN_RxPacket_t));
    
    // 콜백 등록
    CAN_RegisterCallbackFunctionTx(&CAN_AppCallbackTxEvent); 
    CAN_RegisterCallbackFunctionRx(&CAN_AppCallbackRxEvent);
    CAN_RegisterCallbackFunctionError(&CAN_AppCallbackErrorEvent); 
    
    // 하드웨어 시작
    CAN_Init();
}

// [인터럽트 핸들러] 수신된 패킷을 큐에 넣음
static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError) {
    CANMessage_t sRxMsg; 
    CAN_RxPacket_t qPacket; 
    (void)uiRxIndex; (void)uiRxBufferType;

    if (uiError == CAN_ERROR_NONE && CAN_GetNewRxMessage(ucCh, &sRxMsg) == CAN_ERROR_NONE) {
        qPacket.id = sRxMsg.mId; 
        qPacket.dlc = sRxMsg.mDataLength; 
        memcpy(qPacket.data, sRxMsg.mData, 8);
        qPacket.rx_tick = Get_Sys_Tick_Safe(); // 시간 기록
        
        // Non-Blocking으로 큐에 삽입 (인터럽트 컨텍스트이므로 대기 불가)
        if (SAL_QueuePut(g_canRxQueueHandle, &qPacket, sizeof(CAN_RxPacket_t), 0, SAL_OPT_NON_BLOCKING) != SAL_RET_SUCCESS) {
            g_rx_drop_cnt++;
        }
    }
}

// [수신 태스크] 큐에서 꺼내 처리
static void Task_CAN_Rx(void *pArg) {
    (void)pArg;
    CAN_RxPacket_t rxPacket;
    uint32 uiSizeCopied;
    
    while(1) {
        // [수정] 0xFFFFFFFF (무한 대기)를 사용하여 큐가 비었을 때 에러 로그(109) 발생 방지
        // 데이터가 들어올 때까지 여기서 대기합니다.
        if (SAL_QueueGet(g_canRxQueueHandle, &rxPacket, &uiSizeCopied, 0xFFFFFFFF, SAL_OPT_BLOCKING) == SAL_RET_SUCCESS) {
            Process_Packet(&rxPacket); 
        } else {
            // 혹시라도 에러 발생 시 잠시 대기하여 CPU 점유 방지
            SAL_TaskSleep(10);
        }
    }
}

// [패킷 처리] 보안 검증 없이 명령 해석
static void Process_Packet(CAN_RxPacket_t *pkt) {
    uint8 *pData = pkt->data;
    
    // 1. AEB (긴급 제동)
    if (pkt->id == CAN_ID_AEB) {
        if (pData[0] != 0x00) {
            if(!g_aeb_active) {
                mcu_printf("!!! [RX] AEB ACTIVE !!!\n");
                
                // [Test] AEB Latency 측정 (RX 인터럽트 시점 -> 제어 함수 호출 직전)
                mcu_printf("[PERF] AEB Rx->Action Latency: %d ms\n", Get_Sys_Tick_Safe() - pkt->rx_tick);

                g_aeb_active = 1;
                Motor_Emergency_Stop(); // 즉시 하드 스톱
            }
        } else {
            if(g_aeb_active) mcu_printf("... [RX] AEB RELEASED ...\n");
            g_aeb_active = 0;
            g_Drive.lastCmdTick = Get_Sys_Tick_Safe(); // 해제 시 타임아웃 리셋
        }
    }
    // 2. 주행 명령 (보안 검증 없음)
    else if (pkt->id == CAN_ID_DRIVE_CMD) {
        if (g_aeb_active) return; // AEB 중에는 무시

        // [Test] 패킷 유실률 계산 (송신측이 Data[6]에 카운터를 보낸다고 가정)
        uint8 rx_cnt = pData[6];
        if (g_last_cmd_counter != -1) {
            int diff = (int)rx_cnt - (int)g_last_cmd_counter;
            if (diff < 0) diff += 256; // uint8 오버플로우 처리
            
            // [수정] 차이가 너무 크면(예: 200 이상) 송신측 리셋으로 간주하여 유실로 치지 않음
            if (diff > 200) {
                mcu_printf("[INFO] Sender Reset Detected? (%d -> %d)\n", g_last_cmd_counter, rx_cnt);
            }
            // 카운터가 1씩 증가해야 정상. 1보다 크면 중간에 패킷 유실
            else if (diff > 1) {
                g_lost_packets += (diff - 1);
                mcu_printf("[LOSS] Gap Detected: %d -> %d (Lost: %d)\n", g_last_cmd_counter, rx_cnt, diff - 1);
            }
        }
        g_total_packets++;
        g_last_cmd_counter = rx_cnt;

        // [Test] 주기적 통계 출력 (100패킷 마다)
        if (g_total_packets % 100 == 0) {
             mcu_printf("[STATS] Total: %d, Lost: %d\n", g_total_packets, g_lost_packets);
        }

        // 데이터 파싱: [0]=Fwd, [1]=Bwd, [2]=Left, [3]=Right
        g_Drive.targetF = (int32)pData[0] - (int32)pData[1];
        g_Drive.targetR = (int32)pData[3] - (int32)pData[2];
        
        // [중요] 마지막 명령 수신 시간 갱신 (Failsafe용)
        g_Drive.lastCmdTick = Get_Sys_Tick_Safe();
    }
}

// [송신 함수] 텔레메트리 전송 (보안 없음)
static void CAN_Send_Telemetry(uint32 time_ms) {
    CANMessage_t txMsg; 
    uint8 ucTxBufferIndex;
    uint8 payload[8];
    uint16 timestamp = (uint16)(time_ms & 0xFFFF);
    int32 val;

    // 1. SPEED 전송
    memset(&txMsg, 0, sizeof(CANMessage_t));
    val = (int32)(Enc1.speed_mps * 100.0f);
    
    payload[0] = (uint8)((val >> 24) & 0xFF);
    payload[1] = (uint8)((val >> 16) & 0xFF);
    payload[2] = (uint8)((val >> 8) & 0xFF);
    payload[3] = (uint8)(val & 0xFF);
    payload[4] = (uint8)((timestamp >> 8) & 0xFF);
    payload[5] = (uint8)(timestamp & 0xFF);
    payload[6] = g_tx_counter;
    payload[7] = 0; // MAC 없음
    
    memcpy(txMsg.mData, payload, 8);
    txMsg.mId = CAN_ID_SPEED; 
    txMsg.mDataLength = 8; 
    txMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;
    
    CAN_SendMessage(CAN_CH_CMD, &txMsg, &ucTxBufferIndex);
    g_tx_counter++;

    // 2. ACCEL 전송
    memset(&txMsg, 0, sizeof(CANMessage_t));
    val = (int32)(Enc1.accel_mps2 * 1000.0f);
    
    payload[0] = (uint8)((val >> 24) & 0xFF);
    payload[1] = (uint8)((val >> 16) & 0xFF);
    payload[2] = (uint8)((val >> 8) & 0xFF);
    payload[3] = (uint8)(val & 0xFF);
    // Timestamp 동일
    payload[6] = g_tx_counter;
    payload[7] = 0; // MAC 없음
    
    memcpy(txMsg.mData, payload, 8);
    txMsg.mId = CAN_ID_ACCEL; 
    txMsg.mDataLength = 8; 
    txMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;

    CAN_SendMessage(CAN_CH_CMD, &txMsg, &ucTxBufferIndex);
    g_tx_counter++;
}

// ============================================================================
// Other Tasks
// ============================================================================
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
        // 1. AEB 체크 (최우선)
        if (g_aeb_active) {
            Motor_Emergency_Stop();
            SAL_TaskSleep(CONTROL_PERIOD_MS);
            continue;
        }

        // 2. Timeout 체크 (Failsafe)
        // 마지막 명령 수신 후 CMD_TIMEOUT_MS(200ms)가 지나면 정지
        uint32 now = Get_Sys_Tick_Safe();
        if ((now > g_Drive.lastCmdTick) && (now - g_Drive.lastCmdTick > CMD_TIMEOUT_MS)) {
            g_Drive.targetF = 0; 
            g_Drive.targetR = 0;
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
