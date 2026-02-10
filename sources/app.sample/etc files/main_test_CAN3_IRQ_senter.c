// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
* FileName : main.c (Center Board - Controller & Monitor) - Fixed
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <string.h>

// [CAN Driver Headers]
#include "can_config.h"
#include "can_reg.h"  // [수정] 필수 헤더 추가
#include "can.h"
#include "can_drv.h"

/*
***************************************************************************************************
* DEFINES
***************************************************************************************************
*/
#define CAN_RX_QUEUE_SIZE       (64)
#define CAN_CH_CMD              (0)     // 센터 보드의 CAN 채널

// --- 프로토콜 ---
#define CAN_ID_TX_CMD           (0x106)  // [송신] 제어 명령
#define CAN_ID_RX_FEEDBACK      (0x13)  // [수신] 속도 피드백

// Task Configuration
#define TASK_CAN_RX_PRIO        (SAL_PRIO_APP_CFG + 1)
#define TASK_TEST_PRIO          (SAL_PRIO_APP_CFG + 2)
#define TASK_STK_SIZE           (1024)

/*
***************************************************************************************************
* DATA STRUCTURES
***************************************************************************************************
*/
typedef struct
{
    uint32 id;      // [수정] uint32_t -> uint32
    uint32 dlc;
    uint8  data[8]; // [수정] uint8_t -> uint8
} CAN_RxPacket_t;

/*
***************************************************************************************************
* GLOBAL VARIABLES
***************************************************************************************************
*/
// [수정] console.c 에러 방지용 변수 추가
uint32 gALiveMsgOnOff = 1;

static uint32 g_canRxQueueHandle = 0;

// Task Stacks
static uint32 TaskRxID;
static uint32 TaskRxStk[TASK_STK_SIZE];
static uint32 TaskTestID;
static uint32 TaskTestStk[TASK_STK_SIZE];

/*
***************************************************************************************************
* FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void Main_StartTask(void * pArg);
static void AppTaskCreate(void);
static void Task_CAN_Rx_Monitor(void *pArg);
static void Task_Test_Pattern(void *pArg);

static void Init_CAN_HW(void);
// [수정] int8_t -> char (signed char)
static void CAN_Send_Drive_Cmd(char speed, char trig_L, char trig_R);

// Callbacks
static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError);
static void CAN_AppCallbackTxEvent(uint8 ucCh, CANTxInterruptType_t uiIntType);
static void CAN_AppCallbackErrorEvent(uint8 ucCh, CANErrorType_t uiError);

// [수정] can_app.c 링커 에러 방지용 더미 함수
void CAN_consume_rx_message(void *pMsg, void *pPayload); 

/*
***************************************************************************************************
* MAIN
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

    mcu_printf("\n========================================\n");
    mcu_printf("   [CENTER BOARD] Controller Mode Start   \n");
    mcu_printf("   TX ID: 0x12 (CMD) | RX ID: 0x13 (FB)   \n");
    mcu_printf("========================================\n");

    err = (SALRetCode_t)SAL_TaskCreate(&AppTaskStartID, (const uint8 *)"StartTask", (SALTaskFunc)&Main_StartTask,
                                     &AppTaskStartStk[0], ACFG_TASK_MEDIUM_STK_SIZE, SAL_PRIO_APP_CFG, NULL);
    if (err == SAL_RET_SUCCESS) (void)SAL_OsStart();
}

static void Main_StartTask(void * pArg)
{
    (void)pArg;
    (void)SAL_OsInitFuncs();
    AppTaskCreate();
    while (1) { (void)SAL_TaskSleep(5000); }
}

/*
***************************************************************************************************
* TASKS
***************************************************************************************************
*/

// [Task 1] 모터 보드에서 오는 속도 피드백(0x13)을 수신하여 출력
static void Task_CAN_Rx_Monitor(void *pArg)
{
    (void)pArg;
    CAN_RxPacket_t rxPacket;
    SALRetCode_t salRet;
    uint32 uiSizeCopied;
    short speedL, speedR; // [수정] int16_t -> short

    mcu_printf("[Task] Feedback Monitor Ready\n");

    while(1)
    {
        salRet = SAL_QueueGet(g_canRxQueueHandle, &rxPacket, &uiSizeCopied, 0, SAL_OPT_BLOCKING);

        if (salRet == SAL_RET_SUCCESS)
        {
            if (rxPacket.id == CAN_ID_RX_FEEDBACK) // 0x13
            {
                // 데이터 파싱
                speedL = (short)(rxPacket.data[0] | (rxPacket.data[1] << 8));
                speedR = (short)(rxPacket.data[2] | (rxPacket.data[3] << 8));

                mcu_printf("[FB RX] L: %d cm/s | R: %d cm/s\n", speedL, speedR);
            }
        }
        else
        {
            SAL_TaskSleep(10);
        }
    }
}

// [Task 2] 주기적으로 주행 명령(0x12)을 송신 (테스트 시나리오)
static void Task_Test_Pattern(void *pArg)
{
    (void)pArg;
    uint32 step_count = 0; // [수정] uint32_t -> uint32
    
    mcu_printf("[Task] Test Pattern Generator Ready\n");
    SAL_TaskSleep(2000);

    while(1)
    {
        if (step_count < 30) // 0~3초: 전진
        {
            if(step_count == 0) mcu_printf(">>> CMD: Forward 30\n");
            CAN_Send_Drive_Cmd(30, 0, 0); 
        }
        else if (step_count < 50) // 3~5초: 정지
        {
            if(step_count == 30) mcu_printf(">>> CMD: Stop\n");
            CAN_Send_Drive_Cmd(0, 0, 0);
        }
        else if (step_count < 80) // 5~8초: 후진
        {
            if(step_count == 50) mcu_printf(">>> CMD: Backward 30\n");
            CAN_Send_Drive_Cmd(-30, 0, 0);
        }
        else if (step_count < 110) // 8~11초: 우회전
        {
            if(step_count == 80) mcu_printf(">>> CMD: Turn Right\n");
            CAN_Send_Drive_Cmd(30, 0, 50); 
        }
        else // 11초~: 리셋
        {
            step_count = -1; 
        }

        step_count++;
        SAL_TaskSleep(100); 
    }
}

static void AppTaskCreate(void)
{
    SALRetCode_t err;

    err = SAL_QueueCreate(&g_canRxQueueHandle, (const uint8 *)"CAN_Q", CAN_RX_QUEUE_SIZE, sizeof(CAN_RxPacket_t));
    if (err != SAL_RET_SUCCESS) mcu_printf("[ERR] Queue Create Fail\n");

    Init_CAN_HW();

    SAL_TaskCreate(&TaskRxID, (const uint8 *)"RxTask", (SALTaskFunc)&Task_CAN_Rx_Monitor, 
                   &TaskRxStk[0], TASK_STK_SIZE, TASK_CAN_RX_PRIO, NULL);

    SAL_TaskCreate(&TaskTestID, (const uint8 *)"TestTask", (SALTaskFunc)&Task_Test_Pattern, 
                   &TaskTestStk[0], TASK_STK_SIZE, TASK_TEST_PRIO, NULL);
}

/*
***************************************************************************************************
* CAN FUNCTION
***************************************************************************************************
*/

// 명령 패킷 송신 함수
// [수정] int8_t -> char
static void CAN_Send_Drive_Cmd(char speed, char trig_L, char trig_R)
{
    CANMessage_t txMsg;
    uint8 ucTxBufferIndex;
    
    memset(&txMsg, 0, sizeof(CANMessage_t));

    txMsg.mId = CAN_ID_TX_CMD; // 0x12
    txMsg.mDataLength = 8;     
    txMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;

    // 패킷 구조
    txMsg.mData[0] = (uint8)speed;
    txMsg.mData[1] = 0;
    txMsg.mData[2] = (uint8)trig_L;
    txMsg.mData[3] = (uint8)trig_R;
    
    txMsg.mData[4] = 0; txMsg.mData[5] = 0; txMsg.mData[6] = 0; txMsg.mData[7] = 0;

    (void)CAN_SendMessage(CAN_CH_CMD, &txMsg, &ucTxBufferIndex);
}

// 수신 인터럽트
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

// [수정] 더미 함수 추가
void CAN_consume_rx_message(void *pMsg, void *pPayload)
{
    (void)pMsg;
    (void)pPayload;
}

#endif // ( MCU_BSP_SUPPORT_APP_BASE == 1 )