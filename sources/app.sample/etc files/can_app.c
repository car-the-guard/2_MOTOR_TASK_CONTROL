// #include <sal_api.h>
// #include <app_cfg.h>
// #include <bsp.h>
// #include <debug.h>
// #include <string.h>

// #include "can_config.h"
// #include "can_reg.h"
// #include "can.h"
// #include "can_drv.h"
// #include "can_porting.h"

// #include "can_app.h" 
// #include "timestamp.h" 

// // 설정
// #define CAN_CHANNEL            0
// #define CAN_TASK_STK_SIZE     (2048)
// #define CAN_TASK_PRIO         (SAL_PRIO_APP_CFG)

// // 전역 변수
// uint32 g_canTxQueueHandle = 0; // Tx는 큐 사용 유지 (보내는 건 안전하므로)
// static uint32 g_can_tx_task_id = 0;
// static uint32 g_can_tx_task_stk[CAN_TASK_STK_SIZE];
// static uint32 g_can_rx_task_id = 0;
// static uint32 g_can_rx_task_stk[CAN_TASK_STK_SIZE];
// static uint8 g_can_init_done = 0;

// // 디버깅용 카운터 (메시지 수신 횟수)
// volatile uint32 g_can_rx_cnt = 0;

// // 내부 함수
// static void CAN_TxTask_Loop(void *pArg);
// static void CAN_RxTask_Loop(void *pArg);
// static void CAN_TxCallback(uint8 ucCh, CANTxInterruptType_t uiIntType);
// static void CAN_ErrorCallback(uint8 ucCh, CANErrorType_t uiError);

// // -----------------------------------------------------------
// // 큐/풀 관련 (Tx용)
// // -----------------------------------------------------------
// #define CAN_TX_QUEUE_SIZE 10
// #define CAN_TX_POOL_SIZE 16
// static CAN_queue_pkt_t g_canTxPool[CAN_TX_POOL_SIZE];
// static uint8 g_canTxPoolUsed[CAN_TX_POOL_SIZE];

// CAN_queue_pkt_t* CAN_AllocPool(void) {
//     SAL_CoreCriticalEnter();
//     for (uint8 i = 0; i < CAN_TX_POOL_SIZE; i++) {
//         if (g_canTxPoolUsed[i] == 0) {
//             g_canTxPoolUsed[i] = 1;
//             SAL_CoreCriticalExit();
//             return &g_canTxPool[i];
//         }
//     }
//     SAL_CoreCriticalExit();
//     return NULL;
// }

// void CAN_FreePool(CAN_queue_pkt_t *pPkt) {
//     if (pPkt == NULL) return;
//     SAL_CoreCriticalEnter();
//     uint32 index = (uint32)(pPkt - &g_canTxPool[0]);
//     if (index < CAN_TX_POOL_SIZE) {
//         g_canTxPoolUsed[index] = 0;
//         SAL_MemSet(pPkt, 0, sizeof(CAN_queue_pkt_t));
//     }
//     SAL_CoreCriticalExit();
// }

// // -----------------------------------------------------------
// // Callbacks
// // -----------------------------------------------------------
// // ★ Rx Callback 제거함 (폴링으로 처리)
// static void CAN_TxCallback(uint8 ucCh, CANTxInterruptType_t uiIntType) { (void)ucCh; (void)uiIntType; }
// static void CAN_ErrorCallback(uint8 ucCh, CANErrorType_t uiError) { (void)ucCh; (void)uiError; }

// // -----------------------------------------------------------
// // Init
// // -----------------------------------------------------------
// void CAN_init(void) {
//     if (g_can_init_done) return;

//     mcu_printf("[CAN] Initializing...\n");

//     // Tx용 큐/풀 초기화
//     SAL_MemSet(g_canTxPool, 0, sizeof(g_canTxPool));
//     SAL_MemSet(g_canTxPoolUsed, 0, sizeof(g_canTxPoolUsed));
//     SAL_QueueCreate(&g_canTxQueueHandle, (const uint8 *)"CAN_TxQueue", CAN_TX_QUEUE_SIZE, sizeof(CAN_queue_pkt_t*));
    
//     // CAN HW Init
//     if (CAN_Init() != CAN_ERROR_NONE) {
//         CAN_Deinit();
//         CAN_Init();
//     }
    
//     // 콜백 등록 (Rx는 등록 안 함 or NULL)
//     CAN_RegisterCallbackFunctionTx(CAN_TxCallback);
//     // CAN_RegisterCallbackFunctionRx(NULL); // 명시적 해제
//     CAN_RegisterCallbackFunctionError(CAN_ErrorCallback);

//     CAN_InitMessage(CAN_CHANNEL);
//     CAN_SetControllerMode(CAN_CHANNEL, CAN_MODE_OPERATION);

//     g_can_init_done = 1;
//     mcu_printf("[CAN] Init Done & Started\n");
// }

// // -----------------------------------------------------------
// // Tx Task (기존 유지)
// // -----------------------------------------------------------
// static void CAN_TxTask_Loop(void *pArg) {
//     (void)pArg;
//     CANMessage_t sTxMsg;
//     CAN_queue_pkt_t *rxPacket;
//     uint32 uiSizeCopied;
//     uint8 ucTxBufferIndex;

//     SAL_TaskSleep(100);
//     mcu_printf("[CAN] Tx Task Started\n");

//     SAL_MemSet(&sTxMsg, 0, sizeof(CANMessage_t));
//     sTxMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;
//     sTxMsg.mDataLength = 8;

//     for(;;) {
//         if (SAL_QueueGet(g_canTxQueueHandle, &rxPacket, &uiSizeCopied, 0, 0) == SAL_RET_SUCCESS) {
//             if (rxPacket != NULL) {
//                 sTxMsg.mId = rxPacket->id;
//                 uint32_t timestamp_value = 0;
//                 TIMESTAMP_get_ms(&timestamp_value);
//                 rxPacket->body.field.time_ms = (uint16_t)(timestamp_value & 0xFFFF);
//                 SAL_MemCopy(sTxMsg.mData, rxPacket->body.raw, 8);
//                 CAN_SendMessage(CAN_CHANNEL, &sTxMsg, &ucTxBufferIndex);
//                 CAN_FreePool(rxPacket);
//             }
//         } else {
//             SAL_TaskSleep(1);
//         }
//     }
// }

// // -----------------------------------------------------------
// // ★ Rx Task (직접 폴링 방식으로 변경)
// // -----------------------------------------------------------
// static void CAN_RxTask_Loop(void *pArg) {
//     (void)pArg;
//     CANMessage_t sRxMsg;
//     CAN_payload_t payload;

//     mcu_printf("[CAN] Rx Task Started (Polling Mode)\n");

//     for(;;) {
//         // 1. 하드웨어에게 직접 물어봄 "새 메시지 있니?"
//         // (CAN_GetNewRxMessage는 메시지가 없으면 에러 리턴함)
//         if (CAN_GetNewRxMessage(CAN_CHANNEL, &sRxMsg) == CAN_ERROR_NONE) 
//         {
//             // 2. 메시지 있음 -> 카운트 증가 & 처리
//             g_can_rx_cnt++;
            
//             // 데이터 복사
//             SAL_MemCopy(payload.raw, sRxMsg.mData, 8);
            
//             // main.c로 전달
//             CAN_consume_rx_message(&sRxMsg, payload);
//         }
//         else 
//         {
//             // 3. 메시지 없음 -> 1ms 쉼 (CPU 양보)
//             SAL_TaskSleep(1);
//         }
//     }
// }

// // -----------------------------------------------------------
// // Start
// // -----------------------------------------------------------
// void CAN_TX_start_task(void) {
//     CAN_init();
//     SAL_TaskCreate(&g_can_tx_task_id, (const uint8 *)"CAN Tx", (SALTaskFunc)CAN_TxTask_Loop, &g_can_tx_task_stk[0], CAN_TASK_STK_SIZE, CAN_TASK_PRIO, NULL);
// }

// void CAN_RX_start_task(void) {
//     CAN_init();
//     SAL_TaskCreate(&g_can_rx_task_id, (const uint8 *)"CAN Rx", (SALTaskFunc)CAN_RxTask_Loop, &g_can_rx_task_stk[0], CAN_TASK_STK_SIZE, CAN_TASK_PRIO, NULL);
// }

// void CAN_start_task(void) {
//     CAN_TX_start_task();
//     CAN_RX_start_task();
// }