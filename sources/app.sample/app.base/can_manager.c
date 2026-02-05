// can_manager.c
#include <main.h>
#include "can_manager.h"
#include "can_security_utils.h" // [보안]
#include "motor_driver.h"       // AEB 시 호출

// [Fix] CAN 설정 헤더를 먼저 포함해야 함
#include "can_config.h"
#include "can_reg.h"
#include <can.h>
#include <can_drv.h> // 필요시 추가

#include <string.h>
#include <sal_api.h>
#include <debug.h>

#define CAN_RX_QUEUE_SIZE   (128)
#define CAN_CH_CMD          (0)
#define DEBUG_MODE          1

static uint32 g_canRxQueueHandle = 0;
static uint32 g_rx_drop_cnt = 0;
static uint8_t g_tx_counter = 0; // 송신용 카운터

static float g_prev_speed_mps = 0.0f;

// 수신용 보안 카운터
static int32_t g_last_cmd_counter = -1;
static int32_t g_last_aeb_counter = -1;

// [내부] 패킷 처리 및 보안 검증
static void Process_Packet(CAN_RxPacket_t *pkt) {
    uint8_t *pData = pkt->data;
    uint8_t rx_cnt = pData[6];
    uint8_t rx_mac = pData[7];

    // 1. MAC 검증
    if (verify_mac(pData, 7, rx_cnt, rx_mac) == 0) {
        #if (DEBUG_MODE == 1)
        // mcu_printf("[SEC] MAC Fail ID:0x%X\n", pkt->id);
        #endif
        return; 
    }

    // 2. ID별 로직
    if (pkt->id == CAN_ID_AEB) {
        // Anti-Replay
        if (g_last_aeb_counter != -1 && rx_cnt <= g_last_aeb_counter) {
            if (!((g_last_aeb_counter > 240) && (rx_cnt < 15))) return;
        }
        g_last_aeb_counter = rx_cnt;

        if (pData[0] != 0x00) {
            if(!g_aeb_active) {
                mcu_printf("!!! [RX] AEB ACTIVE !!!\n");
                g_aeb_active = 1;
                Motor_Emergency_Stop(); // 즉시 하드 스톱
            }
        } else {
            if(g_aeb_active) mcu_printf("... [RX] AEB RELEASED ...\n");
            g_aeb_active = 0;
            g_Drive.lastCmdTick = Get_Sys_Tick_Safe(); 
        }
    }
    else if (pkt->id == CAN_ID_DRIVE_CMD) {
        if (g_aeb_active) return;

        // Anti-Replay
        if (g_last_cmd_counter != -1 && rx_cnt <= g_last_cmd_counter) {
             if (!((g_last_cmd_counter > 240) && (rx_cnt < 15))) return;
        }
        g_last_cmd_counter = rx_cnt;

        g_Drive.targetF = (int32)pData[0] - (int32)pData[1];
        g_Drive.targetR = (int32)pData[3] - (int32)pData[2];
        g_Drive.lastCmdTick = Get_Sys_Tick_Safe();
    }
}

static void CAN_AppCallbackRxEvent(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError) {
    CANMessage_t sRxMsg; CAN_RxPacket_t qPacket; (void)uiRxIndex; (void)uiRxBufferType;
    if (uiError == CAN_ERROR_NONE && CAN_GetNewRxMessage(ucCh, &sRxMsg) == CAN_ERROR_NONE) {
        qPacket.id = sRxMsg.mId; qPacket.dlc = sRxMsg.mDataLength; memcpy(qPacket.data, sRxMsg.mData, 8);
        if (SAL_QueuePut(g_canRxQueueHandle, &qPacket, sizeof(CAN_RxPacket_t), 0, SAL_OPT_NON_BLOCKING) != SAL_RET_SUCCESS) {
            g_rx_drop_cnt++;
        }
    }
}
static void CAN_AppCallbackTxEvent(uint8 ucCh, CANTxInterruptType_t uiIntType) { (void)ucCh; (void)uiIntType; }
static void CAN_AppCallbackErrorEvent(uint8 ucCh, CANErrorType_t uiError) { (void)ucCh; (void)uiError; }

void CAN_Manager_Init(void) {
    SAL_QueueCreate(&g_canRxQueueHandle, (const uint8 *)"CAN_Q", CAN_RX_QUEUE_SIZE, sizeof(CAN_RxPacket_t));
    CAN_RegisterCallbackFunctionTx(&CAN_AppCallbackTxEvent); 
    CAN_RegisterCallbackFunctionRx(&CAN_AppCallbackRxEvent);
    CAN_RegisterCallbackFunctionError(&CAN_AppCallbackErrorEvent); 
    CAN_Init();
}

void Task_CAN_Rx(void *pArg) {
    (void)pArg;
    CAN_RxPacket_t rxPacket;
    uint32 uiSizeCopied;
    while(1) {
        if (SAL_QueueGet(g_canRxQueueHandle, &rxPacket, &uiSizeCopied, 0, SAL_OPT_BLOCKING) == SAL_RET_SUCCESS) {
            Process_Packet(&rxPacket); 
            while (SAL_QueueGet(g_canRxQueueHandle, &rxPacket, &uiSizeCopied, 0, SAL_OPT_NON_BLOCKING) == SAL_RET_SUCCESS) {
                Process_Packet(&rxPacket); 
            }
        }
    }
}

// 텔레메트리 전송 (보안 적용)
void CAN_Send_Telemetry(uint32 time_ms) {
    CANMessage_t txMsg; 
    uint8 ucTxBufferIndex;
    uint8_t payload[8];
    uint16 timestamp = (uint16)(time_ms & 0xFFFF);
    int32 val;

    // ------------------------------------------------
    // 1. SPEED 전송 (0x38)
    // ------------------------------------------------
    memset(&txMsg, 0, sizeof(CANMessage_t));
    val = (int32)(Enc1.speed_mps * 100.0f); // 스케일링
    
    payload[0] = (uint8)((val >> 24) & 0xFF);
    payload[1] = (uint8)((val >> 16) & 0xFF);
    payload[2] = (uint8)((val >> 8) & 0xFF);
    payload[3] = (uint8)(val & 0xFF);
    payload[4] = (uint8)((timestamp >> 8) & 0xFF);
    payload[5] = (uint8)(timestamp & 0xFF);
    payload[6] = g_tx_counter;
    payload[7] = compute_mac(payload, 7, g_tx_counter);
    
    memcpy(txMsg.mData, payload, 8);
    txMsg.mId = CAN_ID_SPEED; 
    txMsg.mDataLength = 8; 
    txMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;
    
    CAN_SendMessage(CAN_CH_CMD, &txMsg, &ucTxBufferIndex);
    g_tx_counter++; // 카운터 증가

    // ------------------------------------------------
    // 2. [추가] ACCEL 전송 (0x28)
    // ------------------------------------------------
    memset(&txMsg, 0, sizeof(CANMessage_t));
    val = (int32)(Enc1.accel_mps2 * 1000.0f); // 스케일링 (가속도는 정밀도가 더 필요해서 1000배)

    payload[0] = (uint8)((val >> 24) & 0xFF);
    payload[1] = (uint8)((val >> 16) & 0xFF);
    payload[2] = (uint8)((val >> 8) & 0xFF);
    payload[3] = (uint8)(val & 0xFF);
    payload[4] = (uint8)((timestamp >> 8) & 0xFF);
    payload[5] = (uint8)(timestamp & 0xFF);
    payload[6] = g_tx_counter;
    payload[7] = compute_mac(payload, 7, g_tx_counter); // 새로운 카운터로 MAC 다시 계산
    
    memcpy(txMsg.mData, payload, 8);
    txMsg.mId = CAN_ID_ACCEL; 
    txMsg.mDataLength = 8; 
    txMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;

    CAN_SendMessage(CAN_CH_CMD, &txMsg, &ucTxBufferIndex);
    g_tx_counter++; // 카운터 또 증가
}