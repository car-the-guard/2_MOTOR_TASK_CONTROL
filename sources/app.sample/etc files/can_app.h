#ifndef CAN_APP_H
#define CAN_APP_H

#include "can_config.h"
#include "can.h"
#include <stdint.h>

// 데이터 구조체 (사용자 코드와 동일)
typedef union {
    struct {
        uint8_t raw[8];
    };
    struct {
        uint8_t data[6];
        uint16_t time_ms; // 4, 5번 바이트
    } field;
} CAN_payload_t;

// 큐 패킷
typedef struct {
    uint32_t id;
    CAN_payload_t body;
} CAN_queue_pkt_t;

// 함수 선언
void CAN_init(void);
void CAN_start_task(void); // Tx, Rx 동시 시작
void CAN_TX_start_task(void);
void CAN_RX_start_task(void);

CAN_queue_pkt_t* CAN_AllocPool(void);
void CAN_FreePool(CAN_queue_pkt_t *pPkt);

// Rx 처리 훅 (main.c에서 구현)
void CAN_consume_rx_message(CANMessage_t *pMsg, CAN_payload_t payload);

extern uint32 g_canTxQueueHandle;

#endif