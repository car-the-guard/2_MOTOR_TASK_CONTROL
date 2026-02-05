// app_types.h
#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <sal_api.h> // uint32 등 타입 정의 필요

// [데이터 구조체]
typedef struct {
    int32 counter; int32 last_counter;
    float speed_rpm; 
    float speed_mps;
    float accel_mps2;
} Encoder_t;

typedef struct {
    uint32 id; uint32 dlc; uint8 data[8];
} CAN_RxPacket_t;

typedef struct {
    volatile int32 targetF;
    volatile int32 targetR;
    volatile uint32 lastCmdTick; // 타임아웃 체크용
} DriveCtrl_t;

// [전역 변수 공유 (extern)]
// 실체는 main.c에 정의됨
extern DriveCtrl_t g_Drive;
extern Encoder_t Enc1;
extern volatile uint8 g_aeb_active;

#endif // APP_TYPES_H