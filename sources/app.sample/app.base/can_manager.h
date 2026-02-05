// can_manager.h
#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H
#include "app_types.h"

// CAN ID 정의
#define CAN_ID_AEB          (0x10)
#define CAN_ID_DRIVE_CMD    (0x12)
#define CAN_ID_ACCEL        (0x28)
#define CAN_ID_SPEED        (0x38)

void CAN_Manager_Init(void);
void Task_CAN_Rx(void *pArg); // RX Task 함수
void CAN_Send_Telemetry(uint32 time_ms); // TX 함수

// Safe Tick 함수 (main에서 가져옴)
uint32 Get_Sys_Tick_Safe(void); 

#endif // CAN_MANAGER_H