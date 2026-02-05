// encoder_driver.h
#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H
#include "app_types.h"

// 사양 상수
#define COUNTS_PER_REV      (210.0f) 
#define WHEEL_DIA_M         (0.13f)  
#define PI                  (3.141592f)

void Encoder_HW_Init(void);
void Encoder_Update_Calc(float dt_ms); // 주기적으로 호출하여 속도 계산

#endif // ENCODER_DRIVER_H