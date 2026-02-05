// motor_driver.h
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "app_types.h"

// 모터 관련 상수
#define PWM_PERIOD_NS       (1000000UL)     
#define MAX_DUTY_SCALE      (1000UL) 
#define PHY_MIN_DUTY        (150)    
#define STEP_VAL            (50.0f)  
#define TURN_SENSITIVITY    (1.0f)   

void Motor_HW_Init(void);
void Motor_Emergency_Stop(void); // 긴급 정지
void Motor_Process_Drive(void);  // 주기적 주행 로직 (Rate Limit, Mixing 포함)

#endif // MOTOR_DRIVER_H