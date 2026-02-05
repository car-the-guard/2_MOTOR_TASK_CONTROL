// motor_driver.c
#include <main.h> // BSP 헤더 포함
#include "motor_driver.h"
#include <gpio.h>
#include <pdm.h>
#include <string.h>

// 핀맵 정의
#define MOTOR_L_PWM_CH      (0) //GPIO_A10
//주행 방향이 반대라면 입력 신호핀을 반대로 넣어준다
#define MOTOR_L_IN1         GPIO_GPB(21)
#define MOTOR_L_IN2         GPIO_GPB(23) 
#define MOTOR_R_PWM_CH      (1) //GPIO_A11
#define MOTOR_R_IN1         GPIO_GPB(24)
#define MOTOR_R_IN2         GPIO_GPB(22)
#define PWM_PIN_L           GPIO_GPA(10)
#define PWM_PIN_R           GPIO_GPA(11)
#define USE_PMM_MONITOR     (0UL)

// SDK 표준 핀 인덱스 처리
#ifndef GPIO_PERICH_CH0
#define GPIO_PERICH_CH0     (0UL)
#endif

// 내부 변수
static uint32 g_LastDutyL = 0xFFFF;
static uint32 g_LastDutyR = 0xFFFF;
static float curF = 0.0f;
static float curR = 0.0f;

// 내부 유틸 함수
static float Clamp_Val(float x, float lo, float hi) {
    if (x < lo) return lo; 
    if (x > hi) return hi; 
    return x;
}
static float rate_limit(float cur, float target, float step) {
    if (target > cur + step) return cur + step;
    if (target < cur - step) return cur - step;
    return target;
}

// PWM 업데이트 함수
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale) {
    PDMModeConfig_t pdmConfig;
    uint32 *lastDutyPtr = (channel == MOTOR_L_PWM_CH) ? &g_LastDutyL : &g_LastDutyR;
    
    if (*lastDutyPtr == duty_1000_scale) return;
    *lastDutyPtr = duty_1000_scale;

    memset(&pdmConfig, 0, sizeof(PDMModeConfig_t));
    // 포트 매핑 (Fix 1)
    pdmConfig.mcPortNumber = GPIO_PERICH_CH0; 
    
    pdmConfig.mcOperationMode = PDM_OUTPUT_MODE_PHASE_1; 
    pdmConfig.mcOutputCtrl = 1; 
    pdmConfig.mcClockDivide = 0; 
    pdmConfig.mcPeriodNanoSec1 = PWM_PERIOD_NS; 
    
    uint32 final_duty = duty_1000_scale;
    if (final_duty > 0 && final_duty < PHY_MIN_DUTY) final_duty = PHY_MIN_DUTY; 
    if(final_duty > MAX_DUTY_SCALE) final_duty = MAX_DUTY_SCALE;
    pdmConfig.mcDutyNanoSec1 = final_duty * (PWM_PERIOD_NS / MAX_DUTY_SCALE);

    PDM_Disable(channel, USE_PMM_MONITOR);
    uint32 uiCnt = 0;
    while(PDM_GetChannelStatus(channel)) {
        SAL_TaskSleep(1);
        if(++uiCnt > 20) break;
    }
    if (PDM_SetConfig(channel, &pdmConfig) == SAL_RET_SUCCESS) {
        if (final_duty > 0) PDM_Enable(channel, USE_PMM_MONITOR);
    }
}

// 방향 및 속도 설정
static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed) {
    if (speed == 0.0f) {
        GPIO_Set(p1, 0); GPIO_Set(p2, 0);
        Update_PWM_Duty(channel, 0);
        return;
    }
    if (speed > 0) { GPIO_Set(p1, 1); GPIO_Set(p2, 0); }
    else { GPIO_Set(p1, 0); GPIO_Set(p2, 1); speed = -speed; }
    
    Update_PWM_Duty(channel, (uint32)(speed * 10.0f));
}

// [공개 함수] 초기화
void Motor_HW_Init(void) {
    GPIO_Config(MOTOR_L_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_L_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(PWM_PIN_L, GPIO_FUNC(3) | GPIO_OUTPUT);
    GPIO_Config(PWM_PIN_R, GPIO_FUNC(3) | GPIO_OUTPUT);
    
    PDM_Init(); 
    Update_PWM_Duty(MOTOR_L_PWM_CH, 0); 
    Update_PWM_Duty(MOTOR_R_PWM_CH, 0);
}

// [공개 함수] 긴급 정지
void Motor_Emergency_Stop(void) {
    // 1. 목표값 초기화
    g_Drive.targetF = 0; g_Drive.targetR = 0;
    curF = 0.0f; curR = 0.0f;

    // 2. PWM 비활성화
    PDM_Disable(MOTOR_L_PWM_CH, USE_PMM_MONITOR);
    PDM_Disable(MOTOR_R_PWM_CH, USE_PMM_MONITOR);
    
    // 3. GPIO 차단
    GPIO_Set(MOTOR_L_IN1, 0); GPIO_Set(MOTOR_L_IN2, 0);
    GPIO_Set(MOTOR_R_IN1, 0); GPIO_Set(MOTOR_R_IN2, 0);
    
    g_LastDutyL = 0; g_LastDutyR = 0;
}

// [공개 함수] 주행 로직 처리
void Motor_Process_Drive(void) {
    // 1. Rate Limit 적용
    curF = rate_limit(curF, (float)g_Drive.targetF, STEP_VAL);
    curR = rate_limit(curR, (float)g_Drive.targetR, STEP_VAL);
    
    // 2. Mixing (Tank/Car Steering Logic)
    float left_val  = Clamp_Val(curF + (curR * TURN_SENSITIVITY), -100.0f, 100.0f);
    float right_val = Clamp_Val(curF - (curR * TURN_SENSITIVITY), -100.0f, 100.0f);
    
    // 3. HW 제어
    Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, left_val);
    Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, right_val);
}