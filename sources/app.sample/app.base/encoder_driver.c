// encoder_driver.c
#include <main.h>
#include "encoder_driver.h"
#include <ictc.h>
#include <gpio.h>
#include <string.h>

// 핀맵
#define ENCODER_L_ICTC_CH   (0)         
#define ENCODER_PIN_INDEX   (64UL)      
#define ENCODER_GPIO_PIN    GPIO_GPC(4) 
#define ENCODER_PIN_B_GPIO  GPIO_GPC(5) 

// 매크로 정의 (ICT C 헤더 보완)
#ifndef ICTC_IRQ_CTRL_PRDDT_CMP_ISR
#define ICTC_IRQ_CTRL_PRDDT_CMP_ISR      (1UL << 4) 
#endif
#ifndef ICTC_OPEN_CTRL_FLTCNT_EN
#define ICTC_OPEN_CTRL_FLTCNT_EN         (1UL << 0)
#define ICTC_OPEN_CTRL_PDCMPCNT_EN       (1UL << 1)
#endif
#ifndef ICTC_OPMODE_CTRL_RISING_EDGE
#define ICTC_OPMODE_CTRL_ABS_SEL         (0UL)
#define ICTC_OPMODE_CTRL_RISING_EDGE     (1UL << 1) 
#define ICTC_OPMODE_CTRL_TCLK_BYPASS     (1UL << 2)
#define ICTC_OPMODE_CTRL_FLT_IMM_F_MODE  (0UL)
#define ICTC_OPMODE_CTRL_FLT_IMM_R_MODE  (0UL)
#define ICTC_OPMODE_CTRL_PRDDT_CMP_ISR   (1UL << 5)
#define ICTC_OPMODE_CTRL_F_IN_SEL_OFFSET (8)
#endif

static float g_prev_speed_mps = 0.0f;

// 인터럽트 콜백
static void ICTC_Callback(uint32 uiChannel, uint32 uiPeriod, uint32 uiDuty) {
    (void)uiPeriod; (void)uiDuty;
    if (uiChannel == ENCODER_L_ICTC_CH) {
        uint32 stateB = GPIO_Get(ENCODER_PIN_B_GPIO); 
        if (stateB == 0) Enc1.counter++; 
        else Enc1.counter--; 
    }
}

void Encoder_HW_Init(void) {
    ICTCModeConfig_t IctcConfig; 
    memset(&IctcConfig, 0, sizeof(ICTCModeConfig_t));
    
    // 전역 변수 초기화
    Enc1.counter = 0; Enc1.last_counter = 0; 
    
    #ifndef GPIO_PULLUP
        #define GPIO_PULLUP 0 
    #endif
    GPIO_Config(ENCODER_GPIO_PIN, (GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_DS(0x3UL) | GPIO_PULLUP));
    GPIO_Config(ENCODER_PIN_B_GPIO, (GPIO_FUNC(0UL) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP));
    
    IctcConfig.mcTimeout = 0x0FFFFFFFUL;
    IctcConfig.mcEnableIrq = ICTC_IRQ_CTRL_PRDDT_CMP_ISR; 
    IctcConfig.mcEnableCounter = ICTC_OPEN_CTRL_FLTCNT_EN | ICTC_OPEN_CTRL_PDCMPCNT_EN;
    IctcConfig.mcOperationMode = ICTC_OPMODE_CTRL_ABS_SEL | ICTC_OPMODE_CTRL_RISING_EDGE | ICTC_OPMODE_CTRL_TCLK_BYPASS | 
                                 ICTC_OPMODE_CTRL_FLT_IMM_F_MODE | ICTC_OPMODE_CTRL_FLT_IMM_R_MODE | 
                                 ICTC_OPMODE_CTRL_PRDDT_CMP_ISR | (ENCODER_PIN_INDEX << ICTC_OPMODE_CTRL_F_IN_SEL_OFFSET);
    ICTC_Init();
    ICTC_SetCallBackFunc(ENCODER_L_ICTC_CH, (ICTCCallback)&ICTC_Callback);
    ICTC_SetIRQCtrlReg(ENCODER_L_ICTC_CH, IctcConfig.mcEnableIrq);
    ICTC_SetTimeoutValue(ENCODER_L_ICTC_CH, IctcConfig.mcTimeout);
    ICTC_SetEdgeMatchingValue(ENCODER_L_ICTC_CH, 0x100, 0x100, 0xFFFF);
    ICTC_SetCompareRoundValue(ENCODER_L_ICTC_CH, 0x0FFFFFF0UL, 0x0FFFFFF0UL);
    ICTC_SetOpEnCtrlCounter(ENCODER_L_ICTC_CH, IctcConfig.mcEnableCounter);
    ICTC_SetOpModeCtrlReg(ENCODER_L_ICTC_CH, IctcConfig.mcOperationMode);
    ICTC_EnableCapture(ENCODER_L_ICTC_CH);
}

void Encoder_Update_Calc(float dt_ms) {
    if (dt_ms < 0.1f) return;
    
    // 1. 속도 계산 (기존 로직)
    int32 cur_counter = Enc1.counter;
    int32 diff = cur_counter - Enc1.last_counter;
    Enc1.last_counter = cur_counter;
    Enc1.speed_rpm = ((float)diff / COUNTS_PER_REV) * (60000.0f / dt_ms);
    Enc1.speed_mps = (Enc1.speed_rpm * PI * WHEEL_DIA_M) / 60.0f;

    // 2. [복구] 가속도 계산
    // 가속도 = (현재속도 - 이전속도) / 시간(초)
    float dt_sec = dt_ms / 1000.0f;
    Enc1.accel_mps2 = (Enc1.speed_mps - g_prev_speed_mps) / dt_sec;
    
    // 3. 현재 속도를 이전 속도로 저장
    g_prev_speed_mps = Enc1.speed_mps;
}