// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
* FileName : main.c (RTOS Version)
*
* Copyright (c) Telechips Inc.
*
***************************************************************************************************
*/

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#include <main.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <uart.h>
#include <gpio.h>
#include <pdm.h>
#include <string.h>

/*
***************************************************************************************************
* DEFINES & MACROS
***************************************************************************************************
*/
// --- Task Configuration ---
#define TASK_BT_PRIO            (SAL_PRIO_APP_CFG + 1) // 우선순위: 낮음
#define TASK_MOTOR_PRIO         (SAL_PRIO_APP_CFG + 2) // 우선순위: 높음 (모터 제어는 실시간성 중요)
#define TASK_STK_SIZE           (512)                  // 스택 사이즈 (상황에 맞춰 조정)

// --- Hardware Pinmap ---
#define BLUETOOTH_UART_CH       (0)             // UART Ch0
#define HC06_BAUDRATE           (9600)
#define CMD_BUF_SIZE            (64)

#define USE_PMM_MONITOR         (0UL)
#define GPIO_PERICH_CH0         (0UL)

// Motor Pinmap
#define MOTOR_L_PWM_CH          (0)
#define MOTOR_L_IN1             GPIO_GPB(23)
#define MOTOR_L_IN2             GPIO_GPB(21)

#define MOTOR_R_PWM_CH          (1)
#define MOTOR_R_IN1             GPIO_GPB(24)
#define MOTOR_R_IN2             GPIO_GPB(22)

// Control Parameters
#define PWM_PERIOD_NS           (1000000UL)
#define MAX_DUTY_SCALE          (1000UL)
#define TURN_SENSITIVITY        (1.0f)

/*
***************************************************************************************************
* GLOBAL VARIABLES
***************************************************************************************************
*/
uint32                          gALiveMsgOnOff;
static uint32                   gALiveCount;

// Shared Variables (Volatile 필수)
volatile int32 g_TargetF = 0;   // Forward
volatile int32 g_TargetR = 0;   // Rotation

// Motor Internal State
static uint32 g_LastDutyL = 0xFFFF;
static uint32 g_LastDutyR = 0xFFFF;

// Task Stacks & IDs
static uint32 TaskBtID;
static uint32 TaskBtStk[TASK_STK_SIZE];

static uint32 TaskMotorID;
static uint32 TaskMotorStk[TASK_STK_SIZE];

/*
***************************************************************************************************
* FUNCTION PROTOTYPES
***************************************************************************************************
*/
// OS Core Functions
static void Main_StartTask(void * pArg);
static void AppTaskCreate(void);
static void DisplayAliveLog(void);
static void DisplayOTPInfo(void);

// App Tasks
static void Task_Bluetooth(void *pArg);
static void Task_MotorControl(void *pArg);

// Logic Helpers
static void Motor_HW_Init(void);
static void Init_Bluetooth_UART(void);
static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale);
static void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed);
static void Car_Update_Logic(void);
static void Parse_Command(char *cmd);
static int32 Simple_Atoi(char *str);
static float Clamp_Val(float x, float lo, float hi);

/*
***************************************************************************************************
* MAIN ENTRY
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

    mcu_printf("\n===============================\n");
    mcu_printf("    RC Car RTOS Ver Start      \n");
    mcu_printf("===============================\n");
    DisplayOTPInfo();

    // Start Task 생성
    err = (SALRetCode_t)SAL_TaskCreate(&AppTaskStartID,
                                     (const uint8 *)"App Task Start",
                                     (SALTaskFunc) &Main_StartTask,
                                     &AppTaskStartStk[0],
                                     ACFG_TASK_MEDIUM_STK_SIZE,
                                     SAL_PRIO_APP_CFG,
                                     NULL);

    if (err == SAL_RET_SUCCESS)
    {
        (void)SAL_OsStart(); // 스케줄러 시작 (리턴 안 함)
    }
}

/*
***************************************************************************************************
* TASKS
***************************************************************************************************
*/

// [Task 1] Main Start Task: 시스템 초기화 및 태스크 생성, Alive 로그
static void Main_StartTask(void * pArg)
{
    (void)pArg;
    (void)SAL_OsInitFuncs();

    // 여기서 하드웨어 초기화 및 태스크 생성을 수행
    AppTaskCreate();

    while (1)
    {
        DisplayAliveLog();
        (void)SAL_TaskSleep(5000); // 5초마다 생존 신고
    }
}

// [Task 2] Bluetooth Task: UART 입력을 받아 명령어를 파싱
static void Task_Bluetooth(void *pArg)
{
    (void)pArg;
    uint8 rx_char;
    char cmd_buf[CMD_BUF_SIZE];
    uint8 cmd_len = 0;

    mcu_printf("[Task] Bluetooth Ready\n");

    while(1)
    {
        // Polling 모드지만, RTOS에서는 데이터가 없으면 잠시 자야(Sleep) 합니다.
        // 그렇지 않으면 이 태스크가 CPU를 독점하여 다른 태스크가 실행되지 못합니다.
        
        if (UART_Read(BLUETOOTH_UART_CH, &rx_char, 1) > 0)
        {
            // 데이터 수신 시
            if (rx_char == '\n' || rx_char == '\r')
            {
                if (cmd_len > 0)
                {
                    cmd_buf[cmd_len] = '\0';
                    mcu_printf("[BT RX] %s\n", cmd_buf);
                    Parse_Command(cmd_buf); // 전역 변수(g_TargetF 등) 업데이트
                    cmd_len = 0;
                }
            }
            else
            {
                if (cmd_len < (CMD_BUF_SIZE - 1)) 
                    cmd_buf[cmd_len++] = (char)rx_char;
                else 
                    cmd_len = 0; // 버퍼 오버플로우 방지
            }
        }
        else
        {
            // 데이터가 없으면 10ms 정도 Sleep하여 다른 태스크(모터 등)에 CPU 양보
            SAL_TaskSleep(10);
        }
    }
}

// [Task 3] Motor Task: 주기적으로 목표 속도를 반영하여 모터 구동
static void Task_MotorControl(void *pArg)
{
    (void)pArg;
    
    mcu_printf("[Task] Motor Control Ready\n");

    while(1)
    {
        // 1. 모터 로직 수행
        Car_Update_Logic();

        // 2. 주기 설정 (예: 20ms = 50Hz)
        // Delay_Loop 대신 OS Sleep을 사용해야 멀티태스킹이 됩니다.
        SAL_TaskSleep(20); 
    }
}

static void AppTaskCreate(void)
{
    SALRetCode_t err;

    // 1. 하드웨어 초기화 (태스크 실행 전 미리 수행)
    Motor_HW_Init();
    Init_Bluetooth_UART();

    // 2. Bluetooth Task 생성
    err = SAL_TaskCreate(&TaskBtID,
                         (const uint8 *)"BT Task",
                         (SALTaskFunc)&Task_Bluetooth,
                         &TaskBtStk[0],
                         TASK_STK_SIZE,
                         TASK_BT_PRIO,
                         NULL);
    if(err != SAL_RET_SUCCESS) mcu_printf("BT Task Create Fail!\n");
    else mcu_printf("BT Task is Created\n");

    // 3. Motor Task 생성
    err = SAL_TaskCreate(&TaskMotorID,
                         (const uint8 *)"Motor Task",
                         (SALTaskFunc)&Task_MotorControl,
                         &TaskMotorStk[0],
                         TASK_STK_SIZE,
                         TASK_MOTOR_PRIO,
                         NULL);
    if(err != SAL_RET_SUCCESS) mcu_printf("Motor Task Create Fail!\n");
    else mcu_printf("Motor Task is Created\n");
}

/*
***************************************************************************************************
* HELPER FUNCTIONS
***************************************************************************************************
*/

static void Init_Bluetooth_UART(void)
{
    UartParam_t uartParam;
    UART_Close(BLUETOOTH_UART_CH);
    
    uartParam.sCh           = BLUETOOTH_UART_CH;
    uartParam.sPriority     = 10U;
    uartParam.sBaudrate     = HC06_BAUDRATE;
    uartParam.sMode         = UART_POLLING_MODE; // RTOS에서도 Polling+Sleep 조합 사용 가능
    uartParam.sCtsRts       = UART_CTSRTS_OFF;
    uartParam.sPortCfg      = 0;
    uartParam.sWordLength   = WORD_LEN_8;
    uartParam.sFIFO         = ENABLE_FIFO;
    uartParam.s2StopBit     = TWO_STOP_BIT_OFF;
    uartParam.sParity       = 0;
    uartParam.sFnCallback   = NULL;
    
    UART_Open(&uartParam);
}

void Motor_HW_Init(void)
{
    GPIO_Config(MOTOR_L_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_L_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN1, GPIO_FUNC(0) | GPIO_OUTPUT);
    GPIO_Config(MOTOR_R_IN2, GPIO_FUNC(0) | GPIO_OUTPUT);

    PDM_Init();

    g_LastDutyL = 0xFFFF;
    g_LastDutyR = 0xFFFF;
    Update_PWM_Duty(MOTOR_L_PWM_CH, 0);
    Update_PWM_Duty(MOTOR_R_PWM_CH, 0);
}

static void Update_PWM_Duty(uint32 channel, uint32 duty_1000_scale)
{
    PDMModeConfig_t pdmConfig;
    uint32 *lastDutyPtr = (channel == MOTOR_L_PWM_CH) ? &g_LastDutyL : &g_LastDutyR;
    uint32 wait_cnt = 0;

    if (*lastDutyPtr == duty_1000_scale) return;
    *lastDutyPtr = duty_1000_scale;

    memset(&pdmConfig, 0, sizeof(PDMModeConfig_t));
    pdmConfig.mcPortNumber      = GPIO_PERICH_CH0;
    pdmConfig.mcOperationMode   = PDM_OUTPUT_MODE_PHASE_1;
    pdmConfig.mcOutputCtrl      = 1;
    pdmConfig.mcClockDivide     = 0;
    pdmConfig.mcPeriodNanoSec1  = PWM_PERIOD_NS;
    
    if(duty_1000_scale > MAX_DUTY_SCALE) duty_1000_scale = MAX_DUTY_SCALE;
    pdmConfig.mcDutyNanoSec1    = duty_1000_scale * (PWM_PERIOD_NS / MAX_DUTY_SCALE);

    PDM_Disable(channel, USE_PMM_MONITOR);

    // 하드웨어 Busy 체크 (짧은 시간이라 Busy Wait 유지)
    wait_cnt = 0;
    while(PDM_GetChannelStatus(channel) == PDM_STATE_BUSY)
    {
        wait_cnt++;
        if(wait_cnt > 10000) break;
        __asm("nop");
    }

    PDM_SetConfig(channel, &pdmConfig);

    if (duty_1000_scale > 0)
    {
        PDM_Enable(channel, USE_PMM_MONITOR);
    }
}

void Motor_Set_VCP(uint32 channel, uint32 p1, uint32 p2, float speed)
{
    uint32 duty = 0;
    float abs_speed = 0.0f;

    if (speed > 0)
    {
        GPIO_Set(p1, 1);
        GPIO_Set(p2, 0);
        abs_speed = speed;
    }
    else if (speed < 0)
    {
        GPIO_Set(p1, 0);
        GPIO_Set(p2, 1);
        abs_speed = -speed;
    }
    else
    {
        GPIO_Set(p1, 0);
        GPIO_Set(p2, 0);
        abs_speed = 0;
    }

    duty = (uint32)(abs_speed * 10.0f);
    Update_PWM_Duty(channel, duty);
}

void Car_Update_Logic(void)
{
    // 전역 변수 읽기 (Context Switch 방지를 위해 로컬 복사 후 연산 권장되나 단순 제어는 OK)
    float F = (float)g_TargetF;
    float R = (float)g_TargetR;

    float left_val  = F + (R * TURN_SENSITIVITY);
    float right_val = F - (R * TURN_SENSITIVITY);

    left_val  = Clamp_Val(left_val,  -100.0f, 100.0f);
    right_val = Clamp_Val(right_val, -100.0f, 100.0f);

    Motor_Set_VCP(MOTOR_L_PWM_CH, MOTOR_L_IN1, MOTOR_L_IN2, left_val);
    Motor_Set_VCP(MOTOR_R_PWM_CH, MOTOR_R_IN1, MOTOR_R_IN2, right_val);
}

void Parse_Command(char *cmd)
{
    int32 i = 0;
    int32 val = 0;

    while (cmd[i] != '\0')
    {
        if (cmd[i] == 'F') { // Forward
            val = Simple_Atoi(&cmd[i + 1]);
            g_TargetF = val;
        }
        else if (cmd[i] == 'B') { // Back
            val = Simple_Atoi(&cmd[i + 1]);
            g_TargetF = -val;
        }
        else if (cmd[i] == 'R') { // Right
            val = Simple_Atoi(&cmd[i + 1]);
            g_TargetR = val;
        }
        else if (cmd[i] == 'L') { // Left
            val = Simple_Atoi(&cmd[i + 1]);
            g_TargetR = -val;
        }
        i++;
    }
}

int32 Simple_Atoi(char *str)
{
    int32 res = 0;
    int32 i = 0;
    while (str[i] >= '0' && str[i] <= '9')
    {
        res = res * 10 + (str[i] - '0');
        i++;
    }
    return res;
}

static float Clamp_Val(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void DisplayAliveLog(void)
{
    if (gALiveMsgOnOff != 0U)
    {
        mcu_printf("\n %d", gALiveCount);

        gALiveCount++;

        if(gALiveCount >= MAIN_UINT_MAX_NUM)
        {
            gALiveCount = 0;
        }
    }
    else
    {
        gALiveCount = 0;
    }
}

#define LDT1_AREA_ADDR  0xA1011800U
#define PMU_REG_ADDR    0xA0F28000U
static void DisplayOTPInfo(void)
{
    volatile uint32 *ldt1Addr;
    volatile uint32 *chipNameAddr;
    volatile uint32 *remapAddr;
    volatile uint32 *hsmStatusAddr;
    uint32          chipName = 0;
    uint32          dualBankVal = 0;
    uint32          dual_bank = 0;
    uint32          expandFlashVal = 0;
    uint32          expand_flash = 0;
    uint32          remap_mode = 0;
    uint32          hsm_ready = 0;

    //----------------------------------------------------------------
    // OTP LDT1 Read
    // [11:0]Dual_Bank_Selection, [59:48]EXPAND_FLASH
    // Dual_Bank_Sel: [0xC0][11: 0] & [0xD0][11: 0] & [0xE0][11: 0] & [0xF0][11: 0]
    // EXPAND_FLASH : [0xC4][27:16] & [0xD4][27:16] & [0xE4][27:16] & [0xF4][27:16]
    // HwMC_PRG_FLS_LDT1: 0xA1011800

    ldt1Addr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x00C0);
    chipNameAddr = (volatile uint32 *)(LDT1_AREA_ADDR + 0x0300);
    remapAddr = (volatile uint32 *)(PMU_REG_ADDR);
    hsmStatusAddr = (volatile uint32 *)(PMU_REG_ADDR + 0x0020);

    chipName = *chipNameAddr;
    chipName &= 0x000FFFFF;

    dualBankVal = ldt1Addr[ 0];
    expandFlashVal = ldt1Addr[ 1];

    dualBankVal &= ldt1Addr[ 4];
    expandFlashVal &= ldt1Addr[ 5];

    dualBankVal &= ldt1Addr[ 8];
    expandFlashVal &= ldt1Addr[ 9];

    dualBankVal &= ldt1Addr[12];
    expandFlashVal &= ldt1Addr[13];

    dualBankVal = (dualBankVal >> 0) & 0x0FFF;
    expandFlashVal  = (expandFlashVal >> 16) & 0x0FFF;

    dual_bank = (dualBankVal == 0x0FFF) ? 0 : 1;            // (single_bank : dual_bank)
    expand_flash  = (expandFlashVal  == 0x0000) ? 0 : 1;    // (only_eFlash : use_extSNOR)

    remap_mode = remapAddr[ 0];

    mcu_printf("    CHIP   NAME  : %x\n",    chipName);
    mcu_printf("    DUAL   BANK  : %d\n",    dual_bank);
    mcu_printf("    EXPAND FLASH : %d\n",    expand_flash);
    mcu_printf("    REMAP  MODE  : %d\n",    (remap_mode >> 16));

    hsm_ready = hsmStatusAddr[ 0];
    hsm_ready = (hsm_ready >> 2) & 0x0001;
#if 0
    if(hsm_ready)
    {
        mcu_printf("    HSM    READY : %d\n",    hsm_ready);
    }
    else
    {
        while(hsm_ready != 1)
        {
            mcu_printf("    HSM    READY : %d\n",    hsm_ready);
            mcu_printf("    wait...\n");
            hsm_ready = (hsm_ready >> 2) & 0x0001;
        }
    }
#else
    mcu_printf("    HSM    READY : %d\n",    hsm_ready);
#endif
}


#endif  // ( MCU_BSP_SUPPORT_APP_BASE == 1 )