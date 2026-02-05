#include "timestamp.h"

// SAL API 사용을 위한 필수 헤더들
#include <main.h>
#include <app_cfg.h>
#include <bsp.h>
#include <sal_api.h>

void TIMESTAMP_get_ms(uint32_t *ms)
{
    if (ms != 0)
    {
        // RTOS 시스템 틱 가져오기
        SAL_GetTickCount(ms);
    }
}