/**
 * @file    core_delay.c
 * @brief   基于 DWT 内核周期计数的高精度延时实现
 * @author  Ashleycurry
 * @version 1.0.0
 */

#include "core_delay.h"

#if USE_DWT_DELAY

/* DWT 控制寄存器与内核周期计数器 */
#define DWT_CR      *(__IO uint32_t *)0xE0001000
#define DWT_CYCCNT  *(__IO uint32_t *)0xE0001004
#define DEM_CR      *(__IO uint32_t *)0xE000EDFC

#define DEM_CR_TRCENA    (1 << 24)   /* 调试跟踪使能 */
#define DWT_CR_CYCCNTENA (1 <<  0)   /* 周期计数器使能 */

/**
 * @brief  初始化 DWT 周期计数器
 */
void CPU_TS_TmrInit(void)
{
    DEM_CR    |= (uint32_t)DEM_CR_TRCENA;
    DWT_CYCCNT = (uint32_t)0u;
    DWT_CR    |= (uint32_t)DWT_CR_CYCCNTENA;
}

/**
 * @brief  读取当前 DWT 周期计数值
 */
uint32_t CPU_TS_TmrRd(void)
{
    return ((uint32_t)DWT_CYCCNT);
}

/**
 * @brief  微秒级忙等待延时
 * @param  us 延时长度（微秒）
 * @note   以周期差累加判断（兼容计数器回绕），精度约等于 1 个内核周期。
 */
void CPU_TS_Tmr_Delay_US(__IO uint32_t us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;

#if (CPU_TS_INIT_IN_DELAY_FUNCTION)
    CPU_TS_TmrInit();
#endif

    ticks = us * (GET_CPU_ClkFreq() / 1000000);
    told  = (uint32_t)CPU_TS_TmrRd();

    while (1)
    {
        tnow = (uint32_t)CPU_TS_TmrRd();
        if (tnow != told)
        {
            if (tnow > told)
                tcnt += tnow - told;
            else
                tcnt += UINT32_MAX - told + tnow;
            told = tnow;
            if (tcnt >= ticks) break;
        }
    }
}

#endif
