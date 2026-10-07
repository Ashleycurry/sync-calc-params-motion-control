/**
 * @file    core_delay.h
 * @brief   基于 DWT 内核周期计数的高精度延时接口
 * @author  Ashleycurry
 * @version 1.0.0
 *
 * 使用 Cortex-M3 内核 DWT->CYCCNT 计数器实现 us/ms/s 级延时，
 * 不占用任何定时器资源，精度与内核周期同量级。
 */

#ifndef __CORE_DELAY_H
#define __CORE_DELAY_H

#include "stm32f10x.h"

#define USE_DWT_DELAY  1    /* 1：使用 DWT 延时；0：不使用 */

#if USE_DWT_DELAY
#define Delay_ms(ms)  CPU_TS_Tmr_Delay_MS(ms)
#define Delay_us(us)  CPU_TS_Tmr_Delay_US(us)
#define Delay_s(s)    CPU_TS_Tmr_Delay_S(s)

#define GET_CPU_ClkFreq()  (SystemCoreClock)
#define CPU_TS_INIT_IN_DELAY_FUNCTION  0

uint32_t CPU_TS_TmrRd(void);
void     CPU_TS_TmrInit(void);
void     CPU_TS_Tmr_Delay_US(uint32_t us);
#define  CPU_TS_Tmr_Delay_MS(ms)  CPU_TS_Tmr_Delay_US((ms)*1000)
#define  CPU_TS_Tmr_Delay_S(s)    CPU_TS_Tmr_Delay_MS((s)*1000)
#endif

#endif /* __CORE_DELAY_H */
