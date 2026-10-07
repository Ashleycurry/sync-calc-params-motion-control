/**
 * @file    common.h
 * @brief   公共宏与工具函数接口
 * @author  Ashleycurry
 * @version 1.0.0
 */

#ifndef __COMMON_H
#define __COMMON_H

#include "stm32f10x.h"

/* 全局 NVIC 优先级分组配置（各外设驱动统一使用） */
#define macNVIC_PriorityGroup_x   NVIC_PriorityGroup_2

/* 轻量级格式化串口输出（支持 %s / %d） */
void USART_printf(USART_TypeDef *USARTx, char *Data, ...);

#endif /* __COMMON_H */
