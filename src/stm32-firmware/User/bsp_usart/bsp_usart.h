/**
 * @file    bsp_usart.h
 * @brief   调试串口 USART1 驱动与运动指令解析接口
 * @author  Ashleycurry
 * @version 1.0.0
 */

#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f10x.h"
#include <stdio.h>

/* ===================== USART1 调试串口（PA9/PA10） ===================== */
#define DEBUG_USARTx                USART1
#define DEBUG_USART_CLK             RCC_APB2Periph_USART1
#define DEBUG_USART_APBxClkCmd      RCC_APB2PeriphClockCmd
#define DEBUG_USART_BAUDRATE        115200

#define DEBUG_USART_GPIO_CLK        (RCC_APB2Periph_GPIOA)
#define DEBUG_USART_GPIO_APBxClkCmd RCC_APB2PeriphClockCmd
#define DEBUG_USART_TX_GPIO_PORT    GPIOA
#define DEBUG_USART_TX_GPIO_PIN     GPIO_Pin_9
#define DEBUG_USART_RX_GPIO_PORT    GPIOA
#define DEBUG_USART_RX_GPIO_PIN     GPIO_Pin_10
#define DEBUG_USART_IRQ             USART1_IRQn
#define DEBUG_USART_IRQHandler      USART1_IRQHandler

/* ===================== 电机运动参数（由指令解析写入） ===================== */
extern signed int   g_step1;    /* 电机 1 步数（负值为反转） */
extern unsigned int g_accel1;   /* 电机 1 加速度（步/秒²） */
extern unsigned int g_decel1;   /* 电机 1 减速度（步/秒²） */
extern unsigned int g_speed1;   /* 电机 1 最高速度（步/秒） */

extern signed int   g_step2;    /* 电机 2 步数（负值为反转） */
extern unsigned int g_accel2;   /* 电机 2 加速度（步/秒²） */
extern unsigned int g_decel2;   /* 电机 2 减速度（步/秒²） */
extern unsigned int g_speed2;   /* 电机 2 最高速度（步/秒） */

/* ===================== 函数声明 ===================== */
void    USART_Config(void);                         /* 初始化 USART1（115200-8-N-1） */
uint8_t Parse_Cmd_From_Buf(const char *buf);        /* 解析 8 参数指令，成功返回 1 */
void    ShowHelp(void);                             /* 打印指令格式说明 */

#endif /* __BSP_USART_H */
