/**
 * @file    common.c
 * @brief   轻量级串口格式化输出实现（支持 %s / %d 与 \r \n 转义）
 * @author  Ashleycurry
 * @version 1.0.0
 */

#include "common.h"
#include "stm32f10x.h"
#include <stdarg.h>

static char *itoa(int value, char *string, int radix);

/**
 * @brief  轻量级串口格式化输出
 * @param  USARTx 目标串口
 * @param  Data   格式字符串（仅支持 %s / %d 与 \r \n 转义）
 * @note   相比标准库 printf 体积更小；每字节发送后等待 TXE 标志，保证时序。
 */
void USART_printf(USART_TypeDef *USARTx, char *Data, ...)
{
    const char *s;
    int d;
    char buf[16];

    va_list ap;
    va_start(ap, Data);

    while (*Data != 0)
    {
        if (*Data == 0x5c)  /* 转义字符 '\' */
        {
            switch (*++Data)
            {
                case 'r':
                    USART_SendData(USARTx, 0x0d);
                    Data++;
                    break;
                case 'n':
                    USART_SendData(USARTx, 0x0a);
                    Data++;
                    break;
                default:
                    Data++;
                    break;
            }
        }
        else if (*Data == '%')
        {
            switch (*++Data)
            {
                case 's':
                    s = va_arg(ap, const char *);
                    for (; *s; s++)
                    {
                        USART_SendData(USARTx, *s);
                        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
                    }
                    Data++;
                    break;
                case 'd':
                    d = va_arg(ap, int);
                    itoa(d, buf, 10);
                    for (s = buf; *s; s++)
                    {
                        USART_SendData(USARTx, *s);
                        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
                    }
                    Data++;
                    break;
                default:
                    Data++;
                    break;
            }
        }
        else
        {
            USART_SendData(USARTx, *Data++);
        }
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
    }
    va_end(ap);
}

/**
 * @brief  整数转十进制字符串（最大 5 位，仅支持 radix = 10）
 */
static char *itoa(int value, char *string, int radix)
{
    int i, d;
    int flag = 0;
    char *ptr = string;

    if (radix != 10) { *ptr = 0; return string; }
    if (!value)      { *ptr++ = 0x30; *ptr = 0; return string; }
    if (value < 0)   { *ptr++ = '-'; value *= -1; }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;
        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }
    *ptr = 0;
    return string;
}
