/**
 * @file    stm32f10x_it.c
 * @brief   中断服务函数（ESP8266 数据帧接收与 TCP 断线检测）
 * @author  Ashleycurry
 * @version 1.0.0
 */

#include "stm32f10x_it.h"
#include "bsp_esp8266.h"
#include <string.h>

/* 由 main.c 定义，IDLE 中断检测到 "CLOSED" 时置 1 */
extern volatile uint8_t ucTcpClosedFlag;

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

void NMI_Handler(void)        {}
void HardFault_Handler(void)  { while (1); }
void MemManage_Handler(void)  { while (1); }
void BusFault_Handler(void)   { while (1); }
void UsageFault_Handler(void) { while (1); }
void SVC_Handler(void)        {}
void DebugMon_Handler(void)   {}
void PendSV_Handler(void)     {}
void SysTick_Handler(void)    {}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/******************************************************************************/

/**
 * @brief USART3 中断处理（ESP8266 数据接收）
 *        RXNE：逐字节写入帧缓冲区
 *        IDLE：总线空闲，标记一帧接收完成
 */
void USART3_IRQHandler(void)
{
    uint8_t ucCh;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        ucCh = USART_ReceiveData(USART3);

        if (strEsp8266_Fram_Record.InfBit.FramLength < (RX_BUF_MAX_LEN - 1))
            strEsp8266_Fram_Record.Data_RX_BUF[strEsp8266_Fram_Record.InfBit.FramLength++] = ucCh;
    }

    if (USART_GetITStatus(USART3, USART_IT_IDLE) == SET)
    {
        strEsp8266_Fram_Record.InfBit.FramFinishFlag = 1;
        /* 读 SR 再读 DR 以清除 IDLE 标志 */
        ucCh = USART_ReceiveData(USART3);
        (void)ucCh;

        /* 检测连接是否断开 */
        strEsp8266_Fram_Record.Data_RX_BUF[strEsp8266_Fram_Record.InfBit.FramLength] = '\0';
        ucTcpClosedFlag = strstr(strEsp8266_Fram_Record.Data_RX_BUF, "CLOSED\r\n") ? 1 : 0;
    }
}
