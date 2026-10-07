/**
 * @file    bsp_esp8266.h
 * @brief   ESP8266 WiFi 模块驱动接口（AT 指令集封装 + USART3 中断收帧）
 * @author  Ashleycurry
 * @version 1.0.0
 */

#ifndef  __BSP_ESP8266_H
#define  __BSP_ESP8266_H

#include "stm32f10x.h"
#include <stdio.h>
#include <stdbool.h>

#if defined ( __CC_ARM )
#pragma anon_unions
#endif

/******************************* ESP8266 枚举类型定义 ***************************/
typedef enum { STA, AP, STA_AP } ENUM_Net_ModeTypeDef;
typedef enum { enumTCP, enumUDP } ENUM_NetPro_TypeDef;
typedef enum {
    Multiple_ID_0 = 0,
    Multiple_ID_1 = 1,
    Multiple_ID_2 = 2,
    Multiple_ID_3 = 3,
    Multiple_ID_4 = 4,
    Single_ID_0   = 5,
} ENUM_ID_NO_TypeDef;
typedef enum { OPEN = 0, WEP = 1, WPA_PSK = 2, WPA2_PSK = 3, WPA_WPA2_PSK = 4 } ENUM_AP_PsdMode_TypeDef;

/******************************* 接收帧缓冲区 ***************************/
#define RX_BUF_MAX_LEN  1024

extern struct STRUCT_USARTx_Fram
{
    char Data_RX_BUF[RX_BUF_MAX_LEN];
    union {
        __IO u16 InfAll;
        struct {
            __IO u16 FramLength     : 15;   // 帧长度
            __IO u16 FramFinishFlag : 1;    // 帧完成标志
        } InfBit;
    };
} strEsp8266_Fram_Record;

/******************************* ESP8266 硬件引脚宏定义 ***************************/
// CH_PD 引脚 -> PB8
#define macESP8266_CH_PD_APBxClock_FUN      RCC_APB2PeriphClockCmd
#define macESP8266_CH_PD_CLK                RCC_APB2Periph_GPIOB
#define macESP8266_CH_PD_PORT               GPIOB
#define macESP8266_CH_PD_PIN                GPIO_Pin_8

// RST 引脚 -> PB9
#define macESP8266_RST_APBxClock_FUN        RCC_APB2PeriphClockCmd
#define macESP8266_RST_CLK                  RCC_APB2Periph_GPIOB
#define macESP8266_RST_PORT                 GPIOB
#define macESP8266_RST_PIN                  GPIO_Pin_9

// USART3 -> PB10(TX), PB11(RX)
#define macESP8266_USART_BAUD_RATE          115200
#define macESP8266_USARTx                   USART3
#define macESP8266_USART_APBxClock_FUN      RCC_APB1PeriphClockCmd
#define macESP8266_USART_CLK                RCC_APB1Periph_USART3
#define macESP8266_USART_GPIO_APBxClock_FUN RCC_APB2PeriphClockCmd
#define macESP8266_USART_GPIO_CLK           RCC_APB2Periph_GPIOB
#define macESP8266_USART_TX_PORT            GPIOB
#define macESP8266_USART_TX_PIN             GPIO_Pin_10
#define macESP8266_USART_RX_PORT            GPIOB
#define macESP8266_USART_RX_PIN             GPIO_Pin_11
#define macESP8266_USART_IRQ                USART3_IRQn
#define macESP8266_USART_INT_FUN            USART3_IRQHandler

/******************************* 操作宏 ***************************/
// 向 ESP8266 串口发送格式化字符串（使用 USART_printf）
#define macESP8266_Usart(fmt, ...)   USART_printf(macESP8266_USARTx, fmt, ##__VA_ARGS__)
// 向 PC 调试串口（USART1）打印
#define macPC_Usart(fmt, ...)        printf(fmt, ##__VA_ARGS__)

#define macESP8266_CH_ENABLE()       GPIO_SetBits(macESP8266_CH_PD_PORT, macESP8266_CH_PD_PIN)
#define macESP8266_CH_DISABLE()      GPIO_ResetBits(macESP8266_CH_PD_PORT, macESP8266_CH_PD_PIN)
#define macESP8266_RST_HIGH_LEVEL()  GPIO_SetBits(macESP8266_RST_PORT, macESP8266_RST_PIN)
#define macESP8266_RST_LOW_LEVEL()   GPIO_ResetBits(macESP8266_RST_PORT, macESP8266_RST_PIN)

/******************************* 函数声明 ***************************/
void    ESP8266_Init              (void);
void    ESP8266_Rst               (void);
bool    ESP8266_Cmd               (char *cmd, char *reply1, char *reply2, u32 waittime);
bool    ESP8266_AT_Test           (void);
bool    ESP8266_Net_Mode_Choose   (ENUM_Net_ModeTypeDef enumMode);
bool    ESP8266_JoinAP            (char *pSSID, char *pPassWord);
bool    ESP8266_BuildAP           (char *pSSID, char *pPassWord, ENUM_AP_PsdMode_TypeDef enunPsdMode);
bool    ESP8266_Enable_MultipleId (FunctionalState enumEnUnvarnishTx);
bool    ESP8266_Link_Server       (ENUM_NetPro_TypeDef enumE, char *ip, char *ComNum, ENUM_ID_NO_TypeDef id);
bool    ESP8266_StartOrShutServer (FunctionalState enumMode, char *pPortNum, char *pTimeOver);
uint8_t ESP8266_Get_LinkStatus    (void);
bool    ESP8266_UnvarnishSend     (void);
void    ESP8266_ExitUnvarnishSend (void);
bool    ESP8266_SendString        (FunctionalState enumEnUnvarnishTx, char *pStr, u32 ulStrLength, ENUM_ID_NO_TypeDef ucId);
char   *ESP8266_ReceiveString     (FunctionalState enumEnUnvarnishTx);
bool    ESP8266_DHCP              (void);

#endif /* __BSP_ESP8266_H */
