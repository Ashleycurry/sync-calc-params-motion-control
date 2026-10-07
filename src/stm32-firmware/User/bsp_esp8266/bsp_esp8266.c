/**
 * @file    bsp_esp8266.c
 * @brief   ESP8266 WiFi 模块驱动实现（AT 指令 + USART3 中断收帧）
 * @author  Ashleycurry
 * @version 1.0.0
 *
 * 驱动说明：
 *  1. CH_PD/RST 由 GPIO 控制（PB8/PB9），数据经 USART3（PB10/PB11）收发；
 *  2. 发送采用 AT 指令字符串，接收由 USART3 中断（RXNE + IDLE）逐帧写入
 *     strEsp8266_Fram_Record 缓冲区；
 *  3. 支持 STA 模式、TCP 客户端、透传（CIPMODE=1 + CIPSEND）等常用操作。
 */

#include "bsp_esp8266.h"
#include "common.h"
#include "core_delay.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

static void ESP8266_GPIO_Config(void);
static void ESP8266_USART_Config(void);
static void ESP8266_USART_NVIC_Config(void);

struct STRUCT_USARTx_Fram strEsp8266_Fram_Record = { 0 };

/* ===================== 初始化 ===================== */
/**
 * @brief  初始化 ESP8266 控制引脚与串口（复位引脚置高、CH_PD 默认关闭）
 */
void ESP8266_Init(void)
{
    ESP8266_GPIO_Config();
    ESP8266_USART_Config();
    macESP8266_RST_HIGH_LEVEL();
    macESP8266_CH_DISABLE();
}

/**
 * @brief  配置 CH_PD（PB8）与 RST（PB9）控制引脚
 */
static void ESP8266_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    macESP8266_CH_PD_APBxClock_FUN(macESP8266_CH_PD_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = macESP8266_CH_PD_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(macESP8266_CH_PD_PORT, &GPIO_InitStructure);

    macESP8266_RST_APBxClock_FUN(macESP8266_RST_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = macESP8266_RST_PIN;
    GPIO_Init(macESP8266_RST_PORT, &GPIO_InitStructure);
}

/**
 * @brief  配置 USART3（115200-8-N-1），开启 RXNE 与 IDLE 中断
 * @note   RXNE 逐字节收数据，IDLE 在总线空闲时标记一帧结束。
 */
static void ESP8266_USART_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    macESP8266_USART_APBxClock_FUN(macESP8266_USART_CLK, ENABLE);
    macESP8266_USART_GPIO_APBxClock_FUN(macESP8266_USART_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = macESP8266_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(macESP8266_USART_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = macESP8266_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(macESP8266_USART_RX_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = macESP8266_USART_BAUD_RATE;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(macESP8266_USARTx, &USART_InitStructure);

    USART_ITConfig(macESP8266_USARTx, USART_IT_RXNE, ENABLE);
    USART_ITConfig(macESP8266_USARTx, USART_IT_IDLE, ENABLE);

    ESP8266_USART_NVIC_Config();

    USART_Cmd(macESP8266_USARTx, ENABLE);
}

/**
 * @brief  配置 USART3 中断优先级（最高抢占优先级，保证收帧实时性）
 */
static void ESP8266_USART_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(macNVIC_PriorityGroup_x);

    NVIC_InitStructure.NVIC_IRQChannel                   = macESP8266_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ===================== 基础操作 ===================== */
/**
 * @brief  硬件复位 ESP8266（RST 拉低 500 ms 后置高）
 */
void ESP8266_Rst(void)
{
    macESP8266_RST_LOW_LEVEL();
    Delay_ms(500);
    macESP8266_RST_HIGH_LEVEL();
}

/**
 * @brief  开启 DHCP（STA 模式下的 IP 分配）
 * @retval true：设置成功
 */
bool ESP8266_DHCP(void)
{
    return ESP8266_Cmd("AT+CWDHCP=1,1", "OK", NULL, 500);
}

/**
 * @brief  发送 AT 指令并等待应答
 * @param  cmd      待发送的 AT 指令
 * @param  reply1   期望的应答关键字 1（NULL 表示不检查）
 * @param  reply2   期望的应答关键字 2（NULL 表示不检查）
 * @param  waittime 等待应答时长（ms）
 * @retval true：收到任一期望应答；false：超时或应答不符
 */
bool ESP8266_Cmd(char *cmd, char *reply1, char *reply2, u32 waittime)
{
    strEsp8266_Fram_Record.InfBit.FramLength = 0;

    macESP8266_Usart("%s\r\n", cmd);

    if ((reply1 == 0) && (reply2 == 0))
        return true;

    Delay_ms(waittime);

    strEsp8266_Fram_Record.Data_RX_BUF[strEsp8266_Fram_Record.InfBit.FramLength] = '\0';

    macPC_Usart("%s", strEsp8266_Fram_Record.Data_RX_BUF);
    strEsp8266_Fram_Record.InfBit.FramLength      = 0;
    strEsp8266_Fram_Record.InfBit.FramFinishFlag  = 0;

    if ((reply1 != 0) && (reply2 != 0))
        return ((bool)strstr(strEsp8266_Fram_Record.Data_RX_BUF, reply1) ||
                (bool)strstr(strEsp8266_Fram_Record.Data_RX_BUF, reply2));
    else if (reply1 != 0)
        return ((bool)strstr(strEsp8266_Fram_Record.Data_RX_BUF, reply1));
    else
        return ((bool)strstr(strEsp8266_Fram_Record.Data_RX_BUF, reply2));
}

/**
 * @brief  AT 自检（最多重试 10 次，期间执行硬件复位）
 * @retval true：模块应答正常
 */
bool ESP8266_AT_Test(void)
{
    char count = 0;
    macESP8266_RST_HIGH_LEVEL();
    printf("AT test...\r\n");
    Delay_ms(3000);
    while (count < 10)
    {
        printf("AT attempt %d\r\n", count);
        if (ESP8266_Cmd("AT", "OK", NULL, 500))
        {
            printf("AT ok.\r\n");
            return 1;
        }
        ESP8266_Rst();
        ++count;
    }
    return 0;
}

/* ===================== 网络与连接配置 ===================== */
/**
 * @brief  设置工作模式（STA / AP / STA+AP）
 * @param  enumMode 目标模式
 * @retval true：设置成功
 */
bool ESP8266_Net_Mode_Choose(ENUM_Net_ModeTypeDef enumMode)
{
    switch (enumMode)
    {
        case STA:    return ESP8266_Cmd("AT+CWMODE=1", "OK", "no change", 2500);
        case AP:     return ESP8266_Cmd("AT+CWMODE=2", "OK", "no change", 2500);
        case STA_AP: return ESP8266_Cmd("AT+CWMODE=3", "OK", "no change", 2500);
        default:     return false;
    }
}

/**
 * @brief  连接路由器（STA 模式）
 * @param  pSSID     路由器名称
 * @param  pPassWord 路由器密码
 * @retval true：连接成功
 */
bool ESP8266_JoinAP(char *pSSID, char *pPassWord)
{
    char cCmd[120];
    sprintf(cCmd, "AT+CWJAP=\"%s\",\"%s\"", pSSID, pPassWord);
    return ESP8266_Cmd(cCmd, "OK", NULL, 10000);
}

/**
 * @brief  建立热点（AP 模式）
 * @param  pSSID        热点名称
 * @param  pPassWord    热点密码
 * @param  enunPsdMode  加密方式
 * @retval true：建立成功
 */
bool ESP8266_BuildAP(char *pSSID, char *pPassWord, ENUM_AP_PsdMode_TypeDef enunPsdMode)
{
    char cCmd[120];
    sprintf(cCmd, "AT+CWSAP=\"%s\",\"%s\",1,%d", pSSID, pPassWord, enunPsdMode);
    return ESP8266_Cmd(cCmd, "OK", 0, 1000);
}

/**
 * @brief  使能/关闭多连接（透传模式要求单连接）
 * @param  enumEnUnvarnishTx ENABLE：开启多连接；DISABLE：单连接
 * @retval true：设置成功
 */
bool ESP8266_Enable_MultipleId(FunctionalState enumEnUnvarnishTx)
{
    char cStr[20];
    sprintf(cStr, "AT+CIPMUX=%d", (enumEnUnvarnishTx ? 1 : 0));
    return ESP8266_Cmd(cStr, "OK", 0, 500);
}

/**
 * @brief  连接远程 TCP/UDP 服务器
 * @param  enumE  协议类型：enumTCP / enumUDP
 * @param  ip     服务器 IP
 * @param  ComNum 服务器端口
 * @param  id     连接编号（Single_ID_0 为单连接）
 * @retval true：连接成功（含"已连接"应答）
 */
bool ESP8266_Link_Server(ENUM_NetPro_TypeDef enumE, char *ip, char *ComNum, ENUM_ID_NO_TypeDef id)
{
    char cStr[100] = { 0 }, cCmd[120];

    switch (enumE)
    {
        case enumTCP: sprintf(cStr, "\"%s\",\"%s\",%s", "TCP", ip, ComNum); break;
        case enumUDP: sprintf(cStr, "\"%s\",\"%s\",%s", "UDP", ip, ComNum); break;
        default: break;
    }

    if (id < 5)
        sprintf(cCmd, "AT+CIPSTART=%d,%s", id, cStr);
    else
        sprintf(cCmd, "AT+CIPSTART=%s", cStr);

    return ESP8266_Cmd(cCmd, "OK", "ALREAY CONNECT", 4000);
}

/**
 * @brief  开启/关闭模块内置服务器
 * @param  enumMode  非 0：开启；0：关闭
 * @param  pPortNum  监听端口
 * @param  pTimeOver 超时时间（秒）
 * @retval true：设置成功
 */
bool ESP8266_StartOrShutServer(FunctionalState enumMode, char *pPortNum, char *pTimeOver)
{
    char cCmd1[120], cCmd2[120];
    if (enumMode)
    {
        sprintf(cCmd1, "AT+CIPSERVER=%d,%s", 1, pPortNum);
        sprintf(cCmd2, "AT+CIPSTO=%s", pTimeOver);
        return (ESP8266_Cmd(cCmd1, "OK", 0, 500) && ESP8266_Cmd(cCmd2, "OK", 0, 500));
    }
    else
    {
        sprintf(cCmd1, "AT+CIPSERVER=%d,%s", 0, pPortNum);
        return ESP8266_Cmd(cCmd1, "OK", 0, 500);
    }
}

/**
 * @brief  查询 TCP 连接状态
 * @retval 0：无连接；2：已获 IP 未连接；3：已连接；4：连接断开
 */
uint8_t ESP8266_Get_LinkStatus(void)
{
    if (ESP8266_Cmd("AT+CIPSTATUS", "OK", 0, 500))
    {
        if (strstr(strEsp8266_Fram_Record.Data_RX_BUF, "STATUS:2\r\n")) return 2;
        if (strstr(strEsp8266_Fram_Record.Data_RX_BUF, "STATUS:3\r\n")) return 3;
        if (strstr(strEsp8266_Fram_Record.Data_RX_BUF, "STATUS:4\r\n")) return 4;
    }
    return 0;
}

/* ===================== 透传模式 ===================== */
/**
 * @brief  进入透传模式（CIPMODE=1 后发送 CIPSEND）
 * @retval true：进入成功
 */
bool ESP8266_UnvarnishSend(void)
{
    if (!ESP8266_Cmd("AT+CIPMODE=1", "OK", 0, 500))
        return false;
    return ESP8266_Cmd("AT+CIPSEND", "OK", ">", 500);
}

/**
 * @brief  退出透传模式（发送 "+++" 并等待 500 ms）
 * @note   根据 AT 协议，"+++" 前后需静默延时。
 */
void ESP8266_ExitUnvarnishSend(void)
{
    Delay_ms(1000);
    macESP8266_Usart("+++");
    Delay_ms(500);
}

/* ===================== 数据收发 ===================== */
/**
 * @brief  发送字符串
 * @param  enumEnUnvarnishTx ENABLE：透传模式（直接发送）；DISABLE：普通模式（CIPSEND 分包）
 * @param  pStr         待发送字符串
 * @param  ulStrLength  字符串长度
 * @param  ucId         连接编号
 * @retval true：发送成功
 */
bool ESP8266_SendString(FunctionalState enumEnUnvarnishTx, char *pStr, u32 ulStrLength, ENUM_ID_NO_TypeDef ucId)
{
    char cStr[20];
    bool bRet = false;

    if (enumEnUnvarnishTx)
    {
        macESP8266_Usart("%s", pStr);
        bRet = true;
    }
    else
    {
        if (ucId < 5)
            sprintf(cStr, "AT+CIPSEND=%d,%d", ucId, (int)(ulStrLength + 2));
        else
            sprintf(cStr, "AT+CIPSEND=%d", (int)(ulStrLength + 2));

        ESP8266_Cmd(cStr, "> ", 0, 100);
        bRet = ESP8266_Cmd(pStr, "SEND OK", 0, 500);
    }
    return bRet;
}

/**
 * @brief  阻塞接收一帧数据（直到 USART3 中断置位帧完成标志）
 * @param  enumEnUnvarnishTx ENABLE：透传模式；DISABLE：普通模式（过滤 +IPD 帧）
 * @retval 接收缓冲区指针；无有效数据时返回 0
 */
char *ESP8266_ReceiveString(FunctionalState enumEnUnvarnishTx)
{
    char *pRecStr = 0;

    strEsp8266_Fram_Record.InfBit.FramLength      = 0;
    strEsp8266_Fram_Record.InfBit.FramFinishFlag  = 0;

    while (!strEsp8266_Fram_Record.InfBit.FramFinishFlag);
    strEsp8266_Fram_Record.Data_RX_BUF[strEsp8266_Fram_Record.InfBit.FramLength] = '\0';

    if (enumEnUnvarnishTx)
        pRecStr = strEsp8266_Fram_Record.Data_RX_BUF;
    else if (strstr(strEsp8266_Fram_Record.Data_RX_BUF, "+IPD"))
        pRecStr = strEsp8266_Fram_Record.Data_RX_BUF;

    return pRecStr;
}
