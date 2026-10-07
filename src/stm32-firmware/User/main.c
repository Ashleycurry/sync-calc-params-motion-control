/**
 * @file    main.c
 * @brief   双轴步进电机同步运动控制系统——应用主流程
 * @author  Ashleycurry
 * @version 1.0.0
 *
 * 运行流程：
 *  1. 初始化 DWT 延时、调试串口（USART1）、电机驱动（TIM3/TIM4）；
 *  2. 初始化 ESP8266（USART3 + AT 指令）并连接 TCP 服务器，进入透传模式；
 *  3. 主循环检测透传接收帧：解析 8 参数指令 -> Sync_CalcParams 双轴同步算法 -> 同时启动两轴；
 *  4. 检测到 TCP 连接断开时自动重连（重新入网 -> 重连服务器 -> 恢复透传）。
 *
 * 指令格式（上位机通过 TCP 下发，经 ESP8266 透传至 USART3）：
 *  step1 accel1 decel1 speed1 step2 accel2 decel2 speed2
 */

#pragma diag_suppress 870     /* 抑制 ARM 编译器对中文注释的多字节字符告警 870 */

#include "stm32f10x.h"
#include "microstep_driver.h"
#include "bsp_usart.h"
#include "sync_calc_params.h"
#include "bsp_esp8266.h"
#include "core_delay.h"
#include <string.h>

/* ===================== 网络配置（烧录前请按实际环境修改） ===================== */
#define WIFI_SSID    "YOUR_WIFI_SSID"       /* 路由器 SSID */
#define WIFI_PWD     "YOUR_WIFI_PASSWORD"   /* 路由器密码 */
#define SERVER_IP    "192.168.1.100"        /* 上位机 IP（运行 server.py 的电脑） */
#define SERVER_PORT  "5000"                 /* 上位机 TCP 端口 */

volatile uint8_t ucTcpClosedFlag = 0;       /* TCP 连接断开标志（USART3 中断置位） */

/**
 * @brief  网络初始化：AT 自检 -> DHCP -> STA 模式 -> 连接路由器 ->
 *         连接 TCP 服务器 -> 进入透传模式
 * @note   各步骤失败时重试直至成功（演示工程策略，实际产品建议增加超时退出）。
 */
static void Network_Init(void)
{
    printf("\r\nInitializing ESP8266...\r\n");
    macESP8266_CH_ENABLE();

    while (!ESP8266_AT_Test())
        printf("AT test failed, retrying...\r\n");

    while (!ESP8266_DHCP())
        printf("DHCP failed, retrying...\r\n");

    printf("Setting STA mode...\r\n");
    while (!ESP8266_Net_Mode_Choose(STA));

    printf("Connecting WiFi: %s\r\n", WIFI_SSID);
    if (ESP8266_JoinAP(WIFI_SSID, WIFI_PWD))
        printf("WiFi connected.\r\n");
    else
    {
        printf("WiFi failed. Check SSID/password.\r\n");
        while (1);
    }

    printf("Disable multi-connection...\r\n");
    while (!ESP8266_Enable_MultipleId(DISABLE));

    printf("Connecting server %s:%s ...\r\n", SERVER_IP, SERVER_PORT);
    while (!ESP8266_Link_Server(enumTCP, SERVER_IP, SERVER_PORT, Single_ID_0));

    printf("Entering passthrough mode...\r\n");
    while (!ESP8266_UnvarnishSend());

    printf("Ready. Waiting for commands...\r\n");
}

/**
 * @brief  处理一帧服务器指令：解析 -> Sync_CalcParams 双轴同步算法 -> 同时启动两轴
 * @note   解析成功后在启动前执行 Sync_CalcParams 双轴同步算法，
 *         保证两轴按相同的时间轴同时启停。
 */
static void Handle_Server_Frame(void)
{
    char *buf = strEsp8266_Fram_Record.Data_RX_BUF;
    buf[strEsp8266_Fram_Record.InfBit.FramLength] = '\0';

    printf("\r\n[CMD] %s\r\n", buf);

    if (Parse_Cmd_From_Buf(buf))
    {
        printf("OK: M1(%d,%u,%u,%u) M2(%d,%u,%u,%u)\r\n",
               g_step1, g_accel1, g_decel1, g_speed1,
               g_step2, g_accel2, g_decel2, g_speed2);
        Sync_CalcParams();
        MSD_Move1(g_step1, g_accel1, g_decel1, g_speed1);
        MSD_Move2(g_step2, g_accel2, g_decel2, g_speed2);
    }
    else
    {
        printf("Invalid command, ignored.\r\n");
    }

    strEsp8266_Fram_Record.InfBit.FramLength     = 0;
    strEsp8266_Fram_Record.InfBit.FramFinishFlag = 0;
}

/**
 * @brief  TCP 断开后的自动重连：退出透传 -> 查询链路状态 ->
 *         重新入网/重连服务器 -> 恢复透传
 */
static void Reconnect_Server(void)
{
    uint8_t ucStatus;

    printf("\r\nConnection lost, reconnecting...\r\n");
    ESP8266_ExitUnvarnishSend();

    do { ucStatus = ESP8266_Get_LinkStatus(); } while (!ucStatus);

    if (ucStatus == 4)
    {
        while (!ESP8266_JoinAP(WIFI_SSID, WIFI_PWD));
        Delay_ms(500);
        while (!ESP8266_Link_Server(enumTCP, SERVER_IP, SERVER_PORT, Single_ID_0));
        Delay_ms(1000);
    }

    while (!ESP8266_UnvarnishSend());
    Delay_ms(200);

    ucTcpClosedFlag = 0;
    printf("Reconnected. Waiting for commands...\r\n");
}

/**
 * @brief  应用入口：初始化外设与网络后进入主循环
 */
int main(void)
{
    CPU_TS_TmrInit();       /* DWT 延时基准 */
    USART_Config();         /* USART1 调试串口 */
    MSD_Init();             /* 双路步进电机驱动 */
    ShowHelp();             /* 打印指令格式说明 */

    ESP8266_Init();         /* USART3 + ESP8266 控制引脚 */
    Network_Init();         /* 联网并连接上位机 */

    while (1)
    {
        /* 收到一帧完整指令：处理运动 */
        if (strEsp8266_Fram_Record.InfBit.FramFinishFlag)
            Handle_Server_Frame();

        /* 检测到连接断开：自动重连 */
        if (ucTcpClosedFlag)
            Reconnect_Server();
    }
}
