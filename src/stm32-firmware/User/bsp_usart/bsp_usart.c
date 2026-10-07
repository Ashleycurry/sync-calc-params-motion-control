/**
 * @file    bsp_usart.c
 * @brief   调试串口 USART1 驱动与运动指令解析实现
 * @author  Ashleycurry
 * @version 1.0.0
 */

#include "bsp_usart.h"

/* ===================== 电机运动参数（指令解析结果） ===================== */
signed int   g_step1  = 0;
unsigned int g_accel1 = 0;
unsigned int g_decel1 = 0;
unsigned int g_speed1 = 0;

signed int   g_step2  = 0;
unsigned int g_accel2 = 0;
unsigned int g_decel2 = 0;
unsigned int g_speed2 = 0;

/* ===================== 初始化 ===================== */
/**
 * @brief  USART1 中断优先级配置
 * @note   当前工程 USART1 仅用于 printf 调试输出，未开启接收中断，
 *         此处为后续扩展预留。
 */
static void USART1_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel                   = DEBUG_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  初始化 USART1 调试串口（115200-8-N-1，PA9/PA10）
 */
void USART_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

    /* TX：复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = DEBUG_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* RX：浮空输入 */
    GPIO_InitStructure.GPIO_Pin  = DEBUG_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = DEBUG_USART_BAUDRATE;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(DEBUG_USARTx, &USART_InitStructure);

    USART1_NVIC_Config();
    USART_Cmd(DEBUG_USARTx, ENABLE);
}

/* ===================== printf 重定向 ===================== */
/**
 * @brief  将标准库 printf 重定向到 USART1（Keil MicroLIB 约定）
 */
int fputc(int ch, FILE *f)
{
    USART_SendData(DEBUG_USARTx, (uint8_t)ch);
    while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);
    return ch;
}

/* ===================== 指令解析 ===================== */
/**
 * @brief  解析上位机指令："step1 accel1 decel1 speed1 step2 accel2 decel2 speed2"
 * @param  buf 以 '\0' 结尾的指令字符串
 * @retval 1：8 个参数解析成功（结果写入全局变量）；0：格式错误
 */
uint8_t Parse_Cmd_From_Buf(const char *buf)
{
    int ret = sscanf(buf, "%d %u %u %u %d %u %u %u",
                     &g_step1, &g_accel1, &g_decel1, &g_speed1,
                     &g_step2, &g_accel2, &g_decel2, &g_speed2);
    if (ret == 8)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief  打印指令格式帮助信息（开机时输出到调试串口）
 */
void ShowHelp(void)
{
    printf("\r\n ------ Stepper Motor Network Control ------\r\n");
    printf("Command format: step1 accel1 decel1 speed1 step2 accel2 decel2 speed2\r\n");
    printf("Example: 2000 200 200 75 2000 200 200 75\r\n");
}
