/**
 * @file    microstep_driver.h
 * @brief   步进电机微步驱动器接口定义（TIM PWM 时基 + 中断梯形加减速）
 * @author  Ashleycurry
 * @version 1.0.0
 *
 * 两路独立电机通道，分别使用 TIM3_CH1(PA6) / TIM4_CH1(PB6) 输出脉冲，
 * 每个脉冲触发一次定时器更新中断，由中断推进加减速状态机。
 */

#ifndef __MICROSTEP_DRIVER_H
#define __MICROSTEP_DRIVER_H

#include "stm32f10x.h"

/* ===================== 通用定义 ===================== */
#define CW    0             /* 顺时针（正转） */
#define CCW   1             /* 逆时针（反转） */
#define TRUE  1
#define FALSE 0

#define PULSE_WIDTH_US  20  /* 脉冲宽度（微秒），初始时基 50 kHz */

/* ===================== 系统运行状态 ===================== */
struct GLOBAL_FLAGS {
    unsigned char motor1_running : 1;   /* 电机 1 是否正在运动 */
    unsigned char motor2_running : 1;   /* 电机 2 是否正在运动 */
    unsigned char output_enabled : 1;   /* 驱动器输出使能（关闭后当前脉冲结束即停机） */
};
extern struct GLOBAL_FLAGS status;

/* ===================== 运动学计算常量（整数化） =====================
 * 常量推导参考 Atmel AVR446 应用笔记，全部使用整数运算，避免浮点开销：
 *   ALPHA      单个微步对应的角位移（弧度），SPR = 200 * 100 = 20000 微步/圈
 *   A_T_x100   alpha * T1_FREQ * 100
 *   T1_FREQ_148 (T1_FREQ * 0.676) / 100，用于首步延时的整数平方根近似
 *   A_SQ       alpha * 2 * 10^10
 *   A_x20000   alpha * 20000
 */
#define T1_FREQ      1000000                /* 定时器计数频率：1 MHz（1 计数 = 1 µs） */
#define FSPR         200                    /* 电机整步数/圈（1.8° 步距角） */
#define SPR          (FSPR * 100)           /* 100 细分后的微步数/圈 */
#define ALPHA        (2 * 3.14159 / SPR)
#define A_T_x100     ((long)(ALPHA * T1_FREQ * 100))
#define T1_FREQ_148  ((int)((T1_FREQ * 0.676) / 100))
#define A_SQ         (long)(ALPHA * 2 * 10000000000)
#define A_x20000     (int)(ALPHA * 20000)

/* ===================== 速度斜坡状态 ===================== */
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

/* ===================== 速度斜坡参数结构 ===================== */
typedef struct {
    unsigned char run_state : 3;     /* 运行状态：STOP / ACCEL / RUN / DECEL */
    unsigned char dir : 1;           /* 运动方向：CW / CCW */
    unsigned int  step_delay;        /* 当前步间延时（定时器计数） */
    unsigned int  decel_start;       /* 进入减速段的脉冲序号 */
    signed int    decel_val;         /* 减速段计数值（进入减速时装载） */
    signed int    min_delay;         /* 最高速度对应的最小延时 */
    signed int    accel_count;       /* 加速段计数 */
    int           step_position;     /* 累计位置（脉冲数，带符号） */
    unsigned int  step_count;        /* 已走步数（每次运动重置） */
    signed int    rest;              /* 加减速余数（每次运动重置） */
    int           last_accel_delay;  /* 加速阶段最后一个延时 */
} SpeedRampData;

/* ===================== 电机 1 硬件资源：TIM3_CH1 + PA6/PA4/PA5 ===================== */
#define MSD1_PULSE_TIM                    TIM3
#define MSD1_PULSE_TIM_APBxClock_FUN      RCC_APB1PeriphClockCmd
#define MSD1_PULSE_TIM_CLK                RCC_APB1Periph_TIM3
#define MSD1_PULSE_OCx_Init               TIM_OC1Init
#define MSD1_PULSE_TIM_IRQ                TIM3_IRQn
#define MSD1_PULSE_TIM_IRQHandler         TIM3_IRQHandler

#define MSD1_PULSE_GPIO_CLK               RCC_APB2Periph_GPIOA
#define MSD1_PULSE_PORT                   GPIOA
#define MSD1_PULSE_PIN                    GPIO_Pin_6
#define MSD1_DIR_GPIO_CLK                 RCC_APB2Periph_GPIOA
#define MSD1_DIR_PORT                     GPIOA
#define MSD1_DIR_PIN                      GPIO_Pin_4
#define MSD1_ENA_GPIO_CLK                 RCC_APB2Periph_GPIOA
#define MSD1_ENA_PORT                     GPIOA
#define MSD1_ENA_PIN                      GPIO_Pin_5

/* ===================== 电机 2 硬件资源：TIM4_CH1 + PB6/PB7/PB5 ===================== */
#define MSD2_PULSE_TIM                    TIM4
#define MSD2_PULSE_TIM_APBxClock_FUN      RCC_APB1PeriphClockCmd
#define MSD2_PULSE_TIM_CLK                RCC_APB1Periph_TIM4
#define MSD2_PULSE_OCx_Init               TIM_OC1Init
#define MSD2_PULSE_TIM_IRQ                TIM4_IRQn
#define MSD2_PULSE_TIM_IRQHandler         TIM4_IRQHandler

#define MSD2_PULSE_GPIO_CLK               RCC_APB2Periph_GPIOB
#define MSD2_PULSE_PORT                   GPIOB
#define MSD2_PULSE_PIN                    GPIO_Pin_6
#define MSD2_DIR_GPIO_CLK                 RCC_APB2Periph_GPIOB
#define MSD2_DIR_PORT                     GPIOB
#define MSD2_DIR_PIN                      GPIO_Pin_7
#define MSD2_ENA_GPIO_CLK                 RCC_APB2Periph_GPIOB
#define MSD2_ENA_PORT                     GPIOB
#define MSD2_ENA_PIN                      GPIO_Pin_5

/* ===================== 方向控制宏 ===================== */
#define DIR1(dir)   do {                                                     \
                        if ((dir) == CW)                                     \
                            GPIO_ResetBits(MSD1_DIR_PORT, MSD1_DIR_PIN);     \
                        else                                                 \
                            GPIO_SetBits(MSD1_DIR_PORT, MSD1_DIR_PIN);       \
                    } while (0)

#define DIR2(dir)   do {                                                     \
                        if ((dir) == CW)                                     \
                            GPIO_ResetBits(MSD2_DIR_PORT, MSD2_DIR_PIN);     \
                        else                                                 \
                            GPIO_SetBits(MSD2_DIR_PORT, MSD2_DIR_PIN);       \
                    } while (0)

/* ===================== 对外接口 ===================== */
void MSD_Init(void);                                        /* 初始化两路电机的 GPIO、定时器与中断 */
void MSD_ENA1(FunctionalState NewState);                    /* 使能/关闭电机 1 驱动器输出 */
void MSD_ENA2(FunctionalState NewState);                    /* 使能/关闭电机 2 驱动器输出 */
void MSD_Move1(signed int step, unsigned int accel, unsigned int decel, unsigned int speed);  /* 启动电机 1 运动（非阻塞） */
void MSD_Move2(signed int step, unsigned int accel, unsigned int decel, unsigned int speed);  /* 启动电机 2 运动（非阻塞） */
int  MSD_GetPosition1(void);                                /* 读取电机 1 累计位置（脉冲数） */
int  MSD_GetPosition2(void);                                /* 读取电机 2 累计位置（脉冲数） */
unsigned char MSD_IsMotor1Running(void);                    /* 电机 1 是否运动中 */
unsigned char MSD_IsMotor2Running(void);                    /* 电机 2 是否运动中 */

#endif /* __MICROSTEP_DRIVER_H */
