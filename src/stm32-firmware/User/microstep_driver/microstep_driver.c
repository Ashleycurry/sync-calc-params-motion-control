/**
 * @file    microstep_driver.c
 * @brief   步进电机微步驱动器实现（TIM 定时器中断驱动的梯形加减速）
 * @author  Ashleycurry
 * @version 1.0.0
 *
 * 工作原理：
 *  1. 每个定时器更新中断输出一个脉冲（一步），步间延时写入 ARR，
 *     脉冲宽度由 CCR1 保持 50% 占空比，从而实现变频脉冲输出；
 *  2. 步间延时按 加速 -> 匀速 -> 减速 三个阶段动态计算，构成梯形速度曲线；
 *  3. 延时递推与常量推导参考 Atmel AVR446 应用笔记，整数递推避免浮点运算；
 *  4. 两路电机使用独立定时器与中断，可同时启动、各自按曲线运行，互不阻塞。
 */

#include "microstep_driver.h"
#include <math.h>

/* ===================== 模块全局变量 ===================== */
struct GLOBAL_FLAGS status = {FALSE, FALSE, TRUE};

static SpeedRampData s_ramp1;   /* 电机 1 速度斜坡参数 */
static SpeedRampData s_ramp2;   /* 电机 2 速度斜坡参数 */

/* ===================== 中断优先级配置 ===================== */
/**
 * @brief  配置 TIM3 / TIM4 更新中断（NVIC 分组 0）
 * @note   两路中断抢占优先级相同、子优先级错开（电机 1 优先），
 *         保证同时启动时电机 1 的脉冲优先得到响应。
 */
static void TIM_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 中断分组 0：0 位抢占优先级 + 4 位子优先级 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_InitStructure.NVIC_IRQChannel                   = MSD1_PULSE_TIM_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = MSD2_PULSE_TIM_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStructure);
}

/* ===================== 电机 1 GPIO 初始化 ===================== */
/**
 * @brief  配置电机 1 的脉冲（PA6）、方向（PA4）、使能（PA5）引脚
 */
static void MSD1_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(MSD1_PULSE_GPIO_CLK | MSD1_DIR_GPIO_CLK | MSD1_ENA_GPIO_CLK, ENABLE);

    /* 脉冲信号：复用推挽输出（TIM3_CH1） */
    GPIO_InitStructure.GPIO_Pin   = MSD1_PULSE_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MSD1_PULSE_PORT, &GPIO_InitStructure);

    /* 方向信号：推挽输出，默认低电平 */
    GPIO_InitStructure.GPIO_Pin  = MSD1_DIR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MSD1_DIR_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MSD1_DIR_PORT, MSD1_DIR_PIN);

    /* 使能信号：推挽输出，默认低电平（驱动器输出使能） */
    GPIO_InitStructure.GPIO_Pin = MSD1_ENA_PIN;
    GPIO_Init(MSD1_ENA_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MSD1_ENA_PORT, MSD1_ENA_PIN);
}

/* ===================== 电机 2 GPIO 初始化 ===================== */
/**
 * @brief  配置电机 2 的脉冲（PB6）、方向（PB7）、使能（PB5）引脚
 */
static void MSD2_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(MSD2_PULSE_GPIO_CLK | MSD2_DIR_GPIO_CLK | MSD2_ENA_GPIO_CLK, ENABLE);

    /* 脉冲信号：复用推挽输出（TIM4_CH1） */
    GPIO_InitStructure.GPIO_Pin   = MSD2_PULSE_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MSD2_PULSE_PORT, &GPIO_InitStructure);

    /* 方向信号：推挽输出，默认低电平 */
    GPIO_InitStructure.GPIO_Pin  = MSD2_DIR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MSD2_DIR_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MSD2_DIR_PORT, MSD2_DIR_PIN);

    /* 使能信号：推挽输出，默认低电平（驱动器输出使能） */
    GPIO_InitStructure.GPIO_Pin = MSD2_ENA_PIN;
    GPIO_Init(MSD2_ENA_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MSD2_ENA_PORT, MSD2_ENA_PIN);
}

/* ===================== 电机 1 定时器初始化 ===================== */
/**
 * @brief  配置 TIM3：72 分频（1 MHz 计数）、PWM 模式 2、更新中断使能
 * @note   初始 ARR = 脉冲宽度、CCR = ARR / 2（50% 占空比），
 *         配置完成后定时器保持关闭，由 MSD_Move1 启动。
 */
static void MSD1_TIM_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    MSD1_PULSE_TIM_APBxClock_FUN(MSD1_PULSE_TIM_CLK, ENABLE);

    /* 时基：72 MHz / 72 = 1 MHz，1 个计数即 1 µs */
    TIM_TimeBaseStructure.TIM_Period        = PULSE_WIDTH_US;
    TIM_TimeBaseStructure.TIM_Prescaler     = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(MSD1_PULSE_TIM, &TIM_TimeBaseStructure);

    /* PWM 模式 2 + 低极性：输出正脉冲 */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = PULSE_WIDTH_US / 2;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;
    MSD1_PULSE_OCx_Init(MSD1_PULSE_TIM, &TIM_OCInitStructure);

    TIM_ARRPreloadConfig(MSD1_PULSE_TIM, ENABLE);
    TIM_UpdateRequestConfig(MSD1_PULSE_TIM, TIM_UpdateSource_Regular);
    TIM_ClearITPendingBit(MSD1_PULSE_TIM, TIM_IT_Update);
    TIM_ITConfig(MSD1_PULSE_TIM, TIM_IT_Update, ENABLE);
    TIM_Cmd(MSD1_PULSE_TIM, DISABLE);
}

/* ===================== 电机 2 定时器初始化 ===================== */
/**
 * @brief  配置 TIM4：与 TIM3 相同的时基与 PWM 配置
 */
static void MSD2_TIM_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    MSD2_PULSE_TIM_APBxClock_FUN(MSD2_PULSE_TIM_CLK, ENABLE);

    /* 时基：72 MHz / 72 = 1 MHz，1 个计数即 1 µs */
    TIM_TimeBaseStructure.TIM_Period        = PULSE_WIDTH_US;
    TIM_TimeBaseStructure.TIM_Prescaler     = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(MSD2_PULSE_TIM, &TIM_TimeBaseStructure);

    /* PWM 模式 2 + 低极性：输出正脉冲 */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = PULSE_WIDTH_US / 2;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;
    MSD2_PULSE_OCx_Init(MSD2_PULSE_TIM, &TIM_OCInitStructure);

    TIM_ARRPreloadConfig(MSD2_PULSE_TIM, ENABLE);
    TIM_UpdateRequestConfig(MSD2_PULSE_TIM, TIM_UpdateSource_Regular);
    TIM_ClearITPendingBit(MSD2_PULSE_TIM, TIM_IT_Update);
    TIM_ITConfig(MSD2_PULSE_TIM, TIM_IT_Update, ENABLE);
    TIM_Cmd(MSD2_PULSE_TIM, DISABLE);
}

/* ===================== 驱动初始化 ===================== */
/**
 * @brief  初始化两路电机的 GPIO、中断优先级与定时器
 */
void MSD_Init(void)
{
    MSD1_GPIO_Config();
    MSD2_GPIO_Config();
    TIM_NVIC_Config();
    MSD1_TIM_Config();
    MSD2_TIM_Config();
}

/* ===================== 驱动器使能控制 ===================== */
/**
 * @brief  使能/关闭电机 1 驱动输出
 * @param  NewState 非 0：ENA 输出高电平，关闭驱动（电机自由）；
 *                  0：ENA 输出低电平，开启驱动
 * @note   两路电机共用 status.output_enabled 状态，关闭后正在进行的
 *         运动会在当前脉冲结束时停止。
 */
void MSD_ENA1(FunctionalState NewState)
{
    if (NewState)
    {
        GPIO_SetBits(MSD1_ENA_PORT, MSD1_ENA_PIN);
        status.output_enabled = FALSE;
    }
    else
    {
        GPIO_ResetBits(MSD1_ENA_PORT, MSD1_ENA_PIN);
        status.output_enabled = TRUE;
    }
}

/**
 * @brief  使能/关闭电机 2 驱动输出
 * @param  NewState 非 0：关闭驱动；0：开启驱动
 */
void MSD_ENA2(FunctionalState NewState)
{
    if (NewState)
    {
        GPIO_SetBits(MSD2_ENA_PORT, MSD2_ENA_PIN);
        status.output_enabled = FALSE;
    }
    else
    {
        GPIO_ResetBits(MSD2_ENA_PORT, MSD2_ENA_PIN);
        status.output_enabled = TRUE;
    }
}

/* ===================== 运动控制 ===================== */
/**
 * @brief  启动电机 1 按梯形曲线运动指定步数（非阻塞，立即返回）
 * @param  step   目标步数：正数正转（CW），负数反转（CCW）
 * @param  accel  加速度（步/秒²）
 * @param  decel  减速度（步/秒²）
 * @param  speed  最高速度（步/秒）
 * @note   梯形曲线参数在函数内一次性计算完成，之后由 TIM3 中断逐脉冲推进：
 *         - 首步延时 = 0.676 * sqrt(2 * alpha / accel) * T1_FREQ（静止起步点）
 *         - 达到最高速度所需步数 max_s_lim
 *         - 减速起点 decel_start，走完 step 步后恰好减速到 0
 */
void MSD_Move1(signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
    SpeedRampData *data = &s_ramp1;
    unsigned int max_s_lim, accel_lim;

    /* 1. 解析方向：负步数表示反转 */
    if (step < 0)
    {
        data->dir = CCW;
        step = -step;
    }
    else
    {
        data->dir = CW;
    }
    DIR1(data->dir);

    /* 2. 单步运动：一个脉冲即停（直接以减速态结束） */
    if (step == 1)
    {
        data->accel_count = -1;
        data->run_state   = DECEL;
        data->step_delay  = 1000;
        status.motor1_running = TRUE;

        TIM_SetAutoreload(MSD1_PULSE_TIM, PULSE_WIDTH_US);
        TIM_SetCompare1(MSD1_PULSE_TIM, PULSE_WIDTH_US >> 1);
        TIM_Cmd(MSD1_PULSE_TIM, ENABLE);
    }
    /* 3. 多步运动：计算完整梯形曲线参数 */
    else if (step != 0)
    {
        /* 最高速度对应的最小步间延时（µs） */
        data->min_delay  = A_T_x100 / speed;
        /* 首步延时：静止起步时第一步的延时（加速段起点） */
        data->step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel)) / 100;

        /* 达到最高速度所需步数（speed² / (2 * alpha * accel) 的整数形式） */
        max_s_lim = (long)speed * speed / (long)(((long)A_x20000 * accel) / 100);
        if (max_s_lim == 0)
            max_s_lim = 1;

        /* 减速到停止所需步数（step * decel / (accel + decel)） */
        accel_lim = ((long)step * decel) / (accel + decel);
        if (accel_lim == 0)
            accel_lim = 1;

        /* 行程不足以达到最高速度：全程以加减速完成；否则先加速到匀速 */
        if (accel_lim <= max_s_lim)
            data->decel_val = accel_lim - step;
        else
            data->decel_val = -(long)(max_s_lim * accel / decel);

        if (data->decel_val == 0)
            data->decel_val = -1;

        /* 减速起点：总步数 + 减速计数值（负值） */
        data->decel_start = step + data->decel_val;

        /* 首步延时已降至最小延时：全程匀速，否则先加速 */
        if (data->step_delay <= data->min_delay)
        {
            data->step_delay = data->min_delay;
            data->run_state  = RUN;
        }
        else
        {
            data->run_state = ACCEL;
        }

        data->accel_count = 0;
        data->step_count  = 0;
        data->rest        = 0;
        status.motor1_running = TRUE;

        /* 启动定时器，进入中断驱动 */
        TIM_SetAutoreload(MSD1_PULSE_TIM, PULSE_WIDTH_US);
        TIM_SetCompare1(MSD1_PULSE_TIM, PULSE_WIDTH_US >> 1);
        TIM_Cmd(MSD1_PULSE_TIM, ENABLE);
    }
}

/**
 * @brief  启动电机 2 按梯形曲线运动指定步数（非阻塞，立即返回）
 * @param  step   目标步数：正数正转（CW），负数反转（CCW）
 * @param  accel  加速度（步/秒²）
 * @param  decel  减速度（步/秒²）
 * @param  speed  最高速度（步/秒）
 * @note   与 MSD_Move1 逻辑一致，使用 TIM4 通道。
 */
void MSD_Move2(signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
    SpeedRampData *data = &s_ramp2;
    unsigned int max_s_lim, accel_lim;

    /* 1. 解析方向：负步数表示反转 */
    if (step < 0)
    {
        data->dir = CCW;
        step = -step;
    }
    else
    {
        data->dir = CW;
    }
    DIR2(data->dir);

    /* 2. 单步运动：一个脉冲即停（直接以减速态结束） */
    if (step == 1)
    {
        data->accel_count = -1;
        data->run_state   = DECEL;
        data->step_delay  = 1000;
        status.motor2_running = TRUE;

        TIM_SetAutoreload(MSD2_PULSE_TIM, PULSE_WIDTH_US);
        TIM_SetCompare1(MSD2_PULSE_TIM, PULSE_WIDTH_US >> 1);
        TIM_Cmd(MSD2_PULSE_TIM, ENABLE);
    }
    /* 3. 多步运动：计算完整梯形曲线参数 */
    else if (step != 0)
    {
        /* 最高速度对应的最小步间延时（µs） */
        data->min_delay  = A_T_x100 / speed;
        /* 首步延时：静止起步时第一步的延时（加速段起点） */
        data->step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel)) / 100;

        /* 达到最高速度所需步数（speed² / (2 * alpha * accel) 的整数形式） */
        max_s_lim = (long)speed * speed / (long)(((long)A_x20000 * accel) / 100);
        if (max_s_lim == 0)
            max_s_lim = 1;

        /* 减速到停止所需步数（step * decel / (accel + decel)） */
        accel_lim = ((long)step * decel) / (accel + decel);
        if (accel_lim == 0)
            accel_lim = 1;

        /* 行程不足以达到最高速度：全程以加减速完成；否则先加速到匀速 */
        if (accel_lim <= max_s_lim)
            data->decel_val = accel_lim - step;
        else
            data->decel_val = -(long)(max_s_lim * accel / decel);

        if (data->decel_val == 0)
            data->decel_val = -1;

        /* 减速起点：总步数 + 减速计数值（负值） */
        data->decel_start = step + data->decel_val;

        /* 首步延时已降至最小延时：全程匀速，否则先加速 */
        if (data->step_delay <= data->min_delay)
        {
            data->step_delay = data->min_delay;
            data->run_state  = RUN;
        }
        else
        {
            data->run_state = ACCEL;
        }

        data->accel_count = 0;
        data->step_count  = 0;
        data->rest        = 0;
        status.motor2_running = TRUE;

        /* 启动定时器，进入中断驱动 */
        TIM_SetAutoreload(MSD2_PULSE_TIM, PULSE_WIDTH_US);
        TIM_SetCompare1(MSD2_PULSE_TIM, PULSE_WIDTH_US >> 1);
        TIM_Cmd(MSD2_PULSE_TIM, ENABLE);
    }
}

/* ===================== 位置计数与状态查询 ===================== */
/**
 * @brief  电机 1 位置计数器（中断内调用）
 * @param  inc 运动方向：CW 自增，CCW 自减
 */
static void MSD1_StepCounter(signed char inc)
{
    if (inc == CCW)
        s_ramp1.step_position--;
    else
        s_ramp1.step_position++;
}

/**
 * @brief  电机 2 位置计数器（中断内调用）
 * @param  inc 运动方向：CW 自增，CCW 自减
 */
static void MSD2_StepCounter(signed char inc)
{
    if (inc == CCW)
        s_ramp2.step_position--;
    else
        s_ramp2.step_position++;
}

/**
 * @brief  读取电机 1 累计位置（脉冲数，带符号）
 */
int MSD_GetPosition1(void)
{
    return s_ramp1.step_position;
}

/**
 * @brief  读取电机 2 累计位置（脉冲数，带符号）
 */
int MSD_GetPosition2(void)
{
    return s_ramp2.step_position;
}

/**
 * @brief  查询电机 1 是否正在运动
 * @retval 1：运动中；0：已停止
 */
unsigned char MSD_IsMotor1Running(void)
{
    return status.motor1_running;
}

/**
 * @brief  查询电机 2 是否正在运动
 * @retval 1：运动中；0：已停止
 */
unsigned char MSD_IsMotor2Running(void)
{
    return status.motor2_running;
}

/* ===================== 定时器中断服务：脉冲输出与速度斜坡推进 ===================== */
/**
 * @brief  TIM3 更新中断服务（电机 1）：每个中断输出一个脉冲并推进状态机
 *
 * 步间延时递推（AVR446 整数算法，等价于沿加速度曲线改变延时）：
 *   delay(n) = delay(n-1) - (2 * delay(n-1) + rest) / (4 * accel_count + 1)
 *   rest 保存除法余数，避免整数除法的累计精度损失；
 *   ACCEL 阶段延时逐脉冲缩短，DECEL 阶段逐脉冲延长，RUN 阶段保持最小值。
 */
void MSD1_PULSE_TIM_IRQHandler(void)
{
    SpeedRampData *data = &s_ramp1;
    unsigned int new_step_delay;

    if (TIM_GetITStatus(MSD1_PULSE_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(MSD1_PULSE_TIM, TIM_IT_Update);

        /* 更新本步的脉冲宽度与步间延时 */
        MSD1_PULSE_TIM->CCR1 = data->step_delay >> 1;
        MSD1_PULSE_TIM->ARR  = data->step_delay;

        /* 驱动器输出被关闭：立即停机 */
        if (status.output_enabled != TRUE)
            data->run_state = STOP;

        switch (data->run_state)
        {
            case STOP:
                /* 停机：关闭输出比较（停止脉冲）并关闭定时器 */
                data->step_count = 0;
                data->rest       = 0;
                MSD1_PULSE_TIM->CCER &= ~(1 << 0);
                TIM_Cmd(MSD1_PULSE_TIM, DISABLE);
                status.motor1_running = FALSE;
                break;

            case ACCEL:
                /* 加速段：发出脉冲，延时逐脉冲缩短 */
                MSD1_PULSE_TIM->CCER |= 1 << 0;
                MSD1_StepCounter(data->dir);
                data->step_count++;
                data->accel_count++;
                new_step_delay = data->step_delay - (((2 * (long)data->step_delay) + data->rest) / (4 * data->accel_count + 1));
                data->rest     = ((2 * (long)data->step_delay) + data->rest) % (4 * data->accel_count + 1);

                if (data->step_count >= data->decel_start)
                {
                    /* 行程不足以达到最高速度：直接转入减速段 */
                    data->accel_count = data->decel_val;
                    data->run_state   = DECEL;
                }
                else if (new_step_delay <= data->min_delay)
                {
                    /* 已达到最高速度：记录临界延时并转入匀速 */
                    data->last_accel_delay = new_step_delay;
                    new_step_delay         = data->min_delay;
                    data->rest             = 0;
                    data->run_state        = RUN;
                }
                data->step_delay = new_step_delay;
                break;

            case RUN:
                /* 匀速段：保持最小延时 */
                MSD1_PULSE_TIM->CCER |= 1 << 0;
                MSD1_StepCounter(data->dir);
                data->step_count++;
                new_step_delay = data->min_delay;

                if (data->step_count >= data->decel_start)
                {
                    /* 到达减速点：恢复加速结束时的临界延时，进入减速段 */
                    data->accel_count = data->decel_val;
                    new_step_delay    = data->last_accel_delay;
                    data->run_state   = DECEL;
                }
                data->step_delay = new_step_delay;
                break;

            case DECEL:
                /* 减速段：发出脉冲，延时逐脉冲延长，accel_count 归零即停机 */
                MSD1_PULSE_TIM->CCER |= 1 << 0;
                MSD1_StepCounter(data->dir);
                data->step_count++;
                data->accel_count++;
                new_step_delay = data->step_delay - (((2 * (long)data->step_delay) + data->rest) / (4 * data->accel_count + 1));
                data->rest     = ((2 * (long)data->step_delay) + data->rest) % (4 * data->accel_count + 1);

                if (data->accel_count >= 0)
                    data->run_state = STOP;
                data->step_delay = new_step_delay;
                break;
        }
    }
}

/**
 * @brief  TIM4 更新中断服务（电机 2）：逻辑与电机 1 完全一致
 */
void MSD2_PULSE_TIM_IRQHandler(void)
{
    SpeedRampData *data = &s_ramp2;
    unsigned int new_step_delay;

    if (TIM_GetITStatus(MSD2_PULSE_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(MSD2_PULSE_TIM, TIM_IT_Update);

        /* 更新本步的脉冲宽度与步间延时 */
        MSD2_PULSE_TIM->CCR1 = data->step_delay >> 1;
        MSD2_PULSE_TIM->ARR  = data->step_delay;

        /* 驱动器输出被关闭：立即停机 */
        if (status.output_enabled != TRUE)
            data->run_state = STOP;

        switch (data->run_state)
        {
            case STOP:
                /* 停机：关闭输出比较（停止脉冲）并关闭定时器 */
                data->step_count = 0;
                data->rest       = 0;
                MSD2_PULSE_TIM->CCER &= ~(1 << 0);
                TIM_Cmd(MSD2_PULSE_TIM, DISABLE);
                status.motor2_running = FALSE;
                break;

            case ACCEL:
                /* 加速段：发出脉冲，延时逐脉冲缩短 */
                MSD2_PULSE_TIM->CCER |= 1 << 0;
                MSD2_StepCounter(data->dir);
                data->step_count++;
                data->accel_count++;
                new_step_delay = data->step_delay - (((2 * (long)data->step_delay) + data->rest) / (4 * data->accel_count + 1));
                data->rest     = ((2 * (long)data->step_delay) + data->rest) % (4 * data->accel_count + 1);

                if (data->step_count >= data->decel_start)
                {
                    /* 行程不足以达到最高速度：直接转入减速段 */
                    data->accel_count = data->decel_val;
                    data->run_state   = DECEL;
                }
                else if (new_step_delay <= data->min_delay)
                {
                    /* 已达到最高速度：记录临界延时并转入匀速 */
                    data->last_accel_delay = new_step_delay;
                    new_step_delay         = data->min_delay;
                    data->rest             = 0;
                    data->run_state        = RUN;
                }
                data->step_delay = new_step_delay;
                break;

            case RUN:
                /* 匀速段：保持最小延时 */
                MSD2_PULSE_TIM->CCER |= 1 << 0;
                MSD2_StepCounter(data->dir);
                data->step_count++;
                new_step_delay = data->min_delay;

                if (data->step_count >= data->decel_start)
                {
                    /* 到达减速点：恢复加速结束时的临界延时，进入减速段 */
                    data->accel_count = data->decel_val;
                    new_step_delay    = data->last_accel_delay;
                    data->run_state   = DECEL;
                }
                data->step_delay = new_step_delay;
                break;

            case DECEL:
                /* 减速段：发出脉冲，延时逐脉冲延长，accel_count 归零即停机 */
                MSD2_PULSE_TIM->CCER |= 1 << 0;
                MSD2_StepCounter(data->dir);
                data->step_count++;
                data->accel_count++;
                new_step_delay = data->step_delay - (((2 * (long)data->step_delay) + data->rest) / (4 * data->accel_count + 1));
                data->rest     = ((2 * (long)data->step_delay) + data->rest) % (4 * data->accel_count + 1);

                if (data->accel_count >= 0)
                    data->run_state = STOP;
                data->step_delay = new_step_delay;
                break;
        }
    }
}
