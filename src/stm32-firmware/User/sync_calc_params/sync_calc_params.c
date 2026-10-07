/**
 * @file    sync_calc_params.c
 * @brief   Sync_CalcParams 双轴同步算法：按比例缩放从轴参数，实现两轴同时到达
 * @author  Ashleycurry
 * @version 1.0.0
 *
 * 算法原理：
 *  梯形速度曲线的时间轴形状由 speed/accel 与 speed/decel 决定，与步数无关。
 *  取步数多的轴为主轴（参数保持不变），从轴的 speed/accel/decel 统一乘以
 *  比例系数 ratio = 从轴步数 / 主轴步数，则从轴各段时间与主轴完全相同：
 *
 *      t_accel = speed / accel
 *             = (speed主 * ratio) / (accel主 * ratio)
 *             = speed主 / accel主
 *
 *  即两轴同时启动、同时停止，且各自按完整梯形曲线运行。
 *
 * 输入：上位机下发的原始参数（全局变量 g_step/g_accel/g_decel/g_speed）；
 * 输出：写回从轴对应全局变量，供 MSD_Move1 / MSD_Move2 使用。
 */

#include "sync_calc_params.h"
#include "bsp_usart.h"

/**
 * @brief  Sync_CalcParams 双轴同步算法（在启动运动前调用一次）
 * @note   边界处理：
 *         - 任一轴步数为 0 时不参与同步，直接返回；
 *         - 缩放计算使用 32 位中间量，避免乘法溢出；
 *         - 缩放结果最小为 1，防止速度/加速度为 0 导致电机停转。
 */
void Sync_CalcParams(void)
{
    unsigned int steps_m, steps_s;
    unsigned int accel_m, decel_m, speed_m;
    /* 取绝对值用于比较（负值仅表示方向，不影响时间同步） */
    unsigned int abs1 = (g_step1 >= 0) ? (unsigned int)g_step1 : (unsigned int)(-g_step1);
    unsigned int abs2 = (g_step2 >= 0) ? (unsigned int)g_step2 : (unsigned int)(-g_step2);

    if (abs1 == 0 || abs2 == 0)
    {
        /* 存在步数为 0 的轴：无需同步，直接返回 */
        return;
    }

    if (abs1 >= abs2)
    {
        /* 电机 1 为主轴，电机 2 为从轴：按 ratio = abs2 / abs1 缩放 */
        steps_m = abs1;
        steps_s = abs2;
        accel_m = g_accel1;
        decel_m = g_decel1;
        speed_m = g_speed1;

        /* 32 位中间量防止乘法溢出 */
        g_accel2 = (unsigned int)((unsigned long)accel_m * steps_s / steps_m);
        g_decel2 = (unsigned int)((unsigned long)decel_m * steps_s / steps_m);
        g_speed2 = (unsigned int)((unsigned long)speed_m * steps_s / steps_m);

        /* 缩放结果最小值为 1，防止除零与停转 */
        if (g_accel2 == 0) g_accel2 = 1;
        if (g_decel2 == 0) g_decel2 = 1;
        if (g_speed2 == 0) g_speed2 = 1;
    }
    else
    {
        /* 电机 2 为主轴，电机 1 为从轴：按 ratio = abs1 / abs2 缩放 */
        steps_m = abs2;
        steps_s = abs1;
        accel_m = g_accel2;
        decel_m = g_decel2;
        speed_m = g_speed2;

        g_accel1 = (unsigned int)((unsigned long)accel_m * steps_s / steps_m);
        g_decel1 = (unsigned int)((unsigned long)decel_m * steps_s / steps_m);
        g_speed1 = (unsigned int)((unsigned long)speed_m * steps_s / steps_m);

        if (g_accel1 == 0) g_accel1 = 1;
        if (g_decel1 == 0) g_decel1 = 1;
        if (g_speed1 == 0) g_speed1 = 1;
    }
}
