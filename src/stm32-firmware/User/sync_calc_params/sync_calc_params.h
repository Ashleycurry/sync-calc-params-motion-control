/**
 * @file    sync_calc_params.h
 * @brief   Sync_CalcParams 双轴同步算法接口
 * @author  Ashleycurry
 * @version 1.0.0
 */

#ifndef __SYNC_CALC_PARAMS_H
#define __SYNC_CALC_PARAMS_H

#include "stm32f10x.h"

/* Sync_CalcParams 双轴同步算法：按主轴比例缩放从轴的 speed/accel/decel，使两轴运动时间一致 */
void Sync_CalcParams(void);

#endif /* __SYNC_CALC_PARAMS_H */
