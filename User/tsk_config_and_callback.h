/**
* @file tsk_config_and_callback.h
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 *
 */

#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

    void Task_Init();
    void H7_TIM_CALLBACK(TIM_HandleTypeDef *htim);
#ifdef __cplusplus
}
#endif

#endif

