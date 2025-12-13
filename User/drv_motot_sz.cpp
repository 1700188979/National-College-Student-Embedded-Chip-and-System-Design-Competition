//
// Created by Lenovo on 25-12-10.
//
// C++ file:
#include "drv_motot_sz.h"

// C file:
extern "C" {// 使用 C 语言链接方式
#include "arm_math.h"
}


void Class_Motor_SZ::Set_Angle(TIM_HandleTypeDef* htim, float angle)
{
    angle+=PI*0.5f;
    if (angle >= 0.99f*PI) angle = PI;
    if (angle <= 0)   angle=0.01f*PI;
    float duty = (angle / PI * 2.0f + 0.5f) * 0.05f;
    htim->Instance->CCR1 = (int32_t) (duty * 10000);
}
