//
// Created by Lenovo on 25-12-13.
//

#ifndef VISUAL_RECEIVE_H
#define VISUAL_RECEIVE_H

#include "stm32h7xx_hal.h"

#define FRAME_HEADER 0x0A
#define FRAME_TAIL 0x6B
/**
 * @brief openmv数据接收处理函数
 *     0x0A x y z 0x6b
 */
typedef struct Visual_RxConvert_Typedef
{
    uint8_t head;
    float x;
    float y;
    float z;
    uint8_t tail;
}Visual_RxConvert_Typedef;

void Visual_Get_Convert(uint8_t *buffer, uint8_t length);

extern Visual_RxConvert_Typedef Data_Visual_Receive;
extern float intime_x,intime_y,intime_z;
#endif //VISUAL_RECEIVE_H
