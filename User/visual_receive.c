//
// Created by Lenovo on 25-12-13.
//

#include "visual_receive.h"
#include "string.h"

void Visual_Get_Convert(uint8_t *buffer, uint8_t length)
{
    if (length != 14) return; // 通信协议约定数据帧固定长度为 14

    if (buffer[0] != FRAME_HEADER || buffer[13] != FRAME_TAIL)
        return;
    memcpy(&Data_Visual_Receive,buffer+1,sizeof(Visual_RxConvert_Typedef));
}
