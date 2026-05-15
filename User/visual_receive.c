//
// Created by Lenovo on 25-12-13.
//

#include "visual_receive.h"
#include "string.h"
#include "usart.h"


uint8_t text_rx_finish[2]={FRAME_HEADER,FRAME_TAIL};
Visual_RxConvert_Typedef Data_Visual_Receive;
float intime_x,intime_y,intime_z;
uint8_t Vofa_Receive_flag=0;
uint8_t VS_Receive_flag=0;

void Visual_Get_Convert(uint8_t *buffer, uint8_t length)
{
    if (length != 14) return; // 通信协议约定数据帧固定长度为 14

    if (buffer[0] != FRAME_HEADER || buffer[13] != FRAME_TAIL)
        return;
    memcpy(&Data_Visual_Receive,buffer,sizeof(Visual_RxConvert_Typedef));

    VS_Receive_flag=1;
}

uint8_t Send_Data[14];
/*
 * 规定 0x0A (float)x (float)y (float)z 0x6B
 */
void Transmit_Visual(float x, float y, float z)
{
    float result[3];

    memcpy(Send_Data+1, (uint8_t*)result, 12);
    Send_Data[0]=FRAME_HEADER;
    Send_Data[13]=FRAME_TAIL;
    HAL_UART_Transmit(&huart1,Send_Data, sizeof(Send_Data),0xff);
}
