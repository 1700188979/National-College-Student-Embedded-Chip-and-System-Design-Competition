/**
 * @file tsk_config_and_callback.cpp
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 *
 */

/* Includes ------------------------------------------------------------------*/

// .Cpp file:
#include "tsk_config_and_callback.h"         // 包含自定义的头文件

// 添加 .cpp 的头文件（通常放置在此处）
#include "arm_math.h"
#include "drv_tim_h7.h"
#include "drv_fdcan_h7.h"
#include "drv_math.h"
#include "robotic_arm.h"
// .C file:
extern "C" {              // 使用 C 语言链接方式
#include "UART_DMA_h7.h"
#include "vofa.h"
#include "vofa_setting.h"
#include "stdio.h"
#include "stdint.h"
#include "visual_receive.h"
}
// 全局初始化完成标志位
bool init_finished = false;

Class_Robotic_arm RoboticArm;

/**
 * @brief FDCAN1 回调函数
 *
 * @param FDCAN_RxMessage FDCAN1 收到的消息
 */
void Device_FDCAN1_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{
	// FDCAN: 标准帧 = FDCAN_STANDARD_ID，扩展帧 = FDCAN_EXTENDED_ID
	if (FDCAN_RxMessage->Header.IdType == FDCAN_STANDARD_ID)//判断是否为标准帧 —— RM2006/3508/6020,DM电机
	{

		switch (FDCAN_RxMessage->Header.Identifier)
		{
			case 0x00:
			{
				break;
			}
			default:
				break;
		}
	}
	else if (FDCAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID)//判断是否为扩展帧（VESC）
	{
		switch (FDCAN_RxMessage->Header.Identifier)
		{

			default:
				break;
		}
	}
}

/**
 * @brief FDCAN2 回调函数
 *
 * @param FDCAN_RxMessage FDCAN2 收到的消息
 */
void Device_FDCAN2_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{
	// FDCAN: 标准帧 = FDCAN_STANDARD_ID，扩展帧 = FDCAN_EXTENDED_ID
	if (FDCAN_RxMessage->Header.IdType == FDCAN_STANDARD_ID)//判断是否为标准帧 —— RM2006/3508/6020,DM电机
	{

		switch (FDCAN_RxMessage->Header.Identifier)
		{
			case 0x00:
			{
				break;
			}

			default:
				break;
		}
	}
	else if (FDCAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID)//判断是否为扩展帧（VESC）
	{
		switch (FDCAN_RxMessage->Header.Identifier)
		{

			default:
				break;
		}
	}
}

/**
 * @brief TIM5任务回调函数,每1ms进入一次
 *
 */
void Task1ms_TIM5_Callback()
{
	// // 0.5Hz
	// static int alive_mod2000 = 0;
	// alive_mod2000++;
	// if (alive_mod2000 == 1000)
	// {
	// 	alive_mod2000 = 0;
	// 	if (RoboticArm.Intime_Joint_Space_Dynamic_Tuning(RoboticArm.Intime_x,RoboticArm.Intime_y,RoboticArm.Intime_z,RoboticArm.Intime_T,Facing_Downward,Fifth_Order)==1)//有效坐标
	// 	{
	// 		RoboticArm.intime_path_finish_flag=0;
	// 	}
	// }

	//100Hz
	static int interaction_mod10 = 0;
	interaction_mod10++;
	if (interaction_mod10 == 10)
	{
		interaction_mod10 = 0;

		// tempFloat[0]=Vofa_Slider1;
		// tempFloat[1]=Vofa_Slider2;
		// tempFloat[2]=Vofa_Slider3;
		// tempFloat[3]=Vofa_Slider4;

		tempFloat[0]=RoboticArm.Intime_x;
		tempFloat[1]=RoboticArm.Intime_y;
		tempFloat[2]=RoboticArm.Intime_z;
		tempFloat[3]=RoboticArm.Intime_T;

		tempFloat[0+4]=RoboticArm.DH_arm_motor[0].Next_Angle*180/PI;
		tempFloat[1+4]=RoboticArm.DH_arm_motor[0].Next_Omega*180/PI;
		tempFloat[2+4]=RoboticArm.DH_arm_motor[1].Next_Angle*180/PI;
		tempFloat[3+4]=RoboticArm.DH_arm_motor[1].Next_Omega*180/PI;
		tempFloat[4+4]=RoboticArm.DH_arm_motor[2].Next_Angle*180/PI;
		tempFloat[5+4]=RoboticArm.DH_arm_motor[2].Next_Omega*180/PI;
		// Vofa_Transmit(&huart1,10);
	}

	//100Hz
	static int data_mod1 = 0;
	data_mod1++;
	if (data_mod1 == 10)
	{
		data_mod1=0;
		//多点规划模式
		RoboticArm.Multi_Point_Planning_Mode();
		//单点规划模式（即时）
		RoboticArm.Single_Point_Planning_Mode();
		//视觉规划模式（即时）
		RoboticArm.Visual_Planning_Mode();
		//电机PID计算
		RoboticArm.Robotic_TIM_Send_PeriodElapsedCallback();
	}
}
/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
	HAL_Delay(2000);

	// FDCAN 总线初始化
	FDCAN_Init(&hfdcan1, Device_FDCAN1_Callback);
	FDCAN_Init(&hfdcan2, Device_FDCAN2_Callback);

    // UART初始化
    UART_DMA_Receive_init(&huart1, buffer_receive_1, buffer_receive_length_1);//VOFA+
	UART_DMA_Receive_init(&huart2, buffer_receive_2, buffer_receive_length_2);//openmv
	UART_DMA_Receive_init(&huart3, buffer_receive_3, buffer_receive_length_3);//陀螺仪

    // 定时器初始化
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    // 使能调度时钟
    HAL_TIM_Base_Start_IT(&htim5);

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	//机械臂初始化
	RoboticArm.Init();
	/*
	 *-0.03735-0.06时z=0
	 */

	// RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0484,0.3,Facing_Downward,Third_Order);//点
	// RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0554,0.3,Facing_Downward,Third_Order);//点
	// RoboticArm.Set_Target_point(0.18718,-0.0107,-0.06504,0.3,Facing_Downward,Third_Order);//点
	// RoboticArm.Set_Target_point(0.18718,-0.0107,-0.07504,0.3,Facing_Downward,Third_Order);//点
	// RoboticArm.Set_Target_point(0.18718,-0.0107,-0.09004,0.3,Facing_Downward,Third_Order);//点
	// RoboticArm.Set_Target_point(0.18718,-0.0107,-0.09004,1.3,Facing_Downward,Third_Order);//点
	// RoboticArm.Set_Target_point(0.16718,-0.0807,-0.06004,0.5,Facing_Downward,Third_Order);//点
	// RoboticArm.Set_Target_point(0.06061,-0.16288,-0.01,10,Facing_Downward,Third_Order);//3

	RoboticArm.Set_Target_point(0.17718,0.0507,-0.0484,0.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.17718,0.0507,-0.0554,0.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.17718,0.0507,-0.06504,0.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.17718,0.0507,-0.07504,0.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.17718,0.0507,-0.09004,0.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.17718,0.0507,-0.09004,1.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.16718,0.0807,-0.06004,0.5,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.04061,0.18288,-0.02,0.5,Facing_Downward,Third_Order);//3


	// RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0484,0.3,Facing_Downward,Third_Order);//点
	// // RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0554,0.3,Facing_Downward,Third_Order);//点
	// // RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0654,0.3,Facing_Downward,Third_Order);//点
	// // RoboticArm.Set_Target_point(0.16718,-0.0807,-0.07004,0.3,Facing_Downward,Third_Order);//点
	// // RoboticArm.Set_Target_point(0.18718,0.0107,-0.08004,0.3,Facing_Downward,Third_Order);//点
	// RoboticArm.Set_Target_point(0.18718,0.08007,-0.09004,1,Facing_Downward,Third_Order);//点
	// // RoboticArm.Set_Target_point(0.16718,-0.0407,-0.0624,0.3,Facing_Downward,Third_Order);//点
	// // RoboticArm.Set_Target_point(0.15434,0.15434,-0.08004,0.3,Facing_Downward,Third_Order);//1
	// // RoboticArm.Set_Target_point(0.04061,-0.18288,-0.01,0.5,Facing_Downward,Third_Order);//3

	// RoboticArm.Set_Target_point(0.56730,0.39723,-0.10897,0.5,Facing_Downward,Fifth_Order);//点0
	// RoboticArm.Set_Target_point(0.46282,0.46282,-0.05261 ,0.5,Facing_Downward,Fifth_Order);//1
	// RoboticArm.Set_Target_point(0.34833,0.49747,-0.00370,0.5,Facing_Downward,Fifth_Order);//2
	// RoboticArm.Set_Target_point(0.23341,0.50056,0.03625,0.5,Facing_Downward,Fifth_Order);//3
	// RoboticArm.Set_Target_point(0.56730,0.39723,-0.10897,0.5,Facing_Downward,Fifth_Order);//点0
	// RoboticArm.Set_Target_point(0.46282,0.46282,-0.05261 ,0.5,Facing_Downward,Fifth_Order);//1
	// RoboticArm.Set_Target_point(0.34833,0.49747,-0.00370,0.5,Facing_Downward,Fifth_Order);//2
	// RoboticArm.Set_Target_point(0.23341,0.50056,0.03625,0.5,Facing_Downward,Fifth_Order);//3
	RoboticArm.Joint_Space_Preprocessing();
	HAL_Delay(2000);
	init_finished = true;
	HAL_Delay(1500);
	suction_control(1);
	HAL_Delay(5000);
	suction_control(0);
	// Transmit_Visual(0.18718,-0.0107,-0.09404);
	//初始化完成
	init_finished = true;
    while (1)
    {
    	//单点规划模式在主函数中的程序
    	RoboticArm.Single_Point_Planning_Mode_Handle_Main();
    	//视觉规划模式在主函数中的程序
    	RoboticArm.Visual_Planning_Mode_Handle_Main();
    }
}

void H7_TIM_CALLBACK(TIM_HandleTypeDef *htim)
{
	H7_HAL_TIM_PeriodElapsedCallback(htim);
}

//freertos
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	H7_TIM_CALLBACK(htim);
}

