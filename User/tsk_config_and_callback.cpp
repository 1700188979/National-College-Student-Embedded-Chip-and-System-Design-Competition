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
Visual_RxConvert_Typedef Data_Visual_Receive;
float intime_x,intime_y,intime_z;

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
	// 10Hz
	static int alive_mod100 = 0;
	alive_mod100++;
	if (alive_mod100 == 100)
	{
		alive_mod100 = 0;
	}

	//100Hz
	static int interaction_mod10 = 0;
	interaction_mod10++;
	if (interaction_mod10 == 10)
	{
		interaction_mod10 = 0;


		tempFloat[0]=RoboticArm.DH_arm_motor[0].Next_Angle*180/PI;
		tempFloat[1]=RoboticArm.DH_arm_motor[0].Next_Omega*180/PI;
		tempFloat[2]=RoboticArm.DH_arm_motor[1].Next_Angle*180/PI;
		tempFloat[3]=RoboticArm.DH_arm_motor[1].Next_Omega*180/PI;
		tempFloat[4]=RoboticArm.DH_arm_motor[2].Next_Angle*180/PI;
		tempFloat[5]=RoboticArm.DH_arm_motor[2].Next_Omega*180/PI;
		// Vofa_Transmit(&huart2,6);

	}
	//50Hz
	static int data_mod20 = 0;
	data_mod20++;
	if (data_mod20 == 2)
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET);
		data_mod20 = 0;
		//路径规划计算
		if (RoboticArm.path_finish_flag!=1)
		{
			RoboticArm.FunTimes++;
			RoboticArm.Joint_Space_4Angle_Group_Path_Planning();
		}
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
	UART_DMA_Receive_init(&huart2, buffer_receive_2, buffer_receive_length_2);//

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
	// RoboticArm.Set_Target_point(0.49,0.15,-0.0897,0.05,Facing_Downward,Third_Order);//点0

	/*
	 *-0.03735-0.06时z=0
	 *
	 *
	 */
	// RoboticArm.Set_Target_point(0.18718,0.13107,-0.07644,2,Facing_Downward,Third_Order);//点0
	// RoboticArm.Set_Target_point(0.15434,0.15434,-0.06127 ,2,Facing_Downward,Third_Order);//1
	// RoboticArm.Set_Target_point(0.11790,0.16838,-0.04810,2,Facing_Downward,Third_Order);//2
	// RoboticArm.Set_Target_point(0.08061,0.17288,-0.03735-0.06,2,Facing_Downward,Third_Order);//3

	RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0484,0.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0554,0.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0654,0.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0804,0.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0904,0.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0904,1,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.16718,-0.0407,-0.0624,0.3,Facing_Downward,Third_Order);//点
	RoboticArm.Set_Target_point(0.15434,-0.15434,-0.04127,0.3,Facing_Downward,Third_Order);//1
	RoboticArm.Set_Target_point(0.04061,-0.18288,-0.01,0.5,Facing_Downward,Third_Order);//3

	// RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0804,0.2,Facing_Downward,Third_Order);//点
	// RoboticArm.Set_Target_point(0.18718,-0.0107,-0.0804,1,Facing_Downward,Third_Order);//点
	// RoboticArm.Set_Target_point(0.18718,-0.13107,-0.05644,1,Facing_Downward,Third_Order);//点0
	// RoboticArm.Set_Target_point(0.15434,-0.15434,-0.04127,0.5,Facing_Downward,Third_Order);//1
	// RoboticArm.Set_Target_point(0.11790,-0.16838,-0.0210,0.5,Facing_Downward,Third_Order);//2
	// RoboticArm.Set_Target_point(0.08061,-0.17288,-0.0235,0.5,Facing_Downward,Third_Order);//3
	RoboticArm.Joint_Space_Preprocessing();
	HAL_Delay(2000);
	HAL_Delay(200);
    init_finished = true;
	HAL_Delay(1500);
	suction_control(1);
	HAL_Delay(5000);
	suction_control(0);
	//初始化完成

    while (1)
    {
    	// RoboticArm.Robotic_Motor_Set();

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

