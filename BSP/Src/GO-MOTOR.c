#include "GO-MOTOR.h"
#include "main.h"
#include "pid.h"
#include "gait_param.h"
#include "stdio.h"

MOTOR_send Motor_T[8]; // 以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
MOTOR_recv Motor_R[8];

PID_t PID_Position[8];	// 位置PID
PID_t PID_Speed[8];		// 速度PID
PID_t PID_IMU_Climbing; // 爬坡用PID
PID_t PID_IMU_Line;		// 直线修正pid

void Motor_Init(void) // 电机及其pid初始化
{
	for (int i = 0; i < 8; i++) // 电机id设置
	{
		Motor_T[i].id  = i + 1;
		Motor_T[i].mode = 1;
		Motor_T[i].T = 0;  			//前馈力矩
		Motor_T[i].W = 0;  			//期望角速度
		Motor_T[i].Pos = 0;			//期望角度位置
		Motor_T[i].K_P = 0.0;		//期望刚度
		Motor_T[i].K_W = 0.0f;	//期望阻尼
	}
	PID_Set_t pid_set =
		{
			.pid_mode = POSITION_PID,
			.MaxOutput = 6720.0f,
			.IntegralLimit = 2000.0f,
			.p = 0.20f,
			// .i = 0.0008f,
			.d = 0.000f,
		};             
	for (uint8_t i = 0; i < 8; i++) // 20,0.01,0  37,0.008   8.0f, 0.000f   16
	{
		PID_Init(&PID_Position[i], &pid_set); // 设置位置式PID
	}
	pid_set.MaxOutput = 12000.0f;
	pid_set.IntegralLimit = 2000.0f;
	pid_set.p = 0.012f;
	pid_set.i = 0.0001f;

	for (uint8_t i = 0; i < 8; i++) // 20,0.01,0  37,0.008   8.0f, 0.000f   16
	{
		PID_Init(&PID_Speed[i], &pid_set); // 设置速度式PID
	}

	pid_set.MaxOutput = 10000.0f;
	pid_set.IntegralLimit = 0.0f;
	pid_set.p = 10.0f; // 3.5
	pid_set.i = 0.0f;
	pid_set.d = 0.0f;
	PID_Init(&PID_IMU_Climbing, &pid_set); // 设置爬坡PID

	pid_set.MaxOutput = 16.0f;
	pid_set.IntegralLimit = 0.0f;
	pid_set.p = 1.5f;
	pid_set.i = 0.0f;
	pid_set.d = 0.01f;
	PID_Init(&PID_IMU_Line, &pid_set); // 设置IMU直线修正PID

	pid_set.MaxOutput = 10000.0f;
	pid_set.IntegralLimit = 0.0f;
	pid_set.p = 10.0f; // 3.5
	pid_set.i = 0.0f;
	pid_set.d = 0.0f;
	PID_Init(&PID_IMU_Climbing, &pid_set); // 设置爬坡PID

	for (uint8_t i = 0; i < 4; i++)
	{
		Stanford_Type_Lite_Init(&Gait_Data[i], 15.0f, 27.0f); // 设置腿部长度参数 新马腿长22.0f 方案二机构：大腿16.5 小腿27.5
	}
}

void GO_motor_speed(MOTOR_send *motor1, int id, float W) // 速度模式
{
	motor1->id = id; // 给电机控制指令结构体赋值
	motor1->mode = 1;
	motor1->T = 0;
	motor1->W = W;
	motor1->Pos = 0;
	motor1->K_P = 0;
	motor1->K_W = 0.05;
}

void GO_motor_position(MOTOR_send *motor1, int id, float Pos) // 位置模式
{
	motor1->id = id; // 给电机控制指令结构体赋值
	motor1->mode = 1;
	motor1->T = 0;
	motor1->W = 0;//10*6.33f;
	motor1->Pos = Pos * 6.33f; // 直接填写弧度   //意思为电机转子转6.33圈输出端转1圈
	motor1->K_P = motor1->K_P;
	motor1->K_W = motor1->K_W;
	
}

void GO_motor_damp(MOTOR_send *motor1, int id, float K_W) // 阻尼模式
{
	motor1->id = id; // 给电机控制指令结构体赋值
	motor1->mode = 1;
	motor1->T = 0;
	motor1->W = 0;
	motor1->Pos = 0.0;
	motor1->K_P = 0.00;
	motor1->K_W = 0.02;
}

void GO_motor_moment(MOTOR_send *motor1, int id, float T) // 力矩模式
{
	motor1->id = id; // 给电机控制指令结构体赋值
	motor1->mode = 1;
	motor1->T = T;
	motor1->W = 0;
	motor1->Pos = 0.0;
	motor1->K_P = 0.0;
	motor1->K_W = 0.0;
}

void GO_motor_Zero_moment(MOTOR_send *motor1, int id) // 零力矩模式
{
	motor1->id = id; // 给电机控制指令结构体赋值
	motor1->mode = 1;
	motor1->T = 0;
	motor1->W = 0;
	motor1->Pos = 0.0;
	motor1->K_P = 0.0;
	motor1->K_W = 0.0;
}

void GO_motor_Mix_moment(MOTOR_send *motor1, MOTOR_recv *rdata, int id, float T, float W, float Pos, float K_P, float K_W) // 混合力矩模式
{
	motor1->id = id; // 给电机控制指令结构体赋值
	motor1->mode = 1;
	motor1->T = T;
	motor1->W = W;
	motor1->Pos = Pos;
	motor1->K_P = K_P;
	motor1->K_W = K_W;
}
