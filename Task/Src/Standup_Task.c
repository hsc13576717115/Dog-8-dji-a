#include "Standup_Task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "GO-MOTOR.h"
#include "motor_task.h"

#include "pid.h"
#include "ultrasonic.h"
#include "gpio.h"
#include "jump_task.h"
#include "VMC.h"
#include "High_Angle.h"

extern osThreadId_t StandupHandle;
extern PID_t PID_Speed[8];
extern polarCoordinates_C C_polar;
extern PID_t PID_Position[8];
extern MOTOR_send Motor_T[8]; // 以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
extern MOTOR_recv Motor_R[8];
extern float StandUp_Angle_Individual[8];
float Standup_LegR_Offset = 0;
float Standup_LegL_Offset = 0;
float LegLinit_onestep;
float LegRinit_onestep;
float LegLinit_twostep;
float LegRinit_twostep;

float CREEP_LegR = 0;
float CREEP_LegL = 0;
CREEP_Leg_t flag_creep_LR = LRNULL; //1左 2右 

float Initial_Angle[8] = {0, 0, 0, 0, 0, 0, 0, 0};      //初始角度补偿
uint8_t flag_Standup = 0;
uint8_t rate=7;
uint16_t ratetime;
uint16_t high_angle=0;


void Standup_Task(void *argument)
{
        while (1)
        {
					
					
                uint8_t i = 0;
								
                for (int i = 0; i < 8; i++) // 电机id设置
                {
                        Motor_T[i].K_P = 0.35f; //0.35
                        Motor_T[i].K_W = 0.03f; //0.03
                }
								uint16_t K_Pt=Motor_T[0].K_P*200;
								//ratetime=210-(rate-5)*15-K_Pt;//越小越快 大了会有明显顿挫感  //0.4计算为130  但90更为明显/快 有时间试试
								ratetime=90-(rate-5)*15;//0.4 90更快 上面这个还要调
                // 外侧腿为左，内侧腿为右
								for(i=0;i<=65;i+=rate)
								{
									Standup_LegL_Offset = (-i) * 3.14159f / 180.0f;  //负是逆时针
									Standup_LegR_Offset = (i) * 3.14159f / 180.0f;
									osDelay(ratetime);                                    // （小腿16.5 大腿27.5）
								}
								LegLinit_onestep=Standup_LegL_Offset;
								LegRinit_onestep=Standup_LegR_Offset;
								
								for(i=0;i<=65;i+=rate)
								{
									Standup_LegR_Offset -= (rate-2) * 3.14159f / 180.0f;  // 40         35
									Standup_LegL_Offset -= (rate) * 3.14159f / 180.0f; //-60    -55
									osDelay(ratetime);
								}
								osDelay(100);
								
//								high_angle=solve_alpha(100);
//								high_angle=High_To_Angle(150);
//								Change_Angle(high_angle);
//								osDelay(10000);
								
								if (smart == 1)
									CHANGE_KPW(1.00f,0.012f);
								else if (smart == 0)
									CHANGE_KPW(1.55f,0.023f);    //1.55     0.023   1.00/0.012  1.22/0.016 1.55/0.023
								
								
//                for (i = 0; i < 8; i++)
//                {
//                        Motor_T[i].K_P = 1.55f;  //1.22是摩擦小的情况飞   1.55f 测试  3.2   1.00    
//                        Motor_T[i].K_W = 0.023f; //0.016f									0.023f		0.08   0.012
//                }
									
									
								LegLinit_twostep=Standup_LegL_Offset;
								LegRinit_twostep=Standup_LegR_Offset;
                flag_Standup = 1;
//								while(1)
//								{
//									if(HAL_GPIO_ReadPin(GPIOH, RS485_RE_Pin) == 1)
//										break;
//								}

								
								
								
								
								
								
								/* 反转测试 */
								
//								for (int i = 0; i < 8; i++) // 电机id设置
//                {
//                        Motor_T[i].K_P = 0.1f; //0.4
//                        Motor_T[i].K_W = 0.08f;
//                }
//								Standup_LegL_Offset = (-300) * 3.14159f / 180.0f;  //负是逆时针
//								Standup_LegR_Offset = (230) * 3.14159f / 180.0f;
//								osDelay(15000);
//								flag_Standup = 1;
								/*****************************/
                osThreadSuspend(StandupHandle);
        }
}

void Setdown(void)
{
			for (int i = 0; i < 8; i++)
			{
							Motor_T[i].K_P = 0.25f;  //0.25
							Motor_T[i].K_W = 0.08f;  //0.08
			}
			osDelay(150);
			Standup_LegL_Offset = (-0) * 3.14159f / 180.0f;   //-15
			Standup_LegR_Offset = (0)  * 3.14159f / 180.0f;   //10
			CREEP_LegL = 0.0f;
			CREEP_LegR = 0.0f;
			flag_Standup = 0;
}


void CHANGE_HIGH(Robohorse_State now,RC_ctrl_t * rc_data,uint8_t * flag)
{
	if (now==START)
	{
		Standup_LegL_Offset=LegLinit_twostep;
		Standup_LegR_Offset=LegRinit_twostep;
	}
	if (now == NORMAL)
	{
		if (*flag!=0 && rc_data->rc.ch[3]>1600)
		{
			for (int i = 0; i < 8; i++)
			{
							Motor_T[i].K_P = 0.35f;  
							Motor_T[i].K_W = 0.03f;   
			}
			//osDelay(100);
			Standup_LegL_Offset=LegLinit_twostep;
			Standup_LegR_Offset=LegRinit_twostep;
			//flag_changehigh=1;
		}
		else if (rc_data->rc.ch[3]<1000 && *flag != 0)
		{
			for (int i = 0; i < 8; i++)
			{
							Motor_T[i].K_P = 1.55f;  
							Motor_T[i].K_W = 0.023f;   
			}
			(*flag)=0;
		}
	}
	else if (now ==CREEP)
	{
		if (*flag==0 && rc_data->rc.ch[3]>1600)
		{
			for (int i = 0; i < 8; i++)
			{
							Motor_T[i].K_P = 0.15f; //0.25
							Motor_T[i].K_W = 0.03f;   
			}
			//osDelay(110);
			//high_angle=High_To_Angle(128);  //128
			Change_Angle(160);  //creep160
			//flag_changehigh=1;
		}
		else if (rc_data->rc.ch[3]<1000 && *flag == 0)
		{
			for (int i = 0; i < 8; i++)
			{
							Motor_T[i].K_P = 1.55f;  
							Motor_T[i].K_W = 0.023f;   
			}
			(*flag)++;
		}
	}
}

void CHANGE_HIGH_AUTO(Robohorse_State now,uint8_t flag_mode,uint8_t * flag)
{
	if (now==START)
	{
		Standup_LegL_Offset=LegLinit_twostep;
		Standup_LegR_Offset=LegRinit_twostep;
	}
	if (now == NORMAL)
	{
		if (*flag!=0 && flag_mode == 1)
		{
			for (int i = 0; i < 8; i++)
			{
							Motor_T[i].K_P = 0.35f;  
							Motor_T[i].K_W = 0.03f;   
			}
			//osDelay(100);
			Standup_LegL_Offset=LegLinit_twostep;
			Standup_LegR_Offset=LegRinit_twostep;
			//flag_changehigh=1;
		}
		else if (flag_mode == 2 && *flag != 0)
		{
			for (int i = 0; i < 8; i++)
			{
							Motor_T[i].K_P = 1.55f;  
							Motor_T[i].K_W = 0.023f;   
			}
			(*flag)=0;
		}
	}
	else if (now ==CREEP)
	{
		if (*flag==0 && flag_mode == 3)
		{
			for (int i = 0; i < 8; i++)
			{
							Motor_T[i].K_P = 0.15f; //0.25
							Motor_T[i].K_W = 0.03f;   
			}
			//osDelay(110);
			//high_angle=High_To_Angle(128);  //128
			Change_Angle(160);  //creep160
			//flag_changehigh=1;
		}
		else if (flag_mode == 4 && *flag == 0)
		{
			for (int i = 0; i < 8; i++)
			{
							Motor_T[i].K_P = 1.55f;  
							Motor_T[i].K_W = 0.023f;   
			}
			(*flag)++;
		}
	}
}

//p值调大速度就快
//w值大 缓冲就打所以速度就会慢
//p大w小 速度巨快因为没有阻尼 缓冲就小了
//p小w大 速度特慢因为刚有点速度就被阻尼挡住了
void CHANGE_KPW(float k_pt,float k_wt)
{
	for (int i = 0; i < 8; i++)
			{
				Motor_T[i].K_P = k_pt;  //[0,25.599]  刚度
				Motor_T[i].K_W = k_wt;  //[0,25.599]  阻尼
			}
}

