#include "posture_task.h"

#include "auto_action.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "VMC.h"

#include "cycloid_generator.h"
#include "gait_param.h"
#include "pid.h"

#include "serial.h"
#include "ahrs_task.h"
#include "Vision.h"
#include "laser.h"
#include "jy901s.h"

#include "standup_task.h"
#include "ahrs_task.h"
#include "remoctrl_task.h"
#include "jump_task.h"
#include "Vision.h"

#include "math.h"

#include "ultrasonic.h"


float aim_x[4] = {0.0f, 0.0f, 0.0f, 0.0f}, aim_y[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 目标点暂存
float now_time = 0.0f;                                                          // 目前时间 (用于PLAN A)
uint8_t smart_off = 0;
uint8_t turn_flag = 0;
uint8_t flag_change = 0;

float now_state = 0;

extern polarCoordinates_C C_polar;

uint8_t go_back_flag = 0;
uint8_t xuanzhuan_flag = 0;
uint8_t jump_flag = 0;
uint8_t high_jump_flag = 0;

uint8_t flag_go = 0;
uint8_t flag_jump = 0;
uint8_t Buttom[6] = {0}; //按下为1 不按为0

uint8_t end_t = 0;

extern osThreadId_t JumpHandle;


void Calculate_Gait(Robohorse_State state);
void Target_toMotor(void);
void VMC_TargetToMotor(void);


#if smart == 0
/**
 * 步态控制任务
 * 流程：获取当前模式->[轨迹生成器]获得坐标->运动学逆解获得目标弧度
 * */
void Posture_Task(void *argument)
{

    while (1)
    {
        gait_params = state_gait_params[NowState];         // 提取步态参数表（通用）的参数
        RcDetachedParam = state_detached_params[NowState]; // 提取步态参数表（分离）的参数   //没用过

        if (Mode_Change_Flag)
        {
            Last_Time = HAL_GetTick() / 1000.0f; // 获取当前运行时间作为开始时间
            for (uint8_t i = 0; i < 4; i++)
            {
                Cycloid_Gen[i].last_time = Last_Time;
            }
        }
        for (uint8_t i = 0; i < 4; i++)
        {
            Cycloid_Gen[i].param = gait_params; // 步态参数
        }

        Ctrl_byRemoctrl(Cycloid_Gen, &my_rc_ctrl);

        switch (NowState) // 获取当前模式 更改2 NowState->TROT
        {
        // 停止并直立状态
        case STOP:
            for (uint8_t i = 0; i < 4; i++)
            { 
                aim_x[i] = 0.0f; // 此时马腿呈直立态
                aim_y[i] = 20.0f;
            }
            Target_toMotor();
						VMC_TargetToMotor();
            break;
        case CLIMBING:
            Calculate_Gait(CLIMBING);//生成xy的
            Target_toMotor();
						VMC_TargetToMotor();
            break;
        // 慢步
        case WALK:
            Calculate_Gait(WALK);
            Target_toMotor();
						VMC_TargetToMotor();
            break;
        case TROT:
            Calculate_Gait(TROT);
            Target_toMotor();
//						VMC_TargetToMotor();
            break;
        case RUN:
            Calculate_Gait(RUN);
            Target_toMotor();
						VMC_TargetToMotor();
        case JUMP_GROUND:
            osThreadResume(JumpHandle);
            break;
        case JUMP_UPHILL:
            osThreadResume(JumpHandle);
            break;
				case JUMP_UP:
            osThreadResume(JumpHandle);
            break;
				case JUMP_UP_SLOPE:
			osThreadResume(JumpHandle);
			break;				
				case JUMP_TEST:
						osThreadResume(JumpHandle);
            break;
        case MINCE:
            Calculate_Gait(MINCE);
            Target_toMotor();
            break;
				case CREEP:
						Calculate_Gait(CREEP);
            Target_toMotor();
						//VMC_TargetToMotor();
            break;
        case RELEASE:
				// 保持上一个动作0.5s
        default:
            osDelay(500);
            break;
        }
        /*绝对延时*/
        static portTickType xLastWakeTime;
        static const portTickType xFrequency = pdMS_TO_TICKS(6);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#else
/**
 * 自主-步态控制任务
 * 流程：获取当前模式->[轨迹生成器]获得坐标->运动学逆解获得目标弧度
 */
void Posture_Task(void *argument)
{
	while (1)
    {
        if (flag_Standup)
        {
					//uint8_t guangdian = HAL_GPIO_ReadPin(ON_OFF_GPIO_Port,ON_OFF_Pin);
					Buttom[0] = HAL_GPIO_ReadPin(Buttom_1_GPIO_Port,Buttom_1_Pin);
					Buttom[1] = HAL_GPIO_ReadPin(Buttom_2_GPIO_Port,Buttom_2_Pin);
					Buttom[2] = HAL_GPIO_ReadPin(Buttom_3_GPIO_Port,Buttom_3_Pin);
					Buttom[3] = HAL_GPIO_ReadPin(Buttom_4_GPIO_Port,Buttom_4_Pin);
					Buttom[4] = HAL_GPIO_ReadPin(Buttom_5_GPIO_Port,Buttom_5_Pin);
					Buttom[5] = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
					if (my_rc_ctrl.rc.ch[1]>1600 && my_rc_ctrl.rc.ch[3] > 1600)
					{
						Setdown();
						Change_NowState(STOP);
					}
					if (my_rc_ctrl.rc.ch[3]>1600) flag_go = 1;
					else if (my_rc_ctrl.rc.ch[3]<400) flag_go = 0;
					else if (my_rc_ctrl.rc.ch[1]>1600) Smart_State = TROT;
					else if (my_rc_ctrl.rc.ch[1]<400) Smart_State = WALK;
					else if (my_rc_ctrl.rc.ch[0]>1600)  Smart_State = CREEP;
					else if (my_rc_ctrl.rc.ch[0]<400)  Smart_State = MINCE;
					
					switch(vision_Data.vision_state)
					{
						case 0x00:
							Smart_State = STOP;
							break;
						case 0x01:
							Smart_State = TROT;
							break;
					}
					
					if (Buttom[1] == 1) flag_go = 1;
					else if (Buttom[0] == 1) flag_go = 0;
//					else if (Buttom[2] == 1) flag_go = 3;
//					else if (Buttom[3] == 1) flag_go = 4;
//					else if (Buttom[4] == 1) flag_go = 2;
//					if (Buttom[5] == 0) flag_go =0;//光电
            NowState = Smart_State;
            gait_params = state_gait_params[NowState];         // 提取步态参数表（通用）的参数

            if (Mode_Change_Flag)
            {
                Last_Time = HAL_GetTick() / 1000.0f; // 获取当前运行时间作为开始时间
                for (uint8_t i = 0; i < 4; i++)
                {
                    Cycloid_Gen[i].last_time = Last_Time;
                }
            }
            for (uint8_t i = 0; i < 4; i++)
            {
                Cycloid_Gen[i].param = gait_params; // 步态参数
            }
            if (smart_off == 0)
            {
							jy901_on=0;
//							ALL_AUTO_4(Cycloid_Gen);
//				  	ALL_AUTO_2(Cycloid_Gen); // 竞速
//						ALL_AUTO_3(Cycloid_Gen); // 障碍
							switch(flag_go)
							{
								case 0:
									ALL_AUTO_4(Cycloid_Gen); //竞速
									break;
								case 1:
									ALL_AUTO_4(Cycloid_Gen); //竞速
									break;
								case 2:
									//ALL_AUTO_5(Cycloid_Gen); //障碍
									break;
								case 3:
									ALL_AUTO_4(Cycloid_Gen); //暂定
									break;
								case 4:
									ALL_AUTO_4(Cycloid_Gen); //暂定
									break;
							}
						
            }
            else
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    aim_x[i] = 0.0f; // 此时马腿呈直立态
                    aim_y[i] = sqrt(pow(26.0f, 2.0f) - pow(15.0f, 2.0f));;
                }
                Target_toMotor();
            }
        }
        else
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                aim_x[i] = 0.0f; // 此时马腿呈直立态
                aim_y[i] = sqrt(pow(26.0f, 2.0f) - pow(15.0f, 2.0f));;
            }
            Target_toMotor();
            osDelay(10);
        }
				
        /*绝对延时*/
        static portTickType xLastWakeTime;
        static const portTickType xFrequency = pdMS_TO_TICKS(4);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
				
    }
}

void AUTO_RC26(Cycloid_Generator_t *pCycGenerato)
{
	static float switch_yaw = 0.0f;
	static float jy_start_time ;
	static float jy_now_time;
	static float error = 0.0f;
	float CH2_zheng;// = 0.066f;
	float CH2_fu;//    = -0.093f;
	if (vision_Data.x_error!=0 &&vision_Data.dis<45.0f&&vision_Data.dis>2.0f)
	{
		error = vision_Data.x_error ;// 10000.0f;
		CH2_zheng = 0.04f;  //0.04
		CH2_fu = -0.06f;     //-0.06
		//error = vision_Data.x_error;
	}
    else{
			CH2_zheng = 0.02f;  //0.057
			CH2_fu = -0.07;//0.093
        error = 
        (Now_Angle.yaw-switch_yaw);
    }
	
	
	if (flag_go == 1)
	{
		jy_now_time = HAL_GetTick() / 1000.0f - jy_start_time ;
		AUTO_JY901s_advance(pCycGenerato,Smart_State,0.8f,error,CH2_zheng,CH2_fu); // 0.8改成发的前进后退速度，正前进负后退  zheng左fu右 后面改一下
	}
	
	//	else if (flag_go == 2)
//		AUTO_JY901s_advance(pCycGenerato,Smart_State,1.0f,(Now_Angle.yaw-switch_yaw));
//	else if (flag_go == 3)
//		AUTO_JY901s_advance(pCycGenerato,Smart_State,1.2f,(Now_Angle.yaw-switch_yaw));
	
	else
	{
		jy_start_time = HAL_GetTick() / 1000.0f;
		switch_yaw = Now_Angle.yaw;
		AUTO_JY901s_advance(pCycGenerato,STOP,0.0f,0.0f,0.0f,0.0f);
	}
}




void ALL_AUTO_4(Cycloid_Generator_t *pCycGenerato)
{
	static float switch_yaw = 0.0f;
	static float jy_start_time ;
	static float jy_now_time;
	float one_phase   = 0.8f;
	float two_phase   = 0.8f;
	float three_phase = 0.8f;
	float four_phase  = 0.8f;
	float five_phase  = 0.8f;
	float six_phase   = 0.8f;
	float seven_phase = 0.8f;
	static float error = 0.0f;
		//vision_Data.x_error;// + (Now_Angle.yaw-switch_yaw)/2;
	float CH2_zheng;// = 0.066f;
	float CH2_fu;//    = -0.093f;
	//(Now_Angle.yaw-switch_yaw);
	//error = 0.0f;
//        //(Now_Angle.yaw-switch_yaw)/2;
//		vision_Data.x_error / 10000.0f;
	if (vision_Data.x_error!=0 &&vision_Data.dis<45.0f&&vision_Data.dis>2.0f)
	{
		error = vision_Data.x_error ;// 10000.0f;
		CH2_zheng = 0.04f;  //0.04
		CH2_fu = -0.06f;     //-0.06
		//error = vision_Data.x_error;
	}
    else{
			CH2_zheng = 0.02f;  //0.057
			CH2_fu = -0.07;//0.093
        error = 
        (Now_Angle.yaw-switch_yaw);
    }
	
	
	if (flag_go == 1)
	{
		jy_now_time = HAL_GetTick() / 1000.0f - jy_start_time ;
		if (jy_now_time < one_phase)
			AUTO_JY901s_advance(pCycGenerato,Smart_State,0.3f,error,CH2_zheng,CH2_fu); //1
		else if (jy_now_time < one_phase + two_phase)
			AUTO_JY901s_advance(pCycGenerato,Smart_State,0.4f,error,CH2_zheng,CH2_fu); //2
		else if (jy_now_time < one_phase + two_phase + three_phase)
			AUTO_JY901s_advance(pCycGenerato,Smart_State,0.5f,error,CH2_zheng,CH2_fu); //3
		else if (jy_now_time < one_phase + two_phase + three_phase + four_phase)
			AUTO_JY901s_advance(pCycGenerato,Smart_State,0.6f,error,CH2_zheng,CH2_fu); //4
		else if (jy_now_time < one_phase + two_phase + three_phase + four_phase + five_phase)
			AUTO_JY901s_advance(pCycGenerato,Smart_State,0.7f,error,CH2_zheng,CH2_fu); //5
//		else if (jy_now_time < one_phase + two_phase + three_phase + four_phase + five_phase + six_phase)
//			AUTO_JY901s_advance(pCycGenerato,Smart_State,0.8f,(Now_Angle.yaw-switch_yaw)); //6
//		else if (jy_now_time < one_phase + two_phase + three_phase + four_phase + five_phase + six_phase + seven_phase)
//			AUTO_JY901s_advance(pCycGenerato,Smart_State,0.9f,(Now_Angle.yaw-switch_yaw)); //7
		
		else
			AUTO_JY901s_advance(pCycGenerato,Smart_State,0.8f,error,CH2_zheng,CH2_fu); //8
	}
	
	//	else if (flag_go == 2)
//		AUTO_JY901s_advance(pCycGenerato,Smart_State,1.0f,(Now_Angle.yaw-switch_yaw));
//	else if (flag_go == 3)
//		AUTO_JY901s_advance(pCycGenerato,Smart_State,1.2f,(Now_Angle.yaw-switch_yaw));
	
	else
	{
		jy_start_time = HAL_GetTick() / 1000.0f;
		switch_yaw = Now_Angle.yaw;
		AUTO_JY901s_advance(pCycGenerato,STOP,0.0f,0.0f,0.0f,0.0f);
	}
}


//void ALL_AUTO_5(Cycloid_Generator_t *pCycGenerato) //2025障碍
//{
//	switch(vision_Data.action)
//	{
//		case 1: //竖杆
//			AUTO_vision_advance(pCycGenerato,Smart_State,0.5f,vision_Data.x_error);
//			break;
//		case 2:  //竖杆-匍匐
//			if (vision_Data.dis>2.0f)
//			{
//				AUTO_vision_advance(pCycGenerato,Smart_State,0.5f,vision_Data.x_error);
//			}
//			else if (1.0f < vision_Data.dis && vision_Data.dis < 2.0f)
//			{
//				CHANGE_HIGH_AUTO(CREEP,3,&creep_flag);
//				osDelay(200);
//				CHANGE_HIGH_AUTO(CREEP,4,&creep_flag);
//				Smart_State = CREEP;
//				if (creep_flag!=0)
//					AUTO_JY901s_advance(pCycGenerato,Smart_State,0.5f,Now_Angle.yaw);
//			}
//			else  
//			{
//				AUTO_JY901s_advance(pCycGenerato,Smart_State,0.5f,Now_Angle.yaw);
//			}
//			break;
//		case 3: //匍匐-沙坑
//			if (vision_Data.dis > 2.0f) //还在匍匐
//			{
//				AUTO_JY901s_advance(pCycGenerato,Smart_State,0.5f,Now_Angle.yaw);
//			}
//			else if (1.0f < vision_Data.dis && vision_Data.dis < 2.0f)
//			{
//				CHANGE_HIGH_AUTO(NORMAL,1,&creep_flag);
//				osDelay(200);
//				CHANGE_HIGH_AUTO(NORMAL,2,&creep_flag);
//				if (creep_flag == 0)
//				{
//					Smart_State = WALK;
//					AUTO_JY901s_advance(pCycGenerato,Smart_State,0.5f,Now_Angle.yaw);
//				}					
//			}
//			else //到沙坑 <=1
//				{
//					if (flag_jump == 0)
//					{
//						jy901_on = 0;
//						NowState = JUMP_UPHILL;
//						Jump_OnlyOnce();
//						flag_jump = 1;
//					}
//					else
//					{
//						AUTO_JY901s_advance(pCycGenerato,Smart_State,0.5f,Now_Angle.yaw);//可以试试误差用x_error
//					}

//				}
//			break;
//		case 4: //沙坑-高墙
//			if (vision_Data.dis > 2.0f) //沙坑
//			{
//				AUTO_JY901s_advance(pCycGenerato,Smart_State,0.5f,Now_Angle.yaw);
//			}
//			else if (1.0f < vision_Data.dis && vision_Data.dis < 2.0f) //距离暂定 跳出沙坑
//			{
//				
//			}
//			break;
//			
//	}
//}

void ALL_AUTO_2(Cycloid_Generator_t *pCycGenerato)
{
	if (turn_flag == 0)
    {
				if(go_back_flag == 0 &&HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) == 1) // go_back_flag为之前的数字标识
        {
//			if(vision_Data.x_error < -7.0f || vision_Data.x_error > 7.0f)
////                AUTO_turn_PID(pCycGenerato, Smart_State, vision_Data.x_error);
//				AUTO_turn(pCycGenerato, Smart_State, vision_Data.x_error > 0.0f ? 0.5f: -0.5f);
//            else
                AUTO_advance(pCycGenerato, Smart_State, 1.2f, vision_Data.x_error);
        }
        else if(go_back_flag == 0 && HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) == 0)
        {
            xuanzhuan_flag = 1;
            go_back_flag = 1;
        }
        else if(xuanzhuan_flag == 1 && Now_Angle.yaw < 125.0f)//第一个转弯
        {
					
            AUTO_turn(pCycGenerato, Smart_State, -1.0f);
        }
        else if(xuanzhuan_flag == 1 && Now_Angle.yaw >= 125.0f)
        {
					  Laser1.temp = 8000.0f;
            xuanzhuan_flag = 2;
            go_back_flag = 2;
            turn_flag = 8;
        }
        else if(go_back_flag == 2 && HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) == 1)//直行
        {
//			if(vision_Data.x_error < -7.0f || vision_Data.x_error > 7.0f)
////                AUTO_turn_PID(pCycGenerato, Smart_State, vision_Data.x_error);
//				AUTO_turn(pCycGenerato, Smart_State, vision_Data.x_error > 0.0f ? 0.5f: -0.5f);
//            else
                AUTO_advance(pCycGenerato, Smart_State, 1.2f, vision_Data.x_error);
        }
        else if(go_back_flag == 2 && HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) == 0)
        {
            xuanzhuan_flag = 3;
            go_back_flag = 3;
        }
        else if((xuanzhuan_flag == 3 && Now_Angle.yaw < -105.0f) || (xuanzhuan_flag == 3 && Now_Angle.yaw > -100.0f))//第二个转弯
        {
            AUTO_turn(pCycGenerato, Smart_State, -1.0f);
        }
        else if(xuanzhuan_flag == 3 && Now_Angle.yaw >= -105.0f && Now_Angle.yaw <= -100.0f)
        {
					Laser1.temp = 8000.0f;
            xuanzhuan_flag = 4;
            go_back_flag = 4;
            turn_flag = 8;
        }
        else if(go_back_flag == 4 &&  HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) == 1)//第3个直行
        {
//			if(vision_Data.x_error < -7.0f || vision_Data.x_error > 7.0f)
////                AUTO_turn_PID(pCycGenerato, Smart_State, vision_Data.x_error);
//				AUTO_turn(pCycGenerato, Smart_State, vision_Data.x_error > 0.0f ? 0.5f: -0.5f);
//            else
                AUTO_advance(pCycGenerato, Smart_State, 1.2f, vision_Data.x_error);
        }
        else if(go_back_flag == 4 && HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) == 0)
        {
            xuanzhuan_flag = 5;
            go_back_flag = 5;
        }
        else if((xuanzhuan_flag == 5 && Now_Angle.yaw > 135.0f)||(xuanzhuan_flag == 5 && Now_Angle.yaw < 125.0f))//第三个转弯
        {
            AUTO_turn(pCycGenerato, Smart_State, 1.0f);
//			AUTO_turn_PID(pCycGenerato, Smart_State, 1.0f);
        }
        else if(xuanzhuan_flag == 5 && Now_Angle.yaw <= 135.0f && Now_Angle.yaw >= 125.0f)
        {
            xuanzhuan_flag = 6;
            go_back_flag = 6;
            turn_flag = 8;
        }
        else if(go_back_flag == 6 && HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) == 1)
        {
//            if(vision_Data.x_error < -7.0f || vision_Data.x_error > 7.0f)
////                AUTO_turn_PID(pCycGenerato, Smart_State, vision_Data.x_error);
//				AUTO_turn(pCycGenerato, Smart_State, vision_Data.x_error > 0.0f ? 0.3f: -0.3f);
//            else
                AUTO_advance(pCycGenerato, Smart_State, 1.2f, vision_Data.x_error);
        }
        else if(go_back_flag == 6 &&  HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) == 0)
        {
            turn_flag = 8;
            go_back_flag = 7;
        }
    }
    else
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            aim_x[i] = 0.0f; // 此时马腿呈直立态
            aim_y[i] = sqrt(pow(27.0f, 2.0f) - pow(15.0f, 2.0f));
        }
        Target_toMotor();
        osDelay(300);
        if(go_back_flag != 7)
            turn_flag = 0;
        else
            turn_flag = 8;
    }
}

void ALL_AUTO_3(Cycloid_Generator_t *pCycGenerato) // 障碍
{
	if (turn_flag == 0)
    {
		switch(vision_Data.action)
		{
		case 3://爬坡
			jy901_on=1;
			if(go_back_flag == 0 && Laser1.temp > 400)
			{
				AUTO_turn_PID(pCycGenerato, Smart_State, 1.0f, vision_Data.x_error);
			}
			else if(go_back_flag == 0 && Laser1.temp <= 400)
			{
				turn_flag = 1;
				now_state = 3;
				jy901_on=1;
			}
			break;
		case 1://双木桥
			jy901_on=0;
			if(go_back_flag == 0 && Laser1.temp > 6550)//直行
			{
				AUTO_turn_PID(pCycGenerato, Smart_State, 1.0f, vision_Data.x_error);
			}
			else if(go_back_flag == 0 && jump_flag == 0 && Laser1.temp <= 6550)
			{
				jump_flag = 1;
				go_back_flag = 1;
				jy901_on=1;
			}
			else if(jump_flag == 1)
			{
				start_time_jump = HAL_GetTick();
				NowState = JUMP_GROUND;
				Jump_OnlyOnce();
				jump_flag = 2;
				go_back_flag = 2;
				jy901_on=0;
			}
			else if(go_back_flag == 2)//直行
			{
				if(vision_Data.x_error <= -0.5f || vision_Data.x_error >= 0.5f)
					AUTO_turn_PID(pCycGenerato, Smart_State, 1.0f, vision_Data.x_error);
				else
				{
					jump_flag = 3;
					go_back_flag = 3;
				}
			}
			else if(jump_flag == 3)
			{
				start_time_jump = HAL_GetTick();
				NowState = JUMP_GROUND;
				Jump_OnlyOnce();
				jump_flag = 4;
				go_back_flag = 4;
			}
			else if(go_back_flag == 4 && Laser1.temp > 400)//直行
			{
				AUTO_turn_PID(pCycGenerato, Smart_State, 1.0f, vision_Data.x_error);
			}
			else if(go_back_flag == 4 && Laser1.temp <= 400)//直行
			{
				turn_flag = 1;
				now_state = 1;
				
				go_back_flag = 0;
				jump_flag = 0;
			}
			break;
		case 4://阶梯
			if(go_back_flag == 0 && Laser1.temp > 760)//直行
			{
				AUTO_turn_PID(pCycGenerato, Smart_State, 1.0f, vision_Data.x_error);
			}
			else if(go_back_flag == 0 && Laser1.temp < 760)
			{
				go_back_flag = 1;
				jump_flag = 1;
			}
			else if(jump_flag == 1)	// 一次跳
			{
				start_time_jump = HAL_GetTick();
				NowState = JUMP_GROUND;
				Jump_OnlyOnce();
				jump_flag = 2;
				go_back_flag = 2;
			}
			else if(go_back_flag == 2)
			{
				if(vision_Data.x_error <= -0.5f || vision_Data.x_error >= 0.5f)
					AUTO_turn_PID(pCycGenerato, Smart_State, 1.0f, vision_Data.x_error);
				else
				{
					jump_flag = 3;
					go_back_flag = 3;
				}
			}
			else if(jump_flag == 3) // 二次跳
			{
				start_time_jump = HAL_GetTick();
				NowState = JUMP_GROUND;
				Jump_OnlyOnce();
				jump_flag = 4;
				go_back_flag = 4;
			}
			else if(go_back_flag == 4)
			{
				if(vision_Data.x_error <= -0.5f || vision_Data.x_error >= 0.5f)
					AUTO_turn_PID(pCycGenerato, Smart_State, 1.0f, vision_Data.x_error);
				else
				{
					jump_flag = 5;
					go_back_flag = 5;
				}
			}
			else if(jump_flag == 5) // 三次跳
			{
				start_time_jump = HAL_GetTick();
				NowState = JUMP_GROUND;
				Jump_OnlyOnce();
				jump_flag = 6;
				go_back_flag = 6;
			}
			else if(go_back_flag == 6) 
			{
				if(vision_Data.x_error <= -0.5f || vision_Data.x_error >= 0.5f)
					AUTO_turn_PID(pCycGenerato, Smart_State, 1.0f, vision_Data.x_error);
				else
				{
					jump_flag = 7;
					go_back_flag = 7;
				}
			}
			else if(jump_flag == 7) // 四次跳
			{
				start_time_jump = HAL_GetTick();
				NowState = JUMP_GROUND;
				Jump_OnlyOnce();
				jump_flag = 8;
				go_back_flag = 8;
			}
			else if(go_back_flag == 8 && Laser1.temp > 400)
			{
				AUTO_turn_PID(pCycGenerato, Smart_State, 1.0f, vision_Data.x_error);
			}
			else if(go_back_flag == 8 && Laser1.temp <= 400)
			{
				turn_flag = 1;
				now_state = 2;
				go_back_flag = 0;
				jump_flag = 0;
			}
			break;
		case 2://高栏
			if(go_back_flag == 0 && Laser1.temp >= 300.0f) //直行
			{
				AUTO_turn_PID(pCycGenerato, Smart_State, 0.6f, vision_Data.x_error);
			}
			else if(go_back_flag == 0 && Laser1.temp < 300.0f)
			{
				go_back_flag = 1;
				high_jump_flag = 1;
			}
			else if(high_jump_flag == 1)	// 大跳
			{
				start_time_jump = HAL_GetTick();
				NowState = JUMP_UPHILL;
				Jump_OnlyOnce();
				high_jump_flag = 2;
				go_back_flag = 2;
			}
			else if(go_back_flag == 2 && Laser1.temp > 200.0f) //直行
			{
				AUTO_turn_PID(pCycGenerato, Smart_State, 1.0f, vision_Data.x_error);
			}
			else if(go_back_flag == 2 && Laser1.temp < 200.0f)
			{
				turn_flag = 1;
				now_state = 4;
				end_t = 1;
			}
			break;
		}
    }
	else if (turn_flag == 1)
	{
		if(Now_Angle.yaw >= -90.0f && now_state == 1)//第一个转弯
		{
			AUTO_turn(pCycGenerato, Smart_State, 1.0f);
		}
		else if(Now_Angle.yaw <= -90.0f && now_state == 1)
		{
			turn_flag = 8;
		}
		else if(Now_Angle.yaw <= 177.5f && now_state == 2)//第二个转弯
		{
			AUTO_turn(pCycGenerato, Smart_State, 1.0f);
		}
		else if(Now_Angle.yaw > 177.5f && now_state == 2)
		{
			turn_flag = 8;
		}
		else if(88.0f < Now_Angle.yaw && Now_Angle.yaw < 92.0f && now_state == 3)
		{
			turn_flag = 8;
		}
		else if(now_state == 3)//第三个转弯
		{
			AUTO_turn(pCycGenerato, Smart_State, 1.0f);
		}
	}
    else
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            aim_x[i] = 0.0f; // 此时马腿呈直立态
            aim_y[i] = sqrt(pow(27.0f, 2.0f) - pow(15.0f, 2.0f));
        }
        Target_toMotor();
		if(end_t != 1)
			turn_flag = 0;
    }
}




#endif

/**
 * @brief 切换步态
 * @param target 目标步态
 * @return void
 */
void Change_NowState(Robohorse_State target)
{
    if (flag_Standup)
    {
        if (NowState != target)
        {
            taskENTER_CRITICAL();
            NowState = target;
            Mode_Change_Flag = 1;
            taskEXIT_CRITICAL();
        }
    }
    else
        return;
}
/**
 * @brief 步态参数计算
 * @param state
 * @return void
 */
void Calculate_Gait(Robohorse_State state)
{
    now_time = HAL_GetTick() / 1000.0f; // 获取当前时间用于后续运算
    Cycloid_Generator_Solution_t cyc_solution[4];
    switch (state)
    {
        // FIXME:Cycloid_Generator()第三个参数不能为0 否则切换步态会导致一条腿残废
    case WALK:
        if (flag_imu_line == 1)
        {
            Cycloid_Gen[0].param.step_length -= step_len_dev;   //step_len_dev=0;
            Cycloid_Gen[1].param.step_length -= step_len_dev;
            Cycloid_Gen[2].param.step_length += step_len_dev;
            Cycloid_Gen[3].param.step_length += step_len_dev;
            LIMIT_PARAM(Cycloid_Gen[0].param.step_length, 16);  //限制_参数
            LIMIT_PARAM(Cycloid_Gen[1].param.step_length, 16);
            LIMIT_PARAM(Cycloid_Gen[2].param.step_length, 16);
            LIMIT_PARAM(Cycloid_Gen[3].param.step_length, 16);
        }
        // 轨迹生成器运算
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.01f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.51f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    case CLIMBING:
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.01f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.51f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;
    case TROT:
    case RUN:
        if (flag_imu_line == 1)
        {
            Cycloid_Gen[0].param.step_length -= step_len_dev;
            Cycloid_Gen[1].param.step_length -= step_len_dev;
            Cycloid_Gen[2].param.step_length += step_len_dev;
            Cycloid_Gen[3].param.step_length += step_len_dev;
            LIMIT_PARAM(Cycloid_Gen[0].param.step_length, 16);
            LIMIT_PARAM(Cycloid_Gen[1].param.step_length, 16);
            LIMIT_PARAM(Cycloid_Gen[2].param.step_length, 16);
            LIMIT_PARAM(Cycloid_Gen[3].param.step_length, 16);
        }
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.01f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.51f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    case MINCE:
//        if (flag_imu_line == 1)
//        {
//            Cycloid_Gen[0].param.step_length -= step_len_dev;
//            Cycloid_Gen[1].param.step_length -= step_len_dev;
//            Cycloid_Gen[2].param.step_length += step_len_dev;
//            Cycloid_Gen[3].param.step_length += step_len_dev;
//            LIMIT_PARAM(Cycloid_Gen[0].param.step_length, 16);
//            LIMIT_PARAM(Cycloid_Gen[1].param.step_length, 16);
//            LIMIT_PARAM(Cycloid_Gen[2].param.step_length, 16);
//            LIMIT_PARAM(Cycloid_Gen[3].param.step_length, 16);
//        }
        // 轨迹生成器运算
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.01f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.51f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;
		case CREEP:
				Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.01f, &cyc_solution[0]);//1时间 2参与计算的值  3判断是支撑相还是摆动相  4计算的结果 通过while1 来不断计算角度
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.51f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
    default:
        break;
    }

    // 装填结果值 弧度rad
    for (uint8_t i = 0; i < 4; i++)
    {
        aim_x[i] = cyc_solution[i].x;
        aim_y[i] = cyc_solution[i].y;
    }
}

/**
 * @brief 目标坐标转化角度并输出到电机   nijie
 *
 */
void Target_toMotor()
{
		if(flag_Standup == 1)
		{
			for (uint8_t i = 0; i < 4; i++)
			{
        if (i == 0 || i == 1)
        {
            Stanford_Type_Lite_SetPosition(&Gait_Data[i], aim_x[i], aim_y[i]); // 运动学解算待数据填充(数据填充)
            Stanford_Type_Lite_Inverse_Kinematics(&Gait_Data[i], 1);           // 运动学逆解算 坐标转弧度  左侧腿
        }
        else
        {
            Stanford_Type_Lite_SetPosition(&Gait_Data[i], aim_x[i], aim_y[i]); // 运动学解算待数据填充
            Stanford_Type_Lite_Inverse_Kinematics(&Gait_Data[i], 0);           // 运动学逆解算 坐标转弧度
        }
        Aim_Angle[i * 2] = -(pi / 2.0f - Gait_Data[i].angle[1]) / 1.0f; //-顺时针,弧度
        Aim_Angle[i * 2 + 1] = -(pi / 2.0f + Gait_Data[i].angle[0]) / 1.0f;
    }
    Is_Aim_Angle_Get = 1; // 数据填充完毕
		}
		else
		{
			for (uint8_t i = 0; i < 4; i++){
				Aim_Angle[i * 2] = 0.0f; //-顺时针,弧度
				Aim_Angle[i * 2 + 1] = 0.0f;
			}
		Is_Aim_Angle_Get = 1;
		}
    
}

//?有啥用
void VMC_TargetToMotor(void){
//	float F
	get_angle(&C_polar,10.0f,10.0f);
	
} 

