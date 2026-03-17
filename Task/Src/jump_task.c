#include "jump_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gait_param.h"
#include "posture_task.h"
#include "GO-MOTOR.h"
#include "VMC.h"
#include "Ahrs_Task.h"
#include "Standup_Task.h"

extern float start_time_jump;
extern polarCoordinates_C C_polar;
extern osThreadId_t JumpHandle;
extern PID_t PID_Speed[8]; // 速度PID
extern PID_t PID_Position[8];
extern MOTOR_send Motor_T[8]; // 以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
extern MOTOR_recv Motor_R[8];

float start_time_jump = 0.0f;

void Jump_OnlyOnce(void);


void Jump_Task(void *argument)
{
    while (1)
    {

        static uint8_t flag_enterTwice = 0;
        /* 防止上电时运行任务 */
        if (flag_enterTwice == 0)
        { /* 有两个jump任务调用Jump_Task */
            flag_enterTwice++;
        }
        else
        {
            start_time_jump = HAL_GetTick(); // 跳跃开始的时间
            Jump_OnlyOnce();
        }
        osThreadSuspend(JumpHandle);
    }
	
}

/**
 * @brief 跳跃一次，跳跃完成后会将NowState切换为STOP
 * @return void
 */
void Jump_OnlyOnce()
{
    const float prep_time = 0.8f;   // 准备时间 [s]		0.8
    const float launch_time = 0.2f; // 收缩腿前的持续时间 [s]		0.2
    const float fall_time = 0.3f;   // 降落时的减速时间 [s]		0.8
    const float stabilize_time = 0.5f;

    float t = 0;
  
       /*************************************阶梯*************************************/
    if (NowState == JUMP_GROUND)
    {
        while (t <= prep_time + launch_time + fall_time + stabilize_time)
        {
            /* 极坐标 */
            Polar_Coord_Data_t Action_Polar[4] = {
                /*  r   θ */
                17.0, -30.0f,
                17.0, -30.0f,
                17.0, -30.0f,
                17.0, -30.0f};

            t = HAL_GetTick() / 1000.0f - start_time_jump / 1000.0f; // 跳跃开始后的时间
            /* 跳跃储能阶段 */
            if (t < prep_time)
            {
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 3.50f;
                    Motor_T[i].K_W = 0.14f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃上升阶段 */
            else if (t >= prep_time && t < prep_time + launch_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    C_polar.L = Action_Polar[i].radius = 35.0f;//35
                    C_polar.zeta = Action_Polar[i].theta = -30.0f;
                }

                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 3.8f;
                    Motor_T[i].K_W = 0.12f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃减速阶段 */
            else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    C_polar.L = Action_Polar[i].radius = 16.0f;
                    C_polar.zeta = Action_Polar[i].theta = -15.0f;//10
                }

                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 0.5f;
                    Motor_T[i].K_W = 0.12f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃稳定阶段 */
            else if (t > prep_time + launch_time + fall_time && t < prep_time + launch_time + fall_time + stabilize_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    C_polar.L = Action_Polar[i].radius = 16.0f;
                    C_polar.zeta = Action_Polar[i].theta = 0.0f;
                }

                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 1.0f;
                    Motor_T[i].K_W = 0.09f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃结束 */
            else if (t > prep_time + launch_time + fall_time + stabilize_time) // 12000
            {
 

                Change_NowState(STOP);

							  for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 1.7f;
                    Motor_T[i].K_W = 0.12f;
                }
							
            }
        }
					osDelay(200);
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 1.55f;
                    Motor_T[i].K_W = 0.023f;
                }
    }
//		
   /*************************************高栏（遥控器往上）*************************************/
    else if (NowState == JUMP_UPHILL)
    {
				const float prep_time = 1.5f;   // 准备时间 [s]		0.8之后跳，prep英文准备	
				const float launch_time = 0.32f; // 收缩腿前的持续时间 == 腿伸直前需要时间 [s]		0.2   0.15	0.2
				const float tucked_legs_t= 0.16f;	//收前腿后腿时间0.3
				const float fall_time = 0.25f;   // 降落时的减速时间 == 收腿需要时间  [s]		0.4  0.35 0.9
				const float stabilize_time = 0.9f;//稳定时间or结束时间0.5
        while (t <= prep_time + launch_time + fall_time + stabilize_time + tucked_legs_t)
        {
            /* 极坐标 */
            Polar_Coord_Data_t Action_Polar[4] ={
                /*  r   θ */
                15.0, -20.0f,//半径min13,normal=-29.0f		23  27  25
                15.0, -33.0f,
                15.0, -20.0f,//-20
                15.0, -33.0f
			};

            t = HAL_GetTick() / 1000.0f - start_time_jump / 1000.0f; // 跳跃开始后的时间
			//前者毫秒转化为 /1000.0f 将两者转换为秒单位，然后相减得到从跳跃开始后经过的时间 t
            /* 跳跃储能阶段 */
            if (t < prep_time)
            {
                // PID_Reset(PID_Position, 5.0f, 0.0008f, 0.0f);
                // PID_Reset(PID_Speed, 8.0f, 0.0001f, 0.0f);
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 1.0f;//3.8
                    Motor_T[i].K_W = 0.10f;//0.2
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃上升阶段 */
            else if (t >= prep_time && t < prep_time + 0.75f*launch_time)
            {

                for (uint8_t i = 0; i < 4; i++)
                {
//                    Action_Polar[i].radius = 41.5f;  //41.5f
//                    Action_Polar[i].theta = -34.0f;//-33.0f
					if (i==1 || i==3) //1 3是后腿
					{
						Action_Polar[i].radius = 41.5f;
						Action_Polar[i].theta = -25.0f;//34	23	40  25 27  -25
					}
					else
					{
						Action_Polar[i].radius = 41.5f;
						Action_Polar[i].theta =-35.0f;//-35
					}
                }

                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 10.4f;  //5.4  10.0f
                    Motor_T[i].K_W = 0.15f; //0.13
                }

                Polar_setCoord(Action_Polar);
			
            }
			/* 空中收前腿后腿阶段 */
			else if (t >= prep_time + 0.75f*launch_time && t < prep_time + launch_time + tucked_legs_t)
			{
				for (uint8_t i = 0; i < 4; i++)
                {
//                    Action_Polar[i].radius = 41.5f;  //41.5f
//                    Action_Polar[i].theta = -34.0f;//-33.0f
					if (i==1 || i==3) //1 3是后腿
					{
						Action_Polar[i].radius = 25.5f;// 16.5f  25.5
						Action_Polar[i].theta = 40.0f;//34	23	40  25   35
					}
					else
					{
						Action_Polar[i].radius = 20.0f;	//23	17  15
						Action_Polar[i].theta = -40.0f;//-10.f	20	17   -20
					}
				}

                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 10.0f;  //5.4
                    Motor_T[i].K_W = 0.10f; //0.13
                }

                Polar_setCoord(Action_Polar);
			}
            /* 跳跃减速阶段 */
            else if (t >= prep_time + launch_time + tucked_legs_t&& t < prep_time + launch_time + fall_time + tucked_legs_t)
            {	

                for (uint8_t i = 0; i < 4; i++)
                {	if (i==1 || i==3)
					{ //1 3是后腿
						Action_Polar[i].radius = 30.0f;	//23	17 25 33
						Action_Polar[i].theta = 25.0f;//-10.f	20	17 aaa -5
					}else{
					
						Action_Polar[i].radius = 27.0f;	//23	17 25 33
						Action_Polar[i].theta = 35.0f;//-10.f	20	17 aaa -5
					}
                }


                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 0.25f;
										Motor_T[i].K_W = 0.013f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃稳定阶段 */
            else if (t > prep_time + launch_time + fall_time + tucked_legs_t
					&& t < prep_time + launch_time + fall_time + stabilize_time+ tucked_legs_t)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
//                    Action_Polar[i].radius = 17.0f;//15.0f
//                    Action_Polar[i].theta = -5.0f;//-5.0f 20		-2
					if (i==1 || i==3) //1 3是后腿
					{
						Action_Polar[i].radius = 25.5f;// 16.5f
						Action_Polar[i].theta = 10.0f;//34	23	40  25   35
					}
					else
					{
						Action_Polar[i].radius = 25.0f;	//23	17  15 17
						Action_Polar[i].theta = 10.0f;//-10.f	20	17   -20  5
					}
                }

                for (int i = 0; i < 8; i++)
                {
					Motor_T[i].K_P = 0.3f;
					Motor_T[i].K_W = 0.023f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃结束 */
            else if (t > prep_time + launch_time + fall_time + stabilize_time+tucked_legs_t)
            {


                Change_NowState(STOP);
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 1.0f;
                    Motor_T[i].K_W = 0.05f;
                }
            }
						
        }
				osDelay(200);
				CHANGE_KPW(1.55f,0.023f);

	}
/*************************************斜坡大跳(JUMP_UP_slope)*************************************/
    else if (NowState == JUMP_UP_SLOPE)
    {
				const float prep_time = 1.5f;   // 准备时间 [s]		0.8之后跳，prep英文准备	
				const float launch_time = 0.2f; // 收缩腿前的持续时间 == 腿伸直前需要时间 [s]		0.2   0.15	0.2
				const float tucked_legs_t= 0.3;	//收前腿后腿时间
				const float fall_time = 0.9f;   // 降落时的减速时间 == 收腿需要时间  [s]		0.4  0.35 0.9
				const float stabilize_time = 0.5f;//稳定时间or结束时间0.5
        while (t <= prep_time + launch_time + fall_time + stabilize_time + tucked_legs_t)
        {
            /* 极坐标 */
            Polar_Coord_Data_t Action_Polar[4] ={
                /*  r   θ */
                13.0, -30.0f,//半径min13,normal=-29.0f		23  27  25
                13.0, -25.0f,
                13.0, -30.0f,
                13.0, -25.0f
			};

            t = HAL_GetTick() / 1000.0f - start_time_jump / 1000.0f; // 跳跃开始后的时间
			//前者毫秒转化为 /1000.0f 将两者转换为秒单位，然后相减得到从跳跃开始后经过的时间 t
            /* 跳跃储能阶段 */
            if (t < prep_time)
            {
                // PID_Reset(PID_Position, 5.0f, 0.0008f, 0.0f);
                // PID_Reset(PID_Speed, 8.0f, 0.0001f, 0.0f);
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 1.0f;//3.8
                    Motor_T[i].K_W = 0.10f;//0.2
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃上升阶段 */
            else if (t >= prep_time && t < prep_time + launch_time)
            {

                for (uint8_t i = 0; i < 4; i++)
                {
//                    Action_Polar[i].radius = 41.5f;  //41.5f
//                    Action_Polar[i].theta = -34.0f;//-33.0f
					if (i==1 || i==3) //1 3是后腿
					{
						Action_Polar[i].radius = 41.5f;
						Action_Polar[i].theta = -25.0f;//34	23	40  25 27  -25
					}
					else
					{
						Action_Polar[i].radius = 41.5f;
						Action_Polar[i].theta =-35.0f;//-35
					}
                }

                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 6.4f;  //5.4
                    Motor_T[i].K_W = 0.10f; //0.13
                }

                Polar_setCoord(Action_Polar);
			
            }
			/* 空中收前腿后腿阶段 */
			else if (t >= prep_time + launch_time && t < prep_time + launch_time + tucked_legs_t)
			{
				for (uint8_t i = 0; i < 4; i++)
                {
//                    Action_Polar[i].radius = 41.5f;  //41.5f
//                    Action_Polar[i].theta = -34.0f;//-33.0f
					if (i==1 || i==3) //1 3是后腿
					{
						Action_Polar[i].radius = 16.5f;
						Action_Polar[i].theta = 35.0f;//34	23	40  25  
					}
					else
					{
						Action_Polar[i].radius = 15.0f;	//23	17
						Action_Polar[i].theta = -20.0f;//-10.f	20	17
					}
				}

                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 6.4f;  //5.4
                    Motor_T[i].K_W = 0.10f; //0.13
                }

                Polar_setCoord(Action_Polar);
			}
            /* 跳跃减速阶段 */
            else if (t >= prep_time + launch_time + tucked_legs_t&& t < prep_time + launch_time + fall_time + tucked_legs_t)
            {	

                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 16.0f;	//23	17
                    Action_Polar[i].theta = -5.0f;//-10.f	20	17
                }


                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 0.25f;
					Motor_T[i].K_W = 0.013f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃稳定阶段 */
            else if (t > prep_time + launch_time + fall_time + tucked_legs_t
					&& t < prep_time + launch_time + fall_time + stabilize_time+ tucked_legs_t)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 17.0f;//15.0f
                    Action_Polar[i].theta = -2.0f;//-5.0f 20
                }

                for (int i = 0; i < 8; i++)
                {
					Motor_T[i].K_P = 0.3f;
					Motor_T[i].K_W = 0.023f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃结束 */
            else if (t > prep_time + launch_time + fall_time + stabilize_time+tucked_legs_t)
            {


                Change_NowState(STOP);
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 1.0f;
                    Motor_T[i].K_W = 0.05f;
                }
            }
						
        }
				osDelay(200);
				CHANGE_KPW(1.55f,0.023f);

}		
		    /*************************************双木桥（遥控器往上）*************************************/
    else if (NowState == JUMP_UP)
    {
			   const float prep_time = 0.8f;   // 准备时间 [s]		0.8
				const float launch_time = 0.2f; // 收缩腿前的持续时间 [s]		0.2
				const float fall_time = 0.3f;   // 降落时的减速时间 [s]		0.8
				const float stabilize_time = 0.5f;

				float t = 0;
        while (t <= prep_time + launch_time + fall_time + stabilize_time)
        {
            /* 极坐标 */
            Polar_Coord_Data_t Action_Polar[4] = {
                /*  r   θ */
                13.0, -15.0f,
                13.0, -15.0f,
                13.0, -15.0f,
                13.0, -15.0f};

            t = HAL_GetTick() / 1000.0f - start_time_jump / 1000.0f; // 跳跃开始后的时间
            /* 跳跃储能阶段 */
            if (t < prep_time)
            {
                // PID_Reset(PID_Position, 5.0f, 0.0008f, 0.0f);
                // PID_Reset(PID_Speed, 8.0f, 0.0001f, 0.0f);
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 3.0f;
                    Motor_T[i].K_W = 0.20f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃上升阶段 */
            else if (t >= prep_time && t < prep_time + launch_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 34.0f;
                    Action_Polar[i].theta = -23.0f;
                }


                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 3.2f;
                    Motor_T[i].K_W = 0.12f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃减速阶段 */
            else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 14.0f;
                    Action_Polar[i].theta = -10.0f;
                }


                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 2.0f;
                    Motor_T[i].K_W = 0.08f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃稳定阶段 */
            else if (t > prep_time + launch_time + fall_time && t < prep_time + launch_time + fall_time + stabilize_time)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    Action_Polar[i].radius = 15.0f;
                    Action_Polar[i].theta = -5.0f;
                }

  
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 1.5f;
                    Motor_T[i].K_W = 0.08f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 跳跃结束 */
            else if (t > prep_time + launch_time + fall_time + stabilize_time)
            {


                Change_NowState(STOP);
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 1.0f;
                    Motor_T[i].K_W = 0.05f;
                }
            }
						
        }
				osDelay(200);
				for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 3.2f;
                    Motor_T[i].K_W = 0.09f;
                }
    }
		
/*************************************后空翻一段（遥控器暂定）*************************************/
		else if (NowState == JUMP_TEST)
    {
			  const float prep_time = 0.8f;   // 准备时间 [s]		0.8
				const float two_time = 0.15f; // 收缩腿前的持续时间 [s]		0.2
				const float three_time = 0.2f;   // 1跳后收腿 [s]		0.8
				const float four_time = 0.2f; //这里该跳了 four 后腿收 前腿蹬
				const float two_frap_time = 0.15f; //第二次前收腿
				const float two_jump_time = 3.6f; //第二次后蹬腿 强拉摆正
				const float five_time = 0.9f; //1跳 five 降落
				const float six_time = 0.9f; //稳固时间
				flag_role=1; //角度转换标志 要转到70 就得写70/2 130=130/2
				float rollf=0,rollb=0;;
				float t = 0;
        while (t <= prep_time + two_time + three_time + four_time + five_time + six_time)
        {
					if (my_rc_ctrl.rc.ch[0]>1600)
					{
						Setdown();
						Change_NowState(STOP);
					}
					rollf=JUMP_FORWARD_Limit(Now_Angle.roll); //-
					rollb=JUMP_BACK_Limit(Now_Angle.roll);    //+
//						if (Now_Angle.roll <=90.0f && Now_Angle.roll>=-90.0f)
//						{
//							Standup_LegL_Offset=LegLinit_twostep;
//							Standup_LegR_Offset=LegRinit_twostep;
//							roll=Now_Angle.roll;
//						}
//						else
//						{
//							
//							Standup_LegL_Offset = (45) * 3.14159f / 180.0f;  //负是逆时针
//							Standup_LegR_Offset = (-160) * 3.14159f / 180.0f;
//							roll=-Now_Angle.roll;
//						}
					
            /* 极坐标 */
            Polar_Coord_Data_t Action_Polar[4] = {
                /*  r   θ */
                15.0, -20/2, //13  -20
                15.0, -20/2,
                15.0, -15/2,  //-15
                15.0, -15/2}; 

            t = HAL_GetTick() / 1000.0f - start_time_jump / 1000.0f; // 跳跃开始后的时间
            /* 跳跃储能阶段 0.8*/
            if (t < prep_time)
            {
                // PID_Reset(PID_Position, 5.0f, 0.0008f, 0.0f);
                // PID_Reset(PID_Speed, 8.0f, 0.0001f, 0.0f);
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 2.0f; //3.0
                    Motor_T[i].K_W = 0.20f; //0.20
                }

                Polar_setCoord(Action_Polar);
            }
            /* 1跳 two 0.15*/
            else if (t >= prep_time && t < prep_time + two_time)  //0.15
            {
                for (uint8_t i = 0; i < 4; i++)
                {
										if (i==1 || i==3) //1 3是后腿
										{
											Action_Polar[i].radius = 40.0f;
											Action_Polar[i].theta = -7/2;
										}
										else
										{
											Action_Polar[i].radius = 15.0f;
											Action_Polar[i].theta = (Now_Angle.roll*(-1.0f)-10)/2;
										}
                    
                }


                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 4.2f;   //4.2
                    Motor_T[i].K_W = 0.08f;   //0.12
                }

                Polar_setCoord(Action_Polar);
            }
						
						
						
            /* 1跳后收腿  three 0.3*/
            //else if (t >= prep_time + two_time && t < prep_time + two_time + three_time)
						else if (((Now_Angle.roll<=-60 && Now_Angle.roll>=-90) || (t >= prep_time + two_time && t < prep_time + two_time + three_time)))
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    if (i==1 || i==3)
										{
											Action_Polar[i].radius = 32.0f;
											Action_Polar[i].theta = -90.0f/2;
										}
										else
										{
											Action_Polar[i].radius = 13.0f;
											Action_Polar[i].theta = (Now_Angle.roll*(-1.0f)-10.0f)/2;
										}
                }


                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 3.0f;  //2.0
                    Motor_T[i].K_W = 0.10f;
                }

                Polar_setCoord(Action_Polar);
            }
            /* 这里该跳了 four 后腿收 前腿蹬 0.2*/
            else if ((t > prep_time + two_time + three_time && t < prep_time + two_time + three_time + four_time) || 
										(Now_Angle.roll<-100 && Now_Angle.roll>-130))
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    if (i==1 || i==3)
										{
											Action_Polar[i].radius = 20.0f;
											Action_Polar[i].theta = -rollb/2-5;
										}
										else
										{
											Action_Polar[i].radius = 38.0f; //32
											Action_Polar[i].theta = -rollf/2+7;  //+5可以翻过来     //-7试试
										}
                }

  
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 4.2f;   //4.2
                    Motor_T[i].K_W = 0.08f;  //0.12
                }

                Polar_setCoord(Action_Polar);
            }
						
//						/* two_frap_time 第二次前收腿------------*/
//            else if ((t > prep_time + two_time + three_time + four_time && t < prep_time + two_time + three_time + four_time + two_frap_time) || 
//										(Now_Angle.roll>-70 && Now_Angle.roll<-30))
//            {
//                for (uint8_t i = 0; i < 4; i++)
//                {
//                    if (i==1 || i==3)
//										{
//											Action_Polar[i].radius = 15.0f;
//											Action_Polar[i].theta = -rollb/2-5;
//										}
//										else
//										{
//											Action_Polar[i].radius = 20.0f; //32
//											Action_Polar[i].theta = -rollf/2-5;
//										}
//                }

//  
//                for (int i = 0; i < 8; i++)
//                {
//                    Motor_T[i].K_P = 4.2f;   //4.2
//                    Motor_T[i].K_W = 0.12f;  //0.12
//                }

//                Polar_setCoord(Action_Polar);
//            }
//						
//						/* two_jump_time 第二次后蹬腿--陀螺仪在范围内的同时时间也在范围内用&&----------*/
//            else if ((t > prep_time + two_time + three_time + four_time + two_frap_time && t < prep_time + two_time + three_time + four_time + two_frap_time + two_jump_time) ||  
//										(Now_Angle.roll>-135 && Now_Angle.roll<-90))
//            {
//                for (uint8_t i = 0; i < 4; i++)
//                {
//                    if (i==1 || i==3)
//										{
//											Action_Polar[i].radius = 18.0f;
//											Action_Polar[i].theta = -rollb/2-5;
//										}
//										else
//										{
//											Action_Polar[i].radius = 17.0f; //32
//											Action_Polar[i].theta = -rollf/2-5;
//										}
//                }

//  
//                for (int i = 0; i < 8; i++)
//                {
//                    Motor_T[i].K_P = 4.2f;   //4.2
//                    Motor_T[i].K_W = 0.12f;  //0.12
//                }

//                Polar_setCoord(Action_Polar);
//            }
						
						/* 1跳 five 降落 1.5*/
            else if  ((t >= prep_time + two_time + three_time + four_time && t < prep_time + two_time + three_time + four_time +  five_time)
//										((Now_Angle.roll>-170 && Now_Angle.roll<-120)||(Now_Angle.roll>90&&Now_Angle.roll<170)))
										//||((Now_Angle.roll>10 && Now_Angle.roll<30))
									||(120<Now_Angle.roll && Now_Angle.roll < 180)
										)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
										if (i==1 || i==3)
										{
											Action_Polar[i].radius = 22.0f;
											Action_Polar[i].theta = -rollb/2+20;  //???
										}
										else
										{
											Action_Polar[i].radius = 17.0f;
											Action_Polar[i].theta = -rollf/2-5;
										}
                    
                }


                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 3.5f;   //3.2
                    Motor_T[i].K_W = 0.12f;   //0.12
                }

                Polar_setCoord(Action_Polar);
            }
						
						/* 砸地 six */
            else if ((t >= prep_time + two_time + three_time + four_time + five_time && t < prep_time + two_time + three_time + four_time + five_time + six_time)
									||(20 < Now_Angle.roll && Now_Angle.roll <= 60)
										)
            {
                for (uint8_t i = 0; i < 4; i++)
                {
										if (i==1 || i==3)
										{
											Action_Polar[i].radius = 22.0f;
											Action_Polar[i].theta = -rollb/2+20;
										}
										else
										{
											Action_Polar[i].radius = 20.0f;
											Action_Polar[i].theta = -rollf/2;
										}
                    
                }


                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 0.6f;   //3.2
                    Motor_T[i].K_W = 0.03f;   //0.12
                }

                Polar_setCoord(Action_Polar);
            }
						
            /* 跳跃结束 */
            else if (t > prep_time + two_time + three_time + four_time  + five_time + six_time)
            {


                Change_NowState(STOP);
                for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 0.3f;
                    Motor_T[i].K_W = 0.03f; //0.05
                }
            }
						
        }
				osDelay(200);//200
				for (int i = 0; i < 8; i++)
                {
                    Motor_T[i].K_P = 1.00f; //3.2
                    Motor_T[i].K_W = 0.012f;
                }
				flag_role=0;
				
    }
		

}



/**
 * @brief 将目标极坐标转化成电机角度并输出到电机
 * @param Action_Polar_Buffer 目标极坐标 数组长度必须为4
 * @return void
 */
void Polar_setCoord(Polar_Coord_Data_t *Action_Polar_Buffer)
{
    /* 直角坐标暂存区 */
    static Cart_Coord_Data_t Action_Cart_Buffer[4] = {0};

    for (uint8_t i = 0; i < 4; i++)
    {
        if (i == 0 || i == 1)
        {
            Polar_toCartesian(&Action_Polar_Buffer[i], &Action_Cart_Buffer[i]);
            Stanford_Type_Lite_SetPosition(&Gait_Data[i], Action_Cart_Buffer[i].cx, Action_Cart_Buffer[i].cy); // 运动学解算待数据填充   //把值存在了Gait_Data这个里
            Stanford_Type_Lite_Inverse_Kinematics(&Gait_Data[i], 1);  //把计算的结果返回给了Gait_Data中的angle 也就是两个大腿的角度
            //			Stanford_Type_Lite_Forward_Kinematics(&Gait_Data[i]);
        }
        else
        {
            Polar_toCartesian(&Action_Polar_Buffer[i], &Action_Cart_Buffer[i]);
            Stanford_Type_Lite_SetPosition(&Gait_Data[i], Action_Cart_Buffer[i].cx, Action_Cart_Buffer[i].cy); // 运动学解算待数据填充
            Stanford_Type_Lite_Inverse_Kinematics(&Gait_Data[i], 0);
            //			Stanford_Type_Lite_Forward_Kinematics(&Gait_Data[i]);
        }
        Aim_Angle[i * 2] = -(pi / 2.0f - Gait_Data[i].angle[1]) / 1.0f; //-顺时针,弧度
        Aim_Angle[i * 2 + 1] = -(pi / 2.0f + Gait_Data[i].angle[0]) / 1.0f;
    }

    Is_Aim_Angle_Get = 1; // 数据填充完毕
}

//不管怎样都是正 右半  角+时垂直地面
float JUMP_BACK_Limit(float angle)
{
	if (angle<=180 && angle >=0) return angle;
	else angle*=(-1.0f);
	return angle;
}

//不管怎样都是负 *-左半  角-时垂直地面
float JUMP_FORWARD_Limit(float angle)
{
	if (angle<=-0 && angle >= -180) return angle;
	else angle*=(-1.0f);
	return angle;
}

