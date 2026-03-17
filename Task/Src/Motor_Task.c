#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include "usart.h"

#include "motor_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "GO-MOTOR.h"
#include "usart.h"

#include "Gait_Param.h"

#include "Standup_Task.h"

#include "VMC.h"



extern float imu_Leg_offset;

extern MOTOR_send Motor_T[8];
extern MOTOR_recv Motor_R[8];

extern angle_leg Leg;

extern PID_t PID_Position[8];
extern PID_t PID_Speed[8];

float PID_Aim_Angle[8]; // 将用于PID运算的目标角度
extern float Initial_Angle[8];

float StandUp_Angle_Individual[8];

/**
 * 电机驱动任务
 * 流程：装载新的位置-> 双环PID运算->发送新的电流值
 * */
void Motor_Task(void *argument)
{
    while (1)
    {
        if (Is_Aim_Angle_Get == 1) // 是否已经获得了新的目标角度
        {
            for (uint8_t i = 0; i < 8; i++)
            {
                if (i == 1 || i == 3 || i == 4 || i == 6) //内侧腿
                {
                    if (i % 2 == 0) //右  4 6 右左
                        PID_Aim_Angle[i] = ( Aim_Angle[i] + Standup_LegR_Offset - imu_Leg_offset + Initial_Angle[i]); // 数据转移
                    else  //左  1 3 左右
                        PID_Aim_Angle[i] = -( Aim_Angle[i] + Standup_LegR_Offset - imu_Leg_offset - Initial_Angle[i] ); // 数据转移
                }
                else  // 外侧腿
                {
                    if (i % 2 == 0) // 0 2 左左
                        PID_Aim_Angle[i] = -( Aim_Angle[i]+Standup_LegL_Offset+ imu_Leg_offset - Initial_Angle[i]);// 数据转移
                    else //5 7 右右
                        PID_Aim_Angle[i] = (Aim_Angle[i]+Standup_LegL_Offset + imu_Leg_offset + Initial_Angle[i] ); // 数据转移
                }
								
								if ((i == 4 || i == 6 || i == 5 || i == 7) && flag_creep_LR == R) //右
								{
									if (i % 2 == 0)
										PID_Aim_Angle[i] =  ( Aim_Angle[i] + Standup_LegR_Offset + CREEP_LegR - imu_Leg_offset + Initial_Angle[i]);  //试一下 感觉给正 电机好像是逆时针转(给一个负值)
									else
										PID_Aim_Angle[i] =  ( Aim_Angle[i] + Standup_LegL_Offset + CREEP_LegR - imu_Leg_offset + Initial_Angle[i]);  //试一下 感觉给正 电机好像是逆时针转(给一个负值)
								}
								else if (flag_creep_LR == L && (i == 0 || i == 1 || i == 2 || i == 3))//左
								{
									if (i % 2 == 0)
										PID_Aim_Angle[i] = -( Aim_Angle[i] + Standup_LegL_Offset + CREEP_LegL - imu_Leg_offset - Initial_Angle[i] ); //(给正值)
									else
										PID_Aim_Angle[i] = -( Aim_Angle[i] + Standup_LegR_Offset + CREEP_LegL - imu_Leg_offset - Initial_Angle[i] ); //(给正值)
								}
            }
            Is_Aim_Angle_Get = 0; // 清零标志
        }
				
//        
//				PID_Calc(&PID_Position[7],Motor_R[7].Pos/6.33,PID_Aim_Angle[7]);
//				taskENTER_CRITICAL();
//        printf("%f,%f,%f\n",Motor_R[7].Pos/6.33,PID_Aim_Angle[7],PID_Position[7].pos_out);
//        taskEXIT_CRITICAL();
//				taskENTER_CRITICAL();
//				taskEXIT_CRITICAL();

				        
        GO_motor_position(&Motor_T[0], Motor_T[0].id, PID_Aim_Angle[0]);
        GO_motor_position(&Motor_T[1], Motor_T[1].id, PID_Aim_Angle[1]);
        GO_motor_position(&Motor_T[2], Motor_T[2].id, PID_Aim_Angle[2]);
        GO_motor_position(&Motor_T[3], Motor_T[3].id, PID_Aim_Angle[3]);
        GO_motor_position(&Motor_T[4], Motor_T[4].id, PID_Aim_Angle[4]);
        GO_motor_position(&Motor_T[5], Motor_T[5].id, PID_Aim_Angle[5]);
        GO_motor_position(&Motor_T[6], Motor_T[6].id, PID_Aim_Angle[6]);
        GO_motor_position(&Motor_T[7], Motor_T[7].id, PID_Aim_Angle[7]);//PID_Position[7].pos_out);
				
        SERVO_Send_recv(&Motor_T[0], &Motor_R[0]);//发送和接收数据
			  SERVO_Send_recv(&Motor_T[1], &Motor_R[1]);
				SERVO_Send_recv(&Motor_T[2], &Motor_R[2]);	
				SERVO_Send_recv(&Motor_T[3], &Motor_R[3]);
				SERVO_Send_recv(&Motor_T[4], &Motor_R[4]);
				SERVO_Send_recv(&Motor_T[5], &Motor_R[5]);
				SERVO_Send_recv(&Motor_T[6], &Motor_R[6]);
				SERVO_Send_recv(&Motor_T[7], &Motor_R[7]);
				
				
				
				
        // for (int i = 0; i < 8; i++)
        // {
        //     // 位置PID
        //     // PID_Calc(&PID_Position[i], Motor_R[i].Pos, PID_Aim_Angle[i]);
        //     // 速度PID7
        //     // PID_Calc(&PID_Speed[i], Motor_R[i].W, PID_Position[i].pos_out);
        //     // PID_Calc(&PID_Speed[i], Motor_R[i].W, 15);
        //     // GO_motor_speed(&Motor_T[i], Motor_T[i].id, PID_Speed[i].pos_out);
        //     GO_motor_position(&Motor_T[i], Motor_T[i].id,PID_Aim_Angle[i]);
        //     SERVO_Send_recv(&Motor_T[i], &Motor_R[i]); // 将控制指令发送给电机，同时接收返回值
        // }
        /* 绝对延时 */

        static portTickType xLastWakeTime;
        static const portTickType xFrequency = pdMS_TO_TICKS(3);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        //        osDelay(3);
    }
}


void Motor_Set_MaxSpeed(PID_t *pid, uint16_t speed)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        pid[i].MaxOutput = speed;
    }
}

