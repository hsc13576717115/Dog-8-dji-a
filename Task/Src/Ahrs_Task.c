#include "Ahrs_Task.h"
//#include "hwt906.h"

#include "pid.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "gait_param.h"
#include <math.h>

#define L1 (42.0f) //前后腿的距离



extern PID_t PID_IMU[3];
AHRS_Angle_t Now_Angle;

AHRS_Correction_t Correction_Mode = Normal_Correction;

/*步长初始值(可调整)*/
float step_len_initial = 14;
/*步高初始值(可调整)*/
float step_high_initial = 16;
/*步长相对平衡位置差值*/
float step_len_dev = 0.0;
/*步高相对平衡位置差值*/
float step_high_dev = 0.0;
float flight_percent_dev = 0.0;

float yaw_now = 0.0;
float yaw_set = 0.0;

float roll_set = 0.0;
float pitch_set = 0.0;

/*横滚矫正偏移量*/
float roll_offset = 0.0;
/*俯仰矫正偏移量*/
float pitch_offset = 0.0;

/*左腿参数 右腿参数*/
float normal_params_l, normal_params_r;

float revise_LegLenth[4] = {0};

float imu_Leg_offset = 0.0f;
float Target_Angle = 0.0f;

extern uint8_t flag_Standup;
extern PID_t PID_IMU_Line;
extern PID_t PID_IMU_Climbing;

void caculate_LegRevise(AHRS_Angle_t *now_angle);

/**
 * 姿态控制任务
 * 流程：陀螺仪读取->计算校正量（未实装）
 */
void AHRS_Task(void *argument)
{
    while (1)
    {

        //        HWT906_Read(&Now_Angle); //读取陀螺仪各个数据
        JY901_Read(&Now_Angle);
        if (flag_Standup && jy901_on == 1)
        {
            caculate_LegRevise(&Now_Angle);
        }
        // TODO: 计算电机输出校正量的部分

        /*
        if (Correction_Mode & Normal_Correction)
        {
            step_len_dev = PID_Calc(&PID_IMU[2], Now_Angle.Yaw, yaw_set);
            //矫正限幅
            float _dev_limit = 14.0;
            //取绝对值
            if (step_len_dev > _dev_limit)
                step_len_dev = _dev_limit;
            else if (step_len_dev < -_dev_limit)
                step_len_dev = -_dev_limit;

            normal_params_l = step_len_initial - step_len_dev;
            normal_params_r = step_len_initial + step_len_dev;
            if (normal_params_l < 0)
                normal_params_l = 0;
            if (normal_params_r < 0)
                normal_params_r = 0;

            state_detached_params[NORMAL].detached_params_0.step_length = normal_params_l;
            state_detached_params[NORMAL].detached_params_2.step_length = normal_params_l;
            state_detached_params[NORMAL].detached_params_1.step_length = normal_params_r;
            state_detached_params[NORMAL].detached_params_3.step_length = normal_params_r;
        }
        else if (Correction_Mode&Climbing_Correction)
        {
            flight_percent_dev=pid_calc();
        }
 */			
        static portTickType xLastWakeTime;
        static const portTickType xFrequency = pdMS_TO_TICKS(3);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief 保持x=0时，电机轴到马脚与地面接触点连线垂直
 * @param now_angle imu姿态角数据
 *
 */
void caculate_LegRevise(AHRS_Angle_t *now_angle)
{
    taskENTER_CRITICAL();
    imu_Leg_offset = now_angle->roll * 2.0f * 3.14159f / 360;
    imu_Leg_offset = (imu_Leg_offset > 22 * 2.0f * 3.14159f / 360) ? (22 * 2.0f * 3.14159f / 360) : (imu_Leg_offset < -22 * 2.0f * 3.14159f / 360) ? (-22 * 2.0f * 3.14159f / 360)
                                                                                                                                                : (imu_Leg_offset);

//    if (Correction_Mode == Normal_Correction)
//    {
//        step_len_dev = PID_Calc(&PID_IMU_Line, now_angle->yaw, Target_Angle);
//    }
//    else if (Correction_Mode == Climbing_Correction)
//    {
//        flight_percent_dev = PID_Calc(&PID_IMU_Climbing, now_angle->yaw, Target_Angle);
//    }
    taskEXIT_CRITICAL();
}
