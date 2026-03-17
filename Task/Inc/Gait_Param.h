#ifndef GAIT_PARAM_H
#define GAIT_PARAM_H

#include <math.h>
#include "stanford_type.h"
#include "cycloid_generator.h"

#define pi (3.14159265f) // 圆周率

#define ReductionAndAngleRatio (1100.262803376891f) // 3591/187/2/pi*360 编码器值与弧度之比

// 机器马运动模式
typedef enum
{
    NORMAL = 0,

    CLIMBING,
    TURN_LEFT,
    TURN_RIGHT,
    WALK,
    TROT,
    RUN,
    MINCE,

    STOP,
    RELEASE,
    JUMP_GROUND,
    JUMP_UPHILL,
    JUMP_UP,

	/*新增*/
		CREEP,  //25匍匐
	  JUMP_TEST, //26跳跃测试
    START,
    END,
	
	JUMP_UP_SLOPE,
} Robohorse_State;

typedef enum
{
    POSITIVE,//positive
    NAGATIVE//nagative
} Robohorse_Direction;

// 腿部PID增益结构体
typedef struct
{
    float kp_pos; // 位置环
    float kd_pos;

    float kp_spd; // 速度环
    float kd_spd;
} LegGain;

// 腿部摆动轨迹生成器参数
typedef struct
{
    Cycloid_Generator_Param_t detached_params_0;
    Cycloid_Generator_Param_t detached_params_1;
    Cycloid_Generator_Param_t detached_params_2;
    Cycloid_Generator_Param_t detached_params_3;
} DetachedParam;

extern LegGain Gait_Gains;                            // 步态增益（PID）
extern LegGain state_gait_gains[10];                  // 运行状态PID参数
extern Cycloid_Generator_Param_t gait_params;         // 当前步态参数
extern Cycloid_Generator_Param_t state_gait_params[]; // 各个状态的步态参数
extern DetachedParam detached_params;                 // 摆动轨迹生成器参数
extern DetachedParam state_detached_params[4];        // 摆动轨迹生成器（数据分离）参数
extern DetachedParam RcDetachedParam;
extern Robohorse_State NowState; // 当前运行状态
extern Robohorse_Direction NowDir;
extern Stanford_Type_Lite_Data_t Gait_Data[4]; // 步态数据
extern Cycloid_Generator_t Cycloid_Gen[4];     // 轨迹生成器结构体
extern float Aim_Angle[8];                     // 目标角度
extern volatile uint8_t Is_Aim_Angle_Get;      // 是否获得目标角度
extern float Last_Time;                        // 上一次运算时间
extern uint8_t Mode_Change_Flag;               // 模式切换标志

#endif
