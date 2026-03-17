#include "cycloid_generator.h"
#include <math.h>
#include "pid.h"
#include "gait_param.h"
#include "VMC.h"

#define M_PI (3.14159265f)

extern polarCoordinates_C C_polar;
extern uint8_t Mode_Change_Flag;
extern PID_t PID_CycAlgm[2];

float _test = 0;
float _test1 = 0;
// FIXME:Cycloid_Generator()第三个参数不能为0 否则切换步态会导致一条腿残废
void Cycloid_Generator_Calc(
    float now_time,
    Cycloid_Generator_t *cyc,
    float phase,
    Cycloid_Generator_Solution_t *solution)
{
    float temp_steplenth = (cyc->param.step_length < 0) ? (-cyc->param.step_length) : (cyc->param.step_length); //绝对值

    if (Mode_Change_Flag == 1 ||
        Mode_Change_Flag == 2 ||
        Mode_Change_Flag == 3 ||
        Mode_Change_Flag == 4) // 设置初始相位
    {
        cyc->now_phase = phase;
        Mode_Change_Flag += 1;
    }
    else
        Mode_Change_Flag = 0;

    cyc->now_phase += cyc->param.freq * (now_time - cyc->last_time); // 计算当前相位  频率*所用时间
    cyc->last_time = now_time;                                       // 记录当前时间

    cyc->now_phase = fmodf(cyc->now_phase, 1.0f);    // 计算相位量（单位为周期）  //使当前相位的值在(-1,1)之间
    if (cyc->now_phase <= cyc->param.flight_percent) // 处于摆动相
    {
        solution->x = (cyc->now_phase / cyc->param.flight_percent) * temp_steplenth - temp_steplenth / 2.0f;
        solution->y = -cyc->param.up_amp * sinf(M_PI * cyc->now_phase / cyc->param.flight_percent) + cyc->param.body_height;
			  C_polar.L=pow((solution->x * solution->x + solution->y * solution->y),0.5);
				C_polar.zeta=atan(solution->y/solution->x);
    }
    else // 处于支撑相
    {
        float percentBack = (cyc->now_phase - cyc->param.flight_percent) / (1.0f - cyc->param.flight_percent);
        solution->x = -percentBack * temp_steplenth + temp_steplenth / 2.0f;
        solution->y = (cyc->param.down_amp * sinf(M_PI * percentBack) + cyc->param.body_height) / 1.0f;
    }

    if (cyc->param.step_length < 0) // 方向相反
    {
        solution->x = -solution->x;
        NowDir = NAGATIVE;
    }
    else
        NowDir = POSITIVE;
}
