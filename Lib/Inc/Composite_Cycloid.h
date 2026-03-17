#ifndef __COMPOSITE_CYCLOID_H__
#define __COMPOSITE_CYCLOID_H__

typedef struct
{
	float body_height;
	float step_length;    //一步的距离 (cm)
	float up_amp;         //上部振幅y (cm)
	float flight_percent; //摆动相百分比(%)
	float freq;           //一步的频率 (Hz)
} Composite_Cycloid_Param_t;

typedef struct
{
	Composite_Cycloid_Param_t param; //摆线生成器参数结构体
	float last_time;                 //上一次计算时间 (单位：s)
	float now_phase;                 //目前相位 (单位：n个周期)
} Composite_Cycloid_t;

typedef struct
{
	float x; //解算结果x
	float y; //解算结果y
} Composite_Cycloid_Solution_t;

void Composite_Cycloid_Calc(
    float now_time,
    Composite_Cycloid_t *cyc,
    float phase,
    Composite_Cycloid_Solution_t *solution);

#endif
