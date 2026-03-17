/**
 * @file cycloid_generator.h 摆线轨迹生成器(抄完发现是正弦轨迹生成器)
 * @author 网上
 * @note 足端轨迹有好几种，这其实是正弦轨迹生成器，以后再改吧
 */
#ifndef __CYCLOID_GENERATOR_H
#define __CYCLOID_GENERATOR_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>
#include "pid.h"

    typedef struct
    {
        float body_height;    // 身体到地面的距离 (cm)
        float step_length;    // 一步的距离 (cm)  
        float up_amp;         // 上部振幅y (cm)
        float down_amp;       // 下部振幅 (cm)
        float flight_percent; // 摆动相百分比(%)
        float freq;           // 一步的频率 (Hz)
		float change_step_length; //设定的一步的距离：意思为这个就是我们设置的 上面那个是马使用的(随意)
    } Cycloid_Generator_Param_t;

    typedef struct
    {
        Cycloid_Generator_Param_t param; // 摆线生成器参数结构体
        float last_time;                 // 上一次计算时间 (单位：s)
        float now_phase;                 // 目前相位 (单位：n个周期)
    } Cycloid_Generator_t;

    typedef struct
    {
        float x; // 解算结果x
        float y; // 解算结果y
    } Cycloid_Generator_Solution_t;

    /**
     * @brief 摆线生成器运算
     * @param now_time (float)当前时间(单位：s)
     * @param cyc (Cycloid_Generator_t*) 摆线结构体指针
     * @param direction (Cycloid_Generator_Direction_t) 摆线方向
     * @param phase (float) 摆线相位(单位：n个周期)
     * @param solution (Cycloid_Generator_Solution_t*)摆线计算结果结构体指针
     */
    extern void Cycloid_Generator_Calc(
        float now_time,
        Cycloid_Generator_t *cyc,
        float phase,
        Cycloid_Generator_Solution_t *solution);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__CYCLOID_GENERATOR_H*/
