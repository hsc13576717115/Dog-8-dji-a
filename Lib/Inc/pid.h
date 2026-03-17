/**
 * @file pid.h PID算法
 * @author Zhiyuan Mao(2019th)
 * @brief 由其他学校的PID算法改编而来，功能稍微加了一些
 */
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

#include <stdint.h>

#define LLAST 0
#define LAST 1
#define NOW 2

    typedef enum
    {
        POSITION_PID = 0, // 位置式PID
        DELTA_PID = 1,    // 增量式PID
    } PID_Type_t;

    typedef struct __PID_t
    {
        float p;
        float i;
        float d;

        float set[3]; // 目标值,包含NOW， LAST， LLAST上上次
        float get[3]; // 测量值
        float err[3]; // 误差

        float pout; // p输出
        float iout; // i输出
        float dout; // d输出

        float pos_out;        // 本次位置式输出
        float last_pos_out;   // 上次位置式输出
        float delta_u;        // 本次增量式
        float delta_out;      // 本次增量式输出 = last_delta_out + delta_u
        float last_delta_out; // 上次输出

        PID_Type_t pid_mode;    // 位置模式
        uint32_t MaxOutput;     // 输出限幅
        uint32_t IntegralLimit; // 积分限幅
        float max_err;          // 最大误差值，误差超过最大误差值输出0，不使用设为0
        float deadband;         // PID死区，误差小于死区输出0，不使用设为0 err < deadband return
    } PID_t;

    typedef struct __PID_Set_t
    {
        float p;
        float i;
        float d;

        PID_Type_t pid_mode;    // 位置模式
        uint32_t MaxOutput;     // 输出限幅
        uint32_t IntegralLimit; // 积分限幅
        float max_err;          // 最大误差值，误差超过最大误差值输出0，不使用设为0
        float deadband;         // PID死区，误差小于死区输出0，不使用设为0 err < deadband return
    } PID_Set_t;

#define LIMIT_PARAM(param, max) (param) > (max) ? (param = max) : (param) < -(max) ? (param = -max) \
                                                                                   : (param)

    /**
     * @brief PID的初始设置
     * @param pid (PID_t*) PID结构体
     * @param set (PID_Set_t*) PID设置结构体
     * @return (void) NULL
     */
    void PID_Init(PID_t *pid, PID_Set_t *set);

    /**
     * @brief PID运算
     * @param pid (PID_t*) PID结构体
     * @param get (float) 本次测量获得的值
     * @param set (float) 目标值
     * @return (float) 最后输出值
     */
    float PID_Calc(PID_t *pid, float get, float set);
    /**
     * @brief 重新设置pid的参数
     * @param pid (PID_t*) PID结构体
     * @param p   (float) 参数p
     * @param i   (float) 参数i
     * @param d   (float) 参数d
     */
    void PID_Reset(PID_t *pid, float p, float i, float d);

    extern PID_t PID1_Position[8];  // 位置PID
    extern PID_t PID1_Speed[8];     // 速度PID
    extern PID_t PID1_AUTO;


#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__PID_H*/
