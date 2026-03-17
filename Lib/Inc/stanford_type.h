/**
 * @file stanford_type.h 仿斯坦福四足机器人(并联腿)的运动学算法
 * @author Zhiyuan Mao(2019th)
 * @note 目前只完成了lite版本，完善版本由于网上提供的算法有些问题，以后再填坑
 */
#ifndef __STANFORD_TYPE_H
#define __STANFORD_TYPE_H

#include "stm32f4xx.h"
extern uint8_t flag_role;
#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

    typedef struct
    {
        float leg_length[6]; // L1,L2,L3,L4,L5,L6
        float angle[2];      // alpha1,alpha2
        float position[2];   //(x,y)
    } Stanford_Type_Data_t;

    typedef struct
    {
        float leg_length[2]; // L1,L2
        float angle[2];      // alpha1,alpha2
        float position[2];   //(x,y)
    } Stanford_Type_Lite_Data_t;

    typedef struct
    {
        float radius;
        float theta;
    } Polar_Coord_Data_t;

    typedef struct
    {
        float cx;
        float cy;
    } Cart_Coord_Data_t;

    /**
     * @brief 仿斯坦福结构模型结构体初始化
     * @param data (Stanford_Type_Data_t *) 仿斯坦福结构模型结构体指针
     * @param Lx(X=1,2,3,4,5,6) (float) 各条腿的长度
     * @return (void)NULL
     */
    extern inline void Stanford_Type_Init(Stanford_Type_Data_t *data,
                                          float l1, float l2, float l3, float l4, float l5, float l6)
    {
        data->leg_length[0] = l1;
        data->leg_length[1] = l2;
        data->leg_length[2] = l3;
        data->leg_length[3] = l4;
        data->leg_length[4] = l5;
        data->leg_length[5] = l6;
    }

    /**
     * @brief 仿斯坦福结构模型结构体设置角度
     * @param data (Stanford_Type_Data_t *) 仿斯坦福结构模型结构体指针
     * @param alpha1 (float) 目标弧度值1
     * @param alpha2 (float) 目标弧度值2
     * @return (void) NULL
     */
    extern inline void Stanford_Type_SetAngle(
        Stanford_Type_Data_t *data,
        float alpha1,
        float alpha2)
    {
        data->angle[0] = alpha1;
        data->angle[1] = alpha2;
    }

    /**
     * @brief 仿斯坦福结构模型结构体设置坐标
     * @param data (Stanford_Type_Data_t *) 仿斯坦福结构模型结构体指针
     * @param x (float) 目标x坐标
     * @param y (float) 目标y坐标
     * @return (void) NULL
     */
    extern inline void Stanford_Type_SetPosition(
        Stanford_Type_Data_t *data,
        float x,
        float y)
    {
        data->position[0] = x;
        data->position[1] = y;
    }

    /**
     * @brief 仿斯坦福结构模型运动学正解(弧度值转坐标)
     * @param data (Stanford_Type_Data_t *) 仿斯坦福结构模型结构体指针
     * @return (void) NULL
     */
    extern void Stanford_Type_Forward_Kinematics(Stanford_Type_Data_t *data);

    /**
     * @brief 仿斯坦福结构模型运动学逆解(坐标转弧度值)
     * @param data (Stanford_Type_Data_t *) 仿斯坦福结构模型结构体指针
     * @return (void) NULL
     */
    extern void Stanford_Type_Inverse_Kinematics(Stanford_Type_Data_t *data);

    /**
     * @brief 仿斯坦福结构精简版模型结构体初始化
     * @param data (Stanford_Type_Data_t *) 仿斯坦福结构模型结构体指针
     * @param L1 1号腿长度
     * @param L2 2号腿长度
     * @return (void)NULL
     */
    extern inline void Stanford_Type_Lite_Init(
        Stanford_Type_Lite_Data_t *data,
        float l1,
        float l2)
    {
        data->leg_length[0] = l1;
        data->leg_length[1] = l2;
    }

    /**
     * @brief 仿斯坦福结构精简版模型结构体角度设置
     * @param data (Stanford_Type_Data_t *) 仿斯坦福结构模型结构体指针
     * @param alpha1 目标弧度值1
     * @param alpha2 目标弧度值2
     * @return (void) NULL
     */
    extern inline void Stanford_Type_Lite_SetAngle(
        Stanford_Type_Lite_Data_t *data,
        float alpha1,
        float alpha2)
    {
        data->angle[0] = alpha1;
        data->angle[1] = alpha2;
    }

    /**
     * @brief 仿斯坦福结构精简版模型结构体坐标设置
     * @param data (Stanford_Type_Data_t *) 仿斯坦福结构模型结构体指针
     * @param x (float) 目标x坐标
     * @param y (float) 目标y坐标
     * @return (void) NULL
     */
    extern inline void Stanford_Type_Lite_SetPosition(
        Stanford_Type_Lite_Data_t *data,
        float x,
        float y)
    {
        data->position[0] = x;
        data->position[1] = y;
    }

    /**
     * @brief 仿斯坦福结构精简版模型运动学正解(弧度值转坐标)
     * @param data (Stanford_Type_Data_t *) 仿斯坦福结构模型结构体指针
     * @return (void) NULL
     */
    extern void Stanford_Type_Lite_Forward_Kinematics(Stanford_Type_Lite_Data_t *data);

    /**
     * @brief 仿斯坦福结构精简版模型运动学逆解(坐标转弧度值)
     * @param data (Stanford_Type_Data_t *) 仿斯坦福结构模型结构体指针
     * @return (void) NULL
     */
    extern void Stanford_Type_Lite_Inverse_Kinematics(Stanford_Type_Lite_Data_t *data, int leg);

    /**
     * @brief 将极坐标系转化为直角坐标系
     * @param polar 极坐标
     * @param cart 直角坐标
     *
     */
    extern void Polar_toCartesian(Polar_Coord_Data_t *polar, Cart_Coord_Data_t *cart);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__STANFORD_TYPE_H*/
