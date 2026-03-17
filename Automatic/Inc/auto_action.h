#ifndef __AUTO_ACTION_H__
#define __AUTO_ACTION_H__

#include "remoctrl_task.h"

/**
 * @brief 自动前进
 * @param CH3 正前进/负后退
 */
void AUTO_advance(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float CH3, float CH2);

/**
 * @brief 自动转弯
 * @param CH3 正前进/负后退
 */
void AUTO_turn(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float CH3);

/**
 * @brief 自动转弯
 * @param CH3 正前进/负后退
 */
void AUTO_turn_PID(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float CH3, float CH2);

//陀螺仪控制
void AUTO_JY901s_advance(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float CH3, float CH2,float zheng,float fu);

//摄像头前进
void AUTO_vision_advance(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float CH3, float CH2);
#endif
