#ifndef __AUTO_ACTION_H__
#define __AUTO_ACTION_H__

#include "remoctrl_task.h"

/**
 * @brief 自动前进
 * @param CH3 正前进/负后退
 */
void AUTO_advance_t(Composite_Cycloid_t *pCycGenerato, Robohorse_State state, float CH3, float CH2);

/**
 * @brief 自动转弯
 * @param CH3 正前进/负后退
 */
void AUTO_turn_t(Composite_Cycloid_t *pCycGenerato, Robohorse_State state, float CH3);

/**
 * @brief 自动转弯
 * @param CH3 正前进/负后退
 */
void AUTO_turn_PID_t(Composite_Cycloid_t *pCycGenerato, Robohorse_State state, float CH2);

/**
 * @brief 陀螺仪辅助自动转弯
 * @param CH3 正前进/负后退
 */
void AUTO_turn_Gyro_t(Composite_Cycloid_t *pCycGenerato, float set_angle);


#endif
