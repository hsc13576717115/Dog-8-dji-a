#ifndef POSTURE_TASK_H
#define POSTURE_TASK_H

#include "gait_param.h"

#define smart 0 //Ò£¿Ø0 ×ÔÖ÷1

void Posture_Task(void *argument);
void Change_NowState(Robohorse_State target);
void Calculate_Gait(Robohorse_State state);
void Target_toMotor(void);
void ALL_AUTO_2(Cycloid_Generator_t *pCycGenerato);
void ALL_AUTO_3(Cycloid_Generator_t *pCycGenerato);
void ALL_AUTO_4(Cycloid_Generator_t *pCycGenerato);
void ALL_AUTO_5(Cycloid_Generator_t *pCycGenerato); //2025ÕÏ°­
#endif /*POSTURE_TASK_H*/
