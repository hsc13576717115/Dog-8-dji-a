#ifndef AHRS_TASK_H
#define AHRS_TASK_H

#include "jy901s.h"

// 位姿校正模式
typedef enum
{
    No_Correction = 0,
    Normal_Correction = 1,
    Climbing_Correction = 2,
    Balance_Correction = 4,
} AHRS_Correction_t;

extern float Target_Angle;
extern float step_len_dev;
extern AHRS_Angle_t Now_Angle;

void AHRS_Task(void *argument);

#endif /*AHRS_TASK_H*/
