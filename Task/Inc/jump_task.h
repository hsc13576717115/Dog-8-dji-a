#ifndef __JUMP_TASK_H__
#define __JUMP_TASK_H__

#include "stanford_type.h"



void Jump_Task(void *argument);
void Jump_OnlyOnce(void);
void Polar_setCoord(Polar_Coord_Data_t *Action_Polar_Buffer);

float JUMP_BACK_Limit(float angle);
float JUMP_FORWARD_Limit(float angle);

extern float start_time_jump;
#endif
