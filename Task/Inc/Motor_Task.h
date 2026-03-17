#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "pid.h"

void Motor_Task(void *argument);
void Motor_Set_MaxSpeed(PID_t *pid, uint16_t speed);
void Send_Angle(float a,float b,float c,float d,float e,float f,float g,float h);
void Motor_Task_Individual(void *argument);
void Motor_Task_X(void *argument);
#endif /*MOTOR_TASK_H*/

