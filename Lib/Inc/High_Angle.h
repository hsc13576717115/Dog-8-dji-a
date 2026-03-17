#ifndef __HIGH_ANGLE_H__
#define __HIGH_ANGLE_H__

#include "Standup_Task.h"

#define Leglength 260 //[mm]

void Change_Angle(float angle);
double High_To_Angle(float h);

double calculate_h(double alpha);
double calculate_derivative(double alpha);
double solve_alpha(double h_target);

#endif
