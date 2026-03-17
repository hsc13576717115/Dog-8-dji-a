#ifndef __GO_MOTOR_H
#define __GO_MOTOR_H

#include "motor_control.h"
#include "pid.h"

void Motor_Init(void);                                                                                                          // 电机及其pid初始化
void GO_motor_speed(MOTOR_send *motor1, int id, float W);                                                                   // 速度模式
void GO_motor_position(MOTOR_send *motor1, int id, float Pos);                                                              // 位置模式
void GO_motor_damp(MOTOR_send *motor1, int id, float K_W);                                                                  // 阻尼模式
void GO_motor_moment(MOTOR_send *motor1, int id, float T);                                                                  // 力矩模式
void GO_motor_Zero_moment(MOTOR_send *motor1, int id);                                                                      // 零力矩模式
void GO_motor_Mix_moment(MOTOR_send *motor1, MOTOR_recv *rdata, int id, float T, float W, float Pos, float K_P, float K_W); // 混合力矩模式
#endif
