#include "pid.h"
#include "gait_param.h"

#define ABS(x) ((x > 0) ? (x) : (-x))

PID_t PID1_Position[8];  // 位置PID
PID_t PID1_Speed[8];     // 速度PID
PID_t PID1_AUTO;         // 陀螺仪矫正

void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

void PID_Init(PID_t *pid, PID_Set_t *set)
{
    pid->p = set->p;
    pid->i = set->i;
    pid->d = set->d;

    pid->pid_mode = set->pid_mode;
    pid->MaxOutput = set->MaxOutput;
    pid->IntegralLimit = set->IntegralLimit;
    pid->max_err = set->max_err;
    pid->deadband = set->deadband;
}

float PID_Calc(PID_t *pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get; // set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)
        return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

    if (pid->pid_mode == POSITION_PID) // 位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out; // update last time
    }
    else if (pid->pid_mode == DELTA_PID) // 增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out; // update last time
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}

void PID_Reset(PID_t *pid, float p, float i, float d)
{
    pid->p = p;
    pid->i = i;
    pid->d = d;
}
