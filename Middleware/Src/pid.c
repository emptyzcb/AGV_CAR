/**
  * @file    pid.c
  * @brief   PID controller implementation
  */

#include "pid.h"

void PID_Init(PID_t *pid, PID_Mode_t mode,
              float kp, float ki, float kd,
              float dt, float out_max)
{
    pid->mode        = mode;
    pid->kp          = kp;
    pid->ki          = ki;
    pid->kd          = kd;
    pid->dt          = dt;
    pid->out_max     = out_max;
    pid->out_min     = -out_max;
    pid->integral_max = out_max;
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output      = 0.0f;
}

float PID_Compute(PID_t *pid, float error)
{
    if (pid->mode == PID_MODE_POSITION)
    {
        pid->integral += error * pid->dt;
        if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
        if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

        float derivative = (error - pid->prev_error) / pid->dt;
        pid->output = pid->kp * error
                    + pid->ki * pid->integral
                    + pid->kd * derivative;
    }
    else
    {
        float delta = pid->kp * (error - pid->prev_error)
                    + pid->ki * error
                    + pid->kd * (error - 2.0f * pid->prev_error + pid->prev_prev_error);
        pid->output += delta;
        pid->prev_prev_error = pid->prev_error;
    }

    if (pid->output >  pid->out_max) pid->output =  pid->out_max;
    if (pid->output <  pid->out_min) pid->output =  pid->out_min;

    pid->prev_error = error;
    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral        = 0.0f;
    pid->prev_error      = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output          = 0.0f;
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
