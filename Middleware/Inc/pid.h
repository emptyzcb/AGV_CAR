/**
  * @file    pid.h
  * @brief   Generic PID controller (position & incremental mode)
  */

#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef enum {
    PID_MODE_POSITION    = 0,
    PID_MODE_INCREMENTAL = 1
} PID_Mode_t;

typedef struct {
    /* Config */
    PID_Mode_t mode;
    float kp, ki, kd;
    float dt;            /* period in seconds */

    /* Limits */
    float out_max;
    float out_min;
    float integral_max;  /* anti-windup clamp */

    /* State */
    float integral;
    float prev_error;
    float prev_prev_error;
    float output;
} PID_t;

void  PID_Init(PID_t *pid, PID_Mode_t mode,
               float kp, float ki, float kd,
               float dt, float out_max);

float PID_Compute(PID_t *pid, float error);

void  PID_Reset(PID_t *pid);

void  PID_SetGains(PID_t *pid, float kp, float ki, float kd);

#endif /* PID_H */
