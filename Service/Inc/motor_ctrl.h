/**
  * @file    motor_ctrl.h
  * @brief   Motor control service: PID speed closed-loop
  */

#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <stdint.h>
#include "pid.h"

typedef enum {
    MOTOR_CTRL_STOP = 0,
    MOTOR_CTRL_SPEED = 1,
    MOTOR_CTRL_DUTY  = 2
} MotorCtrl_Mode_t;

typedef struct {
    /* Config */
    float counts_per_rev;
    float sample_period_s;

    /* State */
    MotorCtrl_Mode_t mode;
    float target_speed_rps;
    float measured_speed_rps;
    int16_t current_duty;

    /* PID */
    PID_t pid;

    /* Measurement */
    Filter_EMA_t speed_filter;  /* forward declare - need filter.h */
} MotorCtrl_t;

#include "filter.h"

void  MotorCtrl_Init(MotorCtrl_t *ctrl, float counts_per_rev, float sample_period_s);
void  MotorCtrl_SetPID(MotorCtrl_t *ctrl, float kp, float ki, float kd);
void  MotorCtrl_SetSpeed(MotorCtrl_t *ctrl, float speed_rps);
void  MotorCtrl_SetDuty(MotorCtrl_t *ctrl, int16_t duty);
void  MotorCtrl_Stop(MotorCtrl_t *ctrl);
void  MotorCtrl_Update(MotorCtrl_t *ctrl, int motor_id, int encoder_id, uint32_t elapsed_ms);
float MotorCtrl_GetSpeed(MotorCtrl_t *ctrl);

#endif /* MOTOR_CTRL_H */
