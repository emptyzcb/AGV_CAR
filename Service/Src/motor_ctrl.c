/**
  * @file    motor_ctrl.c
  * @brief   Motor PID speed closed-loop control
  */

#include "motor_ctrl.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"

void MotorCtrl_Init(MotorCtrl_t *ctrl, float counts_per_rev, float sample_period_s)
{
    ctrl->counts_per_rev   = counts_per_rev;
    ctrl->sample_period_s  = sample_period_s;
    ctrl->mode             = MOTOR_CTRL_STOP;
    ctrl->target_speed_rps = 0.0f;
    ctrl->measured_speed_rps = 0.0f;
    ctrl->current_duty     = 0;

    PID_Init(&ctrl->pid, PID_MODE_POSITION,
             50.0f, 10.0f, 0.0f,  /* default gains, tune later */
             sample_period_s, 1000.0f);

    Filter_EMA_Init(&ctrl->speed_filter, 0.3f);
}

void MotorCtrl_SetPID(MotorCtrl_t *ctrl, float kp, float ki, float kd)
{
    PID_SetGains(&ctrl->pid, kp, ki, kd);
}

void MotorCtrl_SetSpeed(MotorCtrl_t *ctrl, float speed_rps)
{
    ctrl->target_speed_rps = speed_rps;
    ctrl->mode = MOTOR_CTRL_SPEED;
}

void MotorCtrl_SetDuty(MotorCtrl_t *ctrl, int16_t duty)
{
    ctrl->current_duty = duty;
    ctrl->mode = MOTOR_CTRL_DUTY;
}

void MotorCtrl_Stop(MotorCtrl_t *ctrl)
{
    ctrl->mode = MOTOR_CTRL_STOP;
    ctrl->target_speed_rps = 0.0f;
    ctrl->current_duty = 0;
    PID_Reset(&ctrl->pid);
}

void MotorCtrl_Update(MotorCtrl_t *ctrl, int motor_id, int encoder_id, uint32_t elapsed_ms)
{
    if (ctrl->mode == MOTOR_CTRL_STOP)
    {
        BSP_Motor_Stop((Motor_ID_t)motor_id);
        ctrl->current_duty = 0;
        return;
    }

    /* Read encoder and compute speed in RPS */
    int32_t delta = BSP_Encoder_GetDelta((Encoder_ID_t)encoder_id);
    float raw_rps = ((float)delta / ctrl->counts_per_rev) * (1000.0f / (float)elapsed_ms);
    ctrl->measured_speed_rps = Filter_EMA_Update(&ctrl->speed_filter, raw_rps);

    if (ctrl->mode == MOTOR_CTRL_DUTY)
    {
        BSP_Motor_SetDuty((Motor_ID_t)motor_id, ctrl->current_duty);
        return;
    }

    /* SPEED mode: PID control */
    float error = ctrl->target_speed_rps - ctrl->measured_speed_rps;
    float pid_out = PID_Compute(&ctrl->pid, error);

    int16_t duty = (int16_t)pid_out;
    if (duty > 1000)  duty = 1000;
    if (duty < -1000) duty = -1000;

    ctrl->current_duty = duty;
    BSP_Motor_SetDuty((Motor_ID_t)motor_id, duty);
}

float MotorCtrl_GetSpeed(MotorCtrl_t *ctrl)
{
    return ctrl->measured_speed_rps;
}
