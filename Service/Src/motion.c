/**
  * @file    motion.c
  * @brief   Differential drive forward/inverse kinematics
  */

#include "motion.h"

#define PI 3.14159265f

void Motion_Init(Motion_t *m, float wheel_radius, float wheel_base, float counts_per_rev)
{
    m->wheel_radius   = wheel_radius;
    m->wheel_base     = wheel_base;
    m->counts_per_rev = counts_per_rev;
    m->cmd_linear     = 0.0f;
    m->cmd_angular    = 0.0f;
    m->left_speed_rps = 0.0f;
    m->right_speed_rps = 0.0f;
    m->meas_linear    = 0.0f;
    m->meas_angular   = 0.0f;
}

void Motion_SetVelocity(Motion_t *m, float linear_v, float angular_v)
{
    m->cmd_linear  = linear_v;
    m->cmd_angular = angular_v;
}

void Motion_InverseKinematics(Motion_t *m)
{
    float v_left  = m->cmd_linear - m->cmd_angular * m->wheel_base / 2.0f;
    float v_right = m->cmd_linear + m->cmd_angular * m->wheel_base / 2.0f;

    float circumference = 2.0f * PI * m->wheel_radius;
    m->left_speed_rps  = v_left  / circumference;
    m->right_speed_rps = v_right / circumference;
}

void Motion_ForwardKinematics(Motion_t *m, float left_rps, float right_rps)
{
    float circumference = 2.0f * PI * m->wheel_radius;
    float v_left  = left_rps  * circumference;
    float v_right = right_rps * circumference;
    m->meas_linear  = (v_right + v_left) / 2.0f;
    m->meas_angular = (v_right - v_left) / m->wheel_base;
}

void Motion_Stop(Motion_t *m)
{
    m->cmd_linear     = 0.0f;
    m->cmd_angular    = 0.0f;
    m->left_speed_rps = 0.0f;
    m->right_speed_rps = 0.0f;
}

float Motion_GetLeftRPS(Motion_t *m)
{
    return m->left_speed_rps;
}

float Motion_GetRightRPS(Motion_t *m)
{
    return m->right_speed_rps;
}
