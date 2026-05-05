/**
  * @file    motion.h
  * @brief   Differential drive kinematics
  */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

typedef struct {
    /* Physical params */
    float wheel_radius;    /* meters */
    float wheel_base;      /* meters */
    float counts_per_rev;

    /* Commands */
    float cmd_linear;      /* m/s */
    float cmd_angular;     /* rad/s */

    /* Computed wheel targets */
    float left_speed_rps;
    float right_speed_rps;

    /* Measured */
    float meas_linear;
    float meas_angular;
} Motion_t;

void  Motion_Init(Motion_t *m, float wheel_radius, float wheel_base, float counts_per_rev);
void  Motion_SetVelocity(Motion_t *m, float linear_v, float angular_v);
void  Motion_InverseKinematics(Motion_t *m);
void  Motion_ForwardKinematics(Motion_t *m, float left_rps, float right_rps);
void  Motion_Stop(Motion_t *m);
float Motion_GetLeftRPS(Motion_t *m);
float Motion_GetRightRPS(Motion_t *m);

#endif /* MOTION_H */
