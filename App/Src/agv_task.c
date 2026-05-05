/**
  * @file    agv_task.c
  * @brief   AGV tasks: MotorCtrl (10ms), Motion (20ms), AgvMain (50ms), Heartbeat (500ms)
  */

#include "agv_task.h"
#include "motor_ctrl.h"
#include "motion.h"
#include "monitor.h"
#include "micro_ros_node.h"
#include "cmsis_os2.h"
#include <math.h>

/* ---- Shared state ---- */
static AGV_State_t agv_state = {
    .mode          = AGV_MODE_IDLE,
    .target_linear = 0.0f,
    .target_angular = 0.0f,
    .last_cmd_tick = 0,
    .error_code    = 0,
};

static MotorCtrl_t motor_ctrl[MOTOR_COUNT];
static Motion_t    motion;

/* ---- AGV state access ---- */

const AGV_State_t* AGV_GetState(void)
{
    return &agv_state;
}

void AGV_SetMode(AGV_Mode_t mode)
{
    agv_state.mode = mode;
    if (mode == AGV_MODE_IDLE || mode == AGV_MODE_ERROR)
    {
        MotorCtrl_Stop(&motor_ctrl[MOTOR_LEFT]);
        MotorCtrl_Stop(&motor_ctrl[MOTOR_RIGHT]);
        Motion_Stop(&motion);
    }
}

void AGV_SetVelocity(float linear, float angular)
{
    agv_state.target_linear  = linear;
    agv_state.target_angular = angular;
    agv_state.last_cmd_tick  = osKernelGetTick();
}

/* ---- MotorCtrl Task: 10ms ---- */

void MotorCtrl_TaskEntry(void *argument)
{
    (void)argument;

    /* Default: 1320 counts/rev (11PPR * 4x * 30:1 gear), 10ms period */
    MotorCtrl_Init(&motor_ctrl[MOTOR_LEFT],  1320.0f, 0.01f);
    MotorCtrl_Init(&motor_ctrl[MOTOR_RIGHT], 1320.0f, 0.01f);

    uint32_t tick = osKernelGetTick();
    for (;;)
    {
        MotorCtrl_Update(&motor_ctrl[MOTOR_LEFT],  MOTOR_LEFT,  ENCODER_LEFT,  10);
        MotorCtrl_Update(&motor_ctrl[MOTOR_RIGHT], MOTOR_RIGHT, ENCODER_RIGHT, 10);
        tick += 10;
        osDelayUntil(tick);
    }
}

/* ---- Motion Task: 20ms ---- */

void Motion_TaskEntry(void *argument)
{
    (void)argument;

    /* Default: wheel_radius=0.033m, wheel_base=0.160m */
    Motion_Init(&motion, 0.033f, 0.160f, 1320.0f);

    uint32_t tick = osKernelGetTick();
    for (;;)
    {
        /* Compute inverse kinematics from velocity commands */
        Motion_InverseKinematics(&motion);

        /* Feed wheel speed targets to motor controllers */
        MotorCtrl_SetSpeed(&motor_ctrl[MOTOR_LEFT],  motion.left_speed_rps);
        MotorCtrl_SetSpeed(&motor_ctrl[MOTOR_RIGHT], motion.right_speed_rps);

        /* Forward kinematics for odometry */
        float left_rps  = MotorCtrl_GetSpeed(&motor_ctrl[MOTOR_LEFT]);
        float right_rps = MotorCtrl_GetSpeed(&motor_ctrl[MOTOR_RIGHT]);
        Motion_ForwardKinematics(&motion, left_rps, right_rps);

        /* Update shared odom data for micro-ROS publisher */
        {
            const float dt = 0.020f;
            g_odom_data.linear_vel  = motion.meas_linear;
            g_odom_data.angular_vel = motion.meas_angular;
            g_odom_data.theta += motion.meas_angular * dt;
            /* Normalize theta to [-pi, pi] */
            while (g_odom_data.theta >  3.14159265f) g_odom_data.theta -= 6.28318530f;
            while (g_odom_data.theta < -3.14159265f) g_odom_data.theta += 6.28318530f;
            g_odom_data.x += motion.meas_linear * cosf(g_odom_data.theta) * dt;
            g_odom_data.y += motion.meas_linear * sinf(g_odom_data.theta) * dt;
            g_odom_data.timestamp_ms = osKernelGetTick();
        }

        tick += 20;
        osDelayUntil(tick);
    }
}

/* ---- AgvMain Task: 50ms ---- */

void AgvMain_TaskEntry(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTick();
    for (;;)
    {
        /* Command timeout check: stop if no cmd for 2 seconds */
        if (agv_state.mode == AGV_MODE_REMOTE)
        {
            if ((osKernelGetTick() - agv_state.last_cmd_tick) > 2000)
            {
                AGV_SetMode(AGV_MODE_IDLE);
            }
        }

        /* Apply velocity to motion module */
        if (agv_state.mode != AGV_MODE_IDLE && agv_state.mode != AGV_MODE_ERROR)
        {
            Motion_SetVelocity(&motion, agv_state.target_linear, agv_state.target_angular);
        }
        else
        {
            Motion_Stop(&motion);
        }

        tick += 50;
        osDelayUntil(tick);
    }
}

/* ---- Heartbeat Task: 500ms ---- */

void Heartbeat_TaskEntry(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTick();
    for (;;)
    {
        Monitor_Heartbeat();
        tick += 500;
        osDelayUntil(tick);
    }
}
