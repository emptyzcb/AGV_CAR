/**
  * @file    agv_task.h
  * @brief   AGV state machine and task entry functions
  */

#ifndef AGV_TASK_H
#define AGV_TASK_H

#include <stdint.h>

typedef enum {
    AGV_MODE_IDLE   = 0,
    AGV_MODE_REMOTE = 1,
    AGV_MODE_AUTO   = 2,
    AGV_MODE_MANUAL = 3,
    AGV_MODE_ERROR  = 0xFF
} AGV_Mode_t;

typedef struct {
    AGV_Mode_t mode;
    float target_linear;
    float target_angular;
    uint32_t last_cmd_tick;
    uint32_t error_code;
} AGV_State_t;

/* Task entry functions */
void AgvMain_TaskEntry(void *argument);
void MotorCtrl_TaskEntry(void *argument);
void Motion_TaskEntry(void *argument);
void Heartbeat_TaskEntry(void *argument);

const AGV_State_t* AGV_GetState(void);
void AGV_SetMode(AGV_Mode_t mode);
void AGV_SetVelocity(float linear, float angular);

#endif /* AGV_TASK_H */
