/**
  * @file    task_manager.c
  * @brief   Create all FreeRTOS tasks and inter-task objects
  */

#include "task_manager.h"
#include "agv_task.h"
#include "micro_ros_node.h"

/* Task handles */
osThreadId_t motorCtrlTaskHandle;
osThreadId_t motionTaskHandle;
osThreadId_t heartbeatTaskHandle;
osThreadId_t agvMainTaskHandle;
osThreadId_t microRosTaskHandle;

/* Message queue */
osMessageQueueId_t cmdQueueHandle;

static const osThreadAttr_t motorCtrl_attr = {
    .name       = "MotorCtrl",
    .stack_size = 256 * 4,
    .priority   = osPriorityAboveNormal,
};

static const osThreadAttr_t motion_attr = {
    .name       = "Motion",
    .stack_size = 256 * 4,
    .priority   = osPriorityNormal,
};

static const osThreadAttr_t agvMain_attr = {
    .name       = "AgvMain",
    .stack_size = 512 * 4,
    .priority   = osPriorityNormal,
};

static const osThreadAttr_t heartbeat_attr = {
    .name       = "Heartbeat",
    .stack_size = 128 * 4,
    .priority   = osPriorityBelowNormal,
};

static const osThreadAttr_t microRos_attr = {
    .name       = "MicroROS",
    .stack_size = 4096 * 4,
    .priority   = osPriorityNormal,
};

void TaskManager_CreateAll(void)
{
    cmdQueueHandle = osMessageQueueNew(8, sizeof(uint32_t), NULL);

    motorCtrlTaskHandle = osThreadNew(MotorCtrl_TaskEntry, NULL, &motorCtrl_attr);
    motionTaskHandle    = osThreadNew(Motion_TaskEntry,    NULL, &motion_attr);
    agvMainTaskHandle   = osThreadNew(AgvMain_TaskEntry,   NULL, &agvMain_attr);
    heartbeatTaskHandle = osThreadNew(Heartbeat_TaskEntry, NULL, &heartbeat_attr);
    microRosTaskHandle  = osThreadNew(MicroROS_TaskEntry,  NULL, &microRos_attr);
}
