/**
  * @file    task_manager.h
  * @brief   FreeRTOS task creation and management
  */

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "cmsis_os2.h"

extern osThreadId_t motorCtrlTaskHandle;
extern osThreadId_t motionTaskHandle;
extern osThreadId_t heartbeatTaskHandle;
extern osThreadId_t agvMainTaskHandle;
extern osThreadId_t microRosTaskHandle;

extern osMessageQueueId_t cmdQueueHandle;

/**
 * @brief  Create all tasks, queues, mutexes. Call after osKernelInitialize().
 */
void TaskManager_CreateAll(void);

#endif /* TASK_MANAGER_H */
