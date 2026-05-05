/**
  * @file    monitor.c
  * @brief   System monitor: heartbeat LED, FreeRTOS stack/heap stats
  */

#include "monitor.h"
#include "bsp_system.h"
#include "FreeRTOS.h"
#include "task.h"

void Monitor_Init(void)
{
    /* Nothing extra needed - LED is already init by BSP_System_Init */
}

void Monitor_Heartbeat(void)
{
    BSP_LED_Toggle();
}

void Monitor_CheckStack(void)
{
    /* Can be extended to print via UART */
    (void)0;
}

uint32_t Monitor_GetStackHighWater(TaskHandle_t handle)
{
    if (handle != NULL)
        return uxTaskGetStackHighWaterMark(handle) * sizeof(StackType_t);
    return 0;
}

void Monitor_GetHeapInfo(uint32_t *free_bytes, uint32_t *min_ever)
{
    if (free_bytes) *free_bytes = xPortGetFreeHeapSize();
    if (min_ever)   *min_ever   = xPortGetMinimumEverFreeHeapSize();
}
