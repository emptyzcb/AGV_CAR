/**
  * @file    monitor.h
  * @brief   System monitor: heartbeat LED, stack/heap check
  */

#ifndef MONITOR_H
#define MONITOR_H

#include <stdint.h>

void     Monitor_Init(void);
void     Monitor_Heartbeat(void);
void     Monitor_CheckStack(void);
uint32_t Monitor_GetStackHighWater(const char *task_name);
void     Monitor_GetHeapInfo(uint32_t *free_bytes, uint32_t *min_ever);

#endif /* MONITOR_H */
