/**
  * @file    bsp_system.h
  * @brief   System-level utilities: tick, delay, DWT, LED
  */

#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H

#include "stm32f4xx_hal.h"

void     BSP_System_Init(void);
uint32_t BSP_GetTick(void);
void     BSP_Delay(uint32_t ms);
void     BSP_DelayUs(uint32_t us);
void     BSP_DWT_Init(void);
void     BSP_LED_Toggle(void);
void     BSP_LED_On(void);
void     BSP_LED_Off(void);

#endif /* BSP_SYSTEM_H */
