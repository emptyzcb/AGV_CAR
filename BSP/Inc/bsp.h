/**
  * @file    bsp.h
  * @brief   Master BSP include - init all hardware in correct order
  */

#ifndef BSP_H
#define BSP_H

/**
 * @brief  Initialize all BSP modules.
 *         Call from main() after HAL_Init() and SystemClock_Config(),
 *         before osKernelInitialize().
 */
void BSP_Init(void);

#endif /* BSP_H */
