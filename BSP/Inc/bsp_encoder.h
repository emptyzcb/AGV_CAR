/**
  * @file    bsp_encoder.h
  * @brief   Encoder abstraction (TIM3 left, TIM4 right)
  */

#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

typedef enum {
    ENCODER_LEFT  = 0,
    ENCODER_RIGHT = 1,
    ENCODER_COUNT = 2
} Encoder_ID_t;

/**
 * @brief  Init TIM3/TIM4 in encoder interface mode (4x resolution).
 */
void BSP_Encoder_Init(void);

/**
 * @brief  Get raw counter value.
 */
int32_t BSP_Encoder_GetCount(Encoder_ID_t id);

/**
 * @brief  Get signed delta since last call (handles 16-bit wraparound).
 */
int32_t BSP_Encoder_GetDelta(Encoder_ID_t id);

/**
 * @brief  Calculate speed in counts/sec from delta and elapsed time.
 */
int32_t BSP_Encoder_GetSpeed(Encoder_ID_t id, uint32_t elapsed_ms);

/**
 * @brief  Reset counter to zero.
 */
void BSP_Encoder_Reset(Encoder_ID_t id);

#endif /* BSP_ENCODER_H */
