/**
  * @file    bsp_motor.h
  * @brief   Motor driver abstraction (PWM + H-bridge direction)
  */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1,
    MOTOR_COUNT = 2
} Motor_ID_t;

/**
 * @brief  Init TIM1 PWM (20kHz) and direction GPIOs.
 */
void BSP_Motor_Init(void);

/**
 * @brief  Set motor duty cycle and direction.
 * @param  id    MOTOR_LEFT or MOTOR_RIGHT
 * @param  duty  -1000..+1000 (permille). Positive=forward, negative=backward.
 */
void BSP_Motor_SetDuty(Motor_ID_t id, int16_t duty);

/**
 * @brief  Active brake (both outputs HIGH).
 */
void BSP_Motor_Brake(Motor_ID_t id);

/**
 * @brief  Coast stop (both outputs LOW, PWM disabled).
 */
void BSP_Motor_Stop(Motor_ID_t id);

/**
 * @brief  Emergency stop: brake all motors.
 */
void BSP_Motor_EmergencyStop(void);

#endif /* BSP_MOTOR_H */
