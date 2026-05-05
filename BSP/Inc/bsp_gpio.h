/**
  * @file    bsp_gpio.h
  * @brief   Board pin mapping - all hardware pin assignments in one place
  */

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "stm32f4xx_hal.h"

/* ---- Motor Direction Pins (TB6612 / L298N style H-bridge) ---- */
#define MOTOR_L_IN1_PORT    GPIOB
#define MOTOR_L_IN1_PIN     GPIO_PIN_12
#define MOTOR_L_IN2_PORT    GPIOB
#define MOTOR_L_IN2_PIN     GPIO_PIN_13
#define MOTOR_R_IN1_PORT    GPIOB
#define MOTOR_R_IN1_PIN     GPIO_PIN_14
#define MOTOR_R_IN2_PORT    GPIOB
#define MOTOR_R_IN2_PIN     GPIO_PIN_15

/* ---- PWM Timer (TIM1, APB2 = 168MHz, but timer clock = 168MHz for APB2) ---- */
#define MOTOR_PWM_TIM               TIM1
#define MOTOR_PWM_TIM_CH_L          TIM_CHANNEL_1   /* PA8 */
#define MOTOR_PWM_TIM_CH_R          TIM_CHANNEL_2   /* PA9 */
#define MOTOR_PWM_TIM_AF            GPIO_AF1_TIM1
#define MOTOR_PWM_PORT              GPIOA
#define MOTOR_PWM_PIN_L             GPIO_PIN_8
#define MOTOR_PWM_PIN_R             GPIO_PIN_9
#define MOTOR_PWM_TIM_CLK_ENABLE()  __HAL_RCC_TIM1_CLK_ENABLE()

/* ---- Encoder Left (TIM3, APB1 = 42MHz, timer clock = 84MHz) ---- */
#define ENCODER_L_TIM               TIM3
#define ENCODER_L_TIM_AF            GPIO_AF2_TIM3
#define ENCODER_L_PORT              GPIOA
#define ENCODER_L_PIN_A             GPIO_PIN_6
#define ENCODER_L_PIN_B             GPIO_PIN_7
#define ENCODER_L_TIM_CLK_ENABLE()  __HAL_RCC_TIM3_CLK_ENABLE()
#define ENCODER_L_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

/* ---- Encoder Right (TIM4) ---- */
#define ENCODER_R_TIM               TIM4
#define ENCODER_R_TIM_AF            GPIO_AF2_TIM4
#define ENCODER_R_PORT              GPIOD
#define ENCODER_R_PIN_A             GPIO_PIN_12
#define ENCODER_R_PIN_B             GPIO_PIN_13
#define ENCODER_R_TIM_CLK_ENABLE()  __HAL_RCC_TIM4_CLK_ENABLE()
#define ENCODER_R_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()

/* ---- UART (USART1) ---- */
#define COMM_UART                   USART1
#define COMM_UART_AF                GPIO_AF7_USART1
#define COMM_UART_TX_PORT           GPIOA
#define COMM_UART_TX_PIN            GPIO_PIN_9
#define COMM_UART_RX_PORT           GPIOA
#define COMM_UART_RX_PIN            GPIO_PIN_10
#define COMM_UART_PORT              GPIOA
#define COMM_UART_CLK_ENABLE()      __HAL_RCC_USART1_CLK_ENABLE()
#define COMM_UART_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

/* ---- LED ---- */
#define LED_HEARTBEAT_PORT          GPIOC
#define LED_HEARTBEAT_PIN           GPIO_PIN_13
#define LED_HEARTBEAT_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

/* ---- GPIO helper macros ---- */
#define MOTOR_L_IN1_SET()   HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET)
#define MOTOR_L_IN1_CLR()   HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET)
#define MOTOR_L_IN2_SET()   HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET)
#define MOTOR_L_IN2_CLR()   HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET)
#define MOTOR_R_IN1_SET()   HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_SET)
#define MOTOR_R_IN1_CLR()   HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET)
#define MOTOR_R_IN2_SET()   HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_SET)
#define MOTOR_R_IN2_CLR()   HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET)

#endif /* BSP_GPIO_H */
