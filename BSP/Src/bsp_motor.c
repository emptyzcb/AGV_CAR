/**
  * @file    bsp_motor.c
  * @brief   Motor PWM (TIM1, 20kHz) + H-bridge direction GPIO
  */

#include "bsp_motor.h"
#include "bsp_gpio.h"

static TIM_HandleTypeDef htim_pwm;

/* APB2 timer clock = 168MHz. 20kHz => ARR = 168000000/20000 - 1 = 8399 */
#define PWM_ARR  8399

static void MX_TIM1_PWM_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    MOTOR_PWM_TIM_CLK_ENABLE();

    htim_pwm.Instance               = MOTOR_PWM_TIM;
    htim_pwm.Init.Prescaler         = 0;
    htim_pwm.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim_pwm.Init.Period            = PWM_ARR;
    htim_pwm.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_pwm.Init.RepetitionCounter = 0;
    htim_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim_pwm);

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim_pwm, &oc, MOTOR_PWM_TIM_CH_L);
    HAL_TIM_PWM_ConfigChannel(&htim_pwm, &oc, MOTOR_PWM_TIM_CH_R);
}

/* MSP callback: enable clocks, configure GPIO AF */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == MOTOR_PWM_TIM)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {0};

        /* PWM pins: PA8, PA9 */
        gpio.Pin       = MOTOR_PWM_PIN_L | MOTOR_PWM_PIN_R;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = MOTOR_PWM_TIM_AF;
        HAL_GPIO_Init(MOTOR_PWM_PORT, &gpio);

        /* Direction pins: PB12..PB15 */
        gpio.Pin   = MOTOR_L_IN1_PIN | MOTOR_L_IN2_PIN |
                     MOTOR_R_IN1_PIN | MOTOR_R_IN2_PIN;
        gpio.Mode  = GPIO_MODE_OUTPUT_PP;
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
}

void BSP_Motor_Init(void)
{
    MX_TIM1_PWM_Init();
    HAL_TIM_PWM_Start(&htim_pwm, MOTOR_PWM_TIM_CH_L);
    HAL_TIM_PWM_Start(&htim_pwm, MOTOR_PWM_TIM_CH_R);
}

void BSP_Motor_SetDuty(Motor_ID_t id, int16_t duty)
{
    /* Clamp */
    if (duty > 1000)  duty = 1000;
    if (duty < -1000) duty = -1000;

    /* Set direction */
    if (duty >= 0)
    {
        /* Forward: IN1=HIGH, IN2=LOW */
        if (id == MOTOR_LEFT)  { MOTOR_L_IN1_SET(); MOTOR_L_IN2_CLR(); }
        else                   { MOTOR_R_IN1_SET(); MOTOR_R_IN2_CLR(); }
    }
    else
    {
        /* Backward: IN1=LOW, IN2=HIGH */
        if (id == MOTOR_LEFT)  { MOTOR_L_IN1_CLR(); MOTOR_L_IN2_SET(); }
        else                   { MOTOR_R_IN1_CLR(); MOTOR_R_IN2_SET(); }
        duty = -duty;
    }

    /* Map 0..1000 to 0..PWM_ARR */
    uint32_t compare = (uint32_t)duty * (PWM_ARR + 1) / 1000;
    if (id == MOTOR_LEFT)
        __HAL_TIM_SET_COMPARE(&htim_pwm, MOTOR_PWM_TIM_CH_L, compare);
    else
        __HAL_TIM_SET_COMPARE(&htim_pwm, MOTOR_PWM_TIM_CH_R, compare);
}

void BSP_Motor_Brake(Motor_ID_t id)
{
    if (id == MOTOR_LEFT)  { MOTOR_L_IN1_SET(); MOTOR_L_IN2_SET(); }
    else                   { MOTOR_R_IN1_SET(); MOTOR_R_IN2_SET(); }

    if (id == MOTOR_LEFT)
        __HAL_TIM_SET_COMPARE(&htim_pwm, MOTOR_PWM_TIM_CH_L, 0);
    else
        __HAL_TIM_SET_COMPARE(&htim_pwm, MOTOR_PWM_TIM_CH_R, 0);
}

void BSP_Motor_Stop(Motor_ID_t id)
{
    if (id == MOTOR_LEFT)  { MOTOR_L_IN1_CLR(); MOTOR_L_IN2_CLR(); }
    else                   { MOTOR_R_IN1_CLR(); MOTOR_R_IN2_CLR(); }

    if (id == MOTOR_LEFT)
        __HAL_TIM_SET_COMPARE(&htim_pwm, MOTOR_PWM_TIM_CH_L, 0);
    else
        __HAL_TIM_SET_COMPARE(&htim_pwm, MOTOR_PWM_TIM_CH_R, 0);
}

void BSP_Motor_EmergencyStop(void)
{
    BSP_Motor_Brake(MOTOR_LEFT);
    BSP_Motor_Brake(MOTOR_RIGHT);
}
