/**
  * @file    bsp_encoder.c
  * @brief   Encoder interface: TIM3 (left), TIM4 (right), 4x resolution
  */

#include "bsp_encoder.h"
#include "bsp_gpio.h"

static TIM_HandleTypeDef htim_enc[ENCODER_COUNT];
static int32_t last_count[ENCODER_COUNT];

static void Encoder_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Left encoder: PA6, PA7 */
    ENCODER_L_GPIO_CLK_ENABLE();
    gpio.Pin       = ENCODER_L_PIN_A | ENCODER_L_PIN_B;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = ENCODER_L_TIM_AF;
    HAL_GPIO_Init(ENCODER_L_PORT, &gpio);

    /* Right encoder: PD12, PD13 */
    ENCODER_R_GPIO_CLK_ENABLE();
    gpio.Pin       = ENCODER_R_PIN_A | ENCODER_R_PIN_B;
    gpio.Alternate = ENCODER_R_TIM_AF;
    HAL_GPIO_Init(ENCODER_R_PORT, &gpio);
}

static void Encoder_TIM_Init(void)
{
    TIM_Encoder_InitTypeDef enc = {0};

    enc.EncoderMode  = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = 0x0F;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = 0x0F;

    /* Left: TIM3 */
    ENCODER_L_TIM_CLK_ENABLE();
    htim_enc[ENCODER_LEFT].Instance         = ENCODER_L_TIM;
    htim_enc[ENCODER_LEFT].Init.Prescaler   = 0;
    htim_enc[ENCODER_LEFT].Init.Period       = 0xFFFF;
    htim_enc[ENCODER_LEFT].Init.CounterMode  = TIM_COUNTERMODE_UP;
    htim_enc[ENCODER_LEFT].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(&htim_enc[ENCODER_LEFT], &enc);

    /* Right: TIM4 */
    ENCODER_R_TIM_CLK_ENABLE();
    htim_enc[ENCODER_RIGHT].Instance         = ENCODER_R_TIM;
    htim_enc[ENCODER_RIGHT].Init.Prescaler   = 0;
    htim_enc[ENCODER_RIGHT].Init.Period       = 0xFFFF;
    htim_enc[ENCODER_RIGHT].Init.CounterMode  = TIM_COUNTERMODE_UP;
    htim_enc[ENCODER_RIGHT].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(&htim_enc[ENCODER_RIGHT], &enc);
}

void BSP_Encoder_Init(void)
{
    Encoder_GPIO_Init();
    Encoder_TIM_Init();

    HAL_TIM_Encoder_Start(&htim_enc[ENCODER_LEFT], TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim_enc[ENCODER_RIGHT], TIM_CHANNEL_ALL);

    last_count[ENCODER_LEFT]  = 0;
    last_count[ENCODER_RIGHT] = 0;
}

int32_t BSP_Encoder_GetCount(Encoder_ID_t id)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim_enc[id]);
}

int32_t BSP_Encoder_GetDelta(Encoder_ID_t id)
{
    int32_t now   = (int32_t)__HAL_TIM_GET_COUNTER(&htim_enc[id]);
    int32_t delta = now - last_count[id];

    /* Handle 16-bit wraparound */
    if (delta > 32767)  delta -= 65536;
    if (delta < -32768) delta += 65536;

    last_count[id] = now;
    return delta;
}

int32_t BSP_Encoder_GetSpeed(Encoder_ID_t id, uint32_t elapsed_ms)
{
    int32_t delta = BSP_Encoder_GetDelta(id);
    if (elapsed_ms == 0) return 0;
    return (delta * 1000) / (int32_t)elapsed_ms;
}

void BSP_Encoder_Reset(Encoder_ID_t id)
{
    __HAL_TIM_SET_COUNTER(&htim_enc[id], 0);
    last_count[id] = 0;
}
