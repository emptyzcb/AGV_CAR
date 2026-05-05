/**
  * @file    bsp_system.c
  * @brief   System-level utilities implementation
  */

#include "bsp_system.h"
#include "bsp_gpio.h"

void BSP_System_Init(void)
{
    /* Enable DWT cycle counter for us delay */
    BSP_DWT_Init();

    /* Configure heartbeat LED pin as output */
    LED_HEARTBEAT_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = LED_HEARTBEAT_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_HEARTBEAT_PORT, &gpio);
}

uint32_t BSP_GetTick(void)
{
    return HAL_GetTick();
}

void BSP_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

void BSP_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void BSP_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

void BSP_LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN);
}

void BSP_LED_On(void)
{
    HAL_GPIO_WritePin(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN, GPIO_PIN_SET);
}

void BSP_LED_Off(void)
{
    HAL_GPIO_WritePin(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN, GPIO_PIN_RESET);
}
