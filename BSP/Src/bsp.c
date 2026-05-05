/**
  * @file    bsp.c
  * @brief   Master BSP init - calls all sub-module init functions
  */

#include "bsp.h"
#include "bsp_system.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"

void BSP_Init(void)
{
    BSP_System_Init();
    BSP_Motor_Init();
    BSP_Encoder_Init();
    BSP_UART_Init(115200);
}
