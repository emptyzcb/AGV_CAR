/**
  * @file    bsp_uart.c
  * @brief   USART1 driver: interrupt RX with ring buffer
  *
  * USART1 is initialized by CubeMX (MX_USART1_UART_Init in main.c).
  * This module only provides TX/RX helper functions and a ring buffer
  * for interrupt-driven reception.
  */

#include "bsp_uart.h"
#include "main.h"

#define RX_BUF_SIZE 512

extern UART_HandleTypeDef huart1;

static uint8_t  rx_ring[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static uint8_t  rx_byte;
static BSP_UART_RxCb_t rx_cb = NULL;

void BSP_UART_Init(uint32_t baudrate)
{
    (void)baudrate;
    /* USART1 is already initialized by MX_USART1_UART_Init() in main.c */
}

int BSP_UART_Send(const uint8_t *data, uint16_t len)
{
    if (HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 100) == HAL_OK)
        return 0;
    return -1;
}

int BSP_UART_SendString(const char *str)
{
    uint16_t len = 0;
    while (str[len]) len++;
    return BSP_UART_Send((const uint8_t *)str, len);
}

void BSP_UART_StartReceive(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void BSP_UART_RegisterCallback(BSP_UART_RxCb_t cb)
{
    rx_cb = cb;
}

/* Called from HAL UART IRQ handler */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* Push byte into ring buffer */
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next != rx_tail)
        {
            rx_ring[rx_head] = rx_byte;
            rx_head = next;
        }

        /* Notify user callback */
        if (rx_cb) rx_cb(rx_byte);

        /* Re-arm reception */
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

uint16_t BSP_UART_Available(void)
{
    return (rx_head - rx_tail + RX_BUF_SIZE) % RX_BUF_SIZE;
}

uint16_t BSP_UART_Read(uint8_t *buf, uint16_t len)
{
    uint16_t count = 0;
    while (count < len && rx_tail != rx_head)
    {
        buf[count++] = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    }
    return count;
}
