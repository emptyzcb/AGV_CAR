/**
  * @file    bsp_uart.h
  * @brief   UART abstraction with interrupt RX and ring buffer
  */

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include <stddef.h>

typedef void (*BSP_UART_RxCb_t)(uint8_t byte);

/**
 * @brief  Init USART1 at given baudrate (8N1).
 */
void BSP_UART_Init(uint32_t baudrate);

/**
 * @brief  Send data (blocking).
 */
int BSP_UART_Send(const uint8_t *data, uint16_t len);

/**
 * @brief  Send a null-terminated string (blocking).
 */
int BSP_UART_SendString(const char *str);

/**
 * @brief  Start interrupt-based byte reception.
 */
void BSP_UART_StartReceive(void);

/**
 * @brief  Register callback for each received byte (optional).
 */
void BSP_UART_RegisterCallback(BSP_UART_RxCb_t cb);

/**
 * @brief  Bytes available in RX ring buffer.
 */
uint16_t BSP_UART_Available(void);

/**
 * @brief  Read bytes from RX ring buffer.
 * @return Actual bytes read.
 */
uint16_t BSP_UART_Read(uint8_t *buf, uint16_t len);

#endif /* BSP_UART_H */
