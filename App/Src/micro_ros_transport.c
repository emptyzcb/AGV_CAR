/**
  * @file    micro_ros_transport.c
  * @brief   micro-ROS custom serial transport over USART1
  *
  * Bridges micro-ROS XRCE-DDS client to BSP_UART using the existing
  * interrupt-driven ring buffer. Implements the uxrCustomTransport
  * interface required by rmw_uros_set_custom_transport().
  */

#include "micro_ros_transport.h"
#include "bsp_uart.h"

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include "cmsis_os2.h"

#define MICRO_ROS_BAUDRATE  115200

static bool usart_open(struct uxrCustomTransport *transport)
{
    (void)transport;
    BSP_UART_Init(MICRO_ROS_BAUDRATE);
    BSP_UART_StartReceive();
    return true;
}

static bool usart_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    return true;
}

static size_t usart_write(struct uxrCustomTransport *transport,
                          const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)transport;
    (void)err;

    if (BSP_UART_Send(buf, (uint16_t)len) == 0)
        return len;
    return 0;
}

static size_t usart_read(struct uxrCustomTransport *transport,
                         uint8_t *buf, size_t len,
                         int timeout_ms, uint8_t *err)
{
    (void)transport;
    (void)err;

    uint32_t start_tick = osKernelGetTick();
    size_t total_read = 0;

    while (total_read < len)
    {
        uint16_t avail = BSP_UART_Available();
        if (avail > 0)
        {
            uint16_t to_read = (uint16_t)(len - total_read);
            if (to_read > avail) to_read = avail;
            total_read += BSP_UART_Read(&buf[total_read], to_read);
        }
        else
        {
            if ((int32_t)(osKernelGetTick() - start_tick) >= timeout_ms)
                break;
            osDelay(1);
        }
    }
    return total_read;
}

void micro_ros_transport_init(void)
{
    rmw_uros_set_custom_transport(
        true,
        NULL,
        usart_open,
        usart_close,
        usart_write,
        usart_read
    );
}
