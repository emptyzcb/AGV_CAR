/**
  * @file    bsp_imu.h
  * @brief   IMU660RC (LSM6DSV16X) SPI driver
  */

#ifndef BSP_IMU_H
#define BSP_IMU_H

#include <stdint.h>

typedef struct {
    float accel[3];  /* m/s^2 */
    float gyro[3];   /* rad/s */
} BSP_IMU_Data_t;

void    BSP_IMU_Init(void);
int     BSP_IMU_IsReady(void);
int     BSP_IMU_ReadData(BSP_IMU_Data_t *data);

#endif /* BSP_IMU_H */
