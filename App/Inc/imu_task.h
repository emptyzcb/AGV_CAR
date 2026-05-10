/**
  * @file    imu_task.h
  * @brief   IMU attitude estimation using Mahony complementary filter
  */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>

/* ---- Raw IMU data (filled by driver) ---- */
typedef struct {
    float accel[3];  /* ax, ay, az  (m/s^2) */
    float gyro[3];   /* gx, gy, gz  (rad/s)  */
} IMU_RawData_t;

/* ---- Attitude output ---- */
typedef struct {
    float roll;      /* deg */
    float pitch;     /* deg */
    float yaw;       /* deg */
    float quat[4];   /* [w, x, y, z] */
} IMU_Attitude_t;

/* ---- Mahony filter state ---- */
typedef struct {
    float Kp;        /* proportional gain */
    float Ki;        /* integral gain */
    float dt;        /* sample period (s) */
    float eInt[3];   /* integral error */
    float quat[4];   /* quaternion state [w,x,y,z] */
} Mahony_Filter_t;

/* ---- Task entry (registered in task_manager) ---- */
void IMU_TaskEntry(void *argument);

/* ---- Attitude access ---- */
const IMU_Attitude_t* IMU_GetAttitude(void);
void IMU_GetAttitudeCopy(IMU_Attitude_t *out);

/* ---- Mahony filter API ---- */
void Mahony_Init(Mahony_Filter_t *f, float Kp, float Ki, float dt);
void Mahony_Update(Mahony_Filter_t *f, const IMU_RawData_t *raw);
void Mahony_GetEuler(const Mahony_Filter_t *f, float *roll, float *pitch, float *yaw);

#endif /* IMU_TASK_H */
