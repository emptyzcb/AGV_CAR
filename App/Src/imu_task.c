/**
  * @file    imu_task.c
  * @brief   IMU task (10ms) with Mahony complementary filter attitude estimation
  *
  *          Driver interface is a stub — replace IMU_Driver_ReadData()
  *          when BSP driver is implemented.
  */

#include "imu_task.h"
#include <math.h>
#include "cmsis_os2.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define IMU_TASK_PERIOD_MS  10
#define RAD_TO_DEG          (180.0f / M_PI)

/* ---- Shared attitude output (read-only for other tasks) ---- */
static IMU_Attitude_t imu_attitude;

/* ---- Mahony filter instance ---- */
static Mahony_Filter_t mahony;

/* ============================================================
 * Driver stub — returns zeroed data, replace with real driver
 * ============================================================ */
static int8_t IMU_Driver_ReadData(IMU_RawData_t *data)
{
    (void)data;
    /* TODO: replace with real IMU660RC driver read */
    data->accel[0] = 0.0f;
    data->accel[1] = 0.0f;
    data->accel[2] = 9.8f;  /* stationary: 1g on Z axis */
    data->gyro[0]  = 0.0f;
    data->gyro[1]  = 0.0f;
    data->gyro[2]  = 0.0f;
    return 0; /* success */
}

/* ============================================================
 * Mahony complementary filter
 * ============================================================ */

void Mahony_Init(Mahony_Filter_t *f, float Kp, float Ki, float dt)
{
    f->Kp = Kp;
    f->Ki = Ki;
    f->dt = dt;
    f->eInt[0] = 0.0f;
    f->eInt[1] = 0.0f;
    f->eInt[2] = 0.0f;
    /* identity quaternion */
    f->quat[0] = 1.0f;
    f->quat[1] = 0.0f;
    f->quat[2] = 0.0f;
    f->quat[3] = 0.0f;
}

void Mahony_Update(Mahony_Filter_t *f, const IMU_RawData_t *raw)
{
    float q0 = f->quat[0], q1 = f->quat[1];
    float q2 = f->quat[2], q3 = f->quat[3];

    float ax = raw->accel[0], ay = raw->accel[1], az = raw->accel[2];
    float gx = raw->gyro[0],  gy = raw->gyro[1],  gz = raw->gyro[2];

    /* Normalise accelerometer */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) return;
    ax /= norm; ay /= norm; az /= norm;

    /* Estimated direction of gravity */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* Error = cross product of estimated and measured gravity */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    /* PI controller: accumulate integral error */
    if (f->Ki > 0.0f)
    {
        f->eInt[0] += ex * f->dt;
        f->eInt[1] += ey * f->dt;
        f->eInt[2] += ez * f->dt;
    }
    else
    {
        f->eInt[0] = 0.0f;
        f->eInt[1] = 0.0f;
        f->eInt[2] = 0.0f;
    }

    /* Apply feedback to gyroscope */
    gx += f->Kp * ex + f->Ki * f->eInt[0];
    gy += f->Kp * ey + f->Ki * f->eInt[1];
    gz += f->Kp * ez + f->Ki * f->eInt[2];

    /* Integrate quaternion rate */
    float half_dt = 0.5f * f->dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * half_dt;
    q1 += ( qa * gx + qc * gz - q3 * gy) * half_dt;
    q2 += ( qa * gy - qb * gz + q3 * gx) * half_dt;
    q3 += ( qa * gz + qb * gy - qc * gx) * half_dt;

    /* Normalise quaternion */
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm < 1e-6f) return;
    f->quat[0] = q0 / norm;
    f->quat[1] = q1 / norm;
    f->quat[2] = q2 / norm;
    f->quat[3] = q3 / norm;
}

void Mahony_GetEuler(const Mahony_Filter_t *f, float *roll, float *pitch, float *yaw)
{
    float q0 = f->quat[0], q1 = f->quat[1];
    float q2 = f->quat[2], q3 = f->quat[3];

    *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                    1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp);
    else
        *pitch = asinf(sinp) * RAD_TO_DEG;

    *yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                    1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}

/* ============================================================
 * Attitude access
 * ============================================================ */

const IMU_Attitude_t* IMU_GetAttitude(void)
{
    return &imu_attitude;
}

void IMU_GetAttitudeCopy(IMU_Attitude_t *out)
{
    *out = imu_attitude;
}

/* ============================================================
 * IMU Task: 10ms period
 * ============================================================ */

void IMU_TaskEntry(void *argument)
{
    (void)argument;

    IMU_RawData_t raw;
    float roll, pitch, yaw;

    /* Init Mahony filter: Kp=10, Ki=0.003, dt=0.01s */
    Mahony_Init(&mahony, 10.0f, 0.003f, 0.01f);

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        if (IMU_Driver_ReadData(&raw) == 0)
        {
            Mahony_Update(&mahony, &raw);
            Mahony_GetEuler(&mahony, &roll, &pitch, &yaw);

            imu_attitude.roll  = roll;
            imu_attitude.pitch = pitch;
            imu_attitude.yaw   = yaw;
            imu_attitude.quat[0] = mahony.quat[0];
            imu_attitude.quat[1] = mahony.quat[1];
            imu_attitude.quat[2] = mahony.quat[2];
            imu_attitude.quat[3] = mahony.quat[3];
        }

        tick += IMU_TASK_PERIOD_MS;
        osDelayUntil(tick);
    }
}
