/**
  * @file    bsp_imu.c
  * @brief   IMU660RC (LSM6DSV16X) SPI driver
  *
  *          SPI3: PC10=SCK, PC11=MISO, PC12=MOSI, PB0=CS
  */

#include "bsp_imu.h"
#include "bsp_gpio.h"
#include "bsp_system.h"

/* ---- LSM6DSV16X register map ---- */
#define LSM_REG_WHO_AM_I        0x0F
#define LSM_WHO_AM_I_VAL        0x70

#define LSM_REG_CTRL1           0x10  /* Accel ODR + FS */
#define LSM_REG_CTRL2           0x11  /* Gyro  ODR + FS */
#define LSM_REG_CTRL3           0x12  /* BDU, IF_INC */
#define LSM_REG_OUTX_L_A        0x28  /* Accel XL .. ZH */
#define LSM_REG_OUTX_L_G        0x22  /* Gyro  XL .. ZH */

/* Accel full-scale: ±8g, ODR 104Hz */
#define LSM_CTRL1_VAL           0x44  /* FS=±8g(0x40) | ODR=104Hz(0x04) */
/* Gyro full-scale: ±2000dps, ODR 104Hz */
#define LSM_CTRL2_VAL           0x44  /* FS=±2000(0x40) | ODR=104Hz(0x04) */
/* BDU=1, IF_INC=1 */
#define LSM_CTRL3_VAL           0x44

/* Scale factors */
#define LSM_ACCEL_SCALE         (8.0f / 32768.0f * 9.80665f)   /* ±8g -> m/s^2 */
#define LSM_GYRO_SCALE          (2000.0f / 32768.0f * 0.01745329f) /* ±2000dps -> rad/s */

/* Read bit for SPI multi-byte */
#define LSM_SPI_READ            0x80

/* ---- Driver state ---- */
static SPI_HandleTypeDef hspi_imu;
static volatile int imu_ready = 0;

/* ---- CS pin control ---- */
static inline void CS_Low(void)
{
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
}

static inline void CS_High(void)
{
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
}

/* ---- Low-level SPI ---- */
static uint8_t SPI_TxRx(uint8_t tx)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi_imu, &tx, &rx, 1, 10);
    return rx;
}

static void LSM_WriteReg(uint8_t reg, uint8_t val)
{
    CS_Low();
    SPI_TxRx(reg & 0x7F);
    SPI_TxRx(val);
    CS_High();
}

static uint8_t LSM_ReadReg(uint8_t reg)
{
    uint8_t val;
    CS_Low();
    SPI_TxRx(reg | LSM_SPI_READ);
    val = SPI_TxRx(0xFF);
    CS_High();
    return val;
}

static void LSM_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    CS_Low();
    SPI_TxRx(reg | LSM_SPI_READ);
    for (i = 0; i < len; i++)
        buf[i] = SPI_TxRx(0xFF);
    CS_High();
}

/* ---- SPI3 hardware init ---- */
static void SPI3_Init(void)
{
    /* Enable clocks */
    IMU_SPI_GPIO_CLK_ENABLE();
    IMU_SPI_CLK_ENABLE();
    IMU_CS_GPIO_CLK_ENABLE();

    /* Configure SCK, MISO, MOSI pins */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = IMU_SPI_SCK_PIN | IMU_SPI_MISO_PIN | IMU_SPI_MOSI_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = IMU_SPI_AF;
    HAL_GPIO_Init(IMU_SPI_PORT, &gpio);

    /* Configure CS pin as output, default high */
    gpio.Pin   = IMU_CS_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = 0;
    HAL_GPIO_Init(IMU_CS_PORT, &gpio);
    CS_High();

    /* SPI3 config: Master, Mode0, 8-bit, MSB first, ~10MHz */
    hspi_imu.Instance               = IMU_SPI;
    hspi_imu.Init.Mode              = SPI_MODE_MASTER;
    hspi_imu.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi_imu.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi_imu.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi_imu.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi_imu.Init.NSS               = SPI_NSS_SOFT;
    hspi_imu.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* 84MHz/8 ≈ 10.5MHz */
    hspi_imu.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi_imu.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi_imu.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi_imu);
}

/* ---- Public API ---- */

void BSP_IMU_Init(void)
{
    uint8_t id;

    SPI3_Init();
    BSP_Delay(50);

    /* Software reset */
    LSM_WriteReg(LSM_REG_CTRL3, 0x01);
    BSP_Delay(50);

    /* Verify WHO_AM_I */
    id = LSM_ReadReg(LSM_REG_WHO_AM_I);
    if (id != LSM_WHO_AM_I_VAL)
    {
        imu_ready = 0;
        return;
    }

    /* Configure: BDU=1, IF_INC=1 */
    LSM_WriteReg(LSM_REG_CTRL3, LSM_CTRL3_VAL);
    /* Accel: ±8g, 104Hz */
    LSM_WriteReg(LSM_REG_CTRL1, LSM_CTRL1_VAL);
    /* Gyro: ±2000dps, 104Hz */
    LSM_WriteReg(LSM_REG_CTRL2, LSM_CTRL2_VAL);

    BSP_Delay(20);
    imu_ready = 1;
}

int BSP_IMU_IsReady(void)
{
    return imu_ready;
}

int BSP_IMU_ReadData(BSP_IMU_Data_t *data)
{
    uint8_t buf[12];
    int16_t raw[6];
    uint8_t i;

    if (!imu_ready || data == (void *)0)
        return -1;

    /* Read 6 bytes from gyro (0x22..0x27) + 6 bytes from accel (0x28..0x2D)
       Using auto-increment, read 12 bytes starting from 0x22 */
    LSM_ReadRegs(0x22, buf, 12);

    /* Gyro: bytes 0..5, Accel: bytes 6..11 (little-endian) */
    for (i = 0; i < 6; i++)
        raw[i] = (int16_t)(buf[i * 2] | (buf[i * 2 + 1] << 8));

    /* Gyro: X, Y, Z */
    data->gyro[0]  = raw[0] * LSM_GYRO_SCALE;
    data->gyro[1]  = raw[1] * LSM_GYRO_SCALE;
    data->gyro[2]  = raw[2] * LSM_GYRO_SCALE;

    /* Accel: X, Y, Z */
    data->accel[0] = raw[3] * LSM_ACCEL_SCALE;
    data->accel[1] = raw[4] * LSM_ACCEL_SCALE;
    data->accel[2] = raw[5] * LSM_ACCEL_SCALE;

    return 0;
}
