#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>
#include "esp_err.h"

/* MPU6050 registers */
#define MPU6050_ADDR         0x68        // AD0 low
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_CONFIG       0x1A
#define MPU6050_ACCEL_XOUT_H 0x3B

/* Expected WHO_AM_I value */
#define MPU6050_WHO_AM_I_VAL 0x68

/* Raw data structure */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} imu_raw_data_t;

/* Initialize I2C and MPU6050 */
esp_err_t imu_init(void);

/* Read raw data from sensor (burst read) */
esp_err_t imu_read_raw(imu_raw_data_t *raw);

#endif // IMU_DRIVER_H