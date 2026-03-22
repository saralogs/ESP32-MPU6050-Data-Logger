#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <stdint.h>
#include "imu_driver.h"

/* Scale factors for conversion (now in processing layer) */
#define ACCEL_SCALE_2G       16384.0f   // LSB/g
#define GYRO_SCALE_250DPS    131.0f     // LSB/°/s
#define TEMP_SCALE           340.0f     // LSB/°C
#define TEMP_OFFSET          36.53f     // °C

/* Processed data in physical units */
typedef struct {
    float accel_x;      // g
    float accel_y;      // g
    float accel_z;      // g
    float temp;         // °C
    float gyro_x;       // °/s
    float gyro_y;       // °/s
    float gyro_z;       // °/s
} imu_processed_data_t;

/* Convert raw data to physical units */
void imu_convert_raw(const imu_raw_data_t *raw, imu_processed_data_t *proc);

/* Optional: simple moving average filter (enable by defining FILTER_ORDER) */
void imu_filter_data(imu_processed_data_t *proc);

#endif // IMU_PROCESSING_H
