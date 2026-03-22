#include "imu_processing.h"
#include <string.h>

//#define FILTER_ORDER 4   // uncomment to enable 4‑tap moving average

#ifdef FILTER_ORDER
static float accel_x_buf[FILTER_ORDER] = {0};
static float accel_y_buf[FILTER_ORDER] = {0};
static float accel_z_buf[FILTER_ORDER] = {0};
static float gyro_x_buf[FILTER_ORDER] = {0};
static float gyro_y_buf[FILTER_ORDER] = {0};
static float gyro_z_buf[FILTER_ORDER] = {0};
static int filter_idx = 0;
#endif

void imu_convert_raw(const imu_raw_data_t *raw, imu_processed_data_t *proc)
{
    proc->accel_x = (float)raw->accel_x / ACCEL_SCALE_2G;
    proc->accel_y = (float)raw->accel_y / ACCEL_SCALE_2G;
    proc->accel_z = (float)raw->accel_z / ACCEL_SCALE_2G;
    proc->temp    = (float)raw->temp / TEMP_SCALE + TEMP_OFFSET;
    proc->gyro_x  = (float)raw->gyro_x / GYRO_SCALE_250DPS;
    proc->gyro_y  = (float)raw->gyro_y / GYRO_SCALE_250DPS;
    proc->gyro_z  = (float)raw->gyro_z / GYRO_SCALE_250DPS;
}

#ifdef FILTER_ORDER
void imu_filter_data(imu_processed_data_t *proc)
{
    // Insert new samples into buffers
    accel_x_buf[filter_idx] = proc->accel_x;
    accel_y_buf[filter_idx] = proc->accel_y;
    accel_z_buf[filter_idx] = proc->accel_z;
    gyro_x_buf[filter_idx]  = proc->gyro_x;
    gyro_y_buf[filter_idx]  = proc->gyro_y;
    gyro_z_buf[filter_idx]  = proc->gyro_z;

    filter_idx = (filter_idx + 1) % FILTER_ORDER;

    // Compute averages
    float sum_accel_x = 0, sum_accel_y = 0, sum_accel_z = 0;
    float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
    for (int i = 0; i < FILTER_ORDER; i++) {
        sum_accel_x += accel_x_buf[i];
        sum_accel_y += accel_y_buf[i];
        sum_accel_z += accel_z_buf[i];
        sum_gyro_x  += gyro_x_buf[i];
        sum_gyro_y  += gyro_y_buf[i];
        sum_gyro_z  += gyro_z_buf[i];
    }

    proc->accel_x = sum_accel_x / FILTER_ORDER;
    proc->accel_y = sum_accel_y / FILTER_ORDER;
    proc->accel_z = sum_accel_z / FILTER_ORDER;
    proc->gyro_x  = sum_gyro_x  / FILTER_ORDER;
    proc->gyro_y  = sum_gyro_y  / FILTER_ORDER;
    proc->gyro_z  = sum_gyro_z  / FILTER_ORDER;
}
#else
void imu_filter_data(imu_processed_data_t *proc)
{
    // No filtering, just pass through
    (void)proc;
}
#endif
