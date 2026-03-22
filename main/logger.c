#include "logger.h"
#include "esp_log.h"   // still needed for initialization message

static const char *TAG = "LOGGER";

void logger_init(void)
{
    // Optional: any UART initialization (already done by ESP-IDF)
    ESP_LOGI(TAG, "Logger ready (printf mode)");
}

void log_data(const imu_processed_data_t *data)
{
    // No extra metadata, just clean CSV format
    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
           data->accel_x, data->accel_y, data->accel_z,
           data->temp,
           data->gyro_x, data->gyro_y, data->gyro_z);
}
