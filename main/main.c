#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "imu_driver.h"
#include "logger.h"

void imu_task(void *pvParameters);

void app_main(void)
{
    ESP_LOGI("MAIN", "Starting MPU6050 logger");

    // Initialize logger (UART)
    logger_init();

    // Initialize IMU (I2C + sensor config)
    esp_err_t ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "IMU init failed, restarting in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }

    // Create the sampling task
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 5, NULL, tskNO_AFFINITY);

    // The main task can do other things or just terminate
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI("MAIN", "IMU task created, system ready");
}
