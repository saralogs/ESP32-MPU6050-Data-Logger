#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_driver.h"
#include "imu_processing.h"
#include "logger.h"
#include "esp_log.h"

#define SAMPLE_RATE_HZ  100
#define TASK_DELAY_MS   (1000 / SAMPLE_RATE_HZ)   // 10 ms

// Retry parameters
#define READ_RETRIES     5
#define RETRY_DELAY_MS   2          // short delay between retries (may cause missed deadlines)
#define ERROR_THRESHOLD  10         // consecutive failures before re‑init
#define COOLDOWN_CYCLES  5          // number of cycles to skip output after re‑init

static const char *TAG = "imu_task";

void imu_task(void *pvParameters)
{
    imu_raw_data_t raw;
    imu_processed_data_t proc;
    TickType_t last_wake_time = xTaskGetTickCount();
    int consecutive_errors = 0;
    int cooldown = 0;                // cycles to skip output after re‑init

    while (1) {
        // ---- Retry loop for reading ----
        esp_err_t ret = ESP_FAIL;
        for (int attempt = 0; attempt < READ_RETRIES; attempt++) {
            ret = imu_read_raw(&raw);
            if (ret == ESP_OK) {
                consecutive_errors = 0;   // success: reset error counter
                break;
            }
            if (attempt < READ_RETRIES - 1) {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            }
        }

        // ---- Handle persistent errors ----
        if (ret != ESP_OK) {
            consecutive_errors++;
            ESP_LOGE(TAG, "IMU read failed (attempt %d), error count = %d",
                     READ_RETRIES, consecutive_errors);

            // If error threshold reached, re‑initialize the sensor
            if (consecutive_errors >= ERROR_THRESHOLD) {
                ESP_LOGW(TAG, "Too many errors, re‑initializing MPU6050...");
                if (imu_init() == ESP_OK) {
                    ESP_LOGI(TAG, "Re‑initialization successful");
                    consecutive_errors = 0;
                    cooldown = COOLDOWN_CYCLES;   // skip output for a few cycles
                } else {
                    ESP_LOGE(TAG, "Re‑initialization failed, will retry later");
                }
            }
        } else {
            // ---- No error: convert, filter, and output ----
            if (cooldown > 0) {
                // Still in cooldown period: skip logging this cycle
                cooldown--;
            } else {
                imu_convert_raw(&raw, &proc);
                imu_filter_data(&proc);
                log_data(&proc);      // uses printf (clean CSV)
            }
        }

        // ---- Maintain 100 Hz sampling rate ----
        // If the retry loop caused a deadline miss, this will adjust the next wake time
        // to the original schedule. Missed cycles are acceptable as long as they are rare.
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_DELAY_MS));
    }
}
