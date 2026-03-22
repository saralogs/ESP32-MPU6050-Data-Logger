#ifndef LOGGER_H
#define LOGGER_H

#include "imu_processing.h"

/* Initialize UART logging (if needed beyond printf) */
void logger_init(void);

/* Log one line of processed data */
void log_data(const imu_processed_data_t *data);

#endif // LOGGER_H
