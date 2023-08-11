//#ifndef OUTPUT_H
//#define OUTPUT_H
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "../../../../../esp-idf-v4.4.4/components/esp_adc_cal/include/esp_adc_cal.h"
#include "esp_err.h"


#define ID_MQ7  0X02
#define ESP_ERROR_MQ7_INIT_FAILED ((ID_MQ7 << 12)|(0x00))
#define ESP_ERROR_MQ7_READ_DATA_FAILED ((ID_MQ7 << 12)|(0x01))

esp_err_t mq7_init();
esp_err_t mq7_reading(uint32_t *co);
//#endif