// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "esp_event.h"
// #include "esp_netif.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"

#include "esp_http_client.h"
#include "connect_wifi.h"
// #include "bme280.h"
// #include "driver/i2c.h"
// #include "driver/gpio.h"
// //#include "http.c"
// #include "pms7003.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_attr.h"
#include "esp_spi_flash.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>

#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "../components/mq7/mq7.h"
// #include "../components/mq2/mq2.h"
#include "bme280.h"
// #include "sdcard.h"
#include "pms7003.h"
#include "../components/MHZ14/mhz14a.h"

#define WAIT_10_TICK (TickType_t)(10 / portTICK_RATE_MS)
bmp280_t bme280_device;
bmp280_params_t bme280_params;
// #define PERIOD_GET_DATA_FROM_SENSOR                 (TickType_t)(3000 / portTICK_RATE_MS)
#define PERIOD_GET_DATA_FROM_SENSOR (TickType_t)(5000 / portTICK_RATE_MS)
#define PERIOD_SOUND (TickType_t)(100 / portTICK_RATE_MS)
uart_config_t pms_uart_config = UART_CONFIG_DEFAULT();
uart_config_t mhz_uart_config = MHZ14A_UART_CONFIG_DEFAULT();
struct dataSensor_st
{
    float temperature;
    float pressure;
    float humidity;
    uint32_t pm1_0;
    uint32_t pm2_5;
    uint32_t pm10;
    uint32_t co2;
    // uint32_t gas;
    float probability;
};
struct dataSensor_st dataFromSensor;

#define QUEUE_SIZE 10U
#define LEDC_TIMER_BIT_NUM      10
#define LEDC_BASE_FREQ          700
#define LEDC_CHANNEL_NUM        1
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RESOLUTION    LEDC_TIMER_BIT_NUM
#define GPIO_OUTPUT_SPEED LEDC_HIGH_SPEED_MODE
#define duration 500

SemaphoreHandle_t speaker_semaphore = NULL;
QueueHandle_t dataSensorSentToHTTP_queue;

TaskHandle_t speaker_handle = NULL;
#define WEB_SERVER "api.thingspeak.com"
#define WEB_PORT "80"

static const char *TAG = "example";
char REQUEST[512];
char recv_buf[512];
char SUBREQUEST[150];

/**
 * @brief push data from queue to the thingspeak
 * 
*/
static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    // char recv_buf[64];

    while (1)
    {
        struct dataSensor_st dataSensorReceiveFromQueue;
        if (uxQueueMessagesWaiting(dataSensorSentToHTTP_queue) != 0)
        {
            if (xQueueReceive(dataSensorSentToHTTP_queue, (void *)&dataSensorReceiveFromQueue, portMAX_DELAY) == pdPASS)
            {
                int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

                if (err != 0 || res == NULL)
                {
                    ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    continue;
                }

                /* Code to print the resolved IP.

                    Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
                addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
                ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

                s = socket(res->ai_family, res->ai_socktype, 0);
                if (s < 0)
                {
                    ESP_LOGE(TAG, "... Failed to allocate socket.");
                    freeaddrinfo(res);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    continue;
                }
                ESP_LOGI(TAG, "... allocated socket");

                if (connect(s, res->ai_addr, res->ai_addrlen) != 0)
                {
                    ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
                    close(s);
                    freeaddrinfo(res);
                    vTaskDelay(4000 / portTICK_PERIOD_MS);
                    continue;
                }

                ESP_LOGI(TAG, "... connected");
                freeaddrinfo(res);
                sprintf(SUBREQUEST, "api_key=DO2CX4XEDTZX1VO8&field1=%.2f&field2=%.2f&field3=%d&field5=%u&field6=%u&field7=%u&field8=%.2f", dataSensorReceiveFromQueue.temperature, dataSensorReceiveFromQueue.humidity, dataSensorReceiveFromQueue.co2, dataSensorReceiveFromQueue.pm1_0, dataSensorReceiveFromQueue.pm2_5, dataSensorReceiveFromQueue.pm10, dataSensorReceiveFromQueue.probability);
                //printf("temp= %.2f,hum= %.2f,co2 = %d, pm1_0 = %u, pm2_5= %u, pm10= %u, probability = %.2f ", dataSensorReceiveFromQueue.temperature, dataSensorReceiveFromQueue.humidity, dataSensorReceiveFromQueue.co2, dataSensorReceiveFromQueue.pm1_0, dataSensorReceiveFromQueue.pm2_5, dataSensorReceiveFromQueue.pm10, dataSensorReceiveFromQueue.probability);
                sprintf(REQUEST, "POST /update HTTP/1.1\nHost: api.thingspeak.com\nConection: close\nContent-Type: application/x-www-form-urlencoded\nContent-Length:%d\n\n%s\n", strlen(SUBREQUEST), SUBREQUEST);
                if (write(s, REQUEST, strlen(REQUEST)) < 0)
                {
                    ESP_LOGE(TAG, "... socket send failed");
                    close(s);
                    vTaskDelay(4000 / portTICK_PERIOD_MS);
                    continue;
                }
                ESP_LOGI(TAG, "... socket send success");

                struct timeval receiving_timeout;
                receiving_timeout.tv_sec = 5;
                receiving_timeout.tv_usec = 0;
                if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                               sizeof(receiving_timeout)) < 0)
                {
                    ESP_LOGE(TAG, "... failed to set socket receiving timeout");
                    close(s);
                    vTaskDelay(4000 / portTICK_PERIOD_MS);
                    continue;
                }
                ESP_LOGI(TAG, "... set socket receiving timeout success");

                /* Read HTTP response */
                do
                {
                    bzero(recv_buf, sizeof(recv_buf));
                    r = read(s, recv_buf, sizeof(recv_buf) - 1);
                    for (int i = 0; i < r; i++)
                    {
                        putchar(recv_buf[i]);
                    }
                } while (r > 0);

                ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
                close(s);
                for (int countdown = 10; countdown >= 0; countdown--)
                {
                    ESP_LOGI(TAG, "%d... ", countdown);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                ESP_LOGI(TAG, "Starting again!");
            }
        }
    }
}



/**
 * @brief this task get data fromsensor and use DS-Evidence algorithm to calculate the probability from 4 sensor
 * @authors sang8022002@gmail.com
 * 
*/
void readDataFromSensor()
{
    for (;;)
    {
        TickType_t task_lastWakeTime;
        task_lastWakeTime = xTaskGetTickCount();
        // BME280
        bme280_readSensorData(&bme280_device, &(dataFromSensor.temperature),
                              &(dataFromSensor.pressure),
                              &(dataFromSensor.humidity));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        pms7003_readData(indoor, &(dataFromSensor.pm1_0), &(dataFromSensor.pm2_5), &(dataFromSensor.pm10));

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mhz14a_getDataFromSensorViaUART(&dataFromSensor.co2);
        ESP_LOGI(__func__, "temp= %.2f, hum = %.2f, pm10= %u, co2 = %u ", dataFromSensor.temperature, dataFromSensor.humidity, dataFromSensor.pm10, dataFromSensor.co2);
        float DS_fire;
        float DS_noFire;
        float temperatureProbability = (dataFromSensor.temperature - 20) / 65;
        if (temperatureProbability < 0)
        {
            temperatureProbability = 0;
        }
        ESP_LOGI(__func__, "Xac Suat temperature = %.2f ", temperatureProbability);
        float humidityProbability = (-dataFromSensor.humidity / 100) + 1;
        ESP_LOGI(__func__, "Xac Suat humidity = %.2f ", humidityProbability);
        float co2Probability;
        co2Probability = (float)(dataFromSensor.co2 - 400) / (5000 - 400);
        ESP_LOGI(__func__, "Nong Do co2 = %u ", dataFromSensor.co2);
        ESP_LOGW(__func__, "Xac Suat co2 = %.2f ", co2Probability);
        if (co2Probability < 0)
        {
            co2Probability = 0;
            ESP_LOGI(__func__, "bi gan bang 0");
        }
        ESP_LOGI(__func__, "Xac Suat co2 = %.2f ", co2Probability);
        float dustProbability;
        if (dataFromSensor.pm10 <= 1000)
        {
            dustProbability = (float)dataFromSensor.pm10 / 1000;
        }
        else
        {
            dustProbability = 1;
        }
        ESP_LOGI(__func__, "Xac Suat dust = %.2f ", dustProbability);
        DS_fire = (temperatureProbability * humidityProbability) / (1 - (1 - temperatureProbability) * humidityProbability - temperatureProbability * (1 - humidityProbability));
        DS_noFire = 1 - DS_fire;
        ESP_LOGI(__func__,"DS lan 1 = %.2f", DS_fire);
        DS_fire = (DS_fire * co2Probability) / (1 - (DS_noFire * co2Probability) - DS_fire * (1 - co2Probability));
        DS_noFire = 1 - DS_fire;
        ESP_LOGI(__func__,"DS lan 2 = %.2f", DS_fire);
        DS_fire = (DS_fire * dustProbability) / (1 - (DS_noFire * dustProbability) - DS_fire * (1 - dustProbability));
        DS_noFire = 1 - DS_fire;
        ESP_LOGI(__func__,"DS lan 3 = %.2f", DS_fire);
        dataFromSensor.probability = DS_fire;
        if (xQueueSendToBack(dataSensorSentToHTTP_queue, (void *)&dataFromSensor, WAIT_10_TICK * 5) != pdPASS)
        {
            //ESP_LOGE(__func__, "Failed to post the data sensor to dataSensorSentToHTTP Queue.");
        }
        else
        {
            ESP_LOGI(__func__, "Success to post the data sensor to dataSensorSentToHTTP Queue.");
        }
        if (dataFromSensor.co2 > 1000)
        {
            if (eTaskGetState(speaker_handle) == eSuspended)
            {
                vTaskResume(speaker_handle);
            }
        }
        vTaskDelayUntil(&task_lastWakeTime, PERIOD_GET_DATA_FROM_SENSOR);
    }
}

/**
 * @brief this task make the sound when there is a condition
 * @author sang8022002@gmail.com
 * 
*/
void speaker_task()
{
    while (1)
    {
        if (dataFromSensor.co2 >= 1000)
        {
            ledc_set_duty(GPIO_OUTPUT_SPEED, LEDC_CHANNEL_0, 0xFF); // 12% duty - play here for your speaker or buzzer 0x7f
            ledc_update_duty(GPIO_OUTPUT_SPEED, LEDC_CHANNEL_0);
            //ledc_set_duty_and_update(GPIO_OUTPUT_SPEED,LEDC_CHANNEL_0, 0x7F, 0x7F);
            vTaskDelay(duration/portTICK_PERIOD_MS);
            // stop
            ledc_set_duty(GPIO_OUTPUT_SPEED, LEDC_CHANNEL_0, 0);
            ledc_update_duty(GPIO_OUTPUT_SPEED, LEDC_CHANNEL_0);
            ESP_LOGW(__func__,"speaker doing");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            //vTaskDelayUntil(&task_lastWakeTime, PERIOD_SOUND);
        }
        else
        {
            vTaskSuspend(NULL);
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());
    ESP_LOGI(__func__, "Initialize BME280 sensor(I2C/Wire%d).", CONFIG_BME_I2C_PORT);

    ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_init(&bme280_device, &bme280_params, BME280_ADDRESS,
                                              CONFIG_BME_I2C_PORT, CONFIG_BME_PIN_NUM_SDA, CONFIG_BME_PIN_NUM_SCL));
    ESP_ERROR_CHECK_WITHOUT_ABORT(pms7003_initUart(&pms_uart_config));

    // uint32_t pm1p0_t, pm2p5_t, pm10_t;
    // pms7003_readData(indoor, &pm1p0_t, &pm2p5_t, &pm10_t); //!= ESP_OK;
    ESP_ERROR_CHECK_WITHOUT_ABORT(mhz14a_initUART(&mhz_uart_config));
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    dataSensorSentToHTTP_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    while (dataSensorSentToHTTP_queue == NULL)
    {
        ESP_LOGE(__func__, "Create dataSensorSentToHTTP Queue failed.");
        ESP_LOGI(__func__, "Retry to create dataSensorSentToHTTP Queue...");
        vTaskDelay(500 / portTICK_RATE_MS);
        dataSensorSentToHTTP_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    };
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RESOLUTION,
        .freq_hz = LEDC_BASE_FREQ,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = GPIO_NUM_4,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);
    //ledc_set_freq(GPIO_OUTPUT_SPEED, LEDC_TIMER_0, 700);
    connect_wifi();
    if (wifi_connect_status)
    {
        
        xTaskCreate(&http_get_task, "http_get_task", 8192, NULL, 5, NULL);
        
    }
    xTaskCreate(&speaker_task, "speaker_task", 4096, NULL, 6, &speaker_handle);
    xTaskCreate(&readDataFromSensor, "readDataFromSensor", 8192, NULL, 7, NULL);
}