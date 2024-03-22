#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ultrasonic.h>
#include <esp_err.h>
#include "esp_log.h"

#define MAX_DISTANCE_CM 100 // 5m max
static const char *TAG = "ultra";
#define TRIGGER_GPIO 5
#define ECHO_GPIO 18

void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO};

    ultrasonic_init(&sensor);

    while (true)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            ESP_LOGI(TAG, "Error %d: ", res);
            switch (res)
            {
            case ESP_ERR_ULTRASONIC_PING:
                ESP_LOGI(TAG, "Cannot ping (device is in invalid state)\n");
                break;
            case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                ESP_LOGI(TAG, "Ping timeout (no device found)\n");
                break;
            case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                ESP_LOGI(TAG, "Echo timeout (i.e. distance too big)\n");
                break;
            default:
                ESP_LOGI(TAG, "%s\n", esp_err_to_name(res));
            }
        }
        else
            ESP_LOGI(TAG, "Distance: %0.04f cm\n", distance * 100);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
}