#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ds3231.h>
#include <string.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_sntp.h"
#include "protocol_examples_common.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"

static void obtain_time(void);
static void initialize_sntp(void);
static void ds3231_test(void *pvParam);

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(ds3231_test, "ds3231_test", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void ds3231_test(void *pvParameters)
{
    const char *TAG = "SNTP time";
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));
    time_t now;
    struct tm timeinfo = {0};

    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, 21, 22));

    setenv("TZ", "IST-5:30", 1);
    tzset();
    time(&now);
    localtime_r(&now, &timeinfo);
    // if (ds3231_get_time(&dev, &timeinfo) != ESP_OK)
    // {
    //     ESP_LOGI("error", "could not get time\n");
    // }
    // ESP_LOGI(TAG, "(RTC)seconds count before going to if condition: %lld", now);
    // ESP_LOGI(TAG, "(RTC)time before going to if condition: %02d-%02d-%04d  %02d:%02d:%02d", timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_year, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    // if (timeinfo.tm_year < (2016 - 1900))
    //{
    ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
    obtain_time();
    // update 'now' variable with current time
    time(&now);
    localtime_r(&now, &timeinfo);
    // }

    // ESP_LOGI(TAG, "seconds count: %lld", now);
    // ESP_LOGI(TAG, "current time before setting on DS3231: %02d-%02d-%04d  %02d:%02d:%02d", timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_year, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    ESP_ERROR_CHECK(ds3231_set_time(&dev, &timeinfo));
    // if (ds3231_get_time(&dev, &timeinfo) != ESP_OK)
    // {
    //     ESP_LOGI("error", "could not get time\n");
    // }
    // ESP_LOGI(TAG, "After getting time from NTP, rtc reads, seconds count: %lld", now);
    // ESP_LOGI(TAG, "After getting time from NTP, rtc reads, current time: %02d-%02d-%04d  %02d:%02d:%02d", timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_year, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));

        if (ds3231_get_time(&dev, &timeinfo) != ESP_OK)
        {
            ESP_LOGI("error", "could not get time\n");
            continue;
        }
        ESP_LOGI(TAG, "The current date/time is: %02d-%02d-%04d  %02d:%02d:%02d", timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_year + 1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    }
}

static void obtain_time(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // default event loop created

    ESP_ERROR_CHECK(example_connect()); // function to configure Wifi through menuconfig

    initialize_sntp();
    ESP_ERROR_CHECK(example_disconnect());
}

static void initialize_sntp(void)
{
    const char *TAG = "SNTP time";
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);
    // esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    // esp_sntp_setservername(0, "pool.ntp.org");
    // sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    // esp_sntp_init();
    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK)
    {
        ESP_LOGE("init sntp err", "Failed to update system time within 10s timeout");
    }
}
