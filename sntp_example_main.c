/* LwIP SNTP example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"
#include "esp_sntp.h"
#include <ds3231.h>

#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ultrasonic.h>
#include <esp_err.h>

#include <stdlib.h>
#include <freertos/event_groups.h> //Event groups
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "User_Variables.h"
#include "sdkconfig.h"

#define MAX_DISTANCE_CM 400
#define TRIGGER_GPIO 5
#define ECHO_GPIO 18


void ds3231(void *pvParameters);
static void obtain_time(void);
static void initialize_sntp(void);

void ultrasonic_test(void *pvParameters);

void mpu(void *xyz);
void mpu6050_init();
void mpu6050_read(void *pvParams);



void app_main()
{
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(ds3231, "time", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(mpu, "mpuval", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    vTaskDelete(NULL);
}


/**
 * @brief function to obtain distance from the ultrasonic sensor. The distance readings are printed out after taking
 * an average of 10 readings to remove sudden unwanted inaccurate readings.
*/

void ultrasonic_test(void *pvParameters)
{   static const char *TAG = "ultra";
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO};

    ultrasonic_init(&sensor);

    while (true)
    {
        float distance;
        float distavg;
        for (int i=0; i<10; i++) {
            esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
            distavg+= distance;
        }
        distavg= distavg/10;
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
            ESP_LOGI(TAG, "Distance: %0.04f cm\n", distavg * 100);

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/**
 * @brief main function to print time from ds3231 every second. It first initializes the ds3231, then calls obtain_time function
 * to get time from ntp server. Sets this time on the ds3231, after which the time is read from here and printed out.
*/

void ds3231(void *pvParameters)
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
    ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
    obtain_time();
    // update 'now' variable with current time
    time(&now);
    localtime_r(&now, &timeinfo);

    ESP_ERROR_CHECK(ds3231_set_time(&dev, &timeinfo));

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

/**
 * @brief function to obtain time from ntp server. Does the following steps for that: initializes nvs partition, netif for tcp/ip and
 * creates event loop for WiFi. Then connects to WiFi, calls function to initialize sntp.
*/

static void obtain_time(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // default event loop created

    ESP_ERROR_CHECK(example_connect()); // function to configure Wifi through menuconfig

    initialize_sntp();
    ESP_ERROR_CHECK(example_disconnect());
}

/**
 * @brief function to initialize sntp. It configures the ntp server and updates time from it.
*/

static void initialize_sntp(void)
{
    const char *TAG = "SNTP time";
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);
    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK)
    {
        ESP_LOGE("init sntp err", "Failed to update system time within 10s timeout");
    }
}


/**
 * @brief function to obtain pitch and roll from mpu. It first calls function to initialize mpu, then prints pitch and roll values.
 * 
*/


void mpu(void *xyz)
{
    static const char *TAGmpu = "mpu";
    mpu6050_init();
    while(1){
        ESP_LOGI(TAGmpu, "Pitch: %f Roll: %f", complimentary_angle.pitch, complimentary_angle.roll);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}


// function to initialize mpu
void mpu6050_init()
{

    char *TAG = "MPU6050 Init";

    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.scl_io_num = 22;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0));

    mpu6050_dev = mpu6050_create(I2C_NUM_1, 0x69);
    if (mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS) != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU 6050 Config failed.");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    if (mpu6050_wake_up(mpu6050_dev) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to wake up mpu6050");
    }

    //  Initialise timer to read MPU 6050, for complimentary angles (pitch and roll)
    //  we need to have quick reads (2.5ms) freeRTOS task resolution is 10ms so we use
    //  esp timer instead.

    const esp_timer_create_args_t cal_timer_config = 
    {
        .callback = mpu6050_read,
        .arg = NULL,
        .name = "MPU 6050 Timer",
        .skip_unhandled_events = true,
        .dispatch_method = ESP_TIMER_TASK
    };

    esp_timer_handle_t cal_timer;
    esp_timer_create(&cal_timer_config, &cal_timer);
    esp_timer_start_periodic(cal_timer, 2500); // 2.5ms

    return;
}

void mpu6050_read(void *pvParameters)       // get data from mpu
{
    mpu6050_get_acce(mpu6050_dev, &acce);                                          // update global acce type with current value
    mpu6050_get_gyro(mpu6050_dev, &gyro);                                          // update global gyro type with current value
    mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle); // update global complimentary angle value with current value
    return;
}