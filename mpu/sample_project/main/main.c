#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/event_groups.h> //Event groups
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "User_Variables.h"

void mpu6050_init();
void mpu6050_read(void *pvParams);

void app_main(void)
{
    //char *TAG = "Main";

    // if (i2cdev_init() == ESP_OK)
    // {
    //     ESP_LOGI(TAG, "Initialised i2c mod.");
    // }
    mpu6050_init();
    while(1){
    printf("Pitch: %f", complimentary_angle.pitch);
    printf(" Roll: %f \n", complimentary_angle.roll);
    vTaskDelay(pdMS_TO_TICKS(500));
    }
}



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
void mpu6050_read(void *pvParameters)
{
    //char *TAG = "MPU6050 readings";
    mpu6050_get_acce(mpu6050_dev, &acce);                                          // update global acce type with current value
    mpu6050_get_gyro(mpu6050_dev, &gyro);                                          // update global gyro type with current value
    mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle); // update global complimentary angle value with current value

    return;
}
