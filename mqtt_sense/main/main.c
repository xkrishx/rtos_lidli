/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include <esp_wifi.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>

#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_attr.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"

#include <stdlib.h>
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "sdkconfig.h"

#include <stdbool.h>
#include <ultrasonic.h>
#include <esp_err.h>
#include "protobuf/proto-c/custom_config.pb-c.h"
#include "driver/gpio.h"

#define MAX_DISTANCE_CM 400
#define TRIGGER_GPIO 5
#define ECHO_GPIO 18
#define DISTANCE_READINGS_COUNT 10
#define ULTRA_DELAY 100
const int retry_count = 15;
static mpu6050_handle_t mpu6050_dev = NULL;
// Global sensor values
static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;
static complimentary_angle_t complimentary_angle;
bool MPU_FAIL = false;

esp_mqtt_client_handle_t client;
EventGroupHandle_t wifi_event_group;
bool provisioned;
bool wificonnected;
const int WIFI_CONNECTED_EVENT = BIT0;
const int WIFI_PROV_TIMEOUT_EVENT = BIT1;
const int WIFI_PROV_DONE = BIT2;
float distance;
float distavg;
esp_err_t res;

static const char *TAG = "MQTT";
char strftime_buf[64];

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void event_handler(void *arg, esp_event_base_t event_base,
                   int32_t event_id, void *event_data);
static void mqtt5_app_start(void);
static void obtain_time(void);
void ultrasonic(void *pvParameters);
void ntp_time();
void init_wifi(void);
void mpu(void *xyz);
void mpu6050_read(void *pvParameters);
void mpu6050_init();

void mqtt_all_data();
void gpio_boot(void *pv);
void gpio_init();

void app_main(void)
{

    gpio_init();
    init_wifi();
    mpu6050_init();
    // xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, pdTRUE, pdFAIL, portMAX_DELAY);
    ESP_LOGI(TAG, "Initialising MQTT from Main.");
    mqtt5_app_start();
    gpio_set_level(GPIO_NUM_2, 1);
    xTaskCreate(ultrasonic, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    // xTaskCreate(mpu, "mpuval", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(mqtt_all_data, "mqtt", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreate(gpio_boot, "boot", configMINIMAL_STACK_SIZE, NULL, 10, NULL);
    ESP_LOGI(TAG, "Initialization completed.");

    vTaskDelete(NULL);
}

void gpio_boot(void *pv)
{

    while (1)
    {
        if (gpio_get_level(GPIO_NUM_0) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(3000));
            if (gpio_get_level(GPIO_NUM_0) == 00)
            {

                ESP_ERROR_CHECK(nvs_flash_erase());

                esp_restart();
            }
        }
    }
}

void gpio_init(void)
{
    gpio_config_t boot_button = {
        .pin_bit_mask = BIT64(GPIO_NUM_0),
        .mode = GPIO_MODE_INPUT,
        //.pull_up_en = GPIO_PULLUP_ENABLE,

    };
    gpio_config(&boot_button);

    gpio_config_t led_pin = {
        .pin_bit_mask = BIT64(GPIO_NUM_2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led_pin);
}

void ultrasonic(void *pvParameters)
{
    static const char *TAG = "ultra";
    bool read_fail = false;

    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO};

    ultrasonic_init(&sensor);

    while (true)
    {
        for (int i = 0; i < DISTANCE_READINGS_COUNT; i++)
        {
            res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
            if (res != ESP_OK)
            {
                read_fail = true;
                break;
            }
            distavg += distance;
        }

        if (read_fail)
        {
            read_fail = false;
            switch (res)
            {
            case ESP_ERR_ULTRASONIC_PING:
                ESP_LOGI(TAG, "Cannot ping (device is in invalid state)\n");
                distavg = -1;
                break;
            case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                ESP_LOGI(TAG, "Ping timeout (no device found)\n");
                distavg = -1;
                break;
            case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                ESP_LOGI(TAG, "Echo timeout (i.e. distance too big)\n");
                distavg = MAX_DISTANCE_CM;
                break;
            default:
                ESP_LOGI(TAG, "%s\n", esp_err_to_name(res));
            }
        }

        distavg = distavg / DISTANCE_READINGS_COUNT;

        // ESP_LOGI(TAG, "Distance: %0.04f m\n", distavg);
        vTaskDelay(pdMS_TO_TICKS(ULTRA_DELAY));
        distavg = 0;
    }
}

void ntp_time()
{

    time_t now;
    struct tm timeinfo;
    // char strftime_buf[64];
    // time(&now);
    // localtime_r(&now, &timeinfo);
    obtain_time();
    // update 'now' variable with current time
    time(&now);
    localtime_r(&now, &timeinfo);
    // setenv("TZ", "IST-5:30", 1);
    // tzset();
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
    return;
}

static void obtain_time(void)
{
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org"); // configures to one default ntp server
    esp_netif_sntp_init(&config);                                             // initialize sntp
    int retry = 0;

    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    // time(&now);
    // localtime_r(&now, &timeinfo);
    return;
}

void mpu(void *xyz)
{
    static const char *TAGmpu = "mpu";

    while (1)
    {
        ESP_LOGI(TAGmpu, "Pitch: %f Roll: %f", complimentary_angle.pitch, complimentary_angle.roll);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void mpu6050_init()
{

    char *TAG = "MPU6050 Init";

    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 19;
    conf.scl_io_num = 23;
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
            .dispatch_method = ESP_TIMER_TASK};

    esp_timer_handle_t cal_timer;
    esp_timer_create(&cal_timer_config, &cal_timer);
    esp_timer_start_periodic(cal_timer, 2500); // 2.5ms

    return;
}

void mpu6050_read(void *pvParameters) // get data from mpu
{
    mpu6050_get_acce(mpu6050_dev, &acce);                                          // update global acce type with current value
    mpu6050_get_gyro(mpu6050_dev, &gyro);                                          // update global gyro type with current value
    mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle); // update global complimentary angle value with current value
    return;
}

static void mqtt5_app_start(void)
{
    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = "mqtt://mqtt.eclipseprojects.io",
        .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,
        .network.disable_auto_reconnect = true,
    };

    client = esp_mqtt_client_init(&mqtt5_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void mqtt_all_data()
{
    while (1)
    {
        char dist_buff[10];
        sprintf(dist_buff, "%.4f", distavg);
        char pitch_buff[10];
        sprintf(pitch_buff, "%f", complimentary_angle.pitch);
        char roll_buff[10];
        sprintf(roll_buff, "%f", complimentary_angle.roll);
        time_t now;
        time(&now);
        char time_buff[32];
        sprintf(time_buff, "%lld", now);
        ESP_LOGI(TAG, "time: %lld", now);
        ESP_LOGI(TAG, "distance: %.4f", distavg);
        ESP_LOGI(TAG, "pitch: %f  roll: %f", complimentary_angle.pitch, complimentary_angle.roll);
        esp_mqtt_client_publish(client, "krishnapc/LIPL/Time", time_buff, 0, 1, 0);
        // esp_mqtt_client_publish(client, "krishnapc/LIPL/Pitch", "5.65 deg.", 0, 1, 0);
        // esp_mqtt_client_publish(client, "krishnapc/LIPL/Roll", "-3.24 deg.", 0, 1, 0);
        esp_mqtt_client_publish(client, "krishnapc/LIPL/Distance", dist_buff, 0, 1, 0);
        esp_mqtt_client_publish(client, "krishnapc/LIPL/Pitch", pitch_buff, 0, 1, 0);
        esp_mqtt_client_publish(client, "krishnapc/LIPL/Roll", roll_buff, 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{

    int msg_id;
    esp_mqtt_event_handle_t event = event_data; // here esp_mqtt_event_handle_t is a struct which receieves struct event from mqtt app start funtion
    // esp_mqtt_client_handle_t client = event->client; // making obj client of struct esp_mqtt_client_handle_t and assigning it the receieved event client

    if (event->event_id == MQTT_EVENT_CONNECTED)
    {
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        // msg_id = esp_mqtt_client_subscribe(client, "/krishnapc/LIPL/#", 0);
        //  in mqtt we require a topic to subscribe and client is from event client and 0 is quality of service it can be 1 or 2
        // ESP_LOGI(TAG, "sent subscribe successful");

        esp_mqtt_client_subscribe(client, "krishnapc/LIPL/Data", 1);

        // esp_mqtt_client_subscribe(client, "/krishnapc/LIPL/Time", 0);
    }
    else if (event->event_id == MQTT_EVENT_DISCONNECTED)
    {
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED"); // if disconnected
    }
    else if (event->event_id == MQTT_EVENT_SUBSCRIBED)
    {
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
    }
    else if (event->event_id == MQTT_EVENT_UNSUBSCRIBED) // when subscribed
    {
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
    }
    else if (event->event_id == MQTT_EVENT_DATA) // when unsubscribed
    {
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // Printing topic

        // printing data
        int datalen = event->data_len;
        ESP_LOGI(TAG, "%.*s", datalen, event->data);
    }
    else if (event->event_id == MQTT_EVENT_ERROR) // when any error
    {
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    }
}

/**
 * @brief Initialises WiFi and Provisioning system.
 *
 */
void init_wifi()
{
    char *TAG = "init_wifi";
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    esp_netif_create_default_wifi_sta();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // This config requires NVS initialized
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_prov_mgr_config_t config = {
        /* What is the Provisioning Scheme that we want ?
         * wifi_prov_scheme_softap or wifi_prov_scheme_ble */
        .scheme = wifi_prov_scheme_ble,

        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM};
    /* Initialize provisioning manager with the
     * configuration parameters set above */
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    /* Let's find out if the device is provisioned */
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));
    // esp_wifi_get_mac(WIFI_IF_STA, mac_addr); // Get WiFi Mac Address

    /* If device is not yet provisioned start provisioning service */
    if (!provisioned)
    {
        ESP_LOGI(TAG, "Starting provisioning");
        /* What is the Device Service Name that we want
         * This translates to :
         *     - Wi-Fi SSID when scheme is wifi_prov_scheme_softap
         *     - device name when scheme is wifi_prov_scheme_ble
         */
        char service_name[] = "KPCs Sensor System";
        const char *pop = "LIPL24";
        const char *service_key = NULL;
        wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
        /* This step is only useful when scheme is wifi_prov_scheme_ble. This will
         * set a custom 128 bit UUID which will be included in the BLE advertisement
         * and will correspond to the primary GATT service that provides provisioning
         * endpoints as GATT characteristics. Each GATT characteristic will be
         * formed using the primary service UUID as base, with different auto assigned
         * 12th and 13th bytes (assume counting starts from 0th byte). The client side
         * applications must identify the endpoints by reading the User Characteristic
         * Description descriptor (0x2901) for each characteristic, which contains the
         * endpoint name of the characteristic */
        uint8_t custom_service_uuid[] = {
            /* LSB <---------------------------------------
             * ---------------------------------------> MSB */
            0xb4,
            0xdf,
            0x5a,
            0x1c,
            0x3f,
            0x6b,
            0xf4,
            0xbf,
            0xea,
            0x4a,
            0x82,
            0x03,
            0x04,
            0x90,
            0x1a,
            0x02,
        };
        wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

        // wifi_prov_mgr_endpoint_create("custom-config"); // Keep this above wifi_prov_start_provisioning()
        // wifi_prov_mgr_endpoint_create("ble-communication");

        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, pop, service_name, service_key));
        // wifi_prov_mgr_endpoint_register("custom-config", custom_prov_config_data_handler, NULL);
        // wifi_prov_mgr_endpoint_register("ble-communication", ble_command_data_handler, NULL);
    }
    else
    {
        ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");
        /* We don't need the manager as device is already provisioned,
         * so let's release it's resources */
        wifi_prov_mgr_deinit();

        /* Start Wi-Fi station */
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        xEventGroupSetBits(wifi_event_group, WIFI_PROV_DONE); // let app_main() know that wifi prov is ok
    }
    /* Initialise SNTP */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, pdTRUE, pdFAIL, portMAX_DELAY);
    ntp_time();
    return;
}

/**
 * @brief WiFi and WiFi Provisioning Event handler.
 *
 * @param arg           Void ptr
 * @param event_base    Event Base type
 * @param event_id      Event ID
 * @param event_data    Data (void ptr)
 */
void event_handler(void *arg, esp_event_base_t event_base,
                   int32_t event_id, void *event_data)
{
    char *TAG = "event_handler";
    if (event_base == WIFI_PROV_EVENT)
    {
        switch (event_id)
        {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "Provisioning started");
            break;
        case WIFI_PROV_CRED_RECV:
        {
            wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
            ESP_LOGI(TAG, "Received Wi-Fi credentials"
                          "\n\tSSID     : %s\n\tPassword : %s",
                     (const char *)wifi_sta_cfg->ssid,
                     (const char *)wifi_sta_cfg->password);
            break;
        }
        case WIFI_PROV_CRED_FAIL:
        {
            wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
            ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                          "\n\tPlease reset to factory and retry provisioning",
                     (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
            ESP_ERROR_CHECK(nvs_flash_erase());
            esp_restart();
            break;
        }
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "Provisioning successful");
            xEventGroupSetBits(wifi_event_group, WIFI_PROV_DONE); // let app_main() know that wifi prov is ok
            break;
        case WIFI_PROV_END:
            /* De-initialize manager once provisioning is finished */
            wifi_prov_mgr_deinit();
            break;
        default:
            break;
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {

        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        wificonnected = true;
        /* Signal main application to continue execution */

        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {

        ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
        wificonnected = false;
        esp_wifi_connect();
    }
}