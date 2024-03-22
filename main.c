// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
// What ra
// FreeRTOS dependancies
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/event_groups.h> //Event groups
#include "freertos/timers.h"

// ESP SDK and HAL
#include "driver/gpio.h"
// #include "driver/rmt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
// #include "ir_tools.h"
// #include "driver/rmt.h"
// #include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include <esp_wifi.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>
#include <sys/time.h>
#include <time.h>
#include "esp_sntp.h"      //Network Time Protocol
#include "esp_spi_flash.h" // <---- SPI flash control (NVS dependency)
#include "esp_littlefs.h"
#include "dirent.h"   //Directory Access
#include "sys/stat.h" //File system statistics api
#include "esp_check.h"
#include "esp_http_client.h" //HTTP client requirements
#include "mqtt_client.h"
#include "driver/uart.h"
#include "ds3231.h"
#include "cJSON.h"
#include "mpu6050.h"
#include "ultrasonic.h"
// Custom user includes
#include "pin_defs.h"
#include "Network_definitions.h"
#include "User_Variables.h"
#include "protobuf/proto-c/custom_config.pb-c.h"

// Machine ID
#define MACHINE_ID 4
#define LOCATION "Torq3"
// Function Protos

esp_err_t init_littlefs(void);
esp_err_t init_gpio(void);
void wifi_init_sta(void);
void init_wifi(void);
void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void get_device_service_name(char *buf, size_t max);
void initialize_sntp();
void time_sync_notification_cb();
void init_ds3231();
void mpu6050_init();
long GET_DS3231_UNIX();
void test_sensors();
void parse_sensor_data(void *msg);
esp_err_t update_polling_rate_to_nvs(int global_poll_rate);
esp_err_t apply_polling_rates();
// Timer callback functions
void mpu6050_read(void *pvParams);

// Thread Protos
void monitor_boot(void *pvparams);
void send_sensor_data(void *params);
static void send_sensor_file(void *pvparams);
void monitor_distance(void *pvparams);

// User Function header files
#include "mqtt_functions.h"
// MAIN
void app_main(void)
{
    char *TAG = "Main";

    SensorfileMutexHandle = xSemaphoreCreateMutex();
    vPortCPUInitializeMutex(&my_mutex);
    // Set up MQTT topics

    // Set Subscribe topic names
    sprintf(update_polling_rate_topic, TEST_ "%s/updatePolling/request", mac);
    sprintf(reset_request_topic, TEST_ "%s/resetSystem/request", mac);

    // Set Publish topic names
    sprintf(sensor_data_topic, TEST_ "/sensorData");
    sprintf(birth_topic, TEST_ "/birth/request");
    sprintf(update_polling_rate_resp_topic, TEST_ "/updatePolling/response");
    sprintf(reset_request_resp_topic, TEST_ "/resetSystem/response");
    if (init_gpio() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set GPIOS.");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    if (i2cdev_init() == ESP_OK)
    {
        ESP_LOGI(TAG, "Initialised i2c mod.");
    }
    init_wifi();
    mpu6050_init();
    init_ds3231();

    sntp_set_sync_status(SNTP_SYNC_STATUS_RESET);
    gpio_set_level(LED, 0);
    if (init_littlefs() != ESP_OK)
    {
        ESP_LOGE(TAG, "LittleFS failed to initialize.");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    xEventGroupWaitBits(wifi_event_group, WIFI_PROV_DONE, false, false, portMAX_DELAY); // Wait here until WiFi provisioning is done

    esp_wifi_get_mac(WIFI_IF_STA, mac_addr);                                                                                     // get mac as u8t array
    sprintf(mac, "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]); // Store mac address as char in global char mac[20]
    if (apply_polling_rates() == ESP_OK)
    {
        ESP_LOGI(TAG, "Global Polling rate: %d", global_polling_rate);
    }
    xTaskCreate(monitor_distance, "ultrasonic", 2048, NULL, 10, NULL);
    xTaskCreate(monitor_boot, "Reset Handler", 2048, NULL, 10, NULL);
    xTaskCreatePinnedToCore(send_sensor_data, "sens data", 4096, NULL, 10, &Sensor_task_handle_t, 1);

    ESP_LOGI(TAG, "WiFi provisioned.");

    xTaskCreate(send_sensor_file, "send sensorfile", 5000, NULL, 8, NULL);
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, false, portMAX_DELAY); // Wait here until wifi is connected
    ESP_LOGI(TAG, "WiFi Connected.");
    mqtt_app_start();

    // Turn on the blue LED after all the initializations.
    gpio_set_level(LED, 1);

    vTaskDelete(NULL);
}

/**
 * @brief Gets sensor data and stringify's it in a JSON.
 *
 * @param pvparams
 */
void send_sensor_data(void *pvparams)
{
    char *TAG = "send Sens data";

    cJSON *sensor_json;
    struct timeval tv_now;
    int64_t time_us;
    uint8_t error_counter = 0;
    float temp = 27.0;

    while (1)
    {
        sensor_json = cJSON_CreateObject();
        ds3231_get_temp_float(&i2c_ds3231, &temp);
        // Adding Acccl data to JSON
        cJSON_AddNumberToObject(sensor_json, "x", acce.acce_x * 10);
        cJSON_AddNumberToObject(sensor_json, "y", acce.acce_y * 10);
        cJSON_AddNumberToObject(sensor_json, "z", acce.acce_z * 10);
        // Adding angular acc data to JSON
        cJSON_AddNumberToObject(sensor_json, "gx", gyro.gyro_x);
        cJSON_AddNumberToObject(sensor_json, "gy", gyro.gyro_y);
        cJSON_AddNumberToObject(sensor_json, "gz", gyro.gyro_z);

        // Complimentary angle data (roll & pitch)
        cJSON_AddNumberToObject(sensor_json, "roll", complimentary_angle.roll);
        cJSON_AddNumberToObject(sensor_json, "pitch", complimentary_angle.pitch);

        // Time from NTP and RTC
        gettimeofday(&tv_now, NULL);                                           // Ask system for time, this will return UNIX sync'd time after SNTP sync.
        time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec; // Timestamp in microseconds.
        cJSON_AddNumberToObject(sensor_json, "time", time_us);
        cJSON_AddNumberToObject(sensor_json, "RTC_time", GET_DS3231_UNIX());
        cJSON_AddNumberToObject(sensor_json, "Temperature", temp);

        /**
         * @todo: Distance from uSonic sensor
         *
         */

        cJSON_AddNumberToObject(sensor_json, "distance", distance);

        // Adding Machine Data to JSON
        cJSON_AddNumberToObject(sensor_json, "machine", MACHINE_ID);
        cJSON_AddStringToObject(sensor_json,"location",LOCATION);
        cJSON_AddStringToObject(sensor_json, "mac", mac);

        // Print JSON to validate.
        // char *json_String = cJSON_Print(sensor_json);
        char *json_String = cJSON_PrintBuffered(sensor_json, 390, 0);

        ESP_LOGI(TAG, "\n %s", json_String);
        /**
         * @todo : Add logic wherein it sends to mqtt directly if wifi is present,
         *         or else it writes to a file.
         *
         */
        // Delete JSON data
        printf("size of msg in task: %d\n", strlen(json_String));
        xTaskCreate(parse_sensor_data, "parser", 4000, (void *)json_String, 5, NULL);
        vTaskDelay(pdMS_TO_TICKS(500));
        cJSON_Delete(sensor_json);
        // Check if sensor is Broken
        test_sensors();
        if (MPU_FAIL || RTC_FAIL)
        {
            if (MPU_FAIL)
            {
                ESP_LOGW(TAG, "Attempting to restart MPU");
                if (mpu6050_wake_up(mpu6050_dev) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to wake up mpu6050");
                }
                else
                {
                    ESP_LOGI(TAG, "MPU restart successful");
                    MPU_FAIL = false;
                }
            }
            if (RTC_FAIL)
            {
                ESP_LOGW(TAG, "Reinitializing RTC");
                init_ds3231();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(global_polling_rate * 1000));
        cJSON_free(json_String);
    }
}

// Threads
void monitor_boot(void *pvparams)
{
    char *TAG = "monitor boot";
    while (1)
    {
        if (gpio_get_level(BOOT) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (gpio_get_level(BOOT) == 0)
            {
                ESP_LOGI(TAG, "Reset Handler triggered");
                for (int i = 0; i < 5; i++)
                {
                    gpio_set_level(LED, 0);
                    vTaskDelay(pdMS_TO_TICKS(250));
                    gpio_set_level(LED, 1);
                    vTaskDelay(pdMS_TO_TICKS(250));
                }
                nvs_flash_erase();
                esp_restart();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(RESET_BUTTON_POLL_PERIOD_MS));
    }
    vTaskDelete(NULL);
}

static void send_sensor_file(void *pvparams)
{
    char *TAG = "send_sensor_file";
    do
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    } while (SensorfileMutexHandle == NULL);

    TickType_t xLastWakeTime;

    while (1)
    {
        if (mqtt_connect_status_flag && wificonnected)
        { // Proceed only if WiFi is available and MQTT is connected
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (xSemaphoreTake(SensorfileMutexHandle, pdMS_TO_TICKS(1000)) == pdTRUE)
            {

                FILE *fp = fopen("/littlefs/sensor.txt", "rb");
                if (fp == NULL)
                {
                    ESP_LOGW(TAG, "Failed to open file. Or file does not exist");
                    xSemaphoreGive(SensorfileMutexHandle);
                    fclose(fp);
                }
                else
                {
                    fseek(fp, 0L, SEEK_END);
                    int size = ftell(fp);
                    if (size < 0)
                    {
                        fclose(fp);
                        remove("/littlefs/sensor.txt");
                        ESP_LOGI(TAG, "File empty,removing,");
                        // break;
                    }
                    ESP_LOGI(TAG, "Size is %d bytes", size);
                    char *msg = (char *)malloc(350);
                    if (msg == NULL)
                    {
                        ESP_LOGE(TAG, "Failed to allocate buffer.");
                    }
                    fseek(fp, 0L, SEEK_SET); // Go back to the begining of the file
                    int read;
                    while (fgets(msg, 350, fp))
                    {
                        printf("Retrieved line of length %zu :\n", strlen(msg));
                        printf("%s", msg);
                        int msg_id = esp_mqtt_client_publish(client, sensor_data_topic, msg, strlen(msg), 1, 1);
                        if (msg_id == -1)
                        {
                            ESP_LOGE(TAG, "Failed to send.");
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Sent with msd id: %d", msg_id);
                        }
                        vTaskDelay(pdMS_TO_TICKS(500));
                    }
                    // fread(msg,size,1,fp);
                    //  ESP_LOGW(TAG,"%s \n",msg);
                    free(msg);
                    fclose(fp);
                    remove("/littlefs/sensor.txt");
                    xSemaphoreGive(SensorfileMutexHandle);
                }
            } // If semaphore obtained
            else
            {
                ESP_LOGW(TAG, "Failed to get file mutex, retrying later.");
                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 * 60));
            }
        } // If MQTTCONN flag and wifi conn flag
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void monitor_distance(void *pv)
{

    // Set up of ultrasonic
    char *TAG = "ultrasonic thread";
    ultrasonic_sensor_t distance_sensor = {
        .trigger_pin = TRIG,
        .echo_pin = ECHO};

    ultrasonic_init(&distance_sensor);
    long temp_dist=0;
    long sum_of_dist=0;
    while (1)
    {
        for(int i =0; i <ultrasonic_ping_times; i++){
            esp_err_t res = ultrasonic_measure_cm(&distance_sensor, 200, &temp_dist);
            if (res != ESP_OK)
            {
                ESP_LOGE(TAG, "Error: ");
                switch (res)
                {
                case ESP_ERR_ULTRASONIC_PING:
                    ESP_LOGE(TAG,"Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    ESP_LOGE(TAG,"Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    ESP_LOGE(TAG,"Echo timeout (i.e. distance too big)\n");
                    distance = 200;
                    break;
                default:
                    ESP_LOGE(TAG,"%d\n", res);
                }
            }
            sum_of_dist = sum_of_dist + temp_dist;
            vTaskDelay(pdMS_TO_TICKS(ultrasonic_ping_rate_ms));
        }
        distance = sum_of_dist/ultrasonic_ping_times;
        sum_of_dist = 0;
        temp_dist=0;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// functions

/**
 * @brief Initializes all necessary GPIO pins
 *
 * @return esp_err_t ESP_OK if pass, ESP_FAIL if Fail
 */
esp_err_t init_gpio(void)
{
    esp_err_t ret;
    gpio_config_t gpio_input_config = {
        .pin_bit_mask = BIT64(BOOT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ret = gpio_config(&gpio_input_config);
    gpio_config_t gpio_output_config = {
        .pin_bit_mask = BIT64(LED),
        .mode = GPIO_MODE_OUTPUT,

    };
    ret = gpio_config(&gpio_output_config);
    return ret;
}

/**
 * @brief Initialses LittleFS with the given config structure
 *
 * @return esp_err_t ESP_OK  : PASS
 *                   ESP_FAIL: FAIL
 */
esp_err_t init_littlefs()
{
    char TAG[] = "lfs init: ";
    ESP_LOGI(TAG, "Initializing lfs");
    esp_err_t ret = esp_vfs_littlefs_register(&configLfs_Conf); // Register the configuration. configLfs_Conf in pin_definitions.h
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find LittleFS partition, error code unknown.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
    }
    else
    {
        size_t total_lfs_size = 0, used_lfs_size = 0;
        BaseType_t ret = esp_littlefs_info("littlefs", &total_lfs_size, &used_lfs_size);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to get littleFS usage stats.");
        }
        float total = total_lfs_size;
        float percentage = (used_lfs_size / total) * 100;
        ESP_LOGI(TAG, "LittleFS Partition Size: %d, Used:%d, Percentage Used: %f", total_lfs_size, used_lfs_size, percentage);
    }
    return ret;
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
    esp_wifi_get_mac(WIFI_IF_STA, mac_addr); // Get WiFi Mac Address

    /* If device is not yet provisioned start provisioning service */
    if (!provisioned)
    {
        ESP_LOGI(TAG, "Starting provisioning");
        /* What is the Device Service Name that we want
         * This translates to :
         *     - Wi-Fi SSID when scheme is wifi_prov_scheme_softap
         *     - device name when scheme is wifi_prov_scheme_ble
         */
        char service_name[50];
        get_device_service_name(service_name, sizeof(service_name));
        const char *pop = "TehoSens_23";
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
        wifi_init_sta();
        xEventGroupSetBits(wifi_event_group, WIFI_PROV_DONE); // let app_main() know that wifi prov is ok
    }
    /* Initialise SNTP */
    initialize_sntp(); // initialize sntp and talk to the server to get time every 30 minutes..
    return;
}

/**
 * @brief Initialises WiFi Modem in STAtion mode
 *
 */
void wifi_init_sta(void)
{
    /* Start Wi-Fi in station mode */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
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

/**
 * @brief Get the device service name string (char *)
 *
 * @param service_name Buffer to store service name.
 * @param max size of buffer passed
 */
void get_device_service_name(char *service_name, size_t max)
{
    uint8_t eth_mac[6];
    const char *ssid_prefix = "Teho_Sensor_Fusion_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "%s%02X%02X%02X",
             ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

/**
 * @brief Initialises SNTP
 *
 */
void initialize_sntp()
{
    ESP_LOGI("SNTP setup", "Initializing SNTP");
    /* Setting operating mode, we shall POLL every 30mins, this can be changed by:

    1) menuconfig--> component config--> LWIP-->SNTP
    OR
    2)using below function sntp_set_sync_interval(SyncIntervalInMilliSeconds)
    */

    if (sntp_enabled() == 0)
    {
        sntp_set_sync_interval(USER_CONFIG_SNTP_SYNC_INTERVAL); // USER_CONFIG_SNTP_SYNC_INTERVAL in network_definitions.h
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        ESP_LOGI("SNTP Setup", "Your NTP Server is %s and sync freq is %ull", NTP_SERVER, USER_CONFIG_SNTP_SYNC_INTERVAL);
        sntp_setservername(0, NTP_SERVER);
        sntp_set_time_sync_notification_cb(time_sync_notification_cb); // Callback function when sntp is acquired.
        sntp_init();
    }
}

/**
 * @brief Callback function when time_sync notification is received.
 *
 */
void time_sync_notification_cb()
{
    sntp_set_sync_status(SNTP_SYNC_STATUS_IN_PROGRESS);
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);                                                   // Ask system for time, this will return UNIX sync'd time after SNTP sync.
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec; // Timestamp in microseconds.
    ESP_LOGI("Time Sync CB", "Time sync has occured, timestamp: %llu", time_us);
    sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
    rtc_time = GET_DS3231_UNIX();
    return;
}

/**
 * @brief Initialise DS3231
 *
 */
void init_ds3231(void)
{
    char *TAG = "Init DS3231";
    memset(&i2c_ds3231, 0, sizeof(i2c_dev_t));

    if (ds3231_init_desc(&i2c_ds3231, I2C_NUM_0, SDA, SCL) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialise DS3231");
        vTaskDelay(pdMS_TO_TICKS(5000));
        return;
    }

    // Getting current time and temp from RTC
    float temp = 0;
    struct tm rtcinfo;

    if (ds3231_get_temp_float(&i2c_ds3231, &temp) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get temp");
    }
    if (ds3231_get_time(&i2c_ds3231, &rtcinfo) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get time.");
    }
    else
    {
        rtcinfo.tm_year = rtcinfo.tm_year;
        rtcinfo.tm_isdst = -1;
        ESP_LOGI(TAG, " Time From RTC: %04d-%02d-%02d %02d:%02d:%02d, %.2f deg Cel",
                 rtcinfo.tm_year + 1900, rtcinfo.tm_mon + 1,
                 rtcinfo.tm_mday, rtcinfo.tm_hour, rtcinfo.tm_min, rtcinfo.tm_sec, temp);
    }
    // Updating time

    bool lost_power;
    if (ds3231_get_oscillator_stop_flag(&i2c_ds3231, &lost_power) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read DS3231 stop flag");
    }
    if (lost_power)
    {
        ESP_LOGW(TAG, "DS3231 has lost power. Needs calibration");
        ESP_LOGI(TAG, "Waiting for time sync");
        do
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        } while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED);

        ESP_LOGI(TAG, "SNTP Synchronised");
        time_t now;
        struct tm timeinfo;
        char strftime_buf[64];
        time(&now);
        // now = now + (CONFIG_TIME_ZONE*60*60);
        localtime_r(&now, &timeinfo);                                  // Converts unixtimestamp to timeinfo struct
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo); // stringify's timeinfo struct
        ESP_LOGI(TAG, "The current date/time from NTP is (UTC): %s", strftime_buf);
        struct tm time = {
            .tm_year = timeinfo.tm_year,
            .tm_mon = timeinfo.tm_mon, // 0-based
            .tm_mday = timeinfo.tm_mday,
            .tm_hour = timeinfo.tm_hour,
            .tm_min = timeinfo.tm_min,
            .tm_sec = timeinfo.tm_sec};
        if (ds3231_set_time(&i2c_ds3231, &time) != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not set time.");
        }
        ds3231_clear_oscillator_stop_flag(&i2c_ds3231);
        ESP_LOGI(TAG, "Time set in DS3231");
    }
    else
    {
        ESP_LOGI(TAG, "DS3231 did not have a power failure event.");
    }
    if (ds3231_get_time(&i2c_ds3231, &rtcinfo) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get time.");
    }
    else
    {
        // rtcinfo.tm_year = rtcinfo.tm_year - 1900;
        rtcinfo.tm_isdst = -1;
        ESP_LOGI(TAG, "%04d-%02d-%02d %02d:%02d:%02d, %.2f deg Cel",
                 rtcinfo.tm_year + 1900, rtcinfo.tm_mon + 1,
                 rtcinfo.tm_mday, rtcinfo.tm_hour, rtcinfo.tm_min, rtcinfo.tm_sec, temp);
    }
}
/**
 * @brief Get the ds3231 unix timestamp
 *
 * @return long
 */
long GET_DS3231_UNIX(void)
{
    char *TAG = "DS3231 UNIX:";
    if (i2c_ds3231.addr == 0)
    {
        ESP_LOGE(TAG, "DS3231 not initialised");
        return 0;
    }
    struct tm rtcinfo;
    if (ds3231_get_time(&i2c_ds3231, &rtcinfo) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get time.");
        return 0;
    }
    long time = mktime(&rtcinfo);
    ESP_LOGD(TAG, "%ld", time);
    return time;
}

/**
 * @brief Initialise MPU 6050 along with timer CB that executes every 2.5ms
 *
 */
void mpu6050_init()
{

    char *TAG = "MPU6050 Init";

    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_MPU6050;
    conf.scl_io_num = SCL_MPU6050;
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

    const esp_timer_create_args_t cal_timer_config = {
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

/**
 * @brief Reads MPU 6050, pass this function to a timer CB that executes atleast 200HZ
 *
 * @param pvParameters
 */
void mpu6050_read(void *pvParameters)
{
    mpu6050_get_acce(mpu6050_dev, &acce);                                          // update global acce type with current value
    mpu6050_get_gyro(mpu6050_dev, &gyro);                                          // update global gyro type with current value
    mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle); // update global complimentary angle value with current value
    return;
}

/**
 * @brief Check if any sensor has failed, sets appropriate sensor state flag to true if a sensor has failed.
 *
 * @todo
 */
void test_sensors()
{
    char *TAG = "Test Sensors";
    uint8_t const MPU_ID;
    if (mpu6050_get_deviceid(mpu6050_dev, &MPU_ID) != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU Failure detected.");
        MPU_FAIL = true;
    }
    else
    {
        ESP_LOGI(TAG, "MPU device ID: %d", MPU_ID);
    }
    if (ds3231_get_oscillator_stop_flag(&i2c_ds3231, &RTC_FAIL) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to check RTC, RTC Failure.");
        RTC_FAIL = true;
    }
    else if (RTC_FAIL == true)
    {
        ESP_LOGW(TAG, "RTC lost power and needs calibration, RTC time: %ld", GET_DS3231_UNIX());
    }
    return;
}

esp_err_t reinit_ds3231()
{
    return ESP_FAIL;
}

/**
 * @brief This task-like function checks if
 *
 * @param json_char_void_ptr
 */
void parse_sensor_data(void *json_char_void_ptr)
{
    char *TAG = "parse sensor data";

    char *json_char = (char *)json_char_void_ptr;

    printf("size of msg in func: %d\n", strlen(json_char));
    if (!mqtt_connect_status_flag)
    {
        ESP_LOGW(TAG, "MQTT not connected, writing to file.");
        if (xSemaphoreTake(SensorfileMutexHandle, pdMS_TO_TICKS(2000)) != pdTRUE)
        {
            ESP_LOGE(TAG, "Failed to take File mutex.");
            // esp_restart();
            vTaskDelete(NULL);
        }
        FILE *fp = fopen("/littlefs/sensor.txt", "a+");
        if (fp == NULL)
        {
            ESP_LOGE(TAG, "Failed to open file.");
            xSemaphoreGive(SensorfileMutexHandle);
            vTaskDelete(NULL);
        }
        // long int size = fwrite(json_char,sizeof(json_char),1,fp);
        long int size = fprintf(fp, strcat(json_char, "\n"));
        if (size != strlen(json_char) + 1)
        {
            ESP_LOGE(TAG, "Written Size %ld, actual message size = %u", size, strlen(json_char));
        }
        else
        {
            ESP_LOGI(TAG, "File written");
        }
        fclose(fp); // BREAKS HERE SOMETIMES
        if ((xSemaphoreGive(SensorfileMutexHandle)) == pdTRUE)
        {
            ESP_LOGI(TAG, "Returned Semaphore");
        }
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "MQTT connected sending to MQTT directly");
    int msg_id = esp_mqtt_client_enqueue(client, sensor_data_topic, json_char, strlen(json_char), 1, 1, 1);
    if (msg_id == -1)
    {
        ESP_LOGE(TAG, "Failed to send.");
    }
    else
    {
        ESP_LOGI(TAG, "Sent with msd id: %d", msg_id);
    }

    vTaskDelete(NULL);
}

/**
 * @brief Updates the polling rates to the JSON saved in NVS, function also applies changes
 *
 * @param global_poll_rate
 * @return esp_err_t
 */
esp_err_t update_polling_rate_to_nvs(int global_poll_rate)
{

    char *TAG = "update polling rate";
    cJSON *polling_rates;
    polling_rates = cJSON_CreateObject();

    FILE *fp;
    fp = fopen("/littlefs/polling_rate.txt", "w+");
    if (fp == NULL)
    {
        ESP_LOGE(TAG, "Failed to open polling_rate.txt");
        cJSON_Delete(polling_rates);
        fclose(fp);
        return ESP_FAIL;
    }
    if (global_poll_rate < 0 || global_poll_rate > 255)
    {
        ESP_LOGE(TAG, "Global pollrate too low");
        return -2;
    }
    cJSON_AddNumberToObject(polling_rates, "global_poll_rate", global_poll_rate);
    char *json_String = cJSON_PrintBuffered(polling_rates, 350, 0);
    long int size = fprintf(fp, strcat(json_String, "\n"));
    if (size < 0)
    {
        ESP_LOGE(TAG, "Write to file failed.");
        fclose(fp);
        cJSON_free(json_String);
        cJSON_Delete(polling_rates);
        return -3;
    }
    fclose(fp);
    cJSON_free(json_String);
    cJSON_Delete(polling_rates);
    if (apply_polling_rates() == ESP_OK)
    {
        ESP_LOGI(TAG, "Updated and applied polling rates.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to apply updated polling rates.");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Applies the polling rates present in the saved JSON in NVS
 *
 * @return esp_err_t
 */
esp_err_t apply_polling_rates()
{
    char *TAG = "Apply polling rates";
    cJSON *polling_rates;
    polling_rates = cJSON_CreateObject();
    FILE *fp;
    fp = fopen("/littlefs/polling_rate.txt", "r");
    if (fp == NULL)
    {
        ESP_LOGE(TAG, "polling_rate.txt does not exist or failed to open");
        fclose(fp);
        return ESP_FAIL;
    }
    char *json_string = (char *)malloc(350);
    if (fgets(json_string, 350, fp))
    {
        printf("JSON: %s", json_string);
    }

    polling_rates = cJSON_Parse(json_string);
    if (cJSON_GetObjectItem(polling_rates, "global_poll_rate"))
    {
        global_polling_rate = cJSON_GetObjectItem(polling_rates, "global_poll_rate")->valueint;
        ESP_LOGW(TAG, "Global polling rate applied %d", global_polling_rate);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to get polling rate.");
        fclose(fp);
        cJSON_free(json_string);
        cJSON_Delete(polling_rates);
        return ESP_FAIL;
    }
    fclose(fp);
    cJSON_free(json_string);
    cJSON_Delete(polling_rates);
    return ESP_OK;
}