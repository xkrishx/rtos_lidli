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
#include "protocol_examples_common.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_attr.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"

static const char *TAG = "MQTT";
char strftime_buf[64];

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void mqtt5_app_start(void);
static void obtain_time(void);

void ntp_time();

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());
    mqtt5_app_start();
    ntp_time();
}

void ntp_time()
{
    time_t now;
    struct tm timeinfo;
    // char strftime_buf[64];
    time(&now);
    localtime_r(&now, &timeinfo);
    obtain_time();
    // update 'now' variable with current time
    time(&now);
    localtime_r(&now, &timeinfo);
    setenv("TZ", "IST-5:30", 1);
    tzset();
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
}

static void obtain_time(void)
{
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org"); // configures to one default ntp server
    esp_netif_sntp_init(&config);                                             // initialize sntp
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    time(&now);
    localtime_r(&now, &timeinfo);
    return;
}

static void mqtt5_app_start(void)
{
    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,
        .network.disable_auto_reconnect = true,
        // .credentials.username = "123",
        // .credentials.authentication.password = "456",
        // .session.last_will.topic = "/topic/will",
        // .session.last_will.msg = "i will leave",
        // .session.last_will.msg_len = 12,
        // .session.last_will.qos = 1,
        // .session.last_will.retain = true,
    };
    esp_mqtt_client_handle_t client;
    client = esp_mqtt_client_init(&mqtt5_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    int msg_id;
    esp_mqtt_event_handle_t event = event_data;      // here esp_mqtt_event_handle_t is a struct which receieves struct event from mqtt app start funtion
    esp_mqtt_client_handle_t client = event->client; // making obj client of struct esp_mqtt_client_handle_t and assigning it the receieved event client

    if (event->event_id == MQTT_EVENT_CONNECTED)
    {
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_publish(client, "krishnapc/LIPL/Time", strftime_buf, 0, 0, 1);
        esp_mqtt_client_publish(client, "krishnapc/LIPL/Pitch", "5.65 deg.", 0, 0, 1);
        esp_mqtt_client_publish(client, "krishnapc/LIPL/Roll", "-3.24 deg.", 0, 0, 1);
        esp_mqtt_client_publish(client, "krishnapc/LIPL/Distance", "The distance from USonic sensor is 20 cm.", 0, 0, 1);
        ESP_LOGI(TAG, "published successfully.");

        msg_id = esp_mqtt_client_subscribe(client, "/krishnapc/LIPL/#", 0);
        // in mqtt we require a topic to subscribe and client is from event client and 0 is quality of service it can be 1 or 2
        ESP_LOGI(TAG, "sent subscribe successful");

        // esp_mqtt_client_subscribe(client, "/krishnapc/LIPL/MPU_Readings", 0);

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
        int datalen = event->data_len;
        ESP_LOGI(TAG, "%.*s", datalen, event->data);
    }
    else if (event->event_id == MQTT_EVENT_ERROR) // when any error
    {
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    }
}
