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

static const char *TAG = "example";

#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif

static void obtain_time(void);

void app_main(void)
{
    // ++boot_count;
    // ESP_LOGI(TAG, "Boot count: %d", boot_count);
    while (1)
    {

        time_t now;
        struct tm timeinfo;           // create structure tm to store time details like hours, min...
        time(&now);                   // gives time in seconds since epoch
        localtime_r(&now, &timeinfo); // converts epoch time to local time
        // Is time set? If not, tm_year will be (1970 - 1900).
        if (timeinfo.tm_year < (2016 - 1900))
        {
            ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
            obtain_time(); // if correct time is not set, then get the time from ntp server.
            time(&now);    // update 'now' variable with current time
        }

        char strftime_buf[64];

        // Set timezone to Indian Standard Time and print local time
        setenv("TZ", "IST-5.5", 1);
        tzset();
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void obtain_time(void) // get the time from ntp server
{
    ESP_ERROR_CHECK(nvs_flash_init());                // default nvs partition initialized
    ESP_ERROR_CHECK(esp_netif_init());                // initialize tcp/ip protocols
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // default event loop created

    //     /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    //      * Read "Establishing Wi-Fi or Ethernet Connection" section in
    //      * examples/protocols/README.md for more information about this function.
    //      */

    ESP_ERROR_CHECK(example_connect()); // function to configure Wifi through menuconfig

    //      This is the basic default config with one server and starting the service
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER); // configures to one default ntp server

    esp_netif_sntp_init(&config); // initialize sntp

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 15;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }
    time(&now);
    localtime_r(&now, &timeinfo); // converts the epoch time from ntp to local time

    ESP_ERROR_CHECK(example_disconnect()); // disconnects wifi
    esp_netif_sntp_deinit();               // deinitialize netif module
}
