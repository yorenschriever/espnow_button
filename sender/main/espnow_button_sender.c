#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "../../common/espnow.h"

static const char *TAG = "sender_deep_sleep_example";

volatile bool send_done = false;

const int ext_wakeup_pin_0 = 3;
void example_deep_sleep_register_ext0_wakeup(int level)
{
    printf("Enabling EXT0 wakeup on pin GPIO%d\n", ext_wakeup_pin_0);
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(ext_wakeup_pin_0, level));

    // Configure pullup/downs via RTCIO to tie wakeup pins to inactive level during deepsleep.
    // EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
    // No need to keep that power domain explicitly, unlike EXT1.
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(ext_wakeup_pin_0));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(ext_wakeup_pin_0));
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    send_done = true;

    printf("Send cb, mac: " MACSTR ", status: %d\n", MAC2STR(mac_addr), status);

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    // evt.id = EXAMPLE_ESPNOW_SEND_CB;
    // memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    // send_cb->status = status;
    // if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
    //     ESP_LOGW(TAG, "Send send queue fail");
    // }
    // espnow_queue_event(&evt);

    // if (espnow_queue_event(&evt) != pdTRUE) {
    //     ESP_LOGW(TAG, "Send send queue fail");
    // }
}

unsigned long millis() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

int get_button_status()
{
    return gpio_get_level(GPIO_NUM_3); // Example GPIO pin, replace with actual button GPIO
}

void app_main(void)
{
    unsigned long start = millis();
    printf("Deep sleep example\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    example_wifi_init();
    example_espnow_init(example_espnow_send_cb, NULL);

    uint8_t payload[2] = {0x01, get_button_status() ? 0x01 : 0x00}; 
    if (espnow_broadcast(payload, sizeof(payload)) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        vTaskDelete(NULL);
    }

    printf("Send start, time taken: %lu ms\n", millis() - start);

    while(!send_done) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    printf("Send done, time taken: %lu ms\n", millis() - start);

    example_deep_sleep_register_ext0_wakeup(get_button_status() ? 0 : 1);

    printf("Entering deep sleep...\n");
    esp_deep_sleep_start();
}
