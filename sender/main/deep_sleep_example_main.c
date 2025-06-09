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
#include "deep_sleep_example.h"
#include "../../common/espnow.h"


static const char *TAG = "sender_deep_sleep_example";

// #if SOC_RTC_FAST_MEM_SUPPORTED
static RTC_DATA_ATTR struct timeval sleep_enter_time;
// #else
// static struct timeval sleep_enter_time;
// #endif

volatile bool send_done = false;

static void deep_sleep_task(void *args)
{
//     /**
//      * Prefer to use RTC mem instead of NVS to save the deep sleep enter time, unless the chip
//      * does not support RTC mem(such as esp32c2). Because the time overhead of NVS will cause
//      * the recorded deep sleep enter time to be not very accurate.
//      */
// #if !SOC_RTC_FAST_MEM_SUPPORTED
//     // Initialize NVS
//     esp_err_t err = nvs_flash_init();
//     if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         // NVS partition was truncated and needs to be erased
//         // Retry nvs_flash_init
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         err = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(err);

//     nvs_handle_t nvs_handle;
//     err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
//     if (err != ESP_OK) {
//         printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
//     } else {
//         printf("Open NVS done\n");
//     }

//     // Get deep sleep enter time
//     nvs_get_i32(nvs_handle, "slp_enter_sec", (int32_t *)&sleep_enter_time.tv_sec);
//     nvs_get_i32(nvs_handle, "slp_enter_usec", (int32_t *)&sleep_enter_time.tv_usec);
// #endif

    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }

// #if CONFIG_EXAMPLE_GPIO_WAKEUP
//         case ESP_SLEEP_WAKEUP_GPIO: {
//             uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
//             if (wakeup_pin_mask != 0) {
//                 int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
//                 printf("Wake up from GPIO %d\n", pin);
//             } else {
//                 printf("Wake up from GPIO\n");
//             }
//             break;
//         }
// #endif //CONFIG_EXAMPLE_GPIO_WAKEUP

#if CONFIG_EXAMPLE_EXT0_WAKEUP
        case ESP_SLEEP_WAKEUP_EXT0: {
            printf("Wake up from ext0\n");
            break;
        }
#endif // CONFIG_EXAMPLE_EXT0_WAKEUP

// #ifdef CONFIG_EXAMPLE_EXT1_WAKEUP
//         case ESP_SLEEP_WAKEUP_EXT1: {
//             uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
//             if (wakeup_pin_mask != 0) {
//                 int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
//                 printf("Wake up from GPIO %d\n", pin);
//             } else {
//                 printf("Wake up from GPIO\n");
//             }
//             break;
//         }
// #endif // CONFIG_EXAMPLE_EXT1_WAKEUP

// #ifdef CONFIG_EXAMPLE_TOUCH_WAKEUP
//         case ESP_SLEEP_WAKEUP_TOUCHPAD: {
//             printf("Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
//             break;
//         }
// #endif // CONFIG_EXAMPLE_TOUCH_WAKEUP

        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

#if CONFIG_IDF_TARGET_ESP32
    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);
#endif

    printf("Entering deep sleep\n");

    // get deep sleep enter time
    gettimeofday(&sleep_enter_time, NULL);

// #if !SOC_RTC_FAST_MEM_SUPPORTED
//     // record deep sleep enter time via nvs
//     ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "slp_enter_sec", sleep_enter_time.tv_sec));
//     ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "slp_enter_usec", sleep_enter_time.tv_usec));
//     ESP_ERROR_CHECK(nvs_commit(nvs_handle));
//     nvs_close(nvs_handle);
// #endif

    // enter deep sleep
    esp_deep_sleep_start();
}

static void example_deep_sleep_register_rtc_timer_wakeup(void)
{
    const int wakeup_time_sec = 20;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));
}



/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    send_done = true;

    printf("Send cb, mac: " MACSTR ", status: %d\n", MAC2STR(mac_addr), status);

    // example_espnow_event_t evt;
    // example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

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

void send_button_status()
{

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
    example_espnow_init(example_espnow_send_cb, NULL, false);

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
    // vTaskDelay(5000 / portTICK_PERIOD_MS);

    /* Enable wakeup from deep sleep by rtc timer */
    example_deep_sleep_register_rtc_timer_wakeup();

// #if CONFIG_EXAMPLE_GPIO_WAKEUP
//     /* Enable wakeup from deep sleep by gpio */
//     example_deep_sleep_register_gpio_wakeup();
// #endif

#if CONFIG_EXAMPLE_EXT0_WAKEUP
    /* Enable wakeup from deep sleep by ext0 */
    example_deep_sleep_register_ext0_wakeup(get_button_status() ? 0 : 1);
#endif

// #if CONFIG_EXAMPLE_EXT1_WAKEUP
//     /* Enable wakeup from deep sleep by ext1 */
//     example_deep_sleep_register_ext1_wakeup();
// #endif

// #if CONFIG_EXAMPLE_TOUCH_WAKEUP
//     /* Enable wakeup from deep sleep by touch */
//     example_deep_sleep_register_touch_wakeup();
// #endif

    xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 6, NULL);
}
