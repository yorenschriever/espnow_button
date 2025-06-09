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
#include "../../common/espnow.h"

static const char *TAG = "sender";

static const int BUTTON_ID = 1;

static volatile RTC_DATA_ATTR bool got_ack = false;
static volatile RTC_DATA_ATTR int needs_ack_from = 0;

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    uint8_t *mac_addr = recv_info->src_addr;
    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    Payload * payload = (Payload *)data;

    if (payload->type != TYPE_ACK) {
        ESP_LOGW(TAG, "Received non-acknowledgment message, type: %d", payload->type);
        return;
    }
    if (payload->msg_id != needs_ack_from) {
        ESP_LOGW(TAG, "Received acknowledgment for unexpected message ID: %d, expected: %d", payload->msg_id, needs_ack_from);
        return;
    }
    if (payload->button_id != BUTTON_ID) {
        ESP_LOGW(TAG, "Received acknowledgment for unexpected button ID: %d, expected: %d", payload->button_id, BUTTON_ID);
        return;
    }
    got_ack = true;

    printf("GOT ACK \n");

}

unsigned long millis() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

inline int get_button_status()
{
    return gpio_get_level(GPIO_NUM_3); // Example GPIO pin, replace with actual button GPIO
}

int debounce(int status)
{
    for(int i=0; i<10; i++) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (status != get_button_status()) {
            status = get_button_status();
            i=0;
        }
    }
    return status;
}

void send_button_status(int status)
{
    Payload payload;
    payload.type = TYPE_STATUS_UPDATE;
    payload.button_id = BUTTON_ID; 
    payload.status = status; // 0 for released, 1 for pressed
    payload.msg_id = ++needs_ack_from; // ID of the sender, used for acknowledgment

    if (espnow_broadcast((uint8_t*)&payload, sizeof(payload)) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
    }
    got_ack = false;
}

static void deep_sleep_task(void *args)
{
    vTaskDelay(10 / portTICK_PERIOD_MS); // Allow some time for the last message to be sent
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(GPIO_NUM_3, get_button_status() ? 0 : 1));
    esp_deep_sleep_start();
    vTaskDelete(NULL); 
}

void app_main(void)
{
    unsigned long start = millis();
    printf("Deep sleep example\n");

    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_3, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(GPIO_NUM_3));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(GPIO_NUM_3));
    
    example_nvs_init();
    example_wifi_init();
    example_espnow_init(NULL, example_espnow_recv_cb);

    printf("Send start, time taken: %lu ms\n", millis() - start);

    int status;
    int new_status;
    const int max_attempts = 5;
    int attempts = 0;
    do {
        status = get_button_status();
        // Immediately send the current status for a snappy response
        send_button_status(status);
        // Then wait for the button to be stable for 100ms. 
        new_status = debounce(status);
    } while (
        (   
            // After we got a stable button signal,
            // we check if it still corresponds to the last message we sent out
            // and if we meanwhile have received an acknowledgment.
            // If not, we do the entire thing over again, but limit the total attempts
            // to avoid draining the battery when the host is not turned on.
            status != new_status ||
            !got_ack
        ) &&
        ++attempts < max_attempts
    );

    printf("Send done, time taken: %lu ms\n", millis() - start);

    // example_espnow_deinit();
    // vTaskDelay(100 / portTICK_PERIOD_MS);

    printf("Entering deep sleep\n");

    // Putting the sleep call in another thread avoids getting the following error:
    // E (???) sleep: Deep sleep request is rejected
    // I dont understand why a normal vTaskDelay call would not give enough 
    // time for the other processes to finish, but this works.
    xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 6, NULL);

    
}
