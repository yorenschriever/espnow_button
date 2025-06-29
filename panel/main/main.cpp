
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "../../common/espnow.h"
#include "pwmDriver.hpp"
// #include "../esp32/pwmDriver-esp.hpp"

extern "C"
{
  void app_main(void);
}

static const char *TAG = "panel";

#define SUPPORTED_MIDI_NOTES 16
bool note_status[SUPPORTED_MIDI_NOTES] = {false};

struct ReceiveQueueMessage
{
    char mac[ESP_NOW_ETH_ALEN];
    Payload payload;
} received_queue_message, handled_queue_message;

static QueueHandle_t receive_queue;
const int RECEIVE_QUEUE_SIZE = 10;

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    uint8_t *mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    memcpy(received_queue_message.mac, mac_addr, ESP_NOW_ETH_ALEN);
    memcpy(&received_queue_message.payload, data, sizeof(Payload));
    xQueueSend(receive_queue, (void *)&received_queue_message, (TickType_t)0);
}

void app_main()
{

    ESP_LOGI(TAG, "Creating receive queue");
    receive_queue = xQueueCreate(RECEIVE_QUEUE_SIZE, sizeof(received_queue_message));
    if (receive_queue == NULL)
    {
        ESP_LOGE(TAG, "Create event_queue fail");
        return;
    }

    auto pwm = new PWMDriver_ESP();
    while(!pwm->ready()) vTaskDelay(10 / portTICK_PERIOD_MS);
    for(int i = 0; i < 12; i++) pwm->write(i, 0, false); // Initialize all channels to off
    pwm->show(); // Ensure initial state is applied

    // ESP NOW initialization
    example_nvs_init();
    example_wifi_init();
    example_espnow_init(NULL, example_espnow_recv_cb);

    ESP_LOGI(TAG, "Reading queue");
    while (true)
    {
        if (xQueueReceive(receive_queue, (void *)&handled_queue_message, 100))
        {
            int button = handled_queue_message.payload.button_id;
            int state = handled_queue_message.payload.status;
            bool is_ack = handled_queue_message.payload.type == TYPE_ACK;

            if (is_ack)
            {
                ESP_LOGI(TAG, "Received ACK for button %d", button);
                continue; // Ignore ACK messages
            }

            ESP_LOGI(TAG, "Received button %d state %d: ", button, state);

            
            while(!pwm->ready()) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            pwm->write(button, state ? 4096 : 0, false); 
            pwm->show();
        }
    }
}