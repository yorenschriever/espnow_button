#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include <stdbool.h>
#include <stdlib.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "../../common/espnow.h"

//TODO
static const char *TAG = "example";

// Interface counter
enum interface_count
{
#if CFG_TUD_MIDI
    ITF_NUM_MIDI = 0,
    ITF_NUM_MIDI_STREAMING,
#endif
    ITF_COUNT
};

// USB Endpoint numbers
enum usb_endpoints
{
    // Available USB Endpoints: 5 IN/OUT EPs and 1 IN EP
    EP_EMPTY = 0,
#if CFG_TUD_MIDI
    EPNUM_MIDI,
#endif
};

/** TinyUSB descriptors **/
#define TUSB_DESCRIPTOR_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_MIDI * TUD_MIDI_DESC_LEN)

/**
 * @brief String descriptor
 */
static const char *s_str_desc[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "MULTISCHRIEVER",     // 1: Manufacturer
    "Midi button",        // 2: Product
    "123456",             // 3: Serials, should use chip ID
    // "MIDI button device", // 4: MIDI
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and a MIDI interface
 */
static const uint8_t s_midi_cfg_desc[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, TUSB_DESCRIPTOR_TOTAL_LEN, 0, 100),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 4, EPNUM_MIDI, (0x80 | EPNUM_MIDI), 64),
};

struct ReceiveQueueMessage
{
    char mac[ESP_NOW_ETH_ALEN];
    uint8_t button;
    uint8_t state; // 0 = released, 1 = pressed
} received_queue_message, handled_queue_message;

static QueueHandle_t receive_queue;
const int RECEIVE_QUEUE_SIZE = 10;

static void midi_task_read_example(void *arg)
{
    // The MIDI interface always creates input and output port/jack descriptors
    // regardless of these being used or not. Therefore incoming traffic should be read
    // (possibly just discarded) to avoid the sender blocking in IO
    uint8_t packet[4];
    bool read = false;
    for (;;)
    {
        vTaskDelay(1);
        while (tud_midi_available())
        {
            read = tud_midi_packet_read(packet);
            if (read)
            {
                ESP_LOGI(TAG, "Read - Time (ms since boot): %lld, Data: %02hhX %02hhX %02hhX %02hhX",
                         esp_timer_get_time(), packet[0], packet[1], packet[2], packet[3]);
            }
        }
    }
}

// Basic MIDI Messages
#define NOTE_OFF 0x80
#define NOTE_ON 0x90

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    // example_espnow_event_t evt;
    // example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t *mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    ESP_LOGI(TAG, "Received data, len: %d. Button %d is %d", len, data[0], data[1]);

    received_queue_message.button = data[0];
    received_queue_message.state = data[1];
    memcpy(received_queue_message.mac, mac_addr, ESP_NOW_ETH_ALEN);
    xQueueSend(receive_queue, (void *)&received_queue_message, (TickType_t)0);

}

void app_main(void)
{
    ESP_LOGI(TAG, "USB initialization");

    tinyusb_config_t const tusb_cfg = {
        .device_descriptor = NULL, // If device_descriptor is NULL, tinyusb_driver_install() will use Kconfig
        .string_descriptor = s_str_desc,
        .string_descriptor_count = sizeof(s_str_desc) / sizeof(s_str_desc[0]),
        .external_phy = false,
        .configuration_descriptor = s_midi_cfg_desc,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "USB initialization DONE");

    ESP_LOGI(TAG, "Creating receive queue");
    receive_queue = xQueueCreate(RECEIVE_QUEUE_SIZE, sizeof(received_queue_message));
    if (receive_queue == NULL)
    {
        ESP_LOGE(TAG, "Create event_queue fail");
        return;
    }

    // Read recieved MIDI packets
    ESP_LOGI(TAG, "MIDI read task init");
    xTaskCreate(midi_task_read_example, "midi_task_read_example", 2 * 1024, NULL, 5, NULL);

    // ESP NOW initialization
    ESP_LOGI(TAG, "ESP NOW initialization");
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    example_wifi_init();
    example_espnow_init(NULL, example_espnow_recv_cb);

    ESP_LOGI(TAG, "Reading queue");

    while (true)
    {
        if (xQueueReceive (receive_queue, (void *)&handled_queue_message, 100))
        {
            // Process the received message
            ESP_LOGI(TAG, "Received button %d state %d: ", handled_queue_message.button, handled_queue_message.state);

            // Here you can add code to handle the button press/release
            // For example, send a MIDI message based on the button state
            if (handled_queue_message.state == 1)
            {
                // Button pressed
                ESP_LOGI(TAG, "Button %d pressed", handled_queue_message.button);
                // Send MIDI Note On message
                uint8_t note_on[3] = {NOTE_ON, handled_queue_message.button, 127};
                tud_midi_stream_write(0, note_on, sizeof(note_on));
            }
            else
            {
                // Button released
                ESP_LOGI(TAG, "Button %d released", handled_queue_message.button);
                // Send MIDI Note Off message
                uint8_t note_off[3] = {NOTE_OFF, handled_queue_message.button, 0};
                tud_midi_stream_write(0, note_off, sizeof(note_off));
            }
        }
    }
}
