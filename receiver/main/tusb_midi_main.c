/*
 * SPDX-FileCopyrightText: 2019 Ha Thach (tinyusb.org)
 *
 * SPDX-License-Identifier: MIT
 *
 * SPDX-FileContributor: 2022-2023 Espressif Systems (Shanghai) CO LTD
 */

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

// #include "nvs_flash.h"
// #include "esp_random.h"
// #include "esp_event.h"
// #include "esp_netif.h"
// #include "esp_wifi.h"
// #include "esp_log.h"
// #include "esp_mac.h"
// #include "esp_now.h"
// #include "esp_crc.h"

static const char *TAG = "example";

// #define CONFIG_ESPNOW_CHANNEL 1
// // #define ESPNOW_WIFI_IF ESP_IF_WIFI_STA

// #define ESPNOW_WIFI_MODE WIFI_MODE_STA
// #define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA

// #define ESPNOW_LMK "1234567890123456" // 16 bytes primary master key
// #define ESPNOW_QUEUE_SIZE 10
// #define ESPNOW_MAXDELAY 1000 / portTICK_PERIOD_MS
// // #define CONFIG_ESPNOW_PMK ESPNOW_LMK
// #define CONFIG_ESPNOW_PMK "1234567890123456" // 16 bytes primary master key
// #define CONFIG_ESPNOW_SEND_COUNT 10
// #define CONFIG_ESPNOW_SEND_DELAY 1000 // ms
// #define CONFIG_ESPNOW_SEND_LEN 100 // bytes

// typedef enum {
//     EXAMPLE_ESPNOW_SEND_CB,
//     EXAMPLE_ESPNOW_RECV_CB,
// } example_espnow_event_id_t;

// typedef struct {
//     uint8_t mac_addr[ESP_NOW_ETH_ALEN];
//     esp_now_send_status_t status;
// } example_espnow_event_send_cb_t;

// typedef struct {
//     uint8_t mac_addr[ESP_NOW_ETH_ALEN];
//     uint8_t *data;
//     int data_len;
// } example_espnow_event_recv_cb_t;

// typedef union {
//     example_espnow_event_send_cb_t send_cb;
//     example_espnow_event_recv_cb_t recv_cb;
// } example_espnow_event_info_t;

// /* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
// typedef struct {
//     example_espnow_event_id_t id;
//     example_espnow_event_info_t info;
// } example_espnow_event_t;

// enum {
//     EXAMPLE_ESPNOW_DATA_BROADCAST,
//     EXAMPLE_ESPNOW_DATA_UNICAST,
//     EXAMPLE_ESPNOW_DATA_MAX,
// };

// static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
// static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

/** Helper defines **/

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

// static void periodic_midi_write_example_cb(void *arg)
// {
//     // Example melody stored as an array of note values
//     uint8_t const note_sequence[] = {
//         74, 78, 81, 86, 90, 93, 98, 102, 57, 61, 66, 69, 73, 78, 81, 85, 88, 92, 97, 100, 97, 92, 88, 85, 81, 78,
//         74, 69, 66, 62, 57, 62, 66, 69, 74, 78, 81, 86, 90, 93, 97, 102, 97, 93, 90, 85, 81, 78, 73, 68, 64, 61,
//         56, 61, 64, 68, 74, 78, 81, 86, 90, 93, 98, 102
//     };

//     static uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
//     static uint8_t const channel = 0; // 0 for channel 1
//     static uint32_t note_pos = 0;

//     // Previous positions in the note sequence.
//     int previous = note_pos - 1;

//     // If we currently are at position 0, set the
//     // previous position to the last note in the sequence.
//     if (previous < 0) {
//         previous = sizeof(note_sequence) - 1;
//     }

//     // Send Note On for current position at full velocity (127) on channel 1.
//     ESP_LOGI(TAG, "Writing MIDI data %d", note_sequence[note_pos]);

//     if (tud_midi_mounted()) {
//         uint8_t note_on[3] = {NOTE_ON | channel, note_sequence[note_pos], 127};
//         tud_midi_stream_write(cable_num, note_on, 3);

//         // Send Note Off for previous note.
//         uint8_t note_off[3] = {NOTE_OFF | channel, note_sequence[previous], 0};
//         tud_midi_stream_write(cable_num, note_off, 3);
//     }

//     // Increment position
//     note_pos++;

//     // If we are at the end of the sequence, start over.
//     if (note_pos >= sizeof(note_sequence)) {
//         note_pos = 0;
//     }
// }

// static void example_wifi_init(void)
// {
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
//     ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
//     ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
//     ESP_ERROR_CHECK( esp_wifi_start());
//     ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

// // #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
//     ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
// // #endif
// }

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

    // evt.id = EXAMPLE_ESPNOW_RECV_CB;
    // memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    // recv_cb->data = malloc(len);
    // if (recv_cb->data == NULL) {
    //     ESP_LOGE(TAG, "Malloc receive data fail");
    //     return;
    // }
    // memcpy(recv_cb->data, data, len);
    // recv_cb->data_len = len;
    // if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
    //     ESP_LOGW(TAG, "Send receive queue fail");
    //     free(recv_cb->data);
    // }
}

// static esp_err_t example_espnow_init(void)
// {
//     // example_espnow_send_param_t *send_param;

//     // s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
//     // if (s_example_espnow_queue == NULL) {
//     //     ESP_LOGE(TAG, "Create mutex fail");
//     //     return ESP_FAIL;
//     // }

//     /* Initialize ESPNOW and register sending and receiving callback function. */
//     ESP_ERROR_CHECK( esp_now_init() );
//     //ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
//     ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
// #if CONFIG_ESPNOW_ENABLE_POWER_SAVE
//     ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
//     ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
// #endif
//     /* Set primary master key. */
//     ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

//     /* Add broadcast peer information to peer list. */
//     // esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
//     // if (peer == NULL) {
//     //     ESP_LOGE(TAG, "Malloc peer information fail");
//     //     vSemaphoreDelete(s_example_espnow_queue);
//     //     esp_now_deinit();
//     //     return ESP_FAIL;
//     // }
//     // memset(peer, 0, sizeof(esp_now_peer_info_t));
//     // peer->channel = CONFIG_ESPNOW_CHANNEL;
//     // peer->ifidx = ESPNOW_WIFI_IF;
//     // peer->encrypt = false;
//     // memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
//     // ESP_ERROR_CHECK( esp_now_add_peer(peer) );
//     // free(peer);

//     // /* Initialize sending parameters. */
//     // send_param = malloc(sizeof(example_espnow_send_param_t));
//     // if (send_param == NULL) {
//     //     ESP_LOGE(TAG, "Malloc send parameter fail");
//     //     vSemaphoreDelete(s_example_espnow_queue);
//     //     esp_now_deinit();
//     //     return ESP_FAIL;
//     // }
//     // memset(send_param, 0, sizeof(example_espnow_send_param_t));
//     // send_param->unicast = false;
//     // send_param->broadcast = true;
//     // send_param->state = 0;
//     // send_param->magic = esp_random();
//     // send_param->count = CONFIG_ESPNOW_SEND_COUNT;
//     // send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
//     // send_param->len = CONFIG_ESPNOW_SEND_LEN;
//     // send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
//     // if (send_param->buffer == NULL) {
//     //     ESP_LOGE(TAG, "Malloc send buffer fail");
//     //     free(send_param);
//     //     vSemaphoreDelete(s_example_espnow_queue);
//     //     esp_now_deinit();
//     //     return ESP_FAIL;
//     // }
//     // memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
//     // example_espnow_data_prepare(send_param);

//     // xTaskCreate(example_espnow_task, "example_espnow_task", 2048, send_param, 4, NULL);

//     return ESP_OK;
// }

void app_main(void)
{
    // vTaskDelay(4000 / portTICK_PERIOD_MS); // Wait for the system to stabilize

    ESP_LOGI(TAG, "USB initialization");

    // tusb_desc_device_t descriptor = {
    //     .bLength = sizeof(tusb_desc_device_t),
    //     .bDescriptorType = TUSB_DESC_DEVICE,
    //     .bcdUSB = 0x0200, // USB 2.0
    //     .bDeviceClass = TUSB_CLASS_MISC, // Class code for MIDI
    //     .bDeviceSubClass = 0x00, // Subclass code for MIDI
    //     .bDeviceProtocol = 0x00, // Protocol code for MIDI
    //     .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    //     .idVendor = 0xCafe, // Vendor ID (example)
    //     .idProduct = 0x4001, // Product ID (example)
    //     .bcdDevice = 0x0100, // Device release number
    //     .iManufacturer = 1, // Index of manufacturer string descriptor
    //     .iProduct = 2, // Index of product string descriptor
    //     .iSerialNumber = 3, // Index of serial number string descriptor
    //     .bNumConfigurations = 1, // Number of configurations
    // };
    tinyusb_config_t const tusb_cfg = {
        .device_descriptor = NULL, // If device_descriptor is NULL, tinyusb_driver_install() will use Kconfig
        .string_descriptor = s_str_desc,
        .string_descriptor_count = sizeof(s_str_desc) / sizeof(s_str_desc[0]),
        .external_phy = false,
        .configuration_descriptor = s_midi_cfg_desc,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "USB initialization DONE");

    // // Periodically send MIDI packets
    // int const tempo = 286;
    // const esp_timer_create_args_t periodic_midi_args = {
    //     .callback = &periodic_midi_write_example_cb,
    //     /* name is optional, but may help identify the timer when debugging */
    //     .name = "periodic_midi"
    // };

    // ESP_LOGI(TAG, "MIDI write task init");
    // esp_timer_handle_t periodic_midi_timer;
    // ESP_ERROR_CHECK(esp_timer_create(&periodic_midi_args, &periodic_midi_timer));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_midi_timer, tempo * 1000));

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
    example_espnow_init(NULL, example_espnow_recv_cb, false);

    // while(1) vTaskDelay(10000 / portTICK_PERIOD_MS); // Keep the task running
    // while(1){
    //     vTaskDelay(10000 / portTICK_PERIOD_MS);
    // }

    // BaseType_t xTaskWokenByReceive = pdFALSE;
    // char cRxedChar;
    // ReceiveQueueMessage received_message;

    ESP_LOGI(TAG, "Reading queue");

    while (true)
    {
        //vTaskDelay(10 / portTICK_PERIOD_MS); // Prevent busy-waiting
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
