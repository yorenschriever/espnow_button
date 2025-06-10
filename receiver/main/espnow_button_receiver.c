#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tinyusb.h"
#include <stdbool.h>
#include <stdlib.h>

#include "../../common/espnow.h"
#include "led_strip_encoder.h"

static const char *TAG = "receiver";

static const int LED_PIN = 48; // GPIO pin for the LED

static const int MIDI_CHANNEL = 15; // MIDI channel to use, 0-15

#define SUPPORTED_MIDI_NOTES 16
bool note_status[SUPPORTED_MIDI_NOTES] = {false};

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
tusb_desc_device_t my_descriptor = {
    .bLength = sizeof(my_descriptor),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, // USB version. 0x0200 means version 2.0
    .bDeviceClass = TUSB_CLASS_UNSPECIFIED,
    .bMaxPacketSize0 = 64,

    .idVendor = 0x303A,
    .idProduct = 0x3000,
    .bcdDevice = 0x0101, // Device FW version

    .iManufacturer = 0x01, // see string_descriptor[1] bellow
    .iProduct = 0x02,      // see string_descriptor[2] bellow
    .iSerialNumber = 0x03, // see string_descriptor[3] bellow

    .bNumConfigurations = 0x01};

static const char *my_string_descriptor[5] = {
    // tusb_desc_strarray_device_t my_string_descriptor = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "MULTISCHRIEVER",     // 1: Manufacturer
    "ESP-NOW button",     // 2: Product
    "123456",             // 3: Serials, should use chip ID
    "MIDI button device", // 4: MIDI
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
    Payload payload;
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
                ESP_LOGI(TAG, "MIDI Read - Time (ms since boot): %lld, Data: %02hhX %02hhX %02hhX %02hhX",
                         esp_timer_get_time(), packet[0], packet[1], packet[2], packet[3]);
            }
        }
    }
}

// Basic MIDI Messages
#define NOTE_OFF 0x80
#define NOTE_ON 0x90

rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;
static uint8_t blink_off[3] = {0, 0, 0};
rmt_transmit_config_t tx_config = {
    .loop_count = 0, // no transfer loop
};

void setup_led()
{
    ESP_LOGI(TAG, "Create RMT TX channel");

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = LED_PIN,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = 10000000,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");

    led_strip_encoder_config_t encoder_config = {
        .resolution = 10000000,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));
}

void blinkLed(uint8_t color[3], uint8_t duration_ms)
{
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, color, sizeof(color), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, blink_off, sizeof(blink_off), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

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

void sendMiniNote(uint8_t note, bool on, bool force)
{
    if (note >= SUPPORTED_MIDI_NOTES)
    {
        ESP_LOGE(TAG, "Note %d is out of range (0-%d)", note, SUPPORTED_MIDI_NOTES - 1);
        return;
    }

    if (note_status[note] == on && !force)
    {
        ESP_LOGI(TAG, "Note %d already in state %s", note, on ? "on" : "off");
        return; // No change needed
    }
    note_status[note] = on; // Update the note status

    if (on)
    {
        ESP_LOGI(TAG, "Note %d on", note);
        uint8_t note_on[3] = {NOTE_ON + MIDI_CHANNEL, note, 127};
        tud_midi_stream_write(0, note_on, sizeof(note_on));
    }
    else
    {
        ESP_LOGI(TAG, "Note %d off", note);
        uint8_t note_off[3] = {NOTE_OFF + MIDI_CHANNEL, note, 0};
        tud_midi_stream_write(0, note_off, sizeof(note_off));
    }
}

void initMidi()
{
    for (int i = 0; i < SUPPORTED_MIDI_NOTES; i++)
    {
        // Send a Note Off for all notes to ensure no stuck notes
        sendMiniNote(i, note_status[i], true);
    }
}

void app_main(void)
{
    setup_led();
    ESP_LOGI(TAG, "USB initialization");

    tinyusb_config_t const tusb_cfg = {
        .descriptor = &my_descriptor,
        .string_descriptor = my_string_descriptor,
        .string_descriptor_count = sizeof(my_string_descriptor) / sizeof(my_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = s_midi_cfg_desc,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "Creating receive queue");
    receive_queue = xQueueCreate(RECEIVE_QUEUE_SIZE, sizeof(received_queue_message));
    if (receive_queue == NULL)
    {
        ESP_LOGE(TAG, "Create event_queue fail");
        return;
    }

    // Read received MIDI packets
    ESP_LOGI(TAG, "MIDI read task init");
    xTaskCreate(midi_task_read_example, "midi_task_read_example", 2 * 1024, NULL, 5, NULL);

    // ESP NOW initialization
    example_nvs_init();
    example_wifi_init();
    example_espnow_init(NULL, example_espnow_recv_cb);

    ESP_LOGI(TAG, "Reading queue");

    blinkLed((uint8_t[]){255, 0, 0}, 250); // Blink green LED to indicate startup

    while (true)
    {
        if (xQueueReceive(receive_queue, (void *)&handled_queue_message, 100))
        {
            int button = handled_queue_message.payload.button_id;
            int state = handled_queue_message.payload.status;

            ESP_LOGI(TAG, "Received button %d state %d: ", button, state);

            sendMiniNote(button, state, false);

            Payload ack_payload;
            ack_payload.type = TYPE_ACK;
            ack_payload.button_id = button;
            ack_payload.status = state;
            ack_payload.msg_id = handled_queue_message.payload.msg_id; // Use the same msg_id for acknowledgment
            espnow_broadcast((uint8_t *)&ack_payload, sizeof(ack_payload));

            blinkLed((uint8_t[]){0, 0, 255}, 10); // Blink the LED to indicate a message was received
        }
    }
}
