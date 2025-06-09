#pragma once

// #include "nvs_flash.h"
// #include "nvs.h"

#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

// #define WIFI_STORAGE_RAM
// #define ESPNOW_WIFI_MODE ESPNOW_WIFI_MODE_STATION
#define CONFIG_ESPNOW_CHANNEL 1
// #define ESPNOW_WIFI_IF ESP_IF_WIFI_STA

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA

#define ESPNOW_LMK "1234567890123456" // 16 bytes primary master key
#define ESPNOW_QUEUE_SIZE 10
#define ESPNOW_MAXDELAY 1000 / portTICK_PERIOD_MS
// #define CONFIG_ESPNOW_PMK ESPNOW_LMK
#define CONFIG_ESPNOW_PMK "1234567890123456" // 16 bytes primary master key
#define CONFIG_ESPNOW_SEND_COUNT 10
#define CONFIG_ESPNOW_SEND_DELAY 1000 // ms
#define CONFIG_ESPNOW_SEND_LEN 100 // bytes
// #define CONFIG_ESPNOW_ENABLE_LONG_RANGE 1
// #define CONFIG_ESPNOW_SEND_UNICAST 1
// #define CONFIG_ESPNOW_SEND_BROADCAST 1
// #define CONFIG_ESPNOW_SEND_UNICAST_COUNT 10
// #define CONFIG_ESPNOW_SEND_BROADCAST_COUNT 10
// #define CONFIG_ESPNOW_SEND_UNICAST_DELAY 1000 // ms
// #define CONFIG_ESPNOW_SEND_BROADCAST_DELAY 1000 // ms
// #define CONFIG_ESPNOW_SEND_UNICAST_LEN 100 // bytes
// #define CONFIG_ESPNOW_SEND_BROADCAST_LEN 100 // bytes

// #define ESPNOW_MAXDELAY (portMAX_DELAY)
// #define CONFIG_ESPNOW_ENABLE_POWER_SAVE 1
// #define CONFIG_ESPNOW_WAKE_WINDOW 1000
// #define CONFIG_ESPNOW_WAKE_INTERVAL 1000

typedef enum {
    EXAMPLE_ESPNOW_SEND_CB,
    EXAMPLE_ESPNOW_RECV_CB,
} example_espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} example_espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} example_espnow_event_recv_cb_t;

typedef union {
    example_espnow_event_send_cb_t send_cb;
    example_espnow_event_recv_cb_t recv_cb;
} example_espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    example_espnow_event_id_t id;
    example_espnow_event_info_t info;
} example_espnow_event_t;

enum {
    EXAMPLE_ESPNOW_DATA_BROADCAST,
    EXAMPLE_ESPNOW_DATA_UNICAST,
    EXAMPLE_ESPNOW_DATA_MAX,
};

/* User defined field of ESPNOW data in this example. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t payload[0];                   //Real payload of ESPNOW data.
} __attribute__((packed)) example_espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} example_espnow_send_param_t;

void example_wifi_init(void);
esp_err_t example_espnow_init(esp_now_send_cb_t send_cb, esp_now_recv_cb_t recv_cb, bool create_queue_task);
void example_espnow_deinit(example_espnow_send_param_t *send_param);
// int espnow_queue_event(example_espnow_event_t *evt);
esp_err_t espnow_broadcast(uint8_t *buffer, int len);