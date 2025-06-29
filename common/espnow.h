#pragma once
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"

#include "pmk.h"

#define CONFIG_ESPNOW_CHANNEL 1

#define CONFIG_ESPNOW_ENABLE_POWER_SAVE 0
#define CONFIG_ESPNOW_WAKE_WINDOW 1000
#define CONFIG_ESPNOW_WAKE_INTERVAL 1000

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA

enum {
    TYPE_STATUS_UPDATE = 0x01, // Button status update
    TYPE_ACK = 0x02, // Acknowledgment message
};

typedef struct Payload {
    uint8_t type; // Type of message, e.g., TYPE_STATUS_UPDATE
    uint8_t button_id; // 0x01 for button status
    uint8_t status; // 0 for released, 1 for pressed
    uint8_t msg_id; // ID of the sender, used for acknowledgment
} Payload;

#ifdef __cplusplus
extern "C" {
#endif

void example_nvs_init(void);
void example_wifi_init(void);
esp_err_t example_espnow_init(esp_now_send_cb_t send_cb, esp_now_recv_cb_t recv_cb);
esp_err_t espnow_broadcast(uint8_t *buffer, int len);
void example_espnow_deinit(void);

#ifdef __cplusplus
}
#endif