#pragma once
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"

#define CONFIG_ESPNOW_CHANNEL 1
#define CONFIG_ESPNOW_PMK "1234567890123456" // 16 bytes primary master key

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA

void example_wifi_init(void);
esp_err_t example_espnow_init(esp_now_send_cb_t send_cb, esp_now_recv_cb_t recv_cb);
esp_err_t espnow_broadcast(uint8_t *buffer, int len);