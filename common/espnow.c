#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#include "espnow.h"

static const char *TAG = "espnow";

static QueueHandle_t s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
// static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

// static void example_espnow_deinit(example_espnow_send_param_t *send_param);

void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

// #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
// #endif
}


// static void example_espnow_task(void *pvParameter)
// {
//     example_espnow_event_t evt;
//     uint8_t recv_state = 0;
//     uint16_t recv_seq = 0;
//     int recv_magic = 0;
//     bool is_broadcast = false;
//     int ret;

//     vTaskDelay(5000 / portTICK_PERIOD_MS);
//     ESP_LOGI(TAG, "Start sending broadcast data");

//     // /* Start sending broadcast ESPNOW data. */
//     // example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
//     // if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
//     //     ESP_LOGE(TAG, "Send error");
//     //     example_espnow_deinit(send_param);
//     //     vTaskDelete(NULL);
//     // }

//     while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
//         switch (evt.id) {
//             case EXAMPLE_ESPNOW_SEND_CB:
//             {
//                 example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
//                 is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

//                 ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

//                 if (is_broadcast && (send_param->broadcast == false)) {
//                     break;
//                 }

//                 if (!is_broadcast) {
//                     send_param->count--;
//                     if (send_param->count == 0) {
//                         ESP_LOGI(TAG, "Send done");
//                         example_espnow_deinit(send_param);
//                         vTaskDelete(NULL);
//                     }
//                 }

//                 /* Delay a while before sending the next data. */
//                 if (send_param->delay > 0) {
//                     vTaskDelay(send_param->delay/portTICK_PERIOD_MS);
//                 }

//                 ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

//                 memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
//                 example_espnow_data_prepare(send_param);

//                 /* Send the next data after the previous data is sent. */
//                 if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
//                     ESP_LOGE(TAG, "Send error");
//                     example_espnow_deinit(send_param);
//                     vTaskDelete(NULL);
//                 }
//                 break;
//             }
//             case EXAMPLE_ESPNOW_RECV_CB:
//             {
//                 example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

//                 ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
//                 free(recv_cb->data);
//                 if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
//                     ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

//                     /* If MAC address does not exist in peer list, add it to peer list. */
//                     if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
//                         esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
//                         if (peer == NULL) {
//                             ESP_LOGE(TAG, "Malloc peer information fail");
//                             example_espnow_deinit(send_param);
//                             vTaskDelete(NULL);
//                         }
//                         memset(peer, 0, sizeof(esp_now_peer_info_t));
//                         peer->channel = CONFIG_ESPNOW_CHANNEL;
//                         peer->ifidx = ESPNOW_WIFI_IF;
//                         peer->encrypt = true;
//                         memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
//                         memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
//                         ESP_ERROR_CHECK( esp_now_add_peer(peer) );
//                         free(peer);
//                     }

//                     /* Indicates that the device has received broadcast ESPNOW data. */
//                     if (send_param->state == 0) {
//                         send_param->state = 1;
//                     }

//                     /* If receive broadcast ESPNOW data which indicates that the other device has received
//                      * broadcast ESPNOW data and the local magic number is bigger than that in the received
//                      * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
//                      * ESPNOW data.
//                      */
//                     if (recv_state == 1) {
//                         /* The device which has the bigger magic number sends ESPNOW data, the other one
//                          * receives ESPNOW data.
//                          */
//                         if (send_param->unicast == false && send_param->magic >= recv_magic) {
//                     	    ESP_LOGI(TAG, "Start sending unicast data");
//                     	    ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

//                     	    /* Start sending unicast ESPNOW data. */
//                             memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
//                             example_espnow_data_prepare(send_param);
//                             if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
//                                 ESP_LOGE(TAG, "Send error");
//                                 example_espnow_deinit(send_param);
//                                 vTaskDelete(NULL);
//                             }
//                             else {
//                                 send_param->broadcast = false;
//                                 send_param->unicast = true;
//                             }
//                         }
//                     }
//                 }
//                 else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
//                     ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

//                     /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
//                     send_param->broadcast = false;
//                 }
//                 else {
//                     ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
//                 }
//                 break;
//             }
//             default:
//                 ESP_LOGE(TAG, "Callback type error: %d", evt.id);
//                 break;
//         }
//     }
// }

 esp_err_t example_espnow_init(esp_now_send_cb_t send_cb, esp_now_recv_cb_t recv_cb, bool create_queue_task)
{
    // example_espnow_send_param_t *send_param;

    if (create_queue_task)
    {
        s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
        if (s_example_espnow_queue == NULL) {
            ESP_LOGE(TAG, "Create mutex fail");
            return ESP_FAIL;
        }
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    if (send_cb != NULL) {
        ESP_ERROR_CHECK( esp_now_register_send_cb(send_cb) );
    }
    if (recv_cb != NULL) {
        ESP_ERROR_CHECK( esp_now_register_recv_cb(recv_cb) );
    }
    // ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    // ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    // /* Initialize sending parameters. */
    // send_param = malloc(sizeof(example_espnow_send_param_t));
    // if (send_param == NULL) {
    //     ESP_LOGE(TAG, "Malloc send parameter fail");
    //     vSemaphoreDelete(s_example_espnow_queue);
    //     esp_now_deinit();
    //     return ESP_FAIL;
    // }
    // memset(send_param, 0, sizeof(example_espnow_send_param_t));
    // send_param->unicast = false;
    // send_param->broadcast = true;
    // send_param->state = 0;
    // send_param->magic = esp_random();
    // send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    // send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    // send_param->len = CONFIG_ESPNOW_SEND_LEN;
    // send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    // if (send_param->buffer == NULL) {
    //     ESP_LOGE(TAG, "Malloc send buffer fail");
    //     free(send_param);
    //     vSemaphoreDelete(s_example_espnow_queue);
    //     esp_now_deinit();
    //     return ESP_FAIL;
    // }
    // memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    // //example_espnow_data_prepare(send_param);

    // // example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    // if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
    //     ESP_LOGE(TAG, "Send error");
    //     example_espnow_deinit(send_param);
    //     vTaskDelete(NULL);
    // }

    // if (create_queue_task) {
    //     /* Create a task to handle ESPNOW sending. */
    //     xTaskCreate(example_espnow_task, "example_espnow_task", 2048, send_param, 4, NULL);
    //     ESP_LOGI(TAG, "Create example espnow task");
    // }

    return ESP_OK;
}

 void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_example_espnow_queue);
    esp_now_deinit();
}

// int espnow_queue_event(example_espnow_event_t *evt)
// {
//     // if (xQueueSend(s_example_espnow_queue, evt, ESPNOW_MAXDELAY) != pdTRUE) {
//     //     ESP_LOGW(TAG, "Send send queue fail");
//     // }
//     return xQueueSend(s_example_espnow_queue, evt, ESPNOW_MAXDELAY);
// }

esp_err_t espnow_broadcast(uint8_t *buffer, int len)
{
    // example_espnow_send_param_t *send_param;

    // /* Initialize sending parameters. */
    // send_param = malloc(sizeof(example_espnow_send_param_t));
    // if (send_param == NULL) {
    //     ESP_LOGE(TAG, "Malloc send parameter fail");
    //     vSemaphoreDelete(s_example_espnow_queue);
    //     esp_now_deinit();
    //     return ESP_FAIL;
    // }
    // memset(send_param, 0, sizeof(example_espnow_send_param_t));
    // send_param->unicast = false;
    // send_param->broadcast = true;
    // send_param->state = 0;
    // send_param->magic = esp_random();
    // send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    // send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    // send_param->len = CONFIG_ESPNOW_SEND_LEN;
    // send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    // if (send_param->buffer == NULL) {
    //     ESP_LOGE(TAG, "Malloc send buffer fail");
    //     free(send_param);
    //     vSemaphoreDelete(s_example_espnow_queue);
    //     esp_now_deinit();
    //     return ESP_FAIL;
    // }
    // memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    // //example_espnow_data_prepare(send_param);

    // // example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    // if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
    //     ESP_LOGE(TAG, "Send error");
    //     example_espnow_deinit(send_param);
    //     vTaskDelete(NULL);
    // }

    return esp_now_send(s_example_broadcast_mac, buffer, len);
}