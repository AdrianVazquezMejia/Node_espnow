/* ESPnow Node

The code of this project and all the associate files are classificated and has intelectual property
(Corporacion Estelio). Any use without authorization is forbidden. All rights reserved.


Author: Adrian Vazquez
e-mail: aavm0000@gmail.com
Date: 12-17-2019
*/

/*
	This code make bidirectional communication using one UART and the ESPnow WiFi feature.
	This is oriented to MODBUS RTU protocol. The data comes either from the ESPnow network or serial,
	then It is transmitted  either to serial or to the ESP now network.

	We use tasks for the UART and for the ESPnow feature, Queues are used to pass data between tasks.

	This code works either if the MODBUS master is connected or a slave.
*/

/* Nodo ESPnow

El código de este proyecto y todos los archivos asociados están clasificados y tienen propiedad intelectual.
(Corporación Estelio). Cualquier uso sin autorización está prohibido. Todos los derechos reservados.


Autor: Adrian Vazquez
correo electrónico: aavm0000@gmail.com
Fecha: 17/12/2019
* /

/ *
Este código establece comunicación bidireccional utilizando un UART y la función ESPnow WiFi.
Esto está orientado al protocolo MODBUS RTU. Los datos provienen de la red ESPnow o de serial,
entonces se transmite a serie o a la red ESPnow

Se usan tareas para UART y para la función ESPnow, las colas se usan para pasar datos entre tareas.

Este código funciona si el maestro MODBUS está conectado o es esclavo.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "rom/ets_sys.h"
#include "rom/crc.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"
#include "espnow.h"



static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 1024;


#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

static const char *TAG = "espnow_example";
 //Queue definitions

static xQueueHandle espnow_queue;

static QueueHandle_t uart1_queue;

static xQueueHandle espnow_Squeue;

static xQueueHandle espnow_Rqueue;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
//static uint8_t Node1_mac[ESP_NOW_ETH_ALEN] = { 0xd8 , 0xa0 , 0x1d ,0x69 , 0xf3, 0x04};
//static uint8_t Node2_mac[ESP_NOW_ETH_ALEN] = { 0x24 , 0x6f , 0x28 ,0x24 , 0xac, 0x74};
static uint8_t Node0_mac[ESP_NOW_ETH_ALEN] = { 0xd8 , 0xa0 , 0x1d ,0x69 , 0xe9, 0xf0};

static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void espnow_deinit(espnow_send_param_t *send_param);
static uint8_t Peer[6][6];
static esp_err_t example_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "WiFi started");
        break;
    default:
        break;
    }
    return ESP_OK;
}

/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(example_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

    /* In order to simplify , channel is set after WiFi started.
     * This is not necessary in real application if the two devices have
     * been already on the same channel.
     */
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0) );

#if CONFIG_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }

    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = crc16_le(UINT16_MAX, (uint8_t const *)buf,data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void espnow_data_prepare(espnow_send_param_t *send_param)
{
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;
    assert(send_param->len >= sizeof(espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    buf->crc = crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}
/* Task for send espnow data coming from UART*/
void espnow_send(void *pvParameter){
	espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
	esp_uart_data_t U_data;
	espnow_data_t *buf = (espnow_data_t *)send_param->buffer;
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
    while(xQueueReceive(espnow_Squeue, &U_data, portMAX_DELAY) == pdTRUE){
    	//memcpy(Peer[1],Node1_mac,ESP_NOW_ETH_ALEN);//XXX
    	//memcpy(send_param->dest_mac , Peer[1],ESP_NOW_ETH_ALEN);
    	bzero(buf->payload,ESPNOW_PAYLOAD_SIZE);
    	memcpy(buf->payload,U_data.data,U_data.len);
    	buf->data_len = U_data.len;
    	memcpy(send_param->dest_mac,Peer[1],ESP_NOW_ETH_ALEN);
    	espnow_data_prepare(send_param);

    	ESP_LOGI(TAG, "Send unicast data to: "MACSTR"", MAC2STR(Peer[1]));
    	if (esp_now_send(Peer[1], send_param->buffer, CONFIG_ESPNOW_SEND_LEN) != ESP_OK) {
    		ESP_LOGE(TAG, "Send error");
    		espnow_deinit(send_param);
    		vTaskDelete(NULL);
    	}
/*    	memcpy(send_param->dest_mac,Peer[2],ESP_NOW_ETH_ALEN);
    	espnow_data_prepare(send_param);
    	ESP_LOGI(TAG, "Send unicast data to: "MACSTR"", MAC2STR(Peer[2]));
    	if (esp_now_send(Peer[2], send_param->buffer, CONFIG_ESPNOW_SEND_LEN) != ESP_OK) {
    		ESP_LOGE(TAG, "Send error");
    		espnow_deinit(send_param);
    		vTaskDelete(NULL);
    	}*/



    }
    vTaskDelete(NULL);
}

static void rpeer_espnow_task(void *pvParameter)
{
    espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int recv_magic = 0;
    int ret;
    int Peer_Quantity = 3;
    int count=20;
    esp_uart_data_t U_data;
    vTaskDelay(5000 / portTICK_RATE_MS);

    /* Start sending broadcast ESPNOW data. */
    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
    xTaskCreate(espnow_send, "espnow_send", 1024*2, send_param, 3, NULL);
    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {//XXX add peers manually
        switch (evt.id) {
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                ESP_LOGI(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                /*Verify is broadcast*/
                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                espnow_data_t *buf = (espnow_data_t *)recv_cb->data;

                free(recv_cb->data);
                if (ret == ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        Peer_Quantity++;
                        memcpy(Peer[ Peer_Quantity ], recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_LOGI(TAG, "Peer %dth added,  MAC: "MACSTR"",Peer_Quantity, MAC2STR(Peer[1]));
                        free(peer);
                        /* Send the response to the new peer, for register myself with it*/
                        if (count==0) {
                        send_param->unicast = false;
                        send_param->broadcast = true;
                        if (esp_now_send(broadcast_mac, send_param->buffer, send_param->len) != ESP_OK) {
                            ESP_LOGE(TAG, "Send error");
                            espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        send_param->unicast = true;
                        send_param->broadcast = false;
                    }}//XXX
                }
                else if (ret == ESPNOW_DATA_UNICAST) {
            		ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
					U_data.data=buf->payload;
					U_data.len = buf ->data_len;
					uart_write_bytes(UART_NUM_1, (const char*) U_data.data, U_data.len);

                }
                else{
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"  con ret : %d " , MAC2STR(recv_cb->mac_addr),ret);
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }

    }

}

static esp_err_t espnow_init(void)
{
    espnow_send_param_t *send_param;

    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    espnow_Squeue = xQueueCreate(RX_BUF_SIZE, sizeof(esp_uart_data_t));
    espnow_Rqueue = xQueueCreate(RX_BUF_SIZE, sizeof(esp_uart_data_t));
    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /*Adding to Peer list*/
    memcpy(Peer[1], Node0_mac, ESP_NOW_ETH_ALEN);
//  memcpy(Peer[2], Node2_mac, ESP_NOW_ETH_ALEN);

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = true;
    /*Peer node 1 : MAC1*/
/*    memcpy(peer->peer_addr, Node1_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    ESP_LOGI(TAG, "Peer added,  MAC: "MACSTR"", MAC2STR(peer->peer_addr))*/;
    /*Peer node 2 : MAC2*/
/*    memcpy(peer->peer_addr, Node2_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    ESP_LOGI(TAG, "Peer added,  MAC: "MACSTR"", MAC2STR(peer->peer_addr));*/

    memcpy(peer->peer_addr, Node0_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    ESP_LOGI(TAG, "Peer added,  MAC: "MACSTR"", MAC2STR(peer->peer_addr));
    free(peer);



    /* Initialize sending parameters. */
    send_param = malloc(sizeof(espnow_send_param_t));
    memset(send_param, 0, sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = 0; //Maestro 1, esclavo 0
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;//XXX
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);//XXX
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
    espnow_data_prepare(send_param);

    xTaskCreate(rpeer_espnow_task, "register_peer", 2048, send_param, 4, NULL);
    return ESP_OK;
}

static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}

void UARTinit(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2,  TX_BUF_SIZE * 2,  20, &uart1_queue, 0);

    }

static void rx_task(void *arg)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    esp_uart_data_t U_data;
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RX_BUF_SIZE);
            ESP_LOGI(RX_TASK_TAG, "uart[%d] event:", UART_NUM_1);
            switch(event.type) {
                //Event of UART receving data
                case UART_DATA:
                    ESP_LOGI(RX_TASK_TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM_1, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(RX_TASK_TAG, "[DATA EVT]:");
                    U_data.data = dtmp;
                    U_data.len = (uint8_t) event.size;
                    ESP_LOGI(RX_TASK_TAG,"Data recivida\n");
                    xQueueSend(espnow_Squeue,&U_data,(portTickType)portMAX_DELAY);
                    ESP_LOGI(RX_TASK_TAG,"Y enviada a la cola \n");
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(RX_TASK_TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(RX_TASK_TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(RX_TASK_TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(RX_TASK_TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(RX_TASK_TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(RX_TASK_TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}



void app_main()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    esp_log_level_set(TAG, ESP_LOG_INFO);
    UARTinit();
    // Create UART tasks
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    //Create ESPnow Tasks
    wifi_init();
    espnow_init();

}
