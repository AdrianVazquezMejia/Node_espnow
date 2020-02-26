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
#include "CRC.h"




static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 1024;


#define TXD_PIN (GPIO_NUM_33)
#define RXD_PIN (GPIO_NUM_26)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define RTS_PIN   (25)


#define CTS_PIN   (19)

#define NodeID 0
#define DEFAULT_ID 255
#define BaudaRate 1
#define DEFAULT_BR 8 //115200
#define ROUTING_TABLE_SIZE 255
#define PEER_TABLE_SIZE 255
#define HOLDING_REGISTER_SIZE 513


static const char *TAG = "espnow";
static const char *TAG_MB = "espnow";

 //Queue definitions

static xQueueHandle espnow_queue;

static QueueHandle_t uart1_queue;

static xQueueHandle espnow_Squeue;

static xQueueHandle espnow_Rqueue;

 nvs_handle nvshandle;
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t back_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void espnow_deinit(espnow_send_param_t *send_param);
static uint8_t Peer[6][6];
static uint8_t Node_ID;
static uint8_t BaudaRateID;
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
void vConfigGetNVS(uint8_t *Array , const char *Name){
    esp_err_t err = nvs_flash_init();

	err = nvs_open("storage", NVS_READWRITE, &nvshandle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
		printf("Reading Config from NVS ...\n ");
		size_t size_data = 0;
		int sw = 0;
		if(strcmp(Name, "HoldingRegister") == 0){
			sw = 1;
			size_data =HOLDING_REGISTER_SIZE;
		}
		if(strcmp(Name, "RoutingTable") == 0){
			sw = 2;
			size_data =ROUTING_TABLE_SIZE;
		}
		if(strcmp(Name, "PeerTable") == 0){
			sw = 3;
			size_data =PEER_TABLE_SIZE * ESP_NOW_ETH_ALEN;
			}
		switch(sw){

			case 1:

				err = nvs_get_blob(nvshandle, "HoldingRegister", Array, &size_data);
					switch (err) {
						case ESP_OK:
							printf("Config Holding Register loaded: %d BR and ID %d\n",Array[BaudaRate],Array[NodeID] );
							break;
						case ESP_ERR_NVS_NOT_FOUND:
							printf("The value is not initialized yet!\n");
							break;
						default :
							printf("Error (%s) reading!\n", esp_err_to_name(err));
					}
					break;

			case 2:


			err = nvs_get_blob(nvshandle, "RoutingTable",Array, &size_data);
			switch (err) {
				case ESP_OK:
					printf("Config RoutingTable loaded\n");
					break;
				case ESP_ERR_NVS_NOT_FOUND:
					printf("The RT is not initialized yet!\n");
					break;
				default :
					printf("Error (%s) reading!\n", esp_err_to_name(err));
			}
			break;

			case 3:
			err = nvs_get_blob(nvshandle, "PeerTable",Array, &size_data);
			switch (err) {
				case ESP_OK:
					printf("Peer Table loaded\n");
					break;
				case ESP_ERR_NVS_NOT_FOUND:
					printf("The Peer Table is not initialized yet!\n");
					break;
				default :
					printf("Error (%s) reading!\n", esp_err_to_name(err));
			}
			break;
		}
		nvs_close(nvshandle);
    }
}
void vConfigSetNVS(uint8_t *Array , const char *Name){
    esp_err_t err = nvs_flash_init();
	err = nvs_open("storage", NVS_READWRITE, &nvshandle);
    printf("Reading Config from NVS ...\n ");
    size_t size_data = 0;
    int sw = 0;
    if(strcmp(Name, "HoldingRegister") == 0){
    	sw = 1;
    	size_data = (size_t) HOLDING_REGISTER_SIZE;
    }
    if(strcmp(Name, "RoutingTable") == 0){
        sw = 2;
        size_data =(size_t) ROUTING_TABLE_SIZE;
    }
    if(strcmp(Name, "PeerTable") == 0){
    	sw = 3;
    	size_data = (size_t)(PEER_TABLE_SIZE*ESP_NOW_ETH_ALEN);
        }
    switch(sw){

    	case 1:
	        printf("Updating HR in NVS ... \n");
	        err = nvs_set_blob(nvshandle, "HoldingRegister", Array,size_data);
	        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
		    break;

		case 2:
	        printf("Updating RT in NVS ... \n");
	        err = nvs_set_blob(nvshandle, "RoutingTable", Array,size_data);
	        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
	        break;

		case 3:
	        printf("Updating HR in NVS ... \n");
	        err = nvs_set_blob(nvshandle, "PeerTable", Array,size_data);
	        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
	        break;
		default :
				printf("Error (%s) reading!\n", esp_err_to_name(err));
				break;
    }

    printf("Committing updates in NVS ... ");
    err = nvs_commit(nvshandle);
    printf((err != ESP_OK) ? "Failed!\n" : "Config Done\n");

    // Close
    nvs_close(nvshandle);
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
    if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
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
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *dir)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *dir = buf->dir;
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
    uint8_t HoldingRegister[HOLDING_REGISTER_SIZE] = {0};
    vConfigGetNVS(HoldingRegister,"HoldingRegister");
    buf->Nodeid = HoldingRegister[NodeID];
    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;

    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->dir = send_param->dir;
    buf->crc = crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}
/* Task for send espnow data coming from UART*/
/*void vComGetMac(uint8_t **mac,uint8_t slave ){
	*mac = malloc(ESP_NOW_ETH_ALEN);
	uint8_t RoutingTable[ROUTING_TABLE_SIZE] = {0};
	uint8_t HoldingRegister[HOLDING_REGISTER_SIZE] = {0};
	uint8_t RoutingTable[PEER_TABLE_SIZE][ESP_NOW_ETH_ALEN] = {0};
	uint8_t des_node = 0;
	vConfigGetNVS(RoutingTable,"RoutingTable");
	vConfigGetNVS(HoldingRegister,"HoldingRegister");
	vConfigGetNVS(PeerTable[0],"PeerTable");
	des_node = RoutingTable[slave];
	if (des_node == HoldingRegister[NodeID]){
		memcpy((*mac),back_mac,ESP_NOW_ETH_ALEN);// supose backmac is correctly filled
		printf("ITS AND ANSWER FROM A SLAVE TO MASTER\n");
	}
	else {
		ESP_LOGI(TAG, "ITS FOR NODE %d with MAC: "MACSTR"", des_node, MAC2STR((*mac)));
		memcpy((*mac), PeerTable[des_node],ESP_NOW_ETH_ALEN); //xxx
		//routin to obtain unknown macs
	}
}*/
void espnow_send(void *pvParameter){
	espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
	esp_uart_data_t U_data;
	espnow_data_t *buf = (espnow_data_t *)send_param->buffer;
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
	uint8_t RoutingTable[ROUTING_TABLE_SIZE] = {0};
	uint8_t HoldingRegister[HOLDING_REGISTER_SIZE] = {0};
	uint8_t PeerTable[PEER_TABLE_SIZE*ESP_NOW_ETH_ALEN] = {0};
	uint8_t des_node = 0;
    while(xQueueReceive(espnow_Squeue, &U_data, portMAX_DELAY) == pdTRUE){
    	ESP_LOGI(TAG,"Send Task activated");


    	vConfigGetNVS(RoutingTable,"RoutingTable");
    	vConfigGetNVS(HoldingRegister,"HoldingRegister");
    	vConfigGetNVS(PeerTable,"PeerTable");
    	des_node = RoutingTable[U_data.data[0]];
    	if (des_node == HoldingRegister[NodeID]){
    		memcpy(send_param->dest_mac,back_mac,ESP_NOW_ETH_ALEN);// supose backmac is correctly filled
    		printf("ITS AND ANSWER FROM A SLAVE TO MASTER, RT: %d , slave: %d\n", RoutingTable[U_data.data[0]],U_data.data[0]);
    	}
    	else {
    		memcpy(send_param->dest_mac, PeerTable ,ESP_NOW_ETH_ALEN); //xxx
    		ESP_LOGI(TAG, "ITS FOR NODE %d with MAC: "MACSTR"", des_node, MAC2STR((send_param->dest_mac)));
    	}
    	buf->dir = FORDWARD;
    	if(memcmp(send_param->dest_mac ,back_mac, ESP_NOW_ETH_ALEN ) == 0)
    		buf->dir = BACKWARD;
    	bzero(buf->payload,ESPNOW_PAYLOAD_SIZE);
    	memcpy(buf->payload,U_data.data,U_data.len);
    	buf->data_len = U_data.len;
    	espnow_data_prepare(send_param);
    	ESP_LOGI(TAG, "Send unicast data to: "MACSTR"", MAC2STR(send_param->dest_mac));
    	if (esp_now_send(send_param->dest_mac, send_param->buffer, CONFIG_ESPNOW_SEND_LEN) != ESP_OK) {
    		ESP_LOGE(TAG, "Send error");
    		espnow_deinit(send_param);
    		vTaskDelete(NULL);
    	}
    }
    vTaskDelete(NULL);
}
uint16_t uComGetTransData(int slave){

    uint8_t HoldingRegister[HOLDING_REGISTER_SIZE] ={0}; // value will default to 0, if not set yet in NVS
	vConfigGetNVS(HoldingRegister,"HoldingRegister");
    uint8_t RoutingTable[ROUTING_TABLE_SIZE] ={0};
	vConfigGetNVS(RoutingTable,"RoutingTable");
	uint8_t des_slave = RoutingTable[slave];
	if (HoldingRegister[NodeID] == slave) {
		ESP_LOGI(TAG, "The informations is for ME! node %d\n", slave);
		return NODECONFIG;
	}
	else if (des_slave == HoldingRegister[NodeID]){
			ESP_LOGI(TAG, "The informations is for my RTU! RTU %d\n", slave);
			return SERIAL;
	}
	else {
		ESP_LOGI(TAG, "The informations is NOT for ME! node %d to node %d\n", slave, des_slave);
		return JUMP;
	}
	return -1;
}
void vConfigSetNode(esp_uart_data_t data){
	uint8_t function = data.data[1];
	INT_VAL Address;
	Address.byte.HB = data.data[2];
	Address.byte.LB = data.data[3];
	INT_VAL Value;
	INT_VAL CRC;
	Value.byte.HB = data.data[4];
	Value.byte.LB = data.data[5];
	uint8_t HoldingRegister[HOLDING_REGISTER_SIZE] ={0}; // value will default to 0, if not set yet in NVS
	uint8_t RoutingTable[ROUTING_TABLE_SIZE] ={0};
	vConfigGetNVS(HoldingRegister,"HoldingRegister");
	vConfigGetNVS(RoutingTable,"RoutingTable");
	uint16_t offset = 256;

	if (CRC16(data.data,data.len) == 0){
	switch(function){
	case WRITE_HOLDING_REGISTER:
		if (Address.Val == 0 && Value.Val <=100){
			printf("Node Id not allowed, must be greater than 100\n");
			break;
		}

		HoldingRegister[Address.Val] = Value.Val;
		vConfigSetNVS(HoldingRegister,"HoldingRegister");
		if(Address.Val >=256){
			memcpy(RoutingTable, HoldingRegister+offset, offset);
			vConfigSetNVS(RoutingTable,"RoutingTable");
		}
		printf( "Modifying Holding Register, Value %d  position %d \n",HoldingRegister[Address.Val],Address.Val);
		uart_write_bytes(UART_NUM_1,(const char*)data.data,data.len);
		break;

	case WRITE_COIL:
		printf("Make some coil function stuff\n");
		break;

	case READ_HOLDING:
		printf("Reading HoldingRegister\n");
		data.data[2] = data.data[5]*2;//xxx what if greater than ff number of bytes
		uint8_t inc =3;
		uint8_t data_limit = data.data[5];
		printf("El numero de bytes is %d  y la direccion de inicio es %d\n", data_limit, Address.Val);
		int i = 0;
		while( i < data_limit){
			data.data[inc] = 0;
			inc++;
			data.data[inc] =HoldingRegister[Address.Val+i];
			printf("inc is %d\n", inc);
			inc++;

			i++;
			printf("inc is %d\n", inc);
		}
		CRC.Val = CRC16(data.data,inc);
		data.data[inc] = CRC.byte.LB;
		inc++;
		data.data[inc] = CRC.byte.HB;
		data.len= ++inc;
		printf("AQUI %d bytes limit \n",inc);
		uart_write_bytes(UART_NUM_1,(const char*)data.data, data.len);
	}

	default :
		data.data[1]+= 0x80;
		data.data[2] = 0x01;
		CRC.Val = CRC16(data.data,3);
		data.data[3] = CRC.byte.LB;
		data.data[4] = CRC.byte.HB;
		uart_write_bytes(UART_NUM_1,(const char*)data.data,5);

}
	else {
		ESP_LOGE(TAG_MB," CRC ERROR is %4x", CRC16(data.data,data.len));
		data.data[1]+=0x80;
		data.data[2] = 0x08;
		CRC.Val = CRC16(data.data,3);
		data.data[3] = CRC.byte.LB;
		data.data[4] = CRC.byte.HB;
		uart_write_bytes(UART_NUM_1,(const char*)data.data,5);
	}

}
static void rpeer_espnow_task(void *pvParameter)
{
    espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int recv_magic = 0;
    int ret;
    int Peer_Quantity = 0;
    int count=20;
    esp_uart_data_t U_data;
    vTaskDelay(5000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */

    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
    uint8_t PeerTable[PEER_TABLE_SIZE * ESP_NOW_ETH_ALEN]= {0};
    espnow_data_prepare(send_param);
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        espnow_deinit(send_param);
        vTaskDelete(NULL);
    }
	xTaskCreate(espnow_send, "espnow_send", 2048*4, send_param, 3, NULL);
    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                ESP_LOGI(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
                if (count==0) {
                    break;
                }
                count--;
                /*Delay before send the next data*/
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_RATE_MS);
                }

                espnow_data_prepare(send_param);
                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                /*Create the tasks for communication with UART*/
                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                /*Verify is broadcast*/
                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                espnow_data_t *buf = (espnow_data_t *)recv_cb->data;
                if (buf->dir == FORDWARD){
                memcpy(back_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);//xxx
                ESP_LOGI(TAG, "Back Mac is : "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
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

                        espnow_data_prepare(send_param);
                        send_param->unicast = false;
                        send_param->broadcast = true;
                        if (esp_now_send(broadcast_mac, send_param->buffer, send_param->len) != ESP_OK) {
                            ESP_LOGE(TAG, "Send error");
                            espnow_deinit(send_param);
                            vTaskDelete(NULL);

                        send_param->unicast = true;
                        send_param->broadcast = false;
                       }
                        vConfigGetNVS(PeerTable,"PeerTable");
                        memcpy(PeerTable+(buf->Nodeid)*(ESP_NOW_ETH_ALEN), recv_cb->mac_addr,  ESP_NOW_ETH_ALEN);
                    }
                }
                else if (ret == ESPNOW_DATA_UNICAST) {
            		ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
					U_data.data=buf->payload;
					U_data.len = buf ->data_len;
					uint8_t slave = U_data.data[0];
					uint8_t mode = uComGetTransData(slave);
					switch(mode){
					case SERIAL:
						uart_write_bytes(UART_NUM_1,(const char*) U_data.data ,U_data.len);
					case NODECONFIG:
						vConfigSetNode(U_data);
					case JUMP:
						xQueueSend(espnow_queue, &U_data, portMAX_DELAY);
					}
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
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
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
    send_param->dir = 0; //Maestro 1, esclavo 0
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
    espnow_data_prepare(send_param);

    xTaskCreate(rpeer_espnow_task, "register_peer", 2048*2, send_param, 4, NULL);
    return ESP_OK;
}

static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}

void UARTinit(int BTid) {
	uart_driver_delete(UART_NUM_1);
	int uart_baudarate = 8;
	switch(BTid) {
		case 1:
			uart_baudarate = 9600;
			break;
		case 2:
			uart_baudarate = 14400;
			break;
		case 3:
			uart_baudarate = 19200;
			break;
		case 4:
			uart_baudarate = 28800;
			break;
		case 5:
			uart_baudarate = 38400;
			break;
		case 6:
			uart_baudarate = 57600;
			break;
		case 7:
			uart_baudarate = 76800;
			break;
		case 8:
			uart_baudarate = 115200;
			break;
	}
	ESP_LOGI(TAG, "El baudarate is %d",uart_baudarate );
    const uart_config_t uart_config = {
        .baud_rate = uart_baudarate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, RTS_PIN, CTS_PIN);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2,  TX_BUF_SIZE * 2,  20, &uart1_queue, 0);
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
    }

uint8_t uComDirection(uint8_t *Slave){
uint8_t HoldingRegister[HOLDING_REGISTER_SIZE] = {0};
vConfigGetNVS(HoldingRegister , "HoldingRegister");
printf("%d",*Slave);
	if (HoldingRegister[NodeID] == *Slave){
		printf("ITS FOR ME!\n");
		return NODE;
		}
		else{
			printf("ITS FOR OTHER :-(!\n");
			return EX_SLAVE;
		}
	return -1;
}

static void rx_task(void *arg){
	uint8_t HoldingRegister[HOLDING_REGISTER_SIZE] = {0};
	vConfigGetNVS(HoldingRegister,"HoldingRegister");
    UARTinit(HoldingRegister[BaudaRate]);
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
                    uint8_t info = uComDirection(&dtmp[0]);
                    U_data.data = dtmp;
                    U_data.len = (uint8_t) event.size;
                    ESP_LOGI(RX_TASK_TAG,"Data recivida\n");
                    switch(info){
                    case EX_SLAVE:
                    	// Query of response BACK
                    	xQueueSend(espnow_Squeue,&U_data,(portTickType)portMAX_DELAY);
						break;
                    case NODE:
                    	// TO ME
                    	ESP_LOGI(RX_TASK_TAG,"Node");
                    	vConfigSetNode(U_data);
                    	break;
                    }
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
void vConfigLoad(){
    esp_err_t err = nvs_flash_init();
    printf("IM HERE/n");

	err = nvs_open("storage", NVS_READWRITE, &nvshandle);
	    if (err != ESP_OK) {
	        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	    } else {
	        printf("Done\n");

	        // Read
	        printf("Reading Config from NVS ...\n ");
	        uint8_t HoldingRegister[HOLDING_REGISTER_SIZE] ={0}; // value will default to 0, if not set yet in NVS
	        size_t size_data = sizeof(HoldingRegister);

	        uint8_t RoutingTable[ROUTING_TABLE_SIZE] ={0};
	        size_t size_RT = sizeof(RoutingTable);

	        uint8_t PeerTable[PEER_TABLE_SIZE*ESP_NOW_ETH_ALEN] ={0};
	        size_t size_Peer = sizeof(PeerTable);
	        memset(PeerTable,255, size_Peer);
	        HoldingRegister[NodeID]= DEFAULT_ID;
	        HoldingRegister[BaudaRate]= DEFAULT_BR;
	        uint8_t Peer[6];

	        err = nvs_get_blob(nvshandle, "HoldingRegister", HoldingRegister, &size_data);
	        switch (err) {
	            case ESP_OK:
	                printf("Config Holding Register loaded: %d BR and ID %d\n",HoldingRegister[BaudaRate],HoldingRegister[NodeID] );
	                break;
	            case ESP_ERR_NVS_NOT_FOUND:
	                printf("The value is not initialized yet!\n");
	                break;
	            default :
	                printf("Error (%s) reading!\n", esp_err_to_name(err));
	        }
	        err = nvs_get_blob(nvshandle, "RoutingTable",RoutingTable, &size_RT);
	        switch (err) {
	            case ESP_OK:
	                printf("Config RoutingTable loaded\n");
	                break;
	            case ESP_ERR_NVS_NOT_FOUND:
	                printf("The RT is not initialized yet!\n");
	                break;
	            default :
	                printf("Error (%s) reading!\n", esp_err_to_name(err));
	        }

	        err = nvs_get_blob(nvshandle, "PeerTable",PeerTable, &size_Peer);
	        switch (err) {
	            case ESP_OK:
	                printf("Config Peer Table loaded\n");
	                break;
	            case ESP_ERR_NVS_NOT_FOUND:
	                printf("The Peer Table is not initialized yet!\n");
	                break;
	            default :
	                printf("Error (%s) reading!\n", esp_err_to_name(err));
	        }



	        // Write
	        printf("Updating HR in NVS ... \n");
	        err = nvs_set_blob(nvshandle, "HoldingRegister", HoldingRegister,size_data);
	        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

	        printf("Updating RT in NVS ... \n");
	        err = nvs_set_blob(nvshandle, "RoutingTable", RoutingTable,size_RT);
	        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
	        printf("Updating PT in NVS ... \n");
	        err = nvs_set_blob(nvshandle, "PeerTable", PeerTable,size_Peer);
	        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
	        // Commit written value.
	        // After setting any values, nvs_commit() must be called to ensure changes are written
	        // to flash storage. Implementations may write to storage at other times,
	        // but this is not guaranteed.
	        printf("Committing updates in NVS ... ");
	        err = nvs_commit(nvshandle);
	        printf((err != ESP_OK) ? "Failed!\n" : "Config Done\n");

	        // Close
	        nvs_close(nvshandle);

	        Node_ID = HoldingRegister[NodeID];
	        BaudaRateID = HoldingRegister[BaudaRate];
	    }


}


void app_main()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
   // nvs_flash_erase();
    ESP_ERROR_CHECK( ret );
    esp_log_level_set(TAG, ESP_LOG_INFO);

    vConfigLoad();
   // Create UART tasks
    xTaskCreate(rx_task, "uart_rx_task", 2048*2, NULL, configMAX_PRIORITIES, NULL);
    //uint8_t ex[8] = {0xff, 0x06, 0x01, 0x00, 0x00, 0xc8, 0x9c, 0x7e};
   // printf("CRC  is : %4x \n", CRC16(ex,8));
    //Create ESPnow Tasks

    wifi_init();
    espnow_init();


}
