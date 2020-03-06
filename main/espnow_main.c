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
#include "led.h"
//#include "format_factory.h"




static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 1024;


#define TXD_PIN 25//(GPIO_NUM_33)
#define RXD_PIN 14//14//(GPIO_NUM_26)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define RTS_PIN   27//(25)


#define CTS_PIN   (19)

#define NodeID 0
#define DEFAULT_ID 255
#define BaudaRate 1
#define DEFAULT_BR 10 //115200
#define ROUTING_TABLE_SIZE 512
#define PEER_TABLE_SIZE 256
#define HOLDING_REGISTER_SIZE 513
#define OFFSET	256


#define GPIO_INPUT_IO_0     0 //xxx
#define GPIO_OUTPUT_IO_19	2//19
#define ESP_INTR_FLAG_DEFAULT 0

static const char *TAG = "espnow";
static const char *TAG_MB = "espnow";

 //Queue definitions

static xQueueHandle espnow_queue;

static QueueHandle_t uart1_queue;

static xQueueHandle espnow_Squeue;

static xQueueHandle espnow_Rqueue;


static SemaphoreHandle_t xSemaphore = NULL;

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
							;
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
    	size_data = PEER_TABLE_SIZE*ESP_NOW_ETH_ALEN;
        }
    switch(sw){

    	case 1:
	        err = nvs_set_blob(nvshandle, "HoldingRegister", Array,size_data);
		    break;

		case 2:
	        err = nvs_set_blob(nvshandle, "RoutingTable", Array,size_data);
	        break;

		case 3:
	        err = nvs_set_blob(nvshandle, "PeerTable", Array,size_data);
	        break;
		default :
				printf("Error (%s) reading!\n", esp_err_to_name(err));
				break;
    }

    err = nvs_commit(nvshandle);
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

void RegisterPeer(uint8_t mac[ESP_NOW_ETH_ALEN]){
	esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
	if (peer == NULL) {
		ESP_LOGE(TAG, "Malloc peer information fail");
		return;
	}
	memset(peer, 0, sizeof(esp_now_peer_info_t));
	peer->channel = CONFIG_ESPNOW_CHANNEL;
	peer->ifidx = ESPNOW_WIFI_IF;
	peer->encrypt = true;
	memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
	memcpy(peer->peer_addr, mac, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK( esp_now_add_peer(peer) );
	ESP_LOGI(TAG, "MAC succefully "MACSTR" added ", MAC2STR(mac));
	free(peer);
}

void vEspnowGetOldPeers(void){
	ESP_LOGI(TAG, "Registering");
	uint8_t PeerTable[(PEER_TABLE_SIZE*ESP_NOW_ETH_ALEN)] = {0};
	uint8_t RoutingTable[ROUTING_TABLE_SIZE] = {0};
	uint8_t mac[ESP_NOW_ETH_ALEN] = {0};
	vConfigGetNVS(PeerTable,"PeerTable");
	vConfigGetNVS(RoutingTable,"RoutingTable");
	for (uint16_t j = 0 ; j <=255 ; j++){
		memcpy(mac,PeerTable + (RoutingTable[j]* ESP_NOW_ETH_ALEN),ESP_NOW_ETH_ALEN);//xxx
		if((RoutingTable[j]!=0)&&(memcmp(PeerTable + RoutingTable[j]* ESP_NOW_ETH_ALEN, broadcast_mac , ESP_NOW_ETH_ALEN)) !=0){
			RegisterPeer(mac);
		}
	}
}

void espnow_send(void *pvParameter){
	espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
	esp_uart_data_t U_data;
	espnow_data_t *buf = (espnow_data_t *)send_param->buffer;
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
	uint8_t RoutingTable[ROUTING_TABLE_SIZE] = {0};
	uint8_t HoldingRegister[HOLDING_REGISTER_SIZE] = {0};
	uint8_t PeerTable[PEER_TABLE_SIZE*ESP_NOW_ETH_ALEN] ={0};
	uint8_t des_node = 0;
	uint16_t posicion = 0;
    while(xQueueReceive(espnow_Squeue, &U_data, portMAX_DELAY) == pdTRUE){
    	ESP_LOGI(TAG,"Send Task activated");


    	vConfigGetNVS(RoutingTable,"RoutingTable");
    	vConfigGetNVS(HoldingRegister,"HoldingRegister");
	   	vConfigGetNVS(PeerTable,"PeerTable");
    	des_node = RoutingTable[U_data.data[0]];
    	send_param->dir  = FORDWARD;
    	if ((des_node == HoldingRegister[NodeID])||(U_data.dir == BACKWARD)){
        	des_node = RoutingTable[OFFSET+U_data.data[0]];
    		memcpy(send_param->dest_mac,PeerTable+(ESP_NOW_ETH_ALEN * des_node),ESP_NOW_ETH_ALEN);// supose backmac is correctly filled
    		ESP_LOGI(TAG,"ANSWERING TO MASTER  from node :%d, and slave: %d\n", RoutingTable[OFFSET+U_data.data[0]],U_data.data[0]);
    		send_param->dir = BACKWARD;
    	}
    	else {
    	   	posicion = ESP_NOW_ETH_ALEN * des_node;
    		memcpy(send_param->dest_mac, PeerTable + posicion,ESP_NOW_ETH_ALEN); //xxx
    		ESP_LOGI(TAG, "ITS FOR NODE %d with MAC: "MACSTR"", des_node, MAC2STR(send_param->dest_mac));
    	}
    	bzero(buf->payload,ESPNOW_PAYLOAD_SIZE);
    	memcpy(buf->payload,U_data.data,U_data.len);
    	buf->data_len = U_data.len;
    	espnow_data_prepare(send_param);
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
		return NODECONFIG;
	}
	else if (des_slave == HoldingRegister[NodeID]){
			return SERIAL;
	}
	else {
		return JUMP;
	}
	return -1;
}

void UARTinit(int BTid) {
	uart_driver_delete(UART_NUM_1);
	int uart_baudarate = 115200;
	switch(BTid) {
		case 0:
			uart_baudarate = 300;
		break;
		case 1:
			uart_baudarate = 600;
			break;
		case 2:
			uart_baudarate = 1200;
			break;
		case 3:
			uart_baudarate = 2400;
			break;
		case 4:
			uart_baudarate = 4800;
			break;
		case 5:
			uart_baudarate = 9600;
			break;
		case 6:
			uart_baudarate = 14400;
			break;
		case 7:
			uart_baudarate = 19200;
			break;
		case 8:
			uart_baudarate = 38400;
			break;
		case 9:
			uart_baudarate = 56000;
			break;
		case 10:
			uart_baudarate = 115200;
			break;
		case 11:
			uart_baudarate = 128000;
			break;
		case 12:
			uart_baudarate = 256000;
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
	if (HoldingRegister[NodeID] == *Slave){
		return NODE;
		}
		else{
			return EX_SLAVE;
		}
	return -1;
}

void vConfigSetNode(esp_uart_data_t data, uint8_t dir){
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
			ESP_LOGI(TAG,"Node Id not allowed, must be greater than 100\n");
			break;
		}

		HoldingRegister[Address.Val] = Value.Val;
		vConfigSetNVS(HoldingRegister,"HoldingRegister");
		if(Address.Val >=256){
			RoutingTable[Address.Val-offset] = HoldingRegister[Address.Val];
			vConfigSetNVS(RoutingTable,"RoutingTable");
		}
		ESP_LOGI(TAG, "Modifying Holding Register, Value %d  position %d \n",HoldingRegister[Address.Val],Address.Val);
		if (dir == ESP_NOW){
			data.dir = BACKWARD;
			xQueueSend(espnow_queue, &data, portMAX_DELAY);
		}
		else
		uart_write_bytes(UART_NUM_1,(const char*)data.data,data.len);
		vNotiUart();
		if(Address.Val == BaudaRate){
			vTaskDelay(500);
			UARTinit(HoldingRegister[BaudaRate]);
		}
		break;

	case WRITE_COIL:
		if ((Address.Val == 0)&&(Value.Val ==0xFF00)){
		    printf("Restarting \n");
		    if (dir == ESP_NOW){
				data.dir = BACKWARD;
				xQueueSend(espnow_queue, &data, portMAX_DELAY);
			}
			else
		    uart_write_bytes(UART_NUM_1,(const char*)data.data,data.len);
			vNotiUart();
			vTaskDelay(10);
		    fflush(stdout);
		    esp_restart();
			break;
		}
		break;

	case READ_HOLDING:
		ESP_LOGI(TAG,"Reading HoldingRegister\n");
		data.data[2] = data.data[5]*2;//xxx what if greater than ff number of bytes
		uint8_t inc =3;
		uint8_t data_limit = data.data[5];
		int i = 0;
		while( i < data_limit){
			data.data[inc] = 0;
			inc++;
			data.data[inc] =HoldingRegister[Address.Val+i];
			inc++;
			i++;
		}
		CRC.Val = CRC16(data.data,inc);
		data.data[inc] = CRC.byte.LB;
		inc++;
		data.data[inc] = CRC.byte.HB;
		data.len= ++inc;
		if (dir == ESP_NOW){
			data.dir = BACKWARD;
			xQueueSend(espnow_queue, &data, portMAX_DELAY);
		}
		else
		uart_write_bytes(UART_NUM_1,(const char*)data.data, data.len);
		vNotiUart();
		break;
	default :
		data.data[1]+= 0x80;
		data.data[2] = 0x01;
		CRC.Val = CRC16(data.data,3);
		data.data[3] = CRC.byte.LB;
		data.data[4] = CRC.byte.HB;
		if (dir == ESP_NOW){
			data.dir = BACKWARD;
			xQueueSend(espnow_queue, &data, portMAX_DELAY);
		}
		else
		uart_write_bytes(UART_NUM_1,(const char*)data.data,5);
		vNotiUart();
	}



}
	else {
		ESP_LOGE(TAG_MB," CRC ERROR is %4x", CRC16(data.data,data.len));
		data.data[1]+=0x80;
		data.data[2] = 0x08;
		CRC.Val = CRC16(data.data,3);
		data.data[3] = CRC.byte.LB;
		data.data[4] = CRC.byte.HB;
		if (dir == ESP_NOW){
			data.dir = BACKWARD;
			xQueueSend(espnow_queue, &data, portMAX_DELAY);
		}
		else
		uart_write_bytes(UART_NUM_1,(const char*)data.data,5);
		vNotiUart();
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
    uint8_t mac[ESP_NOW_ETH_ALEN] = {0};
    /* Start sending broadcast ESPNOW data. */

    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
    uint8_t PeerTable[PEER_TABLE_SIZE * ESP_NOW_ETH_ALEN]= {0};
	uint8_t RoutingTable[ROUTING_TABLE_SIZE] = {0};
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
                        vConfigGetNVS(PeerTable,"PeerTable");
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
                        memcpy(PeerTable+(buf->Nodeid)*(ESP_NOW_ETH_ALEN), recv_cb->mac_addr,  ESP_NOW_ETH_ALEN);
                        vConfigSetNVS(PeerTable,"PeerTable");
                		memcpy(mac,PeerTable + (buf->Nodeid)*(ESP_NOW_ETH_ALEN),ESP_NOW_ETH_ALEN);
                        ESP_LOGI(TAG, "MAC added is:  "MACSTR" from Node %d is the position %d", MAC2STR(mac),buf->Nodeid, (buf->Nodeid)*(ESP_NOW_ETH_ALEN));
                    }
                }
                else if (ret == ESPNOW_DATA_UNICAST) {
            		ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
					U_data.data = buf->payload;
					U_data.len = buf->data_len;
            		uint8_t slave = U_data.data[0];
                	if (buf->dir == FORDWARD){
                    vConfigGetNVS(RoutingTable,"RoutingTable");
                    RoutingTable[slave+OFFSET] = buf->Nodeid;
                    RoutingTable[buf->Nodeid] = buf->Nodeid;
                    vConfigSetNVS(RoutingTable,"RoutingTable");
            		ESP_LOGI(TAG, "BackMAC  data from: "MACSTR",  of slave %d ", MAC2STR(recv_cb->mac_addr),slave);
                    }
					uint8_t mode = uComGetTransData(slave);
					switch(buf->dir){
						case FORDWARD:
							switch(mode){
							case SERIAL:
								ESP_LOGI(TAG,"SERIAL");
								uart_write_bytes(UART_NUM_1,(const char*) U_data.data ,U_data.len);
								vNotiUart();
								break;
							case NODECONFIG:
								ESP_LOGI(TAG,"NODECONFIG");
								vConfigSetNode(U_data,ESP_NOW);
								break;
							case JUMP:
								U_data.dir = buf ->dir;
								ESP_LOGI(TAG,"JUMP DIR %d: ", buf ->dir );
								xQueueSend(espnow_Squeue, &U_data, portMAX_DELAY);
							}
							break;
							case BACKWARD:
								vConfigGetNVS(RoutingTable,"RoutingTable");
		                        vConfigGetNVS(PeerTable,"PeerTable");
								memcpy(back_mac, PeerTable+(RoutingTable[slave+OFFSET]*ESP_NOW_ETH_ALEN), ESP_NOW_ETH_ALEN);
								if(memcmp(back_mac, broadcast_mac, ESP_NOW_ETH_ALEN)==0){
									uart_write_bytes(UART_NUM_1,(const char*) U_data.data ,U_data.len);
									vNotiUart();
									ESP_LOGI(TAG,"SERIAL response");
								}
								else {
									U_data.dir = buf ->dir;
									ESP_LOGI(TAG,"JUMP BACK %d: ", buf ->dir );
									xQueueSend(espnow_Squeue, &U_data, portMAX_DELAY);
								};
					}

                }
                else{
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"  con ret : %d " , MAC2STR(recv_cb->mac_addr),ret);
                }
                free(recv_cb->data);
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
    vEspnowGetOldPeers();
    xTaskCreate(rpeer_espnow_task, "register_peer", 2048*4, send_param, 4, NULL);
    return ESP_OK;
}

static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
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
                    	vConfigSetNode(U_data,UART);
                    	break;
                    }
                    vNotiUart();
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
	        HoldingRegister[NodeID]= DEFAULT_ID;
	        HoldingRegister[BaudaRate]= DEFAULT_BR;


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

void vConfigFormatFactory( void ){

	uint8_t HoldingRegister[HOLDING_REGISTER_SIZE] ={0};
	uint8_t RoutingTable[ROUTING_TABLE_SIZE] ={0};
	uint8_t PeerTable[PEER_TABLE_SIZE*ESP_NOW_ETH_ALEN] ={0};
	memset(PeerTable,0xff,PEER_TABLE_SIZE*ESP_NOW_ETH_ALEN);
	bzero(RoutingTable,ROUTING_TABLE_SIZE);
	HoldingRegister[NodeID] = DEFAULT_ID;
	HoldingRegister[BaudaRate] = DEFAULT_BR;
	vConfigSetNVS(HoldingRegister,"HoldingRegister");
	vConfigSetNVS(RoutingTable,"RoutingTable");
	vConfigSetNVS(PeerTable,"PeerTable");
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
   // uint32_t gpio_num = (uint32_t) arg;
    //xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken);
}

void FormatFactory(void *arg){

    gpio_config_t io_conf;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = 1ULL<<GPIO_INPUT_IO_0;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_NEGEDGE);

    //create a queue to handle gpio event from isr
    //gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    xSemaphore = xSemaphoreCreateBinary();
    //install gpio isr service
     gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
     //hook isr handler for specific gpio pin
     gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
     //remove isr handler for gpio number.
     gpio_isr_handler_remove(GPIO_INPUT_IO_0);
     //hook isr handler for specific gpio pin again
     gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
	int t = 0;
	//uint32_t io_num;
	ESP_LOGI(TAG, "FORMAT FACTORY TASK");

	while(xSemaphoreTake( xSemaphore, portMAX_DELAY) == pdTRUE){//xxx
		while(gpio_get_level(GPIO_INPUT_IO_0)==0){
			vTaskDelay(10/portTICK_RATE_MS);
			t++;
			if (t == 500){
				printf("Format factory\n");
				vConfigFormatFactory();
				vTaskDelay(1000/portTICK_RATE_MS);
			    printf("Restarting \n");
			    fflush(stdout);
			    esp_restart();
				break;
			}
		}
		t = 0;
        vTaskDelay(1000/portTICK_RATE_MS);
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
    //nvs_flash_erase();
    ESP_ERROR_CHECK( ret );
    //esp_log_level_set(TAG, ESP_LOG_INFO);
    vConfigLoad();
   // Create UART tasks
    xTaskCreatePinnedToCore(rx_task, "uart_rx_task", 2048*2, NULL, configMAX_PRIORITIES, NULL,1);
    //Create ESPnow Tasks
    wifi_init();
    espnow_init();
    xTaskCreatePinnedToCore(FormatFactory, "FormatFactory", 2048*3, NULL, configMAX_PRIORITIES, NULL,0);
    vNotiLEDinit();
}
