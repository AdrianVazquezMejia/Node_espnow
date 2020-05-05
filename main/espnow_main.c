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
#include "esp32/rom/ets_sys.h"
#include "esp32/rom/crc.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"
#include "espnow.h"
#include "CRC.h"
#include "led.h"
#include "espdefine.h"
#include "driver/gpio.h"
#include "uart_func.h"
#include "hdr/storage.h"




static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 1024;
#define LIMIT_NODE_SLAVE 100
#define TXD_PIN 25//(GPIO_NUM_33)
#define RXD_PIN 14//14//(GPIO_NUM_26)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define RTS_PIN   27//(25)


#define CTS_PIN   (19)

#define NodeID 0
#define DEFAULT_ID 255
#define BaudaRate 1
#define DEFAULT_BR 10 //115200

#define OFFSET	256

#define GPIO_INPUT_IO_0     0
#define GPIO_OUTPUT_IO_19	2//xxx 19
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
static uint16_t s_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };


static void espnow_deinit(espnow_send_param_t *send_param);
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
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
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
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
			size_data = HOLDING_REGISTER_SIZE;
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
							printf("Holding reading success\n");
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
	ESP_LOGE(TAG,"Guardar en la memoria");
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
    nvs_close(nvshandle);
}
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_t.info.send_cb;
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    evt.info.send_t.id = ESPNOW_FROM_CB;
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
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint8_t *dir)
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
    buf->Nodeid = HoldingRegister[NodeID];
    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->dir = send_param->dir;
    buf->crc = crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

void RegisterPeer(uint8_t mac[ESP_NOW_ETH_ALEN]){
	esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
	if (esp_now_is_peer_exist(mac) == false) {
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
	ESP_LOGI(TAG, "MAC succefully "MACSTR" added", MAC2STR(mac));
	free(peer);
	}
}

void vEspnowGetOldPeers(void){
	ESP_LOGI(TAG, "Registering Old Peers");
	uint8_t mac[ESP_NOW_ETH_ALEN] = {0};
	for (uint16_t j = 0 ; j < ROUTING_TABLE_SIZE ; j++){
		memcpy(mac,PeerTable + (RoutingTable[j]* ESP_NOW_ETH_ALEN),ESP_NOW_ETH_ALEN);
		if((RoutingTable[j]!=0)&&(memcmp(PeerTable + RoutingTable[j]* ESP_NOW_ETH_ALEN, broadcast_mac , ESP_NOW_ETH_ALEN)) !=0){
			RegisterPeer(mac);
			ESP_LOGI(TAG, "Node %d of position %d added",RoutingTable[j],j );
		}
		if (j==150 || j==151 )
		ESP_LOGE(TAG,"%d",RoutingTable[j]);
	}
}

void espnow_send(void *pvParameter){
	espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
	esp_uart_data_t U_data;
	espnow_data_t *buf = (espnow_data_t *)send_param->buffer;
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
	uint8_t des_node = 0;
	uint16_t posicion = 0;
    while(xQueueReceive(espnow_Squeue, &U_data, portMAX_DELAY) == pdTRUE){
    	ESP_LOGI(TAG,"Send Function Activated");
    	des_node = RoutingTable[U_data.data[0]];
    	send_param->dir  = FORDWARD;
    	if ((des_node == HoldingRegister[NodeID])||(U_data.dir == BACKWARD)){
        	des_node = RoutingTable[OFFSET+U_data.data[0]];
    		memcpy(send_param->dest_mac,PeerTable+(ESP_NOW_ETH_ALEN * des_node),ESP_NOW_ETH_ALEN);
    		ESP_LOGI(TAG,"ANSWERING TO MASTER  from node :%d, and slave: %d\n", RoutingTable[OFFSET+U_data.data[0]],U_data.data[0]);
    		send_param->dir = BACKWARD;
    	}
    	else {
    		if (des_node == 0) continue; //No route
    	   	posicion = ESP_NOW_ETH_ALEN * des_node;
    		memcpy(send_param->dest_mac, PeerTable + posicion,ESP_NOW_ETH_ALEN);
    		ESP_LOGI(TAG, "ITS FOR NODE %d with MAC: "MACSTR"", des_node, MAC2STR(send_param->dest_mac));
    	}
    	bzero(buf->payload,ESPNOW_PAYLOAD_SIZE);
    	memcpy(buf->payload,U_data.data,U_data.len);
    	buf->data_len = U_data.len;
    	buf->state = false;
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
	uart_baudarate = UARTBaudaRate(BTid);
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
	const uint16_t offset = 256;

	if (CRC16(data.data,data.len) == 0){
		vNotiUart();
		switch(function){
		case WRITE_HOLDING_REGISTER:
			if (Address.Val == 0 && Value.Val <= LIMIT_NODE_SLAVE){
				ESP_LOGI(TAG,"Node Id not allowed, must be greater than 100\n");
				break;
			}
			if (Address.Val > offset){
				HoldingRAM[Value.Val+offset] = Value.Val;
				ESP_LOGI(TAG, "Modifying Holding Register, Value %d  position %d \n",HoldingRAM[Value.Val+offset],Value.Val);
			}
			HoldingRAM[Address.Val] = Value.Val;
			ESP_LOGI(TAG, "Modifying Holding Register, Value %d  position %d \n",HoldingRAM[Address.Val],Address.Val);
			if (dir == ESP_NOW){
				data.dir = BACKWARD;
				xQueueSend(espnow_Squeue, &data, portMAX_DELAY);
			}
			else
			uart_write_bytes(UART_NUM_1,(const char*)data.data,data.len);
			break;

		case WRITE_COIL:
			switch(Address.Val){
			case RESTART:
				if ((Address.Val == 0)&&(Value.Val ==0xFF00)){
					printf("Restarting \n");
					if (dir == ESP_NOW){
						data.dir = BACKWARD;
						xQueueSend(espnow_Squeue, &data, portMAX_DELAY);
					}
					else
					uart_write_bytes(UART_NUM_1,(const char*)data.data,data.len);
					vTaskDelay(10);
					fflush(stdout);
					esp_restart();
				}
				break;
			case SAVE_RAM:
				 if (HoldingRegister[BaudaRate] != HoldingRAM[BaudaRate]){
						vTaskDelay(500);
						UARTinit(HoldingRAM[BaudaRate]);
				 }
				 memcpy(HoldingRegister,HoldingRAM,HOLDING_REGISTER_SIZE);
				 memcpy(RoutingTable,HoldingRAM + offset,ROUTING_TABLE_SIZE-offset);
				 if (dir == ESP_NOW){
					 ESP_LOGI(TAG,"Saved in RAM");
					 data.dir = BACKWARD;
					 xQueueSend(espnow_Squeue, &data, portMAX_DELAY);
				 }
				 else
					 uart_write_bytes(UART_NUM_1,(const char*)data.data,data.len);
				 break;
			case SAVE_FLASH:
				 memcpy(HoldingRegister,HoldingRAM,HOLDING_REGISTER_SIZE);
				 memcpy(RoutingTable,HoldingRAM + offset,ROUTING_TABLE_SIZE-offset);
				 vConfigSetNVS(RoutingTable,"RoutingTable");
				 vConfigSetNVS(HoldingRegister,"HoldingRegister");
				 ESP_LOGI(TAG,"Saved in FLASH");
				 if (dir == ESP_NOW){
					data.dir = BACKWARD;
					xQueueSend(espnow_Squeue, &data, portMAX_DELAY);
				 }

				 else
					 uart_write_bytes(UART_NUM_1,(const char*)data.data,data.len);
				 break;
			}
			break;

		case READ_HOLDING:
			ESP_LOGI(TAG,"Reading HoldingRegister\n");
			data.data[2] = data.data[5]*2;
			uint8_t inc =3;
			uint8_t data_limit = data.data[5];
			for(uint16_t i = 0; i < data_limit;i++){
				data.data[inc] = 0;
				inc++;
				data.data[inc] =HoldingRAM[Address.Val+i];
				inc++;
			}
			CRC.Val = CRC16(data.data,inc);
			data.data[inc++] = CRC.byte.LB;
			data.data[inc] = CRC.byte.HB;
			data.len= ++inc;
			if (dir == ESP_NOW){
				data.dir = BACKWARD;
				xQueueSend(espnow_Squeue, &data, portMAX_DELAY);

			}
			else
				uart_write_bytes(UART_NUM_1,(const char*)data.data, data.len);
			break;
		default :
			data.data[1]+= 0x80;
			data.data[2] = 0x01;
			CRC.Val = CRC16(data.data,3);
			data.data[3] = CRC.byte.LB;
			data.data[4] = CRC.byte.HB;
			if (dir == ESP_NOW){
				data.dir = BACKWARD;
				xQueueSend(espnow_Squeue, &data, portMAX_DELAY);
			}
			else
				uart_write_bytes(UART_NUM_1,(const char*)data.data,5);
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
			xQueueSend(espnow_Squeue, &data, portMAX_DELAY);
		}
		else
			uart_write_bytes(UART_NUM_1,(const char*)data.data,5);
	}
}
static void espnow_manage_task(void *pvParameter)
{
    espnow_event_t evt;
    uint8_t recv_state = 0;
    uint8_t dir = 0;
    uint16_t recv_seq = 0;
    int ret;
    int Peer_Quantity = 0;
    int count=0;
    uint8_t tries = 0;
    esp_uart_data_t U_data;
    vTaskDelay(5000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Espnow_task_manage activated");
    uint8_t mac[ESP_NOW_ETH_ALEN] = {0};
    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
	xTaskCreate(espnow_send, "espnow_send", 2048*4, send_param, 3, NULL);
    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ESPNOW_SEND_CB:
            {
                espnow_send_t *sendinf = &evt.info.send_t;
                switch(sendinf->id){
                	case ESPNOW_FROM_CB:
                	{
                        espnow_event_send_cb_t *send_cb = &evt.info.send_t.info.send_cb;
						ESP_LOGI(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
						if (memcmp(send_cb->mac_addr, broadcast_mac , ESP_NOW_ETH_ALEN) ==0){
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
							break;
						}

						else if ((send_cb->status == 0)||(tries==5))
								break;
							 else{
								tries++;
			                	xQueueSend(espnow_Squeue, &U_data, portMAX_DELAY);
							 }
						break;
                	}
                	case ESPNOW_TO_SEND:
                	{
                		tries = 0;
                		ESP_LOGI(TAG,"Enviando a la cola de enviar");
                		espnow_send_data_t *espnow_data = &evt.info.send_t.info;
                		U_data = espnow_data->send_data;
                		xQueueSend(espnow_Squeue, &U_data, portMAX_DELAY);
                		break;
                	}
                }

                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                /*Verify is broadcast*/
                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq , &dir);
                espnow_data_t *buf = (espnow_data_t *)recv_cb->data;
				U_data.data = buf->payload;
				U_data.len = buf->data_len;
        		uint8_t slave = U_data.data[0];

                if (ret == ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                    uint8_t recv_NodeID = buf->Nodeid;
                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (HoldingRegister[NodeID]== slave || RoutingTable[slave]!=0){
                    	ESP_LOGE(TAG, "Broadcast Slave %d ", slave);
						if ((esp_now_is_peer_exist(recv_cb->mac_addr) == false)) {//xxx about to test
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
							ESP_LOGI(TAG, "Peer %dth added,  MAC: "MACSTR"",Peer_Quantity, MAC2STR(recv_cb->mac_addr));
							free(peer);
							/* Send the response to the new peer, for register myself with it*/
							if(buf->state == false){
								send_param->buffer =recv_cb->data;
								send_param->state = true;
								espnow_data_prepare(send_param);
								send_param->unicast = false;
								send_param->broadcast = true;
								if (esp_now_send(broadcast_mac, send_param->buffer, send_param->len) != ESP_OK) {
									ESP_LOGE(TAG, "Send error");
									espnow_deinit(send_param);
									vTaskDelete(NULL);
								}
								send_param->unicast = true;
								send_param->broadcast = false;
							}
							memcpy(PeerTable+(recv_NodeID)*(ESP_NOW_ETH_ALEN), recv_cb->mac_addr,  ESP_NOW_ETH_ALEN);
							vConfigSetNVS(PeerTable, "PeerTable");
							memcpy(mac,PeerTable + (recv_NodeID)*(ESP_NOW_ETH_ALEN),ESP_NOW_ETH_ALEN);
							ESP_LOGI(TAG, "MAC added is:  "MACSTR" from Node %d is the position %d", MAC2STR(mac),recv_NodeID, (recv_NodeID)*(ESP_NOW_ETH_ALEN));
						}
                    }
                }
                else if (ret == ESPNOW_DATA_UNICAST) {
            		ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
            		vNotiUart();
                	if (dir == FORDWARD){
                    RoutingTable[slave+OFFSET] = buf->Nodeid;
                    vConfigSetNVS(RoutingTable, "RoutingTable");
            		ESP_LOGI(TAG, "BackMAC  data from: "MACSTR",  of slave %d ", MAC2STR(recv_cb->mac_addr),slave);
                    }
					uint8_t mode = uComGetTransData(slave);
					switch(dir){
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
								U_data.dir = dir;
								ESP_LOGI(TAG,"JUMP DIR %d: ", dir );
								xQueueSend(espnow_Squeue, &U_data, portMAX_DELAY);//xxx
							}
							break;
							case BACKWARD:
								memcpy(back_mac, PeerTable+(RoutingTable[slave+OFFSET]*ESP_NOW_ETH_ALEN), ESP_NOW_ETH_ALEN);
								if(memcmp(back_mac, broadcast_mac, ESP_NOW_ETH_ALEN)==0){
									uart_write_bytes(UART_NUM_1,(const char*) U_data.data ,U_data.len);
									vNotiUart();
									ESP_LOGI(TAG,"SERIAL response");
								}
								else {
									U_data.dir = dir;
									ESP_LOGI(TAG,"JUMP BACK %d: ", dir );
									xQueueSend(espnow_Squeue, &U_data, portMAX_DELAY);//xxx
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
    xTaskCreate(espnow_manage_task, "espnow_manage_task", 2048*4, send_param, 4, NULL);
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
    UARTinit(HoldingRegister[BaudaRate]);
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    esp_uart_data_t U_data;
    espnow_event_t evt;
    for(;;) {
        if(xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RX_BUF_SIZE);
            ESP_LOGI(RX_TASK_TAG, "uart[%d] event:", UART_NUM_1);
            switch(event.type) {
                case UART_DATA:
                    uart_read_bytes(UART_NUM_1, dtmp, event.size, portMAX_DELAY);
                    uint8_t info = uComDirection(&dtmp[0]);
                    U_data.data = dtmp;
                    U_data.len = (uint8_t) event.size;
                    ESP_LOGI(RX_TASK_TAG,"Data recivida id %x \n",dtmp[0]);
                    switch(info){
                    case EX_SLAVE:
                    	evt.id = ESPNOW_SEND_CB;
                    	evt.info.send_t.id = ESPNOW_TO_SEND;
                    	evt.info.send_t.info.send_data = U_data;
                    	xQueueSend(espnow_queue,&evt,(portTickType)portMAX_DELAY);
						break;
                    case NODE:
                    	ESP_LOGI(RX_TASK_TAG,"Node");
                    	vConfigSetNode(U_data,UART);
                    	break;
                    }
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    vNotiUart();
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGI(RX_TASK_TAG, "hw fifo overflow");
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGI(RX_TASK_TAG, "ring buffer full");
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                case UART_BREAK:
                    ESP_LOGI(RX_TASK_TAG, "uart rx break");
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGI(RX_TASK_TAG, "uart parity error");
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGI(RX_TASK_TAG, "uart frame error");
                    break;
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
	HoldingRegister[NodeID]= DEFAULT_ID;
	HoldingRegister[BaudaRate]= DEFAULT_BR;
	vConfigGetNVS(HoldingRegister,"HoldingRegister");
	vConfigGetNVS(RoutingTable,"RoutingTable");
	vConfigGetNVS(PeerTable,"PeerTable");
	memcpy(HoldingRAM,HoldingRegister,HOLDING_REGISTER_SIZE);
	ESP_LOGE(TAG,"Configuracion Cargada: BaudaRate ID (%d) y MODBUS ID (%d)",HoldingRegister[BaudaRate],HoldingRegister[NodeID]);
}


void vConfigFormatFactory( void ){

	memset(PeerTable,0xff,PEER_TABLE_SIZE*ESP_NOW_ETH_ALEN);
	bzero(RoutingTable,ROUTING_TABLE_SIZE);
	bzero(HoldingRegister,HOLDING_REGISTER_SIZE);
	HoldingRegister[NodeID] = 152;//xxx
	HoldingRegister[BaudaRate] = DEFAULT_BR;
	vConfigSetNVS(HoldingRegister,"HoldingRegister");
	vConfigSetNVS(RoutingTable,"RoutingTable");
	vConfigSetNVS(PeerTable,"PeerTable");
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken);
}

void FormatFactory(void *arg){
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
	uint8_t ticksTorestart = 5;
    xSemaphore = xSemaphoreCreateBinary();
	ESP_LOGI(TAG, "FORMAT FACTORY TASK CREATED");
	while(xSemaphoreTake( xSemaphore, portMAX_DELAY) == pdTRUE){
		while(gpio_get_level(GPIO_INPUT_IO_0)==0){
			ESP_LOGI(TAG, "Restarting in %d segundos", ticksTorestart);
			vTaskDelay(1000/portTICK_RATE_MS);
			ticksTorestart--;
			if (ticksTorestart == 0){
				vConfigFormatFactory();
				vTaskDelay(1000/portTICK_RATE_MS);
			    ESP_LOGE(TAG,"Restarting \n");
			    fflush(stdout);
			    esp_restart();
			}
		}
		ticksTorestart = 5;
        vTaskDelay(1000/portTICK_RATE_MS);
	}
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    //nvs_flash_erase();
    ESP_ERROR_CHECK( ret );
    esp_log_level_set(TAG, ESP_LOG_INFO);
    vConfigLoad();
   // Create UART tasks
    xTaskCreatePinnedToCore(rx_task, "uart_rx_task", 2048*2, NULL, configMAX_PRIORITIES, NULL,1);
    //Create ESPnow Tasks
    wifi_init();
    espnow_init();
    xTaskCreatePinnedToCore(FormatFactory, "FormatFactory", 2048*3, NULL, configMAX_PRIORITIES, NULL,0);
    vNotiLEDinit();
}
