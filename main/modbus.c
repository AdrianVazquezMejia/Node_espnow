/*
 * modbus.c
 *
 *  Created on: Apr 24, 2020
 *      Author: adrian-estelio
 */
#include "modbus.h"
#include "esp_log.h"
#include "hdr/storage.h"
#include "freertos/FreeRTOS.h"
#include "config.h"
#include "driver/uart.h"
//#include "esp_now.h"
#include "CRC.h"
#include <string.h>
#include "led.h"
#include "nvs.h"


void vExecModbus(uart_data_t data, uint8_t dir){
	uint8_t function = data.data[1];
	INTVAL Address;
	Address.byte.HB = data.data[2];
	Address.byte.LB = data.data[3];
	INTVAL Value;
	INTVAL CRC;
	Value.byte.HB = data.data[4];
	Value.byte.LB = data.data[5];
	uint16_t offset = 256;

	if (CRC16(data.data,data.len) == 0){
		vNotiUart();
		switch(function){
		case WRITE_HOLDING_REGISTER:
			if (Address.Val == 0 && Value.Val <= LIMIT_NODE_SLAVE){
				ESP_LOGI(TAG_,"Node Id not allowed, must be greater than 100\n");
				break;
			}
			if (Address.Val > offset)
				HoldingRAM[Value.Val+offset] = Value.Val;
			HoldingRAM[Address.Val] = Value.Val;
			ESP_LOGI(TAG_, "Modifying Holding Register, Value %d  position %d \n",HoldingRAM[Address.Val],Address.Val);
			if (dir == ESPNOW){
				data.dir = BACKWARD_;
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
					if (dir == ESPNOW){
						data.dir = BACKWARD_;
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
						//UARTinit(HoldingRAM[BaudaRate]);
				 }
				 memcpy(HoldingRegister,HoldingRAM,HOLDING_REGISTER_SIZE);
				 memcpy(RoutingTable,HoldingRAM + offset,ROUTING_TABLE_SIZE);
				 ESP_LOGI(TAG_,"Saved in RAM");
				 if (dir == ESPNOW){
					 data.dir = BACKWARD_;
					 xQueueSend(espnow_Squeue, &data, portMAX_DELAY);
				 }
				 else
					 uart_write_bytes(UART_NUM_1,(const char*)data.data,data.len);
				 break;
			case SAVE_FLASH:
				 memcpy(HoldingRegister,HoldingRAM,HOLDING_REGISTER_SIZE);
				 memcpy(RoutingTable,HoldingRAM + offset,ROUTING_TABLE_SIZE);
				 vConfigSetNVS(RoutingTable,"RoutingTable");
				 vConfigSetNVS(HoldingRegister,"HoldingRegister");
				 ESP_LOGI(TAG_,"Saved in FLASH");
				 if (dir == ESPNOW){
					data.dir = BACKWARD_;
					xQueueSend(espnow_Squeue, &data, portMAX_DELAY);
				 }
				 else
					 uart_write_bytes(UART_NUM_1,(const char*)data.data,data.len);
				 break;
			}
			break;

		case READ_HOLDING:
			ESP_LOGI(TAG_,"Reading HoldingRegister\n");
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
			if (dir == ESPNOW){
				data.dir = BACKWARD_;
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
			if (dir == ESPNOW){
				data.dir = BACKWARD_;
				xQueueSend(espnow_Squeue, &data, portMAX_DELAY);
			}
			else
				uart_write_bytes(UART_NUM_1,(const char*)data.data,5);
		}
}
	else {
		ESP_LOGE(TAG_," CRC ERROR is %4x", CRC16(data.data,data.len));
		data.data[1]+=0x80;
		data.data[2] = 0x08;
		CRC.Val = CRC16(data.data,3);
		data.data[3] = CRC.byte.LB;
		data.data[4] = CRC.byte.HB;
		if (dir == ESPNOW){
			data.dir = BACKWARD_;
			xQueueSend(espnow_Squeue, &data, portMAX_DELAY);
		}
		else
			uart_write_bytes(UART_NUM_1,(const char*)data.data,5);
	}
}

