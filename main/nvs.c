/*
 * nvs.c
 *
 *  Created on: Apr 24, 2020
 *      Author: adrian-estelio
 */
#include "hdr/storage.h"
#include "nvs.h"
#include <string.h>
#include "nvs_flash.h"
#include "esp_err.h"
nvs_handle nvshandle;

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
    nvs_close(nvshandle);
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
