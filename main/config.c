/*
 * config.c
 *
 *  Created on: Apr 24, 2020
 *      Author: adrian-estelio
 */
#include <string.h>
#include "hdr/storage.h"
#include "config.h"
#include "nvs.h"
#include "esp_log.h"
static const char *TAG_CONFIG = "Config";
void vConfigFormatFactory(){

	memset(PeerTable,0xff,PEER_TABLE_SIZE*ESP_NOW_ETH_ALEN);
	bzero(RoutingTable,ROUTING_TABLE_SIZE);
	bzero(HoldingRegister,HOLDING_REGISTER_SIZE);
	HoldingRegister[NodeID] = DEFAULT_ID;
	HoldingRegister[BaudaRate] = DEFAULT_BR;
	vConfigSetNVS(HoldingRegister,"HoldingRegister");
	vConfigSetNVS(RoutingTable,"RoutingTable");
	vConfigSetNVS(PeerTable,"PeerTable");
}
void vConfigLoad(){
	HoldingRegister[NodeID]= DEFAULT_ID;
	HoldingRegister[BaudaRate]= DEFAULT_BR;
	vConfigGetNVS(HoldingRegister,"HoldingRegister");
	vConfigGetNVS(RoutingTable,"RoutingTable");
	vConfigGetNVS(PeerTable,"PeerTable");
	memcpy(HoldingRAM,HoldingRegister,HOLDING_REGISTER_SIZE);
	ESP_LOGE(TAG_CONFIG,"Configuracion Cargada: BaudaRate ID (%d) y MODBUS ID (%d)",HoldingRegister[BaudaRate],HoldingRegister[NodeID]);
}
