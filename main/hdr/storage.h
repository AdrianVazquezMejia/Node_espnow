/*
 * storage.h
 *
 *  Created on: Apr 22, 2020
 *      Author: adrian-estelio
 */
#include <stdint.h>
#define HOLDING_REGISTER_SIZE 512
#define ROUTING_TABLE_SIZE 512 // xx should be 256
#define PEER_TABLE_SIZE 256
#ifndef MAIN_HDR_STORAGE_H_
#define MAIN_HDR_STORAGE_H_

uint8_t HoldingRegister[HOLDING_REGISTER_SIZE];
uint8_t RoutingTable[ROUTING_TABLE_SIZE];
uint8_t HoldingRAM[HOLDING_REGISTER_SIZE];
uint8_t PeerTable[PEER_TABLE_SIZE*ESP_NOW_ETH_ALEN];
#endif /* MAIN_HDR_STORAGE_H_ */
