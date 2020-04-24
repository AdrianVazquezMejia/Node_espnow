/*
 * nvs.h
 *
 *  Created on: Apr 24, 2020
 *      Author: adrian-estelio
 */

#include <stdint.h>


#define ESP_NOW_ETH_ALEN 6

extern void vConfigSetNVS(uint8_t *Array , const char *Name);
extern void vConfigGetNVS(uint8_t *Array , const char *Name);


