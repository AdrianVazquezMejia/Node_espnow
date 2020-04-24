/*
 * modbus.h
 *
 *  Created on: Apr 24, 2020
 *      Author: adrian-estelio
 */
#include "stdint.h"
#ifndef MAIN_MODBUS_H_
#define MAIN_MODBUS_H_
#define LIMIT_NODE_SLAVE 100

const char *TAG_ = "MODBUS";
typedef struct {
	uint8_t len;                              /* length of the data*/
	uint8_t *data;								/* pointing to the data*/
	uint8_t dir;								/* direction od the data*/
} __attribute__((packed)) uart_data_t;
typedef union
{
    uint16_t Val;
    struct
    {
        uint8_t LB;
        uint8_t HB;
    } byte;

} INTVAL;
enum{
	WRITE_COIL =5,
	WRITE_HOLDING_REGISTER
};
enum {
	RESTART,
	SAVE_RAM,
	SAVE_FLASH
} ;

enum{
	READ_HOLDING =3,

};

enum{
	ESPNOW,
	UART_
};
enum{
	FORDWARD_ = 0,
	BACKWARD_
};

#endif /* MAIN_MODBUS_H_ */
