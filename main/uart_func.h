/*
 * uart_func.h
 *
 *  Created on: Apr 3, 2020
 *      Author: adrian-estelio
 */
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp32/rom/ets_sys.h"
#include "esp32/rom/crc.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
//#include "queue.h"

enum{
	EX_SLAVE,
	NODE,

};

#define TXD_PIN 25//(GPIO_NUM_33)
#define RXD_PIN 14//14//(GPIO_NUM_26)
#define RTS_PIN   27//(25)
#define CTS_PIN   (19)

#define  RX_BUF_SIZE   1024
#define TX_BUF_SIZE  1024
QueueHandle_t uart1_queue;

int UARTBaudaRate(int BTid);

uint8_t uComDirection(uint8_t *Slave);

void UARTinit(int BTid);

