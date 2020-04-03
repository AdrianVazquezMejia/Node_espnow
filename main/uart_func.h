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

int UARTBaudaRate(int BTid);
