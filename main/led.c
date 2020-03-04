/*
 * led.c
 *
 *  Created on: Mar 4, 2020
 *      Author: estelio
 */

#include "led.h"
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
#define GPIO_OUTPUT_IO_19	19


void vNotiLEDinit(void){

    gpio_config_t io_conf;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = 1ULL<<GPIO_OUTPUT_IO_19;
    //set as input mode
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    for(uint8_t i = 0; i<3; i++){
    gpio_set_level(GPIO_OUTPUT_IO_19,1);
    vTaskDelay(50);
    gpio_set_level(GPIO_OUTPUT_IO_19,0);
    vTaskDelay(50);
    }
}

void vNotiUart(void){
	gpio_set_level(GPIO_OUTPUT_IO_19,1);
	vTaskDelay(5);
	gpio_set_level(GPIO_OUTPUT_IO_19,0);
}
