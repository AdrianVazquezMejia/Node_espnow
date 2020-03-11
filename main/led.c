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
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp32/rom/ets_sys.h"
#include "esp32/rom/crc.h"
#include "driver/gpio.h"

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
