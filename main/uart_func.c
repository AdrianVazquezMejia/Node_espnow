/*
 * uart_func.c
 *
 *  Created on: Apr 3, 2020
 *      Author: adrian-estelio
 */

#include "uart_func.h"
#include "config.h"
#include "hdr/storage.h"
#include "esp_log.h"
char *UART_TAG ="UART";
int UARTBaudaRate(int BTid) {
	int uart_baudarate = 115200;
	switch(BTid) {
		case 0:
			uart_baudarate = 300;
		break;
		case 1:
			uart_baudarate = 600;
			break;
		case 2:
			uart_baudarate = 1200;
			break;
		case 3:
			uart_baudarate = 2400;
			break;
		case 4:
			uart_baudarate = 4800;
			break;
		case 5:
			uart_baudarate = 9600;
			break;
		case 6:
			uart_baudarate = 14400;
			break;
		case 7:
			uart_baudarate = 19200;
			break;
		case 8:
			uart_baudarate = 38400;
			break;
		case 9:
			uart_baudarate = 56000;
			break;
		case 10:
			uart_baudarate = 115200;
			break;
		case 11:
			uart_baudarate = 128000;
			break;
		case 12:
			uart_baudarate = 256000;
			break;
		default : uart_baudarate = 115200;
			break;
	}
	return uart_baudarate;
}

uint8_t uComDirection(uint8_t *Slave){
	if (HoldingRegister[NodeID] == *Slave){
		return NODE;
		}
		else{
			return EX_SLAVE;
		}
	return -1;
}

void UARTinit(int BTid) {
	uart_driver_delete(UART_NUM_1);
	int uart_baudarate = 115200;
	uart_baudarate = UARTBaudaRate(BTid);
	ESP_LOGI(UART_TAG, "El baudarate is %d",uart_baudarate );
    const uart_config_t uart_config = {
        .baud_rate = uart_baudarate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, RTS_PIN, CTS_PIN);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2,  TX_BUF_SIZE * 2,  20, &uart1_queue, 0);
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
    }
