/*
 * uart_func.c
 *
 *  Created on: Apr 3, 2020
 *      Author: adrian-estelio
 */

#include "uart_func.h"
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
