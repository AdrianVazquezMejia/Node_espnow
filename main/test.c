/*
 * test.c
 *
 *  Created on: May 25, 2020
 *      Author: adrian-estelio
 */
#include <stdint.h>
#include "hdr/test.h"

bool testParity(uint8_t parity){
	switch(parity){
		case 2:
			return true;
		case 3:
			return true;
		case 0:
			return true;
		}
	return false;
}
