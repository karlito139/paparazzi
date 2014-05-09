/*
 * Copyright (C) PPRZ_team
 *
 * This file is part of paparazzi

 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include BOARD_CONFIG
#include "mcu_periph/uart.h"
#include <stdio.h>
#include "modules/Arduino2Ardrone2/Arduino2Ardrone2.h"
#include "subsystems/datalink/downlink.h"

uint16_t cara1, cara2;

uint16_t i, packetSize;
uint8_t data_in[256];
uint16_t values[8];
uint16_t trame[10] = {0};
uint16_t tmp;


PRINT_CONFIG_VAR(UART_ARCH_H)



void ArduInit(void){

	UART1Init();
}

void Get_ADCSValues(void){


/*
	while(UART1Getch() != 0xAA);
	trame[0] = 0xAA;
	trame[1] = UART1Getch();
	trame[2] = UART1Getch();
	trame[3] = UART1Getch();
	trame[4] = UART1Getch();
	trame[5] = 0;

	trame[3] = (trame[3]<<8)+trame[4];

	DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, DefaultDevice, &trame[0], &trame[1], &trame[2], &trame[3], &trame[4], &trame[5]);
*/


	while(UART1Getch() != 0xAA);
	if(UART1Getch() == 0x55){
		packetSize = UART1Getch();
		for(i = 0; i<packetSize; i++){

			data_in[i] = UART1Getch();	
		}

		//todo change that division per 2 by a shift
		for(i=0; i < (packetSize/2); i++){

			values[i] = (data_in[i*2]<<8)+data_in[i*2+1];
		}

		//We now have receaved the full packet we need to interpret it.
		//tmp = (data_in[0]<<8)+data_in[1];


		DOWNLINK_SEND_ARDUINO_MEASURMENTS(DefaultChannel, DefaultDevice, 
			&values[0], &values[1], &values[2], &values[3], &values[4], &values[5], 
			&values[6], &values[7]);

	}



}


