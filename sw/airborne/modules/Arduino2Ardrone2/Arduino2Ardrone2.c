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

	while(UART1Getch() != 0xAA);  //All the packets starts with 0xAA
	if(UART1Getch() == 0x55){     //Followed by 0x55

		packetSize = UART1Getch();  //We then get the packet size (in bytes)

		for(i = 0; i<packetSize; i++){  //We retrieve the corresponding data

			data_in[i] = UART1Getch();	
		}

    //As far as the values we get are 8 bits we need to merge then two by two
    // to actualy get the measured values
		for(i=0; i < (packetSize >> 1); i++){

			values[i] = (data_in[i<<1]<<8)+data_in[(i<<1)+1];
		}

    //We then just got to send those values in the telemetry
		DOWNLINK_SEND_ARDUINO_MEASURMENTS(DefaultChannel, DefaultDevice, 
			&values[0], &values[1], &values[2], &values[3], &values[4], &values[5], 
			&values[6], &values[7]);

	}

}


