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
#include "subsystems/datalink/telemetry.h"

#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/termios.h>
#include <sys/ioctl.h>

uint16_t cara1, cara2;

uint16_t i, packetSize;
uint16_t data_in[256];
uint16_t values[8];
uint16_t tmp;
uint16_t cpt=0;
uint16_t cpt2=0;


int sim_uart_p = 0;

PRINT_CONFIG_VAR(UART_ARCH_H)



void ArduInit(void){

	//UART1Init();

	sim_uart_p = open("/dev/ttyUSB0", O_RDWR|O_NONBLOCK);
	if (sim_uart_p == -1)
	{
		fprintf(stderr,"Error: FireswarmCommunicationSim failed to open %s", "/dev/ttyUSB0");
	}
	else
	{
		struct termios orig_termios, cur_termios;

		//if (tcgetattr(sim_uart_p, &orig_termios)) failwith("getting modem serial device attr");
		cur_termios = orig_termios;

		// input modes
		cur_termios.c_iflag &= ~(IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK|ISTRIP|INLCR|IGNCR
		              |ICRNL |IXON|IXANY|IXOFF|IMAXBEL);
		// pas IGNCR sinon il vire les 0x0D
		cur_termios.c_iflag |= BRKINT;

		// output_flags
		cur_termios.c_oflag &=~(OPOST|ONLCR|OCRNL|ONOCR|ONLRET);

		// control modes
		cur_termios.c_cflag &= ~(CSIZE|CSTOPB|CREAD|PARENB|PARODD|HUPCL|CLOCAL|CRTSCTS);
		cur_termios.c_cflag |= CREAD|CS8|CLOCAL;

		// local modes
		cur_termios.c_lflag &= ~(ISIG|ICANON|IEXTEN|ECHO|FLUSHO|PENDIN);
		cur_termios.c_lflag |= NOFLSH;

		//if (cfsetspeed(&cur_termios, B57600)) failwith("setting modem serial device speed");

		//if (tcsetattr(sim_uart_p, TCSADRAIN, &cur_termios)) failwith("setting modem serial device attr");

	}




	register_periodic_telemetry(DefaultPeriodic, "ARDUINO_MEASURMENTS", Get_ADCSValues);	//DefaultPeriodic
}

int fireswarm_payload_link_has_data(void)
{
  if (sim_uart_p == -1) return 0;

  int bytes;
  ioctl(sim_uart_p, FIONREAD, &bytes);
  return bytes;
}

uint8_t get_uart_char(void){

	uint8_t val = 0;
	uint8_t ret;

	ret = read(sim_uart_p, &val, 1);

	return val;
}

void Get_ADCSValues(void){

	uint16_t nbrMsg = 0;
	uint8_t ret;

	//cpt=10;

	//UART1Getch();

	while(fireswarm_payload_link_has_data()>10){

		cpt = 0;
		ret = read(sim_uart_p, &cpt, 1);
		if(cpt != 0xAA) continue;

		cpt = 0;
		ret = read(sim_uart_p, &cpt, 1);
		if(cpt != 0x55) continue;

		ret = read(sim_uart_p, &cpt, 1);
		ret = read(sim_uart_p, &cpt2, 1);

		values[7] = (cpt<<8)+cpt2;

		DOWNLINK_SEND_ARDUINO_MEASURMENTS(DefaultChannel, DefaultDevice, 
			&values[0], &values[1], &values[2], &values[3], &values[4], &values[5], 
			&values[6], &values[7]);
	}

	/*cpt++;
	DOWNLINK_SEND_ARDUINO_MEASURMENTS(DefaultChannel, DefaultDevice, 
		&cpt, &values[1], &values[2], &values[3], &values[4], &values[5], 
		&values[6], &values[7]);*/


	/*while(fireswarm_payload_link_has_data()){

		while( (fireswarm_payload_link_has_data()) && (get_uart_char() != 0xAA));  //All the packets starts with 0xAA
		if(fireswarm_payload_link_has_data()){
			if(get_uart_char() == 0x55){     //Followed by 0x55

				if(fireswarm_payload_link_has_data()){

					packetSize = get_uart_char();  //We then get the packet size (in bytes)

					for(i = 0; i<packetSize; i++){  //We retrieve the corresponding data

						//if(fireswarm_payload_link_has_data()) data_in[i] = get_uart_char();	
						data_in[i] = get_uart_char();	
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
		}

	}*/

	//}

/*		cpt = get_uart_char();
DOWNLINK_SEND_ARDUINO_MEASURMENTS(DefaultChannel, DefaultDevice, 
					&cpt, &values[1], &values[2], &values[3], &values[4], &values[5], 
					&values[6], &values[7]);*/


















	//DOWNLINK_SEND_ARDUINO_MEASURMENTS(DefaultChannel, DefaultDevice, &nbrMsg);


	//while(UART1ChAvailable()){

		/*while( (UART1ChAvailable()) && (UART1Getch() != 0xAA));  //All the packets starts with 0xAA
		if(UART1ChAvailable()){
			if(UART1Getch() == 0x55){     //Followed by 0x55

				if(UART1ChAvailable()){

					packetSize = UART1Getch();  //We then get the packet size (in bytes)

					for(i = 0; i<packetSize; i++){  //We retrieve the corresponding data

						if(UART1ChAvailable()) data_in[i] = UART1Getch();	
					}

			    	//As far as the values we get are 8 bits we need to merge then two by two
			    	// to actualy get the measured values
					for(i=0; i < (packetSize >> 1); i++){

						values[i] = (data_in[i<<1]<<8)+data_in[(i<<1)+1];
					}


			    	//We then just got to send those values in the telemetry
					DOWNLINK_SEND_ARDUINO_MEASURMENTS(DefaultChannel, DefaultDevice, 
						&cpt, &values[1], &values[2], &values[3], &values[4], &values[5], 
						&values[6], &values[7]);
					DOWNLINK_SEND_ARDUINO_MEASURMENTS(DefaultChannel, DefaultDevice, 
						&values[3]);
				}
			}
		}*/

		/*cpt = 0;
		while(UART1ChAvailable()){

			UART1Getch();
			cpt++;
		}*/
	//}

	/*tmp = UART1ChAvailable();


	values[0] = UART1Getch();*/

	/*cpt++;

	tmp = UART1Getch();


	DOWNLINK_SEND_ARDUINO_MEASURMENTS(DefaultChannel, DefaultDevice, 
				&values[0], &values[1], &cpt, &tmp, &cpt, &values[5], 
				&values[6], &values[7]);*/

}


