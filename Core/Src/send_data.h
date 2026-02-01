/*
 * send_data.h
 *
 *  Created on: Dec 10, 2025
 *      Author: LENOVO
 */

#ifndef SRC_SEND_DATA_H_
#define SRC_SEND_DATA_H_

#define MAX_RPM 140
#include "usbd_cdc_if.h"

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
extern volatile int speed;
uint8_t usb_rx_buffer[CDC_DATA_FS_MAX_PACKET_SIZE + 1];

void process_command(uint8_t* usb_rx_buffer)
{
	if(sscanf((char*)usb_rx_buffer , "%d", &speed) == 1)
	{
		if (speed > MAX_RPM) speed = MAX_RPM;
		if (speed < -MAX_RPM) speed = -MAX_RPM;

	}
}

void CDC_Receive_FS_callBack(uint8_t* buf, uint32_t len){
	if (len == 0) return;
	if (len > CDC_DATA_FS_MAX_PACKET_SIZE)
		len = CDC_DATA_FS_MAX_PACKET_SIZE;
	memcpy(usb_rx_buffer, buf, len);
	usb_rx_buffer[len] = 0;
	for (int i = len -1 ; i >= 0 ; i--){
		if (usb_rx_buffer[i] == '\r' || usb_rx_buffer[i] == '\n')
			usb_rx_buffer[i] = 0;
		else
			break;
	}
	process_command(usb_rx_buffer);
}


#endif /* SRC_SEND_DATA_H_ */
