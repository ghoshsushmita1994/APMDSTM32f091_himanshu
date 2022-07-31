/**
 * ESPDataLogger.c
 *
 *  Created on: May 26, 2020
 *      Author: Harshita Jain
 */

/*ESP8266 Commands for IITD_IOT &TCP Server*/
/**
 * AT - Power & Connection
 *  * AT+RST - Reset ESP8266
 * AT+CWMODE=1 - Used to set WIFI Mode
 * AT+CHIPMUX=0 - Enable or disable multiple connection
 * AT+CHIPSTART="TCP","IP_ADDRESS",TCP_PORT_NUMBER -Connect to the Server
 * AT+CHIPSEND=number of bytes - For sending number of bytes data
 * AT+CHIPCLOSE - For closing the TCP Connection
 *
 * Other useful Commands
 * AT+CIFSR - for getting Static IP and MAC Address
 * AT+CIPSTA? - Returns the IP, Gateway, NetMask.
 *
 */

/*******User Code begins**********/

#include "UartRingbuffer.h"
#include "ESPDataLogger.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart5;

void bufclr (char *buf)
{
	int len = strlen (buf);
	for (int i=0; i<len; i++) buf[i] = '\0';
}

/**************************************************************
 * 			ESP_RST(void)
 * 			Resetting the ESP8266
 */

void ESP_RST(void)
{
		Ringbuf_init();

		Uart_sendstring("AT+RST\r\n");

		HAL_Delay(1000);

		Uart_flush();
}

/**************************************************************
 * 			ESP_Init
 * 			Initializing ESP8266 for WIFI
 */
void ESP_Init (void)		//char *SSID, char *PASSWD) // other than IIT_IOT network SSID PASSWD Requires
{

	Ringbuf_init();


	/********* AT **********/
	Uart_sendstring("AT\r\n");
	//while(!(Wait_for("OK\r\n")));


	Uart_flush();

	/******** AT+CWMODE=1 *********/

	Uart_sendstring("AT+CWMODE=1\r\n");
//	while (!(Wait_for("OK\r\n")));	// Function changed - Himanshu

	Uart_flush();
//	HAL_Delay(10000);

/**********For IIT_IOT WIFI need not to use this command*****/
/*
	******** AT+CWJAP="SSID","PASSWD" *********
	sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	goto hell;
	Uart_sendstring(data);


	Uart_flush();
*/

	/********* AT+CIPMUX=0 **********/
	Uart_sendstring("AT+CIPMUX=0\r\n");

	Uart_flush();
}


/****************************************************************************
 * 			ESP_Send_Data ( float value[])
 * 			Send multiple values(buffer of 8 bytes(Size can be anything))
 * 			to the server
 * 			through TCP Connections
 */
void ESP_Send_Data ( float value[])
{
	char local_buf[16] = {0};
	char cmd_buf[30] = {0};
	char buf2send[128] = {0};
	int i=0;

	/*************Used when data is sending to ThingSpeak*********/
//	while (!(Wait_for("OK\r\n")));

	/*sprintf (local_buf, "GET /update?api_key=%s&field%d=%u\r\n", APIkey, Field_num, value);
	int len = strlen (local_buf);

	sprintf (cmd_buf, "AT+CIPSEND=%d\r\n", len);
	Uart_sendstring(cmd_buf);
	 */

	int len = 0;
	for( i=0; i<11; i++)
	{
		len += sprintf(local_buf, "%.4f,", value[i]);
		strcat(buf2send, local_buf);

	}

	Uart_sendstring("AT+CIPSTART=\"TCP\",\"10.17.10.11\",12000\r\n");
	HAL_Delay(500);

	printf("\r\n\r\nConnected.\r\nData : %s", (char *)buf2send);

	sprintf (cmd_buf, "AT+CIPSEND=%d\r\n", len);
	printf("\r\n%s\r\n", (char *)cmd_buf);
	Uart_sendstring(cmd_buf);
	Uart_sendstring(buf2send);

//	Uart_sendstring("AT+CIPSEND=4\r\n");
//	Uart_sendstring("ok\r\n");
//	char pData[10] = {0};
//	HAL_UART_Transmit(&huart5, (uint8_t *)"AT\r\n", sizeof("AT\r\n"), 100);
//	HAL_UART_Receive(&huart5, (uint8_t *)pData, 10, 1000);
//	printf("Received : %s\r\n", pData);

	HAL_Delay(500);

	//Uart_sendstring("AT+CIPCLOSE\r\n");
	bufclr(local_buf);
	bufclr(cmd_buf);

	Ringbuf_init();

}


/*********************************************************************
 * 			ESP_Send_Multi (uint16_t value[])
 * 			Used for Sending Multiple values to ThingSpeak
 * 			IIT_IOT network is not using this Function
 */

void ESP_Send_Multi (uint16_t value[])
{
	char local_buf[500] = {0};
	char local_buf2[30] = {0};
	char field_buf[200] = {0};


	Uart_sendstring("AT+CIPSTART=\"TCP\",\"10.17.10.11\",12000\r\n");
//	while (!(Wait_for("OK\r\n")));


	/*sprintf (local_buf, "GET /update?api_key=%s", APIkey);
	for (int i=0; i<numberoffileds; i++)
	{
		sprintf(field_buf, "&field%d=%u",i+1, value[i]);
		strcat (local_buf, field_buf);
	}

	strcat(local_buf, "\r\n");
	int len = strlen (local_buf);

	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	Uart_sendstring(local_buf2);
	while (!(Wait_for(">")));
	 */
	Uart_sendstring (local_buf);


	bufclr(local_buf);
	bufclr(local_buf2);

	Ringbuf_init();

}


