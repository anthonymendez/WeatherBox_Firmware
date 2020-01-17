/*
 * esp8266.c
 *
 *  Created on: Jan 16, 2020
 *      Author: Stephan Kim, Anthony Mendez?
 */
#include "esp8266.h"

/**
 * @brief resets the wifi module
 */
void wifiRST(UART_HandleTypeDef huart1)
{
	char rst[] = "AT+RST\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *)rst, strlen(rst), 500);
}

/**
 * @brief initializes the module by setting mode as client, sets baud rate and UART format
 */
void wifiInit(UART_HandleTypeDef huart1)
{
	char set[] = "AT+CWMODE=1\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) set, strlen(set), 500);
	HAL_Delay(5);

	char uart_set[] = "AT+UART_CUR=115200,8,1,0,0\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) uart_set, strlen(uart_set), 500);
	HAL_Delay(5);

	char uart_set_def[] = "AT+UART_DEF=115200,8,1,0,0\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) uart_set_def, strlen(uart_set_def), 500);
	HAL_Delay(5);

	char recv_mode[] = "AT+CIPRECVMODE=1\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) recv_mode, strlen(recv_mode), 500);
	HAL_Delay(5);
}

/**
 * @brief connects to a password protected wifi access point
 */
void connectWifi(char* ssid, char* pass, UART_HandleTypeDef huart1)
{
	char connect[50];
	sprintf(connect, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pass);
	HAL_UART_Transmit(&huart1, (uint8_t *) connect, strlen(connect), 500);
	HAL_Delay(5);
}

/**
 * @brief transmits data to our website, will fix to make more dynamic
 */
void transmitWifi(char* info, UART_HandleTypeDef huart1 )
{

	char start[] = "AT+CIPSTART=\"TCP\",\"weatherbox.azurewebsites.net\",80\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) start, strlen(start), 500);
	HAL_Delay(2000);
	char send[] = "AT+CIPSEND=";
	char ret[] = "\r\n";
	char postFormat[] = "POST /map/data HTTP/1.1\r\nAccept: \"*/*\"\r\nHost: weatherbox.azurewebsites.net\r\nContent-Type: application/json\r\nContent-Length: %i\r\n\r\n";
	char post[sizeof(postFormat)];
	int jsonsize = (int)(strlen(info));
	char jsonStr[sizeof(jsonsize)];
	sprintf(jsonStr, "%u", jsonsize);
	sprintf(post, postFormat, jsonsize);
	int postsize = (int)(strlen(post));
	char postStr[sizeof(postsize)];
	sprintf(postStr, "%u", postsize);


	// Send Command with size of message
	HAL_UART_Transmit(&huart1, (uint8_t *) send, strlen(send), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *) postStr, strlen(postStr), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *) ret, strlen(ret), 500);
	HAL_Delay(1000);

	//Sending POST message
	HAL_UART_Transmit(&huart1, (uint8_t *) post, strlen(post), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *) ret, strlen(ret), 500);
	HAL_Delay(2000);
	HAL_UART_Transmit(&huart1, (uint8_t *) send, strlen(send), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *) jsonStr, strlen(jsonStr), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *) ret, strlen(ret), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *) info, strlen(info), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *) ret, strlen(ret), 500);
}

