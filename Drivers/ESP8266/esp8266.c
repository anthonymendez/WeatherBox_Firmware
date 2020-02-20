/*
 * esp8266.c
 *
 *  Created on: Jan 16, 2020
 *      Author: Stephan Kim, Anthony Mendez?
 */
#include "esp8266.h"

unsigned char UART_DMA_Receive_Buffer[UART_DMA_RECEIVE_BUFFER_LENGTH];
UART_HandleTypeDef *huart;

// TODO: Add INFO
HAL_UART_StateTypeDef Transmit(char *pData)
{
	return HAL_UART_Transmit(huart, (uint8_t *) pData, strlen(pData), UART_TIMEOUT);
}

// TODO: Add INFO
HAL_UART_StateTypeDef Receive_Start()
{
	Clear_Receive_Buffer();
	return HAL_UART_Receive_DMA(huart, UART_DMA_Receive_Buffer, UART_DMA_RECEIVE_BUFFER_LENGTH);
}

// TODO: Add INFO
HAL_UART_StateTypeDef Receive_Resume()
{
	return HAL_UART_DMAResume(huart);
}

// TODO: Add INFO
HAL_UART_StateTypeDef Receive_Pause()
{
	return HAL_UART_DMAPause(huart);
}

// TODO: Add INFO
HAL_UART_StateTypeDef Receive_Stop()
{
	return HAL_UART_DMAStop(huart);
}

// TODO: Add INFO
void Clear_Receive_Buffer()
{
	int i = 0;
	for (; i < UART_DMA_RECEIVE_BUFFER_LENGTH; i++) {
		UART_DMA_Receive_Buffer[i] = 0;
	}
}
/**
 * @brief resets the wifi module
 */
void wifiRST(UART_HandleTypeDef* huart_in)
{
	huart = huart_in;
	char rst[] = "AT+RST\r\n";
	HAL_UART_StateTypeDef status = Receive_Start();
	status |= Transmit(rst);
	HAL_Delay(25);
	status |= Receive_Stop();
}

/**
 * @brief initializes the module by setting mode as client, sets baud rate and UART format
 */
void wifiInit()
{
	char set[] = "AT+CWMODE=1\r\n";
	//TODO: Figure out why we can read after the first time
	HAL_UART_StateTypeDef status = Receive_Start();
	status |= Transmit(set);
	HAL_Delay(25);
	status |= Receive_Stop();
	Clear_Receive_Buffer();

	char uart_set[] = "AT+UART_CUR=115200,8,1,0,0\r\n";
	status |= Transmit(uart_set);
	HAL_Delay(5);

	char uart_set_def[] = "AT+UART_DEF=115200,8,1,0,0\r\n";
	status |= Transmit(uart_set_def);
	HAL_Delay(5);

	char recv_mode[] = "AT+CIPRECVMODE=1\r\n";
	status |= Transmit(recv_mode);
	HAL_Delay(5);

	char transparent_mode[] = "AT+CIPMODE=0\r\n";
	status |= Transmit(recv_mode);
	HAL_Delay(5);

	Clear_Receive_Buffer();
}

/**
 * @brief connects to a password protected wifi access point
 */
void connectWifi(char* ssid, char* pass)
{
	char connect[50];
	sprintf(connect, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pass);
	HAL_UART_Transmit(huart, (uint8_t *) connect, strlen(connect), 500);
	HAL_Delay(500);
}

/**
 * @brief transmits data to our website, will fix to make more dynamic
 */
void transmitWifi(char* info, UART_HandleTypeDef huart1 )
{

	char start[] = "AT+CIPSTART=\"TCP\",\"weatherbox.azurewebsites.net\",80\r\n";
	HAL_UART_Transmit(huart, (uint8_t *) start, strlen(start), 500);
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
	HAL_UART_Transmit(huart, (uint8_t *) send, strlen(send), 500);
	HAL_UART_Transmit(huart, (uint8_t *) postStr, strlen(postStr), 500);
	HAL_UART_Transmit(huart, (uint8_t *) ret, strlen(ret), 500);
	HAL_Delay(1000);

	//Sending POST message
	HAL_UART_Transmit(huart, (uint8_t *) post, strlen(post), 500);
	HAL_UART_Transmit(huart, (uint8_t *) ret, strlen(ret), 500);
	HAL_Delay(2000);
	HAL_UART_Transmit(huart, (uint8_t *) send, strlen(send), 500);
	HAL_UART_Transmit(huart, (uint8_t *) jsonStr, strlen(jsonStr), 500);
	HAL_UART_Transmit(huart, (uint8_t *) ret, strlen(ret), 500);
	HAL_UART_Transmit(huart, (uint8_t *) info, strlen(info), 500);
	HAL_UART_Transmit(huart, (uint8_t *) ret, strlen(ret), 500);
}

//TODO: Add INFO
void wifi_get_timestamp(UART_HandleTypeDef huart1)
{
	char start[] = "AT+CIPSTART=\"TCP\",\"weatherbox.azurewebsites.net\",80\r\n";
	HAL_UART_Transmit(huart, (uint8_t *) start, strlen(start), 500);
	HAL_Delay(2000);
	char send[] = "AT+CIPSEND=";
	char recv[] = "AT+CIPRECVDATA=1000\r\n";
	char ret[] = "\r\n";
	char get[] = "GET /timestamp HTTP/1.1\r\nAccept: \"*/*\"\r\nHost: weatherbox.azurewebsites.net\r\n\r\n";
	int get_size = (int)(strlen(get));
	char get_str[sizeof(get_size)];
	sprintf(get_str, "%u", get_size);

	// Send Command with size of message
	HAL_UART_Transmit(huart, (uint8_t *) send, strlen(send), 500);
	HAL_UART_Transmit(huart, (uint8_t *) get_str, strlen(get_str), 500);
	HAL_UART_Transmit(huart, (uint8_t *) ret, strlen(ret), 500);
	HAL_Delay(1000);

	//Sending GET message
	HAL_UART_Transmit(huart, (uint8_t *) get, strlen(get), 500);
	HAL_UART_Transmit(huart, (uint8_t *) ret, strlen(ret), 500);
	HAL_UART_Transmit(huart, (uint8_t *) recv, strlen(recv), 500);
	HAL_UART_DMAStop(huart);
	Clear_Receive_Buffer();
	HAL_StatusTypeDef status = HAL_UART_Receive_DMA(huart, (uint8_t *) UART_DMA_Receive_Buffer, UART_DMA_RECEIVE_BUFFER_LENGTH);
	HAL_UART_DMAResume(huart);
	HAL_Delay(1);
	HAL_UART_DMAStop(huart);
	Clear_Receive_Buffer();
}
