/*
 * esp8266.c
 *
 *  Created on: Jan 16, 2020
 *      Authors: Stephan Kim, Anthony Mendez
 */
#include "esp8266.h"

/*
 * Debugging Note:
 * 	When debugging DMA receiving, do not put multiple breakpoints to find the associated return value for a transmission.
 * 	You will have to rerun the program for each response for a corresponding transmission.
 */

unsigned char UART_DMA_Receive_Buffer[UART_DMA_RECEIVE_BUFFER_LENGTH];
UART_HandleTypeDef *huart;
char ret[] = "\r\n";

HAL_UART_StateTypeDef Transmit_Receive(char *pData)
{
	HAL_UART_StateTypeDef status = HAL_OK;
	status |= Receive_Start();
	status |= Transmit(pData);
	status |= Receive_Stop();
	return status;
}

// TODO: Add INFO
HAL_UART_StateTypeDef Transmit(char *pData)
{
	HAL_UART_StateTypeDef status = HAL_UART_Transmit(huart, (uint8_t *) pData, strlen(pData), UART_TIMEOUT);
	HAL_Delay(1000);
	return status;
}

// TODO: Add INFO
HAL_UART_StateTypeDef Receive_Start()
{
	Clear_Receive_Buffer();
	HAL_UART_StateTypeDef status = HAL_UART_Receive_DMA(huart, UART_DMA_Receive_Buffer, UART_DMA_RECEIVE_BUFFER_LENGTH);
	HAL_Delay(25);
	return status;
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
HAL_UART_StateTypeDef wifiRST(UART_HandleTypeDef* huart_in)
{
	huart = huart_in;
	char rst[] = "AT+RST\r\n";
	HAL_UART_StateTypeDef status = HAL_OK;
	status |= Transmit_Receive(rst);
	return status;
}

/**
 * @brief initializes the module by setting mode as client, sets baud rate and UART format
 */
HAL_UART_StateTypeDef wifiInit()
{
	HAL_UART_StateTypeDef status = HAL_OK;

	char set[] = "AT+CWMODE=1\r\n";
	status |= Transmit_Receive(set);

	char uart_set[] = "AT+UART_CUR=115200,8,1,0,0\r\n";
	status |= Transmit_Receive(uart_set);

	return status;
}

/**
 * @brief connects to a password protected wifi access point
 */
HAL_UART_StateTypeDef connectWifi(char* ssid, char* pass)
{
	HAL_UART_StateTypeDef status = HAL_OK;

	char connect[50];
	sprintf(connect, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pass);
	status |= Transmit_Receive(connect);

	return status;
}

/**
 * @brief transmits data to our website, will fix to make more dynamic
 */
HAL_UART_StateTypeDef transmitWifi(char* info)
{
	HAL_UART_StateTypeDef status = HAL_OK;
	HAL_Delay(2000);
	char postFormat[] = "POST /map/data HTTP/1.1\r\nAccept: \"*/*\"\r\nHost: weatherbox.azurewebsites.net\r\nContent-Type: application/json\r\nContent-Length: %i\r\n\r\n";
	char post[sizeof(postFormat)];
	int jsonsize = (int)(strlen(info));
	char jsonStr[sizeof(jsonsize)];
	sprintf(jsonStr, "%u", jsonsize);
	sprintf(post, postFormat, jsonsize);
	int postsize = (int)(strlen(post));
	char postStr[sizeof(postsize)];
	sprintf(postStr, "%u", postsize);

	//Sending POST message
	status |= cipSend(postStr, post);
	HAL_Delay(1000);
	
	//Sending info in JSON format
	status |= cipSend(jsonStr, info);

	return status;
}

//TODO: Add INFO
HAL_UART_StateTypeDef wifi_get_timestamp(UART_HandleTypeDef huart1)
{
	HAL_UART_StateTypeDef status = HAL_OK;
	
	//Start connection to website
	char start[] = "AT+CIPSTART=\"TCP\",\"weatherbox.azurewebsites.net\",80\r\n";
	status |= Transmit_Receive(start);
	HAL_Delay(2000);
	
	char get[] = "GET /timestamp HTTP/1.1\r\nAccept: \"*/*\"\r\nHost: weatherbox.azurewebsites.net\r\n\r\n";
	int get_size = (int)(strlen(get));
	char get_str[sizeof(get_size)];
	sprintf(get_str, "%u", get_size);

	// Send Command with GET message and size
	status |= cipSend(get_str, get);

	return status;
}

/*
 *@brief AT+CIPSEND implementation
 */
HAL_UART_StateTypeDef cipSend(char* size, char* info)
{
	HAL_UART_StateTypeDef status = HAL_OK;
	char send[] = "AT+CIPSEND=";
	status |= Transmit_Receive(send);
	status |= Transmit_Receive(size);
	status |= Transmit_Receive(ret);
	HAL_Delay(100);
	status |= Transmit_Receive(info);
	status |= Transmit_Receive(ret);
	
	return status;
}
/**
 * @brief searches a string to see if it contains another string given a specific format
 */
char* stringParser(char* string, char* key)
{
	char* found = strstr(string, key);
	if(!found)
		return 0;
	else
	{
		found+= strlen(key);
	}
	return found;
}
