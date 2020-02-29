/**
 *  Driver for ESP8266-ESP01S Breakout Board
 *
 *  Utilizes HAL API and UART for communication
 */

/**
 * @defgroup ESP8266-ESP01S API
 */
#ifndef ESP8266_H_
#define ESP8266_H_


/* Header includes */
#include "stm32l0xx_hal.h"
#include "string.h"
#include <stdio.h>

/* Defines */
#define UART_DMA_RECEIVE_BUFFER_LENGTH	1000
#define UART_TIMEOUT 500

// TODO: Add INFO
HAL_UART_StateTypeDef Transmit(char *pData);

// TODO: Add INFO
HAL_UART_StateTypeDef Receive_Pause();

// TODO: Add INFO
HAL_UART_StateTypeDef Receive_Start();

// TODO: Add INFO
HAL_UART_StateTypeDef Receive_Stop();

// TODO: Add INFO
void Clear_Receive_Buffer();

/**
 * @brief resets the wifi module
 *
 * @param huart1: the UART typedef
 *
 * @return nothing
 */
HAL_UART_StateTypeDef wifiRST(UART_HandleTypeDef*);

/**
 * @brief initializes the module by setting mode as client, sets baud rate and UART format
 *
 * @param huart1 : the UART typedef
 *
 * @return nothing
 */
HAL_UART_StateTypeDef wifiInit();

/**
 * @brief connects to a wifi access point
 *
 * @param ssid: string with the name of the access point
 * @param pass: password for access point
 * @param huart1: the UART typdef
 *
 * @return nothing
 */
HAL_UART_StateTypeDef connectWifi(char* ssid, char* pass);

/**
 * @brief transmits data to our website, will fix to make more dynamic
 *
 * @param info: POST message payload
 * @param huart1: the UART typedef
 *
 * @return nothing
 */
HAL_UART_StateTypeDef transmitWifi(char* info);



//TODO: Add INFO
HAL_UART_StateTypeDef wifi_get_timestamp();

/**
 * @brief searches a string to see if it contains another string given a specific format
 *
 * @param string: the string to be searched
 * @param key: the string you want to find
 *
 * @return 0 if not found, pointer of the value following the string if found
 */
char* stringParser(char* string, char* key);

#endif
