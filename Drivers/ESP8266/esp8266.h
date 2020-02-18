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

/**
 * @brief resets the wifi module
 *
 * @param huart1: the UART typedef
 *
 * @return nothing
 */
void wifiRST(UART_HandleTypeDef huart1);

/**
 * @brief initializes the module by setting mode as client, sets baud rate and UART format
 *
 * @param huart1 : the UART typedef
 *
 * @return nothing
 */
void wifiInit(UART_HandleTypeDef huart1);

/**
 * @brief connects to a wifi access point
 *
 * @param ssid: string with the name of the access point
 * @param pass: password for access point
 * @param huart1: the UART typdef
 *
 * @return nothing
 */
void connectWifi(char* ssid, char* pass, UART_HandleTypeDef huart1);

/**
 * @brief transmits data to our website, will fix to make more dynamic
 *
 * @param info: POST message payload
 * @param huart1: the UART typedef
 *
 * @return nothing
 */
void transmitWifi(char* info, UART_HandleTypeDef huart1);



//TODO: Add INFO
void wifi_get_timestamp();
static unsigned char reverse(unsigned char b);

#endif
