/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	/* ADC Chip Select Pin */
#define ADC_CS_PIN			SS0_Pin
	/* ADC Chip Select GPIO Port */
#define ADC_CS_GPIO_Port	SS0_GPIO_Port
	/* ADC Start Bit */
#define ADC_START_BIT 0b00000001
	/* ADC SGL/Diff + Channel Select
	 * Leftmost Bit - SGL/Diff
	 * 2nd Leftmost - D2
	 * 3rd Leftmost - D1
	 * 4th Leftmost - D0
	 */
#define ADC_DIN_CH0 0b10000000
#define ADC_DIN_CH1 0b10010000
#define ADC_DIN_CH2 0b10100000
#define ADC_DIN_CH3 0b10110000
#define ADC_DIN_CH4 0b11000000
#define ADC_DIN_CH5 0b11010000
#define ADC_DIN_CH6 0b11100000
#define ADC_DIN_CH7 0b11110000
	/* Channel Select Mapped to Sensors */
#define ADC_WIND_SENSOR_SPEED_CH ADC_DIN_CH0
#define ADC_WIND_SENSOR_TEMP_CH ADC_DIN_CH1
	/* ADC Output Mask */
#define ADC_OUTPUT_MASK 0x03FF
	/* Temperature, Pressure, Humidity Addresses */
#define TPH_OPEN_ADDRESS 0x77
#define TPH_CLOSED_ADDRESS 0x76
	/* Air Quality Addresses */
#define AQ_OPEN_ADDRESS 0x5B
#define AQ_CLOSED_ADDRESS 0x5A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_16_TO_10_BIT(BYTE_1, BYTE_2)  (((BYTE_1 << 8) | BYTE_2) & ADC_OUTPUT_MASK)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Mapped to a zero voltage on the first measurement
float zero_voltage = -1;
const int SPI_TIMEOUT = 500;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Toggle_User_LED();
void Toggle_ADC_Chip_Select();
void HAL_SPI_Transmit_Start();
void Read_ADC(uint8_t, uint16_t*);
static uint16_t reverse(uint16_t);
static float adc_to_voltage(uint16_t);
static float calculate_wind_speed(uint16_t, uint16_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable Interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */

    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  uint16_t wind_speed_digital = 0;
  uint16_t wind_temp_digital = 0;
  uint16_t din_ch2 = 0;
  uint16_t din_ch3 = 0;
  uint16_t din_ch4 = 0;
  uint16_t din_ch5 = 0;
  uint16_t din_ch6 = 0;
  uint16_t din_ch7 = 0;
  uint16_t tph_data = 0;

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  Toggle_User_LED();

  /* Read ADC Wind Speed Sensor Channel */
  Read_ADC((uint8_t) ADC_WIND_SENSOR_SPEED_CH, &wind_speed_digital);

  /* Read ADC Wind Temp Sensor Channel */
  Read_ADC((uint8_t) ADC_WIND_SENSOR_TEMP_CH, &wind_temp_digital);

  Read_ADC((uint8_t) ADC_DIN_CH2, &din_ch2);
  Read_ADC((uint8_t) ADC_DIN_CH3, &din_ch3);
  Read_ADC((uint8_t) ADC_DIN_CH4, &din_ch4);
  Read_ADC((uint8_t) ADC_DIN_CH5, &din_ch5);
  Read_ADC((uint8_t) ADC_DIN_CH6, &din_ch6);
  Read_ADC((uint8_t) ADC_DIN_CH7, &din_ch7);

  // TODO: Figure out how to communicate with the TPH sensor
//  HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)(TPH_OPEN_ADDRESS), &tph_data, sizeof(tph_data));

//  /* Toggle SS1 Pin Low to select sensor */
//  HAL_GPIO_TogglePin(SS1_GPIO_Port, SS1_Pin);
//  // TODO: Read from Sensor
//  HAL_GPIO_TogglePin(SS1_GPIO_Port, SS1_Pin);
//  /* Toggle SS1 High to un-select sensor */
//
//  /* Toggle SS2 Pin Low to select sensor */
//  HAL_GPIO_TogglePin(SS2_GPIO_Port, SS2_Pin);
//  // TODO: Read from Sensor
//  HAL_GPIO_TogglePin(SS2_GPIO_Port, SS2_Pin);
//  /* Toggle SS2 High to un-select sensor */

  /* Calculations Done Here */
  float wind_speed = calculate_wind_speed(wind_speed_digital, wind_temp_digital);
  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
 * 	@brief Toggle LED
 */
void Toggle_User_LED()
{
	HAL_GPIO_TogglePin(User_LED_GPIO_Port, User_LED_Pin);
}

/**
 * 	@brief Toggle ADC Chip Select Pin
 */
void Toggle_ADC_Chip_Select()
{
	HAL_GPIO_TogglePin(ADC_CS_GPIO_Port, ADC_CS_PIN);
}

/**
 * 	@brief Transmit Start Bit in HAL SPI
 */
void HAL_SPI_Transmit_Start()
{
	uint8_t adc_start = (uint8_t) ADC_START_BIT;
	HAL_SPI_Transmit(&hspi1, &adc_start, sizeof(adc_start), SPI_TIMEOUT);
}

/**
 * 	@brief Read MCP3008 ADC based on given channel enum.
 *	Output is set to the pointer of a uint16_t set in the parameters.
 *	Output will be set to 0 before setting the ADC Value to it.
 */
void Read_ADC(uint8_t adc_ch_select, uint16_t *output)
{
	uint8_t adc_byte_1 = 0;
	uint8_t adc_byte_2 = 0;
	/* Set output to 0 */
	*output = 0;

	/* Toggle SS0 Pin (CS) Low to use ADC */
	Toggle_ADC_Chip_Select();
	/* Send to DIN CH0 Select */
	HAL_SPI_Transmit_Start();
	HAL_SPI_TransmitReceive(&hspi1, &adc_ch_select, &adc_byte_1, sizeof(adc_ch_select), SPI_TIMEOUT);
	/* Read from Dout of ADC */
	HAL_SPI_Receive(&hspi1, &adc_byte_2, sizeof(adc_byte_2), SPI_TIMEOUT);
	/* Toggle SS0 High (CS) to signify we're done with a round of the ADC */
	Toggle_ADC_Chip_Select();
	/* Set Output to adc_value */
	*output = ADC_16_TO_10_BIT(adc_byte_1, adc_byte_2);
}

/**
 * 	@brief Function handles converting from LSB o MSB and vice versa
 */
static uint16_t reverse(uint16_t x)
{
	/* Retrieved from http://www.geekviewpoint.com/java/bitwise/reverse_bits_short */
	uint16_t modX = x;
	uint16_t y = 0;
	int position = 7;
	for(; position >= 0; position--){
		y += ((modX&1) << position);
		modX >>= 1;
	}
	return (uint16_t) y;
}

/**
 * 	@brief Function handles converting adc value to a voltage.
 * 	Call reverse_and_shift_adc_value before hand.
 */
static float adc_to_voltage(uint16_t adc_value)
{
	return 5.0 * adc_value / 1024;;
}

/**
 * 	@brief Function handles calculating wind speed from the input voltage.
 * 	Uses the equation from Modern Device
 *  WS_MPH = (((Volts – ZeroWind_V) / (3.038517 * (Temp_C ^ 0.115157 ))) / 0.087288 ) ^ 3.009364
 *	WS_MPH: wind speed in miles per hour
 *	Volts: the output of the Wind Sensor Rev. P at the “Out�? pin in volts
 *	Temp_C: temperature in degrees C
 *	ZeroWind_V: zero wind voltage – measured with the sensor angled at the edge of a table with a glass
 *	over the tip (loop etc.) of the sensor. Let the sensor stabilize for 40 seconds or so, when the voltage
 *	stops dropping and stabilizes, record the voltage.
 *
 *	To calculate Ambient Temperature from the Modern Device Sensor
 *	Tambient = ( Vout – V0°C ) / TC
 *	Checking the datasheet,
 *	VOUT = Temperature Output Voltage
 *	TC, temperature coefficient for our part is 19.5 mV/°C
 *	V0°C, the voltage at zero degrees C is 400 mV
 *	Tambient = ( Vout – 0.400 ) / 0.0195
 *	TODO: Wait 40 seconds until we do the first measurement to let the wind sensor stabilize
 *	TODO: Use the Bosch sensor for ambiant temperature instead of the onboard Modern Device Sensor
 * 	https://moderndevice.com/uncategorized/calibrating-rev-p-wind-sensor-new-regression/?preview=true
 */
static float calculate_wind_speed(uint16_t wind_speed_adc, uint16_t wind_temp_adc)
{
	// Calculate Vin from ADC
	float wind_speed_vout = adc_to_voltage(wind_speed_adc);
	float wind_temp_vout = adc_to_voltage(wind_temp_adc);

	// Zero Voltage not set, set here
	// TODO: Check if 40 seconds of operation have passed before we decided to set the zero voltage.
	if (zero_voltage == -1) {
		zero_voltage = wind_speed_vout;
	}

	// Calculate Ambient Temperature in Celsius
	float TempAmb = (wind_temp_vout - 0.400) / 0.0195;
	float zero = zero_voltage;

	// Calculate the Wind Speed in MPH
	float wind_speed = (wind_speed_vout - zero_voltage) / (3.038517 * pow(TempAmb, 0.115157));
	wind_speed /= 0.087288;
	wind_speed = pow(wind_speed, 3.009364);
	return wind_speed;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
