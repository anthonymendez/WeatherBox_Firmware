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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Mapped to a zero voltage on the first measurement
float zero_voltage = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Toggle_User_LED();
static float adc_to_voltage(uint32_t);
static void calculate_wind_speed(uint32_t, uint32_t, float*, float*);
void bme280_read_data_forced_mode(struct bme280_dev*);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE BEGIN EV */
	/* Unique Device ID */
extern uint32_t stm32_dev_id_word0;
extern uint32_t stm32_dev_id_word1;
extern uint32_t stm32_dev_id_word2;

	/* Timeouts in milliseconds */
extern const int SPI_TIMEOUT;
extern const int I2C_TIMEOUT;
extern const int ADC_TIMEOUT;

	/* HAL Library Configuration Handlers */
extern ADC_HandleTypeDef hadc;
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

	/* BME280 Variables */
extern int bme280_init_complete;
extern struct bme280_settings bme280_device_settings;
extern struct bme280_dev bme280_device;
extern int8_t bme280_rslt;
extern int8_t bme280_init_rslt;
extern uint8_t bme280_settings_sel;
extern struct bme280_data comp_data;

	/* CCS811 Variables */
extern struct ccs811_env_data ccs811_environmental_data;
extern struct ccs811_measurement_data ccs811_measured_data;
extern struct ccs811_dev ccs811_device;
extern int8_t ccs811_init_rslt;
extern int8_t ccs811_rslt;
extern int ccs811_init_complete;

	/* RTC Variables */
extern RTC_TimeTypeDef currentTime;
extern RTC_DateTypeDef currentDate;
extern time_t timestamp_power_on;
extern time_t timestamp;
extern struct tm currTime;

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
  HAL_IncTick();
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
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  uint32_t wind_speed_adc = 0;
  uint32_t wind_temp_adc = 0;
  uint32_t dust_adc = 0;
  float dust_V = 0;
  float dust_voc = 0.6;
  float K = 0.5;
  float bme280_pressure = 0;
  float bme280_temperature = 0;
  float bme280_humidity = 0;
  float md_wind_speed = 0;
  float md_temp = 0;
  char data[250];
  get_current_timestamp();

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  Toggle_User_LED();

  // TODO: Abstract this out into it's own function (not drivers)
  HAL_ADC_Start(&hadc);
  if(HAL_ADC_PollForConversion(&hadc, ADC_TIMEOUT)== HAL_OK)
	  wind_speed_adc = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Start(&hadc);
  if(HAL_ADC_PollForConversion(&hadc, ADC_TIMEOUT)== HAL_OK)
  	  wind_temp_adc = HAL_ADC_GetValue(&hadc);

  HAL_GPIO_TogglePin(GPIOB, Dust_LED_Pin);
  HAL_ADC_Start(&hadc);
  HAL_Delay(280);
  if(HAL_ADC_PollForConversion(&hadc, ADC_TIMEOUT)== HAL_OK)
  	  dust_adc = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);
  HAL_GPIO_TogglePin(GPIOB, Dust_LED_Pin);
  HAL_Delay(9680);
  dust_V = adc_to_voltage(dust_adc);
  if(dust_V < dust_voc)
	  dust_voc = dust_V;
  float dV = dust_V - dust_voc;
  /* Dust Density in units of ug/m3 */
  float dust_density = dV / K * 100.0;

  /* Calculate Windspeed based on ADC Values */
  calculate_wind_speed(wind_speed_adc, wind_temp_adc, &md_wind_speed, &md_temp);

  ccs811_rslt = CCS811_OK;
  ccs811_rslt |= ccs811_read_status_reg(&ccs811_device);
  /* Format Environmental Data Struct */
  ccs811_rslt |= ccs811_convert_temp_and_humid_to_env_data(comp_data.humidity, comp_data.temperature, &ccs811_environmental_data);
  /* Set Environmental Data for CCS811 */
  ccs811_rslt |= ccs811_set_env_data(&ccs811_environmental_data, &ccs811_device);
  /* Wait for data to be ready to read*/
  if (ccs811_device.status_reg & CCS811_STATUS_DATA_READY_MSK)
  {
	  /* Read Algorithm Results from CCS811 */
	  /* TODO: Wait 20 min at startup for CCS811 burnin */
	  ccs811_rslt |= ccs811_read_alg_result_data(&ccs811_measured_data, &ccs811_device);
  }

  /* Calculations Done Here */
  /* Data is output to comp_data */
  // TODO: Double check later if this is properly compensated
  bme280_read_data_forced_mode(&bme280_device);
  bme280_temperature = comp_data.temperature * 0.01; // Celsius
  bme280_humidity = comp_data.humidity / 1024.0; // Output is in percentage... so 43.33 is 43.33 %rH
  bme280_pressure = comp_data.pressure * 0.01; // hPa Pressure Units... for Debug Purposes

  /* Transmit over WiFi
   * Only transmit CCS811 device if data was ready to be read
   * TODO: Change how we construct the JSON String
   */
  if (ccs811_device.status_reg & CCS811_STATUS_DATA_READY_MSK)
  {
	  sprintf(data, "{ \"system_id\":\"%lu%lu%lu\", "
	  		  	  	  "\"timestamp\":\"-1\", "
	  		  	  	  "\"temperature\":\"%f\", "
	  		  	  	  "\"wind_speed\":\"%f\", "
	  		  	  	  "\"pressure\":\"%f\", "
	  		  	  	  "\"humidity\":\"%f\", "
	  		  	  	  "\"air_quality\":\"%u\", "
	  		  	  	  "\"dust\":\"%f\" }",
	  				  stm32_dev_id_word0,
	  				  stm32_dev_id_word1,
	  				  stm32_dev_id_word2,
	  				  bme280_temperature,
	  				  md_wind_speed,
	  				  bme280_pressure,
	  				  bme280_humidity,
	  				  ccs811_measured_data.eco2,
					  dust_density);
  }
  else
  {
	  sprintf(data, "{ \"system_id\":\"%lu%lu%lu\", "
	  	  		  	  "\"timestamp\":\"-1\", "
	  	  		  	  "\"temperature\":\"%f\", "
	  	  		  	  "\"wind_speed\":\"%f\", "
	  	  		  	  "\"pressure\":\"%f\", "
	  	  		  	  "\"humidity\":\"%f\", "
			  	      "\"dust\":\"%f\", }",
	  	  			  stm32_dev_id_word0,
	  	  			  stm32_dev_id_word1,
	  	  			  stm32_dev_id_word2,
	  	  			  bme280_temperature,
	  	  			  md_wind_speed,
	  	  			  bme280_pressure,
	  	  			  bme280_humidity,
					  dust_density);
  }

  transmitWifi(data);

  /* TODO: Read from WiFi Module success code */
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
 * 	@brief Function handles converting adc value to a voltage.
 * 	Call reverse_and_shift_adc_value before hand.
 */
static float adc_to_voltage(uint32_t adc_value)
{
	return 3.3 * adc_value / 4096;
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
 * 	https://moderndevice.com/uncategorized/calibrating-rev-p-wind-sensor-new-regression/?preview=true
 */
static void calculate_wind_speed(uint32_t wind_speed_adc, uint32_t wind_temp_adc, float *wind_speed, float *temp_amb)
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
	*temp_amb = (wind_temp_vout - 0.400) / 0.0195;

	// Calculate the Wind Speed in MPH
	*wind_speed = (wind_speed_vout - zero_voltage) / (3.038517 * pow(*temp_amb, 0.115157));
	*wind_speed /= 0.087288;
	*wind_speed = pow(*wind_speed, 3.009364);

	if (isnanf(*wind_speed))
	{
		*wind_speed = 0;
	}
}

void bme280_read_data_forced_mode(struct bme280_dev *dev)
{
	bme280_rslt = 0;
	/* Set measurement mode to Forced */
	bme280_rslt |= bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
	/* Wait for the measurement to complete */
	dev->delay_ms(500);
	/* Output data to comp_data */
	bme280_rslt |= bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
	/* Set sensor to Sleep */
	bme280_rslt |= bme280_set_sensor_mode(BME280_SLEEP_MODE, dev);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
