/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
	/* Unique Device ID */
uint32_t stm32_dev_id_word0;
uint32_t stm32_dev_id_word1;
uint32_t stm32_dev_id_word2;

	/* Timeouts in milliseconds */
const int SPI_TIMEOUT = 500;
const int I2C_TIMEOUT = 500;
const int ADC_TIMEOUT = 500;

	/* BME280 Variables */
int bme280_init_complete = 0;
struct bme280_settings bme280_device_settings;
struct bme280_dev bme280_device;
int8_t bme280_init_rslt = BME280_OK;
int8_t bme280_rslt = BME280_OK;
uint8_t bme280_settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
struct bme280_data comp_data;

	/* CCS811 Variables */
struct ccs811_fw_boot_version ccs811_firmware_boot_version;
struct ccs811_fw_app_version ccs811_firmware_app_version;
struct ccs811_ntc ccs811_ntc_data;
struct ccs811_env_data ccs811_environmental_data;
struct ccs811_measurement_data ccs811_measured_data;
struct ccs811_dev ccs811_device;
uint16_t ccs811_baseline;
int8_t ccs811_init_rslt = CCS811_OK;
int8_t ccs811_rslt = CCS811_OK;
int ccs811_init_complete = 0;

	/* RTC Variables */
RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;
time_t timestamp_power_on;
time_t timestamp;
struct tm currTime;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void BME280_INIT(void);
static void CCS811_INIT(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Get Unique Device ID */
  stm32_dev_id_word0 = HAL_GetUIDw0();
  stm32_dev_id_word1 = HAL_GetUIDw1();
  stm32_dev_id_word2 = HAL_GetUIDw2();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_ADC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  get_current_timestamp();

  wifiRST(huart1);
  //HAL_Delay(1000);
  wifiInit(huart1);
  wifi_get_timestamp();
  HAL_Delay(1000);
  BME280_INIT();
  bme280_init_complete = 1;
  CCS811_INIT();
  ccs811_init_complete = 1;
  connectWifi("WeatherBox", "WinDrone807", huart1);
  timestamp_power_on = timestamp;
  //HAL_Delay(5000);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000609;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SS0_GPIO_Port, SS0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SS1_GPIO_Port, SS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SS2_Pin|CCS811_RST_Pin|CCS811_WAKE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Dust_LED_GPIO_Port, Dust_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : User_LED_Pin */
  GPIO_InitStruct.Pin = User_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(User_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SS0_Pin SS1_Pin */
  GPIO_InitStruct.Pin = SS0_Pin|SS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SS2_Pin Dust_LED_Pin CCS811_RST_Pin CCS811_WAKE_Pin */
  GPIO_InitStruct.Pin = SS2_Pin|Dust_LED_Pin|CCS811_RST_Pin|CCS811_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * Initializes Bosch BME280 Temperature, Pressure, and Humidity Sensor
 */
static void BME280_INIT(void)
{
	/* Device Sampling, Filter, and Standby Time Settings */
	/* Recommended mode of operation: Indoor navigation */
	bme280_device_settings.osr_p = BME280_OVERSAMPLING_16X;		// Pressure
	bme280_device_settings.osr_t = BME280_OVERSAMPLING_2X;		// Temperature
	bme280_device_settings.osr_h = BME280_OVERSAMPLING_1X;		// Humidity
	bme280_device_settings.filter = BME280_FILTER_COEFF_16;		// Filter
	bme280_device_settings.standby_time = 0;					// Standby Time
	bme280_device.dev_id = BME280_I2C_ADDR_SEC;					// I2C Address
	bme280_device.intf = BME280_I2C_INTF;						// I2C Mode
	bme280_device.read = user_i2c_read;							// Read Function Ptr
	bme280_device.write = user_i2c_write;						// Write Function Ptr
	bme280_device.delay_ms = user_delay_ms;						// Delay Function Ptr
	bme280_device.settings = bme280_device_settings;			// Device Settings set above
	bme280_init_rslt |= bme280_init(&bme280_device);					// Initizialize Device
	bme280_init_rslt |= bme280_set_sensor_settings(bme280_settings_sel, &bme280_device); // Apply Settings
	bme280_init_rslt |= bme280_set_sensor_mode(BME280_SLEEP_MODE, &bme280_device); // Set to sleep mode
}

/**
 *Initializes AMS CCS811 Air Quality Sensor
 */
static void CCS811_INIT(void)
{
	ccs811_device.dev_addr = CCS811_I2C_ADDR_SEC;
	ccs811_device.measure_mode_reg = (uint8_t)(CCS811_DRIVE_MODE_CONSTANT_1s_MODE);
	ccs811_device.read = ccs811_read;
	ccs811_device.write = ccs811_write;
	ccs811_device.delay_ms = user_delay_ms;
	HAL_GPIO_WritePin(CCS811_RST_GPIO_Port, CCS811_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(CCS811_RST_GPIO_Port, CCS811_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(CCS811_WAKE_GPIO_Port, CCS811_WAKE_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(CCS811_WAKE_GPIO_Port, CCS811_WAKE_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	ccs811_device.hw_id = 0;
	ccs811_device.hw_version = 0;
	ccs811_device.status_reg = 0;
	ccs811_device.error_reg = 0;
	ccs811_init_rslt |= ccs811_init(&ccs811_device);
	ccs811_init_rslt |= ccs811_read_fw_boot_version(&ccs811_firmware_boot_version, &ccs811_device);
	ccs811_init_rslt |= ccs811_read_status_reg(&ccs811_device);
	ccs811_init_rslt |= ccs811_read_fw_app_version(&ccs811_firmware_app_version, &ccs811_device);
	ccs811_init_rslt |= ccs811_read_status_reg(&ccs811_device);
	/* NTC Register requires external temperature sensor to be hooked up using resistors.
	 * See Programming Datasheet
	 *	ccs811_init_rslt |= ccs811_read_ntc(&ccs811_ntc_data, &ccs811_device);
	 *	ccs811_init_rslt |= ccs811_read_status_reg(&ccs811_device);
	*/
	ccs811_init_rslt |= ccs811_read_baseline_reg(&ccs811_baseline, &ccs811_device);
	ccs811_init_rslt |= ccs811_read_status_reg(&ccs811_device);
}

/**
 *	CCS811 Read Wrapper Function
 *	Wraps user_i2c_read
 */
int8_t ccs811_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	/* Remove device from sleep mode */
	HAL_GPIO_WritePin(CCS811_WAKE_GPIO_Port, CCS811_WAKE_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);

	int8_t rslt = user_i2c_read(dev_id, reg_addr, reg_data, len);

	/* Put device into sleep mode */
	HAL_GPIO_WritePin(CCS811_WAKE_GPIO_Port, CCS811_WAKE_Pin, GPIO_PIN_SET);

	return rslt;
}

/**
 *	CCS811 Write Wrapper Function
 *	Wraps user_i2c_write
 */
int8_t ccs811_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	/* Remove device from sleep mode */
	HAL_GPIO_WritePin(CCS811_WAKE_GPIO_Port, CCS811_WAKE_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);

	int8_t rslt = user_i2c_write(dev_id, reg_addr, reg_data, len);

	/* Put device into sleep mode */
	HAL_GPIO_WritePin(CCS811_WAKE_GPIO_Port, CCS811_WAKE_Pin, GPIO_PIN_SET);

	return rslt;
}

/*
 *	@brief Function Pointer for Delaying the BME280.
 *	After a number of milliseconds have passed, we
 *	return control.
 * 	@param[in] milliseconds : How much to delay by in milliseconds.
 */
void user_delay_ms(uint32_t milliseconds)
{
	HAL_Delay(milliseconds);
}

/*
 *	@brief Function Pointer for reading data from the BME280 or CCS811 using the I2C protocol.
 * 	@param[in] dev_id : I2C address of the device.
 * 	@param[in] reg_addr : Register address of what we want to read in from the sensor.
 * 	@param[out] reg_data : Data we're reading out from the register.
 * 	@param[in] len : Amount of registers to read from
 *
 *************************************************************
 *
 */
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	/*
	 * Data on the bus should be like
	 * |------------+---------------------|
	 * | I2C action | Data                |
	 * |------------+---------------------|
	 * | Start      | -                   |
	 * | Write      | (reg_addr)          |
	 * | Stop       | -                   |
	 * | Start      | -                   |
	 * | Read       | (reg_data[0])       |
	 * | Read       | (....)              |
	 * | Read       | (reg_data[len - 1]) |
	 * | Stop       | -                   |
	 * |------------+---------------------|
	 */
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	uint16_t read_mode = dev_id;
	/* Check if our dev_id is already left shifted with a read bit */
	read_mode = (dev_id << 1) | 1;
	// Initing then Deiniting fixed I2C Busy Flag bug
	HAL_I2C_Init(&hi2c1);
	rslt |= HAL_I2C_Mem_Read(&hi2c1, read_mode, reg_addr, sizeof(uint8_t), reg_data, len, I2C_TIMEOUT);
	HAL_Delay(500);
	HAL_I2C_DeInit(&hi2c1);
	return rslt;
}

/*
 *	@brief Function Pointer for writing data to the BME280 using the I2C protocol.
 * 	@param[in] dev_id : I2C address of the device.
 * 	@param[in] reg_addr : Register address of what we want to read in from the BME280.
 * 	@param[in] reg_data : Data we're writing to the register
 * 	@param[in] len : Amount of registers to write to
 */
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	/*
	 * Data on the bus should be like
	 * |------------+---------------------|
	 * | I2C action | Data                |
	 * |------------+---------------------|
	 * | Start      | -                   |
	 * | Write      | (reg_addr)          |
	 * | Write      | (reg_data[0])       |
	 * | Write      | (....)              |
	 * | Write      | (reg_data[len - 1]) |
	 * | Stop       | -                   |
	 * |------------+---------------------|
	 */
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	uint16_t write_mode = dev_id;
	/* Check if our dev_id is already left shifted with a write bit */
	write_mode = (dev_id << 1) | 0;
	// Initing then Deiniting fixed I2C Busy Flag bug
	HAL_I2C_Init(&hi2c1);
	rslt = HAL_I2C_Mem_Write(&hi2c1, write_mode, reg_addr, sizeof(uint8_t), reg_data, len, I2C_TIMEOUT);
	HAL_Delay(500);
	HAL_I2C_DeInit(&hi2c1);
	return rslt;
}


void wifi_get_timestamp()
{
	char start[] = "AT+CIPSTART=\"TCP\",\"weatherbox.azurewebsites.net\",80\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *) start, strlen(start), 500);
	HAL_Delay(2000);
	char send[] = "AT+CIPSEND=";
	char recv[] = "AT+CIPRECVDATA=1000\r\n";
	char ret[] = "\r\n";
	char get[] = "GET /timestamp HTTP/1.1\r\nAccept: \"*/*\"\r\nHost: weatherbox.azurewebsites.net\r\n\r\n";
	int get_size = (int)(strlen(get));
	char get_str[sizeof(get_size)];
	sprintf(get_str, "%u", get_size);
	char *receiveBuffer0 = calloc(1000, sizeof(char));


	// Send Command with size of message
	HAL_UART_Transmit(&huart1, (uint8_t *) send, strlen(send), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *) get_str, strlen(get_str), 500);
	HAL_UART_Transmit(&huart1, (uint8_t *) ret, strlen(ret), 500);
	HAL_Delay(1000);

	//Sending GET message
	HAL_UART_Transmit(&huart1, (uint8_t *) get, strlen(get), 500);
//	HAL_UART_Transmit(&huart1, (uint8_t *) ret, strlen(ret), 500);
//	HAL_UART_Transmit(&huart1, (uint8_t *) recv, strlen(recv), 500);
	HAL_UART_Receive(&huart1, &receiveBuffer0, strlen(receiveBuffer0), 5000);
}

/*
 * 	@brief Function to set time_t timestamp above to the current timestamp
 * 	IMPORTANT: ONLY USE AFTER WE GET TIMESTAMP FROM SERVER!
 *
 *
 *  https://stackoverflow.com/questions/48440051/how-to-generate-a-timestamp-in-stm32f303
 *  You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock the values
 *  in the higher-order calendar shadow registers to ensure consistency between the time and date values.
 *  Reading RTC current time locks the values in calendar shadow registers until Current date is read
 *  to ensure consistency between the time and date values.
 */
void get_current_timestamp()
{
	HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);

	currTime.tm_year = currentDate.Year + 2019;  // In fact: 2000 + 18 - 1900
	currTime.tm_mday = currentDate.Date;
	currTime.tm_mon  = currentDate.Month;

	currTime.tm_hour = currentTime.Hours;
	currTime.tm_min  = currentTime.Minutes;
	currTime.tm_sec  = currentTime.Seconds;

	timestamp = mktime(&currTime);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
