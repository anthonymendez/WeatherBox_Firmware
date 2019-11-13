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
/*
 * Left 3 bits are channel select
 * 2nd Bit from the right is SGL/DIFF
 * Rightmost Bit is start bit
 */
#define ADC_DIN_CH0 0b00011
#define ADC_DIN_CH1 0b10011
#define ADC_DIN_CH2 0b01011
#define ADC_DIN_CH3 0b11011
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static uint16_t convert_from_adc(uint16_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */
extern SPI_HandleTypeDef hspi1;
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
  uint16_t wind_speed_adc = 0;
  uint16_t adc_ch_select = (uint16_t)(ADC_DIN_CH0);
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  int timeout = 500;
  /* Toggle LED to signify interrupt */
  HAL_GPIO_TogglePin(User_LED_GPIO_Port, User_LED_Pin);

  /* Toggle SS0 Pin (CS) Low to select wind sensor */
  HAL_GPIO_TogglePin(SS0_GPIO_Port, SS0_Pin);
  /* Send to DIN CH0 Select */
  HAL_SPI_Transmit(&hspi1, &adc_ch_select, sizeof(adc_ch_select), timeout);
  /* Read from Dout of ADC */
  HAL_SPI_Receive(&hspi1, &wind_speed_adc, sizeof(wind_speed_adc), timeout);
  // TODO: Read from Sensor
  HAL_GPIO_TogglePin(SS0_GPIO_Port, SS0_Pin);
  /* Toggle SS0 High (CS) to un-select sensor */

  /* Calculate Wind Sensor Voltage */
  uint16_t wind_speed_digital = convert_from_adc(wind_speed_adc);
  float wind_speed_vin = 5.0 * wind_speed_digital / 1024;

  /* Toggle SS1 Pin Low to select sensor */
  HAL_GPIO_TogglePin(SS1_GPIO_Port, SS1_Pin);
  // TODO: Read from Sensor
  HAL_GPIO_TogglePin(SS1_GPIO_Port, SS1_Pin);
  /* Toggle SS1 High to un-select sensor */

  /* Toggle SS2 Pin Low to select sensor */
  HAL_GPIO_TogglePin(SS2_GPIO_Port, SS2_Pin);
  // TODO: Read from Sensor
  HAL_GPIO_TogglePin(SS2_GPIO_Port, SS2_Pin);
  /* Toggle SS2 High to un-select sensor */
  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
static unsigned int reverse(uint16_t x)
{
	/* Retrieved from https://stackoverflow.com/questions/746171/efficient-algorithm-for-bit-reversal-from-msb-lsb-to-lsb-msb-in-c */
	uint16_t y = 0;
	int position = 15;
	for(; position >= 0; position--){
		y += ((x&1) << position);
		x >>= 1;
	}
	return y;
}

static uint16_t convert_from_adc(uint16_t adc_value)
{
	return reverse(adc_value) << 2;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
