/*
 * ccs811_defs.h
 *
 *  Created on: Nov 22, 2019
 *      Author: Anthony Mendez
 */

#ifndef CCS811_CCS811_DEFS_H_
#define CCS811_CCS811_DEFS_H_

/********************************************************/
/* header includes */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/********************************************************/
/*! @name       Common macros               */
/********************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)   S8_C(x)
#define UINT8_C(x)  U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)  S16_C(x)
#define UINT16_C(x) U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)  S32_C(x)
#define UINT32_C(x) U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)  S64_C(x)
#define UINT64_C(x) U64_C(x)
#endif

/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif

/********************************************************/

/**\name CCS811 Hardware Identifier */
#define CCS811_HW_ID			UINT8_C(0x81)

/**\name I2C Addresses */
#define CCS811_I2C_ADDR_PRIM    UINT8_C(0x5A)
#define CCS811_I2C_ADDR_SEC     UINT8_C(0x5B)

/**\name API success code */
#define CCS811_OK                        INT8_C(0)

/**\name API error codes */
#define CCS811_E_NULL_PTR                INT8_C(0b1 << 0)
#define CCS811_E_DEV_NOT_FOUND           INT8_C(0b1 << 1)
#define CCS811_E_INVALID_LEN             INT8_C(0b1 << 2)
#define CCS811_E_COMM_FAIL               INT8_C(0b1 << 3)
#define CCS811_E_SLEEP_MODE_FAIL         INT8_C(0b1 << 4)
#define CCS811_E_NVM_COPY_FAILED         INT8_C(0b1 << 5)
#define CCS811_E_STATUS_REG_ERROR        INT8_C(0b1 << 6)

/**\name Register Address */
#define CCS811_STATUS_ADDR               UINT8_C(0x00)
#define CCS811_MEAS_MODE_ADDR            UINT8_C(0x01)
#define CCS811_ALG_RESULT_DATA_ADDR      UINT8_C(0x02)
#define CCS811_RAW_DATA_ADDR             UINT8_C(0x03)
#define CCS811_ENV_DATA_ADDR             UINT8_C(0x05)
#define CCS811_NTC_ADDR               	 UINT8_C(0x06)
#define CCS811_THRESHOLDS_ADDR           UINT8_C(0x10)
#define CCS811_BASELINE_ADDR             UINT8_C(0x11)
#define CCS811_HW_ID_ADDR                UINT8_C(0x20)
#define CCS811_HW_VERSION_ADDR           UINT8_C(0x21)
#define CCS811_FW_BOOT_VERSION_ADDR      UINT8_C(0x23)
#define CCS811_FW_APP_VERSION_ADDR       UINT8_C(0x24)
#define CCS811_ERROR_ID_ADDR             UINT8_C(0xE0)
#define CCS811_APP_ERASE_ADDR            UINT8_C(0xF1)
#define CCS811_APP_DATA_ADDR             UINT8_C(0xF2)
#define CCS811_APP_VERIFY_ADDR           UINT8_C(0xF3)
#define CCS811_APP_START_ADDR            UINT8_C(0xF4)
#define CCS811_SW_RESET_ADDR             UINT8_C(0xFF)

/**\name Register Sizes in Bytes */
#define CCS811_STATUS_BYTE_LEN               UINT8_C(0x01)
#define CCS811_MEAS_MODE_BYTE_LEN            UINT8_C(0x01)
#define CCS811_ALG_RESULT_DATA_BYTE_LEN      UINT8_C(0x08)
#define CCS811_RAW_DATA_BYTE_LEN             UINT8_C(0x02)
#define CCS811_ENV_DATA_BYTE_LEN             UINT8_C(0x04)
#define CCS811_NTC_BYTE_LEN               	 UINT8_C(0x04)
#define CCS811_THRESHOLDS_BYTE_LEN           UINT8_C(0x05)
#define CCS811_BASELINE_BYTE_LEN             UINT8_C(0x02)
#define CCS811_HW_ID_BYTE_LEN                UINT8_C(0x01)
#define CCS811_HW_VERSION_BYTE_LEN           UINT8_C(0x01)
#define CCS811_FW_BOOT_VERSION_BYTE_LEN      UINT8_C(0x02)
#define CCS811_FW_APP_VERSION_BYTE_LEN       UINT8_C(0x02)
#define CCS811_ERROR_ID_BYTE_LEN             UINT8_C(0x01)
#define CCS811_APP_ERASE_BYTE_LEN            UINT8_C(0x04)
#define CCS811_APP_DATA_BYTE_LEN             UINT8_C(0x09)
#define CCS811_APP_VERIFY_BYTE_LEN           UINT8_C(0x01)
#define CCS811_APP_START_BYTE_LEN            UINT8_C(0x00)
#define CCS811_SW_RESET_BYTE_LEN             UINT8_C(0x04)

/**\name Sensor Operation Modes */
#define CCS811_DRIVE_MODE_IDLE_MODE                 UINT8_C(0x00)
#define CCS811_DRIVE_MODE_CONSTANT_1s_MODE          UINT8_C(0x10)
#define CCS811_DRIVE_MODE_PULSE_10s_MODE            UINT8_C(0x20)
#define CCS811_DRIVE_MODE_LP_PULSE_60s_MODE         UINT8_C(0x30)
#define CCS811_DRIVE_MODE_CONSTANT_250ms_MODE       UINT8_C(0x40)

/**\name Macros for bit masking registers */
#define CCS811_STATUS_FW_MODE_MSK		UINT8_C(0x80)
#define CCS811_STATUS_APP_VALID_MSK		UINT8_C(0x10)
#define CCS811_STATUS_DATA_READY_MSK	UINT8_C(0x08)
#define CCS811_STATUS_ERROR_MSK		 	UINT8_C(0x01)

#define CCS811_MEAS_MODE_DRIVE_MODE_MSK	 		UINT8_C(0x70)
#define CCS811_MEAS_MODE_INT_DATARDY_MSK 		UINT8_C(0x08)
#define CCS811_MEAS_MODE_INT_THRESH_MSK	 		UINT8_C(0x04)

#define CCS811_RAW_DATA_CURRENT_MSK		UINT16_C(0xFC00)
#define CCS811_RAW_DATA_RAW_ADC_MSK		UINT16_C(0x03FF)

#define CCS811_ENV_DATA_HUMIDITY_PERC_MSK			UINT16_C(0xFE00)
#define CCS811_ENV_DATA_HUMIDITY_FRAC_MSK			UINT16_C(0x01FF)
#define CCS811_ENV_DATA_HUMIDITY_PERC_64_MSK		UINT16_C(0x8000)
#define CCS811_ENV_DATA_HUMIDITY_PERC_32_MSK		UINT16_C(0x4000)
#define CCS811_ENV_DATA_HUMIDITY_PERC_16_MSK		UINT16_C(0x2000)
#define CCS811_ENV_DATA_HUMIDITY_PERC_8_MSK			UINT16_C(0x1000)
#define CCS811_ENV_DATA_HUMIDITY_PERC_4_MSK			UINT16_C(0x0800)
#define CCS811_ENV_DATA_HUMIDITY_PERC_2_MSK			UINT16_C(0x0400)
#define CCS811_ENV_DATA_HUMIDITY_PERC_1_MSK			UINT16_C(0x0200)
#define CCS811_ENV_DATA_HUMIDITY_FRAC_1_2_MSK		UINT16_C(0x0100)
#define CCS811_ENV_DATA_HUMIDITY_FRAC_1_4_MSK		UINT16_C(0x0080)
#define CCS811_ENV_DATA_HUMIDITY_FRAC_1_8_MSK		UINT16_C(0x0040)
#define CCS811_ENV_DATA_HUMIDITY_FRAC_1_16_MSK		UINT16_C(0x0020)
#define CCS811_ENV_DATA_HUMIDITY_FRAC_1_32_MSK		UINT16_C(0x0010)
#define CCS811_ENV_DATA_HUMIDITY_FRAC_1_64_MSK		UINT16_C(0x0008)
#define CCS811_ENV_DATA_HUMIDITY_FRAC_1_128_MSK		UINT16_C(0x0004)
#define CCS811_ENV_DATA_HUMIDITY_FRAC_1_256_MSK		UINT16_C(0x0002)
#define CCS811_ENV_DATA_HUMIDITY_FRAC_1_512_MSK		UINT16_C(0x0001)

#define CCS811_ENV_DATA_TEMPERATURE_PERC_MSK			UINT16_C(0xFE00)
#define CCS811_ENV_DATA_TEMPERATURE_FRAC_MSK			UINT16_C(0x01FF)
#define CCS811_ENV_DATA_TEMPERATURE_PERC_64_MSK			UINT16_C(0x8000)
#define CCS811_ENV_DATA_TEMPERATURE_PERC_32_MSK			UINT16_C(0x4000)
#define CCS811_ENV_DATA_TEMPERATURE_PERC_16_MSK			UINT16_C(0x2000)
#define CCS811_ENV_DATA_TEMPERATURE_PERC_8_MSK			UINT16_C(0x1000)
#define CCS811_ENV_DATA_TEMPERATURE_PERC_4_MSK			UINT16_C(0x0800)
#define CCS811_ENV_DATA_TEMPERATURE_PERC_2_MSK			UINT16_C(0x0400)
#define CCS811_ENV_DATA_TEMPERATURE_PERC_1_MSK			UINT16_C(0x0200)
#define CCS811_ENV_DATA_TEMPERATURE_FRAC_1_2_MSK		UINT16_C(0x0100)
#define CCS811_ENV_DATA_TEMPERATURE_FRAC_1_4_MSK		UINT16_C(0x0080)
#define CCS811_ENV_DATA_TEMPERATURE_FRAC_1_8_MSK		UINT16_C(0x0040)
#define CCS811_ENV_DATA_TEMPERATURE_FRAC_1_16_MSK		UINT16_C(0x0020)
#define CCS811_ENV_DATA_TEMPERATURE_FRAC_1_32_MSK		UINT16_C(0x0010)
#define CCS811_ENV_DATA_TEMPERATURE_FRAC_1_64_MSK		UINT16_C(0x0008)
#define CCS811_ENV_DATA_TEMPERATURE_FRAC_1_128_MSK		UINT16_C(0x0004)
#define CCS811_ENV_DATA_TEMPERATURE_FRAC_1_256_MSK		UINT16_C(0x0002)
#define CCS811_ENV_DATA_TEMPERATURE_FRAC_1_512_MSK		UINT16_C(0x0001)

#define CCS811_FW_BOOT_VERSION_MAJOR_MSK	UINT16_C(0xF000)
#define CCS811_FW_BOOT_VERSION_MINOR_MSK	UINT16_C(0x0F00)
#define CCS811_FW_BOOT_VERSION_TRIVIAL_MSK	UINT16_C(0x00FF)

#define CCS811_FW_APP_VERSION_MAJOR_MSK		UINT16_C(0xF000)
#define CCS811_FW_APP_VERSION_MINOR_MSK		UINT16_C(0x0F00)
#define CCS811_FW_APP_VERSION_TRIVIAL_MSK	UINT16_C(0x00FF)

#define CCS811_ERROR_ID_WRITE_REG_INVALID_MSK	UINT8_C(0x01)
#define CCS811_ERROR_ID_READ_REG_INVALID_MSK 	UINT8_C(0x02)
#define CCS811_ERROR_ID_MEASMODE_INVALID_MSK	UINT8_C(0x04)
#define CCS811_ERROR_ID_MAX_RESISTANCE_MSK	 	UINT8_C(0x08)
#define CCS811_ERROR_ID_HEATER_FAULT_MSK	 	UINT8_C(0x10)
#define CCS811_ERROR_ID_HEATER_SUPPLY_MSK	 	UINT8_C(0x20)

#define CCS811_APP_ERASE_COMPLETE_MSK	UINT8_C(0x40)

#define CCS811_APP_VERIFY_COMPLETE_MSK	UINT8_C(0x20)

#define CCS811_THRESHOLDS_LOW_BYTE0_MSK			UINT16_C(0xFF00)
#define CCS811_THRESHOLDS_LOW_BYTE1_MSK			UINT16_C(0x00FF)
#define CCS811_THRESHOLDS_HIGH_BYTE0_MSK		UINT16_C(0xFF00)
#define CCS811_THRESHOLDS_HIGH_BYTE1_MSK		UINT16_C(0x00FF)

#define CCS811_BASELINE_BYTE0_MSK	UINT16_C(0xFF00)
#define CCS811_BASELINE_BYTE1_MSK	UINT16_C(0x00FF)

#define CCS811_HW_VERSION_HIGH_BYTE_MSK	UINT8_C(0xF0)

/**\name Macros for Byte IDs in multi-byte registers */
#define CCS811_ALG_RESULT_DATA_ECO2_HIGH_BYTE	UINT8_C(0x00)
#define CCS811_ALG_RESULT_DATA_ECO2_LOW_BYTE	UINT8_C(0x01)
#define CCS811_ALG_RESULT_DATA_TVOC_HIGH_BYTE	UINT8_C(0x02)
#define CCS811_ALG_RESULT_DATA_TVOC_LOW_BYTE	UINT8_C(0x03)
#define CCS811_ALG_RESULT_DATA_STATUS_BYTE		UINT8_C(0x04)
#define CCS811_ALG_RESULT_DATA_ERROR_ID_BYTE	UINT8_C(0x05)
#define CCS811_ALG_RESULT_DATA_RAW_DATA_BYTE_0	UINT8_C(0x06)
#define CCS811_ALG_RESULT_DATA_RAW_DATA_BYTE_1	UINT8_C(0x07)

#define CCS811_RAW_DATA_BYTE_0	UINT8_C(0x00)
#define CCS811_RAW_DATA_BYTE_1	UINT8_C(0x01)

#define CCS811_ENV_DATA_HUMIDITY_HIGH_BYTE		UINT8_C(0x00)
#define CCS811_ENV_DATA_HUMIDITY_LOW_BYTE		UINT8_C(0x01)
#define CCS811_ENV_DATA_TEMPERATURE_HIGH_BYTE	UINT8_C(0x02)
#define CCS811_ENV_DATA_TEMPERATURE_LOW_BYTE	UINT8_C(0x03)

#define CCS811_NTC_VREF_MV_HIGH_BYTE	UINT8_C(0x00)
#define CCS811_NTC_VREF_MV_LOW_BYTE		UINT8_C(0x01)
#define CCS811_NTC_VNTC_MV_HIGH_BYTE	UINT8_C(0x02)
#define CCS811_NTC_VNTC_MV_LOW_BYTE		UINT8_C(0x03)

#define CCS811_THRESHOLDS_LOW_MED_HIGH_BYTE		UINT8_C(0x00)
#define CCS811_THRESHOLDS_LOW_MED_LOW_BYTE		UINT8_C(0x01)
#define CCS811_THRESHOLDS_MED_HIGH_HIGH_BYTE	UINT8_C(0x02)
#define CCS811_THRESHOLDS_MED_HIGH_LOW_BYTE		UINT8_C(0x03)
#define CCS811_THRESHOLDS_HYSTERESIS_BYTE		UINT8_C(0x04)

#define CCS811_FW_BOOT_VERSION_MAJOR_MINOR_BYTE	UINT8_C(0x00)
#define CCS811_FW_BOOT_VERSION_TRIVIAL_BYTE		UINT8_C(0x01)

#define CCS811_FW_APP_VERSION_MAJOR_MINOR_BYTE	UINT8_C(0x00)
#define CCS811_FW_APP_VERSION_TRIVIAL_BYTE		UINT8_C(0x01)

/**\name Macros to represent constants */
#define CCS811_HUMIDITY_DEFAULT_VALUE				UINT16_C(0x6400)	// 50%
#define CCS811_TEMPERATURE_DEFAULT_VALUE			UINT16_C(0x6400)	// 25 deg C

#define CCS811_THRESHOLD_LOW_MED_DEFAULT_VALUE		UINT16_C(0x05DC)	// 1500ppm
#define CCS811_THRESHOLD_MED_HIGH_DEFAULT_VALUE		UINT16_C(0x09C4)	// 2500ppm
#define CCS811_THRESHOLD_HYSTERESIS_DEFAULT_VALUE	UINT8_C(0x32)		// 50

#define CCS811_APP_ERASE_RESET_DEFAULT_VALUE_BYTE_0	UINT8_C(0xE7)
#define CCS811_APP_ERASE_RESET_DEFAULT_VALUE_BYTE_1	UINT8_C(0xA7)
#define CCS811_APP_ERASE_RESET_DEFAULT_VALUE_BYTE_2	UINT8_C(0xE6)
#define CCS811_APP_ERASE_RESET_DEFAULT_VALUE_BYTE_3	UINT8_C(0x09)

#define CCS811_SW_RESET_RESET_DEFAULT_VALUE_BYTE_0	UINT8_C(0x11)
#define CCS811_SW_RESET_RESET_DEFAULT_VALUE_BYTE_1	UINT8_C(0xE5)
#define CCS811_SW_RESET_RESET_DEFAULT_VALUE_BYTE_2	UINT8_C(0x72)
#define CCS811_SW_RESET_RESET_DEFAULT_VALUE_BYTE_3	UINT8_C(0x8A)

#define CCS811_APP_START_BOOT_TO_RUN_DEFAULT_VALUE	UINT8_C(0xF4)

#define CCS811_HW_VERSION_TOP_4_BITS	UINT8_C(0x10)

/*!
 *	@brief Type definitions
 */
typedef int8_t (*ccs811_com_fptr_t)(uint8_t ccs811_slave_address, uint8_t reg_addr, uint8_t *data, uint16_t data_len);
typedef void (*ccs811_delay_fptr_t)(uint32_t period);

/*!
 *	@brief Algorithm Result Data Struct
 *	eco2 - Bytes 0:1
 *	TVOC - Bytes 2:3
 *	Status - Byte 4
 *	ERROR ID - Byte 5
 *	Raw Data - Byte 6:7
 */
struct ccs811_measurement_data {
	uint16_t eco2;
	uint16_t tvoc;
	uint8_t status;
	uint8_t error_id;
	uint16_t raw_data;
};

/*!
 *	@brief Threshold Data Struct
 *	Low Threshold - Bytes 0:1
 *	High Threshold - Bytes 2:3
 *	Hysteresis - Byte 4
 */
struct ccs811_threshold_reg {
	uint16_t thresh_low;
	uint16_t thresh_high;
	uint8_t thresh_hyst;
};

/*!
 *	@brief Raw Data Struct
 *	Current - Byte 0, [7:2]
 *	ADC - Byte 0 [1:0], Byte 1 [7:0]
 */
struct ccs811_raw_data {
	uint8_t current_through_sensor;
	uint16_t raw_adc_reading;
};

/*!
 *	@brief Environmental Data Struct
 *	Humidity Percent - Byte 0 [7:1]
 *	Humidity Fraction - Byte 0 [0:0], Byte 1 [7:0]
 *	Temperature 25*C - Byte 2 [7:1]
 *	Temperature 25*C Fraction - Byte 2 [0:0], Byte 3 [7:0]
 */
struct ccs811_env_data {
	uint8_t humidity_perc;
	uint16_t humidity_frac;
	uint8_t temperature_perc;
	uint16_t temperature_frac;
};

/*!
 * 	@brief NTC Data Register Struct
 *	v_ref_mv - Voltage across R_ref in mV - Byte 0 [7:0], Byte 1 [7:0]
 *	v_ntc_mv - Voltage across R_ntc in mV - Byte 2 [7:0], Byte 3 [7:0]
 */
struct ccs811_ntc {
	uint16_t v_ref_mv;
	uint16_t v_ntc_mv;
};

/*!
 *	@brief Firmware Boot Version Struct
 *	major - Byte 0 [7:4]
 *	minor - Byte 0 [3:0]
 *	trivial - Byte 1 [7:0]
 */
struct ccs811_fw_boot_version {
	uint8_t major;
	uint8_t minor;
	uint8_t trivial;
};

/*!
 *	@brief Firmware App Version Struct
 *	major - Byte 0 [7:4]
 *	minor - Byte 0 [3:0]
 *	trivial - Byte 1 [7:0]
 */
struct ccs811_fw_app_version {
	uint8_t major;
	uint8_t minor;
	uint8_t trivial;
};

/*!
 *	@brief CCS811 Device Struct
 */
struct ccs811_dev{
	/*! Hardware ID */
	uint8_t hw_id;

	/*! Hardware ID */
	uint8_t hw_version;

	/*! Device Address */
	uint8_t dev_addr;

	/*! Status Reg */
	uint8_t status_reg;

	/*! Measure Mode Reg */
	uint8_t measure_mode_reg;

	/*! Error Reg */
	uint8_t error_reg;

	/*! Read function pointer */
	ccs811_com_fptr_t read;

	/*! Write function pointer */
	ccs811_com_fptr_t write;

	/*! Delay function pointer */
	ccs811_delay_fptr_t delay_ms;
};

#endif /* CCS811_CCS811_DEFS_H_ */
