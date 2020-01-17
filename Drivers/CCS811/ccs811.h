/*
 * ccs811.h
 *
 *  Created on: Nov 22, 2019
 *      Author: Anthony Mendez
 */

#ifndef CCS811_CCS811_H_
#define CCS811_CCS811_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/* Header Includes */
#include "ccs811_defs.h"

/*!
 *	@brief This API is the entry point.
 *	It reads the hardware-id and triggers the sensor to go from
 *	boot to application mode.
 *
 *	@param[in,out] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_init(struct ccs811_dev *dev);

/*!
 *	@brief This API writes the given data to the given register address of the sensor.
 *
 *	@param[in] reg_addr : Register address from where the data is to be written.
 *	@param[in] reg_data : Pointer to data buffer which is to be written in the sensor.
 *	@param[in] len : Number of bytes of data to write.
 *	@param[in] dev : Pointer to a structure instance of ccs811_dev.
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct ccs811_dev *dev);

/*!
 *	@brief This API reads the data to the given register address of the sensor.
 *
 *	@param[in] reg_addr : Register address where the data is to be read.
 *	@param[in] reg_data : Pointer to data buffer to store the read data.
 *	@param[in] len : Number of bytes of data to read.
 *	@param[in] dev : Pointer to a structure instance of ccs811_dev.
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_regs(uint8_t *reg_addr, uint8_t *reg_data, uint8_t len, const struct ccs811_dev *dev);

/*!
 *	@brief This API reads the status register.
 *
 *	@param[in,out] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_status_reg(struct ccs811_dev *dev);

/*!
 *	@brief This API reads the measurement mode and conditions register.
 *
 *	@param[in,out] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_meas_mode_reg(struct ccs811_dev *dev);

/*!
 *	@brief This API writes to the measurement mode and conditions register.
 *
 *	@param[in,out] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_set_meas_mode_reg(struct ccs811_dev *dev);

/*!
 *	@brief This API reads from the algorithm result data.
 *
 *	@param[out] meas_data : Structure instance of algorithm result
 *	@param[in] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_alg_result_data(struct ccs811_measurement_data *meas_data, struct ccs811_dev *dev);

/*!
 *	@brief This API reads from the raw data.
 *
 *	@param[out] raw_data : Structure instance for raw data.
 *	@param[in] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_raw_data(struct ccs811_raw_data *raw_data, struct ccs811_dev *dev);

/*!
 *	@brief This API writes to the environmental data register.
 *
 *	@param[in] env_data : Structure instance for the environmental data.
 *	@param[in] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_set_env_data(struct ccs811_env_data *env_data, struct ccs811_dev *dev);

/*!
 *	@brief This API reads from the NTC register.
 *
 *	@param[out] raw_data : Structure instance for NTC data.
 *	@param[in] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_ntc(struct ccs811_ntc *ntc, struct ccs811_dev *dev);

/*!
 *	@brief This API writes to the thresholds register.
 *
 *	@param[in] threshold_reg : Structure instance for the thresholds registers.
 *	@param[in] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_set_threshold_reg(struct ccs811_threshold_reg *threshold_reg, struct ccs811_dev *dev);

/*!
 *	@brief This API reads the baseline register.
 *
 *	@param[out] baseline : Encoded baseline value
 *	@param[in] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_baseline_reg(uint16_t *baseline, struct ccs811_dev *dev);

/*!
 *	@brief This API writes to the baseline register.
 *
 *	@param[in] baseline : Encoded baseline value
 *	@param[in] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_set_baseline_reg(uint16_t *baseline, struct ccs811_dev *dev);

/*!
 *	@brief This API reads from the Hardware ID register.
 *
 *	@param[in,out] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_hw_id(struct ccs811_dev *dev);

/*!
 *	@brief This API reads from the Hardware Version register.
 *
 *	@param[out] hw_version : Hardware Version.
 *	@param[in] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_hw_version(struct ccs811_dev *dev);

/*!
 *	@brief This API reads from the Firmware Boot Version register.
 *
 *	@param[out] fw_boot_version : Firmware Boot Version.
 *	@param[in] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_fw_boot_version(struct ccs811_fw_boot_version *fw_boot_version, struct ccs811_dev *dev);

/*!
 *	@brief This API reads from the Firmware App Version register.
 *
 *	@param[out] fw_app_version : Firmware App Version.
 *	@param[in] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status.
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_fw_app_version(struct ccs811_fw_app_version *fw_app_version, struct ccs811_dev *dev);

/*!
 *	@brief This API reads the error id register.
 *
 *	@param[in,out] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_read_error_id(struct ccs811_dev *dev);

/*!
 *	@brief This API sends the software reset code to SW_RESET.
 *
 *	@param[in,out] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_perform_sw_reset(struct ccs811_dev *dev);

/*!
 *	@brief This API sends the app reset code to APP_ERASE.
 *
 *	@param[in,out] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_perform_app_erase(struct ccs811_dev *dev);

/*!
 *	@brief This API transmits flash code to APP_DATA.
 *	app_data must be 9 bytes long.
 *
 *	@param[in,out] dev : Structure instance of ccs811_dev
 *
 *	@return Result of API execution status
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_set_app_data(uint8_t *app_data, struct ccs811_dev *dev);

/*!
 * 	@brief This API transitions the sensor from boot mode to application mode,
 * 	by writing to the APP_Start register.
 *
 * 	@param[in] dev : Structure instance of ccs811_dev
 *
 * 	@return Result of API execution status
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_app_start(struct ccs811_dev *dev);

/*!
 *	@brief This API sets the env_data struct given a 32 bit int rel_humidity,
 *	and relative humidity 32 bit int.
 *
 *	@param[in] rel_humidity : Relative Humidity
 *	@param[in] temperature : Temperature in Celsius
 *	@param[out] env_data : Structure instance of CCS811 environmental data
 *
 * 	@return Result of API execution status
 *	@retval zero -> Success / + value -> Warning / - value -> Error
 */
int8_t ccs811_convert_temp_and_humid_to_env_data(uint32_t rel_humidity, int32_t temperature, struct ccs811_env_data *env_data);

#endif /* CCS811_CCS811_H_ */
