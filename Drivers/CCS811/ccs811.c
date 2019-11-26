/*
 * ccs811.c
 *
 *  Created on: Nov 22, 2019
 *      Author: Anthony Mendez
 */

#include "ccs811.h"

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of ccs811_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t null_ptr_check(const struct ccs811_dev *dev);

/*!
 *	@brief This API is the entry point.
 *	It reads the hardware-id and triggers the sensor to go from
 *	boot to application mode.
 */
int8_t ccs811_init(struct ccs811_dev *dev)
{
	int8_t rslt = CCS811_OK;

	/* R/W Try Count */
	uint8_t try_count = 5;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Try to read Hardware ID of Sensor */
	while (try_count)
	{
		/* Read the hardware-id of the CCS811 sensor */
		rslt = ccs811_read_hw_id(dev);

		if ((rslt == CCS811_OK) && (dev->hw_id == CCS811_HW_ID))
		{
			break;
		}

		/* Try Again */
		try_count--;

		/* Out of tries */
		if (!try_count)
		{
			return rslt;
		}

		/* Wait for 1 ms */
		dev->delay_ms(1);
	}

	try_count = 5;

	/* Try to read Hardware version of the Sensor */
	while (try_count)
	{
		/* Read the status register of the CCS811 sensor*/
		rslt = ccs811_read_hw_version(dev);

		if ((rslt == CCS811_OK) && (dev->status_reg == CCS811_HW_ID))
		{
			break;
		}

		/* Try Again */
		try_count--;

		/* Out of tries */
		if (!try_count)
		{
			return rslt;
		}

		/* Wait for 1 ms */
		dev->delay_ms(1);
	}

	try_count = 5;

	/* Try to read Status Reg of Sensor */
	while (try_count)
	{
		/* Read the status register of the CCS811 sensor*/
		rslt = ccs811_read_status_reg(dev);

		if ((rslt == CCS811_OK) && (dev->status_reg == CCS811_HW_ID))
		{
			break;
		}

		/* Try Again */
		try_count--;

		/* Out of tries */
		if (!try_count)
		{
			return rslt;
		}

		/* Wait for 1 ms */
		dev->delay_ms(1);
	}

	try_count = 5;

	/* Try to write measure mode Reg of Sensor */
	while (try_count)
	{
		/* Write to the measurement mode register */
		rslt = ccs811_set_meas_mode_reg(dev);

		if ((rslt == CCS811_OK))
		{
			break;
		}

		/* Try Again */
		try_count--;

		/* Out of tries */
		if (!try_count)
		{
			return rslt;
		}
	}

	try_count = 5;

	/* Trigger Sensor to go into application mode */
	while(try_count)
	{
		/* Write to the measurement mode register */
		rslt = ccs811_app_start(dev);

		if ((rslt == CCS811_OK))
		{
			break;
		}

		/* Try Again */
		try_count--;

		/* Out of tries */
		if (!try_count)
		{
			return rslt;
		}

		/* Wait for 1 ms */
		dev->delay_ms(1);
	}

	return rslt;
}

/*!
 *	@brief This API writes the given data to the given register address of the sensor.
 */
int8_t ccs811_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct ccs811_dev *dev)
{
	int8_t rslt;
	uint8_t temp_buff[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	/* Check for invalid length */
	if (len == 0)
	{
		rslt = CCS811_E_INVALID_LEN;
	}

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Write to registers */
	rslt = dev->write(dev->dev_addr, reg_addr[0], temp_buff, len);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 *	@brief This API reads the data to the given register address of the sensor.
 */
int8_t ccs811_read_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct ccs811_dev *dev)
{
	int8_t rslt;
	uint8_t temp_buff[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	/* Check for invalid length */
	if (len == 0)
	{
		rslt = CCS811_E_INVALID_LEN;
	}

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read from registers */
	rslt = dev->read(dev->dev_addr, reg_addr[0], temp_buff, len);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 *	@brief This API reads the status register.
 */
int8_t ccs811_read_status_reg(struct ccs811_dev *dev)
{
	int8_t rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_STATUS_ADDR);
	uint8_t len = (uint8_t)(CCS811_STATUS_BYTE_LEN);
	uint8_t temp_buff[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read from status register */
	rslt = ccs811_read_regs(&slave_addr, temp_buff, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 *	@brief This API reads the measurement mode and conditions register.
 */
int8_t ccs811_read_meas_mode_reg(struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_MEAS_MODE_ADDR);
	uint8_t len = (uint8_t)(CCS811_MEAS_MODE_BYTE_LEN);
	uint8_t temp_buff[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read from measure mode register */
	rslt = ccs811_read_regs(&slave_addr, temp_buff, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 *	@brief This API writes to the measurement mode and conditions register.
 */
int8_t ccs811_set_meas_mode_reg(struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_MEAS_MODE_ADDR);
	uint8_t len = (uint8_t)(CCS811_MEAS_MODE_BYTE_LEN);

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Set measure mode register from device */
	rslt = ccs811_set_regs(&slave_addr, &(dev->measure_mode_reg), len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 *	@brief This API reads from the algorithm result data.
 */
int8_t ccs811_read_alg_result_data(struct ccs811_measurement_data *meas_data, struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_ALG_RESULT_DATA_ADDR);
	uint8_t len = (uint8_t)(CCS811_ALG_RESULT_DATA_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read from algorithm results register into temp buffer */
	rslt = ccs811_read_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
		return rslt;
	}

	/* Modify measurement data struct */
	meas_data->eco2 = (temp_buffer[0] << 8) | temp_buffer[1];
	meas_data->tvoc = (temp_buffer[2] << 8) | temp_buffer[3];
	meas_data->status = temp_buffer[4];
	meas_data->error_id = temp_buffer[5];
	meas_data->raw_data = (temp_buffer[6] << 8) | temp_buffer[7];

	return rslt;
}

/*!
 *	@brief This API reads from the raw data.
 */
int8_t ccs811_read_raw_data(struct ccs811_raw_data *raw_data, struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_RAW_DATA_ADDR);
	uint8_t len = (uint8_t)(CCS811_RAW_DATA_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read from raw data results register */
	rslt = ccs811_read_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
		return rslt;
	}

	/* Create full raw data integer */
	uint16_t raw_data_full = (temp_buffer[0] << 8) | temp_buffer[1];
	/* Modify raw data struct */
	raw_data->current_through_sensor =  (raw_data_full & CCS811_RAW_DATA_CURRENT_MSK) >> 10;
	raw_data->raw_adc_reading = CCS811_RAW_DATA_RAW_ADC_MSK & raw_data_full;

	return rslt;
}

/*!
 *	@brief This API writes to the environmental data register.
 */
int8_t ccs811_set_env_data(struct ccs811_env_data *env_data, struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_ENV_DATA_ADDR);
	uint8_t len = (uint8_t)(CCS811_ENV_DATA_BYTE_LEN);
	uint8_t temp_buffer[len];

	temp_buffer[0] = (env_data->humidity_perc << 2) | ((env_data->humidity_frac & 0x300) >> 8);
	temp_buffer[1] = (env_data->humidity_frac & 0x0FF);
	temp_buffer[2] = (env_data->temperature_perc << 2) | ((env_data->temperature_frac & 0x300) >> 8);
	temp_buffer[3] = (env_data->temperature_frac & 0x0FF);


	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Set environmental data register from device */
	rslt = ccs811_set_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 *	@brief This API reads from the NTC register.
 */
int8_t ccs811_read_ntc(struct ccs811_ntc *ntc, struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_NTC_ADDR);
	uint8_t len = (uint8_t)(CCS811_NTC_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read from the ntc results register */
	rslt = ccs811_read_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
		return rslt;
	}

	ntc->v_ref_mv = (temp_buffer[0] << 8) | temp_buffer[1];
	ntc->v_ntc_mv = (temp_buffer[2] << 8) | temp_buffer[3];

	return rslt;
}

/*!
 *	@brief This API writes to the thresholds register.
 */
int8_t ccs811_set_threshold_reg(struct ccs811_threshold_reg *threshold_reg, struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_THRESHOLDS_ADDR);
	uint8_t len = (uint8_t)(CCS811_THRESHOLDS_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	temp_buffer[0] = (threshold_reg->thresh_low & CCS811_THRESHOLDS_LOW_BYTE0_MSK) >> 8;
	temp_buffer[1] = (threshold_reg->thresh_low & CCS811_THRESHOLDS_LOW_BYTE1_MSK);
	temp_buffer[2] = (threshold_reg->thresh_high & CCS811_THRESHOLDS_HIGH_BYTE0_MSK) >> 8;
	temp_buffer[3] = (threshold_reg->thresh_high & CCS811_THRESHOLDS_HIGH_BYTE1_MSK);
	temp_buffer[4] = threshold_reg->thresh_hyst;

	/* Set threshold register from device */
	rslt = ccs811_set_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 *	@brief This API reads the baseline register.
 */
int8_t ccs811_read_baseline_reg(uint16_t *baseline, struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_BASELINE_ADDR);
	uint8_t len = (uint8_t)(CCS811_BASELINE_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read baseline register from the device */
	rslt = ccs811_read_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	*baseline = (temp_buffer[0] << 8) | temp_buffer[1];

	return rslt;
}

/*!
 *	@brief This API writes to the baseline register.
 */
int8_t ccs811_set_baseline_reg(uint16_t *baseline, struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_BASELINE_ADDR);
	uint8_t len = (uint8_t)(CCS811_BASELINE_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	temp_buffer[0] = (*baseline & CCS811_BASELINE_BYTE0_MSK) >> 8;
	temp_buffer[1] = (*baseline & CCS811_BASELINE_BYTE1_MSK);

	/* Set baseline register of device */
	rslt = ccs811_set_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 *	@brief This API reads from the Hardware ID register.
 */
int8_t ccs811_read_hw_id(struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_HW_ID_ADDR);
	uint8_t len = (uint8_t)(CCS811_HW_ID_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read hardware register from the device */
	rslt = ccs811_read_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
		return rslt;
	}

	/* Check if Hardware ID matches what we should have */
	if (temp_buffer[0] != CCS811_HW_ID)
	{
		rslt = CCS811_E_NVM_COPY_FAILED;
		return rslt;
	}

	dev->hw_id = temp_buffer[0];

	return rslt;
}

/*!
 *	@brief This API reads from the Hardware Version register.
 */
int8_t ccs811_read_hw_version(struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_HW_VERSION_ADDR);
	uint8_t len = (uint8_t)(CCS811_HW_VERSION_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read hardware version from the device */
	rslt = ccs811_read_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
		return rslt;
	}

	/* Check if Hardware Version matches what we should have */
	if ((uint8_t)(temp_buffer[0] & CCS811_HW_VERSION_HIGH_BYTE_MSK) !=
			(uint8_t)(CCS811_HW_VERSION_HW_VERSION_TOP_4_BITS))
	{
		rslt = CCS811_E_NVM_COPY_FAILED;
		return rslt;
	}

	dev->hw_version = temp_buffer[0];

	return rslt;
}

/*!
 *	@brief This API reads from the Firmware Boot Version register.
 */
int8_t ccs811_read_fw_boot_version(struct ccs811_fw_boot_version *fw_boot_version, struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_FW_BOOT_VERSION_ADDR);
	uint8_t len = (uint8_t)(CCS811_FW_BOOT_VERSION_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read Firmware Boot Version from the device */
	rslt = ccs811_read_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
		return rslt;
	}

	/* Set Firmware Boot Version data into struct */
	uint16_t fw_boot_version_all = (temp_buffer[0] << 8) | temp_buffer[1];
	fw_boot_version->major = (fw_boot_version_all & CCS811_FW_BOOT_VERSION_MAJOR_MSK) >> 12;
	fw_boot_version->minor = (fw_boot_version_all & CCS811_FW_BOOT_VERSION_MINOR_MSK) >> 8;
	fw_boot_version->trivial = (fw_boot_version_all & CCS811_FW_BOOT_VERSION_TRIVIAL_MSK);

	return rslt;
}

/*!
 *	@brief This API reads from the Firmware App Version register.
 */
int8_t ccs811_read_fw_app_version(struct ccs811_fw_app_version *fw_app_version, struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_FW_APP_VERSION_ADDR);
	uint8_t len = (uint8_t)(CCS811_FW_APP_VERSION_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read Firmware App Version from the device */
	rslt = ccs811_read_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
		return rslt;
	}

	/* Set Firmware App Version data into struct */
	uint16_t fw_app_version_all = (temp_buffer[0] << 8) | temp_buffer[1];
	fw_app_version->major = (fw_app_version_all & CCS811_FW_APP_VERSION_MAJOR_MSK) >> 12;
	fw_app_version->minor = (fw_app_version_all & CCS811_FW_APP_VERSION_MINOR_MSK) >> 8;
	fw_app_version->trivial = (fw_app_version_all & CCS811_FW_APP_VERSION_TRIVIAL_MSK);

	return rslt;
}

/*!
 *	@brief This API reads the error id register.
 */
int8_t ccs811_read_error_id(struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_ERROR_ID_ADDR);
	uint8_t len = (uint8_t)(CCS811_ERROR_ID_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read error id from the device */
	rslt = ccs811_read_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
		return rslt;
	}

	dev->error_reg = temp_buffer[0];

	return rslt;
}

/*!
 *	@brief This API sends the software reset code to SW_RESET.
 */
int8_t ccs811_perform_sw_reset(struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_SW_RESET_ADDR);
	uint8_t len = (uint8_t)(CCS811_SW_RESET_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	temp_buffer[0] = CCS811_SW_RESET_RESET_DEFAULT_VALUE_BYTE_0;
	temp_buffer[1] = CCS811_SW_RESET_RESET_DEFAULT_VALUE_BYTE_1;
	temp_buffer[2] = CCS811_SW_RESET_RESET_DEFAULT_VALUE_BYTE_2;
	temp_buffer[3] = CCS811_SW_RESET_RESET_DEFAULT_VALUE_BYTE_3;

	/* Set sw reset of device */
	rslt = ccs811_set_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 *	@brief This API sends the app reset code to APP_ERASE.
 */
int8_t ccs811_perform_app_erase(struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_APP_ERASE_ADDR);
	uint8_t len = (uint8_t)(CCS811_APP_ERASE_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	temp_buffer[0] = CCS811_APP_ERASE_RESET_DEFAULT_VALUE_BYTE_0;
	temp_buffer[1] = CCS811_APP_ERASE_RESET_DEFAULT_VALUE_BYTE_1;
	temp_buffer[2] = CCS811_APP_ERASE_RESET_DEFAULT_VALUE_BYTE_2;
	temp_buffer[3] = CCS811_APP_ERASE_RESET_DEFAULT_VALUE_BYTE_3;

	/* Set app erase of device */
	rslt = ccs811_set_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 *	@brief This API transmits flash code to APP_DATA.
 *	app_data must be 9 bytes long.
 */
int8_t ccs811_set_app_data(uint8_t *app_data, struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_APP_DATA_ADDR);
	uint8_t len = (uint8_t)(CCS811_APP_DATA_BYTE_LEN);

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Set app data of device */
	rslt = ccs811_set_regs(&slave_addr, app_data, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 * 	@brief This API transitions the sensor from boot mode to application mode,
 * 	by writing to the APP_Start register.
 */
int8_t ccs811_app_start(struct ccs811_dev *dev)
{
	int rslt;
	uint8_t slave_addr = (uint8_t)(CCS811_APP_START_ADDR);
	uint8_t len = (uint8_t)(CCS811_APP_START_BYTE_LEN);
	uint8_t temp_buffer[len];

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Set empty byte */
	temp_buffer[0] = 0;

	/* Set app start of device */
	rslt = ccs811_set_regs(&slave_addr, temp_buffer, len, dev);

	/* Check for communication error */
	if (rslt != CCS811_OK)
	{
		rslt = CCS811_E_COMM_FAIL;
	}

	return rslt;
}

/* Static Functions */

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct ccs811_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = CCS811_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = CCS811_OK;
    }

    return rslt;
}
