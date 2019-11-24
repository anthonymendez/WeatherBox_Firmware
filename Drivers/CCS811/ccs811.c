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
	int8_t rslt;

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
	uint8_t temp_buff;

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read from status register */
	rslt = ccs811_read_regs(CCS811_STATUS_ADDR, &temp_buff, CCS811_STATUS_BYTE_LEN, dev);

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
	uint8_t temp_buff;
	uint8_t slave_addr = (uint8_t)(CCS811_MEAS_MODE_ADDR);
	uint8_t len = (uint8_t)(CCS811_MEAS_MODE_BYTE_LEN);

	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Read from measure mode register */
	rslt = ccs811_read_regs(&slave_addr, &temp_buff, len, dev);

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
	uint8_t temp_buffer[CCS811_ALG_RESULT_DATA_BYTE_LEN];
	uint8_t slave_addr = (uint8_t)(CCS811_ALG_RESULT_DATA_ADDR);
	uint8_t len = (uint8_t)(CCS811_ALG_RESULT_DATA_BYTE_LEN);

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
	uint8_t temp_buffer[CCS811_RAW_DATA_BYTE_LEN];
	uint8_t slave_addr = (uint8_t)(CCS811_RAW_DATA_ADDR);
	uint8_t len = (uint8_t)(CCS811_RAW_DATA_BYTE_LEN);

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

	/* Modify raw data struct */
	raw_data->current_through_sensor = CCS811_RAW_DATA_CURRENT_MSK & temp_buffer[0];
	raw_data->raw_adc_reading = CCS811_RAW_DATA_RAW_ADC_MSK & ((temp_buffer[0] << 8) | temp_buffer[1]);

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
	temp_buffer[0] = (env_data->humidity_perc << 2) | ((env_data->humiditiy_frac & 0x300) >> 8);
	temp_buffer[1] = (env_data->humidity_frac & 0x0FF);
	temp_buffer[2] = (env_data->temperature_perc << 2) | ((env_data->temperature_frac & 0x300) >> 8);
	temp_buffer[3] = (env_data->temperature_frac & 0x0FF);


	/* Check for null pointers in device structure */
	rslt = null_ptr_check(dev);

	if (rslt != CCS811_OK)
	{
		return rslt;
	}

	/* Set measure mode register from device */
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
