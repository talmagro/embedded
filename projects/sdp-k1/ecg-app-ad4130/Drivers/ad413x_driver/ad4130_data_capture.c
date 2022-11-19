/***************************************************************************//**
 *   @file    ad4130_data_capture.c
 *   @brief   AD4130 data capture interface for IIO based applications
 *   @details This module handles the ADC data capturing for IIO client
********************************************************************************
 * Copyright (c) 2021-22 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <string.h>

#include "app_config.h"
#include "ad413x.h"
#include "ad4130_support.h"
#include "ad4130_iio.h"
#include "ad4130_regs.h"
#include "ad4130_data_capture.h"
#include "ad4130_user_config.h"
#include "no_os_gpio.h"
#include "no_os_error.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Timeout count to avoid stuck into potential infinite loop while checking
 * for new data into an acquisition buffer. The actual timeout factor is determined
 * through 'sampling_frequency' attribute of IIO app, but this period here makes sure
 * we are not stuck into a forever loop in case data capture is interrupted
 * or failed in between.
 * Note: This timeout factor is dependent upon the MCU clock frequency. Below timeout
 * is tested for SDP-K1 platform @180Mhz default core clock */
#define BUF_READ_TIMEOUT	0xffffffff

/* Fifo depth limit (watermark count) for data capture */
#define FIFO_SIZE		256		// Range: 1-256

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

/* ADC data buffer */
#if !defined(USE_SDRAM_CAPTURE_BUFFER)
int8_t adc_data_buffer[DATA_BUFFER_SIZE] = { 0 };
#endif

/*
 *@enum		acq_buffer_state_e
 *@details	Enum holding the data acquisition buffer states
 **/
typedef enum {
	BUF_AVAILABLE,
	BUF_EMPTY,
	BUF_FULL
} acq_buffer_state_e;

/*
 *@struct	acq_buf_t
 *@details	Structure holding the data acquisition buffer parameters
 **/
typedef struct {
	acq_buffer_state_e state;	// Buffer state
	uint32_t wr_indx;			// Buffer write index (incremented per sample read)
	uint32_t rd_indx;			// Buffer read index (incremented per sample read)
	int8_t *wr_pdata;			// Data buffer write pointer
	int8_t *rd_pdata;			// Data buffer read pointer
	bool reindex_buffer;		// Reindex buffer to 0th channel
} acq_buf_t;

/* ADC data acquisition buffers */
static volatile acq_buf_t acq_buffer;

/* Flag to indicate data capture status */
static volatile bool start_cont_data_capture = false;

/* Count to track number of actual samples requested by IIO client */
static volatile uint32_t num_of_requested_samples = 0;

/* ADC sample/raw data size in bytes */
static volatile uint8_t sample_size_in_bytes;

/* Max available buffer size (after considering the data alignment with IIO buffer) */
static volatile uint32_t max_buffer_sz;

/* List of channels to be captured */
static volatile uint8_t active_channels[ADC_USER_CHANNELS];

/* Number of active channels */
static volatile uint8_t num_of_active_channels;

/* Current active channel index */
static volatile uint8_t chn_indx;

/* FIFO data capture flags */
static volatile bool start_fifo_mode_data_capture = false;
static volatile bool fifo_data_available = false;
static uint32_t fifo_data[FIFO_SIZE];

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/*!
 * @brief	Function to init the data capture for AD4130 device
 * @return	0 in case of success, negative error code otherwise
 */
int32_t ad4130_data_capture_init(void)
{
	int32_t ret;
	uint8_t preset;
	adc_conv_int_source_e conv_int_source;

	/* Stop any previous conversion */
	ret = ad413x_set_adc_mode(ad4130_dev_inst, AD413X_STANDBY_MODE);
	if (ret) {
		return ret;
	}

	/* Select and enable the interupt pin source for data conversion monitor */
#if defined(AD4130_WLCSP_PACKAGE_TYPE)
	conv_int_source = INT_PIN;
#else
	conv_int_source = CLK_PIN;
#endif

	ret = ad413x_set_int_source(ad4130_dev_inst, conv_int_source);
	if (ret) {
		return ret;
	}

	/* Set the filter FS value (same for all setups/channels for
	 * consistant ODR/sample rate) */
	for (preset = 0; preset <= ADC_PRESETS; preset++) {
		ret = ad413x_set_filter_fs(ad4130_dev_inst, AD4130_FS_CONFIG, preset);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

/*!
 * @brief	Store the list of all previously enabled channels and enable
 *			new channels set in the channel mask argument
 * @param	chn_mask[in] - Active channels list
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t adc_store_active_chns(uint32_t chn_mask)
{
	uint8_t mask = 0x1;
	uint8_t index = 0;
	uint8_t chn;
	int32_t ret;

	/* Enable/Disable channels based on channel mask set in the IIO client */
	for (chn = 0; chn < ADC_USER_CHANNELS; chn++) {
		if (chn_mask & mask) {
			/* Store the active channel */
			active_channels[index++] = chn;
			num_of_active_channels++;

			/* Enable the selected channel */
			ret = ad413x_ch_en(ad4130_dev_inst, chn, 1);
		} else {
			/* Disable the selected channel */
			ret = ad413x_ch_en(ad4130_dev_inst, chn, 0);
		}

		if (ret) {
			return ret;
		}

		mask <<= 1;
	}

	return 0;
}

/*!
 * @brief	Trigger a data capture in continuous/burst mode
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t adc_start_data_capture(void)
{
	int32_t ret;

	/* Stop any previous conversion */
	ret = ad413x_set_adc_mode(ad4130_dev_inst, AD413X_STANDBY_MODE);
	if (ret) {
		return ret;
	}

	/* Trigger new conversion */
	ret = ad413x_set_adc_mode(ad4130_dev_inst, AD413X_CONTINOUS_CONV_MODE);
	if (ret) {
		return ret;
	}

	return 0;
}

/*!
 * @brief	Stop a data capture from continuous/burst/fifo mode
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t adc_stop_data_capture(void)
{
	/* Stop any active conversion */
	return ad413x_set_adc_mode(ad4130_dev_inst, AD413X_STANDBY_MODE);
}

/*!
 * @brief	Trigger a data capture in FIFO mode
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t adc_start_fifo_data_capture(void)
{
	int32_t ret;
	uint32_t fifo_control_reg_val;

	/* Read FIFO control register */
	ret = ad413x_reg_read(ad4130_dev_inst, AD413X_REG_FIFO_CTRL,
			      &fifo_control_reg_val);
	if (ret) {
		return ret;
	}

	/* Store the watermark count in FIFO */
	fifo_control_reg_val = (fifo_control_reg_val & ~AD413X_WATERMARK_MSK) |
			       AD413X_WATERMARK(FIFO_SIZE);

	/* Select the FIFO mode to enable FIFO and enable watermark interrupt */
	fifo_control_reg_val = (fifo_control_reg_val & ~AD4130_FIFO_MODE_MSK) |
			       AD413X_FIFO_MODE(FIFO_OLDEST_SAVE_MODE) |
			       AD413X_WATERMARK_INT_EN;

	/* Disable the FIFO header and status (FIFO status and header is not appended to data) */
	fifo_control_reg_val &= ~(AD413X_ADD_FIFO_HEADER | AD413X_ADD_FIFO_STATUS);

	/* Write to ADC fifo_ctrl register */
	ret = ad413x_reg_write(ad4130_dev_inst, AD413X_REG_FIFO_CTRL,
			       fifo_control_reg_val);
	if (ret) {
		return ret;
	}

	start_fifo_mode_data_capture = true;
	ret = adc_start_data_capture();
	if (ret) {
		return ret;
	}

	return 0;
}

/*!
 * @brief	Read a single sample of ADC
 * @param	adc_raw[in] - Pointer to ADC raw data variable
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t adc_read_single_sample(uint32_t *adc_raw)
{
	if (!adc_raw) {
		return -EINVAL;
	}

	return ad413x_mon_conv_and_read_data(ad4130_dev_inst, adc_raw);
}

/*!
 * @brief	Read a single sample of ADC
 * @param	data[in] - Pointer to FIFO data array
 * @param	samples[in] - Number of samples to read
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t adc_read_fifo(uint32_t *data, uint32_t samples)
{
	if (!data) {
		return -EINVAL;
	}

	return ad4130_read_fifo(ad4130_dev_inst, data, samples);
}

/*!
 * @brief	Read ADC raw data for recently sampled channel
 * @param	adc_data[in, out] - Pointer to adc data read variable
 * @param	input_chn[in] - Input channel (optional)
 * @return	0 in case of success, negative error code otherwise
 * @note	This function is intended to call from the conversion end trigger
 *			event. Therefore, this function should just read raw ADC data
 *			without further monitoring conversion end event.
 *			Continuous conversion mode is used to for this operation.
 */
static int32_t adc_read_converted_sample(uint32_t *adc_data,
		uint8_t input_chn)
{
	if (!adc_data) {
		return -EINVAL;
	}

	/* Read the ADC data for previously sampled channel in sequencer */
	return ad413x_reg_read(ad4130_dev_inst, AD413X_REG_DATA, adc_data);
}

/*!
 * @brief	Function to read the single ADC sample (raw data) for input channel
 * @param	input_chn[in] - Input channel to be sampled and read data for
 * @param	raw_data[in, out]- ADC raw data
 * @return	0 in case of success, negative error code otherwise
 * @note	The single conversion mode is used to read a single sample
 */
int32_t read_single_sample(uint8_t input_chn, uint32_t *adc_raw)
{
	uint32_t chn_mask = 0;
	uint8_t chn;
	int32_t ret;

	if (!adc_raw) {
		return -EINVAL;
	}

	/* Disable all active channels */
	for (chn = 0; chn < ADC_USER_CHANNELS; chn++) {
		if (ad4130_dev_inst->ch[chn].enable) {
			chn_mask |= (1 << chn);

			/* Disable the current channel */
			ret = ad413x_ch_en(ad4130_dev_inst, chn, 0);
			if (ret) {
				return ret;
			}
		}
	}

	/* Enable user input channel */
	if (!ad4130_dev_inst->ch[input_chn].enable) {
		ret = ad413x_ch_en(ad4130_dev_inst, input_chn, 1);
		if (ret) {
			return ret;
		}
	}

	/* Put device into single conversion mode */
	ret = ad413x_set_adc_mode(ad4130_dev_inst, AD413X_SINGLE_CONV_MODE);
	if (ret) {
		return ret;
	}

	/* Monitor conversion and read the result */
	ret = ad413x_mon_conv_and_read_data(ad4130_dev_inst, adc_raw);

	/* Disable user input channel */
	ret = ad413x_ch_en(ad4130_dev_inst, input_chn, 0);
	if (ret) {
		return ret;
	}

	return 0;
}

/********* Device Independent Data Capture Code Begin ************/

/*!
 * @brief	Reset the data capture specific variables
 * @return	none
 */
static void reset_data_capture(void)
{
	/* Reset data capture flags */
	start_cont_data_capture = false;
	start_fifo_mode_data_capture = false;
	num_of_active_channels = 0;
	chn_indx = 0;

	/* Reset acquisition buffer states and clear old data */
	acq_buffer.state = BUF_EMPTY;
	acq_buffer.wr_indx = 0;
	acq_buffer.rd_indx = 0;
	acq_buffer.reindex_buffer = false;
	acq_buffer.wr_pdata = adc_data_buffer;
	acq_buffer.rd_pdata = adc_data_buffer;
	max_buffer_sz = DATA_BUFFER_SIZE;
}

/*!
 * @brief	Function to prepare the data ADC capture for new READBUFF
 *			request from IIO client (for active channels)
 * @param	ch_mask[in] - Channels to enable for data capturing
 * @param	sample_size[in] - Sample size in bytes
 * @return	0 in case of success, negative error code otherwise
 */
int32_t prepare_data_transfer(uint32_t ch_mask, uint8_t sample_size)
{
	int32_t ret;

	/* Reset data capture module specific flags and variables */
	reset_data_capture();

	sample_size_in_bytes = sample_size;

	/* Store active channels */
	ret = adc_store_active_chns(ch_mask);
	if (ret) {
		return ret;
	}

#if (DATA_CAPTURE_MODE == CONTINUOUS_DATA_CAPTURE)
	/* Trigger continuous data capture */
	ret = adc_start_data_capture();
	if (ret) {
		return ret;
	}

	acq_buffer.state = BUF_AVAILABLE;
	start_cont_data_capture = true;
#endif

	return 0;
}

/*!
 * @brief	Function to stop ADC data capture
 * @return	0 in case of success, negative error code otherwise
 */
int32_t end_data_transfer(void)
{
	int32_t ret;
	start_cont_data_capture = false;

	/* Stop data capture */
	ret = adc_stop_data_capture();
	if (ret) {
		return ret;
	}

	/* Reset data capture module specific flags and variables */
	reset_data_capture();

	return 0;
}

/*!
 * @brief	Perform buffer read operations to read requested samples
 * @param	nb_of_samples[in] - Requested number of samples to read
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t buffer_read_operations(uint32_t nb_of_samples)
{
	uint32_t timeout = BUF_READ_TIMEOUT;
	int32_t offset;

	/* Wait until requested samples are available in the buffer to read */
	do {
		if (acq_buffer.wr_indx >= acq_buffer.rd_indx) {
			offset = acq_buffer.wr_indx - acq_buffer.rd_indx;
		} else {
			offset = max_buffer_sz + (acq_buffer.wr_indx - acq_buffer.rd_indx);
		}

		timeout--;
	} while ((offset < (int32_t)(nb_of_samples)) && (timeout > 0));

	if (timeout == 0) {
		/* This returns the empty buffer */
		return -EIO;
	}

	if (acq_buffer.rd_indx >= max_buffer_sz) {
		acq_buffer.rd_indx = 0;
	}

	return 0;
}

/*!
 * @brief	Perform buffer write operations such as buffer full or empty
 *			check, resetting buffer index and pointers, etc
 * @return	none
 */
static void buffer_write_operations(void)
{
	acq_buffer.wr_indx++;

	/* Perform buffer full check and operations */
	if (acq_buffer.wr_indx >= max_buffer_sz) {
		if ((acq_buffer.rd_indx >= num_of_requested_samples)
		    && (acq_buffer.rd_indx != 0)) {
			/* Reset buffer write index and write pointer when enough
			 * space available in the buffer to wrap to start */
			acq_buffer.wr_indx = 0;
			acq_buffer.wr_pdata = adc_data_buffer;
			if (acq_buffer.rd_indx >= max_buffer_sz) {
				/* Wrap the read index and read pointer to start of buffer
				 * if buffer is completely read/emptied */
				acq_buffer.rd_indx = 0;
				acq_buffer.rd_pdata = adc_data_buffer;
			}

			acq_buffer.state = BUF_AVAILABLE;
		} else {
			/* Wait until enough space available to wrap buffer write index
			 * at the start of buffer */
			acq_buffer.wr_indx = max_buffer_sz;
			acq_buffer.state = BUF_FULL;
			acq_buffer.reindex_buffer = true;
		}
	}
}

/*!
 * @brief	This is an ISR (Interrupt Service Routine) to monitor end of conversion event.
 * @param	ctx[in] - Callback context (unused)
 * @return	none
 * @details	This is an Interrupt callback function/ISR invoked in synchronous/asynchronous
 *			manner depending upon the application implementation. The conversion results
 *			are read into acquisition buffer and control continue to sample next channel.
 *			This continues until conversion is stopped (through IIO client command)
 */
void data_capture_callback(void *ctx)
{
	uint32_t adc_sample;
	volatile uint8_t *wr_addr;

	if (start_cont_data_capture == true) {
		/* Read the sample for channel which has been sampled recently */
		if (!adc_read_converted_sample(&adc_sample,
					       active_channels[chn_indx])) {
			do {
				if (acq_buffer.state == BUF_AVAILABLE) {
					if (acq_buffer.reindex_buffer) {
						/* Buffer refilling must start with first active channel data
						 * for IIO client to synchronize the buffered data */
						if (chn_indx != 0) {
							break;
						}
						acq_buffer.reindex_buffer = false;
					}

					/* Copy adc samples into acquisition buffer to transport over
					 * communication link */
					wr_addr = (volatile uint8_t *)(acq_buffer.wr_pdata + (acq_buffer.wr_indx *
								       sample_size_in_bytes));
					memcpy((uint8_t *)wr_addr, &adc_sample, sample_size_in_bytes);
				}

				/* Perform buffer write operations */
				buffer_write_operations();
			} while (0);

			/* Track the count for recently sampled channel */
			chn_indx++;
			if (chn_indx >= num_of_active_channels) {
				chn_indx = 0;
			}
		}
	}
}

/*!
 * @brief	This is an ISR (Interrupt Service Routine) to monitor FIFO data available event.
 *			This function is expected to be called asynchronously when data from internal device
 *			FIFO is available to read.
 * @param	ctx[in] - Callback context (unused)
 * @return	none
 */
void fifo_data_capture_callback(void *ctx)
{
	if (start_fifo_mode_data_capture) {
		fifo_data_available = true;
	}
}

/*!
 * @brief	Capture requested number of ADC samples in burst mode
 * @param	pbuf[out] - Pointer to ADC data buffer
 * @param	nb_of_samples[in] - Number of samples to be read
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t read_burst_data(int8_t *pbuf, uint32_t nb_of_samples)
{
	uint32_t sample_indx = 0;
	uint32_t adc_raw;
	int32_t ret;

	if (!pbuf) {
		return -EINVAL;
	}

	ret = adc_start_data_capture();
	if (ret) {
		return ret;
	}

	while (sample_indx < nb_of_samples) {
		ret = adc_read_single_sample(&adc_raw);
		if (ret) {
			return ret;
		}

		/* Copy adc samples into acquisition buffer to transport over
		 * communication link */
		memcpy((uint8_t *)pbuf, &adc_raw, sample_size_in_bytes);
		pbuf += sample_size_in_bytes;

		sample_indx++;
	}

	/* Stop any active conversion */
	ret = adc_stop_data_capture();
	if (ret) {
		return ret;
	}

	return 0;
}

/*!
 * @brief	Capture requested number of ADC samples in FIFO mode
 * @param	pbuf[in] - Input buffer
 * @param	nb_of_samples[in] - Number of samples to read
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t read_fifo_data(int8_t *pbuf,
			      uint32_t nb_of_samples)
{
	uint32_t sample_cnt;
	uint32_t remaining_samples = nb_of_samples;
	uint32_t timeout = BUF_READ_TIMEOUT;
	int32_t ret;

	if (!pbuf) {
		return -EINVAL;
	}

	fifo_data_available = false;

	ret = adc_start_fifo_data_capture();
	if (ret) {
		return ret;
	}

	/* Read all requeted samples into acquisition buffer */
	do {
		/* Wait for new FIFO event */
		timeout = BUF_READ_TIMEOUT;
		do {
			timeout--;
		} while ((!fifo_data_available) && (timeout > 0));

		if (timeout == 0) {
			return -EIO;
		}

		fifo_data_available = false;

		if (remaining_samples > FIFO_SIZE) {
			nb_of_samples = FIFO_SIZE;
			remaining_samples -= nb_of_samples;
		} else {
			nb_of_samples = remaining_samples;
			remaining_samples = 0;
		}

		/* Read data from FIFO and store into local buffer */
		ret = adc_read_fifo(fifo_data, nb_of_samples);
		if (ret) {
			return ret;
		}

		/* Read offloaded FIFO data and store into acquisition buffer */
		for (sample_cnt = 0; sample_cnt < nb_of_samples; sample_cnt++) {
			memcpy(pbuf, &fifo_data[sample_cnt], sample_size_in_bytes);
			pbuf += sample_size_in_bytes;
		}
	} while (remaining_samples > 0);

	/* Stop any active conversion */
	ret = adc_stop_data_capture();
	if (ret) {
		return ret;
	}

	return 0;
}

/*!
 * @brief	Read requested number of ADC samples in continuous mode
 * @param	pbuf[in] - Pointer to data buffer
 * @param	nb_of_samples[in] - Number of samples to read
 * @return	0 in case of success, negative error code otherwise
 * @note	The actual sample capturing happens through interrupt. This
 *			function tracks the buffer read pointer to read block of data
 */
static int32_t read_continuous_conv_data(int8_t **pbuf, uint32_t nb_of_samples)
{
	volatile int8_t *rd_addr;
	int32_t ret;

	if (!pbuf) {
		return -EINVAL;
	}

	/* Determine the max available buffer size based on the requested
	 * samples count and actual avilable buffer size. Buffer should be
	 * capable of holding all requested 'n' samples from previous write
	 * index upto to the end of buffer, as data is read linearly
	 * from adc buffer in IIO library.
	 * E.g. If actual buffer size is 2048 samples and requested samples
	 * are 1600, max available buffer size is actually 1600. So in given
	 * iteration, only 1600 samples will be stored into buffer and after
	 * that buffer indexes will be wrapped to a start of buffer. If index
	 * is not wrapped, the next 1600 requested samples won't accomodate into
	 * remaining 448 samples space. As buffer is read in linear fashion, the
	 * read index can't be wrapped to start of buffer to read remaining samples.
	 * So max available space in this case is 2048 but only utilized space
	 * will be 1600 in single read buffer request from IIO client.
	 **/
	max_buffer_sz = ((DATA_BUFFER_SIZE / sample_size_in_bytes) /
			 nb_of_samples) * nb_of_samples;

	ret = buffer_read_operations(nb_of_samples);
	if (ret) {
		return ret;
	}

	/* Get the next read address */
	rd_addr = (volatile int8_t *)(acq_buffer.rd_pdata + (acq_buffer.rd_indx *
				      sample_size_in_bytes));
	acq_buffer.rd_indx += nb_of_samples;

	/* Update the IIO buffer pointer to point to next read start location */
	*pbuf = rd_addr;

	return 0;
}

/*!
 * @brief	Function to read the ADC buffered raw data requested
 *			by IIO client
 * @param	pbuf[in] - Pointer to data buffer
 * @param	nb_of_bytes[in] - Number of bytes to read
 * @return	0 in case of success, negative error code otherwise
 */
int32_t read_buffered_data(int8_t **pbuf, uint32_t nb_of_bytes)
{
	int32_t ret;
	num_of_requested_samples = nb_of_bytes / sample_size_in_bytes;

#if (DATA_CAPTURE_MODE == BURST_DATA_CAPTURE)
	ret = read_burst_data(*pbuf, num_of_requested_samples);
#elif (DATA_CAPTURE_MODE == FIFO_DATA_CAPTURE)
	ret = read_fifo_data(*pbuf, num_of_requested_samples);
#else
	ret = read_continuous_conv_data(pbuf, num_of_requested_samples);
#endif

	if (ret) {
		return ret;
	}

	return 0;
}
