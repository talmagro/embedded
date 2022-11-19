/***************************************************************************//**
 *   @file   ad4130_data_capture.h
 *   @brief  Header for AD4130 data capture interfaces
********************************************************************************
 * Copyright (c) 2021-22 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef AD4130_DATA_CAPTURE_H_
#define AD4130_DATA_CAPTURE_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

#if defined(USE_SDRAM_CAPTURE_BUFFER)
#define adc_data_buffer				SDRAM_START_ADDRESS
#define DATA_BUFFER_SIZE			SDRAM_SIZE_BYTES
#else
extern int8_t adc_data_buffer[];
#define DATA_BUFFER_SIZE			(32768)		// 32kbytes
#endif

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

/******************************************************************************/
/************************ Public Declarations *********************************/
/******************************************************************************/

int32_t ad4130_data_capture_init(void);
int32_t read_single_sample(uint8_t input_chn, uint32_t *raw_data);
int32_t read_buffered_data(int8_t **pbuf, uint32_t nb_of_bytes);
int32_t prepare_data_transfer(uint32_t ch_mask, uint8_t sample_size);
int32_t end_data_transfer(void);
void data_capture_callback(void *ctx);
void fifo_data_capture_callback(void *ctx);

#endif /* AD4130_DATA_CAPTURE_H_ */
