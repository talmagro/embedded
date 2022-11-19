/***************************************************************************//**
 *   @file    app_config.c
 *   @brief   Application configurations module
 *   @details This module contains the configurations needed for IIO application
********************************************************************************
 * Copyright (c) 2020-2022 Analog Devices, Inc.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdbool.h>

#include "app_config.h"
#include "ad4130_data_capture.h"
#include "no_os_error.h"
#include "no_os_uart.h"
#include "no_os_irq.h"
#include "no_os_gpio.h"

/******************************************************************************/
/************************ Macros/Constants ************************************/
/******************************************************************************/

/******************************************************************************/
/******************** Variables and User Defined Data Types *******************/
/******************************************************************************/

/* UART init parameters */
struct no_os_uart_init_param uart_init_params = {
	.device_id = NULL,
	.baud_rate = IIO_UART_BAUD_RATE,
	.size = NO_OS_UART_CS_8,
	.parity = NO_OS_UART_PAR_NO,
	.stop = NO_OS_UART_STOP_1_BIT,
	.extra = &uart_extra_init_params
};

/* SPI initialization parameters */
struct no_os_spi_init_param spi_init_params = {
	.max_speed_hz = 10000000,		// Max SPI Speed
	.chip_select = SPI_CSB,			// Chip Select
	.mode = NO_OS_SPI_MODE_3,		// CPOL = 1, CPHA = 1
	.platform_ops = &spi_ops,
	.extra = &spi_extra_init_params	// SPI extra configurations
};

/* External interrupt init parameters */
static struct no_os_irq_init_param ext_int_init_params = {
	.irq_ctrl_id = 0,
	.platform_ops = &irq_ops,
	.extra = &ext_int_extra_init_params
};

/* External interrupt callback descriptor */
static struct no_os_callback_desc ext_int_callback_desc = {
#if (DATA_CAPTURE_MODE == CONTINUOUS_DATA_CAPTURE)
	data_capture_callback,
#elif (DATA_CAPTURE_MODE == FIFO_DATA_CAPTURE)
	fifo_data_capture_callback,
#else
	NULL,
#endif
	NULL,
	NULL
};

/* Conversion monitor GPO init parameters */
static struct no_os_gpio_init_param conv_mon_gpio_init_params = {
	.number = CONV_MON,
	.platform_ops = &gpio_ops,
	.extra = NULL
};

/* UART descriptor */
struct no_os_uart_desc *uart_desc;

/* External interrupt descriptor */
struct no_os_irq_ctrl_desc *external_int_desc;

/* LED GPO descriptor */
struct no_os_gpio_desc *conv_mon_gpio_desc = NULL;

/******************************************************************************/
/************************ Functions Prototypes ********************************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief 	Initialize the GPIOs
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t init_gpio(void)
{
	int32_t ret;

	/* Initialize the conversion monitor GPO */
	ret = no_os_gpio_get_optional(&conv_mon_gpio_desc,
				      &conv_mon_gpio_init_params);
	if (ret) {
		return ret;
	}

	ret = no_os_gpio_direction_input(conv_mon_gpio_desc);
	if (ret) {
		return ret;
	}

	return 0;
}

/**
 * @brief 	Initialize the UART peripheral
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t init_uart(void)
{
	return no_os_uart_init(&uart_desc, &uart_init_params);
}

/**
 * @brief 	Initialize the IRQ contoller
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t init_interrupt(void)
{
	int32_t ret;

#if (DATA_CAPTURE_MODE != BURST_DATA_CAPTURE_MODE)
	/* Init interrupt controller for external interrupt (for monitoring
	 * conversion event on BUSY pin) */
	ext_int_extra_init_params.gpio_irq_pin = CONV_MON;
	ret = no_os_irq_ctrl_init(&external_int_desc, &ext_int_init_params);
	if (ret) {
		return ret;
	}

	/* Register a callback function for external interrupt */
	ret = no_os_irq_register_callback(external_int_desc,
					  EXT_INT_ID,
					  &ext_int_callback_desc);
	if (ret) {
		return ret;
	}

	ret = no_os_irq_trigger_level_set(external_int_desc,
					  EXT_INT_ID, NO_OS_IRQ_EDGE_RISING);
	if (ret) {
		return ret;
	}

	/* Enable external interrupt */
	ret = no_os_irq_enable(external_int_desc, EXT_INT_ID);
	if (ret) {
		return ret;
	}
#endif

	return 0;
}

/**
 * @brief 	Initialize the system peripherals
 * @return	0 in case of success, negative error code otherwise
 */
int32_t init_system(void)
{
	int32_t ret;

	ret = init_gpio();
	if (ret) {
		return ret;
	}

	ret = init_uart();
	if (ret) {
		return ret;
	}

	ret = init_interrupt();
	if (ret) {
		return ret;
	}

#if defined(USE_SDRAM_CAPTURE_BUFFER)
	ret = sdram_init();
	if (ret) {
		return ret;
	}
#endif

	return 0;
}
