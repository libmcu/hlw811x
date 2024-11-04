/*
 * SPDX-FileCopyrightText: 2024 Kyunghwan Kwon <k@libmcu.org>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HLW811X_H
#define HLW811X_H

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "hlw811x_regs.h"

typedef enum {
	HLW811X_ERROR_NONE,
	HLW811X_INVALID_PARAM,
	HLW811X_IO_ERROR,
	HLW811X_INCORRECT_RESPONSE,
	HLW811X_NOT_IMPLEMENTED,
} hlw811x_error_t;

typedef enum {
	HLW811X_CHANNEL_A,
	HLW811X_CHANNEL_B,
} hlw811x_channel_t;

typedef enum {
	HLW811X_UART,
	HLW811X_SPI,
} hlw811x_interface_t;

/**
 * @brief Initializes the HLW811X device with the specified interface.
 *
 * This function sets up the HLW811X device using the provided interface,
 * preparing it for operation.
 *
 * @param[in] interface The interface to be used.
 *
 * @return hlw811x_error_t Returns an error code indicating the success or
 *                         failure of the initialization.
 */
hlw811x_error_t hlw811x_init(hlw811x_interface_t interface);

/**
 * @brief Resets the HLW811X device.
 *
 * This function performs a reset operation on the HLW811X device,
 * restoring it to its default state.
 *
 * @note At least 60ms delay is required after reset before calling any other
 *       functions because the chip needs time to stabilize such as crystal
 *       oscillator start-up time.
 *
 * @return hlw811x_error_t Returns an error code indicating the success or
 *                         failure of the reset operation.
 */
hlw811x_error_t hlw811x_reset(void);

hlw811x_error_t hlw811x_write_reg(hlw811x_reg_addr_t addr,
		const uint8_t *data, size_t datalen);
hlw811x_error_t hlw811x_read_reg(hlw811x_reg_addr_t addr,
		uint8_t *buf, size_t bufsize);

hlw811x_error_t hlw811x_select_channel(hlw811x_channel_t channel);

#if defined(__cplusplus)
}
#endif

#endif /* HLW811X_H */
