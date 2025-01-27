/*
 * SPDX-FileCopyrightText: 2024 Kyunghwan Kwon <k@libmcu.org>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HLW811X_OVERRIDES_H
#define HLW811X_OVERRIDES_H

#if defined(__cplusplus)
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Writes data to the HLW811X device at a low level.
 *
 * This function sends the specified data to the HLW811X device.
 *
 * @param[in] data Pointer to the data to be written.
 * @param[in] datalen Length of the data to be written.
 * @param[in] ctx Pointer to the context or additional parameters required for
 *            the write operation.
 *
 * @return int Returns 0 on success, or a negative error code on failure.
 */
int hlw811x_ll_write(const uint8_t *data, size_t datalen, void *ctx);

/**
 * @brief Reads data from the HLW811X device at a low level.
 *
 * This function reads data from the HLW811X device into the specified buffer.
 *
 * @param[out] buf Pointer to the buffer where the read data will be stored.
 * @param[in] bufsize Size of the buffer.
 * @param[in] ctx Pointer to the context or additional parameters required for
 *            the read operation.
 *
 * @return int Returns the number of bytes read on success, or a negative error
 *             code on failure.
 */
int hlw811x_ll_read(uint8_t *buf, size_t bufsize, void *ctx);

#if defined(__cplusplus)
}
#endif

#endif /* HLW811X_OVERRIDES_H */
