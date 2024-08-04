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

int hlw811x_ll_write(const uint8_t *data, size_t datalen);
int hlw811x_ll_read(uint8_t *buf, size_t bufsize);

#if defined(__cplusplus)
}
#endif

#endif /* HLW811X_OVERRIDES_H */
