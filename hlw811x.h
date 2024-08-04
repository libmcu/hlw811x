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

typedef enum {
	HLW811X_ERROR_NONE		= 0,
} hlw811x_error_t;

hlw811x_error_t hlw811x_init(void);

#if defined(__cplusplus)
}
#endif

#endif /* HLW811X_H */
