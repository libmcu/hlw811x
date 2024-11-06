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
	HLW811X_IO_MISSING_BYTES,
	HLW811X_INCORRECT_RESPONSE,
	HLW811X_NO_RESPONSE,
	HLW811X_NOT_IMPLEMENTED,
	HLW811X_BUFFER_TOO_SMALL,
	HLW811X_CHECKSUM_MISMATCH,
	HLW811X_INVALID_DATA,
} hlw811x_error_t;

enum hlw811x_channel {
	HLW811X_CHANNEL_A = 0x01,
	HLW811X_CHANNEL_B = 0x02,
	HLW811X_CHANNEL_U = 0x04,
	HLW811X_CHANNEL_ALL = HLW811X_CHANNEL_A |
		HLW811X_CHANNEL_B | HLW811X_CHANNEL_U,
};

typedef uint8_t hlw811x_channel_t;

typedef enum {
	HLW811X_UART,
	HLW811X_SPI,
} hlw811x_interface_t;

typedef enum {
	HLW811X_PGA_GAIN_1,
	HLW811X_PGA_GAIN_2,
	HLW811X_PGA_GAIN_4,
	HLW811X_PGA_GAIN_8,
	HLW811X_PGA_GAIN_16,
} hlw811x_pga_gain_t;

typedef enum {
	HLW811X_ACTIVE_POWER_MODE_POS_NEG_ALGEBRAIC,
	HLW811X_ACTIVE_POWER_MODE_POS,
	HLW811X_ACTIVE_POWER_MODE_POS_NEG_ABSOLUTE,
} hlw811x_active_power_mode_t;

typedef enum {
	HLW811X_RMS_MODE_AC,
	HLW811X_RMS_MODE_DC,
} hlw811x_rms_mode_t;

typedef enum {
	HLW811X_DATA_UPDATE_FREQ_HZ_3_4,
	HLW811X_DATA_UPDATE_FREQ_HZ_6_8,
	HLW811X_DATA_UPDATE_FREQ_HZ_13_65,
	HLW811X_DATA_UPDATE_FREQ_HZ_27_3,
} hlw811x_data_update_freq_t;

typedef enum {
	HLW811X_B_MODE_TEMPERATURE, /* measure temperature inside the chip only */
	HLW811X_B_MODE_NORMAL,
} hlw811x_channel_b_mode_t;

typedef enum {
	HLW811X_LINE_FREQ_50HZ,
	HLW811X_LINE_FREQ_60HZ,
} hlw811x_line_freq_t;

typedef enum {
	HLW811x_ZERO_CROSSING_MODE_POSITIVE,
	HLW811x_ZERO_CROSSING_MODE_NEGATIVE,
	HLW811x_ZERO_CROSSING_MODE_BOTH,
} hlw811x_zerocrossing_mode_t;

struct hlw811x_resistor_ratio {
	float K1_A; /* current channel A */
	float K1_B; /* current channel B */
	float K2; /* voltage */
};

struct hlw811x_coeff {
	struct {
		uint16_t A; /* RMS conversion coefficient for current channel A */
		uint16_t B; /* RMS conversion coefficient for current channel B */
		uint16_t U; /* RMS conversion coefficient for voltage */
	} rms;
	struct {
		uint16_t A; /* Active power conversion coefficient for channel A */
		uint16_t B; /* Active power conversion coefficient for channel B */
		uint16_t S; /* Apparent power conversion coefficient */
	} power;
	struct {
		uint16_t A; /* Active energy conversion coefficient for channel A */
		uint16_t B; /* Active energy conversion coefficient for channel B */
	} energy;

	uint16_t hfconst; /* pulse frequency constant */
};

struct hlw811x_pga {
	hlw811x_pga_gain_t A;
	hlw811x_pga_gain_t B;
	hlw811x_pga_gain_t U;
};

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

/**
 * @brief Write data to a specified HLW811X register.
 *
 * This function writes the specified data to the HLW811X register at the given
 * address.
 *
 * @param[in] addr The address of the HLW811X register to write to.
 * @param[in] data Pointer to the data to be written to the register.
 * @param[in] datalen Length of the data to be written.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_write_reg(hlw811x_reg_addr_t addr,
		const uint8_t *data, size_t datalen);

/**
 * @brief Read data from a specified HLW811X register.
 *
 * This function reads data from the HLW811X register at the given address into
 * the specified buffer.
 *
 * @param[in] addr The address of the HLW811X register to read from.
 * @param[out] buf Pointer to the buffer to store the read data.
 * @param[in] bufsize Size of the buffer.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_read_reg(hlw811x_reg_addr_t addr,
		uint8_t *buf, size_t bufsize);

/**
 * @brief Enable a specified HLW811X channel.
 *
 * This function enables the specified HLW811X channel for operation.
 *
 * @param[in] channel The HLW811X channel to enable.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_channel(hlw811x_channel_t channel);

/**
 * @brief Disable a specified HLW811X channel.
 *
 * This function disables the specified HLW811X channel, stopping its operation.
 *
 * @param[in] channel The HLW811X channel to disable.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_channel(hlw811x_channel_t channel);

/**
 * @brief Select the active HLW811X channel.
 *
 * This function selects the specified HLW811X channel as the active channel for
 * subsequent operations.
 *
 * @param[in] channel The HLW811X channel to select.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_select_channel(hlw811x_channel_t channel);

hlw811x_error_t hlw811x_read_current_channel(hlw811x_channel_t *channel);

hlw811x_error_t hlw811x_read_coeff(struct hlw811x_coeff *coeff);
void hlw811x_set_resistor_ratio(const struct hlw811x_resistor_ratio *ratio);
void hlw811x_get_resistor_ratio(struct hlw811x_resistor_ratio *ratio);
hlw811x_error_t hlw811x_set_pga(const struct hlw811x_pga *pga);
hlw811x_error_t hlw811x_get_pga(struct hlw811x_pga *pga);
hlw811x_error_t hlw811x_set_active_power_calc_mode(hlw811x_active_power_mode_t
		mode);
hlw811x_error_t hlw811x_get_active_power_calc_mode(hlw811x_active_power_mode_t
		*mode);
hlw811x_error_t hlw811x_set_rms_calc_mode(hlw811x_rms_mode_t mode);
hlw811x_error_t hlw811x_get_rms_calc_mode(hlw811x_rms_mode_t *mode);
hlw811x_error_t hlw811x_set_data_update_frequency(hlw811x_data_update_freq_t
		freq);
hlw811x_error_t hlw811x_get_data_update_frequency(hlw811x_data_update_freq_t
		*freq);
hlw811x_error_t hlw811x_set_channel_b_mode(hlw811x_channel_b_mode_t mode);
hlw811x_error_t hlw811x_get_channel_b_mode(hlw811x_channel_b_mode_t *mode);
hlw811x_error_t hlw811x_set_zerocrossing_mode(hlw811x_zerocrossing_mode_t mode);
hlw811x_error_t hlw811x_get_zerocrossing_mode(hlw811x_zerocrossing_mode_t
		*mode);
hlw811x_error_t hlw811x_enable_waveform(void);
hlw811x_error_t hlw811x_disable_waveform(void);
hlw811x_error_t hlw811x_enable_zerocrossing(void);
hlw811x_error_t hlw811x_disable_zerocrossing(void);
hlw811x_error_t hlw811x_enable_power_factor(void);
hlw811x_error_t hlw811x_disable_power_factor(void);
hlw811x_error_t hlw811x_enable_energy_clearance(hlw811x_channel_t channel);
hlw811x_error_t hlw811x_disable_energy_clearance(hlw811x_channel_t channel);

hlw811x_error_t hlw811x_get_rms(hlw811x_channel_t channel, int32_t *milliunit);
hlw811x_error_t hlw811x_get_power(hlw811x_channel_t channel, int32_t *milliwatt);
hlw811x_error_t hlw811x_get_energy(hlw811x_channel_t channel, int32_t *Wh);
/* NOTE: WaveEn and ZXEC must be enabled to use this function. */
hlw811x_error_t hlw811x_get_frequency(int32_t *centihertz);
hlw811x_error_t hlw811x_get_power_factor(int32_t *centiunit);
hlw811x_error_t hlw811x_get_phase_angle(int32_t *centidegree,
		hlw811x_line_freq_t freq);

/**
 * @brief Enable pulse output and energy accumulation for a specified channel.
 *
 * This function enables the pulse output and energy accumulation for the
 * specified HLW811X channel.
 *
 * @param[in] channel The HLW811X channel to enable pulse output for.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_pulse(hlw811x_channel_t channel);

/**
 * @brief Disable pulse output and energy accumulation for a specified channel.
 *
 * This function disables the pulse output and energy accumulation for the
 * specified HLW811X channel.
 *
 * @param[in] channel The HLW811X channel to disable pulse output for.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_pulse(hlw811x_channel_t channel);

#if defined(__cplusplus)
}
#endif

#endif /* HLW811X_H */
