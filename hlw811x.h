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

typedef enum {
	HLW811X_INTR_AVERAGE_UPDATED		= 0x0001,
	HLW811X_INTR_PULSE_OUT_A		= 0x0002,
	HLW811X_INTR_PULSE_OUT_B		= 0x0004,
	HLW811X_INTR_ACTIVE_POWER_OVERFLOW_A	= 0x0008,
	HLW811X_INTR_ACTIVE_POWER_OVERFLOW_B	= 0x0010,
	HLW811X_INTR_RESERVED			= 0x0020,
	HLW811X_INTR_IRQ			= 0x0020,
	HLW811X_INTR_INSTANTAENOUS_UPDATED	= 0x0040,
	HLW811X_INTR_OVER_CURRENT_A		= 0x0080,
	HLW811X_INTR_OVER_CURRENT_B		= 0x0100,
	HLW811X_INTR_OVER_VOLTAGE		= 0x0200,
	HLW811X_INTR_OVERLOAD			= 0x0400,
	HLW811X_INTR_UNDER_VOLTAGE		= 0x0800,
	HLW811X_INTR_ZERO_CROSSING_CURRENT_A	= 0x1000,
	HLW811X_INTR_ZERO_CROSSING_CURRENT_B	= 0x2000,
	HLW811X_INTR_ZERO_CROSSING_VOLTAGE	= 0x4000,
	HLW811X_INTR_B_LEAKAGE			= 0x8000,
} hlw811x_intr_t;

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
 * @brief Create and initialize an HLW811X device instance.
 *
 * This function creates and initializes an HLW811X device instance with the
 * specified interface and context.
 *
 * @param[in] interface The interface to be used for communication with the
 *            HLW811X device.
 * @param[in] ctx Pointer to the context or additional parameters required for
 *            the interface.
 *
 * @return struct hlw811x* Pointer to the created HLW811X device instance, or
 *         NULL on failure.
 */
struct hlw811x *hlw811x_create(hlw811x_interface_t interface, void *ctx);

/**
 * @brief Destroy an HLW811X device instance.
 *
 * This function destroys the specified HLW811X device instance, freeing any
 * resources that were allocated for it.
 *
 * @param[in] hlw811x Pointer to the HLW811X device instance to be destroyed.
 */
void hlw811x_destroy(struct hlw811x *hlw811x);

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
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Returns an error code indicating the success or
 *                         failure of the reset operation.
 */
hlw811x_error_t hlw811x_reset(struct hlw811x *self);

/**
 * @brief Write data to a specified HLW811X register.
 *
 * This function writes the specified data to the HLW811X register at the given
 * address.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] addr The address of the HLW811X register to write to.
 * @param[in] data Pointer to the data to be written to the register.
 * @param[in] datalen Length of the data to be written.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_write_reg(struct hlw811x *self,
		hlw811x_reg_addr_t addr, const uint8_t *data, size_t datalen);

/**
 * @brief Read data from a specified HLW811X register.
 *
 * This function reads data from the HLW811X register at the given address into
 * the specified buffer.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] addr The address of the HLW811X register to read from.
 * @param[out] buf Pointer to the buffer to store the read data.
 * @param[in] bufsize Size of the buffer.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_read_reg(struct hlw811x *self,
		hlw811x_reg_addr_t addr, uint8_t *buf, size_t bufsize);

/**
 * @brief Enable a specified HLW811X channel.
 *
 * This function enables the specified HLW811X channel for operation.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to enable.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_channel(struct hlw811x *self,
		hlw811x_channel_t channel);

/**
 * @brief Disable a specified HLW811X channel.
 *
 * This function disables the specified HLW811X channel, stopping its operation.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to disable.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_channel(struct hlw811x *self,
		hlw811x_channel_t channel);

/**
 * @brief Enable pulse output and energy accumulation for a specified channel.
 *
 * This function enables the pulse output and energy accumulation for the
 * specified HLW811X channel.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to enable pulse output for.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_pulse(struct hlw811x *self,
		hlw811x_channel_t channel);

/**
 * @brief Disable pulse output and energy accumulation for a specified channel.
 *
 * This function disables the pulse output and energy accumulation for the
 * specified HLW811X channel.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to disable pulse output for.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_pulse(struct hlw811x *self,
		hlw811x_channel_t channel);

/**
 * @brief Enable waveform output.
 *
 * This function enables the waveform output feature of the HLW811X,
 * allowing the waveform data to be output.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_waveform(struct hlw811x *self);

/**
 * @brief Disable waveform output.
 *
 * This function disables the waveform output feature of the HLW811X,
 * stopping the waveform data from being output.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_waveform(struct hlw811x *self);

/**
 * @brief Enable zero-crossing detection.
 *
 * This function enables the zero-crossing detection feature of the HLW811X,
 * allowing the detection of zero-crossing points in the waveform.
 *
 * @note hlw811x_enable_waveform() must be enabled to use this functionality.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_zerocrossing(struct hlw811x *self);

/**
 * @brief Disable zero-crossing detection.
 *
 * This function disables the zero-crossing detection feature of the HLW811X,
 * stopping the detection of zero-crossing points in the waveform.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_zerocrossing(struct hlw811x *self);

/**
 * @brief Enable power factor measurement.
 *
 * This function enables the power factor measurement feature of the HLW811X,
 * allowing the measurement of the power factor.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_power_factor(struct hlw811x *self);

/**
 * @brief Disable power factor measurement.
 *
 * This function disables the power factor measurement feature of the HLW811X,
 * stopping the measurement of the power factor.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_power_factor(struct hlw811x *self);

/**
 * @brief Enable energy clearance for a specified HLW811X channel.
 *
 * This function enables the energy clearance feature for the specified HLW811X
 * channel, allowing the energy accumulation to be cleared after reading the
 * register.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to enable energy clearance for.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_energy_clearance(struct hlw811x *self,
		hlw811x_channel_t channel);

/**
 * @brief Disable energy clearance for a specified HLW811X channel.
 *
 * This function disables the energy clearance feature for the specified HLW811X
 * channel, preventing the energy accumulation from being cleared.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to disable energy clearance for.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_energy_clearance(struct hlw811x *self,
		hlw811x_channel_t channel);

/**
 * @brief Enable high-pass filter for a specified HLW811X channel.
 *
 * This function enables the high-pass filter (HPF) for the specified HLW811X
 * channel, which can help to remove DC components from the signal.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to enable the high-pass filter for.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_hpf(struct hlw811x *self,
		hlw811x_channel_t channel);

/**
 * @brief Disable high-pass filter for a specified HLW811X channel.
 *
 * This function disables the high-pass filter (HPF) for the specified HLW811X
 * channel, allowing DC components to pass through the signal.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to disable the high-pass filter for.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_hpf(struct hlw811x *self,
		hlw811x_channel_t channel);

/**
 * @brief Enable B channel comparator.
 *
 * This function enables the comparator for the B channel of the HLW811X,
 * allowing for comparison operations on the B channel. When the input signal
 * exceeds 125mV, the comparator output is set to high and an interrupt is
 * generated.
 *
 * @note hlw811x_disable_temperature_sensor() should be called before enabling
 * this functionality.
 *
 * @note The latency from the detection of the input signal to the generation of
 * interrupt is 2ms.
 *
 * @note When the comparator is enabled, the current measurement of the B
 * channel can't be used.
 *
 * @warn The external power of HLW811X must be disconnected and supply power to
 * HLW811X again after catching the interrupt to get it work properly.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_b_channel_comparator(struct hlw811x *self);

/**
 * @brief Disable B channel comparator.
 *
 * This function disables the comparator for the B channel of the HLW811X,
 * preventing comparison operations on the B channel.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_b_channel_comparator(struct hlw811x *self);

/**
 * @brief Enable temperature sensor.
 *
 * This function enables the temperature sensor of the HLW811X,
 * allowing for temperature measurements.
 *
 * @note hlw811x_set_b_channel_mode() must be set to B_MODE_TEMPERATURE to use
 * this functionality.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_temperature_sensor(struct hlw811x *self);

/**
 * @brief Disable temperature sensor.
 *
 * This function disables the temperature sensor of the HLW811X,
 * preventing temperature measurements.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_temperature_sensor(struct hlw811x *self);

/**
 * @brief Enable peak detection.
 *
 * This function enables the peak detection feature of the HLW811X,
 * allowing the detection of peak values in the channel signals.
 *
 * @note hlw811x_enable_waveform() must be enabled to use this functionality.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_peak_detection(struct hlw811x *self);

/**
 * @brief Disable peak detection.
 *
 * This function disables the peak detection feature of the HLW811X,
 * preventing the detection of peak values in the channel signals.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_peak_detection(struct hlw811x *self);

/**
 * @brief Enable overload detection.
 *
 * This function enables the overload detection feature of the HLW811X,
 * allowing the detection of overload conditions in the channel signals.
 *
 * @note hlw811x_enable_waveform() must be enabled to use this functionality.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_overload_detection(struct hlw811x *self);

/**
 * @brief Disable overload detection.
 *
 * This function disables the overload detection feature of the HLW811X,
 * preventing the detection of overload conditions int the channel signals.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_overload_detection(struct hlw811x *self);

/**
 * @brief Enable voltage drop detection.
 *
 * This function enables the voltage drop detection feature of the HLW811X,
 * allowing the detection of voltage drops in the channel signals.
 *
 * @note hlw811x_enable_waveform() must be enabled to use this functionality.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_voltage_drop_detection(struct hlw811x *self);

/**
 * @brief Disable voltage drop detection.
 *
 * This function disables the voltage drop detection feature of the HLW811X,
 * preventing the detection of voltage drops in the channel signals.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_voltage_drop_detection(struct hlw811x *self);

/**
 * @brief Enable specific interrupts for the HLW811X.
 *
 * This function enables the specified interrupts for the HLW811X,
 * allowing the device to generate interrupt signals for the specified events.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] ints The interrupts to enable.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_enable_interrupt(struct hlw811x *self,
		hlw811x_intr_t ints);

/**
 * @brief Disable specific interrupts for the HLW811X.
 *
 * This function disables the specified interrupts for the HLW811X,
 * preventing the device from generating interrupt signals for the specified
 * events.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] ints The interrupts to disable.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_disable_interrupt(struct hlw811x *self,
		hlw811x_intr_t ints);

/**
 * @brief Set the interrupt mode for the HLW811X.
 *
 * This function sets the interrupt mode for the HLW811X, configuring how
 * the device handles and generates interrupt signals.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] int1 Interrupt mode for INT1 to set.
 * @param[in] int2 Interrupt mode for INT2 to set.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_set_interrupt_mode(struct hlw811x *self,
		hlw811x_intr_t int1, hlw811x_intr_t int2);

/**
 * @brief Get the current interrupt status for the HLW811X.
 *
 * This function retrieves the current interrupt status from the HLW811X,
 * indicating which interrupts are currently active.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] ints Pointer to the variable where the current interrupt status
 * will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_interrupt(struct hlw811x *self,
		hlw811x_intr_t *ints);

/**
 * @brief Get the current interrupt status for the HLW811X.
 *
 * This function retrieves the current interrupt status from the HLW811X,
 * indicating which interrupts are currently active.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] ints Pointer to the variable where the current interrupt status
 * will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_interrupt_ext(struct hlw811x *self,
		hlw811x_intr_t *ints);

/**
 * @brief Select the active HLW811X channel.
 *
 * This function selects the specified HLW811X channel as the active channel for
 * subsequent operations.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to select.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_select_channel(struct hlw811x *self,
		hlw811x_channel_t channel);

/**
 * @brief Read the current active HLW811X channel.
 *
 * This function reads the current active channel of the HLW811X and stores it
 * in the provided channel variable.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] channel Pointer to the variable where the current active channel
 * will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_read_current_channel(struct hlw811x *self,
		hlw811x_channel_t *channel);

/**
 * @brief Read the calibration coefficients from the HLW811X.
 *
 * This function reads the calibration coefficients from the HLW811X and stores
 * them in the provided hlw811x_coeff structure.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] coeff Pointer to the structure where the calibration coefficients
 * will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_read_coeff(struct hlw811x *self,
		struct hlw811x_coeff *coeff);

/**
 * @brief Set the resistor ratio for the HLW811X.
 *
 * This function sets the resistor ratio for the HLW811X, which is used for
 * internal calculations related to voltage and current measurements.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param ratio[in] Pointer to the structure containing the resistor ratio
 * values.
 */
void hlw811x_set_resistor_ratio(struct hlw811x *self,
		const struct hlw811x_resistor_ratio *ratio);

/**
 * @brief Get the resistor ratio for the HLW811X.
 *
 * This function retrieves the current resistor ratio settings from the HLW811X.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param ratio[out] Pointer to the structure where the resistor ratio values
 * will be stored.
 */
void hlw811x_get_resistor_ratio(struct hlw811x *self,
		struct hlw811x_resistor_ratio *ratio);

/**
 * @brief Set the programmable gain amplifier (PGA) settings for the HLW811X.
 *
 * This function sets the PGA settings for the HLW811X, which are used to adjust
 * the gain for voltage and current measurements.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] pga Pointer to the structure containing the PGA settings.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_set_pga(struct hlw811x *self,
		const struct hlw811x_pga *pga);

/**
 * @brief Get the programmable gain amplifier (PGA) settings for the HLW811X.
 *
 * This function retrieves the current PGA settings from the HLW811X.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param pga[out] Pointer to the structure where the PGA settings will be
 * stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_pga(struct hlw811x *self, struct hlw811x_pga *pga);

/**
 * @brief Set the active power calculation mode for the HLW811X.
 *
 * This function sets the active power calculation mode for the HLW811X,
 * which determines how the active power is calculated.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] mode The active power calculation mode to set.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_set_active_power_calc_mode(struct hlw811x *self,
		hlw811x_active_power_mode_t mode);

/**
 * @brief Get the active power calculation mode for the HLW811X.
 *
 * This function retrieves the current active power calculation mode from the
 * HLW811X.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] mode Pointer to the variable where the current active power
 * calculation mode will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_active_power_calc_mode(struct hlw811x *self,
		hlw811x_active_power_mode_t *mode);

/**
 * @brief Set the RMS calculation mode for the HLW811X.
 *
 * This function sets the RMS (Root Mean Square) calculation mode for the
 * HLW811X, which determines how the RMS values are calculated.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] mode The RMS calculation mode to set.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_set_rms_calc_mode(struct hlw811x *self,
		hlw811x_rms_mode_t mode);

/**
 * @brief Get the RMS calculation mode for the HLW811X.
 *
 * This function retrieves the current RMS (Root Mean Square) calculation mode
 * from the HLW811X.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] mode Pointer to the variable where the current RMS calculation
 * mode will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_rms_calc_mode(struct hlw811x *self,
		hlw811x_rms_mode_t *mode);

/**
 * @brief Set the data update frequency for the HLW811X.
 *
 * This function sets the data update frequency for the HLW811X, which determines
 * how often the data is updated.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] freq The data update frequency to set.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_set_data_update_frequency(struct hlw811x *self,
		hlw811x_data_update_freq_t freq);

/**
 * @brief Get the data update frequency for the HLW811X.
 *
 * This function retrieves the current data update frequency from the HLW811X.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] freq Pointer to the variable where the current data update
 * frequency will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_data_update_frequency(struct hlw811x *self,
		hlw811x_data_update_freq_t *freq);

/**
 * @brief Set the mode for HLW811X channel B.
 *
 * This function sets the mode for HLW811X channel B, which determines the
 * operational behavior of channel B.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] mode The mode to set for channel B.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_set_channel_b_mode(struct hlw811x *self,
		hlw811x_channel_b_mode_t mode);

/**
 * @brief Get the mode for HLW811X channel B.
 *
 * This function retrieves the current mode of HLW811X channel B.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] mode Pointer to the variable where the current mode of channel B
 * will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_channel_b_mode(struct hlw811x *self,
		hlw811x_channel_b_mode_t *mode);

/**
 * @brief Set the zero-crossing detection mode for the HLW811X.
 *
 * This function sets the zero-crossing detection mode for the HLW811X,
 * which determines how zero-crossing points in the waveform are detected.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] mode The zero-crossing detection mode to set.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_set_zerocrossing_mode(struct hlw811x *self,
		hlw811x_zerocrossing_mode_t mode);

/**
 * @brief Get the zero-crossing detection mode for the HLW811X.
 *
 * This function retrieves the current zero-crossing detection mode from the
 * HLW811X.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] mode Pointer to the variable where the current zero-crossing
 * detection mode will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_zerocrossing_mode(struct hlw811x *self,
		hlw811x_zerocrossing_mode_t *mode);

/**
 * @brief Get the RMS value for a specified HLW811X channel.
 *
 * This function retrieves the RMS (Root Mean Square) value for the specified
 * HLW811X channel and stores it in the provided milliunit variable.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to get the RMS value for.
 * @param[out] milliunit Pointer to the variable where the RMS value (in
 * milliunits) will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_rms(struct hlw811x *self,
		hlw811x_channel_t channel, int32_t *milliunit);

/**
 * @brief Get the power value for a specified HLW811X channel.
 *
 * This function retrieves the power value for the specified HLW811X channel
 * and stores it in the provided milliwatt variable.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to get the power value for.
 * @param[out] milliwatt Pointer to the variable where the power value (in
 * milliwatts) will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_power(struct hlw811x *self,
		hlw811x_channel_t channel, int32_t *milliwatt);

/**
 * @brief Get the energy value for a specified HLW811X channel.
 *
 * This function retrieves the energy value for the specified HLW811X channel
 * and stores it in the provided Wh variable.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[in] channel The HLW811X channel to get the energy value for.
 * @param[out] Wh Pointer to the variable where the energy value (in watt-hours)
 * will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_energy(struct hlw811x *self,
		hlw811x_channel_t channel, int32_t *Wh);

/**
 * @brief Get the frequency of the HLW811X.
 *
 * This function retrieves the frequency of the HLW811X and stores it in the
 * provided centihertz variable.
 *
 * @note hlw811x_enable_waveform() must be enabled to use this functionality.
 *
 * @note hlw811x_enable_zerocrossing() must be enabled to use this
 * functionality.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] centihertz Pointer to the variable where the frequency (in
 * centihertz) will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_frequency(struct hlw811x *self, int32_t *centihertz);

/**
 * @brief Get the power factor of the HLW811X.
 *
 * This function retrieves the power factor of the HLW811X and stores it in the
 * provided centiunit variable.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] centiunit Pointer to the variable where the power factor (in
 * centiunits) will be stored.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_power_factor(struct hlw811x *self, int32_t *centi);

/**
 * @brief Get the phase angle of the HLW811X.
 *
 * This function retrieves the phase angle of the HLW811X and stores it in the
 * provided centidegree variable.
 *
 * @param[in] self A pointer to the HLW811X device structure.
 * @param[out] centidegree Pointer to the variable where the phase angle (in
 * centidegrees) will be stored.
 * @param[int] freq The line frequency to use for the phase angle calculation.
 *
 * @return hlw811x_error_t Error code indicating the result of the operation.
 */
hlw811x_error_t hlw811x_get_phase_angle(struct hlw811x *self,
		int32_t *centidegree, hlw811x_line_freq_t freq);

#if defined(__cplusplus)
}
#endif

#endif /* HLW811X_H */
