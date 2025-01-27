/*
 * SPDX-FileCopyrightText: 2024 Kyunghwan Kwon <k@libmcu.org>
 *
 * SPDX-License-Identifier: MIT
 */

#include "hlw811x.h"
#include "hlw811x_overrides.h"

#include <string.h>
#include <stdlib.h>

#if !defined(HLW811X_MCLK)
#define HLW811X_MCLK		(3579545UL) /* Hz (= 3.579545MHz) */
#endif

#if !defined(HLW811X_DEBUG)
#define HLW811X_DEBUG(...)
#endif
#if !defined(HLW811X_INFO)
#define HLW811X_INFO(...)
#endif
#if !defined(HLW811X_ERROR)
#define HLW811X_ERROR(...)
#endif

enum {
	CMD_ENABLE_WRITE	= 0xE5u,
	CMD_DISABLE_WRITE	= 0xDCu,
	CMD_SET_CHANNEL_A	= 0x5Au,
	CMD_SET_CHANNEL_B	= 0xA5u,
	CMD_RESET_CHIP		= 0x96u,
};

typedef hlw811x_error_t (*encoder_t)(uint8_t *buf, size_t bufsize,
		const uint8_t *data, size_t datalen, size_t *encoded_len);
typedef hlw811x_error_t (*decoder_t)(uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len, size_t *decoded_len);

typedef enum {
	CALC_TYPE_RMS,
	CALC_TYPE_POWER,
	CALC_TYPE_ENERGY,
} calc_type_t;

struct calc_param {
	hlw811x_reg_addr_t addr;
	uint16_t coeff; /* calibration coefficient */
	uint16_t ratio; /* resistor ratio */
	hlw811x_pga_gain_t pga; /* PGA gain */
	uint8_t mult; /* multiplier */
	int64_t resol; /* resolution */
};

struct hlw811x {
	hlw811x_interface_t iface;
	void *ctx;

	struct hlw811x_resistor_ratio ratio;
	struct hlw811x_coeff coeff;
	struct hlw811x_pga pga;
};

static int16_t convert_16bits_to_int16(const uint8_t buf[2])
{
#if defined(HLW811X_BIG_ENDIAN)
	return (int16_t)((buf[1] << 8) | buf[0]);
#else
	return (int16_t)((buf[0] << 8) | buf[1]);
#endif
}

static int32_t convert_24bits_to_int32(const uint8_t buf[3])
{
#if defined(HLW811X_BIG_ENDIAN)
	return (int32_t)
		((buf[2] << 16) | (buf[1] << 8) | buf[0]);
#else
	return (int32_t)
		((buf[0] << 16) | (buf[1] << 8) | buf[2]);
#endif
}

static int32_t convert_32bits_to_int32(const uint8_t buf[4])
{
#if defined(HLW811X_BIG_ENDIAN)
	return (int32_t)
		((buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0]);
#else
	return (int32_t)
		((buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]);
#endif
}

static int32_t fix_bit24_sign(int32_t val)
{
	if (val & (1 << 23)) {
		val = (int32_t)((uint32_t)val & ((1U << 23) - 1)) * -1;
	}
	return val;
}

static uint16_t convert_float_to_uint16_centi(float val)
{
	return (uint16_t)(val * 100);
}

static uint8_t get_regval_from_intr(hlw811x_intr_t ints)
{
	switch (ints) {
	case HLW811X_INTR_PULSE_OUT_A:
		return 0;
	case HLW811X_INTR_PULSE_OUT_B:
		return 1;
	case HLW811X_INTR_B_LEAKAGE:
		return 2;
	case HLW811X_INTR_IRQ:
		return 3;
	case HLW811X_INTR_OVERLOAD:
		return 4;
	case HLW811X_INTR_ACTIVE_POWER_OVERFLOW_A:
		return 5;
	case HLW811X_INTR_ACTIVE_POWER_OVERFLOW_B:
		return 6;
	case HLW811X_INTR_INSTANTAENOUS_UPDATED:
		return 7;
	case HLW811X_INTR_AVERAGE_UPDATED:
		return 8;
	case HLW811X_INTR_ZERO_CROSSING_VOLTAGE:
		return 9;
	case HLW811X_INTR_ZERO_CROSSING_CURRENT_A:
		return 10;
	case HLW811X_INTR_ZERO_CROSSING_CURRENT_B:
		return 11;
	case HLW811X_INTR_OVER_VOLTAGE:
		return 12;
	case HLW811X_INTR_UNDER_VOLTAGE:
		return 13;
	case HLW811X_INTR_OVER_CURRENT_A:
		return 14;
	case HLW811X_INTR_OVER_CURRENT_B:
		return 15;
	default:
		return 0;
	}
}

static hlw811x_error_t encode_uart(uint8_t *buf, size_t bufsize,
		const uint8_t *data, size_t datalen, size_t *encoded_len)
{
	if (bufsize < datalen + 2) {
		HLW811X_ERROR("Buffer size is too small");
		return HLW811X_BUFFER_TOO_SMALL;
	}

	buf[0] = 0xA5;
	uint8_t chksum = buf[0];

	for (size_t i = 0; i < datalen; i++) {
		buf[i + 1] = data[i];
		chksum += data[i];
	}

	buf[datalen + 1] = ~chksum;
	*encoded_len = datalen + 2;

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t decode_uart(uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len, size_t *decoded_len)
{
	if (tx_len < 1) {
		HLW811X_ERROR("Invalid tx_len");
		return HLW811X_INVALID_PARAM;
	}

	if (rx_len < 3 || bufsize < rx_len - 1) {
		HLW811X_ERROR("Invalid rx_len");
		return HLW811X_INVALID_PARAM;
	}

	uint8_t chksum = ~tx[tx_len - 1];

	for (size_t i = 0; i < rx_len - 1; i++) {
		buf[i] = rx[i];
		chksum += rx[i];
	}

	if ((uint8_t)~chksum != rx[rx_len - 1]) {
		HLW811X_ERROR("chksum mismatch %x : %x", ~chksum, rx[rx_len-1]);
		return HLW811X_CHECKSUM_MISMATCH;
	}

	*decoded_len = rx_len - 1;

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t encode(hlw811x_interface_t iface,
		uint8_t *buf, size_t bufsize,
		const uint8_t *data, size_t datalen, size_t *len)
{
	encoder_t encoder = encode_uart;

	if (iface == HLW811X_UART) {
	} else {
		HLW811X_ERROR("Not implemented");
		return HLW811X_NOT_IMPLEMENTED;
	}

	return (*encoder)(buf, bufsize, data, datalen, len);

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t decode(hlw811x_interface_t iface,
		uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len, size_t *len)
{
	decoder_t decoder = decode_uart;

	if (iface == HLW811X_UART) {
	} else {
		HLW811X_ERROR("Not implemented");
		return HLW811X_NOT_IMPLEMENTED;
	}

	return (*decoder)(buf, bufsize, tx, tx_len, rx, rx_len, len);
}

static hlw811x_error_t encode_frame(hlw811x_interface_t iface,
		hlw811x_reg_addr_t addr,
		uint8_t *txbuf, size_t txbuf_len,
		const uint8_t *data, size_t datalen,
		size_t *frame_len)
{
	uint8_t payload[datalen + 1];

	if (datalen > 2 || (data == NULL && datalen != 0)) {
		HLW811X_ERROR("Invalid parameter: %d, %x", datalen, data);
		return HLW811X_INVALID_PARAM;
	}

	payload[0] = (uint8_t)addr;
	for (size_t i = 0; i < datalen; i++) {
		payload[i + 1] = data[i];
	}

	return encode(iface, txbuf, txbuf_len, payload, datalen+1, frame_len);
}

static hlw811x_error_t decode_frame(hlw811x_interface_t iface,
		uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len)
{
	size_t len;
	hlw811x_error_t err;

	if ((err = decode(iface, buf, bufsize, tx, tx_len, rx, rx_len, &len))
			!= HLW811X_ERROR_NONE) {
		return err;
	} else if (len != bufsize) {
		HLW811X_ERROR("decoder returned %d", len);
		return HLW811X_INCORRECT_RESPONSE;
	}

	return HLW811X_ERROR_NONE;
}

static void get_calc_param_rms(struct hlw811x *self,
		hlw811x_channel_t channel, struct calc_param *param)
{
	if (channel == HLW811X_CHANNEL_A) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_RMS_IA,
			.coeff = self->coeff.rms.A,
			.ratio = convert_float_to_uint16_centi(self->ratio.K1_A),
			.pga = self->pga.A,
			.resol = 1ll << 23,
		};
	} else if (channel == HLW811X_CHANNEL_B) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_RMS_IB,
			.coeff = self->coeff.rms.B,
			.ratio = convert_float_to_uint16_centi(self->ratio.K1_B),
			.pga = self->pga.B,
			.resol = 1ll << 23,
		};
	} else if (channel == HLW811X_CHANNEL_U) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_RMS_U,
			.coeff = self->coeff.rms.U,
			.ratio = convert_float_to_uint16_centi(self->ratio.K2),
			.pga = self->pga.U,
			.resol = 1ll << 22,
			.mult = 10,
		};
	} else {
		HLW811X_ERROR("Invalid channel: %d", channel);
	}
}

static void get_calc_param_power(struct hlw811x *self,
		hlw811x_channel_t channel, struct calc_param *param)
{
	if (channel == HLW811X_CHANNEL_A) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_POWER_PA,
			.coeff = self->coeff.power.A,
			.ratio = convert_float_to_uint16_centi(self->ratio.K1_A),
			.pga = self->pga.A,
		};
	} else if (channel == HLW811X_CHANNEL_B) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_POWER_PB,
			.coeff = self->coeff.power.B,
			.ratio = convert_float_to_uint16_centi(self->ratio.K1_B),
			.pga = self->pga.B,
		};
	} else if (channel == HLW811X_CHANNEL_U) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_POWER_S,
			.coeff = self->coeff.power.S,
			.ratio = convert_float_to_uint16_centi(self->ratio.K2),
			.pga = self->pga.U,
		};
	} else {
		HLW811X_ERROR("Invalid channel: %d", channel);
	}

	param->resol = 1ll << 31;
}

static void get_calc_param_energy(struct hlw811x *self,
		hlw811x_channel_t channel, struct calc_param *param)
{
	if (channel == HLW811X_CHANNEL_A) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_ENERGY_PA,
			.coeff = self->coeff.energy.A,
			.ratio = convert_float_to_uint16_centi(self->ratio.K1_A),
			.pga = self->pga.A,
		};
	} else if (channel == HLW811X_CHANNEL_B) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_ENERGY_PB,
			.coeff = self->coeff.energy.B,
			.ratio = convert_float_to_uint16_centi(self->ratio.K1_B),
			.pga = self->pga.B,
		};
	} else {
		HLW811X_ERROR("Invalid channel: %d", channel);
	}

	param->resol = 1ll << 29;
}

static void get_calc_param(struct hlw811x *self, hlw811x_channel_t channel,
		calc_type_t type, struct calc_param *param)
{
	memset(param, 0, sizeof(*param));

	if (type == CALC_TYPE_RMS) {
		get_calc_param_rms(self, channel, param);
	} else if (type == CALC_TYPE_POWER) {
		get_calc_param_power(self, channel, param);
	} else if (type == CALC_TYPE_ENERGY) {
		get_calc_param_energy(self, channel, param);
	} else {
		HLW811X_ERROR("Invalid type: %d", type);
	}
}

static hlw811x_error_t send_frame(const uint8_t *data, size_t datalen, void *ctx)
{
	int err;

	if ((err = hlw811x_ll_write(data, datalen, ctx)) < 0) {
		HLW811X_ERROR("hlw811x_ll_write() failed: %x", err);
		return HLW811X_IO_ERROR;
	}

	if ((size_t)err != datalen) {
		HLW811X_ERROR("tx len mismatch: %d != %d", err, datalen);
		return HLW811X_IO_MISSING_BYTES;
	}

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t write_cmd(struct hlw811x *self,
		hlw811x_reg_addr_t addr, const uint8_t *data, size_t datalen)
{
	uint8_t frame[datalen + 1/*addr*/ + 2/*header+chksum*/];
	size_t frame_len;
	hlw811x_error_t err;

	if ((err = encode_frame(self->iface, addr | 0x80u, frame, sizeof(frame),
				data, datalen, &frame_len))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	return send_frame(frame, frame_len, self->ctx);
}

static hlw811x_error_t reset_chip(struct hlw811x *self)
{
	const uint8_t cmd = CMD_RESET_CHIP;
	return write_cmd(self, HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t enable_write(struct hlw811x *self)
{
	const uint8_t cmd = CMD_ENABLE_WRITE;
	return write_cmd(self, HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t disable_write(struct hlw811x *self)
{
	const uint8_t cmd = CMD_DISABLE_WRITE;
	return write_cmd(self, HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t write_reg(struct hlw811x *self,
		hlw811x_reg_addr_t addr, const uint8_t *data, size_t datalen)
{
	hlw811x_error_t err;

	if ((err = enable_write(self)) != HLW811X_ERROR_NONE) {
		HLW811X_ERROR("enable_write() failed");
		return err;
	}

	if ((err = write_cmd(self, addr, data, datalen))
			!= HLW811X_ERROR_NONE) {
		disable_write(self);
		HLW811X_ERROR("write_cmd() failed");
		return err;
	}

	if ((err = disable_write(self)) != HLW811X_ERROR_NONE) {
		HLW811X_ERROR("disable_write() failed");
	}

	return err;
}

static hlw811x_error_t write_reg16(struct hlw811x *self,
		hlw811x_reg_addr_t addr, const uint16_t val)
{
	uint8_t tmp[2] = { (uint8_t)(val >> 8), (uint8_t)val };
	return write_reg(self, addr, tmp, sizeof(tmp));
}

static hlw811x_error_t read_reg(struct hlw811x *self,
		hlw811x_reg_addr_t addr, uint8_t *buf, size_t bytes_to_read)
{
	hlw811x_error_t err;
	int bytes_received;
	uint8_t rx[bytes_to_read + 1];
	uint8_t tx[3];
	size_t encoded_len;
	size_t tx_len;

	if ((err = encode_frame(self->iface, addr, tx, sizeof(tx),
			0, 0, &encoded_len)) != HLW811X_ERROR_NONE) {
		return err;
	}

	tx_len = encoded_len;
	if (self->iface == HLW811X_UART) {
		tx_len -= 1; /* do not send chksum */
	}

	if ((err = send_frame(tx, tx_len, self->ctx)) != HLW811X_ERROR_NONE) {
		return err;
	}

	if ((bytes_received = hlw811x_ll_read(rx, sizeof(rx), self->ctx)) < 0) {
		HLW811X_ERROR("hlw811x_ll_read() failed");
		return HLW811X_IO_ERROR;
	} else if (bytes_received == 0) {
		return HLW811X_NO_RESPONSE;
	}

	err = decode_frame(self->iface, buf, bytes_to_read, tx, encoded_len,
			rx, (size_t)bytes_received);
	return err;
}

static hlw811x_error_t read_reg16(struct hlw811x *self,
		hlw811x_reg_addr_t addr, uint16_t *reg)
{
	uint8_t buf[2];
	hlw811x_error_t err;

	if ((err = read_reg(self, addr, buf, sizeof(buf)))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*reg = (uint16_t)convert_16bits_to_int16(buf);

	return err;
}

static hlw811x_error_t select_channel(struct hlw811x *self,
		hlw811x_channel_t channel)
{
	uint8_t cmd;

	switch (channel) {
	case HLW811X_CHANNEL_A:
		cmd = CMD_SET_CHANNEL_A;
		break;
	case HLW811X_CHANNEL_B:
		cmd = CMD_SET_CHANNEL_B;
		break;
	default:
		HLW811X_ERROR("Invalid channel: %d", channel);
		return HLW811X_INVALID_PARAM;
	}

	return write_cmd(self, HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t read_current_channel(struct hlw811x *self,
		hlw811x_channel_t *channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_STATUS, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*channel = (hlw811x_channel_t)(((reg >> 21) & 0x01) + 1);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_write_reg(struct hlw811x *self,
		hlw811x_reg_addr_t addr, const uint8_t *data, size_t datalen)
{
	return write_reg(self, addr, data, datalen);
}

hlw811x_error_t hlw811x_read_reg(struct hlw811x *self,
		hlw811x_reg_addr_t addr, uint8_t *buf, size_t bufsize)
{
	return read_reg(self, addr, buf, bufsize);
}

hlw811x_error_t hlw811x_set_active_power_calc_mode(struct hlw811x *self,
		hlw811x_active_power_mode_t mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(3 << 10); /* clear Pmode bit */
	reg |= (mode << 10);

	return write_reg16(self, HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_get_active_power_calc_mode(struct hlw811x *self,
		hlw811x_active_power_mode_t *mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*mode = (hlw811x_active_power_mode_t)((reg >> 10) & 0x03);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_set_rms_calc_mode(struct hlw811x *self,
		hlw811x_rms_mode_t mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(3 << 9); /* clear DC_MODE bit */
	reg |= (mode << 9);

	return write_reg16(self, HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_get_rms_calc_mode(struct hlw811x *self,
		hlw811x_rms_mode_t *mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*mode = (hlw811x_rms_mode_t)((reg >> 9) & 0x03);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_enable_pulse(struct hlw811x *self,
		hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_A) {
		reg |= 1 << 0; /* PARUN */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg |= 1 << 1; /* PBRUN */
	}

	return write_reg16(self, HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_disable_pulse(struct hlw811x *self,
		hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_A) {
		reg &= ~(1 << 0); /* PARUN */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg &= ~(1 << 1); /* PBRUN */
	}

	return write_reg16(self, HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_set_data_update_frequency(struct hlw811x *self,
		hlw811x_data_update_freq_t freq)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(3 << 8); /* clear DUP bit */
	reg |= (freq << 8);

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_get_data_update_frequency(struct hlw811x *self,
		hlw811x_data_update_freq_t *freq)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*freq = (hlw811x_data_update_freq_t)((reg >> 8) & 0x03);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_set_channel_b_mode(struct hlw811x *self,
		hlw811x_channel_b_mode_t mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 7); /* clear CHS_IB bit */
	reg |= (mode << 7);

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_get_channel_b_mode(struct hlw811x *self,
		hlw811x_channel_b_mode_t *mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*mode = (hlw811x_channel_b_mode_t)((reg >> 7) & 0x01);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_set_zerocrossing_mode(struct hlw811x *self,
		hlw811x_zerocrossing_mode_t mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(3 << 7); /* clear ZXDx bits */
	reg |= (mode << 7);

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_get_zerocrossing_mode(struct hlw811x *self,
		hlw811x_zerocrossing_mode_t *mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*mode = (hlw811x_zerocrossing_mode_t)((reg >> 7) & 0x03);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_enable_waveform(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= 1 << 5; /* WaveEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_disable_waveform(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 5); /* WaveEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_enable_zerocrossing(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= 1 << 2; /* ZxEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_disable_zerocrossing(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 2); /* ZxEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_enable_power_factor(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= 1 << 6; /* PfactorEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_disable_power_factor(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 6); /* PfactorEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_enable_energy_clearance(struct hlw811x *self,
		hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_A) {
		reg &= ~(1 << 10); /* EPA_CA */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg &= ~(1 << 11); /* EPA_CB */
	}

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_disable_energy_clearance(struct hlw811x *self,
		hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_A) {
		reg |= 1 << 10; /* EPA_CA */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg |= 1 << 11; /* EPA_CB */
	}

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_enable_hpf(struct hlw811x *self,
		hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_U) {
		reg &= ~(1 << 4); /* HPFUOFF */
	}
	if (channel & HLW811X_CHANNEL_A) {
		reg &= ~(1 << 5); /* HPFAOFF */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg &= ~(1 << 6); /* HPFBOFF */
	}

	return write_reg16(self, HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_disable_hpf(struct hlw811x *self,
		hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_U) {
		reg |= 1 << 4; /* HPFUOFF */
	}
	if (channel & HLW811X_CHANNEL_A) {
		reg |= 1 << 5; /* HPFAOFF */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg |= 1 << 6; /* HPFBOFF */
	}

	return write_reg16(self, HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_enable_b_channel_comparator(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 12); /* comp_off */

	return write_reg16(self, HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_disable_b_channel_comparator(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= 1 << 12; /* comp_off */

	return write_reg16(self, HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_enable_temperature_sensor(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= 1 << 13; /* tensor_en */

	return write_reg16(self, HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_disable_temperature_sensor(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 13); /* tensor_en */

	return write_reg16(self, HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_enable_peak_detection(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= 1 << 1; /* PeakEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_disable_peak_detection(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 1); /* PeakEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_enable_overload_detection(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= 1 << 3; /* OverEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_disable_overload_detection(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 3); /* OverEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_enable_voltage_drop_detection(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= 1 << 4; /* SAGEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_disable_voltage_drop_detection(struct hlw811x *self)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 4); /* SAGEN */

	return write_reg16(self, HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_enable_interrupt(struct hlw811x *self,
		hlw811x_intr_t ints)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_IE, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= ints;

	return write_reg16(self, HLW811X_REG_IE, reg);
}

hlw811x_error_t hlw811x_disable_interrupt(struct hlw811x *self,
		hlw811x_intr_t ints)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_IE, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~ints;

	return write_reg16(self, HLW811X_REG_IE, reg);
}

hlw811x_error_t hlw811x_set_interrupt_mode(struct hlw811x *self,
		hlw811x_intr_t int1, hlw811x_intr_t int2)
{
	hlw811x_error_t err;
	uint16_t reg;

	if (((int1 - 1) & int1) || ((int2 - 1) & int2)) {
		return HLW811X_INVALID_PARAM;
	}

	if ((err = read_reg16(self, HLW811X_REG_INT, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(0xf << 0); /* clear P1sel bits */
	reg &= ~(0xf << 4); /* clear P2sel bits */

	reg |= (get_regval_from_intr(int1) << 0)
		| (get_regval_from_intr(int2) << 4);

	return write_reg16(self, HLW811X_REG_INT, reg);
}

hlw811x_error_t hlw811x_get_interrupt(struct hlw811x *self,
		hlw811x_intr_t *ints)
{
	hlw811x_error_t err;
	uint8_t reg[2];

	if ((err = read_reg(self, HLW811X_REG_IF, reg, 2))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*ints = (hlw811x_intr_t)(((uint16_t)reg[0] << 8) | reg[1]);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_get_interrupt_ext(struct hlw811x *self,
		hlw811x_intr_t *ints)
{
	hlw811x_error_t err;
	uint8_t reg[2];

	if ((err = read_reg(self, HLW811X_REG_RIF, reg, 2))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*ints = (hlw811x_intr_t)(((uint16_t)reg[0] << 8) | reg[1]);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_get_rms(struct hlw811x *self,
		hlw811x_channel_t channel, int32_t *milliunit)
{
	hlw811x_error_t err;
	uint8_t buf[3];
	struct calc_param param;

	get_calc_param(self, channel, CALC_TYPE_RMS, &param);

	if ((err = read_reg(self, param.addr, buf, sizeof(buf)))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	const int32_t raw = convert_24bits_to_int32(buf);
	if (raw & (1 << 23)) {
		return HLW811X_INVALID_DATA;
	}

	/* Multiplied by 1000 first and then divide by 10 to avoid losing
	 * significant digits during the calculation. */
	int64_t val = ((int64_t)raw * param.coeff * 1000)
		/ (param.ratio * param.resol);

	if (channel == HLW811X_CHANNEL_U) {
		val = val * param.mult / (1 << param.pga) / 10;
	} else {
		val = val * (16 >> param.pga) / 10;
	}

	*milliunit = (int32_t)val;

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_get_power(struct hlw811x *self,
		hlw811x_channel_t channel, int32_t *milliwatt)
{
	hlw811x_error_t err;
	uint8_t buf[4];
	struct calc_param param;

	get_calc_param(self, channel, CALC_TYPE_POWER, &param);

	if ((err = read_reg(self, param.addr, buf, sizeof(buf)))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	const int32_t raw = convert_32bits_to_int32(buf);
	uint16_t K2 = convert_float_to_uint16_centi(self->ratio.K2);
	hlw811x_pga_gain_t k2_pga = self->pga.U;

	if (channel == HLW811X_CHANNEL_U) {
		hlw811x_channel_t ch;
#if 0
		if ((err = read_current_channel(&ch)) != HLW811X_ERROR_NONE) {
			return err;
		}
#else
		ch = HLW811X_CHANNEL_A;
#endif

		if (ch == HLW811X_CHANNEL_A) {
			K2 = convert_float_to_uint16_centi(self->ratio.K1_A);
			k2_pga = self->pga.A;
		} else if (ch == HLW811X_CHANNEL_B) {
			K2 = convert_float_to_uint16_centi(self->ratio.K1_B);
			k2_pga = self->pga.B;
		}
	}

	const int64_t div = param.ratio * K2;
	int32_t pga = 16l >> (param.pga + k2_pga);
	int64_t val = (int64_t)raw * param.coeff * pga;
	val = val * 1000/*milli*/ / param.resol * 10000/*K1,K2 scale*/;
	val = val / div;

	*milliwatt = (int32_t)val;

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_get_energy(struct hlw811x *self,
		hlw811x_channel_t channel, int32_t *Wh)
{
	hlw811x_error_t err;
	uint8_t buf[3];
	struct calc_param param;

	get_calc_param(self, channel, CALC_TYPE_ENERGY, &param);

	if ((err = read_reg(self, param.addr, buf, sizeof(buf)))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	const int32_t raw = convert_24bits_to_int32(buf);
	const int32_t pga = (1 << self->pga.U) * (1 << param.pga);
	const uint16_t K2 = convert_float_to_uint16_centi(self->ratio.K2);
	const int64_t div = param.ratio * K2 * 4096;
	int64_t val = (int64_t)raw * param.coeff * self->coeff.hfconst;
	val = val * 100/*watt unit*/
		/ param.resol * 10000/*K1,K2 scaling*/ * 10/*watt unit*/ * pga;
	val = val / div;

	*Wh = (int32_t)val;

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_get_frequency(struct hlw811x *self, int32_t *centihertz)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_FREQUENCY_L_LINE, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*centihertz = 0;
	if (reg) {
		*centihertz = HLW811X_MCLK * 100 / 8 / reg;
	}

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_get_power_factor(struct hlw811x *self, int32_t *centi)
{
	hlw811x_error_t err;
	uint8_t buf[3];

	if ((err = read_reg(self, HLW811X_REG_POWER_FACTOR, buf, sizeof(buf)))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	int32_t val = fix_bit24_sign(convert_24bits_to_int32(buf));
	*centi = val * 100 / ((1 << 23) - 1);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_get_phase_angle(struct hlw811x *self,
		int32_t *centidegree, hlw811x_line_freq_t freq)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_ANGLE, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (freq == HLW811X_LINE_FREQ_50HZ) {
		*centidegree = (int32_t)reg * 805 / 100;
	} else if (freq == HLW811X_LINE_FREQ_60HZ) {
		*centidegree = (int32_t)reg * 965 / 100;
	} else {
		return HLW811X_INVALID_PARAM;
	}

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_select_channel(struct hlw811x *self,
		hlw811x_channel_t channel)
{
	return select_channel(self, channel);
}

hlw811x_error_t hlw811x_read_current_channel(struct hlw811x *self,
		hlw811x_channel_t *channel)
{
	return read_current_channel(self, channel);
}

hlw811x_error_t hlw811x_read_coeff(struct hlw811x *self,
		struct hlw811x_coeff *coeff)
{
	hlw811x_error_t err;
	uint16_t chksum;

	err = read_reg16(self, HLW811X_REG_PULSE_FREQ, &coeff->hfconst);
	err |= read_reg16(self, HLW811X_REG_RMS_IA_COEFF, &coeff->rms.A);
	err |= read_reg16(self, HLW811X_REG_RMS_IB_COEFF, &coeff->rms.B);
	err |= read_reg16(self, HLW811X_REG_RMS_U_COEFF, &coeff->rms.U);
	err |= read_reg16(self, HLW811X_REG_POWER_A_COEFF, &coeff->power.A);
	err |= read_reg16(self, HLW811X_REG_POWER_B_COEFF, &coeff->power.B);
	err |= read_reg16(self, HLW811X_REG_POWER_S_COEFF, &coeff->power.S);
	err |= read_reg16(self, HLW811X_REG_ENERGY_A_COEFF, &coeff->energy.A);
	err |= read_reg16(self, HLW811X_REG_ENERGY_B_COEFF, &coeff->energy.B);

	if (err != HLW811X_ERROR_NONE) {
		return err;
	}

	if ((err = read_reg16(self, HLW811X_REG_COEFF_CHKSUM, &chksum))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	const uint16_t chksum_calc = (uint16_t)~(0xFFFFu
			+ coeff->rms.A + coeff->rms.B + coeff->rms.U
			+ coeff->power.A + coeff->power.B + coeff->power.S
			+ coeff->energy.A + coeff->energy.B);

	if (chksum != chksum_calc) {
		return HLW811X_CHECKSUM_MISMATCH;
	}

	memcpy(&self->coeff, coeff, sizeof(self->coeff));

	HLW811X_DEBUG("Coefficients: HFConst=%d, "
			"RMS_A=%d, RMS_B=%d, RMS_U=%d, "
			"Power_A=%d, Power_B=%d, Power_S=%d, "
			"Energy_A=%d, Energy_B=%d",
			coeff->hfconst,
			coeff->rms.A, coeff->rms.B, coeff->rms.U,
			coeff->power.A, coeff->power.B, coeff->power.S,
			coeff->energy.A, coeff->energy.B);

	return err;
}

void hlw811x_set_resistor_ratio(struct hlw811x *self,
		const struct hlw811x_resistor_ratio *ratio)
{
	memcpy(&self->ratio, ratio, sizeof(self->ratio));
	HLW811X_INFO("Resistor ratio set: K1_A=%d, K1_B=%d, K2=%d",
			ratio->K1_A, ratio->K1_B, ratio->K2);
}

void hlw811x_get_resistor_ratio(struct hlw811x *self,
		struct hlw811x_resistor_ratio *ratio)
{
	memcpy(ratio, &self->ratio, sizeof(self->ratio));
}

hlw811x_error_t hlw811x_set_pga(struct hlw811x *self,
		const struct hlw811x_pga *pga)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_SYS_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(0x1FF << 0); /* clear PGA bits */
	reg |= (pga->A << 0) | (pga->U << 3) | (pga->B << 6);

	if ((err = write_reg16(self, HLW811X_REG_SYS_CTRL, reg))
			== HLW811X_ERROR_NONE) {
		memcpy(&self->pga, pga, sizeof(self->pga));
		HLW811X_INFO("PGA set: A=%d, U=%d, B=%d",
				pga->A, pga->U, pga->B);
	}

	return err;
}

hlw811x_error_t hlw811x_get_pga(struct hlw811x *self, struct hlw811x_pga *pga)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_SYS_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	pga->A = (reg >> 0) & 0x07; /* PGAIA */
	pga->U = (reg >> 3) & 0x07; /* PGAU */
	pga->B = (reg >> 6) & 0x07; /* PGAIB */

	memcpy(&self->pga, pga, sizeof(self->pga));

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_enable_channel(struct hlw811x *self,
		hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_SYS_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_A) {
		reg |= 1 << 9; /* ADC1ON */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg |= 1 << 10; /* ADC2ON */
	}
	if (channel & HLW811X_CHANNEL_U) {
		reg |= 1 << 11; /* ADC3ON */
	}

	if ((err = write_reg16(self, HLW811X_REG_SYS_CTRL, reg))
			== HLW811X_ERROR_NONE) {
		HLW811X_INFO("Channel enabled: %d", channel);
	}

	return err;
}

hlw811x_error_t hlw811x_disable_channel(struct hlw811x *self,
		hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(self, HLW811X_REG_SYS_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_A) {
		reg &= ~(1 << 9); /* ADC1ON */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg &= ~(1 << 10); /* ADC2ON */
	}
	if (channel & HLW811X_CHANNEL_U) {
		reg &= ~(1 << 11); /* ADC3ON */
	}

	if ((err = write_reg16(self, HLW811X_REG_SYS_CTRL, reg))
			== HLW811X_ERROR_NONE) {
		HLW811X_INFO("Channel disabled: %d", channel);
	}

	return err;
}

hlw811x_error_t hlw811x_reset(struct hlw811x *self)
{
	HLW811X_INFO("Resetting HLW811X chip");
	return reset_chip(self);
}

struct hlw811x *hlw811x_create(hlw811x_interface_t interface, void *ctx)
{
	struct hlw811x *hlw811x;

	if ((hlw811x = malloc(sizeof(struct hlw811x))) == NULL) {
		return NULL;
	}

	memset(hlw811x, 0, sizeof(struct hlw811x));
	hlw811x->iface = interface;
	hlw811x->ctx = ctx;

	return hlw811x;
}

void hlw811x_destroy(struct hlw811x *hlw811x)
{
	free(hlw811x);
}
