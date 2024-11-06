/*
 * SPDX-FileCopyrightText: 2024 Kyunghwan Kwon <k@libmcu.org>
 *
 * SPDX-License-Identifier: MIT
 */

#include "hlw811x.h"
#include "hlw811x_overrides.h"

#include <string.h>

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

	struct hlw811x_resistor_ratio ratio;
	struct hlw811x_coeff coeff;
	struct hlw811x_pga pga;
};

static struct hlw811x m;

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

static hlw811x_error_t encode(uint8_t *buf, size_t bufsize,
		const uint8_t *data, size_t datalen, size_t *len)
{
	encoder_t encoder = encode_uart;

	if (m.iface == HLW811X_UART) {
	} else {
		HLW811X_ERROR("Not implemented");
		return HLW811X_NOT_IMPLEMENTED;
	}

	return (*encoder)(buf, bufsize, data, datalen, len);

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t decode(uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len, size_t *len)
{
	decoder_t decoder = decode_uart;

	if (m.iface == HLW811X_UART) {
	} else {
		HLW811X_ERROR("Not implemented");
		return HLW811X_NOT_IMPLEMENTED;
	}

	return (*decoder)(buf, bufsize, tx, tx_len, rx, rx_len, len);
}

static hlw811x_error_t encode_frame(hlw811x_reg_addr_t addr,
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

	return encode(txbuf, txbuf_len, payload, datalen+1, frame_len);
}

static hlw811x_error_t decode_frame(uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len)
{
	size_t len;
	hlw811x_error_t err;

	if ((err = decode(buf, bufsize, tx, tx_len, rx, rx_len, &len))
			!= HLW811X_ERROR_NONE) {
		return err;
	} else if (len != bufsize) {
		HLW811X_ERROR("decoder returned %d", len);
		return HLW811X_INCORRECT_RESPONSE;
	}

	return HLW811X_ERROR_NONE;
}

static void get_calc_param_rms(hlw811x_channel_t channel,
		struct calc_param *param)
{
	if (channel == HLW811X_CHANNEL_A) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_RMS_IA,
			.coeff = m.coeff.rms.A,
			.ratio = convert_float_to_uint16_centi(m.ratio.K1_A),
			.pga = m.pga.A,
			.resol = 1ll << 23,
		};
	} else if (channel == HLW811X_CHANNEL_B) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_RMS_IB,
			.coeff = m.coeff.rms.B,
			.ratio = convert_float_to_uint16_centi(m.ratio.K1_B),
			.pga = m.pga.B,
			.resol = 1ll << 23,
		};
	} else if (channel == HLW811X_CHANNEL_U) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_RMS_U,
			.coeff = m.coeff.rms.U,
			.ratio = convert_float_to_uint16_centi(m.ratio.K2),
			.pga = m.pga.U,
			.resol = 1ll << 22,
			.mult = 10,
		};
	} else {
		HLW811X_ERROR("Invalid channel: %d", channel);
	}
}

static void get_calc_param_power(hlw811x_channel_t channel,
		struct calc_param *param)
{
	if (channel == HLW811X_CHANNEL_A) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_POWER_PA,
			.coeff = m.coeff.power.A,
			.ratio = convert_float_to_uint16_centi(m.ratio.K1_A),
			.pga = m.pga.A,
		};
	} else if (channel == HLW811X_CHANNEL_B) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_POWER_PB,
			.coeff = m.coeff.power.B,
			.ratio = convert_float_to_uint16_centi(m.ratio.K1_B),
			.pga = m.pga.B,
		};
	} else if (channel == HLW811X_CHANNEL_U) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_POWER_S,
			.coeff = m.coeff.power.S,
			.ratio = convert_float_to_uint16_centi(m.ratio.K2),
			.pga = m.pga.U,
		};
	} else {
		HLW811X_ERROR("Invalid channel: %d", channel);
	}

	param->resol = 1ll << 31;
}

static void get_calc_param_energy(hlw811x_channel_t channel,
		struct calc_param *param)
{
	if (channel == HLW811X_CHANNEL_A) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_ENERGY_PA,
			.coeff = m.coeff.energy.A,
			.ratio = convert_float_to_uint16_centi(m.ratio.K1_A),
			.pga = m.pga.A,
		};
	} else if (channel == HLW811X_CHANNEL_B) {
		*param = (struct calc_param) {
			.addr = HLW811X_REG_ENERGY_PB,
			.coeff = m.coeff.energy.B,
			.ratio = convert_float_to_uint16_centi(m.ratio.K1_B),
			.pga = m.pga.B,
		};
	} else {
		HLW811X_ERROR("Invalid channel: %d", channel);
	}

	param->resol = 1ll << 29;
}

static void get_calc_param(hlw811x_channel_t channel,
		calc_type_t type, struct calc_param *param)
{
	memset(param, 0, sizeof(*param));

	if (type == CALC_TYPE_RMS) {
		get_calc_param_rms(channel, param);
	} else if (type == CALC_TYPE_POWER) {
		get_calc_param_power(channel, param);
	} else if (type == CALC_TYPE_ENERGY) {
		get_calc_param_energy(channel, param);
	} else {
		HLW811X_ERROR("Invalid type: %d", type);
	}
}

static hlw811x_error_t send_frame(const uint8_t *data, size_t datalen)
{
	int err;

	if ((err = hlw811x_ll_write(data, datalen)) < 0) {
		HLW811X_ERROR("hlw811x_ll_write() failed: %x", err);
		return HLW811X_IO_ERROR;
	}

	if ((size_t)err != datalen) {
		HLW811X_ERROR("tx len mismatch: %d != %d", err, datalen);
		return HLW811X_IO_MISSING_BYTES;
	}

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t write_cmd(hlw811x_reg_addr_t addr,
		const uint8_t *data, size_t datalen)
{
	uint8_t frame[datalen + 1/*addr*/ + 2/*header+chksum*/];
	size_t frame_len;
	hlw811x_error_t err;

	if ((err = encode_frame(addr | 0x80u, frame, sizeof(frame),
				data, datalen, &frame_len))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	return send_frame(frame, frame_len);
}

static hlw811x_error_t reset_chip(void)
{
	const uint8_t cmd = CMD_RESET_CHIP;
	return write_cmd(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t enable_write(void)
{
	const uint8_t cmd = CMD_ENABLE_WRITE;
	return write_cmd(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t disable_write(void)
{
	const uint8_t cmd = CMD_DISABLE_WRITE;
	return write_cmd(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t write_reg(hlw811x_reg_addr_t addr,
		const uint8_t *data, size_t datalen)
{
	hlw811x_error_t err;

	if ((err = enable_write()) != HLW811X_ERROR_NONE) {
		HLW811X_ERROR("enable_write() failed");
		return err;
	}

	if ((err = write_cmd(addr, data, datalen)) != HLW811X_ERROR_NONE) {
		disable_write();
		HLW811X_ERROR("write_cmd() failed");
		return err;
	}

	if ((err = disable_write()) != HLW811X_ERROR_NONE) {
		HLW811X_ERROR("disable_write() failed");
	}

	return err;
}

static hlw811x_error_t write_reg16(hlw811x_reg_addr_t addr, const uint16_t val)
{
	uint8_t tmp[2] = { (uint8_t)(val >> 8), (uint8_t)val };
	return write_reg(addr, tmp, sizeof(tmp));
}

static hlw811x_error_t read_reg(hlw811x_reg_addr_t addr,
		uint8_t *buf, size_t bytes_to_read)
{
	hlw811x_error_t err;
	int bytes_received;
	uint8_t rx[bytes_to_read + 1];
	uint8_t tx[3];
	size_t encoded_len;
	size_t tx_len;

	if ((err = encode_frame(addr, tx, sizeof(tx), 0, 0, &encoded_len))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	tx_len = encoded_len;
	if (m.iface == HLW811X_UART) {
		tx_len -= 1; /* do not send chksum */
	}

	if ((err = send_frame(tx, tx_len)) != HLW811X_ERROR_NONE) {
		return err;
	}

	if ((bytes_received = hlw811x_ll_read(rx, sizeof(rx))) < 0) {
		HLW811X_ERROR("hlw811x_ll_read() failed");
		return HLW811X_IO_ERROR;
	} else if (bytes_received == 0) {
		return HLW811X_NO_RESPONSE;
	}

	err = decode_frame(buf, bytes_to_read, tx, encoded_len,
			rx, (size_t)bytes_received);
	return err;
}

static hlw811x_error_t read_reg16(hlw811x_reg_addr_t addr, uint16_t *reg)
{
	uint8_t buf[2];
	hlw811x_error_t err;

	if ((err = read_reg(addr, buf, sizeof(buf))) != HLW811X_ERROR_NONE) {
		return err;
	}

	*reg = (uint16_t)convert_16bits_to_int16(buf);

	return err;
}

static hlw811x_error_t select_channel(hlw811x_channel_t channel)
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

	return write_cmd(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t read_current_channel(hlw811x_channel_t *channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_STATUS, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*channel = (hlw811x_channel_t)(((reg >> 21) & 0x01) + 1);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_write_reg(hlw811x_reg_addr_t addr,
		const uint8_t *data, size_t datalen)
{
	return write_reg(addr, data, datalen);
}

hlw811x_error_t hlw811x_read_reg(hlw811x_reg_addr_t addr,
		uint8_t *buf, size_t bufsize)
{
	return read_reg(addr, buf, bufsize);
}

hlw811x_error_t hlw811x_set_active_power_calc_mode(hlw811x_active_power_mode_t
		mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(3 << 10); /* clear Pmode bit */
	reg |= (mode << 10);

	return write_reg16(HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_get_active_power_calc_mode(hlw811x_active_power_mode_t
		*mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*mode = (hlw811x_active_power_mode_t)((reg >> 10) & 0x03);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_set_rms_calc_mode(hlw811x_rms_mode_t mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(3 << 9); /* clear DC_MODE bit */
	reg |= (mode << 9);

	return write_reg16(HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_get_rms_calc_mode(hlw811x_rms_mode_t *mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*mode = (hlw811x_rms_mode_t)((reg >> 9) & 0x03);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_enable_pulse(hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_A) {
		reg |= 1 << 0; /* PARUN */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg |= 1 << 1; /* PBRUN */
	}

	return write_reg16(HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_disable_pulse(hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_A) {
		reg &= ~(1 << 0); /* PARUN */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg &= ~(1 << 1); /* PBRUN */
	}

	return write_reg16(HLW811X_REG_METER_CTRL, reg);
}

hlw811x_error_t hlw811x_set_data_update_frequency(hlw811x_data_update_freq_t
		freq)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(3 << 8); /* clear DUP bit */
	reg |= (freq << 8);

	return write_reg16(HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_get_data_update_frequency(hlw811x_data_update_freq_t
		*freq)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*freq = (hlw811x_data_update_freq_t)((reg >> 8) & 0x03);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_set_channel_b_mode(hlw811x_channel_b_mode_t mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 7); /* clear CHS_IB bit */
	reg |= (mode << 7);

	return write_reg16(HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_get_channel_b_mode(hlw811x_channel_b_mode_t *mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*mode = (hlw811x_channel_b_mode_t)((reg >> 7) & 0x01);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_set_zerocrossing_mode(hlw811x_zerocrossing_mode_t mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(3 << 7); /* clear ZXDx bits */
	reg |= (mode << 7);

	return write_reg16(HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_get_zerocrossing_mode(hlw811x_zerocrossing_mode_t
		*mode)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*mode = (hlw811x_zerocrossing_mode_t)((reg >> 7) & 0x03);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_enable_waveform(void)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= 1 << 5; /* WaveEN */

	return write_reg16(HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_disable_waveform(void)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 5); /* WaveEN */

	return write_reg16(HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_enable_zerocrossing(void)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= 1 << 2; /* ZxEN */

	return write_reg16(HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_disable_zerocrossing(void)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 2); /* ZxEN */

	return write_reg16(HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_enable_power_factor(void)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg |= 1 << 6; /* PfactorEN */

	return write_reg16(HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_disable_power_factor(void)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(1 << 6); /* PfactorEN */

	return write_reg16(HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_enable_energy_clearance(hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_A) {
		reg &= ~(1 << 10); /* EPA_CA */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg &= ~(1 << 11); /* EPA_CB */
	}

	return write_reg16(HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_disable_energy_clearance(hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_METER_CTRL_2, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	if (channel & HLW811X_CHANNEL_A) {
		reg |= 1 << 10; /* EPA_CA */
	}
	if (channel & HLW811X_CHANNEL_B) {
		reg |= 1 << 11; /* EPA_CB */
	}

	return write_reg16(HLW811X_REG_METER_CTRL_2, reg);
}

hlw811x_error_t hlw811x_get_rms(hlw811x_channel_t channel, int32_t *milliunit)
{
	hlw811x_error_t err;
	uint8_t buf[3];
	struct calc_param param;

	get_calc_param(channel, CALC_TYPE_RMS, &param);

	if ((err = read_reg(param.addr, buf, sizeof(buf)))
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

hlw811x_error_t hlw811x_get_power(hlw811x_channel_t channel, int32_t *milliwatt)
{
	hlw811x_error_t err;
	uint8_t buf[4];
	struct calc_param param;

	get_calc_param(channel, CALC_TYPE_POWER, &param);

	if ((err = read_reg(param.addr, buf, sizeof(buf)))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	const int32_t raw = convert_32bits_to_int32(buf);
	uint16_t K2 = convert_float_to_uint16_centi(m.ratio.K2);
	hlw811x_pga_gain_t k2_pga = m.pga.U;

	if (channel == HLW811X_CHANNEL_U) {
		hlw811x_channel_t ch;
		if ((err = read_current_channel(&ch)) != HLW811X_ERROR_NONE) {
			return err;
		}

		if (ch == HLW811X_CHANNEL_A) {
			K2 = convert_float_to_uint16_centi(m.ratio.K1_A);
			k2_pga = m.pga.A;
		} else if (ch == HLW811X_CHANNEL_B) {
			K2 = convert_float_to_uint16_centi(m.ratio.K1_B);
			k2_pga = m.pga.B;
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

hlw811x_error_t hlw811x_get_energy(hlw811x_channel_t channel, int32_t *Wh)
{
	hlw811x_error_t err;
	uint8_t buf[3];
	struct calc_param param;

	get_calc_param(channel, CALC_TYPE_ENERGY, &param);

	if ((err = read_reg(param.addr, buf, sizeof(buf)))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	const int32_t raw = convert_24bits_to_int32(buf);
	const int32_t pga = (1 << m.pga.U) * (1 << param.pga);
	const uint16_t K2 = convert_float_to_uint16_centi(m.ratio.K2);
	const int64_t div = param.ratio * K2 * 4096;
	int64_t val = (int64_t)raw * param.coeff * m.coeff.hfconst;
	val = val * 100/*watt unit*/
		/ param.resol * 10000/*K1,K2 scaling*/ * 10/*watt unit*/ * pga;
	val = val / div;

	*Wh = (int32_t)val;

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_get_frequency(int32_t *centihertz)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_FREQUENCY_L_LINE, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	*centihertz = 0;
	if (reg) {
		*centihertz = HLW811X_MCLK * 100 / 8 / reg;
	}

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_get_power_factor(int32_t *centiunit)
{
	hlw811x_error_t err;
	uint8_t buf[3];

	if ((err = read_reg(HLW811X_REG_POWER_FACTOR, buf, sizeof(buf)))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	int32_t val = fix_bit24_sign(convert_24bits_to_int32(buf));
	*centiunit = val * 100 / ((1 << 23) - 1);

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_get_phase_angle(int32_t *centidegree,
		hlw811x_line_freq_t freq)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_ANGLE, &reg)) != HLW811X_ERROR_NONE) {
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

hlw811x_error_t hlw811x_select_channel(hlw811x_channel_t channel)
{
	return select_channel(channel);
}

hlw811x_error_t hlw811x_read_current_channel(hlw811x_channel_t *channel)
{
	return read_current_channel(channel);
}

hlw811x_error_t hlw811x_read_coeff(struct hlw811x_coeff *coeff)
{
	hlw811x_error_t err;
	uint16_t chksum;

	err = read_reg16(HLW811X_REG_PULSE_FREQ, &coeff->hfconst);
	err |= read_reg16(HLW811X_REG_RMS_IA_COEFF, &coeff->rms.A);
	err |= read_reg16(HLW811X_REG_RMS_IB_COEFF, &coeff->rms.B);
	err |= read_reg16(HLW811X_REG_RMS_U_COEFF, &coeff->rms.U);
	err |= read_reg16(HLW811X_REG_POWER_A_COEFF, &coeff->power.A);
	err |= read_reg16(HLW811X_REG_POWER_B_COEFF, &coeff->power.B);
	err |= read_reg16(HLW811X_REG_POWER_S_COEFF, &coeff->power.S);
	err |= read_reg16(HLW811X_REG_ENERGY_A_COEFF, &coeff->energy.A);
	err |= read_reg16(HLW811X_REG_ENERGY_B_COEFF, &coeff->energy.B);

	if (err != HLW811X_ERROR_NONE) {
		return err;
	}

	if ((err = read_reg16(HLW811X_REG_COEFF_CHKSUM, &chksum))
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

	memcpy(&m.coeff, coeff, sizeof(m.coeff));

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

void hlw811x_set_resistor_ratio(const struct hlw811x_resistor_ratio *ratio)
{
	memcpy(&m.ratio, ratio, sizeof(m.ratio));
	HLW811X_INFO("Resistor ratio set: K1_A=%d, K1_B=%d, K2=%d",
			ratio->K1_A, ratio->K1_B, ratio->K2);
}

void hlw811x_get_resistor_ratio(struct hlw811x_resistor_ratio *ratio)
{
	memcpy(ratio, &m.ratio, sizeof(m.ratio));
}

hlw811x_error_t hlw811x_set_pga(const struct hlw811x_pga *pga)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_SYS_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	reg &= ~(0x1FF << 0); /* clear PGA bits */
	reg |= (pga->A << 0) | (pga->U << 3) | (pga->B << 6);

	if ((err = write_reg16(HLW811X_REG_SYS_CTRL, reg))
			== HLW811X_ERROR_NONE) {
		memcpy(&m.pga, pga, sizeof(m.pga));
		HLW811X_INFO("PGA set: A=%d, U=%d, B=%d",
				pga->A, pga->U, pga->B);
	}

	return err;
}

hlw811x_error_t hlw811x_get_pga(struct hlw811x_pga *pga)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_SYS_CTRL, &reg))
			!= HLW811X_ERROR_NONE) {
		return err;
	}

	pga->A = (reg >> 0) & 0x07; /* PGAIA */
	pga->U = (reg >> 3) & 0x07; /* PGAU */
	pga->B = (reg >> 6) & 0x07; /* PGAIB */

	memcpy(&m.pga, pga, sizeof(m.pga));

	return HLW811X_ERROR_NONE;
}

hlw811x_error_t hlw811x_enable_channel(hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_SYS_CTRL, &reg))
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

	if ((err = write_reg16(HLW811X_REG_SYS_CTRL, reg))
			== HLW811X_ERROR_NONE) {
		HLW811X_INFO("Channel enabled: %d", channel);
	}

	return err;
}

hlw811x_error_t hlw811x_disable_channel(hlw811x_channel_t channel)
{
	hlw811x_error_t err;
	uint16_t reg;

	if ((err = read_reg16(HLW811X_REG_SYS_CTRL, &reg))
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

	if ((err = write_reg16(HLW811X_REG_SYS_CTRL, reg))
			== HLW811X_ERROR_NONE) {
		HLW811X_INFO("Channel disabled: %d", channel);
	}

	return err;
}

hlw811x_error_t hlw811x_reset(void)
{
	HLW811X_INFO("Resetting HLW811X chip");
	return reset_chip();
}

hlw811x_error_t hlw811x_init(hlw811x_interface_t interface)
{
	memset(&m, 0, sizeof(m));

	m.iface = interface;

	return HLW811X_ERROR_NONE;
}
