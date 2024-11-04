/*
 * SPDX-FileCopyrightText: 2024 Kyunghwan Kwon <k@libmcu.org>
 *
 * SPDX-License-Identifier: MIT
 */

#include "hlw811x.h"
#include "hlw811x_overrides.h"

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

typedef size_t (*encoder_t)(uint8_t *buf, size_t bufsize,
		const uint8_t *data, size_t datalen);
typedef size_t (*decoder_t)(uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len);

static hlw811x_interface_t iface;

static int32_t convert_24bit_to_int32(const int32_t value)
{
	int32_t result = value;

	if (result & 0x00800000) {
		return (int32_t)(result | (int32_t)0xFF000000);
	}

	return (int32_t)result;
}

static size_t encode_uart(uint8_t *buf, size_t bufsize,
		const uint8_t *data, size_t datalen)
{
	if (bufsize < datalen + 2) {
		HLW811X_ERROR("Buffer size is too small");
		return 0;
	}

	buf[0] = 0xA5;
	uint8_t chksum = buf[0];

	for (size_t i = 0; i < datalen; i++) {
		buf[i + 1] = data[i];
		chksum += data[i];
	}
	buf[datalen + 1] = ~chksum;

	return datalen + 2;
}

static size_t decode_uart(uint8_t *buf, size_t bufsize,
		const uint8_t *tx, size_t tx_len,
		const uint8_t *rx, size_t rx_len)
{
	if (tx_len < 1) {
		HLW811X_ERROR("Invalid tx_len");
		return 0;
	}

	if (rx_len < 3 || bufsize < rx_len - 1) {
		HLW811X_ERROR("Invalid rx_len");
		return 0;
	}

	uint8_t chksum = tx[tx_len - 1];

	for (size_t i = 0; i < rx_len - 1; i++) {
		buf[i] = rx[i];
		chksum += rx[i];
	}

	if ((uint8_t)~chksum != rx[rx_len - 1]) {
		HLW811X_ERROR("Checksum mismatch %x : %x",
				~chksum, rx[rx_len - 1]);
		return 0;
	}

	return rx_len - 1;
}

static hlw811x_error_t send_frame(hlw811x_reg_addr_t addr,
		const uint8_t *data, size_t datalen)
{
	uint8_t payload[3] = { (uint8_t)addr, 0, 0 };
	uint8_t txbuf[sizeof(payload) + 2] = { 0, };
	encoder_t encoder = encode_uart;

	if (datalen > 2 || (data == NULL && datalen != 0)) {
		HLW811X_ERROR("Invalid parameter");
		return HLW811X_INVALID_PARAM;
	}

	for (size_t i = 0; i < datalen; i++) {
		payload[i + 1] = data[i];
	}

	if (iface == HLW811X_UART) {
	} else {
		HLW811X_ERROR("Not implemented");
		return HLW811X_NOT_IMPLEMENTED;
	}

	const size_t len = (*encoder)(txbuf, sizeof(txbuf), payload, datalen+1);

	if (hlw811x_ll_write(txbuf, len) != 0) {
		HLW811X_ERROR("hlw811x_ll_write() failed");
		return HLW811X_IO_ERROR;
	}

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t write_reg(hlw811x_reg_addr_t addr,
		const uint8_t *data, size_t datalen)
{
	return send_frame((uint8_t)(addr | 0x80u), data, datalen);
}

static hlw811x_error_t read_reg(hlw811x_reg_addr_t addr,
		uint8_t *buf, size_t bytes_to_read)
{
	hlw811x_error_t err;
	int rc;
	size_t len;
	uint8_t rxbuf[bytes_to_read + 3];
	uint8_t txbuf[3];
	encoder_t encoder = encode_uart;
	decoder_t decoder = decode_uart;

	if ((err = send_frame(addr, 0, 0)) != HLW811X_ERROR_NONE) {
		return err;
	}

	if ((rc = hlw811x_ll_read(rxbuf, sizeof(rxbuf))) < 0) {
		HLW811X_ERROR("hlw811x_ll_read() failed");
		return HLW811X_IO_ERROR;
	}

	if (iface == HLW811X_UART) {
	} else {
		HLW811X_ERROR("Not implemented");
		return HLW811X_NOT_IMPLEMENTED;
	}

	len = (*encoder)(txbuf, sizeof(txbuf), (uint8_t *)&addr, 1);
	if ((len = (*decoder)(buf, bytes_to_read,
			txbuf, len, rxbuf, (size_t)rc)) != bytes_to_read) {
		HLW811X_ERROR("decoder returned %d", len);
		return HLW811X_INCORRECT_RESPONSE;
	}

	return HLW811X_ERROR_NONE;
}

static hlw811x_error_t reset_chip(void)
{
	const uint8_t cmd = CMD_RESET_CHIP;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t enable_write(void)
{
	const uint8_t cmd = CMD_ENABLE_WRITE;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t disable_write(void)
{
	const uint8_t cmd = CMD_DISABLE_WRITE;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
}

static hlw811x_error_t select_channel(hlw811x_channel_t channel)
{
	const uint8_t cmd = (channel == HLW811X_CHANNEL_A)?
		CMD_SET_CHANNEL_A : CMD_SET_CHANNEL_B;
	return write_reg(HLW811X_REG_COMMAND, &cmd, 1);
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

hlw811x_error_t hlw811x_select_channel(hlw811x_channel_t channel)
{
	return select_channel(channel);
}

hlw811x_error_t hlw811x_reset(void)
{
	return reset_chip();
}

hlw811x_error_t hlw811x_init(hlw811x_interface_t interface)
{
	iface = interface;
	return HLW811X_ERROR_NONE;
}
