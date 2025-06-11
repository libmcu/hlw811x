/*
 * SPDX-FileCopyrightText: 2024 Kyunghwan Kwon <k@libmcu.org>
 *
 * SPDX-License-Identifier: MIT
 */

#include "CppUTest/TestHarness.h"
#include "CppUTest/TestHarness_c.h"
#include "CppUTestExt/MockSupport.h"

#include "hlw811x.h"
#include "hlw811x_overrides.h"

int hlw811x_ll_write(const uint8_t *data, size_t datalen, void *ctx) {
	return mock().actualCall(__func__)
		.withMemoryBufferParameter("data", data, datalen)
		.returnIntValueOrDefault(0);
}

int hlw811x_ll_read(uint8_t *buf, size_t bufsize, void *ctx) {
	return mock().actualCall(__func__)
		.withOutputParameter("buf", buf)
		.returnIntValueOrDefault(0);
}

TEST_GROUP(HLW811x) {
	struct hlw811x *hlw811x;

	void setup(void) {
		hlw811x = hlw811x_create(HLW811X_UART, NULL);
	}
	void teardown(void) {
		hlw811x_destroy(hlw811x);

		mock().checkExpectations();
		mock().clear();
	}

	void expect_read(const char addr[2], const char *buf, size_t bufsize) {
		mock().expectOneCall("hlw811x_ll_write")
			.withMemoryBufferParameter("data", (const uint8_t *)addr, 2)
			.andReturnValue(2);
		mock().expectOneCall("hlw811x_ll_read")
			.withOutputParameterReturning("buf", (const uint8_t *)buf, bufsize)
			.andReturnValue((int)bufsize);
	}

	void expect_write(const char *buf, size_t bufsize) {
		mock().expectOneCall("hlw811x_ll_write")
			.withMemoryBufferParameter("data", (const uint8_t *)"\xA5\xEA\xE5\x8B", 4)
			.andReturnValue(4);
		mock().expectOneCall("hlw811x_ll_write")
			.withMemoryBufferParameter("data", (const uint8_t *)buf, bufsize)
			.andReturnValue((int)bufsize);
		mock().expectOneCall("hlw811x_ll_write")
			.withMemoryBufferParameter("data", (const uint8_t *)"\xA5\xEA\xDC\x94", 4)
			.andReturnValue(4);
	}

	void expect_coeff_read(struct hlw811x_coeff *buf) {
		struct hlw811x_coeff coeff;
		//expect_read("\xA5\x02", "\x10\x00\x48", 3);
		expect_read("\xA5\x02", "\xFF\xFF\x5A", 3);
		expect_read("\xA5\x70", "\xFF\xFF\xEC", 3);
		expect_read("\xA5\x71", "\xFF\xFF\xEB", 3);
		expect_read("\xA5\x72", "\xFF\xFF\xEA", 3);
		expect_read("\xA5\x73", "\xFF\xFF\xE9", 3);
		expect_read("\xA5\x74", "\xFF\xFF\xE8", 3);
		expect_read("\xA5\x75", "\xFF\xFF\xE7", 3);
		expect_read("\xA5\x76", "\xFF\xFF\xE6", 3);
		expect_read("\xA5\x77", "\xFF\xFF\xE5", 3);
		expect_read("\xA5\x6F", "\x00\x08\xE3", 3);
		LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_read_coeff(hlw811x, &coeff));
		if (buf) {
			memcpy(buf, &coeff, sizeof(coeff));
		}
	}

	void set_default_param(float K1_A = 1) {
		const struct hlw811x_resistor_ratio ratio = {
		    .K1_A = K1_A,
		    .K1_B = 1,
		    .K2 = 1,
		};
		const struct hlw811x_pga pga = {
		    .A = HLW811X_PGA_GAIN_2,
		    .B = HLW811X_PGA_GAIN_2,
		    .U = HLW811X_PGA_GAIN_2,
		};

		hlw811x_set_resistor_ratio(hlw811x, &ratio);

		expect_read("\xA5\x00", "\x0A\x04\x4C", 3);
		expect_write("\xA5\x80\x0A\x49\x87", 5);
		LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_set_pga(hlw811x, &pga));
	}
};

TEST(HLW811x, reset_ShouldSendResetCommand) {
	mock().expectOneCall("hlw811x_ll_write")
		.withMemoryBufferParameter("data", (const uint8_t *)"\xA5\xEA\x96\xDA", 4)
		.andReturnValue(4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_reset(hlw811x));
}

TEST(HLW811x, select_channel_ShouldSendChannelACommand_WhenChannelAIsSelected) {
	mock().expectOneCall("hlw811x_ll_write")
		.withMemoryBufferParameter("data", (const uint8_t *)"\xA5\xEA\x5A\x16", 4)
		.andReturnValue(4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_select_channel(hlw811x, HLW811X_CHANNEL_A));
}

TEST(HLW811x, select_channel_ShouldSendChannelACommand_WhenChannelBIsSelected) {
	mock().expectOneCall("hlw811x_ll_write")
		.withMemoryBufferParameter("data", (const uint8_t *)"\xA5\xEA\xA5\xCB", 4)
		.andReturnValue(4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_select_channel(hlw811x, HLW811X_CHANNEL_B));
}

TEST(HLW811x, write_reg_ShouldSendDataToSpecifiedRegister) {
	expect_write("\xA5\x80\x0A\x04\xCC", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_write_reg(hlw811x, HLW811X_REG_SYS_CTRL, (const uint8_t *)"\x0A\x04", 2));
}

TEST(HLW811x, read_reg_ShouldReadDataFromSpecifiedRegister) {
	mock().expectOneCall("hlw811x_ll_write")
		.withMemoryBufferParameter("data", (const uint8_t *)"\xA5\x00", 2)
		.andReturnValue(2);
	mock().expectOneCall("hlw811x_ll_read")
		.withOutputParameterReturning("buf", (const uint8_t *)"\x0A\x04\x4C", 3)
		.andReturnValue(3);
	uint8_t buf[2];
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_read_reg(hlw811x, HLW811X_REG_SYS_CTRL, buf, sizeof(buf)));
	MEMCMP_EQUAL("\x0A\x04", buf, sizeof(buf));
}

TEST(HLW811x, enable_channel_ShouldSendEnableCommand_WhenAllChannelsAreGiven) {
	expect_read("\xA5\x00", "\x0A\x04\x4C", 3);
	expect_write("\xA5\x80\x0E\x04\xC8", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE,
			hlw811x_enable_channel(hlw811x, HLW811X_CHANNEL_A |
					HLW811X_CHANNEL_B | HLW811X_CHANNEL_U));
}

TEST(HLW811x, disable_channel_ShouldSendDisableCommand_WhenAllChannelsAreGiven) {
	expect_read("\xA5\x00", "\x0A\x04\x4C", 3);
	expect_write("\xA5\x80\x00\x04\xD6", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE,
			hlw811x_disable_channel(hlw811x, HLW811X_CHANNEL_A |
					HLW811X_CHANNEL_B | HLW811X_CHANNEL_U));
}

TEST(HLW811x, get_pga_ShouldReturnPgaValues) {
	struct hlw811x_pga pga;
	expect_read("\xA5\x00", "\x0A\x04\x4C", 3);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_pga(hlw811x, &pga));
	LONGS_EQUAL(HLW811X_PGA_GAIN_16, pga.A);
	LONGS_EQUAL(HLW811X_PGA_GAIN_1, pga.B);
	LONGS_EQUAL(HLW811X_PGA_GAIN_1, pga.U);
}

TEST(HLW811x, set_pga_ShouldSetPgaValues) {
	struct hlw811x_pga pga = {
		.A = HLW811X_PGA_GAIN_1,
		.B = HLW811X_PGA_GAIN_4,
		.U = HLW811X_PGA_GAIN_8,
	};
	expect_read("\xA5\x00", "\x0A\x04\x4C", 3);
	expect_write("\xA5\x80\x0A\x98\x38", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_set_pga(hlw811x, &pga));
}

TEST(HLW811x, energy_ShouldReturnEnergyValue_WhenMaxValueIsGiven) {
	expect_coeff_read(NULL);
	set_default_param();

	int32_t Wh;
	expect_read("\xA5\x28", "\xFF\xFF\xFF\x35", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_energy(hlw811x, HLW811X_CHANNEL_A, &Wh));
	LONGS_EQUAL(32766998, Wh);
	expect_read("\xA5\x28", "\x80\x00\x00\xb2", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_energy(hlw811x, HLW811X_CHANNEL_A, &Wh));
	LONGS_EQUAL(16383500, Wh);
	expect_read("\xA5\x28", "\x7F\xFF\xFF\xb5", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_energy(hlw811x, HLW811X_CHANNEL_A, &Wh));
	LONGS_EQUAL(16383498, Wh);
	expect_read("\xA5\x28", "\x00\x00\x00\x32", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_energy(hlw811x, HLW811X_CHANNEL_A, &Wh));
	LONGS_EQUAL(0, Wh);
	expect_read("\xA5\x28", "\x00\x00\x01\x31", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_energy(hlw811x, HLW811X_CHANNEL_A, &Wh));
	LONGS_EQUAL(1, Wh);
	expect_read("\xA5\x28", "\x00\x00\x30\x02", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_energy(hlw811x, HLW811X_CHANNEL_A, &Wh));
	LONGS_EQUAL(93, Wh);
}

TEST(HLW811x, get_power_ShouldReturnPowerValue_WhenBoundaryValuesAreGiven) {
	expect_coeff_read(NULL);
	set_default_param();

	int32_t mW;
	expect_read("\xA5\x2C", "\xFF\xFF\xFF\xFF\x32", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_power(hlw811x, HLW811X_CHANNEL_A, &mW));
	LONGS_EQUAL(0, mW);
	expect_read("\xA5\x2C", "\x00\x00\x00\x01\x2D", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_power(hlw811x, HLW811X_CHANNEL_A, &mW));
	LONGS_EQUAL(0, mW);
	expect_read("\xA5\x2C", "\x7F\xFF\xFF\xFF\xB2", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_power(hlw811x, HLW811X_CHANNEL_A, &mW));
	LONGS_EQUAL(65534999, mW);
	expect_read("\xA5\x2C", "\x80\x00\x00\x00\xAE", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_power(hlw811x, HLW811X_CHANNEL_A, &mW));
	LONGS_EQUAL(-65535000, mW);
	expect_read("\xA5\x2C", "\x00\x0B\xDB\xBC\x8C", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_power(hlw811x, HLW811X_CHANNEL_A, &mW));
	LONGS_EQUAL(23716, mW);
}

TEST(HLW811x, get_current_rms_ShouldReturnCurrentRmsValue_WhenBoundaryValuesAreGiven) {
	expect_coeff_read(NULL);
	set_default_param();

	int32_t mA;
	expect_read("\xA5\x24", "\x00\x00\x01\x35", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_rms(hlw811x, HLW811X_CHANNEL_A, &mA));
	LONGS_EQUAL(0, mA);
	expect_read("\xA5\x24", "\x00\x01\x00\x35", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_rms(hlw811x, HLW811X_CHANNEL_A, &mA));
	LONGS_EQUAL(1, mA);
	expect_read("\xA5\x24", "\x7F\xFF\xFF\xB9", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_rms(hlw811x, HLW811X_CHANNEL_A, &mA));
	LONGS_EQUAL(65534, mA);
}

TEST(HLW811x, get_voltage_rms_ShouldReturnVoltageRmsValue_WhenBoundaryValuesAreGiven) {
	expect_coeff_read(NULL);
	set_default_param();

	int32_t mV;
	expect_read("\xA5\x26", "\x7F\xFF\xFF\xB7", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_rms(hlw811x, HLW811X_CHANNEL_U, &mV));
	LONGS_EQUAL(131069, mV);
	expect_read("\xA5\x26", "\x00\x00\x01\x33", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_rms(hlw811x, HLW811X_CHANNEL_U, &mV));
	LONGS_EQUAL(0, mV);
}

TEST(HLW811x, energy_ShouldReturn1Wh_When1WhIsGiven) {
	struct hlw811x_coeff coeff;
	expect_read("\xA5\x02", "\xB5\x40\x63", 3);
	expect_read("\xA5\x70", "\xFF\xFF\xEC", 3);
	expect_read("\xA5\x71", "\xFF\xFF\xEB", 3);
	expect_read("\xA5\x72", "\xFF\xFF\xEA", 3);
	expect_read("\xA5\x73", "\xFF\xFF\xE9", 3);
	expect_read("\xA5\x74", "\xFF\xFF\xE8", 3);
	expect_read("\xA5\x75", "\xFF\xFF\xE7", 3);
	expect_read("\xA5\x76", "\xE7\x69\x94", 3);
	expect_read("\xA5\x77", "\xFF\xFF\xE5", 3);
	expect_read("\xA5\x6F", "\x18\x9E\x35", 3);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_read_coeff(hlw811x, &coeff));
	set_default_param(5);

	int32_t Wh;
	expect_read("\xA5\x28", "\x00\x00\x01\x31", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_energy(hlw811x, HLW811X_CHANNEL_A, &Wh));
	LONGS_EQUAL(0, Wh);

	expect_read("\xA5\x28", "\xFF\xFF\xFF\x35", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_get_energy(hlw811x, HLW811X_CHANNEL_A, &Wh));
	LONGS_EQUAL(4194308, Wh); /* It should be 16777215. 0.0001192% error. */
}

TEST(HLW811x, apply_calibration_ShouldApplyCalibration) {
	const struct hlw811x_calibration expected = {
		.hfconst = 0x1234,
		.pa_gain = 0x5678,
		.pb_gain = 0x9ABC,
		.phase_a = 0xDE,
		.phase_b = 0xF0,
		.paos = 0x1111,
		.pbos = 0x2222,
		.rms_iaos = 0x3333,
		.rms_ibos = 0x4444,
		.ib_gain = 0x5555,
		.ps_gain = 0x6666,
		.psos = 0x7777,
	};

	expect_write("\xA5\x82\x12\x34\x92", 5);
	expect_write("\xA5\x85\x56\x78\x07", 5);
	expect_write("\xA5\x86\x9A\xBC\x7E", 5);
	expect_write("\xA5\x87\xDE\xF5", 4);
	expect_write("\xA5\x88\xF0\xE2", 4);
	expect_write("\xA5\x8A\x11\x11\xAE", 5);
	expect_write("\xA5\x8B\x22\x22\x8B", 5);
	expect_write("\xA5\x8E\x33\x33\x66", 5);
	expect_write("\xA5\x8F\x44\x44\x43", 5);
	expect_write("\xA5\x90\x55\x55\x20", 5);
	expect_write("\xA5\x91\x66\x66\xFD", 5);
	expect_write("\xA5\x92\x77\x77\xDA", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_apply_calibration(hlw811x, &expected));
}

TEST(HLW811x, calc_active_power_gain_ShouldReturnCalculatedGain) {
	uint16_t gain;
	float err = 1.0918f; // Example error percentage
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_calc_active_power_gain(hlw811x, err, &gain));
	LONGS_EQUAL(0xFE9F, gain);
}

TEST(HLW811x, calc_active_power_offset_ShouldReturnCalculatedOffset) {
	uint16_t offset;
	float error_pct = -0.2553f; // Example error percentage
	expect_read("\xA5\x2C", "\x00\x0F\x5A\xB7\x0E", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_calc_active_power_offset(hlw811x, HLW811X_CHANNEL_A, error_pct, &offset));
	LONGS_EQUAL(0xa08, offset);
}

TEST(HLW811x, calc_rms_offset_ShouldReturnCalculatedOffset) {
	uint16_t offset;
	expect_read("\xA5\x24", "\x00\x01\xc3\x72", 4);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_calc_rms_offset(hlw811x, HLW811X_CHANNEL_A, &offset));
	LONGS_EQUAL(0xFE3D, offset);
}

TEST(HLW811x, calc_apparent_power_gain_ShouldReturnCalculatedGain) {
	uint16_t gain;
	expect_read("\xA5\x2C", "\x0A\x1F\x36\x94\x3B", 5);
	expect_read("\xA5\x2E", "\x0A\x1F\x45\x26\x98", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_calc_apparent_power_gain(hlw811x, &gain));
	LONGS_EQUAL(0xD7, gain);
}

TEST(HLW811x, calc_apparent_power_offset_ShouldReturnCalculatedOffset) {
	uint16_t offset;
	expect_read("\xA5\x2C", "\x00\x08\xC2\xD4\x90", 5);
	expect_read("\xA5\x2E", "\x00\x08\xC1\xD7\x8C", 5);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_calc_apparent_power_offset(hlw811x, &offset));
	LONGS_EQUAL(0xFD, offset);
}
