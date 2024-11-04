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

int hlw811x_ll_write(const uint8_t *data, size_t datalen) {
	return mock().actualCall(__func__)
		.withMemoryBufferParameter("data", data, datalen)
		.returnIntValueOrDefault(0);
}

int hlw811x_ll_read(uint8_t *buf, size_t bufsize) {
	return mock().actualCall(__func__)
		.withOutputParameter("buf", buf)
		.returnIntValueOrDefault(0);
}

TEST_GROUP(HLW811x) {
	void setup(void) {
		hlw811x_init(HLW811X_UART);
	}
	void teardown(void) {
		mock().checkExpectations();
		mock().clear();
	}
};

TEST(HLW811x, reset_ShouldSendResetCommand) {
	mock().expectOneCall("hlw811x_ll_write")
		.withMemoryBufferParameter("data", (const uint8_t *)"\xA5\xEA\x96\xDA", 4)
		.andReturnValue(0);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_reset());
}

TEST(HLW811x, select_channel_ShouldSendChannelACommand_WhenChannelAIsSelected) {
	mock().expectOneCall("hlw811x_ll_write")
		.withMemoryBufferParameter("data", (const uint8_t *)"\xA5\xEA\x5A\x16", 4)
		.andReturnValue(0);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_select_channel(HLW811X_CHANNEL_A));
}

TEST(HLW811x, select_channel_ShouldSendChannelACommand_WhenChannelBIsSelected) {
	mock().expectOneCall("hlw811x_ll_write")
		.withMemoryBufferParameter("data", (const uint8_t *)"\xA5\xEA\xA5\xCB", 4)
		.andReturnValue(0);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_select_channel(HLW811X_CHANNEL_B));
}

TEST(HLW811x, write_reg_ShouldSendDataToSpecifiedRegister) {
	mock().expectOneCall("hlw811x_ll_write")
		.withMemoryBufferParameter("data", (const uint8_t *)"\xA5\x80\x0A\x04\xCC", 5)
		.andReturnValue(0);
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_write_reg(HLW811X_REG_SYS_CTRL, (const uint8_t *)"\x0A\x04", 2));
}

TEST(HLW811x, read_reg_ShouldReadDataFromSpecifiedRegister) {
	mock().expectOneCall("hlw811x_ll_write").ignoreOtherParameters().andReturnValue(0);
	mock().expectOneCall("hlw811x_ll_read")
		.withOutputParameterReturning("buf", (const uint8_t *)"\x0A\x04\x97", 3)
		.andReturnValue(3);
	uint8_t buf[2];
	LONGS_EQUAL(HLW811X_ERROR_NONE, hlw811x_read_reg(HLW811X_REG_SYS_CTRL, buf, sizeof(buf)));
	MEMCMP_EQUAL("\x0A\x04", buf, sizeof(buf));
}
