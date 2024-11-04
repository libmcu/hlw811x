# SPDX-License-Identifier: MIT

COMPONENT_NAME = hlw811x

SRC_FILES = \
	../hlw811x.c

TEST_SRC_FILES = \
	src/hlw811x_test.cpp \
	src/test_all.cpp \

INCLUDE_DIRS = $(CPPUTEST_HOME)/include ../
MOCKS_SRC_DIRS =
CPPUTEST_CPPFLAGS = -Wno-error=unused-macros

include runners/MakefileRunner
