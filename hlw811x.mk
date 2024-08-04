# SPDX-License-Identifier: MIT

ifneq ($(HLW811X_ROOT),)
hlw811x-basedir := $(HLW811X_ROOT)/
endif

HLW811X_SRCS := \
	$(hlw811x-basedir)hlw811x.c \

HLW811X_INCS := $(hlw811x-basedir)
