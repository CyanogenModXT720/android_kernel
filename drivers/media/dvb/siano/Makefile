#sms1xxx-objs := smscoreapi.o smsusb.o smsdvb.o sms-cards.o

#obj-$(CONFIG_DVB_SIANO_SMS1XXX) += sms1xxx.o

#EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core

#EXTRA_CFLAGS += $(extra-cflags-y) $(extra-cflags-m)
###############################################################################
#
# Siano Mobile Silicon, Inc.
# MDTV receiver kernel modules.
# Copyright (C) 2006-2008, Uri Shkolnik
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
###############################################################################
# Local variables initialization
SMS_DVB3_SUBSYS := 0
SMS_DVB5_S2API_SUBSYS := 0
SMS_HOSTLIB_SUBSYS := 0

SMS_USB_DRV := 0
SMS_SDIO_DRV := 0
SMS_SPI_PXA310_DRV := 0

# enable/disable driver log
#DEBUG = y

#Local variables initailization
MODULE_NAME = smsmdtv

# Default object, include in every build variant
SMSOBJ := smscoreapi.o sms-cards.o smsendian.o smsir.o

# Kernel subsystems support
ifdef CONFIG_SMS_DVB3_SUBSYS
ifneq ($(CONFIG_SMS_DVB3_SUBSYS),n)
SMS_DVB3_SUBSYS := 1
EXTRA_CFLAGS += -DSMS_DVB3_SUBSYS -Idrivers/media/dvb/dvb-core
SMSOBJ += smsdvb.o
endif
endif

ifdef CONFIG_SMS_DVB5_S2API_SUBSYS
ifneq ($(CONFIG_SMS_DVB5_S2API_SUBSYS),n)
SMS_DVB5_S2API_SUBSYS := 1
EXTRA_CFLAGS += -DSMS_DVB5_S2API_SUBSYS
endif
endif

ifdef CONFIG_SMS_HOSTLIB_SUBSYS
ifneq ($(CONFIG_SMS_HOSTLIB_SUBSYS),n)
SMS_HOSTLIB_SUBSYS := 1
EXTRA_CFLAGS += -DSMS_HOSTLIB_SUBSYS
SMSOBJ += smschar.o
endif
endif

ifneq ($(CONFIG_SMS_SPI_DRV), n)
EXTRA_CFLAGS += -DSMS_SPI_DRV
SMSOBJ += smsspilog.o smsspicommon.o smsspiphy_omap34xx.o
endif

ifdef CONFIG_SMS_NET_SUBSYS
ifneq ($(CONFIG_SMS_NET_SUBSYS),n)
SMS_NET_SUBSYS := 1
EXTRA_CFLAGS += -DSMS_NET_SUBSYS
SMSOBJ += smsnet.o
endif
endif

# Hardware (host) interfaces support
ifdef CONFIG_SMS_USB_DRV
ifneq ($(CONFIG_SMS_USB_DRV),n)
SMS_USB_DRV := 1
EXTRA_CFLAGS += -DSMS_USB_DRV
SMSOBJ += smsusb.o
endif
endif

ifdef CONFIG_SMS_SDIO_DRV
ifneq ($(CONFIG_SMS_SDIO_DRV),n)
SMS_SDIO_DRV := 1
EXTRA_CFLAGS += -DSMS_SDIO_DRV
SMSOBJ += smssdio.o
endif
endif

ifdef CONFIG_SMS_SPI_PXA310_DRV
ifneq ($(CONFIG_SMS_SPI_PXA310_DRV),n)
SMS_SPI_PXA310_DRV := 1
EXTRA_CFLAGS += -DSMS_SPI_PXA310_DRV
SMSOBJ += smsspidrv.o smsspicommon.o smsspiphy_pxa.o
endif
endif

#ifeq ($(DEBUG), y)
EXTRA_CFLAGS += -DSMS_KERNEL_DBG
#EXTRA_CFLAGS += -DSPIBUS_DEBUG
#endif

smsmdtv-objs := $(SMSOBJ)

#obj-m := smsmdtv.o
obj-$(CONFIG_MOT_FEAT_TDMB) := smsmdtv.o