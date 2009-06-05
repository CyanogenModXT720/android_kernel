# Generic Composite Driver
#
# @(#) balden@belcarra.com|otg/functions/generic/Makefile-l26|20060419204257|43174
# Copyright (c) 2004-2005 Belcarra Technologies Corp
# Copyright (c) 2005-2006 Belcarra Technologies 2005 Corp

#include /vobs/linuxjava/drivers_1/usb_driver/.config
USB_SRC_DIR = drivers/usb/belcarra_otg/

ifdef CONFIG_MOT_FEAT_GCC_WERR
CFLAGS_RM_generic.o = -Werror
endif

generic_cf-objs := generic.o generic-cf.o
generic_cl-objs := generic-cl.o

obj-$(CONFIG_OTG_GENERIC) += generic_cf.o

OTG =$(USB_SRC_DIR)
OTGCORE_DIR =$(USB_SRC_DIR)/otgcore

#EXTRA_CFLAGS += -I$(OTG)
#EXTRA_CFLAGS += -I$(OTGCORE_DIR)

EXTRA_CFLAGS += -I$(OTG) -Wno-unused -Wno-format -I$(OTGCORE_DIR)
EXTRA_CFLAGS_nostdinc += -I$(OTG) -Wno-unused -Wno-format -I$(OTGCORE_DIR)

