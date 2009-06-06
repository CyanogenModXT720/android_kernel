# Function driver for a Random Mouse
# @(#) balden@belcarra.com|otg/functions/msc/Makefile-l26|20060419204258|01484
#
# Copyright (c) 2004 Belcarra
# Copyright (c) 2005-2006 Belcarra Technologies 2005 Corp

#include /vobs/linuxjava/drivers_1/usb_driver/.config
USB_SRC_DIR = drivers/usb/belcarra_otg/

mouse_fd-objs	:= mouse-fd.o 
mouse_cf-objs	:= mouse-cf.o 
mouse_if-objs	:= mouse-if.o 

ifeq ($(CONFIG_OTG_MOUSE_TRADITIONAL),y)
obj-$(CONFIG_OTG_MOUSE) += mouse_fd.o 
endif
ifeq ($(CONFIG_OTG_MOUSE_INTERFACE), y)
obj-$(CONFIG_OTG_MOUSE) += mouse_if.o 
endif
ifeq ($(CONFIG_OTG_MOUSE_COMPOSITE), y)
obj-$(CONFIG_OTG_MOUSE) += mouse_cf.o
endif

OTG= $(USB_SRC_DIR)
MOUSED=$(OTG)/functions/mouse

EXTRA_CFLAGS += -I$(MOUSED) -I$(OTG) -Wno-unused -Wno-format
EXTRA_CFLAGS_nostdinc += -I$(MOUSED) -I$(OTG) -Wno-unused -Wno-format
