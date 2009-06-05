USB_SRC_DIR = drivers/usb/belcarra_otg

mtp_if-objs     := mtp-fd.o mtp-io.o mtp-linux.o
pbg_if-objs     := mtp-fd.o mtp-io.o pbg-linux.o

obj-$(CONFIG_OTG_MTP) += mtp_if.o
obj-$(CONFIG_OTG_PBG) += pbg_if.o

OTG=$(USB_SRC_DIR)
MTPD=$(OTG)/functions/mtp
OTGCORE_DIR=$(OTG)/otgcore

#CFLAGS += -I$(OTG)
#CFLAGS += -I$(MTPD)
#CFLAGS += -I$(OTGCORE_DIR)

EXTRA_CFLAGS += -I$(MTPD) -I$(OTG) -Wno-unused -Wno-format -I$(OTGCORE_DIR) 
EXTRA_CFLAGS_nostdinc += -I$(MTPD) -I$(OTG) -Wno-unused -Wno-format -I$(OTGCORE_DIR)


