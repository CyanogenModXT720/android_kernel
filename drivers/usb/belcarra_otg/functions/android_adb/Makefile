USB_SRC_DIR = drivers/usb/belcarra_otg/

android_adb_if-objs     := android_adb_fd.o

obj-$(CONFIG_OTG_ANDROID_ADB) += android_adb_if.o

OTG= $(USB_SRC_DIR)
ADBD=$(USB_SRC_DIR)/functions/android_adb

#CFLAGS += -I$(OTG)
#CFLAGS += -I$(ADBD)
 
EXTRA_CFLAGS += -I$(ADBD) -I$(OTG) -Wno-unused -Wno-format
EXTRA_CFLAGS_nostdinc += -I$(ADBD) -I$(OTG) -Wno-unused -Wno-format

