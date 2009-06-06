ifeq ($(CONFIG_MOT_FEAT_FX2LP_I2C), y)
        obj-$(CONFIG_OTG_BRASSBOARD) += otg_hs_bus_if.o
        otg_hs_bus_if-objs := ../brassboard/brassboard-tr.o
        otg_hs_bus_if-objs += ../otglib/pcd.o
        otg_hs_bus_if-objs += ../mxc/mxc-hrt.o
        otg_hs_bus_if-objs += fx2lp_lib.o
        otg_hs_bus_if-objs += fx2lp_ocd.o
        otg_hs_bus_if-objs += fx2lp_pcd.o
        otg_hs_bus_if-objs += fx2lp_tcd.o
        otg_hs_bus_if-objs += fx2lp_ready_events.o

OTG=/vobs/jem/os/GPL/belcarra/usb_driver
OTGCORE_DIR=/vobs/jem/os/GPL/belcarra/usb_driver
FX2LPDIR=/vobs/jem/os/GPL/belcarra/usb_driver/ocd/fx2lp
EXTRA_CFLAGS += -I$(OTG) -Wno-unused -Wno-format -I$(OTGCORE_DIR) -DFX2LP_NODMA -DFX2LP_HSUSB -I$(FX2LPDIR)
EXTRA_CFLAGS_nostdinc += -I$(OTG) -Wno-unused -Wno-format -I$(OTGCORE_DIR)
endif
