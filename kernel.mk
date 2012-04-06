# Copyright (C) 2009 Motorola, Inc.
#####################################################################
#
# Script creation notes by: David Ding (dding@motorola.com)
#
# The intention of creating this script is for Moto-Android platform
# common Kernel and kernel modules developer to make kernel zImage and
# driver modules .ko objects. As long as it is in your execution $PATH
# you can place this script anywhere you may preferred. A suggestion
# place can be in $HOME/bin directory, then make PATH=$PATH:$HOME/bin
#
# How to use:
# -----------
# $ cd {top-moto-android-working-dir}
# $ build_kernel
#
# if you are in the wrong place to start your kernel/module build
# script will quit and reminder you go to the RIGHT place to build
#
######################################################################
#set -x

PWD=$(shell pwd)

TOPDIR=$(PWD)
KERNEL_CONF_OUT_DIR= \
$(PWD)/out/target/pr/generic/obj/PARTITIONS/kernel_intermediates
KERNEL_BUILD_DIR=$(KERNEL_CONF_OUT_DIR)/build
KERNEL_SRC_DIR=$(PWD)/kernel
KERNEL_CROSS_COMPILE=$(PWD)/prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin/arm-eabi-
MOTO_PREBUILT_DIR=$(PWD)/motorola/bsp/prebuilt/target/images
MOTO_MOD_INSTALL=$(MOTO_PREBUILT_DIR)/system/lib/modules
DEPMOD=$(PWD)/motorola/bsp/prebuilt/host/bin/depmod

DEFCONFIGSRC        := ${KERNEL_SRC_DIR}/arch/arm/configs
LJAPDEFCONFIGSRC    := ${DEFCONFIGSRC}/ext_config
PRODUCT_SPECIFIC_DEFCONFIGS := \
    $(DEFCONFIGSRC)/mapphone_defconfig  
_TARGET_DEFCONFIG := __ext_mapphone_defconfig
TARGET_DEFCONFIG := $(DEFCONFIGSRC)/$(_TARGET_DEFCONFIG)

CHK_WARN := $(KERNEL_SRC_DIR)/scripts/chk_gcc_warn.pl
WARN_FILTER := $(KERNEL_SRC_DIR)/scripts/gcc_warn_filter.cfg
KERNEL_ERR_LOG := $(KERNEL_CONF_OUT_DIR)/.kbld_err_log.txt
MOD_ERR_LOG := $(KERNEL_CONF_OUT_DIR)/.kmod_err_log.txt
KFLAG := $(KERNEL_CONF_OUT_DIR)/.kbld_ok.txt
MFLAG := $(KERNEL_CONF_OUT_DIR)/.mbld_ok.txt
FFLAG := $(KERNEL_CONF_OUT_DIR)/.filter_ok.txt


all: config zImage modules modules_install ext_modules

inst_hook:
	@echo "Installing auto coding style hook"
	@-cp -f $(KERNEL_SRC_DIR)/scripts/pre-commit $(KERNEL_SRC_DIR)/.git/hooks/
	@-cp -f $(KERNEL_SRC_DIR)/scripts/checkpatch.pl $(KERNEL_SRC_DIR)/.git/hooks/
	@-chmod ugo+x $(KERNEL_SRC_DIR)/.git/hooks/*

ifneq ($(BLD_CONF),)
PRODUCT_SPECIFIC_DEFCONFIGS := $(DEFCONFIGSRC)/$(BLD_CONF)_defconfig
endif

ifneq ($(PRODUCT),)
PRODUCT_SPECIFIC_DEFCONFIGS += \
    ${LJAPDEFCONFIGSRC}/product/${PRODUCT}.config
endif

ifneq ($(ENG_BLD),)
PRODUCT_SPECIFIC_DEFCONFIGS += \
    ${LJAPDEFCONFIGSRC}/eng_bld.config
endif

ifeq ($(TEST_DRV_CER), 1)
        ifeq ($(TEST_COVERAGE),)
                TEST_COVERAGE=1
        endif
                                                                                                                                               
        ifeq ($(TEST_KMEMLEAK),)
                TEST_KMEMLEAK=1
        endif
                                                                                                                                               
        ifeq ($(TEST_FAULTINJECT),)
                TEST_FAULTINJECT=1
        endif
endif

# Option to enable or disable gcov
ifeq ($(TEST_COVERAGE),1)
        PRODUCT_SPECIFIC_DEFCONFIGS += \
			${LJAPDEFCONFIGSRC}/feature/coverage.config
endif
                                                                                                                                               
ifeq ($(TEST_KMEMLEAK),1)
        PRODUCT_SPECIFIC_DEFCONFIGS += \
			${LJAPDEFCONFIGSRC}/feature/kmemleak.config
endif
                                                                                                                                               
ifeq ($(TEST_FAULTINJECT),1)
        PRODUCT_SPECIFIC_DEFCONFIGS += \
			${LJAPDEFCONFIGSRC}/feature/faultinject.config
endif

#
# make kernel configuration
#--------------------------
config: inst_hook
	mkdir -p $(KERNEL_BUILD_DIR) $(MOTO_MOD_INSTALL)
	( perl -le 'print "# This file was automatically generated from:\n#\t" . join("\n#\t", @ARGV) . "\n"' $(PRODUCT_SPECIFIC_DEFCONFIGS) && cat $(PRODUCT_SPECIFIC_DEFCONFIGS) ) > $(TARGET_DEFCONFIG) || ( rm -f $@ && false )
	make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) \
		KBUILD_DEFCONFIG=$(_TARGET_DEFCONFIG) \
		defconfig modules_prepare

define chk_warn
sleep 2
if [ -f $(1) ]; then cat $(1) | \
$(CHK_WARN) $(KERNEL_SRC_DIR) $(WARN_FILTER); fi
rm -f $(1) 2> /dev/null
endef


define _vmlinux
-rm -f $(KFLAG) 2> /dev/null
((make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
	CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
	O=$(KERNEL_BUILD_DIR) -j4 vmlinux) \
	3>&1 1>&2 2>&3 && (touch $(KFLAG))) \
	| tee $(KERNEL_ERR_LOG)
@$(call chk_warn,  $(KERNEL_ERR_LOG))
rm $(KFLAG) 2> /dev/null
endef
$(WARN_FILTER):
	
$(FFLAG): $(WARN_FILTER)
	@echo "Gcc warning filter changed, clean build will be enforced\n"
	@make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
                 CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
                 O=$(KERNEL_BUILD_DIR) clean
	@touch $(FFLAG)
#
# build kernel and internal kernel modules
# ========================================
# We need to check warning no matter if build passed, failed or interuptted
zImage: $(FFLAG)
	@-$(call chk_warn,  $(KERNEL_ERR_LOG))
	@$(_vmlinux)  
	make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) -j4 zImage
	cp -f $(KERNEL_BUILD_DIR)/arch/arm/boot/zImage $(MOTO_PREBUILT_DIR)

# To build modules (.ko) in specific folder
# It is useful for build specific module with extra options
# (e.g. TEST_DRV_CER)
dir:
	make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) $(DIR_TO_BLD)

modules: $(FFLAG)
	@-$(call chk_warn, $(MOD_ERR_LOG))
	@-rm -f $(MFLAG) 2> /dev/null
	@((make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) \
		DEPMOD=$(DEPMOD) INSTALL_MOD_PATH=$(KERNEL_BUILD_DIR) modules) \
		3>&1 1>&2 2>&3 \
		&& (touch $(MFLAG))) \
	| tee  $(MOD_ERR_LOG)
	@sleep 2
	@(cat $(MOD_ERR_LOG) | \
		$(KERNEL_SRC_DIR)/scripts/chk_gcc_warn.pl \
			$(KERNEL_SRC_DIR) \
			$(KERNEL_SRC_DIR)/scripts/gcc_warn_filter.cfg) \
	 || (find $(KERNEL_BUILD_DIR) -name "*.ko" -exec rm -f {}  \; \
		&& rm -f $(MOD_ERR_LOG) && false)
	@rm $(MFLAG) 2> /dev/null

 
define modules_strip
$(KERNEL_CROSS_COMPILE)strip --strip-debug $(MOTO_MOD_INSTALL)/*.ko
endef

#NOTE: "strip" MUST be done for generated .ko files!!!
modules_install: __modules_install
	$(modules_strip)
__modules_install:
	make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) \
		DEPMOD=$(DEPMOD) \
		INSTALL_MOD_PATH=$(KERNEL_BUILD_DIR) \
		modules_install 
	find $(KERNEL_BUILD_DIR)/lib/modules -name "*.ko" -exec cp -f {} \
		  $(MOTO_MOD_INSTALL) \;

clean: ext_modules_clean kernel_clean
kernel_clean:
	make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) mrproper
	rm -f $(TARGET_DEFCONFIG)
	@rm -f $(KERNEL_CONF_OUT_DIR)/.*.txt

#
#----------------------------------------------------------------------------
# To use "make ext_modules" to buld external kernel modules
#----------------------------------------------------------------------------
# build external kernel modules
#
# NOTE: "strip" MUST be done for generated .ko files!!!
# =============================
ext_modules: tiwlan_drv 
	$(modules_strip)

# TODO:
# ext_modules_clean doesn't work 
# wlan, graphic, SMC drivers need to be updated to fix it
ext_modules_clean: tiwlan_drv_clean 

# wlan driver module
#-------------------
API_MAKE = env -u MAKECMDGOALS make PREFIX=$(KERNEL_BUILD_DIR) \
		CROSS=$(KERNEL_CROSS_COMPILE) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		PROCFAMILY=OMAP_3430 PROJROOT=$(PWD) \
		HOST_PLATFORM=zoom2 \
		KRNLSRC=$(KERNEL_SRC_DIR) KERNEL_DIR=$(KERNEL_BUILD_DIR)
WLAN_DRV_PATH = $(PWD)/system/wlan/ti/wilink_6_1/platforms/os/linux
tiwlan_drv:
	$(API_MAKE) -C $(WLAN_DRV_PATH)
	cp $(WLAN_DRV_PATH)/tiwlan_drv.ko $(MOTO_MOD_INSTALL)

tiwlan_drv_clean:
	$(API_MAKE) -C $(WLAN_DRV_PATH) clean

#
# graphics driver module
#-----------------------
GRAPHICS_DIR=$(TOPDIR)/motorola/bsp/graphics_drv
graphics_drv:
	make -C $(GRAPHICS_DIR) ARCH=arm $(KERN_FLAGS) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) KERNEL_SRC_DIR=$(KERNEL_SRC_DIR) KERNEL_BUILD_DIR=$(KERNEL_BUILD_DIR) INSTALL_DIR=$(MOTO_MOD_INSTALL) MODULE_DIR=$(GRAPHICS_DIR) graphics_drv

graphics_drv_clean:
	make -C $(GRAPHICS_DIR) ARCH=arm $(KERN_FLAGS) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) KERNEL_SRC_DIR=$(KERNEL_SRC_DIR) KERNEL_BUILD_DIR=$(KERNEL_BUILD_DIR) INSTALL_DIR=$(MOTO_MOD_INSTALL) MODULE_DIR=$(GRAPHICS_DIR) distclean

#
# SMC driver module
#-----------------------
SMC_DIR=$(TOPDIR)/motorola/bsp/smc_drv/build
SMC_OUT=$(KERNEL_CONF_OUT_DIR)/smc_drv
smc_drv:
	make -C $(SMC_DIR) ARCH=arm $(KERN_FLAGS) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) KERNEL_SRC_DIR=$(KERNEL_SRC_DIR) LINUX_KERNEL_DIR=$(KERNEL_BUILD_DIR) INSTALL_DIR=$(MOTO_MOD_INSTALL) MODULE_DIR=$(SMC_DIR)

smc_drv_clean:
	make -C $(SMC_DIR) ARCH=arm $(KERN_FLAGS) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) KERNEL_SRC_DIR=$(KERNEL_SRC_DIR) LINUX_KERNEL_DIR=$(KERNEL_BUILD_DIR) INSTALL_DIR=$(MOTO_MOD_INSTALL) MODULE_DIR=$(SMC_DIR) clean

