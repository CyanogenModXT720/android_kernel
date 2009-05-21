/*
 * linux/arch/arm/mach-omap2/board-mapphone-hsmm.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * Derived from linux/arch/arm/mach-omap2/board-sdp-hsmmc.c
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/resource.h>

#include <mach/control.h>
#include <mach/hardware.h>
#include <mach/mmc.h>
#include <mach/board.h>
#include <mach/resource.h>
#include <mach/mux.h>
#include <mach/io.h>

#define GPIO_SIGNAL_MMC_DET 163

static const int mmc2_cd_gpio = OMAP_MAX_GPIO_LINES + 1;

static int hsmmc_card_detect(int irq)
{
	return gpio_get_value_cansleep(GPIO_SIGNAL_MMC_DET);
}

#ifdef CONFIG_OMAP_HS_MMC2
static int hsmmc2_card_detect(int irq)
{
	return gpio_get_value_cansleep(mmc2_cd_gpio);
}
#endif

/*
 * MMC Slot Initialization.
 */
static int hsmmc_late_init(struct device *dev)
{
	int ret = 0;

	/*
	 * Configure TWL4030 GPIO parameters for MMC hotplug irq
	 */
	ret = gpio_request(GPIO_SIGNAL_MMC_DET, NULL);
	if (ret < 0)
		goto err;
	ret = gpio_direction_input(GPIO_SIGNAL_MMC_DET);
	if (ret < 0) {
		gpio_free(GPIO_SIGNAL_MMC_DET);
		goto err;
	}

	return ret;
	/* seems gpio_debounce and gpio_debounce_time are not needed here */
err:
	dev_err(dev, "Failed to configure GPIO MMC_DET\n");
	return ret;
}

static void hsmmc_cleanup(struct device *dev)
{
	gpio_free(GPIO_SIGNAL_MMC_DET);
}

static int hsmmc2_late_init(struct device *dev)
{
	int ret = 0;
	return ret;
}

static void hsmmc2_cleanup(struct device *dev)
{
}

#ifdef CONFIG_PM

/*
 * To mask and unmask MMC Card Detect Interrupt
 * mask : 1
 * unmask : 0
 */
static int mask_cd_interrupt(int mask)
{
	return 0;
}

static int hsmmc_suspend(struct device *dev, int slot)
{
	int ret = 0;

	disable_irq(gpio_to_irq(GPIO_SIGNAL_MMC_DET));
	ret = mask_cd_interrupt(1);

	return ret;
}

static int hsmmc_resume(struct device *dev, int slot)
{
	int ret = 0;

	enable_irq(gpio_to_irq(GPIO_SIGNAL_MMC_DET));
	ret = mask_cd_interrupt(0);

	return ret;
}

static int hsmmc2_suspend(struct device *dev, int slot)
{
	int ret = 0;
	return ret;
}

static int hsmmc2_resume(struct device *dev, int slot)
{
	int ret = 0;
	return ret;
}

#endif

static int hsmmc_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	u32 vdd_sel = 0, devconf = 0, reg = 0;
	int ret = 0;

	if (power_on) {
		switch (1 << vdd) {
		case MMC_VDD_33_34:
		case MMC_VDD_32_33:
			/* TODO: turn power on CPCAP_VWLAN2 */
			break;
		default:
			printk(KERN_WARNING "CPCAP_VWLAN2 only support 3V!\n");
			ret = -1;
			break;
		}

		/* PBIASLITEPWRDNZ0 need to be 0b0 when ramping up, now make it 0b1 */
		omap_ctrl_writel(omap_ctrl_readl(OMAP343X_CONTROL_PBIAS_LITE)
				| (1 << 1), OMAP343X_CONTROL_PBIAS_LITE);

		/* PBIASLITEPWRDNZ1 need to be 0b0 when ramping up, now make it 0b1 */
		omap_ctrl_writel(omap_ctrl_readl(OMAP343X_CONTROL_PBIAS_LITE)
				| (1 << 9), OMAP343X_CONTROL_PBIAS_LITE);

		return ret;
	} else {
		/* Power OFF */
		/* PBIASLITEPWRDNZ0 need to be 0b0 when ramping up, now make it 0b0 */
		omap_ctrl_writel(omap_ctrl_readl(OMAP343X_CONTROL_PBIAS_LITE)
				& ~(1 << 1), OMAP343X_CONTROL_PBIAS_LITE);

		/* PBIASLITEPWRDNZ1 need to be 0b0 when ramping up, now make it 0b0 */
		omap_ctrl_writel(omap_ctrl_readl(OMAP343X_CONTROL_PBIAS_LITE)
				& ~(1 << 9), OMAP343X_CONTROL_PBIAS_LITE);
		/* TODO: turn off CPCAP_VWLAN2 */
	}

	return 0;
}

static int hsmmc2_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	return 0;
}

static int hsmmc3_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	/* Power to the slot is hard wired */
	return 0;
}

static struct omap_mmc_platform_data mmc1_data = {
	.nr_slots			= 1,
	.init				= hsmmc_late_init,
	.cleanup			= hsmmc_cleanup,
#ifdef CONFIG_PM
	.suspend			= hsmmc_suspend,
	.resume				= hsmmc_resume,
#endif
	.dma_mask			= 0xffffffff,
	.slots[0] = {
		.wires			= 8,
		.set_power		= hsmmc_set_power,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34 |
						MMC_VDD_165_195,
		.name			= "first slot",
		/* we get car_detect_irq later */
		.card_detect_irq	= 0,
		.card_detect            = hsmmc_card_detect,
	},
};

static struct omap_mmc_platform_data mmc2_data = {
	.nr_slots			= 1,
	.init				= hsmmc2_late_init,
	.cleanup			= hsmmc2_cleanup,
#ifdef CONFIG_PM
	.suspend			= hsmmc2_suspend,
	.resume				= hsmmc2_resume,
#endif
	.dma_mask			= 0xffffffff,
	.slots[0] = {
		.wires			= 4,
		.set_power		= hsmmc2_set_power,
		.ocr_mask		= MMC_VDD_165_195,
		.name			= "first slot",

		.card_detect_irq        = 0,
		.card_detect            = NULL,
	},
};

static struct omap_mmc_platform_data mmc3_data = {
	.nr_slots			= 1,
	.init				= NULL,
	.cleanup			= NULL,
	.dma_mask			= 0xffffffff,
	.slots[0] = {
		.wires			= 4,
		.set_power		= hsmmc3_set_power,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34,
		.name			= "first slot",

		.card_detect_irq        = 0,
		.card_detect            = NULL,
	},
};

static struct omap_mmc_platform_data *hsmmc_data[OMAP34XX_NR_MMC];

void __init mapphone_hsmmc_init(void)
{
	hsmmc_data[0] = &mmc1_data;
#if defined(CONFIG_OMAP_HS_MMC2) || defined(CONFIG_TIWLAN_SDIO)
	hsmmc_data[1] = &mmc2_data;
#endif
#if defined(CONFIG_OMAP_HS_MMC3) || defined(CONFIG_TIWLAN_SDIO)
	hsmmc_data[2] = &mmc3_data;
#endif
	omap2_init_mmc(hsmmc_data, OMAP34XX_NR_MMC);
}
