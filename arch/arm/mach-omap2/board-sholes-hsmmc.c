/*
 * linux/arch/arm/mach-omap2/board-sdp-hsmmc.c
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Texas Instruments
 * Copyright (C) 2008-2009 Motorola, Inc.
 * Change mmc support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/* Date         Author          Comment
 * ===========  ==============  ==============================================
 * 30-Oct-2008  Motorola        Get MMC1 power supply
 *  2-Apr-2008  Motorola        Add code to support MOT_FEAT_MMC_TST
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

#if defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

#define CPCAP_VSDIO		0x192
#define VSDIO_3V                0x3B
#define VSDIO_18V               0x13
#define CPCAP_VWLAN2            0x198
#define VWLAN2_3V		0x4D
#define CPCAP_CLR		0x00

#define VMMC1_DEV_GRP		0x27
#define P1_DEV_GRP		0x20
#define VMMC1_DEDICATED		0x2A
#define VSEL_3V			0x02
#define VSEL_18V		0x00
#define TWL_GPIO_IMR1A		0x1C
#define TWL_GPIO_ISR1A		0x19
#define LDO_CLR			0x00
#define VSEL_S2_CLR		0x40
#define GPIO_0_BIT_POS		(1 << 0)
#define GPIO_1_BIT_POS		(1 << 1)
#define VSIM_DEV_GRP		0x37
#define VSIM_DEDICATED		0x3A
#define VMMC2_DEV_GRP		0x2B
#define VMMC2_DEDICATED		0x2E

struct res_handle *res_vsdio = NULL;
struct res_handle *res_vwlan2 = NULL;

#define OMAP2_CONTROL_DEVCONF1		(OMAP243X_CTRL_BASE \
					+ OMAP243X_CONTROL_DEVCONF1)
#define OMAP3_CONTROL_DEVCONF0		(OMAP343X_CTRL_BASE \
					+ OMAP343X_CONTROL_DEVCONF0)
#define OMAP3_CONTROL_DEVCONF1		(OMAP343X_CTRL_BASE \
					+ OMAP343X_CONTROL_DEVCONF1)

#define OMAP2_CONTROL_DEVCONF0_LBCLK	(1 << 24)
#define OMAP2_CONTROL_DEVCONF1_ACTOV	(1 << 31)

#define OMAP2_CONTROL_PBIAS_VMODE	(1 << 0)
#define OMAP2_CONTROL_PBIAS_VMODE1	(1 << 8)
#define OMAP2_CONTROL_PBIAS_PWRDNZ	(1 << 1)
#define OMAP2_CONTROL_PBIAS_SCTRL	(1 << 2)
#define OMAP2_CONTROL_PBIAS_PWRDNZ1	(1 << 9)
#define OMAP2_CONTROL_PBIAS_SCTRL1	(1 << 10)

#define GPIO_SIGNAL_MMC_DET 163

static const int mmc1_cd_gpio = OMAP_MAX_GPIO_LINES;		/* HACK!! */
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
	if (ret < 0) {
		goto err;
	}
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

extern unsigned short read_vwlan2();
extern int write_vwlan2(unsigned short value);

static int hsmmc_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	u32 vdd_sel = 0, devconf = 0, reg = 0;
	int ret = 0;

	/* REVISIT: Using address directly till the control.h defines
	 * are settled.
	 */
#define OMAP2_CONTROL_PBIAS 0x48002520
	if (power_on) {
		switch (1 << vdd) {
			case MMC_VDD_33_34:
			case MMC_VDD_32_33:
				/* turn power on CPCAP_VWLAN2 */
				write_vwlan2(0x4d);
				printk(KERN_ERR "\tPOWER ON: read_vwlan2=0x%x\n", read_vwlan2());
				break;
			default:
				printk(KERN_WARNING "CPCAP_VWLAN2 only support 3V!\n");
				ret = -1;
				break;
		}

		/* PBIASLITEPWRDNZ0 need to be 0b0 when ramping up, now make it 0b1 */
		omap_writel(omap_readl(OMAP2_CONTROL_PBIAS) | (1 << 1),
			OMAP2_CONTROL_PBIAS);
		/* if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0)) */
			/* PBIASLITEPWRDNZ1 need to be 0b0 when ramping up, now make it 0b1 */
			omap_writel(omap_readl(OMAP2_CONTROL_PBIAS) | (1 << 9),
				OMAP2_CONTROL_PBIAS);
		return ret;
	} else {
		/* Power OFF */
		/* PBIASLITEPWRDNZ0 need to be 0b0 when ramping up, now make it 0b0 */
		omap_writel(omap_readl(OMAP2_CONTROL_PBIAS) & ~(1 << 1),
					OMAP2_CONTROL_PBIAS);

		/* if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0)) */
			/* PBIASLITEPWRDNZ1 need to be 0b0 when ramping up, now make it 0b0 */
			omap_writel(omap_readl(OMAP2_CONTROL_PBIAS) & ~(1 << 9),
					OMAP2_CONTROL_PBIAS);
		write_vwlan2(1);
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

void __init sholes_hsmmc_init(void)
{
	int ret;

	omap_cfg_reg(G19_24XX_MMC_CLKO);
	omap_cfg_reg(H18_24XX_MMC_CMD);
	omap_cfg_reg(F20_24XX_MMC_DAT0);
	omap_cfg_reg(H14_24XX_MMC_DAT1);
	omap_cfg_reg(E19_24XX_MMC_DAT2);
	omap_cfg_reg(D19_24XX_MMC_DAT3);
	omap_cfg_reg(R21_3430_GPIO163);

	printk(KERN_ERR "\tCONTROL_PADCONF_MMC1_CLK     0x%x\n", omap_readl(0x48002144));
	printk(KERN_ERR "\tCONTROL_PADCONF_MMC1_DAT0    0x%x\n", omap_readl(0x48002148));
	printk(KERN_ERR "\tCONTROL_PADCONF_MMC1_DAT2    0x%x\n", omap_readl(0x4800214c));

	__raw_writel(0, IO_ADDRESS(0x48002144));
	__raw_writel(0x1000100, IO_ADDRESS(0x48002148));
	__raw_writel(0x1000100, IO_ADDRESS(0x4800214c));

	printk(KERN_ERR "\tCONTROL_PADCONF_MMC1_CLK     0x%x\n", omap_readl(0x48002144));
	printk(KERN_ERR "\tCONTROL_PADCONF_MMC1_DAT0    0x%x\n", omap_readl(0x48002148));
	printk(KERN_ERR "\tCONTROL_PADCONF_MMC1_DAT2    0x%x\n", omap_readl(0x4800214c));
	mmc1_data.slots[0].card_detect_irq = gpio_to_irq(GPIO_SIGNAL_MMC_DET);

	hsmmc_data[0] = &mmc1_data;
#if defined(CONFIG_OMAP_HS_MMC2) || defined(CONFIG_TIWLAN_SDIO)
	hsmmc_data[1] = &mmc2_data;
#endif
#if defined(CONFIG_OMAP_HS_MMC3) || defined(CONFIG_TIWLAN_SDIO)
	hsmmc_data[2] = &mmc3_data;
#endif
	omap2_init_mmc(hsmmc_data, OMAP34XX_NR_MMC);

/* google's cpcap framework not finished yet */
#if 0
	res_vwlan2 = resource_get ("LDO", LDO_VWLAN2);
#endif
}
EXPORT_SYMBOL(sholes_hsmmc_init);

static void __exit hsmmc_exit(void)
{
/* google's cpcap framework not finished yet */
#if 0
	if (res_vwlan2 != NULL) {
		resource_put (res_vwlan2);
		res_vwlan2 = NULL;
	}
#endif
}

#else

void __init sholes_hsmmc_init(void)
{

}

#endif
