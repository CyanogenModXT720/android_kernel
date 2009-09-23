/*
 * linux/arch/arm/mach-omap2/board-sholest-panel.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/omapfb.h>

#include <mach/display.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/resource.h>

#define SHOLEST_DISPLAY_RESET_GPIO	136

#ifdef CONFIG_PANEL_HDTV
#define SHOLEST_HDMI_MUX_ENABLE_N_GPIO  69
#define SHOLEST_HDMI_MUX_SELECT_GPIO    7
#endif

struct regulator *display_regulator;

static int sholest_panel_enable(struct omap_dss_device *dssdev)
{
	int ret;

	/* pin the memory bus bw to the highest value */
	ret = resource_request("vdd2_opp", &dssdev->dev, 400000);
	if (ret)
		printk("%s: resource request failed\n", __func__);

	if (!display_regulator) {
		display_regulator = regulator_get(NULL, "vhvio");
		if (IS_ERR(display_regulator)) {
			printk(KERN_ERR "failed to get regulator for display");
			return PTR_ERR(display_regulator);
		}
		regulator_enable(display_regulator);
		return 0;
	}

	regulator_enable(display_regulator);
	msleep(1);
	gpio_request(SHOLEST_DISPLAY_RESET_GPIO, "display reset");
	gpio_direction_output(SHOLEST_DISPLAY_RESET_GPIO, 1);
	msleep(5);
	gpio_set_value(SHOLEST_DISPLAY_RESET_GPIO, 0);
	msleep(5);
	gpio_set_value(SHOLEST_DISPLAY_RESET_GPIO, 1);
	msleep(10);

	return 0;
}

static void sholest_panel_disable(struct omap_dss_device *dssdev)
{
	int ret;

	/* unpin the memory bus */
	ret = resource_release("vdd2_opp", &dssdev->dev);
	if (ret)
		printk("%s: resource request failed\n", __func__);

	gpio_direction_output(SHOLEST_DISPLAY_RESET_GPIO, 1);
	gpio_set_value(SHOLEST_DISPLAY_RESET_GPIO, 0);
	msleep(1);
	regulator_disable(display_regulator);
}

static struct omapfb_platform_data mapphone_fb_data = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			{
				.format = OMAPFB_COLOR_RGB565,
				.format_used = 1,
			},
		},
	},
};

static struct omap_dss_device sholest_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DSI,
	.name = "lcd",
	.driver_name = "sholes-panel",
	.phy.dsi.clk_lane = 3,
	.phy.dsi.clk_pol = 0,
	.phy.dsi.data1_lane = 1,
	.phy.dsi.data1_pol = 0,
	.phy.dsi.data2_lane = 2,
	.phy.dsi.data2_pol = 0,
	.phy.dsi.ddr_clk_hz = 160000000,
	.phy.dsi.lp_clk_hz = 10000000,
	.reset_gpio = SHOLEST_DISPLAY_RESET_GPIO,
	.platform_enable = sholest_panel_enable,
	.platform_disable = sholest_panel_disable,
};

#ifdef CONFIG_PANEL_HDTV
static int sholest_panel_enable_hdtv(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO "%s IN\n", __func__);

	omap_cfg_reg(AG22_34XX_DSS_DATA0);
	omap_cfg_reg(AH22_34XX_DSS_DATA1);
	omap_cfg_reg(AG23_34XX_DSS_DATA2);
	omap_cfg_reg(AH23_34XX_DSS_DATA3);
	omap_cfg_reg(AG24_34XX_DSS_DATA4);
	omap_cfg_reg(AH24_34XX_DSS_DATA5);

	gpio_set_value(SHOLEST_HDMI_MUX_SELECT_GPIO, 1);

	printk(KERN_INFO "%s OUT\n", __func__);
	/* backlight enable? */
	return 0;
}

static void sholest_panel_disable_hdtv(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO "%s IN\n", __func__);

	omap_cfg_reg(AG22_34XX_DSI_DX0);
	omap_cfg_reg(AH22_34XX_DSI_DY0);
	omap_cfg_reg(AG23_34XX_DSI_DX1);
	omap_cfg_reg(AH23_34XX_DSI_DY1);
	omap_cfg_reg(AG24_34XX_DSI_DX2);
	omap_cfg_reg(AH24_34XX_DSI_DY2);

	gpio_set_value(SHOLEST_HDMI_MUX_SELECT_GPIO, 0);

	printk(KERN_INFO "%s OUT\n", __func__);
}

static struct omap_dss_device sholest_hdtv_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "hdtv",
	.driver_name = "hdtv-panel",

	.phy.dpi.data_lines = 24,

	.panel.config = OMAP_DSS_LCD_TFT,

	.platform_enable = sholest_panel_enable_hdtv,
	.platform_disable = sholest_panel_disable_hdtv,
};
#endif

#ifdef CONFIG_TVOUT_SHOLEST
static int sholest_panel_enable_tv(struct omap_dss_device *dssdev)
{
	printk(KERN_WARNING "Enter sholest_panel_enable_tv\n");

	return 0;
}

static void sholest_panel_disable_tv(struct omap_dss_device *dssdev)
{
	printk(KERN_WARNING "Enter sholest_panel_disable_tv\n");
}

static struct omap_dss_device sholest_tvout_device = {
	.type = OMAP_DISPLAY_TYPE_VENC,
	.name = "tv",
	.driver_name = "venc",	
	.phy.venc.type   = OMAP_DSS_VENC_TYPE_COMPOSITE,
	.platform_enable  = sholest_panel_enable_tv,
	.platform_disable = sholest_panel_disable_tv,
};
#endif

static struct omap_dss_device *sholest_dss_devices[] = {
	&sholest_lcd_device,
#ifdef CONFIG_PANEL_HDTV
	&sholest_hdtv_device,
#endif
#ifdef CONFIG_TVOUT_SHOLEST
	&sholest_tvout_device,
#endif
};

static struct omap_dss_board_info sholest_dss_data = {
	.num_devices = ARRAY_SIZE(sholest_dss_devices),
	.devices = sholest_dss_devices,
	.default_device = &sholest_lcd_device,
};

struct platform_device sholest_dss_device = {
	.name = "omapdss",
	.id = -1,
	.dev = {
		.platform_data = &sholest_dss_data,
	},
};

void __init sholest_panel_init(void)
{
	int ret;

	omap_cfg_reg(AG22_34XX_DSI_DX0);
	omap_cfg_reg(AH22_34XX_DSI_DY0);
	omap_cfg_reg(AG23_34XX_DSI_DX1);
	omap_cfg_reg(AH23_34XX_DSI_DY1);
	omap_cfg_reg(AG24_34XX_DSI_DX2);
	omap_cfg_reg(AH24_34XX_DSI_DY2);
	/* disp reset b */
	omap_cfg_reg(AE4_34XX_GPIO136_OUT);

	omapfb_set_platform_data(&mapphone_fb_data);

	ret = gpio_request(SHOLEST_DISPLAY_RESET_GPIO, "display reset");
	if (ret) {
		printk(KERN_ERR "failed to get display reset gpio\n");
		goto error;
	}

	platform_device_register(&sholest_dss_device);
	return;

error:
	gpio_free(SHOLEST_DISPLAY_RESET_GPIO);
}
