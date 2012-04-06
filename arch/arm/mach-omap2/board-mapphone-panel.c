/*
 * linux/arch/arm/mach-omap2/board-mapphone-panel.c
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

#define MAPPHONE_DISPLAY_RESET_GPIO	136

struct regulator *display_regulator;

static int mapphone_panel_enable(struct omap_dss_device *dssdev)
{
	if (!display_regulator) {
		display_regulator = regulator_get(NULL, "vhvio");
		if (IS_ERR(display_regulator)) {
			printk(KERN_ERR "failed to get regulator for display");
			return PTR_ERR(display_regulator);
		}
#ifndef CONFIG_FB_OMAP2_MTD_LOGO
		regulator_enable(display_regulator);
		return 0;
#endif
	}

	regulator_enable(display_regulator);
	msleep(1);
	gpio_request(MAPPHONE_DISPLAY_RESET_GPIO, "display reset");
	gpio_direction_output(MAPPHONE_DISPLAY_RESET_GPIO, 1);
	msleep(5);
	gpio_set_value(MAPPHONE_DISPLAY_RESET_GPIO, 0);
	msleep(5);
	gpio_set_value(MAPPHONE_DISPLAY_RESET_GPIO, 1);
	msleep(10);

	return 0;
}

static void mapphone_panel_disable(struct omap_dss_device *dssdev)
{
	gpio_direction_output(MAPPHONE_DISPLAY_RESET_GPIO, 1);
	gpio_set_value(MAPPHONE_DISPLAY_RESET_GPIO, 0);
	msleep(1);
	regulator_disable(display_regulator);
}

static struct omapfb_platform_data mapphone_fb_data = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			{
				.format = OMAPFB_COLOR_ARGB32,
				.format_used = 1,
			},
		},
	},
};

static struct omap_dss_device mapphone_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DSI,
	.name = "lcd",
	.driver_name = "sholes-panel",
	.phy.dsi.clk_lane = 1,
	.phy.dsi.clk_pol = 0,
	.phy.dsi.data1_lane = 2,
	.phy.dsi.data1_pol = 0,
	.phy.dsi.data2_lane = 3,
	.phy.dsi.data2_pol = 0,
	.phy.dsi.ddr_clk_hz = 160000000,
	.phy.dsi.lp_clk_hz = 10000000,
	.reset_gpio = MAPPHONE_DISPLAY_RESET_GPIO,
	.platform_enable = mapphone_panel_enable,
	.platform_disable = mapphone_panel_disable,
};

static struct omap_dss_device *mapphone_dss_devices[] = {
	&mapphone_lcd_device,
};

static struct omap_dss_board_info mapphone_dss_data = {
	.num_devices = ARRAY_SIZE(mapphone_dss_devices),
	.devices = mapphone_dss_devices,
	.default_device = &mapphone_lcd_device,
};

struct platform_device mapphone_dss_device = {
	.name = "omapdss",
	.id = -1,
	.dev = {
		.platform_data = &mapphone_dss_data,
	},
};

void __init mapphone_panel_init(void)
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

	ret = gpio_request(MAPPHONE_DISPLAY_RESET_GPIO, "display reset");
	if (ret) {
		printk(KERN_ERR "failed to get display reset gpio\n");
		goto error;
	}

	platform_device_register(&mapphone_dss_device);
	return;

error:
	gpio_free(MAPPHONE_DISPLAY_RESET_GPIO);
}
