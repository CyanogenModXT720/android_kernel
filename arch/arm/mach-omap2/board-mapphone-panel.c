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

#include <mach/display.h>
#include <mach/gpio.h>
#include <mach/mux.h>


#define MAPPHONE_DISPLAY_RESET_GPIO	136

static void mapphone_dsi_enable(int enable)
{
	/* XXX power managment to enable dsi block */
#if 0
	u8 ded_val, grp_val;


	if (omap_rev() <= OMAP3430_REV_ES1_0)
		return;

	if (enable) {
		ded_val = ENABLE_VPLL2_DEDICATED;
		grp_val = ENABLE_VPLL2_DEV_GRP;
	} else {
		ded_val = 0;
		grp_val = 0;
	}

	twl4030_i2c_write_u8(PM_RECEIVER, ded_val, TWL4030_VPLL2_DEDICATED);
	twl4030_i2c_write_u8(PM_RECEIVER, grp_val, TWL4030_VPLL2_DEV_GRP);
#endif
}

static int mapphone_dsi_power_up(void)
{
	mapphone_dsi_enable(1);
	return 0;
}

static void mapphone_dsi_power_down(void)
{
	mapphone_dsi_enable(0);
}

static int mapphone_panel_enable_lcd(struct omap_display *display)
{
	/* backlight enable? */
	return 0;
}

static void mapphone_panel_disable_lcd(struct omap_display *display)
{
#if 0
	/* backlight disable? */

#endif
}

static int mapphone_edisco_ctrl_enable(struct omap_display *display)
{
	gpio_request(MAPPHONE_DISPLAY_RESET_GPIO, "display reset");
	gpio_direction_output(MAPPHONE_DISPLAY_RESET_GPIO, 1);
	msleep(5);
	gpio_set_value(MAPPHONE_DISPLAY_RESET_GPIO, 0);
	msleep(5);
	gpio_set_value(MAPPHONE_DISPLAY_RESET_GPIO, 1);
	msleep(10);

	return 0;
}

static void mapphone_edisco_ctrl_disable(struct omap_display *display)
{
	gpio_direction_output(MAPPHONE_DISPLAY_RESET_GPIO, 1);
	gpio_set_value(MAPPHONE_DISPLAY_RESET_GPIO, 0);

	return;
}

static struct omap_dss_display_config mapphone_display_data_lcd = {
	.type = OMAP_DISPLAY_TYPE_DSI,
	.name = "lcd",
	.ctrl_name = "ctrl-edisco",
	.panel_name = "panel-sholes",
	.u.dsi.clk_lane = 1,
	.u.dsi.clk_pol = 0,
	.u.dsi.data1_lane = 2,
	.u.dsi.data1_pol = 0,
	.u.dsi.data2_lane = 3,
	.u.dsi.data2_pol = 0,
	.u.dsi.ddr_clk_hz = 150000000,
	.panel_enable = mapphone_panel_enable_lcd,
	.panel_disable = mapphone_panel_disable_lcd,
	.ctrl_enable = mapphone_edisco_ctrl_enable,
	.ctrl_disable = mapphone_edisco_ctrl_disable,
};

static struct omap_dss_board_info mapphone_dss_data = {
	.dsi_power_up = mapphone_dsi_power_up,
	.dsi_power_down = mapphone_dsi_power_down,
	.num_displays = 1,
	.displays = {
		&mapphone_display_data_lcd,
	}
};

static struct platform_device mapphone_dss_device = {
	.name = "omapdss",
	.id = -1,
	.dev = {
		.platform_data = &mapphone_dss_data,
	},
};

void __init mapphone_panel_init(void)
{
	int ret;

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
