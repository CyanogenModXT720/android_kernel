/*
 * linux/arch/arm/mach-omap2/board-mapphone-sensors.c
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
#include <linux/input.h>
#include <linux/sfh7743.h>
#include <linux/bu52014hfv.h>
#include <linux/lis331dlh.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/vib-gpio.h>

#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/keypad.h>

#define MAPPHONE_PROX_INT_GPIO		180
#define MAPPHONE_HF_NORTH_GPIO		10
#define MAPPHONE_HF_SOUTH_GPIO		111
#define MAPPHONE_AKM8973_INT_GPIO	175
#define MAPPHONE_AKM8973_RESET_GPIO	28
#define MAPPHONE_VIBRATOR_GPIO		181

static struct regulator *mapphone_vibrator_regulator;
static int mapphone_vibrator_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vvib");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_vibrator_regulator = reg;
	return 0;
}

static void mapphone_vibrator_exit(void)
{
	regulator_put(mapphone_vibrator_regulator);
}

static int mapphone_vibrator_power_on(void)
{
	regulator_set_voltage(mapphone_vibrator_regulator, 3000000, 3000000);
	return regulator_enable(mapphone_vibrator_regulator);
}

static int mapphone_vibrator_power_off(void)
{
	if (mapphone_vibrator_regulator)
		return regulator_disable(mapphone_vibrator_regulator);
	return 0;
}

static struct vib_gpio_platform_data mapphone_vib_gpio_data = {
	.gpio = MAPPHONE_VIBRATOR_GPIO,
	.max_timeout = 15000,
	.active_low = 0,
	.initial_vibrate = 0,

	.init = mapphone_vibrator_initialization,
	.exit = mapphone_vibrator_exit,
	.power_on = mapphone_vibrator_power_on,
	.power_off = mapphone_vibrator_power_off,
};

static struct platform_device mapphone_vib_gpio = {
	.name           = "vib-gpio",
	.id             = -1,
	.dev            = {
		.platform_data  = &mapphone_vib_gpio_data,
	},
};

static struct regulator *mapphone_sfh7743_regulator;
static int mapphone_sfh7743_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vsdio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_sfh7743_regulator = reg;
	return 0;
}

static void mapphone_sfh7743_exit(void)
{
	regulator_put(mapphone_sfh7743_regulator);
}

static int mapphone_sfh7743_power_on(void)
{
	return regulator_enable(mapphone_sfh7743_regulator);
}

static int mapphone_sfh7743_power_off(void)
{
	if (mapphone_sfh7743_regulator)
		return regulator_disable(mapphone_sfh7743_regulator);
	return 0;
}

static struct sfh7743_platform_data mapphone_sfh7743_data = {
	.init = mapphone_sfh7743_initialization,
	.exit = mapphone_sfh7743_exit,
	.power_on = mapphone_sfh7743_power_on,
	.power_off = mapphone_sfh7743_power_off,

	.gpio = MAPPHONE_PROX_INT_GPIO,
};

static void __init mapphone_sfh7743_init(void)
{
	gpio_request(MAPPHONE_PROX_INT_GPIO, "sfh7743 proximity int");
	gpio_direction_input(MAPPHONE_PROX_INT_GPIO);
	omap_cfg_reg(Y3_34XX_GPIO180);
}


static struct bu52014hfv_platform_data bu52014hfv_platform_data = {
	.docked_north_gpio = MAPPHONE_HF_NORTH_GPIO,
	.docked_south_gpio = MAPPHONE_HF_SOUTH_GPIO,
	.north_is_desk = 1,
};

static struct regulator *mapphone_lis331dlh_regulator;
static int mapphone_lis331dlh_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_lis331dlh_regulator = reg;
	return 0;
}

static void mapphone_lis331dlh_exit(void)
{
	regulator_put(mapphone_lis331dlh_regulator);
}

static int mapphone_lis331dlh_power_on(void)
{
	return regulator_enable(mapphone_lis331dlh_regulator);
}

static int mapphone_lis331dlh_power_off(void)
{
	if (mapphone_lis331dlh_regulator)
		return regulator_disable(mapphone_lis331dlh_regulator);
	return 0;
}

struct lis331dlh_platform_data mapphone_lis331dlh_data = {
	.init = mapphone_lis331dlh_initialization,
	.exit = mapphone_lis331dlh_exit,
	.power_on = mapphone_lis331dlh_power_on,
	.power_off = mapphone_lis331dlh_power_off,

	.min_interval	= 1,
	.poll_interval	= 200,

	.g_range	= LIS331DLH_G_8G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,
};

static void __init mapphone_akm8973_init(void)
{
	gpio_request(MAPPHONE_AKM8973_RESET_GPIO, "akm8973 reset");
	gpio_direction_output(MAPPHONE_AKM8973_RESET_GPIO, 1);
	omap_cfg_reg(AB10_34XX_GPIO28_OUT);

	gpio_request(MAPPHONE_AKM8973_INT_GPIO, "akm8973 irq");
	gpio_direction_input(MAPPHONE_AKM8973_INT_GPIO);
	omap_cfg_reg(AC3_34XX_GPIO175);
}

struct platform_device sfh7743_platform_device = {
	.name = "sfh7743",
	.id = -1,
	.dev = {
		.platform_data = &mapphone_sfh7743_data,
	},
};

static struct platform_device omap3430_hall_effect_dock = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52014hfv_platform_data,
	},
};

static void mapphone_vibrator_init(void)
{
	gpio_request(MAPPHONE_VIBRATOR_GPIO, "vibrator");
	gpio_direction_output(MAPPHONE_VIBRATOR_GPIO, 0);
	omap_cfg_reg(Y4_34XX_GPIO181);
}

static struct platform_device *mapphone_sensors[] __initdata = {
	&sfh7743_platform_device,
	&omap3430_hall_effect_dock,
	&mapphone_vib_gpio,
};

static void mapphone_hall_effect_init(void)
{
	gpio_request(MAPPHONE_HF_NORTH_GPIO, "mapphone dock north");
	gpio_direction_input(MAPPHONE_HF_NORTH_GPIO);
	omap_cfg_reg(AG25_34XX_GPIO10);

	gpio_request(MAPPHONE_HF_SOUTH_GPIO, "mapphone dock south");
	gpio_direction_input(MAPPHONE_HF_SOUTH_GPIO);
	omap_cfg_reg(B26_34XX_GPIO111);
}

void __init mapphone_sensors_init(void)
{
	mapphone_sfh7743_init();
	mapphone_hall_effect_init();
	mapphone_vibrator_init();
	mapphone_akm8973_init();
	platform_add_devices(mapphone_sensors, ARRAY_SIZE(mapphone_sensors));
}
