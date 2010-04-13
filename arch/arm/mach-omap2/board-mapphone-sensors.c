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
#include <linux/kxtf9.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/vib-gpio.h>

#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/keypad.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#define MAPPHONE_PROX_INT_GPIO		180
#define MAPPHONE_HF_NORTH_GPIO		10
#define MAPPHONE_HF_SOUTH_GPIO		111
#define MAPPHONE_AKM8973_INT_GPIO	175
#define MAPPHONE_AKM8973_RESET_GPIO	28
#define MAPPHONE_VIBRATOR_GPIO		181
#define MAPPHONE_KXTF9_INT_GPIO		22

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

static struct regulator *mapphone_kxtf9_regulator;
static int mapphone_kxtf9_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_kxtf9_regulator = reg;
	return 0;
}

static void mapphone_kxtf9_exit(void)
{
	regulator_put(mapphone_kxtf9_regulator);
}

static int mapphone_kxtf9_power_on(void)
{
	return regulator_enable(mapphone_kxtf9_regulator);
}

static int mapphone_kxtf9_power_off(void)
{
	if (mapphone_kxtf9_regulator)
		return regulator_disable(mapphone_kxtf9_regulator);
	return 0;
}

struct kxtf9_platform_data mapphone_kxtf9_data = {
	.init = mapphone_kxtf9_initialization,
	.exit = mapphone_kxtf9_exit,
	.power_on = mapphone_kxtf9_power_on,
	.power_off = mapphone_kxtf9_power_off,

	.min_interval  = 2,
	.poll_interval = 200,

	.g_range = KXTF9_G_8G,

	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,

	.negate_x = 1,
	.negate_y = 0,
	.negate_z = 1,

	.data_odr_init		= ODR12_5,
	.ctrl_reg1_init		= RES_12BIT | KXTF9_G_2G | TPE | WUFE | TDTE,
	.int_ctrl_init		= IEA | IEN,
	.tilt_timer_init	= 0x03,
	.engine_odr_init	= OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init		= 0x0A,
	.wuf_thresh_init	= 0x20,
	.tdt_timer_init		= 0x78,
	.tdt_h_thresh_init	= 0xB6,
	.tdt_l_thresh_init	= 0x1A,
	.tdt_tap_timer_init	= 0xA2,
	.tdt_total_timer_init	= 0x24,
	.tdt_latency_timer_init	= 0x28,
	.tdt_window_timer_init	= 0xA0,

	.gpio = MAPPHONE_KXTF9_INT_GPIO,
	.gesture = 0,
	.sensitivity_low = {
		0x50, 0xFF, 0xB8, 0x32, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_medium = {
		0x50, 0xFF, 0x68, 0x32, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_high = {
		0x78, 0xB6, 0x1A, 0xA2, 0x24, 0x28, 0xA0,
	},

};

static void __init mapphone_kxtf9_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_path(DT_PATH_ACCELEROMETER);
	if (node) {
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_X, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_x = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_Y, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_y = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_Z, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_z = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_X, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_x = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_Y, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_y = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_Z, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_z = *(u8 *)prop;

		prop = of_get_property(node,
		DT_PROP_ACCELEROMETER_SENS_LOW, &len);

		if (prop && len)
			memcpy(mapphone_kxtf9_data.sensitivity_low,
				(u8 *)prop, len);

		prop = of_get_property(node,
		DT_PROP_ACCELEROMETER_SENS_MEDIUM, &len);

		if (prop && len)
			memcpy(mapphone_kxtf9_data.sensitivity_medium,
			(u8 *)prop, len);

		prop = of_get_property(node,
		DT_PROP_ACCELEROMETER_SENS_HIGH, &len);

		if (prop && len)
			memcpy(mapphone_kxtf9_data.sensitivity_high,
				(u8 *)prop, len);
		of_node_put(node);
	}
#endif
	gpio_request(MAPPHONE_KXTF9_INT_GPIO, "kxtf9 accelerometer int");
	gpio_direction_input(MAPPHONE_KXTF9_INT_GPIO);
	omap_cfg_reg(AF9_34XX_GPIO22_DOWN);
}


static void __init mapphone_akm8973_init(void)
{
	gpio_request(MAPPHONE_AKM8973_RESET_GPIO, "akm8973 reset");
	gpio_direction_output(MAPPHONE_AKM8973_RESET_GPIO, 1);
	omap_cfg_reg(AB10_34XX_GPIO28_OUT);

	gpio_request(MAPPHONE_AKM8973_INT_GPIO, "akm8973 irq");
	gpio_direction_input(MAPPHONE_AKM8973_INT_GPIO);
	omap_cfg_reg(AC3_34XX_GPIO175);
}

struct platform_device kxtf9_platform_device = {
	.name = "kxtf9",
	.id = -1,
	.dev = {
		.platform_data = &mapphone_kxtf9_data,
	},
};

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
	&kxtf9_platform_device,
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
	mapphone_kxtf9_init();
	mapphone_sfh7743_init();
	mapphone_hall_effect_init();
	mapphone_vibrator_init();
	mapphone_akm8973_init();
	platform_add_devices(mapphone_sensors, ARRAY_SIZE(mapphone_sensors));
}
