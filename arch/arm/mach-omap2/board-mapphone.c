/*
 * linux/arch/arm/mach-omap2/board-mapphone.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * Modified from mach-omap3/board-3430sdp.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/reboot.h>
#include <linux/qtouch_obp_ts.h>
#include <linux/led-cpcap-lm3554.h>
#include <linux/led-lm3530.h>
#include <linux/usb/omap.h>
#include <linux/wl127x-rfkill.h>
#include <linux/wl127x-test.h>
#include <linux/omap_mdm_ctrl.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board-mapphone.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/gpmc.h>
#include <mach/usb.h>
#include <linux/delay.h>
#include <mach/control.h>
#include <mach/hdq.h>
#include <mach/system.h>
#include <linux/usb/android.h>
#include <linux/wakelock.h>

#include "cm-regbits-34xx.h"

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#include "pm.h"
#include "prm-regbits-34xx.h"
#include "smartreflex.h"
#include "omap3-opp.h"
#include "sdram-toshiba-hynix-numonyx.h"
#include "prcm-common.h"
#include "cm.h"
#include "clock.h"

#ifdef CONFIG_VIDEO_OLDOMAP3
#include <media/v4l2-int-device.h>
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#include <media/mt9p012.h>

#endif
#ifdef CONFIG_VIDEO_OMAP3_HPLENS
#include <../drivers/media/video/hplens.h>
#endif
#endif

#define MAPPHONE_IPC_USB_SUSP_GPIO	142
#define MAPPHONE_AP_TO_BP_FLASH_EN_GPIO	157
#define MAPPHONE_TOUCH_RESET_N_GPIO	164
#define MAPPHONE_TOUCH_INT_GPIO		99
#define MAPPHONE_LM_3530_INT_GPIO	92
#define MAPPHONE_AKM8973_INT_GPIO	175
#define MAPPHONE_WL1271_NSHUTDOWN_GPIO	179
#define MAPPHONE_WL1271_WAKE_GPIO	8
#define MAPPHONE_WL1271_HOSTWAKE_GPIO	178
#define MAPPHONE_AUDIO_PATH_GPIO	143
#define MAPPHONE_BP_READY_AP_GPIO	141
#define MAPPHONE_BP_READY2_AP_GPIO	59
#define MAPPHONE_BP_RESOUT_GPIO		139
#define MAPPHONE_BP_PWRON_GPIO		137
#define MAPPHONE_AP_TO_BP_PSHOLD_GPIO	138
#define MAPPHONE_AP_TO_BP_FLASH_EN_GPIO	157
#define MAPPHONE_POWER_OFF_GPIO		176
#define MAPPHONE_BPWAKE_STROBE_GPIO	157
#define MAPPHONE_APWAKE_TRIGGER_GPIO	141
#define DIE_ID_REG_BASE			(L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET		0x218
#define MAX_USB_SERIAL_NUM		17
#define MAPPHONE_VENDOR_ID		0x22B8
#define MAPPHONE_PRODUCT_ID		0x41D9
#define MAPPHONE_ADB_PRODUCT_ID		0x41DB
#define FACTORY_PRODUCT_ID		0x41E3
#define FACTORY_ADB_PRODUCT_ID		0x41E2

#define MAPPHONE_MMCPROBE_ENABLED 0

static char device_serial[MAX_USB_SERIAL_NUM];
char *bp_model = "CDMA";

static struct omap_opp mapphone_mpu_rate_table[] = {
	{0, 0, 0},
	/*OPP1*/
	{S125M, VDD1_OPP1, 0x20},
	/*OPP2*/
	{S250M, VDD1_OPP2, 0x27},
	/*OPP3*/
	{S500M, VDD1_OPP3, 0x32},
	/*OPP4*/
	{S550M, VDD1_OPP4, 0x38},
	/*OPP5*/
	{S600M, VDD1_OPP5, 0x3E},
};

#define S80M 80000000
#define S160M 160000000

static struct omap_opp mapphone_l3_rate_table[] = {
	{0, 0, 0},
	/*OPP1*/
	{0, VDD2_OPP1, 0x20},
	/*OPP2*/
	{S80M, VDD2_OPP2, 0x27},
	/*OPP3*/
	{S160M, VDD2_OPP3, 0x2E},
};

static struct omap_opp mapphone_dsp_rate_table[] = {
	{0, 0, 0},
	/*OPP1*/
	{S90M, VDD1_OPP1, 0x20},
	/*OPP2*/
	{S180M, VDD1_OPP2, 0x27},
	/*OPP3*/
	{S360M, VDD1_OPP3, 0x32},
	/*OPP4*/
	{S400M, VDD1_OPP4, 0x38},
	/*OPP5*/
	{S430M, VDD1_OPP5, 0x3E},
};

static void __init mapphone_init_irq(void)
{
	omap2_init_common_hw(JEDEC_JESD209A_sdrc_params,
			mapphone_mpu_rate_table, mapphone_dsp_rate_table,
			mapphone_l3_rate_table);
	omap_init_irq();
#ifdef CONFIG_OMAP3_PM
	scm_clk_init();
#endif
	omap_gpio_init();
}

#define BOOT_MODE_MAX_LEN 30
static char boot_mode[BOOT_MODE_MAX_LEN+1];
int __init board_boot_mode_init(char *s)

{
	strncpy(boot_mode, s, BOOT_MODE_MAX_LEN);

	printk(KERN_INFO "boot_mode=%s\n", boot_mode);

	return 1;
}
__setup("androidboot.mode=", board_boot_mode_init);



static struct android_usb_platform_data andusb_plat = {
	.vendor_id      = 0x22b8,
	.product_id     = 0x41DA,
	.adb_product_id = 0x41DA,
	.product_name   = "A853",
	.manufacturer_name	= "Motorola",
	.serial_number		= device_serial,
};

static struct platform_device androidusb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data	= &andusb_plat,
	},
};

static struct usb_mass_storage_platform_data usbms_plat = {
	.vendor			= "Motorola",
	.product		= "A853",
	.release		= 1,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &usbms_plat,
	},
};

static int cpcap_usb_connected_probe(struct platform_device *pdev)
{
	android_usb_set_connected(1);
	return 0;
}

static int cpcap_usb_connected_remove(struct platform_device *pdev)
{
	android_usb_set_connected(0);
	return 0;
}

static struct platform_driver cpcap_usb_connected_driver = {
	.probe		= cpcap_usb_connected_probe,
	.remove		= cpcap_usb_connected_remove,
	.driver		= {
		.name	= "cpcap_usb_connected",
		.owner	= THIS_MODULE,
	},
};

static void mapphone_gadget_init(void)
{
	unsigned int val[2];
	unsigned int reg;

	reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;
	val[0] = omap_readl(reg);
	val[1] = omap_readl(reg + 4);

	snprintf(device_serial, MAX_USB_SERIAL_NUM, "%08X%08X", val[1], val[0]);

	if (!strcmp(boot_mode, "factorycable"))
		andusb_plat.factory_enabled = 1;
	else
		andusb_plat.factory_enabled = 0;

	andusb_plat.vendor_id = MAPPHONE_VENDOR_ID;

	/* check powerup reason - To be added once kernel support is available*/
	if (andusb_plat.factory_enabled) {
		andusb_plat.product_id = FACTORY_PRODUCT_ID;
		andusb_plat.adb_product_id = FACTORY_ADB_PRODUCT_ID;
	} else {
		andusb_plat.product_id = MAPPHONE_PRODUCT_ID;
		andusb_plat.adb_product_id = MAPPHONE_ADB_PRODUCT_ID;
	}
	platform_device_register(&androidusb_device);
	platform_device_register(&usb_mass_storage_device);
	platform_driver_register(&cpcap_usb_connected_driver);
}

static void mapphone_audio_init(void)
{
	gpio_request(MAPPHONE_AUDIO_PATH_GPIO, "mapphone audio path");

	omap_cfg_reg(P21_OMAP34XX_MCBSP2_FSX);
	omap_cfg_reg(N21_OMAP34XX_MCBSP2_CLKX);
	omap_cfg_reg(R21_OMAP34XX_MCBSP2_DR);
	omap_cfg_reg(M21_OMAP34XX_MCBSP2_DX);
	omap_cfg_reg(K26_OMAP34XX_MCBSP3_FSX);
	omap_cfg_reg(W21_OMAP34XX_MCBSP3_CLKX);
	omap_cfg_reg(U21_OMAP34XX_MCBSP3_DR);
	omap_cfg_reg(V21_OMAP34XX_MCBSP3_DX);

	gpio_direction_output(MAPPHONE_AUDIO_PATH_GPIO, 1);
	omap_cfg_reg(AE5_34XX_GPIO143);
}

static struct omap_uart_config mapphone_uart_config __initdata = {
	.enabled_uarts = ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_board_config_kernel mapphone_config[] __initdata = {
	{OMAP_TAG_UART,		&mapphone_uart_config },
};

static int mapphone_touch_reset(void)
{
	gpio_direction_output(MAPPHONE_TOUCH_RESET_N_GPIO, 1);
	msleep(1);
	gpio_set_value(MAPPHONE_TOUCH_RESET_N_GPIO, 0);
	msleep(20);
	gpio_set_value(MAPPHONE_TOUCH_RESET_N_GPIO, 1);
	msleep(20);

	return 0;
}

/* These are for test event-injection purposes only */
static struct vkey mapphone_touch_vkeys[] = {
	{
		.code		= KEY_BACK,
	},
	{
		.code		= KEY_MENU,
	},
	{
		.code		= KEY_HOME,
	},
	{
		.code		= KEY_SEARCH,
	},
};

static ssize_t mapphone_virtual_keys_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	/* center: x: home: 55, menu: 185, back: 305, search 425, y: 835 */
	/* keys are specified by setting the x,y of the center, the width,
	 * and the height, as such keycode:center_x:center_y:width:height */
	return sprintf(buf, __stringify(EV_KEY) ":"
		       __stringify(KEY_BACK) ":32:906:63:57"
		       ":" __stringify(EV_KEY) ":"
		       __stringify(KEY_MENU) ":162:906:89:57"
		       ":" __stringify(EV_KEY) ":"
		       __stringify(KEY_HOME) ":292:906:89:57"
		       ":" __stringify(EV_KEY) ":"
		       __stringify(KEY_SEARCH) ":439:906:63:57"
		       "\n");
}
static struct kobj_attribute mapphone_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.qtouch-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &mapphone_virtual_keys_show,
};

static struct attribute *mapphone_properties_attrs[] = {
	&mapphone_virtual_keys_attr.attr,
	NULL,
};

static struct attribute_group mapphone_properties_attr_group = {
	.attrs = mapphone_properties_attrs,
};

static struct qtouch_ts_platform_data mapphone_ts_platform_data;

static void mapphone_touch_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *touch_node;
	const void *touch_prop;
	int len = 0;

	if ((touch_node = of_find_node_by_path(DT_PATH_TOUCH))) {
		if ((touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_KEYMAP, &len)) \
			&& len && (0 == len % sizeof(struct vkey))) {
			mapphone_ts_platform_data.vkeys.count = len / sizeof(struct vkey);
			mapphone_ts_platform_data.vkeys.keys = (struct vkey *)touch_prop;
		}
		of_node_put(touch_node);
	}
#endif

	gpio_request(MAPPHONE_TOUCH_RESET_N_GPIO, "mapphone touch reset");
	gpio_direction_output(MAPPHONE_TOUCH_RESET_N_GPIO, 1);
	omap_cfg_reg(H19_34XX_GPIO164_OUT);

	gpio_request(MAPPHONE_TOUCH_INT_GPIO, "mapphone touch irq");
	gpio_direction_input(MAPPHONE_TOUCH_INT_GPIO);
	omap_cfg_reg(AG17_34XX_GPIO99);
}

static void mapphone_als_init(void)
{
	printk(KERN_INFO "%s:Initializing\n", __func__);
	gpio_request(MAPPHONE_LM_3530_INT_GPIO, "mapphone als int");
	gpio_direction_input(MAPPHONE_LM_3530_INT_GPIO);
	omap_cfg_reg(AC27_34XX_GPIO92);
}
static struct qtm_touch_keyarray_cfg mapphone_key_array_data[] = {
	{
		.ctrl = 0,
		.x_origin = 0,
		.y_origin = 0,
		.x_size = 0,
		.y_size = 0,
		.aks_cfg = 0,
		.burst_len = 0,
		.tch_det_thr = 0,
		.tch_det_int = 0,
		.rsvd1 = 0,
		.rsvd2 = 0,
	},
	{
		.ctrl = 0,
		.x_origin = 0,
		.y_origin = 0,
		.x_size = 0,
		.y_size = 0,
		.aks_cfg = 0,
		.burst_len = 0,
		.tch_det_thr = 0,
		.tch_det_int = 0,
		.rsvd1 = 0,
		.rsvd2 = 0,
	},
};

static struct qtouch_ts_platform_data mapphone_ts_platform_data = {
	.irqflags	= (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW),
	.flags		= (QTOUCH_SWAP_XY |
			   QTOUCH_USE_MULTITOUCH |
			   QTOUCH_CFG_BACKUPNV |
			   QTOUCH_EEPROM_CHECKSUM),
	.abs_min_x	= 20,
	.abs_max_x	= 1004,
	.abs_min_y	= 0,
	.abs_max_y	= 960,
	.abs_min_p	= 0,
	.abs_max_p	= 255,
	.abs_min_w	= 0,
	.abs_max_w	= 15,
	.x_delta	= 400,
	.y_delta	= 250,
	.nv_checksum	= 0xfaf5,
	.fuzz_x		= 0,
	.fuzz_y		= 0,
	.fuzz_p		= 2,
	.fuzz_w		= 2,
	.hw_reset	= mapphone_touch_reset,
	.power_cfg	= {
		.idle_acq_int	= 0xff,
		.active_acq_int	= 0xff,
		.active_idle_to	= 0x01,
	},
	.acquire_cfg	= {
		.charge_time	= 12,
		.atouch_drift	= 5,
		.touch_drift	= 20,
		.drift_susp	= 20,
		.touch_autocal	= 0x96,
		.sync		= 0,
	},
	.multi_touch_cfg	= {
		.ctrl		= 0x0b,
		.x_origin	= 0,
		.y_origin	= 0,
		.x_size		= 12,
		.y_size		= 7,
		.aks_cfg	= 0,
		.burst_len	= 0x40,
		.tch_det_thr	= 0x12,
		.tch_det_int	= 0x2,
		.mov_hyst_init	= 0xe,
		.mov_hyst_next	= 0xe,
		.mov_filter	= 0x9,
		.num_touch	= 2,
		.merge_hyst	= 0,
		.merge_thresh	= 3,
		.amp_hyst = 2,
		.x_res = 0x0000,
		.y_res = 0x0000,
		.x_low_clip = 0x00,
		.x_high_clip = 0x00,
		.y_low_clip = 0x00,
		.y_high_clip = 0x00,
	},
	.linear_tbl_cfg = {
		.ctrl = 0x01,
		.x_offset = 0x0000,
		.x_segment = {
			0x48, 0x3f, 0x3c, 0x3E,
			0x3f, 0x3e, 0x3e, 0x3e,
			0x3f, 0x42, 0x41, 0x3f,
			0x41, 0x40, 0x41, 0x46
		},
		.y_offset = 0x0000,
		.y_segment = {
			0x44, 0x38, 0x37, 0x3e,
			0x3e, 0x41, 0x41, 0x3f,
			0x42, 0x41, 0x42, 0x42,
			0x41, 0x3f, 0x41, 0x45
		},
	},
	.grip_suppression_cfg = {
		.ctrl		= 0x00,
		.xlogrip	= 0x00,
		.xhigrip	= 0x00,
		.ylogrip	= 0x00,
		.yhigrip	= 0x00,
		.maxtchs	= 0x00,
		.reserve0	= 0x00,
		.szthr1		= 0x00,
		.szthr2		= 0x00,
		.shpthr1	= 0x00,
		.shpthr2	= 0x00,
	},
	.noise1_suppression_cfg = {
		.ctrl = 0x01,
		.reserved = 0x01,
		.atchthr = 0x64,
		.duty_cycle = 0x08,
	},
	.key_array      = {
		.cfg		= mapphone_key_array_data,
		.num_keys   = ARRAY_SIZE(mapphone_key_array_data),
	},
	.vkeys			= {
		.keys		= mapphone_touch_vkeys,
		.count		= ARRAY_SIZE(mapphone_touch_vkeys),
		.start		= 961,
	},
};

static struct lm3530_platform_data omap3430_als_light_data = {
	.power_up_gen_config = 0x0b,
	.gen_config = 0x3b,
	.als_config = 0x6c,
	.brightness_ramp = 0x00,
	.als_zone_info = 0x00,
	.als_resistor_sel = 0x31,
	.brightness_control = 0x00,
	.zone_boundary_0 = 0x02,
	.zone_boundary_1 = 0x10,
	.zone_boundary_2 = 0x43,
	.zone_boundary_3 = 0xfc,
	.zone_target_0 = 0x51,
	.zone_target_1 = 0x6c,
	.zone_target_2 = 0x6c,
	.zone_target_3 = 0x6c,
	.zone_target_4 = 0x7e,
	.manual_current = 0x33,
	.upper_curr_sel = 6,
	.lower_curr_sel = 3,
	.lens_loss_coeff = 6,
};

static struct lm3554_platform_data mapphone_camera_flash = {
	.torch_brightness_def = 0xa0,
	.flash_brightness_def = 0x78,
	.flash_duration_def = 0x28,
	.config_reg_1_def = 0xe0,
	.config_reg_2_def = 0xf0,
	.vin_monitor_def = 0x01,
	.gpio_reg_def = 0x0,
};

static struct i2c_board_info __initdata mapphone_i2c_bus1_board_info[] = {
	{
		I2C_BOARD_INFO(QTOUCH_TS_NAME, 0x11),
		.platform_data = &mapphone_ts_platform_data,
		.irq = OMAP_GPIO_IRQ(MAPPHONE_TOUCH_INT_GPIO),
	},
	{
		I2C_BOARD_INFO(LD_LM3530_NAME, 0x38),
		.platform_data = &omap3430_als_light_data,
		.irq = OMAP_GPIO_IRQ(MAPPHONE_LM_3530_INT_GPIO),
	},
};

extern struct lis331dlh_platform_data mapphone_lis331dlh_data;

static struct i2c_board_info __initdata mapphone_i2c_bus2_board_info[] = {
	{
		I2C_BOARD_INFO("akm8973", 0x1C),
		.irq = OMAP_GPIO_IRQ(MAPPHONE_AKM8973_INT_GPIO),
	},
	{
		I2C_BOARD_INFO("lis331dlh", 0x19),
		.platform_data = &mapphone_lis331dlh_data,
	},
};

static struct i2c_board_info __initdata mapphone_i2c_bus3_board_info[] = {
	{
		I2C_BOARD_INFO("lm3554_led", 0x53),
		.platform_data = &mapphone_camera_flash,
	},
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
	{
		I2C_BOARD_INFO("mt9p012", 0x36),
		.platform_data = &mapphone_mt9p012_platform_data,
	},
#endif
#ifdef CONFIG_VIDEO_OMAP3_HPLENS
	{
		I2C_BOARD_INFO("HP_GEN_LENS", 0x04),
		.platform_data = &mapphone_hplens_platform_data,
	},
#endif
};

static int __init mapphone_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, mapphone_i2c_bus1_board_info,
			      ARRAY_SIZE(mapphone_i2c_bus1_board_info));
	omap_register_i2c_bus(2, 400, mapphone_i2c_bus2_board_info,
			      ARRAY_SIZE(mapphone_i2c_bus2_board_info));
	omap_register_i2c_bus(3, 400, mapphone_i2c_bus3_board_info,
			      ARRAY_SIZE(mapphone_i2c_bus3_board_info));
	return 0;
}

arch_initcall(mapphone_i2c_init);

extern void __init mapphone_spi_init(void);
extern void __init mapphone_flash_init(void);
extern void __init mapphone_gpio_iomux_init(void);


#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)

static int mapphone_usb_port_startup(struct platform_device *dev, int port)
{
	int r;

	if (port == 2) {
		r = gpio_request(MAPPHONE_IPC_USB_SUSP_GPIO, "ipc_usb_susp");
		if (r < 0) {
			printk(KERN_WARNING "Could not request GPIO %d"
			       " for IPC_USB_SUSP\n",
			       MAPPHONE_IPC_USB_SUSP_GPIO);
			return r;
		}
		gpio_direction_output(MAPPHONE_IPC_USB_SUSP_GPIO, 0);
	} else {
		return -EINVAL;
	}
	return 0;
}

static void mapphone_usb_port_shutdown(struct platform_device *dev, int port)
{
	if (port == 2)
		gpio_free(MAPPHONE_IPC_USB_SUSP_GPIO);
}


static void mapphone_usb_port_suspend(struct platform_device *dev,
				    int port, int suspend)
{
	if (port == 2)
		gpio_set_value(MAPPHONE_IPC_USB_SUSP_GPIO, suspend);
}


static struct omap_usb_port_data usb_port_data[] = {
	[0] = { .flags = 0x0, }, /* disabled */
	[1] = { .flags = 0x0, }, /* disabled */
	[2] = {
		.flags = OMAP_USB_PORT_FLAG_ENABLED |
			OMAP_USB_PORT_FLAG_AUTOIDLE |
			OMAP_USB_PORT_FLAG_NOBITSTUFF,
		.mode = OMAP_USB_PORT_MODE_UTMI_PHY_4PIN,
		.startup = mapphone_usb_port_startup,
		.shutdown = mapphone_usb_port_shutdown,
		.suspend = mapphone_usb_port_suspend,
	},
};

static int omap_ohci_bus_check_ctrl_standby(void);
static struct omap_usb_platform_data usb_platform_data = {
	.port_data = usb_port_data,
	.num_ports = ARRAY_SIZE(usb_port_data),
	.usbhost_standby_status	= omap_ohci_bus_check_ctrl_standby,
};

static struct resource ehci_resources[] = {
	[0] = {
		.start	= OMAP34XX_HSUSB_HOST_BASE + 0x800,
		.end	= OMAP34XX_HSUSB_HOST_BASE + 0x800 + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {         /* general IRQ */
		.start	= INT_34XX_EHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static u64 ehci_dmamask = ~(u32)0;
static struct platform_device ehci_device = {
	.name		= "ehci-omap",
	.id		= 0,
	.dev = {
		.dma_mask		= &ehci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &usb_platform_data,
	},
	.num_resources	= ARRAY_SIZE(ehci_resources),
	.resource	= ehci_resources,
};
#endif

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE) || defined(CONFIG_USB_EHCI_HCD)
static int omap_ohci_bus_check_ctrl_standby(void)
{
	u32 val;

	val = cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, CM_IDLEST);
	if (val & OMAP3430ES2_ST_USBHOST_STDBY_MASK)
		return 1;
	else
		return 0;
}
#endif

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)


static struct resource ohci_resources[] = {
	[0] = {
		.start	= OMAP34XX_HSUSB_HOST_BASE + 0x400,
		.end	= OMAP34XX_HSUSB_HOST_BASE + 0x400 + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {         /* general IRQ */
		.start	= INT_34XX_OHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static u64 ohci_dmamask = ~(u32)0;

static struct omap_usb_config dummy_usb_config = {
	.usbhost_standby_status	= omap_ohci_bus_check_ctrl_standby,
	.usb_remote_wake_gpio = MAPPHONE_BP_READY2_AP_GPIO,
};

static struct platform_device ohci_device = {
	.name		= "ohci",
	.id		= 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data	= &dummy_usb_config,
	},
	.num_resources	= ARRAY_SIZE(ohci_resources),
	.resource	= ohci_resources,
};
#endif /* OHCI specific data */


static void __init mapphone_ehci_init(void)
{
	omap_cfg_reg(AF5_34XX_GPIO142);		/*  IPC_USB_SUSP      */
	omap_cfg_reg(AD1_3430_USB3FS_PHY_MM3_RXRCV);
	omap_cfg_reg(AD2_3430_USB3FS_PHY_MM3_TXDAT);
	omap_cfg_reg(AC1_3430_USB3FS_PHY_MM3_TXEN_N);
	omap_cfg_reg(AE1_3430_USB3FS_PHY_MM3_TXSE0);

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
	platform_device_register(&ehci_device);
#endif
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	if (is_cdma_phone())
		platform_device_register(&ohci_device);
#endif
}

static void __init mapphone_sdrc_init(void)
{
	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_cfg_reg(H16_34XX_SDRC_CKE0);
	omap_cfg_reg(H17_34XX_SDRC_CKE1);
}

static void __init mapphone_serial_init(void)
{
	omap_cfg_reg(AA8_3430_UART1_TX);
	omap_cfg_reg(Y8_3430_UART1_RX);
	omap_cfg_reg(AA9_3430_UART1_RTS);
	omap_cfg_reg(W8_3430_UART1_CTS);
	omap_cfg_reg(AA25_34XX_UART2_TX);
	omap_cfg_reg(AD25_34XX_UART2_RX);
	omap_cfg_reg(AB25_34XX_UART2_RTS);
	omap_cfg_reg(AB26_34XX_UART2_CTS);

	omap_serial_init(MAPPHONE_BPWAKE_STROBE_GPIO, 0x01);
}

/* SMPS I2C voltage control register Address for VDD1 */
#define MAPPHONE_R_VDD1_SR_CONTROL		0x00
/* SMPS I2C voltage control register Address for VDD2 */
#define MAPPHONE_R_VDD2_SR_CONTROL		0x00
/* SMPS I2C Address for VDD1 */
#define MAPPHONE_R_SRI2C_SLAVE_ADDR_SA0		0x1
/* SMPS I2C Address for VDD2 */
#define MAPPHONE_R_SRI2C_SLAVE_ADDR_SA1		0x2
/* SMPS I2C voltage control register Address for VDD1, used for SR command */
#define MAPPHONE_R_SMPS_VOL_CNTL_CMDRA0		0x01
/* SMPS I2C voltage control register Address for VDD2, used for SR command */
#define MAPPHONE_R_SMPS_VOL_CNTL_CMDRA1		0x01

static struct prm_setup_vc mapphone_prm_setup = {
	.clksetup = 0x4c,
	.voltsetup_time1 = 0x94,
	.voltsetup_time2 = 0x94,
	.voltoffset = 0x0,
	.voltsetup2 = 0x0,
	.vdd0_on = 0x65,
	.vdd0_onlp = 0x45,
	.vdd0_ret = 0x19,
	.vdd0_off = 0x00,
	.vdd1_on = 0x65,
	.vdd1_onlp = 0x45,
	.vdd1_ret = 0x19,
	.vdd1_off = 0x00,
	.i2c_slave_ra = (MAPPHONE_R_SRI2C_SLAVE_ADDR_SA1 <<
			OMAP3430_SMPS_SA1_SHIFT) |
			(MAPPHONE_R_SRI2C_SLAVE_ADDR_SA0 <<
			 OMAP3430_SMPS_SA0_SHIFT),
	.vdd_vol_ra = (MAPPHONE_R_VDD2_SR_CONTROL << OMAP3430_VOLRA1_SHIFT) |
			(MAPPHONE_R_VDD1_SR_CONTROL << OMAP3430_VOLRA0_SHIFT),
	/* vdd_vol_ra controls both cmd and vol, set the address equal */
	.vdd_cmd_ra = (MAPPHONE_R_SMPS_VOL_CNTL_CMDRA1 << OMAP3430_CMDRA1_SHIFT) |
		(MAPPHONE_R_SMPS_VOL_CNTL_CMDRA0 << OMAP3430_CMDRA0_SHIFT),
	.vdd_ch_conf = OMAP3430_CMD1 | OMAP3430_RACEN0 |
			OMAP3430_PRM_VC_CH_CONF_SA1 | OMAP3430_RACEN1 |
			OMAP3430_RAV1 | OMAP3430_RAC1, OMAP3430_GR_MOD,
	.vdd_i2c_cfg = OMAP3430_HSEN,
};

#define R_SMPS_VOL_OPP1_RA0		0x02
#define R_SMPS_VOL_OPP1_RA1		0x02
#define R_SMPS_VOL_OPP2_RA0		0x03
#define R_SMPS_VOL_OPP2_RA1		0x03


#ifdef CONFIG_OMAP_SMARTREFLEX
int mapphone_voltagescale_vcbypass(u32 target_opp, u32 current_opp,
					u8 target_vsel, u8 current_vsel)
{

	int sr_status = 0;
	u32 vdd, target_opp_no;
	u8 slave_addr = 0, opp_reg_addr = 0, volt_reg_addr = 0;

	vdd = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);

	if (vdd == VDD1_OPP) {
		sr_status = sr_stop_vddautocomap(SR1);
		slave_addr = MAPPHONE_R_SRI2C_SLAVE_ADDR_SA0;
		volt_reg_addr = MAPPHONE_R_VDD1_SR_CONTROL;
		opp_reg_addr = R_SMPS_VOL_OPP2_RA0;

	} else if (vdd == VDD2_OPP) {
		sr_status = sr_stop_vddautocomap(SR2);
		slave_addr = MAPPHONE_R_SRI2C_SLAVE_ADDR_SA1;
		volt_reg_addr = MAPPHONE_R_VDD2_SR_CONTROL;
		opp_reg_addr = R_SMPS_VOL_OPP2_RA1;
	}

	/* Update the CPCAP SWx OPP2 register, stores the on voltage value */
	omap3_bypass_cmd(slave_addr, opp_reg_addr, target_vsel);

	/* Update the CPCAP SWx voltage register, change the output voltage */
	omap3_bypass_cmd(slave_addr, volt_reg_addr, target_vsel);

	if (target_vsel > current_vsel)
		udelay(target_vsel - current_vsel + 4);

	if (sr_status) {
		if (vdd == VDD1_OPP)
			sr_start_vddautocomap(SR1, target_opp_no);
		else if (vdd == VDD2_OPP)
			sr_start_vddautocomap(SR2, target_opp_no);
	}

	return SR_PASS;
}
#endif

/* Mapphone specific PM */

extern void omap_uart_block_sleep(int num);
static struct wake_lock baseband_wakeup_wakelock;
static int mapphone_bpwake_irqhandler(int irq, void *unused)
{
	omap_uart_block_sleep(0);
	/*
	 * uart_block_sleep keeps uart clock active for 500 ms,
	 * prevent suspend for 1 sec to be safe
	 */
	wake_lock_timeout(&baseband_wakeup_wakelock, HZ);
	return IRQ_HANDLED;
}

static int mapphone_bpwake_probe(struct platform_device *pdev)
{
	int rc;

	gpio_request(MAPPHONE_APWAKE_TRIGGER_GPIO, "BP -> AP IPC trigger");
	gpio_direction_input(MAPPHONE_APWAKE_TRIGGER_GPIO);

	wake_lock_init(&baseband_wakeup_wakelock, WAKE_LOCK_SUSPEND, "bpwake");

	rc = request_irq(gpio_to_irq(MAPPHONE_APWAKE_TRIGGER_GPIO),
			 mapphone_bpwake_irqhandler,
			 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			 "Remote Wakeup", NULL);
	if (rc) {
		wake_lock_destroy(&baseband_wakeup_wakelock);
		printk(KERN_ERR
		       "Failed requesting APWAKE_TRIGGER irq (%d)\n", rc);
		return rc;
	}

	enable_irq_wake(gpio_to_irq(MAPPHONE_APWAKE_TRIGGER_GPIO));
	return 0;
}

static int mapphone_bpwake_remove(struct platform_device *pdev)
{
	wake_lock_destroy(&baseband_wakeup_wakelock);
	free_irq(gpio_to_irq(MAPPHONE_APWAKE_TRIGGER_GPIO), NULL);
	return 0;
}

static int mapphone_bpwake_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	return 0;
}

static int mapphone_bpwake_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mapphone_bpwake_driver = {
	.probe		= mapphone_bpwake_probe,
	.remove		= mapphone_bpwake_remove,
	.suspend	= mapphone_bpwake_suspend,
	.resume		= mapphone_bpwake_resume,
	.driver		= {
		.name		= "mapphone_bpwake",
		.owner		= THIS_MODULE,
	},
};

static struct platform_device mapphone_bpwake_device = {
	.name		= "mapphone_bpwake",
	.id		= -1,
	.num_resources	= 0,
};

/* Choose cold or warm reset
 *    RST_TIME1>4ms will trigger CPCAP to trigger a system cold reset */
static void mapphone_pm_set_reset(char cold)
{
	if (cold) {
		/* Configure RST_TIME1 to 6ms  */
		prm_rmw_mod_reg_bits(OMAP_RSTTIME1_MASK,
		0xc8<<OMAP_RSTTIME1_SHIFT,
		OMAP3430_GR_MOD,
		OMAP3_PRM_RSTTIME_OFFSET);
	} else {
		/* Configure RST_TIME1 to 30us  */
		prm_rmw_mod_reg_bits(OMAP_RSTTIME1_MASK,
		0x01<<OMAP_RSTTIME1_SHIFT,
		OMAP3430_GR_MOD,
		OMAP3_PRM_RSTTIME_OFFSET);
	}
}

static int mapphone_pm_reboot_call(struct notifier_block *this,
			unsigned long code, void *cmd)
{
	int result = NOTIFY_DONE;

	if (code == SYS_RESTART) {
		/* set cold reset */
		mapphone_pm_set_reset(1);
	}

	return result;
}

static struct notifier_block mapphone_pm_reboot_notifier = {
	.notifier_call = mapphone_pm_reboot_call,
};

#ifdef CONFIG_MEM_DUMP

#define WARMRESET 1
#define COLDRESET 0

static unsigned long reset_status = COLDRESET ;
#endif
static void mapphone_pm_init(void)
{
	omap3_set_prm_setup_vc(&mapphone_prm_setup);
	omap3_voltagescale_vcbypass_setup(mapphone_voltagescale_vcbypass);

    /* Set CPCAP SW1/SW2 I2C CNTL Reg to 0x45(PSM/PSM mode, VPLL enabled) to
	 * avoid extra current drain in active case before hit RET once*/

	/* Initialize CPCAP SW1&SW2 OPP1&OPP2 registers */
	/* SW1, OPP1 for RET Voltage --- 1.0V,
	 * OPP2 for ON Voltge --- 1.225V(OPP3)
	 */
	omap3_bypass_cmd(MAPPHONE_R_SRI2C_SLAVE_ADDR_SA0,
				MAPPHONE_R_SMPS_VOL_CNTL_CMDRA0, 0x45);
	omap3_bypass_cmd(MAPPHONE_R_SRI2C_SLAVE_ADDR_SA0,
				R_SMPS_VOL_OPP1_RA0, 0x20);
	omap3_bypass_cmd(MAPPHONE_R_SRI2C_SLAVE_ADDR_SA0,
				R_SMPS_VOL_OPP2_RA0, 0x32);

	/* SW2, OPP1 for RET Voltage --- 1.0V,
	 * OPP2 for ON Voltge --- 1.175V(OPP3)
	 */
	omap3_bypass_cmd(MAPPHONE_R_SRI2C_SLAVE_ADDR_SA1,
				MAPPHONE_R_SMPS_VOL_CNTL_CMDRA1, 0x45);
	omap3_bypass_cmd(MAPPHONE_R_SRI2C_SLAVE_ADDR_SA1,
				R_SMPS_VOL_OPP1_RA1, 0x20);
	omap3_bypass_cmd(MAPPHONE_R_SRI2C_SLAVE_ADDR_SA1,
				R_SMPS_VOL_OPP2_RA1, 0x2E);

	/* Configure BP <-> AP wake pins */
	omap_cfg_reg(AA21_34XX_GPIO157_OUT);
	omap_cfg_reg(AE6_34XX_GPIO141_DOWN);

	platform_device_register(&mapphone_bpwake_device);
	platform_driver_register(&mapphone_bpwake_driver);

#ifdef CONFIG_MEM_DUMP
	if (reset_status == COLDRESET)
		mapphone_pm_set_reset(1);
	else
		mapphone_pm_set_reset(0);
#else
	/* set cold reset, will move to warm reset once ready */
	mapphone_pm_set_reset(1);
#endif
	register_reboot_notifier(&mapphone_pm_reboot_notifier);
}

#ifdef CONFIG_MEM_DUMP
static struct proc_dir_entry *proc_entry ;

ssize_t reset_proc_read(char *page, char **start, off_t off, \
   int count, int *eof, void *data)
{
	int len ;
    /* don't visit offset */
	if (off > 0) {
		*eof = 1 ;
		return 0 ;
	}
	len = snprintf(page, sizeof(page), "%x\n", (unsigned int)reset_status) ;
	return len ;
}

ssize_t reset_proc_write(struct file *filp, const char __user *buff, \
  unsigned long len, void *data)
{
#define MAX_UL_LEN 8
	char k_buf[MAX_UL_LEN] ;
	int count = min((unsigned long)MAX_UL_LEN, len) ;
	int ret ;

	if (copy_from_user(k_buf, buff, count)) {
		ret = -EFAULT ;
		goto err ;
	} else{
		if (k_buf[0] == '0') {
			reset_status = COLDRESET;
			mapphone_pm_set_reset(1);
			printk(KERN_ERR"switch to cold reset\n");
		} else if (k_buf[0] == '1') {
			reset_status = WARMRESET;
			mapphone_pm_set_reset(0);
			printk(KERN_ERR"switch to warm reset\n");
		} else{
			ret = -EFAULT;
			goto err;
		}
	return count ;
	}
err:
	return ret ;
}

static void  reset_proc_init(void)
{
	proc_entry = create_proc_entry("reset_proc", 0666, NULL);
	if (proc_entry == NULL) {
		printk(KERN_INFO"Couldn't create proc entry\n") ;
	} else{
		proc_entry->read_proc = reset_proc_read ;
		proc_entry->write_proc = reset_proc_write ;
		proc_entry->owner = THIS_MODULE ;
	}
}

int __init warmreset_init(char *s)
{
	/* configure to warmreset */
	reset_status = WARMRESET;
	mapphone_pm_set_reset(0);
	return 1;
}
__setup("warmreset_debug=", warmreset_init);
#endif

static void __init config_wlan_gpio(void)
{
	/* WLAN PE and IRQ */
	omap_cfg_reg(AE22_34XX_GPIO186_OUT);
	omap_cfg_reg(J8_3430_GPIO65);
}

static void __init config_mmc2_init(void)
{
	u32 val;

	/* MMC2 */
	omap_cfg_reg(AE2_3430_MMC2_CLK);
	omap_cfg_reg(AG5_3430_MMC2_CMD);
	omap_cfg_reg(AH5_3430_MMC2_DAT0);
	omap_cfg_reg(AH4_3430_MMC2_DAT1);
	omap_cfg_reg(AG4_3430_MMC2_DAT2);
	omap_cfg_reg(AF4_3430_MMC2_DAT3);

	/* Set internal loopback clock */
	val = omap_ctrl_readl(OMAP343X_CONTROL_DEVCONF1);
	omap_ctrl_writel((val | OMAP2_MMCSDIO2ADPCLKISEL),
				OMAP343X_CONTROL_DEVCONF1);
}

/* must match value in drivers/w1/w1_family.h */
#define W1_EEPROM_DS2502        0x89
static struct omap2_hdq_platform_config mapphone_hdq_data = {
	.mode = OMAP_SDQ_MODE,
	.id = W1_EEPROM_DS2502,
};

static int __init omap_hdq_init(void)
{
	omap_cfg_reg(J25_34XX_HDQ_SIO);
	omap_hdq_device.dev.platform_data = &mapphone_hdq_data;
	return platform_device_register(&omap_hdq_device);
}

static int mapphone_wl1271_init(void)
{
	int rc = 0;

	/* wl1271 BT chip init sequence */
	gpio_direction_output(MAPPHONE_WL1271_NSHUTDOWN_GPIO, 0);
	msleep(5);
	gpio_set_value(MAPPHONE_WL1271_NSHUTDOWN_GPIO, 1);
	msleep(10);
	gpio_set_value(MAPPHONE_WL1271_NSHUTDOWN_GPIO, 0);
	msleep(5);

	/* Reserve BT wake and hostwake GPIOs */
	rc = gpio_request(MAPPHONE_WL1271_WAKE_GPIO, "wl127x_wake_gpio");
	if (unlikely(rc))
		return rc;

	rc = gpio_request(MAPPHONE_WL1271_HOSTWAKE_GPIO,
				"wl127x_hostwake_gpio");
	if (unlikely(rc))
		return rc;

	gpio_direction_output(MAPPHONE_WL1271_WAKE_GPIO, 1);
	gpio_direction_input(MAPPHONE_WL1271_HOSTWAKE_GPIO);

	return 0;
}

static int mapphone_wl1271_release(void)
{
	gpio_free(MAPPHONE_WL1271_WAKE_GPIO);
	gpio_free(MAPPHONE_WL1271_HOSTWAKE_GPIO);

	return 0;
}

static int mapphone_wl1271_enable(void)
{
	gpio_set_value(MAPPHONE_WL1271_WAKE_GPIO, 0);
	return 0;
}

static int mapphone_wl1271_disable(void)
{
	gpio_set_value(MAPPHONE_WL1271_WAKE_GPIO, 1);
	return 0;
}

static struct wl127x_rfkill_platform_data mapphone_wl1271_pdata = {
	.bt_nshutdown_gpio = MAPPHONE_WL1271_NSHUTDOWN_GPIO,
	.fm_enable_gpio = -1,
	.bt_hw_init = mapphone_wl1271_init,
	.bt_hw_release = mapphone_wl1271_release,
	.bt_hw_enable = mapphone_wl1271_enable,
	.bt_hw_disable = mapphone_wl1271_disable,
};

static struct platform_device mapphone_wl1271_device = {
	.name = "wl127x-rfkill",
	.id = 0,
	.dev.platform_data = &mapphone_wl1271_pdata,
};

static struct wl127x_test_platform_data mapphone_wl1271_test_pdata = {
	.btwake_gpio = MAPPHONE_WL1271_WAKE_GPIO,
	.hostwake_gpio = MAPPHONE_WL1271_HOSTWAKE_GPIO,
};

static struct platform_device mapphone_wl1271_test_device = {
	.name = "wl127x-test",
	.id = 0,
	.dev.platform_data = &mapphone_wl1271_test_pdata,
};

static void __init mapphone_bt_init(void)
{
	/* Mux setup for Bluetooth chip-enable */
	omap_cfg_reg(T3_34XX_GPIO_179);

	/* Mux setup for BT wake GPIO and hostwake GPIO */
	omap_cfg_reg(AF21_34XX_GPIO8);
	omap_cfg_reg(W7_34XX_GPIO178_DOWN);

	platform_device_register(&mapphone_wl1271_device);
	platform_device_register(&mapphone_wl1271_test_device);
}

static struct omap_mdm_ctrl_platform_data omap_mdm_ctrl_platform_data = {
	.bp_ready_ap_gpio = MAPPHONE_BP_READY_AP_GPIO,
	.bp_ready2_ap_gpio = MAPPHONE_BP_READY2_AP_GPIO,
	.bp_resout_gpio = MAPPHONE_BP_RESOUT_GPIO,
	.bp_pwron_gpio = MAPPHONE_BP_PWRON_GPIO,
	.ap_to_bp_pshold_gpio = MAPPHONE_AP_TO_BP_PSHOLD_GPIO,
	.ap_to_bp_flash_en_gpio = MAPPHONE_AP_TO_BP_FLASH_EN_GPIO,
};

static struct platform_device omap_mdm_ctrl_platform_device = {
	.name = OMAP_MDM_CTRL_MODULE_NAME,
	.id = -1,
	.dev = {
		.platform_data = &omap_mdm_ctrl_platform_data,
	},
};

static int __init mapphone_omap_mdm_ctrl_init(void)
{
	if (!is_cdma_phone())
		return -ENODEV;

	gpio_request(MAPPHONE_BP_READY2_AP_GPIO, "BP Flash Ready");
	gpio_direction_input(MAPPHONE_BP_READY2_AP_GPIO);
	omap_cfg_reg(T4_34XX_GPIO59_DOWN);

	gpio_request(MAPPHONE_BP_RESOUT_GPIO, "BP Reset Output");
	gpio_direction_input(MAPPHONE_BP_RESOUT_GPIO);
	omap_cfg_reg(AE3_34XX_GPIO139_DOWN);

	gpio_request(MAPPHONE_BP_PWRON_GPIO, "BP Power On");
	gpio_direction_output(MAPPHONE_BP_PWRON_GPIO, 0);
	omap_cfg_reg(AH3_34XX_GPIO137_OUT);

	gpio_request(MAPPHONE_AP_TO_BP_PSHOLD_GPIO, "AP to BP PS Hold");
	gpio_direction_output(MAPPHONE_AP_TO_BP_PSHOLD_GPIO, 0);
	omap_cfg_reg(AF3_34XX_GPIO138_OUT);

	return platform_device_register(&omap_mdm_ctrl_platform_device);
}

static struct omap_vout_config mapphone_vout_platform_data = {
	.max_width = 864,
	.max_height = 648,
	.max_buffer_size = 0x112000,
	.num_buffers = 8,
	.num_devices = 2,
	.device_ids = {1, 2},
};

static struct platform_device mapphone_vout_device = {
	.name = "omapvout",
	.id = -1,
	.dev = {
		.platform_data = &mapphone_vout_platform_data,
	},
};
static void __init mapphone_vout_init(void)
{
	platform_device_register(&mapphone_vout_device);
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define RAM_CONSOLE_START   0x8E000000
#define RAM_CONSOLE_SIZE    0x20000
static struct resource ram_console_resource = {
       .start  = RAM_CONSOLE_START,
       .end    = (RAM_CONSOLE_START + RAM_CONSOLE_SIZE - 1),
       .flags  = IORESOURCE_MEM,
};

static struct platform_device ram_console_device = {
       .name = "ram_console",
       .id = 0,
       .num_resources  = 1,
       .resource       = &ram_console_resource,
};

static inline void mapphone_ramconsole_init(void)
{
	platform_device_register(&ram_console_device);
}

static inline void omap2_ramconsole_reserve_sdram(void)
{
	reserve_bootmem(RAM_CONSOLE_START, RAM_CONSOLE_SIZE, 0);
}
#else
static inline void mapphone_ramconsole_init(void) {}

static inline void omap2_ramconsole_reserve_sdram(void) {}
#endif


static struct platform_device mapphone_sgx_device = {
       .name                   = "pvrsrvkm",
       .id             = -1,
};
static struct platform_device mapphone_omaplfb_device = {
	.name			= "omaplfb",
	.id			= -1,
};


static void __init mapphone_sgx_init(void)
{
	platform_device_register(&mapphone_sgx_device);
	platform_device_register(&mapphone_omaplfb_device);
}

static void __init mapphone_bp_model_init(void)
{
#ifdef CONFIG_OMAP_RESET_CLOCKS
	struct clk *clkp;
#endif

#ifdef CONFIG_ARM_OF
	struct device_node *bp_node;
	const void *bp_prop;

	if ((bp_node = of_find_node_by_path(DT_PATH_CHOSEN))) {
		if ((bp_prop = of_get_property(bp_node, \
			DT_PROP_CHOSEN_BP, NULL)))
			bp_model = (char *)bp_prop;

		of_node_put(bp_node);
	}
#endif
#ifdef CONFIG_OMAP_RESET_CLOCKS
	/* Enable sad2d iclk */
	clkp = clk_get(NULL, "sad2d_ick");
	if (clkp)
		clk_enable(clkp);
#endif
}

static void mapphone_pm_power_off(void)
{
	printk(KERN_INFO "mapphone_pm_power_off start...\n");
	local_irq_disable();

	/* config gpio 176 back from safe mode to reset the device */
	omap_writew(0x4, 0x480021D2);
	gpio_direction_output(MAPPHONE_POWER_OFF_GPIO, 0);

	do {} while (1);

	local_irq_enable();
}

static void mapphone_pm_reset(void)
{
	arch_reset('h');
}

static int cpcap_charger_connected_probe(struct platform_device *pdev)
{
	pm_power_off = mapphone_pm_reset;
	return 0;
}

static int cpcap_charger_connected_remove(struct platform_device *pdev)
{
	pm_power_off = mapphone_pm_power_off;
	return 0;
}

static struct platform_driver cpcap_charger_connected_driver = {
	.probe		= cpcap_charger_connected_probe,
	.remove		= cpcap_charger_connected_remove,
	.driver		= {
		.name	= "cpcap_charger_connected",
		.owner	= THIS_MODULE,
	},
};

static void __init mapphone_power_off_init(void)
{
	gpio_request(MAPPHONE_POWER_OFF_GPIO, "mapphone power off");
	gpio_direction_output(MAPPHONE_POWER_OFF_GPIO, 1);
	omap_cfg_reg(AB1_34XX_GPIO176_OUT);

	/* config gpio176 into safe mode with the pull up enabled to avoid
	 * glitch at reboot */
	omap_writew(0x1F, 0x480021D2);
	pm_power_off = mapphone_pm_power_off;

	platform_driver_register(&cpcap_charger_connected_driver);
}

static void __init mapphone_init(void)
{
	int ret = 0;
	struct kobject *properties_kobj = NULL;

	omap_board_config = mapphone_config;
	omap_board_config_size = ARRAY_SIZE(mapphone_config);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
				 &mapphone_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");

	mapphone_bp_model_init();
	mapphone_padconf_init();
	mapphone_gpio_mapping_init();
	mapphone_ramconsole_init();
	mapphone_omap_mdm_ctrl_init();
	mapphone_spi_init();
	mapphone_flash_init();
	mapphone_serial_init();
	mapphone_als_init();
	mapphone_panel_init();
	mapphone_sensors_init();
	mapphone_camera_init();
	mapphone_touch_init();
	mapphone_audio_init();
	usb_musb_init();
	mapphone_ehci_init();
	mapphone_sdrc_init();
	mapphone_pm_init();
	config_mmc2_init();
	config_wlan_gpio();
	omap_hdq_init();
	mapphone_bt_init();
#if mapphone_MMCPROBE_ENABLED
	mapphone_mmcprobe_init();
#else
	mapphone_hsmmc_init();
#endif
	mapphone_vout_init();
	mapphone_sgx_init();
	mapphone_power_off_init();
	mapphone_gadget_init();
#ifdef CONFIG_MEM_DUMP
    reset_proc_init();
#endif
}

static void __init mapphone_map_io(void)
{
	omap2_ramconsole_reserve_sdram();
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(MAPPHONE, "mapphone_")
	/* Maintainer: Motorola, Inc. */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80C00100,
	.map_io		= mapphone_map_io,
	.init_irq	= mapphone_init_irq,
	.init_machine	= mapphone_init,
	.timer		= &omap_timer,
MACHINE_END
