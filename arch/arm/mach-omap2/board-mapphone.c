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
#include <linux/qtouch_obp_ts.h>
#include <linux/led-cpcap-lm3554.h>
#include <linux/led-lm3530.h>
#include <linux/usb/omap.h>
#include <linux/wl127x-rfkill.h>
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
#include <linux/usb/android.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#include "pm.h"
#include "prm-regbits-34xx.h"
#include "smartreflex.h"
#include "omap3-opp.h"

#ifdef CONFIG_VIDEO_OLDOMAP3
#include <media/v4l2-int-device.h>
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#include <media/mt9p012.h>

#endif
#if defined(CONFIG_VIDEO_MT9P012_HP)
#include <../drivers/media/video/mt9p012_hp.h>
#endif
#if defined(CONFIG_VIDEO_MT9P013_HP)
#include <../drivers/media/video/mt9p013_hp.h>
#endif
#ifdef CONFIG_VIDEO_OMAP3_HPLENS
#include <../drivers/media/video/hplens.h>
#endif
#endif

#define MAPPHONE_IPC_USB_SUSP_GPIO	142
#define MAPPHONE_AP_TO_BP_FLASH_EN_GPIO	157
#define MAPPHONE_TOUCH_RESET_N_GPIO	164
#define MAPPHONE_TOUCH_INT_GPIO		109
#define MAPPHONE_LM_3530_INT_GPIO	41
#define MAPPHONE_AKM8973_INT_GPIO	175
#define MAPPHONE_WL1271_NSHUTDOWN_GPIO	179
#define MAPPHONE_AUDIO_PATH_GPIO	143
#define MAPPHONE_BP_READY_AP_GPIO	141
#define MAPPHONE_BP_READY2_AP_GPIO	59
#define MAPPHONE_BP_RESOUT_GPIO		139
#define MAPPHONE_BP_PWRON_GPIO		137
#define MAPPHONE_AP_TO_BP_PSHOLD_GPIO	138
#define MAPPHONE_AP_TO_BP_FLASH_EN_GPIO	157
#define MAPPHONE_POWER_OFF_GPIO		176
#define SHOLEST_HDMI_MUX_SELECT_GPIO    7
#define SHOLEST_HDMI_MUX_EN_N_GPIO  69
#define SHOLEST_LM_3530_EN_GPIO     27
#define DIE_ID_REG_BASE			(L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET		0x218
#define MAX_USB_SERIAL_NUM		17

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

static struct omap_opp mapphone_l3_rate_table[] = {
	{0, 0, 0},
	/*OPP1*/
	{0, VDD2_OPP1, 0x20},
	/*OPP2*/
	{S83M, VDD2_OPP2, 0x27},
	/*OPP3*/
	{S166M, VDD2_OPP3, 0x2E},
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
	omap2_init_common_hw(NULL, mapphone_mpu_rate_table,
			mapphone_dsp_rate_table, mapphone_l3_rate_table);
	omap_init_irq();
#ifdef CONFIG_OMAP3_PM
	scm_clk_init();
#endif
	omap_gpio_init();
}

static struct android_usb_platform_data andusb_plat = {
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

static void mapphone_gadget_init(void)
{
	unsigned int val[2];
	unsigned int reg;

	reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;
	val[0] = omap_readl(reg);
	val[1] = omap_readl(reg + 4);

	snprintf(device_serial, MAX_USB_SERIAL_NUM, "%08x%08x", val[1], val[0]);
	platform_device_register(&androidusb_device);
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

static struct qtouch_ts_platform_data mapphone_ts_platform_data;

static void mapphone_touch_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *touch_node;
	const void *touch_prop;
	int len = 0;
	const uint32_t *touch_propt;

	if ((touch_node = of_find_node_by_path(DT_PATH_TOUCH))) {
		if ((touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_KEYMAP, &len)) \
			&& len && (0 == len % sizeof(struct vkey))) {
			mapphone_ts_platform_data.vkeys.count = len / sizeof(struct vkey);
			mapphone_ts_platform_data.vkeys.keys = (struct vkey *)touch_prop;
		}
		touch_propt = of_get_property(touch_node, \
			DT_PROP_TOUCH_REVERSE_X, \
			&len);
		if (touch_propt && len)
			mapphone_ts_platform_data.reverse_x = *touch_propt;

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

static void mapphone_misc_init(void)
{
    printk(KERN_INFO "%s:Initializing\n", __func__);
    if (gpio_request(SHOLEST_HDMI_MUX_SELECT_GPIO, "HDMI-mux-select") >= 0)
    {
        msleep(1);
        gpio_direction_output(SHOLEST_HDMI_MUX_SELECT_GPIO, 0);
        gpio_set_value(SHOLEST_HDMI_MUX_SELECT_GPIO, 0);
        msleep(5);
    }
    if (gpio_request(SHOLEST_HDMI_MUX_EN_N_GPIO, "HDMI-mux-enable-n") >= 0)
    {
        msleep(1);
        gpio_direction_output(SHOLEST_HDMI_MUX_EN_N_GPIO, 0);
        gpio_set_value(SHOLEST_HDMI_MUX_EN_N_GPIO, 0);
        msleep(5);
    }
    if (gpio_request(SHOLEST_LM_3530_EN_GPIO, "led-enable") >= 0)
    {
        msleep(1);
        gpio_direction_output(SHOLEST_LM_3530_EN_GPIO, 0);
        gpio_set_value(SHOLEST_LM_3530_EN_GPIO, 1);
        msleep(5);
    }
}

static struct vkey mapphone_touch_vkeys[] = {
	{
		.min		= 0,
		.max		= 139,
		.code		= KEY_MENU,
	},
	{
		.min		= 422,
		.max		= 602,
		.code		= KEY_HOME,
	},
	{
		.min		= 884,
		.max		= 1023,
		.code		= KEY_BACK,
	},
//##SSA
#if 0 //for p0, p1, p2
	{
		.min		= 880,
		.max		= 1024,
		.code		= KEY_SEARCH,
	},
#endif
};

static struct qtouch_ts_platform_data mapphone_ts_platform_data = {
	.irqflags	= IRQF_TRIGGER_LOW,
	.flags		= (QTOUCH_SWAP_XY |
			   QTOUCH_USE_MULTITOUCH |
			   QTOUCH_CFG_BACKUPNV),
	.reverse_x      = 0,
	.abs_min_x	= 0,
	.abs_max_x	= 1024,
	.abs_min_y	= 0,
	.abs_max_y	= 960,
	.abs_min_p	= 0,
	.abs_max_p	= 255,
	.abs_min_w	= 0,
	.abs_max_w	= 15,
	.nv_checksum	= 0xf429,
	.fuzz_x		= 0,
	.fuzz_y		= 0,
	.fuzz_p		= 2,
	.fuzz_w		= 2,
	.hw_reset	= mapphone_touch_reset,
	.power_cfg	= {
		.idle_acq_int	= 1,
		.active_acq_int	= 16,
		.active_idle_to	= 25,
	},
	.acquire_cfg	= {
		.charge_time	= 10,
		.atouch_drift	= 5,
		.touch_drift	= 20,
		.drift_susp	= 20,
		.touch_autocal	= 0,
		.sync		= 0,
	},
	.multi_touch_cfg	= {
		.ctrl		= 0x03,
		.x_origin	= 0,
		.y_origin	= 0,
		.x_size		= 18,
		.y_size		= 10,
		.aks_cfg	= 0,
		.burst_len	= 0x40,
		.tch_det_thr	= 0x23,
		.tch_det_int	= 0x4,
		.mov_hyst_init	= 5,
		.mov_hyst_next	= 5,
		.mov_filter	= 0,
		.num_touch	= 4,
		.merge_hyst	= 5,
		.merge_thresh	= 5,
	},
	.linear_tbl_cfg = {
		.ctrl = 0x00,
		.x_offset = 0x0000,
		.x_segment = {0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00},
		.y_offset = 0x0000,
		.y_segment = {0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00},
	},
	.grip_suppression_cfg = {
		.ctrl		= 0x00,
		.xlogrip	= 0x00,
		.xhigrip	= 0x00,
		.ylogrip	= 0x00,
		.yhigrip	= 0x00,
		.maxtchs	= 0x00,
		.szthr1	= 0x00,
		.szthr2	= 0x00,
		.shpthr1	= 0x00,
		.shpthr2	= 0x00,
	},
	.vkeys			= {
		.keys		= mapphone_touch_vkeys,
		.count		= ARRAY_SIZE(mapphone_touch_vkeys),
		.start		= 961,
	},
};

static struct lm3530_platform_data omap3430_als_light_data = {
	.gen_config = 0x33,
	.als_config = 0x7D,
	.brightness_ramp = 0x36,
	.als_zone_info = 0x00,
	.als_resistor_sel = 0x11, /* 13.531kOhm */
	.brightness_control = 0x00,
	.zone_boundary_0 = 0x33,
	.zone_boundary_1 = 0x66,
	.zone_boundary_2 = 0x99,
	.zone_boundary_3 = 0xCC,
	.zone_target_0 = 0x19,
	.zone_target_1 = 0x33,
	.zone_target_2 = 0x4c,
	.zone_target_3 = 0x66,
	.zone_target_4 = 0x7f,
	.manual_current = 0x33,
	.upper_curr_sel = 5,
	.lower_curr_sel = 2,
};

static struct lm3554_platform_data mapphone_camera_flash = {
	.torch_brightness_def = 0xa0,
	.flash_brightness_def = 0x78,
	.flash_duration_def = 0x48,
	.config_reg_1_def = 0xe0,
	.config_reg_2_def = 0xf0,
	.vin_monitor_def = 0x07,
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

extern struct akm8973_platform_data mapphone_akm8973_data;
extern struct lis331dlh_platform_data mapphone_lis331dlh_data;
#ifdef CONFIG_MOT_FEAT_LP3907
extern struct lp3907_platform_data mapphone_lp3907_data;
#endif
static struct i2c_board_info __initdata mapphone_i2c_bus2_board_info[] = {
	{
		I2C_BOARD_INFO("akm8973", 0x1C),
		.platform_data = &mapphone_akm8973_data,
		.irq = OMAP_GPIO_IRQ(MAPPHONE_AKM8973_INT_GPIO),
	},
	{
		I2C_BOARD_INFO("lis331dlh", 0x19),
		.platform_data = &mapphone_lis331dlh_data,
	},
#ifdef CONFIG_MOT_FEAT_LP3907
	{
		I2C_BOARD_INFO("lp3907", 0x61),
		.platform_data = &mapphone_lp3907_data,
	},
#endif
};

static struct i2c_board_info __initdata mapphone_i2c_bus3_board_info[] = {
#if defined(CONFIG_VIDEO_MT9P012_HP)
	{
#if defined(CONFIG_VIDEO_MT9P012_MT9P013_AUTODETECT)
		I2C_BOARD_INFO("mt9p012", 0x10),
#else
		I2C_BOARD_INFO("mt9p012", MT9P012_I2C_ADDR),
#endif
		.platform_data = &mapphone_mt9p012_platform_data,
	},
#endif
#if defined(CONFIG_VIDEO_MT9P013_HP)
	{
#if defined(CONFIG_VIDEO_MT9P012_MT9P013_AUTODETECT)
		I2C_BOARD_INFO("mt9p013", 0x1F),
#else
		I2C_BOARD_INFO("mt9p013", MT9P013_I2C_ADDR),
#endif
		.platform_data = &mapphone_mt9p013_platform_data,
	},
#endif
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
	{
		I2C_BOARD_INFO("tda19989", 0x70),
	},
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
#ifdef CONFIG_MOT_FEAT_MDTV
extern void __init mapphone_mdtv_init(void);
#endif

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

static struct omap_usb_platform_data usb_platform_data = {
	.port_data = usb_port_data,
	.num_ports = ARRAY_SIZE(usb_port_data),
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
	omap_serial_init();
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
	.clksetup = 0x52,
	.voltsetup_time1 = 0x229,
	.voltsetup_time2 = 0x229,
	.voltoffset = 0x0,
	.voltsetup2 = 0x0,
	.vdd0_on = 0x65,
	.vdd0_onlp = 0x45,
	.vdd0_ret = 0x17,
	.vdd0_off = 0x00,
	.vdd1_on = 0x65,
	.vdd1_onlp = 0x45,
	.vdd1_ret = 0x17,
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

#define CPCAP_SMPS_UPDATE_DELAY     170 /* In uSec */

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

	udelay(CPCAP_SMPS_UPDATE_DELAY);

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
static void mapphone_pm_init(void)
{
	omap3_set_prm_setup_vc(&mapphone_prm_setup);
	omap3_voltagescale_vcbypass_setup(mapphone_voltagescale_vcbypass);

	/* Initialize CPCAP SW1&SW2 OPP1&OPP2 registers */
	/* SW1, OPP1 for RET Voltage --- 1.0V,
	 * OPP2 for ON Voltge --- 1.225V(OPP3)
	 */
	omap3_bypass_cmd(MAPPHONE_R_SRI2C_SLAVE_ADDR_SA0,
				R_SMPS_VOL_OPP1_RA0, 0x20);
	omap3_bypass_cmd(MAPPHONE_R_SRI2C_SLAVE_ADDR_SA0,
				R_SMPS_VOL_OPP2_RA0, 0x32);

	/* SW2, OPP1 for RET Voltage --- 1.0V,
	 * OPP2 for ON Voltge --- 1.175V(OPP3)
	 */
	omap3_bypass_cmd(MAPPHONE_R_SRI2C_SLAVE_ADDR_SA1,
				R_SMPS_VOL_OPP1_RA1, 0x20);
	omap3_bypass_cmd(MAPPHONE_R_SRI2C_SLAVE_ADDR_SA1,
				R_SMPS_VOL_OPP2_RA1, 0x2E);
}

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

static struct wl127x_rfkill_platform_data mapphone_wl1271_pdata = {
	.bt_nshutdown_gpio = MAPPHONE_WL1271_NSHUTDOWN_GPIO,
	.fm_enable_gpio = -1,
};

static struct platform_device mapphone_wl1271_device = {
	.name = "wl127x-rfkill",
	.id = 0,
	.dev.platform_data = &mapphone_wl1271_pdata,
};

static void __init mapphone_bt_init(void)
{
	/* Mux setup for Bluetooth chip-enable */
	omap_cfg_reg(T3_34XX_GPIO_179);

	platform_device_register(&mapphone_wl1271_device);
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

	gpio_request(MAPPHONE_BP_READY_AP_GPIO, "BP Normal Ready");
	gpio_direction_input(MAPPHONE_BP_READY_AP_GPIO);
	omap_cfg_reg(AE6_34XX_GPIO141_DOWN);

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

	gpio_request(MAPPHONE_AP_TO_BP_FLASH_EN_GPIO, "AP to BP Flash Enable");
	gpio_direction_output(MAPPHONE_AP_TO_BP_FLASH_EN_GPIO, 0);
	omap_cfg_reg(AA21_34XX_GPIO157_OUT);

	return platform_device_register(&omap_mdm_ctrl_platform_device);
}

#ifdef CONFIG_FB_OMAP2
static struct resource mapphone_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};
#else
static struct resource mapphone_vout_resource[2] = {
};
#endif

static struct platform_device mapphone_vout_device = {
       .name                   = "omap_vout",
       .num_resources  = ARRAY_SIZE(mapphone_vout_resource),
       .resource               = &mapphone_vout_resource[0],
       .id             = -1,
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

	gpio_direction_output(MAPPHONE_POWER_OFF_GPIO, 0);

	do {} while (1);

	local_irq_enable();
}

static void __init mapphone_power_off_init(void)
{
	gpio_request(MAPPHONE_POWER_OFF_GPIO, "mapphone power off");
	gpio_direction_output(MAPPHONE_POWER_OFF_GPIO, 1);
	omap_cfg_reg(AB1_34XX_GPIO176_OUT);

	pm_power_off = mapphone_pm_power_off;
}

static void __init mapphone_init(void)
{
	omap_board_config = mapphone_config;
	omap_board_config_size = ARRAY_SIZE(mapphone_config);
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
	mapphone_misc_init();
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
	mapphone_hsmmc_init();
	mapphone_vout_init();
	mapphone_power_off_init();
	mapphone_gadget_init();
#ifdef CONFIG_MOT_FEAT_MDTV
	mapphone_mdtv_init();
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
