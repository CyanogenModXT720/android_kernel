#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <mach/display.h>
#include <mach/dma.h>
#include <mach/omap-pm.h>

#define DEBUG

#ifdef DEBUG
#define DBG(format, ...) (\
	printk(KERN_DEBUG "hdtv-panel: " format, ## __VA_ARGS__))
#else
#define DBG(format, ...)
#endif

/* 720X480P */
const struct omap_video_timings hdtv_480p_panel_timings = {
	.x_res			= 720,
	.y_res			= 480,
	.pixel_clock		= 27027,
	.hsw			= 62,
	.hfp			= 16,
	.hbp			= 60,
	.vsw			= 6,
	.vfp			= 9,
	.vbp			= 30,
};

/* 1280X720P */
const struct omap_video_timings hdtv_720p_panel_timings = {
	.x_res			= 1280,
	.y_res			= 720,
	.pixel_clock		= 74250,
	.hsw			= 40,
	.hfp			= 110,
	.hbp			= 220,
	.vsw			= 5,
	.vfp			= 5,
	.vbp			= 20,
};

static int hdtv_panel_probe(struct omap_dss_device *dssdev)
{
	DBG("%s IN\n", __func__);

	/* 720p is default */
	dssdev->panel.timings = hdtv_720p_panel_timings;

	DBG("%s OUT\n", __func__);
	return 0;
}

static void hdtv_panel_remove(struct omap_dss_device *dssdev)
{
	DBG("%s not supported\n", __func__);
	return;
}

static int hdtv_panel_enable(struct omap_dss_device *dssdev)
{
    int ret;

	DBG("%s IN\n", __func__);

	/* Set min throughput to 1280 x 720 x 2bpp x 60fps x 3 L3 accesses */
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 331776);

	if (dssdev->platform_enable)
		ret = dssdev->platform_enable(dssdev);
	else
		ret = 0;

	DBG("%s OUT\n", __func__);
	return ret;
}

static void hdtv_panel_disable(struct omap_dss_device *dssdev)
{
	DBG("%s IN\n", __func__);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* Remove throughput requirement */
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);

	DBG("%s OUT\n", __func__);
}

static void hdtv_panel_setup_update(struct omap_dss_device *dssdev,
				      u16 x, u16 y, u16 w, u16 h)
{
	DBG("%s not supported\n", __func__);
}

static int hdtv_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	DBG("%s not supported\n", __func__);
	return 0;
}

static int hdtv_panel_rotate(struct omap_dss_device *display, u8 rotate)
{
	DBG("%s not supported\n", __func__);
	return 0;
}

static int hdtv_panel_mirror(struct omap_dss_device *display, bool enable)
{
	DBG("%s not supported\n", __func__);
	return 0;
}

static int hdtv_panel_run_test(struct omap_dss_device *display, int test_num)
{
	DBG("%s not supported\n", __func__);
	return 0;
}

static int hdtv_panel_suspend(struct omap_dss_device *dssdev)
{
	DBG("%s IN\n", __func__);

	hdtv_panel_disable(dssdev);

	DBG("%s OUT\n", __func__);
	return 0;
}

static int hdtv_panel_resume(struct omap_dss_device *dssdev)
{
    int ret;
	DBG("%s IN\n", __func__);

	ret = hdtv_panel_enable(dssdev);

	DBG("%s OUT\n", __func__);
    return ret;
}

static struct omap_dss_driver hdtv_panel_driver = {
	.probe = hdtv_panel_probe,
	.remove = hdtv_panel_remove,

	.enable = hdtv_panel_enable,
	.disable = hdtv_panel_disable,
	.suspend = hdtv_panel_suspend,
	.resume = hdtv_panel_resume,
	.setup_update = hdtv_panel_setup_update,
	.enable_te = hdtv_panel_enable_te,
	.set_rotate = hdtv_panel_rotate,
	.set_mirror = hdtv_panel_mirror,
	.run_test = hdtv_panel_run_test,

	.driver = {
		.name = "hdtv-panel",
		.owner = THIS_MODULE,
	},
};


static int __init hdtv_panel_init(void)
{
	DBG("%s IN\n", __func__);
	omap_dss_register_driver(&hdtv_panel_driver);
	DBG("%s OUT\n", __func__);
	return 0;
}

static void __exit hdtv_panel_exit(void)
{
	DBG("%s IN\n", __func__);
	omap_dss_unregister_driver(&hdtv_panel_driver);
	DBG("%s OUT\n", __func__);
}

module_init(hdtv_panel_init);
module_exit(hdtv_panel_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("Hdtv Panel Driver");
MODULE_LICENSE("GPL");
