#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/pagemap.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <mach/display.h>
#include <mach/dma.h>

#include "panel-sholes.h"

#define DEBUG

#ifdef DEBUG
#define DBG(format, ...) \
		(printk(KERN_DEBUG "sholes-panel: " format, ## __VA_ARGS__))
#else
#define DBG(format, ...)
#endif

#define EDISCO_CMD_SOFT_RESET		0x01
#define EDISCO_CMD_ENTER_SLEEP_MODE	0x10
#define EDISCO_CMD_EXIT_SLEEP_MODE	0x11
#define EDISCO_CMD_SET_DISPLAY_ON	0x29
#define EDISCO_CMD_SET_DISPLAY_OFF	0x28
#define EDISCO_CMD_SET_COLUMN_ADDRESS	0x2A
#define EDISCO_CMD_SET_PAGE_ADDRESS	0x2B
#define EDISCO_CMD_SET_TEAR_ON		0x35
#define EDISCO_CMD_SET_TEAR_SCANLINE	0x44

#define EDISCO_CMD_VC   0
#define EDISCO_VIDEO_VC 1

#define EDISCO_LONG_WRITE	0x29
#define EDISCO_SHORT_WRITE_1	0x23
#define EDISCO_SHORT_WRITE_0	0x13

static struct omap_video_timings sholes_panel_timings = {
	.x_res          = 480,
	.y_res          = 854,
	/*.pixel_clock  = 25000,*/
	.hfp            = 0,
	.hsw            = 2,
	.hbp            = 2,
	.vfp            = 0,
	.vsw            = 1,
	.vbp            = 1,
};

struct sholes_data {
	struct work_struct work;
	struct omap_dss_device *dssdev;
};

#define DEVICE_NAME  "lcd-sholes"

struct sholes_panel_device {
	struct mutex  mtx; /* Lock for all device accesses */

	int major;
	struct class *cls;
	struct device *dev;

	int opened;

	int fod_en;	/* Freeze-On-Display state */
	int panel_en;	/* Panel hardware state */
	int dss_en;	/* Last DSS state request */
};

static struct sholes_panel_device *gDev;
static struct omap_dss_device *gDssdev;

/*=== DSS Interface Functions =======================================*/

static void sholes_panel_display_on(struct work_struct *work)
{
	struct sholes_data *sholes_data = container_of(work, struct sholes_data,
						       work);
	u8 data;

	sholes_data->dssdev->sync(sholes_data->dssdev);
	data = EDISCO_CMD_SET_DISPLAY_ON;
	dsi_bus_lock();
	dsi_vc_dcs_write(EDISCO_CMD_VC, &data, 1);
	dsi_bus_unlock();
}

static int sholes_panel_dss_probe(struct omap_dss_device *dssdev)
{
	struct sholes_data *data;

	DBG("sholes_panel_dss_probe\n");

	dssdev->ctrl.pixel_size = 24;
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = sholes_panel_timings;
	data = kmalloc(sizeof(struct sholes_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	INIT_WORK(&data->work, sholes_panel_display_on);
	data->dssdev = dssdev;
	dssdev->data = data;
	return 0;
}

static void sholes_panel_dss_remove(struct omap_dss_device *dssdev)
{
	kfree(dssdev->data);
	return;
}

static int sholes_panel_dss_enable(struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;

	DBG("sholes_panel_dss_enable\n");

	if (dssdev->platform_enable) {
		ret = dssdev->platform_enable(dssdev);
		if (ret)
			return ret;
	}

	mutex_lock(&gDev->mtx);
	gDev->dss_en = 1;
	gDev->panel_en = 1;
	mutex_unlock(&gDev->mtx);

	/* turn of mcs register acces protection */
	data[0] = 0xb2;
	data[1] = 0x00;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1, data, 2);

	/* enable lane setting and test registers*/
	data[0] = 0xef;
	data[1] = 0x01;
	data[2] = 0x01;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 3);

	/* 2nd param 61 = 1 line; 63 = 2 lanes */
	data[0] = 0xef;
	data[1] = 0x60;
	data[2] = 0x63;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 3);

	/* 2nd param 0 = WVGA; 1 = WQVGA */
	data[0] = 0xb3;
	data[1] = 0x00;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);

	/* Set dynamic backlight control and PWM; D[7:4] = PWM_DIV[3:0];*/
	/* D[3]=0 (PWM OFF);
	 * D[2]=0 (auto BL control OFF);
	 * D[1]=0 (Grama correction On);
	 * D[0]=0 (Enhanced Image Correction OFF) */
	data[0] = 0xb4;
	data[1] = 0x1f;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);

	/* set page, column address */
	data[0] = EDISCO_CMD_SET_PAGE_ADDRESS;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = (dssdev->panel.timings.y_res - 1) >> 8;
	data[4] = (dssdev->panel.timings.y_res - 1) & 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		goto error;

	data[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = (dssdev->panel.timings.x_res - 1) >> 8;
	data[4] = (dssdev->panel.timings.x_res - 1) & 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		goto error;

	/* turn it on */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	//ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_0, data, 1);
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);

	mdelay(200);

	schedule_work(&((struct sholes_data *)dssdev->data)->work);

	printk("done EDISCO CTRL ENABLE\n");
	return 0;
error:
	return -EINVAL;
}

static void sholes_panel_disable(struct omap_dss_device *dssdev)
{
	u8 data[1];

	data[0] = EDISCO_CMD_SET_DISPLAY_OFF;
	dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);

	data[0] = EDISCO_CMD_ENTER_SLEEP_MODE;
	dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);
	msleep(120);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void sholes_panel_dss_disable(struct omap_dss_device *dssdev)
{
	DBG("sholes_panel_dss_disable\n");

	mutex_lock(&gDev->mtx);

	gDev->dss_en = 0;

	if (gDev->fod_en) {
		DBG("Freezing the last frame on the display\n");
		mutex_unlock(&gDev->mtx);
		return;
	}

	gDev->panel_en = 0;

	mutex_unlock(&gDev->mtx);

	sholes_panel_disable(dssdev);
}

static void sholes_panel_dss_setup_update(struct omap_dss_device *dssdev,
				      u16 x, u16 y, u16 w, u16 h)
{

	u8 data[5];
	int ret;

	/* set page, column address */
	data[0] = EDISCO_CMD_SET_PAGE_ADDRESS;
	data[1] = y >> 8;
	data[2] = y & 0xff;
	data[3] = (y + h - 1) >> 8;
	data[4] = (y + h - 1) & 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		return;

	data[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	data[1] = x >> 8;
	data[2] = x & 0xff;
	data[3] = (x + w - 1) >> 8;
	data[4] = (x + w - 1) & 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		return;
}

static int sholes_panel_dss_enable_te(struct omap_dss_device *dssdev,
								bool enable)
{
	u8 data[3];
	int ret;

	data[0] = EDISCO_CMD_SET_TEAR_ON;
	data[1] = 0x00;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
	if (ret)
		goto error;

	data[0] = EDISCO_CMD_SET_TEAR_SCANLINE;
	data[1] = 0x03;
	data[2] = 0x00;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 3);
	if (ret)
		goto error;

	DBG(" edisco_ctrl_enable_te \n");
	return 0;

error:
	return -EINVAL;
}

static int sholes_panel_dss_rotate(struct omap_dss_device *display, u8 rotate)
{
	return 0;
}

static int sholes_panel_dss_mirror(struct omap_dss_device *display,
								bool enable)
{
	return 0;
}

static int sholes_panel_dss_run_test(struct omap_dss_device *display,
								int test_num)
{
	return 0;
}

static int sholes_panel_dss_suspend(struct omap_dss_device *dssdev)
{
	sholes_panel_dss_disable(dssdev);
	return 0;
}

static int sholes_panel_dss_resume(struct omap_dss_device *dssdev)
{
	return sholes_panel_dss_enable(dssdev);
}

static bool sholes_panel_dss_te_support(struct omap_dss_device *dssdev)
{
	return true;
}

static struct omap_dss_driver sholes_panel_dss_driver = {
	.probe = sholes_panel_dss_probe,
	.remove = sholes_panel_dss_remove,

	.enable = sholes_panel_dss_enable,
	.disable = sholes_panel_dss_disable,
	.suspend = sholes_panel_dss_suspend,
	.resume = sholes_panel_dss_resume,
	.setup_update = sholes_panel_dss_setup_update,
	.enable_te = sholes_panel_dss_enable_te,
	.te_support = sholes_panel_dss_te_support,
	.set_rotate = sholes_panel_dss_rotate,
	.set_mirror = sholes_panel_dss_mirror,
	.run_test = sholes_panel_dss_run_test,

	.driver = {
		.name = "sholes-panel",
		.owner = THIS_MODULE,
	},
};

/*=== Driver Interface Functions =======================================*/

static int sholes_panel_set_fod(int *fod_en)
{
	int rc;
	int en;

	rc = copy_from_user(&en, fod_en, sizeof(int));
	if (rc != 0) {
		DBG("S_FOD copy from user failed\n");
		goto failed;
	}

	en = (en) ? 1 : 0;

	if (en != gDev->fod_en) {
		gDev->fod_en = en;
		if (!en && !gDev->dss_en && gDev->panel_en) {
			dsi_bus_lock();
			gDev->panel_en = 0;
			sholes_panel_disable(gDssdev);
			dsi_bus_unlock();
		}
	}

failed:
	return rc;
}

static int sholes_panel_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	DBG("sholes_panel_open\n");

	if (gDev == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&gDev->mtx);

	/* We only support single open */
	if (gDev->opened) {
		DBG("Device already opened\n");
		rc = -EBUSY;
		goto failed;
	}

	gDev->opened = 1;

failed:
	mutex_unlock(&gDev->mtx);
	return rc;
}

static int sholes_panel_release(struct inode *inode, struct file *file)
{
	int rc = 0;

	DBG("sholes_panel_release\n");

	if (gDev == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&gDev->mtx);

	gDev->opened = 0;

	mutex_unlock(&gDev->mtx);

	return rc;
}

static int sholes_panel_ioctl(struct inode *inode, struct file *file,
							u_int cmd, u_long arg)
{
	int rc = 0;

	if (unlikely(_IOC_TYPE(cmd) != SHOLES_IOCTL_MAGIC)) {
		printk(KERN_ERR "Bad command value (%d)\n", cmd);
		return -EINVAL;
	}

	mutex_lock(&gDev->mtx);

	switch (cmd) {
	case SHOLES_G_FOD:
		rc = put_user(gDev->fod_en, (int *) arg);
		break;
	case SHOLES_S_FOD:
		rc = sholes_panel_set_fod((int *) arg);
		break;
	default:
		DBG("Invalid ioctl (%x)\n", cmd);
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&gDev->mtx);

	return rc;
}

static const struct file_operations sholes_panel_fops = {
	.owner = THIS_MODULE,
	.open = sholes_panel_open,
	.release = sholes_panel_release,
	.ioctl = sholes_panel_ioctl,
};

static int __init sholes_panel_probe(struct platform_device *pdev)
{
	int rc = 0;

	DBG("sholes_panel_probe\n");

	gDev = kzalloc(sizeof(struct sholes_panel_device), GFP_KERNEL);
	if (gDev == NULL)
		return -ENOMEM;

	memset(gDev, 0, sizeof(gDev));

	mutex_init(&gDev->mtx);

	gDev->opened = 0;

	gDev->major = register_chrdev(0, DEVICE_NAME, &sholes_panel_fops);
	if (gDev->major < 0) {
		printk(KERN_ERR "failed chrdev register\n");
		rc = -ENODEV;
		goto failed_chrdev;
	}

	gDev->cls = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(gDev->cls)) {
		printk(KERN_DEBUG "failed class creation\n");
		rc = PTR_ERR(gDev->cls);
		goto failed_class;
	}

	gDev->dev = device_create(gDev->cls, gDev->dev, MKDEV(gDev->major, 0),
							NULL, DEVICE_NAME);

	return 0;

failed_class:
	unregister_chrdev(gDev->major, DEVICE_NAME);
failed_chrdev:
	kfree(gDev);
	gDev = NULL;
	return rc;
}

static int sholes_panel_remove(struct platform_device *pdev)
{
	struct sholes_panel_device *dsw = platform_get_drvdata(pdev);

	DBG("sholes_panel_remove\n");

	if (dsw) {
		class_destroy(dsw->cls);
		unregister_chrdev(dsw->major, DEVICE_NAME);
		kfree(dsw);
	}

	return 0;
}

static struct platform_device sholes_panel_dev = {
	.name = DEVICE_NAME,
	.id = -1,
};

static struct platform_driver sholes_panel_driver = {
	.remove         = sholes_panel_remove,
	.driver         = {
		.name   = DEVICE_NAME,
	},
};

/*=== Init/Exit Interface Functions =======================================*/

static int __init sholes_panel_init(void)
{
	int rc = 0;

	DBG("sholes_panel_init\n");

	rc = platform_device_register(&sholes_panel_dev);
	if (rc != 0) {
		printk(KERN_ERR "failed panel device register %d\n", rc);
		goto faildev;
	}

	rc = platform_driver_probe(&sholes_panel_driver, sholes_panel_probe);
	if (rc != 0) {
		printk(KERN_ERR "failed panel register/probe %d\n", rc);
		goto faildrv;
	}

	rc = omap_dss_register_driver(&sholes_panel_dss_driver);
	if (rc != 0) {
		printk(KERN_ERR "failed panel dss register %d\n", rc);
		goto faildss;
	}

	return 0;

faildss:
	platform_driver_unregister(&sholes_panel_driver);
faildrv:
	platform_device_unregister(&sholes_panel_dev);
faildev:
	return -ENODEV;
}

static void __exit sholes_panel_exit(void)
{
	DBG("sholes_panel_exit\n");

	omap_dss_unregister_driver(&sholes_panel_dss_driver);
	platform_driver_unregister(&sholes_panel_driver);
	platform_device_unregister(&sholes_panel_dev);
}

module_init(sholes_panel_init);
module_exit(sholes_panel_exit);

MODULE_AUTHOR("Rebecca Schultz Zavin <rebecca@android.com>");
MODULE_DESCRIPTION("Sholes Panel Driver");
MODULE_LICENSE("GPL");
