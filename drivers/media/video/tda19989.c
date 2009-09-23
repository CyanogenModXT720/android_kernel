
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kmod.h>
#include <linux/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#if defined(CONFIG_MOT_FEAT_DEVICE_TREE)
#include <mach/mot-gpio-omap.h>
#endif
#include "tda19989.h"

#ifdef TDA19989_CEC_AVAILABLE
#include <linux/regulator/consumer.h>
#endif

#define DEVICE_NAME	"tda19989"

/* TODO: These GPIO numbers should really be part of the platform data */
#define HDMI_INT_PIN_GPIO_NUM	25
#define HDMI_PWR_EN_GPIO_NUM	26

struct tda19989_data {
	struct mutex  mtx; /* Lock for all struct accesses */

	dev_t dev;
	struct cdev cdv;
	struct class *cls;
	int major;

	struct i2c_client *client;

	bool int_enabled;
	bool waiter;
	wait_queue_head_t int_wait;

	bool pwr_enabled;

	bool exiting;

#ifdef TDA19989_CEC_AVAILABLE
	struct regulator *cec_regulator;
#endif
};

static struct tda19989_data *gDev;

/*===========================================================================*/

static irqreturn_t hdmi_int_irq(int irq, void *dev_inst)
{
	printk(KERN_DEBUG "hdmi_int_irq\n");

	if (gDev) {
		if (gDev->waiter && !gDev->exiting)
			wake_up_interruptible(&gDev->int_wait);
	}

	return IRQ_HANDLED;
}

static int i2cTda19989_write(struct i2cMsgArg *pArg)
{
	u8 reg;
	u8 length;
	u8 *pData;
	int rc = 0;

	reg = pArg->firstRegister;
	length = pArg->lenData;
	pData = &pArg->Data[0];

	gDev->client->addr = pArg->slaveAddr;
	while (length--) {
		rc = i2c_smbus_write_byte_data(gDev->client, reg, *pData);
		if (rc != 0) {
			printk(KERN_ERR "I2cTda19989_write error [%d]\n", rc);
			break;
		}

		reg++;
		pData++;
	}
	return ((rc == 0) ? 0 : -EFAULT);
}

static int i2cTda19989_read(struct i2cMsgArg *pArg)
{
	u8 reg;
	u8 length;
	u8 *pData;

	reg = pArg->firstRegister;
	length = pArg->lenData;
	pData = &pArg->Data[0];

	gDev->client->addr = pArg->slaveAddr;
	while (length--) {
		*pData = (u8)i2c_smbus_read_byte_data(gDev->client, reg);
		reg++;
		pData++;
	}
	return 0;
}

/*===========================================================================*/

static int tda19989_open(struct inode *inode, struct file *filp)
{
	int rc;

#ifdef TDA19989_CEC_AVAILABLE
	struct res_handle *cec_rhandle;
#endif

	printk(KERN_DEBUG "tda19989_open\n");

	if (gDev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&gDev->mtx);

	gDev->exiting = false;

	gDev->waiter = false;
	init_waitqueue_head(&gDev->int_wait);

	gDev->pwr_enabled = false;
	rc = gpio_request(HDMI_PWR_EN_GPIO_NUM, "HDMI_PWR_EN");
	if (rc < 0) {
		printk(KERN_ERR "tda19989 GPIO Pwr On request error\n");
		rc = -EFAULT;
		goto failed;
	}
	gpio_direction_output(HDMI_PWR_EN_GPIO_NUM, 0);

	gDev->int_enabled = false;
	rc = gpio_request(HDMI_INT_PIN_GPIO_NUM, "HDMI_INT");
	if (rc < 0) {
		printk(KERN_ERR "tda19989 GPIO INT request error\n");
		rc = -EFAULT;
		goto failed_gpio;
	}
	gpio_direction_input(HDMI_INT_PIN_GPIO_NUM);
	set_irq_type(gpio_to_irq(HDMI_INT_PIN_GPIO_NUM),
						IRQ_TYPE_EDGE_FALLING);
	rc = request_irq(gpio_to_irq(HDMI_INT_PIN_GPIO_NUM), hdmi_int_irq,
					IRQF_TRIGGER_FALLING | IRQF_DISABLED,
					DEVICE_NAME, (void *)NULL);
	if (rc != 0) {
		printk(KERN_ERR	"tda19989 req irq err (%d)\n", rc);
		rc = -EFAULT;
		goto failed_irq;
	}
	disable_irq(OMAP_GPIO_IRQ(HDMI_INT_PIN_GPIO_NUM));

#ifdef TDA19989_CEC_AVAILABLE
	if (gpio_get_value(MMC_DETECT_GPIO_NUM)) {
		cec_regulator = regulator_get(NULL, "vwlan2");
		if (IS_ERR(cec_regulator)) {
			printk(KERN_ERR "tda19989 get regulator failed\n");
			rc = -ENODEV;
			goto failed_irq
		}
		if (regulator_enable(cec_regulator) < 0) {
			printk(KERN_ERR "tda19989 enable regulator failed\n");
			rc = -ENODEV;
			goto failed_irq
		}
		regulator_set_voltage(cec_regulator, 3300000, 3300000);
	} else {
		printk(KERN_ERR "tda19989 mmc is inserted\n");
	}
#endif

	mutex_unlock(&gDev->mtx);

	return 0;

failed_irq:
	gpio_free(HDMI_INT_PIN_GPIO_NUM);
failed_gpio:
	gpio_free(HDMI_PWR_EN_GPIO_NUM);
failed:
	mutex_unlock(&gDev->mtx);
	return rc;
}

static int tda19989_release(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "tda19989_release\n");

	if (gDev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&gDev->mtx);

	gDev->exiting = true;

	if (gDev->int_enabled)
		disable_irq(OMAP_GPIO_IRQ(HDMI_INT_PIN_GPIO_NUM));
	free_irq(gpio_to_irq(HDMI_INT_PIN_GPIO_NUM), (void *)NULL);
	gpio_direction_input(HDMI_INT_PIN_GPIO_NUM);
	gpio_free(HDMI_INT_PIN_GPIO_NUM);

	if (gDev->waiter)
		wake_up_interruptible(&gDev->int_wait);

	gpio_direction_input(HDMI_PWR_EN_GPIO_NUM);
	gpio_free(HDMI_PWR_EN_GPIO_NUM);

	/* TODO: release CEC resource? */

	mutex_unlock(&gDev->mtx);

	return 0;
}

static ssize_t tda19989_read(struct file *fp, char __user *buf,
						size_t count, loff_t *ppos)
{
	int rc;

	printk(KERN_DEBUG "tda19989_read\n");

	mutex_lock(&gDev->mtx);

	if (gDev->exiting) {
		rc = -EINVAL;
		goto exit;
	}

	if (gDev->waiter) {
		rc = -EBUSY;
		goto exit;
	}

	printk(KERN_DEBUG "waiting ...\n");
	gDev->waiter = true;
	mutex_unlock(&gDev->mtx);

	interruptible_sleep_on(&gDev->int_wait);

	mutex_lock(&gDev->mtx);
	printk(KERN_DEBUG "exit waiting\n");
	gDev->waiter = false;

exit:
	mutex_unlock(&gDev->mtx);

	return 0;
}

static ssize_t tda19989_write(struct file *file, const char __user *buf,
						size_t count, loff_t *ppos)
{
	printk(KERN_DEBUG "tda19989_write\n");
	return 0;
}

static int tda19989_ioctl(struct inode *inode, struct file *filp,
						u_int cmd, u_long arg)
{
	int rc = 0;
	struct i2cMsgArg mArg;
	int en;

#ifdef TDA19989_CEC_AVAILABLE
	int i;
	struct timeval prevTime, curTime, resultTime;
#endif

	if (unlikely(_IOC_TYPE(cmd) != TDA19989_IOCTL_MAGIC)) {
		printk(KERN_ERR "Bad command value (%d)\n", cmd);
		return -EINVAL;
	}

	mutex_lock(&gDev->mtx);

	switch (cmd) {
	case TDA19989_I2C_WRITE:
		if (copy_from_user((char *)&mArg, (char *)arg, sizeof(mArg))) {
			printk(KERN_ERR	"tda19989: WRITE copy from error\n");
			rc = -EFAULT;
			break;
		}
		rc = i2cTda19989_write(&mArg);
		break;
	case TDA19989_I2C_READ:
		if (copy_from_user((char *)&mArg, (char *)arg, sizeof(mArg))) {
			printk(KERN_ERR	"tda19989: READ copy from error\n");
			rc = -EFAULT;
			break;
		}
		rc = i2cTda19989_read(&mArg);
		if (copy_to_user((char *)arg, (char *)&mArg, sizeof(mArg))) {
			printk(KERN_ERR	"tda19989: READ copy to error\n");
			rc = -EFAULT;
		}
		break;
	case TDA19989_PWR_ENABLE:
		if (copy_from_user(&en, (int *)arg, sizeof(en))) {
			printk(KERN_ERR	"tda19989: 5V EN copy from error\n");
			rc = -EFAULT;
			break;
		}
		if (en && !gDev->pwr_enabled) {
			gpio_set_value(HDMI_PWR_EN_GPIO_NUM, 1);
			gDev->pwr_enabled = true;
		} else if (!en && gDev->pwr_enabled) {
			gpio_set_value(HDMI_PWR_EN_GPIO_NUM, 0);
			gDev->pwr_enabled = false;
		}
		break;
	case TDA19989_INT_ENABLE:
		if (copy_from_user(&en, (int *)arg, sizeof(en))) {
			printk(KERN_ERR	"tda19989: INT EN copy from error\n");
			rc = -EFAULT;
			break;
		}
		if (en && !gDev->int_enabled) {
			enable_irq(OMAP_GPIO_IRQ(HDMI_INT_PIN_GPIO_NUM));
			gDev->int_enabled = true;
		} else if (!en && gDev->int_enabled) {
			disable_irq(OMAP_GPIO_IRQ(HDMI_INT_PIN_GPIO_NUM));
			gDev->int_enabled = false;
		}
		break;
#ifdef TDA19989_CEC_AVAILABLE
	case TDA19989_CEC_CAL_TIME:
		gpio_direction_output(HDMI_INT_PIN_GPIO_NUM, 0);
		gpio_set_value(HDMI_INT_PIN_GPIO_NUM, 0);

		do_gettimeofday(&prevTime);
		mdelay(9);
		for (i = 0; i < 500; i++) {
			do_gettimeofday(&curTime);
			resultTime.tv_usec = curTime.tv_usec-prevTime.tv_usec;
			if (resultTime.tv_usec > 9980)
				break;
			udelay(2);
		}

		gpio_set_value(HDMI_INT_PIN_GPIO_NUM, 1);
		do_gettimeofday(&curTime);

		gpio_direction_output(HDMI_INT_PIN_GPIO_NUM, 1);
		resultTime.tv_usec = curTime.tv_usec - prevTime.tv_usec;
		printk(KERN_DEBUG "Time interval: %d\n",
						(int)resultTime.tv_usec);
		break;
#endif
	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&gDev->mtx);

	return rc;
}

static const struct file_operations tda19989_fops = {
	.owner = THIS_MODULE,
	.open = tda19989_open,
	.release = tda19989_release,
	.read = tda19989_read,
	.write = tda19989_write,
	.ioctl = tda19989_ioctl,
};

static int i2cTda19989_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	printk(KERN_DEBUG "I2cTda19989_Probe\n");

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR "tda19989 Can't support SMBUS\n");
		return -ENODEV;
	}
	gDev->client = client;

	return 0;
}

static int i2cTda19989_remove(struct i2c_client *client)
{
	printk(KERN_DEBUG "I2cTda19989_Remove\n");
	return 0;
}

static const struct i2c_device_id tda19989_id[] = {
	{ DEVICE_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, tda19989_id);

static struct i2c_driver i2c_driver_tda19989 = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = i2cTda19989_probe,
	.remove = __devexit_p(i2cTda19989_remove),
	.id_table = tda19989_id,
};

static int tda19989_probe(struct platform_device *pdev)
{
	int rc = 0;

	printk(KERN_DEBUG "tda19989_probe\n");

	gDev = kzalloc(sizeof(struct tda19989_data), GFP_KERNEL);
	if (gDev == NULL)
		return -ENOMEM;

	memset(gDev, 0, sizeof(gDev));

	mutex_init(&gDev->mtx);

	gDev->client = NULL;
	rc = i2c_add_driver(&i2c_driver_tda19989);
	if (rc) {
		printk(KERN_ERR "tda19989: i2c add dvr failed (%d)\n", rc);
		goto failed_i2c;
	}

	rc = alloc_chrdev_region(&gDev->dev, 0, 1, DEVICE_NAME);
	if (rc) {
		printk(KERN_ERR "tda19989: alloc chrdev failed (%d)\n", rc);
		goto failed_chrdev;
	}

	cdev_init(&gDev->cdv, &tda19989_fops);
	gDev->cdv.owner = THIS_MODULE;
	rc = cdev_add(&gDev->cdv, gDev->dev, 1);
	if (rc) {
		printk(KERN_ERR "tda19989: add cdev failed (%d)\n", rc);
		goto failed_cdev;
	}
	gDev->major = MAJOR(gDev->dev);

	gDev->cls = class_create(THIS_MODULE, DEVICE_NAME);
	if (!gDev->cls) {
		printk(KERN_ERR "can't create tda19989 class \n");
		rc = -EFAULT;
		goto failed_cls;
	}

	device_create(gDev->cls, NULL, gDev->dev, NULL,	DEVICE_NAME);

	return 0;

failed_cls:
	cdev_del(&gDev->cdv);
failed_cdev:
	unregister_chrdev_region(gDev->dev, 1);
failed_chrdev:
	i2c_del_driver(&i2c_driver_tda19989);
failed_i2c:
	kfree(gDev);
	gDev = NULL;

	return rc;
}

static int tda19989_remove(struct platform_device *pdev)
{
	printk(KERN_DEBUG "tda19989_remove\n");

	device_destroy(gDev->cls, gDev->dev);
	class_destroy(gDev->cls);
	cdev_del(&gDev->cdv);
	unregister_chrdev_region(gDev->dev, 1);
	i2c_del_driver(&i2c_driver_tda19989);
	kfree(gDev);
	gDev = NULL;

	return 0;
}

#if defined(CONFIG_PM)
static int tda19989_suspend(struct platform_device *pdev, pm_message_t event)
{
	printk(KERN_DEBUG "tda19989_suspend\n");
	/*need to check how to control tda19989 power state*/
	return 0;
}

static int tda19989_resume(struct platform_device *pdev)
{
	printk(KERN_DEBUG "tda19989_resume\n");
	/*need to check how to control tda19989 power state*/
	return 0;
}
#endif

static struct platform_driver tda19989_driver = {
	.probe		= tda19989_probe,
	.remove		= tda19989_remove,
#if defined(CONFIG_PM)
	.suspend	= tda19989_suspend,
	.resume		= tda19989_resume,
#endif
	.driver		= {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
};

static struct platform_device tda19989_device = {
	.name		= DEVICE_NAME,
};

static int __init hdmiTda19989_init(void)
{
	int rc;

	rc = platform_device_register(&tda19989_device);
	if (rc != 0) {
		printk(KERN_ERR "tda19989 device register failed (%d)\n", rc);
		goto failed;
	}

	rc = platform_driver_probe(&tda19989_driver, tda19989_probe);
	if (rc != 0) {
		printk(KERN_ERR "tda19989 register/probe failed (%d)\n", rc);
		goto failed_dev;
	}

	return 0;

failed_dev:
	platform_device_unregister(&tda19989_device);
failed:
	return -ENODEV;
}

static void __exit hdmiTda19989_exit(void)
{
	platform_device_unregister(&tda19989_device);
	platform_driver_unregister(&tda19989_driver);
}

module_init(hdmiTda19989_init);
module_exit(hdmiTda19989_exit);

MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
