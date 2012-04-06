/*
 * drivers/media/video/tda19989.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

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
#ifdef TDA19989_CEC_AVAILABLE
#define MMC_DETECT_GPIO_NUM 163
#endif

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

	spinlock_t int_lock; /* Spin lock for missing interrupt*/
	bool int_occurred;

	bool exiting;

#ifdef TDA19989_CEC_AVAILABLE
	struct regulator *cec_regulator;
#endif
	bool cec_use_reg;
};

static struct tda19989_data *gDev;

/*===========================================================================*/

static irqreturn_t hdmi_int_irq(int irq, void *dev_inst)
{
	printk(KERN_DEBUG "hdmi_int_irq\n");

	if (gDev) {
		spin_lock(&gDev->int_lock);
		gDev->int_occurred = true;
		wake_up_interruptible(&gDev->int_wait);
		spin_unlock(&gDev->int_lock);
	}

	return IRQ_HANDLED;
}

static int check_int(void)
{
	int rc = 0;
	unsigned long lock_flags;

	spin_lock_irqsave(&gDev->int_lock, lock_flags);
	if (gDev->int_occurred)	{
		rc = 1;
		gDev->int_occurred = false;
	} else if (gDev->exiting || !gDev->int_enabled) {
		rc = 1;
	}
	spin_unlock_irqrestore(&gDev->int_lock, lock_flags);

	return rc;
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

	if ((length > 1) && (pArg->slaveAddr == 0x34)) {
		rc = i2c_smbus_write_i2c_block_data(
				gDev->client, reg, length, pData);
		if (rc != 0) {
			printk(KERN_ERR "i2cTda19989 W(B) err:%d\n", rc);
		}
	} else {
		while (length--) {
			rc = i2c_smbus_write_byte_data(
					gDev->client, reg, *pData);
			if (rc != 0) {
				printk(KERN_ERR "I2cTda19989 W err:%d\n", rc);
				break;
			}
			reg++;
			pData++;
		}
	}
#ifdef TDA19989_CEC_AVAILABLE
	if (pArg->slaveAddr == 0x34)
	{
		mdelay(2);
	}
#endif
	return ((rc == 0) ? 0 : -EFAULT);
}

static int i2cTda19989_read(struct i2cMsgArg *pArg)
{
	u8 reg;
	u8 length;
	u8 *pData;
	int rc = 0;

	reg = pArg->firstRegister;
	length = pArg->lenData;
	pData = &pArg->Data[0];

	gDev->client->addr = pArg->slaveAddr;

	if ((length > 1) && (pArg->slaveAddr == 0x34)) {
		rc = i2c_smbus_read_i2c_block_data(
				gDev->client, reg, length, pData);
		if ((u8)rc != length) {
			printk(KERN_ERR "i2cTda19989 R(B) err:%d\n", rc);
			return -EFAULT;
		}
	} else {
		while (length--) {
			rc = i2c_smbus_read_byte_data(gDev->client, reg);
			if (rc < 0) {
				printk(KERN_ERR "i2cTda19989 R err:%d\n", rc);
				return -EFAULT;
			}
			*pData = (u8)rc ;
			reg++;
			pData++;
		}
	}
	return 0;
}

#ifdef TDA19989_CEC_AVAILABLE
static int cec_calibration(void)
{
	int i;
	int rc = 0;
	int prevInt_en = 0;
	struct timeval prevTime, curTime, resultTime;

	if (gDev->int_enabled) {
		prevInt_en = 1;
		disable_irq(OMAP_GPIO_IRQ(HDMI_INT_PIN_GPIO_NUM));
		if (gDev->waiter)
			wake_up_interruptible(&gDev->int_wait);
		gDev->int_enabled = false;
	}
	gpio_direction_output(HDMI_INT_PIN_GPIO_NUM, 0);
	gpio_set_value(HDMI_INT_PIN_GPIO_NUM, 0);

	do_gettimeofday(&prevTime);
	mdelay(9);
	for (i = 0; i < 500; i++) {
		do_gettimeofday(&curTime);
		resultTime.tv_usec = curTime.tv_usec-prevTime.tv_usec;
		if (resultTime.tv_usec > 9990)
			break;
		udelay(2);
	}
	gpio_set_value(HDMI_INT_PIN_GPIO_NUM, 1);
	do_gettimeofday(&curTime);

	gpio_direction_input(HDMI_INT_PIN_GPIO_NUM);
	resultTime.tv_usec = curTime.tv_usec - prevTime.tv_usec;
	printk(KERN_DEBUG "Time interval: %d\n", (int)resultTime.tv_usec);

	if (prevInt_en) {
		enable_irq(OMAP_GPIO_IRQ(HDMI_INT_PIN_GPIO_NUM));
		gDev->int_enabled = true;
	}

	return rc;
}

static int cec_regulator_enable(bool en)
{
	int rc = 0;
	int reg_en;
	reg_en = regulator_is_enabled(gDev->cec_regulator);

	if (en) {
		if (!reg_en) {
			if (regulator_enable(gDev->cec_regulator) < 0) {
				printk(KERN_ERR "cec regulator enable failed\n");
				rc = -1;
			}
		} else {
			printk(KERN_DEBUG "already regulator is set\n");
		}
	} else {
		if (reg_en && gpio_get_value(MMC_DETECT_GPIO_NUM)) {
			if (regulator_disable(gDev->cec_regulator) < 0) {
				printk(KERN_ERR "cec regulator disable failed\n");
				rc = -1;
			}
		} else {
			printk(KERN_DEBUG "MMC or Reg disabled:%d\n", reg_en);
		}
	}
	return ((rc == 0) ? 0 : -EFAULT);
}
#endif

int hdmiCec_useReg(void)
{
    return (int)gDev->cec_use_reg;
}
EXPORT_SYMBOL(hdmiCec_useReg);

/*===========================================================================*/

static int tda19989_open(struct inode *inode, struct file *filp)
{
	int rc;

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
	gDev->cec_regulator = regulator_get(NULL, "vwlan2");
	if (IS_ERR(gDev->cec_regulator)) {
		printk(KERN_ERR "cec get regulator failed\n");
		rc = -ENODEV;
		goto failed_irq;
	}
#endif

	gDev->cec_use_reg = false;

	gDev->int_occurred = false;

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

#ifdef TDA19989_CEC_AVAILABLE
	if (gDev->cec_regulator)
		regulator_put(gDev->cec_regulator);
#endif

	mutex_unlock(&gDev->mtx);

	return 0;
}

static ssize_t tda19989_read(struct file *fp, char __user *buf,
						size_t count, loff_t *ppos)
{
	int rc = 0;

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

	if (!gDev->int_enabled) {
		rc = -EAGAIN;
		goto exit;
	}

	printk(KERN_DEBUG "waiting ...\n");
	gDev->waiter = true;

	mutex_unlock(&gDev->mtx);

	wait_event_interruptible(gDev->int_wait, check_int());

	mutex_lock(&gDev->mtx);

	gDev->waiter = false;
	printk(KERN_DEBUG "exit waiting\n");

exit:
	mutex_unlock(&gDev->mtx);

	return rc;
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
			if (gDev->waiter)
				wake_up_interruptible(&gDev->int_wait);
		}
		break;
#ifdef TDA19989_CEC_AVAILABLE
	case TDA19989_CEC_CAL_TIME:
		rc = cec_calibration();
		if (rc != 0) {
			printk(KERN_ERR	"cec calibration error (%d)\n", rc);
			rc = -EFAULT;
		}
		break;
	case TDA19989_CEC_REG_ENABLE:
		if (copy_from_user(&en, (int *)arg, sizeof(en))) {
			printk(KERN_ERR	"cec pull-up pwr copy from error\n");
			rc = -EFAULT;
			break;
		}
		if (en && !gDev->cec_use_reg) {
			rc = cec_regulator_enable(true);
			gDev->cec_use_reg = true;
		} else if (!en && gDev->cec_use_reg) {
			rc = cec_regulator_enable(false);
			gDev->cec_use_reg = false;
		}
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
	spin_lock_init(&gDev->int_lock);

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
	/*printk(KERN_DEBUG "tda19989_suspend\n");*/
	return 0;
}

static int tda19989_resume(struct platform_device *pdev)
{
	/*printk(KERN_DEBUG "tda19989_resume\n");*/
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

