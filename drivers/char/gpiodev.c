/* 
 * GPIODev                                       gpiodev.c
 *
 * Copyright (C) 2008-2009 Motorola, Inc.
 */

/*
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <mach/gpio.h>
#include <linux/autoconf.h>
#include <linux/gpiodev.h>

#if defined(CONFIG_GPIODEV_DEVICE_TREE)
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

MODULE_AUTHOR("Motorola, Inc.");
MODULE_DESCRIPTION("GPIO device for user space accessing GPIO pins");
MODULE_LICENSE("GPL");

/*
 * GPIODEV_DBG macro is for debugging purpose only
 * Should turn it off while releasing official version
 */
#define GPIODEV_DBG 0

#if GPIODEV_DBG
#define trace_msg(fmt, args...)  printk("\n" KERN_ALERT fmt "\n", ##args)
#else
#define trace_msg(fmt, arg...) do { } while(0)
#endif

#define GPIODEV_NAME  "gpiodev"

/*
 * GPIODev state flags
 */
#define GPIODEV_FLAG_OPEN           (0x1 << 0)
#define GPIODEV_FLAG_LOWLEVELACCESS (0x1 << 1)
#define GPIODEV_FLAG_CONFIGURABLE   (0x1 << 2)
#define GPIODEV_FLAG_INTERRUPTED    (0x1 << 3)

const unsigned int int_type[GPIODEV_INTTYPE_MAX] = {
	IRQF_TRIGGER_NONE,
	IRQF_TRIGGER_RISING,
	IRQF_TRIGGER_FALLING,
	(IRQF_TRIGGER_HIGH | IRQF_TRIGGER_RISING),
	(IRQF_TRIGGER_LOW | IRQF_TRIGGER_FALLING)
};

#define GET_INT_TYPE(a)    (int_type[((a) & GPIODEV_CONFIG_INT_MASK) >> GPIODEV_CONFIG_INT_MASK_OFFSET])
#define GPIODEV_IS_INTERRUPTABLE(a)    ((((a) & GPIODEV_CONFIG_INT_MASK) != GPIODEV_CONFIG_INT_NONE) ? 1 : 0)

#define GPIODEV_CONFIG_TYPE unsigned int

/*
 * GPIODEVICE defines a GPIO
 *     pin_nr - pin number of the GPIO
 *     device_name - name which shows up in /dev
 *     init_config - the default configuration of a GPIO
 *     current_config - the current configuration the GPIO 
 *     flags -  status of the device node (see GPIODev state flags above)
 *     event_queue - queue to sleep on while waiting for an interrupt
 */

typedef struct GPIODEVICE {
	unsigned int pin_nr;
#if defined(CONFIG_GPIODEV_DEVICE_TREE)
	char device_name[DT_PROP_GPIODEV_NAMELEN];
#else
	char *device_name;
#endif
	GPIODEV_CONFIG_TYPE init_config;
	GPIODEV_CONFIG_TYPE current_config;
	unsigned int flags;
	wait_queue_head_t event_queue;
	struct mutex lock;
} GPIODEVICE;

/*
 * gpio_devs[] is the actual description of each GPIO to be supported
 */
#if defined(CONFIG_GPIODEV_DEVICE_TREE)

static struct GPIODEVICE *gpio_devs;
static int gpio_devs_size = 0;
static int gpio_devs_count = 0;

#define GPIO_DEVICE_COUNT gpio_devs_size

/**
 * Below structure definition should strictly comform to corresponding 
 * HW device tree format
 */
struct omap_gpiodev_entry {
	u32 pin_num;				/* GPIO pin number  */
	char name[DT_PROP_GPIODEV_NAMELEN];	/* GPIODev name */
	u32 setting;				/* GPIO pin setting, e.g. input, output, interrupt, invalid */
} __attribute__ ((__packed__));

static void gpiodev_devs_init(void *p_data)
{
	struct omap_gpiodev_entry *p = p_data;
	struct GPIODEVICE *p_devs = &gpio_devs[gpio_devs_count];
	int i = 0;

	while ((i<DT_PROP_GPIODEV_NAMELEN) && (' ' != p->name[i++]));
	p->name[i-1] = '\0';

	if (gpio_devs_count++ >= gpio_devs_size) {
		printk(KERN_ERR "Too big gpiodev count \n");
		return;
	}

	p_devs->pin_nr = p->pin_num;
	strcpy(p_devs->device_name, p->name);
	p_devs->init_config = p->setting; 
	p_devs->current_config = GPIODEV_CONFIG_INVALID;
	p_devs->flags = GPIODEV_FLAG_CONFIGURABLE | GPIODEV_FLAG_LOWLEVELACCESS;
}

static int gpio_devs_hwcfg_init(void)
{
	int size, i;
	struct device_node *gpio_node;
	const void *gpio_prop;

	if ((gpio_node = of_find_node_by_path(DT_PATH_GPIOGEV))) {
		if ((gpio_prop = of_get_property(gpio_node, DT_PROP_GPIODEV_INIT, &size))) {

			gpio_devs_size = (size / sizeof(struct omap_gpiodev_entry));
			trace_msg("gpio_dev_size = %d", gpio_devs_size);

			/* allocate space for the table */
			gpio_devs = kmalloc(gpio_devs_size * sizeof(struct GPIODEVICE), GFP_KERNEL);

			if (gpio_devs == NULL) {
				printk(KERN_ERR "Unable to allocate space for GPIODev array\n");
				return -1;
			} else
				for (i=0; i<gpio_devs_size; i++)
					gpiodev_devs_init((struct omap_gpiodev_entry *)gpio_prop + i);
		}

		of_node_put(gpio_node);
	}

	return 0;
}

static void gpio_devs_hwcfg_cleanup(void)
{
	kfree((void *)gpio_devs);
}

#else

GPIODEVICE gpio_devs[] = {
#if GPIODEV_DBG
	{
	 111,
	 "slide_interrupt",
	 GPIODEV_CONFIG_INPUT | GPIODEV_CONFIG_INT_LLEV,
	 GPIODEV_CONFIG_INVALID,
	 GPIODEV_FLAG_CONFIGURABLE | GPIODEV_FLAG_LOWLEVELACCESS},
#endif
	{
	 149,
	 "gps_rts",
	 GPIODEV_CONFIG_OUTPUT_LOW,
	 GPIODEV_CONFIG_INVALID,
	 GPIODEV_FLAG_CONFIGURABLE | GPIODEV_FLAG_LOWLEVELACCESS},
	{
	 59,
	 "gps_reset",
	 GPIODEV_CONFIG_OUTPUT_LOW,
	 GPIODEV_CONFIG_INVALID,
	 GPIODEV_FLAG_CONFIGURABLE | GPIODEV_FLAG_LOWLEVELACCESS},
	{
	 136,
	 "gps_standby",
	 GPIODEV_CONFIG_OUTPUT_LOW,
	 GPIODEV_CONFIG_INVALID,
	 GPIODEV_FLAG_CONFIGURABLE | GPIODEV_FLAG_LOWLEVELACCESS},
	{
	 160,
	 "gps_interrupt",
	 GPIODEV_CONFIG_INPUT | GPIODEV_CONFIG_INT_REDG,
	 GPIODEV_CONFIG_INVALID,
	 GPIODEV_FLAG_CONFIGURABLE | GPIODEV_FLAG_LOWLEVELACCESS}
};

#define GPIO_DEVICE_COUNT   (sizeof(gpio_devs)/sizeof(gpio_devs[0]))
#endif /* defined(CONFIG_GPIODEV_DEVICE_TREE) */

static unsigned long gpiodev_major = 0;
static struct class *gpiodev_class;

/*
 * GPIODev_ISR handles an interrupt on a GPIO triggered by the parameters
 * provided when setting up the line. The interrupt that occurs here will
 * be for a specific GPIO previously configured for a particular interrupt.
 * The function will wake up any process waiting for this interrupt to be 
 * triggered.
 */
static irqreturn_t GPIODev_ISR(int irq, void *param)
{
	GPIODEVICE *dev;

	dev = (GPIODEVICE *) param;

	dev->flags |= GPIODEV_FLAG_INTERRUPTED;

	disable_irq(gpio_to_irq(dev->pin_nr));
	wake_up_interruptible(&dev->event_queue);

	trace_msg("gpio%d interrupt occurs", dev->pin_nr);

	return IRQ_HANDLED;
}

/*
 * ConfigureGPIO configures the way a GPIO operates and adjusts the 
 * current configuration byte appropriately. Any resources in use
 * prior to the reconfiguration will be released and the new configuration
 * will take effect.
 */
static unsigned long ConfigureGPIO(GPIODEVICE * dev, GPIODEV_CONFIG_TYPE newconfig)
{
	unsigned long ret = 0;

	trace_msg("Configure gpio%d with config 0x%04x instead of 0x%04x",
		  dev->pin_nr, newconfig, dev->current_config);


	/* If new conig is invalid, release gpio resource */
	if ( newconfig & GPIODEV_CONFIG_INVALID ) {
		/* Free any currently consumed resources */
		if (!(dev->current_config & GPIODEV_CONFIG_INVALID)) {
			if (GPIODEV_IS_INTERRUPTABLE(dev->current_config)) {
				free_irq(gpio_to_irq(dev->pin_nr), dev);
				dev->flags &= ~GPIODEV_FLAG_INTERRUPTED;
			}
			gpio_free(dev->pin_nr);
		}
	}
	else {
		/* If current gpio is invalid, request gpio resource*/
		if (dev->current_config & GPIODEV_CONFIG_INVALID) {
			if (gpio_request(dev->pin_nr, NULL) < 0) {
				printk(KERN_ERR "Failed to request GPIO for %s \n",
			    	   dev->device_name);
				ret = -EBUSY;
				goto end;
			}
		}

		/* 
		 * Configure GPIO with newconfig
		 */
		/* Step 1: direction and level setting */
		if (newconfig & GPIODEV_CONFIG_INPUT) {
			gpio_direction_input(dev->pin_nr);
		} 
		else if (newconfig & GPIODEV_CONFIG_OUTPUT_HIGH) {
			gpio_direction_output(dev->pin_nr, 1);
		}
		else if (newconfig & GPIODEV_CONFIG_OUTPUT_LOW) {
			gpio_direction_output(dev->pin_nr, 0);
		}
		/* Step 2: request interrupt */
		if (GPIODEV_IS_INTERRUPTABLE(newconfig)) {
			if (((newconfig & GPIODEV_CONFIG_INT_MASK) >> GPIODEV_CONFIG_INT_MASK_OFFSET)
				>= GPIODEV_INTTYPE_MAX) {
				ret = -EINVAL;
				goto end;
			}
			else if (request_irq(gpio_to_irq(dev->pin_nr),
					     &GPIODev_ISR,
					     GET_INT_TYPE(newconfig),
					     dev->device_name, dev) ) {
				printk(KERN_ERR "Fail to request irq for \"%s\"\n", 
						dev->device_name);
				gpio_free(dev->pin_nr);
				ret = -EBUSY;
				goto end;
			}
		}
	}

	dev->current_config = newconfig;
end:
	return ret;
}


/*
 * GPIODev_Open handles a userspace open() to our driver. This function takes
 * the specified device out of the default state by configuring the GPIO.
 */
static int GPIODev_Open(struct inode *inode, struct file *filp)
{
	GPIODEVICE *dev;
	unsigned long minor;
	int ret = 0;

	minor = MINOR(inode->i_rdev);

	if (minor >= GPIO_DEVICE_COUNT)
		return -ENODEV;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (dev->flags & GPIODEV_FLAG_OPEN) {
		ret = -EBUSY;
		goto end;
	}

#if GPIODEV_DBG
	if (dev->pin_nr == 111) {
		free_irq(gpio_to_irq(dev->pin_nr), NULL);
		gpio_free(dev->pin_nr);
	}
#endif

	ret = ConfigureGPIO(dev, dev->init_config);
	if (ret != 0) 
		goto end;

	dev->flags |= GPIODEV_FLAG_OPEN;
	if (!try_module_get(THIS_MODULE)) {
		ret = -EINVAL;
		module_put(THIS_MODULE);
		goto end;
	}

end:
	mutex_unlock(&dev->lock);
	trace_msg("open function return value 0x%04x", ret);
	return ret;
}

/*
 * GPIODev_Close handles a userspace close() on a device previously
 * opened. The close reverses any initialization done on a device.
 */
static int GPIODev_Close(struct inode *inode, struct file *filp)
{
	GPIODEVICE *dev;
	unsigned long minor;
	int ret = 0;

	minor = MINOR(inode->i_rdev);

	if (minor >= GPIO_DEVICE_COUNT)
		return -ENODEV;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (!(dev->flags & GPIODEV_FLAG_OPEN)) {
		ret = 0;
		goto end;
	}

	if (dev->flags & GPIODEV_FLAG_CONFIGURABLE) {
		ret = ConfigureGPIO(dev, GPIODEV_CONFIG_INVALID);
		if (ret != 0)
			goto end;
	}

	dev->flags &= ~GPIODEV_FLAG_OPEN;
	module_put(THIS_MODULE);

end:
	mutex_unlock(&dev->lock);
	trace_msg("close function return value 0x%04x", ret);
	return ret;
}

/*
 * GPIODev_Read handles a userspace read() on an open device. 
 * The read will only read a single byte. A read on a GPIO
 * configured for output will return the value currently in the output
 * register.
 */
static ssize_t GPIODev_Read(struct file *filp, char *buf, size_t count,
			    loff_t * f_pos)
{
	GPIODEVICE *dev;
	unsigned long minor;
	unsigned long result;
	unsigned char value;
	int ret = 1;

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	if (minor >= GPIO_DEVICE_COUNT)
		return -ENODEV;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (!(dev->flags & GPIODEV_FLAG_OPEN)) {
		ret = -EBADF;
		goto end;
	}
	else if ((dev->current_config & GPIODEV_CONFIG_OUTPUT_LOW) 
			|| (dev->current_config & GPIODEV_CONFIG_OUTPUT_HIGH)) {
		ret = -EPERM;
		goto end;
	}

	if (count < 1) {
		ret = 0;
		goto end;
	}

	value = gpio_get_value(dev->pin_nr);
	trace_msg("Reading gpio%d with 0x%02x", dev->pin_nr, value);

	result = copy_to_user(buf, &value, 1);
	if (result)
		ret = -EFAULT;

end:
	mutex_unlock(&dev->lock);
	return ret;
}

/*
 * GPIODev_Write handles a userspace write() on an open device. The write
 * must be one byte in length or the call will fail. A write on a GPIO
 * configured for input will not affect the line's status.
 */
static ssize_t GPIODev_Write(struct file *filp, const char *buf, size_t count,
			     loff_t * f_pos)
{
	GPIODEVICE *dev;
	unsigned long minor;
	unsigned long result;
	unsigned char value;
	int ret = 1;

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	if (minor >= GPIO_DEVICE_COUNT)
		return -ENODEV;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (!(dev->flags & GPIODEV_FLAG_OPEN)) {
		ret = -EBADF;
		goto end;
	}
	else if (dev->current_config & GPIODEV_CONFIG_INPUT) {
		ret = -EPERM;
		goto end;
	}

	if (count < 1) {
		ret = 0;
		goto end;
	}

	value = 0;

	result = copy_from_user(&value, buf, 1);
	if (result) {
		ret = -EFAULT;
		goto end;
	}

	if (value > 1) {
		ret = -EINVAL;
		goto end;
	}

	gpio_set_value(dev->pin_nr, value);
	trace_msg("Writing gpio%d with 0x%02x", dev->pin_nr, value);

end:
	mutex_unlock(&dev->lock);
	trace_msg("write function return value = 0x%04x", ret);
	return ret;
}

/*
 * GPIODev_IOCtl handles the configuration of a GPIO. The following commands are
 * supported:
 * GPIODEV_GET_CONFIG - returns the 1 byte configuration of the GPIO
 * GPIODEV_SET_CONFIG - sets up the 1 byte configuration of the GPIO
 * GPIODEV_INT_REENABLE - If the line is responsive to interrupts, ints will be
 *                        reenabled.
 * GPIODEV_GET_LOWLEVELCONFIG - Get the low level details of a device. If the device doesn't
 *                              allow this operation an error is returned.
 * GPIODEV_SET_LOWLEVELCONFIG - Set the low level details of a device. If the device
 *                              doesn't allow this operation an error is returned. If
 *                              the low level configuration to be set is invalid an
 *                              error is returned.
 * GPIODEV_INT_POLL - Wait for gpiodev interrupt occurrence.
 *
 */
static int GPIODev_IOCtl(struct inode *inode, struct file *filp,
			 unsigned int cmd, unsigned long arg)
{
	GPIODEV_LOWLEVEL_CONFIG llconf;
	GPIODEVICE *dev;
	unsigned long minor;
	unsigned long result;
	GPIODEV_CONFIG_TYPE value;
	int ret = 0;

	minor = MINOR(inode->i_rdev);

	if (minor >= GPIO_DEVICE_COUNT)
		return -ENODEV;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (!(dev->flags & GPIODEV_FLAG_OPEN)) {
		ret = -EBADF;
		goto end;
	}

	result = 0;

	switch (cmd) {
	case GPIODEV_GET_CONFIG:
		{
			result =
			    copy_to_user((unsigned char *) arg,
					 &dev->current_config, sizeof(GPIODEV_CONFIG_TYPE));
			if (result) {
				ret = -EFAULT;
				goto end;
			}
		}
		break;

	case GPIODEV_SET_CONFIG:
		{
			if (!(dev->flags & GPIODEV_FLAG_CONFIGURABLE)) {
				ret = -EBADF;
				goto end;
			}

			result =
			    copy_from_user(&value, (unsigned char *) arg, sizeof(GPIODEV_CONFIG_TYPE));
			if (result) {
				ret = -EFAULT;
				goto end;
			}

			ret = ConfigureGPIO(dev, value);
		}
		break;

	case GPIODEV_INT_REENABLE:
		{
			if (!(GPIODEV_IS_INTERRUPTABLE(dev->current_config))) {
				ret = -EPERM;
				goto end;
			}

			dev->flags &= ~GPIODEV_FLAG_INTERRUPTED;
			enable_irq(gpio_to_irq(dev->pin_nr));
		}
		break;

	case GPIODEV_GET_LOWLEVELCONFIG:
		{
			if (!(dev->flags & GPIODEV_FLAG_LOWLEVELACCESS)) {
				ret = -EPERM;
				goto end;
			}

			llconf.config = dev->current_config;

			result = copy_to_user((unsigned int *) arg, &llconf,
					      sizeof(GPIODEV_LOWLEVEL_CONFIG));
			if (result) {
				ret = -EFAULT;
				goto end;
			}
		}
		break;

	case GPIODEV_SET_LOWLEVELCONFIG:
		{
			if (!(dev->flags & GPIODEV_FLAG_LOWLEVELACCESS) ||
				!(dev->flags & GPIODEV_FLAG_CONFIGURABLE)) {
				ret = -EPERM;
				goto end;
			}

			result =
			    copy_from_user(&llconf, (unsigned int *) arg,
					   sizeof(GPIODEV_LOWLEVEL_CONFIG));
			if (result) {
				ret = -EFAULT;
				goto end;
			}

			ret = ConfigureGPIO(dev, llconf.config);
		}
		break;

	case GPIODEV_INT_POLL:
		if (!(GPIODEV_IS_INTERRUPTABLE(dev->current_config))) {
			ret = -EPERM;
			goto end;
		}
		else { 
			trace_msg("evoke wait_event_interruptible calling \n");
			wait_event_interruptible(dev->event_queue, (dev->flags & GPIODEV_FLAG_INTERRUPTED) != 0);
			dev->flags &= ~GPIODEV_FLAG_INTERRUPTED;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

end:
	mutex_unlock(&dev->lock);
	return ret;
}

/*
 * GPIODev_Poll handles a user's select/poll call. This function will
 * return whatever functionality is currently available on a device.
 * The intended use is to wait for interrupts in a select call. By
 * waiting on an exception, a select/poll call will block until an interrupt
 * occurs.
 */
static unsigned int GPIODev_Poll(struct file *filp, poll_table * table)
{
	GPIODEVICE *dev;
	unsigned long minor;
	unsigned int mask = 0;

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	if (minor >= GPIO_DEVICE_COUNT)
		return POLLERR;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (!(dev->flags & GPIODEV_FLAG_OPEN)
	    || !(GPIODEV_IS_INTERRUPTABLE(dev->current_config))) {
		mask |= POLLNVAL;
		goto end;
	}

	trace_msg("evoke wait_event_interruptible calling \n");
	wait_event_interruptible(dev->event_queue, (dev->flags & GPIODEV_FLAG_INTERRUPTED) != 0);
	mask |= POLLPRI;

end:
	dev->flags &= ~GPIODEV_FLAG_INTERRUPTED;
	mutex_unlock(&dev->lock);
	return mask;
}

static const struct file_operations gpiodev_fops = {
	.owner = THIS_MODULE,
	.open = GPIODev_Open,
	.release = GPIODev_Close,
	.read = GPIODev_Read,
	.write = GPIODev_Write,
	.ioctl = GPIODev_IOCtl,
	.poll = GPIODev_Poll,
};

/*
 * gpiodev_cleanup unregisters the character drivers and removes the /dev
 * entries. If for some reason a GPIO still has a reference count, interrupts
 * are disabled on to effectively 'shut it down.'
 */
static void gpiodev_cleanup(unsigned long device_nr)
{
	GPIODEVICE *gpiodev;
	unsigned long index;

	for (index = 0; index < device_nr; index++) {
		gpiodev = &gpio_devs[index];
		device_destroy(gpiodev_class,
				     MKDEV(gpiodev_major, index));
	}

#if defined(CONFIG_GPIODEV_DEVICE_TREE)
	gpio_devs_hwcfg_cleanup();
#endif
 
	class_destroy(gpiodev_class);
	unregister_chrdev(gpiodev_major, GPIODEV_NAME);

}

static void gpiodev_exit(void)
{
	gpiodev_cleanup(gpio_devs_count);
}

/*
 * gpiodev_init sets up the character driver, configures the GPIOs specified
 * in the gpiodevs list, and creates entries in /dev corresponding to
 * each configured GPIO.
 */
int __init gpiodev_init(void)
{
	GPIODEVICE *gpiodev;
	unsigned long index;
	unsigned long result;

	result = register_chrdev(0, GPIODEV_NAME, &gpiodev_fops);
	if (result < 0) {
		printk(KERN_ERR "Failed to register gpiodev %s \n",
		       GPIODEV_NAME);

		return result;
	}

	gpiodev_major = result;
	gpiodev_class = class_create(THIS_MODULE, GPIODEV_NAME);

#if defined(CONFIG_GPIODEV_DEVICE_TREE)
	if(gpio_devs_hwcfg_init() != 0) {
		class_destroy(gpiodev_class);
		unregister_chrdev(gpiodev_major, GPIODEV_NAME);
		return -1;
	}
#endif

	for (index = 0; index < GPIO_DEVICE_COUNT; index++) {
		gpiodev = &gpio_devs[index];

		mutex_init(&gpiodev->lock);
		init_waitqueue_head(&gpiodev->event_queue);
		if (IS_ERR(device_create(gpiodev_class,
					       NULL,
					       MKDEV(gpiodev_major, index),
					       NULL,
					       "%s", gpiodev->device_name))) {
			printk(KERN_ERR
			       "Device \"%s\" could not be created properly\n",
			       gpiodev->device_name);
			gpiodev_cleanup(index);
			return -1;
		}
		trace_msg("Created device %s", gpiodev->device_name);
	}

	printk(KERN_INFO "Mot GPIODev init successfully \n");
	return 0;
}
module_init(gpiodev_init);
module_exit(gpiodev_exit);
