
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
#include <asm/io.h>
#include <asm/uaccess.h>
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

static dev_t tda19989_dev_num;
static struct cdev tda19989_cdev;
static struct class *tda19989_class;
static int tda19989_major = -1;

static struct i2c_client *tda19989_client=NULL;

static int hdmi_int_enabled=0;
static int hdmi_int_done=0;
static int hdmi_sleep_on=0;
wait_queue_head_t hdmi_int_wait;
#ifdef TDA19989_CEC_AVAILABLE
static struct  regulator *cec_regulator;
#endif

static irqreturn_t hdmi_int_irq(int irq, void *dev_inst)
{
    printk("hdmi_int_irq() pre state : %d \n", hdmi_sleep_on);
    if(hdmi_sleep_on)
    {
        wake_up_interruptible(&hdmi_int_wait);
        hdmi_sleep_on=0;
    }
    else
    {
        printk("hdmi_int_irq error !!! \n");
    }

    return IRQ_HANDLED;
}

static ssize_t tda19989_read(struct file *fp, char __user *buf, size_t count, loff_t *ppos)
{
    printk("Tda19989_read() pre state : %d \n", hdmi_sleep_on);

    if(!hdmi_sleep_on)
    {
        printk("sleep ..... enter !!!!  \n");
        hdmi_sleep_on=1;
        interruptible_sleep_on(&hdmi_int_wait);
        printk("sleep ..... exit !!!!  \n");
    }
    else
    {
        printk("tda19989_read error !!! \n");
    }
    return 0;
}

static ssize_t tda19989_write( struct file * file, const char __user * buf, size_t count, loff_t *ppos )
{
    printk("hdmi_write start \n");
    return 0;
}

static int tda19989_open(struct inode * inode, struct file * filp)
{
    int ret;

    printk("tda19989_open start \n");
    init_waitqueue_head(&hdmi_int_wait);

    ret=gpio_request(HDMI_PWR_EN_GPIO_NUM, "HDMI_PWR_EN");
    if(ret<0)
    {
        printk("tda19989 GPIO Pwr On request error !!! \n");
        return -1;
    }
    gpio_direction_output(HDMI_PWR_EN_GPIO_NUM, 0);

    ret=gpio_request(HDMI_INT_PIN_GPIO_NUM, "HDMI_INT");
    if(ret<0)
    {
        printk("tda19989 GPIO INT request error !!! \n");
        gpio_free(HDMI_PWR_EN_GPIO_NUM);
        return -1;
    }
#ifdef TDA19989_CEC_AVAILABLE
    cec_regulator  = regulator_get(NULL, "vwlan2");
    if (IS_ERR(cec_regulator)) {
        printk("tda19989 failed to get regulator for HDMI");
        return -ENODEV;
    }
    printk("pass 3 \n");
    if (regulator_enable(cec_regulator) < 0)
    {
        printk("tda19989 Failed to enable regulator\n");
        return -ENODEV;
    }
     printk("pass 4 \n");
    regulator_set_voltage(cec_regulator,3300000,3300000);
#endif

    return 0;
}

static int tda19989_release(struct inode * inode, struct file * filp)
{
    printk("tda19989_release start \n");
    return 0;
}

static int I2cTda19989_write(i2cKernelModeArg* pArg)
{
    u8 reg;
    u8 length;
    u8 *pData;
    int retval=0;

    reg = pArg->firstRegister;
    length = pArg->lenData;
    pData = &pArg->Data[0];

    tda19989_client->addr=pArg->slaveAddr;
    /*printk("[W] addr = %x, length = %d, pData = %d \n", tda19989_client->addr, length, *pData);*/
    while(length--)
    {
        retval=i2c_smbus_write_byte_data(tda19989_client, reg, *pData);
        if (retval != 0)
        {
            printk("I2cTda19989_write error [%d] \n", retval);
            break;
        }

        reg++;
        pData++;
    }
    return ((retval==0)?0:-1);
}

static int I2cTda19989_read(i2cKernelModeArg* pArg)
{
    u8 reg;
    u8 length;
    u8 *pData;

    reg = pArg->firstRegister;
    length = pArg->lenData;
    pData = &pArg->Data[0];

    tda19989_client->addr=pArg->slaveAddr;
    /*printk("[R] addr = %x, length = %d, pData = %d \n", tda19989_client->addr, length, *pData);*/
    while(length--)
    {
        *pData = (u8)i2c_smbus_read_byte_data(tda19989_client, reg);
        reg++;
        pData++;
    }
    return 0;
}

static int tda19989_ioctl(struct inode * inode, struct file *filp, u_int cmd, u_long arg)
{
    int result=0;

    switch(cmd)
    {
        case HDMI_I2C_WRITE:
        {
            i2cKernelModeArg mArg;
            if(copy_from_user((char*)&mArg, (char*)arg, sizeof(mArg)))
            {
                printk("tda19989 HDMI_I2C_WRITE copy_from_user error \n");
                result = -EFAULT;
                break;
            }
            result=I2cTda19989_write(&mArg);
        }
        break;

        case HDMI_I2C_READ:
        {
            i2cKernelModeArg mArg;
            if(copy_from_user((char*)&mArg, (char*)arg, sizeof(mArg)))
            {
                printk("tda19989 HDMI_I2C_READ copy_from_user error \n");
                result = -EFAULT;
                break;
            }	
            result=I2cTda19989_read(&mArg);
            if(copy_to_user((char*)arg, (char*)&mArg, sizeof(mArg)))
            {
                printk("tda19989 HDMI_I2C_READ copy_to_user error \n");
                result = -EFAULT;
            }
        }
        break;

        case HDMI_PWR_ONOFF:
        {
            int hdmi5VOn;
            if (copy_from_user(&hdmi5VOn, (int*)arg, sizeof(hdmi5VOn)))
            {
                printk("tda19989 HDMI_5V_ENABLE copy_from_user error \n");
                result = -EFAULT;
                break;
            }	
            if(hdmi5VOn)
                gpio_set_value(HDMI_PWR_EN_GPIO_NUM, 1);
            else
                gpio_set_value(HDMI_PWR_EN_GPIO_NUM, 0);
        }
        break;

        case HDMI_INT_ENABLE:
        {
            int hdmiIntEn;
            if (copy_from_user(&hdmiIntEn, (int*)arg, sizeof(hdmiIntEn)))
            {
                printk("tda19989 HDMI_INT_ENABLE copy_from_user error \n");
                result = -EFAULT;
                break;
            }	

            if(hdmiIntEn)
            {
                if(!hdmi_int_done)
                {
                    gpio_direction_input(HDMI_INT_PIN_GPIO_NUM);
                    set_irq_type(gpio_to_irq(HDMI_INT_PIN_GPIO_NUM), IRQ_TYPE_EDGE_FALLING);
                    result=request_irq(gpio_to_irq(HDMI_INT_PIN_GPIO_NUM), hdmi_int_irq,
                                                  IRQF_TRIGGER_FALLING|IRQF_DISABLED, HDMI_TRNS_NAME, (void *)NULL);
                    if(result)
                    {
                        printk(KERN_ERR "tda19989 request irq Error : %d\n", result);
                        gpio_free(HDMI_INT_PIN_GPIO_NUM);
                        return -1;
                    }
                    hdmi_int_done=1;
                    hdmi_int_enabled=1;
                }
                else
                {
                    if(!hdmi_int_enabled)
                    {
                        enable_irq(OMAP_GPIO_IRQ(HDMI_INT_PIN_GPIO_NUM));
                        hdmi_int_enabled=1;
                    }
                }
            }
            else
            {
                if(hdmi_int_enabled)
                {
                    disable_irq(OMAP_GPIO_IRQ(HDMI_INT_PIN_GPIO_NUM));
                    hdmi_int_enabled=0;
                }
            }
        }
        break;

#ifdef TDA19989_CEC_AVAILABLE
        case HDMI_CEC_CAL_TIME:
        {
            int i;
            struct timeval prevTime, curTime, resultTime;

            gpio_direction_output(HDMI_INT_PIN_GPIO_NUM, 0);
            gpio_set_value(HDMI_INT_PIN_GPIO_NUM, 0);

            do_gettimeofday(&prevTime);
            mdelay(9);
            for(i=0; i<500; i++)
            {
                do_gettimeofday(&curTime);
                resultTime.tv_usec=curTime.tv_usec-prevTime.tv_usec;
                if(resultTime.tv_usec>9980) break;
                udelay(2);
            }

            gpio_set_value(HDMI_INT_PIN_GPIO_NUM, 1);
            do_gettimeofday(&curTime);

            gpio_direction_output(HDMI_INT_PIN_GPIO_NUM, 1);

            resultTime.tv_usec=curTime.tv_usec-prevTime.tv_usec;
            printk("Time interval: %d\n", (int)resultTime.tv_usec);
        }
        break;
#endif

	 default:
        break;
    }

    return result;
}

struct file_operations tda19989_fops =
{
    .owner = THIS_MODULE,
    .read = tda19989_read,
    .write = tda19989_write,
    .open = tda19989_open,
    .release = tda19989_release,
    .ioctl = tda19989_ioctl,
};

static int i2cTda19989_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    printk("I2cTda19989_Probe \n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    {
        printk("tda19989 Can't support SMBUS \n");
        return -ENODEV;
    }	
    tda19989_client = client;

    return 0;
}

static int i2cTda19989_remove(struct i2c_client *client)
{
    printk("I2cTda19989_Remove \n");
    return 0;
}

static const struct i2c_device_id tda19989_id[] = {
	{ HDMI_TRNS_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, tda19989_id);

static struct i2c_driver i2c_driver_tda19989 = 
{
    .driver =
    {
        .name = HDMI_TRNS_NAME,
        .owner = THIS_MODULE,
    },
    .probe = i2cTda19989_probe,
    .remove = __devexit_p(i2cTda19989_remove),
    .id_table = tda19989_id,
};

static int tda19989_probe(struct platform_device *p_dev)
{
    printk("tda19989_probe \n");
    return 0;
}

static int tda19989_remove(struct platform_device *p_dev)
{
    printk("tda19989_remove  \n");
    return 0;
}

#if defined(CONFIG_PM)
static int tda19989_suspend(struct platform_device* p_dev, pm_message_t event)
{
    printk("tda19989_suspend  \n");
    /*need to check how to control tda19989 power state*/
    return 0;
}

static int tda19989_resume(struct platform_device *p_dev)
{
    printk("tda19989_resume  \n");
    /*need to check how to control tda19989 power state*/
    return 0;
}
#endif

static struct platform_driver tda19989_driver =
{
	.probe		= tda19989_probe,
	.remove		= tda19989_remove,
#if defined(CONFIG_PM)
	.suspend		= tda19989_suspend,
	.resume		= tda19989_resume,
#endif
	.driver		= {
		.name	= HDMI_TRNS_NAME,
		.owner	= THIS_MODULE,	
	},
};

static struct platform_device tda19989_device = {
	.name			= HDMI_TRNS_NAME,
};

static int __init hdmiTda19989_init (void)
{
    int ret;

    ret = i2c_add_driver(&i2c_driver_tda19989);
    if(ret){
        printk("tda19989 : i2c add driver fail : err = %d\n", ret);		
        return ret;
    }

    ret=alloc_chrdev_region(&tda19989_dev_num, 0, 1, HDMI_TRNS_NAME);
    if(ret){
        printk("tda19989 : alloc_chrdev_region failed: err = %d\n", ret);		
        return ret;
    }

    cdev_init(&tda19989_cdev, &tda19989_fops);
    tda19989_cdev.owner = THIS_MODULE;
    ret=cdev_add(&tda19989_cdev, tda19989_dev_num, 1);
    if(ret){
        printk("tda19989 : add cdev failed: err = %d\n", ret);			
        goto exit_err1;
    }
    tda19989_major = MAJOR(tda19989_dev_num);

    ret=platform_driver_register(&tda19989_driver);
    if(ret){
        printk("can't register tda19989_driver driver\n");
        goto exit_err2;
    }
	
    ret=platform_device_register(&tda19989_device);
    if(ret){
        printk("can't register tda19989_driver device\n"); 
        goto exit_err3;
    }

    tda19989_class=class_create(THIS_MODULE, HDMI_TRNS_NAME);
    if(!tda19989_class){
        printk("can't create tda19989 class \n"); 
        goto exit_err4;			
    }

    device_create(tda19989_class, NULL, tda19989_dev_num, NULL, HDMI_TRNS_NAME);

    return 0;

exit_err4:
	platform_device_unregister(&tda19989_device);
exit_err3:
	platform_driver_unregister(&tda19989_driver);
exit_err2:
   cdev_del(&tda19989_cdev);  
exit_err1 :
    unregister_chrdev_region(tda19989_dev_num, 1);

    return ret;

}

static void __exit hdmiTda19989_exit (void)
{
    i2c_del_driver(&i2c_driver_tda19989);
    device_destroy(tda19989_class, tda19989_dev_num);
    class_destroy(tda19989_class);
    platform_device_unregister(&tda19989_device);
    platform_driver_unregister(&tda19989_driver);	
    cdev_del(&tda19989_cdev);	
    unregister_chrdev_region(tda19989_dev_num, 1);
}

module_init(hdmiTda19989_init);
module_exit(hdmiTda19989_exit);

MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
