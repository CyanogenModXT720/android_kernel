/*
 * drivers/i2c/chips/leds-bd7885.c
 *
 * I2C slave driver for BD7885 device
 *
 * Copyright (C) 2008-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Revision History:
 *
 * Date          Author    Comment
 * -----------   --------  -------------------
 * Jun-24-2008   Motorola  Initial version
 * 
 */

#include <linux/kernel.h>	/* printk() */
#include <linux/fs.h>		/* everything... */
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include <asm/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>

#include <linux/leds-bd7885.h>

#if defined(CONFIG_LEDS_BU9847)
#include <linux/leds-bu9847.h>
#endif

static int bd7885_open(struct inode *inode, struct file *file);
static int bd7885_release(struct inode *inode, struct file *file);
static int bd7885_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

/*Below two external definition is for strobe manual control.*/
extern int ov8810_strobe_manual_ready(void);
extern int ov8810_strobe_manual_trigger(void);

#if defined(CONFIG_LEDS_BU9847)
int bu9847_fetch_regs(void);
#endif

struct bd7885_chip {
	struct mutex lock;
	struct i2c_client *client;
	int status;               /* flag indicate chip current power state */
};

struct file_operations bd7885_fops = {
	.owner   = THIS_MODULE,
	.open    = bd7885_open,
	.release = bd7885_release,
	.ioctl   = bd7885_ioctl,
};

static struct miscdevice bd7885_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = BD7885_DEVICE_NAME,
  .fops = &bd7885_fops,
};

const static bd7885_cfg bd7885_charege_enalbe_tbl[4] = {
	{BD7885_DRVCNT_REG, 0x03},
	{BD7885_RECHG_REG, 0x11},
	{BD7885_IPEAKADJ_REG, 0x07},
	/* must end invalid register */
	{BD7885_REG_TERM, BD7885_VAL_TERM}
};

const static bd7885_cfg bd7885_charge_disable_tbl[2] = {
       {BD7885_DRVCNT_REG, 0x00},
	/* must end invalid register */
	{BD7885_REG_TERM, BD7885_VAL_TERM}
};

/*for register value keep here.*/
static char bd7885_reg_status_tbl[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

/* for ioctl */
struct i2c_client *bd7885_i2c_client;

/* device major number */
int bd7885_major_num = 0;
int bd7885_minor_num = 0;

#if defined(CONFIG_LEDS_BU9847)
/*for register value keep here.*/
extern char bu9847_reg_info_tbl[18];
#endif

#define BD7885_I2C_RETRY_MAX 3

/* *****************************************************

	Low level interface routine.

* *****************************************************/
static int bd7885_reg_read(struct i2c_client *client, u8 reg_addr, u8 *buf)
{
    /** \var error
     *  \brief Return error code, initialized to no error
     */
    int error = E_OK;
    /** \var i
     *  \brief Retry counter
     */
    int i = 0;
    unsigned char value[BD7885_REG_DATA_LEN];

    /* If number of bytes is out of range or buffer is incorrect, report, exit */
    if ((buf == NULL) || (reg_addr > BD7885_STATUS_REG)) {
	printk(KERN_ERR "invalid read parameter(s)\n");
	return -EINVAL;
    }
    /* If I2C client doesn't exist */
    if (client == NULL) {
	printk(KERN_ERR "bd7885 null i2c read client\n");
	return -EUNATCH;
    }

    /* Do read */
    do {
	/* Establish read address
	*  i2c_master_send returns either the number of bytes sent or negative error from
	*  i2c_transfer
	*/
	/* Copy the data into a buffer for correct format */
	value[0] = reg_addr & 0xFF;
	error = i2c_master_send(client, value, BD7885_REG_DATA_LEN);
	/* Read data if send address succeeded (i2c_master_send returns the number
	*  of bytes transferred) */
	if (error == BD7885_REG_DATA_LEN) {
		/* Retreive data
		*  i2c_master_recv returns either the number of bytes received or negative error
		*  from i2c_transfer
		*/
		error = i2c_master_recv(client, buf, BD7885_REG_DATA_LEN);
	} else {
		printk(KERN_ERR "bd7885 write of address %d failed: %d\n", value[0], error);
	}

	/* On failure (error will contain num_bytes if succeeded), print error code and
	* delay before trying again */
	if (error != BD7885_REG_DATA_LEN) {
		printk(KERN_ERR "bd7885 read[%i] failed: %d\n", i, error);
		msleep(10);
	}
    /* Do while an error exists and we haven't exceeded retry attempts */
    } while ((error != BD7885_REG_DATA_LEN) && ((++i) < BD7885_I2C_RETRY_MAX));



    /* On success, set error to E_OK (i2c_master_recv returns the number of bytes
     *  transferred) */
    if (error == BD7885_REG_DATA_LEN) {
	error = E_OK;
    }

    return error;
}

static int bd7885_reg_write(struct i2c_client *client, u8 reg_addr, u8 reg_value)
{
    unsigned char value[BD7885_REG_ADDR_DATA_LEN];
    int retval = 0;
    int i =  BD7885_I2C_RETRY_MAX;
    
    /* Fail if the length is too small */
    if ((reg_addr == BD7885_HW_ID_REG) || (reg_addr == BD7885_STATUS_REG) \
		|| (reg_addr > BD7885_STATUS_REG)) {
	printk(KERN_ERR "bd7885_reg_write: length is invalid\n");
	return -EINVAL;
    }

    /* Fail if we weren't able to initialize (yet) */
    if (client == NULL) {
	printk(KERN_ERR "bd7885_reg_write: initialization failed\n");
	return -EINVAL;
    }

    /* Copy the data into a buffer for correct format */
    value[0] = reg_addr & 0xFF;
    memcpy(&value[1], &reg_value, BD7885_REG_DATA_LEN);

    /* Write the data to the device (retrying if necessary) */
    do {
	retval = i2c_master_send(client, value, BD7885_REG_ADDR_DATA_LEN);

	/* On failure, output the error code and delay before trying again */
	if (retval < 0) {
		printk(KERN_ERR "bd7885 write 0x%X failed: %d\n",\
				value[0], retval);
		msleep(5);
	}
    } while ((retval < 0) && (i-- >= 0));

    /* On success, set retval to 0 (i2c_master_send returns no. of bytes transfered) */
    if (retval == BD7885_REG_ADDR_DATA_LEN) {
	retval = 0;
	/*Store write value to global status table.*/
	bd7885_reg_status_tbl[reg_addr] = reg_value;
    }

    /* Delay after every I2C access or IC will NAK */
    msleep(5);

    return retval;
}

static int bd7885_reg_writes(struct i2c_client *client, const bd7885_cfg *reglist)
{
	int err = 0;
	const bd7885_cfg *next = reglist;

	while (!((next->reg == BD7885_REG_TERM)
		&& (next->data == BD7885_VAL_TERM))) {
		err = bd7885_reg_write(client, next->reg, next->data);
		udelay(100);
		if (err)
			return err;
		next++;
	}
	return 0;
}

/*
It shoudl be called just one time.
Because it include gpio request. If it is call many time, it makes error condition.
*/
static int bd7885_io_init(void)
{
    return 0;
}

#if defined(CONFIG_LEDS_BU9847)
static int bd7885_quench_level_set(struct i2c_client *client,
    unsigned char level)
{
    int ret = 0;

    /*Quench level is from 1 to 12 if using bu9847.*/
    if ((level < 1) || (level > 12))
		return -1;

    ret = bd7885_reg_write(client, BD7885_QUENCHADJ_REG,
    bu9847_reg_info_tbl[BU9847_QCHC1_REG + level - 1]);

    if (ret != 0) {
		printk(KERN_ERR "bd7885_quench_level_set failed\n");
		return -1;
    }

    return 0;
}
#endif/*CONFIG_LEDS_BU9847*/

static int bd7885_charge_enable_sequence(struct i2c_client *client)
{
    printk(KERN_INFO "%s is called.\n", __FUNCTION__);

    /*Register value write down for charge enable.*/
    if (bd7885_reg_writes(client, bd7885_charege_enalbe_tbl) != 0) {
	printk(KERN_ERR "bd7885_reg_writes failed\n");
	return -EFAULT;
    }

    return 0;
}

int bd7885_charge_disable_sequence(struct i2c_client *client)
{
    printk(KERN_INFO "%s is called.\n", __FUNCTION__);

    /*Register value write down for charge disable.*/
    if (bd7885_reg_writes(client, bd7885_charge_disable_tbl) != 0) {
	printk(KERN_ERR "bd7885_reg_writes failed\n");
	return -EFAULT;
    }

    return 0;
}

static int bd7885_charge_enable(struct i2c_client *client)
{

    if (bd7885_charge_enable_sequence(client) != 0) {
	printk(KERN_ERR "%s: bd7885_reg_writes failed\n", __FUNCTION__);
	return -EFAULT;
    }

    return 0;
}

#if defined(CONFIG_LEDS_FLASH_RESET)
bool bd7885_device_detection(void)
{
	u8 reg_val = 0;
	int ret = 0;

	/*Driver ID fetch and de*/
	ret = bd7885_reg_read(bd7885_i2c_client, BD7885_HW_ID_REG, &reg_val);

	if (ret != 0) {
		printk(KERN_ERR "Xenon flash HW ID read fail\n");
		return false;
	}

	if (reg_val == BD7885_HW_ID_VAL)
		return true;

	return false;
}
#endif

/* *****************************************************

	Probe, Remove

* ******************************************************/
static int bd7885_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bd7885_chip *chip;
	int ret = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE))  {
		ret  = -ENODEV;
		goto exit_check_functionality_failed;
	}

       bd7885_i2c_client = client;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)	{
		return -ENOMEM;
	}

	printk(KERN_INFO "BD7885 driver: bd7885_probe()\n");

	chip->client = client;
	strncpy(client->name, BD7885_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, chip);

       ret = misc_register(&bd7885_device);
	if (ret != 0)
		printk(KERN_INFO "misc_register failed\n");

	mutex_init(&chip->lock);

	printk(KERN_INFO "BD7885 chip probe is finished.\n");

	return ret;

exit_check_functionality_failed:
	return ret;
}

static int bd7885_remove(struct i2c_client *client)
{
	struct bd7885_chip *chip = i2c_get_clientdata(client);

	printk(KERN_INFO "bd7885_remove() is called!\n");
	kfree(chip);

	return 0;
}

static const struct i2c_device_id bd7885_id[] = {
	{ BD7885_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bd7885_id);

static struct i2c_driver bd7885_driver = {
	.driver = {
		.name = BD7885_DRIVER_NAME,
	},
	.probe	  = bd7885_probe,
	.remove	  = __devexit_p(bd7885_remove),
	.id_table = bd7885_id,
};

static int bd7885_open(struct inode *inode, struct file *file)
{
	unsigned char reg_cnt = 0;

	for (reg_cnt = 0; reg_cnt <= BD7885_STATUS_REG; reg_cnt++) {
		if (bd7885_reg_read(bd7885_i2c_client, reg_cnt,\
					&bd7885_reg_status_tbl[reg_cnt]) != 0) {
			printk(KERN_ERR "bd7885_read failed. addr = %d\n",\
					reg_cnt);
			return -EFAULT;
		}
	}

	printk(KERN_INFO "%s is called.\n",\
					__func__);
	return nonseekable_open(inode, file);
}

static int bd7885_release(struct inode *inode, struct file *file)
{
  printk(KERN_INFO "%s is called.\n", __FUNCTION__);
  return 0;
}

static int bd7885_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    unsigned char rwbuf, data1, data2;
    bd7885_cfg rw_reg_buf;

    int error = E_OK;

    /* get information from user */
    switch (cmd) {
    case BD7885_IOCTL_GET_REGISTER:
    case BD7885_IOCTL_SET_REGISTER:
      if (copy_from_user(&rw_reg_buf, argp, sizeof(rw_reg_buf)))
	return -EFAULT;
	break;

    case BD7885_IOCTL_SET_CHARGING:
    case BD7885_IOCTL_SET_CHARGE_LEVEL:
    case BD7885_IOCTL_SET_MODE:
    case BD7885_IOCTL_SET_QUENCH_THRESHOLD:
    case BD7885_IOCTL_GET_STATUS:
    case BD7885_IOCTL_READY_STROBE_MANUAL:
    case BD7885_IOCTL_FIRE_STROBE_MANUAL:
      if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
		return -EFAULT;
      break;

    default:
      printk(KERN_ERR "Abnormal value input = %d.\n", cmd);
      break;
    }

    printk(KERN_ERR "BD7785_cmd = %d.\n", cmd);

    /* interact with driver */
    switch (cmd) {
    case BD7885_IOCTL_GET_REGISTER:
	printk(KERN_ERR "BD7785_cmd = BD7885_IOCTL_GET_REGISTER is called.\n");
       error = bd7885_reg_read(bd7885_i2c_client, rw_reg_buf.reg,\
							&rw_reg_buf.data);
	if (error != 0) {
		printk(KERN_ERR "bd7885_read failed. addr = %d\n",\
						rw_reg_buf.reg);
		return error;
	}
      break;

    case BD7885_IOCTL_SET_REGISTER:
	printk(KERN_ERR "BD7785_cmd = BD7885_IOCTL_SET_REGISTER is called.\n");

	error = bd7885_reg_write(bd7885_i2c_client, rw_reg_buf.reg,\
							rw_reg_buf.data);
	if (error != 0) {
		printk(KERN_ERR "bd7885_read failed. addr = %d\n",\
						rw_reg_buf.reg);
		return error;
	}
	break;

    case BD7885_IOCTL_SET_CHARGING:
	printk(KERN_ERR "BD7785_cmd = BD7885_IOCTL_SET_CHARGING is called.\n");

	if (rwbuf == BD7885_CHARGE_ENABLE) {
		error = bd7885_charge_enable(bd7885_i2c_client);
		if (error != 0) {
			printk(KERN_ERR "BD7885 cmd = %d\n", cmd);
			return error;
		}

	} else if (rwbuf == BD7885_CHARGE_DISABLE) {
		error = bd7885_charge_disable_sequence(bd7885_i2c_client);
		if (error != 0) {
			printk(KERN_ERR "BD7885 cmd = %d\n", cmd);
			return error;
		}
	} else {
		printk(KERN_ERR "BD7885 charging status abnormal.\n");
		return -EFAULT;
	}
	break;

    case BD7885_IOCTL_SET_CHARGE_LEVEL:
	printk(KERN_ERR "BD7785_cmd = BD7885_IOCTL_SET_CHARGE_LEVEL is called.\n");

	rwbuf &= BD7885_FULL_ADJ_LVL_MASK;
       /*Apply FULLADJ setting.*/
       error = bd7885_reg_write(bd7885_i2c_client, BD7885_FULLADJ_REG, rwbuf);
	if (error != 0) {
		printk(KERN_ERR "BD7885 cmd = %d write failure.\n", cmd);
		return error;
	}
	break;

    case BD7885_IOCTL_GET_STATUS:
	printk(KERN_ERR "BD7785_cmd = BD7885_IOCTL_GET_STATUS is called.\n");

       error = bd7885_reg_read(bd7885_i2c_client, BD7885_STATUS_REG, &rwbuf);
	if (error != 0) {
		printk(KERN_ERR "BD7885 cmd = %d write failure.\n", cmd);
		return error;
	}
	break;

    case BD7885_IOCTL_SET_MODE:
	printk(KERN_ERR "BD7785_cmd = BD7885_IOCTL_SET_MODE is called.\n");

       /*Apply Mode setting.*/
       if (rwbuf == BD7885_STROBE_QUENCH_MODE) {
		/* Set QUENCH_EN, Set PCNT_EN */
		data1 = bd7885_reg_status_tbl[BD7885_QUENCHCNT_REG] | \
				BD7885_QUENCH_EN_MASK;
		data2 = bd7885_reg_status_tbl[BD7885_DRVCNT_REG] | \
				BD7885_PCNT_EN_MASK;
       }
      else if (rwbuf == BD7885_STROBE_MANUAL_MODE) {
		/* Clr QUENCH_EN, Set PCNT_EN */
		data1 = bd7885_reg_status_tbl[BD7885_QUENCHCNT_REG] & \
				(~BD7885_QUENCH_EN_MASK);
		data2 = bd7885_reg_status_tbl[BD7885_DRVCNT_REG] | \
				BD7885_PCNT_EN_MASK;
      }
       else {
		/* Clr QUENCH_EN & PCNT_EN */
		data1 = bd7885_reg_status_tbl[BD7885_QUENCHCNT_REG] & \
				(~BD7885_QUENCH_EN_MASK);
		data2 = bd7885_reg_status_tbl[BD7885_DRVCNT_REG] & \
				(~BD7885_PCNT_EN_MASK);
       }

       error = bd7885_reg_write(bd7885_i2c_client, BD7885_QUENCHCNT_REG, data1);
       error |= bd7885_reg_write(bd7885_i2c_client, BD7885_DRVCNT_REG, data2);
	if (error != 0) {
		printk(KERN_ERR "BD7885 cmd = %d write failure.\n", cmd);
		return error;
	}
       break;

    case BD7885_IOCTL_SET_QUENCH_THRESHOLD:
	printk(KERN_ERR "BD7785_cmd = BD7885_IOCTL_SET_QUENCH_THRESHOLD is called.\n");

       /*Apply VSTOPADJ & PTR value to Xenon flash.*/
#if defined(CONFIG_LEDS_BU9847)
	   /*Quench level fetch from EEPROM.*/
		   error = bu9847_fetch_regs();
		   if (error != 0)
				return error;

       /*Quench level fetch from EEPROM.*/
       if (rwbuf != 0) {
		/*Enable PCNT_EN*/
		data1 = bd7885_reg_status_tbl[BD7885_DRVCNT_REG] | \
				BD7885_PCNT_EN_MASK;
		error = bd7885_reg_write(bd7885_i2c_client, \
				BD7885_DRVCNT_REG, data1);
		error |= bd7885_quench_level_set(bd7885_i2c_client, rwbuf);
       } else {
		/*Disable PCNT_EN*/
		data1 = bd7885_reg_status_tbl[BD7885_DRVCNT_REG] & \
				(~BD7885_PCNT_EN_MASK);
		error = bd7885_reg_write(bd7885_i2c_client, \
				BD7885_DRVCNT_REG, data1);
       }
	if (error != 0) {
		printk(KERN_ERR "BD7885 cmd = %d write failure.\n", cmd);
		return error;
	}
#endif
	break;

    case BD7885_IOCTL_READY_STROBE_MANUAL:
		ov8810_strobe_manual_ready();
	break;

    case BD7885_IOCTL_FIRE_STROBE_MANUAL:
		ov8810_strobe_manual_trigger();
	break;

    default:
      return -ENOTTY;
    }

    /* return information to user */
    switch (cmd) {
    case BD7885_IOCTL_GET_REGISTER:
	printk(KERN_ERR "BD7785_cmd = BD7885_IOCTL_GET_REGISTER copy to user.\n");

      if (copy_to_user(argp, &rw_reg_buf, sizeof(rw_reg_buf)))
	return -EFAULT;
      break;

    case BD7885_IOCTL_GET_STATUS:
	printk(KERN_ERR "BD7785_cmd = BD7885_IOCTL_GET_STATUS copy to user.\n");
	printk(KERN_ERR "status value = %d\n", rwbuf);

      if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
	return -EFAULT;
      break;

    default:
      break;
    }

    return error;
}

/*
 *  BD7885 voltage regulator init
 */
static int __init bd7885_init(void)
{
    int ret = -EFAULT;

    ret = i2c_add_driver(&bd7885_driver);
    if (ret != 0) {
	printk(KERN_INFO "BD7885 init failed.\n");
    }
    printk(KERN_INFO "BD7885 i2c driver : init is completed.\n");

    /*Base IO initialize.*/
    bd7885_io_init();

    return ret;
}

static void __exit bd7885_exit(void)
{
	i2c_del_driver(&bd7885_driver);

	printk(KERN_INFO "BD7885  flash driver: exit\n");
}

module_init(bd7885_init);
module_exit(bd7885_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("BD7885 flash regulator driver");
MODULE_LICENSE("GPL");

