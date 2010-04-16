/*
 * Copyright (C) 2007-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <asm/div64.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>
#include <linux/time.h>
#include <linux/miscdevice.h>

#define CPCAP_BATT_IRQ_BATTDET 0x01
#define CPCAP_BATT_IRQ_OV      0x02
#define CPCAP_BATT_IRQ_CC_CAL  0x04
#define CPCAP_BATT_IRQ_ADCDONE 0x08
#define CPCAP_BATT_IRQ_MACRO   0x10
#define CPCAP_REGACC_MIN_LREG CPCAP_REG_MDLC
#define CPCAP_REGACC_MAX_LREG CPCAP_REG_CLEDC
#define CPCAP_REGACC_LEN_LREG \
	(CPCAP_REGACC_MAX_LREG - CPCAP_REGACC_MIN_LREG + 1)

#define REGACC_LREG_INDEX(reg) ((reg) - CPCAP_REGACC_MIN_LREG)
#define BATT_BITMASK_FFS(mask) (ffs((mask)) - 1)

static int cpcap_batt_ioctl(struct inode *inode,
			    struct file *file,
			    unsigned int cmd,
			    unsigned long arg);
static unsigned int cpcap_batt_poll(struct file *file, poll_table *wait);
static int cpcap_batt_open(struct inode *inode, struct file *file);
static ssize_t cpcap_batt_read(struct file *file, char *buf, size_t count,
			       loff_t *ppos);
static int cpcap_batt_probe(struct platform_device *pdev);
static int cpcap_batt_remove(struct platform_device *pdev);
static int cpcap_batt_resume(struct platform_device *pdev);

enum CPCAP_CHARGE_CURRENT_T{
  CPCAP_CHARGE_CURRENT_0MA,

  CPCAP_CHARGE_CURRENT_30MA,
  CPCAP_CHARGE_CURRENT_60MA,
  CPCAP_CHARGE_CURRENT_90MA,
  CPCAP_CHARGE_CURRENT__END_TRICKLE = CPCAP_CHARGE_CURRENT_90MA,

  CPCAP_CHARGE_CURRENT_100MA,
  CPCAP_CHARGE_CURRENT_200MA,
  CPCAP_CHARGE_CURRENT_300MA,
  CPCAP_CHARGE_CURRENT_400MA,
  CPCAP_CHARGE_CURRENT_500MA,
  CPCAP_CHARGE_CURRENT_600MA,
  CPCAP_CHARGE_CURRENT_700MA,
  CPCAP_CHARGE_CURRENT_800MA,
  CPCAP_CHARGE_CURRENT_900MA,
  CPCAP_CHARGE_CURRENT_1000MA,
  CPCAP_CHARGE_CURRENT_1100MA,
  CPCAP_CHARGE_CURRENT_1200MA,
  CPCAP_CHARGE_CURRENT_1300MA,
  CPCAP_CHARGE_CURRENT_1800MA,

  CPCAP_CHARGE_CURRENT__END = CPCAP_CHARGE_CURRENT_1800MA
};

enum CPCAP_CHARGE_VOLTAGE_T{
  CPCAP_CHARGE_VOLTAGE_3_8,
  CPCAP_CHARGE_VOLTAGE_4_1,
  CPCAP_CHARGE_VOLTAGE_4_125,
  CPCAP_CHARGE_VOLTAGE_4_15,
  CPCAP_CHARGE_VOLTAGE_4_175,
  CPCAP_CHARGE_VOLTAGE_4_2,
  CPCAP_CHARGE_VOLTAGE_4_225,
  CPCAP_CHARGE_VOLTAGE_4_25,
  CPCAP_CHARGE_VOLTAGE_4_275,
  CPCAP_CHARGE_VOLTAGE_4_3,
  CPCAP_CHARGE_VOLTAGE_4_325,
  CPCAP_CHARGE_VOLTAGE_4_35,
  CPCAP_CHARGE_VOLTAGE_4_375,
  CPCAP_CHARGE_VOLTAGE_4_4,
  CPCAP_CHARGE_VOLTAGE_4_425,
  CPCAP_CHARGE_VOLTAGE_4_45,

  CPCAP_CHARGE_VOLTAGE__END = CPCAP_CHARGE_VOLTAGE_4_45
};

static const unsigned short lighting_bits[CPCAP_REGACC_LEN_LREG] =
{
  0xFFFF, /*!< CPCAP_REG_MDLC */
  0x7FFF, /*!< CPCAP_REG_KLC */
  0x7FFF, /*!< CPCAP_REG_ADLC */
  0x03FF, /*!< CPCAP_REG_REDC */
  0x03FF, /*!< CPCAP_REG_GREENC */
  0x03FF, /*!< CPCAP_REG_BLUEC */
  0x0FFF, /*!< CPCAP_REG_CFC */
  0x003C, /*!< CPCAP_REG_ABC */
  0x03FF, /*!< CPCAP_REG_BLEDC */
  0x03FF  /*!< CPCAP_REG_CLEDC */
};

struct cpcap_batt_ps {
	struct power_supply batt;
	struct power_supply ac;
#ifdef CONFIG_TTA_CHARGER
	struct power_supply tta;
#endif
	struct power_supply usb;
	struct cpcap_device *cpcap;
	struct cpcap_batt_data batt_state;
	struct cpcap_batt_ac_data ac_state;
#ifdef CONFIG_TTA_CHARGER
	struct cpcap_batt_tta_data tta_state;
#endif
	struct cpcap_batt_usb_data usb_state;
	struct cpcap_adc_request req;
	struct mutex lock;
	char irq_status;
	char data_pending;
	wait_queue_head_t wait;
	char async_req_pending;
	unsigned long last_run_time;
};

static const struct file_operations batt_fops = {
	.owner = THIS_MODULE,
	.open = cpcap_batt_open,
	.ioctl = cpcap_batt_ioctl,
	.read = cpcap_batt_read,
	.poll = cpcap_batt_poll,
};

static struct miscdevice batt_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "cpcap_batt",
	.fops	= &batt_fops,
};

static enum power_supply_property cpcap_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP
};

static enum power_supply_property cpcap_batt_ac_props[] =
{
	POWER_SUPPLY_PROP_ONLINE
};

#ifdef CONFIG_TTA_CHARGER
static enum power_supply_property cpcap_batt_tta_props[] =
{
  POWER_SUPPLY_PROP_ONLINE
};
#endif

static enum power_supply_property cpcap_batt_usb_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_MODEL_NAME
};

static struct platform_driver cpcap_batt_driver = {
	.probe		= cpcap_batt_probe,
	.remove		= cpcap_batt_remove,
	.resume		= cpcap_batt_resume,
	.driver		= {
		.name	= "cpcap_battery",
		.owner	= THIS_MODULE,
	},
};

static struct cpcap_batt_ps *cpcap_batt_sply;

static int cpcap_regacc_lighting_write(struct cpcap_device *cpcap,
						enum cpcap_reg reg,
						unsigned short value,
						unsigned short mask)
{
	int ret = -EINVAL;
	if (((reg >= CPCAP_REGACC_MIN_LREG) && (reg <= CPCAP_REGACC_MAX_LREG) &&
	((mask & ~(lighting_bits[REGACC_LREG_INDEX(reg)])) == 0)) ||
	((reg == CPCAP_REG_CRM) && (mask == CPCAP_BIT_CHRG_LED_EN))) {
		ret = cpcap_regacc_write(cpcap, reg, value, mask);
	}

	return ret;
}

int cpcap_batt_set_charge_current(struct cpcap_device *cpcap,
						int charge_current)
{
  int ret;
  unsigned short mask;
  int shift;
  enum CPCAP_CHARGE_CURRENT_T i;
  static unsigned short charge_current_tbl[CPCAP_CHARGE_CURRENT__END + 1] =
  {
    /* CPCAP_CHARGE_CURRENT_0MA    */ 0,
    /* CPCAP_CHARGE_CURRENT_30MA   */ 30,
    /* CPCAP_CHARGE_CURRENT_60MA   */ 60,
    /* CPCAP_CHARGE_CURRENT_90MA   */ 90,
    /* CPCAP_CHARGE_CURRENT_100MA  */ 100,
    /* CPCAP_CHARGE_CURRENT_200MA  */ 200,
    /* CPCAP_CHARGE_CURRENT_300MA  */ 300,
    /* CPCAP_CHARGE_CURRENT_400MA  */ 400,
    /* CPCAP_CHARGE_CURRENT_500MA  */ 500,
    /* CPCAP_CHARGE_CURRENT_600MA  */ 600,
    /* CPCAP_CHARGE_CURRENT_700MA  */ 700,
    /* CPCAP_CHARGE_CURRENT_800MA  */ 800,
    /* CPCAP_CHARGE_CURRENT_900MA  */ 900,
    /* CPCAP_CHARGE_CURRENT_1000MA */ 1000,
    /* CPCAP_CHARGE_CURRENT_1100MA */ 1100,
    /* CPCAP_CHARGE_CURRENT_1200MA */ 1200,
    /* CPCAP_CHARGE_CURRENT_1300MA */ 1300,
    /* CPCAP_CHARGE_CURRENT_1800MA */ 1800
  };

  for (i = CPCAP_CHARGE_CURRENT__END; i > 0; i--) {
	if (charge_current_tbl[i] <= charge_current)
		break;
  }

  mask = (CPCAP_BIT_ICHRG3 |
	CPCAP_BIT_ICHRG2 |
	CPCAP_BIT_ICHRG1 |
	CPCAP_BIT_ICHRG0 |
	CPCAP_BIT_ICHRG_TR1 |
	CPCAP_BIT_ICHRG_TR0);

  if (i <= CPCAP_CHARGE_CURRENT__END_TRICKLE) {
	shift = BATT_BITMASK_FFS(CPCAP_BIT_ICHRG_TR1 |
				CPCAP_BIT_ICHRG_TR0);

    ret = cpcap_regacc_write(cpcap,
			CPCAP_REG_CRM,
			(unsigned short)(i << shift),
			mask);
  } else {
  shift = BATT_BITMASK_FFS(CPCAP_BIT_ICHRG3 |
			CPCAP_BIT_ICHRG2 |
			CPCAP_BIT_ICHRG3 |
			CPCAP_BIT_ICHRG0);

  i -= CPCAP_CHARGE_CURRENT__END_TRICKLE;

  ret = cpcap_regacc_write(cpcap,
			CPCAP_REG_CRM,
			(unsigned short)(i << shift),
			mask);
 }

  ret = cpcap_regacc_lighting_write(cpcap,
				CPCAP_REG_CRM,
				(unsigned short)(i == CPCAP_CHARGE_CURRENT_0MA ?
						0 : CPCAP_BIT_CHRG_LED_EN),
						CPCAP_BIT_CHRG_LED_EN);

  return ret;
}

int cpcap_batt_set_charge_voltage(struct cpcap_device *cpcap,
						int charge_voltage)
{
  int ret;
  unsigned short mask;
  int shift;
  enum CPCAP_CHARGE_VOLTAGE_T i;
  static unsigned short charge_voltage_tbl[CPCAP_CHARGE_VOLTAGE__END + 1] =
  {
    /* CPCAP_CHARGE_VOLTAGE_3_8   */ 3800,
    /* CPCAP_CHARGE_VOLTAGE_4_1   */ 4100,
    /* CPCAP_CHARGE_VOLTAGE_4_125 */ 4125,
    /* CPCAP_CHARGE_VOLTAGE_4_15  */ 4150,
    /* CPCAP_CHARGE_VOLTAGE_4_175 */ 4175,
    /* CPCAP_CHARGE_VOLTAGE_4_2   */ 4200,
    /* CPCAP_CHARGE_VOLTAGE_4_225 */ 4225,
    /* CPCAP_CHARGE_VOLTAGE_4_25  */ 4250,
    /* CPCAP_CHARGE_VOLTAGE_4_275 */ 4275,
    /* CPCAP_CHARGE_VOLTAGE_4_3   */ 4300,
    /* CPCAP_CHARGE_VOLTAGE_4_325 */ 4325,
    /* CPCAP_CHARGE_VOLTAGE_4_35  */ 4350,
    /* CPCAP_CHARGE_VOLTAGE_4_375 */ 4375,
    /* CPCAP_CHARGE_VOLTAGE_4_4   */ 4400,
    /* CPCAP_CHARGE_VOLTAGE_4_425 */ 4425,
    /* CPCAP_CHARGE_VOLTAGE_4_45  */ 4450
    };

    for (i = CPCAP_CHARGE_VOLTAGE__END; i > 0; i--) {
	if (charge_voltage_tbl[i] <= charge_voltage)
		break;
      }
    mask = (CPCAP_BIT_VCHRG3 |
		CPCAP_BIT_VCHRG2 |
		CPCAP_BIT_VCHRG1 |
		CPCAP_BIT_VCHRG0);

    shift = BATT_BITMASK_FFS(mask);

    ret = cpcap_regacc_write(cpcap,
				CPCAP_REG_CRM,
				(unsigned short)(i << shift),
				mask);

  return ret;
}


void cpcap_batt_irq_hdlr(enum cpcap_irqs irq, void *data)
{
	struct cpcap_batt_ps *sply = data;

	mutex_lock(&sply->lock);
	sply->data_pending = 1;

	switch (irq) {
	case CPCAP_IRQ_BATTDETB:
		sply->irq_status |= CPCAP_BATT_IRQ_BATTDET;
		cpcap_irq_unmask(sply->cpcap, irq);
		break;

	case CPCAP_IRQ_VBUSOV:
		sply->irq_status |=  CPCAP_BATT_IRQ_OV;
		cpcap_irq_unmask(sply->cpcap, irq);
		break;

	case CPCAP_IRQ_CC_CAL:
		sply->irq_status |= CPCAP_BATT_IRQ_CC_CAL;
		cpcap_irq_unmask(sply->cpcap, irq);
		break;

	case CPCAP_IRQ_UC_PRIMACRO_7:
	case CPCAP_IRQ_UC_PRIMACRO_8:
	case CPCAP_IRQ_UC_PRIMACRO_9:
	case CPCAP_IRQ_UC_PRIMACRO_10:
	case CPCAP_IRQ_UC_PRIMACRO_11:
		sply->irq_status |= CPCAP_BATT_IRQ_MACRO;
		break;
	default:
		break;
	}

	mutex_unlock(&sply->lock);

	wake_up_interruptible(&sply->wait);
}

void cpcap_batt_adc_hdlr(struct cpcap_device *cpcap, void *data)
{
	struct cpcap_batt_ps *sply = data;
	mutex_lock(&sply->lock);

	sply->async_req_pending = 0;

	sply->data_pending = 1;

	sply->irq_status |= CPCAP_BATT_IRQ_ADCDONE;

	mutex_unlock(&sply->lock);

	wake_up_interruptible(&sply->wait);
}

static int cpcap_batt_open(struct inode *inode, struct file *file)
{
	file->private_data = cpcap_batt_sply;
	return 0;
}

static unsigned int cpcap_batt_poll(struct file *file, poll_table *wait)
{
	struct cpcap_batt_ps *sply = file->private_data;
	unsigned int ret = 0;

	poll_wait(file, &sply->wait, wait);

	if (sply->data_pending)
		ret = (POLLIN | POLLRDNORM);

	return ret;
}

static ssize_t cpcap_batt_read(struct file *file,
			       char *buf, size_t count, loff_t *ppos)
{
	struct cpcap_batt_ps *sply = file->private_data;
	int ret = -EFBIG;
	unsigned long long temp;

	if (count >= sizeof(char)) {
		mutex_lock(&sply->lock);
		if (!copy_to_user((void *)buf, (void *)&sply->irq_status,
				  sizeof(sply->irq_status)))
			ret = sizeof(sply->irq_status);
		else
			ret = -EFAULT;
		sply->data_pending = 0;
		temp = sched_clock();
		do_div(temp, NSEC_PER_SEC);
		sply->last_run_time = (unsigned long)temp;

		sply->irq_status = 0;
		mutex_unlock(&sply->lock);
	}

	return ret;
}

static int cpcap_batt_ioctl(struct inode *inode,
			    struct file *file,
			    unsigned int cmd,
			    unsigned long arg)
{
	int ret = 0;
	int i;
	struct cpcap_batt_ps *sply = file->private_data;
	struct cpcap_adc_request *req_async = &sply->req;
	struct cpcap_adc_request req;
	struct cpcap_adc_us_request req_us;
	struct spi_device *spi = sply->cpcap->spi;
	struct cpcap_platform_data *data = spi->controller_data;

	switch (cmd) {
	case CPCAP_IOCTL_BATT_DISPLAY_UPDATE:
		if (copy_from_user((void *)&sply->batt_state,
				   (void *)arg, sizeof(struct cpcap_batt_data)))
			return -EFAULT;
		power_supply_changed(&sply->batt);

		if (data->batt_changed)
			data->batt_changed(&sply->batt, &sply->batt_state);
		break;

	case CPCAP_IOCTL_BATT_ATOD_ASYNC:
		mutex_lock(&sply->lock);
		if (!sply->async_req_pending) {
			if (copy_from_user((void *)&req_us, (void *)arg,
					   sizeof(struct cpcap_adc_us_request)
					   )) {
				mutex_unlock(&sply->lock);
				return -EFAULT;
			}

			req_async->format = req_us.format;
			req_async->timing = req_us.timing;
			req_async->type = req_us.type;
			req_async->callback = cpcap_batt_adc_hdlr;
			req_async->callback_param = sply;

			ret = cpcap_adc_async_read(sply->cpcap, req_async);
			if (!ret)
				sply->async_req_pending = 1;
		} else {
			ret = -EAGAIN;
		}
		mutex_unlock(&sply->lock);

		break;

	case CPCAP_IOCTL_BATT_ATOD_SYNC:
		if (copy_from_user((void *)&req_us, (void *)arg,
				   sizeof(struct cpcap_adc_us_request)))
			return -EFAULT;

		req.format = req_us.format;
		req.timing = req_us.timing;
		req.type = req_us.type;

		ret = cpcap_adc_sync_read(sply->cpcap, &req);

		if (ret)
			return ret;

		req_us.status = req.status;
		for (i = 0; i < CPCAP_ADC_BANK0_NUM; i++)
			req_us.result[i] = req.result[i];

		if (copy_to_user((void *)arg, (void *)&req_us,
				 sizeof(struct cpcap_adc_us_request)))
			return -EFAULT;
		break;

	case CPCAP_IOCTL_BATT_ATOD_READ:
		req_us.format = req_async->format;
		req_us.timing = req_async->timing;
		req_us.type = req_async->type;
		req_us.status = req_async->status;
		for (i = 0; i < CPCAP_ADC_BANK0_NUM; i++)
			req_us.result[i] = req_async->result[i];

		if (copy_to_user((void *)arg, (void *)&req_us,
				 sizeof(struct cpcap_adc_us_request)))
			return -EFAULT;
		break;

  case CPCAP_IOCTL_BATT_CHARGER_SET_CHARGE_CURRENT:
  {
    unsigned short chgr_current = 0;
    ret = copy_from_user((void *)&chgr_current, (void *)arg,
						sizeof(chgr_current));

	if (ret != 0)
		ret = -EFAULT;
	else
		ret = cpcap_batt_set_charge_current(sply->cpcap, chgr_current);
	break;
  }

  case CPCAP_IOCTL_BATT_CHARGER_SET_CHARGE_VOLTAGE:
  {
    unsigned short chgr_voltage = 0;
    ret = copy_from_user((void *)&chgr_voltage, (void *)arg,
						sizeof(chgr_voltage));

	if (ret != 0)
		ret = -EFAULT;
	else
		ret = cpcap_batt_set_charge_voltage(sply->cpcap, chgr_voltage);
	break;
  }

	default:
		return -ENOTTY;
		break;
	}

	return ret;
}

#ifdef CONFIG_TTA_CHARGER
static int cpcap_batt_tta_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct cpcap_batt_ps *sply =
		container_of(psy, struct cpcap_batt_ps,	tta);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sply->tta_state.online;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif

static int cpcap_batt_ac_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	int ret = 0;
	struct cpcap_batt_ps *sply = container_of(psy, struct cpcap_batt_ps,
						 ac);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sply->ac_state.online;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static char *cpcap_batt_usb_models[] = {
	"none", "usb", "factory"
};

static int cpcap_batt_usb_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	int ret = 0;
	struct cpcap_batt_ps *sply = container_of(psy, struct cpcap_batt_ps,
						 usb);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sply->usb_state.online;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = sply->usb_state.current_now;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = cpcap_batt_usb_models[sply->usb_state.model];
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int cpcap_batt_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	int ret = 0;
	struct cpcap_batt_ps *sply = container_of(psy, struct cpcap_batt_ps,
						  batt);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = sply->batt_state.status;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = sply->batt_state.health;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = sply->batt_state.present;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = sply->batt_state.capacity;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = sply->batt_state.batt_volt;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = sply->batt_state.batt_temp;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int cpcap_batt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct cpcap_batt_ps *sply;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		ret = -EINVAL;
		goto prb_exit;
	}

	sply = kzalloc(sizeof(struct cpcap_batt_ps), GFP_KERNEL);
	if (sply == NULL) {
		ret = -ENOMEM;
		goto prb_exit;
	}

	sply->cpcap = pdev->dev.platform_data;
	mutex_init(&sply->lock);
	init_waitqueue_head(&sply->wait);

	sply->batt_state.status	= POWER_SUPPLY_STATUS_UNKNOWN;
	sply->batt_state.health	= POWER_SUPPLY_HEALTH_GOOD;
	sply->batt_state.present = 1;
	sply->batt_state.capacity = 100;	/* Percentage */
	sply->batt_state.batt_volt = 4200000;	/* uV */
	sply->batt_state.batt_temp = 230;	/* tenths of degrees Celsius */

	sply->ac_state.online = 0;

	sply->usb_state.online = 0;
	sply->usb_state.current_now = 0;
	sply->usb_state.model = CPCAP_BATT_USB_MODEL_NONE;

	sply->batt.properties = cpcap_batt_props;
	sply->batt.num_properties = ARRAY_SIZE(cpcap_batt_props);
	sply->batt.get_property = cpcap_batt_get_property;
	sply->batt.name = "battery";
	sply->batt.type = POWER_SUPPLY_TYPE_BATTERY;

	sply->ac.properties = cpcap_batt_ac_props;
	sply->ac.num_properties = ARRAY_SIZE(cpcap_batt_ac_props);
	sply->ac.get_property = cpcap_batt_ac_get_property;
	sply->ac.name = "ac";
	sply->ac.type = POWER_SUPPLY_TYPE_MAINS;

#ifdef CONFIG_TTA_CHARGER
	sply->tta.properties = cpcap_batt_tta_props;
	sply->tta.num_properties = ARRAY_SIZE(cpcap_batt_tta_props);
	sply->tta.get_property = cpcap_batt_tta_get_property;
	sply->tta.name = "tta";
	sply->tta.type = POWER_SUPPLY_TYPE_TTA;
#endif

	sply->usb.properties = cpcap_batt_usb_props;
	sply->usb.num_properties = ARRAY_SIZE(cpcap_batt_usb_props);
	sply->usb.get_property = cpcap_batt_usb_get_property;
	sply->usb.name = "usb";
	sply->usb.type = POWER_SUPPLY_TYPE_USB;

	ret = power_supply_register(&pdev->dev, &sply->ac);
	if (ret)
		goto prb_exit;
	ret = power_supply_register(&pdev->dev, &sply->batt);
	if (ret)
		goto unregac_exit;
	ret = power_supply_register(&pdev->dev, &sply->usb);
	if (ret)
		goto unregbatt_exit;
#ifdef CONFIG_TTA_CHARGER
	ret = power_supply_register(&pdev->dev, &sply->tta);
	if (ret)
		goto unregusb_exit;
#endif
	platform_set_drvdata(pdev, sply);
	sply->cpcap->battdata = sply;
	cpcap_batt_sply = sply;

	ret = misc_register(&batt_dev);
	if (ret)
		goto unregusb_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_VBUSOV,
				 cpcap_batt_irq_hdlr, sply);
	if (ret)
		goto unregmisc_exit;
	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_BATTDETB,
				 cpcap_batt_irq_hdlr, sply);
	if (ret)
		goto unregirq_exit;
	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_CC_CAL,
				 cpcap_batt_irq_hdlr, sply);
	if (ret)
		goto unregirq_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_7,
				 cpcap_batt_irq_hdlr, sply);
	cpcap_irq_mask(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_7);

	if (ret)
		goto unregirq_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_8,
				 cpcap_batt_irq_hdlr, sply);
	cpcap_irq_mask(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_8);

	if (ret)
		goto unregirq_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_9,
				 cpcap_batt_irq_hdlr, sply);
	cpcap_irq_mask(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_9);

	if (ret)
		goto unregirq_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_10,
				 cpcap_batt_irq_hdlr, sply);
	cpcap_irq_mask(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_10);

	if (ret)
		goto unregirq_exit;

	ret = cpcap_irq_register(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_11,
				 cpcap_batt_irq_hdlr, sply);
	cpcap_irq_mask(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_11);

	if (ret)
		goto unregirq_exit;

	goto prb_exit;

unregirq_exit:
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_VBUSOV);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_BATTDETB);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_CC_CAL);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_7);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_8);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_9);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_10);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_11);
unregmisc_exit:
	misc_deregister(&batt_dev);
unregusb_exit:
	power_supply_unregister(&sply->usb);
unregbatt_exit:
	power_supply_unregister(&sply->batt);
unregac_exit:
	power_supply_unregister(&sply->ac);

prb_exit:
	return ret;
}

static int cpcap_batt_remove(struct platform_device *pdev)
{
	struct cpcap_batt_ps *sply = platform_get_drvdata(pdev);

	power_supply_unregister(&sply->batt);
	power_supply_unregister(&sply->ac);
	power_supply_unregister(&sply->usb);
#ifdef CONFIG_TTA_CHARGER
	power_supply_unregister(&sply->tta);
#endif
	misc_deregister(&batt_dev);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_VBUSOV);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_BATTDETB);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_CC_CAL);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_7);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_8);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_9);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_10);
	cpcap_irq_free(sply->cpcap, CPCAP_IRQ_UC_PRIMACRO_11);
	sply->cpcap->battdata = NULL;
	kfree(sply);

	return 0;
}

static int cpcap_batt_resume(struct platform_device *pdev)
{
	struct cpcap_batt_ps *sply = platform_get_drvdata(pdev);
	unsigned long cur_time;
	unsigned long long temp;

	temp = sched_clock();
	do_div(temp, NSEC_PER_SEC);
	cur_time = ((unsigned long)temp);
	if ((cur_time - sply->last_run_time) < 0)
		sply->last_run_time = 0;

	if ((cur_time - sply->last_run_time) > 50) {
		mutex_lock(&sply->lock);
		sply->data_pending = 1;
		sply->irq_status |= CPCAP_BATT_IRQ_MACRO;

		mutex_unlock(&sply->lock);

		wake_up_interruptible(&sply->wait);
	}

	return 0;
}

#ifdef CONFIG_TTA_CHARGER
void cpcap_batt_set_tta_prop(struct cpcap_device *cpcap, int online)
{
	struct cpcap_batt_ps *sply = cpcap->battdata;
	struct spi_device *spi = cpcap->spi;
	struct cpcap_platform_data *data = spi->controller_data;

	if (sply != NULL) {
		sply->tta_state.online = online;
		power_supply_changed(&sply->tta);

		if (data->tta_changed)
			data->tta_changed(&sply->tta, &sply->tta_state);
	}
}
EXPORT_SYMBOL(cpcap_batt_set_tta_prop);
#endif

void cpcap_batt_set_ac_prop(struct cpcap_device *cpcap, int online)
{
	struct cpcap_batt_ps *sply = cpcap->battdata;
	struct spi_device *spi = cpcap->spi;
	struct cpcap_platform_data *data = spi->controller_data;

	if (sply != NULL) {
		sply->ac_state.online = online;
		power_supply_changed(&sply->ac);

		if (data->ac_changed)
			data->ac_changed(&sply->ac, &sply->ac_state);
	}
}
EXPORT_SYMBOL(cpcap_batt_set_ac_prop);

void cpcap_batt_set_usb_prop_online(struct cpcap_device *cpcap, int online,
				    enum cpcap_batt_usb_model model)
{
	struct cpcap_batt_ps *sply = cpcap->battdata;
	struct spi_device *spi = cpcap->spi;
	struct cpcap_platform_data *data = spi->controller_data;

	if (sply != NULL) {
		sply->usb_state.online = online;
		sply->usb_state.model = model;
		power_supply_changed(&sply->usb);

		if (data->usb_changed)
			data->usb_changed(&sply->usb, &sply->usb_state);
	}
}
EXPORT_SYMBOL(cpcap_batt_set_usb_prop_online);

void cpcap_batt_set_usb_prop_curr(struct cpcap_device *cpcap, unsigned int curr)
{
	struct cpcap_batt_ps *sply = cpcap->battdata;
	struct spi_device *spi = cpcap->spi;
	struct cpcap_platform_data *data = spi->controller_data;

	if (sply != NULL) {
		sply->usb_state.current_now = curr;
		power_supply_changed(&sply->usb);

		if (data->usb_changed)
			data->usb_changed(&sply->usb, &sply->usb_state);
	}
}
EXPORT_SYMBOL(cpcap_batt_set_usb_prop_curr);

static int __init cpcap_batt_init(void)
{
	return platform_driver_register(&cpcap_batt_driver);
}
subsys_initcall(cpcap_batt_init);

static void __exit cpcap_batt_exit(void)
{
	platform_driver_unregister(&cpcap_batt_driver);
}
module_exit(cpcap_batt_exit);

MODULE_ALIAS("platform:cpcap_batt");
MODULE_DESCRIPTION("CPCAP BATTERY driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
