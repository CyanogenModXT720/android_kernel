/*
 * Copyright (C) 2007 - 2009 Motorola, Inc.
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

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include <linux/regulator/consumer.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>

#ifdef CONFIG_TTA_CHARGER
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>

#define SHOLEST_TTA_CHRG_DET_N_GPIO  34
#define TIME_FOR_GPIO_HIGH           10
#define TTA_IRQ_NAME "tta_IRQ"
#endif

#define SENSE_USB           (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_USB_FLASH     (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_FACTORY       (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_CHARGER_FLOAT (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_SE1_S)

#define SENSE_CHARGER       (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_SE1_S)

enum cpcap_det_state {
	CONFIG,
	SAMPLE_1,
	SAMPLE_2,
	IDENTIFY,
	USB,
	FACTORY,
	CHARGER,
#ifdef CONFIG_TTA_CHARGER
	IDENTIFY_TTA,
	TTA,
#endif
};

enum cpcap_accy {
	CPCAP_ACCY_USB,
	CPCAP_ACCY_FACTORY,
	CPCAP_ACCY_CHARGER,
#ifdef CONFIG_TTA_CHARGER
	CPCAP_ACCY_TTA_CHARGER,
#endif
	CPCAP_ACCY_NONE,

	/* Used while debouncing the accessory. */
	CPCAP_ACCY_UNKNOWN,
};

#ifdef CONFIG_TTA_CHARGER
struct tta_sense_data {
	unsigned short dplus:1;
	unsigned short dminus:1;
	unsigned short gpio_val:1;
};
#endif

struct cpcap_usb_det_data {
	struct cpcap_device *cpcap;
	struct delayed_work work;
	unsigned short sense;
	unsigned short prev_sense;
	enum cpcap_det_state state;
	enum cpcap_accy usb_accy;
	struct platform_device *usb_dev;
	struct platform_device *usb_connected_dev;
	struct platform_device *charger_connected_dev;
	struct regulator *regulator;
	struct wake_lock wake_lock;
	unsigned char is_vusb_enabled;
#ifdef CONFIG_TTA_CHARGER
	struct tta_sense_data sense_tta;
#endif
};

static const char *accy_devices[] = {
	"cpcap_usb_charger",
	"cpcap_factory",
	"cpcap_charger",
#ifdef CONFIG_TTA_CHARGER	
	"cpcap_tta_charger",
#endif	
};

#ifdef CONFIG_TTA_CHARGER
static struct cpcap_usb_det_data *temp_data;
#endif

static void vusb_enable(struct cpcap_usb_det_data *data)
{
	if (!data->is_vusb_enabled) {
		wake_lock(&data->wake_lock);
		regulator_enable(data->regulator);
		data->is_vusb_enabled = 1;
	}
}

static void vusb_disable(struct cpcap_usb_det_data *data)
{
	if (data->is_vusb_enabled) {
		wake_unlock(&data->wake_lock);
		regulator_disable(data->regulator);
		data->is_vusb_enabled = 0;
	}
}

#ifdef CONFIG_TTA_CHARGER
void enable_tta(void)
{
	mdelay(TIME_FOR_GPIO_HIGH);
	gpio_direction_input(SHOLEST_TTA_CHRG_DET_N_GPIO);
}
EXPORT_SYMBOL(enable_tta);

void disable_tta(void)
{
	gpio_direction_output(SHOLEST_TTA_CHRG_DET_N_GPIO, 1);
}
EXPORT_SYMBOL(disable_tta);

void force_to_detect_tta(unsigned int time)
{
	schedule_delayed_work(&temp_data->work, msecs_to_jiffies(time));
}
EXPORT_SYMBOL(force_to_detect_tta);

unsigned char value_of_gpio34(void)
{
	return gpio_get_value(SHOLEST_TTA_CHRG_DET_N_GPIO);
}
EXPORT_SYMBOL(value_of_gpio34);

void disable_tta_irq(void)
{
	disable_irq(gpio_to_irq(SHOLEST_TTA_CHRG_DET_N_GPIO));
}
EXPORT_SYMBOL(disable_tta_irq);

unsigned char is_emu_accessory(void)
{
	if ((temp_data->usb_accy == CPCAP_ACCY_NONE) ||
	    (temp_data->usb_accy == CPCAP_ACCY_TTA_CHARGER))
		return 1;
	else
		return 0;  
}
EXPORT_SYMBOL(is_emu_accessory);

void force_to_detect_usb(void)
{
	unsigned char sense = 0;
	unsigned short value;
	cpcap_regacc_read(temp_data->cpcap, CPCAP_REG_INTS2, &value);
  
	/* Clear ASAP after read. */
	cpcap_regacc_write(temp_data->cpcap, CPCAP_REG_INT2,
			   (CPCAP_BIT_CHRGCURR1_I |
			    CPCAP_BIT_VBUSVLD_I |
			    CPCAP_BIT_SESSVLD_I |
			    CPCAP_BIT_SE1_I),
			   (CPCAP_BIT_CHRGCURR1_I |
			    CPCAP_BIT_VBUSVLD_I |
			    CPCAP_BIT_SESSVLD_I |
			    CPCAP_BIT_SE1_I));

	sense = ((value & CPCAP_BIT_VBUSVLD_S) ? 1 : 0);

	if (!sense)
		schedule_delayed_work(&temp_data->work, msecs_to_jiffies(0));
}
EXPORT_SYMBOL(force_to_detect_usb);

#endif
static int get_sense(struct cpcap_usb_det_data *data)
{
	int retval = -EFAULT;
	unsigned short value;
	struct cpcap_device *cpcap;

	if (!data)
		return -EFAULT;
	cpcap = data->cpcap;

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS1, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT1,
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_GROUND_I),
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_GROUND_I));
	if (retval)
		return retval;

	data->sense = value & (CPCAP_BIT_ID_FLOAT_S |
			       CPCAP_BIT_ID_GROUND_S);

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS2, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT2,
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_VBUSVLD_I |
				     CPCAP_BIT_SESSVLD_I |
				     CPCAP_BIT_SE1_I),
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_VBUSVLD_I |
				     CPCAP_BIT_SESSVLD_I |
				     CPCAP_BIT_SE1_I));
	if (retval)
		return retval;

	data->sense |= value & (CPCAP_BIT_CHRGCURR1_S |
				CPCAP_BIT_VBUSVLD_S |
				CPCAP_BIT_SESSVLD_S |
				CPCAP_BIT_SE1_S);

#ifdef CONFIG_TTA_CHARGER
	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS4, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT1,
					(CPCAP_BIT_DP_I |
					CPCAP_BIT_DM_I),
					(CPCAP_BIT_DP_I |
					CPCAP_BIT_DM_I));

	if (retval)
		return retval;

	data->sense_tta.dplus = ((value & CPCAP_BIT_DP_S) ? 1 : 0);
	data->sense_tta.dminus = ((value & CPCAP_BIT_DM_S) ? 1 : 0);
	data->sense_tta.gpio_val = gpio_get_value(SHOLEST_TTA_CHRG_DET_N_GPIO);
#endif

	return 0;
}

static int configure_hardware(struct cpcap_usb_det_data *data,
			      enum cpcap_accy accy)
{
	int retval;

	/* Take control of pull up from ULPI. */
	retval  = cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
				     CPCAP_BIT_PU_SPI,
				     CPCAP_BIT_PU_SPI);
	retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
				    CPCAP_BIT_DP150KPU,
				    (CPCAP_BIT_DP150KPU | CPCAP_BIT_DP1K5PU |
				     CPCAP_BIT_DM1K5PU | CPCAP_BIT_DPPD |
				     CPCAP_BIT_DMPD));
#ifdef CONFIG_TTA_CHARGER
	get_sense(data);
	if (!(data->sense_tta.dplus)) {
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
						    CPCAP_BIT_PU_SPI |
						    CPCAP_BIT_DMPD_SPI |
						    CPCAP_BIT_DPPD_SPI,
						    CPCAP_BIT_PU_SPI |
						    CPCAP_BIT_DMPD_SPI |
						    CPCAP_BIT_DPPD_SPI);

		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
				CPCAP_BIT_DP150KPU,
				(CPCAP_BIT_DP150KPU | CPCAP_BIT_DP1K5PU |
				CPCAP_BIT_DM1K5PU | CPCAP_BIT_DPPD |
				CPCAP_BIT_DMPD));
	}
#endif

	switch (accy) {
	case CPCAP_ACCY_USB:
	case CPCAP_ACCY_FACTORY:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					     CPCAP_BIT_USBXCVREN,
					     CPCAP_BIT_USBXCVREN);
		/* Give USB driver control of pull up via ULPI. */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     0,
					     CPCAP_BIT_PU_SPI |
					     CPCAP_BIT_DMPD_SPI |
					     CPCAP_BIT_DPPD_SPI);
		break;

	case CPCAP_ACCY_CHARGER:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		break;

#ifdef CONFIG_TTA_CHARGER
	case CPCAP_ACCY_TTA_CHARGER:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
						   CPCAP_BIT_PU_SPI |
						   CPCAP_BIT_DMPD_SPI |
						   CPCAP_BIT_DPPD_SPI,
						   CPCAP_BIT_PU_SPI |
						   CPCAP_BIT_DMPD_SPI |
						   CPCAP_BIT_DPPD_SPI);

		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					     CPCAP_BIT_USBXCVREN,
					     CPCAP_BIT_USBXCVREN);
    break;
#endif

	case CPCAP_ACCY_UNKNOWN:
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		break;

	case CPCAP_ACCY_NONE:
	default:
#ifdef CONFIG_TTA_CHARGER   
        retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     CPCAP_BIT_PU_SPI,
					     CPCAP_BIT_PU_SPI |
					     CPCAP_BIT_DMPD_SPI |
					     CPCAP_BIT_DPPD_SPI);
#endif
        retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2, 0,
					     CPCAP_BIT_USBXCVREN);
		break;
	}

	if (retval != 0)
		retval = -EFAULT;

	return retval;
}

static void notify_accy(struct cpcap_usb_det_data *data, enum cpcap_accy accy)
{
	dev_info(&data->cpcap->spi->dev, "notify_accy: accy=%d\n", accy);

	if ((data->usb_accy != CPCAP_ACCY_NONE) && (data->usb_dev != NULL)) {
		platform_device_del(data->usb_dev);
		data->usb_dev = NULL;
	}

	configure_hardware(data, accy);
	data->usb_accy = accy;

	if (accy != CPCAP_ACCY_NONE) {
		data->usb_dev = platform_device_alloc(accy_devices[accy], -1);
		if (data->usb_dev) {
			data->usb_dev->dev.platform_data = data->cpcap;
			platform_device_add(data->usb_dev);
		}
	} else
		vusb_disable(data);

	if ((accy == CPCAP_ACCY_USB) || (accy == CPCAP_ACCY_FACTORY)) {
		if (!data->usb_connected_dev) {
			data->usb_connected_dev =
			    platform_device_alloc("cpcap_usb_connected", -1);
			platform_device_add(data->usb_connected_dev);
		}
	} else if (data->usb_connected_dev) {
		platform_device_del(data->usb_connected_dev);
		data->usb_connected_dev = NULL;
	}

	if (accy == CPCAP_ACCY_CHARGER) {
		if (!data->charger_connected_dev) {
			data->charger_connected_dev =
			    platform_device_alloc("cpcap_charger_connected",
						  -1);
			platform_device_add(data->charger_connected_dev);
		}
	} else if (data->charger_connected_dev) {
		platform_device_del(data->charger_connected_dev);
		data->charger_connected_dev = NULL;
	}
}

static void detection_work(struct work_struct *work)
{
	struct cpcap_usb_det_data *data =
		container_of(work, struct cpcap_usb_det_data, work.work);
#ifdef CONFIG_TTA_CHARGER  
	static unsigned char first_time = 0;
#endif

	switch (data->state) {
	case CONFIG:
		vusb_enable(data);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);
#ifdef CONFIG_TTA_CHARGER
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DMI);
#endif 
		configure_hardware(data, CPCAP_ACCY_UNKNOWN);

		data->state = SAMPLE_1;
		schedule_delayed_work(&data->work, msecs_to_jiffies(11));
		break;

	case SAMPLE_1:
		get_sense(data);
#ifdef CONFIG_TTA_CHARGER
		if (!(data->sense_tta.gpio_val) &&
			(data->sense & CPCAP_BIT_SESSVLD_S)) {
			disable_tta();
			enable_tta();
		}
#endif    
		data->state = SAMPLE_2;
		schedule_delayed_work(&data->work, msecs_to_jiffies(100));
		break;

	case SAMPLE_2:
		data->prev_sense = data->sense;
		get_sense(data);

		if (data->prev_sense != data->sense) {
			/* Stay in this state */
			data->state = SAMPLE_2;
			schedule_delayed_work(&data->work,
					      msecs_to_jiffies(100));
		} else if (!(data->sense & CPCAP_BIT_SE1_S) &&
			   (data->sense & CPCAP_BIT_ID_FLOAT_S) &&
			   !(data->sense & CPCAP_BIT_ID_GROUND_S) &&
			   !(data->sense & CPCAP_BIT_SESSVLD_S)) {
			data->state = IDENTIFY;
#ifdef CONFIG_TTA_CHARGER
			if (!(data->sense_tta.gpio_val))
				data->state = IDENTIFY_TTA;
#endif
			schedule_delayed_work(&data->work,
					      msecs_to_jiffies(100));
		} else {
			data->state = IDENTIFY;
#ifdef CONFIG_TTA_CHARGER
			if (!(data->sense & CPCAP_BIT_SESSVLD_S) &&
				!(data->sense_tta.gpio_val)) {
				data->state = IDENTIFY_TTA;
		}
#endif
			schedule_delayed_work(&data->work, 0);
		}
		break;
#ifdef CONFIG_TTA_CHARGER
	case IDENTIFY_TTA:
		configure_hardware(data, CPCAP_ACCY_TTA_CHARGER);
		data->state = IDENTIFY;
		schedule_delayed_work(&data->work, 0);
		break;
#endif
	case IDENTIFY:
		get_sense(data);
		data->state = CONFIG;

#ifdef CONFIG_TTA_CHARGER
		if ((data->sense_tta.dplus == data->sense_tta.dminus) &&
			!(data->sense_tta.gpio_val) &&
			!(data->sense & CPCAP_BIT_SESSVLD_S)) {
			notify_accy(data, CPCAP_ACCY_TTA_CHARGER);
			cpcap_irq_clear(data->cpcap, CPCAP_IRQ_DMI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DMI);
			data->state = TTA;
			disable_musb_int();

		} else if ((data->sense == SENSE_USB) ||
			(data->sense == SENSE_USB_FLASH)) {
			notify_accy(data, CPCAP_ACCY_USB);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

			/* Special handling of USB cable undetect. */
			data->state = USB;
		}
#else
		if ((data->sense == SENSE_USB) ||
		    (data->sense == SENSE_USB_FLASH)) {
			notify_accy(data, CPCAP_ACCY_USB);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

			/* Special handling of USB cable undetect. */
			data->state = USB;
		}
#endif
		else if (data->sense == SENSE_FACTORY) {
			notify_accy(data, CPCAP_ACCY_FACTORY);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
#ifdef CONFIG_TTA_CHARGER
			disable_tta();
#endif
			/* Special handling of factory cable undetect. */
			data->state = FACTORY;
		} else if ((data->sense == SENSE_CHARGER_FLOAT) ||
			   (data->sense == SENSE_CHARGER)) {
			notify_accy(data, CPCAP_ACCY_CHARGER);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

			/* Special handling of charger undetect. */
			data->state = CHARGER;
		} else {
			notify_accy(data, CPCAP_ACCY_NONE);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);

			/* When a charger is unpowered by unplugging from the
			 * wall, VBUS voltage will drop below CHRG_DET (3.5V)
			 * until the ICHRG bits are cleared.  Once ICHRG is
			 * cleared, VBUS will rise above CHRG_DET, but below
			 * VBUSVLD (4.4V) briefly as it decays.  If the charger
			 * is re-powered while VBUS is within this window, the
			 * VBUSVLD interrupt is needed to trigger charger
			 * detection.
			 *
			 * VBUSVLD must be masked before going into suspend.
			 * See cpcap_usb_det_suspend() for details.
			 */
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);
#ifdef CONFIG_TTA_CHARGER      
			disable_tta();
			enable_tta();
#endif      
		}
		break;
#ifdef CONFIG_TTA_CHARGER
	case TTA:
		get_sense(data);

		if ((data->sense_tta.dplus != data->sense_tta.dminus) ||
			(data->sense_tta.gpio_val)) {
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DMI);
			disable_tta();
			enable_tta();
			data->state = CONFIG;
			enable_musb_int();
			schedule_delayed_work(&data->work, 0);
		} else {
			data->state = TTA;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DMI);
		}
		break;
#endif
	case USB:
		get_sense(data);

		if ((data->sense & CPCAP_BIT_SE1_S) ||
		    (data->sense & CPCAP_BIT_ID_GROUND_S) ||
		    (!(data->sense & CPCAP_BIT_VBUSVLD_S))) {
			if (data->sense & CPCAP_BIT_SE1_S) {
				/* A partially inserted charger is now fully
				 * seated in the jack. Give SPI control of
				 * PullUp/down bits so SE1 can not be lost. */
				cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
						   CPCAP_BIT_PU_SPI |
						   CPCAP_BIT_DMPD_SPI |
						   CPCAP_BIT_DPPD_SPI,
						   CPCAP_BIT_PU_SPI |
						   CPCAP_BIT_DMPD_SPI |
						   CPCAP_BIT_DPPD_SPI);
			}

			data->state = CONFIG;
			schedule_delayed_work(&data->work, 0);
		} else {
			data->state = USB;

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case FACTORY:
		get_sense(data);

		/* The removal of a factory cable can only be detected if a
		 * charger is attached.
		 */
		if (data->sense & CPCAP_BIT_SE1_S) {
#ifdef CONFIG_TTA_CHARGER
			enable_tta();
#endif
			data->state = CONFIG;
			schedule_delayed_work(&data->work, 0);
		} else {
			data->state = FACTORY;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
		}
		break;

	case CHARGER:
		get_sense(data);

		/* If the battery voltage is above the set charge voltage in
		 * CPCAP and ICHRG is set, CHRGCURR1 will be 0.  Do not undetect
		 * charger in this case. */
		if (!(data->sense & CPCAP_BIT_SE1_S) ||
		    (!(data->sense & CPCAP_BIT_VBUSVLD_S) &&
		     !(data->sense & CPCAP_BIT_CHRGCURR1_S))) {
			data->state = CONFIG;
			schedule_delayed_work(&data->work, 0);
		} else {
			data->state = CHARGER;

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	default:
		/* This shouldn't happen.  Need to reset state machine. */
		vusb_disable(data);
		data->state = CONFIG;
		schedule_delayed_work(&data->work, 0);
		break;
	}
#ifdef CONFIG_TTA_CHARGER
	temp_data = data;
	if (!first_time) {
		enable_musb_int();
		first_time = 1;
	}
#endif
}

#ifdef CONFIG_TTA_CHARGER
irqreturn_t isr_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	struct cpcap_usb_det_data *data;
	data = (struct cpcap_usb_det_data *) dev_id;
	schedule_delayed_work(&data->work, msecs_to_jiffies(0));
	return IRQ_HANDLED;
}
#endif

static void int_handler(enum cpcap_irqs int_event, void *data)
{
	struct cpcap_usb_det_data *usb_det_data = data;
	schedule_delayed_work(&(usb_det_data->work), 0);
}

static int __init cpcap_usb_det_probe(struct platform_device *pdev)
{
	int retval;
	struct cpcap_usb_det_data *data;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cpcap = pdev->dev.platform_data;
	data->state = CONFIG;
	platform_set_drvdata(pdev, data);
	INIT_DELAYED_WORK(&data->work, detection_work);
	data->usb_accy = CPCAP_ACCY_NONE;
	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "usb");

	data->regulator = regulator_get(&pdev->dev, "vusb");
	if (IS_ERR(data->regulator)) {
		dev_err(&pdev->dev, "Could not get regulator for cpcap_usb\n");
		retval = PTR_ERR(data->regulator);
		goto free_mem;
	}
	regulator_set_voltage(data->regulator, 3300000, 3300000);

	retval = cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_DET,
				    int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_CURR1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_SE1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_IDGND,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_VBUSVLD,
				     int_handler, data);

#ifdef CONFIG_TTA_CHARGER
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_DMI,
			int_handler, data);

	if (gpio_request(SHOLEST_TTA_CHRG_DET_N_GPIO, "tta_chrg_cntr") < 0)
		return -EBUSY;

	set_irq_type(OMAP_GPIO_IRQ(SHOLEST_TTA_CHRG_DET_N_GPIO),
			IRQ_TYPE_EDGE_FALLING);
	retval |= request_irq(
			OMAP_GPIO_IRQ(SHOLEST_TTA_CHRG_DET_N_GPIO),
			(void *)isr_handler,
			IRQF_DISABLED, TTA_IRQ_NAME,
			data);
#endif

	/* Now that HW initialization is done, give USB control via ULPI. */
	retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
				     0, CPCAP_BIT_ULPI_SPI_SEL);

	if (retval != 0) {
		dev_err(&pdev->dev, "Initialization Error\n");
		retval = -ENODEV;
		goto free_irqs;
	}

	dev_info(&pdev->dev, "CPCAP USB detection device probed\n");

	/* Perform initial detection */
	detection_work(&(data->work.work));

	return 0;

free_irqs:
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_VBUSVLD);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDGND);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SE1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_DET);
#ifdef CONFIG_TTA_CHARGER
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DMI);
	free_irq(OMAP_GPIO_IRQ(SHOLEST_TTA_CHRG_DET_N_GPIO), 0);
	gpio_free(SHOLEST_TTA_CHRG_DET_N_GPIO);
#endif
	regulator_put(data->regulator);
free_mem:
	wake_lock_destroy(&data->wake_lock);
	kfree(data);

	return retval;
}

static int __exit cpcap_usb_det_remove(struct platform_device *pdev)
{
	struct cpcap_usb_det_data *data = platform_get_drvdata(pdev);

	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_DET);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SE1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDGND);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_VBUSVLD);
#ifdef CONFIG_TTA_CHARGER
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DMI);
	free_irq(OMAP_GPIO_IRQ(SHOLEST_TTA_CHRG_DET_N_GPIO), 0);
	gpio_free(SHOLEST_TTA_CHRG_DET_N_GPIO);
#endif

	configure_hardware(data, CPCAP_ACCY_NONE);
	cancel_delayed_work_sync(&data->work);

	if ((data->usb_accy != CPCAP_ACCY_NONE) && (data->usb_dev != NULL))
		platform_device_del(data->usb_dev);

	vusb_disable(data);
	regulator_put(data->regulator);

	wake_lock_destroy(&data->wake_lock);

	kfree(data);
	return 0;
}

#ifdef CONFIG_PM
static int cpcap_usb_det_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct cpcap_usb_det_data *data = platform_get_drvdata(pdev);

	/* VBUSVLD cannot be unmasked when entering suspend. If left
	 * unmasked, a false interrupt will be received, keeping the
	 * device out of suspend. The interrupt does not need to be
	 * unmasked when resuming from suspend since the use case
	 * for having the interrupt unmasked is over.
	 */
	cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);

	return 0;
}
#else
#define cpcap_usb_det_suspend NULL
#endif

static struct platform_driver cpcap_usb_det_driver = {
	.probe		= cpcap_usb_det_probe,
	.remove		= __exit_p(cpcap_usb_det_remove),
	.suspend	= cpcap_usb_det_suspend,
	.driver		= {
		.name	= "cpcap_usb_det",
		.owner	= THIS_MODULE,
	},
};

static int __init cpcap_usb_det_init(void)
{
	return platform_driver_register(&cpcap_usb_det_driver);
}
/* The CPCAP USB detection driver must be started later to give the MUSB
 * driver time to complete its initialization. */
late_initcall(cpcap_usb_det_init);

static void __exit cpcap_usb_det_exit(void)
{
	platform_driver_unregister(&cpcap_usb_det_driver);
}
module_exit(cpcap_usb_det_exit);

MODULE_ALIAS("platform:cpcap_usb_det");
MODULE_DESCRIPTION("CPCAP USB detection driver");
MODULE_LICENSE("GPL");
