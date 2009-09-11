/*
 * Copyright (C) 2009 Motorola, Inc.
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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/switch.h>

#include <linux/regulator/consumer.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>

enum {
	NO_DEVICE,
	HEADSET_WITH_MIC,
	HEADSET_WITHOUT_MIC,
};

enum analog_switch_path {
	CPCAP_GPIO_SWITCH_OFF = 0,
	CPCAP_GPIO_SWITCH_HSMIC,
	CPCAP_GPIO_SWITCH_ADC,
	CPCAP_GPIO_SWITCH_TVOUT
};

struct cpcap_3mm5_data {
	struct cpcap_device *cpcap;
	struct switch_dev sdev;
	unsigned int key_state;
	struct regulator *regulator;
	unsigned char audio_low_power;
};

static ssize_t print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case HEADSET_WITH_MIC:
		return sprintf(buf, "Headset with mic\n");
	case HEADSET_WITHOUT_MIC:
		return sprintf(buf, "Headset without mic\n");
	}

	return -EINVAL;
}

static void audio_low_power_set(struct cpcap_3mm5_data *data)
{
	if (!data->audio_low_power) {
		regulator_set_mode(data->regulator, REGULATOR_MODE_STANDBY);
		data->audio_low_power = 1;
	}
}

static void audio_low_power_clear(struct cpcap_3mm5_data *data)
{
	if (data->audio_low_power) {
		regulator_set_mode(data->regulator, REGULATOR_MODE_NORMAL);
		data->audio_low_power = 0;
	}
}

static void send_key_event(struct cpcap_3mm5_data *data, unsigned int state)
{
	dev_info(&data->cpcap->spi->dev, "Headset key event: old=%d, new=%d\n",
		 data->key_state, state);

	if (data->key_state != state) {
		data->key_state = state;
		cpcap_broadcast_key_event(data->cpcap, KEY_MEDIA, state);
	}
}

static int init_analog_switch(struct cpcap_3mm5_data *data)
{
	int cpcap_status = 0;
	struct cpcap_3mm5_data *data_3mm5 = data;

	/* set vlev = 2.775V for GPIO 2 */
	cpcap_status = cpcap_regacc_write(data_3mm5->cpcap,
					CPCAP_REG_GPIO2, CPCAP_BIT_GPIO2VLEV,
					CPCAP_BIT_GPIO2VLEV);
	if (cpcap_status < 0) {
		pr_err("Cpcap : %s: "
				"Configuring GPIO2 VLEV failed: \n", __func__);
		return cpcap_status;
	}

	/* set vlev = 2.775V for GPIO 4 */
	cpcap_status = cpcap_regacc_write(data_3mm5->cpcap,
					CPCAP_REG_GPIO4, CPCAP_BIT_GPIO4VLEV,
					CPCAP_BIT_GPIO4VLEV);
	if (cpcap_status < 0) {
		pr_err("Cpcap : %s: "
				"Configuring GPIO4 VLEV failed: \n", __func__);
		return cpcap_status;
	}

	cpcap_status = cpcap_regacc_write(data_3mm5->cpcap,
					CPCAP_REG_GPIO2, CPCAP_BIT_GPIO2DIR,
					CPCAP_BIT_GPIO2DIR);
	if (cpcap_status < 0) {
		pr_err("Cpcap : %s: "
				"Configuring GPIO2 failed: \n", __func__);
		return cpcap_status;
	}
	cpcap_status = cpcap_regacc_write(data_3mm5->cpcap,
					CPCAP_REG_GPIO4, CPCAP_BIT_GPIO4DIR,
					CPCAP_BIT_GPIO4DIR);

	if (cpcap_status < 0) {
		pr_err("Cpcap : %s: "
				"Configuring GPIO4 failed: \n", __func__);
		return cpcap_status;
	}
	return 0;
}

static int control_analog_switch(
		enum analog_switch_path path,
		struct cpcap_3mm5_data *data
)
{
	int cpcap_status_gpio_2 = 0;
	int cpcap_status_gpio_4 = 0;
	struct cpcap_3mm5_data *data_3mm5 = data;

	switch (path) {
	case CPCAP_GPIO_SWITCH_OFF:
		cpcap_status_gpio_2 = cpcap_regacc_write(data_3mm5->cpcap,
			CPCAP_REG_GPIO2, 0,
			CPCAP_BIT_GPIO2DRV);

		cpcap_status_gpio_4 = cpcap_regacc_write(data_3mm5->cpcap,
			CPCAP_REG_GPIO4, 0,
			CPCAP_BIT_GPIO4DRV);
		break;

	case CPCAP_GPIO_SWITCH_HSMIC:
		cpcap_status_gpio_2 = cpcap_regacc_write(data_3mm5->cpcap,
			CPCAP_REG_GPIO2, 0,
			CPCAP_BIT_GPIO2DRV);

		cpcap_status_gpio_4 = cpcap_regacc_write(data_3mm5->cpcap,
			CPCAP_REG_GPIO4, CPCAP_BIT_GPIO4DRV,
			CPCAP_BIT_GPIO4DRV);
		break;

	case CPCAP_GPIO_SWITCH_ADC:
		cpcap_status_gpio_2 = cpcap_regacc_write(data_3mm5->cpcap,
			CPCAP_REG_GPIO2, CPCAP_BIT_GPIO2DRV,
			CPCAP_BIT_GPIO2DRV);

		cpcap_status_gpio_4 = cpcap_regacc_write(data_3mm5->cpcap,
			CPCAP_REG_GPIO4, 0,
			CPCAP_BIT_GPIO4DRV);
		break;

	case CPCAP_GPIO_SWITCH_TVOUT:
		cpcap_status_gpio_2 = cpcap_regacc_write(data_3mm5->cpcap,
			CPCAP_REG_GPIO2, CPCAP_BIT_GPIO2DRV,
			CPCAP_BIT_GPIO2DRV);

		cpcap_status_gpio_4 = cpcap_regacc_write(data_3mm5->cpcap,
			CPCAP_REG_GPIO4, CPCAP_BIT_GPIO4DRV,
			CPCAP_BIT_GPIO4DRV);
		break;

	default:
		break;
	}

	if ((cpcap_status_gpio_2 < 0) || (cpcap_status_gpio_4 < 0)) {
		pr_err("Cpcap TV_out: %s: "
				"Control Analog Switch failed: \n", __func__);
		return -1;
	}
	return 0;
}

static void hs_handler_tv_out(enum cpcap_irqs irq, void *data)
{
	struct cpcap_3mm5_data *data_3mm5 = data;
	int new_state = NO_DEVICE;
	int control_val = 0;

	if (irq != CPCAP_IRQ_HS)
		return;

	/* HS sense of 1 means no headset present, 0 means headset attached. */
	if (cpcap_irq_sense(data_3mm5->cpcap, CPCAP_IRQ_HS, 1) == 1) {
		cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_TXI, 0,
				   (CPCAP_BIT_MB_ON2 | CPCAP_BIT_PTT_CMP_EN));
		cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_RXOA, 0,
				   CPCAP_BIT_ST_HS_CP_EN);
		audio_low_power_set(data_3mm5);

		cpcap_irq_mask(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_mask(data_3mm5->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);

		cpcap_irq_clear(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_clear(data_3mm5->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);

		cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_HS);

		send_key_event(data_3mm5, 0);

		cpcap_uc_stop(data_3mm5->cpcap, CPCAP_MACRO_5);

		control_val = control_analog_switch(CPCAP_GPIO_SWITCH_OFF,
							data_3mm5);

	} else {
		control_val = control_analog_switch(CPCAP_GPIO_SWITCH_HSMIC,
							data_3mm5);

		if (control_val < 0)
			return;

		cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_TXI,
				   (CPCAP_BIT_MB_ON2 | CPCAP_BIT_PTT_CMP_EN),
				   (CPCAP_BIT_MB_ON2 | CPCAP_BIT_PTT_CMP_EN));
		cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_RXOA,
				   CPCAP_BIT_ST_HS_CP_EN,
				   CPCAP_BIT_ST_HS_CP_EN);
		audio_low_power_clear(data_3mm5);

		/* Give PTTS time to settle */
		mdelay(2);

		if (cpcap_irq_sense(data_3mm5->cpcap, CPCAP_IRQ_PTT, 1) <= 0) {
			/* Headset without mic and MFB is detected. (May also
			 * be a headset with the MFB pressed.) */
			new_state = HEADSET_WITHOUT_MIC;
		} else if (cpcap_irq_sense(data_3mm5->cpcap,
					CPCAP_IRQ_MB2, 0) == 0)
			new_state = HEADSET_WITHOUT_MIC;
		else
			new_state = HEADSET_WITH_MIC;

		cpcap_irq_clear(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_clear(data_3mm5->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);

		cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_HS);
		cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);

		cpcap_uc_start(data_3mm5->cpcap, CPCAP_MACRO_5);
		cpcap_uc_start(data_3mm5->cpcap, CPCAP_MACRO_4);
	}

	switch_set_state(&data_3mm5->sdev, new_state);
	dev_info(&data_3mm5->cpcap->spi->dev, "New headset state: %d\n",
		 new_state);
}

static void hs_handler(enum cpcap_irqs irq, void *data)
{
	struct cpcap_3mm5_data *data_3mm5 = data;
	int new_state = NO_DEVICE;

	if (irq != CPCAP_IRQ_HS)
		return;

	/* HS sense of 1 means no headset present, 0 means headset attached. */
	if (cpcap_irq_sense(data_3mm5->cpcap, CPCAP_IRQ_HS, 1) == 1) {
		cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_TXI, 0,
				   (CPCAP_BIT_MB_ON2 | CPCAP_BIT_PTT_CMP_EN));
		cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_RXOA, 0,
				   CPCAP_BIT_ST_HS_CP_EN);
		audio_low_power_set(data_3mm5);

		cpcap_irq_mask(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_mask(data_3mm5->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);

		cpcap_irq_clear(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_clear(data_3mm5->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);

		cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_HS);

		send_key_event(data_3mm5, 0);

		cpcap_uc_stop(data_3mm5->cpcap, CPCAP_MACRO_5);
	} else {
		cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_TXI,
				   (CPCAP_BIT_MB_ON2 | CPCAP_BIT_PTT_CMP_EN),
				   (CPCAP_BIT_MB_ON2 | CPCAP_BIT_PTT_CMP_EN));
		cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_RXOA,
				   CPCAP_BIT_ST_HS_CP_EN,
				   CPCAP_BIT_ST_HS_CP_EN);
		audio_low_power_clear(data_3mm5);

		/* Give PTTS time to settle */
		mdelay(2);

		if (cpcap_irq_sense(data_3mm5->cpcap, CPCAP_IRQ_PTT, 1) <= 0) {
			/* Headset without mic and MFB is detected. (May also
			 * be a headset with the MFB pressed.) */
			new_state = HEADSET_WITHOUT_MIC;
		} else
			new_state = HEADSET_WITH_MIC;

		cpcap_irq_clear(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_clear(data_3mm5->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);

		cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_HS);
		cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);

		cpcap_uc_start(data_3mm5->cpcap, CPCAP_MACRO_5);
		cpcap_uc_start(data_3mm5->cpcap, CPCAP_MACRO_4);
	}

	switch_set_state(&data_3mm5->sdev, new_state);
	dev_info(&data_3mm5->cpcap->spi->dev, "New headset state: %d\n",
		 new_state);
}

static void key_handler(enum cpcap_irqs irq, void *data)
{
	struct cpcap_3mm5_data *data_3mm5 = data;
	struct cpcap_platform_data *platform_data = NULL;
	int barrel_capability;
	int read_hs = 0;

	platform_data = data_3mm5->cpcap->spi->controller_data;
	barrel_capability = platform_data->barrel_capability;

	if ((irq != CPCAP_IRQ_MB2) && (irq != CPCAP_IRQ_UC_PRIMACRO_5))
		return;

	read_hs = cpcap_irq_sense(data_3mm5->cpcap, CPCAP_IRQ_HS, 1);
	if ((barrel_capability == BARREL_CAP_DETECT_TV_OUT) &&
		((read_hs == 1) ||
		(switch_get_state(&data_3mm5->sdev) != HEADSET_WITH_MIC))) {
		hs_handler_tv_out(CPCAP_IRQ_HS, data_3mm5);
		return;
	} else if ((barrel_capability != BARREL_CAP_DETECT_TV_OUT) &&
		((read_hs == 1) ||
		(switch_get_state(&data_3mm5->sdev) != HEADSET_WITH_MIC))) {
		hs_handler(CPCAP_IRQ_HS, data_3mm5);
		return;
	}

	if ((cpcap_irq_sense(data_3mm5->cpcap, CPCAP_IRQ_MB2, 0) == 0) ||
		(cpcap_irq_sense(data_3mm5->cpcap, CPCAP_IRQ_PTT, 0) == 0)) {
		send_key_event(data_3mm5, 1);

		/* If macro not available, only short presses are supported */
		if (!cpcap_uc_status(data_3mm5->cpcap, CPCAP_MACRO_5)) {
			send_key_event(data_3mm5, 0);

			/* Attempt to restart the macro for next time. */
			cpcap_uc_start(data_3mm5->cpcap, CPCAP_MACRO_5);
			cpcap_uc_start(data_3mm5->cpcap, CPCAP_MACRO_4);
		}
	} else
		send_key_event(data_3mm5, 0);

	cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_MB2);
	cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);
}

static int __init cpcap_3mm5_probe(struct platform_device *pdev)
{
	struct cpcap_3mm5_data *data;
	struct cpcap_platform_data *platform_data = NULL;
	int barrel_capability;
	int retval = 0;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cpcap = pdev->dev.platform_data;
	data->audio_low_power = 1;
	data->sdev.name = "h2w";
	data->sdev.print_name = print_name;
	switch_dev_register(&data->sdev);
	platform_set_drvdata(pdev, data);

	platform_data = data->cpcap->spi->controller_data;
	barrel_capability = platform_data->barrel_capability;

	data->regulator = regulator_get(NULL, "vaudio");
	if (IS_ERR(data->regulator)) {
		dev_err(&pdev->dev, "Could not get regulator for cpcap_3mm5\n");
		retval = PTR_ERR(data->regulator);
		goto free_mem;
	}

	regulator_set_voltage(data->regulator, 2775000, 2775000);

	if (barrel_capability == BARREL_CAP_DETECT_TV_OUT) {
		retval = init_analog_switch(data);
		if (retval < 0)
			return retval;
	}

	retval  = cpcap_irq_clear(data->cpcap, CPCAP_IRQ_HS);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_MB2);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);
	if (retval)
		goto reg_put;

	if (barrel_capability == BARREL_CAP_DETECT_TV_OUT) {
		retval = cpcap_irq_register(data->cpcap, CPCAP_IRQ_HS,
					hs_handler_tv_out, data);

	} else {
		retval = cpcap_irq_register(data->cpcap,
					CPCAP_IRQ_HS, hs_handler, data);
	}

	if (retval)
		goto reg_put;

	retval = cpcap_irq_register(data->cpcap, CPCAP_IRQ_MB2, key_handler,
				data);
	if (retval)
		goto free_hs;

	retval = cpcap_irq_register(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_5,
					key_handler, data);
	if (retval)
		goto free_mb2;

	if (barrel_capability == BARREL_CAP_DETECT_TV_OUT)
		hs_handler_tv_out(CPCAP_IRQ_HS, data);
	else
		hs_handler(CPCAP_IRQ_HS, data);

	return 0;

free_mb2:
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_MB2);
free_hs:
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_HS);
reg_put:
	regulator_put(data->regulator);
free_mem:
	kfree(data);

	return retval;
}

static int __exit cpcap_3mm5_remove(struct platform_device *pdev)
{
	struct cpcap_3mm5_data *data = platform_get_drvdata(pdev);

	cpcap_irq_free(data->cpcap, CPCAP_IRQ_MB2);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_HS);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);

	switch_dev_unregister(&data->sdev);
	regulator_put(data->regulator);

	kfree(data);
	return 0;
}

static struct platform_driver cpcap_3mm5_driver = {
	.probe		= cpcap_3mm5_probe,
	.remove		= __exit_p(cpcap_3mm5_remove),
	.driver		= {
		.name	= "cpcap_3mm5",
		.owner	= THIS_MODULE,
	},
};

static int __init cpcap_3mm5_init(void)
{
	return platform_driver_register(&cpcap_3mm5_driver);
}
module_init(cpcap_3mm5_init);

static void __exit cpcap_3mm5_exit(void)
{
	platform_driver_unregister(&cpcap_3mm5_driver);
}
module_exit(cpcap_3mm5_exit);

MODULE_ALIAS("platform:cpcap_3mm5");
MODULE_DESCRIPTION("CPCAP USB detection driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
