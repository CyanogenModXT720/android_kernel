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

#include <linux/interrupt.h>
#include <linux/gpio.h>

#define MAX_ADC_RETRIES 5
#define TVOUT_ADC_THRESHOLD 46

#define NO_DEVICE 0
#define HEADSET_WITH_MIC (1<<0)
#define HEADSET_WITHOUT_MIC (1<<1)
#define TVOUT_DET (1<<5)

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
	struct work_struct work_queue;
	struct work_struct tvint_work_queue;
	unsigned char audio_low_power;
	int is_tv_enabled;
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
	case TVOUT_DET:
		return sprintf(buf, "Television\n");
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
		pr_err("Cpcap TV_out: %s: "
				"Configuring GPIO2 VLEV failed: \n", __func__);
		return cpcap_status;
	}

	/* set vlev = 2.775V for GPIO 4 */
	cpcap_status = cpcap_regacc_write(data_3mm5->cpcap,
					CPCAP_REG_GPIO4, CPCAP_BIT_GPIO4VLEV,
					CPCAP_BIT_GPIO4VLEV);
	if (cpcap_status < 0) {
		pr_err("Cpcap TV_out: %s: "
				"Configuring GPIO4 VLEV failed: \n", __func__);
		return cpcap_status;
	}

	cpcap_status = cpcap_regacc_write(data_3mm5->cpcap,
					CPCAP_REG_GPIO2, CPCAP_BIT_GPIO2DIR,
					CPCAP_BIT_GPIO2DIR);
	if (cpcap_status < 0) {
		pr_err("Cpcap TV_out: %s: "
				"Configuring GPIO2 failed: \n", __func__);
		return cpcap_status;
	}
	cpcap_status = cpcap_regacc_write(data_3mm5->cpcap,
					CPCAP_REG_GPIO4, CPCAP_BIT_GPIO4DIR,
					CPCAP_BIT_GPIO4DIR);

	if (cpcap_status < 0) {
		pr_err("Cpcap TV_out: %s: "
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

static int get_adc9_count(struct cpcap_device *cpcap)
{
	int adc_read_return = 0;
	int adc9_count = 0, read_count = 0;
	struct cpcap_adc_request adc9_req;
	adc9_req.result[CPCAP_ADC_AD9] = 0;

	adc9_req.type   = CPCAP_ADC_TYPE_BANK_1;
	adc9_req.timing = CPCAP_ADC_TIMING_IMM;
	adc9_req.format = CPCAP_ADC_FORMAT_RAW;

	for (read_count = 0; read_count < MAX_ADC_RETRIES; read_count++) {
		adc_read_return = cpcap_adc_sync_read(cpcap, &adc9_req);
		if (adc_read_return == 0)
			break;
	}
	if (adc_read_return == 0) {
		adc9_count = adc9_req.result[CPCAP_ADC_AD9];
	} else {
		pr_err("Cpcap tv_out : %s: "
				"cpcap_adc_sync_read failed: \n", __func__);
		adc_read_return = -1;
	}

	printk(KERN_INFO "get_adc9_count : adc9_count = %d\n", adc9_count);
	return adc9_count;
}

void configure_cpcap_irq(int device, void *data)
{
	struct cpcap_3mm5_data *data_3mm5 = data;

	switch (device) {
	case HEADSET_WITH_MIC:
	case HEADSET_WITHOUT_MIC:
		cpcap_irq_clear(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_clear(data_3mm5->cpcap,
			CPCAP_IRQ_UC_PRIMACRO_5);

		cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_unmask(data_3mm5->cpcap,
			CPCAP_IRQ_UC_PRIMACRO_5);
		cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_HS);

		cpcap_uc_start(data_3mm5->cpcap, CPCAP_MACRO_5);
		cpcap_uc_start(data_3mm5->cpcap, CPCAP_MACRO_4);
		break;

	case TVOUT_DET:
		cpcap_irq_mask(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_mask(data_3mm5->cpcap,
			CPCAP_IRQ_UC_PRIMACRO_5);

		cpcap_irq_clear(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_clear(data_3mm5->cpcap,
			CPCAP_IRQ_UC_PRIMACRO_5);
		cpcap_irq_mask(data_3mm5->cpcap, CPCAP_IRQ_HS);
		break;

	case NO_DEVICE:
		cpcap_irq_mask(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_mask(data_3mm5->cpcap,
			CPCAP_IRQ_UC_PRIMACRO_5);
		cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_HS);

		cpcap_irq_clear(data_3mm5->cpcap, CPCAP_IRQ_MB2);
		cpcap_irq_clear(data_3mm5->cpcap,
			CPCAP_IRQ_UC_PRIMACRO_5);
		break;

	default:
		break;
	}

	printk(KERN_INFO "configure_cpcap_irq : "
				"h2w_switch_state = %d\n", device);
}

static void hs_work(struct work_struct *work)
{
	struct cpcap_3mm5_data *data_3mm5 = NULL;
	struct cpcap_platform_data *platform_data = NULL;
	struct cpcap_3mm5_tvint *tvint = NULL;
	int new_state = NO_DEVICE;
	int read_ptt = 0;
	int read_mb2sns = 0;
	int control_val = 0;
	int present = 0;
	int adc9_count = 0;

	data_3mm5 = container_of(work, struct cpcap_3mm5_data, work_queue);
	platform_data = data_3mm5->cpcap->spi->controller_data;
	tvint = platform_data->tvint;

	/* Redetect */
	switch_set_state(&data_3mm5->sdev, NO_DEVICE);

	dev_info(&data_3mm5->cpcap->spi->dev,
			" Redetect:"
			" Headset detached: Connected=%d\n", NO_DEVICE);

   cpcap_irq_mask(data_3mm5->cpcap, CPCAP_IRQ_PTT);

	/*Enable HSMIC NO0 path*/
	control_val = control_analog_switch(CPCAP_GPIO_SWITCH_HSMIC, data_3mm5);
	if (control_val < 0) {
		cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_TXI, 0,
		(CPCAP_BIT_MB_ON2 | CPCAP_BIT_PTT_CMP_EN));
		return;
	}

	cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_TXI,
			(CPCAP_BIT_MB_ON2 | CPCAP_BIT_PTT_CMP_EN),
			(CPCAP_BIT_MB_ON2 | CPCAP_BIT_PTT_CMP_EN));
	audio_low_power_clear(data_3mm5);

	/* Give PTTS time to settle */
	msleep(20);

    cpcap_irq_unmask(data_3mm5->cpcap, CPCAP_IRQ_PTT);

	read_ptt = cpcap_irq_sense(data_3mm5->cpcap, CPCAP_IRQ_PTT, 1);
	if (read_ptt <= 0) {
		/* 1. Headset without mic is detected.
		2. Parallel headset with the MFB pressed.
		3. AV cable connected to TV
		4. Audio RCA cable*/

		/*Enable ADC NO1 path*/
		control_val = control_analog_switch(
						CPCAP_GPIO_SWITCH_ADC,
						data_3mm5);

		if (control_val < 0) {
			cpcap_regacc_write(data_3mm5->cpcap,
				CPCAP_REG_TXI, 0,
				(CPCAP_BIT_MB_ON2 | CPCAP_BIT_PTT_CMP_EN));
			return;
		}

		/*NO send/end key can be pressed so */
		/*read HIGH and then notify for TV*/

		adc9_count = get_adc9_count(data_3mm5->cpcap);

		if (adc9_count > TVOUT_ADC_THRESHOLD) {
#ifdef CONFIG_TVOUT_SHOLEST
			/* Comment out the below code so that
			present = 0 always and TV out path is never enabled.
			We dont need TVout for this product.
			This is a workaround to solve headset issues.
			present = venc_tv_connect(); */
#endif

			if (present) {
				/*Enable TV NO2 path*/
				control_val = control_analog_switch(
						CPCAP_GPIO_SWITCH_TVOUT,
						data_3mm5);

				if (control_val < 0) {
					cpcap_regacc_write(data_3mm5->cpcap,
						CPCAP_REG_TXI, 0,
						(CPCAP_BIT_MB_ON2 |
						CPCAP_BIT_PTT_CMP_EN));
					return;
				}
				if (data_3mm5->is_tv_enabled == 0) {
					enable_irq(tvint->tvint_irq);
					data_3mm5->is_tv_enabled = 1;
				}

				configure_cpcap_irq(TVOUT_DET, data_3mm5);
				new_state = TVOUT_DET;
			} else {
				/*Select audio path using CPCAP GPIO's*/
				control_val = control_analog_switch(
							CPCAP_GPIO_SWITCH_HSMIC,
							data_3mm5);

				if (control_val < 0) {
					cpcap_regacc_write(data_3mm5->cpcap,
						CPCAP_REG_TXI, 0,
						(CPCAP_BIT_MB_ON2 |
						CPCAP_BIT_PTT_CMP_EN));
					return;
				}

				configure_cpcap_irq(HEADSET_WITHOUT_MIC,
								data_3mm5);
				/*Headset with no mic detected*/
				new_state = HEADSET_WITHOUT_MIC;
			}
		} else {
			/*Select audio path using CPCAP GPIO's*/
			control_val = control_analog_switch(
							CPCAP_GPIO_SWITCH_HSMIC,
							data_3mm5);

			if (control_val < 0) {
				cpcap_regacc_write(data_3mm5->cpcap,
					CPCAP_REG_TXI, 0,
					(CPCAP_BIT_MB_ON2 |
					CPCAP_BIT_PTT_CMP_EN));
				return;
			}

			configure_cpcap_irq(HEADSET_WITHOUT_MIC, data_3mm5);
			/*Headset with no mic detected*/
			new_state = HEADSET_WITHOUT_MIC;
		}
	} else {/* if PTT > 0*/
		read_mb2sns = cpcap_irq_sense(data_3mm5->cpcap,
						CPCAP_IRQ_MB2, 0);

		if (read_mb2sns == 0) {
			configure_cpcap_irq(HEADSET_WITHOUT_MIC, data_3mm5);

			/*HS w/o mic detected or A/V cable not plugged in Tv*/
			new_state = HEADSET_WITHOUT_MIC;
		} else {
			configure_cpcap_irq(HEADSET_WITH_MIC, data_3mm5);
			/*Headset with mic detected*/
			new_state = HEADSET_WITH_MIC;
		}
	}

	switch_set_state(&data_3mm5->sdev, new_state);

	/* LIBtt53617, LIBtt53952*/
	if (data_3mm5->cpcap->h2w_new_state)
		data_3mm5->cpcap->h2w_new_state(new_state);
		
	dev_info(&data_3mm5->cpcap->spi->dev, "New Accessory state: %d\n",
		 new_state);
}


static void hs_handler_tv_out(enum cpcap_irqs irq, void *data)
{
	struct cpcap_3mm5_data *data_3mm5 = data;
	struct cpcap_platform_data *platform_data = NULL;
	struct cpcap_3mm5_tvint *tvint = NULL;
	int read_hs = 0;

	platform_data = data_3mm5->cpcap->spi->controller_data;
	tvint = platform_data->tvint;

	if (irq != CPCAP_IRQ_HS) {
		pr_err("Cpcap TV_out: %s:"
				"Invalid headset IRQ : \n", __func__);
		return;
	}
	read_hs = cpcap_irq_sense(data_3mm5->cpcap, CPCAP_IRQ_HS, 1);

	/* HS sense of 1 means no headset present, 0 means headset attached. */
	if (read_hs == 1) {
		cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_TXI, 0,
			(CPCAP_BIT_MB_ON2 | CPCAP_BIT_PTT_CMP_EN));
		cpcap_regacc_write(data_3mm5->cpcap, CPCAP_REG_RXOA, 0,
				   CPCAP_BIT_ST_HS_CP_EN);
		audio_low_power_set(data_3mm5);

		configure_cpcap_irq(NO_DEVICE, data_3mm5);

		/* Turn off analog switch whenever no headset plugged in */
	    control_analog_switch(CPCAP_GPIO_SWITCH_OFF, data_3mm5);

		send_key_event(data_3mm5, 0);

		cpcap_uc_stop(data_3mm5->cpcap, CPCAP_MACRO_5);

		if (data_3mm5->is_tv_enabled == 1) {
			disable_irq(tvint->tvint_irq);
			data_3mm5->is_tv_enabled = 0;
		}
		switch_set_state(&data_3mm5->sdev, NO_DEVICE);
		dev_info(&data_3mm5->cpcap->spi->dev,
			"New Accessory state: %d\n",
			NO_DEVICE);

	} else {
		schedule_work(&(data_3mm5->work_queue));
	}
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
		audio_low_power_clear(data_3mm5);

		/* Give PTTS time to settle */
		mdelay(20);

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
	if (data_3mm5->cpcap->h2w_new_state)
		data_3mm5->cpcap->h2w_new_state(new_state);

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

static irqreturn_t tvint_irq(int irq, void *data)
{
	struct cpcap_3mm5_data *data_3mm5 = data;
	struct cpcap_platform_data *platform_data = 0;
	struct cpcap_3mm5_tvint *tvint = 0;
	int present = 0;
	platform_data = data_3mm5->cpcap->spi->controller_data;
	tvint = platform_data->tvint;

	present = gpio_get_value(irq_to_gpio(tvint->tvint_irq));

	printk(KERN_INFO "present=%d, is_tv_enabled=%d\n", present, data_3mm5->is_tv_enabled);
	if ((present == 0) && (data_3mm5->is_tv_enabled == 1)) {
		venc_tv_disconnect();
		disable_irq(tvint->tvint_irq);
		data_3mm5->is_tv_enabled = 0;
		schedule_work(&data_3mm5->tvint_work_queue);
	}
	return IRQ_HANDLED;
}

static void tvint_work(struct work_struct *work)
{
	struct cpcap_3mm5_data *data_3mm5;
	int read_hs = 0;

	data_3mm5 = container_of(work, struct cpcap_3mm5_data,
					tvint_work_queue);
	/*Give time for HS to settle */
	mdelay(100);

	hs_handler_tv_out(CPCAP_IRQ_HS, data_3mm5);
}

static int __init cpcap_3mm5_probe(struct platform_device *pdev)
{
	int retval = 0;
	int barrel_capability;
	struct cpcap_3mm5_data *data;
	struct cpcap_platform_data *platform_data = NULL;
	struct cpcap_3mm5_tvint *tvint = NULL;

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
	data->is_tv_enabled = 0;
	switch_dev_register(&data->sdev);
	platform_set_drvdata(pdev, data);

	platform_data = data->cpcap->spi->controller_data;
	tvint = platform_data->tvint;
	barrel_capability = platform_data->barrel_capability;

	if (barrel_capability == BARREL_CAP_DETECT_TV_OUT) {
		retval = init_analog_switch(data);
		if (retval < 0)
			return retval;
	}

	data->regulator = regulator_get(NULL, "vaudio");
	if (IS_ERR(data->regulator)) {
		dev_err(&pdev->dev, "Could not get regulator for cpcap_3mm5\n");
		retval = PTR_ERR(data->regulator);
		goto free_mem;
	}

	regulator_set_voltage(data->regulator, 2775000, 2775000);

	retval  = cpcap_irq_clear(data->cpcap, CPCAP_IRQ_HS);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_MB2);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);
	if (retval)
		goto reg_put;

	if (barrel_capability == BARREL_CAP_DETECT_TV_OUT) {
		retval = cpcap_irq_register(data->cpcap, CPCAP_IRQ_HS,
					hs_handler_tv_out, data);

		INIT_WORK(&(data->tvint_work_queue), tvint_work);
		INIT_WORK(&(data->work_queue), hs_work);
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

	if (barrel_capability == BARREL_CAP_DETECT_TV_OUT) {
		retval = request_irq(tvint->tvint_irq,
				tvint_irq,
				IRQF_TRIGGER_FALLING|IRQF_DISABLED,
				"tvout",
				data);

		if (retval)
			goto free_macro;

		disable_irq(tvint->tvint_irq);
		hs_handler_tv_out(CPCAP_IRQ_HS, data);

	} else
		hs_handler(CPCAP_IRQ_HS, data);

	return 0;

free_macro:
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);
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
	int barrel_capability;
	struct cpcap_3mm5_data *data = platform_get_drvdata(pdev);
	struct cpcap_platform_data *platform_data = NULL;
	struct cpcap_3mm5_tvint *tvint = NULL;

	platform_data = data->cpcap->spi->controller_data;
	tvint = platform_data->tvint;
	barrel_capability = platform_data->barrel_capability;

	cpcap_irq_free(data->cpcap, CPCAP_IRQ_MB2);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_HS);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_5);

	switch_dev_unregister(&data->sdev);
	regulator_put(data->regulator);

	if (barrel_capability == BARREL_CAP_DETECT_TV_OUT)
		free_irq(tvint->tvint_irq, data);

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
