/*
 * drivers/media/video/ov8810.c
 *
 * ov8810 sensor driver
 *
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Leverage ov8810.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#define DEBUG

#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>
#include "ov8810.h"
#include "oldomap34xxcam.h"
#include "oldisp/isp.h"
#include "oldisp/ispcsi2.h"

/*It for manual trigger oepration.*/

#define OV8810_DRIVER_NAME  "ov8810"
#define MOD_NAME "OV8810: "

#define I2C_M_WR 0

/* OV8810 clock related parameters */
struct ov8810_clk_params {
	u32 xclk;
	u32 sclk;
	u32 pclk;
	u32 mipiclk;
	u16 pll_mult;
	u16 pll_pre_div;
	u16 vt_sys_div;
	u16 op_sys_div;
	u16 op_pix_div;
	u16 div8;
	u16 rp_clk_div;
	u16 num_mipi_lanes;
};

/* OV8810 image frame parameters */
struct ov8810_frame_params {
	u16 line_length_pck;
	u16 frame_length_lines;
	u16 line_length_pck_min;
	u16 frame_length_lines_min;
	u8 v_subsample;
	u8 h_subsample;
};

struct ov8810_sensor_id {
	u16 revision;
	u16 model;
	u16 mfr;
};

/**
 * struct ov8810_sensor_params
 */
struct ov8810_sensor_params {
	u32 line_time;  /* usec, q8 */
	u16 gain_frame_delay;
	u16 exp_time_frame_delay;
};

struct ov8810_flash_params {
	u16 flash_time;
	u8 flash_type;
	u8 shutter_type;
};

struct ov8810_shutter_params {
	u32 exp_time;
	u16 delay_time;
};

struct ov8810_exp_params {
	u32 exp_time;
	u32 line_time;
	u16 num_exp_lines;
	u16 min_exp_time;
	u32 max_exp_time;
};

/**
 * struct ov8810_sensor - main structure for storage of sensor information
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @i2c_client: iic client device structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @isize: base image size
 * @ver: ov8810 chip version
 * @width: configured width
 * @height: configuredheight
 * @vsize: vertical size for the image
 * @hsize: horizontal size for the image
 * @crop_rect: crop rectangle specifying the left,top and width and height
 * @state:
 * @frame: image frame parameters
*/
struct ov8810_sensor {
	struct device *dev;
	const struct ov8810_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	int isize;
	int fps;
	unsigned long width;
	unsigned long height;
	unsigned long vsize;
	unsigned long hsize;
	struct v4l2_rect crop_rect;
	int state;
	bool resuming;
	struct ov8810_clk_params clk;
	struct ov8810_frame_params frame;
	struct ov8810_sensor_id sensor_id;
	struct ov8810_flash_params flash;
	struct ov8810_shutter_params shutter;
	struct ov8810_exp_params exposure;
};

static struct ov8810_sensor ov8810 = {
	.timeperframe = {
		.numerator = 1,
		.denominator = 15,
	},
	.sensor_id = {
		.revision = 0,
		.model = 0,
		.mfr = 0
	},
	.state = SENSOR_NOT_DETECTED,
	.clk = {
		.xclk = OV8810_XCLK_MIN,
	}
};

static struct i2c_driver ov8810sensor_i2c_driver;
static enum v4l2_power current_power_state = V4L2_POWER_OFF;

/* List of image formats supported by OV8810 sensor */
const static struct v4l2_fmtdesc ov8810_formats[] = {
	{
		.description	= "RAW10",
		.pixelformat	= V4L2_PIX_FMT_SGRBG10,
	},
	{
		.description	= "Walking 1's pattern",
		.pixelformat	= V4L2_PIX_FMT_W1S_PATT,
	}
};

#define NUM_CAPTURE_FORMATS (sizeof(ov8810_formats) / sizeof(ov8810_formats[0]))

/* register initialization tables for ov8810 */
#define OV8810_REG_TERM 0xFFFF	/* terminating list entry for reg */
#define OV8810_VAL_TERM 0xFF	/* terminating list entry for val */

const static struct ov8810_reg ov8810_strobe_ready_reg[] = {
    {0x30e8 , 0x00},
    {OV8810_REG_TERM, OV8810_VAL_TERM},
};

const static struct ov8810_reg ov8810_strobe_trigger_reg[] = {
    {0x30e7 , 0x03},
    {0x30e8 , 0x80},
    {OV8810_REG_TERM, OV8810_VAL_TERM},
};

/*
 * struct vcontrol - Video controls
 * @v4l2_queryctrl: V4L2 VIDIOC_QUERYCTRL ioctl structure
 * @current_value: current value of this control
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = 0,
			.maximum = -1,
			.step = EXPOSURE_STEP,
			.default_value = DEF_EXPOSURE,
		},
		.current_value = DEF_EXPOSURE,
	},
	{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Analog Gain",
			.minimum = 0,
			.maximum = -1,
			.step = LINEAR_GAIN_STEP,
			.default_value = DEF_LINEAR_GAIN,
		},
		.current_value = DEF_LINEAR_GAIN,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_COLOR_BAR,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Color Bar",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = 0,
		},
		.current_value = 0,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_FLASH_NEXT_FRAME,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Flash On Next Frame",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = 0,
		},
		.current_value = 0,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_SENSOR_ID_REQ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Sensor ID",
			.minimum = 0,
			.maximum = -1,
			.step = 0,
			.default_value = 0,
		},
		.current_value = 0,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_ORIENTATION,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Orientation",
			.minimum = OV8810_NO_HORZ_FLIP_OR_VERT_FLIP,
			.maximum = OV8810_HORZ_FLIP_AND_VERT_FLIP,
			.step = 0,
			.default_value = OV8810_NO_HORZ_FLIP_OR_VERT_FLIP,
		},
		.current_value = OV8810_NO_HORZ_FLIP_OR_VERT_FLIP,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_START_MECH_SHUTTER_CAPTURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Mechanical Shutter",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.current_value = 0,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_SENSOR_REG_REQ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Sensor Register",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.current_value = 0,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_SENSOR_PARAMS_REQ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Sensor Params",
			.minimum = 0,
			.maximum = -1,
			.step = 0,
			.default_value = 0,
		},
		.current_value = 0,
	},
};

struct i2c_client *ov8810_i2c_client;

/*
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array.
 *
 * Returns the index of the requested ID from the control structure array
 */
static int find_vctrl(int id)
{
	int i = 0;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/*
 * Read a value from a register in ov8810 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov8810_read_reg(struct i2c_client *client, u16 data_length, u16 reg,
								u32 *val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;

	/* High byte goes out first */
	data[0] = (u8) (reg >> 8);
	data[1] = (u8) (reg & 0xff);

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {
		mdelay(3);
		msg->flags = I2C_M_RD;
		msg->len = data_length;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = 0;
		/* High byte comes first */
		if (data_length == 1)
			*val = data[0];
		else if (data_length == 2)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
				(data[1] << 16) + (data[0] << 24);
		return 0;
	}
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

/* Write a value to a register in ov8810 sensor device.
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov8810_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[3];
	int retries = 0;

	if (!client->adapter)
		return -ENODEV;
retry:
	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 3;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);
	data[1] = (u8) (reg & 0xff);
	data[2] = val;

	err = i2c_transfer(client->adapter, msg, 1);
	udelay(50);

	if (err >= 0)
		return 0;

	if (retries <= 5) {
		dev_dbg(&client->dev, "Retrying I2C... %d\n", retries);
		retries++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto retry;
	}

	return err;
}

/*
 * Initialize a list of ov8810 registers.
 * The list of registers is terminated by the pair of values
 * {OV8810_REG_TERM, OV8810_VAL_TERM}.
 * @client: i2c driver client structure.
 * @reglist[]: List of address of the registers to write data.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov8810_write_regs(struct i2c_client *client,
					const struct ov8810_reg reglist[])
{
	int err = 0;
	const struct ov8810_reg *next = reglist;

	while (!((next->reg == OV8810_REG_TERM)
		&& (next->val == OV8810_VAL_TERM))) {
		err = ov8810_write_reg(client, next->reg, next->val);
		udelay(100);
		if (err)
			return err;
		next++;
	}
	return 0;
}

/**
 * ov8810_set_exposure_time - sets exposure time per input value
 * @exp_time: exposure time to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 *
 * If the requested exposure time is not within the allowed limits, the
 * exposure time is forced to the limit value. The HW
 * is configured to use the new exposure time, and the
 * video_control[] array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov8810_set_exposure_time(u32 exp_time, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
	/* Inputs exp_time in usec */
	u8 lut[4] = {1, 1, 2, 4}, skip_factor;
	u16 num_exp_lines;
	u32 line_time_q8, min_exposure_time, max_exposure_time;
	int err = 0;
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	/* TEMP - FORCE EXP TIME - GVH */
	/*~ exp_time = 20000; */

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {

		skip_factor = lut[sensor->frame.h_subsample];
		line_time_q8 = /* usec's (q8) */
			((((u32)sensor->frame.line_length_pck * 1000) << 8) /
			(sensor->clk.pclk / 1000) / skip_factor);

		/* Update min/max exposure times */
		/* TODO: Move these calcs to set_framerate */
		/*~ min_exposure_time = (ss->exposure.fine_int_tm * 1000000 /
					~ (current_clk.vt_pix_clk)) + 1; */
		/* use line time for min until LAEC turned on - GVH */
		min_exposure_time = line_time_q8 >> 8;
		max_exposure_time = (line_time_q8 *
			(sensor->frame.frame_length_lines - 1)) >> 8;

		if (exp_time < min_exposure_time) {
			dev_err(&client->dev, "Exposure time %d us not within "
				"the legal range.\n", exp_time);
			dev_err(&client->dev, "Exposure time must be "
				"greater than %d us\n", min_exposure_time);
			exp_time = min_exposure_time;
		}

		if (exp_time > max_exposure_time) {
			dev_err(&client->dev, "Exposure time %d us not within "
				"the legal range.\n", exp_time);
			dev_err(&client->dev, "Exposure time must be "
				"less than %d us\n", max_exposure_time);
			exp_time = max_exposure_time;
		}

		/* calc num lines with rounding */
		num_exp_lines = ((exp_time << 8) + (line_time_q8 >> 1)) /
			line_time_q8;

		dev_dbg(&client->dev, "ov8810:set_exposure_time = %d usec, " \
			"CoarseIntTime = %d, sclk=%d, line_len_pck=%d clks, " \
			"line_tm = %d/256 us, skip factor=%d\n",
			exp_time, num_exp_lines, sensor->clk.sclk,
			sensor->frame.line_length_pck,  line_time_q8,
			skip_factor);

		/* write number of line times to AEL/H registers */
		err = ov8810_write_reg(client, OV8810_AECL_H,
			num_exp_lines >> 8);
		err = ov8810_write_reg(client, OV8810_AECL_L,
			num_exp_lines & 0xFF);

		/* save results */
		sensor->exposure.exp_time = exp_time;
		sensor->exposure.line_time = line_time_q8;
		sensor->exposure.num_exp_lines = num_exp_lines;
		sensor->exposure.min_exp_time = min_exposure_time;
		sensor->exposure.max_exp_time = max_exposure_time;
	}

	if (err)
		dev_err(&client->dev, "Error setting exposure time...%d\n",
									err);
	else {
		if (lvc)
			lvc->current_value = exp_time;
	}

	return err;
}


/**
 * ov8810_set_gain - sets sensor analog gain per input value
 * @lineargain: q8 analog gain value to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 analog gain entry in ov8810_video_control array
 *
 * If the requested analog gain is within the allowed limits, the HW
 * is configured to use the new gain value, and the ov8810_video_control
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov8810_set_gain(u16 linear_gain_Q8, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
	/* Inputs linear Q8 gain */
	u16 gain_stage_2x = 0;
	u16 shift_bits = 0;
	u16 gain_fraction = 0;
	u16 gain_register = 0;
	int err = 0;
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	/* TEMP - FORCE GAIN - GVH */
	/*~ linear_gain_Q8 = 2 * 256; */

	/* TODO: Add Limit Checks */

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		if (linear_gain_Q8 >= 8*256) {
			gain_stage_2x = 0x70;
			shift_bits = 3;
		} else if (linear_gain_Q8 >= 4*256) {
			gain_stage_2x = 0x30;
			shift_bits = 2;
		} else if (linear_gain_Q8 >= 2*256) {
			gain_stage_2x = 0x10;
			shift_bits = 1;
		}

		gain_fraction = linear_gain_Q8 >> shift_bits;
		 /* subt 1 (Q8) and take upper 4 bits */
		gain_fraction = (gain_fraction - (1*256)) >> 4;
		if (gain_fraction > 0x0f)
			gain_fraction = 0x0f;

		gain_register = gain_stage_2x | gain_fraction;

		dev_dbg(&client->dev, "gain =%d/256, gain reg = 0x%x\n",
			linear_gain_Q8, gain_register);

		err = ov8810_write_reg(client, OV8810_AGCL, gain_register);
	}

	if (err) {
		dev_err(&client->dev, "Error setting analog gain: %d\n", err);
		return err;
	} else {
		if (lvc)
			lvc->current_value = linear_gain_Q8;
	}

	return err;
}

/**
 * ov8810_set_color_bar_mode - puts sensor in color bar test mode
 * @enable: 0 = off, 1 = on
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 * This function should be called after the resolution is setup. The sensor
 * will stay in color bar mode until the next resolution is selected.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov8810_set_color_bar_mode(u16 enable, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
	int err = 0;
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		if (enable) {
			err = ov8810_write_reg(client, OV8810_CBAR, 0x1);
			err = ov8810_write_reg(client, OV8810_SIZE_H0, 0x2);
		} else {
			err = ov8810_write_reg(client, OV8810_CBAR, 0x0);
			err = ov8810_write_reg(client, OV8810_SIZE_H0, 0x3);
		}
	}

	if (err)
		dev_err(&client->dev, "Error setting color bar mode\n");
	else {
		if (lvc)
			lvc->current_value = enable;
	}

	return err;
}

int ov8810_strobe_manual_trigger(void)
{
    return ov8810_write_regs(ov8810_i2c_client, ov8810_strobe_trigger_reg);
}

/**
 * ov8810_set_flash_next_frame - configures flash on for the next frame
 * @flash_params: flash type and time
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov8810_set_flash_next_frame(
			struct ov8810_flash_params *flash_params,
			struct v4l2_int_device *s, struct vcontrol *lvc)
{
	int err = 0, data;
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	u32 strb_pulse_width;
	u32	line_time_q8 = sensor->exposure.line_time;

	dev_dbg(&client->dev, "set_flash_next_frame: time=%dusec, " \
		"flash_type=%d, shutter_type=%d, power=%d\n",
		flash_params->flash_time, flash_params->flash_type,
		flash_params->shutter_type,
		(current_power_state == V4L2_POWER_ON) || sensor->resuming);

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		if (flash_params->flash_time != 0) {

			/* Set strobe source frame type */
			ov8810_read_reg(client, 1, OV8810_FRS4, &data);
			if (flash_params->shutter_type == ROLLING_SHUTTER_TYPE)
				data |= 1 << OV8810_FRS4_STRB_SOURCE_SEL_SHIFT;
			else
				data &= \
				~(1 << OV8810_FRS4_STRB_SOURCE_SEL_SHIFT);
			err = ov8810_write_reg(client, OV8810_FRS4, data);

			dev_dbg(&client->dev, "set_flash_next_frame:  " \
				"OV8810_FRS_4=0x%x\n", data);

			/* Set Strobe Ctrl Reg */
			data = 0;
			if (flash_params->shutter_type ==
							ROLLING_SHUTTER_TYPE)
				data |= 1 <<
					OV8810_FRS5_ROLLING_SHUT_STRB_EN_SHIFT;

			if (flash_params->flash_type == LED_FLASH_TYPE)
				data |= 1 << OV8810_FRS5_STROBE_MODE_SHIFT;

			strb_pulse_width = (flash_params->flash_time << 8) /
				line_time_q8;

			if (strb_pulse_width < 1)
				strb_pulse_width = 1;
			else if (strb_pulse_width > 4)
				strb_pulse_width = 4;

			data |= (strb_pulse_width - 1) <<
				OV8810_FRS5_STRB_PLS_WIDTH_SHIFT;

			err |= ov8810_write_reg(client, OV8810_FRS5, data);

			dev_dbg(&client->dev, "set_flash_next_frame:  " \
				"OV8810_FRS_5=0x%x\n", data);

			/* Set Frame Mode Strobe Pulse Width */
			if (flash_params->shutter_type == MECH_SHUTTER_TYPE) {
				strb_pulse_width = \
				(flash_params->flash_time << 8) / line_time_q8;

				if (strb_pulse_width > 15)
					strb_pulse_width = 15;

				err |= ov8810_write_reg(client, OV8810_FRS6,
					strb_pulse_width);

				dev_dbg(&client->dev,
					"set_flash_next_frame:  " \
					"OV8810_FRS_6=0x%x\n",
					strb_pulse_width);
			}

			/* Auto reset */
			flash_params->flash_time = 0;
		}
	}

	if (err)
		dev_err(&client->dev, "Error setting flash register\n");
	else {
		sensor->flash.flash_time = flash_params->flash_time;
		sensor->flash.flash_type = flash_params->flash_type;
		sensor->flash.shutter_type = flash_params->shutter_type;
	}
	return err;
}

/**
 * ov8810_start_mech_shutter_capture - initiates capture using mechanical shutter
 * @shutter_params: expoosure and shutter delay time
 * @s: pointer to standard V4L2 device structure
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov8810_start_mech_shutter_capture(
			struct ov8810_shutter_params *shutter_params,
			struct v4l2_int_device *s, struct vcontrol *lvc)
{
	u16 Tr_lines = 1, Tfrex_lines;
	int err = 0, adjusted_exp_time;
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	u32 line_time_q8 = sensor->exposure.line_time;

	dev_dbg(&client->dev, "expTime=%dus, shutter delay=%dus, " \
		"power=%d\n",
		shutter_params->exp_time, shutter_params->delay_time,
		(current_power_state == V4L2_POWER_ON) || sensor->resuming);

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		if (shutter_params->exp_time != 0) {

			/* Set FREX Precharge Time */
			err = ov8810_write_reg(client, OV8810_FRS1, Tr_lines);

			/* Calc Tfrex time */
			adjusted_exp_time = shutter_params->exp_time -
				shutter_params->delay_time;
			if (adjusted_exp_time < 0)
				adjusted_exp_time = 0;

			/* Convert Tfrex time to lines (with rounding) */
			Tfrex_lines = (((adjusted_exp_time << 8) +
				(line_time_q8 >> 1)) / line_time_q8) + Tr_lines;

			err |= ov8810_write_reg(client, OV8810_FRS2,
				((Tfrex_lines >> 8) & 0xFF));
			err |= ov8810_write_reg(client, OV8810_FRS3,
				((Tfrex_lines) & 0xFF));

			dev_dbg(&client->dev, "start_mech_shutter_capture:  " \
				"expT=%dus, line_time_q8=%dus/256, " \
				"shutter_dly=%dus Tfrex_lines=%d\n",
				shutter_params->exp_time, line_time_q8,
				shutter_params->delay_time, Tfrex_lines);

			/* start FREX capture */
			err |= ov8810_write_reg(client, OV8810_FRS7, 0x01);

			/* Auto reset */
			shutter_params->exp_time = 0;
		}
	}

	if (err) {
		dev_err(&client->dev, "Error setting mech shutter registers\n");
	} else {
		sensor->shutter.exp_time = shutter_params->exp_time;
		sensor->shutter.delay_time = shutter_params->delay_time;
	}

	return err;
}

/*
 * Update Image Frame Parameters.
 */
static int ov8810_update_frame_params(struct v4l2_int_device *s)
{
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	u32 val;

	ov8810_read_reg(client, 1, OV8810_FRM_LEN_LINES_L, &val);
	sensor->frame.frame_length_lines = val & 0xFF;
	ov8810_read_reg(client, 1, OV8810_FRM_LEN_LINES_H, &val);
	sensor->frame.frame_length_lines |= (val & 0xFF) << 8;

	ov8810_read_reg(client, 1, OV8810_LINE_LEN_PCK_L, &val);
	sensor->frame.line_length_pck = val & 0xFF;
	ov8810_read_reg(client, 1, OV8810_LINE_LEN_PCK_H, &val);
	sensor->frame.line_length_pck |= (val & 0xFF) << 8;

	ov8810_read_reg(client, 1, OV8810_IMAGE_TRANSFORM, &val);
	sensor->frame.v_subsample = (val >> 2) & 0x3;
	sensor->frame.h_subsample = val & 0x3;

	/* Initial parameters are used as the minimum values */
	sensor->frame.frame_length_lines_min = sensor->frame.frame_length_lines;
	sensor->frame.line_length_pck_min = sensor->frame.line_length_pck;

	return 0;
}

/*
 * Calculates the PClk.
 * 1) Read pclk related params
 * 2) Calc pclk
 *      Pclk = xclk * PLL_multiplier  / pll_pre_div / div8 /
 * 		vt_sys_div / rp_clk_div
 * NOTE:
 *  - The lookup table 'lut1' has been multiplied by 2 so all its values
 *    are integers. The numerator is multipllied by 2 in the Pclk
 *    calculation to compensate.
 */
static int ov8810_calc_pclk(struct v4l2_int_device *s)
{
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	u32 val;
	u8 lut1[16] = {1, 1, 2, 2, 4, 4, 4, 4, 8, 8, 8, 8, 8, 8, 8, 8};
	u8 lut2[8] = {1, 1, 1, 1, 4, 5, 6, 6};
	u8 lut3[4] = {1, 2, 4, 8};

	ov8810_read_reg(client, 1, OV8810_R_PLL1, &val);
	sensor->clk.vt_sys_div = lut1[(val >> 4) & 0xf];
	sensor->clk.div8 = lut2[val & 0x7];

	ov8810_read_reg(client, 1, OV8810_R_PLL2, &val);
	sensor->clk.op_sys_div = lut1[(val >> 4) & 0xf];
	sensor->clk.op_pix_div = lut1[val & 0xf];

	if ((val & 0xc) >> 2 == 1)
		sensor->clk.num_mipi_lanes = 2;
	else
		sensor->clk.num_mipi_lanes = 1;

	ov8810_read_reg(client, 1, OV8810_R_PLL3, &val);
	sensor->clk.pll_mult = val & 0x7f;

	ov8810_read_reg(client, 1, OV8810_R_PLL4, &val);
	sensor->clk.pll_pre_div = lut1[val & 0xf];

	ov8810_read_reg(client, 1, OV8810_DSIO0, &val);
	sensor->clk.rp_clk_div = lut3[val & 0x3];

	sensor->clk.sclk = sensor->clk.xclk * sensor->clk.pll_mult /
				sensor->clk.pll_pre_div / sensor->clk.div8 /
				sensor->clk.vt_sys_div;

	sensor->clk.pclk = sensor->clk.sclk / sensor->clk.rp_clk_div;

	dev_dbg(&client->dev, "ov8810_calc_pclk: vt_sys_div=%d, div8=%d, " \
		"op_sys_div=%d, op_pix_div=%d, pll_mult=%d, pll_pre_div=%d, " \
		"rp_clk_div=%d, sclk=%d, pclk=%d, # lanes=%d\n",
		sensor->clk.vt_sys_div, sensor->clk.div8,
		sensor->clk.op_sys_div, sensor->clk.op_pix_div,
		sensor->clk.pll_mult, sensor->clk.pll_pre_div,
		sensor->clk.rp_clk_div, sensor->clk.sclk, sensor->clk.pclk,
		sensor->clk.num_mipi_lanes);
	return 0;
}

/* Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum image_size_ov
ov8810_find_size(struct v4l2_int_device *s, unsigned int width,
	unsigned int height)
{
	enum image_size_ov size;
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	if ((width > ov8810_sizes[SIZE_2M].width) ||
		(height > ov8810_sizes[SIZE_2M].height))
		size = SIZE_8M;
	else if ((width > ov8810_sizes[SIZE_500K].width) ||
		(height > ov8810_sizes[SIZE_500K].height))
		size = SIZE_2M;
	else if ((width > ov8810_sizes[SIZE_125K].width) ||
		(height > ov8810_sizes[SIZE_125K].height))
		size = SIZE_500K;
	else
		size = SIZE_125K;

	dev_dbg(&client->dev, "ov8810_find_size: Req Width=%d, "
			"Find Size=%dx%d\n",
			width, (int)ov8810_sizes[size].width,
			(int)ov8810_sizes[size].height);

	return size;
}

/*
 * Set CSI2 Virtual ID.
 */
static int ov8810_set_virtual_id(struct i2c_client *client, u32 id)
{
	return ov8810_write_reg(client, OV8810_MIPI_CTRL02, (0x3 & id) << 6 |
									0x12);
}


/*
 * Calculates the MIPIClk.
 */
static u32 ov8810_calc_mipiclk(struct v4l2_int_device *s)
{
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	u32 val, pre_divider, multiplier, mipi_sys_div;
	u32 mipiclk, mclk = 24000000;

	ov8810_read_reg(client, 1, OV8810_R_PLL4, &val);
	if (val & 0x08)
		pre_divider = 8;
	else if (val & 0x04)
		pre_divider = 4;
	else if (val & 0x02)
		pre_divider = 2;
	else
		pre_divider = 1;

	ov8810_read_reg(client, 1, OV8810_R_PLL3, &val);
	multiplier = val & 0x3F;

	ov8810_read_reg(client, 1, OV8810_R_PLL2, &val);
	if (val & 0x80)
		mipi_sys_div = 8;
	else if (val & 0x40)
		mipi_sys_div = 4;
	else if (val & 0x20)
		mipi_sys_div = 2;
	else
		mipi_sys_div = 1;

	mipiclk = (mclk * multiplier)/(pre_divider * mipi_sys_div);

	dev_dbg(&client->dev, "ov8810: mipiclk=%u  pre_divider=%u  " \
		"multiplier=%u  mipi_sys_div=%u\n",
		mipiclk, pre_divider, multiplier, mipi_sys_div);

	return mipiclk;
}

/*
 * Configure the ov8810 for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the OV8810.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int ov8810_configure(struct v4l2_int_device *s)
{
	enum image_size_ov isize = SIZE_500K;
	int err = 0, i = 0;
	u32 mipiclk;
	enum pixel_format_ov pfmt = RAW10;
	u32 min_hs_zero_nui, min_hs_zero, min_hs_zero_total;
	u32 min_hs_prepare_nui, min_hs_prepare, min_hs_prepare_total;
	u32 max_hs_prepare_nui, max_hs_prepare, max_hs_prepare_total;
	u32 ubound_hs_settle, lbound_hs_settle;
	u32 val;
	struct ov8810_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &sensor->pix;
	struct i2c_client *client = sensor->i2c_client;
	struct vcontrol *lvc = NULL;

	switch (pix->pixelformat) {

	case V4L2_PIX_FMT_SGRBG10:
		pfmt = RAW10;
		break;
	}

	/* Set receivers virtual channel before sensor setup starts.
	 * Only set the sensors virtual channel after all other setup
	 * for the sensor is complete.
	 */
	isp_csi2_ctx_config_virtual_id(0, OV8810_CSI2_VIRTUAL_ID);
	isp_csi2_ctx_update(0, false);

	isize = ov8810_find_size(s, pix->width, pix->height);

	dev_dbg(&client->dev, "ov8810_configure: isize=%d, Req Size=%dx%d, " \
			"Find Size = %dx%d, fps=%d/%d\n", \
			isize, pix->width, pix->height,
		(int)ov8810_sizes[isize].width,
		(int)ov8810_sizes[isize].height,
		sensor->timeperframe.denominator,
		sensor->timeperframe.numerator);

	/* Reset */
	isp_csi2_ctrl_config_if_enable(false);
	isp_csi2_ctrl_update(false);

	ov8810_write_reg(client, OV8810_SYS, 0x80);
	mdelay(5);

	/* configure image size, pll, and pixel format */
	if (pix->pixelformat == V4L2_PIX_FMT_SGRBG10) {
		err = ov8810_write_regs(client, ov8810_common[isize]);
		if (err)
			return err;

	} else if (pix->pixelformat == V4L2_PIX_FMT_W1S_PATT) {
		isize = SIZE_8M;
		err = ov8810_write_regs(client, ov8810_common[isize]);

		err = ov8810_write_reg(client, OV8810_DVP_CTRL08, 0x80);
		if (err)
			return err;
	}

	/* Setting of frame rate (OV suggested way) */
	/*err = ov8810_set_framerate(client, &sensor->timeperframe, isize); */

	/* Set CSI2 common register settings */
	err = ov8810_write_regs(client, ov8810_common_csi2);

	sensor->isize = isize;

	/* if the image size correspond to one of the base image sizes
		then we don't need to scale the image */
	sensor->hsize = pix->width;
	sensor->vsize = pix->height;

	/* Setup the ISP VP based on image format */
	if (pix->pixelformat == V4L2_PIX_FMT_SGRBG10) {
		isp_configure_interface_bridge(0x00);
		isp_csi2_ctrl_config_vp_out_ctrl(2);
		isp_csi2_ctrl_update(false);
	} else {
		isp_configure_interface_bridge(0x03);
		isp_csi2_ctrl_config_vp_out_ctrl(1);
		isp_csi2_ctrl_update(false);
	}

	/* Store image size */
	sensor->width = pix->width;
	sensor->height = pix->height;

	/* Update sensor clk and frame parms */
	ov8810_calc_pclk(s);
	ov8810_update_frame_params(s);

	mipiclk = ov8810_calc_mipiclk(s);

	/* Calculate Valid bounds for High speed settle timing in UIs */
	ov8810_read_reg(client, 1, OV8810_MIPI_CTRL14, &val);
	min_hs_zero_nui = ((val & OV8810_MIPI_CTRL14_MIN_HS_ZERO_NUI_MASK) >>
				OV8810_MIPI_CTRL14_MIN_HS_ZERO_NUI_SHIFT);
	min_hs_zero = ((val & OV8810_MIPI_CTRL14_MIN_HS_ZERO_H_MASK) << 8);
	ov8810_read_reg(client, 1, OV8810_MIPI_CTRL15, &val);
	min_hs_zero |= (val & OV8810_MIPI_CTRL15_MIN_HS_ZERO_L_MASK);
	min_hs_zero_total = ((min_hs_zero_nui * 1000000) / (mipiclk / 1000)) +
				min_hs_zero;

	ov8810_read_reg(client, 1, OV8810_MIPI_CTRL24, &val);
	min_hs_prepare_nui = ((val &
				OV8810_MIPI_CTRL24_MIN_HS_PREPARE_NUI_MASK) >>
				OV8810_MIPI_CTRL24_MIN_HS_PREPARE_NUI_SHIFT);
	min_hs_prepare = ((val &
				OV8810_MIPI_CTRL24_MIN_HS_PREPARE_H_MASK) << 8);
	ov8810_read_reg(client, 1, OV8810_MIPI_CTRL25, &val);
	min_hs_prepare |= (val & OV8810_MIPI_CTRL25_MIN_HS_PREPARE_L_MASK);
	min_hs_prepare_total = ((min_hs_prepare_nui * 1000000) /
				(mipiclk / 1000)) + min_hs_prepare;

	ov8810_read_reg(client, 1, OV8810_MIPI_CTRL26, &val);
	max_hs_prepare_nui = ((val &
				OV8810_MIPI_CTRL26_MAX_HS_PREPARE_NUI_MASK) >>
				OV8810_MIPI_CTRL26_MAX_HS_PREPARE_NUI_SHIFT);
	max_hs_prepare = ((val &
				OV8810_MIPI_CTRL26_MAX_HS_PREPARE_H_MASK) << 8);
	ov8810_read_reg(client, 1, OV8810_MIPI_CTRL27, &val);
	max_hs_prepare |= (val & OV8810_MIPI_CTRL27_MAX_HS_PREPARE_L_MASK);
	max_hs_prepare_total = ((max_hs_prepare_nui * 1000000) /
				(mipiclk / 1000)) + max_hs_prepare;

	ubound_hs_settle = ((min_hs_zero_total + min_hs_prepare_total) *
				((mipiclk >> 1) / 1000000)) / 1000;
	lbound_hs_settle = (max_hs_prepare_total * ((mipiclk >> 1) /
				1000000)) / 1000;

	dev_dbg(&client->dev, "OV8810: mipiclk = %d, lbound_hs_settle = %d, " \
		"ubound_hs_settle = %d \n", mipiclk, lbound_hs_settle,
		ubound_hs_settle);

	/* Send settings to ISP-CSI2 Receiver PHY */
	isp_csi2_calc_phy_cfg0(mipiclk, lbound_hs_settle, ubound_hs_settle);

	/* Set sensors virtual channel*/
	ov8810_set_virtual_id(client, OV8810_CSI2_VIRTUAL_ID);

	isp_csi2_ctrl_config_if_enable(true);
	isp_csi2_ctrl_update(false);

	/* Set initial exposure time */
	i = find_vctrl(V4L2_CID_EXPOSURE);
	if (i >= 0) {
		lvc = &video_control[i];
		ov8810_set_exposure_time(lvc->current_value,
			sensor->v4l2_int_device, lvc);
	}

	/* Set initial gain */
	i = find_vctrl(V4L2_CID_GAIN);
	if (i >= 0) {
		lvc = &video_control[i];
		ov8810_set_gain(lvc->current_value,
			sensor->v4l2_int_device, lvc);
	}

	if (pix->pixelformat != V4L2_PIX_FMT_W1S_PATT) {
		/* Set initial color bars */
		i = find_vctrl(V4L2_CID_PRIVATE_COLOR_BAR);
		if (i >= 0) {
			lvc = &video_control[i];
			ov8810_set_color_bar_mode(lvc->current_value,
				sensor->v4l2_int_device, lvc);
		}
	}

	/* Set initial flash mode */
	i = find_vctrl(V4L2_CID_PRIVATE_FLASH_NEXT_FRAME);
	if (i >= 0) {
		lvc = &video_control[i];
		ov8810_set_flash_next_frame(&(sensor->flash),
			sensor->v4l2_int_device, lvc);
	}

	/* start streaming */
	ov8810_write_reg(client, OV8810_IMAGE_SYSTEM, 0x01);

	return err;
}


/* Detect if an ov8810 is present, returns a negative error number if no
 * device is detected, or pidl as version number if a device is detected.
 */
static int ov8810_detect(struct v4l2_int_device *s)
{
	u16 pid, rev;
	u32 val;
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct ov8810_sensor_id *sensor_id = &(sensor->sensor_id);

	if (!client)
		return -ENODEV;

	if (ov8810_read_reg(client, 1, OV8810_PIDH, &val))
		return -ENODEV;
	pid = (val & 0xff) << 8;

	if (ov8810_read_reg(client, 1, OV8810_PIDL, &val))
		return -ENODEV;
	pid |= val & 0xf0;
	rev = val & 0xf;

	if (pid == OV8810_PID) {
		dev_dbg(&client->dev, "OV8810: Detect success " \
			"(pid=0x%x rev=0x%x\n", pid, rev);

		sensor_id->model = pid;
		sensor_id->revision = rev;

		return 0;
	} else {
		/* We didn't read the values we expected, so
		 * this must not be an OV8810.
		 */
		dev_warn(&client->dev, "OV8810: pid mismatch 0x%x rev 0x%x\n",
			pid, rev);

		return -ENODEV;
	}
}

/*
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
						struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;
	return 0;
}

/*
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */

static int ioctl_g_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct vcontrol *lvc;
	struct  ov8810_sensor_params sensor_params;
	int i, retval = 0;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case  V4L2_CID_EXPOSURE:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_GAIN:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_PRIVATE_COLOR_BAR:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_PRIVATE_FLASH_NEXT_FRAME:
		if (copy_to_user((void *)vc->value, &(sensor->flash),
				sizeof(sensor->flash))) {
			retval = -EINVAL;
			dev_err(&client->dev, "Failed copy_to_user\n");
		}
		break;
	case V4L2_CID_PRIVATE_START_MECH_SHUTTER_CAPTURE:
		if (copy_to_user((void *)vc->value, &(sensor->shutter),
				sizeof(sensor->shutter))) {
			retval = -EINVAL;
			dev_err(&client->dev, "Failed copy_to_user\n");
		}
		break;
	case V4L2_CID_PRIVATE_SENSOR_ID_REQ:
		if (copy_to_user((void *)vc->value, &(sensor->sensor_id),
				sizeof(sensor->sensor_id))) {
			retval = -EINVAL;
			dev_err(&client->dev, "Failed copy_to_user\n");
		}
		break;
	case V4L2_CID_PRIVATE_ORIENTATION:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_PRIVATE_SENSOR_REG_REQ:
		vc->value = lvc->current_value;
		{
			dev_dbg(&client->dev, "V4L2_CID_PRIVATE_SENSOR_" \
				"REG_REQ IOCTL not implemented!\n");
			retval = -EINVAL;
		}
		break;
	case V4L2_CID_PRIVATE_SENSOR_PARAMS_REQ:
		sensor_params.line_time = sensor->exposure.line_time;
		sensor_params.gain_frame_delay = OV8810_GAIN_FRAME_DELAY;
		sensor_params.exp_time_frame_delay =
			OV8810_EXP_TIME_FRAME_DELAY;
		if (copy_to_user((void *)vc->value, &(sensor_params),
				sizeof(sensor_params))) {
			printk(KERN_ERR "Failed copy_to_user\n");
			return -EINVAL;
		}
		break;
	}
	return retval;
}

/*
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	int retval = -EINVAL;
	int i;
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct vcontrol *lvc;
	struct ov8810_flash_params flash_params;
	struct ov8810_shutter_params shutter_params;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &video_control[i];

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		retval = ov8810_set_exposure_time(vc->value, s, lvc);
		break;
	case V4L2_CID_GAIN:
		retval = ov8810_set_gain(vc->value, s, lvc);
		break;
	case V4L2_CID_PRIVATE_COLOR_BAR:
		retval = ov8810_set_color_bar_mode(vc->value, s, lvc);
		break;
	case V4L2_CID_PRIVATE_FLASH_NEXT_FRAME:
		if (copy_from_user(&flash_params,
				(struct ov8810_flash_params *)vc->value,
				sizeof(struct ov8810_flash_params)) == 0) {
			retval = ov8810_set_flash_next_frame(&flash_params,
				s, lvc);
		}
		break;
	case V4L2_CID_PRIVATE_START_MECH_SHUTTER_CAPTURE:
		if (copy_from_user(&shutter_params,
				(struct ov8810_shutter_params *)vc->value,
				sizeof(struct ov8810_shutter_params)) == 0) {
			retval = ov8810_start_mech_shutter_capture(
				&shutter_params, s, lvc);
		}
		break;
	case V4L2_CID_PRIVATE_SENSOR_REG_REQ:
		{
			dev_dbg(&client->dev, "V4L2_CID_PRIVATE_SENSOR_" \
				"REG_REQ IOCTL not implemented!\n");
		}
		break;
	}
	if (!retval)
		lvc->current_value = vc->value;
	return retval;
}

/*
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
 static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
				   struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
	break;
	default:
		return -EINVAL;
	}

	fmt->flags = ov8810_formats[index].flags;
	strlcpy(fmt->description, ov8810_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = ov8810_formats[index].pixelformat;

	return 0;
}


/*
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */

static int ioctl_try_fmt_cap(struct v4l2_int_device *s,
			     struct v4l2_format *f)
{
	int ifmt;
	enum image_size_ov isize;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (pix->width > ov8810_sizes[SIZE_8M].width)
		pix->width = ov8810_sizes[SIZE_8M].width;
	if (pix->height > ov8810_sizes[SIZE_8M].height)
		pix->height = ov8810_sizes[SIZE_8M].height;

	isize = ov8810_find_size(s, pix->width, pix->height);
	pix->width = ov8810_sizes[isize].width;
	pix->height = ov8810_sizes[isize].height;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == ov8810_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = ov8810_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width*2;
	pix->sizeimage = pix->bytesperline*pix->height;
	pix->priv = 0;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_SGRBG10:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}
	return 0;
}


/*
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
 static int ioctl_s_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct ov8810_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;

	rval = ioctl_try_fmt_cap(s, f);
	if (rval)
		return rval;

	sensor->pix = *pix;

	return 0;
}

/*
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct ov8810_sensor *sensor = s->priv;
	f->fmt.pix = sensor->pix;

	return 0;
}

/*
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	struct ov8810_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->timeperframe;

	return 0;
}

/*
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	int rval = 0;
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	struct v4l2_fract timeperframe_old;
	int desired_fps;
	timeperframe_old = sensor->timeperframe;
	sensor->timeperframe = *timeperframe;

	desired_fps = timeperframe->denominator / timeperframe->numerator;
	if ((desired_fps < OV8810_MIN_FPS) || (desired_fps > OV8810_MAX_FPS))
		rval = -EINVAL;

	if (rval)
		sensor->timeperframe = timeperframe_old;
	else
		*timeperframe = sensor->timeperframe;

	dev_dbg(&client->dev, "Setting FPS=%d\n", desired_fps);

	return rval;
}

/*
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct ov8810_sensor *sensor = s->priv;

	return sensor->pdata->priv_data_set(p);
}

/*
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
 static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	struct omap34xxcam_hw_config hw_config;
	int rval;

	rval = ioctl_g_priv(s, &hw_config);
	if (rval) {
		dev_err(&c->dev, "Unable to get hw params\n");
		return rval;
	}

	rval = sensor->pdata->power_set(sensor->dev, on);
	if (rval < 0) {
		dev_err(&c->dev, "Unable to set the power state: "
			OV8810_DRIVER_NAME " sensor\n");
		isp_set_xclk(0, OV8810_USE_XCLKA);
		return rval;
	}

	if (on == V4L2_POWER_ON) {
		isp_set_xclk(sensor->clk.xclk, OV8810_USE_XCLKA);
		/* set streaming off, standby */
		ov8810_write_reg(c, OV8810_IMAGE_SYSTEM, 0x00);
	} else {
		isp_set_xclk(0, OV8810_USE_XCLKA);
	}

	if ((current_power_state == V4L2_POWER_STANDBY) &&
			(on == V4L2_POWER_ON) &&
			(sensor->state == SENSOR_DETECTED)) {
		sensor->resuming = true;
		ov8810_configure(s);
	}

	if ((on == V4L2_POWER_ON) && (sensor->state == SENSOR_NOT_DETECTED)) {

		rval = ov8810_detect(s);
		if (rval < 0) {
			dev_err(&c->dev, "Unable to detect "
					OV8810_DRIVER_NAME " sensor\n");
			sensor->state = SENSOR_NOT_DETECTED;
			return rval;
		}
		sensor->state = SENSOR_DETECTED;
		pr_info(OV8810_DRIVER_NAME " Chip version 0x%02x detected\n",
			sensor->sensor_id.revision);
	}

	sensor->resuming = false;
	current_power_state = on;
	return 0;
}

/*
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call ov8810_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach.  The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.  Returns 0 if
 * ov8810 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct ov8810_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	int err;

	err = ov8810_detect(s);
	if (err < 0) {
		dev_err(&c->dev, "Unable to detect " OV8810_DRIVER_NAME
			" sensor\n");
		return err;
	}

	pr_info(OV8810_DRIVER_NAME " chip version 0x%02x detected\n",
		sensor->sensor_id.revision);

	return 0;
}

/**
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 **/
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
					struct v4l2_frmsizeenum *frms)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frms->pixel_format == ov8810_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= OV_NUM_IMAGE_SIZES)
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = ov8810_sizes[frms->index].width;
	frms->discrete.height = ov8810_sizes[frms->index].height;

	return 0;
}

const struct v4l2_fract ov8810_frameintervals[] = {
	{ .numerator = 3, .denominator = 15 },
	{ .numerator = 1, .denominator = 21 },
	{ .numerator = 1, .denominator = 30 },
};

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					struct v4l2_frmivalenum *frmi)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frmi->pixel_format == ov8810_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */

	if ((frmi->width == ov8810_sizes[3].width) &&
				(frmi->height == ov8810_sizes[3].height)) {
		/* The only framerate supported by SIZE_8M capture is 5 fps
		 */
		if (frmi->index != 0)
			return -EINVAL;
	} else if ((frmi->width == ov8810_sizes[2].width) &&
				(frmi->height == ov8810_sizes[2].height)) {
		/* The only framerates supported by SIZE_2M capture are
		 * 5 & 21 fps
		 */
		if (frmi->index > 1)
			return -EINVAL;
	} else {
		if (frmi->index >= 3)
			return -EINVAL;
	}

	frmi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	frmi->discrete.numerator =
				ov8810_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				ov8810_frameintervals[frmi->index].denominator;

	return 0;
}

static struct v4l2_int_ioctl_desc ov8810_ioctl_desc[] = {
	{vidioc_int_enum_framesizes_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
	{vidioc_int_dev_init_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_exit},
	{vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_priv_num,
	  (v4l2_int_ioctl_func *)ioctl_g_priv},
	{vidioc_int_init_num,
	  (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
	{vidioc_int_try_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_try_fmt_cap},
	{vidioc_int_g_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_s_parm},
	{vidioc_int_queryctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_queryctrl},
	{vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_s_ctrl},
/*
	{ vidioc_int_g_crop_num,
	  (v4l2_int_ioctl_func *)ioctl_g_crop},
	{vidioc_int_s_crop_num,
	  (v4l2_int_ioctl_func *)ioctl_s_crop},
	{ vidioc_int_cropcap_num,
	  (v4l2_int_ioctl_func *)ioctl_cropcap},
*/
};

static struct v4l2_int_slave ov8810_slave = {
	.ioctls		= ov8810_ioctl_desc,
	.num_ioctls	= ARRAY_SIZE(ov8810_ioctl_desc),
};

static struct v4l2_int_device ov8810_int_device = {
	.module	= THIS_MODULE,
	.name	= OV8810_DRIVER_NAME,
	.priv	= &ov8810,
	.type	= v4l2_int_type_slave,
	.u	= {
		.slave = &ov8810_slave,
	},
};

int ov8810_strobe_manual_ready(void)
{
    return ov8810_write_regs(ov8810_i2c_client, ov8810_strobe_ready_reg);
}


/*
 * ov8810_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int __init
ov8810_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ov8810_sensor *sensor = &ov8810;
	int err;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	sensor->pdata = client->dev.platform_data;

	if (!sensor->pdata) {
		dev_err(&client->dev, "No platform data?\n");
		return -ENODEV;
	}

	sensor->v4l2_int_device = &ov8810_int_device;
	sensor->i2c_client = client;
	sensor->dev = &client->dev;

	i2c_set_clientdata(client, sensor);

	/* Set sensor default values */
	sensor->pix.width = ov8810_sizes[SIZE_500K].width;
	sensor->pix.height = ov8810_sizes[SIZE_500K].height;
	sensor->pix.pixelformat = V4L2_PIX_FMT_SGRBG10;

	/* Set min/max limits */
	/* TODO - turn on this section
	sensor->min_exposure_time = OV8810_DEF_MIN_EXPOSURE;
	sensor->fps_max_exposure_time = 33333;
	sensor->abs_max_exposure_time =OV8810_DEF_MAX_EXPOSURE;
	sensor->min_linear_gain = OV8810_MIN_LINEAR_GAIN;
	sensor->max_linear_gain = OV8810_MAX_LINEAR_GAIN;
	*/

	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err)
		i2c_set_clientdata(client, NULL);

       ov8810_i2c_client = client;
	return 0;
}

/*
 * ov8810_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device. Complement of ov8810_probe().
 */
static int __exit
ov8810_remove(struct i2c_client *client)
{
	struct ov8810_sensor *sensor = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ov8810_id[] = {
	{ OV8810_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ov8810_id);

static struct i2c_driver ov8810sensor_i2c_driver = {
	.driver = {
		.name	= OV8810_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe	= ov8810_probe,
	.remove	= __exit_p(ov8810_remove),
	.id_table = ov8810_id,
};

/*
 * ov8810sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init ov8810sensor_init(void)
{
	int err;

	err = i2c_add_driver(&ov8810sensor_i2c_driver);
	if (err) {
		printk(KERN_ERR "Failed to register" OV8810_DRIVER_NAME ".\n");
		return err;
	}
	return 0;
}
late_initcall(ov8810sensor_init);

/*
 * ov8810sensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of ov8810sensor_init.
 */
static void __exit ov8810sensor_cleanup(void)
{
	i2c_del_driver(&ov8810sensor_i2c_driver);
}
module_exit(ov8810sensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OV8810 camera sensor driver");

