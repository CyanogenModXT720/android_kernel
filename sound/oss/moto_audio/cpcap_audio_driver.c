/*
 * Copyright (C) 2007 - 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 *
 * Revision History:
 *
 * Date         Author    Comment
 * ----------   --------  ---------------------------
 * 12/18/2007   Motorola  Initial Version
 * 01/23/2009   Motorola  Updates for BT mono
 * 02/21/2009   Motorola  Fix for power management
 * 02/26/2009   Motorola  Updates to use Sierra APIs
 * 03/16/2009   Motorola  Update to use resource framework API for
 *                        power management
 * 04/11/2009   Motorola  Fix for headset balance
 *
 */

/*===================================================================
			INCLUDE FILES
=====================================================================*/

#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/smp_lock.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include "cpcap_audio_driver.h"
#include <linux/spi/cpcap.h>
#include <mach/resource.h>
#include <linux/regulator/consumer.h>

/*=====================================================================
			CONSTANTS
=======================================================================*/
/********************************************/
/* Product Type Checks                      */
/********************************************/
#define HWCFG_PRODUCT_FILE "/sys/mothwcfg/Chosen@0/product"
/* the product ID is the top 24 bits of a unsigned int */
#define HWCFG_PRODUCT_ID_MASK 0xFFFFFF00
#define HWCFG_PRODUCT_ID_KEYWEST 0x0B00
#define HWCFG_PRODUCT_ID_ICELAND 0x0D00

/********************************************/
/* Audio controls                           */
/********************************************/
/* Which bias speed are we using? set to slow or fast to set delay and bias speed bit */
#define SLEEP_ACTIVATE_POWER SLEEP_ACTIVATE_POWER_SLOW

/* do we reset the codec before turning it on?
 * (if false, reset and activate in same write) */
#define RESET_CODEC_BEFORE_ACTIVATION FALSE

/********************************************/
/* Audio sequence delays in milliseconds:   */
/********************************************/
/* ... after enabling audio bias, before configuring anything,
 * for fast and slow bias speed */
#define SLEEP_ACTIVATE_POWER_SLOW 100
#define SLEEP_ACTIVATE_POWER_FAST 2

#define CLOCK_TREE_RESET_TIME 1

#define SLEEP_CODEC_STDAC_PGA_SWITCH 10

#define SLEEP_OUTPUT_AMPS_ENABLE 10

/* ... after enabling phone codec or st_dac, for "data converter to settle" */
#define SLEEP_CODEC_ENABLE 15

/* ... after sending cdc_reset, before turning on codec */
#define SLEEP_DF_RESET_ON_CODEC_ACTIVATION 10

/* ... before energizing EMU headset or un-muting carkit */
#define SLEEP_EMU_DEVICE_ON 3

/* ... after de-energizing EMU headset or muting carkit */
#define SLEEP_EMU_DEVICE_OFF 20

/* ... after disabling ext loudspeaker amp before turning off atlas output */
#define SLEEP_DISABLE_EXT_LOUDSPEAKER 1

/* ... after turning on atlas output before enabling ext loudspeaker amp */
#define SLEEP_ENABLE_EXT_LOUDSPEAKER 1

/* This flag is to bypass Sierra and call SPI functions directly */
/* #define BYPASS_SIERRA 1 */
/* #define AUDIOIC_DEBUG */

/*=======================================================================
				MACROS
=========================================================================*/
#ifdef AUDIOIC_DEBUG
#define AUDIOIC_DEBUG_LOG(args...)  printk(KERN_INFO "AUDIOIC_DRIVER:" args)
#else
#define AUDIOIC_DEBUG_LOG(args...)
#endif

#define AUDIOIC_ERROR_LOG(args...)  \
			printk(KERN_ERR "AUDIOIC_DRIVER: Error " args)

#define ERROR_EXIT _err
#define TRY(a)  if (unlikely(a)) goto ERROR_EXIT;

/*---------------------------------------
  checks for various state transitions
  ---------------------------------------*/
#define SPEAKER_TURNING_OFF() \
	((previous_state->codec_primary_speaker != AUDIOIC_OUT_NONE && \
	state->codec_primary_speaker == AUDIOIC_OUT_NONE) || \
	(previous_state->codec_secondary_speaker != AUDIOIC_OUT_NONE && \
	state->codec_secondary_speaker == AUDIOIC_OUT_NONE) || \
	(previous_state->stdac_primary_speaker != AUDIOIC_OUT_NONE && \
	state->stdac_primary_speaker == AUDIOIC_OUT_NONE) || \
	(previous_state->stdac_secondary_speaker != AUDIOIC_OUT_NONE && \
	state->stdac_secondary_speaker == AUDIOIC_OUT_NONE) || \
	(previous_state->stdac_primary_speaker != AUDIOIC_OUT_NONE && \
	state->stdac_primary_speaker == AUDIOIC_OUT_NONE) || \
	(previous_state->stdac_secondary_speaker != AUDIOIC_OUT_NONE && \
	state->stdac_secondary_speaker == AUDIOIC_OUT_NONE))

#define TTY_STATE_CHANGED() (state->mode != previous_state->mode && \
				(state->mode == AUDIOIC_MODE_TTY || \
				previous_state->mode == AUDIOIC_MODE_TTY))

/* TTY disables the codec high-pass filters,
 * so a TTY change causes a codec change */
#define CODEC_CHANGED() \
	(state->codec_mode != previous_state->codec_mode || \
	state->codec_rate != previous_state->codec_rate || \
	CODEC_SLOT_CHANGED())
	/* || TTY_STATE_CHANGED()) */

/* "AUX_INTERNAL" has a different timeslot configuration */
#define CODEC_SLOT_CHANGED() \
	(state->microphone != previous_state->microphone && \
	(previous_state->microphone == AUDIOIC_IN_AUX_INTERNAL || \
	state->microphone == AUDIOIC_IN_AUX_INTERNAL))

#define STDAC_CHANGED() (state->stdac_mode != previous_state->stdac_mode || \
			state->stdac_rate != previous_state->stdac_rate)

#define RX_GAIN_CHANGED() (state->output_gain != previous_state->output_gain)

#define MUTE_CHANGED() (state->codec_mute != previous_state->codec_mute || \
			state->stdac_mute != previous_state->stdac_mute || \
			state->analog_source != previous_state->analog_source)

/*---------------------------------------
  checks for various I/O settings
  ---------------------------------------*/
#define OUTPUT_IS_EMU_STEREO() \
	(state->codec_primary_speaker == AUDIOIC_OUT_EXT_BUS_STEREO || \
	state->codec_primary_speaker == AUDIOIC_OUT_EMU_STEREO || \
	state->codec_secondary_speaker == AUDIOIC_OUT_EXT_BUS_STEREO || \
	state->codec_secondary_speaker == AUDIOIC_OUT_EMU_STEREO || \
	state->stdac_primary_speaker == AUDIOIC_OUT_EXT_BUS_STEREO || \
	state->stdac_primary_speaker == AUDIOIC_OUT_EMU_STEREO || \
	state->stdac_secondary_speaker == AUDIOIC_OUT_EXT_BUS_STEREO || \
	state->stdac_secondary_speaker == AUDIOIC_OUT_EMU_STEREO || \
	state->ext_primary_speaker == AUDIOIC_OUT_EXT_BUS_STEREO || \
	state->ext_primary_speaker == AUDIOIC_OUT_EMU_STEREO || \
	state->ext_secondary_speaker == AUDIOIC_OUT_EXT_BUS_STEREO || \
	state->ext_secondary_speaker == AUDIOIC_OUT_EMU_STEREO)

#define OUTPUT_IS_EMU_MONO() \
	(state->codec_primary_speaker == AUDIOIC_OUT_EXT_BUS_MONO || \
	state->codec_primary_speaker == AUDIOIC_OUT_EMU_MONO || \
	state->codec_secondary_speaker == AUDIOIC_OUT_EXT_BUS_MONO || \
	state->codec_secondary_speaker == AUDIOIC_OUT_EMU_MONO || \
	state->stdac_primary_speaker == AUDIOIC_OUT_EXT_BUS_MONO || \
	state->stdac_primary_speaker == AUDIOIC_OUT_EMU_MONO || \
	state->stdac_secondary_speaker == AUDIOIC_OUT_EXT_BUS_MONO || \
	state->stdac_secondary_speaker == AUDIOIC_OUT_EMU_MONO || \
	state->ext_primary_speaker == AUDIOIC_OUT_EXT_BUS_MONO || \
	state->ext_primary_speaker == AUDIOIC_OUT_EMU_MONO || \
	state->ext_secondary_speaker == AUDIOIC_OUT_EXT_BUS_MONO || \
	state->ext_secondary_speaker == AUDIOIC_OUT_EMU_MONO)

#define OUTPUT_IS_EMU_STEREO_HEADSET() \
	(state->codec_primary_speaker == AUDIOIC_OUT_EMU_STEREO || \
	state->codec_secondary_speaker == AUDIOIC_OUT_EMU_STEREO || \
	state->stdac_primary_speaker == AUDIOIC_OUT_EMU_STEREO || \
	state->stdac_secondary_speaker == AUDIOIC_OUT_EMU_STEREO || \
	state->ext_primary_speaker == AUDIOIC_OUT_EMU_STEREO || \
	state->ext_secondary_speaker == AUDIOIC_OUT_EMU_STEREO)

#define OUTPUT_IS_EMU_MONO_HEADSET() \
	(state->codec_primary_speaker == AUDIOIC_OUT_EMU_MONO || \
	state->codec_secondary_speaker == AUDIOIC_OUT_EMU_MONO || \
	state->stdac_primary_speaker == AUDIOIC_OUT_EMU_MONO || \
	state->stdac_secondary_speaker == AUDIOIC_OUT_EMU_MONO || \
	state->ext_primary_speaker == AUDIOIC_OUT_EMU_MONO || \
	state->ext_secondary_speaker == AUDIOIC_OUT_EMU_MONO)

#define OUTPUT_IS_EMU() (OUTPUT_IS_EMU_MONO() || OUTPUT_IS_EMU_STEREO())

#define INPUT_IS_EMU() (state->microphone == AUDIOIC_IN_EXT_BUS)

#define EMU_AUDIO_ACTIVE() (OUTPUT_IS_EMU() || INPUT_IS_EMU())

#define EMU_DEVICE_CHANGED() \
	((state->codec_primary_speaker != \
		previous_state->codec_primary_speaker) || \
	(state->codec_secondary_speaker != \
		previous_state->codec_secondary_speaker) || \
	(state->stdac_primary_speaker != \
		previous_state->stdac_primary_speaker) || \
	(state->stdac_secondary_speaker != \
		previous_state->stdac_secondary_speaker) || \
	(state->microphone != previous_state->microphone))

#define OUTPUT_IS_HEADSET() \
	(state->codec_primary_speaker == AUDIOIC_OUT_STEREO_HEADSET || \
	state->codec_primary_speaker == AUDIOIC_OUT_MONO_HEADSET || \
	state->codec_secondary_speaker == AUDIOIC_OUT_STEREO_HEADSET || \
	state->codec_secondary_speaker == AUDIOIC_OUT_MONO_HEADSET || \
	state->stdac_primary_speaker == AUDIOIC_OUT_STEREO_HEADSET || \
	state->stdac_primary_speaker == AUDIOIC_OUT_MONO_HEADSET || \
	state->stdac_secondary_speaker == AUDIOIC_OUT_STEREO_HEADSET || \
	state->stdac_secondary_speaker == AUDIOIC_OUT_MONO_HEADSET || \
	state->ext_primary_speaker == AUDIOIC_OUT_STEREO_HEADSET || \
	state->ext_primary_speaker == AUDIOIC_OUT_MONO_HEADSET || \
	state->ext_secondary_speaker == AUDIOIC_OUT_STEREO_HEADSET || \
	state->ext_secondary_speaker == AUDIOIC_OUT_MONO_HEADSET)

/* NOTE: This test is only valid if we don't have a case where stdac
 * and codec (or ext) are routing to different outputs
 * for eg: stdac to loudspeaker and codec to BT */
#define OUTPUT_IS_BT_ONLY() \
	((state->codec_primary_speaker == AUDIOIC_OUT_BT_MONO && \
	state->codec_secondary_speaker == AUDIOIC_OUT_NONE) || \
	(state->stdac_primary_speaker == AUDIOIC_OUT_BT_MONO && \
	state->stdac_secondary_speaker == AUDIOIC_OUT_NONE) || \
	(state->ext_primary_speaker == AUDIOIC_OUT_BT_MONO && \
	state->ext_secondary_speaker == AUDIOIC_OUT_NONE))

#define DAI_TURNING_ON() (state->mode == AUDIOIC_MODE_DAI_DOWNLINK  || \
			state->mode == AUDIOIC_MODE_DAI_UPLINK)

#define DAI_TURNING_OFF() \
	((previous_state->mode == AUDIOIC_MODE_DAI_DOWNLINK  || \
	previous_state->mode == AUDIOIC_MODE_DAI_UPLINK) && \
	(state->mode != AUDIOIC_MODE_DAI_DOWNLINK) && \
	(state->mode != AUDIOIC_MODE_DAI_UPLINK))

#define IS_OUTPUT_SOURCE_STEREO() (state->stdac == AUDIOIC_STDAC_ON || \
	state->analog_source == AUDIOIC_ANALOG_SOURCE_STEREO)

/*---------------------------------------
  checks for various device connections
  ---------------------------------------*/
/*all audio devices powered by SwB+ i.e. headsets, PPD - Phone Powered Device */
#define EMU_AUDIO_PPD_CONNECTED() \
	(state->codec_primary_speaker   == AUDIOIC_OUT_EXT_MONO || \
	state->codec_secondary_speaker == AUDIOIC_OUT_EXT_MONO || \
	state->codec_primary_speaker   == AUDIOIC_OUT_EXT_STEREO || \
	state->codec_secondary_speaker == AUDIOIC_OUT_EXT_STEREO || \
	state->stdac_primary_speaker   == AUDIOIC_OUT_EXT_MONO || \
	state->stdac_secondary_speaker == AUDIOIC_OUT_EXT_MONO || \
	state->stdac_primary_speaker   == AUDIOIC_OUT_EXT_STEREO || \
	state->stdac_secondary_speaker == AUDIOIC_OUT_EXT_STEREO)

/* all audio devices activated by UID pulldown i.e. carkits,
 * SPD - Self Powered Device */
#define EMU_AUDIO_SPD_CONNECTED() \
	(state->primary_speaker == AUDIOIC_OUT_EXT_BUS_MONO || \
	state->secondary_speaker == AUDIOIC_OUT_EXT_BUS_MONO || \
	state->primary_speaker == AUDIOIC_OUT_EXT_BUS_STEREO || \
	state->secondary_speaker == AUDIOIC_OUT_EXT_BUS_STEREO)

#define OUTPUT_CHANGED() \
	((state->codec_primary_speaker != \
		previous_state->codec_primary_speaker) || \
	(state->codec_primary_balance != \
		previous_state->codec_primary_balance) || \
	(state->codec_secondary_speaker != \
		previous_state->codec_secondary_speaker) || \
	(state->stdac_primary_speaker != \
		previous_state->stdac_primary_speaker) || \
	(state->stdac_primary_balance != \
		previous_state->stdac_primary_balance) || \
	(state->stdac_secondary_speaker != \
		previous_state->stdac_secondary_speaker) || \
	(state->ext_primary_speaker != \
		previous_state->ext_primary_speaker) || \
	(state->ext_primary_balance != \
		previous_state->ext_primary_balance) || \
	(state->ext_secondary_speaker != \
		previous_state->ext_secondary_speaker))

/*all emu audio devices*/
#define EMU_AUDIO_DEV_CONNECTED() \
	(EMU_AUDIO_PPD_CONNECTED() || EMU_AUDIO_SPD_CONNECTED())

/*all emu devices whose audio mode can change (mono<->stereo)*/
#define EMU_MULTI_MODE_DEV_CONNECTED() \
	((RTIME_MAINT_get_dev_stat(RTIME_MAINT_DEV_EMU_STEREO_HEADSET)\
	== RTIME_MAINT_DEV_STAT_CONN))

/*========================================================================
			LOCAL FUNCTION PROTOTYPES
==========================================================================*/

/*========================================================================
			STRUCTURES AND OTHER TYPEDEFS
==========================================================================*/

enum {
	LOUDSPEAKER_TYPE_MONO_INTERNAL,
		/* "normal" mono loudspeaker on internal amp */
	LOUDSPEAKER_TYPE_MONO_EXTERNAL,
		/* mono loudspeaker on external amp (V2000) */
	LOUDSPEAKER_MONO_STEREO_BREAK,
		/* Add mono types less than this value;
		 * Add stereo types greater than this value! */
	LOUDSPEAKER_TYPE_STEREO_HYBRID,
		/* "normal" stereo loudspeaker: right channel
		 * internal,left-channel external (E398, E1000) */
	LOUDSPEAKER_TYPE_STEREO_HYBRID_INV,
		/* like above, but ext channel is inverted
		 * (Key West) */
	LOUDSPEAKER_TYPE_STEREO_EXTERNAL,
		/* both L and R chanels on external amps
		 * (Iceland) */
	LOUDSPEAKER_TYPE_UNDEFINED /* bumper */
};

#define LOUDSPEAKER_IS_STEREO() \
	(LOUDSPEAKER_MONO_STEREO_BREAK < apal_audioic_get_loudspeaker_type())
#define LOUDSPEAKER_IS_MONO() \
	(LOUDSPEAKER_MONO_STEREO_BREAK > apal_audioic_get_loudspeaker_type())

#ifdef BYPASS_SIERRA
struct KERNEL_REG_ACCESS_T {
	unsigned int reg;
	unsigned short value;
	unsigned short mask;
};
#endif

/*=========================================================================
		       LOCAL VARIABLES
===========================================================================*/
static BOOL emu_device_active = FALSE;
static struct AUDIOIC_STATE_T previous_state_struct = {
	NULL,
	AUDIOIC_MODE_NORMAL,
	AUDIOIC_CODEC_OFF,
	AUDIOIC_CODEC_RATE_8000_HZ,
	AUDIOIC_CODEC_MUTE,
	AUDIOIC_STDAC_OFF,
	AUDIOIC_STDAC_RATE_8000_HZ,
	AUDIOIC_STDAC_MUTE,
	AUDIOIC_ANALOG_SOURCE_OFF,
	AUDIOIC_OUT_NONE,
	AUDIOIC_OUT_NONE,
	AUDIOIC_OUT_NONE,
	AUDIOIC_OUT_NONE,
	AUDIOIC_OUT_NONE,
	AUDIOIC_OUT_NONE,
	AUDIOIC_BALANCE_NEUTRAL,
	AUDIOIC_BALANCE_NEUTRAL,
	AUDIOIC_BALANCE_NEUTRAL,
	0,			/* output gain */
	AUDIOIC_IN_NONE,
	0,			/* input_gain */
	AUDIOIC_RAT_NONE,
	255			/* CPCAP_ACCY_EMU_INVALID */
};

/* Define regulator to turn on the audio portion of cpcap */
struct regulator *audio_reg;

/*===========================================================================
			GLOBAL VARIABLES
=============================================================================*/

/*===========================================================================
			LOCAL FUNCTIONS
=============================================================================*/
/*===========================================================================
FUNCTION: cpcap_write

DESCRIPTION:
	This function is used to write to the CPCAP

ARGUMENTS PASSED:
	None

RETURN VALUE:
	None

PRE-CONDITIONS:
	None

POST-CONDITIONS:
	None

IMPORTANT NOTES:
	None

=============================================================================*/
#ifdef BYPASS_SIERRA
static void cpcap_write(
	unsigned int reg,
	unsigned short value,
	unsigned short mask)
{
	int ret_val = 0;
	int reg_val = 0;
	unsigned short untouched_bits = 0;

	ret_val |= cpcap_spi_read((int)reg, &reg_val);

	if (ret_val == 0)
		untouched_bits = reg_val & (~mask);

	cpcap_spi_write((int)reg, (mask & value) | untouched_bits);

	return;
}
#endif

/*===========================================================================
FUNCTION: audioic_get_loudspeaker_type

DESCRIPTION:
	This function is used to determine the loudspeaker configuration on
	current hardware

ARGUMENTS PASSED:
	None

RETURN VALUE:
	AUDIO_LOUDSPEAKER_TYPE reflecting the current hardware configuration

PRE-CONDITIONS:
	None

POST-CONDITIONS:
	None

IMPORTANT NOTES:

==============================================================================*/
static int audioic_get_loudspeaker_type(void)
{
#if 0
	static int loudspeaker_type =
	    LOUDSPEAKER_TYPE_UNDEFINED;
	unsigned int product = 0;
	int fd = -1;

	AUDIOIC_DEBUG_LOG("%s() called\n", __func__);

	if (loudspeaker_type == LOUDSPEAKER_TYPE_UNDEFINED) {
		fd = open(HWCFG_PRODUCT_FILE, O_RDONLY);
		if (fd != -1) {
			/* get product type */
			if (read(fd, (void *)&product, sizeof(product)) ==
			    sizeof(product)) {
				switch (product & HWCFG_PRODUCT_ID_MASK) {
				case HWCFG_PRODUCT_ID_KEYWEST:
					loudspeaker_type =
					    LOUDSPEAKER_TYPE_STEREO_HYBRID_INV;
					break;
				case HWCFG_PRODUCT_ID_ICELAND:
					loudspeaker_type =
					    LOUDSPEAKER_TYPE_STEREO_EXTERNAL;
					break;
				default:
					loudspeaker_type =
					    LOUDSPEAKER_TYPE_MONO_INTERNAL;
					break;
				}
				AUDIOIC_DEBUG_LOG( \
					"Read product ID %x, masked to %x, \
					which has loudspeaker type %d\n", \
					product, \
					(product & HWCFG_PRODUCT_ID_MASK), \
					loudspeaker_type);
			} else {
				goto GET_LOUDSPEAKER_TYPE_ERROR;
			}
		} else {
			goto GET_LOUDSPEAKER_TYPE_ERROR;
		}
	}

	AUDIOIC_DEBUG_LOG("Returning a loudspeaker type of %d\n",
			  loudspeaker_type);
	return loudspeaker_type;

GET_LOUDSPEAKER_TYPE_ERROR:
	AUDIOIC_ERROR_LOG("Failed to determine product ID; \
			defaulting to mono loudspeaker\n");
#endif
	return LOUDSPEAKER_TYPE_MONO_INTERNAL;
}

/*==========================================================================
FUNCTION: audioic_get_codec_output_amp_switches

DESCRIPTION:
   This function determines the Codec output amp switches to be turned on

ARGUMENTS PASSED:
   Speaker for which switch need to be turned on
   balance - left, right or neutral

RETURN VALUE:
   mask of bits to be turned on

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   This function should only be called from audioic_set_output_amp_switches

============================================================================*/
static unsigned short int audioic_get_codec_output_amp_switches(
	int speaker,
	int balance)
{
	unsigned short int value = 0;

	AUDIOIC_DEBUG_LOG("%s() called with speaker = %d\n", __func__,
			  speaker);

	switch (speaker) {
	case AUDIOIC_OUT_HANDSET:
		value = CPCAP_A1_EAR_CDC_SW;
		break;

	case AUDIOIC_OUT_LOUDSPEAKER:
		{
			int loudspeaker_type =
			    audioic_get_loudspeaker_type();
			switch (loudspeaker_type) {
			case LOUDSPEAKER_TYPE_MONO_INTERNAL:
				/* configure mono loudspeaker on internal amp */
				value = CPCAP_A2_LDSP_L_CDC_SW;
				break;
#if 0
			case LOUDSPEAKER_TYPE_MONO_EXTERNAL:
				/* configure mono loudspeaker on external amp
				 * (LSPL) */
				(*message) |=
				    CPCAP_A2_LDSP_R_EN | CPCAP_A2_LDSP_L_EN;
				enable_ext_loudspeaker = TRUE;
				break;

			case LOUDSPEAKER_TYPE_STEREO_HYBRID:
			case LOUDSPEAKER_TYPE_STEREO_HYBRID_INV:
				/*traditional stereo loudspeaker */
				if (balance != AUDIOIC_BALANCE_L_ONLY)
					(*message) |= AUDIOIC_SET_ALSP_EN;
				if (balance != AUDIOIC_BALANCE_R_ONLY) {
					(*message) |= AUDIOIC_SET_LSPL_EN;
					enable_ext_loudspeaker = TRUE;
				}
				break;
#endif
			default:
				/* We dont' know how to configure the speaker
				 * on this phone */
				AUDIOIC_ERROR_LOG
				    ("Unsupported loudspeaker type %u \n",
				     loudspeaker_type);
				break;
			}
		}
		break;

	case AUDIOIC_OUT_EMU_STEREO:
		if (balance != AUDIOIC_BALANCE_R_ONLY)
			value = CPCAP_PGA_OUTR_USBDP_CDC_SW;
		if (balance != AUDIOIC_BALANCE_L_ONLY)
			value |= CPCAP_PGA_OUTL_USBDN_CDC_SW;
		break;

	case AUDIOIC_OUT_EXT_BUS_MONO:
	case AUDIOIC_OUT_EMU_MONO:
		value = CPCAP_PGA_OUTL_USBDN_CDC_SW;
		break;

	case AUDIOIC_OUT_STEREO_HEADSET:
		if (balance != AUDIOIC_BALANCE_L_ONLY)
			value = CPCAP_ARIGHT_HS_CDC_SW;
		if (balance != AUDIOIC_BALANCE_R_ONLY)
			value |= CPCAP_ALEFT_HS_CDC_SW;
		break;

	case AUDIOIC_OUT_MONO_HEADSET:
		value = CPCAP_ARIGHT_HS_CDC_SW | CPCAP_ALEFT_HS_CDC_SW;
		break;

	case AUDIOIC_OUT_LINEOUT:
		value = CPCAP_A4_LINEOUT_R_CDC_SW | CPCAP_A4_LINEOUT_L_CDC_SW;
		break;

	case AUDIOIC_OUT_BT_MONO:
		value = 0;
		break;
#if 0
	case AUDIOIC_OUT_LINEAR_VIBRATOR:
		(*message) |= AUDIOIC_SET_LSPL_EN;
		break;
#endif

	default:
		break;
	}

	value |= CPCAP_PGA_CDC_EN;

	AUDIOIC_DEBUG_LOG("Exiting %s() with return value = %d\n", __func__,
			  value);
	return value;
}

/*============================================================================
FUNCTION: audioic_get_stdac_output_amp_switches

DESCRIPTION:
   This function determines the stdac output amp switches to be turned on

ARGUMENTS PASSED:
   Speaker for which switch need to be turned on
   balance - right, left or neutral

RETURN VALUE:
   mask of bits to be turned on

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   This function should only be called from audioic_set_output_amp_switches

==============================================================================*/
static unsigned short int audioic_get_stdac_output_amp_switches(
	int speaker,
	int balance)
{
	unsigned short int value = 0;
	int loudspeaker_type =
	    audioic_get_loudspeaker_type();

	AUDIOIC_DEBUG_LOG("%s() called with speaker = %d\n", __func__,
			  speaker);

	switch (speaker) {
	case AUDIOIC_OUT_HANDSET:
		value = CPCAP_A1_EAR_DAC_SW;
		break;

	case AUDIOIC_OUT_EMU_STEREO:
		if (balance != AUDIOIC_BALANCE_R_ONLY)
			value = CPCAP_PGA_OUTL_USBDN_DAC_SW;
		if (balance != AUDIOIC_BALANCE_L_ONLY)
			value |= CPCAP_PGA_OUTR_USBDP_DAC_SW;
		break;

	case AUDIOIC_OUT_EXT_BUS_MONO:
	case AUDIOIC_OUT_EMU_MONO:	/* Add L,R -6dB for mono output */
		value =
		    CPCAP_PGA_OUTL_USBDN_DAC_SW | CPCAP_MONO_DAC0 |
		    CPCAP_MONO_DAC1;
		break;

	case AUDIOIC_OUT_STEREO_HEADSET:
		if (balance != AUDIOIC_BALANCE_R_ONLY)
			value = CPCAP_ALEFT_HS_DAC_SW;
		if (balance != AUDIOIC_BALANCE_L_ONLY)
			value |= CPCAP_ARIGHT_HS_DAC_SW;
		break;

		/* @NOTE: For phones with 3.5mm headset, we are calling 3.5mm
		 * stereo headset with mic as
		 * mono headsets. We may need to add a new type for
		 * this later */
	case AUDIOIC_OUT_MONO_HEADSET:
		value = CPCAP_ARIGHT_HS_DAC_SW | CPCAP_ALEFT_HS_DAC_SW;
		break;

	case AUDIOIC_OUT_LOUDSPEAKER:
		switch (loudspeaker_type) {
		case LOUDSPEAKER_TYPE_MONO_INTERNAL:
			/*configure mono loudspeaker on internal amp */
			value =
			    CPCAP_A2_LDSP_L_DAC_SW | CPCAP_MONO_DAC0 |
			    CPCAP_MONO_DAC1;
			break;
#if 0
		case LOUDSPEAKER_TYPE_MONO_EXTERNAL:
			/*configure mono loudspeaker on external amp (LSPL) */
			(*message) |= CPCAP_A2_LDSP_R_EN | CPCAP_A2_LDSP_L_EN;
			enable_ext_loudspeaker = TRUE;
			break;
		case LOUDSPEAKER_TYPE_STEREO_HYBRID:
		case LOUDSPEAKER_TYPE_STEREO_HYBRID_INV:
			/*traditional stereo loudspeaker */
			if (balance != AUDIOIC_BALANCE_L_ONLY)
				(*message) |= AUDIOIC_SET_ALSP_EN;
			if (balance != AUDIOIC_BALANCE_R_ONLY)
				(*message) |= AUDIOIC_SET_LSPL_EN;
				enable_ext_loudspeaker = TRUE;
			break;
#endif
		default:
			/* We dont' know how to configure the speaker on
			 * this phone */
			AUDIOIC_ERROR_LOG("Unsupported loudspeaker type %u\n",
					  loudspeaker_type);
			break;
		}
		break;

	case AUDIOIC_OUT_LINEOUT:
		value = CPCAP_A4_LINEOUT_R_DAC_SW | CPCAP_A4_LINEOUT_L_DAC_SW;
		break;

	case AUDIOIC_OUT_BT_MONO:
		value = 0;
		break;

#if 0
	case AUDIOIC_OUT_LINEAR_VIBRATOR:
		(*message) |= AUDIOIC_SET_LSPL_EN;
		break;
#endif

	default:
		break;
	}

	value |= CPCAP_PGA_DAC_EN;

	AUDIOIC_DEBUG_LOG("Exiting %s() with return value = %d\n", __func__,
			  value);
	return value;
}

/*============================================================================
FUNCTION: audioic_get_ext_output_amp_switches

DESCRIPTION:
   This function determines the external output amp switches to be turned on

ARGUMENTS PASSED:
   Speaker for which switch need to be turned on
   balance - left, right or neutral

RETURN VALUE:
   mask of bits to be turned on

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   This function should only be called from audioic_set_output_amp_switches

==============================================================================*/
static unsigned short int audioic_get_ext_output_amp_switches(
	int speaker,
	int balance)
{
	unsigned short int value = 0;
	int loudspeaker_type =
	    audioic_get_loudspeaker_type();
	AUDIOIC_DEBUG_LOG("%s() called with speaker %d\n", __func__,
			  speaker);

	switch (speaker) {
	case AUDIOIC_OUT_HANDSET:
		if (is_cdma_supported()) {
			/* sholes cdma use ext left PGA, need
			 * to turn on the adder */
			value |= CPCAP_A1_EAR_EXT_SW | CPCAP_MONO_EXT1;
		} else {
			value |= CPCAP_A1_EAR_EXT_SW;
		}
		break;

	case AUDIOIC_OUT_EMU_STEREO:
		if (balance != AUDIOIC_BALANCE_R_ONLY)
			value = CPCAP_PGA_OUTR_USBDP_EXT_SW;
		if (balance != AUDIOIC_BALANCE_L_ONLY)
			value |= CPCAP_PGA_OUTL_USBDN_EXT_SW;
		break;

	case AUDIOIC_OUT_EXT_BUS_MONO:
	case AUDIOIC_OUT_EMU_MONO:
		value =
		    CPCAP_PGA_OUTL_USBDN_EXT_SW | CPCAP_MONO_EXT0 |
		    CPCAP_MONO_EXT1;
		break;

	case AUDIOIC_OUT_STEREO_HEADSET:
		if (balance != AUDIOIC_BALANCE_L_ONLY)
			value = CPCAP_ARIGHT_HS_EXT_SW;
		if (balance != AUDIOIC_BALANCE_R_ONLY)
			value |= CPCAP_ALEFT_HS_EXT_SW;
		break;

		/* @NOTE: For phones with 3.5mm headset, we are calling 3.5mm
		 * stereo headset with mic as mono headsets.
		 * We may need to add a new type for this later */
	case AUDIOIC_OUT_MONO_HEADSET:
		value = CPCAP_ARIGHT_HS_EXT_SW | CPCAP_ALEFT_HS_EXT_SW;
		break;

	case AUDIOIC_OUT_LOUDSPEAKER:
		switch (loudspeaker_type) {
		case LOUDSPEAKER_TYPE_MONO_INTERNAL:
			/*configure mono loudspeaker on internal amp */
			value =
			    CPCAP_A2_LDSP_L_EXT_SW | CPCAP_MONO_EXT0 |
			    CPCAP_MONO_EXT1;
			break;
#if 0
		case LOUDSPEAKER_TYPE_MONO_EXTERNAL:
			/*configure mono loudspeaker on external amp (LSPL) */
			(*message) |= CPCAP_A2_LDSP_R_EN | CPCAP_A2_LDSP_L_EN;
			enable_ext_loudspeaker = TRUE;
			break;

		case LOUDSPEAKER_TYPE_STEREO_HYBRID:
		case LOUDSPEAKER_TYPE_STEREO_HYBRID_INV:
			/*traditional stereo loudspeaker */
			if (balance != AUDIOIC_BALANCE_L_ONLY)
				(*message) |= AUDIOIC_SET_ALSP_EN;
			if (balance != AUDIOIC_BALANCE_R_ONLY) {
				(*message) |= AUDIOIC_SET_LSPL_EN;
				enable_ext_loudspeaker = TRUE;
			}
			break;
#endif
		default:
			/* We dont' know how to configure the
			 * speaker on this phone */
			AUDIOIC_ERROR_LOG("Unsupported loudspeaker type %u\n",
					  loudspeaker_type);
			break;
		}
		break;

	case AUDIOIC_OUT_LINEOUT:
		value = CPCAP_A4_LINEOUT_R_EXT_SW | CPCAP_A4_LINEOUT_L_EXT_SW;
		break;

	case AUDIOIC_OUT_BT_MONO:
		value = 0;
		break;
#if 0
	case AUDIOIC_OUT_LINEAR_VIBRATOR:
		(*message) |= AUDIOIC_SET_LSPL_EN;
		break;
#endif
	default:
		break;
	}

	value |= CPCAP_PGA_EXT_R_EN | CPCAP_PGA_EXT_L_EN;

	AUDIOIC_DEBUG_LOG("Exiting %s() with return value = %d\n", __func__,
			  value);
	return value;
}

/*============================================================================
FUNCTION: audioic_set_output_amp_switches

DESCRIPTION:
	This function switches on the connection between Codec, Stdac
	or External PGA to the output amps.

ARGUMENTS PASSED:
	Pointers to the new (desired) state and the previous (current) state

RETURN VALUE:
	None

PRE-CONDITIONS:
	None

POST-CONDITIONS:
	None

IMPORTANT NOTES:
	This function should only be called from audioic_configure_output

==============================================================================*/
static void audioic_set_output_amp_switches(struct AUDIOIC_STATE_T *state)
{
	static unsigned int codec_prev_settings = 0, stdac_prev_settings =
	    0, ext_prev_settings = 0;
	struct cpcap_regacc reg_changes;
	unsigned short int value1 = 0, value2 = 0;

	AUDIOIC_DEBUG_LOG("%s() called\n", __func__);

	/* First set codec output amp switches */
	if (state->codec_mode != AUDIOIC_CODEC_OFF) {
		value1 =
		    audioic_get_codec_output_amp_switches(state->
					codec_primary_speaker,
					state->codec_primary_balance);
		value2 =
		    audioic_get_codec_output_amp_switches(state->
					codec_secondary_speaker,
					state->codec_primary_balance);

		reg_changes.mask = value1 | value2 | codec_prev_settings;
		reg_changes.value = value1 | value2;
		codec_prev_settings = reg_changes.value;

		AUDIOIC_DEBUG_LOG("RXCOA_value = %#x, mask = %#x\n",
				  reg_changes.value, reg_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_RXCOA,
						reg_changes.value,
						reg_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_RXCOA \
					returned error\n");
			}
		}
#else
		cpcap_write(REG_CPCAP_RXCOA, reg_changes.value,
			    reg_changes.mask);
#endif
	}

	/* Second Stdac switches */
	if (state->stdac_mode != AUDIOIC_STDAC_OFF) {
		value1 =
		    audioic_get_stdac_output_amp_switches(state->
					stdac_primary_speaker,
					state->stdac_primary_balance);
		value2 =
		    audioic_get_stdac_output_amp_switches(state->
					stdac_secondary_speaker,
					state->stdac_primary_balance);

		reg_changes.mask = value1 | value2 | stdac_prev_settings;
		reg_changes.value = value1 | value2;
		stdac_prev_settings = reg_changes.value;

		AUDIOIC_DEBUG_LOG("RXSDOA_value = %#x, mask = %#x\n",
				  reg_changes.value, reg_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_RXSDOA,
						reg_changes.value,
						reg_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_RXSDOA \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_RXSDOA, reg_changes.value,
			    reg_changes.mask);
#endif
	}

	if (state->analog_source != AUDIOIC_ANALOG_SOURCE_OFF) {
		/* Last External source switches */
		value1 =
		    audioic_get_ext_output_amp_switches(state->
					ext_primary_speaker,
					state->ext_primary_balance);
		value2 =
		    audioic_get_ext_output_amp_switches(state->
					ext_secondary_speaker,
					state->ext_primary_balance);

		reg_changes.mask = value1 | value2 | ext_prev_settings;
		reg_changes.value = value1 | value2;
		ext_prev_settings = reg_changes.value;

		AUDIOIC_DEBUG_LOG("RXEPOA_value = %#x, mask = %#x\n",
				  reg_changes.value, reg_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_RXEPOA,
						reg_changes.value,
						reg_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_RXEPOA \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_RXEPOA, reg_changes.value,
			    reg_changes.mask);
#endif
	}
}

/*============================================================================
FUNCTION: audioic_set_bits_for_speaker

DESCRIPTION:
   This function consolidates the logic that is identical for primary and
   secondary speaker configuration

ARGUMENTS PASSED:
   the speaker being configured and its balance
   pointer to the rx configuration message

RETURN VALUE:
   BOOL -- must the external loudspeaker amp be turned on?

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==============================================================================*/
static BOOL audioic_set_bits_for_speaker(
	int speaker,
	int balance,
	unsigned short int *message)
{
	BOOL enable_ext_loudspeaker = FALSE;
	int loudspeaker_type =
	    audioic_get_loudspeaker_type();

	AUDIOIC_DEBUG_LOG("%s() called with speaker = %d\n", __func__,
			  speaker);

	/* Get the data required to enable each possible path */
	switch (speaker) {
	case AUDIOIC_OUT_HANDSET:
		(*message) |= CPCAP_A1_EAR_EN;
		break;

		/* case AUDIOIC_OUT_EXT_BUS_STEREO: */
	case AUDIOIC_OUT_EMU_STEREO:
		if (balance != AUDIOIC_BALANCE_R_ONLY)
			(*message) |= CPCAP_EMU_SPKR_L_EN;
		if (balance != AUDIOIC_BALANCE_L_ONLY)
			(*message) |= CPCAP_EMU_SPKR_R_EN;
		break;

	case AUDIOIC_OUT_EXT_BUS_MONO:
	case AUDIOIC_OUT_EMU_MONO:
		(*message) |= CPCAP_EMU_SPKR_L_EN;
		break;

	case AUDIOIC_OUT_STEREO_HEADSET:
		if (balance != AUDIOIC_BALANCE_R_ONLY)
			(*message) |= CPCAP_HS_L_EN;
		if (balance != AUDIOIC_BALANCE_L_ONLY)
			(*message) |= CPCAP_HS_R_EN;
		break;

		/* @NOTE: For phones with 3.5mm headset, we are calling 3.5mm
		 * stereo headset with mic as mono headsets.
		 * We may need to add a new type for this later */
	case AUDIOIC_OUT_MONO_HEADSET:
		(*message) |= CPCAP_HS_R_EN | CPCAP_HS_L_EN;
		break;

	case AUDIOIC_OUT_LOUDSPEAKER:
		switch (loudspeaker_type) {
		case LOUDSPEAKER_TYPE_MONO_INTERNAL:
			/*configure mono loudspeaker on internal amp */
			(*message) |= CPCAP_A2_LDSP_L_EN;
			break;
#if 0
		case LOUDSPEAKER_TYPE_MONO_EXTERNAL:
			/*configure mono loudspeaker on external amp (LSPL) */
			(*message) |= CPCAP_A2_LDSP_R_EN | CPCAP_A2_LDSP_L_EN;
			enable_ext_loudspeaker = TRUE;
			break;
		case LOUDSPEAKER_TYPE_STEREO_HYBRID:
		case LOUDSPEAKER_TYPE_STEREO_HYBRID_INV:
			/*traditional stereo loudspeaker */
			if (balance != AUDIOIC_BALANCE_L_ONLY)
				(*message) |= AUDIOIC_SET_ALSP_EN;
			if (balance != AUDIOIC_BALANCE_R_ONLY) {
				(*message) |= AUDIOIC_SET_LSPL_EN;
				enable_ext_loudspeaker = TRUE;
			}
			break;
#endif
		default:
			/* We dont' know how to configure the speaker on
			 * this phone */
			AUDIOIC_ERROR_LOG("Unsupported loudspeaker type %u\n",
					  loudspeaker_type);
			break;
		}
		break;

	case AUDIOIC_OUT_LINEOUT:
		(*message) |= CPCAP_A4_LINEOUT_R_EN | CPCAP_A4_LINEOUT_L_EN;
		break;

	case AUDIOIC_OUT_BT_MONO:
		(*message) |= 0;
		break;

#if 0
	case AUDIOIC_OUT_LINEAR_VIBRATOR:
		(*message) |= AUDIOIC_SET_LSPL_EN;
		break;
#endif
	default:
		break;
	}

	return enable_ext_loudspeaker;
}

/*============================================================================
FUNCTION: audioic_configure_aud_mute

DESCRIPTION:
   This function configures audio muting controls

ARGUMENTS PASSED:
   Pointers to the new (desired) state and the previous (current) state

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==============================================================================*/
static void audioic_configure_aud_mute(
	struct AUDIOIC_STATE_T *state,
	struct AUDIOIC_STATE_T *previous_state)
{
	static unsigned int prev_codec_mute_data;
	static unsigned int prev_stdac_mute_data;
	static unsigned int prev_ext_mute_data;

	AUDIOIC_DEBUG_LOG("%s() called\n", __func__);

	if (MUTE_CHANGED()) {
		struct cpcap_regacc codec_changes = { 0 };
		struct cpcap_regacc stdac_changes = { 0 };
		struct cpcap_regacc ext_changes = { 0 };

		if (state->codec_mute == AUDIOIC_CODEC_UNMUTE)
			codec_changes.value |= CPCAP_CDC_SW;
		/* else if (state->codec_mute == AUDIOIC_CODEC_BYPASS_LOOP)
			not yet implemented - no user */

		/* clear old data */
		codec_changes.mask = codec_changes.value | prev_codec_mute_data;

		/* remember new configuration */
		prev_codec_mute_data = codec_changes.value;

		AUDIOIC_DEBUG_LOG("codec_value = %#x, mask = %#x\n",
				  codec_changes.value, codec_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_RXCOA,
						codec_changes.value,
						codec_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_RXCOA \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_RXCOA, codec_changes.value,
			    codec_changes.mask);
#endif

		if (state->stdac_mute == AUDIOIC_STDAC_UNMUTE)
			stdac_changes.value |= CPCAP_ST_DAC_SW;

		/* clear old data */
		stdac_changes.mask = stdac_changes.value | prev_stdac_mute_data;

		/* remember new configuration */
		prev_stdac_mute_data = stdac_changes.value;

		AUDIOIC_DEBUG_LOG("stdac_value = %#x, mask = %#x\n",
				  stdac_changes.value, stdac_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_RXSDOA,
						stdac_changes.value,
						stdac_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("_POWER_IC_REG_CPCAP_RXSDOA \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_RXSDOA, stdac_changes.value,
			    stdac_changes.mask);
#endif

		if (state->analog_source == AUDIOIC_ANALOG_SOURCE_STEREO) {
			ext_changes.value |=
			    CPCAP_PGA_IN_R_SW | CPCAP_PGA_IN_L_SW;
		}

		/* clear old data */
		ext_changes.mask = ext_changes.value | prev_ext_mute_data;

		/* remember new configuration */
		prev_ext_mute_data = ext_changes.value;

		AUDIOIC_DEBUG_LOG("ext_value = %#x, mask = %#x\n",
				  ext_changes.value, ext_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_RXEPOA,
						ext_changes.value,
						ext_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_RXEPOA \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_RXEPOA, ext_changes.value,
			    ext_changes.mask);
#endif
	}

	return;
}

/*===========================================================================
FUNCTION: audioic_configure_codec

DESCRIPTION:
   This function configures the codec

ARGUMENTS PASSED:
   Pointers to the new (desired) state and the previous (current) state

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

=============================================================================*/
static void audioic_configure_codec(
	struct AUDIOIC_STATE_T *state,
	struct AUDIOIC_STATE_T *previous_state)
{
	static unsigned int prev_codec_data;
	static unsigned int prev_cdai_data;

	AUDIOIC_DEBUG_LOG("%s() called\n", __func__);

	/* PHONE CODEC CONFIGURATION */
	if (CODEC_CHANGED()) {
		unsigned int temp_codec_rate = state->codec_rate;
		struct cpcap_regacc cdai_changes = { 0 };
		struct cpcap_regacc codec_changes = { 0 };

		/* We need to turn off codec before changing its settings
		 * NOTE!!! This will cause a discontinuity in the codec
		 * bitclock! */
		if (previous_state->codec_mode != AUDIOIC_CODEC_OFF) {
			codec_changes.mask = prev_codec_data | CPCAP_DF_RESET;

#ifndef BYPASS_SIERRA
			{
				int ret_val;
				ret_val = cpcap_regacc_write(state->cpcap,
							CPCAP_REG_CC,
							codec_changes.value,
							codec_changes.mask);
				if (ret_val != 0) {
					AUDIOIC_ERROR_LOG("\
						POWER_IC_REG_CPCAP_CC \
						returned error, \
						ret_val =%d\n", \
						ret_val);
				}
			}
#else
			cpcap_write(REG_CPCAP_CC, codec_changes.value,
				    codec_changes.mask);
#endif

			prev_codec_data = 0;
			previous_state->codec_mode = AUDIOIC_CODEC_OFF;
		}

		if (is_cdma_supported()) {
			if (state->rat_type == AUDIOIC_RAT_CDMA) {
				/* Need to select CLK input for Telephony
				 * CODEC Multimedia uses CLK_IN0; Incall
				 * uses CLK_IN1 */
				cdai_changes.value |= CPCAP_CLK_IN_SEL;
			}

			/* CDMA sholes is using Normal mode for uplink with
			 * CDC_DIG_AUD_FS0 and F1 being 0 */
			cdai_changes.value |= CPCAP_CDC_PLL_SEL;
		} else {
			/* -Network mode - 4 time slots
			   -Codec is Master of Fsync Bclk
			   -CODEC uses DAI0
			   -CODEC PLL to be used */
			cdai_changes.value =
			    CPCAP_CDC_DIG_AUD_FS0 | CPCAP_CDC_PLL_SEL |
			    CPCAP_MIC2_TIMESLOT0;
		}

		/* Note: DF_RESET bit should be set when
		   -input clock changes OR
		   -sample rate changes OR
		   -AUDOHPF or AUDIHPF changes */
		if (state->rat_type == AUDIOIC_RAT_3G) {
			codec_changes.value |= \
				CPCAP_CDC_CLK0 | CPCAP_DF_RESET;
			/* 15.36 Mhz */
		} else if (state->rat_type == AUDIOIC_RAT_CDMA) {
			/* CDMA case */
			codec_changes.value |= CPCAP_CDC_CLK0 |
						CPCAP_CDC_CLK1 |
						CPCAP_DF_RESET;
			/* 19.2Mhz */
		} else {
			/* if (state->rat_type == AUDIOIC_RAT_2G) OR for
			 * out-of-call cases */
			codec_changes.value |= \
				CPCAP_CDC_CLK2 | CPCAP_DF_RESET;
			/* 26Mhz */
		}

		/* explicitly munge our codec_rate otherwise the compiler
		 * gets confused */
		temp_codec_rate &= 0x0000000F;
		temp_codec_rate = temp_codec_rate << 9;

		switch (state->codec_mode) {
		case AUDIOIC_CODEC_LOOPBACK:
		case AUDIOIC_CODEC_ON:
			codec_changes.value |=
			    (CPCAP_CDC_EN_RX | CPCAP_CDC_CLOCK_TREE_RESET);

			/* @TODO: AUDIPF and AUDOPF not set yet */
			/* @TODO: Activate digital loopback if in
			 * LOOPBACK mode this needs to be done when setting
			 * register 517 Tx inputs */

			/* falling through intentionally */
		case AUDIOIC_CODEC_CLOCK_ONLY:
			codec_changes.value |= temp_codec_rate;
			cdai_changes.value |= CPCAP_CDC_CLK_EN;
			break;

		case AUDIOIC_CODEC_OFF:
		default:
			break;
		}

		/* Clear old settings */
		codec_changes.mask = codec_changes.value | prev_codec_data;
		prev_codec_data = codec_changes.value;

		AUDIOIC_DEBUG_LOG("codec_value = %#x, mask = %#x\n",
				  codec_changes.value, codec_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_CC,
						codec_changes.value,
						codec_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_CC \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_CC, codec_changes.value,
			    codec_changes.mask);
#endif

		/* Wait for CLK_TREE_RESET to finish initializing the clocks */
		msleep(CLOCK_TREE_RESET_TIME);

		cdai_changes.mask = cdai_changes.value | prev_cdai_data;
		prev_cdai_data = cdai_changes.value;

		AUDIOIC_DEBUG_LOG("cdai_value = %#x, mask = %#x\n",
				  cdai_changes.value, cdai_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_CDI,
						cdai_changes.value,
						cdai_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_CDI \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_CDI, cdai_changes.value,
			    cdai_changes.mask);
#endif
	}

	return;
}

/*==========================================================================
FUNCTION: audioic_configure_stdac

DESCRIPTION:
	This function configures the selected output device and the
	remainder of the output audio path

ARGUMENTS PASSED:
	Pointers to the new (desired) state and the previous (current) state

RETURN VALUE:
	None

PRE-CONDITIONS:
	None

POST-CONDITIONS:
	None

IMPORTANT NOTES:
	None

============================================================================*/
static void audioic_configure_stdac(
	struct AUDIOIC_STATE_T *state,
	struct AUDIOIC_STATE_T *previous_state)
{
	static unsigned int prev_stdac_data;
	static unsigned int prev_sdai_data;

	AUDIOIC_DEBUG_LOG("%s() called\n", __func__);

	/* STDAC CONFIGURATION */
	if (STDAC_CHANGED()) {
		unsigned int temp_stdac_rate = state->stdac_rate;
		struct cpcap_regacc sdai_changes = { 0 };
		struct cpcap_regacc stdac_changes = { 0 };

		/* We need to turn off stdac before changing its settings */
		if (previous_state->stdac_mode != AUDIOIC_STDAC_OFF) {
			stdac_changes.mask =
			    prev_stdac_data | CPCAP_DF_RESET_ST_DAC;

#ifndef BYPASS_SIERRA
			{
				int ret_val;
				ret_val = cpcap_regacc_write(state->cpcap,
							CPCAP_REG_SDAC,
							stdac_changes.value,
							stdac_changes.mask);
				if (ret_val != 0) {
					AUDIOIC_ERROR_LOG(" \
						POWER_IC_REG_CPCAP_SDAC \
						returned error, \
						ret_val = %d\n", \
						ret_val);
				}
			}
#else
			cpcap_write(REG_CPCAP_SDAC, stdac_changes.value,
				    stdac_changes.mask);
#endif

			prev_stdac_data = 0;
			previous_state->stdac_mode = AUDIOIC_STDAC_OFF;
		}

		/* -True I2S mode
		   -STDAC is Master of Fsync Bclk
		   -STDAC uses DAI1
		 */
#ifdef AUDIO_I2S_MODE
		sdai_changes.value =
		    CPCAP_DIG_AUD_IN_ST_DAC | CPCAP_ST_DIG_AUD_FS0 |
		    CPCAP_ST_DIG_AUD_FS1;
#else
		sdai_changes.value =
		    CPCAP_ST_DIG_AUD_FS0 | CPCAP_DIG_AUD_IN_ST_DAC |
		    CPCAP_ST_L_TIMESLOT0;
#endif

		/* TODO: Use DAI 0 for now */
		/* sdai_changes.value = CPCAP_ST_DIG_AUD_FS0 | \
					CPCAP_ST_L_TIMESLOT0; */
		/* End TODO: */

		/* Note: DF_RESET bit should be set when
		   -input clock changes OR
		   -sample rate changes OR
		   -AUDOHPF or AUDIHPF changes */
		if (state->rat_type == AUDIOIC_RAT_3G) {
			stdac_changes.value |=
			    CPCAP_ST_DAC_CLK0 | CPCAP_DF_RESET_ST_DAC;
		} else {
			stdac_changes.value |= CPCAP_ST_DAC_CLK2 |
						CPCAP_DF_RESET_ST_DAC;
			/* 26Mhz */
		}

		/* explicitly munge our stdac_rate otherwise the compiler
		 * gets confused */
		temp_stdac_rate &= 0x0000000F;
		temp_stdac_rate = temp_stdac_rate << 4;

		switch (state->stdac_mode) {
		case AUDIOIC_STDAC_ON:
			stdac_changes.value |=
			    (CPCAP_ST_DAC_EN | CPCAP_ST_CLOCK_TREE_RESET);

			/* falling through intentionally */
		case AUDIOIC_STDAC_CLOCK_ONLY:
			stdac_changes.value |= temp_stdac_rate;
			sdai_changes.value |= CPCAP_ST_CLK_EN;
			break;

		case AUDIOIC_STDAC_OFF:
		default:
			break;
		}

		/* Clear old settings */
		stdac_changes.mask = stdac_changes.value | prev_stdac_data;
		prev_stdac_data = stdac_changes.value;

		AUDIOIC_DEBUG_LOG("STDAC_value = %#x, mask = %#x\n",
				  stdac_changes.value, stdac_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_SDAC,
						stdac_changes.value,
						stdac_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_SDAC \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_SDAC, stdac_changes.value,
			    stdac_changes.mask);
#endif

		/* Wait for CLK_TREE_RESET to finish initializing the clocks */
		msleep(CLOCK_TREE_RESET_TIME);

		sdai_changes.mask = sdai_changes.value | prev_sdai_data;
		prev_sdai_data = sdai_changes.value;

		AUDIOIC_DEBUG_LOG("SDAI_value = %#x, mask = %#x\n",
				  sdai_changes.value, sdai_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_SDACDI,
						sdai_changes.value,
						sdai_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_SDACDI \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_SDACDI, sdai_changes.value,
			    sdai_changes.mask);
#endif
	}

	return;
}

/*========================================================================
FUNCTION: audioic_configure_input_gains

DESCRIPTION:
   This function configures input/mic gains

ARGUMENTS PASSED:
   Pointers to the new (desired) state and the previous (current) state

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==========================================================================*/
static void audioic_configure_input_gains(
	struct AUDIOIC_STATE_T *state,
	struct AUDIOIC_STATE_T *previous_state)
{
	AUDIOIC_DEBUG_LOG
	    ("%s() called with state->input_gain = %d, prev->input_gain = %d\n",
	     __func__, state->input_gain, previous_state->input_gain);

	if (state->input_gain != previous_state->input_gain) {
		struct cpcap_regacc reg_changes = { 0 };
		unsigned int temp_input_gain = state->input_gain & 0x0000001F;

		/* @TODO: for now set both the mics have the same gain */
		/* set the gain */
		reg_changes.value |= ((temp_input_gain << 5) | temp_input_gain);

		reg_changes.mask = 0x3FF;

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_TXMP,
						reg_changes.value,
						reg_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_TXMP \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_TXMP, reg_changes.value,
			    reg_changes.mask);
#endif
	}

	return;
}

/*============================================================================
FUNCTION: audioic_configure_output_gains

DESCRIPTION:
   This function configures the output gains

ARGUMENTS PASSED:
   Pointers to the new (desired) state and the previous (current) state

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==============================================================================*/
static void audioic_configure_output_gains(
	struct AUDIOIC_STATE_T *state,
	struct AUDIOIC_STATE_T *previous_state)
{
	AUDIOIC_DEBUG_LOG("%s() called\n", __func__);

	if (state->output_gain != previous_state->output_gain
	    /* TODO: BEGIN HACK: this is a hack to ensure that there is some
	     * minimal gain setting. Right now Audio manager setting 0
	     * for headsets */
	    && state->output_gain != 0) {
		/* END HACK */
		/* if (state->output_gain != previous_state->output_gain) */
		struct cpcap_regacc reg_changes = { 0 };
		unsigned int temp_output_gain = state->output_gain & 0x0000000F;

		/* @TODO: 1. Both Stdac and Codec are using the same gains
		   2. We are not using the 1dB steps at all right now */

		/* set the gain */
		reg_changes.value |=
		    ((temp_output_gain << 2) | (temp_output_gain << 8) |
		     (temp_output_gain << 12));

		reg_changes.mask = 0xFF3C;

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_RXVC,
						reg_changes.value,
						reg_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_RXVC \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_RXVC, reg_changes.value,
			    reg_changes.mask);
#endif
	}

	return;
}

/*============================================================================
FUNCTION: audioic_configure_output

DESCRIPTION:
	This function configures the selected output device and the remainder
	of the output audio path

ARGUMENTS PASSED:
   Pointers to the new (desired) state and the previous (current) state

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==============================================================================*/
static void audioic_configure_output(
	struct AUDIOIC_STATE_T *state,
	struct AUDIOIC_STATE_T *previous_state)
{
	static unsigned int prev_aud_out_data;
	static BOOL ext_loudspeaker_on = FALSE;

	AUDIOIC_DEBUG_LOG("%s() called\n", __func__);

	/* SELECT OUTPUT DEVICE  */
	if (OUTPUT_CHANGED()) {
		BOOL activate_ext_loudspeaker = FALSE;
		struct cpcap_regacc reg_changes = { 0 };

		audioic_set_output_amp_switches(state);

		/* configure codec primary speaker */
		activate_ext_loudspeaker =
		    audioic_set_bits_for_speaker(state->codec_primary_speaker,
						 state->codec_primary_balance,
						 &(reg_changes.value));

		/* configure codec secondary speaker */
		activate_ext_loudspeaker = activate_ext_loudspeaker ||
		    audioic_set_bits_for_speaker(state->codec_secondary_speaker,
						 AUDIOIC_BALANCE_NEUTRAL,
						 &(reg_changes.value));

		/* configure stdac primary speaker */
		activate_ext_loudspeaker =
		    audioic_set_bits_for_speaker(state->stdac_primary_speaker,
						 state->stdac_primary_balance,
						 &(reg_changes.value));

		/* configure stdac secondary speaker */
		activate_ext_loudspeaker = activate_ext_loudspeaker ||
		    audioic_set_bits_for_speaker(state->stdac_secondary_speaker,
						 AUDIOIC_BALANCE_NEUTRAL,
						 &(reg_changes.value));

		/* configure stdac primary speaker */
		activate_ext_loudspeaker =
		    audioic_set_bits_for_speaker(state->ext_primary_speaker,
						 state->ext_primary_balance,
						 &(reg_changes.value));

		/* configure stdac secondary speaker */
		activate_ext_loudspeaker = activate_ext_loudspeaker ||
		    audioic_set_bits_for_speaker(state->ext_secondary_speaker,
						 AUDIOIC_BALANCE_NEUTRAL,
						 &(reg_changes.value));

		if (!activate_ext_loudspeaker && ext_loudspeaker_on) {
			/* turn off ext loudspeaker before rx0 write */
			/* audioic_activate_external_amp (FALSE); */
			/* audioic_sleep (SLEEP_DISABLE_EXT_LOUDSPEAKER); */
		}

		/* make sure that the old audio out is disabled */
		reg_changes.mask = reg_changes.value | prev_aud_out_data;

		/* remember new configuration */
		prev_aud_out_data = reg_changes.value;

		AUDIOIC_DEBUG_LOG("rx0_value = %#x, rx0_mask = %#x\n",
				  reg_changes.value, reg_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_RXOA,
						reg_changes.value,
						reg_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("_POWER_IC_REG_CPCAP_RXOA \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_RXOA, reg_changes.value,
			    reg_changes.mask);
#endif

		if (activate_ext_loudspeaker && !ext_loudspeaker_on) {
			/* turn on ext loudspeaker after rx0 write */
			/* audioic_sleep (SLEEP_ENABLE_EXT_LOUDSPEAKER); */
			/* audioic_activate_external_amp(TRUE); */
		}

		ext_loudspeaker_on = activate_ext_loudspeaker;
	}

	return;
}

/*============================================================================
FUNCTION: audioic_configure_input

DESCRIPTION:
	This function configures the selected input device. It only enables the
	loopback mode bit if
	loopback is turned on.

ARGUMENTS PASSED:
   Pointers to the new (desired) state and the previous (current) state

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==============================================================================*/
static void audioic_configure_input(
	struct AUDIOIC_STATE_T *state,
	struct AUDIOIC_STATE_T *previous_state)
{
	static unsigned int prev_input_data = 0x0;
	static unsigned int prev_codec_data = 0x0;
	struct cpcap_regacc reg_changes = { 0 };
	struct cpcap_regacc codec_changes = { 0 };

	AUDIOIC_DEBUG_LOG
	    ("%s() called with state->microphone = %d, prev->microphone = %d\n",
	     __func__, state->microphone, previous_state->microphone);

	/* Loopback mode bit is provided in this register
	 * so enable that first */
	if (state->codec_mode == AUDIOIC_CODEC_LOOPBACK)
		reg_changes.value |= CPCAP_DLM;

	if (state->microphone != previous_state->microphone) {
		/* HACK: Right now 3GSM modem is not supporting reading
		 * from TS1. The commented out code below is the correct
		 * code but it will cause the codec to be setup for
		 * secondary mic and there will be no audio. To get around
		 * we leave the below code as is. The below code will
		 * fail in setting the TXI register because power-ic API
		 * doesn't allow touching CPCAP_MB_ON2 bit,
		 * so in effect the old setting that was
		 * in handset mode will stay on */
		/* if (state->microphone != AUDIOIC_IN_AUX_INTERNAL) */
			codec_changes.mask = CPCAP_MIC1_CDC_EN;

		/* Previous_state needs to be checked here to switch off
		 * the MIC2_CDC, if it was enabled before */
		if (state->microphone == AUDIOIC_IN_DUAL_INTERNAL ||
		    state->microphone == AUDIOIC_IN_DUAL_EXTERNAL ||
		    state->microphone == AUDIOIC_IN_AUX_INTERNAL ||
		    previous_state->microphone == AUDIOIC_IN_DUAL_INTERNAL ||
		    previous_state->microphone == AUDIOIC_IN_DUAL_EXTERNAL ||
		    previous_state->microphone == AUDIOIC_IN_AUX_INTERNAL) {
			codec_changes.mask |= CPCAP_MIC2_CDC_EN;
		}

		if ((state->microphone != AUDIOIC_IN_NONE) &&
		    (state->microphone != AUDIOIC_IN_BT_MONO)) {
			codec_changes.value = codec_changes.mask;
		}

		codec_changes.mask |= prev_codec_data;
		prev_codec_data = codec_changes.value;

		AUDIOIC_DEBUG_LOG("codec_value = %#x, mask = %#x\n",
				  codec_changes.value, codec_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_CC,
						codec_changes.value,
						codec_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_CC \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_CC, codec_changes.value,
			    codec_changes.mask);
#endif

		/* Get the data for the audio path selected */
		switch (state->microphone) {
		case AUDIOIC_IN_HANDSET:
			reg_changes.value |=
			    CPCAP_MB_ON1R | CPCAP_MIC1_MUX | CPCAP_MIC1_PGA_EN;
			break;

		case AUDIOIC_IN_HEADSET:
			reg_changes.value |=
			    CPCAP_HS_MIC_MUX | CPCAP_MIC1_PGA_EN;
			break;

		case AUDIOIC_IN_EXT_BUS:
			reg_changes.value |=
			    CPCAP_EMU_MIC_MUX | CPCAP_MIC1_PGA_EN;
			break;

		case AUDIOIC_IN_AUX_INTERNAL:
			/* HACK: Right now 3GSM modem is not supporting reading
			 * from TS1. The commented out code below is the correct
			 * code but it will cause the codec to be setup for
			 * secondary mic and there will be no audio. To get
			 * around we leave the below code as is.
			 * The below code will fail in setting the TXI
			 * register because power-ic API doesn't allow
			 * touching CPCAP_MB_ON2 bit, so in effect the old
			 * setting that was in handset mode will stay on */
			/* reg_changes.value |= CPCAP_MB_ON1L |
						CPCAP_MIC2_MUX |
						CPCAP_MIC2_PGA_EN; */
			reg_changes.value |=
			    CPCAP_MB_ON2 | CPCAP_MIC2_MUX | CPCAP_MIC2_PGA_EN;
			break;

		case AUDIOIC_IN_DUAL_INTERNAL:
			reg_changes.value |=
			    CPCAP_MB_ON1R | CPCAP_MIC1_MUX | CPCAP_MIC1_PGA_EN |
			    CPCAP_MB_ON1L | CPCAP_MIC2_MUX | CPCAP_MIC2_PGA_EN;
			break;

		case AUDIOIC_IN_DUAL_EXTERNAL:
			reg_changes.value |=
			    CPCAP_RX_R_ENCODE | CPCAP_RX_L_ENCODE;
			break;

		case AUDIOIC_IN_BT_MONO:
			reg_changes.value = 0;
			break;

		default:
			break;
		}

		reg_changes.mask = reg_changes.value | prev_input_data;
		prev_input_data = reg_changes.value;

		AUDIOIC_DEBUG_LOG("tx_value = %#x, tx_mask = %#x\n",
				  reg_changes.value, reg_changes.mask);

#ifndef BYPASS_SIERRA
		{
			int ret_val;
			ret_val = cpcap_regacc_write(state->cpcap,
						CPCAP_REG_TXI,
						reg_changes.value,
						reg_changes.mask);
			if (ret_val != 0) {
				AUDIOIC_ERROR_LOG("_POWER_IC_REG_CPCAP_TXI \
					returned error, ret_val = %d\n", \
					ret_val);
			}
		}
#else
		cpcap_write(REG_CPCAP_TXI, reg_changes.value, reg_changes.mask);
#endif
	}

	return;
}

/*============================================================================
FUNCTION: audioic_configure_power

DESCRIPTION:
   This function powers on the Audio IC

ARGUMENTS PASSED:
   Pointers to the new (desired) state and the previous (current) state

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

=============================================================================*/
static void audioic_configure_power(
	struct AUDIOIC_STATE_T *state,
	struct AUDIOIC_STATE_T *previous_state,
	BOOL power)
{
	static BOOL previous_power = FALSE;
	/* static struct res_handle *res_vaudio = NULL; */
	int ret_val;

	AUDIOIC_DEBUG_LOG("%s() called with power = %d\n", __func__, power);

	if (power != previous_power) {
		if (power) {
			regulator_enable(audio_reg);
			AUDIOIC_DEBUG_LOG("Enable audio regulator\n");
		} else {
			regulator_disable(audio_reg);
			AUDIOIC_DEBUG_LOG("Disable audio regulator\n");
		}

		/*
		   if(power) {
			res_vaudio = resource_get ("CPCAP Driver", LDO_VAUDIO);
			resource_request(res_vaudio, LDO_VAUDIO_2V775);
			AUDIOIC_DEBUG_LOG ("Turned on Audio power \
				through resource framework API\n");
		   } else {
			resource_release(res_vaudio);
			resource_put(res_vaudio);
			res_vaudio = NULL;
			AUDIOIC_DEBUG_LOG ("Turned off Audio power \
				through resource framework API\n");
		   }
		 */

		previous_power = power;

		/* sleep if we're turning on */
		if (power)
			msleep(SLEEP_ACTIVATE_POWER);
	}

	/* Power supply for headset should be controlled at the beginning
	 * (if enabling) or end (if disabling) of the audioic setting
	 * sequence */
	if (OUTPUT_CHANGED()) {
		struct cpcap_regacc reg_changes = {
			CPCAP_REG_RXOA, 0, CPCAP_ST_HS_CP_EN };

		if (OUTPUT_IS_HEADSET())
			reg_changes.value = reg_changes.mask;
#ifndef BYPASS_SIERRA
		ret_val = cpcap_regacc_write(state->cpcap,
					CPCAP_REG_RXOA,
					reg_changes.value,
					reg_changes.mask);
		if (ret_val != 0) {
			AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_RXOA \
				returned error, ret_val = %d\n",
				ret_val);
		}
#else
		cpcap_write(REG_CPCAP_RXOA, reg_changes.value,
			    reg_changes.mask);
#endif

		/* If the output type is BT only then put the IC in
		 * lower power mode */
		reg_changes.mask = CPCAP_AUDIO_LOW_PWR | CPCAP_AUD_LOWPWR_SPEED;
		reg_changes.value = 0;

		if (OUTPUT_IS_BT_ONLY()) {
#if (SLEEP_ACTIVATE_POWER == SLEEP_ACTIVATE_POWER_FAST)
			/* CPCAP_AUD_LOWPWR_SPEED must be set if we're
			 * using fast-ramp */
			reg_changes.value = reg_changes.mask;
#else
			reg_changes.value = CPCAP_AUDIO_LOW_PWR;
#endif
		}

		AUDIOIC_DEBUG_LOG("512_value = %#x, 512_mask = %#x\n",
				  reg_changes.value, reg_changes.mask);
		ret_val = cpcap_regacc_write(state->cpcap,
					CPCAP_REG_VAUDIOC,
					reg_changes.value,
					reg_changes.mask);
		if (ret_val != 0) {
			AUDIOIC_ERROR_LOG("POWER_IC_REG_CPCAP_VAUDIOC \
				returned error, ret_val = %d\n",
				ret_val);
		}
	}

	return;
}

/*=========================================================================
FUNCTION: audioic_configure_emu_device

DESCRIPTION:
	This function energizes the emu headset or un-mutes the emu carkit
	if necessary, and changes mode

ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   EMU audio accessory is active

IMPORTANT NOTES:
   None

===========================================================================*/
static void audioic_configure_emu_device(
	struct AUDIOIC_STATE_T *state,
	struct AUDIOIC_STATE_T *previous_state)
{
	AUDIOIC_DEBUG_LOG("%s() called\n", __func__);

	if (INPUT_IS_EMU() || OUTPUT_IS_EMU()) {
		if (EMU_DEVICE_CHANGED() || (!emu_device_active)) {
#if 0 /* comment out as accy detection not ready yet */
			unsigned char emu_mode = CPCAP_ACCY_MODE_HS_OFF;
			unsigned char accy_type = CPCAP_ACCY__NUM_DEVICES;
			int ret_val = 0;

			/* Output is EMU stereo headset */
			if (OUTPUT_IS_EMU_STEREO_HEADSET()) {
				AUDIOIC_DEBUG_LOG("Turning on EMU_STEREO\n");
				accy_type = state->emu_accy;
				/* MOTO_ACCY_TYPE_EMU_HEADSET_STEREO; */
				emu_mode = CPCAP_ACCY_MODE_HS_STEREO;

			}
			/* Output is EMU mono headset */
			else if (OUTPUT_IS_EMU_MONO_HEADSET() ||
				 (state->microphone == AUDIOIC_IN_EMU &&
				  (state->emu_accy == CPCAP_ACCY_EMU_HS_STEREO
				   || state->emu_accy ==
				   CPCAP_ACCY_EMU_HS_MONO))) {
				AUDIOIC_DEBUG_LOG("Turning on EMU_MONO\n");
				accy_type = state->emu_accy;
				/* CPCAP_ACCY_EMU_HS_MONO; */
				emu_mode = CPCAP_ACCY_MODE_HS_MONO;
			} else {
				AUDIOIC_DEBUG_LOG("No EMU device found\n");
			}

			ret_val =
				moto_accy_change_accy_mode(accy_type, emu_mode);

			/* apal_audioic_sleep(SLEEP_EMU_DEVICE_ON); */

			if (ret_val == 0) {
				AUDIOIC_DEBUG_LOG
				    ("EMU accessory power turned on\n");

				emu_device_active = TRUE;
			} else {
				AUDIOIC_ERROR_LOG("EMU accessory power \
					could not be turned on\n");
			}
#endif
		}
	}
}

/*============================================================================
FUNCTION: audioic_deactivate_emu_device

DESCRIPTION:
   This function de-energizes the emu headset and sets the emu mode to non-audio
   or mutes the emu carkit if necessary

ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   EMU headset is de-energized and EMU mode is non-audio

IMPORTANT NOTES:
   None

==============================================================================*/
static void audioic_deactivate_emu_device(
	struct AUDIOIC_STATE_T *state,
	struct AUDIOIC_STATE_T *previous_state)
{
	AUDIOIC_DEBUG_LOG("%s() called\n", __func__);

	if (emu_device_active) {
		int ret_val = 0;

		emu_device_active = FALSE;

		/* ret_val = moto_accy_change_accy_mode (state->emu_accy,
				CPCAP_ACCY_MODE_HS_OFF); */

		if (ret_val != 0)
			AUDIOIC_ERROR_LOG("Error inside %s()\n", __func__);
		else
			AUDIOIC_DEBUG_LOG("EMU power turned off\n");
	}
}

/*============================================================================
FUNCTION: audioic_state_dump

DESCRIPTION:
   This function dumps the state information

ARGUMENTS PASSED:
   AUDIOIC_STATE_T* describing desired audio path configuration

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==============================================================================*/
static void audioic_state_dump(struct AUDIOIC_STATE_T *state)
{
	AUDIOIC_DEBUG_LOG("***************************************\n");
	AUDIOIC_DEBUG_LOG("state->mode = %d\n", state->mode);
	AUDIOIC_DEBUG_LOG("state->codec_mode = %d\n", state->codec_mode);
	AUDIOIC_DEBUG_LOG("state->codec_rate = %d\n", state->codec_rate);
	AUDIOIC_DEBUG_LOG("state->codec_mute = %d\n", state->codec_mute);
	AUDIOIC_DEBUG_LOG("state->stdac_mode = %d\n", state->stdac_mode);
	AUDIOIC_DEBUG_LOG("state->stdac_rate = %d\n", state->stdac_rate);
	AUDIOIC_DEBUG_LOG("state->stdac_mute = %d\n", state->stdac_mute);
	AUDIOIC_DEBUG_LOG("state->analog_source = %d\n", state->analog_source);
	AUDIOIC_DEBUG_LOG("state->codec_primary_speaker = %d\n",
			  state->codec_primary_speaker);
	AUDIOIC_DEBUG_LOG("state->stdac_primary_speaker = %d\n",
			  state->stdac_primary_speaker);
	AUDIOIC_DEBUG_LOG("state->ext_primary_speaker = %d\n",
			  state->ext_primary_speaker);
	AUDIOIC_DEBUG_LOG("state->codec_secondary_speaker = %d\n",
			  state->codec_secondary_speaker);
	AUDIOIC_DEBUG_LOG("state->stdac_secondary_speaker = %d\n",
			  state->stdac_secondary_speaker);
	AUDIOIC_DEBUG_LOG("state->ext_secondary_speaker   = %d\n",
			  state->ext_secondary_speaker);
	AUDIOIC_DEBUG_LOG("state->stdac_primary_balance   = %d\n",
			  state->stdac_primary_balance);
	AUDIOIC_DEBUG_LOG("state->ext_primary_balance     = %d\n",
			  state->ext_primary_balance);
	AUDIOIC_DEBUG_LOG("state->output_gain             = %d\n",
			  state->output_gain);
	AUDIOIC_DEBUG_LOG("state->microphone              = %d\n",
			  state->microphone);
	AUDIOIC_DEBUG_LOG("state->input_gain              = %d\n",
			  state->input_gain);
	AUDIOIC_DEBUG_LOG("state->rat_type                = %d\n",
			  state->rat_type);
	AUDIOIC_DEBUG_LOG("***************************************\n");
}

/*============================================================================
FUNCTION: audioic_register_dump

DESCRIPTION:
   This function dumps the state information

ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==============================================================================*/
static void audioic_register_dump(struct AUDIOIC_STATE_T *state)
{
	unsigned short reg_val = 0;

#ifdef BYPASS_SIERRA
	int ret_val = 0;

	ret_val |= cpcap_spi_read(0x201, &reg_val);
	printk(KERN_INFO "0x201 = %x\n", reg_val);

	ret_val |= cpcap_spi_read(0x202, &reg_val);
	printk(KERN_INFO "0x202 = %x\n", reg_val);

	ret_val |= cpcap_spi_read(0x203, &reg_val);
	printk(KERN_INFO "0x203 = %x\n", reg_val);

	ret_val |= cpcap_spi_read(0x204, &reg_val);
	printk(KERN_INFO "0x204 = %x\n", reg_val);

	ret_val |= cpcap_spi_read(0x205, &reg_val);
	printk(KERN_INFO "0x205 = %x\n", reg_val);

	ret_val |= cpcap_spi_read(0x206, &reg_val);
	printk(KERN_INFO "0x206 = %x\n", reg_val);

	ret_val |= cpcap_spi_read(0x207, &reg_val);
	printk(KERN_INFO "0x207 = %x\n", reg_val);

	ret_val |= cpcap_spi_read(0x208, &reg_val);
	printk(KERN_INFO "0x208 = %x\n", reg_val);

	ret_val |= cpcap_spi_read(0x209, &reg_val);
	printk(KERN_INFO "0x209 = %x\n", reg_val);

	ret_val |= cpcap_spi_read(0x20A, &reg_val);
	printk(KERN_INFO "0x20A = %x\n", reg_val);

	ret_val |= cpcap_spi_read(0x20B, &reg_val);
	printk(KERN_INFO "0x20B = %x\n", reg_val);
#else
	cpcap_regacc_read(state->cpcap, CPCAP_REG_VAUDIOC, &reg_val);
	printk(KERN_INFO "0x200 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_CC, &reg_val);
	printk(KERN_INFO "0x201 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_CDI, &reg_val);
	printk(KERN_INFO "0x202 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_SDAC, &reg_val);
	printk(KERN_INFO "0x203 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_SDACDI, &reg_val);
	printk(KERN_INFO "0x204 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_TXI, &reg_val);
	printk(KERN_INFO "0x205 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_TXMP, &reg_val);
	printk(KERN_INFO "0x206 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_RXOA, &reg_val);
	printk(KERN_INFO "0x207 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_RXVC, &reg_val);
	printk(KERN_INFO "0x208 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_RXCOA, &reg_val);
	printk(KERN_INFO "0x209 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_RXSDOA, &reg_val);
	printk(KERN_INFO "0x20A = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_RXEPOA, &reg_val);
	printk(KERN_INFO "0x20B = %x\n", reg_val);
#endif
}

/*============================================================================
				GLOBAL FUNCTIONS
==============================================================================*/
/*============================================================================
FUNCTION: AUDIOIC_set_audio_state

DESCRIPTION:
   This function is used to set the audio path configuration inside the
   Audio IC.

ARGUMENTS PASSED:
   AUDIOIC_STATE_T* describing desired audio path configuration

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   audio path has been re-configured.  State structure will be altered to
   reflect implicit controls.

IMPORTANT NOTES:
   The following state relations are implicit:
     1) AUDIOIC_PHONE_CODEC_BYPASS_LOOP implies AUDIOIC_PHONE_CODEC_ON
     2) _PHONE_CODEC_OFF or _CLK_ONLY implies AUDIOIC_PHONE_CODEC_MUTE
	(unless superceded by above)
     3) AUDIOIC_STDAC_OFF or _CLK_ONLY implies AUDIOIC_STDAC_MUTE
     4) STDAC_CLK_ONLY imples STDAC_ON so we don't have to worry about
	_CLK_ONLY <-> ON transition
     5) EMU input implies EMU stereo output becoming EMU mono output
	(only two wires for audio)

==============================================================================*/
void AUDIOIC_set_audio_state(struct AUDIOIC_STATE_T *state)
{
	struct AUDIOIC_STATE_T *previous_state = &previous_state_struct;

	AUDIOIC_DEBUG_LOG("%s() called\n", __func__);

	audioic_state_dump(state);

	/*-------------------------------------------------
	 * first enforce requirements from "important notes"
	 *-------------------------------------------------*/
	if (state->codec_mute == AUDIOIC_CODEC_BYPASS_LOOP) {
		/* see note 1 */
		state->codec_mode = AUDIOIC_CODEC_ON;
	}

	if (state->codec_mode == AUDIOIC_CODEC_OFF ||
	    state->codec_mode == AUDIOIC_CODEC_CLOCK_ONLY) {
		/* see note 2 */
		state->codec_mute = AUDIOIC_CODEC_MUTE;
	} else {
		state->codec_mute = AUDIOIC_CODEC_UNMUTE;
	}

	if (state->stdac_mode != AUDIOIC_STDAC_ON) {
		/* see note 3 */
		state->stdac_mute = AUDIOIC_STDAC_MUTE;
	} else {
		state->stdac_mute = AUDIOIC_STDAC_UNMUTE;
	}

	if (state->stdac_mode == AUDIOIC_STDAC_CLOCK_ONLY) {
		/* see note 4 */
		state->stdac_mode = AUDIOIC_STDAC_ON;
	}

	if (INPUT_IS_EMU()) {
		/* see note 5 */
		if (state->codec_primary_speaker == AUDIOIC_OUT_EXT_BUS_STEREO
		    || state->codec_primary_speaker == AUDIOIC_OUT_EMU_STEREO) {
			state->codec_primary_speaker = AUDIOIC_OUT_EXT_BUS_MONO;
		}
		if (state->codec_secondary_speaker == AUDIOIC_OUT_EXT_BUS_STEREO
		    || state->codec_secondary_speaker ==
		    AUDIOIC_OUT_EMU_STEREO) {
			state->codec_secondary_speaker =
			    AUDIOIC_OUT_EXT_BUS_MONO;
		}
	}

	/*-------------------------------------------------
	  now begin actual audio sequencing:
	  generally speaking, audio must always be turned on from inside ->
	  outside, and turned off from outside -> inside.  this leaves the
	  following logical sequence:
	  0) activate audio power
	  1) turn off emu device
	  2) turn off audio output
	  3) turn off gain/mute/misc
	  4) change digital configuration
	  5) turn on gain/mute/misc
	  6) turn on audio output
	  7) turn on emu device
	  8) deactivate audio power

	  actions get more expensive the closer they are to the ends, so we
	  try to keep the sequence as close to the middle as we can.
	  for example:
	  when switching from codec to st_dac execute 3,4,5
	  when completely switching modes, execute 1->7
	  -------------------------------------------------*/

	if (state->codec_mode != AUDIOIC_CODEC_OFF ||
	    state->stdac_mode != AUDIOIC_STDAC_OFF ||
	    state->codec_primary_speaker != AUDIOIC_OUT_NONE ||
	    state->stdac_primary_speaker != AUDIOIC_OUT_NONE ||
	    state->ext_primary_speaker != AUDIOIC_OUT_NONE ||
	    state->microphone != AUDIOIC_IN_NONE) {
		/* step 0 */
		audioic_configure_power(state, previous_state, TRUE);
	}

	if (!INPUT_IS_EMU() && !OUTPUT_IS_EMU()) {
		/* step 1 */
		audioic_deactivate_emu_device(state, previous_state);
	}

	if (SPEAKER_TURNING_OFF()) {
		/* step 2 */
		audioic_configure_output(state, previous_state);
	}

	if (CODEC_CHANGED() || STDAC_CHANGED()) {
		/* step 3...
		 * note that if we're not actually doing a step 4,
		 * then this won't get hit, and we'll fall through
		 * and make the change at step 5 */
		int codec_mute = state->codec_mute;
		int stdac_mute = state->stdac_mute;

		state->codec_mute = AUDIOIC_CODEC_MUTE;
		state->stdac_mute = AUDIOIC_STDAC_MUTE;

		audioic_configure_aud_mute(state, previous_state);

		previous_state->codec_mute = state->codec_mute;
		previous_state->stdac_mute = state->stdac_mute;

		state->codec_mute = codec_mute;
		state->stdac_mute = stdac_mute;

		/* step 4
		 * NOTE: if codec and stdac share a digital audio port,
		 * then they must be sequenced such that they're never
		 * both on at the same time:
		 * configure device turning off
		 * reconfigure SSI
		 * configure device turning on
		 * ...but they have their own ports now so we're
		 * not worrying about that yet */

		audioic_configure_codec(state, previous_state);
		audioic_configure_stdac(state, previous_state);

		/* wait for noisy digital changes to settle before continuing */
		/* apal_audioic_sleep(AUDIOIC_SLEEP_CODEC_ENABLE); */
	}

	audioic_configure_input(state, previous_state);

	audioic_configure_input_gains(state, previous_state);

	/* step 5:
	   ...enable PGA's, enable mixer, configure mono adder */
	audioic_configure_output_gains(state, previous_state);

	msleep(SLEEP_CODEC_STDAC_PGA_SWITCH);

	/* ...and activate the mixer inputs */
	audioic_configure_aud_mute(state, previous_state);

	msleep(SLEEP_OUTPUT_AMPS_ENABLE);

	/* step 6: select speakers */
	audioic_configure_output(state, previous_state);

	/* step 7: configure EMU */
	audioic_configure_emu_device(state, previous_state);

	/* step 8: if turning off audio, disable aud_bias */
	if (state->codec_mode == AUDIOIC_CODEC_OFF &&
	    state->stdac_mode == AUDIOIC_STDAC_OFF &&
	    state->codec_primary_speaker == AUDIOIC_OUT_NONE &&
	    state->stdac_primary_speaker == AUDIOIC_OUT_NONE &&
	    state->ext_primary_speaker == AUDIOIC_OUT_NONE &&
	    state->microphone == AUDIOIC_IN_NONE) {
		audioic_configure_power(state, previous_state, FALSE);
	}

	/* SAVE OLD AUDIO STATE */
	previous_state_struct = *state;

	audioic_register_dump(state);
}

/*============================================================================
FUNCTION: AUDIOIC_init

DESCRIPTION:
   This function is used to initialize all the Audio IC audio registers

ARGUMENTS PASSED:
   None

RETURN VALUE:
   None

PRE-CONDITIONS:
   None

POST-CONDITIONS:
   None

IMPORTANT NOTES:
   None

==============================================================================*/
void AUDIOIC_init(struct AUDIOIC_STATE_T *state)
{
	AUDIOIC_DEBUG_LOG("%s() called\n", __func__);

#ifdef BYPASS_SIERRA
	cpcap_write(REG_CPCAP_CC, 0, 0xFFFF);
	cpcap_write(REG_CPCAP_CDI, 0, 0xBFFF);
	cpcap_write(REG_CPCAP_SDAC, 0, 0xFFF);
	cpcap_write(REG_CPCAP_SDACDI, 0, 0x3FFF);
	cpcap_write(REG_CPCAP_TXI, 0, 0xFDF);
	cpcap_write(REG_CPCAP_TXMP, 0, 0xFFF);
	cpcap_write(REG_CPCAP_RXOA, 0, 0x3FF);
	/* cpcap_write (REG_CPCAP_RXVC, 0, 0xFFF); */
	cpcap_write(REG_CPCAP_RXCOA, 0, 0x7FF);
	cpcap_write(REG_CPCAP_RXSDOA, 0, 0x1FFF);
	cpcap_write(REG_CPCAP_RXEPOA, 0, 0x7FFF);
#else
	cpcap_regacc_write(state->cpcap, CPCAP_REG_CC, 0, 0xFFFF);
	cpcap_regacc_write(state->cpcap, CPCAP_REG_CDI, 0, 0xBFFF);
	cpcap_regacc_write(state->cpcap, CPCAP_REG_SDAC, 0, 0xFFF);
	cpcap_regacc_write(state->cpcap, CPCAP_REG_SDACDI, 0, 0x3FFF);
	cpcap_regacc_write(state->cpcap, CPCAP_REG_TXI, 0, 0xFDF);
	cpcap_regacc_write(state->cpcap, CPCAP_REG_TXMP, 0, 0xFFF);
	cpcap_regacc_write(state->cpcap, CPCAP_REG_RXOA, 0, 0x3FF);
	/* cpcap_regacc_write(state->cpcap, CPCAP_REG_RXVC, 0, 0xFFF); */
	cpcap_regacc_write(state->cpcap, CPCAP_REG_RXCOA, 0, 0x7FF);
	cpcap_regacc_write(state->cpcap, CPCAP_REG_RXSDOA, 0, 0x1FFF);
	cpcap_regacc_write(state->cpcap, CPCAP_REG_RXEPOA, 0, 0x7FFF);
#endif
	audio_reg = regulator_get(NULL, "vaudio");
	printk(KERN_INFO "get audio regulator\n");
	if (IS_ERR(audio_reg))
		AUDIOIC_DEBUG_LOG("could not get regulator for cpcap audio\n");

	AUDIOIC_DEBUG_LOG("%s() END called\n", __func__);
	return;
}
