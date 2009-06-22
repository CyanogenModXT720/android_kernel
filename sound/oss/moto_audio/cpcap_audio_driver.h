 /*
  * Copyright (C)2007 - 2009 Motorola, Inc.
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
  *
  *
  * Revision History:
  *
  * Date         Author    Comment
  * ----------   --------  ---------------------------
  * 12/18/2007   Motorola  File creation
  * 10/16/2008   Motorola  Add EMU accy updates
  *
  *
  */

#ifndef CPCAP_AUDIO_DRIVER_H
#define CPCAP_AUDIO_DRIVER_H

/*===========================================================================
				INCLUDE FILES
=============================================================================*/
#include <linux/soundcard.h>
#include <linux/spi/cpcap.h>
/* #include "platform_defs.h" */

/*===========================================================================
						CONSTANTS
=============================================================================*/
/* #define AUDIO_I2S_MODE */

#ifndef BOOL
#define BOOL unsigned char
#endif

#ifndef TRUE
#define TRUE (BOOL)1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/*===========================================================================
						ENUMS
=============================================================================*/

/*===========================================================================
					STRUCTURES AND OTHER TYPEDEFS
=============================================================================*/
#define REG_CPCAP_VAUDIOC             0x200
#define REG_CPCAP_CC                  0x201
#define REG_CPCAP_CDI                 0x202
#define REG_CPCAP_SDAC                0x203
#define REG_CPCAP_SDACDI              0x204
#define REG_CPCAP_TXI                 0x205
#define REG_CPCAP_TXMP                0x206
#define REG_CPCAP_RXOA                0x207
#define REG_CPCAP_RXVC                0x208
#define REG_CPCAP_RXCOA               0x209
#define REG_CPCAP_RXSDOA              0x20A
#define REG_CPCAP_RXEPOA              0x20B

/* Register 512 - Audio Regulator and Bias Voltage */
#define CPCAP_V_AUDIO_EN              0x00000001
#define CPCAP_VAUDIO_MODE0            0x00000002
#define CPCAP_VAUDIO_MODE1            0x00000004
#define CPCAP_VAUDIOPRISTBY           0x00000010
#define CPCAP_AUD_LOWPWR_SPEED        0x00000020
#define CPCAP_AUDIO_LOW_PWR           0x00000040

/* Register 513 CODEC */
#define CPCAP_AUDIHPF_0               0x00000001
#define CPCAP_AUDIHPF_1               0x00000002
#define CPCAP_AUDOHPF_0               0x00000004
#define CPCAP_AUDOHPF_1               0x00000008
#define CPCAP_MIC1_CDC_EN             0x00000010
#define CPCAP_DF_RESET                0x00000020
#define CPCAP_CDC_EN_RX               0x00000040
#define CPCAP_MIC2_CDC_EN             0x00000080
#define CPCAP_CDC_CLOCK_TREE_RESET    0x00000100
#define CPCAP_CDC_SR0                 0x00000200
#define CPCAP_CDC_SR1                 0x00000400
#define CPCAP_CDC_SR2                 0x00000800
#define CPCAP_CDC_SR3                 0x00001000
#define CPCAP_CDC_CLK0                0x00002000
#define CPCAP_CDC_CLK1                0x00004000
#define CPCAP_CDC_CLK2                0x00008000

/* Register 514 CODEC Digital Audio Interface */
#define CPCAP_SMB_CDC                 0x00000001
#define CPCAP_CLK_INV                 0x00000002
#define CPCAP_FS_INV                  0x00000004
#define CPCAP_MIC1_RX_TIMESLOT0       0x00000008
#define CPCAP_MIC1_RX_TIMESLOT1       0x00000010
#define CPCAP_MIC1_RX_TIMESLOT2       0x00000020
#define CPCAP_MIC2_TIMESLOT0          0x00000040
#define CPCAP_MIC2_TIMESLOT1          0x00000080
#define CPCAP_MIC2_TIMESLOT2          0x00000100
#define CPCAP_CDC_DIG_AUD_FS0         0x00000200
#define CPCAP_CDC_DIG_AUD_FS1         0x00000400
#define CPCAP_CDC_CLK_EN              0x00000800
#define CPCAP_DIG_AUD_IN              0x00001000
#define CPCAP_CLK_IN_SEL              0x00002000
#define CPCAP_CDC_PLL_SEL             0x00008000

/* Register 515 Stereo DAC */
#define CPCAP_ST_DAC_EN               0x00000001
#define CPCAP_ST_DAC_CLK0             0x00000002
#define CPCAP_ST_DAC_CLK1             0x00000004
#define CPCAP_ST_DAC_CLK2             0x00000008
#define CPCAP_ST_SR0                  0x00000010
#define CPCAP_ST_SR1                  0x00000020
#define CPCAP_ST_SR2                  0x00000040
#define CPCAP_ST_SR3                  0x00000080
#define CPCAP_DF_RESET_ST_DAC         0x00000100
#define CPCAP_ST_CLOCK_TREE_RESET     0x00000200
#define CPCAP_SLAVE_PLL_CLK_INPUT     0x00000400
#define CPCAP_FSYNC_CLK_IN_COMMON     0x00000800

/* Register 516 Stereo DAC Digital Audio Interface */
#define CPCAP_SMB_ST_DAC              0x00000001
#define CPCAP_ST_CLK_EN               0x00000002
#define CPCAP_DIG_AUD_IN_ST_DAC       0x00000004
#define CPCAP_ST_DIG_AUD_FS0          0x00000008
#define CPCAP_ST_DIG_AUD_FS1          0x00000010
#define CPCAP_ST_CLK_INV              0x00000020
#define CPCAP_ST_FS_INV               0x00000040
#define CPCAP_ST_DAC_CLK_IN_SEL       0x00000080
#define CPCAP_ST_R_TIMESLOT0          0x00000100
#define CPCAP_ST_R_TIMESLOT1          0x00000200
#define CPCAP_ST_R_TIMESLOT2          0x00000400
#define CPCAP_ST_L_TIMESLOT0          0x00000800
#define CPCAP_ST_L_TIMESLOT1          0x00001000
#define CPCAP_ST_L_TIMESLOT2          0x00002000

/* Register 517 Tx Inputs */
#define CPCAP_DLM                     0x00000001
#define CPCAP_MIC1_PGA_EN             0x00000002
#define CPCAP_MIC1_MUX                0x00000004
#define CPCAP_HS_MIC_MUX              0x00000008
#define CPCAP_EMU_MIC_MUX             0x00000010
#define CPCAP_CDEC_DIS                0x00000020
#define CPCAP_MIC2_PGA_EN             0x00000040
#define CPCAP_MIC2_MUX                0x00000080
#define CPCAP_RX_R_ENCODE             0x00000100
#define CPCAP_RX_L_ENCODE             0x00000200
#define CPCAP_MB_ON1R                 0x00000400
#define CPCAP_MB_ON1L                 0x00000800
#define CPCAP_MB_ON2                  0x00001000
#define CPCAP_HS_ID_TX                0x00002000
#define CPCAP_PTT_CMP_EN              0x00004000
#define CPCAP_PTT_VTH                 0x00008000

/* Register 518 MIC PGA's */
#define CPCAP_MIC1_GAIN_0             0x00000001
#define CPCAP_MIC1_GAIN_1             0x00000002
#define CPCAP_MIC1_GAIN_2             0x00000004
#define CPCAP_MIC1_GAIN_3             0x00000008
#define CPCAP_MIC1_GAIN_4             0x00000010
#define CPCAP_MIC2_GAIN_0             0x00000020
#define CPCAP_MIC2_GAIN_1             0x00000040
#define CPCAP_MIC2_GAIN_2             0x00000080
#define CPCAP_MIC2_GAIN_3             0x00000100
#define CPCAP_MIC2_GAIN_4             0x00000200
#define CPCAP_MB_BIAS_R0              0x00000400
#define CPCAP_MB_BIAS_R1              0x00000800

/* Register 519 RX Output Amplifiers */
#define CPCAP_A1_EAR_EN               0x00000001
#define CPCAP_A2_LDSP_R_EN            0x00000002
#define CPCAP_A2_LDSP_L_EN            0x00000004
#define CPCAP_A4_LINEOUT_R_EN         0x00000008
#define CPCAP_A4_LINEOUT_L_EN         0x00000010
#define CPCAP_HS_R_EN                 0x00000020
#define CPCAP_HS_L_EN                 0x00000040
#define CPCAP_EMU_SPKR_L_EN           0x00000080
#define CPCAP_EMU_SPKR_R_EN           0x00000100
#define CPCAP_ST_HS_CP_EN             0x00000200
#define CPCAP_HS_ID_RX                0x00000400

/* Register 520 RX Volume Control */
#define CPCAP_VOL_CDC_LSB_1dB0        0x00000001
#define CPCAP_VOL_CDC_LSB_1dB1        0x00000002
#define CPCAP_VOL_CDC0                0x00000004
#define CPCAP_VOL_CDC1                0x00000008
#define CPCAP_VOL_CDC2                0x00000010
#define CPCAP_VOL_CDC3                0x00000020
#define CPCAP_VOL_DAC_LSB_1dB0        0x00000040
#define CPCAP_VOL_DAC_LSB_1dB1        0x00000080
#define CPCAP_VOL_DAC0                0x00000100
#define CPCAP_VOL_DAC1                0x00000200
#define CPCAP_VOL_DAC2                0x00000400
#define CPCAP_VOL_DAC3                0x00000800
#define CPCAP_VOL_EXT0                0x00001000
#define CPCAP_VOL_EXT1                0x00002000
#define CPCAP_VOL_EXT2                0x00004000
#define CPCAP_VOL_EXT3                0x00008000

/* Register 521 Codec to Output Amp Switches */
#define CPCAP_A1_EAR_CDC_SW           0x00000001
#define CPCAP_A2_LDSP_R_CDC_SW        0x00000002
#define CPCAP_A2_LDSP_L_CDC_SW        0x00000004
#define CPCAP_A4_LINEOUT_R_CDC_SW     0x00000008
#define CPCAP_A4_LINEOUT_L_CDC_SW     0x00000010
#define CPCAP_ARIGHT_HS_CDC_SW        0x00000020
#define CPCAP_ALEFT_HS_CDC_SW         0x00000040
#define CPCAP_PGA_OUTL_USBDN_CDC_SW   0x00000080
#define CPCAP_PGA_OUTR_USBDP_CDC_SW   0x00000100
#define CPCAP_CDC_SW                  0x00000200
#define CPCAP_PGA_CDC_EN              0x00000400

/* Register 522 RX Stereo DAC to Output Amp Switches */
#define CPCAP_A1_EAR_DAC_SW           0x00000001
#define CPCAP_A2_LDSP_R_DAC_SW        0x00000002
#define CPCAP_A2_LDSP_L_DAC_SW        0x00000004
#define CPCAP_A4_LINEOUT_R_DAC_SW     0x00000008
#define CPCAP_A4_LINEOUT_L_DAC_SW     0x00000010
#define CPCAP_ARIGHT_HS_DAC_SW        0x00000020
#define CPCAP_ALEFT_HS_DAC_SW         0x00000040
#define CPCAP_PGA_OUTL_USBDN_DAC_SW   0x00000080
#define CPCAP_PGA_OUTR_USBDP_DAC_SW   0x00000100
#define CPCAP_MONO_DAC0               0x00000200
#define CPCAP_MONO_DAC1               0x00000400
#define CPCAP_ST_DAC_SW               0x00000800
#define CPCAP_PGA_DAC_EN              0x00001000

/* Register 523 RX External PGA to Output Amp Switches */
#define CPCAP_A1_EAR_EXT_SW           0x00000001
#define CPCAP_A2_LDSP_R_EXT_SW        0x00000002
#define CPCAP_A2_LDSP_L_EXT_SW        0x00000004
#define CPCAP_A4_LINEOUT_R_EXT_SW     0x00000008
#define CPCAP_A4_LINEOUT_L_EXT_SW     0x00000010
#define CPCAP_ARIGHT_HS_EXT_SW        0x00000020
#define CPCAP_ALEFT_HS_EXT_SW         0x00000040
#define CPCAP_PGA_OUTL_USBDN_EXT_SW   0x00000080
#define CPCAP_PGA_OUTR_USBDP_EXT_SW   0x00000100
#define CPCAP_MONO_EXT0               0x00000200
#define CPCAP_MONO_EXT1               0x00000400
#define CPCAP_PGA_IN_R_SW             0x00000800
#define CPCAP_PGA_IN_L_SW             0x00001000
#define CPCAP_PGA_EXT_R_EN            0x00002000
#define CPCAP_PGA_EXT_L_EN            0x00004000

/* Register 524 RX Low Latency(LL) */
#define CPCAP_LL_RX_LOW_LAT_EN        0x00000001
#define CPCAP_LL_A2_LDSP_R_EN         0x00000002
#define CPCAP_LL_A2_LDSP_L_EN         0x00000004
#define CPCAP_LL_A4_LINEOUT_R_EN      0x00000008
#define CPCAP_LL_A4_LINEOUT_L_EN      0x00000010
#define CPCAP_LL_PGA_EXT_R_EN         0x00000020
#define CPCAP_LL_PGA_EXT_L_EN         0x00000040
#define CPCAP_LL_PGA_IN_R_SW          0x00000080
#define CPCAP_LL_PGA_IN_L_SW          0x00000100
#define CPCAP_LL_A2_LDSP_R_EXT_SW     0x00000200
#define CPCAP_LL_A2_LDSP_L_EXT_SW     0x00000400
#define CPCAP_LL_A4_LINEOUT_R_EXT_SW  0x00000800
#define CPCAP_LL_A4_LINEOUT_L_EXT_SW  0x00001000
#define CPCAP_LL_MONO_EXT0            0x00002000
#define CPCAP_LL_MONO_EXT1            0x00004000
#define CPCAP_LL_RX_LOW_LAT_SNS       0x00008000

/* Register 525 A2 Loudspeaker Amplifier and Clock Configuration for HS */
#define CPCAP_A2_CONFIG               0x00000001
#define CPCAP_A2_CLK_EN               0x00000002
#define CPCAP_A2_CLK0                 0x00000004
#define CPCAP_A2_CLK1                 0x00000008
#define CPCAP_A2_CLK2                 0x00000010
#define CPCAP_A2_FREE_RUN             0x00000020
#define CPCAP_A2_CLK_SYNC             0x00000040
#define CPCAP_NCP_CLK_SYNC            0x00000080

enum {
	AUDIOIC_MODE_NORMAL,	/* mode of normal audio operation */
	AUDIOIC_MODE_DAI,	/* AUDIOIC is configured for DAI testing */
	AUDIOIC_MODE_DAI_DOWNLINK = AUDIOIC_MODE_DAI,
	AUDIOIC_MODE_DAI_UPLINK,
	AUDIOIC_MODE_TTY	/* AUDIOIC is configured for TTY accessory */
};

enum {
	AUDIOIC_CODEC_OFF,	/* codec is powered down */
	AUDIOIC_CODEC_CLOCK_ONLY, /* codec is powered down, but clocks
				   * are activated */
	AUDIOIC_CODEC_ON,	/* codec is completely operational */
	AUDIOIC_CODEC_LOOPBACK	/* xcap is put in full
				 * (analog->digital->analog) loopback mode */
};

enum {
	AUDIOIC_CODEC_RATE_8000_HZ,
		/* codec is running at 8Khz sample rate */
	AUDIOIC_CODEC_RATE_11025_HZ,
		/* codec is running at 11.025Khz sample rate */
	AUDIOIC_CODEC_RATE_12000_HZ,
		/* codec is running at 12Khz sample rate */
	AUDIOIC_CODEC_RATE_16000_HZ,
		/* codec is running at 16Khz sample rate */
	AUDIOIC_CODEC_RATE_22050_HZ,
		/* codec is running at 22.05Khz sample rate */
	AUDIOIC_CODEC_RATE_24000_HZ,
		/* codec is running at 24Khz sample rate */
	AUDIOIC_CODEC_RATE_32000_HZ,
		/* codec is running at 32Khz sample rate */
	AUDIOIC_CODEC_RATE_44100_HZ,
		/* codec is running at 44.1Khz sample rate */
	AUDIOIC_CODEC_RATE_48000_HZ,
		/* codec is running at 48Khz sample rate */
};

enum {
	AUDIOIC_CODEC_UNMUTE,	/* codec is unmuted */
	AUDIOIC_CODEC_MUTE,	/* codec is muted */
	AUDIOIC_CODEC_BYPASS_LOOP
				/* codec is bypassed
				 * (analog-only loopback mode) */
};

enum {
	AUDIOIC_STDAC_OFF,
		/* stereo dac is powered down */
	AUDIOIC_STDAC_CLOCK_ONLY,
		/* stereo dac is powered down, but clocks are activated */
	AUDIOIC_STDAC_ON
		/* stereo dac is completely operational */
};

enum {
		/* THESE MUST CORRESPOND TO XAUDIOIC SETTINGS */
	AUDIOIC_STDAC_RATE_8000_HZ,
		/* stereo dac set for 8Khz sample rate */
	AUDIOIC_STDAC_RATE_11025_HZ,
		/* stereo dac set for 11.025Khz sample rate */
	AUDIOIC_STDAC_RATE_12000_HZ,
		/* stereo dac set for 12Khz sample rate */
	AUDIOIC_STDAC_RATE_16000_HZ,
		/* stereo dac set for 16Khz sample rate */
	AUDIOIC_STDAC_RATE_22050_HZ,
		/* stereo dac set for 22.05Khz sample rate */
	AUDIOIC_STDAC_RATE_24000_HZ,
		/* stereo dac set for 24Khz sample rate */
	AUDIOIC_STDAC_RATE_32000_HZ,
		/* stereo dac set for 32Khz sample rate */
	AUDIOIC_STDAC_RATE_44100_HZ,
		/* stereo dac set for 44.1Khz sample rate */
	AUDIOIC_STDAC_RATE_48000_HZ
		/* stereo dac set for 48Khz sample rate */
};

enum {
	AUDIOIC_STDAC_UNMUTE,	/* stereo dac is unmuted */
	AUDIOIC_STDAC_MUTE	/* stereo dac is muted */
};

enum {
	AUDIOIC_ANALOG_SOURCE_OFF,
		/* Analog PGA input is disabled */
	AUDIOIC_ANALOG_SOURCE_R,
		/* Right analog PGA input is enabled */
	AUDIOIC_ANALOG_SOURCE_L,
		/* Left analog PGA input is enabled */
	AUDIOIC_ANALOG_SOURCE_STEREO
		/* Both analog PGA inputs are enabled */
};

enum {
	AUDIOIC_OUT_NONE,
		/* No audio output selected */
	AUDIOIC_OUT_HANDSET = SOUND_MASK_PHONEOUT,
		/* handset (earpiece) speaker */
	AUDIOIC_OUT_LOUDSPEAKER = SOUND_MASK_SPEAKER,
		/* loudspeaker (speakerphone) */
	AUDIOIC_OUT_LINEAR_VIBRATOR,
		/* linear vibrator, if equipped */
	AUDIOIC_OUT_MONO_HEADSET = SOUND_MASK_LINE1,
		/* mono (R channel) x.5mm headset */
	AUDIOIC_OUT_STEREO_HEADSET = SOUND_MASK_RADIO,
		/* stereo x.5mm headset */
	AUDIOIC_OUT_EXT_BUS_MONO = SOUND_MASK_CD,
		/* accessory bus mono output(EMU) */
	AUDIOIC_OUT_EMU_MONO = SOUND_MASK_LINE2,
	AUDIOIC_OUT_EXT_BUS_STEREO = SOUND_MASK_LINE3,
		/* accessory bus stereo output (EMU only) */
	AUDIOIC_OUT_EMU_STEREO = SOUND_MASK_LINE3,
	AUDIOIC_OUT_LINEOUT = SOUND_MASK_LINE,
	AUDIOIC_OUT_BT_MONO = SOUND_MASK_DIGITAL1,
	AUDIOIC_OUT_NUM_OF_PATHS
		/* Max number of audio output paths */
};

enum {
	AUDIOIC_IN_NONE,
		/* No audio input selected */
	AUDIOIC_IN_HANDSET = SOUND_MASK_PHONEIN,
		/* handset (internal) microphone */
	AUDIOIC_IN_AUX_INTERNAL = SOUND_MASK_MIC,
		/* Auxiliary (second) internal mic */
	AUDIOIC_IN_DUAL_INTERNAL = SOUND_MASK_LINE3,
		/* both internal microphones are connected */
	AUDIOIC_IN_HEADSET = SOUND_MASK_LINE1,
		/* Audio <- x.5mm headset microphone */
	AUDIOIC_IN_EXT_BUS = SOUND_MASK_LINE2,
		/* Audio <- accessory bus analog input (EMU) */
	AUDIOIC_IN_EMU = AUDIOIC_IN_EXT_BUS,
	AUDIOIC_IN_HEADSET_BIAS_ONLY = SOUND_MASK_LINE1,
		/* 3.5mm headset control when no mic is selected */
	AUDIOIC_IN_DUAL_EXTERNAL = SOUND_MASK_LINE,
		/* Recording from external source */
	AUDIOIC_IN_BT_MONO = SOUND_MASK_DIGITAL1,
	AUDIOIC_IN_NUM_OF_PATHS
		/* Max number of audio input paths */
};

enum {
		/* Defines the audio path type */
	AUDIOIC_AUDIO_IN_PATH,
		/* Audio input path refers to AUDIOIC_MIC_TYPE */
	AUDIOIC_AUDIO_OUT_PATH
		/* Audio output path refers to AUDIOIC_SPEAKER_TYPE */
};

enum {
	AUDIOIC_BALANCE_NEUTRAL,/* audio routed normally */
	AUDIOIC_BALANCE_R_ONLY,	/* audio routed to left channel only */
	AUDIOIC_BALANCE_L_ONLY	/* audio routed to right channel only */
};

enum {
	AUDIOIC_RAT_NONE,	/* Not in a call mode */
	AUDIOIC_RAT_2G,		/* In 2G call mode */
	AUDIOIC_RAT_3G,		/* In 3G call mode */
	AUDIOIC_RAT_CDMA	/* In CDMA call mode */
};

struct AUDIOIC_STATE_T {
	struct cpcap_device *cpcap;
	int mode;
	int codec_mode;
	int codec_rate;
	int codec_mute;
	int stdac_mode;
	int stdac_rate;
	int stdac_mute;
	int analog_source;
	int codec_primary_speaker;
	int codec_secondary_speaker;
	int stdac_primary_speaker;
	int stdac_secondary_speaker;
	int ext_primary_speaker;
	int ext_secondary_speaker;
	int codec_primary_balance;
	int stdac_primary_balance;
	int ext_primary_balance;
	unsigned int output_gain;
	int microphone;
	unsigned int input_gain;
	int rat_type;
	unsigned char emu_accy;
};

/*===========================================================================
				MACROS
=============================================================================*/

/*===========================================================================
			GLOBAL VARIABLE DECLARATIONS
=============================================================================*/

/*===========================================================================
				FUNCTION PROTOTYPES
=============================================================================*/
void AUDIOIC_set_audio_state(struct AUDIOIC_STATE_T *state);

void AUDIOIC_init(struct AUDIOIC_STATE_T *state);

int is_cdma_supported(void);

#ifdef BYPASS_SIERRA
extern int cpcap_spi_read(int, int *);
extern int cpcap_spi_write(int, int);
#endif /* BYPASS_SIERRA */

#endif /* CPCAP_AUDIO_DRIVER_H */
