#ifndef __INCLUDED_TDA19989__
#define __INCLUDED_TDA19989__

#include <linux/types.h>

/*#define TDA19989_CEC_AVAILABLE*/

#define HDMI_I2C_WRITE		0
#define HDMI_I2C_READ		1
#define HDMI_PWR_ONOFF	2
#define HDMI_INT_ENABLE	3
#ifdef TDA19989_CEC_AVAILABLE
#define HDMI_CEC_CAL_TIME	4
#endif

#define HDMI_TRNS_NAME  "tda19989"

#define HDMI_INT_PIN_GPIO_NUM 25
#define HDMI_PWR_EN_GPIO_NUM 26

typedef struct _i2cKernelModeArg
{
    u8 slaveAddr;
    u8 firstRegister;
    u8 lenData;
    u8 Data[128];
} i2cKernelModeArg;

#endif

