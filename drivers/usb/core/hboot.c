#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/io.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/suspend.h>
#include <mach/gpio.h>                                                                                                                       
#include <linux/gpio_mapping.h> 
#include <linux/dmapool.h>
#include <linux/usb.h>

#define CTRL_DEVNAME "hbootctrl"
#define MAPPHONE_IPC_USB_SUSP_GPIO  142


static void czecho_get_descriptors(void)
{
}
