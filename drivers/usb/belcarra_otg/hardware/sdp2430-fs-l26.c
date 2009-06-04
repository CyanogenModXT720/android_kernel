/*
 * otg/hardware/sdp2430-l26.c - iMX31ADS EVB OTG Peripheral and OTG Controller Drivers Module Initialization
 * @(#) sl@belcarra.com/whiskey.enposte.net|otg/platform/sdp2430/sdp2430-fs-l26.c|20070914025316|53353
 *
 *      Copyright (c) 2004-2005 Belcarra Technologies Corp
 *	Copyright (c) 2005-2006 Belcarra Technologies 2005 Corp
 *
 * By:
 *      Stuart Lynne <sl@belcarra.com>,
 *      Bruce Balden <balden@belcarra.com>
 */
/*!
 * @defgroup SDP2430 TI OMAP High Speed
 * @ingroup Hardware
 */
/*!
 * @file otg/hardware/sdp2430-l26.c
 * @brief TI OMAPHS USB Host Controller Driver
 *
 * Linux SDP2430 OTG PCD/OCD/TCD Driver Initialization
 *
 * This file selects and initializes all of the low level hardware drivers
 * for the TI OMAP High Speed.
 *
 * Notes.
 *
 *
 * @ingroup SDP2430
 * @ingroup OMAPHS
 * @ingroup LINUXOS
 */

#include <otg/pcd-include.h>
#include <otg/otg-dev.h>
#include <linux/module.h>

#include <linux/pci.h>
#include <asm/arch/gpio.h>
#include <asm/arch/board.h>

//#include "mxc-lnx.h"
//#include "mxc-hardware.h"

MOD_AUTHOR ("sl@belcarra.com");
MOD_DESCRIPTION ("Belcarra SDP2430");
EMBED_LICENSE();


MOD_PARM_STR(serial_number_str, "Serial Number String", NULL);


/* ************************************************************************************* */

//extern int omaphs_dev_module_init(struct otg_device_driver *);
extern int omaphs_dev_module_init(struct otg_device_driver *otg_device_driver, 
                int (*device_probe)(struct device *),
                int (*device_remove)(struct device *)
                );

extern void omaphs_dev_module_exit(struct otg_device_driver *);

extern int omaphs_pcd_module_init (struct otg_device_driver *);
extern void omaphs_pcd_module_exit (struct otg_device_driver *); 

static struct otg_device_driver sdp2430_otg_device_driver = {
        .name =         "sdp2430-usbd",
};



/*! sdp2430_dev_probe - called to initialize platform
 * @param dev - device
 *
 * This is used to call the dev level probe functions with
 * the additional otg_device_driver structure. 
 */
static int
sdp2430_dev_probe(struct device *device)
{
        printk(KERN_INFO"%s:\n", __FUNCTION__); 
        RETURN_ZERO_UNLESS (sdp2430_otg_device_driver.probe);
        return sdp2430_otg_device_driver.probe(device, &sdp2430_otg_device_driver);
}

/*! sdp2430_dev_remove- called to remote device
 * @param dev - device
 *
 * This is used to call the dev level probe functions with
 * the additional otg_device_driver structure. 
 */
static int
sdp2430_dev_remove(struct device *device)
{
        int rc;
        printk(KERN_INFO"%s:\n", __FUNCTION__);
        RETURN_ZERO_UNLESS (sdp2430_otg_device_driver.remove);
        rc =  sdp2430_otg_device_driver.remove(device, &sdp2430_otg_device_driver);
        return rc;
}


/*!
 * sdp2430_modexit() - This is used as module exit, and as cleanup if modinit fails.
 */
static void sdp2430_modexit (void)
{
        /* unload the dev driver, this will stop otg and destroy
         * the otg_dev and otg instances etc.
         */
        printk(KERN_INFO"%s:\n", __FUNCTION__);
        omaphs_dev_module_exit(&sdp2430_otg_device_driver);

        /* cleanup the rest of the sub-drivers
         */
        omaphs_pcd_module_exit(&sdp2430_otg_device_driver);
}

/*!
 * sdp2430_modinit() - linux module initialization
 *
 * This needs to initialize the hcd, pcd and tcd drivers. This includes tcd and possibly hcd
 * for some architectures.
 *
 */
static int sdp2430_modinit (void)
{
        int pcd = -1, dev = -1;

	printk(KERN_INFO"%s:\n", __FUNCTION__); 

        /* initialize all of the sub-drivers except for dev 
         */
        pcd = omaphs_pcd_module_init(&sdp2430_otg_device_driver);

        /* ensure everything is ok until now */
        THROW_IF(pcd, error);

        /* serial number needs to be set prior to initializing the dev
         * driver
         */
        sdp2430_otg_device_driver.serial_number = MODPARM(serial_number_str);

        /* initialize the dev driver, this will get all sub-drivers
         * started via their probe functions, create otg and otg_dev
         * instances and finally start otg state machine.
         */
	printk(KERN_INFO"%s:\n", __FUNCTION__); 
	printk(KERN_INFO"%s: calling dev module init\n", __FUNCTION__); 
	printk(KERN_INFO"%s:\n", __FUNCTION__); 

        THROW_IF((dev = omaphs_dev_module_init(&sdp2430_otg_device_driver, sdp2430_dev_probe, sdp2430_dev_remove)), error);

        return 0;

        CATCH(error) {

                printk(KERN_INFO"%s: FAILED\n", __FUNCTION__); 

                UNLESS (dev) omaphs_dev_module_exit(&sdp2430_otg_device_driver);
                UNLESS (pcd) omaphs_pcd_module_exit(&sdp2430_otg_device_driver);

                return -EINVAL;
        }
}

module_init (sdp2430_modinit);
module_exit (sdp2430_modexit);

