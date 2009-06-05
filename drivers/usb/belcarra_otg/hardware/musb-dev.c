/*
 * otg/hardware/musb-dev.c - OMAPHS Device driver 
 * @(#) sl@belcarra.com/whiskey.enposte.net|otg/platform/omap/musb-dev.c|20070912224952|14903
 *
 *      Copyright (c) 2007 Belcarra Technologies 2005 Corp
 *
 * By:
 *      Stuart Lynne <sl@belcarra.com>
 */
/*!
 * @file otg/hardware/musb-dev.c
 * @brief Belcarra OMAPHS Device driver.
 *
 * @ingroup OMAPHS
 * @ingroup OTGDEV
 * @ingroup LINUXOS
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>

#include <linux/usb.h>
#include <linux/delay.h>

#include <otg/otg-compat.h>
#include <otg/otg-api.h>
#include <otg/otg-dev.h>

#include <otg/otg-utils.h>

#include <mach/gpio.h>
#include <mach/board.h>
#include <asm/memory.h>
#include <asm/io.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
#include <linux/platform_device.h>
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) */

#include <musb/musb_belcarra_api.h>

//#include <asm/arch/arc_otg.h>

//#define DRIVER_NAME "musb_hrdrc"

#define OMAPHS_MUSB_NAME "musb_usbd"

static const char driver_name[] = OMAPHS_MUSB_NAME;


/* ********************************************************************************************* */
/* OMAPHS Proc FS 
 */

#define SHOW(s, r, c) seq_printf(s, "%20s [%8x] %08x %s\n", #r, &r, r, c)
#define BITS(s, c) seq_printf(s, "%24s: ", c);
//#define BIT(s, r, m) seq_printf(s, "%s%s ", #m, (r & m) ? "" : "/");
#define NL(s) seq_printf(s, "\n");

/*! omaphs_dev_show - called to display hcd information
 * @param s 
 * @param unused
 */
static int omaphs_dev_show(struct seq_file *s, void *unused)
{
        int                     etdn;
        unsigned long           flags;
        u32                     u;

        seq_printf(s, "OMAPHS USB\n\n");

        seq_printf(s, "\nOTG Registers\n");

        //SHOW(s, UOG_ID, "Host ID");
        seq_printf(s, "\n");

        return 0;
}

/*! omaphs_dev_open - open a hcd file 
 * @param inode
 * @param file
 */

static int omaphs_dev_open(struct inode *inode, struct file *file)
{
        return single_open(file, omaphs_dev_show, PDE(inode)->data);
}

/*!  struct file_operatons omaphs_dev_proc_ops */
static struct file_operations omaphs_dev_proc_ops = {
        .open           = omaphs_dev_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};

static const char proc_filename[] = "arcusb";

/* ********************************************************************************************* */
/*!
 * omaphs_udc_isr() - architecture isr wrapper
 * @param irq
 * @param dev_id
 * @param regs
 */
irqreturn_t omaphs_udc_isr(int irq, void *data)
{

        return IRQ_NONE;
}


/* ********************************************************************************************* */
/*! omaphs_dev_remove - called to remove hcd
 * @param dev
 */
static int __init_or_module
omaphs_dev_remove(struct device *device, struct otg_device_driver *otg_device_driver)
{
        struct platform_device  *platform_device = NULL;
        struct otg_dev *otg_dev = dev_get_drvdata(device);
        struct musb_belcarra_instance *musb = otg_dev->privdata;

        platform_device = to_platform_device(device);
        otg_dev_remove(device, otg_device_driver);

        musb_belcarra_cleanup_driver(musb);

        return 0;
}

extern int musb_transceiver_event(struct musb_belcarra_instance *musb, musb_belcarra_transceiver_event_t event);
extern int musb_ep0_irq(struct musb_belcarra_instance *musb);
extern int musb_rx_irq(struct musb_belcarra_instance *musb, int epnum, int dma);
extern int musb_tx_irq(struct musb_belcarra_instance *musb, int epnum, int dma);
extern int musb_dma_irq(struct musb_belcarra_instance *musb_belcarra, u8 index, u32 addr, u32 count, u16 control);
extern int musb_usb_event(struct musb_belcarra_instance *musb, musb_belcarra_usb_event_t event);

struct musb_ops musb_ops = {
        .transceiver_event = musb_transceiver_event,
        .usb_event = musb_usb_event,
        .ep0_irq = musb_ep0_irq,
        .tx_irq = musb_tx_irq,
        .rx_irq = musb_rx_irq,
        .dma_irq = musb_dma_irq,
};

static void omaphs_dev_release(struct device *dev)
{
}

/*! omaphs_dev_probe - called to initialize hcd 
 * @param dev - hcd device
 */
static int 
omaphs_dev_probe(struct device *device, struct otg_device_driver *otg_device_driver)
{
        struct platform_device  *platform_device = to_platform_device(device);
        struct omaphs_usb_config *config = (struct omaphs_usb_config *)device->platform_data;
        u32 id;
        struct musb_belcarra_instance *musb = NULL;
        int probe = -1;
        struct otg_dev *otg_dev = NULL;

	printk(KERN_INFO"%s: %s %s\n", __FUNCTION__, platform_device->name, driver_name); 
        /* verify correct driver name
         */
        THROW_IF(strcmp(platform_device->name, driver_name), error);

        device->release = omaphs_dev_release;


        THROW_UNLESS((musb = musb_belcarra_register_driver(&musb_ops)), error);

        /* do generic device probe, this will allocate resources (e.g. irqs)
         * and call the sub-driver probe functions.
         */
        THROW_IF((probe = otg_dev_probe(device, otg_device_driver, musb)), error);
        otg_dev = dev_get_drvdata(device);
        musb->usbd_privdata = otg_dev;

        return 0;

        CATCH(error) {
                printk(KERN_INFO"%s: FAILED musb: %d probe: %d\n", __FUNCTION__, probe); 
                if (musb)
                        musb_belcarra_cleanup_driver(musb);
                return -ENODEV;
        }
}

/* ********************************************************************************************* */
/*! Linux 2.6 Device Driver Support
 *
 * The Linux 2.6 Core requires that host driver be registered as a device driver. These
 * implement the requirements for the Linux Device Driver support.
 *
 */

/*! omaphs_device_remove - called to remove hcd
 * @param dev
 */
static int __init_or_module
omaphs_device_remove(struct device *dev)
{
        struct platform_device  *platform_device = to_platform_device(dev);
        struct otg_instance *otg = (struct otg_instance *)dev->driver_data;
        struct otg_dev *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb = otg_dev->privdata;

        return 0;
}

static int
omaphs_device_probe(struct device *device)
{
        printk(KERN_INFO"%s: FAILED (not correct probe function)\n", __FUNCTION__);
        return -EINVAL;
}


/* ********************************************************************************************* */
/*! omaphs_device_suspend - called to suspend udc
 * @param dev - hcd device
 * @param state -
 * @param phase -
 */
static int
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
omaphs_device_suspend(struct device *device, pm_message_t state)
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) */
omaphs_device_suspend(struct device *device, u32 state, u32 phase)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) */
{
        //printk(KERN_INFO"%s:\n", __FUNCTION__);
        return -EINVAL;
}

/*! omaphs_device_resume - called to resume udc 
 * @param dev - hcd device
 * @param phase
 */
static int
//omaphs_device_resume(struct device *device, u32 phase)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
omaphs_device_resume(struct device *device)
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) */
omaphs_device_resume(struct device *device, u32 level)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) */
{
        printk(KERN_INFO"%s:\n", __FUNCTION__);
        return -EINVAL;
}

/* ********************************************************************************************* */
/*! omaphs_dev_driver - define a device_driver structure for this driver
 */
static struct device_driver omaphs_dev_driver = {

        .name =         OMAPHS_MUSB_NAME,
        .bus =          &platform_bus_type,

        .probe =        omaphs_device_probe,
        .remove =       omaphs_device_remove,

        .suspend =      omaphs_device_suspend,
        .resume =       omaphs_device_resume,

};

static struct platform_device omaphs_dev_platform = {
        .name =         OMAPHS_MUSB_NAME,
        .id =           -1,
};


/* ********************************************************************************************* */
/*! Module init/exit
 *
 * Register this a linux device driver and platform. This provides the necessary framework
 * expected by the Linux 2.6 USB Core.
 *
 */

/*! omaphs_dev_module_init
 * @param otg_device_driver - platform device driver structure
 * @param otg_dev_probe - platform probe function
 */
int omaphs_dev_module_init(struct otg_device_driver *otg_device_driver, 
                int (*device_probe)(struct device *),
                int (*device_remove)(struct device *))
{
        int driver_registered = -1;
        int platform_registered = -1;
        struct proc_dir_entry *pde = NULL;

        printk(KERN_INFO"%s: platform: %s driver: %s\n", 
                        __FUNCTION__, otg_device_driver->name, omaphs_dev_driver.name) ;

        /* Setup the otg_device_driver probe function to point back to this one,
         * and the actual driver probe to the provided function. This allows the
         * platform driver to provide the overall device structure.
         *
         * We also provide a wrapper ISR in case we need to do something before
         * or after the otg_dev_isr() function has run.
         */
        THROW_UNLESS(otg_device_driver, error);
        THROW_UNLESS(device_probe, error);

	printk(KERN_INFO"%s: AAAA\n", __FUNCTION__); 
        omaphs_dev_driver.probe = device_probe;
        omaphs_dev_driver.remove = device_remove;
        otg_device_driver->probe = omaphs_dev_probe;
        otg_device_driver->remove = omaphs_dev_remove;
        otg_device_driver->isr = omaphs_udc_isr;

        /* register the pxa2xx-udc and pxa2xx-dev drivers
         */

        THROW_IF((platform_registered = platform_device_register(&omaphs_dev_platform)), error);
	printk(KERN_INFO"%s: BBBB\n", __FUNCTION__); 

        THROW_IF((driver_registered = driver_register(&omaphs_dev_driver)), error);
	printk(KERN_INFO"%s: CCCC\n", __FUNCTION__); 

        THROW_UNLESS((pde = create_proc_entry(proc_filename, 0, NULL)), error);
        pde->proc_fops = &omaphs_dev_proc_ops;

	printk(KERN_INFO"%s: DDDD\n", __FUNCTION__); 
        return 0;
        CATCH(error) {
                printk(KERN_INFO"%s: FAILED\n", __FUNCTION__);
                if (pde) remove_proc_entry(proc_filename, NULL);
                UNLESS (driver_registered) driver_unregister(&omaphs_dev_driver);
                UNLESS (platform_registered) platform_device_unregister(&omaphs_dev_platform);

                return -EINVAL;
        }
}

/*! omaphs_dev_module_exit
 */
void omaphs_dev_module_exit(struct otg_device_driver *otg_device_driver)
{
        otg_device_driver->probe = NULL;
        otg_device_driver->isr = NULL;
        remove_proc_entry(proc_filename, NULL);
        driver_unregister(&omaphs_dev_driver);
        platform_device_unregister(&omaphs_dev_platform);
}

