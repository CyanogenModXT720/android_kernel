/*
 * otg/hardware/musb-belcarra.h
 * @(#) balden@belcarra.com|musb_belcarra.c|20070508195253|62699
 *
 *      Copyright (c) 2007 Belcarra Technologies 2005 Corp
 *
 * By:
 *      Stuart Lynne <sl@belcarra.com>
 *      Shahrad Payandeh <sp@belcarra.com>
 *
 *      Copyright (C) 2008 Motorola, Inc
 *
 * This implements a minimal interface between the Belcarra USBOTG
 * stack musb drivers, or more specifically between the following
 * and the TI plat_uds.c core implementation.
 *
 *      musb-pcd.c
 *      musb-tcd.c
 *
 * This allows the various events and interrupts to be passed to
 * the correct driver.
 *
 * Currently the plat_uds.c driver is loaded first and then will
 * pass this driver a pointer to a struct musb which contains all
 * of the neccessary information. This is held as a global until
 * the USBOTG stack is loaded and connected.
 *
 */

/* Date         Author          Comment
 * ===========  ==============  ==============================================
 * 23-Jul-2008  Motorola        Change Copyright and Changelog
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/stat.h>
//#include <linux/usb_ch9.h>

#include "musb_core.h"
//#include "musb_belcarra.h"
#include "musb_belcarra_api.h"


/* ********************************************************************************************* */
/* transceiver events
 */
static
void _musb_transceiver_event(struct musb *pThis, musb_belcarra_transceiver_event_t event)
{
        struct musb_belcarra_instance *musb_belcarra = pThis->pBelcarra_privdata;

        /* return unless USBOTG connected */
        if (!musb_belcarra || !musb_belcarra->ops || !musb_belcarra->ops->transceiver_event) return;
        if (!musb_belcarra->musb_privdata) return;

        musb_belcarra->ops->transceiver_event(musb_belcarra, event);
}

void musb_g_vbus_valid(struct musb *pThis)
{
        _musb_transceiver_event(pThis, musb_vbus_valid);
}

void musb_g_vbus_invalid(struct musb *pThis)
{
        _musb_transceiver_event(pThis, musb_vbus_invalid);
}

/* usb events
 */
static
void _musb_usb_event(struct musb *pThis, musb_belcarra_usb_event_t event)
{
        struct musb_belcarra_instance *musb_belcarra = pThis->pBelcarra_privdata;
        /* return unless USBOTG connected */
        if (!musb_belcarra || !musb_belcarra->ops || !musb_belcarra->ops->usb_event) return;
        if (!musb_belcarra->musb_privdata) return;

        musb_belcarra->ops->usb_event(musb_belcarra, event);
}

void musb_g_reset(struct musb *pThis)
{
        _musb_usb_event(pThis, musb_reset);
}

void musb_g_suspend(struct musb *pThis)
{
        /* hack alert, if we see suspend, then Vbus must be available */
        musb_g_vbus_valid(pThis);
        _musb_usb_event(pThis, musb_suspend);
}

void musb_g_resume(struct musb *pThis)
{
        _musb_usb_event(pThis, musb_resumed);
}

void musb_g_disconnect(struct musb *pThis)
{
        _musb_usb_event(pThis, musb_disconnect);
        musb_g_vbus_invalid(pThis);
}

void musb_g_stop_activity(struct musb *pThis)
{
        _musb_usb_event(pThis, musb_stop_activity);
}


/* musb_g_ep0_irq
 */
irqreturn_t musb_g_ep0_irq(struct musb *pThis)
{
        struct musb_belcarra_instance *musb_belcarra = pThis->pBelcarra_privdata;
        /* pass interrupt through, return IRQ_NONE if USBOTG not connected */
        if (!musb_belcarra->musb_privdata) {
                return IRQ_NONE;
        }
        return (!musb_belcarra || !musb_belcarra->ops || !musb_belcarra->ops->ep0_irq) ?
                IRQ_NONE : musb_belcarra->ops->ep0_irq(musb_belcarra);
}

/* musb_g_rx
 */
#ifdef CONFIG_USB_BELCARRA_MUSB_HDRC   /* Using the Belcarra released code */
void musb_g_rx(struct musb *pThis, u8 epnum, int dma)
{
        struct musb_belcarra_instance *musb_belcarra = pThis->pBelcarra_privdata;



        /* return unless USBOTG connected */
        if (!musb_belcarra || !musb_belcarra->ops || !musb_belcarra->ops->rx_irq) return;
        musb_belcarra->ops->rx_irq(musb_belcarra, epnum, dma);
}
#else
void musb_g_rx(struct musb *pThis, u8 epnum)
{
	struct musb_ep		*musb_ep = &pThis->endpoints[epnum].ep_out;
        struct musb_belcarra_instance *musb_belcarra = pThis->pBelcarra_privdata;
	struct dma_channel	*dma;
	dma = is_dma_capable() ? musb_ep->dma : NULL;
        /* return unless USBOTG connected */
        if (!musb_belcarra || !musb_belcarra->ops || !musb_belcarra->ops->rx_irq) return;
        musb_belcarra->ops->rx_irq(musb_belcarra, epnum, dma?1:0);
}
#endif

/* musb_g_tx
 */
#ifdef CONFIG_USB_BELCARRA_MUSB_HDRC   /* Using the Belcarra released code */
void musb_g_tx(struct musb *pThis, u8 epnum, int dma)
{
        struct musb_belcarra_instance *musb_belcarra = pThis->pBelcarra_privdata;



        /* return unless USBOTG connected */
        if (!musb_belcarra || !musb_belcarra->ops || !musb_belcarra->ops->tx_irq) return;
        musb_belcarra->ops->tx_irq(musb_belcarra, epnum, dma);
}
#else
void musb_g_tx(struct musb *pThis, u8 epnum)
{
	struct musb_ep		*musb_ep = &pThis->endpoints[epnum].ep_out;
        struct musb_belcarra_instance *musb_belcarra = pThis->pBelcarra_privdata;
	struct dma_channel	*dma;
	dma = is_dma_capable() ? musb_ep->dma : NULL;
        /* return unless USBOTG connected */
        if (!musb_belcarra || !musb_belcarra->ops || !musb_belcarra->ops->tx_irq) return;
        musb_belcarra->ops->tx_irq(musb_belcarra, epnum, dma?1:0);
}
#endif

/* musb_g_dma
 */
void musb_g_dma(struct musb *pThis, u8 index, u32 addr, u32 count, u16 control)
{
        struct musb_belcarra_instance *musb_belcarra = pThis->pBelcarra_privdata;
        /* return unless USBOTG connected */
        if (!musb_belcarra || !musb_belcarra->ops || !musb_belcarra->ops->dma_irq) return;
        musb_belcarra->ops->dma_irq(musb_belcarra, index, addr, count, control);
}


/* Called during SRP. Caller must hold lock */
void musb_g_wakeup(struct musb *musb)
{
//	musb_gadget_wakeup(&musb->g);
}


/* ********************************************************************************************* */
/* ********************************************************************************************* */
/* ********************************************************************************************* */
/* Called from plat_uds.c to pass the pThis pointer for us to use
 */
struct musb *gpThis;
static struct musb_belcarra_instance MUSB_Belcarra;

/* musb_belcarra_setup() - pass pThis pointer from plat_uds
 */
int __init musb_belcarra_setup(struct musb *pThis)
{
        gpThis = pThis;
        pThis->pBelcarra_privdata = (struct musb_belcarra_instance *) &MUSB_Belcarra;
        return 0;
}


/* ********************************************************************************************* */
/* ********************************************************************************************* */
/* Implement the musb_api.h functions
 */

/*!
 * musb_belcarra_cleanup_driver
 */
void musb_belcarra_cleanup_driver(struct musb_belcarra_instance *musb_belcarra)
{
        MUSB_Belcarra.musb_privdata = NULL;
}

/*!
 * musb_belcarra_register_driver
 *
 * Register the belcarra driver. Used by belcarra drivers when
 * registering themselves with the controller.
 *
 * -EINVAL something went wrong (not driver)
 * -EBUSY another belcarra is already using the controller
 * -ENOMEM no memeory to perform the operation
 *
 * @param driver the belcarra driver
 * @return <0 if error, 0 if everything is fine
 */
struct musb_belcarra_instance * musb_belcarra_register_driver(struct musb_ops *ops)
{
        if (MUSB_Belcarra.ops) {
                printk(KERN_INFO"%s: Already registered\n", __FUNCTION__); 
                return NULL;
        }
        if (!gpThis) {
                printk(KERN_INFO"%s: gpThis NULL \n", __FUNCTION__); 
                return NULL;
        }

        MUSB_Belcarra.ops = ops;
        MUSB_Belcarra.musb_privdata = gpThis;
        MUSB_Belcarra.pRegs = gpThis->mregs;    // XXX should not need this
        return &MUSB_Belcarra;
}

EXPORT_SYMBOL(musb_belcarra_register_driver);
EXPORT_SYMBOL(musb_belcarra_cleanup_driver);
/***********************************************************************/

/*
* Program the HDRC to start (enable interrupts, etc.).
*
* This is same as plat_uds.c: musb_start() except that it
* defaults SOFTCONN to disabled and allows control of
* HSENAB.
*/
void _musb_start(struct musb * pThis)
{
        void __iomem *pBase = pThis->mregs;
        u8 state;

	#ifdef CONFIG_PM        
        //if (pThis->asleep)
          //      return;
	#endif /* CONFIG_PM */

        /* TODO: always set ISOUPDATE in POWER (periph mode) and leave it on! */

        /*  Set INT enable registers, enable interrupts */
        musb_writew(pBase, MGC_O_HDRC_INTRTXE, pThis->epmask);
        musb_writew(pBase, MGC_O_HDRC_INTRRXE, pThis->epmask & 0xfffe);
        musb_writeb(pBase, MGC_O_HDRC_INTRUSBE, 0xf7);

        musb_platform_enable(pThis);

        musb_writeb(pBase, MGC_O_HDRC_TESTMODE, 0);

        /* enable high-speed/low-power and start session */
//        #ifdef CONFIG_OTG_HIGH_SPEED
//        #warning HIGH SPEED
        musb_writeb(pBase, MGC_O_HDRC_POWER, MGC_M_POWER_HSENAB);
//        #else
//        #warning FULL SPEED
//        printk(KERN_INFO"********** Forcing Full Speed mode. ************\n");
//        musb_writeb(pBase, MGC_O_HDRC_POWER, 0);
//        #endif

        /* XXX of course this should not be in use for MUSB_HOST at this point */
        switch (pThis->board_mode) {
        case MUSB_HOST:
        case MUSB_OTG:
                musb_writeb(pBase, MGC_O_HDRC_DEVCTL, MGC_M_DEVCTL_SESSION);
                break;
        case MUSB_PERIPHERAL:
                state = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
                musb_writeb(pBase, MGC_O_HDRC_DEVCTL, state & ~MGC_M_DEVCTL_SESSION);
                break;
        }

        DBG(2, "<== power %02x, devctl %02x\n", musb_readb(pBase, MGC_O_HDRC_POWER), musb_readb(pBase, MGC_O_HDRC_DEVCTL));
}



/*!
 * int musb_belcarra_start
 *
 * Start the UDC.
 */
int musb_belcarra_start(struct musb_belcarra_instance *musb_belcarra)
{
        struct musb             *pThis = (struct musb *) musb_belcarra->musb_privdata;

        //printk(KERN_INFO"%s: musb_belcarra: %p musb_belcarra->musb_privdata: %p pThis: %p before musb_start\n", __FUNCTION__, 
        //                musb_belcarra, musb_belcarra->musb_privdata, pThis);

        MUSB_Belcarra.musb_privdata = gpThis;

        if (!musb_belcarra->musb_privdata) {
                printk(KERN_INFO"%s: musb_belcarra: %p musb_belcarra->musb_privdata NULL\n", __FUNCTION__, musb_belcarra); 
                return -EINVAL;
        }

        _musb_start (pThis);

        return 0;
}
EXPORT_SYMBOL(musb_belcarra_start);


/*!
 * int musb_belcarra_stop
 *
 * Stop the UDC.
 */
int musb_belcarra_stop(struct musb_belcarra_instance *musb_belcarra)
{
        struct musb             *musb = (struct musb *) musb_belcarra->musb_privdata;
        u8                      power = musb_readb(musb->mregs, MGC_O_HDRC_POWER);

        if (!musb_belcarra->musb_privdata) return -EINVAL;

        /* disable softcon */
        musb_writeb(musb->mregs, MGC_O_HDRC_POWER, power & ~MGC_M_POWER_SOFTCONN);

        musb_stop (musb);

        return 0;
}
EXPORT_SYMBOL(musb_belcarra_stop);


