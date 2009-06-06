/*
 * otg/hardware/musb-pcd.c -- OMAP USBOTG Peripheral Controller driver
 * @(#) sl@belcarra.com/whiskey.enposte.net|otg/platform/omap/musb-pcd.c|20071005223117|06958
 *
 *      Copyright (c) 2007 Belcarra Technologies 2005 Corp
 *
 * By:
 *      Stuart Lynne <sl@lbelcarra.com>,
 *
 */
/*!
 * @file otg/hardware/musb-pcd.c
 * @brief USB Peripheral Controller Driver
 * This implements the OMAP USBOTG Peripheral Controller Driver.
 *
 * There is no board or platform level code here.
 *
 * @ingroup OMAPHS
 * @ingroup PCD
 */
#include <otg/otg-compat.h>
#include <musb/musb_core.h>
//#include </vobs/linuxjava/ti_kernel/ti_kernel/linux-2.6.x/drivers/usb/musb/musbhdrc.h>

#ifndef MUSB_DEBUG
#define MUSB_DEBUG 0
#endif
//#include </vobs/linuxjava/ti_kernel/ti_kernel/linux-2.6.x/drivers/usb/musb/musbdefs.h>

#include <musb/musb_belcarra_api.h>

#include <linux/dma-mapping.h>

#include <otg/pcd-include.h>
#include <otg/otg-dev.h>

//#include <power_ic/power_ic_kernel.h>

#define MUSB_C_RAM_BITS 12
#define TRACE_VERBOSE 0
#define TRACE_VERY_VERBOSE 0

#define UDC_MAX_ENDPOINTS 16
#define UDC_NAME "arc"
#define EP0_PACKETSIZE 64

#define MGC_SelectEnd(x,y)  musb_ep_select(x,y)

/*
 * USE_DMA_IN           use DMA to fill FIFO
 * USE_BULK_SPLIT       CPU fills FIFO as transferSize, UDC empties as wMaxPacketSize 
 * USE_DOUBLE_IN        allocate two FIFO's
 * 
 * USE_DMA_OUT          use DMA to empty FIFO
 * USE_BULK_COMBINE     UDC fills FIFO as wMaxPacketSize, CPU as transferSize
 * USE_DOUBLE_OUT       allocate two FIFO's
 *
 */

#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,10)
#define channel_abort(c) pfDmaAbortChannel(c)
#define channel_program(c,t,m,d,l) pfDmaProgramChannel(c,t,m,d,l)
#define channel_alloc(c,n,e,a,s) pfDmaAllocateChannel(c,n,e,a,s)
#define channel_release(c) pfDmaReleaseChannel(c)
#endif /* LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10) */



void omaphs_pcd_start_ep0_in(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn, int interrupt);
void omaphs_pcd_start_epn_in(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn, int restart);

void omaphs_pcd_start_ep0_out(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn, int interrupt);
void omaphs_pcd_start_epn_out(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn, int restart);

void omaphs_pcd_finish_epn_out_dma(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn);
void omaphs_pcd_finish_epn_out_pio(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn);

/* Test Mode Support - 
 *      C.f. USB 2.0 Spec 7.1.20 
 *      C.f. Inventra Programmers Guide 21.4 Test Packet
 */
u8 usb_test_packet [53] = {

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
        0xaa, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 
        0xee, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xbf, 0xdf, 
        0xef, 0xf7, 0xfb, 0xfd, 0xfc, 0x7e, 0xbf, 0xdf,
        0xef, 0xf7, 0xfb, 0xfd, 0x7e
};

/* msc_mode indicator, it will be set by msc function driver */
int is_msc_mode = 0;
/* adb_mode idicator, it will be set by adb function driver */
int is_adb_mode = 0;

/* ********************************************************************************************* */
/*! decode_ep0state
 * @param ep0_state -
 * @return state name
 */
static char *decode_ep0stage(musb_ep0_state_t ep0_state)
{
        switch(ep0_state) {
        case MUSB_END0_STAGE_SETUP:      return "STAGE_IDLE";           /* waiting for device request */
        case MUSB_END0_STAGE_TX:         return "STAGE_TX (IN)";        /* waiting for data transmitted interrupt */
        case MUSB_END0_STAGE_RX:         return "STAGE_RX (OUT)";       /* waiting for data received interrupt */
        case MUSB_END0_STAGE_STATUS_IN:   return "STAGE_STATUS_IN";     /* last data sent, wait for IN status */
        case MUSB_END0_STAGE_STATUS_OUT:  return "STAGE_STATUS_OUT";
        case MUSB_END0_STAGE_ACK_WAIT:    return "STAGE_WAIT";
        default:                        return "?";
        }
}
/* ********************************************************************************************* */

/*! musb_belcarra_write_fifo
 * @param musb_belcarra -
 * @param epn -
 * @param wCount -
 * @param pSource -
 * @return none
 */
void musb_belcarra_write_fifo(struct musb_belcarra_instance *musb_belcarra, int epn, u16 wCount, u8 *pSource)
{
        struct musb             *musb = musb_belcarra->musb_privdata; 
        struct otg_dev          *otg_dev = musb_belcarra->usbd_privdata;
        struct pcd_instance     *pcd = otg_dev->pcd_instance;
        TRACE_MSG1(pcd->TAG, "wCount: %d", wCount);
        musb_write_fifo(&musb->endpoints[epn], wCount, pSource);
}

/*! musb_belcarra_read_fifo
 * @param musb_belcarra -
 * @param epn -
 * @param wCount -
 * @param pSource -
 * @return none
 */
void musb_belcarra_read_fifo(struct musb_belcarra_instance *musb_belcarra, int epn, u16 wCount, u8 *pSource)
{
        struct musb             *musb = musb_belcarra->musb_privdata; 
        struct otg_dev          *otg_dev = musb_belcarra->usbd_privdata;
        struct pcd_instance     *pcd = otg_dev->pcd_instance;
        TRACE_MSG1(pcd->TAG, "wCount: %d", wCount);
        musb_read_fifo(&musb->endpoints[epn], wCount, pSource);
}
/* ********************************************************************************************* */

/*! musb_check_speed
 * @param musb_belcarra -
 * @return none
 */
void musb_check_speed(struct musb_belcarra_instance *musb_belcarra)
{
        u8                      power;
        struct otg_dev          *otg_dev = musb_belcarra->usbd_privdata;
        struct pcd_instance     *pcd = otg_dev->pcd_instance;
        struct usbd_bus_instance *bus = pcd->bus;
        void __iomem            *pBase = musb_belcarra->pRegs;

        RETURN_IF(musb_belcarra->speedSet);
        power = musb_readb(pBase, MGC_O_HDRC_POWER);
        bus->high_speed = BOOLEAN(power & MGC_M_POWER_HSMODE);
        musb_belcarra->speedSet = TRUE;
        TRACE_MSG2(pcd->TAG, "power: %04x %s SPEED", power, bus->high_speed ? "HIGH" : "FULL");
}

/* ********************************************************************************************* */
/*! omaphs_epn
 * @param endpoint -
 * @return none
 */
int omaphs_epn(struct usbd_endpoint_instance *endpoint)
{
        return endpoint->bEndpointAddress[0] & 0xf;
}

/* ********************************************************************************************* */

/*! omaphs_pcd_start_epn_in
 * @param musb_belcarra -
 * @param pcd -
 * @param endpoint -
 * @param epn -
 * @param dma -
 * @return none
 */
void omaphs_pcd_start_epn_in(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn, int dma)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct musb             *musb = musb_belcarra->musb_privdata;
        void __iomem            *pBase = musb->mregs;
        int                     wMaxPacketSize = endpoint->wMaxPacketSize[bus->high_speed];
        //int                     transferSize = MIN(endpoint->transferSize[bus->high_speed], MAX_SPLIT_TRANSFERSIZE);
	int                     transferSize;
        struct dma_controller   *dma_controller = musb->dma_controller;
        #ifdef USE_DMA_IN
        struct dma_channel      *dma_channel = (struct dma_channel *)endpoint->privdata;
        #else /* USE_DMA_IN */
        struct dma_channel      *dma_channel = (struct dma_channel *)NULL;
        #endif
        dma_addr_t              dma_addr = DMA_ADDR_INVALID;

	if(is_msc_mode && !is_adb_mode)
	  transferSize = MIN(endpoint->transferSize[bus->high_speed], MSC_TRANSFERSIZE);
	else
	  transferSize = MIN(endpoint->transferSize[bus->high_speed], MAX_SPLIT_TRANSFERSIZE);

        if (TRACE_VERBOSE)
                TRACE_MSG6(pcd->TAG, "[%2x] dma: %d endpoint->state: %x epn: %d wMaxPacketSize: %x transferSize: %x", 
                                epn, dma, endpoint->state, epn, wMaxPacketSize, transferSize);

        for (;;) {
                struct usbd_urb         *tx_urb;
                int                     remaining;
                u16                     wTxCsr;
                u16                     FIFOSize;
                int                     mode;

                /* select and get TX CSR */
                MGC_SelectEnd(pBase, epn);
                wTxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, epn);
                FIFOSize = MGC_ReadCsr16(pBase, MGC_O_HDRC_FIFOSIZE, epn);

                TRACE_MSG8(pcd->TAG, "[%2x] dma: %d wTxCsr: %04x dma_channel: %p bStatus: %d wMaxPacketSize: %d "
                                "transferSize: %d FIFOSize: %x",
                                epn, dma, wTxCsr, dma_channel, dma_channel ? dma_channel->status : 0,
                                wMaxPacketSize, transferSize, FIFOSize);

                /* check for error conditions */
                if (wTxCsr & MGC_M_TXCSR_P_SENTSTALL ) {
                        TRACE_MSG2(pcd->TAG, "[%2x] wTxCsr: %04x STALL EXIT", epn, wTxCsr);
                        return;
                }
                if (wTxCsr & MGC_M_TXCSR_P_UNDERRUN) {
                        TRACE_MSG1(pcd->TAG, "[%2x] UNDERRUN", epn);
                }

                /* return if FIFO is showing as full already */
                RETURN_IF(wTxCsr & MGC_M_TXCSR_TXPKTRDY);

                /* set TxPktRdy for short DMA */
                if (endpoint->state & MUSB_DMA_SHORT) {
                        TRACE_MSG2(pcd->TAG, "[%2x] SHORT DMA last: %d ", epn, endpoint->last);
                        wTxCsr |= MGC_M_TXCSR_TXPKTRDY;
                        wTxCsr &= ~MGC_M_TXCSR_P_UNDERRUN;
                        MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, epn, wTxCsr);
                        endpoint->state &= ~MUSB_DMA_SHORT;

                        /* accelerated return, complete as soon as loaded, not after it is sent */
                        if (endpoint->tx_urb && 
                                        !(endpoint->tx_urb->flags & USBD_URB_SENDZLP) &&
                                        (endpoint->tx_urb->flags & USBD_URB_FAST_FINISH))
                                pcd_tx_complete_irq(endpoint, 0); // XXX should check for errors


                        continue;
                }

                /* normal DMA */
                if (endpoint->state & MUSB_DMA_NORMAL) {
                        endpoint->state &= ~MUSB_DMA_NORMAL;
                        /* DO NOTHING */
                }

                /* abort DMA if still active */
                if (0 && dma_channel && (dma_channel->status == MUSB_DMA_STATUS_BUSY)) {
                        TRACE_MSG1(pcd->TAG, "[%2x] abort DMA", epn);
                        dma_controller->channel_abort(dma_channel);
                }

                /* Return if nothing after finishing current and get pending requet */
                RETURN_UNLESS(tx_urb = pcd_tx_complete_irq(endpoint, 0)); // XXX should check for errors


                /* ZLP or still something to send so continue */
                endpoint->last = pcd_tx_sendzlp(endpoint) ? 0 : MIN(transferSize, tx_urb->actual_length - endpoint->sent);

                /* DMA or PIO */
                THROW_UNLESS (endpoint->last && dma_channel, pio);

                /* Can we get mapped dma address? */
                dma_addr = dma_map_single(musb->controller, tx_urb->buffer + endpoint->sent, endpoint->last, DMA_TO_DEVICE);
                THROW_IF(dma_addr == DMA_ADDR_INVALID, pio);

                /* Program dma */
                if (dma_channel->status != MUSB_DMA_STATUS_FREE) {
                        printk(KERN_INFO"%s: bStatus NOT FREE\n", __FUNCTION__); 
                }
                mode = (endpoint->last < transferSize) ? 0 : 1;
                THROW_UNLESS(dma_controller->channel_program( dma_channel, 
                                        transferSize, mode, dma_addr, endpoint->last), pio);

                /* N.B. the MUSBHSDMA library will set TXPKTRDY after the
                 * DMA completes the transfer of the data into the FIFO.
                 */
                
                switch (mode) {
                        /* short DMA mode */
                case 0:
                        wTxCsr &= ~(MGC_M_TXCSR_AUTOSET | MGC_M_TXCSR_DMAMODE);
                        wTxCsr |= (MGC_M_TXCSR_DMAENAB | MGC_M_TXCSR_MODE);
                        endpoint->state |= MUSB_DMA_SHORT;
                        break;

                        /* normal DMA mode */
                case 1:
                        wTxCsr |= (MGC_M_TXCSR_AUTOSET | MGC_M_TXCSR_DMAENAB | MGC_M_TXCSR_DMAMODE | MGC_M_TXCSR_MODE);
                        endpoint->state |= MUSB_DMA_NORMAL;
                        break;
                default:
                        THROW(pio);
                }

                MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, epn, wTxCsr);

                TRACE_MSG8(pcd->TAG, "[%2x] DMA wTxCsr: %04x mode: %d wMaxPacketSize: %d transferSize: %d "
                                "dma_addr: %p size: %d %s", 
                                epn, wTxCsr, mode, wMaxPacketSize, transferSize, dma_addr, 
                                endpoint->last, (mode ? "" : "SHORT"));

                wTxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, epn);
                TRACE_MSG3(pcd->TAG, "[%2x] DMA wTxCsr: %04x last: %d", epn, wTxCsr, endpoint->last);

                return;
pio:
                /* no dma, no dma channel, map failed or program channel failed 
                 * default to PIO 
                 */
                if (endpoint->last)
                        musb_belcarra_write_fifo(musb_belcarra, epn, endpoint->last, tx_urb->buffer + endpoint->sent);

                /* update wTxCsr */
                wTxCsr |= MGC_M_TXCSR_TXPKTRDY;
                wTxCsr &= ~(MGC_M_TXCSR_P_UNDERRUN | MGC_M_TXCSR_AUTOSET | 
                                MGC_M_TXCSR_DMAMODE | MGC_M_TXCSR_DMAENAB | MGC_M_TXCSR_MODE);

                /* accelerated return, complete as soon as loaded, not after it is sent */
                if (tx_urb->flags & USBD_URB_FAST_FINISH)
                        pcd_tx_complete_irq(endpoint, 0); // XXX should check for errors

                MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, epn, wTxCsr);
                wTxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, epn);

                TRACE_MSG3(pcd->TAG, "[%2x] PIO wTxCsr: %04x last: %d", epn, wTxCsr, endpoint->last);

                /* Loop allowed for PIO */
        }
}

/*! omaphs_pcd_start_endpoint_in
 * @param pcd -
 * @param endpoint -
 * @return none
 */
void omaphs_pcd_start_endpoint_in (struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct otg_instance *otg = pcd->otg;
        struct otg_dev *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        int epn = omaphs_epn (endpoint);
        (epn ? omaphs_pcd_start_epn_in : omaphs_pcd_start_ep0_in) (musb_belcarra, pcd, endpoint, epn, 0);
}

/* ********************************************************************************************* */

/*! omaphs_pcd_start_epn_out
 * @param musb_belcarra -
 * @param pcd -
 * @param endpoint -
 * @param epn -
 * @param restart -
 * @return none
 */
void omaphs_pcd_start_epn_out(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn, int restart)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct musb             *musb = musb_belcarra->musb_privdata;
        void __iomem            *pBase = musb->mregs;
        struct usbd_urb         *rcv_urb;
        struct dma_controller   *dma_controller = musb->dma_controller;
        struct dma_channel      *dma_channel = (struct dma_channel *)endpoint->privdata;
        u16                     wRxCsr;
        u16                     FIFOSize;
        u16                     rxCount;
        u16                     rxMaxp;
        int                     wMaxPacketSize = endpoint->wMaxPacketSize[bus->high_speed];

        int                     transferSize;
        int                     multiplier;

        /* select and get RX CSR */
        MGC_SelectEnd(pBase, epn);
        wRxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, epn);
        FIFOSize = MGC_ReadCsr16(pBase, MGC_O_HDRC_FIFOSIZE, epn);
        rxMaxp = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXMAXP, epn);
        rxCount = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, epn);

        TRACE_MSG8(pcd->TAG, "[%2x] wRxCsr: %04x rxMaxp: %04x rxCount: %04x FIFOSize: %04x wMaxPacketSize: %d %s %s",
                        epn, wRxCsr, rxMaxp, rxCount, FIFOSize, wMaxPacketSize, 
                        (dma_channel ? "DMA" : "PIO"), 
                        (restart ? "RESTART" : "START"));

        /* Get the receive urb, nothing to do otherwise
         */
        UNLESS (rcv_urb = pcd_rcv_next_irq(endpoint)) {
                TRACE_MSG1(pcd->TAG, "[%2x] NO WORK", epn);
                return;
        }

        TRACE_MSG6(pcd->TAG, "[%2x] wRxCsr: %04x rcv_urb: %p buffer_length: %d actual_length: %d alloc_length: %d", 
                        epn, wRxCsr, rcv_urb, rcv_urb->buffer_length, rcv_urb->actual_length, rcv_urb->alloc_length);

        /* Set the new transfer size, done before RXKPTRDY is reset.
         */
	if(is_msc_mode && !is_adb_mode)
	  transferSize = MIN(rcv_urb->buffer_length - rcv_urb->actual_length, MSC_TRANSFERSIZE);
	else
	  transferSize = MIN(rcv_urb->buffer_length - rcv_urb->actual_length, MAX_COMBINE_TRANSFERSIZE);

        multiplier = (transferSize + wMaxPacketSize - 1) / wMaxPacketSize;
        rxMaxp = ((multiplier -1) << 11) | wMaxPacketSize;
        MGC_WriteCsr16(pBase, MGC_O_HDRC_RXMAXP, epn, rxMaxp);

        /* Check if RXPKTRDY is SET, reset it if necessary.
         *
         * We leave RXPKTRDY SET in omaphs_pcd_finish_epn_pio, because we
         * don't know what to set the FIFO transfer size to until we get
         * here and look at the next receive urb buffer length.
         */
        if (wRxCsr & MGC_M_RXCSR_RXPKTRDY) {
                wRxCsr |= MGC_M_RXCSR_P_WZC_BITS;
                wRxCsr &= ~MGC_M_RXCSR_RXPKTRDY;
                MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, epn, wRxCsr);
                wRxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, epn);
                TRACE_MSG2(pcd->TAG, "[%2x] wRxCsr: %04x RESET RXPKTRDY", epn, wRxCsr);
        }

        rxMaxp = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXMAXP, epn);
        wRxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, epn);
        rxCount = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, epn);

        TRACE_MSG6(pcd->TAG, "[%2x] transferSize: %d multiplier: %d rxCount: %d wRxCsr: %04x rxMaxp: %04x", 
                        epn, transferSize, multiplier, rxCount, wRxCsr, rxMaxp);
        return;
}

/*! omaphs_pcd_finish_epn_out_pio
 * @param musb_belcarra -
 * @param pcd -
 * @param endpoint -
 * @param epn -
 * @return none
 *
 * This is called to process a USB OUT endpoint interrupt. This will clear
 * errors, and either start DMA to empty the FIFO or empty the FIFO with
 * PIO.
 */
void omaphs_pcd_finish_epn_out_pio(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct musb             *musb = musb_belcarra->musb_privdata;
        void __iomem            *pBase = musb->mregs;
        struct usbd_urb         *rcv_urb;
        struct dma_controller   *dma_controller = musb->dma_controller;
        struct dma_channel      *dma_channel = (struct dma_channel *)endpoint->privdata;
        u16                     wRxCsr;
        u16                     FIFOSize;
        u16                     rCount;
        u16                     rxMaxp;
        u16                     rxCount;
        int                     wMaxPacketSize = endpoint->wMaxPacketSize[bus->high_speed];

        MGC_SelectEnd(pBase, epn);
        wRxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, epn);
        FIFOSize = MGC_ReadCsr16(pBase, MGC_O_HDRC_FIFOSIZE, epn);
        rxMaxp = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXMAXP, epn);
        rxCount = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, epn);

        TRACE_MSG6(pcd->TAG, "[%2x] wRxCsr: %04x rxMaxp: %04x rxCount: %04x FIFOSize: %04x wMaxPacketSize: %d",
                        epn, wRxCsr, rxMaxp, rxCount, FIFOSize, wMaxPacketSize);

        if (wRxCsr & MGC_M_RXCSR_P_SENTSTALL) {
                
                TRACE_MSG1(pcd->TAG, "[%2x] SENTSTALL", epn);
                wRxCsr |= MGC_M_RXCSR_P_WZC_BITS;
                wRxCsr &= ~MGC_M_RXCSR_P_SENTSTALL;
                MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, epn, wRxCsr); 
                if (dma_channel) {
                        dma_channel->status = MUSB_DMA_STATUS_CORE_ABORT;
                        dma_controller->channel_abort(dma_channel);
                        printk(KERN_INFO"%s: ABORT\n", __FUNCTION__); 
                }
        }                                               
        if (wRxCsr & MGC_M_RXCSR_P_OVERRUN) {          
                TRACE_MSG1(pcd->TAG, "[%2x] OVERRUN", epn);
                // wRxCsr |= MGC_M_RXCSR_P_WZC_BITS;   
                wRxCsr &= ~MGC_M_RXCSR_P_OVERRUN;      
                MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, epn, wRxCsr);
        }
        if (wRxCsr & MGC_M_RXCSR_INCOMPRX) {
                TRACE_MSG1(pcd->TAG, "[%2x] INCOMPRX", epn);
                /* REVISIT not necessarily an error */
        }                               

        UNLESS(wRxCsr & MGC_M_RXCSR_RXPKTRDY) {
                TRACE_MSG1(pcd->TAG, "[%2x] ERROR: RXPKTRDY NOT SET", epn);
                return;
        }

        UNLESS(rcv_urb = pcd_rcv_next_irq(endpoint)) {
                TRACE_MSG1(pcd->TAG, "[%2x] ERROR: no rcv urb", epn);
                return;
        }

        TRACE_MSG5(pcd->TAG, "[%2x] rcv_urb: %p buffer_length: %d actual_length: %d alloc_length: %d", 
                        epn, rcv_urb, rcv_urb->buffer_length, rcv_urb->actual_length, rcv_urb->alloc_length);

        /* get amount of data in FIFO */
        rCount = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, epn);

        TRACE_MSG2(pcd->TAG, "[%2x] rCount: %d", epn, rCount);

        //MGC_WriteCsr16(pBase, MGC_O_HDRC_RXMAXP, epn, 0);

        /* if DMA, then start DMA to empty FIFO */
        if (dma_channel) {
                TRACE_MSG2(pcd->TAG, "[%2x] DMA START rcv_urb->actual_length: %d ", epn, rcv_urb->actual_length);

                /* map dma address */
                rcv_urb->dma_addr = dma_map_single(musb->controller, 
                                rcv_urb->buffer + rcv_urb->actual_length, rCount, DMA_FROM_DEVICE);

                /* have rcv_urb - start RCV DMA */
                wRxCsr |= MGC_M_RXCSR_DMAENAB;
                MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, epn, wRxCsr);

                wRxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, epn);
                TRACE_MSG2(pcd->TAG, "[%2x] DMA wRxCsr: %04x", epn, wRxCsr);

                dma_channel->desired_mode = (rCount < wMaxPacketSize) ? 0 : 1;

                /* start DMA */
                if (dma_channel->status != MUSB_DMA_STATUS_FREE) {
                        TRACE_MSG0(pcd->TAG, "bStatus NOT FREE"); 
                        printk(KERN_INFO"%s: bStatus NOT FREE\n", __FUNCTION__); 
                        return;
                }

                dma_controller->channel_program( dma_channel, wMaxPacketSize, 
                                dma_channel->desired_mode, rcv_urb->dma_addr, rCount);
                endpoint->state |= MUSB_DMA_SHORT;

                /* Wait for DMA complete interrupt */
                return;
        }

        /* default - no DMA, empty FIFO with PIO 
         */
        TRACE_MSG3(pcd->TAG, "[%2x] PIO rCount: %d actual_length: %d READING FIFO", epn, rCount, rcv_urb->actual_length);

        musb_belcarra_read_fifo(musb_belcarra, epn, rCount, rcv_urb->buffer + rcv_urb->actual_length);

        /* We do NOT RESET RXPKTRDY here, we don't know what to set the FIFO
         * transfer size to until we get to omaphs_pcd_start_endpoint_out
         * and look at the next receive urb buffer length. Reseting prior
         * to looking at the next receive urb will allow data to arrive
         * using the possibly incorrect FIFO size. That would cause problems
         * for length based protocols such as mass storage.
         */
        wRxCsr |= MGC_M_RXCSR_P_WZC_BITS;
        MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, epn, wRxCsr);

        rxMaxp = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXMAXP, epn);
        wRxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, epn);

        if (pcd_rcv_complete_irq(endpoint, rCount, 0)) {
                TRACE_MSG3(pcd->TAG, "[%2x] PIO wRxCsr: %04x rxMaxp: %04x RESTARTING", epn, wRxCsr, rxMaxp);
                omaphs_pcd_start_epn_out(musb_belcarra, pcd, endpoint, epn, 1);
                return;
        }

        /* Nothing to do
         */
        TRACE_MSG3(pcd->TAG, "[%2x] PIO wRxCsr: %04x rxMaxp: %04x NOT RESTARTING - NO WORK", epn, wRxCsr, rxMaxp);
}

/*! omaphs_pcd_finish_epn_out_dma 
 * @param musb_belcarra -
 * @param pcd -
 * @param endpoint -
 * @param epn -
 * @return none
 *
 * This is called to process a DMA completion interrupt on OUT endpoint. 
 *
 */
void omaphs_pcd_finish_epn_out_dma(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct musb             *musb = musb_belcarra->musb_privdata;
        void __iomem            *pBase = musb->mregs;
        struct usbd_urb         *rcv_urb;
        struct dma_controller   *dma_controller = musb->dma_controller;
        struct dma_channel      *dma_channel = (struct dma_channel *)endpoint->privdata;
        u16                     wRxCsr;
        u16                     rxMaxp;
        u16                     rxCount;
        u16                     FIFOSize;
        u16                     dmaCount;
        int                     wMaxPacketSize = endpoint->wMaxPacketSize[bus->high_speed];

        /* if DMA was previously started then terminate
         */
        UNLESS (endpoint->state & MUSB_DMA_SHORT) {
                TRACE_MSG1(pcd->TAG, "[%2x] ERROR: not DMA", epn);
                return;
        }

        /* select and get RX CSR */
        MGC_SelectEnd(pBase, epn);
        wRxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, epn);
        FIFOSize = MGC_ReadCsr16(pBase, MGC_O_HDRC_FIFOSIZE, epn);
        rxMaxp = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXMAXP, epn);
        rxCount = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, epn);

        TRACE_MSG6(pcd->TAG, "[%2x] wRxCsr: %04x rxMaxp: %04x rxCount: %04x FIFOSize: %04x wMaxPacketSize: %d",
                        epn, wRxCsr, rxMaxp, rxCount, FIFOSize, wMaxPacketSize);
        
        /* recover length and unmap 
         */
        dmaCount = dma_channel->actual_len;
        if ((rcv_urb = endpoint->rcv_urb) && (rcv_urb->dma_addr != DMA_ADDR_INVALID)) {
                TRACE_MSG2(pcd->TAG, "[%2x] DMA UNMAP dmaCount: %d", epn, dmaCount);
                dma_unmap_single(musb->controller, rcv_urb->dma_addr, dmaCount, DMA_FROM_DEVICE);
                rcv_urb->dma_addr = DMA_ADDR_INVALID;
        }

        TRACE_MSG4(pcd->TAG, "[%2x] DMA COMPLETE rcv_urb: %p dmaCount: %d RXCOUNT: %d", 
                        epn, rcv_urb, dmaCount, MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, epn));

        /* We do NOT RESET RXPKTRDY here, see notes above in
         * omaps_pcd_finish_endpoint_out.
         */
        wRxCsr &= ~(MGC_M_RXCSR_AUTOCLEAR | MGC_M_RXCSR_DMAENAB | MGC_M_RXCSR_DMAMODE /*| MGC_M_RXCSR_RXPKTRDY*/);
        MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, epn, wRxCsr);

        endpoint->state &= ~MUSB_DMA_SHORT;

        rxMaxp = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXMAXP, epn);
        wRxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, epn);

        if (pcd_rcv_complete_irq(endpoint, dmaCount, 0)) {
                TRACE_MSG3(pcd->TAG, "[%2x] DMA wRxCsr: %04x rxMaxp: %04x RESTARTING", epn, wRxCsr, rxMaxp);
                omaphs_pcd_start_epn_out(musb_belcarra, pcd, endpoint, epn, 1);
                return;
        }

        TRACE_MSG3(pcd->TAG, "[%2x] DMA wRxCsr: %04x rxMaxp: %04x NOT RESTARTING - NO WORK", epn, wRxCsr, rxMaxp);
}

/*! omaphs_pcd_start_endpoint_out - start receive
 * @param pcd -
 * @param endpoint -
 * @return none
 */
void omaphs_pcd_start_endpoint_out (struct pcd_instance *pcd,struct usbd_endpoint_instance *endpoint)
{
        struct otg_instance *otg = pcd->otg;
        struct otg_dev *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        int epn = omaphs_epn (endpoint);
        (epn ?  omaphs_pcd_start_epn_out : omaphs_pcd_start_ep0_out) (musb_belcarra, pcd, endpoint, epn, 0);
}

/* ********************************************************************************************* */
/*! omaphs_pcd_start_ep0_in
 * @param musb_belcarra -
 * @param pcd -
 * @param endpoint -
 * @param epn -
 * @param interrupt -
 * @return none
 */
void omaphs_pcd_start_ep0_in(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn, int interrupt)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct musb             *musb = musb_belcarra->musb_privdata;
        void __iomem            *pBase = musb->mregs;
        int                     wMaxPacketSize = endpoint->wMaxPacketSize[bus->high_speed];
        struct usbd_urb         *tx_urb;
        u16                     wCsrVal = MGC_M_CSR0_TXPKTRDY;

        RETURN_UNLESS((tx_urb = endpoint->tx_urb));

        TRACE_MSG5(pcd->TAG, "tx_urb: %p sent: %d len: %d flags: %x interrupt: %d", 
                        tx_urb, endpoint->sent, tx_urb->actual_length, tx_urb->flags, interrupt);
        //RETURN_UNLESS(interrupt);

        /* Fill FIFO - note that if ZLP required we will do extra zero length fill and send */
        endpoint->last = pcd_tx_sendzlp(endpoint) ? 0 : MIN(wMaxPacketSize, tx_urb->actual_length - endpoint->sent);
        musb_belcarra_write_fifo(musb_belcarra, 0, endpoint->last, tx_urb->buffer + endpoint->sent);

        /* tell UDC about it... */
        if ((endpoint->last < wMaxPacketSize) || 
                        ((endpoint->last == tx_urb->actual_length) && !(tx_urb->flags & USBD_URB_SENDZLP))
                        ) 
        {
                TRACE_MSG1(pcd->TAG, "SEND: %d FINISHED", endpoint->last);
                MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_TXPKTRDY | MGC_M_CSR0_P_DATAEND);
                musb_belcarra->ep0_state = MUSB_END0_STAGE_STATUS_IN;
                return;
        }

        /* normal */
        TRACE_MSG1(pcd->TAG, "SEND: %d NOT FINISHED", endpoint->last);
        MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_TXPKTRDY);
        musb_belcarra->ep0_state = MUSB_END0_STAGE_TX;

}

/*! omaphs_pcd_start_ep0_out
 * @param musb_belcarra -
 * @param pcd -
 * @param endpoint -
 * @param epn -
 * @param interrupt -
 * @return none
 */
void omaphs_pcd_start_ep0_out(struct musb_belcarra_instance *musb_belcarra, 
                struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int epn, int interrupt)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct musb             *musb = musb_belcarra->musb_privdata;
        void __iomem            *pBase = musb->mregs;
        int                     wMaxPacketSize = endpoint->wMaxPacketSize[bus->high_speed];
        struct usbd_urb         *rcv_urb;
        int                     remaining;
        u16                     rCsrVal;
        u16                     rCount;

        RETURN_UNLESS((rcv_urb = endpoint->tx_urb));

        MGC_SelectEnd(pBase, 0);        /* select ep0 */
        rCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0);
        rCount = MGC_ReadCsr8(pBase, MGC_O_HDRC_COUNT0, 0);

        TRACE_MSG7(pcd->TAG, "ENTER ----> rCsrVal: %04x, rCount: %d, myaddr: %d, %s (%d) interrupt: %d %s SPEED", rCsrVal, rCount,
                        musb_readb(pBase, MGC_O_HDRC_FADDR), decode_ep0stage(musb_belcarra->ep0_state),
                        musb_belcarra->ep0_state, interrupt, bus->high_speed ? "HIGH" : "FULL");

        switch (musb_belcarra->ep0_state) {
        case MUSB_END0_STAGE_RX:
                break;
        default:
                return;
        }
        RETURN_UNLESS (rCsrVal & MGC_M_CSR0_RXPKTRDY);

        rCount = MGC_ReadCsr8(pBase, MGC_O_HDRC_COUNT0, 0);

        musb_belcarra_read_fifo(musb_belcarra, 0, rCount, rcv_urb->buffer + rcv_urb->actual_length);

        MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_P_SVDRXPKTRDY);
        musb_belcarra->ep0_state = MUSB_END0_STAGE_STATUS_IN;

        pcd_rcv_complete_irq(endpoint, rCount, 0);
        TRACE_MSG2(pcd->TAG, "[%2x] PIO rCsrVal: %04x RESTARTING", epn, rCsrVal);
}

/*! musb_ep0_irq
 * @param musb_belcarra -
 * @return irq handled
 *
 * Called by hardware support to handle an EP0 interrupt
 */
int musb_ep0_irq(struct musb_belcarra_instance *musb_belcarra)
{
        struct musb             *musb = musb_belcarra->musb_privdata; 
        struct otg_dev          *otg_dev = musb_belcarra->usbd_privdata;
        struct otg_instance     *otg = otg_dev->otg_instance;
        struct pcd_instance     *pcd = otg_dev->pcd_instance;
        struct usbd_bus_instance *bus = pcd->bus;
        struct usbd_endpoint_instance *endpoint = bus->endpoint_array + 0;
        struct usbd_urb         *urb = NULL;
        u16                     wCsrVal;
        u16                     wCount;
        void __iomem            *pBase = musb_belcarra->pRegs;
        irqreturn_t             retval = IRQ_NONE;
        struct usbd_device_request request;
        int                     handled = 0;
        
        otg->interrupts++;

        MGC_SelectEnd(pBase, 0);        /* select ep0 */
        wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0);
        wCount = MGC_ReadCsr8(pBase, MGC_O_HDRC_COUNT0, 0);

        TRACE_MSG6(pcd->TAG, "ENTER ----> wCsrVal: %04x, wCount: %d, myaddr: %d, %s (%d) %s SPEED", wCsrVal, wCount,
                        musb_readb(pBase, MGC_O_HDRC_FADDR), decode_ep0stage(musb_belcarra->ep0_state),
                        musb_belcarra->ep0_state, bus->high_speed ? "HIGH" : "FULL");

        /* errors - previously stalled or premature setup end */
        if (wCsrVal & (MGC_M_CSR0_P_SENTSTALL | MGC_M_CSR0_P_SETUPEND)) {

                /* Finish STALL with ACK */
                if (wCsrVal & MGC_M_CSR0_P_SENTSTALL) {
                        TRACE_MSG0(pcd->TAG, "SENTSTALL - finish with ACK"); 
                        MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsrVal & ~MGC_M_CSR0_P_SENTSTALL);
                        retval = IRQ_HANDLED;
                        musb_belcarra->ep0_state = MUSB_END0_STAGE_SETUP;
                        wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0);
                }
                /* premature setup end */
                if (wCsrVal & MGC_M_CSR0_P_SETUPEND) {
                        TRACE_MSG0(pcd->TAG, "PREMATURE END - cleanup and go to SETUP"); 
                        MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_P_SVDSETUPEND);
                        wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0);
                        TRACE_MSG1(pcd->TAG, "wCsrVal: %04x", wCsrVal);
                        retval = IRQ_HANDLED;
                        musb_belcarra->ep0_state = MUSB_END0_STAGE_SETUP;
                        wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0);
                }
        }

        /* pre-processing - handle things that may be in progress 
         * This may change the ep0_state
         */
        TRACE_MSG0(pcd->TAG, "PRE-PROCESSING"); 
        switch (musb_belcarra->ep0_state) {
        
        case MUSB_END0_STAGE_TX:
                TRACE_MSG0(pcd->TAG, "STAGE TX RESTART (PRE)"); 
                if ((urb = pcd_tx_complete_irq(endpoint, 0))) {
                        omaphs_pcd_start_ep0_in(musb_belcarra, pcd, endpoint, 0, 1);
                        return IRQ_HANDLED;
                }
                /* irq on clearing txpktrdy */
                if ((wCsrVal & MGC_M_CSR0_TXPKTRDY) == 0) {
                        MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_P_SVDRXPKTRDY);
                        return IRQ_HANDLED;
                }
                return IRQ_HANDLED;

        case MUSB_END0_STAGE_RX:
                TRACE_MSG0(pcd->TAG, "STAGE RX RESTART (PRE)"); 
                /* irq on set rxpktrdy */
                if (wCsrVal & MGC_M_CSR0_RXPKTRDY) {
                        // XXX restart 
                        //
                        omaphs_pcd_start_ep0_out(musb_belcarra, pcd, endpoint, 0, 1);
                        return IRQ_HANDLED;
                }
                //MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_P_SVDRXPKTRDY);
                return IRQ_HANDLED;

        case MUSB_END0_STAGE_ACK_WAIT:
                TRACE_MSG0(pcd->TAG, "STAGE ACK_WAIT (PRE)"); 
                /* update address (if needed) only @ the end of the
                 * status phase per usb spec, which also guarantees
                 * we get 10 msec to receive this irq... until this
                 * is done we won't see the next packet.
                 */
                if (musb_belcarra->setAddress) {
                        //TRACE_MSG1(pcd->TAG, "ADDRESS SET: %d (PRE)", musb_belcarra->bAddress); 
                        musb_belcarra->setAddress = FALSE;
                        musb_writeb(pBase, MGC_O_HDRC_FADDR, musb_belcarra->bAddress);
                }
                if (musb_belcarra->testMode) {
                        switch (musb_belcarra->testModeSelector) {
                        case MGC_M_TEST_PACKET:
                                TRACE_MSG0(pcd->TAG, "Loading test packet");
                                musb_belcarra_write_fifo(musb_belcarra, 0, sizeof(usb_test_packet), usb_test_packet);
                                MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_TXPKTRDY);
                                //MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_TXPKTRDY | MGC_M_CSR0_P_DATAEND);
                                break;
                        }
                        TRACE_MSG1(pcd->TAG, "selector: %d", musb_belcarra->testModeSelector);
                        musb_writeb(pBase, MGC_O_HDRC_TESTMODE, musb_belcarra->testModeSelector);
                        musb_belcarra->testMode = FALSE;
                }
                musb_belcarra->ep0_state = MUSB_END0_STAGE_SETUP;
                retval = IRQ_HANDLED;
                break;

        case MUSB_END0_STAGE_STATUS_IN:
                TRACE_MSG0(pcd->TAG, "STAGE STATUS_IN (PRE)"); 
                /* finish pending requests */
                pcd_tx_complete_irq(endpoint, 0);
                musb_belcarra->ep0_state = MUSB_END0_STAGE_SETUP;
                retval = IRQ_HANDLED;
                break;

        case MUSB_END0_STAGE_STATUS_OUT:
                pcd_rcv_cancelled_irq(endpoint);
                TRACE_MSG0(pcd->TAG, "STAGE STATUS_OUT (PRE)"); 
                /* end of sequence #1: write to host (TX state) */
                musb_belcarra->ep0_state = MUSB_END0_STAGE_SETUP;
                retval = IRQ_HANDLED;
                break;

        case MUSB_END0_STAGE_SETUP:
                TRACE_MSG0(pcd->TAG, "STAGE SETUP (PRE) - Do nothing"); 
                retval = IRQ_HANDLED;
                break;

        default:
                TRACE_MSG0(pcd->TAG, "STAGE UNKNOWN - NOT HANDLED"); 
                retval = IRQ_NONE;
                break;
        }

        UNLESS (musb_belcarra->ep0_state == MUSB_END0_STAGE_SETUP) {
                /* "can't happen" */
                TRACE_MSG0(pcd->TAG, "UNKNOWN - CANNOT HAPPEN - STALLING");
                MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_P_SENDSTALL);
                musb_belcarra->ep0_state = MUSB_END0_STAGE_SETUP;
                THROW(finished);
        }

        /* Setup Main processing - this may (probably will) change ep0_state */
        THROW_UNLESS (wCsrVal & MGC_M_CSR0_RXPKTRDY, finished);

        /* in case RESET was not reported */
        musb_check_speed(musb_belcarra);
        musb_belcarra->setAddress = musb_belcarra->testMode = FALSE;
        retval = IRQ_HANDLED;

        if (wCount != 8) {
                TRACE_MSG1(pcd->TAG, "length: %d != 8", wCount);
                THROW(finished);
        }

        /* read device request and clear RXPKTRDY */
        musb_belcarra_read_fifo(musb_belcarra, 0, wCount, (u8 *)&request);

        /* determine next stage by looking a length and direction */
        musb_belcarra->ep0_state = request.wLength ? 
                ((request.bmRequestType & USB_DIR_IN) ?  MUSB_END0_STAGE_TX : MUSB_END0_STAGE_RX) : MUSB_END0_STAGE_ACK_WAIT;

        TRACE_MSG2(pcd->TAG, "new stage %s (%d)", decode_ep0stage(musb_belcarra->ep0_state), musb_belcarra->ep0_state);

        switch (musb_belcarra->ep0_state) {

                /* no data stage */
        case MUSB_END0_STAGE_ACK_WAIT:
                TRACE_MSG0(pcd->TAG, "ACK_WAIT - no data stage"); 
                break;

                /* sending data back */
        case MUSB_END0_STAGE_TX:
                TRACE_MSG0(pcd->TAG, "STAGE TX - setup for sending"); 
                MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_P_SVDRXPKTRDY);
                while ((MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0) & MGC_M_CSR0_RXPKTRDY) != 0) {
                        TRACE_MSG0(pcd->TAG, "STAGE TX - RXPKTRDY set"); 
                        cpu_relax();
                }
                break;

                /* receiving data */
        case MUSB_END0_STAGE_RX:
                TRACE_MSG0(pcd->TAG, "STAGE RX - setup for receiving"); 
                MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_P_SVDRXPKTRDY);
                break;
        default:
                TRACE_MSG1(pcd->TAG, "STAGE UNKNOWN (%d) - error before processing", musb_belcarra->ep0_state); 
                break;
        }

        /* process setup reqeust */
        TRACE_MSG0(pcd->TAG, "PROCESS REQUEST)"); 
        if (pcd_recv_setup_irq(pcd, &request)) {
                /* stall */
                TRACE_MSG0(pcd->TAG, "BAD REQUEST - STALLING"); 
                musb_belcarra->ep0_state = MUSB_END0_STAGE_SETUP;
                MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_P_SENDSTALL);
                return IRQ_HANDLED;
        }

        TRACE_MSG0(pcd->TAG, "Post-PROCESSING"); 
        /* post-processing - by this time IN or OUT urb will have been queued.
         */
        switch (musb_belcarra->ep0_state) {
                /* no data stage */
        case MUSB_END0_STAGE_ACK_WAIT:
                TRACE_MSG0(pcd->TAG, "ACK_WAIT - no data stage (POST)"); 
                MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_P_SVDRXPKTRDY | MGC_M_CSR0_P_DATAEND);
                return IRQ_HANDLED;

                /* sending data back */
        case MUSB_END0_STAGE_TX:
                TRACE_MSG0(pcd->TAG, "STAGE TX - do nothing (POST)"); 
                return IRQ_HANDLED;

        case MUSB_END0_STAGE_STATUS_IN:
                TRACE_MSG0(pcd->TAG, "STAGE STATUS_IN - do nothing (POST)"); 
                return IRQ_HANDLED;

                /* receiving data */
        case MUSB_END0_STAGE_RX:
                TRACE_MSG0(pcd->TAG, "STAGE RX - do nothing (POST)"); 
                omaphs_pcd_start_ep0_out(musb_belcarra, pcd, endpoint, 0, 1);
                return IRQ_HANDLED;

        case MUSB_END0_STAGE_STATUS_OUT:
                TRACE_MSG0(pcd->TAG, "STAGE STATUS_OUT - do nothing (POST)"); 
                return IRQ_HANDLED;

        default:
                TRACE_MSG1(pcd->TAG, "STAGE UNKNOWN (%d) - error after processing (POST)", musb_belcarra->ep0_state); 
                break;
        }
        return IRQ_HANDLED;
        CATCH(finished) {
                return retval;
        }
}

/*! musb_rx_irq
 * @param musb_belcarra -
 * @param epn
 * @param dma
 *
 * Called by hardware support to handle an data endpoint receiving data.
 */
int musb_rx_irq(struct musb_belcarra_instance *musb_belcarra, int epn, int dma)
{
        struct otg_dev          *otg_dev = musb_belcarra->usbd_privdata;
        struct otg_instance     *otg = otg_dev->otg_instance;
        struct pcd_instance     *pcd = otg_dev->pcd_instance;
        struct usbd_bus_instance *bus = pcd->bus;
        struct usbd_endpoint_instance *endpoint = bus->endpoint_array + (2 * epn);
        otg->interrupts++;
        (dma ? omaphs_pcd_finish_epn_out_dma : omaphs_pcd_finish_epn_out_pio) (musb_belcarra, pcd, endpoint, epn);
        return 0;
}

/*! musb_tx_irq
 * @param musb_belcarra -
 * @param epn
 * @param dma
 *
 * Called by hardware support to handle an data endpoint sending data.
 */
int musb_tx_irq(struct musb_belcarra_instance *musb_belcarra, int epn, int dma)
{
        struct otg_dev          *otg_dev = musb_belcarra->usbd_privdata;
        struct otg_instance     *otg = otg_dev->otg_instance;
        struct pcd_instance     *pcd = otg_dev->pcd_instance;
        struct usbd_bus_instance *bus = pcd->bus;
        struct usbd_endpoint_instance *endpoint = bus->endpoint_array + (2 * epn + 1);
        otg->interrupts++;
        TRACE_MSG2(pcd->TAG, "dma: %d epn: %d", dma, epn);
        omaphs_pcd_start_epn_in(musb_belcarra, pcd, endpoint, epn, dma);
        return 0;
}

/*! musb_dma_irq
 * @param musb_belcarra -
 * @param index
 * @param addr
 * @param count
 * @param control
 *
 * Called by hardware support to handle an data endpoint sending data.
 */
int musb_dma_irq(struct musb_belcarra_instance *musb_belcarra, u8 index, u32 addr, u32 count, u16 control)
{
        struct otg_dev          *otg_dev = musb_belcarra->usbd_privdata;
        struct otg_instance     *otg = otg_dev->otg_instance;
        struct pcd_instance     *pcd = otg_dev->pcd_instance;
        otg->interrupts++;
        TRACE_MSG4(pcd->TAG, "index: %d addr: %x count: %d control: %04x", index, addr, count, control);
        return 0;
}

/*! musb_usb_event
 * @param musb_belcarra -
 * @param event
 *
 * Called by hardware support to handle an USB event.
 */
int musb_usb_event(struct musb_belcarra_instance *musb_belcarra, musb_belcarra_usb_event_t event)
{
        struct otg_dev          *otg_dev = musb_belcarra ? musb_belcarra->usbd_privdata : NULL;
        struct otg_instance     *otg = otg_dev ? otg_dev->otg_instance : NULL;
        struct pcd_instance     *pcd = otg_dev ? otg_dev->pcd_instance : NULL;
        struct usbd_bus_instance *bus = pcd ? pcd->bus : NULL;
        struct musb             *musb = musb_belcarra ? musb_belcarra->musb_privdata : NULL;
        void __iomem            *pBase = musb ? musb->mregs : NULL;
        u8                      devctl;

        otg->interrupts++;

        switch (event) {
        case musb_reset:
                TRACE_MSG1(pcd->TAG, "RESET event: %d", event);
                devctl =  musb_readb(pBase, MGC_O_HDRC_DEVCTL);
                if (devctl & MGC_M_DEVCTL_HR)
                        musb_writeb(pBase, MGC_O_HDRC_DEVCTL, MGC_M_DEVCTL_SESSION);
                musb_belcarra->speedSet = FALSE;
                musb_belcarra->testMode = FALSE;
                musb_check_speed(musb_belcarra);
                pcd_bus_event_handler_irq (pcd->bus, DEVICE_RESET, 0);
                pcd_bus_event_handler_irq (pcd->bus, DEVICE_ADDRESS_ASSIGNED, 0);
                break;

        case musb_suspend:
                TRACE_MSG1(pcd->TAG, "SUSPEND event: %d", event);
                pcd_bus_event_handler_irq (pcd->bus, DEVICE_BUS_INACTIVE, 0);
                break;

        case musb_resumed:
                TRACE_MSG1(pcd->TAG, "RESUME event: %d", event);
                if (musb_belcarra->resumed) {
                        u8 power = musb_readb(pBase, MGC_O_HDRC_POWER);
                        musb_writeb(pBase, MGC_O_HDRC_POWER, power & ~MGC_M_POWER_RESUME);
                        musb_belcarra->resumed = FALSE;
                }
                pcd_bus_event_handler_irq (pcd->bus, DEVICE_BUS_ACTIVITY, 0);
                break;

        case musb_disconnect:
                TRACE_MSG1(pcd->TAG, "DISCONNECT event: %d", event);
                musb_belcarra->speedSet = FALSE;
                break;

        case musb_stop_activity:
                TRACE_MSG1(pcd->TAG, "STOP ACTIVITY event: %d", event);
                musb_belcarra->speedSet = TRUE;
                break;
        }
        return 0;
}

/* ********************************************************************************************* */
/*! omaphs_pcd_set_endpoints() - setup the physical endpoints for the endpoint map
 * @param pcd
 * @param endpointsRequested
 * @param endpoint_map_array
 *
 * This is called after set configuration.
 */
static int
omaphs_pcd_set_endpoints (struct pcd_instance *pcd, int endpointsRequested, struct usbd_endpoint_map *endpoint_map_array)
{
        struct otg_instance *otg = pcd ? pcd->otg : NULL;
        struct otg_dev *otg_dev = otg ? otg->privdata : NULL;
        struct musb_belcarra_instance *musb_belcarra = otg_dev ? otg_dev->privdata : NULL;
        int i;
        
        TRACE_MSG4(pcd->TAG, "pcd: %x otg: %x otg_dev: %x musb_belcarra: %x", pcd, otg, otg_dev, musb_belcarra);;
        return 0;
}


/*! omaphs_pcd_cancel_urb - cancel urb
 * @param pcd
 * @param urb
 */
void omaphs_pcd_cancel_urb (struct pcd_instance *pcd, struct usbd_urb *urb)
{
        struct otg_instance     *otg = pcd->otg;
        struct otg_dev          *otg_dev = otg->privdata;
        struct usbd_endpoint_instance *endpoint = urb->endpoint;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        struct musb             *musb = musb_belcarra->musb_privdata;
        struct dma_controller   *dma_controller = musb->dma_controller;
        struct dma_channel      *dma_channel = (struct dma_channel *)endpoint->privdata;

        TRACE_MSG0(pcd->TAG, "--");

        if (dma_channel) 
                dma_controller->channel_abort(dma_channel);
}

/*! omaphs_pcd_cancel_in_irq - cancel IN urb
 * @param pcd
 * @param urb
 */
void omaphs_pcd_cancel_in_irq (struct pcd_instance *pcd, struct usbd_urb *urb)
{
        omaphs_pcd_cancel_urb(pcd, urb);
}

/*! omaphs_pcd_cancel_out_irq - cancel OUT urb
 * @param pcd
 * @param urb
 */
void omaphs_pcd_cancel_out_irq (struct pcd_instance *pcd, struct usbd_urb *urb)
{
        omaphs_pcd_cancel_urb(pcd, urb);
}

/*! omaphs_endpoint_halted() - is endpoint halted
 * @param pcd
 * @param endpoint
 */
static int
omaphs_endpoint_halted (struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct otg_instance     *otg = pcd->otg;
        struct otg_dev          *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        struct musb             *musb = musb_belcarra->musb_privdata;
        void __iomem            *pBase = musb->mregs;
        int                     epn = endpoint->bEndpointAddress[bus->high_speed] & 0x7f;
        int                     dir = (endpoint->bEndpointAddress[bus->high_speed] & 0x80) ? 1 : 0;
        u16                     wTxCsr;
        u16                     wRxCsr;

        switch (dir) {
        case USB_DIR_IN:
                wTxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, epn);
                TRACE_MSG3(pcd->TAG, "epn: %d dir: %d wTxCsr: %04x USB_DIR_IN", epn, dir, wTxCsr);
                return BOOLEAN(wTxCsr & MGC_M_CSR0_P_SENDSTALL);
        case USB_DIR_OUT:
                wRxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, epn);
                TRACE_MSG3(pcd->TAG, "epn: %d dir: %d wRxCsr: %04x USB_DIR_OUT", epn, dir, wRxCsr);
                return BOOLEAN(wRxCsr & MGC_M_CSR0_P_SENDSTALL);
        }
        return 0;
}

/*! omaphs_halt_endpoint - halt endpoint
 * @param pcd
 * @param endpoint
 * @param flag
 */
int omaphs_halt_endpoint(struct pcd_instance *pcd, struct usbd_endpoint_instance *endpoint, int flag)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct otg_instance     *otg = pcd->otg;
        struct otg_dev          *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        struct musb             *musb = musb_belcarra->musb_privdata;
        void __iomem            *pBase = musb->mregs;
        int                     epn = endpoint->bEndpointAddress[bus->high_speed] & 0x3f;
        int                     dir = endpoint->bEndpointAddress[bus->high_speed] & USB_DIR_IN;
        u16                     wTxCsr;
        u16                     wRxCsr;

        switch (dir) {
        case USB_DIR_IN:
                wTxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, epn);
                wTxCsr = flag ? (wTxCsr | MGC_M_CSR0_P_SENDSTALL) : (wTxCsr & ~MGC_M_CSR0_P_SENDSTALL);
                MGC_WriteCsr16(musb->mregs, MGC_O_HDRC_TXCSR, epn, wTxCsr);
                TRACE_MSG5(pcd->TAG, "epn: %d dir: %d flag: %d wTxCsr: %04x USB_DIR_IN %s", epn, dir, flag, wTxCsr, 
                                flag ? "STALL" : "CLEAR");
                break;
        case USB_DIR_OUT:
                wRxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, epn);
                wRxCsr =  flag ?  (wRxCsr | MGC_M_CSR0_P_SENDSTALL) : (wRxCsr & ~MGC_M_CSR0_P_SENDSTALL);
                MGC_WriteCsr16(musb->mregs, MGC_O_HDRC_RXCSR, epn, wRxCsr);
                TRACE_MSG5(pcd->TAG, "epn: %d dir: %d flag: %d wRxCsr: %04x USB_DIR_OUT %s", epn, dir, flag, wRxCsr,
                                flag ? "STALL" : "CLEAR");
                break;
        }
        return 0;
}

/*! omaphs_device_feature - device feature
 * @param pcd
 * @param selector
 * @param flag
 */
int omaphs_device_feature(struct pcd_instance *pcd, int selector, int flag)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct otg_instance     *otg = pcd->otg;
        struct otg_dev          *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;

        TRACE_MSG1(pcd->TAG, "selector: %d", selector);

        switch (selector) {
        case USB_TEST_PACKET:
                musb_belcarra->testModeSelector = MGC_M_TEST_PACKET;
                break;
        case USB_TEST_J:
                musb_belcarra->testModeSelector = MGC_M_TEST_J ;
                break;
        case USB_TEST_K:
                musb_belcarra->testModeSelector = MGC_M_TEST_K;
                break;
        case USB_TEST_SE0_NAK:
                musb_belcarra->testModeSelector = MGC_M_TEST_SE0_NAK;
                break;
        default:
                return -EINVAL;
        }
        musb_belcarra->testMode = TRUE;
        return 0;
}

/*! omaphs_pcd_disable_ep -
 * @param pcd
 * @param ep
 * @param endpoint
 */
void omaphs_pcd_disable_ep(struct pcd_instance *pcd, unsigned int ep, struct usbd_endpoint_instance *endpoint)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct otg_instance     *otg = pcd->otg;
        struct otg_dev          *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        struct musb             *musb = musb_belcarra->musb_privdata; 
        struct dma_controller   *dma_controller = musb->dma_controller;
        void __iomem            *pBase = musb->mregs;
        struct dma_channel      *dma_channel = (struct dma_channel *)endpoint->privdata;
        u8                      hs = ep ? bus->high_speed : 0;  /* default to FULL for control */
        u8                      bmAttributes = endpoint->bmAttributes[hs];
        u8                      bEndpointAddress = endpoint->bEndpointAddress[hs];

        u8                      epn = bEndpointAddress & 0x3f;

        TRACE_MSG0(pcd->TAG, "--");
        UNLESS (ep) {
                /* configure control endpoint */
                return;
        }

        /* check if DMA channel was allocated */
        RETURN_UNLESS(dma_channel);

        /* flush */
        switch(bEndpointAddress & USB_DIR_IN) {
        case USB_DIR_IN:
                MGC_WriteCsr16(musb->mregs, MGC_O_HDRC_TXCSR, epn, 0 | MGC_M_TXCSR_FLUSHFIFO);
                MGC_WriteCsr16(musb->mregs, MGC_O_HDRC_TXCSR, epn, 0 | MGC_M_TXCSR_FLUSHFIFO);
                break;
        case USB_DIR_OUT:
                MGC_WriteCsr16(musb->mregs, MGC_O_HDRC_RXCSR, epn, 0 | MGC_M_RXCSR_FLUSHFIFO);
                MGC_WriteCsr16(musb->mregs, MGC_O_HDRC_RXCSR, epn, 0 | MGC_M_RXCSR_FLUSHFIFO);
                break;
        default:
                return;
        }

        /* abort and release */
        dma_controller->channel_abort(dma_channel);
        dma_controller->channel_release(dma_channel);

        endpoint->privdata = NULL;
}

/*! omaphs_pcd_disable -
 * @param pcd
 */
void omaphs_pcd_disable(struct pcd_instance *pcd)
{
        struct otg_instance     *otg = pcd->otg;
        struct otg_dev          *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        struct musb             *musb = musb_belcarra->musb_privdata; 
        struct dma_controller   *dma_controller = musb->dma_controller;
        int i;
        TRACE_MSG0(pcd->TAG, "--");
}

/*! omaphs_pcd_setup_ep - setup endpoint 
 * @param pcd -
 * @param ep -
 * @param endpoint
 * @return none
 */
void omaphs_pcd_setup_ep (struct pcd_instance *pcd, unsigned int ep, struct usbd_endpoint_instance *endpoint)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct otg_instance     *otg = pcd->otg;
        struct otg_dev          *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        struct musb             *musb = musb_belcarra->musb_privdata; 
        void __iomem            *pBase = musb->mregs;
        struct dma_controller   *dma_controller = musb->dma_controller;
        struct dma_channel      *dma_channel = NULL;
        u8                      hs = ep ? (bus->high_speed ? 1 : 0) : 0;  /* default to FULL for control */

        int                     size = 0;
        u8                      c_size = 0;
        u16                     new_ep_offset;
        u16                     wRxCsr;
        u16                     wMaxp;

        u8                      physicalEndpoint = endpoint->physicalEndpoint[hs];
        u8                      bEndpointAddress = endpoint->bEndpointAddress[hs];
        u8                      bmAttributes = endpoint->bmAttributes[hs];
        u8                      epnum = bEndpointAddress & 0x3f;
        u16                     wMaxPacketSize = endpoint->wMaxPacketSize[hs];

        u16                     transferSize = 0;
        u8                      multiplier = 0;

        u16                     fifoSize = wMaxPacketSize;
        u16                     wIntrTxE;

        TRACE_MSG8(pcd->TAG, "[%2x] epnum: %d physicalEndpoint: %d bEndpointAddress: %02x "
                        "bmAttributes: %02x wMaxPacketSize: %d hs: %d next_fifo_offset: %x", 
                        ep, epnum, physicalEndpoint, bEndpointAddress, bmAttributes, 
                        wMaxPacketSize, hs, musb_belcarra->next_fifo_offset); 

        /* Configure Control - done early after reset */
        UNLESS (physicalEndpoint) {
                /* configure control endpoint */
                /* should set fifo registers here, but currently plat_uds.c: fifo_setup()
                 * is doing it and we can live with that ...
                 */
                musb_belcarra->next_fifo_offset = 0x8;
                return;
        }

        /* enable */
        wIntrTxE = musb_readw(pBase, MGC_O_HDRC_INTRTXE);
        wIntrTxE |= (1 << epnum);
        musb_writew(pBase, MGC_O_HDRC_INTRTXE, wIntrTxE);

        /* allocate dma */
        #ifdef USE_DMA_OUT
        endpoint->privdata = dma_channel = dma_controller->channel_alloc( dma_controller, 
                        epnum, bEndpointAddress & USB_DIR_IN, bmAttributes & 0x3, wMaxPacketSize);

//        endpoint->privdata = dma_channel = dma_controller->channel_alloc( dma_controller,
//                        &musb->endpoints[epnum], bEndpointAddress & USB_DIR_IN);
        #else /* USE_DMA_OUT */
        endpoint->privdata = dma_channel = NULL;
        #endif

        /* RxMaxp and TxMaxp 
         * 1. D10:0 Maximum payload - MUST match wMaxPacketSize
         * 2. D12:15 M -1 - M * D10:0 must be less than FIFO allocated or half FIFO if double buffered
         */
        switch(bEndpointAddress & 0x80) {
        case USB_DIR_IN:
	        if(is_msc_mode && !is_adb_mode)
		  transferSize = MIN(endpoint->transferSize[hs], MSC_TRANSFERSIZE);
		else
		  transferSize = MIN(endpoint->transferSize[hs], MAX_SPLIT_TRANSFERSIZE);
                multiplier = (transferSize + wMaxPacketSize - 1) / wMaxPacketSize;
                wMaxp = ((multiplier -1) << 11) | wMaxPacketSize;

                MGC_WriteCsr16(pBase, MGC_O_HDRC_TXMAXP, epnum, wMaxp);

                TRACE_MSG8(pcd->TAG, "[%2x] epn: %d  transferSize: %d multiplier: %d "
                                "multiplier-1: %d TXMAXP: %04x fifoSize: %x IN BULK_SPLIT %s", 
                                ep, epnum, transferSize, multiplier, multiplier -1, wMaxp, fifoSize,
                                ((multiplier>1) ? "ENABLED" : "DISABLED"));

                MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, epnum, MGC_M_TXCSR_MODE | MGC_M_TXCSR_CLRDATATOG | MGC_M_TXCSR_FLUSHFIFO);
                MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, epnum, MGC_M_TXCSR_MODE | MGC_M_TXCSR_CLRDATATOG | MGC_M_TXCSR_FLUSHFIFO);
                
                #ifdef USB_DOUBLE_IN
                c_size |= 0x40;
                fifoSize *= 2;
                TRACE_MSG3(pcd->TAG, "[%2x] Using DOUBLE IN buffering fifoSize: %04x %04x ", ep, fifoSize, fifoSize*2);
                #endif /* USB_DOUBLE_IN */
                break;

        case USB_DIR_OUT:
	        if(is_msc_mode && !is_adb_mode)
		  transferSize = MIN(endpoint->transferSize[hs], MSC_TRANSFERSIZE);
		else
		  transferSize = MIN(endpoint->transferSize[hs],MAX_COMBINE_TRANSFERSIZE);
                multiplier = (transferSize + wMaxPacketSize - 1) / wMaxPacketSize;
                wMaxp = ((multiplier -1) << 11) | wMaxPacketSize;

                TRACE_MSG8(pcd->TAG, "[%2x] epn: %d  transferSize: %d multiplier: %d multiplier-1: %d "
                                "RXMAXP: %04x fifoSize: %x OUT BULK_COMBINE %s", 
                                ep, epnum, transferSize, multiplier, multiplier-1, wMaxp, fifoSize,
                                ((multiplier>1) ? "ENABLED" : "DISABLED"));

                //MGC_WriteCsr16(pBase, MGC_O_HDRC_RXMAXP, epnum, wMaxp);
                MGC_WriteCsr16(pBase, MGC_O_HDRC_RXMAXP, epnum, 0);

                MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, epnum, MGC_M_RXCSR_FLUSHFIFO | MGC_M_RXCSR_CLRDATATOG);
                MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, epnum, MGC_M_RXCSR_FLUSHFIFO | MGC_M_RXCSR_CLRDATATOG);
                
                #ifdef USB_DOUBLE_OUT
                c_size |= 0x40;
                fifoSize *= 2;
                TRACE_MSG3(pcd->TAG, "[%2x] Using DOUBLE OUT buffering fifoSize: %04x %04x ", ep, fifoSize, fifoSize*2);
                #endif /* USB_DOUBLE_OUT */
                break;
        }

        /* configure single ep - these are done after set configuration */
        fifoSize *= multiplier;
        size = ffs(max(fifoSize, (u16) 8)) - 1;
        fifoSize >>= 3;
        c_size |= size - 3;

        if ((musb_belcarra->next_fifo_offset + fifoSize) > DYN_FIFO_SIZE) {
                printk(KERN_INFO"%s: ERROR\n", __FUNCTION__); 
                return;
        }

        /* program the new fifo size */
        musb_writeb(pBase, MGC_O_HDRC_INDEX, epnum);

        switch(bEndpointAddress & 0x80) {
        case USB_DIR_IN:
                musb_writeb(pBase, MGC_O_HDRC_TXFIFOSZ, c_size);
                musb_writew(pBase, MGC_O_HDRC_TXFIFOADD, musb_belcarra->next_fifo_offset );
                TRACE_MSG8(pcd->TAG, "[%2x] epnum: %d IN pr_offset: %x offset: %x "
                                "dma_channel: %p size: %x c_size: %04x fifoSize: %x",
                                ep, epnum, musb_belcarra->next_fifo_offset, musb_readw(pBase, MGC_O_HDRC_TXFIFOADD),
                                dma_channel, size, c_size, fifoSize);
                break;

        case USB_DIR_OUT:
                musb_writeb(pBase, MGC_O_HDRC_RXFIFOSZ, c_size);
                musb_writew(pBase, MGC_O_HDRC_RXFIFOADD, musb_belcarra->next_fifo_offset );
                switch (bmAttributes & 0x3){
                case 3: /* Interrupt */
                        wRxCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, epnum);
                        MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, epnum, wRxCsr | 0x1000);        //Setting DisNyet
                        break;
                }
                TRACE_MSG8(pcd->TAG, "[%2x] epnum: %d OUT pr_offset: %x offset: %x "
                                "dma_channel: %p size: %x c_size: %04x fifoSize: %x",
                                ep, epnum, musb_belcarra->next_fifo_offset, musb_readw(pBase, MGC_O_HDRC_RXFIFOADD),
                                dma_channel, size, c_size, fifoSize);
                break;
        }

        musb_belcarra->next_fifo_offset += fifoSize;
        musb_writeb(pBase, MGC_O_HDRC_INDEX, epnum);
        TRACE_MSG7(pcd->TAG, "[%2x] New values for this endpoint"
                        "epnum: %d rx_size: %x, rx_fifo: %x, tx_size: %x, tx_fifo: %x dma_channel: %p",
                        ep, epnum, musb_readb(pBase, MGC_O_HDRC_RXFIFOSZ),
                        musb_readw(pBase, MGC_O_HDRC_RXFIFOADD), musb_readb(pBase, MGC_O_HDRC_TXFIFOSZ),
                        musb_readw(pBase, MGC_O_HDRC_TXFIFOADD), dma_channel);
}

/*! omaphs_pcd_reset_ep - setup endpoint 
 * @param pcd -
 * @param ep -
 * @return none
 */
void omaphs_pcd_reset_ep (struct pcd_instance *pcd, unsigned int ep)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct otg_instance     *otg = pcd->otg;
        struct otg_dev          *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        struct musb             *musb = musb_belcarra->musb_privdata; 

        TRACE_MSG0(pcd->TAG, "--");
        UNLESS(ep) {
                musb_belcarra->next_fifo_offset = 0x8;
        }
}

/*! omaphs_pcd_set_address - set_address 
 * @param pcd -
 * @param address
 * @return none
 */
void omaphs_pcd_set_address (struct pcd_instance *pcd, unsigned char address)
{
        struct otg_instance *otg = pcd->otg;
        struct otg_dev *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        musb_belcarra->setAddress = TRUE;
        musb_belcarra->bAddress = address;
        pcd_bus_event_handler_irq (pcd->bus, DEVICE_ADDRESS_ASSIGNED, address);
}

/* ********************************************************************************************* */
/* ********************************************************************************************* */
/* omaphs_pcd_start - start the UDC
 * @param pcd - pcd instance
 */
void omaphs_pcd_start (struct pcd_instance *pcd)
{
        struct otg_instance *otg = pcd->otg;
        struct otg_dev *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        TRACE_MSG0(pcd->TAG, "UDC START");
        musb_belcarra_start(musb_belcarra);
}

/* omaphs_pcd_stop - stop the UDC
 * @param pcd - pcd instance
 */
void omaphs_pcd_stop(struct pcd_instance *pcd)
{
        struct otg_instance *otg = pcd->otg;
        struct otg_dev *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;
        struct musb             *musb = musb_belcarra->musb_privdata; 
        struct dma_controller   *dma_controller = musb->dma_controller;
        int i;
        musb_belcarra_stop(musb_belcarra);
}

#ifdef CONFIG_OTG_REMOTE_WAKEUP 
/*!
 * omaphs_remote_wakeup() - perform remote wakeup.
 * Initiate a remote wakeup to the host.
 * @param otg - otg instance
 * @param flag - SET or RESET
 */
static int
omaphs_remote_wakeup(struct pcd_instance *pcd)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct otg_instance     *otg = pcd ? pcd->otg : NULL;
        struct otg_dev          *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;

        void __iomem            *pBase = musb_belcarra->pRegs;
        u8                      power = musb_readb(pBase, MGC_O_HDRC_POWER);

        musb_belcarra->resumed = TRUE;

        musb_writeb(pBase, MGC_O_HDRC_POWER, power | MGC_M_POWER_RESUME);
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout( 1 );
        musb_writeb(pBase, MGC_O_HDRC_POWER, power);
        return 0;
}
#endif /* CONFIG_OTG_REMOTE_WAKEUP */

/*!
 * omaphs_vbus_status() - enable
 * This is called to return Vbus status
 * @param otg - pcd instance
 *
 */
static int
omaphs_vbus_status (struct pcd_instance *pcd)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct otg_instance     *otg = pcd ? pcd->otg : NULL;
        struct otg_dev          *otg_dev = otg->privdata;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;

        void __iomem            *pBase = musb_belcarra->pRegs;

        u32                     vbus = musb_readb(pBase, MGC_O_HDRC_DEVCTL) &&  0x18;

        switch (vbus) {
        case 1:
        case 2:
                return TRUE;
        default:
                break;
        }
        return FALSE;
}

/*! omaphs_softcon 
 * @param otg - otg_instance pointer
 * @param flag - 
 *
 * Enable or disable pullup.
 */
int omaphs_softcon(struct pcd_instance *pcd, int flag)
{
        struct usbd_bus_instance *bus = pcd->bus;
        struct otg_instance     *otg = pcd ? pcd->otg : NULL;
        struct otg_dev          *otg_dev = otg ? otg->privdata : NULL;
        struct musb_belcarra_instance *musb_belcarra = otg_dev ? otg_dev->privdata : NULL;

        void __iomem            *pBase = musb_belcarra->pRegs;

        u32                     power;
	//MOTO_ACCY_MASK_T accy_mask;

        RETURN_EINVAL_UNLESS(otg && otg->privdata);
        power = musb_readb(pBase, MGC_O_HDRC_POWER);

        //otg_dev = otg->privdata;
        //musb_belcarra = otg_dev ? otg_dev->privdata : NULL;
        //pBase = musb_belcarra ? musb_belcarra->pRegs : NULL;
        //RETURN_UNLESS(otg_dev && musb_belcarra);

        switch (flag) {
        case TRUE:
	/*moto_accy_get_all_devices(&accy_mask);
	if( accy_mask & (1 << MOTO_ACCY_TYPE_EMU_CABLE_USB)) {   // general cable
	    moto_accy_change_accy_mode (MOTO_ACCY_TYPE_EMU_CABLE_USB, MOTO_ACCY_MODE_USB_DPLUS_PU_ON);
	} else if(accy_mask & (1 << MOTO_ACCY_TYPE_EMU_CABLE_FACTORY)){   // factory cable
	    moto_accy_change_accy_mode (MOTO_ACCY_TYPE_EMU_CABLE_FACTORY, MOTO_ACCY_MODE_USB_DPLUS_PU_ON);
	}*/
	musb_writeb(pBase, MGC_O_HDRC_POWER, power | MGC_M_POWER_SOFTCONN);
	TRACE_MSG0(pcd->TAG, "OMAPHS PULLUP SET");
	break;
        case FALSE:
	/*moto_accy_get_all_devices(&accy_mask);
	if( accy_mask & (1 << MOTO_ACCY_TYPE_EMU_CABLE_USB)) {   // general cable
	    moto_accy_change_accy_mode (MOTO_ACCY_TYPE_EMU_CABLE_USB, MOTO_ACCY_MODE_USB_DPLUS_PU_OFF);
	} else if(accy_mask & (1 << MOTO_ACCY_TYPE_EMU_CABLE_FACTORY)){   // factory cable
	    moto_accy_change_accy_mode (MOTO_ACCY_TYPE_EMU_CABLE_FACTORY, MOTO_ACCY_MODE_USB_DPLUS_PU_OFF);
	}*/
	musb_writeb(pBase, MGC_O_HDRC_POWER, power & ~MGC_M_POWER_SOFTCONN);
	TRACE_MSG0(pcd->TAG, "OMAPHS PULLUP RESET");
	break;
        }
        return 0;
}


/*! omaphs_pcd_framenum() - get current framenum
 * @param otg
 */
u16
omaphs_pcd_framenum (struct pcd_instance *pcd)
{
        struct otg_instance     *otg = pcd ? pcd->otg : NULL;
        struct otg_dev          *otg_dev = otg ? otg->privdata : NULL;
        struct musb_belcarra_instance *musb_belcarra = otg_dev ? otg_dev->privdata : NULL;

        //printk(KERN_INFO"%s: pcd: %p otg: %p otg_dev: %p musb: %p pRegs: %p\n", __FUNCTION__,
        //                pcd, otg, otg_dev, musb_belcarra, musb_belcarra->pRegs); 

        RETURN_ZERO_UNLESS(musb_belcarra);
        RETURN_ZERO_UNLESS(musb_belcarra->pRegs);

        return (u16) musb_readw(musb_belcarra->pRegs, MGC_O_HDRC_FRAME);
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10)
/*! musb_pcd_ticks - get current ticks
 */
otg_tick_t musb_pcd_ticks (struct pcd_instance *pcd)
{
        unsigned long now = omap2_gptimer_read(GPTIMER3);
	return (otg_tick_t) now;
}
#else /* LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10) */
/* unsigned long do_getmachinecycles(void); */
/*! musb_pcd_ticks - get current ticks
 */
otg_tick_t musb_pcd_ticks (struct pcd_instance *pcd)
{
        unsigned long now = 0l;
        /* now = do_getmachinecycles(); */
	return (otg_tick_t) now;
}
#endif /* LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10) */

#define OMAP_TIMER_COUNTER_REG            0x28


/*! musb_pcd_elapsed - return micro-seconds between two tick values
 * @param t1
 * @param t2
 */
otg_tick_t musb_pcd_elapsed(otg_tick_t *t1, otg_tick_t *t2)
{
        otg_tick_t e = (((*t1 > *t2) ? (*t1 - *t2) : (*t2 - *t1)));
        do_div(e, 13);
        return (otg_tick_t) e * 2;
}


/* ********************************************************************************************* */

struct usbd_pcd_ops usbd_pcd_ops;
struct
usbd_pcd_ops usbd_pcd_ops = {
        .bmAttributes = 
                #ifdef CONFIG_OTG_SELF_POWERED
                USB_BMATTRIBUTE_SELF_POWERED |
                #endif /* CONFIG_OTG_SELF_POWERED */
                #ifdef CONFIG_OTG_REMOTE_WAKEUP
                USB_BMATTRIBUTE_REMOTE_WAKEUP |
                #endif /* CONFIG_OTG_REMOTE_WAKEUP */
                //USB_OTG_HNP_SUPPORTED | USB_OTG_SRP_SUPPORTED |
                USB_BMATTRIBUTE_RESERVED,
        .bMaxPower = CONFIG_OTG_BMAXPOWER,
        .max_endpoints =  0,
        .high_speed_capable = TRUE,
        .ep0_packetsize =  EP0_PACKETSIZE,
        //.capabilities = REMOTE_WAKEUP_SUPPORTED,
        .name =  UDC_NAME,
        .start = omaphs_pcd_start,
        .stop = omaphs_pcd_stop,
        .disable = omaphs_pcd_disable,
        .disable_ep = omaphs_pcd_disable_ep,
        .start_endpoint_in = omaphs_pcd_start_endpoint_in,
        .start_endpoint_out = omaphs_pcd_start_endpoint_out,
        .request_endpoints = pcd_request_endpoints,
        .set_endpoints = omaphs_pcd_set_endpoints,
        .set_address = omaphs_pcd_set_address,
        .setup_ep = omaphs_pcd_setup_ep,
        .reset_ep = omaphs_pcd_reset_ep,
        .halt_endpoint = omaphs_halt_endpoint,
        .device_feature = omaphs_device_feature,
#ifdef CONFIG_OTG_REMOTE_WAKEUP
        .remote_wakeup = omaphs_remote_wakeup,
#endif
        .endpoint_halted = omaphs_endpoint_halted,
        .framenum = omaphs_pcd_framenum,
        .ticks = musb_pcd_ticks,
        .elapsed = musb_pcd_elapsed,

        .vbus_status = omaphs_vbus_status,
        .softcon = omaphs_softcon,

        .cancel_in_irq = omaphs_pcd_cancel_in_irq,
        .cancel_out_irq = omaphs_pcd_cancel_out_irq,
};
/* ********************************************************************************************* */

/*! musb_transceiver_event
 * @param musb_belcarra
 * @param event
 */
int musb_transceiver_event(struct musb_belcarra_instance *musb_belcarra, musb_belcarra_transceiver_event_t event)
{
        struct otg_dev *otg_dev = musb_belcarra->usbd_privdata;
        struct pcd_instance *pcd = otg_dev ? otg_dev->pcd_instance : NULL;
        struct otg_instance *otg = otg_dev ? otg_dev->otg_instance : NULL;

        //printk(KERN_INFO"%s: otg_dev: %p pcd: %p otg: %p\n", __FUNCTION__, otg_dev, pcd, otg); 
        RETURN_ZERO_UNLESS(otg_dev && pcd && otg);

        switch (event) {
        case musb_vbus_valid:
                TRACE_MSG1(pcd->TAG, "VBUS VALID event: %d", event);
                otg_event(otg, VBUS_VLD | B_SESS_VLD, otg->pcd->TAG, "MUSB VBUS VALID");
                break;
        case musb_vbus_invalid:
                TRACE_MSG1(pcd->TAG, "VBUS INVALID event: %d", event);
                otg_event(otg, VBUS_VLD_ | B_SESS_VLD_ | A_SESS_VLD_, otg->pcd->TAG, "MUSB VBUS INVALID");
                break;
        }
        return 0;
}

/* omaphs_transceiver_callback
 * @param musb_belcarra
 * @param event
 */
int omaphs_transceiver_callback(struct musb_belcarra_instance *musb_belcarra, 
                void *privdata, musb_belcarra_transceiver_event_t event)
{
        struct otg_dev *otg_dev = privdata;
        struct otg_instance *otg = otg_dev->otg_instance;
        struct pcd_instance *pcd_instance = otg_dev->pcd_instance;
        
        //printk(KERN_INFO"%s:\n", __FUNCTION__); 
        switch (event) {
        case musb_vbus_valid:
                otg_event(otg, VBUS_VLD | B_SESS_VLD, otg->pcd->TAG, "OMAPHS TCD EN VBUS_VLD");
                break;
        case musb_vbus_invalid:
                otg_event(otg, VBUS_VLD_ | B_SESS_VLD_ | A_SESS_VLD_, otg->pcd->TAG, "OMAPHS TCD EN VBUS_VLD_");
                break;
        }
        return 0;
}

/* ********************************************************************************************* */
/*! default initialization
 */
int pcd_mod_init (struct otg_instance *otg);
void pcd_mod_exit (struct otg_instance *otg);

/* ********************************************************************************************* */
/*! omaphs_pcd_remove() - called to remove hardware
 * @param otg_dev - otg device 
 */
static void
omaphs_pcd_remove(struct otg_dev *otg_dev)
{
        struct otg_instance *otg = otg_dev->otg_instance;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;

        pcd_mod_exit(otg);
}

/*! omaphs_pcd_probe() - called to probe hardware
 * @param otg_dev - otg device 
 *
 * This function should do minimal, one time only hardware recognition,
 * resource reservation and minimal setup. Typically to get to known 
 * disabled state. It should not start the hardware.
 *
 */
static int 
omaphs_pcd_probe(struct otg_dev *otg_dev)
{
        struct otg_instance *otg_instance = otg_dev->otg_instance;
        struct musb_belcarra_instance *musb_belcarra = otg_dev->privdata;

        usbd_pcd_ops.max_endpoints = MUSB_C_NUM_EPS;
        pcd_mod_init(otg_instance);
        return 0;
}

/*! omaphs_pcd_suspend
 * @param otg_dev -
 * @param state -
 * @return none
 */
static void
omaphs_pcd_suspend(struct otg_dev *otg_dev, u32 state)
{
}

/*! omaphs_pcd_resume
 * @param otg_dev -
 * @return none
 */
static void
omaphs_pcd_resume(struct otg_dev *otg_dev)
{
}

/* ********************************************************************************************* */
static struct otg_dev_driver omaphs_pcd_driver = {
        .name =         "omaphs-pcd",
        .id =           OTG_DRIVER_PCD,
        .probe =        omaphs_pcd_probe,
        .remove =       omaphs_pcd_remove,
        //.suspend =      omaphs_pcd_suspend,
        //.resume =       omaphs_pcd_resume,
        .ops =          &pcd_ops,
};
/*! omaphs_pcd_module_init() - module init
 */
int omaphs_pcd_module_init (struct otg_device_driver *otg_device_driver)
{
        return otg_dev_register_driver(otg_device_driver, &omaphs_pcd_driver);
}

/*! omaphs_pcd_module_exit() - module exit
 */
void omaphs_pcd_module_exit (struct otg_device_driver *otg_device_driver)
{
        otg_dev_unregister_driver (otg_device_driver, &omaphs_pcd_driver);
}

EXPORT_SYMBOL(is_msc_mode);
EXPORT_SYMBOL(is_adb_mode);
