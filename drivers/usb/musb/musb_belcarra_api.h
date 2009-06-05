/*
 * otg/hardware/musb_api.h
 * @(#) balden@belcarra.com|musb_belcarra_api.h|20070508195254|03989
 *
 *      Copyright (c) 2007 Belcarra Technologies 2005 Corp
 *
 * By:
 *      Stuart Lynne <sl@belcarra.com>
 *      Shahrad Payandeh <sp@belcarra.com>
 *
 *
 * These entry points will be exported from musb_belcarra.c when linked
 * into the musb module.
 *
 */

#define MUSB_EP0        0
#define MUSB_DIR_OUT    0
#define MUSB_DIR_IN     1
#define MUSB_ENDPOINT_MAX       16

#define DYN_FIFO_SIZE (1<<(MUSB_C_RAM_BITS+2))

#define MUSB_DMA_NONE           (0)
#define MUSB_DMA_SHORT          (1 << 0)
#define MUSB_DMA_NORMAL         (1 << 1)

/* peripheral side ep0 states */
typedef enum musb_ep0_state {
        MUSB_END0_STAGE_SETUP = 0,      /* idle, waiting for setup */
        MUSB_END0_STAGE_TX = 1,         /* IN data */
        MUSB_END0_STAGE_RX = 2,         /* OUT data */
        MUSB_END0_STAGE_STATUS_IN = 3,  /* (after OUT data) */
        MUSB_END0_STAGE_STATUS_OUT = 4, /* (after IN data) */
        MUSB_END0_STAGE_ACK_WAIT = 5,   /* after zlp, before statusin */
} musb_ep0_state_t ;

/* usb events */
typedef enum musb_belcarra_usb_event {
        musb_reset,
        musb_suspend,
        musb_resumed,
        musb_disconnect,
        musb_stop_activity,
} musb_belcarra_usb_event_t;

/* transceiver events */
typedef enum musb_belcarra_transceiver_event {
        musb_vbus_valid,
        musb_vbus_invalid,
} musb_belcarra_transceiver_event_t;

struct musb_ops;

struct musb_belcarra_instance {
        void                    *musb_privdata;
        void                    *usbd_privdata;
        struct musb_ops         *ops;

        void __iomem            *pRegs;

        musb_ep0_state_t        ep0_state;

        int                     setAddress:1;
        int                     speedSet:1;
        int                     testMode:1;
        int                     resumed:1;

        u8                      bAddress;
        u8                      testModeSelector;

        u16                     next_fifo_offset;
        struct usbd_endpoint_map *endpoint_map_array;
};


struct musb_ops {

        int                     (*usb_event)(struct musb_belcarra_instance *, musb_belcarra_usb_event_t);
        int                     (*transceiver_event)(struct musb_belcarra_instance *, musb_belcarra_transceiver_event_t);
        int                     (*ep0_irq) (struct musb_belcarra_instance *);
        int                     (*tx_irq) (struct musb_belcarra_instance *, int, int);
        int                     (*rx_irq) (struct musb_belcarra_instance *, int, int);
        int                     (*dma_irq) (struct musb_belcarra_instance *, u8, u32, u32, u16);
};

/*!
 * musb_belcarra_cleanup_driver
 */
void musb_belcarra_cleanup_driver(struct musb_belcarra_instance *musb_belcarra);

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
struct musb_belcarra_instance * musb_belcarra_register_driver(struct musb_ops *ops);


/*!
 * int musb_belcarra_start
 *
 * Start the UDC.
 */
int musb_belcarra_start(struct musb_belcarra_instance *musb_belcarra);


/*!
 * int musb_belcarra_stop
 *
 * Stop the UDC.
 */
int musb_belcarra_stop(struct musb_belcarra_instance *musb_belcarra);

