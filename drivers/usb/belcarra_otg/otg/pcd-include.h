/*
 * otg/udc.h
 * @(#) sl@belcarra.com/whiskey.enposte.net|otg/otg/pcd-include.h|20061017072623|59852
 *
 *      Copyright (c) 2004-2005 Belcarra
 *	Copyright (c) 2005-2006 Belcarra Technologies 2005 Corp
 *
 * By:
 *      Stuart Lynne <sl@belcarra.com>,
 *      Bruce Balden <balden@belcarra.com>
 *
 */
/*!
 * @file otg/otg/pcd-include.h
 * @brief Linux OS Precompiled Headers
 *
 * @ingroup OTGCore
 */

#include <otg/otg-compat.h>
#include <otg/otg-module.h>

//#include <asm/irq.h>
//#include <asm/system.h>
//#include <asm/io.h>

#include <otg/usbp-chap9.h>
//#include <otg/usbp-func.h>
#include <otg/usbp-bus.h>
#include <otg/otg-trace.h>
#include <otg/otg-api.h>
//#include <otg/otg-task.h>
#include <otg/otg-tcd.h>
#include <otg/otg-hcd.h>
#include <otg/otg-pcd.h>
#include <otg/otg-ocd.h>
#include <otg/usbp-pcd.h>

#undef  USE_DMA_IN
#undef  USE_DMA_OUT
#undef  USE_BULK_SPLIT
#undef  USE_BULK_COMBINE
#undef  USE_DOUBLE_IN
#undef  USE_DOUBLE_OUT
                                                                                                    
#define USE_BULK_COMBINE        1
#define USE_BULK_SPLIT          1
                                                                                                    
#ifdef USE_BULK_COMBINE
#define MAX_COMBINE_TRANSFERSIZE        1024
#else /* USE_BULK_COMBINE */
#define MAX_COMBINE_TRANSFERSIZE        512
#endif /* USE_BULK_COMBINE */
                                                                                                    
#ifdef USE_BULK_SPLIT
#define MAX_SPLIT_TRANSFERSIZE        1024
#else /* USE_BULK_SPLIT */
#define MAX_SPLIT_TRANSFERSIZE        512
#endif /* USE_BULK_SPLIT */

/* FIFO size for mass storage mode */
#define MSC_TRANSFERSIZE         8192

#ifdef CONFIG_USB_INVENTRA_DMA
#define USE_DMA_IN              1
#define USE_DMA_OUT             1
//#warning INVENTRA DMA Enabled
#else
//#warning INVENTRA DMA NOT Enabled
#endif /* CONFIG_USB_INVENTRA_DMA */
