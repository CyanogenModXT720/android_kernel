/*
 * otg/linux-transceiver.h
 * @(#) sl@belcarra.com/whiskey.enposte.net|otg/otg/linux-transceiver.h|20071003202849|34683
 *
 *	Copyright (c) 2007-2008 Belcarra Technologies 2005 Corp
 *
 * By:
 *      Stuart Lynne <sl@belcarra.com>,
 *
 */

/*!
 * @addgroup OTGTCD Transceiver Controller Driver Support
 * @ingroup OTGOCD
 */
/*!
 * @file otg/otg/linux-transceiver.h
 * @brief Defines common to On-The-Go Transceiver Controller Support
 *
 * This file defines the tcd_ops and tcd_instance structures.
 *
 * The tcd_ops structure contains all of the output functions that will
 * be called as required by the OTG event handler when changing states.
 *
 * The tcd_instance structure is used to maintain the global data
 * required by the transceiver controller drivers.
 *
 * @ingroup OTGAPI
 * @ingroup TCD
 */

/*!
 * @name TCD Transceiver Controller Driver
 * @{
 */

typedef void linux_otg_transceiver_t;

linux_otg_transceiver_t * linux_transceiver_dev_probe(struct device *dev, 
                int (*vbus)(linux_otg_transceiver_t *, int), void *privdata);

void linux_transceiver_dev_remove(linux_otg_transceiver_t *);

int linux_transceiver_vbus(linux_otg_transceiver_t *);
int linux_transceiver_softcon(linux_otg_transceiver_t *, int);


