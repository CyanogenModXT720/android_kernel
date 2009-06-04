/*
 * otg/otghw/isp1301/isp1301.h -- USB Transceiver Controller driver 
 * @(#) balden@seth2.belcarratech.com|otg/otghw/isp1301.h|20051116205002|02468
 *
 *      Copyright (c) 2004-2005 Belcarra
 *
 * By: 
 *      Stuart Lynne <sl@lbelcarra.com>, 
 *      Bruce Balden <balden@belcarra.com>
 *
 * Copyright 2005-2006 Motorola, Inc.
 *
 * Changelog:
 * Date               Author           Comment
 * -----------------------------------------------------------------------------
 * 12/12/2005         Motorola         Initial distribution
 * 10/18/2006         Motorola         Add Open Src Software language
 * 12/12/2006         Motorola         Changes for Open Src compliance.
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
/*!
 * @defgroup ISP1301TCD Philips ISP1301 
 * @ingroup OTGTCD
 */
/*!
 * @file otg/otghw/isp1301.h
 * @brief Private structures and defines for ISP1301 Transciever Controller Driver.
 *
 * This file contains the private defines and structures for the ISP1301 Transceiver 
 * Driver. 
 *
 * @ingroup ISP1301TCD
 */

/*!
 * @name ISP1301 ISP1301 configuration
 * @{
 */

#ifdef CONFIG_OMAP_H2
#define ISP1301_GPIO            (2)
#define TCD_ISP1301_I2C_ADDR    ISP1301_I2C_ADDR_HIGH;
#endif

#define ISP1301_NAME    "isp1301"

#define ISP1301_LOC_CONN 0x01

#define ISP1301_INT_SRC         0xeb

/*!
 * @enum otg_transceiver 
 * Define ISP1301 compatible transceivers
 */
typedef enum otg_transceiver {
        unknown_transceiver,    /*!< unknown */
        isp1301,                /*!< Philips ISP1301 */
        max3301e                /*!< Maxim MAX3301e */
} otg_transceiver_t;

/*!
 * @struct otg_transceiver_map 
 * How to determine transceiver model and manufacturer
 */
struct otg_transceiver_map {
        otg_transceiver_t transceiver_type;     /*!< transceiver type */
        u16 vendor;                             /*!< vendor number */
        u16 product;                            /*!< product number */
        u16 revision;                           /*!< revision number */
        char *name;                             /*!< name */
};

/*! 
 * @struct isp1301_private
 * Information required for operation
 */
struct isp1301_private {
        WORK_STRUCT bh;                         /*!< work structure for bottom half handler */
        u16 vendor;                             /*!< vendor number found */
        u16 product;                            /*!< product number found */
        u16 revision;                           /*!< revision number found */
        u32 flags;                              /*!< flags */
        u8  int_src;                            /*!< current interrupt source */
        struct otg_transceiver_map *transceiver_map; /*! pointer to transceiver map for this transceiver */
	BOOL ready;				/*!< it is set when the structure is ready and filled */
};

/*! 
 * @enum tx_mode
 * This defines the various ways that the ISP1301 can be
 * wired into the host USB. 
 *
 *
 * VP_VM unidirectional mode
 *
 * VP_VM bidirectional mode
 *
 *
 * DAT_SE0 unidirectional mode
 *
 * DAT_SE0 bidirectional mode
 *
 *
 *
 *
 * C.f. Figure 23 OTG controller with DAT_SE0 SIE interface
 * 3-Wire - DAT_SE0 Bi-Directional - Single ended (Dat, SE0)
 *                      __
 *  TXEN         ----   OE              --->    OE (9)
 *  
 *  RXD/TXD     <----   TXDAT/RCVDAT    --->    DAT (14 DAT/VP)
 *  
 *  SE0         <----   TXSE0/RCVSE0    --->    SE0 (13 SE0/VM)
 *                      ______
 *  GPIO        <----   OTGIRQ          ---     INT_N (5) Active low
 *
 *
 * C.f. Figure 24 OTG controller with VP_VM SIE interface
 * 4-Wire - VP_VM Bi-directional
 *
 *                      __
 *  TXEN         ----   OE              --->    OE (9)
 *
 *  VP          <----   TXVP/RCVVP      --->    VP (14 DAT/VP)
 *
 *  VM          <----   TXVM/RCVVM      --->    VM (10)
 *
 *  RCV         <----   RCV             ---     RCV (12)
 *                      ______
 *  GPIO        <----   OTGIRQ          ---     INT_N (5) Active low
 *
 *
 *
 *
 * 6-Wire VP_VM Uni-directional
 *                      __
 *  TXEN         ----   OE              --->    OE (9)
 *
 *  TXD          ----   TXDAT           --->    VP (14 DAT/VP)
 *
 *  SE0          ----   TXSE0           --->    VM (13 SE0/VM)
 *
 *  RCV         <----   RCV             ---     RCV (12)
 *
 *  RCVVP       <----   RCVVP           ---     VP (11)
 *
 *  RCVVM       <----   RCVVM           ---     VM (10)
 *                      ______
 *  GPIO        <----   OTGIRQ          ---     INT_N (5) Active low
 *
 *
 *
 */
typedef enum isp1301_tx_mode {
        dat_se0_bidirectional,          /*!< 3-Wire     Single-Ended (DAT, SEO)  / Single-Ended (DAT, SE0) */
        vp_vm_bidirectional,            /*!< 4-Wire     Differential (TxDp,TxDM) / Single-Ended (DAT, SE0)*/
        vp_vm_unidirectional,           /*!< 6-Wire     Differential (TxDp,TxDM) / Differential (RxDp, RxDM, RxD )*/
        dat_se0_unidirectional,         /*!< 6-Wire     Single-Ended (DAT, SE0)  / Differential (RxDp, RxDM, RxD ) */
} isp1301_tx_mode_t;

/*! 
 * @enum spd_ctrl
 * This defines how the speed and suspend pins are controlled
 * for this host.
 */
typedef enum isp1301_spd_ctrl {
        spd_susp_pins,                  /*!< controlled by SPEED and SUSPEND pins */
        spd_susp_reg,                   /*!< controled by SPEED_REG and SUSPEND_REG in Mode Control 1 register */
} isp1301_spd_ctrl_t;


extern struct tcd_ops tcd_ops;
extern struct isp1301_private isp1301_private;
extern struct tcd_instance *tcd_instance;

/* isp1301.c */
extern int isp1301_mod_init(WORK_PROC );
extern int isp1301_mod_init__(void );
extern void isp1301_mod_exit(void);
extern void isp1301_tcd_en(struct otg_instance *, u8 );
extern void isp1301_chrg_vbus(struct otg_instance *, u8 );
extern void isp1301_drv_vbus(struct otg_instance *, u8 );
extern void isp1301_dischrg_vbus(struct otg_instance *, u8 );
extern void isp1301_dp_pullup_func(struct otg_instance *, u8 );
extern void isp1301_dm_pullup_func(struct otg_instance *, u8 );
extern void isp1301_dp_pulldown_func(struct otg_instance *, u8 );
extern void isp1301_dm_pulldown_func(struct otg_instance *, u8 );
extern void isp1301_peripheral_host_func(struct otg_instance *, u8 );
extern void isp1301_dm_det_func(struct otg_instance *, u8 );
extern void isp1301_dp_det_func(struct otg_instance *, u8 );
extern void isp1301_cr_det_func(struct otg_instance *, u8 );
extern void isp1301_bdis_acon_func(struct otg_instance *, u8 );
extern void isp1301_mx21_vbus_drain_func(struct otg_instance *, u8 );
extern void isp1301_id_pulldown_func(struct otg_instance *, u8 );
extern void isp1301_audio_func(struct otg_instance *, u8 );
extern void isp1301_uart_func(struct otg_instance *, u8 );
extern void isp1301_mono_func(struct otg_instance *, u8 );

extern void isp1301_otg_timer(struct otg_instance *, u8 );
extern void isp1301_bh(void *);
extern int isp1301_id (struct otg_instance *);
extern int isp1301_vbus (struct otg_instance *);
extern void  isp1301_configure(isp1301_tx_mode_t, isp1301_spd_ctrl_t );

extern void isp1301_bh_wakeup(int);

/* thread */
extern int isp1301_thread_init (void (bh_proc)(void *));
extern void isp1301_thread_exit (wait_queue_head_t *);
extern void isp1301_thread_wakeup(int enabled, int first);



/* i2c-xxx.c */
int  i2c_configure(char *name, int addr);
extern void  i2c_close(void);
extern u8 i2c_readb(u8 );
extern u16 i2c_readw(u8 );
extern u32 i2c_readl(u8 );
extern void i2c_writeb(u8 , u8 );
extern void i2c_writeb_direct(u8 , u8 );
//extern void i2c_write(u8 , u8 );


/* ********************************************************************************************* */
#define TRACE_I2CB(t,r) TRACE_MSG3(t, "%-40s[%02x] %02x", #r, r, i2c_readb(r))
#define TRACE_I2CW(t,r) TRACE_MSG3(t, "%-40s[%02x] %04x", #r, r, i2c_readw(r))
#define TRACE_GPIO(t,b,r) TRACE_MSG2(t, "%-40s %04x", #r, readw(b + r))
#define TRACE_REGL(t,r) TRACE_MSG2(t, "%-40s %08x", #r, readl(r))

/* @} */

