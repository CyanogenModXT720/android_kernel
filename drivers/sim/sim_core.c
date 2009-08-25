/*
 * Copyright (C) 2005-2009 Motorola, Inc.
 */

/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 */

/*
 * This file includes all of the initialization and tear-down code for the SIM
 * low-level driver and the basic user-mode interface (open(), close(), ioctl(), etc.).
 */

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <mach/dma.h>

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqflags.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/pm.h>

#include <linux/regulator/consumer.h>

#include <mach/clock.h>
#include <mach/hardware.h>
#include <mach/prcm.h>
#include <mach/resource.h>
#include <mach/omap-pm.h>

#include "smart_card.h"
#include "smart_card_kernel.h"

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/* OMAP Clock Framework */
static struct clk *usim_fck;
static struct clk *usim_ick;
static struct clk *omap_96m_fck;
static struct clk *omap_120m_fck;

/******************************************************************************
* Constants
******************************************************************************/
/*
  H/W BASE VIRTUAL ADDRESSES
  
  IMPORTANT NOTE(S) :
  
  1) each S/W platform may have their own memory map be careful
     to use the proper virtual address, as the virtual memory may not be contiguous
     like the physical memory is. 
*/
#define CM_BASE                 (L4_34XX_BASE + 0x4000)
#define CM_AUTOIDLE_WK IO_ADDRESS(CM_BASE + 0x0C30)
#define CM_CLKSEL_WK IO_ADDRESS(CM_BASE + 0x0C40)
#define PBIAS_CONTROL_LITE IO_ADDRESS(0x48002520)
#define DMA_SYSCONFIG IO_ADDRESS(0x4805602C)

/*
  OFFSET TO DMA REGISTER
  This is required to keep DMA from going to sleep on us as the lat APIs don't seem to work
*/

#define DMA_MIDLE 0x3000

/*
  OFFSETS TO PBIAS LITE REGISTERS

  IMPORTANT NOTE(S) :

  1) using direct register writes for PBIAS settings is not the prefferred method to do this. an API
     function should be provided to handle this, but i cannot find one as of yet. 
*/
#define PBIASVMODE1        0x0100
#define PBIASPWRDNZ1       0x0200
#define PBIASSPEEDCNTL1    0x0400
#define PBIASVMODEERROR1   0x0800

/*
  SIM SOURCE CLOCK SELECTION

  IMPORTANT NOTE(S) :
  None
*/
#define SIM_CLK_SEL_MASK   0x0078
#define SIM_CLK_120_DIV16  0x0048
#define SIM_CLK_96_DIV10   0x0030
#define SIM_CLK_SYSCLOCK   0x0008

#define SIM_ICLK_AUTOIDLE  0x0200

/*
  SIM MODULE CONSTANTS

  IMPORTANT NOTE(S) :
  None

*/
#define NO_OF_SIM_REGS     (sizeof(SIM_MODULE_REGISTER_BANK) / sizeof(UINT32))
#define SIM_REMOVED 0
#define SIM_PRESENT 1
#define SIM_MODULE_FREQUENCY_4  3750000
#define SIM_MODULE_FREQUENCY_5 4800000
#define SIM_MODULE_NACK_THRESHOLD 4
#define SIM_MODULE_RX_DMA_REQ     79

#define CMD_PARAMETER_SIZE_1 (sizeof(UINT32))
#define CMD_PARAMETER_SIZE_2 (2 * sizeof(UINT32))
#define CMD_PARAMETER_SIZE_3 (3 * sizeof(UINT32))
#define CMD_PARAMETER_SIZE_4 (4 * sizeof(UINT32))
#define CMD_PARAMETER_SIZE_5 (5 * sizeof(UINT32))

/*
  SIM MODULE OMAP CLOCK FRAMEWORK CONSTANTS

  IMPORTANT NOTE(S) :
  None

*/
#define USIM_FCK "usim_fck"
#define USIM_ICK "usim_ick"
#define OMAP_96M_FCK "omap_96m_fck"
#define OMAP_120M_FCK "dpll5_m2_ck"

/******************************************************************************
 * Local Macros
 *****************************************************************************/
/*
  SIM MODULE TRACEMSG MACRO

  IMPORTANT NOTE(S) :

  Do not enable logging by default. Only use this for debug purposes.
*/
#if 0
#define tracemsg(fmt,args...)  printk(fmt,##args)
#else
#define tracemsg(fmt,args...)
#endif



/*
  SIM MODULE REGISTER ACCESS MACROS

  IMPORTANT NOTE(S) :

  1) to be used only for SIM module register access
*/

#define read_reg(reg)                   (*(reg))
#define write_reg(reg, value)           (*(reg) = (value))
#define write_reg_bits(reg, mask, bits) (*(reg) = ( *(reg) & ~(mask)) | (bits));


/******************************************************************************
* Local type definitions
******************************************************************************/

typedef struct {
	UINT16 buffer_index;
	UINT16 rx_last_index;
	UINT16 tx_length;
	UINT8 error_flag;
	UINT8 *buffer;
	dma_addr_t dma_buffer;
} SIM_MODULE_BUFFER_DATA;

/******************************************************************************
* Local function prototypes
******************************************************************************/
static int sim_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg);
static int sim_open(struct inode *inode, struct file *file);
static int sim_free(struct inode *inode, struct file *file);
static unsigned int sim_poll(struct file *file, poll_table * wait);
static int sim_suspend(struct platform_device *pdev, pm_message_t state);
static int sim_resume(struct platform_device *pdev);
static irqreturn_t sim_module_int_irq_1(int irq, void *dev_id);

static void sim_module_int_reset_detect(UINT8 reader_id);
static void sim_module_int_rx(UINT8 reader_id);
static void sim_module_init_rx_mode(UINT8 reader_id);
static void sim_module_int_tx(UINT8 reader_id);
static void sim_module_set_voltage_level(SIM_MODULE_VOLTAGE_LEVEL level);
static void sim_module_set_clock_rate(UINT8 reader_id,
				      SIM_MODULE_CLOCK_RATE rate);

static int sim_probe(struct platform_device *pdev);
static int sim_remove(struct platform_device *pdev);

static void sim_module_dma_callback(INT32 lch, UINT16 ch_status,
				    void *data);

static int regulator_enabled_flag;
/******************************************************************************
* Local Structures
******************************************************************************/

/* Platform driver structure for the SIM driver */
static struct platform_driver sim_driver = {
	.driver = {
		   .name = SIM_DEV_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = sim_probe,
	.remove = sim_remove,
	.suspend = sim_suspend,
	.resume = sim_resume,
};

/* Platform device structure for the SIM driver */
static struct platform_device sim_device = {
	.name = SIM_DEV_NAME,
	.id = 1,
};

/*This structure defines the file operations for the SIM device */
static struct file_operations sim_fops = {
	.owner = THIS_MODULE,
	.ioctl = sim_ioctl,
	.open = sim_open,
	.release = sim_free,
	.poll = sim_poll,
};


/******************************************************************************
* Local variables
******************************************************************************/
static int sim_module_major;
static int sim_module_dma_channel = 0;

static spinlock_t sim_module_lock = SPIN_LOCK_UNLOCKED;

static UINT32 sim_module_clock_frequency = SIM_MODULE_FREQUENCY_4;

static wait_queue_head_t sim_module_wait =
__WAIT_QUEUE_HEAD_INITIALIZER(sim_module_wait);

static struct class *sim_class;

static UINT32 sim_module_rx_event = SIM_MODULE_EVENT_NONE;

static BOOLEAN sim_module_all_tx_data_sent = FALSE;

static UINT8 sim_module_current_pd_state_sim1 = SIM_REMOVED;
static UINT8 sim_module_nack_counter = 0;
static UINT8 sim_module_opens = 0;

static BOOLEAN sim_low_power_enabled = FALSE;

static SIM_MODULE_INTERRUPT_MODE sim_module_interrupt_mode =
    SIM_MODULE_RX_MODE;
static SIM_MODULE_BUFFER_DATA sim_module_card_data[NUM_SIM_MODULES];

static volatile SIM_MODULE_REGISTER_BANK *sim_registers[NUM_SIM_MODULES] = {
	((volatile SIM_MODULE_REGISTER_BANK *) IO_ADDRESS(SIM1_BASE_ADDR))
};

static volatile SIM_MODULE_REGISTER_BANK
    *sim_phys_registers[NUM_SIM_MODULES] = {
	(volatile SIM_MODULE_REGISTER_BANK *) SIM1_BASE_ADDR
};

struct regulator *vsim_regulator;
struct regulator *vsimcard_regulator;

/******************************************************************************
* Local Functions
******************************************************************************/

/* DESCRIPTION:
       The IOCTL handler for the SIM device
 
   INPUTS:
       inode       inode pointer
       file        file pointer
       cmd         the ioctl() command
       arg         the ioctl() argument

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.   
*/

static int sim_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	int status = 0;

	UINT32 args_kernel[CMD_PARAMETER_SIZE_5] = { 0, 0, 0, 0, 0 };
	volatile UINT32 *sim_module_register_ptr = NULL;

	unsigned long flags;

	UINT8 tx_index;
	UINT8 length;

	int error;

	tracemsg("SIM KERNEL IOCTL -> %X \n", cmd);
	/*
	   These IO control requests must only be used by the protocol layer in user space.
	   Any deviation from this rule WILL cause problems.
	 */

	switch (cmd) {
	case SIM_IOCTL_CONFIGURE_GPIO:

		/* enable the interface clock */
		clk_enable(usim_ick);

		/* enable auto idle mode for the interface clock */
		write_reg_bits((volatile UINT32 *) CM_AUTOIDLE_WK,
			       SIM_ICLK_AUTOIDLE, SIM_ICLK_AUTOIDLE);

		/* enable the functional clock */
		clk_enable(usim_fck);

		/* sleep some time waiting for clocks to stabalize */
		msleep(1);

		/* now that we've got the interface clock configured, reset the SIM module */
		write_reg_bits(&(sim_registers[(UINT8) arg]->sysconfig),
			       SOFTRESET_MASK, SOFTRESET_MASK);

		tracemsg("SIM Module: soft reset SIM module\n\n");

		/* block while the module is in reset ... */
		while ((read_reg(&(sim_registers[(UINT8) arg]->sysstatus))
			& RESETDONE_MASK) == 0) {
			/* yield the processor one time per iteration */
			msleep(1);
			tracemsg
			    ("SIM Module: soft reset not complete yet\n\n");
		}

		/* set the internal SIM clock divider to divide by 2 to obtain
		   a SIM clock of 3.75 MHz */
		write_reg_bits(&(sim_registers[(UINT8) arg]->usimconf2),
			       CONFSCLKDIV_MASK, 0);

		sim_module_set_clock_rate((UINT8) arg, SIM_MODULE_CLOCK_4);

		/* turn the clock back off for now */
		clk_disable(usim_fck);

		break;
	case SIM_IOCTL_READ_DATA_BUFFER:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_4);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg
			    ("buffer_index(read data buffer): %X offset: %X rx_length: %X\n",
			     sim_module_card_data[args_kernel[0]].
			     buffer_index, args_kernel[2], args_kernel[3]);

			if (copy_to_user
			    ((UINT8 *) args_kernel[1],
			     (&
			      (sim_module_card_data[args_kernel[0]].
			       buffer[args_kernel[2]])), args_kernel[3])) {
				tracemsg
				    ("Warning: failed to copy data to user-space while retrieving SIM card reader data for module 2\n");
				status = -EFAULT;
			}
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_UPDATE_DATA_BUFFER:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_3);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			status =
			    copy_from_user(sim_module_card_data
					   [args_kernel[0]].buffer,
					   (UINT8 *) args_kernel[1],
					   args_kernel[2]);
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_INTERRUPT_INIT:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_3);

		if ((UINT8) args_kernel[0] == SIM_MODULE_1) {
			free_irq(INT_SIM_GENERAL, NULL);
			status =
			    request_irq(INT_SIM_GENERAL,
					sim_module_int_irq_1,
					(IRQF_DISABLED | IRQF_NOBALANCING),
					SIM_DEV_NAME, 0);

			if (status != 0) {
				tracemsg
				    ("Warning: the SIM driver failed to request an IRQ. IRQ -> %X Error -> %X\n",
				     INT_SIM_GENERAL, status);
			}
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_INTERRUPT_MODE:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if ((UINT8) args_kernel[0] == SIM_MODULE_1) {
			tracemsg("Set SIM interrupt mode -> arg1: %X\n",
				 args_kernel[1]);

			switch (args_kernel[1]) {
			case SIM_MODULE_RX_MODE:
				tracemsg("SIM Module: RX mode \n\n");
				sim_module_interrupt_mode =
				    SIM_MODULE_RX_MODE;

				if (sim_module_dma_channel != 0) {
					omap_free_dma
					    (sim_module_dma_channel);
					sim_module_dma_channel = 0;
				}
				/* request a DMA logical channel */
				error =
				    omap_request_dma(OMAP34XX_DMA_USIM_RX,
						     SIM_DEV_NAME,
						     sim_module_dma_callback,
						     (void *)
						     sim_module_card_data
						     [args_kernel[0]].
						     dma_buffer,
						     &sim_module_dma_channel);
				if (error == 0) {
					/* configure the DMA parameters */
					omap_set_dma_transfer_params
					    (sim_module_dma_channel,
					     OMAP_DMA_DATA_TYPE_S8, 1,
					     SIM_MODULE_MAX_DATA,
					     OMAP_DMA_SYNC_ELEMENT,
					     OMAP34XX_DMA_USIM_RX,
					     OMAP_DMA_SRC_SYNC);
					omap_set_dma_src_params
					    (sim_module_dma_channel, 0,
					     OMAP_DMA_AMODE_CONSTANT,
					     (unsigned
					      long) (&(sim_phys_registers
						       [args_kernel[0]]->
						       usim_drx)), 0, 0);
					omap_set_dma_dest_params
					    (sim_module_dma_channel, 0,
					     OMAP_DMA_AMODE_POST_INC,
					     (unsigned long)
					     sim_module_card_data
					     [args_kernel[0]].dma_buffer,
					     0, 0);



					omap_start_dma
					    (sim_module_dma_channel);
				}

				if (error != 0) {
					tracemsg
					    ("SIM MODULE DMA CONFIGURATION ERROR -> %X \n",
					     error);
				}

				break;
			case SIM_MODULE_TX_MODE:
				tracemsg("SIM Module: TX mode \n\n");
				sim_module_interrupt_mode =
				    SIM_MODULE_TX_MODE;
				break;
			case SIM_MODULE_RESET_DETECT_MODE:
				tracemsg("SIM Module: reset detect \n\n");

				sim_module_card_data[args_kernel[0]].
				    buffer_index = 0;

				if (sim_module_dma_channel != 0) {
					omap_free_dma
					    (sim_module_dma_channel);
					sim_module_dma_channel = 0;
				}

				sim_module_interrupt_mode =
				    SIM_MODULE_RESET_DETECT_MODE;
				break;
			default:
				tracemsg
				    ("Warning: Invalid SIM interrupt mode. \n");
				break;
			}
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_WRITE_SIM_REG_DATA:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_3);

		tracemsg
		    ("Write SIM module register data -> reader ID: %X register: %X data: %X\n",
		     args_kernel[0], args_kernel[1], args_kernel[2]);

		if (args_kernel[1] <= sizeof(SIM_MODULE_REGISTER_BANK)) {
			if (args_kernel[0] < NUM_SIM_MODULES) {
				sim_module_register_ptr =
				    (UINT32 *) ((UINT32)
						sim_registers[args_kernel
							      [0]] +
						(UINT32) args_kernel[1]);
			} else {
				tracemsg
				    ("Warning: Invalid reader ID in SIM driver request.\n");
				status = -EFAULT;
			}

			if (sim_module_register_ptr != NULL) {
				write_reg(sim_module_register_ptr,
					  args_kernel[2]);

				/* if writing to the transmit FIFO increment the buffer index */
				if (args_kernel[1] == 0x0000003C) {
					sim_module_card_data[args_kernel
							     [0]].
					    buffer_index++;
				}
			}
		} else {
			tracemsg
			    ("Warning: Invalid register address in the SIM driver.\n");
			status = -EFAULT;
		}

		sim_module_register_ptr = NULL;
		break;
	case SIM_IOCTL_READ_SIM_REG_DATA:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_3);
		tracemsg
		    ("Read SIM register data -> Reader ID: %X, register: %X \n",
		     args_kernel[0], args_kernel[1]);
		if (args_kernel[1] <= sizeof(SIM_MODULE_REGISTER_BANK)) {
			if (args_kernel[0] < NUM_SIM_MODULES) {
				sim_module_register_ptr =
				    (UINT32 *) ((UINT32)
						sim_registers[args_kernel
							      [0]] +
						(UINT32) args_kernel[1]);
			} else {
				tracemsg
				    ("Warning: Invalid reader ID in SIM driver request.\n");
				status = -EFAULT;
			}

			if (sim_module_register_ptr != NULL) {
				args_kernel[2] =
				    read_reg(sim_module_register_ptr);

				if (copy_to_user
				    ((UINT32 *) arg, args_kernel,
				     CMD_PARAMETER_SIZE_3)) {
					tracemsg
					    ("Warning: failed to copy data to user-space while reading SIM module register data.\n");
					status = -EFAULT;
				}
			}
		} else {
			tracemsg
			    ("Warning: Invalid register address in the SIM driver.\n");
			status = -EFAULT;
		}

		sim_module_register_ptr = NULL;
		break;
	case SIM_IOCTL_WRITE_SIM_REG_BIT:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_4);

		tracemsg
		    ("Write SIM register bits -> reader ID: %X register: %X bits: %X mask: %X\n",
		     args_kernel[0], args_kernel[2], args_kernel[3],
		     args_kernel[1]);

		if (args_kernel[2] <= sizeof(SIM_MODULE_REGISTER_BANK)) {
			if (args_kernel[0] < NUM_SIM_MODULES) {
				sim_module_register_ptr =
				    (UINT32 *) ((UINT32)
						sim_registers[args_kernel
							      [0]] +
						(UINT32) args_kernel[2]);
			} else {
				tracemsg
				    ("Warning: Invalid reader ID in SIM driver request.\n");
				status = -EFAULT;
			}

			if (sim_module_register_ptr != NULL) {
				write_reg_bits(sim_module_register_ptr,
					       args_kernel[1],
					       args_kernel[3]);

				/* if writing to the transmit FIFO increment the buffer index */
				if (args_kernel[2] == 0x0000003C) {
					sim_module_card_data[args_kernel
							     [0]].
					    buffer_index++;
				}
			}
		} else {
			tracemsg
			    ("Warning: Invalid register address in the SIM driver.\n");
			status = -EFAULT;
		}

		sim_module_register_ptr = NULL;
		break;
	case SIM_IOCTL_SET_SIM_CLOCK_RATE:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		tracemsg
		    ("Set SIM clock rate, reader id -> %X, rate -> %X\n",
		     args_kernel[0], args_kernel[1]);

		sim_module_set_clock_rate(args_kernel[0], args_kernel[1]);
		break;
	case SIM_IOCTL_READ_RX_EVENT:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			args_kernel[1] = sim_module_rx_event;
			if (copy_to_user
			    ((UINT32 *) arg, args_kernel,
			     CMD_PARAMETER_SIZE_2)) {
				tracemsg
				    ("Warning: failed to copy data to user-space while reading SIM RX event type.\n");
				status = -EFAULT;
			}

			sim_module_rx_event = SIM_MODULE_EVENT_NONE;
		}

		else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}

		break;

	case SIM_IOCTL_SET_VOLTAGE_LEVEL:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			sim_module_set_voltage_level(args_kernel[1]);
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_READ_BUFFER_INDEX:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM reader buffer index\n");
			if (sim_module_dma_channel != 0) {
				/* if a DMA transfer already started ... */
				if (omap_get_dma_dst_pos
				    (sim_module_dma_channel) != 0) {
					/* set the number of characters received via DMA */
					args_kernel[1] =
					    (UINT32) (omap_get_dma_dst_pos
						      (sim_module_dma_channel)
						      -
						      sim_module_card_data
						      [args_kernel[0]].
						      dma_buffer);
				}

				/* else, DMA has not yet started ... */
				else {
					/* set the number of characters received via DMA to 0 */
					args_kernel[1] = 0;
				}
			}

			else {
				args_kernel[1] =
				    sim_module_card_data[args_kernel[0]].
				    buffer_index;
			}

			if (copy_to_user
			    ((UINT32 *) arg, args_kernel,
			     CMD_PARAMETER_SIZE_2)) {
				tracemsg
				    ("Warning: failed to copy data to user-space during buffer index retrieval\n");
				status = -EFAULT;
			}
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}

		break;
	case SIM_IOCTL_RESET_CARD_READER_DATA:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_5);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			sim_module_card_data[args_kernel[0]].buffer_index =
			    args_kernel[1];
			sim_module_card_data[args_kernel[0]].
			    rx_last_index = args_kernel[2];
			sim_module_card_data[args_kernel[0]].tx_length =
			    args_kernel[3];
			sim_module_card_data[args_kernel[0]].error_flag =
			    args_kernel[4];

			tracemsg
			    ("reset card reader data -> buffer_index: %X rx_last_index: %X tx_length: %X error_flag: %X\n",
			     args_kernel[1], args_kernel[2],
			     args_kernel[3], args_kernel[4]);
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}

		break;
	case SIM_IOCTL_UPDATE_RX_LAST_INDEX:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM update RX last index, index -> %X\n",
				 args_kernel[1]);
			sim_module_card_data[args_kernel[0]].
			    rx_last_index = args_kernel[1];
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_UPDATE_BUFFER_INDEX:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM update buffer index, index -> %X\n",
				 args_kernel[1]);
			sim_module_card_data[args_kernel[0]].buffer_index =
			    args_kernel[1];
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_ALL_TX_DATA_SENT:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM read all TX data sent\n");
			args_kernel[1] = sim_module_all_tx_data_sent;

			if (copy_to_user
			    ((UINT32 *) arg, args_kernel,
			     CMD_PARAMETER_SIZE_2)) {
				tracemsg
				    ("Warning: failed to copy data to user-space during card reader data retrieval\n");
				status = -EFAULT;
			}
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_RESET_ALL_TX_DATA_SENT:
		if (arg < NUM_SIM_MODULES) {
			tracemsg("SIM reset all TX data sent\n");
			sim_module_all_tx_data_sent = FALSE;
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;

	case SIM_IOCTL_DATA_TX:

		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM data TX\n");
			length = (UINT8) args_kernel[1];

			sim_module_card_data[args_kernel[0]].buffer_index =
			    0;

			/* Enable DMA mode */
			write_reg_bits(&
				       (sim_registers[args_kernel[0]]->
					usim_fifos), SIM_DMA_MODE_MASK,
				       SIM_DMA_MODE_MASK);

			if (sim_module_dma_channel != 0) {
				omap_free_dma(sim_module_dma_channel);
				sim_module_dma_channel = 0;
			}

			/* request a DMA logical channel */
			error =
			    omap_request_dma(OMAP34XX_DMA_USIM_RX,
					     SIM_DEV_NAME,
					     sim_module_dma_callback,
					     (void *)
					     sim_module_card_data
					     [args_kernel[0]].dma_buffer,
					     &sim_module_dma_channel);
			if (error == 0) {

				/* configure the DMA parameters */
				omap_set_dma_transfer_params
				    (sim_module_dma_channel,
				     OMAP_DMA_DATA_TYPE_S8, 1,
				     SIM_MODULE_MAX_DATA,
				     OMAP_DMA_SYNC_ELEMENT,
				     OMAP34XX_DMA_USIM_RX,
				     OMAP_DMA_SRC_SYNC);
				omap_set_dma_src_params
				    (sim_module_dma_channel, 0,
				     OMAP_DMA_AMODE_CONSTANT,
				     (unsigned
				      long) (&(sim_phys_registers
					       [args_kernel[0]]->
					       usim_drx)), 0, 0);
				omap_set_dma_dest_params
				    (sim_module_dma_channel, 0,
				     OMAP_DMA_AMODE_POST_INC,
				     (unsigned long)
				     sim_module_card_data[args_kernel[0]].
				     dma_buffer, 0, 0);

				omap_start_dma(sim_module_dma_channel);
			}

			local_irq_save(flags);

			/* for each character to transmit ... */
			for (tx_index = 0; tx_index < length; tx_index++) {
				/* copy the byte to the TX FIFO */
				write_reg(&
					  (sim_registers[args_kernel[0]]->
					   usim_dtx),
					  sim_module_card_data[args_kernel
							       [0]].
					  buffer[sim_module_card_data
						 [args_kernel[0]].
						 buffer_index++]);
			}

			/* setup the TX threshold value to write more data into the FIFO */
			write_reg_bits(&
				       (sim_registers[args_kernel[0]]->
					usim_fifos), FIFO_TX_TRIGGER_MASK,
				       (SIM_MODULE_TX_FIFO_SIZE - 1) << 2);

			/* clear the transmit complete interrupt */
			write_reg_bits(&
				       (sim_registers[args_kernel[0]]->
					irqstatus), USIM_TX_MASK,
				       USIM_TX_MASK);

			/* enable the transmit complete interrupt */
			write_reg_bits(&
				       (sim_registers[args_kernel[0]]->
					irqenable), USIM_TX_EN_MASK,
				       USIM_TX_EN_MASK);

			/* enable the transmitter */
			write_reg_bits(&
				       (sim_registers[args_kernel[0]]->
					usimconf2), TXNRX_MASK,
				       TXNRX_MASK);

			/* if there is just one block of data to send ... */
			if (sim_module_card_data[args_kernel[0]].
			    tx_length <= SIM_MODULE_TX_FIFO_SIZE) {
				sim_module_card_data[args_kernel[0]].
				    buffer_index = 0;

				/* enable the receiver right away */
				write_reg_bits(&
					       (sim_registers
						[args_kernel[0]]->
						usimconf2), TXNRX_MASK, 0);

				/* indicate this is the last block */
				sim_module_all_tx_data_sent = TRUE;

				/* initialize rx mode */
				sim_module_init_rx_mode(args_kernel[0]);

			}

			local_irq_restore(flags);
		}

		break;

	case SIM_IOCTL_GET_SIM_CLOCK_FREQ:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM get SIM clock frequency\n");
			args_kernel[1] = sim_module_clock_frequency;
			if (copy_to_user
			    ((UINT32 *) arg, args_kernel,
			     CMD_PARAMETER_SIZE_2)) {
				tracemsg
				    ("Warning: failed to copy data to user-space for get frequency\n");
				status = -EFAULT;
			}
		}

		else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}

		break;

	case SIM_IOCTL_LOW_POWER_STATE:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   (sizeof(UINT32) * 2));

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM set constraint, state -> %X\n",
				 args_kernel[1]);
			/*  if we are active ...  */
			if ((BOOL) args_kernel[1] == FALSE) {
                                /* Stop DMA from AKC'ing idle requests */
			        write_reg_bits((volatile UINT32 *)DMA_SYSCONFIG,DMA_MIDLE, 
                                    DMA_SYSCONFIG_MIDLEMODE(1));

				/* Request the latency constraint */
				omap_pm_set_max_mpu_wakeup_lat(&sim_device.
							       dev, 10);

				/* enable the SIM FCLK */
				clk_enable(usim_fck);

				/* wait for the clock to settle */
				msleep(1);
			}

			/* else, we are inactive ... */
			else {
                                /* Allow DMA to ACK idle requests */
			        write_reg_bits((volatile UINT32 *)DMA_SYSCONFIG,DMA_MIDLE, 
                                    DMA_SYSCONFIG_MIDLEMODE(2));

				/* Disable DMA mode for low power mode */
				write_reg_bits(&
					       (sim_registers
						[args_kernel[0]]->
						usim_fifos),
					       SIM_DMA_MODE_MASK, 0);

				/* disable the SIM FCLK */
				clk_disable(usim_fck);

				/* Release the latency constraint */
				omap_pm_set_max_mpu_wakeup_lat(&sim_device.
							       dev, -1);
			}

			sim_low_power_enabled = (BOOL) args_kernel[1];
		}
		break;

	case SIM_IOCTL_SLEEP:
		msleep(arg);
		break;

	default:
		tracemsg
		    ("Warning: Invalid request sent to the SIM driver.\n");
		status = -ENOTTY;
		break;
	}

	return status;
}

/* DESCRIPTION:
       The open() handler for the SIM device.
 
   INPUTS:
       inode       inode pointer
       file        file pointer

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.   
*/
static int sim_open(struct inode *inode, struct file *file)
{
	int status = 0;

	spin_lock(&sim_module_lock);

	/* only allow 1 open */
	if (sim_module_opens > 0) {
		spin_unlock(&sim_module_lock);

		status = -ENODEV;
	}

	if (status == 0) {
		sim_module_opens++;
		spin_unlock(&sim_module_lock);

		/* allocate non-bufferable, or cacheable memory for DMA */
		sim_module_card_data[SIM_MODULE_1].buffer =
		    (UINT8 *) dma_alloc_coherent(NULL, SIM_MODULE_MAX_DATA,
						 &(sim_module_card_data
						   [SIM_MODULE_1].
						   dma_buffer), 0);

		/* if there is not a valid buffer */
		if ((sim_module_card_data[SIM_MODULE_1].buffer == NULL) ||
		    (sim_module_card_data[SIM_MODULE_1].dma_buffer == 0)) {
			tracemsg
			    ("The data buffer was not allocated buffer -> %X, dma_buffer -> %X\n",
			     (unsigned int)
			     sim_module_card_data[SIM_MODULE_1].buffer,
			     (unsigned int)
			     sim_module_card_data[SIM_MODULE_1].
			     dma_buffer);
			status = -EFAULT;
		}
	}

	if (status == 0) {
		/* initialize the dma channel to 0 */
		sim_module_dma_channel = 0;

	}

	tracemsg("sim : sim_open, status -> %X\n", status);

	return status;
}

/* DESCRIPTION:
       The close() handler for the SIM device
 
   INPUTS:
       inode       inode pointer
       file        file pointer

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.   
*/
static int sim_free(struct inode *inode, struct file *file)
{
	tracemsg("sim : sim_free()\n");
	spin_lock(&sim_module_lock);
	sim_module_opens--;
	spin_unlock(&sim_module_lock);
	return 0;
}

/* DESCRIPTION:
       The poll() handler for the SIM driver
 
   INPUTS:
       file        file pointer
       wait        poll table for this poll()

   OUTPUTS:
       Returns 0 if no data to read or POLLIN if data available.

   IMPORTANT NOTES:
       None.   
*/
static unsigned int sim_poll(struct file *file, poll_table * wait)
{
	unsigned int retval = 0;

	if (sim_module_rx_event == SIM_MODULE_EVENT_NONE) {
		/* Add our wait queue to the poll table */
		poll_wait(file, &sim_module_wait, wait);
	}

	if (sim_module_rx_event != SIM_MODULE_EVENT_NONE) {
		retval = POLLIN;
	}

	return retval;
}

/*

DESCRIPTION:
    This function is the interrupt handler SIM receive data.

INPUTS:
    UINT8 reader_id: the card reader ID of the interrupt to handle

OUTPUT:
    None

IMPORTANT NOTES:
    None

*/
static void sim_module_int_rx(UINT8 reader_id)
{
	/* notify user space that data was received */
	sim_module_rx_event |= SIM_MODULE_EVENT_RX_A;

	write_reg(&(sim_registers[reader_id]->irqstatus), USIM_RX_MASK);

	return;
}

/*

DESCRIPTION:
    This function handles the SIM TX interrupt.  In cases where the TX data exceeds the TX FIFO
    length, it refills the TX FIFO on TX threshold interrupts.  On TX complete, it switches the
    system into receive mode.

INPUTS:
    UINT8 reader_id: the card reader ID of the interrupt to handle

OUTPUT:
    None

IMPORTANT NOTES:
    None

*/
static void sim_module_int_tx(UINT8 reader_id)
{
	UINT16 bytes_remaining;
	UINT8 tx_index;
	BOOL last_block;

	last_block = FALSE;

	/* disable the transmit complete interrupt */
	write_reg_bits(&(sim_registers[reader_id]->irqenable),
		       USIM_TX_EN_MASK, 0);

	/* disable the transmitter */
	write_reg_bits(&(sim_registers[reader_id]->usimconf2), TXNRX_MASK,
		       0);

	/* determine remaining number of bytes to transmit */
	bytes_remaining = sim_module_card_data[reader_id].tx_length -
	    sim_module_card_data[reader_id].buffer_index;

	/* if the number of bytes remaining exceeds the FIFO size ... */
	if (bytes_remaining > SIM_MODULE_TX_FIFO_SIZE) {
		/* only transmit the free TX FIFO size */
		bytes_remaining = SIM_MODULE_TX_FIFO_SIZE;
	} else {
		last_block = TRUE;
	}

	/* for each byte of data to transmit ... */
	for (tx_index = 0; tx_index < bytes_remaining; tx_index++) {
		/* write the character to the TX FIFO */
		write_reg(&(sim_registers[reader_id]->usim_dtx),
			  sim_module_card_data[reader_id].
			  buffer[sim_module_card_data[reader_id].
				 buffer_index++]);

	}

	/* setup the TX threshold value to write more data into the FIFO */
	write_reg_bits(&(sim_registers[reader_id]->usim_fifos),
		       FIFO_TX_TRIGGER_MASK,
		       (SIM_MODULE_TX_FIFO_SIZE - 1) << 2);

	/* clear the transmit complete interrupt */
	write_reg_bits(&(sim_registers[reader_id]->irqstatus),
		       USIM_TX_MASK, USIM_TX_MASK);

	/* enable the transmit complete interrupt */
	write_reg_bits(&(sim_registers[reader_id]->irqenable),
		       USIM_TX_EN_MASK, USIM_TX_EN_MASK);

	/* enable the transmitter */
	write_reg_bits(&(sim_registers[reader_id]->usimconf2), TXNRX_MASK,
		       TXNRX_MASK);

	/* if there is just one block of data to send ... */
	if (last_block == TRUE) {
		/* reset the data buffer index */
		sim_module_card_data[reader_id].buffer_index = 0;

		/* enable the receiver right away */
		write_reg_bits(&(sim_registers[reader_id]->usimconf2),
			       TXNRX_MASK, 0);

		/* indicate this is the last block */
		sim_module_all_tx_data_sent = TRUE;

		/* initialize rx mode */
		sim_module_init_rx_mode(reader_id);

	}

	return;
}

/*

DESCRIPTION:
    This function is the interrupt handler for the SIM Reset Detect Mode.  This function only reads
    data and copies it into the receive buffer without checking for parity errors.  The purpose is
    to advance the buffer_index variable for the detection of an incoming ATR message which
    indicates that the SIM card has reset.  An ATR message is the only thing which should be
    received (and then only on SIM card resets) when this interrupt handler is active.

INPUTS:
    UINT8 reader_id: the card reader ID of the interrupt to handle

OUTPUT:
    None

IMPORTANT NOTES:
    None

*/
static void sim_module_int_reset_detect(UINT8 reader_id)
{
	/* mask the RX FIFO full interrupt */
	write_reg_bits(&(sim_registers[reader_id]->irqenable),
		       USIM_RX_EN_MASK, 0);

	/* while there is data in the FIFO ... */
	while (((sim_registers[reader_id]->
		 usim_fifos) & FIFORX_EMPTY_MASK) == 0) {
		/* copy the character into the data buffer */
		sim_module_card_data[reader_id].
		    buffer[sim_module_card_data[reader_id].buffer_index++]
		    = (read_reg(&(sim_registers[reader_id]->usim_drx)) &
		       USIMDRX_MASK);
	}

	/* set an event for user space */
	sim_module_rx_event |= SIM_MODULE_EVENT_BUFFER_INDEX;

	/* clear the interrupt */
	write_reg(&(sim_registers[reader_id]->irqstatus), USIM_RX_MASK);

	return;
}


/*

DESCRIPTION:
    This function is the interrupt service routine that handles non-data interrupts of smart card
    interface module 1.

INPUTS:
    int irq       : the interrupt request number
    void * dev_id : pointer to the device associated with the interrupt

OUTPUT:
    irqreturn_t status : a status indicating if an interrupt was handled successfully

IMPORTANT NOTES:
    None

*/
static irqreturn_t sim_module_int_irq_1(int irq, void *dev_id)
{
	if (sim_low_power_enabled == TRUE) {
		/* enable the SIM FCLK */
		clk_enable(usim_fck);
	}

	/* if the byte resent interrupt fired ... */
	if (((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	      USIM_RESENT_MASK) == USIM_RESENT_MASK)
	    &&
	    ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	      USIM_RESENT_EN_MASK) == USIM_RESENT_EN_MASK)) {
		sim_module_nack_counter++;

		/* if the number of nacks on a character exceed the threshold ... */
		if (sim_module_nack_counter > SIM_MODULE_NACK_THRESHOLD) {
			/* we must have a bad ME->SIM connection.
			   user space must be notified of this fatal error */
			sim_module_rx_event |= SIM_MODULE_EVENT_ERROR_FLAG;
			sim_module_nack_counter = 0;
		}

		write_reg(&(sim_registers[SIM_MODULE_1]->irqstatus),
			  USIM_RESENT_MASK);
	}

	/* if the position detect interrupt fired ... */
	if (((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	      USIM_CD_MASK) == USIM_CD_MASK)
	    &&
	    ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	      USIM_CD_EN_MASK) == USIM_CD_EN_MASK)) {
		/* if card status is changing from removed -> present ... */
		if (((read_reg(&(sim_registers[SIM_MODULE_1]->usimstat)) &
		      STATNOCARD_MASK) == STATNOCARD_MASK)
		    && (sim_module_current_pd_state_sim1 == SIM_REMOVED)) {
			sim_module_rx_event |=
			    SIM_MODULE_EVENT_SIMPD_INSERTION;
			sim_module_current_pd_state_sim1 = SIM_PRESENT;
		}

		/* else if card status is changing from present -> removed ... */

		else if (((read_reg
			   (&(sim_registers[SIM_MODULE_1]->usimstat)) &
			   STATNOCARD_MASK) == 0)
			 && (sim_module_current_pd_state_sim1 ==
			     SIM_PRESENT)) {
			sim_module_rx_event |=
			    SIM_MODULE_EVENT_SIMPD_REMOVAL;
			sim_module_current_pd_state_sim1 = SIM_REMOVED;
		}

		/* else the CD interrupt fired, but the status didn't change, so ignore */

		write_reg(&(sim_registers[SIM_MODULE_1]->irqstatus),
			  USIM_CD_MASK);
	}

	/* if the character timer interrupt fired ... */
	if (((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	      USIM_WT_MASK) == USIM_WT_MASK)
	    &&
	    ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	      USIM_WT_EN_MASK) == USIM_WT_EN_MASK)) {
		/* notify user space */
		sim_module_rx_event |= SIM_MODULE_EVENT_WWT_VIOLATION;
		write_reg_bits(&(sim_registers[SIM_MODULE_1]->irqenable),
			       USIM_WT_EN_MASK, 0);
	}

	/* if the no ATR interrupt fired ... */
	if (((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	      USIM_NATR_MASK) == USIM_NATR_MASK)
	    &&
	    ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	      USIM_NATR_EN_MASK) == USIM_NATR_EN_MASK)) {
		write_reg_bits(&(sim_registers[SIM_MODULE_1]->irqenable),
			       USIM_NATR_EN_MASK, 0);
		/* notify user space */
		sim_module_rx_event |= SIM_MODULE_EVENT_NO_ATR_FLAG;
	}

	/* if one of the data interrupts fired ... */
	if ((((sim_module_interrupt_mode == SIM_MODULE_RX_MODE)
	      || (sim_module_interrupt_mode ==
		  SIM_MODULE_RESET_DETECT_MODE))
	     &&
	     ((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	       USIM_RX_MASK) == USIM_RX_MASK)
	     &&
	     ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	       USIM_RX_EN_MASK) == USIM_RX_EN_MASK))
	    || ((sim_module_interrupt_mode == SIM_MODULE_TX_MODE)
		&&
		((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
		  USIM_TX_MASK) == USIM_TX_MASK)
		&&
		((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
		  USIM_TX_EN_MASK) == USIM_TX_EN_MASK))) {
		/* execute the mode specific data interrupt handler */
		if (sim_module_interrupt_mode == SIM_MODULE_RX_MODE) {
			sim_module_int_rx(SIM_MODULE_1);
		} else if (sim_module_interrupt_mode == SIM_MODULE_TX_MODE) {
			sim_module_int_tx(SIM_MODULE_1);
		} else if (sim_module_interrupt_mode ==
			   SIM_MODULE_RESET_DETECT_MODE)
		{
			sim_module_int_reset_detect(SIM_MODULE_1);
		}
	}

	/* if there was a FIFO overflow ... */
	if (((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	      USIM_RXFULL_MASK) == USIM_RXFULL_MASK)
	    &&
	    ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	      USIM_RXFULL_EN_MASK) == USIM_RXFULL_EN_MASK)) {
		/* there is no reason to read the data in the FIFO, as we've already missed data
		   at this point. this is a fatal error. the OMAP3430 does not have the ability
		   to automatically NACK characters when the FIFO is full, therefore characters will
		   be lost. this can never be allowed to happen because of the lack of flow control
		   on this IC. */

		/* clear and disable the interrupt */
		write_reg(&(sim_registers[SIM_MODULE_1]->irqstatus),
			  USIM_RXFULL_MASK);
		write_reg_bits(&(sim_registers[SIM_MODULE_1]->irqenable),
			       USIM_RXFULL_EN_MASK, 0);

		/* notify user space as this could be a fatal error */
		sim_module_rx_event |= SIM_MODULE_EVENT_FIFO_OVERFLOW;
	}

	/* if this interrupt caused a user space event ... */
	if (sim_module_rx_event != SIM_MODULE_EVENT_NONE) {
		/* wake up the user space event thread */
		wake_up_interruptible(&sim_module_wait);
	}

	if (sim_low_power_enabled == TRUE) {
		/* disable the SIM FCLK */
		clk_disable(usim_fck);
	}

	return (IRQ_RETVAL(1));
}

/*

DESCRIPTION:
    This routine sets up the hardware and interrupt system to receive data.

INPUTS:
    UINT8 card_reader_id: card reader ID for which to setup

OUTPUT:
    None

IMPORTANT NOTES:
    None
*/
static void sim_module_init_rx_mode(UINT8 reader_id)
{
	/* if the reader ID is valid ... */
	if (reader_id < NUM_SIM_MODULES) {
		/* disable TX mode */
		write_reg_bits(&(sim_registers[reader_id]->irqenable),
			       USIM_TX_EN_MASK, 0);

		/* clear the RX fifo threshold */
		write_reg_bits(&(sim_registers[reader_id]->usim_fifos),
			       FIFO_RX_TRIGGER_MASK, 0);

		/* enable DMA mode */
		write_reg_bits(&(sim_registers[reader_id]->usim_fifos),
			       SIM_DMA_MODE_MASK, SIM_DMA_MODE_MASK);

		/* indicate the card is in the non-ATR receive mode */
		sim_module_interrupt_mode = SIM_MODULE_RX_MODE;

		/* if there is no data in the FIFO ... */
		if ((read_reg(&(sim_registers[reader_id]->usim_fifos)) &
		     FIFORX_EMPTY_MASK) == FIFORX_EMPTY_MASK) {
			/* clear the RX fifo interrupt */
			write_reg(&(sim_registers[reader_id]->irqstatus),
				  (USIM_RX_MASK | USIM_RXFULL_MASK));
		}

		/* enable the RX full interrupt */
		write_reg_bits(&(sim_registers[reader_id]->irqenable),
			       (USIM_RX_MASK | USIM_RXFULL_EN_MASK),
			       (USIM_RX_MASK | USIM_RXFULL_EN_MASK));

	}

	return;
}

/*

DESCRIPTION:
    This routine configures the voltage level

INPUTS:
    SIM_MODULE_VOLTAGE_LEVEL level : the voltage level to use

OUTPUT:
    None

IMPORTANT NOTES:
    None
*/
static void sim_module_set_voltage_level(SIM_MODULE_VOLTAGE_LEVEL level)
{
	/* power down the voltage regulator */
	if (regulator_enabled_flag) {
		regulator_disable(vsim_regulator);
		regulator_disable(vsimcard_regulator);
		regulator_enabled_flag = 0;
	}

	/* power down the pads */
	write_reg_bits((volatile UINT32 *) PBIAS_CONTROL_LITE,
		       PBIASPWRDNZ1, 0);

	/* enable fast I/O */
	write_reg_bits((volatile UINT32 *) PBIAS_CONTROL_LITE,
		       PBIASSPEEDCNTL1, PBIASSPEEDCNTL1);

	/* 3.0V is desired */
	if (level == SIM_MODULE_VOLTAGE_HIGH) {
		/* power on the voltagage regulator at 3 volts */
		regulator_set_voltage(vsim_regulator, 2900000, 2900000);
		regulator_set_voltage(vsimcard_regulator, 2900000,
				      2900000);

		if (!(regulator_enabled_flag)) {
			regulator_enable(vsim_regulator);
			regulator_enable(vsimcard_regulator);
			regulator_enabled_flag = 1;
		}

		/* configure the pad for 3.0V operation */
		write_reg_bits((volatile UINT32 *) PBIAS_CONTROL_LITE,
			       PBIASVMODE1, PBIASVMODE1);
	}

	/* 1.8V is desired */
	else if (level == SIM_MODULE_VOLTAGE_LOW) {
		regulator_set_voltage(vsim_regulator, 1800000, 1800000);
		regulator_set_voltage(vsimcard_regulator, 1800000,
				      1800000);
		if (!(regulator_enabled_flag)) {
			regulator_enable(vsim_regulator);
			regulator_enable(vsimcard_regulator);
			regulator_enabled_flag = 1;
		}

		/* configure the pad for 1.8V operation */
		write_reg_bits((volatile UINT32 *) PBIAS_CONTROL_LITE,
			       PBIASVMODE1, 0);
	}

	/* else, leave the voltage supply */
	if ((level == SIM_MODULE_VOLTAGE_HIGH)
	    || (level == SIM_MODULE_VOLTAGE_LOW)) {
		/* the pad bias level doesn't equal the VSIM level ... */
		while ((read_reg((volatile UINT32 *) PBIAS_CONTROL_LITE) &
			PBIASVMODEERROR1) != 0) {
			/* block the current thread, but yeild the processor
			   every iteration of the loop */
			msleep(1);
		}

		/* enable the pad */
		write_reg_bits((volatile UINT32 *) PBIAS_CONTROL_LITE,
			       PBIASPWRDNZ1, PBIASPWRDNZ1);

		/* let voltages stabalize */
		msleep(1);
	}

	return;

}

/*

DESCRIPTION:
    This routine sets the clock rate

INPUTS:
    UINT8 reader_id                : the reader ID for which to set the clock rate
    SIM_MODULE_CLOCK_RATE level : the clock rate to set

OUTPUT:
    None

IMPORTANT NOTES:
    None
*/
static void sim_module_set_clock_rate(UINT8 reader_id,
				      SIM_MODULE_CLOCK_RATE rate)
{
	/* if the reader ID is valid ... */
	if (reader_id < NUM_SIM_MODULES) {
		switch (rate) {
		case SIM_MODULE_CLOCK_0:

			/* disable the SIM FCLK */
			clk_disable(usim_fck);
			break;
		case SIM_MODULE_CLOCK_4:
			/* set the internal SIM clock divider to divide by 2 */
			write_reg_bits(&
				       (sim_registers[reader_id]->
					usimconf2), CONFSCLKDIV_MASK, 0);
			/* disable the SIM FCLK */
			clk_disable(usim_fck);
			/* set the source of the SIM FCLK to the 120 MHz clock divided
			   by 16 to obtain a 3.75 MHz SIM clock */
			clk_set_parent(usim_fck, omap_120m_fck);
			clk_set_rate(usim_fck, 7500000);

			/* enable the SIM FCLK */
			clk_enable(usim_fck);
			sim_module_clock_frequency =
			    SIM_MODULE_FREQUENCY_4;
			break;
		case SIM_MODULE_CLOCK_5:
			/* set the internal SIM clock divider to divide by 2 */
			write_reg_bits(&
				       (sim_registers[reader_id]->
					usimconf2), CONFSCLKDIV_MASK, 0);
			/* disable the SIM FCLK */
			clk_disable(usim_fck);
			/* set the source of the SIM FCLK to the 96 MHz clock divided
			   by 10 to obtain a 4.8 MHz SIM clock */
			clk_set_parent(usim_fck, omap_96m_fck);
			clk_set_rate(usim_fck, 9600000);

			/* enable the SIM FCLK */
			clk_enable(usim_fck);
			sim_module_clock_frequency =
			    SIM_MODULE_FREQUENCY_5;
			break;
		default:
			tracemsg
			    ("Warning: Invalid SIM clock selection.\n");
			break;
		}
	}

	if ((rate == SIM_MODULE_CLOCK_4) || (rate == SIM_MODULE_CLOCK_5)) {
		/* wait for the clock to stabalize */
		msleep(1);
	}

	return;
}

/*

DESCRIPTION:
    This routine is the callback function for the SIM DMA request

INPUTS:
    INT32 lch : the logical channel which caused the callback to execute
    UINT16 ch_status : the status of the logical channel
    void * data : a pointer to the DMA data.

OUTPUT:
    None

IMPORTANT NOTES:
    Given the way we're using DMA due to H/W design and T=0 protocol limitations
    , this shouldn't ever be called unless there is a problem. Either there was an error,
    or we ran out of room in the RAM buffer because of too many errors.
*/
void sim_module_dma_callback(INT32 lch, UINT16 ch_status, void *data)
{

	if (ch_status != 0) {
		tracemsg
		    ("SIM MODULE DMA CALLBACK ERROR lch -> %X, ch_status -> %X\nregister contents: ",
		     lch, ch_status);

		tracemsg(" **************************"
			 " DMA Channel (%d) Registers"
			 "***************************\n",
			 sim_module_dma_channel);
#if(0)
/* TODO: These don't exist on the new kernel, submit a CR to check out where they went */
		tracemsg("OMAP_DMA_CCR_REG(%d)           : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CCR_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CLNK_CTRL_REG(%d)     : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CLNK_CTRL_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CICR_REG(%d)          : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CICR_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CSR_REG(%d)           : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CSR_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CSDP_REG(%d)          : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CSDP_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CEN_REG(%d)           : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CEN_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CFN_REG(%d)           : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CFN_REG(sim_module_dma_channel));
		tracemsg("OMAP2_DMA_CSSA_REG(%d)         : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP2_DMA_CSSA_REG(sim_module_dma_channel));
		tracemsg("OMAP2_DMA_CDSA_REG(%d)         : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP2_DMA_CDSA_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CSEI_REG(%d)          : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CSEI_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CSFI_REG(%d)          : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CSFI_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CDEI_REG(%d)          : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CDEI_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CDFI_REG(%d)          : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CDFI_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CSAC_REG(%d)          : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CSAC_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CDAC_REG(%d)          : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CDAC_REG(sim_module_dma_channel));
		tracemsg("OMAP2_DMA_CCEN_REG(%d)         : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP2_DMA_CCEN_REG(sim_module_dma_channel));
		tracemsg("OMAP2_DMA_CCFN_REG(%d)         : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP2_DMA_CCFN_REG(sim_module_dma_channel));
		tracemsg("OMAP2_DMA_COLOR_REG(%d)        : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP2_DMA_COLOR_REG(sim_module_dma_channel));
#endif

	}

	sim_module_rx_event |= SIM_MODULE_EVENT_ERROR_FLAG;

	/* wake up the user space event thread */
	wake_up_interruptible(&sim_module_wait);

	return;
}

/* DESCRIPTION:
       The probe routine.
 
   INPUTS:
       struct platform_device *pdev : the platform device pointer.  

   OUTPUTS:
       Returns probe sucess/error indication.

   IMPORTANT NOTES:
       None.
*/
static int sim_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *temp_class;

	sim_module_major = register_chrdev(0, SIM_DEV_NAME, &sim_fops);
	if (sim_module_major < 0) {
		tracemsg
		    ("sim_probe: Unable to get a major for SIM driver\n");
		return sim_module_major;
	}

	sim_class = class_create(THIS_MODULE, SIM_DEV_NAME);
	if (IS_ERR(sim_class)) {
		unregister_chrdev(sim_module_major, SIM_DEV_NAME);
		tracemsg("sim_probe: Error creating SIM class.\n");
		ret = PTR_ERR(sim_class);
		return ret;
	}

	temp_class =
	    device_create(sim_class, NULL, MKDEV(sim_module_major, 0),
			  NULL, SIM_DEV_NAME);

	if (IS_ERR(temp_class)) {
		class_destroy(sim_class);
		unregister_chrdev(sim_module_major, SIM_DEV_NAME);
		tracemsg("sim_probe: Error creating SIM class device.\n");
		ret = PTR_ERR(temp_class);
		return ret;
	}

	tracemsg("sim_probe: SIM Module successfully probed\n");
	return ret;
}

/* DESCRIPTION:
       The remove routine to disassociate the SIM driver with the device.
 
   INPUTS:
       struct platform_device *pdev : the platform device pointer.

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.   
*/
static int sim_remove(struct platform_device *pdev)
{
	device_destroy(sim_class, MKDEV(sim_module_major, 0));
	class_destroy(sim_class);
	unregister_chrdev(sim_module_major, SIM_DEV_NAME);
	tracemsg("sim_remove: Driver-device disassociation complete.\n");
	return 0;
}

/* DESCRIPTION:
       This routine suspends the device.

   INPUTS:
       struct platform_device *pdev : the platform device pointer
       pm_message_t state : the power mode that the system is going into

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.
*/
static int sim_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
	if (sim_low_power_enabled == FALSE)
		ret = -EBUSY;
	return ret;
}

/* DESCRIPTION:
       This routine resumes the device.

   INPUTS:
       struct platform_device *pdev : the platform device pointer

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.
*/
static int sim_resume(struct platform_device *pdev)
{
	return 0;
}

/* DESCRIPTION:
       The SIM intialization function.
 
   INPUTS:
       None.

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.   
*/
int __init sim_init(void)
{
	int ret = 0;

	tracemsg("sim_init: SIM driver loading...\n");

	vsim_regulator = regulator_get(NULL, "vsim");
	if (IS_ERR(vsim_regulator)) {
		printk("Could not get VSIM regulator\n");
		return PTR_ERR(vsim_regulator);
	}
	vsimcard_regulator = regulator_get(NULL, "vsimcard");
	if (IS_ERR(vsimcard_regulator)) {
		printk("Could not get VSIMCARD regulator\n");
		return PTR_ERR(vsimcard_regulator);
	}

	/* Register the Driver */
	ret = platform_driver_register(&sim_driver);
	if (ret != 0) {
		tracemsg("sim_init: Driver registration failed.\n");
	} else {
		tracemsg("sim_init: Driver regristration passed.\n");
		ret = platform_device_register(&sim_device);
		if (ret != 0) {
			platform_driver_unregister(&sim_driver);
			tracemsg
			    ("sim_init: Device registration failed.\n");
		} else {
			tracemsg
			    ("sim_init: Device registration passed.\n");
		}
	}

	usim_fck = clk_get(&sim_device.dev, USIM_FCK);
	if (IS_ERR(usim_fck)) {
		tracemsg("sim_init: Error getting USIM FCLK.\n");
		ret = PTR_ERR(usim_fck);
	}

	usim_ick = clk_get(&sim_device.dev, USIM_ICK);
	if (IS_ERR(usim_ick)) {
		tracemsg("sim_init: Error getting USIM ICLK.\n");
		ret = PTR_ERR(usim_ick);
	}

	omap_96m_fck = clk_get(&sim_device.dev, OMAP_96M_FCK);
	if (IS_ERR(omap_96m_fck)) {
		tracemsg("sim_init: Error getting 96M FCLK.\n");
		ret = PTR_ERR(omap_96m_fck);
	}

	omap_120m_fck = clk_get(&sim_device.dev, OMAP_120M_FCK);
	if (IS_ERR(omap_120m_fck)) {
		tracemsg("sim_init: Error getting 120M FCLK.\n");
		ret = PTR_ERR(omap_120m_fck);
	}
	return ret;
}

/* DESCRIPTION:
       The SIM device cleanup function
 
   INPUTS:
       None. 

   OUTPUTS:
       None.

   IMPORTANT NOTES:
       None.   
*/
static void __exit sim_exit(void)
{
	regulator_put(vsim_regulator);
	regulator_put(vsimcard_regulator);


	/* release the clock resources */
	clk_put(usim_fck);
	clk_put(usim_ick);
	clk_put(omap_96m_fck);
	clk_put(omap_120m_fck);

	/* unregister the device */
	platform_device_unregister(&sim_device);
	platform_driver_unregister(&sim_driver);
	tracemsg("sim_exit: SIM driver successfully unloaded.\n");
}


/*
 * Module entry points
 */
module_init(sim_init);
module_exit(sim_exit);

MODULE_DESCRIPTION("SIM driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
