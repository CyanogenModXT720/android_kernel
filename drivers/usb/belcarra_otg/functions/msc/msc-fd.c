/*
 * otg/function/msc/msc.-fd.c
 * @(#) tt/root@belcarra.com/debian286.bbb|otg/functions/msc/msc-fd.c|20071020014806|30308
 *
 *      Copyright (c) 2003-2006 Belcarra Technologies Corp
 *	Copyright (c) 2005-2006 Belcarra Technologies 2005 Corp
 *
 * By:
 *      Stuart Lynne <sl@belcarra.com>,
 *      Bruce Balden <balden@belcarra.com>
 *      Tony Tang <tt@belcarra.com>
 *
 */


/*!
 * @file otg/functions/msc/msc-fd.c
 * @brief Mass Storage Driver private defines
 *
 * This is a Mass Storage Class Function that uses the Bulk Only protocol.
 *
 *
 * Notes:
 *
 * 1. Currently we only support the Bulk Only model. Microsoft states that
 * further support for the mass storage driver will only be done for devices
 * that conform to the Bulk Only model.
 *
 * 2. Multiple LUN's are not supported, but in theory they could be.
 *
 * 3. Error handling should be done with STALL but using ZLP seems to also
 * work. ZLP is usually easier to implement (except possibly on the SA1100.)
 * We may need to make STALL an option if we find devices (perhaps SA1100)
 * that cannot reliaby send a ZLP on BULK-IN endpoint.
 *
 *
 * 4. WinXP will match the following:
 *
 *      USB: Class_08&SubClass_02&Prot_50
 *      USB: Class_08&SubClass_05&Prot_50
 *      USB: Class_08&SubClass_06&Prot_50
 *
 * SubClass 02 is MMC or SFF8020I
 * SubClass 05 is SFF or SFF8070I
 * SubClass 06 is SCSI
 *
 * From the Windows USB Storage FAQ:
 *
 *      RBC not supported
 *
 *      SubClass = 6 (SCSI)
 *              CDBs SHOULD NOT be padded to 12 bytes
 *              ModeSense/ModeSelect SHOULD NOT be translated from 1ah/15h to 5ah/55h
 *              should be used for FLASH
 *
 *      SubClass !=6
 *              CDBs SHOULD be padded to 12 bytes
 *              ModeSense/ModeSelect SHOULD be translated from 1ah/15h to 5ah/55h
 *
 * We are using the former, SubClass = 6, and implement the required SCSI operations.
 *
 *
 * TODO
 *
 *
 * 06/05/2008         Motorola         Modify scsi inquiry and read_capacity cmd handler to 
 *                                     support linux based pc
 * 05/06/2009         Motorola         Dynamic FIFO support
 *
 * TODO FIXME Bus Interface Notes
 *
 * 1. The bus interface driver must correctly implement NAK'ing if not rcv_urb is
 * present (see au1x00.c or wmmx.c for examples.)
 *
 * 2. The bus interface driver must implement USBD_URB_SENDZLP flag (see au1x00.c
 * for example.)
 *
 * @ingroup MSCFunction
 */


#include <otg/otg-compat.h>
#include <otg/usbp-chap9.h>
#include <otg/usbp-func.h>
#include <otg/otg-trace.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/ctype.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>
#include <linux/random.h>
#include <linux/slab.h>

#if defined(LINUX26)
#include <linux/buffer_head.h>
#endif
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/switch.h>

#include "msc-scsi.h"
#include "msc.h"
#include "msc-fd.h"
#include "crc.h"


/* Module Parameters ************************************************************************* */


#define DEVICE_EJECTED          0x0001          // MEDIA_EJECTED
#define DEVICE_INSERTED         0x0002          // MEDIA_INSERT

#define DEVICE_OPEN             0x0004          // WR_PROTECT_OFF
#define DEVICE_WRITE_PROTECTED  0x0008          // WR_PROTECT_ON

#define DEVICE_CHANGE_ON        0x0010          // MEDIA_CHANGE_ON

#define DEVICE_PREVENT_REMOVAL  0x0020


#define DEF_NUMBER_OF_HEADS     0x10
#define DEF_SECTORS_PER_TRACK   0x20

extern int is_msc_mode;

/* MSC ******************************************************************************************** */

int msc_urb_sent (struct usbd_urb *tx_urb, int rc);
static int msc_recv_urb (struct usbd_urb *urb, int rc);

otg_tag_t msc_fd_trace_tag;
/* Sense Key *********************************************************************************** */

/*!
 * @brief set_sense_data - set sensedata in msc private struture
 *
 * This saves sense data. Sense data indicates what type of error
 * has occurred and will be returned to the host when a request sense
 * command is sent.

 * @param function_instance
 * @param sensedata
 * @param info
 * @return none
 */
void set_sense_data(struct usbd_function_instance *function_instance, u32 sensedata, u32 info)
{
        struct msc_private *msc = function_instance->privdata;
        TRACE_SENSE(sensedata, info);
        msc->sensedata = sensedata;
        msc->info = info;
}

/* Check blockdev ****************************************************************************** */

/*!
 * @brief msc_check_blockdev_name - check current status of the block device
 *
 * Check if the block device is operational, either generate a failed CSW
 * or a ZLP if not ready.
 *
 * Returns non-zero if the block device is not available for I/O operations
 * and a failed CSW has already been sent.
 * @param function_instance
 * @param zlp
 * @param name
 * @return int
 */
int msc_check_blockdev_name(struct usbd_function_instance *function_instance, int zlp, char *name)
{
        struct msc_private *msc = function_instance->privdata;
        if (msc->block_dev_state & DEVICE_EJECTED) {
                TRACE_MSG0(MSC,"CHECK BLOCKDEV DEVICE_EJECTED");
                ((SENDZLP == zlp) ? msc_dispatch_query_urb_zlp : msc_start_sending_csw_failed)
                        (function_instance, SCSI_SENSEKEY_MEDIA_NOT_PRESENT, msc->lba, USB_MSC_FAILED);
                return -EINVAL;
        }
         
        if (msc->block_dev_state & DEVICE_CHANGE_ON) {
                msc->block_dev_state &= ~DEVICE_CHANGE_ON;
                TRACE_MSG0(MSC,"CHECK BLOCKDEV DEVICE_CHANGE_ON");
                
                ((SENDZLP == zlp) ? msc_dispatch_query_urb_zlp : msc_start_sending_csw_failed)
                        (function_instance, SCSI_SENSEKEY_NOT_READY_TO_READY_CHANGE, msc->lba, USB_MSC_FAILED);
                return -EINVAL;
        }
        //TRACE_MSG0(MSC,"CHECK BLOCKDEV DEVICE_INSERTED");
        return 0;
}

/* Generic start recv urb and send csw ********************************************************* */

/*!
 * @brief msc_start_recv - queue a receive urb
 *
 * Ensure that size is a multiple of the endpoint packetsize.
 *
 * Returns non-zero if there is an error in the USB layer.
 * @param function_instance
 * @param size
 * @return int
 */
int msc_start_recv_urb(struct usbd_function_instance *function_instance, int size)
{
        struct usbd_urb *rcv_urb = NULL;
        struct msc_private *msc = function_instance->privdata;
        int wMaxPacketSize = usbd_endpoint_wMaxPacketSize(function_instance, BULK_OUT, usbd_high_speed(function_instance));

        


        TRACE_MSG0(MSC,"enter msc_start_recv_urb");
        if ((size % wMaxPacketSize))
                size = ((size + wMaxPacketSize) / wMaxPacketSize) * wMaxPacketSize;
        RETURN_EINVAL_UNLESS((rcv_urb = usbd_alloc_urb (function_instance, BULK_OUT, size, msc_recv_urb)));
        rcv_urb->function_privdata = function_instance;
        msc->rcv_urb_finished = NULL;
        RETURN_ZERO_UNLESS(usbd_start_out_urb(rcv_urb));
        TRACE_MSG0(MSC,"START RECV URB ERROR");
        usbd_free_urb(rcv_urb);
        return -EINVAL;
}

/*!
 * @brief  msc_start_sending_csw - start sending a new data or csw urb
 *
 * Generate a CSW (Command Status Wrapper) to send to the the host.
 *
 * Returns non-zero if there is an error in the USB layer.
 * @param function_instance
 * @param status
 * @return int
 */
int msc_start_sending_csw(struct usbd_function_instance *function_instance, u8 status)
{
        struct msc_private *msc = function_instance->privdata;
        struct msc_command_status_wrapper *csw;
        int length = sizeof(struct msc_command_status_wrapper);
        struct usbd_urb *tx_urb;

        //TRACE_MSG1(MSC,"START SENDING CSW %08x", status);

        msc->command_state = MSC_STATUS;

        RETURN_EINVAL_UNLESS((tx_urb = usbd_alloc_urb (function_instance, BULK_IN, length, msc_urb_sent)));

        tx_urb->actual_length = length;
        tx_urb->function_privdata = function_instance;

        // fill in CSW and queue the urb
        csw = (struct msc_command_status_wrapper *) tx_urb->buffer;
        csw->dCSWSignature = CSW_SIGNATURE;
        csw->dCSWTag = msc->command.dCBWTag;
        csw->dCSWDataResidue = abs(msc->command.dCBWDataTransferLength - msc->data_transferred_in_bytes);
        csw->bCSWStatus = status;

        TRACE_MSG2(MSC,"START SENDING CSW status: %02x data residue: %d", status, csw->dCSWDataResidue);

        RETURN_ZERO_UNLESS(usbd_start_in_urb (tx_urb));
        TRACE_MSG0(MSC,"START SENDING CSW FAILED");
        usbd_free_urb (tx_urb);
        return -EINVAL;
}

/*!
 * @brief msc_start_sending_csw_failed - starting sending a CSW showing failure
 *
 * Sets sensedata and generates a CSW with status set to USB_MSC_FAILED.
 *
 * Returns non-zero if there is an error in the USB layer.
 * @param function_instance
 * @param sensedata
 * @param info
 * @param status
 * @return int
 */
int msc_start_sending_csw_failed(struct usbd_function_instance *function_instance, u32 sensedata, u32 info, int status)
{
        struct msc_private *msc = function_instance->privdata;
        TRACE_MSG2(MSC, "sensedata: %x status: %x", sensedata, status);
        set_sense_data(function_instance, sensedata, info);

        return msc_start_sending_csw(function_instance, status);
}


/* ********************************************************************************************* */

/*!
 * @brief msc_alloc_urb - allocate an urb for returning a query
 * @param function_instance
 * @param length
 * @return pointer to a urb, Returns NULL if there is an error in the USB layer.

 */
struct usbd_urb * msc_alloc_urb(struct usbd_function_instance *function_instance, int length)
{
        struct msc_private *msc = function_instance->privdata;
        struct usbd_urb *urb;

        THROW_IF(!(urb = usbd_alloc_urb (function_instance, BULK_IN, length, msc_urb_sent)), error);
        return urb;
        CATCH(error) {
                msc->command_state = MSC_READY;
                return NULL;
        }
}

/*!
 * @brief  msc_dispatch_query_urb - dispatch an urb containing query data
 *
 * @param function_instance
 * @param urb
 * @param length
 * @param status
 * @return int Returns non-zero if there is an error in the USB layer.
 */
int msc_dispatch_query_urb(struct usbd_function_instance *function_instance, struct usbd_urb *urb, int length, int status)
{
        int rc;
        unsigned long flags;
        struct msc_private *msc = function_instance->privdata;

        TRACE_MSG2(MSC,"DISPATCH URB len: %d flags: %x", length, urb->flags);

        // save information in msc and urb
        //
        urb->function_privdata = function_instance;
        urb->actual_length = msc->TransferLength_in_bytes = length;

        // dispatch urb
        local_irq_save(flags);
        if ((rc = usbd_start_in_urb (urb))) {

                TRACE_MSG0(MSC,"DISPATCH URB FAILED");
                usbd_free_urb (urb);

                // stall?
                msc->command_state = MSC_READY;
                local_irq_restore(flags);
                return -EINVAL;
        }
        msc->command_state = MSC_QUERY;
        msc->status = status;
        local_irq_restore(flags);
        return 0;
}

/*!
 * @brief msc_dispatch_query_urb_zlp - send a ZLP
 *
 * @param function_instance
 * @param sensedata
 * @param info
 * @param status
 * @return int Returns non-zero if there is an error in the USB layer.
 */
int msc_dispatch_query_urb_zlp(struct usbd_function_instance *function_instance, u32 sensedata, u32 info, int status)
{
        struct msc_private *msc = function_instance->privdata;
        struct usbd_urb *urb;
        TRACE_MSG2(MSC, "sensedata: %x status: %x", sensedata, status);
        RETURN_EINVAL_IF(!(urb = msc_alloc_urb(function_instance, 1)));
        set_sense_data(function_instance, sensedata, info);
        urb->flags |= USBD_URB_SENDZLP;
        return msc_dispatch_query_urb(function_instance, urb, 0, status);
}

/* READ 10 COMMAND - read and send data to the host ******************************************** */
extern int msc_scsi_read_10(struct usbd_function_instance *function_instance, char *name, int op);
extern int msc_in_read_10_urb_sent(struct usbd_urb *tx_urb);

/* WRITE 10 - receive data from host and write to block device ********************************* */
extern int msc_scsi_write_10(struct usbd_function_instance *function_instance, char *name, int op);
extern void msc_recv_out_blocks(struct usbd_urb *rcv_urb);

/* SCSI Commands ******************************************************************************* */
extern unsigned char otg_message_cfg_cdrom(void);
/*!
 * @brief msc_scsi_inquiry - process an inquiry
 *
 * Used by:
 *      win2k
 *
 * @param function_instance
 * @param name
 * @param op
 * @return int Returns non-zero if there is an error in the USB layer.
 */
int msc_scsi_inquiry(struct usbd_function_instance *function_instance, char *name, int op)
{
        struct msc_private *msc = function_instance->privdata;
        struct msc_scsi_inquiry_command *command = (struct msc_scsi_inquiry_command *)&msc->command.CBWCB;
        struct msc_scsi_inquiry_data *data;
        struct usbd_urb *urb;
        int length = sizeof(struct msc_scsi_inquiry_data);
        u32 temmp;

        /*
         * C.f. SPC2 7.3 INQUIRY command
         * C.f. Table 46 - Standard INQUIRY data format
         *
         * C.f. Table 47 - Peripheral Qualifier
         *
         * 000b The specified peripheral device type is currently connected to this
         *      logical unit.....
         * 001b The device server is capable of of supporting the peripheral device
         *      type on this logical unit. However, the physical device is not currently
         *      connected to this logical unit.....
         * 010b Reserved
         * 011b The device server is not capable of supporting a physical device on
         *      this logical unit....
         *
         */

        TRACE_MSG4(MSC,"INQUIRY EnableVPD: %02x LogicalUnitNumber: %02x PageCode: %02x AllocLen: %02x",
                        command->EnableVPD, command->LogicalUnitNumber, command->PageCode, command->AllocationLength);
        TRACE_MSG1(MSC, "msc->command.dCBWDataTransferLength:%d", msc->command.dCBWDataTransferLength);
        // XXX THROW_IF(msc->command_state != MSC_READY, error);

        RETURN_EINVAL_IF(!(urb = msc_alloc_urb(function_instance, length)));
        memset(urb->buffer, 0, urb->alloc_length);
        data = (struct msc_scsi_inquiry_data *)urb->buffer;
#if 1
        data->PeripheralQaulifier=(msc->block_dev_state & DEVICE_EJECTED) + ~(msc->block_dev_state & DEVICE_CHANGE_ON);

#else
        data->PeripheralQaulifier = msc->block_dev_state & (DEVICE_EJECTED | DEVICE_CHANGE_ON) ? 0x1 : 0;
#endif

        // Always set PeripheralQaulifier to 0 to support linux based pc.
        data->PeripheralQaulifier = 0;
        
        TRACE_MSG1(MSC,"%d",data->PeripheralQaulifier);
        if (otg_message_cfg_cdrom()) {
                data->PeripheralDeviceType = 0x05;
                data->ResponseDataFormat = 0x2;
        }
        else {
                data->PeripheralDeviceType = 0x00;
                data->ResponseDataFormat = 0x1;
        }
        data->RMB = 0x1;
        data->AdditionalLength = 0x1f;

	    data->ProductRevisionLevel[0] = 0x31;      //Product Rev:1.01.
        data->ProductRevisionLevel[1] = 0x2E; 
        data->ProductRevisionLevel[2] = 0x30;
        data->ProductRevisionLevel[3] = 0x31;

        strncpy(data->VendorInformation, CONFIG_OTG_MSC_MANUFACTURER, strlen(CONFIG_OTG_MSC_MANUFACTURER));
        strncpy(data->ProductIdentification, CONFIG_OTG_MSC_PRODUCT_NAME, strlen(CONFIG_OTG_MSC_PRODUCT_NAME));

        return msc_dispatch_query_urb(function_instance, urb, msc->command.dCBWDataTransferLength, USB_MSC_PASSED);
}

/*!
 * @brief msc_scsi_read_format_capacity - process a query
 *
 * Used by:
 *      win2k
 *
 * @param function_instance
 * @param name
 * @param op
 * @return int Returns non-zero if there is an error in the USB layer.

 */
int msc_scsi_read_format_capacity(struct usbd_function_instance *function_instance, char *name, int op)
{
        struct msc_private *msc = function_instance->privdata;
        struct msc_scsi_read_format_capacity_data *data;
        struct usbd_urb *urb;
        int length = sizeof(struct msc_scsi_read_format_capacity_data);
        u32 block_num = msc->capacity;
        u32 block_len;

        RETURN_EINVAL_IF(!(urb = msc_alloc_urb(function_instance, length)));

        data = (struct msc_scsi_read_format_capacity_data *) urb->buffer;

        data->CapacityListHeader.CapacityListLength = sizeof(data->CurrentMaximumCapacityDescriptor);

        data->CurrentMaximumCapacityDescriptor.NumberofBlocks = block_num;
        data->CurrentMaximumCapacityDescriptor.DescriptorCode = 0x03;
        memcpy(data->CurrentMaximumCapacityDescriptor.BlockLength + 1, &block_len, sizeof(block_len));

        return msc_dispatch_query_urb(function_instance, urb, length, USB_MSC_PASSED);
}

/*!
 * @brief msc_read_capacity - process a read_capacity command
 *
 * Used by:
 *      win2k
 *
* @param function_instance
 * @param name
 * @param op
 * @return int Returns non-zero if there is an error in the USB layer.
  */
int msc_scsi_read_capacity(struct usbd_function_instance *function_instance, char *name, int op)
{
        struct msc_private *msc = function_instance->privdata;
        struct msc_scsi_read_capacity_command *command = (struct msc_scsi_read_capacity_command *)&msc->command.CBWCB;
        struct msc_scsi_read_capacity_data *data;
        struct usbd_urb *urb;
        int length = 8;
        u32 lba;

        /*
         * C.f. RBC 5.3
         */
        lba = be32_to_cpu(command->LogicalBlockAddress);

        TRACE_MSG1(MSC,"READ CAPACITY LBA: %d", lba);

        if ((command->PMI > 1) || (!command->PMI && lba)) {
                TRACE_MSG1(MSC,"READ CAPACITY PMI: %d", command->PMI);
                return msc_start_sending_csw_failed (function_instance, SCSI_SENSEKEY_INVALID_FIELD_IN_CDB, lba, USB_MSC_FAILED);
        }

        // alloc urb
        RETURN_EINVAL_IF(!(urb = msc_alloc_urb(function_instance, length)));

        data = (struct msc_scsi_read_capacity_data *) urb->buffer;

        /*change from capacity to (capacity-1) since LBA starts from 0 and ends at (capacity-1)*/  
        data->LastLogicalBlockAddress = cpu_to_be32(msc->capacity-1);
		

        data->BlockLengthInBytes = cpu_to_be32(msc->block_size);

        TRACE_MSG2(MSC,"RECV READ CAPACITY lba: %x block_size: %x",
                        be32_to_cpu(data->LastLogicalBlockAddress), be32_to_cpu(data->BlockLengthInBytes));

        return msc_dispatch_query_urb(function_instance, urb, length, USB_MSC_PASSED);
}


/*!
 * @brief msc_scsi_request_sense - process a request_sense command
 *
 * @param function_instance
 * @param name
 * @param op
 * @return int Returns non-zero if there is an error in the USB layer.
 */
int msc_scsi_request_sense(struct usbd_function_instance *function_instance, char *name, int op)
{
        struct msc_private *msc = function_instance->privdata;
        struct msc_scsi_request_sense_command *    command = (struct msc_scsi_request_sense_command  *)&msc->command.CBWCB;
        struct msc_scsi_request_sense_data *data;

        /*
         * C.f. SPC2 7.20 REQUEST SENSE command
         */

        struct usbd_urb *urb;
        int length = sizeof(struct msc_scsi_request_sense_data);

        // alloc urb
        RETURN_EINVAL_IF(!(urb = msc_alloc_urb(function_instance, length)));
        memset(urb->buffer, 0, urb->alloc_length);

        data = (struct msc_scsi_request_sense_data *) urb->buffer;
        data->ErrorCode = SCSI_ERROR_CURRENT;
        data->SenseKey = msc->sensedata >> 16;
        data->AdditionalSenseLength = 0xa; /* XXX is this needed */
        data->AdditionalSenseCode = msc->sensedata >> 8;
        data->AdditionalSenseCodeQualifier = msc->sensedata;
        data->Valid = 1;

        set_sense_data(function_instance, SCSI_SENSEKEY_NO_SENSE, 0);

        return msc_dispatch_query_urb(function_instance, urb, msc->command.dCBWDataTransferLength, USB_MSC_PASSED);
}

/*!
 * @brief msc_scsi_mode_sense - process a request_sense command
 *
 * Used by:
 *      win2k
 *
 * @param function_instance
 * @param name
 * @param op
 * @return int Returns non-zero if there is an error in the USB layer.
 */
int msc_scsi_mode_sense(struct usbd_function_instance *function_instance, char *name, int op)
{
	struct msc_private *msc = function_instance->privdata;
        //struct msc_scsi_mode_sense_command* command = (struct msc_scsi_mode_sense_command*)&msc->command.CBWCB;
        struct msc_mode_parameter_header* data;
        // We don't support any mode pages. So just return the Mode Parameter Header
        int length = sizeof(struct msc_mode_parameter_header);

        struct usbd_urb *urb;


        // alloc urb
        RETURN_EINVAL_IF(!(urb = msc_alloc_urb(function_instance, length)));
        memset (urb->buffer, 0, urb->alloc_length);

        data = (struct msc_mode_parameter_header*) urb->buffer;
        data->ModeDataLength = length - 1;
        data->WriteProtect = msc->block_dev_state & DEVICE_WRITE_PROTECTED ? 1 : 0;

        return msc_dispatch_query_urb(function_instance, urb, length, USB_MSC_PASSED);

}

static void put_be32(u8 *buf, u32 val) 
{
    buf[0] = val >> 24;
    buf[1] = val >> 16;
    buf[2] = val >> 8;
    buf[3] = val & 0xff;
}

/*!
 * @brief msc_scsi_read_toc - process a read table of contents command
 *
 * Used by:
 *      win2k
 *
 * @param function_instance
 * @param name
 * @param op
 * @return int Returns non-zero if there is an error in the USB layer.
 */
int msc_scsi_read_toc(struct usbd_function_instance *function_instance, char *name, int op)
{
        struct msc_private *msc = function_instance->privdata;
        struct msc_scsi_read_toc_command* command = (struct msc_scsi_read_toc_command*)&msc->command.CBWCB;
        struct msc_scsi_read_toc_data *data;
        int length = sizeof(struct msc_scsi_read_toc_data);

        struct usbd_urb *urb;
        u8 *cp;

        printk("READ TOC COMMAND: MSF: %02x Reserved7: %02x Alloc: %02x\n", command->MSF, command->Reserved7, command->AllocationLength);

        // alloc urb
        RETURN_EINVAL_IF(!(urb = msc_alloc_urb(function_instance, length)));

        data = (struct msc_scsi_read_toc_data *) urb->buffer;
        memset(data, 0x0, length);

        //data->TOCDataLength = 0x0A;
        data->TOCDataLength = 0x12;
        data->FirstCSN = 1;              
        data->LastCSN = 1;
        data->toc_track_descriptor.DataTrack = 0x16;
        data->toc_track_descriptor.FTNInLastCSN = 0x01;              //0 for FTNInLastCSN
        //data->toc_track_descriptor.LBAOfFTNInLastCS = 0;
        put_be32((u8*)&data->toc_track_descriptor.LBAOfFTNInLastCS, 0);
        data->LOTrackIsData = 0x16;
        data->LOTrackNumber = 0xAA;
        put_be32((u8*)&data->toc_track_descriptor.LBAOfFTNInLastCS, msc->max_blocks);
      
        return msc_dispatch_query_urb(function_instance, urb, length, USB_MSC_PASSED);
}

/*!
 * @brief msc_scsi_read_header - process a read header command
 *
 * Used by:
 *      win2k
 *
 * @param function_instance
 * @param name
 * @param op
 * @return int Returns non-zero if there is an error in the USB layer.
 */
int msc_scsi_read_header(struct usbd_function_instance *function_instance, char *name, int op)
{
        struct msc_private *msc = function_instance->privdata;
        u8 * buffer;
        int length = 8;
        struct usbd_urb *urb;
        u8 * buf;

        buffer = (u8*)&msc->command.CBWCB;

        printk("in read_header buffer[1]=0x%x\n", buffer[1]);
        if ((buffer[1] & ~0x02) != 0)
                return msc_start_sending_csw_failed (function_instance, SCSI_SENSEKEY_INVALID_FIELD_IN_CDB, msc->lba, USB_MSC_FAILED);

        if (msc->lba >= msc->max_blocks)
                return msc_start_sending_csw_failed (function_instance, SCSI_SENSEKEY_BLOCK_ADDRESS_OUT_OF_RANGE, msc->lba, USB_MSC_FAILED);
        // alloc urb
        RETURN_EINVAL_IF(!(urb = msc_alloc_urb(function_instance, length)));

        buf = urb->buffer;
        memset(buf, 0x0, length);

        buf[0] = 0x01;
        put_be32((u8*)&buf[4], msc->lba);

        return msc_dispatch_query_urb(function_instance, urb, length, USB_MSC_PASSED);
}

/*! msc_scsi_mode_sense_10 - process a mode sense command (10 byte version)
 *
 * Used by:
 *      winxp,macos
 *
 *
 * Returns non-zero if there is an error in the USB layer.
 */
int msc_scsi_mode_sense_10(struct usbd_function_instance *function_instance, char *name, int op)
{
	struct msc_private *msc = function_instance->privdata;
        //struct msc_scsi_mode_sense_command* command = (struct msc_scsi_mode_sense_command*)&msc->command.CBWCB;
        struct msc_mode_parameter_header_10* data;
        // We don't support any mode pages. So just return the Mode Parameter Header
        int length = sizeof(struct msc_mode_parameter_header_10);

        struct usbd_urb *urb;


        // alloc urb
        RETURN_EINVAL_IF(!(urb = msc_alloc_urb(function_instance, length)));
        memset (urb->buffer, 0, urb->alloc_length);

        data = (struct msc_mode_parameter_header_10*) urb->buffer;
        data->ModeDataLength = cpu_to_be16((unsigned short)(length - 2));
        data->WriteProtect = msc->block_dev_state & DEVICE_WRITE_PROTECTED ? 1 : 0;

        return msc_dispatch_query_urb(function_instance, urb, length, USB_MSC_PASSED);

}


/*! msc_scsi_test_unit_ready - process a test uint ready command
 *
 * Used by:
 *      win2k
 *
 * Returns non-zero if there is an error in the USB layer.
 */
int msc_scsi_test_unit_ready(struct usbd_function_instance *function_instance, char *name, int op)
{
        return msc_start_sending_csw(function_instance, USB_MSC_PASSED);
}


/*! msc_scsi_prevent_allow - process a prevent/allow command
 *
 * Used by:
 *      win2k
 *
 * Returns non-zero if there is an error in the USB layer.
 */
int msc_scsi_prevent_allow(struct usbd_function_instance *function_instance, char *name, int op)
{
        struct msc_private *msc = function_instance->privdata;
        struct msc_scsi_prevent_allow_media_removal_command* command =
                (struct msc_scsi_prevent_allow_media_removal_command*)&msc->command.CBWCB;

        /*
         * C.f. SPC2 7.12 Table 78 PREVENT ALLOW MEDIA REMOVAL Prevent Field
         *
         * 00b  Medium removal shall be allowed from both the data transport
         *      element and the attached medium changer (if any).
         * 01b  Medium removal shall be prohibited from the data transport
         *      element but allowed from the attached medium changer (if any).
         * 10b  Medium removal shall be allowed for the data transport element
         *      but prohibited for the attached medium changer.
         * 11b  Medium remval shall be prohibited from both the data transport
         *      element and the attached medium changer
         *
         * Prevention shall terminate after 00b or 10b, after a SYNC CACHE or hard reset.
         */

        // XXX TODO
        // this is from storageproto.c, shouldn't we implement something?
        if (command->Prevent)
                return msc_start_sending_csw_failed (function_instance, SCSI_SENSEKEY_INVALID_FIELD_IN_CDB, msc->lba, USB_MSC_FAILED);

        return msc_start_sending_csw(function_instance, USB_MSC_PASSED);
}


/*! msc_scsi_start_stop - process a start/stop command
 *
 * C.f. RBC 5.4 and 5.4.2
 *
 * Used by:
 *      win2k
 *
 * Returns non-zero if there is an error in the USB layer.
 */
int msc_scsi_start_stop(struct usbd_function_instance *function_instance, char *name, int op)
{
        struct msc_private *msc = function_instance->privdata;
        struct msc_scsi_start_stop_command* command = (struct msc_scsi_start_stop_command*)&msc->command.CBWCB;

        TRACE_MSG4(MSC,"START STOP: Immed: %d Power: %x LoEj: %d Start: %d",
                        command->IMMED, command->PowerConditions, command->LoEj, command->Start);
        /*
         * C.f. 5.4
         *
         * IMMED - if set return status immediately after command validation, otherwise
         * return status as soon operation is completed.
         *
         * C.f. 5.4.1 Table 8 POWER CONDITIONS
         *
         * 0 - M - no change in power condition
         * 1 - M - place device in active condition
         * 2 - M - place device in idle condition
         * 3 - M - place device in Standby condition
         * 4 -   - reserved
         * 5 - M - place device in Sleep condition
         * 6 -   - reserved
         * 7 - 0 - Device Control
         *
         * C.f. 5.4.2 Table 9 START STOP control bit definitions
         *
         * Power        Load/Eject      Start
         * 1-7          x               x               LoEj and Start Ignored
         * 0            0               0               Stop the medium
         * 0            0               1               Make the medium ready
         * 0            1               0               Unload the medium
         * 0            1               1               Load the medium
         *
         */
        // XXX TODO
        // this is from storageproto.c, shouldn't we implement something?

        if (command->Start && command->LoEj)
                return msc_start_sending_csw_failed (function_instance, SCSI_SENSEKEY_INVALID_FIELD_IN_CDB, msc->lba, USB_MSC_FAILED);

        return msc_start_sending_csw(function_instance, USB_MSC_PASSED);
}

/*! msc_scsi_verify - process a verify command
 *
 * Used by:
 *      win2k
 *
 * Returns non-zero if there is an error in the USB layer.
 */
int msc_scsi_verify(struct usbd_function_instance *function_instance, char *name, int op)
{
        struct msc_private *msc = function_instance->privdata;
        struct msc_scsi_verify_command *command = (struct msc_scsi_verify_command *)&msc->command.CBWCB;

        /*
         * C.f. RBC 5.7 VERIFY command
         */
        // XXX This actually should use the read_10 function and when
        // finished reading simply send the following
        return msc_start_sending_csw(function_instance, USB_MSC_PASSED);
}


/*! msc_scsi_mode_select - process a select command
 *
 * Returns non-zero if there is an error in the USB layer.
 */
int msc_scsi_mode_select(struct usbd_function_instance *function_instance, char *name, int op)
{
        //SCSI_MODE_SELECT_COMMAND *command = (SCSI_MODE_SELECT_COMMAND *)&msc->command.CBWCB;

        /*
         * C.f. SPC2 7.6 MODE SELECT(6) command
         */

        // if less than correct amount of data return USB_MSC_PHASE_ERROR - see MV
        //
        return msc_start_sending_csw(function_instance, USB_MSC_PASSED);
}

/*! msc_private_pcs - process a private command
 *
 * Returns non-zero if there is an error in the USB layer.
 */
int msc_cmd_private_pcs(struct usbd_function_instance *function_instance, char *name, int op)
{
        struct msc_private *msc = function_instance->privdata;
        return msc_start_sending_csw_failed (function_instance, SCSI_SENSEKEY_INVALID_COMMAND, msc->lba, USB_MSC_FAILED);
}

/*! msc_cmd_unknown - process an unknown command
 *
 * Returns non-zero if there is an error in the USB layer.
 */
int msc_cmd_unknown(struct usbd_function_instance *function_instance, char *name, int op)
{
        struct msc_private *msc = function_instance->privdata;
        return msc_start_sending_csw_failed (function_instance, SCSI_SENSEKEY_INVALID_COMMAND, msc->lba, USB_MSC_FAILED);
}

/*! @struct rbc_dispatch msc-fd.c "otg/functions/msc/msc-fd.c"
 * @brief encapsulation for rbc command
 */
struct rbc_dispatch {
        u8      op;
        char    *name;
        int     (*rbc_command) (struct usbd_function_instance *, char *, int op);
        int     device_check;
        u8      dataphase;
};

/*! Command cross reference
 *
 * This is the list of commands observed from each host OS. It is necessarily
 * incomplete in that we not have reached some condition necessary to have
 * other commands used.
 *                                      Win2k   WinXP
 * SCSI_TEST_UNIT_READY                 yes     yes
 * SCSI_READ_10                         yes     yes
 * SCSI_WRITE_10                        yes     yes
 * SCSI_READ_CAPACITY                   yes     yes
 * SCSI_VERIFY                          yes
 * SCSI_INQUIRY                         yes     yes
 * SCSI_MODE_SENSE                      yes
 * SCSI_READ_FORMAT_CAPACITY            yes     yes
 * SCSI_REQUEST_SENSE
 * SCSI_PREVENT_ALLOW_MEDIA_REMOVAL
 * SCSI_START_STOP
 * SCSI_MODE_SELECT
 * SCSI_FORMAT_UNIT
 *
 */

struct rbc_dispatch rbc_dispatch_table[] = {
        { SCSI_TEST_UNIT_READY,         "SCSI_TEST_UNIT_READY",         msc_scsi_test_unit_ready,     NOZLP,    NODATAPHASE },
        { SCSI_READ_10,                 "SCSI_READ_10",                 msc_scsi_read_10,       SENDZLP,   HASDATAPHASE },
        { SCSI_WRITE_10,                "SCSI_WRITE_10",                msc_scsi_write_10,      NOZLP,     HASDATAPHASE },
        { SCSI_READ_CAPACITY,           "SCSI_READ_CAPACITY",           msc_scsi_read_capacity, SENDZLP,   HASDATAPHASE },
        { SCSI_VERIFY,                  "SCSI_VERIFY",                  msc_scsi_verify,        NOCHK,     NODATAPHASE  },
        { SCSI_INQUIRY,                 "SCSI_INQUIRY",                 msc_scsi_inquiry,       NOCHK,     HASDATAPHASE },
        { SCSI_MODE_SENSE,              "SCSI_MODE_SENSE",              msc_scsi_mode_sense,    NOCHK,     HASDATAPHASE },
        { SCSI_READ_TOC,                "SCSI_READ_TOC",                msc_scsi_read_toc,       NOCHK,    HASDATAPHASE },
//        { SCSI_READ_HEADER,             "SCSI_READ_HEADER",             msc_scsi_read_header,    NOCHK,    HASDATAPHASE },
        { SCSI_MODE_SENSE_10,           "SCSI_MODE_SENSE_10",           msc_scsi_mode_sense_10,  NOCHK,    HASDATAPHASE },
        { SCSI_READ_FORMAT_CAPACITY,    "SCSI_READ_FORMAT_CAPACITY",    msc_scsi_read_format_capacity,     SENDZLP, HASDATAPHASE },
        { SCSI_REQUEST_SENSE,           "SCSI_REQUEST_SENSE",           msc_scsi_request_sense,    NOCHK, HASDATAPHASE},
        { SCSI_PREVENT_ALLOW_MEDIA_REMOVAL, "SCSI_PREVENT_ALLOW_MEDIA_REMOVAL", msc_scsi_prevent_allow,     NOZLP, NODATAPHASE },
        { SCSI_START_STOP,              "SCSI_START_STOP",              msc_scsi_start_stop,      NOZLP,    NODATAPHASE },
        { SCSI_MODE_SELECT,             "SCSI_MODE_SELECT",             msc_scsi_mode_select,     NOCHK ,   NODATAPHASE },
        { SCSI_FORMAT_UNIT,             "SCSI_FORMAT_UNIT",             msc_cmd_unknown,          NOCHK ,   UNSUPPORTED_CMD},

        { SCSI_READ_6,                  "SCSI_READ_6",                  msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },
        { SCSI_WRITE_6,                 "SCSI_WRITE_6",                 msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD},
        { SCSI_RESERVE,                 "SCSI_RESERVE",                 msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },
        { SCSI_RELEASE,                 "SCSI_RELEASE",                 msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },
        { SCSI_SEND_DIAGNOSTIC,         "SCSI_SEND_DIAGNOSTIC",         msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },
        { SCSI_SYNCHRONIZE_CACHE,       "SCSI_SYNCHRONIZE_CACHE",       msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },

        { SCSI_REZERO_UNIT,             "SCSI_REZERO_UNIT",             msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },
        { SCSI_SEEK_10,                 "SCSI_SEEK_10",                 msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },
        { SCSI_WRITE_AND_VERIFY,        "SCSI_WRITE_AND_VERIFY",        msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },
        { SCSI_WRITE_12,                "SCSI_WRITE_12",                msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },
        { SCSI_READ_12,                 "SCSI_READ_12",                 msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },
        { SCSI_MODE_SELECT_10,          "SCSI_MODE_SELECT_10",          msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },
        { SCSI_PRIVATE_PCS,             "SCSI_PRIVATE_PCS",             msc_cmd_private_pcs,      NOCHK,    UNSUPPORTED_CMD },

        { 0xff,                         "SCSI_UNKNOWN",                 msc_cmd_unknown,          NOCHK,    UNSUPPORTED_CMD },
};

void otg_require_switch(u16 mode);

/*! msc_recv_command - process a new CBW
 *
 * Return non-zero if urb was not disposed of.
 */
void msc_recv_command(struct usbd_urb *urb, struct usbd_function_instance *function_instance)
{
        struct msc_private *msc = function_instance->privdata;
        struct msc_command_block_wrapper *command = (struct msc_command_block_wrapper *)urb->buffer;
        u8 op = command->CBWCB[0];
        struct rbc_dispatch *dispatch;
        u8 target_mode = 0;

        /*
         * c.f. section 6.2 - Valid and Meaningful CBW
         * c.f. section 6.2.1 - Valid CBW
         *
         * The CBW was received after the device had sent a CSW or after a
         * reset XXX check that we only set MSC_READY after reset or sending
         * CSW.
         *
         * The CBW is 31 (1Fh) bytes in length and the bCBWSignature is
         * equal to 43425355h.
         */
        TRACE_MSG1(MSC,"urb->actual_length:%d", urb->actual_length);
        THROW_IF(31 != urb->actual_length, error);
        TRACE_MSG3(MSC,"le32_to_cpu(command->dCBWSignature):%08X , le32_to_cpu(command->dCBWTag):%08X, le32_to_cpu(command->dCBWDataTransferLength):%08X", le32_to_cpu(command->dCBWSignature), le32_to_cpu(command->dCBWTag), le32_to_cpu(command->dCBWDataTransferLength));

        THROW_IF(CBW_SIGNATURE != le32_to_cpu(command->dCBWSignature), error);

        /*
         * c.f. section 6.2.2 - Meaningful CBW
         *
         * no reserved bits are set
         * the bCBWLUN contains a valid LUN supported by the device
         * both bCBWCBlength and the content of the CBWCB are in accordance with bInterfaceSubClass
         */

        // XXX checklun etc

        /*
         * Success
         */
        if (op == 0xD6)
        {
                target_mode = command->CBWCB[10];
        }
        memcpy(&msc->command, command, sizeof(struct msc_command_block_wrapper));
        msc->data_transferred_in_bytes = msc->TransferLength_in_blocks = msc->TransferLength_in_bytes = 0;

        TRACE_TAG(command->dCBWTag, urb->framenum);
        usbd_free_urb(urb);
        urb = NULL;

        /*
         * Search using the opcode to find the dispatch function to use and
         * call it.
         */
        for (dispatch = rbc_dispatch_table; dispatch->op != 0xff; dispatch++) {
                CONTINUE_UNLESS ((dispatch->op == op));

                if(dispatch->dataphase == UNSUPPORTED_CMD){
                        if (msc->command.dCBWDataTransferLength==0){
                                msc_start_sending_csw_failed (function_instance, 0, 0, USB_MSC_FAILED);
                                return;
                        }
                        
                        if (msc->command.dCBWDataTransferLength>0) {
                                if (msc->command.bmCBWFlags & 0x80)
                                        usbd_halt_endpoint(function_instance, BULK_IN);  
                                        // this will make host sends CLEAR, so cleared code will be reached next"
                                else
                                        usbd_halt_endpoint(function_instance, BULK_OUT);

                                //msc->data_transferred_in_bytes = 0;
                                msc->command_state = MSC_CBW_FAILED;
                                return;
                        }
                        
                }
                //Case , Device no data
                if(dispatch->dataphase == NODATAPHASE){
                        /*Case 9 case 4 : If Host Expects data, while device expects no data, then report phase error directly */

                        if (msc->command.dCBWDataTransferLength>0) {
                                if (msc->command.bmCBWFlags & 0x80)
                                        usbd_halt_endpoint(function_instance, BULK_IN);
                                else
                                        usbd_halt_endpoint(function_instance, BULK_OUT);

                                //msc->data_transferred_in_bytes = 0;
                                msc->command_state = MSC_CBW_PASSED;    /* XXX always Passed???*/
                                return;
                        }

                }
                TRACE_CBW(dispatch->name, dispatch->op, dispatch->device_check);
                TRACE_RECV(MSC, 8, &(command->CBWCB[1]));

                /* Depending on the command we may need to check if the device is available
                 * and either fail or send a ZLP if it is not
                 */
                if (dispatch->device_check) {
                        RETURN_IF (msc_check_blockdev_name(function_instance, dispatch->device_check, dispatch->name));
                }

                /* Call the specific function that implements the specified command
                 */
                if (dispatch->rbc_command(function_instance, dispatch->name, op))
                        TRACE_MSG0(MSC,"COMMAND ERROR");
                return;
        }

        // If command wasn't on the known list, treat it as a vendor specific.
        if (op == 0xD6)
        {
                otg_require_switch((int)target_mode);
                target_mode = 0;
                msc_start_sending_csw(function_instance, USB_MSC_PASSED);
                return;
        }
        return;

        /* FALL THROUGH if no match is found */

        CATCH(error) {
                TRACE_MSG0(MSC,"RECV CBW ERROR");
                if (urb)
                        usbd_free_urb(urb);

                /* XXX which of these do we stall?
                 */

                TRACE_MSG0(MSC,"sos88 halt endpoint");
                usbd_halt_endpoint(function_instance, BULK_IN);
                usbd_halt_endpoint(function_instance, BULK_OUT);
                msc->endpoint_state=3;
                msc->command_state=MSC_WAITFOR_RESET;
        }
        //msc_cmd_unknown(msc, "CMD_UNKNOWN", op);
}


/* Sent Function - process a sent urb ********************************************************** */

/*! msc_urb_sent - called to indicate URB transmit finished
 * @param tx_urb: pointer to struct usbd_urb
 * @param rc: result
 *
 * This is called when an urb is sent. Depending on current state
 * it may:
 *
 *      - continue sending data
 *      - send a CSW
 *      - start a recv for a CBW
 *
 * This is called from BOTTOM HALF context.
 *
 * @return non-zero if urb was not disposed of.
 */
int msc_urb_sent (struct usbd_urb *tx_urb, int rc)
{
        struct usbd_function_instance *function_instance = tx_urb->function_privdata;
        struct msc_private *msc = function_instance->privdata;
        static unsigned char tempbuf=0;

        RETURN_EINVAL_IF(!(function_instance = tx_urb->function_instance));
        RETURN_EINVAL_IF(usbd_get_device_status(function_instance) == USBD_CLOSING);
        RETURN_EINVAL_IF(usbd_get_device_state(function_instance) != STATE_CONFIGURED);
#if defined(CONFIG_OTG_MSC_PIPES_TEST)
        memset(tx_urb->buffer,tempbuf++,512);
        RETURN_ZERO_UNLESS(usbd_start_in_urb (tx_urb));
        usbd_free_urb (tx_urb);
        return 0;
#else

        switch (msc->command_state) {
        case MSC_DATA_IN_READ:
        case MSC_DATA_IN_READ_FINISHED:
                TRACE_MSG0(MSC,"URB SENT READ");
                return msc_in_read_10_urb_sent(tx_urb);

        case MSC_QUERY:
                // finished, send CSW
                msc->data_transferred_in_bytes=tx_urb->actual_length;
                TRACE_MSG0(MSC,"URB SENT QUERY");
                msc_start_sending_csw(function_instance, USB_MSC_PASSED);
                break;

        case MSC_STATUS:
        default:
                // sent a CSW need to receive the next CBW
                TRACE_MSG0(MSC,"URB SENT STATUS");
                msc->command_state = MSC_READY;
                msc_start_recv_urb(function_instance, sizeof(struct msc_command_block_wrapper));
                break;
        }
        usbd_free_urb (tx_urb);
        return 0;
#endif
}


/* Receive Function - receiving an urb ********************************************************* */

/*! msc_recv_urb - process a received urb
 *
 * Return non-zero if urb was not disposed of.
 */
static int msc_recv_urb (struct usbd_urb *rcv_urb, int rc)
{
        struct usbd_function_instance *function_instance = rcv_urb->function_privdata;
        struct msc_private *msc = function_instance->privdata;
	
        RETURN_EINVAL_IF(!msc->connected);

        TRACE_MSG4(MSC, "RECV URB pointer: %08x, length: %d state: %d, rc:%d", rcv_urb, rcv_urb->actual_length, msc->command_state,rc);

        if(rc!=0) return -EINVAL; //XXX if urb callback with error, now do nothing

#if defined(CONFIG_OTG_MSC_PIPES_TEST)
        printk(KERN_INFO "rcv_urb->actual_length:%d\n",rcv_urb->actual_length);
        RETURN_ZERO_UNLESS(usbd_start_out_urb(rcv_urb));
        TRACE_MSG0(MSC,"START RECV URB ERROR");
        usbd_free_urb(rcv_urb);
        return -EINVAL;
#else /*defined(CONFIG_OTG_MSC_PIPES_TEST) */

        switch(msc->command_state) {

                // ready to start a new transaction
        case MSC_READY:
                msc_recv_command(rcv_urb, function_instance);
                return 0;

                // we think we are receiving data
        case MSC_DATA_OUT_WRITE:
        case MSC_DATA_OUT_WRITE_FINISHED:

                //if(rcv_urb->actual_length)
                //printk(KERN_INFO "sos00 rcv_urb->actual_length: %x\n", rcv_urb->actual_length);
                //else
                // printk(KERN_INFO "sos00 rcv_urb->actual_length = 0\n");

                msc_recv_out_blocks(rcv_urb);
                return 0;

                // we think we are sending data
        case MSC_DATA_IN_READ:
        case MSC_DATA_IN_READ_FINISHED:
                msc_start_sending_csw_failed (function_instance, SCSI_SENSEKEY_INVALID_COMMAND, msc->lba, USB_MSC_FAILED);
                break;

                // we think we are sending status
        case MSC_STATUS:
                msc_start_sending_csw_failed (function_instance, SCSI_SENSEKEY_INVALID_COMMAND, msc->lba, USB_MSC_FAILED);
                break;
        case MSC_WAITFOR_RESET:
                usbd_halt_endpoint(function_instance, BULK_IN);
                usbd_halt_endpoint(function_instance, BULK_OUT);
                msc->endpoint_state=3;
                break;

                // we don't think
        case MSC_UNKNOWN:
        default:
                TRACE_MSG0(MSC,"RECV URB ERROR");
                usbd_halt_endpoint(function_instance, BULK_IN);
                usbd_halt_endpoint(function_instance, BULK_OUT);
                msc->endpoint_state=3;
                msc->command_state=MSC_WAITFOR_RESET;
        }
        // let caller dispose of urb

        return -EINVAL;
#endif /*defined(CONFIG_OTG_MSC_PIPES_TEST)*/

}

/* USB Device Functions ************************************************************************ */


#if 0
static void msc_device_request (struct usbd_function_instance *function, struct usbd_device_request *request)
{
        TRACE_MSG0(MSC,"--");
}

static int msc_set_configuration (struct usbd_function_instance *function, int wValue)
{
        TRACE_MSG1(MSC,"wValue: %02x", wValue);
        return 0;
}

static int msc_set_interface (struct usbd_function_instance *function, int wIndex, int wValue)
{
        TRACE_MSG2(MSC,"wIndex: %02x wValue: %02x", wIndex, wValue);
        return 0;
}
#endif

/* USB Device Functions ************************************************************************ */

/*! msc_device_request - called to indicate urb has been received
 *
 * This function is called when a SETUP packet has been received that
 * should be handled by the function driver. It will not be called to
 * process the standard chapter nine defined requests.
 *
 * Return non-zero for failure.
 */
int msc_device_request(struct usbd_function_instance *function_instance, struct usbd_device_request *request)
{
        struct msc_private *msc = function_instance->privdata;
        struct usbd_urb *urb;

        u8 bRequest = request->bRequest;
        u8 bmRequestType = request->bmRequestType;
        u16 wValue = le16_to_cpu(request->wValue);
        u16 wIndex = le16_to_cpu(request->wIndex);
        u16 wLength = le16_to_cpu(request->wLength);

        TRACE_MSG5(MSC, "MSC RECV SETUP bmRequestType:%02x bRequest:%02x wValue:%04x wIndex:%04x wLength:%04x",
                        bmRequestType, bRequest, wValue, wIndex, wLength);

        // verify that this is a usb class request per cdc-acm specification or a vendor request.
        RETURN_ZERO_IF (!(request->bmRequestType & (USB_REQ_TYPE_CLASS | USB_REQ_TYPE_VENDOR)));

        // determine the request direction and process accordingly
        switch (request->bmRequestType & (USB_REQ_DIRECTION_MASK | USB_REQ_TYPE_MASK)) {

        case USB_REQ_HOST2DEVICE | USB_REQ_TYPE_CLASS:
                switch (request->bRequest) {
                case MSC_BULKONLY_RESET:
                        TRACE_MSG0(MSC, "MSC_BULKONLY_RESET");
                        RETURN_EINVAL_IF((wValue !=0) | (wIndex !=0) | ( wLength !=0));
                        // set msc back to reset state here
                        //.....

                        msc->command_state = MSC_WAITFOR_CLEAR;
                        msc->endpoint_state=3;
                        //////////////////////////
                        return 0;
                default:
                        TRACE_MSG0(MSC, "UNKNOWN H2D");
                        return -EINVAL;

                }

        case USB_REQ_DEVICE2HOST | USB_REQ_TYPE_CLASS:
                switch (request->bRequest) {
                case MSC_BULKONLY_GETMAXLUN:
                        RETURN_EINVAL_IF((wValue !=0) | (wIndex !=0) | ( wLength !=1));
                        RETURN_EINVAL_IF(!(urb = usbd_alloc_urb_ep0(function_instance, 1, NULL)));
                        urb->buffer[0] = 0;
                        urb->actual_length = 1;
                        RETURN_ZERO_IF(!usbd_start_in_urb(urb));
                        usbd_free_urb(urb);
                        return -EINVAL;
                default:
                        TRACE_MSG0(MSC, "UNKNOWN D2H");
                        return -EINVAL;
                }
        default:
                TRACE_MSG0(MSC,"unknown MSC device request");
                return -EINVAL;
                break;
        }
        return -EINVAL;
}

struct usbd_function_instance *proc_function_instance = NULL;

int msc_os_init_l24(struct usbd_function_instance *function_instance);
int msc_io_init_l24(struct usbd_function_instance *function_instance);
void msc_io_exit_l24(void);

static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
        return sprintf(buf, "%s\n", "usb_mass_storage");
}
static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
        return sprintf(buf, "%s\n", (container_of(sdev, struct msc_private, mass_sdev)->mass_storage_online ? "online" : "offline"));
}
void msc_uevent_work(struct work_struct *work)
{
        struct msc_private *msc = container_of(work, struct msc_private, work);
	    printk("mass_storage_online=%d\n", msc->mass_storage_online);
	        switch_set_state(&msc->mass_sdev, msc->mass_storage_online);
}

/*! msc_function_enable - this is called by the USBD core layer
 *
 * This is called to initialize the function when a bus interface driver
 * is loaded.
 */
static int msc_function_enable (struct usbd_function_instance *function_instance)
{
        int rc;
        struct msc_private *msc = NULL;

        RETURN_EINVAL_UNLESS((msc = CKMALLOC(sizeof(struct msc_private))));
	
	/* set is_msc_mode to 1, so musb pcd driver can know it is going into msc mode */
	is_msc_mode = 1;
        // XXX MODULE LOCK HERE

        function_instance->privdata = msc;

        msc->work_queue = create_singlethread_workqueue("msc_uevent");
        INIT_WORK(&msc->work, msc_uevent_work);
        msc->mass_storage_online = 0;
        msc->mass_sdev.name = "usb_mass_storage";
        msc->mass_sdev.print_name = print_switch_name;
        msc->mass_sdev.print_state = print_switch_state;
        rc = switch_dev_register(&msc->mass_sdev);
        if (rc < 0) {
                printk("%s: switch_dev_register failed\n", __FUNCTION__);
                return -EINVAL;
        }

        // XXX TODO need to verify that serial number is minimum of 12

        init_waitqueue_head(&msc->msc_wq);
        init_waitqueue_head(&msc->ioctl_wq);

        msc->block_dev_state = DEVICE_EJECTED;
        msc->command_state = MSC_READY;
        msc->io_state = MSC_INACTIVE;
        msc->command_state = MSC_READY;

        msc_os_init_l24(function_instance);
        msc_io_init_l24(function_instance);

        return 0;
}

extern void msc_close_blockdev (struct usbd_function_instance *function_instance);

/*! msc_function_disable - this is called by the USBD core layer
 *
 * This is called to close the function when a bus interface driver
 * is unloaded.
 */
static void msc_function_disable (struct usbd_function_instance *function_instance)
{
        struct msc_private *msc = function_instance->privdata;

        TRACE_MSG0(MSC,"FUNCTION EXIT");

	/* set is_msc_mode to 0, so musb pcd driver can know it's leaving msc mode */
	is_msc_mode = 0;
        msc->mass_storage_online = 0;
        queue_work(msc->work_queue, &msc->work);

        while (work_pending(&msc->work)) {
                printk("%s: waiting for MSC uevnet offline\n", __FUNCTION__);
                schedule_timeout(20);
        }

        switch_dev_unregister(&msc->mass_sdev);

        msc_close_blockdev(function_instance);
        function_instance->privdata = NULL;

        LKFREE(msc);
        msc_io_exit_l24();
        // XXX MODULE UNLOCK HERE
}


/*! msc_set_configuration 
 * @brief - called to set configuration according received urb 
 * @param function_instance
 * @param configuration
 * @return int
 */
int msc_set_configuration (struct usbd_function_instance *function_instance, int configuration)
{
        struct usbd_interface_instance *interface_instance = (struct usbd_interface_instance *)function_instance;
        struct msc_private *msc = function_instance->privdata;
        struct usbd_urb *tx_urb;

        TRACE_MSG0(MSC, "CONFIGURED");

        msc->mass_storage_online = 1;
        queue_work(msc->work_queue, &msc->work);

        // XXX Need to differentiate between non-zero, zero and non-zero done twice
        usbd_flush_endpoint_index(function_instance,BULK_IN);
        usbd_flush_endpoint_index(function_instance,BULK_OUT);

        TRACE_MSG0(MSC,"EVENT CONFIGURED");
        msc->endpoint_state=0;
        msc->connected = 1;
        msc->command_state = MSC_READY;
#if defined(CONFIG_OTG_MSC_PIPES_TEST)

        msc_start_recv_urb(function_instance, sizeof(struct msc_command_block_wrapper));

        RETURN_EINVAL_UNLESS((tx_urb = usbd_alloc_urb (function_instance, BULK_IN, 512, msc_urb_sent)));

        tx_urb->actual_length = 512;
        tx_urb->function_privdata = function_instance;
        memset(tx_urb->buffer,0x66,512);
        RETURN_ZERO_UNLESS(usbd_start_in_urb (tx_urb));
        usbd_free_urb (tx_urb);
        return -EINVAL;
#else /*defined(CONFIG_OTG_MSC_PIPES_TEST) */
        msc_start_recv_urb(function_instance, 512);
#endif
        #if 0
        local_irq_save(flags);
        if (msc->io_state & MSC_IOCTL_WAITING) {
                msc->io_state &= ~MSC_IOCTL_WAITING;
                TRACE_MSG0(MSC, "WAKEUP");
        }
        local_irq_restore(flags);
        #endif
        wake_up_interruptible(&msc->ioctl_wq);

        return 0;
}

/*! int msc_reset ( struct usbd_function_instance * )
 * @brief - called to process reset (urb) command 
 * @param function_instance
 * @return int
 */
int msc_reset (struct usbd_function_instance *function_instance)
{
        struct usbd_interface_instance *interface_instance = (struct usbd_interface_instance *)function_instance;
        struct msc_private *msc = function_instance ? function_instance->privdata : NULL;
        int connected = msc ? msc->connected : 0;

        TRACE_MSG1(MSC,"msc: %x", (int)msc);


        TRACE_MSG2(MSC,"EVENT RESET: connected %d msc->io_state %d", msc->connected, msc->io_state);

        #if 0
        local_irq_save(flags);
        if (msc->io_state & MSC_IOCTL_WAITING) {
                msc->io_state &= ~MSC_IOCTL_WAITING;
                TRACE_MSG0(MSC, "WAKEUP");
                wake_up_interruptible(&msc->ioctl_wq);
        }
        local_irq_restore(flags);
        #endif
        if (msc->connected ==1) wake_up_interruptible(&msc->ioctl_wq);
        msc->connected = 0;
        RETURN_ZERO_UNLESS(connected);

        // XXX we should have a semaphore to protect this
        RETURN_ZERO_UNLESS (msc->rcv_urb_finished);
        usbd_free_urb (msc->rcv_urb_finished);
        msc->rcv_urb_finished = NULL;
        msc->io_state=0;
        return 0;
}

/*! msc_suspended
 * @brief- called to process msc suspended command 
 * @param function_instance
 * @return int
 */
int msc_suspended (struct usbd_function_instance *function_instance)
{
        struct usbd_interface_instance *interface_instance = (struct usbd_interface_instance *)function_instance;
        struct msc_private *msc = function_instance->privdata;

        TRACE_MSG1(MSC,"msc: %x", (int)msc);
        if (msc->connected ==1) wake_up_interruptible(&msc->ioctl_wq);
        msc->connected =0;
        TRACE_MSG0(MSC, "SUSPENDED");
        return 0;
}


/*!int msc_resumed (struct usbd_function_instance * )
 * @brief msc_resumed - called to process msc resumed command 
 * @param function_instance
 * @return int
 */
int msc_resumed (struct usbd_function_instance *function_instance)
{
        struct usbd_interface_instance *interface_instance = (struct usbd_interface_instance *)function_instance;
        struct msc_private *msc = function_instance->privdata;
        TRACE_MSG1(MSC,"msc: %x", (int)msc);
        if (msc->connected ==0) wake_up_interruptible(&msc->ioctl_wq);
        msc->connected =1;
        TRACE_MSG0(MSC, "RESUMED");

        return 0;
}

/*!
 * @brief msc_endpoint_cleared- called to indicate endpoint has been cleared
 * @param function_instance
 * @param bEndpointAddress
 * @return none
 */
static void msc_endpoint_cleared (struct usbd_function_instance *function_instance, int bEndpointAddress)
{
        struct usbd_interface_instance *interface_instance = (struct usbd_interface_instance *)function_instance;
        struct msc_private *msc = function_instance->privdata;
        TRACE_MSG1(MSC,"bEndpointAddress: %02x", bEndpointAddress);

        switch (msc->command_state) {

        case MSC_CBW_PHASE_ERROR:
                msc_start_sending_csw(function_instance, USB_MSC_PHASE_ERROR);

                break;

        case MSC_CBW_PASSED:
                msc_start_sending_csw(function_instance, USB_MSC_PASSED);

                break;
        case MSC_CBW_FAILED:
                msc_start_sending_csw_failed (function_instance, 0, 0, USB_MSC_FAILED);
                break;
        case MSC_WAITFOR_RESET:
                usbd_halt_endpoint(function_instance, BULK_IN);
                usbd_halt_endpoint(function_instance, BULK_OUT);
                msc->endpoint_state=BULK_IN_HALTED | BULK_OUT_HALTED;
                break;
        case MSC_WAITFOR_CLEAR:
                if(bEndpointAddress&0x80){
                        msc->endpoint_state &= ~BULK_IN_HALTED;
                        TRACE_MSG0(MSC,"Bulk IN cleared");
                }
                else{
                        msc->endpoint_state &= ~BULK_OUT_HALTED;
                        TRACE_MSG0(MSC,"Bulk OUT cleared");
                }

                if(!msc->endpoint_state){
                        msc->command_state = MSC_READY;
          		msc_start_recv_urb(function_instance, sizeof(struct msc_command_block_wrapper));

                        TRACE_MSG0(MSC,"Ready to receive a new CBW");
                }
                break;

        default:
                break;
        }


}


/* ********************************************************************************************* */
/*! msc_function_ops usbd_function_option structure variable, initialized for all usbd core called operations
 */
struct usbd_function_operations msc_function_ops = {
        device_request: msc_device_request,
        function_enable: msc_function_enable,
        function_disable: msc_function_disable,
        set_configuration: msc_set_configuration,
        reset: msc_reset,
        suspended: msc_suspended,
        resumed: msc_resumed,
        endpoint_cleared: msc_endpoint_cleared,
};

/* ********************************************************************************************* */

