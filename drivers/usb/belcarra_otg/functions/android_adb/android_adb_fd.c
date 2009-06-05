/*
 * otg/functions/android_adb/android_adb_fd.c
 *
 * Copyright 2008 Motorola, Inc.
 *
 * Changelog:
 * Date               Author           Comment
 * -----------------------------------------------------------------------------
 * 09/09/2008         Motorola         Changes for Open src compliance
 *
 * This Program is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of
 * MERCHANTIBILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at
 * your option) any later version.  You should have
 * received a copy of the GNU General Public License
 * along with this program; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave,
 * Cambridge, MA 02139, USA
 *
 */
/*!
 * @file otg/functions/android_adb/android_adb_fd.c
 *
 * The primary purpose of this driver is to demonstrate how to
 * implement a simple function driver and to provide a simple
 * uni-directional driver for testing USB Device peripheral drivers.
 */

#include <otg/otg-compat.h>
#include <otg/otg-module.h>

#include <otg/usbp-chap9.h>
#include <otg/usbp-hid.h>
#include <otg/usbp-func.h>
#include <otg/otg-trace.h>
#include <otg/otg-api.h>
#include <public/otg-node.h>
#include <otg/hotplug.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <otg/otg-linux.h>

EMBED_LICENSE();

//#define DEBUG_ADB

#ifdef DEBUG_ADB
#define DEBUG(fmt, arg...)    printk("%s(): " fmt, __FUNCTION__, ##arg)
#define ENTER_FUNC() printk("enter %s\n", __FUNCTION__)
#else
#define DEBUG(fmt, arg...)
#define ENTER_FUNC()
#endif

#define MAX_ADB_URBOUT_BUFFER 4096
#define MAX_ADB_URBIN_BUFFER  4096

enum {
        IF1_DATA_BULK_IN = 0,
        IF1_DATA_BULK_OUT,
        ENDPOINTS_NUM
};
                                                                                 
static u8 android_adb_if1_endpoint_index[] = { IF1_DATA_BULK_IN, IF1_DATA_BULK_OUT, };

static struct usbd_endpoint_request android_adb_endpoint_requests[ENDPOINTS_NUM+1] = {
        { IF1_DATA_BULK_IN, 1, 0, 0, USB_DIR_IN | USB_ENDPOINT_BULK, MAX_ADB_URBIN_BUFFER, MAX_ADB_URBIN_BUFFER, 0, },
        { IF1_DATA_BULK_OUT, 1, 0, 0, USB_DIR_OUT | USB_ENDPOINT_BULK, MAX_ADB_URBOUT_BUFFER, MAX_ADB_URBOUT_BUFFER, 0, },
        { 0, },
};


struct usbd_function_instance *android_adb_function_instance;
struct usbd_urb *android_adb_bulkin_urb, *android_adb_bulkout_urb;

/*! Interface Descriptions
 */
static struct usbd_alternate_description android_adb_if1_alternate_descriptions[] = {
      {
        .iInterface = "Motorola ADB interface",
              .bInterfaceClass = 0xff,
              .bInterfaceSubClass = 0x42,
              .bInterfaceProtocol = 0x01,
              .endpoints = sizeof (android_adb_if1_endpoint_index) / sizeof(u8),
              .endpoint_index =  android_adb_if1_endpoint_index,
      },
};

/*! List of Interface description(s)
 */
static struct usbd_interface_description android_adb_interfaces[] = {
        {
                alternates : sizeof (android_adb_if1_alternate_descriptions) / sizeof (struct usbd_alternate_description),
                alternate_list : android_adb_if1_alternate_descriptions,
        },
};

wait_queue_head_t adb_config_wait; 
wait_queue_head_t adb_read_wait; 
wait_queue_head_t adb_write_wait; 

int adb_read_cb_flag;
int adb_write_cb_flag;

/* adb mode indicator, used by musb-pcd driver to dynamic set USB FIFO size */
extern int is_adb_mode;

/* USB Device Functions ************************************************************************ */
/*! 
 * android_adb_event_handler() - process a device event
 * This function is called to process USB Device Events.
 */
void android_adb_event_handler (struct usbd_function_instance *function, usbd_device_event_t event, int data)
{
        switch (event) {
        case DEVICE_CONFIGURED:
                DEBUG("ADB Configured\n");
                break;

        case DEVICE_RESET:
                DEBUG("ADB Reset\n");
                break;

        case DEVICE_DE_CONFIGURED:
                DEBUG("ADB Reset2\n");
                break;
        default: 
                break;
        }
}

/*! 
 * android_adb_device_request - called to indicate urb has been received
 * @param function
 * @param request
 */
int android_adb_device_request (struct usbd_function_instance *function, struct usbd_device_request *request)
{
        ENTER_FUNC();

        switch (request->bmRequestType & (USB_REQ_DIRECTION_MASK | USB_REQ_TYPE_MASK)) {

        case USB_REQ_HOST2DEVICE:
        case USB_REQ_HOST2DEVICE | USB_REQ_TYPE_CLASS:
                return 0;

        case USB_REQ_HOST2DEVICE | USB_REQ_TYPE_VENDOR: 
                switch (request->bRequest) {
                case 1:
        /* Mode switch request from PC, return error to make usb_vendor_request() work in usbp-bops.c */
                        return -EINVAL;
                default:  
                        break;
                }
                break;

        case USB_REQ_DEVICE2HOST :
        case USB_REQ_DEVICE2HOST | USB_REQ_TYPE_CLASS:
        case USB_REQ_DEVICE2HOST | USB_REQ_TYPE_VENDOR:

                switch (request->bRequest) {
                case USB_REQ_GET_DESCRIPTOR:
                default: 
                        break;
                }
                break;

        default:
                break;
        }
        return 0;
}


int android_adb_bulkout_cb (struct usbd_urb *urb, int rc) 
{
        int ret = 0 ;

        ENTER_FUNC();
        if (USBD_URB_CANCELLED == rc) { 
                DEBUG("%s: USBD_URB_CANCELLED\n", __FUNCTION__); 
        } 
        if (USBD_URB_OK != rc) { 
                DEBUG("%s: USBD_URB_OK\n", __FUNCTION__); 
        }
        adb_read_cb_flag = 1;
        wake_up_interruptible(&adb_read_wait);
        return ret;
}

int android_adb_bulkin_cb (struct usbd_urb *urb, int rc)
{
        int ret = 0;
                                                                                                                
        ENTER_FUNC();
        if (USBD_URB_CANCELLED == rc) {
                DEBUG("%s: USBD_URB_CANCELLED\n", __FUNCTION__);
        }
        if (USBD_URB_OK != rc) {
                DEBUG("%s: USBD_URB_OK\n", __FUNCTION__);
        }
        adb_write_cb_flag = 1;
        wake_up_interruptible(&adb_write_wait);
        return ret;
}

static int android_adb_alloc_urbs(void)
{
        android_adb_bulkin_urb = usbd_alloc_urb(android_adb_function_instance, IF1_DATA_BULK_IN, MAX_ADB_URBIN_BUFFER, android_adb_bulkin_cb);
        android_adb_bulkout_urb = usbd_alloc_urb(android_adb_function_instance, IF1_DATA_BULK_OUT, MAX_ADB_URBOUT_BUFFER, android_adb_bulkout_cb);
                                                                                                             
        return 0;
}
static int android_adb_free_urbs(void)
{
        ENTER_FUNC();
        usbd_free_urb(android_adb_bulkin_urb);
        usbd_free_urb(android_adb_bulkout_urb);
        android_adb_bulkin_urb = android_adb_bulkout_urb = NULL;                                                                                                             
        return 0;
}

int adb_device_config_flag;
int adb_open_flag;

/* file operation for USB DATA test */
static int android_adb_open(struct inode * inode, struct file * file)
{
        ENTER_FUNC();
        if(adb_open_flag != 0) {
                DEBUG("%s failed: ADB has been opened\n", __FUNCTION__);
                return -1;
        }
        adb_open_flag ++;
        return 0;
}
                                                                                                                     
static int android_adb_release(struct inode * inode, struct file * file)
{
        ENTER_FUNC();
        adb_open_flag --;
        return 0;
}

static ssize_t android_adb_read(struct file * filp, char __user * buf, size_t count, loff_t * l)
{
        int ret, copied = 0;
        ENTER_FUNC();
        wait_event_interruptible(adb_config_wait, adb_device_config_flag );

        if (!adb_device_config_flag)
                return -ENODEV;

        if (count > MAX_ADB_URBOUT_BUFFER)
                count = MAX_ADB_URBIN_BUFFER;

read_again:
        if (usbd_start_out_urb(android_adb_bulkout_urb))
                return -EINVAL;

        // wait until read urb is done.
        wait_event_interruptible(adb_read_wait, adb_read_cb_flag );
        adb_read_cb_flag = 0;

        if (!android_adb_bulkout_urb || USBD_URB_CANCELLED == android_adb_bulkout_urb->status || !adb_device_config_flag)
                return -ENODEV;

        if((android_adb_bulkout_urb->status == USBD_URB_OK) && (android_adb_bulkout_urb->actual_length == 0)) {
	    DEBUG("ADB received zero length URB, so let's read again \n ");
            goto read_again;
        }

        ret = android_adb_bulkout_urb->actual_length;
        copied = copy_to_user(buf, android_adb_bulkout_urb->buffer, ret);
        return ret-copied;
}

static ssize_t android_adb_write(struct file * filp, const char * buf, size_t count, loff_t * l)
{
        int copied;
        ENTER_FUNC();
        wait_event_interruptible(adb_config_wait, adb_device_config_flag );

        if (!adb_device_config_flag)
                return -ENODEV;

        if (count > MAX_ADB_URBIN_BUFFER) {
                count = MAX_ADB_URBIN_BUFFER;
        }
        copied = copy_from_user(android_adb_bulkin_urb->buffer, buf, count);
        android_adb_bulkin_urb->buffer_length = count;
        android_adb_bulkin_urb->actual_length = count - copied;

        if(usbd_start_in_urb(android_adb_bulkin_urb))
                return -EINVAL;

        // wait until writ urb is done.
        wait_event_interruptible(adb_write_wait, adb_write_cb_flag );
        adb_write_cb_flag = 0;
        return count - copied;
}

static struct file_operations android_adb_fops = {
        owner:          THIS_MODULE,
        open:           android_adb_open,
        release:        android_adb_release,
        read:           android_adb_read,
        write:          android_adb_write,
};

static struct miscdevice android_adb_device = {
        MISC_DYNAMIC_MINOR, "android_adb", &android_adb_fops,
};

/*! android_adb_function_enable - called by USB Device Core to enable the driver
 * @param function The function instance for this driver to use.
 * @return non-zero if error.
 */
static int android_adb_function_enable (struct usbd_function_instance *function)
{
        ENTER_FUNC();
        android_adb_function_instance = function;
	is_adb_mode = 1;
        return 0;
}

/*! android_adb_function_disable - called by the USB Device Core to disable the driver
 * @param function The function instance for this driver
 */
static void android_adb_function_disable (struct usbd_function_instance *function)
{               
        ENTER_FUNC();
        adb_device_config_flag = 0;
        adb_read_cb_flag = 0;
        adb_write_cb_flag = 0;
	is_adb_mode = 0;
        android_adb_free_urbs();
}

static void android_adb_endpoint_cleared (struct usbd_function_instance *function, int bEndpointAddress)
{
        ENTER_FUNC();
}

static int android_adb_set_configuration (struct usbd_function_instance *function_instance, int bEndpointAddress)
{
        ENTER_FUNC();
        android_adb_alloc_urbs();

        adb_device_config_flag = 1;
        wake_up_interruptible(&adb_config_wait);
        return 0;
}
                                                                                    
int android_adb_set_interface (struct usbd_function_instance *function_instance, int wIndex, int altsetting)
{
        ENTER_FUNC();
        return 0;
}
                                                                                    
int android_adb_reset (struct usbd_function_instance *function_instance)
{
        ENTER_FUNC();
        adb_device_config_flag = 0;
        return 0;
}
int android_adb_suspended (struct usbd_function_instance *function_instance)
{
        ENTER_FUNC();
        adb_device_config_flag = 0;
        return 0;
}
                                                                                    
int android_adb_resumed (struct usbd_function_instance *function_instance)
{
        ENTER_FUNC();
        adb_device_config_flag = 1;
        return 0;
}


/* ********************************************************************************************* */
/*! function_ops - operations table for the USB Device Core
 */
static struct usbd_function_operations android_adb_function_ops = {
        .event_handler = android_adb_event_handler,                     /*!< called for each USB Device Event */
        .device_request = android_adb_device_request,           /*!< called for each received device request */
        .function_enable = android_adb_function_enable,         /*!< called to enable the function driver */
        .function_disable = android_adb_function_disable,       /*!< called to disable the function driver */
        .endpoint_cleared = android_adb_endpoint_cleared,

        .set_configuration = android_adb_set_configuration,
        .set_interface = android_adb_set_interface,
        .reset = android_adb_reset,
        .suspended = android_adb_suspended,
        .resumed = android_adb_resumed,
};

/*! function_driver
 */
struct usbd_interface_driver android_adb_function_driver = {
        .driver = {
                .name = "android_adb-if",
                .fops = &android_adb_function_ops, },
        .interfaces = sizeof (android_adb_interfaces) / sizeof (struct usbd_interface_description),
        .interface_list = android_adb_interfaces,
        .endpointsRequested =  ENDPOINTS_NUM,
        .requestedEndpoints =  android_adb_endpoint_requests,
};

/* Module Parameters ******************************************************** */
/* USB Module init/exit ***************************************************** */

/*! 
 * module init
 */
static int android_adb_modinit (void)
{
        ENTER_FUNC();
        init_waitqueue_head(&adb_config_wait);
        adb_device_config_flag = 0;
        adb_open_flag = 0;
        init_waitqueue_head(&adb_read_wait);
        adb_read_cb_flag = 0;
        init_waitqueue_head(&adb_write_wait);
        adb_write_cb_flag = 0;

        // register as usb function driver
        usbd_register_interface_function (&android_adb_function_driver, "android_adb-if", NULL);
        misc_register(&android_adb_device);

        return 0;
}

static void android_adb_modexit (void)
{
        ENTER_FUNC();
        usbd_deregister_interface_function (&android_adb_function_driver);
        misc_deregister(&android_adb_device);
}

module_init (android_adb_modinit);
module_exit (android_adb_modexit);
