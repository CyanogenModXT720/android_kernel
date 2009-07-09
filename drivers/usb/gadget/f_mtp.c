/*
 * f_mtp.c -- USB MTP gadget driver
 *
 * Copyright (C) 2009 Motorola Corporation
 *
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* #define VERBOSE_DEBUG */

#include <linux/blkdev.h>
#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/freezer.h>
#include <linux/utsname.h>
#include <linux/miscdevice.h>

#include <linux/if.h>
#include <linux/in.h>
#include <linux/inetdevice.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <linux/io.h>
#include <linux/usb/composite.h>

#include "gadget_chips.h"
#include "f_mot_android.h"

#define BULK_BUFFER_SIZE    8192
#define MTP_CANCEL_IO_MARKER    0x1FFFFFFF

/*
 * MTP relevant strings
 */
#define STRING_INTERFACE	0
#define STRING_MTP      	1

/* static strings, in UTF-8 */
static struct usb_string mtp_string_defs[] = {
	[STRING_INTERFACE].s = "Motorola MTP Interface",
	[STRING_MTP].s = "MSFT100\034",
	{  /* ZEROES END LIST */ },
};

static struct usb_gadget_strings mtp_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		mtp_string_defs,
};

static struct usb_gadget_strings *mtp_strings[] = {
	&mtp_string_table,
	NULL,
};

/* There is only one interface. */
static struct usb_interface_descriptor intf_desc = {
	.bLength = sizeof intf_desc,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints = 3,
	.bInterfaceClass = 0x06,
	.bInterfaceSubClass = 0x01,
	.bInterfaceProtocol = 0x01,
};

static struct usb_endpoint_descriptor fs_bulk_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_bulk_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_intr_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_INT,
	.bInterval = 10,
};

static struct usb_descriptor_header *fs_mtp_descs[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	(struct usb_descriptor_header *) &fs_intr_in_desc,
	NULL,
};

static struct usb_endpoint_descriptor hs_bulk_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(512),
	.bInterval = 0,
};

static struct usb_endpoint_descriptor hs_bulk_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(512),
	.bInterval = 0,
};

static struct usb_endpoint_descriptor hs_intr_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize = __constant_cpu_to_le16(64),
	.bInterval = 10,
};

static struct usb_descriptor_header *hs_mtp_descs[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	(struct usb_descriptor_header *) &hs_intr_in_desc,
	NULL,
};

/* used when MTP function is disabled */
static struct usb_descriptor_header *null_mtp_descs[] = {
	NULL,
};

#define MAX_BULK_RX_REQ_NUM 8
#define MAX_BULK_TX_REQ_NUM 8
#define MAX_INTR_TX_REQ_NUM 8
/*---------------------------------------------------------------------------*/
struct usb_mtp_context {
	struct usb_function function;
	struct usb_composite_dev *cdev;

	spinlock_t lock;  /* For RX/TX/INT list */

	struct usb_ep *bulk_in;
	struct usb_ep *bulk_out;
	struct usb_ep *intr_in;

	struct list_head rx_reqs;
	struct list_head tx_reqs;
	struct list_head int_tx_reqs;

	struct list_head rx_done_reqs;
	int online;
	int cancel;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;

	wait_queue_head_t  ctrl_read_wq;
	struct usb_request *ctrl_read_req;
	int ctrl_recv_data_len;
	int ctrl_recv_buf;

	/* the request we're currently reading from */
	struct usb_request *read_req;

	/* buffer to point to available data in the current request */
	unsigned char *read_buf;
	/* available data length */
	int read_pos;
};

static struct usb_mtp_context g_usb_mtp_context;

/*-------------------------------------------------------------------------*/
static struct usb_request *req_new(struct usb_ep *ep, int size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void req_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

/* add a request to the tail of a list */
static void req_put(struct list_head *head, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&g_usb_mtp_context.lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&g_usb_mtp_context.lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&g_usb_mtp_context.lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&g_usb_mtp_context.lock, flags);
	return req;
}

/*-------------------------------------------------------------------------*/
static void mtp_int_complete(struct usb_ep *ep, struct usb_request *req)
{
	/* we don't need to support anything here */
	return;
}

static void mtp_out_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (req->status == 0) {
		req_put(&g_usb_mtp_context.rx_done_reqs, req);
	} else {
		printk(KERN_INFO "%s status is error\n", __func__);
		req_put(&g_usb_mtp_context.rx_reqs, req);
	}
}

static void mtp_in_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (req->status != 0)
		printk(KERN_INFO "%s status is error\n", __func__);

	req_put(&g_usb_mtp_context.tx_reqs, req);

}

static void
mtp_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_request *req;
	unsigned long flags;

	while ((req = req_get(&g_usb_mtp_context.rx_reqs)))
		req_free(req, g_usb_mtp_context.bulk_out);
	while ((req = req_get(&g_usb_mtp_context.rx_done_reqs)))
		req_free(req, g_usb_mtp_context.bulk_out);
	while ((req = req_get(&g_usb_mtp_context.tx_reqs)))
		req_free(req, g_usb_mtp_context.bulk_in);
	while ((req = req_get(&g_usb_mtp_context.int_tx_reqs)))
		req_free(req, g_usb_mtp_context.intr_in);
}

static int __init
mtp_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	int n, rc, id;
	struct usb_ep *ep;
	struct usb_request *req = NULL;
	struct proc_dir_entry *mtp_proc = NULL;

	g_usb_mtp_context.cdev = c->cdev;
	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	intf_desc.bInterfaceNumber = id;

	/* Find all the endpoints we will use */
	ep = usb_ep_autoconfig(g_usb_mtp_context.cdev->gadget,
		&fs_bulk_in_desc);
	if (!ep) {
		printk(KERN_INFO "%s auto-configure hs_bulk_in_desc error\n",
			__func__);
		goto autoconf_fail;
	}
	ep->driver_data = &g_usb_mtp_context;
	g_usb_mtp_context.bulk_in = ep;

	ep = usb_ep_autoconfig(g_usb_mtp_context.cdev->gadget,
			&fs_bulk_out_desc);
	if (!ep) {
		printk(KERN_INFO "%s auto-configure hs_bulk_out_desc error\n",
		      __func__);
		goto autoconf_fail;
	}
	ep->driver_data = &g_usb_mtp_context;
	g_usb_mtp_context.bulk_out = ep;

	ep = usb_ep_autoconfig(g_usb_mtp_context.cdev->gadget,
			&fs_intr_in_desc);
	if (!ep) {
		printk(KERN_INFO "%s auto-configure hs_intr_in_desc error\n",
		      __func__);
		goto autoconf_fail;
	}
	ep->driver_data = &g_usb_mtp_context;
	g_usb_mtp_context.intr_in = ep;

	if (gadget_is_dualspeed(g_usb_mtp_context.cdev->gadget)) {
		/* Assume endpoint addresses are the same for both speeds */
		hs_bulk_in_desc.bEndpointAddress =
		    fs_bulk_in_desc.bEndpointAddress;
		hs_bulk_out_desc.bEndpointAddress =
		    fs_bulk_out_desc.bEndpointAddress;
		hs_intr_in_desc.bEndpointAddress =
		    fs_intr_in_desc.bEndpointAddress;
	}

	rc = -ENOMEM;

	for (n = 0; n < MAX_BULK_RX_REQ_NUM; n++) {
		req = req_new(g_usb_mtp_context.bulk_out, BULK_BUFFER_SIZE);
		if (!req)
			goto autoconf_fail;

		req->complete = mtp_out_complete;
		req_put(&g_usb_mtp_context.rx_reqs, req);
	}
	for (n = 0; n < MAX_BULK_TX_REQ_NUM; n++) {
		req = req_new(g_usb_mtp_context.bulk_in, BULK_BUFFER_SIZE);
		if (!req)
			goto autoconf_fail;

		req->complete = mtp_in_complete;
		req_put(&g_usb_mtp_context.tx_reqs, req);
	}

	/* Interrupt Endpoint */
	for (n = 0; n < MAX_INTR_TX_REQ_NUM; n++) {
		req = req_new(g_usb_mtp_context.intr_in, BULK_BUFFER_SIZE);
		if (!req)
			goto autoconf_fail;

		req->complete = mtp_int_complete;
		req_put(&g_usb_mtp_context.int_tx_reqs, req);
	}

	return 0;

autoconf_fail:
	rc = -ENOTSUPP;
	mtp_function_unbind(c, f);
	return rc;
}

static void mtp_function_disable(struct usb_function *f)
{
	g_usb_mtp_context.online = 0;
	usb_ep_disable(g_usb_mtp_context.bulk_in);
	usb_ep_disable(g_usb_mtp_context.bulk_out);
	usb_ep_disable(g_usb_mtp_context.intr_in);

	g_usb_mtp_context.read_req = 0;
	g_usb_mtp_context.read_buf = 0;
	g_usb_mtp_context.read_pos = 0;
	/* readers may be blocked waiting for us to go online */
}

static void start_out_receive(void)
{
	struct usb_request *req;
	int ret;

	/* if we have idle read requests, get them queued */
	while ((req = req_get(&g_usb_mtp_context.rx_reqs))) {
		req->length = BULK_BUFFER_SIZE;
		ret = usb_ep_queue(g_usb_mtp_context.bulk_out, req, GFP_ATOMIC);
		if (ret < 0)
			req_put(&g_usb_mtp_context.rx_reqs, req);
	}
}

static int mtp_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	int ret;

	ret = usb_ep_enable(g_usb_mtp_context.bulk_in,
			ep_choose(g_usb_mtp_context.cdev->gadget,
				&hs_bulk_in_desc,
				&fs_bulk_in_desc));
	if (ret)
		return ret;
	ret = usb_ep_enable(g_usb_mtp_context.bulk_out,
			ep_choose(g_usb_mtp_context.cdev->gadget,
				&hs_bulk_out_desc,
				&fs_bulk_out_desc));
	if (ret) {
		usb_ep_disable(g_usb_mtp_context.bulk_in);
		return ret;
	}

	ret = usb_ep_enable(g_usb_mtp_context.intr_in,
			ep_choose(g_usb_mtp_context.cdev->gadget,
				&hs_intr_in_desc,
				&fs_intr_in_desc));
	if (ret) {
		usb_ep_disable(g_usb_mtp_context.bulk_in);
		usb_ep_disable(g_usb_mtp_context.bulk_out);
		return ret;
	}

	/* we're online -- get all rx requests queued */
	g_usb_mtp_context.read_req = 0;
	g_usb_mtp_context.read_buf = 0;
	g_usb_mtp_context.read_pos = 0;

	/* we're online -- get all rx requests queued */
	start_out_receive();

	g_usb_mtp_context.online = 1;
	g_usb_mtp_context.cancel = 0;

	usb_interface_enum_cb(MTP_TYPE_FLAG);

	/* readers may be blocked waiting for us to go online */

	return 0;
}

#define MTP_MOD_VENDOR_CODE   0x1C
static int  mtp_ext_id = 4;
static unsigned char mtp_ext_desc[] =
"\050\000\000\000\000\001\004\000\001\000\000\000\000\000\000\000\000\001"
"\115\124\120\000\000\000\000\000\060\060\000\000\000\000\000\000\000\000"
"\000\000\000\000";

static int  mtp_ext_str_idx = 238;

static int mtp_function_setup(struct usb_function *f,
					const struct usb_ctrlrequest *ctrl)
{
	int	value = -EOPNOTSUPP;
	u16     wIndex = le16_to_cpu(ctrl->wIndex);
	u16     wLength = le16_to_cpu(ctrl->wLength);
	struct  usb_request  *req = g_usb_mtp_context.cdev->req;
	struct usb_composite_dev *cdev = f->config->cdev;

	if ((ctrl->bRequestType & (USB_TYPE_CLASS | USB_TYPE_VENDOR)) == 0)
		return value;

	if ((ctrl->bRequest == MTP_MOD_VENDOR_CODE) &&
		((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) &&
		(wIndex == mtp_ext_id)) {
		memcpy(req->buf, mtp_ext_desc, sizeof(mtp_ext_desc));

		if (wLength < mtp_ext_desc[0])
			value = wLength;
		else
			value = mtp_ext_desc[0];
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		req->zero = value < wLength;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			printk(KERN_INFO "usb mtp setup response error\n");
		return value;
	}

	if (g_usb_mtp_context.online == 0)
		return value;

	/* The following process is just after interface is enabled */
	/* If there is data from Host to Device by EP0 */
	if (((ctrl->bRequestType & USB_DIR_IN) != USB_DIR_IN) &&
		wLength > 0)  {
		/* start one EP0 read */
	}
	/*  Save the request data for ctrl device read */

	return value;
}

int __init mtp_function_add(struct usb_composite_dev *cdev,
	struct usb_configuration *c)
{
	int ret = 0;
	int status;

	init_waitqueue_head(&g_usb_mtp_context.read_wq);
	init_waitqueue_head(&g_usb_mtp_context.write_wq);

	INIT_LIST_HEAD(&g_usb_mtp_context.rx_done_reqs);
	INIT_LIST_HEAD(&g_usb_mtp_context.rx_reqs);
	INIT_LIST_HEAD(&g_usb_mtp_context.tx_reqs);
	INIT_LIST_HEAD(&g_usb_mtp_context.int_tx_reqs);

	status = usb_string_id(c->cdev);
	if (status >= 0) {
		mtp_string_defs[STRING_INTERFACE].id = status;
		intf_desc.iInterface = status;
	}
	mtp_string_defs[STRING_MTP].id = mtp_ext_str_idx;

	g_usb_mtp_context.cdev = cdev;
	g_usb_mtp_context.function.name = "mtp";
	g_usb_mtp_context.function.descriptors = null_mtp_descs;
	g_usb_mtp_context.function.hs_descriptors = null_mtp_descs;
	g_usb_mtp_context.function.strings = mtp_strings;
	g_usb_mtp_context.function.bind = mtp_function_bind;
	g_usb_mtp_context.function.unbind = mtp_function_unbind;
	g_usb_mtp_context.function.setup = mtp_function_setup;
	g_usb_mtp_context.function.set_alt = mtp_function_set_alt;
	g_usb_mtp_context.function.disable = mtp_function_disable;

	ret = usb_add_function(c, &g_usb_mtp_context.function);
	if (ret) {
		printk(KERN_ERR "MTP gadget driver failed to initialize\n");
		return ret;
	}

	return 0;
}

struct usb_function *mtp_function_enable(int enable, int id)
{
	if (enable) {
		g_usb_mtp_context.function.descriptors = fs_mtp_descs;
		g_usb_mtp_context.function.hs_descriptors = hs_mtp_descs;
		intf_desc.bInterfaceNumber = id;
	} else {
		g_usb_mtp_context.function.descriptors = null_mtp_descs;
		g_usb_mtp_context.function.hs_descriptors = null_mtp_descs;
	}
	return &g_usb_mtp_context.function;
}
