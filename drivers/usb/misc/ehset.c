/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 *
 */


#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/usb.h>

#include "../core/hub.h"

#define TEST_SE0_NAK			0x0101
#define TEST_J				0x0102
#define TEST_K				0x0103
#define TEST_PACKET 			0x0104
#define TEST_HS_HOST_PORT_SUSPEND_RESUME 0x0106
#define TEST_SINGLE_STEP_GET_DEV_DESC	0x0107
#define TEST_SINGLE_STEP_SET_FEATURE	0x0108

static int ehset_probe(struct usb_interface *intf,
		       const struct usb_device_id *id)
{
	int status = -1;
	struct usb_device *dev = interface_to_usbdev(intf);
	struct usb_device *rh_udev = dev->bus->root_hub;
	struct usb_device *hub_udev = dev->parent;
	int port1 = dev->portnum;
	int test_mode = le16_to_cpu(dev->descriptor.idProduct);

	switch (test_mode) {
	case TEST_SE0_NAK:
		status = usb_control_msg(hub_udev, usb_sndctrlpipe(hub_udev, 0),
			USB_REQ_SET_FEATURE, USB_RT_PORT, USB_PORT_FEAT_TEST,
			(3 << 8) | port1, NULL, 0, 1000);
		break;
	case TEST_J:
		status = usb_control_msg(hub_udev, usb_sndctrlpipe(hub_udev, 0),
			USB_REQ_SET_FEATURE, USB_RT_PORT, USB_PORT_FEAT_TEST,
			(1 << 8) | port1, NULL, 0, 1000);
		break;
	case TEST_K:
		status = usb_control_msg(hub_udev, usb_sndctrlpipe(hub_udev, 0),
			USB_REQ_SET_FEATURE, USB_RT_PORT, USB_PORT_FEAT_TEST,
			(2 << 8) | port1, NULL, 0, 1000);
		break;
	case TEST_PACKET:
		status = usb_control_msg(hub_udev, usb_sndctrlpipe(hub_udev, 0),
			USB_REQ_SET_FEATURE, USB_RT_PORT, USB_PORT_FEAT_TEST,
			(4 << 8) | port1, NULL, 0, 1000);
		break;
	case TEST_HS_HOST_PORT_SUSPEND_RESUME:
		/* Test: wait for 15secs -> suspend -> 15secs delay -> resume */
		msleep(15 * 1000);
		status = usb_control_msg(hub_udev, usb_sndctrlpipe(hub_udev, 0),
			USB_REQ_SET_FEATURE, USB_RT_PORT,
			USB_PORT_FEAT_SUSPEND, port1, NULL, 0, 1000);
		if (status < 0)
			break;
		msleep(15 * 1000);
		status = usb_control_msg(hub_udev, usb_sndctrlpipe(hub_udev, 0),
			USB_REQ_CLEAR_FEATURE, USB_RT_PORT,
			USB_PORT_FEAT_SUSPEND, port1, NULL, 0, 1000);
		break;
	case TEST_SINGLE_STEP_GET_DEV_DESC:
		/* Test: wait for 15secs -> GetDescriptor request */
		msleep(15 * 1000);
		{
			struct usb_device_descriptor *buf;
			buf = kmalloc(USB_DT_DEVICE_SIZE, GFP_KERNEL);
			if (!buf)
				return -ENOMEM;

			status = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
				USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
				USB_DT_DEVICE << 8, 0,
				buf, USB_DT_DEVICE_SIZE,
				USB_CTRL_GET_TIMEOUT);
			kfree(buf);
		}
		break;
	case TEST_SINGLE_STEP_SET_FEATURE:
		/* GetDescriptor's SETUP request -> 15secs delay -> IN & STATUS
		 * Issue request to ehci root hub driver with portnum = 1
		 */
		status = usb_control_msg(rh_udev, usb_sndctrlpipe(rh_udev, 0),
			USB_REQ_SET_FEATURE, USB_RT_PORT, USB_PORT_FEAT_TEST,
			(6 << 8) | 1, NULL, 0, 60 * 1000);

		break;
	default:
		pr_err("%s: undefined test mode ( %X )\n", __func__, test_mode);
		return -EINVAL;
	}

	return (status < 0) ? status : 0;
}

static void ehset_disconnect(struct usb_interface *intf)
{
}

static struct usb_device_id ehset_id_table[] = {
	{ USB_DEVICE(0x1a0a, TEST_SE0_NAK) },
	{ USB_DEVICE(0x1a0a, TEST_J) },
	{ USB_DEVICE(0x1a0a, TEST_K) },
	{ USB_DEVICE(0x1a0a, TEST_PACKET) },
	{ USB_DEVICE(0x1a0a, TEST_HS_HOST_PORT_SUSPEND_RESUME) },
	{ USB_DEVICE(0x1a0a, TEST_SINGLE_STEP_GET_DEV_DESC) },
	{ USB_DEVICE(0x1a0a, TEST_SINGLE_STEP_SET_FEATURE) },
	{ }			/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, ehset_id_table);

static struct usb_driver ehset_driver = {
	.name =		"usb_ehset_test",
	.probe =	ehset_probe,
	.disconnect =	ehset_disconnect,
	.id_table =	ehset_id_table,
};

static int __init ehset_init(void)
{
	return usb_register(&ehset_driver);
}

static void __exit ehset_exit(void)
{
	usb_deregister(&ehset_driver);
}

module_init(ehset_init);
module_exit(ehset_exit);

MODULE_DESCRIPTION("USB Driver for EHSET Test Fixture");
MODULE_LICENSE("Dual BSD/GPL");
