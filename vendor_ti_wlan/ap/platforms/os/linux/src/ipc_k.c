/*
 * ipc_k.c
 *
 * Copyright(c) 1998 - 2010 Texas Instruments. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Texas Instruments nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * src/ipc_k.c
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/netlink.h>
#include <linux/wireless.h>
#include <net/netlink.h>

#include "tidef.h"
#include "WlanDrvIf.h"
#include "osApi.h"
#include "ioctl_init.h"
#include "cli_cu_common.h"
#include "TI_IPC_Api.h"

#define DEBUG_IPC_K(str, args...)

static int custom_event_unicast_init(void);
static void send_custom_event_unicast(TI_HANDLE hOs, union iwreq_data *wrqu, char *extra, int pid);
TI_UINT32 IPCKernelInit    (TI_HANDLE hAdapter,TI_HANDLE  hIPCEv)
{
	return custom_event_unicast_init();
}

TI_UINT32 IPCKernelDeInit  (TI_HANDLE hAdapter)
{
	return 0;
}


/*******************************************************/
TI_INT32 IPC_EventSend(TI_HANDLE hAdapter, TI_UINT8* pEvData, TI_UINT32 EvDataSize)
{
	TWlanDrvIfObj *drv = (TWlanDrvIfObj *) hAdapter;
	union iwreq_data wrqu;

	/* This event is targetted to the OS process Id 0 is not a valid pId for LINUX*/

	if ((( IPC_EVENT_PARAMS *) pEvData) ->uProcessID == 0) {
		(( IPC_EVENT_PARAMS *) pEvData) ->pfEventCallback(( IPC_EV_DATA *) pEvData);
		return 0;
	}
	os_memorySet (hAdapter,&wrqu, 0, sizeof(wrqu));
	wrqu.data.length = sizeof(IPC_EV_DATA);
	/* All events in the system including logger are processed as wext event
	   We generate unicast custom wext event and send to destination process*/
	send_custom_event_unicast(drv, &wrqu, (char *)pEvData, (( IPC_EVENT_PARAMS *) pEvData) ->uProcessID);

	return 0;
}


static struct sk_buff_head custom_event_unicast_queue;

static int custom_event_unicast_init(void)
{
	skb_queue_head_init(&custom_event_unicast_queue);
	return 0;
}

static void custom_event_unicast_process(unsigned long data)
{
	struct sk_buff *skb;
	int err;
	int pid;

	while ((skb = skb_dequeue(&custom_event_unicast_queue))) {
		pid = NETLINK_CB(skb).pid;
		err = netlink_unicast(init_net.rtnl, skb, pid, MSG_DONTWAIT);
		if(err < 0) {
			DEBUG_IPC_K("Can't send unicast message to pid=%d err=%d\n", pid, err);
		}
	}
}

static DECLARE_TASKLET(custom_event_unicast_tasklet, custom_event_unicast_process, 0);


static void send_custom_event_unicast(TI_HANDLE hOs, union iwreq_data *wrqu, char *extra, int pid)
{
	int extra_len = 0;
	struct iw_event  *event;                /* Mallocated whole event */
	int event_len;                          /* Its size */
	int hdr_len;                            /* Size of the event header */
	struct sk_buff *skb;
	struct ifinfomsg *r;
	struct nlmsghdr  *nlh;
	TWlanDrvIfObj *pDrv = (TWlanDrvIfObj *)hOs;
	struct net_device *dev = pDrv->netdev;

	/* Total length of the event */
	if (extra != NULL)
		extra_len = wrqu->data.length;

	hdr_len = IW_EV_POINT_LEN;
	event_len = hdr_len + extra_len;

	/* Create temporary buffer to hold the event */
	event = kmalloc(event_len, GFP_ATOMIC);
	if (event == NULL)
		return;

	/* Fill event */
	event->len = event_len;
	event->cmd = IWEVCUSTOM;
	memcpy(&event->u, ((char *) wrqu) + IW_EV_POINT_OFF, hdr_len - IW_EV_LCP_LEN);
	if (extra)
		memcpy(((char *) event) + hdr_len, extra, extra_len);



	skb = nlmsg_new(NLMSG_DEFAULT_SIZE, GFP_ATOMIC);
	if (!skb) {
		kfree(event);
		return;
	}

	nlh = nlmsg_put(skb, 0, 0, RTM_NEWLINK, sizeof(*r), 0);
	if (nlh == NULL) {
		kfree(skb);
		kfree(event);
		return;
	}
	r = nlmsg_data(nlh);
	r->ifi_family = AF_UNSPEC;
	r->__ifi_pad = 0;
	r->ifi_type = dev->type;
	r->ifi_index = dev->ifindex;
	r->ifi_flags = dev_get_flags(dev);
	r->ifi_change = 0;

	NLA_PUT_STRING(skb, IFLA_IFNAME, dev->name);
	/* Add the wireless events in the netlink packet */
	NLA_PUT(skb, IFLA_WIRELESS, event_len, event);

	if( nlmsg_end(skb, nlh) < 0) {
		kfree(skb);
		kfree(event);
		return;
	}
	NETLINK_CB(skb).dst_group = RTNLGRP_LINK;
	NETLINK_CB(skb).pid = pid;

	/* The skb_queue is interrupt safe, and its lock is not held while calling
	* Netlink, so there is no possibility of dealock. */

	skb_queue_tail(&custom_event_unicast_queue, skb);
	tasklet_schedule(&custom_event_unicast_tasklet);
	kfree(event);
	return;
nla_put_failure:
	nlmsg_cancel(skb, nlh);
	kfree(skb);
	kfree(event);
}
