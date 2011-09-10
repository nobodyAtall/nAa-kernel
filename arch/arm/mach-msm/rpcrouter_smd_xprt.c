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

/*
 * RPCROUTER SMD XPRT module.
 */

#include <linux/platform_device.h>
#include <linux/types.h>

#include <mach/msm_smd.h>
#include "smd_rpcrouter.h"
#include "smd_private.h"

struct rpcrouter_smd_xprt {
	struct rpcrouter_xprt xprt;

	smd_channel_t *channel;
};

static struct rpcrouter_smd_xprt smd_remote_xprt;

static int rpcrouter_smd_remote_read_avail(void)
{
	return smd_read_avail(smd_remote_xprt.channel);
}

static int rpcrouter_smd_remote_read(void *data, uint32_t len)
{
	return smd_read(smd_remote_xprt.channel, data, len);
}

static int rpcrouter_smd_remote_write_avail(void)
{
	return smd_write_avail(smd_remote_xprt.channel);
}

static int rpcrouter_smd_remote_write(void *data, uint32_t len)
{
	return smd_write(smd_remote_xprt.channel, data, len);
}

static int rpcrouter_smd_remote_close(void)
{
	smsm_change_state(SMSM_APPS_STATE, SMSM_RPCINIT, 0);
	return smd_close(smd_remote_xprt.channel);
}

static void rpcrouter_smd_remote_notify(void *_dev, unsigned event)
{
	if (event == SMD_EVENT_DATA)
		msm_rpcrouter_xprt_notify(&smd_remote_xprt.xprt,
					  RPCROUTER_XPRT_EVENT_DATA);
}

#if defined(CONFIG_MSM_RPC_LOOPBACK_XPRT)

static struct rpcrouter_smd_xprt smd_loopback_xprt;

static int rpcrouter_smd_loopback_read_avail(void)
{
	return smd_read_avail(smd_loopback_xprt.channel);
}

static int rpcrouter_smd_loopback_read(void *data, uint32_t len)
{
	return smd_read(smd_loopback_xprt.channel, data, len);
}

static int rpcrouter_smd_loopback_write_avail(void)
{
	return smd_write_avail(smd_loopback_xprt.channel);
}

static int rpcrouter_smd_loopback_write(void *data, uint32_t len)
{
	return smd_write(smd_loopback_xprt.channel, data, len);
}

static int rpcrouter_smd_loopback_close(void)
{
	return smd_close(smd_loopback_xprt.channel);
}

static void rpcrouter_smd_loopback_notify(void *_dev, unsigned event)
{
	if (event == SMD_EVENT_DATA)
		msm_rpcrouter_xprt_notify(&smd_loopback_xprt.xprt,
					  RPCROUTER_XPRT_EVENT_DATA);
}

static int rpcrouter_smd_loopback_probe(struct platform_device *pdev)
{
	int rc;

	smd_loopback_xprt.xprt.name = "rpcrouter_loopback_xprt";
	smd_loopback_xprt.xprt.read_avail = rpcrouter_smd_loopback_read_avail;
	smd_loopback_xprt.xprt.read = rpcrouter_smd_loopback_read;
	smd_loopback_xprt.xprt.write_avail = rpcrouter_smd_loopback_write_avail;
	smd_loopback_xprt.xprt.write = rpcrouter_smd_loopback_write;
	smd_loopback_xprt.xprt.close = rpcrouter_smd_loopback_close;

	/* Open up SMD LOOPBACK channel */
	rc = smd_named_open_on_edge("local_loopback", SMD_LOOPBACK_TYPE,
				    &smd_loopback_xprt.channel, NULL,
				    rpcrouter_smd_loopback_notify);
	if (rc < 0)
		return rc;

	msm_rpcrouter_xprt_notify(&smd_loopback_xprt.xprt,
				  RPCROUTER_XPRT_EVENT_OPEN);
	return 0;
}

static struct platform_driver rpcrouter_smd_loopback_driver = {
	.probe		= rpcrouter_smd_loopback_probe,
	.driver		= {
			.name	= "local_loopback",
			.owner	= THIS_MODULE,
	},
};

#endif

static int rpcrouter_smd_remote_probe(struct platform_device *pdev)
{
	int rc;

	smd_remote_xprt.xprt.name = "rpcrotuer_smd_xprt";
	smd_remote_xprt.xprt.read_avail = rpcrouter_smd_remote_read_avail;
	smd_remote_xprt.xprt.read = rpcrouter_smd_remote_read;
	smd_remote_xprt.xprt.write_avail = rpcrouter_smd_remote_write_avail;
	smd_remote_xprt.xprt.write = rpcrouter_smd_remote_write;
	smd_remote_xprt.xprt.close = rpcrouter_smd_remote_close;

	/* Open up SMD channel */
	rc = smd_open("RPCCALL", &smd_remote_xprt.channel, NULL,
		      rpcrouter_smd_remote_notify);
	if (rc < 0)
		return rc;

	msm_rpcrouter_xprt_notify(&smd_remote_xprt.xprt,
				  RPCROUTER_XPRT_EVENT_OPEN);

	smsm_change_state(SMSM_APPS_STATE, 0, SMSM_RPCINIT);

	return 0;
}

static struct platform_driver rpcrouter_smd_remote_driver = {
	.probe		= rpcrouter_smd_remote_probe,
	.driver		= {
			.name	= "RPCCALL",
			.owner	= THIS_MODULE,
	},
};

static int __init rpcrouter_smd_init(void)
{
#if defined(CONFIG_MSM_RPC_LOOPBACK_XPRT)
	int rc;
	rc = platform_driver_register(&rpcrouter_smd_loopback_driver);
	if (rc < 0)
		return rc;
#endif
	return platform_driver_register(&rpcrouter_smd_remote_driver);
}

module_init(rpcrouter_smd_init);
MODULE_DESCRIPTION("RPC Router SMD XPRT");
MODULE_LICENSE("Dual BSD/GPL");
