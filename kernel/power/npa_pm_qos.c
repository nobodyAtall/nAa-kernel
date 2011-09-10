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

#include <linux/npa.h>
#include <linux/npa_remote.h>
#include <linux/pm_qos_params.h>
#include <linux/completion.h>
#include <linux/err.h>

struct npa_client_info {
	char *resource_name;
	char *request_name;
	s32 value;
	struct npa_client **npa_client;
};

static void npa_res_available_cb(void *u_data, unsigned t, void *d, unsigned n)
{
	struct npa_client_info *client_info = (struct npa_client_info *)u_data;
	struct npa_client *npa_client;

	/* Create NPA 'required' client. */
	npa_client = npa_create_sync_client(client_info->resource_name,
		client_info->request_name, NPA_CLIENT_REQUIRED);
	if (IS_ERR(npa_client)) {
		pr_crit("npa_pm_qos: Failed to create NPA client '%s' "
			"for resource '%s'. (Error %ld)\n",
			client_info->request_name, client_info->resource_name,
			PTR_ERR(npa_client));
		BUG();
	}
	*(client_info->npa_client) = npa_client;

	/* Issue default resource requirement. */
	npa_issue_required_request(npa_client, client_info->value);

	kfree(client_info);
	return;
}

int npa_pm_qos_add(struct pm_qos_object *class, char *request_name,
			s32 value, void **request_data)
{
	struct npa_client_info *client_info;

	/* Non-default NPA requirements are not allowed at boot since
	 * requirements for unavailable resources won't be honoured. */
	BUG_ON(value != class->default_value
		&& system_state == SYSTEM_BOOTING);

	client_info = kzalloc(sizeof(struct npa_client_info), GFP_KERNEL);
	if (!client_info)
		return -ENOMEM;
	client_info->resource_name = (char *)class->plugin->data;
	client_info->request_name = request_name;
	client_info->value = value;
	client_info->npa_client = (struct npa_client **)request_data;

	/* Create NPA client when resource is available. */
	npa_resource_available(client_info->resource_name,
		npa_res_available_cb, client_info);

	return 0;
}

int npa_pm_qos_update(struct pm_qos_object *class, char *request_name,
				s32 value, void **request_data)
{
	struct npa_client *npa_client = (struct npa_client *)(*request_data);
	int rc = 0;

	if (!npa_client) {
		pr_err("%s: Error: No NPA client for resource '%s'.\n",
			__func__, (char *)class->plugin->data);
		return -ENXIO;
	}

	if (value == class->default_value)
		npa_complete_request(npa_client);
	else
		rc = npa_issue_required_request(npa_client, value);

	return rc;
}

int npa_pm_qos_remove(struct pm_qos_object *class, char *request_name,
				s32 value, void **request_data)
{
	struct npa_client *npa_client = (struct npa_client *)(*request_data);

	npa_cancel_request(npa_client);
	npa_destroy_client(npa_client);

	return 0;
}

