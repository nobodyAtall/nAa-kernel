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

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#include <linux/msm_audio.h>

#include <mach/msm_qdsp6_audio.h>
#include <mach/debug_audio_mm.h>

struct dtmf {
	struct mutex lock;
	struct audio_client *ac;
	struct msm_dtmf_config cfg;
};

static long dtmf_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct dtmf *dtmf = file->private_data;
	int rc = 0;

	mutex_lock(&dtmf->lock);
	switch (cmd) {

	case AUDIO_START: {
		if (dtmf->ac) {
			rc = -EBUSY;
		} else {
			dtmf->ac = q6audio_open_dtmf(48000, 2, 0);
			if (!dtmf->ac)
				rc = -ENOMEM;
		}
		break;
	}
	case AUDIO_PLAY_DTMF: {
		rc = copy_from_user((void *)&dtmf->cfg, (void *)arg,
					sizeof(struct msm_dtmf_config));

		rc = q6audio_play_dtmf(dtmf->ac, dtmf->cfg.dtmf_hi,
					dtmf->cfg.dtmf_low, dtmf->cfg.duration,
					dtmf->cfg.rx_gain);
		if (rc) {
			MM_ERR("DTMF_START failed\n");
			break;
		}
		break;
	}
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&dtmf->lock);

	return rc;
}

static int dtmf_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	struct dtmf *dtmf;
	dtmf = kzalloc(sizeof(struct dtmf), GFP_KERNEL);

	if (!dtmf)
		return -ENOMEM;

	mutex_init(&dtmf->lock);

	file->private_data = dtmf;
	return rc;
}

static int dtmf_release(struct inode *inode, struct file *file)
{
	struct dtmf *dtmf = file->private_data;
	if (dtmf->ac)
		q6audio_close(dtmf->ac);
	kfree(dtmf);
	return 0;
}

static const struct file_operations dtmf_fops = {
	.owner		= THIS_MODULE,
	.open		= dtmf_open,
	.release	= dtmf_release,
	.unlocked_ioctl	= dtmf_ioctl,
};

struct miscdevice dtmf_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_dtmf",
	.fops	= &dtmf_fops,
};

static int __init dtmf_init(void)
{
	return misc_register(&dtmf_misc);
}

device_initcall(dtmf_init);
