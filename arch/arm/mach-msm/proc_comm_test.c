/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * PROC COMM TEST Driver source file
 */

#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include "proc_comm.h"

static struct dentry *dent;
static int proc_comm_test_res;

static int proc_comm_reverse_test(void)
{
	uint32_t data1, data2;
	int rc;

	data1 = 10;
	data2 = 20;

	rc = msm_proc_comm(PCOM_OEM_TEST_CMD, &data1, &data2);
	if (rc)
		return rc;

	if ((data1 != 20) || (data2 != 10))
		return -1;

	return 0;
}

static ssize_t debug_read(struct file *fp, char __user *buf,
			  size_t count, loff_t *pos)
{
	char _buf[16];

	snprintf(_buf, sizeof(_buf), "%i\n", proc_comm_test_res);

	return simple_read_from_buffer(buf, count, pos, _buf, strlen(_buf));
}

static ssize_t debug_write(struct file *fp, const char __user *buf,
			   size_t count, loff_t *pos)
{

	unsigned char cmd[64];
	int len;

	if (count < 1)
		return 0;

	len = count > 63 ? 63 : count;

	if (copy_from_user(cmd, buf, len))
		return -EFAULT;

	cmd[len] = 0;

	if (cmd[len-1] == '\n') {
		cmd[len-1] = 0;
		len--;
	}

	if (!strncmp(cmd, "reverse_test", 64))
		proc_comm_test_res = proc_comm_reverse_test();
	else
		proc_comm_test_res = -EINVAL;

	if (proc_comm_test_res)
		pr_err("proc comm test fail %d\n",
		       proc_comm_test_res);
	else
		pr_info("proc comm test passed\n");

	return count;
}

static int debug_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static int debug_open(struct inode *ip, struct file *fp)
{
	return 0;
}

static const struct file_operations debug_ops = {
	.owner = THIS_MODULE,
	.open = debug_open,
	.release = debug_release,
	.read = debug_read,
	.write = debug_write,
};

static void __exit proc_comm_test_mod_exit(void)
{
	debugfs_remove(dent);
}

static int __init proc_comm_test_mod_init(void)
{
	dent = debugfs_create_file("proc_comm", 0444, 0, NULL, &debug_ops);
	proc_comm_test_res = -1;
	return 0;
}

module_init(proc_comm_test_mod_init);
module_exit(proc_comm_test_mod_exit);

MODULE_DESCRIPTION("PROC COMM TEST Driver");
MODULE_LICENSE("Dual BSD/GPL");
