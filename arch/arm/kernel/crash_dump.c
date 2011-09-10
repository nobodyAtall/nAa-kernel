/*	arch/arm/kernel/crash_dump.c
 *
 *	Memory preserving reboot related code.
 *
 *	Copyright (C) 2009 Sony Ericsson Mobile Communications Japan, Inc.
 *
 * 	This software is licensed under the terms of the GNU General Public
 * 	License version 2, as published by the Free Software Foundation, and
 * 	may be copied, distributed, and modified under those terms.
 *
 * 	This program is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 *
 *
 */

#include <linux/errno.h>
#include <linux/crash_dump.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#ifndef CONFIG_PM
#include <../mach-msm/pm.h>
#endif
unsigned long long elfcorehdr_addr;
/**
 * copy_oldmem_page - copy one page from "oldmem"
 * @pfn: page frame number to be copied
 * @buf: target memory address for the copy; this can be in kernel address
 *	space or user address space (see @userbuf)
 * @csize: number of bytes to copy
 * @offset: offset in bytes into the page (based on pfn) to begin the copy
 * @userbuf: if set, @buf is in user address space, use copy_to_user(),
 *	otherwise @buf is in kernel address space, use memcpy().
 *
 * Copy a page from "oldmem". For this page, there is no pte mapped
 * in the current kernel. We stitch up a pte, similar to kmap_atomic.
 */
ssize_t copy_oldmem_page(unsigned long pfn, char *buf,
			size_t csize, unsigned long offset, int userbuf)
{
	void  *vaddr;

	if (!csize)
		return 0;

	vaddr = ioremap(pfn << PAGE_SHIFT, PAGE_SIZE);

	if (userbuf) {
		if (copy_to_user(buf, (vaddr + offset), csize)) {
			iounmap(vaddr);
			return -EFAULT;
		}
	} else
		memcpy(buf, (vaddr + offset), csize);

	iounmap(vaddr);
	return csize;
}

/* Stub functions to allow for capture kernel build for MSM
 * when few configurations in the kernel are disabled.
 */
#ifdef CONFIG_ARCH_MSM
#ifndef CONFIG_PM
void __init msm_pm_set_platform_data(struct msm_pm_platform_data *data)
{
}

void arch_idle(void)
{
}
#endif

#ifndef CONFIG_FB_MSM_MDDI_TMD_NT35580
void nt35580_lcd_device_add(void)
{
}
#endif
#endif
