/* SEMC:modified */
/* 
   Wlan power driver

   Copyright (C) 2009 Sony Ericsson Mobile Communications Japan, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License, version 2, as
   published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>
#include <linux/ctype.h>
#include <mach/vreg.h>

static int wlan_vreg_config (const char *name, int level, int on)
{
    struct vreg *vreg_pmic;
    int rc;

    vreg_pmic = vreg_get(0, name);
    if (!vreg_pmic) {
        printk(KERN_ERR "%s: vreg get failed\n", __func__);
        return -EIO;
    }

    if (on)
    {
        rc = vreg_set_level(vreg_pmic, level);
        if (rc)
        {
            printk(KERN_ERR "%s: %s vreg set level failed (%d)\n", __func__, name, rc);
            return -EIO;
        }
        rc = vreg_enable(vreg_pmic);
        if (rc)
        {
            printk(KERN_ERR "%s: %s vreg enable failed (%d)\n", __func__, name, rc);
            return -EIO;
        }
    }
    else
    {
        rc = vreg_disable(vreg_pmic);
        if (rc)
        {
            printk(KERN_ERR "%s: %s vreg disable failed (%d)\n", __func__, name, rc);
            return -EIO;
        }
    }
    return 0;
}

static int __init wlan_power_init(void)
{
    if (wlan_vreg_config("wlan", 2900, 1))
    pr_err("failed to set vreg_wlan\n");

    if (gpio_request(143, "wlan18_en"))
        pr_err("failed to request gpio wlan18_en\n");
#if (defined(CONFIG_ES209RA_DP0) || defined(CONFIG_ES209RA_DP1))
    if (gpio_request(87,  "wlan12_en"))
        pr_err("failed to request gpio wlan12_en\n");
#elif (defined(CONFIG_ES209RA_SP1) || defined(CONFIG_ES209RA_AP1))
    if (gpio_request(61,  "wlan12_en"))
        pr_err("failed to request gpio wlan12_en\n");
#else
    if (gpio_request(61,  "wlan12_en"))
        pr_err("failed to request gpio wlan12_en\n");
#endif

    gpio_direction_output(143, 0);  /* WLAN18_EN */
    gpio_set_value(143, 1);
#if (defined(CONFIG_ES209RA_DP0) || defined(CONFIG_ES209RA_DP1))
    gpio_direction_output(87, 0);  /* WLAN12_EN */
    gpio_set_value(87, 1);
#elif (defined(CONFIG_ES209RA_SP1) || defined(CONFIG_ES209RA_AP1))
    gpio_direction_output(61, 0);  /* WLAN12_EN */
    gpio_set_value(61, 1);
#else
    gpio_direction_output(61, 0);  /* WLAN12_EN */
    gpio_set_value(61, 1);
#endif
    mdelay(10);

    gpio_free(143);
#if (defined(CONFIG_ES209RA_DP0) || defined(CONFIG_ES209RA_DP1))
    gpio_free(87);
#elif (defined(CONFIG_ES209RA_SP1) || defined(CONFIG_ES209RA_AP1))
    gpio_free(61);
#else
    gpio_free(61);
#endif
    
    printk(KERN_INFO "wlan_power init\n");
    
    return 0;
}

static void __exit wlan_power_exit(void)
{
    if (gpio_request(143, "wlan18_en"))
        pr_err("failed to request gpio wlan18_en\n");
#if (defined(CONFIG_ES209RA_DP0) || defined(CONFIG_ES209RA_DP1))
    if (gpio_request(87,  "wlan12_en"))
        pr_err("failed to request gpio wlan12_en\n");
#elif (defined(CONFIG_ES209RA_SP1) || defined(CONFIG_ES209RA_AP1))
    if (gpio_request(61,  "wlan12_en"))
        pr_err("failed to request gpio wlan12_en\n");
#else
    if (gpio_request(61,  "wlan12_en"))
        pr_err("failed to request gpio wlan12_en\n");
#endif

#if (defined(CONFIG_ES209RA_DP0) || defined(CONFIG_ES209RA_DP1))
    gpio_direction_output(87, 0);  /* WLAN12_EN */
    gpio_set_value(87, 0);
#elif (defined(CONFIG_ES209RA_SP1) || defined(CONFIG_ES209RA_AP1))
    gpio_direction_output(61, 0);  /* WLAN12_EN */
    gpio_set_value(61, 0);
#else
    gpio_direction_output(61, 0);  /* WLAN12_EN */
    gpio_set_value(61, 0);
#endif
    mdelay(10);
    gpio_direction_output(143, 0);  /* WLAN18_EN */
    gpio_set_value(143, 0);

#if (defined(CONFIG_ES209RA_DP0) || defined(CONFIG_ES209RA_DP1))
    gpio_free(87);
#elif (defined(CONFIG_ES209RA_SP1) || defined(CONFIG_ES209RA_AP1))
    gpio_free(61);
#else
    gpio_free(61);
#endif
    gpio_free(143);
    
    if (wlan_vreg_config("wlan", 2900, 0))
    pr_err("failed to set vreg_wlan\n");

    printk(KERN_INFO "wlan_power exit\n");
}

MODULE_AUTHOR("SEMC");
MODULE_DESCRIPTION("WLAN Power driver");
MODULE_LICENSE("GPL");

module_init(wlan_power_init);
module_exit(wlan_power_exit);

