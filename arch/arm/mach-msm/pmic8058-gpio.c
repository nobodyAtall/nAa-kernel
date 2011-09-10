/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
 * Qualcomm PMIC8058 GPIO driver
 *
 */

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include "gpio_chip.h"

#define PM8058_GPIO_TO_INT(n) (PMIC8058_IRQ_BASE + (n))

static int pm8058_gpio_configure(struct gpio_chip *chip,
				 unsigned int gpio,
				 unsigned long flags)
{
	int	rc = 0, direction;
	struct pm8058_chip	*pm_chip;

	gpio -= chip->start;

	if (flags & (GPIOF_INPUT | GPIOF_DRIVE_OUTPUT)) {
		direction = 0;
		if (flags & GPIOF_INPUT)
			direction |= PM_GPIO_DIR_IN;
		if (flags & GPIOF_DRIVE_OUTPUT)
			direction |= PM_GPIO_DIR_OUT;

		pm_chip = dev_get_drvdata(chip->dev);

		if (flags & (GPIOF_OUTPUT_LOW | GPIOF_OUTPUT_HIGH)) {
			if (flags & GPIOF_OUTPUT_HIGH)
				rc = pm8058_gpio_set(pm_chip, gpio, 1);
			else
				rc = pm8058_gpio_set(pm_chip, gpio, 0);

			if (rc) {
				pr_err("%s: FAIL pm8058_gpio_set(): rc=%d.\n",
					__func__, rc);
				goto bail_out;
			}
		}

		rc = pm8058_gpio_set_direction(pm_chip, gpio, direction);
		if (rc)
			pr_err("%s: FAIL pm8058_gpio_config(): rc=%d.\n",
				__func__, rc);
	}

bail_out:
	return rc;
}

static int pm8058_gpio_get_irq_num(struct gpio_chip *chip,
				   unsigned int gpio,
				   unsigned int *irqp,
				   unsigned long *irqnumflagsp)
{
	gpio -= chip->start;
	*irqp = PM8058_GPIO_TO_INT(gpio);
	if (irqnumflagsp)
		*irqnumflagsp = 0;
	return 0;
}

static int pm8058_gpio_read(struct gpio_chip *chip, unsigned n)
{
	struct pm8058_chip	*pm_chip;

	n -= chip->start;
	pm_chip = dev_get_drvdata(chip->dev);
	return pm8058_gpio_get(pm_chip, n);
}

static int pm8058_gpio_write(struct gpio_chip *chip, unsigned n, unsigned on)
{
	struct pm8058_chip	*pm_chip;

	n -= chip->start;
	pm_chip = dev_get_drvdata(chip->dev);
	return pm8058_gpio_set(pm_chip, n, on);
}

struct msm_gpio_chip pm8058_gpio_chip = {
	.chip = {
		.start = NR_GPIO_IRQS,
		.end = NR_GPIO_IRQS + NR_PMIC8058_GPIO_IRQS - 1,
		.configure = pm8058_gpio_configure,
		.get_irq_num = pm8058_gpio_get_irq_num,
		.read = pm8058_gpio_read,
		.write = pm8058_gpio_write,
	}
};

static int __devinit pm8058_gpio_probe(struct platform_device *pdev)
{
	int	rc;

	pm8058_gpio_chip.chip.dev = &pdev->dev;
	rc = register_gpio_chip(&pm8058_gpio_chip.chip);
	pr_info("%s: register_gpio_chip(): rc=%d\n", __func__, rc);

	return rc;
}

static int __devexit pm8058_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver pm8058_gpio_driver = {
	.probe		= pm8058_gpio_probe,
	.remove		= __devexit_p(pm8058_gpio_remove),
	.driver		= {
		.name = "pm8058-gpio",
		.owner = THIS_MODULE,
	},
};

static int __init pm8058_gpio_init(void)
{
	return platform_driver_register(&pm8058_gpio_driver);
}

static void __exit pm8058_gpio_exit(void)
{
	platform_driver_unregister(&pm8058_gpio_driver);
}

subsys_initcall(pm8058_gpio_init);
module_exit(pm8058_gpio_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("PMIC8058 GPIO driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pm8058-gpio");
