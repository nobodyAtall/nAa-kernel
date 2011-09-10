/* SEMC:modified */
/* 
   ES209RA board specific file.   
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

#ifndef _BOARD_ES209RA_H
#define _BOARD_ES209RA_H

#if defined(CONFIG_ARCH_QSD8X50)
#include <mach/irqs-8x50.h>
#include <mach/sirc.h>
#else
#include <mach/irqs-7xxx.h>
#endif

#define INT_ES209RA_GPIO_BATLOW			MSM_GPIO_TO_INT(145)
#define INT_ES209RA_GPIO_LEDC			MSM_GPIO_TO_INT(144)
#define INT_ES209RA_GPIO_UART2DM_RXD		MSM_GPIO_TO_INT(139)
#define INT_ES209RA_GPIO_PROXS			MSM_GPIO_TO_INT(108)
#define INT_ES209RA_GPIO_ETHER			MSM_GPIO_TO_INT(107)
#define INT_ES209RA_GPIO_VSYNC			MSM_GPIO_TO_INT(98)
#define INT_ES209RA_GPIO_ECOMPASS		MSM_GPIO_TO_INT(3)
#define INT_ES209RA_GPIO_ACCEL			MSM_GPIO_TO_INT(82)
#define INT_ES209RA_GPIO_OCV			MSM_GPIO_TO_INT(44)
#define INT_ES209RA_GPIO_TOUCHPAD		MSM_GPIO_TO_INT(37)
#define INT_ES209RA_GPIO_PM				MSM_GPIO_TO_INT(24)
#define INT_ES209RA_GPIO_CARD_INS_N		MSM_GPIO_TO_INT(23)
#define INT_ES209RA_GPIO_BT_HOSTWAKE	MSM_GPIO_TO_INT(21)
#define INT_ES209RA_GPIO_CAM_VSYNC		MSM_GPIO_TO_INT(14)
#define INT_ES209RA_GPIO_CAM_ISP		MSM_GPIO_TO_INT(1) 

/* SEMC: add bit definition for startup_reason - start */
/* PMIC startup reason */
#define STARTUP_REASON_POWER_KEY		(1 << 0)
#define STARTUP_REASON_RTC_ALARM		(1 << 1)
#define STARTUP_REASON_CABLE_POWER_ON	(1 << 2)
#define STARTUP_REASON_SMPL				(1 << 3)
#define STARTUP_REASON_WDOG_RESTART		(1 << 4)
#define STARTUP_REASON_USB_CHARGER		(1 << 5)
#define STARTUP_REASON_WALL_CHARGER		(1 << 6)
/* software startup reason */
#define STARTUP_REASON_FOTA_IU			(1 << 16)
#define STARTUP_REASON_FOTA_FI			(1 << 17)
#define STARTUP_REASON_MR				(1 << 18)
#define STARTUP_REASON_CMZ				(1 << 19)
/* software mode */
#define STARTUP_REASON_TYPE_APPROVAL	(1 << 28)
/* SEMC: add bit definition for startup_reason - end */

extern struct max17040_device_data max17040_dev_data;

#endif

