/*
 * Copyright (C) 2009 Sony Ericsson Mobile Communications AB.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _BOARD_DELTA_H
#define _BOARD_DELTA_H

/* PMIC startup reason */
#define STARTUP_REASON_POWER_KEY	(1 << 0)
#define STARTUP_REASON_RTC_ALARM	(1 << 1)
#define STARTUP_REASON_CABLE_POWER_ON	(1 << 2)
#define STARTUP_REASON_SMPL		(1 << 3)
#define STARTUP_REASON_WDOG_RESTART	(1 << 4)
#define STARTUP_REASON_USB_CHARGER	(1 << 5)
#define STARTUP_REASON_WALL_CHARGER	(1 << 6)

/* Software startup reason */
#define STARTUP_REASON_FOTA_IU		(1 << 16)
#define STARTUP_REASON_FOTA_FI		(1 << 17)
#define STARTUP_REASON_MR		(1 << 18)
#define STARTUP_REASON_CMZ		(1 << 19)

/* Software mode */
#define STARTUP_REASON_TYPE_APPROVAL	(1 << 28)

#endif

