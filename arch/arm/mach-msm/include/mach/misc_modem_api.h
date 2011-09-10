/*
   Copyright (C) 2009 Sony Ericsson Mobile Communications AB.
   Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>

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
#ifndef _MISC_MODEM_API_H
#define _MISC_MODEM_API_H

#include <mach/pmic.h>
enum adc_logical_channel_t {
	APP_ADC_CHANNEL_PROXIMITY,
	APP_ADC_CHANNEL_SYSCON_PHF,
	APP_ADC_CHANNEL_LIGHTSENSOR,
	APP_ADC_CHANNEL_SYSCON_AID,
	APP_ADC_CHANNEL_SYSCON_PHF_HIRES,
	APP_ADC_CHANNEL_SYSCON_AID_HIRES,
	APP_ADC_CHANNEL_POWER_VBATT,
	APP_ADC_CHANNEL_POWER_BATT_ID,
	APP_ADC_CHANNELS_NUMBER
};

int msm_adc_read(enum adc_logical_channel_t, u16 *return_value);

/*
*  1 - enable, 0 - disable
*/
int pmic_boost_control(int enable);

/*
*  if current_mamp > 0
*	- flash MPP is connected to DBUS
*	- Boost voltage is enabled
*	- Led current is set to current_mamp
*  if current_mamp == 0
*	- flash MPP is connected to MANUAL
*	- Boost voltage is disabled
*	- Led current is set to 0
*/
int pmic_enable_camera_flash(u16 current_mamp);
#endif /* _MISC_MODEM_API_H */
