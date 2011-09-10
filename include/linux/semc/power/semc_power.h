/* include/linux/semc/power/semc_power.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Imre Sunyi <imre.sunyi@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _SEMC_POWER_H_
#define _SEMC_POWER_H_

/* Paths for the power device */
#define SEMC_POWER_DEVICE_NAME  "semc_power"

/* IOCTL Magic */
#define SEMC_POWER_IOC_MAGIC  'G'
#define SEMC_POWER_IOC_MAXNR 5

/* Status events the user-space can get */
#define STATUS_CHARGER_CONNECTION_CHANGED 0x01
#define STATUS_USB_MAX_CURRENT_CHANGED    0x02
#define STATUS_CHARGING_CHANGED           0x04
#define STATUS_SAFETY_TIMER_FAULT         0x08
#define STATUS_BDATA_CHANGED              0x10

/* Charging parameters */
struct power_param {
	unsigned int max_battery_charge_voltage; /* Never charge more than this (mV) */
	unsigned int max_battery_charge_current; /* Never charge more than this (mA) */
	unsigned int charging_completed_current_level; /* Charging ends when current falls below this value (mA) */
	unsigned int battery_charge_voltage; /* Regulates the battery voltage (mV) */
	unsigned int battery_charge_current; /* Regulates the current to battery (mA) */
	unsigned int charger_charge_current; /* Regulates the current from charger (mA) */
};

enum battery_temp_status {
	BATTERY_TEMP_STATUS_NORMAL = 0,
	BATTERY_TEMP_STATUS_COLD,
	BATTERY_TEMP_STATUS_WARM,
	BATTERY_TEMP_STATUS_OVERHEAT
};

enum charger_strength {
	CHARGER_STRENGTH_UNKNOWN = 0,
	CHARGER_STRENGTH_GOOD,
	CHARGER_STRENGTH_WEAK
};

struct battery_status {
	enum battery_temp_status temp_status;
	unsigned int ovp; /* over voltage protection */
};

struct charger_status {
	enum charger_strength strength;
};


/* Set charging parameters */
#define SEMC_POWER_IOC_SET_PARAMS          _IOW(SEMC_POWER_IOC_MAGIC, 0, struct power_param *)
/* Set battery status */
#define SEMC_POWER_IOC_SET_BATT_STATUS     _IOW(SEMC_POWER_IOC_MAGIC, 1, struct battery_status *)
/* Set the safety timer in minutes. 0 -> timer is off */
#define SEMC_POWER_IOC_SET_SAFETY_TIMER    _IOW(SEMC_POWER_IOC_MAGIC, 2, unsigned int)
/* Set charger status */
#define SEMC_POWER_IOC_SET_CHG_STATUS      _IOW(SEMC_POWER_IOC_MAGIC, 3, struct charger_status *)
/* Get the status */
#define SEMC_POWER_IOC_GET_STATUS          _IOR(SEMC_POWER_IOC_MAGIC, 4, unsigned int *)
/* Get the maximum current usb is allowed to give */
#define SEMC_POWER_IOC_GET_USB_MAX_CURRENT _IOR(SEMC_POWER_IOC_MAGIC, 5, unsigned int *)

#ifdef __KERNEL__

#include <linux/types.h>

enum event {
	SEMC_POWER_USB_CONNECTED = 0,
	SEMC_POWER_USB_DISCONNECTED,
	SEMC_POWER_USB_CURRENT_AVAILABLE,
	SEMC_POWER_AC_CONNECTED,
	SEMC_POWER_AC_DISCONNECTED,
	SEMC_POWER_CHARGING_READY,
	SEMC_POWER_CHARGING,
	SEMC_POWER_CHARGING_DONE,
	SEMC_POWER_ERROR_CHARGER_OVERVOLTAGE,
	SEMC_POWER_ERROR_SAFETY_TIMER,
	SEMC_POWER_ERROR_DCOUT,
	SEMC_POWER_ERROR_OTHER,
	SEMC_POWER_ERROR_NONE,
	SEMC_POWER_VBATT_CUTOFF,
	SEMC_POWER_BDATA_UPDATE,
	SEMC_POWER_CPU_ON,
	SEMC_POWER_CPU_OFF
};

enum unit {
	UNIT_MV = 0,
	UNIT_OHM,
};

enum battery_technology {
	BATTERY_TECHNOLOGY_UNKNOWN = 0,
	BATTERY_TECHNOLOGY_TYPE1,
	BATTERY_TECHNOLOGY_TYPE2
};

typedef void (*event_callback_t) (enum event event, void *data);
typedef void (*dcio_over_current_callback_t) (u8 set);

struct semc_power_platform_data {
	int (*gpio_init)(void);
};

struct battery_data {
	enum battery_technology technology;
	u32 cap_percent;
	s8 temp_celsius;
};

struct semc_power_dcout_currents {
	u8 nbr_currents_supported; /* Number of currents supported */
	u16 *currents_supported;   /* Array hold 'nbr_currents_supported' currents in
				    * increasing order. Currents in mA.
				    */
};

struct semc_notify_platform {
	u8 charging; /* 0 -> no, 1 -> yes */
	u8 battery_full;
	u8 power_collapse;
	u16 battery_charge_current;
	u16 charger_charge_current;
};
struct semc_power_boot_charging_info {
	u8 state;
	u16 charge_current;
};

struct power_ops {
	int (*turn_on_charger)(u8 usb_compatible);
	int (*turn_off_charger)(void);

	int (*enable_charger)(void);
	int (*disable_charger)(void);

	int (*enable_dcout)(u16 dcout_current_ma);
	int (*disable_dcout)(void);
	int (*get_dcout_currents)(struct semc_power_dcout_currents *currents);

	int (*set_safety_timer)(u16 timer_minutes);
	int (*set_ovp)(u8 onoff);

	int (*set_charger_voltage)(u16 voltage_mv);
	int (*set_charger_current)(u16 current_ma);
	int (*set_input_charger_current)(u16 current_ma);
	int (*set_charger_current_termination)(u16 current_ma);
	int (*set_charger_maximum_voltage)(u16 voltage_ma);
	int (*set_charger_maximum_current)(u16 current_ma);

	int (*get_battery_voltage)(u16 *voltage_mv);
	int (*get_battery_current)(s16 *current_ma);
	int (*get_battery_thermistor)(enum battery_technology type,
				      enum unit unit, u32 *val);

	void (*sync_hw)(void);
	void (*sync_platform)(void);
	void (*notify_platform)(struct semc_notify_platform *notify);
	int  (*platform_callbacks)(u8 enable);
	void (*get_boot_charging_info)(
		struct semc_power_boot_charging_info *charging_info);
	void (*notify_platform_boot_charging_info)(
		struct semc_power_boot_charging_info *charging_info);
};

/* Get the currents supported on DCOUT
 * Returns 0 on success and -errno on failure.
 */
extern int semc_power_get_dcout_currents(
		struct semc_power_dcout_currents *currents);

/* Enable/Disable the DCOUT current.
 * Enable -> enable = 1. Disable -> enable = 0.
 * Variable current_ma limits the DCOUT current in mA. It has no effect
 * when enable = 0.
 * Returns 0 on success and -errno on failure.
 */
extern int semc_power_dcio(u8 enable, u16 current_ma);

/* Register to get callback when DCIO current limit has been exceeded.
 * Returns 0 on success and -errno on failure.
 */
extern int semc_power_register_dcio_over_current_callback(
		dcio_over_current_callback_t callback);

/* Unregister the callback when DCIO current limit has been exceeded.
 * Returns 0 on success and -errno on failure.
 */
extern int semc_power_unregister_dcio_over_current_callback(void);

#endif  /* __KERNEL__ */

#endif /* _SEMC_POWER_H_ */
