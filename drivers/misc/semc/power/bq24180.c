/* drivers/misc/semc/power/bq24180.c
 *
 * Power supply driver for BQ24180 chip
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

/****************** INCLUDE FILES SECTION *************************************/

#include <asm/gpio.h>
#include <linux/autoconf.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/pm_wakeup.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/semc/power/semc_power.h>
#include "bq24180.h"

/****************** CONSTANT AND MACRO SECTION ********************************/

/* #define STUB_I2C */
/* #define PRINT_REGISTERS */
/* #define USE_HW_SAFETY_TIMER */

#define SEMC_POWER "Semc Power Driver"

#ifdef PRINT_REGISTERS
  #define D_REGS(x) x
#else
  #define D_REGS(x)
#endif

#define BQ24180_NAME "bq24180"
#define BQ24180_WATCHDOG_TIMER_MS 10000
#define BQ24180_INIT_TIMEOUT_MS 5000

#define STATUS_CTRL_REG_RESET_WATCHDOG_BIT    7
#define STATUS_CTRL_REG_ENABLE_STAT_BIT       6
#define STATUS_CTRL_REG_STAT_MASK          0x30
#define STATUS_CTRL_REG_CHARGE_IN_PROGRESS 0x10
#define STATUS_CTRL_REG_CHARGE_DONE        0x20
#define STATUS_CTRL_REG_FAULT              0x30
#define STATUS_CTRL_REG_FAULT_MASK         0x07
#define STATUS_CTRL_REG_NO_BATTERY         0x07
#define STATUS_CTRL_REG_TIMER_FAULT        0x06
#define STATUS_CTRL_REG_THERMAL_SHUTDOWN   0x05
#define STATUS_CTRL_REG_DCOUT_CURRENT_LIMIT 0x04
#define STATUS_CTRL_REG_FAULTY_ADAPTER     0x03
#define STATUS_CTRL_REG_VBUS_OVP           0x01

#define CTRL_REG_DCOUT_LIMIT_MASK          0x30
#define CTRL_REG_DCOUT_LIMIT_370MA         0x00
#define CTRL_REG_DCOUT_LIMIT_750MA         0x10
#define CTRL_REG_DCOUT_LIMIT_1125MA        0x20
#define CTRL_REG_DCOUT_LIMIT_1500MA        0x30
#define CTRL_REG_DCOUT_DCOUT_ILIM_1_BIT       5
#define CTRL_REG_DCOUT_DCOUT_ILIM_2_BIT       4
#define CTRL_REG_CURRENT_TERMINATION_BIT      3
#define CTRL_REG_HZ_MODE_BIT                  1
#define CTRL_REG_CHARGER_ENABLE_DISABLE_BIT   2
#define CTRL_REG_DCOUT_ENABLE_DISABLE_BIT     0 /* 1=enabled 0=disabled */
#define CTRL_REG_INPUT_LIMIT_MASK          0xC0
#define CTRL_REG_INPUT_LIMIT_100MA         0x00
#define CTRL_REG_INPUT_LIMIT_500MA         0x40
#define CTRL_REG_INPUT_LIMIT_800MA         0x80
#define CTRL_REG_INPUT_LIMIT_NO            0xC0

#define CTRL_BATTERY_REGULATION_VOLTAGE_MASK   0xFC
#define CTRL_BATTERY_REGULATION_VOLTAGE_3500MV 0x00
#define CTRL_BATTERY_REGULATION_VOLTAGE_4000MV 0x64 /* 01100100-->3500+320+160+20 */
#define CTRL_BATTERY_REGULATION_VOLTAGE_4100MV 0x78 /* 01111000-->3500+320+160+80+40 */
#define CTRL_BATTERY_REGULATION_VOLTAGE_4200MV 0x8C /* 10001100-->3500+640+40+20 */

#define REVISION_MASK 0x07

#define BATTERY_FAST_CHARGE_CURRENT_MASK   0x78
#define BATTERY_FAST_CHARGE_CURRENT_550MA  0x00 /* 00000000 -->550            -->550 */
#define BATTERY_FAST_CHARGE_CURRENT_650MA  0x08 /* 00001000 -->550+100        -->650 */
#define BATTERY_FAST_CHARGE_CURRENT_750MA  0x10 /* 00010000 -->550+200        -->750 */
#define BATTERY_FAST_CHARGE_CURRENT_850MA  0x18 /* 00011000 -->550+100+200    -->850 */
#define BATTERY_FAST_CHARGE_CURRENT_950MA  0x20 /* 00100000 -->550+400        -->950 */
#define BATTERY_FAST_CHARGE_CURRENT_1050MA 0x28 /* 00101000 -->550+100+400    -->1050 */
#define BATTERY_FAST_CHARGE_CURRENT_1150MA 0x30 /* 00110000 -->550+200+400    -->1150 */
#define BATTERY_FAST_CHARGE_CURRENT_1250MA 0x38 /* 00111000 -->550+100+200+400-->1250 */

#define BATTERY_FAST_CHARGE_CURRENT_1350MA 0x40 /* 01000000 -->550+800        -->1350 */
#define BATTERY_FAST_CHARGE_CURRENT_1450MA 0x48 /* 01001000 -->550+100+800    -->1450 */
#define BATTERY_FAST_CHARGE_CURRENT_1550MA 0x50 /* 01010000 -->550+200+800    -->1550 */

#define BATTERY_TERMINATION_CURRENT_MASK  0x07
#define BATTERY_TERMINATION_CURRENT_25MA  0x00
#define BATTERY_TERMINATION_CURRENT_50MA  0x01
#define BATTERY_TERMINATION_CURRENT_75MA  0x02
#define BATTERY_TERMINATION_CURRENT_100MA 0x03
#define BATTERY_TERMINATION_CURRENT_125MA 0x04
#define BATTERY_TERMINATION_CURRENT_150MA 0x05
#define BATTERY_TERMINATION_CURRENT_175MA 0x06
#define BATTERY_TERMINATION_CURRENT_200MA 0x07

#define SAFETY_LIMIT_REG_MAXIMUM_BATTERY_VOLTAGE_MASK 0x0F
#define SAFETY_LIMIT_REG_MAXIMUM_BATTERY_VOLTAGE_4200 0x00

#define SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_MASK   0xF0
#define SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_650MA  0x10  /* 00010000-->550+100 */
#define SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_750MA  0x20  /* 00100000-->550+200 */
#define SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_850MA  0x30  /* 00110000-->550+200+100 */
#define SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_950MA  0x40  /* 01000000-->550+400 */
#define SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1050MA 0x50  /* 01010000-->550+400+100 */
#define SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1150MA 0x60  /* 01100000-->550+400+200 */
#define SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1250MA 0x70  /* 01110000-->550+400+200+100 */
#define SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1350MA 0x80  /* 10000000-->550+800 */
#define SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1450MA 0x90  /* 10010000-->550+800+100 */
#define SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1550MA 0xA0  /* 10100000-->550+800+200 */

#define BATTERY_TERMINATION_FAST_CHARGE_CURRENT_RESET_BIT 7

#define VIN_DPM_VOLTAGE_SAFETY_TIMER_VINDPM_MASK            0x07
#define VIN_DPM_VOLTAGE_SAFETY_TIMER_VINDPM_4440MV          0x03
#define VIN_DPM_VOLTAGE_SAFETY_TIMER_VINDPM_4760MV          0x07
#define VIN_DPM_VOLTAGE_SAFETY_TIMER_REG_LOW_CHG_BIT_REV1_0 3
#define VIN_DPM_VOLTAGE_SAFETY_TIMER_REG_LOW_CHG_BIT_REV1_P 5

#define NTC_MONITOR_REGISTER_SAFETY_TIMER_MASK    0x60
#define NTC_MONITOR_REGISTER_SAFETY_TIMER_27MIN   0x00
#define NTC_MONITOR_REGISTER_SAFETY_TIMER_3H      0x20
#define NTC_MONITOR_REGISTER_SAFETY_TIMER_6H      0x40
#define NTC_MONITOR_REGISTER_SAFETY_TIMER_DISABLE 0x60

#define SET_BIT(bit, val, data) ((val << bit) | ((data) & ~(1 << bit)))
#define CHK_BIT(bit, data) (((data) & (1 << bit)) >> bit)
#define SET_MASK(mask, val, data) (((data) & ~(mask)) | (val))
#define CHK_MASK(mask, data) ((data) & (mask))

/****************** TYPE DEFINITION SECTION ***********************************/

enum bq24180_status {
	READY = 0,
	CHARGE_IN_PROGRESS,
	CHARGE_DONE,
	FAULT
};

enum bq24180_status_fault {
	NORMAL = 0,
	VBUS_OVP,
	SLEEP_MODE,
	FAULTY_ADAPTER,
	DCOUT_LIMIT,
	THERMAL_SHUTDOWN,
	TIMER_FAULT,
	NO_BATTERY
};

enum bq24180_input_curr_limit {
	CURR_LIMIT_100 = 0,
	CURR_LIMIT_500,
	CURR_LIMIT_800,
	CURR_LIMIT_NO
};

struct regs {
	u8 addr;
	u8 data;
};

struct status_control
{
	enum bq24180_status stat;
	enum bq24180_status_fault fault;
};

struct safety_timer {
	struct hrtimer sw_timer;
	struct work_struct work;
	u8 expired;
};

struct bq24180_data {
	struct i2c_client *client;
	struct status_control status;
	struct safety_timer safety_timer;

	struct regs status_ctrl_reg;
	struct regs ctrl_reg;
	struct regs ctrl_battery_voltage_reg;
	struct regs battery_termination_fast_charge_reg;
	struct regs voltage_safety_timer_reg;
	struct regs safety_limit_reg;
	struct regs ntc_monitor_reg;
};

/****************** LOCAL FUNCTION DECLARATION SECTION ************************/

static int __devinit bq24180_probe(struct i2c_client *client,
				   const struct i2c_device_id *id);
static int __devexit bq24180_remove(struct i2c_client *client);
static int bq24180_read_reg(const uint8_t regAddr, uint8_t *const regData);
static int bq24180_write_reg(const uint8_t regAddr, const uint8_t regData);
static void bq24180_update_all_registers(void);
static void bq24180_reset_watchdog(struct work_struct *work);
static void bq24180_start_watchdog_reset(void);
static void bq24180_stop_watchdog_reset(void);
static irqreturn_t bq24180_gpio_interrupt(int irq, void *data);
static void bq24180_handle_interrupt(struct work_struct *work);
static int bq24180_check_status(void);
static void bq24180_reset_charger(void);
static void bq24180_set_init_values(u8 usb_compatible);
static int bq24180_set_charger_current_termination(u16 current_ma);
static int bq24180_set_charger_safety_limit_voltage(u16 voltage_mv);
static int bq24180_set_charger_safety_limit_current(u16 current_ma);
static int bq24180_set_safety_timer(u16 timer_minutes);
static int bq24180_set_charger_voltage(u16 voltage_mv);
static int bq24180_set_charger_current(u16 current_ma);
static int bq24180_set_input_charger_current(u16 current_ma);
static int bq24180_disable_charger(void);

/****************** GLOBAL VARIABLE DECLARATION SECTION ***********************/

/****************** LOCAL VARIABLE DECLARATION SECTION ************************/

static u8 probe_done = 0;
static u8 remove_done = 0;
static u8 irq_wake_enabled = 0;
static struct wake_lock bq24180_wake_lock;
static DECLARE_DELAYED_WORK(interrupt_work, bq24180_handle_interrupt);
static DECLARE_DELAYED_WORK(watchdog_work, bq24180_reset_watchdog);
static DEFINE_MUTEX(bq24180_mutex_lock);

static event_callback_t event_fn = NULL;

static u8 watchdog_enabled = 0;
static u8 disabled_by_input_current = 0;
static u8 disabled_by_charger_current = 0;

static u8 low_chg_bit = 0x00;

#ifdef CONFIG_SEMC_POWER_BOOTLOADER_INITIATES_CHARGING
static u8 boot_initiated_charging = 0;
#endif

static struct bq24180_data data;

static struct i2c_device_id bq24180_idtable[] = {
	{ BQ24180_NAME, 0 },
	{ }
};

static u16 dcout_currents[] = {370, 750, 1125, 1500};

static struct semc_power_dcout_currents bq24180_dcout_currents = {
	.nbr_currents_supported = ARRAY_SIZE(dcout_currents),
	.currents_supported = dcout_currents,
};

MODULE_DEVICE_TABLE(i2c, bq24180_idtable);

static struct i2c_driver bq24180_driver = {
	.driver = {
		.name  = BQ24180_NAME,
		.owner = THIS_MODULE,
	},
	.probe   = bq24180_probe,
	.remove  = __devexit_p(bq24180_remove),
	.id_table = bq24180_idtable,
};

/****************** FUNCTION DEFINITION SECTION *******************************/

static int __devinit bq24180_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct regs vender = {0x03, 0x00};
	struct semc_power_platform_data *platform_data = NULL;
	int ret = 0;

	data.client = client;

	printk(KERN_INFO BQ24180_NAME ": Probing\n");

	/* Make sure we have at least i2c functionality on the bus */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		ret = -EIO;
		printk(KERN_ERR BQ24180_NAME ": No i2c functionality available\n");
		goto probe_exit;
	}

#ifdef CONFIG_SEMC_POWER_BOOTLOADER_INITIATES_CHARGING
	/* Kick the watchdog as soon as possible to not lose the boot setting */
	bq24180_start_watchdog_reset();

	ret = bq24180_check_status();
	if (ret)
		goto probe_exit;

	if (data.status.stat == CHARGE_IN_PROGRESS) {
		printk(KERN_INFO BQ24180_NAME ": BQ24180 initialized by boot SW\n");
		boot_initiated_charging = 1;
	} else {
		printk(KERN_INFO BQ24180_NAME ": BQ24180 not initialized by boot SW\n");
		bq24180_stop_watchdog_reset();
		boot_initiated_charging = 0;
	}
#endif

	platform_data = client->dev.platform_data;
	if (platform_data)
		ret = platform_data->gpio_init();

	/* Request GPIO */
	if (ret) {
		printk(KERN_ERR BQ24180_NAME ": GPIO error\n");
		goto probe_exit;
	}

	/* Reset the charger HW */
#ifdef CONFIG_SEMC_POWER_BOOTLOADER_INITIATES_CHARGING
	if (!boot_initiated_charging)
		bq24180_reset_charger();
#else
	bq24180_reset_charger();
#endif

	ret = bq24180_read_reg(vender.addr, &vender.data);
	if (ret) {
		printk(KERN_INFO BQ24180_NAME ": Failed read chip revision\n");
		goto probe_exit;
	}

	printk(KERN_INFO BQ24180_NAME ": Found HW BQ24180 revision 0x%.2x.\n",
	       vender.data & REVISION_MASK);

	if ((vender.data & REVISION_MASK) == 0x00)
		low_chg_bit = VIN_DPM_VOLTAGE_SAFETY_TIMER_REG_LOW_CHG_BIT_REV1_0;
	else
		low_chg_bit = VIN_DPM_VOLTAGE_SAFETY_TIMER_REG_LOW_CHG_BIT_REV1_P;

	bq24180_update_all_registers();

	bq24180_set_init_values(1);

	/* Request for interrupts (IRQ) */
	ret = request_irq(client->irq,
			  bq24180_gpio_interrupt,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "BQ24180 interrupt", (void *)&data);
	if (ret) {
		printk(KERN_ERR BQ24180_NAME ": Failed requesting IRQ\n");
		goto probe_exit;
	}

	/* Need to make sure interrupt wakes up the device when in sleep */
	if (device_can_wakeup(&client->dev)) {
		ret = enable_irq_wake(client->irq);
		if (ret)
			printk(KERN_CRIT "Failed to enable wakeup on IRQ request\n");
		else
			irq_wake_enabled = 1;
	}

probe_exit:
	probe_done = 1;
	return ret;
}

static int __devexit bq24180_remove(struct i2c_client *client)
{
	flush_scheduled_work();

	if (irq_wake_enabled) {
		(void)disable_irq_wake(client->irq);
		irq_wake_enabled = 0;
	}

	free_irq(client->irq, (void *)&data);

	remove_done = 1;

	return 0;
}

#ifdef PRINT_REGISTERS
static void bq24180_print_registers(void)
{
	u8 data0;
	u8 data1;
	u8 data2;
	u8 data4;
	u8 data5;
	u8 data6;
	u8 data7;

	(void)bq24180_read_reg(0, &data0);
	(void)bq24180_read_reg(1, &data1);
	(void)bq24180_read_reg(2, &data2);
	(void)bq24180_read_reg(4, &data4);
	(void)bq24180_read_reg(5, &data5);
	(void)bq24180_read_reg(6, &data6);
	(void)bq24180_read_reg(7, &data7);

	printk(KERN_INFO BQ24180_NAME ": Registers\n\t0: 0x%.2x, 1: 0x%.2x, 2: 0x%.2x, 4: 0x%.2x, 5: 0x%.2x, 6: 0x%.2x, 7: 0x%.2x\n", data0, data1, data2, data4, data5, data6, data7);
}
#endif /* PRINT_REGISTERS */

static irqreturn_t bq24180_gpio_interrupt(int irq, void *data)
{
	/* Delay the interrupt handling since STATx in register '0' is not
	 * always updated when receiving this.
	 */
	schedule_delayed_work(&interrupt_work, msecs_to_jiffies(300));

	disable_irq(irq);

	return IRQ_HANDLED;
}

static void bq24180_handle_interrupt(struct work_struct *work)
{
	mutex_lock(&bq24180_mutex_lock);

	if (!bq24180_check_status()) {
		if (data.status.stat == READY) {
			printk(KERN_ERR BQ24180_NAME ": Charger Ready\n");
			event_fn(SEMC_POWER_CHARGING_READY, NULL);
		} else if (data.status.stat == CHARGE_IN_PROGRESS) {
			printk(KERN_ERR BQ24180_NAME ": Charge in progress\n");
			event_fn(SEMC_POWER_CHARGING, NULL);
		} else if (data.status.stat == CHARGE_DONE) {
			printk(KERN_ERR BQ24180_NAME ": Charge done\n");
			event_fn(SEMC_POWER_CHARGING_DONE, NULL);
		} else {
			printk(KERN_ERR BQ24180_NAME
				": Charger Fault. Code 0x%x\n",
				data.status.fault);
		}

		/* STATx bits are showing the current state and the FAULTx bits
		 * shows what has happened. FAULTx bits are cleared by reading them.
		 */
		if (data.status.fault == NORMAL ||
		    (data.status.stat != FAULT && data.status.fault == VBUS_OVP)) {
			/* Do not tell "error none" if SW safety timer has expired */
			if (!data.safety_timer.expired)
				event_fn(SEMC_POWER_ERROR_NONE, NULL);
		} else if (data.status.fault == VBUS_OVP) {
			printk(KERN_INFO BQ24180_NAME ": Charger over voltage! Charging stopped!\n");
			event_fn(SEMC_POWER_ERROR_CHARGER_OVERVOLTAGE, NULL);
		} else if (data.status.fault == TIMER_FAULT) {
			printk(KERN_INFO BQ24180_NAME ": HW Safety timer expired. Charging stopped!\n");
			event_fn(SEMC_POWER_ERROR_SAFETY_TIMER, NULL);
		} else if (data.status.fault == DCOUT_LIMIT) {
			printk(KERN_INFO BQ24180_NAME ": DCOUT Current limit tripped\n");
			event_fn(SEMC_POWER_ERROR_DCOUT, NULL);
		} else
			event_fn(SEMC_POWER_ERROR_OTHER, NULL);
	}

	mutex_unlock(&bq24180_mutex_lock);

	/* Scheduled from a work means it was coming from an interrupt. Re-enable interrupt */
	if (work)
		enable_irq(data.client->irq);
}

static int bq24180_read_reg(const u8 regAddr, u8 *const regData)
{
	struct i2c_msg msg[2];
	int ret = 0;
	int i2c_ret = 0;
	struct i2c_client *client = data.client;

	if (!client || !regData) {
		ret = -EINVAL;
		goto read_exit;
	}

	/* First send the address we want to read from */
	msg[0].addr  = client->addr;
	msg[0].flags = 0;
	msg[0].len   = 1;
	msg[0].buf   = (u8 *)&regAddr;

	/* Now read the data from the address */
	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = 1;
	msg[1].buf   = regData;

#ifndef STUB_I2C
	i2c_ret = i2c_transfer(client->adapter, msg, 2);
#endif
	if (i2c_ret < 0) {
		printk(KERN_ERR SEMC_POWER ": Error reading register 0x%x\n", regAddr);
		ret = i2c_ret;
		goto read_exit;
	}

read_exit:
	return ret;
}

static int bq24180_write_reg(const u8 regAddr, const u8 regData)
{
	struct i2c_msg msg;
	uint8_t regs[2] = {regAddr, regData};
	int ret = 0;
	int i2c_ret = 0;
	struct i2c_client *client = data.client;

	if (!client) {
		ret = -EINVAL;
		goto write_exit;
	}

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 2;
	msg.buf   = regs;

#ifndef STUB_I2C
	i2c_ret = i2c_transfer(client->adapter, &msg, 1);
#endif
	if (i2c_ret < 0) {
		printk(KERN_ERR SEMC_POWER ": Error writing register 0x%x\n", regAddr);
		ret = i2c_ret;
	}

write_exit:
	return ret;
}

static void bq24180_reset_charger(void)
{
	struct regs *reg = &data.battery_termination_fast_charge_reg;

	/* At reset eventual safety timer expiration is resetted */
	mutex_lock(&bq24180_mutex_lock);
	data.safety_timer.expired = 0;
	mutex_unlock(&bq24180_mutex_lock);

	/* Never check if this bit is set to '1'. A read will always get '1' */
	reg->data = SET_BIT(BATTERY_TERMINATION_FAST_CHARGE_CURRENT_RESET_BIT, 1, reg->data);

	(void)bq24180_write_reg(reg->addr, reg->data);
}

static void bq24180_set_init_values(u8 usb_compatible)
{
	struct regs *reg;
	u8 vin_dpm;

	printk(KERN_INFO BQ24180_NAME ": Set init values\n");

#ifdef CONFIG_SEMC_POWER_BOOTLOADER_INITIATES_CHARGING
	if (!boot_initiated_charging) {
		(void)bq24180_set_charger_voltage(0);
		(void)bq24180_set_charger_current(0);
	}
#else
	(void)bq24180_set_charger_voltage(0);
	(void)bq24180_set_charger_current(0);
#endif

	/* Disable safety timers */
	(void)bq24180_set_safety_timer(0);

	/* Enable status interrupts */
	reg = &data.status_ctrl_reg;
	reg->data = SET_BIT(STATUS_CTRL_REG_ENABLE_STAT_BIT, 1, reg->data);
	(void)bq24180_write_reg(reg->addr, reg->data);

	if (usb_compatible) {
		/* Set the minimum charger input voltage to be compatible with USB voltage (5 V +- 5 %)*/
		vin_dpm = VIN_DPM_VOLTAGE_SAFETY_TIMER_VINDPM_4760MV;
	} else
		vin_dpm = VIN_DPM_VOLTAGE_SAFETY_TIMER_VINDPM_4440MV;

	reg = &data.voltage_safety_timer_reg;
	reg->data = SET_MASK(VIN_DPM_VOLTAGE_SAFETY_TIMER_VINDPM_MASK, vin_dpm, reg->data);

	(void)bq24180_write_reg(reg->addr, reg->data);
}

static void bq24180_update_all_registers(void)
{
	if (bq24180_read_reg(data.status_ctrl_reg.addr, &data.status_ctrl_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed updating reg 0x%x\n",
		       data.status_ctrl_reg.addr);

	if (bq24180_read_reg(data.ctrl_reg.addr, &data.ctrl_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed updating reg 0x%x\n",
		       data.ctrl_reg.addr);

	if (bq24180_read_reg(data.ctrl_battery_voltage_reg.addr,
			     &data.ctrl_battery_voltage_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed updating reg 0x%x\n",
		       data.ctrl_battery_voltage_reg.addr);

	if (bq24180_read_reg(data.battery_termination_fast_charge_reg.addr,
			     &data.battery_termination_fast_charge_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed updating reg 0x%x\n",
		       data.battery_termination_fast_charge_reg.addr);

	if (bq24180_read_reg(data.voltage_safety_timer_reg.addr, &data.voltage_safety_timer_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed updating reg 0x%x\n",
		       data.voltage_safety_timer_reg.addr);

	if (bq24180_read_reg(data.safety_limit_reg.addr, &data.safety_limit_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed updating reg 0x%x\n",
		       data.safety_limit_reg.addr);

	if (bq24180_read_reg(data.ntc_monitor_reg.addr, &data.ntc_monitor_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed updating reg 0x%x\n",
		       data.ntc_monitor_reg.addr);

	D_REGS(
		printk(KERN_INFO BQ24180_NAME ": Updating all registers\n");
		bq24180_print_registers();
		)
}

static void bq24180_restore_all_registers(void)
{
	/* Need to update the safetylimit register before any other.
	 * Not doing so will lock that register to default values.
	 */
	if (bq24180_write_reg(data.safety_limit_reg.addr, data.safety_limit_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed restore reg 0x%x\n",
		       data.safety_limit_reg.addr);

	if (bq24180_write_reg(data.status_ctrl_reg.addr, data.status_ctrl_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed restore reg 0x%x\n",
		       data.status_ctrl_reg.addr);

	if (bq24180_write_reg(data.ctrl_reg.addr, data.ctrl_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed restore reg 0x%x\n",
		       data.ctrl_reg.addr);

	if (bq24180_write_reg(data.ctrl_battery_voltage_reg.addr,
			      data.ctrl_battery_voltage_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed restore reg 0x%x\n",
		       data.ctrl_battery_voltage_reg.addr);

	/* Make sure reset bit is not set */
	data.battery_termination_fast_charge_reg.data =
		SET_BIT(BATTERY_TERMINATION_FAST_CHARGE_CURRENT_RESET_BIT, 0,
			data.battery_termination_fast_charge_reg.data);

	if (bq24180_write_reg(data.battery_termination_fast_charge_reg.addr,
			      data.battery_termination_fast_charge_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed restore reg 0x%x\n",
		       data.battery_termination_fast_charge_reg.addr);

	if (bq24180_write_reg(data.voltage_safety_timer_reg.addr, data.voltage_safety_timer_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed restore reg 0x%x\n",
		       data.voltage_safety_timer_reg.addr);

	if (bq24180_write_reg(data.ntc_monitor_reg.addr, data.ntc_monitor_reg.data))
		printk(KERN_ERR BQ24180_NAME ": Failed restore reg 0x%x\n",
		       data.ntc_monitor_reg.addr);

	D_REGS(
		printk(KERN_INFO BQ24180_NAME ": Restoring all registers\n");
		bq24180_print_registers();
		)
}

static void bq24180_start_watchdog_reset(void)
{
	if (!watchdog_enabled) {
		printk(KERN_INFO BQ24180_NAME ": Starting watchdog reset\n");

		wake_lock(&bq24180_wake_lock);

		watchdog_enabled = 1;
		bq24180_reset_watchdog(NULL);
	}
}

static void bq24180_stop_watchdog_reset(void)
{
	if (watchdog_enabled) {
		printk(KERN_INFO BQ24180_NAME ": Stopping watchdog reset\n");

		watchdog_enabled = 0;
		cancel_delayed_work(&watchdog_work);

		wake_unlock(&bq24180_wake_lock);
	}
}

static void bq24180_reset_watchdog(struct work_struct *work)
{
	int ret = 0;
	struct regs *reg = &data.status_ctrl_reg;

	ret = bq24180_read_reg(reg->addr, &reg->data);
	if (!ret) {
		/* Never check if this bit is set to '1'.
		 * A read on this bit reports other functionality.
		 */
		reg->data = SET_BIT(STATUS_CTRL_REG_RESET_WATCHDOG_BIT, 1, reg->data);
		ret = bq24180_write_reg(reg->addr, reg->data);
	}

	if (ret)
		printk(KERN_ERR BQ24180_NAME ": Watchdog reset failed\n");

	D_REGS(bq24180_print_registers();)

	schedule_delayed_work(&watchdog_work, msecs_to_jiffies(BQ24180_WATCHDOG_TIMER_MS));
}

static enum hrtimer_restart bq24180_safety_timer_expired(struct hrtimer *timer)
{
	struct safety_timer *stimer = &data.safety_timer;

	schedule_work(&stimer->work);

	return HRTIMER_NORESTART;
}

static void bq24180_safety_timer_expired_work(struct work_struct *work)
{
	printk(KERN_INFO BQ24180_NAME ": SW Safety timer expired. Charging stopped!\n");

	mutex_lock(&bq24180_mutex_lock);
	data.safety_timer.expired = 1;
	mutex_unlock(&bq24180_mutex_lock);

	event_fn(SEMC_POWER_ERROR_SAFETY_TIMER, NULL);
	bq24180_disable_charger();
}

static int bq24180_check_status(void)
{
	struct regs *reg = &data.status_ctrl_reg;
	int ret = bq24180_read_reg(reg->addr, &reg->data);

	if (!ret)
	{
		u8 stat_data = reg->data & STATUS_CTRL_REG_STAT_MASK;
		u8 fault_data = reg->data & STATUS_CTRL_REG_FAULT_MASK;

		if (stat_data == STATUS_CTRL_REG_CHARGE_IN_PROGRESS)
			data.status.stat = CHARGE_IN_PROGRESS;
		else if (stat_data == STATUS_CTRL_REG_CHARGE_DONE)
			data.status.stat = CHARGE_DONE;
		else if (stat_data == STATUS_CTRL_REG_FAULT)
			data.status.stat = FAULT;
		else
			data.status.stat = READY;

		if (fault_data == STATUS_CTRL_REG_DCOUT_CURRENT_LIMIT)
			data.status.fault = DCOUT_LIMIT;
		else if (fault_data == STATUS_CTRL_REG_NO_BATTERY)
			data.status.fault = NO_BATTERY;
		else if (fault_data == STATUS_CTRL_REG_TIMER_FAULT) {
			struct regs *ntc_reg = &data.ntc_monitor_reg;

			if (CHK_MASK(NTC_MONITOR_REGISTER_SAFETY_TIMER_MASK, ntc_reg->data) !=
			    NTC_MONITOR_REGISTER_SAFETY_TIMER_DISABLE)
				data.status.fault = TIMER_FAULT;
			else {
				printk(KERN_INFO BQ24180_NAME ": Strange HW behaviour. Got safety timer fault but HW timer is disabled.\n");
				data.status.fault = NORMAL;
			}
		} else if (fault_data == STATUS_CTRL_REG_THERMAL_SHUTDOWN)
			data.status.fault = THERMAL_SHUTDOWN;
		else if (fault_data == STATUS_CTRL_REG_FAULTY_ADAPTER)
			data.status.fault = FAULTY_ADAPTER;
		else if (fault_data == STATUS_CTRL_REG_VBUS_OVP)
			data.status.fault = VBUS_OVP;
		else
			data.status.fault = NORMAL;
	}

	return ret;
}

static int bq24180_set_input_current_limit(enum bq24180_input_curr_limit limit)
{
	struct regs *reg = &data.ctrl_reg;
	u8 limit_data;
	int ret = 0;
	char *msg_ptr = NULL;

	switch (limit) {
	case CURR_LIMIT_100:
		msg_ptr = "100 mA";
		limit_data = CTRL_REG_INPUT_LIMIT_100MA;
		break;
	case CURR_LIMIT_500:
		msg_ptr = "500 mA";
		limit_data = CTRL_REG_INPUT_LIMIT_500MA;
		break;
	case CURR_LIMIT_800:
		msg_ptr = "800 mA";
		limit_data = CTRL_REG_INPUT_LIMIT_800MA;
		break;
	case CURR_LIMIT_NO:
		/* fall through */
	default:
		msg_ptr = "no limit";
		limit_data = CTRL_REG_INPUT_LIMIT_NO;
		break;
	}

	if (CHK_MASK(CTRL_REG_INPUT_LIMIT_MASK, reg->data) != limit_data) {
		printk(KERN_INFO BQ24180_NAME ": Input current limit set to %s\n", msg_ptr);
		reg->data = SET_MASK(CTRL_REG_INPUT_LIMIT_MASK, limit_data, reg->data);

		ret = bq24180_write_reg(reg->addr, reg->data);
	}

	return ret;
}

static int bq24180_set_350ma_charge_current_limit(unsigned int set)
{
	struct regs *reg = &data.voltage_safety_timer_reg;
	u8 bit_changed = 0;
	int ret = 0;

	if (set && CHK_BIT(low_chg_bit, reg->data) != 1) {
		bit_changed = 1;
		reg->data = SET_BIT(low_chg_bit, 1, reg->data);
	} else if (!set && CHK_BIT(low_chg_bit, reg->data) != 0) {
		bit_changed = 1;
		reg->data = SET_BIT(low_chg_bit, 0, reg->data);
	}


	if (bit_changed) {
		printk(KERN_INFO BQ24180_NAME ": %s 350 mA charge current limit\n",
		       set ? "Set" : "Unset");
		ret = bq24180_write_reg(reg->addr, reg->data);
	}

	return ret;
}

static int bq24180_turn_on_charger(u8 usb_compatible)
{
	int ret = 0;
	struct regs *reg = &data.ctrl_reg;

	printk(KERN_INFO BQ24180_NAME ": Turning on charger. USB-%s mode\n",
	       usb_compatible ? "Host" : "Dedicated");

	bq24180_update_all_registers();
	bq24180_set_init_values(usb_compatible);

	/* No high impedance mode */
	reg->data = SET_BIT(CTRL_REG_HZ_MODE_BIT, 0, reg->data);
	ret = bq24180_write_reg(reg->addr, reg->data);

	/* Need to start watchdog reset otherwise HW will reset itself */
	bq24180_start_watchdog_reset();

	return ret;
}

static int bq24180_turn_off_charger(void)
{
	int ret = 0;
	struct regs *reg = &data.ctrl_reg;

	printk(KERN_INFO BQ24180_NAME ": Turning off charger\n");

	bq24180_stop_watchdog_reset();

	/* High impedance mode */
	reg->data = SET_BIT(CTRL_REG_HZ_MODE_BIT, 1, reg->data);
	ret = bq24180_write_reg(reg->addr, reg->data);

	return ret;
}

static int bq24180_enable_charger(void)
{
	int ret = 0;
	struct regs *reg = &data.ctrl_reg;

	mutex_lock(&bq24180_mutex_lock);
	if (data.safety_timer.expired)
		ret = -EPERM;
	mutex_unlock(&bq24180_mutex_lock);

	if (ret) {
		printk(KERN_INFO BQ24180_NAME ": Safety timer expired. Not allowed to enable charger.\n");
		return ret;
	}

	if (CHK_BIT(CTRL_REG_CHARGER_ENABLE_DISABLE_BIT, reg->data) != 0) {
		printk(KERN_INFO BQ24180_NAME ": Enable charger\n");

		reg->data = SET_BIT(CTRL_REG_CHARGER_ENABLE_DISABLE_BIT, 0, reg->data);

		ret = bq24180_write_reg(reg->addr, reg->data);

		/* This is needed to undo the force of an event in
		 * bq24180_disable_charger().
		 */
		mutex_lock(&bq24180_mutex_lock);
		if (data.status.stat == CHARGE_DONE) {
			event_fn(SEMC_POWER_CHARGING_DONE, NULL);
		}
		mutex_unlock(&bq24180_mutex_lock);
	}

	return ret;
}

static int bq24180_disable_charger(void)
{
	int ret = 0;
	struct regs *reg = &data.ctrl_reg;

	if (CHK_BIT(CTRL_REG_CHARGER_ENABLE_DISABLE_BIT, reg->data) != 1) {
		printk(KERN_INFO BQ24180_NAME ": Disable charger\n");

		reg->data = SET_BIT(CTRL_REG_CHARGER_ENABLE_DISABLE_BIT, 1, reg->data);

		ret = bq24180_write_reg(reg->addr, reg->data);

		/* When disabling charger and the charging state is in
		 * "Charge done" the asic is not toggling the STAT pin that we
		 * monitor by IRQ. That is work-as-design by asic but we need
		 * to show 'Discharging' and force an event to event handler.
		 */
		mutex_lock(&bq24180_mutex_lock);
		if (data.status.stat == CHARGE_DONE) {
			event_fn(SEMC_POWER_CHARGING_READY, NULL);
		}
		mutex_unlock(&bq24180_mutex_lock);
	}

	return ret;
}

static int bq24180_disable_dcout(void)
{
	int ret = 0;
	struct regs *reg = &data.ctrl_reg;

	if (CHK_BIT(CTRL_REG_DCOUT_ENABLE_DISABLE_BIT, reg->data) != 0) {
		printk(KERN_INFO BQ24180_NAME ": Disable DCOUT\n");

		ret = bq24180_read_reg(reg->addr, &reg->data);
		if (ret < 0)
			goto disable_dcout_exit;

		reg->data = SET_BIT(CTRL_REG_DCOUT_ENABLE_DISABLE_BIT, 0, reg->data);
		ret = bq24180_write_reg(reg->addr, reg->data);
	}

disable_dcout_exit:
	return ret;
}

static int bq24180_enable_dcout(u16 dcout_current_ma)
{
	int ret = 0;

	printk(KERN_INFO BQ24180_NAME ": Enable DCOUT with %u mA limit\n", dcout_current_ma);

	if (dcout_current_ma < 370) {
		ret = bq24180_disable_dcout();
	} else {
		struct regs *reg = &data.ctrl_reg;
		u8 dcout_data;

		if (dcout_current_ma < 750) {
			/* 370 mA */
			dcout_data = CTRL_REG_DCOUT_LIMIT_370MA;
		} else if (dcout_current_ma < 1125) {
			/* 750 mA */
			dcout_data = CTRL_REG_DCOUT_LIMIT_750MA;
		} else if (dcout_current_ma < 1500) {
			/* 1125 mA */
			dcout_data = CTRL_REG_DCOUT_LIMIT_1125MA;
		} else {
			/* 1500 mA */
			dcout_data = CTRL_REG_DCOUT_LIMIT_1500MA;
		}

		reg->data = SET_MASK(CTRL_REG_DCOUT_LIMIT_MASK, dcout_data, reg->data);

		/* Enable DCOUT */
		reg->data = SET_BIT(CTRL_REG_DCOUT_ENABLE_DISABLE_BIT, 1, reg->data);

		ret = bq24180_write_reg(reg->addr, reg->data);
	}

	return ret;
}

static int bq24180_get_dcout_currents(struct semc_power_dcout_currents *currents)
{
	int ret = 0;

	if (probe_done)
		*currents = bq24180_dcout_currents;
	else
		ret = -EBUSY;

	return ret;
}

static int bq24180_set_safety_timer(u16 timer_minutes)
{
	struct regs *reg = &data.ntc_monitor_reg;
	struct safety_timer *timer = &data.safety_timer;
	u8 timer_data = NTC_MONITOR_REGISTER_SAFETY_TIMER_DISABLE;
	u8 using_hw_timer = 0;
	int ret = 0;

#ifdef USE_HW_SAFETY_TIMER
	if (27 == timer_minutes) {
		using_hw_timer = 1;
		timer_data = NTC_MONITOR_REGISTER_SAFETY_TIMER_27MIN;
	} else if (180 == timer_minutes) {
		using_hw_timer = 1;
		timer_data = NTC_MONITOR_REGISTER_SAFETY_TIMER_3H;
	} else if (360 == timer_minutes) {
		using_hw_timer = 1;
		timer_data = NTC_MONITOR_REGISTER_SAFETY_TIMER_6H;
	}
#endif /* USE_HW_SAFETY_TIMER */

	printk(KERN_INFO BQ24180_NAME ": Set safety timer to %u minutes\n", timer_minutes);

	/* Stop any SW timer */
	if (hrtimer_active(&timer->sw_timer))
		hrtimer_cancel(&timer->sw_timer);

	/* Set HW timer if necessary
	 * Changing the HW safety timer duration resets the safety timer
	 */
	if (CHK_MASK(NTC_MONITOR_REGISTER_SAFETY_TIMER_MASK, reg->data) != timer_data) {
		reg->data = SET_MASK(NTC_MONITOR_REGISTER_SAFETY_TIMER_MASK,
				     timer_data,
				     reg->data);

		ret = bq24180_write_reg(reg->addr, reg->data);
	}

	/* Set SW timer if necessary */
	if (!using_hw_timer && timer_minutes) {
		ret = hrtimer_start(&timer->sw_timer,
				    ktime_set(timer_minutes * 60, 0),
				    HRTIMER_MODE_REL);
	}

	return ret;
}

static int bq24180_set_charger_voltage(u16 voltage_mv)
{
	int ret = 0;
	struct regs *reg = &data.ctrl_battery_voltage_reg;
	u8 voltage_data;

	if (voltage_mv >= 4200)
		voltage_data = CTRL_BATTERY_REGULATION_VOLTAGE_4200MV;
	else if (voltage_mv >= 4100)
		voltage_data = CTRL_BATTERY_REGULATION_VOLTAGE_4100MV;
	else if (voltage_mv >= 4000)
		voltage_data = CTRL_BATTERY_REGULATION_VOLTAGE_4000MV;
	else
		voltage_data = CTRL_BATTERY_REGULATION_VOLTAGE_3500MV;

	if (CHK_MASK(CTRL_BATTERY_REGULATION_VOLTAGE_MASK, reg->data) != voltage_data) {
		printk(KERN_INFO BQ24180_NAME ": Setting charger voltage to %u mV\n", voltage_mv);
		reg->data = SET_MASK(CTRL_BATTERY_REGULATION_VOLTAGE_MASK, voltage_data, reg->data);

		ret = bq24180_write_reg(reg->addr, reg->data);
	}

	return ret;
}

static int bq24180_set_charger_current(u16 current_ma)
{
	int ret = 0;

	printk(KERN_INFO BQ24180_NAME ": Setting charger current to %u mA\n", current_ma);

	if (current_ma < 350) {
		ret = bq24180_disable_charger();
		disabled_by_charger_current = 1;
	} else {
		if (current_ma < 550) {
			ret = bq24180_set_350ma_charge_current_limit(1);
		} else {
			struct regs *reg = &data.battery_termination_fast_charge_reg;
			u8 current_data;

			ret = bq24180_set_350ma_charge_current_limit(0);

			if (!ret) {
				if (current_ma < 650)
					current_data = BATTERY_FAST_CHARGE_CURRENT_550MA;
				else if (current_ma < 750)
					current_data = BATTERY_FAST_CHARGE_CURRENT_650MA;
				else if (current_ma < 850)
					current_data = BATTERY_FAST_CHARGE_CURRENT_750MA;
				else if (current_ma < 950)
					current_data = BATTERY_FAST_CHARGE_CURRENT_850MA;
				else if (current_ma < 1050)
					current_data = BATTERY_FAST_CHARGE_CURRENT_950MA;
				else if (current_ma < 1150)
					current_data = BATTERY_FAST_CHARGE_CURRENT_1050MA;
				else if (current_ma < 1250)
					current_data = BATTERY_FAST_CHARGE_CURRENT_1150MA;
				else if (current_ma < 1350)
					current_data = BATTERY_FAST_CHARGE_CURRENT_1250MA;
				else if (current_ma < 1450)
					current_data = BATTERY_FAST_CHARGE_CURRENT_1350MA;
				else if (current_ma < 1550)
					current_data = BATTERY_FAST_CHARGE_CURRENT_1450MA;
				else
					current_data = BATTERY_FAST_CHARGE_CURRENT_1550MA;

				if (CHK_MASK(BATTERY_FAST_CHARGE_CURRENT_MASK, reg->data) != current_data) {
					reg->data = SET_MASK(BATTERY_FAST_CHARGE_CURRENT_MASK, current_data, reg->data);

					ret = bq24180_write_reg(reg->addr, reg->data);
				}
			}
		}

		if (!ret) {
			disabled_by_charger_current = 0;

			if (!disabled_by_input_current)
				ret = bq24180_enable_charger();
		}
	}

	return ret;
}

static int bq24180_set_input_charger_current(u16 current_ma)
{
	int ret = 0;

	printk(KERN_INFO BQ24180_NAME ": Setting input charger current to %u mA\n", current_ma);

	if (current_ma < 100) {
		disabled_by_input_current = 1;
		ret = bq24180_disable_charger();
	}
	else {
		enum bq24180_input_curr_limit limit;

		if (current_ma < 500)
			limit = CURR_LIMIT_100;
		else if (current_ma < 800)
			limit = CURR_LIMIT_500;
		else if (current_ma > 800)
			limit = CURR_LIMIT_NO;
		else
			limit = CURR_LIMIT_800;

		ret = bq24180_set_input_current_limit(limit);

		if (!ret) {
			disabled_by_input_current = 0;

			if (!disabled_by_charger_current)
				ret = bq24180_enable_charger();
		}
	}

	return ret;
}

static int bq24180_set_charger_current_termination(u16 current_ma)
{
	int ret = 0;
	struct regs *reg;

	printk(KERN_INFO BQ24180_NAME ": Set charge current termination to %u mA\n", current_ma);

	if (current_ma < 25) {
		/* Disable current termination */
		reg = &data.ctrl_reg;

		if (CHK_BIT(CTRL_REG_CURRENT_TERMINATION_BIT, reg->data) != 0) {
			reg->data = SET_BIT(CTRL_REG_CURRENT_TERMINATION_BIT, 0, reg->data);
			ret = bq24180_write_reg(reg->addr, reg->data);
		}
	} else {
		u8 current_data;

		/* Enable current termination */
		reg = &data.ctrl_reg;
		if (CHK_BIT(CTRL_REG_CURRENT_TERMINATION_BIT, reg->data) != 1) {
			reg->data = SET_BIT(CTRL_REG_CURRENT_TERMINATION_BIT, 1, reg->data);
			(void)bq24180_write_reg(reg->addr, reg->data);
		}

		reg = &data.battery_termination_fast_charge_reg;

		if (current_ma < 50)
			current_data = BATTERY_TERMINATION_CURRENT_25MA;
		else if (current_ma < 75)
			current_data = BATTERY_TERMINATION_CURRENT_50MA;
		else if (current_ma < 100)
			current_data = BATTERY_TERMINATION_CURRENT_75MA;
		else if (current_ma < 125)
			current_data = BATTERY_TERMINATION_CURRENT_100MA;
		else if (current_ma < 150)
			current_data = BATTERY_TERMINATION_CURRENT_125MA;
		else if (current_ma < 175)
			current_data = BATTERY_TERMINATION_CURRENT_150MA;
		else if (current_ma < 200)
			current_data = BATTERY_TERMINATION_CURRENT_175MA;
		else
			current_data = BATTERY_TERMINATION_CURRENT_200MA;

		if (CHK_MASK(BATTERY_TERMINATION_CURRENT_MASK, reg->data) != current_data) {
			reg->data = SET_MASK(BATTERY_TERMINATION_CURRENT_MASK, current_data, reg->data);

			ret = bq24180_write_reg(reg->addr, reg->data);
		}
	}

	return ret;
}

static int bq24180_set_charger_safety_limit_voltage(u16 voltage_mv)
{
	struct regs *reg;

	printk(KERN_INFO BQ24180_NAME ": Set charger safety limit voltage to %u mV\n", voltage_mv);

	/* Charger needs to be resetted before safety limit register gets valid */
#ifdef CONFIG_SEMC_POWER_BOOTLOADER_INITIATES_CHARGING
	if (boot_initiated_charging) {
		boot_initiated_charging = 0;
	}
#endif

	/* But first store the registers before reset */
	bq24180_update_all_registers();

	/* Reset to default settings */
	bq24180_reset_charger();

	/* Update the safety register */
	reg = &data.safety_limit_reg;

	reg->data = SET_MASK(SAFETY_LIMIT_REG_MAXIMUM_BATTERY_VOLTAGE_MASK,
			     SAFETY_LIMIT_REG_MAXIMUM_BATTERY_VOLTAGE_4200, reg->data);

	if (voltage_mv < 4200)
		printk(KERN_ERR BQ24180_NAME ": Safety limit voltage under 4.2 V not supported by HW!\n");
	else if (voltage_mv > 4200)
		printk(KERN_ERR BQ24180_NAME ": Safety limit voltage above 4.2 V not implemented!\n");

	/* Restore the registers before reset and with new safety voltage limit */
	bq24180_restore_all_registers();

	return 0;
}

static int bq24180_set_charger_safety_limit_current(u16 current_ma)
{
	struct regs *reg;
	u8 current_data = 0;

	printk(KERN_INFO BQ24180_NAME ": Set charger safety limit current to %u mA\n", current_ma);

	/* Charger needs to be resetted before safety limit register gets valid */
#ifdef CONFIG_SEMC_POWER_BOOTLOADER_INITIATES_CHARGING
	if (boot_initiated_charging) {
		boot_initiated_charging = 0;
	}
#endif

	/* But first store the registers before reset */
	bq24180_update_all_registers();

	/* Reset to default settings */
	bq24180_reset_charger();

	/* Update the safety register */
	reg = &data.safety_limit_reg;

	if (current_ma < 650)
		current_data = 0x00;
	else if (current_ma < 750)
		current_ma = SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_650MA;
	else if (current_ma < 850)
		current_ma = SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_750MA;
	else if (current_ma < 950)
		current_data = SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_850MA;
	else if (current_ma < 1050)
		current_data =  SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_950MA;
	else if (current_ma < 1150)
		current_data = SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1050MA;
	else if (current_ma < 1250)
		current_data = SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1150MA;
	else if (current_ma < 1350)
		current_data = SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1250MA;
	else if (current_ma < 1450)
		current_data = SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1350MA;
	else if (current_ma < 1550)
		current_data = SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1450MA;
	else
		current_data = SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_1550MA;

	reg->data = SET_MASK(SAFETY_LIMIT_REG_MAXIMUM_CHARGE_CURRENT_MASK,
			     current_data, reg->data);

	/* Restore the registers before reset and with new safety current limit */
	bq24180_restore_all_registers();

	return 0;
}

static u16 bq24180_get_boot_input_current_limit(void)
{
	u8 reg_data;
	u16 input_current;
	struct regs *reg = &data.ctrl_reg;

	(void)bq24180_read_reg(reg->addr, &reg->data);
	reg_data = CHK_MASK(CTRL_REG_INPUT_LIMIT_MASK, reg->data);

	switch (reg_data) {
	case CTRL_REG_INPUT_LIMIT_100MA:
		input_current = 100;
		break;
	case CTRL_REG_INPUT_LIMIT_500MA:
		input_current = 500;
		break;
	case CTRL_REG_INPUT_LIMIT_800MA:
		input_current = 800;
		break;
	case CTRL_REG_INPUT_LIMIT_NO:
		input_current = USHORT_MAX;
		break;
	default:
		input_current = 0;
		break;
	}

	return input_current;

}

static u16 bq24180_get_boot_charger_current(void)
{
	u8 reg_data;
	u16 current_ma;
	struct regs *reg = &data.battery_termination_fast_charge_reg;


	(void)bq24180_read_reg(reg->addr, &reg->data);
	reg_data = CHK_MASK(BATTERY_FAST_CHARGE_CURRENT_MASK, reg->data);

	switch (reg_data) {
	case BATTERY_FAST_CHARGE_CURRENT_550MA:
		current_ma = 550;
		break;
	case BATTERY_FAST_CHARGE_CURRENT_650MA:
		current_ma = 650;
		break;
	case BATTERY_FAST_CHARGE_CURRENT_750MA:
		current_ma = 750;
		break;
	case BATTERY_FAST_CHARGE_CURRENT_850MA:
		current_ma = 850;
		break;
	case BATTERY_FAST_CHARGE_CURRENT_950MA:
		current_ma = 950;
		break;
	case BATTERY_FAST_CHARGE_CURRENT_1050MA:
		current_ma = 1050;
		break;
	case BATTERY_FAST_CHARGE_CURRENT_1150MA:
		current_ma = 1150;
		break;
	case BATTERY_FAST_CHARGE_CURRENT_1250MA:
		current_ma = 1250;
		break;
	case BATTERY_FAST_CHARGE_CURRENT_1350MA:
		current_ma = 1350;
		break;
	case BATTERY_FAST_CHARGE_CURRENT_1450MA:
		current_ma = 1450;
		break;
	case BATTERY_FAST_CHARGE_CURRENT_1550MA:
		current_ma = 1550;
		break;
	default:
		current_ma = 0;
		break;

	}

	return current_ma;


}

static void bq24180_get_boot_charging_info(struct semc_power_boot_charging_info *charging_info)
{
	if (charging_info)
#ifdef CONFIG_SEMC_POWER_BOOTLOADER_INITIATES_CHARGING
		charging_info->state = boot_initiated_charging;
#else
		charging_info->state = 0;
#endif
	if (charging_info->state) {
		charging_info->charge_current = min(
				bq24180_get_boot_input_current_limit(),
				bq24180_get_boot_charger_current());
        }
}

static void bq24180_sync_hw(void)
{
	bq24180_handle_interrupt(NULL);
}

int setup_hw(struct power_ops *ops, event_callback_t fn)
{
	int ret = 0;

	printk(KERN_INFO BQ24180_NAME ": Setting up HW\n");

	if (ops == NULL) {
		ret = -EINVAL;
		goto setup_exit;
	}

	event_fn = fn;

	probe_done = 0;
	remove_done = 0;

	memset(&data, 0, sizeof(data));

	data.status_ctrl_reg.addr                     = 0x00;
	data.ctrl_reg.addr                            = 0x01;
	data.ctrl_battery_voltage_reg.addr            = 0x02;
	data.battery_termination_fast_charge_reg.addr = 0x04;
	data.voltage_safety_timer_reg.addr            = 0x05;
	data.safety_limit_reg.addr                    = 0x06;
	data.ntc_monitor_reg.addr                     = 0x07;

	ops->get_boot_charging_info          = bq24180_get_boot_charging_info;
	ops->turn_on_charger                 = bq24180_turn_on_charger;
	ops->turn_off_charger                = bq24180_turn_off_charger;
	ops->enable_charger                  = bq24180_enable_charger;
	ops->disable_charger                 = bq24180_disable_charger;
	ops->enable_dcout                    = bq24180_enable_dcout;
	ops->disable_dcout                   = bq24180_disable_dcout;
	ops->get_dcout_currents              = bq24180_get_dcout_currents;
	ops->set_safety_timer                = bq24180_set_safety_timer;
	ops->set_charger_voltage             = bq24180_set_charger_voltage;
	ops->set_charger_current             = bq24180_set_charger_current;
	ops->set_input_charger_current       = bq24180_set_input_charger_current;
	ops->set_charger_current_termination = bq24180_set_charger_current_termination;
	ops->set_charger_maximum_voltage     = bq24180_set_charger_safety_limit_voltage;
	ops->set_charger_maximum_current     = bq24180_set_charger_safety_limit_current;
	ops->sync_hw                         = bq24180_sync_hw;

	INIT_WORK(&data.safety_timer.work, bq24180_safety_timer_expired_work);

	hrtimer_init(&data.safety_timer.sw_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data.safety_timer.sw_timer.function = bq24180_safety_timer_expired;

	wake_lock_init(&bq24180_wake_lock, WAKE_LOCK_SUSPEND, "bq24180_watchdog_lock");

	ret = i2c_add_driver(&bq24180_driver);
	if (ret) {
		printk(KERN_ERR "BQ24180 I2C registration failed\n");
		goto setup_exit;
	}

	if (!probe_done) {
		DECLARE_WAIT_QUEUE_HEAD_ONSTACK(setup_wait);
		ret = wait_event_interruptible_timeout(setup_wait, probe_done,
						       msecs_to_jiffies(BQ24180_INIT_TIMEOUT_MS));
		/* Check if wait was timed out */
		if (!ret) {
			printk(KERN_ERR "BQ24180 I2C probing timed out\n");
			ret = -ETIME;
		}
	}

setup_exit:
	if (ret)
		wake_lock_destroy(&bq24180_wake_lock);

	return ret;
}

int teardown_hw(void)
{
	int ret = 0;

	printk(KERN_INFO BQ24180_NAME ": Closing down HW\n");

	bq24180_stop_watchdog_reset();

	if (work_pending(&data.safety_timer.work))
		cancel_work_sync(&data.safety_timer.work);

	if (delayed_work_pending(&interrupt_work))
		cancel_delayed_work_sync(&interrupt_work);

	if (delayed_work_pending(&watchdog_work))
		cancel_delayed_work_sync(&watchdog_work);

	i2c_del_driver(&bq24180_driver);

	if (probe_done && !remove_done) {
		DECLARE_WAIT_QUEUE_HEAD_ONSTACK(teardown_wait);
		ret = wait_event_interruptible_timeout(teardown_wait, remove_done,
						       msecs_to_jiffies(BQ24180_INIT_TIMEOUT_MS));
		/* Check if wait was timed out */
		if (!ret)
			ret = -ETIME;
	}

	return ret;
}
