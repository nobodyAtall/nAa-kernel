#ifndef __LINUX_SPI_CYPRESS_TOUCH_H
#define __LINUX_SPI_CYPRESS_TOUCH_H

#include <linux/ioctl.h>

struct cypress_touch_platform_data {
	u8 gpio_irq_pin;
	u8 gpio_reset_pin;
	u16 x_min, x_max;
	u16 y_min, y_max;
};

struct cy8ctma300_touch_ioctl_clbr {
	u8 clbr_num;		// TODO: To check&fix. The this menber is temporary correspondence.
	u8 prescaler;
	u8 subconversion;
	u8 shift;
	u8 noisethreshold;
	u8 fingerthreshold;
	u8 ninttimeout;
	u8 initmovestep;
	u8 movestep;
};

struct cy8ctma300_touch_reg_read_req {
	__u8 reg;
	__u8 *buf;
	size_t len;
};

struct cy8ctma300_touch_reg_write_req {
	__u8 reg;
	__u8 *buf;
	size_t len;
};

#define IOC_MAGIC 't'

#define IOCTL_VALSET _IOW(IOC_MAGIC, 1, struct cy8ctma300_touch_ioctl_clbr)
#define IOCTL_VALGET _IOR(IOC_MAGIC, 2, struct cy8ctma300_touch_ioctl_clbr)
#define IOCTL_TOUCH_REG_READ _IOWR(IOC_MAGIC, 3, struct cy8ctma300_touch_reg_read_req)
#define IOCTL_TOUCH_REG_WRITE _IOW(IOC_MAGIC, 4, struct cy8ctma300_touch_reg_write_req)

#endif /* __LINUX_SPI_CYPRESS_TOUCH_H */
