/* linux/spi/es209ra_touch.h */

#ifndef __LINUX_SPI_ES209RA_TOUCH_H
#define __LINUX_SPI_ES209RA_TOUCH_H

#include <linux/ioctl.h>

struct es209ra_touch_platform_data {
	__u8	gpio_irq_pin;
	__u8	gpio_reset_pin;
	__u16	x_min, x_max;
	__u16	y_min, y_max;
};

struct es209ra_touch_ioctl_clbr {
	__u8 clbr_num;
	__u8 resolution;
	__u8 scanspeed;
	__u8 prescaler;
	__u8 columnidac;
	__u8 rowidac;
	__u8 noisethreshold;
	__u8 fingerthreshold;
	__u8 negnoisethreshold;
	__u8 lowbaselinereset;
	__u8 blupdatethreshold;
};

/* function declaration */
extern int es209ra_touch_reg_read(struct spi_device *, u8, u8 *, size_t);
extern int es209ra_touch_reg_write(struct spi_device *, u8, u8 *, size_t);

#define IOC_MAGIC 't'

#define IOCTL_VALSET _IOW(IOC_MAGIC, 1, struct es209ra_touch_ioctl_clbr)
#define IOCTL_VALGET _IOR(IOC_MAGIC, 2, struct es209ra_touch_ioctl_clbr)

#endif /* __LINUX_SPI_ES209RA_TOUCH_H */
