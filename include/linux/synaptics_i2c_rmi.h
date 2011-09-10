/*
 * include/linux/synaptics_i2c_rmi.h - platform data structure for f75375s sensor
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
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

#ifndef _LINUX_SYNAPTICS_I2C_RMI_H
#define _LINUX_SYNAPTICS_I2C_RMI_H

#define SYNAPTICS_I2C_RMI_NAME "synaptics-rmi-ts"

#define IOC_MAGIC 't'

#define IOCTL_VALSET _IOW(IOC_MAGIC, 1, struct synaptics_i2c_rmi_ioctl_clbr)
#define IOCTL_VALGET _IOR(IOC_MAGIC, 2, struct synaptics_i2c_rmi_ioctl_clbr)
#define IOCTL_BIST   _IOR(IOC_MAGIC, 3, struct synaptics_i2c_rmi_ioctl_bist)

/*
 * The following defines prefixed with "SYNA_" are provided by Synaptics (2009-10-01)
 * and should be used by any RMI based fw driver to extract register and function values.
 *
 * Start of Page Description Table (PDT)
 *
 *      Register Name                                      Address     Register Description
 *      -------------                                      -------     --------------------
 */
#define SYNA_PDT_P00_F11_2D_QUERY_BASE                     0x00D7   /* Query Base */
#define SYNA_PDT_P00_F11_2D_COMMAND_BASE                   0x00D8   /* Command Base */
#define SYNA_PDT_P00_F11_2D_CONTROL_BASE                   0x00D9   /* Control Base */
#define SYNA_PDT_P00_F11_2D_DATA_BASE                      0x00DA   /* Data Base */
#define SYNA_PDT_P00_F11_2D_INTERRUPTS                     0x00DB   /* Interrupt Source Count */
#define SYNA_PDT_P00_F11_2D_EXISTS                         0x00DC   /* Function Exists */
#define SYNA_PDT_P00_F08_BIST_QUERY_BASE                   0x00DD   /* Query Base */
#define SYNA_PDT_P00_F08_BIST_COMMAND_BASE                 0x00DE   /* Command Base */
#define SYNA_PDT_P00_F08_BIST_CONTROL_BASE                 0x00DF   /* Control Base */
#define SYNA_PDT_P00_F08_BIST_DATA_BASE                    0x00E0   /* Data Base */
#define SYNA_PDT_P00_F08_BIST_INTERRUPTS                   0x00E1   /* Interrupt Source Count */
#define SYNA_PDT_P00_F08_BIST_EXISTS                       0x00E2   /* Function Exists */
#define SYNA_PDT_P00_F01_RMI_QUERY_BASE                    0x00E3   /* Query Base */
#define SYNA_PDT_P00_F01_RMI_COMMAND_BASE                  0x00E4   /* Command Base */
#define SYNA_PDT_P00_F01_RMI_CONTROL_BASE                  0x00E5   /* Control Base */
#define SYNA_PDT_P00_F01_RMI_DATA_BASE                     0x00E6   /* Data Base */
#define SYNA_PDT_P00_F01_RMI_INTERRUPTS                    0x00E7   /* Interrupt Source Count */
#define SYNA_PDT_P00_F01_RMI_EXISTS                        0x00E8   /* Function Exists */
#define SYNA_PDT_P00_F34_FLASH_QUERY_BASE                  0x00E9   /* Query Base */
#define SYNA_PDT_P00_F34_FLASH_COMMAND_BASE                0x00EA   /* Command Base */
#define SYNA_PDT_P00_F34_FLASH_CONTROL_BASE                0x00EB   /* Control Base */
#define SYNA_PDT_P00_F34_FLASH_DATA_BASE                   0x00EC   /* Data Base */
#define SYNA_PDT_P00_F34_FLASH_INTERRUPTS                  0x00ED   /* Interrupt Source Count */
#define SYNA_PDT_P00_F34_FLASH_EXISTS                      0x00EE   /* Function Exists */
#define SYNA_P00_PDT_PROPERTIES                            0x00EF   /* P00_PDT Properties */
#define SYNA_P00_PAGESELECT                                0x00FF   /* Page Select register */

/*
 * Masks for interrupt sources
 *
 *      Symbol Name                                        Mask        Description
 *      -----------                                        ----        -----------
 */
#define SYNA_F01_RMI_INT_SOURCE_MASK_ALL                   0x0002   /* Mask of all Func $01 (RMI) interrupts */
#define SYNA_F01_RMI_INT_SOURCE_MASK_STATUS                0x0002   /* Mask of Func $01 (RMI) 'STATUS' interrupt */
#define SYNA_F08_BIST_INT_SOURCE_MASK_ALL                  0x0004   /* Mask of all Func $08 (BIST) interrupts */
#define SYNA_F08_BIST_INT_SOURCE_MASK_BIST                 0x0004   /* Mask of Func $08 (BIST) 'BIST' interrupt */
#define SYNA_F11_2D_INT_SOURCE_MASK_ABS0                   0x0008   /* Mask of Func $11 (2D) 'ABS0' interrupt */
#define SYNA_F11_2D_INT_SOURCE_MASK_ALL                    0x0008   /* Mask of all Func $11 (2D) interrupts */
#define SYNA_F34_FLASH_INT_SOURCE_MASK_ALL                 0x0001   /* Mask of all Func $34 (FLASH) interrupts */
#define SYNA_F34_FLASH_INT_SOURCE_MASK_FLASH               0x0001   /* Mask of Func $34 (FLASH) 'FLASH' interrupt */


#define SYNA_RMI4_CONTINUOUS_REPORTING_MODE	0x00
#define SYNA_RMI4_REDUCED_REPORTING_MODE	0x01

enum {
	SYNAPTICS_FLIP_X = 1UL << 0,
	SYNAPTICS_FLIP_Y = 1UL << 1,
	SYNAPTICS_SWAP_XY = 1UL << 2,
	SYNAPTICS_SNAP_TO_INACTIVE_EDGE = 1UL << 3,
};

struct synaptics_i2c_rmi_platform_data {
	uint32_t version;	/* Use this entry for panels with */
				/* (major << 8 | minor) version or above. */
				/* If non-zero another array entry follows */
	int (*power)(int on);	/* Only valid in first array entry */
	uint32_t flags;
	unsigned long irqflags;
	uint32_t inactive_left; /* 0x10000 = screen width */
	uint32_t inactive_right; /* 0x10000 = screen width */
	uint32_t inactive_top; /* 0x10000 = screen height */
	uint32_t inactive_bottom; /* 0x10000 = screen height */
	uint32_t snap_left_on; /* 0x10000 = screen width */
	uint32_t snap_left_off; /* 0x10000 = screen width */
	uint32_t snap_right_on; /* 0x10000 = screen width */
	uint32_t snap_right_off; /* 0x10000 = screen width */
	uint32_t snap_top_on; /* 0x10000 = screen height */
	uint32_t snap_top_off; /* 0x10000 = screen height */
	uint32_t snap_bottom_on; /* 0x10000 = screen height */
	uint32_t snap_bottom_off; /* 0x10000 = screen height */
	uint32_t fuzz_x; /* 0x10000 = screen width */
	uint32_t fuzz_y; /* 0x10000 = screen height */
	int fuzz_p;
	int fuzz_w;
	int8_t sensitivity_adjust;
};

struct synaptics_i2c_rmi_ioctl_clbr {
	uint8_t clbr_num;
	uint8_t resolution;
	uint8_t scanspeed;
	uint8_t prescaler;
	uint8_t debounce;
	uint8_t hysteresis;
	uint8_t noisethreshold;
	uint8_t fingerthreshold;
	uint8_t negnoisethreshold;
	uint8_t lowbaselinereset;
	uint8_t blupdatethreshold;
};

struct synaptics_i2c_rmi_ioctl_bist {
	int32_t failure;
	int32_t sensor_no;
	int32_t limit;
};

/******************************
 Debug functions utils
******************************/
#define SNAP_LVL_INFO		0
#define SNAP_LVL_CRITICAL	0
#define SNAP_LVL_ABNORMAL	1
#define SNAP_LVL_LOW		2
#define SNAP_LVL_HIGH		3
#define SNAP_LVL_CODE		4
#define SNAP_LVL_FULL		5

typedef enum {
	SNAP_LOAD	= 0x0001, /*  */
	SNAP_UNLO	= 0x0002, /*  */
	SNAP_TX		= 0x0004, /*  */
	SNAP_RX		= 0x0008, /*  */
	SNAP_ISR	= 0x0010, /*  */
	SNAP_CTRL	= 0x0020, /*  */
	SNAP_ETS	= 0x0040, /*  */
	SNAP_08		= 0x0080, /*  */
	SNAP_09		= 0x0100, /*  */
	SNAP_10		= 0x0200, /*  */
	SNAP_11		= 0x0400, /*  */
	SNAP_12		= 0x0800, /*  */
	SNAP_13		= 0x1000, /*  */
	SNAP_14		= 0x2000, /*  */
	SNAP_15		= 0x4000, /*  */
	SNAP_16		= 0x8000, /*  */
	SNAP_ANY	= 0xffff /* Everything */
} snap_d_mod_t;

/******************************
 SWITCH OFF/ON WITH DEBUG & ETS
******************************/
// #define __DEBUG__
// #define __ETS_SUPPORT__


#ifdef __KERNEL__
#define KASSERT(exp, msg) do { \
	if (unlikely(!(exp))) {   \
		printk msg;     \
		BUG();      \
	}        \
} while (0)
#endif /* __KERNEL__ */

#ifdef __DEBUG__

typedef struct
{
	uint32_t debug_level;
	snap_d_mod_t debug_type;
} debug_t;

static const debug_t core_params =
{
	SNAP_LVL_FULL, SNAP_ANY
};

#define SNAP_MOD { \
	S(SNAP_LOAD) \
	S(SNAP_UNLO) \
	S(SNAP_TX) \
	S(SNAP_RX) \
	S(SNAP_ISR) \
	S(SNAP_CTRL) \
	S(SNAP_ETS) \
	SE(SNAP_ANY) \
};
#define  S(x)   #x,
#define  SE(x)  #x

#define MOD_FUNC_NAME_LEVEL_DEBUG SNAP_LVL_LOW

static const char *snap_d_mod[] = SNAP_MOD

static __inline uint32_t snap_idx(snap_d_mod_t n) {
	uint32_t log = 0;

	if (n == SNAP_ANY)
		return ARRAY_SIZE(snap_d_mod) - 1;

	while (n > 1) {
		n >>= 1;
		++log;
	}
	return log;
}

#define SNAPD(level, type, ... ) { \
	do { \
		if ((core_params.debug_type & type) && (core_params.debug_level >= level)) { \
			if ( core_params.debug_level >= MOD_FUNC_NAME_LEVEL_DEBUG ) { \
				printk(KERN_DEBUG "[%s]%s()[%i] :", \
				snap_d_mod[snap_idx(type)], __FUNCTION__, __LINE__); \
			} \
			printk(__VA_ARGS__); \
		} \
	} while (0); \
}

#define SNAPDX() \
   printk(KERN_DEBUG "%s()[%i]\n", __FUNCTION__, __LINE__);

#else
#define SNAPD(level, type, ... ) \
   do { \
   } while (0);

#define SNAPDX()   \
   do { \
   } while (0);

#endif // __DEBUG__


#endif /* _LINUX_SNAP_I2C_RMI_H */
