/*
   drivers/video/msm/mddi_nt35580_lcd.c   A control program for nt35580-LCD.
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
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/msm_rpcrouter.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include "mddi_tmd_nt35580.h"

#define SET_HORIZONTAL_ADDRESS_0	0x2A00
#define SET_HORIZONTAL_ADDRESS_1	0x2A01
#define SET_HORIZONTAL_ADDRESS_2	0x2A02
#define SET_HORIZONTAL_ADDRESS_3	0x2A03
#define SET_VERTICAL_ADDRESS_0		0x2B00
#define SET_VERTICAL_ADDRESS_1		0x2B01
#define SET_VERTICAL_ADDRESS_2		0x2B02
#define SET_VERTICAL_ADDRESS_3		0x2B03
#define SET_RAM_ADDRESS_0		0x2D00
#define SET_RAM_ADDRESS_1		0x2D01
#define SET_RAM_ADDRESS_2		0x2D02
#define SET_RAM_ADDRESS_3		0x2D03

#define DISP_CTRL2		0x0180
#define FSWCTRL			0x0380
#define SOUTORCTRL		0x0480
#define PORCHCTRL		0x0580
#define FRCTRL1			0x0680

#define CP_CLK			0x2080
#define VR_ADJ			0x2280
#define GVDD_ADJ		0x2480
#define VMH_ADJ			0x2580
#define VCMH_ADJ		0x2780
#define AVDD2S_AVDD2C_CTRL	0x2A80
#define AVDD2S_AVDD2C_ADJ	0x2B80
#define AVDD2S_AVDD2C_CP_CLK	0x2C80
#define VCMM			0x2D80

#define MUX_START		0xD080
#define MUX_WIDTH		0xD180
#define MUX_GAP  		0xD280
#define GSP_START		0xD380
#define GSP_WIDTH		0xD480
#define GCK_START		0xD580
#define GCK_WIDTH		0xD680
#define CMI_START		0xD780
#define LTPS_ENP_CGOE1		0xD880
#define LTPS_ENP_CGOE2		0xD980
#define LTPS_ENP_CGOP1		0xDA80
#define LTPS_ENP_CGOP2		0xDB80
#define TEMPORARY_PAR		0xDD80

#define EXIT_SLEEP_MODE		0x1100
#define SET_DISPLAY_ON		0x2900

#define SET_DISPLAY_OFF		0x2800
#define ENT_SLEEP_MODE		0x1000
#define ENTER_DSTB_MODE		0x4f00
#define ENTER_INVERT_MODE	0x2100
#define SET_ADDRESS_MODE	0x3600
#define SET_HIGH_SPEED_MODE	0x3601
#define SET_PIXEL_FORMAT	0x3A00
#define ANA_SW_CTR		0x6E4F

#define TEST_MODE_CTR_0		0xF280
#define TEST_MODE_CTR_1		0xF281
#define TEST_MODE_CTR_2		0xF282
#define PRE_CHG_CTR		0xF38E

#define SET_TEAR_ON		0x3500
#define SET_TEAR_SCANLINE_1	0x4400
#define SET_TEAR_SCANLINE_2	0x4401

#define GMACTRL_01		0x4080
#define GMACTRL_02		0x4180
#define GMACTRL_03		0x4280
#define GMACTRL_04		0x4380
#define GMACTRL_05		0x4480
#define GMACTRL_06		0x4580
#define GMACTRL_07		0x4680
#define GMACTRL_08		0x4780
#define GMACTRL_09		0x4880
#define GMACTRL_10		0x4980
#define GMACTRL_11		0x4A80
#define GMACTRL_12		0x4B80
#define GMACTRL_13		0x4C80
#define GMACTRL_14		0x4D80
#define GMACTRL_15		0x4E80
#define GMACTRL_16		0x4F80
#define GMACTRL_17		0x5080
#define GMACTRL_18		0x5180
#define GMACTRL_19		0x5880
#define GMACTRL_20		0x5980
#define GMACTRL_21		0x5A80
#define GMACTRL_22		0x5B80
#define GMACTRL_23		0x5C80
#define GMACTRL_24		0x5D80
#define GMACTRL_25		0x5E80
#define GMACTRL_26		0x5F80
#define GMACTRL_27		0x6080
#define GMACTRL_28		0x6180
#define GMACTRL_29		0x6280
#define GMACTRL_30		0x6380
#define GMACTRL_31		0x6480
#define GMACTRL_32		0x6580
#define GMACTRL_33		0x6680
#define GMACTRL_34		0x6780
#define GMACTRL_35		0x6880
#define GMACTRL_36		0x6980
#define GMACTRL_37		0x7080
#define GMACTRL_38		0x7180
#define GMACTRL_39		0x7280
#define GMACTRL_40		0x7380
#define GMACTRL_41		0x7480
#define GMACTRL_42		0x7580
#define GMACTRL_43		0x7680
#define GMACTRL_44		0x7780
#define GMACTRL_45		0x7880
#define GMACTRL_46		0x7980
#define GMACTRL_47		0x7A80
#define GMACTRL_48		0x7B80
#define GMACTRL_49		0x7C80
#define GMACTRL_50		0x7D80
#define GMACTRL_51		0x7E80
#define GMACTRL_52		0x7F80
#define GMACTRL_53		0x8080
#define GMACTRL_54		0x8180
#define GMACTRL_55		0x8880
#define GMACTRL_56		0x8980
#define GMACTRL_57		0x8A80
#define GMACTRL_58		0x8B80
#define GMACTRL_59		0x8C80
#define GMACTRL_60		0x8D80
#define GMACTRL_61		0x8E80
#define GMACTRL_62		0x8F80
#define GMACTRL_63		0x9080
#define GMACTRL_64		0x9180
#define GMACTRL_65		0x9280
#define GMACTRL_66		0x9380
#define GMACTRL_67		0x9480
#define GMACTRL_68		0x9580
#define GMACTRL_69		0x9680
#define GMACTRL_70		0x9780
#define GMACTRL_71		0x9880
#define GMACTRL_72		0x9980
#define GMACTRL_73		0xA080
#define GMACTRL_74		0xA180
#define GMACTRL_75		0xA280
#define GMACTRL_76		0xA380
#define GMACTRL_77		0xA480
#define GMACTRL_78		0xA580
#define GMACTRL_79		0xA680
#define GMACTRL_80		0xA780
#define GMACTRL_81		0xA880
#define GMACTRL_82		0xA980
#define GMACTRL_83		0xAA80
#define GMACTRL_84		0xAB80
#define GMACTRL_85		0xAC80
#define GMACTRL_86		0xAD80
#define GMACTRL_87		0xAE80
#define GMACTRL_88		0xAF80
#define GMACTRL_89		0xB080
#define GMACTRL_90		0xB180
#define GMACTRL_91		0xB880
#define GMACTRL_92		0xB980
#define GMACTRL_93		0xBA80
#define GMACTRL_94		0xBB80
#define GMACTRL_95		0xBC80
#define GMACTRL_96		0xBD80
#define GMACTRL_97		0xBE80
#define GMACTRL_98		0xBF80
#define GMACTRL_99		0xC080
#define GMACTRL_100		0xC180
#define GMACTRL_101		0xC280
#define GMACTRL_102		0xC380
#define GMACTRL_103		0xC480
#define GMACTRL_104		0xC580
#define GMACTRL_105		0xC680
#define GMACTRL_106		0xC780
#define GMACTRL_107		0xC880
#define GMACTRL_108		0xC980
#define DEVICE_CODE_READ	0x1080
#define VENDOR_CODE_READ	0x1280

static void nt35580_lcd_set_disply_on(struct work_struct *ignored);
static DECLARE_WORK(workq_display_on, nt35580_lcd_set_disply_on);

enum mddi_nt35580_lcd_state {
	LCD_STATE_OFF,
	LCD_STATE_WAIT_DISPLAY_ON,
	LCD_STATE_DMA_START,
	LCD_STATE_DMA_COMPLETE,
	LCD_STATE_ON
};

enum mddi_nt35580_client_id {
	CLIENT_ID_UNINITIALIZED,
	CLIENT_ID_TMD_PANEL_OLD_DRIC,
	CLIENT_ID_TMD_PANEL_NEW_DRIC,
	CLIENT_ID_SHARP_PANEL,
	CLIENT_ID_INVALID
};

static enum mddi_nt35580_lcd_state lcd_state;
static enum mddi_nt35580_client_id client_id;

#define write_client_reg(__X, __Y) \
	mddi_queue_register_write(__X, __Y, TRUE, 0);

static uint32 read_client_reg(uint32 addr)
{
	uint32 val;
	mddi_queue_register_read(addr, &val, TRUE, 0);
	return val;
}

struct reg_data {
	uint32 reg_addr;
	uint32 reg_val;
};

static void write_client_reg_table(struct reg_data *table, uint32 size)
{
	int i = 0;
	for (i = 0; i < size; i++)
		write_client_reg(table[i].reg_addr, table[i].reg_val);
}

static struct reg_data exit_sleep_1_tmd_panel_old_dric[] = {
	{ANA_SW_CTR,           0x0005},
	{TEST_MODE_CTR_0,      0x0055},
	{TEST_MODE_CTR_1,      0x00AA},
	{TEST_MODE_CTR_2,      0x0066},
	{PRE_CHG_CTR,          0x0020},
	{DISP_CTRL2,           0x0000},
	{FSWCTRL,              0x0000},
	{SOUTORCTRL,           0x0001},
	{PORCHCTRL,            0x002C},
	{FRCTRL1,              0x0023},
	{CP_CLK,               0x0004},
	{VR_ADJ,               0x0009},
	{GVDD_ADJ,             0x0048},
	{VMH_ADJ,              0x0038},
	{VCMH_ADJ,             0x0070},
	{AVDD2S_AVDD2C_CTRL,   0x0000},
	{AVDD2S_AVDD2C_ADJ,    0x0037},
	{AVDD2S_AVDD2C_CP_CLK, 0x0055},
	{VCMM,                 0x0000},
	{MUX_START,            0x0008},
	{MUX_WIDTH,            0x0015},
	{MUX_GAP,              0x0005},
	{GSP_START,            0x0000},
	{GSP_WIDTH,            0x0062},
	{GCK_START,            0x0001},
	{GCK_WIDTH,            0x005B},
	{CMI_START,            0x0001},
	{LTPS_ENP_CGOE1,       0x00DE},
	{LTPS_ENP_CGOE2,       0x000E},
	{LTPS_ENP_CGOP2,       0x0000},
};

static struct reg_data exit_sleep_1_tmd_panel_new_dric[] = {
	{ANA_SW_CTR,           0x0005},
	{TEST_MODE_CTR_0,      0x0055},
	{TEST_MODE_CTR_1,      0x00AA},
	{TEST_MODE_CTR_2,      0x0066},
	{PRE_CHG_CTR,          0x0020},
	{DISP_CTRL2,           0x0002},
	{FSWCTRL,              0x0000},
	{SOUTORCTRL,           0x0001},
	{PORCHCTRL,            0x002C},
	{FRCTRL1,              0x0023},
	{CP_CLK,               0x0000},
	{VR_ADJ,               0x0000},
	{GVDD_ADJ,             0x0050},
	{VMH_ADJ,              0x006B},
	{VCMH_ADJ,             0x0064},
	{AVDD2S_AVDD2C_CTRL,   0x0000},
	{AVDD2S_AVDD2C_ADJ,    0x00B1},
	{AVDD2S_AVDD2C_CP_CLK, 0x0000},
	{VCMM,                 0x0000},
	{MUX_START,            0x0008},
	{MUX_WIDTH,            0x0016},
	{MUX_GAP,              0x0005},
	{GSP_START,            0x0000},
	{GSP_WIDTH,            0x0062},
	{GCK_START,            0x0001},
	{GCK_WIDTH,            0x005B},
	{CMI_START,            0x0001},
	{LTPS_ENP_CGOE1,       0x00DE},
	{LTPS_ENP_CGOE2,       0x000E},
	{LTPS_ENP_CGOP2,       0x0000},
};

static struct reg_data exit_sleep_1_sharp_panel[] = {
	{DISP_CTRL2,           0x0002},
	{CP_CLK,               0x0043},
	{AVDD2S_AVDD2C_CP_CLK, 0x0000},
	{MUX_START,            0x000F},
	{GCK_WIDTH,            0x0055},
};

static struct reg_data exit_sleep_2_tmd_panel_old_dric[] = {
	{SET_TEAR_ON,          0x0000},
	{SET_TEAR_SCANLINE_1,  0x0000},
	{SET_TEAR_SCANLINE_2,  0x0000},
	{SET_ADDRESS_MODE,     0x0001},
	{LTPS_ENP_CGOP1,       0x0040},
	{ENTER_INVERT_MODE,    0x0000},
};

static struct reg_data exit_sleep_2_tmd_panel_new_dric[] = {
	{GMACTRL_01,  0x0020},
	{GMACTRL_02,  0x0027},
	{GMACTRL_03,  0x0042},
	{GMACTRL_04,  0x0070},
	{GMACTRL_05,  0x0010},
	{GMACTRL_06,  0x0037},
	{GMACTRL_07,  0x0062},
	{GMACTRL_08,  0x008C},
	{GMACTRL_09,  0x001E},
	{GMACTRL_10,  0x0025},
	{GMACTRL_11,  0x00D4},
	{GMACTRL_12,  0x001E},
	{GMACTRL_13,  0x0041},
	{GMACTRL_14,  0x006C},
	{GMACTRL_15,  0x00BC},
	{GMACTRL_16,  0x00DB},
	{GMACTRL_17,  0x0077},
	{GMACTRL_18,  0x0073},
	{GMACTRL_19,  0x0020},
	{GMACTRL_20,  0x0020},
	{GMACTRL_21,  0x0038},
	{GMACTRL_22,  0x0057},
	{GMACTRL_23,  0x0014},
	{GMACTRL_24,  0x003E},
	{GMACTRL_25,  0x0061},
	{GMACTRL_26,  0x0040},
	{GMACTRL_27,  0x001A},
	{GMACTRL_28,  0x0020},
	{GMACTRL_29,  0x0089},
	{GMACTRL_30,  0x001D},
	{GMACTRL_31,  0x0048},
	{GMACTRL_32,  0x006A},
	{GMACTRL_33,  0x00A6},
	{GMACTRL_34,  0x00D9},
	{GMACTRL_35,  0x0078},
	{GMACTRL_36,  0x007F},

	{GMACTRL_37,  0x0020},
	{GMACTRL_38,  0x003A},
	{GMACTRL_39,  0x0055},
	{GMACTRL_40,  0x007C},
	{GMACTRL_41,  0x001A},
	{GMACTRL_42,  0x0041},
	{GMACTRL_43,  0x0063},
	{GMACTRL_44,  0x009B},
	{GMACTRL_45,  0x0019},
	{GMACTRL_46,  0x0024},
	{GMACTRL_47,  0x00DB},
	{GMACTRL_48,  0x001C},
	{GMACTRL_49,  0x003D},
	{GMACTRL_50,  0x006A},
	{GMACTRL_51,  0x00BB},
	{GMACTRL_52,  0x00DB},
	{GMACTRL_53,  0x0077},
	{GMACTRL_54,  0x0073},
	{GMACTRL_55,  0x0020},
	{GMACTRL_56,  0x0022},
	{GMACTRL_57,  0x0038},
	{GMACTRL_58,  0x0058},
	{GMACTRL_59,  0x0016},
	{GMACTRL_60,  0x0041},
	{GMACTRL_61,  0x0061},
	{GMACTRL_62,  0x003A},
	{GMACTRL_63,  0x0019},
	{GMACTRL_64,  0x0022},
	{GMACTRL_65,  0x007B},
	{GMACTRL_66,  0x001B},
	{GMACTRL_67,  0x003E},
	{GMACTRL_68,  0x0063},
	{GMACTRL_69,  0x009A},
	{GMACTRL_70,  0x00CA},
	{GMACTRL_71,  0x0064},
	{GMACTRL_72,  0x007F},

	{GMACTRL_73,  0x0020},
	{GMACTRL_74,  0x003A},
	{GMACTRL_75,  0x0060},
	{GMACTRL_76,  0x008C},
	{GMACTRL_77,  0x0018},
	{GMACTRL_78,  0x0045},
	{GMACTRL_79,  0x0066},
	{GMACTRL_80,  0x00B1},
	{GMACTRL_81,  0x001B},
	{GMACTRL_82,  0x0026},
	{GMACTRL_83,  0x00E5},
	{GMACTRL_84,  0x001C},
	{GMACTRL_85,  0x0047},
	{GMACTRL_86,  0x0069},
	{GMACTRL_87,  0x00BC},
	{GMACTRL_88,  0x00DB},
	{GMACTRL_89,  0x0077},
	{GMACTRL_90,  0x0073},
	{GMACTRL_91,  0x0020},
	{GMACTRL_92,  0x0021},
	{GMACTRL_93,  0x0038},
	{GMACTRL_94,  0x0057},
	{GMACTRL_95,  0x0017},
	{GMACTRL_96,  0x003B},
	{GMACTRL_97,  0x0063},
	{GMACTRL_98,  0x002F},
	{GMACTRL_99,  0x001B},
	{GMACTRL_100, 0x0026},
	{GMACTRL_101, 0x0063},
	{GMACTRL_102, 0x001A},
	{GMACTRL_103, 0x0036},
	{GMACTRL_104, 0x0067},
	{GMACTRL_105, 0x0087},
	{GMACTRL_106, 0x00B4},
	{GMACTRL_107, 0x0064},
	{GMACTRL_108, 0x007F},

	{SET_TEAR_ON,         0x0000},
	{SET_TEAR_SCANLINE_1, 0x0000},
	{SET_TEAR_SCANLINE_2, 0x0000},
	{SET_ADDRESS_MODE,    0x0001},
	{LTPS_ENP_CGOP1,      0x0040},
	{ENTER_INVERT_MODE,   0x0000},
};

static struct reg_data exit_sleep_2_sharp_panel[] = {
	{SET_TEAR_ON,          0x0002},
	{SET_ADDRESS_MODE,     0x0000},
	{SET_HIGH_SPEED_MODE,  0x0001},
	{SET_PIXEL_FORMAT,     0x0077},
	{TEMPORARY_PAR,        0x0005},
};

static struct reg_data set_disply_on_tmd_panel_old_dric[] = {
	{AVDD2S_AVDD2C_CP_CLK, 0x0022},
};

static struct reg_data set_disply_on_tmd_panel_new_dric[] = {
	{AVDD2S_AVDD2C_CP_CLK, 0x0022},
	{CP_CLK,               0x0040},
	{AVDD2S_AVDD2C_ADJ,    0x00BA},
	{VR_ADJ,               0x000C},
};

#define NV_RPC_PROG		0x3000000e
#define NV_RPC_VERS		0x00040001
#define NV_CMD_REMOTE_PROC	9
#define NV_READ			0
#define NV_OEMHW_LCD_VSYNC_I	60007

#define MIN_NV	13389 /* ref100=7468 */
#define MAX_NV	18181 /* ref100=5500 */
#define DEF_NV	16766 /* ref100=5964 */

struct rpc_request_nv_cmd {
	struct rpc_request_hdr hdr;
	uint32_t cmd;
	uint32_t item;
	uint32_t more_data;
	uint32_t desc;
};

struct rpc_reply_nv_cmd {
	struct rpc_reply_hdr hdr;
	uint32_t result;
	uint32_t more_data;
	uint32_t desc;
};

struct nv_oemhw_lcd_vsync {
	uint32_t vsync_usec;
};

static void nt35580_lcd_power_on(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel;
	panel = (struct msm_fb_panel_data *)pdev->dev.platform_data;

	if (panel && panel->panel_ext->power_on)
		panel->panel_ext->power_on();
}

static void nt35580_lcd_driver_initialization(void)
{
	write_client_reg(SET_HORIZONTAL_ADDRESS_0, 0x0000);
	write_client_reg(SET_HORIZONTAL_ADDRESS_1, 0x0000);
	write_client_reg(SET_HORIZONTAL_ADDRESS_2, 0x0001);
	write_client_reg(SET_HORIZONTAL_ADDRESS_3, 0x00DF);

	write_client_reg(SET_VERTICAL_ADDRESS_0, 0x0000);
	write_client_reg(SET_VERTICAL_ADDRESS_1, 0x0000);
	write_client_reg(SET_VERTICAL_ADDRESS_2, 0x0003);
	write_client_reg(SET_VERTICAL_ADDRESS_3, 0x0055);

	write_client_reg(SET_RAM_ADDRESS_0, 0x0000);
	write_client_reg(SET_RAM_ADDRESS_1, 0x0000);
	write_client_reg(SET_RAM_ADDRESS_2, 0x0000);
	write_client_reg(SET_RAM_ADDRESS_3, 0x0000);
}

static void nt35580_lcd_set_client_id(void)
{
	int ret = 0;
	ret = read_client_reg(DEVICE_CODE_READ);

	if (0x58 == ret) {
		client_id = CLIENT_ID_TMD_PANEL_NEW_DRIC;
	} else if (0x55 == ret) {
		ret = read_client_reg(VENDOR_CODE_READ);
		if (0x00 == ret)
			client_id = CLIENT_ID_TMD_PANEL_OLD_DRIC;
		else if (0x01 == ret)
			client_id = CLIENT_ID_SHARP_PANEL;
		else
			client_id = CLIENT_ID_INVALID;
	} else {
		client_id = CLIENT_ID_INVALID;
	}
}

static void nt35580_lcd_exit_sleep(void)
{
	write_client_reg(EXIT_SLEEP_MODE, 0x0000);
	msleep(120);

	if (CLIENT_ID_UNINITIALIZED == client_id)
		nt35580_lcd_set_client_id();

	switch (client_id) {
	case CLIENT_ID_TMD_PANEL_OLD_DRIC:
		write_client_reg_table(exit_sleep_1_tmd_panel_old_dric,
			ARRAY_SIZE(exit_sleep_1_tmd_panel_old_dric));
		break;
	case CLIENT_ID_TMD_PANEL_NEW_DRIC:
		write_client_reg_table(exit_sleep_1_tmd_panel_new_dric,
			ARRAY_SIZE(exit_sleep_1_tmd_panel_new_dric));
		break;
	case CLIENT_ID_SHARP_PANEL:
		write_client_reg_table(exit_sleep_1_sharp_panel,
			ARRAY_SIZE(exit_sleep_1_sharp_panel));
		break;
	default:
		break;
	}

	msleep(10);
	switch (client_id) {
	case CLIENT_ID_TMD_PANEL_OLD_DRIC:
		write_client_reg_table(exit_sleep_2_tmd_panel_old_dric,
			ARRAY_SIZE(exit_sleep_2_tmd_panel_old_dric));
		break;
	case CLIENT_ID_TMD_PANEL_NEW_DRIC:
		write_client_reg_table(exit_sleep_2_tmd_panel_new_dric,
			ARRAY_SIZE(exit_sleep_2_tmd_panel_new_dric));
		break;
	case CLIENT_ID_SHARP_PANEL:
		write_client_reg_table(exit_sleep_2_sharp_panel,
			ARRAY_SIZE(exit_sleep_2_sharp_panel));
		break;
	default:
		break;
	}
}

static void nt35580_lcd_set_disply_on(struct work_struct *ignored)
{
	write_client_reg(SET_DISPLAY_ON, 0x0000);
	msleep(20);

	switch (client_id) {
	case CLIENT_ID_TMD_PANEL_OLD_DRIC:
		write_client_reg_table(set_disply_on_tmd_panel_old_dric,
			ARRAY_SIZE(set_disply_on_tmd_panel_old_dric));
		break;
	case CLIENT_ID_TMD_PANEL_NEW_DRIC:
		write_client_reg_table(set_disply_on_tmd_panel_new_dric,
			ARRAY_SIZE(set_disply_on_tmd_panel_new_dric));
		break;
	case CLIENT_ID_SHARP_PANEL:
		break;
	default:
		break;
	}
	lcd_state = LCD_STATE_ON;
}

static void nt35580_lcd_set_disply_off(void)
{
	write_client_reg(SET_DISPLAY_OFF, 0x0000);
	msleep(70);
}

static void nt35580_lcd_sleep_set(void)
{
	write_client_reg(ENT_SLEEP_MODE, 0x0000);
	msleep(100);
}

static void nt35580_lcd_power_off(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel;
	panel = (struct msm_fb_panel_data *)pdev->dev.platform_data;

	if (panel && panel->panel_ext->power_off)
		panel->panel_ext->power_off();
}

void mddi_nt35580_lcd_display_on(void)
{
	switch (lcd_state) {
	case LCD_STATE_WAIT_DISPLAY_ON:
		lcd_state = LCD_STATE_DMA_START;
		break;
	case LCD_STATE_DMA_START:
		lcd_state = LCD_STATE_DMA_COMPLETE;
		schedule_work(&workq_display_on);
		break;
	default:
		break;
	}
}

static int mddi_nt35580_lcd_lcd_on(struct platform_device *pdev)
{
	switch (lcd_state) {
	case LCD_STATE_OFF:
		nt35580_lcd_power_on(pdev);
		nt35580_lcd_driver_initialization();
		nt35580_lcd_exit_sleep();
		lcd_state = LCD_STATE_WAIT_DISPLAY_ON;
		break;
	default:
		break;
	}
	return 0;
}

static int mddi_nt35580_lcd_lcd_off(struct platform_device *pdev)
{

	switch (lcd_state) {
	case LCD_STATE_ON:
		nt35580_lcd_set_disply_off();
	case LCD_STATE_WAIT_DISPLAY_ON:
	case LCD_STATE_DMA_START:
	case LCD_STATE_DMA_COMPLETE:
		nt35580_lcd_sleep_set();
		nt35580_lcd_power_off(pdev);
		lcd_state = LCD_STATE_OFF;
		break;
	default:
		break;
	}

	return 0;
}

static int nt35580_lcd_get_nv_vsync(void)
{
	struct get_lcd_vsync_req{
		struct rpc_request_nv_cmd	nv;
		struct nv_oemhw_lcd_vsync	data;
	} req;

	struct get_lcd_vsync_rep{
		struct rpc_reply_nv_cmd		nv;
		struct nv_oemhw_lcd_vsync	data;
	} rep;
	struct msm_rpc_endpoint *endpoint;
	uint32_t item = NV_OEMHW_LCD_VSYNC_I;
	int rc;
	endpoint = msm_rpc_connect(NV_RPC_PROG, NV_RPC_VERS, 0);
	if (IS_ERR(endpoint)) {
		MDDI_MSG_ERR("%s: msm_rpc_connect failed\n", __func__);
		return 0;
	}
	req.nv.cmd			= cpu_to_be32(NV_READ);
	req.nv.item			= cpu_to_be32(item);
	req.nv.more_data		= cpu_to_be32(1);
	req.nv.desc			= cpu_to_be32(item);
	rc = msm_rpc_call_reply(endpoint, NV_CMD_REMOTE_PROC,
						  &req, sizeof(req),
						  &rep, sizeof(rep),
						  5 * HZ);
	if (rc < 0) {
		MDDI_MSG_ERR("%s: msm_rpc_call_reply failed!\n", __func__);
		return 0;
	}
	return be32_to_cpu(rep.data.vsync_usec);
}

static int __init mddi_nt35580_lcd_lcd_probe(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel_data;
	int nv_vsync = 0;

	if (!pdev) {
		printk(KERN_ERR "%s: Display failed\n", __func__);
		return -1;
	}
	if (!pdev->dev.platform_data) {
		printk(KERN_ERR "%s: Display failed, no platform data\n",
			__func__);
		return -1;
	}
	panel_data = (struct msm_fb_panel_data *)pdev->dev.platform_data;

	panel_data->on  = mddi_nt35580_lcd_lcd_on;
	panel_data->off = mddi_nt35580_lcd_lcd_off;

	nv_vsync = nt35580_lcd_get_nv_vsync();
	nv_vsync >>= 16;
	nv_vsync &= (0xffff);
	if ((MIN_NV > nv_vsync) || (nv_vsync > MAX_NV))
		nv_vsync = DEF_NV ;
	panel_data->panel_info.lcd.refx100 = 100000000 / nv_vsync;

	msm_fb_add_device(pdev);
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mddi_nt35580_lcd_lcd_probe,
	.driver = {
		.name   = "mddi_tmd_wvga",
	},
};

static int __init mddi_nt35580_lcd_lcd_init(void)
{
	int ret;
	client_id = CLIENT_ID_UNINITIALIZED;
	lcd_state = LCD_STATE_OFF;
	ret = platform_driver_register(&this_driver);
	return ret;
}

module_init(mddi_nt35580_lcd_lcd_init);
