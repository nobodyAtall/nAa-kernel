/* drivers/media/video/msm/dlt001_cam_devdrv_table.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef CAMSENSOR_DLT001_CAM_DEVDRV_TABLE
#define CAMSENSOR_DLT001_CAM_DEVDRV_TABLE

enum reg_bits {
	REG_BITS_8 = 0,
	REG_BITS_16,
	REG_BITS_32
};

struct reg_entry {
	uint16_t address;
	uint32_t data;
	enum reg_bits reg_bits;
};

#define dlt001_set_regs(table) dlt001_send_reg_table(table, (sizeof_##table/sizeof(struct reg_entry)))

extern const struct reg_entry dlt001_GEN_period_1_ES1[];
extern int32_t sizeof_dlt001_GEN_period_1_ES1;
extern const struct reg_entry dlt001_GEN_period_1_ES2[];
extern int32_t sizeof_dlt001_GEN_period_1_ES2;
extern const struct reg_entry dlt001_vendor_1_period_2_ES1[];
extern int32_t sizeof_dlt001_vendor_1_period_2_ES1;
extern const struct reg_entry dlt001_vendor_1_period_2_ES2[];
extern int32_t sizeof_dlt001_vendor_1_period_2_ES2;
extern const struct reg_entry dlt001_vendor_0_period_2_ES1[];
extern int32_t sizeof_dlt001_vendor_0_period_2_ES1;
extern const struct reg_entry dlt001_vendor_0_period_2_ES2[];
extern int32_t sizeof_dlt001_vendor_0_period_2_ES2;
extern const struct reg_entry dlt001_vendor_1_period_3_ES1[];
extern int32_t sizeof_dlt001_vendor_1_period_3_ES1;
extern const struct reg_entry dlt001_vendor_1_period_3_ES2[];
extern int32_t sizeof_dlt001_vendor_1_period_3_ES2;
extern const struct reg_entry dlt001_vendor_0_period_3_ES1[];
extern int32_t sizeof_dlt001_vendor_0_period_3_ES1;
extern const struct reg_entry dlt001_vendor_0_period_3_ES2[];
extern int32_t sizeof_dlt001_vendor_0_period_3_ES2;
extern const struct reg_entry dlt001_vendor_0_SHD_1_ES2[];
extern int32_t sizeof_dlt001_vendor_0_SHD_1_ES2;
extern const struct reg_entry dlt001_vendor_0_SHD_2_ES2[];
extern int32_t sizeof_dlt001_vendor_0_SHD_2_ES2;
extern const struct reg_entry dlt001_vendor_0_SHD_3_ES2[];
extern int32_t sizeof_dlt001_vendor_0_SHD_3_ES2;
extern const struct reg_entry dlt001_vendor_1_SHD_1_ES2[];
extern int32_t sizeof_dlt001_vendor_1_SHD_1_ES2;
extern const struct reg_entry dlt001_vendor_1_SHD_2_ES2[];
extern int32_t sizeof_dlt001_vendor_1_SHD_2_ES2;
extern const struct reg_entry dlt001_vendor_1_SHD_3_ES2[];
extern int32_t sizeof_dlt001_vendor_1_SHD_3_ES2;
extern const struct reg_entry dlt001_vf_resolution_640x480[];
extern int32_t sizeof_dlt001_vf_resolution_640x480;
extern const struct reg_entry dlt001_vf_resolution_320x240[];
extern int32_t sizeof_dlt001_vf_resolution_320x240;
extern const struct reg_entry dlt001_snapshot_resolution_640x480[];
extern int32_t sizeof_dlt001_snapshot_resolution_640x480;
extern const struct reg_entry dlt001_snapshot_resolution_1280x960[];
extern int32_t sizeof_dlt001_snapshot_resolution_1280x960;
extern const struct reg_entry dlt001_snapshot_resolution_1632x1224[];
extern int32_t sizeof_dlt001_snapshot_resolution_1632x1224;
extern const struct reg_entry dlt001_snapshot_resolution_2592x1944[];
extern int32_t sizeof_dlt001_snapshot_resolution_2592x1944;
extern const struct reg_entry dlt001_thumbnail_size_VGA[];
extern int32_t sizeof_dlt001_thumbnail_size_VGA;
extern const struct reg_entry dlt001_thumbnail_size_QVGA[];
extern int32_t sizeof_dlt001_thumbnail_size_QVGA;
extern const struct reg_entry dlt001_mode_monitor[];
extern int32_t sizeof_dlt001_mode_monitor;
extern const struct reg_entry dlt001_mode_half_release[];
extern int32_t sizeof_dlt001_mode_half_release;
extern const struct reg_entry dlt001_hr_auto_start[];
extern int32_t sizeof_dlt001_hr_auto_start;
extern const struct reg_entry dlt001_hr_auto_reset[];
extern int32_t sizeof_dlt001_hr_auto_reset;
extern const struct reg_entry dlt001_hr_twilight[];
extern int32_t sizeof_dlt001_hr_twilight;
extern const struct reg_entry dlt001_hr_reset[];
extern int32_t sizeof_dlt001_hr_reset;
extern const struct reg_entry dlt001_hr_LED[];
extern int32_t sizeof_dlt001_hr_LED;
extern const struct reg_entry dlt001_hr_LED_reset[];
extern int32_t sizeof_dlt001_hr_LED_reset;
extern const struct reg_entry dlt001_test_pattern_on[];
extern int32_t sizeof_dlt001_test_pattern_on;
extern const struct reg_entry dlt001_test_pattern_off[];
extern int32_t sizeof_dlt001_test_pattern_off;
extern const struct reg_entry dlt001_prepare_mode_capture[];
extern int32_t sizeof_dlt001_prepare_mode_capture;
extern const struct reg_entry dlt001_mode_capture[];
extern int32_t sizeof_dlt001_mode_capture;
extern const struct reg_entry dlt001_INTCLR[];
extern int32_t sizeof_dlt001_INTCLR;
extern const struct reg_entry dlt001_MONI_REFRESH_F[];
extern int32_t sizeof_dlt001_MONI_REFRESH_F;
extern const struct reg_entry dlt001_scene_normal[];
extern int32_t sizeof_dlt001_scene_normal;
extern const struct reg_entry dlt001_scene_sports[];
extern int32_t sizeof_dlt001_scene_sports;
extern const struct reg_entry dlt001_scene_twilight[];
extern int32_t sizeof_dlt001_scene_twilight;
extern const struct reg_entry dlt001_framerate_30[];
extern int32_t sizeof_dlt001_framerate_30;
extern const struct reg_entry dlt001_framerate_variable[];
extern int32_t sizeof_dlt001_framerate_variable;
extern const struct reg_entry dlt001_focus_mode_auto[];
extern int32_t sizeof_dlt001_focus_mode_auto;
extern const struct reg_entry dlt001_focus_mode_macro[];
extern int32_t sizeof_dlt001_focus_mode_macro;
extern const struct reg_entry dlt001_focus_mode_continuous[];
extern int32_t sizeof_dlt001_focus_mode_continuous;
extern const uint16_t AEO_table[];
extern const struct reg_entry dlt001_mode_capture_YUV[];
extern int32_t sizeof_dlt001_mode_capture_YUV;
extern const struct reg_entry dlt001_primary_focus_window_continuous[];
extern int32_t sizeof_dlt001_primary_focus_window_continuous;
extern const struct reg_entry dlt001_primary_focus_window_auto[];
extern int32_t sizeof_dlt001_primary_focus_window_auto;
#endif
