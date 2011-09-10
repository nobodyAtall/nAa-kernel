/* drivers/media/video/msm/dlt002_cam_devdrv_table.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef CAMSENSOR_DLT002_CAM_DEVDRV_TABLE
#define CAMSENSOR_DLT002_CAM_DEVDRV_TABLE

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

#define dlt002_set_regs(table) dlt002_send_reg_table(table, (sizeof_##table/sizeof(struct reg_entry)))

extern const struct reg_entry dlt002_GEN_period_1_ES1[];
extern int32_t sizeof_dlt002_GEN_period_1_ES1;
extern const struct reg_entry dlt002_vendor_0_period_2_ES1[];
extern int32_t sizeof_dlt002_vendor_0_period_2_ES1;
extern const struct reg_entry dlt002_vendor_1_period_2_ES1[];
extern int32_t sizeof_dlt002_vendor_1_period_2_ES1;
extern const struct reg_entry dlt002_vendor_0_period_3_ES1[];
extern int32_t sizeof_dlt002_vendor_0_period_3_ES1;
extern const struct reg_entry dlt002_vendor_1_period_3_ES1[];
extern int32_t sizeof_dlt002_vendor_1_period_3_ES1;
extern const struct reg_entry dlt002_snapshot_resolution_640x480[];
extern int32_t sizeof_dlt002_snapshot_resolution_640x480;
extern const struct reg_entry dlt002_snapshot_resolution_1280x960[];
extern int32_t sizeof_dlt002_snapshot_resolution_1280x960;
extern const struct reg_entry dlt002_snapshot_resolution_1632x1224[];
extern int32_t sizeof_dlt002_snapshot_resolution_1632x1224;
extern const struct reg_entry dlt002_snapshot_resolution_2048x1536[];
extern int32_t sizeof_dlt002_snapshot_resolution_2048x1536;
extern const struct reg_entry dlt002_thumbnail_size_VGA[];
extern int32_t sizeof_dlt002_thumbnail_size_VGA;
extern const struct reg_entry dlt002_thumbnail_size_QVGA[];
extern int32_t sizeof_dlt002_thumbnail_size_QVGA;
extern const struct reg_entry dlt002_mode_monitor[];
extern int32_t sizeof_dlt002_mode_monitor;
extern const struct reg_entry dlt002_test_pattern_on[];
extern int32_t sizeof_dlt002_test_pattern_on;
extern const struct reg_entry dlt002_test_pattern_off[];
extern int32_t sizeof_dlt002_test_pattern_off;
extern const struct reg_entry dlt002_prepare_mode_capture[];
extern int32_t sizeof_dlt002_prepare_mode_capture;
extern const struct reg_entry dlt002_mode_capture[];
extern int32_t sizeof_dlt002_mode_capture;
extern const struct reg_entry dlt002_scene_normal[];
extern int32_t sizeof_dlt002_scene_normal;
extern const struct reg_entry dlt002_scene_beach_snow[];
extern int32_t sizeof_dlt002_scene_beach_snow;
extern const struct reg_entry dlt002_scene_sports[];
extern int32_t sizeof_dlt002_scene_sports;
extern const struct reg_entry dlt002_scene_twilight[];
extern int32_t sizeof_dlt002_scene_twilight;
extern const struct reg_entry dlt002_framerate_30[];
extern int32_t sizeof_dlt002_framerate_30;
extern const struct reg_entry dlt002_framerate_variable[];
extern int32_t sizeof_dlt002_framerate_variable;
extern const struct reg_entry dlt002_mode_capture_YUV[];
extern int32_t sizeof_dlt002_mode_capture_YUV;
#endif
