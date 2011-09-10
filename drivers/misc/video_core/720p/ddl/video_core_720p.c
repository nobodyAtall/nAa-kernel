/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 *
 */

#include <linux/unistd.h>

#include "video_core_type.h"
#include "video_core_720p.h"

#if DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define VIDC_720P_VERSION_STRING "VIDC_V1.0"
u8 *vid_c_base_addr;

u32 VID_C_REG_319934_SHADOW;
u32 VID_C_REG_101184_SHADOW;
u32 VID_C_REG_490443_SHADOW;
u32 VID_C_REG_64895_SHADOW;
u32 VID_C_REG_804959_SHADOW;
u32 VID_C_REG_883500_SHADOW;
u32 VID_C_REG_615716_SHADOW[32];
u32 VID_C_REG_603032_SHADOW;
u32 VID_C_REG_792026_SHADOW;
u32 VID_C_REG_844152_SHADOW;
u32 VID_C_REG_147682_SHADOW;
u32 VID_C_REG_407718_SHADOW;
u32 VID_C_REG_854681_SHADOW;
u32 VID_C_REG_128234_SHADOW;
u32 VID_C_REG_988552_SHADOW;
u32 VID_C_REG_94750_SHADOW;
u32 VID_C_REG_76706_SHADOW;
u32 VID_C_REG_486169_SHADOW;
u32 VID_C_REG_699747_SHADOW;
u32 VID_C_REG_166247_SHADOW;
u32 VID_C_REG_926519_SHADOW;
u32 VID_C_REG_965480_SHADOW;


#ifdef VIDC_REGISTER_LOG_INTO_BUFFER
char vidclog[VIDC_REGLOG_BUFSIZE];
unsigned int vidclog_index;
#endif

void vidc_720p_set_device_virtual_base(u8 *p_core_virtual_base_addr)
{
	vid_c_base_addr = p_core_virtual_base_addr;
}

void vidc_720p_init(char **ppsz_version, u32 i_firmware_size,
		     u32 *pi_firmware_address,
		     enum vidc_720p_endian_type e_dma_endian,
		     u32 b_interrupt_off,
		     enum vidc_720p_interrupt_level_selection_type
		     e_interrupt_sel, u32 interrupt_mask)
{
	if (ppsz_version)
		*ppsz_version = VIDC_720P_VERSION_STRING;

	if (e_interrupt_sel == VIDC_720P_INTERRUPT_LEVEL_SEL)
		VIDC_IO_OUT(REG_491082, 0);
	else
		VIDC_IO_OUT(REG_491082, 1);

	if (b_interrupt_off)
		VIDC_IO_OUT(REG_609676, 1);
	else
		VIDC_IO_OUT(REG_609676, 0);

	VIDC_IO_OUT(REG_614776, 1);

	VIDC_IO_OUT(REG_418173, 0);

	VIDC_IO_OUT(REG_418173, interrupt_mask);

	VIDC_IO_OUT(REG_736316, e_dma_endian);

	VIDC_IO_OUT(REG_215724, 0);

	VIDC_IO_OUT(REG_361582, 1);

	VIDC_IO_OUT(REG_591577, i_firmware_size);

	VIDC_IO_OUT(REG_203921, pi_firmware_address);

	VIDC_IO_OUT(REG_531515_ADDR, 0);

	VIDC_IO_OUT(REG_614413, 1);
}

u32 vidc_720p_do_sw_reset(void)
{

	u32 n_fw_start = 0;
	VIDC_BUSY_WAIT(5);
	VIDC_IO_OUT(REG_224135, 0);
	VIDC_BUSY_WAIT(5);
	VIDC_IO_OUT(REG_193553, 0);
	VIDC_BUSY_WAIT(5);
	VIDC_IO_OUT(REG_141269, 1);
	VIDC_BUSY_WAIT(15);
	VIDC_IO_OUT(REG_141269, 0);
	VIDC_BUSY_WAIT(5);
	VIDC_IO_IN(REG_193553, &n_fw_start);

	if (!n_fw_start) {
		DBG("\n VIDC-SW-RESET-FAILS!");
		return FALSE;
	}
	return TRUE;
}

u32 vidc_720p_reset_is_success()
{
	u32 n_stagecounter = 0;
	VIDC_IO_IN(REG_352831, &n_stagecounter);
	n_stagecounter &= 0xff;
	if (n_stagecounter != 0xe5) {
		DBG("\n VIDC-CPU_RESET-FAILS!");
		VIDC_IO_OUT(REG_224135, 0);
		msleep(10);
		return FALSE;
	}
	return TRUE;
}

void vidc_720p_start_cpu(enum vidc_720p_endian_type e_dma_endian,
						  u32 *p_icontext_bufferstart,
						  u32 *p_debug_core_dump_addr,
						  u32  debug_buffer_size)
{
	VIDC_IO_OUT(REG_361582, 0);
	VIDC_IO_OUT(REG_958768, p_icontext_bufferstart);
	VIDC_IO_OUT(REG_736316, e_dma_endian);
	VIDC_IO_OUT(REG_699747, 0x1);
	if (debug_buffer_size) {
		VIDC_IO_OUTF(REG_699747, ALLOCATED_MEM_SIZE, \
					 debug_buffer_size);
		VIDC_IO_OUTF(REG_699747,
					  SET_MEMORY_DUMP_TYPE, 0x2);
		VIDC_IO_OUT(REG_166247, p_debug_core_dump_addr);
	}
	VIDC_IO_OUT(REG_224135, 1);
}

u32 vidc_720p_cpu_start()
{
	u32 fw_status = 0x0;
	VIDC_IO_IN(REG_381535, &fw_status);
	if (fw_status != 0x02)
		return FALSE;
	return TRUE;
}


void vidc_720p_stop_fw(void)
{
   VIDC_IO_OUT(REG_193553, 0);
   VIDC_IO_OUT(REG_224135, 0);
}

void vidc_720p_get_interrupt_status(u32 *p_interrupt_status,
	u32 *p_cmd_err_status, u32 *p_disp_pic_err_status, u32 *p_op_failed)
{
	u32 err_status;
	VIDC_IO_IN(REG_512143, p_interrupt_status);
	VIDC_IO_IN(REG_300310, &err_status);
	*p_cmd_err_status = err_status & 0xffff;
	*p_disp_pic_err_status = (err_status & 0xffff0000) >> 16;
	VIDC_IO_INF(REG_724381, OPERATION_FAILED, \
				 p_op_failed);
}

void vidc_720p_interrupt_done_clear(void)
{
	VIDC_IO_OUT(REG_614776, 1);
	VIDC_IO_OUT(REG_97293, 4);
}

void vidc_720p_submit_command(u32 ch_id, u32 n_cmd_id)
{
	u32 fw_status;
	VIDC_IO_OUT(REG_97293, ch_id);
	VIDC_IO_OUT(REG_62325, n_cmd_id);
	VIDC_DEBUG_REGISTER_LOG;
	VIDC_IO_IN(REG_381535, &fw_status);
	VIDC_IO_OUT(REG_926519, fw_status);
}

u32 vidc_720p_engine_reset(u32 n_ch_id,
	enum vidc_720p_endian_type e_dma_endian,
	enum vidc_720p_interrupt_level_selection_type e_interrupt_sel,
	u32 interrupt_mask
)
{
	u32 n_op_done = 0;
	u32 n_counter = 0;

	VIDC_LOGERR_STRING("ENG-RESET!!");
	/* issue the engine reset command */
	vidc_720p_submit_command(n_ch_id, VIDC_720P_CMD_MFC_ENGINE_RESET);

	do {
		VIDC_BUSY_WAIT(20);
		VIDC_IO_IN(REG_982553, &n_op_done);
		n_counter++;
	} while (!n_op_done && n_counter < 10);

	if (!n_op_done) {
		/* Reset fails */
		return  FALSE ;
	}

	/* write invalid channel id */
	VIDC_IO_OUT(REG_97293, 4);

	/* Set INT_PULSE_SEL */
	if (e_interrupt_sel == VIDC_720P_INTERRUPT_LEVEL_SEL)
		VIDC_IO_OUT(REG_491082, 0);
	else
		VIDC_IO_OUT(REG_491082, 1);

	if (!interrupt_mask) {
		/* Disable interrupt */
		VIDC_IO_OUT(REG_609676, 1);
	} else {
	  /* Enable interrupt */
		VIDC_IO_OUT(REG_609676, 0);
	}

	/* Clear any pending interrupt */
	VIDC_IO_OUT(REG_614776, 1);

	/* Set INT_ENABLE_REG */
	VIDC_IO_OUT(REG_418173, interrupt_mask);

	/*Sets the DMA endianness */
	VIDC_IO_OUT(REG_736316, e_dma_endian);

	/* retun engine reset success */
	return TRUE ;
}

void vidc_720p_set_channel(u32 i_ch_id,
			    enum vidc_720p_enc_dec_selection_type
			    e_enc_dec_sel, enum vidc_720p_codec_type e_codec,
			    u32 *pi_fw, u32 i_firmware_size)
{
	u32 std_sel = 0;
	VIDC_IO_OUT(REG_661565, 0);

	if (e_enc_dec_sel)
		std_sel = VIDC_REG_713080_ENC_ON_BMSK;

	std_sel |= (u32) e_codec;

	VIDC_IO_OUT(REG_713080, std_sel);

	switch (e_codec) {
	default:
	case VIDC_720p_DIVX:
	case VIDC_720p_XVID:
	case VIDC_720p_MPEG4:
		{
			if (VIDC_720p_ENCODER == e_enc_dec_sel)
				VIDC_IO_OUT(REG_765787, pi_fw);
			else
				VIDC_IO_OUT(REG_225040, pi_fw);
			break;
		}
	case VIDC_720p_H264:
		{
			if (VIDC_720p_ENCODER == e_enc_dec_sel)
				VIDC_IO_OUT(REG_942456, pi_fw);
			else
				VIDC_IO_OUT(REG_942170_ADDR_3, pi_fw);
			break;
		}
	case VIDC_720p_H263:
		{
			if (VIDC_720p_ENCODER == e_enc_dec_sel)
				VIDC_IO_OUT(REG_765787, pi_fw);
			else
				VIDC_IO_OUT(REG_942170_ADDR_6, pi_fw);
			break;
		}
	case VIDC_720p_VC1:
		{
			VIDC_IO_OUT(REG_880188, pi_fw);
			break;
		}
	case VIDC_720p_MPEG2:
		{
			VIDC_IO_OUT(REG_40293, pi_fw);
			break;
		}
	}
	VIDC_IO_OUT(REG_591577, i_firmware_size);

	vidc_720p_submit_command(i_ch_id, VIDC_720P_CMD_CHSET);
}

void vidc_720p_encode_set_profile(u32 i_profile, u32 i_level)
{
	VIDC_IO_OUT(REG_839021, 0x0);

	VIDC_IO_OUTF(REG_839021, PROFILE, i_profile);
	VIDC_IO_OUTF(REG_839021, LEVEL, i_level);

}

void vidc_720p_set_frame_size(u32 i_size_x, u32 i_size_y)
{
	VIDC_IO_OUT(REG_999267, i_size_x);

	VIDC_IO_OUT(REG_345712, i_size_y);
}

void vidc_720p_encode_set_fps(u32 i_rc_frame_rate)
{
	VIDC_IO_OUT(REG_625444, 0);

	VIDC_IO_OUTF(REG_625444, FRAME_RATE, i_rc_frame_rate);
}

void vidc_720p_encode_set_short_header(u32 i_short_header)
{
	VIDC_IO_OUT(REG_314290, i_short_header);
}

void vidc_720p_encode_set_vop_time(u32 n_vop_time_resolution,
				    u32 n_vop_time_increment)
{
	if (!n_vop_time_resolution) {
		VIDC_IO_OUT(REG_64895, 0x0);
	} else {
		VIDC_IO_OUTF(REG_64895, VOP_TIMING_ENABLE, 0x1);
		VIDC_IO_OUTF(REG_64895, VOP_TIME_RESOLUTION,
				  n_vop_time_resolution);
		VIDC_IO_OUTF(REG_64895, FRAME_DELTA,
					  n_vop_time_increment);
	}
}

void vidc_720p_encode_hdr_ext_control(u32 n_header_extension)
{
	if (n_header_extension) {
		VIDC_IO_OUTF(REG_128234, HEC_ENABLE, 0x1);
		VIDC_IO_OUT(REG_407718, n_header_extension);
	} else {
		VIDC_IO_OUTF(REG_128234, HEC_ENABLE, 0x0);
	}
}

void vidc_720p_encode_set_qp_params(u32 i_max_qp, u32 i_min_qp)
{
	VIDC_IO_OUT(REG_734318, 0);

	VIDC_IO_OUTF(REG_734318, MAX_QP, i_max_qp);

	VIDC_IO_OUTF(REG_734318, MIN_QP, i_min_qp);
}

void vidc_720p_encode_set_rc_config(u32 b_enable_frame_level_rc,
				     u32 b_enable_mb_level_rc_flag,
				     u32 i_frame_qp, u32 n_pframe_qp)
{
	VIDC_IO_OUT(REG_58211, 0);

	VIDC_IO_OUTF(REG_58211, FR_RC_EN,
		      b_enable_frame_level_rc);

	VIDC_IO_OUTF(REG_58211, MB_RC_EN,
		      b_enable_mb_level_rc_flag);

	VIDC_IO_OUTF(REG_58211, FRAME_QP, i_frame_qp);

	VIDC_IO_OUTF(REG_548359, P_FRAME_QP, n_pframe_qp);
}

void vidc_720p_encode_set_bit_rate(u32 i_target_bitrate)
{
	VIDC_IO_OUTF(REG_174150, BIT_RATE, i_target_bitrate);
}

void vidc_720p_encode_dynamic_req_reset(void)
{
	VIDC_IO_OUTF(REG_128234, INSERT_I_FRAME, 0x0);
	VIDC_IO_OUT(REG_804959, 0x0);
}

void vidc_720p_encode_dynamic_req_set(u32 n_param_type, u32 n_param_val)
{
	switch (n_param_type) {
	case VIDC_720p_ENC_IFRAME_REQ:
		{
			VIDC_IO_OUTF(REG_128234, INSERT_I_FRAME,
				      0x1);
			break;
		}
	case VIDC_720p_ENC_IPERIOD_CHANGE:
		{
			VIDC_IO_OUTF(REG_804959,
				      I_PERIOD_CHANGE, 0x1);
			VIDC_IO_OUT(REG_950374, n_param_val);
			break;
		}
	case VIDC_720p_ENC_FRAMERATE_CHANGE:
		{
			VIDC_IO_OUTF(REG_804959,
				      RC_FRAME_RATE_CHANGE, 0x1);
			VIDC_IO_OUTF(REG_625444, FRAME_RATE,
				      n_param_val);
			break;
		}
	case VIDC_720p_ENC_BITRATE_CHANGE:
		{
			VIDC_IO_OUTF(REG_804959,
				      RC_BIT_RATE_CHANGE, 0x1);
			VIDC_IO_OUT(REG_174150, n_param_val);

			break;
		}
	}
}

void vidc_720p_encode_set_frame_level_rc_params(u32 i_reaction_coeff)
{
	VIDC_IO_OUT(REG_677784, 0);

	VIDC_IO_OUTF(REG_677784, REACT_PARA, i_reaction_coeff);
}

void vidc_720p_encode_set_mb_level_rc_params(u32 b_dark_region_as_flag,
					      u32 b_smooth_region_as_flag,
					      u32 b_static_region_as_flag,
					      u32 b_activity_region_flag)
{
	VIDC_IO_OUT(REG_995041, 0);

	VIDC_IO_OUTF(REG_995041, DARK_DISABLE, b_dark_region_as_flag);

	VIDC_IO_OUTF(REG_995041, SMOOTH_DISABLE,
		      b_smooth_region_as_flag);

	VIDC_IO_OUTF(REG_995041, STATIC_DISABLE,
		      b_static_region_as_flag);

	VIDC_IO_OUTF(REG_995041, ACT_DISABLE, b_activity_region_flag);
}

void vidc_720p_encode_set_entropy_control(enum vidc_720p_entropy_sel_type
					   e_entropy_sel,
					   enum vidc_720p_cabac_model_type
					   e_cabac_model_number)
{
	u32 n_sel = (u32) e_entropy_sel;
	VIDC_IO_OUT(REG_504878, n_sel);

	if (e_entropy_sel == VIDC_720p_ENTROPY_SEL_CABAC) {
		u32 n_num = (u32) e_cabac_model_number;
		VIDC_IO_OUTF(REG_504878, FIXED_NUMBER, n_num);
	}
}

void vidc_720p_encode_set_db_filter_control(enum vidc_720p_DBConfig_type
					     e_db_config,
					     u32 i_slice_alpha_offset,
					     u32 i_slice_beta_offset)
{
	VIDC_IO_OUT(REG_458130, 0);

	VIDC_IO_OUTF(REG_458130,
		      SLICE_ALPHA_C0_OFFSET_DIV2, i_slice_alpha_offset);

	VIDC_IO_OUTF(REG_458130,
		      SLICE_BETA_OFFSET_DIV2, i_slice_beta_offset);

	VIDC_IO_OUTF(REG_458130,
		      DISABLE_DEBLOCKING_FILTER_IDC, e_db_config);
}

void vidc_720p_encode_set_intra_refresh_mb_number(u32 i_cir_mb_number)
{
	VIDC_IO_OUT(REG_857491, i_cir_mb_number);
}

void vidc_720p_encode_set_multi_slice_info(enum
					    vidc_720p_MSlice_selection_type
					    e_m_slice_sel,
					    u32 n_multi_slice_size)
{
	switch (e_m_slice_sel) {
	case VIDC_720P_MSLICE_BY_MB_COUNT:
		{
			VIDC_IO_OUT(REG_588301, 0x1);
			VIDC_IO_OUT(REG_1517, e_m_slice_sel);
			VIDC_IO_OUT(REG_105335, n_multi_slice_size);
			break;
		}
	case VIDC_720P_MSLICE_BY_BYTE_COUNT:
		{
			VIDC_IO_OUT(REG_588301, 0x1);
			VIDC_IO_OUT(REG_1517, e_m_slice_sel);
			VIDC_IO_OUT(REG_561679, n_multi_slice_size);
			break;
		}
	case VIDC_720P_MSLICE_BY_GOB:
		{
			VIDC_IO_OUT(REG_588301, 0x1);
			break;
		}
	default:
	case VIDC_720P_MSLICE_OFF:
		{
			VIDC_IO_OUT(REG_588301, 0x0);
			break;
		}
	}
}

void vidc_720p_encode_set_dpb_buffer(u32 *pi_enc_dpb_addr, u32 alloc_len)
{
	VIDC_IO_OUT(REG_341928_ADDR, pi_enc_dpb_addr);
	VIDC_IO_OUT(REG_319934, alloc_len);
}

void vidc_720p_encode_set_i_period(u32 i_i_period)
{
	VIDC_IO_OUT(REG_950374, i_i_period);
}

void vidc_720p_encode_init_codec(u32 i_ch_id,
				  enum vidc_720p_memory_access_method_type
				  e_memory_access_model)
{

	VIDC_IO_OUT(REG_841539, e_memory_access_model);
	vidc_720p_submit_command(i_ch_id, VIDC_720P_CMD_INITCODEC);
}

void vidc_720p_encode_unalign_bitstream(u32 n_upper_unalign_word,
					 u32 n_lower_unalign_word)
{
	VIDC_IO_OUT(REG_792026, n_upper_unalign_word);
	VIDC_IO_OUT(REG_844152, n_lower_unalign_word);
}

void vidc_720p_encode_set_seq_header_buffer(u32 n_ext_buffer_start,
					     u32 n_ext_buffer_end,
					     u32 n_start_byte_num)
{
	VIDC_IO_OUT(REG_275113_ADDR, n_ext_buffer_start);

	VIDC_IO_OUT(REG_87912, n_ext_buffer_start);

	VIDC_IO_OUT(REG_988007_ADDR, n_ext_buffer_end);

	VIDC_IO_OUT(REG_66693, n_start_byte_num);
}

void vidc_720p_encode_frame(u32 n_ch_id,
			     u32 n_ext_buffer_start,
			     u32 n_ext_buffer_end,
			     u32 n_start_byte_number, u32 n_y_addr,
			     u32 n_c_addr)
{
	VIDC_IO_OUT(REG_275113_ADDR, n_ext_buffer_start);

	VIDC_IO_OUT(REG_988007_ADDR, n_ext_buffer_end);

	VIDC_IO_OUT(REG_87912, n_ext_buffer_start);

	VIDC_IO_OUT(REG_66693, n_start_byte_number);

	VIDC_IO_OUT(REG_99105, n_y_addr);

	VIDC_IO_OUT(REG_777113_ADDR, n_c_addr);

	vidc_720p_submit_command(n_ch_id, VIDC_720P_CMD_FRAMERUN);
}

void vidc_720p_encode_get_header(u32 *pi_enc_header_size)
{
	VIDC_IO_IN(REG_114286, pi_enc_header_size);
}

void vidc_720p_enc_frame_info(struct vidc_720p_enc_frame_info_type
			       *p_enc_frame_info)
{
	VIDC_IO_IN(REG_782249, &p_enc_frame_info->n_enc_size);

	VIDC_IO_IN(REG_441270, &p_enc_frame_info->n_frame_type);

	p_enc_frame_info->n_frame_type &= 0x03;

	VIDC_IO_IN(REG_613254,
		    &p_enc_frame_info->n_metadata_exists);
}

void vidc_720p_decode_bitstream_header(u32 n_ch_id,
					u32 n_dec_unit_size,
					u32 n_start_byte_num,
					u32 n_ext_buffer_start,
					u32 n_ext_buffer_end,
					enum
					vidc_720p_memory_access_method_type
					e_memory_access_model)
{
	VIDC_IO_OUT(REG_965480, 0x0);

	VIDC_IO_OUT(REG_275113_ADDR, n_ext_buffer_start);

	VIDC_IO_OUT(REG_988007_ADDR, n_ext_buffer_end);

	VIDC_IO_OUT(REG_87912, n_ext_buffer_end);

	VIDC_IO_OUT(REG_761892, n_dec_unit_size);

	VIDC_IO_OUT(REG_66693, n_start_byte_num);

	VIDC_IO_OUT(REG_841539, e_memory_access_model);

	vidc_720p_submit_command(n_ch_id, VIDC_720P_CMD_INITCODEC);
}

void vidc_720p_decode_get_seq_hdr_info(struct vidc_720p_seq_hdr_info_type
					*p_seq_hdr_info)
{
	VIDC_IO_IN(REG_999267, &p_seq_hdr_info->n_img_size_x);

	VIDC_IO_IN(REG_345712, &p_seq_hdr_info->n_img_size_y);

	VIDC_IO_IN(REG_257463, &p_seq_hdr_info->n_min_num_dpb);

	VIDC_IO_IN(REG_854281, &p_seq_hdr_info->n_min_dpb_size);

	VIDC_IO_IN(REG_580603, &p_seq_hdr_info->n_dec_frm_size);

	VIDC_IO_INF(REG_606447, DISP_PIC_PROFILE,
				 &p_seq_hdr_info->n_profile);

	VIDC_IO_INF(REG_606447, DIS_PIC_LEVEL,
				 &p_seq_hdr_info->n_level);

	VIDC_IO_INF(REG_612715, DISPLAY_STATUS,
		     &p_seq_hdr_info->n_progressive);
	p_seq_hdr_info->n_progressive =
		((p_seq_hdr_info->n_progressive & 0x4) >> 2);
	/* Read the MPEG4 data partitioning indication */
	VIDC_IO_INF(REG_441270, DATA_PARTITIONED, \
				&p_seq_hdr_info->n_data_partitioned);

}

void vidc_720p_decode_set_dpb_release_buffer_mask(u32
						   i_dpb_release_buffer_mask)
{
	VIDC_IO_OUT(REG_603032, i_dpb_release_buffer_mask);
}

void vidc_720p_decode_set_dpb_buffers(u32 i_buf_index, u32 *pi_dpb_buffer)
{
	VIDC_IO_OUTI(REG_615716, i_buf_index, pi_dpb_buffer);
}

void vidc_720p_decode_set_comv_buffer(u32 *pi_dpb_comv_buffer,
				       u32 n_alloc_len)
{
	VIDC_IO_OUT(REG_456376_ADDR, pi_dpb_comv_buffer);

	VIDC_IO_OUT(REG_490443, n_alloc_len);
}

void vidc_720p_decode_set_dpb_details(u32 n_num_dpb, u32 n_alloc_len,
				       u32 *p_ref_buffer)
{
	VIDC_IO_OUT(REG_518133, p_ref_buffer);

	VIDC_IO_OUT(REG_267567, 0);

	VIDC_IO_OUT(REG_883500, n_num_dpb);

	VIDC_IO_OUT(REG_319934, n_alloc_len);
}

void vidc_720p_decode_set_mpeg4Post_filter(u32 b_enable_post_filter)
{
	if (b_enable_post_filter)
		VIDC_IO_OUT(REG_443811, 0x1);
	else
		VIDC_IO_OUT(REG_443811, 0x0);
}

void vidc_720p_decode_set_error_control(u32 b_enable_error_control)
{
	if (b_enable_error_control)
		VIDC_IO_OUT(REG_846346, 0);
	else
		VIDC_IO_OUT(REG_846346, 1);
}

void vidc_720p_set_deblock_line_buffer(u32 *pi_deblock_line_buffer_start,
					u32 n_alloc_len)
{
	VIDC_IO_OUT(REG_979942, pi_deblock_line_buffer_start);

	VIDC_IO_OUT(REG_101184, n_alloc_len);
}

void vidc_720p_decode_set_mpeg4_data_partitionbuffer(u32 *p_vsp_buf_start)
{
    VIDC_IO_OUT(REG_958768, p_vsp_buf_start);
}

void vidc_720p_decode_setH264VSPBuffer(u32 *pi_vsp_temp_buffer_start)
{
	VIDC_IO_OUT(REG_958768, pi_vsp_temp_buffer_start);
}

void vidc_720p_decode_frame(u32 n_ch_id, u32 n_ext_buffer_start,
			     u32 n_ext_buffer_end, u32 n_dec_unit_size,
			     u32 n_start_byte_num, u32 n_input_frame_tag)
{
	VIDC_IO_OUT(REG_275113_ADDR, n_ext_buffer_start);

	VIDC_IO_OUT(REG_988007_ADDR, n_ext_buffer_end);

	VIDC_IO_OUT(REG_87912, n_ext_buffer_end);

	VIDC_IO_OUT(REG_66693, n_start_byte_num);

	VIDC_IO_OUT(REG_94750, n_input_frame_tag);

	VIDC_IO_OUT(REG_761892, n_dec_unit_size);

	vidc_720p_submit_command(n_ch_id, VIDC_720P_CMD_FRAMERUN);
}

void vidc_720p_issue_eos(u32 i_ch_id)
{
    VIDC_IO_OUT(REG_896825, 0x1);

    VIDC_IO_OUT(REG_761892, 0);

    vidc_720p_submit_command(i_ch_id, VIDC_720P_CMD_FRAMERUN);
}

void vidc_720p_eos_info(u32 *p_disp_status)
{
   VIDC_IO_INF(REG_612715, DISPLAY_STATUS, p_disp_status);
   (*p_disp_status) = (*p_disp_status) & 0x3;
}

void vidc_720p_decode_display_info(struct vidc_720p_dec_disp_info_type
				    *p_disp_info)
{
	u32 display_status = 0;
	VIDC_IO_INF(REG_612715, DISPLAY_STATUS, &display_status);

	p_disp_info->e_disp_status =
	    (enum vidc_720p_display_status_type)((display_status & 0x3));

	p_disp_info->n_disp_is_interlace = ((display_status & 0x4) >> 2);
	p_disp_info->n_crop_exists = ((display_status & 0x8) >> 3);

	p_disp_info->n_resl_change = ((display_status & 0x30) >> 4);

	VIDC_IO_INF(REG_724381, RESOLUTION_CHANGE,
		     &p_disp_info->n_reconfig_flush_done);

	VIDC_IO_IN(REG_999267, &p_disp_info->n_img_size_x);

	VIDC_IO_IN(REG_345712, &p_disp_info->n_img_size_y);
	VIDC_IO_IN(REG_151345, &p_disp_info->n_y_addr);
	VIDC_IO_IN(REG_293983, &p_disp_info->n_c_addr);
	VIDC_IO_IN(REG_370409, &p_disp_info->n_tag_top);
	VIDC_IO_IN(REG_438677, &p_disp_info->n_tag_bottom);
	VIDC_IO_IN(REG_679165, &p_disp_info->n_pic_time_top);
	VIDC_IO_IN(REG_374150, &p_disp_info->n_pic_time_bottom);

	VIDC_IO_INF(REG_881638, CROP_RIGHT_OFFSET,
		     &p_disp_info->n_crop_right_offset);
	VIDC_IO_INF(REG_881638, CROP_LEFT_OFFSET,
		     &p_disp_info->n_crop_left_offset);
	VIDC_IO_INF(REG_161486, CROP_BOTTOM_OFFSET,
		     &p_disp_info->n_crop_bottom_offset);
	VIDC_IO_INF(REG_161486, CROP_TOP_OFFSET,
		     &p_disp_info->n_crop_top_offset);
	VIDC_IO_IN(REG_613254, &p_disp_info->n_metadata_exists);

	VIDC_IO_IN(REG_580603,
		    &p_disp_info->n_input_bytes_consumed);

	VIDC_IO_IN(REG_757835, &p_disp_info->n_input_frame_num);

	VIDC_IO_INF(REG_441270, FRAME_TYPE,
			   &p_disp_info->n_input_frame_type);

	p_disp_info->n_input_is_interlace =
	    ((p_disp_info->n_input_frame_type & 0x4) >> 2);

	p_disp_info->n_input_frame_type &= 0x3;
}

void vidc_720p_decode_skip_frm_details(u32 *p_free_luma_dpb)
{
	u32 n_disp_frm_type;
	VIDC_IO_IN(REG_697961, &n_disp_frm_type);

	if (VIDC_720p_NOTCODED == n_disp_frm_type)
		VIDC_IO_IN(REG_347105, p_free_luma_dpb);
}

void vidc_720p_metadata_enable(u32 n_flag, u32 *p_input_buffer)
{
	VIDC_IO_OUT(REG_854681, n_flag);
	VIDC_IO_OUT(REG_988552, p_input_buffer);
}

void vidc_720p_decode_dynamic_req_reset(void)
{
	VIDC_IO_OUTF(REG_76706, DPB_FLUSH, 0x0);
	VIDC_IO_OUTF(REG_147682, PUT_EXTRADATA, 0x0);
	VIDC_IO_OUT(REG_896825, 0x0);
}

void vidc_720p_decode_dynamic_req_set(u32 n_property)
{
	if (VIDC_720p_FLUSH_REQ == n_property)
		VIDC_IO_OUTF(REG_76706, DPB_FLUSH, 0x1);
	else if (VIDC_720p_EXTRADATA == n_property)
		VIDC_IO_OUTF(REG_147682, PUT_EXTRADATA, 0x1);
}

void vidc_720p_decode_setpassthrough_start(u32 n_pass_startaddr)
{
	VIDC_IO_OUT(REG_486169, n_pass_startaddr);
}

void vidc_720p_RCFrame_skip(u32 n_op, u32 n_vbv_size)
{
	if (!n_op) {
		VIDC_IO_OUTF(REG_128234, FRAME_SKIP_ENABLE,
			      n_op);
		return;
	}
	VIDC_IO_OUTF(REG_128234, FRAME_SKIP_ENABLE, n_op);
	if (n_op == 2) {
		VIDC_IO_OUTF(REG_128234, VBV_BUFFER_SIZE,
			      n_vbv_size);
	}
}
