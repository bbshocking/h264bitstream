/* 
 * h264bitstream - a library for reading and writing H.264 video
 * Copyright (C) 2005-2007 Auroras Entertainment, LLC
 * Copyright (C) 2008-2011 Avail-TVN
 * 
 * Written by Alex Izvorski <aizvorski@gmail.com> and Alex Giladi <alex.giladi@gmail.com>
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _H264_STREAM_H
#define _H264_STREAM_H        1

#include <stdint.h>
#include <stdio.h>
#include <assert.h>

#include "bs.h"
#include "h264_sei.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
   Sequence Parameter Set
   @see 7.3.2.1 Sequence parameter set RBSP syntax
   @see h264_read_seq_parameter_set_rbsp
   @see h264_write_seq_parameter_set_rbsp
   @see h264_debug_sps
*/
typedef struct
{
    int profile_idc;
    int constraint_set0_flag;
    int constraint_set1_flag;
    int constraint_set2_flag;
    int constraint_set3_flag;
    int constraint_set4_flag;
    int constraint_set5_flag;
    int reserved_zero_2bits;
    int level_idc;
    int seq_parameter_set_id;
    int chroma_format_idc;
    int residual_colour_transform_flag;
    int bit_depth_luma_minus8;
    int bit_depth_chroma_minus8;
    int qpprime_y_zero_transform_bypass_flag;
    int seq_scaling_matrix_present_flag;
      int seq_scaling_list_present_flag[8];
      int* ScalingList4x4[6];
      int UseDefaultScalingMatrix4x4Flag[6];
      int* ScalingList8x8[2];
      int UseDefaultScalingMatrix8x8Flag[2];
    int log2_max_frame_num_minus4;
    int pic_order_cnt_type;
      int log2_max_pic_order_cnt_lsb_minus4;
      int delta_pic_order_always_zero_flag;
      int offset_for_non_ref_pic;
      int offset_for_top_to_bottom_field;
      int num_ref_frames_in_pic_order_cnt_cycle;
      int offset_for_ref_frame[256];
    int num_ref_frames;
    int gaps_in_frame_num_value_allowed_flag;
    int pic_width_in_mbs_minus1;
    int pic_height_in_map_units_minus1;
    int frame_mbs_only_flag;
    int mb_adaptive_frame_field_flag;
    int direct_8x8_inference_flag;
    int frame_cropping_flag;
      int frame_crop_left_offset;
      int frame_crop_right_offset;
      int frame_crop_top_offset;
      int frame_crop_bottom_offset;
    int vui_parameters_present_flag;
    
    struct
    {
        int aspect_ratio_info_present_flag;
          int aspect_ratio_idc;
            int sar_width;
            int sar_height;
        int overscan_info_present_flag;
          int overscan_appropriate_flag;
        int video_signal_type_present_flag;
          int video_format;
          int video_full_range_flag;
          int colour_description_present_flag;
            int colour_primaries;
            int transfer_characteristics;
            int matrix_coefficients;
        int chroma_loc_info_present_flag;
          int chroma_sample_loc_type_top_field;
          int chroma_sample_loc_type_bottom_field;
        int timing_info_present_flag;
          int num_units_in_tick;
          int time_scale;
          int fixed_frame_rate_flag;
        int nal_hrd_parameters_present_flag;
        int vcl_hrd_parameters_present_flag;
          int low_delay_hrd_flag;
        int pic_struct_present_flag;
        int bitstream_restriction_flag;
          int motion_vectors_over_pic_boundaries_flag;
          int max_bytes_per_pic_denom;
          int max_bits_per_mb_denom;
          int log2_max_mv_length_horizontal;
          int log2_max_mv_length_vertical;
          int num_reorder_frames;
          int max_dec_frame_buffering;
    } vui;

    struct
    {
        int cpb_cnt_minus1;
        int bit_rate_scale;
        int cpb_size_scale;
          int bit_rate_value_minus1[32]; // up to cpb_cnt_minus1, which is <= 31
          int cpb_size_value_minus1[32];
          int cbr_flag[32];
        int initial_cpb_removal_delay_length_minus1;
        int cpb_removal_delay_length_minus1;
        int dpb_output_delay_length_minus1;
        int time_offset_length;
    } hrd;

} sps_t;


/**
   Picture Parameter Set
   @see 7.3.2.2 Picture parameter set RBSP syntax
   @see h264_read_pic_parameter_set_rbsp
   @see h264_write_pic_parameter_set_rbsp
   @see h264_debug_pps
*/
typedef struct 
{
    int pic_parameter_set_id;
    int seq_parameter_set_id;
    int entropy_coding_mode_flag;
    int pic_order_present_flag;
    int num_slice_groups_minus1;
    int slice_group_map_type;
      int run_length_minus1[8]; // up to num_slice_groups_minus1, which is <= 7 in Baseline and Extended, 0 otheriwse
      int top_left[8];
      int bottom_right[8];
      int slice_group_change_direction_flag;
      int slice_group_change_rate_minus1;
      int pic_size_in_map_units_minus1;
      int slice_group_id[256]; // FIXME what size?
    int num_ref_idx_l0_active_minus1;
    int num_ref_idx_l1_active_minus1;
    int weighted_pred_flag;
    int weighted_bipred_idc;
    int pic_init_qp_minus26;
    int pic_init_qs_minus26;
    int chroma_qp_index_offset;
    int deblocking_filter_control_present_flag;
    int constrained_intra_pred_flag;
    int redundant_pic_cnt_present_flag;

    // set iff we carry any of the optional headers
    int _more_rbsp_data_present;

    int transform_8x8_mode_flag;
    int pic_scaling_matrix_present_flag;
       int pic_scaling_list_present_flag[8];
       int* ScalingList4x4[6];
       int UseDefaultScalingMatrix4x4Flag[6];
       int* ScalingList8x8[2];
       int UseDefaultScalingMatrix8x8Flag[2];
    int second_chroma_qp_index_offset;
} pps_t;


/**
  Slice Header
  @see 7.3.3 Slice header syntax
  @see read_slice_header_rbsp
  @see write_slice_header_rbsp
  @see debug_slice_header_rbsp
*/
typedef struct
{
    int first_mb_in_slice;
    int slice_type;
    int pic_parameter_set_id;
    int frame_num;
    int field_pic_flag;
      int bottom_field_flag;
    int idr_pic_id;
    int pic_order_cnt_lsb;
    int delta_pic_order_cnt_bottom;
    int delta_pic_order_cnt[ 2 ];
    int redundant_pic_cnt;
    int direct_spatial_mv_pred_flag;
    int num_ref_idx_active_override_flag;
    int num_ref_idx_l0_active_minus1;
    int num_ref_idx_l1_active_minus1;
    int cabac_init_idc;
    int slice_qp_delta;
    int sp_for_switch_flag;
    int slice_qs_delta;
    int disable_deblocking_filter_idc;
    int slice_alpha_c0_offset_div2;
    int slice_beta_offset_div2;
    int slice_group_change_cycle;


    struct
    {
        int luma_log2_weight_denom;
        int chroma_log2_weight_denom;
        int luma_weight_l0_flag[64];
        int luma_weight_l0[64];
        int luma_offset_l0[64];
        int chroma_weight_l0_flag[64];
        int chroma_weight_l0[64][2];
        int chroma_offset_l0[64][2];
        int luma_weight_l1_flag[64];
        int luma_weight_l1[64];
        int luma_offset_l1[64];
        int chroma_weight_l1_flag[64];
        int chroma_weight_l1[64][2];
        int chroma_offset_l1[64][2];
    } pwt; // predictive weight table

    struct // FIXME stack or array
    {
        int ref_pic_list_reordering_flag_l0;
        int ref_pic_list_reordering_flag_l1;
        int reordering_of_pic_nums_idc;
        int abs_diff_pic_num_minus1;
        int long_term_pic_num;
    } rplr; // ref pic list reorder

    struct // FIXME stack or array
    {
        int no_output_of_prior_pics_flag;
        int long_term_reference_flag;
        int adaptive_ref_pic_marking_mode_flag;
        int memory_management_control_operation;
        int difference_of_pic_nums_minus1;
        int long_term_pic_num;
        int long_term_frame_idx;
        int max_long_term_frame_idx_plus1;
    } drpm; // decoded ref pic marking

} slice_header_t;


/**
   Access unit delimiter
   @see 7.3.1 NAL unit syntax
   @see h264_read_nal_unit
   @see h264_write_nal_unit
   @see h264_debug_nal
*/
typedef struct
{
    int primary_pic_type;
} aud_t;

/**
   Network Abstraction Layer (NAL) unit
   @see 7.3.1 NAL unit syntax
   @see h264_read_nal_unit
   @see h264_write_nal_unit
   @see h264_debug_nal
*/
typedef struct
{
    int forbidden_zero_bit;
    int nal_ref_idc;
    int nal_unit_type;
    void* parsed; // FIXME
    int sizeof_parsed;

    //uint8_t* rbsp_buf;
    //int rbsp_size;
} nal_t;

typedef struct
{
    int _is_initialized;
    int sps_id;
    int initial_cpb_removal_delay;
    int initial_cpb_delay_offset;
} sei_buffering_t;

typedef struct
{
    int clock_timestamp_flag;
        int ct_type;
        int nuit_field_based_flag;
        int counting_type;
        int full_timestamp_flag;
        int discontinuity_flag;
        int cnt_dropped_flag;
        int n_frames;

        int seconds_value;
        int minutes_value;
        int hours_value;

        int seconds_flag;
        int minutes_flag;
        int hours_flag;

        int time_offset;
} picture_timestamp_t;

typedef struct
{
  int _is_initialized;
  int cpb_removal_delay;
  int dpb_output_delay;
  int pic_struct;
  picture_timestamp_t clock_timestamps[3]; // 3 is the maximum possible value
} sei_picture_timing_t;


typedef struct
{
  int rbsp_size;
  uint8_t* rbsp_buf;
} slice_data_rbsp_t;

/**
   H264 stream
   Contains data structures for all NAL types that can be handled by this library.  
   When reading, data is read into those, and when writing it is written from those.  
   The reason why they are all contained in one place is that some of them depend on others, we need to 
   have all of them available to read or write correctly.
 */
typedef struct
{
    nal_t* nal;
    sps_t* sps;
    pps_t* pps;
    aud_t* aud;
    sei_t* sei; //This is a TEMP pointer at whats in h->seis...    
    int num_seis;
    slice_header_t* sh;
    slice_data_rbsp_t* slice_data;

    sps_t* sps_table[32];
    pps_t* pps_table[256];
    sei_t** seis;

} h264_stream_t;

h264_stream_t* h264_new();
void h264_free(h264_stream_t* h);

int h264_find_nal_unit(uint8_t* buf, int size, int* nal_start, int* nal_end);

int h264_rbsp_to_nal(const uint8_t* rbsp_buf, const int* rbsp_size, uint8_t* nal_buf, int* nal_size);
int h264_nal_to_rbsp(const uint8_t* nal_buf, int* nal_size, uint8_t* rbsp_buf, int* rbsp_size);

int h264_read_nal_unit(h264_stream_t* h, uint8_t* buf, int size);
int h264_peek_nal_unit(h264_stream_t* h, uint8_t* buf, int size);

void h264_read_seq_parameter_set_rbsp(h264_stream_t* h, bs_t* b);
void h264_read_scaling_list(bs_t* b, int* scalingList, int sizeOfScalingList, int useDefaultScalingMatrixFlag );
void h264_read_vui_parameters(h264_stream_t* h, bs_t* b);
void h264_read_hrd_parameters(h264_stream_t* h, bs_t* b);

void h264_read_pic_parameter_set_rbsp(h264_stream_t* h, bs_t* b);

void h264_read_sei_rbsp(h264_stream_t* h, bs_t* b);
void h264_read_sei_message(h264_stream_t* h, bs_t* b);
void h264_read_access_unit_delimiter_rbsp(h264_stream_t* h, bs_t* b);
void h264_read_end_of_seq_rbsp(h264_stream_t* h, bs_t* b);
void h264_read_end_of_stream_rbsp(h264_stream_t* h, bs_t* b);
void h264_read_filler_data_rbsp(h264_stream_t* h, bs_t* b);

void h264_read_slice_layer_rbsp(h264_stream_t* h, bs_t* b);
void h264_read_rbsp_slice_trailing_bits(h264_stream_t* h, bs_t* b);
void h264_read_rbsp_trailing_bits(h264_stream_t* h, bs_t* b);
void h264_read_slice_header(h264_stream_t* h, bs_t* b);
void h264_read_ref_pic_list_reordering(h264_stream_t* h, bs_t* b);
void h264_read_pred_weight_table(h264_stream_t* h, bs_t* b);
void h264_read_dec_ref_pic_marking(h264_stream_t* h, bs_t* b);

int h264_more_rbsp_trailing_data(h264_stream_t* h, bs_t* b);

int h264_write_nal_unit(h264_stream_t* h, uint8_t* buf, int size);

void h264_write_seq_parameter_set_rbsp(h264_stream_t* h, bs_t* b);
void h264_write_scaling_list(bs_t* b, int* scalingList, int sizeOfScalingList, int useDefaultScalingMatrixFlag );
void h264_write_vui_parameters(h264_stream_t* h, bs_t* b);
void h264_write_hrd_parameters(h264_stream_t* h, bs_t* b);

void h264_write_pic_parameter_set_rbsp(h264_stream_t* h, bs_t* b);

void h264_write_sei_rbsp(h264_stream_t* h, bs_t* b);
void h264_write_sei_message(h264_stream_t* h, bs_t* b);
void h264_write_access_unit_delimiter_rbsp(h264_stream_t* h, bs_t* b);
void h264_write_end_of_seq_rbsp(h264_stream_t* h, bs_t* b);
void h264_write_end_of_stream_rbsp(h264_stream_t* h, bs_t* b);
void h264_write_filler_data_rbsp(h264_stream_t* h, bs_t* b);

void h264_write_slice_layer_rbsp(h264_stream_t* h, bs_t* b);
void h264_write_rbsp_slice_trailing_bits(h264_stream_t* h, bs_t* b);
void h264_write_rbsp_trailing_bits(h264_stream_t* h, bs_t* b);
void h264_write_slice_header(h264_stream_t* h, bs_t* b);
void h264_write_ref_pic_list_reordering(h264_stream_t* h, bs_t* b);
void h264_write_pred_weight_table(h264_stream_t* h, bs_t* b);
void h264_write_dec_ref_pic_marking(h264_stream_t* h, bs_t* b);

void h264_debug_sps(sps_t* sps);
void h264_debug_pps(pps_t* pps);
void h264_debug_slice_header(slice_header_t* sh);
void h264_debug_nal(h264_stream_t* h, nal_t* nal);

void h264_debug_bytes(uint8_t* buf, int len);

void h264_read_sei_payload( h264_stream_t* h, bs_t* b, int payloadType, int payloadSize);
void h264_write_sei_payload( h264_stream_t* h, bs_t* b, int payloadType, int payloadSize);

//NAL ref idc codes
typedef enum {
    NAL_REF_IDC_PRIORITY_HIGHEST     = 3,
    NAL_REF_IDC_PRIORITY_HIGH        = 2,
    NAL_REF_IDC_PRIORITY_LOW         = 1, 
    NAL_REF_IDC_PRIORITY_DISPOSABLE  = 0
} nal_ref_idc_t;

//Table 7-1 NAL unit type codes
typedef enum {
    NAL_UNIT_TYPE_UNSPECIFIED                  = 0,        // Unspecified
    NAL_UNIT_TYPE_CODED_SLICE_NON_IDR          = 1,        // Coded slice of a non-IDR picture
    NAL_UNIT_TYPE_CODED_SLICE_DATA_PARTITION_A = 2,        // Coded slice data partition A
    NAL_UNIT_TYPE_CODED_SLICE_DATA_PARTITION_B = 3,        // Coded slice data partition B
    NAL_UNIT_TYPE_CODED_SLICE_DATA_PARTITION_C = 4,        // Coded slice data partition C
    NAL_UNIT_TYPE_CODED_SLICE_IDR              = 5,        // Coded slice of an IDR picture
    NAL_UNIT_TYPE_SEI                          = 6,        // Supplemental enhancement information (SEI)
    NAL_UNIT_TYPE_SPS                          = 7,        // Sequence parameter set
    NAL_UNIT_TYPE_PPS                          = 8,        // Picture parameter set
    NAL_UNIT_TYPE_AUD                          = 9,        // Access unit delimiter
    NAL_UNIT_TYPE_END_OF_SEQUENCE              = 10,       // End of sequence
    NAL_UNIT_TYPE_END_OF_STREAM                = 11,       // End of stream
    NAL_UNIT_TYPE_FILLER                       = 12,       // Filler data
    NAL_UNIT_TYPE_SPS_EXT                      = 13,       // Sequence parameter set extension
                                               // 14..18    // Reserved
    NAL_UNIT_TYPE_CODED_SLICE_AUX              = 19,       // Coded slice of an auxiliary coded picture without partitioning
                                               // 20..23    // Reserved
                                               // 24..31    // Unspecified
} nal_unit_type_t;

//7.4.3 Table 7-6. Name association to slice_type
typedef enum {
    SH_SLICE_TYPE_P       = 0,        // P (P slice)
    SH_SLICE_TYPE_B       = 1,        // B (B slice)
    SH_SLICE_TYPE_I       = 2,        // I (I slice)
    SH_SLICE_TYPE_SP      = 3,        // SP (SP slice)
    SH_SLICE_TYPE_SI      = 4,        // SI (SI slice)
    //as per footnote to Table 7-6, the *_ONLY slice types indicate that all other slices in that picture are of the same type
    SH_SLICE_TYPE_P_ONLY  = 5,        // P (P slice)
    SH_SLICE_TYPE_B_ONLY  = 6,        // B (B slice)
    SH_SLICE_TYPE_I_ONLY  = 7,        // I (I slice)
    SH_SLICE_TYPE_SP_ONLY = 8,        // SP (SP slice)
    SH_SLICE_TYPE_SI_ONLY = 9         // SI (SI slice)
} sh_slice_type_t;

//Appendix E. Table E-1  Meaning of sample aspect ratio indicator
typedef enum {
    SAR_Unspecified = 0,             // Unspecified
    SAR_1_1         = 1,             // 1:1
    SAR_12_11       = 2,             // 12:11
    SAR_10_11       = 3,             // 10:11
    SAR_16_11       = 4,             // 16:11
    SAR_40_33       = 5,             // 40:33
    SAR_24_11       = 6,             // 24:11
    SAR_20_11       = 7,             // 20:11
    SAR_32_11       = 8,             // 32:11
    SAR_80_33       = 9,             // 80:33
    SAR_18_11       = 10,            // 18:11
    SAR_15_11       = 11,            // 15:11
    SAR_64_33       = 12,            // 64:33
    SAR_160_99      = 13,            // 160:99
                   // 14..254        // Reserved
    SAR_Extended    = 255            // Extended_SARe)
} sar_t;

//7.4.3.1 Table 7-7 reordering_of_pic_nums_idc operations for reordering of reference picture lists
typedef enum {
    RPLR_IDC_ABS_DIFF_ADD       = 0,
    RPLR_IDC_ABS_DIFF_SUBTRACT  = 1,
    RPLR_IDC_LONG_TERM          = 2,
    RPLR_IDC_END                = 3,
} rplr_idc_t;

//7.4.3.3 Table 7-9 Memory management control operation (memory_management_control_operation) values
typedef enum {
    MMCO_END                     = 0,
    MMCO_SHORT_TERM_UNUSED       = 1,
    MMCO_LONG_TERM_UNUSED        = 2,
    MMCO_SHORT_TERM_TO_LONG_TERM = 3,
    MMCO_LONG_TERM_MAX_INDEX     = 4,
    MMCO_ALL_UNUSED              = 5,
    MMCO_CURRENT_TO_LONG_TERM    = 6
} mmco_t;

//7.4.2.4 Table 7-5 Meaning of primary_pic_type
typedef enum {
    AUD_PRIMARY_PIC_TYPE_I       = 0,          // I
    AUD_PRIMARY_PIC_TYPE_IP      = 1,          // I, P
    AUD_PRIMARY_PIC_TYPE_IPB     = 2,          // I, P, B
    AUD_PRIMARY_PIC_TYPE_SI      = 3,          // SI
    AUD_PRIMARY_PIC_TYPE_SISP    = 4,          // SI, SP
    AUD_PRIMARY_PIC_TYPE_ISI     = 5,          // I, SI
    AUD_PRIMARY_PIC_TYPE_ISIPSP  = 6,          // I, SI, P, SP
    AUD_PRIMARY_PIC_TYPE_ISIPSPB = 7           // I, SI, P, SP, B
} aud_primary_pic_type_t;

typedef enum {
    H264_PROFILE_BASELINE = 66,
    H264_PROFILE_MAIN     = 77,
    H264_PROFILE_EXTENDED = 88,
    H264_PROFILE_HIGH     = 100
} h264_profile_t;

// file handle for debug output
extern FILE* h264_dbgfile;

#ifdef __cplusplus
}
#endif

#endif
