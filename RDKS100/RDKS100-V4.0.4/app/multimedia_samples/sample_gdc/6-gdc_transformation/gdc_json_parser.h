//----------------------------------------------------------------------------
//   The confidential and proprietary information contained in this file may
//   only be used by a person authorised under and to the extent permitted
//   by a subsisting licensing agreement from ARM Limited or its affiliates.
//
//          (C) COPYRIGHT [2020] ARM Limited or its affiliates.
//              ALL RIGHTS RESERVED
//
/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __GDC_PARSER_H__
#define __GDC_PARSER_H__

#include <stdint.h>
#include "hb_gdc_data_info.h"
#include "hbn_vpf_interface.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define ARM_GDC_API __attribute__((visibility("default")))

//---------------------------------------------------------
/**
 * Parse json layout file to parameters and windows structures
 * @param[in]  buf      array with input json text
 * @param[out] param    common structure
 * @param[out] wnds     array of windows
 * @param[out] wnd_cnt  output number of windows
 * @return  0 if success
 */
ARM_GDC_API int32_t gdc_parse_json(const char *buf, param_t *param, window_t **wnds, uint32_t *wnd_cnt);
//---------------------------------------------------------
/**
 * Clean structures after usage
 * @param wnds[in]     array of windows
 * @param wnd_num[in]  output number of windows
 */
ARM_GDC_API void gdc_parse_json_clean(window_t **wnds, uint32_t wnd_num);
int gdc_cfg_bin_gen(const char* layout_file, char* config_file, void** cfg_buf, uint64_t* config_size);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //__GDC_PARSER_H__
