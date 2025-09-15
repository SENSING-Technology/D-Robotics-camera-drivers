/**
 * @file: ynr_cfg.h
 * @
 * @NO{S090E01C01I}
 * @ASIL{B}
 * @Copyright (c) 2024 by horizon, All Rights Reserved.
 */
/**
 * @file: hbn_ynr_cfg.h
 * @
 * @NO{S090E01C01I}
 * @ASIL{B}
 * @Copyright (c) 2023 by horizon, All Rights Reserved.
 */
#ifndef __HBN_YNR_CFG_INTERFACE_H__
#define __HBN_YNR_CFG_INTERFACE_H__

#define YNR_CFG_ENABLE 1u
#define YNR_CFG_DISABLE 0u
#define YNR_CFG_HW_ID 1u
#define MAX_LINK_MODE_MAX 3u
#define MAX_YNR_HBNK_MAX 255u
#define MAX_YNR_VBNK_MIN 1u
#define MAX_YNR_VBNK_MAX 255u
#define MAX_TIME_OUT_MAX 0xFFFFFFFF
#define MAX_SLOT_ID_MAX 11u
#define MAX_IMG_WIDTH_MIN 8u
#define MAX_IMG_WIDTH_MAX 4096u
#define MAX_IMG_WIDTH_STEP 2u
#define MAX_IMG_HEIGHT_MIN 8u
#define MAX_IMG_HEIGHT_MAX 4096u
#define MAX_IMG_HEIGHT_STEP 2u
#define MAX_NR3D_PIX_OUT_DMA_BYPS_MAX 1u
#define MAX_NR3D_DEBUG_EN_MAX 1u

#include "hbn_ynr_cfg.h"

/**
 * @struct ynr_init_attr
 * @brief YNR initialization configuration information.
 * @NO{S09E00C02}
 */
struct ynr_init_attr {
	/**
	 * @var ynr_init_attr::work_mode
	 * Operating mode.
	 * range:N/A; default: N/A
	 */
	uint32_t work_mode;
	/**
	 * @var ynr_init_attr::slot_id
	 * ISP slot ID.
	 * range:N/A; default: N/A
	 */
	uint32_t slot_id;
	/**
	 * @var ynr_init_attr::width
	 * Image width.
	 * range:N/A; default: N/A
	 */
	uint32_t width;
	/**
	 * @var ynr_init_attr::height
	 * Image height.
	 * range:N/A; default: N/A
	 */
	uint32_t height;
	uint32_t nr_static_switch;
	/**
	 * @var ynr_init_attr::in_stride
	 * y and uv strides.
	 * range:N/A; default: N/A
	 */
	uint32_t in_stride[2];	//just for hbn y\uv
	/**
	 * @var ynr_init_attr::nr2d_en
	 * 2D noise reduction switch.
	 * range:N/A; default: N/A
	 */
	uint32_t nr2d_en;
	/**
	 * @var ynr_init_attr::nr2d_en
	 * 3D noise reduction switch.
	 * range:N/A; default: N/A
	 */
	uint32_t nr3d_en;
	uint32_t dma_output_en;
	uint32_t debug_en;
};


int32_t ynr_node_parser_config(void *ynr, ynr_info_t *info);
#endif
