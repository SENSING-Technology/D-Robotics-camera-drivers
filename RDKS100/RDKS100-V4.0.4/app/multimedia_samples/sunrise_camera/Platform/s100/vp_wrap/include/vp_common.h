/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2024 D-Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-01-30 11:27:41
 * @LastEditTime: 2023-03-05 14:15:22
 ***************************************************************************/
#ifndef VP_COMMONH_
#define VP_COMMONH_

#include <stdint.h>
#include "hb_deserial_interface.h"
#include "hb_camera_interface.h"
#include "hbn_vpf_interface.h"
#include "hbn_vpf_data_info.h"
#include "hb_media_codec.h"
#include "hbn_pym_cfg.h"
#include "hbn_sth_cfg.h"
#include "hb_media_error.h"
#include "hbn_error.h"
#include "vp_sensors.h"

#define VP_MAX_PATH_LENGTH 128
#define VP_MAX_FILES_NUM 128

#define VP_MAX_JSON_DEPTH 10
#define VP_MAX_FIELD_LENGTH 20
#define VP_MAX_CMD_LENGTH 128
#define VP_MAX_RESULT_LENGTH 1024
#define VP_MAX_SENSOR_NAME 10

#define VP_MAX_PIPELINE_NUM 8
#define VP_PIPELINE_MASK 0xffffff00
#define VP_CODEC_MASK 0x00000000
#define VPP_DISPLAY_MASK 0xfffffffe

#define VP_GET_FRAME_TIMEOUT 2000
#define VP_DECODER_GET_FRAME_TIMEOUT 4000

#define VP_MAX_OSD_REGION (4)

/**
 * Align by 16
 */
#define ALIGN_16(v) ((v + (16 - 1)) / 16 * 16)

#define VP_GET_MD_CODEC_TYPE(v) \
	({ \
		int _v = (v); \
		( \
			_v == 0 ? MEDIA_CODEC_ID_H264 : \
			_v == 1 ? MEDIA_CODEC_ID_H265 : \
			_v == 2 ? MEDIA_CODEC_ID_MJPEG : \
			MEDIA_CODEC_ID_NONE \
		); \
	})

typedef struct {
	int32_t width;
	int32_t height;
	int32_t stride;
	int32_t vstride;

	int64_t frame_id;
	int64_t lost_image_num;
	int64_t exp_time;
	int64_t image_timestamp;

	int32_t plane_count;
	uint8_t *data[3];
	uint64_t pdata[3];
	uint32_t data_size[3];

	// media_codec_buffer_t
	media_codec_buffer_t *frame_buffer;
	// media_codec_output_buffer_info_t
	media_codec_output_buffer_info_t *buffer_info;
	// hbn_vnode_image_t
	hbn_vnode_image_t *hbn_vnode_image;

	hbn_vnode_image_group_t *hbn_vnode_image_group; //for release
} ImageFrame;

//////////////////////////////////////////// user config ////////////////////////////////////////////
typedef struct isp_user_config_s {
	int ochn_buffer_count;
} isp_user_config_t;

typedef struct vin_user_config_s {
	int ochn_buffer_count;
} vin_user_config_t;

enum GDC_STATUS{
    GDC_STATUS_INVALID = -1,
    GDC_STATUS_CLOSE = 0,
    GDC_STATUS_OPEN = 1,
};

typedef struct{
	enum GDC_STATUS status; //0: 没有gdc file， 1： 关闭 gdc, 2： 打开gdc
	char sensor_name[64];
	int bin_buf_is_valid;
	hb_mem_common_buf_t bin_buf;
	hbn_vnode_handle_t gdc_fd;
	int input_width;
	int input_height;

	int output_buffer_count;

}gdc_user_info_t;

////////////////////////////////////////////  ////////////////////////////////////////////

typedef enum {
	VP_FLOW_ISP_BYPASS    = 0,    /** 情景0(不需要ISP): Sensor 输出 YUV数据*/
    VP_FLOW_ISP_ONLY      = 1,    /** 情景1(ISP): 不需要运行YNR的Sensor, 或者宽和高 > 2048的sensor */
 	VP_FLOW_ISP_YNR       = 2,    /** 情景2(ISP + YNR): 需要运行YNR的Sensor, 并且宽和高 <= 2048的sensor */
    VP_FLOW_MAX                   /**< 枚举边界，不可用作实际参数 */
} vp_flow_type_t;

typedef struct {
	vp_flow_type_t type;
	int isp_mode;
	int isp_hw_id;
	int isp_slot_id;

	int ynr_mode;
	int ynr_slot_id;

	int pym_slot_id;
	int pym_hw_id;
	int pym_mode;

	//VP_FLOW_ISP_BYPASS
	int is_online_vin_pym;

	//VP_FLOW_ISP_ONLY
	int is_online_vin_isp;
	int is_online_isp_pym;

	//VP_FLOW_ISP_YNR
	int is_online_isp_ynr;
	int is_online_ynr_pym;
}vp_flow_info_t;
void vp_flow_info_reset();
void vp_flow_info_show(const vp_flow_info_t* vp_flow_info, int vp_flow_index);
void vp_flow_info_init(const vp_sensor_config_t *sensor_config, vp_flow_info_t* vp_flow_info);

//////////////////////////////////////////// vp_vflow_contex_t ////////////////////////////////////////////
typedef struct {
	int width;
	int height;
}node_out_info_t;

typedef struct vp_vflow_contex_s {
	//vflow info
	vp_flow_info_t vp_flow_info;

	//vflow
	hbn_vflow_handle_t vflow_fd;

	//sensor
	camera_handle_t cam_fd;
	deserial_handle_t  des_fd;

	int32_t link_port; 	// for gmsl sensor
	int32_t sensor_addr; // for mipi sensor
	int32_t mipi_csi_rx_index;
	int32_t mclk_is_not_configed;
	vp_sensor_config_t *sensor_config;

	//vin
	vin_user_config_t vin_info;
	hbn_vnode_handle_t vin_node_handle;

	//isp
	isp_user_config_t isp_info;
	hbn_vnode_handle_t isp_node_handle;

	//gdc
	gdc_user_info_t gdc_info;
	hbn_vnode_handle_t gdc_node_handle;

	//ynr
	hbn_vnode_handle_t ynr_node_handle;

	//pym
	hbn_vnode_handle_t pym_node_handle;
	node_out_info_t pym_out_info[MAX_DS_NUM];
} vp_vflow_contex_t;

#endif // VP_COMMONH_