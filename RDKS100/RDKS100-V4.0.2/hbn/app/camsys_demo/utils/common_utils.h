/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright(C) 2024, D-Robotics Co., Ltd.
 *                     All rights reserved.
 ***************************************************************************/

#ifndef __COMMMON_UTILS__
#define __COMMMON_UTILS__
#include "hbn_vpf_interface.h"
#include "hb_camera_interface.h"
#include "hbn_vpf_data_info.h"
#include "hb_media_codec.h"
#include "hb_media_error.h"
#include "hbn_pym_cfg.h"
#include "hbn_error.h"
#include "vp_sensors.h"

#ifdef __cplusplus
extern "C" {
#endif	/* __cplusplus */

#define ERR_CON_EQ(ret, a) do {\
		if ((ret) != (a)) {\
			printf("%s(%d) failed, ret %d\n", __func__, __LINE__, (int32_t)(ret));\
			return (ret);\
		}\
	} while(0)\

#define ERR_CON_NE(ret, a) do {\
		if ((ret) == (a)) {\
			printf("%s(%d) failed, ret %ld\n", __func__, __LINE__, (ret));\
			return (ret);\
		}\
	} while(0)\

typedef enum {
	VNODE_WORK_MODE_VFLOW = 0,
	VNODE_WORK_MODE_FEEDBACK,
}vnode_work_mode_e;

typedef struct camera_config_info_s{
	int width;
	int height;
	int fps;
	media_codec_id_t encode_type;
}camera_config_info_t;

typedef struct {
	int vin_isp_is_online;

}vflow_connect_config_t;

typedef struct pipe_contex_s {
	hbn_vflow_handle_t vflow_fd;
	hbn_vnode_handle_t vin_node_handle;
	hbn_vnode_handle_t isp_node_handle;
	hbn_vnode_handle_t pym_node_handle;
	hbn_vnode_handle_t ynr_node_handle;
	hbn_vnode_handle_t gdc_node_handle;
	hbn_vnode_handle_t vpu_node_handle;
	hbn_vnode_handle_t codec_node_handle;

	camera_handle_t cam_fd;
	deserial_handle_t des_fd;

	hb_mem_common_buf_t bin_buf;

	//config
	vp_csi_config_t csi_config;
	vp_sensor_config_t *sensor_config;
	// vflow_connect_config_t vflow_connect_config;
} pipe_contex_t;

//for pym
#define DEF_BL_MAX_EN 5

#define PYM_MIN_HEIGHT 32u
#define PYM_MIN_WIDTH 32u
#define PYM_MAX_HEIGHT 4096u
#define PYM_MAX_WIDTH 4096u

#define DEF_SUFFIX_HB 100
#define DEF_PREFIX_HB 2
#define DEF_SUFFIX_VB 10
#define DEF_PREFIX_VB 0

#define DEF_PIX_NUM_BF_SOL 2

int32_t read_yuv420_file(const char *filename, char *addr0, char *addr1, uint32_t y_size);
int32_t read_yuvv_nv12_file(const char *filename, char *addr0, char *addr1, uint32_t y_size);
int32_t dump_image_to_file(char *filename, uint8_t *src_buffer, uint32_t size);
int32_t dump_2plane_yuv_to_file(char *filename, uint8_t *src_buffer, uint8_t *src_buffer1,
		uint32_t size, uint32_t size1);
int32_t vpm_hb_mem_init(void);
void vpm_hb_mem_deinit(void);
int32_t alloc_graphic_buffer(hbn_vnode_image_t *img, uint32_t width,
			     uint32_t height, uint32_t cached, int32_t format);
uint64_t vio_test_gettime_us(void);
uint32_t load_file_2_buff_nosize(const char *path, char *filebuff);
char* get_program_name();
void configure_vse_max_resolution(int32_t channel, uint32_t input_width, uint32_t input_height,
	uint32_t *output_width, uint32_t *output_height);

int read_nv12_image_to_graphic_buffer(const char *file_path, hb_mem_graphic_buf_t *src_buf, int width, int height);
int read_nv12_image_to_common_buffer(const char *file_path, hb_mem_common_buf_t *src_buf, int width, int height);
int read_nv12_image_to_normal_memory(const char *file_path, uint8_t*virt_addr, int width, int height);
///////////////////////////////////////////// COMMON /////////////////////////////////////////////////////////
typedef struct{
	int mipi_rx;
	int isp_sched_mode;
	isp_channel_t isp_channel;
}channel_and_sched_info_t;
void get_channel_and_sched_info(const pipe_contex_t *pipe_contex, channel_and_sched_info_t *ch_sched_info);
///////////////////////////////////////////// ISP /////////////////////////////////////////////////////////
int get_isp_channel_config_for_single_pipeline(const int mipi_rx,
	const int is_online, isp_channel_t *isp_channel);

///////////////////////////////////////////// PYM /////////////////////////////////////////////////////////
#define ALIGN_2(x)  (((x) + 1) & ~1)	 //  2 字节向上对齐（x=3 → 4）
#define ALIGN_16(x)  (((x) + 15) & ~15)  // 16 字节向上对齐（x=17 → 32）
#define FLOOR_ALIGN_2(x)  ((x) & ~1)	 //  2 字节向下对齐（x=3 → 2）


typedef struct{
	int hw_id;
	int slot_id;
	int sched_mode;
}pym_channel_t;
int get_pym_channel_config_for_single_pipeline(const channel_and_sched_info_t *pym_dependency_info,
	const int is_pym_online, pym_channel_t *pym_channel);

///////////////////////////////////////////// YNR /////////////////////////////////////////////////////////
typedef pym_channel_t ynr_channel_t;
int get_ynr_channel_config_for_single_pipeline(const channel_and_sched_info_t *pym_dependency_info, ynr_channel_t *pym_channel);

#ifdef __cplusplus
	}
#endif	/* __cplusplus */

#endif

