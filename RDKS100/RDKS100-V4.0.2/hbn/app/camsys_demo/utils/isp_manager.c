#include "isp_manager.h"

typedef struct {
    int                         mipi_rx0_cam_count;
    pipeline_isp_info_t        mipi_rx0_cams[MAX_CAMERA_COUNT_FROM_MIPI];

    int                         mipi_rx1_cam_count;
    pipeline_isp_info_t        mipi_rx1_cams[MAX_CAMERA_COUNT_FROM_MIPI];

    int                         mipi_rx4_cam_count;
    pipeline_isp_info_t        mipi_rx4_cams[MAX_CAMERA_COUNT_FROM_MIPI];
} total_camera_config_t;

typedef struct{
	isp_channel_t	rx0[MAX_CAMERA_COUNT_FROM_MIPI];
	isp_channel_t	rx1[MAX_CAMERA_COUNT_FROM_MIPI];
	isp_channel_t	rx2[MAX_CAMERA_COUNT_FROM_MIPI];
}total_isp_channel_t;

static isp_manager_id_t g_id = 0;
static total_isp_channel_t g_total_isp_channel;
static total_camera_config_t g_total_camera_config;

int isp_manager_do_distribute(){
	return 0;
}
isp_manager_id_t isp_manager_add_camera(const int mipi_rx, const pipeline_isp_info_t *isp_info){
	isp_manager_id_t id;
	return id;
}

int isp_manager_get_isp_channel_config(const isp_manager_id_t id, isp_channel_t *isp_channel){
	return 0;
}
