#ifndef __ISP_MANAGER__
#define __ISP_MANAGER__

#ifdef __cplusplus
extern "C" {
#endif	/* __cplusplus */
#include "isp_cfg.h"

#define MAX_CAMERA_COUNT_FROM_MIPI 4

typedef struct{
	int width;
	int height;
	int fps;
	int is_enable_ynr;
	int is_vin_online_isp;
}pipeline_isp_info_t;

typedef int isp_manager_id_t;

int isp_manager_do_distribute();
/**
 * @brief 向ISP管理器注册一个新的Camera设备并关联MIPI接口
 *
 * 该函数用于添加 每个Sensor的配置信息到ISP资源分配器中，ISP资源分配器根据Sensor的情况进行分配ISP资源
 *
 * @param[in] mipi_rx    MIPI RX硬件接口编号（取值：0/1/4）
 * @param[in] isp_info   描述和ISP资源分配相关的参数
 *
 * @return  成功时返回分配的Camera唯一标识符（ID），失败返回 -1
 */
isp_manager_id_t isp_manager_add_camera(const int mipi_rx, const pipeline_isp_info_t *isp_info);

int isp_manager_get_isp_channel_config(const isp_manager_id_t id, isp_channel_t *isp_channel);
void isp_manager_show_isp_channel_config();
#ifdef __cplusplus
	}
#endif	/* __cplusplus */

#endif