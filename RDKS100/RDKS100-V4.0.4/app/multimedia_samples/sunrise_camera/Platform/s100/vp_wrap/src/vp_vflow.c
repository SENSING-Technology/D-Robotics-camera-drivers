/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2024 D-Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-02-23 14:01:59
 * @LastEditTime: 2023-03-05 15:57:48
 ***************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "utils/utils_log.h"

#include "vp_wrap.h"
#include "vp_vflow.h"

int32_t vp_vflow_init(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;
	vp_flow_info_t *vp_flow_info = &vp_vflow_contex->vp_flow_info;

	// 创建HBN flow
	ret = hbn_vflow_create(&vp_vflow_contex->vflow_fd);
	SC_ERR_CON_EQ(ret, 0, "hbn_vflow_create");
	ret = hbn_vflow_add_vnode(vp_vflow_contex->vflow_fd,
							vp_vflow_contex->vin_node_handle);
	SC_ERR_CON_EQ(ret, 0, "hbn_vflow_add_vnode");


	if(vp_flow_info->type == VP_FLOW_ISP_ONLY){
		ret = hbn_vflow_add_vnode(vp_vflow_contex->vflow_fd,
							vp_vflow_contex->isp_node_handle);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_add_vnode");
	}else if(vp_flow_info->type == VP_FLOW_ISP_YNR){
		ret = hbn_vflow_add_vnode(vp_vflow_contex->vflow_fd,
							vp_vflow_contex->isp_node_handle);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_add_vnode");
		ret = hbn_vflow_add_vnode(vp_vflow_contex->vflow_fd,
								vp_vflow_contex->ynr_node_handle);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_add_vnode");
	}else{
		//do nothing
	}
	ret = hbn_vflow_add_vnode(vp_vflow_contex->vflow_fd,
							vp_vflow_contex->pym_node_handle);
	SC_ERR_CON_EQ(ret, 0, "hbn_vflow_add_vnode");

	if(vp_vflow_contex->gdc_info.gdc_fd){
		ret = hbn_vflow_add_vnode(vp_vflow_contex->vflow_fd,
								vp_vflow_contex->gdc_info.gdc_fd);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_add_gdc_vnode");
	}

	SC_LOGD("successful");
	return 0;
}

int32_t vp_vflow_deinit(vp_vflow_contex_t *vp_vflow_contex)
{
	hbn_vflow_destroy(vp_vflow_contex->vflow_fd);
	return 0;
}

int32_t vp_vflow_start(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;
	int sensor_type = vp_vflow_contex->sensor_config->sensor_type;

	vp_flow_info_t *vp_flow_info = &vp_vflow_contex->vp_flow_info;
	if(vp_flow_info->type == VP_FLOW_ISP_BYPASS){
		ret = hbn_vflow_bind_vnode(vp_vflow_contex->vflow_fd,
								vp_vflow_contex->vin_node_handle,
								vp_flow_info->is_online_vin_pym,
								vp_vflow_contex->pym_node_handle,
								0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_bind_vnode");
	}else if(vp_flow_info->type == VP_FLOW_ISP_ONLY){
		ret = hbn_vflow_bind_vnode(vp_vflow_contex->vflow_fd,
								vp_vflow_contex->vin_node_handle,
								vp_flow_info->is_online_vin_isp,
								vp_vflow_contex->isp_node_handle,
								0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_bind_vnode");

		ret = hbn_vflow_bind_vnode(vp_vflow_contex->vflow_fd,
						vp_vflow_contex->isp_node_handle,
						vp_flow_info->is_online_isp_pym,
						vp_vflow_contex->pym_node_handle,
						0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_bind_vnode");

	}else if(vp_flow_info->type == VP_FLOW_ISP_YNR){
		ret = hbn_vflow_bind_vnode(vp_vflow_contex->vflow_fd,
								vp_vflow_contex->vin_node_handle,
								vp_flow_info->is_online_vin_isp,
								vp_vflow_contex->isp_node_handle,
								0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_bind_vnode");

		ret = hbn_vflow_bind_vnode(vp_vflow_contex->vflow_fd,
						vp_vflow_contex->isp_node_handle,
						vp_flow_info->is_online_isp_ynr,
						vp_vflow_contex->ynr_node_handle,
						0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_bind_vnode");

		ret = hbn_vflow_bind_vnode(vp_vflow_contex->vflow_fd,
						vp_vflow_contex->ynr_node_handle,
						vp_flow_info->is_online_ynr_pym,
						vp_vflow_contex->pym_node_handle,
						0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_bind_vnode");
	}else{
		//error
	}

	if(vp_vflow_contex->gdc_info.gdc_fd){
		ret = hbn_vflow_bind_vnode(vp_vflow_contex->vflow_fd,
									vp_vflow_contex->pym_node_handle,
									0,
									vp_vflow_contex->gdc_info.gdc_fd,
									0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_bind_vnode: isp->gdc");
	}

	if(sensor_type == SENSOR_TYPE_NORMAL){
		ret = hbn_camera_attach_to_vin(vp_vflow_contex->cam_fd,
							vp_vflow_contex->vin_node_handle);
		SC_ERR_CON_EQ(ret, 0, "hbn_camera_attach_to_vin");
	}

	ret = hbn_vflow_start(vp_vflow_contex->vflow_fd);
	SC_ERR_CON_EQ(ret, 0, "hbn_vflow_start");

	SC_LOGD("successful");
	return ret;
}

int32_t vp_vflow_stop(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;
	ret = hbn_vflow_stop(vp_vflow_contex->vflow_fd);
	SC_ERR_CON_EQ(ret, 0, "hbn_vflow_stop");

	vp_flow_info_t *vp_flow_info = &vp_vflow_contex->vp_flow_info;
		if(vp_flow_info->type == VP_FLOW_ISP_BYPASS){
		ret = hbn_vflow_unbind_vnode(vp_vflow_contex->vflow_fd,
								vp_vflow_contex->vin_node_handle,
								vp_flow_info->is_online_vin_pym,
								vp_vflow_contex->pym_node_handle,
								0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_unbind_vnode");
	}else if(vp_flow_info->type == VP_FLOW_ISP_ONLY){
		ret = hbn_vflow_unbind_vnode(vp_vflow_contex->vflow_fd,
								vp_vflow_contex->vin_node_handle,
								vp_flow_info->is_online_vin_isp,
								vp_vflow_contex->isp_node_handle,
								0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_unbind_vnode");

		ret = hbn_vflow_unbind_vnode(vp_vflow_contex->vflow_fd,
						vp_vflow_contex->isp_node_handle,
						vp_flow_info->is_online_isp_pym,
						vp_vflow_contex->pym_node_handle,
						0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_unbind_vnode");

	}else if(vp_flow_info->type == VP_FLOW_ISP_YNR){
		ret = hbn_vflow_unbind_vnode(vp_vflow_contex->vflow_fd,
								vp_vflow_contex->vin_node_handle,
								vp_flow_info->is_online_vin_isp,
								vp_vflow_contex->isp_node_handle,
								0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_unbind_vnode");

		ret = hbn_vflow_unbind_vnode(vp_vflow_contex->vflow_fd,
						vp_vflow_contex->isp_node_handle,
						vp_flow_info->is_online_isp_ynr,
						vp_vflow_contex->ynr_node_handle,
						0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_unbind_vnode");

		ret = hbn_vflow_unbind_vnode(vp_vflow_contex->vflow_fd,
						vp_vflow_contex->ynr_node_handle,
						vp_flow_info->is_online_ynr_pym,
						vp_vflow_contex->pym_node_handle,
						0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_unbind_vnode");
	}else{
		//error
	}

	if(vp_vflow_contex->gdc_info.gdc_fd){
		ret = hbn_vflow_unbind_vnode(vp_vflow_contex->vflow_fd,
									vp_vflow_contex->pym_node_handle,
									0,
									vp_vflow_contex->gdc_info.gdc_fd,
									0);
		SC_ERR_CON_EQ(ret, 0, "hbn_vflow_unbind_vnode: isp->gdc");
	}
	SC_LOGD("successful");
	return 0;
}
