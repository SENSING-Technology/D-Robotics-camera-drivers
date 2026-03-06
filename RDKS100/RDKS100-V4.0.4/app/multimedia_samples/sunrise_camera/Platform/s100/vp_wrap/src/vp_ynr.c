#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "utils/utils_log.h"

#include "vp_wrap.h"
#include "vp_ynr.h"

int32_t vp_ynr_init(vp_vflow_contex_t *vp_vflow_contex)
{
	int ret = 0;
	struct ynr_init_attr *attr;
	vp_sensor_config_t *sensor_config = NULL;
	hbn_vnode_handle_t *ynr_node_handle = NULL;
	vp_flow_info_t *vp_flow_info = &vp_vflow_contex->vp_flow_info;

	ynr_node_handle = &vp_vflow_contex->ynr_node_handle;
	sensor_config = vp_vflow_contex->sensor_config;
	attr = sensor_config->ynr_attr;

	attr->work_mode = vp_flow_info->ynr_mode;
	attr->slot_id = vp_flow_info->ynr_slot_id;

	int hw_id = 1; //固定为1
	ret = hbn_vnode_open(HB_YNR, hw_id, AUTO_ALLOC_ID, ynr_node_handle);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_open");

	ret = hbn_vnode_set_attr(*ynr_node_handle, attr);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_attr");

	struct hobot_ynr_channel_input_config channel_input_cfg = {0};
	ret = hbn_vnode_set_ichn_attr(*ynr_node_handle, 0, &channel_input_cfg);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ichn_attr");

	ret = hbn_vnode_set_ichn_attr(*ynr_node_handle, 1, &channel_input_cfg);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ichn_attr");

	struct hobot_ynr_channel_output_config channel_output_cfg = {0};
	ret = hbn_vnode_set_ochn_attr(*ynr_node_handle, 0, &channel_output_cfg);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_attr");

	if (attr->nr3d_en == 1u) {
		hbn_buf_alloc_attr_t alloc_attr;
		alloc_attr.buffers_num = 3;
		alloc_attr.is_contig = 1;
		alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN |
			(uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN | (uint64_t)HB_MEM_USAGE_CACHED);
		ret = hbn_vnode_set_ochn_buf_attr(*ynr_node_handle, 0, &alloc_attr);
		SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_buf_attr");
	}
	return 0;
}

int32_t vp_ynr_deinit(vp_vflow_contex_t *vp_vflow_contex)
{
	hbn_vnode_close(vp_vflow_contex->ynr_node_handle);
	return 0;
}
int32_t vp_ynr_start(vp_vflow_contex_t *vp_vflow_contex){
	return 0;
}
int32_t vp_ynr_stop(vp_vflow_contex_t *vp_vflow_contex){
	return 0;
}
