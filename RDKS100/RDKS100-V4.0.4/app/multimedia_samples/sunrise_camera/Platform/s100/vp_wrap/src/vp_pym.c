#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "utils/utils_log.h"

#include "vp_wrap.h"
#include "vp_pym.h"
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
#define ALIGN_2(x)  (((x) + 1) & ~1)	 //  2 字节向上对齐（x=3 → 4）
#define UPPER_ALIGN_16(x)  (((x) + 15) & ~15)  // 16 字节向上对齐（x=17 → 32）
#define FLOOR_ALIGN_2(x)  ((x) & ~1)	 //  2 字节向下对齐（x=3 → 2）


int32_t vp_pym_init(vp_vflow_contex_t *vp_vflow_contex)
{
	int ret = 0;
	int input_width = 0;
	int input_height = 0;
	int pym_select_chn = 0;
	{
		camera_config_t *camera_config = vp_vflow_contex->sensor_config->camera_config;
		input_width = camera_config->width;
		input_height = camera_config->height;
	}

	pym_cfg_t pym_cfg = {0};
	vp_flow_info_t *vp_flow_info = &vp_vflow_contex->vp_flow_info;
	pym_cfg.hw_id =  vp_flow_info->pym_hw_id;
	pym_cfg.pym_mode = vp_flow_info->pym_mode;
	pym_cfg.slot_id = vp_flow_info->pym_slot_id;

	pym_cfg.output_buf_num = 3; 	//输出buffer的个数
	pym_cfg.fb_buf_num = 2;			//回灌buffer的个数
	pym_cfg.layer_num_trans_next = 0;
	pym_cfg.layer_num_share_prev = -1;
	pym_cfg.out_buf_noinvalid = 1;
	pym_cfg.out_buf_noncached = 0;
	pym_cfg.in_buf_noclean = 1;
	pym_cfg.in_buf_noncached = 0;
	pym_cfg.magicNumber = MAGIC_NUMBER;

	pym_cfg.chn_ctrl.pixel_num_before_sol = DEF_PIX_NUM_BF_SOL;
	pym_cfg.chn_ctrl.suffix_hb_val = DEF_SUFFIX_HB;
	pym_cfg.chn_ctrl.prefix_hb_val = DEF_PREFIX_HB;
	pym_cfg.chn_ctrl.suffix_vb_val = DEF_SUFFIX_VB;
	pym_cfg.chn_ctrl.prefix_vb_val = DEF_PREFIX_VB;

	pym_cfg.chn_ctrl.bl_max_layer_en = DEF_BL_MAX_EN;

	pym_cfg.chn_ctrl.src_in_width = input_width;
	pym_cfg.chn_ctrl.src_in_height = input_height;
	pym_cfg.chn_ctrl.src_in_stride_y = UPPER_ALIGN_16(input_width); //16字节对齐
	pym_cfg.chn_ctrl.src_in_stride_uv = UPPER_ALIGN_16(input_width);//16字节对齐

	printf("\npym config:\n");
	printf("	ichn input width = %d, height = %d\n", input_width, input_height);
	int ratio = 1;
	for(int i = 0; i < MAX_DS_NUM; i++){
		if(i != pym_select_chn){
			continue;
		}

		//限制1：SRC和BL缩小倍数固定，并且长和宽缩小比例相同
		ratio = 1 << i;// 缩小倍数：1(Src层)，2(BL0)，4(BL1)，8(BL2)，16(BL3)，32(BL4)

		//限制2：SRC和BL的输出最小为 32*32
		int bl_width = FLOOR_ALIGN_2(input_width / ratio);
		int bl_height = FLOOR_ALIGN_2(input_height / ratio);
		if(bl_width < PYM_MIN_WIDTH){
			printf("[Error] ochn[%d] ratio= %d, width = %d < PYM_MIN_WIDTH(%d), so not enable.\n",
				i, ratio, bl_width, PYM_MIN_WIDTH);
			continue;
		}
		if(bl_height < PYM_MIN_HEIGHT){
			printf("[Error] ochn[%d] ratio= %d, height = %d < PYM_MIN_HEIGHT(%d), so not enable.\n",
				i, ratio, bl_height, PYM_MIN_HEIGHT);
			continue;
		}

		pym_cfg.chn_ctrl.ds_roi_sel[i] = (i == 0) ? 0 : 1; 				//注意：src层 为0，bl(0-4)层为1
		pym_cfg.chn_ctrl.ds_roi_layer[i] = (i == 0) ? 0 : (i - 1); 		//注意：src层 和 bl0层 的 ds_roi_layer 都是0
		pym_cfg.chn_ctrl.ds_roi_en |= 1 << i; //全部通道使能的情况(5个 BL + 1个SRC)：0b111111;

		roi_box_t* roi_box = &pym_cfg.chn_ctrl.ds_roi_info[i];
		roi_box->start_left = 0;
		roi_box->start_top = 0;
		roi_box->region_width = bl_width; //向下对齐，防止超过BL层大小
		roi_box->region_height = bl_height;//向下对齐，防止超过BL层大小

		//限制3: DS输出的缩小倍率(1/2, 1], 如下是直接输出BL的结果
		roi_box->out_width = FLOOR_ALIGN_2(roi_box->region_width);
		roi_box->out_height = FLOOR_ALIGN_2(roi_box->region_height);

		roi_box->wstride_uv = UPPER_ALIGN_16(roi_box->out_width); //输出(uv分量)宽的stride: 16字节对齐
		roi_box->wstride_y = UPPER_ALIGN_16(roi_box->out_width);  //输出(y分量) 宽的stride: 16字节对齐
		roi_box->vstride = roi_box->out_height;			    //输出高的Stride, 不需要对齐

		//保存每个通道的输出，为下个模块使用
		node_out_info_t* node_out_info = &vp_vflow_contex->pym_out_info[i];
		node_out_info->width = roi_box->out_width;
		node_out_info->height = roi_box->out_height;

		printf("	ochn[%d] ratio= %d, width = %d, height = %d wstride=%d vstride=%d out[%d*%d]\n",
			i, ratio, roi_box->region_width, roi_box->region_height,
			roi_box->wstride_uv, roi_box->vstride, roi_box->out_width, roi_box->out_height);
	}
	printf("\n");
	hbn_vnode_handle_t *pym_node_handle = &vp_vflow_contex->pym_node_handle;
	ret = hbn_vnode_open(HB_PYM, pym_cfg.hw_id, AUTO_ALLOC_ID, pym_node_handle);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_open");

	ret = hbn_vnode_set_attr(*pym_node_handle, &pym_cfg);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_attr");

	ret = hbn_vnode_set_ichn_attr(*pym_node_handle, 0, &pym_cfg);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ichn_attr");

	ret = hbn_vnode_set_ochn_attr(*pym_node_handle, 0, &pym_cfg);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_attr");

	if(pym_cfg.output_buf_num > 0){
		hbn_buf_alloc_attr_t alloc_attr;
		memset(&alloc_attr, 0, sizeof(hbn_buf_alloc_attr_t));
		alloc_attr.buffers_num = pym_cfg.output_buf_num;
		alloc_attr.is_contig = 1;
		alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN | HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
		if (pym_cfg.out_buf_noncached == 0u) {
			alloc_attr.flags |= (uint64_t)HB_MEM_USAGE_CACHED;
		}
		ret = hbn_vnode_set_ochn_buf_attr(*pym_node_handle, 0, &alloc_attr);
		SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_buf_attr");
	}
	return 0;
}

int32_t vp_pym_deinit(vp_vflow_contex_t *vp_vflow_contex)
{
	hbn_vnode_close(vp_vflow_contex->pym_node_handle);
	return 0;
}

int32_t vp_pym_start(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;
	return ret;
}

int32_t vp_pym_stop(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;
	return ret;
}

int32_t vp_pym_get_frame(vp_vflow_contex_t *vp_vflow_contex, ImageFrame *frame)
{	int32_t ret = 0;
	hbn_vnode_handle_t pym_node_handle = vp_vflow_contex->pym_node_handle;
	uint32_t chn_id = 0;
	ret = hbn_vnode_getframe_group(pym_node_handle, chn_id, 1000, frame->hbn_vnode_image_group);
	if (ret != 0) {
		// SC_LOGE("hbn_vnode_getframe %d PYM failed\n", chn_id);
	}else{
		frame->hbn_vnode_image->buffer = frame->hbn_vnode_image_group->buf_group.graph_group[0];
		frame->hbn_vnode_image->info = frame->hbn_vnode_image_group->info;
	}

	return ret;
}

int32_t vp_pym_release_frame(vp_vflow_contex_t *vp_vflow_contex, ImageFrame *frame)
{
	int32_t ret = 0;
	uint32_t chn_id = 0;
	hbn_vnode_handle_t pym_node_handle = vp_vflow_contex->pym_node_handle;

	ret = hbn_vnode_releaseframe_group(pym_node_handle, chn_id, frame->hbn_vnode_image_group);

	return ret;
}
