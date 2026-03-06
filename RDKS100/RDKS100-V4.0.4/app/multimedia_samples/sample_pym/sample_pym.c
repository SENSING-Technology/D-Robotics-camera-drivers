/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright(C) 2024, D-Robotics Co., Ltd.
 *                     All rights reserved.
 ***************************************************************************/

#include <stdio.h>
#include <getopt.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>

#include "common_utils.h"

typedef struct scaler_info
{
	hbn_vflow_handle_t vflow_fd;
	hbn_vnode_handle_t vnode_fd;
	uint32_t input_width;
	uint32_t input_height;
	char* yuv_file;
	int pym_vnode_mode;
} scaler_info_s;

static int verbose_flag = 0;

static struct option const long_options[] = {
	{"input_file", required_argument, NULL, 'i'},
	{"input_width", required_argument, NULL, 'w'},
	{"input_height", required_argument, NULL, 'h'},
	{"feedback", no_argument, NULL, 'f'},
	{"verbose", no_argument, NULL, 'V'},
	{NULL, 0, NULL, 0}
};

int read_nv12_image(scaler_info_s *scaler_info, hbn_vnode_image_t *input_image);
int run_pym(scaler_info_s *scaler_info, hbn_vnode_image_t *input_image);
int create_start_pym_vnode(scaler_info_s *scaler_info);
int stop_destroy_pym_vnode(scaler_info_s *scaler_info);

static void print_help() {
	printf("Usage: sample_pym [OPTIONS]\n");
	printf("Options:\n");
	printf("-i, --input_file FILE\tSpecify the input file\n");
	printf("-w, --input_width WIDTH\tSpecify the input width\n");
	printf("-h, --input_height HEIGHT\tSpecify the input height\n");
	printf("-f, --feedback \t\t\tSpecify feedback mode\n");
	printf("-V, --verbose\t\tEnable verbose mode\n");
}

int main(int argc, char** argv) {
	hbn_vnode_image_t input_image = {0};
	int ret = 0;
	scaler_info_s scaler_info = {0};
	int opt_index = 0;
	int c = 0;

	// 检查标准输入是否来自终端
	if ((!isatty(fileno(stdin))) || (argc == 1)) {
		print_help();
		return 0;
	}

	while((c = getopt_long(argc, argv, "i:w:h:Vf",
							long_options, &opt_index)) != -1) {
		switch (c)
		{
			case 'i':
				scaler_info.yuv_file = optarg;
				break;
			case 'w':
				scaler_info.input_width = atoi(optarg);
				break;
			case 'h':
				scaler_info.input_height = atoi(optarg);
				break;
			case 'f':
				scaler_info.pym_vnode_mode = VNODE_WORK_MODE_FEEDBACK;
				break;
			case 'V':
				verbose_flag = 1;
				break;
			default:
				print_help();
				return 0;
		}
	}
	printf("pym vnode work mode: %s\n", (scaler_info.pym_vnode_mode == VNODE_WORK_MODE_VFLOW)?"vflow":"feedback");
	printf("Using input file:%s, input:%dx%d\n",
			scaler_info.yuv_file,
			scaler_info.input_width, scaler_info.input_height);

	ret = hb_mem_module_open();
	ERR_CON_EQ(ret, 0);
	ret = read_nv12_image(&scaler_info, &input_image);
	ERR_CON_EQ(ret, 0);
	ret = create_start_pym_vnode(&scaler_info);
	ERR_CON_EQ(ret, 0);
	ret = run_pym(&scaler_info, &input_image);
	ERR_CON_EQ(ret, 0);
	ret = stop_destroy_pym_vnode(&scaler_info);
	ERR_CON_EQ(ret, 0);
	ret = hb_mem_free_buf(input_image.buffer.fd[0]);
	hb_mem_module_close();
	return 0;
}

int read_nv12_image(scaler_info_s *scaler_info, hbn_vnode_image_t *input_image) {
	int64_t alloc_flags = 0;
	int ret = 0;
	// uint64_t offset = 0;
	char *input_image_path = scaler_info->yuv_file;
	memset(input_image, 0, sizeof(hbn_vnode_image_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED |
				HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
				HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN |
				HB_MEM_USAGE_CACHED |
				HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	ret = hb_mem_alloc_graph_buf(scaler_info->input_width,
								scaler_info->input_height,
								MEM_PIX_FMT_NV12,
								alloc_flags,
								scaler_info->input_width,
								scaler_info->input_height,
								&input_image->buffer);
	ERR_CON_EQ(ret, 0);
	read_yuvv_nv12_file(input_image_path,
					(char *)(input_image->buffer.virt_addr[0]),
					(char *)(input_image->buffer.virt_addr[1]),
					input_image->buffer.size[0]);

	// 设置一个时间戳
	gettimeofday(&input_image->info.tv, NULL);

	return ret;
}

void vp_vin_print_hbn_frame_info_t(const hbn_frame_info_t *frame_info);
void vp_vin_print_hb_mem_graphic_buf_t(const hb_mem_graphic_buf_t *graphic_buf);

// 打印 hbn_vnode_image_t 结构体的所有字段内容
void vp_vin_print_hbn_vnode_image_t(const hbn_vnode_image_t *frame)
{
	printf("=== Frame Info ===\n");
	vp_vin_print_hbn_frame_info_t(&(frame->info));
	printf("\n=== Graphic Buffer ===\n");
	vp_vin_print_hb_mem_graphic_buf_t(&(frame->buffer));
}

// 打印 hbn_frame_info_t 结构体的所有字段内容
void vp_vin_print_hbn_frame_info_t(const hbn_frame_info_t *frame_info) {
	printf("Frame ID: %u\n", frame_info->frame_id);
	printf("Timestamps: %lu\n", frame_info->timestamps);
	printf("tv: %ld.%06ld\n", frame_info->tv.tv_sec, frame_info->tv.tv_usec);
	printf("trig_tv: %ld.%06ld\n", frame_info->trig_tv.tv_sec, frame_info->trig_tv.tv_usec);
	printf("Frame Done: %u\n", frame_info->frame_done);
	printf("Buffer Index: %d\n", frame_info->bufferindex);
}

// 打印 hb_mem_graphic_buf_t 结构体的所有字段内容
void vp_vin_print_hb_mem_graphic_buf_t(const hb_mem_graphic_buf_t *graphic_buf) {
	printf("File Descriptors: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%d ", graphic_buf->fd[i]);
	}
	printf("\n");

	printf("Plane Count: %d\n", graphic_buf->plane_cnt);
	printf("Format: %d\n", graphic_buf->format);
	printf("Width: %d\n", graphic_buf->width);
	printf("Height: %d\n", graphic_buf->height);
	printf("Stride: %d\n", graphic_buf->stride);
	printf("Vertical Stride: %d\n", graphic_buf->vstride);
	printf("Is Contiguous: %d\n", graphic_buf->is_contig);

	printf("Share IDs: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%d ", graphic_buf->share_id[i]);
	}
	printf("\n");

	printf("Flags: %ld\n", graphic_buf->flags);

	printf("Sizes: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%lu ", graphic_buf->size[i]);
	}
	printf("\n");

	printf("Virtual Addresses: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%p ", graphic_buf->virt_addr[i]);
	}
	printf("\n");

	printf("Physical Addresses: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%lu ", graphic_buf->phys_addr[i]);
	}
	printf("\n");

	printf("Offsets: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%lu ", graphic_buf->offset[i]);
	}
	printf("\n");
}

int run_pym(scaler_info_s *scaler_info, hbn_vnode_image_t *input_image) {
	int ret;
	int timeout = 1000;
	hbn_vnode_image_group_t out_img_group = {0};
	char output_image_path[128] = {0};

	if (verbose_flag) {
		vp_vin_print_hbn_vnode_image_t(input_image);
	}

	// 发送帧并获取输出图像
	ret = hbn_vnode_sendframe(scaler_info->vnode_fd, 0, input_image);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_getframe_group(scaler_info->vnode_fd, 0, timeout, &out_img_group);
	ERR_CON_EQ(ret, 0);

	// 循环处理每个输出通道
	for (int chn_id = 0; chn_id < MAX_DS_NUM; chn_id++) {
		hbn_frame_info_t *info = &out_img_group.info;
		hb_mem_graphic_buf_t* out_img = &out_img_group.buf_group.graph_group[chn_id];

		// 根据当前输出通道属性设置输出文件路径
		snprintf(output_image_path, sizeof(output_image_path),
			"./pym_output_nv12_chn%d_%dx%d_stride_%d.yuv",
			chn_id, out_img->width, out_img->height,
			out_img->stride);
		// 保存输出图像到文件
		dump_2plane_yuv_to_file(output_image_path,
						out_img->virt_addr[0],
						out_img->virt_addr[1],
						out_img->size[0],
						out_img->size[1]);
	}
	hbn_vnode_releaseframe_group(scaler_info->vnode_fd, 0, &out_img_group);

	return 0;
}

int create_start_pym_vnode(scaler_info_s *scaler_info) {
	int ret = 0;
	pym_cfg_t pym_cfg = {0};
	pym_cfg.hw_id = 0;
	pym_cfg.pym_mode = 3; 			//离线模式
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

	pym_cfg.chn_ctrl.src_in_width = scaler_info->input_width;
	pym_cfg.chn_ctrl.src_in_height = scaler_info->input_height;
	pym_cfg.chn_ctrl.src_in_stride_y = ALIGN_16(scaler_info->input_width); //16字节对齐
	pym_cfg.chn_ctrl.src_in_stride_uv = ALIGN_16(scaler_info->input_width);//16字节对齐

	printf("\npym config:\n");
	printf("	ichn input width = %d, height = %d\n", scaler_info->input_width, scaler_info->input_height);
	int ratio = 1;
	for(int i = 0; i < MAX_DS_NUM; i++){

		//限制1：SRC和BL缩小倍数固定，并且长和宽缩小比例相同
		ratio = 1 << i;// 缩小倍数：1(Src层)，2(BL0)，4(BL1)，8(BL2)，16(BL3)，32(BL4)

		//限制2：SRC和BL的输出最小为 32*32
		int bl_width = FLOOR_ALIGN_2(scaler_info->input_width / ratio);
		int bl_height = FLOOR_ALIGN_2(scaler_info->input_height / ratio);
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

		roi_box->wstride_uv = ALIGN_16(roi_box->out_width); //输出(uv分量)宽的stride: 16字节对齐
		roi_box->wstride_y = ALIGN_16(roi_box->out_width);  //输出(y分量) 宽的stride: 16字节对齐
		roi_box->vstride = roi_box->out_height;			    //输出高的Stride, 不需要对齐

		printf("	ochn[%d] ratio= %d, width = %d, height = %d wstride=%d vstride=%d out[%d*%d]\n",
			i, ratio, roi_box->region_width, roi_box->region_height,
			roi_box->wstride_uv, roi_box->vstride, roi_box->out_width, roi_box->out_height);
	}
	printf("\n");

	ret = hbn_vnode_open(HB_PYM, pym_cfg.hw_id, AUTO_ALLOC_ID, &scaler_info->vnode_fd);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_attr(scaler_info->vnode_fd, &pym_cfg);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ichn_attr(scaler_info->vnode_fd, 0, &pym_cfg);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ochn_attr(scaler_info->vnode_fd, 0, &pym_cfg);
	ERR_CON_EQ(ret, 0);

	if(pym_cfg.output_buf_num > 0){
		hbn_buf_alloc_attr_t alloc_attr;
		memset(&alloc_attr, 0, sizeof(hbn_buf_alloc_attr_t));
		alloc_attr.buffers_num = pym_cfg.output_buf_num;
		alloc_attr.is_contig = 1;
		alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN | HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
		if (pym_cfg.out_buf_noncached == 0u) {
			alloc_attr.flags |= (uint64_t)HB_MEM_USAGE_CACHED;
		}
		ret = hbn_vnode_set_ochn_buf_attr(scaler_info->vnode_fd, 0, &alloc_attr);
		ERR_CON_EQ(ret, 0);
	}

	switch (scaler_info->pym_vnode_mode) {
		case VNODE_WORK_MODE_VFLOW:
			ret = hbn_vflow_create(&scaler_info->vflow_fd);
			ERR_CON_EQ(ret, 0);
			ret = hbn_vflow_add_vnode(scaler_info->vflow_fd, scaler_info->vnode_fd);
			ERR_CON_EQ(ret, 0);
			ret = hbn_vflow_start(scaler_info->vflow_fd);
			ERR_CON_EQ(ret, 0);
			break;
		case VNODE_WORK_MODE_FEEDBACK:
			ret = hbn_vnode_start(scaler_info->vnode_fd);
			ERR_CON_EQ(ret, 0);
			break;
		default:
			printf("Unknow pym vnode work mode[%d]\n", scaler_info->pym_vnode_mode);
			break;
	}

	return ret;
}

int stop_destroy_pym_vnode(scaler_info_s *scaler_info) {
	int ret;
	switch (scaler_info->pym_vnode_mode) {
		case VNODE_WORK_MODE_VFLOW:
			ret = hbn_vflow_stop(scaler_info->vflow_fd);
			ERR_CON_EQ(ret, 0);
			hbn_vnode_close(scaler_info->vnode_fd);
			hbn_vflow_destroy(scaler_info->vflow_fd);
			break;
		case VNODE_WORK_MODE_FEEDBACK:
			ret = hbn_vnode_stop(scaler_info->vnode_fd);
			ERR_CON_EQ(ret, 0);
			hbn_vnode_close(scaler_info->vnode_fd);
			break;
		default:
			printf("Unknow pym vnode work mode[%d]\n", scaler_info->pym_vnode_mode);
			break;
	}
	return 0;
}
