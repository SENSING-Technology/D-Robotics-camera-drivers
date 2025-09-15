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
#include <pthread.h>

#include "common_utils.h"

#define DEBUG

static struct option const long_options[] = {
	{"sensor", required_argument, NULL, 's'},
	{"bin_file", required_argument, NULL, 'f'},
	{"link_port", required_argument, NULL, 'l'},
	{NULL, 0, NULL, 0}
};

static int cim_pym_flyby = 0;//根据VIN决定
static int isp_online_ynr = 1;
static int ynr_online_pym = 1;

static int32_t running = 0;
static uint32_t sensor_type = 0;
static uint32_t link_port = 0;
static uint16_t date_type;
static uint32_t sensor_mode = 0; // 1: NORMAL_M; 2: DOL2_M; 6: SLAVE_M
static media_codec_context_t media_context;

typedef struct gdc_info
{
	char* gdc_bin_file;
	hb_mem_common_buf_t bin_buf;
} gdc_info_s;

int create_and_run_vflow(pipe_contex_t *pipe_contex, gdc_info_s *gdc_info);
void *read_gdc_data(void *contex);
int32_t hbn_deserial_create(deserial_config_t *des_config, deserial_handle_t *des_fd);
int32_t hbn_deserial_attach_to_vin(deserial_handle_t des_fd, camera_des_link_t link, vpf_handle_t vin_fd);

int encode_init(void *data, camera_config_info_t* camera_config_info);
int encode_deinit(void *data);

static void print_help() {
	printf("Usage: single_pipe_vin_isp_ynr_pym_gdc_vpu [OPTIONS]\n");
	printf("Options:\n");
	printf("  -s <sensor_index>      Specify sensor index\n");
	printf("  -l <link_port>         Specify gmsl sensor link_port\n");
	printf("  -f <gdc_bin_file>      Specify sensor gdc_bin_file path\n");
	printf("  -h                     Show this help message\n");
	vp_show_sensors_list();
}

void signal_handle(int signo) {
	running = 0;
}


int main(int argc, char** argv) {
	int ret = 0;
	pipe_contex_t pipe_contex = {0};
	pthread_t read_thread;
	gdc_info_s gdc_info = {0};
	int opt_index = 0;
	int c = 0;
	int index = -1;
	bool link_port_specified = false;

	while((c = getopt_long(argc, argv, "s:f:l:h",
							long_options, &opt_index)) != -1) {
		switch (c)
		{
		case 's':
			index = atoi(optarg);
			break;
		case 'f':
			gdc_info.gdc_bin_file = optarg;
			break;
		case 'l':
			link_port = atoi(optarg);
			link_port_specified = true;
			break;
		case 'h':
		default:
			print_help();
			return 0;
		}
	}

	int used_mipi_host = 0;
	if (index < vp_get_sensors_list_number() && index >= 0) {
		pipe_contex.sensor_config = vp_sensor_config_list[index];
		printf("Using index:%d  sensor_name:%s  config_file:%s\n",
				index,
				vp_sensor_config_list[index]->sensor_name,
				vp_sensor_config_list[index]->config_file);

		sensor_type = pipe_contex.sensor_config->sensor_type;
		date_type = pipe_contex.sensor_config->vin_ichn_attr->format;
		if (sensor_type != SENSOR_TYPE_NORMAL && !link_port_specified) {
			print_help();
			printf("[Error] %s is gsml sensor, must config link port according to the hardware connection, can be set to [0-3]\n",
				vp_sensor_config_list[index]->sensor_name);

			return -1;
		}
		if(sensor_type != SENSOR_TYPE_GMSL_YUV){
			printf("[Error] Only Support gmsl YUV sensor! ,please check!\n");
			return -1;
		}else{
			pipe_contex.csi_config.mipi_rx = 4;
		}
	} else {
		printf("Unsupport sensor index:%d\n", index);
		print_help();
		return 0;
	}
	if (gdc_info.gdc_bin_file == NULL) {
		fprintf(stderr, "Error: Missing argument: -f\n");
		print_help();
		exit(EXIT_FAILURE);
	}
	//初始化GSML的 link_desp
	//不同的sensor和加串要重新map成不一样的地址,不然相同的地址解串器没法和他们通讯,所以根据link_port动态设定eeprom_addr serial_addr
	if (sensor_type != SENSOR_TYPE_NORMAL && link_port_specified) {
		camera_config_t *camera_config = pipe_contex.sensor_config->camera_config;
		vp_deserial_config_update(camera_config, link_port);
		vp_deserial_config_show();
	}

	hb_mem_module_open();
	ret = create_and_run_vflow(&pipe_contex, &gdc_info);
	ERR_CON_EQ(ret, 0);

	camera_config_info_t camera_config_info;
	{
		camera_config_info.width = pipe_contex.sensor_config->camera_config->width;
		camera_config_info.height = pipe_contex.sensor_config->camera_config->height;
		camera_config_info.fps = pipe_contex.sensor_config->camera_config->fps;
		camera_config_info.encode_type = MEDIA_CODEC_ID_H264;
	}

	encode_init(&pipe_contex, &camera_config_info);

	running = 1;
	ret = pthread_create(&read_thread, NULL, (void *)read_gdc_data,
						(void *)&pipe_contex);
	pthread_join(read_thread, NULL);
	ret = hbn_vflow_stop(pipe_contex.vflow_fd);
	ERR_CON_EQ(ret, 0);
	hbn_vnode_close(pipe_contex.gdc_node_handle);
	hbn_vnode_close(pipe_contex.pym_node_handle);
	hbn_vnode_close(pipe_contex.isp_node_handle);
	hbn_vnode_close(pipe_contex.vin_node_handle);
	hbn_camera_destroy(pipe_contex.cam_fd);
	hbn_vflow_destroy(pipe_contex.vflow_fd);
	hb_mem_module_close();
	encode_deinit(&pipe_contex);

	return 0;
}

static int create_camera_node(pipe_contex_t *pipe_contex)
{
	camera_config_t camera_config_copy = {0};
	vp_sensor_config_t *sensor_config = NULL;
	int32_t ret = 0;

	sensor_config = pipe_contex->sensor_config;
	camera_config_copy = *sensor_config->camera_config;

	if (sensor_mode >= NORMAL_M && sensor_mode < INVALID_MOD) {
		camera_config_copy.sensor_mode = sensor_mode;
	}

	if(sensor_config->sensor_type == SENSOR_TYPE_NORMAL){
		/*
			1. sensor_config 是全局变量，多个同配置的Sensor实例时，会导致配置被覆盖
			2. vp_csi_config_t 中保存随着输入变化而变化的信息，在配置时使用
	 	*/
		vp_csi_config_t *csi_config = &pipe_contex->csi_config;
		camera_config_copy.addr = csi_config->sensor_addr;
	}else{
		vp_update_camera_config(sensor_config->camera_config, &camera_config_copy, link_port);
	}
	ret = hbn_camera_create(&camera_config_copy, &pipe_contex->cam_fd);
	ERR_CON_EQ(ret, 0);

	return 0;
}

static int create_deserial_node(pipe_contex_t *pipe_contex) {

	vp_sensor_config_t *sensor_config = NULL;
	deserial_config_t *deserial_config = NULL;
	camera_config_t *camera_config = NULL;
	int32_t ret = 0;

	sensor_config = pipe_contex->sensor_config;
	deserial_config = sensor_config->deserial_node_attr;
	camera_config = sensor_config->camera_config;
	//根据link_port动态设定link_desp
	// const deserial_config_t *deserial_config_edited = vp_get_gmsl_link_desp();
	// memcpy(deserial_config, deserial_config_edited, sizeof(deserial_config_t));

	// printf("%s %d link 0 %s 1 %s 2 %s 3 %s\n",__FUNCTION__,__LINE__,
	// 	&deserial_config->link_desp[0][0],
	// 	&deserial_config->link_desp[1][0],
	// 	&deserial_config->link_desp[2][0],
	// 	&deserial_config->link_desp[3][0]);

	ret = hbn_deserial_create(deserial_config, &pipe_contex->des_fd);
	if(ret != 0){
		printf("hbn_deserial_create failed ret = %d\n", ret);
		return ret;
	}
	return 0;
}

static int create_vin_node(pipe_contex_t *pipe_contex) {
	vp_sensor_config_t *sensor_config = NULL;
	vin_node_attr_t *vin_node_attr = NULL;
	vin_ichn_attr_t *vin_ichn_attr = NULL;
	vin_ochn_attr_t *vin_ochn_attr = NULL;
	hbn_vnode_handle_t *vin_node_handle = NULL;
	hbn_buf_alloc_attr_t alloc_attr = {0};
	vin_attr_ex_t vin_attr_ex;
	uint32_t hw_id = 0;
	int32_t ret = 0;
	uint32_t ichn_id = 0;
	uint32_t ochn_id = 0;

	sensor_config = pipe_contex->sensor_config;
	vin_node_attr = sensor_config->vin_node_attr;
	vin_ichn_attr = sensor_config->vin_ichn_attr;
	vin_ochn_attr = sensor_config->vin_ochn_attr;
	vin_node_attr->magicNumber = MAGIC_NUMBER;
	vin_ochn_attr->magicNumber = MAGIC_NUMBER;

	vp_csi_config_t *csi_config = &pipe_contex->csi_config;
	{
		/*
			1. sensor_config 是全局变量，多个同配置的Sensor实例时，会导致配置被覆盖
			2. vp_csi_config_t 中保存随着输入变化而变化的信息，在配置时使用
	 	*/
		vin_node_attr->cim_attr.mipi_rx = csi_config->mipi_rx;
		hw_id = vin_node_attr->cim_attr.mipi_rx;
		vin_ochn_attr->ddr_en = 1;
		vin_node_attr->cim_attr.cim_pym_flyby  = 0; //使用 SENSOR_TYPE_GMSL_YUV 类型 sensor 时，只能 offline 连接到 pym
	}

	if(sensor_config->sensor_type != SENSOR_TYPE_NORMAL){
		vin_node_attr->cim_attr.vc_index = link_port;
		printf("vc_index:%d\n", vin_node_attr->cim_attr.vc_index);
	}

	vin_node_handle = &pipe_contex->vin_node_handle;
	ret = hbn_vnode_open(HB_VIN, hw_id, AUTO_ALLOC_ID, vin_node_handle);
	ERR_CON_EQ(ret, 0);

	// 设置基本属性
	ret = hbn_vnode_set_attr(*vin_node_handle, vin_node_attr);
	ERR_CON_EQ(ret, 0);
	// 设置输入通道的属性
	ret = hbn_vnode_set_ichn_attr(*vin_node_handle, ichn_id, vin_ichn_attr);
	ERR_CON_EQ(ret, 0);
	// 设置输出通道的属性
	ret = hbn_vnode_set_ochn_attr(*vin_node_handle, ochn_id, vin_ochn_attr);
	ERR_CON_EQ(ret, 0);

	if(vin_ochn_attr->ddr_en)
	{
		memset(&alloc_attr, 0, sizeof(hbn_buf_alloc_attr_t));
		alloc_attr.buffers_num = 6;
		alloc_attr.is_contig = 1;
		alloc_attr.flags =
			HB_MEM_USAGE_CPU_READ_OFTEN | HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
		ret = hbn_vnode_set_ochn_buf_attr(*vin_node_handle, ochn_id, &alloc_attr);
		if (ret < 0) {
			printf("hbn_vnode_set_ochn_buf_attr fail ret %d\n", ret);
			return ret;
		}
	}

	return 0;
}

int create_pym_node(pipe_contex_t *pipe_contex) {
	int ret = 0;
	vp_csi_config_t *csi_config = &pipe_contex->csi_config;
	//pym 的配置依赖mipi_rx 和 ISP的配置
	pym_channel_t pym_channel;
	{
		pym_channel.slot_id = 0;
		pym_channel.hw_id = 1;
		pym_channel.sched_mode = 3; //使用 离线模式 
	}
	int input_width = 0;
	int input_height = 0;
	{
		camera_config_t *camera_config = pipe_contex->sensor_config->camera_config;
		input_width = camera_config->width;
		input_height = camera_config->height;
	}


	pym_cfg_t pym_cfg = {0};
	pym_cfg.hw_id = pym_channel.hw_id;
	pym_cfg.pym_mode = pym_channel.sched_mode;
	pym_cfg.slot_id = pym_channel.slot_id;

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
	pym_cfg.chn_ctrl.src_in_stride_y = ALIGN_16(input_width); //16字节对齐
	pym_cfg.chn_ctrl.src_in_stride_uv = ALIGN_16(input_width);//16字节对齐

	printf("\npym config:\n");
	printf("	ichn input width = %d, height = %d\n", input_width, input_height);
	int ratio = 1;
	for(int i = 0; i < MAX_DS_NUM; i++){

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

		roi_box->wstride_uv = ALIGN_16(roi_box->out_width); //输出(uv分量)宽的stride: 16字节对齐
		roi_box->wstride_y = ALIGN_16(roi_box->out_width);  //输出(y分量) 宽的stride: 16字节对齐
		roi_box->vstride = roi_box->out_height;			    //输出高的Stride, 不需要对齐

		printf("	ochn[%d] ratio= %d, width = %d, height = %d wstride=%d vstride=%d out[%d*%d]\n",
			i, ratio, roi_box->region_width, roi_box->region_height,
			roi_box->wstride_uv, roi_box->vstride, roi_box->out_width, roi_box->out_height);
	}
	printf("\n");
	hbn_vnode_handle_t *pym_node_handle = &pipe_contex->pym_node_handle;

	ret = hbn_vnode_open(HB_PYM, pym_cfg.hw_id, AUTO_ALLOC_ID, pym_node_handle);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_attr(*pym_node_handle, &pym_cfg);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ichn_attr(*pym_node_handle, 0, &pym_cfg);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ochn_attr(*pym_node_handle, 0, &pym_cfg);
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
		ret = hbn_vnode_set_ochn_buf_attr(*pym_node_handle, 0, &alloc_attr);
		ERR_CON_EQ(ret, 0);
	}
	return ret;
}

static int32_t read_gdc_config(char *gdc_bin_file, hb_mem_common_buf_t *bin_buf)
{
	int64_t alloc_flags = 0;
	int ret = 0;
	int offset = 0;
	char *cfg_buf = NULL;

	FILE *fp = fopen(gdc_bin_file, "r");
	if (fp == NULL) {
		printf("File %s open failed\n", gdc_bin_file);
		return -1;
	}
	fseek(fp, 0, SEEK_END);
	long file_size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	cfg_buf = malloc(file_size);
	int n = fread(cfg_buf, 1, file_size, fp);
	if (n != file_size) {
		printf("Read file size failed\n");
	}
	fclose(fp);

	memset(bin_buf, 0, sizeof(hb_mem_common_buf_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
	ret = hb_mem_alloc_com_buf(file_size, alloc_flags, bin_buf);
	if (ret != 0 || bin_buf->virt_addr == NULL) {
		printf("hb_mem_alloc_com_buf for bin failed, ret = %d\n", ret);
		return -1;
	}

	memcpy(bin_buf->virt_addr, cfg_buf, file_size);
	ret = hb_mem_flush_buf(bin_buf->fd, offset, file_size);
	ERR_CON_EQ(ret, 0);

	return ret;
}

int create_gdc_vnode(pipe_contex_t *pipe_contex, gdc_info_s *gdc_info) {
	int ret = 0;
	uint32_t hw_id = 0;
	uint32_t chn_id = 0;

	hbn_buf_alloc_attr_t alloc_attr = {0};
	gdc_settings_t gdc_setting;
	uint32_t input_width = 0;
	uint32_t input_height = 0;
	hbn_vnode_handle_t *gdc_node_handle = NULL;
	gdc_node_handle = &pipe_contex->gdc_node_handle;

	ret = read_gdc_config(gdc_info->gdc_bin_file, &gdc_info->bin_buf);
	if(ret != 0){
		printf("gdc bin file [%s] is not valid\n", gdc_info->gdc_bin_file);
		return -1;
	}
	input_width = pipe_contex->sensor_config->isp_attr->size.width;
	input_height = pipe_contex->sensor_config->isp_attr->size.height;

	ret = hbn_vnode_open(HB_GDC, hw_id, AUTO_ALLOC_ID, gdc_node_handle);
	ERR_CON_EQ(ret, 0);

	gdc_setting.gdc_config.config_addr = gdc_info->bin_buf.phys_addr;
	gdc_setting.gdc_config.config_size = gdc_info->bin_buf.size;
	gdc_setting.gdc_config.input_width = input_width;
	gdc_setting.gdc_config.input_height = input_height;
	gdc_setting.gdc_config.input_stride = ALIGN_16(input_width);
	gdc_setting.gdc_config.output_width = input_width;
	gdc_setting.gdc_config.output_height =input_height;
	gdc_setting.gdc_config.output_stride = ALIGN_16(input_width);
	
	gdc_setting.gdc_config.div_width = 0;
	gdc_setting.gdc_config.div_height = 0;
	gdc_setting.gdc_config.total_planes = 2;
	gdc_setting.binary_ion_id = gdc_info->bin_buf.share_id;
	gdc_setting.binary_offset = gdc_info->bin_buf.offset;
	gdc_setting.magicNumber = MAGIC_NUMBER;

	ret = hbn_vnode_set_attr(*gdc_node_handle, &gdc_setting);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ichn_attr(*gdc_node_handle, chn_id, &gdc_setting);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ochn_attr(*gdc_node_handle, chn_id, &gdc_setting);
	ERR_CON_EQ(ret, 0);
	alloc_attr.buffers_num = 3;
	alloc_attr.is_contig = 1;
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN |
					HB_MEM_USAGE_CPU_WRITE_OFTEN |
					HB_MEM_USAGE_CACHED;
	ret = hbn_vnode_set_ochn_buf_attr(*gdc_node_handle, chn_id, &alloc_attr);
	ERR_CON_EQ(ret, 0);

	return ret;
}

int create_and_run_vflow(pipe_contex_t *pipe_contex, gdc_info_s *gdc_info) {
	int32_t ret = 0;

	// 创建 pipeline 中的每个 node
	ret = create_camera_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	ret = create_vin_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	ret = create_pym_node(pipe_contex);
	ERR_CON_EQ(ret, 0);

	ret = create_gdc_vnode(pipe_contex, gdc_info);
	ERR_CON_EQ(ret, 0);

	// 创建 HBN flow
	ret = hbn_vflow_create(&pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->pym_node_handle);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd, 
							pipe_contex->gdc_node_handle);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->vin_node_handle,
								0,
								pipe_contex->pym_node_handle,
								0);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->pym_node_handle,
								0,
								pipe_contex->gdc_node_handle,
								0);
	ERR_CON_EQ(ret, 0);

	if(sensor_type != SENSOR_TYPE_NORMAL) {
		ret = create_deserial_node(pipe_contex);
		ERR_CON_EQ(ret, 0);
		ret = hbn_camera_attach_to_deserial(pipe_contex->cam_fd, pipe_contex->des_fd, link_port);
		ERR_CON_EQ(ret, 0);
		ret = hbn_deserial_attach_to_vin(pipe_contex->des_fd, link_port, pipe_contex->vin_node_handle);
		ERR_CON_EQ(ret, 0);
	}else {
		ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd,
							pipe_contex->vin_node_handle);
		ERR_CON_EQ(ret, 0);
	}

	ret = hbn_vflow_start(pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);

	return 0;
}

void *read_gdc_data(void *context) {
	pipe_contex_t *pipe_context = (pipe_contex_t *)context;
	hbn_vnode_handle_t gdc_node_handle = pipe_context->gdc_node_handle;
	hbn_vnode_image_t out_img = {0};
	char dst_file[128] = {0};
	uint32_t timeout = 2000;
	uint32_t ochn_id = 0;
	uint32_t count = 0;
	int ret = 0;

	//////////////////////////// VPU ///////////////////////////////////
	media_codec_buffer_t input_buffer = {0};
	media_codec_buffer_t ouput_buffer = {0};
	media_codec_output_buffer_info_t info;
	FILE *fp_output = fopen("single_pipe_vin_isp_vse_vpu.h264", "w+b");
	if (NULL == fp_output) {
		printf("Failed to open output file\n");
	}
	uint8_t uuid[] = "dc45e9bd-e6d948b7-962cd820-d923eeef+SEI_D-Robotics";

	uint32_t length = sizeof(uuid)/sizeof(uuid[0]);
	ret = hb_mm_mc_insert_user_data(&media_context, uuid, length);
	if (ret != 0) {
		printf("#### insert user data failed. ret(%d) ####\n", ret);
		return NULL;
	}

	/////////////////////////////////////////////////////////////////
	while (running) {
		ret = hbn_vnode_getframe(gdc_node_handle, ochn_id, timeout, &out_img);
		if(ret != 0){
			printf("hbn_vnode_getframe gdc_node_handle failed.\n");
			break;
		}
		////////////////////////// Save H264/H265 //////////////////////////////////////
		memset(&input_buffer, 0x00, sizeof(media_codec_buffer_t));
		ret = hb_mm_mc_dequeue_input_buffer(&media_context, &input_buffer,
											2000);
		if (ret != 0) {
			printf("hb_mm_mc_dequeue_input_buffer failed\n");
			break;
		}

		int img_width = out_img.buffer.width;
		int img_height =out_img.buffer.height;
		memcpy(input_buffer.vframe_buf.vir_ptr[0],out_img.buffer.virt_addr[0],
			img_width * img_height * 3 / 2);
		ret = hb_mm_mc_queue_input_buffer(&media_context, &input_buffer, 2000);
		if (ret != 0) {
			printf("hb_mm_mc_queue_input_buffer failed\n");
			break;
		}
		memset(&ouput_buffer, 0x0, sizeof(media_codec_buffer_t));
		memset(&info, 0x0, sizeof(media_codec_output_buffer_info_t));
		ret = hb_mm_mc_dequeue_output_buffer(&media_context, &ouput_buffer,
											&info, 2000);
		if (ret != 0) {
			printf("hb_mm_mc_dequeue_output_buffer failed\n");
			break;
		}
		fwrite(ouput_buffer.vstream_buf.vir_ptr,
				ouput_buffer.vstream_buf.size, 1, fp_output);
		// printf("count:%d\n", count);
		ret = hb_mm_mc_queue_output_buffer(&media_context,
											&ouput_buffer, 2000);
		if (ret != 0) {
			printf("hb_mm_mc_queue_output_buffer failed\n");
			break;
		}
		////////////////////////// Save NV12 //////////////////////////////////////
		if (count % (30) == 0) {
			// 将帧数据写入文件
			snprintf(dst_file, sizeof(dst_file),
				"gdc_handle_%d_chn%d_%dx%d_stride_%d_frameid_%d_ts_%ld.yuv",
				(int)gdc_node_handle, ochn_id,
				out_img.buffer.width, out_img.buffer.height, out_img.buffer.stride,
				out_img.info.frame_id, out_img.info.timestamps);
			printf("gdc(%d) dump yuv %dx%d(stride:%d), buffer size: %ld + %ld frame id: %d,"
					" timestamp: %ld\n", (int)gdc_node_handle,
					out_img.buffer.width, out_img.buffer.height,
					out_img.buffer.stride,
					out_img.buffer.size[0], out_img.buffer.size[1],
					out_img.info.frame_id,
					out_img.info.timestamps);
			dump_2plane_yuv_to_file(dst_file,
					out_img.buffer.virt_addr[0],
					out_img.buffer.virt_addr[1],
					out_img.buffer.size[0],
					out_img.buffer.size[1]);
		}

		hbn_vnode_releaseframe(gdc_node_handle, 0, &out_img);

		count++;
	}
	fclose(fp_output);
	return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////////////


static int32_t get_rc_params(media_codec_context_t *context,
			mc_rate_control_params_t *rc_params) {
	int32_t ret = 0;
	ret = hb_mm_mc_get_rate_control_config(context, rc_params);
	if (ret) {
		printf("Failed to get rc params ret=0x%x\n", ret);
		return ret;
	}
	switch (rc_params->mode) {
	case MC_AV_RC_MODE_H264CBR:
		rc_params->h264_cbr_params.intra_period = 30;
		rc_params->h264_cbr_params.intra_qp = 30;
		rc_params->h264_cbr_params.bit_rate = 5000;
		rc_params->h264_cbr_params.frame_rate = 30;
		rc_params->h264_cbr_params.initial_rc_qp = 20;
		rc_params->h264_cbr_params.vbv_buffer_size = 20;
		rc_params->h264_cbr_params.mb_level_rc_enalbe = 1;
		rc_params->h264_cbr_params.min_qp_I = 8;
		rc_params->h264_cbr_params.max_qp_I = 50;
		rc_params->h264_cbr_params.min_qp_P = 8;
		rc_params->h264_cbr_params.max_qp_P = 50;
		rc_params->h264_cbr_params.min_qp_B = 8;
		rc_params->h264_cbr_params.max_qp_B = 50;
		rc_params->h264_cbr_params.hvs_qp_enable = 1;
		rc_params->h264_cbr_params.hvs_qp_scale = 2;
		rc_params->h264_cbr_params.max_delta_qp = 10;
		rc_params->h264_cbr_params.qp_map_enable = 0;
		break;
	case MC_AV_RC_MODE_H264VBR:
		rc_params->h264_vbr_params.intra_qp = 20;
		rc_params->h264_vbr_params.intra_period = 30;
		rc_params->h264_vbr_params.intra_qp = 35;
		break;
	case MC_AV_RC_MODE_H264AVBR:
		rc_params->h264_avbr_params.intra_period = 15;
		rc_params->h264_avbr_params.intra_qp = 25;
		rc_params->h264_avbr_params.bit_rate = 2000;
		rc_params->h264_avbr_params.vbv_buffer_size = 3000;
		rc_params->h264_avbr_params.min_qp_I = 15;
		rc_params->h264_avbr_params.max_qp_I = 50;
		rc_params->h264_avbr_params.min_qp_P = 15;
		rc_params->h264_avbr_params.max_qp_P = 45;
		rc_params->h264_avbr_params.min_qp_B = 15;
		rc_params->h264_avbr_params.max_qp_B = 48;
		rc_params->h264_avbr_params.hvs_qp_enable = 0;
		rc_params->h264_avbr_params.hvs_qp_scale = 2;
		rc_params->h264_avbr_params.max_delta_qp = 5;
		rc_params->h264_avbr_params.qp_map_enable = 0;
		break;
	case MC_AV_RC_MODE_H264FIXQP:
		rc_params->h264_fixqp_params.force_qp_I = 23;
		rc_params->h264_fixqp_params.force_qp_P = 23;
		rc_params->h264_fixqp_params.force_qp_B = 23;
		rc_params->h264_fixqp_params.intra_period = 23;
		break;
	case MC_AV_RC_MODE_H264QPMAP:
		break;
	case MC_AV_RC_MODE_H265CBR:
		rc_params->h265_cbr_params.intra_period = 20;
		rc_params->h265_cbr_params.intra_qp = 30;
		rc_params->h265_cbr_params.bit_rate = 5000;
		rc_params->h265_cbr_params.frame_rate = 30;
		if (context->video_enc_params.width >= 480 ||
			context->video_enc_params.height >= 480) {
			rc_params->h265_cbr_params.initial_rc_qp = 30;
			rc_params->h265_cbr_params.vbv_buffer_size = 3000;
			rc_params->h265_cbr_params.ctu_level_rc_enalbe = 1;
		} else {
			rc_params->h265_cbr_params.initial_rc_qp = 20;
			rc_params->h265_cbr_params.vbv_buffer_size = 20;
			rc_params->h265_cbr_params.ctu_level_rc_enalbe = 1;
		}
		rc_params->h265_cbr_params.min_qp_I = 8;
		rc_params->h265_cbr_params.max_qp_I = 50;
		rc_params->h265_cbr_params.min_qp_P = 8;
		rc_params->h265_cbr_params.max_qp_P = 50;
		rc_params->h265_cbr_params.min_qp_B = 8;
		rc_params->h265_cbr_params.max_qp_B = 50;
		rc_params->h265_cbr_params.hvs_qp_enable = 1;
		rc_params->h265_cbr_params.hvs_qp_scale = 2;
		rc_params->h265_cbr_params.max_delta_qp = 10;
		rc_params->h265_cbr_params.qp_map_enable = 0;
		break;
	case MC_AV_RC_MODE_H265VBR:
		rc_params->h265_vbr_params.intra_qp = 20;
		rc_params->h265_vbr_params.intra_period = 30;
		rc_params->h265_vbr_params.intra_qp = 35;
		break;
	case MC_AV_RC_MODE_H265AVBR:
		rc_params->h265_avbr_params.intra_period = 15;
		rc_params->h265_avbr_params.intra_qp = 25;
		rc_params->h265_avbr_params.bit_rate = 2000;
		rc_params->h265_avbr_params.vbv_buffer_size = 3000;
		rc_params->h265_avbr_params.min_qp_I = 15;
		rc_params->h265_avbr_params.max_qp_I = 50;
		rc_params->h265_avbr_params.min_qp_P = 15;
		rc_params->h265_avbr_params.max_qp_P = 45;
		rc_params->h265_avbr_params.min_qp_B = 15;
		rc_params->h265_avbr_params.max_qp_B = 48;
		rc_params->h265_avbr_params.hvs_qp_enable = 0;
		rc_params->h265_avbr_params.hvs_qp_scale = 2;
		rc_params->h265_avbr_params.max_delta_qp = 5;
		rc_params->h265_avbr_params.qp_map_enable = 0;
		break;
	case MC_AV_RC_MODE_H265FIXQP:
		rc_params->h265_fixqp_params.force_qp_I = 23;
		rc_params->h265_fixqp_params.force_qp_P = 23;
		rc_params->h265_fixqp_params.force_qp_B = 23;
		rc_params->h265_fixqp_params.intra_period = 23;
		break;
	case MC_AV_RC_MODE_H265QPMAP:
		break;
	default:
		ret = HB_MEDIA_ERR_INVALID_PARAMS;
		break;
	}
	return ret;
}

int32_t vp_encode_config_param(media_codec_context_t *context,
							media_codec_id_t codec_type,
							int32_t width, int32_t height,
							int32_t frame_rate, uint32_t bit_rate)
{
	mc_video_codec_enc_params_t *params;

	memset(context, 0x00, sizeof(media_codec_context_t));
	context->encoder = 1;
	params = &context->video_enc_params;
	params->width = width;
	params->height = height;
	params->pix_fmt = MC_PIXEL_FORMAT_NV12;
	params->bitstream_buf_size = (width * height * 3 / 2  + 0x3ff) & ~0x3ff;
	params->frame_buf_count = 5;
	params->external_frame_buf = 0;
	params->bitstream_buf_count = 8;
	params->gop_params.gop_preset_idx = 9;
	params->rot_degree = MC_CCW_0;
	params->mir_direction = MC_DIRECTION_NONE;
	params->frame_cropping_flag = 0;
	params->enable_user_pts = 1;
	params->gop_params.decoding_refresh_type = 2;
	switch (codec_type)
	{
	case MEDIA_CODEC_ID_H264:
		context->codec_id = MEDIA_CODEC_ID_H264;
		params->rc_params.mode = MC_AV_RC_MODE_H264CBR;
		get_rc_params(context, &params->rc_params);
		params->rc_params.h264_cbr_params.frame_rate = frame_rate;
		params->rc_params.h264_cbr_params.bit_rate = bit_rate;
		break;
	case MEDIA_CODEC_ID_H265:
		context->codec_id = MEDIA_CODEC_ID_H265;
		params->rc_params.mode = MC_AV_RC_MODE_H265CBR;
		get_rc_params(context, &params->rc_params);
		params->rc_params.h265_cbr_params.frame_rate = frame_rate;
		params->rc_params.h265_cbr_params.bit_rate = bit_rate;
		break;
	case MEDIA_CODEC_ID_MJPEG:
		context->codec_id = MEDIA_CODEC_ID_MJPEG;
		params->rc_params.mode = MC_AV_RC_MODE_MJPEGFIXQP;
		get_rc_params(context, &params->rc_params);
		params->mjpeg_enc_config.restart_interval = width / 16;
		break;
	case MEDIA_CODEC_ID_JPEG:
		context->codec_id = MEDIA_CODEC_ID_JPEG;
		params->jpeg_enc_config.quality_factor = 50;
		params->mjpeg_enc_config.restart_interval = width / 16;
		break;
	default:
		printf("Not Support encoding type: %d!\n", codec_type);
		return -1;
	}

	return 0;
}

int encode_init(void *data, camera_config_info_t* config_info) {
	int ret = 0;
	pipe_contex_t *pipe_context = NULL;
	mc_av_codec_startup_params_t startup_params = {0};

	int encode_fps = config_info->fps;
	int encode_width = config_info->width;
	int encode_height = config_info->height;
	media_codec_id_t encode_type = config_info->encode_type;

	ret = vp_encode_config_param(&media_context, encode_type,
								encode_width, encode_height,
								encode_fps, 8192);
	ERR_CON_EQ(ret, 0);
	ret = hb_mm_mc_initialize(&media_context);
	ERR_CON_EQ(ret, 0);
	ret = hb_mm_mc_configure(&media_context);
	ERR_CON_EQ(ret, 0);
	ret = hb_mm_mc_start(&media_context, &startup_params);
	printf("%s idx: %d, init successful\n",
			media_context.encoder ? "Encode" : "Decode",
			media_context.instance_index);
	return 0;
}

int encode_deinit(void *data) {
	int ret = 0;
	ret = hb_mm_mc_pause(&media_context);
	ERR_CON_EQ(ret, 0);
	ret = hb_mm_mc_release(&media_context);
	ERR_CON_EQ(ret, 0);

	return 0;
}