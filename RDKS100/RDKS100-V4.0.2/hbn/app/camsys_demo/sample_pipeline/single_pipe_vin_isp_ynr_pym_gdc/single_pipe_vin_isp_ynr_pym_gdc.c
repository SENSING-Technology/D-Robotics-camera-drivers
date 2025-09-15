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

static int vin_online_isp = 0; //根据VIN决定
static int isp_online_ynr = 1;
static int ynr_online_pym = 1;

static int32_t running = 0;
static uint32_t sensor_type = 0;
static uint32_t link_port = 0;
static uint16_t date_type;
static uint32_t sensor_mode = 0; // 1: NORMAL_M; 2: DOL2_M; 6: SLAVE_M

typedef struct gdc_info
{
	char* gdc_bin_file;
	hb_mem_common_buf_t bin_buf;
} gdc_info_s;

int create_and_run_vflow(pipe_contex_t *pipe_contex, gdc_info_s *gdc_info);
void *read_gdc_data(void *contex);
int32_t hbn_deserial_create(deserial_config_t *des_config, deserial_handle_t *des_fd);
int32_t hbn_deserial_attach_to_vin(deserial_handle_t des_fd, camera_des_link_t link, vpf_handle_t vin_fd);

static void print_help() {
	printf("Usage: single_pipe_vin_isp_pym_gdc [OPTIONS]\n");
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
		if(sensor_type == SENSOR_TYPE_GMSL_YUV){
			printf("[Error] Not Support gmsl YUV sensor! ,please check!\n");
			return -1;
		}
		if(sensor_type == SENSOR_TYPE_NORMAL){
			ret = vp_sensor_multi_fixed_mipi_host(pipe_contex.sensor_config,
				&used_mipi_host, &pipe_contex.csi_config);
			if (ret != 0) {
				printf("No Camera Sensor found. Please check if the specified "
					"sensor is connected to the Camera interface.\n");
				return ret;
			}
		} else {
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

	// //初始化GSML的 link_desp
	// //不同的sensor和加串要重新map成不一样的地址,不然相同的地址解串器没法和他们通讯,所以根据link_port动态设定eeprom_addr serial_addr
	// if (sensor_type != SENSOR_TYPE_NORMAL && link_port_specified) {
	// 	camera_config_t *camera_config = pipe_contex.sensor_config->camera_config;
	// 	vp_update_deserial_config(camera_config, link_port);
	// 	vp_shwo_gmsl_link_desp();
	// }

	hb_mem_module_open();
	ret = create_and_run_vflow(&pipe_contex, &gdc_info);
	ERR_CON_EQ(ret, 0);
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
	deserial_handle_t *des_handle = NULL;

	int32_t ret = 0;
	des_handle = &pipe_contex->des_fd;

	sensor_config = pipe_contex->sensor_config;
	deserial_config = sensor_config->deserial_node_attr;
	// //根据link_port动态设定link_desp
	// const deserial_config_t *deserial_config_edited = vp_get_gmsl_link_desp();
	// memcpy(deserial_config, deserial_config_edited, sizeof(deserial_config_t));

	// printf("%s %d link 0 %s 1 %s 2 %s 3 %s\n",__FUNCTION__,__LINE__,
	// 	&deserial_config->link_desp[0][0],
	// 	&deserial_config->link_desp[1][0],
	// 	&deserial_config->link_desp[2][0],
	// 	&deserial_config->link_desp[3][0]);
	ret = hbn_deserial_create(deserial_config, des_handle);
	if(ret != 0){
		printf("hbn_deserial_create failed ret = %d\n", ret);
		return ret;
	}
	printf("deserial_config:,%02x,%s, des_handle:%ld \n\r" ,deserial_config->addr,
	deserial_config->name, *des_handle);
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
		if(csi_config->mipi_rx != 1){
			vin_online_isp = 0;
		}else{
			vin_online_isp = 1;
		}
	}
	{
		/*
			1. sensor_config 是全局变量，多个同配置的Sensor实例时，会导致配置被覆盖
			2. vp_csi_config_t 中保存随着输入变化而变化的信息，在配置时使用
	 	*/
		vin_node_attr->cim_attr.mipi_rx = csi_config->mipi_rx;
		hw_id = vin_node_attr->cim_attr.mipi_rx;

		//online
		if(vin_online_isp){
			vin_ochn_attr->ddr_en = 0;
			vin_node_attr->cim_attr.cim_isp_flyby  = 1;
		}else{
			vin_ochn_attr->ddr_en = 1;
			vin_node_attr->cim_attr.cim_isp_flyby  = 0;
		}
	}
	if(sensor_config->sensor_type == !SENSOR_TYPE_NORMAL){
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

static int create_isp_node(pipe_contex_t *pipe_contex) {
	vp_sensor_config_t *sensor_config = NULL;
	isp_attr_t      *isp_attr = NULL;
	isp_ichn_attr_t *isp_ichn_attr = NULL;
	isp_ochn_attr_t *isp_ochn_attr = NULL;
	hbn_vnode_handle_t *isp_node_handle = NULL;
	hbn_buf_alloc_attr_t alloc_attr = {0};
	int hw_id = 0;
	uint32_t ichn_id = 0;
	uint32_t ochn_id = 0;
	int ret = 0;

	sensor_config = pipe_contex->sensor_config;
	isp_attr = sensor_config->isp_attr;
	isp_ichn_attr = sensor_config->isp_ichn_attr;
	isp_ochn_attr = sensor_config->isp_ochn_attr;
	isp_node_handle = &pipe_contex->isp_node_handle;

	{
		/*
			1. sensor_config 是全局变量，多个同配置的Sensor实例时，会导致配置被覆盖
			2. vp_csi_config_t 中保存随着输入变化而变化的信息，在配置时使用
	 	*/
		vp_csi_config_t *csi_config = &pipe_contex->csi_config;
		if((csi_config->mipi_rx == 0) || (csi_config->mipi_rx == 4)){
			vin_online_isp = 0;
			isp_attr->channel.slot_id = 4; //4 - 11
		}else{
			vin_online_isp = 1;
			isp_attr->channel.slot_id = 0;
		}
		isp_attr->channel.hw_id = 1; // 必须是1

		//vin_online_isp
		if(vin_online_isp){
			isp_attr->sched_mode = SCHED_MODE_PASS_THRU;
		}else{
			isp_attr->sched_mode = SCHED_MODE_MANUAL;
		}

		if(isp_online_ynr){
			isp_ochn_attr->stream_output_mode = STREAM_OUTPUT_MODE_ENABLE;
			isp_ochn_attr->axi_output_mode = AXI_OUTPUT_MODE_DISABLE;
		}else{
			printf("isp must online ynr.\n");
			return -1;
		}
	}
	hw_id = isp_attr->channel.hw_id;

	ret = hbn_vnode_open(HB_ISP, hw_id, AUTO_ALLOC_ID, isp_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_attr(*isp_node_handle, isp_attr);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ochn_attr(*isp_node_handle, ochn_id, isp_ochn_attr);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ichn_attr(*isp_node_handle, ichn_id, isp_ichn_attr);
	ERR_CON_EQ(ret, 0);

	if(!isp_online_ynr){
		alloc_attr.buffers_num = 3;
		alloc_attr.is_contig = 1;
		alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
							| HB_MEM_USAGE_CPU_WRITE_OFTEN
							| HB_MEM_USAGE_CACHED;
		ret = hbn_vnode_set_ochn_buf_attr(*isp_node_handle, ochn_id, &alloc_attr);
		ERR_CON_EQ(ret, 0);
	}

	return 0;
}

int create_pym_node(pipe_contex_t *pipe_contex,
	const channel_and_sched_info_t *ch_sched_info) {
	int ret = 0;
	vp_csi_config_t *csi_config = &pipe_contex->csi_config;

	//pym 的配置依赖mipi_rx 和 ISP的配置
	pym_channel_t pym_channel;
	{
		if(csi_config->mipi_rx == 4){
			//强制设置为1
			pym_channel.slot_id = ch_sched_info->isp_channel.slot_id;
			pym_channel.hw_id = 1;
			pym_channel.sched_mode = 1;
		}else{
			ret = get_pym_channel_config_for_single_pipeline(
					ch_sched_info, isp_online_ynr, &pym_channel);
			if(ret != 0){
				printf("[create_pym_node] pym init failed.\n");
				return -1;
			}
		}
		//强制设置为1
		// pym_channel.hw_id = 1;
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
static int create_ynr_node(pipe_contex_t *pipe_contex, const channel_and_sched_info_t *ch_sched_info) {
	int ret = 0;

	struct ynr_init_attr *attr;
	vp_sensor_config_t *sensor_config = NULL;
	hbn_vnode_handle_t *ynr_node_handle = NULL;

	ynr_node_handle = &pipe_contex->ynr_node_handle;
	sensor_config = pipe_contex->sensor_config;
	attr = sensor_config->ynr_attr;

	ynr_channel_t pym_channel;
	ret = get_ynr_channel_config_for_single_pipeline(ch_sched_info, &pym_channel);
	ERR_CON_EQ(ret, 0);

	attr->work_mode = pym_channel.sched_mode;
	attr->slot_id =  pym_channel.slot_id;

	int hw_id = 1; //固定为1
	ret = hbn_vnode_open(HB_YNR, hw_id, AUTO_ALLOC_ID, ynr_node_handle);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_attr(*ynr_node_handle, attr);
	ERR_CON_EQ(ret, 0);

#if 1
	struct hobot_ynr_channel_input_config channel_input_cfg = {0};
	ret = hbn_vnode_set_ichn_attr(*ynr_node_handle, 0, &channel_input_cfg);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ichn_attr(*ynr_node_handle, 1, &channel_input_cfg);
	ERR_CON_EQ(ret, 0);

	struct hobot_ynr_channel_output_config channel_output_cfg = {0};
	ret = hbn_vnode_set_ochn_attr(*ynr_node_handle, 0, &channel_output_cfg);
	ERR_CON_EQ(ret, 0);
#endif

	if (attr->nr3d_en == 1u) {
		hbn_buf_alloc_attr_t alloc_attr;
		alloc_attr.buffers_num = 3;
		alloc_attr.is_contig = 1;
		alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN |
			(uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN | (uint64_t)HB_MEM_USAGE_CACHED);
		ret = hbn_vnode_set_ochn_buf_attr(*ynr_node_handle, 0, &alloc_attr);
		ERR_CON_EQ(ret, 0);
	}
	return 0;
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
	gdc_setting.gdc_config.input_stride = ALIGN_16(input_width);//16字节对齐
	gdc_setting.gdc_config.output_width = input_width;
	gdc_setting.gdc_config.output_height =input_height;
	gdc_setting.gdc_config.output_stride = ALIGN_16(input_width);//16字节对齐
	
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
	ret = create_isp_node(pipe_contex);
	ERR_CON_EQ(ret, 0);

	channel_and_sched_info_t ch_sched_info;
	get_channel_and_sched_info(pipe_contex, &ch_sched_info);

	ret = create_ynr_node(pipe_contex, &ch_sched_info);
	ERR_CON_EQ(ret, 0);

	ret = create_pym_node(pipe_contex, &ch_sched_info);
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
							pipe_contex->isp_node_handle);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->ynr_node_handle);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->pym_node_handle);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd, 
							pipe_contex->gdc_node_handle);
	ERR_CON_EQ(ret, 0);

	if(vin_online_isp){
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->vin_node_handle,
								1,
								pipe_contex->isp_node_handle,
								0);
		ERR_CON_EQ(ret, 0);
	}else{
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->vin_node_handle,
								0,
								pipe_contex->isp_node_handle,
								0);
		ERR_CON_EQ(ret, 0);
	}

	if(!isp_online_ynr){
		printf("isp must online ynr.\n");
	}
	ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle,
							1,
							pipe_contex->ynr_node_handle,
							0);
	ERR_CON_EQ(ret, 0);


	ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->ynr_node_handle,
								1,
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

	while (running) {
		ret = hbn_vnode_getframe(gdc_node_handle, ochn_id, timeout, &out_img);
		if(ret != 0){
			printf("hbn_vnode_getframe gdc_node_handle failed.\n");
			break;
		}

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
		usleep(1000 * 20);
	}
	return NULL;
}

