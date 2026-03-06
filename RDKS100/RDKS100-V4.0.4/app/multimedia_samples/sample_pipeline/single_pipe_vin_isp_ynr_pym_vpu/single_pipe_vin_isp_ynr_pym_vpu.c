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
typedef enum {
	PIPELINE_SCENE_ISP_BYPASS    = 0,    /** 情景0(不需要ISP): Sensor 输出 YUV数据*/
	PIPELINE_SCENE_ISP_ONLY      = 1,    /** 情景1(ISP): 不需要运行YNR的Sensor, 或者宽和高 > 2048的sensor */
	PIPELINE_SCENE_ISP_YNR       = 2,    /** 情景2(ISP + YNR): 需要运行YNR的Sensor, 并且宽和高 <= 2048的sensor */
	PIPELINE_SCENE_MAX                   /**< 枚举边界，不可用作实际参数 */
} pipeline_usage_scene_type_t;

typedef struct {
	int isp_mode;
	int isp_hw_id;
	int isp_slot_id;

	int ynr_mode;
	int ynr_slot_id;

	int pym_slot_id;
	int pym_hw_id;
	int pym_mode;

	//PIPELINE_SCENE_ISP_BYPASS
	int is_online_vin_pym;

	//PIPELINE_SCENE_ISP_ONLY
	int is_online_vin_isp;
	int is_online_isp_pym;

	//PIPELINE_SCENE_ISP_YNR
	int is_online_isp_ynr;
	int is_online_ynr_pym;


}pipeline_channel_info_t;
static struct option const long_options[] = {
	{"sensor", required_argument, NULL, 's'},
	{"link_port", required_argument, NULL, 'l'},
	{NULL, 0, NULL, 0}
};


static int32_t running = 0;
static uint32_t g_link_port = 0;
static pipeline_channel_info_t g_ch_info;
static pipeline_usage_scene_type_t g_scene_type;
static media_codec_context_t g_media_context;

int create_and_run_vflow(pipe_contex_t *pipe_contex);
void *read_pym_and_encode_data(void *contex);
int32_t hbn_deserial_create(deserial_config_t *des_config, deserial_handle_t *des_fd);
int32_t hbn_deserial_attach_to_vin(deserial_handle_t des_fd, camera_des_link_t link, vpf_handle_t vin_fd);

int encode_init(void *data, camera_config_info_t* camera_config_info);
int encode_deinit(void *data);

static void print_help() {
	printf("Usage: single_pipe_vin_isp_ynr_pym_vpu [OPTIONS]\n");
	printf("Options:\n");
	printf("  -s <sensor_index>      Specify sensor index\n");
	printf("  -l <link_port>         Specify the port for connecting serdes sensors, 0:A 1:B 2:C 3:D\n");
	printf("  -h                     Show this help message\n");
	vp_show_sensors_list(); // Assuming this function displays sensor list
}

void signal_handle(int signo) {
	running = 0;
}

const char* get_link_mode_string(int is_online) {
	return (is_online ? "online" : "offline");
}
/**
	PIPELINE_SCENE_ISP_BYPASS：
		cim_0: online pym0
		cim_1: online pym1
		cim_4: offline PYM4
	PIPELINE_SCENE_ISP_ONLY:
		cim_0: online isp0 online pym0
		cim_1: offline isp0 online pym0
		cim_4: offline isp0 online pym0
	PIPELINE_SCENE_ISP_YNR
		cim_0: offline isp1 online ynr1 online pym1
		cim_1: online isp1 online ynr1 online pym1
		cim_4: offline isp1 online ynr1 online pym1
 */
void pipeline_connect_param_init(pipe_contex_t *pipe_contex, int pipeline_index) {
	vp_sensor_config_t *sensor_config = pipe_contex->sensor_config;

	if(sensor_config->sensor_type == SENSOR_TYPE_GMSL_YUV) {
		g_scene_type = PIPELINE_SCENE_ISP_BYPASS;
	}else{
		if(sensor_config->ynr_attr == NULL) {
			g_scene_type = PIPELINE_SCENE_ISP_ONLY;
		}else if((sensor_config->camera_config->width > 2048) ||
			(sensor_config->camera_config->height > 2048)) {
			g_scene_type = PIPELINE_SCENE_ISP_ONLY;
		}else{
			g_scene_type = PIPELINE_SCENE_ISP_YNR;
		}
	}

	pipeline_channel_info_t* ch_info = &g_ch_info;
	// 情景1(不需要ISP): 尽量online
	if(g_scene_type == PIPELINE_SCENE_ISP_BYPASS) {

		ch_info->pym_hw_id = pipe_contex->csi_config.mipi_rx;
		if(pipe_contex->csi_config.mipi_rx == 4) {
			ch_info->pym_mode = PYM_M2M_MODE; //offline
			ch_info->pym_slot_id = 0; 		  //offline的情况可以不用设置
			ch_info->is_online_vin_pym = 0;
		}else{
			ch_info->pym_mode = PYM_MANUAL_MODE; //online
			ch_info->pym_slot_id = 0;
			ch_info->is_online_vin_pym = 1;
		}

		printf("	[%d] [not use isp].\n", pipeline_index);
		printf("		vin [hw:%d]\n", pipe_contex->csi_config.mipi_rx);
		printf("		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->pym_slot_id, ch_info->pym_hw_id, ch_info->pym_mode);
		printf("		vin ->%s-> pym \n",
			get_link_mode_string(ch_info->is_online_vin_pym));
	// 情景2(需要ISP， 不需要YNR): 固定使用ISP0
	}else if(g_scene_type == PIPELINE_SCENE_ISP_ONLY) {
		ch_info->isp_mode = SCHED_MODE_MANUAL;
		ch_info->isp_hw_id = 0;
		if(pipe_contex->csi_config.mipi_rx == 0) {
			ch_info->isp_slot_id = 0; //online
		}else{
			ch_info->isp_slot_id = 4; //online
		}

		ch_info->pym_slot_id = ch_info->isp_slot_id;
		ch_info->pym_hw_id = 0;  		  			//固定设置为0
		ch_info->pym_mode = PYM_MANUAL_MODE; 		//offline

		if(pipe_contex->csi_config.mipi_rx == 0) {
			ch_info->is_online_vin_isp = 1;
		}else{
			ch_info->is_online_vin_isp = 0;
		}

		ch_info->is_online_isp_pym = 1;

		printf("	[%d] only use [isp only].\n", pipeline_index);
		printf("		vin [hw:%d]\n", pipe_contex->csi_config.mipi_rx);
		printf("		isp [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode);
		printf("		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->pym_hw_id, ch_info->pym_slot_id, ch_info->pym_mode);
		printf("		vin ->%s-> isp ->%s-> pym \n",
			get_link_mode_string(ch_info->is_online_vin_isp),
			get_link_mode_string(ch_info->is_online_isp_pym));

	// 情景3(ISP + YNR): 固定使用ISP1
	}else if(g_scene_type == PIPELINE_SCENE_ISP_YNR) {
		ch_info->isp_mode = SCHED_MODE_MANUAL;
		ch_info->isp_hw_id = 1;
		if(pipe_contex->csi_config.mipi_rx == 1) {
			ch_info->isp_slot_id = 0;
		}else{
			ch_info->isp_slot_id = 4;
		}

		ch_info->ynr_mode = 1; //1:Manaul 模式	2:全online模式
		ch_info->ynr_slot_id = ch_info->isp_slot_id;

		ch_info->pym_slot_id = ch_info->isp_slot_id;
		ch_info->pym_hw_id = 1;
		ch_info->pym_mode = PYM_MANUAL_MODE;

		if(pipe_contex->csi_config.mipi_rx == 1) {
			ch_info->is_online_vin_isp = 1;
		}else{
			ch_info->is_online_vin_isp = 0;
		}
		ch_info->is_online_isp_ynr = 1;
		ch_info->is_online_ynr_pym = 1;

		printf("	[%d] use [isp + ynr].\n", pipeline_index);
		printf("		vin [hw:%d]\n", pipe_contex->csi_config.mipi_rx);
		printf("		isp [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode);
		printf("		ynr [hw:%d] [slot_id:%d] [mode:%d]\n",
			1, ch_info->ynr_slot_id, ch_info->ynr_mode);
		printf("		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			ch_info->pym_hw_id, ch_info->pym_slot_id, ch_info->pym_mode);

		printf("		vin ->%s-> isp ->%s-> ynr ->%s-> pym\n",
			get_link_mode_string(ch_info->is_online_vin_isp),
			get_link_mode_string(ch_info->is_online_isp_ynr),
			get_link_mode_string(ch_info->is_online_ynr_pym));
	}else{
		printf("[error] not support type %d\n", g_scene_type);
	}
}

int main(int argc, char** argv) {
	int ret = 0;
	pipe_contex_t pipe_contex = {0};
	pthread_t read_thread;
	int opt_index = 0;
	int c = 0;
	int index = -1;
	int link_port_specified = false;
	uint32_t sensor_type = 0;

	while((c = getopt_long(argc, argv, "s:l:h",
							long_options, &opt_index)) != -1) {
		switch (c)
		{
		case 's':
			index = atoi(optarg);
			break;
		case 'l':
			g_link_port = atoi(optarg);
			link_port_specified = true;
			break;
		case 'h':
		default:
			print_help();
			return 0;
		}
	}

	if (index == -1) {
		printf("No sensors specified.\n");
		print_help();
		return 0;
	}

	int used_mipi_host = 0;
	if (index < vp_get_sensors_list_number() && index >= 0) {
		pipe_contex.sensor_config = vp_sensor_config_list[index];
		printf("Using index:%d  sensor_name:%s  config_file:%s\n",
				index,
				vp_sensor_config_list[index]->sensor_name,
				vp_sensor_config_list[index]->config_file);

		sensor_type = pipe_contex.sensor_config->sensor_type;
		if (sensor_type != SENSOR_TYPE_NORMAL && !link_port_specified) {
			print_help();
			printf("[Error] %s is gsml sensor, must config link port according to the hardware connection, can be set to [0-3]\n",
				vp_sensor_config_list[index]->sensor_name);

			return -1;
		}

		if(sensor_type == SENSOR_TYPE_NORMAL) {
			ret = vp_sensor_multi_fixed_mipi_host(pipe_contex.sensor_config,
				&used_mipi_host, &pipe_contex.csi_config);
			if (ret != 0) {
				printf("No Camera Sensor found. Please check if the specified "
					"sensor is connected to the Camera interface.\n");
				return ret;
			}
		}else{
			pipe_contex.csi_config.mipi_rx = 4;
		}
	} else {
		printf("Unsupport sensor index:%d\n", index);
		print_help();
		return 0;
	}

	if (sensor_type != SENSOR_TYPE_NORMAL && link_port_specified) {
		camera_config_t *camera_config = pipe_contex.sensor_config->camera_config;
		vp_deserial_config_update(camera_config, g_link_port);
	}
	pipeline_connect_param_init(&pipe_contex, 0);

	hb_mem_module_open();
	ret = create_and_run_vflow(&pipe_contex);
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
	ret = pthread_create(&read_thread, NULL, (void *)read_pym_and_encode_data,
						(void *)&pipe_contex);
	pthread_join(read_thread, NULL);
	ret = hbn_vflow_stop(pipe_contex.vflow_fd);
	ERR_CON_EQ(ret, 0);
	hbn_vnode_close(pipe_contex.pym_node_handle);
	hbn_vnode_close(pipe_contex.isp_node_handle);
	hbn_vnode_close(pipe_contex.vin_node_handle);
	hbn_camera_destroy(pipe_contex.cam_fd);
	hbn_vflow_destroy(pipe_contex.vflow_fd);
	hb_mem_module_close();
	encode_deinit(&pipe_contex);

	return 0;
}

/////////////////////////////////////////////// HBN ////////////////////////////////////////////
static int create_camera_node(pipe_contex_t *pipe_contex, int link_port) {

	camera_config_t camera_config_copy = {0};
	vp_sensor_config_t *sensor_config = NULL;
	int32_t ret = 0;

	sensor_config = pipe_contex->sensor_config;
	camera_config_copy = *sensor_config->camera_config;

	if(sensor_config->sensor_type == SENSOR_TYPE_NORMAL) {
		/*
			1. sensor_config 是全局变量，多个同配置的Sensor实例时，会导致配置被覆盖
			2. vp_csi_config_t 中保存随着输入变化而变化的信息，在配置时使用
	 	*/
		vp_csi_config_t *csi_config = &pipe_contex->csi_config;
		camera_config_copy.addr = csi_config->sensor_addr;
	}else{
		vp_update_camera_config(sensor_config->camera_config, &camera_config_copy, link_port);
		vp_deserial_config_show();
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

	ret = hbn_deserial_create(deserial_config, des_handle);
	if(ret != 0) {
		printf("hbn_deserial_create failed ret = %d\n", ret);
		return ret;
	}
	printf("deserial_config:,%02x,%s, des_handle:%ld \n\r" ,deserial_config->addr, deserial_config->name, *des_handle);
	return 0;
}
static int create_vin_node(pipe_contex_t *pipe_contex, int is_online, int link_port) {
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

	{
		/*
			1. sensor_config 是全局变量，多个同配置的Sensor实例时，会导致配置被覆盖
			2. vp_csi_config_t 中保存随着输入变化而变化的信息，在配置时使用
	 	*/
		vp_csi_config_t *csi_config = &pipe_contex->csi_config;
		vin_node_attr->cim_attr.mipi_rx = csi_config->mipi_rx;
		hw_id = vin_node_attr->cim_attr.mipi_rx;

		//online
		if(is_online) {
			vin_ochn_attr->ddr_en = 0;
			vin_node_attr->cim_attr.cim_isp_flyby  = 1;
		}else{
			vin_ochn_attr->ddr_en = 1;
			vin_node_attr->cim_attr.cim_isp_flyby  = 0;
		}
	}
	if(sensor_config->sensor_type != SENSOR_TYPE_NORMAL) {
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

static int create_isp_node(pipe_contex_t *pipe_contex, int hw_id, int slot_id, int mode, int is_online) {
	vp_sensor_config_t *sensor_config = NULL;
	isp_attr_t      *isp_attr = NULL;
	isp_ichn_attr_t *isp_ichn_attr = NULL;
	isp_ochn_attr_t *isp_ochn_attr = NULL;
	hbn_vnode_handle_t *isp_node_handle = NULL;
	hbn_buf_alloc_attr_t alloc_attr = {0};

	uint32_t ichn_id = 0;
	uint32_t ochn_id = 0;
	int ret = 0;
	sensor_config = pipe_contex->sensor_config;
	isp_attr = sensor_config->isp_attr;
	isp_ichn_attr = sensor_config->isp_ichn_attr;
	isp_ochn_attr = sensor_config->isp_ochn_attr;
	isp_node_handle = &pipe_contex->isp_node_handle;

	{
		isp_attr->channel.hw_id = hw_id;
		isp_attr->channel.slot_id = slot_id;
		isp_attr->sched_mode = mode;

		if(is_online) {
			isp_ochn_attr->stream_output_mode = STREAM_OUTPUT_MODE_ENABLE;  //0: 关闭Online  1: 使能online
			isp_ochn_attr->axi_output_mode = AXI_OUTPUT_MODE_DISABLE;       //0：不输出到DDR，其他值表示输出不同的格式到DDR
		}else{
			isp_ochn_attr->stream_output_mode = STREAM_OUTPUT_MODE_DISABLE;
			isp_ochn_attr->axi_output_mode = AXI_OUTPUT_MODE_YUV420;
		}
	}
	vp_csi_config_t *csi_config = &pipe_contex->csi_config;
	printf("\nINFO: ISP channel info: \n");
	printf("	input info: [mipi_rx: %d] [is_online: %d]\n", csi_config->mipi_rx, is_online);
	printf("	isp channel info: [hw_id: %d] [slot_id: %d] [mode:%d]\n",
		isp_attr->channel.hw_id, isp_attr->channel.slot_id, isp_attr->sched_mode);

	ret = hbn_vnode_open(HB_ISP, hw_id, AUTO_ALLOC_ID, isp_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_attr(*isp_node_handle, isp_attr);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ochn_attr(*isp_node_handle, ochn_id, isp_ochn_attr);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vnode_set_ichn_attr(*isp_node_handle, ichn_id, isp_ichn_attr);
	ERR_CON_EQ(ret, 0);

	if(!is_online) {
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

static int create_ynr_node(pipe_contex_t *pipe_contex, int slot_id, int work_mode) {
	int ret = 0;

	struct ynr_init_attr *attr;
	vp_sensor_config_t *sensor_config = NULL;
	hbn_vnode_handle_t *ynr_node_handle = NULL;

	ynr_node_handle = &pipe_contex->ynr_node_handle;
	sensor_config = pipe_contex->sensor_config;
	attr = sensor_config->ynr_attr;

	attr->work_mode = work_mode;
	attr->slot_id =  slot_id;

	int hw_id = 1; //固定为1
	ret = hbn_vnode_open(HB_YNR, hw_id, AUTO_ALLOC_ID, ynr_node_handle);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_attr(*ynr_node_handle, attr);
	ERR_CON_EQ(ret, 0);

	struct hobot_ynr_channel_input_config channel_input_cfg = {0};
	ret = hbn_vnode_set_ichn_attr(*ynr_node_handle, 0, &channel_input_cfg);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vnode_set_ichn_attr(*ynr_node_handle, 1, &channel_input_cfg);
	ERR_CON_EQ(ret, 0);

	struct hobot_ynr_channel_output_config channel_output_cfg = {0};
	ret = hbn_vnode_set_ochn_attr(*ynr_node_handle, 0, &channel_output_cfg);
	ERR_CON_EQ(ret, 0);

	if (attr->nr3d_en) {
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


int create_pym_node(pipe_contex_t *pipe_contex, int hw_id, int slot_id, int pym_mode, int pym_select_chn) {
	int ret = 0;
	int input_width = 0;
	int input_height = 0;
	{
		camera_config_t *camera_config = pipe_contex->sensor_config->camera_config;
		input_width = camera_config->width;
		input_height = camera_config->height;
	}


	pym_cfg_t pym_cfg = {0};
	pym_cfg.hw_id = hw_id;
	pym_cfg.pym_mode = pym_mode;
	pym_cfg.slot_id = slot_id;

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
	for(int i = 0; i < MAX_DS_NUM; i++) {
		//限制1：SRC和BL缩小倍数固定，并且长和宽缩小比例相同
		ratio = 1 << i;// 缩小倍数：1(Src层)，2(BL0)，4(BL1)，8(BL2)，16(BL3)，32(BL4)

		//限制2：SRC和BL的输出最小为 32*32
		int bl_width = FLOOR_ALIGN_2(input_width / ratio);
		int bl_height = FLOOR_ALIGN_2(input_height / ratio);
		if(bl_width < PYM_MIN_WIDTH) {
			printf("[Error] ochn[%d] ratio= %d, width = %d < PYM_MIN_WIDTH(%d), so not enable.\n",
				i, ratio, bl_width, PYM_MIN_WIDTH);
			continue;
		}
		if(bl_height < PYM_MIN_HEIGHT) {
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

		//保存每个通道的输出，为下个模块使用
		node_out_info_t* node_out_info = &pipe_contex->pym_out_info[i];
		node_out_info->width = roi_box->out_width;
		node_out_info->height = roi_box->out_height;

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

	if(pym_cfg.output_buf_num > 0) {
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


int create_and_run_vflow(pipe_contex_t *pipe_contex) {
	int32_t ret = 0;
	int link_port = g_link_port;
	pipeline_channel_info_t *ch_info = &g_ch_info;
	pipeline_usage_scene_type_t scene_type = g_scene_type;

	// 创建 pipeline 中的每个 node
	ret = create_camera_node(pipe_contex, link_port);
	ERR_CON_EQ(ret, 0);

	int sensor_type = pipe_contex->sensor_config->sensor_type;
	if(scene_type == PIPELINE_SCENE_ISP_ONLY) {

		ret = create_vin_node(pipe_contex, ch_info->is_online_vin_isp, link_port);
		ERR_CON_EQ(ret, 0);

		ret = create_isp_node(pipe_contex, ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode, ch_info->is_online_isp_pym);
		ERR_CON_EQ(ret, 0);
	}else if(scene_type == PIPELINE_SCENE_ISP_YNR) {
		ret = create_vin_node(pipe_contex, ch_info->is_online_vin_isp, link_port);
		ERR_CON_EQ(ret, 0);

		ret = create_isp_node(pipe_contex, ch_info->isp_hw_id, ch_info->isp_slot_id, ch_info->isp_mode, ch_info->is_online_isp_ynr);
		ERR_CON_EQ(ret, 0);

		ret = create_ynr_node(pipe_contex, ch_info->ynr_slot_id, ch_info->ynr_mode); //1: Manaul 模式  2: 全 online模式
		ERR_CON_EQ(ret, 0);
	}else{
		ret = create_vin_node(pipe_contex, ch_info->is_online_vin_pym, link_port);
		ERR_CON_EQ(ret, 0);
	}
	ret = create_pym_node(pipe_contex,
		ch_info->pym_hw_id, ch_info->pym_slot_id, ch_info->pym_mode, -1);
	ERR_CON_EQ(ret, 0);

	// 2. 添加node 到 flow
	ret = hbn_vflow_create(&pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);

	if(scene_type == PIPELINE_SCENE_ISP_ONLY) {
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle);
		ERR_CON_EQ(ret, 0);
	}else if(scene_type == PIPELINE_SCENE_ISP_YNR) {
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle);
		ERR_CON_EQ(ret, 0);
		ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
								pipe_contex->ynr_node_handle);
		ERR_CON_EQ(ret, 0);
	}else{
		//do nothing
	}
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->pym_node_handle);
	ERR_CON_EQ(ret, 0);

	// 3. 绑定 Flow 中的Node
	if(scene_type == PIPELINE_SCENE_ISP_BYPASS) {
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->vin_node_handle,
								ch_info->is_online_vin_pym,
								pipe_contex->pym_node_handle,
								0);
		ERR_CON_EQ(ret, 0);
	}else if(scene_type == PIPELINE_SCENE_ISP_ONLY) {
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->vin_node_handle,
								ch_info->is_online_vin_isp,
								pipe_contex->isp_node_handle,
								0);
		ERR_CON_EQ(ret, 0);

		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
						pipe_contex->isp_node_handle,
						ch_info->is_online_isp_pym,
						pipe_contex->pym_node_handle,
						0);
		ERR_CON_EQ(ret, 0);

	}else if(scene_type == PIPELINE_SCENE_ISP_YNR) {
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->vin_node_handle,
								ch_info->is_online_vin_isp,
								pipe_contex->isp_node_handle,
								0);
		ERR_CON_EQ(ret, 0);

		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
						pipe_contex->isp_node_handle,
						ch_info->is_online_isp_ynr,
						pipe_contex->ynr_node_handle,
						0);
		ERR_CON_EQ(ret, 0);

		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
						pipe_contex->ynr_node_handle,
						ch_info->is_online_ynr_pym,
						pipe_contex->pym_node_handle,
						0);
		ERR_CON_EQ(ret, 0);
	}else{
		//error
	}


	if(pipe_contex->sensor_config->sensor_type != SENSOR_TYPE_NORMAL) {
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
/////////////////////////////////////////////// Debug ////////////////////////////////////////////
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

/////////////////////////////////////////////// Thread ////////////////////////////////////////////
void *read_pym_and_encode_data(void *context) {

	////////////////////////////// HBN /////////////////////////////////
	pipe_contex_t *pipe_context = (pipe_contex_t *)context;
	hbn_vnode_handle_t pym_node_handle = pipe_context->pym_node_handle;
	hbn_vnode_image_group_t out_img_group = {0};
	char dst_file[128] = {0};
	uint32_t count = 0;
	int ret = 0;

	//////////////////////////// VPU ///////////////////////////////////
	media_codec_buffer_t input_buffer = {0};
	media_codec_buffer_t ouput_buffer = {0};
	media_codec_output_buffer_info_t info;
	FILE *fp_output = fopen("single_pipe_vin_isp_ynr_pym_vpu.h264", "w+b");
	if (NULL == fp_output) {
		printf("Failed to open output file\n");
	}

	uint8_t uuid[] = "dc45e9bd-e6d948b7-962cd820-d923eeef+SEI_D-Robotics";

	uint32_t length = sizeof(uuid)/sizeof(uuid[0]);
	ret = hb_mm_mc_insert_user_data(&g_media_context, uuid, length);
	if (ret != 0) {
		printf("#### insert user data failed. ret(%d) ####\n", ret);
		return NULL;
	}

	/////////////////////////////////////////////////////////////////
	while (running) {
		ret = hbn_vnode_getframe_group(pym_node_handle, 0, 3000, &out_img_group);
		if(ret != 0) {
			printf("hbn_vnode_getframe_group failed.\n");
			break;
		}
		////////////////////////// Save H264/H265 //////////////////////////////////////
		memset(&input_buffer, 0x00, sizeof(media_codec_buffer_t));
		ret = hb_mm_mc_dequeue_input_buffer(&g_media_context, &input_buffer,
											2000);
		if (ret != 0) {
			printf("hb_mm_mc_dequeue_input_buffer failed\n");
			break;
		}
		hb_mem_graphic_buf_t* out_img_for_codec = &out_img_group.buf_group.graph_group[0];

		int img_width =out_img_for_codec->width;
		int img_height =out_img_for_codec->height;
		memcpy(input_buffer.vframe_buf.vir_ptr[0],out_img_for_codec->virt_addr[0],
			img_width * img_height * 3 / 2);
		ret = hb_mm_mc_queue_input_buffer(&g_media_context, &input_buffer, 2000);
		if (ret != 0) {
			printf("hb_mm_mc_queue_input_buffer failed\n");
			break;
		}
		memset(&ouput_buffer, 0x0, sizeof(media_codec_buffer_t));
		memset(&info, 0x0, sizeof(media_codec_output_buffer_info_t));
		ret = hb_mm_mc_dequeue_output_buffer(&g_media_context, &ouput_buffer,
											&info, 2000);
		if (ret != 0) {
			printf("hb_mm_mc_dequeue_output_buffer failed\n");
			break;
		}
		fwrite(ouput_buffer.vstream_buf.vir_ptr,
				ouput_buffer.vstream_buf.size, 1, fp_output);
		// printf("count:%d\n", count);
		ret = hb_mm_mc_queue_output_buffer(&g_media_context,
											&ouput_buffer, 2000);
		if (ret != 0) {
			printf("hb_mm_mc_queue_output_buffer failed\n");
			break;
		}

		////////////////////////// Save NV12 //////////////////////////////////////
		if (count % 60 == 0) {
			for (uint32_t i = 0; i < MAX_DS_NUM; ++i) {
				hbn_frame_info_t *img_info = &out_img_group.info;
				hb_mem_graphic_buf_t* out_img = &out_img_group.buf_group.graph_group[i];
				snprintf(dst_file, sizeof(dst_file), "pym_output_nv12_chn%d_%dx%d_stride_%d_count_%d.yuv",
					i, out_img->width, out_img->height, out_img->stride, count);
				dump_2plane_yuv_to_file(dst_file,
					out_img->virt_addr[0],
					out_img->virt_addr[1],
					out_img->size[0],
					out_img->size[1]);
				printf("####################### pym chn %d #######################\n", i);
				printf("=== Frame Info ===\n");
				vp_vin_print_hbn_frame_info_t(img_info);
				printf("\n=== Graphic Buffer ===\n");
				vp_vin_print_hb_mem_graphic_buf_t(out_img);
			}
		}

		hbn_vnode_releaseframe_group(pym_node_handle, 0, &out_img_group);

		count++;
	}
	fclose(fp_output);
	return NULL;
}

/////////////////////////////////////////////// VPU ////////////////////////////////////////////

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

	ret = vp_encode_config_param(&g_media_context, encode_type,
								encode_width, encode_height,
								encode_fps, 8192);
	ERR_CON_EQ(ret, 0);
	ret = hb_mm_mc_initialize(&g_media_context);
	ERR_CON_EQ(ret, 0);
	ret = hb_mm_mc_configure(&g_media_context);
	ERR_CON_EQ(ret, 0);
	ret = hb_mm_mc_start(&g_media_context, &startup_params);
	printf("%s idx: %d, init successful\n",
			g_media_context.encoder ? "Encode" : "Decode",
			g_media_context.instance_index);
	return 0;
}

int encode_deinit(void *data) {
	int ret = 0;
	ret = hb_mm_mc_pause(&g_media_context);
	ERR_CON_EQ(ret, 0);
	ret = hb_mm_mc_release(&g_media_context);
	ERR_CON_EQ(ret, 0);

	return 0;
}
