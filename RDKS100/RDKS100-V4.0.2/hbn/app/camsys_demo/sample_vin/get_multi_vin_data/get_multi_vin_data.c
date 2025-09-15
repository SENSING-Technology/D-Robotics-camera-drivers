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
#include <ctype.h>

#include "common_utils.h"
#include "hb_media_codec.h"
#include "hb_media_error.h"

#define MAX_PIPE_NUM 6

typedef struct {

	int pipeline_index;
	//GSML
	int link_port;

	//sensor config
	uint32_t sensor_mode;
	int select_sensor_id;

	//pym
	int pym_select_chn;
	int pym_output_width;
	int pym_output_height;

	//hbn
	pipe_contex_t pipe_contexts;

	//codec
	char encode_type[32];
	char output_file[256];
	pthread_t read_codec_thread;
	media_codec_context_t media_context;
} pipeline_info_t;


static int32_t running = 0;
static int32_t verbose_flag = 1;
static deserial_handle_t g_des_fd = -1;
static pipeline_info_t g_pipeline_info[MAX_PIPE_NUM] = {0};

static struct option const long_options[] = {
	{"config", required_argument, NULL, 'c'},
	{"help", no_argument, NULL, 'h'},
	{NULL, 0, NULL, 0}
};

static void print_help(void) {
	printf("Usage: %s [Options]\n", get_program_name());
	printf("Options:\n");
	printf("-c, --config=\"sensor=id\"\n");
	printf("\t\tConfigure parameters for each video pipeline, can be repeated up to %d times.\n", MAX_PIPE_NUM);
	printf("\t\tsensor   --  Sensor index,can have multiple parameters, reference sensor list.\n");
	printf("\t\tmode     --  Sensor mode of camera_config_t\n");
	printf("\t\tlink     --  Sensor link port number, gsml sensor must be configured according to the hardware connection, can be set to [0-3].\n");
	printf("-h, --help\tShow help message\n");
	printf("Support sensor list:\n");
	vp_show_sensors_list();
}

void signal_handle(int signo) {
	running = 0;
}

static int is_number(const char *str) {
	while (*str) {
		if (!isdigit(*str)) return 0;
		str++;
	}
	return 1;
}

// 分割字符串并返回数组的个数
static int split_string(const char *str, const char *delim, char *out[], int max_parts) {
	int count = 0;
	char *token;
	char *str_copy = strdup(str);
	char *rest = str_copy;

	while ((token = strtok_r(rest, delim, &rest)) && count < max_parts) {
		out[count++] = strdup(token);
	}

	free(str_copy);
	return count;
}

int parse_config(pipeline_info_t *pipeline_info, const char *config, int pipeline_idx) {
	int ret = 0;
	char *parts[4];
	int sensor_idx = -1;
	int is_output_set = 0;
	int count = split_string(config, " ", parts, 4);
	static int32_t used_mipi_host = 0;

	// 默认值
	uint32_t sensor_type = SENSOR_TYPE_NORMAL;
	pipeline_info->link_port = -1;
	pipeline_info->pym_select_chn = 0;
	pipeline_info->pipeline_index = pipeline_idx;
	strncpy(pipeline_info->encode_type, "h264", sizeof(pipeline_info->encode_type));

	for (int i = 0; i < count; i++) {
		char *key_value[2];
		int kv_count = split_string(parts[i], "=", key_value, 2);
		if (kv_count != 2) {
			fprintf(stderr, "Invalid format in config: %s\n", parts[i]);
			continue;
		}

		if (strcmp(key_value[0], "sensor") == 0) {
			if (!is_number(key_value[1])) {
				fprintf(stderr, "Invalid sensor ID: %s\n", key_value[1]);
				continue;
			}
			sensor_idx = atoi(key_value[1]);

			if (sensor_idx < vp_get_sensors_list_number() && sensor_idx >= 0) {
				pipeline_info->pipe_contexts.sensor_config = vp_sensor_config_list[sensor_idx];
				printf("Using index:%d  sensor_name:%s  config_file:%s\n",
						sensor_idx,
						vp_sensor_config_list[sensor_idx]->sensor_name,
						vp_sensor_config_list[sensor_idx]->config_file);
				sensor_type = pipeline_info->pipe_contexts.sensor_config->sensor_type;
				// printf("sensor_type:%d \n" , sensor_type);
			} else {
				printf("Unsupport sensor index:%d\n", sensor_idx);
				print_help();
				exit(0);
			}
			//gmsl 模组目前没办法探测
			if(sensor_type == SENSOR_TYPE_NORMAL) {
				ret = vp_sensor_multi_fixed_mipi_host(pipeline_info->pipe_contexts.sensor_config, &used_mipi_host,
													&pipeline_info->pipe_contexts.csi_config);
				if (ret < 0) {
					printf("vp sensor fixed mipi host fail, sensor id %d."
						"Maybe No Camera Sensor found. Please check if the specified "
						"sensor is connected to the Camera interface.\n\n", sensor_idx);
					exit(0);
				}
				pipeline_info->select_sensor_id = sensor_idx;
			}else{
				pipeline_info->pipe_contexts.csi_config.mipi_rx = 4; //RDKS100中 GSML相机固定接到 mipi_rx4
			}
		} else if (strcmp(key_value[0], "channel") == 0) {
			if (!is_number(key_value[1])) {
				fprintf(stderr, "Invalid channel ID: %s\n", key_value[1]);
				continue;
			}
			pipeline_info->pym_select_chn = atoi(key_value[1]);
		}else if (strcmp(key_value[0], "link") == 0) {
			if (!is_number(key_value[1])) {
				fprintf(stderr, "Invalid link port: %s\n", key_value[1]);
				continue;
			}

			int link_port = atoi(key_value[1]);
			if((link_port < 0) || (link_port > 3)){
				fprintf(stderr, "Invalid link port: %d, can be set to [0-3]\n", link_port);
				continue;
			}
			pipeline_info->link_port = link_port;
		} else if (strcmp(key_value[0], "mode") == 0) {
			if (!is_number(key_value[1])) {
				fprintf(stderr, "Invalid sensor mode number: %s\n", key_value[1]);
				continue;
			}
			pipeline_info->sensor_mode = atoi(key_value[1]);
		} else if (strcmp(key_value[0], "type") == 0) {
			strncpy(pipeline_info->encode_type, key_value[1], sizeof(pipeline_info->encode_type) - 1);
			pipeline_info->encode_type[sizeof(pipeline_info->encode_type) - 1] = '\0';
		} else if (strcmp(key_value[0], "output") == 0) {
			strncpy(pipeline_info->output_file, key_value[1], sizeof(pipeline_info->output_file) - 1);
			pipeline_info->output_file[sizeof(pipeline_info->output_file) - 1] = '\0';
			is_output_set = 1;
		} else {
			fprintf(stderr, "Unknown key: %s\n", key_value[0]);
		}

		for (int j = 0; j < kv_count; j++) {
			free(key_value[j]);
		}
	}

	if((sensor_type != SENSOR_TYPE_NORMAL) && (pipeline_info->link_port == -1)){
		printf("[Error] %s is gsml sensor, must config link port according to the hardware connection, can be set to [0-3]\n",
			vp_sensor_config_list[sensor_idx]->sensor_name);
		return -1;
	}

	// printf("MIPI host: 0x%x\n", used_mipi_host);
	// for (int i = 0; i < MAX_PIPE_NUM; i++) {
	// 	if (used_mipi_host & (1 << i)) {
	// 		printf("  Host %d: Used\n", i);
	// 	}
	// }

	// set default output file name
	if (is_output_set == 0) {
		sprintf(pipeline_info->output_file, "pipeline%d_%dx%d_%dfps.%s",
			pipeline_idx, pipeline_info->pipe_contexts.sensor_config->camera_config->width,
			pipeline_info->pipe_contexts.sensor_config->camera_config->height,
			pipeline_info->pipe_contexts.sensor_config->camera_config->fps,
			pipeline_info->encode_type);
	}

	for (int i = 0; i < count; i++) {
		free(parts[i]);
	}
	return 0;
}

static int create_deserial_node(deserial_handle_t *des_handle, const deserial_config_t *deserial_config_const) {
	int32_t ret = 0;
	deserial_config_t deserial_config = {0};
	memcpy(&deserial_config, deserial_config_const, sizeof(deserial_config_t));

	ret = hbn_deserial_create(&deserial_config, des_handle);
	ERR_CON_EQ(ret, 0);

	if (verbose_flag) {
		printf("deserial_config:%02x_%s, des_handle:%ld \n\r" ,deserial_config.addr,
		deserial_config.name, *des_handle);
	}
	return 0;
}

static int create_camera_node(pipe_contex_t *pipe_contex, uint32_t sensor_mode, int link_port)
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
		if(is_online){
			vin_ochn_attr->ddr_en = 0;
			vin_node_attr->cim_attr.cim_isp_flyby  = 1;
		}else{
			vin_ochn_attr->ddr_en = 1;
			vin_node_attr->cim_attr.cim_isp_flyby  = 0;
		}
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

static int create_vflow(pipeline_info_t *pipeline_info)
{
	int32_t ret = 0;

	pipe_contex_t *pipe_contex = (pipe_contex_t *)&pipeline_info->pipe_contexts;

	// 1. 创建 node
	ret = create_camera_node(pipe_contex, pipeline_info->sensor_mode, pipeline_info->link_port);
	ERR_CON_EQ(ret, 0);

	int sensor_type = pipe_contex->sensor_config->sensor_type;

	ret = create_vin_node(pipe_contex, 0, pipeline_info->link_port);
	ERR_CON_EQ(ret, 0);

	// 2. 添加node 到 flow
	ret = hbn_vflow_create(&pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);

	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);

	if(sensor_type == SENSOR_TYPE_NORMAL){
		ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd,
							pipe_contex->vin_node_handle);
		ERR_CON_EQ(ret, 0);
	}

	return 0;
}

static int start_vflow(pipeline_info_t *pipeline_info){
	int ret = 0;
	pipe_contex_t *pipe_contex = (pipe_contex_t *)&pipeline_info->pipe_contexts;

	ret = hbn_vflow_start(pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);
	return ret;
}

void *encode_pipe_data(void *context)
{
	pipeline_info_t *pipeline_info = (pipeline_info_t *)context;
	pipe_contex_t *pipe_contex = (pipe_contex_t *)&pipeline_info->pipe_contexts;
	uint16_t date_type = pipe_contex->sensor_config->vin_ichn_attr->format;
	hbn_vnode_handle_t vin_node_handle = pipe_contex->vin_node_handle;
	uint32_t linkprot = pipeline_info->link_port;
	hbn_vnode_image_t out_img = {0};
	uint32_t ochn_id = 0;
	uint32_t count = 0;

	int ret = 0;
	while (running) {
		ret = hbn_vnode_getframe(vin_node_handle, 0, 1000, &out_img);
		if(ret != 0){
			printf("hbn_vnode_getframe_group failed.\n");
			while(1);
			break;
		}
		// 生成基础文件名
		snprintf(pipeline_info->output_file, sizeof(pipeline_info->output_file),
				"handle_%d_chn%d_%dx%d_stride_%d_frameid_%d_ts_%ld",
				(int)vin_node_handle, linkprot,
				out_img.buffer.width, out_img.buffer.height, out_img.buffer.stride,
				out_img.info.frame_id, out_img.info.timestamps);
		// 处理不同数据格式
		switch (date_type) {
			case SENSOR_DATA_TYPE_RAW10:
			case SENSOR_DATA_TYPE_RAW12:
				strcat(pipeline_info->output_file, ".raw");
				printf("Dumping RAW data: handle %d, resolution: %dx%d (stride: %d), size: %ld, frame id: %d, timestamp: %ld\n",
						(int)vin_node_handle,
						out_img.buffer.width, out_img.buffer.height,
						out_img.buffer.stride,
						out_img.buffer.size[0],
						out_img.info.frame_id,
						out_img.info.timestamps);
				dump_image_to_file(pipeline_info->output_file, out_img.buffer.virt_addr[0], out_img.buffer.size[0]);
				break;
			case SENSOR_DATA_TYPE_YUV422:
				strcat(pipeline_info->output_file, ".yuv");
				printf("Dumping YUV data: handle %d, resolution: %dx%d (stride: %d), size: %ld + %ld, frame id: %d, timestamp: %ld\n",
						(int)vin_node_handle,
						out_img.buffer.width, out_img.buffer.height,
						out_img.buffer.stride,
						out_img.buffer.size[0], out_img.buffer.size[1],
						out_img.info.frame_id,
						out_img.info.timestamps);
				ret = dump_2plane_yuv_to_file(pipeline_info->output_file,
						out_img.buffer.virt_addr[0],
						out_img.buffer.virt_addr[1],
						out_img.buffer.size[0],
						out_img.buffer.size[1]);
				if (ret != 0) {
					printf("Error: Failed to dump YUV to file\n");
				} else {
					printf("Dump successful: %s (size: %ld)\n", pipeline_info->output_file, sizeof(pipeline_info->output_file));
				}
				break;
			default:
				printf("Error: Unsupported data type %d\n", out_img.buffer.format);
				break;
		}

		// 释放帧数据
		hbn_vnode_releaseframe(vin_node_handle, ochn_id, &out_img);

		count++;
	}
	return NULL;
}

int main(int argc, char** argv) {
	int ret = 0;
	int c = 0;
	int index = -1;

	if (argc <= 1) {
		print_help();
		return 0;
	}
	int32_t total_pipeline_num = 0;
	while ((c = getopt_long(argc, argv, "c:h", long_options, NULL)) != -1) {
		switch (c) {
		case 'c':
			if (total_pipeline_num >= MAX_PIPE_NUM) {
				fprintf(stderr, "Too many configurations. Maximum allowed is %d.\n", MAX_PIPE_NUM);
				return 1;
			}
			ret = parse_config(&g_pipeline_info[total_pipeline_num], optarg, total_pipeline_num);
			if(ret != 0){
				return -1;
			}
			total_pipeline_num++;
			break;
		case 'h':
		default:
			print_help();
			return 0;
		}
	}

	// 处理后的参数在这里可以使用
	for (int i = 0; i < total_pipeline_num; i++) {
		printf("Pipeline index %d:\n", i);
		printf("\tSensor index: %d\n",g_pipeline_info[i].select_sensor_id);
		printf("\tSensor name: %s\n",g_pipeline_info[i].pipe_contexts.sensor_config->sensor_name);
		printf("\tActive mipi host: %d\n",g_pipeline_info[i].pipe_contexts.csi_config.mipi_rx);
	}
	printf("Verbose: %d\n", verbose_flag);

	hb_mem_module_open();
	for (index = 0; index < total_pipeline_num; index++) {
		ret = create_vflow(&g_pipeline_info[index]);
		if (ret != 0) {
			for (int j = 0; j < index; j++) {
				hbn_vflow_destroy(g_pipeline_info[j].pipe_contexts.vflow_fd);
			}
			return 0;
		}
	}
// for deserial sensor
	int deserial_sensor_count = 0;
	for (int i = 0; i < total_pipeline_num; i++) {
		if(g_pipeline_info[i].link_port == -1){
			continue;
		}
		const camera_config_t *camera_config = g_pipeline_info[i].pipe_contexts.sensor_config->camera_config;
		vp_deserial_config_update(camera_config, g_pipeline_info[i].link_port);
		deserial_sensor_count++;
	}
	if(deserial_sensor_count != 0){
		vp_deserial_config_show();
		const deserial_config_t *deserial_config_edited = vp_deserial_config_get();
		ret = create_deserial_node(&g_des_fd, deserial_config_edited);				// 解串器只有一个，只初始化一次
		ERR_CON_EQ(ret, 0);

		for (int i = 0; i < total_pipeline_num; i++) {
			pipeline_info_t *pipeline_info = &g_pipeline_info[i];
			pipe_contex_t *pipe_contex = (pipe_contex_t *)&pipeline_info->pipe_contexts;
			int sensor_type = pipe_contex->sensor_config->sensor_type;

			if(sensor_type == SENSOR_TYPE_NORMAL){
				continue;
			}

			ret = hbn_camera_attach_to_deserial(pipe_contex->cam_fd, g_des_fd, pipeline_info->link_port);
			ERR_CON_EQ(ret, 0);
			ret = hbn_deserial_attach_to_vin(g_des_fd, pipeline_info->link_port, pipe_contex->vin_node_handle);
			ERR_CON_EQ(ret, 0);
		}
	}

	for (int i = 0; i < total_pipeline_num; i++) {
		ret = start_vflow(&g_pipeline_info[i]);
		if (ret != 0) {
			printf("start_vflow fail for sensor ret = %d\n",  ret);
			return ret;
		}
	}

	running = 1;
	for (index = 0; index < total_pipeline_num; index++) {
		ret = pthread_create(&g_pipeline_info[index].read_codec_thread, NULL, (void *)encode_pipe_data,
							(void *)&g_pipeline_info[index]);
	}
	for (index = 0; index < total_pipeline_num; index++) {
		pthread_join(g_pipeline_info[index].read_codec_thread, NULL);
		ret = hbn_vflow_stop(g_pipeline_info[index].pipe_contexts.vflow_fd);
		ERR_CON_EQ(ret, 0);
		hbn_vflow_destroy(g_pipeline_info[index].pipe_contexts.vflow_fd);
	}
	hb_mem_module_close();

	return 0;
}
