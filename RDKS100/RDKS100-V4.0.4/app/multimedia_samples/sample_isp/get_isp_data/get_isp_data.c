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

#define MAX_SENSORS 4

static struct option const long_options[] = {
	{"sensor", required_argument, NULL, 's'},
	{"online", optional_argument, NULL, 'o'},
	{"link_port", required_argument, NULL, 'l'},
	{NULL, 0, NULL, 0}
};

static int create_and_run_vflow(pipe_contex_t *pipe_contex);
static void handle_user_command(pipe_contex_t *pipe_contex);
int32_t hbn_deserial_create(deserial_config_t *des_config, deserial_handle_t *des_fd);
int32_t hbn_deserial_attach_to_vin(deserial_handle_t des_fd, camera_des_link_t link, vpf_handle_t vin_fd);

static void print_help() {
	printf("Usage: get_isp_data [OPTIONS]\n");
	printf("Options:\n");
	printf("  -s <sensor_index>      Specify sensor index\n");
	printf("  -o <online>            Specify the connection method from VIN to ISP, 1: online 0: offline \n");
	printf("  -l <link_port>         Specify the port for connecting serdes sensors, 0:A 1:B 2:C 3:D\n");
	printf("  -h                     Show this help message\n");
	vp_show_sensors_list(); // Assuming this function displays sensor list
}

static void command_help() {
	printf("\n");
	printf("***************  Command Lists  ***************\n");
	printf(" g	-- get single frame \n");
	printf(" l	-- get a set frames \n");
	printf(" q	-- quit  \n");
	printf(" h	-- print help message\n");
}

static int is_online = 0;
static uint32_t link_port = 0;
static uint32_t sensor_type = 0;

int main(int argc, char** argv) {
	int ret = 0;
	pipe_contex_t pipe_contex = {0};
	int opt_index = 0;
	int c = 0;
	int sensor_index = -1;
	bool link_port_specified = false;

	while((c = getopt_long(argc, argv, "s:o:l:h", //"s:o:t:m:h"
							long_options, &opt_index)) != -1) {
		switch (c)
		{
		case 's':
			sensor_index = atoi(optarg);
			break;
		case 'o':
			is_online = atoi(optarg);
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

	if (sensor_index == -1) {
		printf("No sensors specified.\n");
		print_help();
		return 0;
	}
	int used_mipi_host = 0;
	if (sensor_index < vp_get_sensors_list_number() && sensor_index >= 0) {
		pipe_contex.sensor_config = vp_sensor_config_list[sensor_index];
		printf("Using index:%d  sensor_name:%s  config_file:%s\n",
				sensor_index,
				vp_sensor_config_list[sensor_index]->sensor_name,
				vp_sensor_config_list[sensor_index]->config_file);

		sensor_type = pipe_contex.sensor_config->sensor_type;
		if (sensor_type != SENSOR_TYPE_NORMAL && !link_port_specified) {
			print_help();
			printf("[Error] %s is gsml sensor, must config link port according to the hardware connection, can be set to [0-3]\n",
				vp_sensor_config_list[sensor_index]->sensor_name);

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
		}else {
			pipe_contex.csi_config.mipi_rx = 4;
		}
		if(sensor_type == SENSOR_TYPE_GMSL_YUV){
			printf("%s output yuv format image, can't run this sample.\n",
				vp_sensor_config_list[sensor_index]->sensor_name);
		}
	} else {
		printf("Unsupported sensor index:%d\n", sensor_index);
		print_help();
		return 0;
	}

	if (sensor_type != SENSOR_TYPE_NORMAL && link_port_specified) {
		camera_config_t *camera_config = pipe_contex.sensor_config->camera_config;
		vp_deserial_config_update(camera_config, link_port);
		if(is_online){
			printf("[Error] Serdes sensors can only run in offline mode.\n");
			return -1;
		}
	}

	hb_mem_module_open();

	ret = create_and_run_vflow(&pipe_contex);
	if (ret != 0) {
		printf("create_and_run_vflow failed for sensor %d. ret = %d\n", sensor_index, ret);
		return ret;
	}

	handle_user_command(&pipe_contex);

	ret = hbn_vflow_stop(pipe_contex.vflow_fd);
	if (ret != 0) {
		printf("hbn_vflow_stop failed for sensor %d. ret = %d\n", sensor_index, ret);
	}
	hbn_vnode_close(pipe_contex.vin_node_handle);
	hbn_vnode_close(pipe_contex.isp_node_handle);
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

	if(sensor_config->sensor_type == SENSOR_TYPE_NORMAL){
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
		ret = get_isp_channel_config_for_single_pipeline(
			csi_config->mipi_rx, is_online, &isp_attr->channel
		);
		ERR_CON_EQ(ret, 0);

		//is_online
		if(is_online){
			isp_attr->sched_mode = SCHED_MODE_PASS_THRU;
		}else{
			isp_attr->sched_mode = SCHED_MODE_MANUAL;
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

	alloc_attr.buffers_num = 3;
	alloc_attr.is_contig = 1;
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
						| HB_MEM_USAGE_CPU_WRITE_OFTEN
						| HB_MEM_USAGE_CACHED;
	ret = hbn_vnode_set_ochn_buf_attr(*isp_node_handle, ochn_id, &alloc_attr);
	ERR_CON_EQ(ret, 0);

	return 0;
}


int create_and_run_vflow(pipe_contex_t *pipe_contex) {
	int32_t ret = 0;

	// 创建 pipeline 中的每个 node
	ret = create_camera_node(pipe_contex);
	ERR_CON_EQ(ret, 0);

	ret = create_vin_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	ret = create_isp_node(pipe_contex);
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

	if(is_online){
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
								pipe_contex->vin_node_handle,
								1,
								pipe_contex->isp_node_handle,
								0);
	}else{
		ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
						pipe_contex->vin_node_handle,
						0,
						pipe_contex->isp_node_handle,
						0);
	}

	ERR_CON_EQ(ret, 0);

	if(sensor_type != SENSOR_TYPE_NORMAL) {
		ret = create_deserial_node(pipe_contex);
		ERR_CON_EQ(ret, 0);
		ret = hbn_camera_attach_to_deserial(pipe_contex->cam_fd, pipe_contex->des_fd, link_port);
		ERR_CON_EQ(ret, 0);
		ret = hbn_deserial_attach_to_vin(pipe_contex->des_fd, link_port, pipe_contex->vin_node_handle);
		ERR_CON_EQ(ret, 0);
	}
	else
	{
		ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd,
							pipe_contex->vin_node_handle);
		ERR_CON_EQ(ret, 0);
	}

	ret = hbn_vflow_start(pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);

	return 0;
}

static void isp_dump_func(hbn_vnode_handle_t isp_node_handle) {
	int ret;
	char dst_file[128];
	uint32_t ochn_id = 0;
	uint32_t timeout = 10000;
	hbn_vnode_image_group_t out_img_group;

	// 调用 hbn_vnode_getframe 获取帧数据
	ret = hbn_vnode_getframe_group(isp_node_handle, ochn_id, timeout, &out_img_group);
	if (ret != 0) {
		printf("hbn_vnode_getframe from isp chn:%d failed(%d)\n", ochn_id, ret);
		return;
	}
	hbn_frame_info_t *info = &out_img_group.info;
	hb_mem_graphic_buf_t* out_img = &out_img_group.buf_group.graph_group[ochn_id];

	// 将帧数据写入文件
	snprintf(dst_file, sizeof(dst_file),
		"handle_%d_isp_chn%d_%dx%d_stride_%d_frameid_%d_ts_%ld.yuv",
		(int)isp_node_handle, ochn_id,
		out_img->width, out_img->height, out_img->stride,
		info->frame_id, info->timestamps);
	printf("handle %d isp dump yuv %dx%d(stride:%d), buffer size: %ld + %ld frame id: %d,"
			" timestamp: %ld\n",
			(int)isp_node_handle,
			out_img->width, out_img->height,
			out_img->stride,
			out_img->size[0], out_img->size[1],
			info->frame_id,
			info->timestamps);
	dump_2plane_yuv_to_file(dst_file,
			out_img->virt_addr[0],
			out_img->virt_addr[1],
			out_img->size[0],
			out_img->size[1]);

	// 释放帧数据
	hbn_vnode_releaseframe_group(isp_node_handle, ochn_id, &out_img_group);


}

static void handle_user_command(pipe_contex_t *pipe_contex)
{
	int i = 0, j = 0;
	char option = 'a';
	hbn_vnode_handle_t isp_node_handle;
	int running = -1;

	command_help();
	printf("\nCommand: ");

	while (running && ((option=getchar()) != EOF)) {
		switch (option) {
			case 'q':
				printf("quit\n");
				running = 0;
				return;
			case 'g':  // get a isp yuv file for all sensors
				isp_node_handle = pipe_contex->isp_node_handle;
				isp_dump_func(isp_node_handle);
				break;
			case 'l':// get multiple frames for all sensors
				for (j = 0; j < 12; j++) {
					isp_node_handle = pipe_contex->isp_node_handle;
					isp_dump_func(isp_node_handle);
				}
				break;
			case 'h':
				command_help();
				break;
			case '\n':
			case '\r':
				continue;
			default:
				printf("Command does not supported!\n");
				command_help();
				break;
		}
		printf("\nCommand: ");
	}

	return;
}
