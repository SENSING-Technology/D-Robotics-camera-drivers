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
	{"mode", optional_argument, NULL, 'm'},
	{"link_port", required_argument, NULL, 'l'},
	{NULL, 0, NULL, 0}
};

static int create_and_run_vflow(pipe_contex_t *pipe_contex);
static void handle_user_command(pipe_contex_t *pipe_contex);
int32_t hbn_deserial_create(deserial_config_t *des_config, deserial_handle_t *des_fd);
int32_t hbn_deserial_attach_to_vin(deserial_handle_t des_fd, camera_des_link_t link, vpf_handle_t vin_fd);


static void print_help() {
	printf("Usage: get_vin_data [OPTIONS]\n");
	printf("Options:\n");
	printf("  -s <sensor_index>      Specify sensor index\n");
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

static uint32_t link_port = 0;
static uint32_t sensor_type = 0;
static uint16_t date_type;

int main(int argc, char** argv) {
	int ret = 0;
	pipe_contex_t pipe_contex = {0};
	int opt_index = 0;
	int c = 0;
	int sensor_index = -1;
	bool link_port_specified = false;

	while((c = getopt_long(argc, argv, "s:o:t:m:l:h",
							long_options, &opt_index)) != -1) {
		switch (c)
		{
		case 's':
			sensor_index = atoi(optarg);
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
		date_type = pipe_contex.sensor_config->vin_ichn_attr->format;

		if (sensor_type != SENSOR_TYPE_NORMAL && !link_port_specified) {
			print_help();
			printf("[Error] %s is serdes sensor, must config link port according to the hardware connection, can be set to [0-3]\n",
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
		} else {
			pipe_contex.csi_config.mipi_rx = 4;
		}

	} else {
		printf("Unsupported sensor index:%d\n", sensor_index);
		print_help();
		return 0;
	}

	if (sensor_type != SENSOR_TYPE_NORMAL && link_port_specified) {
		camera_config_t *camera_config = pipe_contex.sensor_config->camera_config;
		vp_deserial_config_update(camera_config, link_port);
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
	hbn_deserial_destroy(pipe_contex.des_fd);
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
		//不同的sensor和加串要重新map成不一样的地址,不然相同的地址解串器没法和他们通讯,
		//所以根据link_port动态设定eeprom_addr serial_addr addr
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

	ret = hbn_deserial_create(deserial_config, &pipe_contex->des_fd);
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
		vin_node_attr->cim_attr.cim_isp_flyby  = 0;

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

int create_and_run_vflow(pipe_contex_t *pipe_contex) {
	int32_t ret = 0;
	// 创建 pipeline 中的每个 node
	ret = create_camera_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	ret = create_vin_node(pipe_contex);
	ERR_CON_EQ(ret, 0);

	// 创建 HBN flow
	ret = hbn_vflow_create(&pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->vin_node_handle);
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

static void vin_dump_func(hbn_vnode_handle_t vin_node_handle) {
	int ret;
	char dst_file[128];
	uint32_t ochn_id = 0;
	uint32_t timeout = 10000;
	hbn_vnode_image_t out_img = {0};

	ret = hbn_vnode_getframe(vin_node_handle, ochn_id, timeout, &out_img);
	if (ret != 0) {
		printf("hbn_vnode_getframe %d CIM failed(%d)\n", ochn_id, ret);
		return;
	}

	// 生成基础文件名
	snprintf(dst_file, sizeof(dst_file),
			 "handle_%d_chn%d_%dx%d_stride_%d_frameid_%d_ts_%ld",
			 (int)vin_node_handle, ochn_id,
			 out_img.buffer.width, out_img.buffer.height, out_img.buffer.stride,
			 out_img.info.frame_id, out_img.info.timestamps);

	// 处理不同数据格式
	switch (date_type) {
		case SENSOR_DATA_TYPE_RAW10:
		case SENSOR_DATA_TYPE_RAW12:
			strcat(dst_file, ".raw");
			printf("Dumping RAW data: handle %d, resolution: %dx%d (stride: %d), size: %ld, frame id: %d, timestamp: %ld\n",
					(int)vin_node_handle,
					out_img.buffer.width, out_img.buffer.height,
					out_img.buffer.stride,
					out_img.buffer.size[0],
					out_img.info.frame_id,
					out_img.info.timestamps);
			dump_image_to_file(dst_file, out_img.buffer.virt_addr[0], out_img.buffer.size[0]);
			break;
		case SENSOR_DATA_TYPE_YUV422:
			strcat(dst_file, ".yuv");
			printf("Dumping YUV data: handle %d, resolution: %dx%d (stride: %d), size: %ld + %ld, frame id: %d, timestamp: %ld\n",
					(int)vin_node_handle,
					out_img.buffer.width, out_img.buffer.height,
					out_img.buffer.stride,
					out_img.buffer.size[0], out_img.buffer.size[1],
					out_img.info.frame_id,
					out_img.info.timestamps);
			ret = dump_2plane_yuv_to_file(dst_file,
					out_img.buffer.virt_addr[0],
					out_img.buffer.virt_addr[1],
					out_img.buffer.size[0],
					out_img.buffer.size[1]);
			if (ret != 0) {
				printf("Error: Failed to dump YUV to file\n");
			} else {
				printf("Dump successful: %s (size: %ld)\n", dst_file, sizeof(dst_file));
			}
			break;
		default:
			printf("Error: Unsupported data type %d\n", out_img.buffer.format);
			break;
	}

	// 释放帧数据
	hbn_vnode_releaseframe(vin_node_handle, ochn_id, &out_img);
}

static void handle_user_command(pipe_contex_t *pipe_contex)
{
	int j = 0;
	char option = 'a';
	hbn_vnode_handle_t vin_node_handle;
	int running = -1;

	command_help();
	printf("\nCommand: ");

	while (running && ((option=getchar()) != EOF)) {
		switch (option) {
			case 'q':
				printf("quit\n");
				running = 0;
				return;
			case 'g':  // get a vin file for all sensors
				vin_node_handle = pipe_contex->vin_node_handle;
				vin_dump_func(vin_node_handle);
				break;
			case 'l':// get multiple frames for all sensors
				for (j = 0; j < 12; j++) {
					vin_node_handle = pipe_contex->vin_node_handle;
					vin_dump_func(vin_node_handle);
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
