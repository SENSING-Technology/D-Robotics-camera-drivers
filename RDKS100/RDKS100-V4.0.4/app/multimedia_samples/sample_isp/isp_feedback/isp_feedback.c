/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright(C) 2024-2025, D-Robotics Co., Ltd.
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
#include "cJSON.h"
#include "common_utils.h"


/* Defined at vp_sensor/dummy_sensor */
extern vp_sensor_config_t dummy_sensor_config;

static struct option const long_options[] = {
	{"file", required_argument, 0, 'f'},
	{"format", required_argument, NULL, 'F'},
	{"height", required_argument, NULL, 'H'},
	{"width", required_argument, NULL, 'W'},
	{"loop", optional_argument, NULL, 'l'},
	{NULL, 0, NULL, 0}
};

static void print_help() {
	printf("Usage: %s [OPTIONS]\n", get_program_name());
	printf("Options:\n");
	printf("  -f <file>              Specify Raw filename\n");
	printf("  -F <format>            Specify Raw format eg: raw8 raw10 raw12\n");
	printf("  -W <width>             Specify Raw width\n");
	printf("  -H <height>            Specify Raw height\n");
	printf("  -l <loop>              Specify feedback Raw loop\n");
	printf("  -h                     Show this help message\n");
}

static int create_camera_node(pipe_contex_t *pipe_contex) {

	camera_config_t *camera_config = NULL;
	vp_sensor_config_t *sensor_config = NULL;
	int32_t ret = 0;
	sensor_config = pipe_contex->sensor_config;
	camera_config = sensor_config->camera_config;
	printf("Creating camera with config: width=%d, height=%d, format=%d\n",
		   camera_config->width, camera_config->height, camera_config->format);
	ret = hbn_camera_create(camera_config, &pipe_contex->cam_fd);
	ERR_CON_EQ(ret, 0);

	return 0;
}

void update_sensor_param_data_width(camera_config_t *camera_config, int bit_width) {

	if (!camera_config || !camera_config->sensor_param)
		return;

	char *json_copy = strdup(camera_config->sensor_param);
	if (!json_copy) {
		fprintf(stderr, "Failed to allocate memory for sensor_param copy\n");
		return;
	}

	cJSON *root = cJSON_Parse(json_copy);
	free(json_copy);
	if (!root) {
		fprintf(stderr, "Failed to parse sensor_param JSON: %s\n", camera_config->sensor_param);
		return;
	}

	// tuning_data
	cJSON *tuning_data = cJSON_GetObjectItem(root, "tuning_data");

	if (tuning_data) {
		cJSON *data_width = cJSON_GetObjectItem(tuning_data, "data_width");
		if (data_width) {
			cJSON_SetIntValue(data_width, bit_width);
		}
	}

	char *updated_str = cJSON_PrintUnformatted(root);
	if (updated_str) {
		camera_config->sensor_param = updated_str;
	}

	cJSON_Delete(root);
}

static int create_vin_node(pipe_contex_t *pipe_contex)
{
	camera_config_t *camera_config = NULL;
	vp_sensor_config_t *sensor_config = NULL;
	vin_node_attr_t *vin_node_attr = NULL;
	vin_ichn_attr_t *vin_ichn_attr = NULL;
	vin_ochn_attr_t *vin_ochn_attr = NULL;
	hbn_vnode_handle_t *vin_node_handle = NULL;
	vin_attr_ex_t vin_attr_ex;
	hbn_buf_alloc_attr_t alloc_attr = {0};
	uint32_t hw_id = 0;
	int32_t ret = 0;
	uint32_t ichn_id = 0;
	uint32_t ochn_id = 0;

	sensor_config = pipe_contex->sensor_config;
	camera_config = sensor_config->camera_config;
	vin_node_attr = sensor_config->vin_node_attr;
	vin_ichn_attr = sensor_config->vin_ichn_attr;
	vin_ochn_attr = sensor_config->vin_ochn_attr;
	vin_node_handle = &pipe_contex->vin_node_handle;

	vin_node_attr->magicNumber = MAGIC_NUMBER;
	vin_ochn_attr->magicNumber = MAGIC_NUMBER;
	{
		/* 修改 dummy_camera_config 的 vin_node_attr_t 中 cim_attr.mipi_rx*/
		vin_node_attr->cim_attr.mipi_rx = 0;
		hw_id = vin_node_attr->cim_attr.mipi_rx;

		vin_node_attr->cim_attr.cim_isp_flyby = 0;
		vin_ochn_attr->ddr_en = 1;
	}

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

	alloc_attr.buffers_num = 3;
	alloc_attr.is_contig = 1;
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
						| HB_MEM_USAGE_CPU_WRITE_OFTEN
						| HB_MEM_USAGE_CACHED;
	ret = hbn_vnode_set_ochn_buf_attr(*vin_node_handle, ochn_id, &alloc_attr);

	return 0;
}

static int create_isp_node(pipe_contex_t *pipe_contex) {
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
		isp_attr->channel.hw_id = 0;
		isp_attr->channel.slot_id = 4;
		isp_attr->sched_mode = SCHED_MODE_MANUAL;
	}

	ret = hbn_vnode_open(HB_ISP, 0, AUTO_ALLOC_ID, isp_node_handle);
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

	printf("[INFO] Create isp node handle: %d\n", (int)*isp_node_handle);

	return 0;
}

// 因为isp的tuning参数在camera_config_t中定义，所以需要初始化一个虚拟Sensor来完成整个flow的配置
int create_and_run_isp_vflow(pipe_contex_t *pipe_contex) {
	int32_t ret = 0;
	// 创建pipeline中的每个node
	ret = create_camera_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	ret = create_vin_node(pipe_contex);
	ERR_CON_EQ(ret, 0);
	ret = create_isp_node(pipe_contex);
	ERR_CON_EQ(ret, 0);

	// 创建HBN vflow
	ret = hbn_vflow_create(&pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd,
							pipe_contex->isp_node_handle);
	ERR_CON_EQ(ret, 0);
	// 不需要把 vin 和 isp bind 在一起
	// hbn_camera_attach_to_vin 是必须的, 否则 sendframe 给 isp 时会报43 和 45 号错误
	ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd,
							pipe_contex->vin_node_handle);
	ERR_CON_EQ(ret, 0);
	ret = hbn_vflow_start(pipe_contex->vflow_fd);
	ERR_CON_EQ(ret, 0);

	return 0;
}

void isp_dump_func(hbn_vnode_handle_t isp_node_handle, hbn_vnode_image_t *input_img) {
	int ret;
	char dst_file[128];
	uint32_t chn_id = 0;
	uint32_t ochn_id = 0;
	uint32_t timeout = 10000;
	hbn_vnode_image_group_t out_img_group;
	struct timespec begin = {0}, end = {0};

	(void)clock_gettime(CLOCK_MONOTONIC_RAW, &begin);

	ret = hbn_vnode_sendframe(isp_node_handle, chn_id, input_img);

	// 调用 hbn_vnode_getframe 获取帧数据
	ret = hbn_vnode_getframe_group(isp_node_handle, ochn_id, timeout, &out_img_group);
	if (ret != 0) {
		printf("hbn_vnode_getframe from isp chn:%d failed(%d)\n", ochn_id, ret);
		while(1);
		return;
	}

	(void)clock_gettime(CLOCK_MONOTONIC_RAW, &end);

	printf("isp process one frame cost:  %ld ns\n",
			(end.tv_sec * 1000000000 + end.tv_nsec) - (begin.tv_sec * 1000000000 + begin.tv_nsec));

	hbn_frame_info_t *info = &out_img_group.info;
	hb_mem_graphic_buf_t* out_img = &out_img_group.buf_group.graph_group[ochn_id];

	// 将帧数据写入文件
	snprintf(dst_file, sizeof(dst_file),
		"isp_handle_%d_chn%d_%dx%d_stride_%d_frameid_%d_ts_%ld.yuv",
		(int)isp_node_handle, ochn_id,
		out_img->width, out_img->height, out_img->stride,
		info->frame_id, info->timestamps);
	printf("isp(%d) dump yuv %dx%d(stride:%d), buffer size: %ld + %ld frame id: %d,"
			" timestamp: %ld\n", (int)isp_node_handle,
			out_img->width, out_img->height,
			out_img->stride,
			out_img->size[0], out_img->size[1],
			info->frame_id,
			info->timestamps);
	// 此处，无论是写磁盘还是写内存，都会影响性能
	dump_2plane_yuv_to_file(dst_file,
			out_img->virt_addr[0],
			out_img->virt_addr[1],
			out_img->size[0],
			out_img->size[1]);

	// 释放帧数据
	hbn_vnode_releaseframe_group(isp_node_handle, ochn_id, &out_img_group);
}

int read_raw_image(char* input_file, hbn_vnode_image_t *input_image, camera_config_t *camera_config) {
	int ret = 0;
	uint64_t offset = 0;
	uint32_t format = 0;
	FILE *file = fopen(input_file, "rb");
	if (!file) {
		fprintf(stderr, "Failed to open file: %s\n", input_file);
		return -1;
	}

	switch ((camera_config->format & 0xFF)) {
		case 8:
			format = MEM_PIX_FMT_RAW8;
			break;
		case 10:
			format = MEM_PIX_FMT_RAW10;
			break;
		case 12:
			format = MEM_PIX_FMT_RAW12;
			break;
		default:
			fprintf(stderr, "Unsupported RAW format: %d\n", format);
			return 0;
	}
	alloc_graphic_buffer(input_image, camera_config->width, camera_config->height, 1, format);
	if (input_image->buffer.virt_addr[0] == NULL) {
		fprintf(stderr, "Invalid buffer: virt_addr[0] is NULL\n");
		fclose(file);
		return -1;
	}

	fread(input_image->buffer.virt_addr[0], 1, (size_t)camera_config->width * camera_config->height * 2, file);
	fclose(file);
	hb_mem_flush_buf(input_image->buffer.fd[0], offset,
			input_image->buffer.size[0]);

	return ret;
}

int main(int argc, char** argv) {
	int ret = 0;
	pipe_contex_t vin_isp_contex = {0};
	pipe_contex_t isp_contex = {0};
	char *file_name = NULL;
	char *feedback_raw_format = NULL;
	hbn_vnode_image_t raw_img = {0};
	uint32_t feedback_raw_hight = 0;
	uint32_t feedback_raw_width = 0;
	int32_t raw_type = 0;
	int32_t bit_width = 0;
	int opt_index = 0;
	int c = 0;
	int index = -1;
	int loop = 10;

	while((c = getopt_long(argc, argv, "f:l:F:H:W:h",
							long_options, &opt_index)) != -1) {
		switch (c)
		{
		case 'f':
			file_name = optarg;
			break;
		case 'H':
			feedback_raw_hight = atoi(optarg);
			break;
		case 'W':
			feedback_raw_width = atoi(optarg);
			break;
		case 'F':
			feedback_raw_format = optarg;
			break;
		case 'l':
			if (optarg)
				loop = atoi(optarg);
			else
				loop = 1;
			break;
		case 'h':
		default:
			print_help();
			return 0;
		}
	}

	if (file_name == NULL || feedback_raw_format == NULL) {
		print_help();
		fprintf(stderr, "Error: -f <file> and -F <format> are required.\n");
		return -1;
	}
	if (feedback_raw_width == 0 || feedback_raw_hight == 0) {
		print_help();
		fprintf(stderr, "Error: You must specify both -W <width> and -H <height>\n");
		return -1;
	}

	for(index = 0; index < vp_get_sensors_list_number(); index++){
		if(!strcmp(vp_sensor_config_list[index]->sensor_name,"dummy")){
			vin_isp_contex.sensor_config = vp_sensor_config_list[index];
			printf("Using index:%d  sensor_name:%s  config_file:%s\n",
				index,
				vp_sensor_config_list[index]->sensor_name,
				vp_sensor_config_list[index]->config_file);
		}
	}

	hb_mem_module_open();
	raw_type = (!strcmp(feedback_raw_format, "raw8")) ? 0x2A :
		(!strcmp(feedback_raw_format, "raw10")) ? 0x2B :
		(!strcmp(feedback_raw_format, "raw12")) ? 0x2C : 0x2B;
	bit_width = (!strcmp(feedback_raw_format, "raw8")) ? 8 :
		(!strcmp(feedback_raw_format, "raw10")) ? 10 :
		(!strcmp(feedback_raw_format, "raw12")) ? 12 : 10;

	vin_isp_contex.sensor_config->camera_config->format = bit_width;
	vin_isp_contex.sensor_config->camera_config->height = feedback_raw_hight;
	vin_isp_contex.sensor_config->camera_config->width = feedback_raw_width;
	vin_isp_contex.sensor_config->vin_ichn_attr->format = raw_type;
	vin_isp_contex.sensor_config->vin_ichn_attr->height = feedback_raw_hight;
	vin_isp_contex.sensor_config->vin_ichn_attr->width = feedback_raw_width;
	vin_isp_contex.sensor_config->vin_ochn_attr->vin_basic_attr.format = raw_type;
	vin_isp_contex.sensor_config->vin_ochn_attr->vin_basic_attr.wstride = feedback_raw_width * 2;
	vin_isp_contex.sensor_config->vin_ochn_attr->vin_basic_attr.vstride = feedback_raw_hight;
	vin_isp_contex.sensor_config->isp_attr->size.width = feedback_raw_width;
	vin_isp_contex.sensor_config->isp_attr->size.height = feedback_raw_hight;
	update_sensor_param_data_width(vin_isp_contex.sensor_config->camera_config, bit_width);
	isp_contex.sensor_config = &dummy_sensor_config;
	ret = create_and_run_isp_vflow(&isp_contex);
	ERR_CON_EQ(ret, 0);

	ret = read_raw_image(file_name, &raw_img, dummy_sensor_config.camera_config);
	ERR_CON_EQ(ret, 0);

	for (int i = 0; i < loop; i++)
		isp_dump_func(isp_contex.isp_node_handle, &raw_img);

	ret = hbn_vflow_stop(isp_contex.vflow_fd);
	ERR_CON_EQ(ret, 0);
	hbn_vnode_close(isp_contex.vin_node_handle);
	hbn_vnode_close(isp_contex.isp_node_handle);
	hbn_camera_destroy(isp_contex.cam_fd);
	hbn_vflow_destroy(isp_contex.vflow_fd);
	hb_mem_module_close();

	return 0;
}
