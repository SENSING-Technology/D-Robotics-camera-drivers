/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright(C) 2024, D-Robotics Co., Ltd.
 *                     All rights reserved.
 ***************************************************************************/

#include "hb_mem_mgr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>

#include "common_utils.h"

int32_t dump_image_to_file(char *filename, uint8_t *src_buffer, uint32_t size)
{
	int yuv_fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);

	if (yuv_fd == -1) {
		printf("Error opening file(%s)\n", filename);
		return -1;
	}

	ssize_t bytes_written = write(yuv_fd, src_buffer, size);
	close(yuv_fd);

	if (bytes_written != size) {
		printf("Error writing to file\n");
		return -1;
	}

	printf("Dump image to file(%s), size(%d) succeeded\n", filename, size);
	return 0;
}

int32_t dump_2plane_yuv_to_file(char *filename, uint8_t *src_buffer, uint8_t *src_buffer1,
		uint32_t size, uint32_t size1)
{
	int yuv_fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);

	if (yuv_fd == -1) {
		printf("Error opening file(%s)\n", filename);
		return -1;
	}

	ssize_t bytes_written = write(yuv_fd, src_buffer, size);
	if (bytes_written != size) {
		printf("Error writing y to file, input %d, actual write %ld\n", size, bytes_written);
		close(yuv_fd);
		return -1;
	}

	bytes_written = write(yuv_fd, src_buffer1, size1);
	if (bytes_written != size1) {
		printf("Error writing uv to file\n");
		close(yuv_fd);
		return -1;
	}

	close(yuv_fd);

	return 0;
}

// hbmem
static pthread_mutex_t ion_lock = PTHREAD_MUTEX_INITIALIZER; /*PRQA S 3004*/
static uint32_t g_ion_opened = 0;
static int32_t m_ionClient = -1;

int32_t vpm_hb_mem_init(void)
{
	uint32_t v_major, V_minor, v_patch;

	pthread_mutex_lock(&ion_lock);
	if (g_ion_opened == 0u) {
		m_ionClient = hb_mem_module_open();
		if (m_ionClient < 0) {
			printf("hb_mem_module_open failed ret %d\n", m_ionClient);
			m_ionClient = -1;
			g_ion_opened = 0;
			pthread_mutex_unlock(&ion_lock);
			return -1;
		}
		hb_mem_get_version(&v_major, &V_minor, &v_patch);
		printf("hb_mem_module_open success version: %u.%u.%u\n", v_major, V_minor, v_patch);
		g_ion_opened = 1;
	} else {
		g_ion_opened++;
		printf("skip hb mem open count %d.\n", g_ion_opened);
	}
	pthread_mutex_unlock(&ion_lock);
	return 0;
}

void vpm_hb_mem_deinit(void)
{
	int32_t ret;

	pthread_mutex_lock(&ion_lock);
	if (g_ion_opened > 0u) {
		g_ion_opened--;
		printf("Try release hb mem  open count %d.\n", g_ion_opened);
	}

	if ((g_ion_opened == 0u) && (m_ionClient == 0)) {
		ret = hb_mem_module_close();
		if (ret < 0) {
			printf("hb_mem_module_close failed ret %d\n", ret);
		} else {
			printf("vpm release hb mem done.\n");
		}
		m_ionClient = -1;
		g_ion_opened = 0;
	}
	pthread_mutex_unlock(&ion_lock);
	return;
}

int32_t alloc_graphic_buffer(hbn_vnode_image_t *img, uint32_t width, uint32_t height, uint32_t cached, int32_t format)
{
	int32_t ret;
	int64_t alloc_flags = 0;
	int32_t stride;
	/* int32_t format = MEM_PIX_FMT_NV12; */

	// vpm_hb_mem_init();

	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
			  HB_MEM_USAGE_CPU_WRITE_OFTEN;

	memset(img, 0, sizeof(hbn_vnode_image_t));
	if (cached == 1)
		alloc_flags = alloc_flags | HB_MEM_USAGE_CACHED;

	switch (format) {
		case MEM_PIX_FMT_RAW24:
			stride = width * 3;
			break;
		case MEM_PIX_FMT_RAW16:
			stride = width * 2;
			break;
		case MEM_PIX_FMT_RAW12:
			stride = width * 2;
			break;
		case MEM_PIX_FMT_RAW10:
			stride = width * 2;
			break;
		case MEM_PIX_FMT_RAW8:
			stride = width;
			break;
		default:
			stride = width;
			break;
	}
	ret = hb_mem_alloc_graph_buf(width, height, format, alloc_flags, stride, height, &img->buffer);
	if (ret < 0) {
		printf("hb_mem_alloc_graph_buf ret %d failed \n", ret);
		return ret;
	}

	return ret;
}
int read_nv12_image_to_common_buffer(const char *file_path, hb_mem_common_buf_t *src_buf, int width, int height)
{
    FILE *file = fopen(file_path, "rb");
    if (!file)
    {
        fprintf(stderr, "Failed to open file: %s\n", file_path);
        return -1;
    }

    size_t read_size = fread(src_buf->virt_addr, 1, width * height * 1.5, file);
    fclose(file);

    if (read_size != width * height * 1.5)
    {
        fprintf(stderr, "Failed to read the entire NV12 image from file: %s\n", file_path);
        return -1;
    }

    return 0;
}



int read_nv12_image_to_graphic_buffer(const char *file_path, hb_mem_graphic_buf_t *src_buf, int width, int height)
{
    FILE *file = fopen(file_path, "rb");
    if (!file)
    {
        fprintf(stderr, "Failed to open file: %s\n", file_path);
        return -1;
    }

    size_t read_size = fread(src_buf->virt_addr[0], 1, width * height * 1.5, file);
    fclose(file);

    if (read_size != width * height * 1.5)
    {
        fprintf(stderr, "Failed to read the entire NV12 image from file: %s\n", file_path);
        return -1;
    }

    return 0;
}

int read_nv12_image_to_normal_memory(const char *file_path, uint8_t*virt_addr, int width, int height)
{
    FILE *file = fopen(file_path, "rb");
    if (!file)
    {
        fprintf(stderr, "Failed to open file: %s\n", file_path);
        return -1;
    }

    size_t read_size = fread(virt_addr, 1, width * height * 1.5, file);
    fclose(file);

    if (read_size != width * height * 1.5)
    {
        fprintf(stderr, "Failed to read the entire NV12 image from file: %s\n", file_path);
        return -1;
    }

    return 0;
}

int32_t read_yuvv_nv12_file(const char *filename, char *addr0, char *addr1, uint32_t y_size)
{
	if (filename == NULL || addr0 == NULL || y_size == 0) {
		printf("ERR(%s):null param.\n", __func__);
		return -1;
	}

	FILE *Fd = NULL;
	Fd = fopen(filename, "r");
	char *buffer = NULL;

	if (Fd == NULL) {
		printf("ERR(%s):open(%s) fail\n", __func__, filename);
		return -1;
	}

	buffer = (char *)malloc(y_size + y_size / 2);

	if (fread(buffer, 1, y_size, Fd) != y_size) {
		printf("read bin(%s) to addr fail #1\n", filename);
		return -1;
	}

	if (fread(buffer + y_size, 1, y_size / 2, Fd) != y_size / 2) {
		printf("read bin(%s) to addr fail #2\n", filename);
		return -1;
	}

	memcpy(addr0, buffer, y_size);
	memcpy(addr1, buffer + y_size, y_size / 2);

	fflush(Fd);

	if (Fd)
		fclose(Fd);
	if (buffer)
		free(buffer);

	printf("(%s):file read(%s), y-size(%d)\n", __func__, filename, y_size);
	return 0;
}

char* get_program_name()
{
	FILE* file = fopen("/proc/self/cmdline", "r");
	if (file == NULL) {
		return NULL;
	}

	static char buffer[1024];
	size_t len = fread(buffer, 1, sizeof(buffer) - 1, file);
	fclose(file);

	if (len <= 0) {
		return NULL;
	}

	buffer[len] = '\0';
	char* program_name = strrchr(buffer, '/');
	if (program_name != NULL) {
		program_name++;
	} else {
		program_name = buffer;
	}

	return program_name;
}


int get_isp_channel_config_for_single_pipeline(const int mipi_rx,
	const int is_online, isp_channel_t *isp_channel){
	int ret = 0;
	if(mipi_rx == 0){
		if(is_online){
			isp_channel->hw_id = 0;
			isp_channel->slot_id = 0;
		}else{
			isp_channel->hw_id = 0;
			isp_channel->slot_id = 4; //4 - 11
		}
	}else if(mipi_rx == 1){
		if(is_online){
			isp_channel->hw_id = 0;
			isp_channel->slot_id = 1;
		}else{
			isp_channel->hw_id = 1;
			isp_channel->slot_id = 4; //4 - 11
		}
	}else if(mipi_rx == 4){
		if(is_online){
			ret = -1;
			printf("[ERROR] mipi_rx_%d isp unsupport online mode.\n", mipi_rx);
		}else{
			isp_channel->hw_id = 0; //0, 1
			isp_channel->slot_id = 4; //4 - 11
		}
	}else{
		printf("\n");
		ret = -1;
	}

	printf("\nINFO: ISP channel info: \n");
	printf("	input info: [mipi_rx: %d] [is_online: %d]\n", mipi_rx, is_online);
	printf("	isp channel info: [hw_id: %d] [slot_id: %d]\n", isp_channel->hw_id, isp_channel->slot_id);

	return ret;
}

void get_channel_and_sched_info(const pipe_contex_t *pipe_contex, channel_and_sched_info_t *ch_sched_info){
	isp_attr_t isp_attr;
	hbn_vnode_get_attr(pipe_contex->isp_node_handle, &isp_attr);

	ch_sched_info->isp_channel = isp_attr.channel;
	ch_sched_info->isp_sched_mode = isp_attr.sched_mode;
	ch_sched_info->mipi_rx = pipe_contex->csi_config.mipi_rx;
}
int get_pym_channel_config_for_single_pipeline(const channel_and_sched_info_t *ch_sched_info,
	const int is_pym_online, pym_channel_t *pym_channel){
	int ret = 0;
	if(ch_sched_info->mipi_rx == 0){
		if(is_pym_online){
			pym_channel->hw_id = ch_sched_info->isp_channel.hw_id; //必须和isp的一致
			if(ch_sched_info->isp_sched_mode == SCHED_MODE_MANUAL){//ISP输入可能是 offline 或 ISP输入时Online内部下DDR（忽略这种模式）
				pym_channel->sched_mode = 1; //Manaul 模式
				pym_channel->slot_id = ch_sched_info->isp_channel.slot_id;//slot_id 必须跟随 isp
			}else{
				pym_channel->sched_mode = 2; //全online模式
				pym_channel->slot_id = ch_sched_info->isp_channel.slot_id; //ISP 和 PYM 都是0
			}
		}else{
			pym_channel->hw_id = 0; // 离线模式pym 硬件ID可以选择 0 1 4，本函数是单路模式，默认与mipi_rx相同
			pym_channel->sched_mode = 3;  //离线模式
			pym_channel->slot_id = 0; //没有作用
		}
	}else if(ch_sched_info->mipi_rx == 1){
		if(is_pym_online){
			pym_channel->hw_id = ch_sched_info->isp_channel.hw_id; //必须和isp的一致
			if(ch_sched_info->isp_sched_mode == SCHED_MODE_MANUAL){//ISP输入可能是 offline 或 ISP输入时Online内部下DDR（忽略这种模式）
				pym_channel->sched_mode = 1; //Manaul 模式
				pym_channel->slot_id = ch_sched_info->isp_channel.slot_id;//slot_id 必须跟随 isp
			}else{
				pym_channel->sched_mode = 2; //全online模式
				pym_channel->slot_id = ch_sched_info->isp_channel.slot_id; //ISP 和 PYM 都是0
			}
		}else{
			pym_channel->hw_id = 1; // 离线模式pym 硬件ID可以选择 0 1 4，本函数是单路模式，默认与mipi_rx相同
			pym_channel->sched_mode = 3;  //离线模式
			pym_channel->slot_id = 0; //没有作用
		}
	}else if(ch_sched_info->mipi_rx == 4){
		if(is_pym_online){
			ret = -1;
			printf("[ERROR] mipi_rx_%d pym unsupport online mode.\n", ch_sched_info->mipi_rx);
		}else{
			pym_channel->hw_id = 4;    // 离线模式pym 硬件ID可以选择 0 1 4，本函数是单路模式，默认与mipi_rx相同
			pym_channel->sched_mode = 3;    //离线模式
			pym_channel->slot_id = 0; //没有作用
		}
	}else{
		printf("\n");
		ret = -1;
	}

	printf("\nINFO: PYM channel info: \n");
	printf("	input info: [mipi_rx: %d] [is_online: %d]\n", ch_sched_info->mipi_rx, is_pym_online);
	printf("	pym channel info: [hw_id: %d] [slot_id: %d] [mode: %d]\n",
		pym_channel->hw_id, pym_channel->slot_id, pym_channel->sched_mode);

	return ret;
}
int get_ynr_channel_config_for_single_pipeline(const channel_and_sched_info_t *ch_sched_info, ynr_channel_t *ynr_channel){
	int ret = 0;
	if(ch_sched_info->isp_channel.hw_id != 1){ //只有 ISP1 可以 连接 到 ynr模块，并且只能是 online
		printf("only isp1 can online ynr, current isp hw id is %d\n", ch_sched_info->isp_channel.hw_id);
		return -1;
	}
	ynr_channel->hw_id = 1; // 只有一个硬件并且id是1
	if(ch_sched_info->isp_sched_mode == SCHED_MODE_MANUAL){
		ynr_channel->sched_mode = 1;//Manaul 模式
		ynr_channel->slot_id = ch_sched_info->isp_channel.slot_id;
	}else{
		ynr_channel->sched_mode = 2; //全online模式
		ynr_channel->slot_id = 0;
	}
	printf("\nINFO: YNR channel info: \n");
	printf("	ynr channel info: [hw_id: %d] [slot_id: %d] [mode: %d]\n",
		ynr_channel->hw_id, ynr_channel->slot_id, ynr_channel->sched_mode);
	return ret;
}