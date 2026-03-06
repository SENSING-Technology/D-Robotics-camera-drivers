#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include "communicate/sdk_common_cmd.h"
#include "communicate/sdk_common_struct.h"
#include "communicate/sdk_communicate.h"

#include "utils/nalu_utils.h"
#include "utils/utils_log.h"
#include "utils/cqueue.h"
#include "utils/common_utils.h"
#include "utils/stream_define.h"
#include "utils/mthread.h"
#include "utils/mqueue.h"
#include "utils/time_utils.h"

#include "model_info.h"
#include "bpu_wrap.h"
#include "vp_wrap.h"
#include "vp_codec.h"
#include "vp_sensors.h"


#include "vpp_preparam.h"
#include "vpp_camera_impl.h"

#define VPP_ISP_OUTBUFFER_COUNT 3
#define VPP_ISP_OUTBUFFER_RELEASE_COUNT 2
#define VPP_CAM_MAX_CHANNELS 32

typedef struct
{
	int pipline_id;
	int drm_init_succesed;
	// vp_drm_context_t *drm_context;
	vp_vflow_contex_t vp_vflow_contex;

	media_codec_user_config_t m_encode_user_config;
	media_codec_context_t m_encode_context;

	bpu_handle_t	m_bpu_handle;

	tsThread 		m_pipe_thread; /* 从pipe获取图像，送入编码 */
	tsThread 		m_venc_thread; /*从编码器获取图像，送入共享内存 */
	tsQueue			m_pipe_to_enc_queue;
	tsQueue			m_enc_to_pipe_queue;

	int pipe_buffer_used_count;

	void *media;
	uint64_t first_frame_timestamp;
	const char *media_type;
	int vpp_impl_index;
} vpp_camera_t;

static vpp_camera_t g_vpp_camera[VPP_CAM_MAX_CHANNELS];

static int32_t send_video_frame_info(int pipeline_id, int frame_id, int64_t timestamp)
{
	int32_t ret = 0;
	char *ws_msg = NULL;

	ws_msg = malloc(200);
	if (NULL == ws_msg) {
		SC_LOGE("Failed to allocate memory for ws_msg");
		return -1;
	}
	sprintf(ws_msg, "{\"kind\":11, \"pipeline\":%d, \"frame_id\":%d, \"timestamp\":%ld}", pipeline_id + 1, frame_id, timestamp);
	ret = SDK_Cmd_Impl(SDK_CMD_WEBSOCKET_SEND_MSG, (void*)ws_msg);
	free(ws_msg);
	return ret;
}

static void vpp_camera_push_stream(vpp_camera_t *vpp_camera, ImageFrame *stream)
{
	if(stream == NULL) {
		SC_LOGE("Param is NULL");
		return;
	}

	media_codec_buffer_t *buffer = (media_codec_buffer_t *)(stream->frame_buffer);
	if(vpp_camera->first_frame_timestamp == 0){
		vpp_camera->first_frame_timestamp = buffer->vstream_buf.pts / 1000;
		SC_LOGI("channel %d recved first frame, and pts is %lld.", vpp_camera->pipline_id, buffer->vstream_buf.pts / 1000);
	}
	send_video_frame_info(vpp_camera->vpp_impl_index, buffer->vstream_buf.src_idx, buffer->vstream_buf.pts);

	T_SDK_MEDIA_SRV_PUSH_PARAM push_param = {
		.media = vpp_camera->media,
		.data = (const char*)buffer->vstream_buf.vir_ptr,
		.data_length = buffer->vstream_buf.size,
		.pts = buffer->vstream_buf.pts /1000,
		.dts = buffer->vstream_buf.pts /1000,
		.codec_name = vpp_camera->media_type
	};

	SDK_Cmd_Impl(SDK_CMD_MEDIA_SERVER_PUSH_DATA, &push_param);
	#if 0
	media_server_push_video(vpp_camera->media, (const char*)buffer->vstream_buf.vir_ptr,
		buffer->vstream_buf.size, buffer->vstream_buf.pts /1000, buffer->vstream_buf.pts / 1000, vpp_camera->media_type);
	#endif
}

static void* venc_get_stream_proc(void *ptr)
{
	int32_t ret = 0;
	tsThread *privThread = (tsThread*)ptr;
	vpp_camera_t *vpp_camera = (vpp_camera_t *)privThread->pvThreadData;

	ImageFrame encode_stream = {0};
	if (vp_allocate_image_frame(&encode_stream) == NULL) {
		SC_LOGE("vp_allocate_image_frame for encode_stream failed, so exit program.");
		exit(-1);
	}
	teQueueStatus status = E_QUEUE_OK;
	ImageFrame *vflow_image = NULL;
	int dequeue_enc_count = 0;
	int enqueue_pipe_count = 0;
	//线程退出时，保证hbn_vnode_image 已经处理完： 放到 m_enc_to_pipe_queue
	while (privThread->eState == E_THREAD_RUNNING){
		status = mQueueDequeueTimed(&vpp_camera->m_pipe_to_enc_queue, 2000, (void **)&vflow_image);
		if(status != E_QUEUE_OK){
			SC_LOGI("channel %d dequeue from enc_to_pipe_queue failed:%d\n", vpp_camera->pipline_id ,status);
			continue;
		}
		dequeue_enc_count++;

		// 送进编码器
		ret = vp_codec_encoder_set_input(&vpp_camera->m_encode_context, vflow_image);
		if(ret != 0){
			if (privThread->eState == E_THREAD_RUNNING) {
				SC_LOGE("vp_codec_set_input failed.");
			}
			break;
		}

		// 从编码器获取码流
		ret = vp_codec_get_output(&vpp_camera->m_encode_context, &encode_stream, 2000);
		if(ret != 0){
			if (privThread->eState == E_THREAD_RUNNING) {
				SC_LOGE("vp_codec_get_output failed.");
			}
			break;
		}
		// 编码器用完Pipe的数据 就释放
		ret = vp_flow_release_frame(&vpp_camera->vp_vflow_contex, vflow_image);
		if (ret != 0) {
			SC_LOGE("vp_flow_release_frame failed.");
			break;
		}
		vpp_camera->pipe_buffer_used_count--;

		// 编码器用完VnodeBuffer,就归还给 Pipe
		while(privThread->eState == E_THREAD_RUNNING){
			status = mQueueEnqueueEx(&vpp_camera->m_enc_to_pipe_queue, vflow_image);
			if (status != E_QUEUE_OK){
				SC_LOGE("channel %d enqueue from enc_to_pipe_queue failed:%d\n", vpp_camera->pipline_id ,status);
				sleep(1);
				continue;
			}
			vflow_image = NULL;
			enqueue_pipe_count++;
			break;
		}


		vpp_camera_push_stream(vpp_camera, &encode_stream);
		ret = vp_codec_release_output(&vpp_camera->m_encode_context, &encode_stream);
		if (ret != 0) {
			SC_LOGE("vp_codec_release_output failed.");
			break;
		}

	}

	int free_dissociate_count = 0;
	if(vflow_image != NULL){
		vp_free_image_frame(vflow_image);
		free(vflow_image);
		free_dissociate_count = 1;
	}
	SC_LOGI("channel %d dequeue enc %d = enqueue isp %d + free_dissociate_count %d.\n",
		vpp_camera->pipline_id , dequeue_enc_count, enqueue_pipe_count, free_dissociate_count);


	vp_free_image_frame(&encode_stream);
	mThreadFinish(privThread);
	return NULL;
}

/******************************************************************************
 * funciton : get stream from each channels
 ******************************************************************************/
static void* vflow_get_stream_proc(void *ptr)
{
	int32_t ret = 0;

	tsThread *privThread = (tsThread*)ptr;
	vpp_camera_t *vpp_camera = (vpp_camera_t *)privThread->pvThreadData;
	mThreadSetNameWidthIndex(privThread, __func__, vpp_camera->pipline_id);

	teQueueStatus status = E_QUEUE_OK;
	ImageFrame	*vflow_frame = NULL;
	struct TimeStatistics time_statistics;

	int dequeue_pipe_count = 0;
	int enqueue_enc_count = 0;

	while (privThread->eState == E_THREAD_RUNNING){
		time_statistics_at_beginning_of_loop(&time_statistics);
		status = mQueueDequeueTimed(&vpp_camera->m_enc_to_pipe_queue, 2000, (void **)&vflow_frame);
		if (status != E_QUEUE_OK){
			SC_LOGI("channel %d dequeue from enc_to_pipe_queue failed:%d\n", vpp_camera->pipline_id, status);
			continue;
		}
		dequeue_pipe_count++;

		int wait_count = 0;
		while(privThread->eState == E_THREAD_RUNNING){
			ret = vp_flow_get_frame(&vpp_camera->vp_vflow_contex, vflow_frame);
			if (ret != 0) {
				wait_count++;
				// 当线程接收到退出信号时，getframe 接口会立即报超时退出
				// 所以只有当线程是正常运行状态下的异常才属于真异常
				if (privThread->eState == E_THREAD_RUNNING) {
					if(vpp_camera->pipe_buffer_used_count > VPP_ISP_OUTBUFFER_COUNT - VPP_ISP_OUTBUFFER_RELEASE_COUNT){
						SC_LOGI("vp_flow_get_frame chn not geted data(%d), because buffer is not enough, pipe used count %d, pipe all count %d, wait %d",
							ret, vpp_camera->pipe_buffer_used_count, VPP_ISP_OUTBUFFER_COUNT, wait_count);
					}else{
						SC_LOGE("vp_flow_get_frame chn 0 failed(%d), pipe used count %d, pipe all count %d.",
							ret, vpp_camera->pipe_buffer_used_count, VPP_ISP_OUTBUFFER_COUNT);
						// vp_print_debug_infos_when_error();
						sleep(1);
					}
					continue;
				}
			}else{
				break;
			}
		}

		vpp_camera->pipe_buffer_used_count++;
		// send to bpu
		if (strlen(vpp_camera->m_bpu_handle.m_model_name) != 0){
			bpu_buffer_info_t bpu_input_buffer;
			hbn_vnode_image_t *hbn_vnode_image = NULL;
			hbn_vnode_image = (hbn_vnode_image_t *)vflow_frame->hbn_vnode_image;
			memset(&bpu_input_buffer, 0, sizeof(bpu_buffer_info_t));
			vpp_graphic_buf_to_bpu_buffer_info(hbn_vnode_image, &bpu_input_buffer);
			bpu_wrap_send_frame(&vpp_camera->m_bpu_handle, &bpu_input_buffer);
		}

		while(privThread->eState == E_THREAD_RUNNING){
			status = mQueueEnqueueEx(&vpp_camera->m_pipe_to_enc_queue, vflow_frame);
			if (status != E_QUEUE_OK){
				printf("channel %d enqueue from enc_to_pipe_queue failed:%d\n", vpp_camera->pipline_id ,status);
				sleep(1);
				continue;
			}
			vflow_frame = NULL;
			enqueue_enc_count++;
			break;
		}


		time_statistics_at_ending_of_loop(&time_statistics);
		time_statistics_info_show(&time_statistics, "read_camera", false);
	}
	int free_dissociate_count = 0;
	if(vflow_frame != NULL){
		vp_free_image_frame(vflow_frame);
		free(vflow_frame);
		free_dissociate_count = 1;
	}

	SC_LOGI("channel %d dequeue isp %d = enqueue enc %d + free_dissociate_count %d.\n",
		vpp_camera->pipline_id , dequeue_pipe_count, enqueue_enc_count, free_dissociate_count);
	mThreadFinish(privThread);
	return NULL;
}

int32_t vpp_camera_init_param_full(solution_cfg_t* solution_cfg){
	int32_t i = 0, ret = 0;
	camera_config_t *camera_config = NULL;
	// isp_ichn_attr_t *isp_ichn_attr = NULL;
	int32_t input_width = 0, input_height = 0;

	memset(&g_vpp_camera, 0, sizeof(g_vpp_camera));

	for (i = 0; i < VPP_CAM_MAX_CHANNELS; i++) {
		g_vpp_camera[i].m_encode_context.codec_id = MEDIA_CODEC_ID_NONE;
	}
	int vpp_camera_index = 0;
	int pipeline_count =solution_cfg->cam_solution.pipeline_count;
	vp_flow_info_reset();

	// 根据camera solution的配置设置vin、venc模块的使能和参数
	for (i = 0; i <solution_cfg->cam_solution.max_pipeline_count; i++) {
		// 1. 配置 vin
		if(solution_cfg->cam_solution.cam_vpp[i].is_valid == 0){
			continue;
		}
		char* sensor_name =solution_cfg->cam_solution.cam_vpp[i].sensor;
		if(solution_cfg->cam_solution.cam_vpp[i].is_enable == 0){
			SC_LOGI("Ignore camera sensor [%s] [%d/%d].", sensor_name, i, pipeline_count);
			continue;
		}
		g_vpp_camera[i].vpp_impl_index = vpp_camera_index;
		g_vpp_camera[i].vp_vflow_contex.sensor_addr = solution_cfg->cam_solution.cam_vpp[i].sensor_addr;
		g_vpp_camera[i].vp_vflow_contex.mipi_csi_rx_index = solution_cfg->cam_solution.cam_vpp[i].csi_index;
		g_vpp_camera[i].vp_vflow_contex.sensor_config = vp_get_sensor_config_by_name(sensor_name);
		g_vpp_camera[i].vp_vflow_contex.mclk_is_not_configed =solution_cfg->cam_solution.cam_vpp[i].mclk_is_not_configed;

		SC_LOGI("Enable camera sensor [%s] [%d/%d] mclk_is_not_configed:[%d]", sensor_name, i, pipeline_count,
			g_vpp_camera[i].vp_vflow_contex.mclk_is_not_configed);
		if (g_vpp_camera[i].vp_vflow_contex.sensor_config == NULL) {
			SC_LOGE("sensor name not found(%s)", sensor_name);
			return -1;
		}
		g_vpp_camera[i].vp_vflow_contex.vin_info.ochn_buffer_count = 3;
		g_vpp_camera[i].vp_vflow_contex.isp_info.ochn_buffer_count = 3;
		g_vpp_camera[i].vp_vflow_contex.gdc_info.output_buffer_count = 3;

		//for vflow
		vp_flow_info_init(g_vpp_camera[i].vp_vflow_contex.sensor_config, &g_vpp_camera[i].vp_vflow_contex.vp_flow_info);
		vp_flow_info_show(&g_vpp_camera[i].vp_vflow_contex.vp_flow_info, vpp_camera_index);

		//codec
		camera_config = g_vpp_camera[i].vp_vflow_contex.sensor_config->camera_config;
		input_width = camera_config->width;
		input_height = camera_config->height;
		SC_LOGI("Pipe %d: input_width: %d input_height: %d", i, input_width, input_height);
		media_codec_user_config_t *codec_user_config = &g_vpp_camera[i].m_encode_user_config;
		codec_user_config->bit_rate = solution_cfg->cam_solution.cam_vpp[i].encode_bitrate;
		codec_user_config->codec_type = VP_GET_MD_CODEC_TYPE(solution_cfg->cam_solution.cam_vpp[i].encode_type);
		codec_user_config->frame_rate = camera_config->fps;
		codec_user_config->width = input_width;
		codec_user_config->height = input_height;

		codec_user_config->input_buffer_is_extrenal = false;
		codec_user_config->input_buffer_count = 0;
		codec_user_config->output_buffer_count = 5;

		ret = vp_encode_config_param(&g_vpp_camera[i].m_encode_context, codec_user_config);
		if (ret != 0)
		{
			SC_LOGE("Encode config param error");
		}

		//gdc
		g_vpp_camera[i].vp_vflow_contex.gdc_info.input_width = input_width;
		g_vpp_camera[i].vp_vflow_contex.gdc_info.input_height = input_height;
		strcpy(g_vpp_camera[i].vp_vflow_contex.gdc_info.sensor_name, sensor_name);
		g_vpp_camera[i].vp_vflow_contex.gdc_info.status = solution_cfg->cam_solution.cam_vpp[i].gdc_status;

		//bpu
		if (strlen(solution_cfg->cam_solution.cam_vpp[i].model) > 1
			&& strcmp(solution_cfg->cam_solution.cam_vpp[i].model, "null") != 0) {
			//g_vpp_camera 与插入的摄像头的顺序一一对应
			//m_vpp_id 与使能的摄像头一一对应
			//比如插入了两个摄像头,只使能第二个: g_vpp_camera[0] 是空 g_vpp_camera[1]是有效的
			// 					  			  m_vpp_id 为0 (决定了算法上报结果的通道号)
			g_vpp_camera[i].m_bpu_handle.m_vpp_id = vpp_camera_index;
			strncpy(g_vpp_camera[i].m_bpu_handle.m_model_name,
				solution_cfg->cam_solution.cam_vpp[i].model,
				sizeof(g_vpp_camera[i].m_bpu_handle.m_model_name) - 1);
			g_vpp_camera[i].m_bpu_handle.m_model_name[sizeof(g_vpp_camera[i].m_bpu_handle.m_model_name) - 1] = '\0';
		}

		vpp_camera_index++;
	}

	return ret;
}
int32_t vpp_camera_init_param(void){
	return vpp_camera_init_param_full(&g_solution_config);
}

int32_t vpp_init_ion_pipeline_fixed_param_from_vflow_contex(
		vpp_camera_t *vpp, solution_cfg_cam_t* cam_cfg, vp_ion_pipeline_fixed_param_t *fixed_param){

	memset(fixed_param, 0, sizeof(vp_ion_pipeline_fixed_param_t));

	return 0;
}

int32_t vpp_camera_ion_param_get(solution_cfg_t* solution_cfg, solution_ion_param_info_t *solution_param_info){
	return 0;
}

int32_t vpp_camera_init(void)
{
	int32_t ret = 0;
	int32_t i = 0;
	vp_vflow_contex_t *vp_vflow_contex = NULL;


	hb_mem_module_open();

	for (i = 0; i < VPP_CAM_MAX_CHANNELS; i++) {
		if (g_vpp_camera[i].vp_vflow_contex.sensor_config == NULL)
			continue;

		vp_vflow_contex = &g_vpp_camera[i].vp_vflow_contex;
		vp_flow_info_t *vp_flow_info = &vp_vflow_contex->vp_flow_info;
		ret = vp_vin_init(vp_vflow_contex);
		if(vp_flow_info->type == VP_FLOW_ISP_ONLY){
			ret |= vp_isp_init(vp_vflow_contex);
		}else if(vp_flow_info->type == VP_FLOW_ISP_YNR){
			ret |= vp_isp_init(vp_vflow_contex);
			ret |= vp_ynr_init(vp_vflow_contex);
		}else{
			//no isp
		}
		ret |= vp_pym_init(vp_vflow_contex);
		ret |= vp_gdc_init(vp_vflow_contex);
		ret |= vp_vflow_init(vp_vflow_contex);
		if (ret != 0){
			SC_LOGE("pipeline init failed for channel %d error", i);
			continue;
		}

		g_vpp_camera[i].drm_init_succesed = 0;

		ret = vp_codec_init(&g_vpp_camera[i].m_encode_context);
		if (ret != 0){
			SC_LOGE("Encode vp_codec_init error for channel %d", i);
			continue;
		}
		SC_LOGI("Init video encode instance %d successful", g_vpp_camera[i].m_encode_context.instance_index);

		if (strlen(g_vpp_camera[i].m_bpu_handle.m_model_name) == 0)
			continue;
		ret = bpu_wrap_model_init(&g_vpp_camera[i].m_bpu_handle, g_vpp_camera[i].m_bpu_handle.m_model_name);
		if (ret != 0) {
			SC_LOGE("bpu_wrap_model_init failed");
			continue;
		}
		// 注册算法结果回调函数
		bpu_wrap_callback_register(&g_vpp_camera[i].m_bpu_handle,
			bpu_wrap_general_result_handle, &g_vpp_camera[i].m_bpu_handle.m_vpp_id);
	}

	return 0;
}

int32_t vpp_camera_uninit(void)
{
	int32_t ret = 0, i = 0;
	vp_vflow_contex_t *vp_vflow_contex = NULL;

	for (i = 0; i < VPP_CAM_MAX_CHANNELS; i++) {
		if (g_vpp_camera[i].vp_vflow_contex.sensor_config == NULL)
			continue;

		vp_vflow_contex = &g_vpp_camera[i].vp_vflow_contex;
		vp_flow_info_t *vp_flow_info = &vp_vflow_contex->vp_flow_info;

		ret = vp_codec_deinit(&g_vpp_camera[i].m_encode_context);
		ret |= vp_vflow_deinit(vp_vflow_contex);
		ret |= vp_gdc_deinit(vp_vflow_contex);
		if(vp_flow_info->type == VP_FLOW_ISP_ONLY){
			ret |= vp_isp_deinit(vp_vflow_contex);
		}else if(vp_flow_info->type == VP_FLOW_ISP_YNR){
			ret |= vp_isp_deinit(vp_vflow_contex);
			ret |= vp_ynr_deinit(vp_vflow_contex);
		}else{
			//no isp
		}
		ret |= vp_pym_deinit(vp_vflow_contex);
		ret |= vp_vin_deinit(vp_vflow_contex);
		SC_ERR_CON_EQ(ret, 0, "vpp_camera_uninit");

		if (strlen(g_vpp_camera[i].m_bpu_handle.m_model_name) == 0)
			continue;
		ret = bpu_wrap_deinit(&g_vpp_camera[i].m_bpu_handle);
		if (ret != 0) {
			SC_LOGE("bpu_wrap_model_init failed");
			return -1;
		}
	}

	hb_mem_module_close();

	vp_print_debug_infos();
	return ret;
}

int32_t vpp_camera_start(void)
{
	int32_t ret = 0;
	int32_t i = 0;
	vp_vflow_contex_t *vp_vflow_contex = NULL;

	for (i = 0; i < VPP_CAM_MAX_CHANNELS; i++) {
		if (g_vpp_camera[i].vp_vflow_contex.sensor_config == NULL)
			continue;
		g_vpp_camera[i].pipline_id = i;
		vp_vflow_contex = &g_vpp_camera[i].vp_vflow_contex;

		char meida_name[64];
		sprintf(meida_name, "ch%d", g_vpp_camera[i].vpp_impl_index);
		g_vpp_camera[i].media_type = vp_codec_get_codec_type_string(g_vpp_camera[i].m_encode_context.codec_id);

		T_SDK_MEDIA_SRV_CREATE_PARAM create_param = {
			.media_name = meida_name,
			.stream_name = "main",
			.codec_type_name = g_vpp_camera[i].media_type,
			.media = NULL,
		};
		SDK_Cmd_Impl(SDK_CMD_MEDIA_SERVER_CREATE, &create_param);
		 	g_vpp_camera[i].media = create_param.media;

		// g_vpp_camera[i].media = media_server_create_media(meida_name, "main", g_vpp_camera[i].media_type);
		//队列的个数根据 isp 输出buffer的个数设置
		teQueueStatus status = mQueueCreate(&g_vpp_camera[i].m_pipe_to_enc_queue, VPP_ISP_OUTBUFFER_COUNT + 1); //必须是加1：mqueue 为了判断空和满的区别，保留了一个item
		if(status != E_QUEUE_OK){
			SC_LOGE("mqueue create failed %d, for channle:%d.", status, i);
			continue;
		}

		status = mQueueCreate(&g_vpp_camera[i].m_enc_to_pipe_queue, VPP_ISP_OUTBUFFER_COUNT + 1); //必须是加1：mqueue 为了判断空和满的区别，保留了一个item
		if(status != E_QUEUE_OK){
			SC_LOGE("mqueue create failed %d, for channle:%d.", status, i);
			continue;
		}

		for (size_t j = 0; j < VPP_ISP_OUTBUFFER_COUNT; j++){
			ImageFrame *image_frame = (ImageFrame *)malloc(sizeof(ImageFrame));
			if (image_frame == NULL){
				SC_LOGE("malloc failed\n");
				exit(-1);
			}
			memset(image_frame, 0, sizeof(ImageFrame));
			vp_allocate_image_frame(image_frame);

			teQueueStatus status = mQueueEnqueue(&g_vpp_camera[i].m_enc_to_pipe_queue, (void *)image_frame);
			if (status != E_QUEUE_OK){
				printf("mqueue enqueue failed:%d\n", status);
				return -1;
			}
		}

		ret = vp_codec_start(&g_vpp_camera[i].m_encode_context);
		if (ret != 0)
		{
			SC_LOGE("Encode vp_codec_start error");
			return -1;
		}
		SC_LOGI("Start video encode instance %d successful", g_vpp_camera[i].m_encode_context.instance_index);
		vp_flow_info_t *vp_flow_info = &vp_vflow_contex->vp_flow_info;
		ret = vp_vin_start(vp_vflow_contex);
		if(vp_flow_info->type == VP_FLOW_ISP_ONLY){
			ret |= vp_isp_start(vp_vflow_contex);
		}else if(vp_flow_info->type == VP_FLOW_ISP_YNR){
			ret |= vp_isp_start(vp_vflow_contex);
			ret |= vp_ynr_start(vp_vflow_contex);
		}else{
			//no isp
		}
		ret |= vp_pym_start(vp_vflow_contex);
		ret |= vp_gdc_start(vp_vflow_contex);
		ret |= vp_vflow_start(vp_vflow_contex);
		SC_ERR_CON_EQ(ret, 0, "vpp_camera_start");

		g_vpp_camera[i].m_pipe_thread.pvThreadData = (void*)&g_vpp_camera[i];
		mThreadStart(vflow_get_stream_proc, &g_vpp_camera[i].m_pipe_thread, E_THREAD_JOINABLE);

		g_vpp_camera[i].m_venc_thread.pvThreadData = (void*)&g_vpp_camera[i];
		mThreadStart(venc_get_stream_proc, &g_vpp_camera[i].m_venc_thread, E_THREAD_JOINABLE);

		if (strlen(g_vpp_camera[i].m_bpu_handle.m_model_name) == 0)
			continue;
		// 设置bpu后处理的原始图像大小为推流图像大小
		bpu_wrap_set_ori_hw(&g_vpp_camera[i].m_bpu_handle,
			g_vpp_camera[i].m_encode_context.video_enc_params.width,
			g_vpp_camera[i].m_encode_context.video_enc_params.height);

		ret = bpu_wrap_start(&g_vpp_camera[i].m_bpu_handle);
		if (ret != 0) {
			SC_LOGE("bpu_wrap_start failed");
			return -1;
		}
	}
	vp_print_debug_infos();
	return ret;
}

int32_t vpp_camera_stop(void)
{
	int32_t ret = 0, i = 0;
	vp_vflow_contex_t *vp_vflow_contex = NULL;

	// 先把所有线程停掉
	for (i = 0; i < VPP_CAM_MAX_CHANNELS; i++) {
		if (g_vpp_camera[i].vp_vflow_contex.sensor_config == NULL)
			continue;

		vp_vflow_contex = &g_vpp_camera[i].vp_vflow_contex;
		mThreadStop(&g_vpp_camera[i].m_venc_thread);
		mThreadStop(&g_vpp_camera[i].m_pipe_thread);

		int enc_remain_count = 0;
		int pipe_remain_count = 0;
		teQueueStatus status = E_QUEUE_OK;
		while(!mQueueIsEmpty(&g_vpp_camera[i].m_pipe_to_enc_queue)){

			ImageFrame *image_frame = NULL;
			status = mQueueDequeueTimed(&g_vpp_camera[i].m_pipe_to_enc_queue, 0, (void **)&image_frame);
			if(status != E_QUEUE_OK){
				SC_LOGE("mqueue clear failed %d, for channle:%d.", status, i);
				break;
			}
			vp_free_image_frame(image_frame);
			free(image_frame);
			enc_remain_count++;
		}
		status = mQueueDestroy(&g_vpp_camera[i].m_pipe_to_enc_queue);
		if(status != E_QUEUE_OK){
			SC_LOGE("mqueue destroy failed %d, for channle:%d.", status, i);
		}

		while(!mQueueIsEmpty(&g_vpp_camera[i].m_enc_to_pipe_queue)){
			ImageFrame *image_frame = NULL;
			status = mQueueDequeueTimed(&g_vpp_camera[i].m_enc_to_pipe_queue, 0, (void **)&image_frame);
			if(status != E_QUEUE_OK){
				SC_LOGE("mqueue clear failed %d, for channle:%d.", status, i);
				break;
			}
			vp_free_image_frame(image_frame);
			free(image_frame);
			pipe_remain_count++;
		}
		SDK_Cmd_Impl(SDK_CMD_MEDIA_SERVER_DESTROY, g_vpp_camera[i].media);
		// media_server_destroy_media(g_vpp_camera[i].media);
		g_vpp_camera[i].media = NULL;
		g_vpp_camera[i].media_type = NULL;
		SC_LOGI("channel %d enc queue remain %d, isp queue remain %d .\n",
				&g_vpp_camera[i].pipline_id , enc_remain_count, pipe_remain_count);
	}

	for (i = 0; i < VPP_CAM_MAX_CHANNELS; i++) {
		if (g_vpp_camera[i].vp_vflow_contex.sensor_config == NULL)
			continue;

		vp_vflow_contex = &g_vpp_camera[i].vp_vflow_contex;
		vp_flow_info_t *vp_flow_info = &vp_vflow_contex->vp_flow_info;

		ret = vp_codec_stop(&g_vpp_camera[i].m_encode_context);
		ret |= vp_vflow_stop(vp_vflow_contex);
		ret |= vp_vin_stop(vp_vflow_contex);
		if(vp_flow_info->type == VP_FLOW_ISP_ONLY){
			ret |= vp_isp_stop(vp_vflow_contex);
		}else if(vp_flow_info->type == VP_FLOW_ISP_YNR){
			ret |= vp_isp_stop(vp_vflow_contex);
			ret |= vp_ynr_stop(vp_vflow_contex);
		}else{
			//no isp
		}
		ret |= vp_pym_stop(vp_vflow_contex);
		ret |= vp_gdc_stop(vp_vflow_contex);
		SC_ERR_CON_EQ(ret, 0, "vpp_camera_stop");
		if (strlen(g_vpp_camera[i].m_bpu_handle.m_model_name) == 0)
			continue;
		ret = bpu_wrap_stop(&g_vpp_camera[i].m_bpu_handle);
		if (ret != 0) {
			SC_LOGE("bpu_wrap_start failed");
			return -1;
		}
	}

	return ret;
}

static int32_t get_pipeline_id_by_video_id(int32_t video_id)
{
	int32_t i = 0;
	int32_t enable_pipeline_count = 0;
	// 遍历所有 pipeline
	// 用 enable_pipeline_count 记录使能的pipeline的编号，这个编号理论上与 web 上的video编号相等
	// 当 enable_pipeline_count == video_id时就说明找到了对应的pipeline
	for (i = 0; i < VPP_CAM_MAX_CHANNELS; i++) {
		if (g_vpp_camera[i].m_encode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			enable_pipeline_count++;
			if (enable_pipeline_count == video_id) {
				return i;
			}
		}
	}
	return 0;
}

int32_t vpp_camera_param_set(SOLUTION_PARAM_E type, char* val, uint32_t length)
{
	int32_t ret = 0;
	return ret;
}

int32_t vpp_camera_param_get(SOLUTION_PARAM_E type, char* val, uint32_t* length)
{
	int32_t i = 0, ret = 0;
	mc_video_codec_enc_params_t *enc_params;
	ImageFrame image_frame = {0};
	char file_name[256] = {0};
	switch(type)
	{
	case SOLUTION_VENC_CHN_PARAM_GET: // 获取某个编码通道的配置
		{
			venc_info_t* param = (venc_info_t*)val;
			param->enable = 0;
			SC_LOGI("param->channel: %d", param->channel);
			for (i = 0; i < VPP_CAM_MAX_CHANNELS; i++) {
				if (g_vpp_camera[i].m_encode_context.codec_id == MEDIA_CODEC_ID_NONE) {
					continue;
				}
				if (g_vpp_camera[i].m_encode_context.instance_index == param->channel) {
					// 填充对外的信息
					enc_params = &g_vpp_camera[i].m_encode_context.video_enc_params;
					param->enable = 1;
					param->width = enc_params->width;
					param->height = enc_params->height;
					param->stream_buf_size = enc_params->bitstream_buf_size;
					if (g_vpp_camera[i].m_encode_context.codec_id == MEDIA_CODEC_ID_H264) {
						param->type = 96;
						param->bitrate = enc_params->rc_params.h264_cbr_params.bit_rate;
						param->framerate = enc_params->rc_params.h264_cbr_params.frame_rate;
					} else if (g_vpp_camera[i].m_encode_context.codec_id == MEDIA_CODEC_ID_H265) {
						param->type = 265;
						param->bitrate = enc_params->rc_params.h265_cbr_params.bit_rate;
						param->framerate = enc_params->rc_params.h265_cbr_params.frame_rate;
					}else{
						SC_LOGE("unsupport codec id %d.", g_vpp_camera[i].m_encode_context.codec_id);
						exit(-1);
					}
					vp_codec_get_user_buffer_param(enc_params, &param->suggest_buffer_region_size,
						&param->suggest_buffer_item_count);
				}
			}
			break;
		}
	case SOLUTION_GET_VENC_CHN_STATUS: // 获取哪些编码通道被使能了
		{
			// 32位的整形，每个通道的状态占其中一个bit
			// 注： 64bit的值位与会有异常，待查
			unsigned int *status = (unsigned int *)val;
			*status = 0;
			int valid_index = 0;
			for (i = 0; i < VPP_CAM_MAX_CHANNELS; i++) {
				if (g_vpp_camera[i].m_encode_context.codec_id != MEDIA_CODEC_ID_NONE) {
					*status |= (1 << valid_index);
					valid_index++;
				}
			}
			SC_LOGI("venc status: 0x%x", *status);
			break;
		}
	case SOLUTION_GET_RAW_FRAME:
		{
			// video_id 代表web上的第几个 video 控件，从1开始计数
			// 需要结合当前使能了多少路pipeline来获取到对应的 pipeline id
			int32_t video_id = *(int32_t *)val;
			int32_t pipeline_id = get_pipeline_id_by_video_id(video_id);
			if (vp_allocate_image_frame(&image_frame) == NULL) {
				SC_LOGE("vp_allocate_image_frame failed");
				return -1;
			}
			ret = vp_vin_get_frame(&g_vpp_camera[pipeline_id].vp_vflow_contex, &image_frame);
			if (ret != 0) {
				SC_LOGE("vp_vin_get_frame failed (%d)", ret);
				vp_free_image_frame(&image_frame);
				return -1;
			}

			hbn_vnode_image_t *hbn_vnode_image = (hbn_vnode_image_t *)image_frame.hbn_vnode_image;

			snprintf(file_name, sizeof(file_name),
				"/tmp/pipeline_%d_vin_chn0_%dx%d_stride_%d_frameid_%d_ts_%ld.raw",
				pipeline_id,
				hbn_vnode_image->buffer.width,
				hbn_vnode_image->buffer.height,
				hbn_vnode_image->buffer.stride,
				hbn_vnode_image->info.frame_id,
				hbn_vnode_image->info.timestamps);

			SC_LOGI("pipeline %d vin dump raw %dx%d(stride:%d), buffer size: %ld frame id: %d,"
				" timestamp: %ld",
				pipeline_id,
				hbn_vnode_image->buffer.width, hbn_vnode_image->buffer.height,
				hbn_vnode_image->buffer.stride,
				hbn_vnode_image->buffer.size[0],
				hbn_vnode_image->info.frame_id,
				hbn_vnode_image->info.timestamps);

			delete_files_with_extension("/tmp", ".raw");
			vp_dump_1plane_image_to_file(file_name, hbn_vnode_image->buffer.virt_addr[0],
				hbn_vnode_image->buffer.size[0]);

			vp_vin_release_frame(&g_vpp_camera[pipeline_id].vp_vflow_contex, &image_frame);
			if (ret != 0) {
				SC_LOGE("vp_vin_release_frame failed.");
				vp_free_image_frame(&image_frame);
				return -1;
			}
			vp_free_image_frame(&image_frame);
			// 通知浏览器下载文件
			SDK_Cmd_Impl(SDK_CMD_WEBSOCKET_UPLOAD_FILE, (void*)file_name);
			break;
		}
	case SOLUTION_GET_ISP_FRAME:
		{
			// video_id 代表web上的第几个 video 控件，从1开始计数
			// 需要结合当前使能了多少路pipeline来获取到对应的 pipeline id
			int32_t video_id = *(int32_t *)val;
			int32_t pipeline_id = get_pipeline_id_by_video_id(video_id);
			if (vp_allocate_image_frame(&image_frame) == NULL) {
				SC_LOGE("vp_allocate_image_frame failed");
				return -1;
			}
			ret = vp_isp_get_frame(&g_vpp_camera[pipeline_id].vp_vflow_contex, &image_frame);
			if (ret != 0) {
				SC_LOGE("vp_isp_get_frame failed (%d)", ret);
				vp_free_image_frame(&image_frame);
				return -1;
			}

			hbn_vnode_image_t* hbn_vnode_image = (hbn_vnode_image_t *)image_frame.hbn_vnode_image;

			snprintf(file_name, sizeof(file_name),
				"/tmp/pipeline_%d_isp_chn0_%dx%d_stride_%d_frameid_%d_ts_%ld.yuv",
				pipeline_id,
				hbn_vnode_image->buffer.width,
				hbn_vnode_image->buffer.height,
				hbn_vnode_image->buffer.stride,
				hbn_vnode_image->info.frame_id,
				hbn_vnode_image->info.timestamps);

			SC_LOGI("pipeline %d isp dump yuv %dx%d(stride:%d), buffer size: %ld frame id: %d,"
				" timestamp: %ld",
				pipeline_id,
				hbn_vnode_image->buffer.width, hbn_vnode_image->buffer.height,
				hbn_vnode_image->buffer.stride,
				hbn_vnode_image->buffer.size[0],
				hbn_vnode_image->info.frame_id,
				hbn_vnode_image->info.timestamps);

			delete_files_with_extension("/tmp", ".yuv");
			vp_dump_2plane_yuv_to_file(file_name,
				hbn_vnode_image->buffer.virt_addr[0],
				hbn_vnode_image->buffer.virt_addr[1],
				hbn_vnode_image->buffer.size[0],
				hbn_vnode_image->buffer.size[1]);

			vp_isp_release_frame(&g_vpp_camera[pipeline_id].vp_vflow_contex, &image_frame);
			if (ret != 0) {
				SC_LOGE("vp_isp_release_frame failed.");
				vp_free_image_frame(&image_frame);
				return -1;
			}
			vp_free_image_frame(&image_frame);
			// 通知浏览器下载文件
			SDK_Cmd_Impl(SDK_CMD_WEBSOCKET_UPLOAD_FILE, (void*)file_name);
			break;
		}
	case SOLUTION_GET_PYM_FRAME:
		{
			// video_id 代表web上的第几个 video 控件，从1开始计数
			// 需要结合当前使能了多少路pipeline来获取到对应的 pipeline id
			int32_t video_id = *(int32_t *)val;
			int32_t pipeline_id = get_pipeline_id_by_video_id(video_id);
			if (vp_allocate_image_frame(&image_frame) == NULL) {
				SC_LOGE("vp_allocate_image_frame failed");
				return -1;
			}
			ret = vp_pym_get_frame(&g_vpp_camera[pipeline_id].vp_vflow_contex, &image_frame);
			if (ret != 0) {
				SC_LOGE("vp_pym_get_frame failed (%d)", ret);
				vp_free_image_frame(&image_frame);
				return -1;
			}

			hbn_vnode_image_t *hbn_vnode_image = (hbn_vnode_image_t *)image_frame.hbn_vnode_image;

			snprintf(file_name, sizeof(file_name),
				"/tmp/pipeline_%d_pym_ochn0_%dx%d_stride_%d_frameid_%d_ts_%ld.yuv",
				pipeline_id,
				hbn_vnode_image->buffer.width,
				hbn_vnode_image->buffer.height,
				hbn_vnode_image->buffer.stride,
				hbn_vnode_image->info.frame_id,
				hbn_vnode_image->info.timestamps);

			SC_LOGI("pipeline %d pym dump yuv %dx%d(stride:%d), buffer size: %ld frame id: %d,"
				" timestamp: %ld",
				pipeline_id,
				hbn_vnode_image->buffer.width, hbn_vnode_image->buffer.height,
				hbn_vnode_image->buffer.stride,
				hbn_vnode_image->buffer.size[0],
				hbn_vnode_image->info.frame_id,
				hbn_vnode_image->info.timestamps);

			delete_files_with_extension("/tmp", ".yuv");
			vp_dump_2plane_yuv_to_file(file_name,
				hbn_vnode_image->buffer.virt_addr[0],
				hbn_vnode_image->buffer.virt_addr[1],
				hbn_vnode_image->buffer.size[0],
				hbn_vnode_image->buffer.size[1]);

			vp_pym_release_frame(&g_vpp_camera[pipeline_id].vp_vflow_contex, &image_frame);
			if (ret != 0) {
				SC_LOGE("vp_pym_release_frame failed.");
				vp_free_image_frame(&image_frame);
				return -1;
			}
			vp_free_image_frame(&image_frame);
			// 通知浏览器下载文件
			SDK_Cmd_Impl(SDK_CMD_WEBSOCKET_UPLOAD_FILE, (void*)file_name);
			break;
		}
	default:
		{
			ret= -1;
			break;
		}
	}
	return ret;
}
