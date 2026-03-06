#include <stdio.h>
#include "vp_common.h"
#define YNR_MAX_WIDTH 2048
#define YNR_MAX_HEIGHT 2048

static int g_isp0_next_slot_id = 4;
static int g_isp1_next_slot_id = 4;

void vp_flow_info_reset(){
	g_isp0_next_slot_id = 4;
	g_isp1_next_slot_id = 4;
}
void vp_flow_info_init(const vp_sensor_config_t *sensor_config, vp_flow_info_t* vp_flow_info){

	if(sensor_config->sensor_type == SENSOR_TYPE_GMSL_YUV){
		vp_flow_info->type = VP_FLOW_ISP_BYPASS;
	}else{
		if(sensor_config->ynr_attr == NULL){
			vp_flow_info->type = VP_FLOW_ISP_ONLY;
		}else if((sensor_config->camera_config->width > YNR_MAX_WIDTH) ||
			(sensor_config->camera_config->height > YNR_MAX_HEIGHT)){
			vp_flow_info->type = VP_FLOW_ISP_ONLY;
		}else{
			vp_flow_info->type = VP_FLOW_ISP_YNR;
		}
	}
	// 情景1(不需要ISP)：Vin_x -offline- PYM0
	if(vp_flow_info->type == VP_FLOW_ISP_BYPASS){
		vp_flow_info->pym_slot_id = 0; 		  //离线模式的情况不用设置
		vp_flow_info->pym_hw_id = 0;  		  //固定设置为0
		vp_flow_info->pym_mode = PYM_M2M_MODE; //离线模式

		vp_flow_info->is_online_vin_pym = 0;
	// 情景2(需要ISP， 不需要YNR): Vin_x -offline- ISP0 -offline- PYM0
	}else if(vp_flow_info->type == VP_FLOW_ISP_ONLY){
		vp_flow_info->isp_mode = SCHED_MODE_MANUAL;
		vp_flow_info->isp_hw_id = 0;
		vp_flow_info->isp_slot_id = g_isp0_next_slot_id;
		g_isp0_next_slot_id++;

		vp_flow_info->pym_slot_id = vp_flow_info->isp_slot_id; 		  //离线模式的情况不用设置
		vp_flow_info->pym_hw_id = 0;  		  //固定设置为0
		vp_flow_info->pym_mode = PYM_M2M_MODE; //离线模式

		vp_flow_info->is_online_vin_isp = 0;
		vp_flow_info->is_online_isp_pym = 0;
	// 情景3(ISP + YNR): Vin_x -offline- ISP1 -online- YNR1 -online- PYM1
	}else if(vp_flow_info->type == VP_FLOW_ISP_YNR){
		vp_flow_info->isp_mode = SCHED_MODE_MANUAL;
		vp_flow_info->isp_hw_id = 1;
		vp_flow_info->isp_slot_id = g_isp1_next_slot_id;
		g_isp1_next_slot_id++;

		vp_flow_info->ynr_mode = 1; //1:Manaul 模式	2:全online模式
		vp_flow_info->ynr_slot_id = vp_flow_info->isp_slot_id;

		vp_flow_info->pym_slot_id = vp_flow_info->isp_slot_id;
		vp_flow_info->pym_hw_id = 1;
		vp_flow_info->pym_mode = PYM_MANUAL_MODE;

		vp_flow_info->is_online_vin_isp = 0;
		vp_flow_info->is_online_isp_ynr = 1;
		vp_flow_info->is_online_ynr_pym = 1;
	}else{
		//error
	}
}

void vp_flow_info_show(const vp_flow_info_t* vp_flow_info, int vp_flow_index){

	// 情景1(不需要ISP)：Vin_x -offline- PYM0
	if(vp_flow_info->type == VP_FLOW_ISP_BYPASS){
		printf("	[%d] not use isp.\n", vp_flow_index);
		printf("		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			vp_flow_info->pym_slot_id, vp_flow_info->pym_hw_id, vp_flow_info->pym_mode);

	// 情景2(需要ISP， 不需要YNR): Vin_x -offline- ISP0 -offline- PYM0
	}else if(vp_flow_info->type == VP_FLOW_ISP_ONLY){
		printf("	[%d] only use isp.\n", vp_flow_index);
		printf("		isp [hw:%d] [slot_id:%d] [mode:%d]\n",
			vp_flow_info->isp_hw_id, vp_flow_info->isp_slot_id, vp_flow_info->isp_mode);
		printf("		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			vp_flow_info->pym_hw_id, vp_flow_info->pym_slot_id, vp_flow_info->pym_mode);

	// 情景3(ISP + YNR): Vin_x -offline- ISP1 -online- YNR1 -online- PYM1
	}else if(vp_flow_info->type == VP_FLOW_ISP_YNR){
		printf("	[%d] use isp + ynr.\n", vp_flow_index);
		printf("		isp [hw:%d] [slot_id:%d] [mode:%d]\n",
			vp_flow_info->isp_hw_id, vp_flow_info->isp_slot_id, vp_flow_info->isp_mode);
		printf("		ynr [hw:%d] [slot_id:%d] [mode:%d]\n",
			1, vp_flow_info->ynr_slot_id, vp_flow_info->ynr_mode);
		printf("		pym [hw:%d] [slot_id:%d] [mode:%d]\n",
			vp_flow_info->pym_hw_id, vp_flow_info->pym_slot_id, vp_flow_info->pym_mode);
	}else{
		//error
	}
}

