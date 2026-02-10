#include "mainTask.h"
#include "Application.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include "DdsFunc.h"
#include "../libSourceCode/commDas/area.h"
#include "comm_func_crc16.h"//20260201

#define CONTROLTEM 0
#define PIU_VIEW_GAIN 10
#define PIU_CMD_RESEND_CYCLES 2
static short clamp_int_to_short(int value)
{
	if(value > 32767)
	{
		return 32767;
	}
	if(value < -32768)
	{
		return -32768;
	}
	return (short)value;
}
/* flush all pending DDS samples on a topic (avoid using stale samples after failure) */
static void dtms_flush_dds_topic(int conn_id)
{
	unsigned int save_message_size = message_size;
	unsigned int save_message_type_id = (unsigned int)message_type_id;
	unsigned int save_transaction_id = (unsigned int)transaction_id;
	int save_enRetCode = enRetCode;
	static unsigned char buf[RECV_MAX_SIZE];
	int guard = 0;

	while(guard < 128)
	{
		message_size = RECV_MAX_SIZE;
		Receive_Message(conn_id, 0, &transaction_id, buf, &message_type_id, &message_size, &enRetCode);
		if(enRetCode != 0)
		{
			break;
		}
		guard++;
	}

	message_size = save_message_size;
	message_type_id = save_message_type_id;
	transaction_id = save_transaction_id;
	enRetCode = save_enRetCode;
}

// 目标融合相关函数
int sendSfsMsgToQuePort(unsigned char* msgA, long int const lenSend);// 向融合发送目标信息
extern int sendVoiceMsgToDtps(unsigned char* msgA, long int const lenSend);//转发给DTPS

// 双机编队发布上下文（避免发布过程中被新的039覆盖）
static unsigned int g_bdfx_double_plan_id = 0;
static int g_bdfx_double_busy_cnt = 0;

// 普通/反潜发布上下文（避免重复039导致反复重置）
static unsigned char g_fabu_active = 0;
static unsigned int g_fabu_plan_id = 0;
static unsigned int g_fabu_route_type = 0;
static unsigned int g_fabu_stage_id = 0;
static int g_fabu_send_uav_num = 0;
static int g_fabu_switch_timeout = 0;
static int g_fabu_switch_ack_cnt = 0;
static unsigned int g_single_request_planning_id = 0;
static unsigned int g_single_uav_request_plan_id = 0;
static unsigned int alloc_single_request_plan_id(void)
{
	if(g_single_request_planning_id < planning_id)
	{
		g_single_request_planning_id = planning_id;
	}
	g_single_request_planning_id++;
	if(g_single_request_planning_id == 0)
	{
		g_single_request_planning_id = 1;
	}
	return g_single_request_planning_id;
}
static unsigned int get_single_request_plan_id(void)
{
	if(g_single_request_planning_id == 0)
	{
		return planning_id;
	}
	return g_single_request_planning_id;
}
static float get_bdfx_double_target_height(unsigned int plan, int uav_index)
{
	unsigned int drone_id = 0;
	if(plan < 3 && uav_index >= 0 && uav_index < UAV_MAX_NUM)
	{
		drone_id = blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.drone_num;
	}
	if(drone_id == 0 && uav_index >= 0 && uav_index < UAV_MAX_NUM)
	{
		drone_id = CCC_DPU_data_3.drone_specific_informations[uav_index].platform_num;
	}
	if(load_file.lead_uav_id != 0 && drone_id == load_file.lead_uav_id)
	{
		return 460.0f;
	}
	if(load_file.lead_uav_id != 0 && drone_id != 0)
	{
		return 660.0f;
	}
	return (uav_index == 0) ? 460.0f : 660.0f;
}
static void sync_bdfx_double_plan_height(unsigned int plan)
{
	if(plan >= 3)
	{
		return;
	}
	for(int uav_index = 0; uav_index < 2; uav_index++)
	{
		planning_informations *p_plan =
				&blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations;
		unsigned short point_num = p_plan->waypoints_number;
		if(point_num == 0)
		{
			continue;
		}
		if(point_num > 250)
		{
			point_num = 250;
		}
		float target_height = get_bdfx_double_target_height(plan, uav_index);
		p_plan->mission_height = (unsigned int)target_height;
		for(unsigned short point_index = 0; point_index < point_num; point_index++)
		{
			p_plan->planning_information_waypoint_informations[point_index].height_validity = 1;
			p_plan->planning_information_waypoint_informations[point_index].height = target_height;
		}
	}
}
static int should_sync_bdfx_double_plan_height(unsigned int plan)
{
	if(plan >= 3)
	{
		return 0;
	}
	if(CCC_DPU_data_6_Ofp[plan].plan_release_mode == 6)
	{
		return 1;
	}
	planning_informations *p_plan0 =
			&blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations;
	planning_informations *p_plan1 =
			&blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations;
	if(p_plan0->waypoints_number == 0 || p_plan1->waypoints_number == 0)
	{
		return 0;
	}
	if(p_plan0->mission_type == 9 && p_plan1->mission_type == 9)
	{
		return 1;
	}
	return 0;
}
/********************************************** 仿真完整功能封装模块 ***********************************/
// 周期接收有人机导航书数据 模拟检测综显上线
void Main_Task()
{
	//心跳打印
	static int heartbit = 0;
	heartbit++;
	if(heartbit % 100 == 0)
	{
		printf("*** DTMS online ***\n");
	}
	// 检测综显上线 并运行，有人机广播轮载信号，作为上线心跳信息
	recv_dpu1_dpu2(DDSTables.DPU_CCC_NT.niConnectionId,DDSTables.DPU2_CCC_NT.niConnectionId,&ofp_nt_broadcast_msg,sizeof ofp_nt_broadcast_msg);
	if(enRetCode == 0)
	{

	}

	// 4.3 有人机导航数据
	recv_dpu1_dpu2(DDSTables.DPU_CCC_4.niConnectionId,DDSTables.DPU2_CCC_4.niConnectionId,&DPU_CCC_data_4,sizeof DPU_CCC_data_4);
	if (enRetCode == 0)
	{
		// 保存收到的有人机经纬度  用于动态设置预加载区域
		plane_lat = DPU_CCC_data_4.manned_longitude_and_latitude_synt.latitude;
		plane_lon = DPU_CCC_data_4.manned_longitude_and_latitude_synt.longitude;
		static int cnt = 0;
		cnt++;
		if(cnt == 30)
		{
			//初始化任务区
			init_blk_ccc_ofp_033();
			//初始化任务点
			init_blk_ccc_ofp_034();
			//长机默认值
			load_file.lead_uav_id = 0x1005;
			//初始化应急航线
//			init_blk_ccc_ofp_045();
//			target_estimation_test();
		}
		DPU_online_flag = 1;

		//发送给pad new20250620
		if(Pad_heart_flag == 1)
		{
			Send_Message(DDSTables.CCC_PAD_777.niConnectionId,0,&transaction_id, &DPU_CCC_data_4, &message_type_id, sizeof (DPU_CCC_data_4), &enRetCode);
		}

	}

	// TODO  有人机掉线逻辑 不完善 存在掉线后再次初始化
	if (DPU_online_flag == 1 && online_flag == 0)
	{
		/**************** 发送初始信息 ***************/
		// TODO 未确定好信息源，为假定值测试
		send_blk_ccc_ofp_033();       // 任务区
		send_blk_ccc_ofp_034();     // 任务点
		send_blk_ccc_ofp_035();      // 任务线
		/**************** 各个模块在综显上线后周期运行 ******************/
		//协同指令集循环队列创建
		createQueue(&queue);
		// 其余模块
		online_flag = 1;
		//初始化一些全局参数
		init_param();
	}
	else if (DPU_online_flag == 1 && online_flag == 1)
	{
		/**************** 各个模块在综显上线后周期运行 ******************/

		recv_all_message();
		/******接收PAD消息****/
		recv_blk_dpu_dlr_011();            // 预规划加载
		send_blk_ccc_ofp_033();            // 预加载区域 只发送一次会出现接收不到情况 改为周期发送
		send_blk_ccc_ofp_034();            // 任务点
		uav_status_handle();               // 无人机飞控状态数据处理模块
		formulate_moduel();                // 全局规划模块
		single_mission_target_plan();      // 单任务区/目标规划模块
		single_uav_plan();                 // 单无人机规划模块
		BDFX_rtn();						   // 编队飞行回报
		BDFX_double_rtn();				   // 双机编队
		rev_zhanfa_plan();                 // 接收辅助决策的战法方案
		stage_send();                      // 分阶段发送航线
		recv_blk_ctas_dtms_009();		   // CTAS解算反馈
		recv_blk_ctas_dtms_010();          // 接收空域信息
		uav_simulation();                  // 无人机飞仿控制模块。(接收综显"发布"指令，注入uav航线)
		send_track_change();               // 无人机航线切换
		send_blk_ccc_ofp_006();            // 无人机下一阶段提示
		scheme_replan();                   // 任务分配结果编辑，任务区编辑
		UAV_Takeover_Process();            // 无人机控制权交接
		// 光电相关
		recv_DPU_CCC_MFD();                           // 接收无人机光电视频MFD控制 DPU-CCC
		send_blk_ccc_kkl_008_026_027_028();             // 发送无人机遥控帧（包含基本遥控指令帧64字节；飞行航线威胁指令帧32字节；任务控制指令帧 64字节） CCC-KKL
		recv_telemetry_data_subframe2();                // 接收载荷遥测与指令响应帧，并解析其中的遥测数据子帧2（光电数据） KKL-CCC
		send_uav_photoelectric_video_ctrl_feedback();   // 将光电视频控制反馈发送给综显 CCC-DPU
		/*发送周期的消息******/
		send_period_message();
#if _PC_SIMULATION_    //仿真
		// 数字环境下的心跳和版本上报处理
		sendSimulateHeartBeat();
#endif
		// 航线冲突处理
		avoidLineCrashProc();
		payload_listen();					//监听无人机载荷

	}
}



/*************************************相关辅助函数*****************************************/
//接收DPU1/DPU2数据处理
void recv_dpu1_dpu2(SINT32_SIX dpu_1,SINT32_SIX dpu_2,void*buf,int len)
{
	Receive_Message(dpu_1, 0, &transaction_id, dds_data_rece.dataA , &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		//收到DPU1就不收DPU2
		memcpy(buf,dds_data_rece.dataA,len);
		//收到DPU1之后把DPU2队列清除
		char lost_buf[4096];
		Receive_Message(dpu_2, 0, &transaction_id, lost_buf , &message_type_id, &message_size, &enRetCode);
		//丢掉后把接收标志恢复
		enRetCode = 0;
	}
	else if(enRetCode != 0)
	{
		//收不到DPU1收DPU2
		Receive_Message(dpu_2, 0, &transaction_id, dds_data_rece.dataA , &message_type_id, &message_size, &enRetCode);
		if(enRetCode == 0)
		{
			memcpy(buf,dds_data_rece.dataA,len);
			//收到DPU2之后把DPU1队列清除
			char lost_buf[4096];
			Receive_Message(dpu_1, 0, &transaction_id, lost_buf , &message_type_id, &message_size, &enRetCode);
			//丢掉后把接收标志恢复
			enRetCode = 0;
		}
	}
}

void init_param()
{
	//初始化无人机任务状态
	memset(&uav_route,0,sizeof(UAV_ROUTE) * 4);
	//初始化无人机信息
	memset(&CCC_DPU_data_3,0,sizeof(drone_state_information));
	//初始化无人机编队信息
	memset(&CCC_DPU_data_4,0,sizeof(formation_link_status_information));
}
//发送信息数据对齐
void align_send_information(void *send_struct, int length, int startPos)
{
	memcpy(send_array.dataA+ startPos, send_struct,length);
	data_length = length - startPos;
	//qDebug()<<"length:"<<data_length;
}

// 用于赋值有效位
int get_bit(unsigned short data, int n)
{
	unsigned short mask = 1 << n; // 创建掩码
	return (data & mask) >> n; // 移位并返回第n位的值
}
int get_int8_bit(char data, int n)
{
	char mask = 1 << n; // 创建掩码
	return (data & mask) >> n; // 移位并返回第n位的值
}
// 给short类型具体位赋值
//
int give_bit(unsigned short data, int n, int value)
{
	if (value == 1 || value == 0)
	{
		data = data | (value , n);
	}
	return data;
}

unsigned short setBit(unsigned short data, int position, int value)
{
	if (value == 1)
	{
		data |= (1 << position);
	}
	else
	{
		unsigned short mask = ~(1<<position);
		data &= mask;
	}
	return data;
}
unsigned short combine_char2_short(unsigned char char1, unsigned char char2)
{
	return (unsigned short)((char2<<8)|char1);
}

//// 光电视频接收显示在DTMS ok
//  void updata_video(QImage img,QByteArray msg){
//      ui.video_test.setText("111111");
//      // //qDebug() ,"图片大小：",img.size();
//      ui.video_test.setPixmap(QPixmap::fromImage(img));
//  }

//按十六进制单字节打印收到的数据，每行16个字节
void printRawData(char* data, unsigned int lenth)
{
	unsigned int len = 0;
	for(len = 0; len < lenth; len++)
	{
		if(len == 0)
		{
			printf("----RAW DATA----\n");
		}
		printf("0x%02x ",*(data+len));
		if((((len+1) % 0x0F) == 0) || (len == (lenth - 1)))
		{
			printf("\n");
		}
	}
}

//航线查询
void uav_hx()
{
	//计时五分钟发一次
	static int hx_cnt = 0;
	static int send_cout = 0;
	hx_cnt ++;
	//是否发送
	if(hx_cnt > 6000 && hx_cx_flag == 1 && formationId[0].isControl == 1)
	{
		send_cout = 4;
		hx_cnt = 0;
	}
	//发送四次
	if(send_cout > 0)
	{
		send_uav_hl(CCC_DPU_data_3.drone_specific_informations[0].platform_num,0x3c);
		send_cout--;
	}
	//失去控制权就结束发送
	if(formationId[0].isControl == 0)
	{
		hx_cx_flag = 0;
	}
}

//双机编队
static char bdfx_double_hx_point[2] = {0,0};
static int bdfx_double_cnt_0x30[2] = {4,4};
static unsigned char bdfx_double_done[2] = {0,0};
static int bdfx_double_expect_route[2] = {0, 0};
static int bdfx_single_expect_route[UAV_MAX_NUM] = {0,0,0,0};
static int bdfx_single_uav_index = -1;
static int build_bdfx_double_air_area(unsigned int plan, int uav_index, double half_width_km, area_information *out_area);
static int build_bdfx_single_air_area(unsigned int plan, int uav_index, double half_width_km, area_information *out_area);
void BDFX_double_rtn()
{
	/* Double-UAV BDON(0x40 route switch) fix: send_index can target only one UAV per cycle.
	 * Old code sent BDON to UAV0 and UAV1 in the same cycle, so the second overwrote the first.
	 * We now send BDON sequentially across cycles (3 sends per UAV) before waiting for ACK.
	 */
	static unsigned char bdfx_inited = 0;
	static unsigned char bdon_uav = 0;
	static unsigned char bdon_done[2] = {0,0};
	static unsigned char bdon_inited = 0;

	// reset helper state when leaving BDFX state
	if(BDFX_double_status != 1)
	{
		bdfx_inited = 0;
	}

	// reset helper state when leaving BDON state
	if(BDFX_double_status != 2)
	{
		bdon_inited = 0;
	}

	//注入航线0x30
	if(BDFX_double_status == 1)
	{
		if(!bdfx_inited)
		{
			for(int uav = 0 ; uav < 2 ; uav++)
			{
				bdfx_double_hx_point[uav] = 0;
				bdfx_double_cnt_0x30[uav] = 4;
				bdfx_double_done[uav] = 0;
				bdfx_double_expect_route[uav] = 0;
			}
			bdfx_inited = 1;
		}

		bdon_uav = 0;
		bdon_done[0] = 0;
		bdon_done[1] = 0;
		bdon_inited = 0;
		for(int uav = 0 ; uav < 2 ; uav++)
			double_uav_BDFX(uav);

		if(bdfx_double_done[0] && bdfx_double_done[1])
		{
			BDFX_double_status = 2;
			bdfx_inited = 0;
		}
	}
	//编队航点切换0x14
	else if(BDFX_double_status == 2)
	{
		if(!bdon_inited)
		{
			bdon_uav = 0;
			bdon_done[0] = 0;
			bdon_done[1] = 0;
			send_cnt2[0] = 0;
			send_cnt2[1] = 0;
			s4D_frame_40[0] = 0;
			s4D_frame_40[1] = 0;
			bdfx_double_expect_route[0] = 0;
			bdfx_double_expect_route[1] = 0;
			bdon_inited = 1;
		}

		// pick a UAV that is not done
		if(bdon_done[bdon_uav])
		{
			bdon_uav = (bdon_uav == 0) ? 1 : 0;
		}

		if(!bdon_done[bdon_uav])
		{
			int pre_cnt = send_cnt2[bdon_uav];
			double_uav_BDON(bdon_uav);
			// pre_cnt==2 means this call is the 3rd send (double_uav_BDON resets send_cnt2 to 0)
			if(pre_cnt >= 2)
			{
				bdon_done[bdon_uav] = 1;
				bdon_uav = (bdon_uav == 0) ? 1 : 0;
			}
		}

		if(bdon_done[0] && bdon_done[1])
		{
			// keep send_index for this cycle; do not clear here
			BDFX_double_status = 3;
			bdon_inited = 0;
		}
	}
	else if(BDFX_double_status == 3)//接收航点切换回报
	{
		static int timeout_BDQH = 0;
		timeout_BDQH ++ ;

		for (int uav = 0; uav < 2; uav++)
		{
			if (s4D_frame_40[uav] != 1 && bdfx_double_expect_route[uav] > 0 && uav_route[uav].route_number == bdfx_double_expect_route[uav])
			{
				s4D_frame_40[uav] = 1;
				printf("BDFX infer success uav=%d route=%d\n", uav, bdfx_double_expect_route[uav]);
			}
		}
		if ((timeout_BDQH % 20) == 1)
		{
			printf("BDFX wait t=%d ack0=%d ack1=%d route0=%d/%d route1=%d/%d\n",
				   timeout_BDQH,
				   s4D_frame_40[0], s4D_frame_40[1],
				   uav_route[0].route_number, bdfx_double_expect_route[0],
				   uav_route[1].route_number, bdfx_double_expect_route[1]);
		}
		if(s4D_frame_40[0] == 1 && s4D_frame_40[1] == 1)
		{

			//发布成功
			s4D_frame_40[0] = 0;
			s4D_frame_40[1] = 0;
			bdfx_double_expect_route[0] = 0;
			bdfx_double_expect_route[1] = 0;
			BDFX_double_status = 0;
			timeout_BDQH = 0;
			//找到方案编号索引，没有运行中的方案plan为零
				unsigned int plan = g_bdfx_double_plan_id % 3;
				sync_bdfx_double_plan_height(plan);
				planning_id = g_bdfx_double_plan_id;
				//双机编队发布成功后，优先以发送给OFP的方案为准，避免重启后运行方案不同步导致航线消失
			CCC_DPU_data_6_Ofp[plan].plan_release_mode = 6;
			memcpy(&CCC_DPU_data_6[plan],&CCC_DPU_data_6_Ofp[plan],sizeof(BLK_CCC_OFP_019));
			for(int uav = 0; uav < 2; uav++)
			{
				int points_number =
						blk_ccc_ofp_024_cunchu[plan][uav].individual_drone_routing_programs.planning_informations.waypoints_number;
				int max_points = (int)(sizeof(uav_route[uav].waypoint) / sizeof(uav_route[uav].waypoint[0]));
				if(points_number <= 0)
				{
					uav_route[uav].tasking = 0;
					continue;
				}
				if(points_number > max_points)
				{
					points_number = max_points;
				}
				uav_route[uav].tasking = 1;
				uav_route[uav].task_type =
						blk_ccc_ofp_024_cunchu[plan][uav].individual_drone_routing_programs.planning_informations.mission_type;
				uav_route[uav].task_id =
						blk_ccc_ofp_024_cunchu[plan][uav].individual_drone_routing_programs.planning_informations.subtask_ID_number;
				uav_route[uav].hull_number = points_number + 3;
				memcpy(&uav_route[uav].waypoint[0],
						&blk_ccc_ofp_024_cunchu[plan][uav].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
						sizeof(planning_information_waypoint_information) * points_number);
			}
			// 发送运行分配结果
			memcpy(&blk_ccc_ofp_017, &CCC_DPU_data_6_Ofp[plan],
					sizeof(BLK_CCC_OFP_017));
//			if(blk_ccc_ofp_019.platform_num>=2){//双机编队判断
			blk_ccc_ofp_017.plan_release_mode = 6;
//			}

			send_blk_ccc_ofp_017();
				printf("BDFX finish plan=%u mode=%u\n",g_bdfx_double_plan_id,blk_ccc_ofp_017.plan_release_mode);
				// 再次发送无人机航线（运行方案）
			send_blk_ccc_ofp_024(plan);
			int area_local_ready = 0;
			for (int uav = 0; uav < 2; uav++)
			{
				area_information tmp_area;
				if (build_bdfx_double_air_area(plan, uav, 1.5, &tmp_area))
				{
					memcpy(&air_area.area_informations[uav + 1], &tmp_area, sizeof(area_information));
					send_area[uav] = 1;
					area_local_ready = 1;
					printf("BDFX local area uav=%d code=%u width=3.0km\n", uav, tmp_area.area_code);
				}
				else
				{
					printf("BDFX local area skip uav=%d points=%u\n",
						   uav,
						   blk_ccc_ofp_024_cunchu[plan][uav].individual_drone_routing_programs.planning_informations.waypoints_number);
				}
			}
			if (area_local_ready == 1)
			{
				init_blk_ctas_dtms_010();
				send_blk_ccc_ofp_033();
				printf("BDFX local area applied plan=%u\n", g_bdfx_double_plan_id);
			}
			scheme_generation_state(2,2, 2, 2);            //发送发布完成状态到综显
		}
		else if (s4D_frame_40[0] == -1 && s4D_frame_40[1] == -1)
		{
			//发布失败
			BDFX_double_status = 0;
			timeout_BDQH = 0;
			bdfx_double_expect_route[0] = 0;
			bdfx_double_expect_route[1] = 0;
			//发送发布失败状态到综显
			sprintf(CCC_DPU_data_0.failreason, "发送航线切换指令返回失败");
			scheme_generation_state(2,2, 3, 2);
			memset(CCC_DPU_data_0.failreason, 0, 200);
			printf("BDFX fail both -1\n");
		}
		else if (timeout_BDQH > 300)
		{
			//发布超时
			BDFX_double_status = 0;
			timeout_BDQH = 0;
			bdfx_double_expect_route[0] = 0;
			bdfx_double_expect_route[1] = 0;
			//发送发布失败状态到综显
			sprintf(CCC_DPU_data_0.failreason, "发送航线切换指令超时");
			scheme_generation_state(2,2, 3, 2);
			memset(CCC_DPU_data_0.failreason, 0, 200);
			printf("BDFX timeout ack0=%d ack1=%d route0=%d/%d route1=%d/%d\n",
				   s4D_frame_40[0], s4D_frame_40[1],
				   uav_route[0].route_number, bdfx_double_expect_route[0],
				   uav_route[1].route_number, bdfx_double_expect_route[1]);
		}
	}


}

void BDFX_rtn()
{
	//注入航线0x30
	if(BDFX_status == 1)
	{
		single_uav_BDFX();
	}
	//编队航点切换0x14
	else if(BDFX_status == 2)
	{
		single_uav_BDON();
	}
	else if(BDFX_status == 3)//接收航点切换回报
	{
		static int timeout_BDQH = 0;
		timeout_BDQH ++ ;
		int uav_index = bdfx_single_uav_index;
		if(uav_index < 0 || uav_index >= UAV_MAX_NUM)
		{
			uav_index = DPU_CCC_data_11.drone_num -1;
		}
		if(uav_index < 0 || uav_index >= UAV_MAX_NUM)
		{
			return;
		}
		if(s4D_frame_40[uav_index] != 1
				&& bdfx_single_expect_route[uav_index] > 0
				&& uav_route[uav_index].route_number == bdfx_single_expect_route[uav_index])
		{
			s4D_frame_40[uav_index] = 1;
			printf("BDFX single infer success uav=%d route=%d\n",uav_index,bdfx_single_expect_route[uav_index]);
		}
		if(s4D_frame_40[uav_index] == 1)
		{
			//发布成功
			s4D_frame_40[uav_index] = 0;
			bdfx_single_expect_route[uav_index] = 0;
			BDFX_status = 0;
			bdfx_single_uav_index = -1;
			timeout_BDQH = 0;
			//找到方案编号索引，没有运行中的方案plan为零
			unsigned int plan_id = g_single_uav_request_plan_id;
			if(plan_id == 0)
			{
				plan_id = get_single_request_plan_id();
			}
			unsigned int plan = plan_id % 3;
			planning_id = plan_id;
			//存入任务分配结果
			memcpy(&CCC_DPU_data_6[plan],&blk_ccc_ofp_019,sizeof(BLK_CCC_OFP_019));
			//单无人机编队发布成功后，固化运行方案模式
			CCC_DPU_data_6[plan].plan_release_mode = 1;
			CCC_DPU_data_6_Ofp[plan].plan_release_mode = 1;
			//存入单无人机航线
			memcpy(&blk_ccc_ofp_024_cunchu[plan][uav_index],&blk_ccc_ofp_024_single[uav_index],sizeof(BLK_CCC_OFP_024_cunchu));
			int points_number =
					blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.waypoints_number;
			int max_points = (int)(sizeof(uav_route[uav_index].waypoint) / sizeof(uav_route[uav_index].waypoint[0]));
			if(points_number <= 0)
			{
				uav_route[uav_index].tasking = 0;
			}
			else
			{
				if(points_number > max_points)
				{
					points_number = max_points;
				}
				uav_route[uav_index].tasking = 1;
				uav_route[uav_index].task_type =
					blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.mission_type;
				uav_route[uav_index].task_id =
					blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.subtask_ID_number;
				uav_route[uav_index].hull_number = points_number + 3;
				memcpy(&uav_route[uav_index].waypoint[0],
						&blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
						sizeof(planning_information_waypoint_information) * points_number);
			}
			// 发送单无人机务运行分配结果
			blk_ccc_ofp_019.plan_release_mode = 1;            //已发布
			send_blk_ccc_ofp_021();
			// 再次发送无人机航线（运行方案）
			send_blk_ccc_ofp_024(plan);
            area_information tmp_area;
            if (build_bdfx_single_air_area(plan, uav_index, 1.5, &tmp_area))
            {
                memcpy(&air_area.area_informations[uav_index + 1], &tmp_area, sizeof(area_information));
                send_area[uav_index] = 1;
                init_blk_ctas_dtms_010();
                send_blk_ccc_ofp_033();
                printf("BDFX single local area uav=%d code=%u width=3.0km\n", uav_index, tmp_area.area_code);
            }
            else
            {
                printf("BDFX single local area skip uav=%d points=%u\n",
                       uav_index,
                       blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.waypoints_number);
            }
			scheme_generation_state(0,2, 2, 2);            //发送发布完成状态到综显
		}
		else if(s4D_frame_40[uav_index] == -1)
		{
			//发布失败
			BDFX_status = 0;
			bdfx_single_uav_index = -1;
			bdfx_single_expect_route[uav_index] = 0;
			timeout_BDQH = 0;
			//发送发布失败状态到综显
			sprintf(CCC_DPU_data_0.failreason, "发送航线切换指令返回失败");
			scheme_generation_state(0,2, 3, 2);
//			scheme_generation_state(2,2, 3, 2);//260130
			memset(CCC_DPU_data_0.failreason, 0, 200);
		}
		else if(timeout_BDQH > 200)
		{
			//发布超时
			BDFX_status = 0;
			bdfx_single_uav_index = -1;
			bdfx_single_expect_route[uav_index] = 0;
			timeout_BDQH = 0;
			//发送发布失败状态到综显
			sprintf(CCC_DPU_data_0.failreason, "发送航线切换指令超时");
			scheme_generation_state(0,2, 3, 2);
			memset(CCC_DPU_data_0.failreason, 0, 200);
		}
	}



}


// 存储航路文件函数
/***********************DLR预规划加载*********************/
//接受加载命令
void recv_blk_dpu_dlr_011()
{
#if _PC_SIMULATION_ //仿真
	// DPU_DLR 启动加载命令
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.DPU_DLR_0.niConnectionId, 0, &transaction_id, &blk_dpu_dlr_011, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0 && blk_dpu_dlr_011.para_load_req_m1.load_status == 1)//启动加载
		//    if(1)
	{
		//反馈综显正在加载
		blk_ccc_dlr_000.load_progress = 0;
		send_blk_ccc_dlr_000(1);
		//反馈综显正在加载
		blk_ccc_dlr_000.load_progress = 20;
		send_blk_ccc_dlr_000(1);
		//读取文件数据loadTaskFile
		int rtn = loadTaskFile((char*)&load_file, sizeof(FILE_DATA));
		if(rtn == 0)
		{
			printf("load file success\n");
			//反馈综显传输完成
			blk_ccc_dlr_000.load_progress = 100;
			send_blk_ccc_dlr_000(2);
			//清零
			memset(&blk_ccc_dlr_000,0,sizeof(BLK_CCC_DLR_000));
			//使用预规划加载的任务区和任务点
			memcpy(&blk_ccc_ofp_033[0].area_number , &load_file.blk_dlr_ccc_033 , sizeof(BLK_DLR_CCC_033));
			// 使用预加载的任务点
			memcpy(&blk_ccc_ofp_034.point_number , &load_file.blk_dlr_ccc_034 , sizeof(BLK_DLR_CCC_034));
		}
		else
		{
			//反馈综显传输失败
			send_blk_ccc_dlr_000(3);
		}
	}

#else//半物理

	//接收任务分配结果
	if(dlr_load_state == 1)
	{
		// 传输中
		//		send_blk_ccc_dlr_000(4);
		//加载任务区
		memcpy(&blk_ccc_ofp_033[0].area_number , &load_file.blk_dlr_ccc_033.area_number , 2 + 8 * sizeof(Area_Info));
		//加载任务点
		memcpy(&blk_ccc_ofp_034.point_number , &load_file.blk_dlr_ccc_034 , sizeof(BLK_DLR_CCC_034));
		//发送设置高度到CTAS
		send_blk_dtms_ctas_010();
		//传输完毕
		send_blk_ccc_dlr_000(5);
		dlr_load_state = 0;
	}

#endif
}
//发送状态反馈
void send_blk_ccc_dlr_000(unsigned char states)
{
#if  _PC_SIMULATION_
	//赋值
	blk_ccc_dlr_000.load_status[18] = states;
#else//半物理
	//赋值
	blk_ccc_dlr_000.load_status = states;
	blk_ccc_dlr_000.load_progress = states;
#endif
	//返回状态反馈
	data_length = sizeof(BLK_CCC_DLR_000);
	Send_Message(DDSTables.CCC_DLR_0.niConnectionId,0,&transaction_id, &blk_ccc_dlr_000, &message_type_id, data_length , &enRetCode);
	if(enRetCode == 0)
	{
		printf("send blk_ccc_dlr_000 success\n");
	}
}

//发送加载文件数据
void send_load_file(unsigned int index)
{

	scheme_generation_state(1,1,0,0);// 返回方案编辑状态到综显
	// 发综显任务分配结果
	memcpy(&blk_ccc_ofp_019.plan_id,&load_file.blk_dlr_ccc_017[index],sizeof(BLK_DLR_CCC_017));
	send_blk_ccc_ofp_019();
	//保存方案
	unsigned int plan = blk_ccc_ofp_019.plan_id % 3;
	memcpy(&CCC_DPU_data_6[plan],&blk_ccc_ofp_019,sizeof(BLK_CCC_OFP_019));

	//解算保存
	for(int i = 0 ; i < 8 ; i ++)
	{
		//浮标布阵规划保存
		if(load_file.blk_dlr_cc_302[index][i].Plan_ID != 0)
		{
			memcpy(&blk_ccc_ofp_302_save[plan][i],&load_file.blk_dlr_cc_302[index][i],sizeof(BLK_CCC_OFP_302));
		}
		//吊声定测点规划保存
		if(load_file.blk_dlr_ccc_403[index][i].Plan_ID != 0)
		{
			memcpy(&blk_ccc_ofp_403_save[plan][i],&load_file.blk_dlr_ccc_403[index][i],sizeof(BLK_CCC_OFP_403));
		}
	}

	//发送浮标和吊声解算
	for(int i = 0 ; i < 8 ; i ++)
	{
		send_buoy_soanr_route_information(plan,i);//发送浮标、吊声信息到综显
	}

	//保存区域划分信息
	memcpy(&blk_ccc_ofp_005[plan], &load_file.blk_dlr_ccc_005[index] ,sizeof(BLK_CCC_OFP_005));
	data_length = sizeof(BLK_CCC_OFP_005);
	// 转发给综显任务区划分信息
	Send_Message(DDSTables.CCC_DPU_30.niConnectionId,0,&transaction_id, &load_file.blk_dlr_ccc_005[index], &message_type_id, data_length, &enRetCode);

	scheme_generation_state(1,2,0,0);// 返回方案编辑状态到综显

}

//发送航线生成信息
void send_blk_dtms_ctas_005(unsigned int plan)
{
	//组包航线生成信息
	BLK_DTMS_CTAS_005 blk_dtms_ctas_005;
	memcpy(&blk_dtms_ctas_005.blk_ccc_ofp_019,&CCC_DPU_data_6[plan],sizeof(BLK_CCC_OFP_019));
	memcpy(&blk_dtms_ctas_005.blk_ccc_ofp_038,&blk_ofp_ccc_038,sizeof(BLK_OFP_CCC_038));
	blk_dtms_ctas_005.solider_num = 1 + CCC_DPU_data_3.drone_number;   //todo：按有人机和无人机总数量赋值
	blk_dtms_ctas_005.solider_infos[0].solider_type = 1; // 类别  1 有人 2 无人
	blk_dtms_ctas_005.solider_infos[0].solider_id = MANNED_ID;//todo:目前有人机id写固定值：9001
	blk_dtms_ctas_005.solider_infos[0].lon_lat_info.latitude =DPU_CCC_data_4.manned_longitude_and_latitude_synt.latitude;
	blk_dtms_ctas_005.solider_infos[0].lon_lat_info.longitude = DPU_CCC_data_4.manned_longitude_and_latitude_synt.longitude;
	blk_dtms_ctas_005.solider_infos[0].speed = DPU_CCC_data_4.groundspeed;
	blk_dtms_ctas_005.solider_infos[0].height = DPU_CCC_data_4.absolute_barometric_altitude;
	blk_dtms_ctas_005.solider_infos[0].hangxiang = DPU_CCC_data_4.true_direction;

	// 因为无人机信息会出现非紧密存储，所以需要跳过无效无人机信息传递无人机信息
	int temUavNum = 0;
	for(int i = 0 ; i < 4 ; i ++ )
	{
		if(CCC_DPU_data_3.drone_specific_informations[i].platform_num == 0)
		{
			continue;
		}

		blk_dtms_ctas_005.solider_infos[temUavNum + 1].solider_type = 2;
		blk_dtms_ctas_005.solider_infos[temUavNum + 1].solider_id = CCC_DPU_data_3.drone_specific_informations[i].platform_num;
		blk_dtms_ctas_005.solider_infos[temUavNum + 1].speed = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.ground_speed;
		blk_dtms_ctas_005.solider_infos[temUavNum + 1].height = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_height;
		blk_dtms_ctas_005.solider_infos[temUavNum + 1].hangxiang = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_heading;
//		if(g_payload_replan[i].CT_Online == 1 && g_payload_replan[i].CT_Status == 1)
		blk_dtms_ctas_005.solider_infos[temUavNum + 1].ct = 1;
		blk_dtms_ctas_005.solider_infos[temUavNum + 1].lon_lat_info.latitude = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_lati;
		blk_dtms_ctas_005.solider_infos[temUavNum + 1].lon_lat_info.longitude = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_longi;
		temUavNum++;
	}

	//区域划分信息
	memcpy(&blk_dtms_ctas_005.blk_ccc_ofp_005,&blk_ccc_ofp_005[plan],sizeof(BLK_CCC_OFP_005));

	//发送航线生成信息到辅助决策
	data_length = sizeof(BLK_DTMS_CTAS_005);
	Send_Message(DDSTables.BLK_DTMS_CTAS_005.niConnectionId,0,&transaction_id, &blk_dtms_ctas_005 , &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0)
	{
		printf("send blk_dtms_ctas_005 success\n");
	}
}
void send_blk_dtms_ctas_010()
{
	data_length = sizeof(BLK_DTMS_CTAS_010) * 2;
	// 转发给综显任务区划分信息
	Send_Message(DDSTables.BLK_DTMS_CTAS_010.niConnectionId,0,&transaction_id, &load_file.blk_dtms_ctas_010, &message_type_id, data_length, &enRetCode);
}
//解算反馈
void recv_blk_ctas_dtms_009()
{
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.BLK_CTAS_DTMS_009.niConnectionId, 0, &transaction_id, &ctas_calc, &message_type_id, &message_size, &enRetCode);
}
void recv_blk_ctas_dtms_047()
{
	//接收安全区
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.BLK_CTAS_DTMS_008.niConnectionId, 0, &transaction_id, &blk_ccc_ofp_047, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		//保存安全区信息
		int plan = blk_ccc_ofp_047.program_number % 3;
		memcpy(&blk_ccc_ofp_047_save[plan],&blk_ccc_ofp_047,sizeof(BLK_CCC_OFP_047));
		//转发到综显
		send_blk_ccc_ofp_047();
	}

}

void send_blk_ccc_ofp_047()
{
	data_length = sizeof(BLK_CCC_OFP_047);
	Send_Message(DDSTables.CCC_DPU_41.niConnectionId,0,&transaction_id, &blk_ccc_ofp_047, &message_type_id, data_length, &enRetCode);
}

static int build_bdfx_double_air_area(unsigned int plan, int uav_index, double half_width_km, area_information *out_area)
{
	if (out_area == NULL || uav_index < 0 || uav_index >= 2)
	{
		return 0;
	}
	planning_informations *p_plan =
		&blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations;
	unsigned short point_num = p_plan->waypoints_number;
	if (point_num < 2)
	{
		return 0;
	}
	int start_index = 0;
	int end_index = point_num - 1;
	while (end_index > start_index)
	{
		double start_lat = p_plan->planning_information_waypoint_informations[start_index].latitude;
		double start_lon = p_plan->planning_information_waypoint_informations[start_index].longitude;
		double end_lat = p_plan->planning_information_waypoint_informations[end_index].latitude;
		double end_lon = p_plan->planning_information_waypoint_informations[end_index].longitude;
		if (fabs(start_lat - end_lat) > 1e-7 || fabs(start_lon - end_lon) > 1e-7)
		{
			break;
		}
		end_index--;
	}
	if (end_index <= start_index)
	{
		return 0;
	}
	double start_lat = p_plan->planning_information_waypoint_informations[start_index].latitude;
	double start_lon = p_plan->planning_information_waypoint_informations[start_index].longitude;
	double end_lat = p_plan->planning_information_waypoint_informations[end_index].latitude;
	double end_lon = p_plan->planning_information_waypoint_informations[end_index].longitude;
	double azimuth = calculate_azimuth(start_lat, start_lon, end_lat, end_lon);
	double left_azimuth = azimuth + (M_PI / 2.0);
	if (left_azimuth >= 2 * M_PI)
	{
		left_azimuth -= 2 * M_PI;
	}
	double right_azimuth = azimuth - (M_PI / 2.0);
	if (right_azimuth < 0)
	{
		right_azimuth += 2 * M_PI;
	}
	double s_left_lat = 0.0;
	double s_left_lon = 0.0;
	double e_left_lat = 0.0;
	double e_left_lon = 0.0;
	double e_right_lat = 0.0;
	double e_right_lon = 0.0;
	double s_right_lat = 0.0;
	double s_right_lon = 0.0;
	calculate_endpoint(start_lat, start_lon, left_azimuth, half_width_km, &s_left_lat, &s_left_lon);
	calculate_endpoint(end_lat, end_lon, left_azimuth, half_width_km, &e_left_lat, &e_left_lon);
	calculate_endpoint(end_lat, end_lon, right_azimuth, half_width_km, &e_right_lat, &e_right_lon);
	calculate_endpoint(start_lat, start_lon, right_azimuth, half_width_km, &s_right_lat, &s_right_lon);
	memset(out_area, 0, sizeof(area_information));
	out_area->area_code = (unsigned int)(uav_index + 9);
	out_area->area_type = 2;
	out_area->area_source = 3;
	out_area->area_shape = 2;
	out_area->area_platform_num = 1;
	out_area->drone_numbe = (unsigned int)(uav_index + 1);
	out_area->upper_height_limit_valid_bit = 1;
	out_area->lower_height_limit_valid_bit = 1;
	out_area->upper_height_limit = (p_plan->mission_height > 0) ? (float)p_plan->mission_height : 1200.0f;
	out_area->lower_height_limit = 1.0f;
	out_area->polygonals.point_number = 4;
	out_area->polygonals.point_coordinates[0].latitude = s_left_lat;
	out_area->polygonals.point_coordinates[0].longitude = s_left_lon;
	out_area->polygonals.point_coordinates[1].latitude = e_left_lat;
	out_area->polygonals.point_coordinates[1].longitude = e_left_lon;
	out_area->polygonals.point_coordinates[2].latitude = e_right_lat;
	out_area->polygonals.point_coordinates[2].longitude = e_right_lon;
	out_area->polygonals.point_coordinates[3].latitude = s_right_lat;
	out_area->polygonals.point_coordinates[3].longitude = s_right_lon;
	return 1;
}

static int build_bdfx_single_air_area(unsigned int plan, int uav_index, double half_width_km, area_information *out_area)
{
	if (out_area == NULL || uav_index < 0 || uav_index >= UAV_MAX_NUM)
	{
		return 0;
	}
	planning_informations *p_plan =
		&blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations;
	unsigned short point_num = p_plan->waypoints_number;
	if (point_num < 2)
	{
		return 0;
	}
	int start_index = 0;
	int end_index = point_num - 1;
	while (end_index > start_index)
	{
		double start_lat = p_plan->planning_information_waypoint_informations[start_index].latitude;
		double start_lon = p_plan->planning_information_waypoint_informations[start_index].longitude;
		double end_lat = p_plan->planning_information_waypoint_informations[end_index].latitude;
		double end_lon = p_plan->planning_information_waypoint_informations[end_index].longitude;
		if (fabs(start_lat - end_lat) > 1e-7 || fabs(start_lon - end_lon) > 1e-7)
		{
			break;
		}
		end_index--;
	}
	if (end_index <= start_index)
	{
		return 0;
	}
	double start_lat = p_plan->planning_information_waypoint_informations[start_index].latitude;
	double start_lon = p_plan->planning_information_waypoint_informations[start_index].longitude;
	double end_lat = p_plan->planning_information_waypoint_informations[end_index].latitude;
	double end_lon = p_plan->planning_information_waypoint_informations[end_index].longitude;
	double azimuth = calculate_azimuth(start_lat, start_lon, end_lat, end_lon);
	double left_azimuth = azimuth + (M_PI / 2.0);
	if (left_azimuth >= 2 * M_PI)
	{
		left_azimuth -= 2 * M_PI;
	}
	double right_azimuth = azimuth - (M_PI / 2.0);
	if (right_azimuth < 0)
	{
		right_azimuth += 2 * M_PI;
	}
	double s_left_lat = 0.0;
	double s_left_lon = 0.0;
	double e_left_lat = 0.0;
	double e_left_lon = 0.0;
	double e_right_lat = 0.0;
	double e_right_lon = 0.0;
	double s_right_lat = 0.0;
	double s_right_lon = 0.0;
	calculate_endpoint(start_lat, start_lon, left_azimuth, half_width_km, &s_left_lat, &s_left_lon);
	calculate_endpoint(end_lat, end_lon, left_azimuth, half_width_km, &e_left_lat, &e_left_lon);
	calculate_endpoint(end_lat, end_lon, right_azimuth, half_width_km, &e_right_lat, &e_right_lon);
	calculate_endpoint(start_lat, start_lon, right_azimuth, half_width_km, &s_right_lat, &s_right_lon);
	memset(out_area, 0, sizeof(area_information));
	out_area->area_code = (unsigned int)(uav_index + 9);
	out_area->area_type = 2;
	out_area->area_source = 3;
	out_area->area_shape = 2;
	out_area->area_platform_num = 1;
	out_area->drone_numbe = (unsigned int)(uav_index + 1);
	out_area->upper_height_limit_valid_bit = 1;
	out_area->lower_height_limit_valid_bit = 1;
	out_area->upper_height_limit = (p_plan->mission_height > 0) ? (float)p_plan->mission_height : 1200.0f;
	out_area->lower_height_limit = 1.0f;
	out_area->polygonals.point_number = 4;
	out_area->polygonals.point_coordinates[0].latitude = s_left_lat;
	out_area->polygonals.point_coordinates[0].longitude = s_left_lon;
	out_area->polygonals.point_coordinates[1].latitude = e_left_lat;
	out_area->polygonals.point_coordinates[1].longitude = e_left_lon;
	out_area->polygonals.point_coordinates[2].latitude = e_right_lat;
	out_area->polygonals.point_coordinates[2].longitude = e_right_lon;
	out_area->polygonals.point_coordinates[3].latitude = s_right_lat;
	out_area->polygonals.point_coordinates[3].longitude = s_right_lon;
	return 1;
}
/**
 * //接收空域信息
 * 目前空域与ofp约定，033第二包的前4个作为无人机的4个相应空域，编号作为有效性（支持非紧密排列）
 */
void recv_blk_ctas_dtms_010()
{
	//接收空域信息
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.BLK_CTAS_DTMS_010.niConnectionId, 0, &transaction_id, &blk_ctas_dtms_010, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		//存入空域信息
		for(int i = 0 ; i < 5 ; i ++)
		{
			//存入有效空域
			if(blk_ctas_dtms_010.id[i] > 0)
			{
				//有人机空域
				if(blk_ctas_dtms_010.id[i] == MANNED_ID)
				{
					air_area.area_informations[0].area_code = 8;//编号
					air_area.area_informations[0].area_type = 2;//类型，空域
					air_area.area_informations[0].area_source = 3;//决策生成
					air_area.area_informations[0].area_shape = 2;//多边形
					air_area.area_informations[0].area_platform_num = 2;//归属平台
					air_area.area_informations[0].drone_numbe = blk_ctas_dtms_010.id[i];//所属平台序号
					air_area.area_informations[0].polygonals.point_number = 4;//多边形点数
					for(int j = 0 ; j < 4 ; j ++)
					{
						air_area.area_informations[0].polygonals.point_coordinates[j].longitude = blk_ctas_dtms_010.airway_area[i].vertexA[j].longitude;
						air_area.area_informations[0].polygonals.point_coordinates[j].latitude = blk_ctas_dtms_010.airway_area[i].vertexA[j].latitude;
					}
					continue;
				}
				else
				{
					// 当存在非紧密编队情况(生成的是在blk_ctas_dtms_010.id下标1，其实是第二架机的，所以更新到第二架机的空域即可)
					if(castCtasToOfpIsNeeded())
					{
						blk_ctas_dtms_010.id[i] = 2;
					}

					//平台序号
					int index = blk_ctas_dtms_010.id[i];
					//无人机空域
					air_area.area_informations[index].area_code = index+8;//编号
					air_area.area_informations[index].area_type = 2;//类型，空域
					air_area.area_informations[index].area_source = 3;//决策生成
					air_area.area_informations[index].area_shape = 2;//多边形
					air_area.area_informations[index].area_platform_num = 1;//归属平台
					air_area.area_informations[index].drone_numbe = blk_ctas_dtms_010.id[i];//所属平台序号
					air_area.area_informations[index].polygonals.point_number = 4;//多边形点数
					for(int j = 0 ; j < 4 ; j ++)
					{
						air_area.area_informations[index].polygonals.point_coordinates[j].longitude = blk_ctas_dtms_010.airway_area[i].vertexA[j].longitude;
						air_area.area_informations[index].polygonals.point_coordinates[j].latitude = blk_ctas_dtms_010.airway_area[i].vertexA[j].latitude;
					}
				}
			}
		}

	}
}
void init_blk_ctas_dtms_010()
{
	//空域生成标志
	static int area_flag[5] = {0,0,0,0,0};
	//无人机空域应用
	for(int i = 0 ;i < 4 ; i ++)
	{
		if(air_area.area_informations[i+1].area_code != 0 && send_area[i] == 1)
		{
			send_area[i] = 0;
			memcpy(&blk_ccc_ofp_033[1].area_informations[i],&air_area.area_informations[i+1],sizeof(area_information));
			//加入无人机空域
			if(area_flag[i+1] == 0)
			{
				blk_ccc_ofp_033[1].area_number++;
				area_flag[i+1] = 1;
			}
		}
	}
	//有人机空域应用
	if(air_area.area_informations[0].area_code != 0)
	{
		memcpy(&blk_ccc_ofp_033[0].area_informations[7],&air_area.area_informations[0],sizeof(area_information));
		//加入有人机空域
		if(area_flag[0] == 0)
		{
			blk_ccc_ofp_033[0].area_number++;
			area_flag[0] = 1;
		}
	}

}
//从dlr文件中取出当前阶段航线
void dlr_cpy_airway(unsigned int stage,unsigned int plan)
{
	//找到方案对应位置A?B
	unsigned int index = 0;
	if(load_file.blk_dlr_ccc_017[0].program_number == plan)
	{
		//预规划方案A
		index = 0;
	}
	else if(load_file.blk_dlr_ccc_017[1].program_number == plan)
	{
		//预规划方案B
		index = 1;
	}
	unsigned int plan_index = plan % 3;
	//取出有人机航线
	memcpy(&blk_ccc_ofp_018_cunchu[plan_index][stage],&load_file.blk_dlr_ccc_018[index][stage],sizeof(BLK_DLR_CCC_018_cunchu));
	//取出无人机航线
	for(int i = 0 ; i < 2 ; i ++)
	{
		memcpy(&blk_ccc_ofp_024_cunchu[plan_index][i],&load_file.blk_dlr_ccc_024[index][i][stage],sizeof(BLK_CCC_OFP_024_cunchu));
	}

}
//分阶段发送航线
void stage_send()
{
	// DPU_DTMS 航线生成命令
	message_size = RECV_MAX_SIZE;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_28.niConnectionId,DDSTables.DPU2_CCC_28.niConnectionId,&blk_ofp_ccc_038,sizeof blk_ofp_ccc_038);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		Receive_Message(DDSTables.PAD_CCC_038.niConnectionId, 0, &transaction_id, &blk_ofp_ccc_038, &message_type_id, &message_size, &enRetCode);
	}
	if (enRetCode == 0)
	{

		scheme_generation_state(1,2,0,1);// 返回方案编辑状态到综显，航线生成中

		//如果未加载返航航线，则生成失败
		int normal_cnt = 0;
		for(int i = 0 ; i < 4 ; i ++)
		{
			//无人机有返航航线且在线
			if(load_file.blk_dlr_ccc_045[i].normal_num > 0 && CCC_DPU_data_3.drone_specific_informations[i].platform_num == load_file.blk_dlr_ccc_045[i].uav_id)
			{
				normal_cnt++;
			}
		}
		if(CCC_DPU_data_3.drone_number > normal_cnt)
		{
			sprintf(CCC_DPU_data_0.failreason,"未检测到有效无人机返航航线");
			scheme_generation_state(2,2,0,3);// 生成失败
			memset(&CCC_DPU_data_0.failreason,0,sizeof CCC_DPU_data_0.failreason);
			return;
		}

		unsigned int plan = blk_ofp_ccc_038.Plan_ID % 3;
		if(blk_ofp_ccc_038.Plan_ID == 0 || blk_ofp_ccc_038.Plan_ID < 0)
		{
			return;
		}
		//收到预规划A/B方案
		if(blk_ofp_ccc_038.Plan_ID <= 3)
		{
			//从缓存中取出航线到综显
			unsigned int task = blk_ofp_ccc_038.stage_id - 1;
			dlr_cpy_airway(task,blk_ofp_ccc_038.Plan_ID);
			//发送航线
			send_blk_ccc_ofp_018(DDSTables.CCC_DPU_7.niConnectionId,plan,task); // 发送有人机通航点 通用航路点分两次发送 第1包40个航点 和 第2包35个航点
			//			send_buoy_soanr_route_information(plan,task);//发送浮标、吊声信息到综显
			send_blk_ccc_ofp_024(plan); // 发送无人机信息，航线生成完成

			scheme_generation_state(1,2,0,2);// 返回方案编辑状态到综显

			//发送无人机航线到辅助决策生成空域
			for(int i = 0 ; i < 4 ;i ++)
			{
				send_uav_airway(plan,i);
			}
			return;
		}

		//发送航线生成信息给辅助决策
		send_blk_dtms_ctas_005(plan);
	}

	// payload replan 预留（原逻辑保留）
	//\tif(g_lineCrashState[0].PayloadReplan == 2)
	//\t{
	//\t\t//判断输入合法性
	//\t\tunsigned int plan = blk_ofp_ccc_038.Plan_ID % 3;
	//\t\tif( blk_ofp_ccc_038.Plan_ID > 100 || blk_ofp_ccc_038.Plan_ID < 0)
	//\t\t{
	//\t\t\treturn;
	//\t\t}
	//\t\t//组包航线生成信息
	//\t\tBLK_DTMS_CTAS_005 blk_dtms_ctas_005;
	//\t\tmemcpy(&blk_dtms_ctas_005.blk_ccc_ofp_019,&CCC_DPU_data_6[plan],sizeof(BLK_CCC_OFP_019));
	//\t\tmemcpy(&blk_dtms_ctas_005.blk_ccc_ofp_038,&blk_ofp_ccc_038,sizeof(BLK_OFP_CCC_038));
	//\t\t//发送航线生成信息到辅助决策
	//\t\tdata_length = sizeof(BLK_DTMS_CTAS_005);
	//\t\tSend_Message(DDSTables.BLK_DTMS_CTAS_005.niConnectionId,0,&transaction_id, &blk_dtms_ctas_005 , &message_type_id, data_length, &enRetCode);
	//\t\tif(enRetCode == 0)
	//\t\t{
	//\t\t\tprintf("send blk_dtms_ctas_005 success\n");
	//\t\t}
	//\t\tg_lineCrashState[0].PayloadReplan = 2;
	//\t}
	// CTAS_DTMS 解算完成
	if(ctas_calc == 10086)
	{
		ctas_calc = 0;
		unsigned int plan = blk_ofp_ccc_038.Plan_ID % 3;
		unsigned int task = blk_ofp_ccc_038.stage_id - 1;
		//保存有人机航路信息
		for(int i = 0 ; i < 2 ; i ++)
		{
			receive_zhanfa_hl(); //完成对航路信息的文件保存
		}

		// 接收无人机航路规划信息
		for(int i = 0 ; i <  8; i ++)
		{
			receive_zhanfa_uav_hl();           // 接收无人机航路规划信息
		}
		// 备份无人机航路点
		for(int drone_index = 0 ;  drone_index < 4 ; drone_index++)
		{
			memcpy(&g_lineCrashUavBak[drone_index], &blk_ccc_ofp_024_cunchu[plan][drone_index], sizeof(g_lineCrashUavBak[drone_index]));
		}

		send_blk_ccc_ofp_018(DDSTables.CCC_DPU_7.niConnectionId,plan,task); // 发送有人机通航点 通用航路点分两次发送 第1包40个航点 和 第2包35个航点

		//生成时需要做冲突检测
		avoidLineCrashJudgeProc();

		//恢复航线生成
		//一控二场景
		int rtn = 0;
		for(int drone_index = 0 ; drone_index < UAV_MAX_NUM; drone_index ++)
		{

			if (g_lineCrashState[drone_index].hasConflict == 1)
			{
				// 恢复航线增加
				rtn = avoidLineCrashSecondLineProc(drone_index);
				if(rtn == -1)
				{
					//有人机航线在无人机任务区内反馈错误
					return;
				}
			}
		}

		// 没冲突，正常发送航线即可
		send_blk_ccc_ofp_024(plan); // 发送无人机信息，航线生成完成

		//载荷重规划检测
		payload_detection();
		scheme_generation_state(1,2, 0, 2); // 返回方案编辑状态到调用方
	}

	// DPU_DTMS 浮标—航路点解算
	message_size = RECV_MAX_SIZE;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_32.niConnectionId,DDSTables.DPU2_CCC_32.niConnectionId,&blk_ofp_ccc_302,sizeof blk_ofp_ccc_302);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//		Receive_Message(DDSTables.PAD_CCC_302.niConnectionId, 0, &transaction_id, &blk_ofp_ccc_302, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode == 0)
	{
		//转发浮标—航路点解算到辅助决策
		data_length = sizeof(BLK_OFP_CCC_302);
		Send_Message(DDSTables.BLK_DTMS_CTAS_006.niConnectionId,0,&transaction_id, &blk_ofp_ccc_302 , &message_type_id, data_length, &enRetCode);
		if(enRetCode == 0)
		{
			printf("send blk_ofp_ccc_302 success\n");
		}
	}

	// DPU_DTMS 吊声—吊声定测点规划
	message_size = RECV_MAX_SIZE;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_33.niConnectionId,DDSTables.DPU2_CCC_33.niConnectionId,&blk_ofp_ccc_402,sizeof blk_ofp_ccc_402);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//		Receive_Message(DDSTables.PAD_CCC_402.niConnectionId, 0, &transaction_id, &blk_ofp_ccc_402, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode == 0)
	{
		//转发吊声—吊声定测点规划到辅助决策
		data_length = sizeof(BLK_OFP_CCC_402);
		Send_Message(DDSTables.BLK_DTMS_CTAS_007.niConnectionId,0,&transaction_id, &blk_ofp_ccc_402 , &message_type_id, data_length, &enRetCode);
		if(enRetCode == 0)
		{
			printf("send blk_ofp_ccc_402 success\n");
		}
	}
}
//******************************************** 接收辅助决策返回的战法方案 ***********************************************/
void rev_zhanfa_plan()
{

	// CATS_DTMS 战术战法推荐规划结果
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.BLK_CTAS_DTMS_001.niConnectionId, 0, &transaction_id, &blk_ccc_ofp_019, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{

		//保存三方案每个方案的任务分配结果
		unsigned int plan = blk_ccc_ofp_019.plan_id % 3;
		memcpy(&CCC_DPU_data_6[plan],&blk_ccc_ofp_019,sizeof(BLK_CCC_OFP_019));
		// 发综显任务分配结果
		// 在非紧密无人机编队下，需要特殊处理发给ofp的信息
		send_blk_ccc_ofp_019_special();
		zhanfa_result_receive_flag = 1;
		//是否是重规划的任务结果
		if(blk_ccc_ofp_019.plan_release_mode == 3)//0204
		{
			//编辑状态发送,成功
			send_blk_ccc_ofp_020(DPU_CCC_data_5.program_number,0,2,NULL);
			zhanfa_result_receive_flag = 0;
		}
	}
	//收到推荐结果保存或转发航路相关信息
	if(zhanfa_result_receive_flag == 1)
	{
		// CATS_DTMS 任务区划分信息
		BLK_CCC_OFP_005 temp;
		message_size = RECV_MAX_SIZE;
		Receive_Message(DDSTables.BLK_CTAS_DTMS_007.niConnectionId, 0, &transaction_id, &temp, &message_type_id, &message_size, &enRetCode);
		if(enRetCode == 0)
		{
			//保存区域划分信息
			int plan = temp.Plan_ID % 3;
			memcpy(&blk_ccc_ofp_005[plan], &temp ,sizeof(BLK_CCC_OFP_005));
			data_length = sizeof(BLK_CCC_OFP_005);
			// 转发给综显任务区划分信息
			Send_Message(DDSTables.CCC_DPU_30.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);

			//发送给pad new20250620
			if(Pad_heart_flag == 1)
			{
				Send_Message(DDSTables.CCC_PAD_005.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);
			}

		}
		for(int i = 0 ; i < 8 ; i ++)
		{
			//接收浮标、吊声规划信息
			receive_buoy_soanr();
			unsigned int plan = blk_ccc_ofp_019.plan_id % 3;
			send_buoy_soanr_route_information(plan,i);//发送浮标、吊声信息到调用方
		}

		zhanfa_result_receive_flag = 0;
		plan_count++;
	}

	if(plan_count == 3)
	{
		scheme_generation_state(1,2,0,0);// 返回方案编辑状态到调用方
		init_blk_ccc_ofp_036(2,1,plan_task_type);
		plan_count = 0;
	}
}

// 接收战法规划方案结果的浮标点和吊声点信息
void receive_buoy_soanr()
{
	// CATS_DTMS 浮标布阵规划
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.BLK_CTAS_DTMS_005.niConnectionId, 0, &transaction_id, &blk_ccc_ofp_302, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		unsigned int plan = blk_ccc_ofp_302.Plan_ID % 3;
		unsigned int task = blk_ccc_ofp_302.SubTask_Id - 1;
		//浮标布阵规划保存
		memcpy(&blk_ccc_ofp_302_save[plan][task],&blk_ccc_ofp_302,sizeof(BLK_CCC_OFP_302));
	}

	// CATS_DTMS 吊声定测点规划
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.BLK_CTAS_DTMS_006.niConnectionId, 0, &transaction_id, &blk_ccc_ofp_403, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		unsigned int plan = blk_ccc_ofp_403.Plan_ID % 3;
		unsigned int task = blk_ccc_ofp_403.SubTask_Id - 1;
		//吊声定测点规划保存
		memcpy(&blk_ccc_ofp_403_save[plan][task],&blk_ccc_ofp_403,sizeof(BLK_CCC_OFP_403));
	}
}

/**************************************** 战法有人机航线 ****************************************/
void receive_zhanfa_hl()
{
	BLK_CCC_OFP_018  temp;
	//收前航路点
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.BLK_CTAS_DTMS_003.niConnectionId, 0, &transaction_id, &temp, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		unsigned int plan = (temp.plan_code) % 3;
		unsigned int task = temp.subtask_index;
		// 加入航路点
		if (temp.airway_point_start_num == 0)
		{ // 第一包40点
			//取基本数据
			memcpy(&blk_ccc_ofp_018_cunchu[plan][task],&temp,21);
			// 将前四十个点保存
			memcpy( &(blk_ccc_ofp_018_cunchu[plan][task].waypoint_informations[0]) , &temp.waypoint_informations[0] ,40 * sizeof(waypoint_information));
		}
		else if(temp.airway_point_start_num == 40)
		{
			//取超出四十的点数量
			temp.airway_point_num -= 40;
			//取基本数据
			memcpy(&blk_ccc_ofp_018_cunchu[plan][task],&temp,21);
			// 将后四十个点保存
			memcpy( &(blk_ccc_ofp_018_cunchu[plan][task].waypoint_informations[40]) , &temp.waypoint_informations[0] ,temp.airway_point_num * sizeof(waypoint_information));
		}
	}
}

/*************************************** 战法无人机航线 *******************************************/

void receive_zhanfa_uav_hl()
{
	BLK_CCC_OFP_024 temp;
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.BLK_CTAS_DTMS_004.niConnectionId, 0, &transaction_id, &temp, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		castCtasToOfpLine(&temp);
		unsigned int plan = (temp.program_number) % 3;
		unsigned int id = (temp.individual_drone_routing_programs.drone_serial_number - 1);
		unsigned int index = temp.individual_drone_routing_programs.planning_informations.packet_id *25;
		// 保存信息
		memcpy(&blk_ccc_ofp_024_cunchu[plan][id],&temp,9 + 10 + 26);
		// 保存航路点信息
		memcpy(&blk_ccc_ofp_024_cunchu[plan][id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index], &temp.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], sizeof(planning_information_waypoint_information) * 25);
	}
}


/*******************  修改借口 ********************/
//  3.1 初始化任务状态信息，为任务状态信息赋值
void init_task_status_information()
{
	CCC_DPU_data_2.current_state = 2;
	CCC_DPU_data_2.current_task_ID_of_the_panel = 1;
	CCC_DPU_data_2.current_task_progress = 3;

}
//任务状态信息发送
void send_task_status_information()
{
	// drone_information *drone_informations = new drone_information;
	init_task_status_information();//初始化，赋值
	align_send_information(&CCC_DPU_data_2,sizeof(CCC_DPU_data_2),0);//对齐
	Send_Message(DDSTables.CCC_DPU_2.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	// // // printf("长度1:",data_length;
	// // // printf("code_1",enRetCode;
	// // send_array.clear();
	//发送给pad new20250620
	//	Send_Message(DDSTables.CCC_PAD_012.niConnectionId,0,&transaction_id, send_array.dataA+2,&message_type_id, data_length, &enRetCode);
}


// 3.2 初始化无人机状态信息  uav黑包获取
void init_drone_state_information(int i)
{
	// 实际
	static int cnt = 0;
	unsigned int current_uav_height=0;
	double current_uav_angle=0.0;
	CCC_DPU_data_3.drone_specific_informations[i].platform_model = 1;// 平台型号
	CCC_DPU_data_3.drone_specific_informations[i].platform_serial_num = i+1;//平台序号
#if !_PC_SIMULATION_ //半物理环境，编号处理
	//暂时写死 20250729
	//	CCC_DPU_data_3.drone_specific_informations[i].platform_num = UAV1_ID;
	//CCC_DPU_data_3.drone_specific_informations[i].platform_num = 0x1000+(unsigned int)((s82_frame.uavCode & 0xf0)>>4);
	CCC_DPU_data_3.drone_specific_informations[i].platform_num = formationId[i].planeId;
#endif

	//s4A帧赋值逻辑
	if(s4A_frame.frame_type != 0)
	{
#if _PC_SIMULATION_  //仿真环境，编号处理
		CCC_DPU_data_3.drone_specific_informations[i].platform_num  = s4A_frame.aircraft_address;//平台编号 1001-1999
#endif
		//CCC_DPU_data_3.drone_specific_informations[i].platform_num = plantform_id;
		// 根据主机要求　有人机地址编号从9001开始
#if _PC_SIMULATION_
		if(s4A_frame.station_address >= 0x3001)
#else
			if(s4A_frame.station_address == MANNED_ID)
#endif
			{
				CCC_DPU_data_3.drone_specific_informations[i].platform_control_status = 1;//平台控制权状态   1 有人机控制 2 见面站控制
				formationId[i].isControl = 1;
				//20250804new 记录有效控制站地址
				formationId[i].station_address = s4A_frame.station_address;
			}
			else if(s4A_frame.station_address == 0x3001 || s4A_frame.station_address == 0x2001)
			{
				// 根据主机要求　有人机地址编号从9001开始
				CCC_DPU_data_3.drone_specific_informations[i].platform_control_status = 2;//平台控制权状态   1 有人机控制 2 见面站控制
				formationId[i].isControl = 0;
				//20250804new 记录有效控制站地址
				formationId[i].station_address = s4A_frame.station_address;
			}

		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_height = (float)(s4A_frame.barom_height*0.2);//气压高度
	}

	//载荷在线信息再任务遥测中判断
	//	CCC_DPU_data_3.drone_specific_informations[i].platform_load = 0xffff;   //todo:update the fix data 0xffff
	//	CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit = 0xffff;//数据有效位

	CCC_DPU_data_3.drone_specific_informations[i].fault_level = 2;//平台故障状态

	//任务状态 20250702new
	CCC_DPU_data_3.drone_specific_informations[i].subtask_type = uav_route[i].task_type;//平台当前执行子任务类型  (todo: 需要根据接收飞仿的数据和协同指控生成的航线进行计算)

	//计算任务剩余时间
	if(uav_route[i].tasking)
	{
		int distance = 0;
		//初始点计算，减去第一个机场点，再减去1找到点下标
		int first_point = uav_route[i].waypoint_number -2;
		//航路点总数有首尾两个点和倒数第二个计算点是固定点需要减去，再减去1得到路段总数
		for(int k = first_point ; k < uav_route[i].hull_number -4 ; k++)
		{
			//加上当前无人机位置到当前航路点的距离
			if(k == uav_route[i].waypoint_number -2)
			{
				double lat = 0;
				double lon = 0;
				lat = ((double)s82_frame.lat) * 90.0 / (pow(2,31) - 1);
				lon = ((double)s82_frame.lon) * 180.0 / (pow(2,31) - 1);
				distance += calculate_distances(lat,lon,uav_route[i].waypoint[k].latitude,uav_route[i].waypoint[k].longitude);
			}



			distance += calculate_distances(uav_route[i].waypoint[k].latitude,
					uav_route[i].waypoint[k].longitude,
					uav_route[i].waypoint[k+1].latitude,
					uav_route[i].waypoint[k+1].longitude);
		}
		//最后一段航段距离
		if(uav_route[i].hull_number -4 == first_point)
		{
			double lat = 0;
			double lon = 0;
			lat = ((double)s82_frame.lat) * 90.0 / (pow(2,31) - 1);
			lon = ((double)s82_frame.lon) * 180.0 / (pow(2,31) - 1);
			distance = calculate_distances(lat,lon,uav_route[i].waypoint[0].latitude,uav_route[i].waypoint[0].longitude);
		}

		//剩余距离除地速
		float groud_speed = 0;
		int time = 0;
		groud_speed = (float)((sqrt((s81_frame.body_grouspeed_x)*(s81_frame.body_grouspeed_x) + (s81_frame.body_grouspeed_y)*(s81_frame.body_grouspeed_y))) * 0.1);
		time = distance * 1000 / groud_speed;
		if(groud_speed < 0.0001 && groud_speed > -0.0001)
		{
			time = 0;
		}
		CCC_DPU_data_3.drone_specific_informations[i].remaining_mission_time = time;//平台剩余任务时间  (todo: 需要根据接收飞仿的数据和协同指控生成的航线进行计算)
		//剩余时间有效位
		CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit = setBit(CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit,0,1);
	}
	else
	{
		//剩余时间有效位，无效，没有任务或者任务已完成
		CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit = setBit(CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit,0,0);
	}



	//11.13补充采集的数据需要核对格式转换关系
	if (s4c_flag == 1)
	{
		CCC_DPU_data_3.drone_specific_informations[i].residual_oil_volume = (unsigned int)(s4C_frame.oil_quantity * 0.5);//平台剩余油量
		CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit = setBit(CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit,1,1);
		CCC_DPU_data_3.drone_specific_informations[i].Ng = (double)(s4C_frame.ng*0.01);//ng
		CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit = setBit(CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit,2,1);
		CCC_DPU_data_3.drone_specific_informations[i].Nr = (double)(s4C_frame.rotor_speed*0.1);//nr
		CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit = setBit(CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit,3,1);
		CCC_DPU_data_3.drone_specific_informations[i].T45 = (double)(s4C_frame.t45*0.1);//T45
		CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit = setBit(CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit,4,1);
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.radio_height = (float)(s4C_frame.radio_height*0.1);//无线电高度* (height_scale) )* 0.2;
	}

	if(s3a_flag == 1)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.sat_height = (float)(s3A_frame.satellite_altitude*0.2);//卫星高度* (height_scale) )* 0.2
	}
	if (s5D_frame.frame_type != 0)
	{
		CCC_DPU_data_3.drone_specific_informations[i].U = (double)(s5D_frame.battery_1_voltage);//U
		CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit = setBit(CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit,5,1);
		CCC_DPU_data_3.drone_specific_informations[i].U_storage = (double)(s5D_frame.battery_2_current);//U蓄
		CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit = setBit(CCC_DPU_data_3.drone_specific_informations[i].data_valid_bit,6,1);
	}

	CCC_DPU_data_3.drone_specific_informations[i].uav_zishu_feedback = 0;

	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.station_lati = (double)(plane_lat + 0.3 * (i+1));//TODO:plane_lat为有人机经纬度，地面站经纬度待定				/*地面站纬度    单位：°  最小值：-90  最大值：90*/
	if(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.station_lati >= -80 && CCC_DPU_data_3.drone_specific_informations[i].uav_infos.station_lati <= 80)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = setBit(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,0,1);
	}
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.station_longi = (double)(plane_lon + 0.3 * (i+1));				/*地面站经度    单位：°  最小值：-180  最大值：180*/
	// CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_controlled_by;   /*无人机控制权归属  0=NA;1=地面测控站;2=有人机;*/
	if(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.station_longi >= -180 && CCC_DPU_data_3.drone_specific_informations[i].uav_infos.station_longi <= 180)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = setBit(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,1,1);
	}

	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_lati = ((double)s82_frame.lat) * 90.0 / (pow(2,31) - 1);					/*无人机纬度    单位：°  最小值：-90  最大值：90*/
	if(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_lati >= -90 && CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_lati <= 90)
	{
#if VALID_LAT_LON
		//屏蔽实验室经纬度跳0
		if(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_lati > 10)
#endif
			formationId[i].lat = ((double)s82_frame.lat) * 90.0 / (pow(2,31) - 1);
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = setBit(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,2,1);
	}
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_longi = ((double)s82_frame.lon) * 180.0 / (pow(2,31) - 1);					/*无人机经度    单位：°  最小值：-180  最大值：180*/
	if(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_longi >= -180 && CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_longi <= 180)
	{
		//屏蔽实验室经纬度跳0
#if VALID_LAT_LON
		if(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_longi > 10)
#endif
			formationId[i].lon = ((double)s82_frame.lon) * 180.0 / (pow(2,31) - 1);
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = setBit(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,3,1);
	}



	current_uav_height=CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_height;
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = setBit(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,4,1);
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_speed = (float)(s81_frame.vacuum_velocity * 0.1 * 3.6);	/*无人机空速度*/
	if(s81_frame.vacuum_velocity * 0.1 >= -300 && s81_frame.vacuum_velocity * 0.1 <= 300)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = setBit(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,5,1);
	}
	// printf("uav air speed [%d] s81_frame.vacuum_velocity [%f]\n",CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_speed,s81_frame.vacuum_velocity * 0.1*3.6);
	/*无人机地速度*/
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.ground_speed = (float)((sqrt((s81_frame.body_grouspeed_x)*(s81_frame.body_grouspeed_x) + (s81_frame.body_grouspeed_y)*(s81_frame.body_grouspeed_y))) * 0.1 * 3.6);
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.vectx_speed = (float)s81_frame.vertical_velocity * 0.1;
	//      CCC_DPU_data_3.drone_specific_informations[i].uav_infos.vectx_speed = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.vectx_speed*0.1;	/*无人机垂直速度*/
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_heading = (float)(s81_frame.course * 0.01);	/*无人机航向*/
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = setBit(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,6,1);


	//2024.11.13原有的angle_scale精度存在问题，改为ICD中设置的0.01
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_pitch = (float)(s81_frame.pit * 0.01);	/*无人机俯仰角*/
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_roll = (float)(s81_frame.roll * 0.01);	/*无人机横滚角*/
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = setBit(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,7,1);
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = setBit(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,8,1);
	/*威胁信息*/

	//CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_height_trend = 0;		//记录无人机上一针高度air_height，作比较进行判断		/*无人机高度变化趋势  0=无效;1=向上;2=向下;*/
	//通过无人机垂速判断无人机高度变化趋势
	cnt++;
	if((cnt % 5)==0)//5帧比较一次
	{
		int uav_height_change = current_uav_height - histroy_uav_data.his_uav_height;  //不使用 (之前使用历史高度判断高度变化趋势)
		//pre_uav_height=CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_height;
		if (CCC_DPU_data_3.drone_specific_informations[i].uav_infos.vectx_speed >= 1)
		{
			CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_height_trend=1;
		}
		else if((CCC_DPU_data_3.drone_specific_informations[i].uav_infos.vectx_speed >= -1) && (CCC_DPU_data_3.drone_specific_informations[i].uav_infos.vectx_speed < 1))
		{
			CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_height_trend=0;
		}
		else
		{
			CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_height_trend=2;
		}
		histroy_uav_data.his_uav_height = current_uav_height;
	}


	//todo: 验证api输出距离和方位角的正确性
	GeoLibDas uav_geolibdas={};
	GeoLibDas manned_aircraft_geolibdas={};
	uav_geolibdas.latitude=CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_lati;
	uav_geolibdas.longitude=CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_longi;
	manned_aircraft_geolibdas.latitude=DPU_CCC_data_4.manned_longitude_and_latitude_synt.latitude;
	manned_aircraft_geolibdas.longitude=DPU_CCC_data_4.manned_longitude_and_latitude_synt.longitude;
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_copter_distance = (unsigned int)(getDistanceGeoLibDas(&uav_geolibdas,&manned_aircraft_geolibdas));
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = setBit(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,10,1);
	/*无人机相对有人机方位角    单位：°  最小值：-180  最大值：180*/
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_copter_angle = getAzimuthGeoLibDas(&manned_aircraft_geolibdas,&uav_geolibdas)*180.0/3.1415926;//顺时针正，逆时针负？
	current_uav_angle=CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_copter_angle;
	if(current_uav_angle >= -180 && current_uav_angle <= 180)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = setBit(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,9,1);
	}
	double manner_navigation_hangxiang_angle = DPU_CCC_data_4.true_direction;  //有人机航向角
	//  0H=左前，1H=前，2H=右前，3H=右，4H=右后，5H=后，6H=左后，7H=左
	int fangWeiJiaoChaZhiChuShi = (int)(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_copter_angle-manner_navigation_hangxiang_angle);
	if (fangWeiJiaoChaZhiChuShi < 0)
	{
		fangWeiJiaoChaZhiChuShi = fangWeiJiaoChaZhiChuShi +360;
	}
	int fangWeiJiaoChaZhi = fangWeiJiaoChaZhiChuShi % 360;
	if(fangWeiJiaoChaZhi > 0 && fangWeiJiaoChaZhi <= 45)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.Thread_UAV_Horizontal_Position_2 = 0x1;
	}
	else if (fangWeiJiaoChaZhi > 45 && fangWeiJiaoChaZhi <= 90)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.Thread_UAV_Horizontal_Position_2 = 0x3;
	}
	else if (fangWeiJiaoChaZhi > 90 && fangWeiJiaoChaZhi <= 135)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.Thread_UAV_Horizontal_Position_2 = 0x3;
	}
	else if (fangWeiJiaoChaZhi > 135 && fangWeiJiaoChaZhi <= 180)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.Thread_UAV_Horizontal_Position_2 = 0x5;
	}
	else if (fangWeiJiaoChaZhi > 180 && fangWeiJiaoChaZhi <= 225)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.Thread_UAV_Horizontal_Position_2 = 0x5;
	}
	else if (fangWeiJiaoChaZhi > 225 && fangWeiJiaoChaZhi <= 270)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.Thread_UAV_Horizontal_Position_2 = 0x7;
	}
	else if (fangWeiJiaoChaZhi > 270 && fangWeiJiaoChaZhi <= 315)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.Thread_UAV_Horizontal_Position_2 = 0x7;
	}
	else if (fangWeiJiaoChaZhi > 315 && fangWeiJiaoChaZhi <= 360)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.Thread_UAV_Horizontal_Position_2 = 0x1;
	}

	if((cnt % 20)==0)//1秒比较一次
	{
		double uav_angle_change=current_uav_angle - histroy_uav_data.his_uav_relative_azmuith;
		if( uav_angle_change> 20)//todo:暂定角度差为20
		{
			CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_angle_change = 1;				/*无人机方位角变化趋势  0=无效;1=顺时针;2=逆时针 ;*/
		}
		else if((uav_angle_change > -20) && (uav_angle_change <20))
		{
			CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_angle_change=0;
		}
		else
		{
			CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_angle_change=2;
		}
		histroy_uav_data.his_uav_relative_azmuith = current_uav_angle;
		cnt=0;
	}
	//(无人机相对有人机方位角-有人机的航向角)取模。[0-45]前，[45-135]右边，[135-225]后，[225-315]左，[315-360]前。向前靠。
	//CCC_DPU_data_3.drone_specific_informations[i].uav_infos.Thread_UAV_Horizontal_Position_2 = 3;/*威胁无人机水平相对位置*/
	//比较无人机和有人机高度，如果无人机的气压高度小于等于有人机气压高度就写为1，表示下。否则为0表示无人机在有人机上。
	CCC_DPU_data_3.drone_specific_informations[i].uav_infos.Thread_UAV_Vertical_Position_2 = 0;	/*威胁无人机垂直相对位置0H-上，1H-下*/
	if (current_uav_height <= DPU_CCC_data_4.absolute_barometric_altitude)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_infos.Thread_UAV_Vertical_Position_2 = 1;	/*威胁无人机垂直相对位置0H-上，1H-下*/
	}

	if(s4a_flag == 1)
	{
		if(s4A_frame.control_1.status == 1)
		{
			//地面
			CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_wow = 1;
		}
		else if(s4A_frame.control_1.status == 0)
		{
			//飞行
			CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_wow = 2;
		}
	}
	// 无人机任务中状态信息
	CCC_DPU_data_3.drone_specific_informations[i].uav_mission_infos.task_id = uav_route[i].task_id;			/*无人机当前任务ID*/
	/*根据无人机信息和指控生成航线信息判断。当前任务状态  0=N/A;1=正在执行任务;2=任务执行完毕;*/
	if(uav_route[i].hull_number - 2 == uav_route[i].waypoint_number)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_mission_infos.mission_status = 2;
	}
	else
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_mission_infos.mission_status = 1;
	}
	if(uav_route[i].tasking)
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_mission_infos.mission_progress = 100 * ((uav_route[i].hull_number - 2) / uav_route[i].waypoint_number);  /*todo。当前任务进度*/
	}
	else
	{
		CCC_DPU_data_3.drone_specific_informations[i].uav_mission_infos.mission_progress = 0;  /*todo。当前任务进度*/
	}

	//新增机电参数 20250619new 新增帧数判断 20250817new
	if(s4c_flag == 1)
	{
		//燃气发生器转速
		CCC_DPU_data_3.drone_specific_informations[i].machine_sta.Np = s4C_frame.np * 0.2;
		//输出轴扭矩
		CCC_DPU_data_3.drone_specific_informations[i].machine_sta.Q = s4C_frame.torque * 0.1;
		//滑油温度
		CCC_DPU_data_3.drone_specific_informations[i].machine_sta.oil_temp = s4C_frame.engine_lubric_temperature * 0.1;
		//滑油压力
		CCC_DPU_data_3.drone_specific_informations[i].machine_sta.oil_pressure = s4C_frame.engine_lubric_pressure * 0.1;
		//燃油压力
		CCC_DPU_data_3.drone_specific_informations[i].machine_sta.fuel_pressure = s4C_frame.pipe_fuel_pressure * 0.001;
	}
	if(s4b_flag == 1)
	{
		//发动机状态
		unsigned char engine_status = s4B_frame.engine.engine_status & 0x0f;
		CCC_DPU_data_3.drone_specific_informations[i].machine_sta.engine_sta = engine_status;
		//动刹车
		if(get_int8_bit(s4B_frame.rotor_system.rotor_system_status,5))
		{
			CCC_DPU_data_3.drone_specific_informations[i].machine_sta.rotor_stop_sta = 2;
		}
		//静刹车
		else if(get_int8_bit(s4B_frame.rotor_system.rotor_system_status,6))
		{
			CCC_DPU_data_3.drone_specific_informations[i].machine_sta.rotor_stop_sta = 1;
		}
		else
		{
			CCC_DPU_data_3.drone_specific_informations[i].machine_sta.rotor_stop_sta = 0;
		}
		//异常
		if(get_int8_bit(s4B_frame.rotor_system.rotor_system_status,5) && get_int8_bit(s4B_frame.rotor_system.rotor_system_status,6))
		{
			CCC_DPU_data_3.drone_specific_informations[i].machine_sta.rotor_stop_sta = 3;
		}
		//增压1
		CCC_DPU_data_3.drone_specific_informations[i].machine_sta.boost1_sta = get_int8_bit(s4B_frame.engine.engine_status,6);
		//增压2
		CCC_DPU_data_3.drone_specific_informations[i].machine_sta.boost2_sta = get_int8_bit(s4B_frame.engine.engine_status,7);
	}
	if(s5e_flag == 1)
	{
		//供油
		CCC_DPU_data_3.drone_specific_informations[i].machine_sta.oil_supply_sta = s5E_frame.power_system_2.supply_status;
	}

	//收到飞控返航回报发送三拍到综显 20250819new
	static char FH_cnt = 0;
	static char FH_rtn = 0;
	static int fh_out[4] = {0,0,0,0};//返航超时计数
	if(fh_timeout[i] == 1)
	{
		fh_out[i]++;
	}
	if(fh_out[i] > 400)
	{
		printf("FH timeout\n");
		//返航超时
		FH_rtn = 3;
		//发送三拍
		FH_cnt = 3;
		//状态回收
		fh_timeout[i] = 0;
		fh_out[i] = 0;
	}
	//收到回报
	if(s82_frame_30[i] == 1)
	{
		printf("FH success\n");
		//返航成功
		FH_rtn = 1;
		//发送三拍
		FH_cnt = 3;
		//状态回收
		s82_frame_30[i] =0;
		fh_timeout[i] = 0;
		fh_out[i] = 0;
	}
	else if(s82_frame_30[i] == -1)
	{
		printf("FH erro\n");
		//返航失败
		FH_rtn = 2;
		//发送三拍
		FH_cnt = 3;
		//状态回收
		s82_frame_30[i] =0;
		fh_timeout[i] = 0;
		fh_out[i] = 0;
	}
	if(FH_cnt > 0)
	{
		//返航状况回报
		CCC_DPU_data_3.drone_specific_informations[i].loading_status.fh_sta = FH_rtn;
		FH_cnt--;
	}
	else
	{
		//重置
		CCC_DPU_data_3.drone_specific_informations[i].loading_status.fh_sta = 0;
	}

	//航线状态 20250821new
	if(uav_route[i].route_number == 1)
	{
		//地面
		CCC_DPU_data_3.drone_specific_informations[i].loading_status.buoy_sta = 1;
	}
	else if(uav_route[i].route_number == 17)
	{
		//返航
		CCC_DPU_data_3.drone_specific_informations[i].loading_status.buoy_sta = 2;
	}
	else if(uav_route[i].route_number == 5 || uav_route[i].route_number == 6 || uav_route[i].route_number == 48 || uav_route[i].route_number == 49)
	{
		//任务
		CCC_DPU_data_3.drone_specific_informations[i].loading_status.buoy_sta = 3;
	}
	else if(uav_route[i].route_number == 45)
	{
		//地面站编队航线
		CCC_DPU_data_3.drone_specific_informations[i].loading_status.buoy_sta = 4;
	}
	else
	{
		//NA
		CCC_DPU_data_3.drone_specific_informations[i].loading_status.buoy_sta = 0;
	}

	//编队能力 20251218new
	CCC_DPU_data_3.drone_specific_informations[i].formation_ability = s7_redundancy[i];

	//空地链相关信息赋值
	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].uav_id =(unsigned short)CCC_DPU_data_3.drone_specific_informations[i].platform_num;
	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].control_attribution = (unsigned char)(CCC_DPU_data_3.drone_specific_informations[i].platform_control_status == 1) ? 2:1;
	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].unmanned_helicopter_latitude = (int)(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_lati * lat_scale2);
	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].unmanned_helicopter_longitude = (int)(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_longi * lon_scale2);
	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].barometric_height = (short)(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_height/0.2);
	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].radio_height = (short)(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.radio_height/0.1);
	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].vertical_velocity = (char)(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.vectx_speed/0.1);
	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].vacuum_speed = (short)(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_speed/0.1);
	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].tilt = (short)(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_pitch/0.01);
	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].roll_angle = (short)(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_roll/0.01);
	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].hangxiang_angle = (unsigned short)(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_heading/0.01);
	if(s3A_frame.frame_type != 0)
	{
		blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].satellite_height = (short)(s3A_frame.satellite_altitude*0.1);
		blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].current_flight_path_num = (char)(s3A_frame.route_number);
		blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].current_flight_waypoint_num = (char)(s3A_frame.waypoint_number);
	}
	if(s81_frame.frame_type != 0)
	{
		blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].absolute_height = (short)(s81_frame.absolute_height*0.2);
		blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].relative_height = (short)(s81_frame.relative_height*0.2);
		blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].body_axis_longitudinal_speed = (short)(s81_frame.body_grouspeed_x*0.1);
		blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].body_axis_lateral_ground_velocity = (short)(s81_frame.body_grouspeed_y*0.1);
	}
	if(s4A_frame.frame_type != 0)
		blk_ccc_kdl_000.unmanned_helicopter_flight_informations[i].flight_path = (unsigned short)(s4A_frame.fly_course*0.1);



	//每拍清空标志位
	s3a_flag = 0;
	s4a_flag = 0;
	s4b_flag = 0;
	s4c_flag = 0;
	s5b_flag = 0;
	s5c_flag = 0;
	s5e_flag = 0;

}
void warnning_detection()
{
	//空域冲突检测
	Point p;
	int rtn = 0;
	//无人机与有人机空域冲突检测
	if(blk_ccc_ofp_033[0].area_informations[7].area_code != 0)//判断有人机空域是否有效
	{
		for(int j = 0 ; j < 4 ; j ++)
		{
			//判断无人机是否有效
			if(CCC_DPU_data_3.drone_specific_informations[j].platform_num != 0)
			{
				p.lon = CCC_DPU_data_3.drone_specific_informations[j].uav_infos.uav_longi;
				p.lat = CCC_DPU_data_3.drone_specific_informations[j].uav_infos.uav_lati;
				rtn = pnpoly((Point *)blk_ccc_ofp_033[0].area_informations[7].polygonals.point_coordinates,
						blk_ccc_ofp_033[0].area_informations[7].polygonals.point_number,
						p);
				//判断是否有空域冲突

	//有人机与无人机空域冲突检测
		//判断无人机空域是否有效
			//判断是否有空域冲突
			if(rtn != 0)
			{
				CCC_DPU_data_3.drone_specific_informations[j].uav_infos.uav_icon_flag = 1;/*无人机提示图标显示标志位  0=无效;1=显示;2=不显示;*/
			}
			else
			{
				CCC_DPU_data_3.drone_specific_informations[j].uav_infos.uav_icon_flag = 2;/*无人机提示图标显示标志位  0=无效;1=显示;2=不显示;*/
			}
			rtn = 0;
		}
	}
	}
	for (int j = 0; j < 4; j++)
	{
		if (blk_ccc_ofp_033[1].area_informations[j].area_code != 0)
		{
			p.lon = DPU_CCC_data_4.manned_longitude_and_latitude_synt.longitude;
			p.lat = DPU_CCC_data_4.manned_longitude_and_latitude_synt.latitude;
			rtn = pnpoly((Point *)blk_ccc_ofp_033[1].area_informations[j].polygonals.point_coordinates,
						 blk_ccc_ofp_033[1].area_informations[j].polygonals.point_number,
						 p);
			if (rtn != 0)
			{
				CCC_DPU_data_3.drone_specific_informations[j].uav_infos.uav_icon_flag = 1; /*无人机提示图标显示标志位  0=无效;1=显示;2=不显示;*/
			}
			else
			{
				CCC_DPU_data_3.drone_specific_informations[j].uav_infos.uav_icon_flag = 2; /*无人机提示图标显示标志位  0=无效;1=显示;2=不显示;*/
			}
			rtn = 0;
		}
	}

	//防撞检测,有人机位置与无人机位置小于2km告警
	for(int j = 0 ; j < 4 ; j ++)
	{
		//判断无人机是否有效
		if(CCC_DPU_data_3.drone_specific_informations[j].platform_num != 0)
		{
			static double distance[4] = {0,0,0,0};
			double distance_new = calculate_distances(DPU_CCC_data_4.manned_longitude_and_latitude_synt.latitude,
					DPU_CCC_data_4.manned_longitude_and_latitude_synt.longitude,
					CCC_DPU_data_3.drone_specific_informations[j].uav_infos.uav_lati,
					CCC_DPU_data_3.drone_specific_informations[j].uav_infos.uav_longi);
			static int cnt[4];
			if(distance[j] > distance_new)
			{
				cnt[j]++;
				if(cnt[j] > 2)
				{
					//靠近
					CCC_DPU_data_3.drone_specific_informations[j].uav_infos.Threat_UAV_near_far = 1;
					cnt[j] = 0;
				}
			}
			else if(distance[j] < distance_new)
			{
				cnt[j]++;
				if(cnt[j] > 2)
				{
					//远离
					CCC_DPU_data_3.drone_specific_informations[j].uav_infos.Threat_UAV_near_far = 2;
					cnt[j] = 0;
				}
			}
			else
			{
				//缺省
				CCC_DPU_data_3.drone_specific_informations[j].uav_infos.Threat_UAV_near_far = 0;
				cnt[j] = 0;
			}
			//记录最新的距离
			distance[j] = distance_new;
			if(distance[j] < 5)
			{
				CCC_DPU_data_3.drone_specific_informations[j].uav_infos.Threat_UAV_Warning_2 = 1;	/*todo:威胁无人机告警标志位  0-不告警，1-告警*/
			}
			else
			{
				CCC_DPU_data_3.drone_specific_informations[j].uav_infos.Threat_UAV_Warning_2 = 0;	/*todo:威胁无人机告警标志位  0-不告警，1-告警*/
			}
		}
	}

}
//计算经纬度之间的距离
double calculate_distances(double lat1, double lon1,double lat2, double lon2)
{
	double dlat = (lat2 - lat1) * M_PI / 180.0;
	double dlon = (lon2 - lon1) * M_PI / 180.0;
	double a = sin(dlat/2) * sin(dlat/2) + cos(lat1*M_PI/180.0) * cos(lat2*M_PI/180.0) * sin(dlon/2) * sin(dlon/2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	return 6371.0 * c;
}
void send_drone_state_information()
{
	//故障告警检测
	warnning_detection();

	align_send_information(&(CCC_DPU_data_3),sizeof(drone_state_information),0);

	//	CCC_DPU_data_3.drone_specific_informations[0].loading_status.bar_out_len = 666;
	// 综显发送---由于发送无需带头，因此开始位置从第三位
	Send_Message_Local(DDSTables.CCC_DPU_3.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	if(enRetCode != 0)
	{
		//qDebug()<<"drone_state_send_error, ret"<<enRetCode;
	}
	else
	{
		//		if(CCC_DPU_data_3.drone_specific_informations[0].platform_num == 0)
		//		{
		//			static int cnt = 0;
		//			cnt++;
		//			printf("yc error %d\n",cnt);
		//		}
	}

	//发送给pad new20250620
	if(Pad_heart_flag == 1)
	{
		Send_Message(DDSTables.CCC_PAD_013.niConnectionId,0,&transaction_id, send_array.dataA,&message_type_id, data_length, &enRetCode);
	}


#if 0
	/*任务系统不增加卫星高度,临时处理*/
	char temChar[4096];
	size_t front = 0;
	size_t back = 0;
	data_length = sizeof(drone_state_information);
	// 必须从后向前拷贝
	for(int i=UAV_MAX_NUM-1; i>=0; i--)
	{
		// 备份
		memcpy(temChar, send_array.dataA, data_length);
		// 前面相同的长度
		front = (size_t)&CCC_DPU_data_3.drone_specific_informations[i].uav_infos.sat_height - (size_t)&CCC_DPU_data_3;
		// 后面需要拷贝的长度
		back = data_length - front - 4;
		memcpy(send_array.dataA+front, temChar+front+4, back);
		// 总长度减4
		data_length -= 4;
	}
#endif

	// 给任务系统发送
	Send_Message(DDSTables.CCC_DPM_0.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}


// 3.3 编队链路状态信息
void init_formation_link_status_information()
{
	//上行锁定是有人机到无人机 20250729
	memset( &blk_kkl_ccc_006 , 0 , sizeof blk_kkl_ccc_006);
	//无人机链路状态数据，0xaa2c06
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.KKL_C_CCC_01.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		memcpy(&blk_kkl_ccc_006,dds_data_rece.dataA,sizeof blk_kkl_ccc_006);
	}
	else
	{
//		printf("recv blk_kkl_ccc_006 erro \n");
	}

	//下行锁定是无人机到有人机 20250729
	memset( &blk_kkl_ccc_007 , 0 , sizeof blk_kkl_ccc_007);
	//本机链路状态数据，0xaa2c07
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.KKL_C_CCC_02.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		memcpy(&blk_kkl_ccc_007,dds_data_rece.dataA,sizeof blk_kkl_ccc_007);
	}
	else
	{
//		printf("recv blk_kkl_ccc_007 erro \n");
	}

	memset( &blk_ccc_ofp_199 , 0 , sizeof blk_ccc_ofp_199);
	//U端本机链路状态数据，0xa222c7
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.KKL_U_CCC_01.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		memcpy(&blk_ccc_ofp_199,dds_data_rece.dataA,sizeof blk_ccc_ofp_199);
		//		// 收到后转给综显
		//		data_length = sizeof(blk_ccc_ofp_199);
		//		Send_Message(DDSTables.CCC_DPU_37.niConnectionId,0,&transaction_id, &blk_ccc_ofp_199, &message_type_id, data_length, &enRetCode);
		//发送给pad new20250620
		//		Send_Message(DDSTables.CCC_PAD_199.niConnectionId,0,&transaction_id, &blk_ccc_ofp_199,&message_type_id, data_length, &enRetCode);

	}

	memset( &CCC_DPU_MMM_200 , 0 , sizeof CCC_DPU_MMM_200);
	//U端无人机链路状态数据，0xa222c8
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.KKL_U_CCC_02.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		memcpy(&CCC_DPU_MMM_200,dds_data_rece.dataA,sizeof CCC_DPU_MMM_200);
		//		// 收到后转给综显
		//		data_length = sizeof(CCC_DPU_MMM_200);
		//		Send_Message(DDSTables.CCC_DPU_38.niConnectionId,0,&transaction_id, &CCC_DPU_MMM_200, &message_type_id, data_length, &enRetCode);
		//发送给pad new20250620
		//		Send_Message(DDSTables.CCC_PAD_200.niConnectionId,0,&transaction_id, &CCC_DPU_MMM_200,&message_type_id, data_length, &enRetCode);
	}

	//U链在网成员 20250725new 0xa22210
	static int cnt_197 = 0;
	message_size = RECV_MAX_SIZE;
	Receive_Message(DDSTables.KKL_U_CCC_11.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		memcpy(&blk_kkl_ccc_197,dds_data_rece.dataA,sizeof blk_kkl_ccc_197);
		//		if(blk_kkl_ccc_197.jianmianzhan1_online == 0)
		//		{
		//			printf("197 erro man %d uman %d jmz %d\n",blk_kkl_ccc_197.manned_aircraft_online
		//					,blk_kkl_ccc_197.unmanned1_aircraft_online,blk_kkl_ccc_197.jianmianzhan1_online);
		//		}
		//收到清零
		cnt_197 = 0;
	}
	else if(enRetCode != 0)
	{
		cnt_197 ++;
	}
	//超过十拍没收到清空 U链在网状态 20250819new
	if(cnt_197 > 10)
	{
		cnt_197 = 0;
		memset( &blk_kkl_ccc_197 , 0 , sizeof blk_kkl_ccc_197);
	}


#if 1
	/************************************************ 协同通信无法仿真这些数据 此处使用假值测试 ***************************************************/
	//初始化
	memset(&CCC_DPU_data_4,0,sizeof(formation_link_status_information));

	CCC_DPU_data_4.drone_number = CCC_DPU_data_3.drone_number;  // 无人机数量
	//	CCC_DPU_data_4.surface_station_num = blk_kdl_ccc_003.KDLCNumber;//tem  blk_kdl_ccc_003.KDLCNumber; // 舰面站数量

	//无人机链路状态i
	for (int i = 0; i < UAV_MAX_NUM; i++)
	{
		// 无效无人机跳过
		if(CCC_DPU_data_3.drone_specific_informations[i].platform_model == 0)
		{
			continue;
		}
		CCC_DPU_data_4.drone_link_status_informations[i].uav_model = CCC_DPU_data_3.drone_specific_informations[i].platform_model;				/*平台1型号  0=N/A;1=WZ2;*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_sn = CCC_DPU_data_3.drone_specific_informations[i].platform_serial_num;					/*平台序号*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_code = CCC_DPU_data_3.drone_specific_informations[i].platform_num;/*平台编号*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_ulink_chanel = CCC_DPU_MMM_200.ULParSet_1[i].DownChannel_3_1;/*平台U链频道号*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_clink_down_channel = blk_kkl_ccc_006.CR_uav_cl_status_info[i].DownChannel;/*平台C链下行频道号*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_clink_up_channel = blk_kkl_ccc_006.CR_uav_cl_status_info[i].UpChannel;/*平台C链上行频道号*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_u_strength[0] =CCC_DPU_MMM_200.ULParSet_1[i].ConSignalStren_2_2_1 ;/*平台U链信号强度*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_u_strength[1] =blk_ccc_ofp_199.ULparSet_1[i].ConSignalStren_2_2 ;/*平台U链信号强度*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_c_strength[0] = blk_kkl_ccc_006.CR_uav_cl_status_info[i].ConSignalStren;/*平台C链信号强度*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_c_strength[1] = blk_kkl_ccc_007.uav_cl_status_info[i].ConSignalStren;/*平台C链信号强度*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_ctrl_mode = CCC_DPU_data_3.drone_specific_informations[i].platform_control_status;
		CCC_DPU_data_4.drone_link_status_informations[i].handover_id = blk_kkl_ccc_006.CR_uav_cl_status_info[i].idWillGetUav;/*接机站ID由KDL发送*/ //20250620需要修改，从空空链收
		CCC_DPU_data_4.drone_link_status_informations[i].handover_up_channel = blk_kkl_ccc_006.CR_uav_cl_status_info[i].HandOverUpChannel;	/*交接上行频道*/
		CCC_DPU_data_4.drone_link_status_informations[i].handover_down_channel = blk_kkl_ccc_006.CR_uav_cl_status_info[i].HandOverDownChannel;	/*交接下行频道*/
		CCC_DPU_data_4.drone_link_status_informations[i].handover_down_speed = blk_kkl_ccc_006.CR_uav_cl_status_info[i].DownRate;/*交接下行速率*/
		CCC_DPU_data_4.drone_link_status_informations[i].time_remaing = CCC_DPU_data_3.drone_specific_informations[i].remaining_mission_time;//剩余时间
	}

	// 舰面站指控状态
	//	for(int j = 0 ; j < 2 ; j++)
	//	{
	//		CCC_DPU_data_4.jiemianzhikong_info[j].station_id = blk_kdl_ccc_003.clcostatus[j].ShipID;			/*舰面站ID*/
	//		CCC_DPU_data_4.jiemianzhikong_info[j].station_sn = j + 1 ;			/*舰面站序号 0:NA, 1:舰面站1 2:舰面站2*/
	//	}

	//暂时写死 20250727
	if(blk_kkl_ccc_197.jianmianzhan1_online == 1)
	{
		CCC_DPU_data_4.surface_station_num = 1;
		CCC_DPU_data_4.jiemianzhikong_info[0].station_id = GCS_ID;
		CCC_DPU_data_4.jiemianzhikong_info[0].station_sn = 1;
		if(formationId[0].isControl == 1)
			CCC_DPU_data_4.jiemianzhikong_info[0].ctrl_uav_code[0].uav_code = 0;
		else
			CCC_DPU_data_4.jiemianzhikong_info[0].controll_uav_num = 1;
		CCC_DPU_data_4.jiemianzhikong_info[0].ctrl_uav_code[0].uav_code = UAV1_ID;
	}

	//舰面站控制无人机数据 20250727
	//	if()


	//U链在网成员发送综显赋值 20250725new
	CCC_DPU_data_4.network_member.U_member.manned_aircraft_online = blk_kkl_ccc_197.manned_aircraft_online;
	CCC_DPU_data_4.network_member.U_member.unmanned1_aircraft_online = blk_kkl_ccc_197.unmanned1_aircraft_online;
	CCC_DPU_data_4.network_member.U_member.unmanned2_aircraft_online = blk_kkl_ccc_197.unmanned2_aircraft_online;
	CCC_DPU_data_4.network_member.U_member.unmanned3_aircraft_online = blk_kkl_ccc_197.unmanned3_aircraft_online;
	CCC_DPU_data_4.network_member.U_member.unmanned4_aircraft_online = blk_kkl_ccc_197.unmanned4_aircraft_online;
	CCC_DPU_data_4.network_member.U_member.jianmianzhan1_online = blk_kkl_ccc_197.jianmianzhan1_online;
	CCC_DPU_data_4.network_member.U_member.monijianmianzhongduan_online = blk_kkl_ccc_197.monijianmianzhongduan_online;
	//	//无人机1，U链联通状态
	//	for(int i =0 ; i < 8 ; i ++)
	//	{
	//		if((blk_ccc_ofp_199.ULparSet[i].NoheliID_1_3 == CCC_DPU_MMM_200.ULParSet_1[0].NoheliID_1_3_1) && (blk_ccc_ofp_199.ULparSet[i].IFNoHeliCon_2_2==1) && (CCC_DPU_MMM_200.ULParSet_1[0].IFNoHeliCon_2_2_1==1))
	//		{
	//			CCC_DPU_data_4.network_member.U_member.unmanned1_aircraft_online=1;
	//		}
	//	}
	//	//无人机2，U链联通状态
	//	for(int i =0 ; i < 8 ; i ++)
	//	{
	//		if((blk_ccc_ofp_199.ULparSet[i].NoheliID_1_3 == CCC_DPU_MMM_200.ULParSet_1[1].NoheliID_1_3_1) && (blk_ccc_ofp_199.ULparSet[i].IFNoHeliCon_2_2==1) && (CCC_DPU_MMM_200.ULParSet_1[1].IFNoHeliCon_2_2_1==1))
	//		{
	//			CCC_DPU_data_4.network_member.U_member.unmanned2_aircraft_online=1;
	//		}
	//	}
	//	//无人机3，U链联通状态
	//	for(int i =0 ; i < 8 ; i ++)
	//	{
	//		if((blk_ccc_ofp_199.ULparSet[i].NoheliID_1_3 == CCC_DPU_MMM_200.ULParSet_1[2].NoheliID_1_3_1) && (blk_ccc_ofp_199.ULparSet[i].IFNoHeliCon_2_2==1) && (CCC_DPU_MMM_200.ULParSet_1[2].IFNoHeliCon_2_2_1==1))
	//		{
	//			CCC_DPU_data_4.network_member.U_member.unmanned3_aircraft_online=1;
	//		}
	//	}
	//	//无人机4，U链联通状态
	//	for(int i =0 ; i < 8 ; i ++)
	//	{
	//		if((blk_ccc_ofp_199.ULparSet[i].NoheliID_1_3 == CCC_DPU_MMM_200.ULParSet_1[3].NoheliID_1_3_1) && (blk_ccc_ofp_199.ULparSet[i].IFNoHeliCon_2_2==1) && (CCC_DPU_MMM_200.ULParSet_1[3].IFNoHeliCon_2_2_1==1))
	//		{
	//			CCC_DPU_data_4.network_member.U_member.unmanned4_aircraft_online=1;
	//		}
	//	}
	//	//舰面端1，U链联通状态
	//	for(int i =0 ; i < 8 ; i ++)
	//	{
	//		if((blk_ccc_ofp_199.ULparSet[i].NoheliID_1_3 == CCC_DPU_MMM_200.ULParSet_2[0].NoheliID_1_4_1) && (blk_ccc_ofp_199.ULparSet[i].IFNoHeliCon_2_2==1) && (CCC_DPU_MMM_200.ULParSet_2[0].IFNoHeliCon_2_2_1_1==1))
	//		{
	//			CCC_DPU_data_4.network_member.U_member.jianmianzhan1_online = 1;
	//		}
	//	}


	//无人机1C链连通情况，上行是否锁定,增加控制站id判断 20250728new
	if((blk_kkl_ccc_006.CR_uav_cl_status_info[0].IfNoHeliCon == 1) && (blk_kkl_ccc_006.CR_uav_cl_status_info[0].UAV_StationID == 0x9001))
	{
		CCC_DPU_data_4.network_member.C_UAV_F.copter_C_uav1_up = 1;//C链连通情况(有人/无人)
	}
	//无人机1C链连通情况，下行是否锁定
	if((blk_kkl_ccc_007.uav_cl_status_info[0].IfNoHeliCon == 1))
	{
		CCC_DPU_data_4.network_member.C_UAV_F.copter_C_uav1_down = 1;//C链连通情况(有人/无人)
	}

	//无人机2C链连通情况，上行是否锁定
	if((blk_kkl_ccc_006.CR_uav_cl_status_info[1].IfNoHeliCon == 1) && (blk_kkl_ccc_006.CR_uav_cl_status_info[1].UAV_StationID == 0x9001))
	{
		CCC_DPU_data_4.network_member.C_UAV_F.copter_C_uav2_up = 1;//C链连通情况(有人/无人)
	}
	//无人机2C链连通情况，下行是否锁定
	if((blk_kkl_ccc_007.uav_cl_status_info[1].IfNoHeliCon == 1))
	{
		CCC_DPU_data_4.network_member.C_UAV_F.copter_C_uav2_down = 1;//C链连通情况(有人/无人)
	}

	//无人机3C链连通情况，上行是否锁定
	if((blk_kkl_ccc_006.CR_uav_cl_status_info[2].IfNoHeliCon == 1) && (blk_kkl_ccc_006.CR_uav_cl_status_info[2].UAV_StationID == 0x9001))
	{
		CCC_DPU_data_4.network_member.C_UAV_F.copter_C_uav3_up = 1;//C链连通情况(有人/无人)
	}
	//无人机3C链连通情况，下行是否锁定
	if((blk_kkl_ccc_007.uav_cl_status_info[2].IfNoHeliCon == 1))
	{
		CCC_DPU_data_4.network_member.C_UAV_F.copter_C_uav3_down = 1;//C链连通情况(有人/无人)
	}

	//无人机4C链连通情况，上行是否锁定
	if((blk_kkl_ccc_006.CR_uav_cl_status_info[3].IfNoHeliCon == 1) && (blk_kkl_ccc_006.CR_uav_cl_status_info[3].UAV_StationID == 0x9001))
	{
		CCC_DPU_data_4.network_member.C_UAV_F.copter_C_uav4_up = 1;//C链连通情况(有人/无人)
	}
	//无人机4C链连通情况，下行是否锁定
	if((blk_kkl_ccc_007.uav_cl_status_info[3].IfNoHeliCon == 1))
	{
		CCC_DPU_data_4.network_member.C_UAV_F.copter_C_uav4_down = 1;//C链连通情况(有人/无人)
	}

	//地面/无人机1上行连通情况
	if((blk_kkl_ccc_006.CR_uav_cl_status_info[0].IfNoHeliCon == 1) && (blk_kkl_ccc_006.CR_uav_cl_status_info[0].UAV_StationID == GCS_ID))
	{
		CCC_DPU_data_4.network_member.stationUAV.station_C_uav1_up = 1;//C链联通情况(地面/无人)
	}
	//地面/无人机1下行连通情况
	if((blk_kdl_ccc_003.clcostatus[0].IFShipCon_2 == 1) && blk_kdl_ccc_003.clcostatus[0].ShipID != MANNED_ID)
	{
		CCC_DPU_data_4.network_member.stationUAV.station_C_uav1_down = 1;//C链联通情况(地面/无人)
	}

	//地面/无人机2上行连通情况
	if((blk_kkl_ccc_006.CR_uav_cl_status_info[1].IfNoHeliCon == 1) && (blk_kkl_ccc_006.CR_uav_cl_status_info[1].UAV_StationID == GCS_ID))
	{
		CCC_DPU_data_4.network_member.stationUAV.station_C_uav2_up = 1;//C链联通情况(地面/无人)
	}
	//地面/无人机2下行连通情况
	if((blk_kdl_ccc_003.clcostatus[1].IFShipCon_2 == 1) && blk_kdl_ccc_003.clcostatus[1].ShipID != MANNED_ID)
	{
		CCC_DPU_data_4.network_member.stationUAV.station_C_uav2_down = 1;//C链联通情况(地面/无人)
	}

	//地面/无人机3上行连通情况
	if((blk_kkl_ccc_006.CR_uav_cl_status_info[2].IfNoHeliCon == 1) /*&& (blk_kkl_ccc_006.CR_uav_cl_status_info[2].UAV_StationID == GCS_ID)*/)
	{
		CCC_DPU_data_4.network_member.stationUAV.station_C_uav3_up = 1;//C链联通情况(地面/无人)
	}
	//地面/无人机3下行连通情况
	if((blk_kdl_ccc_003.clcostatus[2].IFShipCon_2 == 1) && blk_kdl_ccc_003.clcostatus[2].ShipID != MANNED_ID)
	{
		CCC_DPU_data_4.network_member.stationUAV.station_C_uav3_down = 1;//C链联通情况(地面/无人)
	}

	//地面/无人机4上行连通情况
	if((blk_kkl_ccc_006.CR_uav_cl_status_info[3].IfNoHeliCon == 1) /*&& (blk_kkl_ccc_006.CR_uav_cl_status_info[3].UAV_StationID == GCS_ID)*/)
	{
		CCC_DPU_data_4.network_member.stationUAV.station_C_uav4_up = 1;//C链联通情况(地面/无人)
	}
	//地面/无人机4下行连通情况
	if((blk_kdl_ccc_003.clcostatus[3].IFShipCon_2 == 1) && blk_kdl_ccc_003.clcostatus[3].ShipID != MANNED_ID)
	{
		CCC_DPU_data_4.network_member.stationUAV.station_C_uav4_down = 1;//C链联通情况(地面/无人)
	}

	if(blk_kdl_ccc_003.clcostatus[0].IFShipCon ==1  && blk_kdl_ccc_003.clcostatus[0].ShipID == MANNED_ID)
	{
		CCC_DPU_data_4.network_member.KDLLT_Up = 1;
	}
	if(blk_kdl_ccc_003.clcostatus[0].IFShipCon_2 ==1 && blk_kdl_ccc_003.clcostatus[0].ShipID == MANNED_ID)
	{
		CCC_DPU_data_4.network_member.KDLLT_Down = 1;
	}


#else
	//使用以下代码进行编队链路的仿真测试
	//CCC将如下四个数据组合为 CCC-DPU1/DPU2/MMM-014 0xa2220e 编队链路状态信息
	//KKL-CCC/DPU1/DPU2-006 0xaa2c06 无人机链路状态数据
	//KKL-CCC/DPU1/DPU2-007 0xaa2c07 本机链路状态数据
	//U链设备状态信息
	//KDL-CCC/DPU1/DPU2/PMD/SMD-003 0xa62003 空地链链路状态
	CCC_DPU_data_4.drone_number = CCC_DPU_data_3.drone_number;  // 无人机数量
	if (CCC_DPU_data_4.drone_number > 2)
	{
		CCC_DPU_data_4.surface_station_num = 2; // 舰面站数量
	}
	else
	{
		CCC_DPU_data_4.surface_station_num = 1; // 舰面站数量
	}

	//无人机链路状态
	for (int i = 0; i < CCC_DPU_data_4.drone_number; i++)
	{
		CCC_DPU_data_4.drone_link_status_informations[i].uav_model = CCC_DPU_data_3.drone_specific_informations[i].platform_model;				/*平台1型号  0=N/A;1=WZ2;*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_sn = CCC_DPU_data_3.drone_specific_informations[i].platform_serial_num;					/*平台序号*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_code = CCC_DPU_data_3.drone_specific_informations[i].platform_num;				/*平台编号*/
		CCC_DPU_data_4.drone_link_status_informations[i].uav_ulink_chanel = 111;		/*平台U链频道号*/         //因仿真没有KKL，这里先赋死值
		CCC_DPU_data_4.drone_link_status_informations[i].uav_clink_down_channel = 222;  /*平台C链下行频道号*/       //仿真没有KKL，这里先赋死值
		CCC_DPU_data_4.drone_link_status_informations[i].uav_clink_up_channel = 333;	/*平台C链上行频道号*/       //仿真没有KKL，这里先赋死值
		CCC_DPU_data_4.drone_link_status_informations[i].uav_u_strength[0] = 1;			/*平台U链信号强度*/        //仿真没有KKL，这里先赋死值
		CCC_DPU_data_4.drone_link_status_informations[i].uav_c_strength[0] = 1;			/*平台C链信号强度*/        //仿真没有KKL，这里先赋死值
		CCC_DPU_data_4.drone_link_status_informations[i].handover_id = 5001;				/*接机站ID*/       //仿真没有KKL，这里先赋死值
		CCC_DPU_data_4.drone_link_status_informations[i].handover_up_channel = 333;		/*交接上行频道*/          //仿真没有KKL，这里先赋死值
		CCC_DPU_data_4.drone_link_status_informations[i].handover_down_channel = 222;	/*交接下行频道*/          //仿真没有KKL，这里先赋死值
		CCC_DPU_data_4.drone_link_status_informations[i].handover_down_speed = 2;		/*交接下行速率*/          //仿真没有KKL，这里先赋死值
	}

	//U链设备状态信息

	//KDL-CCC/DPU1/DPU2/PMD/SMD-003 0xa62003 空地链链路状态

	// 舰面站指控状态
	for (int j = 0; j < CCC_DPU_data_4.surface_station_num; j++)
	{
		CCC_DPU_data_4.jiemianzhikong_info[j].station_id = j + 1;			/*舰面站ID*/
		CCC_DPU_data_4.jiemianzhikong_info[j].station_sn = j +1 ;			/*舰面站序号 0:NA, 1:舰面站1 2:舰面站2*/
		if (CCC_DPU_data_4.surface_station_num == 1)
		{
			CCC_DPU_data_4.jiemianzhikong_info[j].controll_uav_num = CCC_DPU_data_3.drone_number;	/*指控无人机数量*/
		}
		else if (CCC_DPU_data_4.surface_station_num == 2)
		{
		}
	}
	/*网络拓扑信息*/
	//        CCC_DPU_data_4.copter_link_info.network_member.U_member = CCC_DPU_data_3.drone_number + 1;		/*U链在网成员*/
	//        CCC_DPU_data_4.copter_link_info.network_member.C_UAV_F = 1;			/*C链联通情况（有人/无人）*/
	//        CCC_DPU_data_4.copter_link_info.network_member.C_Station_UAV = 1;   /*C链联通情况（地面/无人）*/
#endif
}

void send_formation_link_status_information()
{
	init_formation_link_status_information();

	align_send_information(&(CCC_DPU_data_4),sizeof (formation_link_status_information),0);

	//pad发送
	if(Pad_heart_flag == 1)
	{
		Send_Message(DDSTables.CCC_PAD_014.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	}

	// 发送给综显：0xa2220e
	Send_Message(DDSTables.CCC_DPU_4.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);

	// 给任务系统发送
	Send_Message(DDSTables.CCC_DPM_1.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	memset(&CCC_DPU_data_4, 0 , sizeof(formation_link_status_information));
}

void init_blk_ccc_ofp_021()
{
	//初始化
	memset(&blk_ccc_ofp_021,0,sizeof(BLK_CCC_OFP_021));
	blk_ccc_ofp_021.plan_id = blk_ccc_ofp_019.plan_id;//方案编号
	blk_ccc_ofp_021.plan_state = blk_ccc_ofp_019.plan_release_mode;//任务分配方案发布
	blk_ccc_ofp_021.plan_manual = blk_ccc_ofp_019.plan_manual_amend_flag;//任务分配方案是否人工修改
	blk_ccc_ofp_021.platform_serial_num = DPU_CCC_data_11.drone_num;// 无人机编号
	blk_ccc_ofp_021.platform_id = DPU_CCC_data_11.drone_id;// 无人机序号
	blk_ccc_ofp_021.signal_FC00 = 0;//单任务规划无人机是否处于全局任务规划当中
	blk_ccc_ofp_021.platform_num = 1;//当前单无人机指控平台个数
	for(int i = 0; i < 4 ; i ++)
	{
		//取平台参数
		memcpy(&blk_ccc_ofp_021.plan_info[i],&blk_ccc_ofp_019.formation_synergy_mission_programs[i+1],14);
		for(int j = 0; j < 8 ; j ++)
		{
			//取任务参数
			/*任务平台子任务ID号*/
			blk_ccc_ofp_021.plan_info[i].task_sequence_informations[j].id = blk_ccc_ofp_019.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].id;
			/*子任务任务类型*/
			blk_ccc_ofp_021.plan_info[i].task_sequence_informations[j].type = blk_ccc_ofp_019.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].type;
			/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
			blk_ccc_ofp_021.plan_info[i].task_sequence_informations[j].point_or_area = blk_ccc_ofp_019.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].point_or_area;
			/*任务区/点/线/目标编号*/
			blk_ccc_ofp_021.plan_info[i].task_sequence_informations[j].point_or_area_id = blk_ccc_ofp_019.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].point_or_area_id;
			/*子任务完成时间有效位*/
			blk_ccc_ofp_021.plan_info[i].task_sequence_informations[j].finish_time_valid = blk_ccc_ofp_019.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].finish_time_valid;
			/*子任务任务高度有效位*/
			blk_ccc_ofp_021.plan_info[i].task_sequence_informations[j].finish_height_valid = blk_ccc_ofp_019.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].finish_height_valid;
			/*子任务完成时间*/
			blk_ccc_ofp_021.plan_info[i].task_sequence_informations[j].finish_time = blk_ccc_ofp_019.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].finish_time;
			/*子任务完成高度*/
			blk_ccc_ofp_021.plan_info[i].task_sequence_informations[j].finish_height = blk_ccc_ofp_019.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].finish_height;
		}
	}
}

void send_blk_ccc_ofp_021()
{
	//单无人机分配结果赋值
	init_blk_ccc_ofp_021();
	data_length = sizeof(BLK_CCC_OFP_021);
	// 综显发送预览方案信息
	Send_Message(DDSTables.CCC_DPU_39.niConnectionId,0,&transaction_id,&blk_ccc_ofp_021, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//qDebug()<<"CCC_DPU_data_8_success!";
	}
}
void send_blk_ccc_ofp_017()
{
	data_length = sizeof(BLK_CCC_OFP_017);
	printf("send017 plan_id=%u mode=%u platform_num=%u\n",blk_ccc_ofp_017.plan_id,blk_ccc_ofp_017.plan_release_mode,blk_ccc_ofp_017.platform_num);
	// 综显发送预览方案信息
	Send_Message(DDSTables.CCC_DPU_6.niConnectionId,0,&transaction_id,&(blk_ccc_ofp_017.plan_id), &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//qDebug()<<"CCC_DPU_data_8_success!";
	}

	// 给pad发送
	if(Pad_heart_flag == 1)
	{
		Send_Message(DDSTables.CCC_PAD_017.niConnectionId,0,&transaction_id,&(blk_ccc_ofp_017), &message_type_id, data_length, &enRetCode);
	}

	// 向空地链发送任务分配结果信息
	send_blk_ccc_kdl_017();
}

//赋值编队信息
void init_blk_ccc_ofp_026()
{
	//长机id,默认1005，从dlr中读取
	blk_ccc_ofp_026.uav_id = load_file.lead_uav_id;
	//长机序号
	blk_ccc_ofp_026.uav_sn = (load_file.lead_uav_id == 0x1005) ? 1 : 2;
	if(s6_bd_flag == 1)
	{
		if(s7_hx_flag == 1)
		{
			//有人机飞行领航
			blk_ccc_ofp_026.temfly_status = 8;
		}
		else if(s7_hx_flag == 0)
		{
			if(s6_hold_flag == 2)
			{
				//具备条件
				if (s9_manned_lead.conditon1 && s9_manned_lead.conditon2 && s9_manned_lead.conditon3 && s9_manned_lead.conditon4 && s9_manned_lead.conditon7 && s9_manned_lead.conditon8)
				{
					//编队-按航线飞-编队保持-具备领航
					blk_ccc_ofp_026.temfly_status = 1;
				}
				//不具备条件
				else
				{
					//编队-按航线飞-编队保持-不具备领航
					blk_ccc_ofp_026.temfly_status = 2;
				}
			}
			else if(s6_hold_flag == 1)
			{
				//编队-按航线飞-编队集结
				blk_ccc_ofp_026.temfly_status = 3;
			}
			else if(s6_hold_flag == 3)
			{
				//编队-按航线飞-队形变换
				blk_ccc_ofp_026.temfly_status = 4;
			}
			else if(s6_hold_flag == 4)
			{
				//编队-按航线飞-编队解散
				blk_ccc_ofp_026.temfly_status = 5;
			}
			else if(s6_hold_flag == 5)
			{
				//编队-按航线飞-退出编队
				blk_ccc_ofp_026.temfly_status = 6;
			}
			else if(s6_hold_flag == 10)
			{
				//编队-按航线飞-单机飞行
				blk_ccc_ofp_026.temfly_status = 7;
			}
		}
	}
	else if(s6_bd_flag == 0)
	{
		//非编队
		blk_ccc_ofp_026.temfly_status = 0;
	}
}
void send_blk_ccc_ofp_026()
{
	init_blk_ccc_ofp_026();
	data_length = sizeof(BLK_CCC_OFP_026);
	// 综显发送预览方案信息
	Send_Message(DDSTables.CCC_DPU_42.niConnectionId,0,&transaction_id,&(blk_ccc_ofp_026), &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//qDebug()<<"CCC_DPU_data_8_success!";
	}
}

//赋值有人机领航进入信息
void init_blk_ccc_ofp_027()
{
	//长机id,默认1005，从dlr中读取
	blk_ccc_ofp_027.uav_id = load_file.lead_uav_id;
	//长机序号
	blk_ccc_ofp_027.uav_sn = (load_file.lead_uav_id == 0x1005) ? 1 : 2;
	blk_ccc_ofp_027.conditon1 = s9_manned_lead.conditon1;
	blk_ccc_ofp_027.conditon2 = s9_manned_lead.conditon2;
	blk_ccc_ofp_027.conditon3 = s9_manned_lead.conditon3;
	blk_ccc_ofp_027.conditon4 = s9_manned_lead.conditon4;
	blk_ccc_ofp_027.conditon5 = s9_manned_lead.conditon7;
	blk_ccc_ofp_027.conditon6 = s9_manned_lead.conditon8;
}
//有人机领航进入
void send_blk_ccc_ofp_027()
{
	init_blk_ccc_ofp_027();
	data_length = sizeof(BLK_CCC_OFP_027);
	// 综显发送预览方案信息
	Send_Message(DDSTables.CCC_DPU_44.niConnectionId,0,&transaction_id,&(blk_ccc_ofp_027), &message_type_id, data_length, &enRetCode);
}

//赋值有人机领航退出信息
void init_blk_ccc_ofp_028()
{
	//长机id,默认1005，从dlr中读取
	blk_ccc_ofp_028.uav_id = load_file.lead_uav_id;
	//长机序号
	blk_ccc_ofp_028.uav_sn = (load_file.lead_uav_id == 0x1005) ? 1 : 2;
	blk_ccc_ofp_028.conditon1 = s9_manned_exit.conditon1;
	blk_ccc_ofp_028.conditon2 = s9_manned_exit.conditon2;
	blk_ccc_ofp_028.conditon3 = s9_manned_exit.conditon3;
	blk_ccc_ofp_028.conditon4 = s9_manned_exit.conditon4;
	blk_ccc_ofp_028.conditon5 = s9_manned_exit.conditon5;
	blk_ccc_ofp_028.conditon6 = s9_manned_exit.conditon6;
}
//有人机领航退出
void send_blk_ccc_ofp_028()
{
	init_blk_ccc_ofp_028();
	data_length = sizeof(BLK_CCC_OFP_028);
	// 综显发送预览方案信息
	Send_Message(DDSTables.CCC_DPU_45.niConnectionId,0,&transaction_id,&(blk_ccc_ofp_028), &message_type_id, data_length, &enRetCode);
}

//无人机碰撞信息赋值
void init_blk_ccc_ofp_029()
{
	blk_ccc_ofp_029.uav_num = 2;
	if(s6_bd_flag == 1)
	{
		blk_ccc_ofp_029.display = 1;
	}
	else
	{
		blk_ccc_ofp_029.display = 0;
	}

	for(int i = 0 ; i < 2 ; i ++)
	{
		blk_ccc_ofp_029.uav_avoid[i].uav_id = formationId[i].planeId;
		blk_ccc_ofp_029.uav_avoid[i].uav_sn = i+1;
		blk_ccc_ofp_029.uav_avoid[i].uav_uav = s9_uav_avoid_flag[i];
		blk_ccc_ofp_029.uav_avoid[i].uav_manned = s9_man_avoid_flag[i];
	}

}
//无人机碰撞启动信息
void send_blk_ccc_ofp_029()
{
	init_blk_ccc_ofp_029();
	data_length = sizeof(BLK_CCC_OFP_029);
	// 综显发送预览方案信息
	Send_Message(DDSTables.CCC_DPU_46.niConnectionId,0,&transaction_id,&(blk_ccc_ofp_029), &message_type_id, data_length, &enRetCode);
}

//应急返航区域信息赋值
void init_blk_ccc_ofp_030()
{
	int num_cnt = 0;
	for(int i = 0 ;i < 4; i ++)
	{
		if(load_file.blk_dlr_ccc_045[i].emergency_num > 0)
		{
			num_cnt++;
		}
	}
	//无人机数量与预加载紧急航线数量一致
	blk_ccc_ofp_030.uav_num = num_cnt;

	for(int i = 0 ; i < 2 ; i ++)
	{
		blk_ccc_ofp_030.uav_emarea[i].uav_id = formationId[i].planeId;
		blk_ccc_ofp_030.uav_emarea[i].uav_sn = i+1;
		blk_ccc_ofp_030.uav_emarea[i].Lat = s9_emergence_area[i].Lat;
		blk_ccc_ofp_030.uav_emarea[i].Lon = s9_emergence_area[i].Lon;
		blk_ccc_ofp_030.uav_emarea[i].Angle = s9_emergence_area[i].Angle;
		blk_ccc_ofp_030.uav_emarea[i].Long = s9_emergence_area[i].Long;
		blk_ccc_ofp_030.uav_emarea[i].wide = s9_emergence_area[i].Wide;
		blk_ccc_ofp_030.uav_emarea[i].emergency_num = load_file.blk_dlr_ccc_045[i].emergency_num;
		memcpy(&blk_ccc_ofp_030.uav_emarea[i].emergency_point[0],&load_file.blk_dlr_ccc_045[i].normal_point[0],sizeof(Avoid_POINT)*25);
	}
}
//应急返航区域
void send_blk_ccc_ofp_030()
{
	init_blk_ccc_ofp_030();
	data_length = sizeof(BLK_CCC_OFP_030);
	// 综显发送预览方案信息
	Send_Message(DDSTables.CCC_DPU_47.niConnectionId,0,&transaction_id,&(blk_ccc_ofp_030), &message_type_id, data_length, &enRetCode);
}

void send_blk_ccc_ofp_019()
{
	data_length = sizeof(BLK_CCC_OFP_019);

	// 综显发送预览方案信息
	Send_Message(DDSTables.CCC_DPU_8.niConnectionId,0,&transaction_id,&blk_ccc_ofp_019, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//qDebug()<<"CCC_DPU_data_8_success!";
	}

	// 给pad发送
	if(Pad_heart_flag == 1)
	{
		Send_Message(DDSTables.CCC_PAD_019.niConnectionId,0,&transaction_id,&blk_ccc_ofp_019, &message_type_id, data_length, &enRetCode);
	}

}

void send_blk_ccc_ofp_019_special()
{
	BLK_CCC_OFP_019  tem_blk_ccc_ofp_019;
	data_length = sizeof(BLK_CCC_OFP_019);

	memcpy(&tem_blk_ccc_ofp_019, &blk_ccc_ofp_019, sizeof(blk_ccc_ofp_019));

	// 在非紧密无人机编队下，需要特殊处理发给ofp的信息
	castCtasToOfpPlan(&tem_blk_ccc_ofp_019);

	// 综显发送预览方案信息
	Send_Message(DDSTables.CCC_DPU_8.niConnectionId,0,&transaction_id,&tem_blk_ccc_ofp_019, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//qDebug()<<"CCC_DPU_data_8_success!";
	}

	// 给pad发送
	if(Pad_heart_flag == 1)
	{
		Send_Message(DDSTables.CCC_PAD_019.niConnectionId,0,&transaction_id,&tem_blk_ccc_ofp_019, &message_type_id, data_length, &enRetCode);
	}

	//KDL发送
	send_blk_ccc_kdl_017();
}

void scheme_generation_state(char fanganType, char fanganGenStatus, char fanganSubStatus, unsigned char hangxianGenStatus)
{
	// 给综显发送编辑方案状态信息
	CCC_DPU_data_0.fanganType = fanganType;//方案类型，区别攻击规划
	CCC_DPU_data_0.fanganGenStatus = fanganGenStatus;
	CCC_DPU_data_0.fanganSubStatus = fanganSubStatus;
	CCC_DPU_data_0.hangxianGenStatus = hangxianGenStatus;

	if(fanganType == 2)
	{
		printf("fanganGenStatus %d\n",fanganGenStatus);
	}

	data_length = sizeof(CCC_DPU_data_0);
	Send_Message(DDSTables.CCC_DPU_0.niConnectionId,0,&transaction_id, &CCC_DPU_data_0, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//        qDebug()<<"CCC_DPU_data_0_success!";
	}

	//发送给pad new20250620
	if(Pad_heart_flag == 1)
	{
		Send_Message(DDSTables.CCC_PAD_001.niConnectionId,0,&transaction_id, &CCC_DPU_data_0, &message_type_id, data_length, &enRetCode);
	}

}


//有人机航路信息发送
void send_buoy_soanr_route_information(unsigned int plan, int task)
{ // i 为无人机自任务

	//找到当前方案的保存下标
	if(blk_ccc_ofp_302_save[plan][task].Plan_ID != 0)
	{
		data_length = sizeof(BLK_CCC_OFP_302);
		// 转发给综显浮标布阵规划
		Send_Message(DDSTables.CCC_DPU_26.niConnectionId,0,&transaction_id, &blk_ccc_ofp_302_save[plan][task], &message_type_id, data_length, &enRetCode);

		//发送给pad new20250620
		if(Pad_heart_flag == 1)
		{
			Send_Message(DDSTables.CCC_PAD_302.niConnectionId,0,&transaction_id, &blk_ccc_ofp_302_save[plan][task], &message_type_id, data_length, &enRetCode);
		}

	}


	if(blk_ccc_ofp_403_save[plan][task].Plan_ID != 0)
	{
		data_length = sizeof(BLK_CCC_OFP_403);
		// 转发给综显吊声定测点规划
		Send_Message(DDSTables.CCC_DPU_27.niConnectionId,0,&transaction_id, &blk_ccc_ofp_403_save[plan][task], &message_type_id, data_length, &enRetCode);

		//发送给pad new20250620
		if(Pad_heart_flag == 1)
		{
			Send_Message(DDSTables.CCC_PAD_403.niConnectionId,0,&transaction_id, &blk_ccc_ofp_403_save[plan][task], &message_type_id, data_length, &enRetCode);
		}

	}
}

void send_blk_ccc_ofp_024_single(unsigned int plan,unsigned int single_index)
{
	BLK_CCC_OFP_024 temp;
	memset(&temp, 0, sizeof(BLK_CCC_OFP_024));
	//取出航线信息分包发送
	for(unsigned char j = 0 ;j < blk_ccc_ofp_024_cunchu[plan][single_index].individual_drone_routing_programs.planning_informations.total_packet;j++)
	{
		//取头
		memcpy(&temp,&blk_ccc_ofp_024_cunchu[plan][single_index],9 + 10 + 26);
		//取航路点
		memcpy(&temp.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], &blk_ccc_ofp_024_cunchu[plan][single_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j * 25], sizeof(planning_information_waypoint_information) * 25);
		//包序号赋值
		temp.individual_drone_routing_programs.planning_informations.packet_id = j;

		data_length = sizeof(BLK_CCC_OFP_024);
		Send_Message(DDSTables.CCC_DPU_11.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);

		//发送给pad new20250620
		if(Pad_heart_flag == 1)
		{
			Send_Message(DDSTables.CCC_PAD_024.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);
		}
		//发送任务系统
		Send_Message(DDSTables.CCC_DPM_7.niConnectionId,0,&transaction_id, &temp,&message_type_id, data_length, &enRetCode);

		// 空地链发送
		memcpy(&blk_ccc_ofp_024,&blk_ccc_ofp_024_cunchu[plan][single_index],sizeof(BLK_CCC_OFP_024));
		send_blk_ccc_kdl_024();
	}
}
// 无人机通航 航路点信息
void send_blk_ccc_ofp_024(unsigned int plan)
{
	//pad发送
	if (Pad_heart_flag == 1)
	{
		// 注 PAD_send_UDPsocket.writeDatagram(send_array,PAD_send_IPadress,PAD_send_Port);
	}
	else
	{
		// // printf("pad 未在线 无法发送";
	}
	if(should_sync_bdfx_double_plan_height(plan))
	{
		sync_bdfx_double_plan_height(plan);
	}
	for(int i = 0 ; i < 4 ; i ++)
	{
		BLK_CCC_OFP_024 temp;
		memset(&temp, 0, sizeof(BLK_CCC_OFP_024));
		//取出航线信息分包发送
		for(unsigned char j = 0 ;j < blk_ccc_ofp_024_cunchu[plan][i].individual_drone_routing_programs.planning_informations.total_packet;j++)
		{
			//取头
			memcpy(&temp,&blk_ccc_ofp_024_cunchu[plan][i],9 + 10 + 26);
			//取航路点
			memcpy(&temp.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], &blk_ccc_ofp_024_cunchu[plan][i].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j * 25], sizeof(planning_information_waypoint_information) * 25);
			//包序号赋值
			temp.individual_drone_routing_programs.planning_informations.packet_id = j;

			data_length = sizeof(BLK_CCC_OFP_024);
			Send_Message(DDSTables.CCC_DPU_11.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);

			//发送给pad new20250620
			if(Pad_heart_flag == 1)
			{
				Send_Message(DDSTables.CCC_PAD_024.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);
			}

			//发送任务系统（内部链路，保持旧逻辑）
			Send_Message(DDSTables.CCC_DPM_7.niConnectionId,0,&transaction_id, &temp,&message_type_id, data_length, &enRetCode);

			// 空地链发送
			memcpy(&blk_ccc_ofp_024,&blk_ccc_ofp_024_cunchu[plan][i],sizeof(BLK_CCC_OFP_024));
			send_blk_ccc_kdl_024();
		}
	}

}
void init_blk_ccc_ofp_025()
{
	blk_ccc_ofp_025.signal_CCC_now = global_stage;
}
//发送当前阶段反馈
void send_blk_ccc_ofp_025()
{
	init_blk_ccc_ofp_025();
	data_length = sizeof(BLK_CCC_OFP_025);
	Send_Message(DDSTables.CCC_DPU_28.niConnectionId,0,&transaction_id, &blk_ccc_ofp_025, &message_type_id, data_length, &enRetCode);
}

// 综显上线后 检测 将任务区任务点信息发送给综显 3.10 / 11
// 3.10 初始化任务区/空域信息
void init_blk_ccc_ofp_033()
{

	//初始化任务区数据
	memset(&blk_ccc_ofp_033,0,sizeof(BLK_CCC_OFP_033) * 2);
	//    unsigned int i;
	blk_ccc_ofp_033[0].curBagNo = 0;
	blk_ccc_ofp_033[0].bagTatal = 2;
	blk_ccc_ofp_033[0].area_number = 0;
	//
	//    for( i = 0; i < blk_ccc_ofp_033[0].area_number; i++){
	//        blk_ccc_ofp_033[0].area_informations[i].area_code = i+1;
	//
	//        blk_ccc_ofp_033[0].area_informations[i].area_source = 1; // 预规划加载
	//        //CCC_DPU_data_13.area_informations[i].drone_numbe = 0x1003+i;
	//        blk_ccc_ofp_033[0].area_informations[i].upper_height_limit_valid_bit = 1;
	//        blk_ccc_ofp_033[0].area_informations[i].lower_height_limit_valid_bit = 1;
	//        blk_ccc_ofp_033[0].area_informations[i].upper_height_limit = 1600;
	//        blk_ccc_ofp_033[0].area_informations[i].lower_height_limit = 1200;
	//    }
	//    blk_ccc_ofp_033[0].area_informations[0].area_type = 1; // 任务区  圆
	//    blk_ccc_ofp_033[0].area_informations[0].drone_numbe = 1;
	//
	//    blk_ccc_ofp_033[0].area_informations[1].area_type = 1; // 任务区   圆
	//    blk_ccc_ofp_033[0].area_informations[1].drone_numbe = 2;  // 空域所属无人机序号
	//
	//    blk_ccc_ofp_033[0].area_informations[0].area_shape = 1; // 1 圆形 2 多边形
	//    blk_ccc_ofp_033[0].area_informations[0].cycles.radius = 5;
	//        blk_ccc_ofp_033[0].area_informations[0].cycles.cycle_longitude_and_latitude.latitude = plane_lat + 0.1; // 增大 上移动
	//        blk_ccc_ofp_033[0].area_informations[0].cycles.cycle_longitude_and_latitude.longitude = plane_lon + 0.025;
	//
	//    // 任务区
	//    blk_ccc_ofp_033[0].area_informations[1].area_shape = 1; // 圆形
	//    blk_ccc_ofp_033[0].area_informations[1].cycles.radius = 8;
	//    blk_ccc_ofp_033[0].area_informations[1].cycles.cycle_longitude_and_latitude.latitude = plane_lat + 0.002;
	//    blk_ccc_ofp_033[0].area_informations[1].cycles.cycle_longitude_and_latitude.longitude = plane_lon + 0.07;
	//    //      CCC_DPU_data_13.area_informations[2].cycles.cycle_longitude_and_latitude.latitude = 10.3;
	//    //      CCC_DPU_data_13.area_informations[2].cycles.cycle_longitude_and_latitude.longitude = 100.3;
	//
	blk_ccc_ofp_033[1].curBagNo = 1;
	blk_ccc_ofp_033[1].bagTatal = 2;
	blk_ccc_ofp_033[1].area_number = 0;


	//初始化空域
	memset(&air_area,0,sizeof(BLK_CCC_OFP_033));

}

void send_blk_ccc_ofp_033()
{
	// send_array.clear();
	//第一包


	// 综显发送
	data_length = sizeof(BLK_CCC_OFP_033);
	Send_Message(DDSTables.CCC_DPU_13.niConnectionId,0,&transaction_id, &(blk_ccc_ofp_033[0].curBagNo), &message_type_id, data_length, &enRetCode);

	//发送给pad new20250620
	if(Pad_heart_flag == 1)
	{
		Send_Message(DDSTables.CCC_PAD_033.niConnectionId,0,&transaction_id, &(blk_ccc_ofp_033[0]), &message_type_id, data_length, &enRetCode);
	}


	//第二包

	//空域有效性判断
	for(int i = 0 ; i < 4 ; i ++)
	{
		if(formationId[i].C_U == 0)
		{
			blk_ccc_ofp_033[1].area_informations[i].upper_height_limit_valid_bit = 0;
		}
		else
		{
			blk_ccc_ofp_033[1].area_informations[i].upper_height_limit_valid_bit = 1;
		}
	}

	// 综显发送
	data_length = sizeof(BLK_CCC_OFP_033);
	Send_Message(DDSTables.CCC_DPU_13.niConnectionId,0,&transaction_id, &(blk_ccc_ofp_033[1].curBagNo), &message_type_id, data_length, &enRetCode);

	//发送给pad new20250620
	if(Pad_heart_flag == 1)
	{
		Send_Message(DDSTables.CCC_PAD_033.niConnectionId,0,&transaction_id, &(blk_ccc_ofp_033[1]), &message_type_id, data_length, &enRetCode);
	}

	// 给kdl发送
	send_blk_ccc_kdl_033();

}

// 3.11 任务点信息
void init_blk_ccc_ofp_034()
{

	//    blk_ccc_ofp_034.point_number = 7;
	//    unsigned int i;
	//    for(i = 0; i < blk_ccc_ofp_034.point_number; i++){
	//        blk_ccc_ofp_034.point_informations[i].point_number = i+1; // 点编号
	//        blk_ccc_ofp_034.point_informations[i].latitude = plane_lat+0.01*i;//35.8967
	//        blk_ccc_ofp_034.point_informations[i].longitude = plane_lon + 0.01*i;//115.5698
	//        blk_ccc_ofp_034.point_informations[i].point_property = 3;// 中立点
	//        blk_ccc_ofp_034.point_informations[i].point_type = 3; // 陆地
	//        blk_ccc_ofp_034.point_informations[i].point_source_of_information = 1; // 来源
	//    }
	memset(&blk_ccc_ofp_034,0,sizeof(BLK_CCC_OFP_034));
}

void send_blk_ccc_ofp_034()
{

	data_length = sizeof(BLK_CCC_OFP_034);
	Send_Message(DDSTables.CCC_DPU_14.niConnectionId,0,&transaction_id, &blk_ccc_ofp_034.point_number, &message_type_id, data_length, &enRetCode);

	//发送给pad new20250620
	if(Pad_heart_flag == 1)
	{
		Send_Message(DDSTables.CCC_PAD_034.niConnectionId,0,&transaction_id, &blk_ccc_ofp_034.point_number,&message_type_id, data_length, &enRetCode);
	}

	// 给kdl发送
	send_blk_ccc_kdl_034();
}

// 3.12 任务线信息
void init_blk_ccc_ofp_035()
{
	blk_ccc_ofp_035.line_number = 3;
	for (int i = 0; i < blk_ccc_ofp_035.line_number; i++)
	{
		blk_ccc_ofp_035.line_informations[i].line_num = i+1;
		blk_ccc_ofp_035.line_informations[i].line_type = 2;
		blk_ccc_ofp_035.line_informations[i].line_point_number = 3;
		for (int j = 0; j < blk_ccc_ofp_035.line_informations[i].line_point_number; j++)
		{
			blk_ccc_ofp_035.line_informations[i].point_coordinates[j].latitude = 36.8911 + 0.01*i;
			blk_ccc_ofp_035.line_informations[i].point_coordinates[j].longitude = 112.5678 + 0.01*i;
		}
	}
}
void send_blk_ccc_ofp_035()
{
	// send_array.clear();
	init_blk_ccc_ofp_035();
	//pad发送
	if (Pad_heart_flag == 1)
	{
		// 注 PAD_send_UDPsocket.writeDatagram(send_array,PAD_send_IPadress,PAD_send_Port);
	}
	else
	{
		//// printf("pad downline fail send\n");
	}
	// 综显发送
	data_length = sizeof(BLK_CCC_OFP_035);
	Send_Message(DDSTables.CCC_DPU_15.niConnectionId,0,&transaction_id, &blk_ccc_ofp_035.line_number , &message_type_id, data_length, &enRetCode);
	//发送给pad new20250620
	//	Send_Message(DDSTables.CCC_PAD_035.niConnectionId,0,&transaction_id, &blk_ccc_ofp_035.line_number,&message_type_id, data_length, &enRetCode);

	send_blk_ccc_kdl_035();

}

// 3.13 协同指控指令信息
void init_blk_ccc_ofp_036(unsigned short command_type,unsigned short state,unsigned short task_type)
{
	//指令序号
	static unsigned short code_id = 1;
	op_code code;
	memset(&code,0,sizeof(op_code));
	code.command_id = code_id;
	code.command_type = command_type;/*指令类型  0=NA;1=重规划;2=方案规划;3=控制权交接;*/
	code.command_send_time_vaild = 1;
	code.command_send_time = DPU_CCC_data_4.time_lable;//导航时间,取值方式未定
	code.receive_state = 1;//暂时不清楚接收失败怎么判断
	code.execute_state = state;/*执行状态  0=NA;1=已执行;2=未执行;*/
	/*内容	0=NA;1=光电搜索跟踪;2=雷达搜索跟踪;3=吊声搜索;4=浮标布放;5=浮标侦听;
	 * 6=磁探搜索;7=磁探跟踪;8=通信中继;9=攻击;10=起飞过渡;11=返航;12=等待;
	 * 13=临时改航;14=预规划A;15=预规划B;16=控制权申请;17=控制权释放;18=检查反潜;19=应召反潜;*/
	code.task_type = task_type;
	enqueue(&queue,&code);
	printf("code enqueue success \n");
	code_id++;
}
void send_blk_ccc_ofp_036()
{
	//发送
	data_length = sizeof(BLK_CCC_OFP_036);
	Send_Message(DDSTables.CCC_DPU_16.niConnectionId,0,&transaction_id, &blk_ccc_ofp_036, &message_type_id, data_length, &enRetCode);
	//发送给pad new20250620
	//    Send_Message(DDSTables.CCC_PAD_036.niConnectionId,0,&transaction_id, &blk_ccc_ofp_036,&message_type_id, data_length, &enRetCode);

}

// 接收重规划消息
void recv_blk_ofp_ccc_154()
{
	// 接收重规划消息才开始处理
	recv_dpu1_dpu2(DDSTables.DPU_CCC_154.niConnectionId,
			DDSTables.DPU2_CCC_154.niConnectionId, &blk_ofp_ccc_154,
			sizeof blk_ofp_ccc_154);
	if (enRetCode == 0)
	{
		if (blk_ofp_ccc_154.plan_query == 1)
		{
			for(int i=0;i<4;i++)
			{
				if (g_lineCrashState[i].hasConflict == 1)
				{
					g_lineCrashState[i].ofpLIneCodeCount++;
					g_lineCrashState[i].recv_fabuCode = 1;// 进入第一阶段发送
					g_lineCrashState[i].ofpLineCode = 1;
				}
			}

			// 发布中提示
			memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
			blk_ccc_ofp_032.tips_type = 21;
			send_blk_ccc_ofp_032();
		}

		if (blk_ofp_ccc_154.plan_query == 2)
		{
			for(int i=0;i<4;i++)
			{
				if (g_lineCrashState[i].hasConflict == 1)
				{
					g_lineCrashState[i].ofpLIneCodeCount++;
					g_lineCrashState[i].recv_fabuCode = 1;// 进入第一阶段发送
					g_lineCrashState[i].ofpLineCode = 2;
				}
			}

			// 弹框消隐
			memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
			blk_ccc_ofp_032.tips_type = 0;
			send_blk_ccc_ofp_032();
		}

		if (blk_ofp_ccc_154.plan_query == 3)
		{
			for(int i=0;i<4;i++)
			{
				if (g_lineCrashState[i].hasConflict == 1)
				{
					g_lineCrashState[i].crashOffConfirm = 1;
					g_lineCrashState[i].recv_fabuCode = 2;// 冲突状态第二次发布
				}
			}
			// 发布中提示
			memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
			blk_ccc_ofp_032.tips_type = 21;
			send_blk_ccc_ofp_032();
		}

		if (blk_ofp_ccc_154.plan_query == 4)
		{
			for(int i=0;i<4;i++)
			{
				memset(&g_lineCrashState[i], 0, sizeof(g_lineCrashState[i]));
			}

			// 弹框消隐
			memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
			blk_ccc_ofp_032.tips_type = 0;
			send_blk_ccc_ofp_032();
		}

		if (blk_ofp_ccc_154.plan_query == 5)
		{
			for(int i=0;i<4;i++)
			{
				memset(&g_lineCrashState[i], 0, sizeof(g_lineCrashState[i]));
			}
			unsigned int plan = blk_ofp_ccc_038.Plan_ID % 3;
			// 使用备份无人机航路点
			for(int uav = 0; uav < UAV_MAX_NUM; uav ++)
			{
				memcpy(&blk_ccc_ofp_024_cunchu[plan][uav] , &g_lineCrashUavBak[uav] , sizeof(g_lineCrashUavBak[uav]));
			}
			//使用备份的任务分配结果，赋回原值
			memcpy(&CCC_DPU_data_6_Ofp[plan],&g_lineCrashPlanBak , sizeof(g_lineCrashPlanBak));
			// 弹框消隐
			memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
			blk_ccc_ofp_032.tips_type = 0;
			send_blk_ccc_ofp_032();
		}

		if (blk_ofp_ccc_154.plan_query == 6)
		{
			for(int i=0;i<4;i++)
			{
				memset(&g_lineCrashState[i], 0, sizeof(g_lineCrashState[i]));
			}
			// 弹框消隐
			memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
			blk_ccc_ofp_032.tips_type = 0;
			send_blk_ccc_ofp_032();
		}

		if (blk_ofp_ccc_154.plan_query == 11)
		{
			//执行载荷重规划
			for(int i=0;i<4;i++)
			{
				if(g_payload_replan[i].Replan == 1)
				{
					//发送任务类型和任务区信息
					memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
					blk_ccc_ofp_032.subtask_type = 5;
					blk_ccc_ofp_032.tips_type = 32;
					unsigned int plan = blk_ofp_ccc_038.Plan_ID % 3;
					blk_ccc_ofp_032.subtask_point_or_area =
							CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[i+1].task_sequence_informations[global_stage-1].point_or_area;
					blk_ccc_ofp_032.subtask_point_or_area_id =
							CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[i+1].task_sequence_informations[global_stage-1].point_or_area_id;
					send_blk_ccc_ofp_032();
				}
			}
		}

		if (blk_ofp_ccc_154.plan_query == 12)
		{
			//忽略载荷重规划
			for(int i=0;i<4;i++)
			{
				if(g_payload_replan[i].Replan == 1)
				{
					g_payload_replan[i].Replan = 2;
				}
			}
			// 弹框消隐
			memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
			blk_ccc_ofp_032.tips_type = 0;
			send_blk_ccc_ofp_032();
		}
	}

	return;
}

//接收综显指令查询指令
void recv_blk_ofp_ccc_037()
{
	message_size = RECV_MAX_SIZE;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_22.niConnectionId,DDSTables.DPU2_CCC_22.niConnectionId,&blk_ofp_ccc_037,sizeof blk_ofp_ccc_037);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//    	Receive_Message(DDSTables.PAD_CCC_037.niConnectionId, 0, &transaction_id, &blk_ofp_ccc_037, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode == 0)
	{
		//查询信息赋值
		blk_ccc_ofp_036.mfdNo = blk_ofp_ccc_037.mfdNo;
		blk_ccc_ofp_036.PageIdCurrent = blk_ofp_ccc_037.QueryCommand;
		blk_ccc_ofp_036.PageIdTotle = queue.count / 10;
		//最小为一页
		if(blk_ccc_ofp_036.PageIdTotle == 0)
			blk_ccc_ofp_036.PageIdTotle = 1;
		blk_ccc_ofp_036.UAV_Serial_Number1_5 = blk_ofp_ccc_037.UAV_Serial_Number1;
		blk_ccc_ofp_036.UAV_ID_2_2_1 = blk_ofp_ccc_037.UAV_ID2_2;
		blk_ccc_ofp_036.command_num = queue.count;
		memcpy(&blk_ccc_ofp_036.xt_commands[0],&queue.buffer[(blk_ofp_ccc_037.QueryCommand-1)*10],sizeof(op_code) * 10);
		//发送指令集信息
		send_blk_ccc_ofp_036();
	}
}

// 3.14 预规划加载结果确认    sg_sttaConDirTopicId[32]

// 3.15 预规划加载结果确认
void init_confirmation_of_preplanning_loading_results()
{
	//初始化confirmation_of_preplanning_loading_results
}
void send_confirmation_of_preplanning_loading_results()
{
	// send_array.clear();
	align_send_information(&CCC_DPU_data_17,sizeof(CCC_DPU_data_17),0);
	//发送
	Send_Message(DDSTables.CCC_DPU_17.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	//发送给pad new20250620
	//	Send_Message(DDSTables.CCC_PAD_037.niConnectionId,0,&transaction_id, send_array.dataA,&message_type_id, data_length, &enRetCode);

}

// 3.16 预规划方案
void init_pre_planning_program()
{
	//初始化pre_planning_program
}
void send_pre_planning_program()
{
	// send_array.clear();
	align_send_information(&CCC_DPU_data_18,sizeof(CCC_DPU_data_18),0);
	//发送
	Send_Message(DDSTables.CCC_DPU_18.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	//发送给pad new20250620
	//	Send_Message(DDSTables.CCC_PAD_038.niConnectionId,0,&transaction_id, send_array.dataA,&message_type_id, data_length, &enRetCode);

}

// 3.17 辅助决策错误提示
void init_assisted_decision_making_error_alerts()
{
	//初始化assisted_decision_making_error_alerts
}
void send_assisted_decision_making_error_alerts()
{
	// // send_array.clear();
	align_send_information(&CCC_DPU_data_19,sizeof(CCC_DPU_data_19),0);
	//发送
	Send_Message(DDSTables.CCC_DPU_19.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	//发送给pad new20250620
	//	Send_Message(DDSTables.CCC_PAD_039.niConnectionId,0,&transaction_id, send_array.dataA,&message_type_id, data_length, &enRetCode);

}

//3.18 协同指控系统自检测故障清单
void init_default_orders()
{
	//初始化assisted_decision_making_error_alerts
}
void send_default_orders()
{
	// // send_array.clear();
	align_send_information(&CCC_DPU_data_20,sizeof(CCC_DPU_data_20),0);
	//发送
	Send_Message(DDSTables.CCC_DPU_20.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	//发送给pad new20250620
	//	Send_Message(DDSTables.CCC_PAD_040.niConnectionId,0,&transaction_id, send_array.dataA,&message_type_id, data_length, &enRetCode);

}

// 新增信息
//无人机光电控制权反馈
void init_guangdian_kongzhi_feedback()
{
	//初始化assisted_decision_making_error_alerts
}
void send_guangdian_kongzhi_feedback()
{
	// // send_array.clear();
	align_send_information(&CCC_DPU_data_21,sizeof(CCC_DPU_data_21),0);
	//发送
	Send_Message(DDSTables.CCC_DPU_21.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	//发送给pad new20250620
	//	Send_Message(DDSTables.CCC_PAD_041.niConnectionId,0,&transaction_id, send_array.dataA,&message_type_id, data_length, &enRetCode);


}

// 控制权限交接
void init_kongzhiquan_jiaojie()
{
	//初始化assisted_decision_making_error_alerts
}
void send_kongzhiquan_jiaojie()
{
	// // send_array.clear();
	align_send_information(&(CCC_DPU_data_1),sizeof(CCC_DPU_data_1),0);
	//发送
	Send_Message(DDSTables.CCC_DPU_1.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	//发送给pad new20250620
	//	Send_Message(DDSTables.CCC_PAD_002.niConnectionId,0,&transaction_id, &send_array.dataA, &message_type_id, data_length, &enRetCode);
}


/****************************************内部通信发送接口(战法规划测试完成)**********************************/
// 发 DTMS_CTAS 相关辅助函数
// 战术战法推荐规划指令
void init_zhanshutuijian()
{

	//初始化
	memset(&blk_dtms_ctas_001, 0 , sizeof(BLK_DTMS_CTAS_001));

	blk_dtms_ctas_001.solider_num = 1 + CCC_DPU_data_3.drone_number;   //todo：按有人机和无人机总数量赋值
	blk_dtms_ctas_001.solider_infos[0].solider_type = 1; // 类别  1 有人 2 无人
	blk_dtms_ctas_001.solider_infos[0].solider_id = MANNED_ID;//todo:目前有人机id写固定值：9001
	blk_dtms_ctas_001.solider_infos[0].lon_lat_info.latitude =DPU_CCC_data_4.manned_longitude_and_latitude_synt.latitude;
	blk_dtms_ctas_001.solider_infos[0].lon_lat_info.longitude = DPU_CCC_data_4.manned_longitude_and_latitude_synt.longitude;
	blk_dtms_ctas_001.solider_infos[0].speed = DPU_CCC_data_4.groundspeed;
	blk_dtms_ctas_001.solider_infos[0].height = DPU_CCC_data_4.absolute_barometric_altitude;
	blk_dtms_ctas_001.solider_infos[0].hangxiang = DPU_CCC_data_4.true_direction;

	// 因为无人机信息会出现非紧密存储，所以需要跳过无效无人机信息传递无人机信息
	int temUavNum = 0;
	for(int i = 0 ; i < 4 ; i ++ )
	{
		if(CCC_DPU_data_3.drone_specific_informations[i].platform_num == 0)
		{
			continue;
		}

		blk_dtms_ctas_001.solider_infos[temUavNum + 1].solider_type = 2;
		blk_dtms_ctas_001.solider_infos[temUavNum + 1].solider_id = CCC_DPU_data_3.drone_specific_informations[i].platform_num;
		blk_dtms_ctas_001.solider_infos[temUavNum + 1].speed = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.ground_speed;
		blk_dtms_ctas_001.solider_infos[temUavNum + 1].height = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_height;
		blk_dtms_ctas_001.solider_infos[temUavNum + 1].hangxiang = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_heading;
//		if(g_payload_replan[i].CT_Online == 1 && g_payload_replan[i].CT_Status == 1)
			blk_dtms_ctas_001.solider_infos[temUavNum + 1].ct = 1;
		blk_dtms_ctas_001.solider_infos[temUavNum + 1].lon_lat_info.latitude = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_lati;
		blk_dtms_ctas_001.solider_infos[temUavNum + 1].lon_lat_info.longitude = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_longi;
		temUavNum++;
	}

	blk_dtms_ctas_001.task_type = DPU_CCC_data_7.mission_type;//规划类型
	blk_dtms_ctas_001.target_type = DPU_CCC_data_7.target_category;
	blk_dtms_ctas_001.zhanshu_sel = DPU_CCC_data_7.tactical_warfare_options;
	blk_dtms_ctas_001.task_reg_num = DPU_CCC_data_7.area_point_num; // 任务区数量

	for(int i = 0;i < blk_dtms_ctas_001.task_reg_num ;i++)
	{
		//检查反潜
		if(DPU_CCC_data_7.mission_type == 0)
		{
			blk_dtms_ctas_001.region_infos[i].reg_id = DPU_CCC_data_18_19.area_settings.area_informations[i].area_code;
			blk_dtms_ctas_001.region_infos[i].reg_serlize_id = i+1;
			blk_dtms_ctas_001.region_infos[i].reg_sour = DPU_CCC_data_18_19.area_settings.area_informations[i].area_source;
			blk_dtms_ctas_001.region_infos[i].reg_type = DPU_CCC_data_18_19.area_settings.area_informations[i].area_type;
			blk_dtms_ctas_001.region_infos[i].reg_shape = DPU_CCC_data_18_19.area_settings.area_informations[i].area_shape;
			switch (blk_dtms_ctas_001.region_infos[i].reg_shape)
			{
			case 1:
			{ // 圆形
				blk_dtms_ctas_001.region_infos[i].reg_circle.radious = DPU_CCC_data_18_19.area_settings.area_informations[i].cycles.radius;
				blk_dtms_ctas_001.region_infos[i].reg_circle.center_lon_lat.latitude = DPU_CCC_data_18_19.area_settings.area_informations[i].cycles.cycle_longitude_and_latitude_synt.latitude;
				blk_dtms_ctas_001.region_infos[i].reg_circle.center_lon_lat.longitude = DPU_CCC_data_18_19.area_settings.area_informations[i].cycles.cycle_longitude_and_latitude_synt.longitude;
			}
			break;
			case 2:
			{ // 多边形
				blk_dtms_ctas_001.region_infos[i].reg_ploygen.point_num = DPU_CCC_data_18_19.area_settings.area_informations[i].polygonals.point_number;
				for (int j = 0; j < blk_dtms_ctas_001.region_infos[i].reg_ploygen.point_num; j++)
				{
					blk_dtms_ctas_001.region_infos[i].reg_ploygen.points_lon_lat[j].latitude = DPU_CCC_data_18_19.area_settings.area_informations[i].polygonals.point_coordinates[j].latitude;
					blk_dtms_ctas_001.region_infos[i].reg_ploygen.points_lon_lat[j].longitude = DPU_CCC_data_18_19.area_settings.area_informations[i].polygonals.point_coordinates[j].longitude;
				}
			}
			break;

			}
			blk_dtms_ctas_001.region_infos[i].reg_top_of_hei_valid = DPU_CCC_data_18_19.area_settings.area_informations[i].upper_height_limit_valid_bit;
			blk_dtms_ctas_001.region_infos[i].reg_down_of_hei_valid = DPU_CCC_data_18_19.area_settings.area_informations[i].lower_height_limit_valid_bit;
			blk_dtms_ctas_001.region_infos[i].kongyu_belong_to_uav_valid = 1;    //对辅助决策无影响
			blk_dtms_ctas_001.region_infos[i].top_of_hei = DPU_CCC_data_18_19.area_settings.area_informations[i].upper_height_limit;
			blk_dtms_ctas_001.region_infos[i].down_of_hei = DPU_CCC_data_18_19.area_settings.area_informations[i].lower_height_limit;
		}
		//应召反潜
		else if(DPU_CCC_data_7.mission_type == 1)
		{
			// 任务点信息
			blk_dtms_ctas_001.region_infos[i].task_point_num = DPU_CCC_data_18_19.point_settings.number_of_point_to_be_modified;
			if (blk_dtms_ctas_001.region_infos[i].task_point_num > 0)
			{
				for (int k = 0; k < blk_dtms_ctas_001.region_infos[i].task_point_num; k++)
				{
					blk_dtms_ctas_001.region_infos[i].point_infos[k].point_serlize_id = DPU_CCC_data_18_19.point_settings.point_informations[k].point_number;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].point_id = DPU_CCC_data_18_19.point_settings.point_informations[k].point_number;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].point_source = DPU_CCC_data_18_19.point_settings.point_informations[k].point_source_of_information;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].point_lon_lat.latitude = DPU_CCC_data_18_19.point_settings.point_informations[k].latitude;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].point_lon_lat.longitude = DPU_CCC_data_18_19.point_settings.point_informations[k].longitude;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].point_speed_valid = DPU_CCC_data_18_19.point_settings.point_informations[k].point_speed_effective;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].point_hangxiang_valid = DPU_CCC_data_18_19.point_settings.point_informations[k].point_direction_effective;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].point_time_valid = DPU_CCC_data_18_19.point_settings.point_informations[k].point_time_effective;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].target_stats_valid = DPU_CCC_data_18_19.point_settings.point_informations[k].point_property_effective;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].target_type_valid = DPU_CCC_data_18_19.point_settings.point_informations[k].point_type_effective;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].target_speed = DPU_CCC_data_18_19.point_settings.point_informations[k].speed;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].target_hangxiang = DPU_CCC_data_18_19.point_settings.point_informations[k].direction;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].year = DPU_CCC_data_18_19.point_settings.point_informations[k].year;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].month = DPU_CCC_data_18_19.point_settings.point_informations[k].month;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].day = DPU_CCC_data_18_19.point_settings.point_informations[k].day;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].secends = DPU_CCC_data_18_19.point_settings.point_informations[k].millisecond;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].target_type = DPU_CCC_data_18_19.point_settings.point_informations[k].point_type;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].target_stats = DPU_CCC_data_18_19.point_settings.point_informations[k].point_property;
					blk_dtms_ctas_001.region_infos[i].point_infos[k].pitch = DPU_CCC_data_18_19.point_settings.point_informations[k].point_batchNumber;
				}
			}
		}
	}

}
void send_zhanshutuijian()
{
	//没有无人机返回失败
	if(CCC_DPU_data_3.drone_number == 0)
	{
		sprintf(CCC_DPU_data_0.failreason,"没有无人机在线");
		scheme_generation_state(1,3,0,0);// 返回方案编辑状态到综显
		memset(CCC_DPU_data_0.failreason,0,200);
	}
	else
	{
		data_length = sizeof(blk_dtms_ctas_001);
		Send_Message(DDSTables.BLK_DTMS_CTAS_001.niConnectionId,0,&transaction_id, &blk_dtms_ctas_001, &message_type_id, data_length, &enRetCode);
		if(enRetCode == 0)
		{
			//        pritnf("blk_dtms_ctas_001\n");
		}
	}
}

void send_single_mission_zhanshutuijian()
{

	data_length = sizeof(blk_dtms_ctas_002);
	Send_Message(DDSTables.BLK_DTMS_CTAS_002.niConnectionId,0,&transaction_id, &blk_dtms_ctas_002, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//        pritnf("blk_dtms_ctas_002;\n");
	}
}

void send_single_uav_zhanshutuijian()
{
	data_length = sizeof(blk_dtms_ctas_002);
	Send_Message(DDSTables.BLK_DTMS_CTAS_002.niConnectionId,0,&transaction_id, &blk_dtms_ctas_002, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//        pritnf("blk_dtms_ctas_002;\n");
	}
}

// 周期接收传感器航迹预测信息
void receive_forecast_target_hj_sensor()
{
	// 需确定具体 topicid
	//    target_hj_data_rece.resize(10000); // 设置信息暂存空间
	// Receive_Message(需增加, 0, &transaction_id, target_hj_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(forecast_target_hj_info),target_hj_data_rece.dataA,sizeof(forecast_target_hj_info));
		// // // printf("success_receive 目标航迹预测";
	}
	else
	{
		// // // printf("未接收信息！！！";
	}
}

// 周期将预测航迹信息发给pad
void send_forecast_target_hj_pad()
{
	// 直接转发即可，不用进行别处理，故使用统一结构体收发信息
	align_send_information(&(forecast_target_hj_info),sizeof(forecast_target_hj_info),0);
	// Send_Message(需增加,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}



/********************************************** 规划模块 **************************************/
/*
 * 接收综显全局规划指令 + 任务区   处理为 战法推荐指令发给辅助决策 ，
 * 接收辅助决策返回的规划方案+航路点  处理为 任务分配结果信息 + 通用航路/浮标/吊声 信息发给综显
 */
void formulate_moduel()
{
	// 4.9 全局任务规划命令  收到指令 赋值给战法规划指令3.1.1
	message_size = 10000;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_7.niConnectionId,DDSTables.DPU2_CCC_7.niConnectionId,&DPU_CCC_data_7,sizeof DPU_CCC_data_7);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		Receive_Message(DDSTables.PAD_CCC_019.niConnectionId, 0, &transaction_id, &DPU_CCC_data_7.mission_type, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode == 0)
	{
		formulate_single = 0;
		task_over_cnt = 0;
		//清空有人机和无人机的航路存储
		memset(&blk_ofp_ccc_018,0,sizeof(BLK_CCC_OFP_018));
		memset(&blk_ccc_ofp_018_cunchu[0][0] , 0 , 3 * 8 * sizeof(BLK_CCC_OFP_018_CunChu));
		memset(&blk_ccc_ofp_024_cunchu[0][0] , 0 , 3 * 4 * sizeof(BLK_CCC_OFP_024_cunchu));
		//清空浮标布阵规划和吊声定测规划存储
		memset(&blk_ccc_ofp_302_save[0][0] , 0 , 3 * 8 * sizeof(BLK_CCC_OFP_302));
		memset(&blk_ccc_ofp_403_save[0][0] , 0 , 3 * 8 * sizeof(BLK_CCC_OFP_403));
		//清空规划方案
		memset(&CCC_DPU_data_6,0,sizeof(BLK_CCC_OFP_019)*3);
		//阶段数置位
		global_stage = 1;
		//方案计数初始化
		if(DPU_CCC_data_7.mission_type == 0)
		{
			formulate_flag = 1;  //检查反潜
			plan_count = 0;
			plan_task_type = 18;
			init_blk_ccc_ofp_036(2,2,plan_task_type);
		}
		else if(DPU_CCC_data_7.mission_type == 1)
		{
			formulate_flag = 2;  //应召反潜
			plan_count = 0;
			plan_task_type = 19;
			init_blk_ccc_ofp_036(2,2,plan_task_type);
		}
		else if(DPU_CCC_data_7.mission_type == 2)
		{
			//预规划A
			//发送加载文件数据
			send_load_file(0);
		}
		else if(DPU_CCC_data_7.mission_type == 3)
		{
			//预规划B
			//发送加载文件数据
			send_load_file(1);
		}

		// 清空重规划标志量，（避免发生冲突，用户重新规划问题）
		memset(&g_lineCrashState, 0, sizeof(g_lineCrashState)*4);

		//载荷重规划标志位重置
		for(int i = 0 ; i < 4 ; i ++)
		{
			g_payload_replan[i].Replan = 0;
		}

	}

	message_size = 10000;
	// 4.8.1 任务区设置
	recv_dpu1_dpu2(DDSTables.DPU_CCC_18.niConnectionId,DDSTables.DPU2_CCC_18.niConnectionId,&DPU_CCC_data_18_19.area_settings,sizeof(area_setting));
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		Receive_Message(DDSTables.PAD_CCC_033.niConnectionId, 0, &transaction_id, &DPU_CCC_data_18_19.area_settings.number_of_areas_to_be_modified, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode == 0)
	{
		if(formulate_flag == 1 )
		{   //检查反潜
			init_zhanshutuijian();  // 初始化战法规划指令信息 (全局规划指令时)
			send_zhanshutuijian();  // 触发规划后只发送一次战法规划指令给 CTAS
			scheme_generation_state(1,1,0,0);// 返回方案编辑状态到综显
			formulate_flag = 0;
		}
		//增加任务区
		if (DPU_CCC_data_18_19.area_settings.number_of_areas_to_be_modified >= 1 && DPU_CCC_data_18_19.area_settings.area_informations[0].reg_opetour == 1)
		{
			//任务区增加已满
			if(blk_ccc_ofp_033[0].area_number == 8 || (blk_ccc_ofp_033[0].area_number >= 7 && blk_ccc_ofp_033[0].area_informations[7].area_code == 0))
			{
				//返回失败
			}
			else
			{
				//查询任务区是否还有空位
				for(int i = 0 ; i < 7 ; i++)
				{
					if(blk_ccc_ofp_033[0].area_informations[i].area_code == 0)
					{
						memcpy(&blk_ccc_ofp_033[0].area_informations[i],
								&DPU_CCC_data_18_19.area_settings.area_informations[0].area_code,
								sizeof(area_information));
						blk_ccc_ofp_033[0].area_informations[i].area_code = i+1;
						blk_ccc_ofp_033[0].area_number++;
						break;
					}
				}
			}


		}
		//删除任务区
		else if (DPU_CCC_data_18_19.area_settings.number_of_areas_to_be_modified >= 1 && DPU_CCC_data_18_19.area_settings.area_informations[0].reg_opetour == 3)
		{
			unsigned int area_num = DPU_CCC_data_18_19.area_settings.area_informations[0].area_code;
			if(area_num <= 8 && area_num != 0)
			{
				memset(&blk_ccc_ofp_033[0].area_informations[area_num - 1] , 0 , sizeof(area_information));
				blk_ccc_ofp_033[0].area_number--;
			}
			else if(area_num > 8 && area_num != 0)
			{
				memset(&blk_ccc_ofp_033[1].area_informations[area_num - 1 - 8] , 0 , sizeof(area_information));
				blk_ccc_ofp_033[1].area_number--;
			}
		}
	}

	if(formulate_flag == 2 )
	{   //应召反潜
		message_size = 10000;
		// 应召点设置
		recv_dpu1_dpu2(DDSTables.DPU_CCC_19.niConnectionId,DDSTables.DPU2_CCC_19.niConnectionId,&DPU_CCC_data_18_19.point_settings,sizeof(point_setting_confirm));
		//收不到就收PAD new20250620
		if(enRetCode != 0)
		{
			Receive_Message(DDSTables.PAD_CCC_034.niConnectionId, 0, &transaction_id, &DPU_CCC_data_18_19.point_settings.number_of_point_to_be_modified, &message_type_id, &message_size, &enRetCode);
		}
		if(enRetCode == 0)
		{
			init_zhanshutuijian();  // 初始化战法规划指令信息 (全局规划指令时)
			send_zhanshutuijian();  // 触发规划后只发送一次战法规划指令给 CTAS
			scheme_generation_state(1,1,0,0);// 返回方案编辑状态到综显
			formulate_flag = 0;
		}
	}

}

/************************************************* 重规划模块 ***********************************************/
//编辑状态发送
void send_blk_ccc_ofp_020(unsigned int Plan_ID,unsigned char ModifyType,unsigned char modify_state,unsigned char * failreason)
{
	//赋值
	blk_ccc_ofp_020.Plan_ID = Plan_ID;
	blk_ccc_ofp_020.ModifyType = ModifyType;
	blk_ccc_ofp_020.modify_state = modify_state;
	if(failreason != NULL)
	{
		memcpy(&blk_ccc_ofp_020.failure.failreason,failreason,sizeof(FailedReason));
	}
	data_length = sizeof(BLK_CCC_OFP_020);
	//发送编辑状态
	Send_Message( DDSTables.CCC_DPU_29.niConnectionId,0,&transaction_id, &blk_ccc_ofp_020 , &message_type_id, data_length, &enRetCode);

	//发送给pad new20250620
	if(Pad_heart_flag == 1)
	{
		Send_Message( DDSTables.CCC_PAD_020.niConnectionId,0,&transaction_id, &blk_ccc_ofp_020 , &message_type_id, data_length, &enRetCode);
	}

}
/*
 * 接收综显 任务分配方案确认信息  对信息进行处理，查看是否发生子任务重规划或航线重规划 进行不同处理
 */
void scheme_replan()
{
	//任务分配结果确认信息
	message_size = RECV_MAX_SIZE;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_5.niConnectionId,DDSTables.DPU2_CCC_5.niConnectionId,&DPU_CCC_data_5,sizeof DPU_CCC_data_5);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		Receive_Message(DDSTables.PAD_CCC_017.niConnectionId, 0, &transaction_id, &DPU_CCC_data_5, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode == 0)
	{
		//子任务修改
		if(DPU_CCC_data_5.task_mod_model == 1)
		{
			//编辑状态发送,子任务编辑中
			send_blk_ccc_ofp_020(DPU_CCC_data_5.program_number,0,1,NULL);
			data_length = sizeof(Information_the_results_of_tasking_confirm);
			//发送修改结果
			Send_Message( DDSTables.BLK_DTMS_CTAS_003.niConnectionId,0,&transaction_id, &DPU_CCC_data_5 , &message_type_id, data_length, &enRetCode);
		}
		//航线修改
		else if(DPU_CCC_data_5.task_mod_model == 2)
		{
			//编辑状态发送,航线编辑中
			send_blk_ccc_ofp_020(DPU_CCC_data_5.program_number,1,1,NULL);
			for(int i = 0 ; i < 3 ; i ++)
			{
				//存储有人机，无人机航线
				route_change(i);
			}
			//编辑状态发送,航线编辑完成
			send_blk_ccc_ofp_020(DPU_CCC_data_5.program_number,1,2,NULL);
		}
	}
	//任务区划分信息修改
	recv_dpu1_dpu2(DDSTables.DPU_CCC_25.niConnectionId,DDSTables.DPU2_CCC_25.niConnectionId,&blk_ofp_ccc_005,sizeof blk_ofp_ccc_005);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//		Receive_Message(DDSTables.PAD_CCC_005.niConnectionId, 0, &transaction_id, &blk_ofp_ccc_005, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode == 0)
	{
		data_length = sizeof(BLK_OFP_CCC_005);
		//发送修改结果
		Send_Message( DDSTables.BLK_DTMS_CTAS_004.niConnectionId,0,&transaction_id, &blk_ofp_ccc_005 , &message_type_id, data_length, &enRetCode);
	}
}

//航线修改
void route_change(int i)
{
	// DPU_DTMS 有人机通用航路修改保存
	BLK_CCC_OFP_018 temp;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_6.niConnectionId,DDSTables.DPU2_CCC_6.niConnectionId,&temp,sizeof temp);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//		Receive_Message(DDSTables.PAD_CCC_018.niConnectionId, 0, &transaction_id, &temp, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode == 0)
	{
		unsigned int plan = (temp.plan_code) % 3;
		unsigned int task = temp.subtask_index;
		// 加入航路点
		if (temp.airway_point_start_num == 0)
		{ // 第一包40点
			//取基本数据
			memcpy(&blk_ccc_ofp_018_cunchu[plan][task],&temp,16);
			// 将前四十个点保存
			memcpy( &(blk_ccc_ofp_018_cunchu[plan][task].waypoint_informations[0]) , &temp.waypoint_informations[0] ,temp.airway_point_num * sizeof(waypoint_information));
		}
		else if(temp.airway_point_start_num == 40)
		{
			//取基本数据
			memcpy(&blk_ccc_ofp_018_cunchu[plan][task],&temp,16);
			// 将后四十个点保存
			memcpy( &(blk_ccc_ofp_018_cunchu[plan][task].waypoint_informations[40]) , &temp.waypoint_informations[0] ,temp.airway_point_num * sizeof(waypoint_information));
		}

		//回复给综显，修改的有人机航线
		data_length = sizeof(BLK_CCC_OFP_018);
		Send_Message(DDSTables.CCC_DPU_7.niConnectionId,0,&transaction_id, &temp.plan_code, &message_type_id, data_length, &enRetCode);
		//发送给pad new20250620
		//		Send_Message(DDSTables.CCC_PAD_018.niConnectionId,0,&transaction_id, &temp.plan_code,&message_type_id, data_length, &enRetCode);

	}

	BLK_CCC_OFP_024 uav;
	static unsigned int plan = 0;
	static unsigned int id = 0;
	static unsigned char route_edit_ready = 0;
	// DPU_DTMS 无人机航路修改保存
	recv_dpu1_dpu2(DDSTables.DPU_CCC_10.niConnectionId,DDSTables.DPU2_CCC_10.niConnectionId,&uav,sizeof uav);
	if(enRetCode == 0)
{
		route_edit_ready = 1;
		plan = (uav.program_number) % 3;
		id = (uav.individual_drone_routing_programs.drone_serial_number - 1);
		unsigned int index = uav.individual_drone_routing_programs.planning_informations.packet_id *25;
		// 保存头信息
		memcpy(&blk_ccc_ofp_024_cunchu[plan][id],&uav,9 + 10 + 26);
		// 保存航路点信息
		memcpy(&blk_ccc_ofp_024_cunchu[plan][id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index], &uav.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], sizeof(planning_information_waypoint_information) * 25);

		//回复给综显，修改的无人机航线
		data_length = sizeof(BLK_CCC_OFP_024);
		Send_Message(DDSTables.CCC_DPU_11.niConnectionId,0,&transaction_id, &uav, &message_type_id, data_length, &enRetCode);
	}
	//接收完修改信息后发送到CTAS重新生成空域
	if(i == 2 && route_edit_ready == 1)
	{
		//发送修改无人机的所有航线到CTAS
		for(unsigned char j = 0 ;j < blk_ccc_ofp_024_cunchu[plan][id].individual_drone_routing_programs.planning_informations.total_packet ; j++)
		{
			BLK_CCC_OFP_024 temp;
			//取头
			memcpy(&temp,&blk_ccc_ofp_024_cunchu[plan][id],9 + 10 + 26);
			//取航路点
			memcpy(&temp.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], &blk_ccc_ofp_024_cunchu[plan][id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j * 25], sizeof(planning_information_waypoint_information) * 25);
			//包序号赋值
			temp.individual_drone_routing_programs.planning_informations.packet_id = j;
			// 综显发送
			data_length = sizeof(BLK_CCC_OFP_024);
			Send_Message(DDSTables.BLK_DTMS_CTAS_008.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);
		}
		//清空变量
		plan = 0;
		id = 0;
		route_edit_ready = 0;
	}
}
//发送无人机单任务到辅助决策，生成空域
void send_uav_airway(unsigned int plan,unsigned int id)
{
	//如果无人机跨区域重新生成空域，空域为任务区,去掉生成的绕行航点
//	if(global_stage > 1)
//	{
//		int task_new = 0;
//		int task_old = 0;
//		task_new = CCC_DPU_data_6[plan].formation_synergy_mission_programs[id+1].task_sequence_informations[global_stage].point_or_area_id;
//		task_old = CCC_DPU_data_6[plan].formation_synergy_mission_programs[id+1].task_sequence_informations[global_stage-1].point_or_area_id;
//		//差值绝对值相差2，则是跳过任务区
//		if(task_new-task_old == 2 || task_new-task_old == -2)
//		{
//			//发送无人机当前任务的所有航线到CTAS
//			for(unsigned char j = 0 ;j < blk_ccc_ofp_024_cunchu[plan][id].individual_drone_routing_programs.planning_informations.total_packet ; j++)
//			{
//				BLK_CCC_OFP_024 temp;
//				//取头
//				memcpy(&temp,&blk_ccc_ofp_024_cunchu[plan][id],9 + 10 + 26);
//				//取航路点
//				memcpy(&temp.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0]
//						,&blk_ccc_ofp_024_cunchu[plan][id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[2+j*25]
//						  ,sizeof(planning_information_waypoint_information)*25);
//				//包序号赋值
//				temp.individual_drone_routing_programs.planning_informations.packet_id = j;
//				//减去绕行的航点
//				temp.individual_drone_routing_programs.planning_informations.waypoints_number -= 2;
//				// CTAS发送
//				data_length = sizeof(BLK_CCC_OFP_024);
//				Send_Message(DDSTables.BLK_DTMS_CTAS_008.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);
//			}
//			return;
//		}
//	}



	//发送无人机当前任务的所有航线到CTAS
	for(unsigned char j = 0 ;j < blk_ccc_ofp_024_cunchu[plan][id].individual_drone_routing_programs.planning_informations.total_packet ; j++)
	{
		BLK_CCC_OFP_024 temp;
		//取头
		memcpy(&temp,&blk_ccc_ofp_024_cunchu[plan][id],9 + 10 + 26);
		//取航路点
		memcpy(&temp.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], &blk_ccc_ofp_024_cunchu[plan][id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j * 25], sizeof(planning_information_waypoint_information) * 25);
		//包序号赋值
		temp.individual_drone_routing_programs.planning_informations.packet_id = j;
		// CTAS发送
		data_length = sizeof(BLK_CCC_OFP_024);
		Send_Message(DDSTables.BLK_DTMS_CTAS_008.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);
	}
}

/******************************* 无人机状态处理 + 通信 模块 ******************************/
/*
 *  接收无人机（飞仿）发来遥测数据黑包，对黑包解码后，将信息组为 无人机状态信息与编队链路信息，发给综显
 */
static void update_lead_uav_id_by_yaoce(void)
{
    unsigned int new_lead = 0;
    int lead_cnt = 0;

    /* Prefer CCC_DPU_data_3 control status (1: manned, 2: ground). */
    for(int i = 0; i < UAV_MAX_NUM; i++)
    {
        unsigned int pid = CCC_DPU_data_3.drone_specific_informations[i].platform_num;
		if (pid == 0)
		{
			continue;
		}

        if(CCC_DPU_data_3.drone_specific_informations[i].platform_control_status == 1)
        {
            new_lead = pid;
            lead_cnt++;
        }
    }

    if(lead_cnt == 1)
    {
        load_file.lead_uav_id = new_lead;
        return;
    }

    /* Fallback: use formationId station_address/isControl if control_status not filled. */
    new_lead = 0;
    lead_cnt = 0;
    for(int i = 0; i < UAV_MAX_NUM; i++)
    {
		if (formationId[i].planeId == 0)
		{
			continue;
		}
        if(formationId[i].isControl == 1 && formationId[i].station_address == MANNED_ID)
        {
            new_lead = formationId[i].planeId;
            lead_cnt++;
        }
    }

    if(lead_cnt == 1)
    {
        load_file.lead_uav_id = new_lead;
    }
}
void uav_status_handle()
{
	// 接收无人机传回 各个帧数据 整理为无人机状态信息 发给综显
	// // printf("************************* 无人机状态处理 + 通信 模块 ****************************";


	recv_blk_kkl_ccc_000_008_010_011(); // 周期接收 uav遥测数据帧接口
	update_lead_uav_id_by_yaoce(); // update lead_uav_id by telemetry/control status
	load_file.lead_uav_id = 0x1005;//测试
//	printf("lead:0x%X\n",load_file.lead_uav_id);
	// 3.2 无人机状态信息
	send_drone_state_information();

	//	if(CCC_DPU_data_3.drone_number > 1)
	//	{
	//		static int cnt = 0;
	//		for(int i = 0; i < 4; i ++)
	//		{
	//			printf("id:%d  ",CCC_DPU_data_3.drone_specific_informations[i].platform_num);
	//		}
	//		printf("cnt%d\n",cnt);
	//		cnt++;
	//	}
	//
	//	for(int i = 0; i < 4; i ++)
	//	{
	//		if(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_heading > 360 || CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_heading < 0)
	//		{
	//			printf("i:%d heading: %f s81: %d\n",i,CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_heading,s81_frame.course);
	//		}
	//	}
	send_formation_link_status_information(); //3.3 编队链路状态信息  （可能有问题）
}

// 解码黑包数据帧
void decode_uav_frame(int uav_index)
{
	//!
	//! 根据每个子帧类别位 取出判断 先完善给综显的信息
	//! 效验规则    帧类别+帧内容+效验码=0
	//! 完善 有些是字节中的某位赋值  已经完成了辅助函数 获取 bit 位 get_bit()   赋值bit 位 give_bit()
	/******************* uav 1 *******************/
	// // // printf("开始解码!!";
	// 1 2帧不循环， 可直接取出
	memset(&(s81_frame),0,sizeof(s81_frame));
	memset(&(s82_frame),0,sizeof(s82_frame));

	memset(&(s3A_frame),0,sizeof(s3A_frame));
	memset(&(s3B_frame),0,sizeof(s3B_frame));

	memset(&(s4A_frame),0,sizeof(s4A_frame));
	memset(&(s4B_frame),0,sizeof(s4B_frame));
	memset(&(s4C_frame),0,sizeof(s4C_frame));
	memset(&(s4D_frame),0,sizeof(s4D_frame));

	memset(&(s5A_frame),0,sizeof(s5A_frame));
	memset(&(s5B_frame),0,sizeof(s5B_frame));
	memset(&(s5C_frame),0,sizeof(s5C_frame));
	memset(&(s5D_frame),0,sizeof(s5D_frame));
	memset(&(s5E_frame),0,sizeof(s5E_frame));


	memset(&(s6_frame),0,sizeof(s6_frame));
	memset(&(s7_frame),0,sizeof(s7_frame));
	memset(&(s8_frame),0,sizeof(s8_frame));
	memset(&(s9_frame),0,sizeof(s9_frame));

	//判断校验和
	if(checkSum(dds_data_rece.dataA+2,30) == 0)
	{
		memcpy(&(s81_frame),dds_data_rece.dataA,sizeof(s81_frame));// 取出第一帧
	}

#ifdef DEBUG
	//// printf("s81_frame");
	//printRawData((char*)&s81_frame,sizeof(s81_frame));
#endif

	//判断校验和
	if(checkSum(dds_data_rece.dataA+2+32,30) == 0)
	{
		memcpy(&(s82_frame),dds_data_rece.dataA+32,sizeof(s82_frame));// 取出第二帧
		parseYaoCeZiZhen82(uav_index);
	}
#ifdef DEBUG
	//// printf("s82_frame");
	//printRawData((char*)&s82_frame,sizeof(s82_frame));
#endif
	// 处理第三帧
	unsigned int len_1_2 = 64;
	short frame_type_3 = 0;
	//判断校验和
	if(checkSum(dds_data_rece.dataA+2+len_1_2,30) == 0)
	{
		memcpy(&(frame_type_3),dds_data_rece.dataA+len_1_2 + 2,1); // 取出帧类别
		switch(frame_type_3)
		{
		case (char)0x3a:
				memcpy(&(s3A_frame),dds_data_rece.dataA+len_1_2,sizeof(s3A_frame));
		parseYaoCeZiZhen3A(uav_index);
		s3a_flag = 1;
#ifdef DEBUG
		//// printf("s3A_frame");
		//printRawData((char*)&s3A_frame,sizeof(s3A_frame));
#endif
		break;

		case (char)0x3b:
				memcpy(&(s3B_frame),dds_data_rece.dataA+len_1_2,sizeof(s3B_frame));
#ifdef DEBUG
		//// printf("s3B_frame");
		//printRawData((char*)&s3B_frame,sizeof(s3B_frame));
#endif
		break;

		default:
			// // // printf("未知3帧类别";
			break;
		}
	}
	// 处理第四帧
	unsigned int len_1_2_3 = 96;
	short frame_type_4 = 0;
	//判断校验和
	if(checkSum(dds_data_rece.dataA+2+len_1_2_3,30) == 0)
	{
		memcpy(&(frame_type_4),dds_data_rece.dataA+len_1_2_3 + 2,1); // 取出帧类别
		// // // printf("4帧类别",frame_type_4;
		switch(frame_type_4)
		{
		case (char)0x4a:
			{
			s4a_flag = 1;
			memcpy(&(s4A_frame),dds_data_rece.dataA+len_1_2_3,sizeof(s4A_frame));
#ifdef DEBUG
			//// printf("s4A_frame");
			//printRawData((char*)&s4A_frame,sizeof(s4A_frame));
#endif
			}
		break;

		case (char)0x4b:
			{
			s4b_flag = 1;
			memcpy(&(s4B_frame),dds_data_rece.dataA+len_1_2_3,sizeof(s4B_frame));
#ifdef DEBUG
			//// printf("s4B_frame");
			//printRawData((char*)&s4B_frame,sizeof(s4B_frame));
#endif
			}
		break;

		case (char)0x4c:
			{
			s4c_flag = 1;
			memcpy(&(s4C_frame),dds_data_rece.dataA+len_1_2_3,sizeof(s4C_frame));
#ifdef DEBUG
			//// printf("s4C_frame");
			//printRawData((char*)&s4C_frame,sizeof(s4C_frame));
#endif
			}
		break;

		case (char)0x4d:
			{
			memcpy(&(s4D_frame),dds_data_rece.dataA+len_1_2_3,sizeof(s4D_frame));

			parseYaoCeZiZhen4D(uav_index);
#ifdef DEBUG
			//// printf("s4D_frame");
			//printRawData((char*)&s4D_frame,sizeof(s4D_frame));
#endif
			}
		break;

		default:
			// // // printf("未知4帧类别";
			break;
		}
	}

	// 处理第五帧
	//    int len_1_2_3_4 = len_1_2_3 + sizeof(*s4A_frame);
	unsigned int len_1_2_3_4 = 128;
	short frame_type_5 = 0;
	//判断校验和
	if(checkSum(dds_data_rece.dataA+2+len_1_2_3_4,30) == 0)
	{
		memcpy(&(frame_type_5),dds_data_rece.dataA+len_1_2_3_4 + 2,1); // 取出帧类别
		// // // printf("5帧类别",frame_type_5;
		switch(frame_type_5)
		{
		case (char)0x5a:
			{
			memcpy(&(s5A_frame),dds_data_rece.dataA + len_1_2_3_4,sizeof(s5A_frame));
#ifdef DEBUG
			//// printf("s5A_frame");
			//printRawData((char*)&s5A_frame,sizeof(s5A_frame));
#endif
			}
		break;

		case (char)0x5b:
			{
			s5b_flag = 1;
			memcpy(&(s5B_frame),dds_data_rece.dataA + len_1_2_3_4,sizeof(s5B_frame));
#ifdef DEBUG
			//// printf("s5B_frame");
			//printRawData((char*)&s5B_frame,sizeof(s5B_frame));
#endif
			}
		break;

		case (char)0x5c:
			{
			s5c_flag = 1;
			memcpy(&(s5C_frame),dds_data_rece.dataA + len_1_2_3_4,sizeof(s5C_frame));
#ifdef DEBUG
			//// printf("s5C_frame");
			//printRawData((char*)&s5C_frame,sizeof(s5C_frame));
#endif
			}
		break;

		case (char)0x5d:
			{
			memcpy(&(s5D_frame),dds_data_rece.dataA + len_1_2_3_4,sizeof(s5D_frame));
#ifdef DEBUG
			//// printf("s5D_frame");
			//printRawData((char*)&s5D_frame,sizeof(s5D_frame));
#endif
			}
		break;

		case (char)0x5e:
			{
			s5e_flag = 1;
			memcpy(&(s5E_frame),dds_data_rece.dataA + len_1_2_3_4,sizeof(s5E_frame));
#ifdef DEBUG
			//// printf("s5E_frame");
			//printRawData((char*)&s5E_frame,sizeof(s5E_frame));
#endif
			}
		break;

		default:
			//// printf("unknow frame 5 \n");
			break;
		}
	}
	// 处理编队飞行遥测帧
	unsigned int len_1_2_3_4_5 = 160;
	//编队1帧
	short frame_type_6 = 0;
	//判断校验和
	if(checkSum(dds_data_rece.dataA+2+len_1_2_3_4_5,38) == 0)
	{
		memcpy(&(frame_type_6),dds_data_rece.dataA+len_1_2_3_4_5 + 2,1); // 取出帧类别
		if(frame_type_6 == 0x81)
		{
			//只判断长机的状态
			if(uav_index == (load_file.lead_uav_id == 0x1005) ? 0 : 1)
			{
				memcpy(&(s6_frame),dds_data_rece.dataA + len_1_2_3_4_5,40);
				//无人机编队状态
				if(get_int8_bit(s6_frame.flyState.fly_stage,0)==0 && get_int8_bit(s6_frame.flyState.fly_stage,1) == 1
										&& get_int8_bit(s6_frame.flyState.fly_stage,2)==1 && get_int8_bit(s6_frame.flyState.fly_stage,3) == 1)
				{
					//编队
					if (s6_bd_flag == 0) // ← 新增：0→1 的上升沿
					{
						memset(&s9_manned_lead, 0, sizeof(MannedLead));
						memset(&s9_manned_exit, 0, sizeof(MannedExit));
					}
					s6_bd_flag = 1;
				}
				else
				{
					//非编队
					s6_bd_flag = 0;
				}

				s6_hold_flag = s6_frame.formStage;
			}
		}
	}
	//编队2帧
	unsigned int len_1_2_3_4_5_6 = 200;
	short frame_type_7 = 0;
	static int s7_manned_lh = 0;//无人机长机领航状态保留
	//判断校验和
	if(checkSum(dds_data_rece.dataA+2+len_1_2_3_4_5_6,38) == 0)
	{
		memcpy(&(frame_type_7),dds_data_rece.dataA+len_1_2_3_4_5_6 + 2,1); // 取出帧类别
		if(frame_type_7 == 0x82)
		{
			char* moni_request = dds_data_rece.dataA + len_1_2_3_4_5_6 + 19;
			//编队遥测2帧20字节
			char s20;
			s20 = *(dds_data_rece.dataA + len_1_2_3_4_5_6 + 20);
			//编队航线装订回报
			if((*moni_request) == 0x30)
			{
				if(get_int8_bit(s20,6) == 0 && get_int8_bit(s20,7) == 0)
				{
					b2_frame_30[uav_index] = 0;
				}
				else
				{
					b2_frame_30[uav_index] = -1;
				}
			}
			//只判断长机的状态
			if(uav_index == (load_file.lead_uav_id == 0x1005) ? 0 : 1)
			{
				//开始领航指令回报
				if((*moni_request) == 0x16)
				{
					//接受到16指令回复

					if(get_int8_bit(s20,6) == 0 && get_int8_bit(s20,7) == 0)
					{
						b2_frame_16[uav_index] = 1;
					}
					else
					{
						b2_frame_16[uav_index] = -1;
					}
				}
				//切换解散点回报
				else if((*moni_request) == 0x14)
				{
					//接收到退出编队的14指令回复
					if(get_int8_bit(s20,6) == 0 && get_int8_bit(s20,7) == 0 )
					{
						b2_frame_14[uav_index] = 1;
					}
					else
					{
						b2_frame_14[uav_index] = -1;
					}
				}
				//编队遥测2帧24字节
				char s24;
				s24 = *(dds_data_rece.dataA + len_1_2_3_4_5_6 + 24);
				if(s24 == 0)
				{
					//按航线飞
					s7_hx_flag = 0;
				}
				else if(s24 == 1)
				{
					//有人机领航飞行
					s7_hx_flag = 1;
					s7_manned_lh = 1;
				}
			}
			//编队遥测82帧25字节
			char s25;
			s25 = *(dds_data_rece.dataA + len_1_2_3_4_5_6 + 25);
			memcpy(&s7_redundancy[uav_index],&s25,sizeof(char));
		}
	}
	//编队3帧
	unsigned int len_1_2_3_4_5_6_7 = 240;
	short frame_type_8 = 0;
	//判断校验和
	if(checkSum(dds_data_rece.dataA+2+len_1_2_3_4_5_6_7,38) == 0)
	{
		memcpy(&(frame_type_8),dds_data_rece.dataA+len_1_2_3_4_5_6_7 + 2,1); // 取出帧类别
	}
	//编队4帧
	unsigned int len_1_2_3_4_5_6_7_8 = 280;
	short frame_type_9 = 0;
	//判断校验和
	if(checkSum(dds_data_rece.dataA+2+len_1_2_3_4_5_6_7_8,38) == 0)
	{
		memcpy(&(frame_type_9),dds_data_rece.dataA+len_1_2_3_4_5_6_7_8 + 2,1); // 取出帧类别
		if(frame_type_9 == 0x4B)
		{
			//只判断长机的状态
			if(uav_index == (load_file.lead_uav_id == 0x1005) ? 0 : 1)
			{
				//领航条件
				//编队遥测4B帧33字节
				char s33;
				s33 = *(dds_data_rece.dataA + len_1_2_3_4_5_6_7_8 + 33);
				//有人机领航条件
				memcpy(&s9_manned_lead,&s33,sizeof(MannedLead));

				//领航退出条件
				//编队遥测4B帧34字节
				char s34;
				s34 = *(dds_data_rece.dataA + len_1_2_3_4_5_6_7_8 + 34);
				//有人机领航退出条件
				memcpy(&s9_manned_exit,&s34,sizeof(MannedExit));

				//航线退出航点号
				//编队遥测4B帧35字节
				unsigned char s35;
				s35 = *(dds_data_rece.dataA + len_1_2_3_4_5_6_7_8 + 35);
				memcpy(&dissolve_point,&s35,sizeof(unsigned char));

			}
			//编队遥测4B帧22字节
			char s22;
			s22 = *(dds_data_rece.dataA + len_1_2_3_4_5_6_7_8 + 22);
			memcpy(&s9_avoidinfo,&s22,sizeof(AvoidInfo));
			//无人机碰撞信息
			s9_uav_avoid_flag[uav_index] = s9_avoidinfo.uav_uav;
			s9_man_avoid_flag[uav_index] = s9_avoidinfo.uav_manned;

			//编队遥测4B帧3~6字节
			int s3_6;
			memcpy(&s3_6,dds_data_rece.dataA + len_1_2_3_4_5_6_7_8 + 3,sizeof(int));
			//编队遥测4B帧7~10字节
			int s7_10;
			memcpy(&s7_10,dds_data_rece.dataA + len_1_2_3_4_5_6_7_8 + 7,sizeof(int));
			//编队遥测4B帧11~12字节
			short s11_12;
			memcpy(&s11_12,dds_data_rece.dataA + len_1_2_3_4_5_6_7_8 + 11,sizeof(short));
			//编队遥测4B帧13~14字节
			unsigned short s13_14;
			memcpy(&s13_14,dds_data_rece.dataA + len_1_2_3_4_5_6_7_8 + 13,sizeof(short));
			//编队遥测4B帧15~16字节
			unsigned short s15_16;
			memcpy(&s15_16,dds_data_rece.dataA + len_1_2_3_4_5_6_7_8 + 15,sizeof(short));
			s9_emergence_area[uav_index].Lon = (float)s3_6 * lon_scale;
			s9_emergence_area[uav_index].Lat = (float)s7_10 * lat_scale;
			s9_emergence_area[uav_index].Angle = (float)s11_12* ((double)180 / (((long long )2 << 14)-1));
			s9_emergence_area[uav_index].Long = s13_14;
			s9_emergence_area[uav_index].Wide = s15_16;

		}
	}

	static int display_jr_cnt = 0;
	//编队状态 编队保持 按航线飞行
	if(blk_ccc_ofp_027.display == 0 && s6_bd_flag == 1 && s6_hold_flag == 2 && s7_hx_flag == 0)
	{
		display_jr_cnt++;
	}
	//持续20拍后开始显示
	if(display_jr_cnt > 20)
	{
		display_jr_cnt = 0;
		blk_ccc_ofp_027.display = 1;
	}
	static int display_tc_cnt = 0;
	//非编队状态 有人机领航
	if(blk_ccc_ofp_027.display == 1 && (s7_hx_flag == 1 || s6_bd_flag != 1))
	{
		display_tc_cnt++;
	}
	//持续20拍后进入领航不显示
	if(display_tc_cnt > 20)
	{
		display_tc_cnt = 0;
		blk_ccc_ofp_027.display = 0;
	}
	//无人机有过有人机领航状态且目前不在领航状态中或退出编队
	if(s7_manned_lh == 1 && (s7_hx_flag == 0 || s6_bd_flag != 1))
	{
		//发送有人机领航退出
		send_blk_ccc_ofp_028();
		//此次领航结束
		s7_manned_lh = 0;
	}

}

void fault_message_init(int i)
{
	/******************* warning_message *******************/
	//旋翼转速高告警 4B9
	if(get_int8_bit(s4B_frame.power_system_1.power_system_status,3))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xyzsg = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xyzsg = 0;
	//旋翼转速低告警 4B9
	if(get_int8_bit(s4B_frame.power_system_1.power_system_status,4))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xyzsd = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xyzsd = 0;
	//主减滑油压力低 4B12
	if(get_int8_bit(s4B_frame.rotor_system.rotor_system_status,7))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.zjhyyld = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.zjhyyld = 0;
	//火警 4B9
	if(get_int8_bit(s4B_frame.power_system_1.power_system_status,5))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.hj = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.hj = 0;
	//发动机停车 5C19
	if(get_int8_bit(s5C_frame.ecu_warning_2.ecu_warning_word,4))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fdjtc = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fdjtc = 0;
	//发动机数控系统失效 5C9 , 确认收到5C后再判断，过滤缺省值 20250806new
	if(get_int8_bit(s5C_frame.ecu_switch.ecu_switch_output,3) == 0 && s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fdjskxtsx = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fdjskxtsx = 0;
	//发动机滑油压低 5B25
	if(get_int8_bit(s5B_frame.warning.warning_signal,0))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fdjhyyd = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fdjhyyd = 0;
	//发动机燃油压低 5B27
	if(get_bit(s5B_frame.attention_1.attention_level_signal,0))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fdjryyd = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fdjryyd = 0;
	//起飞功率时间剩余10s 5C15~16
	if(get_bit(s5C_frame.ecu_warning_1.ecu_warning_word_1,14))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.qfglsjsy10s = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.qfglsjsy10s = 0;
	//Np超转 5C15~16
	if(get_bit(s5C_frame.ecu_warning_1.ecu_warning_word_1,7))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.Npcz = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.Npcz = 0;
	//低油位 4B10
	if(get_int8_bit(s4B_frame.power_system_2.power_system_status,6))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.dyw = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.dyw = 0;
	//燃油阀断油 5E5
	if(s5E_frame.power_system_2.breaking_status)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.ryfdy = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.ryfdy = 0;
	//飞控计算机失效 4A18
	if(s4A_frame.malfunc_flying_computer.cpu_fail == 2)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fkjsjsx = 1;
	else if(s4a_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fkjsjsx = 0;
	//俯仰舵机失效 4A19
	if(s4A_frame.malfunc_steering.pit_fail == 2)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fydjsx = 1;
	else if(s4a_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.fkjsjsx = 0;
	//左横滚舵机失效 4A19
	if(s4A_frame.malfunc_steering.l_roll_fail == 2)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.zhgdjsx = 1;
	else if(s4a_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.zhgdjsx = 0;
	//右横滚舵机失效 4A19
	if(s4A_frame.malfunc_steering.r_roll_fail == 2)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.yhgdjsx = 1;
	else if(s4a_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.yhgdjsx = 0;
	//尾桨舵机失效 4A19
	if(s4A_frame.malfunc_steering.tail_oar_fail == 2)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.wjdjsx = 1;
	else if(s4a_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.wjdjsx = 0;
	//伺服控制器失效 4A20
	if(s4A_frame.malfunc_servo_control.controller_fail == 2)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.sfkzqsx = 1;
	else if(s4a_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.sfkzqsx = 0;
	//姿态角速率失效 4B6
	if(get_int8_bit(s4B_frame.sensor_1.sensor_fault,3))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.ztjslsx = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.ztjslsx = 0;
	//位/速全失效 4B6
	if(get_int8_bit(s4B_frame.sensor_1.sensor_fault,4))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.wsqsx = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.wsqsx = 0;
	//高/升降全失效 4B6
	if(get_int8_bit(s4B_frame.sensor_1.sensor_fault,5))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.gsjqsx = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.gsjqsx = 0;
	//测控失效 4B13
	if(get_int8_bit(s4B_frame.avionics.avionics_status,1))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.cksx = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.cksx = 0;
	//ESMC总失效 5E6
	if(get_int8_bit(s5E_frame.backup_1,3))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.ESMCzsx = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.ESMCzsx = 0;
	//应急设备配电故障 5E6
	if(get_int8_bit(s5E_frame.backup_1,4))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.yjsbpdgz = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.yjsbpdgz = 0;
	//蓄电池组1脱网 5E4
	if(get_int8_bit(s5E_frame.power_system_1.pel_power_system_1,1))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xdcz1tw = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xdcz1tw = 0;
	//蓄电池组2脱网 5E4
	if(get_int8_bit(s5E_frame.power_system_1.pel_power_system_1,2))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xdcz2tw = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xdcz2tw = 0;
	//蓄电池组1超温 5E4
	if(get_int8_bit(s5E_frame.power_system_1.pel_power_system_1,3))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xdcz1cw = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xdcz1cw = 0;
	//蓄电池组2超温 5E4
	if(get_int8_bit(s5E_frame.power_system_1.pel_power_system_1,4))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xdcz2cw = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.xdcz2cw = 0;
	//结冰 4B23
	if(get_int8_bit(s4B_frame.electro_system.electro_system_status_2,0))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.jb = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.jb = 0;
	//采集与监测计算机失效 5B22~23
	if(get_bit(s5B_frame.hums_1.hums_status,0))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.cjyjcjsjsx = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.cjyjcjsjsx = 0;
	//主桨振动超限 5B25
	if(get_int8_bit(s5B_frame.warning.warning_signal,4))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.zjzdcx = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.zjzdcx = 0;
	//尾桨振动超限 5B25
	if(get_int8_bit(s5B_frame.warning.warning_signal,5))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.wjzdcx = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.wjzdcx = 0;
	//水平轴振动振动超限 5B25
	if(get_int8_bit(s5B_frame.warning.warning_signal,6))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.spzzdcx = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.w_message.spzzdcx = 0;

	/******************* attention_message *******************/
	//主减滑油温度高 4B12
	if(get_int8_bit(s4B_frame.rotor_system.rotor_system_status,2))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.zjhywdg = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.zjhywdg = 0;
	//主减金属屑 4B12
	if(get_int8_bit(s4B_frame.rotor_system.rotor_system_status,0))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.zjjsx = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.zjjsx = 0;
	//尾减滑油温度高 4B12
	if(get_int8_bit(s4B_frame.rotor_system.rotor_system_status,3))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wjhywdg = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wjhywdg = 0;
	//尾减金属屑 4B12
	if(get_int8_bit(s4B_frame.rotor_system.rotor_system_status,1))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wjjsx = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wjjsx = 0;
	//发动机燃油滤预堵塞 4B10
	if(get_int8_bit(s4B_frame.power_system_2.power_system_status,0))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjrylyds = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjrylyds = 0;
	//发动机金属屑 4B10
	if(get_int8_bit(s4B_frame.power_system_2.power_system_status,1))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjjsx = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjjsx = 0;
	//发动机滑油温高 4B10
	if(get_int8_bit(s4B_frame.power_system_2.power_system_status,2))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjhywg = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjhywg = 0;
	//降级控制-柔和操纵 5C15~16
	if(get_bit(s5C_frame.ecu_warning_1.ecu_warning_word_1,4))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.jjkzrhcz = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.jjkzrhcz = 0;
	//降级控制-功率受限 5C15~16
	if(get_bit(s5C_frame.ecu_warning_1.ecu_warning_word_1,5))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.jjkzglsx = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.jjkzglsx = 0;
	//Ng高 5C15~16
	if(get_bit(s5C_frame.ecu_warning_1.ecu_warning_word_1,8))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.Ngg = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.Ngg = 0;
	//Np高 5C15~16
	if(get_bit(s5C_frame.ecu_warning_1.ecu_warning_word_1,9))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.Npg = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.Npg = 0;
	//发双通道状态不一致 5C15~16
	if(get_bit(s5C_frame.ecu_warning_1.ecu_warning_word_1,10))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fstdztbyz = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fstdztbyz = 0;
	//发动机超温 5C15~16
	if(get_bit(s5C_frame.ecu_warning_1.ecu_warning_word_1,11))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjcw = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjcw = 0;
	//发动机超扭 5C15~16
	if(get_bit(s5C_frame.ecu_warning_1.ecu_warning_word_1,12))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjcn = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjcn = 0;
	//A通道数控失效 5C19
	if(get_bit(s5C_frame.ecu_warning_2.ecu_warning_word,0))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.Atdsksx = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.Atdsksx = 0;
	//B通道数控失效 5C19
	if(get_bit(s5C_frame.ecu_warning_2.ecu_warning_word,1))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.Btdsksx = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.Btdsksx = 0;
	//发动机一般故障 5C19
	if(get_bit(s5C_frame.ecu_warning_2.ecu_warning_word,2))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjybgz = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjybgz = 0;
	//步进电机检测异常结束 5C19
	if(get_bit(s5C_frame.ecu_warning_2.ecu_warning_word,5))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.bjdjjcycjs = 1;
	else if(s5c_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.bjdjjcycjs = 0;
	//供油管路压力低 5B27~28
	if(get_bit(s5B_frame.attention_1.attention_level_signal,3))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.gyglyld = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.gyglyld = 0;
	//卫星定位失效 4B7
	if(get_int8_bit(s4B_frame.sensor_2.sensor_fault,7))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wxdwsx = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wxdwsx = 0;
	//俯仰舵机余度故障 4A19
	if(s4A_frame.malfunc_steering.pit_fail == 1 || s4A_frame.malfunc_steering.pit_fail == 2)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fydjydgz = 1;
	else if(s4a_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fydjydgz = 0;
	//左横滚舵机余度故障 4A19
	if(s4A_frame.malfunc_steering.l_roll_fail == 1 || s4A_frame.malfunc_steering.l_roll_fail == 2)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.zhgdjydgz = 1;
	else if(s4a_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.zhgdjydgz = 0;
	//右横滚舵机余度故障 4A19
	if(s4A_frame.malfunc_steering.r_roll_fail == 1 || s4A_frame.malfunc_steering.r_roll_fail == 2)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.yhgdjydgz = 1;
	else if(s4a_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.yhgdjydgz = 0;
	//尾桨舵机余度故障 4A19
	if(s4A_frame.malfunc_steering.tail_oar_fail == 1 || s4A_frame.malfunc_steering.tail_oar_fail == 2)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wjdjydgz = 1;
	else if(s4a_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wjdjydgz = 0;
	//伺服控制器余度故障 4A20
	if(s4A_frame.malfunc_servo_control.controller_fail == 1 || s4A_frame.malfunc_servo_control.controller_fail == 2)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.sfkzqydgz = 1;
	else if(s4a_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.sfkzqydgz = 0;
	//光纤惯导故障 4B6
	if(get_int8_bit(s4B_frame.sensor_1.sensor_fault,1))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.gqgdgz = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.gqgdgz = 0;
	//激光惯导故障 4B6
	if(get_int8_bit(s4B_frame.sensor_1.sensor_fault,0))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.jggdgz = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.jggdgz = 0;
	//无线电高度低 4B26
	if(get_int8_bit(s4B_frame.source_3.sensor_source,6))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wxdgdd = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wxdgdd = 0;
	//俯仰角辅助监控故障 4B26
	if(get_int8_bit(s4B_frame.source_3.sensor_source,0) != 0 || get_int8_bit(s4B_frame.source_3.sensor_source,1) != 0)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fyjfzjkgz = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fyjfzjkgz = 0;
	//横滚角辅助监控故障 4B26
	if(get_int8_bit(s4B_frame.source_3.sensor_source,2) != 0 || get_int8_bit(s4B_frame.source_3.sensor_source,3) != 0)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.hgjfzjkgz = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.hgjfzjkgz = 0;
	//航向角辅助监控故障 4B26
	if(get_int8_bit(s4B_frame.source_3.sensor_source,4) != 0 || get_int8_bit(s4B_frame.source_3.sensor_source,5) != 0)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.hxjfzjkgz = 1;
	else if(s4b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.hxjfzjkgz = 0;
	//蓄电池组1容量低 5E4
	if(get_int8_bit(s5E_frame.power_system_1.pel_power_system_1,5))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.xdcz1rld = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.xdcz1rld = 0;
	//蓄电池组2容量低 5E4
	if(get_int8_bit(s5E_frame.power_system_1.pel_power_system_1,6))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.xdcz2rld = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.xdcz2rld = 0;
	//发电机脱网 5E4
	if(get_int8_bit(s5E_frame.power_system_1.pel_power_system_1,0))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjtw = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjtw = 0;
	//地面电源故障 5E4
	if(get_int8_bit(s5E_frame.power_system_1.pel_power_system_1,7))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.dmdygz = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.dmdygz = 0;
	//发电系统故障 5E6
	if(get_int8_bit(s5E_frame.backup_1,0))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdxtgz = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdxtgz = 0;
	//ESMC通道故障 5E6
	if(get_int8_bit(s5E_frame.backup_1,1))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.ESMCtdgz = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.ESMCtdgz = 0;
	//ESMC功能降级 5E6
	if(get_int8_bit(s5E_frame.backup_1,2))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.ESMCgnjj = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.ESMCgnjj = 0;
	//ESMC总线故障 5E6
	if(get_int8_bit(s5E_frame.backup_1,3))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.ESMCzxgz = 1;
	else if(s5e_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.ESMCzxgz = 0;
	//结冰探测失效 5B27~28
	if(get_bit(s5B_frame.attention_1.attention_level_signal,8))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.jbtcsx = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.jbtcsx = 0;
	//主减振动超限 5B27~28
	if(get_bit(s5B_frame.attention_1.attention_level_signal,9))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.zjzdcx = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.zjzdcx = 0;
	//尾减振动超限 5B27~28
	if(get_bit(s5B_frame.attention_1.attention_level_signal,10))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wjzdcx = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wjzdcx = 0;
	//发动机振动超限 5B27~28
	if(get_bit(s5B_frame.attention_1.attention_level_signal,11))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjzdcx = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.fdjzdcx = 0;
	//采集与监测计算机故障 5B22~23
	if(get_bit(s5B_frame.hums_1.hums_status,2))
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.cjyjcjsjgz = 1;
	else if(s5b_flag == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.cjyjcjsjgz = 0;


	//其余为任务遥测

	unsigned short temp;
	//主惯导部件故障 1C20~21 bit0
	memcpy(&temp,&KKL_CCC_data_21_1[i].data[20],2);
	if(get_bit(temp,0) && t1c_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.zgdbjgz = 1;
	else if(t1c_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.zgdbjgz = 0;
	//备惯导部件故障 1C16~17 bit0
	memcpy(&temp,&KKL_CCC_data_21_1[i].data[16],2);
	if(get_bit(temp,0) && t1c_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.bgdbjgz = 1;
	else if(t1c_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.bgdbjgz = 0;
	//大气数据计算机故障 1C24~25 bit2
	memcpy(&temp,&KKL_CCC_data_21_1[i].data[24],2);
	if(get_bit(temp,2) && t1c_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.dqsjjsjgz = 1;
	else if(t1c_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.dqsjjsjgz = 0;
	//系统控制管理1故障 5帧8~9 bit0
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[8],2);
	if(get_bit(temp,0) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.xtkzgl1gz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.xtkzgl1gz = 0;
	//系统控制管理2故障 5帧8~9 bit1
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[8],2);
	if(get_bit(temp,1) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.xtkzgl2gz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.xtkzgl2gz = 0;
	//无线电高度表故障 5帧8~9 bit2
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[8],2);
	if(get_bit(temp,2) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wxdgdbgz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.wxdgdbgz = 0;
	//UHF频段测控故障 5帧8~9 bit3
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[8],2);
	if(get_bit(temp,3) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.UHFpdckgz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.UHFpdckgz = 0;
	//敌我识别故障 5帧8~9 bit13
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[8],2);
	if(get_bit(temp,13) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.dwsbgz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.dwsbgz = 0;
	//C波段测控链故障 5帧8~9 bit14
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[8],2);
	if(get_bit(temp,14) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.Cbdcklgz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.Cbdcklgz = 0;
	//任务处理单元1故障 4A9~10 bit0
	memcpy(&temp,&KKL_CCC_data_21_4[i].data[9],2);
	if(get_bit(temp,0) && t4a_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.rwcldy1gz = 1;
	else if(t4a_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.rwcldy1gz = 0;
	//任务处理单元2故障 4A9~10 bit1
	memcpy(&temp,&KKL_CCC_data_21_4[i].data[9],2);
	if(get_bit(temp,1) && t4a_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.rwcldy2gz = 1;
	else if(t4a_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.rwcldy2gz = 0;
	//磁探杆收回故障 3B2帧4 bit4
	if(get_int8_bit(KKL_CCC_data_21_3[i].data[4],4) && t3b2_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.ctgshgz = 1;
	else if(t3b2_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.ctgshgz = 0;
	//磁探杆伸出故障 3B2帧4 bit5
	if(get_int8_bit(KKL_CCC_data_21_3[i].data[4],5) && t3b2_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.ctgscgz = 1;
	else if(t3b2_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.a_message.ctgscgz = 0;
	/******************* prompt_message *******************/

	//雷达侦查故障 5帧6~7 bit4
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[6],2);
	if(get_bit(temp,4) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.ldzcgz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.ldzcgz = 0;
	//通信侦查故障 5帧6~7 bit5
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[6],2);
	if(get_bit(temp,5) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.txzcgz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.txzcgz = 0;
	//JIDS中继故障 5帧6~7 bit6
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[6],2);
	if(get_bit(temp,6) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.JIDSzjgz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.JIDSzjgz = 0;
	//906数据链故障 5帧6~7 bit7
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[6],2);
	if(get_bit(temp,7) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message._906sjlgz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message._906sjlgz = 0;
	//多信道通信中继1故障 5帧6~7 bit8
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[6],2);
	if(get_bit(temp,8) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.dxdtxzj1gz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.dxdtxzj1gz = 0;
	//多信道通信中继2故障 5帧6~7 bit9
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[6],2);
	if(get_bit(temp,9) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.dxdtxzj2gz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.dxdtxzj2gz = 0;
	//船舶自动识别故障 5帧6~7 bit10
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[6],2);
	if(get_bit(temp,10) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.cbzdsbgz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.cbzdsbgz = 0;
	//ADS-B故障 5帧8~9 bit12
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[8],2);
	if(get_bit(temp,12) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.ADS_bgz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.ADS_bgz = 0;
	//航管应答故障 5帧8~9 bit11
	memcpy(&temp,&KKL_CCC_data_21_5[i].data[8],2);
	if(get_bit(temp,11) && t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.hgydgz = 1;
	else if(t5_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.hgydgz = 0;
	//载荷数据处理单元故障 4A9~10 bit2
	memcpy(&temp,&KKL_CCC_data_21_4[i].data[9],2);
	if(get_bit(temp,2) && t4a_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.zhsjcldygz= 1;
	else if(t4a_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.zhsjcldygz = 0;
	//磁探不在线 4A15~16 bit14
	memcpy(&temp,&KKL_CCC_data_21_4[i].data[15],2);
	if(get_bit(temp,14) == 0 && t4a_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.ctbzx = 1;
	else if(t4a_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.ctbzx = 0;
	//红外传感器故障 2帧16~17 bit2~3
	if(KKL_CCC_data_21_2[i].GD_status1.hwcgq_status == 2 && t2_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.hwcgqgz = 1;
	else if(t2_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.hwcgqgz = 0;
	//电视传感器故障 2帧16~17 bit4~5
	if(KKL_CCC_data_21_2[i].GD_status1.dscgq_status == 2 && t2_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.dscgqgz = 1;
	else if(t2_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.dscgqgz = 0;
	//相机故障 2帧16~17 bit6~7
	if(KKL_CCC_data_21_2[i].GD_status1.xj_status == 2 && t2_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.xjgz = 1;
	else if(t2_flag[i] == 1)
		CCC_DPU_data_3.drone_specific_informations[i].fault_list.p_message.xjgz = 0;


//	//节拍加一
//	if(s6_hold_flag == 1)
//	{
//		//接收到编队保持
//		blk_ccc_ofp_026.temfly_status = 1;
//		blk_ccc_ofp_026.uav_id = blk_ofp_ccc_053.uavCode;
//		blk_ccc_ofp_026.uav_sn = blk_ofp_ccc_053.uavSn;
//		send_blk_ccc_ofp_026();
//		s6_hold_flag = 2;
//	}
//	else if(b2_frame_16[i] == 1)
//	{
//		//接收到16指令回复,成功
//		b2_frame_16[i] = 0;
//		lhtime[i] = 0;
//		start_lh_flag[i] = 0;
//		blk_ccc_ofp_026.temfly_status = 2;  //进入领航
//		blk_ccc_ofp_026.uav_id = blk_ofp_ccc_053.uavCode;
//		blk_ccc_ofp_026.uav_sn = blk_ofp_ccc_053.uavSn;
//		send_blk_ccc_ofp_026();
//	}
//	else if(b2_frame_16[i] == -1)
//	{
//		//接收到16指令回复,失败
//		b2_frame_16[i] = 0;
//		lhtime[i] = 0;
//		start_lh_flag[i] = 0;
//		blk_ccc_ofp_026.temfly_status = 3;  //领航失败
//		blk_ccc_ofp_026.uav_id = blk_ofp_ccc_053.uavCode;
//		blk_ccc_ofp_026.uav_sn = blk_ofp_ccc_053.uavSn;
//		send_blk_ccc_ofp_026();
//	}
//	else if(b2_frame_14[i] == 1)
//	{
//		start_tclh_flag[i] = 0;
//		b2_frame_14[i] = 0;
//		//接收到14指令回复_退出领航成功
//		blk_ccc_ofp_026.temfly_status = 5;//退出领航成功
//		blk_ccc_ofp_026.uav_id = blk_ofp_ccc_053.uavCode;
//		blk_ccc_ofp_026.uav_sn = blk_ofp_ccc_053.uavSn;
//		send_blk_ccc_ofp_026();
//	}
//	else if(b2_frame_14[i] == -1)
//	{
//		start_tclh_flag[i] = 0;
//		b2_frame_14[i] = 0;
//		//接收到14指令回复_退出领航成功
//		blk_ccc_ofp_026.temfly_status = 6;//退出领航失败
//		blk_ccc_ofp_026.uav_id = blk_ofp_ccc_053.uavCode;
//		blk_ccc_ofp_026.uav_sn = blk_ofp_ccc_053.uavSn;
//		send_blk_ccc_ofp_026();
//	}
//	else if(lhtime[i]>400)
//	{
//		lhtime[i] = 0;
//		start_lh_flag[i] = 0;
//		blk_ccc_ofp_026.temfly_status = 4;//领航超时
//		blk_ccc_ofp_026.uav_id = blk_ofp_ccc_053.uavCode;
//		blk_ccc_ofp_026.uav_sn = blk_ofp_ccc_053.uavSn;
//		send_blk_ccc_ofp_026();
//	}
//	else if(tclhtime[i]>400)
//	{
//		tclhtime[i] = 0;
//		start_tclh_flag[i] = 0;
//		blk_ccc_ofp_026.temfly_status = 7;//退出领航超时
//		blk_ccc_ofp_026.uav_id = blk_ofp_ccc_053.uavCode;
//		blk_ccc_ofp_026.uav_sn = blk_ofp_ccc_053.uavSn;
//		send_blk_ccc_ofp_026();
//	}

	//载荷遥测标志
	t1c_flag[i] = 0;
	t2_flag[i] = 0;
	t3b2_flag[i] = 0;
	t4a_flag[i] = 0;
	t5_flag[i] = 0;
}


// 接收 uav数据帧接口
void recv_blk_kkl_ccc_000_008_010_011()
{
#if SIMTESTFLAG

	static int timeoutFlag[UAV_MAX_NUM] = {0,0,0,0};
	int drone_number = 0;
	/******************* uav 1 *******************/
	message_size = 2048;
	// 接收无人机遥测数据帧
	Receive_Message(DDSTables.KKL_CCC_0.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		//C链接收
		decode_uav_frame(0);  // 解析出无人机 1 信息
		init_drone_state_information(0); // 初始化无人机状态信息 无人机 1
		drone_number++;
		uav_info_recv_flag = 1;
		timeoutFlag[0]  = 0;
	}
	else if(enRetCode != 0)
	{
		//U链接收
		Receive_Message(DDSTables.KKL_U_CCC_03.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
		if(enRetCode == 0)
		{
			decode_uav_frame(0);  // 解析出无人机 1 信息
			init_drone_state_information(0); // 初始化无人机状态信息 无人机 1
			drone_number++;
			uav_info_recv_flag = 1;
			timeoutFlag[0]  = 0;
		}
	}

	/******************* uav 2 *******************/
	message_size = 2048;
	Receive_Message(DDSTables.KKL_CCC_6.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		//C链接收
		decode_uav_frame(1);  // 解析出无人机 2 信息
		init_drone_state_information(1); // 初始化无人机状态信息 无人机 2
		drone_number++;
		uav_info_recv_flag = 1;
		timeoutFlag[1]  = 0;
	}
	else if(enRetCode != 0)
	{
		//U链接收
		Receive_Message(DDSTables.KKL_U_CCC_04.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
		if(enRetCode == 0)
		{
			decode_uav_frame(1);  // 解析出无人机 2 信息
			init_drone_state_information(1); // 初始化无人机状态信息 无人机 2
			drone_number++;
			uav_info_recv_flag = 1;
			timeoutFlag[1]  = 0;
		}
	}

	/******************* uav 3 *******************/
	message_size = 2048;
	Receive_Message(DDSTables.KKL_CCC_8.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		//C链接收
		decode_uav_frame(2);  // 解析出无人机 3 信息
		init_drone_state_information(2); // 初始化无人机状态信息 无人机 3
		drone_number++;
		uav_info_recv_flag = 1;
		timeoutFlag[2]  = 0;
	}
	else if(enRetCode != 0)
	{
		//U链接收
		Receive_Message(DDSTables.KKL_U_CCC_05.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
		if(enRetCode == 0)
		{
			decode_uav_frame(2);  // 解析出无人机 3 信息
			init_drone_state_information(2); // 初始化无人机状态信息 无人机 3
			drone_number++;
			uav_info_recv_flag = 1;
			timeoutFlag[2]  = 0;
		}
	}

	/******************* uav 4 *******************/
	message_size = 2048;
	Receive_Message(DDSTables.KKL_CCC_9.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		//C链接收
		decode_uav_frame(3);  // 解析出无人机 4 信息
		init_drone_state_information(3); // 初始化无人机状态信息 无人机 4
		drone_number++;
		uav_info_recv_flag = 1;
		timeoutFlag[3]  = 0;
	}
	else if(enRetCode != 0)
	{
		//U链接收
		Receive_Message(DDSTables.KKL_U_CCC_06.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
		if(enRetCode == 0)
		{
			decode_uav_frame(3);  // 解析出无人机 4 信息
			init_drone_state_information(3); // 初始化无人机状态信息 无人机 4
			drone_number++;
			uav_info_recv_flag = 1;
			timeoutFlag[3]  = 0;
		}
	}

	// 仿真环境超时判断
	const int TIMEOUTCOUNT = 5;// 判定超时次数
	CCC_DPU_data_3.drone_number = 0; // 数量清零重新计数
	for(int i = 0; i< UAV_MAX_NUM; i++)
	{
		// 超时计数自增
		timeoutFlag[i]++;

		// 超时判断
		if(timeoutFlag[i]>TIMEOUTCOUNT)
		{
			// 置为TIMEOUTCOUNT+1；避免自增溢出
			timeoutFlag[i]  = TIMEOUTCOUNT+1;
			memset(&CCC_DPU_data_3.drone_specific_informations[i] , 0, sizeof(drone_specific_information));
		}
		else
		{
			CCC_DPU_data_3.drone_number++;
		}

	}


#else

	/*******************先收u的遥测数据*******************/
	for(int i=0; i<UAV_MAX_NUM; i++)
	{
		message_size = 2048;
		SINT32_SIX niConnectionId = 0;
		switch (i)
		{
		case 0:
			niConnectionId = DDSTables.KKL_U_CCC_03.niConnectionId;
			break;
		case 1:
			niConnectionId = DDSTables.KKL_U_CCC_04.niConnectionId;
			break;
		case 2:
			niConnectionId = DDSTables.KKL_U_CCC_05.niConnectionId;
			break;
		case 3:
			niConnectionId = DDSTables.KKL_U_CCC_06.niConnectionId;
			break;
		default:
			break;
		}


		// 周期消息，循环接收，循环处理

		Receive_Message(niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
		if(enRetCode == 0)
		{
			//收到未知遥控帧，过滤其他不是遥测  20250727
			if(dds_data_rece.dataA[2] != 0x81)
			{
				continue;
			}
			//从遥测的82帧(遥测第47个字节后4位)取出飞机id（尾号，需加0x1000）(已约定)
			int id = 0x1000+(int)((dds_data_rece.dataA[46] & 0xf0)>>4);

			//			//暂时写死 20250729
			//			int id = UAV1_ID;
			//20250925 int index = formationIdManage(id);
			int index = -1;
			//飞控未升级临时处理 0x1000为1005
			if(id == 0x1005)
			{
				//1005id固定为序号1
				index = 0;
				formationId[index].planeId = 0x1005;
			}
			else if(id == 0x1006)
			{
				//1006id固定为序号2
				index = 1;
				formationId[index].planeId = 0x1006;
			}

			if(index != -1)
			{
				// 重置该飞机的超时计数
				formationIdResetCount(index);

				//更新链来源，U
				formationId[index].C_U = 2;

				// 如果u端机本机链路状态数据中的该飞机的u链上行传输速率是51.2kbps，那么就收320字节完整包。如果不是，则只收前160字节
				if(blk_ccc_ofp_199.ULparSet_1[i].UpRate_3 == 2) // ?后续需要用uavid，遍历blk_ccc_ofp_199.ULparSet的所有id，找到匹配的才可以
				{
					// 320
					memcpy(&blk_kkl_ccc_000_008_010_011[index], dds_data_rece.dataA, sizeof(BLK_KKL_CCC_000_008_010_011));
				}
				decode_uav_frame(index);  // 解析出无人机 1 信息
				fault_message_init(index);//解析无人机1故障信息
				init_drone_state_information(index); // 初始化无人机状态信息 无人机 1
			}
		}
	}


	/*******************再收c的，c的收到相同的则直接覆盖u的相同id即可*******************/
	for(int i=0; i<UAV_MAX_NUM; i++)
	{
		message_size = 2048;
		SINT32_SIX niConnectionId = 0;
		switch (i)
		{
		case 0:
			niConnectionId = DDSTables.KKL_CCC_0.niConnectionId;
			break;
		case 1:
			niConnectionId = DDSTables.KKL_CCC_6.niConnectionId;
			break;
		case 2:
			niConnectionId = DDSTables.KKL_CCC_8.niConnectionId;
			break;
		case 3:
			niConnectionId = DDSTables.KKL_CCC_9.niConnectionId;
			break;
		default:
			break;
		}
		Receive_Message(niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
		if(enRetCode == 0)
		{
			//收到未知遥控帧，过滤其他不是遥测  20250727
			if(dds_data_rece.dataA[2] != 0x81)
			{
				continue;
			}
			// 从遥测的82帧(遥测第47个字节后4位)取出飞机id（尾号，需加0x1000）(已约定)
			int id = 0x1000+(int)((dds_data_rece.dataA[46] & 0xf0)>>4);
			//			//暂时写死 20250729
			//			int id = UAV1_ID;
			//int index = formationIdManage(id);
			int index = -1;
			//飞控未升级临时处理 0x1000为1005
			if(id == 0x1005)
			{
				//1005id固定为序号1
				index = 0;
				formationId[index].planeId = 0x1005;
			}
			else if(id == 0x1006)
			{
				//1006id固定为序号2
				index = 1;
				formationId[index].planeId = 0x1006;
			}
			if(index != -1)// 编队中的飞机id
			{
				// 重置该飞机的超时计数
				formationIdResetCount(index);
				//更新链来源，C
				formationId[index].C_U = 1;
				decode_uav_frame(index);  // 解析出无人机 1 信息
				fault_message_init(index);//解析无人机1故障信息
				init_drone_state_information(index); // 初始化无人机状态信息 无人机 1
			}
		}
	}

	// 编队接收计数自增
	formationIdCountIncrease();

	/*****note:CCC_DPU_data_3消息的编队序号根据formationId的编队序号决定******/
	// 根据编队情况，将出编队的无人机信息清空
	CCC_DPU_data_3.drone_number = 0;
	for(int i=0; i< UAV_MAX_NUM; i++)
	{
		// 超时代表没有飞机或飞机不在编队
		if(formationId[i].count > formationIdTimeOutCount)
		{
			memset(&CCC_DPU_data_3.drone_specific_informations[i], 0, sizeof(drone_specific_information));
		}
		else
		{
			// 根据无人机编队情况计算无人机数量
			CCC_DPU_data_3.drone_number++;
		}
	}



#endif


}



/************************ 单无人机规划模块 ************************/
/*
 *  接收综显 单规划指令 + 任务区；根据指令划分为F型与J型机
 *  整理为战法指令发给辅助决策（指令只发一次）
 *  接收辅助决策信息： F： 接收无人机规划方案与航线   J:只接收无人机航航线
 *  将信息发给综显
 */
void single_uav_plan()
{
	static int single_uav_004_idle_cnt = 0;
	static unsigned int single_uav_stage_index = 0;
	//单无人机指控指令
	message_size = RECV_MAX_SIZE;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_11.niConnectionId,DDSTables.DPU2_CCC_11.niConnectionId,&DPU_CCC_data_11,sizeof DPU_CCC_data_11);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//		Receive_Message(DDSTables.PAD_CCC_025.niConnectionId, 0, &transaction_id, &DPU_CCC_data_11, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode == 0 && (single_uav_flag == 2 || single_uav_flag == 3))
	{
		enRetCode = -1;
	}
	if(enRetCode == 0)
	{
		scheme_generation_state(0,1,0,1);// 返回方案编辑状态到综显，方案生成中
		formulate_single = 0;
		single_uav_004_idle_cnt = 0;
		single_uav_stage_index = 0;
		//如果未加载返航航线，则生成失败
		int normal_cnt = 0;
		for(int i = 0 ; i < 4 ; i ++)
		{
			//无人机有返航航线且在线
			if(load_file.blk_dlr_ccc_045[i].normal_num > 0 && CCC_DPU_data_3.drone_specific_informations[i].platform_num == load_file.blk_dlr_ccc_045[i].uav_id)
			{
				normal_cnt++;
			}
		}
		if(CCC_DPU_data_3.drone_number > normal_cnt)
		{
			sprintf(CCC_DPU_data_0.failreason,"未检测到有效无人机返航航线");
			scheme_generation_state(2,2,0,3);// 生成失败
			memset(&CCC_DPU_data_0.failreason,0,sizeof CCC_DPU_data_0.failreason);
			single_uav_flag = 0;
			return;
		}

		if(DPU_CCC_data_11.mission_type == 11)
		{
			//返航标志位
			single_uav_flag = 1;
			return;
		}
		g_single_uav_request_plan_id = alloc_single_request_plan_id();
		dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_011.niConnectionId);
		dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_002.niConnectionId);
		dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_004.niConnectionId);
		dtms_flush_dds_topic(DDSTables.DPU_CCC_35.niConnectionId);
		dtms_flush_dds_topic(DDSTables.DPU2_CCC_35.niConnectionId);
		int selected_uav_index = DPU_CCC_data_11.drone_num - 1;
		if(selected_uav_index >= 0 && selected_uav_index < UAV_MAX_NUM)
		{
			memset(&blk_ccc_ofp_024_single[selected_uav_index],0,sizeof(blk_ccc_ofp_024_single[selected_uav_index]));
		}
		if(DPU_CCC_data_11.mission_object_type == 1)//任务区
		{
			uav_insert_cover = DPU_CCC_data_11.mission_approach;
			init_single_uav_zhanfa();//初始化单无人机指令信息
			send_single_uav_zhanshutuijian();//发送单无人机指控指令
			single_uav_flag = 2;
		}
		else if(DPU_CCC_data_11.mission_object_type == 2)//目标
		{
			uav_insert_cover = DPU_CCC_data_11.mission_approach;
			init_single_uav_zhanfa();//初始化单无人机指令信息
			send_single_uav_zhanshutuijian();//发送单无人机指控指令
			single_uav_flag = 2;
		}
		else if(DPU_CCC_data_11.mission_object_type == 3 || DPU_CCC_data_11.mission_object_type == 4)//光标选点/航路点
		{
			uav_insert_cover = DPU_CCC_data_11.mission_approach;
			init_single_uav_zhanfa();//初始化单无人机指令信息
			send_single_uav_zhanshutuijian();//发送单无人机指控指令
			single_uav_flag = 2;
		}
	}
	if(single_uav_flag == 2)
	{
		message_size = RECV_MAX_SIZE;
		int area_rtn;
		Receive_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, &message_size, &enRetCode);
		if(enRetCode == 0)
		{
			if(area_rtn < 0)
			{
				if(area_rtn == -1)
				{
					sprintf(CCC_DPU_data_0.failreason,"任务目标点超过许可空域范围");
				}
				else if(area_rtn == -2)
				{
					sprintf(CCC_DPU_data_0.failreason,"任务目标点距离许可空域边界过近");
				}
				else if(area_rtn == -3)
				{
					sprintf(CCC_DPU_data_0.failreason,"任务范围区域过小，转弯受限");
				}
				else if(area_rtn == -4)
				{
					sprintf(CCC_DPU_data_0.failreason,"任务区域范围过大，超出侦听距离");
				}
				else if(area_rtn == -5)
				{
					sprintf(CCC_DPU_data_0.failreason,"集结距离小于10km");
				}
				scheme_generation_state(0,3,0,0);// 返回方案编辑状态到综显，生成失败
				memset(&CCC_DPU_data_0.failreason,0,sizeof CCC_DPU_data_0.failreason);
				single_uav_flag = 0;
				return;
			}
		}
		//收到单任务区/目标任务分配结果CTAS->DTMS
		message_size = RECV_MAX_SIZE;
		Receive_Message(DDSTables.BLK_CTAS_DTMS_002.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
		if(enRetCode == 0)
		{
			memcpy(&blk_ccc_ofp_019,dds_data_rece.dataA,sizeof blk_ccc_ofp_019);
			unsigned int expected_plan_id = g_single_uav_request_plan_id;
			if(expected_plan_id == 0)
			{
					expected_plan_id = (planning_id != 0) ? planning_id : 3;
			}
			if(blk_ccc_ofp_019.plan_id != expected_plan_id)
			{
				printf("ignore single_uav result plan_id=%u expected=%u\n",blk_ccc_ofp_019.plan_id,expected_plan_id);
			}
			else
			{
			//方案中的无人机位置与收到的方案的无人机索引不一样，需要计算
			int plan_uav = DPU_CCC_data_11.drone_num;//无人机序号加有人机位置加一再减一，得到方案的无人机索引下标
			//找到当前运行的阶段对应的任务下标
			unsigned int num = (global_stage > 0) ? (global_stage - 1) : 0;
			single_uav_stage_index = 0;
			//索引无人机,CTAS内部无人机是紧凑排列，如果有无人机下线，需要重新查找下标，找到无人机对应关系
			for(int i = 1 ; i < 5 ; i ++)
			{
				if(blk_ccc_ofp_019.formation_synergy_mission_programs[i].platform_code == DPU_CCC_data_11.drone_id)
				{
					//整理预览方案，把平台信息放到无人机位置，防止无人机下线，索引不一致
					memcpy(&blk_ccc_ofp_019.formation_synergy_mission_programs[plan_uav],
							&blk_ccc_ofp_019.formation_synergy_mission_programs[i],
							22);
					if(i != plan_uav)
						memset(&blk_ccc_ofp_019.formation_synergy_mission_programs[i],0,22);

					//整理预览方案，把任务放到当前阶段的位置
					memcpy(&blk_ccc_ofp_019.formation_synergy_mission_programs[plan_uav].task_sequence_informations[num],
							&blk_ccc_ofp_019.formation_synergy_mission_programs[i].task_sequence_informations[0],
							sizeof(task_sequence_information));
					if(i != plan_uav)
						memset(&blk_ccc_ofp_019.formation_synergy_mission_programs[i].task_sequence_informations[0],0,sizeof(task_sequence_information));
				}
			}
			// 发送单无人机务分配结果
			send_blk_ccc_ofp_021();
			single_uav_004_idle_cnt = 0;
			single_uav_flag = 3;
		}
	}
	}
	if(single_uav_flag == 3)
	{
		int route_recv_ok = 0;
		//单任务航线最多发三包，收三次
		for(int i = 0; i < 20 ;i ++)
		{
			BLK_CCC_OFP_024 temp;
			message_size = RECV_MAX_SIZE;
			Receive_Message(DDSTables.BLK_CTAS_DTMS_004.niConnectionId, 0, &transaction_id, &temp, &message_type_id, &message_size, &enRetCode);
			if(enRetCode==0)
			{
				unsigned int expected_plan_id = g_single_uav_request_plan_id;
				if(expected_plan_id == 0)
				{
					expected_plan_id = (planning_id != 0) ? planning_id : 3;
				}
				if(temp.program_number != expected_plan_id && temp.uav_plan_id != expected_plan_id)
				{
					continue;
				}
				// Normalize ids for DPU preview/update matching across multiple single-uav runs.
				temp.program_number = expected_plan_id;
				temp.uav_plan_id = expected_plan_id;
				int selected_uav_index = DPU_CCC_data_11.drone_num - 1;
				if(selected_uav_index < 0 || selected_uav_index >= UAV_MAX_NUM)
				{
					continue;
				}
				unsigned int id = (unsigned int)selected_uav_index;
				unsigned int index = temp.individual_drone_routing_programs.planning_informations.packet_id *25;
				temp.individual_drone_routing_programs.drone_serial_number = (unsigned int)(selected_uav_index + 1);
				temp.individual_drone_routing_programs.drone_num = DPU_CCC_data_11.drone_id;

				//无人机任务序号为0
				temp.individual_drone_routing_programs.subtask_index = single_uav_stage_index;
				// 保存头信息
				memcpy(&blk_ccc_ofp_024_single[id],&temp,9 + 10 + 26);
				// 保存航路点信息
					memcpy(&blk_ccc_ofp_024_single[id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index],
						   &temp.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
				       sizeof(planning_information_waypoint_information)*25);
				//发送航线到综显
				data_length = sizeof(BLK_CCC_OFP_024);
				Send_Message(DDSTables.CCC_DPU_11.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);
				if(enRetCode == 0)
				{
					printf("send single_uav success!!!\n");
				}
				route_recv_ok = 1;
				//发送给pad new20250620
				//				Send_Message(DDSTables.CCC_PAD_024.niConnectionId,0,&transaction_id, &blk_ccc_ofp_024,&message_type_id, data_length, &enRetCode);
			}
		}

		if(route_recv_ok > 0)
		{
			single_uav_004_idle_cnt = 0;
		single_uav_flag = 0;
		scheme_generation_state(0,2,0,2);// 返回方案编辑状态到综显，航线发布完成
		}
		else
		{
			single_uav_004_idle_cnt++;
			if(single_uav_004_idle_cnt > 80)
			{
				sprintf(CCC_DPU_data_0.failreason,"单无人机航线接收超时");
				scheme_generation_state(0,3,0,0);// 返回方案编辑状态到综显，生成失败
				memset(&CCC_DPU_data_0.failreason,0,sizeof CCC_DPU_data_0.failreason);
				single_uav_004_idle_cnt = 0;
				single_uav_flag = 0;
			}
		}
	}
	//接收单无人机发布指令
	message_size = RECV_MAX_SIZE;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_35.niConnectionId,DDSTables.DPU2_CCC_35.niConnectionId,&blk_ofp_ccc_043,sizeof blk_ofp_ccc_043);
	if(enRetCode == 0)
	{
		if(single_uav_flag != 2 && single_uav_flag != 3)
		{
		//发布申请
		if(blk_ofp_ccc_043.plan_state == 4)
		{
			single_uav_flag = 11;
		}
		//修改
		else if(blk_ofp_ccc_043.plan_state == 3)
		{
			single_uav_flag = 22;
			}
		}
	}
	//航线注入
	if(single_uav_flag == 11)
	{
		//        if(DPU_CCC_data_11.mission_type == 12)
		//        {
		//            single_uav_XT();//悬停注入
		//        }
		//        else
		if(DPU_CCC_data_11.mission_type == 9)
		{
			//发布变量重置
			int uav_index = DPU_CCC_data_11.drone_num -1;
			b2_frame_30[uav_index] = 0;
			b2_frame_14[uav_index] = 0;
			s4D_frame_40[uav_index] = 0;
			bdfx_single_expect_route[uav_index] = 0;
			bdfx_single_uav_index = uav_index;
			send_cnt2[uav_index] = 0;
			//开始装订航线
			BDFX_status = 1;
			scheme_generation_state(0,2, 1, 2); // 返回发布状态到综显
			single_uav_flag = 0;
		}
		else
		{
			//其他类型注入

			//覆盖
			if(uav_insert_cover == 2)
			{
				//发布航线时停止查询
				hx_cx_flag = 0;
				//注入无人机航线
				cover_inject(DPU_CCC_data_11.drone_num - 1);
			}
		}

	}
	//单无人机编辑
	if(single_uav_flag == 22)
	{
		static unsigned int single_edit_wait_cnt = 0;
		int edit_ok = 0;
		if(single_edit_wait_cnt == 0)
		{
			send_blk_ccc_ofp_020(DPU_CCC_data_5.program_number,1,1,NULL);
		}
		for(int i = 0 ; i < 20 ; i ++)
		{
			if(edit_uav_route(i % 3) == 1)
			{
				edit_ok = 1;
			}
		}
		if(edit_ok == 1)
		{
			send_blk_ccc_ofp_020(DPU_CCC_data_5.program_number,1,2,NULL);
			single_uav_flag = 0;
			single_edit_wait_cnt = 0;
		}
		else
		{
			single_edit_wait_cnt++;
			if(single_edit_wait_cnt > 100)
			{
				send_blk_ccc_ofp_020(DPU_CCC_data_5.program_number,1,2,NULL);
				single_uav_flag = 0;
				single_edit_wait_cnt = 0;
			}
		}
	}
}
int edit_uav_route(int i)
{
	BLK_CCC_OFP_024 uav;
	static unsigned int id = 0;
	static unsigned int uav_id = 0;
	int recv_ok = 0;
	// DPU_DTMS 无人机航路修改保存
	recv_dpu1_dpu2(DDSTables.DPU_CCC_10.niConnectionId,DDSTables.DPU2_CCC_10.niConnectionId,&uav,sizeof uav);
	if(enRetCode == 0)
	{
		recv_ok = 1;
		uav_id = uav.individual_drone_routing_programs.drone_num;
		id = (uav.individual_drone_routing_programs.drone_serial_number - 1);
		unsigned int index = uav.individual_drone_routing_programs.planning_informations.packet_id *25;
		// 保存头信息
		memcpy(&blk_ccc_ofp_024_single[id],&uav,9 + 10 + 26);
		// 保存航路点信息
		memcpy(&blk_ccc_ofp_024_single[id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index], &uav.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], sizeof(planning_information_waypoint_information) * 25);

		//航线最后一个点的待机参数返回默认65535 20250823new
		if(uav.individual_drone_routing_programs.planning_informations.total_packet - 1 == uav.individual_drone_routing_programs.planning_informations.packet_id)
		{
			int num_index = (uav.individual_drone_routing_programs.planning_informations.waypoints_number % 25) -1;
			//默认65535
			uav.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[num_index].standby_time_lapsNumber_cycleNumber = 65535;
		}
		//回复给综显，修改的无人机航线
		data_length = sizeof(BLK_CCC_OFP_024);
		Send_Message(DDSTables.CCC_DPU_11.niConnectionId,0,&transaction_id, &uav.program_number, &message_type_id, data_length, &enRetCode);
	}
	//接收完修改信息后发送到CTAS重新生成空域
	if(i == 2 && uav_id != 0)
	{
		//发送修改无人机的所有航线到CTAS
		for(unsigned char j = 0 ;j < blk_ccc_ofp_024_single[id].individual_drone_routing_programs.planning_informations.total_packet ; j++)
		{
			BLK_CCC_OFP_024 temp;
			//取头
			memcpy(&temp,&blk_ccc_ofp_024_single[id],9 + 10 + 26);
			//取航路点
			memcpy(&temp.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], &blk_ccc_ofp_024_single[id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j * 25], sizeof(planning_information_waypoint_information) * 25);
			//包序号赋值
			temp.individual_drone_routing_programs.planning_informations.packet_id = j;
			// 综显发送
			data_length = sizeof(BLK_CCC_OFP_024);
			Send_Message(DDSTables.BLK_DTMS_CTAS_008.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);
		}
		//清空变量
		uav_id = 0;
	}
	return recv_ok;
}
//覆盖注入航线
void cover_inject(int uav_index)
{
	scheme_generation_state(0,2,1,2);// 返回发布状态到综显
	//找到方案编号索引，没有运行中的方案plan为零
	unsigned int plan_id = g_single_uav_request_plan_id;
	if(plan_id == 0)
	{
		plan_id = get_single_request_plan_id();
	}
	unsigned int plan = plan_id % 3;
	//单无人机插入，复用航线注入模块，改变方案变量
	blk_ofp_ccc_039.Plan_ID = plan_id;
	blk_ofp_ccc_039.routeType = 1;
	//初始化无人机数据,接收标志
	for(int i = 0 ; i < 4 ; i ++)
	{
		memset(&uav_send[i],0,sizeof(UAV_SEND));
		memset(&s4D_frame_40[i],0,sizeof(int));
		memset(&s4D_frame_38[i],0,sizeof(int));
		send_area[i] = 0;
	}
	//存入任务分配结果
	memcpy(&CCC_DPU_data_6_Ofp[plan],&blk_ccc_ofp_019,sizeof(BLK_CCC_OFP_019));
	memcpy(&CCC_DPU_data_6[plan],&blk_ccc_ofp_019,sizeof(BLK_CCC_OFP_019));
	//存入单无人机航线
	memcpy(&blk_ccc_ofp_024_cunchu[plan][uav_index],&blk_ccc_ofp_024_single[uav_index],sizeof(BLK_CCC_OFP_024_cunchu));
	//获取单无人机编号
	uav_send[uav_index].drone_id = DPU_CCC_data_11.drone_id;
	//航路点数量
	uav_send[uav_index].waypoints_number = blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.waypoints_number;
	//存入当前任务的每个航路点经纬度
	memcpy(&uav_send[uav_index].waypoint_informations[0],
			&blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
			sizeof(planning_information_waypoint_information)*uav_send[uav_index].waypoints_number);
	//发送准备
	uav_send[uav_index].send_ready = 1;
	uav_send[uav_index].first_flag = 0;
	//找到任务类型参数索引
	for(int j = 0;j < 8;j++)
	{
		if(task_param[j].type_id ==
				blk_ccc_ofp_021.plan_info[uav_index].task_sequence_informations[0].type)
		{
			//找到索引退出
			uav_send[uav_index].task_index = j;
			break;
		}
	}
	//判断该无人机是否有航路点，没有航路点则放弃发送
	if(uav_send[uav_index].waypoints_number == 0)
	{
		//清空该无人机数据
		memset(&uav_send[uav_index],0,sizeof(UAV_SEND));
	}
	//初始化发送参数，防止上次指令切换之后的帧变化影响
	frame_count = 1;
	track_point_count = -1;
	send_count = 0;
	g_fabu_send_uav_num = 0;
	g_fabu_switch_timeout = 0;
	g_fabu_switch_ack_cnt = 0;
	single_uav_flag = 33;
}
void single_uav_BDFX()
{
	//无人机编队指令帧临时定义 100~143 数据库管理帧
	int uav_index = bdfx_single_uav_index;
	if(uav_index < 0 || uav_index >= UAV_MAX_NUM)
	{
		uav_index = DPU_CCC_data_11.drone_num -1;
	}
	if(uav_index < 0 || uav_index >= UAV_MAX_NUM)
	{
		return;
	}
	//发送七个航点(新增倒数第二个计算点 20251019new，中点与解散点之间新增一个点)，每个航点发送四次
	static char hx_point = 0;
	static int cnt_0x30 = 10;
	if(b2_frame_30[uav_index] != 0)
	{
		//发布失败
		BDFX_status = 0;
		//回收变量
		hx_point = 0;
		cnt_0x30 = 10;
		//发送发布失败状态到综显
		sprintf(CCC_DPU_data_0.failreason, "发送编队航线装订指令返回失败");
		scheme_generation_state(0,2, 3, 2);
		memset(CCC_DPU_data_0.failreason, 0, 200);
	}

	//航路点基础信息赋值
	//编队航点装订
	blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[0] = 0x30;
	blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[1] = 0x30;
	blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[2] = 0x30;
	//0x30指令数据
	tail_0x30 temp;
	memset(&temp,0,sizeof(tail_0x30));
	//航线号
	temp.hx = 48;
	if(uav_route[uav_index].route_number == 48)
	{
		temp.hx++;
	}

	//机场点赋值
	if(hx_point == 0 || hx_point == 8)
	{
		//高度速度
		temp.height = 160 / gaodu_scale;
		temp.speed = 0/ speed_scale;
		temp.hd = hx_point+1;
		//20260201  05机使用原机场点，新增06机场点
		temp.lat = get_airport_lat(uav_index) / lat_scale;//260201
		temp.lon = get_airport_lon(uav_index) / lon_scale;//260201
		//机场点
		temp.tezhenzi[0] = 0x07;
		cnt_0x30--;
	}
	else if(hx_point == 7)
	{
		//高度速度
		temp.height = 360 / gaodu_scale;
		temp.speed = 25/ speed_scale;

		//倒数第二个计算点，第六个点
		temp.hd = hx_point+1;
		temp.tezhenzi[0] = 0;

		//计算倒数第二个点
		unsigned int id = (unsigned int)uav_index;
		Point last_second;
		//260201  06机使用新机场点
		last_second = last_second_point(get_airport_lat(uav_index),get_airport_lon(uav_index),
				blk_ccc_ofp_024_single[id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[hx_point -2].latitude,
				blk_ccc_ofp_024_single[id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[hx_point -2].longitude);
		temp.lat = last_second.lat / lat_scale;
		temp.lon = last_second.lon / lon_scale;
		cnt_0x30--;
	}
	else if(hx_point == 9)
	{
		//发送完毕退出装订
		hx_point = 0;
		BDFX_status = 2;
	}
	else
	{
		temp.hd = hx_point+1;
		unsigned int id = (unsigned int)uav_index;
		temp.lat =
				blk_ccc_ofp_024_single[id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[hx_point -1].latitude / lat_scale;
		temp.lon =
				blk_ccc_ofp_024_single[id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[hx_point -1].longitude / lon_scale;
		temp.height = blk_ccc_ofp_024_single[id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[hx_point -1].height / gaodu_scale;
		temp.speed = 35/ speed_scale;
		if(hx_point == 1)
		{
			//第二个航点
			temp.tezhenzi[0] = 0x02;
			short rad = 1500;
			memcpy(&temp.tezhenzi[1],&rad,2);
			short circle = 200;
			memcpy(&temp.tezhenzi[3],&circle,sizeof(short));
//			temp.tezhenzi[3] = 0xFF;
//			temp.tezhenzi[4] = 0xFF;
			temp.tezhenzi[5] = 0;
			temp.tezhenzi[6] = 0;
			temp.tezhenzi[7] = 0x04;
			temp.group_id = 5;//群组
			temp.team_id = 1;//队形
			temp.task_type = 0x01;//编队集结
		}
		else if(hx_point == 2)
		{
			//第三个航点
			temp.tezhenzi[0] = 0;
			temp.group_id = 5;//群组
			temp.team_id = 2;//队形
			temp.task_type = 0x02;//队形变换点
		}
		else if(hx_point == 3 || hx_point == 4 || hx_point == 5)
		{
			//第四个航点 中点与解散点之间的点 20251115new
			temp.tezhenzi[0] = 0;
			temp.group_id = 5;//群组
			temp.team_id = 2;//队形
		}
		else if(hx_point == 6)
		{
			//第五个航点
			temp.tezhenzi[0] = 0x02;
			short rad = 1500;
			memcpy(&temp.tezhenzi[1],&rad,2);
			short circle = 200;
			memcpy(&temp.tezhenzi[3],&circle,sizeof(short));
//			temp.tezhenzi[3] = 0xFF;
//			temp.tezhenzi[4] = 0xFF;
			temp.tezhenzi[5] = 0;
			temp.tezhenzi[6] = 0;
			temp.tezhenzi[7] = 0x04;
			temp.group_id = 5;//群组
			temp.task_type = 0x03;//恢复航线，编队解散点
		}
		cnt_0x30--;
	}
	//每个点发四次
	if(cnt_0x30 < 0)
	{
		hx_point++;
		cnt_0x30 = 10;
	}
	//赋值后拷贝进数据库帧中
	memcpy(&blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order_data,&temp ,sizeof(tail_0x30));

}

void single_uav_BDON()
{
	//无人机编队指令帧临时定义 100~143 数据库管理帧
	//航路点基础信息赋值  //0924调试修改
	int uav_index = bdfx_single_uav_index;
	if(uav_index < 0 || uav_index >= UAV_MAX_NUM)
	{
		uav_index = DPU_CCC_data_11.drone_num -1;
	}
	if(uav_index < 0 || uav_index >= UAV_MAX_NUM)
	{
		return;
	}
	int route_number = 48;
	if(uav_route[uav_index].route_number == 48)
	{
		route_number++;
	}
	if(send_cnt2[uav_index] < 3)
	{
		bdfx_single_expect_route[uav_index] = route_number;
		order_data_frames.track_point_chage_orders.track_line_id = route_number;
		order_data_frames.track_point_chage_orders.track_point_id = 2;
		track_threat_frames.order_code[0] = 0x40;
		track_threat_frames.order_code[1] = 0x40;
		track_threat_frames.order_code[2] = 0x40;
		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data),&(order_data_frames.track_point_chage_orders),sizeof(order_data_frames.track_point_chage_orders));
		send_index = uav_index;

		send_cnt2[uav_index]++;
	}
	if(send_cnt2[uav_index] >= 3)
	{
		BDFX_status = 3;
		send_cnt2[uav_index] = 0;
		send_index = uav_index;
	}
}

void double_uav_BDFX(int uav_index)
{
	if(uav_index < 0 || uav_index >= 2)
	{
		return;
	}
	if(bdfx_double_done[uav_index])
	{
		return;
	}

	//发送九个航点(新增倒数第二个计算点 20251019new，中点与解散点之间新增一个点)，每个航点发送四次
	char *hx_point = &bdfx_double_hx_point[uav_index];
	int *cnt_0x30 = &bdfx_double_cnt_0x30[uav_index];
	if(b2_frame_30[uav_index] != 0)
	{
		//发布失败
		BDFX_double_status = 0;
		for(int i = 0 ; i < 2 ; i++)
		{
			bdfx_double_hx_point[i] = 0;
			bdfx_double_cnt_0x30[i] = 4;
			bdfx_double_done[i] = 0;
		}
		//发送发布失败状态到综显
		sprintf(CCC_DPU_data_0.failreason, "发送编队航线装订指令返回失败");
		scheme_generation_state(0,2, 3, 2);
		memset(CCC_DPU_data_0.failreason, 0, 200);
		return;
	}

	//航路点基础信息赋值
	//编队航点装订
	blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[0] = 0x30;
	blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[1] = 0x30;
	blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[2] = 0x30;
	//0x30指令数据
	tail_0x30 temp;
	memset(&temp,0,sizeof(tail_0x30));
	//航线号
	temp.hx = 48;
	if(uav_route[uav_index].route_number == 48)
	{
		temp.hx++;
	}

	//机场点赋值
	if((*hx_point) == 0 || (*hx_point) == 8)
	{
		//高度速度
		temp.height = 25 / gaodu_scale;
		temp.speed = 0/ speed_scale;
		temp.hd = (*hx_point)+1;
		temp.lat = get_airport_lat(uav_index) / lat_scale;//260201
		temp.lon = get_airport_lon(uav_index) / lon_scale;//260201
		temp.group_id = 5;//群组
		temp.team_id = 3;//队形
		//机场点
		temp.tezhenzi[0] = 0x07;
		(*cnt_0x30)--;
	}
	else if((*hx_point) == 7)
	{
		//高度速度
		if(CCC_DPU_data_3.drone_specific_informations[uav_index].platform_num == load_file.lead_uav_id)
			temp.height = 225 / gaodu_scale;
		else
			temp.height = 425 / gaodu_scale;
		temp.speed = 25/ speed_scale;

		//倒数第二个计算点，第八个点
		temp.hd = (*hx_point)+1;
		temp.tezhenzi[0] = 0;
		temp.group_id = 5;//群组
		temp.team_id = 3;//队形
		//计算倒数第二个点
			unsigned int plan = g_bdfx_double_plan_id % 3;
			Point last_second;
		//260201
		last_second = last_second_point(get_airport_lat(uav_index),get_airport_lon(uav_index),
				blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[(*hx_point) -2].latitude,
				blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[(*hx_point) -2].longitude);
		temp.lat = last_second.lat / lat_scale;
		temp.lon = last_second.lon / lon_scale;
		(*cnt_0x30)--;
	}
	else if((*hx_point) == 9)
	{
		//发送完毕退出装订
		(*hx_point) = 0;
		(*cnt_0x30) = 4;
		bdfx_double_done[uav_index] = 1;
		return;
	}
	else
	{
		temp.hd = (*hx_point)+1;
			unsigned int plan = g_bdfx_double_plan_id % 3;
			temp.lat =
				blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[(*hx_point) -1].latitude / lat_scale;
		temp.lon =
				blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[(*hx_point) -1].longitude / lon_scale;
		//编队飞行高度：长机460，僚机660（按平台号判断，不依赖数组顺序）
		if(CCC_DPU_data_3.drone_specific_informations[uav_index].platform_num == load_file.lead_uav_id)
			temp.height = 460 / gaodu_scale;
		else
			temp.height = 660 / gaodu_scale;
		temp.speed = 35/ speed_scale;
		if((*hx_point) == 1)
		{
			//第二个航点，盘旋点
			temp.tezhenzi[0] = 0x02;
			short rad = 1000;
			memcpy(&temp.tezhenzi[1],&rad,2);
			short circle = 20;
			memcpy(&temp.tezhenzi[3],&circle,sizeof(short));
//			temp.tezhenzi[3] = 0xFF;
//			temp.tezhenzi[4] = 0xFF;
			temp.tezhenzi[5] = 0;
			temp.tezhenzi[6] = 0;
			temp.tezhenzi[7] = 0x04;
			temp.group_id = 5;//群组
			temp.team_id = 3;//队形
		}
		else if((*hx_point) == 2)
		{
			//第三个航点，过顶点
			temp.tezhenzi[0] = 0x01;
			temp.group_id = 5;//群组
			temp.team_id = 3;//队形
			temp.task_type = 0x1;//过顶点
		}
		else if((*hx_point) == 3 || (*hx_point) == 4 || (*hx_point) == 5)
		{
			//第四~六个航点 队形保持点
			temp.tezhenzi[0] = 0;
			temp.group_id = 5;//群组
			temp.team_id = 3;//队形
		}
		else if((*hx_point) == 6)
		{
			//第七个航点
			temp.tezhenzi[0] = 0x02;
			short rad = 1200;//260131
			memcpy(&temp.tezhenzi[1],&rad,2);
			short circle = 20;
			memcpy(&temp.tezhenzi[3],&circle,sizeof(short));
//			temp.tezhenzi[3] = 0xFF;
//			temp.tezhenzi[4] = 0xFF;
			temp.tezhenzi[5] = 0;
			temp.tezhenzi[6] = 0;
			temp.tezhenzi[7] = 0x04;
			temp.group_id = 5;//群组
			temp.team_id = 3;//队形
			temp.task_type = 0x03;//恢复航线，编队解散点
		}
		(*cnt_0x30)--;
	}
	//每个点发四次
	if((*cnt_0x30) < 0)
	{
		(*hx_point)++;
		(*cnt_0x30) = 4;
	}
	//赋值后拷贝进数据库帧中
	memcpy(&blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order_data,&temp ,sizeof(tail_0x30));

}
void double_uav_BDON(int uav_index)
{
	//无人机编队指令帧临时定义 100~143 数据库管理帧
	int route_number = 48;
	if(uav_route[uav_index].route_number == 48)
	{
		route_number++;
	}
	if(send_cnt2[uav_index] < 3)
	{
		bdfx_double_expect_route[uav_index] = route_number;
		if (send_cnt2[uav_index] == 0)
		{
			printf("BDFX send 0x40 uav=%d target_route=%d\n", uav_index, route_number);
		}
		order_data_frames.track_point_chage_orders.track_line_id = route_number;
		order_data_frames.track_point_chage_orders.track_point_id = 2;
		track_threat_frames.order_code[0] = 0x40;
		track_threat_frames.order_code[1] = 0x40;
		track_threat_frames.order_code[2] = 0x40;
		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data),&(order_data_frames.track_point_chage_orders),sizeof(order_data_frames.track_point_chage_orders));
		send_index = uav_index;

		send_cnt2[uav_index]++;
	}
	if(send_cnt2[uav_index] >= 3)
	{
		send_cnt2[uav_index] = 0;
		send_index = uav_index;
	}
}
void single_uav_pxlh(unsigned short uav_index)
{
	//无人机编队指令帧临时定义 100~143 数据库管理帧
	//航路点基础信息赋值
	//退出领航
	send_cnt1[uav_index]++;
	if(send_cnt1[uav_index] < 20)
	{
		/* blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[0] = 0x14;
	  blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[1] = 0x14;
	  blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[2] = 0x14;

	  blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order_data[0] = 50;
	  blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order_data[1] = 4;*/
		send_cnt1[uav_index]++;
	}
	if(send_cnt1[uav_index] > 20)
	{
		BDFX_status = 0;
		send_cnt1[uav_index] = 0;
	}
}

void single_uav_lhcs(unsigned short uav_index)
{
	//无人机编队指令帧临时定义 100~143 数据库管理帧
	//航路点基础信息赋值
	//编队航点装订
	//
	//    blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[0] = 0x14;
	//        blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[1] = 0x30;
	//        blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order[2] = 0x30;
	//
	//        blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order_data[0] = 50;
	//        blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order_data[1] = 2;

	//0x16指令数据
	/*tail_0x16 temp;
	memset(&temp,0,sizeof(tail_0x16));
	temp.high_cmd = 400 / 6000 * 65535;
	temp.hori_distance = 6000 / 10000 * 65535;*/
	send_cnt[uav_index]++;
	//每个点发四次
	if(send_cnt[uav_index] > 20)
	{
		start_lh_flag[uav_index] = 0;
		send_cnt[uav_index] = 0;
	}
	//赋值后拷贝进数据库帧中
	//memcpy(&blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.order_data,&temp ,sizeof(tail_0x16));

}

//void single_uav_XT()
//{
//    //单无人机航线注入
//    static int xuanting_flag = 0;
//    //任务类型为悬停
//    if(xuanting_flag == 0)
//    {
//        scheme_generation_state(2,1,2);// 返回发布状态到综显
//        //初始化发送参数，防止上次指令切换之后的帧变化影响
//        frame_count = 1;
//        track_point_count = -1;
//        send_count = 0;
//        xuanting_flag = 1;
//    }
//    //悬停点注入
//    if(xuanting_flag == 1)
//    {
//        static int count = 0;
//        if(count < 4)
//        {
//            send_uav_hl(DPU_CCC_data_11.drone_id,0x30);//发送悬停点指令
//            count++;
//        }
//        if(count == 3)
//        {
//            //发送四拍完成
//            xuanting_flag = 2;
//            count = 0;
//        }
//    }
//    //悬停注入判断返回值
//    if(xuanting_flag == 2)
//    {
//        static int timeout = 0;
//        timeout ++;
//        //注入成功
//        if(s82_frame_30[DPU_CCC_data_11.drone_num] == 1)
////        if(1)
//        {
//            //清空遥调回报
//            s82_frame_30[DPU_CCC_data_11.drone_num] = 0;
//            //保存运行方案id
//            planning_id = blk_ofp_ccc_043.plan_id;
//            //取出保存的方案，发送运行方案到综显
////            memcpy(&blk_ccc_ofp_017,&blk_ccc_ofp_019,sizeof(BLK_CCC_OFP_017));
////            send_blk_ccc_ofp_017();
//            // 发送单无人机务运行分配结果
//            blk_ccc_ofp_019.plan_release_mode = 1;//已发布
//            send_blk_ccc_ofp_021();
//            // 再次发送无人机航线（运行方案）
//            data_length = sizeof(BLK_CCC_OFP_024);
//            Send_Message(DDSTables.CCC_DPU_11.niConnectionId,0,&transaction_id, &blk_ccc_ofp_024_single, &message_type_id, data_length, &enRetCode);
//            scheme_generation_state(2,2,2);// 发布成功
//            xuanting_flag = 0;
//            timeout = 0;
//            single_uav_flag = 0;
//            //应用空域
//            init_blk_ctas_dtms_010();
//        }
//        else if(s82_frame_30[DPU_CCC_data_11.drone_num] == -1)
//        {
//            //清空遥调回报
//            s82_frame_30[DPU_CCC_data_11.drone_num] = 0;
//            sprintf(CCC_DPU_data_0.failreason,"发送悬停指令返回失败");
//            scheme_generation_state(2,3,2);// 发布失败
//            memset(CCC_DPU_data_0.failreason,0,200);
//            xuanting_flag = 0;
//            timeout = 0;
//        }
//        else if(timeout > 100)
//        {
//            //清空遥调回报
//            s82_frame_30[DPU_CCC_data_11.drone_num] = 0;
//            sprintf(CCC_DPU_data_0.failreason,"发送悬停指令超时");
//            scheme_generation_state(2,3,2);// 发布失败
//            memset(CCC_DPU_data_0.failreason,0,200);
//            xuanting_flag = 0;
//            timeout = 0;
//            single_uav_flag = 0;
//        }
//    }
//}
void single_mission_target_plan()
{
	static int ctas004_recv_total = 0;
	static int ctas004_idle_cnt = 0;

	/* 单任务区/目标规划信息 */
	message_size = RECV_MAX_SIZE;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_12.niConnectionId,DDSTables.DPU2_CCC_12.niConnectionId,&DPU_CCC_data_12,sizeof DPU_CCC_data_12);

	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//		Receive_Message(DDSTables.PAD_CCC_026.niConnectionId, 0, &transaction_id, &DPU_CCC_data_12, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode == 0)
	{
		// reset state for new planning request, and flush related CTAS feedback topics
		single_mission_target_flag = 0;
		ctas_calc = 0;
		ctas004_recv_total = 0;
		ctas004_idle_cnt = 0;
		memset(&CCC_DPU_data_0.failreason,0,sizeof CCC_DPU_data_0.failreason);
		dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_011.niConnectionId);
		dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_007.niConnectionId);
		dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_002.niConnectionId);
		dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_004.niConnectionId);
		dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_008.niConnectionId);
		dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_009.niConnectionId);

		alloc_single_request_plan_id();
		if(DPU_CCC_data_12.mission_object_type == 2 || DPU_CCC_data_12.mission_object_type == 3)//目标/光标
		{
			printf("point1 lat:%lf\tpoint1 lon:%lf\n",DPU_CCC_data_12.waypoints_cursorSelection_longitude_and_latitude_synt[0].latitude,DPU_CCC_data_12.waypoints_cursorSelection_longitude_and_latitude_synt[0].longitude);
			printf("point2 lat:%lf\tpoint2 lon:%lf\n",DPU_CCC_data_12.waypoints_cursorSelection_longitude_and_latitude_synt[1].latitude,DPU_CCC_data_12.waypoints_cursorSelection_longitude_and_latitude_synt[1].longitude);
			if(DPU_CCC_data_12.mission_type == 15)
			{
				scheme_generation_state(2,1,0,1);// 返回方案编辑状态到综显，航线发布中
				//如果未加载返航航线，则生成失败
				int normal_cnt = 0;
				for(int i = 0 ; i < 4 ; i ++)
				{
					//无人机有返航航线且在线
					if(load_file.blk_dlr_ccc_045[i].normal_num > 0 && CCC_DPU_data_3.drone_specific_informations[i].platform_num == load_file.blk_dlr_ccc_045[i].uav_id)
					{
						normal_cnt++;
					}
				}
				if(CCC_DPU_data_3.drone_number > normal_cnt)
				{
					sprintf(CCC_DPU_data_0.failreason,"未检测到有效无人机返航航线");
					scheme_generation_state(2,2,0,3);// 生成失败
					memset(&CCC_DPU_data_0.failreason,0,sizeof CCC_DPU_data_0.failreason);
					return;
				}
			}
			else
			{
				scheme_generation_state(2,1,0,1);// 返回方案编辑状态到综显，航线发布中
				//如果未加载返航航线，则生成失败
				int normal_cnt = 0;
				for(int i = 0 ; i < 4 ; i ++)
				{
					//无人机有返航航线且在线
					if(load_file.blk_dlr_ccc_045[i].normal_num > 0 && CCC_DPU_data_3.drone_specific_informations[i].platform_num == load_file.blk_dlr_ccc_045[i].uav_id)
					{
						normal_cnt++;
					}
				}
				if(CCC_DPU_data_3.drone_number > normal_cnt)
				{
					sprintf(CCC_DPU_data_0.failreason,"未检测到有效无人机返航航线");
					scheme_generation_state(2,2,0,3);// 生成失败
					memset(&CCC_DPU_data_0.failreason,0,sizeof CCC_DPU_data_0.failreason);
					return;
				}
			}

			mission_insert_cover = DPU_CCC_data_12.mission_way;
			init_single_mission_target_zhanfa();
			send_single_mission_zhanshutuijian();  // 触发规划后只发送一次战法规划指令给 CTAS
			single_mission_target_flag = 2;

		}
		else if(DPU_CCC_data_12.mission_object_type == 1)//任务区
		{
			scheme_generation_state(2,1,0,1);// 返回方案编辑状态到综显，航线发布中
			//如果未加载返航航线，则生成失败
			int normal_cnt = 0;
			for(int i = 0 ; i < 4 ; i ++)
			{
				//无人机有返航航线且在线
				if(load_file.blk_dlr_ccc_045[i].normal_num > 0 && CCC_DPU_data_3.drone_specific_informations[i].platform_num == load_file.blk_dlr_ccc_045[i].uav_id)
				{
					normal_cnt++;
				}
			}
			if(CCC_DPU_data_3.drone_number > normal_cnt)
			{
				sprintf(CCC_DPU_data_0.failreason,"未检测到有效无人机返航航线");
				scheme_generation_state(2,2,0,3);// 生成失败
				memset(&CCC_DPU_data_0.failreason,0,sizeof CCC_DPU_data_0.failreason);
				return;
			}
			mission_insert_cover = DPU_CCC_data_12.mission_way;
			init_single_mission_target_zhanfa();
			send_single_mission_zhanshutuijian();  // 触发规划后只发送一次战法规划指令给 CTAS
			single_mission_target_flag = 2;

		}
	}
	//接收分配结果发送
	if(single_mission_target_flag == 2)
	{
		message_size = RECV_MAX_SIZE;
		int area_rtn;
		Receive_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, &message_size, &enRetCode);
		if(enRetCode == 0)
		{
			if(area_rtn < 0)
			{
				if(area_rtn == -1)
				{
					sprintf(CCC_DPU_data_0.failreason,"任务目标点超过许可空域范围");
				}
				else if(area_rtn == -2)
				{
					sprintf(CCC_DPU_data_0.failreason,"任务目标点距离许可空域边界过近");
				}
				else if(area_rtn == -3)
				{
					sprintf(CCC_DPU_data_0.failreason,"任务范围区域过小，转弯受限");
				}
				else if(area_rtn == -4)
				{
					sprintf(CCC_DPU_data_0.failreason,"任务区域范围过大，超出侦听距离");
				}
				else if(area_rtn == -5)
				{
					//单机编队
					sprintf(CCC_DPU_data_0.failreason,"集结距离小于10km");
				}
				else if(area_rtn == -6)
				{
					//双机编队260129
					sprintf(CCC_DPU_data_0.failreason,"集结距离小于50km");
				}
				else if(area_rtn == -7)
				{
					//双机编队260129
					sprintf(CCC_DPU_data_0.failreason,"无人机起始位置与集结点间距离小于2km");
				}
				else if(area_rtn == -8)
				{
					//双机编队260131
					sprintf(CCC_DPU_data_0.failreason,"无人机起始位置与集结点间距离大于15km");
				}
				scheme_generation_state(2,2,0,3);// 返回方案编辑状态到综显，生成失败 20260130
				// keep failreason until next request reset to avoid stale DDS timing
				single_uav_flag = 0;
				ctas004_recv_total = 0;
				ctas004_idle_cnt = 0;
				single_mission_target_flag = 0;
				dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_011.niConnectionId);
				dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_007.niConnectionId);
				dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_002.niConnectionId);
				dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_004.niConnectionId);
				dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_008.niConnectionId);
				dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_009.niConnectionId);
				return;
			}
		}
		//接收到单目标、攻击方案解算
		if(ctas_calc == 9527)
		{
			ctas_calc = 0;
			// CATS_DTMS 任务区划分信息
			if(DPU_CCC_data_12.mission_object_type == 1)
			{
				BLK_CCC_OFP_005 temp;
				message_size = RECV_MAX_SIZE;
				Receive_Message(DDSTables.BLK_CTAS_DTMS_007.niConnectionId, 0, &transaction_id, &temp, &message_type_id, &message_size, &enRetCode);
				if(enRetCode == 0)
				{
					//保存区域划分信息
					int plan = temp.Plan_ID % 3;
					memcpy(&blk_ccc_ofp_005[plan], &temp ,sizeof(BLK_CCC_OFP_005));
					data_length = sizeof(BLK_CCC_OFP_005);
					// 转发给综显任务区划分信息
					Send_Message(DDSTables.CCC_DPU_30.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);

					//发送给pad new20250620
					if(Pad_heart_flag == 1)
					{
						Send_Message(DDSTables.CCC_PAD_005.niConnectionId,0,&transaction_id, &temp, &message_type_id, data_length, &enRetCode);
					}

				}
			}

			for(int i = 0 ; i < 3 ; i ++)
			{
				//收到单任务区/目标任务分配结果CTAS->DTMS
				message_size = RECV_MAX_SIZE;
				Receive_Message(DDSTables.BLK_CTAS_DTMS_002.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
				if(enRetCode == 0)
				{
					memcpy(&blk_ccc_ofp_019,dds_data_rece.dataA,sizeof blk_ccc_ofp_019);
					unsigned int plan = blk_ccc_ofp_019.plan_id % 3;
					//如果收到的分配结果id与运行方案id一致就进行插入或覆盖
					if(blk_ccc_ofp_019.plan_id == get_single_request_plan_id())
					{
//						if(mission_insert_cover == 2)
						{
							//覆盖方案方案
							memcpy(&CCC_DPU_data_6[plan],&blk_ccc_ofp_019,sizeof(BLK_CCC_OFP_019));
							// 在非紧密无人机编队下，需要特殊处理发给ofp的信息
							send_blk_ccc_ofp_019_special();
						}
					}
					//没有运行方案就透传
					else
					{
						// 在非紧密无人机编队下，需要特殊处理发给ofp的信息
						send_blk_ccc_ofp_019_special();
					}
				}
			}
			single_mission_target_flag = 3;
			ctas004_recv_total = 0;
			ctas004_idle_cnt = 0;
		}
	}
	//接收无人机航线保存、转发
	else if(single_mission_target_flag == 3)
		{
			int recv_cnt_this_cycle = 0;
			//接收三个方案的航线
			for(int j = 0 ; j < 3 ; j ++)
			{
				//接收攻击安全区
				recv_blk_ctas_dtms_047();		   // 攻击安全区
				for(int i = 0 ; i < 2 ; i ++)
				{
					BLK_CCC_OFP_024 temp;
					message_size = RECV_MAX_SIZE;
					Receive_Message(DDSTables.BLK_CTAS_DTMS_004.niConnectionId, 0, &transaction_id, &temp, &message_type_id, &message_size, &enRetCode);
				if (enRetCode == 0)
				{
						castCtasToOfpLine(&temp);
						unsigned int plan = (temp.program_number) % 3;
						unsigned int id = (temp.individual_drone_routing_programs.drone_serial_number - 1);
						unsigned int index = temp.individual_drone_routing_programs.planning_informations.packet_id *25;
						// 保存信息
						memcpy(&blk_ccc_ofp_024_cunchu[plan][id],&temp,9 + 10 + 26);
						// 保存航路点信息
					memcpy(&blk_ccc_ofp_024_cunchu[plan][id].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index], &temp.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], sizeof(planning_information_waypoint_information) * 25);
						//发送航线
						send_blk_ccc_ofp_024(plan); // 发送无人机信息，航线生成完成
						recv_cnt_this_cycle++;
					}
				}
			}

			if(recv_cnt_this_cycle > 0)
			{
				ctas004_recv_total += recv_cnt_this_cycle;
				ctas004_idle_cnt = 0;
			}
			else
			{
				ctas004_idle_cnt++;
			}

			// 有接收后等待若干空闲周期再结束，确保预览航线能稳定显示
			if((ctas004_recv_total > 0 && ctas004_idle_cnt > 2) || ctas004_idle_cnt > 80)
			{
				printf("CTAS004 done recv=%d idle=%d\n",ctas004_recv_total,ctas004_idle_cnt);
				single_mission_target_flag = 4;
				ctas004_recv_total = 0;
				ctas004_idle_cnt = 0;
			}
		}
		else if(single_mission_target_flag == 4)
	{
		memset(&CCC_DPU_data_0.failreason,0,sizeof CCC_DPU_data_0.failreason);
		if(DPU_CCC_data_12.mission_type == 15)
		{
			scheme_generation_state(2,2,0,2);// 返回方案编辑状态到综显，航线发布中
		}
		else
		{
			scheme_generation_state(2,2,0,2);// 返回方案编辑状态到综显，航线发布中
		}
		ctas004_recv_total = 0;
		ctas004_idle_cnt = 0;
		single_mission_target_flag = 0;
	}
}

// 初始化 单个无人机规划指令发给CATS
void init_single_mission_target_zhanfa()
{

	memset(&blk_dtms_ctas_002,0,sizeof(BLK_DTMS_CTAS_002));
	blk_dtms_ctas_002.planning_id = get_single_request_plan_id();//单任务请求方案id
	blk_dtms_ctas_002.uav_id = 0;//单任务区/目标
	blk_dtms_ctas_002.lead_uav_id = load_file.lead_uav_id;//长机id
	blk_dtms_ctas_002.solider_num = 1 + CCC_DPU_data_3.drone_number;   //todo：按有人机和无人机总数量赋值
	blk_dtms_ctas_002.solider_infos[0].solider_type = 1; // 类别  1 有人 2 无人
	blk_dtms_ctas_002.solider_infos[0].solider_id = MANNED_ID;//todo:目前有人机id写固定值：9001
	blk_dtms_ctas_002.solider_infos[0].lon_lat_info.latitude =DPU_CCC_data_4.manned_longitude_and_latitude_synt.latitude;
	blk_dtms_ctas_002.solider_infos[0].lon_lat_info.longitude = DPU_CCC_data_4.manned_longitude_and_latitude_synt.longitude;
	blk_dtms_ctas_002.solider_infos[0].speed = DPU_CCC_data_4.groundspeed;
	blk_dtms_ctas_002.solider_infos[0].height = DPU_CCC_data_4.absolute_barometric_altitude;
	blk_dtms_ctas_002.solider_infos[0].hangxiang = DPU_CCC_data_4.true_direction;

	for(int i = 0 ; i < 4 ; i ++ )
	{
		if(CCC_DPU_data_3.drone_specific_informations[i].platform_num == 0)
		{
			continue;
		}
		blk_dtms_ctas_002.solider_infos[i + 1].solider_type = 2;
		blk_dtms_ctas_002.solider_infos[i + 1].solider_id = CCC_DPU_data_3.drone_specific_informations[i].platform_num;
		blk_dtms_ctas_002.solider_infos[i + 1].speed = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_speed;
		blk_dtms_ctas_002.solider_infos[i + 1].height = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_height;
		blk_dtms_ctas_002.solider_infos[i + 1].hangxiang = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_heading;
		blk_dtms_ctas_002.solider_infos[i + 1].lon_lat_info.latitude = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_lati;
		blk_dtms_ctas_002.solider_infos[i + 1].lon_lat_info.longitude = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_longi;
	}


	blk_dtms_ctas_002.task_type = DPU_CCC_data_12.mission_type; // 任务类型
	blk_dtms_ctas_002.target_type = DPU_CCC_data_12.mission_object_type; // 目标类型
	blk_dtms_ctas_002.zhanshu_sel = 1;   // 战术战法选择(废弃不使用)
	blk_dtms_ctas_002.task_reg_num = 1;  // 任务区数量
	if (DPU_CCC_data_12.mission_object_type == 1)
	{ //(任务区信息)
		//索引任务区
		int index = -1;
		int packge = -1;
		for(int i = 0 ;i < 8 ; i++)
		{
			if(blk_ccc_ofp_033[0].area_informations[i].area_code == DPU_CCC_data_12.area_point_step_num)
			{
				index = i;
				packge = 0;
			}
			else if(blk_ccc_ofp_033[1].area_informations[i].area_code == DPU_CCC_data_12.area_point_step_num)
			{
				index = i;
				packge = 1;
			}
		}
		if(index < 0)
		{
			printf("no area can be find\n");
			return;
		}
		blk_dtms_ctas_002.region_infos[0].reg_sour = blk_ccc_ofp_033[packge].area_informations[index].area_source;
		blk_dtms_ctas_002.region_infos[0].reg_type = blk_ccc_ofp_033[packge].area_informations[index].area_type;
		blk_dtms_ctas_002.region_infos[0].reg_shape = blk_ccc_ofp_033[packge].area_informations[index].area_shape;
		switch (blk_dtms_ctas_002.region_infos[0].reg_shape)
		{
		case 1:
		{ // 圆形
			blk_dtms_ctas_002.region_infos[0].reg_circle.radious = blk_ccc_ofp_033[packge].area_informations[index].cycles.radius;
			blk_dtms_ctas_002.region_infos[0].reg_circle.center_lon_lat.latitude = blk_ccc_ofp_033[packge].area_informations[index].cycles.cycle_longitude_and_latitude.latitude;
			blk_dtms_ctas_002.region_infos[0].reg_circle.center_lon_lat.longitude = blk_ccc_ofp_033[packge].area_informations[index].cycles.cycle_longitude_and_latitude.longitude;
		}
		break;
		case 2:
		{ // 多边形
			blk_dtms_ctas_002.region_infos[0].reg_ploygen.point_num = blk_ccc_ofp_033[packge].area_informations[index].polygonals.point_number;
			for (int j = 0; j < blk_dtms_ctas_002.region_infos[0].reg_ploygen.point_num; j++)
			{
				blk_dtms_ctas_002.region_infos[0].reg_ploygen.points_lon_lat[j].latitude = blk_ccc_ofp_033[packge].area_informations[index].polygonals.point_coordinates[j].latitude;
				blk_dtms_ctas_002.region_infos[0].reg_ploygen.points_lon_lat[j].longitude = blk_ccc_ofp_033[packge].area_informations[index].polygonals.point_coordinates[j].longitude;
			}
		}
		break;

		}

		blk_dtms_ctas_002.region_infos[0].reg_top_of_hei_valid = blk_ccc_ofp_033[packge].area_informations[index].upper_height_limit_valid_bit;
		blk_dtms_ctas_002.region_infos[0].reg_down_of_hei_valid = blk_ccc_ofp_033[packge].area_informations[index].lower_height_limit_valid_bit;
		blk_dtms_ctas_002.region_infos[0].kongyu_belong_to_uav_valid = 1;
		blk_dtms_ctas_002.region_infos[0].top_of_hei = blk_ccc_ofp_033[packge].area_informations[index].upper_height_limit;
		blk_dtms_ctas_002.region_infos[0].down_of_hei = blk_ccc_ofp_033[packge].area_informations[index].lower_height_limit;
	}
	else if (DPU_CCC_data_12.mission_object_type == 2)
	{ //(目标信息)光标选点
		memset(&(blk_dtms_ctas_002.region_infos),0,sizeof(blk_dtms_ctas_002.region_infos));
		// 任务点信息
		blk_dtms_ctas_002.region_infos[0].task_point_num = 1;
		if ((blk_dtms_ctas_002.region_infos[0].task_point_num) > 0)
		{
			for(int k = 0;k < 30 ;k++)
			{
				//找到批号对应目标,0也是有效的
				if(DPU_CCC_data_0.target_informations[k].target_lot_number1 == DPU_CCC_data_12.area_point_step_num)
				{
					blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude = DPU_CCC_data_0.target_informations[k].target_longitude_and_latitude_synt.latitude;
					blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude = DPU_CCC_data_0.target_informations[k].target_longitude_and_latitude_synt.longitude;
					//航速有效
					if(DPU_CCC_data_0.target_informations[k].valid_flag.speed_valid == 1)
					{
						blk_dtms_ctas_002.region_infos[0].point_infos[0].point_speed_valid = 1;
						blk_dtms_ctas_002.region_infos[0].point_infos[0].target_speed = DPU_CCC_data_0.target_informations[k].speed;
					}
					//航向有效
					if(DPU_CCC_data_0.target_informations[k].valid_flag.heading_valid == 1)
					{
						blk_dtms_ctas_002.region_infos[0].point_infos[0].point_hangxiang_valid = 1;
						blk_dtms_ctas_002.region_infos[0].point_infos[0].target_hangxiang = DPU_CCC_data_0.target_informations[k].direction;
					}
					break;
				}
			}
		}
	}
	else if (DPU_CCC_data_12.mission_object_type == 3 || DPU_CCC_data_12.mission_object_type == 4)
	{ //(光标选点/航路点)
		memset(&(blk_dtms_ctas_002.region_infos),0,sizeof(blk_dtms_ctas_002.region_infos));
		// 任务点信息
		blk_dtms_ctas_002.region_infos[0].task_point_num = 1;
		//编队飞行
		if(DPU_CCC_data_12.mission_type == 9)
			blk_dtms_ctas_002.region_infos[0].task_point_num = 2;
		for(int k = 0;k < blk_dtms_ctas_002.region_infos[0].task_point_num ;k++)
		{
			blk_dtms_ctas_002.region_infos[0].point_infos[0].point_serlize_id = k;
			blk_dtms_ctas_002.region_infos[0].point_infos[0].point_id = k+1;
			blk_dtms_ctas_002.region_infos[0].point_infos[0].point_source = 0;
			blk_dtms_ctas_002.region_infos[0].point_infos[k].point_lon_lat.latitude = DPU_CCC_data_12.waypoints_cursorSelection_longitude_and_latitude_synt[k].latitude;
			blk_dtms_ctas_002.region_infos[0].point_infos[k].point_lon_lat.longitude = DPU_CCC_data_12.waypoints_cursorSelection_longitude_and_latitude_synt[k].longitude;
		}

	}

}

// 初始化 单任务区/目标规划指令发给CATS
void init_single_uav_zhanfa()
{

	memset(&blk_dtms_ctas_002,0,sizeof(BLK_DTMS_CTAS_002));
	if(g_single_uav_request_plan_id != 0)
	{
		blk_dtms_ctas_002.planning_id = g_single_uav_request_plan_id;//单任务请求方案id
	}
	else
	{
	blk_dtms_ctas_002.planning_id = get_single_request_plan_id();//单任务请求方案id
	}
	blk_dtms_ctas_002.uav_id = DPU_CCC_data_11.drone_id;//单无人机编号
	blk_dtms_ctas_002.solider_num = 1 + CCC_DPU_data_3.drone_number;   //todo：按有人机和无人机总数量赋值
	blk_dtms_ctas_002.solider_infos[0].solider_type = 1; // 类别  1 有人 2 无人
	blk_dtms_ctas_002.solider_infos[0].solider_id = MANNED_ID;//todo:目前有人机id写固定值：9001
	blk_dtms_ctas_002.solider_infos[0].lon_lat_info.latitude =DPU_CCC_data_4.manned_longitude_and_latitude_synt.latitude;
	blk_dtms_ctas_002.solider_infos[0].lon_lat_info.longitude = DPU_CCC_data_4.manned_longitude_and_latitude_synt.longitude;
	blk_dtms_ctas_002.solider_infos[0].speed = DPU_CCC_data_4.groundspeed;
	blk_dtms_ctas_002.solider_infos[0].height = DPU_CCC_data_4.absolute_barometric_altitude;
	blk_dtms_ctas_002.solider_infos[0].hangxiang = DPU_CCC_data_4.true_direction;

	// 可能非紧密排列，则计数有效无人机
	int tem_uav_num = 0;
	for(int i = 0 ; i < 4 ; i ++ )
	{
		if(CCC_DPU_data_3.drone_specific_informations[i].platform_num == 0)
		{
			continue;
		}
		blk_dtms_ctas_002.solider_infos[tem_uav_num + 1].solider_type = 2;
		blk_dtms_ctas_002.solider_infos[tem_uav_num + 1].solider_id = CCC_DPU_data_3.drone_specific_informations[i].platform_num;
		blk_dtms_ctas_002.solider_infos[tem_uav_num + 1].speed = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_speed;
		blk_dtms_ctas_002.solider_infos[tem_uav_num + 1].height = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.air_height;
		blk_dtms_ctas_002.solider_infos[tem_uav_num + 1].hangxiang = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_heading;
		blk_dtms_ctas_002.solider_infos[tem_uav_num + 1].lon_lat_info.latitude = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_lati;
		blk_dtms_ctas_002.solider_infos[tem_uav_num + 1].lon_lat_info.longitude = CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_longi;

		tem_uav_num++;
	}


	blk_dtms_ctas_002.task_type = DPU_CCC_data_11.mission_type; // 任务类型
	blk_dtms_ctas_002.target_type = DPU_CCC_data_11.mission_object_type; // 目标类型
	blk_dtms_ctas_002.zhanshu_sel = 1;   // 战术战法选择(废弃不使用)
	if (DPU_CCC_data_11.mission_object_type == 1)
	{										//(任务区信息)
		blk_dtms_ctas_002.task_reg_num = 1;  // 任务区数量
		blk_dtms_ctas_002.region_infos[0].reg_id = DPU_CCC_data_11.area_point_step_num;
		blk_dtms_ctas_002.region_infos[0].reg_serlize_id = DPU_CCC_data_11.area_point_step_num;
		//索引任务区
		int index = -1;
		int packge = -1;
		for(int i = 0 ;i < 8 ; i++)
		{
			if(blk_ccc_ofp_033[0].area_informations[i].area_code == DPU_CCC_data_11.area_point_step_num)
			{
				index = i;
				packge = 0;
			}
			else if(blk_ccc_ofp_033[1].area_informations[i].area_code == DPU_CCC_data_11.area_point_step_num)
			{
				index = i;
				packge = 1;
			}
		}
		if(index < 0)
		{
			printf("no area can be find\n");
			return;
		}
		blk_dtms_ctas_002.region_infos[0].reg_sour = blk_ccc_ofp_033[packge].area_informations[index].area_source;
		blk_dtms_ctas_002.region_infos[0].reg_type = blk_ccc_ofp_033[packge].area_informations[index].area_type;
		blk_dtms_ctas_002.region_infos[0].reg_shape = blk_ccc_ofp_033[packge].area_informations[index].area_shape;
		switch (blk_dtms_ctas_002.region_infos[0].reg_shape)
		{
		case 1:
		{ // 圆形
			blk_dtms_ctas_002.region_infos[0].reg_circle.radious = blk_ccc_ofp_033[packge].area_informations[index].cycles.radius;
			blk_dtms_ctas_002.region_infos[0].reg_circle.center_lon_lat.latitude = blk_ccc_ofp_033[packge].area_informations[index].cycles.cycle_longitude_and_latitude.latitude;
			blk_dtms_ctas_002.region_infos[0].reg_circle.center_lon_lat.longitude = blk_ccc_ofp_033[packge].area_informations[index].cycles.cycle_longitude_and_latitude.longitude;
		}
		break;
		case 2:
		{ // 多边形
			blk_dtms_ctas_002.region_infos[0].reg_ploygen.point_num = blk_ccc_ofp_033[packge].area_informations[index].polygonals.point_number;
			for (int j = 0; j < blk_dtms_ctas_002.region_infos[0].reg_ploygen.point_num; j++)
			{
				blk_dtms_ctas_002.region_infos[0].reg_ploygen.points_lon_lat[j].latitude = blk_ccc_ofp_033[packge].area_informations[index].polygonals.point_coordinates[j].latitude;
				blk_dtms_ctas_002.region_infos[0].reg_ploygen.points_lon_lat[j].longitude = blk_ccc_ofp_033[packge].area_informations[index].polygonals.point_coordinates[j].longitude;
			}
		}
		break;

		}

		blk_dtms_ctas_002.region_infos[0].reg_top_of_hei_valid = blk_ccc_ofp_033[packge].area_informations[index].upper_height_limit_valid_bit;
		blk_dtms_ctas_002.region_infos[0].reg_down_of_hei_valid = blk_ccc_ofp_033[packge].area_informations[index].lower_height_limit_valid_bit;
		blk_dtms_ctas_002.region_infos[0].kongyu_belong_to_uav_valid = 1;
		blk_dtms_ctas_002.region_infos[0].top_of_hei = blk_ccc_ofp_033[packge].area_informations[index].upper_height_limit;
		blk_dtms_ctas_002.region_infos[0].down_of_hei = blk_ccc_ofp_033[packge].area_informations[index].lower_height_limit;

	}
	else if (DPU_CCC_data_11.mission_object_type == 2)
	{ //(目标)
		memset(&(blk_dtms_ctas_002.region_infos),0,sizeof(blk_dtms_ctas_002.region_infos));
		// 任务点信息
		blk_dtms_ctas_002.region_infos[0].task_point_num = 1;

		for (int k = 0; k < 30; k++)
		{
			//找到批号对应目标,0也是有效的
			if(DPU_CCC_data_0.target_informations[k].target_lot_number1 == DPU_CCC_data_11.area_point_step_num)
			{
				blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude = DPU_CCC_data_0.target_informations[k].target_longitude_and_latitude_synt.latitude;
				blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude = DPU_CCC_data_0.target_informations[k].target_longitude_and_latitude_synt.longitude;
				//航速有效
				if(DPU_CCC_data_0.target_informations[k].valid_flag.speed_valid == 1)
				{
					blk_dtms_ctas_002.region_infos[0].point_infos[0].point_speed_valid = 1;
					blk_dtms_ctas_002.region_infos[0].point_infos[0].target_speed = DPU_CCC_data_0.target_informations[k].speed;
				}
				//航向有效
				if(DPU_CCC_data_0.target_informations[k].valid_flag.heading_valid == 1)
				{
					blk_dtms_ctas_002.region_infos[0].point_infos[0].point_hangxiang_valid = 1;
					blk_dtms_ctas_002.region_infos[0].point_infos[0].target_hangxiang = DPU_CCC_data_0.target_informations[k].direction;
				}
				break;
			}
		}
	}
	else if (DPU_CCC_data_11.mission_object_type == 3 || DPU_CCC_data_11.mission_object_type == 4)
	{ //(光标选点/航路点)
		memset(&(blk_dtms_ctas_002.region_infos),0,sizeof(blk_dtms_ctas_002.region_infos));
		// 任务点信息
		blk_dtms_ctas_002.region_infos[0].task_point_num = 1;
		blk_dtms_ctas_002.region_infos[0].point_infos[0].point_serlize_id = 1;
		blk_dtms_ctas_002.region_infos[0].point_infos[0].point_id = 1;
		blk_dtms_ctas_002.region_infos[0].point_infos[0].point_source = 0;
		blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude = DPU_CCC_data_11.waypoints_cursorSelection_longitude_and_latitude_synt.latitude;
		blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude = DPU_CCC_data_11.waypoints_cursorSelection_longitude_and_latitude_synt.longitude;


	}

}
// 无人机下一阶段提示
void send_blk_ccc_ofp_006()
{
	for(int i = 0 ; i < 4 ; i ++)
	{
		// 考虑冲突时的规避航线就是单独盘旋点，执行盘旋点航线就会达到本提示要求，则冲突时不提示
		if(g_lineCrashState[i].hasConflict)
		{
			continue;
		}

		// 当前任务是悬停不提示
		int type = getUavCurrentTask(i);
		// /*子任务任务类型  0=N/A;1=浮标侦收;2=吊声定测;3=浮标布阵;4=通信中继;5=磁探搜索;6=磁探跟踪;7=光电搜索;8=光电跟踪;9=编队飞行;10=任务导航;11=返航;12=悬停;13=临时改航;14=盘旋;15攻击*/
		if(type == 12 || type == 14)
		{
			continue;
		}


		//判断注入的无人机航路点是否已经飞完，倒数第二个计算点和最后一个为机场点不飞(判断最后一个点，且待飞距小于设定值)，待飞距离小于300，处于全局任务规划
		if(uav_route[i].tasking == 1 &&(uav_route[i].hull_number - 2 == uav_route[i].waypoint_number) /*&& g_toCurPointDis[i] < 300 */&& formulate_single == 1)
		{
			//计算待飞距离
			double distances = 0;
			double point_lat,point_lon;
			int point_index = 0;
			point_index = uav_route[i].hull_number - 3 -1;
			point_lat = uav_route[i].waypoint[point_index].latitude;
			point_lon = uav_route[i].waypoint[point_index].longitude;
			distances = calculate_distances(point_lat,point_lon,formationId[i].lat,formationId[i].lon);
			if(distances < 1)
			{
				//任务已完成
				uav_route[i].tasking = 0;
				task_over_cnt++;
				//飞完的无人机数量等于注入的无人机数量
				if(task_over_cnt == uav_success)
				{
					task_over_cnt = 0;
					//计算是否在最后一个阶段
					unsigned int plan = blk_ofp_ccc_039.Plan_ID % 3;
					// 找到任务数量
					int task_num = 0;
					task_num = CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[i+1].platform_subtask_number;
					if(task_num == global_stage -1)
					{
						blk_ccc_ofp_006.Null0 = 1;
					}
					blk_ccc_ofp_006.plan_id = planning_id;
					//发送下一阶段提示
					data_length = sizeof(BLK_CCC_OFP_006);
					Send_Message(DDSTables.CCC_DPU_31.niConnectionId,0,&transaction_id, &blk_ccc_ofp_006 , &message_type_id, data_length, &enRetCode);
					//发送给pad new20250620
					//                Send_Message(DDSTables.CCC_PAD_006.niConnectionId,0,&transaction_id, &blk_ccc_ofp_006,&message_type_id, data_length, &enRetCode);
					memset(&blk_ccc_ofp_006,0,sizeof(BLK_CCC_OFP_006));
				}
			}
		}
	}
}
// 无人机下一阶段提示反馈
void recv_blk_ofp_ccc_041()
{
	message_size = RECV_MAX_SIZE;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_30.niConnectionId,DDSTables.DPU2_CCC_30.niConnectionId,&blk_ofp_ccc_041,sizeof blk_ofp_ccc_041);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//		Receive_Message(DDSTables.PAD_CCC_041.niConnectionId, 0, &transaction_id, &blk_ofp_ccc_041, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode==0)
	{
		printf("blk_ofp_ccc_041.ignal_FC00_DPU1 = %d ",blk_ofp_ccc_041.ignal_FC00_DPU1);
	}
}

// 发布处理
void faBuProc()
{
	unsigned int fabu_plan_id = (g_fabu_active == 1) ? g_fabu_plan_id : blk_ofp_ccc_039.Plan_ID;
	unsigned int fabu_route_type = (g_fabu_active == 1) ? g_fabu_route_type : blk_ofp_ccc_039.routeType;
	unsigned int fabu_stage_id = (g_fabu_active == 1) ? g_fabu_stage_id : blk_ofp_ccc_039.stage_id;
	// 冲突时发布前处理
//	avoidLineCrashFabuProc();
	if ((g_recv_fabuCode[0] == 1) || (g_recv_fabuCode[1] == 1) || (g_recv_fabuCode[2] == 1) || (g_recv_fabuCode[3] == 1))
	{
		// 清空发布标记
		g_recv_fabuCode[0] = 0;
		g_recv_fabuCode[1] = 0;
		g_recv_fabuCode[2] = 0;
		g_recv_fabuCode[3] = 0;

			g_fabu_active = 1;
			g_fabu_plan_id = blk_ofp_ccc_039.Plan_ID;
			g_fabu_route_type = blk_ofp_ccc_039.routeType;
			g_fabu_stage_id = blk_ofp_ccc_039.stage_id;
			fabu_plan_id = g_fabu_plan_id;
			fabu_route_type = g_fabu_route_type;
			fabu_stage_id = g_fabu_stage_id;
			printf("FABU start plan=%u route=%u stage=%u\n",fabu_plan_id,fabu_route_type,fabu_stage_id);

			//发布航线时停止查询
		hx_cx_flag = 0;
		//是否为攻击
		if(fabu_route_type == 2)
		{
			scheme_generation_state(2,2, 1, 2); // 返回发布状态到综显

			//重新生成攻击航线空域
			for(int i = 0 ; i < 2 ; i ++)
				send_uav_airway(fabu_plan_id % 3,i);
		}
		else if(fabu_route_type == 1)
		{
			scheme_generation_state(1,2, 1, 2); // 返回发布状态到综显
		}

		//找到方案编号索引
		unsigned int plan = fabu_plan_id % 3;

		//初始化无人机数据,接收标志,空域应用标志清空
		for (int i = 0; i < 4; i++)
		{
			memset(&uav_send[i], 0, sizeof(UAV_SEND));
			memset(&s4D_frame_40[i], 0, sizeof(int));
			memset(&s4D_frame_38[i], 0, sizeof(int));
			send_area[i] = 0;
		}

		for (unsigned int i = 0; i < 4; i++)
		{

			//进入第二次发布，没有冲突的无人机不再重新注入,只注入存在冲突的无人机恢复航线
			if(blk_ofp_ccc_154.plan_query == 3 && g_lineCrashState[i].hasConflict == 0)
			{
				continue;
			}
			uav_send[i].drone_id =
					CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[i + 1].platform_code;

			//找到任务类型参数索引
			for (int j = 0; j < 8; j++)
			{
				if (task_param[j].type_id == CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[i + 1].task_sequence_informations[fabu_stage_id].type)
				{
					//找到索引退出
					uav_send[i].task_index = j;
					break;
				}
			}

			//航路点数量
			uav_send[i].waypoints_number =
					blk_ccc_ofp_024_cunchu[plan][i].individual_drone_routing_programs.planning_informations.waypoints_number;
			//存入当前任务的每个航路点经纬度
			memcpy(&uav_send[i].waypoint_informations[0],
					&blk_ccc_ofp_024_cunchu[plan][i].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
				   sizeof(planning_information_waypoint_information) * uav_send[i].waypoints_number);

			//发送准备
			uav_send[i].send_ready = 1;
			uav_send[i].first_flag = 0;
			//判断该无人机是否有航路点，没有航路点则放弃发送
			if (uav_send[i].waypoints_number == 0)
			{
				//清空该无人机数据
				memset(&uav_send[i], 0, sizeof(UAV_SEND));
			}
		}
		//初始化发送参数，防止上次指令切换之后的帧变化影响
		frame_count = 1;
		track_point_count = -1;
		send_count = 0;
		g_fabu_send_uav_num = 0;
		g_fabu_switch_timeout = 0;
		g_fabu_switch_ack_cnt = 0;
	}

	//发送的有效无人机数量（全局复位变量）
	//判断无人机信息是否都录入完成并且队列中有还未发送的数据
	if (uav_hl_confirm == 0)
	{
		for (int i = 0; i < 4; i++)
		{
			if (uav_send[i].send_ready == 1)
			{
				//进入发送，保存索引（每次发一架无人机的相关信息，不交叉发）
				uav_hl_confirm = 1;
				send_index = i;
				g_fabu_send_uav_num++;
				//20250818new 加入首尾机场点和计算点
				uav_send[send_index].waypoints_number += 3;
				break;
			}
		}
	}
	//进入发送
	else if (uav_hl_confirm == 1)
	{
		//判断装订是否出错
		if (s4D_frame_38[send_index] != 0)
		{
			//装订出错后初始化
			uav_hl_confirm = 0;
			//初始化无人机数据
			for (int i = 0; i < 4; i++)
			{
				memset(&uav_send[i], 0, sizeof(UAV_SEND));
			}
			//发送发布失败状态到综显
			sprintf(CCC_DPU_data_0.failreason, "发送航线装订指令返回失败");
			//是否为攻击
			if(fabu_route_type == 2)
			{
				scheme_generation_state(2,2, 3, 2);
			}
			else if(fabu_route_type == 1)
			{
				scheme_generation_state(1,2, 3, 2);
			}
			memset(CCC_DPU_data_0.failreason, 0, 200);
				g_fabu_send_uav_num = 0;
				g_fabu_active = 0;
				printf("0x38 failed! %d\n", s4D_frame_38[send_index]);

			//退出函数
			return;
		}
		//航路修订指令字 0x38
		send_uav_hl(uav_send[send_index].drone_id, 0x38);    //发送无人机航线装订指令
		//判断是否发送完一架无人机的所有航路点
		if (frame_count > 4 * uav_send[send_index].waypoints_number + 1)
		{																  // 机场点+一般航迹点+倒数第二个计算点+最后一个汇合点，所以是+3，最后加1是确保最后一个点发送四包
			uav_hl_confirm = 0;    // 航点发送完结束
			frame_count = 1;    //帧计数置位
			track_point_count = -1;    //该航线航点发送完毕，航点计数归 -1
			hx_change_uav_id[send_index] = uav_send[send_index].drone_id; //保存切换id
			//清空该无人机数据
			memset(&uav_send[send_index], 0, sizeof(UAV_SEND));
			//数据区中没有数据后，认为已全部发送完成
			for (int i = 0; i < 4; i++)
			{
				//遍历整个存储区，没有数据后认为发送完成
				if (uav_send[i].send_ready == 1)
				{
					break;
				}
				else if (i == 3)
				{
					//装订完成
					uav_hl_confirm = 2;
					//航点全部发送完成，开始切换航线
					uav_change = 1;
				}
			}
		}
	}
	//等待切换成功
	else if (uav_hl_confirm == 2)
	{

		g_fabu_switch_timeout++;
		for (int i = 0; i < 4; i++)
		{

			//装订成功
			if (s4D_frame_40[i] == 1 && send_area[i] == 0)
			{
				//找到方案编号索引
				unsigned int plan = fabu_plan_id % 3;
				/*****存储发布航线 20250702new*****/
				//注入任务
				uav_route[i].tasking = 1;
				//任务类型
				uav_route[i].task_type =
						blk_ccc_ofp_024_cunchu[plan][i].individual_drone_routing_programs.planning_informations.mission_type;
				//任务id
				uav_route[i].task_id =
						blk_ccc_ofp_024_cunchu[plan][i].individual_drone_routing_programs.planning_informations.subtask_ID_number;
				//航路点数量，加上首尾机场点再加上计算点
				uav_route[i].hull_number =
					blk_ccc_ofp_024_cunchu[plan][i].individual_drone_routing_programs.planning_informations.waypoints_number + 3;
				//获取当前任务的航路点数量
				int points_number =
						blk_ccc_ofp_024_cunchu[plan][i].individual_drone_routing_programs.planning_informations.waypoints_number;
				//存入当前任务的每个航路点信息
				memcpy(&uav_route[i].waypoint[0],
						&blk_ccc_ofp_024_cunchu[plan][i].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
					   sizeof(planning_information_waypoint_information) * points_number);
				//无人机注入成功，可应用空域
				send_area[i] = 1;

				g_fabu_switch_ack_cnt++;
				s4D_frame_40[i] = 0;
				//切换成功数量与发送的有效无人机数量一致
				if (g_fabu_switch_ack_cnt == g_fabu_send_uav_num)
				{
					//保存注入成功的无人机数量
					uav_success = g_fabu_send_uav_num;
					//初始化变量
					g_fabu_switch_ack_cnt = 0;
					g_fabu_send_uav_num = 0;
					g_fabu_switch_timeout = 0;
					uav_hl_confirm = 0;
					//保存运行方案id
					planning_id = fabu_plan_id;
					g_fabu_active = 0;
					//是否为单无人机注入
					if (single_uav_flag == 33)
					{
						// 发送单无人机务运行分配结果
						blk_ccc_ofp_019.plan_release_mode = 1;            //已发布
						send_blk_ccc_ofp_021();
					}
					else
					{
						//普通/反潜发布成功后，固化运行方案模式，避免017模式异常为0
						CCC_DPU_data_6_Ofp[plan].plan_release_mode = 1;
						CCC_DPU_data_6[plan].plan_release_mode = 1;
						//取出保存的方案，发送运行方案到综显
						memcpy(&blk_ccc_ofp_017, &CCC_DPU_data_6_Ofp[plan],
							sizeof(BLK_CCC_OFP_017));

						send_blk_ccc_ofp_017();
						formulate_single = 1;
					}
					// 再次发送综显任务区划分信息（运行方案）
					data_length = sizeof(BLK_CCC_OFP_005);
					Send_Message(DDSTables.CCC_DPU_30.niConnectionId, 0,
							&transaction_id, &blk_ccc_ofp_005[plan],
							&message_type_id, data_length, &enRetCode);
					//发送给pad new20250620
					//					Send_Message(DDSTables.CCC_PAD_005.niConnectionId,0,&transaction_id, &blk_ccc_ofp_005[plan],&message_type_id, data_length, &enRetCode);

					//发送浮标和吊声解算
					for (int i = 0; i < 8; i++)
					{
						send_buoy_soanr_route_information(planning_id % 3, i); //发送浮标、吊声信息到综显
					}

					//发送当前阶段反馈 20250830new
					send_blk_ccc_ofp_025();

					//单无人机航线只发一架机
					if (single_uav_flag == 33)
					{
						unsigned int single_index = DPU_CCC_data_11.drone_num - 1;
						send_blk_ccc_ofp_024_single(plan,single_index);
						single_uav_flag = 0;
					}
					else
					{
						// 再次发送无人机航线（运行方案）
						send_blk_ccc_ofp_024(plan);
					}



					//是否为攻击
					if(fabu_route_type == 2)
					{
						memcpy(&blk_ccc_ofp_047,&blk_ccc_ofp_047_save[plan],sizeof(BLK_CCC_OFP_047));
						//转发到综显
						send_blk_ccc_ofp_047();

						scheme_generation_state(2,2, 2, 2);      //发送发布完成状态到综显
					}
					else if(fabu_route_type == 1)
					{
						scheme_generation_state(1,2, 2, 2);      //发送发布完成状态到综显
					}

					// 冲突时，第一次发布不增加阶段数，除了冲突时第一次发布，其他发布都增加
					int tem_crash_recv_fabuCode = 0;// 四架机都不在冲突发布阶段1
					for(int m =0;m < UAV_MAX_NUM; m++)
					{
						if(g_lineCrashState[m].recv_fabuCode == 1)
						{
							tem_crash_recv_fabuCode = 1;
							break;
						}
					}
					if(tem_crash_recv_fabuCode != 1 /*&& g_lineCrashState[1].PayloadReplan != 2*/)
					{
						//阶段数增加
						global_stage += 1;  //需考虑多架无人机的阶段数
					}

					//第二次发布成功后，清空状态
					if(blk_ofp_ccc_154.plan_query == 3)
						blk_ofp_ccc_154.plan_query = 0;
					//航线查询开始
					hx_cx_flag = 1;
					//应用空域
					init_blk_ctas_dtms_010();

//					for(int j = 0 ; j < UAV_MAX_NUM; j ++)
//					{
//						// 冲突时需要反馈弹窗消隐
//						if (g_lineCrashState[j].hasConflict == 1) {
//							// 恢复弹窗关闭（再确认）
//							memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
//							blk_ccc_ofp_032.tips_type = 0;
//							send_blk_ccc_ofp_032();
//						}
//
//						// 发布成功冲突消解(第二阶段发布才算)
//						if (g_lineCrashState[j].hasConflict == 1 && g_lineCrashState[j].recv_fabuCode == 2 ) {
//							memset(&g_lineCrashState[j], 0, sizeof(g_lineCrashState));
//						}
//					}
				}
				}
			else if (s4D_frame_40[i] == -1)
			{
				//初始化无人机数据
				for (int i = 0; i < 4; i++)
				{
					memset(&uav_send[i], 0, sizeof(UAV_SEND));
				}
				//发送发布失败状态到综显
				sprintf(CCC_DPU_data_0.failreason, "发送航线切换指令返回失败");
				//是否为攻击
				if(fabu_route_type == 2)
				{
					scheme_generation_state(2,2, 3, 2);
				}
				else if(fabu_route_type == 1)
				{
					scheme_generation_state(1,2, 3, 2);
				}
				memset(CCC_DPU_data_0.failreason, 0, 200);
					g_fabu_active = 0;
				if (single_uav_flag == 33)
				{
					//单无人机注入结束
					single_uav_flag = 0;
				}
				printf("0x40 failed! %d\n", s4D_frame_40[i]);
				//装订出错后初始化
				s4D_frame_40[i] = 0;
				//初始化变量
				g_fabu_switch_ack_cnt = 0;
				g_fabu_send_uav_num = 0;
				g_fabu_switch_timeout = 0;
				uav_hl_confirm = 0;

//				for(int j = 0 ; j < UAV_MAX_NUM; j ++)
//				{
//					// 重规划发布失败
//					if (g_lineCrashState[j].hasConflict == 1) {
//						memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
//
//						// 发布失败：重规划规避阶段发布13
//						if(g_lineCrashState[j].recv_fabuCode == 1){
//							blk_ccc_ofp_032.tips_type = 13;
//						}else if(g_lineCrashState[j].recv_fabuCode == 2){// 发布失败：重规划消解阶段发布14
//							blk_ccc_ofp_032.tips_type = 14;
//						}else{
//
//						}
//						send_blk_ccc_ofp_032();
//					}
//				}


				//退出函数
				return;
			}
			else if (g_fabu_switch_timeout > 200)
			{
				//初始化无人机数据
				for (int i = 0; i < 4; i++)
				{
					memset(&uav_send[i], 0, sizeof(UAV_SEND));
				}
				//发送发布失败状态到综显
				sprintf(CCC_DPU_data_0.failreason, "发送航线切换指令超时");
				//是否为攻击
				if(fabu_route_type == 2)
				{
					scheme_generation_state(2,2, 3, 2);
				}
				else if(fabu_route_type == 1)
				{
					scheme_generation_state(1,2, 3, 2);
				}
				memset(CCC_DPU_data_0.failreason, 0, 200);
					g_fabu_active = 0;
					printf("0x40 timeout! %d\n", s4D_frame_40[i]);
				if (single_uav_flag == 33)
				{
					//单无人机注入结束
					single_uav_flag = 0;
				}
				//装订出错后初始化
				s4D_frame_40[i] = 0;
				//初始化变量
				g_fabu_switch_ack_cnt = 0;
				g_fabu_send_uav_num = 0;
				g_fabu_switch_timeout = 0;
				uav_hl_confirm = 0;

//				for(int j = 0 ; j < UAV_MAX_NUM; j ++)
//				{
//					// 重规划发布失败
//					if (g_lineCrashState[j].hasConflict == 1) {
//						memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
//
//						// 发布失败：重规划规避阶段发布13
//						if(g_lineCrashState[j].recv_fabuCode == 1){
//							blk_ccc_ofp_032.tips_type = 13;
//						}else if(g_lineCrashState[j].recv_fabuCode == 2){// 发布失败：重规划消解阶段发布14
//							blk_ccc_ofp_032.tips_type = 14;
//						}else{
//
//						}
//
//						send_blk_ccc_ofp_032();
//					}
//				}


				//退出函数
				return;
			}
		}
	}
}

// 无人机飞仿控制
/*
 * 完成无人机航路规划功能后，综显对我们规划好的航线进行选择，将选定的航线返回来，将这个航线根据无人机id发给相应的无人机
 */
void uav_simulation()
{
	if(BDFX_double_status == 0)
	{
		g_bdfx_double_busy_cnt = 0;
	}
	else
	{
		g_bdfx_double_busy_cnt++;
	}

	// 接收综显航线发布命令
	recv_dpu1_dpu2(DDSTables.DPU_CCC_29.niConnectionId,
			DDSTables.DPU2_CCC_29.niConnectionId, &blk_ofp_ccc_039,
			sizeof blk_ofp_ccc_039);

	//收不到就收PAD new20250620
	if (enRetCode != 0)
	{
		Receive_Message(DDSTables.PAD_CCC_039.niConnectionId, 0,
				&transaction_id, &blk_ofp_ccc_039, &message_type_id,
				&message_size, &enRetCode);
	}
	if (enRetCode == 0)
	{
		if(blk_ofp_ccc_039.routeType == 3)
		{
			int restart_bdfx = 0;
			//空闲态可直接启动
			if(BDFX_double_status == 0)
			{
				restart_bdfx = 1;
			}
			//收到新方案号，允许重启双机发布
			else if(blk_ofp_ccc_039.Plan_ID != g_bdfx_double_plan_id)
			{
				restart_bdfx = 1;
			}
			//同方案长时间卡住时，允许通过再次发布触发重启
			else if(g_bdfx_double_busy_cnt > 300)
			{
				restart_bdfx = 1;
			}

			if(restart_bdfx == 1)
			{
				//初始化双机编队回报
				for(int uav = 0 ; uav < 4 ; uav ++)
				{
					b2_frame_30[uav] = 0;
					b2_frame_14[uav] = 0;
					s4D_frame_40[uav] = 0;
                    memset(&uav_send[uav], 0, sizeof(UAV_SEND));
                    s4D_frame_38[uav] = 0;
                    send_area[uav] = 0;
                    hx_change_uav_id[uav] = 0;
				}
				dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_011.niConnectionId);
				dtms_flush_dds_topic(DDSTables.BLK_CTAS_DTMS_009.niConnectionId);
				g_fabu_active = 0;
                g_fabu_plan_id = 0;
                g_fabu_route_type = 0;
                g_fabu_stage_id = 0;
                g_fabu_send_uav_num = 0;
                g_fabu_switch_timeout = 0;
                g_fabu_switch_ack_cnt = 0;
                uav_hl_confirm = 0;
                uav_change = 0;
                send_flag = 0;
                count = 0;
                frame_count = 1;
                track_point_count = -1;
                send_count = 0;
                BDFX_status = 0;
				scheme_generation_state(2,2, 1, 2); // 返回发布状态到综显
				BDFX_double_status = 1;
				g_bdfx_double_plan_id = blk_ofp_ccc_039.Plan_ID;
				g_bdfx_double_busy_cnt = 0;
				printf("BDFX start/restart plan=%u status=%d\n",blk_ofp_ccc_039.Plan_ID,BDFX_double_status);
			}
			else
			{
				printf("BDFX ignore duplicate plan=%u status=%d busy=%d\n",blk_ofp_ccc_039.Plan_ID,BDFX_double_status,g_bdfx_double_busy_cnt);
			}
			return;
		}

		if(blk_ofp_ccc_039.routeType == 1 || blk_ofp_ccc_039.routeType == 2)
		{
			int same_fabu = 0;
			if (g_fabu_active == 1 && blk_ofp_ccc_039.Plan_ID == g_fabu_plan_id && blk_ofp_ccc_039.routeType == g_fabu_route_type && blk_ofp_ccc_039.stage_id == g_fabu_stage_id)
			{
				same_fabu = 1;
			}

			if(same_fabu == 1)
			{
				printf("FABU ignore duplicate plan=%u route=%u stage=%u hl=%d\n",blk_ofp_ccc_039.Plan_ID,blk_ofp_ccc_039.routeType,blk_ofp_ccc_039.stage_id,uav_hl_confirm);
			}
			else
			{
				for(int drone_index = 0;drone_index<4;drone_index++)
				{
					g_recv_fabuCode[drone_index] = 1;
				}
			}
		}
		else
		{
			// 非航线发布命令，忽略
		}

		// 发布时需要做冲突检测
//		avoidLineCrashJudgeProc();
	}


	// 发布处理
	faBuProc();

}

void send_track_change()
{

	if(send_flag == 0)
	{
		for(unsigned int i = 1; i < 5 ; i++ )
		{
			//使用hx_change_uav_id变量控制需要发送切换指令的无人机id
			if(hx_change_uav_id[i - 1] != 0)
			{
				//找到需要切换的无人机id
				send_flag = i ;
				break;
			}
			//遍历到结尾，没有要切换的无人机清空标志位
			else if(i == 4)
			{
				uav_change = 0;
			}
		}
	}
	//找到可以切换的id并且允许发送
	if (send_flag && uav_change == 1)
	{
		//切换指令需发送4次
		if( count < 4)
		{
			// 0x40为航线切换指令
			send_uav_hl(hx_change_uav_id[send_flag - 1],0x40);
			//			send_uav_hl(hx_change_uav_id[send_flag - 1],0x0);//发送空指令（每个指令后都需要发送空指令）
			send_index = send_flag - 1 ;
			count++;
		}
		else
		{
			//4次切换指令发完后，各控制标志位初始化
			hx_change_uav_id[send_flag - 1] = 0;
			send_flag = 0;
			count = 0;
		}
	}
}


// 发送无人机航路给飞仿
void send_uav_hl(unsigned int uav_id, unsigned char order)
{
	//    send_array.clear();//清空
	//    send_array.resize(2400);

	// 清空指令帧内容 防止干扰
	memset(&order_data_frames,0,sizeof(order_data_frame));
	memset(&track_threat_frames,0,sizeof(track_threat_frame));

	// 无人机航线与威胁帧初始化
	track_threat_frames.tongbu_code = 0xaa55; // 同步码
	track_threat_frames.frame_count = frame_count; // 帧计数
	//指令码赋值(三判二)
	memset(&(track_threat_frames.order_code),order,3);
	// 根据指令处理具体指令帧内容
	switch (order)
	{
	case 0x00:
	{ // 空指令
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data),&(order_data_frames.empty_orders),sizeof(order_data_frames.empty_orders));
	}
	break;
	case 0x30:
	{ // 定点悬停位置注入指令
		// 给具体指令赋值
		order_data_frames.ddxtwzzr_orders.lat = DPU_CCC_data_11.waypoints_cursorSelection_longitude_and_latitude_synt.latitude / lat_scale;
		order_data_frames.ddxtwzzr_orders.lon = DPU_CCC_data_11.waypoints_cursorSelection_longitude_and_latitude_synt.longitude / lon_scale;
		order_data_frames.ddxtwzzr_orders.xd_height = 200 / gaodu_scale;
		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data),&(order_data_frames.ddxtwzzr_orders),sizeof(order_data_frames.ddxtwzzr_orders));
	}
	break;
	case 0x32:
	{ // 应返航机场点注入指令
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data),&(order_data_frames.fhjczr_orders),sizeof(order_data_frames.fhjczr_orders));
	}
	break;
	case 0x34:
	{ // 应急返航指令
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data),&(order_data_frames.yjfh_orders),sizeof(order_data_frames.yjfh_orders));
	}
	break;
	case 0x4A:
	{ // 向兴趣点飞指令
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data),&(order_data_frames.fly_to_xq_order),sizeof(order_data_frames.fly_to_xq_order));
	}
	break;
	case 0x36:
	{ // 航点插入指令
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data),&(order_data_frames.track_point_insert_orders),sizeof(order_data_frames.track_point_insert_orders));
	}
	break;
	case 0x38:
	{ // 航点（绝对）装订指令
		// 发送第一个点 机场点，手动装订机场点
		if (uav_send[send_index].first_flag == 0)
		{

			order_data_frames.track_point_jd_order.track_line_id = track_line;   // 当前航线号，起始航线号+无人机默认编号
			if(track_line == uav_route[send_index].route_number)
			{
				order_data_frames.track_point_jd_order.track_line_id--;
			}
			order_data_frames.track_point_jd_order.hb_height = HEIGHT / gaodu_scale;
			order_data_frames.track_point_jd_order.speed = 35/ speed_scale;

			//当前航点号和下一个航点号赋值
			order_data_frames.track_point_jd_order.track_point_id = 1; // 机场点航点号为1
			//order_data_frames.track_point_jd_order.next_track_point_id = order_data_frames.track_point_jd_order.track_point_id + 1;
			//有人机
			order_data_frames.track_point_jd_order.lat = get_airport_lat(send_index) / lat_scale;
			order_data_frames.track_point_jd_order.lon = get_airport_lon(send_index) / lon_scale;
			//特征字清零
			memset(&(order_data_frames.track_point_jd_order.tezhenzi),0,sizeof(order_data_frames.track_point_jd_order.tezhenzi));

			//特征字赋值
			order_data_frames.track_point_jd_order.tezhenzi[0] = 0x07;
			order_data_frames.track_point_jd_order.tezhenzi[7] = 0x01;
			//                char check_num = 0x01;
			//                memcpy(&(track_threat_frames.check_code)+1,&(check_num),1);
		}

		else if (track_point_count < uav_send[send_index].waypoints_number - 3 && uav_send[send_index].first_flag == 1)
		{ // 航点计数

			order_data_frames.track_point_jd_order.track_line_id = track_line;   // 当前航线号，起始航线号+无人机默认编号
			if(track_line== uav_route[send_index].route_number)
			{
				order_data_frames.track_point_jd_order.track_line_id--;
			}
			// 机场点航点号为1，所以一般航点航点号从2开始增加
			//给第一个一般航路点赋值时，track_point_count = 0
			order_data_frames.track_point_jd_order.track_point_id = track_point_count + 2;
			order_data_frames.track_point_jd_order.hb_height = (uav_send[send_index].waypoint_informations[track_point_count].height)/ gaodu_scale; //海拔高度
			order_data_frames.track_point_jd_order.speed = uav_send[send_index].waypoint_informations[track_point_count].speed / speed_scale; //速度
			//航迹点经纬度
			order_data_frames.track_point_jd_order.lat = uav_send[send_index].waypoint_informations[track_point_count].latitude / lat_scale;
			order_data_frames.track_point_jd_order.lon = uav_send[send_index].waypoint_informations[track_point_count].longitude / lon_scale;

			memset(&(order_data_frames.track_point_jd_order.tezhenzi),0,sizeof(order_data_frames.track_point_jd_order.tezhenzi));

			//生成的最后一个航路点为盘旋点，搜索类航线
			if(track_point_count == uav_send[send_index].waypoints_number -4 && uav_send[send_index].task_index < 7)
			{
				//特征字赋值
				order_data_frames.track_point_jd_order.tezhenzi[0] = 0x02;//顺时针圆心
				//待机半径
				short rad = uav_send[send_index].waypoint_informations[track_point_count].standby_radius;
				//                short rad = 1500;
				memcpy(&order_data_frames.track_point_jd_order.tezhenzi[1],&rad,sizeof(short));//待机半径
				//待机圈数
				short circle = uav_send[send_index].waypoint_informations[track_point_count].standby_time_lapsNumber_cycleNumber;
				memcpy(&order_data_frames.track_point_jd_order.tezhenzi[3],&circle,sizeof(short));
				//                order_data_frames.track_point_jd_order.tezhenzi[3] = 0xff;//待机圈数
				//                order_data_frames.track_point_jd_order.tezhenzi[4] = 0xff;//待机圈数
				order_data_frames.track_point_jd_order.tezhenzi[5] = 0;//待机距离
				order_data_frames.track_point_jd_order.tezhenzi[6] = 0;//待机距离
				order_data_frames.track_point_jd_order.tezhenzi[7] = 0x04;//圈数
			}
			//悬停航点
			else if(track_point_count == uav_send[send_index].waypoints_number -4 && uav_send[send_index].task_index == 7)
			{
				//特征字赋值
				order_data_frames.track_point_jd_order.tezhenzi[0] = 0x08;//悬停航路点
				order_data_frames.track_point_jd_order.tezhenzi[1] = 0;//待机半径
				order_data_frames.track_point_jd_order.tezhenzi[2] = 0;//待机半径
				//待机时间
				//                short time = uav_send[send_index].waypoint_informations[track_point_count].standby_time_lapsNumber_cycleNumber;
				//				memcpy(&order_data_frames.track_point_jd_order.tezhenzi[3],&time,sizeof(short));
				order_data_frames.track_point_jd_order.tezhenzi[3] = 0xff;//待机时间
				order_data_frames.track_point_jd_order.tezhenzi[4] = 0xff;//待机时间
				order_data_frames.track_point_jd_order.tezhenzi[5] = 0;//待机距离
				order_data_frames.track_point_jd_order.tezhenzi[6] = 0;//待机距离
				order_data_frames.track_point_jd_order.tezhenzi[7] = 0;//时间
			}
			//盘旋航点
			else if(track_point_count == uav_send[send_index].waypoints_number -4 && uav_send[send_index].task_index == 8)
			{
				//特征字赋值
				order_data_frames.track_point_jd_order.tezhenzi[0] = 0x02;//顺时针圆心
				//待机半径
				short rad = uav_send[send_index].waypoint_informations[track_point_count].standby_radius;
				memcpy(&order_data_frames.track_point_jd_order.tezhenzi[1],&rad,sizeof(short));
				//待机圈数
				short circle = uav_send[send_index].waypoint_informations[track_point_count].standby_time_lapsNumber_cycleNumber;
				memcpy(&order_data_frames.track_point_jd_order.tezhenzi[3],&circle,sizeof(short));
				//				order_data_frames.track_point_jd_order.tezhenzi[3] = 0xff;//待机圈数
				//				order_data_frames.track_point_jd_order.tezhenzi[4] = 0xff;//待机圈数
				order_data_frames.track_point_jd_order.tezhenzi[5] = 0;//待机距离
				order_data_frames.track_point_jd_order.tezhenzi[6] = 0;//待机距离
				order_data_frames.track_point_jd_order.tezhenzi[7] = 0x04;//圈数
			}
			else if(uav_send[send_index].waypoint_informations[track_point_count].causality == 3)
			{
				//特征字赋值
				order_data_frames.track_point_jd_order.tezhenzi[0] = 0x02;//顺时针圆心
				//待机半径
				short rad = uav_send[send_index].waypoint_informations[track_point_count].standby_radius;
				memcpy(&order_data_frames.track_point_jd_order.tezhenzi[1],&rad,sizeof(short));
				//待机圈数
				short circle = uav_send[send_index].waypoint_informations[track_point_count].standby_time_lapsNumber_cycleNumber;
				memcpy(&order_data_frames.track_point_jd_order.tezhenzi[3],&circle,sizeof(short));
				//				order_data_frames.track_point_jd_order.tezhenzi[3] = 0xff;//待机圈数
				//				order_data_frames.track_point_jd_order.tezhenzi[4] = 0xff;//待机圈数
				order_data_frames.track_point_jd_order.tezhenzi[5] = 0;//待机距离
				order_data_frames.track_point_jd_order.tezhenzi[6] = 0;//待机距离
				order_data_frames.track_point_jd_order.tezhenzi[7] = 0x04;//圈数
			}

		}
		else if(track_point_count == uav_send[send_index].waypoints_number - 3)// 发送倒数第二个点
		{
			order_data_frames.track_point_jd_order.track_line_id = track_line;   // 当前航线号，起始航线号+无人机默认编号
			if(track_line== uav_route[send_index].route_number)
			{
				order_data_frames.track_point_jd_order.track_line_id--;
			}
			// 航点号  机场点+一般航迹点+倒数第二个计算点，所以是+2
			order_data_frames.track_point_jd_order.track_point_id = uav_send[send_index].waypoints_number - 1;
			order_data_frames.track_point_jd_order.hb_height = (400)/ gaodu_scale;
			order_data_frames.track_point_jd_order.speed = 25  /speed_scale;

			//最后一个机场点是定点,一般航路点的最后一个点作为动点
			Point last_second;
			last_second = last_second_point(get_airport_lat(send_index),get_airport_lon(send_index),
					uav_send[send_index].waypoint_informations[track_point_count-1].latitude,
					uav_send[send_index].waypoint_informations[track_point_count-1].longitude);


			//使用返航航线的倒数第二个点的经纬度作为注入航线的倒数第二个解算点
			int point_index = 0;
			point_index = load_file.blk_dlr_ccc_045[send_index].normal_num-2;
//			order_data_frames.track_point_jd_order.lat = load_file.blk_dlr_ccc_045[send_index].normal_point[point_index].lat / lat_scale;
//			order_data_frames.track_point_jd_order.lon = load_file.blk_dlr_ccc_045[send_index].normal_point[point_index].lon /lon_scale;
//			order_data_frames.track_point_jd_order.lat = 47.55793 / lat_scale;//塔哈机场写死 47.55793
//			order_data_frames.track_point_jd_order.lon = 124.24993 /lon_scale;//塔哈机场写死 124.24993

			order_data_frames.track_point_jd_order.lat = last_second.lat / lat_scale;
			order_data_frames.track_point_jd_order.lon = last_second.lon /lon_scale;//260201

			memset(&(order_data_frames.track_point_jd_order.tezhenzi),0,sizeof(order_data_frames.track_point_jd_order.tezhenzi));


		}
		else if (track_point_count >= uav_send[send_index].waypoints_number - 2)
		{ // 发送最后一个点 机舰点

			order_data_frames.track_point_jd_order.track_line_id = track_line;   // 当前航线号，起始航线号+无人机默认编号
			if(track_line== uav_route[send_index].route_number)
			{
				order_data_frames.track_point_jd_order.track_line_id--;
			}
			// 航点号  机场点+一般航迹点+倒数第二个计算点+最后一个汇合点，所以是+3
			order_data_frames.track_point_jd_order.track_point_id = uav_send[send_index].waypoints_number;
			order_data_frames.track_point_jd_order.hb_height = HEIGHT/ gaodu_scale;
			order_data_frames.track_point_jd_order.speed = 35  /speed_scale;

			order_data_frames.track_point_jd_order.lat = get_airport_lat(send_index) / lat_scale;//260201
			order_data_frames.track_point_jd_order.lon = get_airport_lon(send_index) /lon_scale;


			memset(&(order_data_frames.track_point_jd_order.tezhenzi),0,sizeof(order_data_frames.track_point_jd_order.tezhenzi));

			//特征字赋值
			order_data_frames.track_point_jd_order.tezhenzi[0] = 0x07;
			order_data_frames.track_point_jd_order.tezhenzi[7] = 0x01;
		}
		// 特征字赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data), &(order_data_frames.track_point_jd_order), sizeof(order_data_frames.track_point_jd_order));

		//		printf("lat %d lon %d \n",order_data_frames.track_point_jd_order.lat,order_data_frames.track_point_jd_order.lon);
	}
	break;
	case 0x39:
	{ // 航点装订(相对）指令
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data), &(order_data_frames.track_point_xd_orders), sizeof(order_data_frames.track_point_xd_orders));
	}
	break;
	case 0x3A:
	{ // 航点修改指令
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data), &(order_data_frames.track_point_modfiy_orders), sizeof(order_data_frames.track_point_modfiy_orders));
	}
	break;
	case 0x3C:
	{ // 绝对航线、航点查询指令
		// 给具体指令赋值
		order_data_frames.jd_track_point_search_orders.track_line_id = uav_route[send_index].route_number;
		order_data_frames.jd_track_point_search_orders.track_point_id = 0;
		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data), &(order_data_frames.jd_track_point_search_orders), sizeof(order_data_frames.jd_track_point_search_orders));
	}
	break;
	case 0x3D:
	{ // 相对航线、航点查询指令
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data), &(order_data_frames.xd_track_point_search_orders), sizeof(order_data_frames.xd_track_point_search_orders));
	}
	break;
	case 0x3E:
	{ // 航线、航点删除指令
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data), &(order_data_frames.track_point_delete_orders), sizeof(order_data_frames.track_point_delete_orders));
	}
	break;
	case 0x40:
	{ // 航线、航点切换指令
		// 给具体指令赋值
		order_data_frames.track_point_chage_orders.track_line_id = track_line; // 航线号
		if (track_line == uav_route[send_index].route_number)
		{
			order_data_frames.track_point_chage_orders.track_line_id--;
		}
		// 航线航点切换时，航线号为新航线号，航点号应为第二个航点（第一个航点为机场点，导致无人机无法改航，航线装订失败）
		//             order_data_frames.track_point_chage_orders.track_point_id = 0;
		order_data_frames.track_point_chage_orders.track_point_id = 2;
		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data), &(order_data_frames.track_point_chage_orders), sizeof(order_data_frames.track_point_chage_orders));
	}
	break;
	case 0x41:
	{ // 航线长度查询指令
		// 给具体指令赋值
		order_data_frames.track_length_search_orders.all_tracks = uav_route[send_index].route_number;
		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data), &(order_data_frames.track_length_search_orders), sizeof(order_data_frames.track_length_search_orders));
	}
	break;
	case 0x42:
	{ // 威胁数据注入
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data), &(order_data_frames.threat_data_intos), sizeof(order_data_frames.threat_data_intos));
	}
	break;
	case 0x44:
	{ // 威胁区查询指令
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data), &(order_data_frames.threat_area_searchs), sizeof(order_data_frames.threat_area_searchs));
	}
	break;
	case 0x46:
	{ // 威胁区删除指令
		// 给具体指令赋值

		// 将指令存入遥控帧
		memcpy(&(track_threat_frames.order_data), &(order_data_frames.threat_area_deletes), sizeof(order_data_frames.threat_area_deletes));
	}
	break;
	default:
	{
		// qDebug()<<"未知遥控指令";
	}
	break;
	}

	//   TODO  未知代码，不确定功能但编写存在问题，会影响上面航线注入函数，已移动至 send_yaokongframe()

	// 只发航线帧
	//ToDo校验和可能存在问题
	unsigned short frame_check_code = do_crc_table((unsigned char*)&track_threat_frames.frame_count,sizeof(track_threat_frame)-4);
	track_threat_frames.check_code = frame_check_code;

	align_send_information(&track_threat_frames,sizeof(track_threat_frames),0);


	if (send_count < 4)
	{				  // 单点发送四次
		send_count++; // 发一次加一次
	}
	//一帧（一条航线的一个航点）发送4次
	if (send_count == 4)
	{
		send_count = 0;  // 发送次数置 0
		uav_send[send_index].first_flag  = 1; // 第一个点发送完
		track_point_count++; // track_point（航点）发完第一点四次后 发送下一点
	}
	frame_count++; // 发送成功一次帧计数加1


}

//倒数第二个点的经纬度计算
Point last_second_point(double lat1, double lon1,double lat2, double lon2)
{
	Point target;
	double dsitance = calculate_distances(lat1,lon1,lat2,lon2);

	//距离大于2.5km时
	if(dsitance > 2.5)
	{
		double azimuth_AB = calculate_azimuth(lat1,lon1,lat2,lon2);
		calculate_endpoint(lat1 , lon1 , azimuth_AB , 2.5 , &target.lat , &target.lon );
	}
	//距离小于2.5km时
	else
	{
		double azimuth_AB = calculate_azimuth(lat1,lon1,lat2,lon2);
		double reverse_azimuth = azimuth_AB + M_PI;
		if(reverse_azimuth >= 2 * M_PI)
		{
			reverse_azimuth -= 2 * M_PI;
		}
		calculate_endpoint(lat1 , lon1 , reverse_azimuth , 2.5 , &target.lat , &target.lon );
	}

	return target;
}

//角度转弧度
double deg2rad(double deg)
{
	return deg * M_PI / 180.0;
}
//弧度转角度
double rad2deg(double rad)
{
	return rad* 180.0 / M_PI;
}
//计算从点1到点2的初始方位角(弧度，范围0,2π)
double calculate_azimuth(double lat1, double lon1,double lat2, double lon2)
{
	double lat1_rad = deg2rad(lat1);
	double lat2_rad = deg2rad(lat2);
	double lon1_rad = deg2rad(lon1);
	double lon2_rad = deg2rad(lon2);

	double	dlon = lon2_rad - lon1_rad;
	double y = sin(dlon) * cos(lat2_rad);
	double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dlon);

	double azimuth = atan2(y,x);

	//转换为0到2π范围
	if(azimuth < 0)
	{
		azimuth += 2 * M_PI;
	}

	return azimuth;
}
//根据起点、方位角和距离计算终点经纬度
void calculate_endpoint(double start_lat,double start_lon,double azimuth,double distance,double *end_lat,double *end_lon)
{
	double start_lat_rad = deg2rad(start_lat);
	double start_lon_rad = deg2rad(start_lon);

	double angular_dist = distance / 6371.0;//转换为弧度
	double sin_ang = sin(angular_dist);
	double cos_ang = cos(angular_dist);

	//计算终点纬度
	double phi1 = asin(sin(start_lat_rad) * cos_ang + cos(start_lat_rad) * sin_ang * cos(azimuth));
	//计算终点经度
	double lambda1 = start_lon_rad + atan2(sin(azimuth) * sin_ang * cos(start_lat_rad) , cos_ang - sin(start_lat_rad) * sin(phi1));

	*end_lat = rad2deg(phi1);
	*end_lon = rad2deg(lambda1);

	//调整经度到-180,180范围内
	*end_lon = fmod(*end_lon, 360.0);
	if(*end_lon > 180.0)
	{
		*end_lon -= 360.0;
	}
	else if(*end_lon <-180.0)
	{
		*end_lon += 360.0;
	}
}
/*************************************** 控制权交接模块 ***************************************/

void send_blk_ccc_ofp_120(unsigned char response)
{
	blk_ccc_ofp_120.response = response;
	data_length = sizeof(BLK_CCC_OFP_120);
	Send_Message(DDSTables.CCC_DPU_43.niConnectionId,0,&transaction_id, &blk_ccc_ofp_120, &message_type_id, data_length, &enRetCode);
}
void recv_blk_ofp_ccc_148()
{
	//接收综显的事件指令
	static int ykzl = 0;
	static int cnt = 0;
	message_size = RECV_MAX_SIZE;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_049.niConnectionId,DDSTables.DPU2_CCC_049.niConnectionId,&blk_ofp_ccc_148,sizeof blk_ofp_ccc_148);
	if(enRetCode == 0)
	{
		//查询
		if(blk_ofp_ccc_148.op_code == 1)
		{
			ykzl = 1;
			cnt = 3;
		}
		//        //自动返航
		//    	else if(1)
		//        {
		//            ykzl = 1;
		//            cnt = 3;
		//        }
		//        //退出返航
		//        else if(2)
		//        {
		//            ykzl = 2;
		//            cnt = 3;
		//        }
		//高度调整
		else if(blk_ofp_ccc_148.op_code == 2)
		{
			//遥调中
			send_blk_ccc_ofp_120(3);
			int index = blk_ofp_ccc_148.uavSn - 1;//无人机序号
			s4D_frame_12[index] = 0;
			ykzl = 3;
			cnt = 3;
		}
		//速度调整
		else if(blk_ofp_ccc_148.op_code == 3)
		{
			//遥调中
			send_blk_ccc_ofp_120(3);
			int index = blk_ofp_ccc_148.uavSn - 1;//无人机序号
			s4D_frame_0c[index] = 0;
			ykzl = 4;
			cnt = 3;
		}
	}
	//查询事件
	static int cx_flag = 0;
	//调高事件
	static int tg_flag = 0;
	//调速事件
	static int ts_flag = 0;
	//事件处理三次
	if(ykzl > 0 && cnt > 0)
	{
		int index = blk_ofp_ccc_148.uavSn - 1;//无人机序号
		switch(ykzl)
		{
		case 1:
			//查询
			//查询标志置位，事件开始
			cx_flag = 1;

			//        		//判断是否为任务航线
			//        		if(uav_route[index].route_number != 5 || uav_route[index].route_number != 6)
			//        		{
			//        			//返回查询失败，当前未发布任务航线
			//        			send_blk_ccc_ofp_120(1);
			//        			//结束查询
			//					cx_flag = 0;
			//					//事件处理结束
			//					cnt = 0;
			//        			break;
			//        		}
			//一号航线
			if(uav_route[index].route_number == 1)
			{
				int dlr_index = 0;
				for(int i = 0 ; i < 4 ; i ++)
				{
					//找到对应飞机的正常航线
					if(load_file.blk_dlr_ccc_045[i].uav_id == blk_ofp_ccc_148.uavCode)
					{
						dlr_index = i;
					}
				}
				//是否存在下一航点
				if(uav_route[index].waypoint_number + 1 > load_file.blk_dlr_ccc_045[dlr_index].normal_num)
				{
					//无有效下一航点，即当前点已经是最后一个航点，返回失败
					send_blk_ccc_ofp_120(1);
					//结束查询
					cx_flag = 0;
					//事件处理结束
					cnt = 0;
					break;
				}

				//从缓存中读取下一航点信息
				int point = uav_route[index].waypoint_number;//+1下一航点 -1下标
				blk_ccc_ofp_120.uavSn = blk_ofp_ccc_148.uavSn;//无人机序号
				blk_ccc_ofp_120.uavCode = blk_ofp_ccc_148.uavCode;//无人机编号
				blk_ccc_ofp_120.pointNum = uav_route[index].waypoint_number + 1;//+1下一航点
				blk_ccc_ofp_120.longi = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].lon;
				blk_ccc_ofp_120.lati = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].lat;
				blk_ccc_ofp_120.height = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].height;
				blk_ccc_ofp_120.speed = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].speed * 3.6;
				//事件处理结束
				cnt = 0;
				break;
			}
			//判断是否还有下一有效航点
			if(uav_route[index].waypoint_number + 1 > uav_route[index].hull_number - 2)
			{
				//无有效下一航点，即当前点已经是最后一个航点，返回失败
				send_blk_ccc_ofp_120(1);
				//结束查询
				cx_flag = 0;
				//事件处理结束
				cnt = 0;
				break;
			}


			//从缓存中读取下一航点信息
			int point = uav_route[index].waypoint_number - 1;//+1下一航点 -1下标 -1机场点
			blk_ccc_ofp_120.uavSn = blk_ofp_ccc_148.uavSn;//无人机序号
			blk_ccc_ofp_120.uavCode = blk_ofp_ccc_148.uavCode;//无人机编号
			blk_ccc_ofp_120.pointNum = uav_route[index].waypoint_number;//+1下一航点 -1机场点
			blk_ccc_ofp_120.longi = uav_route[index].waypoint[point].longitude;
			blk_ccc_ofp_120.lati = uav_route[index].waypoint[point].latitude;
			blk_ccc_ofp_120.height = uav_route[index].waypoint[point].height;
			blk_ccc_ofp_120.speed = uav_route[index].waypoint[point].speed * 3.6;


			//事件处理结束
			cnt = 0;
			break;
			//            case 1:
			//            //返航
			//                blk_ccc_kkl_008_026_027_028[0].front.basic_yaotiao_order_frame_data.order_code[0] = 0x30;
			//                blk_ccc_kkl_008_026_027_028[0].front.basic_yaotiao_order_frame_data.order_code[1] = 0x30;
			//                blk_ccc_kkl_008_026_027_028[0].front.basic_yaotiao_order_frame_data.order_code[2] = 0x30;
			//                cnt--;
			//                break;
			//            case 2:
			//            //退出返航
			//                blk_ccc_kkl_008_026_027_028[0].front.basic_yaotiao_order_frame_data.order_code[0] = 0x26;
			//                blk_ccc_kkl_008_026_027_028[0].front.basic_yaotiao_order_frame_data.order_code[1] = 0x26;
			//                blk_ccc_kkl_008_026_027_028[0].front.basic_yaotiao_order_frame_data.order_code[2] = 0x26;
			//                cnt--;
			//                break;
		case 3:
			//高度调整
			//调高标志置位，事件开始
			tg_flag = 1;

			//一号航线
			if(uav_route[index].route_number == 1)
			{
				int dlr_index = 0;
				for(int i = 0 ; i < 4 ; i ++)
				{
					//找到对应飞机的正常航线
					if(load_file.blk_dlr_ccc_045[i].uav_id == blk_ofp_ccc_148.uavCode)
					{
						dlr_index = i;
					}
				}
				//是否存在下一航点
				if(uav_route[index].waypoint_number + 1 > load_file.blk_dlr_ccc_045[dlr_index].normal_num)
				{
					//无有效下一航点，即当前点已经是最后一个航点，返回失败
					send_blk_ccc_ofp_120(8);
					//结束调高
					tg_flag = 0;
					//事件处理结束
					cnt = 0;
					break;
				}

				//判断当前点是否已过点
				if(uav_route[index].waypoint_number + 1 > blk_ofp_ccc_148.pointNum /*综显发送的点号*/)
				{
					//从缓存中读取当前航点信息
					int point = uav_route[index].waypoint_number - 1;//-1下标
					blk_ccc_ofp_120.uavSn = blk_ofp_ccc_148.uavSn;//无人机序号
					blk_ccc_ofp_120.uavCode = blk_ofp_ccc_148.uavCode;//无人机编号
					blk_ccc_ofp_120.pointNum = uav_route[index].waypoint_number;//当前航点号
					blk_ccc_ofp_120.longi = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].lon;
					blk_ccc_ofp_120.lati = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].lat;
					blk_ccc_ofp_120.height = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].height;
					blk_ccc_ofp_120.speed = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].speed * 3.6;
					//已过点，返回超时，并把当前航点的信息回传
					send_blk_ccc_ofp_120(7);
					//结束调高
					tg_flag = 0;
					//事件处理结束
					cnt = 0;
					break;
				}
			}
			else
			{
				//判断是否还有下一有效航点
				if(blk_ofp_ccc_148.pointNum + 1/*综显发送的下一点号*/ > uav_route[index].hull_number - 2)/*减去倒数第二个机场点和计算点*/
				{
					//无有效下一航点，即当前点已经是最后一个航点，返回失败
					send_blk_ccc_ofp_120(8);
					//结束调高
					tg_flag = 0;
					//事件处理结束
					cnt = 0;
					break;
				}
				//判断当前点是否已过点
				if(uav_route[index].waypoint_number > blk_ofp_ccc_148.pointNum /*综显发送的点号*/)
				{
					//从缓存中读取当前航点信息
					int point = uav_route[index].waypoint_number-2;//-1下标 -1机场点
					blk_ccc_ofp_120.uavSn = blk_ofp_ccc_148.uavSn;//无人机序号
					blk_ccc_ofp_120.uavCode = blk_ofp_ccc_148.uavCode;//无人机编号
					blk_ccc_ofp_120.pointNum = uav_route[index].waypoint_number - 1;//-1机场点
					blk_ccc_ofp_120.longi = uav_route[index].waypoint[point].longitude;
					blk_ccc_ofp_120.lati = uav_route[index].waypoint[point].latitude;
					blk_ccc_ofp_120.height = uav_route[index].waypoint[point].height;
					blk_ccc_ofp_120.speed = uav_route[index].waypoint[point].speed * 3.6;
					//已过点，返回超时，并把当前航点的信息回传
					send_blk_ccc_ofp_120(7);
					//结束调高
					tg_flag = 0;
					//事件处理结束
					cnt = 0;
					break;
				}
			}



			blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.yt_start = 0x52;
			blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.order_code[0] = 0x12;//绝对高度遥调 ，相对高度遥调0x08
			blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.order_code[1] = 0x12;
			blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.order_code[2] = 0x12;
			blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.order_data[0] = blk_ofp_ccc_148.height /0.2;
			cnt--;
			break;
		case 4:
			//速度调整
			//调速标志置位，事件开始
			ts_flag = 1;
			//一号航线
			if(uav_route[index].route_number == 1)
			{
				int dlr_index = 0;
				for(int i = 0 ; i < 4 ; i ++)
				{
					//找到对应飞机的正常航线
					if(load_file.blk_dlr_ccc_045[i].uav_id == blk_ofp_ccc_148.uavCode)
					{
						dlr_index = i;
					}
				}
				//是否存在下一航点
				if(uav_route[index].waypoint_number + 1 > load_file.blk_dlr_ccc_045[dlr_index].normal_num)
				{
					//无有效下一航点，即当前点已经是最后一个航点，返回失败
					send_blk_ccc_ofp_120(8);
					//结束调速
					ts_flag = 0;
					//事件处理结束
					cnt = 0;
					break;
				}

				//判断当前点是否已过点
				if(uav_route[index].waypoint_number + 1 > blk_ofp_ccc_148.pointNum /*综显发送的点号*/)
				{
					//从缓存中读取当前航点信息
					int point = uav_route[index].waypoint_number - 1;//-1下标
					blk_ccc_ofp_120.uavSn = blk_ofp_ccc_148.uavSn;//无人机序号
					blk_ccc_ofp_120.uavCode = blk_ofp_ccc_148.uavCode;//无人机编号
					blk_ccc_ofp_120.pointNum = uav_route[index].waypoint_number;//当前航点号
					blk_ccc_ofp_120.longi = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].lon;
					blk_ccc_ofp_120.lati = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].lat;
					blk_ccc_ofp_120.height = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].height;
					blk_ccc_ofp_120.speed = load_file.blk_dlr_ccc_045[dlr_index].normal_point[point].speed * 3.6;
					//已过点，返回超时，并把当前航点的信息回传
					send_blk_ccc_ofp_120(7);
					//结束调速
					ts_flag = 0;
					//事件处理结束
					cnt = 0;
					break;
				}
			}
			else
			{
				//判断是否还有下一有效航点
				if(blk_ofp_ccc_148.pointNum + 1/*综显发送的下一点号*/ > uav_route[index].hull_number - 2)/*减去倒数第二个机场点和计算点*/
				{
					//无有效下一航点，即当前点已经是最后一个航点，返回失败
					send_blk_ccc_ofp_120(8);
					//结束调速
					ts_flag = 0;
					//事件处理结束
					cnt = 0;
					break;
				}
				//判断当前点是否已过点
				if(uav_route[index].waypoint_number > blk_ofp_ccc_148.pointNum /*综显发送的点号*/)
				{
					//从缓存中读取当前航点信息
					int point = uav_route[index].waypoint_number-2;//-1下标 -1机场点
					blk_ccc_ofp_120.uavSn = blk_ofp_ccc_148.uavSn;//无人机序号
					blk_ccc_ofp_120.uavCode = blk_ofp_ccc_148.uavCode;//无人机编号
					blk_ccc_ofp_120.pointNum = uav_route[index].waypoint_number - 1;//-1机场点
					blk_ccc_ofp_120.longi = uav_route[index].waypoint[point].longitude;
					blk_ccc_ofp_120.lati = uav_route[index].waypoint[point].latitude;
					blk_ccc_ofp_120.height = uav_route[index].waypoint[point].height;
					blk_ccc_ofp_120.speed = uav_route[index].waypoint[point].speed * 3.6;
					//已过点，返回超时，并把当前航点的信息回传
					send_blk_ccc_ofp_120(7);
					//结束调速
					ts_flag = 0;
					//事件处理结束
					cnt = 0;
					break;
				}
			}


			blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.yt_start = 0x52;
			blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.order_code[0] = 0x0c;
			blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.order_code[1] = 0x0c;
			blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.order_code[2] = 0x0c;
			blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.order_data[0] = (blk_ofp_ccc_148.speed / 3.6) /0.01;//纵向
			blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.order_data[1] = 0;//横向
			cnt--;
			break;
		}
	}

	//发送查询回报
	if(cx_flag == 1)
	{
		//把数据发送给综显,查询成功
		send_blk_ccc_ofp_120(2);
		//标志清空
		cx_flag = 0;
	}
	static int time_out = 0;
	//发送调高回报
	if(tg_flag == 1)
	{
		time_out++;
		//遥调回报成功
		int index = blk_ofp_ccc_148.uavSn - 1;//无人机序号
		if(s4D_frame_12[index] == 1)
		{
			//把数据发送给综显
			blk_ccc_ofp_120.uavSn = blk_ofp_ccc_148.uavSn;//无人机序号
			blk_ccc_ofp_120.uavCode = blk_ofp_ccc_148.uavCode;//无人机编号
			blk_ccc_ofp_120.pointNum = blk_ofp_ccc_148.pointNum;
			blk_ccc_ofp_120.longi = blk_ofp_ccc_148.longi;
			blk_ccc_ofp_120.lati = blk_ofp_ccc_148.lati;
			blk_ccc_ofp_120.height = blk_ofp_ccc_148.height;
			blk_ccc_ofp_120.speed = blk_ofp_ccc_148.speed;
			//遥调成功
			send_blk_ccc_ofp_120(4);
			//更新缓存航点高度,1号航线不更新
			if(uav_route[index].route_number != 1)
			{
				int point = blk_ofp_ccc_148.pointNum - 2;//-1下标 -1机场点
				uav_route[index].waypoint[point].height = blk_ofp_ccc_148.height;
			}
			//标志清空
			tg_flag = 0;
			time_out = 0;
		}
		else if(s4D_frame_12[index] == -1)
		{
			//遥调失败
			send_blk_ccc_ofp_120(5);
			//标志清空
			tg_flag = 0;
			time_out = 0;
		}
		//超时
		else if(time_out > 200)
		{
			//遥调超时
			send_blk_ccc_ofp_120(6);
			//标志清空
			tg_flag = 0;
			time_out = 0;
		}
	}
	//发送调速回报
	if(ts_flag == 1)
	{
		time_out++;
		//遥调回报成功
		int index = blk_ofp_ccc_148.uavSn - 1;//无人机序号
		if(s4D_frame_0c[index] == 1)
		{
			//把数据发送给综显
			blk_ccc_ofp_120.uavSn = blk_ofp_ccc_148.uavSn;//无人机序号
			blk_ccc_ofp_120.uavCode = blk_ofp_ccc_148.uavCode;//无人机编号
			blk_ccc_ofp_120.pointNum = blk_ofp_ccc_148.pointNum;
			blk_ccc_ofp_120.longi = blk_ofp_ccc_148.longi;
			blk_ccc_ofp_120.lati = blk_ofp_ccc_148.lati;
			blk_ccc_ofp_120.height = blk_ofp_ccc_148.height;
			blk_ccc_ofp_120.speed = blk_ofp_ccc_148.speed;
			//遥调成功
			send_blk_ccc_ofp_120(4);
			//更新缓存航点高度,1号航线不更新
			if(uav_route[index].route_number != 1)
			{
				int point = blk_ofp_ccc_148.pointNum - 2;//-1下标 -1机场点
				uav_route[index].waypoint[point].speed = blk_ofp_ccc_148.speed / 3.6;
			}
			//标志清空
			ts_flag = 0;
			time_out = 0;
		}
		else if(s4D_frame_0c[index] == -1)
		{
			//遥调失败
			send_blk_ccc_ofp_120(5);
			//标志清空
			ts_flag = 0;
			time_out = 0;
		}
		//超时
		else if(time_out > 200)
		{
			//遥调超时
			send_blk_ccc_ofp_120(6);
			//标志清空
			ts_flag = 0;
			time_out = 0;
		}
	}
}
// 无人机遥控帧// uav_index无人机序号（0-3）
void init_blk_ccc_kkl_008_026_027_028(int uav_index)
{
	init_blk_ccc_kkl_008_026_027_028_front(uav_index);
	init_blk_ccc_kkl_008_026_027_028_tail(uav_index);
}

// 无人机遥控帧// uav_index无人机序号（0-3）
void init_blk_ccc_kkl_008_026_027_028_front(int uav_index)
{
	/***基本遥控指令******/
	static unsigned char frame_count[UAV_MAX_NUM] = {0,0,0,0};

	//下面的赋值需要参考具体协议
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.sync_code[0] = 0xEB;
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.sync_code[1] = 0x90;
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.frame_type = 0xFC;
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.frame_count = frame_count[uav_index];

	//无人机id填写三遍
	unsigned short plane_adr = CCC_DPU_data_3.drone_specific_informations[uav_index].platform_num;
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.plane_adr[0] = plane_adr;
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.plane_adr[1] = plane_adr;
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.plane_adr[2] = plane_adr;

	//本站id填写
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.station_adr[0] = MANNED_ID;//0x3001 MANNED_ID
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.station_adr[1] = MANNED_ID;
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.station_adr[2] = MANNED_ID;

	//遥调校验和
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.yt_check = checkSum(blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_code, 29);

	//总体校验和
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.check_sum =
			do_crc_table((unsigned char*)blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.sync_code+2, sizeof(blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data)-4);


	/****  飞行航线威胁指令帧 32字节*********/
	if(uav_index == send_index)//控制权在有人机才拷贝航线数据
	{
		memcpy(&(blk_ccc_kkl_008_026_027_028[uav_index].front.track_threat_frame_data),&track_threat_frames,sizeof(track_threat_frame)); //遥控指令帧中航线与威胁指令帧
		//清空数据帧
		memset(&track_threat_frames,0,sizeof(track_threat_frame));
	}
	// 帧头赋值
	blk_ccc_kkl_008_026_027_028[uav_index].front.track_threat_frame_data.tongbu_code = 0xaa55; // 同步码（因为大小端，所以写0xaa55）
	blk_ccc_kkl_008_026_027_028[uav_index].front.track_threat_frame_data.frame_count = frame_count[uav_index]; // 帧计数
	blk_ccc_kkl_008_026_027_028[uav_index].front.track_threat_frame_data.check_code = do_crc_table((unsigned char*)&blk_ccc_kkl_008_026_027_028[uav_index].front.track_threat_frame_data.frame_count,sizeof(track_threat_frame)-4);
	//    printf("ct:%d,order:0x%x,lId:%d,pId:%d.\n",frame_count[uav_index], blk_ccc_kkl_008_026_027_028[uav_index].front.track_threat_frame_data.order_code[0],\
	//    		blk_ccc_kkl_008_026_027_028[uav_index].front.track_threat_frame_data.order_data[0], blk_ccc_kkl_008_026_027_028[uav_index].front.track_threat_frame_data.order_data[1]);


	/****  任务控制指令帧 64字节*********/
	if(frame_count[uav_index]%2 == 0)
	{
		memcpy(&(blk_ccc_kkl_008_026_027_028[uav_index].front.task_control_frame_data),&CCC_UAV_Azhens[uav_index],sizeof(task_control_frame));
		blk_ccc_kkl_008_026_027_028[uav_index].front.task_control_frame_data.data[2] = 0xA0;
		if(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling[0] != 0)
		{
			printf("data = 0x%x\n",CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling[0]);
		}
		memset(&CCC_UAV_Azhens[uav_index],0,sizeof(renwuyaokongshuju_Azhen));// 清理遥控指令帧 保证没有干扰数据
	}
	else
	{
		memcpy(&(blk_ccc_kkl_008_026_027_028[uav_index].front.task_control_frame_data),&CCC_UAV_Bzhens[uav_index],sizeof(task_control_frame));
		blk_ccc_kkl_008_026_027_028[uav_index].front.task_control_frame_data.data[2] = 0xB0;
		memset(&CCC_UAV_Bzhens[uav_index],0,sizeof(renwuyaokongshuju_Bzhen));// 清理遥控指令帧 保证没有干扰数据
	}
	// 帧头赋值
	blk_ccc_kkl_008_026_027_028[uav_index].front.task_control_frame_data.data[0] = 0xEB; // 同步码
	blk_ccc_kkl_008_026_027_028[uav_index].front.task_control_frame_data.data[1] = 0x91; // 同步码
	blk_ccc_kkl_008_026_027_028[uav_index].front.task_control_frame_data.data[3] = frame_count[uav_index]; // 帧计数
	unsigned short check_code = do_crc_table((unsigned char*)&blk_ccc_kkl_008_026_027_028[uav_index].front.task_control_frame_data+2, sizeof(task_control_frame)-4);
	memcpy(&blk_ccc_kkl_008_026_027_028[uav_index].front.task_control_frame_data.data[62], &check_code, 2);
	frame_count[uav_index]++;
}

void init_blk_ccc_kkl_008_026_027_028_tail(int uav_index)
{
	/***基本遥控指令******/
	static unsigned char frame_count[UAV_MAX_NUM] = {0,0,0,0};

	//下面的赋值需要参考具体协议
	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.sync_code[0] = 0xAA;
	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.sync_code[1] = 0x55;
	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.frame_type = 0xFC;
	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.frame_count = frame_count[uav_index];

	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.plane_adress = (uav_index == 0) ? (UAV1_ID & 0xff) : (UAV2_ID & 0xff);
	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.station_adress = MANNED_ID >> 8;

	// 更新有人机位置
	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_start = 'Z';
	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_code[0] = 0x00;// 有人机飞机状态空指令
	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_code[1] = 0x00;// 有人机飞机状态空指令
	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_code[2] = 0x00;// 有人机飞机状态空指令
	// 遥调指令
	unsigned int ytOffset = 0; // 相对遥调指令数据的偏移数
	unsigned char* yt_order_data = blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_data;
#if 1
	// 经度
	int lonTem = (int)(DPU_CCC_data_4.manned_longitude_and_latitude_synt.longitude * lon_scale2);
	memcpy(yt_order_data + ytOffset, &lonTem, sizeof(lonTem));
	ytOffset += sizeof(lonTem);
	// 纬度
	int latTem = (int)(DPU_CCC_data_4.manned_longitude_and_latitude_synt.latitude * lat_scale2);
	memcpy(yt_order_data + ytOffset, &latTem, sizeof(latTem));
	ytOffset += sizeof(latTem);
	// 卫星高度
	int weixing_heigth = DPU_CCC_data_4.WX_HEIGHT * 100;
	//test 20251115
//	int weixing_heigth = 1000 * 100;

	memcpy(yt_order_data + ytOffset, &weixing_heigth, sizeof(weixing_heigth));
	ytOffset += sizeof(weixing_heigth);
	// 指示空速（是不是真空速）
	unsigned short vacuum_speed = 0;
	memcpy(yt_order_data + ytOffset, &vacuum_speed, sizeof(vacuum_speed));
	ytOffset += sizeof(vacuum_speed);
	// 航向角（有人机真航向？）
	short true_direction = DPU_CCC_data_4.true_track_angle * hxjiao_scale;
	memcpy(yt_order_data + ytOffset, &true_direction, sizeof(true_direction));
	ytOffset += sizeof(true_direction);
	//北向地速（有人机 北向地速度）
	short beixiang_groundspeed = DPU_CCC_data_4.SPEED_NORTH * speed_scale_b;
	memcpy(yt_order_data + ytOffset, &beixiang_groundspeed, sizeof(beixiang_groundspeed));
	ytOffset += sizeof(beixiang_groundspeed);
	// 东向地速？
	short dongxiang_groundspeed = DPU_CCC_data_4.SPEED_EAST * speed_scale_b;
	memcpy(yt_order_data + ytOffset, &dongxiang_groundspeed, sizeof(dongxiang_groundspeed));
	ytOffset += sizeof(dongxiang_groundspeed);
	// 天向地速？
	short tianxiang_groundspeed = DPU_CCC_data_4.SPEED_UP * speed_scale_b;
	memcpy(yt_order_data + ytOffset, &tianxiang_groundspeed, sizeof(tianxiang_groundspeed));
	ytOffset += sizeof(tianxiang_groundspeed);
	// 无高
	short wugao = 0;
	memcpy(yt_order_data + ytOffset, &wugao, sizeof(wugao));
	ytOffset += sizeof(wugao);
	// 气压高
	short qiyagao = 0;
	memcpy(yt_order_data + ytOffset, &qiyagao, sizeof(qiyagao));
	ytOffset += sizeof(qiyagao);
	// 指令航向
	short zhilinghx = 0;
	memcpy(yt_order_data + ytOffset, &zhilinghx, sizeof(zhilinghx));
	ytOffset += sizeof(qiyagao);
	// 航迹偏航角
	short hjphj = DPU_CCC_data_4.true_direction * 100;
	memcpy(yt_order_data + ytOffset, &hjphj, sizeof(hjphj));
	ytOffset += sizeof(hjphj);
	// 偏航角速率
	short phjsd = DPU_CCC_data_4.HELI_HEADING_RATE * phjsd_scale;
	memcpy(yt_order_data + ytOffset, &phjsd, sizeof(phjsd));
	ytOffset += sizeof(phjsd);
	// 纵向速度指令
	short zxsd_order = 0;
	memcpy(yt_order_data + ytOffset, &zxsd_order, sizeof(zxsd_order));
	ytOffset += sizeof(zxsd_order);
	// 侧向速度指令
	short cxsd_order = 0;
	memcpy(yt_order_data + ytOffset, &cxsd_order, sizeof(cxsd_order));
	ytOffset += sizeof(cxsd_order);
	// 前向位置指令
	short qxwz_order = 0;
	memcpy(yt_order_data + ytOffset, &qxwz_order, sizeof(qxwz_order));
	ytOffset += sizeof(qxwz_order);
	// 测向位置指令
	short cxwz_order = 0;
	memcpy(yt_order_data + ytOffset, &cxwz_order, sizeof(cxwz_order));
	ytOffset += sizeof(cxwz_order);
	// 横滚角指令
	short hgj_order = 0;
	memcpy(yt_order_data + ytOffset, &hgj_order, sizeof(hgj_order));
	ytOffset += sizeof(hgj_order);
	// 高度指令
	short gd_order = 0;
	memcpy(yt_order_data + ytOffset, &gd_order, sizeof(gd_order));
	ytOffset += sizeof(gd_order);
	// 垂速指令
	short cs_order = 0;
	memcpy(yt_order_data + ytOffset, &cs_order, sizeof(cs_order));
	ytOffset += sizeof(cs_order);
	// 滚转角指令
	short gzj_order = DPU_CCC_data_4.roll_angle * hxjiao_scale;
	memcpy(yt_order_data + ytOffset, &gzj_order, sizeof(gzj_order));
	ytOffset += sizeof(gzj_order);
	// 俯仰角指令
	short fyj_order = DPU_CCC_data_4.tilt * hxjiao_scale;
	memcpy(yt_order_data + ytOffset, &fyj_order, sizeof(fyj_order));
	ytOffset += sizeof(fyj_order);
	// GPS时间年
	unsigned char gps_year = 0;
	memcpy(yt_order_data + ytOffset, &gps_year, sizeof(gps_year));
	ytOffset += sizeof(gps_year);
	// GPS时间月
	unsigned char gps_month = 0;
	memcpy(yt_order_data + ytOffset, &gps_month, sizeof(gps_month));
	ytOffset += sizeof(gps_month);
	// GPS时间日
	unsigned char gps_day = 0;
	memcpy(yt_order_data + ytOffset, &gps_day, sizeof(gps_day));
	ytOffset += sizeof(gps_day);
	// GPS时间时
	unsigned char gps_hour = 0;
	memcpy(yt_order_data + ytOffset, &gps_hour, sizeof(gps_hour));
	ytOffset += sizeof(gps_hour);
	// GPS时间分
	unsigned char gps_min = 0;
	memcpy(yt_order_data + ytOffset, &gps_min, sizeof(gps_min));
	ytOffset += sizeof(gps_min);
	// GPS时间秒
	unsigned char gps_sec = 0;
	memcpy(yt_order_data + ytOffset, &gps_sec, sizeof(gps_sec));
	ytOffset += sizeof(gps_sec);
	// GPS时间毫秒
	unsigned short gps_ms = 0;
	memcpy(yt_order_data + ytOffset, &gps_ms, sizeof(gps_ms));
	ytOffset += sizeof(gps_ms);
#else
	// 经度
	int lonTem = (180) * lon_scale2;
	memcpy(yt_order_data + ytOffset, &lonTem, sizeof(lonTem));
	ytOffset += sizeof(lonTem);
	// 纬度
	int latTem = (-90) * lat_scale2;
	memcpy(yt_order_data + ytOffset, &latTem, sizeof(latTem));
	ytOffset += sizeof(latTem);
	// 卫星高度
	int weixing_heigth = (6550) * 100;
	memcpy(yt_order_data + ytOffset, &weixing_heigth, sizeof(weixing_heigth));
	ytOffset += sizeof(weixing_heigth);
	// 指示空速（是不是真空速）
	unsigned short vacuum_speed = 0;
	memcpy(yt_order_data + ytOffset, &vacuum_speed, sizeof(vacuum_speed));
	ytOffset += sizeof(vacuum_speed);
	// 航向角（有人机真航向？）
	short true_direction = (180) * hxjiao_scale;
	memcpy(yt_order_data + ytOffset, &true_direction, sizeof(true_direction));
	ytOffset += sizeof(true_direction);
	//北向地速（有人机 北向地速度）
	short beixiang_groundspeed = (400) * speed_scale_b;
	memcpy(yt_order_data + ytOffset, &beixiang_groundspeed, sizeof(beixiang_groundspeed));
	ytOffset += sizeof(beixiang_groundspeed);
	// 东向地速？
	short dongxiang_groundspeed = (400) * speed_scale_b;
	memcpy(yt_order_data + ytOffset, &dongxiang_groundspeed, sizeof(dongxiang_groundspeed));
	ytOffset += sizeof(dongxiang_groundspeed);
	// 天向地速？
	short tianxiang_groundspeed = (-400) * speed_scale_b;
	memcpy(yt_order_data + ytOffset, &tianxiang_groundspeed, sizeof(tianxiang_groundspeed));
	ytOffset += sizeof(tianxiang_groundspeed);
	// 无高
	short wugao = 0;
	memcpy(yt_order_data + ytOffset, &wugao, sizeof(wugao));
	ytOffset += sizeof(wugao);
	// 气压高
	short qiyagao = 0;
	memcpy(yt_order_data + ytOffset, &qiyagao, sizeof(qiyagao));
	ytOffset += sizeof(qiyagao);
	// 指令航向
	short zhilinghx = 0;
	memcpy(yt_order_data + ytOffset, &zhilinghx, sizeof(zhilinghx));
	ytOffset += sizeof(qiyagao);
	// 航迹偏航角
	short hjphj = (-180) * 100;
	memcpy(yt_order_data + ytOffset, &hjphj, sizeof(hjphj));
	ytOffset += sizeof(hjphj);
	// 偏航角速率
	short phjsd = (300) * phjsd_scale;
	memcpy(yt_order_data + ytOffset, &phjsd, sizeof(phjsd));
	ytOffset += sizeof(phjsd);
	// 纵向速度指令
	short zxsd_order = 0;
	memcpy(yt_order_data + ytOffset, &zxsd_order, sizeof(zxsd_order));
	ytOffset += sizeof(zxsd_order);
	// 侧向速度指令
	short cxsd_order = 0;
	memcpy(yt_order_data + ytOffset, &cxsd_order, sizeof(cxsd_order));
	ytOffset += sizeof(cxsd_order);
	// 前向位置指令
	short qxwz_order = 0;
	memcpy(yt_order_data + ytOffset, &qxwz_order, sizeof(qxwz_order));
	ytOffset += sizeof(qxwz_order);
	// 测向位置指令
	short cxwz_order = 0;
	memcpy(yt_order_data + ytOffset, &cxwz_order, sizeof(cxwz_order));
	ytOffset += sizeof(cxwz_order);
	// 横滚角指令
	short hgj_order = 0;
	memcpy(yt_order_data + ytOffset, &hgj_order, sizeof(hgj_order));
	ytOffset += sizeof(hgj_order);
	// 高度指令
	short gd_order = 0;
	memcpy(yt_order_data + ytOffset, &gd_order, sizeof(gd_order));
	ytOffset += sizeof(gd_order);
	// 垂速指令
	short cs_order = 0;
	memcpy(yt_order_data + ytOffset, &cs_order, sizeof(cs_order));
	ytOffset += sizeof(cs_order);
	// 滚转角
	short gzj_order = (180) * hxjiao_scale;
	memcpy(yt_order_data + ytOffset, &gzj_order, sizeof(gzj_order));
	ytOffset += sizeof(gzj_order);
	// 俯仰角
	short fyj_order = (-180) * hxjiao_scale;
	memcpy(yt_order_data + ytOffset, &fyj_order, sizeof(fyj_order));
	ytOffset += sizeof(fyj_order);
	// GPS时间年
	unsigned char gps_year = 0;
	memcpy(yt_order_data + ytOffset, &gps_year, sizeof(gps_year));
	ytOffset += sizeof(gps_year);
	// GPS时间月
	unsigned char gps_month = 0;
	memcpy(yt_order_data + ytOffset, &gps_month, sizeof(gps_month));
	ytOffset += sizeof(gps_month);
	// GPS时间日
	unsigned char gps_day = 0;
	memcpy(yt_order_data + ytOffset, &gps_day, sizeof(gps_day));
	ytOffset += sizeof(gps_day);
	// GPS时间时
	unsigned char gps_hour = 0;
	memcpy(yt_order_data + ytOffset, &gps_hour, sizeof(gps_hour));
	ytOffset += sizeof(gps_hour);
	// GPS时间分
	unsigned char gps_min = 0;
	memcpy(yt_order_data + ytOffset, &gps_min, sizeof(gps_min));
	ytOffset += sizeof(gps_min);
	// GPS时间秒
	unsigned char gps_sec = 0;
	memcpy(yt_order_data + ytOffset, &gps_sec, sizeof(gps_sec));
	ytOffset += sizeof(gps_sec);
	// GPS时间毫秒
	unsigned short gps_ms = 0;
	memcpy(yt_order_data + ytOffset, &gps_ms, sizeof(gps_ms));
	ytOffset += sizeof(gps_ms);
#endif

//领航
	if(start_lh_flag[uav_index] == 1)
	{
		static int lh_send_cnt[UAV_MAX_NUM] = {0,0,0,0};
		blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_code[0] = 0x16;
		blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_code[1] = 0x16;
		blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_code[2] = 0x16;
		tail_0x16 temp;
		memset(&temp,0,sizeof(tail_0x16));
		temp.high_cmd = (500.0 / (6000.0/ 65535));
		temp.hori_distance = (6000.0 /(10000.0/ 65535));
		memcpy(&blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_data,&temp,sizeof(temp));

		//发送5拍
		lh_send_cnt[uav_index] ++;
		if(lh_send_cnt[uav_index] > 5)
		{
			lh_send_cnt[uav_index] = 0;
			start_lh_flag[uav_index] = 2;
		}
	}
	if(start_tclh_flag[uav_index] == 1)
	{
		static int tclh_send_cnt[UAV_MAX_NUM] = {0,0,0,0};
		blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_code[0] = 0x14;
		blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_code[1] = 0x14;
		blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_code[2] = 0x14;

//		if(uav_route[uav_index].route_number == 48 || uav_route[uav_index].route_number == 49)
//		{
//			blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_data[0] = uav_route[uav_index].route_number;
//			blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_data[1] = 5;
//		}
//		else
//		{
//			blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_data[0] = uav_route[uav_index].route_number;
//			blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_data[1] = dissolve_point;
//		}
		blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_data[0] = uav_route[uav_index].route_number;
		//20260205退出领航去第七个点改第六个点
		unsigned char exit_point = dissolve_point;
		if((uav_route[uav_index].route_number == 48 || uav_route[uav_index].route_number == 49) && exit_point == 7)
		{
			exit_point = 6;
		}
		blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_data[1] = exit_point;

		//发送5拍
		tclh_send_cnt[uav_index] ++;
		if(tclh_send_cnt[uav_index] > 5)
		{
			tclh_send_cnt[uav_index] = 0;
			start_tclh_flag[uav_index] = 2;
		}
	}

	//遥调校验和
	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_check_sum = checkSum(blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.yt_order_code, 86);

	// 总体校验和
	blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.check_crc =
		do_crc_table((unsigned char *)blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.sync_code + 2, sizeof(blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl) - 4);

	/****  数据库管理指令帧*********/
	blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.sync_code[0] = 0x7E;
	blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.sync_code[1] = 0x7E;
	blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.frame_count = frame_count[uav_index];
	blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.check_crc = do_crc_table((unsigned char *)blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase.sync_code + 2, sizeof(blk_ccc_kkl_008_026_027_028[uav_index].tail.dataBase) - 4);

	frame_count[uav_index]++;
}

void send_blk_ccc_kkl_008_026_027_028()
{
	static int tem1 = 1;
	int temnum = 0;
	if (tem1 == 0)
	{
		temnum = 1;
	}
	else
	{
		temnum = 2;
	}

	//    for(int uav_index = 0; uav_index < UAV_MAX_NUM; uav_index++)
	for (int uav_index = 0; uav_index < temnum; uav_index++)
	{
		if (CCC_DPU_data_3.drone_specific_informations[uav_index].platform_num == 0) // 未收到遥测就不发遥控 20251018new
		{
			continue;
		}
		init_blk_ccc_kkl_008_026_027_028(uav_index);
		// 计算发出开始领航之后的节拍
		if (start_lh_flag[uav_index] == 2)
		{
			lhtime[uav_index]++;
		}
		if (start_tclh_flag[uav_index] == 2)
		{
			tclhtime[uav_index]++;
		}

		// C链只发送前160字节
		// 如果u端机本机链路状态数据中的该飞机的u链上行传输速率是51.2kbps，那么就发320字节完整包。如果不是，则只发前160字节
		// 前160

		align_send_information(&(blk_ccc_kkl_008_026_027_028[uav_index]), sizeof(BLK_CCC_KKL_008_026_027_028_Front), 0);
		// 后160字节

		memcpy(send_array.dataA + 160, &blk_ccc_kkl_008_026_027_028[uav_index].tail, 160);

		SINT32_SIX messageId;
		switch (uav_index)
		{
		case 0:
			messageId = DDSTables.CCC_KKL_0.niConnectionId;
			break;
		case 1:
			messageId = DDSTables.CCC_KKL_5.niConnectionId;
			break;
		case 2:
			messageId = DDSTables.CCC_KKL_6.niConnectionId;
			break;
		case 3:
			messageId = DDSTables.CCC_KKL_7.niConnectionId;
			break;
		default:
			break;
		}

#if !SIMTESTFLAG // 半物理环境有这个逻辑，仿真先不加（如有需要再放开）
				 // if(formationId[uav_index].isControl == 1)// 有控权才发送
#endif
		{
			Send_Message_Local(messageId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
			if (enRetCode != 0)
			{
				printf("send yaokong error.\n");
			}
		}

		// U链发送根据条件判断
		// 如果u端机本机链路状态数据中的该飞机的u链上行传输速率大于等于51.2kbps，那么就发320字节完整包。如果不是，则只发前160字节
		if (blk_ccc_ofp_199.ULparSet_1[0].UpRate_3 >= 2) // 0927转给飞控测试注释
		{
			// 320
			align_send_information(&(blk_ccc_kkl_008_026_027_028[uav_index]), sizeof(BLK_CCC_KKL_008_026_027_028), 0);
		}
		else
		{
			// 前160
			align_send_information(&(blk_ccc_kkl_008_026_027_028[uav_index]), sizeof(BLK_CCC_KKL_008_026_027_028_Front), 0);
		}
		SINT32_SIX messageId_u;
		switch (uav_index)
		{
		case 0:
			messageId_u = DDSTables.CCC_KKL_12.niConnectionId;
			break;
		case 1:
			messageId_u = DDSTables.CCC_KKL_13.niConnectionId;
			break;
		case 2:
			messageId_u = DDSTables.CCC_KKL_14.niConnectionId;
			break;
		case 3:
			messageId_u = DDSTables.CCC_KKL_15.niConnectionId;
			break;
		default:
			break;
		}

#if !SIMTESTFLAG // 半物理环境有这个逻辑，仿真先不加（如有需要再放开）
				 //        if(formationId[uav_index].isControl == 1)// 有控权才发送
#endif
		{
			// 发送U链
			Send_Message_Local(messageId_u, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
			if (enRetCode != 0)
			{
				printf("send yaokong ulink error.\n");
			}
		}

		// 重置
		memset(&blk_ccc_kkl_008_026_027_028[uav_index], 0, sizeof(BLK_CCC_KKL_008_026_027_028));
	}
}

void set_blk_ccc_kkl_000(int uav_index, unsigned int *p_main_link_send)
{
	static int main_link_send_count = 3; // 发送主链交接指令次数
	// 发送交接参数
	if (*p_main_link_send > 0)
	{
		// 类型固定04
		blk_ccc_kkl_000.Type = 0x04;

		/******************************************************
		 *1-10;设置无人机配置参数
		 *11-20：设置下行速率
		 *21-30：设置天线模式
		 * 31-40：发送主链转移指令
		 * /*******************************************/

		switch (*p_main_link_send)
		{
		case 1:
			blk_ccc_kkl_000.UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_000.LianXuLiang = 0x03; // 被控无人机id
			blk_ccc_kkl_000.LianXuLiangH = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID >> 8;
			blk_ccc_kkl_000.LianXuLiangL = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID & 0x00ff;
			break;
		case 2:
			blk_ccc_kkl_000.UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_000.LianXuLiang = 0x09; // 09H 主控站－ＩＤ设置　控制站ID号
			blk_ccc_kkl_000.LianXuLiangH = MANNED_ID >> 8;
			blk_ccc_kkl_000.LianXuLiangL = MANNED_ID & 0x00ff;
			break;
		case 3:
			blk_ccc_kkl_000.UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_000.LianXuLiang = 0x0A; // 0AH 接机站-ID设置 控制站ID号
			blk_ccc_kkl_000.LianXuLiangH = blk_ofp_ccc_014.CommunicationParam.rcv_platform_code >> 8;
			blk_ccc_kkl_000.LianXuLiangL = blk_ofp_ccc_014.CommunicationParam.rcv_platform_code & 0x00ff;
			break;
		case 4:
			blk_ccc_kkl_000.UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_000.LianXuLiang = 0x35; // 35H C下行交接频道 同上
			blk_ccc_kkl_000.LianXuLiangH = blk_ofp_ccc_014.CommunicationParam.handover_down_channel >> 8;
			blk_ccc_kkl_000.LianXuLiangL = blk_ofp_ccc_014.CommunicationParam.handover_down_channel & 0x00ff;
			break;
		case 5:
			blk_ccc_kkl_000.UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_000.LianXuLiang = 0x39; // 39H C上行交接频道 同上
			blk_ccc_kkl_000.LianXuLiangH = blk_ofp_ccc_014.CommunicationParam.handover_up_channel >> 8;
			blk_ccc_kkl_000.LianXuLiangL = blk_ofp_ccc_014.CommunicationParam.handover_up_channel & 0x00ff;
			break;
		case 6:
			// 每个操作完成，阶段性退出发送
			*p_main_link_send = 0;
			break;

			/************11-20：设置下行速率************************/
		case 11:
			blk_ccc_kkl_000.UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_000.LianXuLiang = 0x03; // 被控无人机id
			blk_ccc_kkl_000.LianXuLiangH = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID >> 8;
			blk_ccc_kkl_000.LianXuLiangL = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID & 0x00ff;
			break;
		case 12:
			blk_ccc_kkl_000.UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_000.LianXuLiang = 0x09; // 09H 主控站－ＩＤ设置　控制站ID号
			blk_ccc_kkl_000.LianXuLiangH = MANNED_ID >> 8;
			blk_ccc_kkl_000.LianXuLiangL = MANNED_ID & 0x00ff;
			break;
		case 13:
			blk_ccc_kkl_000.UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_000.LianXuLiang = 0x3A; // 3AH C下行速率 H字节D7-D4：时隙分配 L字节D3-D0：速率控制
			blk_ccc_kkl_000.LianXuLiangH = blk_ofp_ccc_014.CommunicationParam.down_rate >> 8;
			blk_ccc_kkl_000.LianXuLiangL = blk_ofp_ccc_014.CommunicationParam.down_rate & 0x00ff;
			break;
		case 14:
			blk_ccc_kkl_000.UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_000.LianXuLiang = 0x6A; // 6AH UHF接机站频道 同上
			blk_ccc_kkl_000.LianXuLiangH = blk_ofp_ccc_014.CommunicationParam.u_channel >> 8;
			blk_ccc_kkl_000.LianXuLiangL = blk_ofp_ccc_014.CommunicationParam.u_channel & 0x00ff;
			// 每个操作完成，阶段性退出发送
			*p_main_link_send = 0;
			break;

			/************21-30：设置天线模式************************/
		case 21:
			blk_ccc_kkl_000.UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID; // 被控无人机id
			blk_ccc_kkl_000.KaiGuanLiang = 0x33;											 // 全向天线
			break;
		case 22:
			// 每个操作完成，阶段性退出发送
			*p_main_link_send = 0;
			break;

			/************31-40：发送主链转移指令************************/
		case 31:
			blk_ccc_kkl_000.UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_000.KaiGuanLiang = 0x0C; // 0CH 启动主链转移
			break;
		case 32:
			// 每个操作完成，阶段性退出发送
			*p_main_link_send = 0;
			break;

		case 55:
			blk_ccc_kkl_000.KaiGuanLiang = 0x0A; // 0AH C链交接成功确认
			break;
		default:
			*p_main_link_send = 0; // 退出发送
			break;
		}
		if (*p_main_link_send == 0)
		{
			return;
		}

		send_blk_ccc_kkl_000();

		if (--main_link_send_count <= 0) // 发完三次
		{
			(*p_main_link_send)++;	  // 发送下一个数据
			main_link_send_count = 3; // 发送数重置
		}
	}
}

void init_blk_ccc_kkl_000()
{
	static unsigned char frameCount = 0;
	// 赋值
	blk_ccc_kkl_000.TongBuZi1 = 0xEB;
	blk_ccc_kkl_000.TongBuZi2 = 0x90;
	blk_ccc_kkl_000.ZhenChang = 0x10;
	blk_ccc_kkl_000.Num = frameCount++;
	blk_ccc_kkl_000.BenJiID = MANNED_ID;
	blk_ccc_kkl_000.CRC16 = do_crc_table(&blk_ccc_kkl_000.Type, 10); // CRC16 域描述：校验4-13共10个字节
	blk_ccc_kkl_000.HeJiaoYan = checkSum((unsigned char *)&blk_ccc_kkl_000, sizeof(BLK_CCC_KKL_009_029_030_031) - 1);
}

void send_blk_ccc_kkl_000()
{
	// uav_index无人机序号（0-3）
	init_blk_ccc_kkl_000();
	// 主链交接成功，0xa20e00
	data_length = sizeof(BLK_CCC_KKL_000);

	Send_Message(DDSTables.CCC_KKL_000.niConnectionId, 0, &transaction_id, &blk_ccc_kkl_000, &message_type_id, data_length, &enRetCode);
	memset(&blk_ccc_kkl_000, 0, sizeof(BLK_CCC_KKL_000));
}

// uav_index无人机序号（0-3）
void init_blk_ccc_kkl_009_029_030_031(int uav_index)
{
	static unsigned char frameCount = 0;
	// 赋值
	blk_ccc_kkl_009_029_030_031[uav_index].TongBuZi1 = 0xEB;
	blk_ccc_kkl_009_029_030_031[uav_index].TongBuZi2 = 0x90;
	blk_ccc_kkl_009_029_030_031[uav_index].ZhenChang = 0x10;
	blk_ccc_kkl_009_029_030_031[uav_index].Num = frameCount++;
	blk_ccc_kkl_009_029_030_031[uav_index].BenJiID = MANNED_ID;
	blk_ccc_kkl_009_029_030_031[uav_index].CRC16 = do_crc_table((unsigned char *)&blk_ccc_kkl_009_029_030_031[uav_index] + 3, 10); // CRC16 域描述：校验4-13共10个字节
	blk_ccc_kkl_009_029_030_031[uav_index].HeJiaoYan = checkSum((unsigned char *)&blk_ccc_kkl_009_029_030_031[uav_index], sizeof(BLK_CCC_KKL_009_029_030_031) - 1);
}

void send_blk_ccc_kkl_009_029_030_031(int uav_index)
{
	// uav_index无人机序号（0-3）
	init_blk_ccc_kkl_009_029_030_031(uav_index);
	// 主链交接成功，0xa20e09
	data_length = sizeof(BLK_CCC_KKL_009_029_030_031);
	SINT32_SIX messageId = DDSTables.CCC_KKL_1.niConnectionId;
	switch (uav_index)
	{
	case 0:
		messageId = DDSTables.CCC_KKL_1.niConnectionId;
		break;
	case 1:
		messageId = DDSTables.CCC_KKL_8.niConnectionId;
		break;
	case 2:
		messageId = DDSTables.CCC_KKL_9.niConnectionId;
		break;
	case 3:
		messageId = DDSTables.CCC_KKL_10.niConnectionId;
		break;
	default:
		break;
	}
	Send_Message(messageId, 0, &transaction_id, &blk_ccc_kkl_009_029_030_031[uav_index], &message_type_id, data_length, &enRetCode);
	memset(&blk_ccc_kkl_009_029_030_031[uav_index], 0, sizeof(BLK_CCC_KKL_009_029_030_031));
}

// 接收 协同通信信息  空空链.协同指控计算机
void recv_kkl_ccc()
{
	message_size = 2048;
	// u链状态信息
	Receive_Message(DDSTables.KKL_CCC_3.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(KKL_CCC_data_3.KKLCNumber_2), dds_data_rece.dataA, sizeof(KKL_CCC_data_3));
	}

	// 空空链参数反馈信息

	message_size = 2048;
	Receive_Message(DDSTables.KKL_CCC_4.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(KKL_CCC_data_4.KKLCNumber_1_1), dds_data_rece.dataA, sizeof(KKL_CCC_data_4));
	}
}

// 分包发送通用航路点
// 发综显
void send_blk_ccc_ofp_018(int k, unsigned int plan, unsigned int task)
{
	// 任务类型为浮标或者吊声就不发
	if (CCC_DPU_data_6[plan].formation_synergy_mission_programs[0].task_sequence_informations[task].type == 2 ||
		CCC_DPU_data_6[plan].formation_synergy_mission_programs[0].task_sequence_informations[task].type == 3)
	{
		return;
	}
	// 如果该任务航路点不超过四十个就发一包
	if (blk_ccc_ofp_018_cunchu[plan][task].airway_point_start_num == 0)
	{
		// send_array.resize(2400);
		memcpy(send_array.dataA, &blk_ccc_ofp_018_cunchu[plan][task], 21);

		// 加入40个航迹点
		align_send_information(&(blk_ccc_ofp_018_cunchu[plan][task].waypoint_informations[0]), 40 * sizeof(waypoint_information), 21);
		data_length = sizeof(BLK_CCC_OFP_018);
		// 发前40个航路点信息
		//  综显发送
		if (k == DDSTables.CCC_DPU_7.niConnectionId)
		{
			// 发送给 dpu 不需要头
			Send_Message(k, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
		}
		else
		{
			// 发送给 pad 需要头
			Send_Message(k, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
		}
		// 给kdl发送
		send_blk_ccc_kdl_018();
		// send_array.clear();//清空
	}
	// 如果该任务航路点超过四十个就发两包
	else if (blk_ccc_ofp_018_cunchu[plan][task].airway_point_start_num == 40)
	{
		// 保存后40个航路点的有效数
		unsigned short point_num = blk_ccc_ofp_018_cunchu[plan][task].airway_point_num;

		/*********************第一次发送****************************/
		// send_array.resize(2400);
		blk_ccc_ofp_018_cunchu[plan][task].airway_point_start_num = 0;
		blk_ccc_ofp_018_cunchu[plan][task].airway_point_num = 40;
		memcpy(send_array.dataA, &blk_ccc_ofp_018_cunchu[plan][task], 21);

		// 加入40个航迹点
		align_send_information(&(blk_ccc_ofp_018_cunchu[plan][task].waypoint_informations[0]), 40 * sizeof(waypoint_information), 21);
		data_length = sizeof(BLK_CCC_OFP_018);
		// 发前40个航路点信息
		//  综显发送
		if (k == DDSTables.CCC_DPU_7.niConnectionId)
		{
			// 发送给 dpu 不需要头
			Send_Message(k, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
		}
		else
		{
			// 发送给 pad 需要头
			Send_Message(k, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
		}
		// 给kdl发送
		send_blk_ccc_kdl_018();
		// send_array.clear();//清空

		/*********************第二次发送****************************/
		//        send_array.resize(2400);
		blk_ccc_ofp_018_cunchu[plan][task].airway_point_start_num = 40;
		blk_ccc_ofp_018_cunchu[plan][task].airway_point_num = point_num;
		memcpy(send_array.dataA, &blk_ccc_ofp_018_cunchu[plan][task], 21);

		// 加入后40个航迹点
		align_send_information(&(blk_ccc_ofp_018_cunchu[plan][task].waypoint_informations[40]), 40 * sizeof(waypoint_information), 21);
		data_length = sizeof(BLK_CCC_OFP_018);
		// 发后40个航路点信息
		//  综显发送
		if (k == DDSTables.CCC_DPU_7.niConnectionId)
		{
			// 发送给 dpu 不需要头
			Send_Message(k, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
		}
		else
		{
			// 发送给 pad 需要头
			Send_Message(k, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
		}
		// 给kdl发送
		send_blk_ccc_kdl_018();
		// send_array.clear();//清空
	}
}

/***中间件时间*****/
void recv_mids_time_message()
{
	Receive_Message(DDSTables.MIDS_CCC_001.niConnectionId, 0, &transaction_id, &blk_mids_ccc_time, &message_type_id, &message_size, &enRetCode);
}

/******************************** PAD交互模块 ******************************/
void recv_pad_message()
{ // 模块主函数
	recv_pad_heart();
	recv_pad_ccc_778();
}

// 接收pad在线心跳信号
void recv_pad_heart()
{
	static int timeout = -1;
	message_size = sizeof(PAD_hearts);
	// pad_recv.resize(4096);
	Receive_Message(DDSTables.PAD_CCC_777.niConnectionId, 0, &transaction_id, pad_recv.dataA, &message_type_id, &message_size, &enRetCode);

	if (enRetCode == 0)
	{
		timeout = -1;
		memcpy(&PAD_hearts, pad_recv.dataA, sizeof(PAD_hearts));
		if (PAD_hearts.head == 1)
		{
			front_pad_heart_flag = PAD_hearts.heart;
		}
		else
		{
			tail_pad_heart_flag = PAD_hearts.heart;
		}

		Pad_heart_flag = front_pad_heart_flag || tail_pad_heart_flag;
		//		printf("receive pad heart");
	}

	if (timeout > 20)
	{
		Pad_heart_flag = 0;
		front_pad_heart_flag = 0;
		tail_pad_heart_flag = 0;
	}

	timeout++;
}

// 有人机航线决策信息
void recv_pad_ccc_778()
{
	Receive_Message(DDSTables.PAD_CCC_778.niConnectionId, 0, &transaction_id, &blk_pad_ccc_778, &message_type_id, &message_size, &enRetCode);

	if (enRetCode == 0)
	{
		int plan = blk_pad_ccc_778.planId;
		int task = blk_pad_ccc_778.taskId;
		// 发送浮标布阵等
		// 找到当前方案的保存下标
		if (blk_ccc_ofp_302_save[plan][task].Plan_ID != 0)
		{
			data_length = sizeof(BLK_CCC_OFP_302);
			// 转发给综显浮标布阵规划
			Send_Message(DDSTables.CCC_DPU_056.niConnectionId, 0, &transaction_id, &blk_ccc_ofp_302_save[plan][task], &message_type_id, data_length, &enRetCode);
		}

		if (blk_ccc_ofp_403_save[plan][task].Plan_ID != 0)
		{
			data_length = sizeof(BLK_CCC_OFP_403);
			// 转发给综显吊声定测点规划
			Send_Message(DDSTables.CCC_DPU_057.niConnectionId, 0, &transaction_id, &blk_ccc_ofp_403_save[plan][task], &message_type_id, data_length, &enRetCode);
		}
	}
}

// kdl 发送
// 编队综合态势
void init_blk_ccc_kdl_000()
{
	blk_ccc_kdl_000.synchronization_code[0] = (char)0xeb;
	blk_ccc_kdl_000.synchronization_code[1] = (char)0x94;
	blk_ccc_kdl_000.frame_class = (char)0xb3;

	// TODO 从真实数据取。【待确认】无人机遥测数据(无人机状态信息)、有人机实时位置（0x062a10）
	//     integrated_formation_postures.manned_helicopter_flight_informations.manned_helicopter_latitude = (long long)(24.456 * lat_Scale1);
	//     integrated_formation_postures.manned_helicopter_flight_informations.manned_helicopter_longitude = (long long)(123.456 * lon_Scale1);
	//     integrated_formation_postures.manned_helicopter_flight_informations.manned_helicopter_height =  (int)500 / 0.1;

	blk_ccc_kdl_000.manned_helicopter_flight_informations.manned_helicopter_latitude = (long long)(DPU_CCC_data_4.manned_longitude_and_latitude_synt.latitude * lat_Scale1);
	blk_ccc_kdl_000.manned_helicopter_flight_informations.manned_helicopter_longitude = (long long)(DPU_CCC_data_4.manned_longitude_and_latitude_synt.longitude * lon_Scale1);
	blk_ccc_kdl_000.manned_helicopter_flight_informations.manned_helicopter_height = (int)(DPU_CCC_data_4.absolute_barometric_altitude / (0.1));
	blk_ccc_kdl_000.manned_helicopter_flight_informations.manned_helicopter_speed = (int)(DPU_CCC_data_4.vacuum_speed / (0.1));
	blk_ccc_kdl_000.manned_helicopter_flight_informations.manned_helicopter_direction = (int)(DPU_CCC_data_4.true_direction * ((double)360 / (((long long)2 << 31) - 1)));
	blk_ccc_kdl_000.manned_helicopter_flight_informations.manned_helicopter_tilt = (int)(DPU_CCC_data_4.tilt * lon_scale);
	blk_ccc_kdl_000.manned_helicopter_flight_informations.manned_helicopter_roll_angle = (int)(DPU_CCC_data_4.roll_angle * lat_scale);
	blk_ccc_kdl_000.manned_helicopter_flight_informations.manned_helicopter_valid_flag_bit = 1;

	// TODO CCC_DPU_data_3
	//     integrated_formation_postures.unmanned_helicopter_flight_informations[0].uav_id = 0X1005;
	//     integrated_formation_postures.unmanned_helicopter_flight_informations[0].control_attribution = 2;
	//     integrated_formation_postures.unmanned_helicopter_flight_informations[0].unmanned_helicopter_latitude = (int)(22.521 * lat_scale2);
	//     integrated_formation_postures.unmanned_helicopter_flight_informations[0].unmanned_helicopter_longitude = (int)(124.456 * lon_scale2);
	//     integrated_formation_postures.unmanned_helicopter_flight_informations[1].uav_id = 0X1006;
	//     integrated_formation_postures.unmanned_helicopter_flight_informations[1].control_attribution = 2;
	//     integrated_formation_postures.unmanned_helicopter_flight_informations[1].unmanned_helicopter_latitude = (int)(22.321 * lat_scale2);
	//     integrated_formation_postures.unmanned_helicopter_flight_informations[1].unmanned_helicopter_longitude = (int)(124.654 * lon_scale2);

	// 无人机ID从哪里接收？在　CCC_DPU_data_3　中没有。还是说是固定值？
	// 怀疑 控制权归属 control_attribution 与 平台控制权状态 unsigned short platform_control_status（4A　站地址判断） 有关
	// uav_id数据类型前后不一致（uav_id为ushort,platform_num为uint）

	// 接收遥测时处理
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].uav_id =(unsigned short)CCC_DPU_data_3.drone_specific_informations[0].platform_num;
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].control_attribution = (unsigned char)CCC_DPU_data_3.drone_specific_informations[0].platform_control_status;
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].unmanned_helicopter_latitude = (int)(CCC_DPU_data_3.drone_specific_informations[0].uav_infos.uav_lati * lat_scale2);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].unmanned_helicopter_longitude = (int)(CCC_DPU_data_3.drone_specific_informations[0].uav_infos.uav_longi * lon_scale2);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].barometric_height = (short)(CCC_DPU_data_3.drone_specific_informations[0].uav_infos.air_height/0.2);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].satellite_height = (short)(s3A_frame.satellite_altitude*0.1);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].radio_height = (short)(CCC_DPU_data_3.drone_specific_informations[0].uav_infos.radio_height/0.1);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].absolute_height = (short)(s81_frame.absolute_height*0.2);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].relative_height = (short)(s81_frame.relative_height*0.2);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].vertical_velocity = (char)(CCC_DPU_data_3.drone_specific_informations[0].uav_infos.vectx_speed/0.1);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].vacuum_speed = (short)(CCC_DPU_data_3.drone_specific_informations[0].uav_infos.air_speed/0.1);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].body_axis_longitudinal_speed = (short)(s81_frame.body_grouspeed_x*0.1);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].body_axis_lateral_ground_velocity = (short)(s81_frame.body_grouspeed_y*0.1);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].tilt = (short)(CCC_DPU_data_3.drone_specific_informations[0].uav_infos.uav_pitch/0.01);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].roll_angle = (short)(CCC_DPU_data_3.drone_specific_informations[0].uav_infos.uav_roll/0.01);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].hangxiang_angle = (unsigned short)(CCC_DPU_data_3.drone_specific_informations[0].uav_infos.uav_heading/0.01);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].flight_path = (unsigned short)(s4A_frame.fly_course*0.1);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].current_flight_path_num = (char)(s3A_frame.route_number);
	//	blk_ccc_kdl_000.unmanned_helicopter_flight_informations[0].current_flight_waypoint_num = (char)(s3A_frame.waypoint_number);

	//    integrated_formation_postures.unmanned_helicopter_flight_informations[1].uav_id =CCC_DPU_data_3.drone_specific_informations[1].platform_num;
	//    integrated_formation_postures.unmanned_helicopter_flight_informations[1].control_attribution = CCC_DPU_data_3.drone_specific_informations[1].platform_control_status;
	//    integrated_formation_postures.unmanned_helicopter_flight_informations[1].unmanned_helicopter_latitude = (int)CCC_DPU_data_3.drone_specific_informations[1].uav_infos.uav_lati * lat_scale2;
	//    integrated_formation_postures.unmanned_helicopter_flight_informations[1].unmanned_helicopter_longitude = (int)CCC_DPU_data_3.drone_specific_informations[1].uav_infos.uav_longi * lon_scale2;

	// TODO当前目标信息待确认
	blk_ccc_kdl_000.data_fusions.goal_number = 0;
	blk_ccc_kdl_000.data_fusions.goal_informations.standby = 0;
	blk_ccc_kdl_000.data_fusions.goal_informations.goal_num = 0;
	blk_ccc_kdl_000.data_fusions.goal_informations.goal_type = 0;
	blk_ccc_kdl_000.data_fusions.goal_informations.goal_speed = 0;
	blk_ccc_kdl_000.data_fusions.goal_informations.goal_height = 0;
	blk_ccc_kdl_000.data_fusions.goal_informations.goal_lot_num = 0;
	blk_ccc_kdl_000.data_fusions.goal_informations.goal_latitude = 0;
	blk_ccc_kdl_000.data_fusions.goal_informations.goal_direction = 0;
	blk_ccc_kdl_000.data_fusions.goal_informations.goal_longitude = 0;
	blk_ccc_kdl_000.data_fusions.goal_informations.goal_attributes = 0;
	blk_ccc_kdl_000.data_fusions.goal_informations.data_valid_flag_bit = 0;

	// 校验和待计算
	blk_ccc_kdl_000.checksum = checkSum(&blk_ccc_kdl_000, sizeof(BLK_CCC_KDL_000) - 1);
}

void send_blk_ccc_kdl_000()
{
	init_blk_ccc_kdl_000();
	align_send_information(&(blk_ccc_kdl_000), sizeof(blk_ccc_kdl_000), 0);
	// 发送
	Send_Message(DDSTables.CCC_KDL_0.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

// 控制权交接申请
// uav_index无人机序号（0-3）
void init_blk_ccc_kdl_002(int uav_index)
{

	blk_ccc_kdl_002.synchronization_code[0] = 0xEB;
	blk_ccc_kdl_002.synchronization_code[1] = 0x94;
	blk_ccc_kdl_002.frame_class = 0xD8;
	blk_ccc_kdl_002.UAV_ID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID; //(unsigned int)blk_kkl_ccc_007.uav_cl_status_info[uav_index].NoHeliID;
#if _PC_SIMULATION_																	  // 目前仿真环境只能是0x3001
	blk_ccc_kdl_002.F_ID2 = GCS_ID;
#else
	blk_ccc_kdl_002.F_ID2 = MANNED_ID; // MANNED_ID;//有人机固定id,MANNED_ID
#endif
	blk_ccc_kdl_002.CR_StationHandoverParamss.C_Down_channel = blk_ofp_ccc_014.CommunicationParam.handover_down_channel;
	blk_ccc_kdl_002.CR_StationHandoverParamss.C_UP_Channel = blk_ofp_ccc_014.CommunicationParam.handover_up_channel;
	blk_ccc_kdl_002.CR_StationHandoverParamss.CDownRate = blk_ofp_ccc_014.CommunicationParam.down_rate;
	blk_ccc_kdl_002.CR_StationHandoverParamss.U_Channel = blk_ofp_ccc_014.CommunicationParam.u_channel;
	memcpy(&blk_ccc_kdl_002.CR_StationHandoverParamss.valid, &blk_ofp_ccc_014.CommunicationParam.dataValid, sizeof(blk_ofp_ccc_014.CommunicationParam.dataValid)); // 有效位未知
	blk_ccc_kdl_002.jyh = checkSum((unsigned char *)&blk_ccc_kdl_002, sizeof(blk_ccc_kdl_002) - 1);
}

// uav_index无人机序号（0-3）
void send_blk_ccc_kdl_002(int uav_index)
{
	init_blk_ccc_kdl_002(uav_index);
	align_send_information(&(blk_ccc_kdl_002), sizeof(blk_ccc_kdl_002), 0);
	// 发送
	Send_Message(DDSTables.CCC_KDL_1.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

// 无人机1光电视频-发送
void init_blk_ccc_kdl_004()
{
	// 初始化无人机1光电视频
}

void send_blk_ccc_kdl_004()
{
	init_blk_ccc_kdl_004();
	align_send_information(&(blk_ccc_kdl_004), sizeof(blk_ccc_kdl_004), 0);
	Send_Message(DDSTables.CCC_KDL_2.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

// 控制权交接申请应答
void init_blk_ccc_kdl_010()
{
	static unsigned char frameCount = 0;
	blk_ccc_kdl_010.synchronization_code[0] = 0xEB;
	blk_ccc_kdl_010.synchronization_code[1] = 0x94;
	blk_ccc_kdl_010.frame_class = 0xDB;
	blk_ccc_kdl_010.index_id = frameCount++;
	blk_ccc_kdl_010.jyh = checkSum((unsigned char *)&blk_ccc_kdl_010.frame_class, sizeof(blk_ccc_kdl_010) - 2);
}
void send_blk_ccc_kdl_010()
{
	init_blk_ccc_kdl_010();
	align_send_information(&(blk_ccc_kdl_010), sizeof(blk_ccc_kdl_010), 0);
	// 发送
	Send_Message(DDSTables.CCC_KDL_11.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

// 无人机2光电视频
void init_blk_ccc_kdl_012()
{
	// 初始化无人机2光电视频
}
void send_blk_ccc_kdl_012()
{

	init_blk_ccc_kdl_012();
	align_send_information(&(blk_ccc_kdl_012), sizeof(blk_ccc_kdl_012), 0);
	// 发送
	Send_Message(DDSTables.CCC_KDL_3.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

// 编队链路信息
void init_blk_ccc_kdl_014()
{
	blk_ccc_kdl_014.synchronization_code[0] = 0xEB;
	blk_ccc_kdl_014.synchronization_code[1] = 0x9C;
	blk_ccc_kdl_014.frame_type = 0xBA;
	blk_ccc_kdl_014.frame_long = 0x10;
	blk_ccc_kdl_014.plane_id = 0x9001;
	if (blk_kkl_ccc_006.CR_uav_cl_status_info[0].NoHeliID == UAV1_ID)
	{
		blk_ccc_kdl_014.uav1_up_lock = blk_kkl_ccc_006.CR_uav_cl_status_info[0].IfNoHeliCon;
		blk_ccc_kdl_014.uav1_down_lock = blk_kkl_ccc_007.uav_cl_status_info[0].IfNoHeliCon;
	}
	if (blk_kkl_ccc_006.CR_uav_cl_status_info[1].NoHeliID == UAV2_ID)
	{
		blk_ccc_kdl_014.uav2_up_lock = blk_kkl_ccc_006.CR_uav_cl_status_info[1].IfNoHeliCon;
		blk_ccc_kdl_014.uav2_down_lock = blk_kkl_ccc_007.uav_cl_status_info[1].IfNoHeliCon;
	}
	blk_ccc_kdl_014.checksum = checkSum((unsigned char *)&blk_ccc_kdl_014, sizeof(BLK_CCC_KDL_014) - 1);
}
void send_blk_ccc_kdl_014()
{

	init_blk_ccc_kdl_014();
	align_send_information(&(blk_ccc_kdl_014), sizeof(blk_ccc_kdl_014), 0);
	// 发送
	Send_Message(DDSTables.CCC_KDL_4.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

// 任务分配结果信息
void init_blk_ccc_kdl_017()
{
	memset(&(blk_ccc_kdl_017), 0, sizeof(blk_ccc_kdl_017));
	blk_ccc_kdl_017.synchronization_code[0] = 0xEB;
	blk_ccc_kdl_017.synchronization_code[1] = 0x94;
	blk_ccc_kdl_017.frame_type = 0xB5;

	memcpy((char *)&blk_ccc_kdl_017 + 3, (char *)&blk_ccc_ofp_019, sizeof(blk_ccc_ofp_019)); // 各自跳过不一致部分
	blk_ccc_kdl_017.check_num = checkSum((unsigned char *)&blk_ccc_kdl_017, sizeof(blk_ccc_kdl_017) - 1);
}
void send_blk_ccc_kdl_017()
{

	init_blk_ccc_kdl_017();
	align_send_information(&(blk_ccc_kdl_017), sizeof(blk_ccc_kdl_017), 0);
	// 发送
	Send_Message(DDSTables.CCC_KDL_5.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

// 有人机通用航路结果信息   修改直接调用封装函数
void init_blk_ccc_kdl_018()
{
	memset(&blk_ccc_kdl_018, 0, sizeof(blk_ccc_kdl_018));
	blk_ccc_kdl_018.synchronization_code[0] = 0xEB;
	blk_ccc_kdl_018.synchronization_code[1] = 0x90;
	blk_ccc_kdl_018.frame_type = 0xB5;
	memcpy((char *)&blk_ccc_kdl_018 + 3, (char *)send_array.dataA, sizeof(BLK_CCC_OFP_018)); // 各自跳过结构体不一致的地方
	blk_ccc_kdl_018.check_num = checkSum((unsigned char *)&blk_ccc_kdl_018, sizeof(blk_ccc_kdl_018) - 1);
}
void send_blk_ccc_kdl_018()
{
	init_blk_ccc_kdl_018();
	align_send_information(&(blk_ccc_kdl_018), sizeof(blk_ccc_kdl_018), 0);
	// 发送
	Send_Message(DDSTables.CCC_KDL_6.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

// 无人机航路规划
void init_blk_ccc_kdl_024()
{
	memset(&blk_ccc_kdl_024, 0, sizeof(blk_ccc_kdl_024));
	blk_ccc_kdl_024.synchronization_code[0] = 0xEB;
	blk_ccc_kdl_024.synchronization_code[1] = 0x90;
	blk_ccc_kdl_024.frame_type = 0x51;
	blk_ccc_kdl_024.program_number = blk_ccc_ofp_024.program_number;
	blk_ccc_kdl_024.number_of_drones = 1;
	// 取单任务头
	memcpy(&blk_ccc_kdl_024.individual_drone_routing_programs.drone_serial_number, &blk_ccc_ofp_024.individual_drone_routing_programs.drone_serial_number, 2);
	memcpy(&blk_ccc_kdl_024.individual_drone_routing_programs.drone_num, &blk_ccc_ofp_024.individual_drone_routing_programs.drone_num, 4);
	memcpy(&blk_ccc_kdl_024.individual_drone_routing_programs.subtasks_number, &blk_ccc_ofp_024.individual_drone_routing_programs.subtasks_number, 2);
	memcpy(&blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.subtask_ID_number, &blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.subtask_ID_number, 4);
	memcpy(&blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.mission_type, &blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.mission_type, 2);
	memcpy(&blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.point_area_type, &blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.point_area_type, 2);
	memcpy(&blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.area_point_line_goal_number, &blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.area_point_line_goal_number, 4);
	memcpy(&blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.AtomicTimeUpper, &blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.AtomicTimeUpper, 1);
	memcpy(&blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.AtomicHighlyUpper, &blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.AtomicHighlyUpper, 1);
	memcpy(&blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.completion_time, &blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.completion_time, 4);
	memcpy(&blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.mission_height, &blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.mission_height, 4);
	blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.waypoints_number = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.waypoints_number;
	for (int index = 0; index < 13; index++)
	{
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].type = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].type;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].validity_of_longitude = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].validity_of_longitude;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].latitude_validity = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].latitude_validity;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].height_validity = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].height_validity;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].speed_validity = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].speed_validity;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].direction_validity = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].direction_validity;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].time_validity = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].time_validity;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].payloads_validity = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].payloads_validity;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].longitude = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].longitude;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].latitude = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].latitude;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].height = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].height;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].speed = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].speed;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].direction = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].direction;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].time = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].time;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].payloads = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].payloads;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].causality = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].causality;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].standby_type = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].standby_type;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].standby_time_lapsNumber_cycleNumber = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].standby_time_lapsNumber_cycleNumber;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].standby_radius_valid_bit = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].standby_radius_valid_bit;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].standby_time_lapsNumber_cycleNumber_valid_bit = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].standby_time_lapsNumber_cycleNumber_valid_bit;
		blk_ccc_kdl_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].standby_radius = blk_ccc_ofp_024.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[index].standby_radius;
	}

	blk_ccc_kdl_024.check_num = checkSum((unsigned char *)&blk_ccc_kdl_024, sizeof(blk_ccc_kdl_024) - 1);
}

// 向kdl发送无人机航路
void send_blk_ccc_kdl_024()
{
	// 整理发送结构体
	init_blk_ccc_kdl_024();
	data_length = sizeof(blk_ccc_kdl_024);
	Send_Message(DDSTables.CCC_KDL_7.niConnectionId, 0, &transaction_id, &blk_ccc_kdl_024, &message_type_id, data_length, &enRetCode);
}

// 任务区/空域信息
void init_blk_ccc_kdl_033()
{
	blk_ccc_kdl_033.synchronization_code[0] = 0XEB;
	blk_ccc_kdl_033.synchronization_code[1] = 0X94;
	blk_ccc_kdl_033.frame_type = (char)0xb8;
	memcpy((char *)&blk_ccc_kdl_033 + 3, (char *)&blk_ccc_ofp_033 + 4, sizeof(blk_ccc_ofp_033) - 4); // 各自跳过结构体不一致的地方
	blk_ccc_kdl_033.check_num = checkSum((unsigned char *)&blk_ccc_kdl_033, sizeof(blk_ccc_kdl_033) - 1);
}

void send_blk_ccc_kdl_033()
{
	init_blk_ccc_kdl_033();
	align_send_information(&(blk_ccc_kdl_033), sizeof(blk_ccc_kdl_033), 0);
	// 发送
	Send_Message(DDSTables.CCC_KDL_8.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

//  1.10 任务点信息
void init_blk_ccc_kdl_034()
{
	int i;
	blk_ccc_kdl_034.synchronization_code[0] = 0XEB;
	blk_ccc_kdl_034.synchronization_code[1] = 0X94;
	blk_ccc_kdl_034.frame_type = (char)0xB9;
	memcpy((char *)&blk_ccc_kdl_034 + 3, (char *)&blk_ccc_ofp_034, sizeof(blk_ccc_ofp_034)); // 各自跳过结构体不一致的地方
	blk_ccc_kdl_034.check_num = checkSum((unsigned char *)&blk_ccc_kdl_034, sizeof(blk_ccc_kdl_034) - 1);
}

void send_blk_ccc_kdl_034()
{
	init_blk_ccc_kdl_034();
	align_send_information(&(blk_ccc_kdl_034), sizeof(blk_ccc_kdl_034), 0);
	// 发送
	Send_Message(DDSTables.CCC_KDL_9.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	// send_array.clear();
}

// 任务线信息
void init_blk_ccc_kdl_035()
{
	blk_ccc_kdl_035.synchronization_code[0] = (char)0xeb;
	blk_ccc_kdl_035.synchronization_code[1] = (char)0x94;
	blk_ccc_kdl_035.frame_type = (char)0xc1;
	memcpy((char *)&blk_ccc_kdl_035 + 3, (char *)&blk_ccc_ofp_035, sizeof(blk_ccc_ofp_035)); // 各自跳过结构体不一致的地方
	blk_ccc_kdl_035.check_num = checkSum((unsigned char *)&blk_ccc_kdl_035, sizeof(blk_ccc_kdl_035) - 1);
}

void send_blk_ccc_kdl_035()
{
	init_blk_ccc_kdl_035();
	data_length = sizeof(blk_ccc_kdl_035);
	// 发送
	Send_Message(DDSTables.CCC_KDL_10.niConnectionId, 0, &transaction_id, &blk_ccc_kdl_035, &message_type_id, data_length, &enRetCode);
}

// 处理赋值有效位
short set_data_valid(short data, int judg_data, int b)
{ // data 进行比特位赋值的数据  judg_data 判断有效的数据  b 赋值的比特位
	if (judg_data > 0)
	{
		data = give_bit(data, b, 1);
	}
	else
	{
		data = give_bit(data, b, 0);
	}
	return data;
}

///*无人机数据有效标志位  diff*/  // 全有效
//    CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = 0;
//    CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = set_data_valid(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,CCC_DPU_data_3.drone_specific_informations[i].remaining_mission_time,0);
//    CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = set_data_valid(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,CCC_DPU_data_3.drone_specific_informations[i].residual_oil_volume,1);
//    CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = set_data_valid(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,CCC_DPU_data_3.drone_specific_informations[i].Ng ,2);
//    CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = set_data_valid(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,CCC_DPU_data_3.drone_specific_informations[i].Nr ,3);
//    CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = set_data_valid(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,CCC_DPU_data_3.drone_specific_informations[i].T45,4);
//    CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = set_data_valid(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,CCC_DPU_data_3.drone_specific_informations[i].U,5);
//    CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid = set_data_valid(CCC_DPU_data_3.drone_specific_informations[i].uav_infos.uav_data_valid,CCC_DPU_data_3.drone_specific_informations[i].U_storage,6);

/*********** KDL通信模块 ************/
// kdl接收
void recv_kdl()
{
	// dds_data_rece.resize(1500);
	message_size = 1500;
	Receive_Message(DDSTables.KDL_CCC_0.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(KDL_CCC_data_0), dds_data_rece.dataA, sizeof(KDL_CCC_data_0));
	}

	Receive_Message(DDSTables.KDL_CCC_1.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(KDL_CCC_data_1), dds_data_rece.dataA, sizeof(KDL_CCC_data_1));
	}

	Receive_Message(DDSTables.KDL_CCC_2.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(KDL_CCC_data_2), dds_data_rece.dataA, sizeof(KDL_CCC_data_2));
	}

	Receive_Message(DDSTables.KDL_CCC_3.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(KDL_CCC_data_3), dds_data_rece.dataA, sizeof(KDL_CCC_data_3));
	}
}

/*********** KKL通信模块 ************/
// kkl 发送
// 链路交接控制指令
void init_link_jiaojie()
{
}

void send_link_jiaojie()
{
	init_link_jiaojie();
	align_send_information(&(CCC_KKL_data_3), sizeof(CCC_KKL_data_3), 0);
	// 发送
	Send_Message(sg_sttaConDirTopicId[150].niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

// 空空链参数设置  042
void init_ckpartset()
{
	int i;
	CCC_KKL_data_2.KKLCNumber = CCC_DPU_data_3.drone_number;
	for (i = 0; i < CCC_KKL_data_2.KKLCNumber; i++)
	{
		CCC_KKL_data_2.uav_clparset_info[i].uav_id = CCC_DPU_data_3.drone_specific_informations[i].platform_num;
		CCC_KKL_data_2.uav_clparset_info[i].DownWorkMode_1 = 1; // 1 前段工作
		CCC_KKL_data_2.uav_clparset_info[i].DownPower_1 = 1;	// 1 大功率 2 小功率
		CCC_KKL_data_2.uav_clparset_info[i].UpRate_1 = 2;		// 1 25.6kbps 2 51.2kbps  3 102.4kbps
		CCC_KKL_data_2.uav_clparset_info[i].DownRate_1 = 2;		// 1 2mbps 2 4mbps 3 8mbps
		CCC_KKL_data_2.uav_clparset_info[i].UpWorkMode_1 = 2;	// 1 相控阵工作  2 静默
		CCC_KKL_data_2.uav_clparset_info[i].UpPower_1 = 1;
		CCC_KKL_data_2.uav_clparset_info[i].PassWay_1 = 2;	 // 1 密文  2 明文
		CCC_KKL_data_2.uav_clparset_info[i].Cwire_1 = i + 1; // 1 左前天线  2 右前 3 左后 4 右后
		CCC_KKL_data_2.uav_clparset_info[i].Upchannel = 99;
		CCC_KKL_data_2.uav_clparset_info[i].Downchannel = 88;
		CCC_KKL_data_2.uav_clparset_info[i].ControlChange = 2; // 1 启动交接 2 取消交接
		CCC_KKL_data_2.uav_clparset_info[i].RecGroundID = 5001;
		CCC_KKL_data_2.uav_clparset_info[i].ChangeConfirm = 0; // 1确认交接成功
	}
}

void send_ckpartset()
{
	init_ckpartset();
	align_send_information(&(CCC_KKL_data_2), sizeof(CCC_KKL_data_2), 0);
	// 发送
	Send_Message(DDSTables.CCC_KKL_2.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

// 导航数据 025
void init_navigation_data()
{
}

void send_navigation_data()
{

	init_navigation_data();
	align_send_information(&(CCC_KKL_data_4), sizeof(CCC_KKL_data_4), 0);
	// 发送
	Send_Message(DDSTables.CCC_KKL_4.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
}

void send_kkl()
{

	// 链路交接控制指令
	send_link_jiaojie();

	// 空空链参数设置  042
	send_ckpartset();

	// 导航数据 025
	send_navigation_data();
}

// kkl接收
void recv_kkl()
{
	// dds_data_rece.resize(1500);
	message_size = 1500;
	Receive_Message(DDSTables.KKL_CCC_4.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(KKL_CCC_data_4), dds_data_rece.dataA, sizeof(KKL_CCC_data_4));
		//// printf("空空链参数反馈信息 success!!!\n");
	}

	Receive_Message(DDSTables.KKL_CCC_3.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(KKL_CCC_data_3), dds_data_rece.dataA, sizeof(KKL_CCC_data_3));
		//// printf("u链状态信息 success!!!\n");
	}

	Receive_Message(DDSTables.KKL_CCC_5.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(KKL_CCC_data_5), dds_data_rece.dataA, sizeof(KKL_CCC_data_5));
		//// printf("kkl链路状态数据 success!!!\n");
	}

	Receive_Message(DDSTables.KKL_CCC_7.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(KKL_CCC_data_7), dds_data_rece.dataA, sizeof(KKL_CCC_data_7));
		//// printf("链路交接控制指令反馈 success!!!\n");
	}

	Receive_Message(DDSTables.KKL_CCC_20.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(KKL_CCC_data_20), dds_data_rece.dataA, sizeof(KKL_CCC_data_20));
		//// printf("飞行故障清单 success!!! \n");
	}
}
/***************************************************************************************************************/

/***************************** DPM1A交互  20241105 ********************************/
// void send_DPM1A_message(){ // 主函数
//     send_DPM1A_1();
//     send_DPM1A_2();
//     send_DPM1A_3();
//     send_DPM1A_4();
//     send_DPM1A_5();
//     send_DPM1A_6();
//     send_DPM1A_7();
//     send_DPM1A_8();
//     send_DPM1A_9();
//     send_DPM1A_10();
//     send_DPM1A_11();
//     send_DPM1A_12();
//     recv_DPM1A_message();
// }

// 无人机状态信息13
void send_DPM1A_1()
{
#if 0
	CCC_DPM_data_0.drone_number = 1;
	CCC_DPM_data_0.drone_specific_informations[0].Ng=99;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.uav_data_valid=0xFFFF;
	CCC_DPM_data_0.drone_specific_informations[0].platform_num=9999;
	CCC_DPM_data_0.drone_specific_informations[0].subtask_type=1;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.uav_longi = 100.01;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.uav_lati = 10.9147;
	CCC_DPM_data_0.drone_specific_informations[0].remaining_mission_time = 160;
	CCC_DPM_data_0.drone_specific_informations[0].residual_oil_volume = 300;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.uav_pitch = 178;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.uav_roll = 179;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.uav_heading =180;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.air_height = 1100;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.radio_height = 1200;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.air_speed = 1350;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.ground_speed = 1000;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.vectx_speed= 900;
	CCC_DPM_data_0.drone_specific_informations[0].data_valid_bit = 0xffff;
	CCC_DPM_data_0.drone_specific_informations[0].uav_infos.uav_icon_flag = 1;


	align_send_information(&(CCC_DPM_data_0), sizeof(CCC_DPM_data_0), 0);
	//发送

	Send_Message(DDSTables.CCC_DPM_0.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);

	if(enRetCode == 0)
	{
		//                 printf("CCC_DPM_data_0 (len %d) SEND SUCCESS!\n", data_length);
	}
	else
	{
		//                 printf("CCC_DPM_data_0 (len %d) SEND FAILED! ret %d\n", data_length,enRetCode);
	}
#endif
}

// 编队链路状态信息14
void send_DPM1A_2()
{
#if 0
	CCC_DPM_data_1.drone_number=99;
	CCC_DPM_data_1.surface_station_num=99;
	//    CCC_DPM_data_1.drone_link_status_informations[0].u_send_strength=80;
	//    CCC_DPM_data_1.drone_link_status_informations[0].c_send_strength=50;

	align_send_information(&CCC_DPM_data_1, sizeof(CCC_DPM_data_1), 0);
	//发送
	Send_Message(DDSTables.CCC_DPM_1.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);

	if(enRetCode == 0){
		//// printf("编队链路状态信息 buflen=%d send success!!!", sizeof (CCC_DPM_data_1));
	}
#endif
}

// 综合态势(目标融合15
void send_DPM1A_3()
{

	CCC_DPM_data_2.tgt_Number = 99;
	align_send_information(&CCC_DPM_data_2, sizeof(CCC_DPM_data_2), 0);
	// 发送
	Send_Message(DDSTables.CCC_DPM_2.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);

	if (enRetCode == 0)
	{
		//// printf("综合态势（目标融合 buflen=%d send success!!!", sizeof (CCC_DPM_data_2));
	}
}

// 任务分配结果信息17
void send_DPM1A_4()
{

	CCC_DPM_data_3.platform_num = 99;
	align_send_information(&CCC_DPM_data_3, sizeof(CCC_DPM_data_3), 0);
	// 发送
	Send_Message(DDSTables.CCC_DPM_3.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);

	if (enRetCode == 0)
	{
		//// printf("任务分配结果信息 buflen=%d send success!!!", sizeof (CCC_DPM_data_3));
	}
}

// 有人机航路规划结果信息18/22/23
void send_DPM1A_5()
{
#if 0
	// 发送通用航路18
	CCC_DPM_data_4_5_6.blk_ccc_ofp_018.subtask_index=99;
	CCC_DPM_data_4_5_6.blk_ccc_ofp_018.airway_point_num = 4;
	CCC_DPM_data_4_5_6.blk_ccc_ofp_018.airway_point_start_num=10;
	CCC_DPM_data_4_5_6.blk_ccc_ofp_018.waypoint_informations[0].waypoint_longitude_and_latitude.longitude = 101;
	CCC_DPM_data_4_5_6.blk_ccc_ofp_018.waypoint_informations[0].waypoint_longitude_and_latitude.latitude = 11;
	CCC_DPM_data_4_5_6.blk_ccc_ofp_018.waypoint_informations[1].waypoint_longitude_and_latitude.longitude = 101.2;
	CCC_DPM_data_4_5_6.blk_ccc_ofp_018.waypoint_informations[1].waypoint_longitude_and_latitude.latitude = 11.2;
	CCC_DPM_data_4_5_6.blk_ccc_ofp_018.waypoint_informations[2].waypoint_longitude_and_latitude.longitude = 101.5;
	CCC_DPM_data_4_5_6.blk_ccc_ofp_018.waypoint_informations[2].waypoint_longitude_and_latitude.latitude = 11.5;
	CCC_DPM_data_4_5_6.blk_ccc_ofp_018.waypoint_informations[3].waypoint_longitude_and_latitude.longitude = 101.8;
	CCC_DPM_data_4_5_6.blk_ccc_ofp_018.waypoint_informations[3].waypoint_longitude_and_latitude.latitude = 11.8;

	align_send_information(&(CCC_DPM_data_4_5_6.blk_ccc_ofp_018), sizeof(CCC_DPM_data_4_5_6.blk_ccc_ofp_018), 0);

	//发送
	Send_Message(DDSTables.CCC_DPM_4.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	//printf("有人机航路规划结果信息 buflen=%d send success!!!\n", data_length);
	// 发送浮标布阵22
	CCC_DPM_data_4_5_6.buoy_deployment_points.program_number=99;
	align_send_information(&(CCC_DPM_data_4_5_6.buoy_deployment_points), sizeof(CCC_DPM_data_4_5_6.buoy_deployment_points), 0);
	//发送
	//Send_Message(DDSTables.CCC_DPM_5.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	CCC_DPM_data_4_5_6.suspended_sound_fixed_measurement_points.subtasks_number=99;
	// 发送吊声定测23
	align_send_information(&(CCC_DPM_data_4_5_6.suspended_sound_fixed_measurement_points), sizeof(CCC_DPM_data_4_5_6.suspended_sound_fixed_measurement_points), 0);
	//发送
	//    Send_Message(DDSTables.CCC_DPM_6.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);

	if(enRetCode == 0)
	{
		//         printf("有人机航路规划结果信息 buflen=%d send success!!!\n", sizeof (CCC_DPM_data_4_5_6));
		//printf("有人机航路规划结果信息 buflen=%d send success!!!\n", data_length);
		//// printf("浮标布阵 buflen=%d send success!!!", sizeof (CCC_DPM_data_4_5_6.buoy_deployment_points));
		//// printf("吊声定测 buflen=%d send success!!!", sizeof (CCC_DPM_data_4_5_6.suspended_sound_fixed_measurement_points));
	}
#endif
}

// 无人机航路规划24
void send_DPM1A_6()
{
	CCC_DPM_data_7.number_of_drones = 1;
	CCC_DPM_data_7.individual_drone_routing_programs.planning_informations.waypoints_number = 3;
	CCC_DPM_data_7.individual_drone_routing_programs.subtask_index = 3;
	CCC_DPM_data_7.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].waypoint_informations.waypoint_longitude_and_latitude.longitude = 100.5;
	CCC_DPM_data_7.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].waypoint_informations.waypoint_longitude_and_latitude.latitude = 10.5;
	CCC_DPM_data_7.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].waypoint_informations.waypoint_longitude_and_latitude.longitude = 99.8;
	CCC_DPM_data_7.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].waypoint_informations.waypoint_longitude_and_latitude.latitude = 10.2;
	CCC_DPM_data_7.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[2].waypoint_informations.waypoint_longitude_and_latitude.longitude = 99.5;
	CCC_DPM_data_7.individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[2].waypoint_informations.waypoint_longitude_and_latitude.latitude = 9.5;
	// CCC_DPM_data_7.program_number=99;
	align_send_information(&CCC_DPM_data_7, sizeof(CCC_DPM_data_7), 0);
	// 发送
	Send_Message(DDSTables.CCC_DPM_7.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);

	if (enRetCode == 0)
	{
		//// printf("无人机航路规划 buflen=%d send success!!!", sizeof (CCC_DPM_data_7));
	}
}

// 协同任务执行状态提示32
void send_DPM1A_7()
{

	CCC_DPM_data_8.cue_message_type = 99;
	align_send_information(&CCC_DPM_data_8, sizeof(CCC_DPM_data_8), 0);
	// 发送
	Send_Message(DDSTables.CCC_DPM_8.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//// printf("协同任务执行状态提示 buflen=%d send success!!!", sizeof (CCC_DPM_data_8));
	}
}

// 任务区/空域信息33
void send_DPM1A_8()
{

	CCC_DPM_data_9.area_number = 99;
	align_send_information(&CCC_DPM_data_9, sizeof(CCC_DPM_data_9), 0);
	// 发送
	Send_Message(DDSTables.CCC_DPM_9.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//// printf("任务区/空域信息 buflen=%d send success!!!", sizeof (CCC_DPM_data_9));
	}
}

// 任务点信息34
void send_DPM1A_9()
{

	CCC_DPM_data_10.point_number = 99;
	align_send_information(&CCC_DPM_data_10, sizeof(CCC_DPM_data_10), 0);
	// 发送
	Send_Message(DDSTables.CCC_DPM_10.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//// printf("任务点信息 buflen=%d send success!!!", sizeof (CCC_DPM_data_10));
	}
}

// 任务线信息35
void send_DPM1A_10()
{

	CCC_DPM_data_11.line_number = 99;
	align_send_information(&CCC_DPM_data_11, sizeof(CCC_DPM_data_11), 0);
	// 发送
	Send_Message(DDSTables.CCC_DPM_11.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//// printf("任务线信息 buflen=%d send success!!!", sizeof (CCC_DPM_data_11));
	}
}

// 无人机光电控制权反馈41
void send_DPM1A_11()
{

	CCC_DPM_data_12.ssds_control_feedback = 99;
	align_send_information(&CCC_DPM_data_12, sizeof(CCC_DPM_data_12), 0);
	// 发送
	Send_Message(DDSTables.CCC_DPM_12.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0)
	{
		//// printf("无人机光电控制权反馈 buflen=%d send success!!!", sizeof (CCC_DPM_data_12));
	}
}

// 光电视频控制反馈42
void send_DPM1A_12()
{
#if 0
	CCC_DPM_data_13.video_source_type=99;
	align_send_information(&CCC_DPM_data_13, sizeof(CCC_DPM_data_13), 0);
	//发送
	Send_Message(DDSTables.CCC_DPM_13.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0){
		//// printf("光电视频控制反馈 buflen=%d send success!!!", sizeof (CCC_DPM_data_13));
	}
#endif
}

// DPM1A接收 无人机光电视频控制指令16/14
void recv_DPM1A_message()
{
	// dds_data_rece.resize(1500);
	message_size = 1500;
	Receive_Message(DDSTables.DPM_CCC_2.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		memcpy(&(DPM_CCC_data_2), dds_data_rece.dataA, sizeof(DPM_CCC_data_2) + 4);
		for (int i = 0; i < sizeof(DPM_CCC_data_2) + 4; i++)
		{
			//// printf("%02X ",dds_data_rece.dataA[i]);
		}
		//// printf("\n");
		//// printf("DPM1A接收 无人机光电视频控制指令 buflen=%d success!!!\n", sizeof(DPM_guangdina_video_instructions));
	}
	else
	{
		//// printf("recv_DPM1A_message ret=%d\n",enRetCode);
	}
}

void hanover_state(int uav_index, unsigned char ctrl_releasing, unsigned char ctrl_recieving, unsigned short status)
{
	data_length = sizeof(CR_ControlHandoverStatusFeedback);
	CCC_DPU_MMM_03.uav_ID = formationId[uav_index].planeId;
	CCC_DPU_MMM_03.uav_seri_num = uav_index + 1;
	CCC_DPU_MMM_03.ctrl_releasing = ctrl_releasing;
	CCC_DPU_MMM_03.ctrl_recieving = ctrl_recieving;
	CCC_DPU_MMM_03.status = status;
	// 0xa22203
	Send_Message(DDSTables.CCC_DPU_25.niConnectionId, 0, &transaction_id, &CCC_DPU_MMM_03, &message_type_id, data_length, &enRetCode);
	printf("status 0x%x\n", status);
}

// 无人机控制权交接
//  设置blk_ccc_kkl_009_029_030_031包并发送
//  uav_index:无人机序号
void set_blk_ccc_kkl_009_029_030_031(int uav_index, unsigned int *p_main_link_send)
{
	static unsigned int main_link_send_count = 3; // 发送主链交接指令次数
	// 发送交接参数
	if (*p_main_link_send > 0)
	{

		// 类型固定04
		blk_ccc_kkl_009_029_030_031[uav_index].Type = 0x04;

		switch (*p_main_link_send)
		{
		case 1:
			blk_ccc_kkl_009_029_030_031[uav_index].UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiang = 0x03; // 被控无人机id
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangH = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID >> 8;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangL = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID & 0x00ff;
			break;
		case 2:
			blk_ccc_kkl_009_029_030_031[uav_index].UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiang = 0x09; // 09H 主控站－ＩＤ设置　控制站ID号
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangH = MANNED_ID >> 8;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangL = MANNED_ID & 0x00ff;
			break;
		case 3:
			blk_ccc_kkl_009_029_030_031[uav_index].UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiang = 0x0A; // 0AH 接机站-ID设置 控制站ID号
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangH = blk_ofp_ccc_014.CommunicationParam.rcv_platform_code >> 8;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangL = blk_ofp_ccc_014.CommunicationParam.rcv_platform_code & 0x00ff;
			break;
		case 4:
			blk_ccc_kkl_009_029_030_031[uav_index].UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiang = 0x35; // 35H C下行交接频道 同上
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangH = blk_ofp_ccc_014.CommunicationParam.handover_down_channel >> 8;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangL = blk_ofp_ccc_014.CommunicationParam.handover_down_channel & 0x00ff;
			break;
		case 5:
			blk_ccc_kkl_009_029_030_031[uav_index].UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiang = 0x39; // 39H C上行交接频道 同上
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangH = blk_ofp_ccc_014.CommunicationParam.handover_up_channel >> 8;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangL = blk_ofp_ccc_014.CommunicationParam.handover_up_channel & 0x00ff;
			break;

#if 0 // 暂时不发
		case 6:
			blk_ccc_kkl_009_029_030_031[uav_index].UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiang = 0x3A;//3AH C下行速率 H字节D7-D4：时隙分配 L字节D3-D0：速率控制
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangH = blk_ofp_ccc_014.CommunicationParam.down_rate >> 8;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangL = blk_ofp_ccc_014.CommunicationParam.down_rate & 0x00ff;
			break;
		case 7:
			blk_ccc_kkl_009_029_030_031[uav_index].UavID = blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].UAV_ID;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiang = 0x6A;//6AH UHF接机站频道 同上
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangH = blk_ofp_ccc_014.CommunicationParam.u_channel >> 8;
			blk_ccc_kkl_009_029_030_031[uav_index].LianXuLiangL = blk_ofp_ccc_014.CommunicationParam.u_channel & 0x00ff;
			break;
#endif

		case 33:
			blk_ccc_kkl_009_029_030_031[uav_index].KaiGuanLiang = 0x0C;        //0CH 启动主链转移
			break;
		case 55:
			blk_ccc_kkl_009_029_030_031[uav_index].KaiGuanLiang = 0x0A;        //0AH C链交接成功确认
			break;
		default:
			*p_main_link_send = 0;// 退出发送
			break;
		}
		send_blk_ccc_kkl_009_029_030_031(uav_index);

		if(--main_link_send_count<=0)//发完三次
		{
			(*p_main_link_send)++;//发送下一个数据
			main_link_send_count = 3;// 发送数重置
		}
	}
}

//无人机控制权交接-申请控制权
// uav_index:无人机序号
// p_take_over_flag交接过程标志量。0：未在交接；1指针：在申请控制权；2：在释放控制权
void UAV_Takeover_Process_get(int uav_index, unsigned int* p_take_over_flag)
{
#if _PC_SIMULATION_

	static int sleepCount = 40; // 延时计数器
	static int pre_take_over_flag = 0;// 前一拍交接过程标志量。0：未在交接；1：在申请控制权；2：在释放控制权
	static int sendCount = 50;// 发送计数器

	if(*p_take_over_flag!=1)
	{
		pre_take_over_flag = 0;
		return;
	}

	if(uav_index<0 || uav_index > UAV_MAX_NUM-1)
		return;

	// 第一次进入交接
	if(pre_take_over_flag == 0)
	{
		s4D_frame_04[uav_index] = 0;

		//给DPU发控制权交接状态，控制中
		hanover_state(0,1);

		// blk_ccc_kdl_002参数赋值
		send_blk_ccc_kdl_002(uav_index);
	}

	// 接收应答(blk_kdl_ccc_010事件消息，把帧类别当做有效性)
	if(blk_kdl_ccc_010.frame_class != 0xD9)
	{
		pre_take_over_flag = *p_take_over_flag;// 更新前一拍的标志量

		// 没收到则等待
		return;
	}
	else if(blk_kdl_ccc_010.ans != 0) // 拒绝则退出交接(目前认为不同意则为拒绝)，同意则继续下面处理
	{
		//给DPU发控制权交接状态
		hanover_state(0,3);
		// 直接重置，退出交接流程
		*p_take_over_flag = 0;
		blk_kdl_ccc_010.frame_class = 0;

		//完成交接则重置blk_ofp_ccc_014
		memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));
		return;
	}


	//收到启动交接指令
	if(s4D_frame_04[uav_index] == 1)
	{
		// 延时2s
		sleepCount--;
	}

	if(sleepCount < 0)
	{
		if(sendCount-- > 0)
		{
			fc_order_send(uav_index, 1);
		}
		else
		{
			s4D_frame_04[uav_index] = 0;
			sleepCount = 40;
			sendCount = 50;

			//无人机主链交接成功，标志位清零
			*p_take_over_flag = 0;
			blk_kdl_ccc_010.frame_class = 0;// 判断交接应答的
			//给DPU发控制权交接状态，交接完成
			hanover_state(0,2);
			init_blk_ccc_ofp_036(3,1,plan_task_type);
			//完成交接则重置blk_ofp_ccc_014
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));
		}
	}


	pre_take_over_flag = *p_take_over_flag;

#else

	// 标志量
	static unsigned int main_link_send = 0; // 是否发送3包主链交接指令。0：不需要发送；1：C链交接成功确认；2：启动主链转移（释放主链）
	static int pre_take_over_flag = 0;// 前一拍交接过程标志量。0：未在交接；1：在申请控制权；2：在释放控制权
	static int get_step = 0; // 获取控制权的步骤

	if(*p_take_over_flag!=1)
	{
		pre_take_over_flag = 0;
		return;
	}

	if(uav_index<0 || uav_index > UAV_MAX_NUM-1)
		return;


	/***** 交接第一步:发送控制权交接信息send_blk_ccc_kdl_002  ***********/
	if(get_step == 0)
	{
		// 发送控制权交接信息
		send_blk_ccc_kdl_002(uav_index);
		get_step++;
		return;
	}

	/***** 交接第二步:等待接收blk_kdl_ccc_015  ***********/
	if(get_step == 1)
	{
		// 接收应答(blk_kdl_ccc_015事件消息，把帧类别当做有效性)
		if(blk_kdl_ccc_015.frame_class != 0xD9)
		{
			// 没收到则等待
			return;
		}

		// 收到判断结果
		if(blk_kdl_ccc_015.ans != 0) // 拒绝则退出交接(目前认为不同意则为拒绝)，同意则继续下面处理
		{
			//给DPU发控制权交接状态,地面站拒绝
			hanover_state(uav_index,0,3,0x9);
			// 直接重置，退出交接流程
			*p_take_over_flag = 0;
			blk_kdl_ccc_015.frame_class = 0;
			get_step = 0;
			return;
		}
		else
		{
			//给DPU发控制权交接状态,地面站同意-正在交接
			blk_kdl_ccc_015.frame_class = 0;
			hanover_state(uav_index,0,1,0xa);
			get_step++;// 进入下一步
		}

	}

	/***** 交接第三步:CCC将本机C链参数修改成地面站返回的参数  ***********/
	static int step3_send_time = 0; // 发送3次判定
	static int step3_rec_count = 0; // 判断是否切换成功,连续判断5包
	if(get_step == 2)
	{

		if(step3_send_time < 3) // 发送逻辑
		{
			if(step3_rec_count == 0) // 每次为0时发送，然后等待5包
			{

				//使用空地链传过来的参数

				main_link_send = 11; // ?待定
			}

			// 未发送完之前先一直发送
			if(main_link_send!=0)
			{
				set_blk_ccc_kkl_000(uav_index, &main_link_send);
				return;
			}


			int tem_change_success = 0;


			for(int i =0; i<UAV_MAX_NUM; i++)
			{

				if(blk_kkl_ccc_007.uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
				{
					if(1) //判断链路状态,判断设置的速率和频段是否与发送的一致 ，20帧超时
					{
						tem_change_success += 1;
						break;
					}
				}
			}

			if(tem_change_success == 2)// 说明06、07都满足
			{
				get_step++;// 进入下一步
			}
			else
			{
				step3_rec_count++; // 接收次数增加（最多等5拍）

				if(step3_rec_count>5)
				{
					step3_rec_count = 0;// 退出本次等待
					step3_send_time++;// 发送次数自增
				}
			}

		}
		else
		{
			// 本步骤标志量清零
			step3_send_time = 0;
			step3_rec_count = 0;

			// 步骤标志清零
			get_step = 0;


			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 发送拒绝交接指令
			blk_ccc_kdl_010.ans = 1;
			send_blk_ccc_kdl_010();

			// 反馈交接状态,申请控制权失败-链路参数切换不成功
			hanover_state(uav_index,0,3,0x8);

		}

		return;
	}




	/***** 交接第四步:判断上下行锁定3秒，不满足则失败（允许最大连续5包失锁）  ***********/
	static int step4_wait_count = 0;// 进入第三步先等待5拍
	static int step4_lock_time = 0; // 锁定三秒判定
	static int step4_break_count = 0; // 判断是否累计5包失锁，是则失败
	if(get_step == 3)
	{
		// 清空第上步骤标志量
		step3_send_time = 0;
		step3_rec_count = 0;

		// 等待5拍
		if(step4_wait_count < 5)
		{
			step4_wait_count++;
			return;
		}

		int tem_change_success = 0;
		for(int i =0; i<UAV_MAX_NUM; i++)
		{
			// 根据编队信息找到飞机id，根据飞机id找相应的无人机链路信息
			if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
			{
				if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].IfNoHeliCon == 1 && blk_kkl_ccc_006.CR_uav_cl_status_info[i].UAV_StationID == MANNED_ID) //lock？
				{
					tem_change_success += 1;
					break;
				}
			}
		}

		for(int i =0; i<UAV_MAX_NUM; i++)
		{

			if(blk_kkl_ccc_007.uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
			{
				if(blk_kkl_ccc_007.uav_cl_status_info[i].IfNoHeliCon == 1) //lock？
				{
					tem_change_success += 1;
					break;
				}
			}

		}

		if(tem_change_success == 2)// 说明06、07都满足
		{
			step4_lock_time++;// 计时累加
		}
		else
		{
			step4_break_count++; // 接收次数增加（最多等5拍）
		}


		// 累计失锁3分钟则失败
		if(step4_break_count>180*20)
		{
			// 本步骤标志量清零
			step4_wait_count = 0;
			step4_lock_time = 0;
			step4_break_count = 0;

			// 步骤标志清零
			get_step = 0;

			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 反馈交接状态,控制权申请-失败-链路未建立
			hanover_state(uav_index,0,3,0xb);
		}

		// 累计锁定1秒
		if(step4_lock_time>20)
		{
			get_step++;// 进入下一步
		}

		return;
	}



	/***** 交接第五步:发送主链交接成功指令  直接发送一包，进入下一步，后续需要跟无人机部确认***********/
	static int step5_send_time = 0; // 发送1次判定 // 最多几次需确认？
	static int step5_rec_count = 0; // 判断是否切换成功,连续判断5包
	if(get_step == 4)
	{
		// 清空第上步骤标志量
		step4_wait_count = 0;
		step4_lock_time = 0;
		step4_break_count = 0;


		if(step5_send_time < 1) // 发送逻辑
		{
			step5_send_time++;
			main_link_send = 31;
		}

		// 未发送完之前先一直发送
		if(main_link_send!=0)
		{
			set_blk_ccc_kkl_000(uav_index, &main_link_send);
		}
		else
		{
			get_step++;// 进入下一步
		}

		return;
	}



	/***** 交接第六步:发送遥控指令帧结束交接指令（权交接）  ***********/
	static int step6_send_time = 0; // 发送1次判定 // 最多几次需确认？
	static int step6_rec_count = 0; // 判断是否切换成功,连续判断5包
	if(get_step == 5)
	{
		// 清空第上步骤标志量
		step5_send_time = 0;
		step5_rec_count = 0;


		if(step6_send_time < 3) // 发送逻辑
		{
			if(step6_rec_count == 0) // 每次为0时发送，然后等待5包
			{
				// 发送遥控指令帧结束交接指令
				fc_order_send(uav_index, 1);
			}


			// 判断遥测帧
			if(s4D_frame_24[uav_index] == 1)
			{
				get_step++;// 进入下一步
			}
			else
			{
				step6_rec_count++; // 接收次数增加（最多等5拍）

				if(step6_rec_count>5)
				{
					step6_rec_count = 0;// 退出本次等待
					step6_send_time++;// 发送次数自增
				}
			}

		}
		else
		{
			// 本步骤标志量清零
			step6_send_time = 0;
			step6_rec_count = 0;

			// 步骤标志清零
			get_step = 0;

			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 反馈交接状态???飞行遥控回报超时如何判断
			hanover_state(uav_index,3,0,0); //？枚举待更新

		}

		return;
	}


	/***** 交接第七步:根据遥测帧 判断是否交接成功 ***********/
	static int step7_send_time = 0; // 发送1次判定 // 最多几次需确认？
	static int step7_rec_count = 0; // 判断是否切换成功,连续判断5包
	if(get_step == 6)
	{
		// 清空第上步骤标志量
		step6_send_time = 0;
		step6_rec_count = 0;

		if(step7_send_time < 1) // 发送逻辑
		{

			// 判断有控权则成功
			if(formationId[uav_index].isControl == 1)
			{
				get_step++;// 进入下一步
			}
			else
			{
				step7_rec_count++; // 接收次数增加（最多等5拍）

				if(step7_rec_count>5)
				{
					step7_rec_count = 0;// 退出本次等待
					step7_send_time++;// 发送次数自增
				}
			}

		}
		else
		{
			// 本步骤标志量清零
			step7_send_time = 0;
			step7_rec_count = 0;

			// 步骤标志清零
			get_step = 0;

			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 反馈交接状态???未收到遥测帧超时
			//            hanover_state(4,0); //？枚举待更新

		}

		return;
	}



	/***** 交接第八步:完成交接 ***********/
	if(get_step == 7)
	{
		// 清空第上步骤标志量
		step7_send_time = 0;
		step7_rec_count = 0;

		// 步骤标志清零
		get_step = 0;

		// 本机交接类别清零
		*p_take_over_flag = 0;

		// 清空本次交接指令
		memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

		// 反馈交接状态成功,控制权申请-成功
		hanover_state(uav_index,0,2,0xc);
	}




#endif

}

//无人机控制权交接-手动释放控制权
// uav_index:无人机序号
// p_hand_free_flag: 手动步骤（可以外部传入控制步骤，也可以内部自增控制步骤）
void UAV_Takeover_Process_get_hand(int uav_index, unsigned int* p_hand_get_step)
{
#if _PC_SIMULATION_



#else

	static unsigned int main_link_send = 0; // 是否发送3包主链交接指令。0：不需要发送；1：C链交接成功确认；2：启动主链转移（释放主链）


	//    if(uav_index<0 || uav_index > UAV_MAX_NUM-1)
	//        return;


	/***** 交接第0步:判断是否上下行锁定  ***********/
	static int step0_lock_time[4] = {0,0,0,0}; // 锁定三秒判定
	if(*p_hand_get_step==0) // 直接判断是否是0，周期判断
	{
		// 清空第上步骤标志量
		//无


		int tem_change_success = 0;
		for(int i =0; i<UAV_MAX_NUM; i++)
		{
			// 根据编队信息找到飞机id，根据飞机id找相应的无人机链路信息
			if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
			{
				if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].IfNoHeliCon == 1 && blk_kkl_ccc_006.CR_uav_cl_status_info[i].UAV_StationID == MANNED_ID) //lock？
				{
					tem_change_success += 1;
					break;
				}
			}
		}

		for(int i =0; i<UAV_MAX_NUM; i++)
		{
			//c链无人机id与飞控编队无人机id一致，目前该无人机无控制权，遥测数据是从C链来的
			if(blk_kkl_ccc_007.uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId && formationId[uav_index].isControl == 0 && formationId[uav_index].C_U == 1)
			{
				if(blk_kkl_ccc_007.uav_cl_status_info[i].IfNoHeliCon == 1) //lock？
				{
					tem_change_success += 1;
					break;
				}
			}
		}

		if(tem_change_success == 2)// 说明06、07都满足
		{
			step0_lock_time[uav_index]++;// 计时累加
		}


		// 累计锁定3秒60拍，暂时30拍
		if(step0_lock_time[uav_index] == 30)
		{
			//手动获取控制权-主链建立
			hanover_state(uav_index,0,1,0x11);// 返回状态

			//			//锁定每三拍发送一次
			//			step0_lock_time = 0;
		}

		return;
	}

	/***** 交接第1步:发送主链交接和权交接  ***********/
	static int step1_send_time = 0; // 发送1次判定 // 最多几次需确认？
	static int step1_rec_count = 0; // 判断是否切换成功,连续判断5包
	static int step1_time_out = 0; // 超时判断
	static int main_link = 1; // 链交接发送一次三拍
	if(*p_hand_get_step == 1)
	{
		// 清空第上步骤标志量
		step0_lock_time[uav_index] = 0;

		if(step1_send_time < 1) // 发送逻辑
		{
			if(step1_rec_count == 0) // 每次为0时发送，然后等待5包
			{
				// 申请控制权
				fc_order_send(uav_index, 1);
				step1_time_out ++;
				if(main_link)
				{
					//主动获取-正在接收控制权
					hanover_state(uav_index,0,1,0x12);// 返回状态
					// 链交接
					main_link_send = 31;
					main_link = 0;
				}
			}

			// 未发送完之前先一直发送
			if(main_link_send!=0)
			{
				set_blk_ccc_kkl_000(uav_index, &main_link_send);
				return;
			}

			// 判断有控权则成功
			if(formationId[uav_index].isControl == 1)
			{
				//主动获取-控制权交接成功
				hanover_state(uav_index,0,2,0x13); // 反馈

				//清空标志量
				//当前步骤标志清零
				*p_hand_get_step += 1;
				step1_send_time = 0;
				step1_rec_count = 0;
				step1_time_out = 0;
				main_link = 1;
			}
			else if(step1_time_out > 300)
			{
				//主动获取-控制权交接失败
				hanover_state(uav_index,0,2,0x14); // 反馈

				//清空标志量
				//当前步骤标志清零
				*p_hand_get_step += 1;
				step1_send_time = 0;
				step1_rec_count = 0;
				step1_time_out = 0;
				main_link = 1;
			}
		}

	}

	/***** 交接第2步:待综显确认本次交接结束  ***********/
	if(*p_hand_get_step == 2)
	{
		//结束交接判断
		if(blk_ofp_ccc_044.getCtrl == 2)
		{
			*p_hand_get_step = 0;
			printf("get_hand end\n");
		}
	}


#endif
}


void UAV_Takeover_Process_get_hand_test(int uav_index, unsigned int* p_hand_get_step)
{
	static int step1_send_time = 0; // 发送1次判定 // 最多几次需确认？
	static int step1_rec_count = 0; // 判断是否切换成功,连续判断5包
	static int step1_time_out = 0; // 超时判断
	if(*p_hand_get_step == 1)
	{
		//获取控制权
		if(step1_rec_count%5 == 0) // 每次为0时发送，然后等待5包
		{
			// 申请控制权
			fc_order_send(uav_index, 1);
			step1_time_out ++;
			//主动获取-正在接收控制权
			hanover_state(uav_index,0,1,0x12);// 返回状态
		}
		step1_rec_count++;

		// 判断有控权则成功
		if(formationId[uav_index].isControl == 1)
		{
			//主动获取-控制权交接成功
			hanover_state(uav_index,0,2,0x13); // 反馈

			//清空标志量
			//当前步骤标志清零
			*p_hand_get_step += 1;
			step1_send_time = 0;
			step1_rec_count = 0;
			step1_time_out = 0;
		}
		else if(step1_time_out > 300)
		{
			//主动获取-控制权交接失败
			hanover_state(uav_index,0,2,0x14); // 反馈

			//清空标志量
			//当前步骤标志清零
			*p_hand_get_step += 1;
			step1_send_time = 0;
			step1_rec_count = 0;
			step1_time_out = 0;
		}
	}

	else if(*p_hand_get_step == 2)
	{
		//释放控制权
		//结束交接判断
		if(blk_ofp_ccc_044.getCtrl == 2)
		{
			*p_hand_get_step = 0;
			printf("get_hand end\n");
		}
	}
}


//无人机控制权交接-释放控制权
// uav_index:无人机序号
// p_take_over_flag交接过程标志量。0：未在交接；1指针：在申请控制权；2：在释放控制权
void UAV_Takeover_Process_free(int uav_index, unsigned int* p_take_over_flag)
{
#if _PC_SIMULATION_


	static int pre_take_over_flag = 0;// 前一拍交接过程标志量。0：未在交接；1：在申请控制权；2：在释放控制权
	static int sendCount = 100;

	if(*p_take_over_flag!=2)
	{
		pre_take_over_flag = 0;
		return;
	}

	if(uav_index<0 || uav_index > UAV_MAX_NUM-1)
		return;

	// 第一次进入交接
	if(pre_take_over_flag == 0)
	{
		// 重置标志量
		s4D_frame_24[uav_index] = 0;
		//给DPU发控制权交接状态
		hanover_state(1,0);
		sendCount = 100;
	}

	if(sendCount-- > 0)
	{
		// 释放控制权
		fc_order_send(uav_index, 0);
	}

	// 完成交接
	if(s4D_frame_24[uav_index] == 2)
	{
		s4D_frame_24[uav_index] = 0;
		//所有无人机释放成功，标志位清零
		*p_take_over_flag = 0;
		//给DPU发控制权交接状态，释放完成
		hanover_state(2,0);
		sendCount = 100;
		init_blk_ccc_ofp_036(3,1,plan_task_type);
		//完成交接则重置blk_ofp_ccc_014
		memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));
	}

	pre_take_over_flag = *p_take_over_flag;


#else

	static int pre_take_over_flag = 0;// 前一拍交接过程标志量。0：未在交接；1：在申请控制权；2：在释放控制权
	static int free_step = 0; // 释放控制权的步骤
	static unsigned int main_link_send = 0; // 是否发送3包主链交接指令。0：不需要发送；1：C链交接成功确认；2：启动主链转移（释放主链）

	if(*p_take_over_flag!=2)
	{
		free_step = 0;
		pre_take_over_flag = 0;
		return;
	}

	if(uav_index<0 || uav_index > UAV_MAX_NUM-1)
		return;

	/***** 交接第一步:切对应无人机的对应速率到25.6// 判断速率是否切换成功（最多切三次，每次等5拍，都不成功则判为切换失败）  ***********/
	static int step1_send_time = 0; // 发送3次判定
	static int step1_rec_count = 0; // 判断是否切换成功,连续判断5包
	if(free_step == 0)
	{

		//发送之前判断上行锁定或下行锁定，连续15s不锁定就直接失败
		static int lock_cnt_1 = 0;
		//连续十五秒失锁失败
		if(lock_cnt_1 > 20*15)
		{
			// 本步骤标志量清零
			step1_send_time = 0;
			step1_rec_count = 0;

			// 步骤标志清零
			free_step = 0;

			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 发送拒绝交接指令
			blk_ccc_kdl_010.ans = 1;
			send_blk_ccc_kdl_010();

			// 反馈交接状态,释放控制权-交接准备失败-速率切换失败
			hanover_state(uav_index,3,0,0x1);
		}
		if(up_down_lock(uav_index) == 0 && step1_send_time == 0)
		{
			lock_cnt_1++;
			return;
		}
		else
		{
			//上下行锁定，清空超时标志位，进入下一个判断
			lock_cnt_1 = 0;
		}
		//判断006 007 是否是25.6k,就直接进入下一步
		int tem_change_success = 0;
		for(int i =0; i<UAV_MAX_NUM; i++)
		{
			// 根据编队信息找到飞机id，根据飞机id找相应的无人机链路信息
			if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
			{
				if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].DownRate == 1) //25.6?
				{
					tem_change_success += 1;
					break;
				}
			}
		}

		for(int i =0; i<UAV_MAX_NUM; i++)
		{
			if(blk_kkl_ccc_007.uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
			{
				if(blk_kkl_ccc_007.uav_cl_status_info[i].DownRate == 1) //25.6?
				{
					tem_change_success += 1;
					break;
				}
			}
		}
		static int hold_cnt_1 = 0;
		if(tem_change_success == 2)// 说明06、07都满足
		{
			hold_cnt_1++;
		}
		//链保持三帧
		if(hold_cnt_1 > 3)
		{
			hold_cnt_1 = 0;
			free_step++;// 进入下一步
		}
		// 发送逻辑，切换25.6k
		if(step1_send_time < 1)
		{
			main_link_send = 11;
			step1_send_time ++;
		}
		else
		{
			//发送后开始计时
			step1_rec_count++;
		}

		// 未发送完之前先一直发送
		if(main_link_send!=0)
		{
			set_blk_ccc_kkl_000(uav_index, &main_link_send);
		}

		//发送后超过二十拍失败
		if(step1_rec_count > 25)
		{
			// 本步骤标志量清零
			step1_send_time = 0;
			step1_rec_count = 0;

			// 步骤标志清零
			free_step = 0;

			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 发送拒绝交接指令
			blk_ccc_kdl_010.ans = 1;
			send_blk_ccc_kdl_010();

			// 反馈交接状态,释放控制权-交接准备失败-速率切换失败
			hanover_state(uav_index,3,0,0x1);

		}

		return;
	}

	/***** 交接第二步:切对应无人机的对应速率到全天线// 判断天线是否切换成功（最多切三次，每次等5拍，都不成功则判为切换失败）  ***********/
	static int step2_send_time = 0; // 发送3次判定
	static int step2_rec_count = 0; // 判断是否切换成功,连续判断5包
	if(free_step == 1)
	{
		// 清空第上步骤标志量
		step1_send_time = 0;
		step1_rec_count = 0;


		//发送之前判断上行锁定或下行锁定，连续15s不锁定就直接失败
		static int lock_cnt_2 = 0;
		//连续十五秒失锁失败
		if(lock_cnt_2 > 20*15)
		{
			// 本步骤标志量清零
			step2_send_time = 0;
			step2_rec_count = 0;

			// 步骤标志清零
			free_step = 0;

			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 发送拒绝交接指令
			blk_ccc_kdl_010.ans = 1;
			send_blk_ccc_kdl_010();

			// 反馈交接状态,释放控制权-交接准备失败-天线模式切换失败
			hanover_state(uav_index,3,0,0x2);
		}
		if(up_down_lock(uav_index) == 0 && step2_send_time == 0)
		{
			lock_cnt_2++;
			return;
		}
		else
		{
			//上下行锁定，清空超时标志位，进入下一个判断
			lock_cnt_2 = 0;
		}
		//判断 是否是全天线,就直接进入下一步
		int tem_change_success = 0;
		for(int i =0; i<UAV_MAX_NUM; i++)
		{
			// 根据编队信息找到飞机id，根据飞机id找相应的无人机链路信息
			if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
			{
				if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].WorkMode == 1) //全向天线？
				{
					tem_change_success += 1;
					break;
				}
			}
		}

		for(int i =0; i<UAV_MAX_NUM; i++)
		{

			if(blk_kkl_ccc_007.uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
			{
				if(blk_kkl_ccc_007.uav_cl_status_info[i].WorkMode == 1) //全向天线？
				{
					tem_change_success += 1;
					break;
				}
			}
		}

		static int hold_cnt_2 = 0;
		if(tem_change_success == 2)// 说明06、07都满足
		{
			hold_cnt_2++;
		}
		//链保持三帧
		if(hold_cnt_2 > 3)
		{
			hold_cnt_2 = 0;
			free_step+= 2;// 进入下一步,跳过第三步
		}
		// 发送逻辑，切换25.6k
		if(step2_send_time < 1)
		{
			main_link_send = 21;
			step2_send_time ++;
		}
		else
		{
			//发送后开始计时
			step2_rec_count++;
		}

		// 未发送完之前先一直发送
		if(main_link_send!=0)
		{
			set_blk_ccc_kkl_000(uav_index, &main_link_send);
		}
		if(step2_rec_count > 25)
		{
			// 本步骤标志量清零
			step2_send_time = 0;
			step2_rec_count = 0;

			// 步骤标志清零
			free_step = 0;

			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 发送拒绝交接指令
			blk_ccc_kdl_010.ans = 1;
			send_blk_ccc_kdl_010();

			// 反馈交接状态,释放控制权-交接准备失败-天线模式切换失败
			hanover_state(uav_index,3,0,0x2);

		}

		return;
	}

	/***** 交接第三步:判断上下行锁定3秒，不满足则失败（允许最大连续5包失锁）  ***********/
//	static int step3_wait_count = 0;// 进入第三步先等待5拍
//	static int step3_lock_time = 0; // 锁定三秒判定
//	static int step3_break_count = 0; // 判断是否累计5包失锁，是则失败
//	if(free_step == 2)
//	{
//		// 清空第上步骤标志量
//		step2_send_time = 0;
//		step2_rec_count = 0;
//
//		// 等待5拍
//		if(step3_wait_count < 5)
//		{
//			step3_wait_count++;
//			return;
//		}
//
//		int tem_change_success = 0;
//		for(int i =0; i<UAV_MAX_NUM; i++)
//		{
//			// 根据编队信息找到飞机id，根据飞机id找相应的无人机链路信息
//			if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
//			{
//				if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].IfNoHeliCon == 1) //lock？
//				{
//					tem_change_success += 1;
//					break;
//				}
//			}
//		}
//
//		for(int i =0; i<UAV_MAX_NUM; i++)
//		{
//
//			if(blk_kkl_ccc_007.uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
//			{
//				if(blk_kkl_ccc_007.uav_cl_status_info[i].IfNoHeliCon == 1) //lock？
//				{
//					tem_change_success += 1;
//					break;
//				}
//			}
//
//		}
//
//		if(tem_change_success == 2)// 说明06、07都满足
//		{
//			step3_lock_time++;// 计时累加
//		}
//		else
//		{
//			step3_break_count++; // 接收次数增加（最多等5拍）
//		}
//
//
//		// 累计失锁5包则失败
//		if(step3_break_count>5)
//		{
//			// 本步骤标志量清零
//			step3_wait_count = 0;
//			step3_lock_time = 0;
//			step3_break_count = 0;
//
//			// 步骤标志清零
//			free_step = 0;
//
//			// 本机交接类别清零
//			*p_take_over_flag = 0;
//
//			// 清空本次交接指令
//			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));
//
//			// 发送拒绝交接指令
//			blk_ccc_kdl_010.ans = 1;
//			send_blk_ccc_kdl_010();
//
//			// 反馈交接状态,释放控制权-交接准备失败-链路状态不稳定
//			hanover_state(uav_index,3,0,0x3);
//		}
//
//		// 累计锁定3秒
//		if(step3_lock_time>60)
//		{
//			free_step++;// 进入下一步
//		}
//
//		return;
//	}

	/***** 交接第四步:指控给空地链发送同意交接指令  ***********/
	if(free_step == 3)
	{
		// 清空第上步骤标志量
//		step3_wait_count = 0;
//		step3_lock_time = 0;
//		step3_break_count = 0;

		step2_send_time = 0;
		step2_rec_count = 0;

		// 发送同意交接指令
		blk_ccc_kdl_010.ans = 0;
		send_blk_ccc_kdl_010();

		free_step++;
	}



	/***** 交接第五步:发送无人机交接参数 ccc_kkl_000  ***********/
	static int step5_send_time = 0; // 发送1次判定
	static int step5_rec_count = 0; // 判断是否切换成功,连续判断5包
	if(free_step == 4)
	{
		// 清空第上步骤标志量
		// 上步无

		//发送之前判断上行锁定或下行锁定，连续15s不锁定就直接失败
		static int lock_cnt_5 = 0;
		//连续十五秒失锁失败
		if(lock_cnt_5 > 20*15)
		{
			// 本步骤标志量清零
			step5_send_time = 0;
			step5_rec_count = 0;

			// 步骤标志清零
			free_step = 0;

			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 发送拒绝交接指令
			blk_ccc_kdl_010.ans = 1;
			send_blk_ccc_kdl_010();

			// 反馈交接状态,释放控制权-交接准备失败-天线模式切换失败
			hanover_state(uav_index,3,0,0x2);
		}
		if(up_down_lock(uav_index) == 0 && step5_send_time == 0)
		{
			lock_cnt_5++;
			return;
		}
		else
		{
			//上下行锁定，清空超时标志位，进入下一个判断
			lock_cnt_5 = 0;
		}

		if(step5_send_time < 1) // 发送逻辑
		{
			step5_send_time ++;
			main_link_send = 1;
		}

		// 未发送完之前先一直发送
		if(main_link_send!=0)
		{
			set_blk_ccc_kkl_000(uav_index, &main_link_send);
		}
		else
		{
			free_step++;// 发送完成进入下一步
		}

		return;
	}


	/***** 交接第六步:发送遥控指令帧启动交接指令（权交接）  ***********/
	static int step6_send_time = 0; // 发送1次判定 // 最多几次需确认？
	static int step6_rec_count = 0; // 判断是否切换成功,连续判断5包
	if(free_step == 5)
	{
		// 清空第上步骤标志量
		step5_send_time = 0;
		step5_rec_count = 0;


		if(step6_send_time < 3) // 发送逻辑，发送三遍，每遍发送三次，等待二十拍
		{
			if(step6_rec_count < 3) // 每次为0时发送，然后等待5包
			{
				// 释放控制权
				fc_order_send(uav_index, 1);
			}


			// 判断遥测帧
			if(s4D_frame_04[uav_index] == 1)
			{
				free_step++;// 进入下一步
			}
			else
			{
				step6_rec_count++; // 接收次数增加（最多等5拍）

				if(step6_rec_count>23)
				{
					step6_rec_count = 0;// 退出本次等待
					step6_send_time++;// 发送次数自增
				}
			}

		}
		else
		{
			// 本步骤标志量清零
			step6_send_time = 0;
			step6_rec_count = 0;

			// 步骤标志清零
			free_step = 0;

			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 反馈交接状态,释放控制权-控制权交接失败-无人机退回原来参数
			hanover_state(uav_index,3,0,0x5);

		}

		return;
	}


	/***** 交接第七步:发送主链交接启动指令  ***********/
	static int step7_send_time = 0; // 发送1次判定 // 最多几次需确认？
	static int step7_rec_count = 0; // 判断是否切换成功,连续判断5包
	if(free_step == 6)
	{
		// 清空第上步骤标志量
		step6_send_time = 0;
		step6_rec_count = 0;


		//发送之前判断上行锁定或下行锁定，连续15s不锁定就直接失败
		static int lock_cnt_7 = 0;
		//连续十五秒失锁失败
		if(lock_cnt_7 > 20*15)
		{
			// 本步骤标志量清零
			step7_send_time = 0;
			step7_rec_count = 0;

			// 步骤标志清零
			free_step = 0;

			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 发送拒绝交接指令
			blk_ccc_kdl_010.ans = 1;
			send_blk_ccc_kdl_010();

			// 反馈交接状态,释放控制权-交接准备失败-天线模式切换失败
			hanover_state(uav_index,3,0,0x2);
		}
		if(up_down_lock(uav_index) == 0 && step7_send_time == 0)
		{
			lock_cnt_7++;
			return;
		}
		else
		{
			//上下行锁定，清空超时标志位，进入下一个判断
			lock_cnt_7 = 0;
		}


		if(step7_send_time < 1) // 发送逻辑
		{
			step7_send_time++;
			main_link_send = 31;
		}
		else
		{
			step7_rec_count++;
		}
		// 未发送完之前先一直发送
		if(main_link_send!=0)
		{
			set_blk_ccc_kkl_000(uav_index, &main_link_send);
		}
		// 判断无控权则成功
		if(formationId[uav_index].isControl == 0)
		{
			free_step++;// 进入下一步
		}

		//等待180秒，超时失败
		if(step7_rec_count > 20*180)
		{
			// 本步骤标志量清零
			step7_send_time = 0;
			step7_rec_count = 0;

			// 步骤标志清零
			free_step = 0;

			// 本机交接类别清零
			*p_take_over_flag = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			// 反馈交接状态,释放控制权-控制权交接失败-无人机退回原来参数
			hanover_state(uav_index,3,0,0x5);
		}


		return;
	}


	/***** 交接第八步:交接成功  ***********/
	if(free_step == 7)
	{
		// 清空第上步骤标志量
		step7_send_time = 0;
		step7_rec_count = 0;

		// 步骤标志清零
		free_step = 0;

		// 本机交接类别清零
		*p_take_over_flag = 0;

		// 清空本次交接指令
		memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));


		// 发送成功指令给ofp,释放控制权-控制权交接成功
		hanover_state(uav_index,2,0,0x6);
	}


#endif
}

int up_down_lock(unsigned int uav_index)
{
	// 判断锁定
	int lock_cnt = 0;
	for(int i =0; i<UAV_MAX_NUM; i++)
	{
		// 根据编队信息找到飞机id，根据飞机id找相应的无人机链路信息
		if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
		{
			if(blk_kkl_ccc_006.CR_uav_cl_status_info[i].IfNoHeliCon == 1) //lock？
			{
				lock_cnt += 1;
				break;
			}
		}
	}

	for(int i =0; i<UAV_MAX_NUM; i++)
	{

		if(blk_kkl_ccc_007.uav_cl_status_info[i].NoHeliID == formationId[uav_index].planeId)
		{
			if(blk_kkl_ccc_007.uav_cl_status_info[i].IfNoHeliCon == 1) //lock？
			{
				lock_cnt += 1;
				break;
			}
		}
	}

	if(lock_cnt == 2)
	{
		return 1;
	}

	return 0;
}

//无人机控制权交接-手动释放控制权
// uav_index:无人机序号
// p_hand_free_flag: 手动释放步骤（可以外部传入控制步骤，也可以内部自增控制步骤）
void UAV_Takeover_Process_free_hand(int uav_index, unsigned int* p_hand_free_step)
{
#if 0//_PC_SIMULATION_



#else

	static unsigned int main_link_send = 0; // 是否发送3包主链交接指令。0：不需要发送；1：C链交接成功确认；2：启动主链转移（释放主链）

	if(*p_hand_free_step==0)
	{
		return;
	}

	//    if(uav_index<0 || uav_index > UAV_MAX_NUM-1)
	//        return;


	/***** 交接第一步:发送遥控指令帧启动交接指令（权交接）  ***********/
	static int step1_send_time = 0; // 发送1次判定 // 最多几次需确认？
	static int step1_rec_count = 0; // 判断是否切换成功,连续判断5包
	if(*p_hand_free_step == 1)
	{
		// 清空第上步骤标志量
		// 无


		if(step1_send_time < 1) // 发送逻辑
		{
			//            if(step1_rec_count < 3) // 每次为0时发送，然后等待5包
			{
				// 释放控制权
				fc_order_send(uav_index, 0);
			}


			// 判断遥测帧
			if(s4D_frame_04[uav_index] == 1)
			{
				*p_hand_free_step += 1;// 进入下一步
			}
			else
			{
				step1_rec_count++; // 接收次数增加（最多等5拍）

				if(step1_rec_count>200)
				{
					step1_rec_count = 0;// 退出本次等待
					step1_send_time++;// 发送次数自增
				}
			}

		}
		else
		{
			// 本步骤标志量清零
			step1_send_time = 0;
			step1_rec_count = 0;

			// 步骤标志清零
			*p_hand_free_step = 0;

			// 清空本次交接指令
			memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

			//手动释放-开始交接失败，飞控未响应
			hanover_state(uav_index,3,0,0x16);
		}

		return;
	}


	//210秒飞控倒计时
	static int timeout_210 = 0;
	//飞控交接210秒超时开始计时，标志位
	static int flag_210 = 0;
	//100秒主链倒计时
	static int timeout_100 = 0;
	//主链100秒超时开始计时，标志位
	static int flag_100 = 0;
	/***** 交接第二步:发送blk_ccc_ofp_003反馈数据  ***********/
	if(*p_hand_free_step == 2)
	{
		// 清空第上步骤标志量
		step1_send_time = 0;
		step1_rec_count = 0;

		// 步骤标志清零
		*p_hand_free_step = 0;

		// 反馈交接状态,手动释放-开始控制权交接
		hanover_state(uav_index,1,0,0xd);

		//210秒飞控倒计时开始计时
		flag_210 = 1;
		//100秒主链超时开始计时
		flag_100 = 1;
	}

	/***** 交接第三步:发送主链交接指令  ***********/
	static int step3_main_link = 1;
	if(*p_hand_free_step == 3)
	{
		// 清空第上步骤标志量

		//清空100s计时标志位，和拍数
		timeout_100 = 0;
		flag_100 = 0;


		if(step3_main_link)
		{
			main_link_send = 31;
			// 反馈交接状态,手动释放-正在释放控制权
			hanover_state(uav_index,1,0,0x10);
			step3_main_link = 0;
		}


		// 未发送完之前先一直发送
		if(main_link_send!=0)
		{
			set_blk_ccc_kkl_000(uav_index, &main_link_send);
			return;
		}

		// 判断无控权则成功
		if(formationId[uav_index].isControl == 0)
		{
			*p_hand_free_step += 1;// 进入下一步
		}
		else
		{
			//超过飞控210秒权交接时限,改为60s
			if(timeout_210 > 1200)
			{
				//收到明确的遥测，控制权还在本机
				if(formationId[uav_index].station_address == MANNED_ID)
				{
					// 本步骤标志量重置
					step3_main_link = 1;

					// 步骤标志清零
					*p_hand_free_step = 0;


					// 清空本次交接指令
					memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

					// 反馈交接状态,手动释放-控制权交接失败
					hanover_state(uav_index,3,0,0xf);

					//清空210s飞控计时标志位，和拍数
					timeout_210 = 0;
					flag_210 = 0;
				}
				//加上40s断链空间,控制权还在本机上，交接超时
				else if((timeout_210 > 1200 + 800))
				{
					// 本步骤标志量重置
					step3_main_link = 1;

					// 步骤标志清零
					*p_hand_free_step = 0;


					// 清空本次交接指令
					memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

					//手动释放-控制权交接失败，无人机未响应
					hanover_state(uav_index,3,0,0x17);

					//清空210s飞控计时标志位，和拍数
					timeout_210 = 0;
					flag_210 = 0;
				}

			}
		}

	}


	/***** 交接第四步:交接成功  ***********/
	if(*p_hand_free_step == 4)
	{
		// 重置第上步骤标志量
		step3_main_link = 1;

		// 步骤标志清零
		*p_hand_free_step = 0;


		// 清空本次交接指令
		memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));


		// 发送成功指令给ofp,手动释放-控制权交接成功
		hanover_state(uav_index,2,0,0xe);

		//清空210s飞控计时标志位，和拍数
		timeout_210 = 0;
		flag_210 = 0;
	}

	//210秒飞控倒计时开始计时
	if(flag_210)
	{
		timeout_210 ++;
	}
	//100秒主链倒计时
	if(flag_100)
	{
		timeout_100 ++;
		//超时100s
		if(timeout_100 > 2000)
		{
			//手动释放-操作超时，主链交接需要在100s内完成
			hanover_state(uav_index,3,0,0x15);

			//清空标志位
			timeout_100 = 0;
			flag_100 = 0;
			timeout_210 = 0;
			flag_210 = 0;

			// 步骤标志清零
			*p_hand_free_step = 0;
		}
	}



#endif
}

void UAV_Takeover_Process_free_hand_test(int uav_index, unsigned int* p_hand_free_step)
{

	if(*p_hand_free_step==0)
	{
		return;
	}

	//    if(uav_index<0 || uav_index > UAV_MAX_NUM-1)
	//        return;


	/***** 交接第一步:发送遥控指令帧启动交接指令（权交接）  ***********/
	static int step1_send_time = 0; // 发送1次判定 // 最多几次需确认？
	static int step1_rec_count = 0; // 判断是否切换成功,连续判断5包
	if(*p_hand_free_step == 1)
	{
		// 清空第上步骤标志量
		// 无
		if(step1_rec_count%5 == 0) // 每次为0时发送，然后等待5包
		{
			// 释放控制权
			fc_order_send(uav_index, 0);
		}

		// 判断遥测帧
		if(s4D_frame_04[uav_index] == 1)
		{
			*p_hand_free_step += 1;// 进入下一步
		}
		else
		{
			step1_rec_count++; // 接收次数增加（最多等5拍）

			if(step1_rec_count>200)
			{
				step1_rec_count = 0;// 退出本次等待
			}
		}
	}

     /***** 交接第二步:发送blk_ccc_ofp_003反馈数据  ***********/
	else if(*p_hand_free_step == 2)
    {
	// 清空第上步骤标志量
	step1_send_time = 0;
	step1_rec_count = 0;

	// 步骤标志清零
	*p_hand_free_step++;

	// 反馈交接状态,手动释放-开始控制权交接
	hanover_state(uav_index,1,0,0xd);
     }


     /***** 交接第三步:交接成功  ***********/
     if(*p_hand_free_step == 3)
     {

	   // 步骤标志清零
	   *p_hand_free_step = 0;

	   // 清空本次交接指令
	   memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));

	   // 发送成功指令给ofp,手动释放-控制权交接成功
	   hanover_state(uav_index,2,0,0xe);
      }
}

//无人机控制权交接
void UAV_Takeover_Process()
{
#if _PC_SIMULATION_
	/**数字环境控制权交接*/

	// 标志量
	static unsigned int take_over_flag = 0;  // 交接过程标志量。0：未在交接；1：在申请控制权；2：在释放控制权
	static int uav_over_index = -1;

	//收到062a0e交接控制指令
	message_size = sizeof(BLK_OFP_CCC_014);
	Receive_Message(DDSTables.DPU_CCC_2.niConnectionId, 0, &transaction_id, &blk_ofp_ccc_014, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		//收到控制指令，处理无人机的交接指令
		for(int uav_index = 0; uav_index < UAV_MAX_NUM; uav_index++)
		{
			//主动申请控制权
			if(blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].Control_type == 1)
			{
				//收到主动申请控制权指令
				take_over_flag = 1;
				uav_over_index = uav_index;
				plan_task_type = 16;
				init_blk_ccc_ofp_036(3,2,plan_task_type);
			}
			//主动释放控制权
			else if(blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].Control_type == 2)
			{
				//收到主动释放控制权指令
				take_over_flag = 2;
				uav_over_index = uav_index;
				plan_task_type = 17;
				init_blk_ccc_ofp_036(3,2,plan_task_type);
			}
			//同意释放控制权
			else if(blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].Control_type == 3)
			{
				//收到同意释放控制权指令
				take_over_flag = 2;
				uav_over_index = uav_index;
				blk_ccc_kdl_010.ans = 0;
				send_blk_ccc_kdl_010();
			}
			//拒绝
			else if(blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].Control_type == 4)
			{
				blk_ccc_kdl_010.ans = 1;
				send_blk_ccc_kdl_010();
				//给DPU发控制权交接状态
				hanover_state(3,0);

				//完成交接则重置blk_ofp_ccc_014
				memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));
				return;
			}
		}
	}

	//请求控制权
	UAV_Takeover_Process_get(uav_over_index, &take_over_flag);
	//释放控制权
	UAV_Takeover_Process_free(uav_over_index, &take_over_flag);


#else

	/**半物理环境控制权交接*/

	// 标志量
	static unsigned int take_over_flag = 0;  // 交接过程标志量。0：未在交接；1：在申请控制权；2：在释放控制权
	static unsigned int uav_over_index = -1; // 交接飞机的序号（0-3）

	//收到062a0e交接控制指令
	message_size = sizeof(BLK_OFP_CCC_014);
	recv_dpu1_dpu2(DDSTables.DPU_CCC_2.niConnectionId,DDSTables.DPU2_CCC_2.niConnectionId,&blk_ofp_ccc_014,sizeof blk_ofp_ccc_014);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//		Receive_Message(DDSTables.PAD_CCC_014.niConnectionId, 0, &transaction_id, &blk_ofp_ccc_014, &message_type_id, &message_size, &enRetCode);
	}
	if(enRetCode == 0)
	{
		//收到控制指令，处理无人机的交接指令
		for(int uav_index = 0; uav_index < UAV_MAX_NUM; uav_index++)
		{
			//主动申请控制权
			if(blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].Control_type == 1)
			{
				//收到主动申请控制权指令
				take_over_flag = 1;
				//记录本次交接的无人机索引
				uav_over_index = uav_index;
				//给DPU发控制权交接状态,正在申请控制权
				hanover_state(uav_index,0,1,0x7);
			}
			//主动释放控制权
			else if(blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].Control_type == 2)
			{
				//收到主动释放控制权指令
				take_over_flag = 2;
				//记录本次交接的无人机索引
				uav_over_index = uav_index;
			}
			//同意释放控制权
			else if(blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].Control_type == 3)
			{
				//收到同意释放控制权指令
				take_over_flag = 2;
				//记录本次交接的无人机索引
				uav_over_index = uav_index;
			}
			//拒绝
			else if(blk_ofp_ccc_014.CR_Handover_UAV_Infos[uav_index].Control_type == 4)
			{
				// 给kdl应答
				blk_ccc_kdl_010.ans = 1;
				send_blk_ccc_kdl_010();

				//给DPU发控制权交接状态
				hanover_state(uav_index,3,0,0);

				//完成交接则重置blk_ofp_ccc_014
				memset(&blk_ofp_ccc_014, 0, sizeof(blk_ofp_ccc_014));
				return;
			}
		}
	}

	//请求控制权
	UAV_Takeover_Process_get(uav_over_index, &take_over_flag);
	//释放控制权
	UAV_Takeover_Process_free(uav_over_index, &take_over_flag);



	static unsigned int hand_get_step = 0;
	static unsigned int hand_free_step = 0;
	static unsigned int hand_get_step_test = 0;
	static unsigned int hand_free_step_test = 0;
	message_size = sizeof(BLK_OFP_CCC_044);
	recv_dpu1_dpu2(DDSTables.DPU_CCC_044.niConnectionId,DDSTables.DPU2_CCC_044.niConnectionId,&blk_ofp_ccc_044,sizeof blk_ofp_ccc_044);
	if(enRetCode == 0)
	{
		//手动释放,开始控制权交接
		if(blk_ofp_ccc_044.releaseCtrl == 1)
		{
			hand_free_step = 1;
		}
		//手动释放,主链转移
		else if(blk_ofp_ccc_044.releaseCtrl ==2)
		{
			hand_free_step = 3;
			//blk_ofp_ccc_014.CR_Handover_UAV_Infos[0].UAV_ID = CCC_DPU_data_3.drone_specific_informations[0].platform_num;
			blk_ofp_ccc_014.CR_Handover_UAV_Infos[blk_ofp_ccc_044.uav_num-1].UAV_ID = CCC_DPU_data_3.drone_specific_informations[blk_ofp_ccc_044.uav_num-1].platform_num;
		}
		//手动获取,手动获取控制权
		if(blk_ofp_ccc_044.getCtrl == 1)
		{
			hand_get_step = 1;
			//blk_ofp_ccc_014.CR_Handover_UAV_Infos[0].UAV_ID = CCC_DPU_data_3.drone_specific_informations[0].platform_num;
			blk_ofp_ccc_014.CR_Handover_UAV_Infos[blk_ofp_ccc_044.uav_num-1].UAV_ID = CCC_DPU_data_3.drone_specific_informations[blk_ofp_ccc_044.uav_num-1].platform_num;
		}
		else if(blk_ofp_ccc_044.getCtrl == 3)
		{
			hand_get_step_test = 1;
		}
		else if(blk_ofp_ccc_044.getCtrl == 4)
		{
			//释放控制权
			hand_free_step_test = 1;
		}
	}

	//手动请求控制权
	if(hand_get_step > 0)
	{
		UAV_Takeover_Process_get_hand(blk_ofp_ccc_044.uav_num-1, &hand_get_step);
	}
	//监听建链状态
	else if(hand_get_step == 0)
	{
		for(int uav = 0 ; uav < UAV_MAX_NUM ; uav ++)
		{
			UAV_Takeover_Process_get_hand(uav, &hand_get_step);
		}

	}

	//手动释放控制权
	UAV_Takeover_Process_free_hand(blk_ofp_ccc_044.uav_num-1, &hand_free_step);
//	//仿真环境测试接口
//	UAV_Takeover_Process_get_hand_test(blk_ofp_ccc_044.uav_num-1, &hand_get_step_test);
//
//	UAV_Takeover_Process_free_hand_test(blk_ofp_ccc_044.uav_num-1, &hand_free_step_test);

#endif
}

/*************************************** 无人机光电视频控制模块 ***************************************/
/*
DPU-CCC: DPU_CCC_data_14
CCC-KKL(CCC-GD): CCC_KKL_data_11 DPU_CCC_data_14_1
KKL-CCC(GD-CCC): KKL_CCC_data_21 CCC_DPU_data_22_1
CCC-DPU: CCC_DPU_data_22
 */
// 接收无人机光电视频MFD控制 DPU-CCC
void recv_DPU_CCC_MFD()
{

	// 无人机光电控制权设置
	message_size = RECV_MAX_SIZE;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_15.niConnectionId,DDSTables.DPU2_CCC_15.niConnectionId,&DPU_CCC_data_15,sizeof DPU_CCC_data_15);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//		Receive_Message(DDSTables.PAD_CCC_029.niConnectionId, 0, &transaction_id, dds_data_rece.dataA + 2, &message_type_id, &message_size, &enRetCode);
	}
	if (enRetCode == 0)
	{
		// 发送给任务-无人机光电控制权反馈
		CCC_DPM_data_12.ssds_control_feedback = DPU_CCC_data_15.uav_kongzhiquanshezhi;
		align_send_information(&CCC_DPM_data_12, sizeof(DPM_guangdian_kongzhi_feedback), 0);
		Send_Message(DDSTables.CCC_DPM_12.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	}

	static int send_count_gd = 0;
	int recv_mfd_ok = 0;
	// 前后舱都能控制光电，根据前舱综显指令判断接收哪边的控制指令更新
	if(DPU_CCC_data_15.uav_kongzhiquanshezhi == 2)//后舱任务发出
	{
		// 接收从任务发来的无人机光电视频MFD控制
		for(int guard = 0; guard < 64; guard++)
		{
		message_size = RECV_MAX_SIZE;
		Receive_Message(DDSTables.DPM_CCC_2.niConnectionId, 0, &transaction_id, &DPU_CCC_data_14, &message_type_id, &message_size, &enRetCode);
			if(enRetCode != 0)
			{
				break;
			}
			recv_mfd_ok = 1;
		}
	}
	else
	{
		// 接收从综显发来的无人机光电视频MFD控制
		uav_photoic_video_MFD latest_mfd;
		memset(&latest_mfd,0,sizeof(latest_mfd));
		for(int guard = 0; guard < 64; guard++)
		{
		message_size = RECV_MAX_SIZE;
			recv_dpu1_dpu2(DDSTables.DPU_CCC_14.niConnectionId,DDSTables.DPU2_CCC_14.niConnectionId,&latest_mfd,sizeof(latest_mfd));
		//收不到就收PAD new20250620
		if(enRetCode != 0)
			{
				break;
			}
			memcpy(&DPU_CCC_data_14,&latest_mfd,sizeof(DPU_CCC_data_14));
			recv_mfd_ok = 1;
		}
		if(recv_mfd_ok == 0)
		{
			for(int guard = 0; guard < 64; guard++)
			{
				message_size = RECV_MAX_SIZE;
			Receive_Message(DDSTables.PAD_CCC_028.niConnectionId, 0, &transaction_id, &DPU_CCC_data_14, &message_type_id, &message_size, &enRetCode);
				if(enRetCode != 0)
				{
					break;
		}
				recv_mfd_ok = 1;
	}
		}
	}

	// 接收成功，将收到的数据填入对应的结构体
	if (recv_mfd_ok == 1)
	{
		send_count_gd = PIU_CMD_RESEND_CYCLES;
	}
	if(send_count_gd > 0)
	{
		send_count_gd--;
		//找到无人机索引
		int uav_index = DPU_CCC_data_14.video_source_type - 2;
		// 切换无人机的视频源
		if(uav_index >= 0 && uav_index < UAV_MAX_NUM)
		{
		init_drone_information_Azhen(uav_index);         // 机载，更新任务遥控数据A帧
		}
	}
	//接收PIU的光电控制手柄信息
	char piu_buf[256];
	unsigned int piu_msg_size = 0;
	int piu_recv_ok = 0;
	memset(&blk_piu_ccc_006,0,sizeof(BLK_PIU_CCC_006));
	for(int piu_guard = 0; piu_guard < 64; piu_guard++)
	{
		message_size = sizeof(piu_buf);
	Receive_Message(DDSTables.PIU_CCC_0.niConnectionId, 0, &transaction_id, &piu_buf, &message_type_id, &message_size, &enRetCode);
		if(enRetCode != 0)
		{
			break;
		}
		piu_recv_ok = 1;
		piu_msg_size = message_size;
	}
	if(piu_recv_ok == 1)
	{
		//    	printf("dtms %d\n",message_size);
		int block_num = piu_msg_size / 4;
		int flag[4] = {0,0,0,0};
		for(int i = 0 ; i < block_num ; i ++)
		{
			//头
			if(piu_buf[3+i*4] == 128 && flag[0] == 0)
			{
				memcpy(&blk_piu_ccc_006.lable,&piu_buf[i*4],sizeof(LABLE));
				flag[0] = 1;
			}
			//按键,201八进制
			else if(piu_buf[3+i*4] == 129 && flag[1] == 0)
			{
				memcpy(&blk_piu_ccc_006.joy_key_01,&piu_buf[i*4],sizeof(JOY_KEY_01));
				if(blk_piu_ccc_006.joy_key_01.F_NOTE != 0)
				{
					//					printf("1 %d\n",blk_piu_ccc_006.joy_key_01.F_NOTE);
				}
				flag[1] = 1;
			}
			//方位,202八进制
			else if(piu_buf[3+i*4] == 65 && flag[2] == 0)
			{
				memcpy(&blk_piu_ccc_006.joy_key_02,&piu_buf[i*4],sizeof(JOY_KEY_02));
				if(blk_piu_ccc_006.joy_key_02.integer != 0)
				{
					//					printf("2 %d\n",blk_piu_ccc_006.joy_key_02.integer);
				}
				flag[2] = 1;
			}
			//俯仰，,203八进制
			else if(piu_buf[3+i*4] == 193 && flag[3] == 0)
			{
				memcpy(&blk_piu_ccc_006.joy_key_03,&piu_buf[i*4],sizeof(JOY_KEY_03));
				if(blk_piu_ccc_006.joy_key_03.integer != 0)
				{
					//					printf("3 %d\n",blk_piu_ccc_006.joy_key_03.integer);
				}
				flag[3] = 1;
			}
			//判断四个包是否都收到了
			int recv_ok = 0;
			for(int j = 0; j < 4 ;j++)
			{
				if(flag[j] != 1)
				{
					recv_ok = -1;
				}
			}
			//四个包都收到了就不收后续包
			if(recv_ok == 0)
			{
				break;
			}
		}
		//前舱控制
		if(blk_piu_ccc_006.joy_key_01.status == 0 && DPU_CCC_data_15.uav_kongzhiquanshezhi != 2)
		{
			int uav_index = DPU_CCC_data_14.video_source_type - 2;
			//TODO
			//            int uav_index = 0;
			if(uav_index >= 0)
			{
				//按键
				parse_blk_piu_ccc_006_key01(uav_index);

				//方位
				parse_blk_piu_ccc_006_key02(uav_index);

				//俯仰
				parse_blk_piu_ccc_006_key03(uav_index);
			}
		}

		//后舱控制
		if(blk_piu_ccc_006.joy_key_01.status == 1 && DPU_CCC_data_15.uav_kongzhiquanshezhi == 2)
		{
			int uav_index = DPU_CCC_data_14.video_source_type - 2;
			if(uav_index >= 0)
			{
				//按键
				parse_blk_piu_ccc_006_key01(uav_index);
				//方位
				parse_blk_piu_ccc_006_key02(uav_index);
				//俯仰
				parse_blk_piu_ccc_006_key03(uav_index);
			}
		}
	}
}
void parse_blk_piu_ccc_006_key01(int uav_index)
{
	//判断当前是红外还是电视
	if(DPU_CCC_data_14.video_params_setting_feedbacks.ir_pwr_ctrl == 0x01)//红外上电
	{
		//视场(变焦+)
		if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x02)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x98;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x01;
		}
		//视场(变焦-)
		else if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x03)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x98;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0xFF;
		}
		//调焦+
		else if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x04)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x9A;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x01;
		}
		//调焦-
		else if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x05)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x9A;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0xFF;
		}
		//长短焦切换
		else if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x0D)
		{
			//红长
			if(DPU_CCC_data_14.video_params_setting_feedbacks.type == 0x03)
			{
				//程制
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X10;
				//指令代码
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0;
				//参数
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x05;
			}
			//红短
			else if(DPU_CCC_data_14.video_params_setting_feedbacks.type == 0x04)
			{
				//程制
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X10;
				//指令代码
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0;
				//参数
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x07;
			}
		}
		//        // 空
		//        else if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x00)
		//        {
		//            //控制带参数
		//            CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0x00;
		//            //指令代码
		//            CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x00;
		//            //参数
		//            CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x00;
		//        }
	}
	else if(DPU_CCC_data_14.video_params_setting_feedbacks.ir_pwr_ctrl == 0x02 || DPU_CCC_data_14.video_params_setting_feedbacks.ir_pwr_ctrl == 0)//红外下电,默认电视
	{
		//视场(变焦+)
		if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x02)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x80;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x01;
		}
		//视场(变焦-)
		else if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x03)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x80;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0xFF;
		}
		//调焦+
		else if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x04)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x82;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x01;
		}
		//调焦-
		else if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x05)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x82;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0xFF;
		}
		//长短焦切换
		else if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x0D)
		{
			//电长
			if(DPU_CCC_data_14.video_params_setting_feedbacks.type == 0x01)
			{
				//程制
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X10;
				//指令代码
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0;
				//参数
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x01;
			}
			//电短
			else if(DPU_CCC_data_14.video_params_setting_feedbacks.type == 0x02)
			{
				//程制
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X10;
				//指令代码
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0;
				//参数
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x03;
			}
		}
		//        // 空
		//        else if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x00)
		//        {
		//            //控制带参数
		//            CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0x00;
		//            //指令代码
		//            CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x00;
		//            //参数
		//            CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x00;
		//        }
	}
	//收藏(归零)
	if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x0A)
	{
		//控制不带参数
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X20;
		//指令代码
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x34;
		//参数
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0;
	}
	//收藏(归零)
	if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x0A)
	{
		//控制不带参数
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X20;
		//指令代码
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x34;
		//参数
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0;
	}
	//当前目标号,默认为目标1
	static unsigned char target = 0;
	//目标+
	if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x0B)
	{
		target++;
		if(target >= 6)//6个目标
			target = 0;
		//控制带参数
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
		//指令代码
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x3A;
		//参数
		setBit(CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data, target ,1);
	}
	//目标-
	if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x0C)
	{
		target--;
		if(target >= 6)//6个目标
			target = 5;
		//控制带参数
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
		//指令代码
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x3A;
		//参数
		setBit(CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data, target ,1);
	}

	//黑/白热(红外正/负像切换)
	if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x0E)
	{
		//当前状态,默认正像
		static unsigned char change = 0;
		if(change == 0)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0xA0;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x01;
			change = 1;
		}
		else if(change == 1)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X30;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0xA0;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x00;
			change = 0;
		}
	}

	//手动(手控)/跟踪
	if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x10)
	{
		//当前状态,默认手控
		static unsigned char man_flo = 0;
		if(man_flo == 0)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X20;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x30;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0;
			man_flo = 1;
		}
		else if(man_flo == 1)
		{
			//控制带参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X20;
			//指令代码
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x42;
			//参数
			CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0;
			man_flo = 0;
		}
	}
	//    // 空
	//    if(blk_piu_ccc_006.joy_key_01.F_NOTE == 0x00)
	//    {
	//        //控制带参数
	//        CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0x00;
	//        //指令代码
	//        CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x00;
	//        //参数
	//        CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x00;
	//    }
}

void parse_blk_piu_ccc_006_key02(int uav_index)
{
	static short fangwei = 0;
	//消抖
	//    if(abs(blk_piu_ccc_006.joy_key_02.integer) < 100)
	//    {
	//        return;
	//    }
	//    fangwei += blk_piu_ccc_006.joy_key_02.integer /** (5/20/32768)*/;
	fangwei = blk_piu_ccc_006.joy_key_02.integer;
	int scaled_fangwei = (int)fangwei * PIU_VIEW_GAIN;
	CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.dangan_fangwei = clamp_int_to_short(scaled_fangwei);
}

void parse_blk_piu_ccc_006_key03(int uav_index)
{
	static short fuyang = 0;
	//消抖
	//    if(blk_piu_ccc_006.joy_key_03.integer < 100)
	//    {
	//        return;
	//    }
	//    fuyang += blk_piu_ccc_006.joy_key_03.integer /** (5/20/32768)*/;
	fuyang = blk_piu_ccc_006.joy_key_03.integer;
	int scaled_fuyang = (int)fuyang * PIU_VIEW_GAIN;
	CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.dangan_fuyang = clamp_int_to_short(scaled_fuyang);
}
/// 解析遥测数据帧子帧4D  uav_index:无人机序号（0-3）
void parseYaoCeZiZhen4D(int uav_index)
{
	// 同步码判断 待确认大小端转换是否正确
	if(s4D_frame.syn_code_0 != 0x55 || s4D_frame.syn_code_1 != 0xAA)
	{
		return;
	}

	// 遥调指令回报（如果发送的指令，由于校验未通过或者三判二未通过，则回报指令码是0xff）
	if(s4D_frame.zhiLingHuiBao == 0xFF)
	{
		return;
	}


	if(s4D_frame.zhiLingHuiBao == 0x04)
	{
		s4D_frame_04[uav_index] = 1;
	}
	else
	{
		s4D_frame_04[uav_index] = 0;
	}


	if(s4D_frame.zhiLingHuiBao == 0x24)
	{
		s4D_frame_24[uav_index] = 1;
	}
	else
	{
		s4D_frame_24[uav_index] = 0;
	}

	//航线装订
	if(s4D_frame.zhiLingHuiBao == 0x38)
	{
		//仅在发布/装订过程中记录回报，避免装订完成后误触发条件不满足
		if(uav_hl_confirm == 1 && uav_send[uav_index].send_ready == 1)
		{
			s4D_frame_38[uav_index] = s4D_frame.result.result;//00=执行成功 | 01=响应条件不满足执行失败 10=设置参数不合理执行失败 | 11=其他原因执行失败
		}
	}
	//航线切换
	if(s4D_frame.zhiLingHuiBao == 0x40)
	{
		//仅在航线切换流程中记录0x40回报，避免误触发
		int waiting_track_change = 0;
		if((uav_hl_confirm == 2 && send_area[uav_index] == 0) ||
		   (BDFX_double_status == 2 || BDFX_double_status == 3 || BDFX_status == 2 || BDFX_status == 3))
		{
			waiting_track_change = 1;
		}

		if(waiting_track_change == 1)
		{
			int prev_state = s4D_frame_40[uav_index];
			printf("0x40 ack uav=%d result=%d data0=%d hl=%d bdfx2=%d bdfx=%d send_area=%d prev=%d\n", uav_index, s4D_frame.result.result, (unsigned char)s4D_frame.data[0], uav_hl_confirm, BDFX_double_status, BDFX_status, send_area[uav_index], prev_state);
			if(s4D_frame.result.result == 0)
			{
				s4D_frame_40[uav_index] = 1;//00=执行成功 | 01=响应条件不满足执行失败 10=设置参数不合理执行失败 | 11=其他原因执行失败
			}
			else
			{
				if (prev_state == 1)
				{
					printf("0x40 ack ignore fail-after-success uav=%d result=%d data0=%d\n", uav_index, s4D_frame.result.result, (unsigned char)s4D_frame.data[0]);
				}
				else
				{
				s4D_frame_40[uav_index] = -1;//00=执行成功 | 01=响应条件不满足执行失败 10=设置参数不合理执行失败 | 11=其他原因执行失败
			}
		}
		}
	}
	//航线长度查询
	if(s4D_frame.zhiLingHuiBao == 0x41)
	{

		if(s4D_frame.result.result == 0)//00=执行成功 | 01=响应条件不满足执行失败 10=设置参数不合理执行失败 | 11=其他原因执行失败
		{
			if(s4D_frame.data[0] == uav_route[uav_index].route_number)
			{
				s4D_frame_41[uav_index] = s4D_frame.data[1];
			}

		}
		else
		{
			s4D_frame_41[uav_index] = -1;
		}

	}

	//航线查询
	if(s4D_frame.zhiLingHuiBao == 0x3c)
	{
		if(s4D_frame.result.result == 0)//00=执行成功 | 01=响应条件不满足执行失败 10=设置参数不合理执行失败 | 11=其他原因执行失败
		{
			if(s4D_frame.data[0] == uav_route[uav_index].route_number)
			{
				//根据航点数保存航点信息
				int point_index = s4D_frame.data[1] - 1;
				printf("point %d\n",point_index+1);
				//航路点编号
				blk_ccc_ofp_158.point[point_index].hld_idx = s4D_frame.data[1];
				//经纬高速度赋值
				int lat,lon;
				short height,speed;
				memcpy(&lon,&s4D_frame.data[2],sizeof(int));
				memcpy(&lat,&s4D_frame.data[6],sizeof(int));
				memcpy(&height,&s4D_frame.data[10],sizeof(short));
				memcpy(&speed,&s4D_frame.data[12],sizeof(short));
				blk_ccc_ofp_158.point[point_index].longitude = (double)lon* 180.0 / (pow(2,31) - 1);
				blk_ccc_ofp_158.point[point_index].latitude = (double)lat* 90.0 / (pow(2,31) - 1);
				blk_ccc_ofp_158.point[point_index].height = height * 0.2;
				blk_ccc_ofp_158.point[point_index].speed = speed * 0.1;
				//有效性
				blk_ccc_ofp_158.point[point_index].latitude_validity = 1;
				blk_ccc_ofp_158.point[point_index].validity_of_longitude = 1;
				blk_ccc_ofp_158.point[point_index].height_validity = 1;
				blk_ccc_ofp_158.point[point_index].speed_validity = 1;
				//特征字赋值
				if(s4D_frame.data[16] == 0x02)//顺时针圆心
				{
					blk_ccc_ofp_158.point[point_index].causality = 3;
				}
				else if(s4D_frame.data[16] == 0x03)//逆时针圆心
				{
					blk_ccc_ofp_158.point[point_index].causality = 4;
				}
				else if(s4D_frame.data[16] == 0x07)//机场航路点
				{
					blk_ccc_ofp_158.point[point_index].causality = 8;
				}
				else if(s4D_frame.data[16] == 0x08)//悬停
				{
					blk_ccc_ofp_158.point[point_index].causality = 9;
				}
				short rad,circle_time;
				memcpy(&rad,&s4D_frame.data[17],sizeof(short));
				memcpy(&circle_time,&s4D_frame.data[19],sizeof(short));
				if(rad > 0)
				{
					blk_ccc_ofp_158.point[point_index].standby_radius = rad;
					blk_ccc_ofp_158.point[point_index].standby_radius_valid_bit = 1;
				}
				blk_ccc_ofp_158.point[point_index].standby_time_lapsNumber_cycleNumber = circle_time;
				blk_ccc_ofp_158.point[point_index].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
				if(s4D_frame.data[23] == 0x04)//圈数
				{
					blk_ccc_ofp_158.point[point_index].standby_type = 2;
				}
				else if(s4D_frame.data[23] == 0)//时间
				{
					blk_ccc_ofp_158.point[point_index].standby_type = 1;
				}
			}
		}
	}

		//调高
	if(s4D_frame.zhiLingHuiBao == 0x12)
	{
		if(s4D_frame.result.result == 0)
		{
			s4D_frame_12[uav_index] = 1;//00=执行成功 | 01=响应条件不满足执行失败 10=设置参数不合理执行失败 | 11=其他原因执行失败
		}
		else
		{
			s4D_frame_12[uav_index] = -1;//00=执行成功 | 01=响应条件不满足执行失败 10=设置参数不合理执行失败 | 11=其他原因执行失败
		}
	}
	//调速
	if(s4D_frame.zhiLingHuiBao == 0x0c)
	{
		if(s4D_frame.result.result == 0)
		{
			s4D_frame_0c[uav_index] = 1;//00=执行成功 | 01=响应条件不满足执行失败 10=设置参数不合理执行失败 | 11=其他原因执行失败
		}
		else
		{
			s4D_frame_0c[uav_index] = -1;//00=执行成功 | 01=响应条件不满足执行失败 10=设置参数不合理执行失败 | 11=其他原因执行失败
		}
	}

}
void parseYaoCeZiZhen82(int uav_index)
{
	//预控
	if(s82_frame.inst_ret == 0x08)
	{
		if(s82_frame.res.remote_com == 0)
		{
			printf("YK success\n");
		}
		else
		{
			printf("YK erro\n");
		}
	}

	//返航反馈
	if(s82_frame.inst_ret == 0x30)
	{
		if(s82_frame.res.remote_com == 0)
		{
			printf("FH success\n");
			s82_frame_30[uav_index] = 1;//00=执行成功 | 01=响应条件不满足执行失败 10=设置参数不合理执行失败 | 11=其他原因执行失败
		}
		else
		{
			printf("FH erro\n");
			s82_frame_30[uav_index] = -1;//00=执行成功 | 01=响应条件不满足执行失败 10=设置参数不合理执行失败 | 11=其他原因执行失败
		}
	}

	// 新增待飞距离解析（用在下阶段提示）
	g_toCurPointDis[uav_index] = s82_frame.radius * 5;

}
void parseYaoCeZiZhen3A(int uav_index)
{
	//周期获取无人机当前航线和航点
	uav_route[uav_index].route_number = s3A_frame.route_number;
	uav_route[uav_index].waypoint_number = s3A_frame.waypoint_number;
}

/// 数字环境下的心跳和版本上报处理
void sendSimulateHeartBeat()
{
#if _PC_SIMULATION_
	/*****心跳上报***********************/
	FeedBackHeartBeatInfo heartBeatInfo;
	// 获取上电时间
	static int64_t onLineTime = 0;
	if(!onLineTime)
	{

		onLineTime = QDateTime::currentSecsSinceEpoch();

	}


	heartBeatInfo.timeStamp = (UINT32)(QDateTime::currentSecsSinceEpoch() - onLineTime);					//时间戳	（单位：s；范围：0~2^32-1）


	heartBeatInfo.systemID = CoodinatedConmandSystemID;					//系统的ID
	memcpy(heartBeatInfo.systemIP, g_curAppIP, 32);				//系统IP
	heartBeatInfo.controlCommond = SimulatorStart;				//仿真状态反馈
	heartBeatInfo.status = 0x01;						//系统的状况（0:异常,1:正常）
	strcpy(heartBeatInfo.version, "2.0.0.1\0");				//版本号 2.0.0.1
	strcpy_s(heartBeatInfo.versionUpdateTime, "1744185939"); // 1970至今的秒数
	align_send_information(&heartBeatInfo, sizeof(heartBeatInfo), 0);
	Send_Message(DDSTables.CCC_SIMCONTROL_0.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);


	/*****仿真进程控制命令获取*******************/
	SimulatorControl simulatorControl;

	Receive_Message(DDSTables.CCC_SIMCONTROL_2.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		memcpy(&(simulatorControl),dds_data_rece.dataA,sizeof(simulatorControl));
		if(simulatorControl.stationType == AllSystem || simulatorControl.stationType == CoodinatedConmandSystem)
		{
			/**** 反馈控制指令*************************/
			FeedBackControlInfo feedBackControlInfo;
			memset(&feedBackControlInfo, 0, sizeof(FeedBackControlInfo));
			feedBackControlInfo.systemID = CoodinatedConmandSystemID;
			switch(simulatorControl.controlCommand)
			{
			case SimulatorInit:						//初始化
				feedBackControlInfo.initResult = 0x1;//（0:NA,1:成功,2:失败）
				break;
			case SimulatorStart:							//开始
				feedBackControlInfo.startResult = 0x1;//（0:NA,1:成功,2:失败）
				break;
			case SimulatorPause:							//暂停
				feedBackControlInfo.pauseResult = 0x1;//（0:NA,1:成功,2:失败）
				break;
			case SimulatorContinue:						//继续
				feedBackControlInfo.continueResult = 0x1;//（0:NA,1:成功,2:失败）
				break;
			case SimulatorStop:							//停止
				feedBackControlInfo.stopResult = 0x1;//（0:NA,1:成功,2:失败）
				break;
			default:
				break;
			}
			feedBackControlInfo.controlCommond = simulatorControl.controlCommand;
			align_send_information(&feedBackControlInfo, sizeof(feedBackControlInfo), 0);
			Send_Message(DDSTables.CCC_SIMCONTROL_1.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
		}
	}

#endif
}

// 将光电视频控制反馈发送给综显 CCC-DPU
void send_uav_photoelectric_video_ctrl_feedback()
{

	// 新增周期给mids上报当前控制的无人机视频源所在的无人机编号(当前实现基于（因为当前固定是1005和1006))
	char uav_id = 0;
	if(DPU_CCC_data_14.video_source_type == 2)
	{
		uav_id = 5;
	}
	else if(DPU_CCC_data_14.video_source_type == 3)
	{
		uav_id = 6;
	}
	else
	{
		uav_id = 0;
	}
	data_length = sizeof(uav_id);
	Send_Message(DDSTables.CCC_MIDS_001.niConnectionId, 0, &transaction_id, &uav_id, &message_type_id, data_length, &enRetCode);


	// 视频源类型（0H=NA;1H=有人机视频;2H=无人机1视频;3H=无人机2视频;4H=无人机3视频;5H=无人机4视频）
	for(int i = 0 ; i < UAV_MAX_NUM ; i ++)
	{
		if(i != (DPU_CCC_data_14.video_source_type-2))
		{
			continue;
		}
		CCC_DPU_data_22[i].video_source_type = DPU_CCC_data_14.video_source_type;


		align_send_information(&CCC_DPU_data_22[i], sizeof(DPU_guangdina_video), 0);
		Send_Message(DDSTables.CCC_DPU_22.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);

		//发送给pad new20250620
		Send_Message(DDSTables.CCC_PAD_042.niConnectionId,0,&transaction_id, send_array.dataA,&message_type_id, data_length, &enRetCode);


		// 给任务系统发送光电视频控制反馈
		Send_Message(DDSTables.CCC_DPM_13.niConnectionId, 0, &transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
		//printf("|DTMS|ESO rad[%d]\n",CCC_DPU_data_22.video_param_setting_feedbacks.ir_pwr_fb);

	}


	//	//    CCC_DPU_data_22.video_param_setting_feedbacks.type 在kkl_ccc返回状态信息处解析时已赋值
	//	// 当控制指令是0（NA）时，反馈指令保持不变反馈。
	//	int lock_scan_switch_change = 0;// 锁定,扫描状态切换标志量(0,状态未切换；1，扫描切换成锁定状态;2,锁定切换成扫描状态；)
	//	if(DPU_CCC_data_14.video_params_setting_feedbacks.lock_scan_switch != 0)
	//	{
	//		// 有切换
	//		if(CCC_DPU_data_22.video_param_setting_feedbacks.lock_scan_switch != DPU_CCC_data_14.video_params_setting_feedbacks.lock_scan_switch)
	//		{
	//			if(DPU_CCC_data_14.video_params_setting_feedbacks.lock_scan_switch == 1)
	//			{
	//				lock_scan_switch_change = 1;
	//			}
	//			else
	//			{
	//				lock_scan_switch_change = 2;
	//			}
	//		}
	//		CCC_DPU_data_22.video_param_setting_feedbacks.lock_scan_switch = DPU_CCC_data_14.video_params_setting_feedbacks.lock_scan_switch;
	//	}
	//
	//	// 切换成锁定状态，扫描状态清空
	//	if(lock_scan_switch_change == 1)
	//	{
	//		CCC_DPU_data_22.video_param_setting_feedbacks.scan_param = 0;
	//	}
	//	else
	//	{
	//		// 控制不为0，则反馈更新
	//		if(DPU_CCC_data_14.video_params_setting_feedbacks.scan_param != 0)
	//		{
	//			CCC_DPU_data_22.video_param_setting_feedbacks.scan_param = DPU_CCC_data_14.video_params_setting_feedbacks.scan_param;
	//		}
	//	}
	//
	//	// 切换成扫描状态，锁定状态需要清空
	//	if(lock_scan_switch_change == 2)
	//	{
	//		CCC_DPU_data_22.video_param_setting_feedbacks.lock_param = 0;
	//	}
	//	else
	//	{
	//		// 控制不为0，则反馈更新
	//		if(DPU_CCC_data_14.video_params_setting_feedbacks.lock_param != 0)
	//		{
	//			CCC_DPU_data_22.video_param_setting_feedbacks.lock_param = DPU_CCC_data_14.video_params_setting_feedbacks.lock_param;
	//		}
	//	}


}

// 初始化任务遥控数据A帧（CCC_UAV_Azhens），其作为任务控制指令帧封装在遥控指令帧中（CCC_KKL_data_11.task_control_frame_data） CCC-KKL
void init_drone_information_Azhen(int uav_index)
{

	// 光电遥控指令 (机载)
	CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x00;
	CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x00;
	// 红外上电控制 0=NA;1=上电;2=下电
	if (DPU_CCC_data_14.video_params_setting_feedbacks.ir_pwr_ctrl == 0x01)
	{ // 红外上电
		// 指令类型: 0X20 控制状态
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X20;
		// 光电控制命令 红外电源开
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x90;
	}
	else if (DPU_CCC_data_14.video_params_setting_feedbacks.ir_pwr_ctrl == 0x02)
	{ // 红外下电
		// 指令类型: 0X20 控制状态
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X20;
		// 光电控制命令 红外电源关
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.guangdian_control_mingling = 0x92;
	}
	else
	{ // 电视状态
		// 指令类型: 0X10 程控状态
		CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_class = 0X10;
		// 指令数据
		if (DPU_CCC_data_14.video_params_setting_feedbacks.type != 0x00)
		{
			// 电视/红外/长焦/短焦切换 0H=NA;1H=电视长焦;2H=电视短焦;3H=红外长焦;4H=红外短焦
			if (DPU_CCC_data_14.video_params_setting_feedbacks.type == 0x01)
			{
				// 0x03 电视长焦
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x03;
			}
			else if (DPU_CCC_data_14.video_params_setting_feedbacks.type == 0x02)
			{
				// 0x01 电视短焦
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x01;
			}
			else if (DPU_CCC_data_14.video_params_setting_feedbacks.type == 0x03)
			{
				// 0x07 红外长焦
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x07;
			}
			else if (DPU_CCC_data_14.video_params_setting_feedbacks.type == 0x04)
			{
				// 0x05 红外短焦
				CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x05;
			}
		}
		else
		{
			// 锁定/扫描切换 0H=锁定;1H=扫描
			unsigned char lock_scan = DPU_CCC_data_14.video_params_setting_feedbacks.lock_scan_switch;
			if (lock_scan == 0x01)
			{
				// 锁定参数 0H=NA;1H=垂直下视锁定;2H=前向锁定;3H=右向锁定;4H=后向锁定;5H=左侧锁定
				unsigned char lock_param = DPU_CCC_data_14.video_params_setting_feedbacks.lock_param;
				if (lock_param == 0x01)
				{
					// 垂直下视锁定 0x41
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x41;
				}
				else if (lock_param == 0x02)
				{
					// 前向锁定 0x43
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x43;
				}
				else if (lock_param == 0x03)
				{
					// 右向锁定 0x45
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x45;
				}
				else if (lock_param == 0x04)
				{
					// 后向锁定 0x47
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x47;
				}
				else if (lock_param == 0x05)
				{
					// 左向锁定 0x49
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x49;
				}
			}
			else
			{
				// 扫描参数 0H=NA;1H=前向方位扫描;2H=右侧方位扫描;3H=后向方位扫描;4H=左侧方位扫描;5h=前向俯仰扫描;6H=右侧俯仰扫描;7H=后向俯仰扫描;8H=左侧俯仰扫描
				unsigned char scan_param = DPU_CCC_data_14.video_params_setting_feedbacks.scan_param;
				if (scan_param == 0x01)
				{
					// 前向方位扫描 0x11
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x11;
				}
				else if (scan_param == 0x02)
				{
					// 右侧方位扫描 0x13
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x13;
				}
				else if (scan_param == 0x03)
				{
					// 后向方位扫描 0x15
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x15;
				}
				else if (scan_param == 0x04)
				{
					// 左侧方位扫描 0x17
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x17;
				}
				else if (scan_param == 0x05)
				{
					// 前向俯仰扫描 0x19
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x19;
				}
				else if (scan_param == 0x06)
				{
					// 右侧俯仰扫描 0x1B
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x1B;
				}
				else if (scan_param == 0x07)
				{
					// 后向俯仰扫描 0x1D
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x1D;
				}
				else if (scan_param == 0x08)
				{
					// 左侧俯仰扫描 0x1E
					CCC_UAV_Azhens[uav_index].guangdiankongzhizhilings.zhiling_data = 0x1E;
				}
			}
		}
	}

	//	//ToDo:暂未确定消息来源
	//	//为综合任务管理系统赋值
	//	char yaotiaobianma[3];
	//	memcpy(yaotiaobianma,CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling,3);//前三位为遥调编码
	//	switch (yaotiaobianma[0]) {
	//	//多模式雷达
	//	case (char)0X01:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_0,sizeof (CCC_UAV_A_data1_0));
	//	break;
	//	case (char)0X02:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_1,sizeof (CCC_UAV_A_data1_1));
	//	break;
	//	case (char)0X03:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_2,sizeof (CCC_UAV_A_data1_2));
	//	break;
	//	case (char)0X04:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_3,sizeof (CCC_UAV_A_data1_3));
	//	break;
	//	case (char)0X05:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_4,sizeof (CCC_UAV_A_data1_4));
	//	break;
	//	case (char)0X06:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_5,sizeof (CCC_UAV_A_data1_5));
	//	break;
	//	case (char)0X07:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_6,sizeof (CCC_UAV_A_data1_6));
	//	break;
	//	case (char)0X08:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_7,sizeof (CCC_UAV_A_data1_7));
	//	break;
	//	case (char)0X09:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_8,sizeof (CCC_UAV_A_data1_8));
	//	break;
	//	case (char)0X0A:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_9,sizeof (CCC_UAV_A_data1_9));
	//	break;
	//	case (char)0X0B:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_10,sizeof (CCC_UAV_A_data1_10));
	//	break;
	//	case (char)0X0C:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_11,sizeof (CCC_UAV_A_data1_11));
	//	break;
	//	case (char)0X0D:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_12,sizeof (CCC_UAV_A_data1_12));
	//	break;
	//	case (char)0X0E:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_13,sizeof (CCC_UAV_A_data1_13));
	//	break;
	//	case (char)0X0F:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_14,sizeof (CCC_UAV_A_data1_14));
	//	break;
	//	case (char)0X10:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_15,sizeof (CCC_UAV_A_data1_15));
	//	break;
	//	case (char)0X11:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_16,sizeof (CCC_UAV_A_data1_16));
	//	break;
	//	case (char)0X12:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_17,sizeof (CCC_UAV_A_data1_17));
	//	break;
	//	case (char)0X13:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_18,sizeof (CCC_UAV_A_data1_18));
	//	break;
	//	case (char)0X14:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data1_19,sizeof (CCC_UAV_A_data1_19));
	//	break;
	//	//综合任务管理系统
	//	case (char)0X22:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_0,sizeof (CCC_UAV_A_data2_0));
	//	break;
	//	case (char)0X23:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_1,sizeof (CCC_UAV_A_data2_1));
	//	break;
	//	case (char)0X24:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_2,sizeof (CCC_UAV_A_data2_2));
	//	break;
	//	case (char)0X25:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_3,sizeof (CCC_UAV_A_data2_3));
	//	break;
	//	case (char)0X27:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_4,sizeof (CCC_UAV_A_data2_4));
	//	break;
	//	case (char)0X28:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_5,sizeof (CCC_UAV_A_data2_5));
	//	break;
	//	case (char)0X29:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_6,sizeof (CCC_UAV_A_data2_6));
	//	break;
	//	case (char)0X2A:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_7,sizeof (CCC_UAV_A_data2_7));
	//	break;
	//	case (char)0X2B:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_8,sizeof (CCC_UAV_A_data2_8));
	//	break;
	//	case (char)0X2C:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_9,sizeof (CCC_UAV_A_data2_9));
	//	break;
	//	case (char)0X2D:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_10,sizeof (CCC_UAV_A_data2_10));
	//	break;
	//	case (char)0X2E:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_11,sizeof (CCC_UAV_A_data2_11));
	//	break;
	//	case (char)0X2F:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_12,sizeof (CCC_UAV_A_data2_12));
	//	break;
	//	case (char)0X30:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_13,sizeof (CCC_UAV_A_data2_13));
	//	break;
	//	case (char)0X33:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data2_14,sizeof (CCC_UAV_A_data2_14));
	//	break;
	//	//磁探仪/浮标
	//	case (char)0X31:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data3_0,sizeof (CCC_UAV_A_data3_0));
	//	break;
	//	case (char)0X32:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data3_1,sizeof (CCC_UAV_A_data3_1));
	//	break;
	//	case (char)0X34:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data3_2,sizeof (CCC_UAV_A_data3_2));
	//	break;
	//	case (char)0X35:
	//			memcpy(&(CCC_UAV_Azhens[uav_index].zongherenwuguanlixitong_leida_citanzhiling) + 3,&CCC_UAV_A_data3_3,sizeof (CCC_UAV_A_data3_3));
	//	break;
	//	}
}

// 接收载荷遥测与指令响应帧，并解析其中的遥测数据子帧2（光电数据） KKL-CCC
void recv_telemetry_data_subframe2_single(int uav_index)
{

	// dds_data_rece.resize(1500);
	message_size = 1500;

	SINT32_SIX messageId;
	switch (uav_index)
	{
	case 0:
		messageId = DDSTables.KKL_CCC_1.niConnectionId;
		break;
	case 1:
		messageId = DDSTables.KKL_CCC_10.niConnectionId;
		break;
	case 2:
		messageId = DDSTables.KKL_CCC_11.niConnectionId;
		break;
	case 3:
		messageId = DDSTables.KKL_CCC_12.niConnectionId;
		break;
	default:
		break;
	}

	Receive_Message(messageId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if (enRetCode == 0)
	{
		//		YaoCeDataZhen temYaoCeDataZhen;
		//		memcpy(&temYaoCeDataZhen,dds_data_rece.dataA,sizeof(KKL_CCC_data_21[uav_index]));
		//		// // 启用任务遥测子帧4的第12-13字节（从1开始）的备用位作为无人机平台编号判断251002
		//		unsigned short temPlatNum  = 0;
		//		memcpy(&temPlatNum, &temYaoCeDataZhen.YaoCeDataZiZhen4s.data[11], sizeof(temPlatNum));
		//
		//		for(int i = 0; i< UAV_MAX_NUM; i++)
		//		{
		//			if(formationId[i].planeId == temPlatNum)
		//			{
		//				memcpy(&(KKL_CCC_data_21[i]),dds_data_rece.dataA,sizeof(KKL_CCC_data_21[i]));
		//			}
		//			// 临时更改（默认不识别的先放到1006里，后续必须删掉）
		//			else
		//			{
		//				memcpy(&(KKL_CCC_data_21[1]),dds_data_rece.dataA,sizeof(KKL_CCC_data_21[1]));
		//			}
		//		}
		memcpy(&(KKL_CCC_data_21[uav_index]), dds_data_rece.dataA, sizeof(KKL_CCC_data_21[uav_index]));
	}
	else
	{
		//// printf("Telemetry_Frame_0xaa1001_recv_ERROR ret=%d\n", enRetCode);
	}

	// 任务遥测帧，载荷在线相关处理1C帧
	memcpy(&KKL_CCC_data_21_1[uav_index], &KKL_CCC_data_21[uav_index].YaoCeDataZiZhen1s, 64);
	if (KKL_CCC_data_21_1[uav_index].data[2] == 0x65)
	{
		// 每拍收到标志位置位
		t1c_flag[uav_index] = 1;
	}

	// 任务遥测帧，磁探杆相关处理 3B2帧
	memcpy(&KKL_CCC_data_21_3[uav_index], &KKL_CCC_data_21[uav_index].YaoCeDataZiZhen3s, 64);
	// 帧类别
	if (KKL_CCC_data_21_3[uav_index].data[2] == 0x6a)
	{
		static int TG_timeout[4];
		// 每拍收到标志位置位
		t3b2_flag[uav_index] = 1;
		if (get_int8_bit(KKL_CCC_data_21_3[uav_index].data[4], 4)) // 0故障/1正常
		{
			// 收回完成
			CCC_DPU_data_3.drone_specific_informations[uav_index].loading_status.bar_sta = 2;
			// 探杆长度超时状态字清零
			TG_timeout[uav_index] = 0;
			g_payload_replan[uav_index].CT_Status = 0;
		}
		else if (get_int8_bit(KKL_CCC_data_21_3[uav_index].data[4], 5)) // 0故障/1正常
		{
			// 伸出完成
			CCC_DPU_data_3.drone_specific_informations[uav_index].loading_status.bar_sta = 1;
			// 探杆长度超时状态字清零
			TG_timeout[uav_index] = 0;
			g_payload_replan[uav_index].CT_Status = 1;
		}
		else
		{
			TG_timeout[uav_index]++;
			if (TG_timeout[uav_index] > 20)
				g_payload_replan[uav_index].CT_Status = 0;
		}
		unsigned short changdu = 0;
		static unsigned short last_changdu[4] = {0, 0, 0, 0};
		memcpy(&changdu, &KKL_CCC_data_21_3[uav_index].data[8], 2);
		// 探杆伸出长度赋值
		CCC_DPU_data_3.drone_specific_informations[uav_index].loading_status.bar_out_len = changdu * 0.1;
		//		printf("CT %d\n",changdu);
		// 判断磁探杆是否有变化
		int chang_flag = 0;
		if (last_changdu[uav_index] != changdu)
		{
			chang_flag = 1;
		}
		// 遥控指令，超时判断
		static int timeout[4] = {0};
		if (chang_flag == 0 && zhzl_flag == 1) //&& 综显触发指令，200拍后没有变化就不作数
		{
			timeout[uav_index]++;
			if (timeout[uav_index] > 200)
			{
				// 清除DPU指令
				zhzl_flag = 1;
				timeout[uav_index] = 0;
			}
		}
		else if (chang_flag == 1) // 收到后，超时清空
		{
			// 响应指令后清空
			CCC_DPU_data_3.drone_specific_informations[uav_index].loading_status.bar_sta = 0;
			timeout[uav_index] = 0;
		}
		// 记录上次的探杆长度
		last_changdu[uav_index] = changdu;
	}

	// 任务遥测帧，载荷在线相关处理 4A帧
	memcpy(&KKL_CCC_data_21_4[uav_index], &KKL_CCC_data_21[uav_index].YaoCeDataZiZhen4s, 64);
	// 帧类别
	// 平台载荷 bit0:：光电(0：无1：搭载) bit1：雷达(0：无1：搭载) bit2：中继(0：无1：搭载) bit3：磁探(0：无1：搭载) bit4：浮标监听(0：无1：搭载)
	if (KKL_CCC_data_21_4[uav_index].data[2] == 0x6B)
	{
		// 每拍收到标志位置位
		t4a_flag[uav_index] = 1;
		// 载荷相关数据字节15~16
		short zh;
		memcpy(&zh, &KKL_CCC_data_21_4[uav_index].data[15], 2);
		// 光电bit6
		if (get_bit(zh, 6))
		{
			CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load = setBit(CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load, 0, 1);
		}
		else
		{
			CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load = setBit(CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load, 0, 0);
		}
		// 雷达bit13；替换为浮标 20250806new
		if (get_bit(zh, 13))
		{
			CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load = setBit(CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load, 4, 1);
		}
		else
		{
			CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load = setBit(CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load, 4, 0);
		}
		// 磁探在线状态超时
		static int CT_timeout = 0;
		// 磁探bit14
		if (get_bit(zh, 14))
		{
			// 磁探在线超时清零
			CT_timeout = 0;
			g_payload_replan[uav_index].CT_Online = 1;
			CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load = setBit(CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load, 3, 1);
		}
		else
		{
			CT_timeout++;
			if (CT_timeout > 20)
				g_payload_replan[uav_index].CT_Online = 0;
			CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load = setBit(CCC_DPU_data_3.drone_specific_informations[uav_index].platform_load, 3, 0);
		}
		// 其余暂无
	}

	// 任务遥测帧，相关处理 5帧
	memcpy(&KKL_CCC_data_21_5[uav_index], &KKL_CCC_data_21[uav_index].YaoCeDataZiZhen5s, 64);
	// 帧类别
	if (KKL_CCC_data_21_5[uav_index].data[2] == 0x6D)
	{
		// 每拍收到标志位置位
		t5_flag[uav_index] = 1;
	}

	// 任务遥测帧，相关处理 2帧
	memcpy(&KKL_CCC_data_21_2[uav_index], &KKL_CCC_data_21[uav_index].YaoCeDataZiZhen2s, 64);
	if (KKL_CCC_data_21_2[uav_index].zhenLeiBie == 0x66)
	{
		// 每拍收到标志位置位
		t2_flag[uav_index] = 1;
	}
	// 根据综显的控制源选择相应的无人机信息更新CCC_DPU_data_22
	if ((DPU_CCC_data_14.video_source_type - 2) != uav_index)
	{
		return;
	}
	// bit9: 图像主通道 0-电视1-红外，主要通过该位判断给综显的结果。
	g_image_main_channel = (unsigned char)get_bit(KKL_CCC_data_21_2[uav_index].GDstatus4, 9);
	static unsigned char long_short[4] = {0};
	if (g_image_main_channel == 0x00)
	{ // 电视
		// 当图像主通道为电视时，读取CCD视场视场角，当值小于10度时，为长焦 (分辨率0.01度)
		if (KKL_CCC_data_21_2[uav_index].CCDfield * 0.01 < 10)
		{
			long_short[uav_index] = 0x00; // 长焦
		}
		else
		{
			long_short[uav_index] = 0x01;
		}
	}
	else if (g_image_main_channel == 0x01)
	{ // 红外
		// 当图像主通道为红外时，读取IR视场，当物理值小于10度时，为长焦 (分辨率0.01度)
		if (KKL_CCC_data_21_2[uav_index].IRfield * 0.01 < 10)
		{
			long_short[uav_index] = 0x00; // 长焦
		}
		else
		{
			long_short[uav_index] = 0x01;
		}
	}
	// 电视/红外/长焦/短焦切换 0H=NA;1H=电视长焦;2H=电视短焦;3H=红外长焦;4H=红外短焦
	if (g_image_main_channel == 0x00 && long_short[uav_index] == 0x00)
	{
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.type = 0x01;
	}
	else if (g_image_main_channel == 0x00 && long_short[uav_index] == 0x01)
	{
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.type = 0x02;
	}
	else if (g_image_main_channel == 0x01 && long_short[uav_index] == 0x00)
	{
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.type = 0x03;
	}
	else if (g_image_main_channel == 0x01 && long_short[uav_index] == 0x01)
	{
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.type = 0x04;
	}

	// 红外上电状态反馈
	// 0-未上电
	// 1-制冷中 正在上电
	// 2-正常工作 已上电
	// 3-校正中 暂时不用
	if (KKL_CCC_data_21_2[uav_index].GDstatus3.infrared_work_status == 0)
	{
		// 0-未上电
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.ir_pwr_fb = 3; // 红外上电状态反馈 3=未上电
	}
	else if (KKL_CCC_data_21_2[uav_index].GDstatus3.infrared_work_status == 1)
	{
		// 1-制冷中 正在上电
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.ir_pwr_fb = 2; // 红外上电状态反馈 2=正在上电
	}
	else if (KKL_CCC_data_21_2[uav_index].GDstatus3.infrared_work_status == 2)
	{
		// 2-正常工作 已上电
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.ir_pwr_fb = 1; // 红外上电状态反馈 1=已上电
	}
	else
	{
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.ir_pwr_fb = 0; // 红外上电状态反馈 0=NA
	}

	// 光电状态反馈
	float fangweijiao = (float)KKL_CCC_data_21_2[uav_index].fangweijiao * 360 / (1LL << 16);
	float fuyangjiao = (float)KKL_CCC_data_21_2[uav_index].fuyangjiao * 360 / (1LL << 16);
	// fangweijiao = 17.8f;
	// fuyangjiao = 19.3f;
	CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.photoelectric_platform_azimuth = fangweijiao; // 光电平台方位角
	CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.photoelectric_platform_pitch = fuyangjiao;	   // 光电平台俯仰角

	// printf("fw %f fy %f\n",fangweijiao,fuyangjiao);

	unsigned short Target1status = KKL_CCC_data_21_2[uav_index].Target1status;
	unsigned short Target2status = KKL_CCC_data_21_2[uav_index].Target2status;
	GeoLibDas geoLibLocal = {plane_lon, plane_lat};

	// 20241119:由于测试环境存在目标有效性为0x100的数，所以仅对有效性为1的数值进行赋值
	// 对目标1和目标2的有效性判断，并赋值目标1和目标2的距离。
	CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.target_param_significant_bit = setBit(CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.target_param_significant_bit, 0, 0); // 目标1参数有效位
	if (Target1status == 1)
	{
		CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.target_param_significant_bit = setBit(CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.target_param_significant_bit, 0, 1); // 目标1参数有效位
		double Target1Lon = (double)KKL_CCC_data_21_2[uav_index].Target1Lon * 180 / ((1LL << 31) - 1);
		double Target1Lat = (double)KKL_CCC_data_21_2[uav_index].Target1Lat * 180 / ((1LL << 31) - 1);
		GeoLibDas geoLibDas1 = {Target1Lon, Target1Lat};
		CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.target1_dist = (float)getDistanceGeoLibDas(&geoLibDas1, &geoLibLocal); // 目标1距离
	}
	CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.target_param_significant_bit = setBit(CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.target_param_significant_bit, 8, 0); // 目标2参数有效位
	if (Target2status == 1)
	{
		CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.target_param_significant_bit = setBit(CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.target_param_significant_bit, 8, 1); // 目标2参数有效位
		double Target2Lon = (double)KKL_CCC_data_21_2[uav_index].Target2Lon * 180 / ((1LL << 31) - 1);
		double Target2Lat = (double)KKL_CCC_data_21_2[uav_index].Target2Lat * 180 / ((1LL << 31) - 1);
		GeoLibDas geoLibDas2 = {Target2Lon, Target2Lat};
		CCC_DPU_data_22[uav_index].photoelectric_state_feedbacks.target2_dist = (float)getDistanceGeoLibDas(&geoLibDas2, &geoLibLocal); // 目标2距离
	}
	// 扫描/锁定状态 20250723new(多架机需要分开标志量，不然会混乱251001)
	static float last_fangweijiao[UAV_MAX_NUM] = {0};
	static int cnt[UAV_MAX_NUM] = {0};
	// 计算每拍的夹角角度
	float ac = last_fangweijiao[uav_index] - fangweijiao;
	if (ac > 180)
	{
		ac -= 360;
	}
	else if (ac < -180)
	{
		ac += 360;
	}
	// 计算绝对值
	if (ac < 0)
	{
		ac = -ac;
	}

	// 连续5拍变化不动认为锁定
	if (ac < 0.5)
	{
		// 记录五拍持续变化
		cnt[uav_index]++;
	}
	else
	{
		// 扫描状态
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.lock_scan_switch = 2;
		// 扫描状态目前只有前向方位扫描
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.scan_param = 1;
		// 锁定状态清零
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.lock_param = 0;
		// 计数清零
		cnt[uav_index] = 0;
	}
	if (cnt[uav_index] > 15)
	{
		// 锁定状态，// 锁定参数 0H=NA;1H=垂直下视锁定;2H=前向锁定;3H=右向锁定;4H=后向锁定;5H=左侧锁定
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.lock_scan_switch = 1;
		// 垂直下视
		if (fuyangjiao <= -85)
		{
			CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.lock_param = 1;
		}
		// 前向锁定
		else if (fangweijiao > -45 && fangweijiao <= 45)
		{
			CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.lock_param = 2;
		}
		// 右向锁定
		else if (fangweijiao > 45 && fangweijiao <= 135)
		{
			CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.lock_param = 3;
		}
		// 后向锁定
		else if ((fangweijiao > 135 && fangweijiao <= 180) || (fangweijiao <= -135 && fangweijiao >= -180))
		{
			CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.lock_param = 4;
		}
		// 左侧锁定
		else if (fangweijiao > -135 && fangweijiao <= -45)
		{
			CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.lock_param = 5;
		}
		// 扫描状态清零
		CCC_DPU_data_22[uav_index].video_param_setting_feedbacks.scan_param = 0;
	}
	// 记录上一拍的方位角
	last_fangweijiao[uav_index] = fangweijiao;
}

// 接收综显磁探杆指令
void recv_blk_ofp_ccc_045()
{
	// 返航控制指令拍数
	static char fh_cnt = 0;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_36.niConnectionId, DDSTables.DPU2_CCC_36.niConnectionId, &blk_ofp_ccc_045, sizeof blk_ofp_ccc_045);
	if (enRetCode == 0)
	{
		unsigned short index = blk_ofp_ccc_045.drone_num - 1;
		if (blk_ofp_ccc_045.tg == 1) // 探杆 0-NA，1-伸出，2-缩回
		{
			// 收到综显载荷控制指令
			zhzl_flag = 1;
			CCC_UAV_Azhens[index].zongherenwuguanlixitong_leida_citanzhiling[0] = 0x32;
			CCC_UAV_Azhens[index].zongherenwuguanlixitong_leida_citanzhiling[1] = 0x32;
			CCC_UAV_Azhens[index].zongherenwuguanlixitong_leida_citanzhiling[2] = 0x32;
			CCC_UAV_Azhens[index].zongherenwuguanlixitong_leida_citanzhiling[3] = 0xaa; // 0xaa伸出，0x55收回
		}
		else if (blk_ofp_ccc_045.tg == 2)
		{
			// 收到综显载荷控制指令
			zhzl_flag = 1;
			CCC_UAV_Azhens[index].zongherenwuguanlixitong_leida_citanzhiling[0] = 0x32;
			CCC_UAV_Azhens[index].zongherenwuguanlixitong_leida_citanzhiling[1] = 0x32;
			CCC_UAV_Azhens[index].zongherenwuguanlixitong_leida_citanzhiling[2] = 0x32;
			CCC_UAV_Azhens[index].zongherenwuguanlixitong_leida_citanzhiling[3] = 0x55; // 0xaa伸出，0x55收回
		}

		// 返航控制指令
		if (blk_ofp_ccc_045.gd == 1) // 0-NA，1-返航
		{
			fh_cnt = 9;
			fh_timeout[index] = 1;
		}
	}
	/*********** 20250821new 返航，单无人机控制指令*********/
	//    if(single_uav_flag == 1)
	//    {
	//    	fh_cnt = 9;
	//		single_uav_flag = 0;
	////    	single_uav_flag = 111;
	//    }
	/****************************************************/
	if (fh_cnt > 0 && fh_cnt <= 3)
	{
		// 再发控制返航
		unsigned short index = blk_ofp_ccc_045.drone_num - 1;
		blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.fc_order[0] = 0x30;
		blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.fc_order[1] = 0x30;
		blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.fc_order[2] = 0x30;
		fh_cnt--;

		//		//切换
		//		order_data_frames.track_point_chage_orders.track_line_id = 17;   // 航线号
		//		order_data_frames.track_point_chage_orders.track_point_id = 1;
		//		track_threat_frames.order_code[0] = 0x40;
		//		track_threat_frames.order_code[1] = 0x40;
		//		track_threat_frames.order_code[2] = 0x40;
		//		// 将指令存入遥控帧
		//		memcpy(&(track_threat_frames.order_data),&(order_data_frames.track_point_chage_orders),sizeof(order_data_frames.track_point_chage_orders));
	}
	else if (fh_cnt > 6 && fh_cnt <= 9)
	{
		// 先发预控
		unsigned short index = blk_ofp_ccc_045.drone_num - 1;
		blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.fc_order[0] = 0x08;
		blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.fc_order[1] = 0x08;
		blk_ccc_kkl_008_026_027_028[index].front.basic_yaotiao_order_frame_data.fc_order[2] = 0x08;
		fh_cnt--;
	}
	else if (fh_cnt > 3 && fh_cnt <= 6)
	{
		// 延时三拍
		fh_cnt--;
	}
}

void recv_blk_ofp_ccc_047()
{
	// 接收综显语音指令
	recv_dpu1_dpu2(DDSTables.DPU_CCC_047.niConnectionId, DDSTables.DPU2_CCC_047.niConnectionId, &blk_dtms_dtps_047, sizeof(blk_dtms_dtps_047));
	if (enRetCode == 0)
	{
		if (blk_dtms_dtps_047.control == 0 && blk_dtms_dtps_047.confirm == 1) // 确认执行
		{
			// 单无人机规划需要判断是否具备执行条件
			if (signleUavCmdProtect())
			{
				// 发送确认执行指令
				blk_dtps_dtms_043.executeSta = 1;
			}
			else
			{
				// 发送执行失败指令
				blk_dtps_dtms_043.executeSta = 3;
			}

			send_blk_ccc_ofp_043();
		}
		else if (blk_dtms_dtps_047.control == 0 && blk_dtms_dtps_047.confirm == 0) // 取消执行
		{
			// 发送取消确认执行指令
			blk_dtps_dtms_043.executeSta = 0;
			send_blk_ccc_ofp_043();
		}
		else
		{
			// 收到综显语音指令后转发给DTPS
			sendVoiceMsgToDtps((unsigned char *)&blk_dtms_dtps_047, sizeof(blk_dtms_dtps_047));
		}
	}
}
void send_blk_ccc_ofp_043()
{
	data_length = sizeof(BLK_CCC_OFP_043);
	Send_Message(DDSTables.CCC_DPU_043.niConnectionId, 0, &transaction_id, &blk_dtps_dtms_043, &message_type_id, data_length, &enRetCode);
}
// 单无人机指令保护处理 false:不可执行；true:可以执行
BOOL signleUavCmdProtect()
{

	if (blk_dtps_dtms_043.cmdType != 8) // 非单无人机指控
	{
		return TRUE;
	}

	// 判断无人机是否存在
	if (blk_dtps_dtms_043.singleUavSn < 1 || blk_dtps_dtms_043.singleUavSn > UAV_MAX_NUM) // 编队号无效
	{
		return FALSE;
	}

	if (formationId[blk_dtps_dtms_043.singleUavSn - 1].count > formationIdTimeOutCount) // 不在线
	{
		return FALSE;
	}

	// 控权判断
	if (formationId[blk_dtps_dtms_043.singleUavSn - 1].isControl == 0) // 无控权
	{
		return FALSE;
	}

	// 判断任务区是否存在
	if ((blk_dtps_dtms_043.singleUavType == 1) ||
		(blk_dtps_dtms_043.singleUavType == 12) ||
		(blk_dtps_dtms_043.singleUavType == 13))
	{
		// 遍历所有任务区，如果没找到任务区，则返回失败
		BOOL isExit = FALSE;
		for (int i = 0; i < 8; i++)
		{
			if ((blk_ccc_ofp_033[0].area_informations[i].area_type == 1) &&
				(blk_ccc_ofp_033[0].area_informations[i].area_code == blk_dtps_dtms_043.singleUavAreaId))
			{
				isExit = TRUE;
			}
		}

		// 没找到
		if (!isExit)
		{
			return FALSE;
		}
	}

	// 判断目标是否存在
	if ((blk_dtps_dtms_043.singleUavType == 2) ||
		(blk_dtps_dtms_043.singleUavType == 14))
	{
		// 遍历所有任务区，如果没找到任务区，则返回失败
		BOOL isExit = FALSE;

		for (int k = 0; k < 30; k++)
		{
			// 找到批号对应目标,0也是有效的
			if (DPU_CCC_data_0.target_informations[k].target_lot_number1 == blk_dtps_dtms_043.singleUavTgtId)
			{
				isExit = TRUE;
			}
		}

		// 没找到
		if (!isExit)
		{
			return FALSE;
		}
	}

	return TRUE;
}
void recv_blk_ofp_ccc_053()
{
	recv_dpu1_dpu2(DDSTables.DPU_CCC_048.niConnectionId, DDSTables.DPU2_CCC_048.niConnectionId, &blk_ofp_ccc_053, sizeof blk_ofp_ccc_053);
	if (enRetCode == 0)
	{
		if (blk_ofp_ccc_053.cmd == 1)
		{
			// 双机编队领航：两架无人机都要跟随有人机
			start_lh_flag[0] = 1;
			start_lh_flag[1] = 1;
			printf("LH start both uav lead=0x%X", load_file.lead_uav_id);
		}
		else if (blk_ofp_ccc_053.cmd == 2)
		{
			// 双机编队退出领航：两架无人机都要退出
			start_tclh_flag[0] = 1;
			start_tclh_flag[1] = 1;
			printf("LH exit both uav lead=0x%X", load_file.lead_uav_id);
		}
	}
}

void recv_blk_ofp_ccc_155()
{
	static int BD_ability_cnt = 0;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_155.niConnectionId, DDSTables.DPU2_CCC_155.niConnectionId, &blk_ofp_ccc_155, sizeof blk_ofp_ccc_155);
	if (enRetCode == 0)
	{
		// 开始发送编队能力控制指令
		BD_ability_cnt = 9;
	}

	if (BD_ability_cnt > 0 && BD_ability_cnt <= 3)
	{
		// 发送编队能力控制指令
		int uav_index = blk_ofp_ccc_155.uavSn - 1;
		if (blk_ofp_ccc_155.opcode == 0) // 退出编队
		{
			blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.fc_order[0] = 0x02;
			blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.fc_order[1] = 0x02;
			blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.fc_order[2] = 0x02;
		}
		else if (blk_ofp_ccc_155.opcode == 1) // 进入编队
		{
			blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.fc_order[0] = 0x04;
			blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.fc_order[1] = 0x04;
			blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.fc_order[2] = 0x04;
		}
		BD_ability_cnt--;
	}
	else if (BD_ability_cnt > 6 && BD_ability_cnt <= 9)
	{
		// 先发预控
		int uav_index = blk_ofp_ccc_155.uavSn - 1;
		blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.fc_order[0] = 0xf0;
		blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.fc_order[1] = 0xf0;
		blk_ccc_kkl_008_026_027_028[uav_index].tail.baseControl.fc_order[2] = 0xf0;
		BD_ability_cnt--;
	}
	else if (BD_ability_cnt > 3 && BD_ability_cnt <= 6)
	{
		// 延时三拍
		BD_ability_cnt--;
	}
}
void recv_blk_ofp_ccc_156()
{
	static int query_code = 0;
	static int query_timeout_cnt = 0;
	static int failed_cnt = 0;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_156.niConnectionId, DDSTables.DPU2_CCC_156.niConnectionId, &blk_ofp_ccc_156, sizeof blk_ofp_ccc_156);
	if (enRetCode == 0)
	{
		// 查询无人机航线
		// 查询中
		send_blk_ccc_ofp_157(0);
		// 查询航线长度
		query_code = 1;
		send_index = blk_ofp_ccc_156.uavSn - 1;
		// 重置无人机航线长度
		s4D_frame_41[send_index] = 0;
		// 重置失败重发次数
		failed_cnt = 0;
		// 重置航线
		memset(&blk_ccc_ofp_158, 0, sizeof(BLK_CCC_OFP_158));
	}
	// 超时计时
	if (query_code > 0)
	{
		query_timeout_cnt++;
	}
	if (query_timeout_cnt > 30 * 20)
	{
		if (query_code == 4 && failed_cnt < 2)
		{
			// 再次查询一遍
			query_code = 3;
			query_timeout_cnt = 0;
			// 失败重发计数
			failed_cnt++;
			// 查询失败后清空原来的航点数据
			memset(&blk_ccc_ofp_158, 0, sizeof(BLK_CCC_OFP_158));
		}
		else
		{
			// 查询失败
			send_blk_ccc_ofp_157(2);
			// 清空标志位
			query_code = 0;
			query_timeout_cnt = 0;
			send_index = 0;
		}
	}
	// 发送0x41指令查询航线长度
	if (query_code == 1)
	{
		static int send_cnt_0x41 = 0;
		send_cnt_0x41++;
		// 发送0x41
		send_uav_hl(blk_ofp_ccc_156.uavCode, 0x41);
		// 发送三次,开始接收回报
		if (send_cnt_0x41 > 3)
		{
			query_code = 2;
			send_cnt_0x41 = 0;
		}
	}
	else if (query_code == 2)
	{
		//接收到无人机航线长度，进入下一步查询
		if(s4D_frame_41[send_index] > 0)
		{
			//重置超时时间
			query_timeout_cnt = 0;
			query_code = 3;
		}
	}
	else if(query_code == 3)
	{
		//发送查询指令0x3c
		static int send_cnt_0x3c = 0;
		send_cnt_0x3c++;
		//发送0x3c
		send_uav_hl(blk_ofp_ccc_156.uavCode,0x3c);
		//发送三次,开始接收回报
		if(send_cnt_0x3c > 3)
		{
			query_code = 4;
			send_cnt_0x3c = 0;
		}
	}
	else if(query_code == 4)
	{
		//遍历查看航线是否接收完全
		int num = 0;
		for(int i = 0 ; i < s4D_frame_41[send_index] ; i ++)
		{
			if(blk_ccc_ofp_158.point[i].latitude > 0 && blk_ccc_ofp_158.point[i].longitude > 0)
			{
				num++;
			}
		}
		//航线接收完成
		if(num == s4D_frame_41[send_index])
		{
			//查询成功
			send_blk_ccc_ofp_157(1);
			//发送航线
			send_blk_ccc_ofp_158();
			//清空标志位
			query_code = 0;
			query_timeout_cnt = 0;
			send_index = 0;
		}
	}
}
//航线查询状态
void send_blk_ccc_ofp_157(unsigned char status)
{
	blk_ccc_ofp_157.uavSn = blk_ofp_ccc_156.uavSn;
	blk_ccc_ofp_157.uavCode = blk_ofp_ccc_156.uavCode;
	blk_ccc_ofp_157.status = status;
	data_length = sizeof(BLK_CCC_OFP_157);
	Send_Message(DDSTables.CCC_DPU_157.niConnectionId,0,&transaction_id, &blk_ccc_ofp_157, &message_type_id, data_length, &enRetCode);
}

void send_blk_ccc_ofp_158()
{
	blk_ccc_ofp_158.uavSn = blk_ofp_ccc_156.uavSn;
	blk_ccc_ofp_158.uavCode = blk_ofp_ccc_156.uavCode;
	blk_ccc_ofp_158.num =	s4D_frame_41[send_index];
	data_length = sizeof(BLK_CCC_OFP_158);
	Send_Message(DDSTables.CCC_DPU_158.niConnectionId,0,&transaction_id, &blk_ccc_ofp_158, &message_type_id, data_length, &enRetCode);
}
// 接收载荷遥测与指令响应帧，并解析其中的遥测数据子帧2（光电数据） KKL-CCC
void recv_telemetry_data_subframe2()
{
	for(int uav_index = 0; uav_index < UAV_MAX_NUM; uav_index++)
	{
		recv_telemetry_data_subframe2_single(uav_index);
	}
}

//// uav_index无人机序号（0-3）; actionflag 0:开始交接；1：交接结束
void fc_order_send(int uav_index, int actionFlag)
{
	blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.yt_start = 'R';

	//指令编码，控制权交接结束0x24
	if(actionFlag == 0)
	{
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_code[0] = 0x04;
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_code[1] = 0x04;
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_code[2] = 0x04;

		//指令数据，控制站地址
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_data[0] = GCS_ID;//0x3001
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_data[1] = GCS_ID;
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_data[2] = GCS_ID;
	}
	else
	{
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_code[0] = 0x24;
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_code[1] = 0x24;
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_code[2] = 0x24;

		//指令数据，控制站地址
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_data[0] = 0x9001;
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_data[1] = 0x9001;
		blk_ccc_kkl_008_026_027_028[uav_index].front.basic_yaotiao_order_frame_data.order_data[2] = 0x9001;
	}

}






/** 接收空地链的消息****/
void recv_kdl_message()
{
//	recv_blk_kdl_ccc_002();  // 不用收了，kdl直接发给综显
	recv_blk_kdl_ccc_003();
	recv_blk_kdl_ccc_015();
}

/** 接收SMD的消息****/
void recv_SMD_message()
{
	recv_blk_smd_ccc_000();
	recv_blk_smd_ccc_001();
}

//无人声指令，有人机上
void recv_blk_smd_ccc_000()
{
	Receive_Message(DDSTables.SMD_CCC_000.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode==0)
	{
		memcpy(&blk_smd_ccc_000, dds_data_rece.dataA, sizeof(blk_smd_ccc_000));
		int index = formationIdManage((int)blk_smd_ccc_000.UAV_ID);
		if(index >= 0)
		{
			unsigned char check_sum = checkSum((unsigned char*)&blk_smd_ccc_000,(int)sizeof(blk_smd_ccc_000));
			memcpy(&CCC_UAV_Azhens[index].zongherenwuguanlixitong_leida_citanzhiling[0],&blk_smd_ccc_000,sizeof(blk_smd_ccc_000));
			memcpy(&CCC_UAV_Azhens[index].zongherenwuguanlixitong_leida_citanzhiling[35],&check_sum,1);
		}

	}
}

//浮标投放信息
void recv_blk_smd_ccc_001()
{
	Receive_Message(DDSTables.SMD_CCC_001.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode==0)
	{
		BLK_SMD_CCC_001 temp;
		memcpy(&temp, dds_data_rece.dataA, sizeof(BLK_SMD_CCC_001));
		int index = 0;
		if(temp.PlatForm == 0x9001)
		{
			index = 0;
		}
		else if(temp.PlatForm == 0x1005)
		{
			index = 1;
		}
		else if(temp.PlatForm == 0x1006)
		{
			index = 2;
		}
		else
		{
			return ;
		}
		memcpy(&blk_smd_ccc_001[index], &temp, sizeof(BLK_SMD_CCC_001));
	}
}
void recv_blk_kdl_ccc_002()
{
	Receive_Message(DDSTables.KDL_CCC_6.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode==0)
	{
		memcpy(&blk_kdl_ccc_002, dds_data_rece.dataA, sizeof(blk_kdl_ccc_002));

		// 向ofp发送shenqing
		memcpy((unsigned char*)&CCC_DPU_data_1+2, (unsigned char*)&blk_kdl_ccc_002+4, sizeof(CCC_DPU_data_1)-2);//跳过各自不一致
		send_kongzhiquan_jiaojie();
	}
}

void recv_blk_kdl_ccc_003()
{
	memset(&blk_kdl_ccc_003,0,sizeof blk_kdl_ccc_003);
	Receive_Message(DDSTables.KDL_CCC_4.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode==0)
	{
		memcpy(&blk_kdl_ccc_003, dds_data_rece.dataA, sizeof(blk_kdl_ccc_003));
	}
}

void recv_blk_kdl_ccc_015()
{
	Receive_Message(DDSTables.KDL_CCC_015.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode==0)
	{
		memcpy(&blk_kdl_ccc_015, dds_data_rece.dataA, sizeof(blk_kdl_ccc_015));
	}

}


#if _PC_SIMULATION_
#if SIMTESTFLAG
void writeLog(char* block, int length)
{
	QByteArray tem((char*)block, length);

	if(!logFile->open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
	{
		static QTextStream stream(logFile);
		stream <<  tem.toHex(',') << "\n";
		static int count1= 0;
		if(count1++ == 256)
		{
			logFile->close();
			count1 = 0;
		}
	}
}
#endif
#endif

//无人机协同照射控制
void send_blk_dpu_ccc_020()
{
	data_length = sizeof(BLK_CCC_DPU_020);
	Send_Message( DDSTables.CCC_DPU_36.niConnectionId,0,&transaction_id, &blk_ccc_dpu_020 , &message_type_id, data_length, &enRetCode);
}

//射线法判断是否在多边形内
int pnpoly(Point * polygon,int n , Point p)
{
	int inside = 0;
	for(int i = 0 ,j = n- 1; i < n ; j = i ++)
	{
		Point a;
		Point b;
		a = polygon[i];
		b = polygon[j];
		//跳过水平边
		if(fabs(a.lat - b. lat) < (1e-8))
			continue;

		//判断边是否跨越待测点的纬度线
		int a_above;
		int b_above;
		a_above = (a.lat > p.lat + (1e-8)); //a点在射线上方
		b_above = (b.lat > p.lat + (1e-8)); //b点在射线上方
		if(a_above == b_above)
			continue;

		//计算边与射线（y = p.lat）的交点经度x_inter
		double t;
		double x_inter;
		t = (p.lat - a.lat) / (b.lat - a.lat);//参数t
		x_inter = a.lon + t * (b.lon - a.lon);//交点经度

		//待测点在交点左侧时翻转状态（考虑精度误差）
		if(p.lon <x_inter - (1e-8))
		{
			inside = !inside;
		}
	}
	return inside;
}

void recv_ofp_message()
{
	recv_blk_ofp_ccc_000();//接收综显目标集
	recv_blk_ofp_ccc_018(); //接收有人机航线
	recv_blk_ofp_ccc_154();
	recv_blk_ofp_ccc_037();
	recv_blk_ofp_ccc_040();
	recv_blk_ofp_ccc_045();//接收综显磁探杆指令
	recv_blk_ofp_ccc_047();//语音
	recv_blk_ofp_ccc_148();//微调遥控指令
	recv_blk_ofp_ccc_053();//领航指令
	recv_blk_ofp_ccc_155();//编队能力控制
	recv_blk_ofp_ccc_156();//航线查询指令
}

void recv_blk_ofp_ccc_000()
{
	//接收有人机目标集
	message_size=2096;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_0.niConnectionId,DDSTables.DPU2_CCC_0.niConnectionId,&DPU_CCC_data_0,sizeof DPU_CCC_data_0);
	if (enRetCode == 0)
	{
		//		printf("blk_ofp_ccc_000\n");
	}
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//		Receive_Message(DDSTables.PAD_CCC_000.niConnectionId, 0, &transaction_id, &DPU_CCC_data_0.message_ID, &message_type_id, &message_size, &enRetCode);
	}
}

void recv_blk_ofp_ccc_018()
{
	//接收有人机通用航路
	message_size = 2096;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_6.niConnectionId,
			DDSTables.DPU2_CCC_6.niConnectionId, &blk_ofp_ccc_018,
			sizeof blk_ofp_ccc_018);
	if (enRetCode == 0)
	{
		printf("blk_ofp_ccc_018\n");
		// 转给pad
		data_length = sizeof(blk_ofp_ccc_018);
		Send_Message(DDSTables.CCC_PAD_018.niConnectionId,0,&transaction_id, &blk_ofp_ccc_018, &message_type_id, data_length, &enRetCode);
	}

}

void recv_blk_ofp_ccc_040()
{
	message_size=2096;
	recv_dpu1_dpu2(DDSTables.DPU_CCC_24.niConnectionId,DDSTables.DPU2_CCC_24.niConnectionId,&blk_ofp_ccc_040,sizeof blk_ofp_ccc_040);
	//收不到就收PAD new20250620
	if(enRetCode != 0)
	{
		//		Receive_Message(DDSTables.PAD_CCC_040.niConnectionId, 0, &transaction_id, &blk_ofp_ccc_040, &message_type_id, &message_size, &enRetCode);
	}
	if (enRetCode == 0)
	{
		SINT32_SIX niConnectionId = 0;
		niConnectionId = DDSTables.CCC_KKL_16.niConnectionId;

		if(niConnectionId != 0)
		{
			data_length = sizeof(blk_ofp_ccc_040);
			Send_Message(niConnectionId,0,&transaction_id, &blk_ofp_ccc_040, &message_type_id, data_length, &enRetCode);
		}

	}
}


void send_blk_ccc_ofp_015()
{
	// 态势融合数据从态势部分接收，不用处理直接周期转发即可

	//目标航速航向计算

	Send_Message(DDSTables.CCC_DPU_5.niConnectionId, 0, &transaction_id, &CCC_DPU_data_5.tgt_Number, &message_type_id, sizeof(CCC_DPU_data_5), &enRetCode);
	if(enRetCode==0)
	{
		//printf("send_blk_ccc_ofp_015 rong he mu biao\n");
	}
	//发送给pad new20250620
	//	Send_Message(DDSTables.CCC_PAD_015.niConnectionId,0,&transaction_id, &CCC_DPU_data_5.tgt_Number,&message_type_id, data_length, &enRetCode);

}
void init_blk_ccc_ofp_045()
{
	load_file.blk_dlr_ccc_045[0].uav_id = UAV1_ID;
	load_file.blk_dlr_ccc_045[0].normal_num = 12;
	int cnt = 0;//0
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.261683;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.533579;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 160;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 0;

	cnt++;//1
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.2444;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.57;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 550;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 30;

	cnt++;//2
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.2;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.57;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 600;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 30;

	cnt++;//3
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.2;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.533653;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 600;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 30;

	cnt++;//4
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.32;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.533653;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 600;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 30;

	cnt++;//5
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.32;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.497;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 600;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 30;

	cnt++;//6
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.265056;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.497;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 600;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 30;

	cnt++;//7
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.265056;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.585;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 600;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 30;

	cnt++;//8
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.24;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.585;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 600;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 30;

	cnt++;//9
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.2444;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.57;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 500;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 30;

	cnt++;//10
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.24993;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.55793;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 400;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 30;

	cnt++;//11
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lon = 124.261683;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].lat = 47.533579;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].height = 100;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].speed = 30;
	//test
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].tpye = 1;
	load_file.blk_dlr_ccc_045[0].normal_point[cnt].circle_time = 5;

}
void send_blk_ccc_ofp_045()
{
	static int hx_cnt = 0;
	hx_cnt ++;
	//一秒发一次
	if(hx_cnt > 20)
	{
		//发送应急航线
		for(int j = 0 ; j < 4 ; j++)
		{
			data_length = sizeof(BLK_DLR_CCC_045);
			Send_Message(DDSTables.CCC_DPU_40.niConnectionId,0,&transaction_id, &load_file.blk_dlr_ccc_045[j], &message_type_id, data_length, &enRetCode);
		}
		//发送完成清空拍数计时
		hx_cnt = 0;
	}

}

void send_blk_ccc_ofp_032()
{
	data_length = sizeof(blk_ccc_ofp_032);
	Send_Message(DDSTables.CCC_DPU_12.niConnectionId, 0, &transaction_id,
			&blk_ccc_ofp_032, &message_type_id, data_length, &enRetCode);

}

void send_period_message()
{
	send_ofp_period_message();
	send_kdl_period_message();
	send_sfs_period_message();
}

void send_ofp_period_message()
{
	send_blk_ccc_ofp_015();
	send_blk_ccc_ofp_026();
	send_blk_ccc_ofp_027();
	send_blk_ccc_ofp_029();
	send_blk_ccc_ofp_030();
	send_blk_ccc_ofp_045();
}

void send_kdl_period_message()
{
	send_blk_ccc_kdl_000();
	send_blk_ccc_kdl_014();
}

void recv_all_message()
{
	recv_kdl_message();
	recv_SMD_message();
	/******接收综显消息****/
	recv_ofp_message();

	/******接收PAD消息****/
	recv_pad_message();

	/***中间件时间*****/
	recv_mids_time_message();
}


/****************目标融合部分处理*******************************************/
void send_sfs_period_message()
{
	send_blk_ccc_sfs_001();
}

void send_blk_ccc_sfs_001()
{
	/********************************造数据****************/
	//    static char frame = 0;
	//    KKL_CCC_data_21_2[0].tongBuMa1 = 0xeb;//分辨率1,范围EBH 同步码1
	//    KKL_CCC_data_21_2[0].tongBuMa2 = 0x90;//分辨率1,范围90H 同步码2
	//    KKL_CCC_data_21_2[0].zhenLeiBie = 0x66;//分辨率1,范围66H 帧类别
	//    KKL_CCC_data_21_2[0].xuHao = (unsigned int)frame++;    //分辨率1,范围0-255 序号
#if 0
	unsigned short zhiLingLeiXing;//分辨率1 指令类型反馈，根据该位判断指令已被接受
	//备注：若接收命令回告,00为空，FF为无效命令，无效指令清除时间为3s
	short fangweijiao;      //分辨率360/2^16 光电平台方位角
	short fuyangjiao;       //分辨率360/2^16 光电平台俯仰角
	//当图像主通道为电视时，读取CCD视场，当物理值小于10度时，为长焦
	unsigned short CCDfield; //分辨率0.01度 电视视场角
	//当图像主通道为红外时，读取IR视场，当物理值小于10度时，为长焦
	unsigned short IRfield; //分辨率0.01度 红外视场角
	unsigned short Camerfield;  //分辨率0.01度 相机视场角
	unsigned short GDstatus1;  //暂不用
	unsigned short GDstatus2;   //暂不用
	PhotEelecticPlatformStatus GDstatus3;   //用于判断红外工作状态
	// 0-未上电
	// 1-制冷中 正在上电
	// 2-正常工作 已上电
	// 3-校正中 暂时不用 20241119
	unsigned short GDstatus4; //用于状态回复判断
	//当前仅需要状态字4：
	//D0-D4:表示光电工作状态 0-未上电、1-手动、2-自检、3-归零、4-地理跟踪、5-随动引导、6-多目标跟踪、7-位置、
	//D5-D7:单杆灵敏性
	//D8: 跟踪场景 0-对海 1-对岸
	//D9: 图像主通道 0-电视1-红外 ，主要通过该位判断给综显的结果。
	//D10-D11: 图像跟踪状态 0-非跟踪，1-正常跟踪，2-跟踪目标丢失
	//D12-D14: 条带宽度，未定义
	//D15:
	unsigned short spare;//
	unsigned short CRC;//校验核 0mod 65535
#endif

	//    KKL_CCC_data_21_2[0].Target1status = 0xffff; //缺少该信息定义，
	//    KKL_CCC_data_21_2[0].Target1Lotnum = 1;//分辨率1 目标批号
	//    KKL_CCC_data_21_2[0].Target1Velocity = 50;//分辨率0.1KM/H,目标航速
	//    KKL_CCC_data_21_2[0].Target1Course = 1000;//分辨率0.00549316度，目标航向
	//    KKL_CCC_data_21_2[0].Target1Lon = 30*(pow(2,31)-1)/180.0;//分辨率180/（2^31-1）度，
	//    KKL_CCC_data_21_2[0].Target1Lat = 30*(pow(2,31)-1)/180.0;//分辨率180/（2^31-1）度?90?180?
	//    KKL_CCC_data_21_2[0].Target1Hight = 1000.0 * (pow(2,15)-1) / 12000.0;//分辨率12000/（2^15-1） 单位
	//    KKL_CCC_data_21_2[0].Target2status = 0; //缺少该信息定义，
	//    KKL_CCC_data_21_2[0].Target2Lotnum = 0;//分辨率1 目标批号
	//    KKL_CCC_data_21_2[0].Target2Velocity = 0;//分辨率0.1KM/H,目标航速
	//    KKL_CCC_data_21_2[0].Target2Course = 0;//分辨率0.00549316度，目标航向
	//    KKL_CCC_data_21_2[0].Target2Lon = 0;//分辨率180/（2^31-1）度，
	//    KKL_CCC_data_21_2[0].Target2Lat = 0;//分辨率180/（2^31-1）度?90?180?
	//    KKL_CCC_data_21_2[0].Target2Hight = 0;//分辨率12000/（2^15-1） 单位
	//
	//    DPU_CCC_data_0.target_number = 1;
	//
	//    unsigned short  tem = 0xffff;
	//    memcpy(&DPU_CCC_data_0.target_informations[0].valid_flag, &tem, sizeof(DataValidFlag));	/*目标数据有效性*/
	//    DPU_CCC_data_0.target_informations[0].platform_logo = MANNED_ID;//目标平台标识
	//    DPU_CCC_data_0.target_informations[0].target_lot_number1 =1;//目标批号1
	//    DPU_CCC_data_0.target_informations[0].target_lot_number2 =2;//目标批号2
	//    DPU_CCC_data_0.target_informations[0].Send_platform_identification_number = 0x9002;//目标发送平台编识号
	//    DPU_CCC_data_0.target_informations[0].target_longitude_and_latitude_synt.longitude = plane_lon + 0.234;;//目标经纬度
	//    DPU_CCC_data_0.target_informations[0].target_longitude_and_latitude_synt.latitude = plane_lat + 0.234;;//目标经纬度
	//    DPU_CCC_data_0.target_informations[0].direction = 31;//目标航向
	//    DPU_CCC_data_0.target_informations[0].speed = 77;//目标速度
	//    DPU_CCC_data_0.target_informations[0].height = 3000;//目标高度
	//    DPU_CCC_data_0.target_informations[0].tgt_found_time = 80000;   /*目标发现时间*/
	//    DPU_CCC_data_0.target_informations[0].tgt_location = 2;   /*环境类别  0=无报告;1=空中;2=水面;3=水下;4=陆地;5=空间;*/
	//    DPU_CCC_data_0.target_informations[0].simulation_flag = 1;   /*模拟标识  0=非模拟;1=模拟;*/
	//    DPU_CCC_data_0.target_informations[0].training_flag = 0;   /*演习标识  0=非演习;1=演习;*/
	//    DPU_CCC_data_0.target_informations[0].group_or_single = 1;   /*群目标单目标*/
	//    DPU_CCC_data_0.target_informations[0].tgt_locked_status = 1;   /*目标锁定状态  0=解锁;1=锁定;*/
	//    DPU_CCC_data_0.target_informations[0].tgt_attribute = 1;   /*目标属性  0=n.a;1=敌;2=我;3=友;4=中立;5=不明;*/
	//    DPU_CCC_data_0.target_informations[0].tgt_givenby = 1;   /*目标来源  0=905;1=jids;2=本机雷达;3=卫星;4=综合;*/
	//    DPU_CCC_data_0.target_informations[0].tgt_course_quality = 1;   /*航迹质量*/
	//    DPU_CCC_data_0.target_informations[0].tgt_threat_sn = 1;   /*目标威胁序号*/


	/*****************************************************/

	// 将有人机数据添加头后，发给融合
	memcpy(send_array.dataA, &DPU_CCC_data_0,sizeof(manned_aircraft_target_set));// manned_aircraft_target_set中有包头
	send_array.dataA[0] = 0x01; // 包头是0x0001,小端，则倒一下
	send_array.dataA[1] = 0x00; // 包头是0x0001,小端，则倒一下
	sendSfsMsgToQuePort(send_array.dataA, sizeof(manned_aircraft_target_set));


	//将光电数据添加头后，发给融合
	for(int i = 0; i<UAV_MAX_NUM; i++)
	{
		if(KKL_CCC_data_21_2[i].Target1status) // ?只有在有效时发送还是？
		{
			send_array.dataA[0] = 0x02; // 包头是0x0002,小端，则倒一下
			send_array.dataA[1] = 0x00; // 包头是0x0002,小端，则倒一下
			memcpy(send_array.dataA +2, &KKL_CCC_data_21_2[i],sizeof(YaoCeDataZiZhen2));// 跳过包头
			sendSfsMsgToQuePort(send_array.dataA, sizeof(YaoCeDataZiZhen2)+2);
		}

	}

}

/****************************************航线冲突处理开始***********************************************/
// 航线冲突主入口
void avoidLineCrashProc()
{
	// 冲突持续检测
	avoidLineCrashSafeTestProc();
}

// 航线冲突判断入口
void avoidLineCrashJudgeProc()
{
	//一控二场景
	if(CCC_DPU_data_3.drone_number == 2)
	{
		//冲突检测
		avoidLineCrashJudgeDouble();

		//无人机2冲突消解不合法
		if(g_lineCrashState[1].hasConflict == 2 )
		{
			//给综显发送，规避失败，此次发布无人机位置没有解，需要手动调整
			return;
		}

		//直接生成规避点并发布
		// 修改航线为盘旋点，修改任务为盘旋
		avoidLineCrashPanXuanProc();

		for(int drone_index=0;drone_index<UAV_MAX_NUM;drone_index++)
		{
			//有冲突的无人机和规避点发送给综显，冲突信息发送给综显
			if(g_lineCrashState[drone_index].hasConflict == 1)
			{
				g_lineCrashState[drone_index].recv_fabuCode = 1;
				blk_ccc_ofp_159.uav_num++;
				blk_ccc_ofp_159.avoid_info[drone_index].uavSn = drone_index+1;
				blk_ccc_ofp_159.avoid_info[drone_index].uavCode = formationId[drone_index].planeId;
				blk_ccc_ofp_159.avoid_info[drone_index].avoid_num = 1;
			}
		}
		send_blk_ccc_ofp_159();

		return;
	}

	// 冲突消解只能通过距离判断消解，冲突检测一旦有冲突，则不再检测冲突
	for(int drone_index=0;drone_index<UAV_MAX_NUM;drone_index++)
	{
		// （因为非紧密排列则需要判断// 只对有效无人机做冲突检测）
		if(formationId[drone_index].count > formationIdTimeOutCount)
		{
			continue;
		}

		if (g_lineCrashState[drone_index].hasConflict == 0)
		{
			// 发布前添加航线冲突检测（无人机当前点到第一个航路点，和有人机第一个点和第二个的航线冲突检测）
			avoidLineCrashJudgeSingle(drone_index);
//
//			// 判断有冲突则先发送航线冲突提示
//			if (g_lineCrashState[drone_index].hasConflict == 1) {
//				memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
//				blk_ccc_ofp_032.tips_type = 11;
//				if(drone_index == 0)
//				{
//					blk_ccc_ofp_032.uav_sn = 1;
//					blk_ccc_ofp_032.uav_code = 0x1005;
//				}
//				else if(drone_index == 1)
//				{
//					blk_ccc_ofp_032.uav_sn = 2;
//					blk_ccc_ofp_032.uav_code = 0x1006;
//				}
//
//				blk_ccc_ofp_032.latitude = g_lineCrashState[drone_index].avoidancePoint.latitude;
//				blk_ccc_ofp_032.longitude = g_lineCrashState[drone_index].avoidancePoint.longitude;
//				send_blk_ccc_ofp_032();
//
//				// 同时取消本次发布(所有无人机)，等待冲突时的发布指令
//				for(int m=0;m<UAV_MAX_NUM;m++)
//				{
//					g_recv_fabuCode[m] = 0;
//				}
//
//
//			}
		}


	}
}
void send_blk_ccc_ofp_159()
{
	data_length = sizeof(BLK_CCC_OFP_159);
	Send_Message(DDSTables.CCC_DPU_159.niConnectionId,0,&transaction_id, &blk_ccc_ofp_159, &message_type_id, data_length, &enRetCode);
}
//两架无人机航线冲突判断
void avoidLineCrashJudgeDouble()
{
	//有人机与无人机1/无人机2比较
	int uav1_px = 0;//无人机1选择的盘旋点
	uav1_px = avoid_uav_manned(0);
	avoid_uav_manned(1);

	//无人机1与无人机2航线段比较
	avoid_uav1_uav2();

	//无人机2最终与有人机或无人机1有冲突
	if(g_lineCrashState[1].hasConflict == 1)
	{
		//生成一个无人机1未使用的规避点，且同时避开有人机和无人机1的航线
		PolygonGeo polygon;
		AreaRectVertex rectVer;
		MabrGeo resMabrGeo;
		int uav2_px = -1;
		double pathLength_t = 0;
		double minLength_t = 1e20;
		// 有人机航线外接矩形
		polygon.vertexA[0].longitude = plane_lon;
		polygon.vertexA[0].latitude = plane_lat;
		polygon.sumVertex = blk_ofp_ccc_018.airway_point_num + 1;//加上飞机当前位置
		for (int i = 0; i < blk_ofp_ccc_018.airway_point_num; i++)
		{
			polygon.vertexA[i + 1].longitude =
					blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.longitude;
			polygon.vertexA[i + 1].latitude =
					blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.latitude;
		}
		resMabrGeo = getMabrGeo(&polygon);
		// 输出外接矩形经纬度
		for (int i = 0; i < 4; i++)
		{
			rectVer.vertexA[i] = resMabrGeo.vertexA[i];
		}

		//空域内八个区域固定区域计算
		Point center;//空域中心点
		Point px_center[8];//固定盘旋点

		//塔哈空域中心点
		center.lat = CENTER_LAT;
		center.lon = CENTER_LON;

		for(int i = 0 ; i < 8 ; i++)
		{
			calculate_endpoint(center.lat , center.lon , (45 * i) * M_PI / 180.0 , 9 , &px_center[i].lat , &px_center[i].lon );
		}

		//无人机1有规避点，航线只有两个点，使用线段作为判据
		if(g_lineCrashState[0].hasConflict == 1)
		{
			FlightRoute route1;
			FlightRoute route2;
			//开始点无人机2当前经纬度
			route1.start.latitude = formationId[1].lat;
			route1.start.longitude = formationId[1].lon;

			//开始点无人机1当前经纬度,结束点为无人机1的盘旋点
			route2.start.latitude = formationId[0].lat;
			route2.start.longitude = formationId[0].lon;
			route2.end.latitude = g_lineCrashState[0].avoidancePoint.latitude;
			route2.end.longitude = g_lineCrashState[0].avoidancePoint.longitude;

			for(int j = 0; j < 8 ; j ++)
			{
				ConflictResult tmp;
				//跳过无人机1选择过的规避点
				if(j == uav1_px)
				{
					continue;
				}
				//结束点遍历八个固定盘旋点
				route1.end.latitude = px_center[j].lat;
				route1.end.longitude = px_center[j].lon;
				//判断有无冲突,无人机2与有人机航线矩形和无人机1的航线
				if(!avoidLineCrashIsCrossRect(&route1, &rectVer) && (detectConflict(&route1, &route2, &tmp) == 0))
				{
					// 计算最短路径长度
					pathLength_t = getDistanceGeo(&route1.start, &route1.end);
					if (pathLength_t < minLength_t)
					{
						minLength_t = pathLength_t;
						uav2_px = j;
					}
				}
			}
		}
		//无人机1无规避点航线大于两个点，生成一个外接矩形
		else
		{
			//计算无人机1航线的外接矩形
			FlightRoute route1;
			PolygonGeo polygon_uav1;
			AreaRectVertex rectVer_uav1;
			MabrGeo resMabrGeo_uav1;
			unsigned int plan;
			plan = (blk_ofp_ccc_038.Plan_ID) % 3;

			//开始点无人机2当前经纬度
			route1.start.latitude = formationId[1].lat;
			route1.start.longitude = formationId[1].lon;

			// 无人机1航线外接矩形
			polygon_uav1.vertexA[0].longitude = formationId[0].lon;
			polygon_uav1.vertexA[0].latitude = formationId[0].lat;
			polygon_uav1.sumVertex = blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.waypoints_number + 1;// 加上飞机位置
			for (int i = 0; i < blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.waypoints_number; i++)
			{
				polygon_uav1.vertexA[i + 1].longitude =
						blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].longitude;
				polygon_uav1.vertexA[i + 1].latitude =
						blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].latitude;
			}
			resMabrGeo_uav1 = getMabrGeo(&polygon_uav1);
			// 输出外接矩形经纬度
			for (int i = 0; i < 4; i++)
			{
				rectVer_uav1.vertexA[i] = resMabrGeo_uav1.vertexA[i];
			}
			for(int j = 0; j < 8 ; j ++)
			{
				//跳过无人机1选择过的规避点
				if(j == uav1_px)
				{
					continue;
				}
				//结束点遍历八个固定盘旋点
				route1.end.latitude = px_center[j].lat;
				route1.end.longitude = px_center[j].lon;
				//判断有无冲突,无人机2与有人机航线矩形和无人机1的航线矩形
				if(!avoidLineCrashIsCrossRect(&route1, &rectVer) && !avoidLineCrashIsCrossRect(&route1, &rectVer_uav1))
				{
					// 计算最短路径长度
					pathLength_t = getDistanceGeo(&route1.start, &route1.end);
					if (pathLength_t < minLength_t)
					{
						minLength_t = pathLength_t;
						uav2_px = j;
					}
				}
			}
		}
		//找到有效的固定盘旋点覆盖原有盘旋点经纬度
		if(uav2_px != -1)
		{
			g_lineCrashState[1].avoidancePoint.latitude = px_center[uav2_px].lat;
			g_lineCrashState[1].avoidancePoint.longitude = px_center[uav2_px].lon;
		}
		else
		{
			//无人机2生成不出有效的规避点，返回错误
			g_lineCrashState[1].hasConflict = 2;
		}

	}

}
int avoid_uav1_uav2_period()
{
	unsigned int plan;
	unsigned int uav1_index;
	unsigned int uav2_index;
	int uav1_point_num;
	int uav2_point_num;
	FlightRoute uav1Route;
	FlightRoute uav2Route;
	plan = (blk_ofp_ccc_039.Plan_ID) % 3;

	//比较无人机1和无人机2航线有无冲突，无人机1航线使用原航线
//	uav1_point_num = blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.waypoints_number;
//	uav2_point_num = blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.waypoints_number;

	//周期计算无人机剩余航点数量，以及待飞点索引
	uav1_point_num = uav_route[0].hull_number - uav_route[0].waypoint_number - 1;
	uav1_index = uav_route[0].waypoint_number - 1 - 1;
	uav2_point_num = uav_route[1].hull_number - uav_route[1].waypoint_number - 1;
	uav2_index = uav_route[1].waypoint_number - 1 - 1;

	for (int i = 0; i < uav1_point_num; i++)
	{
		// 分类初始化无人机1航线段
		if(i==0)
		{
			// 无人机1初始位置
			uav1Route.start.longitude = formationId[0].lon;
			uav1Route.start.latitude = formationId[0].lat;
			uav1Route.end.longitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0+uav1_index].longitude;
			uav1Route.end.latitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0+uav1_index].latitude;
		}
		else
		{
			// 无人机1起始点
			uav1Route.start.longitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i-1+uav1_index].longitude;
			uav1Route.start.latitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i-1+uav1_index].latitude;

			// 无人机1终点
			uav1Route.end.longitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i+uav1_index].longitude;
			uav1Route.end.latitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i+uav1_index].latitude;
		}
		//无人1/2航线段一一比较
		for(int j =0; j<uav2_point_num; j++)
		{
			// 分类初始化无人机2航线段
			if(j == 0)
			{
				// 无人机2起始点是无人机当前位置
				uav2Route.start.longitude = formationId[1].lon;
				uav2Route.start.latitude = formationId[1].lat;

				// 无人机2终点是无人机航线第一个点
				uav2Route.end.longitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0+uav2_index].longitude;
				uav2Route.end.latitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0+uav2_index].latitude;

			}
			else
			{
				// 无人机2起始点
				uav2Route.start.longitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j-1+uav2_index].longitude;
				uav2Route.start.latitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j-1+uav2_index].latitude;

				// 无人机2终点
				uav2Route.end.longitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j+uav2_index].longitude;
				uav2Route.end.latitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j+uav2_index].latitude;
			}

			// 航段间冲突检测
			detectConflict(&uav1Route, &uav2Route, &g_lineCrashState[1]);
			//检测到冲突退出
			if(g_lineCrashState[1].hasConflict == 1)
			{
				return 1;
			}
		}
	}

	return 0;
}
//无人机1与无人机2航线段比较
void avoid_uav1_uav2()
{
	unsigned int plan;
	int uav1_point_num;
	int uav2_point_num;
	FlightRoute uav1Route;
	FlightRoute uav2Route;
	plan = (blk_ofp_ccc_038.Plan_ID) % 3;
	//有人机与无人机1有冲突
	if(g_lineCrashState[0].hasConflict == 1)
	{
		//比较无人机1和无人机2航线有无冲突，无人机1航线使用规避航线
		uav1_point_num = 1;
		uav2_point_num = blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.waypoints_number;
	}
	else
	{
		//比较无人机1和无人机2航线有无冲突，无人机1航线使用原航线
		uav1_point_num = blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.waypoints_number;
		uav2_point_num = blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.waypoints_number;
	}

	for (int i = 0; i < uav1_point_num; i++)
	{
		// 分类初始化无人机1航线段
		if(i==0)
		{
			// 无人机1初始位置
			uav1Route.start.longitude = formationId[0].lon;
			uav1Route.start.latitude = formationId[0].lat;
			//无人机1与有人机有冲突使用规避的航线
			if(g_lineCrashState[0].hasConflict == 1)
			{
				uav1Route.end.longitude = g_lineCrashState[0].avoidancePoint.longitude;
				uav1Route.end.latitude = g_lineCrashState[0].avoidancePoint.latitude;
			}
			else
			{
				uav1Route.end.longitude =
						blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;
				uav1Route.end.latitude =
						blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;
			}

		}
		else
		{
			// 无人机1起始点
			uav1Route.start.longitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i-1].longitude;
			uav1Route.start.latitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i-1].latitude;

			// 无人机1终点
			uav1Route.end.longitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].longitude;
			uav1Route.end.latitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].latitude;
		}
		//无人1/2航线段一一比较
		for(int j =0; j<uav2_point_num; j++)
		{
			// 分类初始化无人机2航线段
			if(j == 0)
			{
				// 无人机2起始点是无人机当前位置
				uav2Route.start.longitude = formationId[1].lon;
				uav2Route.start.latitude = formationId[1].lat;

				// 无人机2终点是无人机航线第一个点
				uav2Route.end.longitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;
				uav2Route.end.latitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;

			}
			else
			{
				// 无人机2起始点
				uav2Route.start.longitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j-1].longitude;
				uav2Route.start.latitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j-1].latitude;

				// 无人机2终点
				uav2Route.end.longitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j].longitude;
				uav2Route.end.latitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j].latitude;
			}

			// 航段间冲突检测
			detectConflict(&uav1Route, &uav2Route, &g_lineCrashState[1]);
			//检测到冲突退出
			if(g_lineCrashState[1].hasConflict == 1)
			{
				return;
			}
		}
	}
}
//无人机1与无人机2航线段第二次恢复比较
void avoid_uav1_uav2_second(ConflictResult* tmp)
{
	unsigned int plan;
	int uav1_point_num;
	int uav2_point_num;
	FlightRoute uav1Route;
	FlightRoute uav2Route;
	plan = (blk_ofp_ccc_039.Plan_ID) % 3;

	//比较无人机1和无人机2航线有无冲突
	uav1_point_num = blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.waypoints_number;
	uav2_point_num = blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.waypoints_number;


	for (int i = 0; i < uav1_point_num; i++)
	{
		// 分类初始化无人机1航线段
		if(i==0)
		{
			// 无人机1初始位置
			uav1Route.start.longitude = formationId[0].lon;
			uav1Route.start.latitude = formationId[0].lat;
			uav1Route.end.longitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;
			uav1Route.end.latitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;
		}
		else
		{
			// 无人机1起始点
			uav1Route.start.longitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i-1].longitude;
			uav1Route.start.latitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i-1].latitude;

			// 无人机1终点
			uav1Route.end.longitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].longitude;
			uav1Route.end.latitude =
					blk_ccc_ofp_024_cunchu[plan][0].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].latitude;
		}
		//无人1/2航线段一一比较
		for(int j =0; j<uav2_point_num; j++)
		{
			// 分类初始化无人机2航线段
			if(j == 0)
			{
				// 无人机2起始点是无人机当前位置
				uav2Route.start.longitude = formationId[1].lon;
				uav2Route.start.latitude = formationId[1].lat;

				// 无人机2终点是无人机航线第一个点
				uav2Route.end.longitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;
				uav2Route.end.latitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;

			}
			else
			{
				// 无人机2起始点
				uav2Route.start.longitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j-1].longitude;
				uav2Route.start.latitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j-1].latitude;

				// 无人机2终点
				uav2Route.end.longitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j].longitude;
				uav2Route.end.latitude =
						blk_ccc_ofp_024_cunchu[plan][1].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j].latitude;
			}

			// 航段间冲突检测
			detectConflict(&uav1Route, &uav2Route, tmp);
			//检测到冲突退出
			if(tmp->hasConflict == 1)
			{
				return;
			}
		}
	}
}

//有人机与无人机航线段比较
int  avoid_uav_manned(int drone_index)
{
	// 冲突检测
	unsigned int plan;
	double dist;
	int pointIndex = 0;
	double distTem;
	FlightRoute manRoute;
	FlightRoute uavRoute;
	static PolygonGeo polygon;
	AreaRectVertex rectVer;
	MabrGeo resMabrGeo;
	AreaRectCenter resAreaRectCenter;
	AreaRectVertex resAreaRectVertex;
	Geo uavPos;

	// 判断有人机航线点的个数
	if (blk_ofp_ccc_018.airway_point_num < 1)
	{
		return -1;
	}

	plan = (blk_ofp_ccc_038.Plan_ID) % 3;
	// 比较有人机航线段与无人机2航线段
	int man_airway_point_num = blk_ofp_ccc_018.airway_point_num;
	int uav_waypoints_number = blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number;
	for (int i = 0; i < man_airway_point_num; i++)
	{
		// 分类初始化有人机航线段
		if (i == 0)
		{
			// 有人机航线第一个点事有人机位置
			manRoute.start.longitude = plane_lon;
			manRoute.start.latitude = plane_lat;
			manRoute.end.longitude = blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.longitude;
			manRoute.end.latitude = blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.latitude;
		}
		else
		{
			manRoute.start.longitude = blk_ofp_ccc_018.waypoint_informations[i - 1].waypoint_longitude_and_latitude.longitude;
			manRoute.start.latitude = blk_ofp_ccc_018.waypoint_informations[i - 1].waypoint_longitude_and_latitude.latitude;
			manRoute.end.longitude = blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.longitude; // 最后一个点回到下标0的航点
			manRoute.end.latitude = blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.latitude;
		}
		// 有人机航线段与无人1/2航线段一一比较
		for (int j = 0; j < uav_waypoints_number; j++)
		{
			// 分类初始化无人机1/2航线段
			if (j == 0)
			{
				// 无人机起始点是无人机当前位置
				uavRoute.start.longitude = formationId[drone_index].lon;
				uavRoute.start.latitude = formationId[drone_index].lat;

				// 无人机终点是无人机航线第一个点
				uavRoute.end.longitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;
				uavRoute.end.latitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;
			}
			else
			{
				// 无人机起始点
				uavRoute.start.longitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j - 1].longitude;
				uavRoute.start.latitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j - 1].latitude;

				// 无人机终点
				uavRoute.end.longitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j].longitude;
				uavRoute.end.latitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j].latitude;
			}

			// 航段间冲突检测
			detectConflict(&manRoute, &uavRoute, &g_lineCrashState[drone_index]);
			// 无人机1/2与有人机有冲突,生成规避点
			if (g_lineCrashState[drone_index].hasConflict == 1)
			{
				/* 有人机航线外接矩形外扩得到规避矩形，然后求最近顶点作为盘旋点 */
				// 加上有人机飞机位置
				polygon.vertexA[0].longitude = plane_lon;
				polygon.vertexA[0].latitude = plane_lat;
				polygon.sumVertex = blk_ofp_ccc_018.airway_point_num + 1; // 加上飞机位置
				for (int i = 0; i < blk_ofp_ccc_018.airway_point_num; i++)
				{
					polygon.vertexA[i + 1].longitude =
						blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.longitude;
					polygon.vertexA[i + 1].latitude =
						blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.latitude;
				}

				resMabrGeo = getMabrGeo(&polygon);

				// 计算规避点
				for (int i = 0; i < 4; i++)
				{
					rectVer.vertexA[i] = resMabrGeo.vertexA[i];
				}

				resAreaRectCenter = getAreaRectCenterByVertex(&rectVer); // 顶点到中心

				resAreaRectCenter.lenLat += 6000;
				resAreaRectCenter.lenLng += 6000;

				resAreaRectVertex = getAreaRectVertexByCenter(&resAreaRectCenter);

				// 遍历最近顶点作为规避点
				uavPos.longitude =
					formationId[drone_index].lon;
				uavPos.latitude =
					formationId[drone_index].lat;
				dist = getDistanceGeo(&resAreaRectVertex.vertexA[0], &uavPos); // 目标1距离
				for (int i = 1; i < 4; i++)
				{
					distTem = getDistanceGeo(&resAreaRectVertex.vertexA[i], &uavPos); // 目标1距离
					if (dist > distTem)
					{
						dist = distTem;
						pointIndex = i;
					}
				}
				g_lineCrashState[drone_index].avoidancePoint = resAreaRectVertex.vertexA[pointIndex];

				// 再次计算更适合的规避点

				// 空域内八个区域固定区域计算
				Point center;		// 空域中心点
				Point px_center[8]; // 固定盘旋点

				// 塔哈空域中心点
				center.lat = CENTER_LAT;
				center.lon = CENTER_LON;

				for (int i = 0; i < 8; i++)
				{
					calculate_endpoint(center.lat, center.lon, (45 * i) * M_PI / 180.0, 9, &px_center[i].lat, &px_center[i].lon);
				}

				// 判断是否穿过有人机的任务区和航线段，且是最近的固定盘旋点

				/* 有人机航线外接矩形 */
				// 加上有人机飞机位置
				static PolygonGeo polygon_t;
				static MabrGeo resMabrGeo_t;
				static AreaRectVertex rectVer_hx;
				polygon_t.vertexA[0].longitude = plane_lon;
				polygon_t.vertexA[0].latitude = plane_lat;
				polygon_t.sumVertex = blk_ofp_ccc_018.airway_point_num + 1; // 加上飞机位置
				for (int i = 0; i < blk_ofp_ccc_018.airway_point_num; i++)
				{
					polygon_t.vertexA[i + 1].longitude =
						blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.longitude;
					polygon_t.vertexA[i + 1].latitude =
						blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.latitude;
				}
				resMabrGeo_t = getMabrGeo(&polygon_t);
				// 航线矩形经纬度赋值
				for (int h = 0; h < 4; h++)
				{
					rectVer_hx.vertexA[h].latitude = resMabrGeo_t.vertexA[h].latitude;
					rectVer_hx.vertexA[h].longitude = resMabrGeo_t.vertexA[h].longitude;
				}

				// 找到当前任务区
				int areaId = 0;
				static AreaRectVertex rectVer_t;
				SubdomainInfo man_taskarea;
				areaId = CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[0].task_sequence_informations[global_stage - 1].point_or_area_id; // 找到当前有人机的任务区
				for (int i = 0; i < blk_ccc_ofp_005[plan].task_are; i++)
				{
					for (int j = 0; j < blk_ccc_ofp_005[plan].task_are_hf2[i].task_are_hf_num; j++)
					{
						if (areaId == blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].Task_Are_ID)
						{
							memcpy(&man_taskarea, &blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j], sizeof(SubdomainInfo));
							// 赋值任务区
							for (int m = 0; m < 4; m++)
							{
								rectVer_t.vertexA[m].longitude = blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].signal_FC00[m].Index_Lon;
								rectVer_t.vertexA[m].latitude = blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].signal_FC00[m].Index_Lat;
							}
						}
					}
				}
				// 找出最近有效固定盘旋点
				int min_px_center = -1;
				double pathLength_t = 0;
				double minLength_t = 1e20;
				for (int j = 0; j < 8; j++)
				{
					FlightRoute route;
					// 开始点无人机当前经纬度
					route.start.latitude = formationId[drone_index].lat;
					route.start.longitude = formationId[drone_index].lon;
					// 结束点遍历八个固定盘旋点
					route.end.latitude = px_center[j].lat;
					route.end.longitude = px_center[j].lon;
					// 判断有无冲突
					if (!avoidLineCrashIsCrossRect(&route, &rectVer_hx))
					{
						// 无人机在有人机任务区内不判有人机任务区冲突
						Point uav;
						Point *man_area;
						man_area = (Point *)(&man_taskarea.signal_FC00[0]);
						uav.lat = formationId[drone_index].lat;
						uav.lon = formationId[drone_index].lon;
						int rtn = 0;
						rtn = pnpoly(man_area, 4, uav);
						if (rtn == 1)
						{
							// 计算最短路径长度
							pathLength_t = getDistanceGeo(&route.start, &route.end);
							if (pathLength_t < minLength_t)
							{
								minLength_t = pathLength_t;
								min_px_center = j;
							}
						}
						else if (!avoidLineCrashIsCrossRect(&route, &rectVer_t))
						{
							// 计算最短路径长度
							pathLength_t = getDistanceGeo(&route.start, &route.end);
							if (pathLength_t < minLength_t)
							{
								minLength_t = pathLength_t;
								min_px_center = j;
							}
						}
						else
						{
							printf("avoidLineCrashIsCrossRect erro\n");
						}
					}
					else
					{
						// 无人机在有人机航线矩形内，取无人机相对有人机反方向最近的规避点
						Point uav_point;
						Point *hx_area;
						hx_area = (Point *)(&rectVer_t.vertexA[0]);
						uav_point.lat = formationId[drone_index].lat;
						uav_point.lon = formationId[drone_index].lon;
						int rtn = 0;
						rtn = pnpoly(hx_area, 4, uav_point);
						// 无人机在有人机航线矩形内
						if (rtn == 1)
						{
							// 计算无人机朝向有人机的反方向
							double rad;
							double angle;
							rad = calculate_azimuth(uav_point.lat, uav_point.lon, plane_lat, plane_lon);
							angle = rad2deg(rad);
							// 取反方向
							angle += 180.0;
							if (angle > 360)
								angle -= 360;
							// 取夹角最小最近的规避点
							int min_angle = 360;
							for (int i = 0; i < 8; i++)
							{
								// 计算最小的夹角
								int tmp_angle = 0;
								tmp_angle = abs(angle - i * 45);
								if (tmp_angle < min_angle)
								{
									min_angle = tmp_angle;
									min_px_center = i;
								}
							}
							break;
						}
					}
				}

				// 找到有效的固定盘旋点覆盖原有盘旋点经纬度
				if (min_px_center != -1)
				{
					g_lineCrashState[drone_index].avoidancePoint.latitude = px_center[min_px_center].lat;
					g_lineCrashState[drone_index].avoidancePoint.longitude = px_center[min_px_center].lon;
				}
				// 生成规避点后退出
				return min_px_center;
			}
		}
	}
	return -1;
}
// 单架机航线冲突判断
void avoidLineCrashJudgeSingle(int drone_index)
{
	// 冲突检测
	FlightRoute manRoute;
	FlightRoute uavRoute;
	unsigned int plan;
	static PolygonGeo polygon;
	AreaRectVertex rectVer;
	MabrGeo resMabrGeo;
	AreaRectCenter resAreaRectCenter;
	AreaRectVertex resAreaRectVertex;

	Geo uavPos;
	double dist;
	int pointIndex = 0;
	double distTem;

	// 判断有人机航线点的个数
	if (blk_ofp_ccc_018.airway_point_num < 1)
	{
		return;
	}

	plan = (blk_ofp_ccc_038.Plan_ID) % 3;

	// 冲突检测(有人机航线和无人机航线充分检测)
	int man_airway_point_num = blk_ofp_ccc_018.airway_point_num;
	int uav_waypoints_number = blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number;

	for (int i = 0; i < man_airway_point_num + 1; i++)
	{ // 加1是加上有人机当前位置

		// 分类初始化有人机航线段
		if (i == 0)
		{
			// 有人机航线第一个点事有人机位置
			manRoute.start.longitude = plane_lon;
			manRoute.start.latitude = plane_lat;
			manRoute.end.longitude = blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.longitude;
			manRoute.end.latitude = blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.latitude;
		}
		else
		{
			manRoute.start.longitude = blk_ofp_ccc_018.waypoint_informations[i - 1].waypoint_longitude_and_latitude.longitude;
			manRoute.start.latitude = blk_ofp_ccc_018.waypoint_informations[i - 1].waypoint_longitude_and_latitude.latitude;
			manRoute.end.longitude = blk_ofp_ccc_018.waypoint_informations[i % man_airway_point_num].waypoint_longitude_and_latitude.longitude; // 最后一个点回到下标0的航点
			manRoute.end.latitude = blk_ofp_ccc_018.waypoint_informations[i % man_airway_point_num].waypoint_longitude_and_latitude.latitude;
		}

		for (int j = 0; j < uav_waypoints_number + 1; j++)
		{ // 加1是加上

			// 分类初始化无人机航线段
			if (j == 0)
			{
				// 无人机起始点是无人机当前位置
				uavRoute.start.longitude = formationId[drone_index].lon;
				uavRoute.start.latitude = formationId[drone_index].lat;

				// 无人机终点是无人机航线第一个点
				uavRoute.end.longitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;
				uavRoute.end.latitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;
			}
			else
			{
				// 无人机起始点
				uavRoute.start.longitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j - 1].longitude;
				uavRoute.start.latitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j - 1].latitude;

				// 无人机终点
				uavRoute.end.longitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j % uav_waypoints_number].longitude;
				uavRoute.end.latitude =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[j % uav_waypoints_number].latitude;
			}

			// 航段间冲突检测
			detectConflict(&manRoute, &uavRoute, &g_lineCrashState[drone_index]);

			if (g_lineCrashState[drone_index].hasConflict == 1)
			{

				/* 有人机航线外接矩形外扩得到规避矩形，然后求最近顶点作为盘旋点 */
				// 加上有人机飞机位置
				polygon.vertexA[0].longitude = plane_lon;
				polygon.vertexA[0].latitude = plane_lat;
				polygon.sumVertex = blk_ofp_ccc_018.airway_point_num + 1; // 加上飞机位置
				for (int i = 0; i < blk_ofp_ccc_018.airway_point_num; i++)
				{
					polygon.vertexA[i + 1].longitude =
						blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.longitude;
					polygon.vertexA[i + 1].latitude =
						blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.latitude;
				}

				resMabrGeo = getMabrGeo(&polygon);

				// 计算规避点
				for (int i = 0; i < 4; i++)
				{
					rectVer.vertexA[i] = resMabrGeo.vertexA[i];
				}

				resAreaRectCenter = getAreaRectCenterByVertex(&rectVer); // 顶点到中心

				resAreaRectCenter.lenLat += 6000;
				resAreaRectCenter.lenLng += 6000;

				resAreaRectVertex = getAreaRectVertexByCenter(&resAreaRectCenter);

				// 遍历最近顶点作为规避点
				uavPos.longitude =
					formationId[drone_index].lon;
				uavPos.latitude =
					formationId[drone_index].lat;
				dist = getDistanceGeo(&resAreaRectVertex.vertexA[0], &uavPos); // 目标1距离
				for (int i = 1; i < 4; i++)
				{
					distTem = getDistanceGeo(&resAreaRectVertex.vertexA[i], &uavPos); // 目标1距离
					if (dist > distTem)
					{
						dist = distTem;
						pointIndex = i;
					}
				}
				g_lineCrashState[drone_index].avoidancePoint = resAreaRectVertex.vertexA[pointIndex];

				// 再次计算更适合的规避点

				// 空域内八个区域固定区域计算
				Point center;		// 空域中心点
				Point px_center[8]; // 固定盘旋点

				// 塔哈空域中心点
				center.lat = CENTER_LAT;
				center.lon = CENTER_LON;

				for (int i = 0; i < 8; i++)
				{
					calculate_endpoint(center.lat, center.lon, (45 * i) * M_PI / 180.0, 9, &px_center[i].lat, &px_center[i].lon);
				}

				// 判断是否穿过有人机的任务区和航线段，且是最近的固定盘旋点

				/* 有人机航线外接矩形 */
				// 加上有人机飞机位置
				static PolygonGeo polygon_t;
				static MabrGeo resMabrGeo_t;
				static AreaRectVertex rectVer_hx;
				polygon_t.vertexA[0].longitude = plane_lon;
				polygon_t.vertexA[0].latitude = plane_lat;
				polygon_t.sumVertex = blk_ofp_ccc_018.airway_point_num + 1; // 加上飞机位置
				for (int i = 0; i < blk_ofp_ccc_018.airway_point_num; i++)
				{
					polygon_t.vertexA[i + 1].longitude =
						blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.longitude;
					polygon_t.vertexA[i + 1].latitude =
						blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.latitude;
				}
				resMabrGeo_t = getMabrGeo(&polygon_t);
				// 航线矩形经纬度赋值
				for (int h = 0; h < 4; h++)
				{
					rectVer_hx.vertexA[h].latitude = resMabrGeo_t.vertexA[h].latitude;
					rectVer_hx.vertexA[h].longitude = resMabrGeo_t.vertexA[h].longitude;
				}

				// 找到当前任务区
				int areaId = 0;
				static AreaRectVertex rectVer_t;
				SubdomainInfo man_taskarea;
				areaId = CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[0].task_sequence_informations[global_stage - 1].point_or_area_id; // 找到当前有人机的任务区
				for (int i = 0; i < blk_ccc_ofp_005[plan].task_are; i++)
				{
					for (int j = 0; j < blk_ccc_ofp_005[plan].task_are_hf2[i].task_are_hf_num; j++)
					{
						if (areaId == blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].Task_Are_ID)
						{
							memcpy(&man_taskarea, &blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j], sizeof(SubdomainInfo));
							// 赋值任务区
							for (int m = 0; m < 4; m++)
							{
								rectVer_t.vertexA[m].longitude = blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].signal_FC00[m].Index_Lon;
								rectVer_t.vertexA[m].latitude = blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].signal_FC00[m].Index_Lat;
							}
						}
					}
				}
				// 找出最近有效固定盘旋点
				int min_px_center = -1;
				double pathLength_t = 0;
				double minLength_t = 1e20;
				for (int j = 0; j < 8; j++)
				{
					FlightRoute route;
					// 开始点无人机当前经纬度
					route.start.latitude = formationId[drone_index].lat;
					route.start.longitude = formationId[drone_index].lon;
					// 结束点遍历八个固定盘旋点
					route.end.latitude = px_center[j].lat;
					route.end.longitude = px_center[j].lon;
					// 判断有无冲突,有人机航线矩形
					if (!avoidLineCrashIsCrossRect(&route, &rectVer_hx))
					{
						// 无人机在有人机任务区内不判有人机任务区冲突
						Point uav;
						Point *man_area;
						man_area = (Point *)(&man_taskarea.signal_FC00[0]);
						uav.lat = formationId[drone_index].lat;
						uav.lon = formationId[drone_index].lon;
						int rtn = 0;
						rtn = pnpoly(man_area, 4, uav);
						if (rtn == 1)
						{
							// 计算最短路径长度
							pathLength_t = getDistanceGeo(&route.start, &route.end);
							if (pathLength_t < minLength_t)
							{
								minLength_t = pathLength_t;
								min_px_center = j;
							}
						}
						else if (!avoidLineCrashIsCrossRect(&route, &rectVer_t))
						{
							// 计算最短路径长度
							pathLength_t = getDistanceGeo(&route.start, &route.end);
							if (pathLength_t < minLength_t)
							{
								minLength_t = pathLength_t;
								min_px_center = j;
							}
						}
						else
						{
							printf("avoidLineCrashIsCrossRect erro\n");
						}
					}
					else
					{
						// 无人机在有人机航线矩形内，取无人机相对有人机反方向最近的规避点
						Point uav_point;
						Point *hx_area;
						hx_area = (Point *)(&rectVer_t.vertexA[0]);
						uav_point.lat = formationId[drone_index].lat;
						uav_point.lon = formationId[drone_index].lon;
						int rtn = 0;
						rtn = pnpoly(hx_area, 4, uav_point);
						// 无人机在有人机航线矩形内
						if (rtn == 1)
						{
							// 计算无人机朝向有人机的反方向
							double rad;
							double angle;
							rad = calculate_azimuth(uav_point.lat, uav_point.lon, plane_lat, plane_lon);
							angle = rad2deg(rad);
							// 取反方向
							angle += 180.0;
							if (angle > 360)
								angle -= 360;
							// 取夹角最小最近的规避点
							int min_angle = 360;
							for (int i = 0; i < 8; i++)
							{
								// 计算最小的夹角
								int tmp_angle = 0;
								tmp_angle = abs(angle - i * 45);
								if (tmp_angle < min_angle)
								{
									min_angle = tmp_angle;
									min_px_center = i;
								}
							}
							break;
						}
					}
				}

				// 找到有效的固定盘旋点覆盖原有盘旋点经纬度
				if (min_px_center != -1)
				{
					g_lineCrashState[drone_index].avoidancePoint.latitude = px_center[min_px_center].lat;
					g_lineCrashState[drone_index].avoidancePoint.longitude = px_center[min_px_center].lon;
				}

				return; // 找到直接退出
			}
		}
	}

	return;
}

// 航线增加盘旋点处理
void avoidLineCrashPanXuanProc()
{
	unsigned int plan = blk_ofp_ccc_038.Plan_ID % 3;
	for (int drone_index = 0; drone_index < UAV_MAX_NUM; drone_index++)
	{
		// 有冲突，则发送盘旋点航线
		if (g_lineCrashState[drone_index].hasConflict == 1)
		{
			if (blk_ofp_ccc_038.Plan_ID == 0 || blk_ofp_ccc_038.Plan_ID < 0)
			{
				return;
			}

			/***********************暂时不更改任务**************************/
			// 修改任务类型（直接改发给ofp的方案CCC_DPU_data_6_Ofp，暂不考虑修改辅助决策发送的原始方案和反馈给辅助决策）
			// 备份方案到全局变量，在冲突消解时用
			memcpy(&g_lineCrashPlanBak, &CCC_DPU_data_6_Ofp[plan], sizeof(g_lineCrashPlanBak));
			// 只更改发送给ofp的方案状态，本地存储不变
			CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[drone_index + 1].task_sequence_informations[global_stage - 1].type = 14;

			// 清空航路点信息
			memset(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations, 0, sizeof(planning_information_waypoint_information));

			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.mission_type = 14;
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number = 1; // 只发盘旋点，其他点保留，但不发送
			// 设置第一个点
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].hld_idx = 1;
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].validity_of_longitude = 1;
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude =
				g_lineCrashState[drone_index].avoidancePoint.longitude;
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude_validity = 1;
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude =
				g_lineCrashState[drone_index].avoidancePoint.latitude;
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height_validity = 1;
			// 用任务点的高度作为当前悬停点的高度,如果任务点高度无效，则用默认500的高度
			if (blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].height > HEIGHT + 100)
			{

				blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].height;
			}
			else
			{
				blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height = 500;
			}
			//  速度
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed_validity = 1;
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed = 35.0;

			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].type =
				5;
			// 盘旋
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_type =
				2;
			// 顺时针
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].causality =
				3;
			// 盘旋圈数
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber = 10; // 取最大圈数
			// 半径 1000m
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius = 800;

			// 航线发送到辅助决策重新生成空域
			send_uav_airway(plan, drone_index);
		}
	}

	return;
}

void avoidLineCrashSafeTestProc()
{
	static int conflict_reslution = 0;
	// 无人机位置
	static int uav_inside[2] = {1, 1};
	for (int drone_index = 0; drone_index < UAV_MAX_NUM; drone_index++)
	{
		if (g_lineCrashState[drone_index].hasConflict == 0 || g_lineCrashState[drone_index].crashOffTime > 0)
		{
			// 非冲突状态，不处理
			continue;
		}
		// 持续检测有人机位置和有人机航线最后一个点的距离是否已到安全距离
		Point manPos = {
			DPU_CCC_data_4.manned_longitude_and_latitude_synt.longitude,
			DPU_CCC_data_4.manned_longitude_and_latitude_synt.latitude};
		int lastPointNum = blk_ofp_ccc_018.airway_point_num;
		if (lastPointNum <= 0)
		{
			lastPointNum = 1;
		}

		// 有人机到达有人机任务区域内则提示消解成功，且无人机不在无人机任务区内
		SubdomainInfo subdomainInfo;
		avoidLineCrashGetCurrentManRect(&subdomainInfo);

		// 未找到有人机任务区，使用有人机待飞点等信息判断消解条件

		// 有人机与无人机航点段做比较,无冲突
		if (0)
		{
			// 重置冲突
			uav_inside[0] = 1;
			uav_inside[1] = 1;
			// 发送冲突消解提示
			conflict_reslution = 1;
		}

		int isInRect = 0;
		Point *point = (Point *)(&subdomainInfo.signal_FC00[0]);
		isInRect = pnpoly(point, 4, manPos);

		// 计算外扩任务区
		AreaRectVertex manned_taskarea;
		AreaRectCenter manned_taskareaCenter;
		AreaRectVertex resmanned_taskarea;
		for (int m = 0; m < 4; m++)
		{
			manned_taskarea.vertexA[m].longitude = subdomainInfo.signal_FC00[m].Index_Lon;
			manned_taskarea.vertexA[m].latitude = subdomainInfo.signal_FC00[m].Index_Lat;
		}
		manned_taskareaCenter = getAreaRectCenterByVertex(&manned_taskarea); // 顶点到中心
		// 外扩两公里
		manned_taskareaCenter.lenLat += 2000;
		manned_taskareaCenter.lenLng += 2000;
		// 得到外扩后的矩形
		resmanned_taskarea = getAreaRectVertexByCenter(
			&manned_taskareaCenter);

		for (int i = 0; i < 2; i++)
		{
			Point uav;
			Point *point_uav = (Point *)(&resmanned_taskarea.vertexA[0]);
			uav.lat = formationId[i].lat;
			uav.lon = formationId[i].lon;
			uav_inside[i] = pnpoly(point_uav, 4, uav);
		}

		// 一控一场景
		if(CCC_DPU_data_3.drone_number == 1)
		{
			if (isInRect == 0 || uav_inside[drone_index] == 1) // 不再有人机任务区，则未消解,或无人机在有人机任务区内同样未消解
			{
				continue;
			}
			else
			{
				//重置冲突
				uav_inside[drone_index] = 1;
				//发送冲突消解提示
				conflict_reslution = 1;
			}
		}
		//一控二场景
		else if(CCC_DPU_data_3.drone_number == 2)
		{
			if (isInRect == 0 || uav_inside[0] == 1 || uav_inside[1] == 1) // 不再有人机任务区，则未消解,或无人机在有人机任务区内同样未消解
			{
				continue;
			}
			else
			{
				//重置冲突
				uav_inside[0] = 1;
				uav_inside[1] = 1;
				//发送冲突消解提示
				conflict_reslution = 1;
			}
		}

	}
	static int uav_hf = 0;
	// 如果已到安全距离则提示冲突消解
	if (conflict_reslution == 1) // 保证只发一次
	{
		conflict_reslution = 0;
		g_lineCrashState[0].crashOffTime++;
		g_lineCrashState[1].crashOffTime++;


		//航线发送到辅助决策重新生成空域
		unsigned int plan = blk_ofp_ccc_039.Plan_ID % 3;
		send_uav_airway(plan,0);
		send_uav_airway(plan,1);
		//切换无人机航线到恢复航线
		//就算冲突数量
		int num = 0;
		for(int drone_index = 0 ; drone_index < UAV_MAX_NUM ; drone_index ++)
		{
			if(g_lineCrashState[drone_index].hasConflict == 1)
			{
				num++;
			}
		}
		if(num == 1)
		{
			//无人机冲突数量为1，直接切换航线即可
			uav_hf = 1;
		}
		else if(num == 2)
		{
			//无人机1先切航线，后续周期检测无人机1,2再切换无人机2
			uav_hf = 2;
		}
	}

	static int hf_send_cnt = 0;
	//发送切换指令
	//只有一架机直接切换
	if(uav_hf == 1)
	{
		for(int drone_index = 0 ; drone_index < UAV_MAX_NUM ; drone_index ++)
		{
			if(g_lineCrashState[drone_index].hasConflict == 1)
			{
				//航线号为当前飞行航线
				int route_number = uav_route[drone_index].route_number;
				if(hf_send_cnt < 3)
				{
					order_data_frames.track_point_chage_orders.track_line_id = route_number;
					order_data_frames.track_point_chage_orders.track_point_id = 3;
					track_threat_frames.order_code[0] = 0x40;
					track_threat_frames.order_code[1] = 0x40;
					track_threat_frames.order_code[2] = 0x40;
					// 将指令存入遥控帧
					memcpy(&(track_threat_frames.order_data),&(order_data_frames.track_point_chage_orders),sizeof(order_data_frames.track_point_chage_orders));
					send_index = drone_index;
					send_area[drone_index] = 1;
					hf_send_cnt++;
				}
			}
		}
		if(hf_send_cnt >= 3)
		{
			//重置静态变量
			hf_send_cnt = 0;
			uav_hf = 0;

			//切换成功
			//发送当前阶段反馈 20250830new
			send_blk_ccc_ofp_025();
			//阶段数增加
			global_stage += 1;  //需考虑多架无人机的阶段数
			//应用空域
			init_blk_ctas_dtms_010();
			// 清空重规划标志量，（避免发生冲突，用户重新规划问题）
			memset(&g_lineCrashState, 0, sizeof(g_lineCrashState)*4);
		}
	}
	//先切换无人机1
	else if(uav_hf == 2)
	{
		//切换无人机1
		int route_number = uav_route[0].route_number;
		if(hf_send_cnt < 3)
		{
			order_data_frames.track_point_chage_orders.track_line_id = route_number;
			order_data_frames.track_point_chage_orders.track_point_id = 3;
			track_threat_frames.order_code[0] = 0x40;
			track_threat_frames.order_code[1] = 0x40;
			track_threat_frames.order_code[2] = 0x40;
			// 将指令存入遥控帧
			memcpy(&(track_threat_frames.order_data),&(order_data_frames.track_point_chage_orders),sizeof(order_data_frames.track_point_chage_orders));
			send_index = 0;
			send_area[0] = 1;
			hf_send_cnt++;
		}
		if(hf_send_cnt >= 3)
		{
			//进入周期判断
			uav_hf = 3;
			//重置静态变量
			hf_send_cnt = 0;
		}
	}
	//周期判断无人机1,2航线冲突，无冲突切换无人机2
	else if(uav_hf == 3)
	{
		//判断无人机1,2之间有无冲突
		int rtn = 0;
		rtn = avoid_uav1_uav2_period();
		if(rtn)
		{
			return;
		}
		else
		{
			//无人机1,2无冲突，开始切换无人机2
			uav_hf = 4;
		}
	}
	//切换无人机2
	else if(uav_hf == 4)
	{
		//切换无人机2
		int route_number = uav_route[1].route_number;
		if(hf_send_cnt < 3)
		{
			order_data_frames.track_point_chage_orders.track_line_id = route_number;
			order_data_frames.track_point_chage_orders.track_point_id = 3;
			track_threat_frames.order_code[0] = 0x40;
			track_threat_frames.order_code[1] = 0x40;
			track_threat_frames.order_code[2] = 0x40;
			// 将指令存入遥控帧
			memcpy(&(track_threat_frames.order_data),&(order_data_frames.track_point_chage_orders),sizeof(order_data_frames.track_point_chage_orders));
			send_index = 1;
			send_area[1] = 1;
			hf_send_cnt++;
		}
		if(hf_send_cnt >= 3)
		{
			//重置静态变量
			hf_send_cnt = 0;
			uav_hf = 0;

			//切换成功
			//发送当前阶段反馈 20250830new
			send_blk_ccc_ofp_025();
			//阶段数增加
			global_stage += 1;  //需考虑多架无人机的阶段数
			//应用空域
			init_blk_ctas_dtms_010();
			// 清空重规划标志量，（避免发生冲突，用户重新规划问题）
			memset(&g_lineCrashState, 0, sizeof(g_lineCrashState)*4);
		}
	}
}

void avoidLineCrashFabuProc()
{
	// 冲突时收到发布指令或消解后发布指令
	for(int drone_index = 0;drone_index<UAV_MAX_NUM;drone_index++)
	{
		// 同意发布盘旋点
		if(g_lineCrashState[drone_index].hasConflict && g_lineCrashState[drone_index].ofpLineCode == 1)
		{
			// 只在第一次收到冲突提示后的回复，才处理
			if(g_lineCrashState[drone_index].ofpLIneCodeCount==1)
			{
				// 修改航线为盘旋点，修改任务为盘旋
				avoidLineCrashPanXuanProc();
			}

			// 设置发布变量为真，使进入发布流程
			g_recv_fabuCode[drone_index] = 1;

			// 发布状态清空
			g_lineCrashState[drone_index].ofpLineCode = 0;
		}

		// 不同意发布悬停点，直接发布正常航线
		if(g_lineCrashState[drone_index].hasConflict && g_lineCrashState[drone_index].ofpLineCode == 2)
		{
			// 忽略后冲突自行消解
			g_lineCrashState[drone_index].hasConflict = 0;

			// 设置发布变量为真，使进入发布流程
			g_recv_fabuCode[drone_index] = 1;

			// 发布状态清空
			g_lineCrashState[drone_index].ofpLineCode = 0;

			g_lineCrashState[drone_index].recv_fabuCode = 0;

		}

		// 冲突消解时二次规划防冲突航线
		if (g_lineCrashState[drone_index].hasConflict && g_lineCrashState[drone_index].crashOffConfirm == 1)
		{
			// 设置发布变量为真，使进入发布流程
			g_recv_fabuCode[drone_index] = 1;

			//一控二场景,第二次发布处理
			if(CCC_DPU_data_3.drone_number == 2)
			{
				avoidLineCrashSecondDoulbe(drone_index);
			}
			else
			{
				// 第二次航线规避处理
				avoidLineCrashSecondLineProc(drone_index);
			}


			g_lineCrashState[drone_index].crashOffConfirm = 0;
		}
	}

}

void avoidLineCrashSecondDoulbe(int drone_index)
{
	AreaRectVertex rectVer;
	AreaRectCenter resAreaRectCenter;
	AreaRectVertex resAreaRectVertex;
	AreaRectVertex resAreaRectVertexBig;//更大的矩形
	int areaId = 1;
	FlightRoute route1;
	FlightRoute route2;
	FlightRoute route3;
	double minLength = 1e20;
	int bestVertex = -1;
	int bestVertex_2 = -1;
	double pathLength;


	unsigned int plan = blk_ofp_ccc_039.Plan_ID % 3;
	if(blk_ofp_ccc_039.Plan_ID <= 0)
	{
		return;
	}

	//一控二，航线第二次发布

	//生成有人机任务区矩形
	//使用备份的任务分配结果，赋回原值
	memcpy(&CCC_DPU_data_6_Ofp[plan],&g_lineCrashPlanBak , sizeof(g_lineCrashPlanBak));

	// 找到当前任务区
	areaId = CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[0].task_sequence_informations[global_stage-1].point_or_area_id;// 找到当前有人机的任务区
	for(int i = 0; i< blk_ccc_ofp_005[plan].task_are;i++)
	{
		for(int j=0; j<blk_ccc_ofp_005[plan].task_are_hf2[i].task_are_hf_num; j++)
		{
			if(areaId == blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].Task_Are_ID)
			{
				// 赋值任务区
				for (int m = 0; m < 4; m++)
				{
					rectVer.vertexA[m].longitude = blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].signal_FC00[m].Index_Lon;
					rectVer.vertexA[m].latitude = blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].signal_FC00[m].Index_Lat;
				}

				resAreaRectCenter = getAreaRectCenterByVertex(&rectVer); // 顶点到中心

				// 外扩两公里
				resAreaRectCenter.lenLat += 2000;
				resAreaRectCenter.lenLng += 2000;

				// 得到外扩后的矩形
				resAreaRectVertex = getAreaRectVertexByCenter(
						&resAreaRectCenter);

				// 再外扩500（做为过渡顶点）
				resAreaRectCenter.lenLat += 500;
				resAreaRectCenter.lenLng += 500;

				// 得到外扩后的矩形
				resAreaRectVertexBig = getAreaRectVertexByCenter(
						&resAreaRectCenter);

			}
		}
	}

	BOOL needDetour = false;
	// 无人机当前位置到任务航线起始点
	route1.start.latitude = formationId[drone_index].lat;
	route1.start.longitude = formationId[drone_index].lon;
	route1.end.latitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;
	route1.end.longitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;

	// 检查起点到终点的线段是否穿过正方形内部
	needDetour = avoidLineCrashIsCrossRect(&route1, &resAreaRectVertex);

    // 如果不需要绕行，直接输出起点到终点
    if (!needDetour)
    {
    	//计算无人机之间是否存在冲突
    	ConflictResult tmp;
    	avoid_uav1_uav2_second(&tmp);

    	//两架机互相存在冲突
    	if(tmp.hasConflict == 1)
    	{
    		//无人机之间进行规避
    		avoid_doulbe_uav(drone_index,tmp);
    	}
    	else
    	{
        	// 直接恢复即可
        	memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index], &g_lineCrashUavBak[drone_index], sizeof(g_lineCrashUavBak[drone_index]));
        	//航线发送到辅助决策重新生成空域
    		send_uav_airway(plan,drone_index);
    	}
        return ;
    }

    // 检查单顶点是否满足不穿过正方形内部，且找到最近的顶点距离
	route1.start.latitude = formationId[drone_index].lat;
	route1.start.longitude = formationId[drone_index].lon;
	route2.end.latitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;
	route2.end.longitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;

	for (int i = 0; i < 4; i++)
	{
		route1.end = resAreaRectVertexBig.vertexA[i];
		route2.start = resAreaRectVertexBig.vertexA[i];

		if (!avoidLineCrashIsCrossRect(&route1, &resAreaRectVertex) &&
			!avoidLineCrashIsCrossRect(&route2, &resAreaRectVertex))
		{
			// 计算经过各顶点的路径长度
			pathLength = getDistanceGeo(&route1.start, &route1.end) + getDistanceGeo(&route2.start, &route2.end);
			if (pathLength < minLength)
			{
				minLength = pathLength;
				bestVertex = i;
			}
		}
	}

	if (bestVertex != -1)
	{
		// 恢复无人机航线
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index], &g_lineCrashUavBak[drone_index], sizeof(g_lineCrashUavBak[drone_index]));
		// 航线发送到辅助决策重新生成空域
		send_uav_airway(plan, drone_index);
		// 拷贝第一个点到无人机航线
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
			   &g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], sizeof(planning_information_waypoint_information));
		// 更改第一个点的经纬度等为顶点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude =
			resAreaRectVertex.vertexA[bestVertex].longitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude =
			resAreaRectVertex.vertexA[bestVertex].latitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].type =
			1;
		// 一般航路点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_type =
			1;

		// 把后面的航路点赋值即可
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1],
			   &g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
			   sizeof(planning_information_waypoint_information) * 75);
		// 任务点加一
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number =
			g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.waypoints_number + 1;

		// 点的 索引值需要统一设置
		int waypoints_number = blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number;
		for (int i = 0; i < waypoints_number; i++)
		{
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].hld_idx = i + 1;
		}

		// 计算无人机之间是否存在冲突
		ConflictResult tmp;
		avoid_uav1_uav2_second(&tmp);

		// 两架机互相存在冲突
		if (tmp.hasConflict == 1)
		{
			// 无人机之间进行规避
			avoid_doulbe_uav(drone_index, tmp);
		}
		return;
	}

	// 检查双顶点是否满足不穿过正方形内部，且找到最近的顶点距离
	minLength = 1e20;
	bestVertex = -1;
	bestVertex_2 = -1;
	route1.start.latitude = formationId[drone_index].lat;
	route1.start.longitude = formationId[drone_index].lon;
	route3.end.latitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;
	route3.end.longitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;

	for (int i = 0; i < 4; i++)
	{
		route1.end = resAreaRectVertexBig.vertexA[i];
		route2.start = resAreaRectVertexBig.vertexA[i];
		route2.end = resAreaRectVertexBig.vertexA[(i + 1) % 4];
		route3.start = resAreaRectVertexBig.vertexA[(i + 1) % 4];

		if (!avoidLineCrashIsCrossRect(&route1, &resAreaRectVertex) &&
			!avoidLineCrashIsCrossRect(&route2, &resAreaRectVertex) &&
			!avoidLineCrashIsCrossRect(&route3, &resAreaRectVertex))
		{
			// 计算经过各顶点的路径长度
			pathLength = getDistanceGeo(&route1.start, &route1.end) + getDistanceGeo(&route2.start, &route2.end) +
						 getDistanceGeo(&route3.start, &route3.end);
			if (pathLength < minLength)
			{
				minLength = pathLength;
				bestVertex = i;
				bestVertex_2 = (i + 1) % 4;
			}
		}
	}

	for (int i = 0; i < 4; i++)
	{
		route1.end = resAreaRectVertexBig.vertexA[(i + 1) % 4];
		route2.start = resAreaRectVertexBig.vertexA[(i + 1) % 4];
		route2.end = resAreaRectVertexBig.vertexA[i];
		route3.start = resAreaRectVertexBig.vertexA[i];

		if (!avoidLineCrashIsCrossRect(&route1, &resAreaRectVertex) &&
			!avoidLineCrashIsCrossRect(&route2, &resAreaRectVertex) &&
			!avoidLineCrashIsCrossRect(&route3, &resAreaRectVertex))
		{
			// 计算经过各顶点的路径长度
			pathLength = getDistanceGeo(&route1.start, &route1.end) + getDistanceGeo(&route2.start, &route2.end) +
						 getDistanceGeo(&route3.start, &route3.end);
			if (pathLength < minLength)
			{
				minLength = pathLength;
				bestVertex = (i + 1) % 4;
				bestVertex_2 = i;
			}
		}
	}

	if (bestVertex != -1 && bestVertex_2 != -1)
	{
		// 恢复无人机航线
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index], &g_lineCrashUavBak[drone_index], sizeof(g_lineCrashUavBak[drone_index]));
		// 航线发送到辅助决策重新生成空域
		send_uav_airway(plan, drone_index);
		// 拷贝第一个点到无人机航线前两个点
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
			   &g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], sizeof(planning_information_waypoint_information));
		// 更改第一个点的经纬度等为顶点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude =
			resAreaRectVertex.vertexA[bestVertex].longitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude =
			resAreaRectVertex.vertexA[bestVertex].latitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].type =
			1;
		// 一般航路点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_type =
			1;

		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1],
			   &g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], sizeof(planning_information_waypoint_information));
		// 更改第一个点的经纬度等为顶点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].longitude =
			resAreaRectVertex.vertexA[bestVertex_2].longitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].latitude =
			resAreaRectVertex.vertexA[bestVertex_2].latitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].type =
			1;
		// 一般航路点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].standby_type =
			1;

		// 把后面的航路点赋值即可
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[2],
			   &g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
			   sizeof(planning_information_waypoint_information) * 75);
		// 任务点加二
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number =
			g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.waypoints_number + 2;

		// 点的 索引值需要统一设置
		int waypoints_number = blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number;
		for (int i = 0; i < waypoints_number; i++)
		{
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].hld_idx = i + 1;
		}

		// 计算无人机之间是否存在冲突
		ConflictResult tmp;
		avoid_uav1_uav2_second(&tmp);

		// 两架机互相存在冲突
		if (tmp.hasConflict == 1)
		{
			// 无人机之间进行规避
			avoid_doulbe_uav(drone_index, tmp);
		}

		return;
	}
	else
	{
		// 计算无人机之间是否存在冲突
		ConflictResult tmp;
		avoid_uav1_uav2_second(&tmp);

		// 两架机互相存在冲突
		if (tmp.hasConflict == 1)
		{
			// 无人机之间进行规避
			avoid_doulbe_uav(drone_index, tmp);
		}
		else
		{
			// 恢复无人机航线
			memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index], &g_lineCrashUavBak[drone_index], sizeof(g_lineCrashUavBak[drone_index]));
			// 航线发送到辅助决策重新生成空域
			send_uav_airway(plan, drone_index);
		}
	}
	return;
}

// 无人机机互相规避
void avoid_doulbe_uav(int drone_index, ConflictResult tmp)
{
	unsigned int plan;
	plan = (blk_ofp_ccc_039.Plan_ID) % 3;
	// 如果两架机都在进行规避
	if (g_lineCrashState[0].hasConflict == 1 && g_lineCrashState[1].hasConflict == 1)
	{
		// 比较那一架机离任务航线冲突点，较近的无人机先去执行任务，较远的无人机在规避点多盘旋几圈再去执行任务航线
		double lat1, lat2;
		double lon1, lon2;
		double rang, rang1, rang2;
		double speed; // 单位：m/s
		double time;  // 单位：s
		int task_index, avoid_index;
		int other_index;
		other_index = (drone_index == 0) ? 1 : 0;

		// 分别计算无人机到航线冲突点的距离
		lat1 = formationId[drone_index].lat;
		lon1 = formationId[drone_index].lon;
		lat2 = formationId[other_index].lat;
		lon2 = formationId[other_index].lon;
		rang1 = calculate_distances(lat1, lon1, tmp.conflictGeo.latitude, tmp.conflictGeo.longitude);
		rang2 = calculate_distances(lat2, lon2, tmp.conflictGeo.latitude, tmp.conflictGeo.longitude);
		// 根据距离判断那一架无人机去做任务，那一架无人机去规避
		if (rang1 > rang2)
		{
			avoid_index = drone_index;
			task_index = other_index;
		}
		else
		{
			avoid_index = other_index;
			task_index = drone_index;
		}
		// 当前无人机去做任务，不需要注入新航点，退出
		if (task_index == drone_index)
		{
			return;
		}

		// 给需要规避的无人机增加一个盘旋点

		// 计算做任务的无人机到冲突点的时间
		speed = blk_ccc_ofp_024_cunchu[plan][task_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed;
		time = (rang1 * 1000) / speed;
		// 计算盘旋圈数
		double speed_now; // 单位：m/s
		double time_now;  // 单位：s
		double rad = 800; // 盘旋半径，默认800m
		int circle;		  // 盘旋圈数
		// 规避无人机盘旋一圈的时间
		speed_now = blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed;
		time_now = (2 * M_PI * rad) / speed_now;
		// 圈数计算
		circle = (int)(time / time_now) + 1;
		// 增加新的盘旋点到规避无人机
		// 右移一位航线信息
		memcpy(&blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1],
			   &blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
			   sizeof(planning_information_waypoint_information) * 75);
		// 经纬度
		blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude = g_lineCrashState[avoid_index].avoidancePoint.latitude;
		blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude = g_lineCrashState[avoid_index].avoidancePoint.longitude;

		// 盘旋
		blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_type =
			2;
		// 顺时针
		blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].causality =
			3;
		// 盘旋圈数
		blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
		blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber = circle; // 取最大圈数
		// 半径 800m
		blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
		blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius = rad;
		// 航点数量加1
		blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.waypoints_number += 1;
		// 重新排序
		int waypoints_number = blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.waypoints_number;
		for (int i = 0; i < waypoints_number; i++)
		{
			blk_ccc_ofp_024_cunchu[plan][avoid_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].hld_idx = i + 1;
		}
	}
	// 单架机存在冲突，等待另一架机进入任务区，当前无人机去盘旋
	else
	{
		// 计算另一架机到任务航线冲突的点的距离
		double lat;
		double lon;
		double rang;
		double speed; // 单位：m/s
		double time;  // 单位：s
		int other_index;

		other_index = (drone_index == 0) ? 1 : 0;
		lat = formationId[other_index].lat;
		lon = formationId[other_index].lon;
		// 计算另一架无人机到冲突点的大致距离
		rang = calculate_distances(lat, lon, tmp.conflictGeo.latitude, tmp.conflictGeo.longitude);
		// 计算无人机飞过冲突点的时间
		speed = blk_ccc_ofp_024_cunchu[plan][other_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed;
		time = (rang * 1000) / speed;

		// 计算当前无人机需要盘旋的圈数
		double speed_now; // 单位：m/s
		double time_now;  // 单位：s
		double rad = 800; // 盘旋半径，默认800m
		int circle;		  // 盘旋圈数
		speed_now = blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed;
		// 计算当前无人机盘旋一圈的时间
		time_now = (2 * M_PI * rad) / speed_now;
		// 圈数
		circle = (int)(time / time_now) + 1;
		// 增加新的盘旋点到当前无人机
		// 右移一位航线信息
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1],
			   &blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
			   sizeof(planning_information_waypoint_information) * 75);
		// 经纬度
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude = g_lineCrashState[drone_index].avoidancePoint.latitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude = g_lineCrashState[drone_index].avoidancePoint.longitude;

		// 盘旋
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_type =
			2;
		// 顺时针
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].causality =
			3;
		// 盘旋圈数
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber = circle; // 取最大圈数
		// 半径 800m
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius = rad;
		// 航点数量加1
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number += 1;
		// 重新排序
		int waypoints_number = blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number;
		for (int i = 0; i < waypoints_number; i++)
		{
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].hld_idx = i + 1;
		}
	}
}
// 第二次航线规避处理
int avoidLineCrashSecondLineProc(int drone_index)
{
	AreaRectVertex rectVer;
	AreaRectCenter resAreaRectCenter;
	AreaRectVertex resAreaRectVertex;
	AreaRectVertex resAreaRectVertexBig; // 更大的矩形
	int areaId = 1;
	FlightRoute route1;
	FlightRoute route2;
	FlightRoute route3;
	double minLength = 1e20;
	int bestVertex = -1;
	int bestVertex_2 = -1;
	double pathLength;

	unsigned int plan = blk_ofp_ccc_038.Plan_ID % 3;
	if (blk_ofp_ccc_038.Plan_ID <= 0)
	{
		return 0;
	}

	// 使用备份的任务分配结果，赋回原值
	memcpy(&CCC_DPU_data_6_Ofp[plan], &g_lineCrashPlanBak, sizeof(g_lineCrashPlanBak));

	// 找到当前任务区
	areaId = CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[0].task_sequence_informations[global_stage - 1].point_or_area_id; // 找到当前有人机的任务区
	// 有人机不存在任务区
	if (areaId == 0)
	{
		// 判断有人机航线是否在无人机任务区内
		AreaRectVertex UavTaskArea;
		int uav_areaId = 0;
		uav_areaId = CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[drone_index + 1].task_sequence_informations[global_stage - 1].point_or_area_id;
		for (int i = 0; i < blk_ccc_ofp_005[plan].task_are; i++)
		{
			for (int j = 0; j < blk_ccc_ofp_005[plan].task_are_hf2[i].task_are_hf_num; j++)
			{
				if (uav_areaId == blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].Task_Are_ID)
				{
					// 赋值任务区
					for (int m = 0; m < 4; m++)
					{
						UavTaskArea.vertexA[m].longitude = blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].signal_FC00[m].Index_Lon;
						UavTaskArea.vertexA[m].latitude = blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].signal_FC00[m].Index_Lat;
					}
				}
			}
		}
		// 有人机航线段一一比较
		FlightRoute manRoute;
		int manned_point_num = 0;
		manned_point_num = blk_ofp_ccc_018.airway_point_num;
		for (int i = 0; i < manned_point_num; i++)
		{
			// 分类初始化有人机航线段
			if (i == 0)
			{
				// 有人机航线第一个点事有人机位置
				manRoute.start.longitude = plane_lon;
				manRoute.start.latitude = plane_lat;
				manRoute.end.longitude = blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.longitude;
				manRoute.end.latitude = blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.latitude;
			}
			else
			{
				manRoute.start.longitude = blk_ofp_ccc_018.waypoint_informations[i - 1].waypoint_longitude_and_latitude.longitude;
				manRoute.start.latitude = blk_ofp_ccc_018.waypoint_informations[i - 1].waypoint_longitude_and_latitude.latitude;
				manRoute.end.longitude = blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.longitude; // 最后一个点回到下标0的航点
				manRoute.end.latitude = blk_ofp_ccc_018.waypoint_informations[i].waypoint_longitude_and_latitude.latitude;
			}

			int rtn = 0;
			rtn = avoidLineCrashIsCrossRect(&manRoute, &UavTaskArea);
			if (rtn == 1)
			{
				// 返回错误提示，有人机航线在无人机任务区内，无法规避
				return -1;
			}
		}
		// 有人机不存在任务区，但航线不在无人机任务区内
		return 0;
	}
	for (int i = 0; i < blk_ccc_ofp_005[plan].task_are; i++)
	{
		for (int j = 0; j < blk_ccc_ofp_005[plan].task_are_hf2[i].task_are_hf_num; j++)
		{
			if (areaId == blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].Task_Are_ID)
			{
				// 赋值任务区
				for (int m = 0; m < 4; m++)
				{
					rectVer.vertexA[m].longitude = blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].signal_FC00[m].Index_Lon;
					rectVer.vertexA[m].latitude = blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].signal_FC00[m].Index_Lat;
				}

				resAreaRectCenter = getAreaRectCenterByVertex(&rectVer); // 顶点到中心

				// 外扩两公里
				resAreaRectCenter.lenLat += 2000;
				resAreaRectCenter.lenLng += 2000;

				// 得到外扩后的矩形
				resAreaRectVertex = getAreaRectVertexByCenter(
					&resAreaRectCenter);

				// 再外扩500（做为过渡顶点）
				resAreaRectCenter.lenLat += 500;
				resAreaRectCenter.lenLng += 500;

				// 得到外扩后的矩形
				resAreaRectVertexBig = getAreaRectVertexByCenter(
					&resAreaRectCenter);
			}
		}
	}

	BOOL needDetour = false;
	// 无人机当前位置到任务航线起始点
	route1.start.latitude = formationId[drone_index].lat;
	route1.start.longitude = formationId[drone_index].lon;
	route1.end.latitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;
	route1.end.longitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;

	// 检查起点到终点的线段是否穿过正方形内部
	needDetour = avoidLineCrashIsCrossRect(&route1, &resAreaRectVertex);

	// 如果不需要绕行，直接输出起点到终点
	if (!needDetour)
	{
		// 直接恢复即可，保留第一个规避
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index], &g_lineCrashUavBak[drone_index], sizeof(g_lineCrashUavBak[drone_index]));
		// 保留盘旋点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].hld_idx = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].validity_of_longitude = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude =
			g_lineCrashState[drone_index].avoidancePoint.longitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude_validity = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude =
			g_lineCrashState[drone_index].avoidancePoint.latitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height_validity = 1;
		// 用任务点的高度作为当前悬停点的高度,如果任务点高度无效，则用默认500的高度
		if (blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].height > HEIGHT + 100)
		{

			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height =
				blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].height;
		}
		else
		{
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height = 500;
		}
		//  速度
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed_validity = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed = 35.0;

		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].type =
			5;
		// 盘旋
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_type =
			2;
		// 顺时针
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].causality =
			3;
		// 盘旋圈数
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber = 10; // 取最大圈数
		// 半径 1000m
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius = 800;
		// 把后面的航路点赋值即可
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1],
			   &g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
			   sizeof(planning_information_waypoint_information) * 75);
		// 任务点加一
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number =
			g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.waypoints_number + 1;

		// 点的 索引值需要统一设置
		int waypoints_number = blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number;
		for (int i = 0; i < waypoints_number; i++)
		{
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].hld_idx = i + 1;
		}
		return 0;
	}

	// 检查单顶点是否满足不穿过正方形内部，且找到最近的顶点距离
	printf("uav_lati %lf uav_longi %lf\n", formationId[drone_index].lat, formationId[drone_index].lon);
	route1.start.latitude = formationId[drone_index].lat;
	route1.start.longitude = formationId[drone_index].lon;
	route2.end.latitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;
	route2.end.longitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;

	for (int i = 0; i < 4; i++)
	{
		route1.end = resAreaRectVertexBig.vertexA[i];
		route2.start = resAreaRectVertexBig.vertexA[i];

		if (!avoidLineCrashIsCrossRect(&route1, &resAreaRectVertex) &&
			!avoidLineCrashIsCrossRect(&route2, &resAreaRectVertex))
		{
			// 计算经过各顶点的路径长度
			pathLength = getDistanceGeo(&route1.start, &route1.end) + getDistanceGeo(&route2.start, &route2.end);
			if (pathLength < minLength)
			{
				minLength = pathLength;
				bestVertex = i;
			}
		}
	}

	if (bestVertex != -1)
	{
		// 恢复无人机航线
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index], &g_lineCrashUavBak[drone_index], sizeof(g_lineCrashUavBak[drone_index]));

		// 保留盘旋点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].hld_idx = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].validity_of_longitude = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude =
			g_lineCrashState[drone_index].avoidancePoint.longitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude_validity = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude =
			g_lineCrashState[drone_index].avoidancePoint.latitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height_validity = 1;
		// 用任务点的高度作为当前悬停点的高度,如果任务点高度无效，则用默认500的高度
		if (blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].height > HEIGHT + 100)
		{

			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height =
				blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].height;
		}
		else
		{
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height = 500;
		}
		//  速度
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed_validity = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed = 35.0;

		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].type =
			5;
		// 盘旋
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_type =
			2;
		// 顺时针
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].causality =
			3;
		// 盘旋圈数
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber = 10; // 取最大圈数
		// 半径 1000m
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius = 800;

		// 拷贝第二个点到无人机航线
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1],
			   &g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], sizeof(planning_information_waypoint_information));
		// 更改第一个点的经纬度等为顶点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].longitude =
			resAreaRectVertex.vertexA[bestVertex].longitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].latitude =
			resAreaRectVertex.vertexA[bestVertex].latitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].type =
			1;
		// 一般航路点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].standby_type =
			1;

		// 把后面的航路点赋值即可
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[2],
			   &g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
			   sizeof(planning_information_waypoint_information) * 75);
		// 任务点加二
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number =
			g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.waypoints_number + 2;

		// 点的 索引值需要统一设置
		int waypoints_number = blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number;
		for (int i = 0; i < waypoints_number; i++)
		{
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].hld_idx = i + 1;
		}

		return 0;
	}

	// 检查双顶点是否满足不穿过正方形内部，且找到最近的顶点距离
	minLength = 1e20;
	bestVertex = -1;
	bestVertex_2 = -1;
	route1.start.latitude = formationId[drone_index].lat;
	route1.start.longitude = formationId[drone_index].lon;
	route3.end.latitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude;
	route3.end.longitude = g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude;

	for (int i = 0; i < 4; i++)
	{
		route1.end = resAreaRectVertexBig.vertexA[i];
		route2.start = resAreaRectVertexBig.vertexA[i];
		route2.end = resAreaRectVertexBig.vertexA[(i + 1) % 4];
		route3.start = resAreaRectVertexBig.vertexA[(i + 1) % 4];

		if (!avoidLineCrashIsCrossRect(&route1, &resAreaRectVertex) &&
			!avoidLineCrashIsCrossRect(&route2, &resAreaRectVertex) &&
			!avoidLineCrashIsCrossRect(&route3, &resAreaRectVertex))
		{
			// 计算经过各顶点的路径长度
			pathLength = getDistanceGeo(&route1.start, &route1.end) + getDistanceGeo(&route2.start, &route2.end) +
					     getDistanceGeo(&route3.start, &route3.end);
			if (pathLength < minLength)
			{
				minLength = pathLength;
				bestVertex = i;
				bestVertex_2 = (i+1)%4;
			}
		}
	}

	for(int i = 0; i < 4; i++)
	{
		route1.end = resAreaRectVertexBig.vertexA[(i+1)%4];
		route2.start = resAreaRectVertexBig.vertexA[(i+1)%4];
		route2.end = resAreaRectVertexBig.vertexA[i];
		route3.start = resAreaRectVertexBig.vertexA[i];

		if (!avoidLineCrashIsCrossRect(&route1, &resAreaRectVertex) &&
			!avoidLineCrashIsCrossRect(&route2, &resAreaRectVertex) &&
		   !avoidLineCrashIsCrossRect(&route3, &resAreaRectVertex) )
		{
			// 计算经过各顶点的路径长度
			pathLength = getDistanceGeo(&route1.start, &route1.end) + getDistanceGeo(&route2.start, &route2.end) +
						 getDistanceGeo(&route3.start, &route3.end);
			if (pathLength < minLength)
			{
				minLength = pathLength;
				bestVertex = (i+1)%4;
				bestVertex_2 = i;
			}
		}
	}

	if(bestVertex != -1 && bestVertex_2 != -1)
	{
		// 恢复无人机航线
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index], &g_lineCrashUavBak[drone_index], sizeof(g_lineCrashUavBak[drone_index]));

		// 保留盘旋点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].hld_idx = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].validity_of_longitude = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].longitude =
				g_lineCrashState[drone_index].avoidancePoint.longitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude_validity = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].latitude =
				g_lineCrashState[drone_index].avoidancePoint.latitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height_validity = 1;
		// 用任务点的高度作为当前悬停点的高度,如果任务点高度无效，则用默认500的高度
		if(blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].height > HEIGHT + 100)
		{

			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height =
					blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].height;
		}
		else
		{
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].height = 500;
		}
		//  速度
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed_validity = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].speed = 35.0;

		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].type =
				5;
		// 盘旋
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_type =
				2;
		//顺时针
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].causality =
				3;
		// 盘旋圈数
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber = 10;// 取最大圈数
		// 半径 1000m
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0].standby_radius = 800;

		// 拷贝第一个点到无人机航线前三个点
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1],
				&g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1], sizeof(planning_information_waypoint_information));
		// 更改第一个点的经纬度等为顶点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].longitude =
			resAreaRectVertex.vertexA[bestVertex].longitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].latitude =
			resAreaRectVertex.vertexA[bestVertex].latitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].type =
			1;
		// 一般航路点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[1].standby_type =
			1;

		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[2],
			   &g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0], sizeof(planning_information_waypoint_information));
		// 更改第一个点的经纬度等为顶点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[2].longitude =
			resAreaRectVertex.vertexA[bestVertex_2].longitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[2].latitude =
			resAreaRectVertex.vertexA[bestVertex_2].latitude;
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[2].type =
			1;
		// 一般航路点
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[2].standby_type =
			1;

		// 把后面的航路点赋值即可
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[3],
			   &g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
			   sizeof(planning_information_waypoint_information) * 75);
		// 任务点加三
		blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number =
			g_lineCrashUavBak[drone_index].individual_drone_routing_programs.planning_informations.waypoints_number + 3;

		// 点的 索引值需要统一设置
		int waypoints_number = blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.waypoints_number;
		for (int i = 0; i < waypoints_number; i++)
		{
			blk_ccc_ofp_024_cunchu[plan][drone_index].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[i].hld_idx = i + 1;
		}

		return 0;
	}
	else
	{
		// 恢复无人机航线
		memcpy(&blk_ccc_ofp_024_cunchu[plan][drone_index], &g_lineCrashUavBak[drone_index], sizeof(g_lineCrashUavBak[drone_index]));
		// 航线发送到辅助决策重新生成空域
		send_uav_airway(plan, drone_index);
	}
	return 0;
}

// 判读航线是否穿过正方体
BOOL avoidLineCrashIsCrossRect(FlightRoute *route1, AreaRectVertex *resAreaRectVertex)
{
	BOOL needDetour = false;
	FlightRoute route2;
	ConflictResult temConflictResult; // 冲突结果反馈，无意义
	int temResInt = 0;

	for (int i = 0; i < 4; i++)
	{
		route2.start = resAreaRectVertex->vertexA[i];
		route2.end = resAreaRectVertex->vertexA[(i + 1) % 4];
		temResInt = detectConflict(route1, &route2, &temConflictResult);
		if (temResInt == 1)
		{
			needDetour = true;
		}
	}

	return needDetour;
}

void avoidLineCrashGetCurrentManRect(SubdomainInfo *subdomainInfo) // 找到当前有人机任务区
{
	int areaId = -1;
	unsigned int plan = blk_ofp_ccc_039.Plan_ID % 3;
	if (blk_ofp_ccc_039.Plan_ID <= 0)
	{
		return;
	}

	// 找到当前任务区
	areaId = CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[0].task_sequence_informations[global_stage - 1].point_or_area_id; // 找到当前有人机的任务区
	for (int i = 0; i < blk_ccc_ofp_005[plan].task_are; i++)
	{
		for (int j = 0; j < blk_ccc_ofp_005[plan].task_are_hf2[i].task_are_hf_num; j++)
		{
			if (areaId == blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j].Task_Are_ID)
			{
				memcpy(subdomainInfo, &blk_ccc_ofp_005[plan].task_are_hf2[i].signal_FC00[j], sizeof(SubdomainInfo));
			}
		}
	}
}

/****************************************航线冲突处理结束***********************************************/

/****************************************载荷重规划处理开始***********************************************/
// 载荷状态检测
void payload_detection()
{
	for (int uav_index = 0; uav_index < 4; uav_index++)
	{
		unsigned int plan = blk_ofp_ccc_038.Plan_ID % 3;
		// 当前任务为磁探
		if (blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.mission_type == 5)
		{
			// 检测载荷是否可用
			if (g_payload_replan[uav_index].CT_Online == 0 || g_payload_replan[uav_index].CT_Status == 0)
			{
				// 发起重规划
				memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
				// 载荷重规划磁探失效
				blk_ccc_ofp_032.uav_code = formationId[uav_index].planeId;
				blk_ccc_ofp_032.uav_sn = uav_index + 1;
				blk_ccc_ofp_032.tips_type = 14;
				// send_blk_ccc_ofp_032();
			}
		}
	}
}

// 修改任务分配结果
void payload_replan()
{
	for (int uav_index = 0; uav_index < 4; uav_index++)
	{
		unsigned int plan = blk_ofp_ccc_038.Plan_ID % 3;
		unsigned int task = blk_ofp_ccc_038.stage_id - 1;
		// 当前任务为磁探
		if (blk_ccc_ofp_024_cunchu[plan][uav_index].individual_drone_routing_programs.planning_informations.mission_type == 5)
		{
			if (g_payload_replan[uav_index].CT_Online == 0 || g_payload_replan[uav_index].CT_Status == 0)
			{
				// 无人机进入重规划
				g_payload_replan[uav_index].Replan = 1;
				CCC_DPU_data_6[plan].formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[task].type = 7;
			}
		}
	}
}

// 修改后载荷任务发布
void payload_task()
{
	// 发布航线时停止查询
	hx_cx_flag = 0;
	scheme_generation_state(1, 2, 1, 2); // 返回发布状态到综显
	// 找到方案编号索引
	unsigned int plan = (blk_ofp_ccc_039.Plan_ID) % 3;

	// 初始化无人机数据,接收标志
	for (int i = 0; i < 4; i++)
	{
		memset(&uav_send[i], 0, sizeof(UAV_SEND));
		memset(&s4D_frame_40[i], 0, sizeof(int));
		memset(&s4D_frame_38[i], 0, sizeof(int));
	}

	for (unsigned int i = 0; i < 4; i++)
	{
		uav_send[i].drone_id =
			CCC_DPU_data_6[plan].formation_synergy_mission_programs[i + 1].platform_code;

		// 找到任务类型参数索引
		for (int j = 0; j < 8; j++)
		{
			if (task_param[j].type_id == CCC_DPU_data_6[plan].formation_synergy_mission_programs[i + 1].task_sequence_informations[blk_ofp_ccc_039.stage_id].type)
			{
				// 找到索引退出
				uav_send[i].task_index = j;
				break;
			}
		}
		// 确认需要进入重规划的无人机
		if (g_payload_replan[i].Replan == 1)
		{
			// 航路点数量
			uav_send[i].waypoints_number =
				blk_ccc_ofp_024_cunchu[plan][i].individual_drone_routing_programs.planning_informations.waypoints_number;
			// 存入当前任务的每个航路点经纬度
			memcpy(&uav_send[i].waypoint_informations[0],
				   &blk_ccc_ofp_024_cunchu[plan][i].individual_drone_routing_programs.planning_informations.planning_information_waypoint_informations[0],
				   sizeof(planning_information_waypoint_information) * uav_send[i].waypoints_number);
		}

		// 发送准备
		uav_send[i].send_ready = 1;
		uav_send[i].first_flag = 0;
		// 判断该无人机是否有航路点，没有航路点则放弃发送
		if (uav_send[i].waypoints_number == 0)
		{
			// 清空该无人机数据
			memset(&uav_send[i], 0, sizeof(UAV_SEND));
		}
	}
	// 初始化发送参数，防止上次指令切换之后的帧变化影响
	frame_count = 1;
	track_point_count = -1;
	send_count = 0;
	g_fabu_send_uav_num = 0;
	g_fabu_switch_timeout = 0;
	g_fabu_switch_ack_cnt = 0;
}

// 监测正在飞行的无人机载荷是否失效
void payload_listen()
{
	for (int uav_index = 0; uav_index < 4; uav_index++)
	{
		if (uav_route[uav_index].task_type == 7 && formulate_single == 1)
		{
			if (g_payload_replan[uav_index].Replan == 0 &&
				(g_payload_replan[uav_index].CT_Online == 0 || g_payload_replan[uav_index].CT_Status == 0))
			{
				// 运行时冲突
				g_payload_replan[uav_index].Replan = 1;
				// 发起重规划
				memset(&blk_ccc_ofp_032, 0, sizeof(blk_ccc_ofp_032));
				// 载荷重规划磁探失效
				blk_ccc_ofp_032.uav_code = formationId[uav_index].planeId;
				blk_ccc_ofp_032.uav_sn = uav_index + 1;
				blk_ccc_ofp_032.tips_type = 31;
				// send_blk_ccc_ofp_032();
			}
		}
	}
}
/****************************************载荷重规划处理结束***********************************************/

/*************************收辅助决策方案和航线等特殊处理未与综显通信icd格式：**********************/
/*** 因为辅助决策是紧密排列，但是多无人机情况下，要求同无人机一次任务内，编队编号和存储位置对应且不变。所以会出现
 * 有一架机时，不再数组第一个位置，所以，需要将辅助决策生成的方案根据编队编号调整到相应数组下标下（综显是根据下标取得）******
 * ************************************************/
void castCtasToOfpPlan(BLK_CCC_OFP_019 *p_blk_ccc_ofp_019)
{

	// 不存在非紧密编队情况
	if (!castCtasToOfpIsNeeded())
	{
		// 保存到发送给ofp的保存方案中
		unsigned int plan = blk_ccc_ofp_019.plan_id % 3;
		memcpy(&CCC_DPU_data_6_Ofp[plan], p_blk_ccc_ofp_019, sizeof(BLK_CCC_OFP_019));
		return;
	}

	// 特殊处理，在有1006的情况下，但是没有1005时，将1006分配的内容都放在数组第二个上
	memcpy(&p_blk_ccc_ofp_019->formation_synergy_mission_programs[2], &p_blk_ccc_ofp_019->formation_synergy_mission_programs[1],
		   sizeof(p_blk_ccc_ofp_019->formation_synergy_mission_programs[1]));
	p_blk_ccc_ofp_019->formation_synergy_mission_programs[2].platform_sn = 2; // 更改编号（一个任务内无人机都固定）

	// 清空数组第一个
	memset(&p_blk_ccc_ofp_019->formation_synergy_mission_programs[1], 0,
		   sizeof(p_blk_ccc_ofp_019->formation_synergy_mission_programs[1]));

	// 调整后保存到发送给ofp的保存方案中
	unsigned int plan = blk_ccc_ofp_019.plan_id % 3;
	memcpy(&CCC_DPU_data_6_Ofp[plan], p_blk_ccc_ofp_019, sizeof(BLK_CCC_OFP_019));
}

void castCtasToOfpLine(BLK_CCC_OFP_024 *p_blk_ccc_ofp_024)
{

	// 不存在非紧密编队情况
	if (!castCtasToOfpIsNeeded())
	{
		return;
	}

	// 将1006的航线内的编号改成2
	p_blk_ccc_ofp_024->individual_drone_routing_programs.drone_serial_number = 2;
}

// 返回当前编队是否存在非紧密排列，（这种情况需要特殊处理）
BOOL castCtasToOfpIsNeeded()
{
	// 当前判定依据：第一个无人机不存在，第二个无人机存在，则任务编队存在非紧密编队
	if (formationId[0].count > formationIdTimeOutCount && formationId[1].count <= formationIdTimeOutCount)
	{
		return TRUE;
	}

	return FALSE;
}

/*************************收辅助决策方案和航线等特殊处理未与综显通信icd格结束**********************/

// 找到当前编号飞机的当前任务
int getUavCurrentTask(int drone_index)
{
	unsigned int plan = blk_ofp_ccc_039.Plan_ID % 3;
	if (blk_ofp_ccc_039.Plan_ID <= 0)
	{
		return 0;
	}

	// 当前任务类型
	int type = CCC_DPU_data_6_Ofp[plan].formation_synergy_mission_programs[drone_index].task_sequence_informations[global_stage - 1].type;
	return type;
}


