#include "maintask.h"
#include "math.h"
#include <float.h>
#include "../libSourceCode/commDas/geoGrid.h"
#define EARTH_RADIUS 6371000.0
#define DEG_TO_RAD (3.1415926535897932/180.0)
#define RAD_TO_DEG (180.0/3.1415926535897932)
/*
 * 主线程
 *
 * 主要功能集成函数
 * */
void main_task(){
	/*
	 * 战术战法规划
	 *
	 * 接收来自动态任务管理软件的信息并赋值,然后组织战术战法再发送（当前仅搜潜方案的八个战术战法）
	 * DTMS-CTAS    CTAS-DTMS
	 * 当前进度：完成一控一单方案生成测试，三方案生成逻辑待完善 TODO
	 *         一控二测试待测试 TODO
	 *         一控四逻辑待615给出后完善　TODO
	 * */
	//心跳打印
	static int heartbit = 0;
	heartbit++;
	if(heartbit % 100 == 0)
	{
		printf("*** CTAS online ***\n");
	}

	//初始化
	static int init = 0;
	if(init == 0)
	{
		init ++;
		initVar();
	}

	recv_blk_dtms_ctas_010();

	formulate_moduel();//全局任务规划

	signal_moduel();//单无人机指控/单任务指控
	/*
	 * 任务重规划
	 *
	 * 规划---发送---功能未测试　TODO
	 * */
	task_replanning();

	/*
	 * 集群规划
	 *
	 * 接收集群信息，进行集群航路规划
	 * 已完成测试
	 * */
	UAV_jiqun();

	/*
	 * 协同攻击
	 *
	 * 功能还没有设计
	 * */
	//    attack_commend();
}



void initVar() {

	memset(&blk_dtms_ctas_001, 0, sizeof(BLK_DTMS_CTAS_001));                                 //战术战法推荐指令

	memset(&global_mission_planning_commandss, 0, sizeof(global_mission_planning_commands));             //全局任务规划命令(icd同DTMS和综显)
	memset(&drone_state_informations, 0, sizeof(drone_state_information));                               //任务状态信息(icd同DTMS和综显)
	memset(&integrated_postures, 0, sizeof(integrated_posture));                                         //综合态势(icd同DTMS和综显)
	memset(&area_sky_informations, 0, sizeof(area_sky_information));                                     //任务区/空域信息(icd同DTMS和综显)

	memset(&CTAS_DTMS_data_tacticsResults, 0, sizeof(zhanshuzhanfa_result));                             //战术战法推荐结果

	memset(&information_on_the_results_of_taskings, 0, sizeof(Information_the_results_of_tasking));   //任务分配结果信息 (icd同DTMS和综显)

	memset(&CTAS_DTMS_data_mannedRoute, 0, sizeof(manned_aircraft_route_information));       //有人机航路规划结果
	memset(&CTAS_DTMS_data_UAVRoute, 0, sizeof(drone_route_confirmation));                               //无人机航路规划结果

	//初始化任务高度
	for(int i = 0; i < 2 ; i ++)
	{
		blk_dtms_ctas_010[i].FBZS = 500 + i * 200;
		blk_dtms_ctas_010[i].CTSS = 500 + i * 200;
		blk_dtms_ctas_010[i].CTGZ = 500 + i * 200;
		blk_dtms_ctas_010[i].GDSS = 500 + i * 200;
		blk_dtms_ctas_010[i].GDGZ = 500 + i * 200;
		blk_dtms_ctas_010[i].BDFX = 560 + i * 220;
		blk_dtms_ctas_010[i].GJ = 250 + i * 200;
		blk_dtms_ctas_010[i].PX = 500 + i * 200;
	}
}

void recv_blk_dtms_ctas_010()
{
	message_size = 4096;
	BLK_DTMS_CTAS_010 tmp[2];
	Receive_Message(DDSTables.BLK_DTMS_CTAS_010.niConnectionId, 0, &transaction_id, &tmp, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		for(int i = 0 ; i < 2 ; i ++)
		{
			//浮标帧收
			if(tmp[i].FBZS > 0)
				blk_dtms_ctas_010[i].FBZS = tmp[i].FBZS;
			//磁探搜索
			if(tmp[i].CTSS > 0)
				blk_dtms_ctas_010[i].CTSS = tmp[i].CTSS;
			//磁探跟踪
			if(tmp[i].CTGZ > 0)
				blk_dtms_ctas_010[i].CTGZ = tmp[i].CTGZ;
			//光电搜索
			if(tmp[i].GDSS > 0)
				blk_dtms_ctas_010[i].GDSS = tmp[i].GDSS;
			//光电跟踪
			if(tmp[i].GDGZ > 0)
				blk_dtms_ctas_010[i].GDGZ = tmp[i].GDGZ;
			//编队飞行
			if(tmp[i].BDFX > 0)
				blk_dtms_ctas_010[i].BDFX = tmp[i].BDFX;
			//攻击
			if(tmp[i].GJ > 0)
				blk_dtms_ctas_010[i].GJ = tmp[i].GJ;
			//盘旋
			if(tmp[i].PX > 0)
				blk_dtms_ctas_010[i].PX = tmp[i].PX;
		}
	}
}

/********************************战术战法规划**************************************************/

void task_area_replan()
{
	//初始化
	memset( &taskarea_division , 0 , sizeof(BLK_CCC_OFP_005) );
	//3.10 任务区/空域信息
	if(blk_dtms_ctas_001.task_reg_num == 1 && blk_dtms_ctas_001.solider_num == 3)
	{
		int divi_num = integrated_postures.drone_num;
		//任务区数量为1时，划分任务区（暂时划分无人机数量相等的任务区）+有人机
		task_area_division(&blk_dtms_ctas_001.region_infos[0],divi_num);

		taskarea_division.task_are = 1;/*划分任务区个数 对几个任务区划分*/
		taskarea_division.task_are_hf2[0].Task_Are_ID = blk_dtms_ctas_001.region_infos[0].reg_id;
		taskarea_division.task_are_hf2[0].task_are_hf_num = divi_num;

		for(int i = 0 ; i < divi_num ; i ++)
		{
			taskarea_division.task_are_hf2[0].signal_FC00[i].Task_Are_ID = i + 1;
			for(int j = 0 ; j < 4 ; j ++)
			{
				taskarea_division.task_are_hf2[0].signal_FC00[i].signal_FC00[j].Index_Lat = area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude;
				taskarea_division.task_are_hf2[0].signal_FC00[i].signal_FC00[j].Index_Lon= area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude;
			}
		}
	}
	else
	{
		area_sky_informations.area_number = blk_dtms_ctas_001.task_reg_num;//任务区数量
		for(int i = 0;i < area_sky_informations.area_number;i++){
			area_sky_informations.area_informations[i].area_code = blk_dtms_ctas_001.region_infos[i].reg_id;//区域编号
			area_sky_informations.area_informations[i].area_type = blk_dtms_ctas_001.region_infos[i].reg_type;//区域类型
			area_sky_informations.area_informations[i].area_source = blk_dtms_ctas_001.region_infos[i].reg_sour;//区域来源
			area_sky_informations.area_informations[i].area_shape = blk_dtms_ctas_001.region_infos[i].reg_shape;//区域形状
			//        area_sky_informations.area_informations[i].drone_number_valid_bit = DTMS_CTAS_data_tacticsRecommend->region_infos[i].kongyu_belong_to_uav_valid;
			area_sky_informations.area_informations[i].drone_numbe = blk_dtms_ctas_001.region_infos[i].kongyu_belong_to_uav_id;//空域所属无人机序号
			area_sky_informations.area_informations[i].upper_height_limit_valid_bit = blk_dtms_ctas_001.region_infos[i].reg_top_of_hei_valid;//区域高度上限有效位
			area_sky_informations.area_informations[i].lower_height_limit_valid_bit = blk_dtms_ctas_001.region_infos[i].reg_down_of_hei_valid;//区域高度下限有效位
			area_sky_informations.area_informations[i].upper_height_limit = blk_dtms_ctas_001.region_infos[i].top_of_hei;//区域高度上限
			area_sky_informations.area_informations[i].lower_height_limit = blk_dtms_ctas_001.region_infos[i].down_of_hei;//区域高度下限
			//圆区域赋值
			area_sky_informations.area_informations[i].cycles.radius = blk_dtms_ctas_001.region_infos[i].reg_circle.radious;//圆半径
			area_sky_informations.area_informations[i].cycles.longitude = blk_dtms_ctas_001.region_infos[i].reg_circle.center_lon_lat.longitude;//圆形经度
			area_sky_informations.area_informations[i].cycles.latitude = blk_dtms_ctas_001.region_infos[i].reg_circle.center_lon_lat.latitude;//圆形纬度
			//多边形区域赋值
			area_sky_informations.area_informations[i].polygonals.point_number = blk_dtms_ctas_001.region_infos[i].reg_ploygen.point_num;//多边形点数
			for(int j = 0;j < area_sky_informations.area_informations[i].polygonals.point_number;j++){
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude = blk_dtms_ctas_001.region_infos[i].reg_ploygen.points_lon_lat[j].longitude;//点经度
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude = blk_dtms_ctas_001.region_infos[i].reg_ploygen.points_lon_lat[j].latitude;//点纬度
			}
		}
	}

}
//有人机不参与任务
void yz_dispel_manned_strategy()
{
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定


	//方案总时间---待定 total_program_time
	//任务平台个数---3.2 无人机个数n
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number;

	/*
	 * 无人机任务信息分配结果
	 * */
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数 + 返航
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= 1;
		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 6;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[i -1].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 0;
		}

	}

	//最近的无人机去磁探跟踪，另一架机去盘旋
	if(drone_state_informations.drone_number == 2)
	{
		double uav1,uav2;
		uav1 = calculate_distances(integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.latitude,
				integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.longitude,
				blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.latitude,
				blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.longitude);

		uav2 = calculate_distances(integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.latitude,
				integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.longitude,
				blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.latitude,
				blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.longitude);
		//离得远的无人机索引
		int dorne_index = 0;
		dorne_index = (uav1 > uav2) ? 0:1;

		information_on_the_results_of_taskings.formation_synergy_mission_programs[dorne_index].task_sequence_informations[0].sequence_type = 14;
	}
	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}

	//方案时长航程计算
	uav_time_range_calc();
	information_on_the_results_of_taskings.program_attributes = 3;
	renwu_guihua_flag = 1;
	buoy_suspended_flag = 1;
	wurenji_hangxian_flag = 1;
}
//有人机不参与任务
void dispel_manned_strategy()
{
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定


	//方案总时间---待定 total_program_time
	//任务平台个数---3.2 无人机个数n
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number;

	/*
	 * 无人机任务信息分配结果
	 * */
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数 + 返航
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= 1;
		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 5;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[i -1].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 0;
		}

	}

	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}

	//方案时长航程计算
	uav_time_range_calc();
	information_on_the_results_of_taskings.program_attributes = 3;
	//无人机任务区分派

	//预分配任务区
	information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[0].area_code;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = area_sky_informations.area_informations[1].area_code;
	//计算无人机1,2的航线段
	UAVRouteGeneration();
	FlightRoute uav_route[2];
	for(int uav = 0 ; uav < 2 ; uav ++)
	{
		uav_route[uav].start.latitude = integrated_postures.integrated_posture_drone_informations[uav].drone_longitude_and_latitude.latitude;
		uav_route[uav].start.longitude = integrated_postures.integrated_posture_drone_informations[uav].drone_longitude_and_latitude.longitude;
		uav_route[uav].end.latitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[0].planning_information_waypoint_informations[0].latitude;
		uav_route[uav].end.longitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[0].planning_information_waypoint_informations[0].longitude;
	}
	//距离计算
	double uav1,uav2;
	uav1 = calculate_distances(uav_route[0].start.latitude,uav_route[0].start.longitude,uav_route[0].end.latitude,uav_route[0].end.longitude);
	uav2 = calculate_distances(uav_route[1].start.latitude,uav_route[1].start.longitude,uav_route[1].end.latitude,uav_route[1].end.longitude);

	//查看是否有冲突
	int rtn = 0;
	rtn = detectConflict(&uav_route[0],&uav_route[1]);
	//存在冲突,交换任务区
	if(rtn == 1 || (fabs(uav_route[0].end.latitude  - uav_route[1].end.latitude)  < 1e-7 &&
					fabs(uav_route[0].end.longitude - uav_route[1].end.longitude) < 1e-7))
	{
		information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[1].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = area_sky_informations.area_informations[0].area_code;
	}

	renwu_guihua_flag = 1;
	buoy_suspended_flag = 1;
	wurenji_hangxian_flag = 1;
}
/*
 * 战术一
 * 完成战术的数据组织
 * */
void firstStrategy(){
	missionDistributeInformation();     //任务分配结果信息
}

/*
 * 3.6 任务分配结果信息
 * */
void missionDistributeInformation(){
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定


	//方案总时间---待定 total_program_time
	//任务平台个数---3.2 无人机个数n + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number + 1;

	/*
	 * 有人机任务信息分配结果
	 * */
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_model = 1;//平台型号--有人机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID
	//平台任务时---待定
	//平台子任务个数---任务区/应召点个数
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks
	= global_mission_planning_commandss.area_point_num;
	mannedAreaSorted();//组织任务区顺序
	//任务序列信息
	for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 2;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 1;
	}


	/*
	 * 无人机任务信息分配结果
	 * */
	int uav_task_num[4] = {0,0,0,0};
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数 + 返航
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= global_mission_planning_commandss.area_point_num;
		uav_task_num[i-1] = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;
		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[i -1 ].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 0;
		}

	}
	//确定第一轮任务时无人机的任务分配
	UAVAreaDistribute();
	//将后续任务全都赋值为第一个任务（主要原因是如果后面有人机未经过当前无人机的位置，则无人机不应该再移动）
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		for(int j = 1;j < global_mission_planning_commandss.area_point_num;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].target_number;

		}
	}
	//确定余下的任务分配
	for(int i = 1;i < global_mission_planning_commandss.area_point_num;i++){//外层为有人机后续任务编号
		for(int j = 1;j <= drone_state_informations.drone_number;j++){//如果无人机的第一个任务的任务区与有人机后续任务区的编号相同,则把无人机后续的任务区全部赋值成有人机当前任务区编号
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[j].task_sequence_informations[0].target_number == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number){
				for(int k = i;k < global_mission_planning_commandss.area_point_num;k++){
					information_on_the_results_of_taskings.formation_synergy_mission_programs[j].task_sequence_informations[k].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i - 1].target_number;
				}
			}
		}
	}

	//赋值待机
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks - 1;j++){
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number == 0){
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 12;
			}
		}
	}

	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}

	//方案时长航程计算
	uav_time_range_calc();

}
//有/无人机分派方式
void uav_dispatch()
{
	if(drone_state_informations.drone_number == 1)
	{
		// 遍历8个阶段，每个阶段每个平台的任务区分开
		uav_area_distribute_proc();
	}
	else if(drone_state_informations.drone_number == 2)
	{
		//三架飞机，任务区无冲突分配
		uav_area_distribute_third();
	}
}
//三架飞机，任务区无冲突分配
void uav_area_distribute_third()
{
	//找出有人机最近的任务区1或3(任务区2会有交叉问题暂时不考虑)
	AreaRectVertex tmp;
	AreaRectCenter area_1;
	AreaRectCenter area_2;
	double area1_dis = 0;
	double area2_dis = 0;
	//有人机到任务区1的距离
	for(int k = 0; k < 4 ; k ++)
	{
		tmp.vertexA[k].latitude = area_sky_informations.area_informations[0].polygonals.point_coordinates[k].latitude;
		tmp.vertexA[k].longitude = area_sky_informations.area_informations[0].polygonals.point_coordinates[k].longitude;
	}
	area_1 = getAreaRectCenterByVertex(&tmp);
	area1_dis = calculate_distances(area_1.center.latitude,area_1.center.longitude,integrated_postures.latitude,integrated_postures.longitude);

	//有人机到任务区3的距离
	for(int k = 0; k < 4 ; k ++)
	{
		tmp.vertexA[k].latitude = area_sky_informations.area_informations[2].polygonals.point_coordinates[k].latitude;
		tmp.vertexA[k].longitude = area_sky_informations.area_informations[2].polygonals.point_coordinates[k].longitude;
	}
	area_2 = getAreaRectCenterByVertex(&tmp);
	area2_dis = calculate_distances(area_2.center.latitude,area_2.center.longitude,integrated_postures.latitude,integrated_postures.longitude);

	//有人机选择的任务区,无人机可选择的任务区
	int manned_area = 0;
	int choice1_area = 0;
	int choice2_area = 0;
	//任务区1更近
	if(area1_dis < area2_dis)
	{
		manned_area = area_sky_informations.area_informations[0].area_code;
		choice1_area = area_sky_informations.area_informations[1].area_code;
		choice2_area = area_sky_informations.area_informations[2].area_code;
	}
	//任务区3更近
	else
	{
		manned_area = area_sky_informations.area_informations[2].area_code;
		choice1_area = area_sky_informations.area_informations[0].area_code;
		choice2_area = area_sky_informations.area_informations[1].area_code;
	}

	//存在无冲突标志位
	int distribute = -1;
	//总距离
	double total_dis_1 = 0;
	double total_dis_2 = 0;
	//有人机选择任务区
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].target_number = manned_area;
	//计算有人机航线段
	mannedAircraftRoutesGeneration();
	FlightRoute manned;
	manned.start.latitude = integrated_postures.latitude;
	manned.start.longitude = integrated_postures.longitude;
	manned.end.latitude = CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoint_informations[0].latitude;
	manned.end.longitude = CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoint_informations[0].longitude;
	//遍历两种无人机任务区分配方法
	for(int i = 0 ; i < 2 ; i ++)
	{
		//分法一：无人机1在选择任务区1，无人机2在选择任务区2
		if(i == 0)
		{
			//预分配任务区
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = choice1_area;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = choice2_area;
			//计算无人机1,2的航线段
			UAVRouteGeneration();
			FlightRoute uav_route[2];
			for(int uav = 0 ; uav < 2 ; uav ++)
			{
				uav_route[uav].start.latitude = integrated_postures.integrated_posture_drone_informations[uav].drone_longitude_and_latitude.latitude;
				uav_route[uav].start.longitude = integrated_postures.integrated_posture_drone_informations[uav].drone_longitude_and_latitude.longitude;
				uav_route[uav].end.latitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[0].planning_information_waypoint_informations[0].latitude;
				uav_route[uav].end.longitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[0].planning_information_waypoint_informations[0].longitude;
			}
			//距离计算
			double uav1,uav2;
			uav1 = calculate_distances(uav_route[0].start.latitude,uav_route[0].start.longitude,uav_route[0].end.latitude,uav_route[0].end.longitude);
			uav2 = calculate_distances(uav_route[1].start.latitude,uav_route[1].start.longitude,uav_route[1].end.latitude,uav_route[1].end.longitude);
			total_dis_1 = uav1 + uav2;
			//查看是否有冲突
			int rtn[3] = {0,0,0};
			rtn[0] = detectConflict(&manned,&uav_route[0]);
			rtn[1] = detectConflict(&manned,&uav_route[1]);
			rtn[2] = detectConflict(&uav_route[0],&uav_route[1]);
			//都不存在冲突
			if(rtn[0] == 0 && rtn[1] == 0 && rtn[2] == 0)
			{
				//分法一无冲突
				distribute = 1;
			}

		}
		//分法二：无人机1在选择任务区2，无人机2在选择任务区1
		else if(i == 1)
		{
			//预分配任务区
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = choice2_area;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = choice1_area;
			//计算无人机1,2的航线段
			UAVRouteGeneration();
			FlightRoute uav_route[2];
			for(int uav = 0 ; uav < 2 ; uav ++)
			{
				uav_route[uav].start.latitude = integrated_postures.integrated_posture_drone_informations[uav].drone_longitude_and_latitude.latitude;
				uav_route[uav].start.longitude = integrated_postures.integrated_posture_drone_informations[uav].drone_longitude_and_latitude.longitude;
				uav_route[uav].end.latitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[0].planning_information_waypoint_informations[0].latitude;
				uav_route[uav].end.longitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[0].planning_information_waypoint_informations[0].longitude;
			}
			//距离计算
			double uav1,uav2;
			uav1 = calculate_distances(uav_route[0].start.latitude,uav_route[0].start.longitude,uav_route[0].end.latitude,uav_route[0].end.longitude);
			uav2 = calculate_distances(uav_route[1].start.latitude,uav_route[1].start.longitude,uav_route[1].end.latitude,uav_route[1].end.longitude);
			total_dis_2 = uav1 + uav2;
			//查看是否有冲突
			int rtn[3] = {0,0,0};
			rtn[0] = detectConflict(&manned,&uav_route[0]);
			rtn[1] = detectConflict(&manned,&uav_route[1]);
			rtn[2] = detectConflict(&uav_route[0],&uav_route[1]);
			//都不存在冲突
			if(rtn[0] == 0 && rtn[1] == 0 && rtn[2] == 0)
			{
				//分法二无冲突,分法一有冲突
				if(distribute == 0)
				{
					distribute = 2;
				}
				else if(distribute == 1)
				{
					//对比距离,分法一距离比分法二距离长，选择分法二
					if(total_dis_1 > total_dis_2)
					{
						distribute = 2;
					}
				}

			}
		}
	}

	//赋值无冲突且最短路径的分法
	//分法一
	if(distribute == 1)
	{
		information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = choice1_area;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = choice2_area;
	}
	//分法二
	else if(distribute == 2)
	{
		information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = choice2_area;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = choice1_area;
	}
	//都存在冲突,选择最短距离
	else if(distribute == -1)
	{
		if(total_dis_1 < total_dis_2)
		{
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = choice1_area;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = choice2_area;
		}
		else
		{
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = choice2_area;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = choice1_area;
		}
	}

	//分配后续阶段的任务区
	//有人机
	for(int i = 1 ;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks ; i ++)
	{
		int tmp = 0;
		tmp = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i-1].target_number;
		if(i != 3)
		{
			tmp += 1;
			if(tmp > 3)
				tmp = tmp % 3;
		}
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = tmp;
	}

	//无人机1,2
	for(int i = 1 ; i <= 2 ; i ++)
	{
		for(int j = 1 ; j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks ; j ++)
		{
			int tmp = 0;
			tmp = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j-1].target_number;
			if(j != 3)
			{
				tmp += 1;
				if(tmp > 3)
					tmp = tmp % 3;
			}

			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = tmp;
		}
	}
}
//组织任务区顺序
void mannedAreaSorted(){
	area_information area;//临时变量
	int minIndex;//最小值序号
	GeoLibDas location;
	//有人机初始位置赋值
	location.longitude = integrated_postures.longitude;
	location.latitude = integrated_postures.latitude;
	//冒泡排序组织有人机任务区顺序
	for(int i = 0;i < global_mission_planning_commandss.area_point_num - 1;i++){
		minIndex = i;
		for(int j = i + 1;j < global_mission_planning_commandss.area_point_num;j++){
			if(distanceComparison(location,area_sky_informations.area_informations[minIndex],area_sky_informations.area_informations[j])){
				minIndex = j;
			}
		}
		area = area_sky_informations.area_informations[i];
		area_sky_informations.area_informations[i] = area_sky_informations.area_informations[minIndex];
		area_sky_informations.area_informations[minIndex] = area;

		//更新有人机位置
		location = minDIstancePoint(location,area_sky_informations.area_informations[i]);
	}

	//    //有人机最后的位子应该在location
	//    integrated_postures.longitude = location.longitude;
	//    integrated_postures.latitude = location.latitude;
}

char distanceComparison(GeoLibDas location,area_information area1, area_information area2){

	if(minDistance(location,area1) > minDistance(location,area2)){
		return true;
	}
	return false;
}

char UAVdistanceComparison(integrated_posture_drone_information UAV1,integrated_posture_drone_information UAV2,area_information area){
	GeoLibDas location1;
	GeoLibDas location2;
	location1.longitude = UAV1.drone_longitude_and_latitude.longitude;
	location1.latitude = UAV1.drone_longitude_and_latitude.latitude;
	location2.longitude = UAV2.drone_longitude_and_latitude.longitude;
	location2.latitude = UAV2.drone_longitude_and_latitude.latitude;
	if(minDistance(location1,area) > minDistance(location2,area)){
		return true;
	}
	return false;
}

void UAVAreaDistribute(){
	int UAVnum[4] = {5,5,5,5};//已分配任务的无人机序号数组，若分配完任务就将编号写入该数组,该数组长度为无人机最大数量
	int temp = 0;//存储当前最小值
	char flag = false;//判断当前无人机是否已被使用
	GeoLibDas location;

	for(int i = 0;i < global_mission_planning_commandss.area_point_num;i++){
		if(area_sky_informations.area_informations[i].area_code != information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].target_number){//只对非有人机任务区域进行分配
			if(drone_state_informations.drone_number >= 2){//保证无人机数量大于等于2，否则就无需分配
				for(int j = 0; j < integrated_postures.drone_num; j++){//为无人机赋值
					for(int k = 0;k < 4;k++){
						if(UAVnum[k] == j){//如果当前UAV已经被赋值了，就直接退出
							flag = true;
							break;
						}
					}
					//qDebug()<<"flag:"<<flag;
					if(flag == true){//如果当前UAV已经被赋值了，就开始新一轮循环
						flag = false;
						continue;
					}
					temp = j;
					if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[temp],integrated_postures.integrated_posture_drone_informations[j],area_sky_informations.area_informations[i])){
						temp = j;
						//                        qDebug()<<"temp:"<<temp;
					}//如果当前无人机与该区域的值小于最小值，则把最小值置成该无人机序号
				}
				UAVnum[i] = temp;//将当前的最小值加入已分配的
				//为当前最小值的第0个任务的任务区编号进行赋值

				information_on_the_results_of_taskings.formation_synergy_mission_programs[temp + 1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[i].area_code;
				//更新无人机位置
				location = minDIstancePoint(location,area_sky_informations.area_informations[i]);
				//                integrated_postures.integrated_posture_drone_informations[temp].drone_longitude_and_latitude.longitude = location.longitude;
				//                integrated_postures.integrated_posture_drone_informations[temp].drone_longitude_and_latitude.latitude = location.latitude;
			}else{//若只有一架则直接分配即可
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number =  area_sky_informations.area_informations[i].area_code;
			}
		}
	}
}

/*
 * 3.7 有人机航路规划结果信息
 * */
void mannedAircraftRoutesGeneration(){
	area_information area;
	GeoLibDas manned_position;
	manned_task_length = 0;

	memset(&CTAS_DTMS_data_mannedRoute,0,sizeof(manned_aircraft_route_information));

	//iterate all the subtasks
	for(int i=0;i<information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++) {
		//step1. find the 1st taskArea which satisfied the tactic.
		for(int j=0;j<area_sky_informations.area_number;j++){
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number
					==area_sky_informations.area_informations[j].area_code){
				area = area_sky_informations.area_informations[j];
			}
		}
		//fist subtask use the copter longi & lati
		if(i==0){
			manned_position.longitude = integrated_postures.longitude;
			manned_position.latitude = integrated_postures.latitude;

		}else{
			//other subtasks use the last longi&lati of last subtask.
			int manned_task_id = CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i-1].waypoints_number - 1;
			manned_position.longitude = CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i-1].waypoint_informations[manned_task_id].longitude;
			manned_position.latitude = CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i-1].waypoint_informations[manned_task_id].latitude;
		}

		//subtask type:浮标布阵
		if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type==3) {

			OutBuoyLayoutAuto outBuoyLayoutAuto;
			outBuoyLayoutAuto = buyoGeneration(area,manned_position);

			int manned_subtask_id = CTAS_DTMS_data_mannedRoute.buoy_deployment_points.subtasks_number;

			CTAS_DTMS_data_mannedRoute.buoy_deployment_points.subtasks_number++;
			CTAS_DTMS_data_mannedRoute.buoy_deployment_points.program_number =
					information_on_the_results_of_taskings.program_number;

			CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[manned_subtask_id].subtask_ID_number =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number;
			CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[manned_subtask_id].points_number =
					outBuoyLayoutAuto.sum;

			for(int j=0;j<outBuoyLayoutAuto.sum;j++){
				CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[manned_subtask_id].buoy_deployment_point_informations[j].buoy_type = outBuoyLayoutAuto.formation;
				CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[manned_subtask_id].buoy_deployment_point_informations[j].latitude_validity = 1;
				CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[manned_subtask_id].buoy_deployment_point_informations[j].longitude_validity = 1;
				CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[manned_subtask_id].buoy_deployment_point_informations[j].buoy_longitude_and_latitude.longitude = outBuoyLayoutAuto.buoyA[j].geo.longitude;
				CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[manned_subtask_id].buoy_deployment_point_informations[j].buoy_longitude_and_latitude.latitude = outBuoyLayoutAuto.buoyA[j].geo.latitude;
				CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[manned_subtask_id].buoy_deployment_point_informations[j].matching_tube_number = j + 1;
				CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[manned_subtask_id].buoy_deployment_point_informations[j].buoy_type = 1;
			}
			OutBuoyLayoutAirway outBuoyLayoutAirway;
			outBuoyLayoutAirway = buyo_way_generation(CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[manned_subtask_id],manned_position);
			//            qDebug()<<"tongyonghanglu sum :"<<outBuoyLayoutAirway.sumAwp;
			//通用航路赋值
			int manned_subtask_id_common = CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number++;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.program_number =
					information_on_the_results_of_taskings.program_number;

			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].subtask_ID_number =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].task_type =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoints_number =
					outBuoyLayoutAirway.sumAwp;

			for(int j=0;j<outBuoyLayoutAirway.sumAwp;j++){
				CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoint_informations[j].longitude = outBuoyLayoutAirway.awpA[j].geo.longitude;
				CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoint_informations[j].latitude = outBuoyLayoutAirway.awpA[j].geo.latitude;
				CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoint_informations[j].height = outBuoyLayoutAirway.awpA[j].alt;
			}

			//             有人机航路
			manned_task_length += outBuoyLayoutAirway.sumAirway;
		}


		//subtask:吊声定测
		else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type==2) {

			OutSearchDippingSonar outSearchDippingSonar;
			outSearchDippingSonar = SusSoundFixedGeneration(area);

			int manned_subtask_id = CTAS_DTMS_data_mannedRoute.suspended_sound_fixed_measurement_points.subtasks_number;

			CTAS_DTMS_data_mannedRoute.suspended_sound_fixed_measurement_points.subtasks_number++;
			CTAS_DTMS_data_mannedRoute.suspended_sound_fixed_measurement_points.program_number =
					information_on_the_results_of_taskings.program_number;

			CTAS_DTMS_data_mannedRoute.suspended_sound_fixed_measurement_points.suspended_sound_manned_computer_tasks[manned_subtask_id].subtask_ID_number =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number;
			CTAS_DTMS_data_mannedRoute.suspended_sound_fixed_measurement_points.suspended_sound_manned_computer_tasks[manned_subtask_id].fixed_measurement_points_number =
					outSearchDippingSonar.sum;

			for(int j=0;j<outSearchDippingSonar.sum;j++){
				CTAS_DTMS_data_mannedRoute.suspended_sound_fixed_measurement_points.suspended_sound_manned_computer_tasks[manned_subtask_id].suspended_sound_buoy_deployment_point_informations[j].profundity = outSearchDippingSonar.sonarA[j].depth;
				CTAS_DTMS_data_mannedRoute.suspended_sound_fixed_measurement_points.suspended_sound_manned_computer_tasks[manned_subtask_id].suspended_sound_buoy_deployment_point_informations[j].longitude = outSearchDippingSonar.sonarA[j].geo.longitude;
				CTAS_DTMS_data_mannedRoute.suspended_sound_fixed_measurement_points.suspended_sound_manned_computer_tasks[manned_subtask_id].suspended_sound_buoy_deployment_point_informations[j].latitude = outSearchDippingSonar.sonarA[j].geo.latitude;
				CTAS_DTMS_data_mannedRoute.suspended_sound_fixed_measurement_points.suspended_sound_manned_computer_tasks[manned_subtask_id].suspended_sound_buoy_deployment_point_informations[j].HighlyUpper = 1;
				CTAS_DTMS_data_mannedRoute.suspended_sound_fixed_measurement_points.suspended_sound_manned_computer_tasks[manned_subtask_id].suspended_sound_buoy_deployment_point_informations[j].JingduUpper = 1;
				CTAS_DTMS_data_mannedRoute.suspended_sound_fixed_measurement_points.suspended_sound_manned_computer_tasks[manned_subtask_id].suspended_sound_buoy_deployment_point_informations[j].WeiduUpper = 1;
			}


			//通用航路赋值
			int manned_subtask_id_common = CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number++;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.program_number =
					information_on_the_results_of_taskings.program_number;

			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].subtask_ID_number =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].task_type =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoints_number =
					outSearchDippingSonar.sum;

			for(int j=0;j<outSearchDippingSonar.sum;j++){
				CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoint_informations[j].longitude = outSearchDippingSonar.sonarA[j].geo.longitude;
				CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoint_informations[j].latitude = outSearchDippingSonar.sonarA[j].geo.latitude;
			}
			//有人机航路---无长度
			//            manned_task_length += outSearchDippingSonar;
		}
		//other subtask type
		else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type==4){

		}
		else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type==11){

		}
		else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type==12){

		}
		else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type==13){

		}
		//subtask:浮标侦收
		else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type==1) {
			OutBuoyMonitorAirway outBuoyMonitorAirway;
			char has_monitor_route = 0;
			//找出浮标侦听区域对应的浮标布阵子任务ID
			unsigned int ID = 0;
			for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;j++){
				if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[j].target_number &&
						information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[j].sequence_type == 3){
					ID = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[j].subtask_ID_number;
					break;
				}
			}
			for(int j = 0;j < CTAS_DTMS_data_mannedRoute.buoy_deployment_points.subtasks_number;j++){
				if(ID == CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[j].subtask_ID_number){
					outBuoyMonitorAirway = buyo_monitor_way_generation(CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[j],manned_position);
					has_monitor_route = 1;
					break;
				}
			}

			//通用航路
			int manned_subtask_id_common = CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number++;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.program_number =
					information_on_the_results_of_taskings.program_number;

			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].subtask_ID_number =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].task_type =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type;
			if(has_monitor_route){
				CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoints_number =
						outBuoyMonitorAirway.sum;
				for(int j=0;j<outBuoyMonitorAirway.sum;j++){
					CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoint_informations[j].longitude = outBuoyMonitorAirway.awpA[j].longitude;
					CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoint_informations[j].latitude = outBuoyMonitorAirway.awpA[j].latitude;
					CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoint_informations[j].height = outBuoyMonitorAirway.alt;
				}
			}else{
				OutSearchRadarPhoto outSearchRadarPhoto;
				outSearchRadarPhoto = commonRouteGeneration(area,manned_position,6000);
				CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoints_number =
						outSearchRadarPhoto.sumAwp;
				for(int j=0;j<outSearchRadarPhoto.sumAwp;j++){
					CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoint_informations[j].longitude = outSearchRadarPhoto.awpA[j].longitude;
					CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id_common].waypoint_informations[j].latitude = outSearchRadarPhoto.awpA[j].latitude;
				}
			}

			//有人机航路---无长度
			//            manned_task_length += outBuoyMonitorAirway;

		}

		//subtask:通用航路
		else{
			OutSearchRadarPhoto outSearchRadarPhoto;
			outSearchRadarPhoto = commonRouteGeneration(area,manned_position,6000);

			int manned_subtask_id = CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number;

			CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number++;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.program_number =
					information_on_the_results_of_taskings.program_number;

			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id].subtask_ID_number =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id].task_type =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id].waypoints_number =
					outSearchRadarPhoto.sumAwp;
			for(int j=0;j<outSearchRadarPhoto.sumAwp;j++){
				CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id].waypoint_informations[j].longitude = outSearchRadarPhoto.awpA[j].longitude;
				CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[manned_subtask_id].waypoint_informations[j].latitude = outSearchRadarPhoto.awpA[j].latitude;
			}

			//有人机航路---无长度
			manned_task_length += outSearchRadarPhoto.distance;
		}


	}
}
void buoy_suspended_generation()
{
	//初始化
	area_information area;
	GeoLibDas manned_position;
	manned_task_length = 0;
	memset(&CTAS_DTMS_data_mannedRoute,0,sizeof(manned_aircraft_route_information));

	//遍历任务分配结果找出浮标和吊声任务，进行规划
	for(int i=0;i<information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++) {
		//获取任务区域信息
		for(int j=0;j<area_sky_informations.area_number;j++){
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number
					==area_sky_informations.area_informations[j].area_code){
				area = area_sky_informations.area_informations[j];
			}
		}
		//获取有人机当前位置
		if(i==0){
			manned_position.longitude = integrated_postures.longitude;
			manned_position.latitude = integrated_postures.latitude;

		}else{
			int manned_task_id = CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i-1].waypoints_number - 1;
			if(manned_task_id < 0 || manned_task_id > 79)
			{
				manned_position.longitude = integrated_postures.longitude;
				manned_position.latitude = integrated_postures.latitude;
			}
			else
			{
				manned_position.longitude = CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i-1].waypoint_informations[manned_task_id].longitude;
				manned_position.latitude = CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i-1].waypoint_informations[manned_task_id].latitude;
			}
		}

		//浮标布阵规划
		if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type==3) {

			//初始化
			memset( &buoy_arry_plan , 0 , sizeof(BLK_CCC_OFP_302) );

			OutBuoyLayoutAuto outBuoyLayoutAuto;
			outBuoyLayoutAuto = buyoGeneration(area,manned_position);
			//            //应召生成点信息
			//            if(blk_dtms_ctas_001.task_type == 1)
			//            {
			//                //初始化
			//                memset( &buoy_arry_plan , 0 , sizeof(BLK_CCC_OFP_302) );
			//                buoy_arry_plan.AutoPlanInformation.AutoPlaySet_1.AutoPlayMisType = 1;/*自动规划任务类型  0=N/A;1=目标点;2=圆形区域;3=矩形区域;*/
			//                buoy_arry_plan.AutoPlanInformation.target_point.SerNum_FC00_DPU1 = blk_dtms_ctas_001.region_infos[0].point_infos->pitch;
			//                buoy_arry_plan.AutoPlanInformation.target_point.tgtSource1.tgtSource_DPU1 = 13;//目标源  0=N/A;1=雷达;2=吊声;3=浮标;
			//                buoy_arry_plan.AutoPlanInformation.target_point.DATAValid_1.LatValid = 1;//纬度有效位
			//                buoy_arry_plan.AutoPlanInformation.target_point.DATAValid_1.LonValid = 1;//经度有效位
			//                buoy_arry_plan.AutoPlanInformation.target_point.DATAValid_1.RectionValid = 1;//航向有效位
			//                buoy_arry_plan.AutoPlanInformation.target_point.DATAValid_1.SpeedValid = 1;//速度有效位
			//                buoy_arry_plan.AutoPlanInformation.target_point.TargetLat = blk_dtms_ctas_001.region_infos[0].point_infos->point_lon_lat.latitude;//纬度
			//                buoy_arry_plan.AutoPlanInformation.target_point.TargetLon = blk_dtms_ctas_001.region_infos[0].point_infos->point_lon_lat.longitude;//经度
			//                buoy_arry_plan.AutoPlanInformation.target_point.tgtCourseT =0;//航向
			//                buoy_arry_plan.AutoPlanInformation.target_point.tgtSpeed = 0;//速度
			//            }
			//赋值
			buoy_arry_plan.Plan_ID = information_on_the_results_of_taskings.program_number;//方案编号
			buoy_arry_plan.SubTask_Id =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number;/*任务平台子任务ID号*/
			buoy_arry_plan.BuoyPlanProject.ArrayProgramCommand = 1;/*布阵规划指令  1-是/0-否*/
			buoy_arry_plan.BuoyPlanProject.SingleDouble = 1;/*单机或者协同  1-协同/0-单机*/
			buoy_arry_plan.BuoyPlanProject.PlanWay = 1;/*布阵方式  0=N/A;1=自动规划;2=图形规划;3=自定义;*/

			//发送浮标布阵规划
			send_buoy_result();
		}
		//吊声定测规划
		else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type==2) {

			//初始化
			memset( &sonar_detect_plan , 0 , sizeof(BLK_CCC_OFP_403) );

			OutSearchDippingSonar outSearchDippingSonar;
			outSearchDippingSonar = SusSoundFixedGeneration(area);

			//            //应召生成点信息
			//            if(blk_dtms_ctas_001.task_type == 1)
			//            {
			//                //初始化
			//                memset( &sonar_detect_plan , 0 , sizeof(BLK_CCC_OFP_403) );
			//                sonar_detect_plan.AutoPlanInformation.TargetPoint.SerNum_FC00_DPU1 = blk_dtms_ctas_001.region_infos[0].point_infos->pitch;
			//                sonar_detect_plan.AutoPlanInformation.TargetPoint.tgtSource_.tgtSource_DPU1 = 13;//目标源  0=N/A;1=雷达;2=吊声;3=浮标;
			//                sonar_detect_plan.AutoPlanInformation.TargetPoint.DATAValid.LatValid = 1;//纬度有效位
			//                sonar_detect_plan.AutoPlanInformation.TargetPoint.DATAValid.LonValid = 1;//经度有效位
			//                sonar_detect_plan.AutoPlanInformation.TargetPoint.DATAValid.RectionValid = 1;//航向有效位
			//                sonar_detect_plan.AutoPlanInformation.TargetPoint.DATAValid.SpeedValid = 1;//速度有效位
			//                sonar_detect_plan.AutoPlanInformation.TargetPoint.TargetLat = blk_dtms_ctas_001.region_infos[0].point_infos->point_lon_lat.latitude;//纬度
			//                sonar_detect_plan.AutoPlanInformation.TargetPoint.TargetLon = blk_dtms_ctas_001.region_infos[0].point_infos->point_lon_lat.longitude;//经度
			//                sonar_detect_plan.AutoPlanInformation.TargetPoint.tgtCourseT = blk_dtms_ctas_001.region_infos[0].point_infos->target_hangxiang;//航向
			//                sonar_detect_plan.AutoPlanInformation.TargetPoint.tgtSpeed = blk_dtms_ctas_001.region_infos[0].point_infos->target_speed;//速度
			//            }

			//赋值
			sonar_detect_plan.Plan_ID = information_on_the_results_of_taskings.program_number;//方案编号
			sonar_detect_plan.SubTask_Id =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number;/*任务平台子任务ID号*/

			sonar_detect_plan.SonarPlanSimbol_1.POINT_PLAN_SYMBOL = 0;/*协同方式  0=单机;1=双机;2=三机;*/

			sonar_detect_plan.SonarPlanSimbol_1.PlaneFlag = 1;/*长僚机标志  0=N/A;1=长机;2=僚机1;3=僚机2;*/
			sonar_detect_plan.SonarPlanSimbol_1.AvgSinglePointNumMax = 1;/*平均单架机最大定测点数    最小值：1  最大值：10*/
			sonar_detect_plan.SonarPlanSimbol_1.PlanWay = 1;/*规划方式  0=N/A;1=自动规划;2=图形规划;3=自定义;*/
			sonar_detect_plan.AutoPlanInformation.TaskType_1.TaskType1 = 1;/*任务阶段  0=N/A;1=搜索阶段;2=跟踪阶段;3=结束跟踪;*/
			sonar_detect_plan.AutoPlanInformation.TaskType_1.TaskType1_1 = 1;/*任务类型  0=N/A;1=应召反潜;2=巡逻反潜;*/

			//发送吊声定测点规划
			send_sonar_result();
		}
	}
	buoy_suspended_flag = 0;//发送结束
}
//通用航路信息生成并赋值
OutSearchRadarPhoto commonRouteGeneration(area_information area,GeoLibDas manned_position,int width){//传入区域信息和有人机位置,其中有人机位置是上一个任务的通用航路的最后一个位置

	//中间变量
	InAreaSearchRadarPhoto InAreaSearchRadarPhotos;
	//    OutSearchRadarPhoto OutSearchRadarPhotos;
	InAreaSearchRadarPhotos.widthSearch = width;
	if(area.area_shape == 1){//圆形
		GeoLibDas centre;
		RectLibDas *rectLibDas;
		double radius;
		radius = (double)(area.cycles.radius);
		centre.longitude = area.cycles.longitude;
		centre.latitude = area.cycles.latitude;
		rectLibDas = cycle_to_RectLibDas(centre,radius);
		InAreaSearchRadarPhotos.origin.longitude = manned_position.longitude;
		InAreaSearchRadarPhotos.origin.latitude = manned_position.latitude;
		InAreaSearchRadarPhotos.polygon.sumVertexs = 4;

		for(int k = 0;k < InAreaSearchRadarPhotos.polygon.sumVertexs;k ++){
			InAreaSearchRadarPhotos.polygon.vertexA[k].longitude =rectLibDas->vertexA[k].longitude;
			InAreaSearchRadarPhotos.polygon.vertexA[k].latitude = rectLibDas->vertexA[k].latitude;
		}
	}else if (area.area_shape == 2){//多边形
		InAreaSearchRadarPhotos.origin.longitude = manned_position.longitude;
		InAreaSearchRadarPhotos.origin.latitude = manned_position.latitude;
		InAreaSearchRadarPhotos.polygon.sumVertexs = area.polygonals.point_number;
		for(int k = 0;k < InAreaSearchRadarPhotos.polygon.sumVertexs;k ++){
			InAreaSearchRadarPhotos.polygon.vertexA[k].longitude =area.polygonals.point_coordinates[k].longitude;
			InAreaSearchRadarPhotos.polygon.vertexA[k].latitude = area.polygonals.point_coordinates[k].latitude;
		}

	}
	return getAreaSearchRadarPhoto(&InAreaSearchRadarPhotos);
}

//浮标布阵信息生成并赋值
OutBuoyLayoutAuto buyoGeneration(area_information area, GeoLibDas manned_position){

	InBuoyLayoutAuto inBuoyLayoutAuto;

	//中间变量---用于转换多边形
	PolygonLibDas polygonLibDas;
	RectLibDas rectLibDas;

	if(area.area_shape == 1){//当区域类型为圆形时候
		inBuoyLayoutAuto.typeTask = TYPE_BUOY_ROUND_AREA;
		inBuoyLayoutAuto.circle.radius = area.cycles.radius;
		inBuoyLayoutAuto.circle.center.longitude = area.cycles.longitude;
		inBuoyLayoutAuto.circle.center.latitude = area.cycles.latitude;
		inBuoyLayoutAuto.circle.status = 1;

		buoy_arry_plan.AutoPlanInformation.AutoPlaySet_1.AutoPlayMisType = 2;/*自动规划任务类型  0=N/A;1=目标点;2=圆形区域;3=矩形区域;*/
		buoy_arry_plan.AutoPlanInformation.CircleZone.data_status1.CircleValid = 1;/*矩形有效状态  1-有效/0-无效*/
		buoy_arry_plan.AutoPlanInformation.CircleZone.Radius = area.cycles.radius;/*半径    单位：Km  最小值：0  最大值：150*/
		buoy_arry_plan.AutoPlanInformation.CircleZone.LatData = area.cycles.latitude;
		buoy_arry_plan.AutoPlanInformation.CircleZone.LonData = area.cycles.longitude;

	}else if(area.area_shape == 2){
		inBuoyLayoutAuto.typeTask = TYPE_BUOY_RECT_AREA;
		inBuoyLayoutAuto.rect.status = 1;
		//将多边形转换为矩形
		polygonLibDas.sumVertexs = area.polygonals.point_number;
		for(int k = 0;k < polygonLibDas.sumVertexs;k++){
			polygonLibDas.vertexA[k].longitude = area.polygonals.point_coordinates[k].longitude;
			polygonLibDas.vertexA[k].latitude = area.polygonals.point_coordinates[k].latitude;
		}
		rectLibDas = getMabrLibDas(&polygonLibDas);//接收转换的矩形
		for(int k = 0;k < 4;k++){
//			printf("rectLibDas.vertexA[%d].longitude : %lf\n",k,rectLibDas.vertexA[k].longitude);
//			printf("rectLibDas.vertexA[%d].latitude : %lf\n",k,rectLibDas.vertexA[k].latitude);
		}
		//矩形区域信息赋值
		int diagonal = 0;//找出对角点位置
		int zongXiang = -1;
		int hengXiang = -1;
		for(int k = 1;k < 4;k++){
			if(getDistanceGeoLibDas(&(rectLibDas.vertexA[0]),&(rectLibDas.vertexA[diagonal])) < getDistanceGeoLibDas(&(rectLibDas.vertexA[0]),&(rectLibDas.vertexA[k]))){
				diagonal = k;
			}
		}

		for(int k = 1;k < 4;k++){
			if(k != diagonal){
				if(zongXiang == -1){
					zongXiang = k;
				}else if(hengXiang == -1){
					hengXiang = k;
				}
			}
		}

		inBuoyLayoutAuto.rect.status = 1;//数据状态
		inBuoyLayoutAuto.rect.center.longitude = (rectLibDas.vertexA[0].longitude + rectLibDas.vertexA[diagonal].longitude) / 2;
		inBuoyLayoutAuto.rect.center.latitude = (rectLibDas.vertexA[0].latitude + rectLibDas.vertexA[diagonal].latitude) / 2;
		// -pi-pi转到0-360
		inBuoyLayoutAuto.rect.dir = getAzimuthGeoLibDas(&(rectLibDas.vertexA[0]),&(rectLibDas.vertexA[zongXiang])) * 180.0 / PI;
		if(inBuoyLayoutAuto.rect.dir < 0)
		{
			inBuoyLayoutAuto.rect.dir += 360.0;
		}
		inBuoyLayoutAuto.rect.latLength = getDistanceGeoLibDas(&(rectLibDas.vertexA[0]),&(rectLibDas.vertexA[hengXiang]));
		inBuoyLayoutAuto.rect.lngLength = getDistanceGeoLibDas(&(rectLibDas.vertexA[0]),&(rectLibDas.vertexA[zongXiang]));

		buoy_arry_plan.AutoPlanInformation.AutoPlaySet_1.AutoPlayMisType = 3;/*自动规划任务类型  0=N/A;1=目标点;2=圆形区域;3=矩形区域;*/
		buoy_arry_plan.AutoPlanInformation.RectangleZone.data_status.RectangleValid = 1;/*矩形有效状态  1-有效/0-无效*/
		buoy_arry_plan.AutoPlanInformation.RectangleZone.LatData = inBuoyLayoutAuto.rect.center.latitude;
		buoy_arry_plan.AutoPlanInformation.RectangleZone.LonData = inBuoyLayoutAuto.rect.center.longitude;
		buoy_arry_plan.AutoPlanInformation.RectangleZone.AngleData = inBuoyLayoutAuto.rect.dir;
		buoy_arry_plan.AutoPlanInformation.RectangleZone.BroadWiseData = inBuoyLayoutAuto.rect.latLength / 1000.0;/*矩形区域横向边长*/
		buoy_arry_plan.AutoPlanInformation.RectangleZone.DirectWiseData = inBuoyLayoutAuto.rect.lngLength / 1000.0;/*矩形区域纵向边长*/

	}else if(area.area_type == 0){
		inBuoyLayoutAuto.typeTask = TYPE_BUOY_TARGET_POINT;

		buoy_arry_plan.AutoPlanInformation.AutoPlaySet_1.AutoPlayMisType = 1;/*自动规划任务类型  0=N/A;1=目标点;2=圆形区域;3=矩形区域;*/
		buoy_arry_plan.AutoPlanInformation.target_point.SerNum_FC00_DPU1 = blk_dtms_ctas_001.region_infos[0].point_infos->pitch;
		buoy_arry_plan.AutoPlanInformation.target_point.tgtSource1.tgtSource_DPU1 = 3;//目标源  0=N/A;1=雷达;2=吊声;3=浮标;
		buoy_arry_plan.AutoPlanInformation.target_point.DATAValid_1.LatValid = 1;//纬度有效位
		buoy_arry_plan.AutoPlanInformation.target_point.DATAValid_1.LonValid = 1;//经度有效位
		buoy_arry_plan.AutoPlanInformation.target_point.DATAValid_1.RectionValid = 1;//航向有效位
		buoy_arry_plan.AutoPlanInformation.target_point.DATAValid_1.SpeedValid = 1;//速度有效位
		buoy_arry_plan.AutoPlanInformation.target_point.TargetLat = blk_dtms_ctas_001.region_infos[0].point_infos->point_lon_lat.latitude;//纬度
		buoy_arry_plan.AutoPlanInformation.target_point.TargetLon = blk_dtms_ctas_001.region_infos[0].point_infos->point_lon_lat.longitude;//经度
		buoy_arry_plan.AutoPlanInformation.target_point.tgtCourseT = blk_dtms_ctas_001.region_infos[0].point_infos->target_hangxiang;//航向
		buoy_arry_plan.AutoPlanInformation.target_point.tgtSpeed = blk_dtms_ctas_001.region_infos[0].point_infos->target_speed;//速度

	}
	//浮标参数
	inBuoyLayoutAuto.BuoyPara.VLineDepth = 40;
	inBuoyLayoutAuto.BuoyPara.VLineRange = 5 *1000;
	inBuoyLayoutAuto.BuoyPara.ActiveAllDepth = 40;
	inBuoyLayoutAuto.BuoyPara.ActiveAllRange = 5 *1000;
	inBuoyLayoutAuto.BuoyPara.ActiveOptDepth = 40;
	inBuoyLayoutAuto.BuoyPara.ActiveOptRange = 5 *1000;
	inBuoyLayoutAuto.BuoyPara.ActiveTwoDepth = 40;
	inBuoyLayoutAuto.BuoyPara.ActiveTwoRange = 5 *1000;
	inBuoyLayoutAuto.BuoyPara.PassiveAllDepth = 40;
	inBuoyLayoutAuto.BuoyPara.PassiveAllRange = 5 *1000;
	inBuoyLayoutAuto.BuoyPara.PassiveOptDepth = 40;
	inBuoyLayoutAuto.BuoyPara.PassiveOptRange = 5 *1000;
	for(int k = 0;k < 25;k++){
		inBuoyLayoutAuto.loadListA[k].num = k+1;
		inBuoyLayoutAuto.loadListA[k].type = 1;
		inBuoyLayoutAuto.loadListA[k].depth = 40;
		inBuoyLayoutAuto.loadListA[k].time = 4;
		inBuoyLayoutAuto.loadListA[k].errMsg = 0;
	}

	inBuoyLayoutAuto.aircraft.geo.longitude = manned_position.longitude;
	inBuoyLayoutAuto.aircraft.geo.latitude = manned_position.latitude;
	inBuoyLayoutAuto.aircraft.velocity = blk_dtms_ctas_001.solider_infos[0].speed;
	inBuoyLayoutAuto.aircraft.height = blk_dtms_ctas_001.solider_infos[0].height;
	inBuoyLayoutAuto.aircraft.trackAngle = blk_dtms_ctas_001.solider_infos[0].hangxiang;

	OutBuoyLayoutAuto outBla = getBuoyLayoutAuto(&inBuoyLayoutAuto);
	//    qDebug()<<"规划的点数量sum:"<<outBla.sum;
	return outBla;
	//    //方案编号---3.6 方案编号
	//    CTAS_DTMS_data_mannedRoute.common_carrier_routes.program_number = information_on_the_results_of_taskings.program_number;
	//    //有人机子任务个数
	//    CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number
	//            = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;
	//    //有人机子任务
	//    for(int i = 0;i < CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number;i++){
	//        //子任务ID
	//        CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i].subtask_ID_number
	//                = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number;
	//        //为搜索区域中间变量赋值
	//        for(int j = 0;j < area_sky_informations.area_number;j++){//根据任务编号，找到当前任务对应的任务区
	//            if(area_sky_informations.area_informations[j].area_code == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number){
	//                inbuoyLayoutAirway.sumBuoy = 25;
	//                inbuoyLayoutAirway.planFavor = 1;
	//                if(area_sky_informations.area_informations[j].area_shape == 1){//圆形

	//                }else if (area_sky_informations.area_informations[j].area_shape == 2){//多边形

	//                }

	//             }
	//        }
	//    }
}

//浮标航路生成
OutBuoyLayoutAirway buyo_way_generation(buoy_manned_computer_task buyo_info,GeoLibDas manned_position){
	InBuoyLayoutAirway inBuoyLayoutAirway;

	inBuoyLayoutAirway.planFavor = 1;
	inBuoyLayoutAirway.sumBuoy = buyo_info.points_number;
	//qDebug()<<"num:"<<buyo_info.points_number;
	for(int i = 0;i < 25;i++){
		inBuoyLayoutAirway.buoyA[i].geo.longitude = buyo_info.buoy_deployment_point_informations[i].buoy_longitude_and_latitude.longitude;
		inBuoyLayoutAirway.buoyA[i].geo.latitude = buyo_info.buoy_deployment_point_informations[i].buoy_longitude_and_latitude.latitude;
		inBuoyLayoutAirway.buoyA[i].tube.tube = buyo_info.buoy_deployment_point_informations[i].matching_tube_number;
		inBuoyLayoutAirway.buoyA[i].tube.type = buyo_info.buoy_deployment_point_informations[i].buoy_type;
	}

	inBuoyLayoutAirway.aircraft.geo.longitude = manned_position.longitude;
	inBuoyLayoutAirway.aircraft.geo.latitude = manned_position.latitude;
	inBuoyLayoutAirway.aircraft.velocity = blk_dtms_ctas_001.solider_infos[0].speed;
	inBuoyLayoutAirway.aircraft.height = blk_dtms_ctas_001.solider_infos[0].height;
	inBuoyLayoutAirway.aircraft.trackAngle = blk_dtms_ctas_001.solider_infos[0].hangxiang;

	return getBuoyLayoutAirway(&inBuoyLayoutAirway);
}

//浮标侦听航路生成
OutBuoyMonitorAirway buyo_monitor_way_generation(buoy_manned_computer_task buyo_info,GeoLibDas manned_position){
	InBuoyMonitorAirway inBuoyMonitorAirway;


	inBuoyMonitorAirway.geoAircraft.longitude = manned_position.longitude;
	inBuoyMonitorAirway.geoAircraft.latitude = manned_position.latitude;
	inBuoyMonitorAirway.clkWise = 1;
	inBuoyMonitorAirway.sum = buyo_info.points_number;

	for(int i = 0;i < 25;i++){
		inBuoyMonitorAirway.geoBuoyA[i].longitude = buyo_info.buoy_deployment_point_informations[i].buoy_longitude_and_latitude.longitude;
		inBuoyMonitorAirway.geoBuoyA[i].latitude = buyo_info.buoy_deployment_point_informations[i].buoy_longitude_and_latitude.latitude;
	}

	return getBuoyMonitorAirway(&inBuoyMonitorAirway);
}

//吊声定测点信息生成并赋值
OutSearchDippingSonar SusSoundFixedGeneration(area_information area){
	//圆形区域搜索航路规划输入,中间变量
	InAreaSearchDippingSonar InAreaSearchDippingSonars;
	memset(&InAreaSearchDippingSonars,0,sizeof(InAreaSearchDippingSonar));
	//吊声搜索航路规划输出，中间变量，用于接收生成结果
	//    OutSearchDippingSonar OutSearchDippingSonars;
	//中间变量---用于转换多边形
	PolygonLibDas polygonLibDas;
	RectLibDas rectLibDas;


	if(area.area_shape == 1){//圆形
		InAreaSearchDippingSonars.type = TYPE_CIRCLE_AREA_DS;
		//为区域变量赋值
		InAreaSearchDippingSonars.circle.centre.longitude = area.cycles.longitude;
		InAreaSearchDippingSonars.circle.centre.latitude = area.cycles.latitude;
		InAreaSearchDippingSonars.circle.radius = (double)(area.cycles.radius);
		InAreaSearchDippingSonars.rangeSonar = 10;//吊声最佳作用距离
		InAreaSearchDippingSonars.depthSonar = 40;//吊声最佳工作深度

		sonar_detect_plan.AutoPlanInformation.task_area_info.AreaType_1.AreaType = 2;/*区域类型*/
		sonar_detect_plan.AutoPlanInformation.task_area_info.CircleZone.Radius = (double)(area.cycles.radius);
		sonar_detect_plan.AutoPlanInformation.task_area_info.CircleZone.LatData = area.cycles.latitude;
		sonar_detect_plan.AutoPlanInformation.task_area_info.CircleZone.LonData = area.cycles.longitude;

	}else if(area.area_shape == 2){//矩形
		InAreaSearchDippingSonars.type = TYPE_RECT_AREA_DS;
		//将多边形转换为矩形
		//        polygonLibDas.sumVertexs = area.polygonals.point_number;
		//        for(int k = 0;k < polygonLibDas.sumVertexs;k++){
		//            polygonLibDas.vertexA[k].longitude = area.polygonals.point_coordinates[k].longitude;
		//            polygonLibDas.vertexA[k].latitude = area.polygonals.point_coordinates[k].latitude;
		//        }
		//        rectLibDas = getMabrLibDas(&polygonLibDas);//接收转换的矩形
		//        //矩形区域信息赋值
		//        int diagonal = 0;//找出对角点位置
		//        int zongXiang = -1;
		//        int hengXiang = -1;
		//        for(int k = 1;k < 4;k++){
		//            if(getDistanceGeoLibDas(&(rectLibDas.vertexA[0]),&(rectLibDas.vertexA[diagonal])) < getDistanceGeoLibDas(&(rectLibDas.vertexA[0]),&(rectLibDas.vertexA[k]))){
		//                diagonal = k;
		//            }
		//        }
		//        for(int k = 1;k < 4;k++){
		//            if(k != diagonal){
		//                if(zongXiang == -1){
		//                    zongXiang = k;
		//                }else if(hengXiang == -1){
		//                    hengXiang = k;
		//                }
		//            }
		//        }
		AreaRectVertex rectVer;
		for(int i = 0; i < area.polygonals.point_number; ++i) {
			rectVer.vertexA[i].latitude = area.polygonals.point_coordinates[i].latitude;
			rectVer.vertexA[i].longitude = area.polygonals.point_coordinates[i].longitude;
		}

		AreaRectCenter rectCen = getAreaRectCenterByVertex(&rectVer);

		InAreaSearchDippingSonars.rect.centre.longitude = rectCen.center.longitude;
		InAreaSearchDippingSonars.rect.centre.latitude = rectCen.center.latitude;
		// 角度需要从【-pi，pi】转到【0,360】
		InAreaSearchDippingSonars.rect.azimuthArea = rectCen.dir * 180  / PI;
		while(InAreaSearchDippingSonars.rect.azimuthArea < 0)
		{
			InAreaSearchDippingSonars.rect.azimuthArea += 360.0;
		}
		while(InAreaSearchDippingSonars.rect.azimuthArea > 360.0)
		{
			InAreaSearchDippingSonars.rect.azimuthArea -= 360.0;
		}
		InAreaSearchDippingSonars.rect.widthArea = rectCen.lenLat;
		InAreaSearchDippingSonars.rect.heightArea = rectCen.lenLng;
		//InAreaSearchDippingSonars->rect.
		InAreaSearchDippingSonars.rangeSonar = 10000;//吊声最佳作用距离
		InAreaSearchDippingSonars.depthSonar = 40;//吊声最佳工作深度

		sonar_detect_plan.AutoPlanInformation.task_area_info.AreaType_1.AreaType = 1;/*区域类型*/
		sonar_detect_plan.AutoPlanInformation.task_area_info.RectangleZone.LatData = InAreaSearchDippingSonars.rect.centre.latitude;
		sonar_detect_plan.AutoPlanInformation.task_area_info.RectangleZone.LonData = InAreaSearchDippingSonars.rect.centre.longitude;
		sonar_detect_plan.AutoPlanInformation.task_area_info.RectangleZone.AngleData = InAreaSearchDippingSonars.rect.azimuthArea;
		sonar_detect_plan.AutoPlanInformation.task_area_info.RectangleZone.BroadWiseData = InAreaSearchDippingSonars.rect.widthArea / 1000.0;
		sonar_detect_plan.AutoPlanInformation.task_area_info.RectangleZone.DirectWiseData = InAreaSearchDippingSonars.rect.heightArea / 1000.0;
	}else if(area.area_type == 0){

		sonar_detect_plan.AutoPlanInformation.TaskType_1.TaskType1_1 = 1;/*任务类型  0=N/A;1=应召反潜;2=巡逻反潜;*/
		sonar_detect_plan.AutoPlanInformation.TargetPoint.SerNum_FC00_DPU1 = blk_dtms_ctas_001.region_infos[0].point_infos->pitch;
		sonar_detect_plan.AutoPlanInformation.TargetPoint.tgtSource_.tgtSource_DPU1 = 3;//目标源  0=N/A;1=雷达;2=吊声;3=浮标;
		sonar_detect_plan.AutoPlanInformation.TargetPoint.DATAValid.LatValid = 1;//纬度有效位
		sonar_detect_plan.AutoPlanInformation.TargetPoint.DATAValid.LonValid = 1;//经度有效位
		sonar_detect_plan.AutoPlanInformation.TargetPoint.DATAValid.RectionValid = 1;//航向有效位
		sonar_detect_plan.AutoPlanInformation.TargetPoint.DATAValid.SpeedValid = 1;//速度有效位
		sonar_detect_plan.AutoPlanInformation.TargetPoint.TargetLat = blk_dtms_ctas_001.region_infos[0].point_infos->point_lon_lat.latitude;//纬度
		sonar_detect_plan.AutoPlanInformation.TargetPoint.TargetLon = blk_dtms_ctas_001.region_infos[0].point_infos->point_lon_lat.longitude;//经度
		sonar_detect_plan.AutoPlanInformation.TargetPoint.tgtCourseT = blk_dtms_ctas_001.region_infos[0].point_infos->target_hangxiang;//航向
		sonar_detect_plan.AutoPlanInformation.TargetPoint.tgtSpeed = blk_dtms_ctas_001.region_infos[0].point_infos->target_speed;//速度

	}

	return getAreaSearchDippingSonar(&InAreaSearchDippingSonars);

}

//磁探航线生成
OutAirwayLibDas citanGeneration(area_information area,GeoLibDas position){
	//磁探生成
	InAreaLoopAirway inAreaLoopAirway;
	//    OutAirwayLibDas outAirwayLibDas;

	RectLibDas *rect;//中间变量，矩形
	//有人机航路赋值
	inAreaLoopAirway.aircraft.longitude = position.longitude;
	inAreaLoopAirway.aircraft.latitude = position.latitude;
	if(area.area_shape == 1){
		GeoLibDas centre;
		double radius;
		centre.longitude = area.cycles.longitude;
		centre.latitude = area.cycles.latitude;
		radius = area.cycles.radius;
		rect = cycle_to_RectLibDas(centre,radius);
		//光栅覆盖区域赋值--矩形
		for(int i = 0;i < 4;i++){
			inAreaLoopAirway.area.vertexA[i].longitude =
					rect->vertexA[i].longitude;
			inAreaLoopAirway.area.vertexA[i].latitude =
					rect->vertexA[i].latitude;
		}
	}else if(area.area_shape == 2){
		//光栅覆盖区域赋值--矩形
		for(int i = 0;i < 4;i++){
			inAreaLoopAirway.area.vertexA[i].longitude =
					area.polygonals.point_coordinates[i].longitude;
			inAreaLoopAirway.area.vertexA[i].latitude =
					area.polygonals.point_coordinates[i].latitude;
		}
	}

	inAreaLoopAirway.distBuff = 3;
	inAreaLoopAirway.widthScan = 3000;
	inAreaLoopAirway.distOffset = 0;
	//        qDebug()<<"有人机经度："<<inAreaLoopAirway.aircraft.longitude;
	//        qDebug()<<"有人机纬度："<<inAreaLoopAirway.aircraft.latitude;
	//        for(int i = 0;i < 4;i++){
	//            qDebug()<<"经度:"<<inAreaLoopAirway.area.vertexA[i].longitude;
	//            qDebug()<<"纬度:"<<inAreaLoopAirway.area.vertexA[i].latitude;
	//        }

	//磁探搜索生成
	return getAreaLoopAirway(&inAreaLoopAirway);
}

/*
 * 3.8 无人机航路规划方案生成
 * */
static void normalizeRectVertices(GeoLibDas vertexA[4]);

static int isSameGeoPoint(const GeoLibDas *pointA, const GeoLibDas *pointB)
{
	return (fabs(pointA->longitude - pointB->longitude) < 1e-6 &&
			fabs(pointA->latitude  - pointB->latitude)  < 1e-6);
}

static int findSharedVertexPairs(const GeoLibDas rectVertexA[4], const GeoLibDas rectVertexB[4], int sharedA[2], int sharedB[2])
{
	typedef struct {
		int idxA;
		int idxB;
		double distM;
	} VertexPairDist;

	VertexPairDist pairDistA[16];
	int index = 0;
	for(int i = 0; i < 4; i++) {
		for(int j = 0; j < 4; j++) {
			pairDistA[index].idxA = i;
			pairDistA[index].idxB = j;
			pairDistA[index].distM = getDistanceGeoLibDas(&rectVertexA[i], &rectVertexB[j]);
			index++;
		}
	}

	for(int i = 0; i < 15; i++) {
		for(int j = i + 1; j < 16; j++) {
			if(pairDistA[j].distM < pairDistA[i].distM) {
				VertexPairDist temp = pairDistA[i];
				pairDistA[i] = pairDistA[j];
				pairDistA[j] = temp;
			}
		}
	}

	int usedA[4] = {0, 0, 0, 0};
	int usedB[4] = {0, 0, 0, 0};
	int sharedCount = 0;

	for(int i = 0; i < 16; i++) {
		int idxA = pairDistA[i].idxA;
		int idxB = pairDistA[i].idxB;
		if(usedA[idxA] || usedB[idxB]) continue;
		sharedA[sharedCount] = idxA;
		sharedB[sharedCount] = idxB;
		usedA[idxA] = 1;
		usedB[idxB] = 1;
		sharedCount++;
		if(sharedCount == 2) break;
	}

	return sharedCount;
}

static GeoLibDas getNearestPolygonVertexForUav(GeoLibDas uavPosition, area_information area)
{
	GeoLibDas nearestVertex = uavPosition;
	if(area.area_shape != 2 || area.polygonals.point_number == 0) return nearestVertex;

	double minVertexDist = DBL_MAX;
	for(int i = 0; i < area.polygonals.point_number; i++) {
		GeoLibDas vertex;
		vertex.longitude = area.polygonals.point_coordinates[i].longitude;
		vertex.latitude = area.polygonals.point_coordinates[i].latitude;
		double dist = getDistanceGeoLibDas(&uavPosition, &vertex);
		if(dist < minVertexDist) {
			minVertexDist = dist;
			nearestVertex = vertex;
		}
	}
	return nearestVertex;
}

/*
 * 计算点到线段的最近点
 */
static GeoLibDas getNearestPointOnSegment(GeoLibDas p, GeoLibDas a, GeoLibDas b)
{
    double ap_lon = p.longitude - a.longitude;
    double ap_lat = p.latitude - a.latitude;
    double ab_lon = b.longitude - a.longitude;
    double ab_lat = b.latitude - a.latitude;

    double ab2 = ab_lon * ab_lon + ab_lat * ab_lat;
    if (ab2 < 1e-12) return a;

    double t = (ap_lon * ab_lon + ap_lat * ab_lat) / ab2;
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;

    GeoLibDas result;
    result.longitude = a.longitude + t * ab_lon;
    result.latitude = a.latitude + t * ab_lat;
    return result;
}

/*
 * 计算点到多边形边界的最近点
 */
static GeoLibDas getNearestPointOnPolygonBoundary(GeoLibDas target, area_information area)
{
    GeoLibDas bestPoint = target;
    double minDist = DBL_MAX;

    if (area.area_shape != 2 || area.polygonals.point_number < 3)
    {
        return getNearestPolygonVertexForUav(target, area);
    }

    for (int i = 0; i < area.polygonals.point_number; i++)
    {
        GeoLibDas p1 = {
            area.polygonals.point_coordinates[i].longitude,
            area.polygonals.point_coordinates[i].latitude
        };
        GeoLibDas p2 = {
            area.polygonals.point_coordinates[(i + 1) % area.polygonals.point_number].longitude,
            area.polygonals.point_coordinates[(i + 1) % area.polygonals.point_number].latitude
        };

        GeoLibDas closestOnSeg = getNearestPointOnSegment(target, p1, p2);
        double dist = getDistanceGeoLibDas(&target, &closestOnSeg);

        if (dist < minDist)
        {
            minDist = dist;
            bestPoint = closestOnSeg;
        }
    }

    return bestPoint;
}

static void optimizeDualUavInPoints(GeoLibDas uavPositionA[2], area_information areaA[2], GeoLibDas inPointA[2])
{
	if(areaA[0].area_shape != 2 || areaA[1].area_shape != 2) return;
	if(areaA[0].polygonals.point_number == 0 || areaA[1].polygonals.point_number == 0) return;

	double bestNoConflictDist = DBL_MAX;
	double bestAnyDist = DBL_MAX;
	GeoLibDas bestNoConflictA = inPointA[0];
	GeoLibDas bestNoConflictB = inPointA[1];
	GeoLibDas bestAnyA = inPointA[0];
	GeoLibDas bestAnyB = inPointA[1];
	int foundNoConflict = 0;
	int foundAnyNonSame = 0;

	// For split rectangles, evaluate same-boundary-side candidates first.
	if(areaA[0].polygonals.point_number == 4 && areaA[1].polygonals.point_number == 4) {
		GeoLibDas rectVertexA[4];
		GeoLibDas rectVertexB[4];
		for(int i = 0; i < 4; i++) {
			rectVertexA[i].longitude = areaA[0].polygonals.point_coordinates[i].longitude;
			rectVertexA[i].latitude = areaA[0].polygonals.point_coordinates[i].latitude;
			rectVertexB[i].longitude = areaA[1].polygonals.point_coordinates[i].longitude;
			rectVertexB[i].latitude = areaA[1].polygonals.point_coordinates[i].latitude;
		}
		normalizeRectVertices(rectVertexA);
		normalizeRectVertices(rectVertexB);

		int sharedA[2] = {-1, -1};
		int sharedB[2] = {-1, -1};
		int sharedCount = findSharedVertexPairs(rectVertexA, rectVertexB, sharedA, sharedB);
		if(sharedCount == 2) {
			double sharedDist0 = getDistanceGeoLibDas(&rectVertexA[sharedA[0]], &rectVertexB[sharedB[0]]);
			double sharedDist1 = getDistanceGeoLibDas(&rectVertexA[sharedA[1]], &rectVertexB[sharedB[1]]);
			double minEdgeA = DBL_MAX;
			double minEdgeB = DBL_MAX;
			for(int i = 0; i < 4; i++) {
				int j = (i + 1) % 4;
				double edgeA = getDistanceGeoLibDas(&rectVertexA[i], &rectVertexA[j]);
				double edgeB = getDistanceGeoLibDas(&rectVertexB[i], &rectVertexB[j]);
				if(edgeA < minEdgeA) minEdgeA = edgeA;
				if(edgeB < minEdgeB) minEdgeB = edgeB;
			}
			double edgeRef = (minEdgeA < minEdgeB) ? minEdgeA : minEdgeB;
			double sharedThreshold = edgeRef * 0.35;
			if(sharedThreshold < 20.0) sharedThreshold = 20.0;

			int diffA = (sharedA[0] > sharedA[1]) ? (sharedA[0] - sharedA[1]) : (sharedA[1] - sharedA[0]);
			int diffB = (sharedB[0] > sharedB[1]) ? (sharedB[0] - sharedB[1]) : (sharedB[1] - sharedB[0]);
			int adjacentA = (diffA == 1 || diffA == 3);
			int adjacentB = (diffB == 1 || diffB == 3);
			if(!(adjacentA && adjacentB) || sharedDist0 > sharedThreshold || sharedDist1 > sharedThreshold) {
				sharedCount = 0;
			}
		}

		if(sharedCount == 2) {
			int outerA[2];
			int outerB[2];
			int outerCountA = 0;
			int outerCountB = 0;
			for(int i = 0; i < 4; i++) {
				if(i != sharedA[0] && i != sharedA[1]) {
					outerA[outerCountA++] = i;
				}
			}
			for(int i = 0; i < 4; i++) {
				if(i != sharedB[0] && i != sharedB[1]) {
					outerB[outerCountB++] = i;
				}
			}

			if(outerCountA == 2 && outerCountB == 2) {
				GeoLibDas sharedOrigin = rectVertexA[sharedA[0]];
				double dirLon = rectVertexA[sharedA[1]].longitude - sharedOrigin.longitude;
				double dirLat = rectVertexA[sharedA[1]].latitude - sharedOrigin.latitude;
				double dirNorm = sqrt(dirLon * dirLon + dirLat * dirLat);
				if(dirNorm > 1e-12) {
					dirLon /= dirNorm;
					dirLat /= dirNorm;

					double outerAProj0 = (rectVertexA[outerA[0]].longitude - sharedOrigin.longitude) * dirLon +
										 (rectVertexA[outerA[0]].latitude - sharedOrigin.latitude) * dirLat;
					double outerAProj1 = (rectVertexA[outerA[1]].longitude - sharedOrigin.longitude) * dirLon +
										 (rectVertexA[outerA[1]].latitude - sharedOrigin.latitude) * dirLat;
					if(outerAProj1 < outerAProj0) {
						int temp = outerA[0];
						outerA[0] = outerA[1];
						outerA[1] = temp;
					}

					double outerBProj0 = (rectVertexB[outerB[0]].longitude - sharedOrigin.longitude) * dirLon +
										 (rectVertexB[outerB[0]].latitude - sharedOrigin.latitude) * dirLat;
					double outerBProj1 = (rectVertexB[outerB[1]].longitude - sharedOrigin.longitude) * dirLon +
										 (rectVertexB[outerB[1]].latitude - sharedOrigin.latitude) * dirLat;
					if(outerBProj1 < outerBProj0) {
						int temp = outerB[0];
						outerB[0] = outerB[1];
						outerB[1] = temp;
					}

					for(int side = 0; side < 2; side++) {
						GeoLibDas sideVertexA[2] = {rectVertexA[sharedA[side]], rectVertexA[outerA[side]]};
						GeoLibDas sideVertexB[2] = {rectVertexB[sharedB[side]], rectVertexB[outerB[side]]};
						for(int a = 0; a < 2; a++) {
							for(int b = 0; b < 2; b++) {
								GeoLibDas vertexA = sideVertexA[a];
								GeoLibDas vertexB = sideVertexB[b];
								if(isSameGeoPoint(&vertexA, &vertexB)) continue;

								double totalDist = getDistanceGeoLibDas(&uavPositionA[0], &vertexA) +
												   getDistanceGeoLibDas(&uavPositionA[1], &vertexB);
								if(totalDist < bestAnyDist) {
									bestAnyDist = totalDist;
									bestAnyA = vertexA;
									bestAnyB = vertexB;
									foundAnyNonSame = 1;
								}

								FlightRoute routeA, routeB;
								routeA.start.longitude = uavPositionA[0].longitude;
								routeA.start.latitude = uavPositionA[0].latitude;
								routeA.end.longitude = vertexA.longitude;
								routeA.end.latitude = vertexA.latitude;
								routeB.start.longitude = uavPositionA[1].longitude;
								routeB.start.latitude = uavPositionA[1].latitude;
								routeB.end.longitude = vertexB.longitude;
								routeB.end.latitude = vertexB.latitude;

								if(detectConflict(&routeA, &routeB) == 0 && totalDist < bestNoConflictDist) {
									bestNoConflictDist = totalDist;
									bestNoConflictA = vertexA;
									bestNoConflictB = vertexB;
									foundNoConflict = 1;
								}
							}
						}
					}
				}
			}
		}

		// Shared-edge identification failed: keep same-side constraint by index-matched edges.
		if(!foundAnyNonSame) {
			for(int side = 0; side < 4; side++) {
				GeoLibDas sideVertexA[2] = {rectVertexA[side], rectVertexA[(side + 1) % 4]};
				GeoLibDas sideVertexB[2] = {rectVertexB[side], rectVertexB[(side + 1) % 4]};
				for(int a = 0; a < 2; a++) {
					for(int b = 0; b < 2; b++) {
						GeoLibDas vertexA = sideVertexA[a];
						GeoLibDas vertexB = sideVertexB[b];
						if(isSameGeoPoint(&vertexA, &vertexB)) continue;

						double totalDist = getDistanceGeoLibDas(&uavPositionA[0], &vertexA) +
										   getDistanceGeoLibDas(&uavPositionA[1], &vertexB);
						if(totalDist < bestAnyDist) {
							bestAnyDist = totalDist;
							bestAnyA = vertexA;
							bestAnyB = vertexB;
							foundAnyNonSame = 1;
						}

						FlightRoute routeA, routeB;
						routeA.start.longitude = uavPositionA[0].longitude;
						routeA.start.latitude = uavPositionA[0].latitude;
						routeA.end.longitude = vertexA.longitude;
						routeA.end.latitude = vertexA.latitude;
						routeB.start.longitude = uavPositionA[1].longitude;
						routeB.start.latitude = uavPositionA[1].latitude;
						routeB.end.longitude = vertexB.longitude;
						routeB.end.latitude = vertexB.latitude;

						if(detectConflict(&routeA, &routeB) == 0 && totalDist < bestNoConflictDist) {
							bestNoConflictDist = totalDist;
							bestNoConflictA = vertexA;
							bestNoConflictB = vertexB;
							foundNoConflict = 1;
						}
					}
				}
			}
		}

		if(foundNoConflict) {
			inPointA[0] = bestNoConflictA;
			inPointA[1] = bestNoConflictB;
			return;
		}
		if(foundAnyNonSame) {
			inPointA[0] = bestAnyA;
			inPointA[1] = bestAnyB;
			return;
		}
	}

	for(int i = 0; i < areaA[0].polygonals.point_number; i++) {
		GeoLibDas vertexA;
		vertexA.longitude = areaA[0].polygonals.point_coordinates[i].longitude;
		vertexA.latitude = areaA[0].polygonals.point_coordinates[i].latitude;

		for(int j = 0; j < areaA[1].polygonals.point_number; j++) {
			GeoLibDas vertexB;
			vertexB.longitude = areaA[1].polygonals.point_coordinates[j].longitude;
			vertexB.latitude = areaA[1].polygonals.point_coordinates[j].latitude;

			int isSamePoint = isSameGeoPoint(&vertexA, &vertexB);
			double totalDist = getDistanceGeoLibDas(&uavPositionA[0], &vertexA) +
							   getDistanceGeoLibDas(&uavPositionA[1], &vertexB);
			if(!isSamePoint && totalDist < bestAnyDist) {
				bestAnyDist = totalDist;
				bestAnyA = vertexA;
				bestAnyB = vertexB;
				foundAnyNonSame = 1;
			}

			FlightRoute routeA, routeB;
			routeA.start.longitude = uavPositionA[0].longitude;
			routeA.start.latitude = uavPositionA[0].latitude;
			routeA.end.longitude = vertexA.longitude;
			routeA.end.latitude = vertexA.latitude;
			routeB.start.longitude = uavPositionA[1].longitude;
			routeB.start.latitude = uavPositionA[1].latitude;
			routeB.end.longitude = vertexB.longitude;
			routeB.end.latitude = vertexB.latitude;

			if(!isSamePoint && detectConflict(&routeA, &routeB) == 0 && totalDist < bestNoConflictDist) {
				bestNoConflictDist = totalDist;
				bestNoConflictA = vertexA;
				bestNoConflictB = vertexB;
				foundNoConflict = 1;
			}
		}
	}

	if(foundNoConflict) {
		inPointA[0] = bestNoConflictA;
		inPointA[1] = bestNoConflictB;
	} else if(foundAnyNonSame) {
		inPointA[0] = bestAnyA;
		inPointA[1] = bestAnyB;
	}
}

static int is_strategy9_plan_layout_from_result(const Information_the_results_of_tasking *task_result)
{
	int drone_count = 0;
	unsigned int main_area_code = 0;
	if(task_result == NULL)
	{
		return 0;
	}
	if(task_result->number_of_mission_platforms < 2)
	{
		return 0;
	}
	if(task_result->formation_synergy_mission_programs[0].number_of_subtasks < 2)
	{
		return 0;
	}
	if(task_result->formation_synergy_mission_programs[0].task_sequence_informations[0].sequence_type != 3 ||
			task_result->formation_synergy_mission_programs[0].task_sequence_informations[1].sequence_type != 1)
	{
		return 0;
	}
	main_area_code = task_result->formation_synergy_mission_programs[0].task_sequence_informations[0].target_number;
	if(main_area_code == 0)
	{
		return 0;
	}

	drone_count = task_result->number_of_mission_platforms - 1;
	if(drone_count < 1)
	{
		return 0;
	}
	if(drone_count > 4)
	{
		drone_count = 4;
	}

	for(int i = 1; i <= drone_count; i++)
	{
		if(task_result->formation_synergy_mission_programs[i].number_of_subtasks < 2)
		{
			return 0;
		}
		if(task_result->formation_synergy_mission_programs[i].task_sequence_informations[0].sequence_type != 14)
		{
			return 0;
		}
		if(task_result->formation_synergy_mission_programs[i].task_sequence_informations[0].target_number != main_area_code)
		{
			return 0;
		}
		if(task_result->formation_synergy_mission_programs[i].task_sequence_informations[1].sequence_type != 5 &&
				task_result->formation_synergy_mission_programs[i].task_sequence_informations[1].sequence_type != 7)
		{
			return 0;
		}
	}

	return 1;
}

static int get_area_bounds_for_strategy9_entry(const area_information *area,
		double *min_lat, double *max_lat, double *min_lon, double *max_lon)
{
	const double meters_per_deg_lat = 111320.0;
	if(area == NULL || min_lat == NULL || max_lat == NULL || min_lon == NULL || max_lon == NULL)
	{
		return 0;
	}

	if(area->area_shape == 1)
	{
		double radius_m = (double)(area->cycles.radius);
		double delta_lat = radius_m / meters_per_deg_lat;
		double cos_val = cos(area->cycles.latitude * (3.14159265358979323846 / 180.0));
		if(cos_val < 0) cos_val = -cos_val;
		if(cos_val < 0.01) cos_val = 0.01;
		double delta_lon = radius_m / (meters_per_deg_lat * cos_val);

		*min_lat = area->cycles.latitude - delta_lat;
		*max_lat = area->cycles.latitude + delta_lat;
		*min_lon = area->cycles.longitude - delta_lon;
		*max_lon = area->cycles.longitude + delta_lon;
		return 1;
	}

	if(area->area_shape == 2 && area->polygonals.point_number > 0)
	{
		*min_lat = area->polygonals.point_coordinates[0].latitude;
		*max_lat = *min_lat;
		*min_lon = area->polygonals.point_coordinates[0].longitude;
		*max_lon = *min_lon;

		for(int i = 1; i < area->polygonals.point_number; i++)
		{
			double lat = area->polygonals.point_coordinates[i].latitude;
			double lon = area->polygonals.point_coordinates[i].longitude;
			if(lat < *min_lat) *min_lat = lat;
			if(lat > *max_lat) *max_lat = lat;
			if(lon < *min_lon) *min_lon = lon;
			if(lon > *max_lon) *max_lon = lon;
		}
		return 1;
	}

	return 0;
}

void setZoneInPoint(GeoLibDas *inPoint)
{
	int droneNum = integrated_postures.drone_num;
	if(droneNum > 4) droneNum = 4;

	GeoLibDas uavPositionA[4];
	area_information areaA[4];
	int hasArea[4] = {0};

	for(int i = 0; i < droneNum; i++) {
		uavPositionA[i].longitude = integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.longitude;
		uavPositionA[i].latitude = integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.latitude;
		inPoint[i] = uavPositionA[i];

		for(int j = 0; j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].number_of_subtasks; j++) {
			unsigned short taskType = information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].sequence_type;
			if(taskType != 5 && taskType != 7) continue;

			unsigned int targetAreaCode =
					information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].target_number;
			for(int k = 0; k < area_sky_informations.area_number; k++) {
				if(targetAreaCode == area_sky_informations.area_informations[k].area_code) {
					areaA[i] = area_sky_informations.area_informations[k];
					hasArea[i] = 1;
					break;
				}
			}
			if(hasArea[i]) break;
		}
	}

	int use_strategy9_same_side_midpoint = 0;
	int entry_use_lon_side = 0;
	int entry_use_min_side = 0;
	if(droneNum == 2 && hasArea[0] && hasArea[1] &&
			is_strategy9_plan_layout_from_result(&information_on_the_results_of_taskings))
	{
		area_information main_area_tmp;
		area_information listen_area_tmp;
		memset(&main_area_tmp, 0, sizeof(area_information));
		memset(&listen_area_tmp, 0, sizeof(area_information));
		char found_main = 0;
		char found_listen = 0;

		unsigned int main_area_code =
				information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].target_number;
		unsigned int listen_area_code =
				information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].target_number;

		for(int k = 0; k < area_sky_informations.area_number; k++)
		{
			if(area_sky_informations.area_informations[k].area_code == main_area_code)
			{
				main_area_tmp = area_sky_informations.area_informations[k];
				found_main = 1;
			}
			else if(area_sky_informations.area_informations[k].area_code == listen_area_code)
			{
				listen_area_tmp = area_sky_informations.area_informations[k];
				found_listen = 1;
			}
		}

		if(found_main && found_listen)
		{
			double main_min_lat = 0.0, main_max_lat = 0.0, main_min_lon = 0.0, main_max_lon = 0.0;
			double listen_min_lat = 0.0, listen_max_lat = 0.0, listen_min_lon = 0.0, listen_max_lon = 0.0;
			if(get_area_bounds_for_strategy9_entry(&main_area_tmp, &main_min_lat, &main_max_lat, &main_min_lon, &main_max_lon) &&
					get_area_bounds_for_strategy9_entry(&listen_area_tmp, &listen_min_lat, &listen_max_lat, &listen_min_lon, &listen_max_lon))
			{
				double main_center_lat = (main_min_lat + main_max_lat) * 0.5;
				double main_center_lon = (main_min_lon + main_max_lon) * 0.5;
				double listen_center_lat = (listen_min_lat + listen_max_lat) * 0.5;
				double listen_center_lon = (listen_min_lon + listen_max_lon) * 0.5;

				double delta_ns_km = calculate_distances(main_center_lat, main_center_lon, listen_center_lat, main_center_lon);
				double delta_ew_km = calculate_distances(main_center_lat, main_center_lon, main_center_lat, listen_center_lon);

				if(delta_ew_km >= delta_ns_km)
				{
					// Listen Area is East/West. Enter from North or South (Perpendicular).
                    // Determine strict same-side based on proximity.
                    double cost_north = 0, cost_south = 0;
                    for(int u = 0; u < droneNum; u++) {
                         // Approx: North edge is max_lat, South edge is min_lat
                         double uav_lat = integrated_postures.integrated_posture_drone_informations[u].drone_longitude_and_latitude.latitude;
                         double uav_lon = integrated_postures.integrated_posture_drone_informations[u].drone_longitude_and_latitude.longitude;
                         cost_north += calculate_distances(uav_lat, uav_lon, main_max_lat, uav_lon);
                         cost_south += calculate_distances(uav_lat, uav_lon, main_min_lat, uav_lon);
                    }

                    entry_use_lon_side = 0; // Target N/S edges (Latitude constant)
					entry_use_min_side = (cost_south < cost_north) ? 1 : 0; // 1: Min(South), 0: Max(North)
				}
				else
				{
					// Listen Area is North/South. Enter from East or West (Perpendicular).
                    double cost_east = 0, cost_west = 0;
                    for(int u = 0; u < droneNum; u++) {
                         double uav_lat = integrated_postures.integrated_posture_drone_informations[u].drone_longitude_and_latitude.latitude;
                         double uav_lon = integrated_postures.integrated_posture_drone_informations[u].drone_longitude_and_latitude.longitude;
                         cost_east += calculate_distances(uav_lat, uav_lon, uav_lat, main_max_lon);
                         cost_west += calculate_distances(uav_lat, uav_lon, uav_lat, main_min_lon);
                    }

					entry_use_lon_side = 1; // Target E/W edges (Longitude constant)
					entry_use_min_side = (cost_west < cost_east) ? 1 : 0; // 1: Min(West), 0: Max(East)
				}
				use_strategy9_same_side_midpoint = 1;
			}
		}
	}

	if(use_strategy9_same_side_midpoint)
	{
		for(int i = 0; i < droneNum; i++)
		{
			if(hasArea[i])
			{
				double area_min_lat = 0.0, area_max_lat = 0.0, area_min_lon = 0.0, area_max_lon = 0.0;
				if(get_area_bounds_for_strategy9_entry(&areaA[i], &area_min_lat, &area_max_lat, &area_min_lon, &area_max_lon))
				{
					if(entry_use_lon_side)
					{
						inPoint[i].longitude = entry_use_min_side ? area_min_lon : area_max_lon;
						inPoint[i].latitude = (area_min_lat + area_max_lat) * 0.5;
					}
					else
					{
						inPoint[i].longitude = (area_min_lon + area_max_lon) * 0.5;
						inPoint[i].latitude = entry_use_min_side ? area_min_lat : area_max_lat;
					}
					// Snap to actual polygon boundary to avoid "off-area" points causing hangs or bad routes
					inPoint[i] = getNearestPointOnPolygonBoundary(inPoint[i], areaA[i]);
				}
				else
				{
					inPoint[i] = getNearestPolygonVertexForUav(uavPositionA[i], areaA[i]);
				}
			}
		}
		return;
	}

	for(int i = 0; i < droneNum; i++) {
		if(hasArea[i]) {
			inPoint[i] = getNearestPolygonVertexForUav(uavPositionA[i], areaA[i]);
		}
	}

	if(droneNum == 2 && hasArea[0] && hasArea[1]) {
		GeoLibDas dualUavPos[2] = {uavPositionA[0], uavPositionA[1]};
		area_information dualArea[2] = {areaA[0], areaA[1]};
		GeoLibDas dualInPoint[2] = {inPoint[0], inPoint[1]};
		optimizeDualUavInPoints(dualUavPos, dualArea, dualInPoint);
		inPoint[0] = dualInPoint[0];
		inPoint[1] = dualInPoint[1];
	}
}

void UAVRouteGeneration(){

	area_information area;
	GeoLibDas UAV_position;
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));
	//方案编号
	CTAS_DTMS_data_UAVRoute.program_number = information_on_the_results_of_taskings.program_number;
	GeoLibDas inPointA[4];
	setZoneInPoint(inPointA);

	//单个无人机的航路方案
	for(int i = 0;i < integrated_postures.drone_num;i++) {
		//航线长度
		UAV_task_length[i] = 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].drone_serial_number = drone_state_informations.drone_specific_informations[i].platform_serial_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].drone_num = drone_state_informations.drone_specific_informations[i].platform_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].subtasks_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].number_of_subtasks;

		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].number_of_subtasks;j++) {

			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].subtask_ID_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].subtask_ID_number;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].mission_type = information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].sequence_type;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].point_area_type = information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].type_of_mission_point_area;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].area_point_line_goal_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].target_number;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].AtomicTimeUpper = information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].completion_time_valid_Bits;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].AtomicHighlyUpper = information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].task_height_effective_position;
			if(j == 0) {//找到无人机当前位置
				UAV_position.longitude = integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.longitude;
				UAV_position.latitude = integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.latitude;
			}
			//            else
			//            {//todo: 无人机位置后续需要根据子任务的最后一个航路点更新。(作为下一个子任务的初始位置)
			//                int waypoint_index = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j-1].waypoints_number - 1;
			//                UAV_position.longitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j-1].planning_information_waypoint_informations[waypoint_index].longitude;
			//                UAV_position.latitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j-1].planning_information_waypoint_informations[waypoint_index].latitude;
			//            }

			//找到区域后根据不同的任务类型完成相应的任务分配
			for(int k = 0;k < area_sky_informations.area_number;k++){
				//找到对应的任务区域，为临时变量赋值
				if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].target_number == area_sky_informations.area_informations[k].area_code){
					area = area_sky_informations.area_informations[k];
				}
			}

			//如果该无人机的子任务
			if(area.area_code == 0) {
				//任务区code==0时，不生成航线，子任务类型是待机or返航。
				continue;
			}

			//subtask:浮标桢收
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].sequence_type == 1){
				OutBuoyMonitorAirway outBuoyMonitorAirway;
				//找出浮标侦听区域对应的浮标布阵子任务ID
				unsigned int ID = 0;
				for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;j++){
					if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[j].target_number == area.area_code &&
							information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[j].sequence_type == 3){
						ID = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[j].subtask_ID_number;
						break;
					}
				}
				for(int j = 0;j < CTAS_DTMS_data_mannedRoute.buoy_deployment_points.subtasks_number;j++){
					if(ID == CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[j].subtask_ID_number){
						outBuoyMonitorAirway = buyo_monitor_way_generation(CTAS_DTMS_data_mannedRoute.buoy_deployment_points.buoy_manned_computer_tasks[j],UAV_position);
					}
				}


				if(outBuoyMonitorAirway.sum > 75) outBuoyMonitorAirway.sum = 75;
				//通用航路
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].waypoints_number = outBuoyMonitorAirway.sum;

				//计算航路信息包数 20250611new
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].total_packet = outBuoyMonitorAirway.sum/25;
				if(outBuoyMonitorAirway.sum%25)
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].total_packet++;

				for(int k = 0;k < outBuoyMonitorAirway.sum;k++){
					//计算航路点编号 20250611new
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].hld_idx = k+1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].longitude = outBuoyMonitorAirway.awpA[k].longitude;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].latitude = outBuoyMonitorAirway.awpA[k].latitude;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].validity_of_longitude = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].latitude_validity = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].height_validity = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].speed_validity = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].direction_validity = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].time_validity = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].payloads_validity = 1;
					//高度---待补充
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].height = blk_dtms_ctas_010[i].FBZS;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].speed = 35;
					//速度---待补充
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].direction = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].time = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].payloads = 2;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].causality = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_type = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius_valid_bit = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber_valid_bit = 0;
					//最后一点点为盘旋点
					if(k == outBuoyMonitorAirway.sum - 1)
					{
						//航路点待机时间/圈数/循环次数
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber
						= PX_CIRCLE;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
						//航路点待机半径
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius
						= PX_RAD;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius_valid_bit = 1;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].causality = 3;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_type = 2;//圈数
					}
				}


			}else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].sequence_type == 5) {
				//subtask:磁探搜索
				//                OutAirwayLibDas outAirwayLibDas;
				//                outAirwayLibDas = citanGeneration(area,UAV_position);
				//
				//                //最多75个点 20250611new
				//                if(outAirwayLibDas.sumAwp > 75) outAirwayLibDas.sumAwp = 75;
				//                CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].waypoints_number = outAirwayLibDas.sumAwp;
				//                //计算航路信息包数 20250611new
				//                CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].total_packet = outAirwayLibDas.sumAwp/25;
				//                if(outAirwayLibDas.sumAwp%25)
				//                    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].total_packet++;
				OutSearchRadarPhoto outSearchRadarPhoto;
				outSearchRadarPhoto = commonRouteGeneration(area,inPointA[i],600);

				//最多75个点 20250611new
				if(outSearchRadarPhoto.sumAwp > 75) outSearchRadarPhoto.sumAwp = 75;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].waypoints_number = outSearchRadarPhoto.sumAwp;

				//计算航路信息包数 20250611new
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].total_packet = outSearchRadarPhoto.sumAwp/25;
				if(outSearchRadarPhoto.sumAwp%25)
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].total_packet++;

				for(int k = 0;k < outSearchRadarPhoto.sumAwp;k++){
					//计算航路点编号 20250611new
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].hld_idx = k+1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].longitude = outSearchRadarPhoto.awpA[k].longitude;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].latitude = outSearchRadarPhoto.awpA[k].latitude;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].validity_of_longitude = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].latitude_validity = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].height_validity = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].speed_validity = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].direction_validity = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].time_validity = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].payloads_validity = 1;
					//高度---待补充
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].height = blk_dtms_ctas_010[i].CTSS;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].speed = 30;
					//速度---待补充
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].direction = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].time = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].payloads = 2;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].causality = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_type = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius_valid_bit = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber_valid_bit = 0;
					//最后一点点为盘旋点
					if(k == outSearchRadarPhoto.sumAwp - 1)
					{
						//航路点待机时间/圈数/循环次数
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber
						= PX_CIRCLE;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
						//航路点待机半径
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius
						= PX_RAD;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius_valid_bit = 1;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].causality = 3;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_type = 2;//圈数
					}
				}


				//无人机路径长度---无长度
				UAV_task_length[i] += outSearchRadarPhoto.distance;

			}else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].sequence_type == 7){
				//subtask:光电搜索
				OutSearchRadarPhoto outSearchRadarPhoto;
				outSearchRadarPhoto = commonRouteGeneration(area,inPointA[i],12000);

				//最多75个点 20250611new
				if(outSearchRadarPhoto.sumAwp > 75) outSearchRadarPhoto.sumAwp = 75;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].waypoints_number = outSearchRadarPhoto.sumAwp;

				//计算航路信息包数 20250611new
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].total_packet = outSearchRadarPhoto.sumAwp/25;
				if(outSearchRadarPhoto.sumAwp%25)
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].total_packet++;

				for(int k = 0;k < outSearchRadarPhoto.sumAwp;k++){
					//计算航路点编号 20250611new
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].hld_idx = k+1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].longitude = outSearchRadarPhoto.awpA[k].longitude;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].latitude = outSearchRadarPhoto.awpA[k].latitude;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].validity_of_longitude = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].latitude_validity = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].height_validity = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].speed_validity = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].direction_validity = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].time_validity = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].payloads_validity = 1;
					//高度---待补充
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].height = blk_dtms_ctas_010[i].GDSS;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].speed = 35;
					//速度---待补充
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].direction = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].time = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].payloads = 2;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].causality = 1;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_type = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius_valid_bit = 0;
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber_valid_bit = 0;
					//最后一点点为盘旋点
					if(k == outSearchRadarPhoto.sumAwp - 1)
					{
						//航路点待机时间/圈数/循环次数
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber
						= PX_CIRCLE;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
						//航路点待机半径
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius
						= PX_RAD;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_radius_valid_bit = 1;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].causality = 3;
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[k].standby_type = 2;//圈数
					}
				}


				//无人机路径长度---无长度
				UAV_task_length[i] += outSearchRadarPhoto.distance;


			}
			//等待
			else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[j].sequence_type == 12 && j!=0
					&&  CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j-1].waypoints_number != 0)
			{
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].waypoints_number = 1;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].total_packet = 1;

				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].hld_idx = 1;
				//取上一个任务的最后一个点
				unsigned int point_index = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j-1].waypoints_number - 1;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].longitude =
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j-1].planning_information_waypoint_informations[point_index].longitude;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].latitude =
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j-1].planning_information_waypoint_informations[point_index].latitude;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].validity_of_longitude = 1;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].latitude_validity = 1;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].height_validity = 1;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].speed_validity = 1;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].direction_validity = 0;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].time_validity = 0;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].payloads_validity = 1;
				//高度---待补充
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].height = 500;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].speed = 35;
				//航路点待机时间/圈数/循环次数
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber
				= PX_CIRCLE;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
				//航路点待机半径
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].standby_radius
				= PX_RAD;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].causality = 3;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].planning_information_waypoint_informations[0].standby_type = 2;//圈数
			}

			// 盘旋任务处理 (sequence_type == 14)
			// 功能：基于任务区域外扩5公里生成8个候选点，并选取离参考点最近的等待点
			else if (information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1]
			             .task_sequence_informations[j].sequence_type == 14)
			{
			    // =========================
			    // 0) 可选：多机协同占用表（默认关闭，避免“幽灵占用”）
			    // =========================
			    // static unsigned char pts_occupancy[16][8];
			    // if (i == 0 && j < 16) { memset(pts_occupancy[j], 0, sizeof(pts_occupancy[j])); }

			    // =========================
			    // 1) 变量定义
			    // =========================
			    double droneLat = 0.0, droneLon = 0.0;
			    double rawMinLat = 0.0, rawMaxLat = 0.0;
			    double rawMinLon = 0.0, rawMaxLon = 0.0;
			    double finalMinLat = 0.0, finalMaxLat = 0.0, finalMinLon = 0.0, finalMaxLon = 0.0;

			    const double expand_meters = 5000.0;       // 固定外扩 5km
			    const double meters_per_deg_lat = 111320.0; // 1度纬度约 111.32km

			    double avg_lat_rad = 0.0, cos_val = 0.0;
			    double expand_deg_lat = 0.0, expand_deg_lon = 0.0;

			    double minDistance = -1.0;
			    double closestLat = 0.0, closestLon = 0.0;
			    int selected_idx = -1;

			    Geo points[8];
			    int k = 0, idx = 0;
			    int valid_shape = 0;
			    int use_strategy9_custom_wait_point = 0;

			    // =========================
			    // 2) 获取参考位置（默认：用 UAV_position，符合“起始位置最近”口径）
			    // =========================
			    // --- 默认策略：按 UAV_position（起始/初始化点） ---
			    droneLat = UAV_position.latitude;
			    droneLon = UAV_position.longitude;

			    // --- 如需切回“上一阶段终点接力”，用下面这段替换上面两行 ---
			    /*
			    if (j > 0 &&
			        CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			            .planning_informations[j - 1].waypoints_number > 0)
			    {
			        unsigned int last_idx =
			            CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			                .planning_informations[j - 1].waypoints_number - 1;

			        droneLat = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			                       .planning_informations[j - 1]
			                       .planning_information_waypoint_informations[last_idx].latitude;

			        droneLon = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			                       .planning_informations[j - 1]
			                       .planning_information_waypoint_informations[last_idx].longitude;
			    }
			    else
			    {
			        droneLat = UAV_position.latitude;
			        droneLon = UAV_position.longitude;
			    }
			    */

			    // =========================
			    // 3) 解析原始区域 -> 得到 rawMin/Max Lat/Lon
			    // =========================
			    if (area.area_shape == 1) // 圆形
			    {
			        double radius = (double)(area.cycles.radius);
			        if (radius < 10.0) radius = 100.0; // 防呆：半径太小给个默认值

			        double delta_deg_lat = radius / meters_per_deg_lat;
			        rawMinLat = area.cycles.latitude - delta_deg_lat;
			        rawMaxLat = area.cycles.latitude + delta_deg_lat;

			        double center_rad = area.cycles.latitude * (3.14159265358979323846 / 180.0);
			        double cos_c = cos(center_rad);
			        if (cos_c < 0) cos_c = -cos_c;
			        if (cos_c < 0.01) cos_c = 0.01;

			        double delta_deg_lon = radius / (meters_per_deg_lat * cos_c);
			        rawMinLon = area.cycles.longitude - delta_deg_lon;
			        rawMaxLon = area.cycles.longitude + delta_deg_lon;

			        valid_shape = 1;
			    }
			    else if (area.area_shape == 2) // 多边形
			    {
			        if (area.polygonals.point_coordinates != NULL && area.polygonals.point_number > 0)
			        {
			            rawMinLat = area.polygonals.point_coordinates[0].latitude;
			            rawMaxLat = rawMinLat;
			            rawMinLon = area.polygonals.point_coordinates[0].longitude;
			            rawMaxLon = rawMinLon;

			            for (k = 1; k < area.polygonals.point_number; k++)
			            {
			                double tmpLat = area.polygonals.point_coordinates[k].latitude;
			                double tmpLon = area.polygonals.point_coordinates[k].longitude;

			                if (tmpLat < rawMinLat) rawMinLat = tmpLat;
			                if (tmpLat > rawMaxLat) rawMaxLat = tmpLat;
			                if (tmpLon < rawMinLon) rawMinLon = tmpLon;
			                if (tmpLon > rawMaxLon) rawMaxLon = tmpLon;
			            }

			            valid_shape = 1;
			        }
			    }

			    // 解析失败兜底：用参考点当“区域”
			    if (valid_shape == 0)
			    {
			        rawMinLat = droneLat; rawMaxLat = droneLat;
			        rawMinLon = droneLon; rawMaxLon = droneLon;
			    }

			    // 战法9阶段1双机盘旋点：取主任务区对边外2km，且两机分别与侧边两端对齐
			    if (integrated_postures.drone_num == 2 &&
			            j == 0 &&
			            information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks >= 2 &&
			            information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].sequence_type == 3 &&
			            information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].sequence_type == 1 &&
			            information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[0].sequence_type == 14)
			    {
			        unsigned int main_area_code =
			        		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].target_number;
			        unsigned int listen_area_code =
			        		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].target_number;
			        area_information listen_area_tmp;
			        memset(&listen_area_tmp, 0, sizeof(area_information));
			        char found_main_area = 0;
			        char listen_shape_ok = 0;

			        for (int area_idx = 0; area_idx < area_sky_informations.area_number; area_idx++)
			        {
			        	if(area_sky_informations.area_informations[area_idx].area_code == main_area_code) {
			        		found_main_area = 1;
			        	}
			        	else if(area_sky_informations.area_informations[area_idx].area_code == listen_area_code) {
			        		listen_area_tmp = area_sky_informations.area_informations[area_idx];
			        	}
			        }

			        double listen_min_lat = 0.0, listen_max_lat = 0.0;
			        double listen_min_lon = 0.0, listen_max_lon = 0.0;
			        if(listen_area_tmp.area_shape == 1) {
			        	double listen_delta_lat = (double)(listen_area_tmp.cycles.radius) / meters_per_deg_lat;
			        	listen_min_lat = listen_area_tmp.cycles.latitude - listen_delta_lat;
			        	listen_max_lat = listen_area_tmp.cycles.latitude + listen_delta_lat;
			        	double listen_cos = cos(listen_area_tmp.cycles.latitude * (3.14159265358979323846 / 180.0));
			        	if (listen_cos < 0) listen_cos = -listen_cos;
			        	if (listen_cos < 0.01) listen_cos = 0.01;
			        	double listen_delta_lon = (double)(listen_area_tmp.cycles.radius) / (meters_per_deg_lat * listen_cos);
			        	listen_min_lon = listen_area_tmp.cycles.longitude - listen_delta_lon;
			        	listen_max_lon = listen_area_tmp.cycles.longitude + listen_delta_lon;
			        	listen_shape_ok = 1;
			        }
			        else if(listen_area_tmp.area_shape == 2 && listen_area_tmp.polygonals.point_number > 0) {
			        	listen_min_lat = listen_area_tmp.polygonals.point_coordinates[0].latitude;
			        	listen_max_lat = listen_min_lat;
			        	listen_min_lon = listen_area_tmp.polygonals.point_coordinates[0].longitude;
			        	listen_max_lon = listen_min_lon;
			        	for (int lp = 1; lp < listen_area_tmp.polygonals.point_number; lp++) {
			        		double llat = listen_area_tmp.polygonals.point_coordinates[lp].latitude;
			        		double llon = listen_area_tmp.polygonals.point_coordinates[lp].longitude;
			        		if(llat < listen_min_lat) listen_min_lat = llat;
			        		if(llat > listen_max_lat) listen_max_lat = llat;
			        		if(llon < listen_min_lon) listen_min_lon = llon;
			        		if(llon > listen_max_lon) listen_max_lon = llon;
			        	}
			        	listen_shape_ok = 1;
			        }

			        if(found_main_area && listen_shape_ok)
			        {
			        	double main_center_lat = (rawMinLat + rawMaxLat) * 0.5;
			        	double main_center_lon = (rawMinLon + rawMaxLon) * 0.5;
			        	double listen_center_lat = (listen_min_lat + listen_max_lat) * 0.5;
			        	double listen_center_lon = (listen_min_lon + listen_max_lon) * 0.5;

			        	double lat_offset_2km = 2000.0 / meters_per_deg_lat;
			        	double main_cos = cos(main_center_lat * (3.14159265358979323846 / 180.0));
			        	if (main_cos < 0) main_cos = -main_cos;
			        	if (main_cos < 0.01) main_cos = 0.01;
			        	double lon_offset_2km = 2000.0 / (meters_per_deg_lat * main_cos);

			        	double delta_ns_km = calculate_distances(main_center_lat, main_center_lon, listen_center_lat, main_center_lon);
			        	double delta_ew_km = calculate_distances(main_center_lat, main_center_lon, main_center_lat, listen_center_lon);
			        	double uav0_lat = integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.latitude;
			        	double uav0_lon = integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.longitude;
			        	double uav1_lat = integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.latitude;
			        	double uav1_lon = integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.longitude;

			        	if (delta_ew_km >= delta_ns_km)
			        	{
			        		// 监听区在东西向：盘旋点取主任务区对边（西/东）外侧2km，双机贴南北边
			        		double side_lon = (listen_center_lon >= main_center_lon) ? (rawMinLon - lon_offset_2km) : (rawMaxLon + lon_offset_2km);
			        		double side_lat_a = rawMinLat;
			        		double side_lat_b = rawMaxLat;
			        		double cost_ab = calculate_distances(uav0_lat, uav0_lon, side_lat_a, side_lon) +
			        					calculate_distances(uav1_lat, uav1_lon, side_lat_b, side_lon);
			        		double cost_ba = calculate_distances(uav0_lat, uav0_lon, side_lat_b, side_lon) +
			        					calculate_distances(uav1_lat, uav1_lon, side_lat_a, side_lon);

			        		closestLon = side_lon;
			        		if ((cost_ab <= cost_ba && i == 0) || (cost_ab > cost_ba && i == 1))
			        		{
			        			closestLat = side_lat_a;
			        		}
			        		else
			        		{
			        			closestLat = side_lat_b;
			        		}
			        	}
			        	else
			        	{
			        		// 监听区在南北向：盘旋点取主任务区对边（南/北）外侧2km，双机贴东西边
			        		double side_lat = (listen_center_lat >= main_center_lat) ? (rawMinLat - lat_offset_2km) : (rawMaxLat + lat_offset_2km);
			        		double side_lon_a = rawMinLon;
			        		double side_lon_b = rawMaxLon;
			        		double cost_ab = calculate_distances(uav0_lat, uav0_lon, side_lat, side_lon_a) +
			        					calculate_distances(uav1_lat, uav1_lon, side_lat, side_lon_b);
			        		double cost_ba = calculate_distances(uav0_lat, uav0_lon, side_lat, side_lon_b) +
			        					calculate_distances(uav1_lat, uav1_lon, side_lat, side_lon_a);

			        		closestLat = side_lat;
			        		if ((cost_ab <= cost_ba && i == 0) || (cost_ab > cost_ba && i == 1))
			        		{
			        			closestLon = side_lon_a;
			        		}
			        		else
			        		{
			        			closestLon = side_lon_b;
			        		}
			        	}

			        	use_strategy9_custom_wait_point = 1;
			        }
			    }

			    // =========================
			    // 4) 外扩 5km
			    // =========================
			    if(!use_strategy9_custom_wait_point)
			    {
			    	expand_deg_lat = expand_meters / meters_per_deg_lat;
			    	finalMaxLat = rawMaxLat + expand_deg_lat;
			    	finalMinLat = rawMinLat - expand_deg_lat;

			    	avg_lat_rad = (rawMinLat + rawMaxLat) * 0.5 * (3.14159265358979323846 / 180.0);
			    	cos_val = cos(avg_lat_rad);
			    	if (cos_val < 0) cos_val = -cos_val;
			    	if (cos_val < 0.01) cos_val = 0.01;

			    	expand_deg_lon = expand_meters / (meters_per_deg_lat * cos_val);
			    	finalMaxLon = rawMaxLon + expand_deg_lon;
			    	finalMinLon = rawMinLon - expand_deg_lon;

			    	// =========================
			    	// 5) 生成 8 个候选点（左下、右下、左上、右上、左中、右中、下中、上中）
			    	// =========================
			    	points[0].latitude = finalMinLat; points[0].longitude = finalMinLon; // 左下
			    	points[1].latitude = finalMinLat; points[1].longitude = finalMaxLon; // 右下
			    	points[2].latitude = finalMaxLat; points[2].longitude = finalMinLon; // 左上
			    	points[3].latitude = finalMaxLat; points[3].longitude = finalMaxLon; // 右上
			    	points[4].latitude = (finalMinLat + finalMaxLat) / 2.0; points[4].longitude = finalMinLon; // 左中
			    	points[5].latitude = (finalMinLat + finalMaxLat) / 2.0; points[5].longitude = finalMaxLon; // 右中
			    	points[6].latitude = finalMinLat; points[6].longitude = (finalMinLon + finalMaxLon) / 2.0; // 下中
			    	points[7].latitude = finalMaxLat; points[7].longitude = (finalMinLon + finalMaxLon) / 2.0; // 上中

			    	// =========================
			    	// 6) 选最近点（默认：不看占用，纯几何最近）
			    	// =========================
			    	closestLat = points[0].latitude;
			    	closestLon = points[0].longitude;

			    	for (idx = 0; idx < 8; idx++)
			    	{
			    		// 如需启用占用判定，打开下面这行，并同时启用上面的 pts_occupancy 定义与清零
			    		// if (j < 16 && pts_occupancy[j][idx] == 1) continue;

			    		double d = calculate_distances(droneLat, droneLon, points[idx].latitude, points[idx].longitude);
			    		if (minDistance < 0 || d < minDistance)
			    		{
			    			minDistance = d;
			    			closestLat = points[idx].latitude;
			    			closestLon = points[idx].longitude;
			    			selected_idx = idx;
			    		}
			    	}
			    }

			    // 如需标记占用，打开下面这行，并同时启用 pts_occupancy
			    // if (selected_idx >= 0 && j < 16) { pts_occupancy[j][selected_idx] = 1; }

			    // =========================
			    // 7) 回填输出（waypoint[0]）
			    // =========================
			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].longitude = closestLon;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].latitude  = closestLat;

			    // 有效位
			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].validity_of_longitude = 1;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].latitude_validity = 1;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].height_validity = 1;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].speed_validity = 1;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].direction_validity = 0;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].time_validity = 0;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].payloads_validity = 1;

			    // 业务参数
			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].height = blk_dtms_ctas_010[i].PX;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].speed = 35;

			    // 盘旋指令参数（关键：valid_bit 一定要置 1）
			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber = PX_CIRCLE;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].standby_radius = PX_RAD;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].causality = 3;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].planning_information_waypoint_informations[0].standby_type = 2;

			    // 包头计数
			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].waypoints_number = 1;

			    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i]
			        .planning_informations[j].total_packet = 1;
			}

			//其他任务
			else
			{
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[j].total_packet = 1;
			}

		}
	}
}

/*
 * 工具函数---求飞机和区域距离最近的点的距离
 * */
double minDistance(GeoLibDas UAVLocation,area_information area){
	//临时变量
	GeoLibDas pointLocation;//点位置
	if(area.area_shape == 1){
		return getDistanceGeoLibDas(&UAVLocation,&pointLocation) - (double)(area.cycles.radius);
	}
	pointLocation = minDIstancePoint(UAVLocation,area);
	return getDistanceGeoLibDas(&UAVLocation,&pointLocation);
}

GeoLibDas minDIstancePoint(GeoLibDas UAVLocation, area_information area){
	//临时变量
	double mDistance = 0;//当前最小距离
	double distance = 0;//距离
	GeoLibDas pointLocation;//点位置
	GeoLibDas minPointLocation;//最近距离的点

	if(area.area_shape == 1){//当区域是圆形时，取圆心距离减去半径
		pointLocation.longitude = area.cycles.longitude;
		pointLocation.latitude = area.cycles.latitude;
		mDistance = getDistanceGeoLibDas(&UAVLocation,&pointLocation) - (double)(area.cycles.radius);//计算最小距离
		minPointLocation = pointLocation;
	}else if(area.area_shape == 2){//当区域是多边形时，取最小距离点
		//计算第一个点的距离
		pointLocation.longitude = area.polygonals.point_coordinates[0].longitude;
		pointLocation.latitude = area.polygonals.point_coordinates[0].latitude;
		mDistance = getDistanceGeoLibDas(&UAVLocation,&pointLocation);
		minPointLocation = pointLocation;
		//计算后续点距离并与当前最小距离对比
		for(int i = 1;i < area.polygonals.point_number;i++){
			pointLocation.longitude = area.polygonals.point_coordinates[i].longitude;
			pointLocation.latitude = area.polygonals.point_coordinates[i].latitude;
			distance = getDistanceGeoLibDas(&UAVLocation,&pointLocation);
			if(distance < mDistance){
				mDistance = distance;
				minPointLocation = pointLocation;
			}
		}
	}

	return minPointLocation;
}


/*
 * 战法二
 * */
void secondStrategy(){
	secondStrategy_missionDistributeInformation();  //战法二---任务结果分配信息
}

//战法二---任务结果分配信息
void secondStrategy_missionDistributeInformation(){
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定
	//协同搜潜方式---4.9 战术战法选择

	//方案总时间---待定
	//任务平台个数---3.2 无人机个数2 + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number + 1;


	/*
	 * 有人机任务信息分配结果
	 * */
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_model = 1;//平台型号--有人机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID

	//平台任务时---待定
	//平台子任务个数---任务区/应召点个数
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks
	= global_mission_planning_commandss.area_point_num;

	mannedAreaSorted();//组织任务区顺序

	//任务序列信息
	for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 2;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 1;
	}
	//    //分配有人机返航任务
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = 0;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 11;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[global_mission_planning_commandss.area_point_num].presence_of_fixed_point_arrays = 0;



	//无人机任务赋值
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= global_mission_planning_commandss.area_point_num;

		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 5;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[i -1 ].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 0;
		}

	}
	//    //分配无人机返航任务
	//    for(int i = 1;i <= drone_state_informations.drone_number;i++){
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = 0;
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 11;
	//
	//
	//    }


	for(int i = 0;i < area_sky_informations.area_number;i++){//区域最大个数为2
		//找出非有人机第0个任务的区域
		if(area_sky_informations.area_informations[i].area_code != information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].target_number){
			//判断该区域距离哪个无人机近，就由哪个无人机执行磁探搜索任务
			if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[0],integrated_postures.integrated_posture_drone_informations[1],area_sky_informations.area_informations[i])){//如果无人机1比无人机2距离区域远，则无人机1执行中继
				//为无人机1赋值
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].target_number = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 4;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 4;
				//为无人机2赋值
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = area_sky_informations.area_informations[i].area_code;
				if(i == 0){
					information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].target_number = area_sky_informations.area_informations[1].area_code;
				}else if(i == 1){
					information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;
				}
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].sequence_type = 5;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 5;
			}else{
				//为无人机2赋值
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].target_number = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].sequence_type = 4;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 4;
				//为无人机1赋值
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[i].area_code;
				if(i == 0){
					information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].target_number = area_sky_informations.area_informations[1].area_code;
				}else if(i == 1){
					information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;
				}
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 5;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 5;
			}
		}
	}
	//赋值待机
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks - 1;j++){
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number == 0 && information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type != 4){
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 14;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 0;
			}
		}
	}
}


/*
 * 战法三
 * */
void thirdStrategy(){
	thirdStrategy_missionDistributeInformation();   //1.任务结果分配信息
}

//战法三---任务结果分配信息 (根据分配信息，匹配任务区，再做step2的规划)
void thirdStrategy_missionDistributeInformation(){
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定
	//协同搜潜方式---4.9 战术战法选择

	//方案总时间---待定todo
	//任务平台个数---3.2 无人机个数n + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number + 1;


	/*
	 * 有人机任务信息分配结果
	 * */
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_model = 1;//平台型号--有人机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID

	//平台任务时---待定
	//平台子任务个数---任务区/应召点个数
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks
	= global_mission_planning_commandss.area_point_num  + 1;    //阶段数等于子区域数加1

	//组织任务区顺序(sort)
	mannedAreaSorted();

	//任务序列及任务区信息
	for(int i = 0;i < (information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks -1);i++) {
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 3;  //subtask type
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3; //task point or area. (3-area)
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 0;
	}

	//如果浮标布阵区域大于无人机数量，需要有人机加入浮标监听任务
	if(global_mission_planning_commandss.area_point_num == information_on_the_results_of_taskings.number_of_mission_platforms) {
		int minDistance = 0;//记录距离有人机最近的位置
		GeoLibDas location;

		location.longitude = integrated_postures.longitude;
		location.latitude = integrated_postures.latitude;
		for(int i = 0;i < global_mission_planning_commandss.area_point_num;i++){//找出距离有人机的最近位置，分配有人机去
			if(distanceComparison(location,area_sky_informations.area_informations[minDistance],area_sky_informations.area_informations[i])){
				minDistance = i;
			}
		}

		int subtasks = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1;
		//有人机任务分配
		//基础信息
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].subtask_ID_number = subtasks + 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].sequence_type = 1;//subtask type
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].presence_of_buoy_array = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].availability_of_routes = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].target_number = area_sky_informations.area_informations[subtasks -1].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].mission_height = 1000;

	}else{
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].sequence_type = 12;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].target_number =
				information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 2].target_number;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 2].target_number = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 2].presence_of_buoy_array = 0;

	}

	/*
	 * 无人机任务信息分配结果
	 * */
	int uav_task_num[4] = {0,0,0,0};
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数 + 巡逻＋返航
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= global_mission_planning_commandss.area_point_num + 1;
		uav_task_num[i-1] = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;

		//为任务基础数据进行赋值
		for(int j = 0;j < (information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks -1);j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[i -1 ].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 0;
		}

	}

	//确定第一轮任务时无人机的任务分配
	UAVAreaDistribute();
	//将后续任务全都赋值为第一个任务（主要原因是如果后面有人机未经过当前无人机的位置，则无人机不应该再移动）
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		for(int j = 1;j < global_mission_planning_commandss.area_point_num;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].target_number;
		}
	}
	//确定余下的任务分配
	for(int i = 1;i < global_mission_planning_commandss.area_point_num;i++){//外层为有人机后续任务编号
		for(int j = 1;j <= drone_state_informations.drone_number;j++){//如果无人机的第一个任务的任务区与有人机后续任务区的编号相同,则把无人机后续的任务区全部赋值成有人机当前任务区编号
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[j].task_sequence_informations[0].target_number == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number){
				for(int k = i;k < global_mission_planning_commandss.area_point_num;k++){
					information_on_the_results_of_taskings.formation_synergy_mission_programs[j].task_sequence_informations[k].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i - 1].target_number;
				}
			}
		}
	}


	//无人机监听任务
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		//平台子任务个数---任务区/应召点个数 + 巡逻＋返航
		int subtasks = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1;
		//为任务基础数据进行赋值
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].subtask_ID_number = (unsigned)(subtasks + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].sequence_type = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].type_of_mission_point_area = 3;
		int target_num1 = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks -1].target_number;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].target_number = (unsigned)(target_num1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].availability_of_routes = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].presence_of_buoy_array = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[subtasks].presence_of_fixed_point_arrays = 0;

	}


//	//无人机监听任务
//	int used_area[3] = {4,4,4};
//	char used_flag = false;//如果该区域已经被使用，则变为true
//	int min_area = -1;//记录最近区域的距离
//	GeoLibDas drone_location;
//	for(int i = 0;i < drone_state_informations.drone_number;i++){
//		drone_location.longitude = integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.longitude;
//		drone_location.latitude = integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.latitude;
//		for(int j = 0;j < area_sky_informations.area_number;j++){
//			if(area_sky_informations.area_informations[j].area_code != information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 2].target_number){
//
//				//判断该区域是否已经被分配
//				for(int k = 0;k < 3;k++){
//					if(j == used_area[k]){
//						used_flag = true;
//						break;
//					}
//				}
//
//				if(used_flag == true){//如果被分配
//					used_flag = false;
//					continue;
//				}
//				//找出一个未被使用的作为当前最小值
//				min_area = j;
//
//				if(distanceComparison(drone_location,area_sky_informations.area_informations[min_area],area_sky_informations.area_informations[j])){
//					min_area = j;
//				}
//			}
//		}
//		for(int j = 0;j < area_sky_informations.area_number;j++){
//			if(area_sky_informations.area_informations[j].area_code != information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 2].target_number){
//
//				//判断该区域是否已经被分配
//				for(int k = 0;k < 3;k++){
//					if(j == used_area[k]){
//						used_flag = true;
//						break;
//					}
//				}
//
//				if(used_flag == true){//如果被分配
//					used_flag = false;
//					continue;
//				}
//				if(distanceComparison(drone_location,area_sky_informations.area_informations[min_area],area_sky_informations.area_informations[j])){
//					min_area = j;
//				}
//			}
//		}
//		if(min_area != -1){
//			used_area[i] = min_area;
//			information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 1;
//			information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = area_sky_informations.area_informations[min_area].area_code;
//			min_area = -1;
//		}else{//否则就待机
//			information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 12;
//
//			//            information_on_the_results_of_taskings.formation_synergy_mission_programs[i + 1].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = area_sky_informations.area_informations[min_area].area_code;
//		}
//
//	}


	//赋值待机
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks - 1;j++){
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number == 0){
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 14;//20260128
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 0;
			}
		}
	}
	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}

	//方案时长航程计算
	uav_time_range_calc();
}


/*
 * 战术四
 * */
void fourthStrategy(){
	fourthStrategy_missionDistributeInformation();
}

//战法四---任务结果分配信息
void fourthStrategy_missionDistributeInformation(){
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定
	//协同搜潜方式---4.9 战术战法选择

	//方案总时间---待定
	//任务平台个数---3.2 无人机个数n + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number + 1;

	/*
	 * 有人机任务信息分配结果
	 * */
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_model = 1;//平台型号--有人机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID

	//平台任务时---待定
	//平台子任务个数---任务区/应召点个数
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks
	= global_mission_planning_commandss.area_point_num + 1;
	mannedAreaSorted();//组织任务区顺序
	//任务序列信息
	for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 0;
	}
	//如果浮标布阵区域大于无人机数量，需要有人机加入浮标监听任务
	int subtasks = 0;
	if(drone_state_informations.drone_number == 1)
		subtasks = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 2;
	else if(drone_state_informations.drone_number == 2)
		subtasks = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1;
	if(global_mission_planning_commandss.area_point_num > 1){

		int minDistance = 0;//记录距离有人机最近的位置
		GeoLibDas location;
		location.longitude = integrated_postures.longitude;
		location.latitude = integrated_postures.latitude;
		for(int i = 0;i < global_mission_planning_commandss.area_point_num;i++){//找出距离有人机的最近位置，分配有人机去
			if(distanceComparison(location,area_sky_informations.area_informations[minDistance],area_sky_informations.area_informations[i])){
				minDistance = i;
			}
		}

		//        int subtasks = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 2;
		//有人机任务分配
		//基础信息
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].sequence_type = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].target_number = area_sky_informations.area_informations[minDistance].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].availability_of_routes = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].presence_of_buoy_array = 0;
	}else{//否则就待机
		//        int subtasks = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 2;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].sequence_type = 12;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].presence_of_buoy_array = 0;
	}

	//    //分配有人机返航任务
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].sequence_type = 11;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].presence_of_buoy_array = 0;


	//无人机任务与有人机任务执行的浮标布放任务相关,因此以有人机浮标布放任务的顺序为无人机任务分配顺序,第一阶段无人机b盘旋、无人机A执行磁探搜索任务
	/*
	 * 无人机任务信息分配结果
	 * */
	int uav_task_num[4] = {0,0,0,0};
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数 ＋ 返航
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
			= global_mission_planning_commandss.area_point_num + 1;//保持无人机与有人机阶段数一致20260201
		uav_task_num[i-1] = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;

		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 2;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[i -1 ].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 1;
		}

	}
	//    //分配无人机返航任务
	//    for(int i = 1;i <= drone_state_informations.drone_number;i++){
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 11;
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].presence_of_fixed_point_arrays = 0;
	//    }


	//第一次任务分配，无人机A进行磁探搜索、无人机b盘旋
	if(area_sky_informations.area_number == 3){
		//若当前区域个数为3且有人机第一次执行任务的区域与浮标监听区域不是同一个区域，则只剩下一个区域可分配
		//无人机B盘旋,也即是距离有人机第一次任务最近的无人机待机
		//        int remain_area_num = 0;//剩余任务数量
		//        int remain_area[2];//剩余区域序号
		if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[0],integrated_postures.integrated_posture_drone_informations[1],area_sky_informations.area_informations[0])){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 5;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].sequence_type = 14;//20260123悬停改盘旋
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;

			GeoLibDas location;
			location.longitude = integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.longitude;
			location.longitude = integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.latitude;
			if(distanceComparison(location,area_sky_informations.area_informations[1],area_sky_informations.area_informations[2])){
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[2].area_code;
			}else{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[1].area_code;
			}

			//第二、三次分配，无人机A盘旋、无人机B执行任务监听(如果区域一为有人机监听任务，则无人机B继续盘旋，无人机A在第三个区域执行磁探搜索任务)
            		if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].target_number == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].target_number){
            			//第二次任务分配，当有人机第一次任务位置在监测上
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 5;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 14;//20260123悬停改盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].target_number = area_sky_informations.area_informations[2].area_code;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
            			//第三次分配任务，无人机B盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].sequence_type = 14;//20260123悬停改盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].sequence_type = 1;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].target_number = area_sky_informations.area_informations[1].area_code;

            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].presence_of_buoy_array = 1;

            		}else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].target_number == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].target_number){
            			//第二次任务分配，当有人机第二次任务位置在监测上
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 5;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 1;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].target_number = area_sky_informations.area_informations[2].area_code;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;

            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_buoy_array = 1;
            			//第三次分配任务，无人机B盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].sequence_type = 14;//20260123悬停改盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].sequence_type = 1;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].target_number = area_sky_informations.area_informations[0].area_code;

            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].presence_of_buoy_array = 1;
            		}else {
            			//第二次任务分配，当有人机第三次任务位置在监测上
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 14;//20260123悬停改盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 1;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;

            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_buoy_array = 1;
            			//第三次分配任务，无人机B盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].sequence_type = 14;//20260123悬停改盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].sequence_type = 1;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].target_number = area_sky_informations.area_informations[1].area_code;

            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].presence_of_buoy_array = 1;
            		}

		}else{
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].sequence_type = 5;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 14;//20260123悬停改盘旋

			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;

			GeoLibDas location;
			location.longitude = integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.longitude;
			location.longitude = integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.latitude;
			if(distanceComparison(location,area_sky_informations.area_informations[1],area_sky_informations.area_informations[2])){
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = area_sky_informations.area_informations[2].area_code;
			}else{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = area_sky_informations.area_informations[1].area_code;
			}

			//第二、三次分配，无人机A盘旋、无人机B执行任务监听(如果区域一为有人机监听任务，则无人机B继续盘旋，无人机A在第三个区域执行磁探搜索任务)
            		if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].target_number == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].target_number){
            			//第二次任务分配，当有人机第一次任务位置在监测上
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 5;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 14;//20260123悬停改盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].target_number = area_sky_informations.area_informations[2].area_code;

            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;

            			//第三次分配任务，无人机B盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].sequence_type = 14;//20260123悬停改盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].sequence_type = 1;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].target_number = area_sky_informations.area_informations[1].area_code;

            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].presence_of_buoy_array = 1;

            		}else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].target_number == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].target_number){
            			//第二次任务分配，当有人机第二次任务位置在监测上
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 5;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 1;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].target_number = area_sky_informations.area_informations[2].area_code;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;

            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_buoy_array = 1;

            			//第三次分配任务，无人机B盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].sequence_type = 14;//20260123悬停改盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].sequence_type = 1;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].target_number = area_sky_informations.area_informations[0].area_code;

            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].presence_of_buoy_array = 1;

            		}else {
            			//第二次任务分配，当有人机第三次任务位置在监测上
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 14;//20260123悬停改盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 1;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;

            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_buoy_array = 1;

            			//第三次分配任务，无人机B盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].sequence_type = 14;//20260123悬停改盘旋
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].sequence_type = 1;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].target_number = area_sky_informations.area_informations[1].area_code;

            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[2].presence_of_fixed_point_arrays = 0;
            			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[2].presence_of_buoy_array = 1;
            		}

		}
	}else if(area_sky_informations.area_number == 2){
		//第一次任务分配，无人机盘旋，如果有人机任务监听在第一个位置，则无人机A也盘旋
		//无人机B盘旋,也即是距离有人机第一次任务最近的无人机盘旋
		if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[0],integrated_postures.integrated_posture_drone_informations[1],area_sky_informations.area_informations[0])){
			//第一次任务分配
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 5;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[1].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].sequence_type = 14;//20260123悬停改盘旋

			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;

			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].target_number == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].target_number){
				//                information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[1].area_code;
				//第二次任务分配
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 14;//20260123悬停改盘旋
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 14;//20260123悬停改盘旋

				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;

			}else{
				//                information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 12;
				//第二次任务分配
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 14;//20260123悬停改盘旋
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 1;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;

				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_buoy_array = 1;
			}

		}else{
			//第一次任务分配
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].sequence_type = 5;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = area_sky_informations.area_informations[1].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 14;//20260123悬停改盘旋

			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;

			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].target_number == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[subtasks].target_number){
				//                information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[1].area_code;
				//第二次任务分配
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 14;//20260123悬停改盘旋
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 14;//20260123悬停改盘旋

				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;

			}else{
				//                information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 12;
				//第二次任务分配
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].sequence_type = 14;//20260123悬停改盘旋
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].sequence_type = 1;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;

				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].presence_of_fixed_point_arrays = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].presence_of_buoy_array = 1;
			}
		}
	}else {//只有一个任务区就全都盘旋
		if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[0],integrated_postures.integrated_posture_drone_informations[1],area_sky_informations.area_informations[0])){
			//第一次任务分配
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 14;//20260123悬停改盘旋
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].sequence_type = 14;//20260123悬停改盘旋

			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;

			//第二次任务分配
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].sequence_type = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;

			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].presence_of_buoy_array = 1;

		}else{
			//第一次任务分配
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].sequence_type = 14;//20260123悬停改盘旋
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 14;//20260123悬停改盘旋

			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;

			//第二次任务分配
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;

			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].presence_of_fixed_point_arrays = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].presence_of_buoy_array = 1;
		}
		//        information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].sequence_type = 12;
		//        information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 12;
	}

	//方案4：如果无人机上一阶段任务为浮标监听，则增加一个监听任务（保持与有人机任务阶段数一致）260201
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		int last = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks - 1;//新加的末尾阶段
		int prev = last - 1;//之前的最后一个阶段，现在的倒数第二个阶段
		if(prev >= 0 && information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[prev].sequence_type == 1){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].subtask_ID_number = (unsigned)(last + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].sequence_type = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].type_of_mission_point_area
					= information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[prev].type_of_mission_point_area;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].target_number
					= information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[prev].target_number;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].completion_time_valid_Bits
					= information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[prev].completion_time_valid_Bits;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].task_height_effective_position
					= information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[prev].task_height_effective_position;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].task_completion_time
					= information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[prev].task_completion_time;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].mission_height
					= information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[prev].mission_height;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].availability_of_routes
					= information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[prev].availability_of_routes;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].presence_of_buoy_array
					= information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[prev].presence_of_buoy_array;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].presence_of_fixed_point_arrays
					= information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[prev].presence_of_fixed_point_arrays;
		}else{
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].subtask_ID_number = (unsigned)(last + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].sequence_type = 14;//20260201
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].target_number = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].availability_of_routes = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[last].presence_of_fixed_point_arrays = 0;
		}
	}
	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}

	//方案时长航程计算
	uav_time_range_calc();
}


/*
 * 战术五
 * */
void fifthStrategy(){
	fifthStrategy_missionDistributeInformation();
}

//战术五---任务结果分配信息
void fifthStrategy_missionDistributeInformation(){
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定
	//协同搜潜方式---4.9 战术战法选择

	//方案总时间---待定
	//任务平台个数---3.2 无人机个数2 + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = 3;

	/*
	 * 有人机任务信息分配结果
	 * */
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_model = 1;//平台型号--有人机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID

	//平台任务时---待定
	//平台子任务个数---任务区/应召点个数
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks
	= global_mission_planning_commandss.area_point_num;
	mannedAreaSorted();//为区域池排序,然后开始分配浮标

	//任务序列信息
	for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
		// information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 2;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
		//information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 1;
	}
	int remain_buyo_number = 25;//浮标数量
	int use_buyo_number = 0;
	//当前有人机位置
	GeoLibDas manned_position;
	manned_position.longitude = integrated_postures.longitude;
	manned_position.latitude = integrated_postures.latitude;
	OutBuoyLayoutAuto outBuoyLayoutAuto;//接收浮标布阵信息

	for(int i = 0;i < area_sky_informations.area_number;i++){
		outBuoyLayoutAuto = buyoGeneration(area_sky_informations.area_informations[i],manned_position);
		use_buyo_number = outBuoyLayoutAuto.sum;//接收总浮标数量
		//use_buyo_number = used_buyo_number(area_sky_informations.area_informations[i]);
		if(remain_buyo_number >= use_buyo_number){//当前浮标数量尚且够用，就执行浮标布放任务
			// qDebug()<<"执行浮标布放任务";
			remain_buyo_number -= use_buyo_number;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 3;//任务类型为浮标布阵
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i].area_code;//目标区域为当前位置的区域
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 1;
			//更新有人机位置信息为上一次浮标布阵的位置
			manned_position.longitude = outBuoyLayoutAuto.buoyA[use_buyo_number - 1].geo.longitude;
			manned_position.latitude = outBuoyLayoutAuto.buoyA[use_buyo_number - 1].geo.latitude;
		}else{//若不够用，则执行吊声搜索
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 2;//任务类型为吊声定测
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i].area_code;//目标区域为当前位置的区域
		}
	}
	//    //分配有人机返航任务
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].target_number = 0;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].sequence_type = 11;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].presence_of_fixed_point_arrays = 0;

	/*
	 * 无人机任务分配
	 * */
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数 ＋ 返航
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= global_mission_planning_commandss.area_point_num;

		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 2;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[i -1 ].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 1;
		}

	}
	//    //分配无人机返航任务
	//    for(int i = 1;i <= drone_state_informations.drone_number;i++){
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = 0;
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 11;
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].presence_of_fixed_point_arrays = 0;
	//
	//    }

	//哪只有人机第一个位置区域近就由哪只为b机，也就是执行浮标监听任务；另一只无人机为a机，执行通信中继任务
	if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[0],integrated_postures.integrated_posture_drone_informations[1],area_sky_informations.area_informations[0])){
		//第0个无人机比较远为a机，第1个无人机比较近为b机
		for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1;i++){
			//无人机b任务分配结果
			if(i == 0){//第一次分配，无人机b待机；判断第一次任务类型是否为吊声搜索任务，若是则无人机a也待机
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[i].sequence_type = 12;

			}else{//第二次任务分配（最多两个任务区）
				if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i - 1].sequence_type == 3){
					//当有人机上阶段执行的任务类型为浮标布阵时候，无人机b该阶段执行浮标监听任务，执行区域为有人机上阶段浮标布阵区域
					information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[i].sequence_type = 1;
					information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i - 1].area_code;
				}
			}


			//无人机a任务分配结果
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type == 2){
				//当前有人机任务为吊声搜索时，无人机a执行通信中继
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].sequence_type = 4;
			}else{
				//否则待机
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].sequence_type = 12;
			}
		}
	}else {
		//第0个无人机比较远为b机，第1个无人机比较近为a机
		for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
			//无人机b任务分配结果
			if(i == 0){//第一次分配，无人机b待机；判断第一次任务类型是否为吊声搜索任务，若是则无人机a也待机
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].sequence_type = 12;

			}else{//第二次任务分配（最多两个任务区）
				if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i - 1].sequence_type == 3){
					//当有人机上阶段执行的任务类型为浮标布阵时候，无人机b该阶段执行浮标监听任务，执行区域为有人机上阶段浮标布阵区域
					information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].sequence_type = 1;
					information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i - 1].area_code;
				}
			}


			//无人机a任务分配结果
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type == 2){
				//当前有人机任务为吊声搜索时，无人机a执行通信中继
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[i].sequence_type = 4;
			}else{
				//否则待机
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[i].sequence_type = 12;
			}
		}
	}

	//分配无人机返航任务
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 11;

	}
}
int used_buyo_number(area_information area){//判断当前区域需要使用的浮标数量
	int buyo_number = 15;
	return buyo_number;
	InBuoyLayoutAuto inBuoyLayoutAuto;//中间变量,自动布放输入信息
	OutBuoyLayoutAuto outBuoyLayoutAuto;//中间变量,自动布放接收信息
	if(area.area_type == 1){//当区域类型为圆形时候
		inBuoyLayoutAuto.typeTask = TYPE_BUOY_ROUND_AREA;
	}else if(area.area_type == 2){
		inBuoyLayoutAuto.typeTask = TYPE_BUOY_RECT_AREA;
	}else if(area.area_type == 0){
		inBuoyLayoutAuto.typeTask = TYPE_BUOY_TARGET_POINT;
	}
}


/*
 * 战术六
 * */
void sixthStrategy(){
	sixthStrategy_missionDistributeInformation();
}

//战术六---任务结果分配信息
void sixthStrategy_missionDistributeInformation(){
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定
	//协同搜潜方式---4.9 战术战法选择

	//方案总时间---待定
	//任务平台个数---3.2 无人机个数n + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number + 1;


	/*
	 * 有人机任务信息分配结果
	 * */
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_model = 1;//平台型号--有人机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID

	//平台任务时---待定
	//平台子任务个数---任务区/应召点个数
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks
	= global_mission_planning_commandss.area_point_num;
	mannedAreaSorted();//组织任务区顺序
	//任务序列信息
	for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 2;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 1;
	}
	//    //分配有人机返航任务
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = 0;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 11;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[global_mission_planning_commandss.area_point_num].presence_of_fixed_point_arrays = 0;


	/*
	 * 无人机任务信息分配结果
	 * */
	int uav_task_num[4] = {0,0,0,0};
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= global_mission_planning_commandss.area_point_num;
		uav_task_num[i-1] = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;

		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 5;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[i -1 ].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 0;
		}

	}
	//    //分配无人机返航任务
	//    for(int i = 1;i <= drone_state_informations.drone_number;i++){
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = 0;
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 11;
	//
	//    }


	//确定第一轮任务时无人机的任务分配
	UAVAreaDistribute();
	//将后续任务全都赋值为第一个任务（主要原因是如果后面有人机未经过当前无人机的位置，则无人机不应该再移动）
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		for(int j = 1;j < global_mission_planning_commandss.area_point_num;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].target_number;

		}
	}
	//确定余下的任务分配
	for(int i = 1;i < global_mission_planning_commandss.area_point_num;i++){//外层为有人机后续任务编号
		for(int j = 1;j <= drone_state_informations.drone_number;j++){//如果无人机的第一个任务的任务区与有人机后续任务区的编号相同,则把无人机后续的任务区全部赋值成有人机当前任务区编号
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[j].task_sequence_informations[0].target_number == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number){
				for(int k = i;k < global_mission_planning_commandss.area_point_num;k++){
					information_on_the_results_of_taskings.formation_synergy_mission_programs[j].task_sequence_informations[k].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i - 1].target_number;
				}
			}
		}
	}

	//赋值待机
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks - 1;j++){
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number == 0){
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 14;
			}
		}
	}

	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}
	//方案时长航程计算
	uav_time_range_calc();
}


/*
 * 战术七
 * */
void seventhStrategy(){
	seventhStrategy__missionDistributeInformation();
}

//战术七---任务结果分配信息
void seventhStrategy__missionDistributeInformation(){
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定
	//协同搜潜方式---4.9 战术战法选择

	//方案总时间---待定
	//任务平台个数---3.2 无人机个数n + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number + 1;


	/*
	 * 有人机任务信息分配结果
	 * */
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_model = 1;//平台型号--有人机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID

	//平台任务时---待定
	mannedAreaSorted();//组织任务区顺序
	if(drone_state_informations.drone_number > 1){//若无人机个数大于1,则有人机正常分配
		//平台子任务个数---任务区/应召点个数 + 返航任务
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks
		= global_mission_planning_commandss.area_point_num;

		//任务序列信息
		for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 0;
		}
		//        //分配有人机返航任务
		//        information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = 0;
		//        information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 11;
		//        information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[global_mission_planning_commandss.area_point_num].presence_of_buoy_array = 0;

	}else{//若无人机数量小于1，则有人机在执行浮标布放任务之后需要进行一个阶段的浮标监听，因此任务数量是原任务数量的两倍
		//平台子任务个数---任务区/应召点个数 * 2 + 返航任务
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks
		= global_mission_planning_commandss.area_point_num * 2;

		//任务序列信息
		for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 0;

			if(i % 2 == 0 ){//如果是偶数，则证明是浮标布放任务
				information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 3;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i/2].area_code;//（除以二的原因是布阵和监听在同一个区域，整数除法可以直接截断）

			}else{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 1;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 1;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i/2].area_code;//（除以二的原因是布阵和监听在同一个区域，整数除法可以直接截断）

			}
		}

	}


	/*
	 * 无人机任务分配
	 * */
	int uav_task_num[4] = {0,0,0,0};
	for(unsigned int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数 ＋ 返航
		if(drone_state_informations.drone_number == 1 && global_mission_planning_commandss.area_point_num < 4)
		{
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
			= global_mission_planning_commandss.area_point_num * 2;
		}
		else
		{
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
			= global_mission_planning_commandss.area_point_num;
		}
		uav_task_num[i-1] = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;
		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 2;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[i -1 ].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 1;
		}

	}

	if(drone_state_informations.drone_number == 1){//若是只有1架无人机时,无人机执行任务与有人机保持一致
		//平台子任务个数---任务区/应召点个数 * 2 ＋ 返航
		information_on_the_results_of_taskings.formation_synergy_mission_programs[1].number_of_subtasks
		= global_mission_planning_commandss.area_point_num * 2;
		for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[1].number_of_subtasks;i++){

			if(i % 2 == 0){//偶数阶段跟随有人机执行磁探搜索，奇数阶段待机
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].sequence_type = 5;//任务类型为磁探搜索
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number;//任务区域为有人机任务区域
			}else {
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].sequence_type = 14;//任务类型为待机
			}
		}
	}else{//若是两架无人机，则分a,b机执行

		//哪只无人机第一个位置区域近就由哪只为b机，也就是执行浮标监听任务；另一只无人机为a机，执行通信中继任务
		if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[0],integrated_postures.integrated_posture_drone_informations[1],area_sky_informations.area_informations[0])){
			//第0个无人机比较远为a机，第1个无人机比较近为b机

			//a机始终跟随有人机执行磁探搜索
			for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[1].number_of_subtasks;i++){

				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].sequence_type = 5;//任务类型为磁探搜索
				information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number;//任务区域为有人机任务区域
			}

			//当此时为第一阶段时候，b机待机，否则跟随有人机的前一个区域执行浮标桢收
			for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[2].number_of_subtasks;i++){

				if(i == 0){//在第１阶段待机
					information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[i].sequence_type = 14;//任务类型为待机20260128
				}else {
					information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[i].sequence_type = 1;//任务类型为浮标桢收
					information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[i].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i - 1].target_number;//任务区域为有人机任务区域
				}
			}

		}else {
			//第0个无人机比较远为b机，第1个无人机比较近为a机
			//a机始终跟随有人机执行磁探搜索
			for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[2].number_of_subtasks;i++){

				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[i].sequence_type = 5;//任务类型为磁探搜索
				information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[i].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number;//任务区域为有人机任务区域
			}

			//当此时为第一阶段时候，b机待机，否则跟随有人机的前一个区域执行浮标桢收
			for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[1].number_of_subtasks;i++){

				if(i == 0){//在第１阶段待机
					information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].sequence_type = 14;//任务类型为待机
				}else {
					information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].sequence_type = 1;//任务类型为浮标桢收
					information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[i].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i - 1].target_number;//任务区域为有人机任务区域
				}
			}

		}
	}

	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}

	//方案时长航程计算
	uav_time_range_calc();

}
/*
 * 战术八
 * 完成战术的数据组织
 * */
void eighthStrategy(){
	eighthStrategy__missionDistributeInformation();     //任务分配结果信息
}

/*
 * 3.6 任务分配结果信息
 * */
void eighthStrategy__missionDistributeInformation(){
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定
	//协同搜潜方式---4.9 战术战法选择

	//方案总时间---待定 total_program_time
	//任务平台个数---3.2 无人机个数n + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number + 1;

	/*
	 * 有人机任务信息分配结果
	 * */
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_model = 1;//平台型号--有人机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID
	//平台任务时---待定
	//平台子任务个数---任务区/应召点个数
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks
	= global_mission_planning_commandss.area_point_num;
	mannedAreaSorted();//组织任务区顺序
	//任务序列信息
	for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 2;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[i].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 1;
	}


	/*
	 * 无人机任务信息分配结果
	 * */
	int uav_task_num[4] = {0,0,0,0};
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数 + 返航
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= global_mission_planning_commandss.area_point_num;
		uav_task_num[i-1] = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;

		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 5;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[i -1 ].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 0;
		}

	}
	//确定第一轮任务时无人机的任务分配
	UAVAreaDistribute();
	//将后续任务全都赋值为第一个任务（主要原因是如果后面有人机未经过当前无人机的位置，则无人机不应该再移动）
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		for(int j = 1;j < global_mission_planning_commandss.area_point_num;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].target_number;

		}
	}
	//确定余下的任务分配
	for(int i = 1;i < global_mission_planning_commandss.area_point_num;i++){//外层为有人机后续任务编号
		for(int j = 1;j <= drone_state_informations.drone_number;j++){//如果无人机的第一个任务的任务区与有人机后续任务区的编号相同,则把无人机后续的任务区全部赋值成有人机当前任务区编号
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[j].task_sequence_informations[0].target_number == information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number){
				for(int k = i;k < global_mission_planning_commandss.area_point_num;k++){
					information_on_the_results_of_taskings.formation_synergy_mission_programs[j].task_sequence_informations[k].target_number = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i - 1].target_number;
				}
			}
		}
	}

	//赋值待机
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks - 1;j++){
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number == 0){
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 0;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 14;
			}
		}
	}


	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}

	//方案时长航程计算
	uav_time_range_calc();

}

static area_information build_area_from_region_info(const region_info *region, unsigned int area_code)
{
	area_information area;
	memset(&area, 0, sizeof(area_information));
	if(region == NULL) {
		return area;
	}

	area.area_code = area_code;
	area.area_type = region->reg_type;
	area.area_source = region->reg_sour;
	area.area_shape = region->reg_shape;
	area.area_platform_num = 0;
	area.drone_numbe = region->kongyu_belong_to_uav_id;
	area.upper_height_limit_valid_bit = region->reg_top_of_hei_valid;
	area.lower_height_limit_valid_bit = region->reg_down_of_hei_valid;
	area.upper_height_limit = region->top_of_hei;
	area.lower_height_limit = region->down_of_hei;

	if(region->reg_shape == 1) {
		area.cycles.longitude = region->reg_circle.center_lon_lat.longitude;
		area.cycles.latitude = region->reg_circle.center_lon_lat.latitude;
		area.cycles.radius = region->reg_circle.radious;
	} else if(region->reg_shape == 2) {
		unsigned short point_num = region->reg_ploygen.point_num;
		if(point_num > 10) {
			point_num = 10;
		}
		area.polygonals.point_number = point_num;
		for(int i = 0; i < point_num; i++) {
			area.polygonals.point_coordinates[i].longitude = region->reg_ploygen.points_lon_lat[i].longitude;
			area.polygonals.point_coordinates[i].latitude = region->reg_ploygen.points_lon_lat[i].latitude;
		}
	}

	return area;
}

static GeoLibDas move_geo_by_bearing_km(GeoLibDas origin, double bearing_deg, double distance_km)
{
	GeoLibDas output = origin;
	double bearing_rad = bearing_deg * M_PI / 180.0;
	double lat1 = origin.latitude * M_PI / 180.0;
	double lon1 = origin.longitude * M_PI / 180.0;
	double angular_distance = distance_km / 6371.0;

	double sin_lat1 = sin(lat1);
	double cos_lat1 = cos(lat1);
	double sin_ad = sin(angular_distance);
	double cos_ad = cos(angular_distance);

	double lat2 = asin(sin_lat1 * cos_ad + cos_lat1 * sin_ad * cos(bearing_rad));
	double lon2 = lon1 + atan2(sin(bearing_rad) * sin_ad * cos_lat1,
			cos_ad - sin_lat1 * sin(lat2));

	output.latitude = lat2 * 180.0 / M_PI;
	output.longitude = lon2 * 180.0 / M_PI;
	while(output.longitude > 180.0) {
		output.longitude -= 360.0;
	}
	while(output.longitude < -180.0) {
		output.longitude += 360.0;
	}
	return output;
}

static area_information build_listen_area_for_ninth_strategy(const area_information *task_area, unsigned int area_code)
{
	area_information listen_area;
	memset(&listen_area, 0, sizeof(area_information));
	if(task_area == NULL) {
		return listen_area;
	}

	listen_area.area_code = area_code;
	listen_area.area_type = task_area->area_type;
	listen_area.area_source = task_area->area_source;
	listen_area.area_shape = 2;
	listen_area.area_platform_num = task_area->area_platform_num;
	listen_area.drone_numbe = task_area->drone_numbe;
	listen_area.upper_height_limit_valid_bit = task_area->upper_height_limit_valid_bit;
	listen_area.lower_height_limit_valid_bit = task_area->lower_height_limit_valid_bit;
	listen_area.upper_height_limit = task_area->upper_height_limit;
	listen_area.lower_height_limit = task_area->lower_height_limit;
	listen_area.polygonals.point_number = 4;

	GeoLibDas center;
	double half_ew_km = 0.0;
	double half_ns_km = 0.0;

	if(task_area->area_shape == 1) {
		center.longitude = task_area->cycles.longitude;
		center.latitude = task_area->cycles.latitude;
		half_ew_km = task_area->cycles.radius;
		half_ns_km = task_area->cycles.radius;
	} else {
		double min_lat = task_area->polygonals.point_coordinates[0].latitude;
		double max_lat = task_area->polygonals.point_coordinates[0].latitude;
		double min_lon = task_area->polygonals.point_coordinates[0].longitude;
		double max_lon = task_area->polygonals.point_coordinates[0].longitude;
		for(int i = 1; i < task_area->polygonals.point_number; i++) {
			double lat = task_area->polygonals.point_coordinates[i].latitude;
			double lon = task_area->polygonals.point_coordinates[i].longitude;
			if(lat < min_lat) min_lat = lat;
			if(lat > max_lat) max_lat = lat;
			if(lon < min_lon) min_lon = lon;
			if(lon > max_lon) max_lon = lon;
		}

		center.latitude = (min_lat + max_lat) / 2.0;
		center.longitude = (min_lon + max_lon) / 2.0;
		half_ew_km = calculate_distances(center.latitude, min_lon, center.latitude, max_lon) / 2.0;
		half_ns_km = calculate_distances(min_lat, center.longitude, max_lat, center.longitude) / 2.0;
	}

	if(half_ew_km < 0.1) half_ew_km = 0.1;
	if(half_ns_km < 0.1) half_ns_km = 0.1;

	double course = calculate_angle(integrated_postures.latitude, integrated_postures.longitude,
			center.latitude, center.longitude);
	double move_bearing = 180.0;
	double edge_half_km = half_ns_km;

	// 与需求图伪代码一致：按象限取监听区偏置方向与参考边半长
	if(course >= 45.0 && course < 135.0) {
		move_bearing = 90.0;
		edge_half_km = half_ew_km;
	} else if(course >= 135.0 && course < 225.0) {
		move_bearing = 180.0;
		edge_half_km = half_ns_km;
	} else if(course >= 225.0 && course < 315.0) {
		move_bearing = 270.0;
		edge_half_km = half_ew_km;
	} else {
		move_bearing = 180.0;
		edge_half_km = half_ns_km;
	}

	const double listen_half_side_km = 2.5;
	const double safe_delta_km = 1.0;
	GeoLibDas listen_center = move_geo_by_bearing_km(center, move_bearing, edge_half_km + listen_half_side_km + safe_delta_km);
	double corner_distance_km = listen_half_side_km * sqrt(2.0);

	GeoLibDas corner_point = move_geo_by_bearing_km(listen_center, 315.0, corner_distance_km);
	listen_area.polygonals.point_coordinates[0].longitude = corner_point.longitude;
	listen_area.polygonals.point_coordinates[0].latitude = corner_point.latitude;
	corner_point = move_geo_by_bearing_km(listen_center, 45.0, corner_distance_km);
	listen_area.polygonals.point_coordinates[1].longitude = corner_point.longitude;
	listen_area.polygonals.point_coordinates[1].latitude = corner_point.latitude;
	corner_point = move_geo_by_bearing_km(listen_center, 135.0, corner_distance_km);
	listen_area.polygonals.point_coordinates[2].longitude = corner_point.longitude;
	listen_area.polygonals.point_coordinates[2].latitude = corner_point.latitude;
	corner_point = move_geo_by_bearing_km(listen_center, 225.0, corner_distance_km);
	listen_area.polygonals.point_coordinates[3].longitude = corner_point.longitude;
	listen_area.polygonals.point_coordinates[3].latitude = corner_point.latitude;

	return listen_area;
}

static void copy_area_points_to_taskarea_division(const area_information *area, SubdomainCoordinate dst[4])
{
	if(area == NULL || dst == NULL) {
		return;
	}

	if(area->area_shape == 1) {
		const double earth_r_km = 6371.0;
		double center_lat = area->cycles.latitude;
		double center_lon = area->cycles.longitude;
		double radius_km = area->cycles.radius;
		double delta_lat = (radius_km / earth_r_km) * (180.0 / M_PI);
		double lat_rad = center_lat * M_PI / 180.0;
		double cos_lat = cos(lat_rad);
		double delta_lon = 0.0;
		if(fabs(cos_lat) < 1e-9) {
			delta_lon = 180.0;
		} else {
			delta_lon = ((radius_km / earth_r_km) * (180.0 / M_PI)) / cos_lat;
		}

		double lat_min = center_lat - delta_lat;
		double lat_max = center_lat + delta_lat;
		double lon_min = center_lon - delta_lon;
		double lon_max = center_lon + delta_lon;
		if(lat_min < -90.0) lat_min = -90.0;
		if(lat_max > 90.0) lat_max = 90.0;
		lon_min = fmod(lon_min + 180.0, 360.0) - 180.0;
		lon_max = fmod(lon_max + 180.0, 360.0) - 180.0;

		double lon_a = lon_min;
		double lon_b = lon_max;
		if(lon_a > lon_b) {
			lon_a = -180.0;
			lon_b = 180.0;
		}

		// 顺序与task_area_division保持一致
		dst[0].Index_Lat = lat_min; dst[0].Index_Lon = lon_a;
		dst[1].Index_Lat = lat_min; dst[1].Index_Lon = lon_b;
		dst[2].Index_Lat = lat_max; dst[2].Index_Lon = lon_b;
		dst[3].Index_Lat = lat_max; dst[3].Index_Lon = lon_a;
		return;
	}

	int point_num = area->polygonals.point_number;
	if(point_num <= 0) {
		for(int i = 0; i < 4; i++) {
			dst[i].Index_Lon = 0.0;
			dst[i].Index_Lat = 0.0;
		}
		return;
	}

	for(int i = 0; i < 4; i++) {
		int idx = (i < point_num) ? i : (point_num - 1);
		dst[i].Index_Lon = area->polygonals.point_coordinates[idx].longitude;
		dst[i].Index_Lat = area->polygonals.point_coordinates[idx].latitude;
	}
}

static void fill_taskarea_division_for_ninth_strategy(unsigned int task_area_id)
{
	memset(&taskarea_division, 0, sizeof(BLK_CCC_OFP_005));
	unsigned short area_num = area_sky_informations.area_number;
	if(area_num > 6) {
		area_num = 6;
	}

	taskarea_division.task_are = 1;
	taskarea_division.task_are_hf2[0].Task_Are_ID = task_area_id;
	taskarea_division.task_are_hf2[0].task_are_hf_num = (unsigned char)area_num;

	for(int i = 0; i < area_num; i++) {
		taskarea_division.task_are_hf2[0].signal_FC00[i].Task_Are_ID =
				area_sky_informations.area_informations[i].area_code;
		copy_area_points_to_taskarea_division(&area_sky_informations.area_informations[i],
				taskarea_division.task_are_hf2[0].signal_FC00[i].signal_FC00);
	}
}

static int find_closest_area_index_for_uav(const GeoLibDas *uav_position, int area_limit, const int used_flags[8])
{
	if(uav_position == NULL) {
		return -1;
	}
	if(area_limit > 8) {
		area_limit = 8;
	}
	double min_dist = DBL_MAX;
	int min_index = -1;
	for(int i = 0; i < area_limit; i++) {
		if(used_flags != NULL && used_flags[i]) {
			continue;
		}
		if(area_sky_informations.area_informations[i].area_code == 0) {
			continue;
		}
		double dist = minDistance(*uav_position, area_sky_informations.area_informations[i]);
		if(dist < min_dist) {
			min_dist = dist;
			min_index = i;
		}
	}
	return min_index;
}

/*
 * 战术九
 * 有人机布阵后无人机进区搜索
 */
void ninthStrategy()
{
	ninthStrategy_missionDistributeInformation();
}

void ninthStrategy_missionDistributeInformation()
{
	information_on_the_results_of_taskings.tasking_release = 2;
	information_on_the_results_of_taskings.manual_modification = 2;
	information_on_the_results_of_taskings.emphasize_planning = 2;
	information_on_the_results_of_taskings.modification_method = 0;
	information_on_the_results_of_taskings.program_attributes = 0;

	int drone_num = drone_state_informations.drone_number;
	if(drone_num <= 0) {
		return;
	}
	if(drone_num > 4) {
		drone_num = 4;
	}
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_num + 1;

	area_information task_area;
	memset(&task_area, 0, sizeof(area_information));
	unsigned int src_task_area_id = 1;
	if(blk_dtms_ctas_001.task_reg_num > 0) {
		src_task_area_id = blk_dtms_ctas_001.region_infos[0].reg_id;
		task_area = build_area_from_region_info(&blk_dtms_ctas_001.region_infos[0], (unsigned int)(drone_num + 1));
	} else if(area_sky_informations.area_number > 0) {
		task_area = area_sky_informations.area_informations[0];
		src_task_area_id = task_area.area_code;
		task_area.area_code = (unsigned int)(drone_num + 1);
	}
	if(task_area.area_shape == 0) {
		return;
	}

	area_information stage2_areas[4];
	memset(stage2_areas, 0, sizeof(stage2_areas));
	int stage2_area_num = 0;
	if(blk_dtms_ctas_001.task_reg_num > 0) {
		region_info split_region = blk_dtms_ctas_001.region_infos[0];
		task_area_division(&split_region, drone_num);
		stage2_area_num = area_sky_informations.area_number;
		if(stage2_area_num > drone_num) {
			stage2_area_num = drone_num;
		}
		for(int i = 0; i < stage2_area_num; i++) {
			stage2_areas[i] = area_sky_informations.area_informations[i];
		}
	} else {
		for(int i = 0; i < drone_num && i < area_sky_informations.area_number; i++) {
			stage2_areas[i] = area_sky_informations.area_informations[i];
			stage2_area_num++;
		}
	}
	if(stage2_area_num <= 0) {
		stage2_area_num = drone_num;
		for(int i = 0; i < stage2_area_num; i++) {
			stage2_areas[i] = task_area;
		}
	}
	for(int i = 0; i < stage2_area_num; i++) {
		stage2_areas[i].area_code = (unsigned int)(i + 1);
	}

	task_area.area_code = (unsigned int)(stage2_area_num + 1);
	area_information listen_area = build_listen_area_for_ninth_strategy(&task_area, (unsigned int)(stage2_area_num + 2));

	memset(&area_sky_informations, 0, sizeof(area_sky_information));
	area_sky_informations.area_number = (unsigned short)(stage2_area_num + 2);
	for(int i = 0; i < stage2_area_num; i++) {
		area_sky_informations.area_informations[i] = stage2_areas[i];
	}
	area_sky_informations.area_informations[stage2_area_num] = task_area;
	area_sky_informations.area_informations[stage2_area_num + 1] = listen_area;

	global_mission_planning_commandss.area_point_num = area_sky_informations.area_number;
	for(int i = 0; i < global_mission_planning_commandss.area_point_num && i < 8; i++) {
		global_mission_planning_commandss.area_point_number[i] =
				area_sky_informations.area_informations[i].area_code;
	}
	fill_taskarea_division_for_ninth_strategy(src_task_area_id);

	// 有人机：阶段1布阵，阶段2监听
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_model = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks = 2;

	for(int stage = 0; stage < 2; stage++) {
		task_sequence_information *task = &information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[stage];
		task->subtask_ID_number = (unsigned int)(stage + 1);
		task->modify = 0;
		task->type_of_mission_point_area = 3;
		task->completion_time_valid_Bits = 1;
		task->task_height_effective_position = 1;
		task->task_completion_time = 20;
		task->mission_height = 1000;
		task->presence_of_fixed_point_arrays = 0;
	}
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].sequence_type = 3;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].target_number = task_area.area_code;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].availability_of_routes = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].presence_of_buoy_array = 1;

	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].sequence_type = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].target_number = listen_area.area_code;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].availability_of_routes = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].presence_of_buoy_array = 0;

	// 无人机：阶段1盘旋等待，阶段2进区搜索
	int area_used[8] = {0,0,0,0,0,0,0,0};
	unsigned int stage1_area_code = task_area.area_code;
	for(int i = 1; i <= drone_num; i++) {
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number =
				drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number =
				drone_state_informations.drone_specific_informations[i - 1].platform_num;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks = 2;

		GeoLibDas uav_pos;
		uav_pos.longitude = integrated_postures.integrated_posture_drone_informations[i - 1].drone_longitude_and_latitude.longitude;
		uav_pos.latitude = integrated_postures.integrated_posture_drone_informations[i - 1].drone_longitude_and_latitude.latitude;

		int area_index = find_closest_area_index_for_uav(&uav_pos, stage2_area_num, area_used);
		if(area_index < 0) {
			area_index = find_closest_area_index_for_uav(&uav_pos, stage2_area_num, NULL);
		}

		unsigned int target_area_code = area_sky_informations.area_informations[(i - 1) % stage2_area_num].area_code;
		if(area_index >= 0) {
			target_area_code = area_sky_informations.area_informations[area_index].area_code;
			area_used[area_index] = 1;
		}

		for(int stage = 0; stage < 2; stage++) {
			task_sequence_information *task = &information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[stage];
			task->subtask_ID_number = (unsigned int)(stage + 1);
			task->modify = 0;
			task->type_of_mission_point_area = 3;
			task->target_number = (stage == 0) ? stage1_area_code : target_area_code;
			task->completion_time_valid_Bits = 1;
			task->task_height_effective_position = 1;
			task->task_completion_time = 20;
			task->mission_height = 1000;
			task->presence_of_buoy_array = 0;
			task->presence_of_fixed_point_arrays = 0;
		}

		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].sequence_type = 14;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].availability_of_routes = 0;

		if(integrated_postures.integrated_posture_drone_informations[i - 1].ct == 0) {
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[1].sequence_type = 7;
		} else {
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[1].sequence_type = 5;
		}
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[1].availability_of_routes = 1;
	}

	uav_time_range_calc();
}

void yinzhaoRecommend()
{
	//战术1
	//调用战术战法1
	memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
	memset(&area_sky_informations,0 , sizeof(area_sky_information));
	yingzhao_firstStrategy_missionDistributeInformation();
	init_strategy_select(1);

	//战术2
	if((drone_state_informations.drone_number == 2) || (drone_state_informations.drone_number == 3)){//判断是否满足战术战法2的条件
		//调用战术战法2
		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
		memset(&area_sky_informations,0 , sizeof(area_sky_information));
		yingzhao_secondStrategy_missionDistributeInformation();
		init_strategy_select(2);
	}
	//战术3
	//调用战术战法3
	memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
	memset(&area_sky_informations,0 , sizeof(area_sky_information));
	yingzhao_thirdStrategy_missionDistributeInformation();
	init_strategy_select(3);

	//战术4
	//调用战术战法4
	memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
	memset(&area_sky_informations,0 , sizeof(area_sky_information));
	yingzhao_fourthStrategy_missionDistributeInformation();
	init_strategy_select(4);

	//判断当前是否存在时间最短的方案---当方案规划为3的时候发送最短时间方案，将三个发送标志位设置为1，开始发送方案
	if(minTimeStrategySelect.isMinTime && fangan_send_flag == 3){
		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
		switch(minTimeStrategySelect.strategyNo){
		case 1:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_firstStrategy_missionDistributeInformation();
			break;
		case 2:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_secondStrategy_missionDistributeInformation();
			break;
		case 3:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_thirdStrategy_missionDistributeInformation();
			break;
		case 4:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_fourthStrategy_missionDistributeInformation();
			break;
		default:
			information_on_the_results_of_taskings.program_attributes = 0;//如果出错,则此方案作废
			break;
		}
		//选中的第一个战法
		Select_NO[0] = minTimeStrategySelect.strategyNo;
		//将任务发送标志位和航线发送标志位等都设置为发送状态，也即是值设为1
		information_on_the_results_of_taskings.program_attributes = 1;
		renwu_guihua_flag = 1;
		buoy_suspended_flag = 1;
		wurenji_hangxian_flag = 1;
		memset(&minTimeStrategySelect,0,sizeof(StrategySelect));//方案时间最短
		memset(&minUAVStrategySelect,0,sizeof(StrategySelect));//方案所需无人机最少
		memset(&minMannedTask,0,sizeof(StrategySelect)); //方案有人机任务数量最少
	}

	//无人机数量最少---当方案规划为2的时候发送无人机数量最少方案，将三个发送标志位设置为1，开始发送方案
	if(minUAVStrategySelect.isMinUAV && fangan_send_flag == 2){
		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
		switch(minUAVStrategySelect.strategyNo){
		case 1:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_firstStrategy_missionDistributeInformation();
			break;
		case 2:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_secondStrategy_missionDistributeInformation();
			break;
		case 3:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_thirdStrategy_missionDistributeInformation();
			break;
		case 4:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_fourthStrategy_missionDistributeInformation();
			break;
		default:
			information_on_the_results_of_taskings.program_attributes = 0;//如果出错,则此方案作废
			break;
		}
		//选中的第二个战法
		Select_NO[1] = minUAVStrategySelect.strategyNo;
		information_on_the_results_of_taskings.program_attributes = 2;
		renwu_guihua_flag = 1;
		buoy_suspended_flag = 1;
		wurenji_hangxian_flag = 1;
		memset(&minTimeStrategySelect,0,sizeof(StrategySelect));//方案时间最短
		memset(&minUAVStrategySelect,0,sizeof(StrategySelect));//方案所需无人机最少
		memset(&minMannedTask,0,sizeof(StrategySelect)); //方案有人机任务数量最少
	}

	//有人机任务最少---当方案规划为1的时候发送有人机任务最少方案，将三个发送标志位设置为1，开始发送方案
	if(minMannedTask.isMInMannedTask && fangan_send_flag == 1){
		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
		switch(minMannedTask.strategyNo){
		case 1:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_firstStrategy_missionDistributeInformation();
			break;
		case 2:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_secondStrategy_missionDistributeInformation();
			break;
		case 3:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_thirdStrategy_missionDistributeInformation();
			break;
		case 4:
			memset(&area_sky_informations,0 , sizeof(area_sky_information));
			yingzhao_fourthStrategy_missionDistributeInformation();
			break;
		default:
			information_on_the_results_of_taskings.program_attributes = 0;//如果出错,则此方案作废
			break;
		}
		information_on_the_results_of_taskings.program_attributes = 3;
		renwu_guihua_flag = 1;
		buoy_suspended_flag = 1;
		wurenji_hangxian_flag = 1;
		memset(&minTimeStrategySelect,0,sizeof(StrategySelect));//方案时间最短
		memset(&minUAVStrategySelect,0,sizeof(StrategySelect));//方案所需无人机最少
		memset(&minMannedTask,0,sizeof(StrategySelect)); //方案有人机任务数量最少
	}

//	if(fangan_send_flag == 3)
//	{
//		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
//		memset(&area_sky_informations,0 , sizeof(area_sky_information));
//		//战法一
//		yingzhao_firstStrategy_missionDistributeInformation();
//		renwu_guihua_flag = 1;
//		buoy_suspended_flag = 1;
//		wurenji_hangxian_flag = 1;
//	}
//	else if(fangan_send_flag == 2)
//	{
//		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
//		memset(&area_sky_informations,0 , sizeof(area_sky_information));
//		if((drone_state_informations.drone_number == 2) || (drone_state_informations.drone_number == 3))
//		{
//			//战法二
//			yingzhao_secondStrategy_missionDistributeInformation();
//			renwu_guihua_flag = 1;
//			buoy_suspended_flag = 1;
//			wurenji_hangxian_flag = 1;
//		}
//		else
//		{
//			//战法四
//			yingzhao_fourthStrategy_missionDistributeInformation();
//			renwu_guihua_flag = 1;
//			buoy_suspended_flag = 1;
//			wurenji_hangxian_flag = 1;
//		}
//	}
//	else if(fangan_send_flag == 1)
//	{
//		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
//		memset(&area_sky_informations,0 , sizeof(area_sky_information));
//		//战法三
//		yingzhao_thirdStrategy_missionDistributeInformation();
//		renwu_guihua_flag = 1;
//		buoy_suspended_flag = 1;
//		wurenji_hangxian_flag = 1;
//	}
}

/*
 * 战术战法推荐模块
 * */
void strategyRecommend(){
	StrategySelect strategySelect_mid;//中间变量，计算每个战术战法的值
	//战法9固定在方案1（仅检查反潜）
	if(global_mission_planning_commandss.mission_type == 0 && fangan_send_flag == 3){
		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
		ninthStrategy();
		Select_NO[0] = 9;
		information_on_the_results_of_taskings.program_attributes = 1;
		renwu_guihua_flag = 1;
		buoy_suspended_flag = 1;
		wurenji_hangxian_flag = 1;
		memset(&minTimeStrategySelect,0,sizeof(StrategySelect));
		memset(&minUAVStrategySelect,0,sizeof(StrategySelect));
		memset(&minMannedTask,0,sizeof(StrategySelect));
		return;
	}
	//战法9已固定在方案1，方案2/3恢复原始输入态后按老逻辑比较
	if(global_mission_planning_commandss.mission_type == 0 &&
			(fangan_send_flag == 2 || fangan_send_flag == 1) &&
			Select_NO[0] == 9)
	{
		init_zhanshutuijians();
	}

	//战术1
	if(integrated_postures.drone_num >= 1 && integrated_postures.drone_num <= 4 &&
			area_sky_informations.area_number <= integrated_postures.drone_num + 1){//判断是否满足战术战法1的条件
//		//调用战术战法1
		firstStrategy();
		init_strategy_select(1);
	}
	//战术2
//	    if(integrated_postures.drone_num == 2 &&
//	            area_sky_informations.area_number <= 2){//判断是否满足战术战法2的条件
	        //调用战术战法2
//	        secondStrategy();
//	        init_strategy_select(2);
//	    }
	//战术3
	if(integrated_postures.drone_num >= 1 && integrated_postures.drone_num <= 4 &&// 一控四：修改无人机数量判断条件
			area_sky_informations.area_number <= integrated_postures.drone_num + 1){//判断是否满足战术战法3的条件
		//调用战术战法3
		thirdStrategy();
		init_strategy_select(3);
	}
	//战术4
	if(integrated_postures.drone_num == 2 &&
			area_sky_informations.area_number <= 3){//判断是否满足战术战法4的条件
		//调用战术战法4
		fourthStrategy();
		init_strategy_select(4);
	}
	//    //战术5
	//    if(integrated_postures.drone_num == 2 &&
	//            area_sky_informations.area_number <= 2){//判断是否满足战术战法5的条件
	//        //调用战术战法5
	//        fifthStrategy();
	//        init_strategy_select(5);
	//    }
	//战术6
	if(integrated_postures.drone_num >= 1 && integrated_postures.drone_num <= 4 &&
			area_sky_informations.area_number <= integrated_postures.drone_num + 1){//判断是否满足战术战法6的条件
		//调用战术战法6
		sixthStrategy();
		init_strategy_select(6);
	}
	//战术7
	if(integrated_postures.drone_num >= 1 && integrated_postures.drone_num <= 4 &&
			area_sky_informations.area_number <= integrated_postures.drone_num + 1){//判断是否满足战术战法7的条件
		//调用战术战法7
		seventhStrategy();
		init_strategy_select(7);
	}

	//战术8 20250907new
	if(integrated_postures.drone_num >= 1 && integrated_postures.drone_num <= 4 &&
			area_sky_informations.area_number <= integrated_postures.drone_num + 1){//判断是否满足战术战法8的条件
		//调用战术战法8
		eighthStrategy();
		init_strategy_select(8);
	}

	//判断当前是否存在时间最短的方案---当方案规划为3的时候发送最短时间方案，将三个发送标志位设置为1，开始发送方案
	if(minTimeStrategySelect.isMinTime && fangan_send_flag == 3){
		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
		switch(minTimeStrategySelect.strategyNo){
		case 1:
			firstStrategy();
			break;
		case 2:
			secondStrategy();
			break;
		case 3:
			thirdStrategy();
			break;
		case 4:
			fourthStrategy();
			break;
		case 5:
			fifthStrategy();
			break;
		case 6:
			sixthStrategy();
			break;
		case 7:
			seventhStrategy();
			break;
		case 8:
			eighthStrategy();
			break;
		case 9:
			ninthStrategy();
			break;
		default:
			information_on_the_results_of_taskings.program_attributes = 0;//如果出错,则此方案作废
			break;
		}
		//选中的第一个战法
		Select_NO[0] = minTimeStrategySelect.strategyNo;
		//将任务发送标志位和航线发送标志位等都设置为发送状态，也即是值设为1
		information_on_the_results_of_taskings.program_attributes = 1;
		//无人机任务区分派
		if(minTimeStrategySelect.strategyNo != 9){
			uav_dispatch();
		}

		renwu_guihua_flag = 1;
		buoy_suspended_flag = 1;
		wurenji_hangxian_flag = 1;
		memset(&minTimeStrategySelect,0,sizeof(StrategySelect));//方案时间最短
		memset(&minUAVStrategySelect,0,sizeof(StrategySelect));//方案所需无人机最少
		memset(&minMannedTask,0,sizeof(StrategySelect)); //方案有人机任务数量最少
	}

	//无人机数量最少---当方案规划为2的时候发送无人机数量最少方案，将三个发送标志位设置为1，开始发送方案
	if(minUAVStrategySelect.isMinUAV && fangan_send_flag == 2){
		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
		switch(minUAVStrategySelect.strategyNo){
		case 1:
			firstStrategy();
			break;
		case 2:
			secondStrategy();
			break;
		case 3:
			thirdStrategy();
			break;
		case 4:
			fourthStrategy();
			break;
		case 5:
			fifthStrategy();
			break;
		case 6:
			sixthStrategy();
			break;
		case 7:
			seventhStrategy();
			break;
		case 8:
			eighthStrategy();
			break;
		case 9:
			ninthStrategy();
			break;
		default:
			information_on_the_results_of_taskings.program_attributes = 0;//如果出错,则此方案作废
			break;
		}
		//选中的第二个战法
		Select_NO[1] = minUAVStrategySelect.strategyNo;
		information_on_the_results_of_taskings.program_attributes = 2;
		//无人机任务区分派
		if(minUAVStrategySelect.strategyNo != 9){
			uav_dispatch();
		}
		renwu_guihua_flag = 1;
		buoy_suspended_flag = 1;
		wurenji_hangxian_flag = 1;
		memset(&minTimeStrategySelect,0,sizeof(StrategySelect));//方案时间最短
		memset(&minUAVStrategySelect,0,sizeof(StrategySelect));//方案所需无人机最少
		memset(&minMannedTask,0,sizeof(StrategySelect)); //方案有人机任务数量最少
	}

	//有人机任务最少---当方案规划为1的时候发送有人机任务最少方案，将三个发送标志位设置为1，开始发送方案
	if(minMannedTask.isMInMannedTask && fangan_send_flag == 1){
		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
		switch(minMannedTask.strategyNo){
		case 1:
			firstStrategy();
			break;
		case 2:
			secondStrategy();
			break;
		case 3:
			thirdStrategy();
			break;
		case 4:
			fourthStrategy();
			break;
		case 5:
			fifthStrategy();
			break;
		case 6:
			sixthStrategy();
			break;
		case 7:
			seventhStrategy();
			break;
		case 8:
			eighthStrategy();
			break;
		case 9:
			ninthStrategy();
			break;
		default:
			information_on_the_results_of_taskings.program_attributes = 0;//如果出错,则此方案作废
			break;
		}
		information_on_the_results_of_taskings.program_attributes = 3;
		//无人机任务区分派
		if(minMannedTask.strategyNo != 9){
			uav_dispatch();
		}
		renwu_guihua_flag = 1;
		buoy_suspended_flag = 1;
		wurenji_hangxian_flag = 1;
		memset(&minTimeStrategySelect,0,sizeof(StrategySelect));//方案时间最短
		memset(&minUAVStrategySelect,0,sizeof(StrategySelect));//方案所需无人机最少
		memset(&minMannedTask,0,sizeof(StrategySelect)); //方案有人机任务数量最少
	}

}

void init_strategy_select(int strategyNo){
	//计算有人机任务数量
	unsigned short MannedTaskNumber = 0;
	for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++) {
		if(information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type != 12){
			MannedTaskNumber++;
		}
	}

	//过滤已选择的战法
	if(Select_NO[0] == strategyNo || Select_NO[1] == strategyNo)
	{
		return;
	}

	//判断是否比当前最短时长的总时长短
	if(minTimeStrategySelect.isMinTime == false || information_on_the_results_of_taskings.total_program_time < minTimeStrategySelect.totalTime){
		if(minTimeStrategySelect.isMinTime != false){
			isOtherStrategySelect(minTimeStrategySelect);
		}
		minTimeStrategySelect.program_number = information_on_the_results_of_taskings.program_number;
		minTimeStrategySelect.strategyNo = strategyNo;
		minTimeStrategySelect.totalTime = information_on_the_results_of_taskings.total_program_time;
		minTimeStrategySelect.UAVnumber = information_on_the_results_of_taskings.number_of_mission_platforms - 1;
		//有人机任务数量
		minTimeStrategySelect.mannedTaskNumber = MannedTaskNumber;
		//是否时长最短
		minTimeStrategySelect.isMinTime = true;
		minTimeStrategySelect.isMinUAV = false;
		minTimeStrategySelect.isMInMannedTask = false;
//		return;
	}else if(information_on_the_results_of_taskings.total_program_time == minTimeStrategySelect.totalTime && MannedTaskNumber < minTimeStrategySelect.mannedTaskNumber){
		if(minTimeStrategySelect.isMinTime != false){
			isOtherStrategySelect(minTimeStrategySelect);
		}
		minTimeStrategySelect.program_number = information_on_the_results_of_taskings.program_number;
		minTimeStrategySelect.strategyNo = strategyNo;
		minTimeStrategySelect.totalTime = information_on_the_results_of_taskings.total_program_time;
		minTimeStrategySelect.UAVnumber = information_on_the_results_of_taskings.number_of_mission_platforms - 1;
		//有人机任务数量
		minTimeStrategySelect.mannedTaskNumber = MannedTaskNumber;
		//是否时长最短
		minTimeStrategySelect.isMinTime = true;
		minTimeStrategySelect.isMinUAV = false;
		minTimeStrategySelect.isMInMannedTask = false;
//		return;
	}


	//最少有人机任务
	if(minMannedTask.isMInMannedTask == false || MannedTaskNumber < minMannedTask.mannedTaskNumber){//如果当前无,则直接赋值
		if(minMannedTask.isMInMannedTask != false){
			isOtherStrategySelect(minMannedTask);
		}
		//若是有人机子任务个数相同，则判总时长
		minMannedTask.program_number = information_on_the_results_of_taskings.program_number;
		minMannedTask.strategyNo = strategyNo;
		minMannedTask.totalTime = information_on_the_results_of_taskings.total_program_time;
		minMannedTask.UAVnumber = information_on_the_results_of_taskings.number_of_mission_platforms - 1;
		//有人机任务数量
		minMannedTask.mannedTaskNumber = MannedTaskNumber;
		//是否时长最短
		minMannedTask.isMinTime = false;
		minMannedTask.isMinUAV = false;
		minMannedTask.isMInMannedTask = true;
//		return;
	}else if(MannedTaskNumber == minMannedTask.mannedTaskNumber && information_on_the_results_of_taskings.total_program_time < minMannedTask.totalTime){
		if(minMannedTask.isMInMannedTask != false){
			isOtherStrategySelect(minMannedTask);
		}
		//若是有人机子任务个数相同，则判总时长
		minMannedTask.program_number = information_on_the_results_of_taskings.program_number;
		minMannedTask.strategyNo = strategyNo;
		minMannedTask.totalTime = information_on_the_results_of_taskings.total_program_time;
		minMannedTask.UAVnumber = information_on_the_results_of_taskings.number_of_mission_platforms - 1;
		//有人机任务数量
		minMannedTask.mannedTaskNumber = MannedTaskNumber;
		//是否时长最短
		minMannedTask.isMinTime = false;
		minMannedTask.isMinUAV = false;
		minMannedTask.isMInMannedTask = true;
//		return;
	}



	//最少无人机数量
	if(minUAVStrategySelect.isMinUAV == false || information_on_the_results_of_taskings.number_of_mission_platforms - 1 < minUAVStrategySelect.UAVnumber){
		if(minUAVStrategySelect.isMinUAV != false){
			isOtherStrategySelect(minUAVStrategySelect);
		}
		//判断是否比当前无人机个数的无人机少
		minUAVStrategySelect.program_number = information_on_the_results_of_taskings.program_number;
		minUAVStrategySelect.strategyNo = strategyNo;
		minUAVStrategySelect.totalTime = information_on_the_results_of_taskings.total_program_time;
		minUAVStrategySelect.UAVnumber = information_on_the_results_of_taskings.number_of_mission_platforms - 1;
		//有人机任务数量
		minUAVStrategySelect.mannedTaskNumber = MannedTaskNumber;
		//是否时长最短
		minUAVStrategySelect.isMinTime = false;
		minUAVStrategySelect.isMinUAV = true;
		minUAVStrategySelect.isMInMannedTask = false;
//		return;
	}else if(information_on_the_results_of_taskings.number_of_mission_platforms - 1 == minUAVStrategySelect.UAVnumber && information_on_the_results_of_taskings.total_program_time < minUAVStrategySelect.totalTime){
		if(minUAVStrategySelect.isMinUAV != false){
			isOtherStrategySelect(minUAVStrategySelect);
		}
		//判断是否比当前无人机个数的无人机少
		minUAVStrategySelect.program_number = information_on_the_results_of_taskings.program_number;
		minUAVStrategySelect.strategyNo = strategyNo;
		minUAVStrategySelect.totalTime = information_on_the_results_of_taskings.total_program_time;
		minUAVStrategySelect.UAVnumber = information_on_the_results_of_taskings.number_of_mission_platforms - 1;
		//有人机任务数量
		minUAVStrategySelect.mannedTaskNumber = MannedTaskNumber;
		//是否时长最短
		minUAVStrategySelect.isMinTime = false;
		minUAVStrategySelect.isMinUAV = true;
		minUAVStrategySelect.isMInMannedTask = false;
//		return;
	}

	//判断是否比当前其他时长的总时长短
	if(otherStrategySelect.isMinTime == false || information_on_the_results_of_taskings.total_program_time < otherStrategySelect.totalTime){
		otherStrategySelect.program_number = information_on_the_results_of_taskings.program_number;
		otherStrategySelect.strategyNo = strategyNo;
		otherStrategySelect.totalTime = information_on_the_results_of_taskings.total_program_time;
		otherStrategySelect.UAVnumber = information_on_the_results_of_taskings.number_of_mission_platforms - 1;
		//有人机任务数量
		otherStrategySelect.mannedTaskNumber = MannedTaskNumber;
		//是否时长最短
		otherStrategySelect.isMinTime = true;
		otherStrategySelect.isMinUAV = false;
		otherStrategySelect.isMInMannedTask = false;
		return;
	}else if(information_on_the_results_of_taskings.total_program_time == otherStrategySelect.totalTime && MannedTaskNumber < otherStrategySelect.mannedTaskNumber){
		otherStrategySelect.program_number = information_on_the_results_of_taskings.program_number;
		otherStrategySelect.strategyNo = strategyNo;
		otherStrategySelect.totalTime = information_on_the_results_of_taskings.total_program_time;
		otherStrategySelect.UAVnumber = information_on_the_results_of_taskings.number_of_mission_platforms - 1;
		//有人机任务数量
		otherStrategySelect.mannedTaskNumber = MannedTaskNumber;
		//是否时长最短
		otherStrategySelect.isMinTime = true;
		otherStrategySelect.isMinUAV = false;
		otherStrategySelect.isMInMannedTask = false;
		return;
	}

}

void isOtherStrategySelect(StrategySelect strategy){
	//判断是否比当前其他时长的总时长短
	if(otherStrategySelect.isMinTime == false || strategy.totalTime < otherStrategySelect.totalTime){
		otherStrategySelect.program_number = strategy.program_number;
		otherStrategySelect.strategyNo = strategy.strategyNo;
		otherStrategySelect.totalTime = strategy.totalTime;
		otherStrategySelect.UAVnumber = strategy.UAVnumber;
		//有人机任务数量
		otherStrategySelect.mannedTaskNumber = strategy.mannedTaskNumber;
		//是否时长最短
		otherStrategySelect.isMinTime = true;
		otherStrategySelect.isMinUAV = false;
		otherStrategySelect.isMInMannedTask = false;
		return;
	}else if(strategy.totalTime == otherStrategySelect.totalTime && strategy.mannedTaskNumber < otherStrategySelect.mannedTaskNumber){
		otherStrategySelect.program_number = strategy.program_number;
		otherStrategySelect.strategyNo = strategy.strategyNo;
		otherStrategySelect.totalTime = strategy.totalTime;
		otherStrategySelect.UAVnumber = strategy.UAVnumber;
		//有人机任务数量
		otherStrategySelect.mannedTaskNumber = strategy.mannedTaskNumber;
		//是否时长最短
		otherStrategySelect.isMinTime = true;
		otherStrategySelect.isMinUAV = false;
		otherStrategySelect.isMInMannedTask = false;
	}
}

void createTaskFile()
{

	//    for(int i = 0 ; i < 3 ; i ++)
	//    {
	//        char route[100] = {0};
	//        sprintf(route,"D:/hzx/XTZK_F/DTMS/taskFile_%d.dat",&i);
	//        FILE *fp = fopen(route,"wb+");//二进制写入，存在则覆盖
	//        if(!fp)
	//        {
	//            printf("file open error.\n");
	//            return;
	//        }
	//        size_t written;
	//        if(i < 2)
	//        {
	//            int block_num = i;
	//            written = fwrite(&block_num, sizeof(int), 1, fp);
	//            written = fwrite((char*)(&load_file) + (i * 65530), 65530, 1, fp);
	//        }
	//        else if(i == 2)
	//        {
	//            int block_num = i;
	//            written = fwrite(&block_num, sizeof(int), 1, fp);
	//            written = fwrite((char*)(&load_file) + (i * 65530), sizeof(FILE_DATA)-(i * 65530), 1, fp);
	//        }
	//
	//        if(written !=1)
	//        {
	//            printf("write error.\n");
	//            fclose(fp);
	//            return;
	//        }
	//        else if(written == 1)
	//        {
	//            printf("write success\n");
	//            fclose(fp);
	//        }
	//    }
}
void init_blk_ofp_ccc_302(int stage)
{
	//通用航路赋值
	CTAS_DTMS_data_mannedRoute.common_carrier_routes.program_number = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].subtask_ID_number =
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[stage].subtask_ID_number;
	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoints_number = blk_ofp_ccc_302.status_info.RouteTotalNum;

	for(int j=0;j<blk_ofp_ccc_302.status_info.RouteTotalNum;j++){
		CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoint_informations[j].longitude = blk_ofp_ccc_302.RoutePointData[j].BuoyPlanPointLat;
		CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoint_informations[j].latitude = blk_ofp_ccc_302.RoutePointData[j].BuoyPlanPointLon;
		CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoint_informations[j].height = blk_ofp_ccc_302.RoutePointData[j].Altitude;
	}
}
void init_blk_ofp_ccc_402(int stage)
{
	//通用航路赋值
	CTAS_DTMS_data_mannedRoute.common_carrier_routes.program_number = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].subtask_ID_number =
			information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[stage].subtask_ID_number;
	if(blk_ofp_ccc_402.sonar_plan_simbol_1.PlanWay == 1)/*规划方式  0=N/A;1=自动规划;2=图形规划;3=自定义;4=定测点概率圆推算;*/
	{
		//自动规划
		CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoints_number = blk_ofp_ccc_402.AutoPlanInformation.DetectPointNum;
		for(unsigned int j=0;j<blk_ofp_ccc_402.AutoPlanInformation.DetectPointNum;j++){
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoint_informations[j].longitude = blk_ofp_ccc_402.AutoPlanInformation.DetectPointPosition[j].DetectPointLon;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoint_informations[j].latitude = blk_ofp_ccc_402.AutoPlanInformation.DetectPointPosition[j].DetectPointLat;
		}
	}
	else if(blk_ofp_ccc_402.sonar_plan_simbol_1.PlanWay == 2)
	{
		//图形规划
		CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoints_number = blk_ofp_ccc_402.FigurePlanInformation.DetectPointNumq;
		for(unsigned int j=0;j<blk_ofp_ccc_402.FigurePlanInformation.DetectPointNumq;j++){
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoint_informations[j].longitude = blk_ofp_ccc_402.FigurePlanInformation.DetectPointPosition1[j].DetectPointLon;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoint_informations[j].latitude = blk_ofp_ccc_402.FigurePlanInformation.DetectPointPosition1[j].DetectPointLat;
		}
	}
	else if(blk_ofp_ccc_402.sonar_plan_simbol_1.PlanWay == 3)
	{
		//自定义
		CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoints_number = blk_ofp_ccc_402.CustomizePlanInformation.DetectPointNum;
		for(unsigned int j=0;j<blk_ofp_ccc_402.CustomizePlanInformation.DetectPointNum;j++){
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoint_informations[j].longitude = blk_ofp_ccc_402.CustomizePlanInformation.DetectPointPosition[j].DetectPointLon;
			CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoint_informations[j].latitude = blk_ofp_ccc_402.CustomizePlanInformation.DetectPointPosition[j].DetectPointLat;
		}
	}

}
/*
 * 任务规划及航线生成模块 DTMS-CTAS CTAS-DTMS
 * 接收来自动态任务管理软件的信息
 * 规划相应的任务分配、生成对应有无人机航线
 * 将生成的规划结果及航线结果发送
 * */
void formulate_moduel()
{
	//全局规划
	message_size = 4096;
	Receive_Message(DDSTables.BLK_DTMS_CTAS_001.niConnectionId, 0, &transaction_id, &blk_dtms_ctas_001, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0 ) {
		//目前仿真使用的是task_type == 1的分支。
		if(blk_dtms_ctas_001.task_type == 0) { //检查反潜
			//1.将收到的战术战法指令赋值给规划所需使用的任务区、有无人机等数据
			init_zhanshutuijians();
			//方案规划初始化为3，开始发送三个方案
			fangan_send_flag = 3;
			memset(&minTimeStrategySelect,0,sizeof(StrategySelect));//方案时间最短
			memset(&minUAVStrategySelect,0,sizeof(StrategySelect));//方案所需无人机最少
			memset(&minMannedTask,0,sizeof(StrategySelect)); //方案有人机任务数量最少
			//战法选择清空
			Select_NO[0] = 0;
			Select_NO[1] = 0;
		}else if(blk_dtms_ctas_001.task_type == 1){//应召反潜
			//1.将收到的战术战法指令赋值给规划所需使用的任务区、有无人机等数据
			init_zhanshutuijians();
			//方案规划初始化为3，开始发送三个方案
			fangan_send_flag = 3;
			memset(&minTimeStrategySelect,0,sizeof(StrategySelect));//方案时间最短
			memset(&minUAVStrategySelect,0,sizeof(StrategySelect));//方案所需无人机最少
			memset(&minMannedTask,0,sizeof(StrategySelect)); //方案有人机任务数量最少
			//战法选择清空
			Select_NO[0] = 0;
			Select_NO[1] = 0;
		}
	}

	/*
	 * CTAS-DTMS　发送部分
	 * 判断发送标志位，如果需要发送则发送
	 * */
	if(fangan_send_flag > 0)
	{
		char use_only_ninth_strategy = (global_mission_planning_commandss.tactical_warfare_options == 9);
		if(blk_dtms_ctas_001.task_type == 0)
		{
			if(fangan_send_flag == 1 && !use_only_ninth_strategy)
			{
				memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
				//重新划分任务区
				task_area_replan();
				//有人机最少方案
				dispel_manned_strategy();
			}
			else
			{
				//检查反潜推荐
				strategyRecommend();
			}

		}
		else if(blk_dtms_ctas_001.task_type == 1)
		{
			if(fangan_send_flag == 1 && !use_only_ninth_strategy)
			{
				memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
				//有人机最少方案
				yz_dispel_manned_strategy();
			}
			else
			{
				//分割任务区
				yinzhao_point_init();
				//应召反潜推荐
//				yinzhaoRecommend();
				strategyRecommend();
			}

		}
		//方案编号
		fangan_index++;
		information_on_the_results_of_taskings.program_number = fangan_index;
		//首次生成
		information_on_the_results_of_taskings.tasking_release = 2;
	}

	/*浮标布阵规划,吊声定测点规划发送*/
	if(buoy_suspended_flag == 1)
	{
		buoy_suspended_generation();
	}

	//任务规划发送
	if(renwu_guihua_flag == 1)
	{
		//检查反潜
		if(blk_dtms_ctas_001.task_type == 0)
		{
			//任务区划分信息
			send_taskarea_result();
		}
		//任务分配结果
		send_tactics_result();
	}

	//每次任务分配结果、浮标布阵规划,吊声定测点规划发送完成后执行更新fangan_send_flag,直到为0
	if(fangan_send_flag > 0 && renwu_guihua_flag == 0 && buoy_suspended_flag == 0 )
	{
		fangan_send_flag--; //规划方案数
	}

	//航线生成
	message_size = 4096;
	Receive_Message(DDSTables.BLK_DTMS_CTAS_005.niConnectionId, 0, &transaction_id, &blk_dtms_ctas_005, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0 )
	{
		//解析航线生成信息
		
		init_blk_dtms_ctas_005();
		//当前阶段索引
		int stage_index = blk_dtms_ctas_005.blk_ccc_ofp_038.stage_id - 1;
		//当前阶段是否包含浮标或吊声任务
		if(blk_dtms_ctas_005.blk_ccc_ofp_038.fb_ds_tsak == 1)
		{
			//浮标解算结果
			message_size = 4096;
			Receive_Message(DDSTables.BLK_DTMS_CTAS_006.niConnectionId, 0, &transaction_id, &blk_ofp_ccc_302, &message_type_id, &message_size, &enRetCode);
			if(enRetCode == 0)
			{
				init_blk_ofp_ccc_302(stage_index);//浮标解算转换为有人机航路
			}
			//吊声规划信息
			message_size = 4096;
			Receive_Message(DDSTables.BLK_DTMS_CTAS_007.niConnectionId, 0, &transaction_id, &blk_ofp_ccc_402, &message_type_id, &message_size, &enRetCode);
			if(enRetCode == 0)
			{
				init_blk_ofp_ccc_402(stage_index);//吊声规划转换为有人机航路
			}
		}
		else
		{
			//有人机航线生成
			mannedAircraftRoutesGeneration();
		}
		//应召反潜，有人机最少方案，无人机航线为磁探跟踪
		if(blk_dtms_ctas_001.task_type == 1 && information_on_the_results_of_taskings.program_attributes == 3)
		{
			yz_CTGZ();
		}
		else
		{
			//无人机航路规划方案生成
			UAVRouteGeneration();
		}


//#if AIRWAY_LIMIT
//    //空域许可范围
//    int area_rtn = 0;
//    for(int i = 0 ; i < CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number ; i++ )
//    {
//    	area_rtn = airway_area_confirm(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude,
//    			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude);
//    	if(area_rtn < 0)
//    	{
//    		//任务目标点超过许可空域范围 -1 ; 任务目标点距离许可空域边界过近 -2
//    		if(area_rtn == -1 || area_rtn == -2)
//    		{
//    			//异常反馈
//				data_length = sizeof(int);
//				Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
//				if(enRetCode == 0)
//				{
//					printf("airway_area erro\n");
//					return;
//				}
//    		}
//    	}
//    }
//#endif
		/********** 空域生成 *******/
		calc_air_area(stage_index);
		/********** 防撞解算 *******/
		hx_avoid(stage_index);
		//有人机航线发送
		send_manned_aircraft_route_drone_route_information(stage_index);
		//无人机航线发送
		for(int i = 0 ; i < integrated_postures.drone_num ; i ++)
		{
			send_drone_route_confirmation(i,stage_index);
		}
		//发送空域
		send_airway_area();
		//解算完成反馈
		data_length = sizeof(int);
		int rtn = 10086;
		Send_Message(DDSTables.BLK_CTAS_DTMS_009.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
		if(enRetCode == 0)
		{
			printf("jie suan success\n");
		}
	}
}
//空域许可判断
int airway_area_confirm(double task_lon,double task_lat)
{
	//任务空域圆心
	double area_lon = 124.2644;//塔哈 124.2644 		124 15 52  海南
	double area_lat = 47.533611;//塔哈 47.533611 	47 32 01  海南
	//空域半径
	double R = 10.0;//单位km
	//分割方向，正北为0°
	double theta = 83;
	//转弯半径
	double r = 0.8;


	printf("task_lat = %lf  task_lon = % lf\n",task_lat,task_lon);
	//1.中心点距离目标点距离小于半径R
	//计算距离D
	double D = calculate_distances(area_lat,area_lon,task_lat,task_lon);
	if(D > R)
	{
		//任务目标点超过许可空域范围
		printf("distance > R\n");
		return -1;
	}

//	//2.中心点到目标点的夹角在正北方向为基准的，0-θ，180+θ-360°
//	//计算相对夹角beta
//	double beta = calculate_angle(area_lat,area_lon,task_lat,task_lon);
//	if(beta >= theta &&  beta <= 180+theta)
//	{
//		//任务目标点超过许可空域范围
//		printf("beta out of range\n");
//		return -1;
//	}
//
//	//3.0<=β<180 ,180<=β<360 ,剩余距离要大于转弯半径
//	if(beta < 180)
//	{
//		double d_value = sin((theta - beta)  * M_PI /180.0) * D;
//		if(d_value < r)
//		{
//			//任务目标点距离许可空域边界过近
//			printf("too close to airway1\n");
//			return -2;
//		}
//	}
//	else if(beta >= 180)
//	{
//		double d_value = sin((beta - theta - 180) * M_PI /180.0) * D;
//		if(d_value < r)
//		{
//			//任务目标点距离许可空域边界过近
//			printf("too close to airway2\n");
//			return -2;
//		}
//	}

	//4.中心点与目标点的距离要小于空域半径-转弯半径
	if(D > R - r)
	{
		//任务目标点距离许可空域边界过近
		printf("too close to airway3\n");
		return -2;
	}

	return 1;
}
double calculate_angle(double lat1, double lon1,double lat2, double lon2)
{
	//转换为弧度
	double lat1_rad = lat1 * M_PI / 180.0;
	double lon1_rad = lon1 * M_PI / 180.0;
	double lat2_rad = lat2 * M_PI / 180.0;
	double lon2_rad = lon2 * M_PI / 180.0;

	//计算经度差
	double delta_lon = lon2_rad - lon1_rad;

	//计算方位角的分子和分母
	double numerator = sin(delta_lon) * cos(lat2_rad);
	double denominator = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon);

	//计算反正切值(弧度)
	double alpha_rad = atan2(numerator,denominator);

	//转换为度数
	double alpha_deg = alpha_rad *180.0 / M_PI;

	//调整到0-360°范围
	if(alpha_deg < 0.0)
	{
		alpha_deg += 360.0;
	}

	return alpha_deg;
}
void signal_moduel()
{
	static int plan_tpye = 0;
	//单任务区/目标、单无人机规划，攻击规划
	message_size = 4096;
	memset(&blk_dtms_ctas_002,0,sizeof(blk_dtms_ctas_002));
	Receive_Message(DDSTables.BLK_DTMS_CTAS_002.niConnectionId, 0, &transaction_id, &blk_dtms_ctas_002, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0 )
	{
		//初始化任务分配结果
		memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
		if(blk_dtms_ctas_002.uav_id == 0)
		{
			//单任务区/目标
			plan_tpye = 1;
		}
		else if(blk_dtms_ctas_002.uav_id > 0)
		{
			//单无人机指控
			plan_tpye = 2;
		}
	}
	if(plan_tpye == 1)
	{
		//将收到的战术战法指令赋值给规划所需使用的任务区、有无人机等数据
		init_zhanshutuijians_single();

		if(blk_dtms_ctas_002.target_type == 1)//任务区
		{
			//任务区划分
			if(global_mission_planning_commandss.tactical_warfare_options != 9)
			{
				single_area_division();
			}
			//浮标帧收
			if(blk_dtms_ctas_002.task_type == 1)
			{
				single_area_FBZS();
			}
			//磁探搜索
			else if(blk_dtms_ctas_002.task_type == 5)
			{
				single_area_CTSS();
			}
			//光电搜索
			else if(blk_dtms_ctas_002.task_type == 7)
			{
				single_area_GDSS();
			}

		}
		else if(blk_dtms_ctas_002.target_type == 2 || blk_dtms_ctas_002.target_type == 3)//目标/光标
		{
			//攻击
			if(blk_dtms_ctas_002.task_type == 15)
			{
				//生成三个方案
				for(int i = 1 ; i <= 3 ; i ++)
				{
					//攻击规划
					OutputResults output;
					//初始化
					memset(&output , 0 , sizeof(OutputResults));
					attack_commend(&output,i);
					//攻击规划结果赋值
					attack_init(output,i);
//					//有人机航线发送
//					send_manned_route();
					//无人机航线发送
					send_drone_route();
					//发送安全区、威胁区
					send_blk_ctas_dtms_008();
					//单任务区任务分配结果发送
					send_single_result();
				}
				//解算完成反馈
				data_length = sizeof(int);
				int rtn = 9527;
				Send_Message(DDSTables.BLK_CTAS_DTMS_009.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
				if(enRetCode == 0)
				{
					printf("attack jie suan success\n");
				}

			}
			//双机编队
			else if(blk_dtms_ctas_002.task_type == 9)
			{
				int bdfx_ret = double_uav_BDFX();
				//解算完成反馈
				data_length = sizeof(int);
				int rtn = (bdfx_ret == 0) ? 9527 : bdfx_ret;
				Send_Message(DDSTables.BLK_CTAS_DTMS_009.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
				if(enRetCode == 0)
				{
					printf("BDFX jie suan %s ret=%d\n", (bdfx_ret==0)?"success":"FAILED", bdfx_ret);
				}
			}
			//磁探跟踪
			else
			{
				single_CT();
			}
		}
		plan_tpye = 0;
	}
	//单无人机指控
	if(plan_tpye ==2)
	{
		if(blk_dtms_ctas_002.target_type == 1)//任务区
		{
			//浮标帧收
			if(blk_dtms_ctas_002.task_type == 1)
			{
				single_uav_FBZS();
			}
			//磁探搜索
			if(blk_dtms_ctas_002.task_type == 5)
			{
				single_uav_CTSS();
			}
			//光电搜索
			if(blk_dtms_ctas_002.task_type == 7)
			{
				single_uav_GDSS();
			}
		}
		else if(blk_dtms_ctas_002.target_type == 2 || blk_dtms_ctas_002.target_type == 3 || blk_dtms_ctas_002.target_type == 4)//目标/光标选点/航路点
		{
			//悬停等待/盘旋等待
			if(blk_dtms_ctas_002.task_type == 12 || blk_dtms_ctas_002.task_type == 14 )
			{
				single_uav_XT();
			}
			//磁探跟踪
			if(blk_dtms_ctas_002.task_type == 6)
			{
				single_uav_CTGZ();
			}
			//光电跟踪
			if(blk_dtms_ctas_002.task_type == 8)
			{
				single_uav_GDGZ();
			}
			//编队飞行
			if(blk_dtms_ctas_002.task_type == 9)
			{
				single_uav_BDFX();
			}
		}
		plan_tpye = 0;
	}
}
//单任务区指控任务区切分
void single_area_division()
{
	//初始化
	memset( &taskarea_division , 0 , sizeof(BLK_CCC_OFP_005) );
	//3.10 任务区/空域信息
	if(blk_dtms_ctas_002.task_reg_num == 1)
	{
		int divi_num = integrated_postures.drone_num;
		//任务区数量为1时，划分任务区（暂时划分无人机数量相等的任务区）+有人机
		task_area_division(&blk_dtms_ctas_002.region_infos[0],divi_num);

		taskarea_division.task_are = 1;/*划分任务区个数 对几个任务区划分*/
		taskarea_division.task_are_hf2[0].Task_Are_ID = blk_dtms_ctas_002.region_infos[0].reg_id;
		taskarea_division.task_are_hf2[0].task_are_hf_num = divi_num;

		for(int i = 0 ; i < divi_num ; i ++)
		{
			taskarea_division.task_are_hf2[0].signal_FC00[i].Task_Are_ID = i + 1;
			for(int j = 0 ; j < 4 ; j ++)
			{
				taskarea_division.task_are_hf2[0].signal_FC00[i].signal_FC00[j].Index_Lat = area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude;
				taskarea_division.task_are_hf2[0].signal_FC00[i].signal_FC00[j].Index_Lon= area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude;
			}
		}
	}
	else if(blk_dtms_ctas_002.task_reg_num > 1)
	{
		area_sky_informations.area_number = blk_dtms_ctas_002.task_reg_num;//任务区数量
		for(int i = 0;i < area_sky_informations.area_number;i++){
			area_sky_informations.area_informations[i].area_code = blk_dtms_ctas_002.region_infos[i].reg_id;//区域编号
			area_sky_informations.area_informations[i].area_type = blk_dtms_ctas_002.region_infos[i].reg_type;//区域类型
			area_sky_informations.area_informations[i].area_source = blk_dtms_ctas_002.region_infos[i].reg_sour;//区域来源
			area_sky_informations.area_informations[i].area_shape = blk_dtms_ctas_002.region_infos[i].reg_shape;//区域形状
			//        area_sky_informations.area_informations[i].drone_number_valid_bit = DTMS_CTAS_data_tacticsRecommend->region_infos[i].kongyu_belong_to_uav_valid;
			area_sky_informations.area_informations[i].drone_numbe = blk_dtms_ctas_002.region_infos[i].kongyu_belong_to_uav_id;//空域所属无人机序号
			area_sky_informations.area_informations[i].upper_height_limit_valid_bit = blk_dtms_ctas_002.region_infos[i].reg_top_of_hei_valid;//区域高度上限有效位
			area_sky_informations.area_informations[i].lower_height_limit_valid_bit = blk_dtms_ctas_002.region_infos[i].reg_down_of_hei_valid;//区域高度下限有效位
			area_sky_informations.area_informations[i].upper_height_limit = blk_dtms_ctas_002.region_infos[i].top_of_hei;//区域高度上限
			area_sky_informations.area_informations[i].lower_height_limit = blk_dtms_ctas_002.region_infos[i].down_of_hei;//区域高度下限
			//圆区域赋值
			area_sky_informations.area_informations[i].cycles.radius = blk_dtms_ctas_002.region_infos[i].reg_circle.radious;//圆半径
			area_sky_informations.area_informations[i].cycles.longitude = blk_dtms_ctas_002.region_infos[i].reg_circle.center_lon_lat.longitude;//圆形经度
			area_sky_informations.area_informations[i].cycles.latitude = blk_dtms_ctas_002.region_infos[i].reg_circle.center_lon_lat.latitude;//圆形纬度
			//多边形区域赋值
			area_sky_informations.area_informations[i].polygonals.point_number = blk_dtms_ctas_002.region_infos[i].reg_ploygen.point_num;//多边形点数
			for(int j = 0;j < area_sky_informations.area_informations[i].polygonals.point_number;j++){
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude = blk_dtms_ctas_002.region_infos[i].reg_ploygen.points_lon_lat[j].longitude;//点经度
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude = blk_dtms_ctas_002.region_infos[i].reg_ploygen.points_lon_lat[j].latitude;//点纬度
			}
		}
	}
}
void single_CT()
{
	//初始化任务分配结果
	memset(&information_on_the_results_of_taskings,0,sizeof(Information_the_results_of_tasking));
	//目标经纬度
	double lat,lon;
	lat = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude;
	lon = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude;

	//找到最近的无人机
	unsigned int uav_index = distance_min( lat , lon );
	if(uav_index <= 0)
	{
		return;
	}
	if(blk_dtms_ctas_002.planning_id == 0)
	{
		information_on_the_results_of_taskings.program_number = 3;
	}
	else
	{
		information_on_the_results_of_taskings.program_number = blk_dtms_ctas_002.planning_id;
	}
	information_on_the_results_of_taskings.tasking_release = 6;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

	information_on_the_results_of_taskings.number_of_mission_platforms = 2;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index].platform_model = 2;//平台型号--无人机

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index].platform_serial_number = \
			drone_state_informations.drone_specific_informations[uav_index - 1].platform_serial_num;//平台序号---无人机i

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index].platform_number = \
			drone_state_informations.drone_specific_informations[uav_index - 1].platform_num;//平台编号---无

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index].number_of_subtasks = 1;//任务数量为1

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index].task_sequence_informations[0].sequence_type = \
			blk_dtms_ctas_002.task_type;//任务类型
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index].task_sequence_informations[0].modify = 2;//任务序列变更
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位

	//另一架机去盘旋
	int px_uav = 0;
	px_uav = (uav_index-1 == 0) ? 1:0;
	if(drone_state_informations.drone_number == 2)
	{
		information_on_the_results_of_taskings.formation_synergy_mission_programs[px_uav+1].platform_model = 2;//平台型号--无人机

		information_on_the_results_of_taskings.formation_synergy_mission_programs[px_uav+1].platform_serial_number = \
				drone_state_informations.drone_specific_informations[px_uav].platform_serial_num;//平台序号---无人机i

		information_on_the_results_of_taskings.formation_synergy_mission_programs[px_uav+1].platform_number = \
				drone_state_informations.drone_specific_informations[px_uav].platform_num;//平台编号---无

		information_on_the_results_of_taskings.formation_synergy_mission_programs[px_uav+1].number_of_subtasks = 1;//任务数量为1

		information_on_the_results_of_taskings.formation_synergy_mission_programs[px_uav+1].task_sequence_informations[0].sequence_type = 14;

		information_on_the_results_of_taskings.formation_synergy_mission_programs[px_uav+1].task_sequence_informations[0].modify = 2;//任务序列变更
		information_on_the_results_of_taskings.formation_synergy_mission_programs[px_uav+1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
		information_on_the_results_of_taskings.formation_synergy_mission_programs[px_uav+1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
		information_on_the_results_of_taskings.formation_synergy_mission_programs[px_uav+1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位
	}

	SGeo UAV_aircraft;
	SGeo goal_point;
	UAV_aircraft.Lon = integrated_postures.integrated_posture_drone_informations[uav_index-1].drone_longitude_and_latitude.longitude;
	UAV_aircraft.Lat = integrated_postures.integrated_posture_drone_informations[uav_index-1].drone_longitude_and_latitude.latitude;
	goal_point.Lon = lon;
	goal_point.Lat = lat;
	//无人机地速km/h
	float speed = integrated_postures.integrated_posture_drone_informations[uav_index-1].drone_speed;
	//磁探跟踪航路规划
	new_route_generate(UAV_aircraft,speed,goal_point,uav_index-1);
	//航路信息
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index -1].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index - 1].platform_serial_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index -1].drone_num = drone_state_informations.drone_specific_informations[uav_index - 1].platform_num;

	//生成空域
	single_air_area(uav_index -1);
	//发送空域信息
	send_airway_area();

	//悬停空域计算
	XT_area_calc(blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude
			,blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude
			,px_uav + 1);
	//发送空域信息
	send_airway_area();

	//航线发送
	send_drone_route_confirmation(uav_index - 1,0);
	send_drone_route_confirmation(px_uav,0);

	//单任务区任务分配结果发送
	send_single_result();

	//解算完成反馈
	data_length = sizeof(int);
	int rtn = 9527;
	Send_Message(DDSTables.BLK_CTAS_DTMS_009.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0)
	{
		printf("CT jie suan success\n");
	}
}
void single_area_CTSS()
{
	//初始化无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));

	//单任务区指控
	information_on_the_results_of_taskings.program_number = 3;//3为假定值

	information_on_the_results_of_taskings.tasking_release = 6;//任务分配方案发布---修改
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

	information_on_the_results_of_taskings.number_of_mission_platforms = integrated_postures.drone_num;
	int droneNum = integrated_postures.drone_num;
	if(droneNum > 4) droneNum = 4;

	GeoLibDas uavPositionA[4];
	GeoLibDas searchOriginA[4];
	area_information areaA[4];
	for(int i = 0; i < droneNum; i++) {
		uavPositionA[i].longitude = integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.longitude;
		uavPositionA[i].latitude = integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.latitude;
		areaA[i] = area_sky_informations.area_informations[i];
		searchOriginA[i] = getNearestPolygonVertexForUav(uavPositionA[i], areaA[i]);
	}
	if(droneNum == 2) {
		GeoLibDas dualUavPos[2] = {uavPositionA[0], uavPositionA[1]};
		area_information dualArea[2] = {areaA[0], areaA[1]};
		GeoLibDas dualInPoint[2] = {searchOriginA[0], searchOriginA[1]};
		optimizeDualUavInPoints(dualUavPos, dualArea, dualInPoint);
		searchOriginA[0] = dualInPoint[0];
		searchOriginA[1] = dualInPoint[1];
	}

	for(int uav_index = 0; uav_index < integrated_postures.drone_num ; uav_index ++)
	{
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_model = 2;//平台型号--无人机

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_serial_number = \
				drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;//平台序号---无人机i

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_number = \
				drone_state_informations.drone_specific_informations[uav_index].platform_num;//平台编号---无

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].number_of_subtasks = 1;//任务数量为1

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].sequence_type = \
				blk_dtms_ctas_002.task_type;//任务类型
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].modify = 2;//任务序列变更
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[uav_index].area_code;
		//赋值航路信息
		CTAS_DTMS_data_UAVRoute.program_number = information_on_the_results_of_taskings.program_number;
		CTAS_DTMS_data_UAVRoute.plan_type = 3;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].mission_type = blk_dtms_ctas_002.task_type;//磁探搜索
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].point_area_type = 3;/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].area_point_line_goal_number = area_sky_informations.area_informations[uav_index].area_code;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
		//找到无人机位置
		//subtask:磁探搜索(光电搜索暂时替代)
		OutSearchRadarPhoto outSearchRadarPhoto;
		outSearchRadarPhoto = commonRouteGeneration(area_sky_informations.area_informations[uav_index],searchOriginA[uav_index],1000);

		//最多75个点 20250611new
		if(outSearchRadarPhoto.sumAwp > 75) outSearchRadarPhoto.sumAwp = 75;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number = outSearchRadarPhoto.sumAwp;

		//计算航路信息包数 20250611new
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet = outSearchRadarPhoto.sumAwp/25;
		if(outSearchRadarPhoto.sumAwp%25)
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet++;

		for(int k = 0;k < outSearchRadarPhoto.sumAwp;k++){
			//计算航路点编号 20250611new
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].hld_idx = k+1;
			//        CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].type = blk_dtms_ctas_002.task_type;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].longitude = outSearchRadarPhoto.awpA[k].longitude;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].latitude = outSearchRadarPhoto.awpA[k].latitude;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].validity_of_longitude = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].latitude_validity = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].height_validity = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].speed_validity = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].direction_validity = 0;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].time_validity = 0;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].payloads_validity = 1;
			//高度---待补充
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].height = blk_dtms_ctas_010[uav_index].CTSS;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].speed = 27;
			//速度---待补充
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].direction = 0;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].time = 0;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].payloads = 2;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].causality = 1;
			//最后一个点为盘旋点
			if(k == outSearchRadarPhoto.sumAwp - 1)
			{
				//航路点待机时间/圈数/循环次数
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber
				= PX_CIRCLE;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
				//航路点待机半径
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_radius
				= PX_RAD;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_radius_valid_bit = 1;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].causality = 3;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_type = 2;//圈数
			}
		}

		//生成空域
		single_air_area(uav_index);
		//发送空域信息
		send_airway_area();
	}
	//两架机考虑航线冲突
	if(integrated_postures.drone_num == 2)
	{
		//生成成功后判断航线是否有交叉
		FlightRoute route_uav1,route_uav2;
		route_uav1.start.latitude = integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.latitude;
		route_uav1.start.longitude = integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.longitude;
		route_uav2.start.latitude = integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.latitude;
		route_uav2.start.longitude = integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.longitude;
		route_uav1.end.latitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].planning_information_waypoint_informations[0].latitude;
		route_uav1.end.longitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].planning_information_waypoint_informations[0].longitude;
		route_uav2.end.latitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1].planning_information_waypoint_informations[0].latitude;
		route_uav2.end.longitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1].planning_information_waypoint_informations[0].longitude;
		int rtn = 0;
		rtn = detectConflict(&route_uav1,&route_uav2);
		//有冲突
		if(rtn == 1 || (fabs(route_uav1.end.latitude  - route_uav2.end.latitude)  < 1e-7 &&
						fabs(route_uav1.end.longitude - route_uav2.end.longitude) < 1e-7))
		{
			//任务区交换
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number =
					area_sky_informations.area_informations[1].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number =
					area_sky_informations.area_informations[0].area_code;

			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].area_point_line_goal_number =
					area_sky_informations.area_informations[1].area_code;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1].area_point_line_goal_number =
					area_sky_informations.area_informations[0].area_code;
			//航线交换
			planning_information tmp;
			memcpy(&tmp,&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1],sizeof(planning_information));
			memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1],
					&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1],sizeof(planning_information));
			memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1],&tmp,sizeof(planning_information));
		}
	}

	for(int i = 0 ; i < integrated_postures.drone_num ; i ++)
	{
		//航线发送,下标为1防止多加入无人机当前航路点
		send_drone_route_confirmation(i,1);
	}
	//任务区划分信息
	send_taskarea_result();
	//单任务区任务分配结果发送
	send_single_result();
	//解算完成反馈
	data_length = sizeof(int);
	int rtn = 9527;
	Send_Message(DDSTables.BLK_CTAS_DTMS_009.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0)
	{
		printf("ctss jie suan success\n");
	}
}
void single_area_GDSS()
{
	//初始化无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));

	//单任务区指控
	information_on_the_results_of_taskings.program_number = 3;//3为假定值

	information_on_the_results_of_taskings.tasking_release = 6;//任务分配方案发布---修改
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

	information_on_the_results_of_taskings.number_of_mission_platforms = integrated_postures.drone_num;
	int droneNum = integrated_postures.drone_num;
	if(droneNum > 4) droneNum = 4;

	GeoLibDas uavPositionA[4];
	GeoLibDas searchOriginA[4];
	area_information areaA[4];
	for(int i = 0; i < droneNum; i++) {
		uavPositionA[i].longitude = integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.longitude;
		uavPositionA[i].latitude = integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.latitude;
		areaA[i] = area_sky_informations.area_informations[i];
		searchOriginA[i] = getNearestPolygonVertexForUav(uavPositionA[i], areaA[i]);
	}
	if(droneNum == 2) {
		GeoLibDas dualUavPos[2] = {uavPositionA[0], uavPositionA[1]};
		area_information dualArea[2] = {areaA[0], areaA[1]};
		GeoLibDas dualInPoint[2] = {searchOriginA[0], searchOriginA[1]};
		optimizeDualUavInPoints(dualUavPos, dualArea, dualInPoint);
		searchOriginA[0] = dualInPoint[0];
		searchOriginA[1] = dualInPoint[1];
	}

	for(int uav_index = 0; uav_index < integrated_postures.drone_num ; uav_index ++)
	{
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_model = 2;//平台型号--无人机

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_serial_number = \
				drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;//平台序号---无人机i

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_number = \
				drone_state_informations.drone_specific_informations[uav_index].platform_num;//平台编号---无

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].number_of_subtasks = 1;//任务数量为1

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].sequence_type = \
				blk_dtms_ctas_002.task_type;//任务类型
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].modify = 2;//任务序列变更
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[uav_index].area_code;
		//赋值航路信息
		CTAS_DTMS_data_UAVRoute.program_number = information_on_the_results_of_taskings.program_number;
		CTAS_DTMS_data_UAVRoute.plan_type = 3;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].mission_type = blk_dtms_ctas_002.task_type;//光电搜索
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].point_area_type = 3;/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].area_point_line_goal_number = area_sky_informations.area_informations[uav_index].area_code;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
		//找到无人机位置
		GeoLibDas UAV_position;
		UAV_position.longitude = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.longitude;
		UAV_position.latitude = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.latitude;
		//subtask:磁探搜索(光电搜索暂时替代)
		OutSearchRadarPhoto outSearchRadarPhoto;
		outSearchRadarPhoto = commonRouteGeneration(area_sky_informations.area_informations[uav_index],searchOriginA[uav_index],12000);

		//最多75个点 20250611new
		if(outSearchRadarPhoto.sumAwp > 75) outSearchRadarPhoto.sumAwp = 75;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number = outSearchRadarPhoto.sumAwp;

		//计算航路信息包数 20250611new
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet = outSearchRadarPhoto.sumAwp/25;
		if(outSearchRadarPhoto.sumAwp%25)
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet++;

		for(int k = 0;k < outSearchRadarPhoto.sumAwp;k++){
			//计算航路点编号 20250611new
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].hld_idx = k+1;
			//        CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].type = blk_dtms_ctas_002.task_type;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].longitude = outSearchRadarPhoto.awpA[k].longitude;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].latitude = outSearchRadarPhoto.awpA[k].latitude;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].validity_of_longitude = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].latitude_validity = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].height_validity = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].speed_validity = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].direction_validity = 0;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].time_validity = 0;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].payloads_validity = 1;
			//高度---待补充
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].height = blk_dtms_ctas_010[uav_index].CTSS;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].speed = 27;
			//速度---待补充
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].direction = 0;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].time = 0;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].payloads = 2;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].causality = 1;
			//最后一个点为盘旋点
			if(k == outSearchRadarPhoto.sumAwp - 1)
			{
				//航路点待机时间/圈数/循环次数
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber
				= PX_CIRCLE;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
				//航路点待机半径
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_radius
				= PX_RAD;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_radius_valid_bit = 1;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].causality = 3;
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_type = 2;//圈数
			}
		}

		//生成空域
		single_air_area(uav_index);
		//发送空域信息
		send_airway_area();
	}
	//两架机考虑航线冲突
	if(integrated_postures.drone_num == 2)
	{
		//生成成功后判断航线是否有交叉
		FlightRoute route_uav1,route_uav2;
		route_uav1.start.latitude = integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.latitude;
		route_uav1.start.longitude = integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.longitude;
		route_uav2.start.latitude = integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.latitude;
		route_uav2.start.longitude = integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.longitude;
		route_uav1.end.latitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].planning_information_waypoint_informations[0].latitude;
		route_uav1.end.longitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].planning_information_waypoint_informations[0].longitude;
		route_uav2.end.latitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1].planning_information_waypoint_informations[0].latitude;
		route_uav2.end.longitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1].planning_information_waypoint_informations[0].longitude;
		int rtn = 0;
		rtn = detectConflict(&route_uav1,&route_uav2);
		//有冲突
		if(rtn == 1 || (fabs(route_uav1.end.latitude  - route_uav2.end.latitude)  < 1e-7 &&
						fabs(route_uav1.end.longitude - route_uav2.end.longitude) < 1e-7))
		{
			//任务区交换
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number =
					area_sky_informations.area_informations[1].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number =
					area_sky_informations.area_informations[0].area_code;

			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].area_point_line_goal_number =
					area_sky_informations.area_informations[1].area_code;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1].area_point_line_goal_number =
					area_sky_informations.area_informations[0].area_code;
			//航线交换
			planning_information tmp;
			memcpy(&tmp,&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1],sizeof(planning_information));
			memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1],
					&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1],sizeof(planning_information));
			memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1],&tmp,sizeof(planning_information));
		}
	}

	for(int i = 0 ; i < integrated_postures.drone_num ; i ++)
	{
		//航线发送,下标为1防止多加入无人机当前航路点
		send_drone_route_confirmation(i,1);
	}
	//任务区划分信息
	send_taskarea_result();
	//单任务区任务分配结果发送
	send_single_result();
	//解算完成反馈
	data_length = sizeof(int);
	int rtn = 9527;
	Send_Message(DDSTables.BLK_CTAS_DTMS_009.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0)
	{
		printf("ctss jie suan success\n");
	}
}
int FBZS_init(SGeo UAV_aircraft,unsigned int uav_index)
{
	//任务区转换
	RectLibDas area;
	if(area_sky_informations.area_informations[0].area_shape == 1) //圆形
	{
		RectLibDas *tmp;
		GeoLibDas centre;
		double radius;
		radius = (double)(area_sky_informations.area_informations[0].cycles.radius);
		centre.longitude = area_sky_informations.area_informations[0].cycles.longitude;
		centre.latitude = area_sky_informations.area_informations[0].cycles.latitude;
		tmp = cycle_to_RectLibDas(centre,radius);
		memcpy(&area,tmp,sizeof(RectLibDas));
	}
	else if(area_sky_informations.area_informations[0].area_shape == 2)//矩形
	{
		for(int i = 0 ; i < 4 ; i ++)
		{
			area.vertexA[i].latitude = area_sky_informations.area_informations[0].polygonals.point_coordinates[i].latitude;
			area.vertexA[i].longitude = area_sky_informations.area_informations[0].polygonals.point_coordinates[i].longitude;
		}
	}

	//长宽大小
	double L1 = calculate_distances(area.vertexA[0].latitude,area.vertexA[0].longitude,area.vertexA[1].latitude,area.vertexA[1].longitude);
	double L2 = calculate_distances(area.vertexA[1].latitude,area.vertexA[1].longitude,area.vertexA[2].latitude,area.vertexA[2].longitude);
	//矩形中心
	SGeo center;
	center.Lat = (area.vertexA[0].latitude + area.vertexA[2].latitude) / 2.0;
	center.Lon = (area.vertexA[0].longitude + area.vertexA[2].longitude) / 2.0;

	//航路点数量和航路点经纬度
	unsigned short point_num = 0;
	SGeo route_point[25];

	//航线生成
	//L1小于2km或L2小于2km
	if(L1 < 2.0 || L2 < 2.0)
	{
		//生成失败，任务范围区域过小，转弯受限
		return -3;
	}
	//L1大于50km或L2大于50km
	else if(L1 > 50.0 || L2 > 50.0)
	{
		//生成失败，任务区域范围过大，超出侦听距离
		return -4;
	}
	//L1大于2km且L1小于5km，L2大于2km且L2小于5km
	else if( (L1 > 2.0 && L1 < 5.0) || (L2 > 2.0 && L2 < 5.0) )
	{
		//中心点盘旋
		route_point[0].Lat = center.Lat;
		route_point[0].Lon = center.Lon;
		point_num++;
	}
	//L1大于5km且L1小于50km，L2大于5km且L2小于50km
	else if( (L1 > 5.0 && L1 < 50.0) && (L2 > 5.0 && L2 < 50.0) )
	{
		//寻找距离当前无人机距离最短的任务区端点
		double shortest_D = 0;//最短距离
		unsigned int  shortest_n = 0;//最短距离序号
		for(int n = 0 ; n < 4 ; n++)
		{
			if(n == 0)
			{
				shortest_D = calculate_distances(UAV_aircraft.Lat,UAV_aircraft.Lon,area.vertexA[n].latitude,area.vertexA[n].longitude);
			}
			else
			{
				double temp = calculate_distances(UAV_aircraft.Lat,UAV_aircraft.Lon,area.vertexA[n].latitude,area.vertexA[n].longitude);
				if(temp < shortest_D)
				{
					shortest_D = temp;
					shortest_n = n;
				}
			}
		}

		SGeo list[4];
		for(int j = 0 ; j < 4 ; j ++)
		{
			unsigned int N1 = shortest_n + j;
			if(N1 > 3)
			{
				N1 -= 4;
			}
			//计算方位角
			double course = calculate_angle(center.Lat,center.Lon,area.vertexA[j].latitude,area.vertexA[j].longitude);
			//基于中心点和方位角，距离，计算出目标点位置
			SPolar goal;
			double course_A;
			course_A = 90 - course;
			goal.A = course_A;
			goal.R = 5.0;
			list[j]= Polar_to_Geo(center,goal);


		}
		//循环3次
		for(int i = 0 ; i < 3 ; i++)
		{
			route_point[4*i] = list[0];
			route_point[4*i+1] = list[1];
			route_point[4*i+2] = list[2];
			route_point[4*i+3] = list[3];
			point_num += 4;

			//增加最后一个中心点作为盘旋点 20251021new
			if(i == 2)
			{
				AreaRectVertex tmp;
				AreaRectCenter rtn;
				for(int k = 0; k < 4 ; k ++)
				{
					tmp.vertexA[k].latitude = list[k].Lat;
					tmp.vertexA[k].longitude = list[k].Lon;
				}
				rtn = getAreaRectCenterByVertex(&tmp);

				route_point[4*i+4].Lat = rtn.center.latitude;
				route_point[4*i+4].Lon = rtn.center.longitude;

				point_num += 1;
			}
		}
	}


	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.program_number = information_on_the_results_of_taskings.program_number;//20260129
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;

	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number = point_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].subtask_ID_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet++;
	for(int i = 0;i < point_num;i++){
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude = route_point[i].Lon;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude = route_point[i].Lat;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].height = blk_dtms_ctas_010[uav_index].FBZS;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].speed = 35;
		//最后一个点为盘旋点
		if(i == point_num - 1)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber
			= PX_CIRCLE;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius
			= PX_RAD;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 3;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_type = 2;//圈数
		}
	}

	return 1;
}
int single_area_FBZS_init(SGeo UAV_aircraft,unsigned int uav_index)
{
	//任务区转换
	RectLibDas area;
	if(area_sky_informations.area_informations[uav_index].area_shape == 1) //圆形
	{
		RectLibDas *tmp;
		GeoLibDas centre;
		double radius;
		radius = (double)(area_sky_informations.area_informations[uav_index].cycles.radius);
		centre.longitude = area_sky_informations.area_informations[uav_index].cycles.longitude;
		centre.latitude = area_sky_informations.area_informations[uav_index].cycles.latitude;
		tmp = cycle_to_RectLibDas(centre,radius);
		memcpy(&area,tmp,sizeof(RectLibDas));
	}
	else if(area_sky_informations.area_informations[uav_index].area_shape == 2)//矩形
	{
		for(int i = 0 ; i < 4 ; i ++)
		{
			area.vertexA[i].latitude = area_sky_informations.area_informations[uav_index].polygonals.point_coordinates[i].latitude;
			area.vertexA[i].longitude = area_sky_informations.area_informations[uav_index].polygonals.point_coordinates[i].longitude;
		}
	}

	//长宽大小
	double L1 = calculate_distances(area.vertexA[0].latitude,area.vertexA[0].longitude,area.vertexA[1].latitude,area.vertexA[1].longitude);
	double L2 = calculate_distances(area.vertexA[1].latitude,area.vertexA[1].longitude,area.vertexA[2].latitude,area.vertexA[2].longitude);
	//矩形中心
	SGeo center;
	center.Lat = (area.vertexA[0].latitude + area.vertexA[2].latitude) / 2.0;
	center.Lon = (area.vertexA[0].longitude + area.vertexA[2].longitude) / 2.0;

	//航路点数量和航路点经纬度
	unsigned short point_num = 0;
	SGeo route_point[25];

	//航线生成
	//L1小于2km或L2小于2km
	if(L1 < 2.0 || L2 < 2.0)
	{
		//生成失败，任务范围区域过小，转弯受限
		return -3;
	}
	//L1大于50km或L2大于50km
	else if(L1 > 50.0 || L2 > 50.0)
	{
		//生成失败，任务区域范围过大，超出侦听距离
		return -4;
	}
	//L1大于2km且L1小于5km，L2大于2km且L2小于5km
	else if( (L1 > 2.0 && L1 < 5.0) || (L2 > 2.0 && L2 < 5.0) )
	{
		//中心点盘旋
		route_point[0].Lat = center.Lat;
		route_point[0].Lon = center.Lon;
		point_num++;
	}
	//L1大于5km且L1小于50km，L2大于5km且L2小于50km
	else if( (L1 > 5.0 && L1 < 50.0) && (L2 > 5.0 && L2 < 50.0) )
	{
		//寻找距离当前无人机距离最短的任务区端点
		double shortest_D = 0;//最短距离
		unsigned int  shortest_n = 0;//最短距离序号
		for(int n = 0 ; n < 4 ; n++)
		{
			if(n == 0)
			{
				shortest_D = calculate_distances(UAV_aircraft.Lat,UAV_aircraft.Lon,area.vertexA[n].latitude,area.vertexA[n].longitude);
			}
			else
			{
				double temp = calculate_distances(UAV_aircraft.Lat,UAV_aircraft.Lon,area.vertexA[n].latitude,area.vertexA[n].longitude);
				if(temp < shortest_D)
				{
					shortest_D = temp;
					shortest_n = n;
				}
			}
		}

		SGeo list[4];
		for(int j = 0 ; j < 4 ; j ++)
		{
			unsigned int N1 = shortest_n + j;
			if(N1 > 3)
			{
				N1 -= 4;
			}
			//计算方位角
			double course = calculate_angle(center.Lat,center.Lon,area.vertexA[j].latitude,area.vertexA[j].longitude);
			//基于中心点和方位角，距离，计算出目标点位置
			SPolar goal;
			double course_A;
			course_A = 90 - course;
			goal.A = course_A;
			goal.R = 3.0;
			list[j]= Polar_to_Geo(center,goal);


		}
		//循环3次
		for(int i = 0 ; i < 3 ; i++)
		{
			route_point[4*i] = list[0];
			route_point[4*i+1] = list[1];
			route_point[4*i+2] = list[2];
			route_point[4*i+3] = list[3];
			point_num += 4;

			//增加最后一个中心点作为盘旋点 20251021new
			if(i == 2)
			{
				AreaRectVertex tmp;
				AreaRectCenter rtn;
				for(int k = 0; k < 4 ; k ++)
				{
					tmp.vertexA[k].latitude = list[k].Lat;
					tmp.vertexA[k].longitude = list[k].Lon;
				}
				rtn = getAreaRectCenterByVertex(&tmp);

				route_point[4*i+4].Lat = rtn.center.latitude;
				route_point[4*i+4].Lon = rtn.center.longitude;

				point_num += 1;
			}
		}
	}


	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.plan_type = 3;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;

	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number = point_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].subtask_ID_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet++;
	for(int i = 0;i < point_num;i++){
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].hld_idx = i+1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude = route_point[i].Lon;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude = route_point[i].Lat;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].height = blk_dtms_ctas_010[uav_index].FBZS;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].speed = 35;
		//最后一个点为盘旋点
		if(i == point_num - 1)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber
			= PX_CIRCLE;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius
			= PX_RAD;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 3;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_type = 2;//圈数
		}
	}

	return 1;
}
void single_uav_FBZS()
{
	//将收到的战术战法指令赋值给规划所需使用的任务区、有无人机等数据
	init_zhanshutuijians_single();
	//初始化无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));
	//找到无人机索引
	unsigned int uav_index = 0;
	for(unsigned int i = 0 ; i < 4 ; i ++)
	{
		if(drone_state_informations.drone_specific_informations[i].platform_num == blk_dtms_ctas_002.uav_id)
		{
			uav_index = i;
			break;
		}
		uav_index = 44;
	}
	//索引异常
	if(uav_index > 4)
	{
		int area_rtn = -6;
		data_length = sizeof(int);
		Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
		return;
	}
	//方案编号获取
	if(blk_dtms_ctas_002.planning_id == 0)
	{
		//无运行方案，首次进行单无人机指控
		information_on_the_results_of_taskings.program_number = 3;//3为假定值
	}
	else
	{
		information_on_the_results_of_taskings.program_number = blk_dtms_ctas_002.planning_id;
	}



	information_on_the_results_of_taskings.tasking_release = 3;//任务分配方案发布---修改
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number;//20260129
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_model = 2;//平台型号--无人机

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_serial_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;//平台序号---无人机i

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_num;//平台编号---无

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].number_of_subtasks = 1;//任务数量为1

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].sequence_type = \
			blk_dtms_ctas_002.task_type;//任务类型
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].modify = 2;//任务序列变更
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位
	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].mission_type = blk_dtms_ctas_002.task_type;//浮标帧收
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].point_area_type = 3;/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].area_point_line_goal_number = area_sky_informations.area_informations[0].area_code;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
	//找到无人机位置
	SGeo UAV_position;
	UAV_position.Lon = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.longitude;
	UAV_position.Lat = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.latitude;
	int rtn = FBZS_init(UAV_position,uav_index);
	//生成失败直接返回
	if(rtn != 1)
	{
		Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
		printf("FB erro\n");
		return;
	}
#if AIRWAY_LIMIT
	//空域许可范围
	int area_rtn = 0;
	for(int i = 0 ; i < CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number ; i++ )
	{
		area_rtn = airway_area_confirm(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude,
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude);
		if(area_rtn < 0)
		{
			//任务目标点超过许可空域范围 -1 ; 任务目标点距离许可空域边界过近 -2
			if(area_rtn == -1 || area_rtn == -2)
			{
				//异常反馈
				data_length = sizeof(int);
				Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
				if(enRetCode == 0)
				{
					printf("airway_area erro\n");
					return;
				}
			}
		}
	}
#endif

	//盘旋点
	if(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number == 1)
	{
		//悬停空域计算
		XT_area_calc(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[0].longitude
				,CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[0].latitude
				,uav_index + 1);
	}
	//任务航线
	else
	{
		//生成空域
		single_air_area(uav_index);
	}
	//发送空域信息
	send_airway_area();
	//航线发送,下标为1防止多加入无人机当前航路点
	send_drone_route_confirmation(uav_index,1);
	//单任务区任务分配结果发送
	send_single_result();
}
//单任务区浮标帧收
void single_area_FBZS()
{
	//初始化无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));

	//单任务区指控
	information_on_the_results_of_taskings.program_number = 3;//3为假定值

	information_on_the_results_of_taskings.tasking_release = 6;//任务分配方案发布---修改
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

	information_on_the_results_of_taskings.number_of_mission_platforms = integrated_postures.drone_num;
	for(int uav_index = 0; uav_index < integrated_postures.drone_num ; uav_index ++)
	{
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_model = 2;//平台型号--无人机

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_serial_number = \
				drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;//平台序号---无人机i

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_number = \
				drone_state_informations.drone_specific_informations[uav_index].platform_num;//平台编号---无

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].number_of_subtasks = 1;//任务数量为1

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].sequence_type = \
				blk_dtms_ctas_002.task_type;//任务类型
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].modify = 2;//任务序列变更
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[uav_index].area_code;
		//赋值航路信息
		CTAS_DTMS_data_UAVRoute.program_number = information_on_the_results_of_taskings.program_number;
		CTAS_DTMS_data_UAVRoute.plan_type = 3;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].mission_type = blk_dtms_ctas_002.task_type;//浮标帧收
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].point_area_type = 3;/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].area_point_line_goal_number = area_sky_informations.area_informations[uav_index].area_code;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
		//找到无人机位置
		SGeo UAV_position;
		UAV_position.Lon = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.longitude;
		UAV_position.Lat = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.latitude;
		int rtn = single_area_FBZS_init(UAV_position,uav_index);
		//生成失败直接返回
		if(rtn != 1)
		{
			Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
			printf("FB erro\n");
			return;
		}
		//盘旋点
		if(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number == 1)
		{
			//悬停空域计算
			XT_area_calc(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[0].longitude
					,CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[0].latitude
					,uav_index + 1);
		}
		//任务航线
		else
		{
			//生成空域
			single_air_area(uav_index);
		}
		//发送空域信息
		send_airway_area();
	}
	//两架机考虑航线冲突
	if(integrated_postures.drone_num == 2)
	{
		//生成成功后判断航线是否有交叉
		FlightRoute route_uav1,route_uav2;
		route_uav1.start.latitude = integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.latitude;
		route_uav1.start.longitude = integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.longitude;
		route_uav2.start.latitude = integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.latitude;
		route_uav2.start.longitude = integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.longitude;
		route_uav1.end.latitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].planning_information_waypoint_informations[0].latitude;
		route_uav1.end.longitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].planning_information_waypoint_informations[0].longitude;
		route_uav2.end.latitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1].planning_information_waypoint_informations[0].latitude;
		route_uav2.end.longitude =
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1].planning_information_waypoint_informations[0].longitude;
		int rtn = 0;
		rtn = detectConflict(&route_uav1,&route_uav2);
		//有冲突
		if(rtn == 1 || (fabs(route_uav1.end.latitude  - route_uav2.end.latitude)  < 1e-7 &&
						fabs(route_uav1.end.longitude - route_uav2.end.longitude) < 1e-7))
		{
			//任务区交换
			information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number =
					area_sky_informations.area_informations[1].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[2].task_sequence_informations[0].target_number =
					area_sky_informations.area_informations[0].area_code;

			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].area_point_line_goal_number =
					area_sky_informations.area_informations[1].area_code;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1].area_point_line_goal_number =
					area_sky_informations.area_informations[0].area_code;
			//航线交换
			planning_information tmp;
			memcpy(&tmp,&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1],sizeof(planning_information));
			memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1],
					&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1],sizeof(planning_information));
			memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[1].planning_informations[1],&tmp,sizeof(planning_information));
		}
	}

	for(int i = 0 ; i < integrated_postures.drone_num ; i ++)
	{
		//航线发送,下标为1防止多加入无人机当前航路点
		send_drone_route_confirmation(i,1);
	}
	//任务区划分信息
	send_taskarea_result();
	//单任务区任务分配结果发送
	send_single_result();
	//解算完成反馈
	data_length = sizeof(int);
	int rtn = 9527;
	Send_Message(DDSTables.BLK_CTAS_DTMS_009.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0)
	{
		printf("fbzs jie suan success\n");
	}
}
void BDFX_init(SGeo UAV_aircraft, SGeo goal_point ,unsigned int uav_index)
{
	SGeo route_point[10];//生成的航线点集合
	unsigned short point_num = 0;//航线点数量
	double distance, bearing;
	calculate_distance_bearing(UAV_aircraft, goal_point, &distance, &bearing);

	// 计算1.5km处的点，集结点
	route_point[0] = calculate_new_coordinate(UAV_aircraft, 1.5, bearing);
	point_num++;

	// 单机编队航点结构与双机编队对齐：共6点（集结-过顶-保持-中点-保持-解散）
	double remain_distance = distance - 1.5;
	if(remain_distance < 0)
	{
		remain_distance = 0;
	}

	// 第二个点：过顶点（剩余航段的1/10处）
	route_point[1] = calculate_new_coordinate(route_point[0], remain_distance / 10, bearing);
	point_num++;

	// 第三个点：队形保持点（剩余航段的3/10处）
	route_point[2] = calculate_new_coordinate(route_point[0], remain_distance * 3 / 10, bearing);
	point_num++;

	// 第四个点：中点（剩余航段的1/2处）
	route_point[3] = calculate_new_coordinate(route_point[0], remain_distance / 2, bearing);
	point_num++;

	// 第五个点：队形保持点（剩余航段的7/8处）
	route_point[4] = calculate_new_coordinate(route_point[0], remain_distance * 7 / 8, bearing);
	point_num++;

	// 第六个点：解散点
	route_point[5] = goal_point;
	point_num++;

	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;

	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number = point_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].subtask_ID_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet = 1;

	for(int i = 0;i < point_num;i++){
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude = route_point[i].Lon;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude = route_point[i].Lat;
		//CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].height = blk_dtms_ctas_010[uav_index].BDFX;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].height = ((blk_dtms_ctas_002.lead_uav_id == drone_state_informations.drone_specific_informations[uav_index].platform_num) ? 460 : 660);
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].speed = 35;
		//集结点
		if(i == 0)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber
			= 200;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius
			= 1500;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 12;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_type = 2;//圈数
		}
		//队形变换点
		if(i == 1)
		{
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 13;

		}
		//解散点
		if(i == 5)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber
			= 200;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius
			= 1500;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 14;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_type = 2;//圈数
		}
	}
}

//双机编队
void BDFX_double_init(SGeo UAV_aircraft, SGeo goal_point1 ,SGeo goal_point2 ,unsigned int uav_index)
{
	SGeo route_point[10];//生成的航线点集合
	unsigned short point_num = 0;//航线点数量
	double distance, bearing;
	//计算集结到解散点的朝向和距离
	calculate_distance_bearing(goal_point1, goal_point2, &distance, &bearing);
	//第一个点盘旋点(集结点)
	route_point[0] = goal_point1;
	point_num++;
	//第二个点过顶点(一半距离的1/5处)
	route_point[1] = calculate_new_coordinate(goal_point1, distance/2*1/5, bearing);;
	point_num++;
	//第三个点队形保持点第一个点（一半距离的2/5处）
	route_point[2] = calculate_new_coordinate(goal_point1, distance/2*3/5, bearing);
	point_num++;
	//第四个点 中点
	route_point[3] = calculate_new_coordinate(goal_point1,distance/2, bearing);
	point_num++;
	//第五个点队形保持点第三个点，后半段等分改31开20260201
	route_point[4] = calculate_new_coordinate(goal_point1,distance*7/8, bearing);
	point_num++;
	//第六个点 集散点
	route_point[5] = goal_point2;
	point_num++;

	SGeo route_point_l[10];//僚机航线
	//计算方向
	bearing -= 90;
	if(bearing < 0)
		bearing += 360;
	//第一个点集结点
	int dist = 3;
	route_point_l[0] = calculate_new_coordinate(route_point[0], dist, bearing);
	//第二个点队形变换点(一半距离的1/5处)
	route_point_l[1] = calculate_new_coordinate(route_point[1], dist, bearing);
	//第三个点队形保持点第一个点（一半距离的2/5处）
	route_point_l[2] = calculate_new_coordinate(route_point[2], dist, bearing);
	//第四个点 中点
	route_point_l[3] = calculate_new_coordinate(route_point[3], dist, bearing);
	//第五个点队形保持点第三个点
	route_point_l[4] = calculate_new_coordinate(route_point[4], dist, bearing);
	//第六个点 集散点
	route_point_l[5] = calculate_new_coordinate(route_point[5], dist, bearing);


	if(blk_dtms_ctas_002.lead_uav_id != drone_state_informations.drone_specific_informations[uav_index].platform_num)
	{
		memcpy(&route_point,&route_point_l,sizeof(route_point));
	}

	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;

	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number = point_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].subtask_ID_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet = 1;

	// 260201: 更改双机编队飞行高度，长机460，僚机660
//	unsigned short bdfx_height = (blk_dtms_ctas_002.lead_uav_id == drone_state_informations.drone_specific_informations[uav_index].platform_num) ? 460 : 660;
	for(int i = 0;i < point_num;i++){
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude = route_point[i].Lon;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude = route_point[i].Lat;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].height = blk_dtms_ctas_010[uav_index].BDFX;
		//CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].height = bdfx_height; //260201
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].speed = 35;
		//集结点
		if(i == 0)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber
			= 20;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius
			= 1000;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 12;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_type = 2;//圈数
		}
		//队形变换点
		if(i == 1)
		{
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 13;

		}
		//解散点
		if(i == 5)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber
			= 20;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius
			= 1000;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 14;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_type = 2;//圈数
		}
	}
}

// 将角度转换为弧度
double to_radians(double degree) {
	return degree * M_PI / 180.0;
}

// 计算两点间距离和方位角
void calculate_distance_bearing(SGeo start, SGeo end, double *distance, double *bearing) {
	double dlon = to_radians(end.Lon - start.Lon);
	double dlat = to_radians(end.Lat - start.Lat);

	double a = sin(dlat/2) * sin(dlat/2) + cos(to_radians(start.Lat)) *
			cos(to_radians(end.Lat)) * sin(dlon/2) * sin(dlon/2);
	*distance = 2 * 6371.0 * asin(sqrt(a));

	double y = sin(dlon) * cos(to_radians(end.Lat));
	double x = cos(to_radians(start.Lat)) * sin(to_radians(end.Lat)) -
			sin(to_radians(start.Lat)) * cos(to_radians(end.Lat)) * cos(dlon);
	*bearing = fmod(atan2(y, x) * 180.0 / M_PI + 360.0, 360.0);
}

// 计算指定距离和方位角的新坐标
SGeo calculate_new_coordinate(SGeo origin, double distance, double bearing) {

	double rad_bearing;
	double lat1,lon1,lat2,lon2;

	rad_bearing = to_radians(bearing);

	lat1 = to_radians(origin.Lat);
	lon1 = to_radians(origin.Lon);

	lat2 = asin(sin(lat1) * cos(distance/6371.0) +
			cos(lat1) * sin(distance/6371.0) * cos(rad_bearing));

	lon2 = lon1 + atan2(sin(rad_bearing) * sin(distance/6371.0) * cos(lat1),
			cos(distance/6371.0) - sin(lat1) * sin(lat2));

	SGeo rtn;
	rtn.Lon = fmod(lon2 * 180.0 / M_PI + 360.0, 360.0);
	rtn.Lat = fmod(lat2 * 180.0 / M_PI + 360.0, 360.0);

	return rtn;
}

void single_uav_BDFX()
{
	//将收到的战术战法指令赋值给规划所需使用的任务区、有无人机等数据
	init_zhanshutuijians_single();
	//初始化无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));
	//找到无人机索引
	unsigned int uav_index = 0;
	for(unsigned int i = 0 ; i < 4 ; i ++)
	{
		if(drone_state_informations.drone_specific_informations[i].platform_num == blk_dtms_ctas_002.uav_id)
		{
			uav_index = i;
			break;
		}
		uav_index = 44;
	}
	//索引异常
	if(uav_index > 4)
	{
		int area_rtn = -6;
		data_length = sizeof(int);
		Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
		return;
	}
	//方案编号获取
	if(blk_dtms_ctas_002.planning_id == 0)
	{
		//无运行方案，首次进行单无人机指控
		information_on_the_results_of_taskings.program_number = 3;//3为假定值
	}
	else
	{
		information_on_the_results_of_taskings.program_number = blk_dtms_ctas_002.planning_id;
	}
	information_on_the_results_of_taskings.tasking_release = 3;//任务分配方案发布---修改
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

	information_on_the_results_of_taskings.number_of_mission_platforms = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_model = 2;//平台型号--无人机

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_serial_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;//平台序号---无人机i

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_num;//平台编号---无

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].number_of_subtasks = 1;//任务数量为1

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].sequence_type = \
			blk_dtms_ctas_002.task_type;//任务类型
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].modify = 2;//任务序列变更
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位
	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].mission_type = blk_dtms_ctas_002.task_type;//编队飞行
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].point_area_type = 1;/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].area_point_line_goal_number = area_sky_informations.area_informations[0].area_code;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;


	//生成航线
	SGeo UAV_aircraft;
	SGeo goal_point;
	//无人机位置
	UAV_aircraft.Lon = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.longitude;
	UAV_aircraft.Lat = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.latitude;
	//目标位置
	goal_point.Lon = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude;
	goal_point.Lat = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude;

	double distance = 0;
	distance = calculate_distances(UAV_aircraft.Lat,UAV_aircraft.Lon,goal_point.Lat,goal_point.Lon);
	//小于十千米生成失败
	if(distance < 10)
	{
		int rtn = -5;
		Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
		if(enRetCode == 0)
		{
			printf("BDFX erro\n");
			return;
		}
	}
	//编队飞行算法
	BDFX_init(UAV_aircraft,goal_point,uav_index);
#if AIRWAY_LIMIT
	//空域许可范围
	int area_rtn = 0;
	for(int i = 0 ; i < CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number ; i++ )
	{
		area_rtn = airway_area_confirm(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude,
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude);
		if(area_rtn < 0)
		{
			//任务目标点超过许可空域范围 -1 ; 任务目标点距离许可空域边界过近 -2
			if(area_rtn == -1 || area_rtn == -2)
			{
				//异常反馈
				data_length = sizeof(int);
				Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
				if(enRetCode == 0)
				{
					printf("airway_area erro\n");
					return;
				}
			}
		}
	}
#endif
//	//生成空域
//	single_air_area(uav_index);
//	//发送空域信息
//	send_airway_area();
	//航线发送,下标为1防止多加入无人机当前航路点
	send_drone_route_confirmation(uav_index,1);
	//单任务区任务分配结果发送
	send_single_result();

}
//双机编队
int double_uav_BDFX()
{
	//将收到的战术战法指令赋值给规划所需使用的任务区、有无人机等数据
	init_zhanshutuijians_single();
	//初始化无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));

	for(unsigned int uav_index = 0 ; uav_index < drone_state_informations.drone_number ; uav_index ++)
	{
		//方案编号获取
		if(blk_dtms_ctas_002.planning_id == 0)
		{
			//无运行方案，首次进行单无人机指控
			information_on_the_results_of_taskings.program_number = 3;//3为假定值
		}
		else
		{
			information_on_the_results_of_taskings.program_number = blk_dtms_ctas_002.planning_id;
		}
		//260129
			printf("[CTAS][BDFX] uav_index=%u planning_id=%u -> program_number=%u task_type=%u\n",
			            (unsigned int)uav_index,
			            (unsigned int)blk_dtms_ctas_002.planning_id,
		                (unsigned int)information_on_the_results_of_taskings.program_number,
			            (unsigned int)blk_dtms_ctas_002.task_type);
		information_on_the_results_of_taskings.tasking_release = 6;//任务分配方案发布---修改
		information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
		information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
		information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
		information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

		information_on_the_results_of_taskings.number_of_mission_platforms = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_model = 2;//平台型号--无人机

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_serial_number = \
				drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;//平台序号---无人机i

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_number = \
				drone_state_informations.drone_specific_informations[uav_index].platform_num;//平台编号---无

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].number_of_subtasks = 1;//任务数量为1

		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].sequence_type = \
				blk_dtms_ctas_002.task_type;//任务类型
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].modify = 2;//任务序列变更
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位
		//赋值航路信息
		CTAS_DTMS_data_UAVRoute.program_number = information_on_the_results_of_taskings.program_number;//20260129
		CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
		CTAS_DTMS_data_UAVRoute.plan_type = 4;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].mission_type = blk_dtms_ctas_002.task_type;//编队飞行
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].point_area_type = 1;/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].area_point_line_goal_number = area_sky_informations.area_informations[0].area_code;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;


		//生成航线
		SGeo UAV_aircraft;
		SGeo goal_point1;
		SGeo goal_point2;
		//无人机位置
		UAV_aircraft.Lon = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.longitude;
		UAV_aircraft.Lat = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.latitude;
		//目标1位置,集结点
		goal_point1.Lon = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude;
		goal_point1.Lat = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude;
		//目标2位置,解散点
		goal_point2.Lon = blk_dtms_ctas_002.region_infos[0].point_infos[1].point_lon_lat.longitude;
		goal_point2.Lat = blk_dtms_ctas_002.region_infos[0].point_infos[1].point_lon_lat.latitude;
		//260129虚拟点
//		goal_point2.Lon = 108.39965000;
//		goal_point2.Lat = 19.30761000;
		double distance = 0;
		//260129
		if(uav_index == 0){
		            printf("[CTAS][BDFX] A(lat,lon)=(%.8f,%.8f) B(lat,lon)=(%.8f,%.8f)\n",
		               goal_point1.Lat, goal_point1.Lon, goal_point2.Lat, goal_point2.Lon);
		       }
		distance = calculate_distances(goal_point1.Lat,goal_point1.Lon,goal_point2.Lat,goal_point2.Lon);
		//小于50千米生成失败
		if(distance < 50)
		{
			int rtn = -6;
			message_type_id = 0;
			data_length = sizeof(int);
			Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
			printf("BDFX erro distance<50\n");
			return -6;
		}
		double dist_start_to_assembly = calculate_distances(goal_point1.Lat,goal_point1.Lon,UAV_aircraft.Lat,UAV_aircraft.Lon);
		//无人机位置与集结点位置小于2km,260129
		if(dist_start_to_assembly < 2)
		{
			int rtn = -7;
			message_type_id = 0;
			data_length = sizeof(int);
			Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
			printf("BDFX erro dist<2km\n");
			return -7;
		}
		//无人机位置与集结点位置大于15km,260131
		if(dist_start_to_assembly > 15)
		{
			int rtn = -8;
			message_type_id = 0;
			data_length = sizeof(int);
			Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &rtn, &message_type_id, data_length, &enRetCode);
			printf("BDFX erro dist>15km\n");
			return -8;
		}
		//编队飞行算法
		BDFX_double_init(UAV_aircraft,goal_point1,goal_point2,uav_index);
#if AIRWAY_LIMIT
	//空域许可范围
	int area_rtn = 0;
	for(int i = 0 ; i < CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number ; i++ )
	{
		area_rtn = airway_area_confirm(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude,
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude);
		if(area_rtn < 0)
		{
			//任务目标点超过许可空域范围 -1 ; 任务目标点距离许可空域边界过近 -2
			if(area_rtn == -1 || area_rtn == -2)
			{
				//异常反馈
				data_length = sizeof(int);
				Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
				printf("airway_area erro ret=%d\n", area_rtn);
				return area_rtn;
			}
		}
	}
#endif
//		//生成空域
//		single_air_area(uav_index);
//		//发送空域信息
//		send_airway_area();
	//260128
		//航线发送,下标为1防止多加入无人机当前航路点
//		send_drone_route_confirmation(uav_index,1);
	}
	//单任务区任务分配结果发送
//	send_single_result();

	//260129
	//单任务区任务分配结果发送,先002
	send_single_result();
	//再按机发004
	for(unsigned int uav_index = 0;uav_index < drone_state_informations.drone_number ; uav_index++){
		//航线发送,下标为1防止多加入无人机当前航路点
		send_drone_route_confirmation(uav_index,1);
	}
	return 0;
}
//应召磁探跟踪
void yz_CTGZ()
{
	SGeo UAV_aircraft;
	SGeo goal_point;
	float speed;
	unsigned int uav_index;
	//找到做磁探跟踪的无人机
	for(int uav = 0 ; uav < 2 ; uav ++)
	{
		if(information_on_the_results_of_taskings.formation_synergy_mission_programs[uav+1].task_sequence_informations[0].sequence_type == 6)
		{
			UAV_aircraft.Lat = integrated_postures.integrated_posture_drone_informations[uav].drone_longitude_and_latitude.latitude;
			UAV_aircraft.Lon = integrated_postures.integrated_posture_drone_informations[uav].drone_longitude_and_latitude.longitude;
			speed = integrated_postures.integrated_posture_drone_informations[uav].drone_speed;
			uav_index = uav;
		}
	}
	goal_point.Lat = blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.latitude;
	goal_point.Lon = blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.longitude;

	SGeo route_point[25];//生成的航线点集合
	unsigned short point_num = 0;//航线点数量
	//加入目标初始点
	route_point[0].Lon = goal_point.Lon;
	route_point[0].Lat = goal_point.Lat;
	point_num++;

	//航向有效
	if(blk_dtms_ctas_001.region_infos[0].point_infos[0].point_hangxiang_valid == 1)
	{
		float goal_vel = 0;
		float goal_course = blk_dtms_ctas_001.region_infos[0].point_infos[0].target_hangxiang; //目标航向 若没有的话就按正常的8字搜，不做目标点位置的偏移
		//航速有效
		if(blk_dtms_ctas_001.region_infos[0].point_infos[0].point_speed_valid == 1)
		{
			goal_vel = blk_dtms_ctas_001.region_infos[0].point_infos[0].target_speed;
		}
		else
		{
			goal_vel = 6;//目标航速  若没有则默认六节
		}
		//定义变量
		SPolar SPgoal1;
		SPolar SProute1[4];
		SGeo SPgoal_point1[3];
		//计算无人机到目标点的预计时间
		float time_to_target = 0;
		if(speed > 0)
		{
			double distance = calculate_distances(UAV_aircraft.Lat,UAV_aircraft.Lon,goal_point.Lat,goal_point.Lon);
			time_to_target = distance/speed;
		}
		else
		{
			time_to_target = 0;
		}
		//计算未来目标经过的点
		for(int i = 0 ; i < 3 ; i ++)
		{
			//求出当前循环中目标点的预测位置
			SPgoal1.R = (((4.5/60)*i + time_to_target)* goal_vel);
			SPgoal1.A = 90 - goal_course;
			SPgoal_point1[i] = Polar_to_Geo(goal_point, SPgoal1);
		}
		//计算航线
		for(int i = 0; i < 12 ; i++)
		{
			int j = i % 4;
			int n = i/4; // 求是第几个循环
			SProute1[j].R = 3;
			switch (i)
			{
			case 0:
				SProute1[j].A = goal_course + 180;
				break;
			case 1:
				SProute1[j].A = goal_course + 240;
				break;
			case 2:
				SProute1[j].A = goal_course + 60;
				break;
			case 3:
				SProute1[j].A = goal_course;
				break;
			}
			route_point[i + 1] = Polar_to_Geo(SPgoal_point1[n], SProute1[j]);
			point_num++;
		}

	}
	else//航向无效，直接做8字搜索
	{
		SPolar SPgoal_point;//目标点极坐标
		SPolar SProute1[4];//所求航线点极坐标
		//若不存在目标航向
		SPgoal_point = Geo_to_Polar( goal_point , UAV_aircraft );
		//第一个航线点是无人机到目标点的延伸3km
		SProute1[0].R = 3;
		SProute1[0].A = SPgoal_point.A + 180;
		//第二个航线点是无人机到目标点的延伸
		SProute1[1].R = 3;
		SProute1[1].A = SPgoal_point.A + 240;
		//第三个航线点是无人机到目标点的延伸
		SProute1[2].R = 3;
		SProute1[2].A = SPgoal_point.A + 60;
		//第四个航线点是无人机到目标点的延伸
		SProute1[3].R = 3;
		SProute1[3].A = SPgoal_point.A;
		for(int i = 0; i < 12 ; i++)
		{
			int j = i % 4;
			//由极坐标求出第二个点的经纬度
			route_point[i + 1] = Polar_to_Geo(goal_point,SProute1[j]);
			point_num++;
		}
	}

	//增加最后一个矩形的中心点作为盘旋点 20251021new
	AreaRectVertex tmp;
	AreaRectCenter rtn;
	for(int k = 0; k < 4 ; k ++)
	{
		tmp.vertexA[k].latitude = route_point[k+9].Lat;
		tmp.vertexA[k].longitude = route_point[k+9].Lon;
	}
	//生成矩形中心点
	rtn = getAreaRectCenterByVertex(&tmp);

	route_point[13].Lat = rtn.center.latitude;
	route_point[13].Lon = rtn.center.longitude;

	point_num++;




	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.program_number = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].mission_type = 6;//磁探跟踪
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].AtomicHighlyUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].total_packet = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].waypoints_number = point_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].subtask_ID_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].AtomicHighlyUpper = 1;

	for(int i = 0;i < point_num;i++){
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].causality = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].longitude = route_point[i].Lon;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].latitude = route_point[i].Lat;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].height = blk_dtms_ctas_010[uav_index].CTGZ;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].speed = 30;
		//最后一个点为盘旋点
		if(i == point_num - 1)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber
			= PX_CIRCLE;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].standby_radius
			= PX_RAD;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].causality = 3;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].standby_type = 2;//圈数
		}
	}
	//另一架机做盘旋
	if(integrated_postures.drone_num == 2)
	{
		int px_uav = 0;
		px_uav = (uav_index == 0) ? 1:0;
		//目标航向反方向5km的点计算
		double bearing;
		SGeo px_point;
		bearing = blk_dtms_ctas_001.region_infos[0].point_infos[0].target_hangxiang + 180;
		if(bearing > 360)
			bearing -= 360;
		px_point = calculate_new_coordinate(goal_point,5.0,bearing);
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].subtasks_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].drone_serial_number = drone_state_informations.drone_specific_informations[px_uav].platform_serial_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].drone_num = drone_state_informations.drone_specific_informations[px_uav].platform_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].mission_type = 14;//盘旋等待
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].AtomicTimeUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].AtomicHighlyUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].total_packet = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].waypoints_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].subtask_ID_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].AtomicTimeUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].AtomicHighlyUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].longitude = px_point.Lon;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].latitude = px_point.Lat;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].height = blk_dtms_ctas_010[uav_index].CTGZ;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].speed = 30;
		//航路点待机时间/圈数/循环次数
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber
		= PX_CIRCLE;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
		//航路点待机半径
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].standby_radius
		= PX_RAD;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].causality = 3;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].standby_type = 2;//圈数
	}
}
void CTGZ_init(SGeo UAV_aircraft, float speed,SGeo goal_point ,unsigned int uav_index)
{

	SGeo route_point[25];//生成的航线点集合
	unsigned short point_num = 0;//航线点数量
	//加入目标初始点
	route_point[0].Lon = goal_point.Lon;
	route_point[0].Lat = goal_point.Lat;
	point_num++;

	//航向有效
	if(blk_dtms_ctas_002.region_infos[0].point_infos[0].point_hangxiang_valid == 1)
	{
		float goal_vel = 0;
		float goal_course = blk_dtms_ctas_002.region_infos[0].point_infos[0].target_hangxiang; //目标航向 若没有的话就按正常的8字搜，不做目标点位置的偏移
		//航速有效
		if(blk_dtms_ctas_002.region_infos[0].point_infos[0].point_speed_valid == 1)
		{
			goal_vel = blk_dtms_ctas_002.region_infos[0].point_infos[0].target_speed;
		}
		else
		{
			goal_vel = 6;//目标航速  若没有则默认六节
		}
		//定义变量
		SPolar SPgoal1;
		SPolar SProute1[4];
		SGeo SPgoal_point1[3];
		//计算无人机到目标点的预计时间
		float time_to_target = 0;
		//速度小于100，用常规速度
		if(speed < 100)
			speed = 128;
		if(speed > 0)
		{
			double distance = calculate_distances(UAV_aircraft.Lat,UAV_aircraft.Lon,goal_point.Lat,goal_point.Lon);
			time_to_target = distance/speed;
		}
		else
		{
			time_to_target = 0;
		}
		//计算未来目标经过的点
		for(int i = 0 ; i < 3 ; i ++)
		{
			//求出当前循环中目标点的预测位置
			SPgoal1.R = (((4.5/60)*i + time_to_target)* goal_vel);
			SPgoal1.A = 90 - goal_course;
			SPgoal_point1[i] = Polar_to_Geo(goal_point, SPgoal1);
		}
		//计算航线
		for(int i = 0; i < 12 ; i++)
		{
			int j = i % 4;
			int n = i/4; // 求是第几个循环
			SProute1[j].R = 3;
			switch (i)
			{
			case 0:
				SProute1[j].A = (90 - goal_course) + 180;
				break;
			case 1:
				SProute1[j].A = (90 - goal_course) + 240;
				break;
			case 2:
				SProute1[j].A = (90 - goal_course) + 60;
				break;
			case 3:
				SProute1[j].A = (90 - goal_course);
				break;
			}
			route_point[i + 1] = Polar_to_Geo(SPgoal_point1[n], SProute1[j]);
			point_num++;
		}

	}
	else//航向无效，直接做8字搜索
	{
		SPolar SPgoal_point;//目标点极坐标
		SPolar SProute1[4];//所求航线点极坐标
		//若不存在目标航向
		SPgoal_point = Geo_to_Polar( goal_point , UAV_aircraft );
		//第一个航线点是无人机到目标点的延伸3km
		SProute1[0].R = 3;
		SProute1[0].A = SPgoal_point.A + 180;
		//第二个航线点是无人机到目标点的延伸
		SProute1[1].R = 3;
		SProute1[1].A = SPgoal_point.A + 240;
		//第三个航线点是无人机到目标点的延伸
		SProute1[2].R = 3;
		SProute1[2].A = SPgoal_point.A + 60;
		//第四个航线点是无人机到目标点的延伸
		SProute1[3].R = 3;
		SProute1[3].A = SPgoal_point.A;
		for(int i = 0; i < 12 ; i++)
		{
			int j = i % 4;
			//由极坐标求出第二个点的经纬度
			route_point[i + 1] = Polar_to_Geo(goal_point,SProute1[j]);
			point_num++;
		}
	}

	//增加最后一个矩形的中心点作为盘旋点 20251021new
	AreaRectVertex tmp;
	AreaRectCenter rtn;
	for(int k = 0; k < 4 ; k ++)
	{
		tmp.vertexA[k].latitude = route_point[k+9].Lat;
		tmp.vertexA[k].longitude = route_point[k+9].Lon;
	}
	//生成矩形中心点
	rtn = getAreaRectCenterByVertex(&tmp);

	route_point[13].Lat = rtn.center.latitude;
	route_point[13].Lon = rtn.center.longitude;

	point_num++;

	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;

	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number = point_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].subtask_ID_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;

	for(int i = 0;i < point_num;i++){
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude = route_point[i].Lon;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude = route_point[i].Lat;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].height = blk_dtms_ctas_010[uav_index].CTGZ;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].speed = 30;
		//最后一个点为盘旋点
		if(i == point_num - 1)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber
			= PX_CIRCLE;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius
			= PX_RAD;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 3;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_type = 2;//圈数
		}
	}
}
void single_uav_CTGZ()
{
	//将收到的战术战法指令赋值给规划所需使用的任务区、有无人机等数据
	init_zhanshutuijians_single();
	//初始化无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));
	//找到无人机索引
	unsigned int uav_index = 0;
	for(unsigned int i = 0 ; i < 4 ; i ++)
	{
		if(drone_state_informations.drone_specific_informations[i].platform_num == blk_dtms_ctas_002.uav_id)
		{
			uav_index = i;
			break;
		}
		uav_index = 44;
	}
	//索引异常
	if(uav_index > 4)
	{
		int area_rtn = -6;
		data_length = sizeof(int);
		Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
		return;
	}
	//方案编号获取
	if(blk_dtms_ctas_002.planning_id == 0)
	{
		//无运行方案，首次进行单无人机指控
		information_on_the_results_of_taskings.program_number = 3;//3为假定值
	}
	else
	{
		information_on_the_results_of_taskings.program_number = blk_dtms_ctas_002.planning_id;
	}
	information_on_the_results_of_taskings.tasking_release = 3;//任务分配方案发布---修改
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

	information_on_the_results_of_taskings.number_of_mission_platforms = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_model = 2;//平台型号--无人机

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_serial_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;//平台序号---无人机i

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_num;//平台编号---无

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].number_of_subtasks = 1;//任务数量为1

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].sequence_type = \
			blk_dtms_ctas_002.task_type;//任务类型
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].modify = 2;//任务序列变更
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位
	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].mission_type = blk_dtms_ctas_002.task_type;//磁探跟踪
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].point_area_type = 1;/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].area_point_line_goal_number = area_sky_informations.area_informations[0].area_code;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet = 1;
	//生成航线
	SGeo UAV_aircraft;
	SGeo goal_point;
	//无人机位置
	UAV_aircraft.Lon = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.longitude;
	UAV_aircraft.Lat = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.latitude;
	//目标位置
	goal_point.Lon = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude;
	goal_point.Lat = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude;
	//无人机地速km/h
	float speed = integrated_postures.integrated_posture_drone_informations[uav_index].drone_speed;
	printf("CTGZ goal_vel = %f goal_course = %f \n",blk_dtms_ctas_002.region_infos[0].point_infos[0].target_speed,
			blk_dtms_ctas_002.region_infos[0].point_infos[0].target_hangxiang);
	//磁探跟踪航路规划
	CTGZ_init(UAV_aircraft,speed,goal_point,uav_index);
#if AIRWAY_LIMIT
	//空域许可范围
	int area_rtn = 0;
	for(int i = 0 ; i < CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number ; i++ )
	{
		area_rtn = airway_area_confirm(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude,
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude);
		if(area_rtn < 0)
		{
			//任务目标点超过许可空域范围 -1 ; 任务目标点距离许可空域边界过近 -2
			if(area_rtn == -1 || area_rtn == -2)
			{
				//异常反馈
				data_length = sizeof(int);
				Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
				if(enRetCode == 0)
				{
					printf("airway_area erro\n");
					return;
				}
			}
		}
	}
#endif
	//生成空域
	single_air_area(uav_index);
	//发送空域信息
	send_airway_area();
	//航线发送,下标为1防止多加入无人机当前航路点
	send_drone_route_confirmation(uav_index,1);
	//单任务区任务分配结果发送
	send_single_result();
	//    //异常反馈,无异常为0
	//	data_length = sizeof(int);
	//	Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
}
void single_uav_CTSS()
{
	//将收到的战术战法指令赋值给规划所需使用的任务区、有无人机等数据
	init_zhanshutuijians_single();
	//初始化无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));
	//找到无人机索引
	unsigned int uav_index = 0;
	for(unsigned int i = 0 ; i < 4 ; i ++)
	{
		if(drone_state_informations.drone_specific_informations[i].platform_num == blk_dtms_ctas_002.uav_id)
		{
			uav_index = i;
			break;
		}
		uav_index = 44;
	}
	//索引异常
	if(uav_index > 4)
	{
		int area_rtn = -6;
		data_length = sizeof(int);
		Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
		return;
	}
	//方案编号获取
	if(blk_dtms_ctas_002.planning_id == 0)
	{
		//无运行方案，首次进行单无人机指控
		information_on_the_results_of_taskings.program_number = 3;//3为假定值
	}
	else
	{
		information_on_the_results_of_taskings.program_number = blk_dtms_ctas_002.planning_id;
	}
	information_on_the_results_of_taskings.tasking_release = 3;//任务分配方案发布---修改
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

	information_on_the_results_of_taskings.number_of_mission_platforms = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_model = 2;//平台型号--无人机

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_serial_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;//平台序号---无人机i

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_num;//平台编号---无

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].number_of_subtasks = 1;//任务数量为1

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].sequence_type = \
			blk_dtms_ctas_002.task_type;//任务类型
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].modify = 2;//任务序列变更
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位
	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].mission_type = blk_dtms_ctas_002.task_type;//磁探搜索
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].point_area_type = 3;/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].area_point_line_goal_number = area_sky_informations.area_informations[0].area_code;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
	//找到无人机位置
	GeoLibDas UAV_position;
	UAV_position.longitude = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.longitude;
	UAV_position.latitude = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.latitude;
	//subtask:磁探搜索(光电搜索暂时替代)
	OutSearchRadarPhoto outSearchRadarPhoto;
	outSearchRadarPhoto = commonRouteGeneration(area_sky_informations.area_informations[0],
												getNearestPolygonVertexForUav(UAV_position, area_sky_informations.area_informations[0]),1000);

	//最多75个点 20250611new
	if(outSearchRadarPhoto.sumAwp > 75) outSearchRadarPhoto.sumAwp = 75;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number = outSearchRadarPhoto.sumAwp;

	//计算航路信息包数 20250611new
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet = outSearchRadarPhoto.sumAwp/25;
	if(outSearchRadarPhoto.sumAwp%25)
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet++;

	for(int k = 0;k < outSearchRadarPhoto.sumAwp;k++){
		//计算航路点编号 20250611new
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].hld_idx = k+1;
		//        CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].type = blk_dtms_ctas_002.task_type;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].longitude = outSearchRadarPhoto.awpA[k].longitude;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].latitude = outSearchRadarPhoto.awpA[k].latitude;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].validity_of_longitude = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].latitude_validity = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].height_validity = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].speed_validity = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].direction_validity = 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].time_validity = 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].payloads_validity = 1;
		//高度---待补充
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].height = blk_dtms_ctas_010[uav_index].CTSS;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].speed = 27;
		//速度---待补充
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].direction = 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].time = 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].payloads = 2;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].causality = 1;
		//最后一个点为盘旋点
		if(k == outSearchRadarPhoto.sumAwp - 1)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber
			= PX_CIRCLE;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_radius
			= PX_RAD;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].causality = 3;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_type = 2;//圈数
		}
	}
#if AIRWAY_LIMIT
	//空域许可范围
	int area_rtn = 0;
	for(int i = 0 ; i < CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number ; i++ )
	{
		area_rtn = airway_area_confirm(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude,
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude);
		if(area_rtn < 0)
		{
			//任务目标点超过许可空域范围 -1 ; 任务目标点距离许可空域边界过近 -2
			if(area_rtn == -1 || area_rtn == -2)
			{
				//异常反馈
				data_length = sizeof(int);
				Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
				if(enRetCode == 0)
				{
					printf("airway_area erro\n");
					return;
				}
			}
		}
	}
#endif
	//生成空域
	single_air_area(uav_index);
	//发送空域信息
	send_airway_area();
	//航线发送,下标为1防止多加入无人机当前航路点
	send_drone_route_confirmation(uav_index,1);
	//单任务区任务分配结果发送
	send_single_result();
	//    //异常反馈,无异常为1
	//	data_length = sizeof(int);
	//	Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);

}
void single_uav_GDSS()
{
	//将收到的战术战法指令赋值给规划所需使用的任务区、有无人机等数据
	init_zhanshutuijians_single();
	//初始化无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));
	//找到无人机索引
	unsigned int uav_index = 0;
	for(unsigned int i = 0 ; i < 4 ; i ++)
	{
		if(drone_state_informations.drone_specific_informations[i].platform_num == blk_dtms_ctas_002.uav_id)
		{
			uav_index = i;
			break;
		}
		uav_index = 44;
	}
	//索引异常
	if(uav_index > 4)
	{
		int area_rtn = -6;
		data_length = sizeof(int);
		Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
		return;
	}
	//方案编号获取
	if(blk_dtms_ctas_002.planning_id == 0)
	{
		//无运行方案，首次进行单无人机指控
		information_on_the_results_of_taskings.program_number = 3;//3为假定值
	}
	else
	{
		information_on_the_results_of_taskings.program_number = blk_dtms_ctas_002.planning_id;
	}
	information_on_the_results_of_taskings.tasking_release = 3;//任务分配方案发布---修改
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

	information_on_the_results_of_taskings.number_of_mission_platforms = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_model = 2;//平台型号--无人机

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_serial_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;//平台序号---无人机i

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_num;//平台编号---无

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].number_of_subtasks = 1;//任务数量为1

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].sequence_type = \
			blk_dtms_ctas_002.task_type;//任务类型
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].modify = 2;//任务序列变更
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位
	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].mission_type = blk_dtms_ctas_002.task_type;//光电搜索
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].point_area_type = 3;/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].area_point_line_goal_number = area_sky_informations.area_informations[0].area_code;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
	//找到无人机位置
	GeoLibDas UAV_position;
	UAV_position.longitude = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.longitude;
	UAV_position.latitude = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.latitude;
	//subtask:光电搜索
	OutSearchRadarPhoto outSearchRadarPhoto;
	outSearchRadarPhoto = commonRouteGeneration(area_sky_informations.area_informations[0],
												getNearestPolygonVertexForUav(UAV_position, area_sky_informations.area_informations[0]),12000);

	//最多75个点 20250611new
	if(outSearchRadarPhoto.sumAwp > 75) outSearchRadarPhoto.sumAwp = 75;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number = outSearchRadarPhoto.sumAwp;

	//计算航路信息包数 20250611new
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet = outSearchRadarPhoto.sumAwp/25;
	if(outSearchRadarPhoto.sumAwp%25)
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet++;

	for(int k = 0;k < outSearchRadarPhoto.sumAwp;k++){
		//计算航路点编号 20250611new
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].hld_idx = k+1;
		//        CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].type = blk_dtms_ctas_002.task_type;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].longitude = outSearchRadarPhoto.awpA[k].longitude;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].latitude = outSearchRadarPhoto.awpA[k].latitude;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].validity_of_longitude = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].latitude_validity = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].height_validity = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].speed_validity = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].direction_validity = 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].time_validity = 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].payloads_validity = 1;
		//高度---待补充
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].height = blk_dtms_ctas_010[uav_index].GDSS;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].speed = 35;
		//速度---待补充
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].direction = 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].time = 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].payloads = 2;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].causality = 1;
		//最后一个点为盘旋点
		if(k == outSearchRadarPhoto.sumAwp - 1)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber
			= PX_CIRCLE;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_radius
			= PX_RAD;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].causality = 3;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[k].standby_type = 2;//圈数
		}
	}
#if AIRWAY_LIMIT
	//空域许可范围
	int area_rtn = 0;
	for(int i = 0 ; i < CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number ; i++ )
	{
		area_rtn = airway_area_confirm(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude,
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude);
		if(area_rtn < 0)
		{
			//任务目标点超过许可空域范围 -1 ; 任务目标点距离许可空域边界过近 -2
			if(area_rtn == -1 || area_rtn == -2)
			{
				//异常反馈
				data_length = sizeof(int);
				Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
				if(enRetCode == 0)
				{
					printf("airway_area erro\n");
					return;
				}
			}
		}
	}
#endif
	//生成空域
	single_air_area(uav_index);
	//发送空域信息
	send_airway_area();
	//航线发送,下标为1防止多加入无人机当前航路点
	send_drone_route_confirmation(uav_index,1);
	//单任务区任务分配结果发送
	send_single_result();
	//    //异常反馈,无异常为1
	//	data_length = sizeof(int);
	//	Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);

}

void calculate_rectangle_vertices(SGeo in,double azimuth,double side,SGeo*out)
{
	SPolar temp;
	temp.R = side / sqrt(2);
	temp.A = - azimuth + 90;
	for (int n = 0;  n < 4 ; n++)
	{
		while (temp.A <= -180.0)
		{
			temp.A = temp.A+ 360.0;
		}
		while (temp.A >= 180.0)
		{
			temp.A = temp.A - 360.0;
		}
		out[n] = Polar_to_Geo(in,temp);
		temp.A = temp.A + 90;
	}



	//	//转换角度为弧度制
	//	double azimuth_rad = azimuth * M_PI / 180.0;
	//	//计算半边长
	//	double half_side = side / 2.0;
	//	//目标点转换为弧度
	//	double lat_rad = in.Lat * M_PI / 180.0;
	//	double cos_lat = cos(lat_rad);
	//
	//	//沿方位角的位移分量
	//	double delta_lat = (half_side * cos(azimuth_rad)) / 6371.0;
	//	double delta_lon = (half_side * sin(azimuth_rad)) / (6371.0 * cos_lat);
	//
	//	//沿正交方向（方位角 + 90°）的位移分量
	//	double ortho_azimuth_rad = (azimuth + 90) * M_PI / 180.0;
	//	double delta_lat_ortho = (half_side * cos(ortho_azimuth_rad)) / 6371.0;
	//	double delta_lon_ortho = (half_side * sin(ortho_azimuth_rad)) / (6371.0 * cos_lat);
	//
	//	//转换为度数并计算顶点坐标
	//	out[0].Lat = in.Lat + delta_lat * 180.0 / M_PI + delta_lat_ortho * 180.0 / M_PI;
	//	out[0].Lon = in.Lon + (delta_lon + delta_lon_ortho) * 180.0 / M_PI / cos_lat;
	//
	//	out[1].Lat = in.Lat + delta_lat * 180.0 / M_PI - delta_lat_ortho * 180.0 / M_PI;
	//	out[1].Lon = in.Lon + (delta_lon - delta_lon_ortho) * 180.0 / M_PI / cos_lat;
	//
	//	out[2].Lat = in.Lat - delta_lat * 180.0 / M_PI - delta_lat_ortho * 180.0 / M_PI;
	//	out[2].Lon = in.Lon - (delta_lon + delta_lon_ortho) * 180.0 / M_PI / cos_lat;
	//
	//	out[3].Lat = in.Lat - delta_lat * 180.0 / M_PI + delta_lat_ortho * 180.0 / M_PI;
	//	out[3].Lon = in.Lon - (delta_lon - delta_lon_ortho) * 180.0 / M_PI / cos_lat;

}

void calculate_rectangle_vertices_new(SGeo in,double azimuth,double side,SGeo*out)
{
	SPolar temp;
	temp.R = side / sqrt(2);
	temp.A = azimuth + 45;
	for (int n = 0;  n < 4 ; n++)
	{
		while (temp.A <= -180.0)
		{
			temp.A = temp.A+ 360.0;
		}
		while (temp.A >= 180.0)
		{
			temp.A = temp.A - 360.0;
		}
		out[n] = Polar_to_Geo(in,temp);
		temp.A = temp.A + 90;
	}

}

double angle_range_process( double angle)
{
	//使角度数值的范围恢复到0~360内
	while (angle >= 360)
	{
		angle = angle - 360;
	}
	while(angle < 0)
	{
		angle = angle + 360;
	}
	return angle;
}

void GDGZ_init(SGeo UAV_aircraft,float speed, SGeo goal_point ,unsigned int uav_index)
{

	SGeo route_point[25];//生成的航线点集合
	unsigned short point_num = 0;//航线点数量
	//航向有效
	if(blk_dtms_ctas_002.region_infos[0].point_infos[0].point_hangxiang_valid == 1)
	{
		float goal_vel = 0;
		float goal_course = blk_dtms_ctas_002.region_infos[0].point_infos[0].target_hangxiang;
		//航速有效
		if(blk_dtms_ctas_002.region_infos[0].point_infos[0].point_speed_valid == 1)
		{
			goal_vel = blk_dtms_ctas_002.region_infos[0].point_infos[0].target_speed;
		}
		else
		{
			goal_vel = 8;//目标航速  若没有则默认八节
		}
		//定义变量
		SPolar SPgoal1;
		SPolar SProute1[4];
		SGeo SPgoal_point1[3];
		//计算无人机到目标点的预计时间
		float time_to_target = 0;
		//速度小于100，用常规速度
		if(speed < 100)
			speed = 128;
		if(speed > 0)
		{

			double distance = calculate_distances(UAV_aircraft.Lat,UAV_aircraft.Lon,goal_point.Lat,goal_point.Lon);
			time_to_target = distance/speed;
		}
		else
		{
			time_to_target = 0;
		}

		//计算未来目标经过的点
		for(int i = 0 ; i < 3 ; i ++)
		{
			//求出当前循环中目标点的预测位置
			SPgoal1.R = (((4.5/60)*i + time_to_target)* goal_vel);
			SPgoal1.A = 90 - goal_course;
			SPgoal_point1[i] = Polar_to_Geo(goal_point, SPgoal1);
		}
		//
		double delta_course = calculate_angle(UAV_aircraft.Lat,UAV_aircraft.Lon,goal_point.Lat,goal_point.Lon) - goal_course;
		delta_course = angle_range_process(delta_course);
		double getin_course = 0;
		int M = 0;
		if ((delta_course >= 0) && (delta_course < 90))
		{
			getin_course =angle_range_process (goal_course + 225);
		}
		else if ((delta_course >= 90) && (delta_course < 180))
		{
			getin_course =angle_range_process (goal_course + 225);
			M = 1;
		}
		else if ((delta_course >= 180) && (delta_course < 270))
		{
			getin_course =angle_range_process (goal_course + 45);
		}
		else if ((delta_course >= 270) && (delta_course < 360))
		{
			getin_course =angle_range_process (goal_course + 45);
			M = 1;
		}

		if (M == 1)
		{
			SPolar temp;
			temp.R = 12.0 / sqrt(2);
			temp.A =  - getin_course ;
			while (temp.A <= -180.0)
			{
				temp.A = temp.A+ 360.0;
			}
			while (temp.A >= 180.0)
			{
				temp.A = temp.A - 360.0;
			}
			route_point[0] = Polar_to_Geo(SPgoal_point1[0],temp);
			point_num++;
		}
		//
		//计算航线
		for(int i = 0 ; i < 3 ; i ++)
		{
			calculate_rectangle_vertices(SPgoal_point1[i],getin_course,12.0,&route_point[i*4+M]);
			point_num += 4;
		}

		//增加最后一个矩形的中心点作为盘旋点 20251021new
		AreaRectVertex tmp;
		AreaRectCenter rtn;
		for(int k = 0; k < 4 ; k ++)
		{
			tmp.vertexA[k].latitude = route_point[k+7].Lat;
			tmp.vertexA[k].longitude = route_point[k+7].Lon;
		}
		//生成矩形中心点
		rtn = getAreaRectCenterByVertex(&tmp);

		int indx = 0;
		if(M == 1)
		{
			indx = 13;
		}
		else
		{
			indx = 12;
		}
		route_point[indx].Lat = rtn.center.latitude;
		route_point[indx].Lon = rtn.center.longitude;

		point_num++;
	}
	else//航向无效，直接做切向回环搜索3圈
	{
		//计算无人机向目标点的切入点角度
		double dis = calculate_distances(UAV_aircraft.Lat,UAV_aircraft.Lon, goal_point.Lat,goal_point.Lon);//计算距离
		if (dis >= 8500)
		{
			double azimuth = calculate_angle(UAV_aircraft.Lat,UAV_aircraft.Lon,goal_point.Lat,goal_point.Lon) + asin(6000/dis);
		}
		double azimuth = 0;

		for(int i = 0 ; i < 3 ; i ++)
		{
			calculate_rectangle_vertices(goal_point,azimuth,12.0,&route_point[i*4]);
			point_num += 4;
		}
		//增加最后一个矩形的中心点作为盘旋点 20251021new
		AreaRectVertex tmp;
		AreaRectCenter rtn;
		for(int k = 0; k < 4 ; k ++)
		{
			tmp.vertexA[k].latitude = route_point[k].Lat;
			tmp.vertexA[k].longitude = route_point[k].Lon;
		}
		//生成矩形中心点
		rtn = getAreaRectCenterByVertex(&tmp);

		route_point[12].Lat = rtn.center.latitude;
		route_point[12].Lon = rtn.center.longitude;

		point_num++;

	}

	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;

	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number = point_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].subtask_ID_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;

	for(int i = 0;i < point_num;i++){
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude = route_point[i].Lon;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude = route_point[i].Lat;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].height = blk_dtms_ctas_010[uav_index].GDGZ;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].speed = 35;
		//最后一个点为盘旋点
		if(i == point_num - 1)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber
			= PX_CIRCLE;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius
			= PX_RAD;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].causality = 3;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].standby_type = 2;//圈数
		}
	}
}
void single_uav_GDGZ()
{
	//将收到的战术战法指令赋值给规划所需使用的任务区、有无人机等数据
	init_zhanshutuijians_single();
	//初始化无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));
	//找到无人机索引
	unsigned int uav_index = 0;
	for(unsigned int i = 0 ; i < 4 ; i ++)
	{
		if(drone_state_informations.drone_specific_informations[i].platform_num == blk_dtms_ctas_002.uav_id)
		{
			uav_index = i;
			break;
		}
		uav_index = 44;
	}
	//索引异常
	if(uav_index > 4)
	{
		int area_rtn = -6;
		data_length = sizeof(int);
		Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
		return;
	}
	//方案编号获取
	if(blk_dtms_ctas_002.planning_id == 0)
	{
		//无运行方案，首次进行单无人机指控
		information_on_the_results_of_taskings.program_number = 3;//3为假定值
	}
	else
	{
		information_on_the_results_of_taskings.program_number = blk_dtms_ctas_002.planning_id;
	}
	information_on_the_results_of_taskings.tasking_release = 3;//任务分配方案发布---修改
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

	information_on_the_results_of_taskings.number_of_mission_platforms = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_model = 2;//平台型号--无人机

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_serial_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;//平台序号---无人机i

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_num;//平台编号---无

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].number_of_subtasks = 1;//任务数量为1

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].sequence_type = \
			blk_dtms_ctas_002.task_type;//任务类型
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].modify = 2;//任务序列变更
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位
	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].mission_type = blk_dtms_ctas_002.task_type;//光电跟踪
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].point_area_type = 1;/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].area_point_line_goal_number = area_sky_informations.area_informations[0].area_code;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].AtomicHighlyUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].total_packet = 1;
	//生成航线
	SGeo UAV_aircraft;
	SGeo goal_point;
	//无人机位置
	UAV_aircraft.Lon = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.longitude;
	UAV_aircraft.Lat = integrated_postures.integrated_posture_drone_informations[uav_index].drone_longitude_and_latitude.latitude;
	//目标位置
	goal_point.Lon = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude;
	goal_point.Lat = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude;
	//无人机地速km/h
	float speed = integrated_postures.integrated_posture_drone_informations[uav_index].drone_speed;
	printf("GDGZ goal_vel = %f goal_course = %f \n",blk_dtms_ctas_002.region_infos[0].point_infos[0].target_speed,
			blk_dtms_ctas_002.region_infos[0].point_infos[0].target_hangxiang);
	//光电跟踪航路规划
	GDGZ_init(UAV_aircraft,speed,goal_point,uav_index);
#if AIRWAY_LIMIT
	//空域许可范围
	int area_rtn = 0;
	for(int i = 0 ; i < CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number ; i++ )
	{
		area_rtn = airway_area_confirm(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].longitude,
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[i].latitude);
		if(area_rtn < 0)
		{
			//任务目标点超过许可空域范围 -1 ; 任务目标点距离许可空域边界过近 -2
			if(area_rtn == -1 || area_rtn == -2)
			{
				//异常反馈
				data_length = sizeof(int);
				Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
				if(enRetCode == 0)
				{
					printf("airway_area erro\n");
					return;
				}
			}
		}
	}
#endif
	//生成空域
	single_air_area(uav_index);
	//发送空域信息
	send_airway_area();
	//航线发送,下标为1防止多加入无人机当前航路点
	send_drone_route_confirmation(uav_index,1);
	//单任务区任务分配结果发送
	send_single_result();
	//    //异常反馈,无异常为0
	//	data_length = sizeof(int);
	//	Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
}
void single_uav_XT()
{
	//将收到的战术战法指令赋值给规划所需使用的任务区、有无人机等数据
	init_zhanshutuijians_single();
	//初始化无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));
	//找到无人机索引
	unsigned int uav_index = 0;
	for(unsigned int i = 0 ; i < 4 ; i ++)
	{
		if(drone_state_informations.drone_specific_informations[i].platform_num == blk_dtms_ctas_002.uav_id)
		{
			uav_index = i;
			break;
		}
		uav_index = 44;
	}
	//索引异常
	if(uav_index > 4)
	{
		int area_rtn = -6;
		data_length = sizeof(int);
		Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
		return;
	}
	//方案编号获取
	if(blk_dtms_ctas_002.planning_id == 0)
	{
		//无运行方案，首次进行单无人机指控
		information_on_the_results_of_taskings.program_number = 3;//3为假定值
	}
	else
	{
		information_on_the_results_of_taskings.program_number = blk_dtms_ctas_002.planning_id;
	}
	information_on_the_results_of_taskings.tasking_release = 3;//任务分配方案发布---修改
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定

	information_on_the_results_of_taskings.number_of_mission_platforms = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_model = 2;//平台型号--无人机

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_serial_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;//平台序号---无人机i

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].platform_number = \
			drone_state_informations.drone_specific_informations[uav_index].platform_num;//平台编号---无

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].number_of_subtasks = 1;//任务数量为1

	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].sequence_type = \
			blk_dtms_ctas_002.task_type;//任务类型
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].modify = 2;//任务序列变更
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].subtask_ID_number = 1;//子任务ID号
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].task_height_effective_position = 1;//高度有效位
	information_on_the_results_of_taskings.formation_synergy_mission_programs[uav_index + 1].task_sequence_informations[0].completion_time_valid_Bits = 1;//时间有效位

	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.uav_plan_id = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number = drone_state_informations.drone_specific_informations[uav_index].platform_serial_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_num = drone_state_informations.drone_specific_informations[uav_index].platform_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].mission_type = blk_dtms_ctas_002.task_type;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].waypoints_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].subtask_ID_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].AtomicHighlyUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].total_packet = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].longitude
	= blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].latitude
	= blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude;
#if AIRWAY_LIMIT
	//空域许可范围
	int area_rtn = 0;
	area_rtn = airway_area_confirm(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].longitude,
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].latitude);
	if(area_rtn < 0)
	{
		//任务目标点超过许可空域范围 -1 ; 任务目标点距离许可空域边界过近 -2
		if(area_rtn == -1 || area_rtn == -2)
		{
			//异常反馈
			data_length = sizeof(int);
			Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
			if(enRetCode == 0)
			{
				printf("airway_area erro\n");
				return;
			}
		}
	}
#endif

	//盘旋点特征字
	if(blk_dtms_ctas_002.task_type == 14)
	{
		//航路点待机时间/圈数/循环次数
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber
		= PX_CIRCLE;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
		//航路点待机半径
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].standby_radius
		= PX_RAD;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
		//高度速度,类型
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].height = blk_dtms_ctas_010[uav_index].PX;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].speed = 35;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].causality = 3;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].standby_type = 2;//圈数
	}
	//悬停点特征字
	else if(blk_dtms_ctas_002.task_type == 12)
	{
		//航路点待机时间/圈数/循环次数
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber
		= 65535;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
		//航路点待机半径
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].standby_radius
		= 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
		//高度速度，类型
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].height = blk_dtms_ctas_010[uav_index].PX;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].speed = 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].causality = 9;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[0].standby_type = 1;//时间
	}
	//悬停空域计算
	XT_area_calc(blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude
			,blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude
			,uav_index + 1);
	//发送空域信息
	send_airway_area();
	//航线发送
	send_drone_route_confirmation(uav_index,0);
	//单任务区任务分配结果发送
	send_single_result();
	//	//异常反馈,无异常为0
	//	data_length = sizeof(int);
	//	Send_Message(DDSTables.BLK_CTAS_DTMS_011.niConnectionId, 0, &transaction_id, &area_rtn, &message_type_id, data_length, &enRetCode);
}

//悬停点/盘旋点空域计算
void XT_area_calc(double centerLon,double centerLat,int index)
{
	SGeo out[4];
	SGeo in;
	SPolar temp;
	in.Lat = centerLat;
	in.Lon = centerLon;
	temp.R = 2.0 / sqrt(2);
	temp.A = 0;
	for (int n = 0;  n < 4 ; n++)
	{
		while (temp.A <= -180.0)
		{
			temp.A = temp.A+ 360.0;
		}
		while (temp.A >= 180.0)
		{
			temp.A = temp.A - 360.0;
		}
		out[n] = Polar_to_Geo(in,temp);
		temp.A = temp.A + 90;
	}
	for(int i = 0 ; i < 4 ; i++)
	{
		blk_ctas_dtms_010.airway_area[index].vertexA[i].longitude = out[i].Lon;
		blk_ctas_dtms_010.airway_area[index].vertexA[i].latitude = out[i].Lat;
	}
//	//初始化
//	double square[4][2];
//	memset(&blk_ctas_dtms_010,0,sizeof(BLK_CTAS_DTMS_010));
//	generateSquare(centerLon,centerLat,2000.0,square);
//	for(int i = 0 ; i < 4 ; i++)
//	{
//		blk_ctas_dtms_010.airway_area[index].vertexA[i].longitude = square[i][0];
//		blk_ctas_dtms_010.airway_area[index].vertexA[i].latitude = square[i][1];
//	}
	blk_ctas_dtms_010.id[index] = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[index - 1].drone_serial_number;
}
//计算目标与无人机最短距离的无人机索引
unsigned int distance_min(double lat,double lon)
{
	unsigned int min_index = 0;
	double min_dist = 0;
	double current_dist = 0;
	//找到第一个有效无人机作为初始最小值
	for(unsigned int i = 0 ; i < 4 ; i ++)
	{
		//无人机有效
		if(integrated_postures.integrated_posture_drone_informations[i].drone_number > 0)
		{
			min_dist = calculate_distances(lat,lon,\
					integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.latitude,\
					integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.longitude);
			break;
		}
	}
	//无人机比较
	for(unsigned int i = 0 ; i < 4 ; i ++)
	{
		//无人机有效
		if(integrated_postures.integrated_posture_drone_informations[i].drone_number > 0)
		{
			current_dist = calculate_distances(lat,lon,\
					integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.latitude,\
					integrated_postures.integrated_posture_drone_informations[i].drone_longitude_and_latitude.longitude);
			if(current_dist <= min_dist)
			{
				min_dist = current_dist;
				min_index = i + 1;
			}
		}
	}
	return min_index ;
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
void calc_air_area(int stage)
{
	//获取有、无人机的航点
	Point points[5][250];
	Point hull[5][250];
	memset(&points[0][0],0,sizeof(Point)*5*250);
	memset(&hull[0][0],0,sizeof(Point)*5*250);
	memset(&blk_ctas_dtms_010,0,sizeof(BLK_CTAS_DTMS_010));
	int num[5] = {0,0,0,0,0};
	//有人机航线获取
	//    num[0] = CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoints_number;
	//    for(int i = 0 ; i < num[0] ; i ++)
	//    {
	//        points[0][i].lat = CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoint_informations[i].latitude;
	//        points[0][i].lon = CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[stage].waypoint_informations[i].longitude;
	//    }

	//无人机航线获取
	for(int i = 0 ; i < 4 ; i ++)
	{
		num[i+1] = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[stage].waypoints_number;
		for(int j = 0 ; j < num[i+1] ; j ++)
		{
			points[i+1][j].lat = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[stage].planning_information_waypoint_informations[j].latitude;
			points[i+1][j].lon = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[stage].planning_information_waypoint_informations[j].longitude;
		}
	}
	//计算空域
	for(int i = 0 ; i < 5 ; i ++)
	{
		MabrGeo tmp;
		tmp = air_area(points[i],hull[i],num[i]);
		//航线只有一个航路点或两个航路点
		if(num[i] == 1)
		{
			//生成矩形空域
			double square[4][2];
			// 使用当前平台对应航点，避免空域与下一个平台错位
			generateSquare(points[i][0].lon,points[i][0].lat,4000.0,square);
			for(int k = 0 ; k < 4; k ++)
			{
				tmp.vertexA[k].longitude = square[k][0];
				tmp.vertexA[k].latitude = square[k][1];
			}
		}
		else if(num[i] == 2)
		{
			//使用任务区作为空域
			for(int k = 0;k < area_sky_informations.area_number;k++)
			{
				//找到对应的任务区域
				if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[stage].target_number == area_sky_informations.area_informations[k].area_code)
				{
					//圆形
					if(area_sky_informations.area_informations[k].area_shape == 1)
					{
						RectLibDas *rectLibDas;
						GeoLibDas center;
						double radius = 0;
						center.longitude = area_sky_informations.area_informations[k].cycles.longitude;
						center.latitude = area_sky_informations.area_informations[k].cycles.latitude;
						radius = area_sky_informations.area_informations[k].cycles.radius;
						//转换为矩形
						rectLibDas = cycle_to_RectLibDas(center,radius);
						for(int m = 0 ; m < 4 ; m ++)
						{
							tmp.vertexA[m].longitude = rectLibDas->vertexA[m].longitude;
							tmp.vertexA[m].latitude = rectLibDas->vertexA[m].latitude;
						}
					}
					//矩形
					else if(area_sky_informations.area_informations[k].area_shape == 2)
					{
						for(int m = 0 ; m < 4 ; m ++)
						{
							tmp.vertexA[m].longitude = area_sky_informations.area_informations[k].polygonals.point_coordinates[m].longitude;
							tmp.vertexA[m].latitude = area_sky_informations.area_informations[k].polygonals.point_coordinates[m].latitude;
						}

					}
				}
			}
		}
		memcpy(&blk_ctas_dtms_010.airway_area[i],&tmp,sizeof(MabrGeo));
		//有效空域
		if(num[i] > 0)
		{
			if(i == 0)
			{
				blk_ctas_dtms_010.id[i] = MANNED_ID;
			}
			else
			{
				blk_ctas_dtms_010.id[i] = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i-1].drone_serial_number;
			}
		}
	}

}
void send_airway_area()
{
	//组织当前平台和任务序列
	data_length = sizeof(BLK_CTAS_DTMS_010);
	Send_Message(DDSTables.BLK_CTAS_DTMS_010.niConnectionId, 0, &transaction_id, &blk_ctas_dtms_010, &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0)
	{
		printf("send air_area success\n");
	}
}

static int hx_find_area_index_by_code(int area_code)
{
	for(int i = 0; i < area_sky_informations.area_number; i++)
	{
		if(area_sky_informations.area_informations[i].area_code == area_code)
		{
			return i;
		}
	}
	return -1;
}

static int hx_geo_is_valid(double lat, double lon)
{
	if(lat != lat || lon != lon)
	{
		return 0;
	}
	if(lat < -90.0 || lat > 90.0)
	{
		return 0;
	}
	if(lon < -180.0 || lon > 180.0)
	{
		return 0;
	}
	return 1;
}

static int hx_rect_is_reasonable_for_route(const AreaRectVertex* rect, const FlightRoute* route1, const FlightRoute* route3)
{
	AreaRectVertex rect_local;
	memcpy(&rect_local, rect, sizeof(AreaRectVertex));
	AreaRectCenter center = getAreaRectCenterByVertex(&rect_local);
	if(!hx_geo_is_valid(center.center.latitude, center.center.longitude))
	{
		return 0;
	}
	// 中间障碍区应与当前规避航段处于同一作战区域，过远说明数据异常/陈旧
	double d_start = calculate_distances(route1->start.latitude, route1->start.longitude,
			center.center.latitude, center.center.longitude);
	double d_end = calculate_distances(route3->end.latitude, route3->end.longitude,
			center.center.latitude, center.center.longitude);
	if(d_start > 3000.0 || d_end > 3000.0)
	{
		return 0;
	}
	// 矩形边长做基本约束，过滤明显异常顶点
	for(int i = 0; i < 4; i++)
	{
		int j = (i + 1) % 4;
		double edge = calculate_distances(rect->vertexA[i].latitude, rect->vertexA[i].longitude,
				rect->vertexA[j].latitude, rect->vertexA[j].longitude);
		if(edge < 0.01 || edge > 2000.0)
		{
			return 0;
		}
	}
	return 1;
}

//航线冲突规避
void hx_avoid(int stage)
{
	//第一阶段不存在跳任务区，当前只考虑一控二
	if(stage == 0 || integrated_postures.drone_num == 1)
	{
		return;
	}
	//检测是否有跳过任务区的情况
	for(int uav = 0 ; uav < integrated_postures.drone_num ; uav ++)
	{
		int task_new_code = information_on_the_results_of_taskings.formation_synergy_mission_programs[uav+1].task_sequence_informations[stage].target_number;
		int task_old_code = information_on_the_results_of_taskings.formation_synergy_mission_programs[uav+1].task_sequence_informations[stage-1].target_number;
		int task_new = hx_find_area_index_by_code(task_new_code);
		int task_old = hx_find_area_index_by_code(task_old_code);
		// 按任务区索引判断是否“跳过中间任务区”
		if(task_new >= 0 && task_old >= 0 && (task_new-task_old == 2 || task_new-task_old == -2))
		{
			int mid_task_index = (task_new + task_old) / 2;
			if(mid_task_index < 0 || mid_task_index >= area_sky_informations.area_number)
			{
				continue;
			}
			if(area_sky_informations.area_informations[mid_task_index].area_shape != 2 ||
			   area_sky_informations.area_informations[mid_task_index].polygonals.point_number < 4)
			{
				continue;
			}
			//计算外扩的矩形顶点
			AreaRectVertex task_rect;
			AreaRectVertex new_rect;
			AreaRectCenter new_rect_center;
			int rect_valid = 1;
			for(int i = 0 ; i < 4 ; i ++)
			{
				double lat = area_sky_informations.area_informations[mid_task_index].polygonals.point_coordinates[i].latitude;
				double lon = area_sky_informations.area_informations[mid_task_index].polygonals.point_coordinates[i].longitude;
				if(!hx_geo_is_valid(lat, lon))
				{
					rect_valid = 0;
					break;
				}
				task_rect.vertexA[i].latitude = lat;
				task_rect.vertexA[i].longitude = lon;
			}
			if(!rect_valid)
			{
				continue;
			}
			// 标准化顶点顺序，避免中间任务区顶点顺序异常导致中心/外扩结果异常
			setAreaRectVertexsClock(&task_rect, ROTATE_CLOCKWISE);
			//计算新矩形中心信息
			new_rect_center = getAreaRectCenterByVertex(&task_rect);
			// 外扩一公里
			new_rect_center.lenLat += 1000;
			new_rect_center.lenLng += 1000;
			//生成外扩矩形
			new_rect = getAreaRectVertexByCenter(&new_rect_center);
			setAreaRectVertexsClock(&new_rect, ROTATE_CLOCKWISE);
			for(int i = 0; i < 4; i++)
			{
				if(!hx_geo_is_valid(new_rect.vertexA[i].latitude, new_rect.vertexA[i].longitude))
				{
					rect_valid = 0;
					break;
				}
			}
			if(!rect_valid)
			{
				continue;
			}

			//计算最合适的两个顶点
			int point_1 = -1;
			int point_2 = -1;
			int last_index = 0;
			double minLength = 1e20;
			double pathLength;
			FlightRoute route1;
			FlightRoute route2;
			FlightRoute route3;
			//起点是上一个任务的最后一个点
			int prev_num = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage-1].waypoints_number;
			int curr_num = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].waypoints_number;
			if(prev_num <= 0 || curr_num <= 0)
			{
				continue;
			}
			last_index = prev_num - 1;
			route1.start.latitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage-1].planning_information_waypoint_informations[last_index].latitude;
			route1.start.longitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage-1].planning_information_waypoint_informations[last_index].longitude;
			//终点是当前阶段任务的第一个点
			route3.end.latitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[0].latitude;
			route3.end.longitude = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[0].longitude;
			if(!hx_geo_is_valid(route1.start.latitude, route1.start.longitude) ||
			   !hx_geo_is_valid(route3.end.latitude, route3.end.longitude))
			{
				continue;
			}
			if(!hx_rect_is_reasonable_for_route(&task_rect, &route1, &route3) ||
			   !hx_rect_is_reasonable_for_route(&new_rect, &route1, &route3))
			{
				continue;
			}
			//找到最合适的两个点的下标
			for (int i = 0; i < 4; i++)
			{
				route1.end = new_rect.vertexA[i];
				route2.start = new_rect.vertexA[i];
				route2.end = new_rect.vertexA[(i+1)%4];
				route3.start = new_rect.vertexA[(i+1)%4];

				if(!avoidLineCrashIsCrossRect(&route1, &task_rect) && \
				   !avoidLineCrashIsCrossRect(&route2, &task_rect) && \
				   !avoidLineCrashIsCrossRect(&route3, &task_rect) )
				{
					// 计算经过各顶点的路径长度
					pathLength = getDistanceGeo(&route1.start, &route1.end) + getDistanceGeo(&route2.start, &route2.end) + \
							     getDistanceGeo(&route3.start, &route3.end);
					if (pathLength < minLength) {
						minLength = pathLength;
						point_1 = i;
						point_2 = (i+1)%4;
					}
				}
			}
			for(int i = 0; i < 4; i++)
			{
				route1.end = new_rect.vertexA[(i+1)%4];
				route2.start = new_rect.vertexA[(i+1)%4];
				route2.end = new_rect.vertexA[i];
				route3.start = new_rect.vertexA[i];

				if(!avoidLineCrashIsCrossRect(&route1, &task_rect) && \
				   !avoidLineCrashIsCrossRect(&route2, &task_rect) && \
				   !avoidLineCrashIsCrossRect(&route3, &task_rect) )
				{
					// 计算经过各顶点的路径长度
					pathLength = getDistanceGeo(&route1.start, &route1.end) + getDistanceGeo(&route2.start, &route2.end) + \
								 getDistanceGeo(&route3.start, &route3.end);
					if (pathLength < minLength) {
						minLength = pathLength;
						point_1 = (i+1)%4;
						point_2 = i;
					}
				}
			}
			//根据中间任务区外扩的四个顶点中两个顶点作为规避航点
			if(point_1 == -1 || point_2 == -1)
			{
				return;
			}
			// 对最终规避点做最后一道地理合理性校验，避免插入异常远点
			Geo avoid_p1 = new_rect.vertexA[point_1];
			Geo avoid_p2 = new_rect.vertexA[point_2];
			if(!hx_geo_is_valid(avoid_p1.latitude, avoid_p1.longitude) ||
			   !hx_geo_is_valid(avoid_p2.latitude, avoid_p2.longitude))
			{
				continue;
			}
			double p1_d_start = calculate_distances(route1.start.latitude, route1.start.longitude,
					avoid_p1.latitude, avoid_p1.longitude);
			double p1_d_end = calculate_distances(route3.end.latitude, route3.end.longitude,
					avoid_p1.latitude, avoid_p1.longitude);
			double p2_d_start = calculate_distances(route1.start.latitude, route1.start.longitude,
					avoid_p2.latitude, avoid_p2.longitude);
			double p2_d_end = calculate_distances(route3.end.latitude, route3.end.longitude,
					avoid_p2.latitude, avoid_p2.longitude);
			if(p1_d_start > 3000.0 || p1_d_end > 3000.0 ||
			   p2_d_start > 3000.0 || p2_d_end > 3000.0)
			{
				continue;
			}
			//保存当前无人机航线
			planning_information UAVRoute_save;
			memcpy(&UAVRoute_save,&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage],sizeof(planning_information));
			//赋值第一个航点
			memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[0],
					&UAVRoute_save.planning_information_waypoint_informations[0],sizeof(planning_information_waypoint_information));
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[0].latitude
				= new_rect.vertexA[point_1].latitude;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[0].longitude
				= new_rect.vertexA[point_1].longitude;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[0].causality = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[0].standby_type = 0;
			//赋值第二个航点
			memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[1],
					&UAVRoute_save.planning_information_waypoint_informations[0],sizeof(planning_information_waypoint_information));
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[1].latitude
				= new_rect.vertexA[point_2].latitude;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[1].longitude
				= new_rect.vertexA[point_2].longitude;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[1].causality = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[1].standby_type = 0;
			//任务点数量增加2（注意数组上限）
			int max_waypoints = (int)(sizeof(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations) /
					sizeof(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[0]));
			int old_waypoints_number = UAVRoute_save.waypoints_number;
			if(old_waypoints_number > max_waypoints - 2)
			{
				old_waypoints_number = max_waypoints - 2;
			}
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].waypoints_number = old_waypoints_number + 2;
			//赋值原有航路点
			memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[2],
					&UAVRoute_save.planning_information_waypoint_informations[0],
					sizeof(planning_information_waypoint_information) * old_waypoints_number);
			//任务点的 索引值需要统一设置
			int waypoints_number = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].waypoints_number;
			for(int i =0; i< waypoints_number; i++)
			{
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].planning_information_waypoint_informations[i].hld_idx = i+1;
			}
			//重新计算包数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].total_packet = waypoints_number / 25;
			if(waypoints_number % 25)
			{
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].total_packet++;
			}
			if(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].total_packet == 0)
			{
				CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav].planning_informations[stage].total_packet = 1;
			}
		}
	}
}
/*
 * 单任务区划分
 * */
static void normalizeRectVertices(GeoLibDas vertexA[4])
{
	const double eps = 1e-9;
	double cLat = 0.0, cLon = 0.0;
	int order[4] = {0, 1, 2, 3};
	GeoLibDas sorted[4];

	for(int i = 0; i < 4; i++) {
		cLat += vertexA[i].latitude;
		cLon += vertexA[i].longitude;
	}
	cLat /= 4.0;
	cLon /= 4.0;

	for(int i = 0; i < 3; i++) {
		for(int j = i + 1; j < 4; j++) {
			int idxI = order[i];
			int idxJ = order[j];
			double dxI = vertexA[idxI].longitude - cLon;
			double dyI = vertexA[idxI].latitude - cLat;
			double dxJ = vertexA[idxJ].longitude - cLon;
			double dyJ = vertexA[idxJ].latitude - cLat;
			double angI = atan2(dyI, dxI);
			double angJ = atan2(dyJ, dxJ);
			double dist2I = dxI * dxI + dyI * dyI;
			double dist2J = dxJ * dxJ + dyJ * dyJ;

			if(angJ < angI - eps || (fabs(angJ - angI) < eps && dist2J < dist2I)) {
				int tmp = order[i];
				order[i] = order[j];
				order[j] = tmp;
			}
		}
	}

	for(int i = 0; i < 4; i++) {
		sorted[i] = vertexA[order[i]];
	}

	double area2 = 0.0;
	for(int i = 0; i < 4; i++) {
		int j = (i + 1) % 4;
		area2 += sorted[i].longitude * sorted[j].latitude - sorted[j].longitude * sorted[i].latitude;
	}
	if(area2 < 0.0) {
		GeoLibDas tmp = sorted[0];
		sorted[0] = sorted[3];
		sorted[3] = tmp;
		tmp = sorted[1];
		sorted[1] = sorted[2];
		sorted[2] = tmp;
	}

	int startIdx = 0;
	for(int i = 1; i < 4; i++) {
		if(sorted[i].latitude < sorted[startIdx].latitude - eps ||
		   (fabs(sorted[i].latitude - sorted[startIdx].latitude) < eps &&
			sorted[i].longitude < sorted[startIdx].longitude)) {
			startIdx = i;
		}
	}

	for(int i = 0; i < 4; i++) {
		vertexA[i] = sorted[(startIdx + i) % 4];
	}
}

void task_area_division(region_info * area,int divi_num)
{
	//区域形状为圆形
	if(area->reg_shape == 1 )
	{
		double circle_center_lat = area->reg_circle.center_lon_lat.latitude;
		double circle_center_lon = area->reg_circle.center_lon_lat.longitude;
		double circle_center_rad = area->reg_circle.radious;
		//地球半径常量
		const double R = 6371.0;
		//计算纬度方向的变化量
		double delta_lat = ( circle_center_rad / R ) * ( 180.0 / M_PI);
		//中心纬度转换为弧度
		double lat_center_rad = circle_center_lat * M_PI / 180.0;
		//处理经度反向的变化量
		double cos_val = cos(lat_center_rad);
		double delta_lon,lon_min,lon_max;
		//处理极地情况
		if(fabs(cos_val) < 1e-9)
		{
			lon_min = -180.0;
			lon_max = 180.0;
		}
		else
		{
			delta_lon = (( circle_center_rad / R ) * ( 180.0 / M_PI)) / cos_val ;
			lon_min = circle_center_lon - delta_lon;
			lon_max = circle_center_lon + delta_lon;
			//将经度调整到±180的范围内；
			lon_min = fmod(lon_min + 180.0,360) - 180.0;
			lon_max = fmod(lon_max + 180.0,360) - 180.0;
			//处理经度反转情况
			if(lon_min > lon_max)
			{
				lon_min = -180.0;
				lon_max = 180.0;
			}
		}
		//处理纬度边界并调整范围
		double lat_min = circle_center_lat - delta_lat;
		double lat_max = circle_center_lat + delta_lat;
		if(lat_min < -90.0) lat_min = -90.0;
		if(lat_max > 90.0) lat_max = 90.0;

		//将外接正方形分解成n行，m列，分解成divi_num个数区域；
		int n,m;
		//        for(n = (int)sqrt(divi_num); n >= 1; n --)
		//        {
		//            if(divi_num % n == 0)
		//            {
		//                m = divi_num / n;
		//                break;
		//            }
		//        }
		n = divi_num;
		m = 1;
		//计算每个小正方形的步长
		double step_lat = (lat_max - lat_min) / m;
		double step_lon = (lon_max - lon_min) / n;

		//任务区计数
		int index = 0;
		//最多八个任务区域，矩形四个顶点
		double lat[8][4] = {0};
		double lon[8][4] = {0};
		for(int i = 0 ; i < m ; i ++)
		{
			for(int j = 0 ; j < n ; j ++)
			{
				lat[index][0] = lat_min + i * step_lat;
				lon[index][0] = lon_min + j * step_lon;

				lat[index][1] = lat[index][0];
				lon[index][1] = lon_min + (j + 1) * step_lon;

				lat[index][2] = lat_min + (i + 1) * step_lat;
				lon[index][2] = lon[index][1];

				lat[index][3] = lat[index][2];
				lon[index][3] = lon[index][0];

				index++;
			}
		}
		//赋值
		area_sky_informations.area_number = divi_num;
		for(int i = 0; i < divi_num ; i ++)
		{
			area_sky_informations.area_informations[i].area_shape = 2;//区域形状
			area_sky_informations.area_informations[i].polygonals.point_number = 4;
			area_sky_informations.area_informations[i].area_code = i + 1;//区域编号
			area_sky_informations.area_informations[i].area_type = blk_dtms_ctas_001.region_infos[0].reg_type;//区域类型
			area_sky_informations.area_informations[i].area_source = blk_dtms_ctas_001.region_infos[0].reg_sour;//区域来源
			area_sky_informations.area_informations[i].drone_numbe = blk_dtms_ctas_001.region_infos[0].kongyu_belong_to_uav_id;//空域所属无人机序号
			area_sky_informations.area_informations[i].upper_height_limit_valid_bit = blk_dtms_ctas_001.region_infos[0].reg_top_of_hei_valid;//区域高度上限有效位
			area_sky_informations.area_informations[i].lower_height_limit_valid_bit = blk_dtms_ctas_001.region_infos[0].reg_down_of_hei_valid;//区域高度下限有效位
			area_sky_informations.area_informations[i].upper_height_limit = blk_dtms_ctas_001.region_infos[0].top_of_hei;//区域高度上限
			area_sky_informations.area_informations[i].lower_height_limit = blk_dtms_ctas_001.region_infos[0].down_of_hei;//区域高度下限

			for(int j = 0 ; j < 4 ; j ++)
			{
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude = lat[i][j];
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude = lon[i][j];
			}
		}
	}
	//区域形状为多边形
	else if(area->reg_shape == 2 && area->reg_ploygen.point_num == 4)
	{
		//        //获取矩形四个顶点经纬度
		//        longitude_and_latitude vertices[4];
		//        for(int i = 0 ; i < 4 ; i ++)
		//        {
		//            vertices[i].latitude = area->reg_ploygen.points_lon_lat[i].latitude;
		//            vertices[i].longitude = area->reg_ploygen.points_lon_lat[i].longitude;
		//        }

		//        double lat_min = vertices[0].latitude, lat_max = vertices[0].latitude;
		//        double lon_min = vertices[0].longitude , lon_max = vertices[0].longitude;

		//        //遍历找出经纬度范围
		//        for(int i = 0 ; i < 4 ; i ++)
		//        {
		//            if(vertices[i].latitude < lat_min) lat_min = vertices[i].latitude;
		//            if(vertices[i].latitude > lat_max) lat_max = vertices[i].latitude;
		//            if(vertices[i].longitude < lon_min) lon_min = vertices[i].longitude;
		//            if(vertices[i].longitude > lon_max) lon_max = vertices[i].longitude;
		//        }
		//        //找出n行，m列的划分
		//        int n,m;
		////        for(n = (int)sqrt(divi_num); n >= 1; n --)
		////        {
		////            if(divi_num % n == 0)
		////            {
		////                m = divi_num / n;
		////                break;
		////            }
		////        }

		////        //处理质数情况
		////        if(n < 1)
		////        {
		////            n = 1;
		////            m = divi_num;
		////        }
		//        n = 1;
		//        m = divi_num;
		//        double step_lat = (lat_max - lat_min) / n;
		//        double step_lon = (lon_max - lon_min) / m;

		//        //任务区计数
		//        int index = 0;
		//        //最多八个任务区域，矩形四个顶点
		//        double lat[8][4] = {0};
		//        double lon[8][4] = {0};
		//        for(int i = 0 ; i < n ; i ++)
		//        {
		//            for(int j = 0 ; j < m ; j ++)
		//            {
		//                double lat_start = lat_min + i * step_lat;
		//                double lon_start = lon_min + j * step_lon;
		//                double lat_end = lat_start + step_lat;
		//                double lon_end = lon_start + step_lon;

		//                lat[index][0] = lat_start;
		//                lon[index][0] = lon_start;

		//                lat[index][1] = lat_start;
		//                lon[index][1] = lon_end;

		//                lat[index][2] = lat_end;
		//                lon[index][2] = lon_end;

		//                lat[index][3] = lat_end;
		//                lon[index][3] = lon_start;

		//                index++;
		//            }
		//        }



		// 整合输入数据
		PolygonGeo temPolygonGeo;
		temPolygonGeo.sumVertex = 4;
		//获取矩形四个顶点经纬度
		for(int i = 0 ; i < 4 ; i ++)
		{
			temPolygonGeo.vertexA[i].longitude = area->reg_ploygen.points_lon_lat[i].longitude;
			temPolygonGeo.vertexA[i].latitude = area->reg_ploygen.points_lon_lat[i].latitude;
		}

		// 定义输出并调用
		AreaRectZone resAreaRectZone;
		resAreaRectZone =  getAreaRectZone(&temPolygonGeo, divi_num, 1);
		//赋值
		area_sky_informations.area_number = divi_num;
		for(int i = 0; i < divi_num ; i ++)
		{
			area_sky_informations.area_informations[i].area_shape = 2;
			area_sky_informations.area_informations[i].polygonals.point_number = 4;
			area_sky_informations.area_informations[i].area_code = i + 1;//区域编号
			area_sky_informations.area_informations[i].area_type = blk_dtms_ctas_001.region_infos[0].reg_type;//区域类型
			area_sky_informations.area_informations[i].area_source = blk_dtms_ctas_001.region_infos[0].reg_sour;//区域来源
			area_sky_informations.area_informations[i].drone_numbe = blk_dtms_ctas_001.region_infos[0].kongyu_belong_to_uav_id;//空域所属无人机序号
			area_sky_informations.area_informations[i].upper_height_limit_valid_bit = blk_dtms_ctas_001.region_infos[0].reg_top_of_hei_valid;//区域高度上限有效位
			area_sky_informations.area_informations[i].lower_height_limit_valid_bit = blk_dtms_ctas_001.region_infos[0].reg_down_of_hei_valid;//区域高度下限有效位
			area_sky_informations.area_informations[i].upper_height_limit = blk_dtms_ctas_001.region_infos[0].top_of_hei;//区域高度上限
			area_sky_informations.area_informations[i].lower_height_limit = blk_dtms_ctas_001.region_infos[0].down_of_hei;//区域高度下限

			GeoLibDas tempVertices[4];
			for(int j = 0 ; j < 4 ; j ++)
			{
				tempVertices[j].latitude = resAreaRectZone.rectA[i].vertexA[j].latitude;
				tempVertices[j].longitude = resAreaRectZone.rectA[i].vertexA[j].longitude;
			}
			normalizeRectVertices(tempVertices);
			for(int j = 0 ; j < 4 ; j ++)
			{
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude = tempVertices[j].latitude;
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude = tempVertices[j].longitude;
			}
		}
	}
}

void init_blk_dtms_ctas_005()
{
	//取出任务分配结果
	memcpy(&information_on_the_results_of_taskings,&blk_dtms_ctas_005.blk_ccc_ofp_019,sizeof(Information_the_results_of_tasking));

	//3.2 兵力信息
	drone_state_informations.drone_number = blk_dtms_ctas_005.solider_num - 1;//无人机数量 = 兵力数量 - 1（有人机数量）

	int cnt = 0 ;
	for(unsigned int i = 1;i < 5;i++)
	{
		if(blk_dtms_ctas_005.solider_infos[i].solider_type == 2){
			drone_state_informations.drone_specific_informations[cnt].platform_model = 1;
			drone_state_informations.drone_specific_informations[cnt].platform_serial_num = i;
			drone_state_informations.drone_specific_informations[cnt].platform_num = blk_dtms_ctas_005.solider_infos[i].solider_id;
			cnt++;
		}
	}

	//3.4 综合态势
	integrated_postures.drone_num = drone_state_informations.drone_number;
	int droneNum = 0;//当前接收到的无人机个数
	for(unsigned int i = 0;i < 5;i++){
		if(blk_dtms_ctas_005.solider_infos[i].solider_type == 1){//为有人机信息赋值
			integrated_postures.longitude = blk_dtms_ctas_005.solider_infos[i].lon_lat_info.longitude;
			integrated_postures.latitude = blk_dtms_ctas_005.solider_infos[i].lon_lat_info.latitude;
			integrated_postures.ABSOLUTE_PRESSURE_ALT_A = blk_dtms_ctas_005.solider_infos[i].height;
			integrated_postures.GROUND_SPEED = blk_dtms_ctas_005.solider_infos[i].speed;
			integrated_postures.direction = blk_dtms_ctas_005.solider_infos[i].hangxiang;
		}else if(blk_dtms_ctas_005.solider_infos[i].solider_type == 2){//为无人机信息赋值
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_serial_number = i;
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_number = blk_dtms_ctas_005.solider_infos[i].solider_id;//士兵序号
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_longitude_and_latitude.longitude = blk_dtms_ctas_005.solider_infos[i].lon_lat_info.longitude;//经度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_longitude_and_latitude.latitude = blk_dtms_ctas_005.solider_infos[i].lon_lat_info.latitude;//纬度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_height = blk_dtms_ctas_005.solider_infos[i].height;//高度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_speed = blk_dtms_ctas_005.solider_infos[i].speed;//速度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_direction = blk_dtms_ctas_005.solider_infos[i].hangxiang;//航向
			integrated_postures.integrated_posture_drone_informations[droneNum].ct = blk_dtms_ctas_005.solider_infos[i].ct;//载荷可用状态
			droneNum++;
		}
	}

	char is_strategy9_layout = is_strategy9_plan_layout_from_result(&information_on_the_results_of_taskings);
	if(is_strategy9_layout)
	{
		// 战法9固定在方案1后，不再依赖 tactical_warfare_options；航线阶段按005回灌任务区
		unsigned short area_num_005 = blk_dtms_ctas_005.blk_ccc_ofp_005.task_are_hf2[0].task_are_hf_num;
		if(area_num_005 > 0)
		{
			area_sky_informations.area_number = area_num_005;
			if(area_sky_informations.area_number > 8)
			{
				area_sky_informations.area_number = 8;
			}
			for(int i = 0; i < area_sky_informations.area_number ; i ++)
			{
				area_sky_informations.area_informations[i].area_shape = 2;//区域形状
				area_sky_informations.area_informations[i].polygonals.point_number = 4;
				area_sky_informations.area_informations[i].area_code = i + 1;//区域编号

				for(int j = 0 ; j < 4 ; j ++)
				{
					area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude =
							blk_dtms_ctas_005.blk_ccc_ofp_005.task_are_hf2[0].signal_FC00[i].signal_FC00[j].Index_Lat;
					area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude =
							blk_dtms_ctas_005.blk_ccc_ofp_005.task_are_hf2[0].signal_FC00[i].signal_FC00[j].Index_Lon;
				}
			}
		}
	}
	//应召反潜且非战法9，没有任务区划分信息
	else if(blk_dtms_ctas_001.task_type == 1 &&
			global_mission_planning_commandss.tactical_warfare_options != 9)
	{
		yinzhao_point_init();
	}
	else if(global_mission_planning_commandss.tactical_warfare_options != 9 &&
			information_on_the_results_of_taskings.program_attributes != 3)
	{
		//任务区划分赋值
		area_sky_informations.area_number = blk_dtms_ctas_005.blk_ccc_ofp_005.task_are_hf2[0].task_are_hf_num;
		for(int i = 0; i < area_sky_informations.area_number ; i ++)
		{
			area_sky_informations.area_informations[i].area_shape = 2;//区域形状
			area_sky_informations.area_informations[i].polygonals.point_number = 4;
			area_sky_informations.area_informations[i].area_code = i + 1;//区域编号

			for(int j = 0 ; j < 4 ; j ++)
			{
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude =
						blk_dtms_ctas_005.blk_ccc_ofp_005.task_are_hf2[0].signal_FC00[i].signal_FC00[j].Index_Lat;
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude =
						blk_dtms_ctas_005.blk_ccc_ofp_005.task_are_hf2[0].signal_FC00[i].signal_FC00[j].Index_Lon;
			}
		}
	}

}
/*
 * 战术战法推荐指令基础信息赋值
 * */
void init_zhanshutuijians(){

	//3.2 兵力信息
	drone_state_informations.drone_number = blk_dtms_ctas_001.solider_num - 1;//无人机数量 = 兵力数量 - 1（有人机数量）

	int cnt = 0 ;
	for(unsigned int i = 1;i < 5;i++)
	{
		if(blk_dtms_ctas_001.solider_infos[i].solider_type == 2){
			drone_state_informations.drone_specific_informations[cnt].platform_model = 1;
			drone_state_informations.drone_specific_informations[cnt].platform_serial_num = i;
			drone_state_informations.drone_specific_informations[cnt].platform_num = blk_dtms_ctas_001.solider_infos[i].solider_id;
			cnt++;
		}
	}

	//3.4 综合态势
	integrated_postures.drone_num = drone_state_informations.drone_number;
	int droneNum = 0;//当前接收到的无人机个数
	for(unsigned int i = 0;i < 5;i++){
		if(blk_dtms_ctas_001.solider_infos[i].solider_type == 1){//为有人机信息赋值
			integrated_postures.longitude = blk_dtms_ctas_001.solider_infos[i].lon_lat_info.longitude;
			integrated_postures.latitude = blk_dtms_ctas_001.solider_infos[i].lon_lat_info.latitude;
			integrated_postures.ABSOLUTE_PRESSURE_ALT_A = blk_dtms_ctas_001.solider_infos[i].height;
			integrated_postures.GROUND_SPEED = blk_dtms_ctas_001.solider_infos[i].speed;
			integrated_postures.direction = blk_dtms_ctas_001.solider_infos[i].hangxiang;
		}else if(blk_dtms_ctas_001.solider_infos[i].solider_type == 2){//为无人机信息赋值
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_serial_number = i;
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_number = blk_dtms_ctas_001.solider_infos[i].solider_id;//士兵序号
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_longitude_and_latitude.longitude = blk_dtms_ctas_001.solider_infos[i].lon_lat_info.longitude;//经度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_longitude_and_latitude.latitude = blk_dtms_ctas_001.solider_infos[i].lon_lat_info.latitude;//纬度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_height = blk_dtms_ctas_001.solider_infos[i].height;//高度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_speed = blk_dtms_ctas_001.solider_infos[i].speed;//速度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_direction = blk_dtms_ctas_001.solider_infos[i].hangxiang;//航向
			integrated_postures.integrated_posture_drone_informations[droneNum].ct = blk_dtms_ctas_001.solider_infos[i].ct;//载荷可用状态
			droneNum++;
		}
	}

	//初始化
	memset( &taskarea_division , 0 , sizeof(BLK_CCC_OFP_005) );
	//3.10 任务区/空域信息
	if(blk_dtms_ctas_001.task_reg_num == 1)
	{
		int divi_num = droneNum + 1;
		//任务区数量为1时，划分任务区（暂时划分无人机数量相等的任务区）+有人机
		task_area_division(&blk_dtms_ctas_001.region_infos[0],divi_num);

		taskarea_division.task_are = 1;/*划分任务区个数 对几个任务区划分*/
		taskarea_division.task_are_hf2[0].Task_Are_ID = blk_dtms_ctas_001.region_infos[0].reg_id;
		taskarea_division.task_are_hf2[0].task_are_hf_num = divi_num;

		for(int i = 0 ; i < divi_num ; i ++)
		{
			taskarea_division.task_are_hf2[0].signal_FC00[i].Task_Are_ID = i + 1;
			for(int j = 0 ; j < 4 ; j ++)
			{
				taskarea_division.task_are_hf2[0].signal_FC00[i].signal_FC00[j].Index_Lat = area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude;
				taskarea_division.task_are_hf2[0].signal_FC00[i].signal_FC00[j].Index_Lon= area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude;
			}
		}
	}
	else if(blk_dtms_ctas_001.task_reg_num > 1)
	{
		area_sky_informations.area_number = blk_dtms_ctas_001.task_reg_num;//任务区数量
		for(int i = 0;i < area_sky_informations.area_number;i++){
			area_sky_informations.area_informations[i].area_code = blk_dtms_ctas_001.region_infos[i].reg_id;//区域编号
			area_sky_informations.area_informations[i].area_type = blk_dtms_ctas_001.region_infos[i].reg_type;//区域类型
			area_sky_informations.area_informations[i].area_source = blk_dtms_ctas_001.region_infos[i].reg_sour;//区域来源
			area_sky_informations.area_informations[i].area_shape = blk_dtms_ctas_001.region_infos[i].reg_shape;//区域形状
			//        area_sky_informations.area_informations[i].drone_number_valid_bit = DTMS_CTAS_data_tacticsRecommend->region_infos[i].kongyu_belong_to_uav_valid;
			area_sky_informations.area_informations[i].drone_numbe = blk_dtms_ctas_001.region_infos[i].kongyu_belong_to_uav_id;//空域所属无人机序号
			area_sky_informations.area_informations[i].upper_height_limit_valid_bit = blk_dtms_ctas_001.region_infos[i].reg_top_of_hei_valid;//区域高度上限有效位
			area_sky_informations.area_informations[i].lower_height_limit_valid_bit = blk_dtms_ctas_001.region_infos[i].reg_down_of_hei_valid;//区域高度下限有效位
			area_sky_informations.area_informations[i].upper_height_limit = blk_dtms_ctas_001.region_infos[i].top_of_hei;//区域高度上限
			area_sky_informations.area_informations[i].lower_height_limit = blk_dtms_ctas_001.region_infos[i].down_of_hei;//区域高度下限
			//圆区域赋值
			area_sky_informations.area_informations[i].cycles.radius = blk_dtms_ctas_001.region_infos[i].reg_circle.radious;//圆半径
			area_sky_informations.area_informations[i].cycles.longitude = blk_dtms_ctas_001.region_infos[i].reg_circle.center_lon_lat.longitude;//圆形经度
			area_sky_informations.area_informations[i].cycles.latitude = blk_dtms_ctas_001.region_infos[i].reg_circle.center_lon_lat.latitude;//圆形纬度
			//多边形区域赋值
			area_sky_informations.area_informations[i].polygonals.point_number = blk_dtms_ctas_001.region_infos[i].reg_ploygen.point_num;//多边形点数
			for(int j = 0;j < area_sky_informations.area_informations[i].polygonals.point_number;j++){
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude = blk_dtms_ctas_001.region_infos[i].reg_ploygen.points_lon_lat[j].longitude;//点经度
				area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude = blk_dtms_ctas_001.region_infos[i].reg_ploygen.points_lon_lat[j].latitude;//点纬度
			}
		}
	}


	//4.9 全局任务规划命令---赋值
	global_mission_planning_commandss.mission_type = blk_dtms_ctas_001.task_type;//任务类型
	global_mission_planning_commandss.tactical_warfare_options = blk_dtms_ctas_001.zhanshu_sel;//战术战法选择
	global_mission_planning_commandss.target_category = blk_dtms_ctas_001.target_type;//目标类别
	global_mission_planning_commandss.area_point_num = area_sky_informations.area_number;//任务区/应召点个数
	for(int i = 0;i < global_mission_planning_commandss.area_point_num;i++){
		global_mission_planning_commandss.area_point_number[i] = area_sky_informations.area_informations[i].area_code;//区域编号赋值
	}

}

/*
 * 单任务战术战法推荐指令基础信息赋值
 * */
void init_zhanshutuijians_single(){

	//3.2 兵力信息
	drone_state_informations.drone_number = blk_dtms_ctas_002.solider_num - 1;//无人机数量 = 兵力数量 - 1（有人机数量）

	//取出有效的无人机
	int cnt = 0 ;
	for(unsigned int i = 1;i < 5;i++)
	{
		if(blk_dtms_ctas_002.solider_infos[i].solider_type == 2){
			drone_state_informations.drone_specific_informations[cnt].platform_model = 1;
			drone_state_informations.drone_specific_informations[cnt].platform_serial_num = i;
			drone_state_informations.drone_specific_informations[cnt].platform_num = blk_dtms_ctas_002.solider_infos[i].solider_id;
			cnt++;
		}
	}

	//3.4 综合态势
	integrated_postures.drone_num = drone_state_informations.drone_number;
	int droneNum = 0;//当前接收到的无人机个数
	for(unsigned int i = 0;i < 4;i++){
		if(blk_dtms_ctas_002.solider_infos[i].solider_type == 1){//为有人机信息赋值
			integrated_postures.longitude = blk_dtms_ctas_002.solider_infos[i].lon_lat_info.longitude;
			integrated_postures.latitude = blk_dtms_ctas_002.solider_infos[i].lon_lat_info.latitude;
			integrated_postures.ABSOLUTE_PRESSURE_ALT_A = blk_dtms_ctas_002.solider_infos[i].height;
			integrated_postures.GROUND_SPEED = blk_dtms_ctas_002.solider_infos[i].speed;
			integrated_postures.direction = blk_dtms_ctas_002.solider_infos[i].hangxiang;
		}else if(blk_dtms_ctas_002.solider_infos[i].solider_type == 2){//为无人机信息赋值
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_serial_number = i;
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_number = blk_dtms_ctas_002.solider_infos[i].solider_id;//士兵序号
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_longitude_and_latitude.longitude = blk_dtms_ctas_002.solider_infos[i].lon_lat_info.longitude;//经度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_longitude_and_latitude.latitude = blk_dtms_ctas_002.solider_infos[i].lon_lat_info.latitude;//纬度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_height = blk_dtms_ctas_002.solider_infos[i].height;//高度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_speed = blk_dtms_ctas_002.solider_infos[i].speed;//速度
			integrated_postures.integrated_posture_drone_informations[droneNum].drone_direction = blk_dtms_ctas_002.solider_infos[i].hangxiang;//航向
			droneNum++;
		}
	}

	//3.10 任务区/空域信息
	area_sky_informations.area_number = blk_dtms_ctas_002.task_reg_num;//任务区数量
	for(int i = 0;i < area_sky_informations.area_number;i++){
		area_sky_informations.area_informations[i].area_code = blk_dtms_ctas_002.region_infos[i].reg_id;//区域编号
		area_sky_informations.area_informations[i].area_type = blk_dtms_ctas_002.region_infos[i].reg_type;//区域类型
		area_sky_informations.area_informations[i].area_source = blk_dtms_ctas_002.region_infos[i].reg_sour;//区域来源
		area_sky_informations.area_informations[i].area_shape = blk_dtms_ctas_002.region_infos[i].reg_shape;//区域形状
		//        area_sky_informations.area_informations[i].drone_number_valid_bit = DTMS_CTAS_singleRecommend.region_infos[i].kongyu_belong_to_uav_valid;
		area_sky_informations.area_informations[i].drone_numbe = blk_dtms_ctas_002.region_infos[i].kongyu_belong_to_uav_id;//空域所属无人机序号
		area_sky_informations.area_informations[i].upper_height_limit_valid_bit = blk_dtms_ctas_002.region_infos[i].reg_top_of_hei_valid;//区域高度上限有效位
		area_sky_informations.area_informations[i].lower_height_limit_valid_bit = blk_dtms_ctas_002.region_infos[i].reg_down_of_hei_valid;//区域高度下限有效位
		area_sky_informations.area_informations[i].upper_height_limit = blk_dtms_ctas_002.region_infos[i].top_of_hei;//区域高度上限
		area_sky_informations.area_informations[i].lower_height_limit = blk_dtms_ctas_002.region_infos[i].down_of_hei;//区域高度下限
		//圆区域赋值
		area_sky_informations.area_informations[i].cycles.radius = blk_dtms_ctas_002.region_infos[i].reg_circle.radious;//圆半径
		area_sky_informations.area_informations[i].cycles.longitude = blk_dtms_ctas_002.region_infos[i].reg_circle.center_lon_lat.longitude;//圆形经度
		area_sky_informations.area_informations[i].cycles.latitude = blk_dtms_ctas_002.region_infos[i].reg_circle.center_lon_lat.latitude;//圆形纬度
		//多边形区域赋值
		area_sky_informations.area_informations[i].polygonals.point_number = blk_dtms_ctas_002.region_infos[i].reg_ploygen.point_num;//多边形点数
		for(int j = 0;j < area_sky_informations.area_informations[i].polygonals.point_number;j++){
			area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude = blk_dtms_ctas_002.region_infos[i].reg_ploygen.points_lon_lat[j].longitude;//点经度
			area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude = blk_dtms_ctas_002.region_infos[i].reg_ploygen.points_lon_lat[j].latitude;//点纬度
		}
	}


	//4.9 全局任务规划命令---赋值
	global_mission_planning_commandss.mission_type = blk_dtms_ctas_002.task_type;//任务类型
	global_mission_planning_commandss.tactical_warfare_options = blk_dtms_ctas_002.zhanshu_sel;//战术战法选择
	global_mission_planning_commandss.target_category = blk_dtms_ctas_002.target_type;//目标类别
	global_mission_planning_commandss.area_point_num = area_sky_informations.area_number;//任务区/应召点个数
	for(int i = 0;i < global_mission_planning_commandss.area_point_num;i++){
		global_mission_planning_commandss.area_point_number[i] = area_sky_informations.area_informations[i].area_code;//区域编号赋值
	}

}

/*
 * CTAS-DTMS 战术战法规划结果 0x10000112
 * 将规划好的任务分配结果发送给DTMS
 * */
void send_single_result(){

	//组织当前平台和任务序列
	data_length = sizeof(Information_the_results_of_tasking);
	Send_Message(DDSTables.BLK_CTAS_DTMS_002.niConnectionId, 0, &transaction_id, &information_on_the_results_of_taskings, &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0){

	}
}


/*
 * CTAS-DTMS 战术战法规划结果 0x10000112
 * 将规划好的任务分配结果发送给DTMS
 * */
void send_tactics_result(){

	//组织当前平台和任务序列
	data_length = sizeof(Information_the_results_of_tasking);
	Send_Message(DDSTables.BLK_CTAS_DTMS_001.niConnectionId, 0, &transaction_id, &information_on_the_results_of_taskings, &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0){
		renwu_guihua_flag = 0;//发送任务分配结果结束
	}
}

/*
 * CTAS-DTMS 浮标布阵规划结果 0x10000113
 * */
void send_buoy_result()
{
	data_length = sizeof(BLK_CCC_OFP_302);
	Send_Message(DDSTables.BLK_CTAS_DTMS_005.niConnectionId, 0, &transaction_id, &buoy_arry_plan, &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0)
	{

	}
}

/*
 * CTAS-DTMS 吊声定测点布阵规划 0x10000114
 * */
void send_sonar_result()
{
	data_length = sizeof(BLK_CCC_OFP_403);
	Send_Message(DDSTables.BLK_CTAS_DTMS_006.niConnectionId, 0, &transaction_id, &sonar_detect_plan, &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0)
	{

	}
}

/*
 * CTAS-DTMS 任务区划分信息 0x10000115
 * */
void send_taskarea_result()
{
	//赋值
	taskarea_division.Plan_ID = information_on_the_results_of_taskings.program_number;//方案编号
	data_length = sizeof(BLK_CCC_OFP_005);
	Send_Message(DDSTables.BLK_CTAS_DTMS_007.niConnectionId, 0, &transaction_id, &taskarea_division, &message_type_id, data_length, &enRetCode);
	if(enRetCode == 0)
	{

	}
}

/*
 * 有人机航路信息发送 CTAS-DTMS
 * 通用航路、浮标布阵、吊声定测都分别分包发送
 * */
void send_manned_route(){
	for(int i = 0 ; i < CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number ; i ++)
	{
		send_manned_aircraft_route_drone_route_information(i);
	}
}

// 每包带头发送
void send_manned_aircraft_route_drone_route_information(int i){ // i 为有人机子任务

	/*
	 * CTAS-DTMS 通用航路 0x10000222
	 * 分为两个周期发送，每次40个航路点
	 * */
	//通用航路发送

	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i].waypoint_start_number = 0;
	// 头部分 8 字节
	memcpy(send_array.dataA,&(CTAS_DTMS_data_mannedRoute.common_carrier_routes),6);
	// 接着加入子任务
	memcpy(send_array.dataA + 6,&(CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i]),15);
	// 加入40个航迹点
	align_send_information(
			&(CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i].waypoint_informations[0]),\
			40 * sizeof (CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i].waypoint_informations[0]),\
			15 + 6
	);

	//发前40个航路点信息
	Send_Message(DDSTables.BLK_CTAS_DTMS_003.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
	if (enRetCode == 0) {
		//        qDebug() << "CTAS_DTMS_commonRoute_40 send OK, manned_index: " << i;
	} else {
		//        qDebug() << "CTAS_DTMS_commonRoute_40 send ERROR, manned_index: " << i;
	}


	// 继续发送有人机任务的后40个点
	if(CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i].waypoints_number > 40){

		CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i].waypoint_start_number = 40;
		memcpy(send_array.dataA,&(CTAS_DTMS_data_mannedRoute.common_carrier_routes),6);
		memcpy(send_array.dataA + 6,&(CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i]),15);
		align_send_information(
				&(CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i].waypoint_informations[40]),\
				40 * sizeof (CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[i].waypoint_informations[0]),\
				15 + 6
		);
		//发送后40个航路点信息
		Send_Message(DDSTables.BLK_CTAS_DTMS_003.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
		if (enRetCode == 0) {
			//            qDebug() << "CTAS_DTMS_commonRoute_80 send OK, manned_index: " << i;
		} else {
			//            qDebug() << "CTAS_DTMS_commonRoute_80 send ERROR, manned_index: " << i;
		}
	}
}


/*
 * 无人机航线发送函数 CTAS-DTMS
 * 无人机航线分包发送，将包头与航点拼接后发送
 * */
void send_drone_route(){

	for(int i = 0 ; i < integrated_postures.drone_num ; i ++)
	{
		for(int j = 0 ; j < CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].subtasks_number ; j ++)
		{
			send_drone_route_confirmation(i,j);
		}
	}
}

//3.8 子任务航线分包   头+规划方案（3个 每个方案里有各自的子任务）
//每次发头+规划方案中一个子任务
void send_drone_route_confirmation(int drone_index,int drone_subtask_index){

	//    //第一个子任务第一个航点为无人机当前位置
	//    if(drone_subtask_index == 0)
	//    {
	//        int waypoints_number = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations->waypoints_number;
	//        //航路点后移一位
	//        for(int i = waypoints_number - 1; i >=  0; i --)
	//        {
	//            if(i == 74)
	//                continue;
	//            CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations[drone_subtask_index].planning_information_waypoint_informations[i + 1].latitude =
	//                    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations[drone_subtask_index].planning_information_waypoint_informations[i].latitude;
	//            CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations[drone_subtask_index].planning_information_waypoint_informations[i + 1].longitude =
	//                    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations[drone_subtask_index].planning_information_waypoint_informations[i].longitude;
	//        }
	//        CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations[drone_subtask_index].planning_information_waypoint_informations[0].latitude =
	//                integrated_postures.integrated_posture_drone_informations[drone_index].drone_longitude_and_latitude.latitude;
	//        CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations[drone_subtask_index].planning_information_waypoint_informations[0].longitude =
	//                integrated_postures.integrated_posture_drone_informations[drone_index].drone_longitude_and_latitude.longitude;
	//        if(waypoints_number < 75)
	//            CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations->waypoints_number ++;
	//        //重新计算航点编号
	//        for(int j = 0 ; j < CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations->waypoints_number ; j ++)
	//        {
	//            CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations[drone_subtask_index].planning_information_waypoint_informations[j].hld_idx = j+1;
	//        }
	//    }
	//给无人机子任务序号赋值
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].subtask_index = drone_subtask_index;
	//分包发送无人机航线
	for(int i = 0;i <  CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations[drone_subtask_index].total_packet ; i++)
	{
		//包序号
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations[drone_subtask_index].packet_id = i;
		// 取头
		memcpy(send_array.dataA,&CTAS_DTMS_data_UAVRoute,9);
		// 加入规划方案前俩项目
		memcpy(send_array.dataA + 9,&(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index]),10);
		//取分包参数
		memcpy(send_array.dataA + 9 + 10,
				&(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations[drone_subtask_index])
				,26);
		//取相应的25包航路点信息
		memcpy(send_array.dataA + 9 + 10 + 26,
				&(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[drone_index].planning_informations[drone_subtask_index].planning_information_waypoint_informations[i*25])
				,sizeof(planning_information_waypoint_information)*25);
//		data_length = 9 + 10 + sizeof(planning_information_waypoint_information)*25;
		data_length = 9 + 10 + 26 + sizeof(planning_information_waypoint_information)*25;//260129
		Send_Message(DDSTables.BLK_CTAS_DTMS_004.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
		//        qDebug() << "BLK_DTMS_CTAS_008 send OK, drone_index: " << drone_index << ", subtask_index: " << drone_subtask_index<< data_length;
	}
}

//新航线生成
void new_route_generate(SGeo UAV_aircraft, float speed,SGeo goal_point , unsigned int uav_index){
	SGeo route_point[25];//生成的航线点集合
	unsigned short point_num = 0;//航线点数量
	//加入目标初始点
	route_point[0].Lon = goal_point.Lon;
	route_point[0].Lat = goal_point.Lat;
	point_num++;

	//航向有效
	if(blk_dtms_ctas_002.region_infos[0].point_infos[0].point_hangxiang_valid == 1)
	{
		float goal_vel = 0;
		float goal_course = blk_dtms_ctas_002.region_infos[0].point_infos[0].target_hangxiang; //目标航向 若没有的话就按正常的8字搜，不做目标点位置的偏移
		//航速有效
		if(blk_dtms_ctas_002.region_infos[0].point_infos[0].point_speed_valid == 1)
		{
			goal_vel = blk_dtms_ctas_002.region_infos[0].point_infos[0].target_speed;
		}
		else
		{
			goal_vel = 6;//目标航速  若没有则默认六节
		}
		//定义变量
		SPolar SPgoal1;
		SPolar SProute1[4];
		SGeo SPgoal_point1[3];
		//计算无人机到目标点的预计时间
		float time_to_target = 0;
		//速度小于100，用常规速度
		if(speed < 100)
			speed = 128;
		if(speed > 0)
		{
			double distance = calculate_distances(UAV_aircraft.Lat,UAV_aircraft.Lon,goal_point.Lat,goal_point.Lon);
			time_to_target = distance/speed;
		}
		else
		{
			time_to_target = 0;
		}
		//计算未来目标经过的点
		for(int i = 0 ; i < 3 ; i ++)
		{
			//求出当前循环中目标点的预测位置
			SPgoal1.R = (((4.5/60)*i + time_to_target)* goal_vel);
			SPgoal1.A = 90 - goal_course;
			SPgoal_point1[i] = Polar_to_Geo(goal_point, SPgoal1);
		}
		//计算航线
		for(int i = 0; i < 12 ; i++)
		{
			int j = i % 4;
			int n = i/4; // 求是第几个循环
			SProute1[j].R = 3;
			switch (i)
			{
			case 0:
				SProute1[j].A = (90 - goal_course) + 180;
				break;
			case 1:
				SProute1[j].A = (90 - goal_course) + 240;
				break;
			case 2:
				SProute1[j].A = (90 - goal_course) + 60;
				break;
			case 3:
				SProute1[j].A = (90 - goal_course);
				break;
			}
			route_point[i + 1] = Polar_to_Geo(SPgoal_point1[n], SProute1[j]);
			point_num++;
		}

	}
	else//航向无效，直接做8字搜索
	{
		SPolar SPgoal_point;//目标点极坐标
		SPolar SProute1[4];//所求航线点极坐标
		//若不存在目标航向
		SPgoal_point = Geo_to_Polar( goal_point , UAV_aircraft );
		//第一个航线点是无人机到目标点的延伸3km
		SProute1[0].R = 3;
		SProute1[0].A = SPgoal_point.A + 180;
		//第二个航线点是无人机到目标点的延伸
		SProute1[1].R = 3;
		SProute1[1].A = SPgoal_point.A + 240;
		//第三个航线点是无人机到目标点的延伸
		SProute1[2].R = 3;
		SProute1[2].A = SPgoal_point.A + 60;
		//第四个航线点是无人机到目标点的延伸
		SProute1[3].R = 3;
		SProute1[3].A = SPgoal_point.A;
		for(int i = 0; i < 12 ; i++)
		{
			int j = i % 4;
			//由极坐标求出第二个点的经纬度
			route_point[i + 1] = Polar_to_Geo(goal_point,SProute1[j]);
			point_num++;
		}
	}

	//增加最后一个矩形的中心点作为盘旋点 20251021new
	AreaRectVertex tmp;
	AreaRectCenter rtn;
	for(int k = 0; k < 4 ; k ++)
	{
		tmp.vertexA[k].latitude = route_point[k+9].Lat;
		tmp.vertexA[k].longitude = route_point[k+9].Lon;
	}
	//生成矩形中心点
	rtn = getAreaRectCenterByVertex(&tmp);

	route_point[13].Lat = rtn.center.latitude;
	route_point[13].Lon = rtn.center.longitude;

	point_num++;

	//赋值航路信息
	CTAS_DTMS_data_UAVRoute.program_number = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.plan_type = 4;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].subtasks_number = 1;

	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].mission_type = 6;//磁探跟踪
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].waypoints_number = point_num;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].subtask_ID_number = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].AtomicTimeUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].AtomicHighlyUpper = 1;
	CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].total_packet = 1;

	for(int i = 0;i < point_num;i++){
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].causality = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].longitude = route_point[i].Lon;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].latitude = route_point[i].Lat;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].height = blk_dtms_ctas_010[uav_index].CTGZ;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].speed = 30;
		//最后一个点为盘旋点
		if(i == point_num - 1)
		{
			//航路点待机时间/圈数/循环次数
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber
			= PX_CIRCLE;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
			//航路点待机半径
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].standby_radius
			= PX_RAD;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].standby_radius_valid_bit = 1;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].causality = 3;
			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[0].planning_information_waypoint_informations[i].standby_type = 2;//圈数
		}
	}
	//另一架机做盘旋
	if(integrated_postures.drone_num == 2)
	{
		int px_uav = 0;
		px_uav = (uav_index == 0) ? 1:0;
		//目标航向反方向5km的点计算
		double bearing;
		SGeo px_point;
		bearing = blk_dtms_ctas_002.region_infos[0].point_infos[0].target_hangxiang + 180;
		if(bearing > 360)
			bearing -= 360;
		px_point = calculate_new_coordinate(goal_point,5.0,bearing);
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].subtasks_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].drone_serial_number = drone_state_informations.drone_specific_informations[px_uav].platform_serial_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].drone_num = drone_state_informations.drone_specific_informations[px_uav].platform_num;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].mission_type = 14;//盘旋等待
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].AtomicTimeUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].AtomicHighlyUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].total_packet = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].waypoints_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].subtask_ID_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].AtomicTimeUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].AtomicHighlyUpper = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].longitude = px_point.Lon;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].latitude = px_point.Lat;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].height = blk_dtms_ctas_010[uav_index].CTGZ;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].speed = 30;
		//航路点待机时间/圈数/循环次数
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber
		= PX_CIRCLE;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
		//航路点待机半径
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].standby_radius
		= PX_RAD;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].causality = 3;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[px_uav].planning_informations[0].planning_information_waypoint_informations[0].standby_type = 2;//圈数
	}

}

/*
 * 发送数据对齐工具
 * 将发送的数据send_struct存入send_array，并对发送长度进行赋值
 * */
void align_send_information(void *send_struct,int length,int startPos){//发送信息数据对齐
	memcpy(send_array.dataA+ startPos, send_struct,length);
	data_length = length - startPos;
}


void yinzhao_point_init()
{
	region_info area;
	area.reg_shape = 1;
	area.reg_circle.center_lon_lat.latitude = blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.latitude;
	area.reg_circle.center_lon_lat.longitude = blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.longitude;
	area.reg_circle.radious =  5;//圆半径
	task_area_division(&area,blk_dtms_ctas_001.solider_num);
}
void yingzhao_area_divide(point_coordinate point, unsigned int time){
	//    area_sky_informations.area_number = 2;
	//
	//    area_sky_informations.area_informations[0].area_shape = 1;//圆
	//    area_sky_informations.area_informations[0].area_code = 1;//区域编号
	//    area_sky_informations.area_informations[0].cycles.longitude = point.longitude;
	//    area_sky_informations.area_informations[0].cycles.latitude = point.latitude;
	//
	//    area_sky_informations.area_informations[1].cycles.radius = time * 5;//圆半径
	//    area_sky_informations.area_informations[0].cycles.radius = time * 5 - 2;//圆半径
	//
	//    //外侧圆
	//    area_sky_informations.area_informations[1].area_shape = 1;
	//    area_sky_informations.area_informations[1].area_code = 2;//区域编号
	//    area_sky_informations.area_informations[1].area_type = 5;
	//    area_sky_informations.area_informations[1].area_source = 7;
	//    area_sky_informations.area_informations[1].cycles.longitude = point.longitude;
	//    area_sky_informations.area_informations[1].cycles.latitude = point.latitude;

	region_info area;
	area.reg_shape = 1;
	area.reg_circle.center_lon_lat.latitude = point.latitude;
	area.reg_circle.center_lon_lat.longitude = point.longitude;
	area.reg_circle.radious = time * 5;//圆半径
	task_area_division(&area,2);
}

/*
 * 应召---战术一
 * */
void yingzhao_firstStrategy(){


}
//应召战术一任务区规划
void yingzhao_firstStrategy_missionDistributeInformation(){
	point_coordinate point;//应召点
	unsigned int time = 1;//应召时间

	point.latitude =
			blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.latitude;

	point.longitude =
			blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.longitude;
	//应召点/应召时间赋值
	//应召区域划分
	yingzhao_area_divide(point,time);

	mannedAreaSorted();//组织任务区顺序
	//分配有人机执行吊声定测任务
	//任务分配信息赋值
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定
	//协同搜潜方式---4.9 战术战法选择

	//方案总时间---待定
	//任务平台个数---3.2 无人机个数n + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number + 1;


	//平台子任务个数---吊声搜索 + 返航任务
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID

	//任务序列信息
	for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 2;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[0].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 1;
	}
	//    //分配有人机返航任务
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].target_number = 0;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].sequence_type = 11;


	//分配无人机执行磁探搜索任务和通信中继任务
	int citan_search_No = 0;//磁探搜索无人机序号，取离区域最近的无人机且搭载磁探搜索的
	int tongxinzhongji_No = 0;//通信中继无人机序号，取离区域第二近的无人机且搭载中继的
	for(int i = 0;i < drone_state_informations.drone_number;i++){
		if(get_bit(drone_state_informations.drone_specific_informations[i].platform_load,3) == 1 && UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[citan_search_No],integrated_postures.integrated_posture_drone_informations[i],area_sky_informations.area_informations[0])){
			//当前无人机带有磁探,并且当前无人机的距离较近
			if(get_bit(drone_state_informations.drone_specific_informations[citan_search_No].platform_load,2) == 1 && UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[tongxinzhongji_No],integrated_postures.integrated_posture_drone_informations[citan_search_No],area_sky_informations.area_informations[0])){
				tongxinzhongji_No = citan_search_No;
			}
			citan_search_No = i;
		}else if(get_bit(drone_state_informations.drone_specific_informations[i].platform_load,2) == 1 && UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[tongxinzhongji_No],integrated_postures.integrated_posture_drone_informations[i],area_sky_informations.area_informations[0])){
			tongxinzhongji_No = i;
		}
	}


	/*
	 * 无人机任务信息分配结果
	 * */
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---返航+
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= 1;

		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 12;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 0;
		}
	}
	//分配磁探搜索和通信中继任务
	//磁探
	information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[1].area_code;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[1].task_sequence_informations[0].sequence_type = 5;//磁探搜索
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[citan_search_No + 1].task_sequence_informations[0].target_number = area_sky_informations.area_informations[0].area_code;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[citan_search_No + 1].task_sequence_informations[0].sequence_type = 5;//磁探搜索
	//通信
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[tongxinzhongji_No + 1].task_sequence_informations[0].sequence_type = 4;//通信中继

	//    //分配无人机返航任务
	//    for(unsigned int i = 1;i <= drone_state_informations.drone_number;i++){
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks - 1].target_number = 0;
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks - 1].sequence_type = 11;
	//
	//    }

	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}

	//方案时长航程计算
	uav_time_range_calc();
}

int get_bit(unsigned short data, int n) {
	unsigned short mask = 1 << n; // 创建掩码
	return (data & mask) >> n; // 移位并返回第n位的值
}

/*
 * 应召---战术二
 * */
void yingzhao_secondStrategy(){

}
void yingzhao_secondStrategy_missionDistributeInformation(){
	point_coordinate point;//应召点
	unsigned int time = 1;//应召时间
	point.latitude =
			blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.latitude;

	point.longitude =
			blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.longitude;
	//应召区域划分
	yingzhao_secondStrategy_areaDivid(point,time);
	mannedAreaSorted();//组织任务区顺序
	//分配有人机执行吊声定测任务
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定
	//协同搜潜方式---4.9 战术战法选择

	//方案总时间---待定
	//任务平台个数---3.2 无人机个数n + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number + 1;


	//平台子任务个数---吊声搜索 + 返航任务
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID

	//任务序列信息
	for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 2;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[0].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 1;
	}
	//分配有人机返航任务
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].target_number = 0;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].sequence_type = 11;

	/*
	 * 无人机任务信息分配结果
	 * */
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数 + 返航
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= 1;

		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 0;
		}

	}
	//确定一轮任务时无人机的任务分配
	UAVAreaDistribute();
	//分配无人机返航任务
	//    for(int i = 1;i <= drone_state_informations.drone_number;i++){
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = 0;
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 11;
	//
	//    }
	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}

	//方案时长航程计算
	uav_time_range_calc();

}
void yingzhao_secondStrategy_areaDivid(point_coordinate point,unsigned int time){//应召战术二的区域划分
	//    area_sky_informations.area_number = drone_state_informations.drone_number + 1;
	//    area_sky_informations.area_informations[0].area_code = 1;
	//    area_sky_informations.area_informations[0].area_shape = 1;//圆
	//    area_sky_informations.area_informations[0].cycles.longitude = point.longitude;
	//    area_sky_informations.area_informations[0].cycles.latitude = point.latitude;
	//
	//    area_sky_informations.area_informations[1].cycles.radius = time * 5;//圆半径
	//    area_sky_informations.area_informations[0].cycles.radius = time * 5 - 2;//圆半径
	//
	//    //外侧圆
	//    area_sky_informations.area_informations[1].area_shape = 1;
	//    area_sky_informations.area_informations[1].area_type = 5;
	//    area_sky_informations.area_informations[1].area_source = 7;
	//    area_sky_informations.area_informations[1].cycles.longitude = point.longitude;
	//    area_sky_informations.area_informations[1].cycles.latitude = point.latitude;
	//    GeoLibDas centre;
	//    centre.longitude = point.longitude;
	//    centre.latitude = point.latitude;
	//    double radius_large = time * 5;
	//    double radius_little = time * 5 - 2;
	//    PolygonLibDas* result;
	//    result = loop_to_RectLibDas(centre,radius_large,radius_little,(int)(drone_state_informations.drone_number));
	//    for(int i = 1;i < (int)(drone_state_informations.drone_number);i++){
	//        area_sky_informations.area_informations[i].area_shape = 2;
	//        area_sky_informations.area_informations[i].area_code = i+1;
	//        area_sky_informations.area_informations[i].area_type = 5;
	//        area_sky_informations.area_informations[i].polygonals.point_number = result[i-1].sumVertexs;
	//        for(int j = 0;j < result[i-1].sumVertexs; j++){
	//            area_sky_informations.area_informations[i].polygonals.point_coordinates[j].longitude = result[i-1].vertexA[j].longitude;
	//            area_sky_informations.area_informations[i].polygonals.point_coordinates[j].latitude = result[i-1].vertexA[j].latitude;
	//        }
	//    }
	region_info area;
	area.reg_shape = 1;
	area.reg_circle.center_lon_lat.latitude = point.latitude;
	area.reg_circle.center_lon_lat.longitude = point.longitude;
	area.reg_circle.radious = time * 5;//圆半径
	task_area_division(&area,2);
}

/*
 * 应召---战术三
 * */
void yingzhao_thirdStrategy(){

}
void yingzhao_thirdStrategy_missionDistributeInformation(){
	point_coordinate point;//应召点
	unsigned int time = 1;//应召时间
	point.latitude =
			blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.latitude;

	point.longitude =
			blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.longitude;
	//区域划分
	yingzhao_thirdStrategy_areaDivid(point,time);
	mannedAreaSorted();//组织任务区顺序
	//分配有人机执行吊声定测任务
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定
	//协同搜潜方式---4.9 战术战法选择

	//方案总时间---待定
	//任务平台个数---3.2 无人机个数n + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number + 1;


	//平台子任务个数---浮标布放 +...+ 返航任务
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks = 2;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID

	//任务序列信息
	for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[0].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 0;
	}

	//如果无人机数量大于1,则有人机第二阶段待机,否则有人机第二阶段执行浮标监听任务
	if(drone_state_informations.drone_number > 1){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].sequence_type = 12;
	}else{
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].sequence_type = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].availability_of_routes = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].presence_of_buoy_array = 0;
	}
	//    //分配有人机返航任务
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].target_number = 0;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].sequence_type = 11;


	/*
	 * 无人机任务信息分配结果
	 * */
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数 + 返航
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= 2;

		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 5;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[1].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 0;
		}

	}

	if(drone_state_informations.drone_number > 1){//如果只有一架无人机,则该无人机前两阶段都执行磁探搜索任务,则无需改动,因此,只需要在无人机数量大于1的时候进行判断
		//找出磁探搜索的无人机
		int citan_No = 0;//磁探搜索无人机的序号
		int fubiao_No = 0;//浮标监听无人机序号
		for(int i = 0;i < drone_state_informations.drone_number;i++){
			if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[citan_No],integrated_postures.integrated_posture_drone_informations[i],area_sky_informations.area_informations[0])){
				//当前无人机带有磁探,并且当前无人机的距离较近
				if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[fubiao_No],integrated_postures.integrated_posture_drone_informations[citan_No],area_sky_informations.area_informations[0])){
					fubiao_No = citan_No;
				}
				citan_No = i;
			}else if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[fubiao_No],integrated_postures.integrated_posture_drone_informations[i],area_sky_informations.area_informations[0])){
				fubiao_No = i;
			}
		}
		//把其他无人机的第一阶段任务变成待机
		for(int i = 1;i <= drone_state_informations.drone_number;i++){
			if(i != citan_No + 1){
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].sequence_type = 12;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].target_number = 0;
			}
		}
		//把其他无人机的第一阶段任务变成待机
		for(int i = 1;i <= drone_state_informations.drone_number;i++){
			if(i != fubiao_No + 1 && i != citan_No + 1){
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[1].sequence_type = 12;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[1].target_number = 0;
			}
		}
		//分配浮标监听无人机第二次任务
		information_on_the_results_of_taskings.formation_synergy_mission_programs[fubiao_No + 1].task_sequence_informations[1].sequence_type = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[fubiao_No + 1].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;
	}

	//    //分配无人机返航任务
	//    for(int i = 1;i <= drone_state_informations.drone_number;i++){
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = 0;
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 11;
	//
	//    }

	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}

	//方案时长航程计算
	uav_time_range_calc();

}
void yingzhao_thirdStrategy_areaDivid(point_coordinate point,unsigned int time){
	//    area_sky_informations.area_number = 2;
	//
	//    area_sky_informations.area_informations[0].area_shape = 1;//圆
	//    area_sky_informations.area_informations[0].area_code = 1;//区域编号
	//    area_sky_informations.area_informations[0].cycles.longitude = point.longitude;
	//    area_sky_informations.area_informations[0].cycles.latitude = point.latitude;
	//
	//    area_sky_informations.area_informations[1].cycles.radius = time * 5;//圆半径
	//    area_sky_informations.area_informations[0].cycles.radius = time * 5 - 2;//圆半径
	//
	//    //外侧圆
	//    area_sky_informations.area_informations[1].area_shape = 1;
	//    area_sky_informations.area_informations[1].area_code = 2;//区域编号
	//    area_sky_informations.area_informations[1].area_type = 5;
	//    area_sky_informations.area_informations[1].area_source = 7;
	//    area_sky_informations.area_informations[1].cycles.longitude = point.longitude;
	//    area_sky_informations.area_informations[1].cycles.latitude = point.latitude;
	region_info area;
	area.reg_shape = 1;
	area.reg_circle.center_lon_lat.latitude = point.latitude;
	area.reg_circle.center_lon_lat.longitude = point.longitude;
	area.reg_circle.radious = time * 5;//圆半径
	task_area_division(&area,2);
}


/*
 * 应召---战术四
 * */
void yingzhao_fourthStrategy(){

}
void yingzhao_fourthStrategy_missionDistributeInformation(){
	point_coordinate point;//应召点
	unsigned int time = 1;//应召时间

	point.latitude =
			blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.latitude;

	point.longitude =
			blk_dtms_ctas_001.region_infos[0].point_infos[0].point_lon_lat.longitude;

	//区域划分
	yingzhao_fourthStrategy_areaDivid(point,time);
	mannedAreaSorted();//组织任务区顺序
	//分配有人机执行浮标布放任务
	//任务分配信息赋值
	//    information_on_the_results_of_taskings.program_number += 1;//方案编号
	information_on_the_results_of_taskings.tasking_release = 2;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 0;//方案属性---待定
	//协同搜潜方式---4.9 战术战法选择

	//方案总时间---待定
	//任务平台个数---3.2 无人机个数n + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = drone_state_informations.drone_number + 1;


	//平台子任务个数---吊声搜索 + 返航任务
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks = 2;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID

	//任务序列信息
	for(int i = 0;i < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks + 1;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].subtask_ID_number = (unsigned)(i + 1);
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].sequence_type = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].type_of_mission_point_area = 3;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number = area_sky_informations.area_informations[0].area_code;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].availability_of_routes = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_buoy_array = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].presence_of_fixed_point_arrays = 0;
	}

	//有人机第二阶段在外侧执行吊声定测任务
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].sequence_type = 2;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].target_number = area_sky_informations.area_informations[1].area_code;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].presence_of_fixed_point_arrays = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[1].presence_of_buoy_array = 0;
	//若无人机数量大于等于三，则有人机第三阶段没有任务，否则有人机执行浮标监听
	if(drone_state_informations.drone_number >= 3){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks++;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[2].sequence_type = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[2].availability_of_routes = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[2].target_number = area_sky_informations.area_informations[0].area_code;
	}

	//    //分配有人机返航任务
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].target_number = 0;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks - 1].sequence_type = 11;

	/*
	 * 无人机任务信息分配结果
	 * */
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = drone_state_informations.drone_specific_informations[i - 1].platform_serial_num;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		//平台任务时---待定
		//平台子任务个数---任务区/应召点个数 + 返航
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks
		= 2;

		//为任务基础数据进行赋值
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++){
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].subtask_ID_number = (unsigned)(j + 1);
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].modify = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 5;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].type_of_mission_point_area = 3;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number = area_sky_informations.area_informations[1].area_code;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].completion_time_valid_Bits = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_height_effective_position = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = 1000;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].availability_of_routes = 1;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_buoy_array = 0;
			information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].presence_of_fixed_point_arrays = 0;
		}

	}

	//找出磁探搜索的无人机
	//    int citan_No = 0;//磁探搜索无人机的序号
	//    int tongxin_No = 0;//通信中继无人机序号
	//    for(int i = 0;i < drone_state_informations.drone_number;i++){
	//        if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[citan_No],integrated_postures.integrated_posture_drone_informations[i],area_sky_informations.area_informations[0])){
	//            //当前无人机带有磁探,并且当前无人机的距离较近
	//            if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[tongxin_No],integrated_postures.integrated_posture_drone_informations[citan_No],area_sky_informations.area_informations[0])){
	//                tongxin_No = citan_No;
	//            }
	//            citan_No = i;
	//        }else if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[tongxin_No],integrated_postures.integrated_posture_drone_informations[i],area_sky_informations.area_informations[0])){
	//            tongxin_No = i;
	//        }
	//    }
	//    //把其他无人机的第一阶段任务变成待机
	//    for(int i = 1;i <= drone_state_informations.drone_number;i++){
	//        if(i != citan_No + 1){
	//            information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].sequence_type = 12;
	//            information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].target_number = 0;
	//        }
	//    }

	//把其他无人机的第二阶段任务变成待机
	for(int i = 1;i <= drone_state_informations.drone_number;i++){
		//        if(i != tongxin_No + 1 && i != citan_No + 1){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[1].sequence_type = 12;
		//            information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[1].target_number = 0;
		//        }
	}
	//    //分配通信中继无人机第二次任务
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[tongxin_No + 1].task_sequence_informations[1].sequence_type = 4;
	//    information_on_the_results_of_taskings.formation_synergy_mission_programs[tongxin_No + 1].task_sequence_informations[1].target_number = 0;
	//浮标监听
	//    if(drone_state_informations.drone_number >= 3){//若无人机数量大于等于3
	//        int fubiao_index = -1;//浮标监听无人机序号
	//        for(int i = 0;i < drone_index;i++){
	//            if(i != citan_No && i != tongxin_No){
	//                if(fubiao_index == -1){
	//                    fubiao_index = i;
	//                }else if(UAVdistanceComparison(integrated_postures.integrated_posture_drone_informations[tongxin_No],integrated_postures.integrated_posture_drone_informations[i],area_sky_informations.area_informations[0])){
	//                    fubiao_index = i;
	//                }
	//            }
	//        }
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[fubiao_index + 1].task_sequence_informations[1].sequence_type = 1;
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[fubiao_index + 1].task_sequence_informations[1].target_number = area_sky_informations.area_informations[0].area_code;
	//    }

	//    //分配无人机返航任务
	//    for(int i = 1;i <= drone_state_informations.drone_number;i++){
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].target_number = 0;
	//        information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[global_mission_planning_commandss.area_point_num].sequence_type = 11;
	//
	//    }
	//载荷重规划
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 &&
					integrated_postures.integrated_posture_drone_informations[i-1].ct == 0)
			{
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type = 7;
			}
		}
	}

	//方案时长航程计算
	uav_time_range_calc();
}
void yingzhao_fourthStrategy_areaDivid(point_coordinate point,unsigned int time){
	//    area_sky_informations.area_number = 2;
	//
	//    area_sky_informations.area_informations[0].area_shape = 1;//圆
	//    area_sky_informations.area_informations[0].area_code = 1;//区域编号
	//    area_sky_informations.area_informations[0].cycles.longitude = point.longitude;
	//    area_sky_informations.area_informations[0].cycles.latitude = point.latitude;
	//
	//    area_sky_informations.area_informations[1].cycles.radius = time * 5;//圆半径
	//    area_sky_informations.area_informations[0].cycles.radius = time * 5 - 2;//圆半径
	//
	//    //外侧圆
	//    area_sky_informations.area_informations[1].area_shape = 1;
	//    area_sky_informations.area_informations[1].area_code = 2;//区域编号
	//    area_sky_informations.area_informations[1].area_type = 5;
	//    area_sky_informations.area_informations[1].area_source = 7;
	//    area_sky_informations.area_informations[1].cycles.longitude = point.longitude;
	//    area_sky_informations.area_informations[1].cycles.latitude = point.latitude;
	region_info area;
	area.reg_shape = 1;
	area.reg_circle.center_lon_lat.latitude = point.latitude;
	area.reg_circle.center_lon_lat.longitude = point.longitude;
	area.reg_circle.radious = time * 5;//圆半径
	task_area_division(&area,2);
}


/*
 * 应召－－－战术五
 * */
void yingzhao_fifthStrategy(){

}
void yingzhao_fifthStrategy_missionDistributeInformation(){
	//
}
void yingzhao_fifthStrategy_areaDivid(){

}



/********************************任务重规划**************************************************/
/*
 * 任务重规划－－－接收－规划－发送
 * 根据修改的子任务，重新生成航线。
 * */
void task_replanning(){

	//接收任务重规划指令
	Receive_Message(DDSTables.BLK_DTMS_CTAS_003.niConnectionId, 0, &transaction_id, &information_on_the_results_of_taskings, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		//修改
		information_on_the_results_of_taskings.tasking_release = 3;
		information_on_the_results_of_taskings.manual_modification = 1;
		//任务分配结果
		send_tactics_result();
	}

	Receive_Message(DDSTables.BLK_DTMS_CTAS_004.niConnectionId, 0, &transaction_id, &taskarea_division, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		//重规划
	}

	//接收修改航线
	Receive_Message(DDSTables.BLK_DTMS_CTAS_008.niConnectionId, 0, &transaction_id, &send_array.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0)
	{
		//清空航线空域信息
		memset(&blk_ctas_dtms_010,0,sizeof(BLK_CTAS_DTMS_010));
		//取头
		memcpy(&CTAS_DTMS_data_UAVRoute,send_array.dataA,9);
		memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0],send_array.dataA+9,10);
		memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1],send_array.dataA+9+10,26);
		//取航线信息
		unsigned int index = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].packet_id;
		memcpy(&CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].planning_information_waypoint_informations[index*25]
							 ,send_array.dataA+9+10+26
							 ,sizeof(planning_information_waypoint_information)*25);
		//判断是否发送完成
		if( (index+1) == CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].total_packet)
		{
			//只有一个航路点为盘旋或悬停点
			if(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].waypoints_number == 1)
			{
				XT_area_calc(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].planning_information_waypoint_informations[0].longitude,
						CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].planning_information_waypoint_informations[0].latitude,
						1);
			}
			else if(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].mission_type == 5 ||
					CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].mission_type == 7)
			{
				MabrGeo tmp;
				//找到任务区，把任务区当做空域
				for(int k = 0;k < area_sky_informations.area_number;k++)
				{
					//找到对应的任务区域
					if(CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].planning_informations[1].area_point_line_goal_number == area_sky_informations.area_informations[k].area_code)
					{
						//圆形
						if(area_sky_informations.area_informations[k].area_shape == 1)
						{
							RectLibDas *rectLibDas;
							GeoLibDas center;
							double radius = 0;
							center.longitude = area_sky_informations.area_informations[k].cycles.longitude;
							center.latitude = area_sky_informations.area_informations[k].cycles.latitude;
							radius = area_sky_informations.area_informations[k].cycles.radius;
							//转换为矩形
							rectLibDas = cycle_to_RectLibDas(center,radius);
							for(int m = 0 ; m < 4 ; m ++)
							{
								tmp.vertexA[m].longitude = rectLibDas->vertexA[m].longitude;
								tmp.vertexA[m].latitude = rectLibDas->vertexA[m].latitude;
							}
						}
						//矩形
						else if(area_sky_informations.area_informations[k].area_shape == 2)
						{
							for(int m = 0 ; m < 4 ; m ++)
							{
								tmp.vertexA[m].longitude = area_sky_informations.area_informations[k].polygonals.point_coordinates[m].longitude;
								tmp.vertexA[m].latitude = area_sky_informations.area_informations[k].polygonals.point_coordinates[m].latitude;
							}

						}
					}
				}
				memcpy(&blk_ctas_dtms_010.airway_area[1],&tmp,sizeof(MabrGeo));
				blk_ctas_dtms_010.id[1] = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[0].drone_serial_number;
			}
			else
			{
				//生成空域
				single_air_area(0);
			}
			//发送空域
			send_airway_area();
		}
	}

}

void single_air_area(unsigned int uav_index)
{
	//获取有、无人机的航点
	Point points[250];
	Point hull[250];
	memset(&points[0],0,sizeof(Point)*250);
	memset(&hull[0],0,sizeof(Point)*250);
	memset(&blk_ctas_dtms_010,0,sizeof(BLK_CTAS_DTMS_010));
	int num = 0;

	//无人机航线获取
	num = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].waypoints_number;
	for(int j = 0 ; j < num ; j ++)
	{
		points[j].lat = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[j].latitude;
		points[j].lon = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].planning_informations[1].planning_information_waypoint_informations[j].longitude;
	}

	//计算空域
	MabrGeo tmp;
	tmp = air_area(points,hull,num);
	if(num == 2)
	{
		//圆形
		if(area_sky_informations.area_informations[0].area_shape == 1)
		{
			RectLibDas *rectLibDas;
			GeoLibDas center;
			double radius = 0;
			center.longitude = area_sky_informations.area_informations[0].cycles.longitude;
			center.latitude = area_sky_informations.area_informations[0].cycles.latitude;
			radius = area_sky_informations.area_informations[0].cycles.radius;
			//转换为矩形
			rectLibDas = cycle_to_RectLibDas(center,radius);
			for(int m = 0 ; m < 4 ; m ++)
			{
				tmp.vertexA[m].longitude = rectLibDas->vertexA[m].longitude;
				tmp.vertexA[m].latitude = rectLibDas->vertexA[m].latitude;
			}
		}
		//矩形
		else if(area_sky_informations.area_informations[0].area_shape == 2)
		{
			for(int m = 0 ; m < 4 ; m ++)
			{
				tmp.vertexA[m].longitude = area_sky_informations.area_informations[0].polygonals.point_coordinates[m].longitude;
				tmp.vertexA[m].latitude = area_sky_informations.area_informations[0].polygonals.point_coordinates[m].latitude;
			}

		}
	}
	memcpy(&blk_ctas_dtms_010.airway_area[1],&tmp,sizeof(MabrGeo));
	//有效空域
	if(num > 0)
	{
		blk_ctas_dtms_010.id[1] = CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[uav_index].drone_serial_number;
	}

}

PolygonLibDas* loop_to_RectLibDas(GeoLibDas centre,double radius_large,double radius_little,int num){
	//如果生成区域的数量是2
	if(num==2){
		//计算两个三角形的三个顶点坐标
		double half_bottom = sqrt(radius_large*radius_large - radius_little*radius_little);
		double left_triangle_leftPoint_x = centre.longitude - radius_large;
		double left_triangle_leftPoint_y = centre.latitude;
		double left_triangle_topPoint_x = centre.longitude - radius_little;
		double left_triangle_topPoint_y = centre.latitude - half_bottom;
		double left_triangle_bottomPoint_x = centre.longitude - radius_little;
		double left_triangle_bottomPoint_y = centre.latitude + half_bottom;

		double right_triangle_rightPoint_x = centre.longitude + radius_large;
		double right_triangle_rightPoint_y = centre.latitude;
		double right_triangle_topPoint_x = centre.longitude + radius_little;
		double right_triangle_topPoint_y = centre.latitude - half_bottom;
		double right_triangle_bottomPoint_x = centre.longitude + radius_little;
		double right_triangle_bottomPoint_y = centre.latitude + half_bottom;
		//结果存储
		PolygonLibDas result[2];
		result[0].sumVertexs = 3;
		result[1].sumVertexs = 3;
		result[0].vertexA[0].longitude = left_triangle_leftPoint_x;
		result[0].vertexA[0].latitude = left_triangle_leftPoint_y;
		result[0].vertexA[1].longitude = left_triangle_topPoint_x ;
		result[0].vertexA[1].latitude = left_triangle_topPoint_y;
		result[0].vertexA[2].longitude = left_triangle_bottomPoint_x;
		result[0].vertexA[2].latitude = left_triangle_bottomPoint_y;

		result[1].vertexA[0].longitude = right_triangle_rightPoint_x;
		result[1].vertexA[0].latitude = right_triangle_rightPoint_y;
		result[1].vertexA[1].longitude = right_triangle_topPoint_x ;
		result[1].vertexA[1].latitude = right_triangle_topPoint_y;
		result[1].vertexA[2].longitude = right_triangle_bottomPoint_x;
		result[1].vertexA[2].latitude = right_triangle_bottomPoint_y;

		return &result;
	}

	if(num==3){
		//计算两个三角形和一个下方矩形的节点坐标
		double half_bottom = sqrt(radius_large*radius_large - radius_little*radius_little);
		double left_triangle_leftPoint_x = centre.longitude - radius_large;
		double left_triangle_leftPoint_y = centre.latitude;
		double left_triangle_topPoint_x = centre.longitude - radius_little;
		double left_triangle_topPoint_y = centre.latitude - half_bottom;
		double left_triangle_bottomPoint_x = centre.longitude - radius_little;
		double left_triangle_bottomPoint_y = centre.latitude + half_bottom;

		double right_triangle_rightPoint_x = centre.longitude + radius_large;
		double right_triangle_rightPoint_y = centre.latitude;
		double right_triangle_topPoint_x = centre.longitude + radius_little;
		double right_triangle_topPoint_y = centre.latitude - half_bottom;
		double right_triangle_bottomPoint_x = centre.longitude + radius_little;
		double right_triangle_bottomPoint_y = centre.latitude + half_bottom;

		//矩形的四个顶点坐标
		double right_rectangle_top_leftPoint_x = centre.longitude - radius_little;
		double right_rectangle_top_leftPoint_y = centre.latitude + radius_little;
		double right_rectangle_top_rightPoint_x = centre.longitude + radius_little;
		double right_rectangle_top_rightPoint_y = centre.latitude + radius_little;
		double right_rectangle_bottom_leftPoint_x = centre.longitude - radius_little;
		double right_rectangle_bottom_leftPoint_y = centre.latitude + half_bottom;
		double right_rectangle_bottom_rightPoint_x = centre.longitude + radius_little;
		double right_rectangle_bottom_rightPoint_y = centre.latitude + half_bottom;

		//结果存储
		PolygonLibDas result[3];
		result[0].sumVertexs = 3;
		result[1].sumVertexs = 3;
		result[2].sumVertexs = 4;
		result[0].vertexA[0].longitude = left_triangle_leftPoint_x;
		result[0].vertexA[0].latitude = left_triangle_leftPoint_y;
		result[0].vertexA[1].longitude = left_triangle_topPoint_x ;
		result[0].vertexA[1].latitude = left_triangle_topPoint_y;
		result[0].vertexA[2].longitude = left_triangle_bottomPoint_x;
		result[0].vertexA[2].latitude = left_triangle_bottomPoint_y;

		result[1].vertexA[0].longitude = right_triangle_rightPoint_x;
		result[1].vertexA[0].latitude = right_triangle_rightPoint_y;
		result[1].vertexA[1].longitude = right_triangle_topPoint_x ;
		result[1].vertexA[1].latitude = right_triangle_topPoint_y;
		result[1].vertexA[2].longitude = right_triangle_bottomPoint_x;
		result[1].vertexA[2].latitude = right_triangle_bottomPoint_y;

		result[2].vertexA[0].longitude = right_rectangle_top_leftPoint_x;
		result[2].vertexA[0].latitude = right_rectangle_top_leftPoint_y;
		result[2].vertexA[1].longitude = right_rectangle_top_rightPoint_x;
		result[2].vertexA[1].latitude = right_rectangle_top_rightPoint_y;
		result[2].vertexA[2].longitude = right_rectangle_bottom_leftPoint_x;
		result[2].vertexA[2].latitude = right_rectangle_bottom_leftPoint_y;
		result[2].vertexA[3].longitude = right_rectangle_bottom_rightPoint_x;
		result[2].vertexA[3].latitude = right_rectangle_bottom_rightPoint_y;

		return &result;
	}

	return NULL;
}

RectLibDas* cycle_to_RectLibDas(GeoLibDas centre,double radius){
	//计算圆的内接正方形的四个坐标
	//    double side_length = sqrt(radius * radius + radius * radius);
	//    double left_square_leftPoint_x = centre.longitude - 0.5*side_length;
	//    double left_square_leftPoint_y = centre.latitude - 0.5*side_length;
	//    double left_square_rightPoint_x = centre.longitude + 0.5*side_length;
	//    double left_square_rightPoint_y = centre.latitude - 0.5*side_length;
	//    double left_square_bottomLeftPoint_x = centre.longitude - 0.5*side_length;
	//    double left_square_bottomLeftPoint_y = centre.latitude + 0.5*side_length;
	//    double left_square_bottomRightPoint_x = centre.longitude + 0.5*side_length;
	//    double left_square_bottomRightPoint_y = centre.latitude + 0.5*side_length;

	//    RectLibDas* result = new RectLibDas[1];
	//    result->vertexA[0].longitude = left_square_leftPoint_x;
	//    result->vertexA[0].latitude = left_square_leftPoint_y;
	//    result->vertexA[1].longitude = left_square_rightPoint_x;
	//    result->vertexA[1].latitude = left_square_rightPoint_y;
	//    result->vertexA[2].longitude = left_square_bottomLeftPoint_x;
	//    result->vertexA[2].latitude = left_square_bottomLeftPoint_y;
	//    result->vertexA[3].longitude = left_square_bottomRightPoint_x;
	//    result->vertexA[3].latitude = left_square_bottomRightPoint_y;

	//    return result;

	RectLibDas result;
	//内接矩形
	//    double delta = radius / sqrt(2.0);
	//    double dx[4] = { delta , -delta , -delta , delta };
	//    double dy[4] = { delta , delta , -delta , -delta };
	//
	//    //纬度转换为弧度
	//    double lat0_rad = centre.latitude * (M_PI / 180.0);
	//    for(int i = 0; i < 4 ; i ++)
	//    {
	//        //计算经度变化
	//        double dlat = (dy[i] / 6371.0) * (180.0 / M_PI);
	//
	//        //计算纬度变化
	//        double dlon = (dx[i] / (6371.0 * cos(lat0_rad)) ) * (180.0 / M_PI);
	//
	//        result.vertexA[i].latitude  = centre.latitude   + dlat;
	//        result.vertexA[i].longitude = centre.longitude  + dlon;
	//    }

	double delta_phi_rad = 0;
	double delta_phi_deg = 0;
	double center_lat_rad = 0;
	double cos_center_lat = 0;
	double delta_lambda_rad = 0;
	double delta_lambda_deg = 0;
	//外接矩形
	delta_phi_rad = radius / 6371.0;	//弧度差
	delta_phi_deg = delta_phi_rad * (180.0/M_PI);	//转换为角度


	center_lat_rad = centre.latitude * M_PI / 180.0;//圆心纬度转弧度
	cos_center_lat = cos(center_lat_rad);
	delta_lambda_rad = radius / (6371.0 * cos_center_lat);//弧度差
	delta_lambda_deg = delta_lambda_rad * (180.0/M_PI);//转换为角度

	//计算四个顶点经纬度
	result.vertexA[0].latitude = centre.latitude + delta_phi_deg;
	result.vertexA[0].longitude = centre.longitude + delta_lambda_deg;

	result.vertexA[1].latitude = centre.latitude - delta_phi_deg;
	result.vertexA[1].longitude = centre.longitude + delta_lambda_deg;

	result.vertexA[2].latitude = centre.latitude - delta_phi_deg;
	result.vertexA[2].longitude = centre.longitude - delta_lambda_deg;

	result.vertexA[3].latitude = centre.latitude + delta_phi_deg;
	result.vertexA[3].longitude = centre.longitude - delta_lambda_deg;

	//调整经纬度
	for(int i = 0 ; i < 4 ; i ++)
	{
		adjust_longitude(&result.vertexA[i].longitude);
		adjust_latitude(&result.vertexA[i].latitude);
	}

	return &result;
}

//调整经度到-180，180
void adjust_longitude(double *lon)
{
	*lon = fmod(*lon,360.0);//取模
	if(*lon >180.0)
		*lon -= 360.0;
	else if(*lon < -180)
		*lon += 360.0;
}
//调整纬度到-90,90
void adjust_latitude(double *lat)
{
	*lat = fmod(*lat,360.0);
	if(*lat >90.0)
		*lat = 180.0 - *lat;
	else if(*lat < -90)
		*lat = -180.0 - *lat;
}
/********************************无人机集群通讯**************************************************/
void UAV_jiqun(){

	//集群子任务个数初始化
	CTAS_jiQun_data_swarmRoute.subtasks_number = 0;

	message_size = 15000;
	//集群 战术战法推荐规划指令
	Receive_Message(DDSTables.BLK_JQ_CTAS_001.niConnectionId, 0, &transaction_id, dds_data_rece.dataA, &message_type_id, &message_size, &enRetCode);
	if(enRetCode == 0 && message_size == sizeof jiQun_CTAS_data_swarmRecommend){
		//将接收到的信息赋值给对应的对象
		memcpy(&jiQun_CTAS_data_swarmRecommend,dds_data_rece.dataA,sizeof jiQun_CTAS_data_swarmRecommend);


		//磁探生成
		InAreaLoopAirway inAreaLoopAirway;
		OutAirwayLibDas outAirwayLibDas;
		//有人机航路赋值
		inAreaLoopAirway.aircraft.longitude = jiQun_CTAS_data_swarmRecommend.solider_infos[0].lon_lat_info.longitude;
		inAreaLoopAirway.aircraft.latitude = jiQun_CTAS_data_swarmRecommend.solider_infos[0].lon_lat_info.latitude;
		//光栅覆盖区域赋值--矩形
		for(int i = 0;i < 4;i++){
			inAreaLoopAirway.area.vertexA[i].longitude =
					jiQun_CTAS_data_swarmRecommend.region_infos[1].reg_ploygen.points_lon_lat[i].longitude;
			inAreaLoopAirway.area.vertexA[i].latitude =
					jiQun_CTAS_data_swarmRecommend.region_infos[1].reg_ploygen.points_lon_lat[i].latitude;

		}
		inAreaLoopAirway.distBuff = 3;
		inAreaLoopAirway.widthScan = 1000;
		inAreaLoopAirway.distOffset = 0;

		//磁探搜索生成
		outAirwayLibDas = getAreaLoopAirway(&inAreaLoopAirway);

		//赋值生成的通用航路信息
		int manned_subtask_id = CTAS_jiQun_data_swarmRoute.subtasks_number;

		CTAS_jiQun_data_swarmRoute.subtasks_number++;
		CTAS_jiQun_data_swarmRoute.program_number = 0;

		CTAS_jiQun_data_swarmRoute.manned_computer_tasks[0].subtask_ID_number =1;
		CTAS_jiQun_data_swarmRoute.manned_computer_tasks[0].waypoints_number =
				outAirwayLibDas.sumAwp;
		for(int j = 0;j < 80;j++){
			CTAS_jiQun_data_swarmRoute.manned_computer_tasks[0].waypoint_informations[j].longitude = outAirwayLibDas.awpA[j].longitude;
			CTAS_jiQun_data_swarmRoute.manned_computer_tasks[0].waypoint_informations[j].latitude = outAirwayLibDas.awpA[j].latitude;
		}


		//发送通用航路---前40节点
		memcpy(send_array.dataA,&(CTAS_jiQun_data_swarmRoute),6);
		//发送前40个节点
		CTAS_jiQun_data_swarmRoute.manned_computer_tasks[0].waypoint_start_number = 0;
		memcpy(send_array.dataA + 6,&(CTAS_jiQun_data_swarmRoute.manned_computer_tasks[0]),6);
		memcpy(send_array.dataA + 16,&(CTAS_jiQun_data_swarmRoute.manned_computer_tasks[0].waypoint_informations[0]),40 * sizeof (CTAS_jiQun_data_swarmRoute.manned_computer_tasks[0].waypoint_informations[0]));
		//发送
		data_length = 16 + 40 * sizeof (CTAS_jiQun_data_swarmRoute.manned_computer_tasks[0].waypoint_informations[0]);
		Send_Message(DDSTables.BLK_CTAS_JQ_001.niConnectionId,0,&transaction_id, send_array.dataA, &message_type_id, data_length, &enRetCode);
		//        //发送通用航路---后40节点
	}

}


void attack_init(OutputResults output,unsigned int fangan)
{
//	if(blk_dtms_ctas_002.planning_id == 0)
//	{
//		information_on_the_results_of_taskings.program_number = 3;
//	}
//	else
//	{
//		information_on_the_results_of_taskings.program_number = blk_dtms_ctas_002.planning_id;
//	}
	fangan_index++;
	information_on_the_results_of_taskings.program_number = fangan_index;
	information_on_the_results_of_taskings.tasking_release = 5;//任务分配方案发布---首次生成
	information_on_the_results_of_taskings.manual_modification = 2;//是否人工修改---未修改
	information_on_the_results_of_taskings.emphasize_planning = 2;//是否重规划---否
	information_on_the_results_of_taskings.modification_method = 0;//修改方式---无
	information_on_the_results_of_taskings.program_attributes = 3+fangan;//方案属性---待定
	//任务平台个数---3.2 无人机个数n + 有人机个数1
	information_on_the_results_of_taskings.number_of_mission_platforms = blk_dtms_ctas_002.solider_num;
	/*
	 * 有人机任务信息分配结果
	 * */
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_model = 1;//平台型号--有人机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_serial_number = 0;//平台序号---本机
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].platform_number = MANNED_ID;//平台编号---有人机 MANNED_ID
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks = 1;

	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].subtask_ID_number = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].modify = 0;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].sequence_type = 15;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].type_of_mission_point_area = 2;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].completion_time_valid_Bits = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].task_height_effective_position = 1;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].task_completion_time = 20;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].mission_height = 1000;
	information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[0].availability_of_routes = 1;

	/*
	 * 无人机任务信息分配结果
	 * */
	for(int i = 1;i <= blk_dtms_ctas_002.solider_num - 1;i++){
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_model = 2;//平台型号--无人机
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_serial_number = i;//平台序号---无人机i
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].platform_number = drone_state_informations.drone_specific_informations[i - 1].platform_num;//平台编号---无
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks = 1;

		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].subtask_ID_number = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].modify = 0;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].sequence_type = 15;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].type_of_mission_point_area = 2;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].completion_time_valid_Bits = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].task_height_effective_position = 1;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].task_completion_time = 20;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].mission_height = 1000;
		information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[0].availability_of_routes = 1;
	}
	//有人机航线
//	memset(&CTAS_DTMS_data_mannedRoute,0,sizeof(manned_aircraft_route_information));
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.program_number = information_on_the_results_of_taskings.program_number;
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.subtasks_number = 1;
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].subtask_ID_number = 1;
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoints_number = 2;
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoint_start_number = 0;
//	//有人机当前位置
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoint_informations[0].latitude =
//			blk_dtms_ctas_002.solider_infos[0].lon_lat_info.latitude;
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoint_informations[0].longitude =
//			blk_dtms_ctas_002.solider_infos[0].lon_lat_info.longitude;
//	//有效性
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoint_informations[0].latitude_validity = 1;
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoint_informations[0].validity_of_longitude = 1;
//	//有人机攻击位置
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoint_informations[1].latitude =
//			output.manned_attack_path_lat;
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoint_informations[1].longitude =
//			output.manned_attack_path_long;
//	//有效性
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoint_informations[1].latitude_validity = 1;
//	CTAS_DTMS_data_mannedRoute.common_carrier_routes.manned_computer_tasks[0].waypoint_informations[1].validity_of_longitude = 1;

	//无人机航线
	memset(&CTAS_DTMS_data_UAVRoute,0,sizeof(drone_route_confirmation));
	CTAS_DTMS_data_UAVRoute.program_number = information_on_the_results_of_taskings.program_number;
	CTAS_DTMS_data_UAVRoute.plan_type = 4;
	for(int i = 0 ; i < integrated_postures.drone_num; i ++)
	{
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].drone_num = blk_dtms_ctas_002.solider_infos[i + 1].solider_id;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].subtask_index = 0;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].subtasks_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].drone_serial_number = i+1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].total_packet = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].subtask_ID_number = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].mission_type = 15;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].point_area_type = 2;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].waypoints_number = 1;
		// 航点为一个盘旋点
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].hld_idx = 1;
		//航点经纬度
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].longitude =
				output.drone_path_long[i][0];
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].latitude =
				output.drone_path_lat[i][0];
		//有效性
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].validity_of_longitude = 1;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].latitude_validity = 1;

		//航路点待机时间/圈数/循环次数
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber
		= PX_CIRCLE;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].standby_time_lapsNumber_cycleNumber_valid_bit = 1;
		//航路点待机半径
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].standby_radius
		= PX_RAD;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].standby_radius_valid_bit = 1;
		//高度速度,类型
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].height = blk_dtms_ctas_010[i].GJ;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].speed = 35;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].causality = 3;
		CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[0].standby_type = 2;//圈数

//		while(drone_start_index > 0)
//		{
//			drone_start_index--;
//			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[drone_start_index].hld_idx = drone_start_index + 1;
//			//航点经纬度
//		    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[drone_start_index].longitude =
//							output.drone_path_long[i][drone_start_index];
//			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[drone_start_index].latitude =
//					  output.drone_path_lat[i][drone_start_index];
//			//有效性
//		    CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[drone_start_index].validity_of_longitude = 1;
//			CTAS_DTMS_data_UAVRoute.individual_drone_routing_programs[i].planning_informations[0].planning_information_waypoint_informations[drone_start_index].latitude_validity = 1;
//		}
	}

	//计算航程和时间
	if(integrated_postures.drone_num == 1)
	{
		//计算无人机航线长度
		double range = 0;
		float speed = 0;
		range = calculate_distances(integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.latitude,
				integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.longitude,
				output.drone_path_lat[0][0],
				output.drone_path_long[0][0]);

		information_on_the_results_of_taskings.range = range;
		speed = integrated_postures.integrated_posture_drone_informations[0].drone_speed / 3.6;
		if(speed < 10)
		{
			speed = 35.0;
		}
		information_on_the_results_of_taskings.total_program_time = (information_on_the_results_of_taskings.range * 1000) / speed;
	}
	else if(integrated_postures.drone_num == 2)
	{
		//找出最长的航程
		double range_1 = 0;
		double range_2 = 0;
		float speed = 0;
		range_1 = calculate_distances(integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.latitude,
				integrated_postures.integrated_posture_drone_informations[0].drone_longitude_and_latitude.longitude,
						output.drone_path_lat[0][0],
						output.drone_path_long[0][0]);
		range_2 = calculate_distances(integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.latitude,
				integrated_postures.integrated_posture_drone_informations[1].drone_longitude_and_latitude.longitude,
					output.drone_path_lat[1][0],
					output.drone_path_long[1][0]);
		if(range_1 > range_2)
		{
			information_on_the_results_of_taskings.range = range_1;
			speed = integrated_postures.integrated_posture_drone_informations[0].drone_speed / 3.6;
			if(speed < 10)
			{
				speed = 35.0;
			}
		}
		else
		{
			information_on_the_results_of_taskings.range = range_2;
			speed = integrated_postures.integrated_posture_drone_informations[1].drone_speed / 3.6;
			if(speed < 10)
			{
				speed = 35.0;
			}
		}
		information_on_the_results_of_taskings.total_program_time = (information_on_the_results_of_taskings.range * 1000) / speed;
		information_on_the_results_of_taskings.range = range_1 + range_2;
	}

	//安全区方案编号
	blk_ctas_dtms_008.program_number = information_on_the_results_of_taskings.program_number;
}
void attack_commend(OutputResults* output,int plan){

	InputParams input;
	Geo temMan;
	Geo temTgt;
	double temDis;

	// 初始化输入参数
	input.target_long = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.longitude;
	input.target_lat = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_lon_lat.latitude;
	input.target_alt = 0;
	input.target_speed = blk_dtms_ctas_002.region_infos[0].point_infos[0].target_speed;
	input.target_heading = blk_dtms_ctas_002.region_infos[0].point_infos[0].target_hangxiang;
	input.target_type = blk_dtms_ctas_002.region_infos[0].point_infos[0].target_type;
	input.target_info_source = blk_dtms_ctas_002.region_infos[0].point_infos[0].point_source;
	input.target_long_valid = VALID;
	input.target_lat_valid = VALID;
	input.target_speed_valid = VALID;
	input.target_heading_valid = VALID;
	input.manned_long = blk_dtms_ctas_002.solider_infos[0].lon_lat_info.longitude;
	input.manned_lat = blk_dtms_ctas_002.solider_infos[0].lon_lat_info.latitude;
	input.manned_alt = 400;//DTMS_CTAS_singleRecommend.solider_infos[0].height
	input.manned_speed =  blk_dtms_ctas_002.solider_infos[0].speed;
	input.num_drones = blk_dtms_ctas_002.solider_num - 1;
	int uav_index = 0;// 有效无人机序号（非紧密排列）
	for (int i = 0; i < MAX_DRONES; i++) {
		// 跳过无效无人机
		if(blk_dtms_ctas_002.solider_infos[i+1].solider_id == 0)
		{
			continue;
		}
		input.drone_long[uav_index] = blk_dtms_ctas_002.solider_infos[i+1].lon_lat_info.longitude;
		input.drone_lat[uav_index] = blk_dtms_ctas_002.solider_infos[i+1].lon_lat_info.latitude;
		input.drone_alt[uav_index] = blk_dtms_ctas_002.solider_infos[i+1].height;
		uav_index++;
	}
	input.num_torpedoes = 8;
	input.num_depth_charges = 8;
	input.num_anti_ship_missiles = 8;
	//重置解算结果为无效
	output->return_info = 0; // 0-无效

	//协同攻击总体方案推荐
	collaborative_attack(&input, output);

	//        attackPosInputParams attack_input;
	//        attackPosOutputResults attack_output;
	//        //有人机占位解算;此接口实现未开发完成；直接调用615客户提供的接口，不再开发
	//        manned_attack_position_calculation(&attack_input, &attack_output);

	SPlacePosCalc sPlacePosCalc;
	sPlacePosCalc.iAcGeo.Lon = input.manned_long;//载机经度
	sPlacePosCalc.iAcGeo.Lat = input.manned_lat;//载机纬度
	sPlacePosCalc.iAcSpdGnd = input.manned_speed;//载机地速
	sPlacePosCalc.iTgtSerNo = input.target_type;//目标批号
	sPlacePosCalc.iTgtGeo.Lon = input.target_long;//目标经度
	sPlacePosCalc.iTgtGeo.Lat = input.target_lat;//目标纬度
	sPlacePosCalc.iTgtYawT = input.target_heading;//目标航向（真）
	sPlacePosCalc.iTgtSpd = input.target_speed;//目标速度 input.target_speed
	sPlacePosCalc.iDisAtk = 0.5;//武器攻击范围
	sPlacePosCalc.iDisAtkL = 5;//武器攻击范围左边界
	sPlacePosCalc.iDisAtkR = 5;//武器攻击范围右边界
	//有人机占位解算
//	PlacePos_Calc(&sPlacePosCalc);
//	if(sPlacePosCalc.oPlaceVld == 1)
//	{
//		//解算成功使用结算
//		output->manned_attack_path_lat = sPlacePosCalc.oPlaceGeo.Lat;
//		output->manned_attack_path_long = sPlacePosCalc.oPlaceGeo.Lon;
//	}
//	else
//	{
//		//解算失败使用有人机当前位置
//
//		output->manned_attack_path_lat = input.manned_lat;
//		output->manned_attack_path_long = input.manned_long;
//	}

	/*计算目标相对于当前飞机的方位和距离L*/

	memcpy(&temMan, &sPlacePosCalc.iAcGeo, sizeof(Geo));
	memcpy(&temTgt, &sPlacePosCalc.iTgtGeo, sizeof(Geo));
	temDis = getDistanceGeo(&temMan, &temTgt);

	//如果距离小于0.5km,那么就将目标位置作为占位点
	if(temDis <= 500)
	{
		output->manned_attack_path_long = input.target_long;
		output->manned_attack_path_lat = input.target_lat;
	}
	else //如果目标距离大于0.5km,则将本机向目标方位，延伸（L-0.5）作为占位点。
	{
		// 转平面坐标（）
		Grid temManGrid;
		temManGrid = getGridByGeo(&temTgt, &temMan);
		// 计算相对方位的坐标
		Grid resGrid;
		resGrid.east=temManGrid.east*(temDis-500)/temDis;
		resGrid.north=temManGrid.north*(temDis-500)/temDis;
		Geo resGeo;
		resGeo = getGeoByGrid(&resGrid,&temMan);
		output->manned_attack_path_long = resGeo.longitude;
		output->manned_attack_path_lat = resGeo.latitude;
	}


	threatAreaInputParams threat_input;
	threatAreaOutputResults threat_output;

	threat_input.calc_position_long = output->manned_attack_path_long; // 预估占位点经度
	threat_input.calc_position_lat = output->manned_attack_path_lat; // 预估占位点纬度

	threat_input.manned_long = input.manned_long; // 载机经度
	threat_input.manned_lat  = input.manned_lat; // 载机纬度
	threat_input.manned_alt = input.manned_alt; // 载机高度

	threat_input.weapon_range = 0.5 * 1000; // 武器打击距离，单位：米
	//计算有人机到目标点的角度
	double m_t_angle;
	m_t_angle = calculate_angle(input.manned_lat,input.manned_long,input.target_lat,input.target_long);
	threat_input.weapon_angle = m_t_angle * M_PI / 180.0; // 武器打击角度，单位：弧度;代码里假设  假设预计飞行航向角等于武器打击角度 blk_dtms_ctas_002.solider_infos[0].hangxiang
	threat_input.max_bias_coefficient = Distance_Bias_Max; // 攻击距离偏移最大阈值系数

	// 计算威胁区
	weapon_threat_area_calculation(&threat_input, &threat_output);

	avoidanceRegionResults avoidance_region_output;


	int uav_turn = 0;
	int safe_dist = 0;
	//效能优先
	if(plan == 1)
	{
		uav_turn = 1500;
		safe_dist = 3000;
	}
	//安全优先
	else if(plan == 2)
	{
		uav_turn = 3000;
		safe_dist = 9000;
	}
	//悬停投放
	else if(plan == 3)
	{
		uav_turn = 1500;
		safe_dist = 0;
	}
	// 计算规避区域
	attack_safety_and_avoidance_calculation(&threat_output, &avoidance_region_output,uav_turn,safe_dist);

	attackModeAllocInputParams  attack_mode_input;
	attackModeAllocOutputResults attack_mode_output;

	//以载机当前位置为基准点
	attack_mode_input.manned_pos_x = 0; // 载机当前位置x
	attack_mode_input.manned_pos_y = 0; // 载机当前位置y
	attack_mode_input.manned_speed = input.manned_speed/3.6; // 载机航速，单位：m/s
	//        attack_mode_input.manned_attack_pos_x = attack_output.position_x; // 载机攻击位置，单位米/*
	//        attack_mode_input.manned_attack_pos_y = attack_output.position_y; // 载机攻击位置，单位米*/
	//将经纬度转换为北东天坐标系
	double position_x,position_y,position_z;
	llh_to_ned(output->manned_attack_path_lat, output->manned_attack_path_long, 0.0, input.manned_lat, input.manned_long,\
			0.0, &position_x, &position_y, &position_z);
	attack_mode_input.manned_attack_pos_x = position_x; // 载机攻击位置，单位米
	attack_mode_input.manned_attack_pos_y = position_y; // 载机攻击位置，单位米


	double target_x, target_y, target_z;

	//将目标经纬度转换为北东天坐标系,以载机当前位置为基准
	llh_to_ned(input.target_lat, input.target_long, input.target_alt, input.manned_lat, \
			input.manned_long, input.manned_alt, &target_x, &target_y, &target_z);

	attack_mode_input.target_pos_x = target_x; // 目标位置，单位米
	attack_mode_input.target_pos_y = target_y; // 目标位置，单位米

	attack_mode_input.num_drones = input.num_drones; //无人机数量

	//只有一架机沿用原有逻辑
	if(attack_mode_input.num_drones == 1)
	{
		for(unsigned int i = 0; i < attack_mode_input.num_drones; i++)
		{
			//将无人机经纬度转换为北东天坐标系,以载机当前位置为基准
			llh_to_ned(input.drone_lat[i], input.drone_long[i], input.drone_alt[i], input.manned_lat, \
					input.manned_long, input.manned_alt, &target_x, &target_y, &target_z);

			attack_mode_input.drone_pos_x[i] = target_x; //无人机位置x，单位米
			attack_mode_input.drone_pos_y[i] = target_y; //无人机位置Y，单位米
			attack_mode_input.drone_speed[i] = target_z;//无人机速度
		}

		//规避区域1/2的x/Y位置
		attack_mode_input.avoid_regon_center_x[0] = avoidance_region_output.avoidance_gegion_center_x[0];
		attack_mode_input.avoid_regon_center_y[0] = avoidance_region_output.avoidance_gegion_center_y[0];
		attack_mode_input.avoid_regon_center_y[1] = avoidance_region_output.avoidance_gegion_center_y[1];
		attack_mode_input.avoid_regon_center_x[1] = avoidance_region_output.avoidance_gegion_center_x[1];

		//协同攻击方式分配计算
		collaborative_attack_mode_allocation(&attack_mode_input, &attack_mode_output);

		//规避区域索引
		int index = (attack_mode_output.avoid_regin[0] - 1);

		//将规避区域中心点作为无人机的盘旋航点
		for(int i = 0 ; i < input.num_drones ; i++)
		{
			output->drone_path_points[i] = 1;

			double height,lat_tmp,lon_tmp;
			//将规避区域中心点转换为经纬度，中心点作为无人机的航点
			ned_to_llh(avoidance_region_output.avoidance_gegion_center_x[index], avoidance_region_output.avoidance_gegion_center_y[index], 0.0,
					input.manned_lat, input.manned_long, 0.0,
					&lat_tmp, &lon_tmp, &height);
			output->drone_path_lat[i][0] = lat_tmp;
			output->drone_path_long[i][0] = lon_tmp;
		}

		int index_area = (attack_mode_output.avoid_regin[0] - 1) * 4;
		//安全区、威胁区赋值
		for(int i = 0 ; i < input.num_drones ; i ++)
		{
			//无人机数量超过2架机，避免溢出
			if(input.num_drones > 2)
			{
				i = 0;
			}
			blk_ctas_dtms_008.taskarea_num = input.num_drones;
			//安全区
			blk_ctas_dtms_008.task_area[i].uav_sn = i + 1;
			blk_ctas_dtms_008.task_area[i].uav_id = blk_dtms_ctas_002.solider_infos[i+1].solider_id;
			blk_ctas_dtms_008.task_area[i].point_num = 4;
			for(int j = 0; j < 4 ; j ++)
			{
				double height,lat_tmp,lon_tmp;
				//将规避区域顶点转换为经纬度
				ned_to_llh(avoidance_region_output.avoidance_region_x[index_area + j],
						avoidance_region_output.avoidance_region_y[index_area + j], 0.0,
						input.manned_lat, input.manned_long, 0.0,
						&lat_tmp,&lon_tmp,&height);

				blk_ctas_dtms_008.task_area[i].point_coordinates[j].latitude = lat_tmp;
				blk_ctas_dtms_008.task_area[i].point_coordinates[j].longitude = lon_tmp;
			}
//        //威胁区
//        if(i == 0)//有人机攻击路线
//        {
//             region[i + 2].point_num = 4;
//             for(int j = 0; j < 4 ; j ++)
//             {
//                 region[i + 2].points_lon_lat[j].latitude = threat_output.threat_region_x[j];
//                 region[i + 2].points_lon_lat[j].longitude = threat_output.threat_region_y[j];
//             }
//        }
//        else if(i == 1)//武器投放威胁区
//        {
//            region[i + 2].point_num = 4;
//            for(int j = 0; j < 4 ; j ++)
//            {
//                double height;
//                //将武器投放威胁区转换为经纬度
//                ned_to_llh(threat_output.threat_region_x[i*4 + j],
//                        threat_output.threat_region_y[i*4 + j], 0.0,
//                        input.manned_lat, input.manned_long, 0.0,
//                        &region[i + 2].points_lon_lat[j].latitude,
//                        &region[i + 2].points_lon_lat[j].longitude,
//                        &height);
//            }
//        }
		}
	}
	//双机情况
	else if(attack_mode_input.num_drones == 2)
	{
		//算出无人机最佳规避区
		for(unsigned int i = 0; i < attack_mode_input.num_drones; i++)
		{
			//将无人机经纬度转换为北东天坐标系,以载机当前位置为基准
			llh_to_ned(input.drone_lat[i], input.drone_long[i], input.drone_alt[i], input.manned_lat, \
					input.manned_long, input.manned_alt, &target_x, &target_y, &target_z);

			attack_mode_input.drone_pos_x[i] = target_x; //无人机位置x，单位米
			attack_mode_input.drone_pos_y[i] = target_y; //无人机位置Y，单位米
			attack_mode_input.drone_speed[i] = target_z;//无人机速度
		}
		//规避区域1/2的x/Y位置
		attack_mode_input.avoid_regon_center_x[0] = avoidance_region_output.avoidance_gegion_center_x[0];
		attack_mode_input.avoid_regon_center_y[0] = avoidance_region_output.avoidance_gegion_center_y[0];
		attack_mode_input.avoid_regon_center_y[1] = avoidance_region_output.avoidance_gegion_center_y[1];
		attack_mode_input.avoid_regon_center_x[1] = avoidance_region_output.avoidance_gegion_center_x[1];

		//协同攻击方式分配计算
		collaborative_attack_mode_allocation(&attack_mode_input, &attack_mode_output);

		//规避区域是否在同侧

		//非同侧
		if(attack_mode_output.avoid_regin[0] != attack_mode_output.avoid_regin[1])
		{
			//计算无人机航线
			for(int i = 0 ; i < input.num_drones ; i++)
			{
				//规避区域索引
				int index = (attack_mode_output.avoid_regin[i] - 1);
				output->drone_path_points[i] = 1;
				double height,lat_tmp,lon_tmp;
				//将规避区域中心点作为无人机的盘旋航点
				ned_to_llh(avoidance_region_output.avoidance_gegion_center_x[index], avoidance_region_output.avoidance_gegion_center_y[index], 0.0,
						input.manned_lat, input.manned_long, 0.0,
						&lat_tmp, &lon_tmp, &height);
				output->drone_path_lat[i][0] = lat_tmp;
				output->drone_path_long[i][0] = lon_tmp;
			}
			//安全区赋值
			for(int i = 0 ; i < input.num_drones ; i ++)
			{
				//规避区域索引
				int index = (attack_mode_output.avoid_regin[i] - 1) * 4;
				//无人机数量超过2架机，避免溢出
				if(input.num_drones > 2)
				{
					i = 0;
				}
				blk_ctas_dtms_008.taskarea_num = input.num_drones;
				//安全区
				blk_ctas_dtms_008.task_area[i].uav_sn = i + 1;
				blk_ctas_dtms_008.task_area[i].uav_id = blk_dtms_ctas_002.solider_infos[i+1].solider_id;
				blk_ctas_dtms_008.task_area[i].point_num = 4;
				for(int j = 0; j < 4 ; j ++)
				{
					double height,lat_tmp,lon_tmp;
					//将规避区域顶点转换为经纬度
					ned_to_llh(avoidance_region_output.avoidance_region_x[index + j],
							avoidance_region_output.avoidance_region_y[index + j], 0.0,
							input.manned_lat, input.manned_long, 0.0,
							&lat_tmp,&lon_tmp,&height);

					blk_ctas_dtms_008.task_area[i].point_coordinates[j].latitude = lat_tmp;
					blk_ctas_dtms_008.task_area[i].point_coordinates[j].longitude = lon_tmp;
				}
			}
		}
		//在同侧
		else if(attack_mode_output.avoid_regin[0] == attack_mode_output.avoid_regin[1])
		{
			//方案1/2处理
			if(plan == 1 || plan == 2)
			{
				//生成新的规避区
				int index = 0;
				double bearing = 0;
				SGeo origin;
				SGeo new_center;
				SGeo new_area[4];
				double height_old,lati_old,longt_old;
				//判断规避区在左边或右边
				if(attack_mode_output.avoid_regin[0] == 1)
				{
					bearing = m_t_angle + 270.0;
				}
				else if(attack_mode_output.avoid_regin[0] == 2)
				{
					bearing = m_t_angle + 90.0;
				}
				//调整角度范围
				if(bearing > 360)
					bearing -= 360.0;

				//规避区域索引
				index = (attack_mode_output.avoid_regin[0] - 1);
				//计算规避区中心点经纬度
				ned_to_llh(avoidance_region_output.avoidance_gegion_center_x[index], avoidance_region_output.avoidance_gegion_center_y[index], 0.0,
						input.manned_lat, input.manned_long, 0.0,
						&lati_old, &longt_old, &height_old);
				//起点经纬度(规避区中心点)
				origin.Lat = lati_old;
				origin.Lon = longt_old;
				//边长计算
				double side = 0;
				if(plan == 1)
				{
					side = 3.0;
				}
				else if(plan == 2)
				{
					side = 6.0;
				}
				//计算出新的规避区中心点经纬度
				new_center = calculate_new_coordinate(origin,side+0.5,bearing);
				//计算新的规避区的四个顶点经纬度
				calculate_rectangle_vertices_new(new_center,m_t_angle,side,&new_area[0]);
				//把新的规避区替换另一侧规避区
				for(int j = 0; j < 4; j++)
				{
					//新的规避区转换为北东天坐标系,以载机当前位置为基准
					llh_to_ned(new_area[j].Lat, new_area[j].Lon, height_old, input.manned_lat, \
							input.manned_long, input.manned_alt, &target_x, &target_y, &target_z);
					int index_a = 0;
					index_a = (((attack_mode_output.avoid_regin[0] == 1) ? 2 : 1) - 1) *4 + j;
					avoidance_region_output.avoidance_region_x[index_a] = target_x;
					avoidance_region_output.avoidance_region_y[index_a] = target_y;
				}
				//替换规避区中心点
				int index_center = 0;
				index_center = ((attack_mode_output.avoid_regin[0] == 1) ? 2 : 1) - 1;
				//新的规避区转换为北东天坐标系,以载机当前位置为基准
				llh_to_ned(new_center.Lat, new_center.Lon, height_old, input.manned_lat, \
						input.manned_long, input.manned_alt, &target_x, &target_y, &target_z);
				avoidance_region_output.avoidance_gegion_center_x[index_center] = target_x;
				avoidance_region_output.avoidance_gegion_center_y[index_center] = target_y;
			}

			//重新分配规避区
			//无人机2选择另一个规避区
			attack_mode_output.avoid_regin[1] = (attack_mode_output.avoid_regin[1] == 1) ? 2 : 1;
			//航线有无交叉，有交叉互换规避区
			int index = 0;
			int rtn = 0;
			FlightRoute routeA;
			FlightRoute routeB;
			double height,lati[2],longt[2];
			for(int i = 0 ; i < 2 ; i ++)
			{
				//规避区域索引
				index = (attack_mode_output.avoid_regin[i] - 1);
				//计算规避区中心点经纬度
				ned_to_llh(avoidance_region_output.avoidance_gegion_center_x[index], avoidance_region_output.avoidance_gegion_center_y[index], 0.0,
						input.manned_lat, input.manned_long, 0.0,
						&lati[i], &longt[i], &height);
			}
			//无人机1航线段
			routeA.start.longitude = input.drone_long[0];
			routeA.start.latitude = input.drone_lat[0];
			routeA.end.longitude = longt[0];
			routeA.end.latitude = lati[0];
			//无人机2航线段
			routeB.start.longitude = input.drone_long[1];
			routeB.start.latitude = input.drone_lat[1];
			routeB.end.longitude = longt[1];
			routeB.end.latitude = lati[1];
			rtn = detectConflict(&routeA,&routeB);
			if(rtn == 1)
			{
				//有冲突互换规避区
				attack_mode_output.avoid_regin[0] = (attack_mode_output.avoid_regin[0] == 1) ? 2 : 1;
				attack_mode_output.avoid_regin[1] = (attack_mode_output.avoid_regin[1] == 1) ? 2 : 1;
			}

			//计算无人机航线
			for(int i = 0 ; i < input.num_drones ; i++)
			{
				//规避区域索引
				int index = (attack_mode_output.avoid_regin[i] - 1);
				output->drone_path_points[i] = 1;
				double height,lat_tmp,lon_tmp;
				//将规避区域中心点作为无人机的盘旋航点
				ned_to_llh(avoidance_region_output.avoidance_gegion_center_x[index], avoidance_region_output.avoidance_gegion_center_y[index], 0.0,
						input.manned_lat, input.manned_long, 0.0,
						&lat_tmp, &lon_tmp, &height);
				output->drone_path_lat[i][0] = lat_tmp;
				output->drone_path_long[i][0] = lon_tmp;
			}
			//安全区赋值
			for(int i = 0 ; i < input.num_drones ; i ++)
			{
				//规避区域索引
				int index = (attack_mode_output.avoid_regin[i] - 1) * 4;
				//无人机数量超过2架机，避免溢出
				if(input.num_drones > 2)
				{
					i = 0;
				}
				blk_ctas_dtms_008.taskarea_num = input.num_drones;
				//安全区
				blk_ctas_dtms_008.task_area[i].uav_sn = i + 1;
				blk_ctas_dtms_008.task_area[i].uav_id = blk_dtms_ctas_002.solider_infos[i+1].solider_id;
				blk_ctas_dtms_008.task_area[i].point_num = 4;
				for(int j = 0; j < 4 ; j ++)
				{
					double height,lat_tmp,lon_tmp;
					//将规避区域顶点转换为经纬度
					ned_to_llh(avoidance_region_output.avoidance_region_x[index + j],
							avoidance_region_output.avoidance_region_y[index + j], 0.0,
							input.manned_lat, input.manned_long, 0.0,
							&lat_tmp,&lon_tmp,&height);

					blk_ctas_dtms_008.task_area[i].point_coordinates[j].latitude = lat_tmp;
					blk_ctas_dtms_008.task_area[i].point_coordinates[j].longitude = lon_tmp;
				}
			}

		}
	}


	// 如果到达这里，表示输入参数有效
	output->return_info = 1; // 解算成功
	printf("end calculate!");

}

/**
 * 璁＄畻涓ゆ潯绾挎鐨勪氦鐐�
 */
int lineIntersection(Grid p1, Grid p2, Grid p3, Grid p4, Grid* intersection)
{
    double dx1;
    double dy1;
    double dx2;
    double dy2;

    double denominator;

    double u1;
    double u2;

    dx1 = p2.east - p1.east;
    dy1 = p2.north - p1.north;
    dx2 = p4.east - p3.east;
    dy2 = p4.north - p3.north;

    denominator = dx1 * dy2 - dy1 * dx2;

    if (fabs(denominator) < 1e-6) {
        return 0;  // 绾挎骞宠鎴栭噸鍚�
    }

    u1 = ((p3.east - p1.east) * dy2 - (p3.north - p1.north) * dx2) / denominator;

    intersection->east = p1.east + u1 * dx1;
    intersection->north = p1.north + u1 * dy1;

    // 妫�煡浜ょ偣鏄惁鍦ㄧ嚎娈典笂
    if (u1 < 0 || u1 > 1) return 0;

    u2 = ((p3.east - p1.east) * dy1 - (p3.north - p1.north) * dx1) / denominator;
    if (u2 < 0 || u2 > 1) return 0;

    return 1;
}


/**
 * 妫�祴涓ゆ潯鑸嚎鏄惁鏈夊啿绐�
 * @param routeA 鏈変汉鏈鸿埅绾�
 * @param routeB 鏃犱汉鏈鸿埅绾�
 * @param result 鍐茬獊缁撴灉
 * @return 0-鎴愬姛锛屽叾浠�澶辫触
 */
int detectConflict(FlightRoute* routeA, FlightRoute* routeB)
{

	// 閫夋嫨鑸嚎A鐨勮捣鐐逛綔涓哄弬鑰冪偣
    Geo* origin;

    // 灏嗚埅绾胯浆鎹负鍖椾笢鍧愭爣绯�
    Grid routeAStart;
    Grid routeAEnd;
    Grid routeBStart;
    Grid routeBEnd;
    Grid intersection;

    // 閫夋嫨鑸嚎A鐨勮捣鐐逛綔涓哄弬鑰冪偣
    origin = &routeA->start;

    // 灏嗚埅绾胯浆鎹负鍖椾笢鍧愭爣绯�
    routeAStart = getGridByGeo(&routeA->start, origin);
    routeAEnd = getGridByGeo(&routeA->end, origin);
    routeBStart = getGridByGeo(&routeB->start, origin);
    routeBEnd = getGridByGeo(&routeB->end, origin);

    // 妫�煡绾挎鏄惁鐩镐氦

    if (lineIntersection(routeAStart, routeAEnd, routeBStart, routeBEnd, &intersection)) {

        return 1;
    }

    return 0;
}

// 判读航线是否穿过正方体
char avoidLineCrashIsCrossRect(FlightRoute* route1, AreaRectVertex* resAreaRectVertex)
{
	BOOL needDetour = false;
	FlightRoute route2;
	int temResInt = 0;

	for(int i = 0; i< 4;i++)
	{
		route2.start = resAreaRectVertex->vertexA[i];
		route2.end = resAreaRectVertex->vertexA[(i+1)%4];
		temResInt = detectConflict(route1, &route2);
		if(temResInt == 1)
		{
			needDetour = true;
		}
	}

	return needDetour;
}

//发送安全区、威胁区
void send_blk_ctas_dtms_008()
{
	data_length = sizeof(BLK_CTAS_DTMS_008);
	Send_Message(DDSTables.BLK_CTAS_DTMS_008.niConnectionId, 0, &transaction_id, &blk_ctas_dtms_008, &message_type_id, data_length, &enRetCode);
	if(enRetCode != 0)
	{
		printf("send blk_ctas_dtms_008 success\n");
	}
}

static int calc_strategy9_total_time(const int manned_stage_time[8], const int uav_stage_max_time[8], int old_total_time)
{
	if(manned_stage_time == NULL || uav_stage_max_time == NULL)
	{
		return old_total_time;
	}
	// 战法9：阶段1按有人机时间，阶段2按无人机时间
	if(manned_stage_time[0] > 0 && uav_stage_max_time[1] > 0)
	{
		return manned_stage_time[0] + uav_stage_max_time[1];
	}
	return old_total_time;
}

void uav_time_range_calc()
{
	//方案预估时长
	int longest_time = 0;
	int total_time[5] = {0,0,0,0,0};
	int manned_stage_time[8] = {0,0,0,0,0,0,0,0};
	int uav_stage_max_time[8] = {0,0,0,0,0,0,0,0};
	int max_time = 0;
	//20260122有人机方案时间，航程计算
	{

		int i = 0;//有人机
		int temp_time = 0;

		//有人机固定参数
		float speed_kmh = 180.0;
		float speeds_val = speed_kmh/3.6; //50m/s
		//遍历有人机任务
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
				{
					int temp_time_before_stage = temp_time;
					int task_type = information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type;
					//1.寻找任务区域并计算面积，中心点距离
					double dist = 0;


					//任务区面积
					float area = 0;
					for(int a = 0 ; a < 8 ; a++)
					{
						if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number ==
								area_sky_informations.area_informations[a].area_code && area_sky_informations.area_informations[a].area_code != 0)
						{
							//获取有人机当前经纬度
							double plane_lat = integrated_postures.latitude;
							double plane_lon = integrated_postures.longitude;
							//圆形
							if(area_sky_informations.area_informations[a].area_shape == 1)
							{
								float rad = area_sky_informations.area_informations[a].cycles.radius;
								area = M_PI * rad * rad * 1000 * 1000;
								dist = calculate_distances(plane_lat
										,plane_lon
										,area_sky_informations.area_informations[a].cycles.latitude
										,area_sky_informations.area_informations[a].cycles.longitude);

							}
							//矩形
							else if(area_sky_informations.area_informations[a].area_shape == 2)
							{
								float Long = 0;
								float Wide = 0;
								Long = calculate_distances(area_sky_informations.area_informations[a].polygonals.point_coordinates[0].latitude,
										area_sky_informations.area_informations[a].polygonals.point_coordinates[0].longitude,
										area_sky_informations.area_informations[a].polygonals.point_coordinates[1].latitude,
										area_sky_informations.area_informations[a].polygonals.point_coordinates[1].longitude);
								Wide = calculate_distances(area_sky_informations.area_informations[a].polygonals.point_coordinates[1].latitude,
										area_sky_informations.area_informations[a].polygonals.point_coordinates[1].longitude,
										area_sky_informations.area_informations[a].polygonals.point_coordinates[2].latitude,
										area_sky_informations.area_informations[a].polygonals.point_coordinates[2].longitude);
								area = Long * Wide * 1000 * 1000;


								dist = calculate_distances(plane_lat
										,plane_lon
										,area_sky_informations.area_informations[a].polygonals.point_coordinates[0].latitude
										,area_sky_informations.area_informations[a].polygonals.point_coordinates[0].longitude);

							}
							break;
						}
					}
					double task_op_time = 0;//任务耗时
					double flight_time = 0; // 航程耗时

					//场景1：有人机吊声预估
					if(task_type ==2){
						//航程时间
						flight_time = (dist*1000)/speeds_val;
						//作业时间
						double r_work = 10000.0;
						int points =(int)ceil(area/(M_PI*r_work*r_work));
						double one_drop_time = (r_work*2.0)/speeds_val;//直径/速度
						task_op_time = points*1800;
					}
					//场景2:有人机浮标布阵
					if(task_type ==3){
						//航程时间
						flight_time = (dist*1000)/speeds_val;
						//作业时间
						double r_work = 2500.0;
						int points =(int)ceil(area/(M_PI*r_work*r_work));
						double one_drop_time = (r_work*2.0)/speeds_val;//直径/速度
						task_op_time = points*one_drop_time;
					}
					//场景3:有人机浮标监听
					if (task_type == 1 ){
						//假设监听要先飞回去
						if(j==0) flight_time = (dist*1000)/speeds_val;
						task_op_time=1200;//固定值
					}
					//累加时间
					double segment_time = flight_time+task_op_time;
					temp_time += segment_time;
					if(j < 8)
					{
						manned_stage_time[j] = temp_time - temp_time_before_stage;
					}

					//结果回填
					if(segment_time>0){
						//单位秒转分钟
						information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time=task_op_time/60;
						//填充高度
						information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = integrated_postures.ABSOLUTE_PRESSURE_ALT_A;
					}
				}
		total_time[0]= temp_time;
		information_on_the_results_of_taskings.range+=(temp_time/3600.0)*180.0;
		temp_time = 0;
	}
	//20260122有人机方案时间，航程计算结束
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		int temp_time = 0;
		for(int j = 0;j < information_on_the_results_of_taskings.formation_synergy_mission_programs[i].number_of_subtasks;j++)
		{
			int temp_time_before_stage = temp_time;

			//无人机速度单位m/s
			float speed = 0;
			speed = integrated_postures.integrated_posture_drone_informations[i-1].drone_speed / 3.6;
			// 避免悬停状态估算错误，设置特定值
			if(speed < 10)
			{
				speed = 28.0;
			}

			//任务区面积
			float area = 0;
			for(int a = 0 ; a < 8 ; a++)
			{
				if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].target_number ==
						area_sky_informations.area_informations[a].area_code && area_sky_informations.area_informations[a].area_code != 0)
				{
					//圆形
					if(area_sky_informations.area_informations[a].area_shape == 1)
					{
						float rad = area_sky_informations.area_informations[a].cycles.radius;
						area = M_PI * rad * rad * 1000 * 1000;

						if(j == 0)
						{
							//时间计算加上无人机到任务区的距离
							double d_area = 0;
							d_area = calculate_distances(integrated_postures.integrated_posture_drone_informations[i-1].drone_longitude_and_latitude.latitude
									,integrated_postures.integrated_posture_drone_informations[i-1].drone_longitude_and_latitude.longitude
									,area_sky_informations.area_informations[a].cycles.latitude
									,area_sky_informations.area_informations[a].cycles.longitude);
							temp_time += (d_area * 1000) / speed;
						}
					}
					//矩形
					else if(area_sky_informations.area_informations[a].area_shape == 2)
					{
						float Long = 0;
						float Wide = 0;
						Long = calculate_distances(area_sky_informations.area_informations[a].polygonals.point_coordinates[0].latitude,
								area_sky_informations.area_informations[a].polygonals.point_coordinates[0].longitude,
								area_sky_informations.area_informations[a].polygonals.point_coordinates[1].latitude,
								area_sky_informations.area_informations[a].polygonals.point_coordinates[1].longitude);
						Wide = calculate_distances(area_sky_informations.area_informations[a].polygonals.point_coordinates[1].latitude,
								area_sky_informations.area_informations[a].polygonals.point_coordinates[1].longitude,
								area_sky_informations.area_informations[a].polygonals.point_coordinates[2].latitude,
								area_sky_informations.area_informations[a].polygonals.point_coordinates[2].longitude);
						area = Long * Wide * 1000 * 1000;

						if(j == 0)
						{
							//时间计算加上无人机到任务区的距离
							double d_area = 1e20;
							for(int polyg = 0 ; polyg < 4 ; polyg ++)
							{
								double temp = 0;
								temp = calculate_distances(integrated_postures.integrated_posture_drone_informations[i-1].drone_longitude_and_latitude.latitude
										,integrated_postures.integrated_posture_drone_informations[i-1].drone_longitude_and_latitude.longitude
										,area_sky_informations.area_informations[a].polygonals.point_coordinates[polyg].latitude
										,area_sky_informations.area_informations[a].polygonals.point_coordinates[polyg].longitude);
								if(d_area > temp)
									d_area = temp;
							}
							temp_time += (d_area * 1000) / speed;
						}
					}
					break;
				}
			}

			//光电搜索
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 7 && area !=0 && speed != 0)
			{
				temp_time += area / (5000 * speed);
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = (area / (5000 * speed))/60;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = blk_dtms_ctas_010[i-1].GDSS;
			}
			//磁探搜索
			else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 5 && area !=0 && speed != 0)
			{
				temp_time += area / (600 * speed);
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = (area / (600 * speed))/60;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = blk_dtms_ctas_010[i-1].CTSS;
			}
			//悬停等待
			else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 12)
			{
				temp_time += 300;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 5;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = blk_dtms_ctas_010[i-1].PX;
			}
			//浮标帧收
			else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 1)
			{
				temp_time += 1200;//原6000
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = 20;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = blk_dtms_ctas_010[i-1].FBZS;
			}
			//磁探跟踪
			else if(information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].sequence_type == 6)
			{
				temp_time += ((3*3*6 - 1) *1000) / speed;
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].task_completion_time = ((3*3*6 - 1) *1000) / speed /60.0;//同一统一分钟单位
				information_on_the_results_of_taskings.formation_synergy_mission_programs[i].task_sequence_informations[j].mission_height = blk_dtms_ctas_010[i-1].CTGZ;
			}

			if(j < 8)
			{
				int stage_time = temp_time - temp_time_before_stage;
				if(stage_time > uav_stage_max_time[j])
				{
					uav_stage_max_time[j] = stage_time;
				}
			}
		}
		if(temp_time > longest_time)
			longest_time = temp_time;
		total_time[i] = temp_time;
		temp_time = 0;
	}
	if(total_time[0]>longest_time)
		longest_time = total_time[0];
	information_on_the_results_of_taskings.total_program_time = longest_time;
	if(is_strategy9_plan_layout_from_result(&information_on_the_results_of_taskings))
	{
		information_on_the_results_of_taskings.total_program_time =
				calc_strategy9_total_time(manned_stage_time, uav_stage_max_time, information_on_the_results_of_taskings.total_program_time);
	}
	//航程,全部无人机的航程
	for(int i = 1 ; i <= drone_state_informations.drone_number ; i ++)
	{
		//无人机速度单位km/h
		float speed = 0;
		float time = total_time[i];
		if(integrated_postures.integrated_posture_drone_informations[i-1].drone_speed < 10)
		{
			//默认速度
			speed = 100;
		}
		else
		{
			speed = integrated_postures.integrated_posture_drone_informations[i-1].drone_speed;
		}

		information_on_the_results_of_taskings.range += ((time / 3600) * speed);
	}
}

// 多架无人机时的任务区分配处理
void uav_area_distribute_proc()
{
	// 遍历8个阶段，每个阶段每个平台的任务区分开
	for(int i = 0 ;i < TASKNUMMAX ; i ++)
	{
		char area_used[8] = {0,0,0,0,0,0,0,0};
		for(int uav = 0 ; uav < drone_state_informations.drone_number; uav ++)
		{
			// 超过该平台任务数，忽略
			if(information_on_the_results_of_taskings.formation_synergy_mission_programs[uav+1].task_sequence_informations[i].sequence_type == 0)
			{
				continue;
			}

			// 第一阶段加上距离判定（将无人机分配给距离最近的任务区）
			if(i == 0)
			{
				uav_area_distribute_stage1_proc(i, uav, area_used);
				continue;
			}

			// 有人机当前阶段使用的任务区编号
			int manned_area_id = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number;
			for(int area = 0;  area < information_on_the_results_of_taskings.formation_synergy_mission_programs[0].number_of_subtasks; area ++)
			{
				if(area >= global_mission_planning_commandss.area_point_num)
				{
					int index = area % global_mission_planning_commandss.area_point_num;
					//过滤有人机使用的任务区和已使用的
					if(manned_area_id != area_sky_informations.area_informations[index].area_code && area_used[area] == 0)
					{
						information_on_the_results_of_taskings.formation_synergy_mission_programs[uav+1].task_sequence_informations[i].target_number
						= area_sky_informations.area_informations[index].area_code;
						area_used[area] = 1;
					}
//					information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i].target_number =
//							information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[i-1].target_number;
				}
				else
				{
					//过滤有人机使用的任务区和已使用的
					if(manned_area_id != area_sky_informations.area_informations[area].area_code && area_used[area] == 0)
					{
						information_on_the_results_of_taskings.formation_synergy_mission_programs[uav+1].task_sequence_informations[i].target_number
						= area_sky_informations.area_informations[area].area_code;
						area_used[area] = 1;
					}
				}

			}
		}
	}
}

// 多架无人机时的任务区阶段1时分配单独处理(stage:阶段；uav：无人机编号；area_used：任务区使用情况)
void uav_area_distribute_stage1_proc(int stage, int uav,  char area_used[8])
{
	// 遍历所有未使用的任务区和当前无人机距离，找到最近的分配给当前无人机
	double temMiniDis = 1e20;
	int minIndex = -1;// 初始化最近任务区是-1
	GeoLibDas location;
	//有人机初始位置赋值
	location.longitude = blk_dtms_ctas_001.solider_infos[uav+1].lon_lat_info.longitude;
	location.latitude = blk_dtms_ctas_001.solider_infos[uav+1].lon_lat_info.latitude;
	// 有人机当前阶段使用的任务区编号
	int manned_area_id = information_on_the_results_of_taskings.formation_synergy_mission_programs[0].task_sequence_informations[stage].target_number;
	for(int area = 0;  area < global_mission_planning_commandss.area_point_num; area ++)
	{
		//过滤有人机使用的任务区和已使用的
		if(manned_area_id != area_sky_informations.area_informations[area].area_code && area_used[area] == 0)
		{
			// 没有最近的任务区时，直接更新第一个没有使用的任务区为最近任务区
			if(minIndex == -1)
			{
				minIndex = area;
				continue;
			}


			// 比较剩下的和最近的，如果更近，则更新
			if(distanceComparison(location,area_sky_informations.area_informations[minIndex],area_sky_informations.area_informations[area]))
			{
				minIndex = area;
			}

		}

	}

	if(minIndex != -1)// 保护
	{
		information_on_the_results_of_taskings.formation_synergy_mission_programs[uav+1].task_sequence_informations[stage].target_number
							= area_sky_informations.area_informations[minIndex].area_code;
		area_used[minIndex] = 1;
	}

}
