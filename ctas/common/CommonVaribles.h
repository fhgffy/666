#ifndef COMMONDVARIBLES_H
#define COMMONDVARIBLES_H

#include <math.h>
#include <string.h>
#include "synthesizeCooperativeAllegationComputer.h"
#include "cooperativeAllegationComputer-synthesis.h"
#include "interfaceHead/interfaceDippingSonar.h"
#include "interfaceHead/interfaceRadarPhoto.h"
#include "interfaceHead/interfaceLibDas.h"
#include "Application/Application.h"
#include "interior_DTMS_to_CTAS.h"
#include "interior_CTAS_to_DTMS.h"
#include "interfaceHead/interfaceBuoy.h"
#include "interfaceHead/interfaceAirway.h"
#include "attacklib/PlacePosCalc.h"
#include "attacklib/in_out_param.h"
#include "attacklib/collaborative_attack.h"
#include "attacklib/manned_attack_position_calculation.h"
#include "attacklib/weapon_threat_area_calculation.h"
#include "attacklib/attack_safety_and_avoidance_calculation.h"
#include "attacklib/collaborative_attack_mode_allocation.h"
#include "DDSUtil/DDSUtil.h"

#include "NewSrc/common/CommonDefine.h"
#include "NewSrc/common/comm_func.h"

#pragma pack(1)


extern RETURN_CODE_TYPE enRetCode;
extern TRANSACTION_ID_TYPE 	transaction_id;
extern MESSAGE_TYPE_GUID message_type_id;
extern char *send_data;
extern MESSAGE_SIZE_TYPE data_length;
extern UINT32_SIX send_id;
extern MyByteArray dds_data_rece;
extern MyByteArray dds_data_send;
//    QByteArray send_array;
extern MESSAGE_SIZE_TYPE message_size;



/*
 *3 协同指控计算机-综显
 * */
extern MyByteArray send_array;

/*
 * 3.1 任务状态信息-发送
 * */
extern task_status_information task_status_informations;

/*
 * 3.2 无人机状态信息-发送
 * */
extern drone_state_information drone_state_informations;

/*
 * 3.3 编队链路状态信息-发送
 * */
extern formation_link_status_information formation_link_status_informations;

/*
 * 3.4 综合态势-发送
 * */
extern integrated_posture integrated_postures;//综合态势变量

/*
 *3.5 应急规避提示-发送
 * */
extern emergency_avoidance_tips emergency_avoidance_tipss;

/*
 *3.6 任务分配结果信息（事件发送）
 * */
extern Information_the_results_of_tasking information_on_the_results_of_taskings;

/*
 * 3.9 协同任务执行状态提示
 * */
extern collaborative_task_execution_status_alerts collaborative_task_execution_status_alertss;

/*
 * 3.10 任务区/空域信息
 * */
extern area_sky_information area_sky_informations;

/*
 * 3.11 任务点信息
 * */
extern taks_point_information taks_point_informations;

/*
 * 3.12 任务线信息
 * */
extern task_line_information task_line_informations;

/*
 * 3.13 协同指控指令信息
 * */
extern collaborative_charge_directive_information collaborative_charge_directive_informations;
/*
 * 3.15 预规划加载结构确认
 * */
extern confirmation_of_preplanning_loading_results confirmation_of_preplanning_loading_resultss;

/*
 * 3.16 预规划方案
 * */
extern pre_planning_program pre_planning_programs;

/*
 * 3.17 辅助决策错误提示
 * */
extern assisted_decision_making_error_alerts assisted_decision_making_error_alertss;

/*
 * 4.9
 * */
extern global_mission_planning_commands global_mission_planning_commandss;

/*
 * 航线重规划
 * */
extern task_sequence_information_confirm task_sequence_information_confirms;//子任务信息
extern Information_the_results_of_tasking_confirm information_on_the_results_of_tasking_confirms;//4.5任务分配结果确认信息
extern manned_aircraft_route_information manned_route_replanning_information;
extern drone_route_confirmation drone_route_replanning;

/*
 * 战术战法信息列表
 * */
extern StrategySelect minTimeStrategySelect;   //方案时间最短
extern StrategySelect minUAVStrategySelect;    //方案所需无人机最少
extern StrategySelect minMannedTask;           //方案有人机任务数量最少
extern StrategySelect otherStrategySelect;     //其他方案
extern int Select_NO[2];
/****************************************与DTMS交互 战术战法推荐及航路规划***************************************************************/
/*
 * 战术战法推荐指令-接收
 * */
extern BLK_DTMS_CTAS_001 blk_dtms_ctas_001;//全局规划
extern BLK_DTMS_CTAS_002 blk_dtms_ctas_002;//单任务区/目标，单无人机，攻击规划
extern BLK_DTMS_CTAS_005 blk_dtms_ctas_005;//单阶段航线生成信息
extern BLK_DTMS_CTAS_010 blk_dtms_ctas_010[2];//任务高度
extern BLK_OFP_CCC_302   blk_ofp_ccc_302;/*浮标—航路点解算 描述:只对本机航路点进行解算。*/
extern BLK_OFP_CCC_402   blk_ofp_ccc_402;/*吊声—吊声定测点规划*/
/*
 * 战术战法推荐结果---发送
 * */
extern zhanshuzhanfa_result CTAS_DTMS_data_tacticsResults;//原zhanshuzhanfa_results
extern int index_platform;//当前发送的是第几架机的序号
extern int index_task;//当前发送的任务序号
//新增 2025.3.3
extern BLK_CCC_OFP_302 buoy_arry_plan;     //浮标布阵规划
extern BLK_CCC_OFP_403 sonar_detect_plan;  //吊声定测点规划
extern BLK_CCC_OFP_005 taskarea_division;  //任务区划分信息
extern BLK_CTAS_DTMS_008		 blk_ctas_dtms_008;	 //安全区
extern BLK_CTAS_DTMS_010         blk_ctas_dtms_010;     //空域
/*
 * 3.7 有人机航路规划结果信息---发送
 * */
extern manned_aircraft_route_information CTAS_DTMS_data_mannedRoute;//原manned_aircraft_route_drone_route_informations
extern int manned_index;//当前发送的子任务序号
/*
 * 3.8 无人机航路规划结果---发送
 * */
extern drone_route_confirmation CTAS_DTMS_data_UAVRoute;//原drone_route_confirmations
extern int drone_index;//当前发送的无人机序号
extern int drone_subtask_index;//当前发送的无人机子任务序号
/************************************************************************************************************************************/


/**************************************与集群交互 集群航线规划***************************************************************************/
//集群战术推荐---接收
extern BLK_DTMS_CTAS_001 jiQun_CTAS_data_swarmRecommend;  //原jiqun_zhanshutuijian
//集群---通用航路
extern common_carrier_route CTAS_jiQun_data_swarmRoute;    //原jiqun_common_carrier_route

//（三）方案生成发送标志位
extern char fangan_send_flag;
extern unsigned int fangan_index;
//任务规划发送标志位
extern char renwu_guihua_flag;
//有人机航线规划发送标志位
extern char buoy_suspended_flag;
//无人机航线规划发送标志位
extern char wurenji_hangxian_flag;

//有人机子任务总路程
extern double manned_task_length;
//无人机子任务总路程
extern double UAV_task_length[4];



#pragma pack()
#endif // COMMONDEFINE_H
