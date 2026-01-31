#include "CommonVaribles.h"


RETURN_CODE_TYPE enRetCode;
TRANSACTION_ID_TYPE 	transaction_id;
MESSAGE_TYPE_GUID message_type_id = -1;
char *send_data;
MESSAGE_SIZE_TYPE data_length;
UINT32_SIX send_id;
MyByteArray dds_data_rece;
MyByteArray dds_data_send;
//    QByteArray send_array;
MESSAGE_SIZE_TYPE message_size;



/*
 *3 协同指控计算机-综显
 * */
MyByteArray send_array;

/*
 * 3.1 任务状态信息-发送
 * */
task_status_information task_status_informations;

/*
 * 3.2 无人机状态信息-发送
 * */
drone_state_information drone_state_informations;

/*
 * 3.3 编队链路状态信息-发送
 * */
formation_link_status_information formation_link_status_informations;

/*
 * 3.4 综合态势-发送
 * */
integrated_posture integrated_postures;//综合态势变量

/*
 *3.5 应急规避提示-发送
 * */
emergency_avoidance_tips emergency_avoidance_tipss;

/*
 *3.6 任务分配结果信息（事件发送）
 * */
Information_the_results_of_tasking information_on_the_results_of_taskings;

/*
 * 3.9 协同任务执行状态提示
 * */
collaborative_task_execution_status_alerts collaborative_task_execution_status_alertss;

/*
 * 3.10 任务区/空域信息
 * */
area_sky_information area_sky_informations;

/*
 * 3.11 任务点信息
 * */
taks_point_information taks_point_informations;

/*
 * 3.12 任务线信息
 * */
task_line_information task_line_informations;

/*
 * 3.13 协同指控指令信息
 * */
collaborative_charge_directive_information collaborative_charge_directive_informations;
/*
 * 3.15 预规划加载结构确认
 * */
confirmation_of_preplanning_loading_results confirmation_of_preplanning_loading_resultss;

/*
 * 3.16 预规划方案
 * */
pre_planning_program pre_planning_programs;

/*
 * 3.17 辅助决策错误提示
 * */
assisted_decision_making_error_alerts assisted_decision_making_error_alertss;

/*
 * 4.9
 * */
global_mission_planning_commands global_mission_planning_commandss;

/*
 * 航线重规划
 * */
task_sequence_information_confirm task_sequence_information_confirms;//子任务信息
Information_the_results_of_tasking_confirm information_on_the_results_of_tasking_confirms;//4.5任务分配结果确认信息
manned_aircraft_route_information manned_route_replanning_information;
drone_route_confirmation drone_route_replanning;

/*
 * 战术战法信息列表
 * */
StrategySelect minTimeStrategySelect;   //方案时间最短
StrategySelect minUAVStrategySelect;    //方案所需无人机最少
StrategySelect minMannedTask;           //方案有人机任务数量最少
StrategySelect otherStrategySelect;     //其他方案
int Select_NO[2] = {0,0};
/****************************************与DTMS交互 战术战法推荐及航路规划***************************************************************/
/*
 * 战术战法推荐指令-接收
 * */
BLK_DTMS_CTAS_001 blk_dtms_ctas_001;//全局规划
BLK_DTMS_CTAS_002 blk_dtms_ctas_002;//单任务区/目标，单无人机，攻击规划
BLK_DTMS_CTAS_005 blk_dtms_ctas_005;//单阶段航线生成信息
BLK_DTMS_CTAS_010 blk_dtms_ctas_010[2];//任务高度
BLK_OFP_CCC_302   blk_ofp_ccc_302;/*浮标—航路点解算 描述:只对本机航路点进行解算。*/
BLK_OFP_CCC_402   blk_ofp_ccc_402;/*吊声—吊声定测点规划*/
/*
 * 战术战法推荐结果---发送
 * */
zhanshuzhanfa_result CTAS_DTMS_data_tacticsResults;//原zhanshuzhanfa_results
int index_platform = 0;//当前发送的是第几架机的序号
int index_task = 0;//当前发送的任务序号
//新增 2025.3.3
BLK_CCC_OFP_302 buoy_arry_plan;     //浮标布阵规划
BLK_CCC_OFP_403 sonar_detect_plan;  //吊声定测点规划
BLK_CCC_OFP_005 taskarea_division;  //任务区划分信息
BLK_CTAS_DTMS_008		  blk_ctas_dtms_008;	 //安全区
BLK_CTAS_DTMS_010         blk_ctas_dtms_010;     //空域
/*
 * 3.7 有人机航路规划结果信息---发送
 * */
manned_aircraft_route_information CTAS_DTMS_data_mannedRoute;//原manned_aircraft_route_drone_route_informations
int manned_index = 0;//当前发送的子任务序号
/*
 * 3.8 无人机航路规划结果---发送
 * */
drone_route_confirmation CTAS_DTMS_data_UAVRoute;//原drone_route_confirmations
int drone_index = 0;//当前发送的无人机序号
int drone_subtask_index = 0;//当前发送的无人机子任务序号
/************************************************************************************************************************************/


/**************************************与集群交互 集群航线规划***************************************************************************/
//集群战术推荐---接收
BLK_DTMS_CTAS_001 jiQun_CTAS_data_swarmRecommend;  //原jiqun_zhanshutuijian
//集群---通用航路
common_carrier_route CTAS_jiQun_data_swarmRoute;    //原jiqun_common_carrier_route

//（三）方案生成发送标志位
char fangan_send_flag = 0;
unsigned int fangan_index = 4;
//任务规划发送标志位
char renwu_guihua_flag = 0;
//有人机航线规划发送标志位
char buoy_suspended_flag = 0;
//无人机航线规划发送标志位
char wurenji_hangxian_flag = 0;

//有人机子任务总路程
double manned_task_length = 0;
//无人机子任务总路程
double UAV_task_length[4] = {0};



