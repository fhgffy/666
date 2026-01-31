#ifndef MAINTASK_H
#define MAINTASK_H

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

#include "NewSrc/common/comm_func.h"
#include "NewSrc/common/CommonVaribles.h"
#include <math.h>

#define MANNED_ID 	0x9001//有人机ID

#define TASKNUMMAX 8 // 任务阶段
#define AIRWAY_LIMIT	0 //空域限制
#define PX_CIRCLE		15 //盘旋圈数
#define PX_HIGH			500 //盘旋高度
#define PX_SPEED		35 //盘旋速度
#define PX_RAD			800 //盘旋半径
void createTaskFile();

void task_area_replan();
void yz_dispel_manned_strategy();
void dispel_manned_strategy();
//战术一
void firstStrategy();
/*
 * 3.6 任务分配结果信息
 * */
void missionDistributeInformation();//任务分配结果信息
void mannedAreaSorted();//组织有人机任务区顺序
char distanceComparison(GeoLibDas location,area_information area1,area_information area2);//比较有人机和任务点的距离，根据距离排序
char UAVdistanceComparison(integrated_posture_drone_information UAV1,integrated_posture_drone_information UAV2,area_information area);//比较有人机和任务点的距离，根据距离排序
void UAVAreaDistribute();//无人机分配任务序列
/*
 * 3.7 有人机航路规划结果信息
 * */
void mannedAircraftRoutesGeneration();
void init_blk_ofp_ccc_302(int stage);//有人机浮标解算航路转换
void init_blk_ofp_ccc_402(int stage);//有人机吊声规划航路转换
//浮标、吊声规划
void buoy_suspended_generation();
OutSearchRadarPhoto commonRouteGeneration(area_information area,GeoLibDas manned_position,int width);//3.7.1 通用航路
OutBuoyLayoutAuto buyoGeneration(area_information area,GeoLibDas manned_position);//3.7.2 浮标布阵
OutBuoyLayoutAirway buyo_way_generation(buoy_manned_computer_task buyo_info,GeoLibDas manned_position);//浮标布阵航路生成
OutBuoyMonitorAirway buyo_monitor_way_generation(buoy_manned_computer_task buyo_info,GeoLibDas manned_position);//浮标侦听航线生成
OutSearchDippingSonar SusSoundFixedGeneration(area_information area); //3.7.3 吊声定测点
OutAirwayLibDas citanGeneration(area_information area,GeoLibDas position);//磁探搜索航线生成

/*
 * 3.8 无人机航路规划方案生成
 * */
void UAVRouteGeneration();
void yz_CTGZ();

double minDistance(GeoLibDas UAVLocation,area_information area);//求最近的区域点距离
GeoLibDas minDIstancePoint(GeoLibDas UAVLocation,area_information area);//求最近距离的点

//战术二
void secondStrategy();
void secondStrategy_missionDistributeInformation();

//战术三
void thirdStrategy();
void thirdStrategy_missionDistributeInformation();

//战术四
void fourthStrategy();
void fourthStrategy_missionDistributeInformation();

//战术五
void fifthStrategy();
void fifthStrategy_missionDistributeInformation();
int used_buyo_number(area_information area);//所使用的浮标数量

//战术六
void sixthStrategy();
void sixthStrategy_missionDistributeInformation();

//战术七
void seventhStrategy();
void seventhStrategy__missionDistributeInformation();

//战术八
void eighthStrategy();
void eighthStrategy__missionDistributeInformation();

//方案推荐模块
void strategyRecommend();
void yinzhaoRecommend();//应召推荐
void init_zhanshutuijians();//战术战法推荐指令---赋值
void init_zhanshutuijians_single();//单任务战术战法推荐指令---赋值
void init_blk_dtms_ctas_005();//航线生成信息解析
void init_strategy_select(int strategyNo);//选择策略
void init_strategy(char plan_stats[]);//战术战法赋值模块
void isOtherStrategySelect(StrategySelect strategy);//判断原结果是否满足其他赋值条件
void task_area_division(region_info * area,int divi_num);//区域划分算法
void single_area_division();//单任务区指控任务区切分
void calc_air_area(int stage);//计算空域
void hx_avoid(int stage);//航线冲突规避
void single_air_area(unsigned int uav_index);//修改空域信息
void send_airway_area();//发送空域信息
int airway_area_confirm(double task_lon,double task_lat);//空域许可判断
void single_CT();//单目标磁探搜索
void single_area_FBZS();//单任务区浮标帧收
void single_area_CTSS();//单任务区磁探搜索
void single_area_GDSS();//单任务区光电搜索
void single_uav_FBZS();//单无人机浮标帧收规划
void single_uav_BDFX();//单无人机编队飞行规划
void double_uav_BDFX();//双机编队飞行规划
void single_uav_CTGZ();//单无人机磁探跟踪规划
void single_uav_CTSS();//单无人机磁探搜索规划
void single_uav_GDSS();//单无人机光电搜索规划
void single_uav_GDGZ();//单无人机光电跟踪规划
void single_uav_XT();//单无人机悬停规划
void XT_area_calc(double centerLon,double centerLat,int index);//悬停/盘旋空域计算
unsigned int distance_min(double lat, double lon);//计算目标与无人机最短距离的无人机索引
double calculate_distances(double lat1, double lon1,double lat2, double lon2);//计算距离
double calculate_angle(double lat1, double lon1,double lat2, double lon2);//计算夹角，正北0°
void calculate_rectangle_vertices(SGeo in,double azimuth,double side,SGeo*out);//计算以目标点为中心的矩形四个点经纬度
void calculate_rectangle_vertices_new(SGeo in,double azimuth,double side,SGeo*out);//计算以目标点为中心的矩形四个点经纬度
int single_area_FBZS_init(SGeo UAV_aircraft,unsigned int uav_index);
int FBZS_init(SGeo UAV_aircraft,unsigned int uav_index);//浮标帧收
void CTGZ_init(SGeo UAV_aircraft,float speed, SGeo goal_point ,unsigned int uav_index);//磁探跟踪航线生成
void GDGZ_init(SGeo UAV_aircraft,float speed, SGeo goal_point ,unsigned int uav_index);//光电跟踪航线生成
void BDFX_init(SGeo UAV_aircraft, SGeo goal_point ,unsigned int uav_index);//编队飞行航线生成
void BDFX_double_init(SGeo UAV_aircraft, SGeo goal_point1 ,SGeo goal_point2 ,unsigned int uav_index);//双机编队
double to_radians(double degree);//转弧度制
double angle_range_process( double angle);//角度修正
void calculate_distance_bearing(SGeo start, SGeo end, double *distance, double *bearing);//计算编队飞行的距离和方向
SGeo calculate_new_coordinate(SGeo origin, double distance, double bearing);//计算编队飞行的点经纬度
int lineIntersection(Grid p1, Grid p2, Grid p3, Grid p4, Grid* intersection);
int detectConflict(FlightRoute* routeA, FlightRoute* routeB);//航线段冲突检测
char avoidLineCrashIsCrossRect(FlightRoute* route1, AreaRectVertex* resAreaRectVertex);// 判读航线是否穿过正方体
//DDS接收模块
void formulate_moduel();//全局规划
void signal_moduel();//单任务区/目标、单无人机规划，攻击规划
//DDS发送模块
void send_tactics_result();
void send_single_result();
void send_buoy_result();
void send_sonar_result();
void send_taskarea_result();

//3.7 有人机航路规划结果信息发送
void send_manned_aircraft_route_drone_route_information(int i);
void send_manned_route();

//3.8 无人机航路点信息发送
void send_drone_route_confirmation(int i,int j);
void send_drone_route();

//与动态任务管理软件交互
//    void dynasmic_rece_send();
void align_send_information(void *send_struct,int length,int startPos);

//应召点变成区域
void yinzhao_point_init();
//应召---战术一
void yingzhao_firstStrategy();
void yingzhao_firstStrategy_missionDistributeInformation();
//应召区域划分,根据中心点划分出任务区域
void yingzhao_area_divide(point_coordinate point,unsigned int time);
int get_bit(unsigned short data, int n);

//应召---战术二
void yingzhao_secondStrategy();
void yingzhao_secondStrategy_missionDistributeInformation();
void yingzhao_secondStrategy_areaDivid(point_coordinate point,unsigned int time);//应召战术二的区域划分

//应召---战术三
void yingzhao_thirdStrategy();
void yingzhao_thirdStrategy_missionDistributeInformation();
void yingzhao_thirdStrategy_areaDivid(point_coordinate point,unsigned int time);//应召战术三的区域划分

//应召---战术四
void yingzhao_fourthStrategy();
void yingzhao_fourthStrategy_missionDistributeInformation();
void yingzhao_fourthStrategy_areaDivid(point_coordinate point,unsigned int time);//应召战术四的区域划分

//应召－－－战术五
void yingzhao_fifthStrategy();
void yingzhao_fifthStrategy_missionDistributeInformation();
void yingzhao_fifthStrategy_areaDivid();

//任务重规划－－－接收－规划－发送
void task_replanning();//无人机重规划函数---接收---规划---发送

PolygonLibDas* loop_to_RectLibDas(GeoLibDas centre,double radius_large,double radius_little,int num);//环形区域生成外接矩形
RectLibDas* cycle_to_RectLibDas(GeoLibDas centre,double radius);//圆形区域生成内接矩形
void adjust_longitude(double *lon);
void adjust_latitude(double *lat);
//无人机集群消息通讯
void UAV_jiqun();

//攻击生成结果赋值
void attack_init(OutputResults output,unsigned int fangan);
//协同攻击决策
void attack_commend(OutputResults* output,int plan);
//发送安全区、威胁区
void send_blk_ctas_dtms_008();
//新航路规划
void new_route_generate(SGeo UAV_aircraft,float speed,SGeo goal_point ,unsigned int uav_index);

void initVar();//初始化清零指针变量

void recv_blk_dtms_ctas_010();//接收任务高度信息
//    void ddsSend();
//    void ddsReceive_Send();
void main_task();

//无人机方案时长航程计算
void uav_time_range_calc();
//无人机分派方式
void uav_dispatch();
//三架飞机，任务区无冲突分配
void uav_area_distribute_third();
// 多架无人机时的任务区分配处理
void uav_area_distribute_proc();
// 多架无人机时的任务区阶段1时分配单独处理(stage:阶段；uav：无人机编号；area_used：任务区使用情况)
void uav_area_distribute_stage1_proc(int stage, int uav,  char area_used[8]);


#endif // MAINTASK_H
