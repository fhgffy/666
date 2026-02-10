/*
 * mainTask.h
 *
 *  Created on: 2025年4月20日
 *      Author: Admin
 */

#ifndef MAINTASK_H_
#define MAINTASK_H_




#include "CommonVaribles.h"


void Main_Task();

/***********************防撞规避算法接口*************************/
void task_replan(); // 任务发生重新规划
void avoid_collision(); // 防撞规避算法
int threat_judge(longitude_and_latitude_synt L,longitude_and_latitude_synt N); // 威胁判定函数
void receive_newHD_plane(); // 接收有人机重规划返回的新航点
void receive_newHD_uav(); // 无人机接收
void send_algorithm_info(); // 发送威胁信息 + 待航点
void rev_threat_info_hold_point(); // 接收威胁信息等
void rev_zhanfa_plan(); // 接收战法规划方案
void receive_zhanfa_hl(); // 接收战法规划方案结果的航路信息
void receive_buoy_soanr();//接收浮标点和吊声点
void receive_zhanfa_uav_hl(); // 接收战法规划方案结果的航路信息

void uav_angle_handel(); // 3.4 综合态势无人机方位角计算

/***********************外部信息收发接口************************/
void align_send_information(void *send_struct,int length,int startPos);//发送信息对齐,发送的结构体，发送长度，开始位置
//任务状态信息-发送
void init_task_status_information();
void send_task_status_information();
//接收消息
void dataReceived();
//综合态势信息
void init_integrated_posture();
void send_integrated_posture();
//无人机状态信息
void init_drone_state_information( int i);
void warnning_detection();//警告检测
double calculate_distances(double lat1, double lon1,double lat2, double lon2);
void send_drone_state_information();
//编队链路状态信息
void init_formation_link_status_information();
void send_formation_link_status_information();
//应急规避提示
void init_emergency_avoidance_tips();
void send_emergency_avoidance_tips();
//任务分配结果信息
void init_blk_ccc_ofp_021();//单无人机分配结果赋值
void send_blk_ccc_ofp_021();//单无人机分配结果
void send_blk_ccc_ofp_017();
void send_blk_ccc_ofp_019();
void send_blk_ccc_ofp_019_special(); // 针对全局规划时非紧密排列排列时的特殊处理

//赋值编队信息
void init_blk_ccc_ofp_026();
//编队状态
void send_blk_ccc_ofp_026();
//赋值有人机领航进入信息
void init_blk_ccc_ofp_027();
//有人机领航进入
void send_blk_ccc_ofp_027();
//赋值有人机领航退出信息
void init_blk_ccc_ofp_028();
//有人机领航退出
void send_blk_ccc_ofp_028();
//无人机碰撞信息赋值
void init_blk_ccc_ofp_029();
//无人机碰撞启动信息
void send_blk_ccc_ofp_029();
//应急返航区域信息赋值
void init_blk_ccc_ofp_030();
//应急返航区域
void send_blk_ccc_ofp_030();
//有人机航路规划结果信息
void send_buoy_soanr_route_information(unsigned int plan,int task);
//任务区/空域信息
void init_area_sky_informations();
//单无人机航路规划
void send_blk_ccc_ofp_024_single(unsigned int plan,unsigned int single_index);
//无人机航路规划
void send_blk_ccc_ofp_024(unsigned int plan);
//发送当前阶段反馈
void init_blk_ccc_ofp_025();
void send_blk_ccc_ofp_025();
//协同任务执行状态提示
void init_collaborative_task_execution_status_alerts();
void send_collaborative_task_execution_status_alerts();
//任务区/空域信息
void init_blk_ccc_ofp_033();
void send_blk_ccc_ofp_033();
//任务点信息
void init_blk_ccc_ofp_034();
void send_blk_ccc_ofp_034();
//任务线信息
void init_blk_ccc_ofp_035();
void send_blk_ccc_ofp_035();
//预规划加载结果确认
void init_confirmation_of_preplanning_loading_results();
void send_confirmation_of_preplanning_loading_results();
//预规划方案
void init_pre_planning_program();
void send_pre_planning_program();
//辅助决策错误提示1
void init_assisted_decision_making_error_alerts();
void send_assisted_decision_making_error_alerts();
//协同指控指令信息
void init_blk_ccc_ofp_036(unsigned short command_type,unsigned short state,unsigned short task_type);
void send_blk_ccc_ofp_036();

//3.18 协同指控系统自检测故障清单
void init_default_orders();
void send_default_orders();

// 新增信息
//无人机光电控制权反馈
void init_guangdian_kongzhi_feedback();
void send_guangdian_kongzhi_feedback();

// 控制权限交接
void init_kongzhiquan_jiaojie();
void send_kongzhiquan_jiaojie();

// 编辑方案状态
void scheme_generation_state(char fanganType,char fanganGenStatus,char fanganSubStatus,unsigned char hangxianGenStatus);
//分阶段发送航线
void stage_send();
//航线修改
void route_change(int i);
/******************************* 协同指控计算机-协同通信系统 *********************/
//空地链链路遥测信息
void init_Air_ground_link_telemetry_information();
void send_Air_ground_link_telemetry_information();
//编队综合态势
void init_blk_ccc_kdl_000();
void send_blk_ccc_kdl_000();
//控制权交接申请
void init_blk_ccc_kdl_002(int uav_index);// uav_index无人机序号（0-3）
void send_blk_ccc_kdl_002(int uav_index);// uav_index无人机序号（0-3）
//无人机1光电视频
void init_blk_ccc_kdl_004();
void send_blk_ccc_kdl_004();
// 控制权交接申请应答
void init_blk_ccc_kdl_010();
void send_blk_ccc_kdl_010();
//无人机2光电视频
void init_blk_ccc_kdl_012();
void send_blk_ccc_kdl_012();
//空地链链路信息
void init_blk_ccc_kdl_014();
void send_blk_ccc_kdl_014();
//协同数据链状态信息数据帧
void init_collaborative_data_chain_status_message_data_frame();
void send_collaborative_data_chain_status_message_data_frame();
//任务分配结果信息
void init_blk_ccc_kdl_017();
void send_blk_ccc_kdl_017();
//有人机航路规划结果信息
void init_blk_ccc_kdl_018();
void send_blk_ccc_kdl_018();
//无人机航路规划（空地链）
void init_blk_ccc_kdl_024();
void send_blk_ccc_kdl_024();
//任务区/空域信息
void init_blk_ccc_kdl_033();
void send_blk_ccc_kdl_033();
//任务点信息
void init_blk_ccc_kdl_034();
void send_blk_ccc_kdl_034();
//任务线信息
void init_blk_ccc_kdl_035();
void send_blk_ccc_kdl_035();
//1.12-1.15
void init_basic_remote_control_commands();
void send_basic_remote_control_commands();
//2.1-2.3
void init_yaoce_data_zizhen_1();
void init_yaoce_data_zizhen_2();
void init_yaoce_data_zizhen_B1();
void init_yaoce_data_zizhen_B2();
void init_yaoce_data_zizhen_C();
//视频下传设置指令帧
void init_video_downlink_setting_command_frames();
//声纳浮标目标信息
void init_sonobuoy_targeting_information();

/*
 * 协同通讯空地链-协同指控计算机
 * */

/*
 * 协同通讯空空链-协同指控计算机
 * */


void data_receive();
void udp_data_send();

//5.1
void init_wurenjifubiaoyaokongxinxi();

int get_bit(unsigned short data,int n);
int get_int8_bit(char data, int n);
int give_bit(unsigned short data, int n, int value);
void align_receive_message(void *send_struct,int length);

/**********************内部通信 DTMS_CTAS 发送相关辅助函数*************************/
void init_zhanshutuijian();   // 战术战法推荐规划指令
void send_zhanshutuijian();


/*************************** 无人机群通信 ***************************/
void init_uav_cluster_info();
void uav_cluster_send();
// 接收无人机群信息
void receive_uav_cluster_info();

/************* 规划模块 **************/
void formulate_moduel();
void send_blk_ccc_ofp_018(int k,unsigned int plan,unsigned int task);


/************************************/


/************* 重规划模块 ************/
void scheme_replan();
void send_blk_ccc_ofp_020(unsigned int Plan_ID,unsigned char ModifyType,unsigned char modify_state,unsigned char * failreason);

/************************************/


/************************************/

/***** 无人机状态处理 + 通信 模块 ******/
void uav_status_handle();  // 初始化赋值uav 信息
void recv_blk_kkl_ccc_000_008_010_011(); // 接收 uav数据帧接口
void fault_message_init(int i);//解析遥测帧中的故障信息
void decode_uav_frame(int uav_index); // 解码黑包数据帧 uav_index:无人机序号（0-3）

/************************************/





/*********** 单无人机规划模块 **********/
void insert_cover();//任务插入或覆盖
void single_uav_plan();//单无人机
void init_single_uav_zhanfa();
void send_single_uav_zhanshutuijian();
void single_mission_target_plan();//单任务区/目标
void init_single_mission_target_zhanfa();
void send_single_mission_zhanshutuijian();
void double_uav_BDFX(int uav_index);//双机编队注入
void double_uav_BDON(int uav_index);//双机编队开始
void single_uav_BDFX();//编队注入
void single_uav_BDON();//编队开始
void single_uav_lhcs(unsigned short uav_index);//领航参数
void single_uav_pxlh(unsigned short uav_index);//盘旋领航
void single_uav_XT();//悬停注入
void cover_inject(int uav_index);//覆盖注入
int edit_uav_route(int i);//单无人机航线修改

void uav_hx();//无人机航线查询
void uav_simulation(); // 无人机飞仿控制
void BDFX_double_rtn();//双机编队
void BDFX_rtn();//编队飞行发布反馈
void send_blk_ccc_ofp_006(); //下一阶段提示
void recv_blk_ofp_ccc_041(); //下一阶段提示反馈
void send_uav_hl(unsigned int uav_id , unsigned char order); // 发送无人机航路给飞仿
Point last_second_point(double lat1, double lon1,double lat2, double lon2);//计算倒数第二个点的经纬度
double deg2rad(double deg);//角度转弧度
double rad2deg(double rad);//弧度转角度
double calculate_azimuth(double lat1, double lon1,double lat2, double lon2);//计算从点1到点2的初始方位角(弧度，范围0,2π)
void calculate_endpoint(double start_lat,double start_lon,double azimuth,double distance,double *end_lat,double *end_lon);//根据起点、方位角和距离计算终点经纬度
void recv_kkl_ccc(); // 接收 协同通信信息  空空链->协同指控计算机
void init_ckpartset();
void send_ckpartset();// 空空链参数设置
void init_link_controls();
void send_link_controls();// 链路交接控制指令
void init_blk_ccc_kkl_008_026_027_028(int uav_index);
void init_blk_ccc_kkl_008_026_027_028_front(int uav_index);
void init_blk_ccc_kkl_008_026_027_028_tail(int uav_index);
void recv_blk_ofp_ccc_148();//接收综显的事件指令
void send_blk_ccc_ofp_120(unsigned char response);//发送指令微调回报
void send_blk_ccc_kkl_008_026_027_028(); // 无人机遥控帧

void init_blk_ccc_kkl_009_029_030_031(int uav_index);
void send_blk_ccc_kkl_009_029_030_031(int uav_index);

void set_blk_ccc_kkl_000(int uav_index, unsigned int* p_main_link_send);
void init_blk_ccc_kkl_000();
void send_blk_ccc_kkl_000();

/************************************/

/************************************/

/**********************************/
//DPM交互
void send_DPM1A_1();
void send_DPM1A_2();
void send_DPM1A_3();
void send_DPM1A_4();
void send_DPM1A_5();
void send_DPM1A_6();
void send_DPM1A_7();
void send_DPM1A_8();
void send_DPM1A_9();
void send_DPM1A_10();
void send_DPM1A_11();
void send_DPM1A_12();
void recv_DPM1A_message();

//UAV控制权交接
void set_blk_ccc_kkl_009_029_030_031(int uav_index, unsigned int* p_main_link_send); // 设置blk_ccc_kkl_009_029_030_031包并发送
void UAV_Takeover_Process_get(int uav_index, unsigned int* p_take_over_flag);// uav_index:无人机序号；p_take_over_flag：交接状态指针
void UAV_Takeover_Process_free(int uav_index, unsigned int* p_take_over_flag);// uav_index:无人机序号；p_take_over_flag：交接状态指针
void UAV_Takeover_Process_get_hand(int uav_index, unsigned int* p_hand_get_step);
void UAV_Takeover_Process_get_hand_test(int uav_index, unsigned int* p_hand_get_step);  // 手动获取控制权跳过链交接
void UAV_Takeover_Process_free_hand(int uav_index, unsigned int* p_hand_free_step);
void UAV_Takeover_Process_free_hand_test(int uav_index, unsigned int* p_hand_free_step); //手动释放控制权跳过链交接
int up_down_lock(unsigned int uav_index);//上下行是否锁定
void UAV_Takeover_Process();
void hanover_state(int uav_index,unsigned char ctrl_releasing,unsigned char ctrl_recieving,unsigned short status);
void fc_order_send(int uav_index, int actionFlag);//// uav_index无人机序号（0-3）; actionflag 0:开始交接；1：交接结束

// 光电相关
void recv_DPU_CCC_MFD();
void send_uav_photoelectric_video_ctrl_feedback();
void recv_telemetry_data_subframe2();
void recv_telemetry_data_subframe2_single(int uav_index);


/***********************遥测数据帧解析*************************************/
/// 解析遥测数据帧子帧4D  uav_index:无人机序号（0-3）
void parseYaoCeZiZhen82(int uav_index);
void parseYaoCeZiZhen4D(int uav_index);
void parseYaoCeZiZhen3A(int uav_index);
/***********************态势融合测试接口*************************************/
void init_blk_ccc_ofp_015();//态势融合测试接口
void send_blk_ccc_ofp_015();//态势融合测试接口
void init_blk_ccc_ofp_045();//应急航线初始航线
void send_blk_ccc_ofp_045();//发送应急航线
void send_blk_ccc_ofp_032();//发送重规划反馈

/***********************DLR预规划加载*********************/
void recv_blk_dpu_dlr_011();
void send_blk_ccc_dlr_000(unsigned char states);
void send_load_file(unsigned int index);
void dlr_cpy_airway(unsigned int stage,unsigned int plan);
void send_uav_airway(unsigned int plan,unsigned int id);

/***********************无人机协同照射攻击*********************/
void send_blk_dpu_ccc_020();
int pnpoly(Point * polygon,int n , Point p);
#if _PC_SIMULATION_
void writeLog(char* block, int length);
#endif
void recv_ofp_message();
void recv_blk_ofp_ccc_000();
void recv_blk_ofp_ccc_018();
void recv_blk_ofp_ccc_040();
void recv_blk_ofp_ccc_154();
void recv_blk_ofp_ccc_037();
void recv_blk_ofp_ccc_045();//接收综显磁探杆指令
void recv_blk_ofp_ccc_047();//接收综显语音指令
void recv_blk_ofp_ccc_053();//接收领航指令
void recv_blk_ofp_ccc_155();//接收编队能力控制指令
void recv_blk_ofp_ccc_156();//航线查询指令


/***********************航线查询************************************/
void send_blk_ccc_ofp_157(unsigned char status);//航线查询状态
void send_blk_ccc_ofp_158();//查询的航线信息

/***********************语音消息************************************/
void send_blk_ccc_ofp_043();
BOOL signleUavCmdProtect();

/***********************接收SMD消息************************************/
void recv_SMD_message();
void recv_blk_smd_ccc_000();
void recv_blk_smd_ccc_001();
/***********************接收空地链消息************************************/
void recv_kdl_message();
void recv_blk_kdl_ccc_002();
void recv_blk_kdl_ccc_003();
void recv_blk_kdl_ccc_015();

/***********************接收pad信息************************************/
void recv_pad_message();
void recv_pad_heart(); // 心跳
void recv_pad_ccc_778(); // 有人机航线决策信息

void recv_all_message();
void send_period_message();
void send_ofp_period_message();
void send_kdl_period_message();

void send_message();
void init_drone_information_Azhen(int uav_index);
void drone_information_send_Bzhen();
void parse_blk_piu_ccc_006_key01(int uav_index);
void parse_blk_piu_ccc_006_key02(int uav_index);
void parse_blk_piu_ccc_006_key03(int uav_index);


/****************目标融合部分处理*******************************************/
void send_sfs_period_message();
void send_blk_ccc_sfs_001();




void send_track_change(); // 发送无人机飞仿 航线切换指令 10个周期后运行
void send_blk_dtms_ctas_005(unsigned int plan);//发送航线生成信息
void send_blk_dtms_ctas_010();//发送任务高度信息
void recv_blk_ctas_dtms_009();//解算反馈
void recv_blk_ctas_dtms_047(); //接受辅助决策安全区
void send_blk_ccc_ofp_047();//发送安全区到综显
void recv_blk_ctas_dtms_010();//接收空域信息
void init_blk_ctas_dtms_010();//应用空域
void init_param();//初始化参数
void recv_dpu1_dpu2(SINT32_SIX dpu_1,SINT32_SIX dpu_2,void*buf,int len);//接收DPU1/DPU2数据处理

/**************************航线冲突检测及处理******************************************/
void avoidLineCrashProc();// 航线冲突主入口
void avoidLineCrashJudgeProc();// 航线冲突判断入口
void avoidLineCrashJudgeDouble();//两架无人机航线冲突判断
int avoid_uav_manned(int drone_index);//有人机与无人机航线段比较
void avoid_uav1_uav2();//无人机1与无人机2航线段比较（考虑有人机情况）
void avoid_uav1_uav2_second(ConflictResult* tmp);//无人机1与无人机2航线段第二次恢复比较
void avoidLineCrashJudgeSingle(int drone_index);// 单架无人机航线冲突判断
void avoidLineCrashSafeTestProc();// 航线冲突持续检测
void avoidLineCrashFabuProc();// 航线冲突发布处理
void avoidLineCrashPanXuanProc();// 航线增加盘旋点处理
int avoidLineCrashSecondLineProc(int drone_index);// 第二次航线规避处理
void avoidLineCrashSecondDoulbe();//一控二第二次航线规避处理
void avoid_doulbe_uav(int drone_index,ConflictResult tmp);//无人机机互相规避
BOOL avoidLineCrashIsCrossRect();// 判读航线是否穿过正方体
void avoidLineCrashGetCurrentManRect(SubdomainInfo* subdomainCoordinate);// 找到当前有人机任务区
int avoid_uav1_uav2_period();//无人机1,2航线段比较
void send_blk_ccc_ofp_159();//规避信息

/**************************载荷冲突检测及处理******************************************/
void payload_detection();//载荷重规划检测
void payload_replan();//载荷任务修改
void payload_task();//修改后载荷任务发布
void payload_listen();//监测正在飞行的无人机载荷是否失效

/***********************发布函数，便于复用*******************************************/
void faBuProc();
/****接收中间件的时间消息****/
void recv_mids_message();

/************************收辅助决策方案和航线等特殊处理未与综显通信icd格式：**********************/
/*** 因为辅助决策是紧密排列，但是多无人机情况下，要求同无人机一次任务内，编队编号和存储位置对应且不变。所以会出现
 * 有一架机时，不再数组第一个位置，所以，需要将辅助决策生成的方案根据编队编号调整到相应数组下标下（综显是根据下标取得）******
 * ************************************************/
void castCtasToOfpPlan(BLK_CCC_OFP_019* p_blk_ccc_ofp_019);// 非紧密排列时，特殊处理方案
void castCtasToOfpLine(BLK_CCC_OFP_024* p_blk_ccc_ofp_024);// 非紧密排列时，特殊处理航线
BOOL castCtasToOfpIsNeeded();// 返回当前编队是否存在非紧密排列，（这种情况需要特殊处理）

int getUavCurrentTask(int drone_index);// 找到当前编号飞机的当前任务

#endif /* MAINTASK_H_ */
