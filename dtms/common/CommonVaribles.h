/*
 * CommonVaribles.h
 *
 *  Created on: 2025年4月20日
 *      Author: Admin
 */

#ifndef COMMONVARIBLES_H_
#define COMMONVARIBLES_H_


#include "icd_ccc_ofp.h"
#include "icd_ofp_ccc.h"
#include "cooperativeAllegationComputer_cooperativeCommunicationSystems.h"
#include "cooperativeCommunicationSystem-cooperativeChargComputer.h"
#include "jbykzl.h"
#include "icd_kkl_ccc.h"
#include "icd_ccc_kkl.h"
#include "icd_kdl_ccc.h"
#include "icd_ccc_kdl.h"
#include "kongdilian_jisuanji.h"
#include "toXTZK.h"
#include "wurenyaokongzhiling.h"
#include "interior_DTMS_to_CTAS.h"
#include "interior_CTAS_to_DTMS.h"
#include "interoir_DTMS_PAD.h"
#include <String.h>
#include "aide_func.h"
#include "DDSUtil.h"
#include "Application.h"
#include "interfaceLibDas.h"
#include "aide_func.h"
#include "DPM1A.h"
#include "yaocedatazizhen.h"
#include "comm_func_crc16.h"
#include "icd_ccc_dlr.h"
#include "icd_dlr_ccc.h"
#include "icd_piu_ccc.h"
#include "CommonDefine.h"
#include "icd_pmd_smd_ccc.h"
#include "flightConflictDetection.h"
#include "icd_ccc_mmm.h"
#include "icd_pad_ccc.h"
#include "icd_ccc_pad.h"

#pragma pack(1)

#if __cplusplus
//extern "C"
//{
#endif


typedef struct
{
    unsigned char type_id;//任务编号
    unsigned int speed;//任务速度
    unsigned int height;//任务高度
}TASK_PARAM;


extern RETURN_CODE_TYPE enRetCode;
extern TRANSACTION_ID_TYPE 	transaction_id;
extern MESSAGE_TYPE_GUID message_type_id;
extern char *send_data;
extern MESSAGE_SIZE_TYPE data_length;
extern UINT32_SIX send_id;


extern MESSAGE_SIZE_TYPE message_size;


// 定义当前软件的ip地址全局变量
extern char g_curAppIP[32];


/********** 存储相关信息容器 ***********/
extern MyByteArray send_array;             // 存储发送信息
extern MyByteArray dds_data_rece;          // 存储 除pad外其余信息
extern MyByteArray target_hj_data_rece;    // 存储 传感器航迹预测信息
extern MyByteArray hl_array;               // 存储 所有战法航路信息
extern MyByteArray pad_recv;            // 存储 pad接收数据
/************************************/
/********** 航线注入相关参数 ***********/
extern TASK_PARAM task_param[9];
/************************************/
/********** 辅助全局变量定义 ***********/
extern int DPU_online_flag;                // 综显上线 标志
extern int online_flag;                    // 判断上线一次后就发送预加载信息
extern double plane_lat,plane_lon;             // 保存收到的有人机经纬度  用于动态设置预加载区域
extern int hl_point_flag;                  // 航路点分包检测
extern int subtask_count;                  // 计数发送八个周期 （每个子任务一个一个周期（即分包发送））
extern int plan_recv_flag;                 // 战法方案收到标志
extern int stop_subtask_flag;                  // 子任务发送停止标志
extern unsigned short shuxing;                 // 任务分配方案 方案号 用于区分多个规划方案
extern unsigned short plant_count;         // 平台个数计数
extern unsigned short plant_subtask_count; // 平台子任务计数
extern int formulate_flag;                 // 全局规划标志
extern int formulate_single;               // 全局/单无人机标志 1-全局 2-单机
extern int single_mission_target_flag;     // 单任务/目标规划标志
extern int mission_insert_cover;           // 单任务/目标插入覆盖标志
extern int single_uav_flag;                // 单无人机规划标志
extern int single_uav_fabu_flag;               //单无人机接收发布航线标志
extern int uav_insert_cover;               // 单无人机插入覆盖标志
extern int uav_info_recv_flag;             // 无人机飞行遥测帧接收标志
extern unsigned char frame_count;          // 帧计数
extern int uav_hl_confirm;                 // 接收综显无人机航路确认信息接收标志
extern int send_index;                 //发送的无人机位置
extern unsigned int planning_id;                        //运行方案id
extern int communication_54_flag;          // 协同通信给出的能否通信信号
extern int track_point_count;             // 发送给无人机航路点计数 检测与综显确认航线是否相同
extern int send_count;                         // 发送计数  4次  发送无人机航线帧 每帧发四次
extern int first_flag[4];                     // 无人机航线 发送第一个点 （出发点（不在规划出的航线中需单独发送））
extern unsigned char track_line;           // 发送给无人机航线号 从6开始增加 / 6 <= x < 17
extern int uav_success;                    //注入成功的无人机数量
extern int plantform_id;                       // 全局无人机id 用于从飞行遥测获取无人机 id 赋值  （飞行遥测帧中编号帧不是一直发的，但为了编号不变故设置此变量）
extern int recv_Control;                   // 接收到综显控制权交接申请
extern int Control_flag[4];              // 各个无人机控制权交接标志 （便于更新发送的无人机控权状态）
extern int Pad_heart_flag;                 // 总体pad心跳标志
extern int front_pad_heart_flag;                 // 前舱pad心跳标志
extern int tail_pad_heart_flag;                 // 后舱pad心跳标志
extern int count;                          // 飞仿航线单帧要求发送四次
extern int send_area[4];              		// 无人机注入成功，空域是否应用
extern unsigned short plan_count;         // 方案收到数量
extern unsigned short plan_task_type;      // 指令方案类型
extern unsigned char zhanfa_result_receive_flag; //战术战法接收结果标志
extern unsigned char zhzl_flag;//无人机载荷指令标志
extern unsigned char fh_timeout[4];//返航超时标志
extern unsigned char hx_cx_flag;//航线查询标志
extern unsigned char BDFX_status;//编队飞行发布状态变量
extern unsigned char BDFX_double_status;//编队飞行发布状态变量
extern unsigned int ctas_calc;//DTMS解算结果
extern unsigned int task_over_cnt;//无人机发送计数
/************************************/


extern unsigned char g_image_main_channel;


/***************************** 声明定义的变量 ********************************/

/***************** 原仿真时模拟 （54所只能模拟这俩个信息用于测试） *******************/


/*********************************** 发送给无人机任务AB帧 ***********************************/
// CCC_UAV
extern renwuyaokongshuju_Azhen                       CCC_UAV_Azhens[4];             // 任务遥控数据A帧结构  原  renwuyaokongshuju_Azhens
extern renwuyaokongshuju_Bzhen                       CCC_UAV_Bzhens[4];             // 任务遥控数据B帧结构   原 renwuyaokongshuju_Bzhens
extern guangdianyaokongzhiling                       CCC_UAV_Photoelectric ;     // 光电遥控指令                 原 guangdianyaokongzhilings

//任务遥控A帧遥调指令信息表
//多模式雷达
extern duihaijingjiesearchcontrolcanshu              CCC_UAV_A_data1_0;         // 对海警戒搜索控制参数  原 duihaijingjiesearchcontrolcanshus
extern duihaigenzongjianshicontrolcanshu             CCC_UAV_A_data1_1;         // 对海跟踪监视控制参数  原 duihaigenzongjianshicontrolcanshus
extern tiaodaichengxiangcontrolcanshu                CCC_UAV_A_data1_2;         // 条带成像控制参数     原 tiaodaichengxiangcontrolcanshus
extern jushuchengxiangcontrolcanshu                  CCC_UAV_A_data1_3;         // 聚束成像控制参数     原 jushuchengxiangcontrolcanshus
extern duidisousuojianshicontrolcanshu               CCC_UAV_A_data1_4;         // 对地搜索监视控制参数  原 duidisousuojianshicontrolcanshus
extern tongshiSARGMTTcontrolcanshu                   CCC_UAV_A_data1_5;         // 同时SAR/GMTT控制参数 原 tongshiSARGMTTcontrolcanshus
extern duikongsousuojianshicontrolcanshu             CCC_UAV_A_data1_6;         // 对空搜索监视控制参数  原 duikongsousuojianshicontrolcanshus
extern jiaobamoshicontrolcanshu                      CCC_UAV_A_data1_7;         // 校靶模式控制参数     原 jiaobamoshicontrolcanshus
extern jiaozhengcontrolcanshu                        CCC_UAV_A_data1_8;         // 校正控制参数        原 jiaozhengcontrolcanshus
extern qixiangcontrolcanshu                          CCC_UAV_A_data1_9;         // 气象控制参数        原 qixiangcontrolcanshus
extern mubiaoganyucaozuo                             CCC_UAV_A_data1_10;        // 目标干预操作        原 mubiaoganyucaozuos
extern fuzhugongnengcontrolcanshu                    CCC_UAV_A_data1_11;        // 辅助功能控制参数     原 fuzhugongnengcontrolcanshus
extern workstatecontrolzhiling                       CCC_UAV_A_data1_12;        // 工作状态控制指令     原 workstatecontrolzhilings
extern jinyongpindiankongzhi                         CCC_UAV_A_data1_13;        // 禁用频点控制        原 jinyongpindiankongzhis
extern leidatongyongzhiling                          CCC_UAV_A_data1_14;        // 雷达通用指令        原  leidatongyongzhilings
extern shebeifuweicontrolzhiling                     CCC_UAV_A_data1_15;        // 设备复位控制指令     原 shebeifuweicontrolzhilings
extern jiluyishebeicontrol                           CCC_UAV_A_data1_16;        // 记录仪设备控制       原 jiluyishebeicontrols
extern sifucontrolcanshu                             CCC_UAV_A_data1_17;        // 伺服控制参数         原 sifucontrolcanshus
extern tiaoshicontrolcanshu                          CCC_UAV_A_data1_18;        // 调试控制参数         原 tiaoshicontrolcanshus
extern huifangcontrolcanshu                          CCC_UAV_A_data1_19;        // 回放控制参数         原 huifangcontrolcanshus
extern sysgongzuomoshicontrolzhiling                 CCC_UAV_A_data1_20;        // 系统工作模式控制指令   原 sysgongzuomoshicontrolzhilings
//综合任务管理系统
extern zaiheshangxiadiancontrolzhiling               CCC_UAV_A_data2_0;         // 载荷上下电控制指令     原 zaiheshangxiadiancontrolzhilings
extern missiongouxingcheckzhuangdingzhiling          CCC_UAV_A_data2_1;         // 任务构型检查装订指令    原 missiongouxingcheckzhuangdingzhilings
extern chuanganqixietongyindaozhiling                CCC_UAV_A_data2_2;         // 传感器协同引导指令     原 chuanganqixietongyindaozhilings
extern missionchongguihuaareasetzhiling              CCC_UAV_A_data2_3;         // 任务重规划区域设置指令  原 missionchongguihuaareasetzhilings
extern missionchongguihuastartzhiling                CCC_UAV_A_data2_4;         // 任务重规划启动指令     原 missionchongguihuastartzhilings
extern mubiaohangjironghezhiling                     CCC_UAV_A_data2_5;         // 目标航迹融合指令       原 mubiaohangjironghezhilings
extern shujulianfenfazhiling906                      CCC_UAV_A_data2_6;         // 906数据链分发指令     原 shujulianfenfazhiling906s
extern dataloadzhiling                               CCC_UAV_A_data2_7;         // 数据加载指令          原 dataloadzhilings
extern datarecordzhiling                             CCC_UAV_A_data2_8;         // 数据记录指令          原 datarecordzhilings
extern datasearchzhiling                             CCC_UAV_A_data2_9;         // 数据检索指令          原 datasearchzhilings
extern yewuDatayasuoxiachuanzhiling                  CCC_UAV_A_data2_10;        // 业务数据压缩下传指令    原 yewuDatayasuoxiachuanzhilings
extern datadeletezhiling                             CCC_UAV_A_data2_11;        // 数据删除指令           原 datadeletezhilings
extern softwareshengjiweihuzhiling                   CCC_UAV_A_data2_12;        // 软件升级维护指令        原 softwareshengjiweihuzhilings
extern shujulianmubiaozhishizhiling906               CCC_UAV_A_data2_13;        // 906数据链目标指示指令    原 shujulianmubiaozhishizhiling906s
extern shujulianchuanshupeizhizhiling906             CCC_UAV_A_data2_14;        // 906数据链传输配置指令    原 shujulianchuanshupeizhizhiling906s
//磁探仪/浮标
extern citancontrolzhiling                           CCC_UAV_A_data3_0;         //磁探仪控制指令            原 citancontrolzhilings
extern citanshoufangzhuangzhicontrolzhiling          CCC_UAV_A_data3_1;         // 磁探收放装置控制指令      原 citanshoufangzhuangzhicontrolzhilings
extern fubiaocontrolzhiling                          CCC_UAV_A_data3_2;         // 浮标控制指令             原 fubiaocontrolzhilings
extern fubiaostatecontrolzhiling                     CCC_UAV_A_data3_3;         // 浮标状态控制指令          原 fubiaostatecontrolzhilings

//任务遥控B帧遥调指令信息表
extern canshuqueryandloadzhiling                     CCC_UAV_B_data_0;          // 参数查询和加载指令         原 canshuqueryandloadzhilings
extern zongheshepinchonggouzhiling                   CCC_UAV_B_data_1;          // 综合射频重构指令           原 chongheshepinchonggouzhilings
extern duoxindaotongxinzhongjibodaocanshuSet         CCC_UAV_B_data_2;          // 多信道通信中继波道参数设置   原 duoxindaotongxinzhongjibodaocanshuSets
extern duoxindaotongxinzhongjishebeidizhicanshuSet   CCC_UAV_B_data_3;          // 信道通信中继设备地址设置     原 duoxindaotongxinzhongjishebeidizhicanshuSets
extern duoxindaotongxinzhongjiTODcanshuSet           CCC_UAV_B_data_4;          // 多信道通信中继TOD参数设置    原 duoxindaotongxinzhongjiTODcanshuSets
extern diwoshibieyingdagongzuocanshuSet              CCC_UAV_B_data_5;          // 敌我识别应答工作参数参数设置  原 diwoshibieyingdagongzuocanshuSets
extern hangguanyingdagongzuocanshuSet                CCC_UAV_B_data_6;          // 航管应答工作参数参数设置     原 hangguanyingdagongzuocanshuSets
extern AISgongzuocanshuSet                           CCC_UAV_B_data_7;          // AIS工作参数参数设置         原 AISgongzuocanshuSets
extern wuxiandianshengyuheightset                    CCC_UAV_B_data_8;          // 无线电高度剩余高度设置       原 wuxiandianshengyuheightsets
extern shujuliangongzuocanshu906set                  CCC_UAV_B_data_9;          // 906数据链工作参数设置        原 shujuliangongzuocanshu906sets
extern JIDScanshuset                                 CCC_UAV_B_data_10;         // JIDS参数设置               原 JIDScanshusets
extern elezhenchaworkcanshuset                       CCC_UAV_B_data_11;         // 电子侦查工作参数设置指令      原 elezhenchaworkcanshusets
extern leidazhenchaworkcanshuset                     CCC_UAV_B_data_12;         // 雷达侦察工作参数设置指令      原 leidazhenchaworkcanshusets
extern tongxinzhenchaworkcanshuset                   CCC_UAV_B_data_13;         // 通信侦察工作参数设置指令      原 tongxinzhenchaworkcanshusets
extern leidazhenchachulicanshuset                    CCC_UAV_B_data_14;         // 雷达侦查处理参数设置指令      原 leidazhenchachulicanshusets
extern tongxinzhenchachulicanshuset                  CCC_UAV_B_data_15;         // 通信侦查处理设置指令         原 tongxinzhenchachulicanshusets
extern baomicanshuset                                CCC_UAV_B_data_16;         // 保密参数设置                原 baomicanshusets
extern jiguangguanxingbujiancanshuset                CCC_UAV_B_data_17;         // 激光惯性部件控制参数设置      原 jiguangguanxingbujiancanshusets
extern jiguangguanxingbujiandaishujucanshuset        CCC_UAV_B_data_18;         // 激光惯性部件带数据参数设置    原 jiguangguanxingbujiandaishujucanshusets
extern jiguangguanxingbujiananzhuangcanshu1set       CCC_UAV_B_data_19;         // 激光惯性部件安装误差参数1设置 原 jiguangguanxingbujiananzhuangcanshu1sets
extern jiguangguanxingbujiananzhuangcanshu2set       CCC_UAV_B_data_20;         // 激光惯性部件安装误差参数2设置 原 jiguangguanxingbujiananzhuangcanshu2sets
extern guangqianguanxingbujiankongzhicanshuset       CCC_UAV_B_data_21;         // 光纤惯性部件控制参数设置     原 guangqianguanxingbujiankongzhicanshusets
extern guangqianguanxingbujiandaishujucanshuset      CCC_UAV_B_data_22;         // 光纤惯性部件带数据参数设置    原 guangqianguanxingbujiandaishujucanshusets
extern guangqianguanxingbujiananzhuangcanshu1set     CCC_UAV_B_data_23;         // 光纤惯性部件带数据参数设置    原  guangqianguanxingbujiananzhuangcanshu1sets
extern guangxianguanxingbujiananzhuangcanshu2set     CCC_UAV_B_data_24;         // 光纤惯性部件安装误差参数2设置 原 guangxianguanxingbujiananzhuangcanshu2sets
extern weihucontrolset                               CCC_UAV_B_data_25;         // 维护控制设置               原 guangxianguanxingbujiananzhuangcanshu2sets
extern Cduancekonglianset                            CCC_UAV_B_data_26;         // C段测控链工作参数设置       原 Cduancekongliansets
extern Uduancekonglianset                            CCC_UAV_B_data_27;         // U段测控链工作参数设置        原 Uduancekongliansets

/***************************************************************************************************************/


/****************************************** 协同指控计算机与综显通信  **********************************************/
// 协同指控计算机-综显   CCC-DPU
extern fangan_bianji_zhaungtai                                    CCC_DPU_data_0;       // 协同方案生成状态提示信息
extern kongzhiquan_jiaojie                                        CCC_DPU_data_1;       // 控制权交接申请        原 kongzhiquan_jiaojies
extern task_status_information                                    CCC_DPU_data_2;       // 任务状态信息发送      原 task_status_informations
extern drone_state_information                                    CCC_DPU_data_3;       // 无人机状态信息        原 drone_state_informations
extern formation_link_status_information                          CCC_DPU_data_4;       // 编队链路状态信息      原 formation_link_status_informations
extern integrated_posture                                         CCC_DPU_data_5;       // 初始化综合态势(目标融合信息) 原 integrated_postures
extern BLK_CCC_OFP_019                         blk_ccc_ofp_019;       // 任务分配结果信息      原  information_on_the_results_of_taskings
extern BLK_CCC_OFP_019                         CCC_DPU_data_6[3];       // 任务分配结果信息三方案保存
extern BLK_CCC_OFP_019                         CCC_DPU_data_6_Ofp[3];       // 任务分配结果信息三方案保存(发给ofp部分的保存，当有非紧密排列无人机时，需要调整ctas的方案后给ofp)
extern BLK_CCC_OFP_017                         blk_ccc_ofp_017;       // 任务分配结果信息      原  information_on_the_results_of_taskings
extern manned_aircraft_route_drone_route_confirmation_information CCC_DPU_data_7_9_10;  // 有人机通航路规划结果  原 manned_aircraft_route_drone_route_informations
// CCC_DPU_data_8;                                                               // 编辑中任务分配信息
// CCC_DPU_data_9       存于 CCC_DPU_data_7_8_9 主结构体中                                                                    // 浮标布阵点发送
// CCC_DPU_data_10               存于 CCC_DPU_data_7_8_9 主结构体中                                                        	     // 吊声定测点发送
extern BLK_CCC_OFP_024                                            blk_ccc_ofp_024;      // 无人机航路规划        原 drone_route_confirmations
extern BLK_CCC_OFP_025											  blk_ccc_ofp_025;		//当前阶段反馈
extern UAV_SEND                                                   uav_send[4];          // 无人机航线发送
extern UAV_ROUTE                                                  uav_route[4];        //航线信息
extern unsigned int                                               uav_change;    //无人机id队列
extern unsigned int                                               hx_change_uav_id[4];  //控制不同无人机航线切换的无人机id号
extern unsigned int                                               send_flag;              //切换指令标志位
extern unsigned int                                               global_stage;     //全局阶段数
extern BLK_CCC_OFP_032                                            blk_ccc_ofp_032;      // 协同任务执行状态提示   原 collaborative_task_execution_status_alertss
extern BLK_CCC_OFP_033                                            blk_ccc_ofp_033[2];   // 初始化任务区/空域信息  原  area_sky_informations
extern BLK_CCC_OFP_033 										      air_area;	 		    //接收CTAS的空域信息
extern BLK_CCC_OFP_034                                            blk_ccc_ofp_034;      // 任务点信息           原 taks_point_informations
extern BLK_CCC_OFP_035                                            blk_ccc_ofp_035;      // 任务线信息           原 task_line_informations
extern BLK_CCC_OFP_036                                            blk_ccc_ofp_036;      // 协同指控指令信息      原 collaborative_charge_directive_informations
extern CircularQueue                                              queue;//循环队列句柄
extern confirmation_of_preplanning_loading_results                CCC_DPU_data_17;      // 预规划加载结果确认    原 confirmation_of_preplanning_loading_resultss
extern pre_planning_program                                       CCC_DPU_data_18;      // 预规划方案           原 pre_planning_programs
extern assisted_decision_making_error_alerts                      CCC_DPU_data_19;      // 辅助决策错误提示      原 assisted_decision_making_error_alertss
extern default_orders                                             CCC_DPU_data_20;      // 协同指控系统自检测故障清单 原 default_orderss
extern guangdian_kongzhi_feedback                                 CCC_DPU_data_21;      // 无人机光电控制权反馈      原 guangdian_kongzhi_feedbacks
extern DPU_guangdina_video                                        CCC_DPU_data_22[4];      // 无人机光电视频控制反馈 原CCC_DPU_data_21 0xa2222a CCC->DPU/MMM(协同指控系统至综显和MMM)
extern DPU_guangdina_video                                        CCC_DPU_data_22_1;    // 无人机光电视频控制反馈 （仿真）
// CCC_DPU_data_23;  见 version_info.c                                            // U链端机维护信息
// CCC_DPU_data_24;  见 version_info.c                                            // 协同指控计算机维护信息
extern BLK_CCC_OFP_020                                            blk_ccc_ofp_020;       //人工修改状态信息反馈
extern BLK_CCC_OFP_021                                            blk_ccc_ofp_021;       //单无人机规划任务分配结果
extern BLK_CCC_OFP_026                                            blk_ccc_ofp_026;       //领航反馈
extern BLK_CCC_OFP_027											  blk_ccc_ofp_027;		  //有人机进入领航条件 20251113new
extern BLK_CCC_OFP_028											  blk_ccc_ofp_028;		  //有人机退出领航条件 20251113new
extern BLK_CCC_OFP_029											  blk_ccc_ofp_029;		  //无人机碰撞启动信息 20251113new
extern BLK_CCC_OFP_030											  blk_ccc_ofp_030;		  //应急返航区域 20251113new
extern BLK_CCC_OFP_047                                            blk_ccc_ofp_047;       //安全区
extern BLK_CCC_OFP_047                                            blk_ccc_ofp_047_save[3];//安全区保存
extern BLK_CCC_OFP_120                                            blk_ccc_ofp_120;       //指令微调
extern BLK_CCC_OFP_043                                            blk_dtps_dtms_043;       // 语音识别反馈
extern BLK_CCC_OFP_157											  blk_ccc_ofp_157;			//查询回报
extern BLK_CCC_OFP_158											  blk_ccc_ofp_158;			//查询航线
extern BLK_CCC_OFP_159											  blk_ccc_ofp_159;   		//规避信息
/*****************************************************************************************************************/


// 综显-协同指控计算机  DPU-CCC
extern OFP_NT_BROADCAST_MSG 											  ofp_nt_broadcast_msg; //广播轮载消息
extern manned_aircraft_target_set                                         DPU_CCC_data_0 ;      // 有人机目标集      原 manned_aircraft_target_sets
// DPU_CCC_data_1 ;     见 version_info.c                                                 // 维护控制指令（协同指控计算机）
extern BLK_OFP_CCC_014                                                    blk_ofp_ccc_014 ;      // 控制权交接指令    原 control_instruction_transfers
extern drone_autonomous_privilege_level_setting                           DPU_CCC_data_3 ;      // 无人机自主权限等级 原 drone_autonomous_privilege_level_settings
extern manned_navigation_data_information                                 DPU_CCC_data_4 ;      // 有人机导航数据信息 原  manned_navigation_data_informations
extern Information_the_results_of_tasking_confirm                         DPU_CCC_data_5 ;      // 任务分配结果信息   原   information_on_the_results_of_tasking_confirms
extern manned_aircraft_route_drone_route_confirmation_information_confirm DPU_CCC_data_6_8_9;   // 有人机通用航路    原 manned_aircraft_route_drone_route_information_conforms
extern BLK_CCC_OFP_018_CunChu                                                    confirm_hl_cunchu;    // 通用航路点存储  便于分包发送
extern global_mission_planning_commands                                   DPU_CCC_data_7 ;      // 全局任务规划命令  原 global_mission_planning_commandss
//DPU_CCC_data_8 ;     存于 DPU_CCC_data_6_8_9 主结构体中                                                                                          // 浮标布阵点
//DPU_CCC_data_9 ;     存于 DPU_CCC_data_6_8_9 主结构体中                                                                			 // 吊声定测点
extern drone_route_confirmation_confirm                                   DPU_CCC_data_10;      // 无人机航路规划      原 drone_route_confirmation_confirms
extern single_drone_charge_order                                          DPU_CCC_data_11;      // 单无人机指控命令     原 single_drone_charge_orders
extern single_task_area_allegation_order                                  DPU_CCC_data_12;      // 单任务区指控命令     原 single_task_area_allegation_orders
extern UAV_optical_handle_control_volume                                  DPU_CCC_data_13;      // 无人机光电手柄控制量  原  UAV_optical_handle_control_volumes

extern uav_photoic_video_MFD                                              DPU_CCC_data_14;      // 无人机光电视频MFD控制 MFD 原 uav_photoic_video_MFDs
extern uav_photoic_video_MFD                                              DPU_CCC_data_14_1;    // 无人机光电视频MFD控制 MFD （仿真）
extern uav_guangdiankongzhiquanshezhi                                     DPU_CCC_data_15;      // 无人机光电控制权设置   原 uav_guangdiankongzhiquanshezhis
extern BLK_PIU_CCC_006                                                    blk_piu_ccc_006;      //光电控制手柄信息
// DPU_CCC_data_16;      																 // 预规划方案
extern BLK_CCC_OFP_018                                                    blk_ofp_ccc_018;       // ofp发布的有人机航路
extern area_point_setting                                                 DPU_CCC_data_18_19;   // 任务区设置           原  area_point_settings
// DPU_CCC_data_19;   存于 DPU_CCC_data_18_19                                              // 任务点设置
extern status_alert_receipt                                               DPU_CCC_data_20;      // 状态提示回执         原  status_alert_receipts
// DPU_CCC_data_21;                                                                      // 无人机载荷控制信息
extern BLK_OFP_CCC_154                                                    blk_ofp_ccc_154;
extern BLK_OFP_CCC_037                                                    blk_ofp_ccc_037;      // 协同控制指令信息请求   原 cooper_order_finds
// DPU_CCC_data_23;    见 version_info.c                                                  // 维护控制指令（U链端机）
extern BLK_OFP_CCC_040                                                    blk_ofp_ccc_040; // 链路参数控制指令 DPU1-CCC/MMM-040
extern BLK_OFP_CCC_038                                                    blk_ofp_ccc_038;       //航线生成命令
extern BLK_OFP_CCC_302                                                    blk_ofp_ccc_302;      //浮标—航路点解算
extern BLK_OFP_CCC_402                                                    blk_ofp_ccc_402;      //吊声—吊声定测点规划
extern BLK_OFP_CCC_005                                                    blk_ofp_ccc_005;      //任务区划分信息修改
extern BLK_OFP_CCC_039                                                    blk_ofp_ccc_039;      //航线发布命令
extern BLK_OFP_CCC_041                                                    blk_ofp_ccc_041;      /*主动下一阶段提示反馈*/
extern BLK_OFP_CCC_043                                                    blk_ofp_ccc_043;      /*无人机指控任务分配结果修改及发布*/
extern BLK_OFP_CCC_045                                                    blk_ofp_ccc_045;      //无人机载荷指令
extern BLK_OFP_CCC_044                                                    blk_ofp_ccc_044;
extern BLK_OFP_CCC_047                                                    blk_dtms_dtps_047;      // 语音识别
extern BLK_OFP_CCC_053                                                    blk_ofp_ccc_053;      // 领航指令
extern BLK_OFP_CCC_148                                                    blk_ofp_ccc_148;      // 指令微调
extern BLK_OFP_CCC_155													  blk_ofp_ccc_155;		//编队能力控制 20251208new 0x62a9b
extern BLK_OFP_CCC_156													  blk_ofp_ccc_156;		//航线查询指令
/*****************************************************************************************************************/

/********************************************* 协同指控计算机与空空链通信 *********************************************/
// KKL-CCC
extern BLK_KKL_CCC_006 blk_kkl_ccc_006;         //0xaa2c06 收C链无人机状态信息
extern BLK_KKL_CCC_007 blk_kkl_ccc_007;         //0xaa2c07 收C链本机链路状态信息
// 接收载荷遥测与指令响应帧，并解析其中的遥测数据子帧2（光电数据）  KKL-CCC
extern YaoCeDataZhen       KKL_CCC_data_21[4]; // 无人机载荷遥测与指令响应数据信息（黑包） 原YaoCeDataZhen_FC_IN_0xaa1001
extern YaoCeDataZiZhen1    KKL_CCC_data_21_1[4]; // 遥测数据子帧1（光电数据）原g_YaoCeDataZiZhen1
extern YaoCeDataZiZhen2    KKL_CCC_data_21_2[4]; // 遥测数据子帧2（光电数据）原g_YaoCeDataZiZhen2
extern YaoCeDataZiZhen3    KKL_CCC_data_21_3[4]; // 遥测数据子帧3（光电数据）原g_YaoCeDataZiZhen3
extern YaoCeDataZiZhen4    KKL_CCC_data_21_4[4]; // 遥测数据子帧4 载荷在线状态
extern YaoCeDataZiZhen5    KKL_CCC_data_21_5[4]; // 遥测数据子帧5（光电数据）原g_YaoCeDataZiZhen5

extern ckpartset          CCC_KKL_data_2;    // 空空链参数设置    原 ckpartsets
extern ksend_link_controlS CCC_KKL_data_3;    // 链路交接控制指令  原 send_link_controlss
extern daohang_dataS       CCC_KKL_data_4;    // 导航数据         原 daohang_datas


// 遥控信息帧发送 每个无人机都会发
extern BLK_CCC_KKL_009_029_030_031 blk_ccc_kkl_009_029_030_031[UAV_MAX_NUM];
extern BLK_CCC_KKL_008_026_027_028 blk_ccc_kkl_008_026_027_028[UAV_MAX_NUM];           // 遥控指令帧 原yaokong_frames
extern task_control_frame task_control_frames; // 任务控制指令帧
extern track_threat_frame track_threat_frames; // 飞行与威胁帧
extern order_data_frame order_data_frames;     // 指令内容帧
extern BLK_CCC_KKL_000 blk_ccc_kkl_000;

extern BLK_SMD_CCC_000 blk_smd_ccc_000;		//无人声命令，有人机上
extern BLK_SMD_CCC_001 blk_smd_ccc_001[5];		//浮标投放信息
// KKL_CCC 空空链->协同指控计算机
/****** 用于接收无人机飞行遥测帧黑包解包  详细信息见 flying_telemetry_data.h ********/
extern s81 s81_frame;  // 81帧
extern s82 s82_frame;
extern s3A s3A_frame;
extern s3B s3B_frame;
extern s4A s4A_frame;
extern s4B s4B_frame;
extern s4C s4C_frame;
extern s4D s4D_frame;
extern s5A s5A_frame;
extern s5B s5B_frame;
extern s5C s5C_frame;
extern s5D s5D_frame;
extern s5E s5E_frame;
extern FormSubFrame1 s6_frame;
extern FormSubFrame2 s7_frame;
extern FormSubFrame3 s8_frame;
extern FormSubFrame4 s9_frame;
extern AvoidInfo s9_avoidinfo;
extern MannedLead s9_manned_lead;
extern MannedExit s9_manned_exit;
extern EMERGENCE_AREA s9_emergence_area[4];

extern int s3a_flag;//每拍接收3a帧后置位
extern int s4a_flag;//每拍接收4a帧后置位
extern int s4b_flag;//每拍接收4b帧后置位
extern int s4c_flag;//每拍接收4c帧后置位
extern int s5b_flag;//每拍接收5b帧后置位
extern int s5c_flag;//每拍接收5c帧后置位
extern int s5e_flag;//每拍接收5e帧后置位
extern int s6_bd_flag;//每次接收遥测数据子帧1编队状态
extern int s6_hold_flag;//每次接收遥测数据子帧1编队保持后置位
extern int s6_finshed_flag;//每次接收遥测数据子帧1集结完成后置位
extern int s7_hx_flag;//每次接收遥测数据子帧2航线飞行状态
extern int s8_flag;//每次接收遥测数据子帧3后置位
extern int s9_uav_avoid_flag[4];//无人机防撞启动
extern int s9_man_avoid_flag[4];//有人机防撞启动
extern int dissolve_point;
extern int start_lh_flag[4];//开始领航标志
extern int start_tclh_flag[4];//开始退出领航标志
extern int startlh_timeout; //开始领航后的节拍数
extern int starttclh_timeout;//开始退出领航后的节拍数
extern int startlh_time;
extern int starttclh_time;
extern int s7_redundancy[4];

//载荷遥测标志
extern int t1c_flag[4];//每拍接收1c帧后置位
extern int t2_flag[4];//每拍接收2帧后置位
extern int t3b2_flag[4];//每拍接收3b帧后置位
extern int t4a_flag[4] ;//每拍接收4a帧后置位
extern int t5_flag[4];//每拍接收5帧后置位
/******************************************************************************/
extern U_link_status           KKL_CCC_data_3;      // u链状态信息         原 U_link_statuss
extern kkl_canshu_feedback     KKL_CCC_data_4;      // 空空链参数反馈信息   原 kkl_canshu_feedbacks
extern kkl_status              KKL_CCC_data_5;      // kkl链路状态数据     原  kkl_statuss
extern link_control_feedback   KKL_CCC_data_7;      // 链路交接控制指令反馈  原 link_control_feedbacks
extern fly_error_order         KKL_CCC_data_20;     // 飞行故障清单        原  fly_error_orders
extern BLK_KKL_CCC_000_008_010_011 blk_kkl_ccc_000_008_010_011[UAV_MAX_NUM];

/*****************************************************************************************************************/


/********************************************* 协同指控计算机与空地链通信 *********************************************/
// CCC_kDl 协同指控计算机->空地链



// KDL_CCC 空地链->协同指控计算机
extern ShengTargetInf      KDL_CCC_data_0;  // 声目标信息黑包  ShengTargetInfs
extern CiTargetInf         KDL_CCC_data_1;  // 磁目标信息黑包  CiTargetInfs
extern ci_ty_data_pack     KDL_CCC_data_2;  // 磁通用数据黑包  ci_ty_data_packs
extern vioce_ty_data_pack  KDL_CCC_data_3;  // 声通用数据黑包  vioce_ty_data_packs
extern BLK_KDL_CCC_002     blk_kdl_ccc_002; // 控制权交接申请
extern BLK_KDL_CCC_003     blk_kdl_ccc_003; // kdl链路状态
extern BLK_KDL_CCC_015     blk_kdl_ccc_015; // 控制权交接申请应答 0xa62a0a
/*****************************************************************************************************************/



/*****************************************************内部通信**************************************************/
// DTMS_CATS 动态任务管理软件_辅助决策软件
extern BLK_DTMS_CTAS_001                blk_dtms_ctas_001 ;        // 战术战法推荐规划指令     原 zhanshutuijians
extern BLK_DTMS_CTAS_002                blk_dtms_ctas_002;         // 单任务区战术战法推荐规划指令
extern BLK_CTAS_DTMS_010                blk_ctas_dtms_010;     //空域信息
/*************************************************************************************************************/
//新增 2025.3.4
extern BLK_CCC_OFP_302 blk_ccc_ofp_302;     //浮标布阵规划
extern BLK_CCC_OFP_403 blk_ccc_ofp_403;  //吊声定测点规划
extern BLK_CCC_OFP_302 blk_ccc_ofp_302_save[3][8];     //浮标布阵规划保存
extern BLK_CCC_OFP_403 blk_ccc_ofp_403_save[3][8];  //吊声定测点规划保存
extern BLK_CCC_OFP_005 blk_ccc_ofp_005[3];  //任务区划分信息
extern BLK_CCC_OFP_006 blk_ccc_ofp_006;  /*主动下一阶段提示*/
/****************规划结果信息*********************/
extern manned_aircraft_route_drone_route_confirmation_information zhanfa_hl_plane ; // 战法方案有人机航路信息
extern BLK_CCC_OFP_024_cunchu     blk_ccc_ofp_024_cunchu[3][4] ;        // 战法方案无人机航路信息
extern BLK_CCC_OFP_024_cunchu     blk_ccc_ofp_024_single[4] ;           // 单无人机航路信息
extern BLK_CCC_OFP_018_CunChu     blk_ccc_ofp_018_cunchu[3][8] ;        // 通用航路75个点分包发送 转存结构体 辅助完善份航路点发送
/*************************************************/

/*****************************************************************************************************************/


// PAD_CCC  PAD_动态任务管理软件
extern PAD_heart PAD_hearts;     // 心跳信息
/*****************************************************************************************************************/


// 预测目标航迹信息  待后续文档文件完善修改
extern forecast_target_hj    forecast_target_hj_info ;          // 预测的航迹信息
extern threat_reg_hold_point threat_reg_hold_point_info;        // 发给协同任务规划软件信息 信息来源  pad uav
extern threat_reg_hold_point threat_reg_hold_point_info_rev;    // 接收威胁信息

/****************************** 重规划结果 中间辅助信息处理结构体  *************************/
extern manned_aircraft_route_drone_route_confirmation_information  new_hd_plane;  // 有人机重规划航路信息
extern drone_route_confirmation_confirm                            new_hd_uav;    // 无人机新航路信息



/****************************** UAV控制权交接 *************************/
/**********无人机控制权交接**********/
extern CR_ControlHandoverStatusFeedback        CCC_DPU_MMM_03;// CCC-DPU1/DPU2/MMM-003 控制权交接状态反馈
extern BLK_KKL_CCC_197						blk_kkl_ccc_197;//U链在网成员 20250725new ，与54链路协定icd
extern BLK_CCC_OFP_199                   blk_ccc_ofp_199;// CCC-DPU1/DPU2/MMM-199 U端本机链路状态数据
extern BLK_CCC_OPF_200                          CCC_DPU_MMM_200;// CCC-DPU1/DPU2/MMM-200 U端机无人机链路状态数据

/************ DPM1A交互   20241105 *************/
// drone_state_information                                         CCC_DPM_data_0;     // 无人机状态信息  DPM_drone_state_informations
// formation_link_status_information                           	 CCC_DPM_data_1;     // 编队链路状态信息   DPM_formation_link_status_informations
extern DPM_integrated_posture                                          CCC_DPM_data_2;     // 综合态势(目标融合 DPM_integrated_postures
extern DPM_Information_the_results_of_tasking                          CCC_DPM_data_3;     // 任务分配结果信息   DPM_Information_the_results_of_taskings
// manned_aircraft_route_drone_route_confirmation_information      CCC_DPM_data_4_5_6; // 有人机航路规划结果信息 DPM_manned_aircraft_route_drone_route_confirmation_informations
extern DPM_drone_route_confirmation                                    CCC_DPM_data_7;     // 无人机航路规划  DPM_drone_route_confirmations
extern DPM_collaborative_task_execution_status_alerts                  CCC_DPM_data_8;     // 协同任务执行状态提示  DPM_collaborative_task_execution_status_alertss
extern DPM_area_sky_information                                        CCC_DPM_data_9;     // 任务区/空域信息   DPM_area_sky_informations
extern DPM_taks_point_information                                      CCC_DPM_data_10;    // 任务点信息  DPM_taks_point_informations
extern DPM_task_line_information                                       CCC_DPM_data_11;    // 任务线信息   DPM_task_line_informations
extern DPM_guangdian_kongzhi_feedback                                  CCC_DPM_data_12;    // 无人机光电控制权反馈   DPM_guangdian_kongzhi_feedbacks
extern DPM_guangdina_video                                             CCC_DPM_data_13 ;   // 光电视频控制反馈    DPM_guangdina_videos
extern DPM_guangdina_video_instruction                                 DPM_CCC_data_2 ;   // 无人机光电视频控制指令  DPM_guangdina_video_instructions

/************ DLR预加载   20250315 *************/
extern BLK_DPU_DLR_011                                                 blk_dpu_dlr_011;    //启动加载命令
extern BLK_CCC_DLR_000                                                 blk_ccc_dlr_000;    //参数状态反馈
extern BLK_DLR_CCC_017                                                 blk_dlr_ccc_017;    //任务分配结果
extern BLK_DLR_CCC_018                                                 blk_dlr_ccc_018;    //有人机通用航路
extern BLK_DLR_CCC_022                                                 blk_dlr_ccc_022;    //浮标布阵点
extern BLK_DLR_CCC_023                                                 blk_dlr_ccc_023;    //吊声定测点
extern BLK_DLR_CCC_024                                                 blk_dlr_ccc_024;    //无人机航路规划
extern BLK_DLR_CCC_033                                                 blk_dlr_ccc_033;    //任务区/空域信息
extern BLK_DLR_CCC_034                                                 blk_dlr_ccc_034;    //任务点信息
extern FILE_DATA                                                       load_file;          //文件联合结构体
extern unsigned int                                                    area_load;          //任务区预加载标志
extern unsigned int                                                    point_load;         //任务点预加载标志
extern unsigned int                         dlr_load_state;         //任务点预加载标志 0:未开始；1：传输中；2：传输完毕
/************ 无人机协同照射攻击  202503126 *************/
extern BLK_DPU_CCC_011                                                 blk_dpu_ccc_011;//无人机协同照射攻击信息
extern BLK_CCC_DPU_020                                                 blk_ccc_dpu_020;//无人机协同照射控制
/*给kdl发送消息全局变量定义******************/
extern BLK_CCC_KDL_000    blk_ccc_kdl_000;// 综合态势信息数据帧 000
extern BLK_CCC_KDL_002    blk_ccc_kdl_002;// CCC-KDL/MMM-002 控制权交接申请
extern BLK_CCC_KDL_004    blk_ccc_kdl_004;// 无人机1光电视频 004
extern BLK_CCC_KDL_010    blk_ccc_kdl_010;// 控制权交接申请应答
extern BLK_CCC_KDL_012    blk_ccc_kdl_012;// 无人机2光电视频 012
extern BLK_CCC_KDL_014    blk_ccc_kdl_014;// 编队链路状态信息 014
extern BLK_CCC_KDL_017    blk_ccc_kdl_017;// 任务分配结果信息 017
extern BLK_CCC_KDL_018    blk_ccc_kdl_018;// 有人机通用航路 018
extern BLK_CCC_KDL_024    blk_ccc_kdl_024;// 无人机航路规划 024
extern BLK_CCC_KDL_033    blk_ccc_kdl_033;// 任务区/空域信息 033
extern BLK_CCC_KDL_034    blk_ccc_kdl_034;// 任务点信息 034
extern BLK_CCC_KDL_035    blk_ccc_kdl_035;// 任务线信息 035
/*******收mids时间消息*************/
extern BLK_MIDS_CCC_TIME blk_mids_ccc_time;
/************pad_ccc*******************/
extern BLK_PAD_CCC_778 blk_pad_ccc_778;
/************ccc_pad_779*******************/
extern BLK_CCC_PAD_778 blk_ccc_pad_778;


//调高调速
extern unsigned int s4D_frame_12[4];
extern unsigned int s4D_frame_0c[4];
// 控制权交接
extern unsigned int s4D_frame_04[4];
extern unsigned int s4D_frame_24[4];

// 悬停点
extern int s82_frame_30[4];
// 航线装订
extern int s4D_frame_38[4];
// 航线切换
extern int s4D_frame_40[4];
// 航线长度查询
extern int s4D_frame_41[4];
// 航线查询
extern int s4D_frame_3c[4];

extern int b2_frame_16[4];//每次接收遥测数据子帧2 16指令回复后置位
extern int b2_frame_14[4];//每次接收遥测数据子帧2 14指令回复后置位
extern int b2_tclh_frame_14[4];//每次接收遥测数据子帧2 14指令退出领航回复后置位
extern int b2_frame_30[4];//每次接收遥测数据子帧2 30指令回复后置位
extern int lhtime[4];
extern int tclhtime[4];
extern int send_cnt[4];
extern int send_cnt1[4];
extern int send_cnt2[4];

extern int temSendUavHlFlag;
extern char temSendUavHlBuffer[3000];
extern histroy_uav_data_type histroy_uav_data;
extern plane_pool plane_pool_info; // 飞机池信息

// 编队序号管理
extern FormationId formationId[UAV_MAX_NUM];
extern const int formationIdTimeOutCount;

// 航线冲突相关变量
extern ConflictResult g_lineCrashState[UAV_MAX_NUM];

// 发布指令全局变量
extern int g_recv_fabuCode[4]; // 记录收到发布的那个周期。0：未收到；1：本周期收到发布指令
extern BLK_CCC_OFP_019 g_lineCrashPlanBak[UAV_MAX_NUM];// 备份冲突时的方案（冲突任务改成盘旋）
extern BLK_CCC_OFP_024_cunchu g_lineCrashUavBak[UAV_MAX_NUM];// 备份冲突时的无人机航线
extern PayloadReplan g_payload_replan[4];

extern double g_toCurPointDis[UAV_MAX_NUM];// 到当前航点待飞距

#pragma pack()

#endif // COMMONDEFINE_H
