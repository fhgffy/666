/*
 * DDSUtil.h
 *
 *  Created on: 2024??12??10??
 *      Author: Admin
 */

#ifndef DDSUTILE_H_
#define DDSUTILE_H_

#include "Application.h"

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(1)

/*
 * DDSUtil文件说明
 * 功能：该文件是DDS的初始化工具函数，主要用于对DDS表格对应
 * 使用方法:	1.在程序最开始的时候调用initDDSTable，无需再调用DDS本身的初始化函数。该initDDSTable只需执行一次，因此不需要放进线程循环中
 * 			2.当需要新加一条接收或者发送消息时，首先在该项目的Application文件夹（DDS的文件包）下的config_topic.c文件中sg_sttaConfigTable表格中增加一条（位置不重要，最好有序）
 * 									         然后在DDSUtil.h的DDSTable中添加该条消息的DDSTableMap
 * 									         最后在DDSUtil.c的initUiTopicId函数中添加对该消息的DDSTableMap的uiTopicId赋值，
 * 									         	该uiTopicId的赋值应与sg_sttaConfigTable表格中的uiTopicId保持一
 *
 * 			3.当需要接收或发送消息时，只需要在send或receive函数的第一个函数放入对应的DDSTableMap的niConnectionId即可
 * */


//

/*
 * DDSTableMap
 * 该表格建立DDS对应关系
 * uiTopicId：对应于该项目的Application文件夹（DDS的文件包）下的config_topic.c文件中sg_sttaConfigTable表格中的uiTopicId
 * niConnectionId：对应于DDS中的sg_sttaConDirTopicId，也即是收发时候传入的第一个参数
 * index：对应于该条消息在DDS中的sg_sttaConDirTopicId中的数据
 */
typedef struct DDSTableMap{
    UINT32_SIX uiTopicId;			//DDS发送表格中sg_sttaConfigTable的uiTopicId
    SINT32_SIX niConnectionId;		//DDS连接建立中sg_sttaConDirTopicId的niConnectionId
    int index;						//对应于数组的第几位
}DDSTableMap , *TableMap;

/*
 * DDSTable
 * 该表格是DDS收发表的对应表格
 */
typedef struct DDSTable{

    //  CCC_MMM  250909
    DDSTableMap CCC_MMM_001;  /*吊声—吊声定测点规划*/
    DDSTableMap MIDS_CCC_001; // 时间消息

    //设定uiTopicId
    /****************************************** 协同指控计算机与综显通信 **********************************************/
    // 协同指控计算机-综显   CCC-DPU
    DDSTableMap CCC_DPU_0  ;// 协同方案生成状态提示信息
    DDSTableMap CCC_DPU_1  ;// 控制权交接申请
    DDSTableMap CCC_DPU_2  ; // 任务状态信息发送
    DDSTableMap CCC_DPU_3  ; // 无人机状态信息
    DDSTableMap CCC_DPU_4  ; // 编队链路状态信息
    DDSTableMap CCC_DPU_5  ; // 初始化综合态势(目标融合信息
    DDSTableMap CCC_DPU_6  ; // 任务分配结果信息
    DDSTableMap CCC_DPU_7  ; // 有人机通航路规划结果
    DDSTableMap CCC_DPU_8  ; //编辑中任务分配信息
    DDSTableMap CCC_DPU_9  ; // 浮标布阵点发送
    DDSTableMap CCC_DPU_10 ;  // 吊声定测点发送
    DDSTableMap CCC_DPU_11 ;  // 无人机航路规划-
    DDSTableMap CCC_DPU_12 ;  // 协同任务执行状态提示
    DDSTableMap CCC_DPU_13 ;  // 初始化任务区/空域信息
    DDSTableMap CCC_DPU_14 ;  // 任务点信息
    DDSTableMap CCC_DPU_15 ;  // 任务线信息
    DDSTableMap CCC_DPU_16 ;  // 协同指控指令信息
    DDSTableMap CCC_DPU_17 ;  // 预规划加载结果确认
    DDSTableMap CCC_DPU_18 ;  // 预规划方案
    DDSTableMap CCC_DPU_19 ;  // 辅助决策错误提示
    DDSTableMap CCC_DPU_20 ;  //协同指控系统自检测故障清单
    DDSTableMap CCC_DPU_21 ;  //无人机光电控制权反馈
    DDSTableMap CCC_DPU_22 ;  //无人机光电视频控制反馈
    DDSTableMap CCC_DPU_22_1 ;  //无人机光电视频控制反馈   0xa2222aa
    DDSTableMap CCC_DPU_23 ;    //U链端机维护信息
    DDSTableMap CCC_DPU_24 ;   //协同指控计算机维护信息
    DDSTableMap CCC_DPU_25 ; /*控制权交接状态反馈 20241108 added*/
    //20250108新增
    DDSTableMap CCC_DPU_26 ; /*浮标—浮标布阵规划 */
    DDSTableMap CCC_DPU_27 ; /*吊声—吊声定测点规划  */
    DDSTableMap CCC_DPU_28 ; /*当前任务阶段反馈  */
    DDSTableMap CCC_DPU_29 ; /*人工修改状态信息反馈  */
    DDSTableMap CCC_DPU_30 ; /*任务区划分信息  */
    DDSTableMap CCC_DPU_31 ; /*主动下一阶段提示  */
    DDSTableMap CCC_DPU_32 ; /*水下目标发现提示  */
    DDSTableMap CCC_DPU_33 ; /*接机参数反馈*/
    DDSTableMap CCC_DPU_34 ; /*交机参数反馈*/
    DDSTableMap CCC_DPU_35 ; /*端机飞行故障清单*/
    //20250326新增
    DDSTableMap CCC_DPU_36 ; /*无人机协同照射控制*/
    DDSTableMap CCC_DPU_37;//收U端本机链路状态数据                  0xa222c7
    DDSTableMap CCC_DPU_38;//收U端机无人机链路状态数据               0xa222c8

    DDSTableMap CCC_DPU_39;//单无人机分配结果              0xa22215
    DDSTableMap CCC_DPU_40;//应急航线预加载              0xa2222d
    DDSTableMap CCC_DPU_41;//安全区/威胁区  20250909new            0xa222b4
    DDSTableMap CCC_DPU_42;//领航反馈  20250909new             0xa2222e
    DDSTableMap CCC_DPU_43;//指令微调  20250909new            0xa22278
    DDSTableMap CCC_DPU_44 ;//有人机进入领航条件 20251113new 0xa22233
    DDSTableMap CCC_DPU_45 ;//有人机退出领航条件 20251113new 0xa22234
    DDSTableMap CCC_DPU_46 ;//无人机碰撞启动信息 20251113new 0xa22235
    DDSTableMap CCC_DPU_47 ;//应急返航区域 20251113new 0xa22236
    DDSTableMap CCC_DPU_043;// 语音控制反馈20251003            0xa2222b
    DDSTableMap CCC_DPU_157;// 查询回报
    DDSTableMap CCC_DPU_158;// 查询航线
    DDSTableMap CCC_DPU_159;// 规避信息
    DDSTableMap CCC_DPU_347;// 重规划提示
    DDSTableMap CCC_DPU_056;// PAD决策-浮标布阵规划
    DDSTableMap CCC_DPU_057;// PAD决策-吊声规划

    // 综显-协同指控计算机  DPU-CCC  (综显分为DPU1/2 通过中间件后 DTMS只需接收一个，此处使用DPU1的message_id)
    DDSTableMap DPU_CCC_NT ; //有人机轮载广播信号                0x062800  20250727new
    DDSTableMap DPU_CCC_0  ; //有人机目标集                0x062a00
    DDSTableMap DPU_CCC_1  ; //维护控制指令（协同指控计算机）  0x062a01
    DDSTableMap DPU_CCC_2  ; //控制权交接指令               0x062a0e
    DDSTableMap DPU_CCC_3  ; //无人机自主权限等级设置         0x062a0f
    DDSTableMap DPU_CCC_4  ; //有人机导航数据信息             0x062a100
    DDSTableMap DPU_CCC_5  ; //任务分配结果信息              0x062a111
    DDSTableMap DPU_CCC_6  ; //有人机通用航路               0x062a12
    DDSTableMap DPU_CCC_7  ; //全局任务规划命令             0x062a13
    DDSTableMap DPU_CCC_8  ; //浮标布阵点                  0x062a16
    DDSTableMap DPU_CCC_9  ; //吊声定测点                  0x062a17
    DDSTableMap DPU_CCC_10 ; //无人机航路规划               0x062a18
    DDSTableMap DPU_CCC_11 ; //单无人机指控命令             0x062a19
    DDSTableMap DPU_CCC_12 ; //单任务区指控命令             0x062a1a
    DDSTableMap DPU_CCC_13 ; //无人机光电手柄控制量          0x062a1b
    DDSTableMap DPU_CCC_14 ; //无人机光电视频MFD控制         0x062a1c
    DDSTableMap DPU_CCC_14_1 ; //无人机光电视频MFD控制         0x062a1cc
    DDSTableMap DPU_CCC_15 ; //无人机光电控制权设置          0x062a1d
    DDSTableMap DPU_CCC_16 ; //预规划方案加载               0x062a1f
    DDSTableMap DPU_CCC_17 ; //预规划查询                  0x062a20
    DDSTableMap DPU_CCC_18 ; //任务区设置                  0x062a21
    DDSTableMap DPU_CCC_19 ; //任务点设置                  0x062a22
    DDSTableMap DPU_CCC_20 ; //状态提示回执                0x062a23
    DDSTableMap DPU_CCC_21 ; //无人机载荷控制信息           0x062a24
    DDSTableMap DPU_CCC_22 ; //协同控制指令信息请求         0x062a25
    DDSTableMap DPU_CCC_23 ; //维护控制指令（U链端机）      0x062a65
    DDSTableMap DPU_CCC_24 ; // 链路参数控制指令 DPU1-CCC/MMM-040 0x062a28
    //20250108新增
    DDSTableMap DPU_CCC_25 ;/*任务区划分信息修改*/
    DDSTableMap DPU_CCC_26 ;/*全局任务规划航线生成命令*/
    DDSTableMap DPU_CCC_27 ;/*无人机载荷控制信息*/
    DDSTableMap DPU_CCC_28 ;/*航线生成命令*/
    DDSTableMap DPU_CCC_29 ;/*航线发布命令*/
    DDSTableMap DPU_CCC_30 ;/*主动下一阶段提示反馈*/
    DDSTableMap DPU_CCC_31 ;/*水下目标发现提示反馈*/
    DDSTableMap DPU_CCC_32 ;/*浮标—航路点解算*/
    DDSTableMap DPU_CCC_33 ;/*吊声—吊声定测点规划*/
    //20250326新增
    DDSTableMap DPU_CCC_35 ;/*无人机指控任务分配结果修改及发布*/
	//20250630新增
    DDSTableMap DPU_CCC_36 ;/*载荷控制指令*/
	DDSTableMap DPU_CCC_044 ;
	DDSTableMap DPU_CCC_047 ; // 语音识别
	DDSTableMap DPU_CCC_048 ; // 领航指令 20250909new
	DDSTableMap DPU_CCC_049 ; // 指令微调 20250909new
	DDSTableMap DPU_CCC_154 ; // 重规划反馈
	DDSTableMap DPU_CCC_155 ; // 编队能力控制
	DDSTableMap DPU_CCC_156 ; // 航线查询命令
    /****************************************************************************************************************/
	//DPU2
	DDSTableMap DPU2_CCC_NT ; //有人机轮载广播信号                0x062800  20250727new
	DDSTableMap DPU2_CCC_0  ; //有人机目标集                0x062a00
	DDSTableMap DPU2_CCC_1  ; //维护控制指令（协同指控计算机）  0x062a01
	DDSTableMap DPU2_CCC_2  ; //控制权交接指令               0x062a0e
	DDSTableMap DPU2_CCC_3  ; //无人机自主权限等级设置         0x062a0f
	DDSTableMap DPU2_CCC_4  ; //有人机导航数据信息             0x062a100
	DDSTableMap DPU2_CCC_5  ; //任务分配结果信息              0x062a111
	DDSTableMap DPU2_CCC_6  ; //有人机通用航路               0x062a12
	DDSTableMap DPU2_CCC_7  ; //全局任务规划命令             0x062a13
	DDSTableMap DPU2_CCC_8  ; //浮标布阵点                  0x062a16
	DDSTableMap DPU2_CCC_9  ; //吊声定测点                  0x062a17
	DDSTableMap DPU2_CCC_10 ; //无人机航路规划               0x062a18
	DDSTableMap DPU2_CCC_11 ; //单无人机指控命令             0x062a19
	DDSTableMap DPU2_CCC_12 ; //单任务区指控命令             0x062a1a
	DDSTableMap DPU2_CCC_13 ; //无人机光电手柄控制量          0x062a1b
	DDSTableMap DPU2_CCC_14 ; //无人机光电视频MFD控制         0x062a1c
	DDSTableMap DPU2_CCC_14_1 ; //无人机光电视频MFD控制         0x062a1cc
	DDSTableMap DPU2_CCC_15 ; //无人机光电控制权设置          0x062a1d
	DDSTableMap DPU2_CCC_16 ; //预规划方案加载               0x062a1f
	DDSTableMap DPU2_CCC_17 ; //预规划查询                  0x062a20
	DDSTableMap DPU2_CCC_18 ; //任务区设置                  0x062a21
	DDSTableMap DPU2_CCC_19 ; //任务点设置                  0x062a22
	DDSTableMap DPU2_CCC_20 ; //状态提示回执                0x062a23
	DDSTableMap DPU2_CCC_21 ; //无人机载荷控制信息           0x062a24
	DDSTableMap DPU2_CCC_22 ; //协同控制指令信息请求         0x062a25
	DDSTableMap DPU2_CCC_23 ; //维护控制指令（U链端机）      0x062a65
	DDSTableMap DPU2_CCC_24 ; // 链路参数控制指令 DPU1-CCC/MMM-040 0x062a28
	//20250108新增
	DDSTableMap DPU2_CCC_25 ;/*任务区划分信息修改*/
	DDSTableMap DPU2_CCC_26 ;/*全局任务规划航线生成命令*/
	DDSTableMap DPU2_CCC_27 ;/*无人机载荷控制信息*/
	DDSTableMap DPU2_CCC_28 ;/*航线生成命令*/
	DDSTableMap DPU2_CCC_29 ;/*航线发布命令*/
	DDSTableMap DPU2_CCC_30 ;/*主动下一阶段提示反馈*/
	DDSTableMap DPU2_CCC_31 ;/*水下目标发现提示反馈*/
	DDSTableMap DPU2_CCC_32 ;/*浮标—航路点解算*/
	DDSTableMap DPU2_CCC_33 ;/*吊声—吊声定测点规划*/
	//20250326新增
	DDSTableMap DPU2_CCC_34 ;/*无人机协同照射机攻击信息*/
	DDSTableMap DPU2_CCC_35 ;/*无人机指控任务分配结果修改及发布*/
	//20250630新增
	DDSTableMap DPU2_CCC_36 ;/*载荷控制指令*/
	DDSTableMap DPU2_CCC_044 ;
	DDSTableMap DPU2_CCC_047 ; // 语音识别
	DDSTableMap DPU2_CCC_048 ; // 领航指令 20250909new
	DDSTableMap DPU2_CCC_049 ; // 指令微调 20250909new
	DDSTableMap DPU2_CCC_154 ; // 重规划反馈
	DDSTableMap DPU2_CCC_155 ; // 编队能力控制
	DDSTableMap DPU2_CCC_156 ; // 航线查询命令
	/****************************************************************************************************************/

    /****************************************** 协同指控计算机与空地链通信 **********************************************/
    // 协同指控计算机-空地链 CCC_KDL
    DDSTableMap CCC_KDL_0   ;  //综合态势信息数据帧           0xa22400
    DDSTableMap CCC_KDL_1   ;  //控制权交接申请              0xa22402
    DDSTableMap CCC_KDL_2   ;  //无人直升机1光电电视频        0xa22404
    DDSTableMap CCC_KDL_3   ;  //无人直升机2光电电视频        0xa22404
    DDSTableMap CCC_KDL_4   ;  //编队链路状态信息            0xa2240e
    DDSTableMap CCC_KDL_5   ;  //任务分配结果信息            0xa22411
    DDSTableMap CCC_KDL_6   ;  //有人机通用航路              0xa22412
    DDSTableMap CCC_KDL_7   ;  //无人机航路规划              0xa22418
    DDSTableMap CCC_KDL_8   ;  //任务区/空域信息             0xa22421
    DDSTableMap CCC_KDL_9   ;  //任务点信息                 0xa22422
    DDSTableMap CCC_KDL_10  ;  //任务线信息                 0xa22423
	DDSTableMap CCC_KDL_11   ;  //控制权交接申请响应         0xa2240a

    // 空地链-协同指控计算机 KDL_CCC
    //KDL->CCC/PMD/SMD/MMM/PC（宽待传输C收发组合至协同指控计算机、任务显示器和MMM）
    DDSTableMap KDL_CCC_0  ; //声目标数据			0xa61000
    DDSTableMap KDL_CCC_1  ; //磁目标数据			0xa61001
    DDSTableMap KDL_CCC_2  ; //磁通用数据包			0xa61002
    DDSTableMap KDL_CCC_3  ; //声通用数据包			0xa61003

    //KDL->DPU1/DPU2/CCC/PMD/SMD/FC（宽带传输C收发组合链路状态组播）
    DDSTableMap KDL_CCC_4  ;  //空地链链路状态		0xa62003

    //KDL->CCC/MMM/FC（宽带传输C收发组合到协同指控和MMM）
    DDSTableMap KDL_CCC_5  ;  //航线确认回报及确认航线		0xa62a01
    DDSTableMap KDL_CCC_6  ;  //控制权交接申请			0xa62a02
    DDSTableMap KDL_CCC_7  ;  //视频下传设置指令		    0xa62a04
    DDSTableMap KDL_CCC_015  ;  //控制权交接申请响应			0xa62a05

    //KDL->CCC/DPU1/DPU2
    DDSTableMap KDL_CCC_8  ;  //飞行故障清单			0xa62cbe
    DDSTableMap KDL_CCC_9  ;  //维护信息			0xa62cc9

    /**************************************************************************************************************/


    /****************************************** 协同指控计算机与空空链通信 **********************************************/
    // 协同指控计算机-空空链 CCC_KKL
    DDSTableMap CCC_KKL_0   ; //无人机1遥控指令帧             0xa20e08
    DDSTableMap CCC_KKL_1   ; //无人机1编队飞行指令帧          0xa20e09
    DDSTableMap CCC_KKL_2   ; //空空链路参数控制指令           0xa20e0a
    DDSTableMap CCC_KKL_3   ; //链路交接控制指令              0xa20e0b
    DDSTableMap CCC_KKL_4   ; //导航信息                    0xa20e19
    DDSTableMap CCC_KKL_5   ; //无人机2遥控指令帧             0xa20e1a
    DDSTableMap CCC_KKL_6   ; //无人机3遥控指令帧             0xa20e1b
    DDSTableMap CCC_KKL_7   ; //无人机4遥控指令帧             0xa20e1c
    DDSTableMap CCC_KKL_8   ; //无人机2编队飞行指令帧          0xa20e1d
    DDSTableMap CCC_KKL_9   ; //无人机3编队飞行指令帧          0xa20e1e
    DDSTableMap CCC_KKL_10  ; //无人机4编队飞行指令帧          0xa20e1f
    DDSTableMap CCC_KKL_11  ; //当前无人机遥控指令侦           0xa20e08

    DDSTableMap CCC_KKL_12  ; //无人机1基本遥控信息数据帧 U链
    DDSTableMap CCC_KKL_13  ; //无人机2基本遥控信息数据帧 U链
    DDSTableMap CCC_KKL_14  ; //无人机3基本遥控信息数据帧 U链
    DDSTableMap CCC_KKL_15  ; //无人机4基本遥控信息数据帧 U链

    DDSTableMap CCC_KKL_16;  //无人机1链路遥控信息数据帧 U链
    DDSTableMap CCC_KKL_17;  //无人机1链路遥控信息数据帧 U链
    DDSTableMap CCC_KKL_18;  //无人机1链路遥控信息数据帧 U链
    DDSTableMap CCC_KKL_19;  //无人机1链路遥控信息数据帧 U链
    DDSTableMap CCC_KKL_000;  //空空链参数控制指令  0xa20e00

    // 空空链-协同指控计算机 KKL—CCC
    // KKL_CCC/PMD/SMD/MMM/FC(协同通信C收发组合至协同指控计算机、水声和MMM)
    DDSTableMap KKL_CCC_0  ; //无人机1飞行遥测信息数据帧（黑包）             0xaa1000
    DDSTableMap KKL_CCC_1  ; //无人机1载荷遥测与指令响应数据信息（黑包）       0xaa1001
    DDSTableMap KKL_CCC_2  ; //无人机1任务业务信息（黑包）                  0xaa1002
    DDSTableMap KKL_CCC_3  ; //u链状态数据                               0xaa1003
    DDSTableMap KKL_CCC_4  ; //空空链链路参数控制反馈数据                   0xaa1004
    DDSTableMap KKL_CCC_5  ; //空空链链路状态数据                         0xaa1005
    DDSTableMap KKL_CCC_6  ; //无人机2飞行遥测信息数据帧（黑包）            0xaa1008
    DDSTableMap KKL_CCC_7  ; //链路交接控制指令反馈                       0xaa1009
    DDSTableMap KKL_CCC_8  ; //无人机3飞行遥测信息数据帧（黑包）            0xaa100a
    DDSTableMap KKL_CCC_9  ; //无人机4飞行遥测信息数据帧（黑包）
    DDSTableMap KKL_CCC_10 ;  //无人机2载荷遥测与指令响应数据信息（黑包）     0xaa100c
    DDSTableMap KKL_CCC_11 ;  //无人机3载荷遥测与指令响应数据信息（黑包）     0xaa100d
    DDSTableMap KKL_CCC_12 ;  //无人机4载荷遥测与指令响应数据信息（黑包）     0xaa100e
    DDSTableMap KKL_CCC_13 ;  //无人机2任务业务信息（黑包）                0xaa100f
    DDSTableMap KKL_CCC_14 ;  //无人机3任务业务信息（黑包）                0xaa1010
    DDSTableMap KKL_CCC_15 ;  //无人机4任务业务信息（黑包）                0xaa1011
    DDSTableMap KKL_CCC_16 ;  //无人机11链路遥测信息数据帧（黑包）          0xaa1012
    DDSTableMap KKL_CCC_17 ;  //无人机21链路遥测信息数据帧（黑包）          0xaa1013
    DDSTableMap KKL_CCC_18 ;  //无人机31链路遥测信息数据帧（黑包）          0xaa1014
    DDSTableMap KKL_CCC_19 ;  //无人机41链路遥测信息数据帧（黑包）          0xaa1015
    DDSTableMap KKL_CCC_20 ;  //飞行故障清单                            0xaa10be

    //KKL（C链）-协同指控计算机
    DDSTableMap KKL_C_CCC_01;//（CR控制权交接）C链无人机状态信息        0xaa2c06
    DDSTableMap KKL_C_CCC_02;//（CR控制权交接）本机链路状态信息         0xaa2c07

    //KKL（U链）-协同指控计算机
    DDSTableMap KKL_U_CCC_01;//收U端本机链路状态数据                  0xa222c7
    DDSTableMap KKL_U_CCC_02;//收U端机无人机链路状态数据               0xa222c8

    DDSTableMap KKL_U_CCC_03;//无人机1飞行遥测信息数据帧（黑包,160字节）+编队遥控信息数据帧（160字节,编队遥控信息数据帧在加密或25.6K时没有）
    DDSTableMap KKL_U_CCC_04;//无人机2飞行遥测信息数据帧（黑包,160字节）+编队遥控信息数据帧（160字节,编队遥控信息数据帧在加密或25.6K时没有）
    DDSTableMap KKL_U_CCC_05;//无人机3飞行遥测信息数据帧（黑包,160字节）+编队遥控信息数据帧（160字节,编队遥控信息数据帧在加密或25.6K时没有）
    DDSTableMap KKL_U_CCC_06;//无人机4飞行遥测信息数据帧（黑包,160字节）+编队遥控信息数据帧（160字节,编队遥控信息数据帧在加密或25.6K时没有）
    DDSTableMap KKL_U_CCC_07;//无人机1 U链路遥测信息
    DDSTableMap KKL_U_CCC_08;//无人机2 U链路遥测信息
    DDSTableMap KKL_U_CCC_09;//无人机3 U链路遥测信息
    DDSTableMap KKL_U_CCC_10;//无人机4 U链路遥测信息
    DDSTableMap KKL_U_CCC_11;//U链在网成员 20250725new
    /**************************************************************************************************************/
    DDSTableMap SMD_CCC_000;//无人机浮标参数设置指令 0x9c5000  20250807new
    DDSTableMap SMD_CCC_001;//已投浮标信息 0x9c5001  20250807new

    /****************************************** 协同指控计算机与数据处理单元通信 **********************************************/
    // 协同指控计算机_数据处理单元 CCC_DPM
    // CCC_DPM1A/DPM5A/MMM（协同指控系统至数据处理单元和MMM）
    DDSTableMap CCC_DPM_0   ;  //无人机状态信息		0xa2260d
    DDSTableMap CCC_DPM_1   ;  //编队链路状态信息		0xa2260e
    DDSTableMap CCC_DPM_2   ;   //目标融合信息			0xa2260f
    DDSTableMap CCC_DPM_3   ;   //任务分配结果信息		0xa22611
    DDSTableMap CCC_DPM_4   ;   //有人机通用航路		0xa22612
    DDSTableMap CCC_DPM_5   ;   //浮标布阵点			0xa22616
    DDSTableMap CCC_DPM_6   ;   //吊声定测点			0xa22617
    DDSTableMap CCC_DPM_7   ;   //无人机航路规划	    0xa22618
    DDSTableMap CCC_DPM_8   ;   //协同任务执行状态提示  0xa22620
    DDSTableMap CCC_DPM_9   ;   //任务区/空域信息		0xa22621
    DDSTableMap CCC_DPM_10  ;   //任务点信息			0xa22622
    DDSTableMap CCC_DPM_11  ;   //任务线信息			0xa22623
    DDSTableMap CCC_DPM_12  ;   //无人机光电控制权反馈	0xa22629
    DDSTableMap CCC_DPM_13  ;   //无人机光电视频控制反馈	0xa2262a
    DDSTableMap CCC_DPM_14  ;  // 光电视频控制反馈 // 未找到 message_id
    // DPM_CCC
    DDSTableMap DPM_CCC_0  ;  //DPM2-A-CCC-000  声目标信息               0x6c5000
    DDSTableMap DPM_CCC_1  ;  //DPM5-A-CCC-014  无人机光电视频控制指令      0x84500e
    DDSTableMap DPM_CCC_2  ;  //DPM1-A-CCC-016  无人机光电视频控制指令      0x645010

    /*******************************************************************************************************************/


    /****************************************** 协同指控计算机其余接收 **********************************************/
    // PMD_CCC
    DDSTableMap PMD_CCC_0  ;  //PMD-CCC-000     无人机浮标参数设置指令                       0x985000
    DDSTableMap PMD_CCC_1  ;  //PMD-CCC-001     已投浮标信息                               0x985001
    DDSTableMap PMD_CCC_2  ;  //SMD-CCC-000     无人机浮标参数设置指令                       0x9c5000
    DDSTableMap PMD_CCC_3  ;  //SMD-CCC-001     已投浮标信息                               0x9c5001

    // PIU_CCC
    DDSTableMap PIU_CCC_0  ;  //PIU-CCC-006     光电搜索设备到协同指控计算机的手柄信息          0x0c5006

    // DLR_CCC
    DDSTableMap DLR_CCC_0  ;   //DLR-CCC-017       任务分配结果信息                        0x1c5011
    DDSTableMap DLR_CCC_1  ;   //DLR-CCC-018       有人机通用航路                          0x1c5012
    DDSTableMap DLR_CCC_2  ;   //DLR-CCC-022       浮标布阵点                             0x1c5016
    DDSTableMap DLR_CCC_3  ;   //DLR-CCC-023       吊声定测点                             0x1c5017
    DDSTableMap DLR_CCC_4  ;   //DLR-CCC-024       无人机航路规划                          0x1c5018
    DDSTableMap DLR_CCC_5  ;   //DLR-CCC-033       任务区/空域信息                         0x1c5021
    DDSTableMap DLR_CCC_6  ;   //DLR-CCC-034       任务点信息                             0x1c5022

    // CCC_DLR
    DDSTableMap CCC_DLR_0  ;   //CCC-DLR-000       参数加载状态反馈                     半物理0xa00e00/数字仿真0x1e0000

    // DPU_DLR
    DDSTableMap DPU_DLR_0  ;   //DPU1-DLR-011       启动加载指令      数字仿真代替DLR 0x040e0b
    /*******************************************************************************************************************/

    /************************内部通信*********************/
    //发送
    DDSTableMap BLK_DTMS_CTAS_001;//全局战术战法推荐指令
    DDSTableMap BLK_DTMS_CTAS_002;//单任务/无人机战术战法推荐指令
    DDSTableMap BLK_DTMS_CTAS_003;//任务序列修改
    DDSTableMap BLK_DTMS_CTAS_004;//任务区修改
    DDSTableMap BLK_DTMS_CTAS_005;//航线生成信息
    DDSTableMap BLK_DTMS_CTAS_006;//浮标解算信息
    DDSTableMap BLK_DTMS_CTAS_007;//吊声规划信息
    DDSTableMap BLK_DTMS_CTAS_008;//无人机航线修改信息
    DDSTableMap BLK_DTMS_CTAS_009;//有人机航线修改信息
    DDSTableMap BLK_DTMS_CTAS_010;//任务高度信息
    //接收
    DDSTableMap BLK_CTAS_DTMS_001;//战术战法规划结果
    DDSTableMap BLK_CTAS_DTMS_002;//单任务区战术战法规划结果
    DDSTableMap BLK_CTAS_DTMS_003;//有人机航路
    DDSTableMap BLK_CTAS_DTMS_004;//无人机航线
    DDSTableMap BLK_CTAS_DTMS_005;//浮标布阵规划（分阶段新增）
    DDSTableMap BLK_CTAS_DTMS_006;//吊声定测点布阵规划（分阶段新增）
    DDSTableMap BLK_CTAS_DTMS_007;//任务区划分信息（分阶段新增）
    DDSTableMap BLK_CTAS_DTMS_008;//安全区、威胁区
    DDSTableMap BLK_CTAS_DTMS_009;//解算完成反馈
    DDSTableMap BLK_CTAS_DTMS_010;//空域信息
    DDSTableMap BLK_CTAS_DTMS_011;//空域许可解算

    // 任务管理软件——动态任务规划软件
    DDSTableMap DTMS_DTRP_0; // 有人机/航线信息
    DDSTableMap DTMS_DTRP_1; // 无人机/任务航线变更
    DDSTableMap DTMS_DTRP_2; // 威胁区域信息 + 待航点位置
    // TODO 未确定来源，需补充
    DDSTableMap THREAT_INFO; // 接收威胁信息

    // 动态任务规划软件--任务管理软件
    DDSTableMap DTRP_DTMS_0;  // 有人机新航线
    DDSTableMap DTRP_DTMS_1;  // 无人机新航线

    /*数字环境心跳/版本上报/程序控制反馈********************************************/
    DDSTableMap CCC_SIMCONTROL_0; //心跳上报
    DDSTableMap CCC_SIMCONTROL_1; //控制指令反馈
    DDSTableMap CCC_SIMCONTROL_2; //控制指令获取

    /******************************CCC_PAD******************************/
    DDSTableMap CCC_PAD_001; /*方案编辑状态反馈*/
    DDSTableMap CCC_PAD_002;	/*控制权交接申请*/
    DDSTableMap CCC_PAD_003; /*控制权交接状态反馈 0xa22203  20241108*/
    DDSTableMap CCC_PAD_012;	/*有人机任务状态信息*/
    DDSTableMap CCC_PAD_013;	/*无人机状态信息*/
    DDSTableMap CCC_PAD_014;	/*编队链路状态信息*/
    DDSTableMap CCC_PAD_015;	/*目标信息*/
    DDSTableMap CCC_PAD_017;	/*任务分配结果信息 (运行方案)*/
    DDSTableMap CCC_PAD_018;	/*有人机通用航路*/
    DDSTableMap CCC_PAD_019;	/*编辑中的任务分配信息(预览方案)*/
    DDSTableMap CCC_PAD_022;	/*浮标布阵点*/
    DDSTableMap CCC_PAD_023;	/*吊声定测点*/
    DDSTableMap CCC_PAD_024;	/*无人机航路规划*/
    DDSTableMap CCC_PAD_032;	/*协同任务执行状态提示*/
    DDSTableMap CCC_PAD_033;	/*任务区/空域信息*/
    DDSTableMap CCC_PAD_034;	/*任务点信息*/
    DDSTableMap CCC_PAD_035;	/*任务线信息*/
    DDSTableMap CCC_PAD_036;	/*协同指控指令信息*/
    DDSTableMap CCC_PAD_037;	/*预规划加载结果确认*/
    DDSTableMap CCC_PAD_038;	/*预规划方案*/
    DDSTableMap CCC_PAD_039;	/*辅助决策错误提示*/
    DDSTableMap CCC_PAD_040; /*协同指控系统自检测故障清单*/
    DDSTableMap CCC_PAD_041; /*无人机光电控制权反馈*/
    DDSTableMap CCC_PAD_042; /*无人机光电视频控制反馈 20241104 新增*/


    DDSTableMap CCC_PAD_127; /*接机参数反馈 0xa2227F*/
    DDSTableMap CCC_PAD_139; /*交机参数反馈 0xa2228B*/
    DDSTableMap CCC_PAD_198; /*U端机飞行故障清单 0xa222c6*/

    DDSTableMap CCC_PAD_199;  /*本机 U链路状态数据 1103新增 */
    DDSTableMap CCC_PAD_200;  /*无人机 U链路状态数据 1103新增*/
    DDSTableMap CCC_PAD_201;  /*维护消息  U链 */
    DDSTableMap CCC_PAD_202;  /*维护消息  协同指控计算机 */

    DDSTableMap CCC_PAD_302;  /*浮标—浮标布阵规划	*/
    DDSTableMap CCC_PAD_403;  /*吊声—吊声定测点规划	*/
    DDSTableMap CCC_PAD_025;  /*当前任务阶段反馈*/
    DDSTableMap CCC_PAD_020;  /*人工修改状态信息反馈*/
    DDSTableMap CCC_PAD_005;  /*任务区划分信息  （方案生成后收到）*/
    DDSTableMap CCC_PAD_006;  /*主动下一阶段提示*/
    DDSTableMap CCC_PAD_007;  /*水下目标发现提示*/
    DDSTableMap CCC_PAD_777;  /*有人机导航数据信息(自己加的，协同指控转)*/

    /*******************************PAD_CCC*******************************/
    //DDSTableMap PAD_CCC_000;  /*块名称注释。。。*/
    DDSTableMap PAD_CCC_777;	/*心跳*/
    DDSTableMap PAD_CCC_000;	/*有人机目标集*/
    DDSTableMap PAD_CCC_014;	/*控制权交接指令*/
    DDSTableMap PAD_CCC_015;	/*无人机自主权限等级设置*/
    DDSTableMap PAD_CCC_016;	/*有人机导航数据信息*/
    DDSTableMap PAD_CCC_017;	/*任务分配结果确认信息*/
    DDSTableMap PAD_CCC_018;	/*有人机 通用航路*/
    DDSTableMap PAD_CCC_019;	/*全局任务规划命令*/
    DDSTableMap PAD_CCC_022;	/*有人机 浮标布阵点信息*/
    DDSTableMap PAD_CCC_023;	/*有人机 吊声定测点*/
    DDSTableMap PAD_CCC_024;	/*无人机航线确认信息*/
    DDSTableMap PAD_CCC_025;	/*单无人机指控命令*/
    DDSTableMap PAD_CCC_026;	/*单任务区指控命令*/
    //DDSTableMap PAD_CCC_027; /*无人机光电手柄控制量 1104根据最新协调单要求删除*/
    DDSTableMap PAD_CCC_028;	/*无人机光电视频MFD控制  1104新增*/
    DDSTableMap PAD_CCC_029;	/*无人机光电控制权设置*/
    DDSTableMap PAD_CCC_031; /*预规划方案加载*/
    DDSTableMap PAD_CCC_032;	/*预规划查询*/
    DDSTableMap PAD_CCC_033;	/*任务区域设置*/
    DDSTableMap PAD_CCC_034;	/*任务点设置*/
    DDSTableMap PAD_CCC_035;	/*状态提示回执*/
    DDSTableMap PAD_CCC_037;	/*协同指控指令查询*/

    DDSTableMap PAD_CCC_100;  /*维护控制指令   协同指控计算机*/
    DDSTableMap PAD_CCC_101;  /*维护控制指令   U链*/

    //DDSTableMap PAD_CCC_038;  /*空空链参数控制指令 1103新增  综显直接发给KKL，无需再发给CCC*/
    DDSTableMap PAD_CCC_040;  /*参数控制指令 1103新增*/


    //20250103新增
    DDSTableMap PAD_CCC_005;  /*任务区划分信息修改*/
    DDSTableMap PAD_CCC_027;  /*全局任务规划航线生成命令*/
    DDSTableMap PAD_CCC_036;  /*无人机载荷控制信息*/
    DDSTableMap PAD_CCC_038;  /*航线生成命令*/
    DDSTableMap PAD_CCC_039;  /*航线发布命令*/
    DDSTableMap PAD_CCC_041;  /*主动下一阶段提示反馈*/
    DDSTableMap PAD_CCC_042;  /*水下目标发现提示反馈*/
    DDSTableMap PAD_CCC_302;  /*浮标—航路点解算*/
    DDSTableMap PAD_CCC_402;  /*吊声—吊声定测点规划*/
    DDSTableMap PAD_CCC_778;

    // CCC_MIDS
    DDSTableMap CCC_MIDS_001;  /*将光电视频控制反馈发送给MIDS*/

}DDSTable;

extern DDSTable DDSTables;//DDS??????????

/*
 * ?????uiTopicId???DDSTables?ж??????????????и??
 */
void initUiTopicId();

//????????????uiTopicId?????DDSTables???????????niConnectionId??index
void initDDSTable();



#pragma pack()

#ifdef __cplusplus
}
#endif

#endif /* DDSUTILE_H_ */
