/*
 * DDSUtil.c
 *
 *  Created on: 2024年12月10日
 *      Author: Admin
 */

#include "CommonDefine.h"
#include "DDSUtil.h"

DDSTable DDSTables;//DDS关系对应表格


/*
 *	初始化uiTopicId，将DDSTables中的每一条消息对应的uiTopicId进行赋值，所赋的值的来源为原DDS表中的uiTopicId(详见DDSUtil.h中的说明)
 */
//初始化uiTopicId
void initUiTopicId(){


    //  CCC_MMM  250909
    DDSTables.CCC_MMM_001.uiTopicId = 0x00a04800;  /**/
    DDSTables.MIDS_CCC_001.uiTopicId = 0xccccdddd;  /**/

	//设定uiTopicId
    /****************************************** 协同指控计算机与综显通信 **********************************************/
    // 协同指控计算机-综显   CCC-DPU
    DDSTables.CCC_DPU_0.uiTopicId = 0xa22201;   // 协同方案生成状态提示信息
    DDSTables.CCC_DPU_1.uiTopicId = 0xa22202;   // 控制权交接申请
    DDSTables.CCC_DPU_2.uiTopicId = 0xa2220c;    // 任务状态信息发送
    DDSTables.CCC_DPU_3.uiTopicId = 0xa2220d;    // 无人机状态信息
    DDSTables.CCC_DPU_4.uiTopicId = 0xa2220e;    // 编队链路状态信息
    DDSTables.CCC_DPU_5.uiTopicId = 0xa2220f;    // 初始化综合态势(目标融合信息)
    DDSTables.CCC_DPU_6.uiTopicId = 0xa22211;    // 任务分配结果信息
    DDSTables.CCC_DPU_7.uiTopicId = 0xa22212;    // 有人机通航路规划结果
    DDSTables.CCC_DPU_8.uiTopicId = 0xa22213;    //编辑中任务分配信息
    DDSTables.CCC_DPU_9.uiTopicId = 0xa22216;    // 浮标布阵点发送
    DDSTables.CCC_DPU_10.uiTopicId =  0xa22217;   // 吊声定测点发送
    DDSTables.CCC_DPU_11.uiTopicId =  0xa22218;   // 无人机航路规划-
    DDSTables.CCC_DPU_12.uiTopicId =  0xa22220;   // 协同任务执行状态提示
    DDSTables.CCC_DPU_13.uiTopicId =  0xa22221;   // 初始化任务区/空域信息
    DDSTables.CCC_DPU_14.uiTopicId =  0xa22222;   // 任务点信息
    DDSTables.CCC_DPU_15.uiTopicId =  0xa22223;   // 任务线信息
    DDSTables.CCC_DPU_16.uiTopicId =  0xa22224;   // 协同指控指令信息
    DDSTables.CCC_DPU_17.uiTopicId =  0xa22225;   // 预规划加载结果确认
    DDSTables.CCC_DPU_18.uiTopicId =  0xa22226;   // 预规划方案
    DDSTables.CCC_DPU_19.uiTopicId =  0xa22227;   // 辅助决策错误提示
    DDSTables.CCC_DPU_20.uiTopicId =  0xa22228;   //协同指控系统自检测故障清单
    DDSTables.CCC_DPU_21.uiTopicId =  0xa22229;   //无人机光电控制权反馈
    DDSTables.CCC_DPU_22.uiTopicId =  0xa2222a;   //无人机光电视频控制反馈 原0xa2222a
    DDSTables.CCC_DPU_22_1.uiTopicId =  0xa2222aa;   //无人机光电视频控制反馈

    // DDSTables.CCC_DPU_23.uiTopicId =  0xa2222c9;  //U链端机维护信息
    DDSTables.CCC_DPU_24.uiTopicId =  0xa222ca;  //协同指控计算机维护信息
    DDSTables.CCC_DPU_25.uiTopicId =  0xa22203;  //CCC-DPU1/DPU2/MMM-003 控制权交接状态反馈

    DDSTables.CCC_DPU_26.uiTopicId = 0xa2232e; /*浮标—浮标布阵规划 */
    DDSTables.CCC_DPU_27.uiTopicId = 0xa22393; /*吊声—吊声定测点规划  */
    DDSTables.CCC_DPU_28.uiTopicId = 0xa22204; /*当前任务阶段反馈  */
    DDSTables.CCC_DPU_29.uiTopicId = 0xa22200; /*人工修改状态信息反馈  */
    DDSTables.CCC_DPU_30.uiTopicId = 0xa22205; /*任务区划分信息  */
    DDSTables.CCC_DPU_31.uiTopicId = 0xa22206; /*主动下一阶段提示  */
    DDSTables.CCC_DPU_32.uiTopicId = 0xa22207; /*水下目标发现提示  */
    DDSTables.CCC_DPU_33.uiTopicId = 0xa2227F; /*接机参数反馈*/
    DDSTables.CCC_DPU_34.uiTopicId = 0xa2228B; /*交机参数反馈*/
    DDSTables.CCC_DPU_35.uiTopicId = 0xa222c6; /*端机飞行故障清单*/
    //20250326新增
    DDSTables.CCC_DPU_36.uiTopicId = 0x00000a;/*无人机协同照射控制*/
    DDSTables.CCC_DPU_37.uiTopicId = 0xa222c7;//收U端本机链路状态数据                  0xa222c7
    DDSTables.CCC_DPU_38.uiTopicId = 0xa222c8;//收U端机无人机链路状态数据               0xa222c8

    DDSTables.CCC_DPU_39.uiTopicId = 0xa22215;//单无人机分配结果                       0xa22215
    DDSTables.CCC_DPU_40.uiTopicId = 0xa2222d;//应急航线预加载              0xa2222d

    DDSTables.CCC_DPU_41.uiTopicId = 0xa222b4;//安全区/威胁区      20250909new        0xa2222d
    DDSTables.CCC_DPU_42.uiTopicId = 0xa2222e;//领航反馈       20250909new        0xa2222d
    DDSTables.CCC_DPU_43.uiTopicId = 0xa22278;//指令微调         20250909new       0xa2222d
    DDSTables.CCC_DPU_44.uiTopicId = 0x0a22233;//有人机进入领航条件 20251113new 0xa22233
    DDSTables.CCC_DPU_45.uiTopicId = 0x0a22234;//有人机退出领航条件 20251113new 0xa22234
    DDSTables.CCC_DPU_46.uiTopicId = 0x0a22235;//无人机碰撞启动信息 20251113new 0xa22235
    DDSTables.CCC_DPU_47.uiTopicId = 0x0a22236;//应急返航区域 20251113new 0xa22236
    DDSTables.CCC_DPU_043.uiTopicId = 0xa2222b;// 语音控制反馈20251003            0xa2222b
    DDSTables.CCC_DPU_157.uiTopicId = 0xa2229d;// 查询回报            0xa2229d
    DDSTables.CCC_DPU_158.uiTopicId = 0xa2229e;// 查询航线            0xa2229e
    DDSTables.CCC_DPU_159.uiTopicId = 0xa2229f;// 规避信息            0xa2229f
    DDSTables.CCC_DPU_347.uiTopicId = 0xa2222f;// 重规划提示
    DDSTables.CCC_DPU_056.uiTopicId = 0xa22238;// PAD决策-浮标布阵规划
    DDSTables.CCC_DPU_057.uiTopicId = 0xa22239;// PAD决策-吊声规划

    // 综显-协同指控计算机  DPU-CCC  (综显分为DPU1/2 通过中间件后 DTMS只需接收一个，此处使用DPU1的message_id)
    DDSTables.DPU_CCC_NT.uiTopicId = 0x062800; //有人机轮载广播信号      0x062800  20250727new
    DDSTables.DPU_CCC_0.uiTopicId = 0x062a00;  //有人机目标集                0x062a00
    DDSTables.DPU_CCC_1.uiTopicId = 0x062a01;  //维护控制指令（协同指控计算机）  0x062a01
    DDSTables.DPU_CCC_2.uiTopicId = 0x062a0e;  //控制权交接指令               0x062a0e
    DDSTables.DPU_CCC_3.uiTopicId = 0x062a0f;  //无人机自主权限等级设置         0x062a0f
    DDSTables.DPU_CCC_4.uiTopicId = 0x062a10;  //有人机导航数据信息             0x062a10
    DDSTables.DPU_CCC_5.uiTopicId = 0x062a11;  //任务分配结果信息              0x062a11
    DDSTables.DPU_CCC_6.uiTopicId = 0x062a12;  //有人机通用航路               0x062a12
    DDSTables.DPU_CCC_7.uiTopicId = 0x062a13;  //全局任务规划命令             0x062a13
    DDSTables.DPU_CCC_8.uiTopicId = 0x062a16;  //浮标布阵点                  0x062a16
    DDSTables.DPU_CCC_9.uiTopicId = 0x062a17;  //吊声定测点                  0x062a17
    DDSTables.DPU_CCC_10.uiTopicId = 0x062a18; //无人机航路规划               0x062a18
    DDSTables.DPU_CCC_11.uiTopicId = 0x062a19; //单无人机指控命令             0x062a19
    DDSTables.DPU_CCC_12.uiTopicId = 0x062a1a; //单任务区指控命令             0x062a1a
//    DDSTables.DPU_CCC_13.uiTopicId = 0x062a1b; //无人机光电手柄控制量          0x062a1b
    DDSTables.DPU_CCC_14.uiTopicId = 0x062a1c; //无人机光电视频MFD控制         0x062a1c
    DDSTables.DPU_CCC_14_1.uiTopicId = 0x062a1cc; //无人机光电视频MFD控制         0x062a1cc

    DDSTables.DPU_CCC_15.uiTopicId = 0x062a1d; //无人机光电控制权设置          0x062a1d
    DDSTables.DPU_CCC_16.uiTopicId = 0x062a1f; //预规划方案加载               0x062a1f
    DDSTables.DPU_CCC_17.uiTopicId = 0x062a20; //预规划查询                  0x062a20
    DDSTables.DPU_CCC_18.uiTopicId = 0x062a21; //任务区设置                  0x062a21
    DDSTables.DPU_CCC_19.uiTopicId = 0x062a22; //任务点设置                  0x062a22
    DDSTables.DPU_CCC_20.uiTopicId = 0x062a23; //状态提示回执                0x062a23
    DDSTables.DPU_CCC_21.uiTopicId = 0x062a24; //无人机载荷控制信息           0x062a24
    DDSTables.DPU_CCC_22.uiTopicId = 0x062a25; //协同控制指令信息请求         0x062a25
    DDSTables.DPU_CCC_23.uiTopicId = 0x062a65; //维护控制指令（U链端机）      0x062a65
    DDSTables.DPU_CCC_24.uiTopicId = 0x062a28; // 链路参数控制指令 DPU1-CCC/MMM-040 0x062a28

    DDSTables.DPU_CCC_25.uiTopicId = 0x062a05;/*任务区划分信息修改*/
    DDSTables.DPU_CCC_26.uiTopicId = 0x062a1b;/*全局任务规划航线生成命令*/
    DDSTables.DPU_CCC_28.uiTopicId = 0x062a26;/*航线生成命令*/
    DDSTables.DPU_CCC_29.uiTopicId = 0x062a27;/*航线发布命令*/
    DDSTables.DPU_CCC_30.uiTopicId = 0x062a29;/*主动下一阶段提示反馈*/
    DDSTables.DPU_CCC_31.uiTopicId = 0x062a2a;/*水下目标发现提示反馈*/
    DDSTables.DPU_CCC_32.uiTopicId = 0x062b2e;/*浮标—航路点解算*/
    DDSTables.DPU_CCC_33.uiTopicId = 0x062b92;/*吊声—吊声定测点规划*/

    //20250326新增
    DDSTables.DPU_CCC_35.uiTopicId = 0x062a2b;/*无人机指控任务分配结果修改及发布*/
	//20250630新增
    DDSTables.DPU_CCC_36.uiTopicId = 0x062a2d;/*载荷控制指令*/
	DDSTables.DPU_CCC_044.uiTopicId = 0x062a2c;//手动控制权
	DDSTables.DPU_CCC_047.uiTopicId = 0x062a2f;//语音识别
	DDSTables.DPU_CCC_048.uiTopicId = 0x062a35; // 领航指令 20250909new
	DDSTables.DPU_CCC_049.uiTopicId = 0x062a94; // 指令微调 20250909new
	DDSTables.DPU_CCC_154.uiTopicId = 0x062a36;// 重规划反馈
	DDSTables.DPU_CCC_155.uiTopicId = 0x062a9b;// 编队能力控制
	DDSTables.DPU_CCC_156.uiTopicId = 0x062a9c;// 航线查询命令
    /****************************************************************************************************************/
	//DPU2
	DDSTables.DPU2_CCC_NT.uiTopicId = 0x0a2800; //有人机轮载广播信号      0x062800  20250727new
	DDSTables.DPU2_CCC_0.uiTopicId = 0x0a2a00;  //有人机目标集                0x062a00
	DDSTables.DPU2_CCC_1.uiTopicId = 0x0a2a01;  //维护控制指令（协同指控计算机）  0x062a01
	DDSTables.DPU2_CCC_2.uiTopicId = 0x0a2a0e;  //控制权交接指令               0x062a0e
	DDSTables.DPU2_CCC_3.uiTopicId = 0x0a2a0f;  //无人机自主权限等级设置         0x062a0f
	DDSTables.DPU2_CCC_4.uiTopicId = 0x0a2a10;  //有人机导航数据信息             0x062a10
	DDSTables.DPU2_CCC_5.uiTopicId = 0x0a2a11;  //任务分配结果信息              0x062a11
	DDSTables.DPU2_CCC_6.uiTopicId = 0x0a2a12;  //有人机通用航路               0x062a12
	DDSTables.DPU2_CCC_7.uiTopicId = 0x0a2a13;  //全局任务规划命令             0x062a13
	DDSTables.DPU2_CCC_8.uiTopicId = 0x0a2a16;  //浮标布阵点                  0x062a16
	DDSTables.DPU2_CCC_9.uiTopicId = 0x0a2a17;  //吊声定测点                  0x062a17
	DDSTables.DPU2_CCC_10.uiTopicId = 0x0a2a18; //无人机航路规划               0x062a18
	DDSTables.DPU2_CCC_11.uiTopicId = 0x0a2a19; //单无人机指控命令             0x062a19
	DDSTables.DPU2_CCC_12.uiTopicId = 0x0a2a1a; //单任务区指控命令             0x062a1a
	DDSTables.DPU2_CCC_14.uiTopicId = 0x0a2a1c; //无人机光电视频MFD控制         0x062a1c
	DDSTables.DPU2_CCC_14_1.uiTopicId = 0x0a2a1cc; //无人机光电视频MFD控制         0x062a1cc

	DDSTables.DPU2_CCC_15.uiTopicId = 0x0a2a1d; //无人机光电控制权设置          0x062a1d
	DDSTables.DPU2_CCC_16.uiTopicId = 0x0a2a1f; //预规划方案加载               0x062a1f
	DDSTables.DPU2_CCC_17.uiTopicId = 0x0a2a20; //预规划查询                  0x062a20
	DDSTables.DPU2_CCC_18.uiTopicId = 0x0a2a21; //任务区设置                  0x062a21
	DDSTables.DPU2_CCC_19.uiTopicId = 0x0a2a22; //任务点设置                  0x062a22
	DDSTables.DPU2_CCC_20.uiTopicId = 0x0a2a23; //状态提示回执                0x062a23
	DDSTables.DPU2_CCC_21.uiTopicId = 0x0a2a24; //无人机载荷控制信息           0x062a24
	DDSTables.DPU2_CCC_22.uiTopicId = 0x0a2a25; //协同控制指令信息请求         0x062a25
	DDSTables.DPU2_CCC_23.uiTopicId = 0x0a2a65; //维护控制指令（U链端机）      0x062a65
	DDSTables.DPU2_CCC_24.uiTopicId = 0x0a2a28; // 链路参数控制指令 DPU1-CCC/MMM-040 0x062a28

	DDSTables.DPU2_CCC_25.uiTopicId = 0x0a2a05;/*任务区划分信息修改*/
	DDSTables.DPU2_CCC_26.uiTopicId = 0x0a2a1b;/*全局任务规划航线生成命令*/
	DDSTables.DPU2_CCC_27.uiTopicId = 0x0a2a24;/*无人机载荷控制信息*/
	DDSTables.DPU2_CCC_28.uiTopicId = 0x0a2a26;/*航线生成命令*/
	DDSTables.DPU2_CCC_29.uiTopicId = 0x0a2a27;/*航线发布命令*/
	DDSTables.DPU2_CCC_30.uiTopicId = 0x0a2a29;/*主动下一阶段提示反馈*/
	DDSTables.DPU2_CCC_31.uiTopicId = 0x0a2a2a;/*水下目标发现提示反馈*/
	DDSTables.DPU2_CCC_32.uiTopicId = 0x0a2b2e;/*浮标—航路点解算*/
	DDSTables.DPU2_CCC_33.uiTopicId = 0x0a2b92;/*吊声—吊声定测点规划*/

	//20250326新增
	DDSTables.DPU2_CCC_35.uiTopicId = 0x0a2a2b;/*无人机指控任务分配结果修改及发布*/
	//20250630新增
	DDSTables.DPU2_CCC_36.uiTopicId = 0x0a2a2d;/*载荷控制指令*/
	DDSTables.DPU2_CCC_044.uiTopicId = 0x0a2a2c;//手动控制权
	DDSTables.DPU2_CCC_047.uiTopicId = 0x0a2a2f;//语音识别
	DDSTables.DPU2_CCC_048.uiTopicId = 0x0a2a35;; // 领航指令 20250909new
	DDSTables.DPU2_CCC_049.uiTopicId = 0x0a2a94;; // 指令微调 20250909new
	DDSTables.DPU2_CCC_154.uiTopicId = 0x0a2a36;// 重规划反馈
	DDSTables.DPU2_CCC_155.uiTopicId = 0x0a2a9b;// 编队能力控制
	DDSTables.DPU2_CCC_156.uiTopicId = 0x0a2a9c;// 航线查询命令
	/****************************************************************************************************************/


    /****************************************** 协同指控计算机与空地链通信 **********************************************/
    // 协同指控计算机-空地链 CCC_KDL
    DDSTables.CCC_KDL_0.uiTopicId  = 0xa22400;   //综合态势信息数据帧           0xa22400
    DDSTables.CCC_KDL_1.uiTopicId  = 0xa22402;   //控制权交接申请              0xa22402
    DDSTables.CCC_KDL_2.uiTopicId  = 0xa22404;   //无人直升机1光电电视频        0xa22404
    DDSTables.CCC_KDL_3.uiTopicId  = 0xa22404;   //无人直升机2光电电视频        0xa22404
    DDSTables.CCC_KDL_4.uiTopicId  = 0xa2240e;   //编队链路状态信息            0xa2240e
    DDSTables.CCC_KDL_5.uiTopicId  = 0xa22411;   //任务分配结果信息            0xa22411
    DDSTables.CCC_KDL_6.uiTopicId  = 0xa22412;   //有人机通用航路              0xa22412
    DDSTables.CCC_KDL_7.uiTopicId  = 0xa22418;   //无人机航路规划              0xa22418
    DDSTables.CCC_KDL_8.uiTopicId  = 0xa22421;   //任务区/空域信息             0xa22421
    DDSTables.CCC_KDL_9.uiTopicId  = 0xa22422;   //任务点信息                 0xa22422
    DDSTables.CCC_KDL_10.uiTopicId  = 0xa22423;  //任务线信息                 0xa22423
    DDSTables.CCC_KDL_11.uiTopicId  = 0xa2240a;  //控制权交接申请反馈          0xa2240a
	
    // 空地链-协同指控计算机 KDL_CCC
    //KDL->CCC/PMD/SMD/MMM/PC（宽待传输C收发组合至协同指控计算机、任务显示器和MMM）
    DDSTables.KDL_CCC_0.uiTopicId = 0xa61000;  //声目标数据			0xa61000
    DDSTables.KDL_CCC_1.uiTopicId = 0xa61001;  //磁目标数据			0xa61001
    DDSTables.KDL_CCC_2.uiTopicId = 0xa61002;  //磁通用数据包			0xa61002
    DDSTables.KDL_CCC_3.uiTopicId = 0xa61003;  //声通用数据包			0xa61003

    //KDL->DPU1/DPU2/CCC/PMD/SMD/FC（宽带传输C收发组合链路状态组播）
    DDSTables.KDL_CCC_4.uiTopicId = 0xa62003;  //空地链链路状态		0xa62003

    //KDL->CCC/MMM/FC（宽带传输C收发组合到协同指控和MMM）
    DDSTables.KDL_CCC_5.uiTopicId = 0xa62a01;  //航线确认回报及确认航线		0xa62a01
    DDSTables.KDL_CCC_6.uiTopicId = 0xa62a02;  //控制权交接申请			0xa62a02
    DDSTables.KDL_CCC_7.uiTopicId = 0xa62a04;  //视频下传设置指令		    0xa62a04
    DDSTables.KDL_CCC_015.uiTopicId = 0xa62a05;  //控制权交接申请应答			0xa62a05

    //KDL->CCC/DPU1/DPU2
    DDSTables.KDL_CCC_8.uiTopicId = 0xa62cbe;  //飞行故障清单			0xa62cbe
    DDSTables.KDL_CCC_9.uiTopicId = 0xa62cc9;  //维护信息			0xa62cc9

    /**************************************************************************************************************/


    /****************************************** 协同指控计算机与空空链通信 **********************************************/
    // 协同指控计算机-空空链 CCC_KKL
    DDSTables.CCC_KKL_0.uiTopicId  = 0xa20e08;  //无人机1遥控指令帧             0xa20e08
#if _PC_SIMULATION_
    // 仿真使用
    DDSTables.CCC_KKL_1.uiTopicId  = 0X0a7b1001;  //无人机1编队飞行指令帧
    DDSTables.CCC_KKL_8.uiTopicId  = 0X0a7b1002;  //无人机2编队飞行指令帧          0xa20e1d
    DDSTables.CCC_KKL_9.uiTopicId  = 0X0a7b1003;  //无人机3编队飞行指令帧          0xa20e1e
    DDSTables.CCC_KKL_10.uiTopicId  = 0X0a7b1004; //无人机4编队飞行指令帧          0xa20e1f
    DDSTables.CCC_KKL_11.uiTopicId = 0X0000CB0D;//当前无人机遥控指令侦           0xa20e08
#else
    DDSTables.CCC_KKL_1.uiTopicId  = 0xa20e09;  //无人机1编队飞行指令帧          0xa20e09
    DDSTables.CCC_KKL_8.uiTopicId  = 0xa20e1d;  //无人机2编队飞行指令帧          0xa20e1d
    DDSTables.CCC_KKL_9.uiTopicId  = 0xa20e1e;  //无人机3编队飞行指令帧          0xa20e1e
    DDSTables.CCC_KKL_10.uiTopicId  = 0xa20e1f; //无人机4编队飞行指令帧          0xa20e1f
#endif

    DDSTables.CCC_KKL_2.uiTopicId  = 0xa20e0a;  //空空链路参数控制指令           0xa20e0a
    DDSTables.CCC_KKL_3.uiTopicId  = 0xa20e0b;  //链路交接控制指令              0xa20e0b
    DDSTables.CCC_KKL_4.uiTopicId  = 0xa20e19;  //导航信息                    0xa20e19
    DDSTables.CCC_KKL_5.uiTopicId  = 0xa20e1a;  //无人机2遥控指令帧             0xa20e1a
    DDSTables.CCC_KKL_6.uiTopicId  = 0xa20e1b;  //无人机3遥控指令帧             0xa20e1b
    DDSTables.CCC_KKL_7.uiTopicId  = 0xa20e1c;  //无人机4遥控指令帧             0xa20e1c


    DDSTables.CCC_KKL_12.uiTopicId  = 0X0000CB02; //无人机1基本遥控信息数据帧 U链
    DDSTables.CCC_KKL_13.uiTopicId  = 0X0000CB03; //无人机2基本遥控信息数据帧 U链
    DDSTables.CCC_KKL_14.uiTopicId  = 0X0000CB04; //无人机3基本遥控信息数据帧 U链
    DDSTables.CCC_KKL_15.uiTopicId  = 0X0000CB05; //无人机4基本遥控信息数据帧 U链

    DDSTables.CCC_KKL_16.uiTopicId  = 0X0000CB06;  //无人机1链路遥控信息数据帧 U链
    DDSTables.CCC_KKL_17.uiTopicId  = 0X0000CB07;  //无人机1链路遥控信息数据帧 U链
    DDSTables.CCC_KKL_18.uiTopicId  = 0X0000CB08;  //无人机1链路遥控信息数据帧 U链
    DDSTables.CCC_KKL_19.uiTopicId  = 0X0000CB09;  //无人机1链路遥控信息数据帧 U链
    DDSTables.CCC_KKL_000.uiTopicId  = 0xa20e00;  //空空链参数控制指令  0xa20e00

    /*******************************************空空链-协同指控计算机 KKL—CCC*********************************************/
#if _PC_SIMULATION_
    // 仿真使用
    DDSTables.KKL_CCC_0.uiTopicId = 0X0a7e1001;   //无人机1飞行遥测信息数据帧（黑包）
    DDSTables.KKL_CCC_6.uiTopicId = 0X0a7e1002;   //无人机2飞行遥测信息数据帧（黑包）            0xaa1008/
    DDSTables.KKL_CCC_8.uiTopicId = 0X0a7e1003;   //无人机3飞行遥测信息数据帧（黑包）            0xaa100a
    DDSTables.KKL_CCC_9.uiTopicId = 0X0a7e1004;   //无人机4飞行遥测信息数据帧（黑包）
#else
    // KKL_CCC/PMD/SMD/MMM/FC(协同通信C收发组合至协同指控计算机、水声和MMM)
    DDSTables.KKL_CCC_0.uiTopicId = 0xaa1000;   //无人机1飞行遥测信息数据帧（黑包）             0xaa1000
    DDSTables.KKL_CCC_6.uiTopicId = 0xaa1008;   //无人机2飞行遥测信息数据帧（黑包）
    DDSTables.KKL_CCC_8.uiTopicId = 0xaa100a;   //无人机3飞行遥测信息数据帧（黑包）            0xaa100a
    DDSTables.KKL_CCC_9.uiTopicId = 0xaa100b;   //无人机4飞行遥测信息数据帧（黑包）            0xaa100b
#endif

    DDSTables.KKL_CCC_1.uiTopicId = 0xaa1001;   //无人机1载荷遥测与指令响应数据信息（黑包）       0xaa1001
    DDSTables.KKL_CCC_2.uiTopicId = 0xaa1002;   //无人机1任务业务信息（黑包）                  0xaa1002
    DDSTables.KKL_CCC_3.uiTopicId = 0xaa1003;   //u链状态数据                               0xaa1003
    DDSTables.KKL_CCC_4.uiTopicId = 0xaa1004;   //空空链链路参数控制反馈数据                   0xaa1004
    DDSTables.KKL_CCC_5.uiTopicId = 0xaa1005;   //空空链链路状态数据                         0xaa1005
    DDSTables.KKL_CCC_7.uiTopicId = 0xaa1009;   //链路交接控制指令反馈                       0xaa1009

    DDSTables.KKL_CCC_10.uiTopicId = 0xaa100c;  //无人机2载荷遥测与指令响应数据信息（黑包）     0xaa100c
    DDSTables.KKL_CCC_11.uiTopicId = 0xaa100d;  //无人机3载荷遥测与指令响应数据信息（黑包）     0xaa100d
    DDSTables.KKL_CCC_12.uiTopicId = 0xaa100e;  //无人机4载荷遥测与指令响应数据信息（黑包）     0xaa100e
    DDSTables.KKL_CCC_13.uiTopicId = 0xaa100f;  //无人机2任务业务信息（黑包）                0xaa100f
    DDSTables.KKL_CCC_14.uiTopicId = 0xaa1010;  //无人机3任务业务信息（黑包）                0xaa1010
    DDSTables.KKL_CCC_15.uiTopicId = 0xaa1011;  //无人机4任务业务信息（黑包）                0xaa1011
    DDSTables.KKL_CCC_16.uiTopicId = 0xaa1012;  //无人机11链路遥测信息数据帧（黑包）          0xaa1012
    DDSTables.KKL_CCC_17.uiTopicId = 0xaa1013;  //无人机21链路遥测信息数据帧（黑包）          0xaa1013
    DDSTables.KKL_CCC_18.uiTopicId = 0xaa1014;  //无人机31链路遥测信息数据帧（黑包）          0xaa1014
    DDSTables.KKL_CCC_19.uiTopicId = 0xaa1015;  //无人机41链路遥测信息数据帧（黑包）          0xaa1015
    DDSTables.KKL_CCC_20.uiTopicId = 0xaa10be;  //飞行故障清单                            0xaa10be
    //KKL（C链）-协同指控计算机
	DDSTables.KKL_C_CCC_01.uiTopicId = 0xaa2c06;//（CR控制权交接）C链无人机状态信息        0xaa2c06
    DDSTables.KKL_C_CCC_02.uiTopicId = 0xaa2c07;//（CR控制权交接）本机链路状态信息         0xaa2c07

    //KKL（U链）-协同指控计算机
    DDSTables.KKL_U_CCC_01.uiTopicId = 0X0000CA0C;//收U端本机链路状态数据                  0xa222c7
    DDSTables.KKL_U_CCC_02.uiTopicId = 0X0000CA0D;//收U端机无人机链路状态数据               0xa222c8
    DDSTables.KKL_U_CCC_03.uiTopicId = 0X0000CA01;//无人机1飞行遥测信息数据帧（黑包,160字节）+编队遥控信息数据帧（160字节,编队遥控信息数据帧在加密或25.6K时没有）
    DDSTables.KKL_U_CCC_04.uiTopicId = 0X0000CA02;//无人机2飞行遥测信息数据帧（黑包,160字节）+编队遥控信息数据帧（160字节,编队遥控信息数据帧在加密或25.6K时没有）
    DDSTables.KKL_U_CCC_05.uiTopicId = 0X0000CA03;//无人机3飞行遥测信息数据帧（黑包,160字节）+编队遥控信息数据帧（160字节,编队遥控信息数据帧在加密或25.6K时没有）
    DDSTables.KKL_U_CCC_06.uiTopicId = 0X0000CA04;//无人机4飞行遥测信息数据帧（黑包,160字节）+编队遥控信息数据帧（160字节,编队遥控信息数据帧在加密或25.6K时没有）
    DDSTables.KKL_U_CCC_07.uiTopicId = 0X0000CA05;//无人机1 U链路遥测信息
    DDSTables.KKL_U_CCC_08.uiTopicId = 0X0000CA06;//无人机1 U链路遥测信息
    DDSTables.KKL_U_CCC_09.uiTopicId = 0X0000CA07;//无人机1 U链路遥测信息
    DDSTables.KKL_U_CCC_10.uiTopicId = 0X0000CA08;//无人机1 U链路遥测信息
    DDSTables.KKL_U_CCC_11.uiTopicId = 0X0000CA0E;//U链在网成员 20250725new 0xa22210
    /**************************************************************************************************************/

    DDSTables.SMD_CCC_000.uiTopicId = 0x9c5000;//无人机浮标参数设置指令 0x9c5000  20250807new
    DDSTables.SMD_CCC_001.uiTopicId = 0x9c5001;//已投浮标信息 0x9c5001  20250807new
    /****************************************** 协同指控计算机与数据处理单元通信 **********************************************/
    // 协同指控计算机_数据处理单元 CCC_DPM
    // CCC_DPM1A/DPM5A/MMM（协同指控系统至数据处理单元和MMM）
    DDSTables.CCC_DPM_0.uiTopicId  = 0xa2260d;  //无人机状态信息		0xa2260d
    DDSTables.CCC_DPM_1.uiTopicId  = 0xa2260e;  //编队链路状态信息		0xa2260e
    DDSTables.CCC_DPM_2.uiTopicId  = 0xa2260f;   //目标融合信息			0xa2260f
    DDSTables.CCC_DPM_3.uiTopicId  = 0xa22611;   //任务分配结果信息		0xa22611
    DDSTables.CCC_DPM_4.uiTopicId  = 0xa22612;   //有人机通用航路		0xa22612
    DDSTables.CCC_DPM_5.uiTopicId  = 0xa22616;   //浮标布阵点			0xa22616
    DDSTables.CCC_DPM_6.uiTopicId  = 0xa22617;   //吊声定测点			0xa22617
    DDSTables.CCC_DPM_7.uiTopicId  = 0xa22618;   //无人机航路规划	    0xa22618
    DDSTables.CCC_DPM_8.uiTopicId  = 0xa22620;   //协同任务执行状态提示  0xa22620
    DDSTables.CCC_DPM_9.uiTopicId  = 0xa22621;   //任务区/空域信息		0xa22621
    DDSTables.CCC_DPM_10.uiTopicId  = 0xa22622;  //任务点信息			0xa22622
    DDSTables.CCC_DPM_11.uiTopicId  = 0xa22623;  //任务线信息			0xa22623
    DDSTables.CCC_DPM_12.uiTopicId  = 0xa22629;  //无人机光电控制权反馈	0xa22629
    DDSTables.CCC_DPM_13.uiTopicId  = 0xa2262a;  //无人机光电视频控制反馈	0xa2262a
    DDSTables.CCC_DPM_14.uiTopicId  = 0Xef200f; // 光电视频控制反馈 // 未找到 message_id
    // DPM_CCC
    DDSTables.DPM_CCC_0.uiTopicId = 0x6c5000;  //DPM2-A-CCC-000  声目标信息               0x6c5000
    DDSTables.DPM_CCC_1.uiTopicId = 0x84500e;  //DPM5-A-CCC-014  无人机光电视频控制指令      0x84500e
    DDSTables.DPM_CCC_2.uiTopicId = 0x645010;  //DPM1-A-CCC-016  无人机光电视频控制指令      0x645010

    /*******************************************************************************************************************/


    /****************************************** 协同指控计算机其余接收 **********************************************/
    // PMD_CCC
    DDSTables.PMD_CCC_0.uiTopicId = 0x985000;  //PMD-CCC-000     无人机浮标参数设置指令                       0x985000
    DDSTables.PMD_CCC_1.uiTopicId = 0x985001;  //PMD-CCC-001     已投浮标信息                               0x985001
    DDSTables.PMD_CCC_2.uiTopicId = 0x9c5000;  //SMD-CCC-000     无人机浮标参数设置指令                       0x9c5000
    DDSTables.PMD_CCC_3.uiTopicId = 0x9c5001;  //SMD-CCC-001     已投浮标信息                               0x9c5001

    // PIU_CCC
    DDSTables.PIU_CCC_0.uiTopicId = 0x0c5000;  //PIU-CCC-006     光电搜索设备到协同指控计算机的手柄信息          0x0c5006

    // DLR_CCC
    DDSTables.DLR_CCC_0.uiTopicId = 0x1c5011;   //DLR-CCC-017       任务分配结果信息                        0x1c5011
    DDSTables.DLR_CCC_1.uiTopicId = 0x1c5012;   //DLR-CCC-018       有人机通用航路                          0x1c5012
    DDSTables.DLR_CCC_2.uiTopicId = 0x1c5016;   //DLR-CCC-022       浮标布阵点                             0x1c5016
    DDSTables.DLR_CCC_3.uiTopicId = 0x1c5017;   //DLR-CCC-023       吊声定测点                             0x1c5017
    DDSTables.DLR_CCC_4.uiTopicId = 0x1c5018;   //DLR-CCC-024       无人机航路规划                          0x1c5018
    DDSTables.DLR_CCC_5.uiTopicId = 0x1c5021;   //DLR-CCC-033       任务区/空域信息                         0x1c5021
    DDSTables.DLR_CCC_6.uiTopicId = 0x1c5022;   //DLR-CCC-034       任务点信息                             0x1c5022

    /*******************************************************************************************************************/
#if _PC_SIMULATION_
    // CCC_DLR
    DDSTables.CCC_DLR_0.uiTopicId = 0x1e0000;   //CCC-DLR-000       参数加载状态反馈      半物理 0xa00e00/数字仿真代替DLR 0x1e0000
    // DPU_DLR
    DDSTables.DPU_DLR_0.uiTopicId = 0x040e0b;   //DPU1-DLR-011       启动加载指令      数字仿真代替DLR 0x040e0b
#else
    // CCC_DLR
    DDSTables.CCC_DLR_0.uiTopicId = 0xa00e00;   //CCC-DLR-000       参数加载状态反馈      半物理 0xa00e00/数字仿真 0x1e0000

#endif

 
    /************************内部通信*********************/
    //发送
    DDSTables.BLK_DTMS_CTAS_001.uiTopicId = 0x10000000;//战术战法推荐指令
    DDSTables.BLK_DTMS_CTAS_002.uiTopicId = 0x10000001;//战术战法推荐指令
    DDSTables.BLK_DTMS_CTAS_003.uiTopicId = 0x10000111;//任务序列修改
    DDSTables.BLK_DTMS_CTAS_004.uiTopicId = 0x10000116;//任务区修改
    DDSTables.BLK_DTMS_CTAS_005.uiTopicId = 0x10000118;//航线生成信息
    DDSTables.BLK_DTMS_CTAS_006.uiTopicId = 0x10000119;//浮标解算信息
    DDSTables.BLK_DTMS_CTAS_007.uiTopicId = 0x1000011a;//吊声规划信息
    DDSTables.BLK_DTMS_CTAS_008.uiTopicId = 0x1000011d;//无人机航线修改信息
    DDSTables.BLK_DTMS_CTAS_009.uiTopicId = 0x1000011e;//有人机航线修改信息
    DDSTables.BLK_DTMS_CTAS_010.uiTopicId = 0x10000120;//任务高度信息
    //接收
    DDSTables.BLK_CTAS_DTMS_001.uiTopicId = 0x10000112;//战术战法规划结果
    DDSTables.BLK_CTAS_DTMS_002.uiTopicId = 0X10000101;//单任务区战术战法规划结果
    DDSTables.BLK_CTAS_DTMS_003.uiTopicId = 0x10000222;//有人机航路
    DDSTables.BLK_CTAS_DTMS_004.uiTopicId = 0x10000225;//无人机航线
    DDSTables.BLK_CTAS_DTMS_005.uiTopicId = 0X10000113;//浮标布阵规划（分阶段新增）
    DDSTables.BLK_CTAS_DTMS_006.uiTopicId = 0X10000114;//吊声定测点布阵规划（分阶段新增）
    DDSTables.BLK_CTAS_DTMS_007.uiTopicId = 0X10000115;//任务区划分信息（分阶段新增）
    DDSTables.BLK_CTAS_DTMS_008.uiTopicId = 0X10000117;//安全区、威胁区
    DDSTables.BLK_CTAS_DTMS_009.uiTopicId = 0X1000011b;//解算完成反馈
    DDSTables.BLK_CTAS_DTMS_010.uiTopicId = 0X1000011c;//空域信息
    DDSTables.BLK_CTAS_DTMS_011.uiTopicId = 0X1000011f;//空域许可解算

    // 任务管理软件——动态任务规划软件
    DDSTables.DTMS_DTRP_0.uiTopicId = 0x10001112; // 有人机/航线信息
    DDSTables.DTMS_DTRP_1.uiTopicId = 0x1000011C; // 无人机/任务航线变更
    DDSTables.DTMS_DTRP_2.uiTopicId = 0X1000011A; // 威胁区域信息 + 待航点位置
    // TODO 未确定来源，需补充
    DDSTables.THREAT_INFO.uiTopicId = 0X1000011B;  // 接收威胁信息

    // 动态任务规划软件--任务管理软件
    DDSTables.DTRP_DTMS_0.uiTopicId = 0X10000118;  // 有人机新航线
    DDSTables.DTRP_DTMS_1.uiTopicId = 0X10000119;  // 无人机新航线

    /*数字环境心跳/版本上报/程序控制反馈********************************************/
    DDSTables.CCC_SIMCONTROL_0.uiTopicId = 0x00000200; //心跳上报
    DDSTables.CCC_SIMCONTROL_1.uiTopicId = 0x00000201; //控制指令反馈
    DDSTables.CCC_SIMCONTROL_2.uiTopicId = 0x00000100;  //控制指令获取

    /******************************CCC_PAD******************************/
    DDSTables.CCC_PAD_001.uiTopicId = 0xdddd0001; /*方案编辑状态反馈*/
    DDSTables.CCC_PAD_002.uiTopicId = 0xdddd0002;	/*控制权交接申请*/
    DDSTables.CCC_PAD_003.uiTopicId = 0xdddd0003; /*控制权交接状态反馈 0xa22203  20241108*/
    DDSTables.CCC_PAD_012.uiTopicId = 0xdddd0012;	/*有人机任务状态信息*/
    DDSTables.CCC_PAD_013.uiTopicId = 0xdddd0013;	/*无人机状态信息*/
    DDSTables.CCC_PAD_014.uiTopicId = 0xdddd0014;	/*编队链路状态信息*/
    DDSTables.CCC_PAD_015.uiTopicId = 0xdddd0015;	/*目标信息*/
    DDSTables.CCC_PAD_017.uiTopicId = 0xdddd0017;	/*任务分配结果信息 (运行方案)*/
    DDSTables.CCC_PAD_018.uiTopicId = 0xdddd0018;	/*有人机通用航路*/
    DDSTables.CCC_PAD_019.uiTopicId = 0xdddd0019;	/*编辑中的任务分配信息(预览方案)*/
    DDSTables.CCC_PAD_022.uiTopicId = 0xdddd0022;	/*浮标布阵点*/
    DDSTables.CCC_PAD_023.uiTopicId = 0xdddd0023;	/*吊声定测点*/
    DDSTables.CCC_PAD_024.uiTopicId = 0xdddd0024;	/*无人机航路规划*/
    DDSTables.CCC_PAD_032.uiTopicId = 0xdddd0032;	/*协同任务执行状态提示*/
    DDSTables.CCC_PAD_033.uiTopicId = 0xdddd0033;	/*任务区/空域信息*/
    DDSTables.CCC_PAD_034.uiTopicId = 0xdddd0034;	/*任务点信息*/
    DDSTables.CCC_PAD_035.uiTopicId = 0xdddd0035;	/*任务线信息*/
    DDSTables.CCC_PAD_036.uiTopicId = 0xdddd0036;	/*协同指控指令信息*/
    DDSTables.CCC_PAD_037.uiTopicId = 0xdddd0037;	/*预规划加载结果确认*/
    DDSTables.CCC_PAD_038.uiTopicId = 0xdddd0038;	/*预规划方案*/
    DDSTables.CCC_PAD_039.uiTopicId = 0xdddd0039;	/*辅助决策错误提示*/
    DDSTables.CCC_PAD_040.uiTopicId = 0xdddd0040; /*协同指控系统自检测故障清单*/
    DDSTables.CCC_PAD_041.uiTopicId = 0xdddd0041; /*无人机光电控制权反馈*/
    DDSTables.CCC_PAD_042.uiTopicId = 0xdddd0042; /*无人机光电视频控制反馈 20241104 新增*/
    DDSTables.CCC_PAD_127.uiTopicId = 0xdddd0127; /*接机参数反馈 0xa2227F*/
    DDSTables.CCC_PAD_139.uiTopicId = 0xdddd0139; /*交机参数反馈 0xa2228B*/
    DDSTables.CCC_PAD_198.uiTopicId = 0xdddd0198; /*U端机飞行故障清单 0xa222c6*/
    DDSTables.CCC_PAD_199.uiTopicId = 0xdddd0199;  /*本机 U链路状态数据 1103新增 */
    DDSTables.CCC_PAD_200.uiTopicId = 0xdddd0200;  /*无人机 U链路状态数据 1103新增*/
    DDSTables.CCC_PAD_201.uiTopicId = 0xdddd0201;  /*维护消息  U链 */
    DDSTables.CCC_PAD_202.uiTopicId = 0xdddd0202;  /*维护消息  协同指控计算机 */
    DDSTables.CCC_PAD_302.uiTopicId = 0xdddd0302;  /*浮标—浮标布阵规划	*/
    DDSTables.CCC_PAD_403.uiTopicId = 0xdddd0403;  /*吊声—吊声定测点规划	*/
    DDSTables.CCC_PAD_025.uiTopicId = 0xdddd0025;  /*当前任务阶段反馈*/
    DDSTables.CCC_PAD_020.uiTopicId = 0xdddd0020;  /*人工修改状态信息反馈*/
    DDSTables.CCC_PAD_005.uiTopicId = 0xdddd0005;  /*任务区划分信息  （方案生成后收到）*/
    DDSTables.CCC_PAD_006.uiTopicId = 0xdddd0006;  /*主动下一阶段提示*/
    DDSTables.CCC_PAD_007.uiTopicId = 0xdddd0007;  /*水下目标发现提示*/
    DDSTables.CCC_PAD_777.uiTopicId = 0xdddd0777;  /*有人机导航数据信息(自己加的，协同指控转)*/


    /*******************************PAD_CCC*******************************/
    //DDSTables.PAD_CCC_000.uiTopicId = 0xeeee0;  /*块名称注释。。。*/
    DDSTables.PAD_CCC_777.uiTopicId = 0xeeee0777;	/*心跳*/
    DDSTables.PAD_CCC_000.uiTopicId = 0xeeee0000;	/*有人机目标集*/
    DDSTables.PAD_CCC_014.uiTopicId = 0xeeee0014;	/*控制权交接指令*/
    DDSTables.PAD_CCC_015.uiTopicId = 0xeeee0015;	/*无人机自主权限等级设置*/
    DDSTables.PAD_CCC_016.uiTopicId = 0xeeee0016;	/*有人机导航数据信息*/
    DDSTables.PAD_CCC_017.uiTopicId = 0xeeee0017;	/*任务分配结果确认信息*/
    DDSTables.PAD_CCC_018.uiTopicId = 0xeeee0018;	/*有人机 通用航路*/
    DDSTables.PAD_CCC_019.uiTopicId = 0xeeee0019;	/*全局任务规划命令*/
    DDSTables.PAD_CCC_022.uiTopicId = 0xeeee0022;	/*有人机 浮标布阵点信息*/
    DDSTables.PAD_CCC_023.uiTopicId = 0xeeee0023;	/*有人机 吊声定测点*/
    DDSTables.PAD_CCC_024.uiTopicId = 0xeeee0024;	/*无人机航线确认信息*/
    DDSTables.PAD_CCC_025.uiTopicId = 0xeeee0025;	/*单无人机指控命令*/
    DDSTables.PAD_CCC_026.uiTopicId = 0xeeee0026;	/*单任务区指控命令*/
    //DDSTables.PAD_CCC_027.uiTopicId = 0xeeee0; /*无人机光电手柄控制量 1104根据最新协调单要求删除*/
    DDSTables.PAD_CCC_028.uiTopicId = 0xeeee0028;	/*无人机光电视频MFD控制  1104新增*/
    DDSTables.PAD_CCC_029.uiTopicId = 0xeeee0029;	/*无人机光电控制权设置*/
    DDSTables.PAD_CCC_031.uiTopicId = 0xeeee0031; /*预规划方案加载*/
    DDSTables.PAD_CCC_032.uiTopicId = 0xeeee0032;	/*预规划查询*/
    DDSTables.PAD_CCC_033.uiTopicId = 0xeeee0033;	/*任务区域设置*/
    DDSTables.PAD_CCC_034.uiTopicId = 0xeeee0034;	/*任务点设置*/
    DDSTables.PAD_CCC_035.uiTopicId = 0xeeee0035;	/*状态提示回执*/
    DDSTables.PAD_CCC_037.uiTopicId = 0xeeee0037;	/*协同指控指令查询*/
    DDSTables.PAD_CCC_100.uiTopicId = 0xeeee0100;  /*维护控制指令   协同指控计算机*/
    DDSTables.PAD_CCC_101.uiTopicId = 0xeeee0101;  /*维护控制指令   U链*/
    //DDSTables.PAD_CCC_038.uiTopicId = 0xeeee0;  /*空空链参数控制指令 1103新增  综显直接发给KKL，无需再发给CCC*/
    DDSTables.PAD_CCC_040.uiTopicId = 0xeeee0040;  /*参数控制指令 1103新增*/
    //20250103新增
    DDSTables.PAD_CCC_005.uiTopicId = 0xeeee0005;  /*任务区划分信息修改*/
    DDSTables.PAD_CCC_027.uiTopicId = 0xeeee0027;  /*全局任务规划航线生成命令*/
    DDSTables.PAD_CCC_036.uiTopicId = 0xeeee0036;  /*无人机载荷控制信息*/
    DDSTables.PAD_CCC_038.uiTopicId = 0xcdef0038;  /*航线生成命令*/
    DDSTables.PAD_CCC_039.uiTopicId = 0xeeee0039;  /*航线发布命令*/
    DDSTables.PAD_CCC_041.uiTopicId = 0xeeee0041;  /*主动下一阶段提示反馈*/
    DDSTables.PAD_CCC_042.uiTopicId = 0xeeee0042;  /*水下目标发现提示反馈*/
    DDSTables.PAD_CCC_302.uiTopicId = 0xeeee0302;  /*浮标—航路点解算*/
    DDSTables.PAD_CCC_402.uiTopicId = 0xeeee0402;  /*吊声—吊声定测点规划*/
    DDSTables.PAD_CCC_778.uiTopicId = 0xeeee0778;  /*吊声—吊声定测点规划*/


    // CCC_MIDS
    DDSTables.CCC_MIDS_001.uiTopicId = 0xffff0001;  /*将光电视频控制反馈发送给MIDS*/

}


//根据uiTopicId初始化DDSTable，包括初始化niConnectionId和index
void initDDSTable(){
	//初始化DDS
	ApplicationInit();
	//获取当前配置表中的消息个数
    int TableMapNum = sizeof(DDSTables) / sizeof(DDSTableMap);
    initUiTopicId();
	//给配置表赋值
    DDSTableMap tableMap;//Map，指向当前的位置
    char *DDSTablesPoint;
    DDSTablesPoint = &DDSTables;
	int i , j;
    for(i = 0; i < TableMapNum; i++){
		//当前位置为初始位置加上前面Map的个数
        memcpy(&tableMap ,DDSTablesPoint + i * sizeof(DDSTableMap), sizeof(DDSTableMap));
		//根据uiTopicId找出对应的sg_sttaConDirTopicId中的消息条目，并将niConnectionId赋值
        for(j = 0;j < sizeof(sg_sttaConDirTopicId) / sizeof(SConnectionDirectionTopicId);j++){
            if(tableMap.uiTopicId == sg_sttaConDirTopicId[j].uiTopicId){
                tableMap.niConnectionId = sg_sttaConDirTopicId[j].niConnectionId;
                tableMap.index = j;
                //将赋过的值重新赋值回原位置
                memcpy(DDSTablesPoint + i * sizeof(DDSTableMap),&tableMap, sizeof(DDSTableMap));
                break;
			}
		}

	}

}
