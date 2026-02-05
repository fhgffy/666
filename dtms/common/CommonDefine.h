#ifndef COMMONDEFINE_H
#define COMMONDEFINE_H

#pragma pack(1)





#include "icd_ofp_ccc.h"
#include "icd_ccc_ofp.h"
#include "geoGrid.h"




#define   SIMTESTFLAG   0
// 数字环境宏定义
#define _PC_SIMULATION_  0 //是否仿真
// 无人机数量
#define   UAV_MAX_NUM       4

#define   RECV_MAX_SIZE   4096

#define  VALID_LAT_LON 	1 //屏蔽实验室异常经纬度
#if 0
//实验室
#define	GCS_ID		0x2001//地面站ID
#define UAV1_ID		0x1005//无人机1 ID
#define UAV2_ID		0x1006//无人机2 ID
#else
//机上
#define	GCS_ID		0x2001//地面站ID
#define UAV1_ID		0x1005//无人机1 ID
#define UAV2_ID		0x1006//无人机2 ID
#endif

#define MANNED_ID 	0x9001//有人机ID
#define HEIGHT		160		//机场高度 160m 默认加
#define LON_A		108.772425//1005机场经度
#define LAT_A		19.143772//1005机场纬度
#define LON_A_UAV2  108.772173    // 1006机机场经度20260201
#define LAT_A_UAV2  19.143703     // 1006机机场纬度20260201

//塔哈空域中心
#define	CENTER_LAT 47.533611
#define	CENTER_LON 124.2644
//TODO 4架无人机？

//设备定义
#define CCC		0x01//协同指控计算机
#define KKL		0x02//空空链，协同通信C收发组合
#define KDL		0x03//空地链，宽带传输C收发组合
#define DPU1	        0x04//综显1
#define DPU2	        0x05//综显2
#define DPM		0x06//数据处理单元，1A、5A
#define MMM		0x07//大容量存储设备
#define PAD           0X08 // 协同显控终端


/****************************** U链 *************************/
typedef struct histroy_uav_data_type
{
	unsigned int his_uav_height;//高度
	double his_uav_relative_azmuith;//相对于本机的方位角
}histroy_uav_data_type;

/***********************************************************/

/************************防撞结构体定义*************************/
typedef struct plane{  // 飞机信息
	unsigned short plane_id; // 飞机编号
	unsigned short vehicle_type; // 飞机型号 区分无/有人机 1—有人机 2-无人机
	int replan_flag ; // 是否改航标志 0：未改航 1：改航
	waypoint_information_confirm waypoint_infos[75];  // 航点信息
}plane;

typedef struct plane_pool{  // 存储所有飞机
	int plane_num; // 飞机数量
	plane planes[10];
}plane_pool;



typedef struct threat_info{  // 威胁信息
	unsigned short plane_id; // 飞机编号
	unsigned short vehicle_type; // 飞机型号 区分无/有人机 1—有人机 2-无人机
	int isthreat; // 是否存在威胁 0 ：不威胁 1 ：威胁
	vector_fake threat_hd_index; // 存在威胁的航点
}threat_info;
/***********************************************************/



#define PORT 5555    //发送端口号1915
#define IP LocalHost //发送IP地址
#define INTEGRATED_POSTURES_DELAY 200   //综合态势信息间隔200毫秒发送一次
#define DRONE_STATE_INFORMATION_DELAY 200   //无人机状态信息间隔
#define AREA_SKY_INFORMATION_DELAY 200 //任务区/空域信息
#define HEADSIZE 6  //接收的头大小

//遥控注入
#define lon_scale ((double)180 / (((long long )2 << 30)-1))  // 经度比例
#define lat_scale ((double)90 / (((long long )2 << 30)-1))   // 纬度比例
//#define lon_scale M_PI/(2^31 - 1)  // 经度比例
//#define lat_scale M_PI/(2^32 - 1)   // 纬度比例
#define height_scale ((double)12000 / (((long long )2 << 14)-1))   // 高度比例
#define angle_scale ((double)360 / (((long long )2 << 15)-1))   // 角度比例
#define speed_scale 0.1
#define gaodu_scale 0.2


/********** 相关信息分辨率宏定义 *********/
// kdl
#define lon_Scale ((double)180 / (((long long )2 , 63)-1))       // 经度比例
#define lat_Scale ((double)90 / (((long long )2 , 63)-1))        // 纬度比例
// 编队空指令
#define lon_Scale1 (0x8000000000000000L / (double)180)           // 经度比例
#define lat_Scale1 (0x8000000000000000L / (double)90)            // 纬度比例

#define lon_scale2 ((0x80000000L - 1) / 180.0)                        // 经度比例
#define lat_scale2 ((0x80000000L - 1) / 90.0)                         // 纬度比例
#define hxjiao_scale ((0x8000L-1) / 180.0)                           //航向角比例
#define speed_scale_b ((0x8000L-1) / 400.0)                            //地速比例
#define wugao_scale ((0x8000L-1)/1000.0)                             //无高比例
#define zhilinghangx_scale ((0x8000L-1) / 360.0)                     //指令航向比例
#define phjsd_scale  ((0x8000L-1) / 400.0)                           //偏航角速率比例
#define zxsdcmd_scale  ((0x8000L-1) / 100.0)                         //纵向速率比例
#define qxwzcmd_scale  ((0x8000L-1) / 3000.0)                        //前向位置比例
#define hgjcmd_scale  ((0x8000L-1) / 50.0)                        //横滚角指令位置比例
#define gdcmd_scale  ((0x8000L-1) / 8000.0)                        //高度指令比例
#define cscmd_scale  ((0x8000L-1) / 30.0)                        //垂速指令比例
#define PI 3.14159265358979                                      // 常量 派
#define MAX_QUEUE   30                                           //队列长度
/************************************/

typedef struct {

    char dataA[4096];
    int size;
}MyByteArray;

typedef struct
{
    int count;          //当前元素数量
    op_code buffer[MAX_QUEUE];   //数据存储区
}CircularQueue;


typedef struct
{
    int planeId;// 无人机id，为0则说明没有绑定（首次入队则完成绑定，绑定后下电前一直保持）
    int count;// 超时计数（计数周期自增（收到会清空，用来判断超时，超时则代表出编队））
    int isControl;//是否有控权，0，无，1，有
    int station_address;//控制站id 20250804new
    int C_U;//从链路接收的数据是C还是U 1-C，2-U 20250804new
    double lat;//纬度
    double lon;//经度
}FormationId;// 编队id信息结构体





/*******************航线冲突相关***********************/

// 定义航线
typedef struct {
    Geo start;  // 起始点
    Geo end;    // 结束点
} FlightRoute;

// 定义冲突检测结果
typedef struct {
    int hasConflict;   // 是否有冲突 1-有冲突，0-无冲突
    Grid conflictPoint;  // 冲突点（北东坐标系）
    Geo conflictGeo;  // 冲突点（经纬度）
    Geo avoidancePoint;  // 规避点（经纬度）
    int ofpLineCode;// 指控的提示冲突重规划后ofp的回复。0：默认。1：同意重规划，2：不同意重规划（发布原航线）。
    int ofpLIneCodeCount;//指控的提示冲突重规划后ofp的回复的次数。（主要记录第一次的回复，因为需要备份原航线，如果发布不成功，多次恢复，每次都备份，原始航线会变成规避盘旋点）。
    int recv_fabuCode;// 冲突时，标识当前的发布状态；0：表示未收到发布或发布处理完成；1：表示冲突时第一次发布规避航线（盘旋点，主要是发布后阶段数不增加）；2：表示冲突时第二次发布规避航线（冲突后正常执行任务航线）；
    int crashOffTime; // 控制提示消解次数（保证消解提示不一直提示）
    int crashOffConfirm; // 指控提示消解后ofp的回复。1：进入发布阶段（只用一次，标识发布的那个阶段）
    int PayloadReplan;	//载荷重规划状态 0-NA 1-确认进入重规划 2-正在进行重规划
} ConflictResult;

//载荷重规划相关变量
typedef struct
{
	unsigned char 	CT_Online;		//磁探在线状态 0-不在线 1-在线
	unsigned char 	CT_Status;		//磁探杆伸出收回状态 0-收回 1-伸出
	unsigned char  	Replan;			//重规划状态 0-NA 1-触发重规划  2-运行时冲突
}PayloadReplan;






#pragma pack()

#endif /* COMMONDEFINE_H_ */
