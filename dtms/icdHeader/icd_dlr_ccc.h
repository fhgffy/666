#ifndef ICD_DLR_CCC_H
#define ICD_DLR_CCC_H

#include "icd_ccc_ofp.h"

#pragma pack(1)

typedef struct //任务序列信息
{
    unsigned int subtask_ID_number;//任务平台子任务ID号
    unsigned short modify;//任务平台子任务是否变更
    unsigned short sequence_type;//任务平台子任务任务序列类型
    unsigned short type_of_mission_point_area;//任务平台子任务任务点/区域类型
    unsigned int target_number;//任务平台子任务任务区/点/线/目标编号
    char completion_time_valid_Bits;//任务平台子任务完成时间有效位
    char task_height_effective_position;//任务平台子任务任务高度有效位
    unsigned int task_completion_time;//任务平台子任务完成时间
    int mission_height;//任务平台子任务任务高度
    char availability_of_routes;//任务平台子任务是否存在航线
    char presence_of_buoy_array;//任务平台子任务是否存在浮标阵
    char presence_of_fixed_point_arrays;//任务平台子任务是否存在定测点阵型
#ifdef _SPARE_
    unsigned char spare[13];
#endif
}plan_info;
typedef struct //编队任务协同方案
{
    unsigned short platform_model;//任务平台型号
    unsigned short platform_serial_number;//任务平台序号
    unsigned int platform_number;//任务平台编号
    unsigned int duration_of_the_mission;//任务平台任务时长
    unsigned short number_of_subtasks;//任务平台子任务个数m
    plan_info task_sequence_informations[8];//任务序列信息（循环8次）
#ifdef _SPARE_
    unsigned char spare[66];
#endif
}subtask_info;
typedef struct
{
    unsigned int program_number;//方案编号
    unsigned short tasking_release;//任务分配发布
    unsigned short manual_modification;//任务分配方案是否人工修改
    unsigned short emphasize_planning;//任务分配方案是否重规划
    unsigned short modification_method;//任务分配方案修改方式
    unsigned short program_attributes;//方案属性
    unsigned short search_and_dive_method;//协同搜潜方式
    unsigned int total_program_time;//方案总时间
    unsigned short number_of_mission_platforms;//任务平台个数n
    subtask_info formation_synergy_mission_programs[5];//编队协同任务方案（循环5次）
}BLK_DLR_CCC_017;//任务分配结果



typedef struct
{
    unsigned short type;//航路点类型
	unsigned char validity_of_longitude;//航路点经度有效性
	unsigned char latitude_validity;//航路点维度有效性
	unsigned char height_validity;//航路点高度有效性
	unsigned char speed_validity;//航路点速度有效性
	unsigned char direction_validity;//航路点航向有效性
	unsigned char time_validity;//航路点时间有效性
	unsigned char payloads_validity;//航路点载荷有效性
	double longitude;//经度
	double latitude;//纬度
	float height;//航路点高度
	float speed;//航路点速度
	float direction;//航路点航向
	unsigned int time;//航路点时间
	unsigned short payloads;//航路点载荷
#ifdef _SPARE_
    unsigned char spare[5];
#endif
}AirWay_Point_Info;//航路点信息

typedef struct //有人机子任务
{
    unsigned int program_number;//方案编号
    unsigned short subtasks_number;//有人机子任务个数
    unsigned int subtask_ID_number;//子任务ID号
    unsigned short waypoints_number ;//航路点个数n
    unsigned short waypoint_start_number;//航路点起始编号
    AirWay_Point_Info AirWay_Point_Infos[40];//航路点信息
}BLK_DLR_CCC_018;//有人机通用航路

typedef struct //有人机子任务
{
    unsigned int   plan_code;					/*方案编号*/
	unsigned short subtask_cnt;					/*有人机子任务个数m*/
	unsigned int   subtask_index;				/*子任务ID号*/
	unsigned short task_type;					/*有人机子任务类型 20250528新增*/
	unsigned short airway_point_num;			/*航路点个数n    最小值：0  最大值：40*/
	unsigned short airway_point_start_num;		/*航路点起始编号*/
	unsigned char  plan_type;		/*航线状态 全局规划or单无人机规划    20250606 new*/
	unsigned int   uav_plan_id;			/*单无人机任务规划方案编号 航线航线状态为单无人机任务规划时有效  20250606 new*/
    AirWay_Point_Info AirWay_Point_Infos[80];//航路点信息
}BLK_DLR_CCC_018_cunchu;//有人机通用航路

typedef struct //浮标布阵点信息
{
    unsigned short matching_tube_number;//浮标布阵点匹配筒位号
    unsigned short buoy_type;//浮标布阵点浮标类型
    char longitude_validity;//浮标布阵点经度有效性
    char latitude_validity;//浮标布阵点维度有效性
    double longitude;//经度
    double latitude;//纬度
#ifdef _SPARE_
    unsigned char spare[3];
#endif
}fbinfo;

typedef struct //有人机子任务
{
    unsigned int subtask_ID_number;//子任务ID号
    unsigned short points_number;//浮标布阵点总数
    fbinfo fbinfos[25];//浮标布阵点信息
#ifdef _SPARE_
    unsigned char spare[19];
#endif
}F_SubTask_Info;

typedef struct //浮标布阵点
{
    unsigned int program_number;//方案编号
    unsigned short subtasks_number;//有人机子任务个数m
    F_SubTask_Info F_SubTask_Infos;//有人机子任务
}BLK_DLR_CCC_022;//浮标布阵点



typedef struct
{
	char JingduUpper;//吊声定测点经度有效性
	char WeiduUpper;//纬度有效性
	char HighlyUpper;//深度有效性
    double longitude;//经度
    double latitude;//纬度
    double profundity;//吊声定测点深度
#ifdef _SPARE_
    unsigned char spare[7];
#endif
}dsdc_info;//吊声定测点信息

typedef struct //有人机子任务
{
    unsigned int subtask_ID_number;//子任务ID
    unsigned short fixed_measurement_points_number;//定测点总数
    dsdc_info dsdc_infos[25];//吊声定测点信息
#ifdef _SPARE_
    unsigned char spare[124];
#endif
}D_SubTask_Info;

typedef struct //吊声定测点
{
    unsigned int program_number;//方案编号
    unsigned short subtasks_number;//有人机子任务个数m
    D_SubTask_Info D_SubTask_Infos;//有人机子任务
}BLK_DLR_CCC_023;//吊声定测点



typedef struct //航路点信息
{
    unsigned short hld_idx;				/*航路点编号		20250606 new*/
    unsigned short type;//航路点类型
    char validity_of_longitude;//航路点经度有效性
    char latitude_validity;//航路点维度有效性
    char height_validity;//航路点高度有效性
    char speed_validity;//航路点速度有效性
    char direction_validity;//航路点航向有效性
    char time_validity;//航路点时间有效性
    char payloads_validity;//航路点载荷有效性
    double longitude;//经度
    double latitude;//维度
    float height;//航路点高度
    float speed;//航路点速度
    float direction;//航路点航向
    unsigned int time;//航路点时间
    unsigned short payloads;//航路点载荷
    unsigned short causality;//航路点属性
    unsigned short standby_type;//航路点待机类型
    char standby_time_lapsNumber_cycleNumber_valid_bit;//航路点待机时间/圈数/循环次数有效位
    char standby_radius_valid_bit;//航路点待机半径有效位
    unsigned int standby_time_lapsNumber_cycleNumber;//航路点待机时间/圈数/循环次数
    unsigned int standby_radius;//航路点待机半径
#ifdef _SPARE_
    unsigned char spare[7];
#endif
}UAV_AirWay_Point_Info;

typedef struct //单无人机单序列中的规划信息
{
    unsigned int subtask_ID_number;//子任务ID号
    unsigned char  total_packet;				/*航路点总包数    20250606 new*/
    unsigned char  packet_id;					/*当前包序号		20250606 new*/
    unsigned short mission_type;//子任务任务类型
    unsigned short point_area_type;//任务点/区域类型
    unsigned int area_point_line_goal_number;//子任务任务区/点/线/目标编号
    unsigned char AtomicTimeUpper;//子任务完成时间有效位
    unsigned char AtomicHighlyUpper;//子任务高度有效位
    unsigned int completion_time;//子任务完成时间
    unsigned int mission_height;//子任务任务高度
//    unsigned char completion_time_valid_Bits;//子任务完成时间有效位
//    unsigned char ask_height_effective_position;//子任务任务高度有效位
    unsigned short waypoints_number;//航路点个数n
    UAV_AirWay_Point_Info UAV_AirWay_Point_Infos[250];//航路点信息
#ifdef _SPARE_
    unsigned char spare[76];
#endif
}SubTask_Info;
typedef struct //单个无人机航路方案
{
    unsigned short drone_serial_number;//无人机序号
    unsigned int drone_num;//无人机编号
    unsigned short subtasks_number ;//当前无人机子任务个数
    unsigned short subtask_index; // 从0开始
    SubTask_Info SubTask_Infos;//单无人机单序列中的规划信息
#ifdef _SPARE_
    unsigned char spare[72];
#endif
}UAV_SubTask_Plan;

typedef struct
{
    unsigned int program_number;//方案编号
    unsigned char  plan_type;		/*航线状态 全局规划or单无人机规划    20250606 new*/
    unsigned int   uav_plan_id;		/*无人机任务规划方案编号 航线航线状态为单无人机任务规划时有效  20250606 new*/
//    unsigned short number_of_drones;//无人机数量
    UAV_SubTask_Plan UAV_SubTask_Plans;//单个无人机航路方案
}BLK_DLR_CCC_024;//无人机航路规划


typedef struct //点坐标
{
    double longitude;//经度
    double latitude;//纬度
}Index_Info;
typedef struct
{
    double longitude;//经度
    double latitude;//纬度
    float radius;//圆心半径
}yuanxing;
typedef struct //多边形
{
    unsigned short point_number;//点数n
    Index_Info Index_Infos[10];//点坐标
#ifdef _SPARE_
    unsigned char spare[8];
#endif
}duobianxing;
typedef struct //区域信息
{
    unsigned int area_code;//区域编号1-30
    unsigned short area_type;//区域类型：1-任务区，2-空域，3-危险区
    unsigned short area_source;//区域来源0-无效，1-预规划加载，2-本机手动编辑，3-决策生成
    unsigned short area_shape;//区域形状：0-无效，1-圆形，2-多边形
    unsigned short area_platform_num;  //区域所属平台：0-无效，1-归属无人机，2-归属平台
    unsigned int drone_numbe;//空域所属无人机序号：根据编队情况1234
    unsigned char upper_height_limit_valid_bit;//区域高度上限有效位置：0-无效，1-有效
    unsigned char lower_height_limit_valid_bit;//区域高度下限有效位置：0-无效，1-有效
    float upper_height_limit;//区域高度上限：单位米1600
    float lower_height_limit;//区域高度下限：单位米1
    yuanxing cycles;//圆
    duobianxing polygonals;//多边形
#ifdef _SPARE_
    unsigned char spare[4];
#endif
}Area_Info;
typedef struct
{
    unsigned short curBagNo; /*当前包号 从0开始*/
    unsigned short bagTatal; /*总包数*/
    unsigned short area_number;//区域个数
    Area_Info Area_Infos[8];//区域信息
}BLK_DLR_CCC_033;//任务区/空域信息



typedef struct //点信息
{
    unsigned int point_number;//点编号
    unsigned short point_source_of_information;//点信息来源
    double longitude;//经度
    double latitude;//纬度
    char point_speed_effective;//点目标航速有效
    char point_direction_effective;//点目标航向有效
    char point_time_effective;//点目标时间有效
    char point_property_effective;//点目标属性有效位置
    char point_type_effective;//点目标类型有效位置
    float speed;//目标航速
    float direction;//目标航向
    unsigned short year;//年
    unsigned short month;//月
    unsigned short day;//日
    unsigned int millisecond;//毫秒
    unsigned short point_property;//目标属性
    unsigned short point_type;//目标类型
    unsigned int point_batchNumber;//批号
#ifdef _SPARE_
    unsigned char spare[5];
#endif
}Task_Point_Info;
typedef struct
{
    unsigned short point_number;//任务点个数
    Task_Point_Info Task_Point_Infos[30];//点信息
}BLK_DLR_CCC_034;//任务点信息


typedef struct
{
    unsigned short uhf1_audio_if_start_load   :1;        /*超台话音*/
    unsigned short uhf1_data_if_start_load    :1;        /*超台数据*/
    unsigned short uhf_compass_if_start_load   :1;        /*罗盘*/
    unsigned short uhf_rescue_if_start_load    :1;        /*搜救*/
    unsigned short hf_if_start_load    :1;               /*短波*/
    unsigned short jids_if_start_load  :1;               /*JIDS*/
    unsigned short data905Chain_if_start_load  :1;       /*905数据链（仅J有）*/
    unsigned short weaponChain_if_start_load  :1;        /*武协链*/

    unsigned short kuSat_if_start_load  :1;              /*卫通*/
    unsigned short audioWarnFile_if_start_load  :1;      /*音频告警文件*/
    unsigned short audioWarnTable_if_start_load :1;      /*音频告警表*/
    unsigned short bd_if_start_load  :1;                 /*北斗*/
    unsigned short wt_ip_if_start_load  :1;                 /*卫通IP*/
    unsigned short uhf3_audio_if_start_load  :1;             /*超3话（仅J有）*/
    unsigned short spare0  :2;

    unsigned short spare4 : 4;
    unsigned short xtzk_if_start_load : 1;				/*协同指控*/
    unsigned short xttx_if_start_load : 1;				/*协同通信*/
    unsigned short wayPointRoute_if_start_load  :1;      /*航点航线*/
    unsigned short esps_radar_if_start_load  :1;                  /*电子侦察-雷达库*/
    unsigned short esps_launchProgram_if_start_load  :1;          /*电子侦察-投放程序*/
    unsigned short spare2  :7;

    unsigned short spare3;
}SIG_IF_START_LOAD_DLR;  /*是否启动加载 NOTE: 0-不启动加载，1-启动加载*/

typedef struct
{
      unsigned char load_status;            /*加载启动 NOTE: 0-N/A，1-请求加载，2-取消加载*/
      SIG_IF_START_LOAD_DLR if_start_load;   /*是否启动加载 NOTE: 0-不启动加载，1-启动加载.*/
}SIG_PARA_LOAD_REQ_M1;  /*加载请求状态*/

typedef struct
{
    SIG_PARA_LOAD_REQ_M1 para_load_req_m1;
}BLK_DPU_DLR_011;/*启动参数加载请求*/


typedef struct
{
	double lon;						//经度
	double lat;						//纬度
	unsigned int height;			//高度 m
	unsigned short speed;			//速度 m/s
	unsigned char tpye;				//航路点类型 0-一般航路点 1-盘旋点 2-悬停点
	unsigned short circle_time;		//盘旋圈数/悬停时间
	double rad;						//盘旋半径
}BLK_DLR_CCC_POINT;//航点信息

typedef struct
{
	unsigned int uav_id;					//无人机id
	unsigned char normal_num;				//正常航点数量
	BLK_DLR_CCC_POINT normal_point[25];		//正常航线信息
	unsigned char emergency_num;			//应急航点数量
	BLK_DLR_CCC_POINT emergency_point[25];	//应急航线信息
}BLK_DLR_CCC_045;//航线信息

typedef struct
{
	unsigned int FBZS;	//浮标帧收
	unsigned int CTSS;	//磁探搜索
	unsigned int CTGZ;	//磁探跟踪
	unsigned int GDSS;	//光电搜索
	unsigned int GDGZ;	//光电跟踪
	unsigned int BDFX;	//编队飞行
	unsigned int GJ;	//攻击
	unsigned int PX;	//盘旋
}BLK_DTMS_CTAS_010;

typedef struct
{
    BLK_DLR_CCC_017         blk_dlr_ccc_017[2];           	//任务分配结果 						1172*2 = 2344
    BLK_DLR_CCC_018_cunchu  blk_dlr_ccc_018[2][8];        	//有人机通用航路(8个子任务)			3461*2*8 = 55376
    BLK_DLR_CCC_024         blk_dlr_ccc_024[2][2][8];     	//无人机航路规划(2架飞机，8个子任务)	14795*2*2*8 = 473440
    BLK_CCC_OFP_302			blk_dlr_cc_302[2][8];			//浮标布阵规划						1713*2*8 = 27408
    BLK_CCC_OFP_403			blk_dlr_ccc_403[2][8];			//吊声规划							656*2*8 = 10496
    BLK_DLR_CCC_033         blk_dlr_ccc_033;           		//任务区/空域信息						1670
    BLK_DLR_CCC_034         blk_dlr_ccc_034;           		//任务点信息							1562
    BLK_CCC_OFP_005			blk_dlr_ccc_005[2];				//任务区划分信息						1657*2 = 3314
    BLK_DLR_CCC_045			blk_dlr_ccc_045[4];				//航线信息							1656*4 = 6624
    BLK_DTMS_CTAS_010		blk_dtms_ctas_010[2];			//航线盘旋高度设置
    int lead_uav_id;										//长机id 0x1005 0x1006
}FILE_DATA;//文件联合结构体 AB两个方案			2344 + 55376 + 473440 + 27408 + 10496 + 1670 + 1562 + 3314 + 6624 = 582234


#pragma pack()

#endif // ICD_DLR_CCC_H




