#ifndef KDL_ICD_H
#define KDL_ICD_H

#define WORD unsigned short
#define BYTE char
#define DWORD int

#pragma pack(1)

//经纬度
typedef struct cocolongitude_and_latitude
{
    double longitude;//经度
    double latitude;//维度

}colongitude_and_latitude;
typedef struct copoint_coordinate//点坐标
{
    colongitude_and_latitude point_colongitude_and_latitude;//点经纬度
}copoint_coordinate;








/***************************** 发送给kdl **********************************/

/*BLK_CCC_KDL_000 start******************/

//1.3 综合态势信息数据帧 000
typedef struct cogoal_information//目标信息
{
    unsigned short goal_num;//目标序号
    char goal_type;//目标类型
    // char goal_source;//目标来源
    long long goal_longitude;
    long long goal_latitude;
    //    colongitude_and_latitude goal_colongitude_and_latitude;//目标经纬度
    int goal_height;//目标高度
    int goal_speed;//目标速度
    int goal_direction;//目标航向
    char goal_attributes;//目标属性
    long long goal_lot_num;//目标批号
    char data_valid_flag_bit;//目标数据有效位
    char standby;//备用
}cogoal_information;

typedef struct data_fusion//数据融合
{
    unsigned short goal_number;//目标数目
    cogoal_information goal_informations;//目标信息
}data_fusion;

typedef struct kunmanned_helicopter_flight_information//无人直升机1飞行信息
{
    //    float ground_control_station_longitude;//地面控制站经度
    //    float ground_control_station_latitude;//地面控制站维度
    unsigned short uav_id;// 飞机id
    char backup[6]; // 备份
    unsigned char control_attribution;//控制权归属 1-地面 2-有人机
    int unmanned_helicopter_longitude;//无人直升机经度
    int unmanned_helicopter_latitude;//无人直升机维度
    short barometric_height;//气压高度
    short satellite_height;//卫星高度
    short radio_height;//无线电高度
    short absolute_height;//绝对高度
    short relative_height;//相对高度
    char vertical_velocity;//垂直速度
    short vacuum_speed;//真空速
    short body_axis_longitudinal_speed;//机体轴纵向速度
    short body_axis_lateral_ground_velocity;//机体轴侧向地速
    short tilt;//俯仰角
    short roll_angle;//滚转角
    unsigned short hangxiang_angle; // 航向角
    unsigned short flight_path;//应飞航向
    char current_flight_path_num;//当前飞行航线号
    char current_flight_waypoint_num;//当前飞行航点号
    // 以下有具体信号注解
    char first_level_control_state;//第一级控制状态
    char second_level_control_state;//第二级控制状态
    char emergency_failure_disposal_mode;//应急故障处置模式
    char emergency_troubleshooting_program;//应急故障处置方案
    char flight_subphase;//飞行子阶段
    char standby;//备用
}unmanned_helicopter_flight_information;

typedef struct manned_helicopter_flight_information//有人直升机飞行信息
{
    long long manned_helicopter_longitude;//经度
    long long manned_helicopter_latitude;//纬度
    //    colongitude_and_latitude manned_helicopter_colongitude_and_latitude;//有人机经纬度
    int manned_helicopter_height;//有人机高度
    int manned_helicopter_speed;//有人机速度
    int manned_helicopter_direction;//有人机航向
    int manned_helicopter_tilt;//有人机俯仰角
    int manned_helicopter_roll_angle;//有人机横滚角
    unsigned short manned_helicopter_valid_flag_bit;//有人机数据有效位
    unsigned char beifen;//备份(待定)
}manned_helicopter_flight_information;

typedef struct BLK_CCC_KDL_000
{
    // unsigned int head = 0X0202;
    char  synchronization_code[2];//同步码 2  / 0xEB 0x90
    char frame_class;//帧类别 1 / 0xB3
    manned_helicopter_flight_information manned_helicopter_flight_informations;//有人直升机飞行信息
    unmanned_helicopter_flight_information unmanned_helicopter_flight_informations[2];//无人直升机1、2飞行信息
    data_fusion data_fusions;//数据融合
    //    char standby;//备用
    char checksum;//校验和
}BLK_CCC_KDL_000;

/*BLK_CCC_KDL_000 end******************/


/*BLK_CCC_KDL_002 start******************/

// CCC-KDL/MMM-002 控制权交接申请(MessageID:0xa22402)
typedef struct {
    unsigned short C_Down_channel;//C链下行频道号
    unsigned short C_UP_Channel;//C链上行频道号
    unsigned short CDownRate;//C链下行速率:0-NA,1-2Mb/s,2-4M/s,3-8M/S
    unsigned short U_Channel;//U链频道号
    unsigned char valid;//数据有效位：
    //bit0:C链下行频道，0无效，1有效；
    //bit1:C链上行频道，0无效，1有效；
    //bit2:C链下行速率，0无效，1有效；
    //bit3:U链下行速率，0无效，1有效；
    //bit4~7:spare
} SIGNAL_FC00_Send;//舰面站交接参数

typedef struct {
    char synchronization_code[2];//同步码 2 0xEB, 0x94
    char frame_class;//帧类别 1 0xD8
    unsigned char index_id;// 流水号 1  1-255循环
    unsigned int UAV_ID;//无人机ID 4
    unsigned int F_ID2;//有人机ID 4
    SIGNAL_FC00_Send CR_StationHandoverParamss; //舰面站交接参数 9
    unsigned char jyh; //校验和 1       0-256(和校验)
} BLK_CCC_KDL_002;//控制权交接申请

/*BLK_CCC_KDL_002 end******************/


/*BLK_CCC_KDL_004 start******************/

//1.1 无人机1光电视频 004
typedef struct BLK_CCC_KDL_004
{
    unsigned int head;
    char synchronization_code[2];//同步码 2 0xEB, 0x94
    char frame_class;//帧类别 1 0xB7
    char airplane_ID[4];//飞机ID 4
    char video_type;//视频类型 1 0:光电电视；1：光电红外
    char frame_length[2];//帧长 2
    char video[2];//无人机1视频数据内容（包头＋单元区域视频数据）video_data_content
}BLK_CCC_KDL_004;

/*BLK_CCC_KDL_004 end******************/


/*BLK_CCC_KDL_010 start******************/

// 控制权交接申请应答
typedef struct {
    char synchronization_code[2];//同步码 2 0xEB, 0x94
    char frame_class;//帧类别 1 0xDB
    unsigned char index_id;// 流水号 1  1-255循环
    unsigned char ans; // 应答信息 1 /  0：同意；1：拒绝  /0-拒绝 1-同意 2-参数设置失败 3-参数设置成功
    unsigned char jyh; //校验和 1       0-256(和校验)
} BLK_CCC_KDL_010;//控制权交接申请

/*BLK_CCC_KDL_010 end******************/


/*BLK_CCC_KDL_012 start******************/

//1.2 无人机2光电视频 012
typedef struct BLK_CCC_KDL_012
{
    unsigned int head;
    char synchronization_code[2];//同步码 2  / 0xEB, 0x94
    char frame_class;//帧类别 1  / 0xB7
    char airplane_ID[4];//飞机ID 4
    char video_type;//视频类型 1 /视频类型 1 0:光电电视；1：光电红外
    char frame_length[2];//帧长 2
    char spare[2];//无人机2视频数据内容 2
}BLK_CCC_KDL_012;

/*BLK_CCC_KDL_012 end******************/


/*BLK_CCC_KDL_014 start******************/
typedef union
{
    unsigned short value;
    struct
    {
        unsigned short manned_helicopter : 1; // 有人直升机（0，不在网；1，在网）
        unsigned short uav1 : 1; // 无人直升机1（0，不在网；1，在网）
        unsigned short uav2 : 1; // 无人直升机2（0，不在网；1，在网）
        unsigned short uav3 : 1; // 无人直升机3（0，不在网；1，在网）
        unsigned short uav4 : 1; // 无人直升机4（0，不在网；1，在网）
        unsigned short ship : 1; // 协同通信系统（模拟舰面端）（0，不在网；1，在网）
        unsigned short ground : 1; // 测控地面终端（0，不在网；1，在网）
        unsigned short spare : 9; // 备用
    };

}U_MEMBER;

typedef union
{
    unsigned short value;
    struct
    {
        unsigned short spare0 : 1; // 备用
        unsigned short uav1 : 1; // 无人直升机1（0，未通；1，联通）
        unsigned short uav2 : 1; // 无人直升机2（0，未通；1，联通）
        unsigned short uav3 : 1; // 无人直升机3（0，未通；1，联通）
        unsigned short uav4 : 1; // 无人直升机4（0，未通；1，联通）
        unsigned short uav1DownRate : 2; // 无人直升机1下传速率（0，2Mbps；1，4Mbps; 2, 8Mbps）
        unsigned short uav2DownRate : 2; // 无人直升机2（0，2Mbps；1，4Mbps; 2, 8Mbps）
        unsigned short uav3DownRate : 2; // 无人直升机3（0，2Mbps；1，4Mbps; 2, 8Mbps）
        unsigned short uav4DownRate : 2; // 无人直升机4（0，2Mbps；1，4Mbps; 2, 8Mbps）
        unsigned short spare1 : 3; // 备用
    };

}C_UAV_F;

typedef union
{
    unsigned short value;
    struct
    {
        unsigned short spare0 : 1; // 备用
        unsigned short manned_helicopter : 1; // 有人直升机（0，未通；1，联通）
        unsigned short downRate : 2; // 地空C链下行传输速率（0，2Mbps；1，4Mbps; 2, 8Mbps）
        unsigned short spare1 : 13; // 备用
    };

}C_Station_F;

// 编队链路状态信息 014
typedef struct kNETWORK_MEMBER
{
    U_MEMBER u_member;		/*U链在网成员*/
    C_UAV_F c_uav_f;			/*C链联通情况（有人/无人）*/
    C_Station_F c_station_f;   /*C链联通情况（有人/模拟舰面端）*/
}kNETWORK_MEMBER;/*网络拓扑信息*/

//typedef struct BLK_CCC_KDL_014{
//    char synchronization_code[2];//同步码 2  / 0xEB, 0x9C
//    char frame_type; // 帧类别 1  / 0xBA
//    kNETWORK_MEMBER NETWORK_MEMBER_info; ///*网络拓扑信息*/  5
//    char check_num; // 校验和（和校验mod256）
//}BLK_CCC_KDL_014;

/*BLK_CCC_KDL_014 end******************/


/*BLK_CCC_KDL_017 start******************/

// 任务分配结果信息 017
typedef struct ktask_sequence_information//任务序列信息
{
    unsigned int id;					/*任务平台子任务ID号*/
    unsigned short modify_flag;			/*任务平台子任务是否变更  0=N/A;1=任务序列变更;2=任务区/任务点变更;3=任务航线变更;4=浮标/定测点阵型变更;*/
    unsigned short type;				/*子任务任务类型  0=N/A;1=浮标侦收;2=吊声定测;3=浮标布阵;4=通信中继;5=磁探搜索;6=磁探跟踪;7=光电搜索;8=光电跟踪;9=编队飞行;10=任务导航;11=返航;12=等待;13=临时改航;*/
    unsigned short point_or_area;		/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
    unsigned int point_or_area_id;		/*任务区/点/线/目标编号*/
    unsigned char finish_time_valid;	/*子任务完成时间有效位*/
    unsigned char finish_height_valid;  /*子任务任务高度有效位*/
    unsigned int finish_time;			/*子任务完成时间*/
    unsigned int finish_height;			/*子任务完成高度*/
    unsigned char exist_airway;			/*子任务是否存在航线*/
    unsigned char exist_buoy_deploy;	/*子任务是否存在浮标阵*/
    unsigned char exist_sonar_measurement_format;/*子任务是否存在定测点阵型*/
}ktask_sequence_information;

typedef struct kformation_synergy_mission_program//编队任务协同方案
{
    unsigned short platform_model;   /*平台1型号  0=N/A;1=20F;2=WZ2;*/
    unsigned short platform_sn;   /*平台序号*/
    unsigned int platform_code;   /*平台编号*/
    unsigned int platform_task_time;   /*任务平台任务时长*/
    unsigned short platform_subtask_number;   /*任务平台子任务个数m*/
    ktask_sequence_information task_sequence_informations[8];//任务序列信息（循环8次）

}kformation_synergy_mission_program;

typedef struct coinformation_on_the_results_of_tasking
{
    char synchronization_code[2];//同步码 2 /  0xEB 0x94
    char frame_type; // 帧类别   / 0xB5
    unsigned int plan_id;                   /*方案编号*/
    unsigned short plan_release_mode;		/*任务分配方案发布  0=N/A;1=发布;2=首次生成;3=修改;4=取消;*/
    unsigned short plan_manual_amend_flag;  /*任务分配方案是否人工修改  0=N/A;1=修改;2=未修改;*/
    unsigned short plan_replanning_flag;	/*任务分配方案是否自动重规划  0=N/A;1=是;2=否;*/
    unsigned short plan_altermode;			/*任务分配方案修改方式  0=N/A;1=子任务修改;2=航线修改;*/
    unsigned short plan_attribute;			/*方案属性  0=NA;*/
    unsigned short submarine_search_mode;   /*协同搜潜方式  0=均匀搜索;1=协同包围;2=协同拦阻;*/
    unsigned int   plan_total_time;			/*方案总时间 单位 S（秒）*/
    unsigned short platform_num;			/*任务平台个数n（1-5）*/
    kformation_synergy_mission_program formation_synergy_mission_programs[5];//编队协同任务方案（循环5次） 230
    char check_num; // 校验和

}BLK_CCC_KDL_017;

/*BLK_CCC_KDL_017 end******************/


/*BLK_CCC_KDL_018 start******************/

// 有人机通用航路 018

typedef struct kwaypoint_information
{
    unsigned short type;//航路点类型
    unsigned char validity_of_longitude;//航路点经度有效性
    unsigned char latitude_validity;//航路点维度有效性
    unsigned char height_validity;//航路点高度有效性
    unsigned char speed_validity;//航路点速度有效性
    unsigned char direction_validity;//航路点航向有效性
    unsigned char payloads_validity;//航路点载荷有效性
    unsigned char time_validity;//航路点时间有效性
    colongitude_and_latitude waypoint_longitude_and_latitude;//航路点经纬度
    unsigned int height;//无人机高度
    unsigned int speed;//航路点速度
    unsigned int direction;//航路点航向
    unsigned int time;//航路点时间
    unsigned short payloads;//航路点载荷
}kwaypoint_information;//航路点信息

typedef struct BLK_CCC_KDL_018//通用航路
{
    // unsigned int head = 0X0207;
    char synchronization_code[2];//同步码 2  /  0xEB 0x94
    char frame_type; // 帧类别 / 0xB5
    unsigned int   plan_code;					/*方案编号*/
    unsigned short subtask_cnt;					/*有人机子任务个数m*/
    unsigned int   subtask_index;				/*子任务ID号*/
    unsigned short airway_point_num;			/*航路点个数n    最小值：0  最大值：40*/
    unsigned short airway_point_start_num;		/*航路点起始编号*/
    kwaypoint_information waypoint_informations[15];//航路点信息
    char check_num; // 校验和

}BLK_CCC_KDL_018;

/*BLK_CCC_KDL_018 end******************/


/*BLK_CCC_KDL_024 start******************/

// 无人机航路规划 024
typedef struct kplanning_information_waypoint_information//航路点信息
{
	unsigned char type;//航路点类型
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
	unsigned char payloads;//航路点载荷
    unsigned char causality;//航路点属性
    unsigned char standby_type;//航路点待机类型
    unsigned int standby_time_lapsNumber_cycleNumber;//航路点待机时间/圈数/循环次数
    unsigned char standby_radius_valid_bit;//航路点待机半径有效位
    unsigned char standby_time_lapsNumber_cycleNumber_valid_bit;//航路点待机时间/圈数/循环次数有效位
    unsigned int standby_radius;//航路点待机半径
}kplanning_information_waypoint_information;

typedef struct kplanning_information//单无人机单序列中的规划信息
{
    unsigned int subtask_ID_number;//子任务ID号
    unsigned short mission_type;//子任务任务类型
    unsigned short point_area_type;//任务点/区域类型
    unsigned int area_point_line_goal_number;//子任务任务区/点/线/目标编号
    unsigned char AtomicTimeUpper;//子任务完成时间有效位
    unsigned char AtomicHighlyUpper;//子任务高度有效位
    unsigned int completion_time;//子任务完成时间
    unsigned int mission_height;//子任务任务高度
    unsigned int waypoints_number;//航路点个数n
    kplanning_information_waypoint_information planning_information_waypoint_informations[13];//航路点信息

}kplanning_information;

typedef struct kindividual_drone_routing_program//单个无人机航路方案
{
    unsigned short drone_serial_number;//无人机序号
    unsigned int drone_num;//无人机编号
    unsigned short subtasks_number ;//当前无人机子任务个数
    // unsigned short subtask_index; // 从0开始
    kplanning_information planning_informations;
}kindividual_drone_routing_program;

typedef struct BLK_CCC_KDL_024
{
    char synchronization_code[2];//同步码 2  / 0xEB 0x90
    char frame_type; // 帧类别  / 0x51
    unsigned int program_number;//方案编号
    unsigned short number_of_drones;//无人机数量
    kindividual_drone_routing_program individual_drone_routing_programs;//单个无人机航路方案
    char check_num; // 校验和
}BLK_CCC_KDL_024;

/*BLK_CCC_KDL_024 end******************/


/*BLK_CCC_KDL_033 Start******************/

// 任务区/空域信息 033
typedef struct kcycle
{
    colongitude_and_latitude cycle_longitude_and_latitude;//圆心经纬度
    float radius;//圆心半径
}kcycle;

typedef struct kpolygonal//多边形
{
    unsigned short point_number;//点数n
    colongitude_and_latitude point_coordinates[10];//点坐标

}kpolygonal;

typedef struct karea_information//区域信息
{
    unsigned int area_code;//区域编号
    unsigned short area_type;//区域类型
    unsigned short area_source;//区域来源
    unsigned short area_shape;//区域形状
    unsigned short area_platform_num;//区域所属平台
    unsigned int drone_numbe;//空域所属无人机序号
    unsigned char upper_height_limit_valid_bit;//区域高度上限有效位置
    unsigned char lower_height_limit_valid_bit;//区域高度下限有效位置
    float upper_height_limit;//区域高度上限
    float lower_height_limit;//区域高度下限
    kcycle cycles;//圆
    kpolygonal polygonals;//多边形

}karea_information;

typedef struct BLK_CCC_KDL_033
{
    char synchronization_code[2];//同步码 2   / 0xEB 0x94
    char frame_type; // 帧类别  /0xB8
    unsigned short area_number;//区域个数
    karea_information area_informations[8];//区域信息
    char check_num; // 校验和
}BLK_CCC_KDL_033;

/*BLK_CCC_KDL_033 end******************/


/*BLK_CCC_KDL_034 start******************/

// 任务点信息 034
typedef struct kpoint_information//点信息
{
    unsigned int point_number;//点编号
    unsigned short point_source_of_information;//点信息来源
    colongitude_and_latitude point_longitude_and_latitude;//点经纬度
    char point_speed_effective;//点目标航速有效
    char point_direction_effective;//点目标航向有效
    char point_time_effective;//点目标时间有效
    char point_property_effective;//点目标属性有效位置
    char point_type_effective;//点目标类型有效位置
    char point_batchNumber_effective;//点目标批号有效位置
    float speed;//目标航速
    float direction;//目标航向
    unsigned short year;//年
    unsigned short month;//月
    unsigned short day;//日
    int millisecond;//毫秒
    unsigned short point_property;//目标属性
    unsigned short point_type;//目标类型
    unsigned int point_batchNumber;//批号

}kpoint_information;

typedef struct kcotaks_point_information
{
    char synchronization_code[2];//同步码 2  / 0xEB 0x94
    char frame_type; // 帧类别  /0xB9
    unsigned short point_number;//任务点个数
    kpoint_information point_informations[30];//点信息
    char check_num; // 校验和
}BLK_CCC_KDL_034;

/*BLK_CCC_KDL_034 end******************/


/*BLK_CCC_KDL_035 start******************/

// 任务线信息 035
typedef struct kline_information
{
    unsigned int line_num;//线编号
    unsigned short line_type;//线类型
    unsigned short line_point_number;//任务线点个数
    copoint_coordinate point_coordinates[10];//点经纬度

}kline_information;

typedef struct BLK_CCC_KDL_035
{
    char synchronization_code[2];//同步码 2  / 0xEB 0x94
    char frame_type; // 帧类别  / 0xC1
    unsigned short line_number;//任务线条数
    kline_information line_informations[5];//线信息
    char check_num; // 校验和
}BLK_CCC_KDL_035;

/*BLK_CCC_KDL_035 end******************/

//空空链状态帧
typedef struct
{
	char synchronization_code[2];//同步码 2   / 0xEB 0x9c
	char frame_type; // 帧类别  /0xBA
	char frame_long; // 帧长 0x10
	char frame_cout; // 帧计数
	unsigned short plane_id; //0x9001
	unsigned char uav1_up_lock;//无人机1上行锁定
	unsigned char uav2_up_lock;//无人机2上行锁定
	unsigned char uav1_down_lock;//无人机1下行锁定
	unsigned char uav2_down_lock;//无人机1下行锁定
	char spare[4];
	char checksum;//前15字节之和
}BLK_CCC_KDL_014;




#pragma pack()





#endif // KDL_ICD_H
