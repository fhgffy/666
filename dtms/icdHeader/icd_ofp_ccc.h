#ifndef SYNTHESIZECOOPERATIVEALLEGATIONCOMPUTER_H
#define SYNTHESIZECOOPERATIVEALLEGATIONCOMPUTER_H

/*
 * 综显-协同指控计算机
**/


#pragma pack(1)

//广播轮载消息 20250727new
typedef struct
{
	unsigned int mainHns		:2;/*当前主惯导。0-无，1-激光1,2-激光2*/
	unsigned int wowStatus		:1;/*轮载信号 0-空中，1-地面*/
	unsigned int umsSource		:2;/*新增：机电数据源：0-NA；1-UMS1；2-UMS2；3-NA*/
	unsigned int HelicopterType	:1;/*新增20200414：机型识别：0-J突击运输；1-F反潜反舰*/
	unsigned int timeSource		:2;/*新增20201214：时间源：0-NA；1-数据链；2-导航；3-人工*/
	unsigned int spare2			:8;

	unsigned int MDCS_Line		:1;/*任务显示控制软件 0-OFF;1-ON*/
	unsigned int TPS_Line		:1;/*战术处理软件 0-OFF;1-ON*/
	unsigned int FJ_FIRE_Line	:1;/*反舰攻击解算软件 0-OFF;1-ON*/
	unsigned int SSZS_LINE_Line	:1;/*水声战术支持软件 0-OFF;1-ON*/
	unsigned int FQ_FIRE_Line	:1;/*反潜攻击解算软件 0-OFF;1-ON*/
	unsigned int DATA_TS_Line	:1;/*数据融合与态势综合软件 0-OFF;1-ON*/
	unsigned int RADAR_HJ_Line	:1;/*雷达航迹处理软件 0-OFF;1-ON*/
	unsigned int ESPS_PRO_Line	:1;/*电子侦察/自卫控制处理软件 0-OFF;1-ON*/
	unsigned int ESPS_WY_Line	:1;/*电子侦察/自卫无源定位处理软件 0-OFF;1-ON*/
	unsigned int SSDS			:1;/*光电搜索设备 0-OFF;1-ON*/
	unsigned int IFF			:1;/*敌我识别应答 0-OFF;1-ON*/
	unsigned int BUOY			:1;/*浮标投放装置 0-OFF;1-ON*/
	unsigned int spare1			:4;
}OFP_NT_BROADCAST_MSG;

//经纬度
typedef struct longitude_and_latitude_synt
{
    double longitude;//经度
    double latitude;//维度
}longitude_and_latitude_synt;


// DPU1-CCC/MMM-014 控制权交接指令（MessageID：0x062a0e）
typedef struct{
    unsigned short UAV_Type;//无人机型号（Enum）（0H=N/A,1H=WZ2）
    unsigned short UAV_Seri_Number;//无人机序号
    unsigned int UAV_ID;//无人机编号
    unsigned short Control_type;//无人机控制权交接命令（Enum）（0H=NA，1H=申请控制权，2H=主动释放控制权，3H=同意舰面申请控制权, 4H=拒绝舰面申请控制权）
//    unsigned int   recieve_id;		/*0914新增 接收站地址 （当需要直接给地面站控制权时使用，目前未使用） 告诉李翔加上diff*/
#ifdef _SPARE_
    unsigned char spare[6];
#endif
}CR_Handover_UAV_Info;//交接无人机信息（循环次数4）

typedef struct
{
    unsigned short Null0 : 11;
    unsigned short signal_FC00_CCC11 : 1;     /*交接下行速率是否有效  0=无效;1=有效;*/
    unsigned short signal_FC00_CCC11_1 : 1;   /*交接下行频道是否有效  0=无效;1=有效;*/
    unsigned short signal_FC00_CCC11_2 : 1;   /*交接上行频道是否有效  0=无效;1=有效;*/
    unsigned short signal_FC00_CCC11_3 : 1;   /*U链频道号是否有效  0=无效;1=有效;*/
    unsigned short signal_FC00_CCC11_4 : 1;   /*接收平台编号是否有效  0=无效;1=有效;*/
}LinkValidBits;  /*数据有效位*/

typedef struct{
    LinkValidBits dataValid;
    unsigned int uav_code;				/*uav平台编号*/
    unsigned int rcv_platform_code;		/*uav接收平台编号*/
    unsigned short u_channel;				/*uav U链频道号*/
    unsigned short handover_up_channel;	/*交接上行频道号*/
    unsigned short handover_down_channel;	/*交接下行频道号*/
    unsigned short down_rate;				/*下行速率*/
#ifdef _SPARE_
    unsigned char spare[2];
#endif
}CR_CommunicationParam;//交接参数

typedef struct{
    unsigned short UAV_Number;//无人机个数n，范围：0-4
    CR_Handover_UAV_Info CR_Handover_UAV_Infos[4];//交接无人机信息（循环次数4）
    CR_CommunicationParam CommunicationParam;//交接参数
}BLK_OFP_CCC_014;//控制权交接指令



//4.2无人机自主权限等级设置
//typedef struct drone_permission_information//无人机权限信息
//{
//    unsigned short drone_type;//无人机型号
//    unsigned short drone_serial_number;//无人机序号
//    unsigned int drone_number;//无人机编号
//    unsigned short drone_autonomous_authority_levels;//无人机自主权限等级
//}drone_permission_information;
typedef struct drone_autonomous_privilege_level_setting
{
    unsigned short head; //0X0401;
    unsigned short uav_self_authority;  /*无人机自主权限等级  0=N/A;1=低自主等级;2=高自主等级;*/
    // unsigned short drone_num ;//无人机个数
    //drone_permission_information drone_permission_informations[4];//无人机权限信息
}drone_autonomous_privilege_level_setting;


typedef struct
{
	//1031 已更新
	unsigned short spare:1;  					/*有人机磁差 1219*/
	unsigned short TrueWindHeadingValid : 1;	/*0-有人机真风向（分布大气）无效 1-有效*/
	unsigned short WindSpeedValid : 1;			/*0-有人机风速（分布大气）无效 1-有效*/
	unsigned short HeliMagneticValid : 1;		/*0-磁差无效 1-有效*/
	unsigned short HeliTrueHeadingValid : 1;	/*0-真航向无效 1-有效*/
	unsigned short HeliRollValid : 1;			/*0-横滚角无效 1-有效*/
	unsigned short HeliPitchValid : 1;			/*0-俯仰角无效 1-有效*/
	unsigned short TrueGroundTrackAngleValid : 1;/*0-真航迹角无效 1-有效*/
	unsigned short GroundSpeedValid : 1;		/*0-地速无效 1-有效*/
	unsigned short TrueAirSpeedValid : 1;		/*0-真空速(分布大气)无效 1-有效*/
	unsigned short WXHeightValid : 1;			/*0-卫星高度无效 1-有效*/
	unsigned short WXDHeightValid : 1;			/*0-无线电高度无效 1-有效*/
	unsigned short AbsolutePressureAltValid : 1;/*0-绝对气压高度(分布大气)无效 1-有效*/
	unsigned short HeliLatValid : 1;			/*0-有人机纬度无效 1-有效 */
	unsigned short HeliLongValid : 1;			/*0-有人机经度无效 1-有效*/
	unsigned short TimeTagValid : 1;			/*0-有人机时间标签无效 1-有效*/

}DataValid_yourenji;

typedef struct
{
	 ///////////////////////////////新增 ///////////////
	unsigned char spare:1;  					/*有人机磁差 1219*/
	unsigned char HELI_MAGNETIC_HEADING_Valid : 1;			/*0-无效 1-有效*/
	unsigned char SPEED_EAST_Valid : 1;			/*0-无效 1-有效*/
	unsigned char SPEED_NORTH_Valid : 1;			/*0-无效 1-有效*/
	unsigned char SPEED_UP_Valid : 1;			/*0-无效 1-有效*/
	unsigned char MAGNETIC_GROUND_TRACK_ANGLE_Valid : 1;			/*0-无效 1-有效*/
	unsigned char HELI_HEADING_RATE_Valid : 1;			/*0-无效 1-有效*/
	unsigned char WX_HEIGHT_Valid : 1;			/*0-无效 1-有效*/
}DataValid2;


//4.3 有人机导航数据信息
//TODO 确认块号：MessageID 0x062a10
//TODO 补充其他结构体的MessageID
typedef struct manned_navigation_data_information
{
	DataValid_yourenji valid_flag;
    //unsigned short data_valid_bits;//数据有效位
    double time_lable;//时间标签
    longitude_and_latitude_synt manned_longitude_and_latitude_synt;//直升机经纬度
    double absolute_barometric_altitude;//有人机绝对气压高度
    double radio_altitude;//无线电高度
    long long weixing_heigth; // 卫星高度
    double vacuum_speed; //真空速（大气分布）
    double groundspeed; //有人机 地速度
    double true_track_angle; //真航迹角
    double tilt;//有人机俯仰角
    double roll_angle;//有人机横滚角
    double true_direction;//有人机真航向
    double magnetic_difference;//磁差
    double air_velocity;//风速
    double custom_direction;//风向
    DataValid2 valid_flag2;
    double HELI_MAGNETIC_HEADING;   /*磁航向  当浮点数大于360或小于0时，DPU将判定数据无效，显示--；,整数显示时，舍入到360度 时显示为0度。*/
    double SPEED_EAST;   /*东向速度  HNS需限定输出值在[-1024,1024]区间内；超出此范围任务机认为数据无效。*/
    double SPEED_NORTH;   /*北向速度  HNS需限定输出值在[-1024,1024]区间内；超出此范围任务机认为数据无效。*/
    double SPEED_UP;   /*天向速度  HNS需限定输出值在[-1024,1024]区间内；超出此范围任务机认为数据无效。*/
    double MAGNETIC_GROUND_TRACK_ANGLE;   /*磁航迹角  当浮点数大于360或小于0时，DPU将判定数据无效，显示--；*/
    double HELI_HEADING_RATE;   /*航向角速率  相对航向顺时针为正；		当浮点数大于180或小于-180时，DPU将判定数据无效，显示--*/
    short WX_HEIGHT;/*卫星高度*/
}manned_navigation_data_information;



//4.4 有人机目标集
typedef struct DataValidFlag
{
    unsigned short Null0 : 9;
    unsigned short longi_valid : 1;   /*经度  1-有效/0-无效*/
    unsigned short lati_valid : 1;   /*纬度  1-有效/0-无效*/
    unsigned short heading_valid : 1;   /*航向  1-有效/0-无效*/
    unsigned short speed_valid : 1;   /*速度  1-有效/0-无效*/
    unsigned short height_valid : 1;   /*高度  1-有效/0-无效*/
    unsigned short time_tag_valid : 1;   /*发现时间  1-有效/0-无效*/
    unsigned short threat_sn_valid : 1;   /*威胁序号  1-有效/0-无效*/
}DataValidFlag;  /*目标数据有效性*/

typedef struct target_information //目标信息
{
    DataValidFlag valid_flag;	/*目标数据有效性*/
    char platform_logo;//目标平台标识
    unsigned int target_lot_number1;//目标批号1
    unsigned int target_lot_number2;//目标批号2
    unsigned short Send_platform_identification_number;//目标发送平台编识号
    longitude_and_latitude_synt target_longitude_and_latitude_synt;//目标经纬度
    float direction;//目标航向
    unsigned short speed;//目标速度
    unsigned short height;//目标高度
    unsigned int tgt_found_time;   /*目标发现时间*/
    unsigned char tgt_location;   /*环境类别  0=无报告;1=空中;2=水面;3=水下;4=陆地;5=空间;*/
    unsigned char simulation_flag;   /*模拟标识  0=非模拟;1=模拟;*/
    unsigned char training_flag;   /*演习标识  0=非演习;1=演习;*/
    unsigned char group_or_single;   /*群目标单目标*/
    unsigned char tgt_locked_status;   /*目标锁定状态  0=解锁;1=锁定;*/
    unsigned char tgt_attribute;   /*目标属性  0=n.a;1=敌;2=我;3=友;4=中立;5=不明;*/
    unsigned char tgt_givenby;   /*目标来源  0=905;1=jids;2=本机雷达;3=卫星;4=综合;*/
    char tgt_course_quality;   /*航迹质量*/
    unsigned char tgt_threat_sn;   /*目标威胁序号*/
}target_information;
typedef struct manned_aircraft_target_set
{
    unsigned short message_ID;//消息ID
    char link_identifier;//链路标识
    char target_number ;//目标数量
    target_information target_informations[30];//目标信息
}manned_aircraft_target_set;



//4.5 任务分配结果确认信息
typedef struct task_sequence_information_confirm//任务序列信息
{
    unsigned int subtask_ID_number;//任务平台子任务ID号
    unsigned short modify;//任务平台子任务是否变更方式
    unsigned short sequence_type;//任务平台子任务任务序列类型
    unsigned short type_of_mission_point_area;//任务平台子任务任务点/区域类型
    unsigned int target_number;//任务平台子任务任务区/点/线/目标编号
    char completion_time_valid_Bits;//任务平台子任务完成时间有效位
    char task_height_effective_position;//任务平台子任务任务高度有效位
    unsigned int task_completion_time;//任务平台子任务完成时间
    unsigned int height;   /*子任务完成高度*/
    unsigned char airline_exist_flag;   /*子任务是否存在航线*/
    unsigned char buoy_exist;   /*子任务是否存在浮标阵*/
    unsigned char sonar_measure_exist;   /*子任务是否存在定测点阵型*/
//    unsigned char spare[13];
}task_sequence_information_confirm;

typedef struct formation_synergy_mission_program_confirm//编队任务协同方案
{
    unsigned short platform_model;//任务平台型号  有人机/无人机
    unsigned short platform_serial_number;//任务平台序号
    unsigned int platform_number;//任务平台编号
    unsigned int duration_of_the_mission;//任务平台任务时长
    unsigned short number_of_subtasks;//任务平台子任务个数m
    task_sequence_information_confirm task_sequence_information_confirms[8];//任务序列信息（循环8次）
//    unsigned char spare[66];
}formation_synergy_mission_program_confirm;

//TODO 应该是DPU给CCC
typedef struct information_on_the_results_of_tasking_confirm
{
    unsigned int program_number;   //方案编号
    unsigned short tasking_release;//任务分配发布
    unsigned short manual_modification;    //任务分配方案是否人工修改
    unsigned short emphasize_planning;     //任务分配方案是否重规划
    unsigned short task_mod_model;         // 任务方案修改方式
    unsigned short program_attributes;     //方案属性
    unsigned short search_and_dive_method; //协同搜潜方式
    unsigned int total_program_time;        //方案总时间
    unsigned short number_of_mission_platforms;//任务平台个数n
    formation_synergy_mission_program_confirm formation_synergy_mission_program_confirms[5];//编队协同任务方案（循环5次）
}Information_the_results_of_tasking_confirm;


// 111
//4.6 有人机航线信息
typedef struct suspended_sound_buoy_deployment_point_information_confirm
{
    unsigned char lon_valid;  // 经纬度有效性
    unsigned char lat_valid;
    unsigned char profundity_valid;  // 深度有效性
    longitude_and_latitude_synt point_longitude_and_latitude_synt; //吊声点经纬度
    double profundity; // 吊声点深度

}suspended_sound_buoy_deployment_point_information_confirm;//吊声点信息
typedef struct suspended_sound_manned_computer_task_confirm//有人机子任务
{
    unsigned int subtask_ID_number;//子任务ID
    unsigned short fixed_measurement_points_number;//定测点总数
    suspended_sound_buoy_deployment_point_information_confirm suspended_sound_buoy_deployment_point_informations[25];//吊声点信息
    // char spare[124]; // 信息补位
}suspended_sound_manned_computer_task_confirm;
typedef struct suspended_sound_fixed_measurement_point_confirm//吊声定测点
{
    unsigned short head; //0X0405;
    unsigned int program_number;//方案编号
    unsigned short subtasks_number;//有人机子任务个数m
    suspended_sound_manned_computer_task_confirm suspended_sound_manned_computer_tasks;//有人机子任务（循环8次）
}suspended_sound_fixed_measurement_point_confirm;
typedef struct buoy_deployment_point_information_confirm//浮标布阵点信息
{
    unsigned short matching_tube_number;//浮标布阵点匹配筒位号
    unsigned short buoy_type;//浮标布阵点浮标类型
    char longitude_validity;//浮标布阵点经度有效性
    char latitude_validity;//浮标布阵点维度有效性
    longitude_and_latitude_synt buoy_longitude_and_latitude_synt;//浮标布阵点经纬度

}buoy_deployment_point_information_confirm;
typedef struct buoy_manned_computer_task_confirm//有人机子任务
{
//    unsigned int subtask_ID_number;//子任务ID号
//    unsigned short points_number;//浮标布阵点总数
    buoy_deployment_point_information_confirm buoy_deployment_point_informations[25];//浮标布阵点信息

}buoy_manned_computer_task_confirm;
typedef struct buoy_deployment_point_confirm//浮标布阵点
{
    unsigned short head; //0X0406;
    unsigned int program_number;//方案编号
    unsigned short subtasks_number;//有人机子任务个数m
    buoy_manned_computer_task_confirm buoy_manned_computer_tasks;//有人机子任务（循环8次）
}buoy_deployment_point_confirm;
typedef struct waypoint_information_confirm
{
	unsigned short hld_idx;				/*航路点编号		20250606 new*/
	unsigned short type;   /*航路点类型  0=N/A;1=普通航路点;2=攻击点;3=集结点;4=返航点;*/
	unsigned char longi_valid;   /*航路点经度有效性*/
	unsigned char lati_valid;   /*航路点纬度有效性*/
	unsigned char height_valid;   /*航路点高度有效性*/
	unsigned char speed_valid;   /*航路点速度有效性*/
	unsigned char heading_valid;   /*航路点航向有效性*/
	unsigned char time_valid;   /*航路点时间有效性*/
	unsigned char loading_valid;   /*航路点载荷有效性*/
    double longitude;//经度
    double latitude;//维度
	unsigned int height;   /*航路点高度*/
	unsigned int speed;   /*航路点速度*/
	unsigned int heading;   /*航路点航向*/
	unsigned int time;   /*航路点时间*/
	unsigned short loading;   /*航路点载荷  0=N/A;1=雷达;2=光电;3=吊声;4=浮标;5=武器;*/
}waypoint_information_confirm;//航路点信息
typedef struct manned_computer_task_confirm//有人机子任务
{
    waypoint_information_confirm waypoint_informations[40];//航路点信息

}manned_computer_task_confirm;
typedef struct common_carrier_route_confirm//通用航路
{
    unsigned short head; //0X0407;
    unsigned int program_number;//方案编号
    unsigned short subtasks_number;//有人机子任务个数
    unsigned int subtask_index;   /*子任务ID号*/
    unsigned short airway_point_num;   /*航路点个数n    最小值：0  最大值：40*/
    unsigned short airway_point_start_num;   /*航路点起始编号*/
    manned_computer_task_confirm manned_computer_tasks;//有人机子任务（循环8次）

}common_carrier_route_confirm;
typedef struct manned_aircraft_route_drone_route_information_conform
{
    common_carrier_route_confirm common_carrier_routes;//通用航路
    buoy_deployment_point_confirm buoy_deployment_points;//浮标布阵点
    suspended_sound_fixed_measurement_point_confirm suspended_sound_fixed_measurement_points;//吊声定测点
}manned_aircraft_route_drone_route_confirmation_information_confirm;



//4.7 无人机航线确认信息
typedef struct planning_information_waypoint_information_confirm//航路点信息
{
    waypoint_information_confirm waypoint_informations;//航路点信息
    unsigned short causality;//航路点属性
    unsigned short standby_type;//航路点待机类型
    unsigned char standby_time_lapsNumber_cycleNumber_valid_bit;//航路点待机时间/圈数/循环次数有效位
    unsigned char standby_radius_valid_bit;//航路点待机半径有效位
    unsigned int standby_time_lapsNumber_cycleNumber;//航路点待机时间/圈数/循环次数
    unsigned int standby_radius;//航路点待机半径
#ifdef _SPARE_
    unsigned char spare[7];
#endif
}planning_information_waypoint_information_confirm;
typedef struct planning_information_confirm//单无人机单序列中的规划信息
{
    unsigned int subtask_ID_number;//子任务ID号
    unsigned short mission_type;//子任务任务类型
    unsigned short point_area_type;//任务点/区域类型
    unsigned int area_point_line_goal_number;//子任务任务区/点/线/目标编号
    unsigned char completion_time_valid_Bits;//子任务完成时间有效位
    unsigned char ask_height_effective_position;//子任务任务高度有效位
    unsigned int completion_time;//子任务完成时间
    unsigned int mission_height;//子任务任务高度
    unsigned short waypoints_number;//航路点个数n
    planning_information_waypoint_information_confirm planning_information_waypoint_informations[25];//航路点信息
#ifdef _SPARE_
    unsigned char spare[76];
#endif
}planning_information_confirm;
typedef struct individual_drone_routing_program_confirm//单个无人机航路方案
{
        unsigned short drone_serial_number;//无人机序号
        unsigned int drone_id; // 无人机编号
        unsigned short subtasks_number ;//当前无人机子任务个数
        unsigned short subtask_index; // 从0开始
        planning_information_confirm planning_informations;//单无人机子任务规划信息
#ifdef _SPARE_
    unsigned char spare[72];
#endif
}individual_drone_routing_program_confirm;

//CCC-DPU1/DPU2/MMM-024 无人机航路规划 MessageID 0xa22218
typedef struct drone_route_confirmation_confirm
{
    unsigned short head; //0X0408;
    unsigned int program_number;//方案编号
    unsigned short number_of_drones ;//无人机数量
    individual_drone_routing_program_confirm individual_drone_routing_programs;//单个无人机子任务方案
}drone_route_confirmation_confirm;


//4.8 任务区/任务点设置

typedef struct //点信息
{
    unsigned short point_operations;//点操作
    unsigned int point_number;//点编号
    unsigned short point_source_of_information;//点信息来源
    double longitude;//经度
    double latitude;//维度
    unsigned char point_speed_effective;//点目标航速有效
    unsigned char point_direction_effective;//点目标航向有效
    unsigned char point_time_effective;//点目标时间有效
    unsigned char point_property_effective;//点目标属性有效位置
    unsigned char point_type_effective;//点目标类型有效位置
//    unsigned char point_batchNumber_effective;//点目标批号有效位置
    float speed;//目标航速
    float direction;//目标航向
    short year;//年
    short month;//月
    short day;//日
    int millisecond;//毫秒
    unsigned short point_property;//目标属性
    unsigned short point_type;//目标类型
    unsigned int point_batchNumber;//批号
#ifdef _SPARE_
    unsigned char spare[5];
#endif
}point_information_confirm;
typedef struct //任务点设置
{
    unsigned short number_of_point_to_be_modified;//需要修改的点个数
    point_information_confirm point_informations[30];//点信息
}point_setting_confirm;


//// 任务区相关
typedef struct point_coordinate_confirm//点坐标
{
    double longitude;//经度
    double latitude;//维度
}point_coordinate_confirm;

typedef struct cycle_confirm
{
    longitude_and_latitude_synt cycle_longitude_and_latitude_synt;//圆心经纬度
    float radius;//圆心半径
}cycle_confirm;
typedef struct polygonal_confirm//多边形
{
    unsigned short point_number;//点数n
    point_coordinate_confirm point_coordinates[10]; // 顶点坐标
    // char spare[8]; // 补位信息
}polygonal_confirm;
typedef struct area_information_confirm//区域信息
{
    unsigned short reg_opetour; // 区域操作
    unsigned int area_code;//区域编号
    unsigned short area_type;//区域类型
    unsigned short area_source;//区域来源
    unsigned short area_shape;//区域形状
    // unsigned char drone_number_valid_bit;//空域所属无人机编号有效位
    unsigned short area_plat; // 空域所属平台
    unsigned int drone_numbe;//空域所属无人机序号
    unsigned char upper_height_limit_valid_bit;//区域高度上限有效位置
    unsigned char lower_height_limit_valid_bit;//区域高度下限有效位置
    float upper_height_limit;//区域高度上限
    float lower_height_limit;//区域高度下限
    cycle_confirm cycles;//圆
    polygonal_confirm polygonals;//多边形
}area_information_confirm;

typedef struct area_setting//任务区域设置
{
    unsigned short number_of_areas_to_be_modified ;//需要修改的区域个数
    area_information_confirm area_informations[8];//区域信息
}area_setting;

typedef struct area_point_setting
{
    area_setting area_settings;//任务区域设置
    point_setting_confirm point_settings;//任务点设置
}area_point_setting;



//4.9 全局任务规划命令
typedef struct {
    unsigned int area_point_number; // 编号
} area_point_number;

typedef struct
{
    unsigned short mission_type;                 /*任务类型  0=检查反潜;1=应召反潜;2=预规划A;3=预规划B;*/
    unsigned short tactical_warfare_options;     /*战术战法选择  0=水面均匀覆盖;1=包围;2=拦阻;*/
    unsigned short target_category;              /*目标类别  0=N/A;1=任务区;2=任务点;*/
    unsigned short area_point_num ;              /*任务区/应召点个数*/
    area_point_number area_point_numbers[10];    //任务区/映召点编号
}global_mission_planning_commands;


//4.10 单无人机指控命令
typedef struct
{
    unsigned int drone_id; // 无人机编号
    unsigned short drone_num;//无人机序号
    unsigned short mission_approach;//任务方式0=N/A;1=插入;2=覆盖;
    unsigned short mission_type;//任务类型
    unsigned short mission_object_type;//任务对象类型 0=N/A;1=任务区;2=目标;3=光标选点;4=航点;5=我方平台;
    unsigned int area_point_step_num ;//任务区/目标/平台编号
    longitude_and_latitude_synt waypoints_cursorSelection_longitude_and_latitude_synt;//航点/光标选点经纬度
}single_drone_charge_order;


//4.11 单任务区指控命令
typedef struct
{
    unsigned short mission_way;//任务方式  0=N/A;1=插入;2=覆盖;
    unsigned short mission_object_type;//任务对象类型  0=N/A;1=任务区;2=目标;3=光标选点;4=航点;5=我方平台;
    unsigned short mission_type;//0=N/A;1=浮标侦收;2=吊声定测（无人机不使用）;3=浮标布阵（无人机不使用）;4=通信中继;5=磁探搜索;6=磁探跟踪;7=光电搜索;8=光电跟踪;9=编队飞行;10=任务导航;11=返航;12=悬停等待;13=临时改航;14=盘旋等待;15=攻击
    unsigned int area_point_step_num;//任务区/目标/平台编号
    longitude_and_latitude_synt waypoints_cursorSelection_longitude_and_latitude_synt[2];//航点/光标选点经纬度
}single_task_area_allegation_order;


//4.12 无人机光电手柄控制量
typedef struct UAV_optical_handle_control_volume
{
    unsigned short head; //0X0414;
    unsigned short handle_state;//手柄状态
    unsigned short drone_control_status;//无人机控制状态
    unsigned int drone_num;//无人机平台编号
    unsigned short handle_button_information;//手柄按键信息
    unsigned int handle_orientation_vernier;//手柄方位游标量
    unsigned int handle_pitch_vernier;//手柄俯仰游标量
}UAV_optical_handle_control_volume;


//4.13 无人机光电视频MFD控制
//typedef struct uav_photoic_video_MFD
//{
//    unsigned short head; //0X0415;
//    unsigned short uav_xuhao;  //无人机序号 2
//    unsigned int uav_bianhao;  // 无人机编号 4
//    char hongwaishangdain[2];   // 红外上电 2
//    char graph_zengqiang[2];   // 图像增强 2
//    char zhenchachangjingshezhi[2]; // 侦查场景设置 2
//    char yijianjujiao[2]; // 一键聚焦 2
//    char hongwaitiaojiao[2]; //红外调焦 2
//}uav_photoic_video_MFD;
typedef struct {
//    unsigned char tv_infrared_switch; // 电视/红外切换 0H=电视;1H=红外
//    unsigned char long_short_switch; // 长焦/短焦切换 0H=长焦;1H=短焦
//    unsigned char spare0; // SPARE
    unsigned char type; // 电视/红外/长焦/短焦切换 0H=NA;1H=电视长焦;2H=电视短焦;3H=红外长焦;4H=红外短焦
    unsigned char lock_scan_switch; // 锁定/扫描切换 0H=NA;1H=锁定;2H=扫描
    unsigned char lock_param; // 锁定参数 0H=NA;1H=垂直下视锁定;2H=前向锁定;3H=右向锁定;4H=后向锁定;5H=左侧锁定
    unsigned char scan_param; // 扫描参数 0H=NA;1H=前向方位扫描;2H=右侧方位扫描;3H=后向方位扫描;4H=左侧方位扫描;5h=前向俯仰扫描;6H=右侧俯仰扫描;7H=后向俯仰扫描;8H=左侧俯仰扫描
    unsigned char ir_pwr_ctrl; // 红外上电控制 0=NA;1=上电;2=下电
#ifdef _SPARE_
    unsigned char spare[53];
#endif
} video_params_setting_feedback;
typedef struct {
    unsigned char video_source_type; // 视频源类型
    // 0H=NA;1H=有人机视频;2H=无人机1视频;3H=无人机2视频;4H=无人机3视频;5H=无人机4视频;
    video_params_setting_feedback video_params_setting_feedbacks; // 视频参数设置
} uav_photoic_video_MFD;

//4.14 无人机光电控制权设置
typedef struct uav_guangdiankongzhiquanshezhi
{
    unsigned short uav_kongzhiquanshezhi; //无人机光电控制权设置 2

}uav_guangdiankongzhiquanshezhi;

//4.15 预规划方案加载
//unsigned short head; //0X0417;

//4.16 预规划查询
typedef struct
{
	unsigned short plan_query;   /*预规划查询  0=NA;1=查询;*/
}BLK_OFP_CCC_032;/*预规划查询*/


//4.17 状态提示回执
typedef struct status_alert_receipt
{
    unsigned short head; //0X0419;
    unsigned short cue_message_water_num;//提示信息流水号
    unsigned short response_type;//响应方式
}status_alert_receipt;

//4.18 已投浮标信息
//typedef struct HXT_20F
//{

//}HXT_20F;

//typedef struct yitoufubiao_info
//{
//    unsigned short head; //0X0420;
//    int messageid = 0x9c005;
//    char message[64];
//    char FB_NO[4]; //
//    char SP_channel;
//    char FB_type;
//    char Work_frequency;
//    char FB_Lifetime[2];
//    char Work_Deep[2];
//    char Radio_Frequency;
//    char FB_Shape;
//    char CODAR_No;
//    char FB_distance;
//    char Location_Vaild;
//    char Entry_Latitude[8];
//    char Entry_Longitude[8];
//    //HXT_20F hxt_20f_info;
//}yitoufubiao_info;



/*********************037*********************/
typedef struct
{
    unsigned short	UAV_Serial_Number1;	/*平台序号*/
    unsigned int	UAV_ID2_2;			/*平台编号*/
    unsigned char  mfdNo;              /*mfd序号 0-NA 1-mfd1 2-mfd2 3-mfd3 4-mfd4 （1219已告知李翔改为1字节)*/
    unsigned short	QueryCommand;		/*查询页面号，0-(N-1)*/
}BLK_OFP_CCC_037;/*协同指控指令信息请求*/





/*********************040*********************/
// DPU1-CCC/MMM-040 链路参数控制指令(MessageID：0x062a28)
typedef struct {
    unsigned char TongBuZi1_1;//同步字1（域描述：0xEB）
    unsigned char TongBuZi2_1;//同步字2（域描述：0x90）
    unsigned char ZhenChang_1;//帧长（域描述：0x10）
    unsigned char Type_2;//类型（04H无人机记载链路控制，21H地面链路控制/有人机空空链链路控制）
    unsigned char Num_1;//序号/帧计数（0x00-0x7F循环）
    unsigned short BenjiID_1;//本机ID号
    unsigned short UavID_1;//被控设备ID号
    unsigned char KaiGuanLiang_1;//开关量指令码
    //域描述：
    //00H 空
    //03H 地面-单机
    //05H 地面-双机
    //06H 地面-中继
    //09H 启动副链交接
    //0AH C链交接成功确认
    //0CH 启动主链转移
    //30H
    //33H 全向天线
    //35H 定向天线
    //36H 机载-补盲天线
    //39H
    //3AH 机载-定向补盲自动切换
    //3CH C自动选频-关
    //3FH C自动选频-开
    //50H
    //53H C-工作
    //55H C-静默
    //56H C-大功率
    //59H C-小功率
    //5AH C-去调制
    //5CH C-误码测试
    //5FH C-功率测试（连续工作）
    //60H
    //63H UHF工作（退出所有测试状态）
    //65H UHF静默
    //66H UHF大功率
    //69H UHF小功率
    //6AH UHF-去调试
    //6CH UHF-系统误码测试
    //6FH UHF-功率测试
    //90H
    //93H UHF终止交接
    //95H UHF交接成功
    //96H
    //99H
    //9AH UHF明码
    //9CH UHF密码
    //9FH
    //A0H 密码指令解锁
    //A3H C明码
    //A5H C密码
    unsigned char LianXuLiang_1;//连续量指令码
    //描述域：
    //00H 空
    //03H 被控机载A-ID设置 机载ＩＤ号
    //05H 被控机载Ｂ-ID设置　机载ＩＤ号
    //06H 副链中继机ＩＤ设置　机载ＩＤ号
    //09H 主控站－ＩＤ设置　控制站ID号
    //0AH 接机站-ID设置 控制站ID号
    //30H
    //33H C下行工作频道 0x0000-0x0096 0-150
    //35H C下行交接频道 同上
    //36H C上行工作频道 0x0000-0x0032 0-50
    //39H C上行交接频道 同上
    //3AH C下行速率 H字节D7-D4：时隙分配 L字节D3-D0：速率控制
    //0x0000: TM
    //0x0001: 4M
    //0x0010: 8M
    //0x0011: 16M
    //3CH C上行速率
    //0x0000常规（单指控）
    //0x0001着舰引导
    //3FH UHF上行速率 H字节D7-D4：时隙分配 L字节D3-D0：速率控制
    //0x0000: 25.6kbps
    //0x0001: 51.2kbps
    //0x0010: 102.4kbps
    //63H UHF工作频道 频道：0-50
    //6AH UHF接机站频道 同上
    unsigned char LianXuLiangL_1;//连续量数据L
    unsigned char LianXuLiangH_1;//连续量数据H
    unsigned short CRC16_1;//CRC16 域描述：校验4-13字节共10字节
    unsigned char HeJiaoYan_1;//和校验 域描述：从EB90开始
} BLK_OFP_CCC_040;//链路参数控制指令


typedef struct
{
      unsigned short MAINT_REQ:8;   /*维护请求  0=N/A;1=状态;2=显示;*/
      unsigned short MFL_PAGE_TURNING:8;   /*维护故障清单翻页  0=N/A;1=上一个故障清单;2=下一个故障清单;3=第一个故障清单;*/
}SIG_DPU1_CCC_MAINT_REQ_KKL;  /*维护清单请求和翻页 描述:当进入分系统或设备的维护主画面时，维护故障清单请求=1；
当进入子分系统或设备的维护故障清单画面时，维护故障清单请求=2，维护故障清单翻页=3
*/

typedef struct
{
    unsigned int drone_id; // 无人机编号
    unsigned short drone_num;//无人机序号
    unsigned char tg;           //探杆 0-NA，1-伸出，2-缩回
    unsigned char gd;           //光电  20250819new临时修改为返航 0-NA，1-返航
    unsigned char ct;           //磁探
    unsigned char fb;           //浮标
    unsigned char spare[8];
}BLK_OFP_CCC_045;//磁探杆伸缩指令


/*********************100/101*********************/
typedef struct
{
      unsigned short MFL_TEST:8;   /*维护自检测  0=N/A;1=开始;2=停止;*/
      unsigned short MFL_DELETE:8;   /*删除控制  0=N/A;1=删除当前显示故障清单;2=删除全部故障清单;*/
}SIG_DPU1_CCC_MAINT_CMD_KKL;  /*维护自检控制和维护清单删除*/

typedef struct
{
      SIG_DPU1_CCC_MAINT_REQ_KKL MAINT_REQ_CCC;      /*维护清单请求和翻页  信号描述：当进入分系统或设备的维护主画面时，维护故障清单请求=1；
当进入子分系统或设备的维护故障清单画面时，维护故障清单请求=2，维护故障清单翻页=3
*/
      SIG_DPU1_CCC_MAINT_CMD_KKL MAINT_CMD_CCC;      /*维护自检控制和维护清单删除*/
}BLK_OFP_CCC_100;/*维护控制指令  协同指控计算机*/


typedef struct
{
      SIG_DPU1_CCC_MAINT_REQ_KKL MAINT_REQ_CCC;      /*维护清单请求和翻页  信号描述：当进入分系统或设备的维护主画面时，维护故障清单请求=1；
当进入子分系统或设备的维护故障清单画面时，维护故障清单请求=2，维护故障清单翻页=3
*/
      SIG_DPU1_CCC_MAINT_CMD_KKL MAINT_CMD_CCC;      /*维护自检控制和维护清单删除*/
}BLK_OFP_CCC_101;/*维护控制指令   U链端机*/


typedef struct
{
	unsigned short uav_num;         //无人机序号
	unsigned int uav_id;            //无人机ID
	unsigned short releaseCtrl; 	//手动释放	0=NA 1=开始控制权交接 2=主链转移
	unsigned short getCtrl;			//手动获取	0=NA 1=手动获取控制权
}BLK_OFP_CCC_044;


typedef struct
{
	unsigned short confirm; 	// 语音识别确认执行指令  0-NA；1-执行；2-取消
	unsigned short control;			// 语音识别控制 0-NA；5F5F-启动识别（指控接收到5F5F启动识别，aaaa停止识别）251007
}BLK_OFP_CCC_047;

/************** 053 *****************/
typedef struct
{
	unsigned int uavCode;		/*无人机平台编号*/
	unsigned short uavSn;		/*无人机序号*/
	unsigned char cmd; 			/*0-NA 1-开始领航  2-退出领航*/
}BLK_OFP_CCC_053; //单无人机指控 编队飞行  开始/退出领航的命令  20250909new 0x62a35

/************** 148 *****************/
typedef struct
{
	unsigned short uavSn;		/*无人机序号*/
	unsigned int uavCode;		/*无人机平台编号*/
	unsigned char op_code;		/*0-NA 1-查询 2-调高 3-调速*/
	unsigned char response; 	/*0-NA 1-航点查询失败 2-航点查询成功 3-遥调中 4-遥调成功 5-遥调失败 6-遥调超时 7-已过点 8-已过点，无下一航点*/
	unsigned char pointNum;		/*航路点序号*/
	double  lati;				/*航路点纬度*/
	double  longi;				/*航路点经度*/
	char 	pointType;			/*航路点类型   */
	float   height;				/*航路点高度 米       */
	float   speed;				/*航路点速度 km/h */
}BLK_OFP_CCC_148;//指令微调 20250909new 0x62a94





/************** 154 *****************/
typedef struct
{
	/*主动提示的操作  0-NA
	1-menu1执行(发布重规划航线（立即给ofp反馈032块21发布中；成功后反馈032块0；失败反馈032块13）)
	2-menu1(忽略重规划航线， 发布原有航线（收到清空冲突状态，给ofp反馈032块0）)
	3-menu2执行(继续任务，冲突解除后发布后续航线到uav（立即给ofp反馈032块21发布中；成功后反馈032块0；失败反馈032块14）)
	4-menu2(忽略冲突消解（收到清空冲突状态，给ofp反馈032块0）)
	5：退出航线重规划状态（当发布规避航线时，发布失败后的恢复，收到清空冲突状态）
	6：退出冲突消解（当发布消解航线时，发布失败后的恢复，收到清空冲突状态）
	11：执行载荷重规划（收到清空冲突状态，给ofp反馈032块类型32，包含任务类型和区域信息）
	12：忽略载荷重规划(停止周期检测载荷在线状态)*/
	unsigned char plan_query;
}BLK_OFP_CCC_154; /*重规划控制指令*/

/************** 155 *****************/
typedef struct
{
	unsigned short uavSn;		/*无人机序号*/
	unsigned int uavCode;		/*无人机平台编号*/
	unsigned char opcode;		/*编队能力控制 0-不具备 1-具备*/
}BLK_OFP_CCC_155;//编队能力控制 20251208new 0x62a9b

/************** 156 *****************/
typedef struct
{
	unsigned short uavSn;		/*无人机序号*/
	unsigned int uavCode;		/*无人机平台编号*/
}BLK_OFP_CCC_156;//航线查询命令 20251218new 0x062a9c


#pragma pack()


#endif // SYNTHESIZECOOPERATIVEALLEGATIONCOMPUTER_H

