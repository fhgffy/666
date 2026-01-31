/*
 * DPM1A.h
 *
 *  Created on: 2024年11月7日
 *      Author: yang_rui
 */

#ifndef DPM1A_H_
#define DPM1A_H_
#include "icd_ccc_ofp.h"

#pragma pack(1)
// CCC->DPM1A   20241105


// 无人机状态信息
typedef struct DPM_uav_info{
    unsigned short uav_data_valid;		/*无人机数据有效标志位  diff*/
    double station_lati;				/*地面站纬度    单位：°  最小值：-90  最大值：90*/     // 无
    double station_longi;				/*地面站经度    单位：°  最小值：-180  最大值：180*/   // 无
    double uav_lati;					/*无人机纬度    单位：°  最小值：-90  最大值：90*/
    double uav_longi;					/*无人机经度    单位：°  最小值：-180  最大值：180*/
    unsigned int radio_height;			/*无人机无线电高度*/
    unsigned int air_height;			/*无人机气压高度*/
    unsigned int air_speed;				/*无人机空速*/
    unsigned int ground_speed;		    /*无人机地速*/
    unsigned int vectx_speed;			/*无人机垂直速度*/
    unsigned int uav_heading;			/*无人机航向*/
    unsigned int uav_pitch;				/*无人机俯仰角*/
    unsigned int uav_roll;				/*无人机横滚角*/
    /*威胁信息*/
    unsigned short uav_icon_flag;					/*无人机提示图标显示标志位  0=无效;1=显示;2=不显示;*/
    unsigned short uav_height_trend;				/*无人机高度变化趋势  0=无效;1=向上;2=向下;*/
    double uav_copter_angle;						/*无人机相对有人机方位角    单位：°  最小值：-180  最大值：180*/
    unsigned int uav_copter_distance;				/*无人机相对有人机距离*/
    unsigned short uav_angle_change;				/*无人机方位角变化趋势  0=无效;1=顺时针;2=逆时针 ;*/
    unsigned char Threat_UAV_Warning_2;				/*威胁无人机告警标志位*/
    unsigned short Thread_UAV_Horizontal_Position_2;/*威胁无人机水平相对位置*/
    unsigned short Thread_UAV_Vertical_Position_2;	/*威胁无人机垂直相对位置*/

} DPM_uav_info;

typedef struct DPM_uav_mission_info{
    unsigned int task_id;			/*无人机当前任务ID*/
        unsigned short mission_status;	/*当前任务状态  0=N/A;1=正在执行任务;2=任务执行完毕;*/
        unsigned short mission_progress;/*当前任务进度*/

} DPM_uav_mission_info;

typedef struct DPM_drone_specific_information
{
    unsigned short platform_model;//平台型号
    unsigned short platform_serial_num;//平台序号
    unsigned int platform_num;//平台编号
    unsigned short platform_control_status;//平台控制权状态     4A 站地址判断
    unsigned short platform_load;//平台载荷                   无  是否任务帧存在
    //平台链路状态 Platform Link Status(待定)
    unsigned short data_valid_bit;//数据有效位
    unsigned short  fault_level; //平台故障状态                 // 4A系统故障等级 同映射关系
    unsigned short fault_list;  //平台故障清单                 无  4A  4B *（飞行 机电相关故障）
    unsigned short subtask_type;//平台当前执行子任务类型
    unsigned int remaining_mission_time;//平台剩余任务时间       // 按分钟发送
    unsigned int residual_oil_volume;//平台剩余油量             // 总油箱
    short Ng; //Ng
    short Nr;//Nr                                           // 4c /旋翼转速
    short T45;//T45                                         //T45温度计算值
    short U;//U                // 不确定     5
    short U_storage;//U蓄      // 不确定
    int uav_zishu_feedback;// 无人机自主反馈
    DPM_uav_info uav_infos; // 无人机飞行信息
    DPM_uav_mission_info uav_mission_infos; // 无人机任务状态信息
} DPM_drone_specific_information;//无人机具体信息

typedef struct DPM_drone_state_information
{
    // unsigned short head = 0X0101;
    unsigned short drone_number;//无人机个数n
    DPM_drone_specific_information drone_specific_informations[4];//无人机具体信息
}DPM_drone_state_information;



// 编队链路状态信息
typedef struct DPM_drone_link_status_information//无人机链路状态信息
{
    unsigned short uav_model;			/*平台1型号  0=N/A;1=WZ2;*/  //默认 1
    unsigned short uav_sn;					/*平台序号*/
    unsigned int   uav_code;				/*平台编号*/   // 4097 4098 4099 4100 （实装修改为动态绑定）   4A
    unsigned short uav_ulink_chanel;		/*平台U链频道号*/            // 无
    unsigned short uav_clink_down_channel;  /*平台C链下行频道号*/         // 无
    unsigned short uav_clink_up_channel;	/*平台C链上行频道号*/         // 无
    //unsigned short uav_u_strength;			/*平台U链信号强度*/
    //unsigned short uav_c_strength;			/*平台C链信号强度*/         // 无
    unsigned char u_send_strength;
    unsigned char u_recv_strength;
    unsigned char c_send_strength;
    unsigned char c_recv_strength;
    unsigned int   handover_id;				/*接机站ID*/  //          // 无
    unsigned short handover_up_channel;		/*交接上行频道*/           // 待确定
    unsigned short handover_down_channel;	/*交接下行频道*/           // 待确定
    unsigned short handover_down_speed;		/*交接下行速率*/           // 待确定

}DPM_drone_link_status_information;

typedef struct DPM_ControlledUavId
{
    unsigned int uav_code;
}DPM_ControlledUavId;

typedef struct DPM_jiemianzhikong
{
    unsigned int station_id;			/*舰面站ID*/         //   空地链  找
    unsigned short station_sn;			/*舰面站序号 0:NA, 1:舰面站1 2:舰面站2*/
    unsigned short controll_uav_num;	/*指控无人机数量*/
    DPM_ControlledUavId ctrl_uav_code[2];	/*控制无人机的编号*/
    unsigned short u_channel;			/*平台U链频道号*/
    unsigned short u_strength;			/*平台U链信号强度*/
    unsigned short c_up_channel;		/*平台C链上行频道号*/
    unsigned short c_down_channel;		/*平台C链下行频道号*/
    unsigned short c_strength;			/*平台C链信号强度*/
    unsigned short bk_strength;			/*编宽链信号强度*/

}DPM_jiemianzhikong;

typedef struct DPM_NETWORK_MEMBER
{
   unsigned short U_member;		/*U链在网成员*/
   unsigned short C_UAV_F;			/*C链联通情况（有人/无人）*/
   unsigned short C_Station_UAV;   /*C链联通情况（地面/无人）*/
}DPM_NETWORK_MEMBER;/*网络拓扑信息*/

typedef struct DPM_CopterLinkStatus
{
    unsigned int code;				/*平台编号*/
    unsigned short u_channel;		/*U链频道*/
    unsigned short u_power;			/*U链功率  0=N/A;1=大功率;2=小功率 ;*/
    unsigned short u_rate;			/*U链速率  0=N/A;1=25.6;2=51.2;3=102.4;*/
    unsigned short u_silent_flag;	/*U链静默  0=N/A;1=静默;2=非静默 ;*/
    unsigned short u_encrypt;		/*U链加密方式  0=N/A;1=明文;2=密文;*/
    unsigned short c_up_channel;	/*平台C链上行频道号*/
    unsigned short c_down_channel;	/*平台C链下行频道号*/
    unsigned short c_up_power;		/*C链上行功率  0=N/A;1=大功率;2=小功率 ;*/
    unsigned short c_down_power;	/*C链下行功率  0=N/A;1=大功率;2=小功率 ;*/
    unsigned short c_up_rate;		/*C链上行速率  0=N/A;1=25.6;2=51.2;3=102.4;*/
    unsigned short c_down_rate;		/*C链下行速率  0=N/A;1=2;2=4;3=8;*/
    unsigned short c_silent;		/*C链静默  0=N/A;1=静默;2=非静默 ;*/
    unsigned short c_encrypt;		/*C链加密方式  0=N/A;1=明文;2=密文;*/
    //unsigned short bk_link_established;	/*（不使用）编宽链建立标志  0=NA;1=建立;2=未建立 ;*/
    unsigned char bk_link_established1;		/*编宽链建立舰面站1标志  0=NA;1=建立;2=未建立 ;*/
    unsigned char bk_link_established2;		/*编宽链建立舰面站2标志  0=NA;1=建立;2=未建立 ;*/
    DPM_NETWORK_MEMBER network_member;			/*网络拓扑信息*/
}DPM_CopterLinkStatus;/*有人机链路状态*/

typedef struct DPM_formation_link_status_information
{
    // unsigned short head = 0X0102;
    unsigned short drone_number;//无人机个数n
    DPM_drone_link_status_information drone_link_status_informations[4];//无人机链路状态信息
    unsigned short surface_station_num;  //舰面站个数m
    DPM_jiemianzhikong jiemianzhikong_info[2];  //舰面站指控状态    // 无
    DPM_CopterLinkStatus copter_link_info;	/*有人机链路状态  diff*/
}DPM_formation_link_status_information;


// 综合态势(目标融合)
typedef struct DPM_goal_information//目标信息
{
    unsigned short tgt_sn;		/*目标序号*/
        unsigned short tgt_type;	/*目标类型  0=NA;1=空中;2=陆地;3=水面;4=水下;*/
        unsigned short tgt_source;	/*目标来源  0=NA;1=有人机数据链;2=无人机光电;3=有人机后舱水声;*/
        double tgt_lati;			/*目标纬度    单位：°  最小值：-90  最大值：90*/
        double tgt_longi;			/*目标经度    单位：°  最小值：-180  最大值：180*/
        unsigned int tgt_height;	/*目标高度*/
        unsigned int tgt_speed;		/*目标速度*/
        unsigned int tgt_heading;   /*目标航向*/
        unsigned short tgt_property;/*目标属性  0=NA;1=敌;2=我;3=友;4=中立;5=不明;*/
        unsigned int tgt_code;		/*目标批号*/
        unsigned short tgt_data_valid; /*目标数据有效位*/
}DPM_goal_information;

typedef struct DPM_integrated_posture
{
    // unsigned short head = 0X0103;
    unsigned short tgt_Number;			/*目标个数n*/
    DPM_goal_information goal_informations[25];		/*目标信息*/

}DPM_integrated_posture;


// 任务分配结果信息
//tip: 3个方案，5个平台，8个任务（当接收全局任务规划指令时，会同时生成3个方案，按方案进行分包发送。）
typedef struct DPM_task_sequence_information//任务序列信息
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
}DPM_task_sequence_information;

typedef struct DPM_formation_synergy_mission_program//编队任务协同方案
{
    unsigned short platform_model;   /*平台1型号  0=N/A;1=20F;2=WZ2;*/
    unsigned short platform_sn;   /*平台序号*/
    unsigned int platform_code;   /*平台编号*/
    unsigned int platform_task_time;   /*任务平台任务时长*/
    unsigned short platform_subtask_number;   /*任务平台子任务个数m*/
    DPM_task_sequence_information task_sequence_informations[8];//任务序列信息（循环8次）

}DPM_formation_synergy_mission_program;

typedef struct DPM_information_on_the_results_of_tasking
{
    // unsigned short head;
    unsigned int plan_id;                   /*方案编号*/
    unsigned short plan_release_mode;		/*任务分配方案发布  0=N/A;1=发布;2=首次生成;3=修改;4=取消;*/
    unsigned short plan_manual_amend_flag;  /*任务分配方案是否人工修改  0=N/A;1=修改;2=未修改;*/
    unsigned short plan_replanning_flag;	/*任务分配方案是否自动重规划  0=N/A;1=是;2=否;*/
    unsigned short plan_altermode;			/*任务分配方案修改方式  0=N/A;1=子任务修改;2=航线修改;*/
    unsigned short plan_attribute;			/*方案属性  0=NA;*/
    unsigned short submarine_search_mode;   /*协同搜潜方式  0=均匀搜索;1=协同包围;2=协同拦阻;*/
    unsigned int   plan_total_time;			/*方案总时间*/
    unsigned short platform_num;			/*任务平台个数n*/
    DPM_formation_synergy_mission_program formation_synergy_mission_programs[5];//编队协同任务方案（循环5次）
}DPM_Information_the_results_of_tasking;


// 有人机航路规划结果信息
typedef struct DPM_suspended_sound_buoy_deployment_point_information
{
   unsigned char JingduUpper;//吊声定测点经度有效性
   unsigned char WeiduUpper;//纬度有效性
   unsigned char HighlyUpper;//深度有效性
   longitude_and_latitude point_longitude_and_latitude;//浮标布阵点经纬度
   double profundity ;//浮标布阵点深度
}DPM_suspended_sound_buoy_deployment_point_information;//浮标布阵点信息

typedef struct DPM_suspended_sound_manned_computer_task//有人机子任务
{
   unsigned int subtask_ID_number ;//子任务ID
   unsigned short fixed_measurement_points_number;//定测点总数
   DPM_suspended_sound_buoy_deployment_point_information suspended_sound_buoy_deployment_point_informations[25];
}DPM_suspended_sound_manned_computer_task;

typedef struct DPM_suspended_sound_fixed_measurement_point//吊声定测点
{
   // unsigned short head = 0X0108;
   unsigned int program_number;//方案编号
   unsigned short subtasks_number;//有人机子任务个数m
   DPM_suspended_sound_manned_computer_task suspended_sound_manned_computer_tasks;//有人机子任务（循环8次）
}DPM_suspended_sound_fixed_measurement_point;

typedef struct DPM_buoy_deployment_point_information//浮标布阵点信息
{
   unsigned short matching_tube_number;//浮标布阵点匹配筒位号
   unsigned short buoy_type  ;//浮标布阵点浮标类型
   unsigned char longitude_validity;//浮标布阵点经度有效性
   unsigned char latitude_validity;//浮标布阵点维度有效性
   longitude_and_latitude buoy_longitude_and_latitude;//浮标布阵点经纬度

}DPM_buoy_deployment_point_information;

typedef struct DPM_buoy_manned_computer_task//有人机子任务
{
   unsigned int subtask_ID_number ;//子任务ID号
   unsigned short points_number ;//浮标布阵点总数
   DPM_buoy_deployment_point_information buoy_deployment_point_informations[25];//浮标布阵点信息
}DPM_buoy_manned_computer_task;

typedef struct DPM_buoy_deployment_point//浮标布阵点
{
   // unsigned short head = 0X0107;
   unsigned int program_number;//方案编号
   unsigned short subtasks_number;//有人机子任务个数m
   DPM_buoy_manned_computer_task buoy_manned_computer_tasks;//有人机子任务
}DPM_buoy_deployment_point;



typedef struct DPM_waypoint_information
{
   unsigned short type;//航路点类型
   unsigned char validity_of_longitude;//航路点经度有效性
   unsigned char latitude_validity;//航路点维度有效性
   unsigned char height_validity;//航路点高度有效性
   unsigned char speed_validity;//航路点速度有效性
   unsigned char direction_validity;//航路点航向有效性
   unsigned char time_validity;//航路点时间有效性
   unsigned char payloads_validity;//航路点载荷有效性
   longitude_and_latitude waypoint_longitude_and_latitude;//航路点经纬度
   unsigned int height;//航路点高度
   unsigned int speed;//航路点速度
   unsigned int direction;//航路点航向
   unsigned int time;//航路点时间
   unsigned short payloads;//航路点载荷
}DPM_waypoint_information;//航路点信息

typedef struct DPM_common_carrier_route//通用航路
{
   // unsigned short head = 0X0106;
   unsigned int   plan_code;					/*方案编号*/
   unsigned short subtask_cnt;					/*有人机子任务个数m*/
   unsigned int   subtask_index;				/*子任务ID号*/
   unsigned short airway_point_num;			/*航路点个数n    最小值：0  最大值：40*/
   unsigned short airway_point_start_num;		/*航路点起始编号*/
   DPM_waypoint_information waypoint_informations[40];//航路点信息
}DPM_common_carrier_route;

typedef struct DPM_manned_aircraft_route_drone_route_information
{
   DPM_common_carrier_route common_carrier_routes;//通用航路
   DPM_buoy_deployment_point buoy_deployment_points;//浮标布阵点
   DPM_suspended_sound_fixed_measurement_point suspended_sound_fixed_measurement_points;//吊声定测点
}DPM_manned_aircraft_route_drone_route_confirmation_information;



// 无人机航路规划
typedef struct DPM_planning_information_waypoint_information//航路点信息
{
    DPM_waypoint_information waypoint_informations;//航路点信息
    unsigned short causality;//航路点属性
    unsigned short standby_type;//航路点待机类型
    unsigned char standby_time_lapsNumber_cycleNumber_valid_bit;//航路点待机时间/圈数/循环次数有效位
    unsigned char standby_radius_valid_bit;//航路点待机半径有效位
    unsigned int standby_time_lapsNumber_cycleNumber;//航路点待机时间/圈数/循环次数
    unsigned int standby_radius;//航路点待机半径


}DPM_planning_information_waypoint_information;

typedef struct DPM_planning_information//单无人机单序列中的规划信息
{
    unsigned int subtask_ID_number;//子任务ID号
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
    DPM_planning_information_waypoint_information planning_information_waypoint_informations[25];//航路点信息

}DPM_planning_information;

typedef struct DPM_individual_drone_routing_program//单个无人机航路方案
{
    unsigned short drone_serial_number;//无人机序号
    unsigned int drone_num;//无人机编号
    unsigned short subtasks_number ;//当前无人机子任务个数
    unsigned short subtask_index; // 从0开始
    DPM_planning_information planning_informations;
}DPM_individual_drone_routing_program;

typedef struct DPM_drone_route_confirmation
{
    // unsigned short head = 0X0109;
    unsigned int program_number;//方案编号
    unsigned short number_of_drones;//无人机数量
    DPM_individual_drone_routing_program individual_drone_routing_programs;//单个无人机航路方案
}DPM_drone_route_confirmation;





// 协同任务执行状态提示
typedef struct DPM_collaborative_task_execution_status_alerts
{
    // unsigned short head = 0X0110;
    unsigned int UAV_Serial_Number;//提示无人机编号
    unsigned short cue_drone_serial_number;//提示无人机序号
    unsigned short cue_message_water_number;//提示信息流水号
    unsigned short subtask_type;//无人机当前执行子任务类型
    unsigned short point_area_type;//子任务任务点/区域类型
    unsigned int area_point_line_goal_number;//子任务的任务区/点/线/目标编号
//    char area_point_line_goal_number_valid_bit;//子任务的任务区/点/线/目标编号字段有效位
    unsigned short cue_message_type;//提示信息类型
}DPM_collaborative_task_execution_status_alerts;


// 任务区/空域信息
typedef struct DPM_cycle
{
    longitude_and_latitude cycle_longitude_and_latitude;//圆心经纬度
    float radius;//圆心半径
}DPM_cycle;

typedef struct DPM_polygonal//多边形
{
    unsigned short point_number;//点数n
    point_coordinate point_coordinates[10];//点坐标

}DPM_polygonal;

typedef struct DPM_area_information//区域信息
{
    unsigned int area_code;//区域编号
    unsigned short area_type;//区域类型
    unsigned short area_source;//区域来源
    unsigned short area_shape;//区域形状

    unsigned short area_platform_num;//区域所属平台
//    unsigned short drone_number_valid_bit;//空域所属无人机编号有效位
    unsigned int drone_numbe;//空域所属无人机序号
    unsigned char upper_height_limit_valid_bit;//区域高度上限有效位置
    unsigned char lower_height_limit_valid_bit;//区域高度下限有效位置
    float upper_height_limit;//区域高度上限
    float lower_height_limit;//区域高度下限
    DPM_cycle cycles;//圆
    DPM_polygonal polygonals;//多边形

}DPM_area_information;

typedef struct DPM_area_sky_information
{
    // unsigned short head = 0X0111;
    unsigned short area_number;//区域个数
    DPM_area_information area_informations[8];//区域信息
}DPM_area_sky_information;


// 任务点信息
typedef struct DPM_point_information//点信息
{
    unsigned int point_number;//点编号
    unsigned short point_source_of_information;//点信息来源
    longitude_and_latitude point_longitude_and_latitude;//点经纬度
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

}DPM_point_information;

typedef struct DPM_taks_point_information
{
    // unsigned short head = 0X0112;
    unsigned short point_number;//任务点个数
    DPM_point_information point_informations[30];//点信息
}DPM_taks_point_information;



// 任务线信息
typedef struct DPM_line_information
{
    unsigned int line_num;//线编号
    unsigned short line_type;//线类型
    unsigned short line_point_number;//任务线点个数
    point_coordinate point_coordinates[10];//点经纬度

}DPM_line_information;

typedef struct DPM_task_line_information
{
    // unsigned short head = 0X0113;
    unsigned short line_number;//任务线条数
    DPM_line_information line_informations[5];//线信息
}DPM_task_line_information;


// 无人机光电控制权反馈
typedef struct DPM_guangdian_kongzhi_feedback
{

    unsigned short ssds_control_feedback;   /*无人机光电控制权反馈  0=NA;1=前舱;2=后舱;*/
}DPM_guangdian_kongzhi_feedback;/*无人机光电控制权反馈*/


// 视频参数设置反馈
typedef struct DPM_video_param_setting_feedback {
    char tv_infrared_switch; // 电视/红外切换
    char long_short_switch; // 长焦/短焦切换
    char lock_scan_switch; // 锁定/扫描切换
    char lock_param; // 锁定参数
    char scan_param; // 扫描参数
    char spare[53]; // SPARE
} DPM_video_param_setting_feedback;

// 光电状态反馈
typedef struct DPM_photoelectric_state_feedback {
    float photoelectric_platform_azimuth; // 光电平台方位角
    float photoelectric_platform_pitch; // 光电平台俯仰角
    unsigned short target_param_significant_bit; // 目标参数有效位
    float target1_dist; // 目标1距离
    float target2_dist2; // 目标2距离
    char spare[94]; // SPARE
} DPM_photoelectric_state_feedback;

// 无人机光电视频控制反馈
typedef struct DPM_guangdina_video{
    char video_source_type; // 视频源类型
    DPM_video_param_setting_feedback DPM_video_param_setting_feedbacks; // 视频参数设置反馈
    DPM_photoelectric_state_feedback DPM_photoelectric_state_feedbacks; // 光电状态反馈
}DPM_guangdina_video;

// 无人机光电视频控制指令
typedef struct DPM_guangdina_video_instruction{
    char video_source_type; // 视频源类型
    DPM_video_param_setting_feedback DPM_video_param_setting_feedbacks; // 视频参数设置反馈
}DPM_guangdina_video_instruction;

#pragma pack()
#endif /* DPM1A_H_ */
