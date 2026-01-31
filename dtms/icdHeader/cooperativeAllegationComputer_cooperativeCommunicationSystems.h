#ifndef COOPERATIVEALLEGATIONCOMPUTER_COOPERATIVECOMMUNICATIONSYSTEMS_H
#define COOPERATIVEALLEGATIONCOMPUTER_COOPERATIVECOMMUNICATIONSYSTEMS_H
///*
// *协同指控计算机-协同通信系统
// * */
//
//
//
////经纬度
//typedef struct cocolongitude_and_latitude
//{
//    double longitude;//经度
//    double latitude;//维度
//
//}colongitude_and_latitude;
//typedef struct copoint_coordinate//点坐标
//{
//    colongitude_and_latitude point_colongitude_and_latitude;//点经纬度
//}copoint_coordinate;
//
//
////1.1 无人机1光电视频
//typedef struct drone_1_photovoltaic_video
//{
//    unsigned int head;
//    char synchronization_code[2];//同步码 2
//    char frame_class;//帧类别 1
//    char airplane_ID[2];//飞机ID 2
//    char video_type;//视频类型 1
//    char frame_length[2];//帧长 2
//    char video[1000];//无人机1视频数据内容（包头＋单元区域视频数据）video_data_content
//}drone_1_photovoltaic_video;
//
//
////1.2 无人机2光电视频
//typedef struct drone_2_photovoltaic_video
//{
//    unsigned int head;
//    char synchronization_code[2];//同步码 2
//    char frame_class;//帧类别 1
//    char airplane_ID[2];//飞机ID 2
//    char video_type;//视频类型 1
//    char frame_length[2];//帧长 2
//    char video[1000];//无人机2视频数据内容（包头＋单元区域视频数据）video_data_content
//}drone_2_photovoltaic_video;
//
//
////1.3 编队综合态势
//typedef struct cogoal_information//目标信息
//{
//    unsigned int goal_num;//目标序号
//    char goal_type;//目标类型
//    // char goal_source;//目标来源 // 8.3版本文档,无这条信息
//    colongitude_and_latitude goal_colongitude_and_latitude;//目标经纬度
//    double goal_height;//目标高度
//    double goal_speed;//目标速度
//    double goal_direction;//目标航向
//    double goal_attributes;//目标属性
//    double goal_lot_num;//目标批号
//    char data_valid_flag_bit;//目标数据有效位
//}cogoal_information;
//typedef struct data_fusion//数据融合
//{
//    unsigned short goal_number;//目标数目
//    cogoal_information goal_informations[25];//目标信息
//}data_fusion;
//typedef struct unmanned_helicopter_flight_information//无人直升机1飞行信息
//{
//    float ground_control_station_longitude;//地面控制站经度
//    float ground_control_station_latitude;//地面控制站维度
//    char control_attribution;//控制权归属
//    float unmanned_helicopter_longitude;//无人直升机经度
//    float unmanned_helicopter_latitude;//无人直升机维度
//    unsigned short barometric_height;//气压高度
//    unsigned short satellite_height;//卫星高度
//    unsigned short radio_height;//无线电高度
//    unsigned short absolute_height;//绝对高度
//    unsigned short relative_height;//相对高度
//    char vertical_velocity;//垂直速度
//    unsigned short vacuum_speed;//真空速
//    unsigned short body_axis_longitudinal_speed;//机体轴纵向速度
//    unsigned short body_axis_lateral_ground_velocity;//机体轴侧向地速
//    unsigned short tilt;//俯仰角
//    unsigned short roll_angle;//滚转角
//    unsigned short hangxiang_angle; // 航向角
//    unsigned short flight_path;//应飞航向
//    char current_flight_path_num;//当前飞行航线号
//    char current_flight_waypoint_num;//当前飞行航点号
//    // 以下有具体信号注解
//    char first_level_control_state;//第一级控制状态
//    char second_level_control_state;//第二级控制状态
//    char emergency_failure_disposal_mode;//应急故障处置模式
//    char emergency_troubleshooting_program;//应急故障处置方案
//    char flight_subphase;//飞行子阶段
//    char standby;//备用
//}unmanned_helicopter_flight_information;
//typedef struct manned_helicopter_flight_information//有人直升机飞行信息
//{
//    colongitude_and_latitude manned_helicopter_colongitude_and_latitude;//有人机经纬度
//    float manned_helicopter_height;//有人机高度
//    float manned_helicopter_speed;//有人机速度
//    float manned_helicopter_direction;//有人机航向
//    float manned_helicopter_tilt;//有人机俯仰角
//    float manned_helicopter_roll_angle;//有人机横滚角
//    unsigned short manned_helicopter_valid_flag_bit;//有人机数据有效位
//    //备份(待定)
//}manned_helicopter_flight_information;
//typedef struct integrated_formation_posture
//{
//    //unsigned short head;
//    char synchronization_code[2];//同步码 2
//    char frame_class;//帧类别 1
//    manned_helicopter_flight_information manned_helicopter_flight_informations;//有人直升机飞行信息
//    unmanned_helicopter_flight_information unmanned_helicopter_flight_informations[2];//无人直升机1、2飞行信息
//    data_fusion data_fusions;//数据融合
//    char standby;//备用
//    char checksum;//校验和
//}integrated_formation_posture;
//
//
//
//
//
////1.4 协同数据链状态信息数据帧
//typedef struct collaborative_data_chain_status_message_data_frame
//{
//    unsigned int head;
//    char synchronization_code[2];//同步码 2
//    char frame_class;//帧类别 1
//    char Uchain_members_in_network[2];//U链在网成员 2
//    char Cchain_connectivity_manned_unmanned;//C链联通情况（有人/无人）  1
//    char Cchain_connectivity_ground_unmanned;//C链联通情况（地面/无人）  1
//    char reserve[9];//预留  9
//    char checksum;//校验和 1
//}collaborative_data_chain_status_message_data_frame;
//
//
////1.5 空地链链路遥测信息
//typedef struct UHF//空地链链路遥测信息数据内容定义表（UHF）
//{
//    unsigned int head;
//    char synchronized_characters1;//同步字 1
//    char synchronized_characters2;//同步字 1
//    char frame_length;//帧长 1
//    char type;//类型 1
//    char serial_number_frame_count;//序号/帧计数 1
//    char local_device_ID[2];//本机设备ID 2
//    char communication_object_ID[2];//通信对象ID 2
//
//    char communication_mode;//通信模式 1
//    char UTC_time[3];//UTC时间 3
//    char coordinate_valid_flags;//坐标有效标志 1
//    char native_coordinates[2];//本机坐标 12
//    char reserve1;//备用 1
//    char wideband_transmission_C_transceiver_combination_status_information[1000];//宽带传输C收发组合状态信息 待定
//    char C_band_RF_front_end_status[1000];//C波段射频前端状态  待定
//    char forward_transmitting_frequency;//前端发射频点 1
//    char Received_signal_amplitude1;//接收信号幅度 1
//    char antenna_operating_status[1000];//天线工作状态 待定
//    char reserve2;//备用 1
//    char reserve3;//备用 1
//    char master_link_handover_state;//主链交接状态 1
//    char state;//状态 1
//    char version_number1;//版本号 1
//    char C_uplink_handover_channel_number;//C上行交接频道号 1
//    char reserve4;//备用 1
//    char uplink_rate_frequency_points1;//上行速率、频点 1
//    char Received_signal_amplitude2;//接收信号幅度 1
//    char channel_BER1;//信道误码率 1
//    char downlink_speed1;//下行速率 1
//    char version_number2;//版本号 1
//    char C_downlink_handover_channel_number;//C下行交接频道号 1
//    char reserve5;//备用 1
//    char uplink_rate_frequency_points2;//上行速率、频点 1
//    char Received_signal_amplitude3;//接收信号幅度 1
//    char channel_BER2;//信道误码率 1
//    char downlink_speed2;//下行速率 1
//    char version_number3;//版本号 1
//    char C_handover_station_communication_mode;//C交接站通信模式 1
//    char secondary_chain_handover_mode;//副链交接模式 1
//    char UHF_band_transceiver_combination_state;//UHF波段收发组合状态 1
//    char reserve6;//备用 1
//    char reserve7;//备用 1
//    char temp;//温度 1
//    char interface_status;//接口状态 1
//    char reception_state;//接收状态 1
//    char AGC;//AGC 1
//    char channel_BER3;//信道误码率 1
//    char system_BER;//系统误码率 1
//    char operating_frequency;//工作频点 1
//    char handover_frequency;//交接频点 1
//    char checksums;//和校验 1
//}UHF;
//typedef struct Air_Earth_link_telemetry_information_data_content_definition_table//空地链链路遥测信息数据内容定义表
//{
//    unsigned int head;
//    char synchronized_characters1;//同步字 1
//    char synchronized_characters2;//同步字 1
//    char frame_length;//帧长 1
//    char type;//类型 1
//    char serial_number_frame_count;//序号/帧计数 1
//    char local_device_ID[2];//本机设备ID 2
//    char communication_object_ID[2];//通信对象ID 2
//    char communication_mode;//通信模式 1
//    char UTC_time[3];//UTC时间 3
//    char coordinate_valid_flags;//坐标有效标志 1
//    char native_coordinates[12];//本机坐标 12
//    char reserve1;//备用 1
//    char wideband_transmission_C_transceiver_combination_status_information[1000];//宽带传输C收发组合状态信息 待定
//    char C_band_RF_front_end_status[1000];//C波段射频前端状态  待定
//    char forward_transmitting_frequency;//前端发射频点 1
//    char Received_signal_amplitude1;//接收信号幅度 1
//    char antenna_operating_status[1000];//天线工作状态 待定
//    char reserve2;//备用 1
//    char reserve3;//备用 1
//    char master_link_handover_state;//主链交接状态 1
//    char state;//状态 1
//    char version_number1;//版本号 1
//    char C_uplink_handover_channel_number;//C上行交接频道号 1
//    char reserve4;//备用 1
//    char uplink_rate_frequency_points1;//上行速率、频点 1
//    char Received_signal_amplitude2;//接收信号幅度 1
//    char channel_BER1;//信道误码率 1
//    char downlink_speed1;//下行速率 1
//    char version_number2;//版本号 1
//    char C_downlink_handover_channel_number;//C下行交接频道号 1
//    char reserve5;//备用 1
//    char uplink_rate_frequency_points2;//上行速率、频点 1
//    char Received_signal_amplitude3;//接收信号幅度 1
//    char channel_BER2;//信道误码率 1
//    char downlink_speed2;//下行速率 1
//    char version_number3;//版本号 1
//    char C_handover_station_communication_mode;//C交接站通信模式 1
//    char secondary_chain_handover_mode;//副链交接模式 1
//    char UHF_band_transceiver_combination_state;//UHF波段收发组合状态 1
//    char reserve6;//备用 1
//    char reserve7;//备用 1
//    char temp;//温度 1
//    char interface_status;//接口状态 1
//    char reception_state;//接收状态 1
//    char AGC;//AGC 1
//    char channel_BER3;//信道误码率 1
//    char system_BER;//系统误码率 1
//    char operating_frequency;//工作频点 1
//    char handover_frequency;//交接频点 1
//    char checksums;//和校验 1
//    char C_band_RF_front_returns;//C波段射频前端回报 1
//    char reserve8;//备用 1
//    char reserve9;//备用 1
//}Air_Earth_liknk_telemetry_information_data_content_definition_table;
//typedef struct Air_ground_link_telemetry_information
//{
//	Air_Earth_liknk_telemetry_information_data_content_definition_table Air_Earth_link_telemetry_information_data_content_definition_tables;//空地链链路遥测信息数据内容定义表
//    UHF UHFs;//空地链链路遥测信息数据内容定义表（UHF）
//}Air_ground_link_telemetry_information;
//
//
//
////1.6 任务分配结果信息
//typedef struct cotask_sequence_information//任务序列信息
//{
//    unsigned int subtask_ID_number;//任务平台子任务ID号
//    char modify;//任务平台子任务是否变更
//    char sequence_type;//任务平台子任务任务序列类型
//    char type_of_mission_point_area;//任务平台子任务任务点/区域类型
//    unsigned int target_number;//任务平台子任务任务区/点/线/目标编号
//    char completion_time_valid_Bits;//任务平台子任务完成时间有效位
//    char task_height_effective_position;//任务平台子任务任务高度有效位
//    unsigned int task_completion_time;//任务平台子任务完成时间
//    int mission_height;//任务平台子任务任务高度
//    char availability_of_routes;//任务平台子任务是否存在航线
//    char presence_of_buoy_array;//任务平台子任务是否存在浮标阵
//    char presence_of_fixed_point_arrays;//任务平台子任务是否存在定测点阵型
//}cotask_sequence_information;
//typedef struct coformation_synergy_mission_program//编队任务协同方案
//{
//    char platform_model;//任务平台型号
//    unsigned int platform_serial_number;//任务平台序号
//    unsigned int platform_number;//任务平台编号
//    unsigned int duration_of_the_mission;//任务平台任务时长
//    unsigned int number_of_subtasks;//任务平台子任务个数m
//    cotask_sequence_information task_sequence_informations[8];//任务序列信息（循环8次）
//
//}coformation_synergy_mission_program;
//typedef struct coinformation_on_the_results_of_tasking
//{
//    unsigned int head;
//    char tasking_release;//任务分配发布
//    char manual_modification;//任务分配方案是否人工修改
//    char emphasize_planning;//任务分配方案是否重规划
//    char program_attributes;//方案属性
//    unsigned int program_number;//方案编号
//    char search_and_dive_method;//协同搜潜方式
//    unsigned int total_program_time;//方案总时间
//    unsigned short number_of_mission_platforms;//任务平台个数n
//    coformation_synergy_mission_program formation_synergy_mission_programs[5];//编队协同任务方案（循环5次）
//
//}coInformation_the_results_of_tasking;
//
//
////1.7 有人机航路规划结果信息
//typedef struct cosuspended_sound_buoy_deployment_point_information
//{
//    colongitude_and_latitude point_colongitude_and_latitude;//浮标布阵点经纬度
//    double profundity;//浮标布阵点深度
//}cosuspended_sound_buoy_deployment_point_information;//浮标布阵点信息
//typedef struct cosuspended_sound_manned_computer_task//有人机子任务
//{
//    unsigned int subtask_ID_number;//子任务ID
//    unsigned int fixed_measurement_points_number;//定测点总数
//    cosuspended_sound_buoy_deployment_point_information suspended_sound_buoy_deployment_point_informations[10];//浮标布阵点信息
//}cosuspended_sound_manned_computer_task;
//typedef struct cosuspended_sound_fixed_measurement_point//吊声定测点
//{
//    unsigned int head;
//    unsigned int program_number;//方案编号
//    unsigned short subtasks_number;//有人机子任务个数m
//    cosuspended_sound_manned_computer_task suspended_sound_manned_computer_tasks[8];//有人机子任务（循环8次）
//}cosuspended_sound_fixed_measurement_point;
//typedef struct cobuoy_deployment_point_information//浮标布阵点信息
//{
//    char matching_tube_number;//浮标布阵点匹配筒位号
//    char buoy_type;//浮标布阵点浮标类型
//    colongitude_and_latitude buoy_colongitude_and_latitude;//浮标布阵点经纬度
//    char longitude_validity;//浮标布阵点经度有效性
//    char latitude_validity;//浮标布阵点维度有效性
//}cobuoy_deployment_point_information;
//typedef struct cobuoy_manned_computer_task//有人机子任务
//{
//    unsigned int subtask_ID_number;//子任务ID号
//    unsigned int points_number;//浮标布阵点总数
//    cobuoy_deployment_point_information buoy_deployment_point_informations[25];//浮标布阵点信息
//}cobuoy_manned_computer_task;
//typedef struct cobuoy_deployment_point//浮标布阵点
//{
//    unsigned int head;
//    unsigned int program_number;//方案编号
//    unsigned short subtasks_number;//有人机子任务个数m
//    cobuoy_manned_computer_task buoy_manned_computer_tasks[8];//有人机子任务（循环8次）
//}cobuoy_deployment_point;
//typedef struct cowaypoint_information
//{
//    char type;//航路点类型
//    char validity_of_longitude;//航路点经度有效性
//    char dimensional_validity;//航路点维度有效性
//    char height_validity;//航路点高度有效性
//    char speed_validity;//航路点速度有效性
//    char direction_validity;//航路点航向有效性
//    char time_validity;//航路点时间有效性
//    char payloads_validity;//航路点载荷有效性
//    colongitude_and_latitude waypoint_colongitude_and_latitude;//航路点经纬度
//    unsigned int height;//航路点高度
//    unsigned int speed;//航路点速度
//    unsigned int direction;//航路点航向
//    unsigned int time;//航路点时间
//    char payloads;//航路点载荷
//}cowaypoint_information;//航路点信息
//typedef struct comanned_computer_task//有人机子任务
//{
//    unsigned int subtask_ID_number;//子任务ID号
//    unsigned short waypoints_number;//航路点个数n
//     unsigned short waypoint_start_number;//航路点起始编号
//    cowaypoint_information waypoint_informations[75];//航路点信息
//
//}comanned_computer_task;
//typedef struct cocommon_carrier_route//通用航路
//{
//    unsigned int head;
//    unsigned int program_number;//方案编号
//    unsigned short subtasks_number;//有人机子任务个数
//    comanned_computer_task manned_computer_tasks[8];//有人机子任务（循环8次）
//
//}cocommon_carrier_route;
//typedef struct comanned_aircraft_route_drone_route_information
//{
//    cocommon_carrier_route common_carrier_routes;//通用航路
//    cobuoy_deployment_point buoy_deployment_points;//浮标布阵点
//    cosuspended_sound_fixed_measurement_point suspended_sound_fixed_measurement_points;//吊声定测点
//}comanned_aircraft_route_drone_route_confirmation_information;
//
//
////1.8 无人机航路规划（空空链）
//typedef struct coplanning_information_waypoint_information//航路点信息
//{
//    cowaypoint_information waypoint_informations;//航路点信息
//    char causality;//航路点属性
//    char standby_type;//航路点待机类型
//    unsigned int standby_time_lapsNumber_cycleNumber;//航路点待机时间/圈数/循环次数
//    unsigned int standby_radius;//航路点待机半径
//    char standby_time_lapsNumber_cycleNumber_valid_bit;//航路点待机时间/圈数/循环次数有效位
//    char standby_radius_valid_bit;//航路点待机半径有效位
//}coplanning_information_waypoint_information;
//typedef struct coplanning_information//单无人机单序列中的规划信息
//{
//    unsigned int subtask_ID_number;//子任务ID号
//    char mission_type;//子任务任务类型
//    char point_area_type;//任务点/区域类型
//    unsigned int area_point_line_goal_number;//子任务任务区/点/线/目标编号
//    unsigned int completion_time;//子任务完成时间
//    unsigned int mission_height;//子任务任务高度
//    char completion_time_valid_Bits;//子任务完成时间有效位
//    char ask_height_effective_position;//子任务任务高度有效位
//    unsigned short waypoints_number;//航路点个数n
//    coplanning_information_waypoint_information planning_information_waypoint_informations[255];//航路点信息
//
//}coplanning_information;
//typedef struct coindividual_drone_routing_program//单个无人机航路方案
//{
//    unsigned short drone_serial_number;//无人机序号
//    unsigned short subtasks_number;//当前无人机子任务个数
//    coplanning_information planning_informations[8];//单无人机单序列中的规划信息(循环8次)
//}coindividual_drone_routing_program;
//typedef struct codrone_route_confirmation
//{
//    unsigned short head;
//    unsigned int program_number;//方案编号
//    unsigned int number_of_drones;//无人机数量
//    coindividual_drone_routing_program individual_drone_routing_programs[4];//单个无人机航路方案
//}codrone_route_confirmation;
//
//
//
//
//
//
////1.9 任务区/空域信息（空地链）
//typedef struct cocycle
//{
//    colongitude_and_latitude cycle_colongitude_and_latitude;//圆心经纬度
//    float radius;//圆心半径
//}cocycle;
//typedef struct copolygonal//多边形
//{
//    unsigned short point_number;//点数n
//    copoint_coordinate copoint_coordinates[10];//点坐标
//}copolygonal;
//typedef struct coarea_information//区域信息
//{
//    unsigned int area_code;//区域编号
//    char area_type;//区域类型
//    char area_source;//区域来源
//    char area_shape;//区域形状
//    char drone_number_valid_bit;//空域所属无人机编号有效位
//    unsigned int drone_numbe;//空域所属无人机序号
//    char upper_height_limit_valid_bit;//区域高度上限有效位置
//    char lower_height_limit_valid_bit;//区域高度下限有效位置
//    float upper_height_limit;//区域高度上限
//    float lower_height_limit;//区域高度下限
//    cocycle cycles;//圆
//    copolygonal polygonals;//多边形
//
//}coarea_information;
//typedef struct coarea_sky_information
//{
//    unsigned int head;
//    unsigned short area_number;//区域个数
//    coarea_information area_informations[255];//区域信息
//}coarea_sky_information;
//
//
////1.10 任务点信息
//typedef struct copoint_information//点信息
//{
//    unsigned int point_number;//点编号
//    char point_source_of_information;//点信息来源
//    colongitude_and_latitude point_colongitude_and_latitude;//点经纬度
//    char point_speed_effective;//点目标航速有效
//    char point_direction_effective;//点目标航向有效
//    char point_time_effective;//点目标时间有效
//    char point_property_effective;//点目标属性有效位置
//    char point_type_effective;//点目标类型有效位置
//    char point_batchNumber_effective;//点目标批号有效位置
//    float speed;//目标航速
//    float direction;//目标航向
//    short year;//年
//    char month;//月
//    char day;//日
//    int millisecond;//毫秒
//    char point_property;//目标属性
//    char point_type;//目标类型
//    unsigned int point_batchNumber;//批号
//}copoint_information;
//typedef struct cotaks_point_information
//{
//    unsigned int head;
//    unsigned short point_number;//任务点个数
//    copoint_information point_informations[10];//点信息
//}tcoaks_point_information;
//
//
//
////1.11 任务线信息（空地链）
//typedef struct coline_information
//{
//    unsigned int line_num;//线编号
//    char line_type;//线类型
//    unsigned short line_point_number;//任务线点个数
//    copoint_coordinate copoint_coordinates[10];//点经纬度
//}coline_information;
//typedef struct cotask_line_information
//{
//    unsigned int head;
//    unsigned short line_number;//任务线条数
//    coline_information line_informations[10];//线信息
//}cotask_line_information;
//
//8.1 基本遥控指令
typedef struct empty_instruction//空指令
{
    char distance_from_station[4];//机站距离 4
    char unmanned_helicopter_azimuth[2];//无人直升机方位角2
    char ship_longitude[4];//舰船经度4
    char ship_latitude[4];//舰船维度4
    char groundspeed[2];//舰船地速2
    char direction[2];//舰船航向2
    char state_data_valid_bit[2];//状态及数据有效标志位2
    char standby[6];//备用6
}empty_instruction;
typedef struct aircraft_remote_control_command//飞机遥调指令
{
    char start;//起始1
    char telecommand_code[3];//遥调指令编码3
    char telecommand_data[26];//遥调指令数据26
    char check;//校验1
}aircraft_remote_control_command;
typedef struct basic_remote_control_commands
{
    char synchronization_code[2];//同步码2
    char frame_class;//帧类别1
    char frame_counter;//帧计数器1
    char aircraft_address[6];//飞机地址6
    char control_station_address[6];//控制站地址6
    char airplane_remote_control_commands[3];//飞机遥控指令3
    aircraft_remote_control_command aircraft_remote_control_commands;//飞机遥调指令
    //飞机连续指令
    char checksum1[2];//校验和2
    char synchronization_code2[2];//同步码2
    char frame_count;//帧计数1
    char data_binding_command_code[3];//数据装订指令码3
    char data_binding_content[24];//数据装订内容24
    char checksum2[2];//校验和2

}basic_remote_control_commands;

//
//
#endif // COOPERATIVEALLEGATIONCOMPUTER_COOPERATIVECOMMUNICATIONSYSTEMS_H
