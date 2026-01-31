#ifndef COOPERATIVECOMMUNICATIONSYSTEMCOOPERATIVECHARGCOMPUTER_H
#define COOPERATIVECOMMUNICATIONSYSTEMCOOPERATIVECHARGCOMPUTER_H

//经纬度
typedef struct perlongitude_and_latitude
{
    double longitude;//经度
    double latitude;//维度

}perlongitude_and_latitude;

// 2.1-2.3

//2.1 载荷遥测与指令响应帧（无人机下传）

//2.2 任务重规划结果（无人机下传）


//2.3 任务业务信息

#pragma pack(1)

//2.4 视频下传设置指令帧
typedef struct video_downlink_setting_command_frames
{
    char synchronization_code[2];//同步码 2
    char frame_class;//帧类别 1
    char downlaod_settings[2];//下传设置 2
    char reserve[2];//备用 2
    char checksum;//和校验 1
}video_downlink_setting_command_frames;



//2.5 声纳浮标目标信息（舰面站上传）
typedef struct sonobuoy_targeting_information
{
    char synchronization_code[2];//同步码 2
    char frame_class;//帧类别 1
    unsigned short goal_number;//目标数量
    int goal_num;//目标编号
    char target_validity;//目标有效性
    int timestamp;//时戳
    char data_symbol;//数据标志
    char target_batch_num;//目标批次号
    char target_sequence_num;//目标顺序号（发现次数）
    char discovery_time_valid;//发现时间有效
    char target_position_valid;//目标位置有效
    char target_orientation_valid;//目标方位有效
    char target_distance_valid;//目标距离有效
    char target_heading_valid;//目标航向有效
    char distance_azimuth_reference_point_valid;//距离方位参考点有效
    unsigned short year;//年
    char month;//月
    char day;//日
    int millisecond;//毫秒
    perlongitude_and_latitude target_location_longitude_and_latitude;//目标位置经纬度
    double target_position_accuracy;//目标位置精度（目标位置误差半径）
    float target_location;//目标方位
    float target_location_accuracy;//目标方位精度
    float target_distance;//目标距离
    float target_distance_accuracy;//目标距离精度
    float target_heading;//目标航向
    float target_heading_accuracy;//目标航向精度
    perlongitude_and_latitude distance_azimuth_reference_point_longitude_and_latitude;//距离方位参考点经纬度
    char judgement_mode;//判决方式
    char target_properties;//目标属性
    char target_type;//目标类型
    int positioning_buoy_num[7];//定位浮标编号1-7
    char check_digit;//校验字节

}sonobuoy_targeting_information;



//2.6 航线确认回报及确认航线
typedef struct route_confirmation_returns_and_confirmed_routes
{
    char synchronization_code[2];//同步码 2
    char frame_class;//帧类别 1
    char frame_counter;//帧计数器 1
    char aircraft_address[6];//飞机地址 6
    char station_address[6];//站地址 6
    char mission_program_messages[8];//任务方案消息  8
    char route_and_threat_management_data_frames[32];//航线与威胁管理数据帧 32
}route_confirmation_returns_and_confirmed_routes;

#pragma pack()

#endif // COOPERATIVECOMMUNICATIONSYSTEMCOOPERATIVECHARGCOMPUTER_H

