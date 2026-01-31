#ifndef INTERIOR_CTAS_TO_DTMS_H
#define INTERIOR_CTAS_TO_DTMS_H


#include "interior_DTMS_to_CTAS.h"

#pragma pack(1)

/******************************任务信息结构体**********************************/
// 3.2.1 战术战法推荐规划结果  分包
typedef struct zhanshuzhanfa_result{
    unsigned int plan_id; // 方案id
    unsigned short plan_stats; // 方案属性
    unsigned short cooper_souqie_mode; // 协同搜潜方式
    unsigned int plan_total_time; // 方案预计时间
    unsigned short plant_num; // 平台个数
    // 与协同指控计算机-综显中 3.6 任务分配结果信息   编队协同任务方案（循环5次）相同故直接调用
    formation_synergy_mission_program formation_synergy_mission_programs[5];//编队协同任务方案（循环5次）
}zhanshuzhanfa_result;


// 3.2.2 兵力区域分配结果
// 结果信息中区域信息
typedef struct result_reg_info{  // 区域信息
    unsigned int reg_serlize_id; // 区域序号
    char solider_type; // 兵力类型
    unsigned int reg_solider_id; // 任务区兵力序号
    unsigned short reg_id; // 区域编号
    char reg_type[2]; // 区域类别
    char reg_sour[2]; // 区域来源
    char reg_shape[2]; // 区域形状
    char kongyu_belong_to_uav_valid; // 空域所属无人机有效位
    unsigned int kongyu_belong_to_uav_id; // 空域所属无人机序号
    char reg_top_of_hei_valid; // 区域高度上限有效位
    char reg_down_of_hei_valid; // 区域高度下限有效位
    float top_of_hei;   // 区域高度上限
    float down_of_hei;  // 区域高度下限
    circle reg_circle;  // 圆形区域
    ploygen reg_ploygen; // 多边形区域
}result_reg_info;

typedef struct solider_reg_result{  // 兵力区域分配结果
    unsigned short task_reg_num;  // 任务区数量
    result_reg_info result_reg_infos[3]; // 区域信息
}solider_reg_result;

// 3.2.3 光电雷达搜索航路结果
typedef struct hangli_point{  //航路点
    char type;//航路点类型
    char validity_of_longitude;//航路点经度有效性
    char latitude_validity;//航路点维度有效性
    char height_validity;//航路点高度有效性
    char speed_validity;//航路点速度有效性
    char direction_validity;//航路点航向有效性
    char time_validity;//航路点时间有效性
    char payloads_validity;//航路点载荷有效性
    longitude_and_latitude waypoint_longitude_and_latitude;//航路点经纬度
    double height;//航路点高度
    double speed;//航路点速度
    double direction;//航路点航向
    unsigned int time;//航路点时间
    char payloads;//航路点载荷
    char causality;//航路点属性
    char standby_type;//航路点待机类型
    unsigned int standby_time_lapsNumber_cycleNumber;//航路点待机时间/圈数/循环次数
    unsigned int standby_radius;//航路点待机半径
    char standby_time_lapsNumber_cycleNumber_valid_bit;//航路点待机时间/圈数/循环次数有效位
    char standby_radius_valid_bit;//航路点待机半径有效位
}hangli_point;

typedef struct exit_hanglu{ // 退出航线时飞机状态
    char exit_info_valid; // 退出信息是否有效
    float plane_lat;  // 飞机纬度
    float plane_lon;  // 飞机经度
    float search_height; // 搜索高度
}exit_hanglu;

typedef  struct gdldsshl_result{ // 光电雷达搜索航路结果
    hangli_point hangli_point_info[75]; // 航路点信息
    exit_hanglu exit_hanglu_info; // 退出航线时飞机状态信息
}  gdldsshl_result;


// 3.2.4 DTMS/CTAS/003 光电雷达跟踪航路结果（待定）
typedef struct gdldgzhl_result{
    // 待定
}gdldgzhl_result;

// 3.2.5 DTMS/CTAS/004 定测点规划结果
typedef struct result_Target_info{  // 结果中目标信息
    unsigned int target_pitch;  // 目标批次号
    unsigned int target_type;  // 目标类型
    unsigned int target_status; // 目标状态
    longitude_and_latitude target_lon_lat; // 目标位置经纬度
    char target_loc_valid; // 目标位置有效性
    double target_loc_acc; // 目标精度
    char target_loc_acc_valid; // 目标精度有效性
    double target_speed;  // 目标速度
    char target_speed_valid; // 目标速度有效性
    double target_speed_acc;  // 目标航速精度
    char target_speed_acc_valid; // 目标航速精度有效性
    double target_orident;    // 目标航向
    char target_orident_valid; // 目标航向有效性
    double target_orident_acc;  // 目标航向精度
    char target_orident_acc_valid;  // 目标航向精度有效性
    unsigned int target_time_flag; // 目标时标
    unsigned int reg_type; // 区域类型
}result_Target_info;

typedef struct result_Dingcedian{  // 结果中顶测点
    unsigned int  dingcedian_num; // 顶测点总数量
   longitude_and_latitude dingcedian_lon_lat; // 顶测点经纬度
   double dingcedian_deepth; // 顶测点深度
} result_Dingcedian;

typedef struct dingcedian_result{  // 定测点规划结果
    unsigned int confirm_order_id; // 响应指令编号
    unsigned int encode_success_flag; // 解算成功标识
    unsigned int formulate_model; // 规划方式
    unsigned int image_select; // 图形选择（联合体）
    parallel_secoter reg_parallel_secoter; // 平行扇形区域
    zhizi_secoter reg_zhizhi_secoter; // 之字扇形
    more_line reg_more_lines; // 多直线形
    sawtooth reg_sawtooth; // 锯齿形
    square_extend reg_square_extend; // 方形扩展
    spiral_extend reg_spiral_extend; // 螺旋扩展
    unsigned int data_status; // 数据状态
    char target_info_num; // 目标信息个数
    result_Target_info target_infos; // 目标信息
    rectangle reg_rectangle;  // 矩形区域
    circle reg_circle;  // 圆形区域
    unsigned int diaoshenggenzong_status; // 吊声跟踪状态
    unsigned int diaosheng_loc_status; // 吊声位置状态
    unsigned int diaosheng_shoufang_status; // 吊声收放状态
    double diaosheng_xiafang_deepth; // 吊声下放深度 （收扰时刻）
    unsigned int diaosheng_shoulong_time; // 吊声收拢时刻
    float plan_search_status; // 方案搜索状态
    result_Dingcedian result_Dingcedian_info[10]; // 顶测点信息
}dingcedian_result;

// 3.2.6 DTMS/CTAS/005 悬停过渡航路规划结果
typedef struct xtgdhl_result{ // 悬停过渡航路规划结果
    unsigned int hanglu_point_num; // 航路点数量
    hangli_point hangli_point_info[75]; // 航路点信息
    exit_hanglu exit_hanglu_info; // 退出航线时飞机状态信息
}xtgdhl_result;

// 3.2.7 DTMS/CTAS/006 浮标布阵规划结果
typedef struct result_custom_formation{ // 结果中自定义阵
    char bouy_type[2]; // 浮标类型
    char bouy_deepth; // 浮标深度
    char bouy_work_time; // 浮标工作时间
    char info[2]; // 未存活补投指令，补齐数据位，布阵点数量 等
} result_custom_formation;

typedef struct fubiaobuzhen_result{  // 浮标布阵规划结果
    unsigned int confirm_order_id; // 响应指令编号
    unsigned int encode_success_flag; // 解算成功标识
    char buzhen_formulate_order[4];  // 布阵规划指令
    char buzhen_kongzhi_pram[2]; // 布阵控制参数 自动规划任务类型和阵形选择
    Target_info target_infos; // 目标信息（目标点模式下有效）
    rectangle_region rectangle_reg_info; // 矩形区域（矩形模式下有效）
    circle_region circle_region_info;  // 圆形区域（圆形模式下有效）
    char zhengxing_select[4];  // 阵形选择 （图形选择）
    circle_formation circle_formation_info; // 圆形阵信息
    secoter_formation secoter_formation_info; // 扇形阵信息
    more_line_formation more_line_formation_info; // 多直线阵信息
    ployline_formation ployline_formation_info; // 折线阵信息
    square_cover_formation square_cover_formations; // 方形覆盖阵信息
    zhudong_add_dis_formation zhudong_add_dis_formation_info; // 主动增程阵信息
    DIFIX_formation DIFIX_formation_info; // DIFIX阵信息
    CODAR_formation CODAR_formation_info; // CODAR阵信息
    result_custom_formation custom_formation_info; // 自定义阵信息
    longitude_and_latitude points[75]; // 点1经纬度(循环75次)
    unsigned int buoy_num; // 浮标数量
    longitude_and_latitude buoy_lon_lat[25]; // 浮标进纬度
    buoy_info buoy_infos; // 浮标信息
    char buoy_channel;  // 浮标通道号
    char buoy_pipei_result; // 浮标匹配结果
}fubiaobuzhen_result;


// 3.2.8 DTMS/CTAS/007 布阵航路规划结果
typedef struct hanglu_result{
    longitude_and_latitude hl_points; // 航路点1经纬度
    float hl_heigth; // 航路点高度 （取自当前载机高度）
} hanglu_result;
typedef struct buzhenhanglu_result{  // 布阵航路规划结果
    unsigned int confirm_order_id; // 响应指令编号
    unsigned int encode_success_flag; // 解算成功标识
    char data_status[3]; // 数据状态
    hanglu_result hanglu_result_info[75]; // 航路点信息
    unsigned int presee_toufang_time; // 预期投放时间
    unsigned int presee_task_dis; // 预期任务里程
    bouy_formation bouy_formation_info[25]; // 浮标布阵点信息 循环25次
}buzhenhanglu_result;

// 3.2.9 DTMS/CTAS/008 监听航路规划结果
typedef struct jiantinghanglu_result{ // 监听航路规划结果
    unsigned int confirm_order_id; // 响应指令编号
    unsigned int encode_success_flag; // 解算成功标识
    circle jianting_circle_reg; //监听区域圆心信息及半径
    float jianting_heigth; // 监听高度
    hanglu_result jianting_hanglu_point; // 航路点信息
}jiantinghanglu_result;

// 3.2.10 DTMS/CTAS/009 磁探搜索航路规划结果
typedef struct hangxian_info{ // 航路点信息
    char hl_point_num; // 航路点数量
    longitude_and_latitude hl_points; // 航路点1经纬度
}hangxian_info;

typedef struct citansousuohanglu_result{  // 磁探搜索航路规划结果
    int back_status; // 返回状态代码
    float hangxian_heigth; // 指定航线高度
    float hangxian_speed; // 指定航线速度
    float presee_fly_time; // 预计飞行耗时
    hangxian_info hangxian_infos[30]; // 航线信息
}citansousuohanglu_result;

// 3.2.11 DTMS/CTAS/010 磁探跟踪航路规划结果
typedef struct citangenzonghanglu_result{ // 磁探跟踪航路规划结果
    int back_status; // 返回状态代码
    float hangxian_heigth; // 指定航线高度
    float hangxian_speed; // 指定航线速度
    float presee_fly_time; // 预计飞行耗时
    hangxian_info hangxian_infos[30]; // 航线信息
}citangenzonghanglu_result;

// 3.2.12 DTMS/CTAS/011 通信中继航路规划结果
typedef struct tongxinzhongjihanglu_result{ // 通信中继航路规划结果
    int back_status; // 返回状态代码
    float hangxian_heigth; // 指定航线高度
    float hangxian_speed; // 指定航线速度
    float presee_fly_time; // 预计飞行耗时
    hangxian_info hangxian_infos[30]; // 航线信息
}tongxinzhongjihanglu_result;

typedef struct  {
    double longitude;//经度
    double latitude;//纬度
} lon_lat;//地理坐标结构体

typedef struct  {
    lon_lat vertexA[4];
} Airway_area; //最小面积外接矩形

typedef struct  {
    Airway_area airway_area[5];
    unsigned int id[5];
} BLK_CTAS_DTMS_010; //空域信息

// 
#pragma pack()


#endif // INTERIOR_CTAS_TO_DTMS_H
