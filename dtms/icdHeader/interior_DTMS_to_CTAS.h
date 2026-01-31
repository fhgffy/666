#ifndef NEIBUTONGXINJIEGOUTI_H
#define NEIBUTONGXINJIEGOUTI_H
#include "icd_ccc_ofp.h"
#pragma pack(1)

/******************************相关公用结构体**********************************/

//typedef struct longitude_and_latitude  //经纬度
//{
//    double longitude;//经度
//    double latitude;//维度

//}longitude_and_latitude;

//typedef struct point_coordinate  //点坐标
//{
//    longitude_and_latitude point_longitude_and_latitude; //点经纬度
//}point_coordinate;

typedef struct solider_info{     // 兵力信息
    unsigned char solider_type ; // 士兵类别
    unsigned short solider_id; // 任务区士兵序号
    longitude_and_latitude lon_lat_info; // 经纬度信息
    float height; // 高度
    float speed; // 速度
    float hangxiang; // 航向
    unsigned char ct; // 磁探载荷在线状态 0-不可用 1-可用
}solider_info;

typedef struct circle{ // 圆形
    longitude_and_latitude center_lon_lat; // 圆形经纬度
    float radious; // 圆半径
}circle;

typedef struct ploygen{ // 多边形
    unsigned short point_num; // 点数
    point_coordinate points_lon_lat[10];  // 点经纬度信息
}ploygen;

typedef struct rectangle{ //矩形
    longitude_and_latitude rectangle_center; // 矩形区域中心经纬度
    double zongxiangyunorth_angle;  // 矩形纵向与正北方向夹角
    double hengxiang_length; // 矩形横向边长
    double zongxiang_lenget; // 矩形纵向边长
} rectangle;

typedef struct Point_info{ // 点信息
    unsigned int point_serlize_id; // 点序号
    unsigned int point_id; // 点编号
    unsigned short point_source; // 点信息来源
    point_coordinate point_lon_lat; // 点经纬度
    unsigned char point_speed_valid; // 点航速有效
    unsigned char point_hangxiang_valid; // 点航向有效
    unsigned char point_time_valid; // 点时间有效
    unsigned char target_stats_valid; // 目标属性有效
    unsigned char target_type_valid; // 目标类型有效
//    unsigned char target_pitch_valid; // 目标批号有效
    float target_speed; // 目标航速
    float target_hangxiang; // 目标航向
    short year;//年
    short month;//月
    short day;//日
    int secends; // 毫秒
    unsigned short target_stats; // 目标属性
    unsigned short target_type; // 目标类型
    unsigned int pitch;  // 批号
#ifdef _SPARE_
    unsigned char spare[5];
#endif
}Point_info;
typedef struct region_info{   // 区域信息
    unsigned int reg_serlize_id; // 区域序号
    unsigned short reg_id; // 区域编号
    unsigned short reg_type; // 区域类别
    unsigned short reg_sour; // 区域来源
    unsigned short reg_shape; // 区域形状
    unsigned short kongyu_belong_to_uav_valid; // 空域所属无人机有效位
    unsigned int kongyu_belong_to_uav_id; // 空域所属无人机序号
    unsigned short reg_top_of_hei_valid; // 区域高度上限有效位
    unsigned short reg_down_of_hei_valid; // 区域高度下限有效位
    float top_of_hei;   // 区域高度上限
    float down_of_hei;  // 区域高度下限
    circle reg_circle;  // 圆形区域
    ploygen reg_ploygen; // 多边形区域
    unsigned short task_point_num; // 任务点数量
    Point_info point_infos[10];  // 点信息
}region_info;

/******************************任务信息结构体**********************************/
/*
 * DTMS->CTAS  动态任务管理软件 -〉协同辅助决策软件   发
 * */
// 3.1.1 DTMS/CTAS/000 战术战法推荐规划指令
typedef struct {
   unsigned short task_type; // 任务类型
   unsigned short zhanshu_sel; // 战术战法选择
   unsigned short target_type; // 目标类别
   unsigned int solider_num;
   solider_info solider_infos[5]; // 兵力信息
   unsigned short task_reg_num; // 任务区数量
   region_info region_infos[3]; // 区域信息
}BLK_DTMS_CTAS_001;

typedef struct
{
   unsigned short task_type; // 任务类型
   unsigned short zhanshu_sel; // 战术战法选择
   unsigned short target_type; // 目标类别
   unsigned int solider_num;
   solider_info solider_infos[5]; // 兵力信息
   unsigned short task_reg_num; // 任务区数量
   region_info region_infos[3]; // 区域信息
   unsigned int planning_id;    //运行方案id
   unsigned int uav_id;    //单无人机id 0=单任务区/目标,单无人机id
   unsigned int lead_uav_id;	//长机id
}BLK_DTMS_CTAS_002;


typedef struct
{
   BLK_CCC_OFP_019 blk_ccc_ofp_019;//任务分配结果
   BLK_OFP_CCC_038 blk_ccc_ofp_038;//航线生成命令
   unsigned int solider_num;
   solider_info solider_infos[5]; // 兵力信息
   BLK_CCC_OFP_005 blk_ccc_ofp_005;//区域划分信息
}BLK_DTMS_CTAS_005;//单阶段航线生成命令

// 3.1.2 DTMS/CTAS/001 兵力区域分配指令
typedef struct bingliquyufenpei{
    unsigned short solider_num; // 兵力数量
    solider_info solider_infos; // 兵力信息
    unsigned task_reg_num; // 任务区数量
    region_info region_infos; // 区域信息
}bingliquyufenpei;

// 3.1.3 DTMS/CTAS/002 光电雷达搜索航路规划指令
typedef struct gdldsshlgh{
    float plane_lat;  // 飞机纬度
    float plane_lon;  // 飞机经度
    float search_height; // 搜索高度
    unsigned char plane_type; // 飞机类别
    unsigned char sensor_type; // 传感器类别
    float sensor_dis; // 传感器距离
    int point_num; // 顶点数量
    point_coordinate point_infos[9]; // 顶点经纬度
}gdldsshlgh;

// 3.1.4 DTMS/CTAS/003 光电雷达跟踪航路规划指令（待定）
typedef struct gdldgzhlgh{
    // 待定
}gdldgzhlgh;

// 3.1.5 DTMS/CTAS/004 定测点规划指令
typedef struct parallel_secoter{ // 平行扇形
    longitude_and_latitude secoter_center; // 扇形中心经纬度
    double search_orident; // 搜索方向
    double secoter_angle; // 扇形张角
    double search_radious; // 搜索半径
    unsigned int search_level_num; // 搜索层数
    double dingcedian_dis; // 顶测点间距系数
}parallel_secoter;

typedef struct zhizi_secoter{  // 之字扇形
    longitude_and_latitude secoter_center; // 扇形中心经纬度
    double search_orident; // 搜索方向
    double secoter_angle; // 扇形张角
    double search_radious; // 搜索半径
    double extend_steeps; // 外扩步长
    double dingcedian_dis; // 顶测点间距系数
} zhizi_secoter;

typedef struct more_line{  // 多直线形
    longitude_and_latitude first_point_center; // 首线中心点经纬度
    double search_orident; // 搜索方向
    double search_width; // 搜索宽度
    double search_dis; // 搜索距离
    double line_dis; // 行间距
    double dingcedian_dis; // 顶测点间距系数
} more_line;

typedef struct sawtooth{ // 锯齿形
    longitude_and_latitude first_point; // 初始点经纬度
    double search_orident; // 搜索方向
    double search_width; // 搜索宽度
    double line_dis; // 行间距
    double search_dis; // 搜索距离
} sawtooth;

typedef struct square_extend{ // 方形扩展
    longitude_and_latitude first_point; // 初始点经纬度
    double search_orident; // 搜索方向
    double dingcedian_dis; // 定测点间距系数
    double end_line_dis; // 终边边长
} square_extend;

typedef struct spiral_extend{  // 螺旋扩展
    longitude_and_latitude spiral_center; // 螺旋中心经纬度
    double dingcedian_dis; // 顶测点间距系数
    double extend_steeps; // 扩展步长
    double spiral_radious; // 螺旋半径
}spiral_extend;

typedef struct Target_info{   // 目标信息
    unsigned int target_pitch;  // 目标批次号
    unsigned int target_type;  // 目标类型
    unsigned int target_status; // 目标状态
    longitude_and_latitude target_lon_lat; // 目标位置经纬度
    unsigned char target_loc_valid; // 目标位置有效性
    double target_loc_acc; // 目标精度
    unsigned char target_loc_acc_valid; // 目标精度有效性
    double target_speed;  // 目标速度
    unsigned char target_speed_valid; // 目标速度有效性
    double target_speed_acc;  // 目标航速精度
    unsigned char target_speed_acc_valid; // 目标航速精度有效性
    double target_orident;    // 目标航向
    unsigned char target_orident_valid; // 目标航向有效性
    double target_orident_acc;  // 目标航向精度
    unsigned char target_orident_acc_valid;  // 目标航向精度有效性
    unsigned int target_time_flag; // 目标时标
}Target_info;

typedef struct Dingcedian{
   longitude_and_latitude dingcedian_lon_lat; // 顶测点经纬度
   double dingcedian_deepth; // 顶测点深度
} Dingcedian;
typedef struct dingcedianguihua{  // 定测点规划指令
    unsigned int order_type; // 指令编号
    float diaosheng_dis; // 吊声作用距离
    float diaosheng_deepth; // 吊声最佳深度
    int formulae_type; // 规划方式
    unsigned int image_type; // 图形类型选择
    parallel_secoter reg_parallel_secoter; // 平行扇形区域
    zhizi_secoter reg_zhizhi_secoter; // 之字扇形
    more_line reg_more_lines; // 多直线形
    sawtooth reg_sawtooth; // 锯齿形
    square_extend reg_square_extend; // 方形扩展
    spiral_extend reg_spiral_extend; // 螺旋扩展
    int task_type; // 任务类型
    unsigned char task_period; // 任务阶段
    unsigned int target_indo_num; // 目标信息数量
    Target_info target_infos; // 目标信息
    unsigned int reg_type; // 区域类型
    rectangle reg_rectangle;  // 矩形区域
    circle reg_circle; // 圆形区域
    unsigned int diaoshenggenzong_status; // 吊声跟踪状态
    unsigned int diaosheng_loc_status; // 吊声位置状态
    unsigned int diaosheng_shoufang_status; // 吊声收放状态
    double diaosheng_xiafang_deepth; // 吊声下放深度 （收扰时刻）
    unsigned int diaosheng_shoulong_time; // 吊声收拢时刻
    unsigned int  dingcedian_num; // 顶测点总数量
    Dingcedian dingcedians[10]; // 顶测点信息
}dingcedianguihua;

// 3.1.6 DTMS/CTAS/005 悬停过渡航路规划指令
typedef struct xtgdhlgh{
    float plane_lat;  // 飞机纬度
    float plane_lon;  // 飞机经度
    float wind_speed; // 风速
    float wind_orident; // 风向
    int dingcedian_num; // 顶测点数量
    longitude_and_latitude dingcedian_lon_lat[10]; // 顶测点经纬度
}xtgdhlgh;

// 3.1.7 DTMS/CTAS/006 浮标布阵规划指令
typedef struct rectangle_region{ // 矩形区域
    unsigned char data_status_1;   // 数据状态
    unsigned char data_status_2;   // 数据状态
    rectangle rectangle_info; // 矩形信息
} rectangle_region;

typedef struct circle_region{ // 圆形区域
    unsigned char data_status_1;   // 数据状态
    unsigned char data_status_2;   // 数据状态
    circle circle_info; // 圆形信息
} circle_region;

typedef struct buoy_info{
    unsigned char buoy_type; // 浮标类型
    unsigned char buoy_deepth; // 浮标深度
    unsigned char buoy_work_time; // 浮标工作时间
} buoy_info;

typedef struct circle_formation{ // 圆形阵
    unsigned char info[2]; // 圆形有效，补齐数据位，规模控制，圆心布设，补齐数据位 等
    circle circle_info; // 圆形信息
    unsigned int outdoor_huan_num; // 外环数量
    double dis; // 间距系数
} circle_formation;

typedef struct secoter_formation{ // 扇形阵
    unsigned char info[2]; // 扇形有效，补齐数据位，直边浮标布设，补齐数据位 等
    circle circle_info; // 扇形中圆形信息
    double shanzhou_oridet; // 扇轴方向
    double shanzhou_angel; // 扇区张角
    double dis; // 间距系数
} secoter_formation;

typedef struct more_line_formation{  // 多直线阵
    unsigned char info[2]; // 多直线阵有效，补齐数据位，交叉选项，补齐数据位 等
    longitude_and_latitude first_point_center; // 首线中心点经纬度
    double lanjie_orident; // 拦截方向
    double lanjie_width; // 拦截宽度
    double double_line_dis; // 双线间距
    int line_num; // 行数
    double dis; // 间距系数
}more_line_formation;

typedef struct ployline_formation{  // 折线阵
    unsigned char info[2]; // 折线阵有效，补齐数据位，顶点浮标布设选项，补齐数据位 等
    longitude_and_latitude point_center; // 顶点经纬度
    double lanjie_orident; // 拦截方向
    double ployline_angle; // 折线夹角
    double ployline_dis; // 折线边长
    double dis; // 间距系数
} ployline_formation;

typedef struct square_cover_formation{  // 方形覆盖阵
    unsigned char info[2]; // 方形覆盖阵有效，补齐数据位，交叉选项，补齐数据位 等
    longitude_and_latitude point_center; // 中心点经纬度
    float hover_angle; // 旋转角度
    double heng_length; // 横长
    double zong_length; // 纵长
    double dis; // 间距系数
}square_cover_formation;

typedef struct zhudong_add_dis_formation{  // 主动增程阵
    unsigned char info[2]; // 主动增程阵有效，补齐数据位 等
    longitude_and_latitude point_center; // 中心点经纬度
    float hover_angle; // 旋转角度
    unsigned short line_num; // 行数
    unsigned short row_num; // 列数
    double dis; // 间距系数
} zhudong_add_dis_formation;

typedef struct DIFIX_formation{ // DIFIX阵
    unsigned char info[2]; // DIFIX阵有效，补齐数据位 等
    longitude_and_latitude start_point; // 起始点1经纬度
    double fangwei_angle; // 方位角
    double DIFIX_startPoint_dis; // DIFIX阵对相距起始点距离
    double bouy_cp_dis; // 浮标对间距
} DIFIX_formation;

typedef struct CODAR_formation{ // CODAR阵
    unsigned char info[2]; // CODAR阵有效，补齐数据位，浮标对间距，补齐数据位 等
    longitude_and_latitude start_point; // 起始点1经纬度
    double fangwei_angle; // 方位角
    double CODAR_startPoint_dis; // CODAR阵中心对起始点距离
}CODAR_formation;

typedef struct custom_formation{ // 自定义阵
    unsigned char bouy_type[2]; // 浮标类型
    unsigned char bouy_deepth; // 浮标深度
    unsigned char bouy_work_time; // 浮标工作时间
    unsigned char info[2]; // 未存活补投指令，补齐数据位，布阵点数量 等
    longitude_and_latitude points[75]; // 点1经纬度(循环75次)
} custom_formation;

typedef struct shuisheng{ // 水声信息
    float bdqxjlgj; // 被动全向距离估计
    unsigned int bdqxgzsd; // 被动全向工作深度
    float bddxjlgj;   // 被动定向距离估计
    unsigned int bddxgzsd; // 被动定向工作深度
    float zdqxjlgj; // 主动全向距离估计
    unsigned int zdqxgzsd; // 动全向工作深度
    float zddxjlgj;   // 主动定向距离估计
    unsigned int zddxgzsd; // 主动定向工作深度
    float czxlz_dis; // 垂直线列阵距离估计
    unsigned int czxlz_deepth; // 垂直线列阵工作深度
    float zdsy_kzz_dis;   // 主动声源+扩展阵距离估计
    unsigned int zdsy_kzz_deepth; // 主动声源+扩展阵工作深度
} shuisheng;


typedef struct fubiaobuzhenguihua{  // 浮标布阵规划指令
    unsigned int message_id; // 消息编号
    unsigned char buzhen_formulate_orders[4]; // 布阵规划指令
    unsigned char auto_formulat_task_and_zhenxing[2]; // 自动规划任务类型和阵形选择
    Target_info target_infos; // 目标信息（目标点模式下有效）
    rectangle_region rectangle_reg_info; // 矩形区域（矩形模式下有效）
    circle_region circle_region_info;  // 圆形区域（圆形模式下有效）
    unsigned char zhengxing_select[4];  // 阵形选择
    buoy_info buoy_info_1; // 浮标信息
    circle_formation circle_formation_info; // 圆形阵信息
    secoter_formation secoter_formation_info; // 扇形阵信息
    more_line_formation more_line_formation_info; // 多直线阵信息
    ployline_formation ployline_formation_info; // 折线阵信息
    square_cover_formation square_cover_formations; // 方形覆盖阵信息
    zhudong_add_dis_formation zhudong_add_dis_formation_info; // 主动增程阵信息
    DIFIX_formation DIFIX_formation_info; // DIFIX阵信息
    CODAR_formation CODAR_formation_info; // CODAR阵信息
    custom_formation custom_formation_info; // 自定义阵信息
    shuisheng shuisheng_info; // 水声信息
    unsigned char bouy_info_2[4]; // 浮标类型 工作状态 频率 寿命 深度 通道号 spare 等信息
}fubiaobuzhenguihua;

// 3.1.8 DTMS/CTAS/007 布阵航路规划指令
typedef struct  bouy_formation{  // 浮标布阵点
    unsigned char bouy_point_type; // 浮标布阵点类型
    unsigned char bouy_point_barrel_id; // 浮标布阵点筒位号
    longitude_and_latitude bouy_points; // 浮标布阵点经纬度
}bouy_formation;

typedef struct buzhenhanglu{  // 布阵航路规划指令
    unsigned int message_id; // 消息编号
    unsigned char bouy_plan[4]; // 浮标布阵航路方案
    bouy_formation bouy_formation_info[25]; // 浮标布阵点信息 循环25次
}buzhenhanglu;

// 3.1.9 DTMS/CTAS/008 监听航路规划指令
typedef struct jiantinghanglu{
    int bouy_num; // 浮标数量
    longitude_and_latitude bouy_lon_lat[25]; // 浮标经纬度
    float jianting_heigth; // 监听高度
}jiantinghanglu;

// 3.1.10 DTMS/CTAS/009 磁探搜索航路规划指令
typedef struct shape_8_hanglu{ // 8字形航路
    longitude_and_latitude center_loc; // 中心位置经纬度
    float in_orident; // 进入方向
    float shape_8_angle; // 8字张角
    float shape_8_radious; // 8字半径
    float pianzhizhouxiang; // 偏置轴向
    float pianzhi_dis; // 偏置距离
    unsigned char search_circlr_num; // 搜索圈数
} shape_8_hanglu;

typedef struct circle_hanglu{ // 环形前飞航路
    longitude_and_latitude base_loc; // 基准位置经纬度
    unsigned char base_loc_type; // 基准位置类型
    unsigned char first_in_orident; // 初始进入方向
    float search_orident; // 搜索轴向
    float search_cell_width; // 搜索单元宽度
    float search_cell_length; // 搜索单元长度
    float search_cell_dis; // 搜索单元间隔
    unsigned char search_cell_num; // 搜索单元数量
}circle_hanglu;

typedef struct muxuye_hanglu{ // 苜蓿叶航路
    longitude_and_latitude center_loc; // 中心位置经纬度
    float search_length; // 搜索长度
    float search_width; // 搜索宽度
    unsigned char in_orident; // 进入方向
    unsigned char in_turn_orident; // 进入转弯方向
    float pianzhizhouxiang; // 偏置轴向
    float pianzhi_dis; // 偏置距离
    unsigned char search_circlr_num; // 搜索圈数
}muxuye_hanglu;

typedef struct citansousuohanglu{  // 磁探搜索航路规划指令
    float hangxian_heigth; // 指定航线高度
    float hangxian_speed; // 航线速度
    unsigned char hanglu_type; // 航路类型选择
    shape_8_hanglu shape_8_hanglu_info; // 8字航路参数
    circle_hanglu circle_hanglu_info; // 环形前飞航路
    muxuye_hanglu muxuye_hanglu_info;// 苜蓿叶航路
}citansousuohanglu;

// 3.1.11 DTMS/CTAS/010 磁探跟踪航路规划指令
typedef struct precise_position_track{ // 精确定位与连续追踪规划
    unsigned char target_info_valid; // 目标信息有效性
    unsigned char target_info_source; // 目标信息来源
    unsigned char target_loc_valid; // 目标位置有效性
    unsigned char target_speed_valid; // 目标速度有效性
    unsigned char target_orident_valid; // 目标航向有效性
    longitude_and_latitude target_lon_lat; // 目标位置经纬度
    float target_speed;  // 目标速度
    float target_orident;    // 目标航向
    short year_target; // 目标接触时间 年
    unsigned char month_target; // 目标接触时间 月
    unsigned char day_target; // 目标接触时间 日
    int secends_target; // 目标接触时间 毫秒
    short year_plane; // 机载当前时间 年
    unsigned char month_plane; // 机载当前时间 月
    unsigned char day_plane; // 机载当前时间 日
    int secends_plane; // 机载当前时间 毫秒
}precise_position_track;
typedef struct citangenzonghanglu{ // 磁探跟踪航路规划指令
    float hangxian_heigth; // 指定航线高度
    float hangxian_speed; // 航线速度
    unsigned char fanqie_type; // 反潜类型
    unsigned char formulate_type; // 规划指标类型
    precise_position_track precise_position_track_info; // 精确定位与连续追踪规划
}citangenzonghanglu;

// 3.1.12 DTMS/CTAS/011 通信中继航路规划指令
typedef struct zhongji_slider{ // 中继兵力
    unsigned int reg_serlize_id; // 区域序号
    unsigned char solider_type; // 兵力类型
    unsigned int task_reg_solider_id; // 任务区域兵力编号
    unsigned int reg_id; // 区域编号
    unsigned char reg_type[2];  // 区域类型
    unsigned char reg_source[2]; // 区域来源
    unsigned char reg_shape[2]; // 区域形状
    char kongyu_uav_valid; // 空域所属无人机编号有效位
    unsigned int kongyu_uav_id; // 空域所属无人机编号
    char reg_top_of_hei_valid; // 区域高度上限有效位
    char reg_down_of_hei_valid; // 区域高度下限有效位
    float top_of_hei;   // 区域高度上限
    float down_of_hei;  // 区域高度下限
    circle reg_circle;  // 圆形区域
    ploygen reg_ploygen; // 多边形区域
}zhongji_slider;
typedef struct tongxinzhongjihanglu{ // 通信中继航路规划指令
    unsigned short zhongji_slider_num; // 待中继兵力个数
    zhongji_slider zhongji_slider_info; // 中继兵力

}tongxinzhongjihanglu;


// 3.1.17 DTMS/CTAS/016 攻击航路规划指令
typedef struct attack_target{  // 攻击目标信息
    unsigned char target_info_valid; // 目标信息有效性
    unsigned char target_info_source; // 目标信息来源
    unsigned char target_loc_valid; // 目标位置有效性
    unsigned char target_speed_valid; // 目标速度有效性
    unsigned char target_orident_valid; // 目标航向有效性
    longitude_and_latitude target_lon_lat; // 目标位置经纬度
    float target_speed;  // 目标速度
    float target_orident;    // 目标航向
    short year_target; // 目标接触时间 年
    unsigned char month_target; // 目标接触时间 月
    unsigned char day_target; // 目标接触时间 日
    int secends_target; // 目标接触时间 毫秒
    short year_plane; // 机载当前时间 年
    unsigned char month_plane; // 机载当前时间 月
    unsigned char day_plane; // 机载当前时间 日
    int secends_plane; // 机载当前时间 毫秒
}attack_target;

typedef struct gongjihanglu{ // 攻击航路规划指令
    attack_target attack_target_info;  // 攻击目标信息
}gongjihanglu;


#pragma pack()
#endif // NEIBUTONGXINJIEGOUTI_H
