#ifndef ICD_KKL_CCC_H
#define ICD_KKL_CCC_H


//! \1字节对其
#pragma pack(1)

//! \brief      飞行遥测通信协议
//! \           遥测数据帧结构定义
//! \            遥测帧由5个子帧构成，每个子帧长度为32字节，遥测帧共160个字节，每个子帧均有同步码和效验码
//! \           其中遥测子帧1、2传输速率为20Hz；遥测子帧3分A、B两幅帧，传输速率为10Hz，遥测子帧4分A、B、C、D四幅帧，
//! \           传输速率为5Hz， 遥测子帧5分A、B、C、D、E五副帧，传输速率为4Hz。遥测数据帧的编码结构定义如下表所示：
//! ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//! |       | 子帧 1 | 子帧 2 |      子帧 3     |             子帧 4              |                  子帧 5                  |
//! ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//! |       |   -   |    -   | 副帧3A | 副帧3B | 副帧4A | 副帧4B | 副帧4C | 副帧4D | 副帧5A | 副帧5B | 副帧5C | 副帧5D | 副帧5E |
//! ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//! | 帧类别 |  81H  |  82H   |  3AH  |   3BH  |  4AH  |  4BH   |  4CH  |  4DH   |  5AH  |  5BH   |  5CH  |  5DH   |   5EH  |
//! ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//!
//!
//! 效验规则    帧类别+帧内容+效验码=0
#define quint16 unsigned short
#define qint8  signed char
#define qint32  int
#define quint8 unsigned char
#define qint16  short
//s81/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// namespace s81 {

//! \子帧1 帧类别81 （实时姿态信息）  频率20Hz
typedef struct StructStructs81
{
    //! \brief          : 1     : 0xEB90        //同步码
    quint16 syn_code;

    //! \brief          : 1     : 0x81          //帧类型
    quint8 frame_type;

    //! \brief          : 1     : 0~255         //帧计数
    quint8 frame_num    ;

    //! \brief          : 0.01  : ±90           //俯仰角   抬头为正     （°）
    qint16 pit          ;

    //! \brief          : 0.01  : ±180          //滚转角   右倾为正     （°）
    qint16 roll         ;

    //! \brief          : 0.01  : 0~360         //航向角   无符号     （°）
    quint16 course      ;

    //! \brief          : 0.01  : ±300         //俯仰角速率  俯仰角增大为正     （°/s）
    qint16 pit_rate     ;

    //! \brief          : 0.01  : ±300         //滚转角速率  滚转增大为正      （°/s）
    qint16 roll_rate    ;

    //! \brief          : 0.01  : ±300         //航向角速率  偏航角增大为正      （°/s）
    qint16 course_rate  ;

    //! \brief          : 0.2   : ±25          //机体纵向加速度    向前为正    （m/s^2)
    qint8 body_acce_x   ;

    //! \brief          : 0.2   : ±25          //机体横向加速度    向右为正    （m/s^2)
    qint8 body_acce_y   ;

    //! \brief          : 0.2   : ±25          //机体法向加速度    向上为正    （m/s^2)
    qint8 body_acce_z   ;

    //! \brief 19byte         : 0.1   : -300~300     //真空速            向前为正    （m/s）
    qint16 vacuum_velocity ;

    //! \brief          : 0.1   : -50~300      //机体纵向地速    向前为正    （m/s)
    qint16 body_grouspeed_x ;

    //! \brief          : 0.1   : -100~100     //机体横向地速    向右为正    （m/s)
    qint16 body_grouspeed_y ;

    //! \brief          : 0.1   : -12~12        //垂直速度      向上为正    （m/s）
    qint8 vertical_velocity ;

    //! \brief          : 0.2   : -400~6550    //绝对高度   （m）
    qint16 absolute_height ;

    //! \brief          : 0.2   : -3000~6000   //相对高度（m）
    qint16 relative_height ;

    //! \brief          : 0.01  : 0~2           //超声波高度 （m）
    quint8 ultrasonic_height ;

    //! \brief          : 1     : 0~255         //效验和
    quint8 check;

}s81;

//}

//s82/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s82 {


//! \遥控指令响应结果
typedef struct Structremote_com_res
{
    //! \brief      飞行模态未响应原因
    //!                 000=满足条件，执行成功       001=空/地状态不满足       010=速度条件不满足
    //!                 011=高度标间不满足          100=内回路信号失效        101=升降速度信号失效
    //!                 110=加速度信号失效          111=其他条件不满足
	quint8 flight_mode_un : 3;

    //! \brief      遥控指令执行结果
    //!                 00=执行成功              01=响应条件不满足执行失败      10=设置参数不合理执行失败
    //!                 11=其他原因执行失败
	quint8 remote_com : 2;

    //! \brief      飞行模态异常退出
    //!                 000=执行成功             001=空/地状态不满足           010=速度条件不满足
    //!                 011=高度套件不满足        100=内回路信号失效            101=升降速度信号失效
    //!                 110=加速度信号失效        111=其他条件不满足
	quint8 flight_mode_exit : 3;
}remote_com_res;

//! \子帧1 帧类别82 （控制律/制导律信息）  频率20Hz
typedef struct Structs82
{
    //! \brief          : 1     : 0x55AA        //同步码
    quint16 syn_code;  // 2

    //! \brief          : 1     : 0x82          //帧类型
    quint8 frame_type;  // 1

    //! \brief          : 0.1   : -100~100      //纵向速度控制指令  向前增速为正  （m/s）
    qint16 speed_control_instr_x ;  // 2

    //! \brief          : 0.1   : -100~100      //横向速度控制指令  向前增速为正  （m/s）
    qint16 speed_control_instr_y ;  // 2

    //! \brief          : 0.1   : -500~500      //纵向位置指令    前向为正      （m）
    qint16 pos_inst_x           ;

    //! \brief          : 0.1   : -500~500      //侧向位置指令    右向为正      （m）
    qint16 pos_inst_y           ;

    //! \brief          : 0.2   : -400~6000     //高度控制指令    增高为正      （m）
    qint16 speed_control_instr_ ;   // 13

    //! \brief          : 0.1   : -12~12        //垂直速度控制指令   爬升为正     （m）
    qint8 pos_inst_z            ;   // 1

    //! \brief      备份
    qint8 uavCode                ;  // 后4位是飞机编号（20250519）

    //! \brief          : 0.1   : -180~180      //偏航角控制指令   右转为正    （°）
    qint16 yawing_control_inst  ;  // 2

    //! \brief          : 180 / (2^31 - 1)  : -180~180      // 经度   （°）
    qint32 lon                  ;  // 4

    //! \brief          : 90 / (2^31 - 1)   ：-90`90         //纬度    （°）
    qint32 lat                  ;  // 4 25

    //! \brief          : 0.2   : -6500~6500      //侧偏角     飞机在航线右侧为正   （m）
    qint16 side_angle           ;

    //! \brief          : 5     :-150000~150000     //待飞距/半径        （m）
    qint16 radius               ;

    //! \brief          :   //飞行开关指令回报
    qint8  inst_ret             ;   // 30

    //! \brief          :   //遥控指令响应结果
    remote_com_res  res;

    //! \brief          : 1     : 0~255         //效验和
    quint8 check;

}s82;
//}

//s3A/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s3A {
//! \brief      重规划评估结果结构体
typedef struct Structevaluation_result
{
    //! \brief          威胁评估结果      0=未冲突             1=威胁冲突
	quint8 threat_ass : 1;
    //! \brief          自主避障状态      0=禁止自主避障        1=运行自主避障
	quint8 auto_avoi_status : 1;
    //! \brief          重规划执行状态     0=未规划            1=执行中
	quint8 replanning_status : 1;
    //! \brief          重规划结果         0=等待规划中       1=规划成功      2=规划失败      3=规划不完整
	quint8 replanning : 2;
    //! \brief          任务航线挂接状态    0=空             1=挂接正常      2=挂接失败      其他备用
	quint8 task_route : 2;
    //! \brief          自主避障结果         0=空           1=避障成功
	quint8 auto_avoi : 1;

}evaluation_result;

//! \brief      在线规划航线状态
typedef struct Structplan_route_state
{
    //! \brief      : 0.5       : 0~30          //着舰剩余时间        （min）
	quint8 residual_time : 6;

    //! \brief      最新规划航线号     0=无有效航线         1=28号航线     2=29号航线     3=30号航线
	quint8 route_number : 2;

}plan_route_state;

//! \3A 帧类别3A （浆距信息/航线状态）  频率10Hz
typedef struct Structs3A
{
    //! \brief          : 1     : 0x55AA        //同步码
    quint16 syn_code;

    //! \brief          : 1     : 0x3A          //帧类型
    quint8 frame_type;

    //! \brief          : 0.01  : -10~14        //纵向周期变距 前推为正   （°）
    qint16 peri_vari_x          ;

    //! \brief          : 0.01  : -8~8         //横向周期变距 前推为正   （°）
    qint16 peri_vari_y          ;

    //! \brief          : 0.01  : -12~24        //尾浆距   尾桨增距为正  （°）
    qint16 tail_pitch           ;

    //! \brief          : 0.01  : -2~16         //总距    提距为正    （°）
    qint16 total_dist           ;

    //! \brief          : 0.5   : 0~110         //总距油门开度    （°）
    quint8  total_dist_throttle ;

    //! \brief          : 0.25  : 0~56          //独立油门开度    （°）
    quint8 indep_throttle ;

    //! \brief      连续指令回报      源码转发
    qint8 instr_ret[10];

    //! \brief          : 0.5   : 0~100         //风速    （m/s）
    quint8 wind_speed           ;

    //! \brief          : 2     : -180~180      //风向    （m/s)
    qint8  wind_direction       ;

    //! \brief          : 0.2   : -400~6550     //卫星高度      （m）
    qint16 satellite_altitude   ;

    //! \brief          : 1     : 1~50          //当前飞行航线号
    quint8 route_number         ;

    //! \brief          : 1     : 1~50          //当前飞行航点号
    quint8 waypoint_number      ;

    //! \brief      重规划评估结果
    evaluation_result result;

    //! \brief      在线规划航线状态
    plan_route_state plan_state;

    //! \brief          : 1     : 0~255         //效验和
    quint8 check                ;

}s3A;
//}

//s3B/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s3B {

//! \brief      着舰阶段
typedef struct Structlanding_ship_stage
{
    //! \brief      着舰子阶段       0000=N/A        0001=返舰盘旋     0010=进场进近
    //!                             0011=减速下滑   0100=悬停跟进      0101=跟进上舰
    //!                             0110=舰面下降   0111=降落着舰
	quint8 stage : 4;
    //! \brief      着舰复飞子阶段     00=N/A      01=复飞离舰船        10=复飞航行
	quint8 flyover_stage : 2;

    //! \brief      运动信息          0=失效        1=有效
	quint8 run_info : 1;

    //! \brief      内部引导有效性     0=失效        1=有效
	quint8 guiding : 1;

}landing_ship_stage;

//! \brief      着舰引导信息源
typedef struct Structguiding_msg
{
    //! \brief      引导信息源       000=全部失效                001=雷达引导
    //!                             010=惯性差分组合引导        011=惯性雷达组合引导
    //!                             100=RTK差分引导             101=位置差分引导
	quint8 msg : 3;

    //! \brief      引导模式        00=无效   01=外部引导 10=内部引导
	quint8 guding_mode : 2;

    //! \brief      相对高度标志      0=地/海面      1=舰面
	quint8 height_flags : 1;

    //! \brief      自动着舰流转标志    00=等待       01=指令流转     10=自动流转
	quint8 flow_flags : 2;

}guiding_msg;

//! \brief      着舰指令响应结果
typedef struct Structresponse_result_zj
{
    //! \brief      响应结果        0=执行成功 | 1=执行失败
	quint8 result : 1;
    //! \brief      着舰坐标系       0=甲板稳定坐标系 | 1=甲板固联坐标系
	quint8 coord_sys : 1;
    //! \brief      舰船惯导源       0=前区/惯导1 | 1=后区/惯导2 | 2=无效
	quint8 inertia : 2;
    //! \brief      舰船惯导工作模式    0=组合 | 1=阻尼 | 2=无效
	quint8 inertial_mode : 2;
    //! \brief      着舰控制状态       0=正常控制 | 1=增强控制
	quint8 control_status : 1;
    //! \brief      备用
	quint8 backup : 1;


}response_result_zj;

//! \brief      引导设备状态
typedef struct Structdevice_status
{
    //! \brief      雷达引导源              00=无效 | 01=综合CNI | 10=U链 | 11=应答机
	quint8 radar_guide : 2;
    //! \brief      惯性差分组合引导源       00=无效 | 01=激光制导 | 10=光纤制导
	quint8 inertia_diff_guide : 2;
    //! \brief      惯性雷达组合引导源       00=无效 | 01=激光制导 | 10=光纤制导
	quint8 inertia_radar_guide : 2;
    //! \brief      RTK差分引导源           00=无效 | 01=激光制导 | 10=光纤制导
	quint8 RTK_guide : 2;
}device_status;


//! \brief      雷达引导状态
typedef struct Structguiding_status
{

    //! \brief      设备状态        00=无效 | 01=正常 | 10=预警 | 11=故障
	quint8 device_status : 2;
    //! \brief      复飞决策        00=正常 | 01=环境警告 | 10=N/A | 11=飞机状态警告
	quint8 flight_decision : 2;
    //! \brief      备用
	quint8 backup : 1;
    //! \brief      着舰模式        0=鱼叉辅助 | 1=自由着舰
	quint8 mode : 1;
    //! \brief      着舰时机预测结果    00=允许着舰 | 01=警告 | 10=禁止着舰
	quint8 predictive_result : 2;

}guiding_status;

//! \3B 帧类别3B （着舰引导信息）  频率10Hz
typedef struct Structs3B
{
    //! \brief          : 1     : 0x55AA        //同步码
    quint16 syn_code;

    //! \brief          : 1     : 0x3B          //帧类型
    quint8 frame_type;

    //! \brief          : 0.05  : -400km~400km        //着舰相对纵向位置 舰船坐标系   （m）
    qint8 relat_pos_x[3];

    //! \brief          : 0.05  : -400km~400km        //着舰相对侧向位置 舰船坐标系   （m）
    qint8 relat_pos_y[3];

    //! \brief          : 100/(2^15-)   : -100~100      //着舰相对纵向速度  舰船坐标系   （m/s）
    qint16 relat_speed_x            ;

    //! \brief          : 20/(2^7-1)    : -20~20        //着舰相对侧向速度      舰船坐标系   （m/s）   大于20发20 小于-20发-20
    qint8 relat_speed_y             ;

    //! \brief          : 10/(2^7-1)        : -10~10        //着舰相对垂向速度      舰船坐标系       （m/s）   大于10发10，小于-10发-10
    qint8 relat_speed_z             ;

    //! \brief          : 1000/(2^15-1)     : -100~1000     //着舰相对高度        舰船坐标系       （m）
    qint16 relat_pos_z              ;

    //! \brief          : 0.05          : 0~10          //差分水平误差        （m） 组合A支路水平定位精度HPE大于10m发送10m
    qint8 diff_error_x              ;

    //! \brief          : 0.05          : 0~10          //差分垂直误差        （m） 组合A支路水平定位精度HPE大于10m发送10m
    qint8 diff_error_z              ;

    //! \brief          : 60/120        : -60~60        //着舰轨迹侧向偏差  着舰剖面坐标系 （m） .
    qint8 traject_devia_y           ;

    //! \brief          : 60/120        : -60~60        //着舰轨迹高度偏差  着舰剖面坐标系 （m） .
    qint8 traject_devia_z           ;

    //! \brief      着舰阶段
    landing_ship_stage stage;

    //! \brief      着舰引导信息源
    guiding_msg guid_msg;

    //! \brief          : 0.1       : 0~20              //静息期预测值
    qint8 predictive_value ;

    //! \brief      激光惯导组合A工作模式
    //!                 0x5A=N/A | 0x5B=位置差分 | 0x1C=惯性/gps伪距差分 | 0x2C=惯性/gps载波相位差分（浮点）
    //!                 0x3C=惯性/gps载波相位差分（定点） | 0x4C=惯性/北斗民码伪距差分 | 0x5C=惯性/北斗民码载波相位差分（浮点）
    //!                 0x6C=惯性/北斗民码载波相位差分（定点） | 0x7C=惯性/北斗军码伪距差分 | 0x8C=惯性/北斗军码载波相位差分（浮点）
    //!                 0x9C=惯性/北斗军码载波相位差分（定点）
    qint8 laser_inert_navi_a        ;

    //! \brief      激光惯导组合A工作模式
    //!             0x5A=N/A | 0x5B=位置差分 | 0x5C=惯性/雷达
    qint8 laser_inert_navi_b        ;

    //! \brief      光纤惯导组合A工作模式
    //!                 0x5A=N/A | 0x5B=位置差分 | 0x1C=惯性/gps伪距差分 | 0x2C=惯性/gps载波相位差分（浮点）
    //!                 0x3C=惯性/gps载波相位差分（定点） | 0x4C=惯性/北斗民码伪距差分 | 0x5C=惯性/北斗民码载波相位差分（浮点）
    //!                 0x6C=惯性/北斗民码载波相位差分（定点） | 0x7C=惯性/北斗军码伪距差分 | 0x8C=惯性/北斗军码载波相位差分（浮点）
    //!                 0x9C=惯性/北斗军码载波相位差分（定点）
    qint8 optical_fiber_inert_navi_a    ;

    //! \brief      光纤惯导组合A工作模式
    //!             0x5A=N/A | 0x5B=位置差分 | 0x5C=惯性/雷达
    qint8 optical_fiber_inert_navi_b    ;

    //! \brief          : 0.01      : 0~360     //舰船航向角 (°)
    quint16 course_angle                ;

    //! \brief      着舰指令响应结果
    response_result_zj result;

    //! \brief      引导设备状态
    device_status dev_status;

    //! \brief      雷达引导状态
    guiding_status guid_status;

    //! \brief          : 1     : 0~255         //效验和
    quint8 check                        ;

}s3B;

//}

//s4A/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s4A {


//! \brief      第一级控制状态
typedef struct Structcontrol_status_1
{
    //! \brief      控制模式    00=遥控 | 01=指令控制 | 10=自主控制 | 11=N/A
	quint8 control_mode : 2;

    //! \brief      人工操纵手柄权限    1=小权限操纵 | 0=全权限操纵
	quint8 handle_authority : 1;

    //! \brief      预留
	quint8 backup : 1;

    //! \brief      启动模式        0=地面加电 | 1=空中/复位
	quint8 start_mode : 1;

    //! \brief      空地状态（飞管）    0=空中 | 1=地面
	quint8 status : 1;

    //! \brief      自检模式（平台）    00=无自检 | 01=PBIT | 10=MBIT | 11=N/A
	quint8 self_checking : 2;

}control_status_1;

//! \brief      第二级控制状态
typedef struct Structcontrol_status_2
{
    //! \brief      飞行阶段
    //!                             0000=等待 | 0001=舰基起飞 | 0010=陆基起飞 | 0011=航行
    //!                             0100=机动 | 0101=着舰 | 0110=着舰复飞 | 0111=着路
    //!                             1000=备用 | 1001=自毁 | 1010=自转下滑 | 1011=撤收
    //!                             1100=指令飞行 | 1101=遥控飞行
	quint8 flight_stage : 4;

    //! \brief      组合动作
    //!                             0000=空 | 0001=陆基自动启动 | 0010=自动着陆 | 0011=备用
    //!                             0100=自动悬停 | 0101=自动返航 | 0110=定点悬停 | 0111=舰基自动起飞
    //!                             1000=跟进上舰 | 1001=舰面下降 | 1010=降落着舰 | 1011=离舰出航
    //!                             1100=悬停跟进 | 1101=左回转 | 1110=右回转 | 1111=自动改平
	quint8 comb_action : 4;

}control_status_2;

//! \brief      第三级控制状态
typedef struct Structcontrol_status_3
{

    //! \brief      高度轴
    //!                             00=空 | 01=平飞 | 10=爬升 | 11=下降
	quint8 z_axle : 2;

    //! \brief      纵轴
    //!                             000=空 | 001=悬停前飞 | 010=悬停倒飞 | 011=悬停侧飞
    //!                             100=均速 | 101=加速 | 110=减速
	quint8 x_axle : 3;

    //! \brief      横侧轴
    //!                             000=空 | 001=左转 | 010=右转 | 定向
	quint8 y_axle : 3;


}control_status_3;

//! \brief      第四级控制状态
typedef struct Structcontrol_status_4
{
    //! \brief      航向轴
    //!                             00=姿态保持 | 01=航向保持 | 10=备用 | 11=区域导航
	quint8 heading_axle : 2;

    //! \brief      横滚轴
    //!                             000=姿态保持 | 001=地速保持 | 010=位置保持 | 011=相对位置保持
    //!                             100=相对速度保持
	quint8 roll_axle : 2;

    //! \brief      俯仰轴
    //!                             000=姿态保持 | 001=空速保持 | 010=地速保持 | 011=位置保持
    //!                             100=相对位置保持 | 101=相对速度保持
	quint8 pit_axle : 2;

    //! \brief      总距轴
    //!                             0000=空 | 0001=相对高度保持 | 0010=绝对高度保持 | 0011=垂速保持
    //!                             0100=自动下滑 | 0101=陆基自动起飞 | 0110=舰基自动起飞 | 0111=自动着舰
    //!                             1000=自动着陆
	quint8 total_distance_axis : 4;

}control_status_4;

//! \brief      应急故障处置模式
typedef struct Structtroubleshooting_mode
{
    //! \brief      应急故障处置模式
    //!                             000=人工故障处置模式 | 001=自动故障处置模式 | 010=备用 | 011=备用
	quint8 emerg_fail :3;

    //! \brief      测控失效处置模式
    //!                             0=返航优先 | 1=任务优先
	quint8 measu_fail : 1;

    //! \brief      飞控计算机背板总线故障
    //!                             0=正常 | 1=故障
	quint8 bus_fail : 1;

    //! \brief      系统故障等级
    //!                             0x00=无故障 | 0x01=轻微故障（系统姿态Ⅰ） | 0x02=轻度故障（系统姿态Ⅱ）
    //!                             0x03=严重故障（系统姿态Ⅲ） | 0x04=灾难故障（系统姿态Ⅳ）
	quint8 fail_grade : 3;

}troubleshooting_mode;

//! \brief      应急故障处置方案
typedef struct Structtroubleshooting_scheme
{
    //! \brief      应急故障处置方案
    //!                             0x00=无 | 0x12=地面异常处置 | 0x14=常规故障处置 | 0x16=任务失败故障处置 | 0x18=起降段一般故障处理
    //!                             0x1A=起降段特殊故障处置 | 0x1C=最短时间应急返航 | 0x1E=最省燃油应急返航 | 0x20=最短路径应急返航
    //!                             0x22=最安全应急返航 | 0x24=直接应急返航 | 0x26=测控失效故障处置 | 0x28=自转下滑处置 | 0x2A=紧急迫降处置
    //!                             0x2C=自毁处置 | 0x2E=信息安全处置 | 0x30=高空应急返航 | 0x32=中空应急返航 | 0x34=低空应急返航
    //!                             0x36=正常航线返航 | 0x38=加功率飞行 | 0x38=降功率飞行
	quint8 disposal_plan;

}troubleshooting_scheme;

//! \brief       飞控计算机故障字
typedef struct Structflying_computer_malfunc
{

    //! \brief      计算机cpu故障
    //!                             00=无故障 | 01=cpu一次故障 | 10=cpu二次故障
	quint8 cpu_fail : 2;

    //! \brief      接口故障
    //!                             00=无故障 | 01=接口一次故障 | 10=接口二次故障
	quint8 inter_fail : 2;

    //! \brief      电源故障
    //!                             00=无故障 | 01=电源一次故障 | 10=电源二次故障
	quint8 supply_fail : 2;

    //! \brief      输出回绕故障
    //!                             0=正常 | 1=故障
	quint8 outloop_fail : 1;

    //! \brief      备用总线故障
    //!                             0=正常 | 1=故障
	quint8 bus_fail : 1;
}flying_computer_malfunc;

//! \brief       主/尾浆舵机故障字
typedef struct Structsteering_malfunc
{

    //! \brief      俯仰舵机故障
    //!                             00=无故障 | 01=俯仰舵机一次故障 | 10=俯仰舵机二次故障
	quint8 pit_fail : 2;

    //! \brief      左横滚舵机故障
    //!                             00=无故障 | 01=左横滚舵机一次故障 | 10=左横滚舵机二次故障
	quint8 l_roll_fail : 2;

    //! \brief      右横滚舵机故障
    //!                             00=无故障 | 01=右横滚舵机一次故障 | 10=右横滚舵机二次故障
	quint8 r_roll_fail : 2;

    //! \brief      尾桨舵机故障
    //!                             00=无故障 | 01=尾桨舵机一次故障 | 10=尾桨舵机二次故障
	quint8 tail_oar_fail : 2;

}steering_malfunc;

//! \brief       伺服控制器故障字
typedef struct Structservo_control_malfunc
{

    //! \brief      伺服控制器故障
    //!                             00=无故障 | 01=控制器一次故障 | 10=控制器二次故障
	quint8 controller_fail : 2;

    //! \brief      伺服电源故障
    //!                             00=无故障 | 01=电源一次故障 | 10=电源二次故障
	quint8 supply_fail : 2;

    //! \brief      主rs485通信故障
    //!                             00=无故障 | 01=一次故障 | 10=二次故障
	quint8 main485_fail : 2;

    //! \brief      备用rs485通信故障
    //!                             00=无故障 | 01=一次故障 | 10=二次故障
	quint8 secondary485_fail : 2;
}servo_control_malfunc;

//! \brief       油门舵机故障字
typedef struct Structthrottle_steering_malfunc
{
    //! \brief      独立油门舵机故障
    //!                             00=无故障 | 01=独立油门一次故障 | 10=独立油门二次故障
	quint8 independence_fail : 2;

    //! \brief      总距油门舵机故障
    //!                             00=无故障 | 01=总距油门一次故障 | 10=总距油门二次故障
	quint8 total_dist_fail : 2;

    //! \brief      备份
	quint8 backup : 4;
}throttle_steering_malfunc;

//! \brief       油门控制器故障字
typedef struct Structthrottle_control_malfunc
{

    //! \brief      油门控制器故障
    //!                             00=无故障 | 01=控制器一次故障 | 10=控制器二次故障
	quint8 controller_fail : 2;

    //! \brief       油门电源故障
    //!                             00=无故障 | 01=电源一次故障 | 10=电源二次故障
	quint8 supply_fail : 2;

    //! \brief      主rs485通信故障
    //!                             00=无故障 | 01=一次故障 | 10=二次故障
	quint8 rs485_fail : 2;

    //! \brief      备份
	quint8 backup : 2;

}throttle_control_malfunc;

//! \brief      PBIT无法投入/异常退出原因/状态
typedef struct StructPBIT_exit_status
{

    //! \brief      机轮承载为空中
    //!                             1=申报 | 0=不申报
	quint8 wheel_air : 1;

    //! \brief      发动机不为停车
    //!                             1=申报 | 0=不申报
	quint8 no_parking : 1;

    //! \brief      发动机转速大于300
    //!                             1=申报 | 0=不申报
	quint8 rotations_300 : 1;

    //! \brief      发动机转速小于50
    //!                             1=申报 | 0=不申报
	quint8 rotations_50 : 1;

    //! \brief      备用
	quint8 backup : 1;

    //! \brief      总距大于5度
    //!                             1=申报 | 0=不申报
	quint8 total_dist_5 : 1;

    //! \brief      已处于MBIT模式
    //!                             1=申报 | 0=不申报
	quint8 mbit_mode : 1;

    //! \brief      PBIT进入指令表决不一致
    //!                             1=申报 | 0=不申报
	quint8 instruction_vary : 1;

}PBIT_exit_status;

//! \brief      PBIT检测进度/结果申报
typedef struct StructPBIT_progress_declare
{
    //! \具体错误位标识，请查阅相关文件
    qint16 declare ;
}PBIT_progress_declare;

//! \brief      PUBIT结果
typedef struct StructPUBIT_results
{
	quint8 results ;
}PUBIT_results;

//! \brief      飞行子阶段
typedef struct Structflight_stage
{
	quint8 results ;
}flight_stage;

//! \4A 帧类别4A （飞行阶段、工作模式、故障状态信息）  频率5Hz
typedef struct Structs4A
{
    //! \brief          : 1     : 0x55AA        //同步码
    quint16 syn_code;

    //! \brief          : 1     : 0x3B          //帧类型
    quint8 frame_type;

    //! \brief          : 1     : 0~65535       //飞机地址   平台编号
    quint16 aircraft_address         ;

    //! \brief          : 1     : 0~65535       //站地址  // 控制权所在 控交用到
    quint16 station_address         ;

    //! \brief      第一级控制状态
    control_status_1 control_1;

    //! \brief      第二级控制状态
    control_status_2 control_2;

    //! \brief      第三级控制状态
    control_status_3 control_3;

    //! \brief      第四级控制状态
    control_status_4 control_4;

    //! \brief           : 0.1      : 0~360     //应飞航向      （°）
    quint16 fly_course          ;

    //! \brief      应急故障处置模式
    troubleshooting_mode troub_mode;

    //! \brief      应急故障处置方案
    troubleshooting_scheme troub_scheme;

    //! \brief      : 0.2       : -400~6550         //气压高度  （m）
    qint16 barom_height         ;

    //! \brief       飞控计算机故障字
    flying_computer_malfunc malfunc_flying_computer;

    //! \brief       主/尾浆舵机故障子
    steering_malfunc malfunc_steering;

    //! \brief       伺服控制器故障字
    servo_control_malfunc malfunc_servo_control;

    //! \brief       油门舵机故障字
    throttle_steering_malfunc malfunc_throttle_steering;

    //! \brief       油门控制器故障字
    throttle_control_malfunc malfunc_throttle_control;

    //! \brief       剩余盘旋圈数/时间      待与602确认
    qint8 circle_time           ;

    //! \brief      : 70/(2^8-1)        : 0~70      //俯仰舵机电流    （A）
    quint8 pit_steering_current ;

    //! \brief      : 300/(2^7-1)        : -55~280      //俯仰舵机温度    （°C）
    quint8 pit_steering_tempe   ;


    //! \brief      PBIT无法投入/异常退出原因/状态
    PBIT_exit_status pbit_exit_status;
    //! \brief      PBIT检测进度/结果申报
    PBIT_progress_declare pbit_progress_declare;
    //! \brief      PUBIT结果
    PUBIT_results pubit_results;
    //! \brief      飞行子阶段
    flight_stage stage;

    //! \brief          : 1     : 0~255         //效验和
    quint8 check            ;

}s4A;




//}

//s4B/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s4B {


//! \brief      设备链路状态字1
typedef struct Structdevice_link_status_1
{
	quint8 device_link_status ;
}device_link_status_1;

//! \brief      设备链路状态字2
typedef struct Structdevice_link_status_2
{
	quint8 device_link_status ;
}device_link_status_2;

//! \brief      设备链路状态字3
typedef struct Structdevice_link_status_3
{
	quint8 device_link_status ;
}device_link_status_3;

//! \brief      传感器故障字1
typedef struct Structsensor_fault_1
{
	quint8 sensor_fault ;
}sensor_fault_1;

//! \brief      传感器故障字3
typedef struct Structsensor_fault_2
{
	quint8 sensor_fault ;
}sensor_fault_2;

//! \brief      传感器故障字3
typedef struct Structsensor_fault_3
{
	quint8 sensor_fault ;
}sensor_fault_3;

//! \brief      动力系统状态字1
typedef struct Structpower_system_status_1
{
	quint8 power_system_status ;
}power_system_status_1;

//! \brief      动力系统状态字2
typedef struct Structpower_system_status_2
{
	quint8 power_system_status ;
}power_system_status_2;

//! \brief      旋翼系统状态字
typedef struct Structrotor_system_status
{
	quint8 rotor_system_status ;
}rotor_system_status;

//! \brief      航电任务状态字
typedef struct Structavionics_status
{

	quint8 avionics_status ;
}avionics_status;

//! \brief      发动机状态字
typedef struct Structengine_status
{
	quint8 engine_status ;
}engine_status;

//! \brief      机电系统状态字
typedef struct Structelectro_system_status
{
	quint8 electro_system_status_1 ;
	quint8 electro_system_status_2 ;
}electro_system_status;

//! \brief      传感器信号源1
typedef struct Structsensor_source_1
{
	quint8 sensor_source ;
}sensor_source_1;

//! \brief      传感器信号源2
typedef struct Structsensor_source_2
{
	quint8 sensor_source ;
}sensor_source_2;

//! \brief      传感器信号源3
typedef struct Structsensor_source_3
{
	quint8 sensor_source ;
}sensor_source_3;

//! \brief      离散输出字1
typedef struct Structdiscrete_output_1
{
	quint8 discrete_output ;
}discrete_output_1;

//! \brief      离散输出字2
typedef struct Structdiscrete_output_2
{
	quint8 discrete_output ;
}discrete_output_2;

//! \brief      离散输出字3
typedef struct Structdiscrete_output_3
{
	quint8 discrete_output ;
}discrete_output_3;

//! \brief      离散输出字4
typedef struct Structdiscrete_output_4
{
	quint8 discrete_output ;
}discrete_output_4;


//! \4B 帧类别4B （飞控系统外部交联设备状态）  频率5Hz
typedef struct Structs4B
{
    //! \brief          : 1     : 0x55AA        //同步码
    quint16 syn_code;

    //! \brief          : 1     : 0x4B          //帧类型
    quint8 frame_type;

    //! \brief          设备链路状态字1
    device_link_status_1 device_link_1;

    //! \brief          设备链路状态字2
    device_link_status_2 device_link_2;

    //! \brief          设备链路状态字3
    device_link_status_3 device_link_3;

    //! \brief          传感器故障字1
    sensor_fault_1 sensor_1;

    //! \brief          传感器故障字2
    sensor_fault_2 sensor_2;

    //! \brief          传感器故障字3
    sensor_fault_3 sensor_3;

    //! \brief      动力系统状态字1
    power_system_status_1 power_system_1;

    //! \brief      动力系统状态字2
    power_system_status_2 power_system_2;

    //! \brief      备用
    qint8 backup_1 ;

    //! \brief      旋翼系统状态字
    rotor_system_status rotor_system;

    //! \brief      航电任务状态字
    avionics_status avionics;

    //! \brief      发动机状态字
    engine_status engine;

    //! \brief     :0.5     :-50~50         //俯仰舵机反馈    正：伸； 负：缩(mm)
    qint8 pit_turn ;

    //! \brief      : 0.5   : -50~50        //左横滚舵机反馈     正：伸； 负：缩(mm)
    qint8 l_roll_turn ;

    //! \brief      : 0.5   : -50~50        //右横滚舵机反馈     正：伸； 负：缩(mm)
    qint8 r_roll_turn ;

    //! \brief      : 0.5   : -50~50        //航向舵机反馈     正：面对摇臂顺时针（°）
    qint8 course_turn ;

    //! \brief      : 0.5   : -50~50        //总距油门舵机指令     （%）
    qint8 total_dist_turn ;

    //! \brief      : 0.5   : -50~50        //独立油门舵机指令     （%）
    qint8 independence_turn ;

    //! \brief      备用
    qint8 backup_2 ;

    //! \brief      机电系统状态字
    electro_system_status electro_system;

    //! \brief      传感器信号源1
    sensor_source_1 source_1;

    //! \brief      传感器信号源2
    sensor_source_2 source_2;

    //! \brief      传感器信号源3
    sensor_source_3 source_3;

    //! \brief      离散输出字1
    discrete_output_1 outpu1;

    //! \brief      离散输出字2
    discrete_output_2 outpu2;

    //! \brief      离散输出字3
    discrete_output_3 outpu3;

    //! \brief      离散输出字4
    discrete_output_4 outpu4;

    //! \brief          : 1     : 0~255         //效验和
    quint8 check ;

}s4B;

//}

//s4C/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s4C {

//! \4C 帧类别4C （发动机参数、助降参数、飞机重量等监控信息）  频率5Hz
typedef struct Structs4C
{
    //! \brief          : 1     : 0x55AA        //同步码
    quint16 syn_code;

    //! \brief          : 1     : 0x4C          //帧类型
    quint8 frame_type;

    //! \brief          :0.01       : 0~120.5   //燃气发生器转速Ng     （%）
    quint16 ng ;

    //! \brief          : 0.2       : 0~10000   //动力涡轮转速Np  （rpm)
    quint16 np ;

    //! \brief          : 0.1       : 0~1000    //旋翼转速  （rpm）
    quint16 rotor_speed ;

    //! \brief          : 0.1       : 0~16      //发动机滑油压力   （bar）
    quint8 engine_lubric_pressure ;

    //! \brief          : 0.01      : 0~0.5     //供油管路燃油压力
    quint16 pipe_fuel_pressure ;

    //! \brief          : 0.1       : -55~150   //发动机滑油温度 (°C)
    qint16  engine_lubric_temperature ;

    //! \brief          : 1         : 0~1000    //主减滑油压力    （KPa）
    quint16 main_lubric_pressure ;

    //! \brief          : 2         : -40~170   //主减滑油温度    （°C）
    qint8 main_lubric_temperature ;

    //! \brief          : 2         : -40~170   //尾减滑油温度    （°C）
    qint8 tail_lubric_temperature ;

    //! \brief          : 0.5       : 0~600     //前油箱总油量    （Kg）
    quint16 front_oiltank ;

    //! \brief          : 0.5       : 0~600     //后油箱总油量    （Kg）
    quint16 back_oiltank ;

    //! \brief          : 0.5       : 0~1000    //总油量   （Kg）
    quint16 oil_quantity ;

    //! \brief          : 50/(2^8-1)    : 0~50  //飞控系统电源电压  （V）
    quint8 power_voltage ;

    //! \brief          : 0.1       : -55~1100  //T45温度计算值
    qint16 t45 ;

    //! \brief          : 0.1       : -110~110  //发动机扭矩 （%）
    qint16 torque ;

    //! \brief          : 0.1       : -3000~3000    //无线电高度 (m)
    qint16 radio_height ;

    //! \brief          : 1     : 0~255         //效验和
    quint8 check ;

}s4C;
//}

//s4D/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s4D {


//! \brief          传输内容类型/摇调/航线指令响应结果
typedef struct Structresponse_result
{
    //! \brief      传输内容类型
    //!                             0x01=摇调指令回报 | 0x02=航线规划下传
	quint8 type : 6;

    //! \brief      摇调/航线指令执行结果
    //!                             00=执行成功 | 01=响应条件不满足执行失败
    //!                             10=设置参数不合理执行失败 | 11=其他原因执行失败
	quint8 result : 2;

}response_result;


//! \4D 帧类别4D （遥控指令回报）  频率5Hz
typedef struct Structs4D
{
    //! \brief          : 1     : 0x55AA        //同步码
    quint8 syn_code_0 ;
    quint8 syn_code_1 ;

    //! \brief          : 1     : 0x4D          //帧类型
    quint8 frame_type ;

    //! \brief          传输内容类型/摇调/航线指令响应结果
    response_result result;

    quint8 zhiLingHuiBao;
    //! \brief          摇调参数回报  优先响应摇调指令回报
    qint8 data[26];

    //! \brief          : 1     : 0~255         //效验和
    quint8 check ;

}s4D;


//}

//s5A/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s5A {

//! \brief          航姿状态字   备份航姿
typedef struct Structroute_status
{
    //! \brief      备份航姿工作状态
    //!                         00=准备状态 | 01=对准状态 | 10=纯惯性导航状态 | 11=组合导航状态
	quint8 backup_route : 2;
    //! \brief      厂家自定义
    //!                         00=NA | 01=未修正 | 10=已修正 | 11=修正正确
	quint8 manufacturer_custom : 2;

    //! \brief      备用
	quint8 backup : 4;

}route_status;

//! \brief          故障字 备份航姿
//!                                  1位解算        0=正常 | 1=故障
typedef struct Structmalfunction
{

    //! \brief      俯仰角失效
	quint8 pit : 1;

    //! \brief      横滚角失效
	quint8 roll : 1;

    //! \brief      航向角失效
	quint8 course : 1;

    //! \brief      俯仰角速率失效
	quint8 pit_rate : 1;

    //! \brief      横滚角速率失效
	quint8 roll_rate : 1;

    //! \brief      航向角速率失效
	quint8 course_rate : 1;

    //! \brief      横轴加速度失效
	quint8 x_acceleration : 1;

    //! \brief      纵轴加速度失效
	quint8 y_acceleration : 1;

    //! \brief      法轴加速度失效
	quint8 z_acceleration : 1;

    //! \brief      备用
	quint8 backup : 7;

}malfunction;

//! \brief          前油量传感器状态字
typedef struct Structfront_oiltank_status
{
    //! \brief      油量传感信号器测量电路故障
	quint8 circuit_failure : 1;

    //! \brief      油量传感信号器cpu故障
	quint8 cpu_failure : 1;

    //! \brief      油量传感信号器油位传感器故障
	quint8 position_failure : 1;

    //! \brief      油量传感信号器温度传感器故障
	quint8 temperature_failure : 1;

    //! \brief      油量传感信号器eeprom故障
	quint8 eeprom_failure : 1;

    //! \brief      与油量传感信号器通信故障
	quint8 comm_failure : 1;

    //! \brief      备用
	quint8 backup : 2;

}front_oiltank_status;

//! \brief          后油量传感器状态字
typedef struct Structback_oiltank_status
{
    //! \brief      油量传感信号器测量电路故障
	quint8 circuit_failure : 1;

    //! \brief      油量传感信号器cpu故障
	quint8 cpu_failure : 1;

    //! \brief      油量传感信号器油位传感器故障
	quint8 position_failure : 1;

    //! \brief      油量传感信号器温度传感器故障
	quint8 temperature_failure : 1;

    //! \brief      油量传感信号器eeprom故障
	quint8 eeprom_failure : 1;

    //! \brief      备用
	quint8 backup : 3;

}back_oiltank_status;


//! \5A 帧类别5A （时戳、备份航姿信息）  频率4Hz
typedef struct Structs5A
{
    //! \brief          : 1     : 0x55AA        //同步码
    quint16 syn_code;

    //! \brief          : 1     : 0x5A          //帧类型
    quint8 frame_type;

    ////////////////////////////////////////////////////////////////////
    /// 年信息只包含年的后两位，接收端自动+2000；来源组合导航
    //! \brief          : 1     : 0~255          //年 始终为正（year）
    qint8 yesr ;
    //! \brief          : 1     : 0~12           //月 始终为正（month）
    qint8 month ;
    //! \brief          : 1     : 1~31           //日 始终为正（day）
    qint8 day ;
    //! \brief          : 1     : 0~23           //时 始终为正（h）
    qint8 hours ;
    //! \brief          : 1     : 0~59           //分 始终为正（min）
    qint8 min ;
    //! \brief          : 1     : 0~59           //秒 始终为正（sec）
    qint8 sec ;
    ///////////////////////////////////////////////////////////////////

    //! \brief          航姿状态字   备份航姿
    route_status route;

    //! \brief          故障字
    malfunction malfunc;

    //! \brief          : 90/(2^15-1)   : -90~90        //俯仰角 抬头为正 （°）
    qint16 pit ;

    //! \brief          : 180/(2^15-1)  : -180~180      //横滚角 右倾为正  （°）
    qint16 roll ;

    //! \brief          : 180/(2^15-1)  : -180~180      //航向角 右偏为正  （°）
    qint16 course ;

    //! \brief          : 200/(2^15-1)  : -200~200      //俯仰角速率 抬头为正    （°/s）
    qint16 pit_rate ;

    //! \brief          : 200/(2^15-1)  : -200~200      //横滚角速率 右倾为正    （°/s）
    qint16 roll_rate ;

    //! \brief          : 200/(2^15-1)  : -200~200      //航向角速率 右偏为正    （°/s）
    qint16 course_rate ;

    //! \brief          : 0.2           : ±25           //横轴加速度 向前为正    （m/s^2)
    qint8 x_acceleration ;

    //! \brief          : 0.2           : ±25           //纵轴加速度 向前为正    （m/s^2)
    qint8 y_acceleration ;

    //! \brief          : 0.2           : ±25           //法轴加速度 向前为正    （m/s^2)
    qint8 z_acceleration ;

    //! \brief          前油量传感器状态字
    front_oiltank_status front_status;

    //! \brief          后油量传感器状态字
    back_oiltank_status back_status;

    //! \brief          : 0.001         : 0~50          //主减振动检测    主减上端面低频振动量值 （g）
    quint16 jarring_testing ;


    //! \brief          : 1     : 0~255         //效验和
    quint8 check ;

}s5A;

//}

//s5B/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s5B {

//! \brief          激光惯导状态字1
typedef struct Structlaser_status_1
{
	quint8 laser_status ;
}laser_status_1;

//! \brief          激光/光纤惯导状态字2
typedef struct Structlaser_optic_status_2
{
	quint8 laser_optic_status ;
}laser_optic_status_2;

//! \brief          激光惯导有效字
typedef struct Structlaser_inertial_guide
{
	quint8 laser_inertial_guide ;
}laser_inertial_guide;

//! \brief          激光惯导故障字
typedef struct Structlaser_inertial_navig_fault
{
    quint16 laser_inertial_navig_fault ;
}laser_inertial_navig_fault;

//! \brief          光纤惯导状态字1
typedef struct Structoptic_status_1
{
	quint8 optic_status ;
}optic_status_1;

//! \brief          激光差分有效性
typedef struct Structlaser_diffe_effect
{
	quint8 laser_diffe_effect ;
}laser_diffe_effect;

//! \brief          光纤惯导有效字
typedef struct Structoptic_inertial_guide
{
	quint8 optic_inertial_guide ;
}optic_inertial_guide;

//! \brief          光纤惯导故障字
typedef struct Structoptic_inertial_navig_fault
{
    quint16 optic_inertial_navig_fault ;

}optic_inertial_navig_fault;

//! \brief          大气/无高状态字
typedef struct Structradio_height_mode
{
	quint8 radio_height_mode ;
}radio_height_mode;

//! \brief          激光差分故障字1
typedef struct Structlaser_diffe_fault_1
{
	quint8 laser_diffe_fault ;
}laser_diffe_fault_1;

//! \brief          助降状态字1
typedef struct Structhelp_down_status_1
{
	quint8 help_down_status ;
}help_down_status_1;

//! \brief          助降状态字2
typedef struct Structhelp_down_status_2
{
	quint8 help_down_status ;

}help_down_status_2;

//! \brief          HUMS状态字1
typedef struct Structhums_status_1
{
    quint16 hums_status ;
}hums_status_1;

//! \brief          危险级信号
typedef struct Structdanger_signal
{
	quint8 danger_signal ;
}danger_signal;

//! \brief          警告级信号
typedef struct Structwarning_signal
{
	quint8 warning_signal ;
}warning_signal;

//! \brief          激光差分故障字2
typedef struct Structlaser_diffe_fault_2
{
	quint8 laser_diffe_fault ;
}laser_diffe_fault_2;

//! \brief          注意级信号1
typedef struct Structattention_level_signal_1
{
    quint16 attention_level_signal ;
}attention_level_signal_1;

//! \brief          指示级信号
typedef struct Structindicated_signal
{
	quint8 indicated_signal ;

}indicated_signal;


//! \5B 帧类别5B （基本航电工作状态、助降系统信息等）  频率4Hz
typedef struct Structs5B
{
    //! \brief          : 1     : 0x55AA        //同步码 0~1
    quint16 syn_code;

    //! \brief          : 1     : 0x5B          //帧类型 2
    quint8 frame_type ;

    //! \brief          激光惯导状态字1 3
    laser_status_1 laser_1;

    //! \brief          激光/光纤惯导状态字2 4
    laser_optic_status_2 laser_optic_2;

    //! \brief          激光惯导有效字 5
    laser_inertial_guide laser_inertial;

    //! \brief          激光惯导故障字 6~7
    laser_inertial_navig_fault laser_inertial_navig;

    //! \brief          光纤惯导状态字1 8
    optic_status_1 optic_1;

    //! \brief          激光差分有效性 9
    laser_diffe_effect laser_diffe;

    //! \brief          光纤惯导有效字 10
    optic_inertial_guide  optic_inertial;

    //! \brief          光纤惯导故障字 11~12
    optic_inertial_navig_fault optic_inertial_navig;

    //! \brief          大气/无高状态字 13
    radio_height_mode radio_height;

    //! \brief          激光差分故障字1 14
    laser_diffe_fault_1 laser_diffe_1;

    //! \brief          助降状态字1 15
    help_down_status_1 help_down_1;

    //! \brief          助降状态字2 16
    help_down_status_2 help_down_2;

    //! \brief          : 0.01          : 0~2^8-1   //（鱼叉）工作时间  XX.XX秒  （s） 17
    qint8 harpoon ;

    //! \brief          : 0.01          : 0~300     //助降装置（鱼叉）位置    XXX.XXmm 18~19
    quint16 drop_gear ;

    //! \brief          : 1             : 0~65535   //着舰（工作）次数 20~21
    qint16 descent_number ;

    //! \brief          HUMS状态字1 22~23
    hums_status_1 hums_1;

    //! \brief          危险级信号 24
    danger_signal danger;

    //! \brief          警告级信号 25
    warning_signal warning;

    //! \brief          激光差分故障字2 26
    laser_diffe_fault_2 laser_diffe_2;

    //! \brief          注意级信号1 27~28
    attention_level_signal_1 attention_1;

    //! \brief          指示级信号 29
    indicated_signal indicated;

    //! \brief          : 1             : 0~24      //激光差分结算星数 30
    qint8 satellite_number ;

    //! \brief          : 1     : 0~255         //效验和 31
    quint8 check ;
}s5B;
//}

//s5C/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s5C {

//! \brief          ECU开关量输出
typedef struct Structecu_switch_output
{
	quint8 ecu_switch_output ;
}ecu_switch_output;

//! \brief          ECU故障1
typedef struct Structecu_malfunction_1
{
    quint16 ecu_malfunction_1 ;
}ecu_malfunction_1;

//! \brief          ECU故障2
typedef struct Structecu_malfunction_2
{
    quint16 ecu_malfunction_2 ;
}ecu_malfunction_2;

//! \brief          ECU警告字1
typedef struct Structecu_warning_word_1
{
    quint16 ecu_warning_word_1 ;
}ecu_warning_word_1;

//! \brief          ECU状态字
typedef struct Structecu_status
{
    quint16 ecu_status ;
}ecu_status;

//! \brief          ECU警告字2
typedef struct Structecu_warning_word_2
{
    quint8 ecu_warning_word ;
}ecu_warning_word_2;

//! \brief          动力/传动构型
typedef struct Structtransm_configura
{
	quint8 transm_configura ;
}transm_configura;

//! \brief          激光/光纤差分卫星工作模式
typedef struct Structdiffe_mode
{
	quint8 diffe_mode ;
}diffe_mode;

//! \brief          HUMS状态字2
typedef struct Structhums_status_2
{
	quint8 hums_status ;
}hums_status_2;

//! \brief          应答机状态字
typedef struct Structanswer_mach_status
{
	quint8 answer_mach_status ;
}answer_mach_status;


//! \5C 帧类别5C （发动机电调信息，适用国内发动机）  频率4Hz
typedef struct Structs5C
{
    //! \brief          : 1     : 0x55AA        //同步码
    quint16 syn_code;

    //! \brief          : 1     : 0x5C          //帧类型
    quint8 frame_type ;

    //! \brief          : 0.1       : -12.5~112.5       //CLP位置 （%）
    qint16 clp_pos ;

    //! \brief          : 0.1       : -100~100          //功率裕度  （%）
    qint16 power ;

    //! \brief          : 0.1       : -500~500          //温度裕度  （°）
    qint16 temperature ;

    //! \brief          ECU开关量输出
    ecu_switch_output ecu_switch;

    //! \brief          备用
    qint8 backup_1 ;

    //! \brief          ECU故障1
    ecu_malfunction_1 ecu_malfunc_1;

    //! \brief          ECU故障2
    ecu_malfunction_2 ecu_malfunc_2;

    //! \brief          ECU警告字1
    ecu_warning_word_1 ecu_warning_1;

    //! \brief          ECU状态字
    ecu_status ecu;

    //! \brief          ECU警告字2
    ecu_warning_word_2 ecu_warning_2;

    //! \brief          备用
    qint8 backup_2 ;

    //! \brief          动力/传动构型
    transm_configura transm;

    //! \brief          : 1         : 0~255             //激光惯导卫星星数
    //!                                                     Bit位 0~3：北斗卫星星数； Bit位 4~7：GNSS卫星星数。（当收星个数大于15发送15）
    qint8 laser_satellite_number ;

    //! \brief          : 1         : 0~255             //光纤惯导卫星星数
    //!                                                     Bit位 0~3：北斗卫星星数； Bit位 4~7：GNSS卫星星数。（当收星个数大于15发送15）
    qint8 optical_satellite_number ;

    //! \brief          激光/光纤差分卫星工作模式
    diffe_mode diffe;

    //! \brief          备用
    qint8 backup_3 ;

    //! \brief          : 0.1       : 0~300             //发动发电机电流 来自HUMS （A）
    quint16 generator_current ;

    //! \brief          HUMS状态字2
    hums_status_2 hums_2;

    //! \brief          备用
    qint8 backup_4 ;

    //! \brief          应答机状态字
    answer_mach_status answer_mach;

    //! \brief          : 1     : 0~255         //效验和
    quint8 check ;
}s5C;
//}

//s5D/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s5D {

//! \brief          接触器状态字
typedef struct Structcontactor_status
{
	quint8 contactor_status ;
}contactor_status;


//! \5D 帧类别5D （电气综合管理系统信息）  频率4Hz
typedef struct Structs5D
{
    //! \brief          : 1     : 0x55AA        //同步码
    quint16 syn_code ;

    //! \brief          : 1     : 0x5D          //帧类型
    quint8 frame_type ;

    //! \brief          : 0.1   : 0~50          //应急汇流条电压   （V）
    quint16 emergency_voltage ;

    //! \brief          : 0.1   : 0~50          //任务汇流条电压   （V）
    quint16 task_voltage ;

    //! \brief          : 0.1   : 0~50          //发动机电压   （V）
    quint16 dynamo_voltage ;

    //! \brief          : 1   : -800~400        //发动机电流   （A）
    qint16 dynamo_current ;

    //! \brief          : 0.1   : 0~40          //外电源电压   （V）
    quint16 power_voltage ;

    //! \brief          : 0.1   : 0~40          //电池组1总电压   （V）
    quint16 battery_1_voltage ;

    //! \brief          : 1     : -400~400      //电池组1总电流   按有符号数解，正-放电，负-充电 （A）
    qint16 battery_1_current ;

    //! \brief          : 1     : -60~127       //电池组1总温度   最高位为符号位，负温为1，正温为0， 低7位为源码。 （°C）
    qint8 battery_1_temperature ;

    //! \brief          : 1     : 0~100         //电池组1当前容量   额定容量的百分比表示以50Ah为基准百分比 （%）
    qint8 battery_1_capacity ;

    //! \brief          : 0.1   : 0~40          //电池组2总电压   （V）
    quint16 battery_2_voltage ;

    //! \brief          : 1     : -400~400      //电池组2总电流   按有符号数解，正-放电，负-充电 （A）
    qint16 battery_2_current ;

    //! \brief          : 1     : -60~127       //电池组2总温度   最高位为符号位，负温为1，正温为0， 低7位为源码。 （°C）
    qint8 battery_2_temperature ;

    //! \brief          : 1     : 0~100         //电池组2当前容量   额定容量的百分比表示以50Ah为基准百分比 （%）
    qint8 battery_2_capacity ;

    //! \brief          接触器状态字
    contactor_status contactor;

    //! \brief          : 1     : -128~127      //大气总温  （°C）
    qint8 atmos_whole_temper ;

    //! \brief          : 1     : -128~127      //大气静温  （°C）
    qint8 atmos_static_tempe ;

    //! \brief          备用
    qint8 backup[3];

    //! \brief          : 1     : 0~255         //效验和
    quint8 check ;
}s5D;
//}

//s5E/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//namespace s5E {

//! \brief          电池工作状态字
typedef struct Structbattery_work_status
{

    //! \brief      电池组1充电状态        0=未充电 | 1=充电
    qint8 charging_status_1 : 1;

    //! \brief      电池组1充满状态        0=未充满 | 1=充满
    qint8 full_status_1 : 1;

    //! \brief      电池组1加热状态        0=未加热 | 1=加热
    qint8 heating_status_1 : 1;

    //! \brief      电池组2充电状态        0=未充电 | 1=充电
    qint8 charging_status_2 : 1;

    //! \brief      电池组2充满状态        0=未充满 | 1=充满
    qint8 full_status_2 : 1;

    //! \brief      电池组2加热状态        0=未加热 | 1=加热
    qint8 heating_status_2 : 1;

    //! \brief      充放电控制过热保护       0=未过热保护 | 1=过热保护
    qint8 overheat_protection : 1;

    //! \brief      加热允许                0=不允许 | 1=允许
    qint8 heating_permit : 1;

}battery_work_status;

//! \brief          电源系统PEL故障字1
typedef struct Structpel_power_system_1
{
	quint8 pel_power_system_1 ;
}pel_power_system_1;

//! \brief          电源系统PEL故障字2
typedef struct Structpel_power_system_2
{
    //! \brief      电池1故障
	quint8 battery_1_fault : 1;

    //! \brief      电池2故障
	quint8 battery_2_fault : 1;

    //! \brief      充电器故障
	quint8 charger_fault : 1;

    //! \brief      GCU故障
	quint8 gcu_fault : 1;

    //! \brief      排期风扇“前”故障
	quint8 front_fault : 1;

    //! \brief      排期风扇“后”故障
	quint8 after_fault :1;

    //! \brief      供油状态
	quint8 supply_status : 1;

    //! \brief      断油状态
	quint8 breaking_status : 1;
}pel_power_system_2;

//! \brief          电源系统PEL故障字3
typedef struct Structpel_power_system_3
{
    //! \brief
}pel_power_system_3;

//! \brief          SSPC通道状态字
typedef struct Structsspc_status
{

    //! \brief      1号通道    0=1号通道断开 | 1=1号通道接通
	quint8 channel_1_1 : 1;
    //! \brief      1号通道    0=1号通道正常 | 1=1号通道故障
	quint8 channel_1_2 : 1;

    //! \brief      2号通道    0=2号通道断开 | 1=2号通道接通
	quint8 channel_2_1 : 1;
    //! \brief      1号通道    0=2号通道正常 | 1=2号通道故障
	quint8 channel_2_2 : 1;

    //! \brief      3号通道    0=3号通道断开 | 1=3号通道接通
	quint8 channel_3_1 : 1;
    //! \brief      1号通道    0=3号通道正常 | 1=3号通道故障
	quint8 channel_3_2 : 1;

    //! \brief      4号通道    0=4号通道断开 | 1=4号通道接通
	quint8 channel_4_1 : 1;
    //! \brief      1号通道    0=4号通道正常 | 1=4号通道故障
	quint8 channel_4_2 : 1;

}sspc_status;



//! \5E 帧类别5E （电气综合管理系统信息）  频率4Hz
typedef struct Structs5E
{
    //! \brief          : 1     : 0x55AA        //同步码
    quint16 syn_code;

    //! \brief          : 1     : 0x5E          //帧类型
    quint8 frame_type;

    //! \brief          电池工作状态字
    battery_work_status battery_work;

    //! \brief          电源系统PEL故障字1
    pel_power_system_1 power_system_1;

    //! \brief          电源系统PEL故障字2
    pel_power_system_2 power_system_2;

    //! \brief          电源系统PEL故障字3     备份
    qint8 backup_1 ;

    //! \brief          电源系统PEL故障字4     备份
    qint8 backup_2 ;

    //! \brief          SSPC通道状态字（01-04)
    sspc_status sspc_01_04;

    //! \brief          SSPC通道状态字（05-08)
    sspc_status sspc_05_08;

    //! \brief          SSPC通道状态字（09-12)
    sspc_status sspc_09_12;

    //! \brief          SSPC通道状态字（13-16)
    sspc_status sspc_13_16;

    //! \brief          SSPC通道状态字（17-20)
    sspc_status sspc_17_20;

    //! \brief          SSPC通道状态字（21-24)
    sspc_status sspc_21_24;

    //! \brief          SSPC通道状态字（25-28)
    sspc_status sspc_25_28;

    //! \brief          SSPC通道状态字（29-32)
    sspc_status sspc_29_32;

    //! \brief          SSPC通道状态字（33-36)
    sspc_status sspc_33_36;

    //! \brief          SSPC通道状态字（37-40)
    sspc_status sspc_37_40;

    //! \brief          SSPC通道状态字（41-44)
    sspc_status sspc_41_44;

    //! \brief          SSPC通道状态字（45-48)
    sspc_status sspc_45_48;

    //! \brief          SSPC通道状态字（49-52)
    sspc_status sspc_49_52;

    //! \brief          SSPC通道状态字（53-56)
    sspc_status sspc_53_56;

    //! \brief          SSPC通道状态字（57-60)
    sspc_status sspc_57_60;

    //! \brief          SSPC通道状态字（61-64)
    sspc_status sspc_61_64;

    //! \brief          SSPC通道状态字（65-68)
    sspc_status sspc_65_68;

    //! \brief          SSPC通道状态字（69-70)
    sspc_status sspc_69_70;

    //! \brief          备份
    qint8 backup[5];

    //! \brief          : 1     : 0~255         //效验和
    quint8 check ;
}s5E;
//}








//! \brief      无人机编队飞行遥测指令帧格式
//! \            由4个子帧构成，每个子帧长度为40字节，共160个字节，每个子帧均有同步码和效验码
//! \           其中遥测子帧1、2、3、4传输速率为20Hz
//! ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//! |       | 子帧 1         |   子帧 2    |      子帧 3     |             子帧 4              |  |
//! ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//! | 同步码 |  AAH  55H    |   AAH  55H   |    AAH  55H    |              AAH  55H           |   |
//! ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//! | 帧类别 |  81H         |  82H         |      83H       |             84H                |
//! ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//!
//!
//! 效验规则 (和校验)   帧类别+帧内容+效验码=0


typedef struct
{
	qint8 fly_stage:4; //飞行阶段 1110编队
	qint8 fly_comb:4; //
}FlyState;
typedef struct
{
    quint8 syn_code[2]; // AA 55
    quint8 frame_type;  // 0x81
    quint8 frame_count; // 帧计数
    quint8 grounpNum;   // 目标群号
    quint16 tgtUavName;     // 目标飞机名称   目标飞机名称与目标群号组合使用（目标飞机名称是65535时， 目标群号是255代表目标飞机是所有飞机，是1-24则是指定子群内的所有飞机）
    quint16 selfName;   // 本机名称 （本机飞机地址）
    quint16 showSpeed; // 指示空速 分辨率0.1 0-75m/s
    quint16 norSpeed; // 北向空速 分辨率0.1 0-75m/s
    quint16 eastSpeed; // 东向空速 分辨率0.1 0-75m/s
    quint16 upSpeed; // 天向速度 分辨率0.1 0-75m/s
    quint16 orderHead; // 指令航向 分辨率0.1 0-360度
    quint16 track; // 航迹偏航角 分辨率0.1 0-360度
    quint16 sideRate; // 侧偏变化率 分辨率0.1 -300-300m/s
    quint8 formState; // 无人机编队状态 分辨率1 0=不处于编队状态；1=处于编队状态
    quint16 radHeight; // 无高  分辨率0.1 0-1000 m
    quint16 airHeight; // 气高 分辨率0.1 0-6000 m
    qint16 sateHeight; // 卫星高度 分辨率0.2 -500-6000 m
    qint8 pad; // 备用
    qint8 yinDaoMode; // 引导模式 0=航线引导；1=编队控制；2=目标跟踪
    FlyState flyState; // 飞行阶段 0-15
    qint8 flySubState; // 飞行子阶段 0-15
    qint8 cFgtLineNum; // 当前航线号 0-60
    qint8 cFgtPointNum; // 当前航点号  0-255
    qint8 formStage; // 编队阶段 0=空；1=编队集结；2=编队保持；3=队形变换；4=编队解散；5=退出编队；10=单机飞行
    qint8 formSubStage; // 编队子阶段
    quint8 grounpid; // 当前队形号 0-15
    qint8 check_sum;   // 和校验
}FormSubFrame1;

typedef struct
{
    unsigned char syn_code[2]; // AA 55
    unsigned char frame_type;  // 0x82
    unsigned char data[36];
    unsigned char check_sum;   // 和校验
}FormSubFrame2;

typedef struct
{
    unsigned char syn_code[2]; // AA 55
    unsigned char frame_type;  // 0x83
    unsigned char data[36];
    unsigned char check_sum;   // 和校验
}FormSubFrame3;


typedef struct
{
	quint8 uav_uav:1;		//无人机防撞启动标志
	quint8 uav_manned:1;	//有人机防撞启动标志
	quint8 conditon3:1;
	quint8 conditon4:1;
	quint8 conditon5:1;
	quint8 conditon6:1;
	quint8 conditon7:1;
	quint8 conditon8:1;
}AvoidInfo;

typedef struct
{
	quint8 conditon1:1;	//有人机水平速度(>28m/s) 		0-不满足 1-满足
	quint8 conditon2:1;	//与有人机水平距离(<20km)		0-不满足 1-满足
	quint8 conditon3:1;	//有人机绝对高度大于安全高度	0-不满足 1-满足
	quint8 conditon4:1;	//有人机航迹角差(<=90)		0-不满足 1-满足
	quint8 conditon5:1;	//水平距离参数[5000,10000]	0-不满足 1-满足
	quint8 conditon6:1;	//高度差参数[300,500]			0-不满足 1-满足
	quint8 conditon7:1;	//有人机U链					0-不满足 1-满足
	quint8 conditon8:1;	//编队保持					0-不满足 1-满足
}MannedLead;

typedef struct
{
	quint8 conditon1:1;	//有人机水平速度(<20m/s)且距离(<8km) 		0-不满足 1-满足
	quint8 conditon2:1;	//与有人机水平距离(>25km)					0-不满足 1-满足
	quint8 conditon3:1;	//有人机绝对高度小于安全高度				0-不满足 1-满足
	quint8 conditon4:1;	//有人机航迹角差(>135)					0-不满足 1-满足
	quint8 conditon5:1;	//有人机U链失效							0-不满足 1-满足
	quint8 conditon6:1;	//低高保安								0-不满足 1-满足
	quint8 spare:2;
}MannedExit;

//typedef struct
//{
//	quint8 conditon1:1;	//本机机间遥测完全失效 		0-有效 1-失效
//	quint8 conditon2:1;	//本机编队能力				0-无效 1-有效
//	quint8 conditon3:1;	//本机判无人机1编队能力		0-无效 1-有效
//	quint8 conditon4:1;	//本机判无人机2编队能力		0-无效 1-有效
//	quint8 conditon5:1;	//本机判无人机3编队能力		0-无效 1-有效
//	quint8 conditon6:1;	//本机判无人机4编队能力		0-无效 1-有效
//	quint8 conditon7:1;	//退出集结轨迹标志			0-直线 1-圆弧
//	quint8 spare:1;
//}Redundancy3;

//应急区域
typedef struct
{
	float Lon;		//遥测4B帧 3~6字节 分辨率 180/(2^31-1),-180-180
	float Lat;		//遥测4B帧 7~10字节 分辨率 90/(2^31-1),-90-90
	float Angle;	//遥测4B帧 11~12字节 分辨率 180/(2^15-1),-180-180
	quint16 Long;	//遥测4B帧 13~14字节 单位：m
	quint16 Wide;	//遥测4B帧 15~16字节 单位：m
}EMERGENCE_AREA;

typedef struct
{
    unsigned char syn_code[2]; // AA 55
    unsigned char frame_type;  // 0x84
    unsigned char data[36];  // 遥调/数据库参数回报 （优先响应遥调参数回报）
    unsigned char check_sum;   // 和校验
}FormSubFrame4;

typedef struct
{
    FormSubFrame1 formSubFrame1;
    FormSubFrame2 formSubFrame2;
    FormSubFrame3 formSubFrame3;
    FormSubFrame4 formSubFrame4;
}FormTelemetry;//编队遥测


typedef struct
{
    unsigned char front[160];
    FormTelemetry tail; // 后160

}BLK_KKL_CCC_000_008_010_011;

/******************************************  接收结构体 ***************************************/
// 1. u链状态信息 003
typedef struct uav_CLCoSTATUS{  // 无人机U链连接状态信息
    unsigned int uav_id; // 无人机id
    unsigned char IFNoHeliCon_1; // 是否锁定  1锁定 2挂锁
    unsigned char ConSignalStren_1; // 信号强度
    // char spare[10]; // 补位信息
}CLCoSTATUS;

// 无人地面站U链连接状态信息
typedef struct station_CLCoSTATUS{
    unsigned int uav_id; // 无人机id
    unsigned char IFNoHeliCon_1_1; // 是否锁定  0 无 1 锁定 2 挂锁
    unsigned char ConSignalStren_1_1; // 信号强度
    // char spare[10]; // 补位信息

}station_CLCoSTATUS;


//  主结构体
typedef struct U_link_status{

    unsigned short KKLCNumber_2; // u链连接无人机数量
    CLCoSTATUS uav_CLCoSTATUS_info[4];  // 无人机U链连接状态信息
    unsigned short KKLCNumber_2_1; // u链连接地面站数量
    station_CLCoSTATUS station_CLCoSTATUS_info[4];  // 无人地面站U链连接状态信息

} U_link_status;



// 2. 空空链参数反馈信息 004
typedef struct uav_clparset{ // 无人机c链参数设置
    unsigned int uav_id; // 无人机id
    unsigned char DownWorkMode_1; // 下行工作模式
    unsigned char DownPower_1; // 下行功率选择
    unsigned char UpRate_1; // 上行传输速率
    unsigned char DownRate_1; // 下行传输速率
    unsigned char UpWorkMode_1; // 上行工作模式
    unsigned char UpPower_1; // 上行功率选择
    unsigned char PassWay_1; // 加密方式
    unsigned char Cwire_1; // c相控阵天线选择
    unsigned char Upchannel; // 上行频道选择
    unsigned char Downchannel; // 下行频道选择
    // char spare[22]; // 补位信息
}uav_clparset;

//  主结构体
typedef struct kkl_canshu_feedback{

    unsigned short KKLCNumber_1_1;  // c链连接无人机数量
    uav_clparset uav_clparset_info[4]; // 无人机c链参数设置
}kkl_canshu_feedback;




// 3. kkl链路状态数据  005
typedef struct uav_cl_status{
    unsigned int uav_id; // 无人机id
    unsigned char IFNoHeliCon_1; // 是否锁定  1锁定 2挂锁
    unsigned char ConSignalStren_1; // 信号强度
}uav_cl_status;
// 主结构体
typedef struct kkl_status{

    unsigned short KKLCNumber;  // c链连接无人机数量
    uav_cl_status uav_cl_status_info[4]; // 无人机c链连接状态信息
}kkl_status;



// 4. 链路交接控制指令反馈
typedef struct uav_cl_jiaojie{ // 无人机C链交接指令

    unsigned char ControlChange; // 启动主链转
    unsigned int RecGroundID; // 接机站id
    unsigned char ChangeConfirm; // c交接成功确认

}uav_cl_jiaojie;
// 主结构体  009
typedef struct link_control_feedback{
    unsigned short KKLCNumber_1_1_1_1;  // c链连接无人机数量
    uav_cl_jiaojie uav_cl_jiaojie_info[4]; // 无人机C链交接指令

}link_control_feedback;


// 5. 飞行故障清单
// 主结构体 190
typedef struct fly_error_order{
    unsigned int warn_level; // 飞行故障警告级
    unsigned int attention_level; // 飞行故障注意级
    unsigned int tips_level; // 飞行故障提示级
} fly_error_order;


// KKL-CCC/DPU1/DPU2-006 无人机链路状态数据（MessageID：0xaa2c06） 不确定
typedef struct cr_uav_cl_status{

    unsigned int NoHeliID;    // 无人机ID
    unsigned short UAV_StationID;   //控制站ID
    unsigned char UpChannel;    // 上行频道选择
    unsigned char DownChannel;  // 下行频道选择

    unsigned char WorkMode;   // 工作模式


    unsigned char Power;      // 功率选择

    unsigned char UpRate;       // 上行传输数据
    unsigned char DownRate;     // 下行传输数据

    unsigned char PassWay;      // 加密方式
    unsigned char IfNoHeliCon;  // 上行是否锁定

    unsigned char ConSignalStren;       // 无人机接收信号强度
    unsigned char ChangeState;          // 交接状态

    unsigned char HandOverUpChannel;    // 交接上行频道选择
    unsigned char HandOverDownChannel;  // 交接下行频道选择
    unsigned short idWillGetUav;	/*接机站ID   		20250530 new*/
	unsigned char attena_mode;		/*天线工作模式 0-全向  1-定向   	20250530 new*/
	unsigned char com_model;		//通信模式 0-无效 1-地空 4-空空 20251014new
}CR_uav_cl_status;

typedef struct cr_kkl_status{

    unsigned char KKLCNumber;   // c链连接无人机数量
    CR_uav_cl_status CR_uav_cl_status_info[4]; // 无人机c链连接状态信息
}BLK_KKL_CCC_006;



// KKL-CCC/DPU1/DPU2-007 本机链路状态数据（MessageID：0xaa2c07） 不确定
typedef struct uav_cl_status1{

    unsigned short NoHeliID;    // 无人机ID
    unsigned char UpChannel;    // 上行频道选择
    unsigned char DownChannel;  // 下行频道选择

    unsigned char WorkMode;   // 工作模式//工作模式


    unsigned char Power;      // 功率选择

    unsigned char UpRate;       // 上行传输数据
    unsigned char DownRate;     // 下行传输数据

    unsigned char PassWay;      // 加密方式
    unsigned char IfNoHeliCon;  // 下行是否锁定

    unsigned char ConSignalStren;       // 有人机接收信号强度
    unsigned char ChangeState;          // 交接状态
    unsigned char com_model;		//通信模式 0-无效 1-地空 4-空空 20251014new
}uav_cl_status1;

typedef struct kkl_status1{

    char KKLCNumber;   // c链连接无人机数量
    uav_cl_status1 uav_cl_status_info[4]; // 空空链路状态（C链）状态
}BLK_KKL_CCC_007;

typedef struct
{
	unsigned char spare :8;
	unsigned char manned_aircraft_online :1; 		//有人机在网状态
	unsigned char unmanned1_aircraft_online :1; 	//无人机1在网状态
	unsigned char unmanned2_aircraft_online :1; 	//无人机2在网状态
	unsigned char jianmianzhan1_online :1;			//舰面站1状态
	unsigned char monijianmianzhongduan_online :1;	//模拟舰面站状态
	unsigned char unmanned3_aircraft_online :1; 	//无人机3在网状态
	unsigned char unmanned4_aircraft_online :1; 	//无人机4在网状态
	unsigned char unmanned5_aircraft_online :1; 	//无人机5在网状态 暂未使用
}BLK_KKL_CCC_197;//U链在网成员 20250725new ，与54链路协定icd 0xa22210



#pragma pack()

#endif // ICD_KKL_CCC_H
