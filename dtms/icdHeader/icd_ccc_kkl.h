#ifndef ICD_CCC_KKL_H
#define ICD_CCC_KKL_H

#pragma pack(1)


//! 基本遥控信息数据帧 由 基本遥控指令帧 （64字节） + 航线与威胁管理帧（32字节） + 任务控制指令帧（64字节） = 160 字节
//! —————————————————————————————————————————————————————————————————————————————————
//! |         | 基本遥控指令帧  |      航线与威胁管理帧      |      任务控制指令帧    |
//! ————————————————————————————————————————————————————————————————————————————————
//! | 同步码 |  EBH   |  90H  |    55H      |     AAH     |                        |
//! ————————————————————————————————————————————————————————————————————————————————

// 说明 当前测试进度较急 故只完善航线帧 其余俩个帧 未完善只完善所占字节数 保证航线帧位置正确 及整个遥控帧完整

typedef struct empty_order{ // 空指令 0x00
    unsigned char order[3];
}empty_order;

typedef struct ddxtwzzr_order{ // 定点悬停位置注入指令 0x30
    unsigned int lon;  // 经度
    unsigned int lat;  // 纬度
    unsigned short xd_height; // 相对高度
}ddxtwzzr_order;

typedef struct fhjczr_order{ // 返航机场点注入指令 0x32
    unsigned int target_lon;  // 目标点经度
    unsigned int target_lat;  // 目标点纬度
    unsigned short targrt_height; // 目标点海拔高度
}fhjczr_order;

typedef struct yjfh_order{ // 应急返航指令 0x34
    unsigned int target_lon;  // 目标点经度
    unsigned int target_lat;  // 目标点纬度
    unsigned short targrt_height; // 目标点海拔高度
    unsigned char yjfhyh_target; // 应急返航优化目标
    unsigned char tezhenzi; // 特征字
}yjfh_order;

typedef struct fly_to_xq_order{ // 向兴趣点飞指令  0x4a
    unsigned int lon;  // 经度
    unsigned int lat;  // 纬度
    unsigned short hb_height; // 海拔高度
    unsigned short speed; // 速度
    unsigned char tezhenzi[8]; // 特征字
}fly_to_xq_order;

typedef struct track_point_insert_order{ // 航点插入指令 0x36
    unsigned char track_line_id;  // 航线号
    unsigned char track_point_id; // 航点号
    unsigned int lon;  // 经度
    unsigned int lat;  // 纬度
    unsigned short hb_height; // 海拔高度
    unsigned short speed; // 速度
    unsigned char next_track_line_id;  // 航线号
    unsigned char next_track_point_id; // 航点号
    unsigned char tezhenzi[8]; // 特征字
}track_point_insert_order;

typedef struct track_point_jd_order{ // 航点（绝对）装订指令 0x38
    unsigned char track_line_id;  // 航线号
    unsigned char track_point_id; // 航点号
    int lon;  // 经度
    int lat;  // 纬度
    unsigned short hb_height; // 海拔高度
    unsigned short speed; // 速度
    unsigned char next_track_line_id;  // 下一航线号
    unsigned char next_track_point_id; // 下一航点号
    unsigned char tezhenzi[8]; // 特征字
}track_point_jd_order;

typedef struct track_point_xd_order{ // 航点装订(相对）指令 0x39
    unsigned char track_line_id;  // 航线号
    unsigned char track_point_id; // 航点号
    unsigned int lon;  // 航点横坐标
    unsigned int lat;  // 航点纵坐标
    unsigned short hb_height; // 航点相对高度
    unsigned short speed; // 速度
    unsigned char beiyong1;  // 备用
    unsigned char beiyong2; // 备用
    unsigned char tezhenzi[8]; // 特征字
}track_point_xd_order;

typedef struct track_point_modfiy_order{ // 航点修改指令 0x3a
    unsigned char track_line_id;  // 航线号
    unsigned char track_point_id; // 航点号
    unsigned int lon;  // 航点横坐标
    unsigned int lat;  // 航点纵坐标
    unsigned short hb_height; // 航点相对高度
    unsigned short speed; // 速度
    unsigned char beiyong1;  // 备用
    unsigned char beiyong2; // 备用
    unsigned char tezhenzi[8]; // 特征字
}track_point_modfiy_order;

typedef struct jd_track_point_search_order{ // 绝对航线、航点查询指令 0x3c
    unsigned char track_line_id;  // 航线号
    unsigned char track_point_id; // 航点号
}jd_track_point_search_order;

typedef struct xd_track_point_search_order{ // 相对航线、航点查询指令 0x3d
    unsigned char track_line_id;  // 航线号
    unsigned char track_point_id; // 航点号
}xd_track_point_search_order;

typedef struct track_point_delete_order{ // 航线、航点删除指令 0x3e
    unsigned char track_line_id;  // 航线号
    unsigned char track_point_id; // 航点号
}track_point_delete_order;

typedef struct track_point_chage_order{ // 航线、航点切换指令 0x40
    unsigned char track_line_id;  // 航线号
    unsigned char track_point_id; // 航点号
}track_point_chage_order;

typedef struct track_length_search_order{ // 航线长度查询指令 0x41
    unsigned char all_tracks;  // 航线号
}track_length_search_order;

typedef struct threat_data_into{ // 威胁数据注入 0x42
    unsigned char area_code;//区域号
    unsigned char point_number;//点数量
    unsigned char topPoint_code;//顶点号
    unsigned int lon;  // 经度
    unsigned int lat;  // 纬度
    unsigned short hb_top_height; // 顶高
    unsigned short hb_low_height; // 底高
    unsigned short radius; // 半径
    unsigned char tezhenzi; // 特征字
}threat_data_into;

typedef struct threat_area_search{ // 威胁区查询指令 0x44
    unsigned char area_code;  // 区域号
}threat_area_search;

typedef struct threat_area_delete{ // 威胁区删除指令 0x46
    unsigned char area_delete;  // 区域号
}threat_area_delete;


// 指令内容帧
typedef struct order_data_frame{
    empty_order empty_orders; // 空指令
    ddxtwzzr_order ddxtwzzr_orders; // 定点悬停位置注入指令
    fhjczr_order fhjczr_orders; // 返航机场点注入指令
    yjfh_order yjfh_orders; // 应急返航指令
    fly_to_xq_order fly_to_xq_order; // 向兴趣点飞指令
    track_point_insert_order track_point_insert_orders; // 航点插入指令
    track_point_jd_order track_point_jd_order; // 航点（绝对）装订指令
    track_point_xd_order track_point_xd_orders; // 航点装订(相对）指令
    track_point_modfiy_order track_point_modfiy_orders; // 航点修改指令
    jd_track_point_search_order jd_track_point_search_orders; // 绝对航线、航点查询指令
    xd_track_point_search_order xd_track_point_search_orders; // 相对航线、航点查询指令
    track_point_delete_order track_point_delete_orders; // 航线、航点删除指令
    track_point_chage_order track_point_chage_orders;   // 航线、航点切换指令
    track_length_search_order track_length_search_orders; // 航线长度查询指令
    threat_data_into threat_data_intos;     // 威胁数据注入
    threat_area_search threat_area_searchs;  // 威胁区查询指令
    threat_area_delete threat_area_deletes; // 威胁区删除指令
}order_data_frame;

// 飞行航线威胁指令帧
typedef struct track_threat_frame{
    unsigned short tongbu_code;  // 同步码 55H AAH
    unsigned char frame_count; // 帧计数
    unsigned char order_code[3];  // 三判二
    unsigned char order_data[24];  // 指令数据
    unsigned short check_code; // 检验和
}track_threat_frame;

// 基本遥控指令帧
typedef struct basic_yaotiao_order_frame{

    unsigned char data[64];

}basic_yaotiao_order_frame;


//任务控制指令帧
typedef struct task_control_frame{
    unsigned char data[64];
}task_control_frame;

//飞行遥控指令帧
typedef struct
{
    unsigned char 	sync_code[2];  	// 同步码 EBH 90H
    unsigned char  	frame_type;		//0xFC
    unsigned char	frame_count;	//0~255循环
    unsigned short 	plane_adr[3];	//飞机地址
    unsigned short 	station_adr[3];	//控制站地址
    unsigned char 	fc_order[3];	//飞行遥控指令
    unsigned char 	yt_start;		//飞行遥调指令
    unsigned char 	order_code[3];	//遥控指令编码
    unsigned short 	order_data[13];	//遥控指令数据
    unsigned char 	yt_check;		//遥调指令校验
    unsigned char 	lx_start;		//飞行连续指令
    unsigned short 	lon_code;		//纵向控制指令
    unsigned short 	lat_code;		//横向控制指令
    unsigned short 	total_code;		//总距控制指令
    unsigned short 	course_code;	//航向控制指令
    unsigned char 	indep_acce;		//独立油门微调
    unsigned char 	total_acce;		//总距油门微调
    unsigned char 	lx_check;		//连续指令校验
    unsigned short check_sum;		//校验和
}FC_ORDER;

// 基本遥控信息数据帧
typedef struct{

    FC_ORDER basic_yaotiao_order_frame_data;  //  基本遥控指令帧 64字节
    track_threat_frame track_threat_frame_data;   //  飞行航线威胁指令帧 32字节
    task_control_frame task_control_frame_data;   //  任务控制指令帧 64字节

}BLK_CCC_KKL_008_026_027_028_Front;




//! 无人机飞控编队飞行遥控指令帧 由 基本遥控指令帧 （100字节） + 数据库管理指令帧（43字节） + 备用字节（17字节） = 160 字节
//! —————————————————————————————————————————————————————————————————————————————————
//! |         | 基本遥控指令帧  |       数据库管理指令帧      |      备用字节    |
//! ————————————————————————————————————————————————————————————————————————————————
//! | 同步码 |  AAH   |  55H   |    7EH      |     7EH     |                        |
//! ————————————————————————————————————————————————————————————————————————————————

typedef struct
{
    unsigned char 	sync_code[2];  	// 同步码 AAH 55H
    unsigned char  	frame_type;		//0xFC
    unsigned char	frame_count;	//0~255循环
    unsigned char 	plane_adress;	//机地址
    unsigned char 	station_adress;	//站地址
    unsigned char 	groupNum;	//群组号 三判二
    unsigned char 	fc_order[3];	//飞行遥控指令 三判二
    unsigned char 	yt_start;		//飞行遥调指令起始  起始标识是"Z"
    unsigned char 	yt_order_code[3];	//遥调指令编码 三判二
    unsigned char 	yt_order_data[83];	//遥调指令数据
    unsigned char 	yt_check_sum;   //遥调指令校验(和校验)
    unsigned short  check_crc;		//校验(crc)
}BaseControl;

typedef struct
{
    unsigned char 	sync_code[2];  	// 同步码  7EH 7EH
    unsigned char	frame_count;	//0~255循环
    unsigned char 	order[3];	//数据装订指令码 三判二
    unsigned char 	order_data[35];	//数据装订指令内容
    unsigned short  check_crc;		//校验(crc)
}DataBase;

typedef struct
{
    unsigned char spare[17];
}SpareBytes;

typedef struct
{
    BaseControl baseControl;
    DataBase dataBase;
    SpareBytes spareBytes;
}BLK_CCC_KKL_008_026_027_028_Tail;

//内部接口数据包
/***********************************************************************************************/
typedef struct
{
    unsigned char hx; //航线号
    unsigned char hd; //航点号
    int lon; //经度
    int lat; //纬度
    unsigned short height; //高度
    unsigned short speed; //速度
    unsigned char next_hx; //下一航线号
    unsigned char next_hd; //下一航点号
    unsigned char tezhenzi[8]; //特征字
    unsigned char UTC_time[8]; //UTC时间
    unsigned char group_id; //群组号
    unsigned char team_id; //队形号
    unsigned char task_type; //任务类型
}tail_0x30;

typedef struct
{
    unsigned short hori_distance; //与有人机的水平间距
    unsigned short high_cmd; //有人机领航时的高度指令
}tail_0x16;
/***********************************************************************************************/

//基本遥控信息数据帧（黑包,160字节）+编队遥控信息数据帧（160字节,编队遥控信息数据帧在加密或25.6K时没有）
typedef struct
{
    BLK_CCC_KKL_008_026_027_028_Front front;
    BLK_CCC_KKL_008_026_027_028_Tail tail;//编队遥控信息数据帧，暂时为黑包
}BLK_CCC_KKL_008_026_027_028;





/*********** CCC-KKL/MMM-000 空空链参数控制指令 ***********/
typedef struct
{
    unsigned char TongBuZi1;//同步字1 域描述：0xEB
    unsigned char TongBuZi2;//同步字2 域描述：0x90
    unsigned char ZhenChang;//帧长 域描述：0x10
    unsigned char Type;//类型 域描述：04H无人机机载链路控制，21H地面链路控制/有人机空空链路控制
    unsigned char Num;//序号/帧计数 域描述：0x00-0x7f循环
    unsigned short BenJiID;//本机ID
    unsigned short UavID;//被控设备ID
    unsigned char KaiGuanLiang;//开关量指令码
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
    unsigned char LianXuLiang;//连续量指令码
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
    unsigned char LianXuLiangL;//连续量数据L
    unsigned char LianXuLiangH;//连续量数据H
    unsigned short CRC16;//CRC16 域描述：校验4-13共10个字节
    unsigned char HeJiaoYan;//和校验 域描述：从EB90开始
} BLK_CCC_KKL_000;



/******************************************  发送结构体 ***************************************/
typedef struct C_control_order{ // c链交接指令
    unsigned char ControlChange_1; // 启动主链转
    unsigned int RecGroundID_1; // 接机站id
    unsigned char ChangeConfirm_1; // c交接成功确认
    char spare[30]; // 补位信息
}C_control_order;

// 链路交接控制指令  011
typedef struct send_link_control{
    unsigned short KKLCNumber_1_1;  // c链连接无人机数量
    C_control_order C_control_order_info[4]; // c链交接指令
}ksend_link_controlS;



typedef struct uav_C_parset{ // 无人机c链参数设置
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
    unsigned char ControlChange; // 启动主链转移
    unsigned int RecGroundID; // 接机站id
    unsigned char ChangeConfirm; // c交接成功确认
    char spare[16]; // 补位信息
}uav_C_parset;

// 空空链参数设置  042
typedef struct ckpartset{

    unsigned short KKLCNumber;  // c链连接无人机数量
    uav_C_parset uav_clparset_info[4]; // 无人机c链参数设置

}ckpartset;

// 无人机遥控帧
// 见 "yaokong_frame.h"

// 导航数据 025
typedef struct daohang_data{
    char data[2096];
}daohang_dataS;

// CCC-KKL/MMM-009 无人机1链路遥控指令帧(MessageID:0xa20e09)
typedef struct {
    unsigned char TongBuZi1;//同步字1 域描述：0xEB
    unsigned char TongBuZi2;//同步字2 域描述：0x90
    unsigned char ZhenChang;//帧长 域描述：0x10
    unsigned char Type;//类型 域描述：04H无人机机载链路控制，21H地面链路控制/有人机空空链路控制
    unsigned char Num;//序号/帧计数 域描述：0x00-0x7f循环
    unsigned short BenJiID;//本机ID
    unsigned short UavID;//被控设备ID
    unsigned char KaiGuanLiang;//开关量指令码
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
    unsigned char LianXuLiang;//连续量指令码
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
    unsigned char LianXuLiangL;//连续量数据L
    unsigned char LianXuLiangH;//连续量数据H
    unsigned short CRC16;//CRC16 域描述：校验4-13共10个字节
//    char spare;//备份
    unsigned char HeJiaoYan;//和校验 域描述：从EB90开始
} BLK_CCC_KKL_009_029_030_031;//无人机1链路遥控指令帧
#pragma pack()

#endif // ICD_CCC_KKL_H
