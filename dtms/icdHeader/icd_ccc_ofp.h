#ifndef COOPERATIVEALLEGATIONCOMPUTERSYNTHESIS_H
#define COOPERATIVEALLEGATIONCOMPUTERSYNTHESIS_H

/*
 * 协同指控计算机-综显
 * */

#pragma pack(1)
//经纬度
typedef struct longitude_and_latitude
{
    double longitude;//经度
    double latitude;//维度

}longitude_and_latitude;

typedef struct point_coordinate//点坐标
{
    double longitude;//经度
    double latitude;//维度
}point_coordinate;

// vector
typedef struct StructVector{
    int data[1000];
    int size;
}vector_fake;



// 控制权交接申请
typedef struct jmz_info
{
    unsigned short C_Down_channel_1_1_1;
    unsigned short C_UP_Channel_1_1_1;
    unsigned short C_UP_Power_1_1;
    unsigned short C_Down_Power_1_1_1;
    unsigned short C_Up_speed_1_1;			/*U链上行速率*/
    unsigned short CDownRate_1_1;			/*C链下行速率*/
    unsigned short CSilence_1_1;
    unsigned short CCpyto_1_1;
    unsigned short U_channe_l_1_1_1;
    unsigned short UPower_1_1;
    unsigned short USpeed_1_1;
    unsigned short USilence_1_1;
    unsigned short UCpyto_1_1;
}jmz_info;/*舰面站交接参数*/

typedef struct kongzhiquan_jiaojie
{
    unsigned int UAV_ID_1;			/*无人机ID*/
    unsigned int JMZ_ID;			/*舰面站ID*/
    jmz_info  signal_fc00;		/*舰面站交接参数*/
}kongzhiquan_jiaojie;/*控制权交接申请*/


// CCC-DPU1/DPU2/MMM-003 控制权交接状态反馈（MessageID：0xa22203）
typedef struct {
	unsigned short uav_seri_num;    //无人机序号
	unsigned int   uav_ID;          //无人机ID
    unsigned char ctrl_releasing;	/*控制权正在释放中  0-NA 1-控制权正在释放  2-控制权已经释放 3-拒绝释放控制权*/
    unsigned char ctrl_recieving;	/*控制权正在接收中  0-NA 1-控制权正在接收  2-控制权已经接收 3-控制权拒绝接收*/
    unsigned short status;
    /*控制权交接状态（Enum）（0H=NA;1H=释放控制权-交接准备失败-速率切换失败；2H=释放控制权-交接准备失败-天线模式切换失败；
    3H=释放控制权-交接准备失败-链路状态不稳定；4H=释放控制权-交接准备失败-配置参数设置失败；5H=释放控制权-控制权交接失败-无人机退回原来参数；
    6H=释放控制权-控制权交接成功；7H=申请控制权-正在申请控制权；8H=申请控制权失败-链路参数切换不成功；9H=控制权申请-地面站拒绝；
    AH=控制权申请-地面站同意-正在交接；BH=控制权申请-失败-链路未建立；CH=控制权申请-成功；DH=手动释放-开始控制权交接；
    EH=手动释放-控制权交接成功；FH=手动释放-控制权交接失败；10H=手动释放-正在释放控制权；11H=手动获取控制权-主链建立；
    12H=主动获取-正在接收控制权；13H=主动获取-控制权交接成功；14H=主动获取-控制权交接失败）*/
} CR_ControlHandoverStatusFeedback;//控制权交接状态反馈


//3.1  任务状态信息

//typedef struct drone_information//无人机信息
//{
//    unsigned short drone_serial_number;//无人机序号
//    unsigned int drone_num ;//无人机编号
//    unsigned int current_task_ID;//当前任务ID
//    unsigned short current_task_state;//当前任务状态
//    unsigned short current_task_progress;//当前任务进度
//}drone_information;

//CCC-DPU1/DPU2/MMM-012 MessageID 0xa2220c
typedef struct task_status_information
{
    unsigned int current_task_ID_of_the_panel;//有人机当前任务ID
    unsigned short current_state;//当前状态
    unsigned short current_task_progress;//当前任务进度
    // unsigned short drone_number = 1;//无人机个数n
    // drone_information drone_informations[4];//无人机信息
}task_status_information;



//3.2 无人机状态信息
typedef struct
{
    unsigned long long xyzsg : 1;		//旋翼转速高(0-无故障;1-故障)
    unsigned long long xyzsd : 1;		//旋翼转速低(0-无故障;1-故障)
    unsigned long long zjhyyld : 1;		//主减滑油压力低(0-无故障;1-故障)
    unsigned long long hj : 1;			//火警(0-无故障;1-故障)
    unsigned long long fdjtc : 1;		//发动机停车(0-无故障;1-故障)
    unsigned long long fdjskxtsx : 1;	//发动机数控系统失效(0-无故障;1-故障)
    unsigned long long fdjhyyd : 1;		//发动机滑油压低(0-无故障;1-故障)
    unsigned long long fdjryyd : 1;		//发动机燃油压低(0-无故障;1-故障)
    unsigned long long qfglsjsy10s : 1;	//起飞功率时间剩余10s(0-无故障;1-故障)
    unsigned long long Npcz : 1;			//Np超转(0-无故障;1-故障)
    unsigned long long dyw : 1;			//低油位(0-无故障;1-故障)
    unsigned long long ryfdy : 1;		//燃油阀断油(0-无故障;1-故障)
    unsigned long long fkjsjsx : 1;		//飞控计算机失效(0-无故障;1-故障)
    unsigned long long fydjsx : 1;		//俯仰舵机失效(0-无故障;1-故障)
    unsigned long long zhgdjsx : 1;		//左横滚舵机失效(0-无故障;1-故障)
    unsigned long long yhgdjsx : 1;		//右横滚舵机失效(0-无故障;1-故障)
    unsigned long long wjdjsx : 1;		//尾桨舵机失效(0-无故障;1-故障)
    unsigned long long sfkzqsx : 1;		//伺服控制器失效(0-无故障;1-故障)
    unsigned long long ztjslsx : 1;		//姿态角速率失效(0-无故障;1-故障)
    unsigned long long wsqsx : 1;		//位/速全失效(0-无故障;1-故障)
    unsigned long long gsjqsx : 1;		//高/升降全失效(0-无故障;1-故障)
    unsigned long long cksx : 1;			//测控失效(0-无故障;1-故障)
    unsigned long long ydyqsx : 1;		//引导源全失效(0-无故障;1-故障)
    unsigned long long zdgp : 1;			//自动改平(0-无故障;1-故障)
    unsigned long long czsdcx : 1;		//垂直速度超限(0-无故障;1-故障)
    unsigned long long jcydxxsx : 1;		//舰船运动信息失效(0-无故障;1-故障)
    unsigned long long nbydxxsx : 1;		//内部引导信息失效(0-无故障;1-故障)
    unsigned long long wbydxxsx : 1;		//外部引导信息失效(0-无故障;1-故障)
    unsigned long long ESMCzsx : 1;		//ESMC总失效(0-无故障;1-故障)
    unsigned long long yjsbpdgz : 1;		//应急设备配电故障(0-无故障;1-故障)
    unsigned long long xdcz1tw : 1;		//蓄电池组1脱网(0-无故障;1-故障)
    unsigned long long xdcz2tw : 1;		//蓄电池组2脱网(0-无故障;1-故障)
    unsigned long long xdcz1cw : 1;		//蓄电池组1超温(0-无故障;1-故障)
    unsigned long long xdcz2cw : 1;		//蓄电池组2超温(0-无故障;1-故障)
    unsigned long long jb : 1;			//结冰(0-无故障;1-故障)
    unsigned long long cjyjcjsjsx : 1;	//采集与监测计算机失效(0-无故障;1-故障)
    unsigned long long zjzdcx : 1;		//主桨振动超限(0-无故障;1-故障)
    unsigned long long wjzdcx : 1;		//尾桨振动超限(0-无故障;1-故障)
    unsigned long long spzzdcx : 1;		//水平轴振动超限(0-无故障;1-故障)
    unsigned long long spare:25;
}warning_message;//告警级故障信息
typedef struct
{
    unsigned char zjhywdg : 1;		//主减滑油温度高(0-无故障;1-故障)
    unsigned char zjjsx : 1;		//主减金属屑(0-无故障;1-故障)
    unsigned char wjhywdg : 1;		//尾减滑油温度高(0-无故障;1-故障)
    unsigned char wjjsx : 1;		//尾减金属屑(0-无故障;1-故障)
    unsigned char fdjrylyds : 1;	//发动机燃油滤预堵塞(0-无故障;1-故障)
    unsigned char fdjjsx : 1;		//发动机金属屑(0-无故障;1-故障)
    unsigned char fdjhywg : 1;		//发动机燃油温高(0-无故障;1-故障)
    unsigned char jjkzrhcz : 1;		//降级控制-柔和操控(0-无故障;1-故障)
    unsigned char jjkzglsx : 1;		//降级控制-功率受限(0-无故障;1-故障)
    unsigned char Ngg : 1;			//Ng高(0-无故障;1-故障)
    unsigned char Npg : 1;			//Np高(0-无故障;1-故障)
    unsigned char fstdztbyz : 1;	//发双通道状态不一致(0-无故障;1-故障)
    unsigned char fdjcw : 1;		//发动机超温(0-无故障;1-故障)
    unsigned char fdjcn : 1;		//发动机超扭(0-无故障;1-故障)
    unsigned char qdgngz : 1;		//起动功能故障(0-无故障;1-故障)
    unsigned char tcdcfgz : 1;		//停车电磁阀故障(0-无故障;1-故障)
    unsigned char czbhmkzhgz : 1;	//超转保护模块综合故障(0-无故障;1-故障)
    unsigned char Atdsksx : 1;		//A通道数控失效(0-无故障;1-故障)
    unsigned char Btdsksx : 1;		//B通道数控失效(0-无故障;1-故障)
    unsigned char fdjybgz : 1;		//发动机一般故障(0-无故障;1-故障)
    unsigned char bjdjjcycjs : 1;	//步进电机检测异常结束(0-无故障;1-故障)
    unsigned char gyglyld : 1;		//供油管路压力低(0-无故障;1-故障)
    unsigned char fkjsjydgz : 1;	//飞控计算机余度故障(0-无故障;1-故障)
    unsigned char wxdwsx : 1;		//卫星定位失效(0-无故障;1-故障)
    unsigned char fydjydgz : 1;		//俯仰舵机余度故障(0-无故障;1-故障)
    unsigned char zhgdjydgz : 1;	//左横滚舵机余度故障(0-无故障;1-故障)
    unsigned char yhgdjydgz : 1;	//右横贯舵机余度故障(0-无故障;1-故障)
    unsigned char wjdjydgz : 1;		//尾桨舵机余度故障(0-无故障;1-故障)
    unsigned char sfkzqydgz : 1;	//伺服控制器余度故障(0-无故障;1-故障)
    unsigned char ydyjj : 1;		//引导源降级(0-无故障;1-故障)
    unsigned char gqgdgz : 1;		//光纤惯导故障(0-无故障;1-故障)
    unsigned char jggdgz : 1;		//激光惯导故障(0-无故障;1-故障)
    unsigned char wxdgdd : 1;		//无线电高度低(0-无故障;1-故障)
    unsigned char fyjfzjkgz : 1;	//俯仰角辅助监控故障(0-无故障;1-故障)
    unsigned char hgjfzjkgz : 1;	//横滚角辅助监控故障(0-无故障;1-故障)
    unsigned char hxjfzjkgz : 1;	//航向角辅助监控故障(0-无故障;1-故障)
    unsigned char xdcz1rld : 1;		//蓄电池组1容量低(0-无故障;1-故障)
    unsigned char xdcz2rld : 1;		//蓄电池组2容量低(0-无故障;1-故障)
    unsigned char fdjtw : 1;		//发电机脱网(0-无故障;1-故障)
    unsigned char dmdygz : 1;		//地面电源故障(0-无故障;1-故障)
    unsigned char fdxtgz : 1;		//发电系统故障(0-无故障;1-故障)
    unsigned char fdjkzqtxgz : 1;	//发电机控制器通讯故障(0-无故障;1-故障)
    unsigned char ESMCtdgz : 1;		//ESMC通道故障(0-无故障;1-故障)
    unsigned char ESMCgnjj : 1;		//EMSC功能降级(0-无故障;1-故障)
    unsigned char ESMCzxgz : 1;		//EMSC总线故障(0-无故障;1-故障)
    unsigned char xdcz1glgz : 1;	//蓄电池组1管理故障(0-无故障;1-故障)
    unsigned char xdcz2glgz : 1;	//蓄电池组2管理故障(0-无故障;1-故障)
    unsigned char cfqtxgz : 1;		//充放器通讯故障(0-无故障;1-故障)
    unsigned char cfqgz : 1;		//充放器故障0-无故障;1-故障)
    unsigned char jbtcsx : 1;		//结冰探测失效(0-无故障;1-故障)
    unsigned char zjzdcx : 1;		//主减振动超限(0-无故障;1-故障)
    unsigned char wjzdcx : 1;		//尾减振动超限(0-无故障;1-故障)
    unsigned char fdjzdcx : 1;		//发动机振动超限(0-无故障;1-故障)
    unsigned char cjyjcjsjgz : 1;	//采集与监测计算机故障(0-无故障;1-故障)
    unsigned char zgdbjgz : 1;		//主惯导部件故障(0-无故障;1-故障)
    unsigned char bgdbjgz : 1;		//备惯导部件故障(0-无故障;1-故障)
    unsigned char dqsjjsjgz : 1;	//大气数据计算机故障(0-无故障;1-故障)
    unsigned char xtkzgl1gz : 1;	//系统控制管理1故障(0-无故障;1-故障)
    unsigned char xtkzgl2gz : 1;	//系统控制管理2故障(0-无故障;1-故障)
    unsigned char wxdgdbgz : 1;		//无线电高度表故障(0-无故障;1-故障)
    unsigned char UHFpdckgz : 1;	//UHF频段测控故障(0-无故障;1-故障)
    unsigned char dwsbgz : 1;		//敌我识别故障(0-无故障;1-故障)
    unsigned char Cbdcklgz : 1;		//C波段测控链故障(0-无故障;1-故障)
    unsigned char rwcldy1gz : 1;	//任务处理单元1故障(0-无故障;1-故障)
    unsigned char rwcldy2gz : 1;	//任务处理单元2故障(0-无故障;1-故障)
    unsigned char ctgshgz : 1;		//磁探杆收回故障(0-无故障;1-故障)
    unsigned char ctgscgz : 1;		//磁探杆伸出故障(0-无故障;1-故障)
    unsigned char sdcVNE : 1;		//速度超VNE(0-无故障;1-故障)
    unsigned char spare1 : 4;
    unsigned char spare2;
}attention_message;//注意级故障信息
typedef struct
{
    unsigned char ldzcgz : 1;		//雷达侦查故障(0-无故障;1-故障)
    unsigned char txzcgz : 1;		//通信侦查故障(0-无故障;1-故障)
    unsigned char JIDSzjgz : 1;		//JIDS中继故障(0-无故障;1-故障)
    unsigned char _906sjlgz : 1;		//906数据链故障(0-无故障;1-故障)
    unsigned char dxdtxzj1gz : 1;	//多信道通信中继1故障(0-无故障;1-故障)
    unsigned char dxdtxzj2gz : 1;	//多信道通信中继2故障(0-无故障;1-故障)
    unsigned char cbzdsbgz : 1;		//船舶自动识别故障(0-无故障;1-故障)
    unsigned char ADS_bgz : 1;		//ADS-B故障(0-无故障;1-故障)
    unsigned char hgydgz : 1;		//航管应答故障(0-无故障;1-故障)
    unsigned char zhsjcldygz : 1;	//载荷数据处理单元故障(0-无故障;1-故障)
    unsigned char fbbzx : 1;		//浮标不在线(0-无故障;1-故障)
    unsigned char ctbzx : 1;		//磁探不在线(0-无故障;1-故障)
    unsigned char hwcgqgz : 1;		//红外传感器故障(0-无故障;1-故障)
    unsigned char dscgqgz : 1;		//电视传感器故障(0-无故障;1-故障)
    unsigned char xjgz : 1;			//相机故障(0-无故障;1-故障)
    unsigned char spare : 1;
}prompt_message;//提示信息
typedef struct
{
    warning_message w_message;
    attention_message a_message;
    prompt_message p_message;
}Fault_List;
typedef struct uav_info{
    unsigned short uav_data_valid;		/*无人机数据有效标志位  diff*/
    double station_lati;				/*地面站纬度    单位：°  最小值：-90  最大值：90*/     // 无
    double station_longi;				/*地面站经度    单位：°  最小值：-180  最大值：180*/   // 无
    double uav_lati;					/*无人机纬度    单位：°  最小值：-90  最大值：90*/
    double uav_longi;					/*无人机经度    单位：°  最小值：-180  最大值：180*/
    float radio_height;			/*无人机无线电高度 */
    float air_height;			/*无人机气压高度*/
    float air_speed;				/*无人机空速*/ //	千米 /时
    float ground_speed;		    /*无人机地速*/
    float vectx_speed;			/*无人机垂直速度*/
    float uav_heading;			/*无人机航向*/ //单位：度
    float uav_pitch;				/*无人机俯仰角*/
    float uav_roll;				/*无人机横滚角*/
    /*威胁信息*/
    unsigned short uav_icon_flag;					/*无人机提示图标显示标志位  0=无效;1=显示;2=不显示;*/
    unsigned short uav_height_trend;				/*无人机高度变化趋势  0=无效;1=向上;2=向下;*/
    double uav_copter_angle;						/*无人机相对有人机方位角    单位：°  最小值：-180  最大值：180*/
    unsigned int uav_copter_distance;				/*无人机相对有人机距离*/
    unsigned short uav_angle_change;				/*无人机方位角变化趋势  0=无效;1=顺时针;2=逆时针 ;*/
    unsigned char Threat_UAV_Warning_2;				/*威胁无人机告警标志位*/
    unsigned short Thread_UAV_Horizontal_Position_2;/*威胁无人机水平相对位置*///
        /*
         * 威胁无人机水平相对位置
         * 数据位：0-15
         * 数据类型：Enum
         * 变量描述：Enum 0H=左前，1H=前，2H=右前，3H=右，4H=右后，5H=后，6H=左后，7H=左
         * */
	unsigned short Thread_UAV_Vertical_Position_2;	/*威胁无人机垂直相对位置 	 数据位：0-15 数据类型：Enum  0H=上，1H=下*/
	unsigned char Threat_UAV_near_far;				/*0-NA 1-靠近  2-远离*/
	unsigned char uav_wow;							/*无人机轮载状态 0-NA 1-地面  2-飞行*/
	float sat_height;/*卫星高度 20250905new*/
}uav_info;

typedef struct uav_mission_info{
        unsigned int task_id;			/*无人机当前任务ID*/
        unsigned short mission_status;	/*当前任务状态  0=N/A;1=正在执行任务;2=任务执行完毕;*/
        unsigned short mission_progress;/*当前任务进度*/

}uav_mission_info;

typedef struct UAV_LOADING_STATE
{
    unsigned char bar_sta;	/*探杆状态  0-NA 1-已伸出 2-已收回*/
    float bar_out_len;		/*探杆伸出长度	 mm*/
    unsigned char gd_sta;	/*光电状态  */
    unsigned char ct_sta;	/*磁探状态  */
    unsigned char buoy_sta;	/*浮标状态  */
    unsigned int fh_sta;	/*返航状态回报 0-NA 1-返航发送成功 20250819new 备用*/
}UavLoadingStatus;/*无人机载荷信息*/

typedef struct UAV_MACHINE_STATE
{
    double Np;				/*燃气发生器转速 单位rpm  分辨率1*/
    double Q;				/*输出轴扭矩  单位 N  分辨率1%*/
    double oil_temp;		/*滑油温度  单位 度  分辨率1*/
    double oil_pressure;	/*滑油压力  单位Bar  分辨率0.1*/
    double fuel_pressure;	/*燃油压力  单位MPA  分辨率0.01*/
    unsigned char engine_sta;		/*发动机状态 0-NA 1-停车 2-启动中 3-慢车 4-飞行 5-冷转 6-假启动 7-备用*/
    unsigned char rotor_stop_sta;	/*旋翼刹车状态 0-松刹 1-静刹 2-动刹 3-异常*/
    unsigned char boost1_sta;		/*增压1状态 0-断电 1-通电*/
    unsigned char boost2_sta;		/*增压2状态 0-断电 1-通电*/
    unsigned char oil_supply_sta;	/*供油状态  0-非供油  1-供油*/
}UavMachineStatus;/*无人机机电信息*/

typedef struct drone_specific_information
{
        unsigned short platform_model;//平台型号
        unsigned short platform_serial_num;//平台序号（同队内编号）（一次任务内固定）  0~4 0-未入编队 1-4-入编队的无人机（当前实现与数组下标对应，1在数组下标0,4在下标3；这样即使某无人机出队，其他无人机也会在固定的数组下标下存储）
        unsigned int platform_num;//平台编号（同平台代号） （10XX等）
        unsigned short platform_control_status;//平台控制权状态     4A 站地址判断 0x9001以上控制权在有人机
        unsigned short platform_load;//平台载荷 bit0:：光电(0：无1：搭载) bit1：雷达(0：无1：搭载) bit2：中继(0：无1：搭载) bit3：磁探(0：无1：搭载) bit4：浮标监听(0：无1：搭载)
        //平台链路状态 Platform Link Status(待定)
        unsigned short data_valid_bit;//数据有效位 /*数据有效位标识  bit0：平台1剩余任务时间 bit1：平台1剩余油量 bit2：Ng bit3：Nr bit4：T45 bit5：U bit6：U蓄  （说明：0无效 1有效）*/
        unsigned short fault_level; //平台故障状态                  // 4A系统故障等级 同映射关系
        Fault_List fault_list;  //平台故障清单                   无  4A  4B *（飞行 机电相关故障）
        unsigned short subtask_type;//平台当前执行子任务类型
        unsigned int remaining_mission_time;//平台剩余任务时间       //按分钟发送
        unsigned int residual_oil_volume;//平台剩余油量           // 总油箱
        double Ng; 				//Ng     机电参数 20250710修改short
        double Nr;				//Nr     // 4c /旋翼转速
        double T45;				//T45    //T45温度计算值
        double U;				//U                    // 不确定     5D帧中的电压
        double U_storage;		//U蓄           // 不确定 5D帧中的电压
        unsigned short uav_zishu_feedback;// 无人机自主反馈
        uav_info uav_infos; // 无人机飞行信息
        uav_mission_info uav_mission_infos; // 无人机任务状态信息
        UavLoadingStatus loading_status;	/*无人机载荷状态 20250619*/
        UavMachineStatus machine_sta;		/*无人机机电信息 20250619*/
        unsigned char formation_ability;    //无人机编队能力 0-不具备 1-具备
#ifdef _SPARE_
        unsigned char spare;
#endif
}drone_specific_information;//无人机具体信息

// CCC-DPU1/DPU2/MMM-013 MessageID 0xa2220D
typedef struct drone_state_information
{
    unsigned short drone_number;//无人机个数n
    drone_specific_information drone_specific_informations[4];//无人机具体信息
}drone_state_information;



//3.3 编队链路状态信息
typedef struct drone_link_status_information//无人机链路状态信息
{
    unsigned short uav_model;				/*平台1型号  0=N/A;1=WZ2;*/
    unsigned short uav_sn;					/*平台序号*/
    unsigned int   uav_code;				/*平台编号*/
    unsigned short uav_ulink_chanel;		/*平台U链频道号*/
    unsigned short uav_clink_down_channel;  /*平台C链下行频道号*/
    unsigned short uav_clink_up_channel;	/*平台C链上行频道号*/
    unsigned char uav_u_strength[2];		/*平台U链信号强度*/
    unsigned char uav_c_strength[2];		/*平台C链信号强度*/
    unsigned short uav_ctrl_mode;			//20241108修改补充
    unsigned int   handover_id;				/*接机站ID*/
    unsigned short handover_up_channel;		/*交接上行频道*/
    unsigned short handover_down_channel;	/*交接下行频道*/
    unsigned short handover_down_speed;		/*交接下行速率*/
    unsigned int time_remaing;				/*平台剩余时间*/ //20241108修改补充
#ifdef _SPARE_
	unsigned char spare[2];
#endif    
}drone_link_status_information;

typedef struct ControlledUavId
{
    unsigned int uav_code;
    unsigned short u_channel;			/*平台U链频道号*/
    unsigned short u_strength;			/*平台U链信号强度*/
    unsigned short c_down_channel;		/*平台C链下行频道号*/
    unsigned short c_up_channel;		/*平台C链上行频道号*/
    unsigned short c_strength;			/*平台C链信号强度*/
//    unsigned short bk_strength;			/*编宽链信号强度*/
}ControlledUavId;

typedef struct jiemianzhikong
{
    unsigned int station_id;			/*舰面站ID*/
    unsigned short station_sn;			/*舰面站序号 0:NA, 1:舰面站1 2:舰面站2*/
    unsigned short controll_uav_num;	/*指控无人机数量*/
    ControlledUavId ctrl_uav_code[2];	/*控制无人机的编号*/
}jiemianzhikong;


typedef struct U_Link_Online{
    unsigned short manned_aircraft_online : 1;
    unsigned short unmanned1_aircraft_online : 1;
    unsigned short unmanned2_aircraft_online : 1;
    unsigned short unmanned3_aircraft_online : 1;
    unsigned short unmanned4_aircraft_online : 1;
    unsigned short jianmianzhan1_online : 1;
    unsigned short jianmianzhan2_online : 1;
    unsigned short monijianmianzhongduan_online : 1;
    unsigned short reserve : 8;
}U_Link_Online;

typedef struct
{
    unsigned short spare0 : 1;
    unsigned short copter_C_uav1_up : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short copter_C_uav2_up : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short copter_C_uav3_up : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short copter_C_uav4_up : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short copter_C_uav1_down : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short copter_C_uav2_down : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short copter_C_uav3_down : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short copter_C_uav4_down : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short spare : 7;
}C_MEMBER;	/*uav与有人机 C链上行:平台->uav  下行:uav->平台*/

typedef struct
{
    unsigned short spare0 : 1;
    unsigned short station_C_uav1_up : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short station_C_uav2_up : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short station_C_uav3_up : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short station_C_uav4_up : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short station_C_uav1_down : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short station_C_uav2_down : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short station_C_uav3_down : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short station_C_uav4_down : 1;		/*0-不在网  1-在网 (锁定与否)*/
    unsigned short spare : 7;
}STATION_MEMBER;	/*uav与舰面站 C链 上下行*/

typedef struct Network_member{
    U_Link_Online U_member;//U链在网成员，1表示在网
    C_MEMBER C_UAV_F;//C链连通情况(有人/无人)
    STATION_MEMBER stationUAV;//C链联通情况(地面/无人)
    unsigned char KDLLT_Up;//空地C链上行锁定情况0-未联通，1-已联通
    unsigned char KDLLT_Down;//空地C链下行锁定情况0-未联通，1-已联通
}Network_member;//网络拓扑信息

///CCC-DPU1/DPU2/MMM-014 编队链路状态消息 MessageID 0xa2220e
typedef struct formation_link_status_information
{
    unsigned short drone_number;//无人机个数n
    drone_link_status_information drone_link_status_informations[4];//无人机链路状态信息
    unsigned short surface_station_num;  //舰面站个数m
    jiemianzhikong jiemianzhikong_info[2];  //舰面站指控状态
    Network_member network_member;//网络拓扑信息////11.16根据综显结构体进行调整补充

}formation_link_status_information;//



//3.4 综合态势
typedef struct goal_information//目标信息  // 此处有问题
{
    unsigned short tgt_sn;		/*目标序号*/
    unsigned char tgt_data_valid; // 目标有效位
    unsigned short tgt_type;	/*目标类型  0=NA;1=空中;2=陆地;3=水面;4=水下;*/
    unsigned short tgt_source;	/*目标来源  0=NA;1=有人机数据链;2=无人机光电;3=有人机后舱水声;*/
    double tgt_lati;			/*目标纬度    单位：°  最小值：-90  最大值：90*/
    double tgt_longi;			/*目标经度    单位：°  最小值：-180  最大值：180*/
    unsigned int tgt_height;	/*目标高度*/
    unsigned int tgt_speed;		/*目标速度*/
    unsigned int tgt_heading;   /*目标航向*/
    unsigned short tgt_property;/*目标属性  0=NA;1=敌;2=我;3=友;4=中立;5=不明;*/
    unsigned int tgt_code;		/*目标批号*/
    // unsigned short tgt_data_valid; // 目标有效位
#ifdef _SPARE_
    unsigned char spare[22];
#endif
}goal_information;

///CCC-DPU1/DPU2/MMM-015 MessageID 0xa2220f
typedef struct  integrated_posture
{
    unsigned short tgt_Number;			/*目标个数n*/
    goal_information goal_informations[25];		/*目标信息*/

}integrated_posture;






//3.6 任务分配结果信息
//tip: 3个方案，5个平台，8个任务（当接收全局任务规划指令时，会同时生成3个方案，按方案进行分包发送。）
typedef struct task_sequence_information//任务序列信息
{
    unsigned int id;					/*任务平台子任务ID号*/
    unsigned short modify_flag;			/*任务平台子任务是否变更  0=N/A;1=任务序列变更;2=任务区/任务点变更;3=任务航线变更;4=浮标/定测点阵型变更;*/
    unsigned short type;				/*子任务任务类型  0=N/A;1=浮标侦收;2=吊声定测;3=浮标布阵;4=通信中继;5=磁探搜索;6=磁探跟踪;7=光电搜索;8=光电跟踪;9=编队飞行;10=任务导航;11=返航;12=等待;13=临时改航;14=盘旋*/
    unsigned short point_or_area;		/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
    unsigned int point_or_area_id;		/*任务区/点/线/目标编号*/
    unsigned char finish_time_valid;	/*子任务完成时间有效位*/
    unsigned char finish_height_valid;  /*子任务任务高度有效位*/
    unsigned int finish_time;			/*子任务完成时间*/
    unsigned int finish_height;			/*子任务完成高度*/
    unsigned char exist_airway;			/*子任务是否存在航线*/
    unsigned char exist_buoy_deploy;	/*子任务是否存在浮标阵*/
    unsigned char exist_sonar_measurement_format;/*子任务是否存在定测点阵型*/
//    unsigned char spare[13];
}task_sequence_information;
typedef struct formation_synergy_mission_program//编队任务协同方案
{
    unsigned short platform_model;   /*平台1型号  0=N/A;1=20F;2=WZ2;*/
    unsigned short platform_sn;   /*平台序号*/
    unsigned int platform_code;   /*平台编号*/
    unsigned int platform_task_time;   /*任务平台任务时长*/
    unsigned short platform_subtask_number;   /*任务平台子任务个数m*/
    task_sequence_information task_sequence_informations[8];//任务序列信息（循环8次）
//    unsigned char spare[66];
}formation_synergy_mission_program;

//CCC-DPU1/DPU2/MMM-017 MessageID 0xa22211
typedef struct
{
    unsigned int plan_id;                   /*方案编号*/
    unsigned short plan_release_mode;		/*任务分配方案发布  0=N/A;1=发布;2=首次生成;3=修改;4=取消;*/ //5=单任务区覆盖
    unsigned short plan_manual_amend_flag;  /*任务分配方案是否人工修改  0=N/A;1=修改;2=未修改;*/
    unsigned short plan_replanning_flag;	/*任务分配方案是否自动重规划  0=N/A;1=是;2=否;*/
    unsigned short plan_altermode;			/*任务分配方案修改方式  0=N/A;1=子任务修改;2=航线修改;*/
    unsigned short plan_attribute;			/*方案属性  0=NA;*/
    unsigned short range;   //航程 单位km /*协同搜潜方式  0=均匀搜索;1=协同包围;2=协同拦阻;*/
    unsigned int   plan_total_time;			/*方案总时间*/
    unsigned short platform_num;			/*任务平台个数n*/
    formation_synergy_mission_program formation_synergy_mission_programs[5];//编队协同任务方案（循环5次）
}BLK_CCC_OFP_017, BLK_CCC_OFP_019;/*任务分配结果信息 描述:当接收全局任务规划指令生成时，任务分配结果会同步生成3个方案，分包发送。*/
//019为编辑中的任务分配信息;




 //3.7 有人机航路规划结果信息
typedef struct suspended_sound_buoy_deployment_point_information
{
    unsigned char JingduUpper;//吊声定测点经度有效性
    unsigned char WeiduUpper;//纬度有效性
    unsigned char HighlyUpper;//深度有效性
    longitude_and_latitude point_longitude_and_latitude;//浮标布阵点经纬度
    double profundity ;//浮标布阵点深度
#ifdef _SPARE_
    unsigned char spare[7];
#endif
}suspended_sound_buoy_deployment_point_information;//浮标布阵点信息

typedef struct suspended_sound_manned_computer_task//有人机子任务
{
    unsigned int subtask_ID_number ;//子任务ID
    unsigned short fixed_measurement_points_number;//定测点总数
    suspended_sound_buoy_deployment_point_information suspended_sound_buoy_deployment_point_informations[25];
#ifdef _SPARE_
    unsigned char spare[124];
#endif
}suspended_sound_manned_computer_task;

typedef struct suspended_sound_fixed_measurement_point//吊声定测点
{
    unsigned int program_number;//方案编号
    unsigned short subtasks_number;//有人机子任务个数m
    suspended_sound_manned_computer_task suspended_sound_manned_computer_tasks;//有人机子任务（循环8次）
}suspended_sound_fixed_measurement_point;

typedef struct buoy_deployment_point_information//浮标布阵点信息
{
    unsigned short matching_tube_number;//浮标布阵点匹配筒位号
    unsigned short buoy_type  ;//浮标布阵点浮标类型
    unsigned char longitude_validity;//浮标布阵点经度有效性
    unsigned char latitude_validity;//浮标布阵点维度有效性
    longitude_and_latitude buoy_longitude_and_latitude;//浮标布阵点经纬度
#ifdef _SPARE_
    unsigned char spare[3];
#endif
}buoy_deployment_point_information;

typedef struct buoy_manned_computer_task//有人机子任务
{
    unsigned int subtask_ID_number ;//子任务ID号
    unsigned short points_number ;//浮标布阵点总数
    buoy_deployment_point_information buoy_deployment_point_informations[25];//浮标布阵点信息
#ifdef _SPARE_
    unsigned char spare[19];
#endif
}buoy_manned_computer_task;

typedef struct buoy_deployment_point//浮标布阵点
{
    unsigned int program_number;//方案编号
    unsigned short subtasks_number;//有人机子任务个数m
    buoy_manned_computer_task buoy_manned_computer_tasks;//有人机子任务
}buoy_deployment_point;



typedef struct waypoint_information
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
    float height;//航路点高度
    float speed;//航路点速度
    float direction;//航路点航向
    unsigned int time;//航路点时间
    unsigned short payloads;//航路点载荷
#ifdef _SPARE_
    unsigned char spare[5];
#endif
}waypoint_information;//航路点信息

//typedef struct manned_computer_task//有人机子任务
//{
////    unsigned int subtask_ID_number = 6  ;//子任务ID号
////    unsigned short waypoints_number ;//航路点个数n
////    unsigned short waypoint_start_number;//航路点起始编号
//    waypoint_information waypoint_informations[40];//航路点信息
//}manned_computer_task;

//  //3.7 有人机航路规划结果存储信息

typedef struct //通用航路
{
    unsigned int   plan_code;					/*方案编号*/
    unsigned short subtask_cnt;					/*有人机子任务个数m*/
    unsigned int   subtask_index;				/*子任务ID号*/
    unsigned short task_type;					/*有人机子任务类型 20250528新增*/
    unsigned short airway_point_num;			/*航路点个数n    最小值：0  最大值：40*/
    unsigned short airway_point_start_num;		/*航路点起始编号*/
    unsigned char  plan_type;		/*航线状态 全局规划or单无人机规划    20250606 new*/
    unsigned int   uav_plan_id;			/*单无人机任务规划方案编号 航线航线状态为单无人机任务规划时有效  20250606 new*/
    waypoint_information waypoint_informations[80];//航路点信息
//    common_carrier_route tem[2];
}BLK_CCC_OFP_018_CunChu;

typedef struct //通用航路
{
    unsigned int   plan_code;					/*方案编号*/
    unsigned short subtask_cnt;					/*有人机子任务个数m*/
    unsigned int   subtask_index;				/*子任务ID号*/
    unsigned short task_type;					/*有人机子任务类型 20250528新增*/
    unsigned short airway_point_num;			/*航路点个数n    最小值：0  最大值：40*/
    unsigned short airway_point_start_num;		/*航路点起始编号*/
    unsigned char  plan_type;		/*航线状态 全局规划or单无人机规划    20250606 new*/
    unsigned int   uav_plan_id;			/*单无人机任务规划方案编号 航线航线状态为单无人机任务规划时有效  20250606 new*/
    waypoint_information waypoint_informations[40];//航路点信息
}BLK_CCC_OFP_018;

typedef struct
{
    BLK_CCC_OFP_018 blk_ccc_ofp_018;//通用航路
    buoy_deployment_point buoy_deployment_points;//浮标布阵点
    suspended_sound_fixed_measurement_point suspended_sound_fixed_measurement_points;//吊声定测点
}manned_aircraft_route_drone_route_confirmation_information;





//3.8 无人机航路规划
typedef struct planning_information_waypoint_information//航路点信息
{
    unsigned short hld_idx;				/*航路点编号		20250606 new*/
    unsigned short type;//航路点类型
    unsigned char validity_of_longitude;//航路点经度有效性
    unsigned char latitude_validity;//航路点维度有效性
    unsigned char height_validity;//航路点高度有效性
    unsigned char speed_validity;//航路点速度有效性
    unsigned char direction_validity;//航路点航向有效性
    unsigned char time_validity;//航路点时间有效性
    unsigned char payloads_validity;//航路点载荷有效性
    double longitude;//经度
    double latitude;//维度
    float height;//航路点高度
    float speed;//航路点速度
    float direction;//航路点航向
    unsigned int time;//航路点时间
    unsigned short payloads;//航路点载荷
    unsigned short causality;//航路点属性
    unsigned short standby_type;//航路点待机类型
    unsigned char standby_time_lapsNumber_cycleNumber_valid_bit;//航路点待机时间/圈数/循环次数有效位
    unsigned char standby_radius_valid_bit;//航路点待机半径有效位
    unsigned int standby_time_lapsNumber_cycleNumber;//航路点待机时间/圈数/循环次数
    unsigned int standby_radius;//航路点待机半径
#ifdef _SPARE_
    unsigned char spare[7];
#endif

}planning_information_waypoint_information;
typedef struct planning_information//单无人机单序列中的规划信息
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
    planning_information_waypoint_information planning_information_waypoint_informations[25];//航路点信息
#ifdef _SPARE_
    unsigned char spare[76];
#endif
}planning_information;
typedef struct individual_drone_routing_program//单个无人机航路方案
{
    unsigned short drone_serial_number;//无人机序号
    unsigned int drone_num;//无人机编号
    unsigned short subtasks_number ;//当前无人机子任务个数
    unsigned short subtask_index; // 从0开始
    planning_information planning_informations;
#ifdef _SPARE_
    unsigned char spare[72];
#endif
}individual_drone_routing_program;

typedef struct BLK_CCC_OFP_024
{
    unsigned int program_number;//方案编号
    unsigned char  plan_type;		/*航线状态 全局规划or单无人机规划    20250606 new*/
    unsigned int   uav_plan_id;		/*无人机任务规划方案编号 航线航线状态为单无人机任务规划时有效  20250606 new*/
//    unsigned short number_of_drones;//无人机数量
    individual_drone_routing_program individual_drone_routing_programs;//单个无人机航路方案
}BLK_CCC_OFP_024;


//无人机航路规划本地存储 20250609修改
/***********************************************************************************************/
typedef struct //单无人机单序列中的规划信息
{
    unsigned int subtask_ID_number;//子任务ID号
    unsigned char  total_packet;				/*航路点总包数    20250606 new*/ //最多十个包
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
    planning_information_waypoint_information planning_information_waypoint_informations[250];//航路点信息,最多十个包
#ifdef _SPARE_
    unsigned char spare[76];
#endif
}planning_informations;
typedef struct //单个无人机航路方案
{
    unsigned short drone_serial_number;//无人机序号
    unsigned int drone_num;//无人机编号
    unsigned short subtasks_number ;//当前无人机子任务个数
    unsigned short subtask_index; // 从0开始
    planning_informations planning_informations;
#ifdef _SPARE_
    unsigned char spare[72];
#endif
}individual_drone_routing_programs;

typedef struct
{
    unsigned int program_number;//方案编号
    unsigned char  plan_type;		/*航线状态 全局规划or单无人机规划    20250606 new*/
    unsigned int   uav_plan_id;		/*无人机任务规划方案编号 航线航线状态为单无人机任务规划时有效  20250606 new*/
//    unsigned short number_of_drones;//无人机数量
    individual_drone_routing_programs individual_drone_routing_programs;//单个无人机航路方案
}BLK_CCC_OFP_024_cunchu;

typedef struct
{
    unsigned int task_index;//任务类型参数索引
    unsigned int first_flag;//机场位置发送标志位
    unsigned int send_ready;//发送准备
    unsigned int drone_id; // 无人机编号
    unsigned short subtasks_number ;//无人机子任务个数
    unsigned short waypoints_index;//当前航路点
    unsigned short waypoints_number;//航路总数
    planning_information_waypoint_information waypoint_informations[255];//航路点信息
}UAV_SEND;

typedef struct
{
	unsigned short tasking;//是否存在任务
	unsigned short task_type;//任务类型
	unsigned short task_id;//任务id
    unsigned short route_number;//当前航线号
    unsigned short waypoint_number;//当前航点号
    unsigned short hull_number;//航路总数
    planning_information_waypoint_information waypoint[255];//航路点信息
}UAV_ROUTE;
/***********************************************************************************************/
//无人机领航命令 20250906new 0xa2222e
typedef struct
{
	unsigned int uav_id;			//无人机编号
	unsigned short uav_sn;			//无人机序号
	/* 0-非编队
	 * 1-编队-按航线飞-编队保持-具备领航
	 * 2-编队-按航线飞-编队保持-不具备领航
	 * 3-编队-按航线飞-编队集结
	 * 4-编队-按航线飞-队形变换
	 * 5-编队-按航线飞-编队解散
	 * 6-编队-按航线飞-退出编队
	 * 7-编队-按航线飞-单机飞行
	 * 8-编队-有人机领航 */
	unsigned char temfly_status;	//领航状态 0-NA 1-集结完成 2-进入领航 3-领航失败 4-领航超时 5-退出领航成功 6-退出领航失败 7-退出领航超时
}BLK_CCC_OFP_026;

//有人机进入领航条件 20251113new 0xa22233
typedef struct
{
	unsigned int uav_id;			//无人机编号
	unsigned short uav_sn;			//无人机序号
	unsigned char display;			//显示标志位 0-不显示 1-显示
	unsigned char conditon1;		//有人机水平速度(>28m/s) 		0-不满足 1-满足
	unsigned char conditon2;		//与有人机水平距离(<20km)		0-不满足 1-满足
	unsigned char conditon3;		//有人机绝对高度大于安全高度	0-不满足 1-满足
	unsigned char conditon4;		//有人机航迹角差(<=90)		0-不满足 1-满足
	unsigned char conditon5;		//有人机U链					0-不满足 1-满足
	unsigned char conditon6;		//编队保持					0-不满足 1-满足
}BLK_CCC_OFP_027;

//有人机退出领航条件 20251113new 0xa22234
typedef struct
{
	unsigned int uav_id;			//无人机编号
	unsigned short uav_sn;			//无人机序号
	unsigned char conditon1;		//有人机水平速度(>28m/s) 		0-不满足 1-满足
	unsigned char conditon2;		//与有人机水平距离(<20km)		0-不满足 1-满足
	unsigned char conditon3;		//有人机绝对高度大于安全高度	0-不满足 1-满足
	unsigned char conditon4;		//有人机航迹角差(<=90)		0-不满足 1-满足
	unsigned char conditon5;		//有人机U链					0-不满足 1-满足
	unsigned char conditon6;		//编队保持					0-不满足 1-满足
}BLK_CCC_OFP_028;

//无人机碰撞启动信息 20251113new 0xa22235
//无人机防撞信息
typedef struct
{
	unsigned int uav_id;			//无人机编号
	unsigned short uav_sn;			//无人机序号
	unsigned char uav_manned;		//与有人机防碰撞启动 0-未启动 1-启动
	unsigned char uav_uav;			//与无人机防碰撞启动 0-未启动 1-启动
}UAV_AVOID;
typedef struct
{
	unsigned char uav_num;			//无人机数量 固定填2
	unsigned char display;			//显示标志位 0-不显示 1-显示
	UAV_AVOID uav_avoid[2];			//无人机碰撞信息
}BLK_CCC_OFP_029;

//应急返航区域 20251113new 0xa22236
//无人机应急区域信息
typedef struct
{
	double lon;						//经度
	double lat;						//纬度
	unsigned int height;			//高度 m
	unsigned short speed;			//速度 m/s
	unsigned char tpye;				//航路点类型 0-一般航路点 1-盘旋点 2-悬停点
	unsigned short circle_time;		//盘旋圈数/悬停时间
	double rad;						//盘旋半径
}Avoid_POINT;//航点信息
typedef struct
{
	unsigned int uav_id;			//无人机编号
	unsigned short uav_sn;			//无人机序号
	double Lat;						//区域中心经度
	double Lon;						//区域中心纬度
	double Angle;					//返航区域长度方向角
	int Long;						//航区域长度 单位:m
	int wide;						//航区域宽度 单位:m
	unsigned char emergency_num;			//应急航点数量
	Avoid_POINT emergency_point[25];	//应急航线信息
}UAV_EMAREA;
typedef struct
{
	unsigned char uav_num;			//无人机数量
	UAV_EMAREA uav_emarea[2];			//无人机应急区域信息
}BLK_CCC_OFP_030;

//3.9 协同任务执行状态提示
typedef struct
{
	unsigned int uav_code;					/*提示无人机编号*/
	unsigned short uav_sn;					/*提示无人机序号*/
	unsigned short tips_index;				/*提示信息流水号 1-2048循环使用*/
	unsigned short subtask_type;			/*子任务任务类型  0=N/A;1=浮标侦收;2=吊声定测（无人机不使用）;3=浮标布阵（无人机不使用）;4=通信中继;5=磁探搜索;6=磁探跟踪;7=光电搜索;8=光电跟踪;9=编队飞行;10=任务导航;11=返航;12=悬停等待;13=临时改航;14=盘旋等待;15=攻击*/
	unsigned short subtask_point_or_area;	/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
	unsigned int   subtask_point_or_area_id;/*任务区/点/线/目标编号*/
	unsigned short tips_type;				/*提示信息类型  0=NA;1=无人机Xxx任务即将完成，是否重新执行XXXX任务;2=目标态势改变，是否进行重规划;3=无人机X即将抵达XXX任务点;4=无人机X任务降级;5=无人机X故障;6=无人机X C链通信中断;7=无人机X U链通信中断;8=XXXXX申请无人机Y控制权，是否移交？;9=无人机X 控制权交接;10=无人机X开始执行XXX任务;
	                                                                                                                                重规划:11:航线冲突提示；12：航线冲突消解提示 ；13：重规划航线发布失败提示；14：消解航线发布失败提示；21：航线发布中提示；0：消隐弹窗；32：载荷重规划任务类型，区域信息*/
    double longitude;//规避点经度
    double latitude;//规避点纬度
}BLK_CCC_OFP_032;/*协同任务执行状态提示*/

//3.10 任务区/空域信息
typedef struct cycle
{
    longitude_and_latitude cycle_longitude_and_latitude;//圆心经纬度
    float radius;//圆心半径
}cycle;
typedef struct polygonal//多边形
{
    unsigned short point_number;//点数n
    point_coordinate point_coordinates[10];//点坐标
#ifdef _SPARE_
    unsigned char spare[8];
#endif
}polygonal;
typedef struct area_information//区域信息
{
    unsigned int area_code;//区域编号1-30
    unsigned short area_type;//区域类型：1-任务区，2-空域，3-危险区
    unsigned short area_source;//区域来源0-无效，1-预规划加载，2-本机手动编辑，3-决策生成
    unsigned short area_shape;//区域形状：0-无效，1-圆形，2-多边形

    unsigned short area_platform_num;  //区域所属平台：0-无效，1-归属无人机，2-归属平台
    unsigned int drone_numbe;//空域所属无人机序号：根据编队情况1234
    unsigned char upper_height_limit_valid_bit;//空域有效性：0-无效，1-有效
    unsigned char lower_height_limit_valid_bit;//区域高度下限有效位置：0-无效，1-有效
    float upper_height_limit;//区域高度上限：单位米1600
    float lower_height_limit;//区域高度下限：单位米1
    cycle cycles;//圆
    polygonal polygonals;//多边形
#ifdef _SPARE_
    unsigned char spare[4];
#endif
}area_information;
typedef struct
{
    unsigned short curBagNo; /*当前包号 从0开始*/
    unsigned short bagTatal; /*总包数*/
    unsigned short area_number;//区域个数
    area_information area_informations[8];//区域信息
}BLK_CCC_OFP_033;


//3.11 任务点信息
typedef struct point_information//点信息
{
    unsigned int point_number;//点编号
    unsigned short point_source_of_information;//点信息来源
    double longitude;//经度
    double latitude;//维度
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
    int millisecond;//毫秒
    unsigned short point_property;//目标属性
    unsigned short point_type;//目标类型
    unsigned int point_batchNumber;//批号
#ifdef _SPARE_
    unsigned char spare[5];
#endif
}point_information;
typedef struct BLK_CCC_OFP_034
{
    unsigned short point_number;//任务点个数
    point_information point_informations[30];//点信息
}BLK_CCC_OFP_034;



//3.12 任务线信息
typedef struct line_information
{
    unsigned int line_num;//线编号
    unsigned short line_type;//线类型
    unsigned short line_point_number;//任务线点个数
    point_coordinate point_coordinates[10];//点经纬度
#ifdef _SPARE_
    unsigned char spare[52];
#endif
}line_information;
typedef struct BLK_CCC_OFP_035
{
    unsigned short line_number;//任务线条数
    line_information line_informations[5];//线信息
}BLK_CCC_OFP_035;




//3.13 协同指控指令信息
typedef struct
{
    unsigned int subtask_point_or_area_id;   /*目标区/点编号*/
}AREA_NUMBER;/*目标区域个数*/

typedef struct
{
    unsigned short command_id;				/*指令序号*/
    unsigned short command_type;			/*指令类型  0=NA;1=重规划;2=方案规划;3=控制权交接;*/
    unsigned char command_send_time_vaild;  /*数据有效标识 Boolean*/
    unsigned int command_send_time;			/*发送时间*/
    unsigned short receive_state;			/*接收状态  0=NA;1=已接收;2=未接受;*/
    unsigned short execute_state;			/*执行状态  0=NA;1=已执行;2=未执行;*/
    unsigned short task_type;				/*内容	0=NA;1=光电搜索跟踪;2=雷达搜索跟踪;3=吊声搜索;4=浮标布放;5=浮标侦听;6=磁探搜索;7=磁探跟踪;8=通信中继;9=攻击;10=起飞过渡;11=返航;12=等待;13=临时改航;14=预规划A;15=预规划B;16=控制权申请;17=控制权释放;18=检查反潜;19=应召反潜;*/
    unsigned short area_or_point_number;	/*目标区/点个数*/
    AREA_NUMBER area_number[8];				/*目标区域编号  diff*/
}op_code;/*协同指控指令*/

typedef struct
{
      unsigned char  mfdNo;				/*mfd序号 0-NA 1-mfd1 2-mfd2 3-mfd3 4-mfd4*/
      unsigned short PageIdCurrent;			/*当前页数*/
      unsigned short PageIdTotle;			/*总页数*/
      unsigned short UAV_Serial_Number1_5;	/*平台序号*/
      unsigned int	 UAV_ID_2_2_1;			/*平台编号*/
      unsigned short command_num;			/*协同指控指令条数*/
      op_code xt_commands[10];/*协同指控指令*/
}BLK_CCC_OFP_036;/*协同指控指令信息*/



//3.15 预规划加载结果确认
typedef struct confirmation_of_preplanning_loading_results
{
    char success_or_failure;//是否成功
}confirmation_of_preplanning_loading_results;


//3.16 预规划方案
typedef struct pre_planning_program
{
    unsigned short pre_planning_program_number;//预规划方案个数
    unsigned int pre_planning_program_num1;//预规划方案1编号
    unsigned int pre_planning_program_num2;//预规划方案2编号
//    unsigned int pre_planning_program_num3;//预规划方案3编号
//    unsigned int pre_planning_program_num4;//预规划方案4编号
}pre_planning_program;


//3.17 辅助决策错误提示
typedef struct assisted_decision_making_error_alerts
{
    char wrong_MSGid[20];//存在错误的MSGid
    char alert_message_text[200];//提示信息文本
}assisted_decision_making_error_alerts;


//3.18 协同指控系统自检测故障清单
typedef struct default_orders
{
    unsigned char spare[64];
}default_orders;/*协同指控系统自检测故障清单*/


//无人机光电控制权反馈
typedef struct guangdian_kongzhi_feedback
{
    unsigned short ssds_control_feedback;   /*无人机光电控制权反馈  0=NA;1=前舱;2=后舱;*/
}guangdian_kongzhi_feedback;/*无人机光电控制权反馈*/

typedef struct {
    //    unsigned char tv_infrared_switch; // 电视/红外切换 0H=电视;1H=红外
    //    unsigned char long_short_switch; // 长焦/短焦切换 0H=长焦;1H=短焦
    //    unsigned char spare0; //
        unsigned char type; // 电视/红外/长焦/短焦切换 0H=NA;1H=电视长焦;2H=电视短焦;3H=红外长焦;4H=红外短焦
        unsigned char lock_scan_switch; // 锁定/扫描切换 0H=NA;1H=锁定;2H=扫描
        unsigned char lock_param; // 锁定参数 0H=NA;1H=垂直下视锁定;2H=前向锁定;3H=右向锁定;4H=后向锁定;5H=左侧锁定
        unsigned char scan_param; // 扫描参数 0H=NA;1H=前向方位扫描;2H=右侧方位扫描;3H=后向方位扫描;4H=左侧方位扫描;5h=前向俯仰扫描;6H=右侧俯仰扫描;7H=后向俯仰扫描;8H=左侧俯仰扫描
        unsigned char ir_pwr_fb; // 红外上电状态反馈 0=NA;1=已上电;2=正在上电;3=未上电
#ifdef _SPARE_
    unsigned char spare[53];
#endif
} DPU_video_param_setting_feedback;// 视频参数设置反馈

typedef struct {
    float photoelectric_platform_azimuth; // 光电平台方位角
    float photoelectric_platform_pitch;   // 光电平台俯仰角
    unsigned short target_param_significant_bit; // 目标参数有效位
    // 8-15 目标2参数有效位 0H=无效;1H=有效
    // 0-7  目标1参数有效位 0H=无效;1H=有效
    float target1_dist; // 目标1距离
    float target2_dist; // 目标2距离
#ifdef _SPARE_
    unsigned char spare[94];
#endif
} DPU_photoelectric_state_feedback;// 光电状态反馈

typedef struct {
    unsigned char video_source_type; // 视频源类型（0H=NA;1H=有人机视频;2H=无人机1视频;3H=无人机2视频;4H=无人机3视频;5H=无人机4视频）
    DPU_video_param_setting_feedback video_param_setting_feedbacks; // 视频参数设置反馈
    DPU_photoelectric_state_feedback photoelectric_state_feedbacks; // 光电状态反馈
} DPU_guangdina_video;// 无人机光电视频控制反馈


typedef struct
{
	unsigned short uav_sn;					//无人机序号
	unsigned int uav_id;					//无人机编号
	unsigned short point_num;				//顶点个数 0~8
	point_coordinate point_coordinates[10];	//点坐标
}TASK_AREA;

typedef struct
{
	unsigned int program_number;//方案编号
	unsigned char taskarea_num;
	TASK_AREA task_area[8];
}BLK_CCC_OFP_047;//攻击安全区 0xa222b4


/************** 120 *****************/
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
}BLK_CCC_OFP_120;//指令微调 0xa22278

/************ Icd 更新新增 ***************/
typedef struct
{
	char fanganType;//0-NA 1-全局规划 2-单目标攻击
    char fanganGenStatus; /*方案生成状态信息反馈，0-NA, 1-方案生成中，2-方案已生成,3-生成失败*/
    char fanganSubStatus; /*方案发布状态信息反馈，0-NA， 1-方案发布中,2-方案已发布,3-发布失败*/
    unsigned char hangxianGenStatus;   /*航线生成状态提示信息  0=NA;1=航线生成中;2=航线已生成;3-生成失败  20250103*/
    char failreason[200];   /*修改失败原因*/
}fangan_bianji_zhaungtai;/*方案编辑状态反馈*/


typedef struct
{
    unsigned char ctrl_releasing;	/*控制权正在释放中  0-NA 1-控制权正在释放  2-控制权已经释放*/
    unsigned char ctrl_recieving;	/*控制权正在接收中  0-NA 1-控制权正在接收  2-控制权已经接收*/
}BLK_CCC_OFP_003;/*控制权交接状态反馈*/


/************************************CCC_OFP****************************************/
/**************302*****************/

typedef struct
{
    unsigned int ArrayProgramCommand:1;   /*布阵规划指令  1-是/0-否*/
    unsigned int SingleDouble:1;   /*单机或者协同  1-协同/0-单机*/
    unsigned int Null1:6;
    unsigned int PlanWay:8;   /*布阵方式  0=N/A;1=自动规划;2=图形规划;3=自定义;*/
    unsigned int Null0:16;
}BuoyArrayPlan;  /*浮标布阵规划*/

typedef struct
{
    unsigned short AutoPlayMisType:8;   /*自动规划任务类型  0=N/A;1=目标点;2=圆形区域;3=矩形区域;*/
    unsigned short Null0:8;
}AutoPlanSet;  /*自动规划设置*/


typedef struct
{
    unsigned short tgtSource_DPU1:8;   /*目标源  0=N/A;1=雷达;2=吊声;3=浮标;4=电侦;5=光电;6=JIDS;7=905;8=卫通链;9=AIS;10=融合;11=吊声(外推);12=浮标(外推);13=人工;*/
    unsigned short TargetProperty:8;   /*目标属性  0=N/A;1=敌;2=我;3=友;4=中立;5=不明;*/
}TargetSource;  /*目标源*/

typedef struct
{
    unsigned short LonValid:1;   /*经度有效位  1-有效/0-无效*/
    unsigned short LatValid:1;   /*纬度有效位  1-有效/0-无效*/
    unsigned short RectionValid:1;   /*目标航向有效  1-有效/0-无效*/
    unsigned short SpeedValid:1;   /*目标速度有效  1-有效/0-无效*/
    unsigned short PositionAccuracyValid:1;   /*位置精度有效位  1-有效/0-无效*/
    unsigned short DirectionAccuracyValid:1;   /*航向精度有效位  1-有效/0-无效*/
    unsigned short SpeedAccuracyValid:1;   /*目标航速精度有效  1-有效/0-无效*/
    unsigned short Null0:9;
}DataValid;  /*数据有效位*/

typedef struct
{
    TargetSource tgtSource1;      /*目标源*/
    unsigned char Null0;
    unsigned int SerNum_FC00_DPU1;   /*目标批号    单位：无*/
    unsigned int LastFindTime_DPU1;   /*最后发现时间    单位：毫秒*/
    DataValid DATAValid_1;      /*数据有效位*/
    double TargetLon;   /*目标经度*/
    double TargetLat;   /*目标纬度*/
    double tgtCourseT;   /*目标航向（真）    单位：°  最小值：0  最大值：360*/
    double tgtSpeed;   /*目标速度    单位：Km/h  最小值：0  最大值：300*/
    double TargetPositionAccuracy;   /*目标位置精度    单位：米  最小值：0  最大值：10000*/
    double TargetHDGAccuracy;   /*目标航向精度    单位：度  最小值：0  最大值：360*/
    double TargetSpeedAccuracy;   /*目标航速精度    单位：米/秒  最小值：0  最大值：1000*/
}TargetPoint;/*目标点*/

typedef struct
{
    unsigned short RectangleValid:1;   /*矩形有效状态  1-有效/0-无效*/
    unsigned short Null0:15;
}DataStatus;  /*数据状态*/

typedef struct
{
    DataStatus data_status;      /*数据状态*/
    double LonData;   /*矩形区域中心点经度    单位：°  最小值：-180  最大值：180*/
    double LatData;   /*矩形区域中心点纬度    单位：°  最小值：-90  最大值：90*/
    double AngleData;   /*矩形区域纵向与正北方向夹角    单位：°  最小值：0  最大值：360*/
    double BroadWiseData;   /*矩形区域横向边长    单位：km  最小值：0  最大值：150*/
    double DirectWiseData;   /*矩形区域纵向边长    单位：km  最小值：0  最大值：150*/
}RectangularRegion;/*矩形区域*/


typedef struct
{
    unsigned short CircleValid:1;   /*圆形有效状态  1-有效/0-无效*/
    unsigned short Null0:15;
}DataStatus1;  /*数据状态*/


typedef struct
{
    DataStatus1 data_status1;      /*数据状态*/
    double LonData;   /*圆心经度    单位：°  最小值：-180  最大值：180*/
    double LatData;   /*圆心纬度    单位：°  最小值：-90  最大值：90*/
    double Radius;   /*半径    单位：Km  最小值：0  最大值：150*/
}CircleRegion;/*圆形区域*/


typedef struct
{
    AutoPlanSet AutoPlaySet_1;      /*自动规划设置*/
    TargetPoint target_point;      /*目标点*/
    RectangularRegion RectangleZone;      /*矩形区域*/
    CircleRegion CircleZone;      /*圆形区域*/
}AutoPlanInfo;/*自动规划信息*/

typedef struct
{
    unsigned int PlayChoose:8;   /*阵形选择  0=N/A;1=圆形阵;2=扇形阵;3=多直线阵;4=折线阵;5=方形覆盖阵;6=主动增程阵;7=Difix定位阵;8=CODAR定位阵;*/
    unsigned int BuoyType:8;   /*浮标类型  0=N/A;1=被动全向（L）;2=被动定向（D）;3=主动全向1（R）;4=主动全向2;5=主动全向3;6=主动全向4;7=主动定向1（T）;8=主动定向2;9=主动定向3;10=主动定向4;11=垂直线列阵（V）;12=低频声源（P）;13=扩展阵（X）;14=海噪声（A）;15=温深（B ）;*/
    unsigned int Null0:16;
}GraphicPlanSet;  /*图形规划设置*/


typedef struct
{
    unsigned short CircleValid:1;   /*圆形有效状态  1-有效/0-无效*/
    unsigned short Null1:7;
    unsigned short ControlItem:1;   /*控制选项  1-数量控制/0-半径控制*/
    unsigned short CircleCenterChoose:1;   /*圆心布置选项  1-布置/0-不布置*/
    unsigned short Null0:6;
}DataStatus2;  /*数据状态*/

typedef struct
{
    DataStatus2 data_status2;      /*数据状态*/
    double LonData;   /*圆心经度    单位：°  最小值：-180  最大值：180*/
    double LatData;   /*圆心纬度    单位：°  最小值：-90  最大值：90*/
    unsigned short Num;   /*外环数量    单位：个  最小值：3  最大值：25*/
    double Radius;   /*半径    单位：Km  最小值：0  最大值：150*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
}CirclePlan;/*圆形阵*/

typedef struct
{
    unsigned short CircleValid:1;   /*扇形有效状态  1-有效/0-无效*/
    unsigned short Null1:7;
    unsigned short LineChoose:1;   /*直边浮标布设选项  1-布置/0-不布置*/
    unsigned short Null0:7;
}DataStatus3;  /*数据状态*/


typedef struct
{
    DataStatus3 data_status3;      /*数据状态*/
    double LonData;   /*圆心经度    单位：°  最小值：-180  最大值：180*/
    double LatData;   /*圆心纬度    单位：°  最小值：-90  最大值：90*/
    double AxleDirection;   /*轴方位角    单位：°  最小值：0  最大值：360*/
    double SectorAngle;   /*扇区张角    单位：°  最小值：0  最大值：360*/
    double Radius;   /*半径    单位：Km  最小值：0  最大值：150*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
}SectorPlan;/*扇形阵 描述:*/

typedef struct
{
    unsigned short MultiLineStatus:1;   /*多直线阵有效状态  1-有效/0-无效*/
    unsigned short Null1:7;
    unsigned short AcrossChoose:1;   /*交叉选项  1-是/0-否*/
    unsigned short Null0:7;
}DataStatus4;  /*数据状态*/

typedef struct
{
    DataStatus4 data_status4;      /*数据状态*/
    double LonData;   /*经度    单位：°  最小值：-180  最大值：180*/
    double LatData;   /*纬度    单位：°  最小值：-90  最大值：90*/
    double PreventDirection_1;   /*拦截方向    单位：°  最小值：0  最大值：360*/
    double PreventWide;   /*拦截宽度    单位：Km  最小值：0  最大值：150*/
    double DoubleLineDistance;   /*双线间距    单位：Km  最小值：0  最大值：150*/
    unsigned short LineNum;   /*行数    单位：个  最小值：1  最大值：10*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
}MultiLinePlan;/*多直线阵 描述: */

typedef struct
{
    unsigned short FoldLineStatus:1;   /*折线阵有效状态  1-有效/0-无效*/
    unsigned short Null1:7;
    unsigned short VertexOpt:1;   /*顶点浮标布设选项  1-顶点放置/0-顶点不放置*/
    unsigned short Null0:7;
}DataStatus4_1;  /*数据状态*/


typedef struct
{
    DataStatus4_1 data_status4_1;      /*数据状态*/
    double LonData;   /*经度    单位：°  最小值：-180  最大值：180*/
    double LatData;   /*纬度    单位：°  最小值：-90  最大值：90*/
    double PreventDirection_1;   /*拦截方向    单位：°  最小值：0  最大值：360*/
    double FoldLineAngle;   /*折线夹角    单位：°  最小值：0  最大值：180*/
    double FoldLineLength;   /*折线边长    单位：Km  最小值：0  最大值：150*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
}FoldLinePlan;/*折线阵 描述:*/

typedef struct
{
    unsigned short RectangleValid:1;   /*方形覆盖阵有效状态  1-有效/0-无效*/
    unsigned short Null1:7;
    unsigned short AcrossChoose:1;   /*交叉选项  1-是/0-否*/
    unsigned short Null0:7;
}DataStatus_1;  /*数据状态*/


typedef struct
{
    DataStatus_1 data_status_1;      /*数据状态*/
    double LonData;   /*矩形区域中心点经度    单位：°  最小值：-180  最大值：180*/
    double LatData;   /*矩形区域中心点纬度    单位：°  最小值：-90  最大值：90*/
    double AngleData;   /*矩形区域纵向与正北方向夹角    单位：°  最小值：0  最大值：360*/
    double BroadWiseData;   /*矩形区域横向边长    单位：Km  最小值：0  最大值：150*/
    double DirectWiseData;   /*矩形区域纵向边长    单位：Km  最小值：0  最大值：150*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
}RectangleOverPlan;/*方形覆盖阵 描述:*/

typedef struct
{
    unsigned short ActiveIncreaseArrayValid:1;   /*主动增程阵有效状态  1-有效/0-无效*/
    unsigned short Null0:15;
}DataStatus_1_1;  /*数据状态*/

typedef struct
{
    unsigned int LineNum:16;   /*行数    最小值：1  最大值：5*/
    unsigned int ColumnNum:16;   /*列数    最小值：1  最大值：5*/
}LineColum;  /*行列数*/

typedef struct
{
    DataStatus_1_1 data_status_1_1;      /*数据状态*/
    double LonData;   /*矩形区域中心点经度    单位：°  最小值：-180  最大值：180*/
    double LatData;   /*矩形区域中心点纬度    单位：°  最小值：-90  最大值：90*/
    double DirectionAngle;   /*方位角    单位：°  最小值：0  最大值：360*/
    LineColum LineColumnNum;      /*行列数*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
}ActiveIncreasePlan;/*主动增程阵 描述:*/

typedef struct
{
    unsigned short DifixStatus:1;   /*Difix定位阵有效状态  1-有效/0-无效*/
    unsigned short Null0:15;
}DataStatus4_2;  /*数据状态*/

typedef struct
{
    DataStatus4_2 data_status4_2;      /*数据状态*/
    double LonData;   /*起点经度    单位：°*/
    double LatData;   /*起点纬度    单位：°*/
    double DirectionAngle;   /*方位角    单位：°  最小值：0  最大值：360*/
    double Distance_1_1;   /*Difix阵对相对起点1的距离    单位：Km  最小值：0  最大值：10*/
    double DoubleLineDistance1;   /*浮标对间距    单位：Km  最小值：0  最大值：10*/
}DifixLocationPlan;/*Difix定位阵 描述:*/

typedef struct
{
    unsigned short CodarStatus:1;   /*Codar定位阵有效状态  1-有效/0-无效*/
    unsigned short Null1:7;
    unsigned short DoubleLineDistance1:1;   /*浮标对间距  1-210m/0-105m*/
    unsigned short Null0:7;
}DataStatus4_2_1;  /*数据状态*/


typedef struct
{
    DataStatus4_2_1 data_status4_2_1;      /*数据状态*/
    double Point1LonValid;   /*起点1经度    单位：°*/
    double Point1LatValid;   /*起点1纬度    单位：°*/
    double DirectionAngle;   /*方位角    单位：°  最小值：0  最大值：360*/
    double Distance_1;   /*Codar阵对相对起点1的距离    单位：km  最小值：10*/
}CodarLocationPlan;/*Codar定位阵 描述:
                                        */


typedef struct
{
    GraphicPlanSet FigurePlaySet_1;      /*图形规划设置*/
    CirclePlan circle_plan;      /*圆形阵*/
    SectorPlan sector_plan;      /*扇形阵  信号描述： */
    MultiLinePlan MultiLine;      /*多直线阵  信号描述：*/
    FoldLinePlan FoldLine;      /*折线阵  信号描述：  */
    RectangleOverPlan RectangleOverLap;      /*方形覆盖阵  信号描述：  */
    ActiveIncreasePlan ActiveIncreaseArray;      /*主动增程阵  信号描述：			*/
    DifixLocationPlan DifixLocation;      /*Difix定位阵  信号描述：				*/
    CodarLocationPlan DoubleLineDistance11;      /*Codar定位阵  信号描述：  */
}GraphicPlanInfo;/*图形规划信息*/

typedef struct
{
    double LatData;   /*纬度    单位：°*/
    double LonData;   /*经度    单位：°*/
    unsigned int BuoyStationNum;   /*分配的浮标工位号*/
}CustomPoint;/*自定义点位信息*/

typedef struct
{
    unsigned int Null0:8;
    unsigned int BuoyType:8;   /*浮标类型  0=N/A;1=被动全向（L）;2=被动定向（D）;3=主动全向1（R）;4=主动全向2;5=主动全向3;6=主动全向4;7=主动定向1（T）;8=主动定向2;9=主动定向3;10=主动定向4;11=垂直线列阵（V）;12=低频声源（P）;13=扩展阵（X）;14=海噪声（A）;15=温深（B ）;*/
    unsigned int BuoyWorkDepth:8;   /*浮标工作深度  0=N/A;1=15米;2=40米;3=60米;4=80米;5=150米;6=300米;*/
    unsigned int WorkChannel:8;   /*浮标设定工作时间  0=N/A;1=10分钟;2=1小时;3=4小时;4=6小时;5=8小时;*/
}GraphicPlanSet_1;  /*规划设置*/

typedef struct
{
    unsigned short AddDieBuoy:1;   /*叠加未存活补投指令  1-叠加/0-不叠加*/
    unsigned short Null0:7;
    unsigned short PlanLocationNum:8;   /*布阵点数量    单位：个  最小值：0  最大值：75*/
}DataStatus5_1;  /*数据状态*/

typedef struct
{
    GraphicPlanSet_1 FigurePlaySet_1_1;      /*规划设置*/
    DataStatus5_1 data_status;      /*数据状态*/
    CustomPoint custom_point[60];      /*自定义点位信息*/
}CustomPlanInfo;/*自定义规划信息*/

typedef struct
{
    unsigned int Plan_ID;   /*方案编号*/
    unsigned int SubTask_Id;   /*任务平台子任务ID号*/
    BuoyArrayPlan BuoyPlanProject;      /*浮标布阵规划*/
    AutoPlanInfo AutoPlanInformation;      /*自动规划信息*/
    GraphicPlanInfo FigurePlanInformation;      /*图形规划信息*/
    CustomPlanInfo CustomPlanInformation;      /*自定义规划信息*/
}BLK_CCC_OFP_302;/*浮标—浮标布阵规划 描述:实际发送长度=块标识+相应规划信息长度。*/




/**************403*****************/
typedef struct
{
    unsigned int POINT_PLAN_SYMBOL:2;   /*协同方式  0=单机;1=双机;2=三机;*/
    unsigned int PlaneFlag:2;   /*长僚机标志  0=N/A;1=长机;2=僚机1;3=僚机2;*/
    unsigned int Null0:12;
    unsigned int AvgSinglePointNumMax:8;   /*平均单架机最大定测点数    最小值：1  最大值：10*/
    unsigned int PlanWay:8;   /*规划方式  0=N/A;1=自动规划;2=图形规划;3=自定义;*/
}SonarPlanSimbol;  /*吊声定测点规划标识*/

typedef struct
{
    unsigned int Null0:16;
    unsigned int TaskType1:8;   /*任务阶段  0=N/A;1=搜索阶段;2=跟踪阶段;3=结束跟踪;*/
    unsigned int TaskType1_1:8;   /*任务类型  0=N/A;1=应召反潜;2=巡逻反潜;*/
}TaskTypeStage;  /*任务类型及阶段*/

typedef struct
{
    TargetSource tgtSource_;      /*目标源*/
    unsigned char Null0;
    unsigned int SerNum_FC00_DPU1;   /*目标批号    单位：无*/
    unsigned int LastFindTime_DPU1;   /*最后发现时间    单位：ms*/
    DataValid DATAValid;      /*数据有效位*/
    double TargetLon;   /*目标经度*/
    double TargetLat;   /*目标纬度*/
    double tgtCourseT;   /*目标航向（真）    单位：°  最小值：0  最大值：360*/
    double tgtSpeed;   /*目标速度    单位：Km/h  最小值：0  最大值：300*/
    double TargetPositionAccuracy;   /*目标位置精度    单位：米  最小值：0  最大值：10000*/
    double TargetSpeedAccuracy;   /*目标航速精度    单位：米/秒  最小值：0  最大值：1000*/
    double TargetHDGAccuracy;   /*目标航向精度    单位：度  最小值：0  最大值：360*/
}TargetPoint_1;/*目标点 描述:应召反潜任务、跟踪阶段下有效*/


typedef struct
{
    double LonData;   /*圆心经度    单位：°*/
    double LatData;   /*圆心纬度    单位：°*/
    double Radius;   /*半径    单位：Km  最小值：0  最大值：150*/
}CircleRegion_1;/*圆形区域*/

typedef struct
{
    double LonData;   /*矩形区域中心点经度    单位：°*/
    double LatData;   /*矩形区域中心点纬度    单位：°*/
    double AngleData;   /*矩形区域纵向与正北方向夹角    单位：°  最小值：0  最大值：360*/
    double BroadWiseData;   /*矩形区域横向边长    单位：Km  最小值：0  最大值：150*/
    double DirectWiseData;   /*矩形区域纵向边长    单位：Km  最小值：0  最大值：150*/
}RectangularRegion_1;/*矩形区域*/


typedef struct
{
    unsigned int Null0:24;
    unsigned int AreaType:8;   /*区域类型*/
}AreaType;  /*区域类型*/

typedef struct
{
    AreaType AreaType_1;      /*区域类型*/
    RectangularRegion_1 RectangleZone;      /*矩形区域*/
    CircleRegion_1 CircleZone;      /*圆形区域*/
}TaskAreaInfo;/*任务区域信息*/


typedef struct
{
    TaskTypeStage TaskType_1;      /*任务类型及阶段*/
    TargetPoint_1 TargetPoint;      /*目标点  信号描述：应召反潜任务、跟踪阶段下有效*/
    TaskAreaInfo task_area_info;      /*任务区域信息*/
}AutoPlanInfo_1;/*定测点自动规划信息*/


typedef struct
{
    double StartLon;   /*起点经度    单位：°  最小值：-180  最大值：180*/
    double StartLat;   /*起点纬度    单位：°  最小值：-90  最大值：90*/
    double PreventDirection_1;   /*搜索方向    单位：°  最小值：0  最大值：360*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
    unsigned char ClockWise_1;   /*顺逆时针 0-逆时针，1-顺时针，界面默认逆时针*/
//    unsigned int Null0:24;
}RectExpanPlan;/*扩展方形 描述:
                                 */

typedef struct
{
    double LonData;   /*圆心经度    单位：°  最小值：-180  最大值：180*/
    double LatData;   /*圆心纬度    单位：°  最小值：-90  最大值：90*/
    double AxleDirection;   /*搜索方向    单位：°  最小值：0  最大值：360*/
    double SectorAngle_1_1;   /*扇区张角    单位：°  最小值：0  最大值：360*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
    double ExpantionStepLen;   /*外扩步长    单位：Km  最小值：0  最大值：50*/
    unsigned char ClockWise_1;   /*顺逆时针 0-逆时针，1-顺时针，界面默认逆时针
                                   以搜索方向为基准，第2个点在左边为逆时针，在右边为顺时针*/
//    unsigned int Null0:24;
}ZhiSectorPlan;/*之字扇形 描述:
                                 */

typedef struct
{
    double SpiralCentreLon;   /*起点经度    单位：°  最小值：-180  最大值：180*/
    double SpiralCentreLat;   /*起点纬度    单位：°  最小值：-90  最大值：90*/
    double SearchDirection;   /*搜索方向    单位：°  最小值：0  最大值：360*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
}HoldBlockPlan;/*拦阻搜索 描述:

                                 */

typedef struct
{
    double StartLon;   /*初始位置点经度    单位：°  最小值：-180  最大值：180*/
    double StartLat;   /*初始位置点纬度    单位：°  最小值：-90  最大值：90*/
    double SearchDirection;   /*搜索方向    单位：°  最小值：0  最大值：360*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
    unsigned char ClockWise_1;   /*顺逆时针 0-逆时针，1-顺时针，界面默认逆时针*/
//    unsigned int Null0:24;
}CircleExpanPlan;/*扩展圆形 描述:
                                   */

typedef struct
{
    double FirstLineCentreLon;   /*原点经度    单位：°  最小值：-180  最大值：180*/
    double FirstLineCentreLat;   /*原点纬度    单位：°  最小值：-90  最大值：90*/
    double PreventDirection_1;   /*搜索方向    单位：°  最小值：0  最大值：360*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
}CurvePlan;/*并排锯齿*/

typedef struct
{
    double LonData;   /*圆心经度    单位：°  最小值：-180  最大值：180*/
    double LatData;   /*圆心纬度    单位：°  最小值：-90  最大值：90*/
    double AxleDirection;   /*搜索方向    单位：°  最小值：0  最大值：360*/
    double SectorAngle_1;   /*扇区张角    单位：°  最小值：0  最大值：360*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
}SecterPlan;/*扇形 描述:
                              */

typedef struct
{
    double LonData;   /*原点经度    单位：°  最小值：-180  最大值：180*/
    double LatData;   /*原点纬度    单位：°  最小值：-90  最大值：90*/
    double AxleDirection;   /*搜索方向    单位：°  最小值：0  最大值：360*/
    double SpacingCoefficient;   /*间距系数    最小值：1  最大值：4*/
    unsigned short LineNum;   /*行数    单位：行  最小值：1  最大值：3*/
    unsigned short Null0;
}LinePlan;/*并排直线 描述:
                            */
typedef struct
{
    unsigned int Null0:24;
    unsigned int FigureType:8;   /*图形类型选择*/
}GraphicTypeSelect;  /*图形类型选择*/

typedef struct
{
    GraphicTypeSelect FigureType_1;      /*图形类型选择*/
    LinePlan line_plan;      /*并排直线  信号描述：
                                              */
    SecterPlan secter_plan;      /*扇形  信号描述：
                                                  */
    CurvePlan curve_plan;      /*并排锯齿*/
    CircleExpanPlan SpiralExpansion;      /*扩展圆形  信号描述：
                                                            */
    HoldBlockPlan HoldblockPlan;      /*拦阻搜索  信号描述：

                                                        */
    ZhiSectorPlan ZhiSectorPlanPlan;      /*之字扇形  信号描述：
                                                        */
    RectExpanPlan RectExpanPlansion;      /*扩展方形  信号描述：
                                                        */
}GraphicPlanInfo_1;/*定测点图形规划信息*/


typedef struct
{
    double DetectPointLon;   /*定测点经度    单位：°  最小值：-180  最大值：180*/
    double DetectPointLat;   /*定测点纬度    单位：°  最小值：-90  最大值：90*/
    double DetectPointDep;   /*定测点深度    单位：m*/
}DetectPointPositionInfo;/*定测点位置信息*/

typedef struct
{
    unsigned int Null0:24;
    unsigned int DetectPointNum:8;   /*定测点总数量    最小值：0  最大值：10*/
}DetectPointNum;  /*定测点总数量*/

typedef struct
{
    DetectPointNum DetectPointNum_1;      /*定测点总数量*/
    DetectPointPositionInfo DetectPointPosition[10];      /*定测点位置信息*/
}DetectCustomPlanInfo;/*定测点自定义规划信息*/

typedef struct
{
    unsigned int Plan_ID;   /*方案编号*/
    unsigned int SubTask_Id;   /*任务平台子任务ID号*/
    SonarPlanSimbol SonarPlanSimbol_1;      /*吊声定测点规划标识*/
    AutoPlanInfo_1 AutoPlanInformation;      /*定测点自动规划信息*/
    GraphicPlanInfo_1 FigurePlanInformation;      /*定测点图形规划信息*/
    DetectCustomPlanInfo CustomizePlanInformation;      /*定测点自定义规划信息*/
}BLK_CCC_OFP_403;/*吊声—吊声定测点规划 描述:实际发送长度=块标识+相应的吊声规划信息*/




/**************025*****************/

typedef struct
{
    unsigned int signal_next_ID;   /*当前阶段子任务ID*/
}NextSubtaskID;/*下一阶段子任务ＩD */

typedef struct
{
    unsigned int signal_now_ID;						/*当前阶段子任务ID*/
}CurrentSubtaskID; /*当前阶段子任务ＩD */

typedef struct
{
    char signal_CCC_now;				/*当前任务阶段*/
    unsigned char Null0;
    unsigned char signal_CCC_now_num;   /*当前阶段子任务数量*/
    CurrentSubtaskID signal_now[5]; /*当前阶段子任务ＩD */
    unsigned char signal_CCC_next_num;  /*下一阶段子任务数量    20250103备注错误？与总体待确认*/
    NextSubtaskID signal_next[5];      /*下一阶段子任务ＩD */
}BLK_CCC_OFP_025;/*当前任务阶段反馈*/



/**************020*****************/
typedef struct
{
    unsigned char failreason[200];   /*修改失败原因*/
}FailedReason;/*修改失败原因*/

typedef struct
{
    unsigned int Plan_ID;				/*方案编号*/
    unsigned char ModifyType;			/*修改类型  0=任务分配方案修改;1=航线信息修改;*/
    unsigned char modify_state;			/*修改状态  0=ＮＡ;1=修改中;2=修改成功;3=修改失败;*/
    FailedReason failure;  /*修改失败原因*/
}BLK_CCC_OFP_020;/*人工修改状态信息反馈*/




/**************005*****************/
typedef struct
{
    double Index_Lon;   /*顶点经度*/
    double Index_Lat;   /*顶点纬度*/
}SubdomainCoordinate;/*子区域坐标*/

typedef struct
{
    unsigned int Task_Are_ID;						/*子区域编号  1-1/ 1-2*/
    SubdomainCoordinate signal_FC00[4]; /*子区域坐标 4个点*/
#ifdef _SPARE_
    unsigned int SPARE0;
#endif
}SubdomainInfo;/*子区域情况*/

typedef struct
{
    unsigned int Task_Are_ID;							/*区域编号  1*/
    unsigned char task_are_hf_num;						/*当前任务区划分个数*/
    SubdomainInfo signal_FC00[6];     /*子区域情况*/
#ifdef _SPARE_
    unsigned char spare[71];
#endif
}TaskAreaSpecificDivisionInfo;/*任务区具体划分信息*/

typedef struct
{
    unsigned int Plan_ID;		/*方案编号*/
    unsigned char task_are;		/*划分任务区个数 对几个任务区划分*/
    TaskAreaSpecificDivisionInfo task_are_hf2[4];      /*任务区具体划分信息*/    
}BLK_CCC_OFP_005;/*任务区划分信息*/




/**************006*****************/
typedef struct
{
    unsigned short PlatformModel;   /*平台1型号  0=N/A;1=WZ2;*/
    unsigned short UAV_Serial_Number;   /*平台序号*/
    unsigned int UAV_ID;   /*平台编号*/
    unsigned int UAV_Task_ID;   /*无人机当前任务ID*/
}TaskEndingPlantformInfo;/*任务即将结束平台信息*/

typedef struct
{
    unsigned int plan_id;   /*方案编号*/
    unsigned char Null0;	/*任务即将结束平台数量*/
    TaskEndingPlantformInfo signal_FC00[5];      /*任务即将结束平台信息*/
}BLK_CCC_OFP_006;/*主动下一阶段提示*/



/**************007*****************/
typedef struct
{
    unsigned short target_num:8;   /*目标数量    单位：批  最大值：7*/
    unsigned short Null0:8;
}TargetNum;  /*目标数量*/

typedef struct
{
    unsigned short TimeValid:1;   /*时间有效位  1-有效/0-无效*/
    unsigned short TagPositonValid:1;   /*目标位置有效位  1-有效/0-无效*/
    unsigned short AzimuthValid:1;   /*目标方位有效  1-有效/0-无效*/
    unsigned short DistanceValid:1;   /*目标距离有效  1-有效/0-无效*/
    unsigned short SpeedValid:1;   /*目标速度有效  1-有效/0-无效*/
    unsigned short RectionValid:1;   /*目标航向有效  1-有效/0-无效*/
    unsigned short TagPositonAccuracyValid:1;   /*位置精度有效位  1-有效/0-无效*/
    unsigned short AzimuthAccuracyValid:1;   /*方位精度有效  1-有效/0-无效*/
    unsigned short DistanceAccuracyValid:1;   /*距离精度有效  1-有效/0-无效*/
    unsigned short SpeedAccuracyValid:1;   /*航速精度有效  1-有效/0-无效*/
    unsigned short RectionAccuracyValid:1;   /*航向精度有效  1-有效/0-无效*/
    unsigned short Null0:5;
}DataValid_1;  /*数据有效位*/

typedef struct
{
    int date:8;   /*日期*/
    int MONTH:8;   /*月份*/
    unsigned int YEAR:16;   /*年份*/
}TargetDiscoverDate;  /*目标发现日期*/


typedef struct
{
    DataValid_1 DATAValid_1_1;      /*数据有效位*/
    TargetDiscoverDate TARGET_DISCOVERY_DATE_1;      /*目标发现日期*/
    unsigned int TARGET_DISCOVERY_TIME;   /*目标发现时刻    单位：ms*/
    double TARGET_LONGITUDE;   /*目标位置经度    单位：°*/
    double TARGET_LATITUDE;   /*目标位置纬度    单位：°*/
    float TARGET_POSITION;   /*目标方位    单位：°  最小值：0  最大值：360*/
    unsigned int TARGET_DISTANCE;   /*目标距离    单位：米*/
    unsigned short TARGET_SPEED;   /*目标航速    单位：米/秒*/
    float TARGET_COURSE;   /*目标航向    单位：°  最小值：0  最大值：360*/
    unsigned short TARGET_ACCURACY;   /*目标位置精度    单位：m*/
    float TARGET_POSITION_ACCURACY;   /*目标方位精度    单位：°  最小值：0  最大值：360*/
    unsigned short TARGET_DISTANCE_ACCURACY;   /*目标距离精度    单位：米*/
    unsigned short TARGET_SPEED_ACCURACY;   /*目标航速精度    单位：米/秒*/
    float TARGET_COURSE_ACCURACY;   /*目标航向精度    单位：°  最小值：0  最大值：360*/
    unsigned short TARGET_CONFIDENCE;   /*目标置信度*/
}TargetParam;/*探测点迹参数 描述:点迹必须要求：位置或者方位距离必须有效。
                              数组下标：0～3，时间由近到远。*/

typedef struct
{
    unsigned short tgtSource:3;   /*目标源  0=N/A;1=吊声;2=浮标;3=综合;*/
    unsigned short TargetProperty:3;   /*空间属性  0=N/A;1=水下;2=水面;*/
    unsigned short TargetProperty_1:3;   /*目标属性  0=N/A;1=敌;2=我;3=友;4=中立;5=不明;*/
    unsigned short tgtPara:3;   /*目标类型  0=N/A;1=潜艇;2=水面舰艇;3=民船;4=其他;*/
    unsigned short Null0:4;
}TargetProperties;  /*目标特性*/

typedef struct
{
    unsigned char point_num:4;   /*有效点迹数量; 单位:个*/
    unsigned char CalculatePointValid:1;   /*声推算点有效状态  1-有效/0-无效*/
    unsigned char Null0:3;
}PointNum;  /*点迹数量*/

typedef struct
{
    unsigned short TimeValid:1;   /*时间有效位  1-有效/0-无效*/
    unsigned short TagPositonValid:1;   /*目标位置有效位  1-有效/0-无效*/
    unsigned short AzimuthValid:1;   /*目标方位有效  1-有效/0-无效*/
    unsigned short DistanceValid:1;   /*目标距离有效  1-有效/0-无效*/
    unsigned short SpeedValid:1;   /*目标速度有效  1-有效/0-无效*/
    unsigned short RectionValid:1;   /*目标航向有效  1-有效/0-无效*/
    unsigned short Null0:10;
}DataValid_1_1;  /*数据有效位*/

typedef struct
{
    int date:8;   /*日期*/
    int MONTH:8;   /*月份*/
    unsigned int YEAR:16;   /*年份*/
}TargetDiscoverDate_1;  /*目标发现日期*/


typedef struct
{
    DataValid_1_1 DATAValid_1_1_1;      /*数据有效位*/
    TargetDiscoverDate_1 TARGET_DISCOVERY_DATE_1_1;      /*目标发现日期*/
    unsigned int TARGET_DISCOVERY_TIME;   /*目标发现时刻    单位：ms*/
    double TARGET_LONGITUDE;   /*目标位置经度    单位：°*/
    double TARGET_LATITUDE;   /*目标位置纬度    单位：°*/
    float TARGET_POSITION;   /*目标方位    单位：°  最小值：0  最大值：360*/
    unsigned int TARGET_DISTANCE;   /*目标距离    单位：米*/
    unsigned short TARGET_SPEED;   /*目标航速    单位：米/秒*/
    float TARGET_COURSE;   /*目标航向    单位：°  最小值：0  最大值：360*/
}CalculatePointParam;/*声推算点迹参数*/

typedef struct
{
    TargetProperties tgtPara;      /*目标特性*/
    unsigned int SerNum;   /*目标批号    单位：无*/
    PointNum PointNum_1;      /*点迹数量*/
    TargetParam TargetPara[4];      /*探测点迹参数  信号描述：点迹必须要求：位置或者方位距离必须有效。
                                                     数组下标：0～3，时间由近到远。*/
    CalculatePointParam CalculatePoint;      /*声推算点迹参数*/
}TargetInfo;/*目标信息*/

typedef struct
{
    TargetNum target_num;      /*目标数量*/
    TargetInfo TgtInfo[7];      /*目标信息*/
}BLK_CCC_OFP_007;/*水下目标发现提示*/


// CCC-DPU1/DPU2/MMM-199 U端本机链路状态数据（MessageID：0xa222c7）
typedef struct{
    unsigned short NoheliID_1_3;//无人机ID/舰面端id
    unsigned char DownChannel_3;//频道选择（域描述0-50）
    unsigned char WorkMode_3;//工作模式（0-NA 1-工作，2-静默）
//    unsigned char DownWorkMode_3;//下行工作模式（0=NA 1-工作，2-静默）20241111与DPU模块协商
    unsigned char DownPower_3;//功率选择（0-NA 1-小功率，2-大功率）
    unsigned char UpRate_3;//上行传输速率（域描述；0-NA 1-25.6kbps，2-51.2kbps，3-102.4kbps）
    unsigned char DownRate_3;//下行传输速率（域描述：0-NA 1-2Mbps，2-4Mbps，3-8Mbps，4-16Mbps）
    unsigned char PathWay_3;//加密方式（域描述：0-NA 1-密文，2-明文）
    unsigned char IFNoHeliCon_2_2;//下行是否锁定（域描述：0-NA 1-锁定，2-失锁）
    unsigned char ConSignalStren_2_2;//有人机接收信号强度（域描述：0-5）
    unsigned char ChangeState;//交接状态（0-NA 1-交接中，2-交接成功，3-交接失败）
    unsigned char ChangeChannel;//频道选择（0-50）
}CR_drone_UlinkState;//无人机U链状态（循环次数4）

typedef struct{
    unsigned char UUavNumber;//U链连接无人机数量
    CR_drone_UlinkState ULparSet_1[4];//无人机U链状态（循环次数4）
    unsigned char platformNumber;//地面站数量
    CR_drone_UlinkState ULparSet_2[4];//地面站U链状态（循环次数4）
}BLK_CCC_OFP_199;//U端本机链路状态数据

/************************************OFP_CCC****************************************/

/*********************001*********************/
typedef struct
{
    unsigned short req1:8;	/*维护请求 0-NA 1-状态 2-显示*/ //byte1
    unsigned short req2:8;	/*维护故障清单翻页  0-NA 1-上一个故障清单 2-下一个故障清单 3-第一个故障清单*///byet0
}CMD_REQ;	/*当进入分系统或设备的维护主画面时，维护故障清单请求=1
              当进入子分系统或设备的维护故障清单画面时，维护故障清单请求=2， 维护故障清单翻页=3*/


typedef struct
{
    unsigned short req1:8;	/*维护自检 0-NA 1-开始 2-停止*/ //byte1
    unsigned short req2:8;	/*删除控制 0-NA 2-删除当前显示故障清单 2-删除全部故障清单*///byet0
}CMD_CMD;


typedef struct
{
    CMD_REQ  req;	/*维护清单请求和翻页 */
    CMD_CMD  cmd2;	/*维护自检控制和维护清单删除*/
}BLK_OFP_CCC_001; /*维护控制指令*/

/*********************005*********************/

typedef struct
{
    unsigned int Plan_ID;   /*方案编号*/
    unsigned char task_are;   /*划分任务区个数*/
    TaskAreaSpecificDivisionInfo task_are_hf2[4];      /*任务区具体划分信息*/
}BLK_OFP_CCC_005;/*任务区划分信息修改*/


/*********************027*********************/
typedef struct
{
    unsigned short Platform_Type;   /*平台1型号  0=N/A;1=20F;2=WZ2;*/
    unsigned short Platform_Serial_Number;   /*平台序号*/
    unsigned int Platform_Id;   /*平台编号*/
    unsigned short SubTask_Type;   /*子任务任务类型  0=N/A;1=浮标侦收;2=吊声定测;3=浮标布阵;4=通信中继;5=磁探搜索;6=磁探跟踪;7=光电搜索;8=光电跟踪;9=编队飞行;10=任务导航;11=返航;12=等待;13=临时改航;*/
    unsigned short SubTask_Point_Or_Area_type;   /*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
    unsigned int SubTask_Point_Or_Area_ID;   /*任务区/点/线/目标编号*/
}SubTask;/*各平台子任务*/

typedef struct
{
    unsigned char stage;   /*航线生成命令  0=NA;1=第一阶航线生成;2=第二阶航线生成;3=第三阶航线生成;4=第四阶航线生成;5=第五阶航线生成;6=第六阶航线生成;7=第七阶航线生成;8=第八阶航线生成;*/
    SubTask sub_task[5];      /*各平台子任务*/
}BLK_OFP_CCC_027;/*全局任务规划航线生成命令*/


/*********************038*********************/
typedef struct
{
    unsigned int Plan_ID;   /*子任务编号*/
}SubtaskAirlineID;/*生成航线子任务编号*/


typedef struct
{
    unsigned int Plan_ID;   /*方案编号*/
    unsigned char subtask_airline_number;   /*生成航线子任务数量*/
    unsigned char stage_id;      /*阶段号*/
    unsigned char fb_ds_tsak;   /*当前阶段是否包含浮标布防或吊声定测任务*/
    unsigned int Plan_ID_1;   /*浮标布防或吊声子任务编号*/
}BLK_OFP_CCC_038;/*航线生成命令*/




/*********************039*********************/
typedef struct
{
    unsigned int Plan_ID;   /*子任务编号*/
}SubtaskAirlineID_1;/*激活航线子任务编号*/


typedef struct
{
    unsigned int Plan_ID;   /*方案编号*/
    unsigned char subtask_airline_number;   /*生成航线子任务数量*/
    unsigned char stage_id;      /*阶段号*/
    unsigned char fb_ds_tsak;   /*当前阶段是否包含浮标布防或吊声定测任务,暂时为攻击判断 0-NA 1-单目标攻击*/
    unsigned int Plan_ID_1;   /*浮标布防或吊声子任务编号*/
    unsigned char routeType;   /* 0-NA 1-全局规划 2-单目标攻击 3-双机编队*/
}BLK_OFP_CCC_039;/*航线发布命令*/


/*********************041*********************/
typedef struct
{
    unsigned char ignal_FC00_DPU1;   /*反馈信息*/
}BLK_OFP_CCC_041;/*主动下一阶段提示反馈*/





/*********************042*********************/

typedef struct
{
    unsigned char signal_FC00_DPU1;   /*反馈状态*/
}BLK_OFP_CCC_042;/*水下目标发现提示反馈*/





/*********************302*********************/
typedef struct
{
    unsigned int Null0:16;
    unsigned int RouteTotalNum:8;   /*航路点总数    最大值：75*/
    unsigned int BuoyPlanPointNum:8;   /*浮标布阵点总数    单位：个  最大值：30*/
}StatusInfo;  /*状态信息*/

typedef struct
{
    double BuoyPlanPointLat;   /*浮标航路点经度    单位：°*/
    double BuoyPlanPointLon;   /*浮标航路点纬度    单位：°*/
    float Altitude;   /*高度    单位：m*/
}RoutePointInfo;/*航路点信息*/

typedef struct
{
    unsigned short BuoyType:8;   /*浮标类型  0=N/A;1=被动全向（L）;2=被动定向（D）;3=主动全向1（R）;4=主动全向2;5=主动全向3;6=主动全向4;7=主动定向1（T）;8=主动定向2;9=主动定向3;10=主动定向4;11=垂直线列阵（V）;12=低频声源（P）;13=扩展阵（X）;14=海噪声（A）;15=温深（B ）;*/
    unsigned short BuoyPlanPointPositon:8;   /*浮标布阵点匹配筒位号*/
}BuoyOtherParam;  /*浮标其他参数*/

typedef struct
{
    BuoyOtherParam BuoyOtherPara_1;      /*浮标其他参数*/
    double BuoyPlanPointLat;   /*浮标布阵点经度    单位：°*/
    double BuoyPlanPointLon;   /*浮标布阵点纬度    单位：°*/
}BuoyPosition;/*布阵点位置*/

typedef struct
{
    unsigned int Plan_ID;   /*方案编号*/
    unsigned int SubTask_Id;   /*任务平台子任务ID号*/
    unsigned char signal_FC00_DPU111;   /*是否修改*/
    StatusInfo status_info;      /*状态信息*/
    RoutePointInfo RoutePointData[75];      /*航路点信息*/
    unsigned int PreThrowTaskTime;   /*预期任务时间    单位：ms*/
    unsigned int PreTaskDistance;   /*预期任务里程    单位：m*/
    BuoyPosition buoy_position[25];      /*布阵点位置*/
}BLK_OFP_CCC_302;/*浮标—航路点解算 描述:只对本机航路点进行解算。*/



/*********************402*********************/
typedef struct
{
    unsigned int DetectPointSource:1;   /*定测点响应源  1-任务操作员主动推送（辅助规划）/0-响应指挥长*/
    unsigned int ProjectValid:1;   /*方案有效状态  1-有效/0-无效*/
    unsigned int Null0:22;
    unsigned int PlanWay:8;   /*规划方式  0=N/A;1=自动规划;2=图形规划;3=自定义;4=定测点概率圆推算;*/
}SonarPlanSimbol_1;  /*吊声定测点规划标识*/



typedef struct
{
    TaskTypeStage TaskType_1;      /*任务类型及阶段*/
    float GetPossibilatity;   /*方案搜索概率    最小值：0  最大值：1*/
    unsigned int DetectPointNum;   /*定测点总数量    最小值：0  最大值：10*/
    DetectPointPositionInfo DetectPointPosition[10];      /*定测点位置信息  信号描述：当为跟踪时，定测点只有1个。*/
}AutoPlanOut;/*定测点自动规划输出*/



typedef struct
{
    unsigned int FigureType;   /*图形类型选择*/
    float CalculateSuccessSimbol;   /*SPARE*/
    unsigned int DetectPointNumq;   /*定测点总数量    最小值：0  最大值：12*/
    DetectPointPositionInfo DetectPointPosition1[12];      /*载机1定测点位置信息*/
    unsigned int DetectPointNum2;   /*定测点总数量    最小值：0  最大值：12*/
    DetectPointPositionInfo DetectPointPosition2[12];      /*载机2定测点位置信息*/
    unsigned int DetectPointNum3;   /*定测点总数量    最小值：0  最大值：12*/
    DetectPointPositionInfo DetectPointPosition3[12];      /*载机3定测点位置信息*/
}GraphicPlanOut;/*定测点图形规划输出*/


typedef struct
{
    unsigned int DetectPointNum;   /*定测点总数量    最小值：0  最大值：10*/
    DetectPointPositionInfo DetectPointPosition[10];      /*定测点位置信息*/
}CustomPlanReply;/*定测点自定义规划回复*/


typedef struct
{
    unsigned int Plan_ID;   /*方案编号*/
    unsigned int SubTask_Id;   /*任务平台子任务ID号*/
    unsigned char signal_FC00_DPU111;   /*是否修改*/
    SonarPlanSimbol_1 sonar_plan_simbol_1;      /*吊声定测点规划标识*/
    AutoPlanOut AutoPlanInformation;      /*定测点自动规划输出*/
    GraphicPlanOut FigurePlanInformation;      /*定测点图形规划输出*/
    CustomPlanReply CustomizePlanInformation;      /*定测点自定义规划回复*/
    DetectPointPositionInfo CalcDetectPointPosition[2];      /*定测点位置推算信息*/
}BLK_OFP_CCC_402;/*吊声—吊声定测点规划*/

/**********************************************/
// BLK_CCC_OFP_302   /*浮标—浮标布阵规划	*/
// BLK_CCC_OFP_403   /*吊声—吊声定测点规划	*/
// BLK_CCC_OFP_025   /*当前任务阶段反馈*/
// BLK_CCC_OFP_020   /*人工修改状态信息反馈*/
// BLK_CCC_OFP_005   /*任务区划分信息  （方案生成后收到）*/
// BLK_CCC_OFP_006   /*主动下一阶段提示*/
// BLK_CCC_OFP_007   /*水下目标发现提示*/

// BLK_OFP_CCC_005   /*任务区划分信息修改*/
// BLK_OFP_CCC_027   /*全局任务规划航线生成命令*/
// BLK_OFP_CCC_036   /*无人机载荷控制信息*/
// BLK_OFP_CCC_038   /*航线生成命令*/
// BLK_OFP_CCC_039   /*航线发布命令*/
// BLK_OFP_CCC_041   /*主动下一阶段提示反馈*/
// BLK_OFP_CCC_042   /*水下目标发现提示反馈*/
// BLK_OFP_CCC_302   /*浮标—航路点解算*/
// BLK_OFP_CCC_402   /*吊声—吊声定测点规划*/
/**********************************************/

typedef struct
{
    unsigned int integer:8;             //发射数量
    unsigned int F_NOTE:8;              //当前武器类型
    unsigned int missile:8;             //导弹方式 0H=N/A;1H=单发;2H=快发;3H=连发
    unsigned int exposure_end:1;        //空地导弹无人机协同照射发射结束 0-否;1-是
    unsigned int danger_area_valid:1;   //危险区有效 0-无效;1-有效
    unsigned int delay_time_valid:1;    //延迟时间有效(它机) 0-无效;1-有效
    unsigned int launch_valid:1;        //空地导弹允许发射(它机) 0-无效;1-有效
    unsigned int sapre1:4;
    unsigned int target_1:1;            //目标1有效(它机) 0-无效;1-有效
    unsigned int target_2:1;            //目标2有效(它机) 0-无效;1-有效
    unsigned int target_3:1;            //目标3有效(它机) 0-无效;1-有效
    unsigned int target_4:1;            //目标4有效(它机) 0-无效;1-有效
    unsigned int sapre2:4;
    unsigned int plane_1:1;             //照射机1在攻击区 0-不在;1-在
    unsigned int plane_2:1;             //照射机2在攻击区 0-不在;1-在
    unsigned int plane_3:1;             //照射机3在攻击区 0-不在;1-在
    unsigned int plane_4:1;             //照射机4在攻击区 0-不在;1-在
    unsigned int sapre3:12;
}EXPOSURE_DATA;
typedef struct
{
    double latitude;//纬度
    double longitude;//经度
}EXPOSURE_AREA;
typedef struct
{
    unsigned long id;   //照射机编号
    double angle;       //目标方位角度
    double latitude;    //目标纬度
    double longitude;   //目标经度
}EXPOSURE_PLANE;
typedef struct
{
    unsigned int HKWGXT:1;  //火控外挂系统 0-不满足;1-满足
    unsigned int WQZW:1;    //武器占位 0-不满足;1-满足
    unsigned int LZXH:1;    //轮载信号 0-不满足;1-满足
    unsigned int WQZDY:1;   //武器总电源 0-不满足;1-满足
    unsigned int SDKG:1;    //随动开关 0-不满足;1-满足
    unsigned int WGLTX:1;    //外挂梁通信 0-不满足;1-满足
    unsigned int WGLXT:1;   //外挂梁系统0-不满足;1-满足
    unsigned int GJJX:1;    //挂架极限 0-不满足;1-满足
    unsigned int GJCC:1;    //挂架超差 0-不满足;1-满足
    unsigned int spare1:1;
    unsigned int WQXT:1;    //武器系统 0-不满足;1-满足
    unsigned int WGWSB:1;   //外挂物识别 0-不满足;1-满足
    unsigned int JGBM:1;    //激光编码 0-不满足;1-满足
    unsigned int JDQYGD:1;  //绝对气压高度 0-不满足;1-满足
    unsigned int ZSYSYD:2;  //照射延时应答  0H=N/A;1H=有应答;2H=无应答
    unsigned int WQXTTX:1;  //武器系统通信 0-不满足;1-满足
    unsigned int MBYX:1;    //目标有效 0-no;1-yes
    unsigned int ZSAQQ:1;   //照射安全区 0-不满足;1-满足
    unsigned int JGZBH:1;   //激光准备好 0-不满足;1-满足
    unsigned int ZJHGJ:1;   //载机横滚角 0-不满足;1-满足
    unsigned int MBFW:1;    //目标方位角 0-不满足;1-满足
    unsigned int MBJL:1;    //目标距离 0-不满足;1-满足
    unsigned int FSJG:1;    //发射间隔 0-不满足;1-满足
    unsigned int HXFS:1;    //横向风速 0-不满足;1-满足
    unsigned int ZSJAQ:1;   //照射机安全 0-不满足;1-满足
    unsigned int ZSJTX:1;   //照射机通信 0-不满足;1-满足
    unsigned int spare2:5;
}FIRE_COND_OTHER;

typedef struct
{
    EXPOSURE_DATA exposure_data;            //照射参数
    unsigned int batch_num;                 //当前攻击批号
    unsigned short target_valid;            //当前攻击目标有效 0-无效;1-有效
    double angle_range;                     //允许发射目标角度范围
    EXPOSURE_AREA area_left_position[6];    //左侧照射区域经纬度信息
    EXPOSURE_AREA area_right_position[6];   //右侧照射区域经纬度信息
    EXPOSURE_AREA area_position[7];         //危险区域经纬度信息
    double delay_time;                      //照射延迟时间
    EXPOSURE_PLANE exposure_plane[8];       //照射机编号和目标信息
    FIRE_COND_OTHER fire_cond_other;        //本攻它照火控发射条件
    double cur_tar_lat;                     //当前攻击目标纬度
    double cur_tar_lon;                     //当前攻击目标经度
    double cur_tar_hgh;                     //当前攻击目标高度
}BLK_DPU_CCC_011;//无人机协同照射攻击信息

typedef struct
{
    unsigned int launch_suspend:1;     //空地导弹中止发射 0-否;1-是
    unsigned int spare1:7;
    unsigned int plane1_valid:1;        //照射机1信息有效 0-无效;1-有效
    unsigned int plane2_valid:1;        //照射机2信息有效 0-无效;1-有效
    unsigned int plane3_valid:1;        //照射机3信息有效 0-无效;1-有效
    unsigned int plane4_valid:1;        //照射机4信息有效 0-无效;1-有效
    unsigned int spare2:20;
}PAR_OF_OPERATION;
typedef struct
{
    unsigned int data_status:1;     //数据状态 0-失败;1-成功
    unsigned int irr:1;             //能否照射 0-否;1-是
    unsigned int delay_time:1;      //是否收到延迟时间 0-否;1-是
    unsigned int safe_area:1;       //是否在安全区 0-否;1-是
    unsigned int tar_valid:1;       //目标有效 0-无效;1-有效
    unsigned int spare:27;
}IRR_STATUS;
typedef struct
{
    unsigned long number;           //照射机编号
    IRR_STATUS irr_status;          //照射机状态
    unsigned char tar_num;          //目标数量
    unsigned long irr_laser_num;    //激光编码
    double lat;                     //照射机纬度
    double lon;                     //照射机经度
    double hgh;                     //照射机气压高度
    double tar_lat;                 //目标纬度
    double tar_lon;                 //目标经度
    double tar_hgh;                 //目标气压高度
    double tar_speed;               //目标速度
    double tar_hx;                  //目标真航向
}IRR_PLANE;
typedef struct
{
    PAR_OF_OPERATION par_of_operation;  //本攻它照方式控制
    unsigned char irr_heli_number;      //照射机数量
    IRR_PLANE irr_plane[4];             //照射机信息
}BLK_CCC_DPU_020;//无人机协同照射控制

typedef struct
{
    double lon;//经度
    double lat;//维度
}Point;



// 新增给ofp的信息 20250226
/**************127*****************/
typedef struct
{
    unsigned short Null0:11;
    unsigned short HandoverRateValid:1;   /*交接下行速率是否有效  0=无效;1=有效;*/
    unsigned short HandoverDownChannelValid:1;   /*交接下行频道是否有效  0=无效;1=有效;*/
    unsigned short HandoverUpChannelValid:1;   /*交接上行频道是否有效  0=无效;1=有效;*/
    unsigned short UchannelValid:1;   /*U链频道号是否有效  0=无效;1=有效;*/
    unsigned short UavIdValid:1;   /*接收平台编号是否有效  0=无效;1=有效;*/
}DATA_Valid;  /*数据有效位*/

typedef struct
{
    //unsigned short data_valid; /*数据有效位（说明：0无效 1有效） bit0：接收平台编号 bit1:U链频道号 bit2：交接上行频道 bit3：交接下行频道 bit4：交接下行速率 */
    DATA_Valid valid;
    unsigned int uav_code;					/*平台编号*/
    unsigned short u_channel;				/*U链频道号*/
    unsigned short handover_up_channel;		/*交接上行频道*/
    unsigned short handover_down_channel;	/*交接下行频道*/
    unsigned short handover_rate;			/*交接下行速率*/
}BLK_CCC_OFP_127; /*接机参数反馈  20250103新增*/


/**************139*****************/

typedef struct
{
    //unsigned short data_valid; /*数据有效位（说明：0无效 1有效） bit0：接收平台编号 bit1:U链频道号 bit2：交接上行频道 bit3：交接下行频道 bit4：交接下行速率 */
    DATA_Valid valid;
    unsigned int uav_code;					/*平台编号*/
    unsigned int rcv_platform_code;			/*接收平台编号*/
    unsigned short u_channel;				/*交接u链频道号*/
    unsigned short handover_up_channel;		/*交接上行频道*/
    unsigned short handover_down_channel;	/*交接下行频道*/
    unsigned short handover_rate;			/*交接下行速率*/
}BLK_CCC_OFP_139; /*交机参数反馈*/


/**************198*****************/
typedef struct
{
    unsigned int CniFrontFault:1;   /*射频前端状态  1-是/0-否*/
    unsigned int Null0:31;
}PFL_ATT_LENVEL;  /*飞行故障清单注意级*/

typedef struct
{
    unsigned int pfl_warn_level;
    //unsigned int pfl_att_lenvel;	/*飞行故障清单注意级 (0否，1是) bit31：射频前端状态*/
    PFL_ATT_LENVEL pfl_att_lenvel;	/*飞行故障清单注意级*/
    unsigned int pfl_tip_level;
}BLK_CCC_OFP_198; /*U端机飞行故障清单*/


/**************200*****************/
// CCC-DPU1/DPU2/MMM-200 U端机无人机链路状态数据（MessageID：0xa222c8）
typedef struct{
    unsigned short NoheliID_1_3_1;//无人机ID
    unsigned short NoheliID_1_4;//无人机控制站ID
    unsigned char DownChannel_3_1;//频道选择（0-50）
    unsigned char WorkMode_3_1;//工作模式（0-NA 1-工作，2-静默）
    //unsigned char DownWorkMode_3_1;//下行工作模式（0=NA 1-工作，2-静默）20241111与DPU模块协商
    unsigned char DownPower_3_1;//功率选择（0-NA 1-小功率，2-大功率）
    unsigned char UpRate_3_1;//上行传输速率（域描述；0-NA 1-25.6kbps，2-51.2kbps，3-102.4kbps）
    unsigned char DownRate_3_1;//下行传输速率（域描述：0-NA 1-2Mbps，2-4Mbps，3-8Mbps，4-16Mbps）
    unsigned char PathWay_3_1;//加密方式（域描述：0-NA 1-密文，2-明文）
    unsigned char IFNoHeliCon_2_2_1;//上行是否锁定（域描述：0-NA 1-锁定，2-失锁）
    unsigned char ConSignalStren_2_2_1;//无人机接收信号强度（域描述：0-5）
    unsigned char ChangeState_1;//交接状态（0-NA 1-交接中，2-交接成功，3-交接失败）
} CR_drone_UlinkState_1;//无人机U链状态1（循环次数4）

typedef struct{
    unsigned short NoheliID_1_3_1_1;//无人机ID
    unsigned short NoheliID_1_4_1;//无人机控制站ID
    unsigned char DownChannel_3_1_1;//频道选择（0-50）
    unsigned char WorkMode_3_1_1;//工作模式（0-NA 1-工作，2-静默）
    //unsigned char DownWorkMode_3_1_1;//下行工作模式（0=NA 1-工作，2-静默）20241111与DPU模块协商
    unsigned char DownPower_3_1_1;//功率选择（0-NA 1-小功率，2-大功率）
    unsigned char UpRate_3_1_1;//上行传输速率（域描述；0-NA 1-25.6kbps，2-51.2kbps，3-102.4kbps）
    unsigned char DownRate_3_1_1;//下行传输速率（域描述：0-NA 1-2Mbps，2-4Mbps，3-8Mbps，4-16Mbps）
    unsigned char PathWay_3_1_1;//加密方式（域描述：0-NA 1-密文，2-明文）
    unsigned char IFNoHeliCon_2_2_1_1;//上行是否锁定（域描述：0-NA 1-锁定，2-失锁）
    unsigned char ConSignalStren_2_2_1_1;//无人机接收信号强度（域描述：0-5）
    unsigned char ChangeState_1_1;//交接状态（0-NA 1-交接中，2-交接成功，3-交接失败）
} CR_drone_UlinkState_2;//无人机U链状态2（循环次数4）

typedef struct{
     unsigned char UUavNumber_1;//U链连接无人机数量
     CR_drone_UlinkState_1 ULParSet_1[4];//无人机U链状态1（循环次数4）
     unsigned char UUavNumber_1_1;//U链连接地面站数量
     CR_drone_UlinkState_2 ULParSet_2[4];//无人机U链状态2（循环次数4） //TODO待定 无人机U链地面站状态？
}BLK_CCC_OPF_200;//U端机无人机链路状态数据




/**************201/202*****************/
typedef struct
{
      unsigned short MFL_STATUS:1;   /*bit15 维护故障清单存在  1-存在/0-不存在*/
      unsigned short Null0:15;
}SIG_CCC_DPU1_MFL_STATUS_CCC;  /*维护故障清单状态  (大端方式)*/

typedef struct
{
      unsigned short SoftValid:1;   /*软件有效位状态  1-有效/0-无效*/
      unsigned short Null0:15;
}SIG_CCC_DPU1_VALIDSTAUTS_1_1;  /*软件有效状态*/

typedef struct
{
      unsigned long long New4Sign:8;   /*新4段标识  0xff为软件版本号新4段标识，非0xff未原2段式软件版本*/
      unsigned long long Paragraph1:8;   /*第1段*/
      unsigned long long Paragraph2:8;   /*第2段*/
      unsigned long long PointFront_Paragraph3:8;   /*小数点前/第3段*/
      unsigned long long PointBack_Paragraph4:8;   /*小数点后/第4段*/
//      unsigned long long null:24;
}SIG_CCC_DPU1_SOFTWARE_VERSION_1_1;  /*软件版本*/

typedef struct
{
      unsigned int Day:8;   /*日期*/
      unsigned int Month:8;   /*月份*/
      unsigned int Year:16;   /*年份*/
}SIG_CCC_DPU1_SOFTWARE_VERSION_DATE_1_1;  /*软件版本日期*/

typedef struct
{
      SIG_CCC_DPU1_VALIDSTAUTS_1_1 ValidStauts_1_1;      /*软件有效状态*/
      unsigned char SoftWareName[37];   /*软件名称(对应4段显示)  如实际内容小于40个字符（中文计数2个，英文和数字计数1个）*/
      SIG_CCC_DPU1_SOFTWARE_VERSION_1_1 SOFTWARE_VERSION_1_1;      /*软件版本*/
      SIG_CCC_DPU1_SOFTWARE_VERSION_DATE_1_1 SOFTWARE_VERSION_DATE_1_1;      /*软件版本日期*/
}SIG_CCC_DPU1_SOFTCONFIG;/*软件版本配置*/


/**************202*****************/
typedef struct
{
      SIG_CCC_DPU1_MFL_STATUS_CCC MFL_STATUS_KDL;   /*维护故障清单状态*/
      unsigned short SOFTWARE_QUANTITY;   			/*软件配置项数量*/
      SIG_CCC_DPU1_SOFTCONFIG SoftConfig[20];      	/*软件版本配置*/
}BLK_CCC_OFP_202;/*维护消息   协同指控计算机 */


/**************201*****************/
typedef struct
{
      SIG_CCC_DPU1_MFL_STATUS_CCC MFL_STATUS_KDL;   /*维护故障清单状态*/
      unsigned short SOFTWARE_QUANTITY;   			/*软件配置项数量*/
      SIG_CCC_DPU1_SOFTCONFIG SoftConfig[8];      	/*软件版本配置*/
}BLK_CCC_OFP_201;/*维护消息  U链 */

/**************021/043*****************/
typedef struct //任务序列信息
{
    unsigned int id;					/*任务平台子任务ID号*/
    unsigned short type;				/*子任务任务类型  0=N/A;1=浮标侦收;2=吊声定测;3=浮标布阵;4=通信中继;5=磁探搜索;6=磁探跟踪;7=光电搜索;8=光电跟踪;9=编队飞行;10=任务导航;11=返航;12=等待;13=临时改航;*/
    unsigned short point_or_area;		/*子任务任务点/区域类型  0=N/A;1=任务点;2=线;3=区域;*/
    unsigned int point_or_area_id;		/*任务区/点/线/目标编号*/
    unsigned char finish_time_valid;	/*子任务完成时间有效位*/
    unsigned char finish_height_valid;  /*子任务任务高度有效位*/
    unsigned int finish_time;			/*子任务完成时间*/
    unsigned int finish_height;			/*子任务完成高度*/
}SubTasks_Info;
typedef struct //编队任务协同方案
{
    unsigned short platform_model;   /*平台1型号  0=N/A;1=20F;2=WZ2;*/
    unsigned short platform_serial_num;   /*平台序号*/
    unsigned int platform_code;   /*平台编号*/
    unsigned int platform_task_time;   /*任务平台任务时长*/
    unsigned short platform_subtask_number;   /*任务平台子任务个数m*/
    SubTasks_Info task_sequence_informations[8];//任务序列信息（循环8次）
}Plan_Info;
typedef struct
{
    unsigned int plan_id;//方案编号
    unsigned int subtask_id;//任务子平台ID号
}Task_Info;
typedef struct //受影响航线平台信息
{
    unsigned short platform_serial_num;//平台序号
    unsigned int platform_id;//平台编号
    unsigned short affected_task_num;//受影响子任务数量
    Task_Info task_info[8];//子任务序号
}Affected_Info;
typedef struct
{
    unsigned int plan_id;//方案编号
    unsigned short plan_state;//任务分配方案发布
    unsigned short plan_manual;//任务分配方案是否人工修改
    unsigned short platform_serial_num;//单无人机指控平台序号
    unsigned int platform_id;//单无人机指控平台编号
    unsigned char signal_FC00;//单任务规划无人机是否处于全局任务规划当中
    unsigned short platform_num;//当前单无人机指控平台个数
    Plan_Info plan_info[4];//单无人机指控规划
    unsigned short affected_num;//受影响航线平台个数
    Affected_Info affected_info[4];//受影响航线平台信息
}BLK_CCC_OFP_021,BLK_OFP_CCC_043;//单无人机规划任务分配结果||无人机指控任务分配结果修改及发布


/************** 043 *****************/
typedef struct
{
	unsigned short voiceRecognizeSta;	/*识别状态  0-NA 1-识别中  2-识别成功  3-识别失败*/
	unsigned short needReply;			/*是否需要应答  0-不需要应答  1-需要应答   0923*/
	unsigned short fromWho;			    /*触发识别指令的席位  0-主驾驶  1-副驾驶  2-PAD前  3-PAD后*/
	unsigned short executeSta;			/*指令的执行确认状态  0-NA  1-指令已确认执行  2-指令取消执行 3-任务执行失败  0923*/
	unsigned short cmdType;				/*指令类型  0-NA
	                                      1-页面调显（直接执行）
	                                      2-主模式切换（需确认）
	                                      3-页面切换（直接执行）
	                                      4-设备控制（上下电）(需确认)
	                                      5-参数设置（是否需要确认见具体指令）
	                                      6-协同任务页面显示控制（直接执行）
	                                      7-编队任务规划（预留）
	                                      8-单机任务规划/重规划（需确认）
	                                      9-控制权/编队控制（需确认）*/

	unsigned short mfdPage;				/*页面调显：页面类型
	                                     0-na
	                                     1-垂直
	                                     2-综合
	                                     3-机电
	                                     4-历程
	                                     5-辅助决策
	                                     6-雷达   7-地图
	                                     8-三维地图  9-水平
	                                     10-机外监控  11-飞行计划
	                                     12-航路点 13-航线参数
	                                     14-态势 15-数据
	                                     16-激光 17-光纤
	                                     18-通信 19-自卫
	                                     20-数据加载 21-HUMS 22-三轴  23-夜市  24-自检测
	                                     25-稳瞄 26-维护 27-协同任务 28-编队管理 29-指令 30-态势列表 31-火控 32-编队计划 33-故障清单 34-主菜单页面
	                                     35-返回上一层级页面 36-飞控 37-电侦 38-北斗 39-数据 40-任务重构 */

	unsigned short mfdNo;				/*页面调显：mfd序号 0-na,1-mfd1，2-mfd2, 3-mfd3, 4-mfd4 */
	unsigned short mainMode;			/*主模式切换:0-na,1-反潜模式，2-反舰模式，3-na,4-导航主模式 */

	unsigned short pageSwitch;			/*页面切换：页面切换类型 0-na,1-本机稳瞄，2-本机攻击（预留不使用），3-本机火控（预留不使用），
	                                               4-无人机稳瞄，5-无人机攻击（预留不使用），6-无人机火控（预留不使用），*/
	unsigned short pageSwitchUavSn;		/*页面切换：编队内无人机编号*/

	unsigned short deviceCtrl;			/*设备控制：
	                                     0-na
	                                     1-本机激光上电
	                                     2-本机稳瞄上电（预留）
	                                     3-本机雷达上电（预留）
	                                     4-本机红外上电
	                                     5-无人机Y激光上电
	                                     6-无人机Y稳瞄上电（预留）
	                                     7-无人机Y雷达上电（预留）
	                                     8-无人机Y红外上电
	                                     9-本机激光下电
	                                     10-本机红外下电
	                                     11-无人机Y激光下电
	                                     12-无人机Y红外下电
	                                     13-无人机Y磁探探杆伸出
	                                     14-无人机Y磁探探杆缩回*/
	unsigned short devCtrlUavSn;		/*设备控制：编队内无人机编号*/

	unsigned short ctrlTyp;				/*参数设置：
	                                    0-na
	                                    1-26预留
	                                    27-无人机Y激光测距开始（需确认）
	                                    28-无人机Y激光测距停止（需确认）
	                                    29-无人机Y一键调焦
	                                    30-无人机Y红外
	                                    31-无人机Y电视
	                                    32-无人机Y增大亮度
	                                    33-无人机Y减小亮度
	                                    34-无人机Y增大对比度
	                                    35-无人机Y减小对比度
	                                    36-无人机Y修改激光编码X（需确认）
	                                    37-无人机Y切换武器X（X为武器类型）（需确认）
	                                    38-无人机Y切换占位挂点X（需确认）
	                                    39-无人机Y光电手动
	                                    40-无人机Y光电收藏
	                                    41-无人机Y光电扫描
	                                    42-无人机Y光电锁定
	                                    43-无人机Y光电跟踪
	                                    44-无人机Y光电地理跟踪
	                                    45-无人机Y跟踪目标X（预留）
	                                    46-无人机Y光电黑热
	                                    47-无人机Y光电白热
	                                    48-无人机Y光电视场+
	                                    49-无人机Y光电视场-
	                                    50-无人机Y光电大视场
	                                    51-无人机Y光电中视场
	                                    52-无人机Y光电小视场
	                                    53-无人机Y光电防拥开
	                                    54-无人机Y光电防拥关
	                                    55-无人机光电目标+(预留)
	                                    56-无人机光电目标-(预留)
	                                    57-无人机Y光电自主(预留)
										58-无人机Y光电人工(预留)
	                                    59-本机稳瞄激光测距开始（需确认）
	                                    60-本机稳瞄激光测距停止（需确认）
	                                    61-本机稳瞄一键调焦
	                                    62-本机稳瞄红外
	                                    63-本机稳瞄电视
	                                    64-本机修改激光编码Y（需确认）
	                                    65-本机光电切换手动模式（预留）
	                                    66-本机光电切换收藏模式（预留）
	                                    67-本机光电切换扫描模式（预留）
	                                    68-本机光电切换锁定模式（预留）
	                                    69-本机光电切换跟踪模式（预留）
	                                    70-本机光电切换地理跟踪模式（预留）
	                                    71-本机跟踪目标Y（预留）
	                                    72-本机光电黑热
	                                    73-本机光电白热
	                                    74-本机光电视场+（预留）
	                                    75-本机光电视场-（预留）
	                                    76-本机光电大视场
	                                    77-本机光电中视场
	                                    78-本机光电小视场
	                                    79-本机光电防拥开（预留）
	                                    80-本机光电防拥关（预留）
	                                    81-本机光电目标+（预留）
	                                    82-本机光电目标-（预留）
	                                */
	unsigned short ctrlUavSn;			/*参数设置：编队内无人机编号*/
	unsigned short ctrlWeaponType;		/*参数设置：武器类型（预留）
	                                      1-AKF9A
	                                      2-AKF9B
	                                      3-AKF9C
	                                      4-AKF9D
	                                      5-AKF10A
	                                      6-AKF10B
	                                      7-AKF10C
	                                      8-AKF10D
	                                      9-TL2
	                                      10-FT8*/
	unsigned short ctrlWeaponParam1;	/*参数设置：武器占位（预留）1-9*/
	unsigned short ctrlLaserCode;		/*参数设置：激光编码 （预留）LSB=1*/
	unsigned short ctrlTgtCode;			/*参数设置：目标编码*/

	unsigned short xtPageType;			/*协同页面显示控制： 类型
	                                     0-na
	                                     1-以有人机为中心
	                                     2-以无人机Y为中心
	                                     3-以光标为中心
	                                     4-显示任务进度（预留）
	                                     5-显示本机载荷（预留）
	                                     6-显示无人机载荷（预留）*/
	unsigned short xtPageUavSn;			/*协同页面显示控制： 编队内无人机编号*/

	unsigned short singleUavType;		/*单机任务规划
	                                    0-na
	                                    1-无人机X搜索任务区Y（预留）
	                                    2-无人机X跟踪目标Y（预留）
	                                    3-无人机X打击目标Y（预留）
	                                    4-无人机X导航至航路点Y（预留）
	                                    5-无人机X运输至着陆区Y（预留）
	                                    6-编队飞行（预留）
	                                    7-无人机X盘旋时间Y分钟（预留）
	                                    8-无人机X盘旋圈数Y（预留）
	                                    9-无人机X悬停等待Y分钟（预留）
	                                    10-无人机X盘旋等待Y分钟（预留）
	                                    11-无人机X照射引导Y目标（预留）
	                                    12-无人机X浮标侦听任务区Y
	                                    13-无人机X磁探搜索任务区Y
	                                    14-无人机X磁探跟踪目标Y
	                                    15-无人机X打击规避目标Y
	                                    */
	unsigned short singleUavSn;			/*单机任务规划:编队内无人机编号*/
	unsigned int singleUavAreaId;		/*单机规划:任务区编号*/
	unsigned int singleUavTgtId;		/*单机规划:目标编号*/
	unsigned short singleUavHldId;		/*单机规划:航路点编号 LSB=1*/
	unsigned short singleUavTime;		/*单机规划:时间 LSB=1分钟*/

	unsigned short authorCtrl;			/*控制权/编队控制
	                                     0-na
	                                     1-申请无人机Y三级控制权（预留）
	                                     2-申请无人机Y四级控制权
	                                     3-申请无人机Y五级控制权（预留）
	                                     4-释放无人机Y三级控制权（预留）
	                                     5-释放无人机Y四级控制权（预留）
	                                     6-释放无人机Y五级控制权（预留）
	                                     7-无人机X加入编队（预留） authorCtrlUavId
	                                     8-无人机Y退出编队（预留） authorCtrlUavId
	                                     9-同意无人机Y地面站申请
	                                     10-拒绝无人机Y地面站申请
	                                     11-接收无人机Y控制权
	                                     */
	unsigned short authorCtrlUavSn;		/*控制权/编队控制:编队内无人机编号*/
	unsigned short authorCtrlUavId;		/*控制权/编队控制:编队内无人机id(用于加入编队控制)*/

	char recognizeMsg[100];				/*语音识别文字*/

}BLK_CCC_OFP_043;  //语音识别状态反馈  0910 暂定


/************** 347 *****************/
typedef struct
{
	unsigned char tips_type;		/*重规划提示类型   0-NA 1-航线冲突,执行重规划?  2-冲突消解,继续执行任务  3-发布失败,重新发布*/
}BLK_CCC_OFP_347;  /*航线生成时，如果有航线冲突，提示给综显*/

/************** 157 *****************/
typedef struct
{
	unsigned short uavSn;		/*无人机序号*/
	unsigned int uavCode;		/*无人机平台编号*/
	unsigned char status;		/*0-查询中 1-成功 2-失败*/
}BLK_CCC_OFP_157;//查询回报 20251218new 0xa2229d

/************** 158 *****************/
typedef struct
{
	unsigned short uavSn;									/*无人机序号*/
	unsigned int uavCode;									/*无人机平台编号*/
	unsigned short num;										/*航点个数*/
	planning_information_waypoint_information point[25];	/*航点信息*/
}BLK_CCC_OFP_158;//查询航线 20251218new 0xa2229e

/************** 159 *****************/
typedef struct
{
	unsigned short uavSn;									/*无人机序号*/
	unsigned int uavCode;									/*无人机平台编号*/
	unsigned short avoid_num;								/*规避点号，与ofp之间约定的航点序号*/
	unsigned short avoid_time;								/*规避时间，单位:分钟*/
}Avoid_Info;//规避信息
typedef struct
{
	unsigned short uav_num;									/*规避无人机数量*/
	Avoid_Info avoid_info[2];
}BLK_CCC_OFP_159;//规避提示接口 20251220new 0xa2229f

#pragma pack()

#endif // COOPERATIVEALLEGATIONCOMPUTERSYNTHESIS_H




