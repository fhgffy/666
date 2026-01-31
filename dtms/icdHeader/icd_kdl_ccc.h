#ifndef ICD_KDL_CCC_H
#define ICD_KDL_CCC_H
#pragma pack(1)

#define WORD unsigned short
#define BYTE char
#define DWORD int


/***************************** kdl接收 **********************************/

union FCAE_BUOY_DATA
{
    unsigned long long Var;  ///<64位
    struct
    {
        unsigned long long status:8;///<来源
        unsigned long long type:4;///<
        unsigned long long fc:3;///<属性
        unsigned long long deep:3;///<类型
        unsigned long long life:9;///<保留
        unsigned long long rfChnl:7;///<保留
        unsigned long long vhfVal:8;///<保留
        unsigned long long spare1:14;///<保留
        unsigned long long src:1;///<保留
        unsigned long long spare2:7;///<保留
    }data;
    /*
    8	56~63	Enum	对应浮标状态(Enum)( 0 H=未知;1 H=失效;2 H=存活未接触;3 H=存活已接触 )	8
    7	52~55	Enum	浮标类型(Enum)( 0 H=未装填;1 H=有浮筒无浮标;2 H=被动全向浮标;3 H=被动定向浮标;4 H=主动全向浮标;5 H=主动定向浮标;6 H=垂直线列阵浮标;7 H=低频声源浮标;8 H=扩展阵浮标;9 H=海噪声浮标;A H=温深浮标 )	4
    7	49~51	Enum	浮标频率(Enum)( 0 H=0无效;1 H=1425Hz;2 H=4600Hz;3 H=5200Hz;4 H=5800Hz;5 H=6400Hz )	3
    6	46~48	Enum	浮标深度(Enum)( 0 H=无效;1 H=15米;2 H=40米;3 H=60米;4 H=80米;5 H=150米;6 H=300米 )	3
    5	37~45	FixData 浮标寿命(单位:分钟；最大值:511；最小值:0；LSB:1；MSB:256；)	9
    4	30~36	FixData 浮标通道号(最大值:99；最小值:0；LSB:1；MSB:64；) 7
    3	22~29	Integer 通道电平(最小值:0；最大值:255；)	8
    2	8~21		SPARE	14
    1	7~7 Bool	浮标来源(0-本平台;1-全平台)	1
    1	0~6 	SPARE	7
    */
};


///浮标投放信息单值  主
typedef struct FCAE_DATABUOY_S
{
    //DWORD id;   ///<编号
    BYTE id;
    BYTE tong;
    WORD plan;
    union FCAE_BUOY_DATA buoydata;
    double lo;   ///<经度
    double la;   ///<纬度
}FCAE_DATABUOY;


//  1 浮标状态信息
typedef struct YR_JM_FBStt_Unit
{
    WORD 	PingTaiID;						//平台编号:有人机0 无人机1001~1999
    WORD 	SeaZone_buoynum;						//海域浮标数量
    //BuoyInfo buoys[16];				//浮标信息
    FCAE_DATABUOY buoys[16];
}YR_JM_FBStt_Unit;




/***************************** KDL_CCC_000 **********************************/

// 2 声目标信息  000
typedef struct ShengTargetInf{
    char   head[2];			//同步码    0xEB 0x94 0xB1
    char   zhentype;		//帧类别 0xD1
    short  planeId;         // 无人平台编号  1001-1999
    int    tgtId;           // 目标编号     new
    char   tgtAvail;        // 目标有效性    new
    unsigned int timeStamp;   // 时戳         new  从零开始的整秒数
    char  datFlag;          // 数据标志 0：无效 1：有效
    //char   tgtBatch;          // 目标批次号 0~255
    int	  tgtBatch;
    int   tgtIdx;           // 目标顺序号（发现次数）
    char  timeeff;          // 发现时间有效 0：无效 1：有效
    char  poseff;           // 目标位置有效 0：无效 1：有效
    char  direff;           // 目标方位有效 0：无效 1：有效
    char  disteff;          // 目标距离有效 0：无效 1：有效
    char  coueff;           // 目标航向有效 0：无效 1：有效
    char  veleff;           // 目标航速有效 0：无效 1：有效
    char  refeff;           // 距离方位参考点有效 0：无效 1：有效
    short year;             // 年[2000，2100]
    char  month;            // 月 [1,12]
    char  day;              // 日 [1,31]
    int   tgtTms;           // 毫秒 [1,86400000]
    double   tgtlon;        // 目标位置经度[-180,180)
    double   tgtlat;        // 目标位置纬度[-90,90]
    float    posAccur;      // 目标位置精度（误差半径）
    float    tgtDirect;     // 目标方位 [0，360)
    float    dirAccur;      // 目标方位精度
    float    tgtDist;       // 目标距离
    float    distAccur;     // 目标距离精度
    float    tgtHead;         // 目标航向 [0，360)
    float    headAccur;     // 目标航向精度
    float    tgtSpeed;        // 目标航速  [0,25] m/s
    float    speedAccur;  // 目标航速精度
    double   reflon;          // 距离方位参考点经度 [-180,180)
    double   reflat;           // 距离方位参考点纬度 [-90,90]
    char     judgeMode;  // 判决方式 0：N/A；1：自动；2：浮标；3：吊声；4：磁探；5：声磁融合
    char     tgtAttr;          // 目标属性 0：N/A，1：敌，2：友；
    char     tgtType;        // 目标类型 0：N/A    1：潜艇，2：水面舰，3：民船，4：其它；
    char     confidence;  // 目标置信度 [0，100]
    unsigned int locbuo[8];  //10    // 参与定位浮标编号
    char     crc;           //数据校验字节（前面所有字节相加，模256）

}ShengTargetInf;


// 3 磁目标信息 001
typedef struct CiTargetInf{
    short num; 		// 目标数量
    int id; 		// 目标编号；该目标ID号
    char valid; 	// 目标有效性；0-无效，1-有效
    short year; 	// 年 [2000, 2100]
    char month; 	// 月 [1, 12]
    char day; 		// 日 [1,31]
    int ms; 		// 毫秒；当天0点开始的毫秒数
    double lo; 		// 目标经度
    double la; 		// 目标纬度
    char state; 	// 目标识别状态； 0-无效，1-自动，2-人工
    char type; 		// 目标属性；0-无效，1-铁磁性目标，2非铁磁性目标
    char confidence;// 目标置信度 ; 值域[1，100]
    float distance; // 目标连续接触距离;单位m
    float strength; // 目标信号强度，单位nt
    float error; 	// 目标位置误差， 单位m
    char side; 		// 目标左右侧：0-无效，1-左侧，2-右侧
    float slope; 	// 目标斜距：单位m
    float flightCourse; // 飞行航向； 单位°
    float flightSpeed; // 飞行速度；单位m/s
    float flightHeight; // 飞行高度；单位m
}CiTargetInf;


// 4 磁通用数据包 002
typedef struct ci_ty_data_pack{

    unsigned short tongbu_code; // 同步码
    char frame_type; // 帧类别
    char frame_head; // 帧头
    unsigned data_source_id; // 数据源编号
    unsigned data_mubiao_id; // 数据目的编号
    char data_flag; // 数据标志
    unsigned short serial_number; // 序列号
    unsigned short valid_data_lem; // 有效数据长度
    char data[185]; // 数据包
    char check_bit; // 校验字节
    char frame_eop; // 帧尾
}ci_ty_data_pack;

// 5 声通用数据包 003
typedef struct vioce_ty_data_pack{
    unsigned short tongbu_code; // 同步码
    char frame_type; // 帧类别
    char frame_head; // 帧头
    unsigned data_source_id; // 数据源编号
    unsigned data_mubiao_id; // 数据目的编号
    char data_flag; // 数据标志
    unsigned short serial_number; // 序列号
    unsigned short valid_data_lem; // 有效数据长度
    char data[185]; // 数据包
    char check_bit; // 校验字节
    char frame_eop; // 帧尾
} vioce_ty_data_pack;


//typedef struct Sheng_UniDataPack{
//    char	head[2]; 	//声处理状态信息同步码			0xEB 0x94
//    char	msgID;	 	//帧类别					0xD2
//    WORD	SrcID;   	//数据源编号
//    WORD 	DstID;   	//数据目的编号
//    char 	dataFlag;	//数据标识
//    char 	hop;	 	//帧头 					0x66
//    WORD  	Serialnum;	//序列号
//    WORD  	len;		//有效数据长度
//    char 	data[185];	//数据内容
//    char 	crc;		//校验
//    char 	eop;		//帧尾					0xBB
//}Sheng_UniDataPack;






/*BLK_KDL_CCC_002 start******************/

// CCC-KDL/MMM-002 控制权交接申请(MessageID:0xa62a02)
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
} SIGNAL_FC00;//舰面站交接参数

typedef struct {
    char synchronization_code[2];//同步码 2 0xEB, 0x94
    char frame_class;//帧类别 1 0xDA
    unsigned char index_id;// 流水号 1  1-255循环
    unsigned int UAV_ID;//无人机ID 4
    unsigned int F_ID2;//有人机ID 4
    SIGNAL_FC00 CR_StationHandoverParamss; //舰面站交接参数 9
    unsigned char jyh; //校验和 1       0-256(和校验)
} BLK_KDL_CCC_002;//控制权交接申请

/*BL_KDL_CCC_002 end******************/



/*BLK_KDL_CCC_003******************/
//空地链状态
typedef struct
{
    unsigned int 	ShipID;				//舰面端ID
    unsigned char 	IFShipCon;			//上行是否锁定  0-NA，1-锁定，2-失锁
    unsigned char 	IFShipCon_2;		//下行是否锁定  0-NA，1-锁定，2-失锁
    unsigned char 	ConSignalStren;		//信号强度
    unsigned char 	SPARE[9];			//spare
}CLCOStatus;
typedef struct kdl_status3{
    char KDLCNumber;   			// c链连接舰面端数量
    CLCOStatus clcostatus[4]; 	// 舰面端C链连接状态信息
}BLK_KDL_CCC_003;				//0xa62003


/*BLK_KDL_CCC_015 start******************/

// 控制权交接申请应答   0xa62a0a
typedef struct {
    unsigned char synchronization_code[2];//同步码 2 0xEB, 0x94
    unsigned char frame_class;//帧类别 1 0xD9
    unsigned char index_id;// 流水号 1  1-255循环
    unsigned char ans; // 应答信息 1 /  0：同意；1：拒绝
    unsigned char jyh; //校验和 1       0-256(和校验)
} BLK_KDL_CCC_015;

/*BLK_KDL_CCC_010 end******************/





#pragma pack()
#endif // ICD_KDL_CCC_H
