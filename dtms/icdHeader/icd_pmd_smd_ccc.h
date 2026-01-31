#ifndef ICD_PMD_SMD_CCC_H
#define ICD_PMD_SMD_CCC_H

#pragma pack(1)

typedef struct
{
    unsigned char Order_flag;           //命令标识 0-NA 1-fb参数命令 2-发射机命令 3-fb控制命令 4-维护自检 5-功能自检
    unsigned char OrderSource;          //命令来源  0-NA 1-有人机 2-舰面站
    unsigned char Data[30];             //数据内容_黑包
}B_Control_Data;//浮标遥调数据
//typedef struct
//{
//    unsigned short UAV_ID;              //无人机地址
//    unsigned char B_Control_Code;       //浮标遥调编码 磁探控制指令:0x31 浮标控制指令:0x34 0x35 交替发送
//    unsigned char B_Control_Code_1;     //浮标遥调编码  三判二重复
//    unsigned char B_Control_Code_1_1;   //浮标遥调编码  三判二重复
//    unsigned char Sum_Check;            //校验和
//}BLK_PMD_CCC_000,BLK_SMD_CCC_000;//无人机浮标参数设置指令 0x985000

typedef struct
{
//	unsigned short UAV_ID;				//无人机地址
//	unsigned char Control_Code_1;		//浮标遥调编码
//	unsigned char Control_Code_2;		//浮标遥调编码
//	unsigned char Control_Code_3;		//浮标遥调编码
//	unsigned char Control_Data[32];		//浮标遥调数据
//	unsigned char Sum_Check;			//校验和

	unsigned char Control_Code_1;		//浮标遥调编码
	unsigned char Control_Code_2;		//浮标遥调编码
	unsigned char Control_Code_3;		//浮标遥调编码
	unsigned char send_Data[2];			//接收/目的编码
	unsigned short UAV_ID;				//无人机地址
	unsigned char Control_Data[28];		//浮标遥调数据
}BLK_PMD_CCC_000,BLK_SMD_CCC_000;//无人机浮标参数设置指令 0x9c5000  20250807new

typedef struct
{
    unsigned int DYFBZT:8;  //对应浮标状态 0-未知 1-失效 2-存活未接触 3-存活已解除
    unsigned int FBLX:4;    //浮标类型 0-未装填 1-有浮筒无浮标 2-被动全向浮标 3-被动定向浮标 4-主动全向浮标
    unsigned int FBPL:3;    //浮标频率
    unsigned int FBSD:3;    //浮标深度
    unsigned int FBSM:9;    //浮标寿命
    unsigned int FBTDH:7;   //浮标通道号
    unsigned int TDDP:8;    //通道电平
    unsigned int spare1:14;
    unsigned int FBLY:1;    //浮标来源 0-本平台 1-全平台
    unsigned int spare2:7;
}SeaZoneBouyInfo;
typedef struct
{
    unsigned int BOUY_ID1;                      //浮标编号
    SeaZoneBouyInfo sea_zone_bouy_info;         //海域浮标信息
    double BouyPointLon;                        //海域浮标经度
    double BouyPointLat;                        //海域浮标纬度
}BouyData;//浮标信息
typedef struct
{
    unsigned short PlatForm;                    //平台编号
    unsigned short BouyNum;                     //海域浮标数量
    BouyData bouy_data[16];                     //浮标信息
}BLK_PMD_CCC_001,BLK_SMD_CCC_001;//已投浮标信息 0x985001
#pragma pack()

#endif // ICD_PMD_SMD_CCC_H
