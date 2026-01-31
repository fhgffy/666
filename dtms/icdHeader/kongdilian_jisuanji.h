#ifndef KONGDILIAN_JISUANJI_H
#define KONGDILIAN_JISUANJI_H

/*
 * 协同通讯空地链-协同指控计算机
 *
 * */

#pragma pack(1)

//声纳浮标信息
typedef struct KDL_CCC_000
{
    int head;
    unsigned short length;
    char message[64]; //黑包,长度为64
}KDL_CCC_000;

//航线确认回报及确认航线
typedef struct KDL_CCC_001
{
    int head;
    unsigned short length;
    char message[64]; //黑包,长度为64
}KDL_CCC_001;

//空地链路状态
typedef struct CLCoStatus
{

    char ShipID_1[4];//舰面端ID 4
    char IFShipCon_1;//是否锁定 1
    char ConSignalStren_1;//信号强度 1
    char SPARE[10];//SPARE 10
}CLCoStatus;
typedef struct KDL_CCC_002
{
    int head;
    unsigned short length;
    char message[2096]; //消息,长度为2096
    unsigned short KDLCNumber_1;//C链连接舰面端数量
    CLCoStatus cLCoStatus[4];
}KDL_CCC_002;


/*
 * 协同通讯空空链-协同指控计算机
 * */

//飞行遥测信息数据帧
typedef struct KKL_CCC_000
{
    int head;
    unsigned short length;
    char message[160]; //黑包,长度为160
}KKL_CCC_000;

//载荷遥测与指令响应数据信息
typedef struct KKL_CCC_001
{
    int head;
    unsigned short length;
    char message[384]; //黑包,长度为384
}KKL_CCC_001;

//任务重规划结果
typedef struct KKL_CCC_002
{
    int head;
    unsigned short length;
    char message[512]; //黑包,长度为512
}KKL_CCC_002;


//任务业务信息
typedef struct KKL_CCC_003
{
    int head;
    unsigned short length;
    char message[2096]; //黑包,长度为2096
}KKL_CCC_003;

//任务业务信息

typedef struct CLParSet
{
    char NoHeliID_1_1[4];//无人机ID 4
    char DownWorkMode_1;//下行工作模式
    char DownPower_1;//下行功率选择
    char UpRate_1;//上行传输速率
    char DownRate_1;//下行传输速率
    char UpWorkMode_1;//上行工作模式
    char UpPower_1;//上行功率选择
    char PassWay_1;//加密方式
    char Cwire_1;//C相控阵天线选择
    char UpChannel_1;//上行频道选择
    char DownChannel_1;//下行频道选择
    char SPARE[22];//SPARE

}CLParSet;
typedef struct KKL_CCC_004
{
    int head;
    unsigned short length;
    char message[2096]; //数据长度为2096
    unsigned short KKLCNumber_1_1;//C链连接无人机数量
    CLParSet cLParSet[4];//无人机C链参数设置
}KKL_CCC_004;


//空空链链路状态数据
typedef struct CLCoStatus_KKL
{
    char NoHeliID[4];//无人机ID 4
    char IFNoHeliCon_1;//是否锁定 1
    char ConSignalStren_1;//信号强度 1
    char SPARE[10];//SPARE 10
}CLCoStatus_KKL;
typedef struct KKL_CCC_005
{
    int head;
    unsigned short length;
    char message[2096]; //数据长度为2096
    unsigned short KKLCNumber;//C链连接无人机数量
    CLCoStatus_KKL cLCoStatus[4];//无人机C链连接状态信息
}KKL_CCC_005;



//链路交接控制指令反馈
typedef struct CLParSet_009
{
    char ControlChange;//启动主链转移 1
    char RecGroundID[4];//接机站ID 4
    char ChangeConfirm;//C交接成功确认 1
    char SPARE[30];//SPARE 30

}CLParSet_009;
typedef struct KKL_CCC_009
{
    int head;
    unsigned short length;
    char message[2096]; //数据长度为2096
    unsigned short KKLCNumber_1_1_1_1;//C链连接无人机数量
    CLParSet_009 cLParSet[4];//无人机C链参数设置
}KKL_CCC_009;


//飞行故障清单
typedef struct KKL_CCC_190
{
    int head;
    unsigned short length;
    char message[2096]; //数据长度为2096
    char LEVEL_WARN_PFL_KKL_1[4];//飞行故障清单警告级
    char LEVEL_ATT_PFL_KKL_1[4];//飞行故障清单注意级
    char LEVEL_TIP_PFL_KKL_1[4];//飞行故障清单提示级
}KKL_CCC_190;




typedef struct wurenjifubiaoyaokongxinxi{
    unsigned short wurenjidizhi;
    char yaotiaobianma[3];
    char fubiaoyaokongxinxi[32];

}wurenjifubiaoyaokongxinxi;





#pragma pack()










#endif // KONGDILIAN_JISUANJI_H
