#ifndef YAOCEDATAZIZHEN_H
#define YAOCEDATAZIZHEN_H
#pragma pack(1)
typedef struct YaoCeDataZiZhen1{
    unsigned char data[64];//64字节遥测相关的数据，暂不处理
}YaoCeDataZiZhen1;

typedef struct
{
	unsigned short sfdy_status:2;		//伺服单元状态 00正常 01未上点 10故障 11检测中
	unsigned short hwcgq_status:2;		//红外传感器状态
	unsigned short dscgq_status:2;		//电视传感器状态
	unsigned short xj_status:2;			//相机状态
	unsigned short txgzdy_status:2;		//图像跟踪单元状态
	unsigned short kzgldy_status:2;		//控制管理单元状态
	unsigned short gdzh_status:2;		//光电载荷状态
	unsigned short spare:2;
}GDstatus1;

typedef struct PhotoelecticPlatformStatus{
    //D0-D1 红外工作状态
    // 0-未上电
    // 1-制冷中 正在上电
    // 2-正常工作 已上电
    // 3-校正中 暂时不用
    unsigned short infrared_work_status : 2;
    //D2-D4 相机工作状态
    unsigned short camera_work_status : 3;

    //D5-D7 图像跟踪方式
    unsigned short image_track_mode : 3;

    //D8红外增强
    unsigned short infrared_enhancement : 1;

    //D9红外电子变倍
    unsigned short infrared_zoom : 1;

    //D10电视透露
    unsigned short tv_defogging : 1;

    //D11红外正向/负向
    unsigned short infrared_positive_negative : 1;

    //D12融合状态
    unsigned short fusion_status : 1;

    //D13-D15记忆跟踪状态
    unsigned short memory_track_status : 3;
}PhotEelecticPlatformStatus;

//光电载荷遥测数据 20赫兹
typedef struct YaoCeDataZiZhen2{
    unsigned char tongBuMa1;//分辨率1,范围EBH 同步码1
    unsigned char tongBuMa2;//分辨率1,范围90H 同步码2
    unsigned char zhenLeiBie;//分辨率1,范围66H 帧类别
    unsigned char xuHao;    //分辨率1,范围0-255 序号
    unsigned short zhiLingLeiXing;//分辨率1 指令类型反馈，根据该位判断指令已被接受
    //备注：若接收命令回告,00为空，FF为无效命令，无效指令清除时间为3s
    short fangweijiao;      //分辨率360/2^16 光电平台方位角
    short fuyangjiao;       //分辨率360/2^16 光电平台俯仰角
    //当图像主通道为电视时，读取CCD视场，当物理值小于10度时，为长焦
    unsigned short CCDfield; //分辨率0.01度 电视视场角
    //当图像主通道为红外时，读取IR视场，当物理值小于10度时，为长焦
    unsigned short IRfield; //分辨率0.01度 红外视场角
    unsigned short Camerfield;  //分辨率0.01度 相机视场角
    GDstatus1 GD_status1;  //光电平台状态字1
    unsigned short GDstatus2;   //暂不用
    PhotEelecticPlatformStatus GDstatus3;   //用于判断红外工作状态
    // 0-未上电
    // 1-制冷中 正在上电
    // 2-正常工作 已上电
    // 3-校正中 暂时不用 20241119
    unsigned short GDstatus4; //用于状态回复判断
    //当前仅需要状态字4，其中D0-D4表示光电工作状态 0-未上电、1-手动、2-自检、3-归零、4-地理跟踪、5-随动引导、6-多目标跟踪、7-位置、
    //D5-D7单杆灵敏性
    //D8 跟踪场景 0-对海 1-对岸
    //D9 图像主通道 0-电视1-红外 ，主要通过该位判断给综显的结果。
    //D10-D11 图像跟踪状态 0-非跟踪，1-正常跟踪，2-跟踪目标丢失
    //D12-D14 条带宽度，未定义
    //D15
    unsigned short Target1status; //缺少该信息定义，
    unsigned short Target1Lotnum;//分辨率1 目标批号
    unsigned short Target1Velocity;//分辨率0.1KM/H,目标航速
    unsigned short Target1Course;//分辨率0.00549316度，目标航向
    int Target1Lon;//分辨率180/（2^31-1）度，
    int Target1Lat;//分辨率180/（2^31-1）度
    short Target1Hight;//分辨率12000/（2^15-1） 单位
    unsigned short Target2status;//缺少该信息定义，
    unsigned short Target2Lotnum;//分辨率1 目标批号
    unsigned short Target2Velocity;//分辨率0.1KM/H,目标航速
    unsigned short Target2Course;//分辨率0.00549316度，目标航向
    int Target2Lon;//分辨率180/（2^31-1）度
    int Target2Lat;//分辨率180/（2^31-1）度
    short Target2Hight;//分辨率12000/（2^15-1） 单位
    unsigned short spare;//
    unsigned short CRC;//校验核 0mod 65535
}YaoCeDataZiZhen2;
typedef struct YaoCeDataZiZhen3{
    unsigned char data[64];//雷达和磁探相关数据，暂不处理
}YaoCeDataZiZhen3;
// 启用任务遥测第12-13字节（从1开始）的备用位作为无人机平台编号判断251002
typedef struct YaoCeDataZiZhen4{
    unsigned char data[64];//64字节暂不处理
}YaoCeDataZiZhen4;
typedef struct YaoCeDataZiZhen5{
    unsigned char data[64];//64字节暂不处理
}YaoCeDataZiZhen5;
typedef struct YaoCeDataZiZhen6{
    unsigned char data[64];//事件帧
}YaoCeDataZiZhen6;
//无人机1载荷遥测与指令响应信息数据 0xaa1002
//载荷遥测数据 KKL-CCC 小端数据共384字节
//class YaoCeDataZiZhen
//{
//public:
//    YaoCeDataZiZhen();
//};
typedef struct {
    YaoCeDataZiZhen1 YaoCeDataZiZhen1s;
    YaoCeDataZiZhen2 YaoCeDataZiZhen2s;
    YaoCeDataZiZhen3 YaoCeDataZiZhen3s;
    YaoCeDataZiZhen4 YaoCeDataZiZhen4s;
    YaoCeDataZiZhen5 YaoCeDataZiZhen5s;
    YaoCeDataZiZhen6 YaoCeDataZiZhen6s;
}  YaoCeDataZhen;
#pragma pack()

#endif // YAOCEDATAZIZHEN_H
