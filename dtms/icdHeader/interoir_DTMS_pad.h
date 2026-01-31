#ifndef INTEROIR_DTMS_PAD_H
#define INTEROIR_DTMS_PAD_H

#pragma pack(1)

// 航迹预测信息
typedef struct forecast_target_hj{


}forecast_target_hj;


// 接收威胁区域信息 + 待航点位置信息
typedef struct Longitude_Latitude
{
    double longitude; //经度
    double latitude; //纬度
    double seaaltitude;//海拔高
}Longitude_Latitude;

typedef struct threat_circle
{
    Longitude_Latitude center; // 圆心坐标
    double Radius;	// 圆半径
}threat_circle;

typedef struct threat_polygon{
    int point_num; // 顶点数
    Longitude_Latitude point[10]; // 顶点坐标
}threat_polygon;

typedef struct threat_reg{
    int threat_type;  // 威胁类型 1-导弹
    int Level;//威胁级别 默认 0 调用时设置
    int threat_shape;  //威胁形状.1-圆；2-多边形
    threat_circle ThrCir;//威胁圆
    threat_polygon ThrPgn;//威胁多边形
} threat_reg;


typedef struct hold_point{ // 待航点位置
    Longitude_Latitude hold_point_loc;
    double NorthCurDir;  //真航向，与正北夹角，单位弧度
}hold_point;

//TODO 找不到MessageID
typedef struct threat_reg_hold_point{

   hold_point hold_point_info;  // 待航点位置
   threat_reg threat_info[5];   // 威胁区域信息

}threat_reg_hold_point;


// pad 心跳信息
typedef struct PAD_heart{
    unsigned short head;// 1:前舱；2：后舱
    int heart; // 0，不在线；1：在线；
}PAD_heart;





#pragma pack()

#endif // INTEROIR_DTMS_PAD_H
