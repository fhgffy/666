#ifndef COMM_FUNCTION_CRC16_H
#define COMM_FUNCTION_CRC16_H

#include "NewSrc/common/CommonDefine.h"
#include "libSourceCode/commDas/area.h"

typedef struct
{
    double lon;//经度
    double lat;//纬度
}Point;

typedef struct {
    Geo start;  // 起始点
    Geo end;    // 结束点
} FlightRoute;

MabrGeo air_area(Point * points,Point * hull,int n);

void generateSquare(double centerLon,double centerLat,double sideMeters,double result[4][2]);

#endif
