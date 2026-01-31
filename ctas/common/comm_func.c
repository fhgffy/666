#include <stdlib.h>
#include <string.h>
#include "NewSrc/common/comm_func.h"


double cross1(Point a, Point b ,Point c)
{
    return (b.lon - a.lon) * (c.lat - a.lat) - (b.lat - a.lat) * (c.lon - a.lon);
}

int compare(const void *vp1, const void *vp2)
{
    Point* p1 = (Point*)vp1;
    Point* p2 = (Point*)vp2;
    if(p1->lon != p2->lon) return p1->lon < p2->lon;
    return p1->lat < p2->lat;
}

int convexHull(Point * points,Point * hull,int n)
{
    //按x坐标排序(x相同则按y排序)
    qsort(points,n,sizeof(Point),compare);

    //构建下凸包(从左到右扫描)
    int k = 0;
    for(int i = 0 ; i < n ; i ++)
    {
        while(k >= 2 && cross1(hull[k-2],hull[k-1],points[i]) <= 0) k--;
        hull[k++] = points[i];
    }

    //构建上凸包(从右到左扫描)
    int t = k + 1;
    for(int i = n-2 ; i >= 0 ; i --)
    {
        while(k >= t && cross1(hull[k-2],hull[k-1],points[i]) <= 0) k--;
        hull[k++] = points[i];
    }

    n = k -1;//去除重复的起点

    return n;
}


MabrGeo air_area(Point * points,Point * hull,int n)
{
    //输入航路点，算出凸包多边形
    int num = 0;
    num = convexHull(points,hull, n);
    //凸包点数最多15个
    if(num > 15)num =15;
    //初始化外接矩形的输入
    PolygonGeo poly_hull;
    poly_hull.sumVertex = num;
    for(int i = 0 ; i < num ; i ++)
    {
        poly_hull.vertexA[i].longitude = hull[i].lon;
        poly_hull.vertexA[i].latitude = hull[i].lat;
    }

    //算出外接矩形，即空域
    MabrGeo temp = getMabrGeo(&poly_hull);

    return temp;
}

//经纬度转墨卡托坐标
void geoToMercator(double lon , double lat,double *x,double *y)
{
    *x = lon * (M_PI/180.0) * 6378137.0;
    double latRad = lat * (M_PI/180.0);
    *y = log(tan((M_PI/4) + (latRad/2))) * 6378137.0;
}
//墨卡托坐标转经纬度
void mercatorToGeo(double x,double y , double *lon,double *lat)
{
    *lon = x * (180.0/M_PI)/6378137.0;
    double yRad = y/6378137.0;
    *lat = (2* atan(exp(yRad)) - M_PI/2) * (180.0/M_PI);
}
//正方形顶点
void generateSquare(double centerLon,double centerLat,double sideMeters,double result[4][2])
{
    //计算半边长
    double halfSide = sideMeters/2.0;
    //中心点转墨卡托坐标
    double centerMercX,centerMercY;
    geoToMercator(centerLon,centerLat,&centerMercX,&centerMercY);
    //计算四个顶点的墨卡托坐标
    double offsets[4][2] =
    {
        {-halfSide,-halfSide},
        { halfSide,-halfSide},
        { halfSide, halfSide},
        {-halfSide, halfSide}
    };
    //转换为经纬度并存储结果
    for(int i = 0 ; i < 4 ; i ++)
    {
        double mercX = centerMercX + offsets[i][0]* cos(centerLat *(M_PI/180.0));
        double mercY = centerMercY + offsets[i][1];
        mercatorToGeo(mercX,mercY,&result[i][0],&result[i][1]);
    }
}









