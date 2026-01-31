#pragma once

#include "CommonDefine.h"

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(1)

/**
 * 灏嗙粡绾害杞崲涓哄寳涓滃潗鏍囩郴
 * @param point 缁忕含搴︾偣
 * @param origin 鍙傝�鐐癸紙鍘熺偣锛�
 * @return 鍖椾笢鍧愭爣绯荤偣
 */
Grid geoToNE(Geo point, Geo origin);

/**
 * 妫�祴涓ゆ潯鑸嚎鏄惁鏈夊啿绐�
 * @param routeA 鏈変汉鏈鸿埅绾�
 * @param routeB 鏃犱汉鏈鸿埅绾�
 * @param result 鍐茬獊缁撴灉
 * @return 0-鎴愬姛锛屽叾浠�澶辫触
 */
int detectConflict(FlightRoute* routeA, FlightRoute* routeB, ConflictResult* result);

/**
 * 璁＄畻瑙勯伩鐐�
 * @param routeA 鏈変汉鏈鸿埅绾�
 * @param routeB 鏃犱汉鏈鸿埅绾�
 * @param conflictPoint 鍐茬獊鐐�
 * @param avoidancePoint 瑙勯伩鐐圭粨鏋�
 * @return 0-鎴愬姛锛屽叾浠�澶辫触
 */
int calculateAvoidancePoint(FlightRoute* routeA, FlightRoute* routeB, Grid conflictPoint, Geo* avoidancePoint);


#pragma pack()

#ifdef __cplusplus
}
#endif
