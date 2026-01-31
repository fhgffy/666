#include "flightConflictDetection.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EARTH_RADIUS 6371000.0  // 鍦扮悆鍗婂緞锛屽崟浣嶏細绫�
#define CONFLICT_DISTANCE 3000.0  // 鍐茬獊璺濈锛屽崟浣嶏細绫�

/**
 * 璁＄畻涓ょ偣闂寸殑璺濈
 */
double distance(Grid p1, Grid p2) {
    double dx;
    double dy;

    dx = p2.east - p1.east;
    dy = p2.north - p1.north;
    return sqrt(dx * dx + dy * dy);
}

/**
 * 鍒ゆ柇鐐规槸鍚﹀湪绾挎涓�
 */
int isPointOnSegment(Grid lineStart, Grid lineEnd, Grid point) {

	double crossProduct;

	double dotProduct;

	double squaredLength;



    crossProduct = (point.east - lineStart.east) * (lineEnd.north - lineStart.north) -
                         (point.north - lineStart.north) * (lineEnd.east - lineStart.east);
    
    if (fabs(crossProduct) > 1e-6) return 0;  // 涓嶅湪鐩寸嚎涓�

    dotProduct = (point.east - lineStart.east) * (lineEnd.east - lineStart.east) +
                       (point.north - lineStart.north) * (lineEnd.north - lineStart.north);
    
    if (dotProduct < 0) return 0;  // 鍦ㄨ捣鐐圭殑鍚庨潰
    
    squaredLength = (lineEnd.east - lineStart.east) * (lineEnd.east - lineStart.east) +
                          (lineEnd.north - lineStart.north) * (lineEnd.north - lineStart.north);
    
    if (dotProduct > squaredLength) return 0;  // 鍦ㄧ粓鐐圭殑鍓嶉潰
    
    return 1;  // 鍦ㄧ嚎娈典笂
}

/**
 * 璁＄畻鐐瑰埌鐩寸嚎鐨勮窛绂诲拰鍨傝冻
 */
double pointToLineDistance(Grid lineStart, Grid lineEnd, Grid point, Grid* footPoint) {
    double dx;
    double dy;
    double u;

    dx = lineEnd.east - lineStart.east;
    dy = lineEnd.north - lineStart.north;
    
    if (fabs(dx) < 1e-6 && fabs(dy) < 1e-6) {
        // 绾挎涓ょ鐐归噸鍚�
        footPoint->east = lineStart.east;
        footPoint->north = lineStart.north;
        return distance(point, lineStart);
    }
    
    // 璁＄畻鍨傝冻
    u = ((point.east - lineStart.east) * dx + (point.north - lineStart.north) * dy) / (dx * dx + dy * dy);
    footPoint->east = lineStart.east + u * dx;
    footPoint->north = lineStart.north + u * dy;
    
    // 璁＄畻璺濈
    return distance(point, *footPoint);
}

/**
 * 璁＄畻涓ゆ潯绾挎鐨勪氦鐐�
 */
int lineIntersection(Grid p1, Grid p2, Grid p3, Grid p4, Grid* intersection) {
    double dx1;
    double dy1;
    double dx2;
    double dy2;
    
    double denominator;
    
    double u1;
    double u2;

    dx1 = p2.east - p1.east;
    dy1 = p2.north - p1.north;
    dx2 = p4.east - p3.east;
    dy2 = p4.north - p3.north;

    denominator = dx1 * dy2 - dy1 * dx2;

    if (fabs(denominator) < 1e-6) {
        return 0;  // 绾挎骞宠鎴栭噸鍚�
    }
    
    u1 = ((p3.east - p1.east) * dy2 - (p3.north - p1.north) * dx2) / denominator;
    
    intersection->east = p1.east + u1 * dx1;
    intersection->north = p1.north + u1 * dy1;
    
    // 妫�煡浜ょ偣鏄惁鍦ㄧ嚎娈典笂
    if (u1 < 0 || u1 > 1) return 0;
    
    u2 = ((p3.east - p1.east) * dy1 - (p3.north - p1.north) * dx1) / denominator;
    if (u2 < 0 || u2 > 1) return 0;
    
    return 1;
}


/**
 * 妫�祴涓ゆ潯鑸嚎鏄惁鏈夊啿绐�
 * @param routeA 鏈変汉鏈鸿埅绾�
 * @param routeB 鏃犱汉鏈鸿埅绾�
 * @param result 鍐茬獊缁撴灉
 * @return 0-鎴愬姛锛屽叾浠�澶辫触
 */
int detectConflict(FlightRoute* routeA, FlightRoute* routeB, ConflictResult* result) {

	// 閫夋嫨鑸嚎A鐨勮捣鐐逛綔涓哄弬鑰冪偣
    Geo* origin;

    // 灏嗚埅绾胯浆鎹负鍖椾笢鍧愭爣绯�
    Grid routeAStart;
    Grid routeAEnd;
    Grid routeBStart;
    Grid routeBEnd;
    Grid intersection;

	// 鍒濆鍖栫粨鏋�
    result->hasConflict = 0;


    
    // 閫夋嫨鑸嚎A鐨勮捣鐐逛綔涓哄弬鑰冪偣
    origin = &routeA->start;
    
    // 灏嗚埅绾胯浆鎹负鍖椾笢鍧愭爣绯�
    routeAStart = getGridByGeo(&routeA->start, origin);
    routeAEnd = getGridByGeo(&routeA->end, origin);
    routeBStart = getGridByGeo(&routeB->start, origin);
    routeBEnd = getGridByGeo(&routeB->end, origin);
    
    // 妫�煡绾挎鏄惁鐩镐氦

    if (lineIntersection(routeAStart, routeAEnd, routeBStart, routeBEnd, &intersection)) {
        result->hasConflict = 1;
        result->conflictPoint = intersection;
        result->conflictGeo = getGeoByGrid(&intersection, origin);
        return 1;
    }
    
//    // 妫�煡鑸嚎B鐨勭鐐瑰埌鑸嚎A鐨勮窛绂�
//    Grid footPoint;
//    double dist1 = pointToLineDistance(routeAStart, routeAEnd, routeBStart, &footPoint);
//    if (dist1 <= CONFLICT_DISTANCE && isPointOnSegment(routeAStart, routeAEnd, footPoint)) {
//        result->hasConflict = 1;
//        result->conflictPoint = routeBStart;
//        result->conflictGeo = routeB.start;
//        return 0;
//    }
//
//    double dist2 = pointToLineDistance(routeAStart, routeAEnd, routeBEnd, &footPoint);
//    if (dist2 <= CONFLICT_DISTANCE && isPointOnSegment(routeAStart, routeAEnd, footPoint)) {
//        result->hasConflict = 1;
//        result->conflictPoint = routeBEnd;
//        result->conflictGeo = routeB.end;
//        return 0;
//    }
//
//    // 妫�煡鑸嚎A鐨勭鐐瑰埌鑸嚎B鐨勮窛绂�
//    double dist3 = pointToLineDistance(routeBStart, routeBEnd, routeAStart, &footPoint);
//    if (dist3 <= CONFLICT_DISTANCE && isPointOnSegment(routeBStart, routeBEnd, footPoint)) {
//        result->hasConflict = 1;
//        result->conflictPoint = routeAStart;
//        result->conflictGeo = routeA.start;
//        return 0;
//    }
//
//    double dist4 = pointToLineDistance(routeBStart, routeBEnd, routeAEnd, &footPoint);
//    if (dist4 <= CONFLICT_DISTANCE && isPointOnSegment(routeBStart, routeBEnd, footPoint)) {
//        result->hasConflict = 1;
//        result->conflictPoint = routeAEnd;
//        result->conflictGeo = routeA.end;
//        return 0;
//    }
//
    return 0;
}

/**
 * 璁＄畻瑙勯伩鐐�
 * @param routeA 鏈変汉鏈鸿埅绾�
 * @param routeB 鏃犱汉鏈鸿埅绾�
 * @param conflictPoint 鍐茬獊鐐�
 * @param avoidancePoint 瑙勯伩鐐圭粨鏋�
 * @return 0-鎴愬姛锛屽叾浠�澶辫触
 */
int calculateAvoidancePoint(FlightRoute* routeA, FlightRoute* routeB, Grid conflictPoint, Geo* avoidancePoint) {
    // 閫夋嫨鑸嚎A鐨勮捣鐐逛綔涓哄弬鑰冪偣
    Geo* origin;

    // 灏嗚埅绾胯浆鎹负鍖椾笢鍧愭爣绯�
    Grid routeAStart;
    Grid routeAEnd;
    Grid routeBStart;
    Grid footPoint;
    Grid avoidanceNE;
    double dx;
    double dy;
    double dist;

    double lineDx;
    double lineDy;
    
    // 閫夋嫨鑸嚎A鐨勮捣鐐逛綔涓哄弬鑰冪偣
	origin = &routeA->start;

	// 灏嗚埅绾胯浆鎹负鍖椾笢鍧愭爣绯�
	routeAStart = getGridByGeo(&routeA->start, origin);
	routeAEnd = getGridByGeo(&routeA->end, origin);
	routeBStart = getGridByGeo(&routeB->start, origin);
    
    // 璁＄畻浠庤埅绾緽璧风偣鍒拌埅绾緼鐨勫瀭瓒�

    pointToLineDistance(routeAStart, routeAEnd, routeBStart, &footPoint);
    
    // 璁＄畻鏂瑰悜鍚戦噺锛堜粠鍨傝冻鍒拌埅绾緽璧风偣锛�
    dx = routeBStart.east - footPoint.east;
    dy = routeBStart.north - footPoint.north;
    dist = sqrt(dx * dx + dy * dy);
    
    if (dist < 1e-6) {
        // 濡傛灉鏃犱汉鏈鸿捣鐐规濂藉湪鏈変汉鏈鸿埅绾夸笂锛屽垯閫夋嫨鍨傜洿鏂瑰悜
        // 璁＄畻鑸嚎A鐨勬柟鍚戝悜閲�
        lineDx = routeAEnd.east - routeAStart.east;
        lineDy = routeAEnd.north - routeAStart.north;
        
        // 鍨傜洿鏂瑰悜鍚戦噺锛堟棆杞�0搴︼級
        dx = -lineDy;
        dy = lineDx;
        dist = sqrt(dx * dx + dy * dy);
    }
    
    // 褰掍竴鍖栨柟鍚戝悜閲�
    dx /= dist;
    dy /= dist;
    
    // 璁＄畻璺濈鑸嚎3鍏噷鐨勭偣
    avoidanceNE.east = footPoint.east + dx * CONFLICT_DISTANCE;
    avoidanceNE.north = footPoint.north + dy * CONFLICT_DISTANCE;
    
    // 杞崲涓虹粡绾害
    *avoidancePoint = getGeoByGrid(&avoidanceNE, origin);
    
    return 0;
}
