/*
 * comm_func_crc16.c
 *
 *  Created on: 2025年1月21日
 *      Author: Admin
 */

#include "comm_func_crc16.h"
#include "CommonVaribles.h"
//#include "target_estimation.h"

#define EARTH_RADIUS 6371000.0  // 地球半径，单位：米
#define RAD_PER_DEG (PI / 180.0)
#define DEG_PER_RAD (180.0 / PI)

// 观测数据结构
typedef struct {
    time_t timestamp;   // 时间戳（秒）
    double latitude;    // 纬度（度，北纬为正）
    double longitude;   // 经度（度，东经为正）
} Observation;

// 目标运动状态
typedef struct {
    double speed_knots;     // 航速（节）
    double speed_mps;       // 航速（米/秒）
    double course_deg;      // 航向（度，0°为正北，顺时针增加）
    double course_rad;      // 航向（弧度）
    double latitude;        // 当前位置纬度
    double longitude;       // 当前位置经度
    time_t last_update;     // 最后更新时间
} TargetState;

// 模型计算结果
typedef struct {
    int valid;              // 计算结果是否有效
    double speed_knots;     // 航速（节）
    double speed_mps;       // 航速（米/秒）
    double course_deg;      // 航向（度）
    double pred_lat;        // 预测纬度
    double pred_lon;        // 预测经度
    double distance;        // 航行距离（海里）
    double distance_m;      // 航行距离（米）
} CalculationResult;


unsigned int crc_ta[256] = { /*   CRC余式表   */
	0x0000,   0x1021,   0x2042,   0x3063,   0x4084,   0x50a5,   0x60c6,   0x70e7,
	0x8108,   0x9129,   0xa14a,   0xb16b,   0xc18c,   0xd1ad,   0xe1ce,   0xf1ef,
	0x1231,   0x0210,   0x3273,   0x2252,   0x52b5,   0x4294,   0x72f7,   0x62d6,
	0x9339,   0x8318,   0xb37b,   0xa35a,   0xd3bd,   0xc39c,   0xf3ff,   0xe3de,
	0x2462,   0x3443,   0x0420,   0x1401,   0x64e6,   0x74c7,   0x44a4,   0x5485,
	0xa56a,   0xb54b,   0x8528,   0x9509,   0xe5ee,   0xf5cf,   0xc5ac,   0xd58d,
	0x3653,   0x2672,   0x1611,   0x0630,   0x76d7,   0x66f6,   0x5695,   0x46b4,
	0xb75b,   0xa77a,   0x9719,   0x8738,   0xf7df,   0xe7fe,   0xd79d,   0xc7bc,
	0x48c4,   0x58e5,   0x6886,   0x78a7,   0x0840,   0x1861,   0x2802,   0x3823,
	0xc9cc,   0xd9ed,   0xe98e,   0xf9af,   0x8948,   0x9969,   0xa90a,   0xb92b,
	0x5af5,   0x4ad4,   0x7ab7,   0x6a96,   0x1a71,   0x0a50,   0x3a33,   0x2a12,
	0xdbfd,   0xcbdc,   0xfbbf,   0xeb9e,   0x9b79,   0x8b58,   0xbb3b,   0xab1a,
	0x6ca6,   0x7c87,   0x4ce4,   0x5cc5,   0x2c22,   0x3c03,   0x0c60,   0x1c41,
	0xedae,   0xfd8f,   0xcdec,   0xddcd,   0xad2a,   0xbd0b,   0x8d68,   0x9d49,
	0x7e97,   0x6eb6,   0x5ed5,   0x4ef4,   0x3e13,   0x2e32,   0x1e51,   0x0e70,
	0xff9f,   0xefbe,   0xdfdd,   0xcffc,   0xbf1b,   0xaf3a,   0x9f59,   0x8f78,
	0x9188,   0x81a9,   0xb1ca,   0xa1eb,   0xd10c,   0xc12d,   0xf14e,   0xe16f,
	0x1080,   0x00a1,   0x30c2,   0x20e3,   0x5004,   0x4025,   0x7046,   0x6067,
	0x83b9,   0x9398,   0xa3fb,   0xb3da,   0xc33d,   0xd31c,   0xe37f,   0xf35e,
	0x02b1,   0x1290,   0x22f3,   0x32d2,   0x4235,   0x5214,   0x6277,   0x7256,
	0xb5ea,   0xa5cb,   0x95a8,   0x8589,   0xf56e,   0xe54f,   0xd52c,   0xc50d,
	0x34e2,   0x24c3,   0x14a0,   0x0481,   0x7466,   0x6447,   0x5424,   0x4405,
	0xa7db,   0xb7fa,   0x8799,   0x97b8,   0xe75f,   0xf77e,   0xc71d,   0xd73c,
	0x26d3,   0x36f2,   0x0691,   0x16b0,   0x6657,   0x7676,   0x4615,   0x5634,
	0xd94c,   0xc96d,   0xf90e,   0xe92f,   0x99c8,   0x89e9,   0xb98a,   0xa9ab,
	0x5844,   0x4865,   0x7806,   0x6827,   0x18c0,   0x08e1,   0x3882,   0x28a3,
	0xcb7d,   0xdb5c,   0xeb3f,   0xfb1e,   0x8bf9,   0x9bd8,   0xabbb,   0xbb9a,
	0x4a75,   0x5a54,   0x6a37,   0x7a16,   0x0af1,   0x1ad0,   0x2ab3,   0x3a92,
	0xfd2e,   0xed0f,   0xdd6c,   0xcd4d,   0xbdaa,   0xad8b,   0x9de8,   0x8dc9,
	0x7c26,   0x6c07,   0x5c64,   0x4c45,   0x3ca2,   0x2c83,   0x1ce0,   0x0cc1,
	0xef1f,   0xff3e,   0xcf5d,   0xdf7c,   0xaf9b,   0xbfba,   0x8fd9,   0x9ff8,
	0x6e17,   0x7e36,   0x4e55,   0x5e74,   0x2e93,   0x3eb2,   0x0ed1,   0x1ef0
};


/*
 kkl crc16计算函数
 */
unsigned short do_crc_table(unsigned char *ptr,int len)
{
	unsigned short crc;
	unsigned char da;

	crc = 0x0000;

	while (len-- != 0) {
		da = (unsigned short) crc >> 8; /*   以8位二进制数的形式暂存CRC的高8位   */
		crc <<= 8; /*   左移8位，相当于CRC的低8位乘以 256    */
		crc ^= crc_ta[da ^ *ptr]; /*   高8位和当前字节相加后再查表求CRC   ，再加上以前的CRC   */
		ptr++;
	}
	return (crc);
}

/*
 * 计算使数据满足模256为0的校验码
 * data 数据其实指针
 * length 数据长度（字节）
 */
unsigned char checkSum(unsigned char *ptr,int len)
{
    unsigned char sum = 0;
    // 逐一叠加计算和
    for(int i = 0; i < len; ++i)
    {
        sum += ptr[i];
    }

    return sum;
}

//协同指令集,循环队列创建
void createQueue(CircularQueue * q)
{
    memset(&q->buffer[0],0,sizeof(op_code)*MAX_QUEUE);
    q->count = 0;
}

//入队
void enqueue(CircularQueue * q, op_code * op)
{
    //队列满时整体向前移一位，删除最早的一条指令
    if(q->count == MAX_QUEUE)
    {
        for(int i = 0 ; i < MAX_QUEUE - 1 ; i ++)
        {
            q->buffer[i] = q->buffer[i + 1];
        }
        q->count--;
    }
    //插入新元素到队尾
    memcpy(&q->buffer[q->count],op,sizeof(op_code));
    q->count++;
}




/**
 * @brief  无人机入编队，入队成功返回序号，不成功返回-1
 * @param uavId
 * @return
 */
int formationIdInsert(int uavId)
{
    int rtn = -1;
    for(int i = 0 ; i < UAV_MAX_NUM ; i++)
    {
        if(formationId[i].planeId == 0)
        {
            formationId[i].planeId = uavId;
            rtn = i;
            break;
        }
    }
    return rtn;
}

/**
 * @brief formationIdManage 输入飞机id（0x10XX样式），返回id在编队的序号，如果不在编队，有空则加入空位，没空则返回-1
 * @param id
 * @return
 */
int formationIdManage(int uavId)
{
    int index = -1;
    // 如果有匹配的，则说明该无人机id已在编队，则直接返回序号即可
    for(int i = 0 ; i < UAV_MAX_NUM ; i++)
    {
        if(uavId == formationId[i].planeId)
        {
            index = i;
            break;
        }
    }

    // 如果没有匹配，则找空位放下，找不到则返回-1
    if(index == -1)
    {
        index = formationIdInsert(uavId);
    }
    return index;
}

/**
 * @brief formationIdCountIncrease,超时计数自增
 */
void formationIdCountIncrease()
{
    for(int i= 0; i<UAV_MAX_NUM;i++)
    {
        // 接收计数自增
        formationId[i].count++;

        // 空id则置为formationIdTimeOutCount+1即可（设为超时，同时避免自增溢出）
        if(formationId[i].planeId ==0)
        {
            formationId[i].count = formationIdTimeOutCount+1;
        }

        // 超过超时计数则判为超时，退出编队
        if(formationId[i].count > formationIdTimeOutCount)
        {
        	// （设为超时，同时避免自增溢出）
            formationId[i].count = formationIdTimeOutCount+1;
            //20250803new，不清空控制权
//            formationId[i].isControl= 0;
            //超时清空控制站 20250804new
            formationId[i].station_address = 0;
            //超时清空链来源
            formationId[i].C_U = 0;
        }

    }
}

/**
 * @brief  清空指定编队序号uavindex的的超时计数,表示收到了
 * @param uavid 是序号，非id
 */
void formationIdResetCount(int uavIndex)
{
    if(uavIndex<0 || uavIndex>=UAV_MAX_NUM)
        return;

    // 查找
    formationId[uavIndex].count = 0;
}

/**********************************************************************************************************************************************************************************/

/**
 * 将角度转换为弧度
 */
double deg_to_rad(double deg) {
    return deg * RAD_PER_DEG;
}

/**
 * 将弧度转换为角度
 */
double rad_to_deg(double rad) {
    return rad * DEG_PER_RAD;
}

/**
 * 将米/秒转换为节（1节 = 0.514444米/秒）
 */
double mps_to_knots(double mps) {
    return mps / 0.514444;
}

/**
 * 将节转换为米/秒
 */
double knots_to_mps(double knots) {
    return knots * 0.514444;
}

/**
 * 计算两点间的大圆距离（米）
 * 使用Haversine公式
 */
double calculate_distance_m(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = deg_to_rad(lat1);
    double phi2 = deg_to_rad(lat2);
    double delta_phi = deg_to_rad(lat2 - lat1);
    double delta_lambda = deg_to_rad(lon2 - lon1);

    double a = sin(delta_phi/2.0) * sin(delta_phi/2.0) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda/2.0) * sin(delta_lambda/2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return EARTH_RADIUS * c;
}

/**
 * 计算两点间的初始航向（度）
 * 使用球面航向公式
 */
double calculate_initial_course(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = deg_to_rad(lat1);
    double phi2 = deg_to_rad(lat2);
    double lambda1 = deg_to_rad(lon1);
    double lambda2 = deg_to_rad(lon2);

    double y = sin(lambda2 - lambda1) * cos(phi2);
    double x = cos(phi1) * sin(phi2) -
               sin(phi1) * cos(phi2) * cos(lambda2 - lambda1);

    double course_rad = atan2(y, x);
    double course_deg = fmod(rad_to_deg(course_rad) + 360.0, 360.0);

    return course_deg;
}

/**
 * 根据起始点、距离和航向计算目标点坐标
 */
void calculate_destination_point(double lat1, double lon1,
                                 double distance, double course,
                                 double *lat2, double *lon2) {
    double angular_distance = distance / EARTH_RADIUS;
    double course_rad = deg_to_rad(course);

    double phi1 = deg_to_rad(lat1);
    double lambda1 = deg_to_rad(lon1);

    double phi2 = asin(sin(phi1) * cos(angular_distance) +
                       cos(phi1) * sin(angular_distance) * cos(course_rad));

    double lambda2 = lambda1 + atan2(sin(course_rad) * sin(angular_distance) * cos(phi1),
                                     cos(angular_distance) - sin(phi1) * sin(phi2));

    // 标准化经度到[-180, 180]
    lambda2 = fmod(lambda2 + 3.0 * PI, 2.0 * PI) - PI;

    *lat2 = rad_to_deg(phi2);
    *lon2 = rad_to_deg(lambda2);
}

/**
 * 从两个观测点解算目标运动参数
 */
CalculationResult calculate_motion_parameters(Observation obs1, Observation obs2) {
    CalculationResult result = {0};

    // 检查时间差是否有效
    double time_diff = difftime(obs2.timestamp, obs1.timestamp);
    if (time_diff <= 0) {
        result.valid = 0;
        return result;
    }

    // 计算距离
    double distance_m = calculate_distance_m(obs1.latitude, obs1.longitude,
                                            obs2.latitude, obs2.longitude);
    double distance_nm = distance_m / 1852.0;  // 转换为海里

    // 计算航速
    result.speed_mps = distance_m / time_diff;
    result.speed_knots = mps_to_knots(result.speed_mps);

    // 计算航向
    result.course_deg = calculate_initial_course(obs1.latitude, obs1.longitude,
                                                obs2.latitude, obs2.longitude);

    result.distance = distance_nm;
    result.distance_m = distance_m;
    result.pred_lat = obs2.latitude;
    result.pred_lon = obs2.longitude;
    result.valid = 1;

    return result;
}

/**
 * 根据历史运动参数推算未来位置
 */
void predict_future_position(TargetState *state, time_t future_time,
                            double *pred_lat, double *pred_lon) {
    double time_diff = difftime(future_time, state->last_update);

    if (time_diff <= 0) {
        *pred_lat = state->latitude;
        *pred_lon = state->longitude;
        return;
    }

    double travel_distance = state->speed_mps * time_diff;

    calculate_destination_point(state->latitude, state->longitude,
                               travel_distance, state->course_deg,
                               pred_lat, pred_lon);
}

/**
 * 更新目标状态（使用多个观测点的加权平均）
 */
int update_target_state(TargetState *state, Observation *observations,
                       int count, double *confidence) {
    if (count < 2) {
        *confidence = 0.0;
        return 0;
    }

    double total_weight = 0.0;
    double weighted_speed_knots = 0.0;
    double weighted_speed_mps = 0.0;
    double weighted_course_sin = 0.0;
    double weighted_course_cos = 0.0;

    for (int i = 1; i < count; i++) {
        CalculationResult res = calculate_motion_parameters(
            observations[i-1], observations[i]);

        if (res.valid) {
            // 使用时间差作为权重（最近的数据权重更高）
            double time_diff = difftime(observations[i].timestamp,
                                       observations[i-1].timestamp);
            double weight = time_diff;

            weighted_speed_knots += res.speed_knots * weight;
            weighted_speed_mps += res.speed_mps * weight;

            // 处理航向的角度平均
            double course_rad = deg_to_rad(res.course_deg);
            weighted_course_sin += sin(course_rad) * weight;
            weighted_course_cos += cos(course_rad) * weight;

            total_weight += weight;
        }
    }

    if (total_weight == 0.0) {
        *confidence = 0.0;
//        return 0;
    }

    // 计算加权平均值
    state->speed_knots = weighted_speed_knots / total_weight;
    state->speed_mps = weighted_speed_mps / total_weight;

    // 计算平均航向
    double avg_sin = weighted_course_sin / total_weight;
    double avg_cos = weighted_course_cos / total_weight;
    state->course_rad = atan2(avg_sin, avg_cos);
    state->course_deg = fmod(rad_to_deg(state->course_rad) + 360.0, 360.0);

    // 使用最后一个观测点作为当前位置
    state->latitude = observations[count-1].latitude;
    state->longitude = observations[count-1].longitude;
    state->last_update = observations[count-1].timestamp;

    // 计算置信度（基于数据点数量和时间跨度）
    double time_span = difftime(observations[count-1].timestamp,
                               observations[0].timestamp);
    *confidence = fmin(1.0, (double)count / 10.0 *
                      fmin(1.0, time_span / 3600.0));  // 最大1小时

    return 1;
}

/**
 * 格式化时间显示
 */
void format_time(time_t timestamp, char *buffer, size_t size) {
    struct tm *tm_info = localtime(&timestamp);
    strftime(buffer, size, "%Y-%m-%d %H:%M:%S", tm_info);
}

/**
 * 打印计算结果
 */
void print_result(CalculationResult *result, time_t current_time) {
//    printf("\n=== print_result ===\n");
//    printf("valid: %s\n", result->valid ? "valid" : "invalid");
//
//    if (result->valid) {
//        printf("speed_knots: %.2f j (speed_mps %.2f m/s)\n",
//               result->speed_knots, result->speed_mps);
//        printf("course_deg: %.1f°\n", result->course_deg);
//        printf("distance: %.2f hl\n", result->distance);
//        printf("pred_lat: %.6f°N,pred_lon %.6f°E\n",
//               result->pred_lat, result->pred_lon);
//    }
//    printf("=======================\n");
}

/**
 * 打印目标状态
 */
void print_target_state(TargetState *state, double confidence) {
    char time_str[20];
    format_time(state->last_update, time_str, sizeof(time_str));

//    printf("\n=== state ===\n");
//    printf("time_str: %s\n", time_str);
//    printf("latitude: %.6f°N, longitude %.6f°E\n",
//           state->latitude, state->longitude);
    printf("speed_mps: %.3f m/s\n", state->speed_mps);
    printf("course_deg: %.1f°\n", state->course_deg);
//    printf("confidence: %.1f%%\n", confidence * 100);
//    printf("====================\n");
}

/**
 * 示例主函数
 */
int target_estimation_test() {
    printf("target_estimation_test v1.0\n");
    printf("=====================\n\n");

    // 示例：模拟一组观测数据
    Observation observations[5];
    time_t base_time = 47700;// 13:15

    // 观测点1
    observations[0].timestamp = 45000;  // 12:30 45000
    observations[0].latitude = 30.0;
    observations[0].longitude = 120.0;

    // 观测点2
    observations[1].timestamp = 45600;  // 12:40 45600
    observations[1].latitude = 30.014444;
    observations[1].longitude = 120.009722;

    // 观测点3
    observations[2].timestamp = 46800;  // 13:00 46800
    observations[2].latitude = 30.043333;
    observations[2].longitude = 120.028888;

    // 观测点4
    observations[3].timestamp = 47100;  // 13:05 47100
    observations[3].latitude = 30.050555;
    observations[3].longitude = 120.033611;

    // 步骤1：计算最近两个观测点间的运动参数
//    printf("calculate_motion_parameters...\n");
//    CalculationResult motion_result = calculate_motion_parameters(
//        observations[2], observations[3]);
//    print_result(&motion_result, base_time);

    // 步骤2：更新目标状态（使用所有观测点）
    TargetState current_state;
    double confidence;

    if (update_target_state(&current_state, observations, 4, &confidence)) {
        print_target_state(&current_state, confidence);

        // 步骤3：预测未来位置
        printf("\n=== predict_future_position===\n");

        // 预测30分钟后的位置
        time_t future_time_30min = base_time;
        double pred_lat, pred_lon;
        predict_future_position(&current_state, future_time_30min,
                               &pred_lat, &pred_lon);

        char future_time_str[20];
        format_time(future_time_30min, future_time_str, sizeof(future_time_str));
//        printf("future_time_str: %s (+15分钟)\n", future_time_str);
        printf("pred_lat: %.6f°N,pred_lon  %.6f°E\n", pred_lat, pred_lon);

//        // 预测1小时后的位置
//        time_t future_time_1hr = base_time + 3600;
//        predict_future_position(&current_state, future_time_1hr,
//                               &pred_lat, &pred_lon);
//
//        format_time(future_time_1hr, future_time_str, sizeof(future_time_str));
//        printf("\n预测时间: %s (+1小时)\n", future_time_str);
//        printf("预测位置: %.6f°N, %.6f°E\n", pred_lat, pred_lon);
    }

    return 0;
}

