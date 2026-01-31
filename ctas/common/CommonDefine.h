#ifndef COMMONDEFINE_H
#define COMMONDEFINE_H

#pragma pack(1)

typedef struct {
    double speed;
    double radius;
    double height;
    double eleDistance;
    double megDistance;
}UAV;

typedef struct {
    unsigned int program_number;//方案编号
    int strategyNo;//战术战法序号，标示所使用的是哪个战术战法
    unsigned int totalTime;//方案总时长
    unsigned short UAVnumber;//无人机数量
    unsigned short mannedTaskNumber;//有人机任务数量
    char isMinTime;//是否是最短时长
    char isMinUAV;//是否是无人机数量最少
    char isMInMannedTask;//是否是有人机任务数量最少
}StrategySelect;

typedef struct {

    char dataA[4096];
    int size;
}MyByteArray;


#pragma pack()

#endif // COMMONDEFINE_H
