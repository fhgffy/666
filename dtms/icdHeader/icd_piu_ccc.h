#ifndef ICD_PIU_CCC_H
#define ICD_PIU_CCC_H

#pragma pack(1)

typedef struct
{
    unsigned int p:1; //奇校验
    unsigned int SSM:2;//MSB LSB
    unsigned int spare1:16;
    unsigned int status_change:2; //状态控制切换反馈 0H=N/A;1H=有人直升机光电;2H=无人机光电
    unsigned int quanxian:1; //当前权限状态 0H=驾驶舱(OFP);1H=任务舱(MDCS);上电前默认前舱
    unsigned int spare2:2;
    unsigned int tab:8;//标号1
}LABLE;//光电设备权限状态
typedef struct
{
    unsigned int p:1; //奇校验
    unsigned int SSM:2;//MSB LSB
    unsigned int spare1:12;
    unsigned int F_NOTE:8;/*手柄按键信息 0H=空;1H=权限切换;;2H=视场+;3H=视场-;4H=调焦+;5H=调焦-;
                            AH=收藏(归零);BH=(跟踪)目标+;CH=(跟踪)目标-;DH=电视/红外;EH=黑/白热(红外正/负像);10H=手动(手控)/跟踪;11H=激光测距/停止(激光单次测量/无停止);*/
    unsigned int status:1;//手柄状态 0H=前舱;1H=后舱
    unsigned int tab:8;//标号 201
}JOY_KEY_01;
typedef struct
{
    unsigned int p:1; //奇校验
    unsigned int SSM:2;//MSB LSB
    unsigned int spare1:4;
    int integer:16;
    unsigned int status:1;//手柄状态 0H=前舱;1H=后舱
    unsigned int tab:8;//标号 202
}JOY_KEY_02;
typedef struct
{
    unsigned int p:1; //奇校验
    unsigned int SSM:2;//MSB LSB
    unsigned int spare1:4;
    int integer:16;
    unsigned int status:1;//手柄状态 0H=前舱;1H=后舱
    unsigned int tab:8;//标号 203
}JOY_KEY_03;
typedef struct
{
    LABLE lable;            //光电设备权限状态
    JOY_KEY_01 joy_key_01;  //手柄按键信息
    JOY_KEY_02 joy_key_02;  //手柄方位游标信息
    JOY_KEY_03 joy_key_03;  //手柄俯仰游标信息
}BLK_PIU_CCC_006;//手柄控制量


#pragma pack()

#endif // ICD_PIU_CCC_H
