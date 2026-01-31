#ifndef ICD_CCC_DLR_H
#define ICD_CCC_DLR_H

#include "CommonDefine.h"
#pragma pack(1)

#if  _PC_SIMULATION_
typedef struct
{
      char load_status[20]; /*加载状态： 0-N/A，1-传输中，2-传输完毕，3-传输失败，4-正在加载，
                            5-加载成功，6-加载失败-原因1，7-加载失败-原因2,8-加载失败-原因3

                            系统代号：0 - 超台话音, 1 - 超台数传, 2 - 罗盘, 3 - 空置,
                            4 - 短波, 5 - JIDS, 6 - 武协链, 7 -  905数据链（仅J有）, 8 - 搜救,
                            9 - 卫通, 10 - 音频告警文件, 11 - 音频告警表, 12 - 北斗, 13 - 航点航线,
                            14 - 雷达数据库, 15 - 投放程序, 16-卫通IP，17-超3话（仅J有）18-协同指控 19-协同通信*/
      char spare[4];
      char load_progress;   /*加载进度,0-100*/
}BLK_CCC_DLR_000;/*参数加载状态响应*/
#else//半物理
typedef struct
{
    unsigned char load_status;//参数状态反馈 0-NA,1-传输中,2-传输完毕,3-传输失败,4-正在加载,5-加载成功,6-加载失败
    unsigned char load_progress;//进度
}BLK_CCC_DLR_000;

#endif
#pragma pack()

#endif // ICD_CCC_DLR_H
