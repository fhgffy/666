/*
 * icd_ccc_mmm.h
 *
 *  Created on: 2025年9月9日
 *      Author: Admin
 */

#ifndef ICD_CCC_MMM_H_
#define ICD_CCC_MMM_H_

#pragma pack(1)


/*日期*/
typedef struct
{
	unsigned int year; /*年*/
	unsigned int month;/*月*/
	unsigned int day;  /*日*/
} FC_DATE_LOCAL;

/*时间*/
typedef struct
{
	unsigned int hour;       /*时*/
	unsigned int minute;     /*分*/
	unsigned int second;     /*秒*/
	unsigned int millisecond;/*毫秒*/
	unsigned int microsecond;/*微秒*/
	unsigned int nanosecond; /*纳秒*/
} FC_TIME_LOCAL;

typedef struct
{

	FC_DATE_LOCAL fc_date;
	FC_TIME_LOCAL fc_time;
}BLK_MIDS_CCC_TIME;


typedef struct
{
    unsigned int head;// 0xAAAAAAAA
    unsigned char day;// 1-31
    unsigned char month;// 1-12
    unsigned short year;
    unsigned int msc;//毫秒时
    unsigned int msgId;// 发送消息id
    unsigned char srcId;// 源标记号
    unsigned char length[3];// 消息长度
    // unsigned char msgData[?];// 变长消息，所以不在结构体中定义，所以在发送缓存时添加
    // unsigned int tail;// 0xFFFFFFFF,变长，所以在发送缓存时添加
}BLK_CCC_MMM_001;

#pragma pack()

#endif /* ICD_CCC_MMM_H_ */
