
/*
* 作    者：
* 创建日期：2024-11-04
* 模块名称：CRC16 校验函数
* 模块功能：
* 模块说明：
*/


/* 防止头文件被重复引用 */
#ifndef COMM_FUNCTION_CRC16_H
#define COMM_FUNCTION_CRC16_H

#include "CommonDefine.h"
/*
kkl crc16计算函数
*/
unsigned short do_crc_table(unsigned char *ptr,int len);

unsigned char checkSum(unsigned char *ptr,int len);

//协同指令集,循环队列创建
void createQueue(CircularQueue * q);

//入队
void enqueue(CircularQueue * q, op_code * op);


int formationIdManage(int uavId);

void formationIdCountIncrease();

void formationIdResetCount(int uavIndex);

int target_estimation_test();

#endif
