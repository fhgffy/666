/*
 * 版权 2019-2020 中航工业西安航空计算技术研究所
 *
 * 对本文件的拷贝、发布、修改或任何其它用途必须得到
 * 中航工业西安航空计算技术研究所的书面协议许可.
 *
 * Copyrights（C）2019-2020 ACTRI
 * All Rights Reserved.
 */

/*
 * 变更历史：
 * 2022-05-13 slei  删除appMain中的参数。
 * 2022-05-11 ldpeng3 更新声明
 * 2020-12-16 slei  增加自动测试的相关代码，通过配置可使能。
 * 2011-09-05 slei  创建该文件。
 */

/*
 * @file  init.c
 * @brief
 *    功能:
 *    <li>本文件用于作为分区运行的基本模板。 </li>
 */

/* 包含文件 */
#include <os/pos/apex/apexLib.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "header_DTMS/main_header.h"
#include "Application/Application.h"
#include "versionSend/version_info.h"
#include "video/videoCpm.h"
#include "video/taskPacket.h"
#include "DDSUtil/DDSUtil.h"
//#include "./voice_module/voice_main.h"

#ifdef CONFIG_APP_AUTO_TEST
#include <test/acAutoTest.h>
#include <test/ac653TestCos.h>
#endif /* CONFIG_APP_AUTO_TEST */

extern STATUS pciFindDevice(int vendorId, int deviceId, int index, int* pBusNo,
		int* pDeviceNo, int* pFuncNo);

extern STATUS pcieConfigInLong(int nPEX, int busNo, int deviceNo, int funcNo,
		int offset, UINT32 *pData);


extern FILE_DATA                            load_file;     /// 预加载流数据处理
extern unsigned int                         dlr_load_state;         //任务点预加载标志 0:未开始；1：传输中；2：传输完毕

extern integrated_posture                    CCC_DPU_data_5;// 融合用

/* 函数声明 */
void appMain(void);
void USER1(void);
void USER2(void);
void taskSpeech(void);
void RecvVoiceDtps();
extern void taskSpeechXunFei(void);

void user_snd_recv(void);
void user_snd_send(void);

void suspendSelfNs(signed long long const ns) {
	RETURN_CODE_TYPE ret = NO_ERROR;
	SUSPEND_SELF(ns, &ret);
}

signed long long getSysTimeNs(void)
{
	RETURN_CODE_TYPE retCode = NO_ERROR;
	SYSTEM_TIME_TYPE timeSys;
	GET_TIME(&timeSys, &retCode);

	return timeSys;
}

static QUEUING_PORT_ID_TYPE gIdQuePortRecvMids01;
static QUEUING_PORT_ID_TYPE gIdQuePortRecvMids02;/// 预加载流数据处理
static QUEUING_PORT_ID_TYPE gIdQuePortRecvSfs01;
static QUEUING_PORT_ID_TYPE gIdQuePortSendSfs01;
static QUEUING_PORT_ID_TYPE gIdQuePortSendVoiceDtps;
static QUEUING_PORT_ID_TYPE gIdQuePortRecvVoiceDtps;

void initQuePort(void) {
	RETURN_CODE_TYPE ret = NO_ERROR;

	CREATE_QUEUING_PORT("DTMS_R_MIDS_01", 0x9608, 16, DESTINATION, FIFO,
			&gIdQuePortRecvMids01, &ret);

	printf("|dtms| CREATE_QUEUING_PORT ret(%d) \n", ret);

	CREATE_QUEUING_PORT("DTMS_R_SFS_01",
						2048,
						8,
						DESTINATION,
						FIFO,
						&gIdQuePortRecvSfs01,
						&ret);
	printf("|dtms| CREATE_QUEUING_PORT DTMS_R_SFS_01 ret(%d) ID(%ld) \n",
			ret,
			gIdQuePortRecvSfs01);

	CREATE_QUEUING_PORT("DTMS_S_SFS_01",
						2048,
						8,
						SOURCE,
						FIFO,
						&gIdQuePortSendSfs01,
						&ret);
	printf("|dtms| CREATE_QUEUING_PORT DTMS_S_SFS_01 ret(%d) ID(%ld) \n",
			ret,
			gIdQuePortSendSfs01);

    CREATE_QUEUING_PORT("DTMS_R_DTPS_VOICE",
                        512,
                        8,
                        DESTINATION,
                        FIFO,
                        &gIdQuePortRecvVoiceDtps,
                        &ret);
    printf("|dtms| CREATE_QUEUING_PORT DTMS_R_DTPS_VOICE ret(%d) ID(%ld) \n",
            ret,
            gIdQuePortRecvVoiceDtps);

    CREATE_QUEUING_PORT("DTMS_S_DTPS_VOICE",
                        512,
                        8,
                        SOURCE,
                        FIFO,
                        &gIdQuePortSendVoiceDtps,
                        &ret);
    printf("|dtms| CREATE_QUEUING_PORT DTMS_S_DTPS_VOICE ret(%d) ID(%ld) \n",
            ret,
            gIdQuePortSendVoiceDtps);
}

/// 预加载流数据处理
void initQuePort_loadTask(void) {
	RETURN_CODE_TYPE ret = NO_ERROR;

	CREATE_QUEUING_PORT("DTMS_R_MIDS_02", 0x100000, 1, DESTINATION, FIFO,
						&gIdQuePortRecvMids02, &ret);
//	__asm__("bkpt #0");
	printf("|dtms| CREATE_QUEUING_PORT_loadTask ret(%d) \n", ret);
}


//static unsigned char* packA = NULL;
//static unsigned char* msgA = NULL;

int taskRecvQuePortMids01(void) {
	RETURN_CODE_TYPE ret = NO_ERROR;
	MESSAGE_SIZE_TYPE lenRecv;

//	__asm__("bkpt #0");

	unsigned char* msgA = (unsigned char*) malloc(0x9608);

	SYSTEM_TIME_TYPE timeStart, timeEnd;

	//    unsigned char* packA = (unsigned char*)malloc(0x9608);
	TaskPacket();

//	printf("|dtms| taskRecvQuePortMids01 start \n");
	while (1) {
		PERIODIC_WAIT(&ret);

		while (1) {

			static SYSTEM_TIME_TYPE last_timeStart = 0;
			GET_TIME(&timeStart, &ret);

			// 读取预加载数据
			if(0 == dlr_load_state) {
				RECEIVE_QUEUING_MESSAGE(gIdQuePortRecvMids02, 0, &load_file, &lenRecv, &ret);
				if (0 == ret)
				{
					dlr_load_state = 1;// 开始传输
					printf("|dtms| RECEIVE_load_task_QUEUING_MESSAGE lenRecv(%d) \n",  lenRecv);

				}
			}

			//读空队列
			RECEIVE_QUEUING_MESSAGE(gIdQuePortRecvMids01, 0, msgA, &lenRecv,
					&ret);
			if(ret != 0)
			{
//				printf("|dtms| recv PortMids01 ERR: %d\n",ret);
			}
//			else
//				printf("|dtms| recv PortMids01 len %d\n",lenRecv);
//
//			GET_TIME(&timeEnd, &ret);
//
//			printf("|dtms| process PortMids01 time(%lld); period:(%lld) \n",
//					timeEnd - timeStart, timeStart - last_timeStart);
//
//			last_timeStart = timeStart;

//			if(0==ret)
//			{
//				printf("5[%x]6[%x]\n",*(msgA+5),*(msgA+6));
//			}
			if (0 == lenRecv) {
//				                printf("|dtms| RECEIVE_QUEUING_MESSAGE ret(%d) \n",ret);
				break;
			}

			if (lenRecv < 4) {
				continue;
			}

			if (lenRecv > 38404) {
				continue;
			}

			if (!isValidVideoMsgHead(msgA + 4)) {
				continue;
			}

			//GET_TIME(&timeStart, &ret);

			msgProcess(msgA + 4, (unsigned int) lenRecv - 4);

//			GET_TIME(&timeEnd, &ret);
//			printf("|dtms| vedio msgProcess time(%lld)\n",
//					timeEnd - timeStart);

//			suspendSelfNs(2000000);
//			printf("|video| msgProcess msgProcess \n");
//		    signed long long timeSys = getSysTimeNs();
//		    while( (getSysTimeNs() - timeSys) < 1000000 ) {
//		    	;
//		    }


		}
	}

	return TRUE;
}

int taskRecvQuePortSfs01(void)
{
	RETURN_CODE_TYPE ret = NO_ERROR;
	MESSAGE_SIZE_TYPE lenRecv;
	printf("|dtms| taskProcSfs start \n");

	unsigned char* msgA = (unsigned char*) malloc(0x1000); //长度需要与DTMS.xml匹配
	while (1) {
		PERIODIC_WAIT(&ret);

		RECEIVE_QUEUING_MESSAGE(gIdQuePortRecvSfs01,
								0,
								msgA,
								&lenRecv,
								&ret);


		if(ret == 0)
		{
			// 根据包头，把缓存内容拷贝到相应的变量中
			if(msgA[0] == 0x01 && msgA[1] == 0x01)// 融合目标信息包头为0x0101，小端
			{
				memcpy(&CCC_DPU_data_5, msgA, sizeof(integrated_posture));// 不用跳过包头
			}
			else if(msgA[0] == 0x02 && msgA[1] == 0x01)// 版本信息包头为0x0102，小端
			{
				// 跳过包头
			}

		}
	}

	return TRUE;
}

int sendVoiceMsgToDtps(unsigned char* msgA, long int const lenSend)
{
    RETURN_CODE_TYPE ret = NO_ERROR;
    SEND_QUEUING_MESSAGE(gIdQuePortSendVoiceDtps, msgA, lenSend, 0, &ret);


    if (ret != NO_ERROR) {
        printf("|dtms| err SEND_QUEUING_MESSAGE DTMS_S_DTPS_VOICE ret(%d) \n",
                ret);
        return -1;
    }

    return 0;
}


int sendSfsMsgToQuePort(unsigned char* msgA, long int const lenSend)
{
	RETURN_CODE_TYPE ret = NO_ERROR;
	SEND_QUEUING_MESSAGE(gIdQuePortSendSfs01, msgA, lenSend, 0, &ret);

	if (ret != NO_ERROR) {
		printf("|dtms| err sendSfsMsgToQuePort ret(%d) \n",
				ret);
		return -1;
	}

	return 0;
}

/* 全局数据 */

/* 进程参数 */
PROCESS_ATTRIBUTE_TYPE ProcessesPTR[]={
#if 1
		{ "USER1",   /* 进程名;*/
				USER1,      /* 进程入口地址 */
				819200,       /* 进程栈大小 */
				15,         /* 进程优先级 */
				50000000ll,          /* 进程周期 */
				30000000ll,         /* 进程截止期 */
				SOFT        /* 进程截止期类型 */
		},
#endif

#if 1
		{ "taskRecvQuePortMids01", /* 进程名;*/
				taskRecvQuePortMids01,    /* 进程入口地址 */
				819200,       /* 进程栈大小 */
				16,          /* 进程优先级 */
				50000000ll,          /* 进程周期 */
				20000000ll,         /* 进程截止期 */
				SOFT        /* 进程截止期类型 */
		},
#endif

#if 0
		{ "taskRecvQuePortSfs01", /* 进程名;*/
		taskRecvQuePortSfs01,    /* 进程入口地址 */
		8192,       /* 进程栈大小 */
		17,          /* 进程优先级 */
		50000000ll,          /* 进程周期 */
		2000000ll,         /* 进程截止期 */
		SOFT        /* 进程截止期类型 */
		},
#endif

#if 0
		{ "taskSpeech", /* 进程名;*/
		taskSpeech,    /* 进程入口地址 */
		819200,       /* 进程栈大小 */
		14,          /* 进程优先级 */
		50000000ll,          /* 进程周期 */
		7000000ll,         /* 进程截止期 */
		SOFT        /* 进程截止期类型 */
		},
#endif

#if 0
		{ "taskSpeechXunFei", /* 进程名;*/
		taskSpeechXunFei,    /* 进程入口地址 */
		819200,       /* 进程栈大小 */
		14,          /* 进程优先级 */
		50000000ll,          /* 进程周期 */
		8000000ll,         /* 进程截止期 */
		SOFT        /* 进程截止期类型 */
		},
#endif

};

/* 记录当前分区ID */
PARTITION_ID_TYPE curPartitionId = 0;

/* 实现 */
/*
 * @brief
 *    该函数为用户进程1的函数体。
 * @return
 *    无。
 * @implements
 */
void USER1(void) {
#ifndef CONFIG_APP_AUTO_TEST
	RETURN_CODE_TYPE retCode = NO_ERROR;
	PARTITION_STATUS_TYPE partitionStatus;
	char msg[256];
	/*获取分区状态*/
	GET_PARTITION_STATUS(&partitionStatus, &retCode);
	if (retCode != NO_ERROR) {
		printf("|dtms| err partitionStatus ret(%d) \n", retCode);
	}

//	printf("|dtms| process USER1 \n");
	//    return;

	static short flag = 0;

	SYSTEM_TIME_TYPE timeStart, timeEnd;

	printf("|dtms| Ver 2.01 build %s, %s \n",
			__DATE__, __TIME__);

	while (1)
	{
		static SYSTEM_TIME_TYPE last_timeStart = 0;
		PERIODIC_WAIT(&retCode);
		GET_TIME(&timeStart, &retCode);

		Main_Task();

		versionInfoSend();

		RecvVoiceDtps();

		GET_TIME(&timeEnd, &retCode);
//		printf("|dtms| process USER1 time(%lld); period:(%lld) \n",
//				timeEnd - timeStart, timeStart - last_timeStart);
		//SUSPEND_SELF(1000000000ll,&retCode);

		last_timeStart = timeStart;

	}
#else
	/* 只有在分区设置为正常模式后部分测试方可运行 */
	printf("\n auto test procedure");
	auto_test_procedure();
#endif /* CONFIG_APP_AUTO_TEST */
}

/*
 * @brief
 *    该函数为用户进程2的函数体。
 * @return
 *    无。
 * @implements
 */
void USER2(void) {
	RETURN_CODE_TYPE retCode = NO_ERROR;
	PARTITION_STATUS_TYPE partitionStatus;
	char msg[256];

	/*获取分区状态*/
	GET_PARTITION_STATUS(&partitionStatus, &retCode);

	while (1) {
		//		sprintf(msg,"partition %d USER2 running!\n",(int)curPartitionId);
		//		printf("%s",msg);
		//		send_track_change(); // 跨周期发送无人机航线帧数据
		SUSPEND_SELF(100000000ll, &retCode);

		if (retCode != TIMED_OUT) {
			sprintf(msg, "partition %d suspend DATA_RECV failure! retCode:%d\n",
					(int) curPartitionId, retCode);
			printf("%s", msg);
		}
	}
}

///**
// * 语音识别任务
// */
//void taskSpeech(void)
//{
//	RETURN_CODE_TYPE retCode = NO_ERROR;
//	PARTITION_STATUS_TYPE partitionStatus;
//
//	/*获取分区状态*/
//	GET_PARTITION_STATUS(&partitionStatus, &retCode);
//	if (retCode != NO_ERROR) {
//		printf("|dtms| err partitionStatus ret(%d) \n", retCode);
//	}
//
//	SYSTEM_TIME_TYPE timeStart, timeEnd;
//
//
//	while (1)
//	{
//		static SYSTEM_TIME_TYPE last_timeStart = 0;
//		PERIODIC_WAIT(&retCode);
//		GET_TIME(&timeStart, &retCode);
//
//		speechMainProc();
////		recv_voice_main();
//
//		GET_TIME(&timeEnd, &retCode);
////		printf("|dtms| process taskSpeech time(%lld); period:(%lld) \n",
////				timeEnd - timeStart, timeStart - last_timeStart);
//		//SUSPEND_SELF(1000000000ll,&retCode);
//
//		last_timeStart = timeStart;
//	}
//}
//
///**
// * 语音识别任务,因为讯飞的pcie通信必须循环读取空，所以单独建立一个任务
// */
//void taskSpeechXunFei(void)
//{
//	RETURN_CODE_TYPE retCode = NO_ERROR;
//	PARTITION_STATUS_TYPE partitionStatus;
//	char msg[256];
//
//	/*获取分区状态*/
//	GET_PARTITION_STATUS(&partitionStatus, &retCode);
//	if (retCode != NO_ERROR) {
//		printf("|dtms| err partitionStatus ret(%d) \n", retCode);
//	}
//
//
//	static short flag = 0;
//
//	SYSTEM_TIME_TYPE timeStart, timeEnd;
//
//
//	while (1)
//	{
//		PERIODIC_WAIT(&retCode);
//		GET_TIME(&timeStart, &retCode);
//
//		// 暂时讯飞不具备条件，先和ofp测试，所以屏蔽20251004
//		speechXunFeiMainProc();
//
//		GET_TIME(&timeEnd, &retCode);
////		printf("|dtms| process taskSpeech time(%lld) \n",
////				timeEnd - timeStart);
//		//SUSPEND_SELF(1000000000ll,&retCode);
//	}
//}

void RecvVoiceDtps()
{
	RETURN_CODE_TYPE ret = NO_ERROR;
	MESSAGE_SIZE_TYPE lenRecv;
	RECEIVE_QUEUING_MESSAGE(gIdQuePortRecvVoiceDtps,
							0,
							&blk_dtps_dtms_043,
							&lenRecv,
							&ret);
	//接收到DTPS返回的板卡信息,转发给综显
	if(ret == 0)
	{
//		printf("recv blk_dtps_dtms_043 len %d\n",lenRecv);
		data_length = sizeof(BLK_CCC_OFP_043);
		Send_Message(DDSTables.CCC_DPU_043.niConnectionId,0,&transaction_id, &blk_dtps_dtms_043, &message_type_id, data_length, &enRetCode);
	}
}
/*收取数据*/
void DataRecv(void) {
#ifndef CONFIG_APP_AUTO_TEST

	RETURN_CODE_TYPE retCode = NO_ERROR;
	PARTITION_STATUS_TYPE partitionStatus;
	char msg[256];

	/*获取分区状态*/
	GET_PARTITION_STATUS(&partitionStatus, &retCode);

	if (retCode != NO_ERROR) {
		printf("|dtms| ERROR??DATA_RECV partitionStatus ret(%d) \n", retCode);
		return;
	} else {
		printf("|dtms| process DATA_RECV \n");
	}

	while (1) {
		PERIODIC_WAIT(&retCode);
		//		sprintf(msg,"partition %d (DATA_RECV) running!\n",(int)curPartitionId);
		//		printf("%s",msg);

	}

#else
	/* 只有在分区设置为正常模式后部分测试方可运行 */
	printf("\n auto test procedure");
	auto_test_procedure();
#endif /* CONFIG_APP_AUTO_TEST */
}

void user_snd_recv(void) {
}

void user_snd_send(void) {
	RETURN_CODE_TYPE retCode = NO_ERROR;
	SUSPEND_SELF(1000000000ll, &retCode);
	//asm("bkpt #0");
}

/*
 * @brief
 *    该函数为初始化进程的入口。
 * @return
 *    无。
 * @implements
 */
void appMain(void) {
	//	Application
	//__asm__("bkpt #0");
	RETURN_CODE_TYPE retCode = NO_ERROR;
	PROCESS_ID_TYPE pid[10];
	PARTITION_STATUS_TYPE partitionStatus;

	SUSPEND_SELF(6000000000ll, &retCode);
	printf("\n|dtms| Enter App Entry\n");

	/*获取分区状态*/
	GET_PARTITION_STATUS(&partitionStatus, &retCode);

	if (retCode == NO_ERROR) {
		printf("|dtms| Partition %d has started!\n",
				(int) partitionStatus.IDENTIFIER);
		curPartitionId = partitionStatus.IDENTIFIER;
	} else {
		printf("|dtms| err partition status failure??retCode:%d\n", retCode);
		return;
	}

	initQuePort();

	initQuePort_loadTask();/// 预加载流数据处理

	initAddrOffsetToVpm();

	//    SET_PARTITION_MODE(NORMAL,&retCode);
	//    return ;

	initDDSTable();

	int sumProcess = sizeof(ProcessesPTR) / sizeof(ProcessesPTR[0]);
	printf("|dtms| sumProcess(%d) \n", sumProcess);

	int i;

	for (i = 0; i < sumProcess; ++i) {
		CREATE_PROCESS(&ProcessesPTR[i], &pid[i], &retCode);

		if (retCode != NO_ERROR) {
			printf("|dtms| Parition %d create process %s failure,retCode:%d\n",
					(int) curPartitionId, ProcessesPTR[i].NAME, retCode);
		}

		START(pid[i], &retCode);

		if (retCode != NO_ERROR) {
			printf("|dtms| Parition %d start process %s failure,retCode:%d\n",
					(int) curPartitionId, ProcessesPTR[i].NAME, retCode);
		}
	}

	SET_PARTITION_MODE(NORMAL, &retCode);
}

