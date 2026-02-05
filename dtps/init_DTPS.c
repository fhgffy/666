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
#include "speechMain.h"
#include "sndCardDriver.h"

#ifdef CONFIG_APP_AUTO_TEST
#include <test/acAutoTest.h>
#include <test/ac653TestCos.h>
#endif /* CONFIG_APP_AUTO_TEST */

/* 全局数据 */
/* 记录当前分区ID */
static PARTITION_ID_TYPE curPartitionId = 0;

static QUEUING_PORT_ID_TYPE gIdQuePortSendVoiceDtms;
static QUEUING_PORT_ID_TYPE gIdQuePortRecvVoiceDtms;

//接收
BLK_DTMS_DTPS_047 blk_dtms_dtps_047;
//发送
BLK_DTPS_DTMS_043 blk_dtps_dtms_043;

void taskSpeechXunFei(void);

void initQuePort(void)
{
    RETURN_CODE_TYPE ret = NO_ERROR;

    CREATE_QUEUING_PORT("DTPS_R_DTMS_VOICE",
                        512,
                        8,
                        DESTINATION,
                        FIFO,
                        &gIdQuePortRecvVoiceDtms,
                        &ret);
    printf("|dtps| CREATE_QUEUING_PORT DTPS_R_DTMS_VOICE ret(%d) ID(%ld) \n",
            ret,
            gIdQuePortRecvVoiceDtms);

    CREATE_QUEUING_PORT("DTPS_S_DTMS_VOICE",
                        512,
                        8,
                        SOURCE,
                        FIFO,
                        &gIdQuePortSendVoiceDtms,
                        &ret);
    printf("|dtps| CREATE_QUEUING_PORT DTPS_S_DTMS_VOICE ret(%d) ID(%ld) \n",
            ret,
            gIdQuePortSendVoiceDtms);


}

int sendVoiceMsgToDtms(unsigned char* msgA, long int const lenSend)
{
    RETURN_CODE_TYPE ret = NO_ERROR;
    SEND_QUEUING_MESSAGE(gIdQuePortSendVoiceDtms, msgA, lenSend, 0, &ret);


    if (ret != NO_ERROR) {
//        printf("|dtps| err SEND DTPS_S_DTMS_VOICE ret(%d) \n",
//                ret);
        return -1;
    }

    return 0;
}

void taskRecvDtmsVoice(void)
{
    RETURN_CODE_TYPE ret = NO_ERROR;
    MESSAGE_SIZE_TYPE lenRecv;
//    SYSTEM_TIME_TYPE timeStart, timeEnd;

    unsigned char msgA[512];

    printf("|dtps| taskRecvDtmsVoice start \n");
    while (1)
    {
    	PERIODIC_WAIT(&ret);
//    	static SYSTEM_TIME_TYPE last_timeStart = 0;

//        GET_TIME(&timeStart, &ret);
        RECEIVE_QUEUING_MESSAGE(gIdQuePortRecvVoiceDtms,
                                0,
                                msgA,
                                &lenRecv,
                                &ret);
        //TO DO
        if(ret == 0)
        {
        	//收到DTMS转发综显的指令信息
        	memcpy(&blk_dtms_dtps_047,msgA,sizeof(BLK_DTMS_DTPS_047));
        	speechMainProc();
        }

        //test
//        speechMainProc();

//        GET_TIME(&timeEnd, &ret);
//		printf("|dtps| process USER1 time(%lld); period:(%lld) \n",
//				timeEnd - timeStart, timeStart - last_timeStart);

//		last_timeStart = timeStart;

    }
}



/* 进程参数 */
static PROCESS_ATTRIBUTE_TYPE ProcessesPTR[] = {
    { "taskRecvDtmsVoice", /* 进程名;*/
      taskRecvDtmsVoice,    /* 进程入口地址 */
      819200,       /* 进程栈大小 */
      16,          /* 进程优先级 */
      50000000ll,          /* 进程周期 */
      20000000ll,         /* 进程截止期 */
      SOFT        /* 进程截止期类型 */
    },

	{ "taskSpeechXunFei", /* 进程名;*/
	taskSpeechXunFei,    /* 进程入口地址 */
	819200,       /* 进程栈大小 */
	16,          /* 进程优先级 */
	50000000ll,          /* 进程周期 */
	30000000ll,         /* 进程截止期 */
	SOFT        /* 进程截止期类型 */
	},

};

/**
 * 语音识别任务,因为讯飞的pcie通信必须循环读取空，所以单独建立一个任务
 */
void taskSpeechXunFei(void)
{
	RETURN_CODE_TYPE retCode = NO_ERROR;
	PARTITION_STATUS_TYPE partitionStatus;
	char msg[256];

	/*获取分区状态*/
	GET_PARTITION_STATUS(&partitionStatus, &retCode);
	if (retCode != NO_ERROR) {
		printf("|dtps| err partitionStatus ret(%d) \n", retCode);
	}

	static short flag = 0;

	SYSTEM_TIME_TYPE timeStart, timeEnd;

	while (1)
	{
		PERIODIC_WAIT(&retCode);
//		GET_TIME(&timeStart, &retCode);

		// 暂时讯飞不具备条件，先和ofp测试，所以屏蔽20251004
		speechXunFeiMainProc();

//		GET_TIME(&timeEnd, &retCode);
//		printf("|dtms| process taskSpeech time(%lld) \n",
//				timeEnd - timeStart);
		//SUSPEND_SELF(1000000000ll,&retCode);
	}
}

/*
* @brief      
*    该函数为初始化进程的入口。
* @return      
*    无。
* @implements 
*/
void appMain( void )
{

    RETURN_CODE_TYPE retCode = NO_ERROR;
    PROCESS_ID_TYPE pid[10];
    PARTITION_STATUS_TYPE partitionStatus;

    SUSPEND_SELF(6000000000ll, &retCode);
    printf("\n|dtps| Enter App Entry\n");

    /*获取分区状态*/
    GET_PARTITION_STATUS(&partitionStatus, &retCode);

    if (retCode == NO_ERROR) {
        printf("|dtps| Partition %d has started!\n",
                (int) partitionStatus.IDENTIFIER);
        curPartitionId = partitionStatus.IDENTIFIER;
    } else {
        printf("|dtps| err partition status failure??retCode:%d\n", retCode);
        return;
    }

    initQuePort();


	//语音卡初始化
    int ret = 0;
	ret = sndCardInit(0, VENDOR_ID, DEVICE_ID, 0);
	if(ret < 0)
	{
		printf("sndCardInit failed\n");
		return;
	}
	else
	{
		printf("sndCardInit success\n");
	}


    int sumProcess = sizeof(ProcessesPTR) / sizeof(ProcessesPTR[0]);
    printf("|dtps| sumProcess(%d) \n", sumProcess);

    int i;
    for (i = 0; i < sumProcess; ++i) {
        CREATE_PROCESS(&ProcessesPTR[i], &pid[i], &retCode);

        if (retCode != NO_ERROR) {
            printf("|dtps| Parition %d create process %s failure,retCode:%d\n",
                    (int) curPartitionId, ProcessesPTR[i].NAME, retCode);
        }

        START(pid[i], &retCode);

        if (retCode != NO_ERROR) {
            printf("|dtps| Parition %d start process %s failure,retCode:%d\n",
                    (int) curPartitionId, ProcessesPTR[i].NAME, retCode);
        }
    }

    SET_PARTITION_MODE(NORMAL,&retCode);
}

