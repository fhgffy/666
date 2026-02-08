#include "voiceToText.h"

#include "sndCardDriver.h"

#include "speechGlobal.h"

#include "speechMain.h"

#include <os/pos/apex/apexLib.h>



extern int sendVoiceMsgToDtms(unsigned char* msgA, long int const lenSend);

extern BLK_DTMS_DTPS_047 blk_dtms_dtps_047;

extern BLK_DTPS_DTMS_043 blk_dtps_dtms_043;



static inline int8_t is_BigEnd(void)

{

	int8_t ret = 0;

	union end { uint8_t a[2]; uint16_t b; };

	union end c;

	c.b = 0x1234;

	return (c.a[0] == 0x12) ? 1 : 0;

}



static uint32_t htonl(uint32_t val)

{

	uint8_t *ptr = (uint8_t *)&val;

	uint32_t data = val;

	if(!is_BigEnd())

	{

		*(ptr + 0) = ((data >> 24) & 0xff);

		*(ptr + 1) = ((data >> 16) & 0xff);

		*(ptr + 2) = ((data >> 8) & 0xff);

		*(ptr + 3) = ((data >> 0) & 0xff);

	}

	return val;

}



static uint16_t htons(uint16_t val)

{

	uint8_t *ptr = (uint8_t *)&val;

	uint16_t data = val;

	if(!is_BigEnd())

	{

		*(ptr + 0) = ((data >> 8) & 0xff);

		*(ptr + 1) = ((data >> 0) & 0xff);

	}

	return val;

}



// 任务入口
// Absolute time log (ms, system time)
// 绝对时间打印（毫秒，系统时间）

void speechMainProc() { speechRecv(); }

void speechXunFeiMainProc() { speechXunfeiRecv(); }

void speechRecv() { speechOfpRecv(); }

void speechOfpRecv() { recv_blk_dtms_dtps_047(); }



//按键监听

void recv_blk_dtms_dtps_047()

{

	static unsigned short lastFlag = 0;



	if(blk_dtms_dtps_047.control != lastFlag)

	{

		blk_ccc_xunfei_001.PDU_ID = htonl(0x06000C01);

		blk_ccc_xunfei_001.PDU_length = htonl(0x3);

		blk_ccc_xunfei_001.sourceType = 0x5;

		blk_ccc_xunfei_001.recgMode = 0x0;



		// 按下

		if(blk_dtms_dtps_047.control == 0x5f5f)

		{

			blk_ccc_xunfei_001.recgState = 0x1;

			memset(g_pcie_buffer_send,0,sizeof(g_pcie_buffer_send));

			memcpy(g_pcie_buffer_send, &blk_ccc_xunfei_001,  sizeof(blk_ccc_xunfei_001));

			sndCardPutdata(g_pcie_buffer_send,  sizeof(blk_ccc_xunfei_001));



			// 极简打印

			printf("[Manual] Key Press\n");

		}

		// 松开

		else if(blk_dtms_dtps_047.control == 0xaaaa)

		{

			blk_ccc_xunfei_001.recgState = 0x2;

			memset(g_pcie_buffer_send,0,sizeof(g_pcie_buffer_send));

			memcpy(g_pcie_buffer_send, &blk_ccc_xunfei_001, sizeof(blk_ccc_xunfei_001));

			sndCardPutdata(g_pcie_buffer_send,  sizeof(blk_ccc_xunfei_001));



			// 极简打印

			printf("[Manual] Key Release\n");

		}

		lastFlag = blk_dtms_dtps_047.control;

	}

}



//  接收主任务

void speechXunfeiRecv()

{

	INT32 actualLen;

	static SYSTEM_TIME_TYPE timeStart = 0;



	static unsigned int test_cnt = 0;

	int ret = 0;



	SYSTEM_TIME_TYPE timeNow;

	RETURN_CODE_TYPE retT;



	memset(g_pcie_buffer, 0, sizeof(g_pcie_buffer));



	// 上电初始化

	{

		static int init_reset_flag = 0;

		if(init_reset_flag == 0)

		{

			memset(&blk_dtps_dtms_043, 0, sizeof(blk_dtps_dtms_043));

			blk_dtps_dtms_043.voiceRecognizeSta = 3;

			send_blk_dtps_dtms_043();

			init_reset_flag = 1;

		}

	}



	// 压测脚本 (#if 0 关闭)

#if 0

	test_cnt++;

	if(test_cnt >= 200) test_cnt = 0;

	if(test_cnt == 1) { // 模拟按下

		blk_ccc_xunfei_001.recgState = 0x1;

		blk_ccc_xunfei_001.PDU_ID = htonl(0x06000C01);

		blk_ccc_xunfei_001.PDU_length = htonl(0x3);

		blk_ccc_xunfei_001.sourceType = 0x5;

		blk_ccc_xunfei_001.recgMode = 0x0;

		memset(g_pcie_buffer_send, 0, sizeof(g_pcie_buffer_send));

		memcpy(g_pcie_buffer_send, &blk_ccc_xunfei_001, sizeof(blk_ccc_xunfei_001));

		sndCardPutdata(g_pcie_buffer_send, sizeof(blk_ccc_xunfei_001));

		printf("[Script] Press\n");

	} else if(test_cnt == 80) { // 模拟松开

		blk_ccc_xunfei_001.recgState = 0x2;

		blk_ccc_xunfei_001.PDU_ID = htonl(0x06000C01);

		blk_ccc_xunfei_001.PDU_length = htonl(0x3);

		blk_ccc_xunfei_001.sourceType = 0x5;

		blk_ccc_xunfei_001.recgMode = 0x0;

		memset(g_pcie_buffer_send, 0, sizeof(g_pcie_buffer_send));

		memcpy(g_pcie_buffer_send, &blk_ccc_xunfei_001, sizeof(blk_ccc_xunfei_001));

		sndCardPutdata(g_pcie_buffer_send, sizeof(blk_ccc_xunfei_001));

		printf("[Script] Release\n");

	}

#endif



	ret = sndCardGetdata(g_pcie_buffer, MAXPCIELEN, &actualLen, 10);





	if(timeStart > 0)

	{

		GET_TIME(&timeNow, &retT);

		if((timeNow - timeStart) > 1000000000LL)

		{

			printf("[TIME] Fatal Timeout (>5s). Reset.\n");

			blk_xunfei_ccc_002.recgState = 0x03;

			memset(&blk_dtps_dtms_043, 0, sizeof(blk_dtps_dtms_043));

			blk_dtps_dtms_043.voiceRecognizeSta = 3;

			send_blk_dtps_dtms_043();

			timeStart = 0;

		}

	}



	if (ret == 0) {



		if ((g_pcie_buffer[3] == 0x00))

		{

			// 心跳不打印

		}

		else if ((g_pcie_buffer[3] == 0x02))

		{

			memset(&blk_xunfei_ccc_002, 0, sizeof(blk_xunfei_ccc_002));

			memcpy(&blk_xunfei_ccc_002, &g_pcie_buffer, sizeof(g_pcie_buffer));



			// 0x01: 录音中 (重置计时)

			if(blk_xunfei_ccc_002.recgState == 0x01) {

				timeStart = 0;



				memset(&blk_dtps_dtms_043, 0, sizeof(blk_dtps_dtms_043));

				blk_dtps_dtms_043.voiceRecognizeSta = 1;

				send_blk_dtps_dtms_043();

			}

			// 0x02: 识别中 (开始计时)

			else if(blk_xunfei_ccc_002.recgState == 0x02) {

				GET_TIME(&timeStart, &retT);


			}

		}

		else if ((g_pcie_buffer[3] == 0x03))

		{
			timeStart = 0;
			blk_xunfei_ccc_002.recgState = 0x03;
			speechProc();
		}

		else

		{

			// 异常包仅打印头

			printf("Recv Unknown 0x%02x\n", g_pcie_buffer[3]);

		}

	}

}



void speechProc() {

	//testCasesProc(); // 模拟数据









	CMD_ICD_STRUCT *icd = searsh_result((char *) &g_pcie_buffer[15]);



	if (icd != NULL) {

		memcpy(&blk_dtps_dtms_043, icd, sizeof(blk_dtps_dtms_043));

		send_blk_dtps_dtms_043();




		printf("Res:%s\n", icd->voiceIdentify);

	} else {

		blk_dtps_dtms_043.voiceRecognizeSta = 3;

		send_blk_dtps_dtms_043();




		printf("Not found\n");

	}

}



void send_blk_dtps_dtms_043() {

	sendVoiceMsgToDtms((unsigned char*) &blk_dtps_dtms_043, sizeof(BLK_DTPS_DTMS_043));

}



// 测试用例

void testCasesProc() {

	static unsigned int procTime = 0;

	int i = 0;

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "MFD1打开地图"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "MFD2进入协同任务"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "MFD1切换编队管理"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机1磁探伸出"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机1磁探收回"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机1光电扫描"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机2光电锁定"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机1搜索任务区1"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机1跟踪目标1"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机1磁探搜索任务区1"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机1磁探跟踪目标1"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机1浮标侦听任务区1"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机2搜索任务区2"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机2跟踪目标999"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机2磁探搜索任务区2"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机2磁探跟踪目标1"); procTime++; return; }

	if (i++ == procTime) { sprintf((char*) &g_pcie_buffer[15], "%s", "无人机2浮标侦听任务区2"); procTime++; return; }

	procTime = 0;

}

