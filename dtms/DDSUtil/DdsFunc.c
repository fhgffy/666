#include "DdsFunc.h"
#include "DDSUtil.h"
#include "icd_ccc_mmm.h"

#define PERIODTIMEOUTCOUNT 3 // 周期消息未收到20次则认为超时
#define MAXPERIODNUM  30

/*******收mids时间消息*************/
extern BLK_MIDS_CCC_TIME blk_mids_ccc_time;

/*
 * DDS发送信息
 * */
int arrPeriodTimeout[MAXPERIODNUM]= {0};// 周期消息超时标志数组
int arrPeriodList[MAXPERIODNUM] = {0};


/**
 * 初始化周期消息列表
 */
void initPeroidList()
{
	int index  = 0;
	// c链遥测数据
	arrPeriodList[index++] = DDSTables.KKL_CCC_0.niConnectionId;
	arrPeriodList[index++] = DDSTables.KKL_CCC_6.niConnectionId;
	arrPeriodList[index++] = DDSTables.KKL_CCC_8.niConnectionId;
	arrPeriodList[index++] = DDSTables.KKL_CCC_9.niConnectionId;
	// u链遥测数据
	arrPeriodList[index++] = DDSTables.KKL_U_CCC_03.niConnectionId;
	arrPeriodList[index++] = DDSTables.KKL_U_CCC_04.niConnectionId;
	arrPeriodList[index++] = DDSTables.KKL_U_CCC_05.niConnectionId;
	arrPeriodList[index++] = DDSTables.KKL_U_CCC_06.niConnectionId;
}


// 筛选周期接收的消息，是返回true，反之false
int isPeriodMessage(CONNECTION_ID_TYPE connection_id, int** ppTimeoutFlag)
{
    // 只做一次列表初始化
	static unsigned int initTime = 1;
	if(initTime-->0)
	{
		initPeroidList();
	}


	for(int i=0; i< MAXPERIODNUM; i++)
	{
		if(connection_id == arrPeriodList[i])
		{
			// 将外部超时指针的值改为相应的超时数地址
			*ppTimeoutFlag = &arrPeriodTimeout[i];
			return 1;
		}
	}


    return 0;

}

/**
 * @brief Send_Message_Local 封装发送函数，方便添加个性化操作
 */
void Send_Message_Local(CONNECTION_ID_TYPE connection_id, TIMEOUT_TYPE timeout, TRANSACTION_ID_TYPE *transaction_id, void *message, MESSAGE_TYPE_GUID *message_type_id, MESSAGE_SIZE_TYPE message_size, RETURN_CODE_TYPE *return_code)
{
    Send_Message(connection_id, timeout, transaction_id, message, message_type_id, message_size, return_code);

    // 发送MMM处理
    Send_Message_To_MMM_Proc(connection_id, timeout, transaction_id, message, message_type_id, message_size, return_code);

}

/**
 * @brief Receive_Message_Local 封装接收函数，方便添加个性化操作
 * @return 仅在周期消息时有意义。为了尽可能少的改动
 * 0:周期消息未超时，非零：周期消息超时。
 */
int Receive_Message_Local(CONNECTION_ID_TYPE connection_id, TIMEOUT_TYPE timeout, TRANSACTION_ID_TYPE *transaction_id, void *message, MESSAGE_TYPE_GUID *message_type_id, MESSAGE_SIZE_TYPE *message_size, RETURN_CODE_TYPE *return_code)
{
    // 筛选周期发送的消息，如果是周期发送的消息，因为会累积缓冲区，造成报错，所以循环取空，其余的，则直接区就行

    int* pTimeoutFlag = 0;
    if(!isPeriodMessage(connection_id, &pTimeoutFlag))// 非周期消息则正常取
    {
        Receive_Message(connection_id, timeout, transaction_id, message, message_type_id, message_size, return_code);
        return 0;

    }

    // 周期消息处理
    return Receive_Message_Period_2(connection_id, timeout, transaction_id, message, message_type_id, message_size, return_code);
}

int Receive_Message_Period(CONNECTION_ID_TYPE connection_id, TIMEOUT_TYPE timeout, TRANSACTION_ID_TYPE *transaction_id, void *message, MESSAGE_TYPE_GUID *message_type_id, MESSAGE_SIZE_TYPE *message_size, RETURN_CODE_TYPE *return_code)
{
    RETURN_CODE_TYPE return_code_flag = INVALID_PARAM ;
    int *pTimeoutFlag = 0;// 用这个指针通过isPeriodMessage取出超时标志
    isPeriodMessage(connection_id, &pTimeoutFlag);// 更新timeoutFlag

    do
    {
        Receive_Message(connection_id, timeout, transaction_id, message, message_type_id, message_size, return_code);

        if(*return_code == NO_ERROR)
        {

            return_code_flag = NO_ERROR;
        }
    }
    while(*return_code == NO_ERROR);

    if(return_code_flag == NO_ERROR)
    {
        *return_code = NO_ERROR;
        *pTimeoutFlag = 0;// 超时标志清空
    }
    else
    {
        (*pTimeoutFlag)++;// 超时标志自增

        // 判断超时
        if(*pTimeoutFlag > PERIODTIMEOUTCOUNT)
            return 1;
    }

    return 0;
}


int Receive_Message_Period_2(CONNECTION_ID_TYPE connection_id, TIMEOUT_TYPE timeout, TRANSACTION_ID_TYPE *transaction_id, void *message, MESSAGE_TYPE_GUID *message_type_id, MESSAGE_SIZE_TYPE *message_size, RETURN_CODE_TYPE *return_code)
{
    RETURN_CODE_TYPE return_code_flag = INVALID_PARAM ;
    int *pTimeoutFlag = 0;// 用这个指针通过isPeriodMessage取出超时标志
    isPeriodMessage(connection_id, &pTimeoutFlag);// 更新timeoutFlag

    Receive_Message(connection_id, timeout, transaction_id, message, message_type_id, message_size, return_code);

	if(*return_code == NO_ERROR)
	{

		return_code_flag = NO_ERROR;
	}

    if(return_code_flag == NO_ERROR)
    {
        *return_code = NO_ERROR;
        *pTimeoutFlag = 0;// 超时标志清空
    }
    else
    {
        (*pTimeoutFlag)++;// 超时标志自增

        // 判断超时
        if(*pTimeoutFlag > PERIODTIMEOUTCOUNT)
            return 1;
    }

    return 0;
}
/**
 * @brief Send_Message_To_MMM_Proc 发送已发消息给MMM总入口
 */
void Send_Message_To_MMM_Proc(CONNECTION_ID_TYPE connection_id, TIMEOUT_TYPE timeout, TRANSACTION_ID_TYPE *transaction_id, void *message, MESSAGE_TYPE_GUID *message_type_id, MESSAGE_SIZE_TYPE message_size, RETURN_CODE_TYPE *return_code)
{
	unsigned int uiTopicId = 0;

    int TableMapNum = sizeof(DDSTables) / sizeof(DDSTableMap);
    DDSTableMap *DDSTablesPoint = &DDSTables;// 当数组用

	for(int i = 0; i< TableMapNum;i++, DDSTablesPoint++)
	{
		// 暂时屏蔽内部消息，不记录
		if(DDSTables.BLK_DTMS_CTAS_001.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.BLK_DTMS_CTAS_002.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.BLK_DTMS_CTAS_003.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.BLK_DTMS_CTAS_004.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.BLK_DTMS_CTAS_005.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.BLK_DTMS_CTAS_007.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.BLK_DTMS_CTAS_008.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.BLK_DTMS_CTAS_009.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}


		if(DDSTables.DTMS_DTRP_0.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.DTMS_DTRP_1.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.DTMS_DTRP_2.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}



		if(DDSTables.CCC_PAD_001.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_002.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_003.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_012.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_013.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_014.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}


		if(DDSTables.CCC_PAD_015.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_017.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_019.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_018.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_022.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_023.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_024.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_032.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_033.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_034.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_035.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_036.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_037.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_038.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_039.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_040.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_041.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_042.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_127.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_139.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_198.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_199.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_200.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_201.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_202.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_302.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_403.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_025.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_020.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}


		if(DDSTables.CCC_PAD_005.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_006.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_007.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}

		if(DDSTables.CCC_PAD_777.niConnectionId == connection_id)//全局战术战法推荐指令
		{
		    return;
		}


		if(DDSTablesPoint->niConnectionId == connection_id)
		{
			uiTopicId = DDSTablesPoint->uiTopicId;
			Send_Message_To_MMM(uiTopicId, timeout, transaction_id, message, message_type_id, message_size, return_code);
		}
	}
}

/**
 * @brief Send_Message_To_MMM 发送已发消息给MMM单独消息处理
 */
void Send_Message_To_MMM(CONNECTION_ID_TYPE connection_id, TIMEOUT_TYPE timeout, TRANSACTION_ID_TYPE *transaction_id, void *message, MESSAGE_TYPE_GUID *message_type_id, MESSAGE_SIZE_TYPE message_size, RETURN_CODE_TYPE *return_code)
{
//    除内部消息外都发送给MMM

	// 初始化
    int cpyLength = 0;// 本次组包长度
    static char MMMMessage[4096];
    // 每次清空
    memset(MMMMessage, 0, sizeof(MMMMessage));

    //组建MMM包
    BLK_CCC_MMM_001 blk_ccc_mmm_001_tem;
    blk_ccc_mmm_001_tem.head = 0xAAAAAAAA;
    blk_ccc_mmm_001_tem.day = blk_mids_ccc_time.fc_date.day;
    blk_ccc_mmm_001_tem.month = blk_mids_ccc_time.fc_date.month;
    blk_ccc_mmm_001_tem.year = blk_mids_ccc_time.fc_date.year;
    blk_ccc_mmm_001_tem.msc = blk_mids_ccc_time.fc_time.hour*60*60*1000 + \
    		blk_mids_ccc_time.fc_time.minute*60*1000 + \
    		blk_mids_ccc_time.fc_time.second*1000 + \
    		blk_mids_ccc_time.fc_time.millisecond;
    blk_ccc_mmm_001_tem.msgId = connection_id;
    blk_ccc_mmm_001_tem.srcId = 40; //确认十进制
    // 长度是3字节，则直接取低位3字节。(低地址存数据的高位是大端。低地址存数据的地位是小端)
    memcpy(&blk_ccc_mmm_001_tem.length[0], (char*)&message_size, sizeof(blk_ccc_mmm_001_tem.length));

    // 组建发送包
    memcpy(MMMMessage, &blk_ccc_mmm_001_tem, sizeof(blk_ccc_mmm_001_tem));
    cpyLength = sizeof(blk_ccc_mmm_001_tem);
    // 发送消息数据拷贝
    memcpy(MMMMessage+sizeof(blk_ccc_mmm_001_tem), message, message_size);
    cpyLength = cpyLength + message_size;
    // 消息尾拷贝
    unsigned int tail;// 0xFFFFFFFF
    tail = 0xFFFFFFFF;
    memcpy(MMMMessage + cpyLength, &tail, sizeof(tail) );
    cpyLength = cpyLength + sizeof(tail);

    Send_Message(DDSTables.CCC_MMM_001.niConnectionId, timeout, transaction_id, MMMMessage, message_type_id, cpyLength, return_code);

}
