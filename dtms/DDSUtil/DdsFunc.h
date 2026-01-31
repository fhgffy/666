#ifndef DDSFUNC_H
#define DDSFUNC_H

#include "SixIaseComponentConfig.h"
#include "SixIaseApplicationService.h"

#ifdef __cplusplus
extern "C" {
#endif


#pragma pack(1)

extern int arrPeriodTimeout[30]; //周期消息超时列表

// 初始化周期消息列表
void initPeroidList();

// 筛选周期发送的消息，是返回true，反之false
int isPeriodMessage(CONNECTION_ID_TYPE connection_id, int** timeoutFlag);

/**
 * @brief Send_Message_Local 封装发送函数，方便添加个性化操作
 */
void Send_Message_Local(
        /* in    */	 CONNECTION_ID_TYPE 	connection_id,
        /* in    */	 TIMEOUT_TYPE 			timeout,
        /* inout */  TRANSACTION_ID_TYPE 	*transaction_id,
        /* inout */  void                	*message,
        /* in    */	 MESSAGE_TYPE_GUID 		*message_type_id,
        /* in    */  MESSAGE_SIZE_TYPE 		message_size,
        /* out   */	 RETURN_CODE_TYPE 		*return_code
        );

/**
 * @brief Receive_Message_Local 封装接收函数，方便添加个性化操作
 */
int Receive_Message_Local
(
/* in    */	 CONNECTION_ID_TYPE 	connection_id,
/* in    */	 TIMEOUT_TYPE 			timeout,
/* inout */  TRANSACTION_ID_TYPE 	*transaction_id,
/* inout */  void                	*message,
/* inout */	 MESSAGE_TYPE_GUID 		*message_type_id,
/* inout */	 MESSAGE_SIZE_TYPE 		*message_size,
/* out   */  RETURN_CODE_TYPE 		*return_code
);

/**
 * @brief Receive_Message_Local 封装接收函数，方便添加个性化操作
 */
int Receive_Message_Period
(
/* in    */	 CONNECTION_ID_TYPE 	connection_id,
/* in    */	 TIMEOUT_TYPE 			timeout,
/* inout */  TRANSACTION_ID_TYPE 	*transaction_id,
/* inout */  void                	*message,
/* inout */	 MESSAGE_TYPE_GUID 		*message_type_id,
/* inout */	 MESSAGE_SIZE_TYPE 		*message_size,
/* out   */  RETURN_CODE_TYPE 		*return_code
);

/**
 * @brief Receive_Message_Local
 */
int Receive_Message_Period_2
(
/* in    */	 CONNECTION_ID_TYPE 	connection_id,
/* in    */	 TIMEOUT_TYPE 			timeout,
/* inout */  TRANSACTION_ID_TYPE 	*transaction_id,
/* inout */  void                	*message,
/* inout */	 MESSAGE_TYPE_GUID 		*message_type_id,
/* inout */	 MESSAGE_SIZE_TYPE 		*message_size,
/* out   */  RETURN_CODE_TYPE 		*return_code
);

/**
 * @brief Send_Message_To_MMM_Proc 发送已发消息给MMM总入口
 */
void Send_Message_To_MMM_Proc(
        /* in    */	 CONNECTION_ID_TYPE 	connection_id,
        /* in    */	 TIMEOUT_TYPE 			timeout,
        /* inout */  TRANSACTION_ID_TYPE 	*transaction_id,
        /* inout */  void                	*message,
        /* in    */	 MESSAGE_TYPE_GUID 		*message_type_id,
        /* in    */  MESSAGE_SIZE_TYPE 		message_size,
        /* out   */	 RETURN_CODE_TYPE 		*return_code
        );

/**
 * @brief Send_Message_To_MMM 发送已发消息给MMM单独消息处理
 */
void Send_Message_To_MMM(
        /* in    */	 CONNECTION_ID_TYPE 	connection_id,
        /* in    */	 TIMEOUT_TYPE 			timeout,
        /* inout */  TRANSACTION_ID_TYPE 	*transaction_id,
        /* inout */  void                	*message,
        /* in    */	 MESSAGE_TYPE_GUID 		*message_type_id,
        /* in    */  MESSAGE_SIZE_TYPE 		message_size,
        /* out   */	 RETURN_CODE_TYPE 		*return_code
        );


#pragma pack()

#ifdef __cplusplus
}
#endif

#endif // DDSFUNC_H
