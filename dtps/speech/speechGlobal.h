#include <sysTypes.h>

#pragma pack(1)

#define MAXPCIELEN  200

// 发给讯飞 1.1心跳
typedef struct
{

	uint32_t pdu_id;  //big endian 低到高顺序0x06000C00
	uint32_t pdu_len; // pdu 负载长度 12
	uint32_t recycle_count;  // 循环计数
	uint16_t DID;  // 版本标识   固定0
	uint8_t software_version_a;  // 软件版本 固定0
	uint8_t software_vesio_xx;
	uint8_t software_vesion_yy;
	uint8_t software_vesion_year;
	uint8_t software_vesion_month;
	uint8_t software_vesion_day;

}BLK_CCC_XUNFEI_000;

// 发给讯飞 1.2 语音识别开/关指令
typedef struct
{

	uint32_t PDU_ID;//big endian 低到高顺序0x06000C01
	uint32_t PDU_length;// 固定3
	uint8_t  sourceType; // 源设备类型0x5
	uint8_t  recgMode; //识别模式1:esr;0:ed
	uint8_t   recgState;// 0x01开始；0x02关闭

}BLK_CCC_XUNFEI_001;



// 发给讯飞 1.3 语音识别开/关响应指令
typedef struct
{

	uint32_t PDU_ID;//big endian 低到高顺序0x06000C02
	uint32_t PDU_length;// 固定3
	uint8_t  sourceType; // 源设备类型0x3
	uint8_t  recgMode; //识别模式1:esr;0:ed
	uint8_t   recgState;// 0x01识别开启；0x02关闭；0x03开启失败

}BLK_XUNFEI_CCC_002;


// 发给讯飞 1.4 语音识别结果
typedef struct
{

	uint32_t PDU_ID;//big endian 低到高顺序0x06000C03
	uint32_t PDU_length;// 数据载荷长度加7
	uint8_t  sourceType; // 源设备类型0x7
	uint32_t index;     // 序列号 从0开始累加
	uint8_t  recgMode; // 识别模式1:esr;0:ed
	uint8_t  recgState;// 0x01无结果；0x02有结果
	uint8_t  text[MAXPCIELEN-15];// 识别结果

}BLK_XUNFEI_CCC_003;



#pragma pack()







extern int ret;
extern UINT8 g_pcie_buffer[MAXPCIELEN];
extern UINT8 g_pcie_buffer_send[MAXPCIELEN];

extern BLK_CCC_XUNFEI_000 blk_ccc_xunfei_000;// 发送心跳
extern BLK_CCC_XUNFEI_001 blk_ccc_xunfei_001;

extern BLK_CCC_XUNFEI_000 blk_xunfei_ccc_001;// 回复心跳
extern BLK_XUNFEI_CCC_002 blk_xunfei_ccc_002;
extern BLK_XUNFEI_CCC_003 blk_xunfei_ccc_003;
