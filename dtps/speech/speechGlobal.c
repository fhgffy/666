#include "speechGlobal.h"

int ret = 0;
UINT8 g_pcie_buffer[MAXPCIELEN]={0};
UINT8 g_pcie_buffer_send[MAXPCIELEN]={0};

BLK_CCC_XUNFEI_000 blk_ccc_xunfei_000;// 发送心跳
BLK_CCC_XUNFEI_001 blk_ccc_xunfei_001;

BLK_CCC_XUNFEI_000 blk_xunfei_ccc_001;// 回复心跳
BLK_XUNFEI_CCC_002 blk_xunfei_ccc_002;
BLK_XUNFEI_CCC_003 blk_xunfei_ccc_003;
