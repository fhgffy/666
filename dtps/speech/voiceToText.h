#ifndef __VOICETOTEXT_H__
#define __VOICETOTEXT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#pragma pack(1)

typedef struct CMD_ICD
{
    unsigned short identify_state; // 0-NA；1-识别中；2-识别成功；3-识别失败
    unsigned short isACK;          // 0-不需要应答；1-需要应答；
    unsigned short cmdSeat;        // 0-主驾驶；1-副驾驶；2-PAD前；3-PAD后
    unsigned short ConfirmState;   // 0-NA；1-指令已确认执行；2-指令取消执行
    unsigned short cmdType;        // 0-N/A 1-页面调显（直接执行）2-主模式切换（需确认）3-页面切换（直接执行）等
    unsigned short pageDisType;    // 0-N/A 1-垂直 2-综合 3-机电等
    unsigned short pageDisIndex;   // 0-N/A  1 - MFD1 ，2 - MFD2 ，3 - MFD3 ，4 - MFD4
    unsigned short modeSwitch;     // 0-N/A 1-反潜模式 2-反舰模式等
    unsigned short pageSwitchType; // 0-N/A 1-本机稳瞄 2-本机攻击等
    unsigned short pageSwitchUAVNumber; // 页面切换：无人机编队内编号
    unsigned short devControl;     // 0-NA；1-本机激光上电；2-本机稳瞄上电等
    unsigned short devControlUAVNumber; // 设备控制：无人机编队内编号
    unsigned short param;          // 参数设置：0-N/A 1~ 26-预留等
    unsigned short paramUAVNumber; // 参数设置：无人机编队内编号
    unsigned short weaponType;     // 武器类型:1-AKF9A 2-AKF9B等
    unsigned short weaponPlace;    // 参数设置：武器占位:1-9
    unsigned short laserCoding;    // 参数设置：激光编码:LSB=1
    unsigned short targetNumber;   // 参数设置：目标编号
    unsigned short co_taskType;    // 协同任务页面显示控制：类型:0-N/A  1 -以有人机为中心等
    unsigned short co_taskUAVNumber; // 协同任务页面显示控制：无人机编队内编号
    unsigned short SingleTaskPlane; // 单机任务规划：0-N/A 1-无人机X搜索任务区Y等
    unsigned short STPUAVNumber;    // 单机任务规划：无人机编队内编号
    unsigned int STPTaskNumber;    // 单机任务规划：任务区编号
    unsigned int STPTargetNumber;  // 单机任务规划：目标编号
    unsigned short wayPointNumber; // 单机任务规划：航路点编号 LSB=1
    unsigned short STPTime;        // 单机任务规划：时间 LSB=1分钟
    unsigned short controlPermit;  // 控制权/编队控制:0-N/A  1-申请无人机Y三级控制权等
    unsigned short CPUAVNumber;    // 控制权/编队控制：无人机编队内编号
    unsigned short CPUAVID;        // 控制权/编队控制：无人机ID（用于加入编队控制）
    unsigned char voiceIdentify[100]; // 语音识别文字
} CMD_ICD_STRUCT;

typedef struct cmd_node {
    int category_id;   // 类别ID（0~7）
    int cmd_id;        // 命令ID（唯一）
    int hasNumber;     // 1：含有可变数字； 0：完全匹配
    CMD_ICD_STRUCT cmd; // icd命令

} CMDNode;

typedef struct HashNode {
    CMDNode meta;      // 匹配到的信息
    struct HashNode *next; // 链地址法处理冲突
} HashNode;

#define TABLE_SIZE 10001  // 取一个质数，减少哈希冲突

typedef struct {
    HashNode *buckets[TABLE_SIZE];
} HashTable;


#pragma pack


void hash_table_init(HashTable *table);
unsigned int djb2_hash(const char *str);
int hash_table_lookup(const HashTable *table, const char *key, CMDNode *result);
void hash_table_insert(HashTable *table, const char *key, CMDNode meta);
void processString(const char *input, char *output);
int ends_with(const char* input, const char* word);
CMD_ICD_STRUCT* searsh_result(char *input) ;
void has_number_Proc(const char *input,CMDNode* p_CMDNode);// 当指令中含有可变数组的时候赋值的特殊处理
int has_number_get_tail_num(const char *input);// 得到末尾连续数字的值


#ifdef __cplusplus
}
#endif

#endif // __VOICETOTEXT_H__
