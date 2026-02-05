#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "voiceToText.h"
#include "paramConfig.h"

/**
 * @brief 初始化哈希表，将所有桶指针置为NULL
 * @param table 哈希表指针
 */
void hash_table_init(HashTable *table) {
	if (table == NULL) {
		printf("Error: HashTable pointer is NULL\n");
		return;
	}
	memset(table->buckets, 0, sizeof(table->buckets));
}

/**
 * @brief djb2哈希算法，对字符串进行哈希，返回哈希值
 * @param str 输入字符串
 * @return 哈希值
 */
unsigned int djb2_hash(const char *str) {
    unsigned int hash = 5381;
    int c;
    while ((c = *str++))
        hash = ((hash << 5) + hash) + c;  // hash * 33 + c
    return hash;
}
/**
 * @brief 在哈希表中查找指定key的节点，并返回其结构体内容
 * @param table 哈希表指针
 * @param key   要查找的字符串
 * @param result [输出] 查找到时存放结果
 * @return 1：查找成功；0：未找到
 */
int hash_table_lookup(const HashTable *table, const char *key, CMDNode *result) {
    unsigned int index = djb2_hash(key) % TABLE_SIZE;
    HashNode *node = table->buckets[index]; // 将变量声明移到循环外部
    while (node != NULL) {
        if (strcmp(node->meta.cmd.voiceIdentify, key) == 0) {
            *result = node->meta;
            return 1;  // 成功匹配
        }
        node = node->next;
    }
    return 0;  // 未匹配
}

/**
 * @brief 向哈希表插入一个key对应的数据节点
 * @param table 哈希表指针
 * @param key   作为索引的字符串
 * @param meta  要插入的数据结构体
 */
void hash_table_insert(HashTable *table, const char *key, CMDNode meta) {
    unsigned int index = djb2_hash(key) % TABLE_SIZE;
    HashNode *node = (HashNode *)malloc(sizeof(HashNode));
    if (node == NULL) {
        printf("Memory allocation failed\n");
        return;
    }
    strncpy(node->meta.cmd.voiceIdentify, key, sizeof(node->meta.cmd.voiceIdentify) - 1);
    node->meta.cmd.voiceIdentify[sizeof(node->meta.cmd.voiceIdentify) - 1] = '\0';
    node->meta = meta;
    node->next = table->buckets[index];
    table->buckets[index] = node;
}

/**
 * @brief 通过处理输入字符串，查找与之全词匹配的命令结构体。
 *
 * 功能说明：
 * - 先初始化一个哈希表，将 configParm 数组中的命令信息插入哈希表。
 * - 使用 processString 处理输入字符串，规范化之后查找哈希表。
 * - 如果找到匹配项，返回对应的命令结构体指针；否则返回 NULL。
 *
 * @param input  输入的字符串（例如语音识别结果）。
 * @return CMD_ICD_STRUCT*  匹配到的命令结构体指针，未找到则返回 NULL。返回指针为静态局部变量，调用者无需释放。
 */
CMD_ICD_STRUCT* searsh_result(char *input) {
	static CMD_ICD_STRUCT ret_icd;
	static HashTable table;
	static int inited = 0;
    // 将 configParm 数组中的数据插入到哈希表中
	if (!inited) {
	   hash_table_init(&table);   // 只初始化一次
	   int i;
	   int num_entries = sizeof(configParm) / sizeof(configParm[0]);
	   for (i = 0; i < num_entries; i++) {
		   hash_table_insert(&table, configParm[i].cmd.voiceIdentify, configParm[i]);
	   }
	   inited = 1;
	}
    CMDNode result;
    char output[100] = {0}; // 用于保存最终结果
    processString(input, output);
    if (hash_table_lookup(&table, output, &result)) {
    	has_number_Proc(input, &result);// 可变数字特殊赋值
        ret_icd = result.cmd; // 结构体直接赋值
        return &ret_icd;
    } else {
    	return NULL;
    }
}


/**
 * @brief 检查输入字符串 input 的末尾是否为指定的2字中文词 word。
 *
 * @param input 输入字符串。
 * @param word  指定的2字中文词（需要与 input 的末尾对齐比较）。
 * @return int  如果 input 以 word 结尾，返回 1；否则返回 0。
 */
int ends_with(const char* input, const char* word) {
    int len = strlen(input);
    int wlen = strlen(word);
    if (len < wlen) return 0;
    return (strcmp(input + len - wlen, word) == 0);
}
/**
 * @brief 处理输入字符串，从末尾寻找数字与特定中文词（如“分钟”或“目标”）并格式化输出。
 *
 * 功能说明：
 * - 如果 input 以“分钟”或“目标”结尾，则提取这两个字前面的第一个连续数字，并将数字前的内容与后缀保留，中间用六个英文点“......”隔开。
 * - 如果 input 末尾是数字，则提取末尾的连续数字段，将其前面的内容与数字用“......”隔开。
 * - 如果末尾没有数字，也没有特定后缀，则输出原内容。
 *
 * @param input   输入字符串。
 * @param output  输出字符串，格式化后存储。
 * @param number  提取到的第一个数字字符串。
 */
void processString(const char *input, char *output) {
    int len = strlen(input);
    int i = len - 1;
	if (ends_with(input, "分钟") || ends_with(input, "目标")) {
		int suffix_len = 4; // GBK编码下“分钟”或“目标”是4字节
		int i = len - suffix_len - 1;
		// Step 1: 从“分钟”前往前找到第一个数字
		while (i >= 0 && !isdigit((unsigned char)input[i])) i--;
		if (i < 0) {
			strcpy(output, input); // 没有数字则原样输出
			return;
		}
		// Step 2: 从这个数字往前找到一段连续数字
		while (i >= 0 && isdigit((unsigned char)input[i])) i--;
		int num_start = i + 1;

		// 拼接output
		// 1. 保留number前的所有内容
		strncpy(output, input, num_start);
		output[num_start] = '\0';
		// 2. 拼接六个英文点
		strcat(output, "......");
		// 3. 拼接“分钟”或“目标”
		strcat(output, input + len - suffix_len);
		return;
	}

   // 从末尾向前，找到第一个连续的数字段
	while (i >= 0 && isdigit((unsigned char)input[i])) {
	  i--;
	}
	int num_start = i + 1; // 数字段的起始下标
	int num_len = len - num_start;
	if (num_len > 0) {
	  // before部分：数字段前面的内容
	  strncpy(output, input, num_start);
	  output[num_start] = '\0';
	  // 拼接六个省略号
	  strcat(output, "......");
	} else {
	  // 没有末尾数字，则输出原内容
	  strcpy(output, input);
	}
}

/**
 * // 当指令中含有可变数组的时候赋值的特殊处理
 * input:原始输入字符串；
 * p_CMDNode:识别后的指令
 */

void has_number_Proc(const char *input,CMDNode* p_CMDNode)
{
	// 非可变参数的不处理
	if(!p_CMDNode->hasNumber)
		return;


	// 如果是
	if(strcmp((const char *)p_CMDNode->cmd.voiceIdentify, "无人机1搜索任务区......") == 0 || \
	   strcmp((const char *)p_CMDNode->cmd.voiceIdentify, "无人机2搜索任务区......") == 0	 || \
	   strcmp((const char *)p_CMDNode->cmd.voiceIdentify, "无人机1磁探搜索任务区......") == 0	 || \
	   strcmp((const char *)p_CMDNode->cmd.voiceIdentify, "无人机2磁探搜索任务区......") == 0	 || \
	   strcmp((const char *)p_CMDNode->cmd.voiceIdentify, "无人机1浮标侦听任务区......") == 0	 || \
	   strcmp((const char *)p_CMDNode->cmd.voiceIdentify, "无人机2浮标侦听任务区......") == 0)
	{
		int temNum = has_number_get_tail_num(input);

		if(temNum>7 || temNum<1)
		{
			temNum = 1;
		}

		p_CMDNode->cmd.STPTaskNumber = temNum;

	}

	if(strcmp((const char *)p_CMDNode->cmd.voiceIdentify, "无人机1跟踪目标......") == 0 || \
	   strcmp((const char *)p_CMDNode->cmd.voiceIdentify, "无人机2跟踪目标......") == 0	 || \
	   strcmp((const char *)p_CMDNode->cmd.voiceIdentify, "无人机1磁探跟踪目标......") == 0	 || \
	   strcmp((const char *)p_CMDNode->cmd.voiceIdentify, "无人机2磁探跟踪目标......") == 0)
	{
		int temNum = has_number_get_tail_num(input);

//		if(temNum>50 || temNum<1)
//		{
//			temNum = 1;
//		}

		p_CMDNode->cmd.STPTargetNumber = temNum;

	}


	// 恢复原字符串
	strcpy((char *)p_CMDNode->cmd.voiceIdentify, (const char *)input);
}


int has_number_get_tail_num(const char *input)// 得到末尾连续数字的值
{
	// 找出任务区数字
	int len = strlen(input);
	int i = len - 1;
	// 从末尾向前，找到第一个连续的数字段
	while (i >= 0 && isdigit((unsigned char)input[i])) {
	  i--;
	}
	int num_start = i + 1; // 数字段的起始下标
	int num_len = len - num_start;
	int resNum = 0;
	int levelNum = 1;// 位数
	int j=0;
	while (num_len-- > 0) {
		//最后开始是个位
		int temNum = 0;
		temNum = input[len-1-j] - '0';
		resNum = temNum * levelNum + resNum;
		  // before部分：数字段前面的内容

		// 每次位数乘10
		levelNum *= 10;
	}

	return resNum;
}






