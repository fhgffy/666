#ifndef __ICD_H__
#define __ICD_H__


// 基本数据类型及布尔值定义
typedef void            VOID;
typedef bool            BOOL;
typedef char            CHAR;
typedef char            I8;
typedef unsigned char   U8;
typedef short           I16;
typedef unsigned short  U16;
typedef int             I32;
typedef unsigned int    U32;
typedef long            I64;
typedef unsigned long   U64;
typedef float           F32;
typedef double          F64;
typedef bool            EMPTY;

// 高速业务数据包同步头1、2、3、4
#define TASK_SYNC_HEAD1  0x55
#define TASK_SYNC_HEAD2  0xAA
#define TASK_SYNC_HEAD3  0x1C
#define TASK_SYNC_HEAD4  0x1C

// 数据包内容长度最大值
#define PACKET_BODY_LEN     38388
// 单元数据发送最大长度
#define UNIT_SEND_BUF_LEN   38412

// 高速业务数据包头
#pragma pack(1)
typedef struct _TaskPacketHead
{
    U8  syncHead1;          // 同步字：0x55AA1C1C
    U8  syncHead2;
    U8  syncHead3;
    U8  syncHead4;
    U16 loop;               // 包计数：从0开始循环计数
    U16 packetLen;          // 包长度：12（1类包，空包）、8640（2类包）、18600（3类包）、28600（4类包）、38400（5类包）
    U16 packetAttr;         // 包属性：D15～D8宝属性：0x01-1类包、0x02-2类包、0x03-3类包、0x04-4类包、0x06-5类包
//    U8 UAVID;
    U16 firstUnitOffset;    // 第一个单元偏移量（本包中第一个单元的首地址，从包头的同步字开始按照字节计数），单元起始必须对齐到4字节，多余部分使用空闲(0x99)填充
}STaskPacketHead;

// 高速业务数据包
typedef struct _TaskPacket
{
    STaskPacketHead packetHead;     // 业务数据包头
    U8  dataArray[PACKET_BODY_LEN]; // 数据内容:若干单元+若干空闲
}STaskPacket;

// 高速业务数据单元属性字节比特位定义
typedef union _TaskAttr
{
    // 字变量
    U16 value;
    // 位结构体变量
    struct
    {
        U16 reserve1     : 7;   // 保留
        U16 nextUnitFlag : 1;   // 下一单元有效：0-本包最后一个单元
        U16 reserve2     : 6;   // 保留
        U16 endFlag      : 1;   // 帧内结束单元：1-结束
        U16 beginFlag    : 1;   // 帧内起始单元：1-起始
    }bits;
}UTaskAttr;

// 高速业务数据单元
typedef struct _TaskUnitHead
{
    U8  mainType;           // 主类型
    U8  minorType;          // 次类型
    U16 frameLoop;          // 所属帧帧计数
    U16 numInFrame;         // 帧内编号
    UTaskAttr unitAttr;     // 单元属性
    U16 unitLoop;           // 单元计数（同一种类）：从0开始循环计数
    U16 dataLen;            // 单元有效数据长度(单元内有效数据的字节数)
    U8  checkcode;          // 单元校验（单元数据的异或校验结果）
    U8  reserve;            // 预留
    U16 nextUnitOffset;     // 下一个单元偏移量（本包中下一个单元的首地址，从包头的同步字开始按照字节计数）
    U8  dataArray[0];       // 单元有效数据(最大长度34792)
}STaskUnitHead;
#pragma pack()


#endif // __ICD_H__
