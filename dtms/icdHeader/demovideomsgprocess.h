#ifndef DEMOVIDEOMSGPROCESS_H
#define DEMOVIDEOMSGPROCESS_H
#define TRUE (1)
#define FALSE (0)

// 高速业务数据包同步头1、2、3、4
#define TASK_SYNC_HEAD1  0x55
#define TASK_SYNC_HEAD2  0xAA
#define TASK_SYNC_HEAD3  0x1C
#define TASK_SYNC_HEAD4  0x1C

// 数据包内容长度最大值
#define PACKET_BODY_LEN     38388
// 单元数据发送最大长度
#define UNIT_SEND_BUF_LEN   38412

#define PACKET_TYPE_AMOUNT  5

#define ID8_BROADCAST   (255)       // 8位广播ID
#define ID16_BROADCAST  (65535)     // 16位广播ID

// 设备ID别名
#define devId1  bytes.sysType
#define devId2  bytes.sysNum
#define devId3  bytes.devType
#define devId4  bytes.devNum

#pragma pack(1)
typedef struct _TaskPacketHead
{
    unsigned char  syncHead1;          // 同步字：0x55AA1C1C
    unsigned char  syncHead2;
    unsigned char  syncHead3;
    unsigned char  syncHead4;
    unsigned short loop;               // 包计数：从0开始循环计数
    unsigned short packetLen;          // 包长度：12（1类包，空包）、8640（2类包）、18600（3类包）、28600（4类包）、38400（5类包）
    unsigned short packetAttr;         // 包属性：D15～D8宝属性：0x01-1类包、0x02-2类包、0x03-3类包、0x04-4类包、0x06-5类包
//    unsigned char drone_ID;           //无人机新增接口  20250929
    unsigned short firstUnitOffset;    // 第一个单元偏移量（本包中第一个单元的首地址，从包头的同步字开始按照字节计数），单元起始必须对齐到4字节，多余部分使用空闲(0x99)填充
}TaskPacketHead;

// 高速业务数据包
typedef struct _TaskPacket
{
    TaskPacketHead packetHead;     // 业务数据包头
    unsigned char  dataArray[PACKET_BODY_LEN]; // 数据内容:若干单元+若干空闲
}TaskPacket;

typedef union _TaskAttr
{
    // 字变量
    unsigned short value;
    // 位结构体变量
    struct
    {
        unsigned short reserve1     : 7;   // 保留
        unsigned short nextUnitFlag : 1;   // 下一单元有效：0-本包最后一个单元
        unsigned short reserve2     : 6;   // 保留
        unsigned short endFlag      : 1;   // 帧内结束单元：1-结束
        unsigned short beginFlag    : 1;   // 帧内起始单元：1-起始
    }bits;
}TaskAttr;

typedef struct _TaskUnitHead
{
    unsigned char  mainType;           // 主类型
    unsigned char  minorType;          // 次类型
    unsigned short frameLoop;          // 所属帧帧计数
    unsigned short numInFrame;         // 帧内编号
    TaskAttr unitAttr;     // 单元属性
    unsigned short unitLoop;           // 单元计数（同一种类）：从0开始循环计数
    unsigned short dataLen;            // 单元有效数据长度(单元内有效数据的字节数)
    unsigned char  checkcode;          // 单元校验（单元数据的异或校验结果）
    unsigned char  reserve;            // 预留
    unsigned short nextUnitOffset;     // 下一个单元偏移量（本包中下一个单元的首地址，从包头的同步字开始按照字节计数）
    unsigned char  dataArray[0];       // 单元有效数据(最大长度34792)
}TaskUnitHead;

typedef union _InnerNetFrameAttr
{
    // 属性值
    unsigned char attr;
    // 属性位结构体定义
    struct
    {
        unsigned char priority  : 2;   // 优先级（b0～b1）：0-最低，1-普通，2-高级，3-关键
        unsigned char frameType : 2;   // SF/CF（b2～b3）：0-单独帧，1-起始帧，2-中间帧，3-结尾帧
        unsigned char reserve   : 4;   // 保留
    }bits;
}InnerNetFrameAttr;

// 设备标识结构定义
typedef union _DeviceId
{
    // 设备ID
    unsigned int devId;
    // 设备ID字节序列
    struct
    {
        unsigned char devNum;  // 设备编号
        unsigned char devType; // 设备类型
        unsigned char sysNum;  // 系统编号
        unsigned char sysType; // 系统类型
    }bytes;
}DeviceId;

typedef struct _InnerNetFrameHead
{
    DeviceId          sourDevId;   // 源设备标识
    unsigned short     sourSoftId;  // 源软件标识
    DeviceId          destDevId;   // 目的设备标识
    unsigned short     destSoftId;  // 目的软件标识
    unsigned int       time;        // 时间，距今日零点的毫秒数
    InnerNetFrameAttr msgAttr;     // 消息属性
    unsigned char      loop;        // 序列号：0～255循环
    unsigned short     msgLen;      // 消息内容长度
    unsigned short     msgId;       // 消息代码
    unsigned char      reserve[2];  // 备用
}InnerNetFrameHead;

#pragma pack()

extern unsigned short packetAttrList[PACKET_TYPE_AMOUNT];
extern unsigned int packetLenList[PACKET_TYPE_AMOUNT];

#endif // DEMOVIDEOMSGPROCESS_H
