////SMD->协同指控(向协同指控发送组包后的舰面浮标状态信息)////////////////////////////////////////////////////////////////////////////////////////////////
//
//#define WORD unsigned short
//#define BYTE char
//#define DWORD int
//
//
//
//
//union FCAE_BUOY_DATA
//{
//	unsigned long long Var;  ///<64位
//	struct
//	{
//	        unsigned long long status:8;///<来源
//	        unsigned long long type:4;///<
//	        unsigned long long fc:3;///<属性
//	        unsigned long long deep:3;///<类型
//	        unsigned long long life:9;///<保留
//	        unsigned long long rfChnl:7;///<保留
//	        unsigned long long vhfVal:8;///<保留
//	        unsigned long long spare1:14;///<保留
//	        unsigned long long src:1;///<保留
//	        unsigned long long spare2:7;///<保留
//	}data;
//	/*
//	8	56~63	Enum	对应浮标状态(Enum)( 0 H=未知;1 H=失效;2 H=存活未接触;3 H=存活已接触 )	8
//	7	52~55	Enum	浮标类型(Enum)( 0 H=未装填;1 H=有浮筒无浮标;2 H=被动全向浮标;3 H=被动定向浮标;4 H=主动全向浮标;5 H=主动定向浮标;6 H=垂直线列阵浮标;7 H=低频声源浮标;8 H=扩展阵浮标;9 H=海噪声浮标;A H=温深浮标 )	4
//	7	49~51	Enum	浮标频率(Enum)( 0 H=0无效;1 H=1425Hz;2 H=4600Hz;3 H=5200Hz;4 H=5800Hz;5 H=6400Hz )	3
//	6	46~48	Enum	浮标深度(Enum)( 0 H=无效;1 H=15米;2 H=40米;3 H=60米;4 H=80米;5 H=150米;6 H=300米 )	3
//	5	37~45	FixData 浮标寿命(单位:分钟；最大值:511；最小值:0；LSB:1；MSB:256；)	9
//	4	30~36	FixData 浮标通道号(最大值:99；最小值:0；LSB:1；MSB:64；) 7
//	3	22~29	Integer 通道电平(最小值:0；最大值:255；)	8
//	2	8~21		SPARE	14
//	1	7~7 Bool	浮标来源(0-本平台;1-全平台)	1
//	1	0~6 	SPARE	7
//	*/
//};
/////浮标投放信息单值
//typedef struct FCAE_DATABUOY_S
//{
//    BYTE id;
//    BYTE tong;
//    WORD plan;
//    union FCAE_BUOY_DATA buoydata;
//        double lo;///<经度
//        double la;///<纬度
//}FCAE_DATABUOY;
//
//typedef struct YR_JM_FBStt_Unit
//{
//    WORD 	PingTaiID;						//平台编号:有人机0 无人机1001~1999
//    WORD 	SeaZone_buoynum;						//海域浮标数量
//    FCAE_DATABUOY buoys[16];
//}YR_JM_FBStt_Unit;
//
////融合目标////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
/////目标信息单值
//union UN_ICD_OBJSpecial
//{
//	WORD Var;  ///<32位
//	struct
//	{
//	#if 0
//	        WORD special_src:3;///<来源
//	        WORD special_aro:3;///< //1:水下；2：水面    0:其它 /// 20220903an    ????
//	        WORD special_prop:3;///<属性 （转）目标属性  1:敌方   2:我方   3:友方(盟军)  4:中立  5:不明/// 20220903an
//	        WORD special_type:3;///<类型  // //目标类型  0:不明 (其它)  1:潜艇   2:水面舰   3:民船   /// ???？
//	        WORD special_save1:4;///<保留
//    #else
//        WORD special_src:3;///<目标源(Enum)( 0 H=N/A;1 H=吊声;2 H=浮标;3 H=综合;4-无人磁/5-无人浮/6-声磁融合)
//	    WORD special_aro:3;///< 空间属性(Enum)( 0 H=N/A;1 H=水下;2 H=水面 )
//	    WORD special_prop:3;///<目标属性(Enum)( 0 H=N/A;1 H=敌;2 H=我;3 H=友;4 H=中立;5 H=不明 )
//	    WORD special_type:3;///<目标类型(Enum)( 0 H=N/A;1 H=潜艇;2 H=水面舰艇;3 H=民船;4 H=其他 )
//	    WORD special_wrflightID:4;///<无人机编号
//    #endif
//	}data;
//};
//
//union UN_ICD_OBJValid
//{
//   WORD Var;
//   struct{
//       WORD time_valid:1;
//       WORD targ_loc_valid:1;
//       WORD targ_amiz_valid:1;
//       WORD targ_dist_valid:1;
//       WORD targ_spd_valid:1;
//       WORD targ_dirc_valid:1;
//       WORD targ_loc_acc:1;
//       WORD targ_amiz_acc:1;
//       WORD targ_dist_acc:1;
//       WORD targ_spd_acc:1;
//       WORD targ_dirc_acc:1;
//       WORD spare:5;
//   }data;
//
//};
//
/////探测点迹参数
//typedef struct POINTTARGET_S{
//    union UN_ICD_OBJValid valid;  ///<有效标识
//    BYTE dd; ///<目标发现时刻(日)
//    BYTE mm; ///<目标发现时刻(月)
//    WORD yy; ///<目标发现时刻(年)
//    DWORD tms; ///<目标发现时刻(秒)
//    WORD lonT[4]; ///<目标位置经度
//    WORD latT[4]; ///<目标位置纬度
//    DWORD amiz; ///<目标方位
//    DWORD dist; ///<目标距离
//    WORD speed; ///<目标航速
//    DWORD direc; ///<目标航向
//    WORD locpre; ///<目标位置精度
//    DWORD amizpre; ///<目标方位精度
//    WORD distpre; ///<目标距离精度
//    WORD speedpre; ///<目标航速精度
//    DWORD direcpre; ///<目标航向精度
//    WORD coeff; ///<目标置信度		[0, 100]  预留
//}POINTTARGET;
//
/////单批目标信息(含4个点迹),（循环7次）
//typedef struct DPUTRACK_S{
//    union UN_ICD_OBJSpecial special;  ///<
//    DWORD batch; ///<目标批次号
//    BYTE  pointNum;///<个数
//    POINTTARGET pointTar[4];			///<探测点迹参数 （循环4次）
//    WORD valid;///<有效字
//    BYTE dd; ///<目标发现时刻(日)
//    BYTE mm; ///<目标发现时刻(月)
//    WORD yy; ///<目标发现时刻(年)
//    DWORD tms; ///<目标发现时刻(秒)
//    WORD lonT[4]; ///<目标位置经度
//    WORD latT[4]; ///<目标位置纬度
//    DWORD amiz; ///<目标方位
//    DWORD dist; ///<目标距离
//    WORD speed; ///<目标航速
//    DWORD direc; ///<目标航向
//} DPUTRACK;
//
/////目标航迹（7组4+1）
//typedef struct FCSend22_UTSS_002_TG_S
//{
//    BYTE num; ///< 目标航迹个数（最大8个）—— 改为7个，短消息限制
//    BYTE spare0; ///<保留
//    DPUTRACK track[7];///< 最多7批目标
//}FCSend22_UTSS_002_TG;
//
//
//
////SMD-协同指控-000(飞行遥控信息数据帧（黑包）)/////////////////////////////////////////////////////////////////////////////////////////////
//
/////1.15	TD 工作指令PMD_UTSS_001	7
//typedef struct FCSend23_CCC_000_WRCMD_S
//{
//	WORD pingtaiID;
//	BYTE SynCode[3];   //	磁命令  0x31  0x31  0x31  声命令  0x34  0x34  0x34
//	BYTE CmdType;
//	BYTE CmdSrc;
//	BYTE data[30];
//	BYTE crc;
//}FCSend23_CCC_000_WRCMD;
