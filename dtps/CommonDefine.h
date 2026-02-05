
#ifndef COMMONDEFINE_H
#define COMMONDEFINE_H

#pragma pack(1)


/************** 043 *****************/
typedef struct
{
	unsigned short voiceRecognizeSta;	/*识别状态  0-NA 1-识别中  2-识别成功  3-识别失败*/
	unsigned short needReply;			/*是否需要应答  0-不需要应答  1-需要应答   0923*/
	unsigned short fromWho;			    /*触发识别指令的席位  0-主驾驶  1-副驾驶  2-PAD前  3-PAD后*/
	unsigned short executeSta;			/*指令的执行确认状态  0-NA  1-指令已确认执行  2-指令取消执行 3-任务执行失败  0923*/
	unsigned short cmdType;				/*指令类型  0-NA
	                                      1-页面调显（直接执行）
	                                      2-主模式切换（需确认）
	                                      3-页面切换（直接执行）
	                                      4-设备控制（上下电）(需确认)
	                                      5-参数设置（是否需要确认见具体指令）
	                                      6-协同任务页面显示控制（直接执行）
	                                      7-编队任务规划（预留）
	                                      8-单机任务规划/重规划（需确认）
	                                      9-控制权/编队控制（需确认）*/

	unsigned short mfdPage;				/*页面调显：页面类型
	                                     0-na
	                                     1-垂直
	                                     2-综合
	                                     3-机电
	                                     4-历程
	                                     5-辅助决策
	                                     6-雷达   7-地图
	                                     8-三维地图  9-水平
	                                     10-机外监控  11-飞行计划
	                                     12-航路点 13-航线参数
	                                     14-态势 15-数据
	                                     16-激光 17-光纤
	                                     18-通信 19-自卫
	                                     20-数据加载 21-HUMS 22-三轴  23-夜市  24-自检测
	                                     25-稳瞄 26-维护 27-协同任务 28-编队管理 29-指令 30-态势列表 31-火控 32-编队计划 33-故障清单 34-主菜单页面
	                                     35-返回上一层级页面 36-飞控 37-电侦 38-北斗 39-数据 40-任务重构 */

	unsigned short mfdNo;				/*页面调显：mfd序号 0-na,1-mfd1，2-mfd2, 3-mfd3, 4-mfd4 */
	unsigned short mainMode;			/*主模式切换:0-na,1-反潜模式，2-反舰模式，3-na,4-导航主模式 */

	unsigned short pageSwitch;			/*页面切换：页面切换类型 0-na,1-本机稳瞄，2-本机攻击（预留不使用），3-本机火控（预留不使用），
	                                               4-无人机稳瞄，5-无人机攻击（预留不使用），6-无人机火控（预留不使用），*/
	unsigned short pageSwitchUavSn;		/*页面切换：编队内无人机编号*/

	unsigned short deviceCtrl;			/*设备控制：
	                                     0-na
	                                     1-本机激光上电
	                                     2-本机稳瞄上电（预留）
	                                     3-本机雷达上电（预留）
	                                     4-本机红外上电
	                                     5-无人机Y激光上电
	                                     6-无人机Y稳瞄上电（预留）
	                                     7-无人机Y雷达上电（预留）
	                                     8-无人机Y红外上电
	                                     9-本机激光下电
	                                     10-本机红外下电
	                                     11-无人机Y激光下电
	                                     12-无人机Y红外下电
	                                     13-无人机Y磁探探杆伸出
	                                     14-无人机Y磁探探杆缩回*/
	unsigned short devCtrlUavSn;		/*设备控制：编队内无人机编号*/

	unsigned short ctrlTyp;				/*参数设置：
	                                    0-na
	                                    1-26预留
	                                    27-无人机Y激光测距开始（需确认）
	                                    28-无人机Y激光测距停止（需确认）
	                                    29-无人机Y一键调焦
	                                    30-无人机Y红外
	                                    31-无人机Y电视
	                                    32-无人机Y增大亮度
	                                    33-无人机Y减小亮度
	                                    34-无人机Y增大对比度
	                                    35-无人机Y减小对比度
	                                    36-无人机Y修改激光编码X（需确认）
	                                    37-无人机Y切换武器X（X为武器类型）（需确认）
	                                    38-无人机Y切换占位挂点X（需确认）
	                                    39-无人机Y光电手动
	                                    40-无人机Y光电收藏
	                                    41-无人机Y光电扫描
	                                    42-无人机Y光电锁定
	                                    43-无人机Y光电跟踪
	                                    44-无人机Y光电地理跟踪
	                                    45-无人机Y跟踪目标X（预留）
	                                    46-无人机Y光电黑热
	                                    47-无人机Y光电白热
	                                    48-无人机Y光电视场+
	                                    49-无人机Y光电视场-
	                                    50-无人机Y光电大视场
	                                    51-无人机Y光电中视场
	                                    52-无人机Y光电小视场
	                                    53-无人机Y光电防拥开
	                                    54-无人机Y光电防拥关
	                                    55-无人机光电目标+(预留)
	                                    56-无人机光电目标-(预留)
	                                    57-无人机Y光电自主(预留)
										58-无人机Y光电人工(预留)
	                                    59-本机稳瞄激光测距开始（需确认）
	                                    60-本机稳瞄激光测距停止（需确认）
	                                    61-本机稳瞄一键调焦
	                                    62-本机稳瞄红外
	                                    63-本机稳瞄电视
	                                    64-本机修改激光编码Y（需确认）
	                                    65-本机光电切换手动模式（预留）
	                                    66-本机光电切换收藏模式（预留）
	                                    67-本机光电切换扫描模式（预留）
	                                    68-本机光电切换锁定模式（预留）
	                                    69-本机光电切换跟踪模式（预留）
	                                    70-本机光电切换地理跟踪模式（预留）
	                                    71-本机跟踪目标Y（预留）
	                                    72-本机光电黑热
	                                    73-本机光电白热
	                                    74-本机光电视场+（预留）
	                                    75-本机光电视场-（预留）
	                                    76-本机光电大视场
	                                    77-本机光电中视场
	                                    78-本机光电小视场
	                                    79-本机光电防拥开（预留）
	                                    80-本机光电防拥关（预留）
	                                    81-本机光电目标+（预留）
	                                    82-本机光电目标-（预留）
	                                */
	unsigned short ctrlUavSn;			/*参数设置：编队内无人机编号*/
	unsigned short ctrlWeaponType;		/*参数设置：武器类型（预留）
	                                      1-AKF9A
	                                      2-AKF9B
	                                      3-AKF9C
	                                      4-AKF9D
	                                      5-AKF10A
	                                      6-AKF10B
	                                      7-AKF10C
	                                      8-AKF10D
	                                      9-TL2
	                                      10-FT8*/
	unsigned short ctrlWeaponParam1;	/*参数设置：武器占位（预留）1-9*/
	unsigned short ctrlLaserCode;		/*参数设置：激光编码 （预留）LSB=1*/
	unsigned short ctrlTgtCode;			/*参数设置：目标编码*/

	unsigned short xtPageType;			/*协同页面显示控制： 类型
	                                     0-na
	                                     1-以有人机为中心
	                                     2-以无人机Y为中心
	                                     3-以光标为中心
	                                     4-显示任务进度（预留）
	                                     5-显示本机载荷（预留）
	                                     6-显示无人机载荷（预留）*/
	unsigned short xtPageUavSn;			/*协同页面显示控制： 编队内无人机编号*/

	unsigned short singleUavType;		/*单机任务规划
	                                    0-na
	                                    1-无人机X搜索任务区Y（预留）
	                                    2-无人机X跟踪目标Y（预留）
	                                    3-无人机X打击目标Y（预留）
	                                    4-无人机X导航至航路点Y（预留）
	                                    5-无人机X运输至着陆区Y（预留）
	                                    6-编队飞行（预留）
	                                    7-无人机X盘旋时间Y分钟（预留）
	                                    8-无人机X盘旋圈数Y（预留）
	                                    9-无人机X悬停等待Y分钟（预留）
	                                    10-无人机X盘旋等待Y分钟（预留）
	                                    11-无人机X照射引导Y目标（预留）
	                                    12-无人机X浮标侦听任务区Y
	                                    13-无人机X磁探搜索任务区Y
	                                    14-无人机X磁探跟踪目标Y
	                                    15-无人机X打击规避目标Y
	                                    */
	unsigned short singleUavSn;			/*单机任务规划:编队内无人机编号*/
	unsigned int singleUavAreaId;		/*单机规划:任务区编号*/
	unsigned int singleUavTgtId;		/*单机规划:目标编号*/
	unsigned short singleUavHldId;		/*单机规划:航路点编号 LSB=1*/
	unsigned short singleUavTime;		/*单机规划:时间 LSB=1分钟*/

	unsigned short authorCtrl;			/*控制权/编队控制
	                                     0-na
	                                     1-申请无人机Y三级控制权（预留）
	                                     2-申请无人机Y四级控制权
	                                     3-申请无人机Y五级控制权（预留）
	                                     4-释放无人机Y三级控制权（预留）
	                                     5-释放无人机Y四级控制权（预留）
	                                     6-释放无人机Y五级控制权（预留）
	                                     7-无人机X加入编队（预留） authorCtrlUavId
	                                     8-无人机Y退出编队（预留） authorCtrlUavId
	                                     9-同意无人机Y地面站申请
	                                     10-拒绝无人机Y地面站申请
	                                     11-接收无人机Y控制权
	                                     */
	unsigned short authorCtrlUavSn;		/*控制权/编队控制:编队内无人机编号*/
	unsigned short authorCtrlUavId;		/*控制权/编队控制:编队内无人机id(用于加入编队控制)*/

	char recognizeMsg[100];				/*语音识别文字*/

}BLK_DTPS_DTMS_043;  //语音识别状态反馈  0910 暂定

typedef struct
{
	unsigned short confirm; 	// 语音识别确认执行指令  0-NA；1-执行；2-取消
	unsigned short control;			// 语音识别控制 0-NA；5F5F-启动识别（指控接收到5F5F启动识别，aaaa停止识别）251007
}BLK_DTMS_DTPS_047;

#pragma pack()

#endif /* COMMONDEFINE_H_ */
