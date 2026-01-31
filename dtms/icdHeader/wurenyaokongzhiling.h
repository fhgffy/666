#ifndef WURENYAOKONGZHILING_H
#define WURENYAOKONGZHILING_H

#include "interior_DTMS_to_CTAS.h"
#pragma pack(1)

/*
 * 1.15 无人机任务载荷控制指令
 * */

//对海警戒搜索控制参数
typedef struct duihaijingjiesearchcontrolcanshu{
    char work_pattern[2];
    char tianxian_zhuansu;
    char gongzuo_pindian;
    char jiance_menxian;
    char maichong_repeat_pinlv[2];
    char tance_distance_1[4];
    char tance_distance_2[4];
    char maichong_width_1[2];
    char maichong_width_2[2];
    char xinhao_width_1[2];
    char xinhao_width_2[2];
}duihaijingjiesearchcontrolcanshu;

//对海跟踪监视控制参数
typedef struct duihaigenzongjianshicontrolcanshu{
    char work_pattern[2];
    char work_pindian;
    char saomiao_zhongxinjiao[2];
    char jiance_menxian;
    char maichong_repeat_pinlv[2];
    char maichong_width[2];
    char xinhao_width[2];
    char tance_distance[4];
    char mubiao_jingdu[4];
    char mubiao_weidu[4];
    char mubiao_height[2];
}duihaigenzongjianshicontrolcanshu;

//条带成像控制参数
typedef struct tiaodaichengxiangcontrolcanshu{
    char work_pattern[2];
    char ceshi_direction;
    char fenbianlv;
    char tance_distance[4];
    char mubiao_height[2];
    char maichong_repeat_pinlv[2];
    char maichong_width[2];
    char xinhao_width[2];
    char yasuo_biaoshi;
}tiaodaichengxiangcontrolcanshu;

//聚束成像控制参数
typedef struct jushuchengxiangcontrolcanshu{
    char work_pattern[2];
    char fenbianlv;
    char mubiao_jingdu[4];
    char mubiao_weidu[4];
    char mubiao_height[2];
    char maichong_repeat_pinlv[2];
    char maichong_width[2];
    char xinhao_width[2];
    char yasuo_biaoshi;
}jushuchengxiangcontrolcanshu;

//对地搜索监视控制参数
typedef struct duidisousuojianshicontrolcanshu{
    char work_pattern[2];
    char work_pindian;
    char tance_distance[4];
    char mubiao_height[2];
    char saomiao_zhongxinjiao[2];
    char maichong_repeat_pinlv[2];
    char maichong_width[2];
    char xinhao_width[2];
}duidisousuojianshicontrolcanshu;

//同时SAR/GMTT控制参数
typedef struct tongshiSARGMTTcontrolcanshu{
    char work_pattern[2];
    char ceshi_direction;
    char fenbianlv;
    char tance_distance[4];
    char mubiao_height[2];
    char maichong_repeat_pinlv[2];
    char maichong_width_1[2];
    char maichong_width_2[2];
    char xinhao_width_1[2];
    char xinhao_width_2[2];
    char yasuo_biaoshi;
}tongshiSARGMTTcontrolcanshu;


//对空搜索监视控制参数
typedef struct duikongsousuojianshicontrolcanshu{
    char work_pattern[2];
    char work_pindian;
    char tance_distance[4];
    char mubiao_height[2];
    char saomiaozhongxinJiao[2];
    char chongpin_fangshi_Select;
}duikongsousuojianshicontrolcanshu;

//校靶模式控制参数
typedef struct jiaobamoshicontrolcanshu{
    char work_pattern[2];
    char tianxian_zhuansu;
    char work_pindian;
}jiaobamoshicontrolcanshu;

//校正控制参数
typedef struct jiaozhengcontrolcanshu{
    char work_pattern[2];
    char jiaozheng_class;
    char jiaozheng_pindian;
    char zidong_jiaozheng;
}jiaozhengcontrolcanshu;

//气象控制参数
typedef struct qixiangcontrolcanshu{
    char work_pattern[2];
    char qita;
}qixiangcontrolcanshu;

//目标干预操作
typedef struct mubiaoganyucaozuo{
    char hangji_zhuangtai;
    char mubiao_pihao[2];
    char mubiao_position_jingdu[4];
    char mubiao_position_weidu[4];
    char disu[2];
    char hangxiang[2];
}mubiaoganyucaozuo;

//辅助功能控制参数
typedef struct fuzhugongnengcontrolcanshu{
    char work_pattern[2];
    char mubiao_pihao[2];
    char mubiao_jingdu[4];
    char mubiao_weidu[4];
    char mubiao_height[2];
    char jingxiang_speed[2];
    char xieju[2];
    char fenbianlv;
    char kaiguan_state;
}fuzhugongnengcontrolcanshu;

//工作状态控制指令
typedef struct workstatecontrolzhiling{
    char work_pattern;
}workstatecontrolzhiling;

//禁用频点控制
typedef struct jinyongpindiankongzhi{
    char pindian0_7;
    char pindian8_15;
    char pindian16_23;
    char pindian24_31;
    char pindian32_39;
    char pindian40;
}jinyongpindiankongzhi;

//雷达通用指令
typedef struct leidatongyongzhiling{
    char celiangliang_save;
    char biaozhunzhi_load;
    char guandaoyuan_select;
    char momu_kaiguan;
    char momu_distance[4];
    char momu_speed[2];
}leidatongyongzhiling;

//设备复位控制指令
typedef struct shebeifuweicontrolzhiling{
    char cifu_control_fuwei;
    char xinhao_chuli_fuwei;
    char data_chuli_fuwei;
    char zonghechulidanyuanzhengji_fuwei;
    char moniqisheibei_fuwei;
    char jiluyi_fuwei;
}shebeifuweicontrolzhiling;

//记录仪设备控制
typedef struct jiluyishebeicontrol{
    char mingling_bianma;
    char data_type;
    char huifang_PRF[2];
    char year[2];
    char month;
    char day;
    char hour;
    char minute;
    char second;
    char file_beizhu[21];
}jiluyishebeicontrol;

//伺服控制参数
typedef struct sifucontrolcanshu{
    char wireless_state;
    char wireless_direction[2];
    char zhuansu;
}sifucontrolcanshu;

//调试控制参数
typedef struct tiaoshicontrolcanshu{
    char tiaoshi_canshu_bianma[2];
    char tiaoshi_canshu_value[2];
}tiaoshicontrolcanshu;

//回放控制参数
typedef struct huifangcontrolcanshu{
    char CFAR_jiance_menxian;
    char chulifangshi;
    char CFAR_baohudanyuanshu;
    char CFAR_cankaodanyuanshu;
    char niying_menxian;
    char huifang_shineng;
    char DBF_chulishineng;
    char canshu_select;
    char huifang_PRF[4];
}huifangcontrolcanshu;


//系统工作模式控制指令
typedef struct sysgongzuomoshicontrolzhiling{
    char beifen;
    char beifen_2;
    char mission_zhixingfangshi_control;
    char jiangji_gongzuomoshi_control;
}sysgongzuomoshicontrolzhiling;

//载荷上下电控制指令
typedef struct zaiheshangxiadiancontrolzhiling{
    char beifen[2];
    char mission_zaiheshangdian_control[2];
}zaiheshangxiadiancontrolzhiling;

//任务构型检查装订指令
typedef struct missiongouxingcheckzhuangdingzhiling{
    char gouxing_check_zhuangding_shineng;
    char gouxing_zhuangding_value_or_current_gouxing;
}missiongouxingcheckzhuangdingzhiling;

//传感器协同引导指令
typedef struct chuanganqixietongyindaozhiling{
    char yindaofangshi_select;
    char yindao_canshu[18];
}chuanganqixietongyindaozhiling;

//引导参数定义
typedef struct yindaocanshudefinition{
    char leida_zhencha_yindao_leida_duihai[8];
    char leida_zhencha_yindao_leida_duidi[8];
    char leida_zhencha_yindao_leida_jusuchengxiang[9];
    char leida_yindao_guangdian[18];
}yindaocanshudefinition;

//任务重规划区域设置指令
typedef struct missionchongguihuaareasetzhiling{
    char chonguihua_mission_id;
    char weixiequ_1_sheding;
    char weixiequ_2_sheding;
    char weixiequ_3_sheding;
    char weixiequ_4_sheding;
    char weixiequ_5_sheding;
}missionchongguihuaareasetzhiling;

//任务重规划启动指令
typedef struct missionchongguihuastartzhiling{
    char chonguihua_mission_id;
    char chonguihua_mission_id_2;
}missionchongguihuastartzhiling;

//目标航迹融合指令
typedef struct mubiaohangjironghezhiling{
    char duomubiao_ronghe_zhiling[2];
    char danmubiao_ronghe_pihao
    ;
}mubiaohangjironghezhiling;

//906数据链传输配置指令
typedef struct shujulianchuanshupeizhizhiling906{
    char fasongfang_address[4];
    char fasongfang_taiweihao[2];
    char jieshoufang_address[4];
    char jieshoufang_taiweihao[2];
    char jiami_biaoshi[2];
}shujulianchuanshupeizhizhiling906;

// 协同任务分配
typedef struct task_reg{ // 任务区信息
    char reg_shape[2]; // 区域形状
    circle reg_circle;  // 圆形区域
    ploygen reg_ploygen; // 多边形区域
}task_reg;

typedef struct target_point{
    double goal_longitude;  //目标经纬度
    double goal_latitude;  //目标经纬度
    double goal_height; //目标高度
    double goal_speed; //目标速度
    double goal_direction;//目标航向
} target_point;

typedef struct cooper_task_fenpei{  // 协同任务分配
    unsigned int task_plan_id; // 任务方案号
    char task_operator; // 任务操作
    unsigned int operator_loc; // 操作位置
    unsigned int subtask_id; // 子任务id
    char subtask_type; // 子任务类型
    unsigned int reg_point_taishi; // 原方案存在，保留待定
    unsigned int subtask_time; // 子任务时间
    unsigned int subtask_heigth; // 子任务执行高度
    // 新增信息
    task_reg task_reg_info; // 任务区信息
    target_point target_point_info; // 目标点信息
}cooper_task_fenpei;

// 协同任务规划启动
typedef struct cooper_task_formulate_start{
    unsigned int task_plan_id; // 任务方案号
    char formulate_start; // 规划启动
    unsigned int subtask_id; // 子任务id
    char result_status; // 规划结果状态
    char apply_status; // 应用状态回复

} cooper_task_formulate_start;

//906数据链分发指令
typedef struct shujulianfenfazhiling906{
    char fenfa_state_shezhi;
    char mubiao_fenfa_state_start;
    char fenfamubiao_pihao[2];
    char fenfacankaodian_jingdu[2];
    char fenfacankaodian_weidu[2];
    char fenfamubiao_pihao_1[2];
    char fenfamubiao_pihao_2[2];
    char fenfamubiao_pihao_3[2];
    char fenfamubiao_pihao_4[2];
    char fenfamubiao_pihao_5[2];
    char fenfamubiao_pihao_6[2];
    char fenfamubiao_pihao_7[2];
    char fenfamubiao_pihao_8[2];
    char fenfamubiao_pihao_9[2];
    char fenfamubiao_pihao_10[2];
    char fenfamubiao_pihao_11[2];
    char fenfamubiao_pihao_12[2];
}shujulianfenfazhiling906;

//906数据链目标指示指令
typedef struct shujulianmubiaozhishizhiling906{
    char mubiao_zhishi_state_set;
    char mubiao_fenfa_state_start;
    char mubiao_zhishi_mubiao_pihao[2];
    char mubiao_zhishi_cankaodian_jingdu[2];
    char mubiao_zhishi_cankaodian_weidu[2];
    char mubiaozhishi_jieshoupingtaiNum[2];
    char mubiao_id[2];
}shujulianmubiaozhishizhiling906;

//数据加载指令
typedef struct dataloadzhiling{
    char query_loadfile_state[2];
    char choose_file_in_need_loading[2];
}dataloadzhiling;

//数据记录指令
typedef struct datarecordzhiling{
    char data_record[2];
}datarecordzhiling;

//数据检索指令
typedef struct datasearchzhiling{
    char searchData_type;
    char searchData_tiaojian;
    char searchData_startTime[4];
    char searchData_endTime[4];
}datasearchzhiling;

//业务数据压缩下传指令
typedef struct yewuDatayasuoxiachuanzhiling{
    char set_or_query;
    char yewu_data_keyong_zongdaikuan_shezhi;
    char dianshi_video_width;
    char hongwaivideo_width;
    char shumazhaopian_width;
    char leida_keyasuo_width;
    char qita_data_kaiguan[2];
    char xiachuan_shexiangtou_width;
    char qianshi_shexiangtou_width;
}yewuDatayasuoxiachuanzhiling;

//数据删除指令
typedef struct datadeletezhiling{
    char data_type;
    char data_tiaojian;
    char data_startTime[4];
    char data_endTime[4];
}datadeletezhiling;

//软件升级维护指令
typedef struct softwareshengjiweihuzhiling{
    char software_zaixianshengjikongzhi_zhiling[2];
    char select_software_edition_num[4];
}softwareshengjiweihuzhiling;

//磁探控制指令
typedef struct citancontrolzhiling{
    char beifen[2];
    char citan_control[2];
    char zidingyi_control[16];
}citancontrolzhiling;

//磁探收放装置控制指令
typedef struct citanshoufangzhuangzhicontrolzhiling{
    char citan_shoufang_zhuangzhi_control;
}citanshoufangzhuangzhicontrolzhiling;

//浮标状态控制指令
typedef struct fubiaostatecontrolzhiling{
    char control_zhiling_source;
    char mingling_biaoshi;
    char source_platform_num[4];
    char target_platform_num[4];
}fubiaostatecontrolzhiling;

//浮标控制指令
typedef struct fubiaocontrolzhiling{
    char source_data_set;
    char beifen;
    char source_platform_num[4];
    char target_platform_num[4];
    char data_frame_head;
    char data_frame_biaoshi;
    char big_bag_num;
    char fenbao_sum;
    char big_bag_data_general_length[2];
    char fenbao_num;
    char fenbao_data_available_length;
    char data_content[13];
}fubiaocontrolzhiling;


//参数查询和加载指令
typedef struct canshuqueryandloadzhiling{
    char current_work_canshu_diaoxian[2];
    char diaoxian_geshebei_yuzhicanshu[2];
    char zongheshepin_gouxinSet[2];
}canshuqueryandloadzhiling;

//综合射频重构指令
typedef struct zongheshepinchonggouzhiling{
    char new_chonggou_function;
    char old_chonggou_function;
}zongheshepinchonggouzhiling;

//多信道通信中继波道参数设置
typedef struct duoxindaotongxinzhongjibodaocanshuSet{
    char duoxindao_zhongji_select;
    char bodao_num;
    char work_pattern;
    char work_pinlv[4];
    char maxing;
    char zuhao;
    char pinduan;
    char tuan;
    char tiaosu;
    char biaohao;
    char wanghao;
    char miyaohao;
    char PRGzhishi;
    char user_speed;
    char xinxi_miyao_num;
    char net_pattern;
    char diantai_level;
    char jiamifangshi;
    char xindao_jieru_fangshi;
    char language_code_pattern;
}duoxindaotongxinzhongjibodaocanshuSet;

//多信道通信中继设备地址设置
typedef struct duoxindaotongxinzhongjishebeidizhicanshuSet{
    char duoxindao_zhongji_select;
    char MAC_Address;
    char IP_Address[4];
    char address_code[2];
    char shebei_id[2];
    char changjia_id[2];
    char shebeiNum_id[2];
    char shangji_wanghao;
    char net_guihuajiedian_num;
    char yuliu[2];
}duoxindaotongxinzhongjishebeidizhicanshuSet;

//多信道通信中继TOD参数设置
typedef struct duoxindaotongxinzhongjiTODcanshuSet{
    char duoxindao_zhongji_select;
    char year[2];
    char month;
    char day;
    char hour;
    char minute;
    char second;
}duoxindaotongxinzhongjiTODcanshuSet;

//敌我识别应答工作参数参数设置
typedef struct diwoshibieyingdagongzuocanshuSet{
    char jingmo_control;
    char changgui_pattern;
    char xiaomi_control;
}diwoshibieyingdagongzuocanshuSet;

//航管应答工作参数参数设置
typedef struct hangguanyingdagongzuocanshuSet{
    char canshu_control;
    char beifen;
    char ATC_yingdama[2];
    char s_pattern_address_num[4];
    char hangbanhao_1;
    char hangbanhao_2;
    char hangbanhao_3;
    char hangbanhao_4;
    char hangbanhao_5;
    char hangbanhao_6;
    char hangbanhao_7;
    char hangbanhao_8;
}hangguanyingdagongzuocanshuSet;

//AIS工作参数参数设置
typedef struct AISgongzuocanshuSet{
    char kaiguan_control;
}AISgongzuocanshuSet;

//无线电高度剩余高度设置
typedef struct wuxiandianshengyuheightset{
    char beiyong;
    char beiyong_2[2];
    char shengyu_height[2];
}wuxiandianshengyuheightset;

//906数据链工作参数设置
typedef struct shujuliangongzuocanshu906set{
    char bianhao;
    char fangshi;
    char work_pattern;
    char zhongduan_pattern;
    char kaiguan_control;
    char jingmo_fangshi;
    char work_pindian;
    char jiami_biaoshi;
    char net_bianhao;
    char mission_num_1;
    char mission_num_2;
    char mission_num_3;
    char mission_num_4;
    char mission_num_5;
    char mission_num_6;
    char work_speed;
    char shixi_process;
    char state;
}shujuliangongzuocanshu906set;

//JIDS参数设置
typedef struct JIDScanshuset{
    char wangcan_guihuahao;
    char beifen;
    char platform_bianshi_num[4];
    char jingmo_control;
    char yingjijiami;
    char fashetianxian_select;
    char train_model;
    char train_kuhao;
    char train_num;
}JIDScanshuset;

//电子侦查工作参数设置指令
typedef struct elezhenchaworkcanshuset{
    char canshu_1;
}elezhenchaworkcanshuset;

//雷达侦察工作参数设置指令
typedef struct leidazhenchaworkcanshuset{
    char canshu_1;
    char canshu_2;
    char canshu_3;
    char canshu_4;
    char canshu_5;
    char canshu_6;
    char canshu_7;
    char canshu_8;
    char canshu_9;
    char canshu_10;
    char canshu_11;
    char canshu_12;
    char canshu_13;
    char canshu_14;
    char canshu_15;
    char canshu_16;
    char canshu_17;
    char canshu_18;
    char canshu_19;
}leidazhenchaworkcanshuset;


//通信侦察工作参数设置指令
typedef struct tongxinzhenchaworkcanshuset{
    char canshu_1;
    char canshu_2;
}tongxinzhenchaworkcanshuset;

//雷达侦查处理参数设置指令
typedef struct leidazhenchachulicanshuset{
    char chulicanshu_type;
    char canshu_1[29];
}leidazhenchachulicanshuset;

//通信侦查处理设置指令
typedef struct tongxinzhenchachulicanshuset{
    char canshu_1[29];
}tongxinzhenchachulicanshuset;

// 保密参数设置
typedef struct baomicanshuset{


}baomicanshuset;
//激光惯性部件控制参数设置
typedef struct jiguangguanxingbujiancanshuset{
    char guangdao_work_state;
    char kongzhong_duizhun_control;
    char weixing_work_state;
    char guandao_zuhe_pattern;
}jiguangguanxingbujiancanshuset;

//激光惯性部件带数据参数设置
typedef struct jiguangguanxingbujiandaishujucanshuset{
    char canshu_shezhi_youxiaozi;
    char rengong_set_weidu[4];
    char rengong_set_jingdu[4];
    char rengong_set_height[4];
    char changya_zhuangdingzhi[2];
}jiguangguanxingbujiandaishujucanshuset;

//激光惯性部件安装误差参数1设置
typedef struct jiguangguanxingbujiananzhuangcanshu1set{
    char canshu_shezhi_youxiaozi[2];
    char guandaofuyang[2];
    char guandaohenggun[2];
    char guandaohangxiang[2];
    char shuangtianxianhangxiang[2];
    char shuangtianxianfuyang[2];
    char shuangtianxianhenggun[2];
    char yidongshijizhun_hengzhou[2];
    char yidongshijizhun_zongzhou[2];
    char yidongshijizhun_fazhou[2];
    char kangganrao_hengzhou[2];
    char kangganrao_zongzhou[2];
    char kangganrao_fazhou[2];
    char celiangxing_hengzhou[2];
    char celiangxing_zongzhou[2];
    char celiangxing_fazhou[2];
}jiguangguanxingbujiananzhuangcanshu1set;

//激光惯性部件安装误差参数2设置
typedef struct jiguangguanxingbujiananzhuangcanshu2set{
    char canshu_shezhi_youxiaozi[2];
    char yuchazhongxin_hengzhou[2];
    char yuchazhongxin_zongzhou[2];
    char yuchazhongxin_fazhou[2];
}jiguangguanxingbujiananzhuangcanshu2set;

//光纤惯性部件控制参数设置
typedef struct guangqianguanxingbujiankongzhicanshuset{
    char guandao_work_state;
    char kongzhong_duizhun_control;
    char weixing_work_state;
    char guandao_zuhe_pattern;
}guangqianguanxingbujiankongzhicanshuset;

//光纤惯性部件带数据参数设置
typedef struct guangqianguanxingbujiandaishujucanshuset{
    char canshushezhi_youxiaozi;
    char rengong_shezhi_weidu[4];
    char rengong_shezhi_jingdu[4];
    char rengong_shezhi_gaodu[4];
    char changya_zhuangding_value[2];
}guangqianguanxingbujiandaishujucanshuset;

//光纤惯性部件带数据参数设置
typedef struct guangqianguanxingbujiananzhuangcanshu1set{
    char canshu_shezhi_youxiaozi[2];
    char guandaofuyang[2];
    char guandaohenggun[2];
    char guandaohangxiang[2];
    char shuangtianxianhangxiang[2];
    char shuangtianxianfuyang[2];
    char shuangtianxianhenggun[2];
    char yidongshijizhun_hengzhou[2];
    char yidongshijizhun_zongzhou[2];
    char yidongshijizhun_fazhou[2];
    char kangganrao_hengzhou[2];
    char kangganrao_zongzhou[2];
    char kangganrao_fazhou[2];
    char celiangxing_hengzhou[2];
    char celiangxing_zongzhou[2];
    char celiangxing_fazhou[2];
}guangqianguanxingbujiananzhuangcanshu1set;

//光纤惯性部件安装误差参数2设置
typedef struct guangxianguanxingbujiananzhuangcanshu2set{
    char canshu_shezhi_youxiaozi[2];
    char yuchazhongxin_hengzhou[2];
    char yuchazhongxin_zongzhou[2];
    char yuchazhongxin_fazhou[2];
}guangxianguanxingbujiananzhuangcanshu2set;

//维护控制设置
typedef struct weihucontrolset{
    char sys_id;
    char enter_state;
    char weihuguzhang_qingdan_qingqiu;
    char weihuguzhang_qingdan_fanye;
    char delete_control;
    char weihu_zijiance;
}weihucontrolset;

//C段测控链工作参数设置
typedef struct Cduancekonglianset{
    char shebei_id[2];
    char zhukongzhan_id[2];
    char kongzhi_pattern;
    char shangxing_pindian;
    char xiaxing_pindian;
}Cduancekonglianset;

//U段测控链工作参数设置
typedef struct Uduancekonglianset{
    char shebei_id[2];
    char zhukongzhan_id[2];
    char gongzuo_pindian;
    char jiamizhuangtai;
}Uduancekonglianset;





//光电遥控指令
typedef struct guangdianyaokongzhiling{
    unsigned char zhiling_class;
    //0X10 程控状态
    //0X20 控制状态
    //0X30 参数设置状态
    //00   空指令
    unsigned char guangdian_control_mingling;
    //D7 境内外标志 0境内 1境外
    //D6 折返标志 0不折返 1折返
    //D5 0主航路点 1任务航路点
    //0x90 红外电源开
    //0x92 红外电源关
    unsigned char zhiling_data;
    //0x00 空指令
    //0x01 电视短焦
    //0x03 电视长焦
    //0x05 红外短焦
    //0x07 红外长焦
    //0x11 前向方位扫描
    //0x13 右侧方位扫描
    //0x15 后向方位扫描
    //0x17 左侧方位扫描
    //0x19 前向俯仰扫描
    //0x1B 右侧俯仰扫描
    //0x1D 后向俯仰扫描
    //0x1E 左侧俯仰扫描
    //0x21 红外短焦高扫 先默认高扫 20241113
    //0x23 红外短焦低扫
    //0x25 红外长焦高扫
    //0x27 红外长焦低扫
    //0x31 电视短焦高扫
    //0x33 电视短焦低扫
    //0x35 电视长焦高扫
    //0x37 电视长焦低扫
    //0x41 垂直下视锁定
    //0x43 前向锁定
    //0x45 右向锁定
    //0x47 后向锁定
    //0x49 左侧锁定
    char beiyong;//备用
    short dangan_fangwei;
    short dangan_fuyang;
    char param_1[4];//参数1
    char param_2[4];//参数2
    char param_3[2];//参数3
}guangdianyaokongzhiling;
//任务遥控数据A帧结构
typedef struct renwuyaokongshuju_Azhen
{
    unsigned char tongbuma1 ;//同步码1 0XEB
    unsigned char tongbuma2 ;//同步码2 0X91
    unsigned char zhenleixing;//帧类型 0XA0
    unsigned char zhenjishu;//帧计数 0-255步进循环
    guangdianyaokongzhiling guangdiankongzhizhilings;//光电控制指令
    char zongherenwuguanlixitong_leida_citanzhiling[36];//综合任务管理系统/雷达/磁探指令
    char beifen[2];//备份(飞机ID)
    char beifen2[2];//备份（飞机ID）
    unsigned short CRC_jiaoyan;//CRC校验
}renwuyaokongshuju_Azhen;

//任务遥控数据B帧结构
typedef struct renwuyaokongshuju_Bzhen
{
    char tongbuma1;//同步码1
    char tongbuma2 ;//同步码2
    char zhenleixing ;//帧类型
    char zhenjishu;//帧计数
    guangdianyaokongzhiling guangdiankongzhizhilings;//光电控制指令
    char zongheshepinzhiling[36];//综合射频系统指令、导航、HUMS以及维护指令
    char baoliu[4];//保留
    char CRC_jiaoyan[2];//CRC校验
}renwuyaokongshuju_Bzhen;


#pragma pack()
#endif // WURENYAOKONGZHILING_H
