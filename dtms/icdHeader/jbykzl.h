
#ifndef JBYKZL_H
#define JBYKZL_H

#include "header_DTMS/aide_func.h"

//基本遥控指令
//遥控指令数据
typedef struct yaokong_data{
    QByteArray jizhan_distance;
    QByteArray uav_hc_ganweiJiao;
    QByteArray ship_jingDu;
    QByteArray ship_weiDu;
    QByteArray ship_groundSpeed;
    QByteArray ship_direction;
    QByteArray zhuangtai_and_data_sign;
    QByteArray beiyong;
}yaokong_data;

//飞行连续指令
typedef struct feixing_lianxu_zhiling{
    QByteArray start;
    QByteArray zongxiang_control_zhiling;
    QByteArray hengxiang_control_zhiling;
    QByteArray zongju_control_zhiling;
    QByteArray hangxiang_control_zhiling;
    QByteArray duli_youmen_weitiao;
    QByteArray zongju_youmen_weitiao;
    QByteArray jiaoyan;
}feixing_lianxu_zhiling;

//遥控指令-26
typedef struct feixing_yaokong_zhiling{
    char start;
    char yaokong_zhiling_bianma;
    yaokong_data yaokong_zhiling_shuju;
    QByteArray jiaoYan;
}feixing_yaokong_zhiling;



//基本遥控指令帧编码结构
typedef struct jbykzlbmjg{
    QByteArray synchronization_code;
    QByteArray frame_class;
    QByteArray frame_count;
    QByteArray plane_address;
    QByteArray controlStation_address;
    char plane_yaokong_zhiling;
    feixing_yaokong_zhiling feixing_Yaokong_Zhiling;
    feixing_lianxu_zhiling feixing_Lianxu_Zhiling;
    QByteArray jiaoYanHe;
    QByteArray synchronization_code_2;
    QByteArray frame_count_num;
    char data_zhuangding_zhilingma;
    QByteArray data_zhuangding_content;
    QByteArray jiaoYanHe_2;
}jbykzlbmjg;


typedef struct control_right_startjiaoJie{
    QByteArray control_station_address;
    QByteArray control_station_address_2;
    QByteArray control_station_address_3;

}control_right_startjiaoJie;
typedef struct control_right_finishjiaoJie{
    QByteArray control_station_address;
    QByteArray control_station_address_2;
    QByteArray control_station_address_3;

}control_right_finishjiaoJie;
//遥调指令详细编码表
typedef struct yaokong_zhiling_xiangxibianma_table{
    QByteArray mission_class_zhuangding;
    control_right_startjiaoJie control_right_startjiaojie;
    control_right_finishjiaoJie control_right_finishjiaojie;
    QByteArray position_yaoTiao;
    QByteArray relative_position_yaoTiao;
    QByteArray relative_height_yaoTiao;
    QByteArray absolute_height_yaoTiao;
    QByteArray hangxiang_yaoTiao;
    QByteArray speed_yaoTiao;
    QByteArray chuizhi_speed_yaoTiao;
    QByteArray zhuolu_changYaGaoDuZhuRu;
    QByteArray shoudong_info_deal;
    QByteArray zhuojian_relative_direction;
    QByteArray SSPC_control;
    QByteArray guzhang_pattern_zhuru;
    QByteArray ship_ganbi_param_zhuru;
    QByteArray ship_jiaban_chushuiHeight;
    QByteArray zhuojian_yaodaoyuan_Choose;
    QByteArray zhuojian_zuobiaoxi_Select;
    QByteArray zitaineihuilu_xinhao_Select;
    QByteArray zhuojian_pattern_Choose;
    QByteArray qifei_tiaojian_Search;
    feixing_yaokong_zhiling empty_zhiling;
}yaokong_zhiling_xiangxibianma_table;

//飞行连续指令详细编码
typedef struct feixing_lianxuzhiling_xiangxi_bianma{
    QByteArray yaokongfeixing_zongxiangzhouqi_bianju;
    QByteArray zhilingfeixing_zongxiangspeed_zhiling;
    QByteArray yaokongfeixing_hengxiangzhouqi_bianju;
    QByteArray zhilingfeixing_hengxiang_speed_zhiling;
    QByteArray zhilingfeixing_pianhang_jiaosudu_zhiling;
    QByteArray yaokongfeixing_zongju;
    QByteArray zhilingfeixing_chuixiangspeed_zhiling;
    QByteArray yaokongfeixing_weijiangju;
    QByteArray zhilingfeixing_pianhang_jiaosudu_zhiling_2;
    QByteArray zhilingfeixing_hengxiang_speed_zhiling_2;
    QByteArray duliyoumen_weiTiao;
    QByteArray zongjuyoumen_weiTiao;
}feixing_lianxuzhiling_xiangxi_bianma;

//航向与威胁管理指令内容
typedef struct hangxiang_and_weixie_guanlizhilingContent{
    QByteArray kongzhiLing;
    QByteArray dingdianxuanting_position_zhuru;
    QByteArray fanhang_airportPoint_zhuru;
    QByteArray yingjifanHang;
    QByteArray xiangxingqudianFei;
    QByteArray hangdianInsert;
    QByteArray hangdianzhuangding_absolute;
    QByteArray hangdianzhuangding_relative;
    QByteArray hangdianxiugai;
    QByteArray absolute_hangxian_hangdian_query;
    QByteArray relative_hangxian_hangdian_query;
    QByteArray hangxian_hangdian_delete;
    QByteArray hangxian_hangdian_qiehuan;
    QByteArray hangxian_length_query;
    QByteArray weixie_data_insert;
    QByteArray weixie_area_query;
    QByteArray weixie_area_delete;
}hangxiang_and_weixie_guanlizhilingContent;

//光电遥控指令
typedef struct guangdian_yaokong_zhiling{
    QByteArray zhiling_class;
    QByteArray guangdian_control_mingling;
    QByteArray zhiling_data;
    QByteArray beiyong;
    QByteArray dangan_fangwei;
    QByteArray dangan_fuyang;
    QByteArray param_1;
    QByteArray param_2;
    QByteArray param_3;
}guangdian_yaokong_zhiling;



//任务遥控数据A帧结构
typedef struct mission_yaokong_data_A{
    QByteArray synchronization_code_1;
    QByteArray synchronization_code_2;
    QByteArray frame_class;
    int frame_count;
    guangdian_yaokong_zhiling guangdian_control_zhiling;
    QByteArray zongheshepingxitongzhiling_daohang_HUMS_weihu_zhiling; //有疑问
    QByteArray baoliu;
    QByteArray crc_jiaoyan;
}mission_yaokong_data_A;

//任务遥控数据A帧遥调指令
typedef struct mission_yaokong_yaotiaozhiling_A{
    QByteArray yaotiao_bianma;
    QByteArray data;
    QByteArray jianyan;
}mission_yaokong_yaotiaozhiling_A;

//对海警戒搜索控制参数
typedef struct duihai_jingjie_search_control_canshu{
    QByteArray work_pattern;
    QByteArray tianxian_zhuansu;
    QByteArray gongzuo_pindian;
    QByteArray jiance_menxian;
    QByteArray maichong_repeat_pinlv;
    QByteArray tance_distance_1;
    QByteArray tance_distance_2;
    QByteArray maichong_width_1;
    QByteArray maichong_width_2;
    QByteArray xinhao_width_1;
    QByteArray xinhao_width_2;
}duihai_jingjie_search_control_canshu;

//对海跟踪监视控制参数
typedef struct duihai_genzong_jianshi_control_canshu{
    QByteArray work_pattern;
    QByteArray work_pindian;
    QByteArray saomiao_zhongxinjiao;
    QByteArray jiance_menxian;
    QByteArray maichong_repeat_pinlv;
    QByteArray maichong_width;
    QByteArray xinhao_width;
    QByteArray tance_distance;
    QByteArray mubiao_jingdu;
    QByteArray mubiao_weidu;
    QByteArray mubiao_height;
}duihai_genzong_jianshi_control_canshu;

//条带成像控制参数
typedef struct tiaodai_chengxiang_control_canshu{
    QByteArray work_pattern;
    QByteArray ceshi_direction;
    QByteArray fenbianlv;
    QByteArray tance_distance;
    QByteArray mubiao_height;
    QByteArray maichong_repeat_pinlv;
    QByteArray maichong_width;
    QByteArray xinhao_width;
    QByteArray yasuo_biaoshi;
}tiaodai_chengxiang_control_canshu;

//聚束成像控制参数
typedef struct jushu_chengxiang_control_canshu{
    QByteArray work_pattern;
    QByteArray fenbianlv;
    QByteArray mubiao_jingdu;
    QByteArray mubiao_weidu;
    QByteArray mubiao_height;
    QByteArray maichong_repeat_pinlv;
    QByteArray maichong_width;
    QByteArray xinhao_width;
    QByteArray yasuo_biaoshi;
}jushu_chengxiang_control_canshu;

//对地搜索监视控制参数
typedef struct duidi_sousuo_jianshi_control_canshu{
    QByteArray work_pattern;
    QByteArray work_pindian;
    QByteArray tance_distance;
    QByteArray mubiao_height;
    QByteArray saomiao_zhongxinjiao;
    QByteArray maichong_repeat_pinlv;
    QByteArray maichong_width;
    QByteArray xinhao_width;
}duidi_sousuo_jianshi_control_canshu;

//同时SAR/GMTT控制参数
typedef struct tongshi_SAR_GMTT_control_canshu{
    QByteArray work_pattern;
    QByteArray ceshi_direction;
    QByteArray fenbianlv;
    QByteArray tance_distance;
    QByteArray mubiao_height;
    QByteArray maichong_repeat_pinlv;
    QByteArray maichong_width_1;
    QByteArray maichong_width_2;
    QByteArray xinhao_width_1;
    QByteArray xinhao_width_2;
    QByteArray yasuo_biaoshi;
}tongshi_SAR_GMTT_control_canshu;


//对空搜索监视控制参数
typedef struct duikong_sousuo_jianshi_control_canshu{
    QByteArray work_pattern;
    QByteArray work_pindian;
    QByteArray tance_distance;
    QByteArray mubiao_height;
    QByteArray saomiaozhongxinJiao;
    QByteArray chongpin_fangshi_Select;
}duikong_sousuo_jianshi_control_canshu;

//校靶模式控制参数
typedef struct jiaoba_moshi_control_canshu{
    QByteArray work_pattern;
    QByteArray tianxian_zhuansu;
    QByteArray work_pindian;
}jiaoba_moshi_control_canshu;

//校正控制参数
typedef struct jiaozheng_control_canshu{
    QByteArray work_pattern;
    QByteArray jiaozheng_class;
    QByteArray jiaozheng_pindian;
    QByteArray zidong_jiaozheng;
}jiaozheng_control_canshu;

//气象控制参数
typedef struct qixiang_control_canshu{
    QByteArray work_pattern;
    QByteArray qita;
}qixiang_control_canshu;

//目标干预操作
typedef struct mubiao_ganyu_caozuo{
    QByteArray hangji_zhuangtai;
    QByteArray mubiao_pihao;
    QByteArray mubiao_position_jingdu;
    QByteArray mubiao_position_weidu;
    QByteArray disu;
    QByteArray hangxiang;
}mubiao_ganyu_caozuo;

//辅助功能控制参数
typedef struct fuzhugongneng_control_canshu{
    QByteArray work_pattern;
    QByteArray mubiao_pihao;
    QByteArray mubiao_jingdu;
    QByteArray mubiao_weidu;
    QByteArray mubiao_height;
    QByteArray jingxiang_speed;
    QByteArray xieju;
    QByteArray fenbianlv;
    QByteArray kaiguan_state;
}fuzhugongneng_control_canshu;

//工作状态控制指令
typedef struct workstate_control_zhiling{
    QByteArray work_pattern;
}workstate_control_zhiling;

//禁用频点控制
typedef struct jinyong_pindian_kongzhi{
    QByteArray pindian0_7;
    QByteArray pindian8_15;
    QByteArray pindian16_23;
    QByteArray pindian24_31;
    QByteArray pindian32_39;
    QByteArray pindian40;
}jinyong_pindian_kongzhi;

//雷达通用指令
typedef struct leida_tongyong_zhiling{
    QByteArray celiangliang_save;
    QByteArray biaozhunzhi_load;
    QByteArray guandaoyuan_select;
    QByteArray momu_kaiguan;
    QByteArray momu_distance;
    QByteArray momu_speed;
}leida_tongyong_zhiling;

//设备复位控制指令
typedef struct shebei_fuwei_control_zhiling{
    QByteArray cifu_control_fuwei;
    QByteArray xinhao_chuli_fuwei;
    QByteArray data_chuli_fuwei;
    QByteArray zonghechulidanyuanzhengji_fuwei;
    QByteArray moniqisheibei_fuwei;
    QByteArray jiluyi_fuwei;
}shebei_fuwei_control_zhiling;

//记录仪设备控制
typedef struct jiluyi_shebei_control{
    QByteArray mingling_bianma;
    QByteArray data_type;
    QByteArray huifang_PRF;
    QByteArray year;
    QByteArray month;
    QByteArray day;
    QByteArray hour;
    QByteArray minute;
    QByteArray second;
    QByteArray file_beizhu;
}jiluyi_shebei_control;

//伺服控制参数
typedef struct sifu_control_canshu{
    QByteArray wireless_state;
    QByteArray wireless_direction;
    QByteArray zhuansu;
}sifu_control_canshu;

//调试控制参数
typedef struct tiaoshi_control_canshu{
    QByteArray tiaoshi_canshu_bianma;
    QByteArray tiaoshi_canshu_value;
}tiaoshi_control_canshu;

//回放控制参数
typedef struct huifang_control_canshu{
    QByteArray CFAR_jiance_menxian;
    QByteArray chulifangshi;
    QByteArray CFAR_baohudanyuanshu;
    QByteArray CFAR_cankaodanyuanshu;
    QByteArray niying_menxian;
    QByteArray huifang_shineng;
    QByteArray DBF_chulishineng;
    QByteArray canshu_select;
    QByteArray huifang_PRF;
}huifang_control_canshu;

//系统工作模式控制指令
typedef struct sys_gongzuo_moshi_control_zhiling{
    QByteArray beifen;
    QByteArray beifen_2;
    QByteArray mission_zhixingfangshi_control;
    QByteArray jiangji_gongzuomoshi_control;
}sys_gongzuo_moshi_control_zhiling;

//载荷上下电控制指令
typedef struct zaihe_shangxiadian_control_zhiling{
    QByteArray beifen;
    QByteArray mission_zaiheshangdian_control;
}zaihe_shangxiadian_control_zhiling;

//任务构型检查装订指令
typedef struct mission_gouxing_check_zhuangding_zhiling{
    QByteArray gouxing_check_zhuangding_shineng;
    QByteArray gouxing_zhuangding_value_or_current_gouxing;
}mission_gouxing_check_zhuangding_zhiling;

//传感器协同引导指令
typedef struct chuanganqi_xietong_yindao_zhiling{
    QByteArray yindaofangshi_select;
    QByteArray yindao_canshu;
}chuanganqi_xietong_yindao_zhiling;

//引导参数定义
typedef struct yindao_canshu_definition{
    QByteArray leida_zhencha_yindao_leida_duihai;
    QByteArray leida_zhencha_yindao_leida_duidi;
    QByteArray leida_zhencha_yindao_leida_jusuchengxiang;
    QByteArray leida_yindao_guangdian;
}yindao_canshu_definition;

//任务重规划区域设置指令
typedef struct mission_chongguihua_area_set_zhiling{
    QByteArray chonguihua_mission_id;
    QByteArray weixiequ_1_sheding;
    QByteArray weixiequ_2_sheding;
    QByteArray weixiequ_3_sheding;
    QByteArray weixiequ_4_sheding;
    QByteArray weixiequ_5_sheding;
}mission_chongguihua_area_set_zhiling;

//任务重规划启动指令
typedef struct mission_chongguihua_start_zhiling{
    QByteArray chonguihua_mission_id;
    QByteArray chonguihua_mission_id_2;
}mission_chongguihua_start_zhiling;

//目标航迹融合指令
typedef struct mubiao_hangji_ronghe_zhiling{
    QByteArray duomubiao_ronghe_zhiling;
    QByteArray danmubiao_ronghe_pihao
    ;
}mubiao_hangji_ronghe_zhiling;

//906数据链传输配置指令
typedef struct shujulian_chuanshupeizhi_zhiling_906{
    QByteArray fasongfang_address;
    QByteArray fasongfang_taiweihao;
    QByteArray jieshoufang_address;
    QByteArray jieshoufang_taiweihao;
    QByteArray jiami_biaoshi;
}shujulian_chuanshupeizhi_zhiling_906;

//906数据链分发指令
typedef struct shujulian_fenfa_zhiling_906{
    QByteArray fenfa_state_shezhi;
    QByteArray mubiao_fenfa_state_start;
    QByteArray fenfamubiao_pihao;
    QByteArray fenfacankaodian_jingdu;
    QByteArray fenfacankaodian_weidu;
    QByteArray fenfamubiao_pihao_1;
    QByteArray fenfamubiao_pihao_2;
    QByteArray fenfamubiao_pihao_3;
    QByteArray fenfamubiao_pihao_4;
    QByteArray fenfamubiao_pihao_5;
    QByteArray fenfamubiao_pihao_6;
    QByteArray fenfamubiao_pihao_7;
    QByteArray fenfamubiao_pihao_8;
    QByteArray fenfamubiao_pihao_9;
    QByteArray fenfamubiao_pihao_10;
    QByteArray fenfamubiao_pihao_11;
    QByteArray fenfamubiao_pihao_12;
}shujulian_fenfa_zhiling_906;

//906数据链目标指示指令
typedef struct shujulian_mubiao_zhishi_zhiling_906{
    QByteArray mubiao_zhishi_state_set;
    QByteArray mubiao_fenfa_state_start;
    QByteArray mubiao_zhishi_mubiao_pihao;
    QByteArray mubiao_zhishi_cankaodian_jingdu;
    QByteArray mubiao_zhishi_cankaodian_weidu;
    QByteArray mubiaozhishi_jieshoupingtaiNum;
    QByteArray mubiao_id;
}shujulian_mubiao_zhishi_zhiling_906;

//数据加载指令
typedef struct data_load_zhiling{
    QByteArray query_loadfile_state;
    QByteArray choose_file_in_need_loading;
}data_load_zhiling;

//数据记录指令
typedef struct data_record_zhiling{
    QByteArray data_record;
}data_record_zhiling;

//数据检索指令
typedef struct data_search_zhiling{
    QByteArray searchData_type;
    QByteArray searchData_tiaojian;
    QByteArray searchData_startTime;
    QByteArray searchData_endTime;
}data_search_zhiling;

//业务数据压缩下传指令
typedef struct yewuData_yasuo_xiachuan_zhiling{
    QByteArray set_or_query;
    QByteArray yewu_data_keyong_zongdaikuan_shezhi;
    QByteArray dianshi_video_width;
    QByteArray hongwaivideo_width;
    QByteArray shumazhaopian_width;
    QByteArray leida_keyasuo_width;
    QByteArray qita_data_kaiguan;
    QByteArray xiachuan_shexiangtou_width;
    QByteArray qianshi_shexiangtou_width;
}yewuData_yasuo_xiachuan_zhiling;

//数据删除指令
typedef struct data_delete_zhiling{
    QByteArray data_type;
    QByteArray data_tiaojian;
    QByteArray data_startTime;
    QByteArray data_endTime;
}data_delete_zhiling;

//软件升级维护指令
typedef struct software_shengji_weihu_zhiling{
    QByteArray software_zaixianshengjikongzhi_zhiling;
    QByteArray select_software_edition_num;
}software_shengji_weihu_zhiling;

//磁探控制指令
typedef struct citan_control_zhiling{
    QByteArray beifen;
    QByteArray citan_control;
    QByteArray zidingyi_control;
}citan_control_zhiling;

//磁探收放装置控制指令
typedef struct citan_shoufang_zhuangzhi_control_zhiling{
    QByteArray citan_shoufang_zhuangzhi_control;
}citan_shoufang_zhuangzhi_control_zhiling;

//浮标状态控制指令
typedef struct fubiao_state_control_zhiling{
    QByteArray control_zhiling_source;
    QByteArray mingling_biaoshi;
    QByteArray source_platform_num;
    QByteArray target_platform_num;
}fubiao_state_control_zhiling;

//浮标控制指令
typedef struct fubiao_control_zhiling{
    QByteArray source_data_set;
    QByteArray beifen;
    QByteArray source_platform_num;
    QByteArray target_platform_num;
    QByteArray data_frame_head;
    QByteArray data_frame_biaoshi;
    QByteArray big_bag_num;
    QByteArray fenbao_sum;
    QByteArray big_bag_data_general_length;
    QByteArray fenbao_num;
    QByteArray fenbao_data_available_length;
    QByteArray data_content;
}fubiao_control_zhiling;

//任务遥控B帧遥控指令
typedef struct mission_yaokong_zhiling_B{
    QByteArray bianma;
    QByteArray data_content;
    QByteArray jiaoyan;
}mission_yaokong_zhiling_B;

//参数查询和加载指令
typedef struct canshu_query_and_load_zhiling{
    QByteArray current_work_canshu_diaoxian;
    QByteArray diaoxian_geshebei_yuzhicanshu;
    QByteArray zongheshepin_gouxinSet;
}canshu_query_and_load_zhiling;

//综合射频重构指令
typedef struct zonghe_shepin_chonggou_zhiling{
    QByteArray new_chonggou_function;
    QByteArray old_chonggou_function;
}zonghe_shepin_chonggou_zhiling;

//多信道通信中继波道参数设置
typedef struct duoxindao_tongxinzhongjibodao_canshuSet{
    QByteArray duoxindao_zhongji_select;
    QByteArray bodao_num;
    QByteArray work_pattern;
    QByteArray work_pinlv;
    QByteArray maxing;
    QByteArray zuhao;
    QByteArray pinduan;
    QByteArray tuan;
    QByteArray tiaosu;
    QByteArray biaohao;
    QByteArray wanghao;
    QByteArray miyaohao;
    QByteArray PRGzhishi;
    QByteArray user_speed;
    QByteArray xinxi_miyao_num;
    QByteArray net_pattern;
    QByteArray diantai_level;
    QByteArray jiamifangshi;
    QByteArray xindao_jieru_fangshi;
    QByteArray language_code_pattern;
}duoxindao_tongxinzhongjibodao_canshuSet;

//多信道通信中继设备地址设置
typedef struct duoxindao_tongxinzhongjishebeidizhi_canshuSet{
    QByteArray duoxindao_zhongji_select;
    QByteArray MAC_Address;
    QByteArray IP_Address;
    QByteArray address_code;
    QByteArray shebei_id;
    QByteArray changjia_id;
    QByteArray shebeiNum_id;
    QByteArray shangji_wanghao;
    QByteArray net_guihuajiedian_num;
    QByteArray yuliu;
}duoxindao_tongxinzhongjishebeidizhi_canshuSet;

//多信道通信中继TOD参数设置
typedef struct duoxindao_tongxinzhongjiTOD_canshuSet{
    QByteArray duoxindao_zhongji_select;
    QByteArray year;
    QByteArray month;
    QByteArray day;
    QByteArray hour;
    QByteArray minute;
    QByteArray second;
}duoxindao_tongxinzhongjiTOD_canshuSet;

//敌我识别应答工作参数参数设置
typedef struct diwoshibie_yingda_gongzuo_canshuSet{
    QByteArray jingmo_control;
    QByteArray changgui_pattern;
    QByteArray xiaomi_control;
}diwoshibie_yingda_gongzuo_canshuSet;

//航管应答工作参数参数设置
typedef struct hangguan_yingda_gongzuo_canshuSet{
    QByteArray canshu_control;
    QByteArray beifen;
    QByteArray ATC_yingdama;
    QByteArray s_pattern_address_num;
    QByteArray hangbanhao_1;
    QByteArray hangbanhao_2;
    QByteArray hangbanhao_3;
    QByteArray hangbanhao_4;
    QByteArray hangbanhao_5;
    QByteArray hangbanhao_6;
    QByteArray hangbanhao_7;
    QByteArray hangbanhao_8;
}hangguan_yingda_gongzuo_canshuSet;

//AIS工作参数参数设置
typedef struct AIS_gongzuo_canshuSet{
    QByteArray kaiguan_control;
}AIS_gongzuo_canshuSet;

//无线电高度剩余高度设置
typedef struct wuxiandian_shengyuheight_set{
    QByteArray beiyong;
    QByteArray beiyong_2;
    QByteArray shengyu_height;
}wuxiandian_shengyuheight_set;

//906数据链工作参数设置
typedef struct shujulian_gongzuocanshu_906_set{
    QByteArray bianhao;
    QByteArray fangshi;
    QByteArray work_pattern;
    QByteArray zhongduan_pattern;
    QByteArray kaiguan_control;
    QByteArray jingmo_fangshi;
    QByteArray work_pindian;
    QByteArray jiami_biaoshi;
    QByteArray net_bianhao;
    QByteArray mission_num_1;
    QByteArray mission_num_2;
    QByteArray mission_num_3;
    QByteArray mission_num_4;
    QByteArray mission_num_5;
    QByteArray mission_num_6;
    QByteArray work_speed;
    QByteArray shixi_process;
    QByteArray state;
}shujulian_gongzuocanshu_906_set;

//JIDS参数设置
typedef struct JIDS_canshu_set{
    QByteArray wangcan_guihuahao;
    QByteArray beifen;
    QByteArray platform_bianshi_num;
    QByteArray jingmo_control;
    QByteArray yingjijiami;
    QByteArray fashetianxian_select;
    QByteArray train_model;
    QByteArray train_kuhao;
    QByteArray train_num;
}JIDS_canshu_set;

//电子侦查工作参数设置指令
typedef struct ele_zhencha_workcanshu_set{
    QByteArray canshu_1;
}ele_zhencha_workcanshu_set;

//雷达侦察工作参数设置指令
typedef struct leida_zhencha_workcanshu_set{
    QByteArray canshu_1;
    QByteArray canshu_2;
    QByteArray canshu_3;
    QByteArray canshu_4;
    QByteArray canshu_5;
    QByteArray canshu_6;
    QByteArray canshu_7;
    QByteArray canshu_8;
    QByteArray canshu_9;
    QByteArray canshu_10;
    QByteArray canshu_11;
    QByteArray canshu_12;
    QByteArray canshu_13;
    QByteArray canshu_14;
    QByteArray canshu_15;
    QByteArray canshu_16;
    QByteArray canshu_17;
    QByteArray canshu_18;
    QByteArray canshu_19;
}leida_zhencha_workcanshu_set;


//通信侦察工作参数设置指令
typedef struct tongxin_zhencha_workcanshu_set{
    QByteArray canshu_1;
    QByteArray canshu_2;
}tongxin_zhencha_workcanshu_set;

//雷达侦查处理参数设置指令
typedef struct leida_zhencha_chulicanshu_set{
    QByteArray chulicanshu_type;
    QByteArray canshu_1;
}leida_zhencha_chulicanshu_set;

//通信侦查处理设置指令
typedef struct tongxin_zhencha_chulicanshu_set{
    QByteArray canshu_1;
}tongxin_zhencha_chulicanshu_set;

//激光惯性部件控制参数设置
typedef struct jiguang_guanxing_bujian_canshu_set{
    QByteArray guangdao_work_state;
    QByteArray kongzhong_duizhun_control;
    QByteArray weixing_work_state;
    QByteArray guandao_zuhe_pattern;
}jiguang_guanxing_bujian_canshu_set;

//激光惯性部件带数据参数设置
typedef struct jiguang_guanxing_bujian_daishuju_canshu_set{
    QByteArray canshu_shezhi_youxiaozi;
    QByteArray rengong_set_weidu;
    QByteArray rengong_set_jingdu;
    QByteArray rengong_set_height;
    QByteArray changya_zhuangdingzhi;
}jiguang_guanxing_bujian_daishuju_canshu_set;

//激光惯性部件安装误差参数1设置
typedef struct jiguang_guanxing_bujian_anzhuang_canshu1_set{
    QByteArray canshu_shezhi_youxiaozi;
    QByteArray guandaofuyang;
    QByteArray guandaohenggun;
    QByteArray guandaohangxiang;
    QByteArray shuangtianxianhangxiang;
    QByteArray shuangtianxianfuyang;
    QByteArray shuangtianxianhenggun;
    QByteArray yidongshijizhun_hengzhou;
    QByteArray yidongshijizhun_zongzhou;
    QByteArray yidongshijizhun_fazhou;
    QByteArray kangganrao_hengzhou;
    QByteArray kangganrao_zongzhou;
    QByteArray kangganrao_fazhou;
    QByteArray celiangxing_hengzhou;
    QByteArray celiangxing_zongzhou;
    QByteArray celiangxing_fazhou;
}jiguang_guanxing_bujian_anzhuang_canshu1_set;

//激光惯性部件安装误差参数2设置
typedef struct jiguang_guanxing_bujian_anzhuang_canshu2_set{
    QByteArray canshu_shezhi_youxiaozi;
    QByteArray yuchazhongxin_hengzhou;
    QByteArray yuchazhongxin_zongzhou;
    QByteArray yuchazhongxin_fazhou;
}jiguang_guanxing_bujian_anzhuang_canshu2_set;

//光纤惯性部件控制参数设置
typedef struct guangqian_guanxing_bujian_kongzhi_canshu_set{
    QByteArray guandao_work_state;
    QByteArray kongzhong_duizhun_control;
    QByteArray weixing_work_state;
    QByteArray guandao_zuhe_pattern;
}guangqian_guanxing_bujian_kongzhi_canshu_set;

//光纤惯性部件带数据参数设置
typedef struct guangqian_guanxing_bujian_daishuju_canshu_set{
    QByteArray canshushezhi_youxiaozi;
    QByteArray rengong_shezhi_weidu;
    QByteArray rengong_shezhi_jingdu;
    QByteArray rengong_shezhi_gaodu;
    QByteArray changya_zhuangding_value;
}guangqian_guanxing_bujian_daishuju_canshu_set;

//光纤惯性部件带数据参数设置
typedef struct guangqian_guanxing_bujian_anzhuang_canshu1_set{
    QByteArray canshu_shezhi_youxiaozi;
    QByteArray guandaofuyang;
    QByteArray guandaohenggun;
    QByteArray guandaohangxiang;
    QByteArray shuangtianxianhangxiang;
    QByteArray shuangtianxianfuyang;
    QByteArray shuangtianxianhenggun;
    QByteArray yidongshijizhun_hengzhou;
    QByteArray yidongshijizhun_zongzhou;
    QByteArray yidongshijizhun_fazhou;
    QByteArray kangganrao_hengzhou;
    QByteArray kangganrao_zongzhou;
    QByteArray kangganrao_fazhou;
    QByteArray celiangxing_hengzhou;
    QByteArray celiangxing_zongzhou;
    QByteArray celiangxing_fazhou;
}guangqian_guanxing_bujian_anzhuang_canshu1_set;

//光纤惯性部件安装误差参数2设置
typedef struct guangxian_guanxing_bujian_anzhuang_canshu2_set{
    QByteArray canshu_shezhi_youxiaozi;
    QByteArray yuchazhongxin_hengzhou;
    QByteArray yuchazhongxin_zongzhou;
    QByteArray yuchazhongxin_fazhou;
}guangxian_guanxing_bujian_anzhuang_canshu2_set;

//维护控制设置
typedef struct weihu_control_set{
    QByteArray sys_id;
    QByteArray enter_state;
    QByteArray weihuguzhang_qingdan_qingqiu;
    QByteArray weihuguzhang_qingdan_fanye;
    QByteArray delete_control;
    QByteArray weihu_zijiance;
}weihu_control_set;

//C段测控链工作参数设置
typedef struct Cduan_cekonglian_set{
    QByteArray shebei_id;
    QByteArray zhukongzhan_id;
    QByteArray kongzhi_pattern;
    QByteArray shangxing_pindian;
    QByteArray xiaxing_pindian;
}Cduan_cekonglian_set;

//U段测控链工作参数设置
typedef struct Uduan_cekonglian_set{
    QByteArray shebei_id;
    QByteArray zhukongzhan_id;
    QByteArray gongzuo_pindian;
    QByteArray jiamizhuangtai;
}Uduan_cekonglian_set;

//遥测数据子帧1A-1
typedef struct yaoce_data_zizhen_1{
    QByteArray synchronization_code_1;
    QByteArray synchronization_code_2;
    QByteArray frame_type;
    QByteArray xuhao;
    QByteArray guandaoduizhundata_youxiaozi;
    QByteArray guandaoduizhun_weidu;
    QByteArray guandaoduizhun_jingdu;
    QByteArray guandaoduizhun_gaodu;
    QByteArray guandaoduizhun_chixuTime;
    QByteArray guandaoduizhun_cicha;
    QByteArray guandaoduizhun_zhenhangxiang;
    QByteArray beifen;
    QByteArray jiaoyanhe;
}yaoce_data_zizhen_1;

//遥测数据子帧1A-2
typedef struct yaoce_data_zizhen_2{
    QByteArray synchronization_code_1;
    QByteArray synchronization_code_2;
    QByteArray frame_type;
    QByteArray xuhao;
    QByteArray shujuyouxiaowei_1;
    QByteArray zhishengji_jingdu;
    QByteArray zhishengji_weidu;
    QByteArray zhishengji_gaodu;
    QByteArray dongxiang_speed;
    QByteArray beixiang_speed;
    QByteArray tianxiang_speed;
    QByteArray fuyangjiao;
    QByteArray henggunjiao;
    QByteArray zhenhangxiangjiao;
    QByteArray hengzhou_jiaosudu;
    QByteArray zongzhou_jiaosudu;
    QByteArray fazhou_jiaosudu;
    QByteArray hengzhou_jiasudu;
    QByteArray zongzhou_jiasudu;
    QByteArray fazhou_jiasudu;
    QByteArray weixing_jingdu;
    QByteArray weixing_weidu;
    QByteArray weixing_gaodu;
    QByteArray weixing_dongxiang_speed;
    QByteArray weixing_beixiang_speed;
    QByteArray weixing_tianxiang_speed;
    QByteArray shouxing_num;
    QByteArray jiaoyanhe;
}yaoce_data_zizhen_2;

//遥测数据子帧1B-1
typedef struct yaoce_data_zizhen_B1{
    QByteArray synchronization_code_1;
    QByteArray synchronization_code_2;
    QByteArray frame_type;
    QByteArray xuhao;
    QByteArray guandaoduizhundata_youxiaozi;
    QByteArray guandaoduizhun_weidu;
    QByteArray guandaoduizhun_jingdu;
    QByteArray guandaoduizhun_gaodu;
    QByteArray guandaoduizhun_chixuTime;
    QByteArray guandaoduizhun_cicha;
    QByteArray guandaoduizhun_zhenhangxiang;
    QByteArray beifen;
    QByteArray jiaoyanhe;
}yaoce_data_zizhen_B1;

//遥测数据子帧1B-2
typedef struct yaoce_data_zizhen_B2{
    QByteArray synchronization_code_1;
    QByteArray synchronization_code_2;
    QByteArray frame_type;
    QByteArray xuhao;
    QByteArray shujuyouxiaowei_1;
    QByteArray zhishengji_jingdu;
    QByteArray zhishengji_weidu;
    QByteArray zhishengji_gaodu;
    QByteArray dongxiang_speed;
    QByteArray beixiang_speed;
    QByteArray tianxiang_speed;
    QByteArray fuyangjiao;
    QByteArray henggunjiao;
    QByteArray zhenhangxiangjiao;
    QByteArray hengzhou_jiaosudu;
    QByteArray zongzhou_jiaosudu;
    QByteArray fazhou_jiaosudu;
    QByteArray hengzhou_jiasudu;
    QByteArray zongzhou_jiasudu;
    QByteArray fazhou_jiasudu;
    QByteArray weixing_jingdu;
    QByteArray weixing_weidu;
    QByteArray weixing_gaodu;
    QByteArray weixing_dongxiang_speed;
    QByteArray weixing_beixiang_speed;
    QByteArray weixing_tianxiang_speed;
    QByteArray shouxing_num;
    QByteArray jiaoyanhe;
}yaoce_data_zizhen_B2;

//遥测数据子帧1C
typedef struct yaoce_data_zizhen_C{
    QByteArray synchronization_code_1;
    QByteArray synchronization_code_2;
    QByteArray frame_type;
    QByteArray xuhao;
    QByteArray beiyong;
    QByteArray daqi_data_youxiaozi;
    QByteArray jiguang_guandaozhuangtai_1;
    QByteArray jiguang_guandaozhuangtai_2;
    QByteArray guangxian_guandaozhuangtai_1;
    QByteArray guangxian_guandaozhuangtai_2;
    QByteArray jiguang_feixingguzhang;
    QByteArray beiyong2;
    QByteArray guangxian_feixingguzhang;
    QByteArray beiyong3;
    QByteArray daqi_feixingguzhang;
    QByteArray absolute_qiya_height;
    QByteArray relative_qiya_height;
    QByteArray zongwen;
    QByteArray jingwen;
    QByteArray zhishikongsu;
    QByteArray zhenkongsu;
    QByteArray shengjiang_speed;
    QByteArray jingya;
    QByteArray zongya;
    QByteArray beifen4;
    QByteArray jiaoyanhe;
}yaoce_data_zizhen_C;

//惯导误差安装有效位
typedef struct guandao_wuchaanzhuang_youxiaowei{
    QByteArray beifen;
    QByteArray anzhuangwuchaqingqiu;
    QByteArray yuchazhongxin_jiguandao_youxiaowei;
    QByteArray celiang_jizaiguandao_youxiaowei;
    QByteArray kangganrao_jizaiguandao_youxiaowei;
    QByteArray yidongshi_jizaiguandao_youxiaowei;
    QByteArray shuangtianxian_henggunwucha_youxiaowei;
    QByteArray shuangtianxian_yangjiao_youxiaowei;
    QByteArray shuangtianxian_pianjiao_youxiaowei;
    QByteArray guandao_youxiaowei;
    QByteArray guandaohengwu_zhuangdingzhi;
    QByteArray hengdaofuyang_zhuangdingzhi;
}guandao_wuchaanzhuang_youxiaowei;

//导航对准数据有效位
typedef struct daohang_duizhun_xinxi_youxiaowei{
    QByteArray duizhunshujulaiyuan;
    QByteArray duizhunzhenhangxiangyouxiao;
    QByteArray duizhuncichayouxiao;
    QByteArray duizhunchixushijianyouxiao;
    QByteArray duizhungaoduyouxiao;
    QByteArray duizhunjingweiduyouxiao;
}daohang_duizhun_xinxi_youxiaowei;

//导航数据有效位定义表
typedef struct daohang_data_youxiaowei_definition_table{
    QByteArray jingweidu_youxiaowei;
    QByteArray fuyangjiao_henggunjiao;
    QByteArray zhenhangxiang;
    QByteArray dongxiang_beixiangSpeed;
    QByteArray zhishengjigaodu;
    QByteArray tianxiangspeed;
    QByteArray jitixisanzhou_jiaosudu;
    QByteArray jitixisanzhou_jiasudu;
    QByteArray fengsu_fengxiang;
    QByteArray qita_daohangcanshu;
    QByteArray weixing_data;
    QByteArray riqi_data;
}daohang_data_youxiaowei_definition_table;

//大气数据有效位定义表
typedef struct daqi_data_youxiaowei_definition_table{
    QByteArray juedui_qiya_gaodu;
    QByteArray xiangdui_qiya_gaodu;
    QByteArray zongwen;
    QByteArray jingwen;
    QByteArray zhishi_kongsu;
    QByteArray zhenkongsu;
    QByteArray shengjiangsudu;
    QByteArray jingya;
    QByteArray zongya;
}daqi_data_youxiaowei_definition_table;

//惯导状态1定义表
typedef struct guandao_zhuangtai_1_definition_table{
    QByteArray daohang_gongzuo;
    QByteArray beidou_jiema;
    QByteArray zhuojian_zhuolu;
    QByteArray guandao_zuhe;
    QByteArray rengong_zidong;
}guandao_zhuangtai_1_definition_table;

//惯导状态2定义表
typedef struct guandao_zhuangtai_2_definition_table{
    QByteArray duizhun_zhuangtai;
    QByteArray rengongduizhuncanshu_request;
    QByteArray rengongduizhuncanshu_chaoxian;
    QByteArray rengongduizhuncanshu_youxiao;
    QByteArray kongzhongduizhun_state;
    QByteArray kongzhongduizhun_qingqiu;
    QByteArray kongzhongshibai_reason;
    QByteArray jianlujiduizhunshibai_reason;
}guandao_zhuangtai_2_definition_table;
//惯导故障定义表
typedef struct guandao_guzhang_definition_table{
    QByteArray guandaobujian_guzhang;
    QByteArray weixingxinhao_wuxiao;
    QByteArray celiangxingweixingtianxian_guzhang;
    QByteArray guandaojunma_wuxiao;
    QByteArray guandao_429_guzhang;
    QByteArray guandao_422_guzhang;
    QByteArray guandao_feikong422;;
    QByteArray guandao_shepin422;
}guandao_guzhang_definition_table;

//大气数据故障定义表
typedef struct daqi_data_guzhang_definition_table{
    QByteArray zhenkongsu_wuxiao;
    QByteArray changyazhuangdinxinhao_wuxiao;
    QByteArray daqishujujisuanji_guzhang;
    QByteArray daqiwenduchuanganqi_guzhang;
    QByteArray jingwechaoxian_jiance;
    QByteArray changyachaoxian_jiance;
}daqi_data_guzhang_definition_table;

//目标数据结构
typedef struct mubiao_data{
    QByteArray youxiaozi;
    QByteArray pihao;
    QByteArray hangsu;
    QByteArray hangxiang;
    QByteArray jingdu;
    QByteArray weidu;
    QByteArray gaodu;
}mubiao_data;

//遥测数据子帧2
typedef struct yaoceshuju_zizhen_2{
    QByteArray synchronization_code_1;
    QByteArray synchronization_code_2;
    QByteArray frame_type;
    QByteArray xuhao;
    QByteArray zhilingleixing;
    QByteArray fangweijiao;
    QByteArray fuyangjiao;
    QByteArray CCDshichangjiao;
    QByteArray IRshichangjiao;
    QByteArray xiangjishichangjiao;
    QByteArray guangdianzaihezhuangtaizi_1;
    QByteArray guangdianzaihezhuangtaizi_2;
    QByteArray guangdianzaihezhuangtaizi_3;
    QByteArray guangdianzaihezhuangtaizi_4;
    mubiao_data mubiao1;
    mubiao_data mubiao2;
    QByteArray beifen;
    QByteArray jiaoyanhe;
}yaoceshuju_zizhen_2;

//光电平台状态字1
typedef struct guangdianpingtai_zhuangtaizi_1{
    QByteArray sifudanyuan_zhuangtai;
    QByteArray hongwaichuanganqi_zhuangtai;
    QByteArray dianshichuanganqi_zhuangtai;
    QByteArray xiangji_state;
    QByteArray tuxiangenzongdanyuan_state;
    QByteArray kongzhiguanlidanyuan_state;
    QByteArray guangdianzaihe_state;
    QByteArray beiyong;
}guangdianpingtai_zhuangtaizi_1;
//光电平台状态字2
typedef struct guangdianpingtai_zhuangtaizi_2{
    QByteArray waifangwei;
    QByteArray waifuyang;
    QByteArray neifangwei;
    QByteArray neifuyang;
    QByteArray fangweituoluo;
    QByteArray fuyangtuoluo;
    QByteArray chuanganqikongzhi;
    QByteArray dianshidianzitouwu;
    QByteArray dianshidianzitou4wu;
}guangdianpingtai_zhuangtaizi_2;
//光电平台状态字3
typedef struct guangdianpingtai_zhuangtaizi_3{
    QByteArray hongwai;
    QByteArray xiangji;
    QByteArray tuxianggenzong;
    QByteArray hongwaizengqiang;
    QByteArray hongwaidianzibianbei;
    QByteArray dianshitouwu;
    QByteArray hongwaizhengxiang;
    QByteArray rongheState;
    QByteArray jiyigenzong_state;
}guangdianpingtai_zhuangtaizi_3;
//光电平台状态字4
typedef struct guangdianpingtai_zhuangtaizi_4{
    QByteArray guangdianzaihe;
    QByteArray danganlingmindu;
    QByteArray genzongchangjing;
    QByteArray tuxiangzhutongdao;
    QByteArray tuxianggenzong_state;
    QByteArray tiaodai_width;
    QByteArray beiyong;
}guangdianpingtai_zhuangtaizi_4;
//遥测数据子帧5
typedef struct yaoceshuju_zizhen_5{
    QByteArray synchronization_code_1;
    QByteArray synchronization_code_2;
    QByteArray frame_type;
    QByteArray xuhao;
    QByteArray wuxiandian_height;
    QByteArray mokuaizaixian_state;
    QByteArray feixingguzhang_menu;
    QByteArray beifen;
    QByteArray dianzizhenchaxitong_state;
    QByteArray leizhengongzuo_state;
    QByteArray tongzhengongzuo_state;
    QByteArray tongzhensaveNum;
    QByteArray leizhensaveNum;
    QByteArray kuancaicunchusaveNum;
    QByteArray JIDSzhongjizhuangtai_1;
    QByteArray JIDSzhongjizhuangtai_2;
    QByteArray JIDSzhongjikongzhiChannel;
    QByteArray JIDSzhongjikongzhiKuHao;
    QByteArray RTTbianshihao;
    QByteArray RTTtiaozhengliang;
    QByteArray JIDS_zhongjishiyuan;
    QByteArray JIDS_zhongjishipin;
    QByteArray JIDS_zhongjishixi;
    QByteArray huolu_1_channel;
    QByteArray huolu_2_channel;
    QByteArray ATCshangtianxian;
    QByteArray ATCxiatianxian;
    QByteArray IFFshangtianxian;
    QByteArray IFFxiatianxian;
    QByteArray beifen1;
    QByteArray jiaoyanhe;
}yaoceshuju_zizhen_5;

//综合射频飞行故障定义表
typedef struct zonghe_shepin_feixing_guzhang_definition_table{
    QByteArray sys_manage_1;
    QByteArray sys_manage_2;
    QByteArray gaodubiao;
    QByteArray UHF_pinduan;
    QByteArray leida_zhencha;
    QByteArray tongxin_zhencha;
    QByteArray JIDSzhongji;
    QByteArray shujulian_906;
    QByteArray duoxindaotongxin_1;
    QByteArray duoxindaotongxin_2;
    QByteArray chuanbozidongshibie;
    QByteArray hanguanyingda;
    QByteArray ADS_B;
    QByteArray diwoshibie;
    QByteArray boduancekonglian;
}zonghe_shepin_feixing_guzhang_definition_table;

//综合射频模块在线状态定义表
typedef struct zonghe_shepin_feixing_zaixian_definition_table{
    QByteArray sys_manage_1;
    QByteArray sys_manage_2;
    QByteArray gaodubiao;
    QByteArray UHF_pinduan;
    QByteArray leida_zhencha;
    QByteArray tongxin_zhencha;
    QByteArray JIDSzhongji;
    QByteArray shujulian_906;
    QByteArray duoxindaotongxin_1;
    QByteArray duoxindaotongxin_2;
    QByteArray chuanbozidongshibie;
    QByteArray hanguanyingda;
    QByteArray ADS_B;
    QByteArray diwoshibie;
    QByteArray boduancekonglian;
}zonghe_shepin_feixing_zaixian_definition_table;

//遥测数据子帧6
typedef struct yaoce_shuju_zizhen_6{
    QByteArray synchronization_code_1;
    QByteArray synchronization_code_2;
    QByteArray frame_type;
    QByteArray xuhao;
    QByteArray yingda_state;
    QByteArray xiangyingzhenNum;
    QByteArray yaoTiaoNum;
    QByteArray yaoTiaoZhenSum;
    QByteArray currentZhenNum;
    QByteArray csys_model_Num;
    QByteArray content;
    QByteArray jiaoyanHe;
}yaoce_shuju_zizhen_6;

//雷达应答指令应答内容
typedef struct leida_yingda_zhiling_content{
    QByteArray info_biaoqian;
    QByteArray message_state;
}leida_yingda_zhiling_content;

//JIDS网参规划参数上报
typedef struct JIDS_wangcan_guihuacanshu_upload{
    QByteArray net_design_num_1_1;
    QByteArray net_design_num_1_2;
    QByteArray net_design_num_2_1;
    QByteArray net_design_num_2_2;
    QByteArray net_design_num_3_1;
    QByteArray net_design_num_3_2;
    QByteArray net_design_num_4_1;
    QByteArray net_design_num_4_2;
}JIDS_wangcan_guihuacanshu_upload;

//备份
//包头
typedef struct beifen_baotou{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray gudingzhi;
    QByteArray bag_count;
    QByteArray length;
    QByteArray shuxing;
    QByteArray pianyiliang;
}beifen_baotou;

//单元
typedef struct beifen_danyuan{
    QByteArray main_type;
    QByteArray second_type;
    QByteArray frame_count;
    QByteArray frame_num;
    QByteArray shuxing;
    QByteArray count;
    QByteArray length;
    QByteArray jiaoyan;
    QByteArray yuliu;
    QByteArray next_pianyiliang;
    QByteArray content_data;
}beifen_danyuan;

//图像/视频帧格式
typedef struct image_video_frame{
    QByteArray main_type;
    QByteArray second_type;
    QByteArray frame_count;
    QByteArray shichuo;
    QByteArray zaiji_jingdu;
    QByteArray zaiji_weidu;
    QByteArray zaiji_height;
    QByteArray shuxing;
    QByteArray yuliu;
    QByteArray start_pianyi;
    QByteArray yuliu2;
    QByteArray fei_image_data;
    QByteArray image_data;
}image_video_frame;

//数据帧格式
typedef struct data_frame{
    QByteArray main_type;
    QByteArray second_type;
    QByteArray frame_count;
    QByteArray shichuo;
    QByteArray zaiji_jingdu;
    QByteArray zaiji_weidu;
    QByteArray zaiji_height;
    QByteArray yuliu;
    QByteArray data;
}data_frame;

//遥测数据子帧1
typedef struct yaoce_data_frame_1{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray frame_count;
    QByteArray fuyangjiao;
    QByteArray gunzhuanjiao;
    QByteArray hangxiangjiao;
    QByteArray fuyangjiao_speed;
    QByteArray gunzhuanjiao_speed;
    QByteArray pianhangjiao_speed;
    QByteArray jiti_zongzhou_speed;
    QByteArray jiti_hengzhou_speed;
    QByteArray jiti_fazhou_speed;
    QByteArray zhenkongsu;
    QByteArray jiti_zongxiang_groundSpeed;
    QByteArray jiti_cexiang_groundSpeed;
    QByteArray chuizhi_speed;
    QByteArray absolute_height;
    QByteArray relative_height;
    QByteArray empty;
    QByteArray chaoshengbo_height;
    QByteArray jiaoyanhe;
}yaoce_data_frame_1;

//遥测数据子帧2
typedef struct yaoce_data_frame_2{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray zongxiang_speed_control;
    QByteArray hengxiang_speed_control;
    QByteArray zongxiang_position;
    QByteArray cexiang_position;
    QByteArray height_control;
    QByteArray chuizhi_speed_control;
    QByteArray beiyong;
    QByteArray pianhangjiao_control;
    QByteArray jingdu;
    QByteArray weidu;
    QByteArray empty;
    QByteArray cepianju;
    QByteArray daifeiju;
    QByteArray feixing_kaiguan_huibao;
    QByteArray yaokong_zhiling_result;
    QByteArray jiaoyanhe;
}yaoce_data_frame_2;

//遥测数据子帧3A
typedef struct yaoce_data_frame_3A{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray zongxiang_zhouqi_bianju;
    QByteArray hengxiang_zhouqi_bianju;
    QByteArray weijiangju;
    QByteArray zongju;
    QByteArray zongju_youmen_kaidu;
    QByteArray duli_youmen_kaidu;
    QByteArray lianxu_zhiling;
    QByteArray fengsu;
    QByteArray fengxiang;
    QByteArray weixing_height;
    QByteArray current_hangxiang_num;
    QByteArray current_hangdian_num;
    QByteArray chongguihua_result;
    QByteArray online_guihua_hangxian;
    QByteArray jiaoyanhe;
}yaoce_data_frame_3A;

//遥测数据子帧3B
typedef struct yaoce_data_frame_3B{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray zhuojian_zongxiang_position;
    QByteArray zhuojian_cexiang_position;
    QByteArray zhuojian_zongxiang_speed;
    QByteArray zhuojian_cexiang_speed;
    QByteArray zhuojian_chuixiang_speed;
    QByteArray zhuojian_relative_height;
    QByteArray chafen_shuiping_wucha;
    QByteArray chafen_chuizhi_wucha;
    QByteArray zhuojian_guiji_cexiang;
    QByteArray zhuojian_height;
    QByteArray zhuojian_stage;
    QByteArray zhuojian_info_source;
    QByteArray jingxiqi_yucezhi;
    QByteArray jiguang_guandao_A;
    QByteArray jiguang_guandao_B;
    QByteArray guangxian_guandao_A;
    QByteArray guangxian_guandao_B;
    QByteArray jianchuan_hangxiangjiao;
    QByteArray zhuojian_zhiling_result;
    QByteArray yindaoshebei_state;
    QByteArray leidayindao_state;
    QByteArray jiaoyanhe;
}yaoce_data_frame_3B;

//遥测数据子帧4A
typedef struct yaoce_data_frame_4A{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray plane_address;
    QByteArray station_address;
    QByteArray first_control_state;
    QByteArray second_control_state;
    QByteArray third_control_state;
    QByteArray fourth_control_state;
    QByteArray yingfei_hangxiang;
    QByteArray yingjiguzhanghandle_model;
    QByteArray yingjiguzhanghandle_fangan;
    QByteArray qiya_height;
    QByteArray feikongjisuanji_guzhangzi;
    QByteArray zhuweijiangduoji_guzhangzi;
    QByteArray sifukongzhiqi_guzhangzi;
    QByteArray youmenduoji_guzhangzi;
    QByteArray youmenkongzhiqi_guzhangzi;
    QByteArray shengyupanxuan_quanshu_time;
    QByteArray fuyangduoji_dianliu;
    QByteArray fuyangduoji_wendu;
    QByteArray PBTT_wufatouru;
    QByteArray PBTTjiancejindu;
    QByteArray PUBIT_result;
    QByteArray feixing_zijieduan;
    QByteArray jiaoyanhe;
}yaoce_data_frame_4A;

//遥测数据子帧4B
typedef struct yaoce_data_frame_4B{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray shebeilianlu_zhuangtaizi_1;
    QByteArray shebeilianlu_zhuangtaizi_2;
    QByteArray shebeilianlu_zhuangtaizi_3;
    QByteArray chuanganqi_guzhangzi_1;
    QByteArray chuanganqi_guzhangzi_2;
    QByteArray chuanganqi_guzhangzi_3;
    QByteArray dongli_zhuangtai_1;
    QByteArray dongli_zhuangtai_2;
    QByteArray beiyong;
    QByteArray xuanyixitong_zhuangtaizi;
    QByteArray hangdian_mission_zhuangtaizi;
    QByteArray fadongji_zhuangtaizi;
    QByteArray fuyangduoji_zhuangtaizi;
    QByteArray zuohenggunduo_fankui;
    QByteArray youhenggunduo_fankui;
    QByteArray hangxiangduoji_fankui;
    QByteArray zongjuyoumen;
    QByteArray duliyoumen;
    QByteArray beiyong2;
    QByteArray jidianxitong_zhuangtaizi;
    QByteArray chuanganqi_info_source_1;
    QByteArray chuanganqi_info_source_2;
    QByteArray chuanganqi_info_source_3;
    QByteArray lisan_shuchu_1;
    QByteArray lisan_shuchu_2;
    QByteArray lisan_shuchu_3;
    QByteArray lisan_shuchu_4;
    QByteArray jiaoyanhe;
}yaoce_data_frame_4B;

//遥测数据子帧4C
typedef struct yaoce_data_frame_4C{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray ranqifashengqi_zhuansu;
    QByteArray dongliwolun_zhuansu;
    QByteArray xuanyi_zhuansu;
    QByteArray fadongji_huayou_yali;
    QByteArray gongyouguan_ranyou_yali;
    QByteArray fadongji_huayou_wendu;
    QByteArray zhujianhuayou_yali;
    QByteArray zhujianhuayou_wendu;
    QByteArray weijianhuayou_wendu;
    QByteArray qianyouxiang_zongyouliang;
    QByteArray houyouxiang_zongyouliang;
    QByteArray zongyouliang;
    QByteArray feikongxitong_dianyuandianya;
    QByteArray T45wendujisuanzhi;
    QByteArray fadongji_niuju;
    QByteArray wuxiandian_height;
    QByteArray jiaoyanhe;
}yaoce_data_frame_4C;

//遥测数据子帧4D
typedef struct yaoce_data_frame_4D{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray chuanshu_content_result;
    QByteArray yaotiao_zhiling_huibao;
    QByteArray yaotiao_canshu_huibao;
    QByteArray jiaoyanhe;
}yaoce_data_frame_4D;

//遥调指令响应结果定义表
typedef struct yaotiao_zhiling_xiangying_result_table{
    QByteArray content_type;
    QByteArray zhixing_result;
}yaotiao_zhiling_xiangying_result_table;

//起飞检查指令汇报内容
typedef struct qifei_tiaojian_huibao_content{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray chuanshu_content_result;
    QByteArray yaotiaozhiling_huibao;
    QByteArray qifeijiaojian_jiancha;
    QByteArray zhengchanghangxian_hefaxing;
    QByteArray renwuhangxian_hefaxing;
    QByteArray fanhanghangxian_hefaxing;
    QByteArray zhuolufufeihangxian_hefaxing;
    QByteArray dongli;
    QByteArray xuanyi;
    QByteArray hangdian;
    QByteArray jidian;
    QByteArray feikong;
    QByteArray beiyong;
    QByteArray beiyong2;
    QByteArray jiaoyanhe;
}qifei_tiaojian_huibao_content;

//4D航线长度查询指令回报
typedef struct hangxian_query_zhiling_huibao_4D{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray chuanshu_content_result;
    QByteArray yaotiaozhiling_huibao;
    QByteArray hangxian_hao;
    QByteArray hangxian_length;
    QByteArray beiyong;
    QByteArray jiaoyanhe;
}hangxian_query_zhiling_huibao_4D;

//4D绝对航点查询指令回报
typedef struct hangdian_query_zhiling_huibao_4D{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray chuanshu_content_result;
    QByteArray yaotiaozhiling_huibao;
    QByteArray hangxian_hao;
    QByteArray hangdian_hao;
    QByteArray jingdu;
    QByteArray weidu;
    QByteArray haiba_height;
    QByteArray speed;
    QByteArray next_hangxian_hao;
    QByteArray next_hangdian_hao;
    QByteArray tezhengzi;
    QByteArray beiyong;
    QByteArray beiyong2;
    QByteArray jiaoyanhe;
}hangdian_query_zhiling_huibao_4D;

//4D相对航点查询指令回报
typedef struct xiangduihangdian_query_zhiling_huibao_4D{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray chuanshu_content_result;
    QByteArray yaotiaozhiling_huibao;
    QByteArray hangxian_hao;
    QByteArray hangdian_hao;
    QByteArray hangdian_hengzuobiao;
    QByteArray hangdian_zongzuobiao;
    QByteArray hangdian_xiangduigaodu;
    QByteArray speed;
    QByteArray beiyong;
    QByteArray beiyong2;
    QByteArray tezhengzi;
    QByteArray beiyong3;
    QByteArray beiyong4;
    QByteArray jiaoyanhe;
}xiangduihangdian_query_zhiling_huibao_4D;

//4D相对航点查询指令回报_2
typedef struct xiangduihangdian_query_zhiling_huibao_4D_2{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray chuanshu_content_result;
    QByteArray yaotiaozhiling_huibao;
    QByteArray jianchuan_bianma;
    QByteArray guandao_biaoshi;
    QByteArray canshu_num;
    QByteArray ganbi_canshu;
    QByteArray ganbi_canshu2;
    QByteArray ganbi_canshu3;
    QByteArray ganbi_canshu4;
    QByteArray ganbi_canshu5;
    QByteArray beiyong;
    QByteArray jiaoyanhe;
}xiangduihangdian_query_zhiling_huibao_4D_2;

//5A遥测数据子帧
typedef struct yaoce_shuju_zizhen_5A{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray year;
    QByteArray month;
    QByteArray day;
    QByteArray hour;
    QByteArray minute;
    QByteArray second;
    QByteArray hangzi_zhuangtaizi;
    QByteArray guzhangzi;
    QByteArray fuyangjiao;
    QByteArray henggunjiao;
    QByteArray hangxiangjiao;
    QByteArray hengzhou_jiasudu;
    QByteArray zongzhou_jiasudu;
    QByteArray fazhou_jiasudu;
    QByteArray qianyouliang_chuanganqi;
    QByteArray houyouliang_chuanganqi;
    QByteArray zhujianzhendong_jiance;
    QByteArray jiaoyanhe;
}yaoce_shuju_zizhen_5A;

//5B遥测数据子帧
typedef struct yaoce_shuju_zizhen_5B{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray jiguang_guandao_zhuangtaizi1;
    QByteArray jiguang_guandao_zhuangtaizi2;
    QByteArray jiguang_guandao_youxiaozi;
    QByteArray jiguang_guandao_guzhangzi;
    QByteArray guangxian_guandao_zhuangtaizi1;
    QByteArray jiguangchafen;
    QByteArray guangxian_guandao_youxiaozi;
    QByteArray guangxian_guandao_guzhangzi;
    QByteArray daqi_wugao_zhuangtaizi;
    QByteArray jiguang_chafen_guzhangzi1;
    QByteArray zhujiang_zhuangtaizi1;
    QByteArray zhujiang_zhuangtaizi2;
    QByteArray work_time;
    QByteArray zhujiang_pos;
    QByteArray zhuojian_cishu;
    QByteArray HUMS_zhuangtai_1;
    QByteArray weixianji_xinhao;
    QByteArray gaojinji_xinhao;
    QByteArray jiguang_chafen_guzhangzi_2;
    QByteArray zhuyiji_xinhao_1;
    QByteArray zhishiji_xinhao;
    QByteArray jiguang_chafen_suanshuxing;
    QByteArray jiaoyanhe;
}yaoce_shuju_zizhen_5B;

//5C遥测数据子帧
typedef struct yaoce_shuju_zizhen_5C{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray CLP_weizhi;
    QByteArray gonglv_yudu;
    QByteArray wendu_yudu;
    QByteArray ECU_kaiguanliang;
    QByteArray beiyong;
    QByteArray ECU_guzhang_1;
    QByteArray ECU_guzhang_2;
    QByteArray ECU_gaojingzi_1;
    QByteArray ECU_zhuangtaizi;
    QByteArray ECU_gaojingzi_2;
    QByteArray beiyong2;
    QByteArray dongli_chuandong_gouxing;
    QByteArray jiguang_guandao_weixingshu;
    QByteArray guangxian_guandao_weixingshu;
    QByteArray jiguang_guangxian_chafenweixing;
    QByteArray beiyong3;
    QByteArray qidong_fadongji;
    QByteArray HUMS_zhuangtaizi_2;
    QByteArray beiyong4;
    QByteArray yingdaji_zhuangtaizi;
    QByteArray jiaoyanhe;
}yaoce_shuju_zizhen_5C;

//5D遥测数据子帧
typedef struct yaoce_shuju_zizhen_5D{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray yingji_huiliutiao_dianya;
    QByteArray renwu_huiliutiao_dianya;
    QByteArray fadianji_dianya;
    QByteArray fadianji_dianliu;
    QByteArray waidianyuan_dianya;
    QByteArray dianchizu_1_zongdianya;
    QByteArray dianchizu_1_zongdianliu;
    QByteArray dianchizu_1_zongwen;
    QByteArray dianchizu_1_dangqianrongliang;
    QByteArray dianchizu_2_zongdianya;
    QByteArray dianchizu_2_zongdianliu;
    QByteArray dianchizu_2_zongwen;
    QByteArray dianchizu_2_dangqianrongliang;
    QByteArray jiechuqi_zhuangtaizi;
    QByteArray daqi_zongwen;
    QByteArray daqi_jingwen;
    QByteArray beiyong1;
    QByteArray beiyong2;
    QByteArray beiyong3;
    QByteArray jiaoyanhe;
}yaoce_shuju_zizhen_5D;

//5E遥测数据子帧
typedef struct yaoce_shuju_zizhen_5E{
    QByteArray synchronization_code;
    QByteArray frame_type;
    QByteArray dianchigongzuo_zhuangtaizi;
    QByteArray PFL_guzhangzi_1;
    QByteArray PFL_guzhangzi_2;
    QByteArray PFL_guzhangzi_3;
    QByteArray PFL_guzhangzi_4;
    QByteArray SSPC_channel_zhuangtaizi_1;
    QByteArray SSPC_channel_zhuangtaizi_2;
    QByteArray SSPC_channel_zhuangtaizi_3;
    QByteArray SSPC_channel_zhuangtaizi_4;
    QByteArray SSPC_channel_zhuangtaizi_5;
    QByteArray SSPC_channel_zhuangtaizi_6;
    QByteArray SSPC_channel_zhuangtaizi_7;
    QByteArray SSPC_channel_zhuangtaizi_8;
    QByteArray SSPC_channel_zhuangtaizi_9;
    QByteArray SSPC_channel_zhuangtaizi_10;
    QByteArray SSPC_channel_zhuangtaizi_11;
    QByteArray SSPC_channel_zhuangtaizi_12;
    QByteArray SSPC_channel_zhuangtaizi_13;
    QByteArray SSPC_channel_zhuangtaizi_14;
    QByteArray SSPC_channel_zhuangtaizi_15;
    QByteArray SSPC_channel_zhuangtaizi_16;
    QByteArray SSPC_channel_zhuangtaizi_17;
    QByteArray SSPC_channel_zhuangtaizi_18;
    QByteArray beifen;
    QByteArray jiaoyanhe;
}yaoce_shuju_zizhen_5E;
#endif // JBYKZL_H
