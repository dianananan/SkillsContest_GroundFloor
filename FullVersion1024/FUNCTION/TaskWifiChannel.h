#ifndef __TASKWIFICHANNEL_H
#define __TASKWIFICHANNEL_H

#include "sys.h"
#include "PathPlanning.h"

#define  PLATENUM	8//车牌

#define  ZCKZ_ADDR	0xAA				//主车控制
#define  CCZ_ADDR	0x02        //从车控制

#define  RUNSTART	0x01    				//小车启动
#define  QRCODE		0x02						//二维码识别
#define  PLATEREV1	0x10						//车牌识别
#define  PLATEREV2	0x11						//车牌识别
#define	 SHAPE		0x04						//图形识别
#define  TRAFFIC    0x08						//红绿灯
//#define	 TRAFFIC	0x05					  //交通灯识别
//#define  POINT	0x06						//坐标代号

extern u8 wifi_send_PLATE_flag;//车牌识别点已到达
extern u8 wifi_send_QR_flag;//wifi发送二维码读命令给平板  0:init 1:car->pad 2:pad->car
extern u8 wifi_send_SHAPE_flag;
extern u8 wifi_rev_card_flag_1;

extern u8 Timeout;
extern u8 wifitask;

extern u8 GETPONT[3];//wifi接收坐标
extern u8 qrread[1];
//extern u8 shapedata[3];//图形识别数据
extern u8 trafficread[1];
extern u8 taskover;//任务点已完成

//void Wifi_Rev_Control(void);
void Wifi_Send_Dispose(u8 Wifi_signal);
void Wifi_Send_Dispose_Wifi(u8 *Wifi_signal);
//void switchBackinfo(u8 vaule);//判断上层的回传信息
void hw_Data_Dispose(u8 *Data); //处理立体显示的红外信息
void Wifi_Send_Dispose_zigbee(u8 *Wifi_signal);  //处理zigbee信息 //道闸和//红绿灯特别处理
void Wifi_Send_Dispose_hw(u8 *Wifi_signal);  //处理红外信息 
void Wifi_Send_Dispose_run(u8 *Wifi_signal);//处理行走命令
void Wifi_Send_Dispose_action(u8 *Wifi_signal);//处理动作
void Wifi_Send_Dispose_Car(u8 *Wifi_signal);  //小车电原件处理
void Wifi_Remote_Control (void); //wifi信号接收	
#endif
