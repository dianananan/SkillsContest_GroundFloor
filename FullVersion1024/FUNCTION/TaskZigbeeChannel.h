#ifndef __ZIGBEE_H
#define __ZIGBEE_H
#include "sys.h"

void Zigbee_Rev_Control(void);
void Zig_Send_Dispose(u8 );

void TACKZERO(void); //行走延时
void SendVoice(u8 *data, int length);  //语音播报
void send_TFT_data(u8 mode);//选择TFT任务
void send_SEG_data(u8 mode);  //选择数码管任务


extern u8 zig_send_CK_flag;//车库状态
//extern u8 Now_Plies; //车库几层
//extern u8 TriggerF_B;  //前后红外触发状态 =0全触发  =1后触发 =2前触发 =4没触发
extern u8 zig_send_door_Rev_flag;
extern u8 zig_send_voice_Rev_flag;
extern u8 zig_send_etc_flag;
//extern u8 zig_send_smg_Rev_flag;

//计时模式
//关闭计时器
static u8 countclose[8] = {0x55,0x0b,0x30,0x00,0x00,0x00,0x30,0xbb};
//打开计时器
static u8 countopen[8] = {0x55,0x0b,0x30,0x01,0x00,0x00,0x31,0xbb};
//计时器清零
static u8 countclean[8] = {0x55,0x0b,0x30,0x02,0x00,0x00,0x32,0xbb};


//由第二副指令指定显示哪张图片
static u8 witch_pic[8] = {0x55,0x0b,0x10,0x00,0x01,0x00,0x11,0xbb};
//图片上翻
static u8 pic_up[8] = {0x55,0x0b,0x10,0x01,0x00,0x00,0x11,0xbb};
//图片下翻
static u8 pic_down[8] = {0x55,0x0b,0x10,0x02,0x00,0x00,0x12,0xbb};
//图片自动向下翻页显示，间隔时间10s
static u8 auto_show[8] = {0x55,0x0b,0x10,0x03,0x00,0x00,0x13,0xbb};
/*-----------------TFT显示器模块----------------------*/
#define PLATEMODE	1	//车牌显示模式
#define	HEXSHOW		2	//十进制显示模块
#define DISMODE		3	//距离显示模式


/*-----------------图片显示--------------------------*/
#define WITCH_PIC	4		//由第二副指令指定显示哪张图片
#define PIC_UP		5		//图片上翻
#define PIC_DOWN	6		//图片下翻
#define AUTOMATION_PIC	7	//图片自动向下翻页显示，间隔时间10S


/*-----------------计时器模式------------------------*/
#define CLOSE_COUNT	8	//计时模式关闭
#define OPEN_COUNT	9	//计时模式打开
#define CLEAR_COUNT	10	//计时模式清零


#define OPEN_SEG  1 //数码管计时开
#define CLOSE_SEG 2 //数码管计时关
#define SHOW1_SEG 3	//显示第一排
#define SHOW2_SEG 4	//显示第二排
#define DIS_SEG	  5	//显示距离

//车牌显示模式  前三位
extern u8 plateA[8];

//车牌显示模式  后三位
extern u8 plateB[8];

//六位显示数据(hex格式)
extern u8 sixbits[8];

//距离显示模式（十进制）
extern u8 disshow[8];

#endif
