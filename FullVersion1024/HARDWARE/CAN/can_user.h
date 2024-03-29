#include "stm32f4xx.h"
#ifndef __CAN_USER_H__
#define __CAN_USER_H__

#ifdef __CAN_USER_C__
#define GLOBAL
#else
#define GLOBAL extern
#endif

#define  ZIGB_RX_MAX    200
#define  WIFI_MAX_NUM   200

struct Signal	//信号接收结构体
{
	u8 StackData[8];
	struct Signal *Next;
};
extern struct Signal WifiSignalQueue[5];
extern struct Signal *Head; //头指针
extern struct Signal *Tail; //尾指针

//extern uint8_t Wifi_Rx_Buf[ WIFI_MAX_NUM ];
extern uint8_t Zigb_Rx_Buf[ ZIGB_RX_MAX ];
extern uint8_t Wifi_Rx_num;
extern uint8_t Wifi_Rx_flag;  //接收完成标志位
extern uint8_t Zigbee_Rx_num;
extern uint8_t Zigbee_Rx_flag;  //接收完成标志位

extern uint8_t Host_AGV_Return_Flag;
extern uint8_t AGV_data_Falg;
extern uint32_t canu_wifi_rxtime;
extern uint32_t canu_zibe_rxtime;

void WifiSignal_Rx_Init(void);//设置wifi接收初始化

GLOBAL void Canuser_Init(void);
GLOBAL void Canuser_main(void);

GLOBAL void Can_WifiRx_Save(uint8_t res);
GLOBAL void Can_ZigBeeRx_Save(uint8_t res);

u8 Normal_data(u8 mode, u8 *buff);  //校验mode1-->wifi	2-->zigbee
void Can_WifiRx_Check(void);
void Can_ZigBeeRx_Check(void);
void Red_Card_Track(uint8_t sp);
extern uint8_t ETC_open_Flag;
extern void ETC_move(uint8_t sp,uint16_t mp);
void Full_End_Car(void);

#undef GLOBAL


#endif //__CAN_USER_H__

