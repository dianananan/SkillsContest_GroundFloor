/**
工程名称：2019主车综合程序			
修改时间：2019.5.11
*/
#include <stdio.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "infrared.h"
#include "cba.h"
#include "ultrasonic.h"
#include "canp_hostcom.h"
#include "hard_can.h"
#include "bh1750.h"
#include "syn7318.h"
#include "power_check.h"
#include "can_user.h"
#include "data_base.h"
#include "roadway_check.h"
#include "tba.h"
#include "data_base.h"
#include "swopt_drv.h"
#include "uart_a72.h"
#include "Can_check.h"
#include "can_user.h"
#include "Timer.h"
#include "Rc522.h"
#include "TaskWifiChannel.h"
#include "TaskZigbeeChannel.h"
#include "Init.h"

RCC_ClocksTypeDef RCC_Clocks;

uint16_t main_cont;
u16 DataSum=0;

//u8 array_test[10];

/**
函数功能：硬件初始化
参    数：无
返 回 值：无
*/
void Hardware_Init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);					//中断分组

	delay_init(168);
	
	Tba_Init();														//任务板初始化
	Infrared_Init();												//红外初始化
	Cba_Init();														//核心板初始化
	Ultrasonic_Init();												//超声波初始化
	Hard_Can_Init();												//CAN总线初始化
	BH1750_Configure();												//BH1750初始化配置
	SYN7318_Init();													//语音识别初始化
	Electricity_Init();												//电量检测初始化

	UartA72_Init();
	Can_check_Init(83,7);  //8us											//CAN总线定时器初始化
	
	roadway_check_TimInit(167,2000-1);//4ms 延时时间过短会妨碍can总线上的码盘和循迹灯	//路况检测
	
	Timer_Init(167,999);										    //串行数据通讯时间帧
	Readcard_daivce_Init();											//RFID初始化
}


uint8_t open_road_buf[] = {0x55,0x03,0x01,0x01,0x00,0x00,0x02,0xBB};			//道闸测试
uint8_t test_buf[] = {0xFD,0x00,0x06,0x01,0x01,0xC4,0xFA,0xBA,0xC3};			//语音播报“您好”
uint8_t repo_buf[] = {0x03,0x05,0x14,0x45,0xDE,0x92};							//打开红外报警

	
/**
函数功能：按键检测
参    数：无
返 回 值：无
*/
uint16_t Light_Value = 4564;	
void KEY_Check()
{
	if(S1==0){	
		while(S1==0);
//		startRun();
//		Track_Test(CARSPEED, ZERO, ROADONE, ROADMODE); 
		TaskBoardTest(1);
	}
	if(S2==0){
		while(S2==0);
//		Send_WifiData_To_Fifo(QRCode, 8);
		TaskBoardTest(2);
	}
	if(S3==0){
		while(S3==0);
//		Send_WifiData_To_Fifo(ShapeRead, 8);
		TaskBoardTest(3);
	}
	if(S4==0){
		while(S4==0);
//		Send_WifiData_To_Fifo(TrafficLight, 8);
		TaskBoardTest(4);
	}	
}



static uint32_t Power_check_times;		  //电量检测周期
static uint32_t LED_twinkle_times;		  //LED闪烁周期
static uint32_t WIFI_Upload_data_times;   //通过Wifi上传数据周期
static uint32_t RFID_Init_Check_times;

int main(void)
{	
//	uint16_t Light_Value = 0;				//光强度值	
	uint16_t CodedDisk_Value = 0;			//码盘
	uint16_t Nav_Value = 0;					//角度
	u8 sum=31;
	
	Hardware_Init();						//硬件初始化
	
	LED_twinkle_times =  gt_get() + 50;     
	Power_check_times =  gt_get() + 1000;
	WIFI_Upload_data_times = gt_get() + 200;
	RFID_Init_Check_times = gt_get()+200;
	
	Principal_Tab[0] = 0x55;
	Principal_Tab[1] = 0xAA;	
	
	Follower_Tab[0] = 0x55;
	Follower_Tab[1] = 0x02;
	
	Send_UpMotor(0 ,0);
//*******************************************************************
	WifiSignal_Rx_Init();
	NowTaskPot=0;
	Send_InfoData_To_Fifo((u8 *)"OK\n", sizeof("OK\n"));
	delay_ms(5);	//延时启动
	STOP();
	runtimeInit();  //初始化任务标志位
	initStartCoord(1, 0, 1); //设置起始位置
//    xydInit(&passivity.TerrainPot, 5, 3, 1); //设置地形标物位置
//    xydInit(&passivity.TrafficPot, 5, 4, 3); //设置交通灯标物位置G4
//    xydInit(&passivity.ETCPot, 4, 1, 0); //设置ETC标物位置  E2
//    xydInit(&passivity.BarrierszPot, 2, 5, 0); //设置道闸位置
//    xydInit(&passivity.RFIDCard, 4, 5, 0); //设置RFID位置
	InitDataBase();   //初始化默认的数据//超声波//光源挡位//车库几层
//	Host_Set_UpTrack(50);  //更新数据上传时间
//******************************************************************	
	while(1)
	{
		KEY_Check();//按键检测
//		Zigbee_Rev_Control();//zigbee接收控制
//		Wifi_Remote_Control ();//WIFI接收控制
        runControl();//路线行驶控制
        RunTaskControl();
		
		Can_WifiRx_Check();
		Can_ZigBeeRx_Check();
//=============================================22点注释
//		if(RFID_S50.RFID_Mode != SLEEP)	//寻卡
//		{
//			Read_Card();
//		}
		if(gt_get_sub(Power_check_times) == 0) 			
		{
			Power_check_times =  gt_get() + 1000;		//电池电量检测
			Power_Check();		
		} 
		
//		
//		#if 0
//		if(gt_get_sub(LED_twinkle_times) == 0) 			
//		{
//			LED_twinkle_times =  gt_get() + 50;			//LED4状态取反
//			LED4 = !LED4;
//		} 
//		#endif
//				
//		
//		#if 1
//		if(gt_get_sub(RFID_Init_Check_times) == 0) 			
//		{
//			RFID_Init_Check_times =  gt_get() + 200;	//RFID初始化检测
//			if(Rc522_GetLinkFlag() == 0)					
//			{
//				Readcard_daivce_Init();
//				MP_SPK = !MP_SPK;
//			} else {
//				MP_SPK = 0;
//				LED4 = !LED4;
//				Rc522_LinkTest();
//			}
//		} 
//		#endif
//		if()
//		
//
//		if(gt_get_sub(WIFI_Upload_data_times) == 0)  //看门狗？？？
//		{
//			WIFI_Upload_data_times =  gt_get() + 200;
//		
//			if(Host_AGV_Return_Flag == RESET)
//			{
//				Principal_Tab[2] = Stop_Flag;				//运行状态
//				Principal_Tab[3] = Get_tba_phsis_value();	//光敏状态值返回
//				
//				Ultrasonic_Ranging();						//超声波数据						
//				Principal_Tab[4]=dis%256;        
//				Principal_Tab[5]=dis/256;
//				
//				Light_Value = Get_Bh_Value();				//光强度传感器	
//				Principal_Tab[6]=Light_Value%256;	    	//光照数据
//				Principal_Tab[7]=Light_Value/256;
//				
//				CodedDisk_Value = CanHost_Mp;				//码盘
//				Principal_Tab[8]=CodedDisk_Value%256;	    	
//				Principal_Tab[9]=CodedDisk_Value/256;
//				
//				Nav_Value = CanHost_Navig;					//角度
//				Principal_Tab[10]=Nav_Value%256;	    	
//				Principal_Tab[11]=Nav_Value/256;
//				
//				Send_WifiData_To_Fifo(Principal_Tab,12);
//				UartA72_TxClear();
//				UartA72_TxAddStr(Principal_Tab,12);
//				UartA72_TxStart();
//			} else if((Host_AGV_Return_Flag == SET) && (AGV_data_Falg == SET)){
//				
//				UartA72_TxClear();
//				UartA72_TxAddStr(Follower_Tab,50);
//				UartA72_TxStart();
//				Send_WifiData_To_Fifo(Follower_Tab,50);
//				AGV_data_Falg = 0;
//			}
//		}
	}		
}

