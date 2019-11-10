/**
�������ƣ�2019�����ۺϳ���			
�޸�ʱ�䣺2019.5.11
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
�������ܣ�Ӳ����ʼ��
��    ������
�� �� ֵ����
*/
void Hardware_Init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);					//�жϷ���

	delay_init(168);
	
	Tba_Init();														//������ʼ��
	Infrared_Init();												//�����ʼ��
	Cba_Init();														//���İ��ʼ��
	Ultrasonic_Init();												//��������ʼ��
	Hard_Can_Init();												//CAN���߳�ʼ��
	BH1750_Configure();												//BH1750��ʼ������
	SYN7318_Init();													//����ʶ���ʼ��
	Electricity_Init();												//��������ʼ��

	UartA72_Init();
	Can_check_Init(83,7);  //8us											//CAN���߶�ʱ����ʼ��
	
	roadway_check_TimInit(167,1000-1);//4ms ��ʱʱ����̻����can�����ϵ����̺�ѭ����	//·�����
	
	Timer_Init(167,999);										    //��������ͨѶʱ��֡
	Readcard_daivce_Init();											//RFID��ʼ��
}


//uint8_t open_road_buf[] = {0x55,0x03,0x01,0x01,0x00,0x00,0x02,0xBB};			//��բ����
uint8_t test_buf[9] = {0xFD,0x00,0x06,0x01,0x00,0xBB,0xB6,0xD3,0xAD};			//�������������á�   
uint8_t repo_buf[] = {0x03,0x05,0x14,0x45,0xDE,0x92};							//�򿪺��ⱨ��

	
/**
�������ܣ��������
��    ������
�� �� ֵ����
*/
//uint16_t Light_Value = 4564;	
u8 ASR_Buf[] ={0xFD,0x00,0x02,0x10,0x03};
u8 R_ASR_Buf[9];
void KEY_Check()
{
	if(S1==0){	
		while(S1==0);
		startRun();
//		RFID_S50.RFID_Mode = READ1;
//		Track_Test(CARSPEED, ZERO, ROADONE, ROADMODE); 
//		TaskBoardTest(1);
	}
	if(S2==0){
		while(S2==0);
		RFID_S50.RFID_Mode = READ1;
//		PrintfDebug(isRunControl.NowPot);		//��ǰ�����
//		RFID_S50.RFID_Mode = SLEEP;
//		TaskBoardTest(2);
	}
	if(S3==0){
		while(S3==0);
		Send_ZigbeeData_To_Fifo(LTCK_R,8);
//		SYN7318_Test();
//		PrintfDebug(Get_Bh_Value());
//		Send_WifiData_To_Fifo(ShapeRead, 8);
//		TaskBoardTest(3);
	}
	if(S4==0){
		while(S4==0);
		Send_ZigbeeData_To_Fifo(DZ_R,8);
//		Send_WifiData_To_Fifo(TrafficLight, 8);
//		TaskBoardTest(4);
	}	
}

void RFID_Check()
{
	if(RFID_S50.RFID_Mode != SLEEP)	//Ѱ��
	{
		Read_Card(RFID_S50.Area);
		if(RFID_S50.RFID_Read_Ok == 1 && (RFID_S50.RFID_Mode == READ1 || RFID_S50.RFID_Mode == READ2))
		{
			RFID_S50.RFID_Read_Ok=0;
			RFID_S50.RFID_Mode = SLEEP;
		}
	}
}

static uint32_t Power_check_times;		  //�����������
static uint32_t LED_twinkle_times;		  //LED��˸����
static uint32_t WIFI_Upload_data_times;   //ͨ��Wifi�ϴ���������
static uint32_t RFID_Init_Check_times;

int main(void)
{	
//	uint16_t Light_Value = 0;				//��ǿ��ֵ	
	uint16_t CodedDisk_Value = 0;			//����
	uint16_t Nav_Value = 0;					//�Ƕ�
	u8 sum=31;
	
	Hardware_Init();						//Ӳ����ʼ��	
	Power_check_times =  gt_get() + 1000;	
//	LED_twinkle_times =  gt_get() + 50;     
//	WIFI_Upload_data_times = gt_get() + 200;
//	RFID_Init_Check_times = gt_get()+200;
	
//	Send_UpMotor(0 ,0);
//*******************************************************************
	WifiSignal_Rx_Init();
	NowTaskPot=0;
	Send_InfoData_To_Fifo((u8 *)"OK\n", sizeof("OK\n"));
	delay_ms(5);	//��ʱ����
	STOP();
	runtimeInit();  //��ʼ�������־λ
	initStartCoord(6, 5, 0); //������ʼλ��
//	initStartCoord(1, 5, 2 );
//    xydInit(&passivity.TerrainPot, 5, 3, 1); //���õ��α���λ��
//    xydInit(&passivity.TrafficPot, 5, 4, 3); //���ý�ͨ�Ʊ���λ��G4
//    xydInit(&passivity.ETCPot, 4, 3, 0); //����ETC����λ��  E2
//    xydInit(&passivity.BarrierszPot, 2, 5, 0); //���õ�բλ��
//    xydInit(&passivity.RFIDCard, 4, 5, 0); //����RFIDλ��
	InitDataBase();   //��ʼ��Ĭ�ϵ�����//������//��Դ��λ//���⼸��
	Host_Set_UpTrack(50);  //���������ϴ�ʱ��
//******************************************************************	
	while(1)
	{
		KEY_Check();//�������
        runControl();//·����ʻ����
		RFID_Check();//RFID���		
        RunTaskControl();
		
		Can_WifiRx_Check();
		Can_ZigBeeRx_Check();

		if(gt_get_sub(Power_check_times) == 0) 			
		{
			Power_check_times =  gt_get() + 200;		//��ص������
			Power_Check();		
		} 
//		
//		#if 0
//		if(gt_get_sub(LED_twinkle_times) == 0) 			
//		{
//			LED_twinkle_times =  gt_get() + 50;			//LED4״̬ȡ��
//			LED4 = !LED4;
//		} 
//		#endif
//				
//		
//		#if 1
//		if(gt_get_sub(RFID_Init_Check_times) == 0) 			
//		{
//			RFID_Init_Check_times =  gt_get() + 200;	//RFID��ʼ�����
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
//		
//		
//
//		if(gt_get_sub(WIFI_Upload_data_times) == 0)  //���Ź�������
//		{
//			WIFI_Upload_data_times =  gt_get() + 200;
//		
//			if(Host_AGV_Return_Flag == RESET)
//			{
//				Principal_Tab[2] = Stop_Flag;				//����״̬
//				Principal_Tab[3] = Get_tba_phsis_value();	//����״ֵ̬����
//				
//				Ultrasonic_Ranging();						//����������						
//				Principal_Tab[4]=dis%256;        
//				Principal_Tab[5]=dis/256;
//				
//				Light_Value = Get_Bh_Value();				//��ǿ�ȴ�����	
//				Principal_Tab[6]=Light_Value%256;	    	//��������
//				Principal_Tab[7]=Light_Value/256;
//				
//				CodedDisk_Value = CanHost_Mp;				//����
//				Principal_Tab[8]=CodedDisk_Value%256;	    	
//				Principal_Tab[9]=CodedDisk_Value/256;
//				
//				Nav_Value = CanHost_Navig;					//�Ƕ�
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

