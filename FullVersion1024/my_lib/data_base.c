#include "stm32f4xx.h"
#include "data_base.h"
#include "PathPlanning.h"
#include "delay.h"
#include "task.h"
#include "canp_hostcom.h"
#include "TaskZigbeeChannel.h"
#include "TaskWifiChannel.h"
#include "Init.h"


uint8_t Principal_Tab[Principal_Length];
uint8_t Follower_Tab[Follower_Length];
u8 C_Tab[10];	//����״̬����
u8 zig_send_door_Rev_flag = 0;	  //��բ�򿪱�־λ
u8 zig_send_voice_Rev_flag = 0;   // ����оƬ״̬����

u8 getNEWTask() //���ö�������
{
    NowTaskPot++;

    switch(NowTaskPot)  //2����1����0����3��
    {
//    case 1 :
//		initTask(setVaule(5, 1, 1 ),  0, 0, 0, 0 );  //����ܿ�ʼ��ʱ
//        setNowTask(TaskZigbee, CMD_ZG_SEG, OPEN_SEG);
//		Send_ZigbeeData_To_Fifo(SMG_SHOWTWO,8); //��ʾ�ڶ���
//		break;
//	case 2 :
//		initTask(setVaule(5, 1, 1 ),  0, 0, 0, 0 );  //��բ
//		setNowTask(TaskZigbee, CMD_ZG_DOOR, 0);
//		break;
	case 1 :
		initTask(setVaule(5, 1, 1 ),  0, 0, 0, 0 );  //��ͨ��
		setNowTask(TaskWifi, CMD_LIGHT_READ, 0);	
		break;
//	case 4:
//		initTask(setVaule(3, 5, 1 ),  0, GO, 100, 1 );  //��Դ����2
//		setNowTask(TaskHW, HW_LIGHT, 0);	
//		break;
//	case 5:
////		initTask(setVaule(1, 5, 1 ),  0, 0, 0, 0 );  //����ʶ��
////		setNowTask(TaskWifi, CMD_PLATE_READ, 0);
//		break;
//	case 6:
////		initTask(setVaule(1, 5, 3 ),  RIGHT45, 0, 0, 1 );  //��ά��ʶ��
////		setNowTask(TaskWifi, CMD_QR_READ, 0);	
//		break;
//	case 7 :
//		initTask(setVaule(1, 3, 2 ),  0, 0, 1000, 0 );  //����
//		setNowTask(TaskCar, UPRIGHT, 0);
//		break;
//	case 8 :
//		initTask(setVaule(1, 3, 2 ),  0, 0, 0, 0 );  //���
//		setNowTask(TaskZigbee, CMD_GETPOINT1, 0);
//		break;		
//	case 9 :
//		initTask(setVaule(1, 3, 2 ),  0, 0, 0, 0 ); 
//		setNowTask(TaskZigbee, CMD_ZG_SEG, CLOSE_SEG);
//		break;	
    default :
        NowTaskPot = 255;
        break;
    }
	
	return 1;
}




