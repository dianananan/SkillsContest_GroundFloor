#include "stm32f4xx.h"
#include "data_base.h"
#include "PathPlanning.h"
#include "delay.h"
#include "task.h"
#include "canp_hostcom.h"
#include "TaskZigbeeChannel.h"
#include "TaskWifiChannel.h"
#include "Init.h"


//uint8_t Principal_Tab[Principal_Length];
//uint8_t Follower_Tab[Follower_Length];
u8 C_Tab[10];	//����״̬����
u8 zig_send_door_Rev_flag = 0;	  //��բ�򿪱�־λ
u8 zig_send_voice_Rev_flag = 0;   // ����оƬ״̬����
struct RFID_Card RFID_S50;
u8 M08;

u8 getNEWTask() //���ö�������
{
    NowTaskPot++;

    switch(NowTaskPot)  //2����1����0����3��
    {
//    case 1 :
//		initTask(setVaule(1, 3, 1 ),  LEFT45, 0, 0, 1 );  //����ܿ�ʼ��ʱ
//        setNowTask(TaskCar, DELAY, 500);
//		break;
//	case 2 :
//		initTask(setVaule(3, 3, 2 ), 0, 0, 0, 0 );  //��բ
//		setNowTask(TaskCar, DELAY, 500);
//		break;	
//    case 3 :
//		initTask(setVaule(3, 3, 1 ),  0, 0, 0, 0 );  //����ܿ�ʼ��ʱ
//        setNowTask(TaskCar, DELAY, 500);
//		break;
//    case 4 :
//		initTask(setVaule(3, 5, 2 ),  0, 0, 0, 0 );  //����ܿ�ʼ��ʱ
//        setNowTask(TaskCar, DELAY, 500);
//		break;	
		
		
	case 1 :
		SYN_TTS((u8 *)"����ʼ");
		initTask(setVaule(6, 5, 0 ),  0, 0, 0, 0 , 0);  //����ܼ�ʱ
		setNowTask(TaskZigbee, CMD_ZG_SEG, OPEN_SEG);
		break;
	
	case 2 :
		RFID_S50.RFID_Mode=READ1;
		RFID_S50.Area=1;
		Walk_Speed_Cut=20;			
		initTask(setVaule(5, 3, 2 ), 0 , GO, 150, 1 ,REGRESSION);  //��������
		setNowTask(TaskZigbee, CMD_VOICE, 0);
//		setNowTask(TaskCar, DELAY, 500);
		break;
	
	case 3 :
		initTask(setVaule(5, 1, 0 ), LEFT45 , 0, 0, 1 ,REGRESSION);  //��ά��
		setNowTask(TaskWifi, CMD_QR_READ, 0);
//		setNowTask(TaskCar, DELAY, 500);
		break;	

	case 4 :
		RFID_S50.RFID_Mode = SLEEP;//�ر�Ѱ��
		Walk_Speed_Cut=0;
		initTask(setVaule(3, 1, 3 ), 0 , 0, 0 , 0 ,REGRESSION);  //����ʶ��
		setNowTask(TaskWifi, CMD_PLATE_READ, 0);
		break;	

	case 5 :
		RFID_S50.RFID_Mode=READ1;
		RFID_S50.Area=1;
		Walk_Speed_Cut=20;		
		initTask(setVaule(3, 1, 0 ), LEFT45 , 0, 0, 0 ,0);  //������ʾ
		setNowTask(TaskHW, HW_PLATESHOW, RFID_S50.RFID_XYD);
		break;

	case 6 :
		initTask(setVaule(3, 3, 1 ), RIGHT45 , 0, 0, 1 ,REGRESSION);  //���̨ 
		setNowTask(TaskZigbee, CMD_ZG_SEG, SHOW2_SEG);
		setNowTask(TaskHW, HW_OPENBJQ, 0);
		break;

	case 7 :
		initTask(setVaule(3, 5, 1 ), 0 , GO, 200, 1 ,0);  //·��
		setNowTask(TaskHW, HW_LIGHT, 3);
		break;	
	
	case 8 :
		RFID_S50.RFID_Mode = SLEEP;//�ر�Ѱ��
		Walk_Speed_Cut=0;
		initTask(setVaule(3, 5, 0 ), 0 ,0 , 0, 0 ,0); // ��բ
		setNowTask(TaskZigbee, CMD_ZG_DOOR, 0);				
		break;
	
	case 9 :
		initTask(setVaule(1, 5, 2 ), 0 ,0 , 500, 0 ,0); // ����
		setNowTask(TaskCar, UPRIGHT, 0);
		break;	
	
	case 10 :
		PrintfDebug(2);
		initTask(setVaule(1, 5, 2 ), 0 ,0 ,0, 0 ,0); // ���
		setNowTask(TaskZigbee, CMD_GETPOINT1, 500);		
		setNowTask(TaskZigbee, CMD_ZG_SEG, CLOSE_SEG);
		break;

    default :
        NowTaskPot = 255;
        break;
    }
	
	return 1;
}




//    case 1 :
//		initTask(setVaule(1, 0, 1 ),  0, 0, 0, 0 );  //����ܼ�ʱ
//		setNowTask(TaskZigbee, CMD_ZG_SEG, OPEN_SEG);
//		setNowTask(TaskCar, BEEP_OPEN, 1800);
//		setNowTask(TaskZigbee, CMD_ZG_DOOR, 0);
//		break;	
//	
//    case 2 :
//		initTask(setVaule(1, 3, 1 ), LEFT45 , 0, 0, 1 );  //��ά������
//        setNowTask(TaskWifi, CMD_QR_READ, 0);
//		break;

//    case 3 :
//		initTask(setVaule(1, 5, 0 ), 0 , 0, 0, 0 );  //ͼ��ʶ��
//        setNowTask(TaskWifi, CMD_SHAPE_READ, 0);
//		break;
//	
//    case 4 :
//		initTask(setVaule(1, 5, 1 ), 0 , GO, 50, 1 );  //������ʾ
//        setNowTask(TaskHW, HW_TYPESHOW, 3);
//		break;	
//	
//    case 5 :
////		RFID_S50.RFID_Mode = SLEEP;//�ر�Ѱ��
//		initTask(setVaule(5, 5, 1 ), 0 , GO, 120, 1 );  //��Դ����
//        setNowTask(TaskHW, HW_LIGHT, 3);
//		break;	
//	
//    case 6 :
////		RFID_S50.RFID_Mode = READ; //����Ѱ��
//		initTask(setVaule(5, 5, 2 ), 0 , BACK, 150, 1 );  //����ʶ��
//        setNowTask(TaskWifi, CMD_PLATE_READ, 0);
//		break;
//	
//    case 7 :
//		initTask(setVaule(5, 3, 0 ), 0 , 0, 0, 0 );  //��ETC
//        setNowTask(TaskZigbee, CMD_ETC, 0);
//		break;
//	
//    case 8 :
//		initTask(setVaule(3, 3, 0 ), RIGHT45 , 0, 0, 1 );  //����̨
//        setNowTask(TaskHW, HW_OPENBJQ, 0);
//		break;	
//	
//	case 9 :
//		initTask(setVaule(3, 3, 0 ), 0 , 0, 0, 0 );
//		M05=MailboxRe.ConfigInfo.LightLevelNow;
//		M05=(M05*9%8+1)%3+1;
//		if(M05 == 1)
//			NowTaskPot=34;
//		else if(M05 == 2)
//			NowTaskPot=34;
//		else
//			NowTaskPot=34;		
//		break;
//	
//    case 15 :
//		initTask(setVaule(5, 1, 1 ), 0 , 0, 1000, 0 );  //���1
//		setNowTask(TaskCar, UPRIGHT, 0);		//����
//		break;
//	case 16:
//		initTask(setVaule(5, 1, 1 ), 0 , 0, 0, 0 );
//		setNowTask(TaskZigbee, CMD_GETPOINT1, 0);	//���
//		setNowTask(TaskCar, BEEP_OPEN, 3);			//������
//		setNowTask(TaskZigbee, CMD_MAGLEV, 0);		//���߳��
//		break;
//	case 17:
//		send_SEG_data(CLOSE_SEG);	//�ر������
//		break;
//	
//    case 25 :
//		initTask(setVaule(3, 1, 1 ), 0 , 0, 1000, 0 );  //���2
//		setNowTask(TaskCar, UPRIGHT, 0);		//����
//		break;	
//	case 26:
//		initTask(setVaule(3, 1, 1 ), 0 , 0, 0, 0 );
//		setNowTask(TaskZigbee, CMD_GETPOINT1, 0);	//���
//		setNowTask(TaskCar, BEEP_OPEN, 3);			//������
//		break;
//	case 27:
//		RepeatedlySend_ZG(MAGLEV, 8, 20);
//		send_SEG_data(CLOSE_SEG);	//�ر������		
//		break;
//	
//    case 35 :
//		initTask(setVaule(1, 1, 1 ), 0 , 0, 1000, 0 );  //���3
//		setNowTask(TaskCar, UPRIGHT, 0);			 //����
//		break;
//	case 36:
//		initTask(setVaule(1, 1, 1 ), 0 , 0, 0, 0 );
//		setNowTask(TaskZigbee, CMD_ZG_DOOR, 0);		 //����բ
//		setNowTask(TaskZigbee, CMD_GETPOINT1, 0);	 //���
//		setNowTask(TaskCar, BEEP_OPEN, 3);			//������
//		break;
//	case 37:
//		RepeatedlySend_ZG(MAGLEV, 4, 10);
//		send_SEG_data(CLOSE_SEG);	//�ر������
//		break;


