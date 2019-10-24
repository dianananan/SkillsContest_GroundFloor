#ifndef __TASKWIFICHANNEL_H
#define __TASKWIFICHANNEL_H

#include "sys.h"
#include "PathPlanning.h"

#define  PLATENUM	8//����

#define  ZCKZ_ADDR	0xAA				//��������
#define  CCZ_ADDR	0x02        //�ӳ�����

#define  RUNSTART	0x01    				//С������
#define  QRCODE		0x02						//��ά��ʶ��
#define  PLATEREV1	0x10						//����ʶ��
#define  PLATEREV2	0x11						//����ʶ��
#define	 SHAPE		0x04						//��״ʶ��
//#define	 TRAFFIC	0x05					  //��ͨ��ʶ��
//#define  POINT	0x06						//�������

struct Mailbox	//����
{
	u8 Graph_Sum_Shape[5][10];	//ͼ����������״
	u8 PlateNumber[2][8];		//����
	u8 TrafficLightA[8];			//��ͨ��
	xyd CoordinatePoint[1];		//���ص�
	u8 OneNum;					//��������
};//��������

extern struct Mailbox MailboxReceiving;

extern u8 wifi_send_PLATE_flag;//����ʶ����ѵ���
extern u8 wifi_send_QR_flag;//wifi���Ͷ�ά��������ƽ��  0:init 1:car->pad 2:pad->car
extern u8 wifi_send_SHAPE_flag;
extern u8 wifi_rev_card_flag_1;

extern u8 light_xxx;
extern u8 gg_flag;

extern u8 Timeout;
extern u8 wifitask;

extern u8 GETPONT[3];//wifi��������
extern u8 qrread[1];
//extern u8 shapedata[3];//ͼ��ʶ������
extern u8 trafficread[1];
extern u8 taskover;//����������

//void Wifi_Rev_Control(void);
void Wifi_Send_Dispose(u8 Wifi_signal);
void Wifi_Send_Dispose_Wifi(u8 *Wifi_signal);
void switchBackinfo(u8 vaule);//�ж��ϲ�Ļش���Ϣ
void hw_Data_Dispose(u8 *Data); //����������ʾ�ĺ�����Ϣ
void Wifi_Send_Dispose_zigbee(u8 *Wifi_signal);  //����zigbee��Ϣ //��բ��//���̵��ر���
void Wifi_Send_Dispose_hw(u8 *Wifi_signal);  //���������Ϣ 
void Wifi_Send_Dispose_run(u8 *Wifi_signal);//������������
void Wifi_Send_Dispose_action(u8 *Wifi_signal);//������
void Wifi_Send_Dispose_Car(u8 *Wifi_signal);  //С����ԭ������
void Wifi_Remote_Control (void); //wifi�źŽ���	
#endif
