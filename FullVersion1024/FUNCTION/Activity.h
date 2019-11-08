#ifndef __ACTIVITY_H
#define __ACTIVITY_H

#include "stm32f4xx.h"
#include "roadway_check.h"
#include "data_base.h"

#define ROW           3				//��ά������
#define COLUMN		  100    		//��ά������
#define PATHROW   	  0				//��0
#define PATHLENGTH    1				//�������鳤��

#define LEFT45		0x01				//��ת45��	
#define LEFT 		0x02				//��ת
#define LEFT180		0x03				//��ת180��	
#define RIGHT45		0x04				//��ת45��	
#define RIGHT 		0x05  				//��ת	
#define RIGHT180	0x06				//��ת180��								
#define GO			0x07				//ǰ��											
#define BACK	    0x08				//����	
#define CARSTOP     0x09				//ͣ��										
#define CNTONE   	0x0A				//ѭ��һ��ʮ��·��    			
#define CNTTWO		0x0B  				//ѭ������ʮ��·��				
#define CNTTHREE    0x0C				//ѭ������ʮ��·��
#define CNTFOUR		0x0D				//ѭ���ĸ�·��
#define CNTFIVE 	0x0E				//ѭ�����·��
#define TRACKBLACK	0x0F 				//ѭ��һ��·�ں���׼ȷͣ
#define TRACKLENTH	0x10				//ѭ������						
#define MIDHALF		0x11				//ѭ���еȳ��ȵ�һ��	
#define GOCARBOYLEN	0x12				//��һ���������
#define LEFTSMALL	0x13				//��תǰѭ����һС�ξ���												
#define RIGHTSMALL	0x14				//��תǰѭ����һС�ξ���
#define TRAMP		0x15				//�Զ���ѭ��������ֵ	
#define STOPCAR		0x16				//С��ֹͣ
#define MIDDLE_LONGISH 0x17				//�ߵ��м䣨�ϳ����Ǳߣ�
#define MIDDLE_SHORTE  0x18				//�ߵ��м䣨�϶̵ıߣ�
#define GO_TERRAIN	0x19 				//�����α�־��
#define TRACK_LINE_SHORT 0x1A			//��һ���ߣ��̱ߣ�
#define TRACK_LINE_LANG  0x1B			//(��һ����) �����ߣ�
#define DEBUG	0x1C					//debug



#define NOW_BARRIERSZ 0xf1			//��բ����
#define NOW_ETC	0xf2				//ETC����
#define NOW_TRAFFIC	0xf3			//��ͨ�Ƴ���
#define NOW_TASK  0xff    			//����һ�����������
//***************************************************************************
#define TURNSPEED 	((80-Wheel_Speed_Cut)+Wheel_Speed_Up)	//ת���ٶ�
#define CARSPEED   	((50-Walk_Speed_Cut)+Walk_Speed_Up)	//�ܵ��ٶ�
//***************************************************************************
#define MAXHALFLEN 	1150				//�������һ���MPֵ
#define MIDHALFLEN	1400				//�����еȳ���һ���MPֵ

#define BACKLEN		40					//���˵�MPֵ

#define CARBODYLEN	  600			//��һ���������
#define MP_LINE_SHORT 1150			//��һ����d������ֵ���̱ߣ�
#define MP_LINE_LANG  1400			//(��һ���ߵ�����ֵ) �����ߣ�

#define ZERO	0					//��
#define ROADONE	1					//ѭ��һ��·��
#define ROADTWO	2					//ѭ������·��
#define ROADTHREE 3					//ѭ������·��
#define ROADFOUR 4	                //ѭ���ĸ�·��
#define ROADFIVE 5	                //ѭ�����·��

#define ROADMODE 1         //ѭ��·��ģʽ
#define BLACKMODE 2         //ѭ������ģʽ
#define LENMODE  3			//ѭ������ģʽ
#define TRACKTERRAIN 4		//�ߵ���ģʽ	

#define MAXPATH 30		//�����������

#define NAV45 1//ת��Ƕ�
#define NAV90 2
#define NAV180 3

//extern u8 intocorner,RodCnt; //·���жϱ�־��·�ڼ���

extern u8 Walk_Speed_Cut;		//���߼���
extern u8 Walk_Speed_Up;		//���߼���
extern u8 Wheel_Speed_Cut;		//ת�����
extern u8 Wheel_Speed_Up;		//ת�����

typedef struct
{
    u8 NowPot;		//��ǰ��
    u8 PathCount; //����
	u16 mpvaule;		//���ߵ�����ֵ
    u8 PathGather[MAXPATH];
} RunContorl;
extern RunContorl isRunControl;

void Go_Test( u8 sp , u16 len);
void Back_Test(u8 sp , u16 len ); 
void Left_Test( u8 sp, u8 NAV) ;
void Right_Test(u8 sp,u8 NAV);  // ��80���ٶ���ת��
void Track_Test( u8 sp, u16 len, u8 cntrod, u8 state);  //ѭ������
void TrackingLamp_Test(u8 sp);
void STOP(void);   //ֹͣ

void runControl(void);
u8 Car_Run(u8 order);//���ߺ���
u8 getRunMpValue(int type, float typenum, float carbody, float linewidthnum); //mp���㺯��  1 0 1 0
void trackLength(int tramp);
u8 getNowPathVaule(void);
void InitRunPathContorl(u8 *gather, u8 len); //��ʼ��
void setMPVaule(u16 vaule);
u16 getMPVaule(void);
void Stop_Test(void); 

void runtimeInit(void);
void startAction(void);
void endAction(void);
u8 getActionState(void);
void startRun(void);
void endRun(void);
u8 getRunState(void);
void startTask(void);
void endTask(void);
u8 getTaskState(void);

#endif



