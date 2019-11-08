#ifndef __TASK_H
#define __TASK_H

#include "pathPlanning.h"


//void RunTaskDataControl(void);	
u8 Task_Chooce(u8 taskchoose);	//����ѡ��
u16 getCsbDis(void);			//������̽�����

typedef struct 
{
	u8 TaskBegPoint;	//�������ڵ� 
	u8 TaskEndPoint;	//���������
	u8 option[6];		//����ѡ��
	u8 TaskContent[6];	//����
	u16 TaskVaule[6];	//����ֵ
}TaskOption;

typedef struct
{
		u16 distance;		//����
		u8 LightLevelNow;	//���ڵĹ�Դ��λ
		u8 LightLevelTask;	//����Ҫ��λ
		u8 carport;			//����
		xyd TaskPot;		//��ĳ������õ�������
		u8 Shape[3];		//ͼ����Ϣ
}DataBase;

struct Mailbox	//����
{
	u8 Graph_Sum_Shape[3];		//ͼ��[0]��ɫ[1]��״[2]����
	u8 PlateNumber[6];			//����	/0��Ĭ�ϳ���  1�Ƕ�ȡ���ĳ���
	u8 TrafficLightA;			//��ͨ��
	xyd CoordinatePoint;		//���ص�
	u8 OneNum;					//��������
	DataBase ConfigInfo;		//����ʱ��õĶ�̬��Ϣ
};//��������

extern struct Mailbox MailboxRe;	//��������
extern TaskOption CarRunTask;		//��������

void setNowTask(u8 NowTask,u8 cmd,u16 Vaule);
u8 getTaskCmd(void);
u8 getNowTask(void);
void TaskMenu(void);
void RunTaskControl(void);
u8 getNowLight(void);
void setNowlight(u8 );
u8 getNowGarage(void);

/************************************************************************/
void InitDataBase(void);//��ʼ�����ݣ����ó�ֵ��ֹ��ΪĬ��ֵ��
u8 Compute(u8 *array,u8 len,u8 object); //object��������ֵĸ���
/***********************************************************************/

enum
{
	TaskHW = 1,
	TaskWifi = 2 ,
	TaskZigbee = 3,
	TaskCar = 4
};


#endif
