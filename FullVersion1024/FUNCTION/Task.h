#ifndef __TASK_H
#define __TASK_H

#include "pathPlanning.h"

extern u8 CP_SHOW1[6];//s������ʾ
extern u8 CP_SHOW2[6];//������ʾ��ʾ����Э��
extern u8 DZ_CPF[8];//ZigBee��ʾǰ ����
extern u8 DZ_CPB[8];//ZigBee��ʾ�� ����
void RunTaskDataControl(void);
u8 Task_Chooce(u8 taskchoose);
u16 getCsbDis(void);//������̽�����

typedef struct 
{
	u8 TaskBegPoint;  //�������ڵ� 
	u8 TaskEndPoint;  //���������
	u8 option[6];		//����ѡ��
	u8 TaskContent[6];	//����
	u16 TaskVaule[6];	//����ֵ
}TaskOption;

typedef struct
{
		u16 distance;			//����
		u8 LightLevelNow;	//���ڵĹ�Դ��λ
		u8 LightLevelTask;	//����Ҫ��λ
		u8 carport;			//����
		xyd TaskPot;	//��ĳ������õ�������
		u8 Shape[3];	//ͼ����Ϣ
}DataBase;

//extern DataBase dynamicInfo;	

struct Mailbox	//����
{
	u8 Graph_Sum_Shape[5][10];	//ͼ����������״
	u8 PlateNumber[2][8];		//����
	u8 TrafficLightA[8];		//��ͨ��
	xyd CoordinatePoint[1];		//���ص�
	u8 OneNum;					//��������
	DataBase ConfigInfo;		//����ʱ��õĶ�̬��Ϣ
};//��������

extern struct Mailbox MailboxRe;	//��������
extern TaskOption CarRunTask;


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
u8 Compute(u8 *array,u8 len,u8 object); //����
/***********************************************************************/

enum
{
	TaskHW = 1,
	TaskWifi = 2 ,
	TaskZigbee = 3,
	TaskCar = 4
};


#endif
