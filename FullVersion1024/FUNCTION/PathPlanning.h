#ifndef __PATHPLANNING_H
#define __PATHPLANNING_H	 

#include <stdio.h>
#include "stm32f4xx.h"
#include "Activity.h"
#include "PathPlanning.h"

#define MAXPATH 30
#define MAXLIMIT 10

#define  getxy(vaule) ((vaule)&0x3f) //ȡ��x��y
#define	 getDirection(vaule) ((vaule)>>6&3) //ȡ������
#define	 getVauleX(vaule) ((vaule)&0x07)
#define  getVauleY(vaule) ((vaule)>>3&0x07)
#define  setVaule(x,y,dir) ((x)|(y<<3)|(dir<<6))
#define  setVauleDis(xy,dir)((dir<<6)|(xy))


#define LEFTDIR 0   //������
#define FRONTDIR 1     //�����ϣ�
#define RIGHTDIR 2      //�����ң�
#define BACKDIR 3     //�ϣ��£�


typedef union
{
    u8 x,y,dir;
    u8 data;
}xyd;


struct LineNode
{
    xyd target;             //Ŀ��������������
    xyd sexy[MAXLIMIT];     //����ؾ���
    u8 TaskDir;     //������ ��Ҫ���� 45�� Ϊ�˼������� ����ֻ������ת45�� ������ǰ��� xyd ����
    u8 LastLineOption;  //���һ���Ĳ���  �Ƿ���mp ǰ�����Ǻ���
    u8 isback;
	u8 SpecialDispose;		//���⴦���
};
extern struct LineNode TaskControl;

struct passivityGather
{
	 xyd TerrainPot;				//����λ��
	 xyd ETCPot;						//etc����λ��
	 xyd TrafficPot;				//��ͨ�Ƴ���λ��
	 xyd BarrierszPot;				//��բ����λ��
	 xyd RFIDCard;						//RFID������λ��
	//û���ֽڶ���
};
extern struct passivityGather passivity;

extern xyd LimitPot[MAXLIMIT];     //����ؾ���
extern u8 map[7][7];
extern u8 Carx,Cary,Cardirection;
extern u8 car_x,car_y,cardirection;	
extern short int rotate;//С����ת45
extern u8 NowTaskPot;		//��ǰ�������
extern u8 ForwardMPFlag;	//�Ƿ�ת��ǰһС�ξ���

void setMap(u8 x,u8 y,u8 s);
void initStartCoord(u8 x,u8 y,u8 dir);
void addCarLine(void);
u8 taskDirectionDeal(void);
void lastLine(void);
void setLimitPot(u8 sxy ,u8 loc);
u8 JudgeIntersection(int i ); 
void walkIntersection(void);
void dfs(u8 runxy,int n);
u8 getStep(u8 *x,u8 *y,u8 type);
u8 initTask(u8 target, u8 dir, u8 LastOption, u16 mpvaule, u8 runback, u8 Special);
u8 getMPvaule(void);
u8 getNowTask(void);
void setTaskLimitPot(void);
char getNewDirection(u8 type, u8 *current);
//void setLimitPot(u8 sxy,u8 exy);
void getCarPosition(u8 *currentdirection,u8 *x,u8 *y,u8 type);
void setTaskLimitGather(u8 *,u8); //���ö�����Ƶ�
void xydInit(xyd *data,u8 x ,u8 y,u8 dir);




#endif	
	
