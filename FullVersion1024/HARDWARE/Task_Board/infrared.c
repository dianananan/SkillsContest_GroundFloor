#include <stdio.h>
#include "stm32f4xx.h"
#include "infrared.h"
#include "data_base.h"
#include "activity.h"
#include "tba.h"
#include "delay.h"
#include "TaskWifiChannel.h"
#include "TaskZigbeeChannel.h"
#include "Task.h"
#include "canp_hostcom.h"
#include "bh1750.h"

void Infrared_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);

	//GPIOF11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  //ͨ�����
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //������� 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	RI_TXD = 1;
}

/***************************************************************
** ���ܣ�     ���ⷢ���ӳ���
** ������	  *s��ָ��Ҫ���͵�����
**             n�����ݳ���
** ����ֵ��    ��
****************************************************************/
void Infrared_Send(u8 *s,int n)
{
   u8 i,j,temp;

    RI_TXD=0;
    delay_ms(9);
    RI_TXD=1;
    delay_ms(4);
	delay_us(560);

	for(i=0;i<n;i++)
	{
		for(j=0;j<8;j++)
		  {
		     temp=(s[i]>>j)&0x01;
		     if(temp==0)//����0
		        {
		           RI_TXD=0;
		           delay_us(500);//��ʱ0.5ms
		           RI_TXD=1;
		           delay_us(500);//��ʱ0.5ms
		         }
		     if(temp==1)//����1
		         {
		           RI_TXD=0;
		           delay_us(500);//��ʱ0.5ms
		           RI_TXD=1;
				   delay_ms(1);
		           delay_us(800);//��ʱ1.69ms  690
		
		         }
		  }
	}
    RI_TXD=0;//����
    delay_us(560);//��ʱ0.56ms
    RI_TXD=1;//�رպ��ⷢ��
}



u8 getligthsum() //���ص�ǰ����
{
   u8 lightsum = 0;
   u16 light1=0, light2=0;
   while(lightsum < 20)
   {
		 light2 = Get_Bh_Value();		//���Թ���ǿ��
		 if(light1 > light2)break;
		 Infrared_Send(H_N[0], 4);
		 delay_ms(1200);
		 light1 = light2;
		 //Send_Debug_Info("ok\n", sizeof("ok")); // �ϴ�������Ϣ
		 ++lightsum;
	 }	
	lightsum=5-lightsum; //�����Դ����
	Acquire_rank(lightsum);//��λ��λ	
	delay_ms(1000);
//	dynamicInfo.LightTransmission=lightsum;//¼���Դ��λ��Ϣ
	return lightsum;
}

void Acquire_rank(u8 num) //����
{
    if(num == 1)
	{
		Send_InfoData_To_Fifo((u8 *)"\n1",sizeof("\n1"));
		return ;
	}
    Infrared_Send(H_N[num - 2], 4);////��һ����n�����м�ֻҪ��n-1�����˲���������num-2��
}


void light_Self_Tes() //·�Ƶĵ�λ���  
{
    u8 i = 0;
    u16 light1, light2;
    while(i < 20)
    {
        light2 = Get_Bh_Value();		//���Թ���ǿ�� 	Get_Bh_Value
        if(light1 > light2)
		{
			break;
		}
        Infrared_Send(H_N[0], 4);
        delay_ms(1200);
        light1 = light2;
        ++i;
		Send_InfoData_To_Fifo((u8 *)"ts",sizeof("ts"));
    }
}

void light_control(u8 Rank)  //���ڵ�����
{
    light_Self_Tes();	 
    Acquire_rank(Rank);
	delay_ms(800);
//	dynamicInfo.LightTransmission=Rank;//�������֮��//¼���Դ��λ��Ϣ
}




u8 HW_Send_Dispose(u8 taskChoose)
{
	HW_Send_Choose(taskChoose);
	return 0;
}

u8 HW_Send_Choose(u8 choose_task)
{
	u8 i;
	u8 x=0;
	u8 ll[6]={0,0,0,0,0,0};
	startTask();
	switch(choose_task)
	{
		case HW_LIGHT://��Դ��λ����
			light_control(getNowLight());
//			light_control(2);
//			delay_ms(2000);
			break;
		
		case HW_LIGHTRE://��Դ��λ����
			getligthsum();delay_ms(100);
			break;
			
//-----------��в��Ժ�����������ʱ��ע��-----------------//
//-----------��PrintValue�������һ�и�Ϊwifi����---------//
//			while(1)
//			{
//						delay_ms(500);
//						light = Dispose();
//						x=light%256;	
//						y=light/256;
//						PrintValue(x);
//						PrintValue(y);
//			}
////--------------------------------------------------------//
		
		//������Ŀ���ڵ�λ���ֶ���
//			if(light >= ONE_SMALL && light <= ONE_BIG)
//			{
//				//�ж�Ϊһ��
//				
//				Infrared_Send(H_2,4);  //��2��
//			}else
//			if(light >= TWO_SMALL && light <= TWO_BIG)
//			{
//				//�ж�Ϊ����
//				
//				Infrared_Send(H_1,4);//��1��
//			}else
//			if(light >= THREE_SMALL && (light <= THREE_BIG))
//			{
//				//Infrared_Send(H_1,4);
//			}else
//			if(light >= FOUR_SMALL && light <= FOUR_BIG)
//			{

//				Infrared_Send(H_3,4);//��3��
//			}

//			light_get();
//		  break;
		case HW_PLATESHOW://������ʾ����
			delay_ms(500);
			for(x=0;x<2;x++)
			{		
				Infrared_Send(CP_SHOW1,6);
				delay_ms(400);
				Infrared_Send(CP_SHOW2,6);
				delay_ms(400);
			}
			break;
		
		case HW_TYPESHOW://ͼ����ʾHW_TYPESHOW
			for(i = 0;i<5;i++)
			{
				delay_ms(1200);
//				CP_G1[6]={0xFF,0x12,0x00,0x00,0x00,0x00};
				ll[0]=0xff;
				ll[1]=0x12;
				ll[2]=gg_flag;
				ll[3]=ll[4]=ll[5]=0x00;
				Infrared_Send(ll,6);
			}
			delay_ms(2000);
		   break;
		
		case HW_HUESHOW ://��ɫ��ʾ
			Infrared_Send(CP_G1,6); delay_ms(5); 
		    break;
		
		case HW_LKSHOW ://·����ʾ
			Infrared_Send(SD_LK,6);delay_ms(5);
			break;
			
		case HW_LASTSHOW : //Ĭ����ʾ
			Infrared_Send(SD_MRXX,6);delay_ms(5);
			break;	
			
		case HW_TENNEL://����ŷ�
			Infrared_Send(H_SD,4);delay_ms(5);
			break;
			
		case HW_PICUP://ͼƬ�Ϸ�
			Infrared_Send(H_S,4);delay_ms(5);
			break;
			
		case HW_PICDOWN://ͼƬ�·�
			Infrared_Send(H_X,4);delay_ms(5);
			break;
			
		case HW_OPENBJQ://�򿪱�����
			Infrared_Send( HW_K,6);delay_ms(100);
			break;
			
		case HW_CLOSEBJQ://�رձ�����
			Infrared_Send(HW_G,6);delay_ms(5);
			break;
			
		case HW_DISSHOW :  //������ʾ����  cm������
			SD_JL[2] = ASCLLSwitch(dynamicInfo.distance/100);
			SD_JL[3] = ASCLLSwitch(dynamicInfo.distance/10);
			for(i = 0;i<10;i++)
			{
				Infrared_Send(SD_JL,6);
				delay_ms(150);
			}
			break;
		
		default:
			return 0;
		    break;
	}
	    endTask();	
	return 1; 
}


u8 HD_BCDSwitch(u16 object)  //ʮ�����Ƶ�BCDת��
{
	u8 ResultSum=0;

	ResultSum=object%100;
	ResultSum=((ResultSum)/10)*16+(ResultSum%10);
	return ResultSum;
}

u8 ASCLLSwitch(u16 object)   //ASCLLת�� ʮ����ת��ASCLL��
{
	u8 ResultSum=0;
	ResultSum=(object%10)+'0';  //48
	return ResultSum;
}




