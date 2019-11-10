#include "stm32f4xx.h"
#include "CanP_Hostcom.h"
#include "delay.h"
#include "roadway_check.h"
#include "PathPlanning.h"
#include "cba.h"
#include "Timer.h"
#include "activity.h"
#include "Init.h"

uint8_t L_Flag =0,R_Flag = 0;
uint8_t G_Flag = 0,B_Flag = 0;
u8 Upright_Flag=0;
uint8_t Track_Flag = 0;
uint8_t wheel_Nav_Flag = 0; 
u8 TrackingLamp_Flag =0; //ѭ���ı�־λ
u8 Regression_Flag =0;	//ת�������־λ

uint8_t Line_Flag = 0;
uint16_t count = 0;
uint8_t Stop_Flag = 0;
int LSpeed = 0,RSpeed = 0;
int Car_Spend = 0;
uint16_t temp_MP = 0,MP=0,TraLen=0;
uint16_t temp_Nav = 0;

uint32_t Wheel_flag = 0;  

void Back_Track(uint8_t gd);

//_________________________________________________________
int16_t Roadway_cmp;
extern int16_t CanHost_Mp;

/*
	����ͬ��
**/
void Roadway_mp_syn(void)
{
	Roadway_cmp = CanHost_Mp;	
}

/*
	���̻�ȡ
**/
uint16_t Roadway_mp_Get(void)
{
	uint32_t ct;
	if(CanHost_Mp > Roadway_cmp)
		ct = CanHost_Mp - Roadway_cmp;
	else
		ct = Roadway_cmp - CanHost_Mp;
	if(ct > 0x8000) //32768  //ffff 65535
		ct = 0xffff - ct;
	return ct;
}

//_______________________________________________________________



//_________________________________________________________
uint16_t Roadway_Navig;
extern uint16_t CanHost_Navig;

/*
	�Ƕ�ͬ��
**/
void Roadway_nav_syn(void)
{
	Roadway_Navig = CanHost_Navig;
}

/*
	��ȡ�ǶȲ�ֵ
**/
uint16_t Roadway_nav_Get(void)
{
	uint16_t ct;
	if(CanHost_Navig > Roadway_Navig)
		ct = CanHost_Navig - Roadway_Navig;
	else
		ct = Roadway_Navig - CanHost_Navig;
	while(ct >= 36000)
		ct -= 36000;
	return ct;
}

//_______________________________________________________________

void Roadway_Flag_clean(void)	//��־λ���
{
	L_Flag =0;R_Flag = 0;
	G_Flag = 0;B_Flag = 0;
	wheel_Nav_Flag=0;
	Track_Flag = 0;
	TrackingLamp_Flag=0;
	Stop_Flag = 0;
	temp_MP = 0;
	
	RodCnt=0;  //·�ڼ�������
	TraLen=0; 
	MP=0;
}

/**
	ǰ�����
*/
void Go_and_Back_Check(void)
{	
	if(G_Flag == 1 && Track_Flag==0)
	{
		if(MP >= temp_MP)
		{
			STOP();
			Stop_Flag = 3;
			getCarPosition(&cardirection,&car_x,&car_y,2);
			endAction();
		}
	} 
	else if(B_Flag == 1 && Track_Flag==0)
	{
		if(MP >= temp_MP)
		{
			STOP();
			Stop_Flag=3;
			getCarPosition(&cardirection,&car_x,&car_y,3);
			endAction();
		}
	}
}


uint8_t Roadway_GoBack_Check(void)
{
	return ((G_Flag == 0)&&(B_Flag == 0)&&(Track_Flag == 0)&&(L_Flag == 0)&&(R_Flag == 0))? 1:0;
}

/**
	����ѭ����ת��
*/
//uint32_t Mp_Value = 0;
void wheel_Track_check(void)
{
	if(L_Flag==1)
	{
		if(wheel_Nav_Flag==NAV45)
			wheel_Track_ANGLE(wheel_Nav_Flag,Left_45_MP);
		else if(wheel_Nav_Flag==NAV90)
			wheel_Track_ANGLE(wheel_Nav_Flag,LeftMP1);
		else if(wheel_Nav_Flag==NAV180)
			wheel_Track_ANGLE(wheel_Nav_Flag,LSWERVEMP180);						
	}
	if(R_Flag==1)
	{
		if(wheel_Nav_Flag==NAV45)
			wheel_Track_ANGLE(wheel_Nav_Flag,Right_45_MP);
		else if(wheel_Nav_Flag==NAV90)
			wheel_Track_ANGLE(wheel_Nav_Flag,RightMP1);
		else if(wheel_Nav_Flag==NAV180)
			wheel_Track_ANGLE(wheel_Nav_Flag,RSWERVEMP180);			
	}
}

void wheel_Track_ANGLE(u8 angle,u16 min)	//ת��Ƕ�
{
	gd=Get_Host_UpTrack(TRACK_H8);
	gdg=Get_Host_UpTrack(TRACK_Q7);	
	if(angle==NAV45 && MP>=min)
	{
		Wheel_flag=0;
		Stop_Flag=2;
		if(L_Flag==1)
		{
			cardirection = getNewDirection(3,&cardirection);
		}
		else if(R_Flag)
		{
			cardirection = getNewDirection(4,&cardirection);
		}
		STOP();
		endAction();
	}
	else if(angle==NAV90)
	{
		if(((!(gd &0X18))&& MP>=min-50 && (!(gdg &0X18)))|| MP>min+50)
		{
			if(Wheel_flag) 
			{	
				Wheel_flag=0;
				Stop_Flag=2;
//				Host_Close_UpTrack();  // �ر�Ѱ�������ϴ�
				if(L_Flag==1)
				{
					cardirection = getNewDirection(0,&cardirection);
				}
				else if(R_Flag)
				{
					cardirection = getNewDirection(1,&cardirection);
				}
				STOP();
				endAction();
			}			
		}
	}
	else if((angle==NAV180 && MP>=min-50 && (!(gd &0X18))) || MP>=min+50)
	{
		if(Wheel_flag)
		{	
			Wheel_flag=0;
			Stop_Flag=2;
//			Host_Close_UpTrack();  // �ر�Ѱ�������ϴ�		
			cardirection = getNewDirection(2,&cardirection);
			STOP();
			endAction();
		}
	}
	if( (gd & 0x3C) !=0x3C) Wheel_flag=1;
}

/**
	ѭ�����
*/
u8 gd, gdg;
u8 intocorner=0;
u8 light_flagF=0,light_flagB=0;  //light led sum
u8 reflag=0;   //·�ڼ���
u8 RodCnt=0; 

void Set_UpTrack_Value(u8 mode)	//����ȡ��ȡ��
{
	if(mode == 0)
	{
		gd=Get_Host_UpTrack(TRACK_H8);
		gdg=Get_Host_UpTrack(TRACK_Q7);	
	}
	else if(mode == 1)
	{
		gd=Get_Host_UpTrack(NEGATION_H8);
		gdg=Get_Host_UpTrack(NEGATION_Q7);
	}
	light_flagF=Countbits((~gdg) & 0x7f);
	light_flagB=Countbits(~gd);		
}

void Track_Check(u16 tracklen, u8 roadSum, u8 state)	//ѭ�����
{
	if(state==ROADMODE)//���ʮ��·��
	{
		if(((light_flagF >= 5) || (((gdg & 0x18) != 0x18) && (light_flagF > 3) ) ) && (intocorner == 0))
		{
			intocorner=1;
		}
		else if(((light_flagB >= 5) || (((gd & 0x18) != 0x18) && (light_flagB >3) )) && (intocorner == 1)) //||((gdg&0x18)!=0x18)  )
		{   
			intocorner = 2;
		}
		else if((light_flagF < 3) && (intocorner == 2)) ////ǰ�ų�ȥ��
		{
			intocorner = 3; //
		}
		else if((light_flagB < 3) && (intocorner == 3)) //����
		{
			intocorner = 4; 
		}
		else if(intocorner == 4)
		{
			getCarPosition(&cardirection, &car_x, &car_y, 0);  //��������
			intocorner = 0;
			reflag++;
			if(reflag >= roadSum)
			{
				reflag = 0;
				endAction();   //��������
				STOP();
			}			
		}
	}
    else if(state == BLACKMODE)//ͣ��ѭ������
    {
        if(light_flagF >= 3)//
        {
			getCarPosition(&cardirection, &car_x, &car_y, 0);
			endAction();
			STOP();
        }
    }
	else if((state == LENMODE )  && (MP >= tracklen))	//ѭ������
	{	
		if(tracklen > 2100)
			getCarPosition(&cardirection, &car_x, &car_y, 0);
		else 
			getCarPosition(&cardirection, &car_x, &car_y, 1);
		STOP();
		endAction();
	}	

    if( Track_Flag == 0 && TrackingLamp_Flag ==0)
    {
		STOP();
//		Host_Close_UpTrack( );  // �ر�Ѱ�������ϴ�
    }
	else 
	{
		Track_Correct(Car_Spend,Car_Spend,1);
	}
}


void Roadway_Check(void)  //��·���
{
	if(Track_Flag==ROADMODE||Track_Flag == BLACKMODE||Track_Flag==LENMODE)//
	{
		Set_UpTrack_Value(0);
		Track_Check(TraLen,RodCnt,Track_Flag);	//ѭ������
	}
	TrackingLamp_check();  //����
	Go_and_Back_Check();	//ǰ��
	wheel_Track_check();  //ת��
	Regression_check();	  //����
}

void TrackingLamp_check(void)	//���μ��
{
	
}

u8 GoOrBack=0;
void Regression_check(void)		//�������   ˵����
{
	if(Regression_Flag >0)	//ת�����
	{
		Set_UpTrack_Value(0);
		if((Roadway_mp_Get()) <= 500 && GoOrBack == 0)
			Track_Correct(Car_Spend,Car_Spend,1);
		else 
			GoOrBack=1;
		if(((Roadway_mp_Get()) >=500 || GoOrBack ==1) && Regression_Flag !=0)
		{
			if(light_flagF >2)	//�����˵�ʮ��·��ֹͣ
			{
				Roadway_mp_syn();
				GoOrBack=0;
				Regression_Flag--;
				if(Regression_Flag == 0)//�������һ��
					Go_Test(30, 500);	//go��������Ķ��������Խ�����������Ҫ����
				
			}
			else
				Control(-Car_Spend,-Car_Spend);	
		}
	}	
}
	

/***************************************************************
** ���ܣ�     ������ƺ���
** ������	  L_Spend����������ٶ�
**            R_Spend����������ٶ�
** ����ֵ��   ��	  
****************************************************************/
void Control(int L_Spend,int R_Spend)
{
	TIM_Cmd(TIM9, ENABLE); //������ʱ��2
//	delay_ms(1);
	if(L_Spend>=0)
	{	
		if(L_Spend>100)L_Spend=100;if(L_Spend<5)L_Spend=5;		//�����ٶȲ���
	}
	else 
	{
		if(L_Spend<-100)L_Spend= -100;if(L_Spend>-5)L_Spend= -5;     //�����ٶȲ���
	}	
	if(R_Spend>=0)
	{	
		if(R_Spend>100)R_Spend=100;if(R_Spend<5)R_Spend=5;		//�����ٶȲ���
	}
	else
	{	
		if(R_Spend<-100)R_Spend= -100;if(R_Spend>-5)R_Spend= -5;		//�����ٶȲ���		
	}
	Send_UpMotor(L_Spend ,R_Spend);
    TIM1->BDTR |= 1 << 15; //����OC��OCN���  	
}

/***************************************************************
** ���ܣ�     ѭ������
** ������	  �޲���
** ����ֵ��   ��
****************************************************************/
u8 Countbits(u8 tstByte)
{
    u8 nCount = 0;
    while(tstByte)
    {
        if( tstByte & 1)
        {
            nCount ++;
        }
        tstByte  >>= 1;
    }
    return nCount;
}

void Track_Correct(int Left,int Right,u8 Mode)	//ѭ�����ٶȿ���
{
	   	Stop_Flag=0;
	if(gd==0xFF && gdg ==0x7F)   //ѭ����ȫ�� 
	{
		if(count > 10)//�ߵ������Ϻ�  �����һ�ѭ����
		{
			if(count > (10+500)) //
			{
				count=0;
				STOP();
				if(Line_Flag ==0) 
					Stop_Flag=4;
			}
			Control(-50,-50);
			return ;
		}	
		else 
			count++;				
	}
	else 
		count=0;
	
	
    if(gdg == 0X77 || (light_flagB >= 3) || (gd == 0xE7 ) || (gdg == 0x7F  && gd == 0xFF)) //)//1���м�3/4��������⵽���ߣ�ȫ������  ȫ����ǰ��
    {
        LSpeed = Left;
        RSpeed = Right;
    }

    if(Line_Flag != 2)
    {
        if(gd == 0XF7 || gdg == 0X73) //�� 1111 0111  �� ��x111 0011
        {
            LSpeed = Left + 50;
            RSpeed = 0;
            Line_Flag = 0;
        }
        else if(gd == 0XF3 || gd == 0XFB) //2���м�4��3��������⵽���ߣ�΢�ҹ� 1111 0011    1111 1011
        {
            LSpeed = Left + 50;
            RSpeed = 0;
            Line_Flag = 0;
        }
        else if(gd == 0XF9 || gd == 0XFD) //3���м�3��2��������⵽���ߣ���΢�ҹ�1111 1001   1111 1101
        {
            LSpeed = Left + 50;
            RSpeed = Right - 90;

            Line_Flag = 0;
        }
        else if(gd == 0XFC) //4���м�2��1��������⵽���ߣ�ǿ�ҹ�    1111 1100
        {
            LSpeed = Left + 90;
            RSpeed = Right - 120;
            Line_Flag = 0;
        }
        else if(gd == 0XFE) //5�����ұ�1��������⵽���ߣ���ǿ�ҹ�   1111 1110
        {
            LSpeed = Left + 120;
            RSpeed = Right - 150;
            Line_Flag = 1;
        }
    }

    if(Line_Flag != 1)
    {
        if(gd == 0XEF || gdg == 0X67) //1110 1111 x110 0111
        {
            RSpeed = Right + 50;
            LSpeed = 0;
            Line_Flag = 0;
        }
        if(gd == 0XCF || gd == 0XDF) //6���м�6��5��������⵽���ߣ�΢���
        {
            RSpeed = Right + 50;
            LSpeed = 0;
            Line_Flag = 0;
        }
        else if(gd == 0X9F || gd == 0XBF) //7���м�7��6��������⵽���ߣ���΢���
        {
            RSpeed = Right + 50;
            LSpeed = Left - 90;
            Line_Flag = 0;
            //tempMP = tempMP + 3;
        }
        else if(gd == 0X3F) //8���м�8��7��������⵽���ߣ�ǿ���
        {
            RSpeed = Right + 90;
            LSpeed = Left - 120;
            Line_Flag = 0;
        }
        else if(gd == 0X7F) //9������8��������⵽���ߣ���ǿ���
        {
            RSpeed = Right + 120;
            LSpeed = Left - 150;
            Line_Flag = 2;
        }
    }
	if(Mode == 1)
	{
		Control(LSpeed,RSpeed);
	}
	else if(Mode == 0)		//��ת����
	{
		Control(-RSpeed,-LSpeed);
	}
//	if(Track_Flag != 0)
//	{
//		Control(LSpeed,RSpeed);
//	}
}


//______________________________________________


void roadway_check_TimInit(uint16_t arr,uint16_t psc)
{
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	
	TIM_InitStructure.TIM_Period = arr;
	TIM_InitStructure.TIM_Prescaler = psc;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM9,&TIM_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM9, ENABLE);
}


void TIM1_BRK_TIM9_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM9,TIM_IT_Update) == SET)
	{
		MP=Roadway_mp_Get(); //����ͬ��
		Roadway_Check();								//·�����
	}
	TIM_ClearITPendingBit(TIM9,TIM_IT_Update);
}





//end file



