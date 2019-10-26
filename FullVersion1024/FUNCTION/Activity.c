#include "activity.h"
#include "stm32f4xx.h"
#include "Init.h"
#include "roadway_check.h"
#include "canp_hostcom.h"
#include "task.h"
#include "data_base.h"
#include "PathPlanning.h"

//Ѱ��ʱ������ʱ���������м䵲סѭ��
//��¼��һ�ε�С�����������ж����ж�
//����С������ģʽ


void Go_Test( u8 sp , u16 len) // ��50���ٶ�ǰ��
{
	Roadway_mp_syn();MP=0;
    G_Flag = 1;
    Stop_Flag = 0;
//    temp_MP = 0;
    temp_MP = len;
    Car_Spend = sp;
    Control(Car_Spend, Car_Spend);
}

void Back_Test(u8 sp , u16 len ) // ��50���ٶȺ���
{
	Roadway_mp_syn();MP=0;
    B_Flag = 1;
    Stop_Flag = 0;
//    temp_MP = 0;
    temp_MP = len;
    Car_Spend = sp;
    Control(-Car_Spend, -Car_Spend);

}

void Left_Test( u8 sp, u8 NAV)  // ��80���ٶ���ת��
{
	Roadway_mp_syn();MP=0;
    L_Flag = 1;
    wheel_Nav_Flag=NAV;
    Stop_Flag = 0;
    Car_Spend = sp;
    Control(-Car_Spend, Car_Spend);
}

void Right_Test(u8 sp,u8 NAV)  // ��80���ٶ���ת��
{
	Roadway_mp_syn();MP=0;
    R_Flag = 1;
    wheel_Nav_Flag=NAV;
    Stop_Flag = 0;
    Car_Spend = sp;
    Control(Car_Spend, -Car_Spend);
}


void Track_Test( u8 sp, u16 len, u8 cntrod, u8 state) // ��50���ٶ�ѭ��·��
{
	Roadway_mp_syn();MP=0;
	RodCnt=0;  //·�ڼ���
	TraLen=0;
    intocorner = 0;
    reflag = 0;//���·������
    Car_Spend = sp;
    Track_Flag = state;
    TraLen = len;
    RodCnt = cntrod;
//    Host_Open_UpTrack( Up_Track_Speed );  // ����Ѱ�������ϴ�
//    delay_ms(100); // �ȴ�һ��Ѱ�������ϴ�
    TIM_Cmd(TIM9, ENABLE);
}

void TrackingLamp_Test(u8 sp)
{
	Roadway_mp_syn();MP=0;
	TrackingLamp_Flag =1;
    intocorner = 0;
    Car_Spend = sp;
    Control(Car_Spend,Car_Spend);
}

void STOP(void)
{
	Roadway_Flag_clean();  //�����־λ
	
	TIM1->CCR1 = 100;
    TIM1->CCR2 = 100;
    TIM1->CCR3 = 100;
    TIM1->CCR4 = 100;
	
    TIM1->BDTR &=~(1<<15);  //����OC��OCN���
	
	TIM_Cmd(TIM9,DISABLE);
	Send_UpMotor(0 , 0);
	Roadway_mp_syn(); 	//ͬ������
}

void Stop_Test(void)    //ֹͣ����
{	
    startAction();
    STOP();
    if(getTaskState() == 0)  //�����ǰû���������������
    {
        endAction();
    }
}



RunContorl isRunControl;  //С����������ṹ��

void runControl(void)
{
    if(getActionState() || getTaskState() || getRunState() == 0) //����������
    {
        return ;
    }
    else if(getNowPathVaule() == NOW_TASK || getNowPathVaule() == NOW_BARRIERSZ || getNowPathVaule() == NOW_TRAFFIC || getNowPathVaule() == NOW_ETC)
    {
        return ;
    }
    else if(isRunControl.PathCount > isRunControl.NowPot)		//·�߻�û������
    {	
        startAction();
        Car_Run(getNowPathVaule());
        isRunControl.NowPot++;
    }
    else 			//·��������
    {
        getNEWTask();
    }
}


u8 Car_Run(u8 order)//���ߺ���
{
    //	char data[10];
    //	sprintf(data,"t = %d\n",order);
    //	Send_Debug_Info(data,strlen(data));
    switch(order)
    {
    case LEFT://��ת
		delay_ms(50);
        if(L_Flag == 0)
            Left_Test(TURNSPEED,NAV90);//��ת
        break;
    case RIGHT://��ת
		delay_ms(50);
        if(R_Flag == 0)
            Right_Test(TURNSPEED,NAV90);//��ת
        break;
    case GO://ǰ��
		delay_ms(50);
        if(G_Flag == 0)
            Go_Test(CARSPEED, getMPVaule());
        break;
    case BACK://����
		delay_ms(50);
        if(B_Flag == 0)
            Back_Test(CARSPEED, getMPVaule()); //����
        break;
    case CNTONE://ѭ����һ��ʮ��·��
        //������ϰ� ��������·�ߣ�break;
		delay_ms(50);
        if(Track_Flag == 0){
            Track_Test(CARSPEED, ZERO, ROADONE, ROADMODE); //ѭ��һ��·��
				}
        break;
    case CNTTWO:
        //������ϰ� ��������·�ߣ�break;
		delay_ms(50);
        if(Track_Flag == 0)
            Track_Test(CARSPEED, ZERO, ROADTWO, ROADMODE); //ѭ��·ѭ���ڶ���ʮ��·��
        break;
    case CNTTHREE:
        //������ϰ� ��������·�ߣ�break;
		delay_ms(50);
        if(Track_Flag == 0)
            Track_Test(CARSPEED, ZERO, ROADTHREE, ROADMODE); //ѭ������ʮ��·��
        break;
    case CNTFOUR:
		delay_ms(50);
        if(Track_Flag == 0)
            Track_Test(CARSPEED, ZERO, ROADFOUR, ROADMODE); 
        break;
    case CNTFIVE:
		delay_ms(50);
        if(Track_Flag == 0)
            Track_Test(CARSPEED, ZERO, ROADFIVE, ROADMODE); 
        break;
		
    case TRACKLENTH://ѭ����mp
        //������ϰ� ��������·�ߣ�break;
		delay_ms(50);
        if(Track_Flag == 0)
//			 Track_Test(CARSPEED,*(t+taskindex),ZERO,LENMODE);
            Track_Test(CARSPEED, getMPVaule(), ZERO, LENMODE); 
        break;
		
//***********************************************************************************************//		
    case MIDHALF://ѭ���еȳ��ȵ�һ��
		delay_ms(50);
        if(Track_Flag == 0)
            Track_Test(CARSPEED, 70, ZERO, LENMODE); //ѭ���еȳ��ȵ�һ��
        break;
		
//***********************************************************************************************//			
    case TRACKBLACK://ѭ��ѭ����һ��·��׼ȷͣ�ں�����
		delay_ms(50);
        if(Track_Flag == 0)
            Track_Test(CARSPEED, ZERO, ZERO, BLACKMODE); //ѭ��ѭ����һ��·��׼ȷͣ�ں�����
        break;
		
    case TRAMP://�Զ���ѭ��������ֵ
		delay_ms(50);
        if(Track_Flag == 0)
            trackLength(getRunMpValue(0, 0.5, 0, 0.5));
        break;
		
    case MIDDLE_LONGISH: //ѭ��һ��ĳ���x��
		delay_ms(50);
        if(Track_Flag == 0)
            trackLength(MAXHALFLEN);
        break;
		
    case MIDDLE_SHORTE://ѭ��һ��ĳ���y��
		delay_ms(50);
        if(Track_Flag == 0)
            trackLength(MIDHALFLEN);
        break;
		
    case LEFT45://��ת45��
		delay_ms(50);
        if(wheel_Nav_Flag == 0)
            Left_Test(80,NAV45);
        break;
		
    case RIGHT45://��ת45��
		delay_ms(50);
        if(wheel_Nav_Flag == 0)
            Right_Test(80,NAV45);
        break;
		
    case LEFT180://��ת180��
		delay_ms(50);
        if(wheel_Nav_Flag == 0)
            Left_Test(80,NAV180);
        break;
		
    case RIGHT180://��ת180��
		delay_ms(50);
        if(wheel_Nav_Flag == 0)
            Right_Test(80,NAV180);
        break;

    case RIGHTSMALL://��תǰѭ��һС�ξ���
		delay_ms(50);
        if(G_Flag == 0)
			Go_Test(50,RSMALLLEN);//��תǰѭ��һС�ξ���//�ϳ�18
//            Track_Test(CARSPEED, RSMALLLEN, ZERO, LENMODE);
        
        break;

    case LEFTSMALL://��תǰѭ��һС�ξ���
		delay_ms(50);
        if(G_Flag == 0)
				 Go_Test(50,LSMALLLEN);//��תǰ��һС�ξ���//�ϳ�18
//            Track_Test(CARSPEED, LSMALLLEN, ZERO, LENMODE);
        break;
//***********************************************************************************************//		
    case GOCARBOYLEN:
		delay_ms(100);
        if(G_Flag == 0)
            Go_Test(CARSPEED, getRunMpValue(2, 1, 0.5, 1.5)); //ǰ��һС��
        break;
//***********************************************************************************************//				
    case STOPCAR:
        Stop_Test();
        break;
    case GO_TERRAIN:
		TrackingLamp_Test(CARSPEED);
		break;
//        Track_Test(CARSPEED, ZERO, ZERO, TRACKTERRAIN); //�ߵ��α�־��
//        delay_ms(50);
//        break;
//	case TRACK_LINE_SHORT:
//		Track_Test(CARSPEED, MP_LINE_SHORT, ZERO, LENMODE); //�ߵ��α�־��
//		break;
//	case TRACK_LINE_LANG:
//		Track_Test(CARSPEED, MP_LINE_LANG, ZERO, LENMODE); //�ߵ��α�־��
//		break;
	case DEBUG:
		endAction();
		break;
    default:
        return 0;
    }
    return 1;
}

u8 getRunMpValue(int type, float typenum, float carbody, float linewidthnum) //mp���㺯��  1 0 1 0
{
    u8 mpValue = 0;
    float temp = 0.0;
    if(type == 0)//��󳤶�
    {
        temp =(float) (typenum * 719.0f + carbody * 290.0f + linewidthnum * 30.0f);
    }
    else if(type == 1)//�еȳ���
    {
        temp =(float) (typenum * 567.0f + carbody * 290.0f + linewidthnum * 30.0f);
    }
    else if(type == 2)//��С����
    {
        temp =(float) (typenum * 370.0f + carbody * 290.0f + linewidthnum * 30.0f);
    }

    mpValue =(u8) temp / 5.33f;
    return mpValue;
}

void trackLength(int tramp)
{
    if(Track_Flag == 0)
        Track_Test(CARSPEED, tramp, ZERO, LENMODE); //ѭ�����ϳ��ĳ���һ��
    delay_ms(50);
}
u8 getNowPathVaule(void)
{
    return 	isRunControl.PathGather[isRunControl.NowPot];
}
void InitRunPathContorl(u8 *gather, u8 len) //��ʼ��
{
	  u8 i;
  if(len >= MAXPATH) //���������󳤶�
    {
        return ;
    }
    for(i = 0; i < len; i++)
        isRunControl.PathGather[i] = gather[i];
    isRunControl.PathCount = len;
    isRunControl.NowPot = 0;
}
void setMPVaule(u16 vaule)
{
    isRunControl.mpvaule = vaule;
}
u16 getMPVaule(void)
{
    return isRunControl.mpvaule;
}



//***********************************�����־******************************************//
runtimeControl isrun; 

void runtimeInit() //��ʼ������
{
    isrun.taskflag = 0;
    isrun.runflag = 0;
    isrun.actionsign = 0;
    isrun.TaskCarryOut = 0;//��ʾĬ��Ϊ ��������
}
void startAction(void) //����һ������
{
    Track_Flag = 0;
    isrun.actionsign = 1;
}
u8 getActionState(void)	//��ȡ��ǰ����
{
    return isrun.actionsign;
}
void endAction(void) //��������
{
    isrun.actionsign = 0;
//	  STOP();
}
void startRun(void)	//��ʼ����
{
    isrun.runflag = 1;
//    RFID = 0;
}
void endRun(void)	//��������
{
    isrun.runflag = 0;
//    RFID = 0;
}
u8 getRunState(void)	//��ȡ����״̬
{
    return isrun.runflag;
}
void startTask(void)	//��ʼ����
{
    isrun.taskflag = 1;
}
void endTask(void)	//��������
{
		isrun.taskflag = 0;
}
u8 getTaskState(void)	//��ȡ����״̬
{
    return isrun.taskflag;
}

