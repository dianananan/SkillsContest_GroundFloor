#include "activity.h"
#include "stm32f4xx.h"
#include "Init.h"
#include "roadway_check.h"
#include "canp_hostcom.h"
#include "task.h"
#include "data_base.h"
#include "PathPlanning.h"

//寻卡时，卡有时候会摆设在中间挡住循迹
//记录上一次的小车动作，进行二次判断
//设置小车行走模式


void Go_Test( u8 sp , u16 len) // 以50的速度前进
{
	Roadway_mp_syn();MP=0;
    G_Flag = 1;
    Stop_Flag = 0;
//    temp_MP = 0;
    temp_MP = len;
    Car_Spend = sp;
    Control(Car_Spend, Car_Spend);
}

void Back_Test(u8 sp , u16 len ) // 以50的速度后退
{
	Roadway_mp_syn();MP=0;
    B_Flag = 1;
    Stop_Flag = 0;
//    temp_MP = 0;
    temp_MP = len;
    Car_Spend = sp;
    Control(-Car_Spend, -Car_Spend);

}

void Left_Test( u8 sp, u8 NAV)  // 以80的速度左转弯
{
	Roadway_mp_syn();MP=0;
    L_Flag = 1;
    wheel_Nav_Flag=NAV;
    Stop_Flag = 0;
    Car_Spend = sp;
    Control(-Car_Spend, Car_Spend);
}

void Right_Test(u8 sp,u8 NAV)  // 以80的速度右转弯
{
	Roadway_mp_syn();MP=0;
    R_Flag = 1;
    wheel_Nav_Flag=NAV;
    Stop_Flag = 0;
    Car_Spend = sp;
    Control(Car_Spend, -Car_Spend);
}


void Track_Test( u8 sp, u16 len, u8 cntrod, u8 state) // 已50的速度循迹路口
{
	Roadway_mp_syn();MP=0;
	RodCnt=0;  //路口计数
	TraLen=0;
    intocorner = 0;
    reflag = 0;//清空路口数量
    Car_Spend = sp;
    Track_Flag = state;
    TraLen = len;
    RodCnt = cntrod;
//    Host_Open_UpTrack( Up_Track_Speed );  // 开启寻迹数据上传
//    delay_ms(100); // 等待一会寻迹数据上传
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
	Roadway_Flag_clean();  //清除标志位
	
	TIM1->CCR1 = 100;
    TIM1->CCR2 = 100;
    TIM1->CCR3 = 100;
    TIM1->CCR4 = 100;
	
    TIM1->BDTR &=~(1<<15);  //开启OC和OCN输出
	
	TIM_Cmd(TIM9,DISABLE);
	Send_UpMotor(0 , 0);
	Roadway_mp_syn(); 	//同步码盘
}

void Stop_Test(void)    //停止动作
{	
    startAction();
    STOP();
    if(getTaskState() == 0)  //如果当前没有任务，则结束动作
    {
        endAction();
    }
}



RunContorl isRunControl;  //小车行走任务结构体

void runControl(void)
{
    if(getActionState() || getTaskState() || getRunState() == 0) //不可以行走
    {
        return ;
    }
    else if(getNowPathVaule() == NOW_TASK || getNowPathVaule() == NOW_BARRIERSZ || getNowPathVaule() == NOW_TRAFFIC || getNowPathVaule() == NOW_ETC)
    {
        return ;
    }
    else if(isRunControl.PathCount > isRunControl.NowPot)		//路线还没有走完
    {	
        startAction();
        Car_Run(getNowPathVaule());
        isRunControl.NowPot++;
    }
    else 			//路线走完了
    {
        getNEWTask();
    }
}


u8 Car_Run(u8 order)//行走函数
{
    //	char data[10];
    //	sprintf(data,"t = %d\n",order);
    //	Send_Debug_Info(data,strlen(data));
    switch(order)
    {
    case LEFT://左转
		delay_ms(50);
        if(L_Flag == 0)
            Left_Test(TURNSPEED,NAV90);//左转
        break;
    case RIGHT://右转
		delay_ms(50);
        if(R_Flag == 0)
            Right_Test(TURNSPEED,NAV90);//右转
        break;
    case GO://前进
		delay_ms(50);
        if(G_Flag == 0)
            Go_Test(CARSPEED, getMPVaule());
        break;
    case BACK://后退
		delay_ms(50);
        if(B_Flag == 0)
            Back_Test(CARSPEED, getMPVaule()); //后退
        break;
    case CNTONE://循迹第一个十字路口
        //如果有障碍 重新生成路线，break;
		delay_ms(50);
        if(Track_Flag == 0){
            Track_Test(CARSPEED, ZERO, ROADONE, ROADMODE); //循迹一个路口
				}
        break;
    case CNTTWO:
        //如果有障碍 重新生成路线，break;
		delay_ms(50);
        if(Track_Flag == 0)
            Track_Test(CARSPEED, ZERO, ROADTWO, ROADMODE); //循迹路循迹第二个十字路口
        break;
    case CNTTHREE:
        //如果有障碍 重新生成路线，break;
		delay_ms(50);
        if(Track_Flag == 0)
            Track_Test(CARSPEED, ZERO, ROADTHREE, ROADMODE); //循迹三个十字路口
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
		
    case TRACKLENTH://循迹走mp
        //如果有障碍 重新生成路线，break;
		delay_ms(50);
        if(Track_Flag == 0)
//			 Track_Test(CARSPEED,*(t+taskindex),ZERO,LENMODE);
            Track_Test(CARSPEED, getMPVaule(), ZERO, LENMODE); 
        break;
		
//***********************************************************************************************//		
    case MIDHALF://循迹中等长度的一半
		delay_ms(50);
        if(Track_Flag == 0)
            Track_Test(CARSPEED, 70, ZERO, LENMODE); //循迹中等长度的一半
        break;
		
//***********************************************************************************************//			
    case TRACKBLACK://循迹循迹到一个路口准确停在黑线上
		delay_ms(50);
        if(Track_Flag == 0)
            Track_Test(CARSPEED, ZERO, ZERO, BLACKMODE); //循迹循迹到一个路口准确停在黑线上
        break;
		
    case TRAMP://自定义循迹走码盘值
		delay_ms(50);
        if(Track_Flag == 0)
            trackLength(getRunMpValue(0, 0.5, 0, 0.5));
        break;
		
    case MIDDLE_LONGISH: //循迹一半的长度x轴
		delay_ms(50);
        if(Track_Flag == 0)
            trackLength(MAXHALFLEN);
        break;
		
    case MIDDLE_SHORTE://循迹一半的长度y轴
		delay_ms(50);
        if(Track_Flag == 0)
            trackLength(MIDHALFLEN);
        break;
		
    case LEFT45://左转45度
		delay_ms(50);
        if(wheel_Nav_Flag == 0)
            Left_Test(80,NAV45);
        break;
		
    case RIGHT45://右转45度
		delay_ms(50);
        if(wheel_Nav_Flag == 0)
            Right_Test(80,NAV45);
        break;
		
    case LEFT180://左转180度
		delay_ms(50);
        if(wheel_Nav_Flag == 0)
            Left_Test(80,NAV180);
        break;
		
    case RIGHT180://右转180度
		delay_ms(50);
        if(wheel_Nav_Flag == 0)
            Right_Test(80,NAV180);
        break;

    case RIGHTSMALL://右转前循迹一小段距离
		delay_ms(50);
        if(G_Flag == 0)
			Go_Test(50,RSMALLLEN);//右转前循迹一小段距离//老车18
//            Track_Test(CARSPEED, RSMALLLEN, ZERO, LENMODE);
        
        break;

    case LEFTSMALL://左转前循迹一小段距离
		delay_ms(50);
        if(G_Flag == 0)
				 Go_Test(50,LSMALLLEN);//左转前的一小段距离//老车18
//            Track_Test(CARSPEED, LSMALLLEN, ZERO, LENMODE);
        break;
//***********************************************************************************************//		
    case GOCARBOYLEN:
		delay_ms(100);
        if(G_Flag == 0)
            Go_Test(CARSPEED, getRunMpValue(2, 1, 0.5, 1.5)); //前进一小段
        break;
//***********************************************************************************************//				
    case STOPCAR:
        Stop_Test();
        break;
    case GO_TERRAIN:
		TrackingLamp_Test(CARSPEED);
		break;
//        Track_Test(CARSPEED, ZERO, ZERO, TRACKTERRAIN); //走地形标志物
//        delay_ms(50);
//        break;
//	case TRACK_LINE_SHORT:
//		Track_Test(CARSPEED, MP_LINE_SHORT, ZERO, LENMODE); //走地形标志物
//		break;
//	case TRACK_LINE_LANG:
//		Track_Test(CARSPEED, MP_LINE_LANG, ZERO, LENMODE); //走地形标志物
//		break;
	case DEBUG:
		endAction();
		break;
    default:
        return 0;
    }
    return 1;
}

u8 getRunMpValue(int type, float typenum, float carbody, float linewidthnum) //mp计算函数  1 0 1 0
{
    u8 mpValue = 0;
    float temp = 0.0;
    if(type == 0)//最大长度
    {
        temp =(float) (typenum * 719.0f + carbody * 290.0f + linewidthnum * 30.0f);
    }
    else if(type == 1)//中等长度
    {
        temp =(float) (typenum * 567.0f + carbody * 290.0f + linewidthnum * 30.0f);
    }
    else if(type == 2)//最小长度
    {
        temp =(float) (typenum * 370.0f + carbody * 290.0f + linewidthnum * 30.0f);
    }

    mpValue =(u8) temp / 5.33f;
    return mpValue;
}

void trackLength(int tramp)
{
    if(Track_Flag == 0)
        Track_Test(CARSPEED, tramp, ZERO, LENMODE); //循迹边上长的长度一半
    delay_ms(50);
}
u8 getNowPathVaule(void)
{
    return 	isRunControl.PathGather[isRunControl.NowPot];
}
void InitRunPathContorl(u8 *gather, u8 len) //初始化
{
	  u8 i;
  if(len >= MAXPATH) //如果超过最大长度
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



//***********************************任务标志******************************************//
runtimeControl isrun; 

void runtimeInit() //初始化设置
{
    isrun.taskflag = 0;
    isrun.runflag = 0;
    isrun.actionsign = 0;
    isrun.TaskCarryOut = 0;//表示默认为 启动任务
}
void startAction(void) //启动一个动作
{
    Track_Flag = 0;
    isrun.actionsign = 1;
}
u8 getActionState(void)	//获取当前动作
{
    return isrun.actionsign;
}
void endAction(void) //结束动作
{
    isrun.actionsign = 0;
//	  STOP();
}
void startRun(void)	//开始运行
{
    isrun.runflag = 1;
//    RFID = 0;
}
void endRun(void)	//结束运行
{
    isrun.runflag = 0;
//    RFID = 0;
}
u8 getRunState(void)	//获取运行状态
{
    return isrun.runflag;
}
void startTask(void)	//开始任务
{
    isrun.taskflag = 1;
}
void endTask(void)	//结束任务
{
		isrun.taskflag = 0;
}
u8 getTaskState(void)	//获取任务状态
{
    return isrun.taskflag;
}

