#include <string.h>
#include "TaskWifiChannel.h"
#include "TaskZigbeeChannel.h"
#include "data_base.h"
#include "PathPlanning.h"
#include "delay.h"
#include "task.h"
#include "activity.h"
#include "tba.h"
#include "canp_hostcom.h"
#include "infrared.h"
#include "uart_a72.h"
#include "can_user.h"
#include "infrared.h"
#include "Init.h"


//u32 counttime = 0;   //定时器计数值

//************************************
//道闸//Etc//
//发送车牌，图片识别，红绿灯，交通灯//特殊处理
//***********************************

int i = 0;
u8 taskover = 0;//任务点已完成
u8 wifi_send_QR_flag = 0;//二维码识别点已到达
u8 wifi_send_PLATE_flag = 0;//车牌识别点已到达
u8 wifi_send_SHAPE_flag = 0;//图形识别点已到达
u8 wifi_send_GARAGE_flag = 0;//得到计算的入库点
u8 wifi_send_HLLIGHT_flag = 0; //红绿灯识别点已到达

u8 light_xxx=2;

//u8 shapedata[3] = {0};//图形识别数据
u8 HW_SEND_SHAPE[10] = {0};//形状
u8 GETPONT[3] = {0};//wifi接收坐标

u8 getWifiData[3];
u8 qrread[1] = {0x00};
u8 times = 0;//红外发送次数控制
u8 times1 = 0;////红外发送次数控制
u8 wifi_rev_card_flag_1 = 0;//车牌的前三位接收标志
u8 wifitask = 0;
u8 Timeout;


//u8 Light_Rank = 0; //光源挡位
u8 Dispose_Data_array[20];//用来解析来自WiFi中的信号
u8 Dispose_array_num = 1; //数组中信息的个数 //默认为1
//u8 fulfill_flag = 0; //wifi命令执行成功标志位
u8 con_num = 0;

u8 TempArray[8]={0x55,0x0e,0x02,0x00,0x00,0x00,0x00,0xbb};

void hw_Data_Dispose(u8 *Data) //处理立体显示的红外信息
{
    switch(Data[2])
    {
    case 0x01 :  //显示距离
        //Transmition(SD_JL, 6);
        setNowTask(TaskHW, HW_PLATESHOW, 0);
        break;
    case 0x02 :  //显示图形
        //Transmition(CP_G1, 6);
        setNowTask(TaskHW, HW_TYPESHOW, 0);
        break;
    case 0x03 :  //显示颜色
        CP_G2[2] = Data[5];
        //Transmition(CP_G1, 6);
        setNowTask(TaskHW, HW_HUESHOW, 0);
        break;
    case 0x04 :  //显示路况
        SD_LK[2] = Data[5];
        //Transmition(SD_LK, 6);
        setNowTask(TaskHW, HW_LKSHOW, 0);
        break;
    case 0x05 :   //显示默认
        //Transmition(SD_MRXX, 6);
        setNowTask(TaskHW, HW_LASTSHOW, 0);
        break;
    }
}


void Wifi_Send_Dispose_zigbee(u8 *Wifi_signal)  //处理zigbee信息 //道闸和//红绿灯特别处理
{
    Send_ZigbeeData_To_Fifo(Wifi_signal, 8);
}


void Wifi_Send_Dispose_hw(u8 *Wifi_signal)  //处理红外信息
{
    switch(Wifi_signal[3])
    {
    case 0x01 :
        Infrared_Send(HW_K, 6); //报警台开
        break;

    case 0x02 :  //立体显示
        hw_Data_Dispose(Wifi_signal);
        break;

    case 0x03 ://光源几档
        light_control(Wifi_signal[4]);
        break;

    case 0x04 :
        if(Wifi_signal[4] == 0x01) //照片上翻
            Infrared_Send(H_S, 4);
        else if(Wifi_signal[4] == 0x00) //照片下翻
            Infrared_Send(H_X, 4);
        break;
    default  :
        return ;
    }
    //send_data_wifi(Wifi_signal, 8);	//发送完成信号
}


void Wifi_Send_Dispose_run(u8 *Wifi_signal)//处理行走命令
{
    switch(Wifi_signal[3])
    {
    case 0x01 :
        Dispose_Data_array[0] = CNTONE;
        break;
    case 0x02 :
        Dispose_Data_array[0] = CNTTWO;
        break;
    case 0x03 :
        Dispose_Data_array[0] = CNTTHREE;
        break;
    case 0x04 :
        Dispose_Data_array[0] = CNTFOUR;
        break;
    case 0x05 :
        Dispose_Data_array[0] = CNTFIVE;
        break;
    case 0x06 :  //前进走码盘值
        if(Wifi_signal[4] == 0x01) //循迹
        {
            setMPVaule(Wifi_signal[5]);
            Dispose_Data_array[0] = TRACKLENTH;
        }
        else if(Wifi_signal[4] == 0x02) //不循迹
        {
            setMPVaule(Wifi_signal[5]);
            Dispose_Data_array[0] = GO;
        }
        break;
    case 0x07 :  //倒退
        setMPVaule(Wifi_signal[5]);
        Dispose_Data_array[0] = BACK;
        break;
    default  :
        return ;
    }
    InitRunPathContorl(Dispose_Data_array, 1); //组建数组,执行
}

void Wifi_Send_Dispose_action(u8 *Wifi_signal)//处理动作
{
    switch(Wifi_signal[3])
    {
    case 0x01 :
        Dispose_Data_array[0] = LEFT45;
        break;
    case 0x02 :
        Dispose_Data_array[0] = LEFT;
        break;
    case 0x03 :
        Dispose_Data_array[0] = LEFT180;
        break;
    case 0x04 :
        Dispose_Data_array[0] = RIGHT45;
        break;
    case 0x05 :
        Dispose_Data_array[0] = RIGHT;
        break;
    case 0x06 :
        Dispose_Data_array[0] = RIGHT180;
        break;
    case 0x07 :
        Go_Test(CARSPEED, 15);
    case 0x08 :
        Go_Test(CARSPEED, 14);
    default  :
        return ;
    }
    InitRunPathContorl(Dispose_Data_array, 1); //组建数组,执行
    //setNowTask(TaskWifi,CMD_TRAFFIC_READ);
}

void Wifi_Send_Dispose_Car(u8 *Wifi_signal)  //小车电原件
{
    switch(Wifi_signal[3])
    {
    case 0x01 : //蜂鸣器
        if(Wifi_signal[4]==0x00)  //关
            Set_tba_Beep(0);
        else if(Wifi_signal[4]==0x01) //开
            Set_tba_Beep(1);
        break;

    case 0x02 : //左灯
        if(Wifi_signal[4]==0x00)
            Set_tba_WheelLED(L_LED,1);
        else if(Wifi_signal[4]==0x01)
            Set_tba_WheelLED(L_LED,0);
        break;

    case 0x03 : //右灯
        if(Wifi_signal[4]==0x00)
            Set_tba_WheelLED(R_LED,1);
        else if(Wifi_signal[4]==0x01)
            Set_tba_WheelLED(R_LED,1);
        break;

    default :
        break;
    }
}
//判断上层返回信息 如果返回了 则将对应的标志位设置为 0xff
void switchBackinfo(u8 vaule)
{
    switch (vaule)
    {
    case 0x01:
        wifi_send_QR_flag = 0xff;
        break;
    case 0x02:
        wifi_send_PLATE_flag = 0xff;
        break;
    case 0x03:
        wifi_send_SHAPE_flag = 0xff;
        break;
//    case 0x04:
//        wifi_send_TRAFFIC_flag = 0xff;
//        break;
    case 0x05:
        wifi_send_HLLIGHT_flag = 0xff;
        break;
    default:
        break;
    }
}
void Wifi_Remote_Control () //wifi信号接收
{
//    u8 data[30];
    u8 readBuf[8];
    u8 i;
    if(Wifi_Rx_flag > 0) //有接收到WiFi信号
    {
        if((Head->StackData[0] == 0x55 && Head->StackData[7] == 0xbb) 
			|| (Head->StackData[0] == 0xff && Head->StackData[7] == 0xf0) ) //两种协议的包头
        {
            if(Normal_data(1, Head->StackData)) //正常数据校验码验和
			{
				for(i=0;i<8;i++)
					readBuf[i] = Head->StackData[i];
			}
        }		
		Head=Head->Next;
        Wifi_Rx_flag--;
    }

    if(Rx_Flag == 1) //正确信息，执行wifi命令
    {
        Rx_Flag = 0;
		Send_ZigbeeData_To_Fifo(readBuf,8); //打印数据
        if(readBuf[0] == 0x55 && readBuf[7] == 0xbb && readBuf[1] != 0xAA) //ZigBee信息直接转发
        {
            //Wifi_Send_Dispose_zigbee(Wifi_Rx_Buf);
        }
        else if(readBuf[0] == 0xff && readBuf[7] == 0xf0 && readBuf[1] == 0x01)//遥控
        {
            switch(readBuf[2])
            {
				case 0x01 :
					Wifi_Send_Dispose_run(readBuf); //行走命令
					break;
				case 0x02:
					Wifi_Send_Dispose_action(readBuf);//动作命令
					break;
				case 0x03 ://二维码任务点命令
					wifi_send_QR_flag = 1;
					break;
				case 0x04:
					STOP();
					break;
				case 0x05 :
					Wifi_Send_Dispose_hw(readBuf);		//接收红外信号
					TaskMenu(); //执行命令
					break;
				case 0x06 :
					Wifi_Send_Dispose_Car(readBuf); //元件命令
					break;
				case 0x07 ://二维码任务点命令
					light_xxx=readBuf[3];
					Send_ZigbeeData_To_Fifo(readBuf,8);
					wifi_send_QR_flag = 2;
					break;
				case 0x08 : //红绿灯
					TempArray[3] = (readBuf[3] + 1);	 //00/01/02
//					JTD_END[6] =CheckSum(JTD_END,3);
					TempArray[6] =TempArray[3];
					PrintfDebug(TempArray[3]);
					wifi_send_HLLIGHT_flag = 1; //接收到信息
					break;
//				case 0x09:
//					switchBackinfo(readBuf[3]);
//					break;
				default :
					break ;
            }
        }
		
        else if(readBuf[1] == 0xAA)		//第一套控制协议
		{
			switch(readBuf[2])//主车
			{
				case RUNSTART:
					Send_InfoData_To_Fifo((u8 *)"RUN\n",sizeof("RUM\n"));
					if(getRunState() == 0)
//					xydInit(&passivity.RFIDCard, getVauleX(readBuf[3]), getVauleY(readBuf[3]), 0);		//初始化RFID位置//不可删除
//					sprintf(data,"RFID %x\n", readBuf[3]);
//					Send_InfoData_To_Fifo((u8 *)data,strlen(data));
					startRun();
					isrun.TaskCarryOut = readBuf[4];//选择启动方式
					break;

				case QRCODE : //二维码识别
//					if(Wifi_Rx_Buf[3]==Wifi_Rx_Buf[4]) //所有点接收完成
//					 {
//						 initTask(Wifi_Rx_Buf[5],0,0,0,0); //将最后一个点设置为要到达的点 //00方向000（x）000（y）
//						 wifi_send_QR_flag=1;
//					 }
//					else if(Wifi_Rx_Buf[3]>Wifi_Rx_Buf[4])//没接收完就继续设置必进点
//					{
//						setLimitPot(Wifi_Rx_Buf[5],Wifi_Rx_Buf[4]);
//						setTaskLimitPot();
//						send_data_wifi(Wifi_Rx_Buf,8);
//					}
					break;
			case PLATEREV1://车牌1
				if(readBuf[3] == 0x01 && readBuf[4] == 0x01 && readBuf[5] == 0x01)
				{
					delay_ms(3000);
					Send_WifiData_To_Fifo(PlateRead, 8);
				}
				else
				{
					for(i = 0; i < 3; i++)
					{
						MailboxRe.PlateNumber[0][i]=readBuf[3+i];
					}
					wifi_rev_card_flag_1 = 1;
				}
				wifi_send_PLATE_flag = 1;
				break;
				
			case PLATEREV2://车牌2
				for(i = 0; i < 3; i++)
					MailboxRe.PlateNumber[0][i+3]=readBuf[3 + i];
//                CP_SHOW2[4] = 'A' + car_x;
//                CP_SHOW2[5] = '1' + car_y;
				// HW_Send_Choose(HW_PICUP);  //TFT图片上翻**************************************
				wifi_rev_card_flag_1 = 2;
				break;
			
			case SHAPE://图形识别
				if(readBuf[3] == 0x01 && readBuf[4] == 0x01 && readBuf[5] == 0x01) //如果数据不为这个则识别成功
				{
					//HW_Send_Choose(HW_PICUP);
					//send_data_zigbee(SMG_SHOWTWO,8);
					delay_ms(3000);
					Send_WifiData_To_Fifo(ShapeRead, 8);
				}
				else
				{
					// HW_Send_Choose(HW_PICUP);//******************************************
					wifi_send_SHAPE_flag = 1;
				}
				break;
			}
		}
        //memset(Wifi_Rx_Buf,0,sizeof(Wifi_Rx_Buf));//执行完后清空数组
    }
}

#define TIME_OUT    1000     //计时ms
void Wifi_Send_Dispose(u8 Wifi_signal)
{
    /***************************二维码***********************************************************/

    if((Wifi_signal == CMD_QR_READ) && (getTaskState() == 0)) //发送QR_Read cmd 二维码指令
    {
        startTask();
		wifi_send_QR_flag = 0;
		DelayTimerMS(3000); //等待摄像头矫正
		WaitTimer_ms=gt_get()+1000;
        memset(Dispose_Data_array, 0, sizeof(Dispose_Data_array));
        Send_WifiData_To_Fifo(QRCode, 8); //trackpath[0]---》改为具体的缓冲区--数组 发送二维码指令
        return ;			//不执行其他任务的判断
    }

    if(Wifi_signal == CMD_QR_READ ) //pad=>car read qr finished小车识别二维码结束
    {

        if(wifi_send_QR_flag == 0)			//如果没有接收到上层发来的回传信息
        {
            if(gt_get_sub(WaitTimer_ms) == 0)
            {
                ++WaitTimer_const;
                if(WaitTimer_const > 2)	//如果发送三次都没有收到则取消发送 强制结束任务     在中间可以加上处理失败以后的处理方法
                {
                    endTask();
                }
                else
                {
                    Send_WifiData_To_Fifo(QRCode, 8); //重新发送
                    WaitTimer_ms=gt_get()+1000;//重新计时
                }
            }
        }
        else	if( (wifi_send_QR_flag == 1 || wifi_send_QR_flag == 2))         //已经完成了任务
        {
            wifi_send_QR_flag = 0; //wifi发送标志初始化
            con_num = 0;
            endTask();//任务完成
			EndWaitTim();
        }
        return ;
    }
    /*-----------------------------车牌---------------------------------*/

    if((Wifi_signal == CMD_PLATE_READ) && (getTaskState() == 0)) //发送车牌已到达指令
    {
        startTask();//有任务了
        wifi_send_PLATE_flag = 0; //wifi发送初始化
        DelayTimerMS(3000);
		WaitTimer_ms=gt_get()+1000;
        Send_WifiData_To_Fifo(PlateRead, 8);
        return ;			//不执行其他任务的判断
    }
    if(Wifi_signal == CMD_PLATE_READ )		//车牌识别完成
    {
        if(wifi_send_PLATE_flag == 0)
        {
            if(gt_get_sub(WaitTimer_ms) == 0)
            {
                ++WaitTimer_const;
                if(WaitTimer_const > 2)	//如果发送三次都没有收到则取消发送 强制结束任务
                {
                    endTask();
                }
                else
                {
                    Send_WifiData_To_Fifo(PlateRead, 8);
                    WaitTimer_ms=gt_get()+1000;
                }
            }
        }
        else  if( wifi_send_PLATE_flag == 1 && wifi_rev_card_flag_1 == 2)      //任务已完成
        {
            wifi_send_PLATE_flag = 0; //wifi发送标志初始化
            wifi_rev_card_flag_1 = 0;
            endTask();
			EndWaitTim();
        }
        return ;
    }

    /*--------------------------图形识别--------------------------------------*/

    if((Wifi_signal == CMD_SHAPE_READ) && (getTaskState() == 0)) //发送图形识别
    {
        startTask();//有任务了
		wifi_send_SHAPE_flag = 0; //wifi发送初始化
        DelayTimerMS(3000);
		WaitTimer_ms=gt_get()+1000;
        Send_WifiData_To_Fifo(ShapeRead, 8);
        return ;
    }
    if(Wifi_signal == CMD_SHAPE_READ ) //pad=>car read qr finished小车识别图片结束
    {

        if(wifi_send_SHAPE_flag == 0)
        {
            if(gt_get_sub(WaitTimer_ms) == 0)
            {
                ++WaitTimer_const;
                if(WaitTimer_const > 2)	//如果发送三次都没有收到则取消发送 强制结束任务
                {
                    endTask();
                }
                else
                {
                    Send_WifiData_To_Fifo(PlateRead, 8);
                    WaitTimer_ms=gt_get()+1000;
                }
            }
        }
        else if(wifi_send_SHAPE_flag == 1)
        {
            wifi_send_SHAPE_flag = 0; //wifi发送标志初始化
            endTask();//任务完成
			EndWaitTim();
        }
        return ;
    }
    /***************************红绿灯****************************************/
    if(Wifi_signal == CMD_LIGHT_READ && (getTaskState() == 0)) //红绿灯
    {
        startTask();//有任务了
		Send_InfoData_To_Fifo((u8 *)"0\n",sizeof("0\n"));
        wifi_send_HLLIGHT_flag = 0;
        DelayTimerMS(3000);
        Send_ZigbeeData_To_Fifo(JTD_READ, 8); //发送红绿灯进入识别模式
		delay_ms(600);
		WaitTimer_ms=gt_get()+2000;
        Send_WifiData_To_Fifo(TrafficLight,8);//发送到达信号
        return ;
    }
    else if(Wifi_signal == CMD_LIGHT_READ )
    {
        if(wifi_send_HLLIGHT_flag == 0)
        {
            if(gt_get_sub(WaitTimer_ms) == 0)
            {
                ++WaitTimer_const;
                if(WaitTimer_const > 4)	//如果发送三次都没有收到则取消发送 强制结束任务
                {
                    wifi_send_HLLIGHT_flag=1;
                }
                else
                {
                    Send_WifiData_To_Fifo(TrafficLight, 8);
					WaitTimer_ms=gt_get()+2000;
                }
            }
        }
        else if(  wifi_send_HLLIGHT_flag == 1)
        {
            RepeatedlySend_ZG(TempArray, 4,50); //发送识别结果
            wifi_send_HLLIGHT_flag = 0;
            endTask();
			EndWaitTim();
        }
        return ;
    }
//----------------------------------------寻找入库点--------------------------------------------
    //		if((*(q+taskindex)==CMD_REV_POINT) && (taskflag==0))//任务为寻找入库任务
    //		{
    //	      taskflag=1;//有任务了
    //				wifi_send_GARAGE_flag=0;//wifi发送初始
    //				delay_ms(40);
    //			  send_data_wifi(GetPoint,8);//
    //		}
    //
    //		if(*(q+taskindex)== CMD_REV_POINT&& wifi_send_GARAGE_flag == 1)//
    //		{
    //			 taskflag=2;//任务完成
    //			 wifi_send_GARAGE_flag =0;//wifi发送标志初始化
    //		}
}
