#include "TaskZigbeeChannel.h"
#include <string.h>
#include "TaskWifiChannel.h"
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
#include "syn7318.h"
#include "Init.h"
#include "Timer.h"


u8 zig_send_door_flag = 0;//道闸发送标志位
u8 zig_send_seg_flag = 0;//数码管发送标志位
u8 zig_send_etc_flag = 0; //ETC
u8 zig_send_CK_flag = 0; //入库阶段
u8 Now_Plies = 0; //车库几层
u8 TriggerF_B = 0; //前后红外触发状态  =1后触发 =2前触发 =3全触发 =4没触发
u8 zig_send_CKC_flag = 0; //出库
u8 zigbee_count = 0; //计数


//六位显示数据(hex格式)
u8 sixbits[8] = {0x55, 0x0b, 0x40, 0x00, 0x00, 0x00, 0x40, 0xbb};

//距离显示模式（十进制）
u8 disshow[8] = {0x55, 0x0b, 0x50, 0x00, 0x00, 0x00, 0x50, 0xbb};


void Zigbee_Rev_Control(void)
{
    u8 buf[8];
    if(Zigbee_Rx_flag == 1)	 //zigbee返回信息
    {
        delay_us(5);
        memcpy(buf, Zigb_Rx_Buf, 8);
        memset(Zigb_Rx_Buf, 0, sizeof(Zigb_Rx_Buf));
        Zigbee_Rx_flag = 0;

        if( (buf[1] == 0x03) || (buf[1] == 0x0c)) // 道闸 或 ETC
        {
            if(buf[2] == 0x01)
            {
                if(buf[3] == 0x01 || (buf[3] == 0x00))
                {
                    if(buf[4] == 0x05) //道闸
                    {
                        zig_send_door_Rev_flag = 1;
                    }
                    else if(buf[4] == 0x06) //ETC
                    {
                        zig_send_etc_flag = 1;
                    }
                }
            }
        }
        else if(buf[1] == 0x0D && buf[2] == 0x03) //车库返回信息
        {
            if(buf[3] == 0x01) //返回车库几层
            {
                Now_Plies = buf[4]; //u8 number of plies层数
				PrintfDebug(Now_Plies);
            }
        }
        else if(buf[1] == 0x21)	 //第一组数据
        {
            C_Tab[2] = buf[2];
            C_Tab[3] = buf[3];
            C_Tab[4] = buf[4];
            C_Tab[5] = buf[5];
        }
        else if(buf[1] == 0x22)	 //第二组数据
        {
            C_Tab[6] = buf[2];
            C_Tab[7] = buf[3];
            C_Tab[8] = buf[4];
            C_Tab[9] = buf[5];
        }
        else if(buf[1] == 0x06)
        {
            if(buf[2] == 0x01)
            {
                zig_send_voice_Rev_flag = 1;
                // 语音芯片状态返回
            }
        }
        Zigbee_Rx_flag = 0;
    }
}

void Zig_Send_Dispose(u8 taskchoose)
{
/*********************************道闸****************************/
	if(taskchoose == CMD_ZG_DOOR)
	{
		if(getTaskState() == 0)
		{
			startTask();
			zig_send_door_flag = 0;
//			RepeatedlySend_ZG(DZ_K,1,20); //zigbb发送道闸开启命令
			WaitTimer_ms=gt_get()+1000;
		}
		if(zig_send_door_Rev_flag == 0)
		{
			if(gt_get_sub(WaitTimer_ms) == 0)
			{
				RepeatedlySend_ZG(DZ_K,4,20);
//				Send_ZigbeeData_To_Fifo(DZ_K,8);
				WaitTimer_const++;
				RepeatedlySend_ZG(DZ_R,4,20);
				WaitTimer_ms=gt_get()+1000;
			}
		}
		if(  zig_send_door_Rev_flag == 1 || WaitTimer_const >=4)
		{
			endTask();
			EndWaitTim();  //结束等待
			zig_send_door_Rev_flag = 0; //道闸接收到反馈信息，完成任务
			return ;
		}		
	}

/*************************************ETC******************************/
	if(taskchoose == CMD_ETC)
	{
		if(getTaskState() == 0)
		{
			startTask();
			zig_send_etc_flag = 0;
			WaitTimer_ms=gt_get()+2000;	
		}
		if(zig_send_etc_flag == 1 || gt_get_sub(WaitTimer_ms))
		{
			endTask();
			EndWaitTim();
			zig_send_etc_flag = 0; //接收完成
			return ;			
		}
	}
/**********************************数码管*******************************/
    if(taskchoose == CMD_ZG_SEG && getTaskState() == 0)
    {
        startTask();
       send_SEG_data(isRunTask.TaskVaule); //发送数码管开启命令，计时开始
        delay_ms(20);
        endTask();
        return ;
    }
/************************************语音****************************/
    if(taskchoose == CMD_VOICE && getTaskState() == 0)
    {
        startTask();
//        SYN7318_Open();  //语音开
//        SYN7318_Test();   //语音识别开启
		SYN_7318_One_test(1,0);	 //语音识别开启
        endTask();
        return ;
    }
    //*********************发送车牌到道闸*********************************/
    if(taskchoose == CMD_ZG_DOORCP && getTaskState() == 0)
    {
				DZ_CPF[3] = 65;
				DZ_CPF[4] = 65;
				DZ_CPF[5] = 65;
				DZ_CPF[6] = (DZ_CPF[2]+DZ_CPF[3]+DZ_CPF[4]+DZ_CPF[5])%255;
				DZ_CPB[3] = 65;
				DZ_CPB[4] = 65;
				DZ_CPB[5] = 65;
				DZ_CPB[6] =  (DZ_CPB[2]+DZ_CPB[3]+DZ_CPB[4]+DZ_CPB[5])%255;
				Send_InfoData_To_Fifo((u8 *)"sendcp\n",sizeof("snedcp\n"));delay_ms(500);
			
        startTask();
        Send_ZigbeeData_To_Fifo(DZ_CPF, 8);
        delay_ms(500);
        Send_ZigbeeData_To_Fifo(DZ_CPB, 8);
        delay_ms(500);
        endTask();
        return ;
    }
    /*************************************磁悬浮******************************/
    if(taskchoose == CMD_MAGLEV && getTaskState() == 0)
    {
        startTask();
        RepeatedlySend_ZG(MAGLEV, 8, 20); //zigbb发送磁悬浮开启命令
        endTask();
        return ;
    }

    /***********************************TFT显示器****************************/
    if(taskchoose == CMD_TFT_SHOW && getTaskState() == 0)
    {
        startTask();
        send_TFT_data(isRunTask.TaskVaule);
        endTask();
        return ;
    }
    /**********************************入车库*****************************************/
    if(taskchoose == CMD_GETPOINT1) //车库在第一层，没有其他动作在执行
    {
        if(getTaskState() == 0) //开启任务
		{
			startTask();
			zig_send_CK_flag = 1;	
			WaitTimer_ms=gt_get()+1000;
		}
		else if(zig_send_CK_flag == 1 ) //倒退
		{
			Back_Test(30,1100);
			TACKZERO();
			zig_send_CK_flag=2;	
			RepeatedlySend_ZG(LTCK_K[getNowGarage()-1],4,10);
		}
		if(zig_send_CK_flag == 2)	//入库
		{
			if(Now_Plies ==getNowGarage() || WaitTimer_const>15)
			{
				endTask();
				EndWaitTim();
				zig_send_CK_flag = 0;
			}
			else if(gt_get_sub(WaitTimer_ms) == 0)
			{
				++WaitTimer_const;
				WaitTimer_ms=gt_get()+1000;
				RepeatedlySend_ZG(LTCK_R,4,10);
			}			
		}
    }
}


void TACKZERO() //走路线延迟
{
    while(1)
    {
        if(G_Flag == 0 && B_Flag == 0 && Track_Flag == 0)break;
    }
	MP=0;
}



void send_TFT_data(u8 mode)//发送zigbee
{
    switch(mode)
    {
    case WITCH_PIC://由第二副指令指定显示哪张图片
        Send_ZigbeeData_To_Fifo(witch_pic, 8);
        break;
    case PIC_UP://图片向上翻
        Send_ZigbeeData_To_Fifo(pic_up, 8);
        break;
    case PIC_DOWN://图片向下翻
        Send_ZigbeeData_To_Fifo(pic_down, 8);
        break;
    case AUTOMATION_PIC://图片自动向下翻页显示，间隔10s
        Send_ZigbeeData_To_Fifo(auto_show, 8);
        break;
    case PLATEMODE://TFT车牌显示模式
        Send_ZigbeeData_To_Fifo(TFT_CPF, 8);
        delay_ms(500);
        Send_ZigbeeData_To_Fifo(TFT_CPB, 8);
        delay_ms(500);
        break;
    case CLOSE_COUNT://计时模式关闭
        Send_ZigbeeData_To_Fifo(countclose, 8);
        break;
    case OPEN_COUNT://计时模式打开
        Send_ZigbeeData_To_Fifo(countopen, 8);
        break;
    case CLEAR_COUNT://计时模式清零
        Send_ZigbeeData_To_Fifo(countclean, 8);
        break;
    case HEXSHOW://HEX显示模块
        Send_ZigbeeData_To_Fifo(sixbits, 8);
        break;
    case DISMODE://距离显示模式（十进制）
        Send_ZigbeeData_To_Fifo(disshow, 8);
        break;
    default:
        break;
    }
}

void send_SEG_data(u8 mode)
{
	switch(mode)
	{
		case OPEN_SEG :  //打开计数器
			RepeatedlySend_ZG(SMG_JSK,4,10);
			break;
		case CLOSE_SEG :	//关闭计数器
			RepeatedlySend_ZG(SMG_JSG,4,10);
			break;
		case SHOW1_SEG ://显示第一排
			Send_ZigbeeData_To_Fifo(SMG_SHOW,8);
			break;
		case SHOW2_SEG ://显示第二排
			Send_ZigbeeData_To_Fifo(SMG_SHOWTWO,8);
			break;
		case DIS_SEG:	//显示距离
			Send_ZigbeeData_To_Fifo(SMG_JL,8);
			break;
		default:
			break;
	}
}

void SendVoice(u8 *data, int length)	//发送语音播报
{
    u8 head[] =  {0xfd, 0x00, 0x00, 0x01, 0x01};
    head[2] = length + 2;
    Send_ZigbeeData_To_Fifo(head, 5);
    Send_ZigbeeData_To_Fifo(data, length);
}


