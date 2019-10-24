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


u8 zig_send_door_flag = 0;//��բ���ͱ�־λ
u8 zig_send_seg_flag = 0;//����ܷ��ͱ�־λ
u8 zig_send_etc_flag = 0; //ETC
u8 zig_send_CK_flag = 0; //���׶�
u8 Now_Plies = 0; //���⼸��
u8 TriggerF_B = 0; //ǰ����ⴥ��״̬  =1�󴥷� =2ǰ���� =3ȫ���� =4û����
u8 zig_send_CKC_flag = 0; //����
u8 zigbee_count = 0; //����


//��λ��ʾ����(hex��ʽ)
u8 sixbits[8] = {0x55, 0x0b, 0x40, 0x00, 0x00, 0x00, 0x40, 0xbb};

//������ʾģʽ��ʮ���ƣ�
u8 disshow[8] = {0x55, 0x0b, 0x50, 0x00, 0x00, 0x00, 0x50, 0xbb};


void Zigbee_Rev_Control(void)
{
    u8 buf[8];
    if(Zigbee_Rx_flag == 1)	 //zigbee������Ϣ
    {
        delay_us(5);
        memcpy(buf, Zigb_Rx_Buf, 8);
        memset(Zigb_Rx_Buf, 0, sizeof(Zigb_Rx_Buf));
        Zigbee_Rx_flag = 0;

        if( (buf[1] == 0x03) || (buf[1] == 0x0c)) // ��բ �� ETC
        {
            if(buf[2] == 0x01)
            {
                if(buf[3] == 0x01 || (buf[3] == 0x00))
                {
                    if(buf[4] == 0x05) //��բ
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
        else if(buf[1] == 0x0D && buf[2] == 0x03) //���ⷵ����Ϣ
        {
            if(buf[3] == 0x01) //���س��⼸��
            {
                Now_Plies = buf[4]; //u8 number of plies����
				PrintfDebug(Now_Plies);
            }
        }
        else if(buf[1] == 0x21)	 //��һ������
        {
            C_Tab[2] = buf[2];
            C_Tab[3] = buf[3];
            C_Tab[4] = buf[4];
            C_Tab[5] = buf[5];
        }
        else if(buf[1] == 0x22)	 //�ڶ�������
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
                // ����оƬ״̬����
            }
        }
        Zigbee_Rx_flag = 0;
    }
}

void Zig_Send_Dispose(u8 taskchoose)
{
/*********************************��բ****************************/
	if(taskchoose == CMD_ZG_DOOR)
	{
		if(getTaskState() == 0)
		{
			startTask();
			zig_send_door_flag = 0;
//			RepeatedlySend_ZG(DZ_K,1,20); //zigbb���͵�բ��������
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
			EndWaitTim();  //�����ȴ�
			zig_send_door_Rev_flag = 0; //��բ���յ�������Ϣ���������
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
			zig_send_etc_flag = 0; //�������
			return ;			
		}
	}
/**********************************�����*******************************/
    if(taskchoose == CMD_ZG_SEG && getTaskState() == 0)
    {
        startTask();
       send_SEG_data(isRunTask.TaskVaule); //��������ܿ��������ʱ��ʼ
        delay_ms(20);
        endTask();
        return ;
    }
/************************************����****************************/
    if(taskchoose == CMD_VOICE && getTaskState() == 0)
    {
        startTask();
//        SYN7318_Open();  //������
//        SYN7318_Test();   //����ʶ����
		SYN_7318_One_test(1,0);	 //����ʶ����
        endTask();
        return ;
    }
    //*********************���ͳ��Ƶ���բ*********************************/
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
    /*************************************������******************************/
    if(taskchoose == CMD_MAGLEV && getTaskState() == 0)
    {
        startTask();
        RepeatedlySend_ZG(MAGLEV, 8, 20); //zigbb���ʹ�������������
        endTask();
        return ;
    }

    /***********************************TFT��ʾ��****************************/
    if(taskchoose == CMD_TFT_SHOW && getTaskState() == 0)
    {
        startTask();
        send_TFT_data(isRunTask.TaskVaule);
        endTask();
        return ;
    }
    /**********************************�복��*****************************************/
    if(taskchoose == CMD_GETPOINT1) //�����ڵ�һ�㣬û������������ִ��
    {
        if(getTaskState() == 0) //��������
		{
			startTask();
			zig_send_CK_flag = 1;	
			WaitTimer_ms=gt_get()+1000;
		}
		else if(zig_send_CK_flag == 1 ) //����
		{
			Back_Test(30,1100);
			TACKZERO();
			zig_send_CK_flag=2;	
			RepeatedlySend_ZG(LTCK_K[getNowGarage()-1],4,10);
		}
		if(zig_send_CK_flag == 2)	//���
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


void TACKZERO() //��·���ӳ�
{
    while(1)
    {
        if(G_Flag == 0 && B_Flag == 0 && Track_Flag == 0)break;
    }
	MP=0;
}



void send_TFT_data(u8 mode)//����zigbee
{
    switch(mode)
    {
    case WITCH_PIC://�ɵڶ���ָ��ָ����ʾ����ͼƬ
        Send_ZigbeeData_To_Fifo(witch_pic, 8);
        break;
    case PIC_UP://ͼƬ���Ϸ�
        Send_ZigbeeData_To_Fifo(pic_up, 8);
        break;
    case PIC_DOWN://ͼƬ���·�
        Send_ZigbeeData_To_Fifo(pic_down, 8);
        break;
    case AUTOMATION_PIC://ͼƬ�Զ����·�ҳ��ʾ�����10s
        Send_ZigbeeData_To_Fifo(auto_show, 8);
        break;
    case PLATEMODE://TFT������ʾģʽ
        Send_ZigbeeData_To_Fifo(TFT_CPF, 8);
        delay_ms(500);
        Send_ZigbeeData_To_Fifo(TFT_CPB, 8);
        delay_ms(500);
        break;
    case CLOSE_COUNT://��ʱģʽ�ر�
        Send_ZigbeeData_To_Fifo(countclose, 8);
        break;
    case OPEN_COUNT://��ʱģʽ��
        Send_ZigbeeData_To_Fifo(countopen, 8);
        break;
    case CLEAR_COUNT://��ʱģʽ����
        Send_ZigbeeData_To_Fifo(countclean, 8);
        break;
    case HEXSHOW://HEX��ʾģ��
        Send_ZigbeeData_To_Fifo(sixbits, 8);
        break;
    case DISMODE://������ʾģʽ��ʮ���ƣ�
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
		case OPEN_SEG :  //�򿪼�����
			RepeatedlySend_ZG(SMG_JSK,4,10);
			break;
		case CLOSE_SEG :	//�رռ�����
			RepeatedlySend_ZG(SMG_JSG,4,10);
			break;
		case SHOW1_SEG ://��ʾ��һ��
			Send_ZigbeeData_To_Fifo(SMG_SHOW,8);
			break;
		case SHOW2_SEG ://��ʾ�ڶ���
			Send_ZigbeeData_To_Fifo(SMG_SHOWTWO,8);
			break;
		case DIS_SEG:	//��ʾ����
			Send_ZigbeeData_To_Fifo(SMG_JL,8);
			break;
		default:
			break;
	}
}

void SendVoice(u8 *data, int length)	//������������
{
    u8 head[] =  {0xfd, 0x00, 0x00, 0x01, 0x01};
    head[2] = length + 2;
    Send_ZigbeeData_To_Fifo(head, 5);
    Send_ZigbeeData_To_Fifo(data, length);
}


