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


//************************************
//��բ//Etc//
//���ͳ��ƣ�ͼƬʶ�𣬺��̵ƣ���ͨ��//���⴦��
//***********************************

int i = 0;
u8 taskover = 0;//����������
u8 wifi_send_QR_flag = 0;//��ά��ʶ����ѵ���
u8 wifi_send_PLATE_flag = 0;//����ʶ����ѵ���
u8 wifi_send_SHAPE_flag = 0;//ͼ��ʶ����ѵ���
u8 wifi_send_GARAGE_flag = 0;//�õ����������
u8 wifi_send_HLLIGHT_flag = 0; //���̵�ʶ����ѵ���
u8 GraphChoose_Flag =0;		//ͼƬѡ��

//u8 shapedata[3] = {0};//ͼ��ʶ������
u8 HW_SEND_SHAPE[10] = {0};//��״
u8 GETPONT[3] = {0};//wifi��������

u8 getWifiData[3];
u8 qrread[1] = {0x00};
u8 times = 0;//���ⷢ�ʹ�������
u8 times1 = 0;////���ⷢ�ʹ�������
u8 wifi_rev_card_flag_1 = 0;//���Ƶ�ǰ��λ���ձ�־
u8 wifitask = 0;
u8 Timeout;

u8 Dispose_Data_array[20];//������������WiFi�е��ź�
u8 Dispose_array_num = 1; //��������Ϣ�ĸ��� //Ĭ��Ϊ1
u8 con_num = 0;

u8 TempArray[8]={0x55,0x0e,0x02,0x00,0x00,0x00,0x00,0xbb};

void hw_Data_Dispose(u8 *Data) //����������ʾ�ĺ�����Ϣ
{
    switch(Data[2])
    {
    case 0x01 :  //��ʾ����
        //Transmition(SD_JL, 6);
        setNowTask(TaskHW, HW_PLATESHOW, 0);
        break;
    case 0x02 :  //��ʾͼ��
        //Transmition(CP_G1, 6);
        setNowTask(TaskHW, HW_TYPESHOW, 0);
        break;
    case 0x03 :  //��ʾ��ɫ
        CP_G2[2] = Data[5];
        //Transmition(CP_G1, 6);
        setNowTask(TaskHW, HW_HUESHOW, 0);
        break;
    case 0x04 :  //��ʾ·��
        SD_LK[2] = Data[5];
        //Transmition(SD_LK, 6);
        setNowTask(TaskHW, HW_LKSHOW, 0);
        break;
    case 0x05 :   //��ʾĬ��
        //Transmition(SD_MRXX, 6);
        setNowTask(TaskHW, HW_LASTSHOW, 0);
        break;
    }
}


void Wifi_Send_Dispose_zigbee(u8 *Wifi_signal)  //����zigbee��Ϣ //��բ��//���̵��ر���
{
    Send_ZigbeeData_To_Fifo(Wifi_signal, 8);
}


void Wifi_Send_Dispose_hw(u8 *Wifi_signal)  //���������Ϣ
{
    switch(Wifi_signal[3])
    {
		case 0x0A :
			if(Wifi_signal[4] == 0x01)
				 setNowTask(TaskHW, HW_OPENBJQ, 0);//����̨��
			else 
				 setNowTask(TaskHW, HW_CLOSEBJQ, 0);//����̨��
			break;
			
		case 0x0B :  //������ʾ
			hw_Data_Dispose(Wifi_signal);
			break;

		case 0x0C ://��Դ����
			light_control(Wifi_signal[4]);
			break;

		default  :
			return ;
    }
    TaskMenu(); //ִ������
}


void Wifi_Send_Dispose_run(u8 *Wifi_signal)//������������
{
	u16 sum;
    switch((Wifi_signal[3] & 0x0F))	//ȡ����λ
    {
		case 0x01 :
			Dispose_Data_array[0] = (Wifi_signal[4]+CNTONE)-1;
			break;
		case 0x07 :  //ǰ��������ֵ
			if((Wifi_signal[3] & 0xF0) == 0x01) //ѭ��
			{
				sum = (Wifi_signal[4]*256+Wifi_signal[5]);
				setMPVaule(sum);
				Dispose_Data_array[0] = TRACKLENTH;
			}
			else if((Wifi_signal[3] & 0xF0) == 0x00) //��ѭ��
			{
				sum = (Wifi_signal[4]*256+Wifi_signal[5]);
				setMPVaule(sum);
				Dispose_Data_array[0] = GO;
			}
			break;
		case 0x08 :  //����
			sum = (Wifi_signal[4]*256+Wifi_signal[5]);
			setMPVaule(sum);
			Dispose_Data_array[0] = BACK;
			break;
		case 0x09:	//ѭ��ѹ��
			Dispose_Data_array[0]=TRACKBLACK;
			break;
		default  :
			return ;
    }
    InitRunPathContorl(Dispose_Data_array, 1); //�齨����,ִ��
}

void Wifi_Send_Dispose_action(u8 *Wifi_signal)//������
{
	u16 sum;
    switch(Wifi_signal[3])
    {
		case 0x03:	//��ת
			Dispose_Data_array[0] = (Wifi_signal[4]+LEFT45-1);
			break;
		case 0x04:	//��ת
			Dispose_Data_array[0] = (Wifi_signal[4]+RIGHT45-1);
			break;
		case 0x05:	//��ת����
			Dispose_Data_array[0] =LEFTSMALL;
			break;
		case 0x06:	//��ת����
			Dispose_Data_array[0] =RIGHTSMALL;
			break;
		default  :
			return ;
    }
    InitRunPathContorl(Dispose_Data_array, 1); //�齨����,ִ��
    //setNowTask(TaskWifi,CMD_TRAFFIC_READ);
}

void Wifi_Send_Dispose_Car(u8 *Wifi_signal)  //С����ԭ��
{
    switch(Wifi_signal[3])
    {
		case 0x10 : //������
			if(Wifi_signal[4]==0x00)  //��
				Set_tba_Beep(0);
			else if(Wifi_signal[4]==0x01) //��
				Set_tba_Beep(1);
			break;

		case 0x11 : //���
			if(Wifi_signal[4]==0x00)
				Set_tba_WheelLED(L_LED,0);
			else if(Wifi_signal[4]==0x01)
				Set_tba_WheelLED(L_LED,0);
			break;

		case 0x12 : //�ҵ�
			if(Wifi_signal[4]==0x00)
				Set_tba_WheelLED(R_LED,0);
			else if(Wifi_signal[4]==0x01)
				Set_tba_WheelLED(R_LED,1);
			break;

		default :
			break;
    }
}

//�ж��ϲ㷵����Ϣ ��������� �򽫶�Ӧ�ı�־λ����Ϊ 0xff
//void switchBackinfo(u8 vaule)
//{
//    switch (vaule)
//    {
//    case 0x01:
//        wifi_send_QR_flag = 0xff;
//        break;
//    case 0x02:
//        wifi_send_PLATE_flag = 0xff;
//        break;
//    case 0x03:
//        wifi_send_SHAPE_flag = 0xff;
//        break;
////    case 0x04:
////        wifi_send_TRAFFIC_flag = 0xff;
////        break;
//    case 0x05:
//        wifi_send_HLLIGHT_flag = 0xff;
//        break;
//    default:
//        break;
//    }	
//}
void Wifi_Remote_Control () //wifi�źŽ���
{
    u8 readBuf[8];
    u8 i;
    if(Wifi_Rx_flag > 0) //�н��յ�WiFi�ź�
    {
        if((Head->StackData[0] == 0x55 && Head->StackData[7] == 0xbb) 
			|| (Head->StackData[0] == 0xff && Head->StackData[7] == 0xf0) ) //����Э��İ�ͷ
        {
            if(Normal_data(1, Head->StackData)) //��������У�������
			{
				for(i=0;i<8;i++)
					readBuf[i] = Head->StackData[i];
			}
        }		
		Head=Head->Next;
        Wifi_Rx_flag--;
    }

    if(Rx_Flag == 1) //��ȷ��Ϣ��ִ��wifi����
    {
        Rx_Flag = 0;
		Send_ZigbeeData_To_Fifo(readBuf,8); //��ӡ����
//        if(readBuf[0] == 0x55 && readBuf[7] == 0xbb && readBuf[1] != 0xAA) //ZigBee��Ϣֱ��ת��
//        {
//            //Wifi_Send_Dispose_zigbee(Wifi_Rx_Buf);
//        }
        if(readBuf[0] == 0xff && readBuf[7] == 0xf0 && readBuf[1] == 0xAB)//ң��
        {
            switch(readBuf[2])
            {
				case 0x01 :
					Wifi_Send_Dispose_run(readBuf); //��������
					break;
				case 0x02:
					Wifi_Send_Dispose_action(readBuf);//��������
					break;
				case 0x03 ://�Զ�·��
//					wifi_send_QR_flag = 1;
					break;
				case 0x05 :
					Wifi_Send_Dispose_hw(readBuf);		//���պ����ź�
					break;
				case 0x06 :
					Wifi_Send_Dispose_Car(readBuf); //Ԫ������
					break;
//				case 0x09:
//					switchBackinfo(readBuf[3]);
//					break;
				default :
					break ;
            }
        }
		
        else if(readBuf[1] == 0xAA)		//��һ�׿���Э��
		{
			switch(readBuf[2])//����
			{
				case 0x01:	//��������
					Send_InfoData_To_Fifo((u8 *)"RUN\n",sizeof("RUM\n"));
					if(getRunState() == 0)
//					xydInit(&passivity.RFIDCard, getVauleX(readBuf[3]), getVauleY(readBuf[3]), 0);		//��ʼ��RFIDλ��//����ɾ��
//					sprintf(data,"RFID %x\n", readBuf[3]);
//					Send_InfoData_To_Fifo((u8 *)data,strlen(data));
					startRun();
					isrun.TaskCarryOut = readBuf[4];//ѡ��������ʽ
					break;

				case 0x02 : //��ά��ʶ��
					MailboxRe.ConfigInfo.LightLevelTask = (readBuf[3] %4)+1;//����  (N%4)+1
					MailboxRe.ConfigInfo.carport =((readBuf[5] *5)%4)+1;	//����  ((D*5)%4)+1
					wifi_send_QR_flag=1;
					break;
				case 0x03:	//ͼ��ѡ��
					if(((readBuf[3] & 0x0F)==0x01) && (GraphChoose_Flag == (readBuf[3] & 0xF0)) )//�ɹ�
						GraphChoose_Flag=readBuf[3];
					else if((readBuf[3] & 0xF0) == 0x00)	//ʧ��
					{
						GraphChoose_Flag=0;
						Send_ZigbeeData_To_Fifo(TFT_XS,8);	//���Ϸ�ҳ
						delay_ms(2000); //�ȴ�ͼƬ��ҳ
						Send_WifiData_To_Fifo(Pictures_Chose, 8); 	//���Ͷ�������
					}
					break;
				case 0x04://ͼ��ʶ��
					wifi_send_SHAPE_flag = 1;
					MailboxRe.Graph_Sum_Shape[0] = readBuf[3];//���θ���
					MailboxRe.Graph_Sum_Shape[1] = readBuf[4];//Բ�θ���
					MailboxRe.Graph_Sum_Shape[2] = readBuf[5];//�����θ���
					break;
				
				case 0x05 :	//���̵�
					TempArray[3] = readBuf[3];
					wifi_send_HLLIGHT_flag = 1; //���յ���Ϣ				
					break;	
				
				case 0x10://����1
					for(i = 0; i < 3; i++)
					{
						MailboxRe.PlateNumber[i]=readBuf[3+i];
					}
					wifi_rev_card_flag_1 = 1;
					break;
					
				case 0x11://����2
					for(i = 0; i < 3; i++)
						MailboxRe.PlateNumber[i+3]=readBuf[3 + i];
//	                 CP_SHOW2[4] = 'A' + car_x;
//	                 CP_SHOW2[5] = '1' + car_y;
					// HW_Send_Choose(HW_PICUP);  //TFTͼƬ�Ϸ�**************************************
					wifi_rev_card_flag_1 = 2;
					break;
			}
		}
    }
}

#define TIME_OUT    1000     //��ʱms
void Wifi_Send_Dispose(u8 Wifi_signal)
{
    /***************************��ά��***********************************************************/

    if((Wifi_signal == CMD_QR_READ) && (getTaskState() == 0)) //����QR_Read cmd ��ά��ָ��
    {
        startTask();
		wifi_send_QR_flag = 0;
		DelayTimerMS(3000); //�ȴ�����ͷ����
		WaitTimer_ms=gt_get()+3000;
        memset(Dispose_Data_array, 0, sizeof(Dispose_Data_array));
        Send_WifiData_To_Fifo(QRCode, 8); //trackpath[0]---����Ϊ����Ļ�����--���� ���Ͷ�ά��ָ��
        return ;			//��ִ������������ж�
    }

    if(Wifi_signal == CMD_QR_READ ) //pad=>car read qr finishedС��ʶ���ά�����
    {
        if(wifi_send_QR_flag == 0)			//���û�н��յ��ϲ㷢���Ļش���Ϣ
        {
            if(gt_get_sub(WaitTimer_ms) == 0)
            {
                if(WaitTimer_const > 3)	//���ʱ�������ǿ�ƽ�������  
                {
                    wifi_send_QR_flag=1;
                }
                else
                {
                    Send_WifiData_To_Fifo(QRCode, 8); //���·���
                    WaitTimer_ms=gt_get()+3000;//���¼�ʱ
                }
				 ++WaitTimer_const; //�ۼ�
            }
        }
        else if( (wifi_send_QR_flag == 1 || wifi_send_QR_flag == 2))         //�Ѿ����������
        {
            wifi_send_QR_flag = 0; //wifi���ͱ�־��ʼ��
            con_num = 0;
            endTask();//�������
			EndWaitTim();
        }
        return ;
    }
    /*-----------------------------����---------------------------------*/

    if((Wifi_signal == CMD_PLATE_READ) && (getTaskState() == 0)) //���ͳ����ѵ���ָ��
    {
        startTask();//��������
        wifi_send_PLATE_flag = 0; //wifi���ͳ�ʼ��
        DelayTimerMS(3000);
		WaitTimer_ms=gt_get()+5000;
        Send_WifiData_To_Fifo(PlateRead, 8);
        return ;			//��ִ������������ж�
    }
    if(Wifi_signal == CMD_PLATE_READ )		//����ʶ�����
    {
        if(wifi_send_PLATE_flag == 0)
        {
            if(gt_get_sub(WaitTimer_ms) == 0)
            {
                if(WaitTimer_const > 2)	//����������ζ�û���յ���ȡ������ ǿ�ƽ�������
                {
                    wifi_send_PLATE_flag = 1 ;
					wifi_rev_card_flag_1 = 2 ;
                }
                else
                {
                    Send_WifiData_To_Fifo(PlateRead, 8);
                    WaitTimer_ms=gt_get()+5000;
                }
				++WaitTimer_const;
            }
        }
        else  if( wifi_send_PLATE_flag == 1 && wifi_rev_card_flag_1 == 2)      //���������
        {
            wifi_send_PLATE_flag = 0; //wifi���ͱ�־��ʼ��
            wifi_rev_card_flag_1 = 0;
            endTask();
			EndWaitTim();
        }
        return ;
    }

    /*--------------------------ͼ��ʶ��--------------------------------------*/

    if((Wifi_signal == CMD_SHAPE_READ) && (getTaskState() == 0)) //����ͼ��ʶ��
    {
        startTask();//��������
		wifi_send_SHAPE_flag = 0; //wifi���ͳ�ʼ��
        DelayTimerMS(3000);
		WaitTimer_ms=gt_get()+5000;
        Send_WifiData_To_Fifo(ShapeRead, 8);
        return ;
    }
    if(Wifi_signal == CMD_SHAPE_READ ) //pad=>car read qr finishedС��ʶ��ͼƬ����
    {
        if(wifi_send_SHAPE_flag == 0)
        {
            if(gt_get_sub(WaitTimer_ms) == 0)
            {
                if(WaitTimer_const > 2)	//����������ζ�û���յ���ȡ������ ǿ�ƽ�������
                {
                    endTask();
                }
                else
                {
                    Send_WifiData_To_Fifo(PlateRead, 8);
                    WaitTimer_ms=gt_get()+5000;
                }
				++WaitTimer_const;
            }
        }
        else if(wifi_send_SHAPE_flag == 1)
        {
            wifi_send_SHAPE_flag = 0; //wifi���ͱ�־��ʼ��
            endTask();//�������
			EndWaitTim();
        }
        return ;
    }
    /***************************���̵�****************************************/
    if(Wifi_signal == CMD_LIGHT_READ && (getTaskState() == 0)) //���̵�
    {
        startTask();//��������
		Send_InfoData_To_Fifo((u8 *)"0\n",sizeof("0\n"));
        wifi_send_HLLIGHT_flag = 0;
        DelayTimerMS(3000);
        Send_ZigbeeData_To_Fifo(JTD_READ, 8); //���ͺ��̵ƽ���ʶ��ģʽ
		delay_ms(600);
		WaitTimer_ms=gt_get()+5000;
        Send_WifiData_To_Fifo(TrafficLight,8);//���͵����ź�
        return ;
    }
    else if(Wifi_signal == CMD_LIGHT_READ )
    {
        if(wifi_send_HLLIGHT_flag == 0)
        {
            if(gt_get_sub(WaitTimer_ms) == 0)
            {
                if(WaitTimer_const > 4)	//����������ζ�û���յ���ȡ������ ǿ�ƽ�������
                {
                    wifi_send_HLLIGHT_flag=1;
                }
                else
                {
                    Send_WifiData_To_Fifo(TrafficLight, 8);
					WaitTimer_ms=gt_get()+5000;
                }
				++WaitTimer_const;
            }
        }
        else if(wifi_send_HLLIGHT_flag == 1)
        {
            RepeatedlySend_ZG(TempArray, 4,50); //����ʶ����
            wifi_send_HLLIGHT_flag = 0;
            endTask();
			EndWaitTim();
        }
        return ;
    }
//**********************************ͼ��ѡ��*****************************//	
	if(Wifi_signal == PICTURES_CHOOSE && (getTaskState() == 0))	
	{
		startTask();//��������
		DelayTimerMS(3000);
		WaitTimer_ms=gt_get()+5000;
		RevisalProtocol(Pictures_Chose,3,CarRunTask.TaskVaule[CarRunTask.TaskEndPoint]);	//������������
		Send_WifiData_To_Fifo(Pictures_Chose, 8); 
		GraphChoose_Flag=((Pictures_Chose[3]<<4)&0xF0);
	}
	else if(Wifi_signal == PICTURES_CHOOSE )
	{
		if(((GraphChoose_Flag & 0x0F)==0))
		{
			if(gt_get_sub(WaitTimer_ms) == 0)
			{
                if(WaitTimer_const > 10)	//����������ζ�û���յ���ȡ������ ǿ�ƽ�������
                {
                    GraphChoose_Flag=1;
                }
                else
					WaitTimer_ms=gt_get()+5000;	
				++WaitTimer_const;
			}
		}
		else if(GraphChoose_Flag & 0x0F)
		{	
            endTask();
			EndWaitTim();	
			GraphChoose_Flag=0;	
		}
	}
//----------------------------------------Ѱ������--------------------------------------------
    //		if((*(q+taskindex)==CMD_REV_POINT) && (taskflag==0))//����ΪѰ���������
    //		{
    //	      taskflag=1;//��������
    //				wifi_send_GARAGE_flag=0;//wifi���ͳ�ʼ
    //				delay_ms(40);
    //			  send_data_wifi(GetPoint,8);//
    //		}
    //
    //		if(*(q+taskindex)== CMD_REV_POINT&& wifi_send_GARAGE_flag == 1)//
    //		{
    //			 taskflag=2;//�������
    //			 wifi_send_GARAGE_flag =0;//wifi���ͱ�־��ʼ��
    //		}
}




