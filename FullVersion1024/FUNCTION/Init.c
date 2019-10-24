#include "Init.h"
#include "delay.h"
#include "canp_hostcom.h"
#include "activity.h"
#include "uart_a72.h"

//*************************Timer******************************/


//******************************math***********************/Upright_Flag
void ExtendArray(u8 *array, u8 Len ,u8 adr,u8 cont)  //��չ��������
{
	u8 i=0;
	if(Len+1>=MAXPATH)return ;
	for(i=Len;i>adr;i--)
	{
		array[i+1]=array[i];
	}
	array[adr+1]=cont;
}

void bubble_sort(u16 arr[], u8 len)  //ð������
{
    u8 i, j;
    u16 temp;
    for (i = 0; i < len - 1; i++)
        for (j = i; j < len - 1 - i; j++)
            if (arr[j] > arr[j + 1])
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
}


//**********************tool*********************************/
void RepeatedlySend_ZG(u8 *array,u8 sum,u8 delay) //��������
{
	u8 i=0;
	for(i=0;i<sum;i++)
	{
		Send_ZigbeeData_To_Fifo(array,8);
		delay_ms(delay);
	}
}

void RepeatedlySend_WI(u8 *array,u8 sum,u8 delay) //��������
{
	u8 i=0;
	for(i=0;i<sum;i++)
	{
		Send_ZigbeeData_To_Fifo(array,8);
		delay_ms(delay);
	}
}

void RepeatedlySend_HW(u8 *array,u8 sum,u8 delay) //��������
{
	u8 i=0;
	for(i=0;i<sum;i++)
	{
		Send_ZigbeeData_To_Fifo(array,8);
		delay_ms(delay);
	}
}


void PrintfDebug(u16 sum)  //debug���͵������ݵ�Debug
{
	u8 Array[6],Temp[6];
	u8 len=0;
	u16 temp =0;
	for(temp=sum;temp > 0;temp/=10)
	{
		Temp[len++]=(temp%10)+'0';
	}
	for(temp=len;temp>0;temp--)
	{
		Array[len-temp]=Temp[temp-1];
	}
	Array[len++]='\n';
	Send_InfoData_To_Fifo(Array,len);
	delay_ms(5);
}

void TaskBoardTest(u8 mode) //��ӡ����ǿ�ȵ���Ϣ��debug���  mode=0 =1dis
{
	u16 ProData;
	if(mode == 1)	//�����������ҵ�
	{
		Set_tba_Beep(1);
		Set_tba_WheelLED(L_LED,1);
		Set_tba_WheelLED(R_LED,1);	
		DelayTimerMS(4000);
		Set_tba_Beep(0);
		Set_tba_WheelLED(L_LED,0);
		Set_tba_WheelLED(R_LED,0);		
	}
	else if(mode == 2 || mode == 3)
	{
		if(mode == 2)	//������
			ProData=getCsbDis();
		else if(mode ==3)	//����
			ProData=Get_Bh_Value();
		PrintfDebug(ProData);
	}
	else if(mode == 4)
	{
		Infrared_Send(HW_K,6);
		Infrared_Send(H_N[0],4);
	}
}

void DelayTimerMS(u16 time)		//���ø���ʱ����
{
	u16 sum1=0,sum2=0;
	u8 i=0;
	sum1=time/700;
	sum2=time%700;
	for(i=0;i<sum1;i++)
	{
		delay_ms(700);//��ʱһ�����ֻҪ700ms
	}
	delay_ms(sum2);
}

//void Send_A72Data_To_Fifo(u8 *p ,u8 len)	//���ʹ�����Ϣ
//{
//	UartA72_TxAddStr(JTD_READ,16);
//	UartA72_TxStart();
//}

u8 CheckSum(u8 *array,u8 orig)	//����У���
{
	u8 i=0;
	u8 result=0;
	for(i=orig;i<6;i++)
		result=(result+array[i])%256;
	return result;
	
}