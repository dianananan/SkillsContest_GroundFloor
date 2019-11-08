#include "Init.h"
#include "delay.h"
#include "canp_hostcom.h"
#include "activity.h"
#include "uart_a72.h"
#include "TaskZigbeeChannel.h"
#include "task.h"


u8 Sdata[6];
//*************************kerlen******************************/
void Send_USART_To_Fifo(u8 *array,u8 len) //发送串口信息
{
	UartA72_TxClear();
	UartA72_TxAddStr(array,len);	//发送串口信息
	UartA72_TxStart();	
}

void RFID_Funition(void)	//读卡时要执行的任务函数
{
	u8 i=0;
	int coding;
	u8 Tdata[6],Cursor=0;
	Send_InfoData_To_Fifo(RFID_S50.RXRFID,16);
	delay_ms(100);
	Send_InfoData_To_Fifo((u8*)"\n",2);
	delay_ms(100);
	
	if(RFID_S50.RFID_Mode == READ1)
	{
		MailboxRe.ConfigInfo.carport =(RFID_S50.RXRFID[0]%4)+1;
	}
	else if(RFID_S50.RFID_Mode == READ2)
	{
		getCarPosition(&cardirection, &car_x, &car_y, 1);				//更新坐标
		PrintfDebug(car_x);
		PrintfDebug(car_y);
		RFID_S50.RFID_XYD = setVaule(car_x,car_y,cardirection)	;		//保存卡的位置

		
		//霍夫曼编码
		for(i=0;i<4;i++)	//取值
		{
			coding = coding << 8;
			coding = RFID_S50.RXRFID[i];
		}
		for(i=0;i<32;)	//取码
		{
			if(((coding<<i++) &0x8000) == 1)
			{
				if(((coding<<i++) &0x8000) == 1)
					Tdata[Cursor++] ='B';		
				else		
					Tdata[Cursor++] ='H';
			}
			else 
			{
				if(((coding<<i++) &0x8000) == 1)
				{
					if(((coding<<i++) &0x8000) == 1)
						Tdata[Cursor++] ='T';		
					else		
						Tdata[Cursor++] ='M';				
				}
				else		
					Tdata[Cursor++] ='K';			
			}
			if(Cursor>=6)
				break;
		}
		Sdata[0]=Tdata[0] + Tdata[1];
		Sdata[1]=Tdata[2] - Tdata[3];
		Sdata[2]=(Tdata[4] ^ Tdata[5]);
		Sdata[5]=Tdata[5];
		bubble_sort(Tdata,6,1);	//从小到大排序
		Sdata[3]=Tdata[0];
		Sdata[4]=Tdata[5];		
	}	

}

//******************************math***********************/Upright_Flag

/**********扩展数组内容******************
	array:数组的首地址
	len:数组的最大长度
	adr:添加进数组的位置
	cont:添加进的内容*/
void ExtendArray(u8 *array, u8 Len ,u8 adr,u8 cont)  
{
	u8 i=0;
	if(Len+1>=MAXPATH)return ;
	for(i=Len;i>adr;i--)
	{
		array[i+1]=array[i];
	}
	array[adr+1]=cont;
	
}

void bubble_sort(u8 *arr, u8 len,u8 mode)  //冒泡排序  mode=1从小到大  mode=2从大到小
{
    u8 i, j;
    u16 temp;

	for (i = 0; i < len - 1; i++)
		for (j = i; j < len - 1 - i; j++){
				if((mode ==1 && (arr[j] > arr[j + 1])) ||(mode ==2 && (arr[j] < arr[j + 1]))){
					temp = arr[j];
					arr[j] = arr[j + 1];
					arr[j + 1] = temp;
				}
			}
}
//取最大值/最小值
//求同或/异或

//**********************tool*********************************/
void RepeatedlySend_ZG(u8 *array,u8 sum,u8 delay) //反复发送
{
	u8 i=0;
	for(i=0;i<sum;i++)
	{
		Send_ZigbeeData_To_Fifo(array,8);
		delay_ms(delay);
	}
}

void RepeatedlySend_WI(u8 *array,u8 sum,u8 delay) //反复发送
{
	u8 i=0;
	for(i=0;i<sum;i++)
	{
		Send_WifiData_To_Fifo(array,8);
		delay_ms(delay);
	}
}

void RepeatedlySend_HW(u8 *array,u8 sum,u8 delay) //反复发送
{
	u8 i=0;
	for(i=0;i<sum;i++)
	{
		Infrared_Send(array,6);
		delay_ms(delay);
	}
}


void PrintfDebug(u16 sum)  //debug发送单个数据到Debug面板上来
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

void TaskBoardTest(u8 mode) //打印关照强度的信息到debug面板  mode=0 =1dis
{
	u16 ProData;
	if(mode == 1)	//蜂鸣器和左右灯
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
		if(mode == 2)	//超声波
			ProData=getCsbDis();
		else if(mode ==3)	//光敏
			ProData=Get_Bh_Value();
		PrintfDebug(ProData);
	}
	else if(mode == 4)
	{
		Infrared_Send(HW_K,6);
		delay_ms(100);
		Infrared_Send(H_N[0],4);
	}
}

void DelayTimerMS(u16 time)		//设置高延时函数
{
	if(time==0)return ;
	u16 sum1=0,sum2=0;
	u8 i=0;
	sum1=time/700;
	sum2=time%700;
	for(i=0;i<sum1;i++)
	{
		delay_ms(700);//延时一次最高只要700ms
	}
	delay_ms(sum2);
}

u8 CheckSum(u8 *array,u8 orig)	//计算校验和
{
	u8 i=0;
	u8 result=0;
	for(i=orig;i<6;i++)
		result=(result+array[i])%256;
	return result;
}

void RevisalProtocol(u8 *array,u8 dis,u8 content)	//协议数组，更改的位置，更改的内容
{
	if(dis <1 || dis>6)	//包头包尾校验和，主控位不能做修改
		return;
	array[6] =(array[6]-array[dis])%256;
	array[dis]=content;
	array[6] =(array[6]+array[dis])%256;
}



