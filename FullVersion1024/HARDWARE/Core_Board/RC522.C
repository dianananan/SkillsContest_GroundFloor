#include "sys.h"
#include "rc522.h"
#include "delay.h"
#include "string.h" 
#include "cba.h"
#include "Timer.h"
#include "canp_hostcom.h"
#include "Init.h"


#define MAXRLEN 18  

#define MF522_RST   PFout(1)
static uint8_t Rc522_LinkFlag;

//#define RC522_LINKTEST_RT(rt)	if(Rc522_LinkFlag == 0) return (rt)	
//#define RC522_LINKTEST_NULL()	if(Rc522_LinkFlag == 0) return	


#define WriteRawRC(addr,datas)	if(WriteRawRC_HDL(addr,datas)!= STATUS_SUCCESS) return MI_ERR


void delay_ns(u32 ns)
{
  static volatile uint32_t i;
  for(i=0;i<ns;i++)
  {
    __nop();
    __nop();
    __nop();
  }
}

//struct RFID_Card RFID_S50;
uint32_t Rs522_cmd_cnt;
uint8_t TXRFID[16] = {0x41,0x31,0x42,0x32,0x43,0x33,0x44,0x34,0x46,0x31,0x42,0x32,0x43,0x33,0x44,0x34};


/*
函数功能：全自动读卡函数
参    数：Chunk  块地址
返 回 值：无
**/
void Read_Card(u8 Chunk)
{
	char status = MI_ERR;
	u8 i=100;
	uint8_t CT[2]={0x00,0x00};						//卡类型
	uint8_t SN[4]={0x00,0x00,0x00,0x00};			//卡号
	uint8_t KEY[6]={0xff,0xff,0xff,0xff,0xff,0xff}; //密钥

//	uint8_t RXRFIDH[8];
	#define  DATA_LEN    16                     	//定义数据字节长度	


	status = PcdRequest(PICC_REQALL,CT);//**********************************//寻卡，并填充型号
	if(status == MI_OK)	//寻卡成功
	{		
		status=MI_ERR;
		TIM_Cmd(TIM9,DISABLE);
		Send_UpMotor(0 , 0);
		status = PcdAnticoll(SN);//**********************************//防冲撞
		if(status == MI_OK)
		{
			PrintfDebug(1);
			delay_ms(200);
			status=MI_ERR;			
			status =PcdSelect(SN);				//选定此卡
			if(status == MI_OK)					//选定成功
			{
				status=MI_ERR;		
				status =PcdAuthState(0x60,0x03,KEY,SN);//**********************************//验证密钥
				if(status == MI_OK)
				{
//					Set_tba_Beep(1);			//蜂鸣器	
					status = MI_ERR;
					if(RFID_S50.RFID_Mode == READ1 || RFID_S50.RFID_Mode==READ1)
					{
						status=PcdRead(Chunk,RFID_S50.RXRFID);//**********************************//读卡 
						if(status == MI_OK)
						{
							PrintfDebug(5);
							RFID_S50.RFID_Read_Ok=1;	//读卡成功
							status = MI_ERR;					
							RFID_Funition();
						}								
					}	
					else if(RFID_S50.RFID_Mode == WRITE)
					{
						status = PcdWrite(Chunk,TXRFID);//**********************************//写卡
						if(status == MI_OK)
						{
							status = MI_ERR;
							RFID_S50.RFID_Write_Ok=1;
						}							
					}
					delay_ms(200);
//					Set_tba_Beep(0);			//蜂鸣器	
				}
			}
		}
		//			Send_UpMotor(L_Speed ,R_Speed);				
		TIM_Cmd(TIM9,ENABLE);	//重新启动
	}
}


//初始化IO 串口1 
//bound:波特率
void RC522_Uart_init(u32 baudrate)
{
	GPIO_InitTypeDef  GPIO_TypeDefStructure;
	USART_InitTypeDef USART_TypeDefStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	//PA9-Tx
	GPIO_TypeDefStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_TypeDefStructure.GPIO_Mode = GPIO_Mode_AF;		//复用功能
	GPIO_TypeDefStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_TypeDefStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
	GPIO_TypeDefStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_TypeDefStructure);
	
	USART_TypeDefStructure.USART_BaudRate = baudrate;					       //波特率
	USART_TypeDefStructure.USART_HardwareFlowControl = 				       //无硬件控制流
												 USART_HardwareFlowControl_None;  
	USART_TypeDefStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx; //接收与发送模式
	USART_TypeDefStructure.USART_Parity = USART_Parity_No; 		       //无校验位
	USART_TypeDefStructure.USART_StopBits = USART_StopBits_1;        //停止位1
	USART_TypeDefStructure.USART_WordLength = USART_WordLength_8b;   //数据位8位
	USART_Init(USART1,&USART_TypeDefStructure);

	USART_Cmd(USART1, ENABLE);                    //使能串口 

	Rc522_LinkFlag = 0;
}

char InitRc522(void)
{
	  if(PcdReset() != MI_OK)
		  return MI_ERR;
	  PcdAntennaOff();
	  delay_ms(2);  
	  PcdAntennaOn();
	  if(M500PcdConfigISOType( 'A' ) != MI_OK)
		  return MI_ERR;
	  Rc522_LinkFlag = 1;
	  Rc522_ToSRst();
	  return MI_OK;
}

void Readcard_daivce_Init(void)//RFID初始化
{
	RC522_Uart_init(9600);	// 串口初始化为9600
	delay_ms(500);
	InitRc522();			//读卡器初始化
}

void Reset_RC522(void)
{
  PcdReset();
  PcdAntennaOff();
  delay_ms(2);  
  PcdAntennaOn();
}   
/////////////////////////////////////////////////////////////////////
//功    能：寻卡
//参数说明: req_code[IN]:寻卡方式
//                0x52 = 寻感应区内所有符合14443A标准的卡
//                0x26 = 寻未进入休眠状态的卡
//			pTagType[OUT]：卡片类型代码
//                0x4400 = Mifare_UltraLight
//                0x0400 = Mifare_One(S50)
//                0x0200 = Mifare_One(S70)
//                0x0800 = Mifare_Pro(X)
//                0x4403 = Mifare_DESFire
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdRequest(unsigned char req_code,unsigned char *pTagType)
{
   char status;  
   unsigned int  unLen;
   unsigned char ucComMF522Buf[MAXRLEN]; 
//  unsigned char xTest ;
   ClearBitMask(Status2Reg,0x08);
   WriteRawRC(BitFramingReg,0x07);

//  xTest = ReadRawRC(BitFramingReg);
//  if(xTest == 0x07 )
 //   { LED_GREEN  =0 ;}
 // else {LED_GREEN =1 ;while(1){}}
   SetBitMask(TxControlReg,0x03);
 
   ucComMF522Buf[0] = req_code;

   status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen); //读取并发送
//     if(status  == MI_OK )
//   { LED_GREEN  =0 ;}
//   else {LED_GREEN =1 ;}
   if ((status == MI_OK) && (unLen == 0x10))
   {    
       *pTagType     = ucComMF522Buf[0];
       *(pTagType+1) = ucComMF522Buf[1];
   }
   else
   {   status = MI_ERR;   }
   return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：防冲撞
//参数说明: pSnr[OUT]:卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////  
char PcdAnticoll(unsigned char *pSnr)
{
    char status;
    unsigned char i,snr_check=0;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    

    ClearBitMask(Status2Reg,0x08);
    WriteRawRC(BitFramingReg,0x00);
    ClearBitMask(CollReg,0x80);
 
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

    if (status == MI_OK)
    {
    	 for (i=0; i<4; i++)
         {   
             *(pSnr+i)  = ucComMF522Buf[i];
             snr_check ^= ucComMF522Buf[i];
         }
         if (snr_check != ucComMF522Buf[i])
         {   status = MI_ERR;    }
    }
    
    SetBitMask(CollReg,0x80);
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：选定卡片
//参数说明: pSnr[IN]:卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdSelect(unsigned char *pSnr)
{
    char status;
    unsigned char i;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
    	ucComMF522Buf[i+2] = *(pSnr+i);
    	ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
    ClearBitMask(Status2Reg,0x08);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
    
    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }

    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：验证卡片密码
//参数说明: auth_mode[IN]: 密码验证模式
//                 0x60 = 验证A密钥
//                 0x61 = 验证B密钥 
//          addr[IN]：块地址
//          pKey[IN]：密码
//          pSnr[IN]：卡片序列号，4字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////               
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
 //   memcpy(&ucComMF522Buf[2], pKey, 6); 
 //   memcpy(&ucComMF522Buf[8], pSnr, 4); 
    
    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：读取M1卡一块数据
//参数说明: addr[IN]：块地址
//          pData[OUT]：读出的数据，16字节
//返    回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////// 
char PcdRead(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
   
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if ((status == MI_OK) && (unLen == 0x90))
 //   {   memcpy(pData, ucComMF522Buf, 16);   }
    {
        for (i=0; i<16; i++)
        {    *(pData+i) = ucComMF522Buf[i];   }
    }
    else
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：写数据到M1卡一块
//参数说明: addr[IN]：块地址
//          pData[IN]：写入的数据，16字节
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////                  
char PcdWrite(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        //memcpy(ucComMF522Buf, pData, 16);
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pData+i);   }
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    
    return status;
}



/////////////////////////////////////////////////////////////////////
//功    能：命令卡片进入休眠状态
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdHalt(void)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    return status;//MI_OK;
}

/////////////////////////////////////////////////////////////////////
//用MF522计算CRC16函数
/////////////////////////////////////////////////////////////////////
char CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
    ClearBitMask(DivIrqReg,0x04);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   WriteRawRC(FIFODataReg, *(pIndata+i));   }
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do 
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = ReadRawRC(CRCResultRegL);
    pOutData[1] = ReadRawRC(CRCResultRegM);
	return MI_OK;
}

/////////////////////////////////////////////////////////////////////
//功    能：复位RC522
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdReset(void)
{
	/*
    MF522_RST=1;
    delay_ns(10);
    MF522_RST=0;
    delay_ns(10);
    MF522_RST=1;
    delay_ns(10);
	*/
	if(Rc522_OutSRst()!=MI_OK)
		return MI_ERR;
	
    WriteRawRC(CommandReg,PCD_RESETPHASE);
	  delay_ns(10);

    WriteRawRC(ModeReg,0x3D);            //和Mifare卡通讯，CRC初始值0x6363
    WriteRawRC(TReloadRegL,30);           
    WriteRawRC(TReloadRegH,0);
    WriteRawRC(TModeReg,0x8D);
    WriteRawRC(TPrescalerReg,0x3E);
    WriteRawRC(TxAutoReg,0x40); 
	  
    return MI_OK;
	
//	return MI_ERR;
}
//////////////////////////////////////////////////////////////////////
//设置RC522的工作方式 
//////////////////////////////////////////////////////////////////////
char M500PcdConfigISOType(unsigned char type)
{
   if (type == 'A')                     //ISO14443_A
   { 
		   
       ClearBitMask(Status2Reg,0x08);
       WriteRawRC(ModeReg,0x3D);//3F
       WriteRawRC(RxSelReg,0x86);//84
       WriteRawRC(RFCfgReg,0x7F);   //4F
   	   WriteRawRC(TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
			 WriteRawRC(TReloadRegH,0);
       WriteRawRC(TModeReg,0x8D);
	     WriteRawRC(TPrescalerReg,0x3E);
			 delay_ms(10);
       PcdAntennaOn();
//       WriteRawRC(SerialSpeedReg,0x7A);//
		 
		   //WriteRawRC(CommandReg,0x00);//
		   
   }
   else{ return -1; }
   
   return MI_OK;
}
/*************************************************
Function:       Send_data
Description:
     write a byte to serial port
Parameter:
     ch            the byte to write
Return:
     None
**************************************************/
void Send_data(unsigned char ch)
{	
	  USART1->SR;
	while (!(USART1->SR & USART_FLAG_TXE));
    USART_SendData(USART1,ch); 
		//while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
	
}

/*************************************************
Function:       Rece_data
Description:
     get a byte from serial port
Parameter:
     ch            the byte to save the received byte
     WaitTime      the counter for polling, avoid endless loop.
Return:
     STATUS_SUCCESS         get a byte data successfully
     STATUS_IO_TIMEOUT      time out
**************************************************/
short Rece_data(unsigned char *ch, unsigned int WaitTime)
{
	/*
    unsigned int j;
    j=0;
  while(!(USART1->SR & USART_FLAG_RXNE))
	{
		j++;
		if(j>=WaitTime)          //avoid endless loop
			break;
	}
	if(j<WaitTime)
	{
		*ch = USART_ReceiveData(USART1);
		//RI=0;
		return STATUS_SUCCESS;
	}
	else
	    return STATUS_IO_TIMEOUT;
	*/
	
//	while(!(UART4->SR & USART_FLAG_RXNE));
//	*ch = (uint8_t)USART_ReceiveData(UART4);
//	return STATUS_SUCCESS;
	
//	while(!(USART3->SR & USART_FLAG_RXNE));
//	*ch = (uint8_t)USART_ReceiveData(USART3);
//	return STATUS_SUCCESS;

	uint32_t tt;
	tt = gt_get() + WaitTime/2000;
	
	while(gt_get_sub(tt))
	{
		if(USART1->SR & USART_FLAG_RXNE)
		{
			*ch = (uint8_t)USART_ReceiveData(USART1);
			return STATUS_SUCCESS;			
		}
	}
	Rc522_LinkFlag = 0;
	return STATUS_IO_TIMEOUT;
	
}
/////////////////////////////////////////////////////////////////////
//功    能：读RC632寄存器
//参数说明：Address[IN]:寄存器地址
//返    回：读出的值
/////////////////////////////////////////////////////////////////////
unsigned char ReadRawRC(unsigned char Address)
{
    unsigned char RegVal;
    short status;
    Address = (Address & 0x3f) | 0x80;   //code the first byte
    Send_data(Address);
    status = Rece_data(&RegVal, 10000);
    if(status != STATUS_SUCCESS)
        return 0xff;
    return RegVal;
}

/////////////////////////////////////////////////////////////////////
//功    能：写RC632寄存器
//参数说明：Address[IN]:寄存器地址
//          value[IN]:写入的值
/////////////////////////////////////////////////////////////////////
//short WriteRawRC(unsigned char Address, unsigned char value)
//{  
//    unsigned char EchoByte;
//    short status;
//    Address &= 0x3f;   //code the first byte
//    Send_data(Address);
//    Send_data(value);
//		
//    status = Rece_data(&EchoByte, 10000);
//	return status;
//}

short WriteRawRC_HDL2(unsigned char Address, unsigned char value)
{  
    unsigned char EchoByte;
    short status;
    Address &= 0x3f;   //code the first byte
    Send_data(Address);
	Send_data(value);
    status = Rece_data(&EchoByte, 10000);	
	//delay_ns(10);
	if(status == STATUS_SUCCESS)
	{
		if(Address == EchoByte)
			Rs522_cmd_cnt++;
		else status = STARUS_ADDR_RERR;
		//Send_data(value);
	}
//	delay_ms(2);
	return status;
}

short WriteRawRC_HDL(unsigned char Address, unsigned char value)
{  
    unsigned char EchoByte;
    short status;
	uint8_t e = 3;
    Address &= 0x3f;   //code the first byte
	for(e = 0;e<3;e++)
	{
		Send_data(Address);
		status = Rece_data(&EchoByte, 10000);	
		//delay_ns(10);
		if(status == STATUS_SUCCESS)
		{
			if(Address == EchoByte)
			{
				Rs522_cmd_cnt++;
				Send_data(value);
				break;
			}
			else
			{
				//Rc522_OutSRst();
				status = STARUS_ADDR_RERR;
			}
			
		}
	}
	
//	delay_ms(2);
	return status;
}

/////////////////////////////////////////////////////////////////////
//功    能：置RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:置位值
/////////////////////////////////////////////////////////////////////
char SetBitMask(unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg,tmp | mask);  // set bit mask
	return MI_OK;
}

/////////////////////////////////////////////////////////////////////
//功    能：清RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//          mask[IN]:清位值
/////////////////////////////////////////////////////////////////////
char ClearBitMask(unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask);  // clear bit mask
	return MI_OK;
} 

/////////////////////////////////////////////////////////////////////
//功    能：通过RC522和ISO14443卡通讯
//参数说明：Command[IN]:RC522命令字
//          pInData[IN]:通过RC522发送到卡片的数据
//          InLenByte[IN]:发送数据的字节长度
//          pOutData[OUT]:接收到的卡片返回数据
//          *pOutLenBit[OUT]:返回数据的位长度
/////////////////////////////////////////////////////////////////////
char PcdComMF522(unsigned char Command, 
                 unsigned char *pInData, 
                 unsigned char InLenByte,
                 unsigned char *pOutData, 
                 unsigned int  *pOutLenBit)
{
    char status = MI_ERR;
    unsigned char irqEn   = 0x00;
    unsigned char waitFor = 0x00;
    unsigned char lastBits;
    unsigned char n;
    unsigned int i;
    switch (Command)
    {
       case PCD_AUTHENT:
          irqEn   = 0x12;
          waitFor = 0x10;
          break;
       case PCD_TRANSCEIVE:
          irqEn   = 0x77;
          waitFor = 0x30;
          break;
       default:
         break;
    }
   
    WriteRawRC(ComIEnReg,irqEn|0x80);
    ClearBitMask(ComIrqReg,0x80);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    
    for (i=0; i<InLenByte; i++)
    {   WriteRawRC(FIFODataReg, pInData[i]);    }
    WriteRawRC(CommandReg, Command);
   
    
    if (Command == PCD_TRANSCEIVE)
    {    SetBitMask(BitFramingReg,0x80);  }
    
    i = 600;//根据时钟频率调整，操作M1卡最大等待时间25ms
 //i = 2000;
    do 
    {
         n = ReadRawRC(ComIrqReg);
         i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitFor));
    ClearBitMask(BitFramingReg,0x80);
	      
    if (i!=0)
    {    
         if(!(ReadRawRC(ErrorReg)&0x1B))
         {
             status = MI_OK;
             if (n & irqEn & 0x01)
             {   status = MI_NOTAGERR;   }
             if (Command == PCD_TRANSCEIVE)
             {
               	n = ReadRawRC(FIFOLevelReg);
              	lastBits = ReadRawRC(ControlReg) & 0x07;
                if (lastBits)
                {   *pOutLenBit = (n-1)*8 + lastBits;   }
                else
                {   *pOutLenBit = n*8;   }
                if (n == 0)
                {   n = 1;    }
                if (n > MAXRLEN)
                {   n = MAXRLEN;   }
                for (i=0; i<n; i++)
                {   pOutData[i] = ReadRawRC(FIFODataReg);    }
            }
         }
         else
         {   status = MI_ERR;   }
        
   }
   

   SetBitMask(ControlReg,0x80);           // stop timer now
   WriteRawRC(CommandReg,PCD_IDLE); 
   return status;
}


/////////////////////////////////////////////////////////////////////
//开启天线  
//每次启动或关闭天险发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
void PcdAntennaOn()
{
    unsigned char i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}


/////////////////////////////////////////////////////////////////////
//关闭天线
/////////////////////////////////////////////////////////////////////
void PcdAntennaOff()
{
    ClearBitMask(TxControlReg, 0x03);
}

//等待卡离开
void WaitCardOff(void)
{
	unsigned char status, TagType[2];

	while(1)
	{
		status = PcdRequest(REQ_ALL, TagType);
		if(status)
		{
			status = PcdRequest(REQ_ALL, TagType);
			if(status)
			{
				status = PcdRequest(REQ_ALL, TagType);
				if(status)
				{
					return;
				}
			}
		}
		delay_ms(10);
	}
}






//_______________________________________________________________

uint8_t Rc522_GetLinkFlag(void)
{
	return Rc522_LinkFlag;
}

void Rc522_LinkTest(void)
{
	ReadRawRC(TxControlReg);
}

int8_t Rc522_OutSRst(void)
{
	uint16_t i;
	uint8_t Rd;
	uint8_t Rt = MI_ERR;
	for(i = 0;i<100;i++)
	{
		Send_data(0x55);		
		if(Rece_data(&Rd, 10000) == STATUS_SUCCESS)
		//if(Rece_data(&Rd, 1000) == STATUS_SUCCESS)
		{
			//if (Rd == 0)
				//Rc522_LinkTest();
				Send_data(0x55);
				delay_ms(100);
				Rt = MI_OK;
				break;
		}
	}
	return Rt;
}

int8_t Rc522_ToSRst(void)
{
	//WriteRawRC(CommandReg,PCD_IDLE|0x10);
	return 0;
}

//extern void Send_InfoData_To_Fifo( uint8_t *p ,uint8_t len);   
//extern void Full_Diplay(uint32_t vluae);

void Rc522_DispCnt(void)
{
	//Send_InfoData_To_Fifo((uint8_t *)"Rc522ccnt=",10);
	//Full_Diplay(Rs522_cmd_cnt);
}

//_______________________________________________________________
