#ifndef __ZIGBEE_H
#define __ZIGBEE_H
#include "sys.h"

void Zigbee_Rev_Control(void);
void Zig_Send_Dispose(u8 );

void TACKZERO(void); //������ʱ
void SendVoice(u8 *data, int length);  //��������
void send_TFT_data(u8 mode);//ѡ��TFT����
void send_SEG_data(u8 mode);  //ѡ�����������


extern u8 zig_send_CK_flag;//����״̬
//extern u8 Now_Plies; //���⼸��
//extern u8 TriggerF_B;  //ǰ����ⴥ��״̬ =0ȫ����  =1�󴥷� =2ǰ���� =4û����
extern u8 zig_send_door_Rev_flag;
extern u8 zig_send_voice_Rev_flag;
extern u8 zig_send_etc_flag;
//extern u8 zig_send_smg_Rev_flag;

//��ʱģʽ
//�رռ�ʱ��
static u8 countclose[8] = {0x55,0x0b,0x30,0x00,0x00,0x00,0x30,0xbb};
//�򿪼�ʱ��
static u8 countopen[8] = {0x55,0x0b,0x30,0x01,0x00,0x00,0x31,0xbb};
//��ʱ������
static u8 countclean[8] = {0x55,0x0b,0x30,0x02,0x00,0x00,0x32,0xbb};


//�ɵڶ���ָ��ָ����ʾ����ͼƬ
static u8 witch_pic[8] = {0x55,0x0b,0x10,0x00,0x01,0x00,0x11,0xbb};
//ͼƬ�Ϸ�
static u8 pic_up[8] = {0x55,0x0b,0x10,0x01,0x00,0x00,0x11,0xbb};
//ͼƬ�·�
static u8 pic_down[8] = {0x55,0x0b,0x10,0x02,0x00,0x00,0x12,0xbb};
//ͼƬ�Զ����·�ҳ��ʾ�����ʱ��10s
static u8 auto_show[8] = {0x55,0x0b,0x10,0x03,0x00,0x00,0x13,0xbb};
/*-----------------TFT��ʾ��ģ��----------------------*/
#define PLATEMODE	1	//������ʾģʽ
#define	HEXSHOW		2	//ʮ������ʾģ��
#define DISMODE		3	//������ʾģʽ


/*-----------------ͼƬ��ʾ--------------------------*/
#define WITCH_PIC	4		//�ɵڶ���ָ��ָ����ʾ����ͼƬ
#define PIC_UP		5		//ͼƬ�Ϸ�
#define PIC_DOWN	6		//ͼƬ�·�
#define AUTOMATION_PIC	7	//ͼƬ�Զ����·�ҳ��ʾ�����ʱ��10S


/*-----------------��ʱ��ģʽ------------------------*/
#define CLOSE_COUNT	8	//��ʱģʽ�ر�
#define OPEN_COUNT	9	//��ʱģʽ��
#define CLEAR_COUNT	10	//��ʱģʽ����


#define OPEN_SEG  1 //����ܼ�ʱ��
#define CLOSE_SEG 2 //����ܼ�ʱ��
#define SHOW1_SEG 3	//��ʾ��һ��
#define SHOW2_SEG 4	//��ʾ�ڶ���
#define DIS_SEG	  5	//��ʾ����

//������ʾģʽ  ǰ��λ
extern u8 plateA[8];

//������ʾģʽ  ����λ
extern u8 plateB[8];

//��λ��ʾ����(hex��ʽ)
extern u8 sixbits[8];

//������ʾģʽ��ʮ���ƣ�
extern u8 disshow[8];

#endif
