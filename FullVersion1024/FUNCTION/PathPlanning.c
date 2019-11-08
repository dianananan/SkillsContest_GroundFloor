#include "PathPlanning.h"
#include "roadway_check.h"
#include "Activity.h"
#include "data_base.h"
#include "canp_hostcom.h"
#include "Init.h"

#define MAXPOT 30

const u8 inf = 0x3f;
void *memset(void *, int, unsigned int);

int min;
u8 TempFind[MAXPOT];
u8 Find[MAXPOT];
u8 car_x=0, car_y=0,cardirection=0;		//С���ĵ�ǰ����
u8 ForwardMPFlag = 1;					//�ж��ڿ�ʼ��ʱ���Ƿ�Ҫ����ǰ��һС�ξ���
u8 CrossingCount = 0; //·�ڸ���
u8 tasklinecount = 0;
u8 NowTaskPot;		//��ǰ�������

u8 Carx, Cary, Cardirection;	//С���е�����귽��
struct passivityGather passivity;  //�ϰ�����ֵ�λ��

u8 trackpath[MAXPATH];
xyd LimitPot[MAXLIMIT];

//���ڵ�ͼ���ƣ���ʼ����Ϊ1.5
u8 map[7][7] =   //��ͼ
{
    0, 1, 0, 1, 0, 1, 0,
    1, 1, 1, 1, 1, 1, 1,
    0, 1, 0, 1, 0, 1, 0,
    1, 1, 1, 1, 1, 1, 1,
    0, 1, 0, 1, 0, 1, 0,
    1, 1, 1, 1, 1, 1, 1,
    0, 1, 0, 1, 0, 1, 0,
};
short int div[4][2] = {{ -1, 0}, {0, 1}, {1, 0}, {0, -1}}; //�ң��ϣ�����


struct LineNode TaskControl;

void initStartCoord(u8 x, u8 y, u8 dir)
{
    car_x = Carx = x;
    car_y =  Cary = y;
    cardirection =  Cardirection = dir;
}

void xydInit(xyd *data, u8 x, u8 y, u8 dir)
{
    data->data = setVaule(x, y, dir);
}

void setMap(u8 x, u8 y, u8 s)//�����ϰ���
{
    map[x][y] = s;
}

void setLimitPot(u8 sxy, u8 exy)
{
    LimitPot[0].data = sxy;
    LimitPot[1].data = exy;
}

void setTaskLimitPot()  //����·������
{
    u8 i = 0;
    for(; i < MAXLIMIT; i++)
    {
        if(getVauleX(LimitPot[i].data) == car_x && getVauleY(LimitPot[i].data) == car_y)
            continue;
        TaskControl.sexy[i] = LimitPot[i];
        LimitPot[i].data = 0;
    }
}

void setTaskLimitGather(u8 *arr, u8  len)  //����һ��·����Ϊ�ؾ���
{
    u8 i;

    if(len >= MAXLIMIT)
    {
        return ;
    }
    for(i = 0; i < len; i++)
    {
        LimitPot[i].data = arr[i];
    }
}

u8 initTask(u8 target, u8 dir, u8 LastOption, u16 mpvaule, u8 runback)
{
	u8 i=0;
    Carx = car_x;
    Cary = car_y;

    Cardirection = cardirection;
    TaskControl.target.data = target; //��¼x ��y�� dis
    memset(TaskControl.sexy, 0, sizeof(TaskControl.sexy));

    setTaskLimitPot();

    TaskControl.TaskDir = dir;
    TaskControl.LastLineOption = LastOption;
//    if(LastOption != 0)
//    {
        setMPVaule(mpvaule); //��ֵ����ʱ����ֵ temp
//    }
    TaskControl.isback = runback;   //�Ƿ����

    for(i = 0; i < MAXPATH; i++)			//���·������
        isRunControl.PathGather[i] =0;
	
    tasklinecount = 0;
    min = inf;  //inf=x03f

    map[Carx][Cary] = 0;
    dfs(setVaule(Carx, Cary, Cardirection), 1);
    map[Carx][Cary] = 1;

    if(min != inf)		//�ɹ�����·��
    {
        if(min == 1)		//·�߹滮Ϊ��ǰλ��
        {
			lastLine();
            InitRunPathContorl(trackpath, tasklinecount);
			Send_WifiData_To_Fifo(trackpath,tasklinecount);
//			ForwardMPFlag = 0;			
        }
        else
        {
            Find[0] = setVaule(Carx, Cary, Cardirection);
            CrossingCount = 0;
            tasklinecount = 0;
            addCarLine();
            lastLine();
            //send_data_wifi(Find,min);
            InitRunPathContorl(trackpath, tasklinecount);
            Send_WifiData_To_Fifo(trackpath,tasklinecount);	//��ӡ·��
        }
        return 1;
    }
	else
	{
		Send_InfoData_To_Fifo((u8 *)"path planning error",sizeof("path planning error"));
	}
}

u8 judge[MAXLIMIT] = {0};
void dfs(u8 runxy, int n)
{
    //    u8 data[10];
    u8 i, cntlimit, j;
    if(n >= min)return ;//��֦
    if(getxy(runxy) == getxy(TaskControl.target.data)) //��ǰλ�þ����յ�λ��
    {
        memset(judge, 0, sizeof(judge));  //
        cntlimit = 0;
        for(i = 0; i < n; i++) //1
        {
            for(j = 0; j < MAXLIMIT; j++) //2
            {
                if(judge[j] == 1)continue;
                if((TaskControl.sexy[j].data == 0 ) || (getxy(TaskControl.sexy[j].data) == getxy(TempFind[i]) )) //�жϱؾ���
                {
                    //printf("test = %d %d\n",TaskControl.sexy[j].data,getxy(TempFind[i]));
                    ++cntlimit;
                    judge[j] = 1;
                }
            }
        }
        if(cntlimit == MAXLIMIT)
        {

            for(i = 0; i < n; i++)
            {
                Find[i] = TempFind[i];
            }
            min = n;
            //						sprintf(data,"min = %d\n",min);
            //						Send_Debug_Info(data,strlen(data));
            //						send_data_wifi(Find,min);
            return ;
        }

    }
    else
    {
        for(i = 0; i < 4; i++)
        {
            int tx = getVauleX(runxy) + div[i][0];
            int ty = getVauleY(runxy) + div[i][1];

            if(tx < 0 || ty < 0)continue;
            if(tx > 6 || ty > 6)continue;
            if(map[tx][ty] == 0)continue;
            if((tx == getVauleX(passivity.TerrainPot.data)) && (ty == getVauleY(passivity.TerrainPot.data)))
            {
                //�������α�־��
                if(!((getDirection(passivity.TerrainPot.data) & 1) == (i & 1)))
                {
                    continue;//������
                }
            }
            else   if((tx == getVauleX(passivity.TrafficPot.data)) && (ty == getVauleY(passivity.TrafficPot.data)))
            {
                //������ͨ�� ֻ���������߹�
                if(i != getDirection(passivity.TrafficPot.data))
                    continue;
            }
            else if ((tx == getVauleX(passivity.ETCPot.data)) && (ty == getVauleY(passivity.ETCPot.data)))
            {
                //����ETCֻ�ܴ������߹�
                if(i != getDirection(passivity.ETCPot.data))
                    continue;
            }
            else if ((tx == getVauleX(passivity.BarrierszPot.data)) && (ty == getVauleY(passivity.BarrierszPot.data)))
            {
                //������բ���Դ����߹�
                //               if(i != getDirection(passivity.BarrierszPot.data))continue;
                if(((getDirection(passivity.BarrierszPot.data) & 1) != (i & 1)))continue;
            }
            map[tx][ty] = 0;
            //	printf("n = %d %d %d %d %d %d\n",tx,ty,getVauleX(runxy),getVauleY(runxy),div[i][0],div[i][1]);
            //	sprintf(data,"n = %d %d %d\n",tx,ty,i);

            TempFind[n] = setVaule(tx, ty, i);
            dfs(TempFind[n], n + 1);
            map[tx][ty] = 1;           //����
        }
    }
    return ;
}

void walkIntersection()
{
    if(CrossingCount == 1)
    {
        trackpath[(tasklinecount)++] = CNTONE;
    }
    else if(CrossingCount == 2)
    {
        trackpath[(tasklinecount)++] = CNTTWO;
    }
    else if(CrossingCount == 3)
    {
        trackpath[(tasklinecount)++] = CNTTHREE;
    }
    else if(CrossingCount == 4)
    {
        trackpath[(tasklinecount)++] = CNTFOUR;
    }
    else if(CrossingCount == 5)
    {
        trackpath[(tasklinecount)++] = CNTONE;
    }
    CrossingCount = 0;
}

u8 JudgeIntersection(int i )   //�ж�������ǲ���·��
{
    if(getxy(passivity.TerrainPot.data) == getxy(Find[i])) //�������α�־��
    {
        walkIntersection();//���߹���·�ڲ��뵽��̬·��
        trackpath[(tasklinecount)++] = GO_TERRAIN; //���������������
    }
    else if(getxy(passivity.TrafficPot.data) == getxy(Find[i]))
    {
        //������ͨ��
        walkIntersection();//���߹���·�ڲ��뵽��̬·��
        trackpath[(tasklinecount)++] = NOW_TRAFFIC; //���������������
    }
    else if (getxy(passivity.ETCPot.data) == getxy(Find[i]))
    {
        //����ETC
        walkIntersection();//���߹���·�ڲ��뵽��̬·��
        trackpath[(tasklinecount)++] = NOW_ETC; //���������������
    }
    else if (getxy(passivity.BarrierszPot.data) == getxy(Find[i]))
    {
        //������բ
        walkIntersection();//���߹���·�ڲ��뵽��̬·��
        trackpath[(tasklinecount)++] = NOW_BARRIERSZ; //���������������
    }
    else if( getxy(passivity.RFIDCard.data) == getxy(Find[i]))
    {
        walkIntersection();//���߹���·�ڲ��뵽��̬·��
        //������һ����ĳ�ͷ����� ����ѡ��Ҫ�ߵ�mpֵ
        if(i != 0)
        {
            if((getVauleX(Find[i]) & 1) && (getVauleY(Find[i - 1]) & 1)) //����������·����
            {
                if(getDirection(Find[i - 1]) & 1)          //�����������ϵ�λ�õĻ���������������
                    trackpath[(tasklinecount)++] = TRACK_LINE_SHORT; //����Ϊ 1 3 �߶̱�
                else
                    trackpath[(tasklinecount)++] = TRACK_LINE_LANG;//����Ϊ 0 2 �߳���
            }
        }

    } else  if(((getVauleY(Find[i]) + 1 ) % 2 == 0 && (getVauleX(Find[i]) + 1) % 2 == 0) //����������һ��·��
               || ((getVauleY(Find[i]) + 1) % 2 == 0 && (getVauleX(Find[i]) == 0)) //�߽�һ����·�����⴦��
               || ((getVauleY(Find[i]) == 6||(getVauleY(Find[i]) == 0)) && (getVauleX(Find[i]) + 1) % 2 == 0))
    {

        (CrossingCount)++;
        return 1;
    }
    return 0;
}

void lastLine()
{
	taskDirectionDeal();	//����ͬ��ת��
    if(TaskControl.TaskDir != 0)
    {
		if(TaskControl.TaskDir == LEFT45)
		{
			if( ForwardMPFlag == 1)	//���һ��ķ���ǰ��Ӱ����һ��·�߹滮��ʼ
			{
				ForwardMPFlag = 0;
				trackpath[tasklinecount++] = LEFTSMALL;
			}
			trackpath[tasklinecount++] = LEFT45;   
		}
		else if(TaskControl.TaskDir == RIGHT45)
		{
			if( ForwardMPFlag == 1)
			{
				ForwardMPFlag = 0;
				trackpath[tasklinecount++] = RIGHTSMALL;  
			}
			trackpath[tasklinecount++] = RIGHT45;
			ForwardMPFlag = 0;
		}
    }

    if(TaskControl.LastLineOption == BACK || TaskControl.LastLineOption == GO || TaskControl.LastLineOption == TRACKLENTH)
    {
        trackpath[tasklinecount++] = TaskControl.LastLineOption;
    }

    trackpath[tasklinecount++] = NOW_TASK;
    //if(TaskControl.LastLineOption != 0)
    //{
    //   trackpath[++tasklinecount] = TaskControl.LastLineOption;
    //}
    if(TaskControl.isback == 1) //�������������ڹ���
    {
        if(TaskControl.LastLineOption != 0)
        {
            if(TaskControl.LastLineOption == BACK)
                trackpath[tasklinecount++] = GO;
            else if(TaskControl.LastLineOption == GO || TaskControl.LastLineOption == TRACKLENTH)
                trackpath[tasklinecount++] = BACK;
        }
        if(TaskControl.TaskDir == RIGHT45)
        {
            trackpath[tasklinecount++] = LEFT45;
			
        }
        else if(TaskControl.TaskDir ==  LEFT45)
        {
            trackpath[tasklinecount++] = RIGHT45;
        }
    }
    else		//��������
    {
        if(TaskControl.TaskDir == RIGHT45)
        {
            trackpath[tasklinecount++] = RIGHT45;
        }
        else if(TaskControl.TaskDir ==  LEFT45)
        {
            trackpath[tasklinecount++] = LEFT45;
        }
    }
}

u8 taskDirectionDeal() //�ø����һ��·�ڵ��ж�
{
    if(getDirection(TaskControl.target.dir) != Cardirection)  //����ͬ����ת��
    {
        if(getDirection(TaskControl.target.dir) + 1 == Cardirection || Cardirection == getDirection(TaskControl.target.dir) - 3)
        {
            if(ForwardMPFlag==1){
				ForwardMPFlag=0;
				trackpath[tasklinecount++] = RIGHTSMALL;
			}
            trackpath[tasklinecount++] = RIGHT;
        }
        else if(getDirection(TaskControl.target.dir) - 1 == Cardirection || Cardirection == getDirection(TaskControl.target.dir) + 3)
        {
            if(ForwardMPFlag==1)
			{
				ForwardMPFlag=0;
				trackpath[tasklinecount++] = LEFTSMALL;
			}
            trackpath[tasklinecount++] = LEFT;

        }
        else
        {
            if(ForwardMPFlag==1)
			{
				ForwardMPFlag=0;
				trackpath[tasklinecount++] = LEFTSMALL;
			}
            trackpath[tasklinecount++] = LEFT;
            trackpath[tasklinecount++] = LEFT;
        }
		
        return 1;
    }
    else
        return 0;
    
}
void addCarLine()
{
    u8 i;	
    for(i = 0; i < min - 1; i++)
    {
        if(getDirection(Find[i + 1]) != Cardirection) //�˵�ķ���ͬ
        {
            if(i != 0)	//��ǰ����·�ڲ�����
                JudgeIntersection(i);//�жϵ�ǰ���ǲ���·��
            walkIntersection();//���߹���·����ӵ����������� 

            if((Cardirection + 1 == getDirection(Find[i + 1]) || Cardirection - 3 == getDirection(Find[i + 1])))
            {
                 if(ForwardMPFlag == 1)
					 trackpath[tasklinecount++] = LEFTSMALL;
                trackpath[tasklinecount++] = LEFT;
            }
            else if((Cardirection - 1 == getDirection(Find[i + 1]) || Cardirection + 3 == getDirection(Find[i + 1])))
            {
                 if(ForwardMPFlag == 1)
                    trackpath[tasklinecount++] = RIGHTSMALL; 
                trackpath[tasklinecount++] = RIGHT;
            }
            else
            {
                if(ForwardMPFlag == 1)
                    trackpath[tasklinecount++] = LEFTSMALL; 
                trackpath[tasklinecount++] = LEFT;
                trackpath[tasklinecount++] = LEFT;
            }
			
            Cardirection =  getDirection(Find[i + 1]);		//���µ�ǰ�ķ���
        }
        else     //���ߵĵ�ĸ���
        {
            //�߹��������Ϊһ��·�� ���һ��λ������Ϊ·����������������⴦��
            if(i != 0)	//��ǰ����·�ڲ�����
                JudgeIntersection(i);       //·�ڸ�������
        }
		ForwardMPFlag=1;  //�߹�һ�����Ժ�ˢ��ת������
    }
    //����ǰ�����յ��·���ж�
    if(min - 1 > 0)
    {
        if(JudgeIntersection(min - 1) == 0)
        {
            walkIntersection();//���߹���·����ӵ�����������
            if(Cardirection % 2 == 0)
            {
                trackpath[tasklinecount ++] = MIDDLE_LONGISH;
            }
            else
            {
                trackpath[tasklinecount ++] = MIDDLE_SHORTE;
            }
        }
        else    //	Send_Debug_Info("de1\n",sizeof("de1\n"));
            walkIntersection();//���߹���·����ӵ�����������
    }
}


u8 getStep(u8 *x,u8 *y,u8 type) //ʮ��·���жϣ���ΪMPֵ�ǵ�ʮ��·������
{
    //typeΪ0ʮ��·���жϣ�Ϊ1�������������ʮ��·�ڹ���ǰ��һС�Σ�����ǰ��һ����MPֵ�����۶��ٲ���Ϊ1��
    //��Ϊ��������ʼһ������Ҫ��ѭ��һ��·�ڣ�
    u8 step;
    if(type == 0)
    {
        if((cardirection == 2 && (*y == 0 )) || (cardirection == 1 && (*y == 5))) //����·��Ϊ��һ�����꣬����Ϊ��2������
            step = 1;
        else if(*x % 2 != 0 && *y % 2 != 0)
            step = 2;
        else
            step = 1;
    }
    else
        step = 1;
    return step;
}
#define MINMPS 500
#define MINMPL 700
void getCarPosition(u8 *currentdirection, u8 *x, u8 *y, u8 type) //С����ʵ������
{
    //    char data[10];
//    u8 tx = car_x, ty = car_y;
    if(type == 0)//
    {
        switch(*currentdirection)//ʮ��·��
        {
        case 0:
            car_x -= getStep(&car_x, &car_y, type);
            break;
        case 1:
            car_y += getStep(&car_x, &car_y, type);
            break;
        case 2:
            car_x += getStep(&car_x, &car_y, type);
            break;
        case 3:
            car_y -= getStep(&car_x, &car_y, type);
            break;
        default:
            break;
        }
//        if(car_x == getVauleX(passivity.TerrainPot.data) && car_y == getVauleY(passivity.TerrainPot.data)) //����ǵ��α�־�� �򲻸���·��
//        {
//            car_x = tx;
//            car_y = ty;
//        }
    }
    else if(type == 1) //ѭ��������ֵ
    {
        switch(*currentdirection)	//����Ϊ��  ����Ϊ��
        {
			case 0:
				if(MP > MINMPL)car_x -= getStep(&car_x, &car_y, type);
				break;
			case 1:
				if(MP > MINMPS)car_y += getStep(&car_x, &car_y, type);
				break;
			case 2:
				if(MP > MINMPL)car_x += getStep(&car_x, &car_y, type);
				break;
			case 3:
				if(MP > MINMPS)car_y -= getStep(&car_x, &car_y, type);
				break;
			default:
				break;
        }
    }
    else if(type == 3)//����
    {
        switch(*currentdirection)
        {
			case 0:
				if(MP > MINMPL) *x += 1;
				break;
			case 1:
				if(MP > MINMPS) *y -= 1;
				break;
			case 2:
				if(MP > MINMPL) *x -= 1;
				break;
			case 3:
				if(MP > MINMPS) *y += 1;
				break;
			default:
				break;
        }
    }
}

short int rotate = 0;//С����ת45
char getNewDirection(u8 type, u8 *current) //����ת����ķ���
{

    short  int  result = 0;

    /*����ת����ʱ��Ŵ���ʵ��45����תӦ��Ҳ��������*/
    //	Send_Debug_Info("re1\n",5);
    //	PrintValue(rotate);
    //	Send_Debug_Info("\n",2);
    switch(type)
    {
    case 0:
        rotate = 0;
        result += 1;
        break;
    case 1:
        rotate = 0;
        result -= 1;
        break;
    case 2:
        rotate = 0;
        result += 2;
        break;
    case 3://��45
        rotate += 1;
        break;
    case 4://��45
        rotate -= 1;
        break;
    }
    //*current = rotate;
    *current = (*current + rotate / 2 + result + 4) % 4;
    if(rotate > 1 || rotate < -1)rotate = 0;
    return *current;

}
