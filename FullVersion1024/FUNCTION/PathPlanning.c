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
u8 car_x=0, car_y=0,cardirection=0;		//小车的当前方向
u8 ForwardMPFlag = 1;					//判断在开始的时候是否要加上前进一小段距离
u8 CrossingCount = 0; //路口个数
u8 tasklinecount = 0;
u8 NowTaskPot;		//当前的任务点

u8 Carx, Cary, Cardirection;	//小车中点的坐标方向
struct passivityGather passivity;  //障碍点出现的位置

u8 trackpath[MAXPATH];
xyd LimitPot[MAXLIMIT];

//由于地图限制，起始坐标为1.5
u8 map[7][7] =   //地图
{
    0, 1, 0, 1, 0, 1, 0,
    1, 1, 1, 1, 1, 1, 1,
    0, 1, 0, 1, 0, 1, 0,
    1, 1, 1, 1, 1, 1, 1,
    0, 1, 0, 1, 0, 1, 0,
    1, 1, 1, 1, 1, 1, 1,
    0, 1, 0, 1, 0, 1, 0,
};
short int div[4][2] = {{ -1, 0}, {0, 1}, {1, 0}, {0, -1}}; //右，上，左，下


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

void setMap(u8 x, u8 y, u8 s)//设置障碍点
{
    map[x][y] = s;
}

void setLimitPot(u8 sxy, u8 exy)
{
    LimitPot[0].data = sxy;
    LimitPot[1].data = exy;
}

void setTaskLimitPot()  //设置路线数组
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

void setTaskLimitGather(u8 *arr, u8  len)  //接收一段路径作为必经点
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
    TaskControl.target.data = target; //记录x ，y， dis
    memset(TaskControl.sexy, 0, sizeof(TaskControl.sexy));

    setTaskLimitPot();

    TaskControl.TaskDir = dir;
    TaskControl.LastLineOption = LastOption;
//    if(LastOption != 0)
//    {
        setMPVaule(mpvaule); //赋值给临时码盘值 temp
//    }
    TaskControl.isback = runback;   //是否回退

    for(i = 0; i < MAXPATH; i++)			//清除路线数组
        isRunControl.PathGather[i] =0;
	
    tasklinecount = 0;
    min = inf;  //inf=x03f

    map[Carx][Cary] = 0;
    dfs(setVaule(Carx, Cary, Cardirection), 1);
    map[Carx][Cary] = 1;

    if(min != inf)		//成功生成路线
    {
        if(min == 1)		//路线规划为当前位置
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
            Send_WifiData_To_Fifo(trackpath,tasklinecount);	//打印路径
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
    if(n >= min)return ;//剪枝
    if(getxy(runxy) == getxy(TaskControl.target.data)) //当前位置就在终点位置
    {
        memset(judge, 0, sizeof(judge));  //
        cntlimit = 0;
        for(i = 0; i < n; i++) //1
        {
            for(j = 0; j < MAXLIMIT; j++) //2
            {
                if(judge[j] == 1)continue;
                if((TaskControl.sexy[j].data == 0 ) || (getxy(TaskControl.sexy[j].data) == getxy(TempFind[i]) )) //判断必经点
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
                //遇到地形标志物
                if(!((getDirection(passivity.TerrainPot.data) & 1) == (i & 1)))
                {
                    continue;//不能走
                }
            }
            else   if((tx == getVauleX(passivity.TrafficPot.data)) && (ty == getVauleY(passivity.TrafficPot.data)))
            {
                //遇到交通灯 只能走正面走过
                if(i != getDirection(passivity.TrafficPot.data))
                    continue;
            }
            else if ((tx == getVauleX(passivity.ETCPot.data)) && (ty == getVauleY(passivity.ETCPot.data)))
            {
                //遇到ETC只能从正面走过
                if(i != getDirection(passivity.ETCPot.data))
                    continue;
            }
            else if ((tx == getVauleX(passivity.BarrierszPot.data)) && (ty == getVauleY(passivity.BarrierszPot.data)))
            {
                //遇到道闸可以从两边过
                //               if(i != getDirection(passivity.BarrierszPot.data))continue;
                if(((getDirection(passivity.BarrierszPot.data) & 1) != (i & 1)))continue;
            }
            map[tx][ty] = 0;
            //	printf("n = %d %d %d %d %d %d\n",tx,ty,getVauleX(runxy),getVauleY(runxy),div[i][0],div[i][1]);
            //	sprintf(data,"n = %d %d %d\n",tx,ty,i);

            TempFind[n] = setVaule(tx, ty, i);
            dfs(TempFind[n], n + 1);
            map[tx][ty] = 1;           //回溯
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

u8 JudgeIntersection(int i )   //判断这个点是不是路口
{
    if(getxy(passivity.TerrainPot.data) == getxy(Find[i])) //遇到地形标志物
    {
        walkIntersection();//将走过的路口插入到动态路线
        trackpath[(tasklinecount)++] = GO_TERRAIN; //插入这个被动任务
    }
    else if(getxy(passivity.TrafficPot.data) == getxy(Find[i]))
    {
        //遇到交通灯
        walkIntersection();//将走过的路口插入到动态路线
        trackpath[(tasklinecount)++] = NOW_TRAFFIC; //插入这个被动任务
    }
    else if (getxy(passivity.ETCPot.data) == getxy(Find[i]))
    {
        //遇到ETC
        walkIntersection();//将走过的路口插入到动态路线
        trackpath[(tasklinecount)++] = NOW_ETC; //插入这个被动任务
    }
    else if (getxy(passivity.BarrierszPot.data) == getxy(Find[i]))
    {
        //遇到道闸
        walkIntersection();//将走过的路口插入到动态路线
        trackpath[(tasklinecount)++] = NOW_BARRIERSZ; //插入这个被动任务
    }
    else if( getxy(passivity.RFIDCard.data) == getxy(Find[i]))
    {
        walkIntersection();//将走过的路口插入到动态路线
        //根据上一个点的车头方向和 坐标选择要走的mp值
        if(i != 0)
        {
            if((getVauleX(Find[i]) & 1) && (getVauleY(Find[i - 1]) & 1)) //如果这个点在路口上
            {
                if(getDirection(Find[i - 1]) & 1)          //如果卡在最边上的位置的话那这样就有问题
                    trackpath[(tasklinecount)++] = TRACK_LINE_SHORT; //方向为 1 3 走短边
                else
                    trackpath[(tasklinecount)++] = TRACK_LINE_LANG;//方向为 0 2 走长边
            }
        }

    } else  if(((getVauleY(Find[i]) + 1 ) % 2 == 0 && (getVauleX(Find[i]) + 1) % 2 == 0) //如果这个点是一个路口
               || ((getVauleY(Find[i]) + 1) % 2 == 0 && (getVauleX(Find[i]) == 0)) //边界一定是路口特殊处理
               || ((getVauleY(Find[i]) == 6||(getVauleY(Find[i]) == 0)) && (getVauleX(Find[i]) + 1) % 2 == 0))
    {

        (CrossingCount)++;
        return 1;
    }
    return 0;
}

void lastLine()
{
	taskDirectionDeal();	//方向不同先转弯
    if(TaskControl.TaskDir != 0)
    {
		if(TaskControl.TaskDir == LEFT45)
		{
			if( ForwardMPFlag == 1)	//最后一点的方向前进影响下一次路线规划开始
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
    if(TaskControl.isback == 1) //回退先走码盘在拐弯
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
    else		//不回退则
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

u8 taskDirectionDeal() //用给最后一个路口的判断
{
    if(getDirection(TaskControl.target.dir) != Cardirection)  //方向不同，先转弯
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
        if(getDirection(Find[i + 1]) != Cardirection) //此点的方向不同
        {
            if(i != 0)	//当前点是路口不处理
                JudgeIntersection(i);//判断当前点是不是路口
            walkIntersection();//将走过的路口添加到行走数组中 

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
			
            Cardirection =  getDirection(Find[i + 1]);		//更新当前的方向
        }
        else     //行走的点的个数
        {
            //走过的这个点为一个路口 最后一个位置是作为路口来算的这里做特殊处理
            if(i != 0)	//当前点是路口不处理
                JudgeIntersection(i);       //路口个数增加
        }
		ForwardMPFlag=1;  //走过一个点以后刷新转弯码盘
    }
    //处理前进到终点的路口判断
    if(min - 1 > 0)
    {
        if(JudgeIntersection(min - 1) == 0)
        {
            walkIntersection();//将走过的路口添加到行走数组中
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
            walkIntersection();//将走过的路口添加到行走数组中
    }
}


u8 getStep(u8 *x,u8 *y,u8 type) //十字路口判断，因为MP值是到十字路口清零
{
    //type为0十字路口判断，为1，其它情况（非十字路口拐弯前的一小段，而且前行一定的MP值，无论多少操作为1，
    //因为不可能像开始一样，需要多循迹一个路口）
    u8 step;
    if(type == 0)
    {
        if((cardirection == 2 && (*y == 0 )) || (cardirection == 1 && (*y == 5))) //经过路口为加一个坐标，否则为加2个坐标
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
void getCarPosition(u8 *currentdirection, u8 *x, u8 *y, u8 type) //小车的实际坐标
{
    //    char data[10];
//    u8 tx = car_x, ty = car_y;
    if(type == 0)//
    {
        switch(*currentdirection)//十字路口
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
//        if(car_x == getVauleX(passivity.TerrainPot.data) && car_y == getVauleY(passivity.TerrainPot.data)) //如果是地形标志物 则不更新路线
//        {
//            car_x = tx;
//            car_y = ty;
//        }
    }
    else if(type == 1) //循迹走码盘值
    {
        switch(*currentdirection)	//左右为长  上下为短
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
    else if(type == 3)//倒退
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

short int rotate = 0;//小车左转45
char getNewDirection(u8 type, u8 *current) //计算转动后的方向
{

    short  int  result = 0;

    /*连续转动的时候才处理，实际45度旋转应该也是连续的*/
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
    case 3://左45
        rotate += 1;
        break;
    case 4://右45
        rotate -= 1;
        break;
    }
    //*current = rotate;
    *current = (*current + rotate / 2 + result + 4) % 4;
    if(rotate > 1 || rotate < -1)rotate = 0;
    return *current;

}
