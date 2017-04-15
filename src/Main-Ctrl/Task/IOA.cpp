#include "IOA.h"
#include <string.h>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>
using namespace std;
IOA *IOA::m_pCIOA;
#include <pthread.h>
pthread_mutex_t g_Mutex_CurPose;


IOA_Pos IOA::m_stSickARange[SICK_LINES];
IOA_Pos IOA::m_stSickBRange[SICK_LINES];

unsigned int IOA::s_cur_t(0);
unsigned int IOA::s_las_t(0);

float IOA::m_fAngle[SICK_LINES][2];
float IOA::m_fSecurityAreaRange[SECURITY_AREA_NUM];

IOAParams IOA::m_stIOAParams;
IOA::IOA(void)
{
	m_pCIOA=this;
	pthread_mutex_init(&g_Mutex_CurPose,NULL);

}

IOA::~IOA(void)
{
	//pthread_mutex_destory(&g_Mutex_CurPose);
}


int IOA::IOAInit()
{
	int i;

	float fStart=3.1415926/180*INIT_ANGLE;
	float fSetp=SICK_RESOLUTION/180*3.1415926;
	for (i=0;i<SICK_LINES;i++)
	{
		m_fAngle[i][0]=cos(fStart);
		m_fAngle[i][1]=sin(fStart);

	//	printf("m_fAngle:%f  ,%f  \n",m_fAngle[i][0],m_fAngle[i][1]);
		fStart-=fSetp;
	}

	m_fSecurityAreaRange[0]=LELVEL0;
	m_fSecurityAreaRange[1]=LELVEL1;
	m_fSecurityAreaRange[2]=LELVEL2;
	m_fSecurityAreaRange[3]=LELVEL3;
	m_fSecurityAreaRange[4]=LELVEL4;


	m_fLocalSpeedLimitList[0]=MAX_SPEED_L0;
	m_fLocalSpeedLimitList[1]=MAX_SPEED_L1;
	m_fLocalSpeedLimitList[2]=MAX_SPEED_L2;
	m_fLocalSpeedLimitList[3]=MAX_SPEED_L3;
	m_fLocalSpeedLimitList[4]=MAX_SPEED_L4;
	return RTN_OK;
}

int IOA::IOARun()
{
	return RTN_OK;
}
int IOA::IOAStop()
{
	return RTN_OK;
}
int IOA::IOAUninit()
{
	return RTN_OK;
}

int IOA::GetLaserAData(vector<float> vctSICKData)
{
	int i;

	/*float fStart=3.1415926/180*(-45);
	float fSetp=SICK_RESOLUTION/180*3.1415926;
	for (i=0;i<SICK_LINES;i++)
	{
		m_fAngle[i][0]=cos(fStart);
		m_fAngle[i][1]=sin(fStart);
		fStart+=fSetp;
	}*/

	++s_cur_t;
	if(s_cur_t >= 1000000) s_cur_t = 1;

	for(i=0;i<vctSICKData.size();i++)
	{

		m_stSickARange[i].fDis=vctSICKData[i];


		m_stSickARange[i].fX=m_fAngle[i][0]*m_stSickARange[i].fDis;
		m_stSickARange[i].fY=m_fAngle[i][1]*m_stSickARange[i].fDis;


		m_stSickARange[i].nIdxX=m_stSickARange[i].fX/0.1;
		m_stSickARange[i].nIdxY=m_stSickARange[i].fY/0.1;
	}
	return RTN_OK;
}
int IOA::GetLaserBData(vector<float> vctSICKData)
{
	int i;

	// get time of this new scan
	// struct timeval t;
	// gettimeofday(&t,0);
	// IOA::s_cur_t = t.tv_sec*1000 + t.tv_usec/1000.;
	for(i=0;i<541;i++)
	{
		m_pCIOA->m_stSickBRange[i].fDis=vctSICKData[i];


		m_pCIOA->m_stSickBRange[i].fX=m_pCIOA->m_fAngle[i][0]*vctSICKData[i];
		m_pCIOA->m_stSickBRange[i].fY=m_pCIOA->m_fAngle[i][1]*vctSICKData[i];

		m_pCIOA->m_stSickBRange[i].nIdxX=m_pCIOA->m_stSickARange[i].fX/0.1;
		m_pCIOA->m_stSickBRange[i].nIdxY=m_pCIOA->m_stSickARange[i].fY/0.1;
	}
	return RTN_OK;
}

int IOA::CanRunAreaCheck_Blend()
{
	int i;
	char cCount[SECURITY_AREA_NUM],cObj;
	memset(cCount,0,sizeof(char)*SECURITY_AREA_NUM);
	cObj=0;

	m_fLocalSpeedLimit=-1;
	for(i=0;i<SICK_LINES;i++)
	{
		if(fabs(m_pCIOA->m_stSickARange[i].fX)<SECURTY_WIDTH)
		{
			if (m_pCIOA->m_stSickARange[i].fY<BLEND_LEVEL)
			{
				cCount[0]++;
				cObj++;
			}
		}
	}
	if(cObj>=5)
	{
		return -1;
	}
	return 0;
}

int IOA::CanRunAreaCheck()
{
	int i;
	char cCount[SECURITY_AREA_NUM],cObj;
	memset(cCount,0,sizeof(char)*SECURITY_AREA_NUM);
	cObj=0;

	//printf("CanRunAreaCheck!!!!\n");
	m_fLocalSpeedLimit=-1;
	for(i=90;i<360;i++)
	{

		if(m_stSickARange[i].fX==0&&m_stSickARange[i].fY==0)
		{
			continue;
		}

		/*if(fabs(m_pCIOA->m_stSickARange[i].fX)<SECURTY_WIDTH)
		{
			&&fabs(m_pCIOA->m_stSickARange[i].fY)<SECURTY_WIDTH
		}*/
		/*if(m_pCIOA->m_stSickARange[i].fX<50&&m_pCIOA->m_stSickARange[i].fX>0)
		{
			printf("%%%m_pCIOA->m_stSickARange[i].fX:%f \n",m_pCIOA->m_stSickARange[i].fX);
		}*/
		if(fabs(m_stSickARange[i].fX)<SECURTY_WIDTH)
		{
			if (m_stSickARange[i].fY<m_fSecurityAreaRange[0])
			{
				cCount[0]++;
				cObj++;
			}
			else if(m_stSickARange[i].fY<m_fSecurityAreaRange[1])
			{
				cCount[1]++;
				cObj++;
			}
			else if(m_stSickARange[i].fY<m_fSecurityAreaRange[2])
			{
				cCount[2]++;
				cObj++;
			}
			else if(m_stSickARange[i].fY<m_fSecurityAreaRange[3])
			{
				cCount[3]++;
				cObj++;
			}
			else if(m_stSickARange[i].fY<m_fSecurityAreaRange[4])
			{
				cCount[4]++;
				cObj++;
			}
		/*	if(m_pCIOA->m_stSickARange[i].fY<m_fSecurityAreaRange[4])
			{
				cCount[4]++;
				cObj++;
				if (m_pCIOA->m_stSickARange[i].fY<m_fSecurityAreaRange[3])
				{
					cCount[3]++;
					cObj++;
					if (m_pCIOA->m_stSickARange[i].fY<m_fSecurityAreaRange[2])
					{
						cCount[2]++;
						cObj++;
						if (m_pCIOA->m_stSickARange[i].fY<m_fSecurityAreaRange[1])
						{
							cCount[1]++;
							cObj++;
							if (m_pCIOA->m_stSickARange[i].fY<m_fSecurityAreaRange[0])
							{
								cCount[0]++;
								cObj++;
							}
						}
					}
				}
			}*/
		}

		if(cObj>0)
		{
			if(cCount[0]!=0)
			{
				//printf("STop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1\n");
				return -10;
			}
			else if(cCount[1]!=0)
			{
				//m_fLocalSpeedLimit=m_fLocalSpeedLimitList[1];
				//if(m_fDis2MileStone>m_fSecurityAreaRange[1])
				//{
				//	Dodge(1);
				//}
				//printf("ZONE 111111111111!!!!!!!!!!!!1\n");

				return -1;
				
			}
			else if(cCount[2]!=0)
			{
				//printf("ZONE 2222222222222!!!!!!!!!!!!1\n");
				//Dodge(2);
				//if(m_fDis2MileStone>m_fSecurityAreaRange[2])
				//{
				//	Dodge(2);
				//}
				return -1;
			}
			else if(cCount[3]!=0)
			{
				m_fLocalSpeedLimit=m_fLocalSpeedLimitList[3];
			}
			else if(cCount[4]!=0)
			{
				m_fLocalSpeedLimit=m_fLocalSpeedLimitList[4];
			}
			break;
		}
	}
	return RTN_OK;
}

int IOA::Dodge(int nSecurityRange)
{

	//m_CPathPlanning.DodgePath(m_pCIOA->m_stSickBRange,&m_pCIOA->m_stCurPos);
	return 0;
}

int IOA::GetCurPose(float *pfPos)
{
	pthread_mutex_lock(&g_Mutex_CurPose);

	pfPos[0]=m_pCIOA->m_stCurPos.stPoint.fX;
	pfPos[1]=m_pCIOA->m_stCurPos.stPoint.fY;
	pfPos[2]=m_pCIOA->m_stCurPos.fTheta;

	pthread_mutex_unlock(&g_Mutex_CurPose);

	return 0;
}

int IOA::DataFusionResult(float fX,float fY,float fTheta)
{


	if(pthread_mutex_lock(&g_Mutex_CurPose)==0)
	{
		m_pCIOA->m_stCurPos.stPoint.fX=fX;
		m_pCIOA->m_stCurPos.stPoint.fY=fY;
		m_pCIOA->m_stCurPos.fTheta=fTheta;
		pthread_mutex_unlock(&g_Mutex_CurPose);
	}
	return 0;
}


int IOA::Vel2RPM(float fVel)
{
	return fVel/WHEEL_RADIOUS*60/PI;	
}

int IOA::Step2MileStone(Point_f stPoint,RectilinearCtrl stRectilinearCtrl)
{
	float fErr=999999;
	float fWheelSpd;
	int nTimeSlice=10,nRPM,nCount=0;
	switch(stRectilinearCtrl.cType)
	{
		case 1://yun su
			fWheelSpd=stRectilinearCtrl.fCurSpd;
			nRPM=Vel2RPM(fWheelSpd);
			//IICTRL_Wheel_nW(nRPM,nRPM,10);
			while(!m_bStopThisStep&&fErr<MOTION_ERR)
			{
				fErr=(stPoint.fX-m_stCurPos.stPoint.fX)*(stPoint.fX-m_stCurPos.stPoint.fX)+
					(stPoint.fY-m_stCurPos.stPoint.fY)*(stPoint.fY-m_stCurPos.stPoint.fY);
				m_fDis2MileStone=fErr;
				usleep(10000);
			}
		break;
		case 2:// 
			while(!m_bStopThisStep&&fErr<MOTION_ERR)
			{
				fWheelSpd=stRectilinearCtrl.fA*nCount+stRectilinearCtrl.fCurSpd;
				nRPM=Vel2RPM(fWheelSpd);
			//	IICTRL_Wheel_nW(nRPM,nRPM,10);

				fErr=(stPoint.fX-m_stCurPos.stPoint.fX)*(stPoint.fX-m_stCurPos.stPoint.fX)+
					(stPoint.fY-m_stCurPos.stPoint.fY)*(stPoint.fY-m_stCurPos.stPoint.fY);
				m_fDis2MileStone=fErr;
				usleep(10000);
				nCount+=nTimeSlice;
			}
		break;
		case 3:
			while(!m_bStopThisStep&&fErr<MOTION_ERR)
			{
				if(nCount<stRectilinearCtrl.fTAcc)
				{
					fWheelSpd=MAX_ACC_SPD*nCount+stRectilinearCtrl.fCurSpd;
					nRPM=Vel2RPM(fWheelSpd);
				//	IICTRL_Wheel_nW(nRPM,nRPM,10);
				}
				else if(nCount>=stRectilinearCtrl.fTAcc&&nCount<stRectilinearCtrl.fTAcc+stRectilinearCtrl.fTUni)
				{
				//	IICTRL_Wheel_nW(nRPM,nRPM,10);
					nCount=0;
				}
				else 
				{
					fWheelSpd=MAX_DEACC_SPD*nCount+stRectilinearCtrl.fCurSpd;
					nRPM=Vel2RPM(fWheelSpd);
				//	IICTRL_Wheel_nW(nRPM,nRPM,10);
				}
				fErr=(stPoint.fX-m_stCurPos.stPoint.fX)*(stPoint.fX-m_stCurPos.stPoint.fX)+
					(stPoint.fY-m_stCurPos.stPoint.fY)*(stPoint.fY-m_stCurPos.stPoint.fY);
				m_fDis2MileStone=fErr;
				usleep(10000);
				nCount+=nTimeSlice;
			}
		break;
		case 4:
			while(!m_bStopThisStep&&fErr<MOTION_ERR)
			{
				if(nCount<stRectilinearCtrl.fTAcc)
				{
					fWheelSpd=MAX_ACC_SPD*nCount+stRectilinearCtrl.fCurSpd;
					nRPM=Vel2RPM(fWheelSpd);
					m_fDis2MileStone=fErr;
				//	IICTRL_Wheel_nW(nRPM,nRPM,10);
				}
				else 
				{
					fWheelSpd=MAX_DEACC_SPD*nCount+stRectilinearCtrl.fCurSpd;
					nRPM=Vel2RPM(fWheelSpd);
					m_fDis2MileStone=fErr;
				//	IICTRL_Wheel_nW(nRPM,nRPM,10);
				}
				fErr=(stPoint.fX-m_stCurPos.stPoint.fX)*(stPoint.fX-m_stCurPos.stPoint.fX)+
					(stPoint.fY-m_stCurPos.stPoint.fY)*(stPoint.fY-m_stCurPos.stPoint.fY);
				usleep(10000);
			}
		break;
		default:
		break;
	}
		
	return 0;
}


// RIGHT -> LEFT
#define LEFT_OCC 60
#define RIGHT_OCC 40
#define ANGLE_THRE 20 
#define MIN_DIS 3
#define SPEED_DIS 5
#define ROBOT_LEN 0.8
#ifndef PM_R2D
#define PM_R2D (180./M_PI)
#define PM_D2R (M_PI/180.)
#endif
/*
* [start - end) [input] check area :[180-360)
* [RIGHT_OCC LEFT_OCC] occlusion area
* slow_dis [input] occlusion dis to slow : 3 m,
* stop_dis [input] occlusion dis to stop : 1.5 m, 
* win_range [input] search width range to forward : 5 almost 5'
* angle [output] angle to rotate [left +, right -]
* Return : 
* 	-1 error happened
*	0 speed up
*	1 stop -> turn angle
*	2 stop -> turn random angle
*	3 slow -> turn angle
* 	4 slow -> not turn angle
*	5 not changed
*/ 
int IOA::Look4Window2( float& angle, int start, int end, int slow_dis, int stop_dis, int win_range)
{
	if(end <= start || start <0 || end > 541)
	{
		cout<<"IOA.cpp: range error!"<<endl;
		return -1;
	}
	// check time stamp
	if(IOA::s_cur_t == IOA::s_las_t)
	{
		cout<<"IOA.cpp same scan!"<<endl;
		return 0;
	}
	IOA::s_las_t = IOA::s_cur_t;

	int N = end - start;
	int ret_Status = 0;
	vector<float> frontpts(N,0);
	vector<float> weights(N,0);
	vector<float> bweights(N,0);
#define MAX_R 10000
#define MIN_R 0.0001
	float min_r = MAX_R;
	float max_r = MIN_R;
	float c_r;

	int single_d = N/2; // current robot 's orientation
	for(int i=0;i<N;i++){
		c_r = m_stSickARange[i+start].fDis/100.;
		if(c_r == 0) continue;
		if( single_d - RIGHT_OCC < i && i < single_d + LEFT_OCC ){
			if(c_r <= stop_dis)
			{
				ret_Status = 1; // stop
			}else if(ret_Status !=1 && c_r <= slow_dis)
			{	
				ret_Status = 3; // slow
			}
		}
		frontpts[i] = c_r;
		if(c_r > max_r) max_r = c_r;
		if(c_r < min_r) min_r = c_r;
	}
	if(min_r == MAX_R || max_r == MIN_R){
		cout<<"IOA.cpp: this scan is not valid!!"<<endl;
		return -1;
	}

	vector<int> segs;
	int n_of_seg = segment_Scan2(frontpts,segs); // segment the scan
	// vector<float> score(n_of_seg);
	vector<int> index(n_of_seg+1,0);
	vector<float> score(n_of_seg+1,0);
	float max_score = MIN_R;
	// float score;
	int ret_index = 0;
	
	for(int i=1;i<=n_of_seg;i++)
	{
		score[i] = segment_Score2(frontpts, segs, i, index[i], start);
		/*score = segment_Score(frontpts,segs,i,index[i], start);*/
		if(score[i] > max_score) 
		{
			max_score = score[i];
			ret_index = index[i];
		}
	}
	//if(max_score < ROBOT_LEN){
	if(max_score <m_stIOAParams.g_robot_len){
		cout<<"IOA.cpp: No available area!"<<endl;
		return 2;
	}

	if(ret_index == 0){
	    cout<<"IOA.cpp: No nice segment!"<<endl;
	    return 2;
	}
	
	int min_change_angle = MAX_R;
	for(int i=1; i<= n_of_seg; i++)
	{
		int change = abs(index[i] - single_d);
	//	if(change < min_change_angle && score[i] >= ROBOT_LEN)
		if(change < min_change_angle && score[i] >= m_stIOAParams.g_robot_len)
		{
			min_change_angle = change;
			ret_index = index[i];
		}
	}

/*
	for(int i=0; i< N; i++){
		weights[i] = (frontpts[i] - min_r)/(max_r - min_r);
	}
	int size_w = win_range;
	float norm = (1./((2*size_w)+1));
	int ret_index = 0; // the max weight orientation
	float max_w = 0;  
	for(int i=size_w; i < N-size_w-1; i++){
		float w = 0;
		for(int j=i-size_w; j<=i+size_w; j++){
			w+=weights[i];
		}
		w*=norm;
		if(w>max_w) {max_w = w; ret_index = i;}
	}
*/	
	// find the angle to rotate
	int angle_change = ret_index - single_d;
	angle = PM_D2R*((float)(angle_change)*0.5); 
	
	//if( ret_Status !=0 && (angle_change > -ANGLE_THRE && angle_change < ANGLE_THRE ))
	if( ret_Status !=0 && (angle_change > -m_stIOAParams.g_small_angle
			&& angle_change < m_stIOAParams.g_small_angle ))
        {
		// cout<<"small angle "<<PM_R2D*angle<<", no need rotation!"<<endl;
		return ++ret_Status;
	}
	if(ret_Status == 0)
	{
	    	float mean_dis = m_stSickARange[start+ret_index].fDis/100.;
		if( mean_dis < SPEED_DIS)
		{	
			return 5; // continue;
		}
	}
	// cout<<"index: "<< ret_index <<" rotation : "<<PM_R2D*angle<<endl;
	return ret_Status;
}

double IOA::segment_Score2(vector<float>& f, vector<int>& segs, int seg, int& index, int shift)
{
	double sum = 0;
	int cnt = 0;
	int start = 0;
	int end = f.size()-1;
	bool bfirst = true;
	bool bfound = false;

	int i=0;
	int ac_st = 0;
	int ac_et = f.size()-1;
	while(f[i]==0){i++; ac_st = i;}
	i = ac_et;
	while(f[i]==0){i--; ac_et = i;}

	for(int i=ac_st;i<=ac_et;i++){
		if(segs[i] == seg)
		{
			bfound = true;
			if(bfirst) 
			{
				bfirst = false;
				start = i;
			}
			end = i;
			++cnt;
			// sum+=f[i];	
		}else if(segs[i] != seg && !bfirst){
			break;
		}		
	}
	
	if(cnt <= 10 || !bfound) return 0;

	int st, et;
	if(start == ac_st || end == ac_et){
		st = start;
		et = end;
	}else{
		st = start - 1;
		et = end + 1;
		//while(st > ac_st && (f[st] ==0 || f[st] > MIN_DIS))
		while(st > ac_st && (f[st] ==0 || f[st] > m_stIOAParams.g_min_dis))
			st -- ;
		//while((et < ac_et) && (f[et] == 0 || f[et] > MIN_DIS))
		while((et < ac_et) && (f[et] == 0 || f[et] > m_stIOAParams.g_min_dis))
			et++;
	}

	// angle displacement
	index = st+(et-st)/2;
	double tangle = sin(PM_D2R*(et-st)*0.5*0.5);
	if(tangle < 0) tangle *=-1.;
	double line = f[st] < f[et] ?  f[st] : f[et];
	double score = line*tangle*2;
	
	/*double x1,y1,x2,y2;
	x1 = m_stSickARange[st+shift].fX/100.;  y1 = m_stSickARange[st+shift].fY/100.;
	x2 = m_stSickARange[et+shift].fX/100.;  y2 = m_stSickARange[et+shift].fY/100.;
	double score = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
	*/

	return score;

	// angle displacement
	/*index = start+(end-start)/2;
	double tangle = tan(PM_D2R*(end-start)*0.5*0.5);
	if(cnt<3) {return 0;}
	
	double dis = sum/cnt;
	return (dis*2.*tangle);*/
}

double IOA::segment_Score(vector<float>& f, vector<int>& segs, int seg, int& index, int shift)
{
	double sum = 0;
	int cnt = 0;
	int start = 0;
	int end = f.size()-1;
	bool bfirst = true;
	for(int i=0;i<f.size();i++){
		if(segs[i] == seg){
			if(bfirst) {
				bfirst = false;
				start = i;
			}
			end = i;
			++cnt;
			sum+=f[i];	
		}else if(segs[i] > seg && !bfirst){
			break;
		}		
	}
	if(cnt<3) {return 0;}
	// distance to robot 
	sum /= cnt;
	// length of the segment
	float sx = m_stSickARange[start+shift].fX/100.;
	float sy = m_stSickARange[start+shift].fY/100.;
	float ex = m_stSickARange[end+shift].fX/100.;
	float ey = m_stSickARange[end+shift].fY/100.;
	double length = sqrt((ex-sx)*(ex-sx) + (ey-sy)*(ey-sy));
	// angle displacement
	int middle = f.size()/2;
	index = start+(end-start)/2;
	double angle_dis = cos(PM_D2R*(index - middle)*0.5);

	return (sum*length*angle_dis);
}

int IOA::segment_Scan2(vector<float>& f, vector<int>& segs)
{
	segs.clear();
	segs.resize(f.size(),0);
	//const float   MAX_DIST = MIN_DIS;//max range diff between conseq. points in a seg
	const float   MAX_DIST = m_stIOAParams.g_min_dis;
	double   dr;
	int       seg_cnt = 0;
	int       i,cnt;
	bool      break_seg;
	if(f.size()<2){
		cout<<"IOA.cpp error in segment_Scan!"<<endl;
		return -1;
	}

	seg_cnt = 1;
	cnt = 0;
	for(int i=0;i<f.size();i++){
		if(f[i]> MAX_DIST){
			segs[i] = seg_cnt;
			cnt++;
		}else{
			segs[i] = 0;
			if(i>1 && cnt <= 1)
			{
				segs[i-1] = 0;
			}else if(cnt > 1){
				seg_cnt ++;
			}
			cnt = 0;
		}
	}

	return seg_cnt;
}

int IOA::segment_Scan(vector<float>& f, vector<int>& segs)
{
	segs.clear();
	segs.resize(f.size(),0);
	const float   MAX_DIST = 0.2;//max range diff between conseq. points in a seg 
	double   dr;
	int       seg_cnt = 0;
	int       i,cnt;
	bool      break_seg;
	if(f.size()<2){
		cout<<"IOA.cpp error in segment_Scan!"<<endl;
		return -1;
	}

	seg_cnt = 1;

	//init:
	if ( fabsf ( f[0]-f[1] ) < MAX_DIST ) //are they in the same segment?
	{
		segs[0] = seg_cnt;
		segs[1] = seg_cnt;
		cnt        = 2;    //2 points in the segment
	}
	else
	{
		segs[0] = 0; //point is a segment in itself
		segs[1] = seg_cnt;
		cnt        = 1;
	}
	for(i=2;i<f.size();i++){
		break_seg = false;
		if(f[i] == 0){
			break_seg = true;
			segs[i] = 0;
		}else{
			dr = f[i] - (2.0*f[i-1] - f[i-2]); 
			if(fabs(f[i]-f[i-1]) < MAX_DIST || \
				((segs[i-1] == segs[i-2]) && fabsf(dr) < MAX_DIST)){
					cnt++; 
					segs[i] = seg_cnt;
				}else{
					break_seg = true;
				}
		}

		if(break_seg){
			if(cnt==1){
				dr = f[i] - (2.0*f[i-1] - f[i-2]);
				if(segs[i-2] ==0 && f[i]!=0 && f[i-1] != 0 \
				&& f[i-2] !=0 && fabsf(dr) < MAX_DIST)
				{
					segs[i] = seg_cnt;
					segs[i] = seg_cnt;
					segs[i] = seg_cnt;
					cnt = 3;
				}else{
					segs[i-1] = 0;
					segs[i] = seg_cnt;
					cnt =1;
				}
			}else{
				++seg_cnt; 
				segs[i] = seg_cnt;
				cnt = 1;
			}
		}
	}
	return seg_cnt;
}


int IOA::Look4Window()
{
	int i,nCount,j;
	vector<float> vctSICKData;
	GetLaserAData(vctSICKData);

	int nStart,nEnd;
	int nClearDirect;

	nCount=0;

	vector<int> vctTmp;

	printf("$$$$$$$$$$$$$$$$$$$$$$$$$$4\n");
	for(i=180;i<360;i++)
	{
		nCount=0;
		printf("SICK Dis:%f  \n",m_pCIOA->m_stSickARange[i].fDis);
		if(m_stSickARange[i].fDis>1.5)
		{

			nStart=i;
			for(j=i;j<360;j++)
			{
				if(m_stSickARange[j].fDis>1.5)
				{
				//	printf("$$$$$$Data%f  \n", m_pCIOA->m_stSickARange[j].fDis);
					nCount++;
					i++;
				}
				else
				{
					printf("#######Data%f  \n", m_stSickARange[j].fDis);
					break;
				}
			}
			nEnd=i;

			vctTmp.push_back(nStart);
			vctTmp.push_back(nEnd);
			vctTmp.push_back(nCount);
		}
	}
	printf("~~~~~~~~~~~~~~~~~~~~~~\n");
	int nMaxValue=0,nMaxIdx;

	if(vctTmp.size()!=0)
	{
		for(i=0;i<vctTmp.size();i=i+3)
		{
			if(vctTmp[i*3+2]>nMaxValue)
			{
				nMaxIdx=i;
				nMaxValue=vctTmp[i+2];
			}
		}

		nStart=vctTmp[nMaxIdx*3];
		nEnd=vctTmp[nMaxIdx*3+1];

		nClearDirect=(nStart+nEnd)/2;
		nClearDirect=nClearDirect/2;
		printf("&&&&&&&&&&&&&&&&&&&&&&&\n");
	}

	return nClearDirect;
}

int IOA::Look4Window3( float& angle,
		int start, int end,
		int slow_dis, int stop_dis,
		int win_range,float fDestX,float fDestY)
{
	if(end <= start || start <0 || end > 541)
	{
		cout<<"IOA.cpp: range error!"<<endl;
		return -1;
	}
	// check time stamp
	if(IOA::s_cur_t == IOA::s_las_t)
	{
		cout<<"IOA.cpp same scan!"<<endl;
		return 0;
	}
	IOA::s_las_t = IOA::s_cur_t;

	int N = end - start;
	int ret_Status = 0;
	vector<float> frontpts(N,0);
	vector<float> weights(N,0);
	vector<float> bweights(N,0);
#define MAX_R 10000
#define MIN_R 0.0001
	float min_r = MAX_R;
	float max_r = MIN_R;
	float c_r;

	int single_d = N/2; // current robot 's orientation
	for(int i=0;i<N;i++){
		c_r = m_stSickARange[i+start].fDis/100.;
		if(c_r == 0) continue;
		if( single_d - RIGHT_OCC < i && i < single_d + LEFT_OCC ){
			if(c_r <= stop_dis)
			{
				ret_Status = 1; // stop
			}else if(ret_Status !=1 && c_r <= slow_dis)
			{
				ret_Status = 3; // slow
				printf("ret_Status==3!!!!!!!!!!!!!\n");
			}
		}
		frontpts[i] = c_r;
		if(c_r > max_r) max_r = c_r;
		if(c_r < min_r) min_r = c_r;
	}
	if(min_r == MAX_R || max_r == MIN_R){
		cout<<"IOA.cpp: this scan is not valid!!"<<endl;
		return -1;
	}

	vector<int> segs;
	int n_of_seg = segment_Scan2(frontpts,segs); // segment the scan

	// vector<float> score(n_of_seg);
	vector<int> index(n_of_seg+1,0);
	vector<float> score(n_of_seg+1,0);
	float max_score = MIN_R;
	// float score;
	int ret_index = 0;

	for(int i=1;i<=n_of_seg;i++)
	{
		score[i] = segment_Score2(frontpts, segs, i, index[i], start);
		/*score = segment_Score(frontpts,segs,i,index[i], start);*/
		if(score[i] > max_score)
		{
			max_score = score[i];
			ret_index = index[i];
		}
	}
	//if(max_score < ROBOT_LEN){
	if(max_score <m_stIOAParams.g_robot_len){
		cout<<"IOA.cpp: No available area!"<<endl;
		return 2;
	}

	if(ret_index == 0){
	    cout<<"IOA.cpp: No nice segment!"<<endl;
	    return 2;
	}

	int min_change_angle = MAX_R;


	float fCurPos[3];
	GetCurPose(fCurPos);
	fCurPos[0]/=1000;
	fCurPos[1]/=1000;
	float fDX,fDY,fCurDirect;
	fDX=fDestX-fCurPos[0];
	fDY=fDestY-fCurPos[1];

	printf("fDX:%f  ,fDY:%f  \n",fDX,fDY);

	float fDestDirect=atan2(fDY,fDX);

	if(fDestDirect<0)fDestDirect=fDestDirect+3.1415926*2;
	printf("fDestDirect1:%f  \n",fDestDirect);
	//fCurDirect=fCurPos[2]+3.1415926/2;

	printf("fCurDirect:%f  \n",fCurDirect);


	fDestDirect=fDestDirect-fCurPos[2];


	printf("fDestDirect2:%f  \n",fDestDirect);
	//fDestDirect=NormalAngle(fDestDirect);
	printf("fDestDirect3	:%f  \n",fDestDirect);
	float fMinErrValue=10000,fErrValue,fWinAngle,fBestWinAngle;
	int nOptimumDirect;


	printf("fCurPos:%f ,%f ,%f \n",fCurPos[0],fCurPos[1],fCurPos[2]);
	printf("Dest:%f ,%f \n",fDestX,fDestY);
	printf("fDestDirect:%f  \n",fDestDirect);


	if(fDestDirect>0&&fDestDirect<3.1415926*2)
	{
		for(int i=1; i<= n_of_seg; i++)
		{
		/*	int change = abs(index[i] - single_d);
			if(change < min_change_angle && score[i] >= m_stIOAParams.g_robot_len)
			{
				min_change_angle = change;
				ret_index = index[i];
			}*/
			int change = index[i] - single_d;
			fWinAngle=(0.5*change+90)/180*3.1415926;

			printf("change:%d  ,fWinOrigal:%f  ,fWinAngle:%f  \n",change,0.5*change/180*3.1415926,fWinAngle);




			fErrValue=fabs(fWinAngle-fDestDirect);
			if(fErrValue<fMinErrValue&&score[i]&&score[i]>m_stIOAParams.g_robot_len)
			{
				nOptimumDirect=i;
				fMinErrValue=fErrValue;
				fBestWinAngle=fWinAngle;
			}
		}
	}
	else
	{
		//stop and turn
		if(fDestDirect>-3.1415926/2)
		{
			angle=fDestDirect-3.1415926/2;
			return 1;
		}
		else
		{
			angle=3.1415926+fDestDirect+3.1415926/2;
			return 1;
		}
	}

	printf("nOptimumDirect:%d, fMinErrValue:%f \n",nOptimumDirect,fMinErrValue);

	if(fMinErrValue==10000)
	{
		// Spin
		return 2;
	}
	else
	{
		ret_index=index[nOptimumDirect];
	}

	float fStartAngle,fEndAngle,fBestAngle;
	SearchBoundaryAngle(fStartAngle,fEndAngle,segs,nOptimumDirect,fBestWinAngle);
	fBestAngle=BestTurnAngle(fStartAngle,fEndAngle,fBestWinAngle,score[nOptimumDirect],fDestDirect);
	printf("BestTurnAngle111 reslut:%f  \n",fBestAngle);
	fBestAngle=fBestAngle-3.1415926/2;
	printf("BestTurnAngle222 reslut:%f  \n",fBestAngle);

	/*for(int i=1; i<= n_of_seg; i++)
	{
		int change = abs(index[i] - single_d);
	//	if(change < min_change_angle && score[i] >= ROBOT_LEN)
		if(change < min_change_angle && score[i] >= m_stIOAParams.g_robot_len)
		{
			min_change_angle = change;
			ret_index = index[i];
		}
	}*/

/*
	for(int i=0; i< N; i++){
		weights[i] = (frontpts[i] - min_r)/(max_r - min_r);
	}
	int size_w = win_range;
	float norm = (1./((2*size_w)+1));
	int ret_index = 0; // the max weight orientation
	float max_w = 0;
	for(int i=size_w; i < N-size_w-1; i++){
		float w = 0;
		for(int j=i-size_w; j<=i+size_w; j++){
			w+=weights[i];
		}
		w*=norm;
		if(w>max_w) {max_w = w; ret_index = i;}
	}
*/
	// find the angle to rotate

	//angle = PM_D2R*((float)(angle_change)*0.5);
	int angle_change = ret_index - single_d;
	angle_change=fBestAngle;
	angle=fBestAngle;

	//if( ret_Status !=0 && (angle_change > -ANGLE_THRE && angle_change < ANGLE_THRE ))

	//SS
	//if( ret_Status !=0 && (angle_change > -m_stIOAParams.g_small_angle
	//		&& angle_change < m_stIOAParams.g_small_angle ))
	if(ret_Status !=0 && (angle_change > -10/180*3.1415926
			&& angle_change < 10/180*3.1415926 ))
        {
		printf("++ret_Status !!!!!!!!!!!!!!1\n");
		// cout<<"small angle "<<PM_R2D*angle<<", no need rotation!"<<endl;
		return ++ret_Status;
	}
	if(ret_Status == 0)
	{
	    	float mean_dis = m_stSickARange[start+ret_index].fDis/100.;
		if( mean_dis < SPEED_DIS)
		{
			return 5; // continue;
		}
	}
	// cout<<"index: "<< ret_index <<" rotation : "<<PM_R2D*angle<<endl;
	return ret_Status;
}

int IOA::SearchBoundaryAngle(float &fStartAngle,
							float &fEndAngle,
							vector<int> vctSeg,int nIdxValue,float fStandarAngle)
{
	int i;
	int nStartIdx=0,nEndIdx=0,nMidIdx=0;

	printf("SearchBoundaryAngle: %f, %f , %d  ,%f  \n ",
			fStartAngle,fEndAngle,nIdxValue,fStandarAngle);
	vctSeg[0]=0;
	vctSeg[vctSeg.size()-1]=0;
	for(i=0;i<vctSeg.size();i++)
	{
		//printf("vctSeg:%d  \n",vctSeg[i]);
		if(vctSeg[i]==nIdxValue)
		{
			nStartIdx=i;
			while(vctSeg[i]==nIdxValue)
			{
				i++;
			}
			nEndIdx=i;
			break;
		}
	}

	printf("SearchBoundaryAngle:  %d  ,%d  \n",nStartIdx,nEndIdx);

	nMidIdx=(nStartIdx+nEndIdx)/2;

	fStartAngle=0.0087266461*(nStartIdx-nMidIdx)+fStandarAngle;
	fEndAngle=0.0087266461*(nEndIdx-nMidIdx)+fStandarAngle;


	return 0;
}
float IOA::BestTurnAngle(float fStartAngle,float fEndAngle,
		float fWinAngle,float fWinSize,float fGoalDirct)
{
	float fRtnAngle;
	fRtnAngle=fWinAngle;

	float fLeft,fRight,fStep=0.087266461,fTmpAngle;
	float fTmp1,fTmp2,fTmp3;

	printf("BestTurnAngle1: %f , %f , %f , %f , %f\n",
			fStartAngle,fEndAngle,fWinAngle,fWinSize,fGoalDirct);

	fLeft=m_stIOAParams.g_robot_len/3;
	fRight=m_stIOAParams.g_robot_len/3*2;


	printf("BestTurnAngle2: %f , %f\n",
			fLeft,fRight);

	fTmpAngle=fWinAngle;
	fTmp3=fEndAngle-fStartAngle;
	if(fGoalDirct<fWinAngle)
	{

		while(fTmpAngle>fGoalDirct)
		{
			fTmp1=fTmpAngle-fStartAngle;
			fTmp2=fEndAngle-fTmpAngle;

			fTmp1=fTmp1/fTmp3*fWinSize;
			fTmp2=fTmp2/fTmp3*fWinSize;

			printf("BestTurnAngle3: %f , %f  ,%f  ,%f  \n",
								fTmp1,fTmp2,fTmpAngle,fGoalDirct);
			fRtnAngle=fTmpAngle;

			if(fTmp1>fLeft&&fTmp2>fRight)
			{
				fTmpAngle-=fStep;
			}
			else
			{
				fRtnAngle=fTmpAngle;
				break;
			}

		}
	}
	else
	{
		while(fTmpAngle<fGoalDirct)
		{
			fTmp1=fTmpAngle-fStartAngle;
			fTmp2=fEndAngle-fTmpAngle;

			fTmp1=fTmp1/fTmp3*fWinSize;
			fTmp2=fTmp2/fTmp3*fWinSize;

			printf("BestTurnAngle4: %f , %f  ,%f  ,%f  \n",
					fTmp1,fTmp2,fTmpAngle,fGoalDirct);

			fTmpAngle+=fStep;
			if(fTmp1>fLeft&&fTmp2>fRight)
			{
				fTmpAngle+=fStep;
			}
			else
			{
				fRtnAngle=fTmpAngle;
				break;
			}


		}
	}

	printf("BestTurnAngle5:  %f  \n",fRtnAngle);
	return fRtnAngle;
}

float IOA::NormalAngle(float fAngle)
{
	float fTmp=fabs(fAngle);
	float fTmp2=fTmp/3.1415926;
	float fRslt;

	while(fTmp2>1)
	{
		fTmp2=fTmp/3.1415926;
		if(fTmp2<1)
		{
			break;
		}
		else
		{
			fTmp-=3.1415926;
		}
	}

	if(fAngle>=0)
	{
		fRslt= fTmp2*3.1415926;
	}
	else
	{
		fRslt= -fTmp2*3.1415926;
	}
	return fRslt;
}
