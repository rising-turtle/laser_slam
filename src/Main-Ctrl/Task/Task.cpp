#include "Task.h"
#include <string.h>
#include <stdio.h>
#include <iostream>
#include "pthread.h"

using namespace std;

Task *Task::m_pCTask;
#include <time.h>

float Task::SPEED[4] = {0,MAX_SPEED_L0,MAX_SPEED_L1,MAX_SPEED_L3};
int Task::S_INDEX = 1;


pthread_t Task::m_hThreadIOA;
bool Task::m_bStopThreadIOA;
CallBack_RobotPos Task::m_cbRobotPos;
CallBack_NetUpload Task::m_cbSend2RS_A;
CallBack_NetUpload Task::m_cbSend2RS_B;
CallBack_SendNKJCMD Task::m_cbSendNKJCMD;
CallBack_SendNKJCMD_Rot Task::m_cbSendNKJCMDRot;
pthread_t Task::m_hThreadDrive;
float Task::m_fTurnSpd[2];
float Task::m_fTurnSpdVel[2];
float Task::m_fStep2DestErrTolerance;
bool Task::m_bFreezeRobot;
Task::Task()
{
	m_last_cmd = -1;
	m_pCTask=this;
	m_bFreezeRobot=false;
}

Task::~Task()
{
	m_pCTask=NULL;
}


int Task::Step2Dest(vector<Point_f> vctMileStone)
{
	int i,nCMD;
	float fDestX,fDestY;
	float fCurPos[3];
	char cSendData[8];

	for(i=0;i<vctMileStone.size();i++)
	{
		if(m_bFreezeRobot==true)
		{
			break;
		}
		//fDestX=vctMileStone[i];
		//fDestY=vctMileStone[i+1];
		fDestX=vctMileStone[i].fX;
		fDestY=vctMileStone[i].fY;
		IOA::GetCurPose(fCurPos);
		if(CalDis(fCurPos[0],fCurPos[1],fDestX,fDestY)<m_fStep2DestErrTolerance)
		{
			nCMD=ROB_REACH_MIL;
			memcpy(cSendData,&nCMD,4);
			memcpy(cSendData+4,&i,4);
			m_cbSend2RS_B(cSendData,8);
			continue;

		}
		else
		{
			m_pCTask->RandomRun(fDestX,fDestY);
			nCMD=ROB_REACH_MIL;
			memcpy(cSendData,&nCMD,4);
			memcpy(cSendData+4,&i,4);
			m_cbSend2RS_B(cSendData,8);
		}

	}
	m_pCTask->SpdAdjust(0,0);

	printf("I reach !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

	return 0;
}

int Task::TaskInit(float *pfTaskParams)
{
	memcpy(&IOA::m_stIOAParams,pfTaskParams,sizeof(IOAParams));

/*	printf("1:%f, 2:%f ,3:%f ,4:%f, 5:%f, 6:%f, 7:%f\n",
			IOA::m_stIOAParams.g_robot_len,
			IOA::m_stIOAParams.g_small_angle,
			IOA::m_stIOAParams.g_min_dis,
			IOA::m_stIOAParams.g_start_angle,
			IOA::m_stIOAParams.g_end_angle,
			IOA::m_stIOAParams.g_stop_dis,
			IOA::m_stIOAParams.g_slow_dis);*/

	SPEED[1]=pfTaskParams[7];
	SPEED[2]=pfTaskParams[8];
	SPEED[3]=pfTaskParams[9];

	m_fTurnSpd[0]=pfTaskParams[10];
	m_fTurnSpd[1]=pfTaskParams[11];

	m_fTurnSpdVel[0]=m_fTurnSpd[0]/180*3.1415926*ROBOT_L/2;
	m_fTurnSpdVel[1]=m_fTurnSpd[1]/180*3.1415926*ROBOT_L/2;

	m_fStep2DestErrTolerance=pfTaskParams[13];


//	printf("SPEED[1]:%f  ,SPEED[2]:%f  ,SPEED[3]:%f \n",SPEED[1],SPEED[2],SPEED[3]);
//	printf("m_fTurnSpd[0]:%f  ,m_fTurnSpdVel[0]:%f  ,m_fTurnSpd[1]:%f  ,m_fTurnSpdVel[1]:%f\n",
//			m_fTurnSpd[0],m_fTurnSpdVel[0],m_fTurnSpd[1],m_fTurnSpdVel[1]);
	m_C2DMap._2DMapInit();
	return 0;
}


int Task::TaskRun()
{
	if(pthread_create(&m_hThreadIOA,NULL,ThreadIOA,this)!=0)
	{
		printf("Create ThreadSubCtrlCom Thread Failed!!!!\n");
	}


	usleep(100000);
	return 0;
}

void Task::speedUp(){

	static int nCount=0;

	printf("S_INDEX:%d \n",S_INDEX);


	if(Task::S_INDEX<3){
		SpdAdjust(Task::SPEED[S_INDEX],SPEED[S_INDEX+1]);
		++S_INDEX;
	}

	if(nCount>10)
	{
		printf("speedUp:%d  \n",nCount);
		SpdAdjust(Task::SPEED[S_INDEX],SPEED[S_INDEX]);
		nCount=0;
	}
	else
	{
		nCount++;
	}
}

void Task::slowDown(){
	if(Task::S_INDEX == 0){
		S_INDEX = 1;
		SpdAdjust(SPEED[0],SPEED[S_INDEX]);
	}
	else if(Task::S_INDEX>1){
		SpdAdjust(SPEED[S_INDEX],SPEED[S_INDEX-1]);
		--S_INDEX;
	}
}

void Task::stopTurn(){
	if(S_INDEX != 1)
	{	
		SpdAdjust(SPEED[S_INDEX],SPEED[1]);
		S_INDEX = 1;
	}
}

void Task::stop(){
//	if(S_INDEX != 0)
	{
		SpdAdjust(SPEED[S_INDEX],SPEED[0]);
		S_INDEX = 0;
	}
}
int Task::RandomRun(float fDestX,float fDestY)
{
	float randAngle = 45.*3.141592654/180.;
		float lrandA = randAngle*-1.;
		int ins_count = 10;
		int ncout = 0;
		float fCurPos[3];
		float fDis;
		float fAngleOffset=0;
		static int nStuation1Count;
		while(!m_bFreezeRobot)
		{
			IOA::GetCurPose(fCurPos);
			fDis=CalDis(fCurPos[0]/1000,fCurPos[1]/1000,fDestX,fDestY);

			printf("fDis:%f  \n",fDis);
			if(fDis<m_fStep2DestErrTolerance)
			{
				return 0;
			}

			float nAngle;
			int ret = m_pCTask->m_CIOA.Look4Window3(nAngle,
					IOA::m_stIOAParams.g_start_angle,
					IOA::m_stIOAParams.g_end_angle,
					IOA::m_stIOAParams.g_slow_dis,
					IOA::m_stIOAParams.g_stop_dis,
					5,fDestX,fDestY);


			printf("m_pCTask->m_CIOA.Look4Window3 :%d \n",ret);
			switch(ret)
			{
			case 0: // not change
			    if(m_last_cmd != 0 || ncout >= ins_count)
			    {
			    	ncout = 0;
			    	cout<<"0 : speedUp()"<<endl;
					speedUp(); // speed up
				}else{
					ncout++;
				}
				m_last_cmd = 0;
				nStuation1Count=0;
				break;
			case 1: // stop turn angle
				if(m_last_cmd != 1)
					cout<<"1 : stop and turn angle!"<<endl;
				//if(m_last_cmd != 1 || ncout >= ins_count)

					ncout = 0;
					//stopTurn();
					stop();
					usleep(50000);
					// MySpin(nAngle);
					cout<<" Task.cpp: nAngle: "<<nAngle<<endl;


					if(nStuation1Count>3)
					{
						if(nAngle>0)
						{
							fAngleOffset=1.0*nStuation1Count*2/180*3.1415926;
						}
						else
						{
							fAngleOffset=-1.0*nStuation1Count*2/180*3.1415926;
						}
					}

					if(fabs(nAngle)<5.0/180*3.1415926)
					{
						if(nAngle<0)nAngle=-5.0/180*3.1415926;
						else nAngle=5.0/180*3.1415926;

						nAngle+=fAngleOffset;
						TrunAngle(SPEED[0],nAngle);
					}
					else
					{
						nAngle+=fAngleOffset;
						TrunAngle(SPEED[0],nAngle);
					}
					nStuation1Count++;
					m_last_cmd = 1;
					break;

			case 2: // stop turn random angle
				if(m_last_cmd !=2 )
				{
					ncout = 0;
					cout<<"2 : stop random angle!"<<endl;
					stop();
					usleep(50000);
					///TrunAngle(SPEED[0], randAngle);
					// randAngle *=-1.;
					MySpin(randAngle);
					// usleep(50000);

				}else
				{
					ncout ++;
				}
				m_last_cmd = 2;
				nStuation1Count=0;
				break;
			case 3: // slow turn angle
				if(m_last_cmd != 3 || ncout >= ins_count)
				{
					ncout = 0;
					cout<<"3: slow and turn angle!"<<endl;
					slowDown();

					if(nAngle > randAngle )
					    nAngle = randAngle;
					else if(nAngle < lrandA)
					    nAngle = lrandA;

					TrunAngle(SPEED[S_INDEX],nAngle);
					m_last_cmd = 3;
				}else
				{
					ncout++;
				}
				nStuation1Count=0;
				break;
			case 4: // slow not turn angle
				if(m_last_cmd !=4 || ncout >= ins_count)
				{
					ncout = 0;
					cout<<"4: slow not turn!"<<endl;
					slowDown();
				}else
				{
					ncout++;
				}
				m_last_cmd = 4;
				nStuation1Count=0;
				break;
			case 5:
				cout<<"Task.cpp continue to move, not speed or slow!"<<endl;
				//if(S_INDEX ==0 ){
					speedUp();
			//	}
				nStuation1Count=0;
				break;
			default:
				cout<<"Task.cpp: error happened!"<<endl;
				break;
			}
			usleep(50000);
		}
		return 0;
}
int Task::RandomRun()
{
	float randAngle = 45.*3.141592654/180.;
	float lrandA = randAngle*-1.;
	int ins_count = 10;
	int ncout = 0;
	while(1)
	{
		/*int nAngle;
		nAngle=m_pCTask->m_CIOA.Look4Window();*/
		float nAngle;
		int ret = m_pCTask->m_CIOA.Look4Window2(nAngle,
				IOA::m_stIOAParams.g_start_angle,
				IOA::m_stIOAParams.g_end_angle,
				IOA::m_stIOAParams.g_slow_dis,
				IOA::m_stIOAParams.g_stop_dis,
				5);
		
		switch(ret)
		{
		case 0: // not change
		    if(m_last_cmd != 0 || ncout >= ins_count)
		    {
		    	ncout = 0;
		    	cout<<"0 : speedUp()"<<endl;
				speedUp(); // speed up
			}else{
				ncout++;
			}
			m_last_cmd = 0;
			break;
		case 1: // stop turn angle
			if(m_last_cmd != 1)
				cout<<"1 : stop and turn angle!"<<endl;
			if(m_last_cmd != 1 || ncout >= ins_count)
			{
				ncout = 0;
				//stopTurn();
				stop();
				usleep(50000);
				// MySpin(nAngle);
				cout<<" Task.cpp: nAngle: "<<nAngle<<endl;
				if(nAngle > randAngle ) 
				    nAngle = randAngle;
				else if(nAngle < lrandA)
				    nAngle = lrandA;
				TrunAngle(SPEED[0],nAngle);
				// TrunAngle(SPEED[0],nAngle);
				//MySpin(nAngle);
				m_last_cmd = 1;
			}else{
				ncout ++;
			}
			break;
		case 2: // stop turn random angle
			if(m_last_cmd !=2 )
			{
				ncout = 0;
				cout<<"2 : stop random angle!"<<endl;
				stop();
				usleep(50000);
				///TrunAngle(SPEED[0], randAngle);
				// randAngle *=-1.;
				MySpin(randAngle);
				// usleep(50000);
			
			}else
			{
				ncout ++;
			}
			m_last_cmd = 2;
			break;
		case 3: // slow turn angle
			if(m_last_cmd != 3 || ncout >= ins_count)
			{
				ncout = 0;
				cout<<"3: slow and turn angle!"<<endl;
				slowDown();
	
				if(nAngle > randAngle ) 
				    nAngle = randAngle;
				else if(nAngle < lrandA)
				    nAngle = lrandA;

				TrunAngle(SPEED[S_INDEX],nAngle);
				m_last_cmd = 3;
			}else
			{
				ncout++;
			}
			break;
		case 4: // slow not turn angle
			if(m_last_cmd !=4 || ncout >= ins_count)
			{
				ncout = 0;
				cout<<"4: slow not turn!"<<endl;
				slowDown();
			}else
			{
				ncout++;
			}
			m_last_cmd = 4;
			break;
		case 5:
			cout<<"Task.cpp continue to move, not speed or slow!"<<endl;
			//if(S_INDEX ==0 ){
				speedUp();
			//}
			break;
		default:
			cout<<"Task.cpp: error happened!"<<endl;
			break;
		}
		//if(m_last_cmd != 0)
			// printf("nAngle  :%d  \n",nAngle);
		usleep(20000);
	}
	return 0;
}

int Task::TaskStop()
{
	
	return 0;
}


int Task::TaskUninit()
{
	return 0;
}

void* Task::ThreadIOA(void* lpParam)
{
	/*while(!m_bStopThreadIOA)
	{
		m_pCTask->m_CIOA.CanRunAreaCheck();
		usleep(10000);
	}*/

	while(1)
	{
		m_pCTask->m_CPathPlanning.InstantView(m_pCTask->m_CIOA.m_stSickARange,
		m_pCTask->m_CIOA.m_stSickBRange);
		usleep(20000);
	}


}

/*
void* Task::ThreadOriErr(void* lpParam)
{
	float fAngle,fX,fY;
	float fPos[3],fAngleErr;

	fX=m_stStartPt.fX-m_stEndPt.fX;
	fY=m_stStartPt.fY-m_stEndPt.fY;

	fAngle=atan2(fY-fX);

	while(!m_bEndThisTime)
	{
		m_pCTask->m_CIOA.GetCurPose(fPos);
		fAngleErr=fabs(fPos[2]-fAngle);

		if(fAngleErr>3.1415926/180*5)
		{
			m_bEndThisTime=true;

		}
	}
	return NULL;
}*/
int Task::NewTaskIn(int nTaskType,char *pcData,int nDataLen)
{
	int i,nNum=(nDataLen>>2),nRtn;
	Point_f stPoint;
	vector<float> vctSpdLmt;
	float fPos[3],fDiffX,fDiffY;
	Point_f stPt;
	vector<Point_f> vctSegmentLen;
	vector<Point_f> vctBlendSegmentLen;
	if (nTaskType==1)//new path in
	{
		m_bFreezeRobot=true;
		sleep(2);
		m_pCTask->SpdAdjust(0,0);

		vector<Point_f> vctPath;
		float *pfPoint=new float[nNum];
		memcpy(pfPoint,pcData,nDataLen);
		printf("Path Len %d  \n",nNum);
		m_pCTask->m_vctTaskPath.clear();
		m_bFreezeRobot=false;
		for(i=1;i<nNum;i=i+2)
		{
				stPoint.fX=-(pfPoint[i]/1000-9.8);
				stPoint.fY=-(pfPoint[i-1]/1000-27.1);
				printf("x:  %f ,y:   %f\n",stPoint.fX,stPoint.fY);
				m_pCTask->m_vctTaskPath.push_back(stPoint);
				vctSpdLmt.push_back(MAX_SPD);
		}
		IOA::GetCurPose(fPos);
		fPos[0]=fPos[0]/1000;
		fPos[1]=fPos[1]/1000;

		FaceToMilestone(fPos[0],fPos[1],stPoint.fX,stPoint.fY,fPos[2]);


		if(m_pCTask->Drive(vctSegmentLen,vctBlendSegmentLen,0)==-1)
		{
		}
		delete [] pfPoint;
	}
	else if (nTaskType==10)//Stop 1Robot Slowly
	{
		m_pCTask->m_vctTaskPath.clear();
		printf("Stop Robot Slowly!!!!\n");
		//Send Break CMD to Lower
	}
	else if(nTaskType==16)
	{
		printf("2D Grid Map In.....\n");
		m_pCTask->m_C2DMap.AddNew2DGridMap(pcData);
	}
	else if(nTaskType==RE_TASK_PATH)
	{
		m_bFreezeRobot=true;
		sleep(2);
		m_pCTask->SpdAdjust(0,0);


		float *pfPoint=new float[nNum];
		memcpy(pfPoint,pcData,nDataLen);
		m_pCTask->m_vctTaskPath.clear();
		for(i=1;i<nNum;i=i+2)
		{
			stPoint.fX=-(pfPoint[i]/1000-9.8);
			stPoint.fY=-(pfPoint[i-1]/1000-27.1);
			m_pCTask->m_vctTaskPath.push_back(stPoint);
			vctSpdLmt.push_back(MAX_SPD);
		}
		IOA::GetCurPose(fPos);
		fPos[0]=fPos[0]/1000;
		fPos[1]=fPos[1]/1000;

		FaceToMilestone(fPos[0],fPos[1],stPoint.fX,stPoint.fY,fPos[2]);
		if(m_pCTask->Drive(vctSegmentLen,vctBlendSegmentLen,0)==-1)
		{
		}
		delete [] pfPoint;
	}
}

/*
int Task::MoveLinear()
{
	float fEndSpd,fTmpR,fTmpL,fX,fY,fAngle,fOriErr;
	float fV,fXS=0,fYS=0,fXAcc=0,fYAcc=0,fThetaAcc;
	float fPos[3];
	vector<float> vctVL;
	vector<float> vctVR;
	vctVL.clear();
	vctVR.clear();
	fX=vctSegmentLen[i].fX-vctSegmentLen[i+1].fX;
	fY=vctSegmentLen[i].fY-vctSegmentLen[i+1].fY;
	fAngle=atan2(fY,fX);
	m_pCTask->m_CTrajectory.NewSegmentRectilinear(
				fInitSpd,
				fEndSpd,
				MAX_SPD,
				vctSegmentLen[i],
				vctSegmentLen[i+1],
				vctVL,vctVR);
	stEndPt=vctSegmentLen[i+1];

	for(j=0;j<vctVL.size();j++)
	{
		fTmpR=m_pCTask->m_CTrajectory.m_vctRecvLinearVL[i][m]*1000;
		m_cbSendNKJCMD(fTmpL,fTmpR,nRunTime,nRunTime);
		m_pCTask->m_CIOA.GetCurPose(fPos);
		fV=m_pCTask->m_CTrajectory.m_vctRecvLinearVL[i][m]*CMD_SLICE_LEN;
		fXS=cos(fThetaAcc)*fV;
		fYS=-sin(fThetaAcc)*fV;
		fXAcc+=fYS;
		fYAcc+=fXS;
		m_cbRobotPos(fXAcc,fYAcc,fThetaAcc);

		fOriErr=fabs(fPos[2]-fAngle);
		if(fOriErr>ADJUST_ANGLE_THRS)
		{
			float fAjustX=cos(fPos[2])+fPos[0];
			float fAjustY=sin(fPos[2])+fPos[1];
			Point_f stAdjustPath;
			vector<Point_f> vctAdjustPath;
			vector<float> vctLmtSpd;
			stAdjustPath.fX=fPos[0];
			stAdjustPath.fY=fPos[1];
			vctAdjustPath.push_back(stAdjustPath);
			stAdjustPath.fX=fPos[0];
			stAdjustPath.fY=fPos[1];
			vctAdjustPath.push_back(stAdjustPath);
			stAdjustPath.fX=stEndPt.fX;
			stAdjustPath.fY=stEndPt.fY;
			vctAdjustPath.push_back(stAdjustPath);
			vector<Point_f> vctAdjustSegmentLen;
			vector<Point_f> vctAdjustBlendSegmentLen;
			m_pCTask->m_CTrajectory.GetMileStone(vctAdjustPath,
					vctAdjustSegmentLen,
					vctAdjustBlendSegmentLen);
			m_pCTask->Drive(vctAdjustSegmentLen,vctAdjustBlendSegmentLen,fTmpR);
			j=vctVL.size();
			bLinearFinish=true;
			}
			usleep(nSliceTime);
	}
}*/
float Task::DisBetweenPtAndLine(Point_f stLinePtA,Point_f stLinePtB,Point_f stPt)
{
	float fDis,fK,fB;
	if(stLinePtA.fX==stLinePtB.fX)
	{
		fDis=fabs(stPt.fX-stLinePtA.fX);
	}
	else
	{
		fK=(stLinePtB.fY-stLinePtA.fY)/(stLinePtB.fX-stLinePtA.fX);
		fB=stLinePtA.fY-fK*stLinePtA.fX;

		fDis=fabs(fK*stPt.fX-stPt.fY+fB)/(sqrt(fK*fK+1));
	}
	return fDis;
}



int Task::Drive(vector<Point_f> vctSegmentLen,
			vector<Point_f> vctBlendSegmentLen,
			vector<bool> &vctUsingSLAMData,
			vector<float> &vctTurningSPD,
			int nTurnningIDx,
			float fInitSpd,
			float fFinnalEndSpd,
			int nLayers)
{
	int i,j,k,m;
	float fEndSpd,fTmpR,fTmpL,fX,fY,fAngle,fOriErr;
	float fV,fXS=0,fYS=0,fXAcc=0,fYAcc=0,fThetaAcc;
	float fPos[3];
	vector<float> vctVL;
	vector<float> vctVR;
	int nCMD;
	int nSliceTime=(int)((float)CMD_SLICE_LEN*1000.0+0.5)*1000;
	int nRunTime=(float)CMD_SLICE_LEN*1000.0+0.5;
	Point_f stStartPt,stEndPt,stBlendPoint[3];
	bool bLinearFinish=false,bBlendFinish=false,bStart2Turn=false;
	float fTheta,fXD,fYD,fDis,fDisOri,fDis2,fDisPtLine;

	int nRtn=0;
	bool bFrozen=false;
	int nIOARslt;
	float fTmpV;
	k=0;

	int nLocalTurnningIDx=nTurnningIDx;
	nLayers++;
	bool bUseSLAMDataRunLinear;
	printf("$$$$$$$$$$$$$$$$$$$$$$nSliceTime :%d  \n",nSliceTime);

	for (i=0;i<vctSegmentLen.size()-3;i=i+2)
	{
		if(!bFrozen)nRtn=-1;
		//zhi xian yun dong
		fEndSpd=MAX_TURNNING_SPD;
		bLinearFinish=false;

		printf("Run Linear!!!!!\n");
		while(!bLinearFinish)
		{
			printf("cur layers :%d \n",nLayers);
			printf("original X:%f  ,Y:%f  \n",vctSegmentLen[i].fX,vctSegmentLen[i].fY);
			printf("Dest X:%f  ,Y:%f  \n",vctSegmentLen[i+1].fX,vctSegmentLen[i+1].fY);
			m_pCTask->m_CIOA.GetCurPose(fPos);
			vctVL.clear();
			vctVR.clear();

			fX=vctSegmentLen[i+1].fX-vctSegmentLen[i].fX;
			fY=vctSegmentLen[i+1].fY-vctSegmentLen[i].fY;

			bUseSLAMDataRunLinear=false;
			if(sqrt(fX*fX+fY*fY)>0.5)
			{
				printf("Using SLAM Data to run linear!!!!!\n");
				vctSegmentLen[i].fX=fPos[0]+X_OFFSET;
				vctSegmentLen[i].fY=fPos[1]+Y_OFFSET;
				bUseSLAMDataRunLinear=true;
			}




			printf("CurPos:  X:%f   ,Y:%f \n",vctSegmentLen[i].fX,vctSegmentLen[i].fY);
			fX=vctSegmentLen[i+1].fX-vctSegmentLen[i].fX;
			fY=vctSegmentLen[i+1].fY-vctSegmentLen[i].fY;
			fAngle=atan2(fY,fX);




			fEndSpd=vctTurningSPD[nLocalTurnningIDx];
			m_pCTask->m_CTrajectory.NewSegmentRectilinear(
					fInitSpd,
					fEndSpd,
					MAX_SPD,
					vctSegmentLen[i],
					vctSegmentLen[i+1],
					vctVL,vctVR);
			stStartPt=vctSegmentLen[i];
			stEndPt=vctSegmentLen[i+1];

			fDisOri=sqrt((stStartPt.fX-stEndPt.fX)*(stStartPt.fX-stEndPt.fX)+
					(stStartPt.fY-stEndPt.fY)*(stStartPt.fY-stEndPt.fY));


			printf("vctVL.size():%d \n",vctVL.size());
			for(j=0;j<vctVL.size();j++)
			{
				fTmpR=vctVL[j]*1000;


				m_cbSendNKJCMD(fTmpR,fTmpR,nRunTime*3,nRunTime*3);

				printf("fTmpR:%f  ,nRunTime:%d  \n",fTmpR,nRunTime*3);
				m_pCTask->m_CIOA.GetCurPose(fPos);
			/*	fV=vctVL[j]*CMD_SLICE_LEN;
				fXS=cos(fThetaAcc)*fV;
				fYS=-sin(fThetaAcc)*fV;
				fXAcc+=fYS;
				fYAcc+=fXS;
				m_cbRobotPos(fXAcc,fYAcc,fThetaAcc);*/

#ifdef OPEN_IOA

				nIOARslt=m_pCTask->m_CIOA.CanRunAreaCheck();
				printf("m_pCTask->m_CIOA.CanRunAreaCheck():%d  \n",nIOARslt);
				if(nIOARslt==-10)
				{
					printf("break!!!\n");
					bFrozen=true;
					m_cbSendNKJCMD(0,0,nRunTime*3,nRunTime*3);
					printf("Frozen!!!!!!\n");

					//while(1)
					int nAngle;
					m_cbSendNKJCMDRot(30,30);
					nIOARslt=m_pCTask->m_CIOA.CanRunAreaCheck();
					nAngle=m_pCTask->m_CIOA.Look4Window();
					nAngle-=90;
					float fTmp1=(float)nAngle/180*3.1415926;
					m_pCTask->TrunAngle(fTmpR,fTmp1);
					return -1;
				}
				else if(nIOARslt==-1)
				{
					printf("look 4 window!!!\n");
					int nAngle;
					nAngle=m_pCTask->m_CIOA.Look4Window();
					nAngle-=90;
					float fTmp1=(float)nAngle/180*3.1415926;
					m_pCTask->TrunAngle(fTmpR,fTmp1);

				/*	vector<Point_f> vctIOAPath;
					printf("m_pCTask->m_CPathPlanning.MileStoneSlct  \n");
					m_pCTask->m_CPathPlanning.MileStoneSlct(
							m_pCTask->m_CPathPlanning.m_pucInstantView,
							IOA_VIEWHEIGHT,
							vctIOAPath);
					for(m=0;m<vctIOAPath.size();m++)
					{
						vctIOAPath[m].fX+=fPos[0];
						vctIOAPath[m].fY+=fPos[1];
					}
					vctIOAPath.push_back(stEndPt);

					vector<Point_f> vctIOASegmentLen;
					vector<Point_f> vctIOABlendSegmentLen;
					vector<bool> vctIOAUsingSLAMData;
					vector<float> vctIOATurningSPD;

					m_pCTask->m_CTrajectory.GetMileStone(vctIOAPath,
							vctIOASegmentLen,
							vctIOABlendSegmentLen,
							vctIOAUsingSLAMData,
							vctIOATurningSPD
					);
					fTmpV=fTmpR/1000;
					if(m_pCTask->Drive(vctIOASegmentLen,
							vctIOABlendSegmentLen,
							vctIOAUsingSLAMData,
								vctIOATurningSPD,
										0,
										fTmpV,
										fEndSpd,
							nLayers)==-1)
					{
						printf("Can not Run!!!!\n");
						bFrozen=true;
						return -1;
					}*/



				/*	m_pCTask->m_CTrajectory.GetMileStone(vctIOAPath,
							vctIOASegmentLen,
							vctIOABlendSegmentLen);
					if(m_pCTask->Drive(vctIOASegmentLen,vctIOABlendSegmentLen,fTmpR)==-1)
					{
						bFrozen=true;
						return -1;
					}*/
					j=vctVL.size();
					bLinearFinish=true;
					break;
				}
#endif


#ifdef OPEN_ADJUST
				fOriErr=fabs(fPos[2]-fAngle);


				fXD=fPos[0]-stEndPt.fX;
				fYD=fPos[1]-stEndPt.fY;
				fDis=sqrt(fXD*fXD+fYD*fYD);

				fXD=fPos[0]-stStartPt.fX;
				fYD=fPos[1]-stStartPt.fY;
				fDis2=sqrt(fXD*fXD+fYD*fYD);

				Point_f stTmpPt1;
				stTmpPt1.fX=fPos[0];
				stTmpPt1.fY=fPos[1];

				fDisPtLine=DisBetweenPtAndLine(stStartPt,stEndPt,stTmpPt1);

				printf("stEndPt.fX:%f  ,stEndPt.fY:%f  \n",stEndPt.fX,stEndPt.fY);
						printf("fPos[0]:%f,fPos[1]:%f,fPos[2]:%f\n",fPos[0],fPos[1],fPos[2]);
						printf("fOriErr:%f  ,  fPos[2]:%f  ,fAngle:%f, fDis:%f  \n ",
												fOriErr,fPos[2],fAngle,fDis);


				printf("stStartPt: %f  ,%f  ,stEndPt:%f  ,%f  ,stTmpPt1:%f  ,%f \n",
						stStartPt.fX,stStartPt.fY,stEndPt.fX,stEndPt.fY,stTmpPt1.fX,stTmpPt1.fY);

				printf("fDisOri:  %f, fDis1:%f , fDis2:%f \n",fDisOri,fDis2,fDis);
				printf("fDisPtLine:  %f \n",fDisPtLine);
				if(
						//(fOriErr>ADJUST_ANGLE_THRS&&fDis>ADJUST_Len)
						//||fDis2+fDis>fDisOri*1.2
						//||
						fDisPtLine>PT_LINE_ERR
						&&bUseSLAMDataRunLinear==true
					)
				{
					printf("Angle ajust##############################\n");


					float fAdjustAngle=fPos[2]+3.1415926/2;

					if(fAdjustAngle>3.1415926)
					{
						fAdjustAngle=3.1415926*2-fAdjustAngle;
					}

					float fAjustX=cos(fAdjustAngle)*fDis*0.3+fPos[0];
					float fAjustY=sin(fAdjustAngle)*fDis*0.3+fPos[1];


					Point_f stAdjustPath;
					vector<Point_f> vctAdjustPath;
					vector<float> vctLmtSpd;
					vector<bool> vctAdjustUsingSLAMData;
					vector<float> vctAdjustTurningSPD;
					vctAdjustPath.clear();

					printf("why!!!!!!!!!!!!!!!!!!!!!!!!1");
					stAdjustPath.fX=fPos[0];
					stAdjustPath.fY=fPos[1];
					printf("start pos:%f  ,%f  \n",stAdjustPath.fX,stAdjustPath.fY);
					vctAdjustPath.push_back(stAdjustPath);
					stAdjustPath.fX=fAjustX;
					stAdjustPath.fY=fAjustY;
					printf("mid pos:%f  ,%f  \n",stAdjustPath.fX,stAdjustPath.fY);
					vctAdjustPath.push_back(stAdjustPath);
					stAdjustPath.fX=stEndPt.fX;
					stAdjustPath.fY=stEndPt.fY;
					printf("end pos:%f  ,%f  \n",stAdjustPath.fX,stAdjustPath.fY);
					vctAdjustPath.push_back(stAdjustPath);
					stAdjustPath=vctBlendSegmentLen[k];
					vctAdjustPath.push_back(stAdjustPath);
					printf("Bu  why!!!!!!!!!!!!!!!!!!!!!!!!1");

					vector<Point_f> vctAdjustSegmentLen;
					vector<Point_f> vctAdjustBlendSegmentLen;
					vctAdjustBlendSegmentLen.clear();
					vctAdjustSegmentLen.clear();



					m_pCTask->m_CTrajectory.GetMileStone(vctAdjustPath,
							vctAdjustSegmentLen,
							vctAdjustBlendSegmentLen,
							vctAdjustUsingSLAMData,
							vctAdjustTurningSPD
							);
					fTmpV=fTmpR/1000;
					if(m_pCTask->Drive(vctAdjustSegmentLen,
							vctAdjustBlendSegmentLen,
							vctAdjustUsingSLAMData,
							vctAdjustTurningSPD,
							0,
							fTmpV,
							fEndSpd,
							nLayers)==-1)
					{
						printf("Can not Run!!!!\n");
						bFrozen=true;
						return -1;
					}
					j=vctVL.size();
					bLinearFinish=true;
					break;
				}
#endif

				usleep(nSliceTime);
			}
			bLinearFinish=true;
		}
#ifdef SLAM_LOCATION

		printf("start 2 get milestone by SLAM!!!\n");
		bStart2Turn=false;
		while(!bStart2Turn&&vctUsingSLAMData[nLocalTurnningIDx])
		{
			m_pCTask->m_CIOA.GetCurPose(fPos);
			fPos[0]+=X_OFFSET;
			fPos[1]+=Y_OFFSET;
			fXD=fPos[0]-vctBlendSegmentLen[k].fX;
			fYD=fPos[1]-vctBlendSegmentLen[k].fY;
			fDis=sqrt(fXD*fXD+fYD*fYD);
			m_cbSendNKJCMD(fTmpR,fTmpR,nRunTime*3,nRunTime*3);
			if(fDis<TURN_POS_ERR)
			{
				bStart2Turn=true;
			}
			usleep(nSliceTime);
		}
#endif

		printf("Run Blend!!!!!\n");
		bBlendFinish=false;
		while(!bBlendFinish)
		{
			printf("turn cur layers :%d \n",nLayers);
			int nTurnIdx0,nTurnIdx1,nTurnIdx2;
			printf("start 2 turning !!!\n");
			m_pCTask->m_CIOA.GetCurPose(fPos);
			fInitSpd=fEndSpd;

			nTurnIdx0=k++;
			nTurnIdx1=k++;
			nTurnIdx2=k++;


			if(vctUsingSLAMData[nLocalTurnningIDx])
			{
				stBlendPoint[0].fX=fPos[0]+X_OFFSET;
				stBlendPoint[0].fY=fPos[1]+Y_OFFSET;
				stBlendPoint[1]=vctBlendSegmentLen[nTurnIdx1];
				stBlendPoint[2]=vctBlendSegmentLen[nTurnIdx2];

				printf("I do use SLAM Data%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
			}
			else
			{
				stBlendPoint[0]=vctBlendSegmentLen[nTurnIdx0];
				stBlendPoint[1]=vctBlendSegmentLen[nTurnIdx1];
				stBlendPoint[2]=vctBlendSegmentLen[nTurnIdx2];
				printf("I do not  use SLAM Data@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
			}
			int t;
			for(t=0;t<3;t++)
			{
				printf("Blend :%d , X:  %f,Y:%f  \n  ",t,stBlendPoint[t].fX,stBlendPoint[t].fY);
			}
			vctVL.clear();
			vctVR.clear();
			m_pCTask->m_CTrajectory.NewSegmentBlend(
					stBlendPoint,
					vctVL,
					vctVR,
					fInitSpd);
			for(j=0;j<vctVL.size();j++)
			{
				fTheta=(vctVR[j]-vctVL[j])/ROBOT_L;
				fV=(vctVR[j]+vctVL[j])/2;

				fTmpL=vctVL[j]*1000*CMD_SLICE_HZ;
				fTmpR=vctVR[j]*1000*CMD_SLICE_HZ;

				printf("fTmpL:%f,  fTmpR:%f  ,nRunTime:%d  \n",fTmpL,fTmpR,nRunTime*3);
				m_cbSendNKJCMD(fTmpL,fTmpR,nRunTime*3,nRunTime*3);
#ifdef OPEN_IOA
			/*	if(m_pCTask->m_CIOA.CanRunAreaCheck_Blend()==-1)
				{
					bFrozen=true;
					break;
				}*/
#endif
			//	printf("Turn  VL:%f  ,VR:%f  \n",fTmpL,fTmpR);
				fThetaAcc+=fTheta;
				fXS=cos(fThetaAcc)*fV;
				fYS=-sin(fThetaAcc)*fV;
				fXAcc+=fYS;
				fYAcc+=fXS;
				m_cbRobotPos(fXAcc,fYAcc,fThetaAcc);

				usleep(nSliceTime);
			}
			nLocalTurnningIDx++;
			bBlendFinish=true;
		}
	}

	fEndSpd=fFinnalEndSpd;
	//fInitSpd=fInitSpd;
	m_pCTask->m_CTrajectory.NewSegmentRectilinear(fInitSpd,
			fEndSpd,
			MAX_SPD,
			vctSegmentLen[vctSegmentLen.size()-2],
			vctSegmentLen[vctSegmentLen.size()-1],
			vctVL,vctVR);
	bLinearFinish=false;
	while(!bLinearFinish)
	{
		printf("tail cur layers :%d \n",nLayers);
		vctVL.clear();
		vctVR.clear();


		vctSegmentLen[i].fX=fPos[0]+X_OFFSET;
		vctSegmentLen[i].fY=fPos[1]+Y_OFFSET;

		fX=vctSegmentLen[i+1].fX-vctSegmentLen[i].fX;
		fY=vctSegmentLen[i+1].fY-vctSegmentLen[i].fY;

		printf("DestX:%f, DestY:%f, OriX:%f, OriY:%f \n",vctSegmentLen[i+1].fX,
				vctSegmentLen[i+1].fY,vctSegmentLen[i].fX,vctSegmentLen[i].fY);
		fAngle=atan2(fY,fX);
		m_pCTask->m_CIOA.GetCurPose(fPos);

		m_pCTask->m_CTrajectory.NewSegmentRectilinear(
					fInitSpd,
					fEndSpd,
					MAX_SPD,
					vctSegmentLen[i],
					vctSegmentLen[i+1],
					vctVL,vctVR);


		stEndPt=vctSegmentLen[i+1];
		stStartPt=vctSegmentLen[i];


		for(j=0;j<vctVL.size();j++)
		{
			fTmpR=vctVL[j]*1000;
			m_cbSendNKJCMD(fTmpR,fTmpR,nRunTime*3,nRunTime*3);

			printf("fTmpR:%f  ,nRunTime:%d  \n",fTmpR,nRunTime*3);
			m_pCTask->m_CIOA.GetCurPose(fPos);
			fV=vctVL[j]*CMD_SLICE_LEN;
			fXS=cos(fThetaAcc)*fV;
			fYS=-sin(fThetaAcc)*fV;
			fXAcc+=fYS;
			fYAcc+=fXS;
			m_cbRobotPos(fXAcc,fYAcc,fThetaAcc);

#ifdef OPEN_IOA
			nIOARslt=m_pCTask->m_CIOA.CanRunAreaCheck();
			printf("m_pCTask->m_CIOA.CanRunAreaCheck():%d  \n",nIOARslt);
			if(nIOARslt==-10)
			{
				bFrozen=true;
				m_cbSendNKJCMD(0,0,nRunTime*3,nRunTime*3);
				printf("Frozen!!!!!!\n");
				return -1;
			}
			else if(nIOARslt==-1)
			{

				int nAngle;
				nAngle=m_pCTask->m_CIOA.Look4Window();
				nAngle-=90;
				float fTmp1=(float)nAngle/180*3.1415926;
				m_pCTask->TrunAngle(fTmpR,fTmp1);


				/*vector<Point_f> vctIOAPath;
				m_pCTask->m_CPathPlanning.MileStoneSlct(
						m_pCTask->m_CPathPlanning.m_pucInstantView,
						IOA_VIEWHEIGHT,
						vctIOAPath);
				for(m=0;m<vctIOAPath.size();m++)
				{
					vctIOAPath[m].fX+=fPos[0];
					vctIOAPath[m].fY+=fPos[1];
				}
				vctIOAPath.push_back(stEndPt);

				vector<Point_f> vctIOASegmentLen;
				vector<Point_f> vctIOABlendSegmentLen;
				m_pCTask->m_CTrajectory.GetMileStone(vctIOAPath,
							vctIOASegmentLen,
							vctIOABlendSegmentLen);
				if(m_pCTask->Drive(vctIOASegmentLen,vctIOABlendSegmentLen,fTmpR)==-1)
				{
					bFrozen=true;
					return -1;
				}*/

			/*	vector<Point_f> vctIOAPath;
									m_pCTask->m_CPathPlanning.MileStoneSlct(
											m_pCTask->m_CPathPlanning.m_pucInstantView,
											IOA_VIEWHEIGHT,
											vctIOAPath);
									for(m=0;m<vctIOAPath.size();m++)
									{
										vctIOAPath[m].fX+=fPos[0];
										vctIOAPath[m].fY+=fPos[1];
									}
									vctIOAPath.push_back(stEndPt);

									vector<Point_f> vctIOASegmentLen;
									vector<Point_f> vctIOABlendSegmentLen;
									vector<bool> vctIOAUsingSLAMData;
									vector<float> vctIOATurningSPD;

									m_pCTask->m_CTrajectory.GetMileStone(vctIOAPath,
											vctIOASegmentLen,
											vctIOABlendSegmentLen,
											vctIOAUsingSLAMData,
											vctIOATurningSPD
									);
									fTmpV=fTmpR/1000;
									if(m_pCTask->Drive(vctIOASegmentLen,
											vctIOABlendSegmentLen,
											vctIOAUsingSLAMData,
												vctIOATurningSPD,
														0,
														fTmpV,
														fEndSpd,
											nLayers)==-1)
									{
										printf("Can not Run!!!!\n");
										bFrozen=true;
										return -1;
									}*/
				j=vctVL.size();
				bLinearFinish=true;
				break;
			}
#endif

#ifdef OPEN_ADJUST

			fOriErr=fabs(fPos[2]-fAngle);
			fXD=fPos[0]-stEndPt.fX;
			fYD=fPos[1]-stEndPt.fY;
			fDis=sqrt(fXD*fXD+fYD*fYD);

			fXD=fPos[0]-stStartPt.fX;
			fYD=fPos[1]-stStartPt.fY;
			fDis2=sqrt(fXD*fXD+fYD*fYD);

			Point_f stTmpPt1;
			stTmpPt1.fX=fPos[0];
			stTmpPt1.fY=fPos[1];
			fDisPtLine=DisBetweenPtAndLine(stStartPt,stEndPt,stTmpPt1);

			printf("stEndPt.fX:%f  ,stEndPt.fY:%f  \n",stEndPt.fX,stEndPt.fY);
			printf("fPos[0]:%f,fPos[1]:%f,fPos[2]:%f\n",fPos[0],fPos[1],fPos[2]);
			printf("fOriErr:%f  ,  fPos[2]:%f  ,fAngle:%f, fDis:%f  \n ",
														fOriErr,fPos[2],fAngle,fDis);
			printf("fDisOri:  %f, fDis1:%f , fDis2:%f \n",fDisOri,fDis2,fDis);
			if(
					//(fOriErr>ADJUST_ANGLE_THRS&&fDis>ADJUST_Len)
					//	||fDis2+fDis>fDisOri*1.2
					//	||
						fDisPtLine>PT_LINE_ERR
				)
			{
				printf("Tail  Angle ajust##############################\n");
				//float fAjustX=cos(fPos[2])*fDis*0.3+fPos[0];
				//float fAjustY=sin(fPos[2])*fDis*0.3+fPos[1];

				float fAdjustAngle=fPos[2]+3.1415926/2;

				if(fAdjustAngle>3.1415926)
				{
					fAdjustAngle=3.1415926*2-fAdjustAngle;
				}

				float fAjustX=cos(fAdjustAngle)*fDis*0.3+fPos[0];
				float fAjustY=sin(fAdjustAngle)*fDis*0.3+fPos[1];


				Point_f stAdjustPath;
				vector<Point_f> vctAdjustPath;
				vector<float> vctLmtSpd;
				vector<bool> vctAdjustUsingSLAMData;
				vector<float> vctAdjustTurningSPD;
				stAdjustPath.fX=fPos[0];
				stAdjustPath.fY=fPos[1];
				printf("%%%start pos:%f  ,%f  \n",stAdjustPath.fX,stAdjustPath.fY);
				vctAdjustPath.push_back(stAdjustPath);
				stAdjustPath.fX=fAjustX;
				stAdjustPath.fY=fAjustY;
				printf("%%%mid pos:%f  ,%f  \n",stAdjustPath.fX,stAdjustPath.fY);
				vctAdjustPath.push_back(stAdjustPath);
				stAdjustPath.fX=stEndPt.fX;
				stAdjustPath.fY=stEndPt.fY;
				printf("%%%end pos:%f  ,%f  \n",stAdjustPath.fX,stAdjustPath.fY);
				vctAdjustPath.push_back(stAdjustPath);
				vector<Point_f> vctAdjustSegmentLen;
				vector<Point_f> vctAdjustBlendSegmentLen;


				m_pCTask->m_CTrajectory.GetMileStone(vctAdjustPath,
											vctAdjustSegmentLen,
											vctAdjustBlendSegmentLen,
											vctAdjustUsingSLAMData,
											vctAdjustTurningSPD
											);
				fTmpV=fTmpR/1000;
				if(m_pCTask->Drive(vctAdjustSegmentLen,
											vctAdjustBlendSegmentLen,
											vctAdjustUsingSLAMData,
											vctAdjustTurningSPD,
											0,
											fTmpV,
											0,
											nLayers)==-1)
				{
					printf("Can not Run!!!!\n");
					bFrozen=true;
					return -1;
				}
				j=vctVL.size();

			}
#endif
			usleep(nSliceTime);
		}
		bLinearFinish=true;
	}
}


int Task::Drive(vector<Point_f> vctSegmentLen,vector<Point_f> vctBlendSegmentLen,float fInitSpd)
{
	int i,j,k,m;
	float fEndSpd,fTmpR,fTmpL,fX,fY,fAngle,fOriErr;
	float fV,fXS=0,fYS=0,fXAcc=0,fYAcc=0,fThetaAcc;
	float fPos[3];
	vector<float> vctVL;
	vector<float> vctVR;
	int nCMD;
	int nSliceTime=(int)((float)CMD_SLICE_LEN*1000.0+0.5)*1000;
	int nRunTime=(float)CMD_SLICE_LEN*1000.0+0.5;
	Point_f stEndPt,stBlendPoint[3];
	bool bLinearFinish=false,bBlendFinish=false,bStart2Turn=false;
	float fTheta,fXD,fYD,fDis;

	int nRtn=0;
	bool bFrozen=false;
	int nIOARslt;
	k=0;

	for (i=0;i<vctSegmentLen.size()-3;i=i+2)
	{
		if(!bFrozen)nRtn=-1;
		//zhi xian yun dong
		fEndSpd=MAX_TURNNING_SPD;
		bLinearFinish=false;

		printf("Run Linear!!!!!\n");
		while(!bLinearFinish)
		{
			printf("original X:%f  ,Y:%f  \n",vctSegmentLen[i].fX,vctSegmentLen[i].fY);
			m_pCTask->m_CIOA.GetCurPose(fPos);
			vctVL.clear();
			vctVR.clear();

			vctSegmentLen[i].fX=fPos[0]-0.12;
			vctSegmentLen[i].fY=fPos[1];


			printf("CurPos:  X:%f   ,Y:%f \n",vctSegmentLen[i].fX,vctSegmentLen[i].fY);
			fX=vctSegmentLen[i].fX-vctSegmentLen[i+1].fX;
			fY=vctSegmentLen[i].fY-vctSegmentLen[i+1].fY;
			fAngle=atan2(fY,fX);
			m_pCTask->m_CTrajectory.NewSegmentRectilinear(
					fInitSpd,
					fEndSpd,
					MAX_SPD,
					vctSegmentLen[i],
					vctSegmentLen[i+1],
					vctVL,vctVR);
			stEndPt=vctSegmentLen[i+1];

			for(j=0;j<vctVL.size();j++)
			{
				fTmpR=vctVL[j]*1000;
				m_cbSendNKJCMD(fTmpR,fTmpR,nRunTime*3,nRunTime*3);
				m_pCTask->m_CIOA.GetCurPose(fPos);
				fV=vctVL[j]*CMD_SLICE_LEN;
				fXS=cos(fThetaAcc)*fV;
				fYS=-sin(fThetaAcc)*fV;
				fXAcc+=fYS;
				fYAcc+=fXS;
				m_cbRobotPos(fXAcc,fYAcc,fThetaAcc);

#ifdef OPEN_IOA
				nIOARslt=m_pCTask->m_CIOA.CanRunAreaCheck();
				if(nIOARslt==-10)
				{
					bFrozen=true;
					m_cbSendNKJCMD(0,0,nRunTime*3,nRunTime*3);
					printf("Frozen!!!!!!\n");
					return -1;
				}
				else if(nIOARslt==-1)
				{
					m_pCTask->m_CIOA.Look4Window();

				/*	vector<Point_f> vctIOAPath;

					m_pCTask->m_CPathPlanning.MileStoneSlct(
							m_pCTask->m_CPathPlanning.m_pucInstantView,
							IOA_VIEWHEIGHT,
							vctIOAPath);
					for(m=0;m<vctIOAPath.size();m++)
					{
						vctIOAPath[m].fX+=fPos[0];
						vctIOAPath[m].fY+=fPos[1];
					}

					vctIOAPath.push_back(stEndPt);
					vector<Point_f> vctIOASegmentLen;
					vector<Point_f> vctIOABlendSegmentLen;
					m_pCTask->m_CTrajectory.GetMileStone(vctIOAPath,
							vctIOASegmentLen,
							vctIOABlendSegmentLen);
					if(m_pCTask->Drive(vctIOASegmentLen,vctIOABlendSegmentLen,fTmpR)==-1)
					{
						bFrozen=true;
						return -1;
					}*/
					j=vctVL.size();
					bLinearFinish=true;
					break;
				}
#endif


#ifdef OPEN_ADJUST
				fOriErr=fabs(fPos[2]-fAngle);
				fXD=fPos[0]-stEndPt.fX;
				fYD=fPos[1]-stEndPt.fY;
				fDis=sqrt(fXD*fXD+fYD*fYD);
				if(fOriErr>ADJUST_ANGLE_THRS&&fDis<ADJUST_Len)
				{
					float fAjustX=cos(fPos[2])+fPos[0];
					float fAjustY=sin(fPos[2])+fPos[1];
					Point_f stAdjustPath;
					vector<Point_f> vctAdjustPath;
					vector<float> vctLmtSpd;
					vctAdjustPath.clear();

					stAdjustPath.fX=fPos[0];
					stAdjustPath.fY=fPos[1];
					vctAdjustPath.push_back(stAdjustPath);
					stAdjustPath.fX=fAjustX;
					stAdjustPath.fY=fAjustY;
					vctAdjustPath.push_back(stAdjustPath);
					stAdjustPath.fX=stEndPt.fX;
					stAdjustPath.fY=stEndPt.fY;
					vctAdjustPath.push_back(stAdjustPath);
					stAdjustPath=vctBlendSegmentLen[k];
					vctAdjustPath.push_back(stAdjustPath);


					vector<Point_f> vctAdjustSegmentLen;
					vector<Point_f> vctAdjustBlendSegmentLen;
					vctAdjustBlendSegmentLen.clear();
					vctAdjustSegmentLen.clear();
					m_pCTask->m_CTrajectory.GetMileStone(vctAdjustPath,
							vctAdjustSegmentLen,
							vctAdjustBlendSegmentLen);
					if(m_pCTask->Drive(vctAdjustSegmentLen,vctAdjustBlendSegmentLen,fTmpR)==-1)
					{
						bFrozen=true;
						return -1;
					}
					j=vctVL.size();
					bLinearFinish=true;
					break;
				}
#endif

				usleep(nSliceTime);
			}
			bLinearFinish=true;
		}
#ifdef SLAM_LOCATION

		printf("start 2 get milestone by SLAM!!!\n");
		bStart2Turn=false;
		while(!bStart2Turn)
		{
			printf("start 2 get milestone by SLAM in $$$$$$$$$$$!!!\n");
			m_pCTask->m_CIOA.GetCurPose(fPos);
			printf("fX:  %f ,fY:%f  \n",fPos[0],fPos[1]);

			printf("fX:  %f ,fY:%f  \n",vctBlendSegmentLen[k].fX,vctBlendSegmentLen[k].fY);
			fXD=fPos[0]-vctBlendSegmentLen[k].fX;
			fYD=fPos[1]-vctBlendSegmentLen[k].fY;
			printf("sasasasas!!!!!!!!!!!\n");
			fDis=sqrt(fXD*fXD+fYD*fYD);
			m_cbSendNKJCMD(fTmpR,fTmpR,nRunTime*3,nRunTime*3);
			printf("sasasasas2222222222!!!!!!!!!!!\n");
			if(fDis<TURN_POS_ERR)
			{
				bStart2Turn=true;
			}
			usleep(nSliceTime);
		}
#endif

		printf("Run Blend!!!!!\n");
		bBlendFinish=false;
		while(!bBlendFinish)
		{
			int nTurnIdx0,nTurnIdx1,nTurnIdx2;
			printf("start 2 turning !!!\n");
			m_pCTask->m_CIOA.GetCurPose(fPos);
			fInitSpd=MAX_TURNNING_SPD;
			//stBlendPoint[0]=vctBlendSegmentLen[k++];

			nTurnIdx0=k++;
			nTurnIdx1=k++;
			nTurnIdx2=k++;

			stBlendPoint[0]=vctBlendSegmentLen[nTurnIdx0];
			stBlendPoint[1]=vctBlendSegmentLen[nTurnIdx1];

			printf(" 1:%d,  x:%f  ,y:%f  \n",nTurnIdx1,stBlendPoint[1].fX,stBlendPoint[1].fY);
			stBlendPoint[2]=vctBlendSegmentLen[nTurnIdx2];
			printf(" 2:%d,  x:%f  ,y:%f  \n",nTurnIdx2,stBlendPoint[2].fX,stBlendPoint[2].fY);
			printf("original blend :X:%f  ,Y:%f \n",
					vctBlendSegmentLen[nTurnIdx0].fX,
					vctBlendSegmentLen[nTurnIdx0].fY);

			fXD=fPos[0]-stBlendPoint[1].fX;
			fYD=fPos[1]-stBlendPoint[1].fY;
			fDis=sqrt(fXD*fXD+fYD*fYD);

			/*if(fDis<0.2)
			{
				stBlendPoint[0]=vctBlendSegmentLen[nTurnIdx0];
			}
			else
			{
				stBlendPoint[0].fX=fPos[0];
				stBlendPoint[0].fY=fPos[1];
			}*/



			int t;
			for(t=0;t<3;t++)
			{
				printf("Blend :%d , X:  %f,Y:%f  \n  ",t,stBlendPoint[t].fX,stBlendPoint[t].fY);
			}
			vctVL.clear();
			vctVR.clear();
			m_pCTask->m_CTrajectory.NewSegmentBlend(stBlendPoint,vctVL,vctVR,fInitSpd);

			printf("vctVL.size():%d  \n",vctVL.size());
			for(j=0;j<vctVL.size();j++)
			{
				fTheta=(vctVR[j]-vctVL[j])/ROBOT_L;
				fV=(vctVR[j]+vctVL[j])/2;

				fTmpL=vctVL[j]*1000*CMD_SLICE_HZ;
				fTmpR=vctVR[j]*1000*CMD_SLICE_HZ;
				m_cbSendNKJCMD(fTmpL,fTmpR,nRunTime,nRunTime);
/*#ifdef OPEN_IOA
				if(m_pCTask->m_CIOA.CanRunAreaCheck_Blend()==-1)
				{
					bFrozen=true;
					m_cbSendNKJCMD(0,0,nRunTime*3,nRunTime*3);
					printf("Frozen!!!!!!\n");
				}
#endif*/
				printf("Turn  VL:%f  ,VR:%f  \n",fTmpL,fTmpR);
				fThetaAcc+=fTheta;
				fXS=cos(fThetaAcc)*fV;
				fYS=-sin(fThetaAcc)*fV;
				fXAcc+=fYS;
				fYAcc+=fXS;
				m_cbRobotPos(fXAcc,fYAcc,fThetaAcc);

				usleep(nSliceTime);
			}
			bBlendFinish=true;
		}
	}

	fEndSpd=0;
	fInitSpd=fTmpR;



	m_pCTask->m_CTrajectory.NewSegmentRectilinear(fInitSpd,
			fEndSpd,
			MAX_SPD,
			vctSegmentLen[vctSegmentLen.size()-2],
			vctSegmentLen[vctSegmentLen.size()-1],
			vctVL,vctVR);
	bLinearFinish=false;
	while(!bLinearFinish)
	{
		vctVL.clear();
		vctVR.clear();
		vctSegmentLen[i].fX=fPos[0]+X_OFFSET;
		vctSegmentLen[i].fY=fPos[1]+Y_OFFSET;

		fX=vctSegmentLen[i].fX-vctSegmentLen[i+1].fX;
		fY=vctSegmentLen[i].fY-vctSegmentLen[i+1].fY;
		fAngle=atan2(fY,fX);
		m_pCTask->m_CIOA.GetCurPose(fPos);

		m_pCTask->m_CTrajectory.NewSegmentRectilinear(
					fInitSpd,
					fEndSpd,
					MAX_SPD,
					vctSegmentLen[i],
					vctSegmentLen[i+1],
					vctVL,vctVR);
		stEndPt=vctSegmentLen[i+1];

		for(j=0;j<vctVL.size();j++)
		{
			fTmpR=vctVL[j]*1000;
			m_cbSendNKJCMD(fTmpR,fTmpR,nRunTime*3,nRunTime*3);
			m_pCTask->m_CIOA.GetCurPose(fPos);
			fV=vctVL[j]*CMD_SLICE_LEN;
			fXS=cos(fThetaAcc)*fV;
			fYS=-sin(fThetaAcc)*fV;
			fXAcc+=fYS;
			fYAcc+=fXS;
			m_cbRobotPos(fXAcc,fYAcc,fThetaAcc);

#ifdef OPEN_IOA
			nIOARslt=m_pCTask->m_CIOA.CanRunAreaCheck();
			if(nIOARslt==-10)
			{
				bFrozen=true;
				m_cbSendNKJCMD(0,0,nRunTime*3,nRunTime*3);
				printf("Frozen!!!!!!\n");
				return -1;
			}
			/*else if(nIOARslt==-1)
			{
				vector<Point_f> vctIOAPath;
				m_pCTask->m_CPathPlanning.MileStoneSlct(
						m_pCTask->m_CPathPlanning.m_pucInstantView,
						IOA_VIEWHEIGHT,
						vctIOAPath);
				for(m=0;m<vctIOAPath.size();m++)
				{
					vctIOAPath[m].fX+=fPos[0];
					vctIOAPath[m].fY+=fPos[1];
				}
				vctIOAPath.push_back(stEndPt);

				vector<Point_f> vctIOASegmentLen;
				vector<Point_f> vctIOABlendSegmentLen;
				m_pCTask->m_CTrajectory.GetMileStone(vctIOAPath,
							vctIOASegmentLen,
							vctIOABlendSegmentLen);
				if(m_pCTask->Drive(vctIOASegmentLen,vctIOABlendSegmentLen,fTmpR)==-1)
				{
					bFrozen=true;
					return -1;
				}
				j=vctVL.size();
				bLinearFinish=true;
				break;
			}*/
#endif

#ifdef OPEN_ADJUST
			fOriErr=fabs(fPos[2]-fAngle);
			fXD=fPos[0]-stEndPt.fX;
			fYD=fPos[1]-stEndPt.fY;
			fDis=sqrt(fXD*fXD+fYD*fYD);
			if(fOriErr>ADJUST_ANGLE_THRS&&fDis<ADJUST_Len)
			{
				float fAjustX=cos(fPos[2])+fPos[0];
				float fAjustY=sin(fPos[2])+fPos[1];
				Point_f stAdjustPath;
				vector<Point_f> vctAdjustPath;
				vector<float> vctLmtSpd;
				stAdjustPath.fX=fPos[0];
				stAdjustPath.fY=fPos[1];
				vctAdjustPath.push_back(stAdjustPath);
				stAdjustPath.fX=fPos[0];
				stAdjustPath.fY=fPos[1];
				vctAdjustPath.push_back(stAdjustPath);
				stAdjustPath.fX=stEndPt.fX;
				stAdjustPath.fY=stEndPt.fY;
				vctAdjustPath.push_back(stAdjustPath);
				vector<Point_f> vctAdjustSegmentLen;
				vector<Point_f> vctAdjustBlendSegmentLen;
				m_pCTask->m_CTrajectory.GetMileStone(vctAdjustPath,
							vctAdjustSegmentLen,
							vctAdjustBlendSegmentLen);
				if(m_pCTask->Drive(vctAdjustSegmentLen,vctAdjustBlendSegmentLen,fTmpR)==-1)
				{
					bFrozen=true;
					return -1;
				}
				j=vctVL.size();

			}
#endif
			usleep(nSliceTime);
		}
		bLinearFinish=true;
	}
}



int Task::Drive()
{
	int i,j=0,k,m,n;
	float fX,fY,fTheta;
	float fTmpL,fTmpR,fDis;
	int nCMD;
	int nSliceTime=(int)((float)CMD_SLICE_LEN*1000.0+0.5)*1000;
	int nRunTime=(float)CMD_SLICE_LEN*1000.0+0.5;

	time_t timeStart,timeEnd;
	printf("nSliceTime  :%d   ,%d\n",nSliceTime,nRunTime);
	Point_f stPoint1,stPoint2;
	stPoint1=m_pCTask->m_vctTaskPath[0];
	stPoint2=m_pCTask->m_vctTaskPath[1];

	fX=stPoint2.fX-stPoint1.fX;
	fY=stPoint2.fY-stPoint1.fY;
	float fV,fXS=0,fYS=0,fXAcc=0,fYAcc=0,fThetaAcc,fOriErr;
	fThetaAcc=atan2(fY,fX)-3.1415926/2;

	timeStart=clock();
	usleep(nSliceTime);
	timeEnd=clock();
	printf("Test!!!!!! time cost:%d\n",timeEnd-timeStart);

	//printf("fX:%f ,  fY:%f,  fThetaAcc  :  %f  \n",fX,fY,atan2(fY,fX));
	//printf("size():%d \n",m_pCTask->m_CTrajectory.m_vctRecvLinearVL.size());
	char cSendData[1024];
	Point_f stEndPt;
	for(i=0;i<m_pCTask->m_CTrajectory.m_vctRecvLinearVL.size();i++)
	{
	//	printf("m_pCTask->m_CTrajectory.m_vctRecvLinearVL[i].size():  %d",
		//					m_pCTask->m_CTrajectory.m_vctRecvLinearVL[i].size());
		printf("start 2 line!!!!!\n");
		for(m=0;m<m_pCTask->m_CTrajectory.m_vctRecvLinearVL[i].size();m++)
		{	

			//timeStart=clock();
			fTmpL=m_pCTask->m_CTrajectory.m_vctRecvLinearVL[i][m]*1000;
			fTmpR=m_pCTask->m_CTrajectory.m_vctRecvLinearVL[i][m]*1000;
			m_cbSendNKJCMD(fTmpL,fTmpR,nRunTime*3,nRunTime*3);
		//	printf("Turn  VL:%f  ,VR:%f  \n",fTmpL,fTmpR);
			fV=m_pCTask->m_CTrajectory.m_vctRecvLinearVL[i][m]*CMD_SLICE_LEN;
			fXS=cos(fThetaAcc)*fV;
			fYS=-sin(fThetaAcc)*fV;
			fXAcc+=fYS;
			fYAcc+=fXS;
			m_cbRobotPos(fXAcc,fYAcc,fThetaAcc);
			usleep(nSliceTime);
		}

		nCMD=ROB_REACH_MIL;
		memcpy(cSendData,&nCMD,4);
		memcpy(cSendData+4,&i,4);
		m_cbSend2RS_B(cSendData,8);

		printf("start 2 Turn!!!!!\n");

		if(i<m_pCTask->m_CTrajectory.m_vctBlendVL.size())
		{
		/*	bool Jmup=false;
			float fPos[3],fXD,fYD,fDis;
			while(!Jmup)
			{
				m_pCTask->m_CIOA.GetCurPose(fPos);
				fXD=fPos[0]-m_pCTask->m_CTrajectory.m_vctBlendSegmentLen[i].fX;
				fYD=fPos[1]-m_pCTask->m_CTrajectory.m_vctBlendSegmentLen[i].fY;
				fDis=sqrt(fXD*fXD+fYD*fYD);
				if(fDis<0.3)
				{
					Jmup=true;
				}
				m_cbSendNKJCMD(fTmpL,fTmpR,nRunTime,nRunTime);
				usleep(nSliceTime);
			}*/


			for(j=0;j<m_pCTask->m_CTrajectory.m_vctBlendVL[i].size();j++)
			{
				fTheta=(m_pCTask->m_CTrajectory.m_vctBlendVR[i][j]-m_pCTask->m_CTrajectory.m_vctBlendVL[i][j])/ROBOT_L;
				fV=(m_pCTask->m_CTrajectory.m_vctBlendVR[i][j]+m_pCTask->m_CTrajectory.m_vctBlendVL[i][j])/2;

				fTmpL=m_pCTask->m_CTrajectory.m_vctBlendVL[i][j]*1000*CMD_SLICE_HZ;
				fTmpR=m_pCTask->m_CTrajectory.m_vctBlendVR[i][j]*1000*CMD_SLICE_HZ;
				m_cbSendNKJCMD(fTmpL,fTmpR,nRunTime*3,nRunTime*3);
				printf("Turn  VL:%f  ,VR:%f  \n",fTmpL,fTmpR);
				fThetaAcc+=fTheta;
				fXS=cos(fThetaAcc)*fV;
				fYS=-sin(fThetaAcc)*fV;
				fXAcc+=fYS;
				fYAcc+=fXS;
				m_cbRobotPos(fXAcc,fYAcc,fThetaAcc);
				usleep(nSliceTime);
			}
		}
	}
		m_cbSendNKJCMD(fTmpL,fTmpR,0,0);
	return 0;
}

void* Task::ThreadDrive(void* lpParam)
{

	int i;
	printf("start to ThreadDrive  !!!\n");
	pthread_detach(pthread_self());
	vector<Point_f> vctSegmentLen;
	vector<Point_f> vctBlendSegmentLen;
	vector<bool> vctUsingSLAMData;
	vector<float> vctTurningSPD;
//	m_pCTask->m_CTrajectory.NewTrajectory(m_pCTask->m_vctTaskPath,m_pCTask->m_vctSpdLmt,0);
//	m_pCTask->Drive();
/*	m_pCTask->m_CTrajectory.GetMileStone(m_pCTask->m_vctTaskPath,vctSegmentLen,vctBlendSegmentLen);

	for(i=0;i<vctSegmentLen.size();i++)
	{
		printf("vctSegmentLen :%d  ,x:%f,  y:%f \n",i,vctSegmentLen[i].fX,vctSegmentLen[i].fY);
	}

	for(i=0;i<vctBlendSegmentLen.size();i++)
	{
		printf("vctBlendSegmentLen :%d  ,x:%f,  y:%f \n",i,vctBlendSegmentLen[i].fX,vctBlendSegmentLen[i].fY);
	}
	if(m_pCTask->Drive(vctSegmentLen,vctBlendSegmentLen,0)==-1)
	{
				//call for help!!!!\n
	}*/


	//m_pCTask->RandomRun();
	Step2Dest(m_pCTask->m_vctTaskPath);



/*	float fDX,fDY,fAngle;

	fDX=m_pCTask->m_vctTaskPath[0].fX-m_pCTask->m_vctTaskPath[1].fX;
	fDY=m_pCTask->m_vctTaskPath[0].fY-m_pCTask->m_vctTaskPath[1].fY;

	fAngle=atan2(fDY,fDX);

	fAngle=fAngle+3.1415926/2;

	if(fAngle>3.1415926)
	{
		fAngle=3.1415926*2-fAngle;
	}

	fAngle=fAngle/3.1415926*180;

	int nT=int(fabs(fAngle/20)+0.5);
	m_cbSendNKJCMDRot(fAngle,20);

	printf("fAngle:%f  \n",fAngle);
	sleep(3);*/

/*	m_pCTask->m_CTrajectory.GetMileStone(m_pCTask->m_vctTaskPath,
			vctSegmentLen,
			vctBlendSegmentLen,
			vctUsingSLAMData,
			vctTurningSPD);
	if(m_pCTask->Drive(vctSegmentLen,vctBlendSegmentLen,vctUsingSLAMData,vctTurningSPD,0,0,0,0)==-1)
		{
					//call for help!!!!\n
		}*/

}



int Task::TaskIn(int nTaskType,char *pcData,int nDataLen)
{
	int i,nNum=(nDataLen>>2),nRtn;
	Point_f stPoint;
	//vector<float> vctSpdLmt;
	float fPos[3],fDiffX,fDiffY;
	Point_f stPt;
	if (nTaskType==1)//new path in
	{
		float *pfPoint=new float[nNum];
		memcpy(pfPoint,pcData,nDataLen);
		printf("Path Len %d  \n",nNum);
		m_pCTask->m_vctTaskPath.clear();
		for(i=1;i<nNum;i=i+2)
		{
			stPoint.fX=-(pfPoint[i]/1000-9.8);
			stPoint.fY=-(pfPoint[i-1]/1000-27.1);
			printf("x:  %f ,y:   %f\n",stPoint.fX,stPoint.fY);
			m_pCTask->m_vctTaskPath.push_back(stPoint);
			m_pCTask->m_vctSpdLmt.push_back(MAX_SPD);
		}

	/*	if(pthread_create(&m_pCTask->m_hThreadDrive,NULL,ThreadDrive,NULL)!=0)
		{
				printf("Create ThreadSLAM Thread Failed!!!!\n");
		}*/
		delete [] pfPoint;
	}
	else if (nTaskType==10)//Stop 1Robot Slowly
	{
		m_pCTask->m_vctTaskPath.clear();
		printf("Stop Robot Slowly!!!!\n");
		//Send Break CMD to Lower
	}
	else if(nTaskType==16)
	{
		printf("2D Grid Map In.....\n");
		m_pCTask->m_C2DMap.AddNew2DGridMap(pcData);
	}
	else if(nTaskType==RE_TASK_PATH)
	{
		printf("RE_TASK_PATH!!!!!!!!!\n");
		float *pfPoint=new float[nNum];
		memcpy(pfPoint,pcData,nDataLen);
		printf("Path Len %d  \n",nNum);
		m_pCTask->m_vctTaskPath.clear();
		for(i=1;i<nNum;i=i+2)
		{
			stPoint.fX=-(pfPoint[i]/1000-9.8);
			stPoint.fY=-(pfPoint[i-1]/1000-27.1);
			printf("x:  %f ,y:   %f\n",stPoint.fX,stPoint.fY);
			m_pCTask->m_vctTaskPath.push_back(stPoint);
			m_pCTask->m_vctSpdLmt.push_back(MAX_SPD);
		}
		/*RepathPlanning1Step(m_pCTask->m_vctTaskPath[0].fX,m_pCTask->m_vctTaskPath[0].fY,
					m_pCTask->m_vctTaskPath[1].fX,m_pCTask->m_vctTaskPath[1].fY);
		Drive();*/
		delete [] pfPoint;
	}
	return 0;
}
int Task::RepathPlanning1Step(float fSX,float fSY,float fEX,float fEY)
{
	float fPos[3];
	float fX,fY,fTH,fDis,fSpdLmt=1;
	float fDiffX,fDiffY;
	m_pCTask->m_CIOA.GetCurPose(fPos);
	vector<Point_f> vctTmp;
	Point_f stPoint;
	vector<float> vctSpdLmt;
	vctSpdLmt.push_back(fSpdLmt);

	fX=fEX-fPos[0];
	fY=fEY-fPos[1];
	fDis=sqrt(fX*fX+fY*fY);

	if(fDis<1)
	{
		Break(0);
		m_pCTask->m_CIOA.GetCurPose(fPos);
		//FaceToMilestone(fPos[0],fPos[1],fSX,fSY);
		stPoint.fX=fPos[0];
		stPoint.fY=fPos[1];
		vctTmp.push_back(stPoint);
		stPoint.fX=fSX;
		stPoint.fY=fSY;
		vctTmp.push_back(stPoint);
		m_pCTask->m_CTrajectory.NewTrajectory(vctTmp,vctSpdLmt,0);
		Drive();
		//FaceToMilestone(fSX,fSY,fEX,fEY);
		return 1;
	}
	else
	{
		fX=fEX-fSX;
		fY=fEY-fSY;
		fTH=atan2(fY,fX);
		fTH=fPos[2]-fTH;


		if(fabs(fTH)<3.1415926/2)
		{
			Break(0);
			m_pCTask->m_CIOA.GetCurPose(fPos);
			fX=fSX-fPos[0];
			fY=fSY-fPos[1];
		//	FaceToMilestone(fPos[0],fPos[1],fEX,fEY);
			return 2;
		}
		else
		{
			Break(0.5);
			return 3;
		}
	}


	return 0;
}

int Task::FaceToMilestone(float fSX,float fSY,float fEX,float fEY,float fCurFaceDirect)
{

	float fDX,fDY,fCurDirect,fFaceDiff;
	fCurDirect=fCurFaceDirect;
	fDX=fEX-fSX;
	fDY=fSY-fEY;

	printf("fDX:%f  ,fDY:%f  \n",fDX,fDY);
	float fDestDirect=atan2(fDY,fDX);
	if(fDestDirect<0)fDestDirect=fDestDirect+3.1415926*2;

	if(fCurFaceDirect>3.1415926/2&&fCurFaceDirect<=3.1415926)
	{
		fCurFaceDirect=fCurFaceDirect-3.1415926*2;
	}
	else
	{
		fCurFaceDirect+=3.1415926/2;
	}
	if(fCurFaceDirect<0)fCurFaceDirect=fCurFaceDirect+3.1415926*2;
	fFaceDiff=fCurFaceDirect-fDestDirect;

	MySpin_2(fFaceDiff);

	return 0;
}

int Task::Break(float fV)
{
	return 0;
}

float Task::CalDis(float fX1,float fY1,float fX2,float fY2)
{
	float fTmpX=fX2-fX1;
	float fTmpY=fY2-fY1;
	return sqrt(fTmpX*fTmpX+fTmpY*fTmpY);
}

int Task::TrunAngle(float fCurSpd,float fAngle)
{
	printf("fCurSpd:%f  ,fAngle:%f  \n",fCurSpd,fAngle);
	float fW=3.1415926/10;  //30 degree  per second
	float fTmp=fW*ROBOT_L;

	float fWL,fWR;
	bool bJump=false;

	if(fCurSpd<0.1)
	{
	/*	fAngle=fAngle/3.145926*180;
		m_cbSendNKJCMDRot(fAngle,50.0);
		printf("Spin by NKJ!!!!\n");
		float fT=fabs(fAngle)/50.0;
		float fSleepT=fT*1000*1000;
		int nSleep=fSleepT;

		printf("nSleep:%d ,%f  %f \n",nSleep,fT,fSleepT);
		usleep(nSleep);*/

		MySpin_2(fAngle);

		usleep(50000);

		return 0;
	}

	float fTmp2=fTmp/(2*fCurSpd);

	if(fAngle<0)
	{
		fWL=(1+fTmp2)*fCurSpd;
		fWR=(1-fTmp2)*fCurSpd;
	}
	else
	{
		fWL=(1-fTmp2)*fCurSpd;
		fWR=(1+fTmp2)*fCurSpd;
	}

	float fT=fabs(fAngle)/fW;
	float fSleepT=fT*1000*1000;
	int nSleep=fSleepT;
	printf("Turn Angle Alfa :%f ,Sleep Time:%d\n",fTmp2,nSleep);

	m_cbSendNKJCMD(fWL,fWR,MAX_MAINTAIN_TIME,MAX_MAINTAIN_TIME);
	usleep(nSleep);
	m_cbSendNKJCMD(fCurSpd,fCurSpd,MAX_MAINTAIN_TIME,MAX_MAINTAIN_TIME);
	printf("Have Sent Turn Angle Alfa :%f ,Sleep Time:%d\n",fTmp2,nSleep);
	return 0;
}

int Task::SpdAdjust(float finitSpd,float fEndSpd)
{
	m_cbSendNKJCMD(fEndSpd,fEndSpd,MAX_MAINTAIN_TIME,MAX_MAINTAIN_TIME);
	printf("Send CMD 2 NKJ %f   ,  %f  !!!!!!!\n",finitSpd,fEndSpd);
	return 0;
}

int Task::MySpin(float fAngle)
{
	if(fAngle>0)
	m_cbSendNKJCMD(-200,200,MAX_MAINTAIN_TIME,MAX_MAINTAIN_TIME);
	else m_cbSendNKJCMD(200,-200,MAX_MAINTAIN_TIME,MAX_MAINTAIN_TIME);
	return 0;
}

int Task::MySpin_2(float fAngle)
{
	fAngle=fAngle/3.1415926*180;
	printf("MySpin_2:%f  \n",fAngle);
	float fT=fabs(fAngle)/m_fTurnSpd[0];
	float fSleepT=fT*1000*1000*1.2;
	int nSleep=fSleepT;

	m_cbSendNKJCMDRot(fAngle,m_fTurnSpd[0]);
	usleep(nSleep);


	/*float fT=fabs(fAngle)/0.74;
	float fSleepT=fT*1000*1000;
	int nSleep=fSleepT;
	printf("fAngle:%f  ,%d  \n",fAngle,nSleep);
	if(fAngle>0)
	{
		m_cbSendNKJCMD(-200,200,MAX_MAINTAIN_TIME,MAX_MAINTAIN_TIME);
		usleep(nSleep);
	}

	else
	{
		m_cbSendNKJCMD(200,-200,MAX_MAINTAIN_TIME,MAX_MAINTAIN_TIME);
		usleep(nSleep);
	}
	m_cbSendNKJCMD(0,0,MAX_MAINTAIN_TIME,MAX_MAINTAIN_TIME);*/
}

