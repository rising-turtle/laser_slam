#pragma once
#include "../MainCtrl_Define.h"
#include "Trajectory.h"
#include "IOA.h"
#include <vector>
#include <pthread.h>
#include "_2DMap.h"
#include "PathPlanning.h"

typedef int (*CallBack_RobotPos)(float x,float y,float th);
using namespace std;
class Task
{
public:
	Task();
	~Task();

	int TaskInit(float *pfTaskParams);

	int TaskRun();
	int TaskStop();
	int TaskUninit();

	static int TaskIn(int nTaskType,char *pcData,int nDataLen);
	static int NewTaskIn(int nTaskType,char *pcData,int nDataLen);

	static pthread_t m_hThreadIOA;
	static void* ThreadIOA(void* lpParam);
	static bool m_bStopThreadIOA;


	CallBack_LogFile m_cbLogFile;
	static CallBack_RobotPos m_cbRobotPos;
	static CallBack_NetUpload m_cbSend2RS_A;
	static CallBack_NetUpload m_cbSend2RS_B;

	static CallBack_SendNKJCMD m_cbSendNKJCMD;
	static  CallBack_SendNKJCMD_Rot m_cbSendNKJCMDRot;

	static void* ThreadOriErr(void* lpParam);
	static int CalAdjustPath();

	static float CalDis(float fX1,float fY1,float fX2,float fY2);

	static int MoveLinear();


	//sem_t StartOri_sgl;
	//sem_t AdjustOri_sgl;
	//Point_f m_stEndPt;
	//Point_f m_stStartPt;
	//bool m_bEndThisTime;

	static pthread_t m_hThreadDrive;
	static void* ThreadDrive(void* lpParam);

	static bool bFroze;


	int TrunAngle(float fCurSpd,float fAngle);

	int SpdAdjust(float finitSpd,float fEndSpd);
	
	static float SPEED[4];
	static int S_INDEX;

	static float m_fTurnSpd[2];
	static float m_fTurnSpdVel[2];
	void speedUp(); 
	void slowDown();
	void stopTurn();
	void stop();
	int m_last_cmd ;

	int RandomRun();
	int RandomRun(float fDestX,float fDestY);
protected:
private:
	

//	vector<float> m_vctTaskPath;
	static Task *m_pCTask;

	vector<Point_f> m_vctTaskPath;
	Trajectory m_CTrajectory;

	static int Drive();
	static int Drive(vector<Point_f> vctSegmentLen,vector<Point_f> vctBlendSegmentLen,float fInitSpd);
	static int Drive(vector<Point_f> vctSegmentLen,
			vector<Point_f> vctBlendSegmentLen,
			vector<bool> &vctUsingSLAMData,
			vector<float> &vctTurningSPD,
			int nTurnningIDx,
			float fInitSpd,
			float fEndSpd,
			int nLayers);

	static int FaceToMilestone(float fSX,float fSY,float fEX,float fEY,float fCurFaceDirect);
	static int Break(float fV);
	static int RepathPlanning1Step(float fSX,float fSY,float fEX,float fEY);
	static float DisBetweenPtAndLine(Point_f stLinePtA,Point_f stLinePtB,Point_f stPt);


	static int MySpin(float fAngle);
	static int MySpin_2(float fAngle);

	static int Step2Dest(vector<Point_f> vctMileStone);

	static float m_fStep2DestErrTolerance;

	IOA m_CIOA;

	_2DMap m_C2DMap;
	
	PathPlanning m_CPathPlanning;

	vector<float> m_vctSpdLmt;

	static bool m_bFreezeRobot;
};
