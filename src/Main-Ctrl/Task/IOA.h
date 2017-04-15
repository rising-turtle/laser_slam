#pragma once
#include "../MainCtrl_Define.h"
#include "PathPlanning.h"
#include <vector>
#include <math.h>
#include "../SubCtrlCom/IICtrl.h"
#include <unistd.h>
#include "Trajectory.h"

#include <pthread.h>


typedef struct IOAParams
{
	float g_robot_len  ; // the width of the robot
	float g_small_angle ; // 10' small angle not turn
	float g_min_dis ; // minimum dis to make segments
	float g_start_angle ; // serach area from start
	float g_end_angle; // to end
	float g_stop_dis; // stop if found objects in front
	float g_slow_dis; // slow if found objects in front
}IOAParams;


using namespace std;
class IOA
{
public:
	IOA(void);
	~IOA(void);

	int IOAInit();
	int IOARun();
	int IOAStop();
	int IOAUninit();


	SimpleCtrlCmd m_stSimpleCtrlCmd;
	
	static int GetLaserAData(vector<float> vctSICKData);
	static int GetLaserBData(vector<float> vctSICKData);
 	static int DataFusionResult(float fX,float fY,float fTheta);	


 	static int GetCurPose(float *pfPos);

	CallBack_LogFile m_cbLogFile;

	int Step2MileStone(Point_f stPoint,RectilinearCtrl stRectilinearCtrl);
	int Turning();

	float m_fDis2MileStone;

	float m_fGlobalSpeedLimit;
	float m_fLocalSpeedLimit;

	int CanRunAreaCheck();
	int CanRunAreaCheck_Blend();

	static float m_fAngle[SICK_LINES][2];
	static float m_fSecurityAreaRange[SECURITY_AREA_NUM];
	static IOA_Pos m_stSickARange[SICK_LINES];
	static IOA_Pos m_stSickBRange[SICK_LINES];
	static IOA *m_pCIOA;



	Pos_f m_stCurPos;
	bool m_bStopThisStep;
	int Vel2RPM(float fVel);

	float m_fLocalSpeedLimitList[SECURITY_AREA_NUM];

	int Dodge(int nSecurityRange);
	PathPlanning m_CPathPlanning;

	int IOAMileStoneSlct(unsigned char *pucInstantView); 
	
	Trajectory m_CTrajectory;

	float m_fPosBeforeIOA[3];

	int Look4Window();
	
	/* Return:
	*	-1 error 
	*	0 continue
	* 	1 stop -> turn angle
	*	2 stop -> not turn angle
	*	3 slow -> turn angle
	*	4 slow -> not turn angle
	*/
	static unsigned int s_cur_t;
	static unsigned int s_las_t;
	int Look4Window2(float&, int start = 0, int end = 540, \
			int slow_dis = 1.6, int stop_dis = 1.4, \
			int win_range = 5);


	int Look4Window3(float&, int start = 0, int end = 540, \
				int slow_dis = 1.6, int stop_dis = 1.4, \
				int win_range = 5,float fDestX=0,float fDestY=0);
	int segment_Scan(vector<float>&, vector<int>& segs);
	int segment_Scan2(vector<float>&, vector<int>&);
	double segment_Score(vector<float>&, vector<int>&, int, int&, int);
	double segment_Score2(vector<float>&, vector<int>&, int, int&, int);
	double segment_Score3(vector<float>&, vector<int>&, int, int&, int);

	static IOAParams m_stIOAParams;

	int SearchBoundaryAngle(float &fStartAngle,
							float &fEndAngle,
							vector<int> vctSeg,int nIdxValue,float fStandarAngle);

	float BestTurnAngle(float fStartAngle,float fEndAngle,float fWinAngle,float fWinSize,float fGoalDirct);

	float NormalAngle(float fAngle);
};
