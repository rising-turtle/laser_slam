#pragma once
#include "../MainCtrl_Define.h"
#include "_2DMap.h"

#include <pthread.h>
#include <vector>

using namespace std;

#define IOA_FREEAREA 255
#define IOA_OBSTACLE 100
#define IOA_VIEWWIDTH 20
#define IOA_VIEWHEIGHT 50
#define IOA_Y_RANGE 10 //10m
class PathPlanning
{
public:
	PathPlanning(void);
	~PathPlanning(void);

	CallBack_LogFile m_cbLogFile;
	_2DMap m_C2DMap;
	
	int DodgePath(Pos_f *pstDest,vector<Point_f> &vctPath);
	int InstantView(IOA_Pos *pstSICKView_A,IOA_Pos *pstSICKView_B);

	int SeedGrowing(int nStartPosIdx,unsigned char *pucLocalMap);
	int MergeGridsBasedOnRobotSize(int nRobotSize,unsigned char *pucLocalMap);
	int MileStoneSlct(unsigned char *pucInstantView,int nArrayLen,vector<Point_f> &vctPath);
	int Back2OriPath();

	int GetCurPose(float *pfPos);
	int GetPosBeforeIOA(float *pfPos);
	int GetMileStonePos(float *pfPos);

	int Inverse(float *pfMat,float *pfInv);
	int MulMat(float *pfMatA,float *pfMatB,float *pfMatC);
	int MulVec(float *pfMat,float *pfVec,float *pfVecRslt);
	int GetTransMat(float *pfOriPos,float *pfCurPos,float *pfMat);


	int CallNewPath();



	unsigned char *m_pucInstantView;
	float m_fPosBefIOA[3];
	float m_fDest[3];
};
