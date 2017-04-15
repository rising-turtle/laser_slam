#pragma once
#include <math.h>
#include <vector>
#include "../MainCtrl_Define.h"
using namespace std;




class Trajectory
{
public:
	Trajectory(void);
	~Trajectory(void);


	//Nonhonolomic Robot Control Test
	int GenTraj();
	int MatMul(float *fA,float *fB,float *fR);
	int MatMulVec(float *fA,float *fB,float *fR);
	int MatPlusVec(float *fA,float *fB,float *fR);
	int MatMinusVec(float *fA,float *fB,float *fR);


	//Polynomial Fitting
	int SegmentBlend(Point_f *stPt,vector<float> &vctWL,vector<float> &vctWR);
	int SegmentRectilinear (float fInitSpd,float fEndSpd,float fLen,vector<float> &vctWL,vector<float> &vctWR);
	int CalWheelVel(vector<Point_f> vctPath);

	float m_fDisMaxAccSpd;
	int DisMaxAccSpd();

	vector<float> m_vctMileStone;

	vector<RectilinearCtrl> m_vctRectilinearCtrl;
	int NewTrajectory(vector<Point_f> vctPath,vector<float> vctSpdLmt,float fInitV);
	int NewSegmentRectilinear(float fInitSpd,float &fEndSpd,float fSpdlmt,Point_f stStartPoint,Point_f stEndPoint,vector<float> &vctVL,vector<float> &vctVR);
	int NewSegmentBlend(Point_f *stPt,vector<float> &vctWL,vector<float> &vctWR,float fInitSpd);

	int GetMileStone(vector<Point_f> vctPath,vector<Point_f> &vctSegmentLen,vector<Point_f> &vctBlendSegmentLen);
	AccTable m_stAccTable[SPD_LMT_NUM];

	int CalMidSpd(float fV0,float fVt,float fAcc,float fDeacc,float fDis,float &fMidSpd,float &fT1,float &fT2);

	vector<vector<float> > m_vctRecvLinearVL;
	vector<vector<float> > m_vctRecvLinearVR;

	vector<vector<float> > m_vctBlendVL;
	vector<vector<float> > m_vctBlendVR;

	vector<Point_f> m_vctSegmentLen;
	vector<Point_f> m_vctBlendSegmentLen;

	int GetMileStone(vector<Point_f> vctPath,vector<Point_f> &vctSegmentLen,vector<Point_f> &vctBlendSegmentLen,
			vector<bool> &vctUsingSLAMData,vector<float> &vctTurningSPD);

	int Spin(float fAngle);
};
