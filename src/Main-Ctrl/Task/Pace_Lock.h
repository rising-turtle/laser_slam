#pragma once
//#define DLL_EXPORT __declspec(dllexport)
//class DLL_EXPORT Pace_Lock
#include <vector>
using namespace std;
class Pace_Lock
{
public:
	Pace_Lock(void);
	~Pace_Lock(void);

	int Pace_LockRun(float *pfPath,int nPathLen,float *pfPaceLock,int &nPaceLockLen);
	int Pace_LockRun(float *pfPath,int nPathLen,char *pcPaceLockData,int &nDataLen);
	int Pace_LockRun_Dest(float *pfPath,int nPathLen,vector<float> &vctPath);
private:
	void bubble_sort(int *x, int n);
	float TwoPointsDis(float x1,float y1,float x2,float y2);
	int GenerateInflectionPoint(float *fPointArray,int nStartIdx,
								int nEndIdx,float fInflexionThres,float fMaxOneStep,
								int *pnInflectionPointsIdxArray,int &nInflectionPointsNum);
	float LookForInflectionPoint(float *fPointArray,int nStartIdx,int nEndIdx,int *pnMaxValIdx,float *pfMaxVal);
};
