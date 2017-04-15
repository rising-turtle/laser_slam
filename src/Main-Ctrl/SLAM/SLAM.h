#ifndef SLAM_H_
#define SLAM_H_

#pragma once
#include <vector>
#include "../MainCtrl_Define.h"
using namespace std;

#define uint64_t unsigned long long int


typedef int (*CallBack_Odometry)(float *pfOdometry);
typedef int (*CallBack_BNLocation)(float *pfLocation);
typedef int (*CallBack_MainSICK)(float*, uint64_t&);
typedef int (*CallBack_MinorSICK)(float*, uint64_t&);

typedef int (*CallBack_SICK)(vector<float> vctSICKData);
typedef int (*CallBack_DataFusionResult)(float fX,float fY,float fTheta);
typedef int (*CallBack_LocalMap)(float *pfLocalMapData);
typedef int (*CallBack_GlobalMap)(float *pfGlobalMapData);
typedef int (*CallBack_ErrList)(int nErrCode);


typedef int (*CallBack_OnlySLAMResult)(float fX,float fY,float fTheta);
typedef int (*CallBack_OnlyOdoResult)(float fX,float fY,float fTheta);
typedef int (*CallBack_OnlyBNResult)(float fX,float fY,float fTheta);

typedef int (*CallBack_DataFusionAndPC)(float* b, int n, double px, double py, double pth);
typedef struct SLAM_CallBack
{
/*	CallBack_Odometry cbOdometry;
	CallBack_BNLocation cbBNLocation;
	CallBack_SICK cbSICKA;
	CallBack_SICK cbSICKB;
	CallBack_DataFusionResult cbDataFusionResult;
	CallBack_LocalMap cbLocalMap;
	CallBack_GlobalMap cbGlobalMap;
	CallBack_ErrList cbErrList;

	CallBack_OnlySLAMResult cbOnlySLAMResult;
	CallBack_OnlyOdoResult cbOnlyOdoResult;
	CallBack_OnlyBNResult cbOnlyBNResult;*/

	CallBack_Odometry cbOdometry;
	CallBack_BNLocation cbBNLocation;
	CallBack_MainSICK cbMainSICK;
	CallBack_MinorSICK cbMinorSICK;
	CallBack_SICK cbSICKA;
	CallBack_SICK cbSICKB;
	CallBack_DataFusionResult cbDataFusionResult;
	CallBack_LocalMap cbLocalMap;
	CallBack_GlobalMap cbGlobalMap;
	CallBack_ErrList cbErrList;
	CallBack_OnlySLAMResult cbOnlySLAMResult;
	CallBack_OnlyOdoResult cbOnlyOdoResult;
	CallBack_OnlyBNResult cbOnlyBNResult;
	CallBack_DataFusionAndPC cbDataFusionAndPC;
} SLAM_CallBack;


typedef struct SLAMParams
{
	char cHost_IP_A[16];
	int nHost_Port_A;
	char cSICK_IP_A[16];
	int nSICK_Port_A;

	char cHost_IP_B[16];
	int nHost_Port_B;
	char cSICK_IP_B[16];
	int nSICK_Port_B;
}SLAMParams;


class SLAM
{
	public:
		SLAM();
		~SLAM();
		
		int Init(SLAM_CallBack stCallBackSet,SLAMParams stParams);
		int Run();
		int Stop();
		int Uninit();	

	CallBack_NetUpload m_cbNetUpload;
	SLAM_CallBack m_stSLAM_CallBack;
};
#endif
