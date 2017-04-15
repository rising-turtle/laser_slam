#ifndef SLAM_H_
#define SLAM_H_

#pragma once
#include <vector>
using namespace std;


typedef int (*CallBack_Odometry)(float *pfOdometry);
typedef int (*CallBack_BNLocation)(float *pfLocation);


typedef int (*CallBack_SICK)(vector<float> vctSICKData);
typedef int (*CallBack_DataFusionResult)(float fX,float fY,float fTheta);
typedef int (*CallBack_LocalMap)(float *pfLocalMapData);
typedef int (*CallBack_GlobalMap)(float *pfGlobalMapData);


typedef struct SLAM_CallBack
{
	CallBack_Odometry cbOdometry;
	CallBack_BNLocation cbBNLocation;
	CallBack_SICK cbSICKA;
	CallBack_SICK cbSICKB;
	CallBack_DataFusionResult cbDataFusionResult;
	CallBack_LocalMap cbLocalMap;
	CallBack_GlobalMap cbGlobalMap;
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
};
#endif
