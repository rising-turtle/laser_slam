#ifndef  C_C_H_
#define  C_C_H_
#pragma once

#include "pthread.h"
#include "MainCtrl_Define.h"
#include "NetPortal/NetPortal.h"
#include "Task/IOA.h"
//#include "SLAM/SLAM.h"
#include "SubCtrlCom/SubCtrlCom.h"
#include "ParseXML.h"
#include "Task/Task.h"
#include "LogFile/LogFile.h"
#include "GPS/CGPS.h"
#include "BN/BN.h"
#include "MapService/LocalMapBuilder.h"
#include "MapService/AmbientGridMap.h"
#include <signal.h>
#include "slam_v1.h"
#include "CSICK.h"
#include <stdint.h>
class CSlamV1;


extern float m_fSLAMParams[15];
extern bool _EXTERN_SLAM_PARAM;

class C_C
{
public:
	C_C(void);
	~C_C(void);

	int C_CInit();
	int C_CRun(int argc, char* argv[]);
	int C_CStop();
	int C_CUninit();

	NetPortal m_CNetPortal;
	SubCtrlCom m_CSubCtrlCom;
//	SLAM m_CSLAMTest;


	
	static CSlamV1* m_pCSLAM;

	//guoliang
	CSICK m_DuoSick;



	IOA m_CIOA;
	Task m_CTask;
	LogFile m_CLogFile;
	ParseXML m_CParseXML;
	CGPS m_CGPS;
	BN m_CBN;
	LocalMapBuilder m_CLocalMapBuilder;
	AmbientGridMap m_CAmbientGridMap;


	//static int m_nSysErrList;
	//SubCtrlCom m_CSubCtrlCom;

	static int ExceptionMsg();
	static int GetSerialPortNum(char* pcKey,char *pcPortNum);

	static int ReadOdometry(float *pcData);
	static int ErrList(int nErrCode);
	static float m_fOdometryData[4];
	static float m_fVoltBatteryCtrl;
	static float m_fVoltBatteryPower;
	static float m_fGYRO_Global;
	static float m_fSICK_IO_Read;

	static int m_nSysErrList;

	static float m_fTaskParams[14];

private:
	char *m_pcLogFilePath;


	int LoadConfig(char *pcConfigPath);
	char m_cNetPortalParams[80];
	int ParseIP(char *pcData,char *pcIP);

	static float m_fScanData[541];


	pthread_t m_hThreadSLAM;
	pthread_t m_hThreadNetPortal;
	pthread_t m_hThreadIOA;
	pthread_t m_hThreadSubCtrlCom;
	pthread_t m_hThreadLogFile;
	pthread_t m_hThreadGPS;
	pthread_t m_hThreadBN;
	pthread_t m_hReadRobotStatus;
	pthread_t m_hThreadMainSick;
	pthread_t m_hThreadMinorSick;
	pthread_t m_hThreadSystemMonitor;
	pthread_t m_hThreadUploadPointCloud;
	pthread_t m_hThreadAskRobotStatus;
	pthread_t m_hThreadNetMonitor;
	pthread_t m_hThreadGetScanData;
	pthread_t m_hThreadUploadDF_PC;

	static void* ThreadSLAM(void* lpParam);
	static void* ThreadNetPortal(void* lpParam);
	static void* ThreadIOA(void* lpParam);
	static void* ThreadSubCtrlCom(void* lpParam);
	static void* ThreadLogFile(void* lpParam);
	static void* ThreadGPS(void* lpParam);
	static void* ThreadBN(void* lpParam);
	static void* ThreadReadRobotStatus(void* lpParam);
	static void* ThreadMainSick(void* lpParam);
	static void* ThreadMinorSick(void* lpParam);

	static void *ThreadSystemMonitor(void* lpParam);
	static void *ThreadUploadPointCloud(void* lpParam);
	static void *ThreadNetMonitor(void* lpParam);

	static void *ThreadAskRobotStatus(void* lpParam);
	static void *ThreadGetScanData(void* lpParam);
	static void *ThreadUploadDF_PC(void* lpParam);

	unsigned char m_ucRobotID;

	SLAM_CallBack m_stSLAM_CallBack;

	static float m_fCurPos[3];

	//zhanhe
	static int cbSLAM(float x,float y,float th);
//	static int cbSICK(vector<float>& bearing);
	static int cbOnlySLAMRslt(float x,float y,float th);
	static int cbOnlyOdoRslt(float x,float y,float th);
	static int cbOnlyBNRslt(float x,float y,float th);
	static int cbCtrlCmdParse(float x,float y,float th);
	static int cbDataFusionAndPC(float* b, int n, double px, double py, double pth);

	static int readSickForOD(void* lpParam);


	static C_C *m_pCC_CThis;

	static int ParseMsg(char *pcData,float *pfValue,int &nID);

	static vector<float>  m_vctPC;

	static float m_fOffset[3];

	static int m_nTrustData;
	static int m_nRefreshPCOnRS;

	static int SendLocation2RS();



	static float m_fLocationData[12];

};
#endif
