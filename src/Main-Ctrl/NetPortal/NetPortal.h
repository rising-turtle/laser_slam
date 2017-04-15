#pragma once
#include "../MainCtrl_Define.h"
#include "ClientNet.h"
#include "LaserCom.h"





class NetPortal
{
public:
	NetPortal(void);
	~NetPortal(void);

	
	int NetPortalInit(char *pcNetConfig);
	int NetPortalRun();
	int NetPortalStop(char *pcNetConfig);

	
	static int UploadA(char *pcData,int nDataLen);
	static int UploadB(char *pcData,int nDataLen);
	static int UploadMapBuilder(char *pcData,int nDataLen);
	CallBack_LaserData m_cbLaserData[LASER_NUM];
	static int ParseNetDataA(char *pcData,int nDataLen);
	static int ParseNetDataB(char *pcData,int nDataLen);
	static int ParseNetDataM(char *pcData,int nDataLen);
	static int ParseNetDataL(char *pcData,int nDataLen);

	static CallBack_TaskIn m_cbTaskIn;
	static CallBack_LogFile m_cbLogFile;

	bool m_bSysClose;
	static char m_cCncStatus[3][2];
	char m_cNetConfig[3][16];
	static pthread_t m_hThreadListen[3];
	static pthread_t m_hThreadReCNC[3];
private:


	typedef struct NP_CNCParams
	{
		NetPortal *pCThis;
		ClientNet *pCClientNet;
		char cID;
	}NP_CNCParams;

	int InitNetByClass(ClientNet *pCClientNet,char *pcNetConfig);


	static ClientNet m_CClientNetA;
	static ClientNet m_CClientNetB;
	static ClientNet m_CClientNetMapBuilder;

	

	//static UINT ThreadListenA(LPVOID lpParam);
	//static UINT ThreadListenB(LPVOID lpParam);


	static LaserCom m_CLaserCom[LASER_NUM];

	//static UINT ThreadLaserCom(LPVOID lpParam);

	pthread_t m_hThreadListenA;
	pthread_t m_hThreadListenB;
	pthread_t m_hThreadLaserCom;
	pthread_t m_hThreadNetMapBuilder;
	pthread_t m_hThreadNetMonitor;
	pthread_t m_hThreadHeartBit;

	static void* ThreadListenA(void* lpParam);
	static void* ThreadListenB(void* lpParam);
	static void* ThreadLaserCom(void* lpParam);
	static void* ThreadNetMapBuilder(void* lpParam);
	static void* ThreadNetMonitor(void* lpParam);
	static void* ThreadHeartBit(void* lpParam);

	static void* ThreadRestartCnc(void* lpParam);
	static void* ThreadListen(void* lpParam);

	static void* ReCNCAll(void* lpParam);

	static pthread_mutex_t m_MutexSendBLock;
	static pthread_mutex_t m_MutexSendALock;
	static pthread_mutex_t m_MutexSendMLock;

	static NetPortal *m_pCNetPortal;



};
