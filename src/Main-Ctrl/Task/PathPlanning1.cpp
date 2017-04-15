#include "PathPlanning.h"

//#include   <windows.h>  
#include   <stdio.h>  
//#include   <process.h> 
//CallBackSet PathPlanning::m_stCallBackSet;
PathPlanning::PathPlanning()
{

}

PathPlanning::~PathPlanning()
{

}
/*
int PathPlanning::GenerateFlexionByPace_Lock(int nThreadNum,float *pfPath,int nPathLen,char* pcCmd,char * pcCmdIdx)
{
	//Thread lock
	DataCarrier stDataCarrier;
	stDataCarrier.nPathLen=nPathLen;
	stDataCarrier.pfPath=pfPath;
	stDataCarrier.pcCmd=pcCmd;
	stDataCarrier.pcCmdIdx=pcCmdIdx;
	stDataCarrier.nThreadNum=nThreadNum;
	stDataCarrier.bHasRecv=false;
	HANDLE hHandle=CreateThread(
		0,0,
		(LPTHREAD_START_ROUTINE)ThreadPace_Lock,
		&stDataCarrier,
		0,0
		);

	//_beginthreadex(NULL, 0, &ThreadPace_Lock, &stDataCarrier,0,NULL);
	while (1)
	{
		if (stDataCarrier.bHasRecv)
		{
			break;
		}
		Sleep(10);
	}
	return 0;
}

int PathPlanning::ThreadPace_Lock(LPVOID lpParam)
//unsigned int PathPlanning::ThreadPace_Lock(void* lpParam)
{


	int nDataLen;
	DataCarrier stDataCarrier;
	DataCarrier *pstTmp=(DataCarrier *)lpParam;
	memcpy(&stDataCarrier,lpParam,sizeof(DataCarrier));
	pstTmp->bHasRecv=1;

	char *pcDataSpace;
	bool bJump=false;
	while (!bJump)
	{
		if((pcDataSpace=m_stCallBackSet.cbMemoryManager(1))!=NULL)
		{
			bJump=true;
		}

		else if((pcDataSpace=m_stCallBackSet.cbMemoryManager(2))!=NULL)
		{
			bJump=true;
		}

		Sleep(5);
	}

	Pace_Lock CPace_Lock;
	CPace_Lock.Pace_LockRun(stDataCarrier.pfPath,stDataCarrier.nPathLen,pcDataSpace+30,nDataLen);

	pcDataSpace[0]=1;
	memcpy(pcDataSpace+1,&nDataLen,4);
	memcpy(pcDataSpace+18,stDataCarrier.pcCmd,2);
	memcpy(pcDataSpace+20,stDataCarrier.pcCmdIdx,2);
	m_stCallBackSet.cbSendData(stDataCarrier.nThreadNum,pcDataSpace);
	return 0;
}*/
