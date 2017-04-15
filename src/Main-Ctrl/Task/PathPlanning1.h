#pragma once
//#include <Windows.h>
#include "Pace_Lock.h"
//#include "Coordinator_Define.h"

typedef struct DataCarrier 
{
	char* pcCmd;
	char* pcCmdIdx;
	int nPathLen;
	float *pfPath;
	int nThreadNum;
	bool bHasRecv;
}DataCarrier;
class PathPlanning
{
public:
	PathPlanning();
	~PathPlanning();

/*	static CallBackSet m_stCallBackSet;
	int GenerateFlexionByPace_Lock(int nThreadNum,float *pfPath,int nPathLen,char* pcCmd,char * pcCmdIdx);
	static int ThreadPace_Lock(LPVOID lpParam);*/
	//static unsigned int __stdcall ThreadPace_Lock(void* lpParam);
protected:
private:
};
