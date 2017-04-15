#pragma once
#include "ClientNet.h"
#include "../MainCtrl_Define.h"

class LaserCom
{
public:
	LaserCom();
	~LaserCom();


	int LaserComInit(char *pcConfig);
	int LaserComRun();
	int LaserComStop();
	int LaserComUninit();

	CallBack_LaserData m_cbLaserData;
	CallBack_LogFile m_cbLogFile;
protected:
private:
	ClientNet m_CClientNet;
	bool m_bStart2Run;
	//static UINT ThreadGetLaserData(LPVOID lpParam);
	pthread_t m_hThreadGetLaserData;
	static void* ThreadGetLaserData(void*  lpParam);
};
