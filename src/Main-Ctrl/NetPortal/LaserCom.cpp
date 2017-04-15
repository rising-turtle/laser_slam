#include "LaserCom.h"
#include <unistd.h>

LaserCom::LaserCom()
{
	m_bStart2Run=false;
}

LaserCom::~LaserCom()
{

}

int LaserCom::LaserComInit(char *pcConfig)
{
	if(m_CClientNet.ClientNetInit(pcConfig)==RTN_OK)
	{
		m_CClientNet.m_cbNetUpload=m_cbLaserData;

	}
	return RTN_OK;
}

int LaserCom::LaserComRun()
{
	m_bStart2Run=true;

	/*CreateThread(NULL,0,
		(LPTHREAD_START_ROUTINE)ThreadGetLaserData,
		this,
		0,0);*/

	if(pthread_create(&m_hThreadGetLaserData,NULL,ThreadGetLaserData,this)!=0)
	{
		printf("Create ThreadLogFile Thread Failed!!!!\n");
	}
	usleep(100);
	return RTN_OK;
}


int LaserCom::LaserComStop()
{
	m_CClientNet.m_bStopListen=true;
	return RTN_OK;
}


int LaserCom::LaserComUninit()
{
	return RTN_OK;
}


//UINT LaserCom::ThreadGetLaserData(LPVOID lpParam)
void* LaserCom::ThreadGetLaserData(void*  lpParam)
{
	int nRtn;
	LaserCom *pLaserCom=(LaserCom *)lpParam;
	while (!pLaserCom->m_bStart2Run)
	{
		usleep(20);
	}
	
	if(pLaserCom->m_CClientNet.Listen2Host()!=-1)
	{
		nRtn=RTN_OK;
		return &nRtn;
	}	
	else
	{
		//与主机失去联系,马上停止机器人工作
		nRtn=RTN_LOSS_RS_CNC;
		return &nRtn;
	}
	return 0;
}
