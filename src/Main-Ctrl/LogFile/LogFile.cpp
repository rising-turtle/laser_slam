#include "LogFile.h"
#include "string.h"
#include <time.h> 
#include <stdio.h> 
//#include "afxmt.h"
//CCriticalSection g_Log_Critical_section;

pthread_mutex_t g_Mutex_LogFile;

LogFile *LogFile::m_pCLogFile;

LogFile::LogFile(void)
{
}

LogFile::~LogFile(void)
{
}


int LogFile::LogFileInit(char *pcLogFilePath)
{
	//m_LogFile.open(pcLogFilePath,ios::app);
	int i;

	if (strlen(pcLogFilePath)>=1024)
	{
		return -1;
	}
	m_pcLogFileName=new char[1024];
	//m_pcLogFileName="1.txt";
	memset(m_pcLogFileName,0,1024);

	memcpy(m_pcLogFileName,pcLogFilePath,strlen(pcLogFilePath));
	m_pcReserveSpace=new char[1024*1024];
	
	m_pCLogFile=this;

	m_pcDataTimehead="[Data_Time: ";
	m_pcModuleHead="[Module: ";
	m_pcContentHead="[Content: ";
	m_pcTail="]  ";
	m_pcEndL="\n";

	for (i=0;i<20;i++)
	{
		memset(m_ccModuleTable[i],0,MODULE_NUM);
	}
	
	memcpy(m_ccModuleTable[0],"NetPortal",9);
	memcpy(m_ccModuleTable[1],"SLAM",4);
	memcpy(m_ccModuleTable[2],"IOA",3);
	memcpy(m_ccModuleTable[3],"SubCtrl",7);


	pthread_mutex_init(&g_Mutex_LogFile,0);

	if (m_LogFile==NULL)
	{
		return -1;
	}
	return 0;
}

int LogFile::LogFileRun()
{
	m_bStopLog=false;
	//m_vctLogContent.clear();

	while(!m_bStopLog)
	{
		while(m_vctLogContent.size()!=0)
		{
			m_LogFile.open(m_pcLogFileName,ios::app);
			Write2LogFile(&m_vctLogContent.front());
			m_vctLogContent.erase(m_vctLogContent.begin()+0);
			m_LogFile.close();
		}
		sleep(1);
	}



/*	m_hThreadLogFileRun=CreateThread(
		NULL,0,
		(LPTHREAD_START_ROUTINE)ThreadLogFileRun,
		//&m_stThreadPoolParams,
		this,
		0,0
		);*/
	return 0;
}

/*
UINT LogFile::ThreadLogFileRun(LPVOID lpParam)
{
	LogFile*pcLogFile=(LogFile*)lpParam;

	while(!pcLogFile->m_bStopLog)
	{
		while(pcLogFile->m_vctLogContent.size()!=0)
		{
			pcLogFile->m_LogFile.open(pcLogFile->m_pcLogFileName,ios::app);
			pcLogFile->Write2LogFile(&pcLogFile->m_vctLogContent.front());
			pcLogFile->m_vctLogContent.erase(pcLogFile->m_vctLogContent.begin()+0);
			pcLogFile->m_LogFile.close();
		}
		Sleep(1000);
	}

	return 0;
}
*/

int LogFile::LogFileStop()
{
	return 0;
}

int LogFile::LogFileUninit()
{
	if (m_pcLogFileName!=NULL)
	{
		delete [] m_pcLogFileName;
		m_pcLogFileName=NULL;
	}
	
	return 0;
}

int LogFile::PushContent2LogStack(LogFormat *pstContent)
{
	m_pCLogFile->m_vctLogContent.push_back(*pstContent);
	return 0;
}


int LogFile::PushContent2LogStack(char* pcContent,int nModuleIdx)
{
	if (strlen(pcContent)<MAX_CNT_LEN)
	{

		LogFormat stTest;
		memset(&stTest,0,sizeof(LogFormat));
		time_t t = time(0); 
		strftime( stTest.cDataTime, sizeof(stTest.cDataTime), "%Y/%m/%d %X",localtime(&t) ); 
		memcpy(stTest.cModuleName,m_pCLogFile->m_ccModuleTable[nModuleIdx],strlen(m_pCLogFile->m_ccModuleTable[nModuleIdx]));
		memcpy(stTest.cContent,pcContent,strlen(pcContent));
		//g_Log_Critical_section.Lock();
		pthread_mutex_lock(&g_Mutex_LogFile);
		m_pCLogFile->m_vctLogContent.push_back(stTest);
		//g_Log_Critical_section.Unlock();
		pthread_mutex_unlock(&g_Mutex_LogFile);
		return 0;
	}

	else
		return -1;

	
}


//int LogFile::Write2LogFile(ofstream pcFile,LogFormat *pstContent)
int LogFile::Write2LogFile(LogFormat *pstContent)
{
	m_LogFile.seekp(0,ios::end);
	m_LogFile.write(m_pcDataTimehead,strlen(m_pcDataTimehead));
	m_LogFile.write(pstContent->cDataTime,20);
	m_LogFile.write(m_pcTail,strlen(m_pcTail));

	m_LogFile.write(m_pcModuleHead,strlen(m_pcModuleHead));
	m_LogFile.write(pstContent->cModuleName,strlen(pstContent->cModuleName));
	m_LogFile.write(m_pcTail,strlen(m_pcTail));
	m_LogFile.write(m_pcEndL,strlen(m_pcEndL));

	m_LogFile.write(m_pcContentHead,strlen(m_pcContentHead));
	m_LogFile.write(pstContent->cContent,strlen(pstContent->cContent));
	m_LogFile.write(m_pcTail,strlen(m_pcTail));

	m_LogFile.write(m_pcEndL,strlen(m_pcEndL));
	m_LogFile.write(m_pcEndL,strlen(m_pcEndL));
	return 0;
}

/*
int main()
{
	LogFile m_CLogFile;

	m_CLogFile.LogFileInit();
	m_CLogFile.LogFileRun();
	
m_CLogFile.Test();
	while(1)
	{
		
		
		Sleep(30);
	}
	return 0;
}


UINT LogFile::ThreadTest1(LPVOID lpParam)
{
	LogFile*pcLogFile=(LogFile*)lpParam;
	while(1)
	{
		pcLogFile->PushContent2LogStack("I wana this11111111111111111",0);
		Sleep(10);
	}
	
	return 0;
}
UINT LogFile::ThreadTest2(LPVOID lpParam)
{
	LogFile*pcLogFile=(LogFile*)lpParam;
	while(1)
	{
		pcLogFile->PushContent2LogStack("I wana this22222222",0);
		Sleep(20);
	}

	return 0;
}
UINT LogFile::ThreadTest3(LPVOID lpParam)
{
	LogFile*pcLogFile=(LogFile*)lpParam;
	while(1)
	{
		pcLogFile->PushContent2LogStack("I wana this3333333",0);
		Sleep(10);
	}
	return 0;
}*/
